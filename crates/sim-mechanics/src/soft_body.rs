//! ソフトボディ(XPBD)。設計: docs/10-mechanics/06-soft-body-particles.md。
//!
//! P3 スコープの最小実装: 距離拘束(設計 §2.2「距離拘束」)のみを持つロープ用途の
//! `SoftBody`。曲げ拘束・体積拘束(布・ゼリー)・剛体/流体との結合・自己衝突は未実装。
//! `MechanicsSolver` とは独立に動作する(設計 §3 の型をそのまま単独クレート内実装とした、
//! `sim_statistical::BrownianParticleSet` と同様のパターン)。

use sim_math::Vec3;

/// 距離拘束(設計 §2.2)。`lambda` は XPBD の累積ラグランジュ乗数(各サブステップ冒頭で0に戻す)。
pub struct DistanceConstraint {
    pub i: usize,
    pub j: usize,
    pub rest: f64,
    /// コンプライアンス $\alpha=1/k$(設計 §2.2)。0 は完全剛(伸びなし)。
    pub compliance: f64,
    lambda: f64,
}

/// 設計 §9 既定値(サブステップ優先: 反復より分割が精度に効く、Macklin et al. 2019)。
pub const DEFAULT_SUBSTEPS: u32 = 4;
pub const DEFAULT_ITERATIONS: u32 = 2;
pub const DEFAULT_DAMPING: f64 = 0.1;

/// 設計 §3 `SoftBody`。粒子集合 + 距離拘束。`inv_mass=0` はピン留め(固定点)。
pub struct SoftBody {
    pub position: Vec<Vec3>,
    pub prev_position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub inv_mass: Vec<f64>,
    pub constraints: Vec<DistanceConstraint>,
}

impl Default for SoftBody {
    fn default() -> Self {
        Self::new()
    }
}

impl SoftBody {
    pub fn new() -> SoftBody {
        SoftBody {
            position: Vec::new(),
            prev_position: Vec::new(),
            velocity: Vec::new(),
            inv_mass: Vec::new(),
            constraints: Vec::new(),
        }
    }

    pub fn add_particle(&mut self, position: Vec3, mass: f64) -> usize {
        let idx = self.position.len();
        self.position.push(position);
        self.prev_position.push(position);
        self.velocity.push(Vec3::ZERO);
        self.inv_mass
            .push(if mass > 0.0 { 1.0 / mass } else { 0.0 });
        idx
    }

    /// 質点をピン留め(固定点)にする。
    pub fn pin(&mut self, idx: usize) {
        self.inv_mass[idx] = 0.0;
    }

    pub fn add_distance_constraint(&mut self, i: usize, j: usize, rest: f64, compliance: f64) {
        self.constraints.push(DistanceConstraint {
            i,
            j,
            rest,
            compliance,
            lambda: 0.0,
        });
    }

    /// 設計 §4 の XPBD 標準ループ。サブステップ `n_sub` × 反復 `n_iter`。
    /// 剛体・地形との衝突/自己衝突は未実装(このスコープでは端点ピン留めのロープのみ扱う)。
    pub fn step(&mut self, dt: f64, gravity: Vec3, n_sub: u32, n_iter: u32, damping: f64) {
        let sub_dt = dt / n_sub as f64;
        for _ in 0..n_sub {
            for i in 0..self.position.len() {
                if self.inv_mass[i] > 0.0 {
                    self.velocity[i] = self.velocity[i].addcarry_scaled(gravity, sub_dt);
                }
                self.prev_position[i] = self.position[i];
                self.position[i] = self.position[i].addcarry_scaled(self.velocity[i], sub_dt);
            }

            for c in &mut self.constraints {
                c.lambda = 0.0;
            }
            for _ in 0..n_iter {
                for c in &mut self.constraints {
                    let (i, j) = (c.i, c.j);
                    let delta = self.position[i] - self.position[j];
                    let len = delta.length();
                    if len < 1e-12 {
                        continue;
                    }
                    let n_dir = delta.scale(1.0 / len);
                    let constraint_val = len - c.rest;
                    let alpha_tilde = c.compliance / (sub_dt * sub_dt);
                    let (wi, wj) = (self.inv_mass[i], self.inv_mass[j]);
                    let denom = wi + wj + alpha_tilde;
                    if denom <= 0.0 {
                        continue;
                    }
                    let delta_lambda = (-constraint_val - alpha_tilde * c.lambda) / denom;
                    c.lambda += delta_lambda;
                    let correction = n_dir.scale(delta_lambda);
                    self.position[i] = self.position[i].addcarry_scaled(correction, wi);
                    self.position[j] = self.position[j].addcarry_scaled(correction, -wj);
                }
            }

            let decay = (-damping * sub_dt).exp();
            for i in 0..self.position.len() {
                self.velocity[i] = (self.position[i] - self.prev_position[i])
                    .scale(1.0 / sub_dt)
                    .scale(decay);
            }
        }
    }
}

/// 直線ロープの生成ヘルパ(設計 §3)。`from`-`to` 間を `segments` 分割し、両端は未ピン留め
/// (呼び出し側で `pin` する)。各粒子の質量は `mass_per_particle`、各拘束のレスト長は
/// `total_rest_length/segments`(`from`-`to` の距離と異なってよい — たるみのあるロープを
/// 表現できる)。
pub fn rope(
    from: Vec3,
    to: Vec3,
    segments: usize,
    mass_per_particle: f64,
    total_rest_length: f64,
    compliance: f64,
) -> SoftBody {
    let mut body = SoftBody::new();
    let rest = total_rest_length / segments as f64;
    for k in 0..=segments {
        let t = k as f64 / segments as f64;
        let pos = from + (to - from).scale(t);
        body.add_particle(pos, mass_per_particle);
    }
    for k in 0..segments {
        body.add_distance_constraint(k, k + 1, rest, compliance);
    }
    body
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 完全懸垂線(カテナリー)$y=a\cosh(x/a)$ のパラメータ `a` を、全長 `length` と
    /// 端点間の水平距離 `span` から二分法で逆算する($length = 2a\sinh(span/(2a))$)。
    fn solve_catenary_a(length: f64, span: f64) -> f64 {
        let f = |a: f64| 2.0 * a * (span / (2.0 * a)).sinh() - length;
        let (mut lo, mut hi) = (span * 1e-3, span * 1000.0);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if f(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// M13: ロープの垂れ — 静止形状がカテナリー $y=a\cosh(x/a)$ と一致すること、
    /// 端点間1m・20分割で最大偏差 < 2%(端点間隔基準、docs/21-verification/01-analytic-tests.md M13)。
    #[test]
    fn m13_hanging_rope_settles_into_catenary_shape() {
        let span = 1.0; // 端点間の水平距離
        let total_length = 1.2; // ロープ自体の長さ(たるみを持たせる)
        let segments = 20;
        let mass_per_particle = 0.01;
        let gravity = Vec3::new(0.0, -9.80665, 0.0);

        let from = Vec3::new(-span / 2.0, 0.0, 0.0);
        let to = Vec3::new(span / 2.0, 0.0, 0.0);
        // ほぼ非伸縮(コンプライアンス≈0)としてカテナリー理論(非伸縮ロープ)と比較できるようにする。
        let mut body = rope(from, to, segments, mass_per_particle, total_length, 1e-10);
        body.pin(0);
        body.pin(segments);

        let dt = 1.0 / 120.0;
        // 十分に減衰させて静止状態に収束させる(設計§4の減衰付きXPBDループ)。
        for _ in 0..2400 {
            body.step(dt, gravity, DEFAULT_SUBSTEPS, DEFAULT_ITERATIONS, 2.0);
        }

        let a = solve_catenary_a(total_length, span);
        // 頂点(x=0)の理論y座標(端点高さ基準の相対値)を求め、シミュレーションのy座標を
        // 同じ基準(端点=0)に合わせて比較する。
        let y_at = |x: f64| a * (x / a).cosh();
        let y_endpoint = y_at(span / 2.0);

        let mut max_dev: f64 = 0.0;
        for k in 0..=segments {
            let x = body.position[k].x;
            let y_theory = y_at(x) - y_endpoint;
            let y_sim = body.position[k].y;
            max_dev = max_dev.max((y_sim - y_theory).abs());
        }
        let rel_dev = max_dev / span;
        assert!(rel_dev < 0.02, "max_dev={max_dev} rel_dev={rel_dev}");
    }

    /// M14: ロープの伸び $\delta=WL_0/(EA)$、rel 5%(docs/21-verification/01-analytic-tests.md M14)。
    /// ロープ自体をほぼ質量ゼロにし、下端に集中荷重(質量 $W/g$)を吊るすことで、
    /// ロープ全長にわたる張力をほぼ一様(=W)にする(理論式が仮定する「質量なしロープ+
    /// 先端荷重」の状況を再現する)。
    #[test]
    fn m14_rope_stretch_under_load_matches_wl_over_ea() {
        let gravity_mag = 9.80665;
        let gravity = Vec3::new(0.0, -gravity_mag, 0.0);
        let l0 = 1.0; // ロープ自然長
        let young_modulus = 1.0e9; // Pa(設計§9 ナイロンロープの桁に近い代表値)
        let area = 1.0e-6; // m²(断面積、径約1.1mm相当)
        let segments = 10;
        let weight_newtons = 50.0;

        let k_rope = young_modulus * area / l0; // ロープ全体の等価剛性(直列ばね則、設計§2.3)
        let expected_stretch = weight_newtons * l0 / (young_modulus * area);

        // 直列に繋いだ segments 個のばねが全体でk_ropeになるよう、
        // 各セグメントの剛性は k_rope*segments(直列ばねの合成則の逆)。
        let compliance_per_segment = 1.0 / (k_rope * segments as f64);

        // ロープ自体の質量(集中荷重に対して無視できる水準)。極端に軽くしすぎると
        // 隣接する質点間の質量比が大きくなりすぎ、少ない反復回数のGauss-Seidel型
        // ソルバでは連鎖が数値的に不安定になる(実装検証中に発見)。
        let negligible_mass = 1.0e-3;
        let load_mass = weight_newtons / gravity_mag;

        let top = Vec3::new(0.0, 0.0, 0.0);
        let bottom = Vec3::new(0.0, -l0, 0.0);
        let mut body = rope(
            top,
            bottom,
            segments,
            negligible_mass,
            l0,
            compliance_per_segment,
        );
        body.pin(0);
        let bottom_idx = segments;
        body.inv_mass[bottom_idx] = 1.0 / load_mass;

        let dt = 1.0 / 240.0;
        // 各セグメントの固有振動周期(sqrt(m/k_seg)のオーダー)が既定のサブステップ幅
        // (dt/DEFAULT_SUBSTEPS)より短く、粗いサブステップでは正しい剛性に収束しない
        // (実装検証中に発見: 既定4サブステップでは伸びが理論値の約5.6倍に収束してしまう)。
        // このテスト固有の高い剛性・軽い質量比に合わせてサブステップ数を増やす。
        let n_sub = 60;
        for _ in 0..2400 {
            body.step(dt, gravity, n_sub, DEFAULT_ITERATIONS, 3.0);
        }

        let current_length: f64 = (0..segments)
            .map(|k| (body.position[k + 1] - body.position[k]).length())
            .sum();
        let measured_stretch = current_length - l0;
        let rel_err = (measured_stretch - expected_stretch).abs() / expected_stretch;
        assert!(
            rel_err < 0.05,
            "measured_stretch={measured_stretch} expected_stretch={expected_stretch} rel_err={rel_err}"
        );
    }
}
