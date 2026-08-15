//! ソフトボディ(XPBD)。設計: docs/10-mechanics/06-soft-body-particles.md。
//!
//! P3 スコープの最小実装: 距離拘束(設計 §2.2「距離拘束」)のみを持つロープ用途の
//! `SoftBody`。**群4で曲げ拘束・体積拘束を実装した**(`BendingConstraint`/
//! `VolumeConstraint`)。剛体/流体との結合・自己衝突は引き続き未実装。
//! `MechanicsSolver` とは独立に動作する(設計 §3 の型をそのまま単独クレート内実装とした、
//! `sim_statistical::BrownianParticleSet` と同様のパターン)。

use sim_math::Vec3;

/// 距離拘束(設計 §2.2)。`lambda` は XPBD の累積ラグランジュ乗数(各サブステップ冒頭で0に戻す)。
#[derive(Clone)]
pub struct DistanceConstraint {
    pub i: usize,
    pub j: usize,
    pub rest: f64,
    /// コンプライアンス $\alpha=1/k$(設計 §2.2)。0 は完全剛(伸びなし)。
    pub compliance: f64,
    lambda: f64,
}

impl DistanceConstraint {
    /// 生値から拘束を1本組み立てる(**生状態スナップショット(`raw_state`)で追加**、
    /// `sim_world::export::to_scenario`のdoc参照)。`lambda`は各サブステップ冒頭で
    /// 0に戻される作業変数なので復元対象ではなく、常に0から始めてよい
    /// (`SoftBody::step`の「`c.lambda = 0.0`」参照)。
    pub fn new(i: usize, j: usize, rest: f64, compliance: f64) -> DistanceConstraint {
        DistanceConstraint {
            i,
            j,
            rest,
            compliance,
            lambda: 0.0,
        }
    }
}

/// **曲げ拘束(設計「曲げ拘束(布・ゼリー)」、群4で追加)**。
///
/// 3粒子 `i`-`j`-`k` が作る角度を保つ。`j` が中央(蝶番)。
/// **距離拘束の組み合わせでは曲げ剛性を表現できない**——ロープや布は
/// 「伸びないが自由に折れ曲がる」性質があり、伸びの硬さと曲げの硬さは独立だからである
/// (実際、増分Hのロープは曲げ剛性ゼロで、カテナリー形状の検証はそれで正しかった。
/// 一方で「腰のある布」「ゼリー」を表現するには曲げ拘束が要る)。
///
/// **距離ベースの簡略形を採る**: 角度そのものではなく `i`-`k` 間の距離を
/// 保つ形にする(XPBD の実装で広く使われる "distance bending")。真の角度拘束
/// (二面角ベース)に比べ、曲げ方向の対称性がわずかに崩れるが、①勾配が距離拘束と
/// 同じ形で書けるため既存のソルバへそのまま乗る ②角度の逆三角関数を通らないので
/// 数値的に素直、という利点がある。**この簡略化を明記して採用する**。
#[derive(Clone, Copy, Debug)]
pub struct BendingConstraint {
    pub i: usize,
    pub j: usize,
    pub k: usize,
    /// `i`-`k` 間の基準距離(生成時の配置から決める)。
    pub rest: f64,
    /// コンプライアンス $\alpha=1/k$。大きいほど柔らかい(0 = 完全剛)。
    pub compliance: f64,
    lambda: f64,
}

impl BendingConstraint {
    /// 生値から曲げ拘束を組み立てる(`DistanceConstraint::new`と同じ理由、
    /// **生状態スナップショット(`raw_state`)で追加**)。`add_bending_constraint`は
    /// `rest`を現在の配置から採ってしまうため、時間発展後のスナップショットを
    /// 復元する用途では使えない(基準長が今の形に書き換わってしまう)。
    pub fn new(i: usize, j: usize, k: usize, rest: f64, compliance: f64) -> BendingConstraint {
        BendingConstraint {
            i,
            j,
            k,
            rest,
            compliance,
            lambda: 0.0,
        }
    }
}

/// **体積拘束(設計「体積拘束(布・ゼリー)」、群4で追加)**。
///
/// 4粒子が作る四面体の符号付き体積を保つ。ゼリーのような「押しても体積が
/// 変わらない」振る舞いの土台。体積は
/// $V=\frac16(\mathbf{p}_1-\mathbf{p}_0)\cdot[(\mathbf{p}_2-\mathbf{p}_0)\times(\mathbf{p}_3-\mathbf{p}_0)]$
/// で、各粒子に対する勾配は閉形式で書ける(下記 `solve` 参照)。
#[derive(Clone, Copy, Debug)]
pub struct VolumeConstraint {
    pub particles: [usize; 4],
    /// 基準体積(生成時の配置から決める、符号付き)。
    pub rest_volume: f64,
    pub compliance: f64,
    lambda: f64,
}

impl VolumeConstraint {
    /// 生値から体積拘束を組み立てる(`BendingConstraint::new`と同じ理由、
    /// **生状態スナップショット(`raw_state`)で追加**)。`add_volume_constraint`は
    /// `rest_volume`を現在の配置から採るため、復元用途では使えない。
    pub fn new(particles: [usize; 4], rest_volume: f64, compliance: f64) -> VolumeConstraint {
        VolumeConstraint {
            particles,
            rest_volume,
            compliance,
            lambda: 0.0,
        }
    }
}

/// 設計 §9 既定値(サブステップ優先: 反復より分割が精度に効く、Macklin et al. 2019)。
pub const DEFAULT_SUBSTEPS: u32 = 4;
pub const DEFAULT_ITERATIONS: u32 = 2;
pub const DEFAULT_DAMPING: f64 = 0.1;

/// 設計 §3 `SoftBody`。粒子集合 + 距離拘束。`inv_mass=0` はピン留め(固定点)。
#[derive(Clone)]
pub struct SoftBody {
    pub position: Vec<Vec3>,
    pub prev_position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub inv_mass: Vec<f64>,
    pub constraints: Vec<DistanceConstraint>,
    /// 曲げ拘束(**群4で追加**、`BendingConstraint`のdoc参照)。
    pub bending_constraints: Vec<BendingConstraint>,
    /// 体積拘束(**群4で追加**、`VolumeConstraint`のdoc参照)。
    pub volume_constraints: Vec<VolumeConstraint>,
    /// **増分Hで追加した自動ステップ用の積分設定**。
    ///
    /// **なぜフィールドとして持つのか**: `SoftBody`は長らく`Solver`を実装しておらず、
    /// `World::step()`が回すドメイン一覧から漏れていた——つまり`enable_soft_body`で
    /// 載せても**シーンを再生しても一切動かなかった**(D13のテストが
    /// `world.soft_body_mut().unwrap().step(...)`と手で回していたのはこのため)。
    /// シーンギャラリーへ出すには自動ステップが要るが、`Solver::step`の引数は
    /// `(dt, &mut SolverContext)`で、`SolverContext`は`materials`/`rng`/`events`
    /// しか持たず**重力もサブステップ数も渡す口が無い**。そこでソルバ自身の設定として
    /// ここに置く。既存の`step(dt, gravity, n_sub, n_iter, damping)`は引数で
    /// 上書きする形のまま残す(呼び出し側の回帰ゼロ)。
    pub gravity: Vec3,
    pub substeps: u32,
    pub iterations: u32,
    pub damping: f64,
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
            bending_constraints: Vec::new(),
            volume_constraints: Vec::new(),
            // 既定はD13(ロープ)のテストが手回しで使っていた値をそのまま採る
            // (damping=2.0 は`DEFAULT_DAMPING`ではない——D13/M13が実際に渡していた値)。
            gravity: Vec3::new(0.0, -9.80665, 0.0),
            substeps: DEFAULT_SUBSTEPS,
            iterations: DEFAULT_ITERATIONS,
            damping: 2.0,
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

    /// 曲げ拘束を追加する(**群4**)。`rest` は現在の `i`-`k` 距離から自動で採る。
    pub fn add_bending_constraint(&mut self, i: usize, j: usize, k: usize, compliance: f64) {
        let rest = (self.position[i] - self.position[k]).length();
        self.bending_constraints.push(BendingConstraint {
            i,
            j,
            k,
            rest,
            compliance,
            lambda: 0.0,
        });
    }

    /// 体積拘束を追加する(**群4**)。`rest_volume` は現在の配置から自動で採る。
    pub fn add_volume_constraint(&mut self, particles: [usize; 4], compliance: f64) {
        let rest_volume = tetrahedron_volume(
            self.position[particles[0]],
            self.position[particles[1]],
            self.position[particles[2]],
            self.position[particles[3]],
        );
        self.volume_constraints.push(VolumeConstraint {
            particles,
            rest_volume,
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
            for c in &mut self.bending_constraints {
                c.lambda = 0.0;
            }
            for c in &mut self.volume_constraints {
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

                // **曲げ拘束(群4)**。`i`-`k` 距離を保つ距離ベースの簡略形なので、
                // 上の距離拘束と同じ式がそのまま使える(`BendingConstraint`のdoc参照)。
                for c in &mut self.bending_constraints {
                    let (i, k) = (c.i, c.k);
                    let delta = self.position[i] - self.position[k];
                    let len = delta.length();
                    if len < 1e-12 {
                        continue;
                    }
                    let n_dir = delta.scale(1.0 / len);
                    let constraint_val = len - c.rest;
                    let alpha_tilde = c.compliance / (sub_dt * sub_dt);
                    let (wi, wk) = (self.inv_mass[i], self.inv_mass[k]);
                    let denom = wi + wk + alpha_tilde;
                    if denom <= 0.0 {
                        continue;
                    }
                    let delta_lambda = (-constraint_val - alpha_tilde * c.lambda) / denom;
                    c.lambda += delta_lambda;
                    let correction = n_dir.scale(delta_lambda);
                    self.position[i] = self.position[i].addcarry_scaled(correction, wi);
                    self.position[k] = self.position[k].addcarry_scaled(correction, -wk);
                }

                // **体積拘束(群4)**。四面体の符号付き体積 V を基準値へ戻す。
                // 勾配は閉形式:
                //   ∇₁V = (p₂-p₀)×(p₃-p₀)/6, ∇₂V = (p₃-p₀)×(p₁-p₀)/6,
                //   ∇₃V = (p₁-p₀)×(p₂-p₀)/6, ∇₀V = -(∇₁+∇₂+∇₃)
                for c in &mut self.volume_constraints {
                    let [i0, i1, i2, i3] = c.particles;
                    let (p0, p1, p2, p3) = (
                        self.position[i0],
                        self.position[i1],
                        self.position[i2],
                        self.position[i3],
                    );
                    let volume = tetrahedron_volume(p0, p1, p2, p3);
                    let constraint_val = volume - c.rest_volume;
                    let g1 = (p2 - p0).cross(p3 - p0).scale(1.0 / 6.0);
                    let g2 = (p3 - p0).cross(p1 - p0).scale(1.0 / 6.0);
                    let g3 = (p1 - p0).cross(p2 - p0).scale(1.0 / 6.0);
                    let g0 = (g1 + g2 + g3).scale(-1.0);
                    let gradients = [g0, g1, g2, g3];
                    let alpha_tilde = c.compliance / (sub_dt * sub_dt);
                    let mut denom = alpha_tilde;
                    for (n, &index) in c.particles.iter().enumerate() {
                        denom += self.inv_mass[index] * gradients[n].length_sq();
                    }
                    if denom <= 1e-18 {
                        continue;
                    }
                    let delta_lambda = (-constraint_val - alpha_tilde * c.lambda) / denom;
                    c.lambda += delta_lambda;
                    for (n, &index) in c.particles.iter().enumerate() {
                        self.position[index] = self.position[index]
                            .addcarry_scaled(gradients[n], self.inv_mass[index] * delta_lambda);
                    }
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

/// 四面体の符号付き体積 $V=\frac16(\mathbf{p}_1-\mathbf{p}_0)\cdot[(\mathbf{p}_2-\mathbf{p}_0)\times(\mathbf{p}_3-\mathbf{p}_0)]$。
pub fn tetrahedron_volume(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    (p1 - p0).dot((p2 - p0).cross(p3 - p0)) / 6.0
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

/// **増分Hで追加**。これが無いあいだ`SoftBody`は`World::step()`のドメイン一覧から
/// 漏れており、`enable_soft_body`で載せても再生しても一切動かなかった
/// (D13のテストが`world.soft_body_mut().unwrap().step(...)`と手で回していたのは
/// このため)。シーンギャラリーへD13を出すには自動ステップが要る。
///
/// 積分パラメータ(重力・サブステップ数・反復数・減衰)は`SolverContext`が
/// 運べないため`SoftBody`自身のフィールドから採る(フィールドのdoc参照)。
impl sim_core::Solver for SoftBody {
    /// XPBDは拘束を反復で解くため陽的な安定限界を持たない(サブステップ分割で
    /// 剛性を稼ぐ設計)。`ThermalSolver`の陰的Eulerと同じく無制限を返す。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, _ctx: &mut sim_core::SolverContext) {
        let (gravity, substeps, iterations, damping) =
            (self.gravity, self.substeps, self.iterations, self.damping);
        SoftBody::step(self, dt, gravity, substeps, iterations, damping);
    }

    fn state_hash(&self, hasher: &mut sim_core::StateHasher) {
        hasher.write_u64(self.position.len() as u64);
        for p in &self.position {
            hasher.write_f64(p.x);
            hasher.write_f64(p.y);
            hasher.write_f64(p.z);
        }
    }

    /// 運動エネルギー Σ½mv² と重力ポテンシャル Σ m·(−g)·r。**ピン留め粒子
    /// (`inv_mass=0`)は無限質量なので両方から除く**——含めると発散する。
    /// 距離拘束の弾性エネルギーは、XPBDのコンプライアンスが実質0(D13は1e-10)で
    /// 拘束が剛体的に効く運用のため計上しない(縮約、`elastic`は0のまま)。
    fn total_energy(&self) -> sim_core::EnergyBreakdown {
        let mut kinetic = 0.0;
        let mut potential = 0.0;
        for i in 0..self.position.len() {
            if self.inv_mass[i] <= 0.0 {
                continue;
            }
            let m = 1.0 / self.inv_mass[i];
            let v = self.velocity[i];
            kinetic += 0.5 * m * (v.x * v.x + v.y * v.y + v.z * v.z);
            let r = self.position[i];
            potential -= m * (self.gravity.x * r.x + self.gravity.y * r.y + self.gravity.z * r.z);
        }
        sim_core::EnergyBreakdown {
            kinetic,
            potential,
            ..Default::default()
        }
    }

    fn approximations(&self) -> Vec<sim_core::Approximation> {
        vec![
            sim_core::Approximation {
                name: "距離拘束のみ(XPBD)",
                reason: "自己衝突・剛体/流体との結合は未実装(曲げ拘束・体積拘束は群4で実装済み)。長いチェーンの剛な拘束は Gauss-Seidel の収束が遅く、反復数で精度が変わる。",
                doc: "docs/10-mechanics/06-soft-body-particles.md",
                can_disable: false,
            },
            sim_core::Approximation {
                name: "弾性エネルギーを計上しない",
                reason: "コンプライアンスが実質0で拘束が剛体的に効く運用のため、\
                         elasticは0のまま。",
                doc: "docs/10-mechanics/06-soft-body-particles.md",
                can_disable: false,
            },
        ]
    }
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

#[cfg(test)]
mod group4_tests {
    use super::*;

    /// **曲げ拘束がロープに「腰」を与えること**(群4)。
    ///
    /// 根元2粒子をピン留めした片持ちロープは、曲げ剛性が無ければ重力で
    /// 根元の真下へ垂れる。曲げ拘束を入れると水平に近い姿勢を保つ——
    /// **これが距離拘束だけでは表現できない性質**(ロープは伸びないが自由に折れる)。
    ///
    /// **収束は反復数に依存する**(設計 §4.1「長いチェーン(10リンク超)は収束が遅い
    /// —— 反復増で対応」)。ここでは①十分な反復で水平に近づくこと ②**反復を
    /// 増やすほど単調に真っ直ぐになること**の両方を見る。後者があることで、
    /// 「水平になり切らない」のがモデルの誤りではなく Gauss-Seidel の収束速度の
    /// 問題であることが区別できる。
    #[test]
    fn bending_constraints_keep_a_cantilever_rope_from_collapsing() {
        let gravity = Vec3::new(0.0, -9.80665, 0.0);
        let build = |bending_compliance: Option<f64>, n_sub: u32, n_iter: u32| {
            let mut body = rope(
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                10,
                0.1,
                1.0,
                0.0, // 距離拘束は完全剛(伸びない)。
            );
            // **根元の2粒子をピン留めする**のが要点。1粒子だけだとロープが
            // 「真っ直ぐなまま」剛体棒のように振り子運動してしまい、曲げ剛性の
            // 有無を区別できない(最初 pin(0) だけで書いて実際にそうなった:
            // 曲げ拘束ありの先端が (-0.26, -0.94) と、長さを保ったまま真下へ回った)。
            // 2粒子を留めると根元の向きが固定され、曲げ剛性が形状に効く。
            body.pin(0);
            body.pin(1);
            if let Some(compliance) = bending_compliance {
                for k in 1..body.position.len() - 1 {
                    body.add_bending_constraint(k - 1, k, k + 1, compliance);
                }
            }
            // **強めの減衰で静止させてから測る**。減衰が弱いとロープが振り子の
            // ように振れ続け、測った時刻によって先端がどこにでも来る
            // (減衰0.1・2.5秒で測って実際にそうなった)。ここで見たいのは
            // 過渡ではなく**静止形状**なので、揺れを落として比べる。
            for _ in 0..3000 {
                body.step(1.0 / 240.0, gravity, n_sub, n_iter, 3.0);
            }
            *body.position.last().unwrap()
        };

        let floppy_tip = build(None, 8, 30);
        let stiff_tip = build(Some(1e-8), 8, 30);

        // 曲げ剛性が無ければ先端は根元の下へ垂れ下がる(水平には伸びない)。
        assert!(
            floppy_tip.y < -0.7 && floppy_tip.x < 0.4,
            "曲げ拘束なしでは垂れ下がるはず: tip={floppy_tip:?}"
        );
        // 曲げ剛性があれば水平に近い姿勢を保つ。
        assert!(
            stiff_tip.y > -0.45 && stiff_tip.x > 0.85,
            "曲げ拘束ありでは水平に近い姿勢を保つはず: tip={stiff_tip:?}"
        );

        // **反復を増やすほど真っ直ぐになる**(残る垂れ下がりは Gauss-Seidel の
        // 収束速度によるもので、曲げ拘束のモデル誤りではない)。
        let few = build(Some(1e-8), 4, 8);
        let many = build(Some(1e-8), 8, 30);
        assert!(
            many.y > few.y + 0.05,
            "反復を増やすほど水平に近づくはず: few={few:?} many={many:?}"
        );
    }

    /// **体積拘束が四面体の体積を保つこと**(群4)。
    ///
    /// 1頂点を強く押し込んでも、体積拘束があれば他の頂点が押し出されて
    /// 体積が戻る。**符号付き体積の閉形式**(`tetrahedron_volume`)と
    /// **その解析的な勾配**が正しいことの確認でもある。
    #[test]
    fn volume_constraint_restores_the_tetrahedron_volume_after_compression() {
        let mut body = SoftBody::new();
        let a = body.add_particle(Vec3::ZERO, 1.0);
        let b = body.add_particle(Vec3::new(1.0, 0.0, 0.0), 1.0);
        let c = body.add_particle(Vec3::new(0.0, 1.0, 0.0), 1.0);
        let d = body.add_particle(Vec3::new(0.0, 0.0, 1.0), 1.0);
        body.add_volume_constraint([a, b, c, d], 0.0);
        let rest = body.volume_constraints[0].rest_volume;
        assert!(
            (rest - 1.0 / 6.0).abs() < 1e-12,
            "単位四面体の体積は1/6: {rest}"
        );

        // 頂点 d を原点方向へ押し込む(体積を半分にする)。
        body.position[d] = Vec3::new(0.0, 0.0, 0.5);
        let compressed = tetrahedron_volume(
            body.position[a],
            body.position[b],
            body.position[c],
            body.position[d],
        );
        assert!((compressed - rest / 2.0).abs() < 1e-12, "{compressed}");

        // 重力なしで解かせる(拘束だけを働かせる)。
        for _ in 0..200 {
            body.step(1.0 / 240.0, Vec3::ZERO, 4, 8, 0.0);
        }
        let restored = tetrahedron_volume(
            body.position[a],
            body.position[b],
            body.position[c],
            body.position[d],
        );
        assert!(
            (restored - rest).abs() / rest < 1e-3,
            "体積拘束は基準体積へ戻すはず: restored={restored} rest={rest}"
        );
    }
}
