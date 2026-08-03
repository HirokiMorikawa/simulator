//! 万有引力・N体問題(総当たり + leapfrog)。設計: docs/16-astro/01-gravitation-nbody.md。
//!
//! Pα の基礎部分: 総当たり($O(N^2)$、少数体は Barnes-Hut より高精度・十分速いと
//! 設計 §4.1 が明記する既定モード)+ leapfrog(kick-drift-kick、シンプレクティック)。
//! Barnes-Hut($N\gtrsim256$)・WHFast・浮動原点・レジーム切替(§4.2 の残り)は Phase 3+ で拡張する。

use sim_core::{Approximation, EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::Vec3;

/// 万有引力定数 [N m^2/kg^2]。設計 §2、CODATA 値。
pub const GRAVITATIONAL_CONSTANT: f64 = 6.674e-11;

/// 大気抗力設定(再突入シナリオの土台、設計docs/16-astro/02-orbital-mechanics.md §2.3
/// 「大気圏再突入」)。`exponential_atmosphere_density`(既にA6検証で単体テスト済み)を
/// `central_body`からの距離(=高度、`planet_radius`基準)で評価し、`ballistic_coefficient`
/// (抗力係数×断面積/質量)が設定された他の各ボディへ抗力を適用する。**縮約実装の理由**:
/// 大気は`central_body`と共回転する前提を置かず(実際は地表に対する相対風速が正しいが、
/// ここでは中心天体に対する相対速度をそのまま使う)。抗力は非保存力のため、この力が
/// 働く間はleapfrogの厳密なシンプレクティック性(エネルギー誤差の有界振動)は失われる
/// (物理的に正しい散逸なので許容する)。空力加熱・アブレーションは`reentry_heating`
/// フィールド(`ReentryHeatingState`のdoc参照)で別途構成する。設計が求める
/// 「レジーム切替時の自動微細刻み」は未実装(`atmosphere`モジュールdoc参照)。ボディは
/// `enable_atmospheric_drag`を呼ぶ前に全て`add_body`しておく必要がある
/// (`ballistic_coefficient`はその時点のボディ数で初期化するため)。
#[derive(Clone)]
pub struct AtmosphericDragConfig {
    pub central_body: usize,
    pub surface_density: f64,
    pub scale_height: f64,
    pub planet_radius: f64,
    /// ボディごとの弾道係数(抗力係数×断面積/質量)。`None`なら抗力を受けない
    /// (中心天体自身や無関係な他の天体に使う)。
    pub ballistic_coefficient: Vec<Option<f64>>,
    /// ボディごとの空力加熱・アブレーション設定(`ReentryHeatingState`のdoc参照)。
    /// `None`なら加熱評価を行わない。
    pub reentry_heating: Vec<Option<ReentryHeatingState>>,
}

/// 空力加熱・アブレーション設定+状態(設計docs/16-astro/02-orbital-mechanics.md §2.3
/// 「空力加熱」「アブレーション」の縮約実装、`atmosphere::{sutton_graves_heat_flux,
/// ablation_mass_loss}`のdoc参照)。`AtmosphericDragConfig`と同じ中心天体・大気
/// パラメータ(密度・相対速度)を再利用する——大気圏内でのみ意味を持つ量のため。
#[derive(Clone, Copy, Debug)]
pub struct ReentryHeatingState {
    /// よどみ点先端半径(Sutton-Graves式の$R_n$)。
    pub nose_radius: f64,
    /// 熱を受け取る熱シールドの面積(アブレーション質量損失の計算に使う)。
    pub heat_shield_area: f64,
    /// 熱シールド材の気化潜熱(アブレーション質量損失の計算に使う)。
    pub latent_heat_vaporization: f64,
    /// 残存する熱シールド質量。0に達すると焼失(`EventKind::PhaseChanged`を発行、
    /// ボディ自体の質量除去は行わない——熱シールド質量は本体質量とは別の簿記量として
    /// 扱う縮約実装、設計§5「詳細な熱防護材の化学は非対象」の範囲)。
    pub remaining_shield_mass: f64,
}

/// 1PN相対論補正の設定(オプトイン、D39「相対論ON/OFF」、設計docs/16-astro/
/// 03-relativistic-corrections.md)。`central_body`まわりのtest-particle近似
/// (`relativity::pn1_acceleration`をそのまま使う、`central_body`自身には適用しない)。
/// **縮約実装の理由**: 設計が示す`RelativitySettings`構造体(複数天体への一般化・
/// GR効果の個別トグル)ではなく、`sim-astro`のモジュールdocが明記する既存の縮約
/// (「1体・test-particle近似」)をそのまま`NBodySystem`へ接続した最小形。
#[derive(Clone, Copy)]
pub struct RelativisticCorrectionConfig {
    pub central_body: usize,
    pub speed_of_light: f64,
}

/// N体系。設計 §3 の `NBodySystem` から、Barnes-Hut ツリー・積分器種別の選択機構を除いた
/// P0 スコープ(総当たり + leapfrog 固定)。
#[derive(Clone)]
pub struct NBodySystem {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub mass: Vec<f64>,
    /// 近接特異点の緩和(設計 §2)。既定 0(実天体は接触を剛体/再突入に委ねる)。
    pub softening: f64,
    /// 大気抗力(`AtmosphericDragConfig`のdoc参照)。既定`None`(抗力無し)。
    pub atmospheric_drag: Option<AtmosphericDragConfig>,
    /// 1PN相対論補正(`RelativisticCorrectionConfig`のdoc参照)。既定`None`(補正無し)。
    pub relativistic_correction: Option<RelativisticCorrectionConfig>,
}

impl NBodySystem {
    pub fn new(softening: f64) -> NBodySystem {
        NBodySystem {
            position: Vec::new(),
            velocity: Vec::new(),
            mass: Vec::new(),
            softening,
            atmospheric_drag: None,
            relativistic_correction: None,
        }
    }

    /// `central_body`まわりの1PN補正を有効化する(`RelativisticCorrectionConfig`の
    /// doc参照)。
    pub fn enable_relativistic_correction(&mut self, central_body: usize, speed_of_light: f64) {
        self.relativistic_correction = Some(RelativisticCorrectionConfig {
            central_body,
            speed_of_light,
        });
    }

    pub fn add_body(&mut self, position: Vec3, velocity: Vec3, mass: f64) -> usize {
        let idx = self.position.len();
        self.position.push(position);
        self.velocity.push(velocity);
        self.mass.push(mass);
        idx
    }

    /// `central_body`を中心とする指数大気モデルによる抗力を有効化する。この時点で
    /// 既に`add_body`済みの全ボディに対して`ballistic_coefficient`を`None`
    /// (抗力無し)で初期化する(`AtmosphericDragConfig`のdoc「呼ぶ前に全て
    /// `add_body`しておく必要がある」参照)。
    pub fn enable_atmospheric_drag(
        &mut self,
        central_body: usize,
        surface_density: f64,
        scale_height: f64,
        planet_radius: f64,
    ) {
        self.atmospheric_drag = Some(AtmosphericDragConfig {
            central_body,
            surface_density,
            scale_height,
            planet_radius,
            ballistic_coefficient: vec![None; self.len()],
            reentry_heating: vec![None; self.len()],
        });
    }

    /// `body`の弾道係数(抗力係数×断面積/質量)を設定する。`enable_atmospheric_drag`が
    /// 未呼び出しなら何もしない。
    pub fn set_ballistic_coefficient(&mut self, body: usize, value: f64) {
        if let Some(drag) = &mut self.atmospheric_drag {
            drag.ballistic_coefficient[body] = Some(value);
        }
    }

    /// `body`の空力加熱・アブレーション設定を有効化する(`ReentryHeatingState`のdoc
    /// 参照)。`enable_atmospheric_drag`が未呼び出しなら何もしない(密度・相対速度の
    /// 評価に中心天体・大気パラメータを再利用するため)。
    pub fn set_reentry_heating(
        &mut self,
        body: usize,
        nose_radius: f64,
        heat_shield_area: f64,
        latent_heat_vaporization: f64,
        shield_mass: f64,
    ) {
        if let Some(drag) = &mut self.atmospheric_drag {
            drag.reentry_heating[body] = Some(ReentryHeatingState {
                nose_radius,
                heat_shield_area,
                latent_heat_vaporization,
                remaining_shield_mass: shield_mass,
            });
        }
    }

    /// `body`の残存熱シールド質量(`set_reentry_heating`未設定、または
    /// `enable_atmospheric_drag`未呼び出しなら`None`)。
    pub fn heat_shield_mass(&self, body: usize) -> Option<f64> {
        self.atmospheric_drag
            .as_ref()?
            .reentry_heating
            .get(body)?
            .map(|h| h.remaining_shield_mass)
    }

    /// `body`の現在のよどみ点熱流束(Sutton-Graves式、W/m²)。加熱設定が無ければ`None`。
    pub fn reentry_heat_flux(&self, body: usize) -> Option<f64> {
        let drag = self.atmospheric_drag.as_ref()?;
        if body == drag.central_body {
            return None;
        }
        let heating = drag.reentry_heating.get(body)?.as_ref()?;
        let altitude =
            (self.position[body] - self.position[drag.central_body]).length() - drag.planet_radius;
        let speed = (self.velocity[body] - self.velocity[drag.central_body]).length();
        let density = crate::atmosphere::exponential_atmosphere_density(
            altitude,
            drag.surface_density,
            drag.scale_height,
        );
        Some(crate::atmosphere::sutton_graves_heat_flux(
            density,
            heating.nose_radius,
            speed,
        ))
    }

    /// 空力加熱・アブレーションを1ステップ分評価する(`step()`末尾から呼ぶ、
    /// `ReentryHeatingState`のdoc参照)。熱流束はステップ末(積分後)の位置・速度で
    /// 評価する——`accelerations()`はleapfrogの半キックごとに2回呼ばれるため、
    /// 質量損失の評価はそこに混ぜず`step()`終端で1回だけ行う(二重計上を避ける)。
    fn apply_reentry_heating_and_ablation(&mut self, dt: f64, ctx: &mut SolverContext) {
        let Some(drag) = &self.atmospheric_drag else {
            return;
        };
        let central_body = drag.central_body;
        let planet_radius = drag.planet_radius;
        let surface_density = drag.surface_density;
        let scale_height = drag.scale_height;
        let central_position = self.position[central_body];
        let central_velocity = self.velocity[central_body];
        let n = self.len();
        let position = &self.position;
        let velocity = &self.velocity;
        let drag = self.atmospheric_drag.as_mut().expect("checked Some above");
        for i in 0..n {
            if i == central_body {
                continue;
            }
            let Some(heating) = drag.reentry_heating.get_mut(i).and_then(|h| h.as_mut()) else {
                continue;
            };
            if heating.remaining_shield_mass <= 0.0 {
                continue;
            }
            let altitude = (position[i] - central_position).length() - planet_radius;
            let speed = (velocity[i] - central_velocity).length();
            let density = crate::atmosphere::exponential_atmosphere_density(
                altitude,
                surface_density,
                scale_height,
            );
            let heat_flux =
                crate::atmosphere::sutton_graves_heat_flux(density, heating.nose_radius, speed);
            let mass_loss = crate::atmosphere::ablation_mass_loss(
                heat_flux,
                heating.heat_shield_area,
                dt,
                heating.latent_heat_vaporization,
            );
            heating.remaining_shield_mass = (heating.remaining_shield_mass - mass_loss).max(0.0);
            if heating.remaining_shield_mass <= 0.0 {
                ctx.events.push(sim_core::Event {
                    step: 0,
                    source: sim_core::SourceId(i as u64),
                    kind: sim_core::EventKind::PhaseChanged,
                });
            }
        }
    }

    pub fn len(&self) -> usize {
        self.position.len()
    }

    pub fn is_empty(&self) -> bool {
        self.position.is_empty()
    }

    /// 設計 §2: 総当たり重ね合わせによる各体の加速度。$O(N^2)$。
    fn accelerations(&self) -> Vec<Vec3> {
        let n = self.len();
        let mut acc = vec![Vec3::ZERO; n];
        let eps_sq = self.softening * self.softening;
        for (i, acc_i) in acc.iter_mut().enumerate() {
            for j in 0..n {
                if i == j {
                    continue;
                }
                let d = self.position[j] - self.position[i];
                let dist_sq = d.length_sq() + eps_sq;
                let dist = dist_sq.sqrt();
                let factor = GRAVITATIONAL_CONSTANT * self.mass[j] / (dist_sq * dist);
                *acc_i = acc_i.addcarry_scaled(d, factor);
            }
        }
        if let Some(drag) = &self.atmospheric_drag {
            for (i, acc_i) in acc.iter_mut().enumerate() {
                if i == drag.central_body {
                    continue;
                }
                let Some(Some(beta)) = drag.ballistic_coefficient.get(i).copied() else {
                    continue;
                };
                let r_rel = self.position[i] - self.position[drag.central_body];
                let altitude = r_rel.length() - drag.planet_radius;
                let v_rel = self.velocity[i] - self.velocity[drag.central_body];
                let speed = v_rel.length();
                if speed < 1e-9 {
                    continue;
                }
                let density = crate::atmosphere::exponential_atmosphere_density(
                    altitude,
                    drag.surface_density,
                    drag.scale_height,
                );
                let drag_accel_magnitude = 0.5 * density * speed * speed * beta;
                *acc_i = acc_i.addcarry_scaled(v_rel, -drag_accel_magnitude / speed);
            }
        }
        if let Some(rel) = &self.relativistic_correction {
            let gm_central = GRAVITATIONAL_CONSTANT * self.mass[rel.central_body];
            for (i, acc_i) in acc.iter_mut().enumerate() {
                if i == rel.central_body {
                    continue;
                }
                let r_vec = self.position[i] - self.position[rel.central_body];
                let v_vec = self.velocity[i] - self.velocity[rel.central_body];
                *acc_i = *acc_i
                    + crate::relativity::pn1_acceleration(
                        gm_central,
                        rel.speed_of_light,
                        r_vec,
                        v_vec,
                    );
            }
        }
        acc
    }
}

impl Solver for NBodySystem {
    /// シンプレクティック積分は明示的な CFL 条件を持たない(軌道周期に対する刻みの妥当性は
    /// Orchestrator の sub-step 決定に委ねる、設計 §4.2「天体は独立時間軸」)。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    /// leapfrog(kick-drift-kick)。設計 §4.2:
    /// v_{1/2}=v_0+dt/2・a_0、x_1=x_0+dt・v_{1/2}、v_1=v_{1/2}+dt/2・a_1。
    fn step(&mut self, dt: f64, ctx: &mut SolverContext) {
        let n = self.len();
        if n == 0 {
            return;
        }
        let a0 = self.accelerations();
        for (v, &a) in self.velocity.iter_mut().zip(a0.iter()) {
            *v = v.addcarry_scaled(a, dt * 0.5);
        }
        for (p, &v) in self.position.iter_mut().zip(self.velocity.iter()) {
            *p = p.addcarry_scaled(v, dt);
        }
        let a1 = self.accelerations();
        for (v, &a) in self.velocity.iter_mut().zip(a1.iter()) {
            *v = v.addcarry_scaled(a, dt * 0.5);
        }
        self.apply_reentry_heating_and_ablation(dt, ctx);
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        let n = self.len();
        hasher.write_u64(n as u64);
        for i in 0..n {
            hasher.write_vec3(self.position[i]);
            hasher.write_vec3(self.velocity[i]);
        }
    }

    /// 運動エネルギー + 重力ポテンシャル(対ごとに1回、$-Gm_im_j/|r_i-r_j|$)。
    /// 設計 §2 の支配方程式から導かれるポテンシャル(EnergyBreakdown.potential に計上)。
    fn total_energy(&self) -> EnergyBreakdown {
        let n = self.len();
        let mut kinetic = 0.0;
        for i in 0..n {
            kinetic += 0.5 * self.mass[i] * self.velocity[i].length_sq();
        }
        let mut potential = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                let dist = (self.position[j] - self.position[i]).length();
                potential -= GRAVITATIONAL_CONSTANT * self.mass[i] * self.mass[j] / dist;
            }
        }
        EnergyBreakdown {
            kinetic,
            potential,
            ..Default::default()
        }
    }

    fn approximations(&self) -> Vec<Approximation> {
        let mut out = vec![Approximation {
            name: "天体: 質点N体",
            reason: "形状・自転・剛体潮汐は扱わない(潮汐力は独立した純粋関数として提供)。",
            doc: "docs/16-astro/01-gravitation-nbody.md",
            can_disable: false,
        }];
        if self.atmospheric_drag.is_some() {
            out.push(Approximation {
                name: "大気: 指数関数モデル",
                reason: "スケールハイト一定の等温大気。大気は中心天体と共回転しない。",
                doc: "docs/16-astro/02-orbital-mechanics.md",
                can_disable: false,
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{EventQueue, MaterialDb};
    use sim_math::SimRng;

    fn step_n(sys: &mut NBodySystem, dt: f64, n: u32) {
        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        for _ in 0..n {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            sys.step(dt, &mut ctx);
        }
    }

    /// A3: 円軌道速度 v=sqrt(GM/r)、rel 0.1%(docs/21-verification/01-analytic-tests.md A3)。
    /// 太陽-地球相当(1AU、太陽質量)で1公転させ、半径がほぼ一定に保たれることを確認する。
    #[test]
    fn a3_circular_orbit_speed_matches_vis_viva_formula() {
        let mass_sun = 1.989e30;
        let r = 1.496e11; // 1 AU
        let mut sys = NBodySystem::new(0.0);
        sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_sun);
        let v_circ = (GRAVITATIONAL_CONSTANT * mass_sun / r).sqrt();
        let idx = sys.add_body(Vec3::new(r, 0.0, 0.0), Vec3::new(0.0, v_circ, 0.0), 1.0);

        let period =
            2.0 * std::f64::consts::PI * (r.powi(3) / (GRAVITATIONAL_CONSTANT * mass_sun)).sqrt();
        let steps = 10_000u32;
        let dt = period / steps as f64;
        step_n(&mut sys, dt, steps);

        // 1周後、出発点付近に戻り半径がほぼ一定であること。
        let final_r = sys.position[idx].length();
        assert!((final_r - r).abs() / r < 0.001, "final_r={final_r} r={r}");
        let final_speed = sys.velocity[idx].length();
        assert!(
            (final_speed - v_circ).abs() / v_circ < 0.001,
            "final_speed={final_speed} v_circ={v_circ}"
        );
    }

    /// A2(縮約版): 二体のエネルギー・角運動量保存。10⁶周のフル検証は長時間級のため、
    /// 縮約(100周)でシンプレクティック積分のドリフトが小さいことを確認する
    /// (docs/21-verification/01-analytic-tests.md A2 注記: 10⁴周縮約は分級/通常CI)。
    #[test]
    fn a2_two_body_energy_and_angular_momentum_drift_stays_small_over_many_orbits() {
        let mass_sun = 1.989e30;
        let r = 1.496e11;
        let mut sys = NBodySystem::new(0.0);
        sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_sun);
        let v_circ = (GRAVITATIONAL_CONSTANT * mass_sun / r).sqrt();
        let idx = sys.add_body(Vec3::new(r, 0.0, 0.0), Vec3::new(0.0, v_circ, 0.0), 1.0);

        let period =
            2.0 * std::f64::consts::PI * (r.powi(3) / (GRAVITATIONAL_CONSTANT * mass_sun)).sqrt();
        let steps_per_orbit = 1000u32;
        let dt = period / steps_per_orbit as f64;
        let orbits = 100u32;

        let e0 = sys.total_energy().total();
        let l0 = sys.position[idx].cross(sys.velocity[idx]).length();

        step_n(&mut sys, dt, steps_per_orbit * orbits);

        let e1 = sys.total_energy().total();
        let l1 = sys.position[idx].cross(sys.velocity[idx]).length();

        let e_drift = (e1 - e0).abs() / e0.abs();
        let l_drift = (l1 - l0).abs() / l0;
        assert!(
            e_drift < 1e-6,
            "energy drift {e_drift} over {orbits} orbits"
        );
        assert!(
            l_drift < 1e-9,
            "angular momentum drift {l_drift} over {orbits} orbits"
        );
    }

    /// A7: 三体カオス決定論 — 同一初期条件を2回実行すると状態ハッシュが厳密一致する
    /// (docs/21-verification/01-analytic-tests.md A7)。
    #[test]
    fn a7_three_body_chaos_is_deterministic_across_runs() {
        let run = || {
            let mut sys = NBodySystem::new(1e9); // 弱いソフトニングで近接発散を避ける
            sys.add_body(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 1.0e30);
            sys.add_body(
                Vec3::new(1.0e11, 0.0, 0.0),
                Vec3::new(0.0, 2.0e4, 0.0),
                5.0e29,
            );
            sys.add_body(
                Vec3::new(-0.6e11, 0.8e11, 0.0),
                Vec3::new(-1.5e4, -1.0e4, 0.0),
                3.0e29,
            );
            step_n(&mut sys, 3600.0, 2000);
            let mut hasher = StateHasher::new();
            sys.state_hash(&mut hasher);
            hasher.finish()
        };
        assert_eq!(run(), run());
    }

    /// A1: ケプラー第3法則 T²∝a³、rel 0.1%(docs/21-verification/01-analytic-tests.md A1)。
    /// 設計は「太陽系8惑星」を想定するが、実際の8惑星(水星88日〜海王星165年)を刻み解像
    /// 良く高速テストとして回すのは非現実的なため、同じ中心天体の周りに8個の合成衛星を
    /// 幾何級数的な半径(比1.4、最大/最小の周期比 ≈34倍で高速に完走できる)で配置し、
    /// 同一のN体シミュレータ・積分器でT²∝a³が全軌道スケールで成立することを検証する
    /// (法則自体は距離のスケールに依らない — a3/a2 で単一衛星の円軌道physicsは既に検証済み。
    /// 本テストは「複数衛星を同時にシミュレートしても各軌道が独立にケプラー則を満たすか」を
    /// 追加検証する)。各衛星の公転周期は y 座標が負→正に転じる時刻(1周目の帰還)から実測する。
    #[test]
    fn a1_kepler_third_law_holds_across_orbital_scales() {
        let mass_central = 1.989e30;
        let mut sys = NBodySystem::new(0.0);
        sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_central);

        let n_satellites = 8;
        let mut indices = Vec::new();
        let mut radii = Vec::new();
        for k in 0..n_satellites {
            let r = 5.0e10 * 1.4_f64.powi(k);
            let v = (GRAVITATIONAL_CONSTANT * mass_central / r).sqrt();
            let idx = sys.add_body(Vec3::new(r, 0.0, 0.0), Vec3::new(0.0, v, 0.0), 1.0);
            indices.push(idx);
            radii.push(r);
        }

        let analytic_period = |r: f64| {
            2.0 * std::f64::consts::PI
                * (r.powi(3) / (GRAVITATIONAL_CONSTANT * mass_central)).sqrt()
        };
        let t_min = analytic_period(radii[0]);
        let t_max = analytic_period(radii[n_satellites as usize - 1]);
        let dt = t_min / 2000.0;
        let total_steps = (1.05 * t_max / dt).ceil() as u32;

        let materials = sim_core::MaterialDb::standard();
        let mut rng = sim_math::SimRng::new(1, 1);
        let mut events = sim_core::EventQueue::new();

        let mut gone_negative = vec![false; n_satellites as usize];
        let mut measured_period: Vec<Option<f64>> = vec![None; n_satellites as usize];
        let mut prev_y: Vec<f64> = indices.iter().map(|&idx| sys.position[idx].y).collect();
        let mut prev_t = 0.0;
        for step in 0..total_steps {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            sys.step(dt, &mut ctx);
            let t = (step + 1) as f64 * dt;
            for (s, &idx) in indices.iter().enumerate() {
                if measured_period[s].is_some() {
                    continue;
                }
                let y = sys.position[idx].y;
                if y < 0.0 {
                    gone_negative[s] = true;
                } else if gone_negative[s] {
                    // 線形補間でゼロ交差時刻をサブステップ精度で求める(離散検出の誤差 O(dt/T) を
                    // O(dt^2/T) に落とし、rel 0.1% 判定に十分な精度を確保する)。
                    let frac = -prev_y[s] / (y - prev_y[s]);
                    measured_period[s] = Some(prev_t + frac * (t - prev_t));
                }
                prev_y[s] = y;
            }
            prev_t = t;
        }

        let expected_ratio = 4.0 * std::f64::consts::PI * std::f64::consts::PI
            / (GRAVITATIONAL_CONSTANT * mass_central);
        for (s, &r) in radii.iter().enumerate() {
            let t_measured =
                measured_period[s].expect("orbit should complete within simulated window");
            let ratio = t_measured * t_measured / r.powi(3);
            let rel_err = (ratio - expected_ratio).abs() / expected_ratio;
            assert!(
                rel_err < 0.001,
                "satellite {s}: T^2/a^3={ratio} expected={expected_ratio} rel_err={rel_err}"
            );
        }
    }

    /// A4: ホーマン遷移 — 解析式どおりのΔv1で出発円軌道(半径r1)から瞬間噴射すると、
    /// 半周後の遠地点半径が目標円軌道半径r2に、解析式どおりのΔv2で円軌道化すると
    /// 最終速度が目標円軌道速度v2に、それぞれrel 0.5%以内で一致すること
    /// (docs/21-verification/01-analytic-tests.md A4)。
    #[test]
    fn a4_hohmann_transfer_delta_v_matches_analytic_value() {
        let mass_central = 1.989e30; // 太陽質量
        let r1 = 1.496e11; // 1AU(出発円軌道)
        let r2 = 2.0 * r1; // 目標円軌道(2AU)

        let v1_circ = (GRAVITATIONAL_CONSTANT * mass_central / r1).sqrt();
        let v2_circ = (GRAVITATIONAL_CONSTANT * mass_central / r2).sqrt();
        let a_transfer = 0.5 * (r1 + r2);
        let v_transfer_r1 =
            (GRAVITATIONAL_CONSTANT * mass_central * (2.0 / r1 - 1.0 / a_transfer)).sqrt();
        let v_transfer_r2 =
            (GRAVITATIONAL_CONSTANT * mass_central * (2.0 / r2 - 1.0 / a_transfer)).sqrt();
        let dv1 = v_transfer_r1 - v1_circ;
        let dv2 = v2_circ - v_transfer_r2;

        let mut sys = NBodySystem::new(0.0);
        sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_central);
        // 出発円軌道上、進行方向に沿って瞬間噴射(Δv1、近地点噴射)。
        let idx = sys.add_body(
            Vec3::new(r1, 0.0, 0.0),
            Vec3::new(0.0, v1_circ + dv1, 0.0),
            1.0,
        );

        let transfer_time = std::f64::consts::PI
            * (a_transfer.powi(3) / (GRAVITATIONAL_CONSTANT * mass_central)).sqrt();
        let steps = 200_000u32;
        let dt = transfer_time / steps as f64;
        step_n(&mut sys, dt, steps);

        // 半周後、遠地点(半径最大)に到達しているはず。
        let apoapsis_r = sys.position[idx].length();
        let rel_err_r2 = (apoapsis_r - r2).abs() / r2;
        assert!(
            rel_err_r2 < 0.005,
            "apoapsis_r={apoapsis_r} r2={r2} rel_err={rel_err_r2}"
        );

        // 円軌道化の噴射(Δv2、進行方向に加算)。
        let v_dir = sys.velocity[idx].normalize_or_zero();
        sys.velocity[idx] = sys.velocity[idx].addcarry_scaled(v_dir, dv2);

        let final_speed = sys.velocity[idx].length();
        let rel_err_v2 = (final_speed - v2_circ).abs() / v2_circ;
        assert!(
            rel_err_v2 < 0.005,
            "final_speed={final_speed} v2_circ={v2_circ} rel_err={rel_err_v2}"
        );
    }

    /// 再突入シナリオの土台(ワークストリームB): `atmosphere`モジュールdocが
    /// 指摘していた「抗力摂動はNBodySystem本体には未統合」というギャップを埋める。
    /// 地球相当の中心天体+低軌道衛星を`enable_atmospheric_drag`/
    /// `set_ballistic_coefficient`で構成し、実際の`NBodySystem::step()`
    /// (leapfrog)経由で、弾道係数を設定した衛星が設定しない場合より明確に速く
    /// 高度を失うことを確認する(既存のA6検証(`atmosphere.rs`、直接組んだ
    /// velocity Verlet風ループ)と同じ物理・同じ大気パラメータだが、今回は
    /// `NBodySystem`本体の`accelerations()`に統合された経路で検証する)。
    #[test]
    fn atmospheric_drag_integrated_into_nbody_step_decays_low_orbit_faster_than_without_drag() {
        let gm_earth = 3.986_004_418e14;
        let r_earth = 6.371e6;
        let mass_earth = gm_earth / GRAVITATIONAL_CONSTANT;
        let altitude0 = 180e3;
        let r0 = r_earth + altitude0;
        let v_circ = (gm_earth / r0).sqrt();
        let period = 2.0 * std::f64::consts::PI * (r0.powi(3) / gm_earth).sqrt();
        let steps_per_orbit = 2000u32;
        let dt = period / steps_per_orbit as f64;
        let orbits = 40u32;
        let steps = steps_per_orbit * orbits;

        let mut with_drag = NBodySystem::new(0.0);
        let central = with_drag.add_body(Vec3::ZERO, Vec3::ZERO, mass_earth);
        let sat = with_drag.add_body(Vec3::new(r0, 0.0, 0.0), Vec3::new(0.0, v_circ, 0.0), 1.0);
        with_drag.enable_atmospheric_drag(central, 1.225, 8500.0, r_earth);
        with_drag.set_ballistic_coefficient(sat, 2.2 * 1e-4); // Cd=2.2, area/mass=1e-4 (atmosphere.rsのA6高抗力と同値)

        let mut without_drag = with_drag.clone();
        without_drag.atmospheric_drag = None;

        step_n(&mut with_drag, dt, steps);
        step_n(&mut without_drag, dt, steps);

        let final_altitude_with_drag = with_drag.position[sat].length() - r_earth;
        let final_altitude_without_drag = without_drag.position[sat].length() - r_earth;

        assert!(
            final_altitude_with_drag < altitude0,
            "drag must cause net altitude loss: final={final_altitude_with_drag} initial={altitude0}"
        );
        assert!(
            final_altitude_with_drag < final_altitude_without_drag - 1000.0,
            "drag-enabled satellite must lose meaningfully more altitude than the \
             drag-free control: with_drag={final_altitude_with_drag} \
             without_drag={final_altitude_without_drag}"
        );
    }

    /// 空力加熱・アブレーション(設計§2.3「空力加熱」「アブレーション」)を
    /// `NBodySystem::step()`へ統合した検証。極端に薄い(すぐ焼失する)熱シールドを
    /// 持つカプセルを、高速(7km/s級)・低高度(50km)——Sutton-Graves熱流束が
    /// 非常に大きくなる条件——に置き、1stepで熱シールド質量が0まで減り
    /// `EventKind::PhaseChanged`イベントが発行されることを確認する(数値自体の
    /// 精度は`atmosphere.rs`の`sutton_graves_heat_flux`/`ablation_mass_loss`単体
    /// テストが厳密式評価で担う。ここでは実際の`step()`経由の配線——密度・相対
    /// 速度の評価・質量減衰・イベント発行——が機能することを確認する)。
    #[test]
    fn reentry_heating_depletes_shield_mass_and_emits_phase_changed_event_on_burn_through() {
        let r_earth = 6.371e6;
        let mass_earth = 5.972e24;
        let mut sys = NBodySystem::new(0.0);
        let central = sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_earth);
        let capsule = sys.add_body(
            Vec3::new(r_earth + 50_000.0, 0.0, 0.0),
            Vec3::new(0.0, 7000.0, 0.0),
            1000.0,
        );
        sys.enable_atmospheric_drag(central, 1.225, 8500.0, r_earth);
        sys.set_reentry_heating(capsule, 0.5, 1.0, 2.0e6, 0.001);

        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        sys.step(1e-3, &mut ctx);

        assert_eq!(
            sys.heat_shield_mass(capsule),
            Some(0.0),
            "shield mass must clamp to zero once burned through"
        );

        let drained = events.drain_sorted();
        assert!(
            drained
                .iter()
                .any(|e| e.kind == sim_core::EventKind::PhaseChanged
                    && e.source == sim_core::SourceId(capsule as u64)),
            "burning through the shield must emit a PhaseChanged event for the capsule body: {drained:?}"
        );
    }

    /// 加熱設定が無い場合(`set_reentry_heating`未呼び出し)は、大気抗力・重力は
    /// 従来どおり働くが焼失イベントは発行されない(誤発火防止の裏取り)。
    #[test]
    fn reentry_heat_flux_and_shield_mass_are_none_without_reentry_heating_configured() {
        let r_earth = 6.371e6;
        let mass_earth = 5.972e24;
        let mut sys = NBodySystem::new(0.0);
        let central = sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_earth);
        let capsule = sys.add_body(
            Vec3::new(r_earth + 50_000.0, 0.0, 0.0),
            Vec3::new(0.0, 7000.0, 0.0),
            1000.0,
        );
        sys.enable_atmospheric_drag(central, 1.225, 8500.0, r_earth);

        assert_eq!(sys.heat_shield_mass(capsule), None);
        assert_eq!(sys.reentry_heat_flux(capsule), None);

        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        sys.step(1e-3, &mut ctx);
        assert!(events.drain_sorted().is_empty());
    }

    /// D39 相対論ON/OFF(1PN補正を`NBodySystem`本体へ統合、設計docs/16-astro/
    /// 03-relativistic-corrections.mdが「未実装」と明記していたギャップを埋める)。
    /// `crates/sim-astro/src/relativity.rs`のA8テスト(近日点移動)と同じ誇張
    /// $GM/c^2$比(現実の水星の43″/世紀は非現実的な周回数を要するため、モジュールdoc
    /// が明記する誇張パラメータでの解析式比較方式)・同じ離心率ベクトル追跡法だが、
    /// 直接組んだvelocity Verlet風ループではなく実際の`NBodySystem::step()`
    /// (KDM leapfrog)経由で検証する点が新しい。`enable_relativistic_correction`
    /// ONでは近日点移動率が解析式`pn1_precession_per_orbit`とrel<1%で一致し、OFFでは
    /// 有意な歳差が検出されない(Keplerの閉軌道、ドリフトは数値誤差のみ)ことを確認する。
    #[test]
    fn d39_relativity_on_off_matches_analytic_precession_via_nbody_step() {
        let gm: f64 = 1.0;
        let c: f64 = 100.0; // A8と同じ誇張GM/c^2比(モジュールdoc参照)
        let a: f64 = 1.0;
        let e: f64 = 0.5;

        let r_peri = a * (1.0 - e);
        let v_peri = ((gm / a) * (1.0 + e) / (1.0 - e)).sqrt();

        let eccentricity_vector_angle = |r: Vec3, v: Vec3| -> f64 {
            let h = r.cross(v);
            let e_vec = v.cross(h).scale(1.0 / gm) - r.scale(1.0 / r.length());
            e_vec.y.atan2(e_vec.x)
        };

        let period = 2.0 * std::f64::consts::PI * (a.powi(3) / gm).sqrt();
        let orbits = 20;
        let steps_per_orbit = 8000;
        let dt = period / steps_per_orbit as f64;

        let run = |relativistic_correction_enabled: bool| -> f64 {
            let mut sys = NBodySystem::new(0.0);
            let central = sys.add_body(Vec3::ZERO, Vec3::ZERO, gm / GRAVITATIONAL_CONSTANT);
            let probe = sys.add_body(
                Vec3::new(r_peri, 0.0, 0.0),
                Vec3::new(0.0, v_peri, 0.0),
                0.0,
            );
            if relativistic_correction_enabled {
                sys.enable_relativistic_correction(central, c);
            }

            let initial_angle = eccentricity_vector_angle(sys.position[probe], sys.velocity[probe]);
            let mut unwrapped_angle = initial_angle;
            let mut prev_angle = initial_angle;

            let materials = MaterialDb::standard();
            let mut rng = SimRng::new(1, 1);
            let mut events = EventQueue::new();
            for _ in 0..(orbits * steps_per_orbit) {
                let mut ctx = SolverContext {
                    materials: &materials,
                    rng: &mut rng,
                    events: &mut events,
                };
                sys.step(dt, &mut ctx);

                let raw_angle = eccentricity_vector_angle(sys.position[probe], sys.velocity[probe]);
                let mut delta = raw_angle - prev_angle;
                while delta > std::f64::consts::PI {
                    delta -= 2.0 * std::f64::consts::PI;
                }
                while delta < -std::f64::consts::PI {
                    delta += 2.0 * std::f64::consts::PI;
                }
                unwrapped_angle += delta;
                prev_angle = raw_angle;
            }

            (unwrapped_angle - initial_angle) / orbits as f64
        };

        let measured_on = run(true);
        let analytic = crate::relativity::pn1_precession_per_orbit(gm, c, a, e);
        let rel_err_on = (measured_on - analytic).abs() / analytic;
        assert!(
            rel_err_on < 0.01,
            "ON: measured={measured_on:.6} analytic={analytic:.6} rel_err={rel_err_on:.4}"
        );

        let measured_off = run(false);
        assert!(
            measured_off.abs() < analytic * 0.05,
            "OFF: should show no meaningful precession (Keplerian closed orbit): \
             measured_off={measured_off:.6} analytic_on={analytic:.6}"
        );
    }
}

#[cfg(test)]
mod lagrange_tests {
    use super::*;
    use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
    use sim_math::SimRng;

    /// **設計 docs/16-astro/01-gravitation-nbody.md §7「ラグランジュ点 L4/L5 の
    /// トロヤ群の安定滞在」**(§7網羅監査で未カバーと判明し増分Lで追加)。
    ///
    /// L4 は主星-伴星を結ぶ線と正三角形をなす点で、**質量比が十分小さければ
    /// 線形安定**(Routh の条件 $m_2/(m_1+m_2) < 0.0385$)。太陽-木星系はこの
    /// 条件を満たし、実際にトロヤ群小惑星が滞在している。
    ///
    /// **判定**: L4 に静止衛星を置き(共回転系で静止 = 慣性系では主星まわりを
    /// 伴星と同じ角速度で回る)、複数周期にわたって**主星からの距離と伴星との
    /// 角度差が保たれる**ことを見る。不安定なら角度差が単調にずれていく。
    ///
    /// **対照実験を併せて行う**: 同じ構成で L4 から意図的に外した初期条件
    /// (角度を10°ずらす)は、同じ時間で角度差が大きく変動する。これにより
    /// 「たまたま動かないだけ」ではなく L4 が特別な点であることを示す。
    #[test]
    fn trojan_at_l4_stays_near_the_equilateral_point() {
        let m1 = 1.0e6; // 主星
        let m2 = 1.0e3; // 伴星(質量比 ≈ 1e-3 << 0.0385 なので安定条件を満たす)
        let separation = 1.0_f64;
        let gm_total = GRAVITATIONAL_CONSTANT * (m1 + m2);
        let omega = (gm_total / separation.powi(3)).sqrt(); // 共回転角速度
        let period = 2.0 * std::f64::consts::PI / omega;

        // 重心を原点に置く(そうしないと系全体が並進してしまう)。
        let r1 = -separation * m2 / (m1 + m2);
        let r2 = separation * m1 / (m1 + m2);

        // L4: 主星-伴星と正三角形をなす点(重心まわりに同じ角速度で回る)。
        let build = |angle_offset_deg: f64| -> (NBodySystem, usize) {
            let mut sys = NBodySystem::new(0.0);
            sys.add_body(Vec3::new(r1, 0.0, 0.0), Vec3::new(0.0, omega * r1, 0.0), m1);
            sys.add_body(Vec3::new(r2, 0.0, 0.0), Vec3::new(0.0, omega * r2, 0.0), m2);
            // L4 は主星から見て伴星より60°先。重心からの位置で書く。
            let theta = (60.0 + angle_offset_deg).to_radians();
            // 正三角形の頂点(主星・伴星のどちらからも `separation` の距離)。
            let x = r1 + separation * theta.cos();
            let y = separation * theta.sin();
            // 共回転: v = ω × r(重心まわり)。
            let trojan = sys.add_body(
                Vec3::new(x, y, 0.0),
                Vec3::new(-omega * y, omega * x, 0.0),
                0.0, // 試験粒子(制限3体問題)
            );
            (sys, trojan)
        };

        // 伴星から見たトロヤ群の角度差(重心を頂点とする角)の振れ幅を測る。
        let angular_spread = |offset_deg: f64| -> f64 {
            let (mut sys, trojan) = build(offset_deg);
            let materials = MaterialDb::standard();
            let mut rng = SimRng::new(3, 1);
            let mut events = EventQueue::new();
            let steps = 20_000;
            let dt = 5.0 * period / steps as f64; // 5周期ぶん
            let angle_of = |sys: &NBodySystem| -> f64 {
                let secondary = sys.position[1];
                let t = sys.position[trojan];
                let a = secondary.y.atan2(secondary.x);
                let b = t.y.atan2(t.x);
                let mut d = (b - a).to_degrees();
                while d > 180.0 {
                    d -= 360.0;
                }
                while d < -180.0 {
                    d += 360.0;
                }
                d
            };
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for _ in 0..steps {
                let mut ctx = SolverContext {
                    materials: &materials,
                    rng: &mut rng,
                    events: &mut events,
                };
                sys.step(dt, &mut ctx);
                let d = angle_of(&sys);
                lo = lo.min(d);
                hi = hi.max(d);
            }
            hi - lo
        };

        let at_l4 = angular_spread(0.0);
        assert!(
            at_l4 < 1.0,
            "L4に置いたトロヤ群は5周期にわたり正三角形配置を保つべき: 角度振れ幅={at_l4}°"
        );

        // 対照: L4から10°外すと振れ幅が桁違いに大きくなる(秤動が始まる)。
        let displaced = angular_spread(10.0);
        assert!(
            displaced > 5.0 * at_l4.max(1e-6),
            "L4から外すと秤動して振れ幅が大きくなるはず(L4が特別な点である証拠): \
             at_l4={at_l4}° displaced={displaced}°"
        );
    }
}
