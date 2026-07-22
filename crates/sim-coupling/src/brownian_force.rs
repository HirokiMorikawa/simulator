//! `BrownianForce`(設計 docs/20-integration/01-coupling-matrix.md §3「P4: 温度・粘性 →
//! 微小剛体のランダム力」、docs/15-statistical/03-diffusion-brownian.md §2.1 のランジュバン
//! 方程式)。
//!
//! **縮約実装の理由**: 設計 docs/15-statistical/03-diffusion-brownian.md §4.1 が示す
//! BAOAB(Leimkuhler-Matthews)は Ornstein-Uhlenbeck 過程の厳密解を使う高精度分割法だが、
//! それは `sim-statistical` 自身の(この Coupling とは独立な)粒子システム向けの積分器
//! である。本実装は「熱・粘性 → 力学」という単方向の量の受け渡しに徹する `Coupling` として、
//! ランジュバン方程式($m\dot v=-\gamma v+\sqrt{2\gamma k_BT}\,\xi$)を素朴な
//! Euler-Maruyama で離散化する縮約版とする(慣性時間 $m/\gamma$ に対して $\Delta t$ が
//! 十分小さければ安定、設計§5「過減衰は $\gamma\Delta t/m>10$」の逆側の領域)。
//!
//! 保存量の対記帳(設計§1)については、他の2つの実装(`DissipationToHeat`・`JouleHeat`)と
//! 性質が異なることを明記する: 摩擦散逸・ジュール熱は決定的な量を右から左へ移すだけだが、
//! ゆらぎ散逸定理に基づくブラウン力は「平均としてのみ」熱浴とエネルギーが釣り合う
//! 統計的コップリングであり、1step ごとの厳密な対記帳(ΔE=0検算)はそもそも成立しない
//! (ランダムな注入・平均としての散逸は同じ`gamma`(Stokes抵抗)から出るという「一意性」の
//! 検証が本質、設計§2.1「抵抗とゆらぎは同じ分子衝突の2つの顔」)。そのため本実装の検証は
//! 単発のエネルギー差分ではなく、長時間平均のエネルギー等分配則
//! ($\langle\frac12 mv^2\rangle=\frac32k_BT$)への収束で行う。
//!
//! 決定論的乱数(設計docs/01-math/04-random.md)は`Coupling`トレイト自体にはrng引数が
//! 無い(設計docs/00-foundation/04-architecture.md §1.3のシグネチャ、`domain_states`
//! モジュールdoc参照)ため、この`Coupling`実装が自身の`SimRng`を保持する(`World`の
//! 中央ストリーム管理への正式な組み込みは、他のCoupling同様まだ`World::step()`
//! パイプラインに接続されていないため後続増分)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_math::SimRng;

/// ボルツマン定数 [J/K](CODATA、`sim-statistical::BOLTZMANN_CONSTANT`と同値。
/// crateをまたいだ依存を避けるためこの値をここでも定義する、`sim-em::raytracer`の
/// ローカル`KB`定数と同じ慣行)。
const BOLTZMANN_CONSTANT: f64 = 1.380649e-23;

/// 温度・粘性から微小剛体へランジュバン方程式のランダム力(+ストークス抵抗)を注入する
/// (設計§1「保存量の橋」— ただしモジュールdoc参照のとおり統計的な釣り合いであり、
/// 1step毎の厳密な対記帳ではない)。
#[derive(Clone)]
pub struct BrownianForce {
    /// 対象剛体(`sim_mechanics::RigidBodySet`のindex)。
    pub body_index: usize,
    /// 剛体半径 [m](ストークス抵抗 $\gamma=6\pi\mu r$、設計§2.1)。
    pub radius: f64,
    /// 周囲媒質の粘性 [Pa·s]。
    pub viscosity: f64,
    /// 温度の参照元`ThermalNode`のインデックス。
    pub thermal_node: usize,
    rng: SimRng,
}

impl BrownianForce {
    pub fn new(
        body_index: usize,
        radius: f64,
        viscosity: f64,
        thermal_node: usize,
        seed: u64,
        stream: u64,
    ) -> BrownianForce {
        BrownianForce {
            body_index,
            radius,
            viscosity,
            thermal_node,
            rng: SimRng::new(seed, stream),
        }
    }

    /// ストークス抵抗係数 $\gamma=6\pi\mu r$(設計§2.1)。
    fn gamma(&self) -> f64 {
        6.0 * std::f64::consts::PI * self.viscosity * self.radius
    }
}

impl Coupling for BrownianForce {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Thermal)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(thermal) = &world.thermal else {
            return;
        };
        let Some(node) = thermal.nodes.get(self.thermal_node) else {
            return;
        };
        let temperature = node.temperature;
        let gamma = self.gamma();
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }

        let v = world.mechanics.bodies.linear_velocity[self.body_index];
        // ランジュバン方程式 m dv = -gamma*v*dt + sqrt(2*gamma*kB*T*dt)*N(0,1) の
        // Euler-Maruyama離散化(モジュールdoc参照)。
        let drag_dv = v.scale(-gamma / mass * dt);
        let noise_sigma = (2.0 * gamma * BOLTZMANN_CONSTANT * temperature * dt).sqrt() / mass;
        let noise_dv = self.rng.maxwell_boltzmann_velocity(noise_sigma);

        world.mechanics.bodies.linear_velocity[self.body_index] = v + drag_dv + noise_dv;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// 揺動散逸定理の直接検証(設計docs/15-statistical/03-diffusion-brownian.md §7
    /// 「速度分布: マクスウェル分布への収束」に対応するエネルギー等分配版): 重力・接触なしの
    /// 微小剛体(1μm相当)に`BrownianForce`のみを外力として長時間適用し、時間平均の運動
    /// エネルギーが等分配則$\langle\frac12mv^2\rangle=\frac32k_BT$に収束することを確認する。
    /// 単発のエネルギー対記帳ではなく統計的収束の検証であるため(モジュールdoc参照)、
    /// 許容誤差は緩め(rel<10%)にとる — 独立サンプル数は物理時間/緩和時間
    /// ($\tau=m/\gamma$)程度しかなく、統計誤差が$1/\sqrt{N_{eff}}$で効くため
    /// (実装検証中の実測rel_errは2.2%だが、乱数シード依存の変動を見込んで余裕を持たせた)。
    #[test]
    fn brownian_force_converges_to_energy_equipartition() {
        let materials = MaterialDb::standard();
        let mut mechanics = MechanicsSolver::new(0.0); // 重力なし: ランジュバン平衡のみを見る
        let water_like_density = 1050.0; // ポリスチレン球相当
        let radius: f64 = 1.0e-6;
        let volume = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
        let mass = water_like_density * volume;

        // MaterialDbから密度が一致する材料を作る代わりに、任意の材料を使い質量を上書きする
        // (RigidBodyDesc::mass_overrideで指定、ミクロスケールの密度較正は本テストの対象外)。
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        desc.mass_override = Some(mass);
        let body_idx = mechanics.create_body(desc, &materials);

        let temperature = 293.15;
        let mut thermal = ThermalSolver::new(temperature);
        let node_idx = thermal.add_node(ThermalNode::new(temperature, 1000.0));

        let viscosity = 1.002e-3; // 水の粘性(20℃)
        let mut coupling = BrownianForce::new(body_idx, radius, viscosity, node_idx, 42, 99);

        let gamma = coupling.gamma();
        let tau = mass / gamma; // 慣性時間
        let dt = tau / 50.0; // 緩和時間より十分小さいdt(明示的Euler-Maruyamaの安定域)
        let steps = 400_000u32;

        let mut sum_ke = 0.0;
        let mut sampled = 0u32;
        // 最初の10*tau分は初期条件(v=0)からの過渡応答として捨てる。
        let warmup_steps = (10.0 * tau / dt) as u32;
        for step in 0..steps {
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: Some(&mut thermal),
                em_circuit: None,
                em_electrostatics: None,
                gas: None,
                grid_fluid: None,
            };
            coupling.apply(&mut states, dt);
            if step >= warmup_steps {
                let v = mechanics.bodies.linear_velocity[body_idx];
                sum_ke += 0.5 * mass * v.length_sq();
                sampled += 1;
            }
        }

        let mean_ke = sum_ke / sampled as f64;
        let expected_ke = 1.5 * BOLTZMANN_CONSTANT * temperature;
        let rel_err = (mean_ke - expected_ke).abs() / expected_ke;
        assert!(
            rel_err < 0.1,
            "mean_ke={mean_ke:e} expected_ke={expected_ke:e} rel_err={rel_err:.4}"
        );
    }
}
