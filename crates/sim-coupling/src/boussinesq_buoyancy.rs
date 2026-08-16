//! `BoussinesqBuoyancy`(設計 docs/20-integration/01-coupling-matrix.md §3
//! 「P3: 温度場 → 流体運動量」、docs/11-fluid/02-eulerian-grid.md §4.2の
//! Boussinesq浮力項 $f_y = -\beta(T-T_{amb})g$)。
//!
//! **縮約実装の理由**: 設計は温度場(`Grid3<f64>`、`GridFluid2D`本体のセルごとの温度)から
//! 空間的に変化する浮力を想定するが、`sim_fluid::GridFluid2D`(このcrateが使う縮約版)は
//! 温度場自体を持たない(`grid_fluid`モジュールdoc参照、周期境界の速度場のみ)。そのため
//! 本実装は、単一の`ThermalNode`(シーン全体を代表する「熱源」温度、`PistonGas`の
//! `GasCompartment`単一区画と同じ縮約の精神)と周囲温度`ambient_temperature`との差から、
//! 空間一様な浮力加速度を流体の速度場全体(`u`は不変、`v`のみ、鉛直=y軸)に一括で加える。
//! セルごとの温度差による渦(プルーム)の形成は再現できないが、「暖かい熱源の近くで
//! 流体全体に一様な浮力が働く」という単純化されたシーン(例: 均一に暖められた部屋の
//! 空気循環の粗い近似)には十分な精度で、テスト可能な解析的挙動(一様加速度)を持つ。
//!
//! 重力は`DomainStates::mechanics`(`World`全体で常時有効なドメイン)の
//! 重力場(`sim_mechanics::GravityField`)をそのまま使う —
//! 独自の重力パラメータを持たせると`World`の重力設定と食い違うリスクがあるため。
//!
//! # 重力の向きへの追従(**重力追従増分**)
//!
//! 移行前はスカラー縮約`MechanicsSolver::gravity()`だけを読み、浮力加速度を
//! 格子の`v`成分(=鉛直)にのみ足していた。つまり**重力がワールド`-y`向き**で
//! あることが暗黙の前提だった。
//!
//! ここは一様場に限って向きへ追従する: 浮力加速度をベクトル
//! $\mathbf{a}_b = -\beta(T-T_{amb})\,\mathbf{g}$ として組み、その
//! **x成分を`u`へ、y成分を`v`へ**足す。格子の軸がワールドのx/y軸に対応するのは
//! この2D格子の既定の解釈で、`sim_coupling::GridFluidRigid`がセル座標を
//! ワールド座標の`(pos.x, pos.y)`と直接突き合わせているのと同じ約束である。
//! 重力が既定の`-y`向きならx成分は厳密に0になり、`u`には**一切触れない**
//! ——移行前とビット単位で同じ結果を保つため(`apply`内のコメント参照)。
//!
//! **既知の限界(正直な記録)**:
//!
//! - **非一様な場(`PointSource`)では浮力を出さない**。`GridFluid2D`は
//!   ワールド座標系での位置(原点・セル→ワールドの写像)を持たないので、
//!   位置依存の場を評価する点そのものが存在しない。代表点を発明して
//!   $\mathbf{g}$を1つ選ぶより、効かないことを明示する方を選ぶ
//!   (移行前も`gravity()`が0.0を返して無効だったので、挙動は変わらない)。
//!   剛体の浮力(`sim_coupling::BuoyancyDrag`)はこの制約を持たない
//!   ——剛体はワールド座標の位置を持つから。
//! - **重力のz成分は落ちる**。2D格子に対応する速度成分が無い。
//! - 温度場を持たない縮約(上記「縮約実装の理由」)はそのまま。

use crate::domain_states::{Coupling, CouplingKind, DomainStates};
use sim_core::DomainId;
use sim_mechanics::GravityField;

/// 単一の`ThermalNode`(`thermal_node`インデックス)の温度と周囲温度
/// `ambient_temperature`の差から、格子流体の速度場全体に一様なBoussinesq浮力加速度を
/// 加える(モジュールdoc参照)。
#[derive(Clone)]
pub struct BoussinesqBuoyancy {
    pub thermal_node: usize,
    /// 周囲温度 $T_{amb}$ [K]。
    pub ambient_temperature: f64,
    /// 熱膨張係数 $\beta$ [K⁻¹](設計の目安: 空気は$1/T_{amb}\approx3.4\times10^{-3}$、
    /// docs/11-fluid/02-eulerian-grid.md の表参照)。
    pub thermal_expansion_coefficient: f64,
}

impl Coupling for BoussinesqBuoyancy {
    fn kind(&self) -> CouplingKind {
        CouplingKind::BoussinesqBuoyancy
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Thermal, DomainId::Fluid]
    }

    fn describe(&self) -> String {
        format!(
            "BoussinesqBuoyancy thermal_node[{}] T_amb={}K beta={}",
            self.thermal_node, self.ambient_temperature, self.thermal_expansion_coefficient
        )
    }

    fn referenced_thermal_nodes(&self) -> Vec<usize> {
        vec![self.thermal_node]
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(thermal) = &world.thermal else {
            return;
        };
        let Some(node) = thermal.nodes.get(self.thermal_node) else {
            return;
        };
        let temperature = node.temperature;
        // 一様場だけが「格子全体に効く1つの重力ベクトル」を持つ(モジュールdoc
        // 「既知の限界」——`GridFluid2D`はワールド座標を持たないので、位置依存の
        // 場を評価する点が無い)。`magnitude`と「上」向きに分けて受け取り、
        // 移行前と同じ乗算順序を保つ。
        let Some((up, gravity)) = (match world.mechanics.gravity_field() {
            field @ GravityField::Uniform { .. } => field.up_and_magnitude_at(sim_math::Vec3::ZERO),
            GravityField::PointSource { .. } | GravityField::Zero => None,
        }) else {
            return;
        };
        let Some(grid_fluid) = &mut world.grid_fluid else {
            return;
        };
        // a_b = -beta*(T-T_amb)*g、g = -gravity*up なので
        // a_b = beta*(T-T_amb)*gravity * up
        // (暖かい熱源(T>T_amb)ほど「上」向きに浮力が働く)。
        let accel =
            self.thermal_expansion_coefficient * (temperature - self.ambient_temperature) * gravity;
        // 格子の軸はワールドのx/y軸(モジュールdoc)。**`u`は水平成分が厳密に
        // 0でないときだけ触る**——重力が既定の`-y`向きなら`up.x`は0で、移行前は
        // `u`に一切書き込んでいなかった。`*u += 0.0`でも`-0.0`が`+0.0`へ
        // 変わりうるので、決定論(`state_hash`)を守るには「触らない」ままにする。
        if up.x != 0.0 {
            for u in grid_fluid.u.iter_mut() {
                *u += accel * up.x * dt;
            }
        }
        // `v`は移行前と同じく無条件に更新する(`up.y == 1.0`なら`accel * 1.0`は
        // `accel`そのもので、移行前の`accel_y`と1ビットも変わらない)。
        for v in grid_fluid.v.iter_mut() {
            *v += accel * up.y * dt;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_fluid::GridFluid2D;
    use sim_mechanics::MechanicsSolver;
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// 熱源が周囲より暖かい場合、格子流体の鉛直速度場全体が解析的な浮力加速度
    /// $\beta(T-T_{amb})g$分だけ一様に増加すること(`u`は不変であること)を確認する。
    #[test]
    fn boussinesq_buoyancy_adds_uniform_upward_acceleration_matching_analytic_formula() {
        let ambient = 293.15;
        let node_temp = 313.15; // 熱源が20K暖かい
        let beta = 3.4e-3; // 空気の熱膨張係数の目安
        let gravity = 9.80665;

        let mut thermal = ThermalSolver::new(ambient);
        let node_idx = thermal.add_node(ThermalNode::new(node_temp, 1000.0));
        let mut mechanics = MechanicsSolver::new(gravity);

        let mut fluid = GridFluid2D::new(4, 4, 0.1);
        let u_before = fluid.u.clone();

        let mut coupling = BoussinesqBuoyancy {
            thermal_node: node_idx,
            ambient_temperature: ambient,
            thermal_expansion_coefficient: beta,
        };

        let dt = 0.01;
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, dt);

        let expected_accel = beta * (node_temp - ambient) * gravity;
        let expected_dv = expected_accel * dt;
        for &v in &fluid.v {
            assert!(
                (v - expected_dv).abs() < 1e-12,
                "v={v} expected_dv={expected_dv}"
            );
        }
        assert_eq!(
            fluid.u, u_before,
            "u (horizontal velocity) should be unaffected"
        );
    }

    /// 熱源が周囲と同温なら浮力はゼロ(速度場は変化しない)。
    #[test]
    fn boussinesq_buoyancy_is_zero_when_node_matches_ambient_temperature() {
        let ambient = 293.15;
        let mut thermal = ThermalSolver::new(ambient);
        let node_idx = thermal.add_node(ThermalNode::new(ambient, 1000.0));
        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut fluid = GridFluid2D::new(4, 4, 0.1);

        let mut coupling = BoussinesqBuoyancy {
            thermal_node: node_idx,
            ambient_temperature: ambient,
            thermal_expansion_coefficient: 3.4e-3,
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, 0.01);

        assert!(fluid.v.iter().all(|&v| v == 0.0));
    }

    /// 温度差・重力・格子はそのままに、重力場だけを差し替えて1step適用し、
    /// `(u, v)`の変化を返す(以下3テスト共通のフィクスチャ)。
    fn buoyancy_increment(field: sim_mechanics::GravityField) -> (Vec<f64>, Vec<f64>) {
        let ambient = 293.15;
        let node_temp = 313.15;
        let mut thermal = ThermalSolver::new(ambient);
        let node_idx = thermal.add_node(ThermalNode::new(node_temp, 1000.0));
        let mut mechanics = MechanicsSolver::new(0.0);
        mechanics.set_gravity_field(field);
        let mut fluid = GridFluid2D::new(4, 4, 0.1);

        let mut coupling = BoussinesqBuoyancy {
            thermal_node: node_idx,
            ambient_temperature: ambient,
            thermal_expansion_coefficient: 3.4e-3,
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, 0.01);
        (fluid.u.clone(), fluid.v.clone())
    }

    /// **重力追従増分**: 一様重力を傾けると、Boussinesq浮力も傾いた「上」向きへ
    /// 回る——水平成分は`u`へ、鉛直成分は`v`へ入る(移行前は`v`しか動かず、
    /// 「重力はワールド`-y`向き」が暗黙の前提だった)。
    #[test]
    fn boussinesq_buoyancy_follows_a_tilted_uniform_gravity_into_both_velocity_components() {
        let (ambient, node_temp, beta, gravity) = (293.15, 313.15, 3.4e-3, 9.80665);
        let tilt = 30.0_f64.to_radians();
        let up = sim_math::Vec3::new(tilt.sin(), tilt.cos(), 0.0);
        let (u, v) = buoyancy_increment(sim_mechanics::GravityField::Uniform {
            magnitude: gravity,
            direction: sim_math::Vec3::ZERO - up,
        });

        let magnitude = beta * (node_temp - ambient) * gravity * 0.01;
        for (&u, &v) in u.iter().zip(v.iter()) {
            assert!((u - magnitude * up.x).abs() < 1e-12, "u={u}");
            assert!((v - magnitude * up.y).abs() < 1e-12, "v={v}");
        }
        // 水平成分が実際に立っている(=`u`固定の実装なら破れる)。
        assert!(u[0] > 0.0, "u={:?}", u[0]);
    }

    /// **重力追従増分の既知の限界**: `GridFluid2D`はワールド座標を持たないので、
    /// 位置依存の場(`PointSource`)を評価する点が無い。`Zero`も同じく「上」が
    /// 無い。どちらも浮力を出さない——移行前(`gravity()`が0.0を返して無効)と
    /// 同じ挙動で、**意図した縮退**であることをここで固定する
    /// (剛体側の`BuoyancyDrag`は`PointSource`でも効く、との対比に注意)。
    #[test]
    fn boussinesq_buoyancy_is_inert_in_non_uniform_gravity_fields() {
        for field in [
            sim_mechanics::GravityField::Zero,
            sim_mechanics::GravityField::PointSource {
                center: sim_math::Vec3::new(0.0, -1000.0, 0.0),
                mu: 245.0,
            },
        ] {
            let (u, v) = buoyancy_increment(field);
            assert!(u.iter().all(|&u| u == 0.0), "field={field:?} u={u:?}");
            assert!(v.iter().all(|&v| v == 0.0), "field={field:?} v={v:?}");
        }
        // 一様場に戻せば確かに効く(上のゼロが「たまたま」ではない)。
        let (_, v) = buoyancy_increment(sim_mechanics::GravityField::Uniform {
            magnitude: 9.80665,
            direction: sim_math::Vec3::new(0.0, -1.0, 0.0),
        });
        assert!(v.iter().all(|&v| v > 0.0));
    }
}
