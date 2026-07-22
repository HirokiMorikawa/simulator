//! ヘッドレスデモランナー(設計docs/21-verification/03-demo-scenarios.md「TDDでの位置づけ」
//! §8「ヘッドレスランナー」)。デモ = シーン + 合格基準。「合格」の定義は「合格基準の
//! ヘッドレステストGreen **+ 目視チェック**」(同docs §7冒頭)であり、目視チェックは
//! フロントエンド(ワークストリームD、現時点で未着手)が無いと行えない。本モジュールは
//! 前半(ヘッドレステストGreen)のみを先に確立する — 設計が明記するとおり
//! 「フロントエンドの視覚UIが無くてもヘッドレスで合格判定できる」ため、
//! ワークストリームDの着手前に進められる部分。
//!
//! **縮約実装の理由**: `Scenario`/`from_scenario`(シーンJSON)は現時点で`linear_velocity`
//! (初速)・`atmosphere`(抗力)フィールドを持たない縮約スキーマ(`scenario`モジュールdoc
//! 参照)のため、初速や抗力比較が必要なデモ(D2等)はJSON経由ではなく`World`公開API
//! (`create_body`・`mechanics_mut()`)を直接使って構築する。D1–D39のうち、まず
//! Phase 1スモーク(既存の解析解テストとほぼ1対1対応、新規物理実装が不要なもの)から
//! D1(落下時計)・D2(弾道)の2本を実装する。残りは後続増分。

#[cfg(test)]
mod tests {
    use crate::{World, WorldOptions};
    use sim_math::Vec3;
    use sim_mechanics::{DragModel, RigidBodyDesc, Shape};

    fn foam_material(world: &mut World, name: &'static str) -> sim_core::MaterialId {
        world.materials_mut().push(sim_core::Material {
            name,
            density: 30.0,
            friction: 0.3,
            restitution: 0.3,
            youngs_modulus: None,
            specific_heat: 1300.0,
            conductivity: 0.03,
            emissivity: 0.9,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "test fixture",
            uncertainty: 0.0,
        })
    }

    /// D1 落下時計(docs/21-verification/03-demo-scenarios.md Phase 1表)。
    /// 「高さ可変の球の落下。ストップウォッチと予測式を並記」「合格基準: M1。
    /// 空気抵抗ON/OFF差」。M1自体(自由落下の到達時刻)は`sim-mechanics`の専用解析解
    /// テストで既に検証済みのため、ここではデモ全体の組み立て(`World`経由)+
    /// デモ固有の追加合格基準(抗力ON/OFFで到達時刻が有意に変わる)を確認する。
    #[test]
    fn d1_falling_clock_matches_free_fall_time_and_shows_drag_on_off_difference() {
        let height = 20.0;
        let radius = 0.3;
        let dt = 1.0 / 240.0; // 着地時刻をサブdt精度で捉えるため既定より細かく

        let time_to_ground = |with_drag: bool| -> f64 {
            let mut world = World::new(WorldOptions {
                dt,
                ..WorldOptions::default()
            });
            let material = if with_drag {
                foam_material(&mut world, "test-d1-foam")
            } else {
                world.materials().find_by_name("鋼(炭素鋼)").unwrap()
            };
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, material);
            desc.transform.position = Vec3::new(0.0, height, 0.0);
            if with_drag {
                desc.drag = DragModel::Sphere { radius };
            }
            let body = world.create_body(desc);
            if with_drag {
                world.mechanics_mut().atmosphere =
                    Some(sim_fluid::Atmosphere::still(1.225, 1.81e-5));
            }

            let mut t = 0.0;
            loop {
                world.step();
                t += dt;
                let y = world.body_position(body).unwrap().y;
                if y <= radius || t > 10.0 {
                    return t;
                }
            }
        };

        let t_vacuum = time_to_ground(false);
        let analytic = (2.0 * height / 9.80665_f64).sqrt();
        let rel_err = (t_vacuum - analytic).abs() / analytic;
        assert!(
            rel_err < 0.01,
            "M1: t_vacuum={t_vacuum} analytic={analytic} rel_err={rel_err:.4}"
        );

        let t_drag = time_to_ground(true);
        assert!(
            t_drag > t_vacuum * 1.02,
            "drag should measurably slow the fall (D1 pass criterion): t_vacuum={t_vacuum} t_drag={t_drag}"
        );
    }

    /// D2 弾道(同docs Phase 1表)。「角度・初速可変の投射。真空side-by-side・解析軌道の
    /// 補助線」「合格基準: M2, F1」。M2(45°最大到達距離)は専用解析解テストで既に検証
    /// 済みのため、ここではデモ全体の組み立て(`World`経由、初速を持つ剛体)+ 45°最大
    /// 到達距離の式一致(真空側)と、抗力ありでは到達距離が真空側より短くなる
    /// (F1、side-by-side比較の定性的な合格基準)ことを確認する。
    #[test]
    fn d2_ballistic_range_matches_45_degree_formula_and_drag_shortens_range() {
        let dt = 1.0 / 240.0;
        let v0 = 20.0;
        let radius = 0.1;
        let angle = std::f64::consts::FRAC_PI_4; // 45°(最大到達距離)
        let velocity = Vec3::new(v0 * angle.cos(), v0 * angle.sin(), 0.0);

        let range = |with_drag: bool| -> f64 {
            let mut world = World::new(WorldOptions {
                dt,
                ..WorldOptions::default()
            });
            let material = if with_drag {
                foam_material(&mut world, "test-d2-foam")
            } else {
                world.materials().find_by_name("鋼(炭素鋼)").unwrap()
            };
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, material);
            desc.linear_velocity = velocity;
            if with_drag {
                desc.drag = DragModel::Sphere { radius };
            }
            let body = world.create_body(desc);
            if with_drag {
                world.mechanics_mut().atmosphere =
                    Some(sim_fluid::Atmosphere::still(1.225, 1.81e-5));
            }

            loop {
                world.step();
                let pos = world.body_position(body).unwrap();
                if pos.y <= radius && world.time() > dt * 2.0 {
                    return pos.x;
                }
                if world.time() > 20.0 {
                    return pos.x;
                }
            }
        };

        let range_vacuum = range(false);
        let analytic = v0 * v0 / 9.80665;
        let rel_err = (range_vacuum - analytic).abs() / analytic;
        assert!(
            rel_err < 0.02,
            "M2: range_vacuum={range_vacuum} analytic={analytic} rel_err={rel_err:.4}"
        );

        let range_drag = range(true);
        assert!(
            range_drag < range_vacuum * 0.98,
            "F1: drag should shorten the range relative to the vacuum trajectory (D2 side-by-side pass criterion): range_vacuum={range_vacuum} range_drag={range_drag}"
        );
    }
}
