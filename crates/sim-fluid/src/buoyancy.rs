//! 自由表面・浮力(集中定数モデル)。設計: docs/11-fluid/04-free-surface-buoyancy.md。
//!
//! P1 スコープ: 直立姿勢(回転無視)の直方体のみ — F4(立方体喫水)・F5(浮体上下振動)は
//! いずれも直立浮体シナリオのため十分。一般姿勢の凸多面体切断(§4 の表)・球冠体積・
//! 水中抗力(§4 の `buoyancy_step` 内 `F_d`)は Phase 3 に拡張する。
//!
//! # 水域の一般化(`StaticWaterRegion` → `FluidRegion`)
//!
//! 移行前は「無限に広い静止水面」1枚(`StaticWaterRegion`)だけを表せた。設計 §3 が
//! 挙げる境界(AABB)・水温をこの型が持たなかったためで、シーンJSON側の
//! `fluids`も同じ縮約に縛られていた(`sim_world::scenario`モジュールdoc)。
//!
//! `FluidRegion`はここを3点だけ広げる:
//!
//! 1. **形状**(`FluidShape`): 移行前と同じ水平半空間(`HalfSpace`)に加え、
//!    軸並行直方体で横に閉じた水域(`Aabb`)を表せる。水槽・プールのように
//!    「そこにしか水が無い」配置を書けるようにするための最小の追加であり、
//!    球・任意凸形状は対象外(必要になった時点で変種を足す)。
//! 2. **温度**(`temperature`): 水温[K]を任意で持てる。**熱ドメインへの結合は
//!    まだ配線していない**(`FluidRegion::temperature`のdoc参照)。
//! 3. **複数領域**: 領域を持つ側(`sim_mechanics::MechanicsSolver::fluids`)が
//!    `Vec`になった。どの領域が効くかの決着規則はそちらのdocに書く。
//!
//! **浮力の式そのものは一切変えていない**——`FluidShape`が決めるのは
//! 「その領域が**どこに**効くか」だけで、水面下体積(`submerged_box_axis_aligned`)も
//! 力(`buoyancy_force`)も移行前と同一である。したがって`HalfSpace`の`FluidRegion`は
//! 移行前の`StaticWaterRegion`とビット単位で同じ挙動になる。

use sim_math::Vec3;

/// 流体領域の広がり(モジュールdoc「水域の一般化」参照)。
///
/// **`sim_mechanics::Aabb`を使わない理由**: `sim-mechanics`は`sim-fluid`に依存する
/// 側なので、こちらから参照すると依存が循環する。`min`/`max`の2点という中身は同じ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FluidShape {
    /// 水平半空間(無限に広い静止水面)。**移行前の`StaticWaterRegion`と同一**で、
    /// 「領域内か」の判定を一切行わない(`contains`は常に`true`)——水面より上に
    /// あるかどうかは浮力の式(`submerged_box_axis_aligned`が返す水面下体積)が
    /// 判断する、という移行前の役割分担をそのまま保つため。ここで
    /// `point.y <= water_level`のような判定を足すと、**水面をまたいで浮いている
    /// 剛体**(重心が水面より上にある浮体、D6がまさにそれ)が領域外と見なされて
    /// 浮力を失う。
    HalfSpace,
    /// 軸並行直方体(水槽・プール)。`contains`は基準点の**厳密な内外判定**。
    ///
    /// **既知の限界(正直な記録)**: 判定が基準点1点の内外なので、`max.y`を
    /// 水面(`water_level`)と同じ高さに置くと、浮上して重心が水面上へ出た剛体が
    /// 領域外に落ちて浮力を失い、沈んで戻ってはまた浮く、という不連続な
    /// チャタリングを起こす。**`max.y`は想定される浮上高さより上**(水槽の縁)に
    /// 取ること。剛体の広がりを考慮した部分的な内外判定(領域と剛体の交差体積)は
    /// 浮力の式そのものへの踏み込みになるため対象外とする。
    Aabb { min: Vec3, max: Vec3 },
}

/// 流体領域。設計 §3 の `StaticWaterRegion` の一般化(モジュールdoc「水域の一般化」参照)。
///
/// 一様流(設計 §3 の流速)は引き続き対象外——`sim_fluid`側に静止水域中の
/// 相対流速を使う式が無く、持たせても読む先が無いため。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidRegion {
    /// 自由表面の高さ(ワールドy)。浮力の式が読む唯一の幾何量で、
    /// `shape`が`Aabb`でも意味は変わらない(水槽が満水でない状態を書ける)。
    pub water_level: f64,
    pub density: f64,
    /// 領域の広がり(既定は移行前と同じ`HalfSpace`)。
    pub shape: FluidShape,
    /// 水温 [K]。`None`(既定)なら温度を持たない。
    ///
    /// **正直な限界**: この値を読む結合はまだ**存在しない**。熱ドメインとの
    /// 対流結合(`sim_coupling::ConvectionLink`)は流体側も`ThermalNode`の
    /// index(`fluid_node`)で受け取る設計で、生の流体温度を熱源にする経路が
    /// 無いためである。ここは「シーンJSONで書けて、`World`から引けて、往復する」
    /// までを用意するデータ側の一歩で、実際に熱を動かすには
    /// `ConvectionLink`側へ流体領域を熱源に取る変種を足す必要がある(後続増分)。
    /// 引くには`sim_mechanics::MechanicsSolver::fluid_temperature_at`を使う。
    pub temperature: Option<f64>,
}

impl FluidRegion {
    /// 水平半空間の水域(**移行前の`StaticWaterRegion::new`と完全に同一**)。
    pub fn new(water_level: f64, density: f64) -> FluidRegion {
        FluidRegion {
            water_level,
            density,
            shape: FluidShape::HalfSpace,
            temperature: None,
        }
    }

    /// AABBで境界づけられた水域。`water_level`は自由表面の高さで、`max.y`とは
    /// 独立に与える(満水でない水槽を書けるようにするため)。
    pub fn aabb(min: Vec3, max: Vec3, water_level: f64, density: f64) -> FluidRegion {
        FluidRegion {
            water_level,
            density,
            shape: FluidShape::Aabb { min, max },
            temperature: None,
        }
    }

    /// 水温を与えた同じ領域を返す(`temperature`のdocの限界に注意)。
    pub fn with_temperature(self, temperature: f64) -> FluidRegion {
        FluidRegion {
            temperature: Some(temperature),
            ..self
        }
    }

    /// 基準点`point`がこの領域に属するか(`FluidShape`の各変種のdoc参照)。
    /// `HalfSpace`では常に`true`——判定を持たないのが移行前の挙動だから。
    pub fn contains(&self, point: Vec3) -> bool {
        match self.shape {
            FluidShape::HalfSpace => true,
            FluidShape::Aabb { min, max } => {
                point.x >= min.x
                    && point.x <= max.x
                    && point.y >= min.y
                    && point.y <= max.y
                    && point.z >= min.z
                    && point.z <= max.z
            }
        }
    }
}

/// 静水圧。設計 §2.1: p = p0 + ρ g d(d: 深さ、水面下で正)。F6。
pub fn hydrostatic_pressure(region: &FluidRegion, depth: f64, gravity: f64) -> f64 {
    region.density * gravity * depth.max(0.0)
}

/// 直立姿勢(ローカル+Y=ワールド+Y)の直方体の水面下体積と浮心。設計 §4 の一般姿勢
/// 切断アルゴリズムの直立特殊ケース(回転による姿勢依存は P1 では扱わない、モジュール冒頭注記)。
/// 戻り値: (V_sub, 浮心のワールド座標)。水面下体積が 0 なら浮心は無意味(body 中心を返す)。
pub fn submerged_box_axis_aligned(
    center: Vec3,
    half_extents: Vec3,
    water_level: f64,
) -> (f64, Vec3) {
    let bottom = center.y - half_extents.y;
    let top = center.y + half_extents.y;
    let submerged_top = water_level.min(top);
    let h_sub = (submerged_top - bottom).clamp(0.0, 2.0 * half_extents.y);
    if h_sub <= 0.0 {
        return (0.0, center);
    }
    let base_area = 4.0 * half_extents.x * half_extents.z;
    let volume = base_area * h_sub;
    let centroid = Vec3::new(center.x, bottom + h_sub * 0.5, center.z);
    (volume, centroid)
}

/// アルキメデスの浮力。設計 §2.1: F_b = -ρ_f g V_sub(上向き、鉛直成分のみ)。
pub fn buoyancy_force(volume_submerged: f64, fluid_density: f64, gravity: f64) -> Vec3 {
    Vec3::new(0.0, fluid_density * gravity * volume_submerged, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// F6: 静水圧 p=ρgh(代数検算、docs/21-verification/01-analytic-tests.md F6)。
    #[test]
    fn f6_hydrostatic_pressure_matches_rho_g_h() {
        let region = FluidRegion::new(0.0, 998.2);
        let p = hydrostatic_pressure(&region, 2.0, 9.80665);
        assert!((p - 998.2 * 9.80665 * 2.0).abs() < 1e-9);
    }

    #[test]
    fn pressure_above_surface_is_zero() {
        let region = FluidRegion::new(0.0, 998.2);
        let p = hydrostatic_pressure(&region, -1.0, 9.80665);
        assert_eq!(p, 0.0);
    }

    /// F4 相当の体積検算: 密度比 r の一辺 a の立方体は水面下 r*a まで沈む
    /// (docs/11-fluid/04-free-surface-buoyancy.md §2.2)。V_sub は底面積×喫水深に一致する。
    #[test]
    fn submerged_volume_matches_waterline_depth() {
        let half = 0.5; // 一辺 1m
        let center = Vec3::new(0.0, -0.2, 0.0); // 喫水 0.5m相当まで沈める配置
        let (v, c) = submerged_box_axis_aligned(center, Vec3::new(half, half, half), 0.3);
        // bottom = -0.7, water_level=0.3 -> h_sub = 0.3-(-0.7) = 1.0 (全没)
        assert!((v - 1.0).abs() < 1e-12, "v={v}");
        assert!((c.y - (-0.2)).abs() < 1e-12);
    }

    #[test]
    fn fully_dry_box_has_zero_submerged_volume() {
        let (v, _) =
            submerged_box_axis_aligned(Vec3::new(0.0, 10.0, 0.0), Vec3::new(1.0, 1.0, 1.0), 0.0);
        assert_eq!(v, 0.0);
    }

    /// **アルキメデスの原理の閉形式解**(浮力が任意の重力場に追従するようになる
    /// 将来の変更に備えた基準点)。質量 $m$・水線面積 $A$ の直立直方体の釣り合い
    /// 喫水は $\rho_f\,g\,A\,d = m\,g$ を $d$ について解いた
    /// $d = m/(\rho_f A)$ で、**$g$ には依存しない**(両辺から消える)。
    ///
    /// ここでは `submerged_box_axis_aligned` が返す水面下体積と `buoyancy_force` が
    /// この閉形式の喫水で厳密に重量と釣り合うことを、代数計算だけで確認する
    /// (数値積分を挟まないので許容誤差は倍精度の丸め誤差ぶんの 1e-12(相対))。
    /// あわせて $g$ を変えても釣り合い喫水が動かないことも見る。
    ///
    /// **前提の明示**: このモデルは水面を「ワールドy座標の水平面」として定義し、
    /// 浮力を常に`+y`向きに出す(モジュール冒頭注記・`buoyancy_force`の実装)。
    /// つまり「重力の向き=ワールド-y」が暗黙の前提であり、浮力が重力場の局所的な
    /// 向きへ追従するようになったら、この前提は保証されなくなる——その時点で
    /// 本テストには「重力が-y向きの場合に限る」旨の注記を足す必要がある。
    #[test]
    fn equilibrium_draft_matches_archimedes_closed_form() {
        let water_density = 998.2;
        let half = Vec3::new(0.5, 0.5, 0.5);
        let waterline_area = 4.0 * half.x * half.z;
        let water_level = 0.0;

        for &ratio in &[0.3, 0.6, 0.9] {
            let mass = ratio * water_density * 8.0 * half.x * half.y * half.z;
            // ρ_f g A d = m g  ⇔  d = m / (ρ_f A)。
            let draft = mass / (water_density * waterline_area);
            let center_y = water_level - draft + half.y;

            for &gravity in &[9.80665, 1.62, 24.79] {
                let (v_sub, centroid) =
                    submerged_box_axis_aligned(Vec3::new(0.0, center_y, 0.0), half, water_level);
                assert!(
                    (v_sub - waterline_area * draft).abs() / v_sub < 1e-12,
                    "V_sub = A·d のはず: v_sub={v_sub} A·d={}",
                    waterline_area * draft
                );
                // 浮心は水面下部分の中心(重量との釣り合いには効かないが、
                // 姿勢に効くので位置も固定しておく)。
                let expected_centroid_y = center_y - half.y + 0.5 * draft;
                assert!((centroid.y - expected_centroid_y).abs() < 1e-12);

                let buoyancy = buoyancy_force(v_sub, water_density, gravity).y;
                let weight = mass * gravity;
                assert!(
                    (buoyancy - weight).abs() / weight < 1e-12,
                    "釣り合い喫水では浮力と重量が厳密に一致する(gには依存しない): \
                     ratio={ratio} gravity={gravity} buoyancy={buoyancy} weight={weight}"
                );
            }
        }
    }

    /// `HalfSpace`は**内外判定を持たない**(常に`true`)。移行前の
    /// `StaticWaterRegion`が判定そのものを持っていなかったことの固定
    /// ——ここが`point.y <= water_level`になると、水面をまたいで浮いている
    /// 浮体(D6)が領域外になって浮力を失う(`FluidShape::HalfSpace`のdoc)。
    #[test]
    fn half_space_region_contains_every_point_including_those_above_the_waterline() {
        let region = FluidRegion::new(0.0, 998.2);
        assert!(region.contains(Vec3::new(0.0, -5.0, 0.0)));
        assert!(region.contains(Vec3::new(1e6, 5.0, -1e6)));
        assert_eq!(region.shape, FluidShape::HalfSpace);
        assert_eq!(region.temperature, None);
    }

    /// `Aabb`は基準点の厳密な内外判定(境界上は内側)。
    #[test]
    fn aabb_region_contains_only_points_inside_the_box() {
        let region = FluidRegion::aabb(
            Vec3::new(-1.0, -2.0, -1.0),
            Vec3::new(1.0, 0.5, 1.0),
            0.0,
            998.2,
        );
        assert!(region.contains(Vec3::new(0.0, -1.0, 0.0)));
        // 境界上は内側に含める(閉区間)。
        assert!(region.contains(Vec3::new(-1.0, 0.5, 1.0)));
        // 横に外れる: 「水面下の高さ」でも領域外。
        assert!(!region.contains(Vec3::new(5.0, -1.0, 0.0)));
        assert!(!region.contains(Vec3::new(0.0, -1.0, -3.0)));
        // 上下に外れる。
        assert!(!region.contains(Vec3::new(0.0, 1.0, 0.0)));
        assert!(!region.contains(Vec3::new(0.0, -3.0, 0.0)));
    }

    /// `water_level`は`max.y`とは独立(満水でない水槽を書ける)。
    /// 水温は`with_temperature`で足しても他のフィールドを変えない。
    #[test]
    fn aabb_region_keeps_water_level_and_temperature_independent_of_its_bounds() {
        let region = FluidRegion::aabb(
            Vec3::new(-1.0, -2.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            -0.5, // 縁(max.y=1.0)より下の水面 = 満水ではない水槽
            998.2,
        )
        .with_temperature(280.0);
        assert_eq!(region.water_level, -0.5);
        assert_eq!(region.temperature, Some(280.0));
        assert_eq!(
            region.shape,
            FluidShape::Aabb {
                min: Vec3::new(-1.0, -2.0, -1.0),
                max: Vec3::new(1.0, 1.0, 1.0),
            }
        );
        // 水面より上・縁より下の点も「領域内」——浮力を出すかは水面下体積が決める。
        assert!(region.contains(Vec3::new(0.0, 0.8, 0.0)));
        let (v_sub, _) = submerged_box_axis_aligned(
            Vec3::new(0.0, 0.8, 0.0),
            Vec3::new(0.1, 0.1, 0.1),
            region.water_level,
        );
        assert_eq!(v_sub, 0.0);
    }

    #[test]
    fn partial_submersion_volume_is_base_area_times_depth() {
        let half = Vec3::new(0.5, 0.5, 0.5);
        // center.y=0 -> bottom=-0.5, top=0.5, water_level=0.0 -> h_sub=0.5
        let (v, c) = submerged_box_axis_aligned(Vec3::ZERO, half, 0.0);
        assert!((v - (1.0 * 0.5)).abs() < 1e-12, "v={v}");
        assert!((c.y - (-0.25)).abs() < 1e-12);
    }
}
