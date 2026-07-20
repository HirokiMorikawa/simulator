//! 自由表面・浮力(集中定数モデル)。設計: docs/11-fluid/04-free-surface-buoyancy.md。
//!
//! P1 スコープ: 直立姿勢(回転無視)の直方体のみ — F4(立方体喫水)・F5(浮体上下振動)は
//! いずれも直立浮体シナリオのため十分。一般姿勢の凸多面体切断(§4 の表)・球冠体積・
//! 水中抗力(§4 の `buoyancy_step` 内 `F_d`)・水域境界(AABB)は Phase 3 に拡張する。

use sim_math::Vec3;

/// 静的水域。設計 §3 の `StaticWaterRegion` から、P1 では境界(aabb)・一様流を省き
/// 「無限に広い静止水面」に単純化する(境界付き水域・流れは Phase 3)。
#[derive(Clone, Copy, Debug)]
pub struct StaticWaterRegion {
    pub water_level: f64,
    pub density: f64,
}

impl StaticWaterRegion {
    pub fn new(water_level: f64, density: f64) -> StaticWaterRegion {
        StaticWaterRegion {
            water_level,
            density,
        }
    }
}

/// 静水圧。設計 §2.1: p = p0 + ρ g d(d: 深さ、水面下で正)。F6。
pub fn hydrostatic_pressure(region: &StaticWaterRegion, depth: f64, gravity: f64) -> f64 {
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
        let region = StaticWaterRegion::new(0.0, 998.2);
        let p = hydrostatic_pressure(&region, 2.0, 9.80665);
        assert!((p - 998.2 * 9.80665 * 2.0).abs() < 1e-9);
    }

    #[test]
    fn pressure_above_surface_is_zero() {
        let region = StaticWaterRegion::new(0.0, 998.2);
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

    #[test]
    fn partial_submersion_volume_is_base_area_times_depth() {
        let half = Vec3::new(0.5, 0.5, 0.5);
        // center.y=0 -> bottom=-0.5, top=0.5, water_level=0.0 -> h_sub=0.5
        let (v, c) = submerged_box_axis_aligned(Vec3::ZERO, half, 0.0);
        assert!((v - (1.0 * 0.5)).abs() < 1e-12, "v={v}");
        assert!((c.y - (-0.25)).abs() < 1e-12);
    }
}
