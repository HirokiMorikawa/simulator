//! レジーム切替(Local⇔Astro)。設計: docs/20-integration/06-regime-switching.md。
//!
//! **縮約実装の理由**: 設計の切替プロトコル全体(§3: 切替時刻の量子化・状態受け渡し・
//! 巻き戻し可否)は`World`のスナップショット・コマンドキュー・イベント順序(いずれもPhase C
//! 未実装)に依存する。ここでは`World`本体なしで検証可能な部分 — `TimeRegime`型(設計§2の
//! 定義そのまま)と、状態受け渡し(§3.2)が使う基礎変換(`sim_core::frame::FrameTree::
//! transform_state`、フレーム階層の増分で実装済み)を天体軌道状態に適用したときの保存性
//! 検証(設計§4「状態受け渡しの保存性」)— を実装する。切替時刻の量子化・リプレイ一致・
//! 巻き戻しは`World`本体(ワークストリームB)に持ち越す。

use sim_core::{FrameId, FrameTree};
use sim_math::Vec3;

/// 時間加速の2レジーム(設計§2)。`World`の$\Delta t$はどちらでも変更しない。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TimeRegime {
    /// ローカル物理 + (必要なら)天体をWorld dtに従属させて共進行。
    Local { steps_per_frame: u32 },
    /// 天体のみ。独立時間軸dt_astro(固定)。ローカル物理は凍結。
    Astro { dt_astro: f64, steps_per_frame: u32 },
}

/// Astro→Local状態受け渡し(設計§3.2): 天体状態(軌道位置・速度、ROOT/慣性系)を
/// 突入天体の地表(回転)フレームのローカル座標・速度に厳密変換する。
pub fn astro_to_local_state(
    frames: &FrameTree,
    local_frame: FrameId,
    orbital_position: Vec3,
    orbital_velocity: Vec3,
) -> (Vec3, Vec3) {
    frames.transform_state(
        FrameId::ROOT,
        local_frame,
        orbital_position,
        orbital_velocity,
    )
}

/// Local→Astro状態受け渡し(設計§3.2の逆方向): ローカル状態をROOT/慣性系の軌道状態へ集約する。
pub fn local_to_astro_state(
    frames: &FrameTree,
    local_frame: FrameId,
    local_position: Vec3,
    local_velocity: Vec3,
) -> (Vec3, Vec3) {
    frames.transform_state(local_frame, FrameId::ROOT, local_position, local_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::Quat;

    /// 状態受け渡しの保存性(設計§4): Astro→Local→Astroの往復変換前後で、ROOT換算の
    /// 運動エネルギー・運動量が一致すること(rel 1e-9、設計が明記する基準そのまま)。
    /// 再突入(D37)を模した設定 — 自転+公転する惑星の地表フレームへ、軌道上のカプセルの
    /// 状態(位置・速度)を厳密変換する。
    #[test]
    fn astro_to_local_round_trip_preserves_root_frame_energy_and_momentum() {
        let mut frames = FrameTree::new();
        let earth_surface = frames.add_frame(
            FrameId::ROOT,
            Vec3::new(1.496e8, 0.0, 0.0), // 太陽からの距離(km、公転半径)
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 1.1),
            Vec3::new(0.0, 29.78, 0.0),    // 地球の公転速度(km/s)
            Vec3::new(0.0, 0.0, 7.292e-5), // 地球の自転角速度(rad/s)
        );

        // 再突入直前のカプセルの軌道状態(ROOT/太陽系バリセントリック系、km・km/s単位)。
        let capsule_mass = 5000.0; // kg
        let orbital_position = Vec3::new(1.496e8 + 6528.0, 120.0, -30.0);
        let orbital_velocity = Vec3::new(0.0, 29.78 + 7.8, 0.15);

        let (local_pos, local_vel) =
            astro_to_local_state(&frames, earth_surface, orbital_position, orbital_velocity);
        let (back_pos, back_vel) =
            local_to_astro_state(&frames, earth_surface, local_pos, local_vel);

        let momentum_before = orbital_velocity.scale(capsule_mass);
        let momentum_after = back_vel.scale(capsule_mass);
        let momentum_rel_err =
            (momentum_after - momentum_before).length() / momentum_before.length();
        assert!(
            momentum_rel_err < 1e-9,
            "momentum not preserved across Astro->Local->Astro round trip: rel_err={momentum_rel_err:e}"
        );

        let kinetic_energy = |v: Vec3| 0.5 * capsule_mass * v.length_sq();
        let ke_before = kinetic_energy(orbital_velocity);
        let ke_after = kinetic_energy(back_vel);
        let ke_rel_err = (ke_after - ke_before).abs() / ke_before;
        assert!(
            ke_rel_err < 1e-9,
            "kinetic energy not preserved across Astro->Local->Astro round trip: rel_err={ke_rel_err:e}"
        );

        let pos_rel_err = (back_pos - orbital_position).length() / orbital_position.length();
        assert!(
            pos_rel_err < 1e-9,
            "position not preserved across Astro->Local->Astro round trip: rel_err={pos_rel_err:e}"
        );
    }

    #[test]
    fn time_regime_variants_carry_expected_fields() {
        let local = TimeRegime::Local { steps_per_frame: 2 };
        let astro = TimeRegime::Astro {
            dt_astro: 0.1 * 86400.0,
            steps_per_frame: 1,
        };
        match local {
            TimeRegime::Local { steps_per_frame } => assert_eq!(steps_per_frame, 2),
            _ => panic!("expected Local variant"),
        }
        match astro {
            TimeRegime::Astro {
                dt_astro,
                steps_per_frame,
            } => {
                assert!((dt_astro - 0.1 * 86400.0).abs() < 1e-9);
                assert_eq!(steps_per_frame, 1);
            }
            _ => panic!("expected Astro variant"),
        }
    }
}
