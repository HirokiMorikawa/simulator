//! フレーム階層(floating origin)。設計: docs/20-integration/05-frame-hierarchy.md、
//! docs/00-foundation/02-scale-ladder.md §2.2。
//!
//! **縮約実装の理由**: 設計§8のフェーズ対応は「P1: `FrameId` を剛体状態に導入(全て ROOT の
//! 単一フレーム動作)」「Pα: 複数フレーム・跨ぎ判定・非慣性項・L5 供給の実装」に分かれる。
//! このcrateには複数フレームの木構造・フレーム間変換(§2)・非慣性項の計算(§5)までを実装する。
//! 跨ぎ判定(re-parenting、§3)と接触/拘束の跨ぎ処理(§4)は `World` のブロードフェーズ・
//! アイランド管理(まだ未実装、Phase C)に依存するため、`World` 本体(ワークストリームB)に
//! 持ち越す。§7 の単体テストのうち、跨ぎ判定を必要としない「往復変換」「コリオリ検算」の
//! 2本はこの縮約範囲で検証可能なため実装する。

use crate::FrameId;
use sim_math::{Quat, Transform, Vec3};

/// フレーム。ROOT(`FrameId::ROOT`)は`parent = None`で、`origin_in_parent`等は無視される。
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub id: FrameId,
    pub parent: Option<FrameId>,
    /// 親フレーム内でのこのフレームの原点位置・姿勢・速度・角速度(設計§2)。
    pub origin_in_parent: Vec3,
    pub rotation_in_parent: Quat,
    pub velocity_in_parent: Vec3,
    pub angular_velocity_in_parent: Vec3,
}

impl Frame {
    fn root() -> Frame {
        Frame {
            id: FrameId::ROOT,
            parent: None,
            origin_in_parent: Vec3::ZERO,
            rotation_in_parent: Quat::IDENTITY,
            velocity_in_parent: Vec3::ZERO,
            angular_velocity_in_parent: Vec3::ZERO,
        }
    }

    /// このフレームから親フレームへの剛体変換(親座標 = to_parent.apply_point(このフレームの座標))。
    fn to_parent_transform(self) -> Transform {
        Transform {
            position: self.origin_in_parent,
            rotation: self.rotation_in_parent,
        }
    }
}

/// フレームの木構造(ROOTを根とする、深さ上限は設計§9が既定4とするが実装上は無制限)。
pub struct FrameTree {
    frames: Vec<Frame>,
}

impl Default for FrameTree {
    fn default() -> FrameTree {
        FrameTree::new()
    }
}

impl FrameTree {
    pub fn new() -> FrameTree {
        FrameTree {
            frames: vec![Frame::root()],
        }
    }

    /// 新規フレームを追加する(木構造、閉路なし: 親は既存フレームのみ指定可能なため
    /// 構築時に自動的に閉路が排除される)。
    #[allow(clippy::too_many_arguments)]
    pub fn add_frame(
        &mut self,
        parent: FrameId,
        origin_in_parent: Vec3,
        rotation_in_parent: Quat,
        velocity_in_parent: Vec3,
        angular_velocity_in_parent: Vec3,
    ) -> FrameId {
        assert!(
            self.contains(parent),
            "parent frame {parent:?} does not exist"
        );
        let id = FrameId(self.frames.len() as u32);
        self.frames.push(Frame {
            id,
            parent: Some(parent),
            origin_in_parent,
            rotation_in_parent,
            velocity_in_parent,
            angular_velocity_in_parent,
        });
        id
    }

    fn contains(&self, id: FrameId) -> bool {
        (id.0 as usize) < self.frames.len()
    }

    pub fn frame(&self, id: FrameId) -> &Frame {
        &self.frames[id.0 as usize]
    }

    /// idからROOTまでの祖先チェーン(id自身を先頭に含む)。
    fn ancestor_chain(&self, id: FrameId) -> Vec<FrameId> {
        let mut chain = vec![id];
        let mut cur = id;
        while let Some(p) = self.frame(cur).parent {
            chain.push(p);
            cur = p;
        }
        chain
    }

    /// フレームidの座標系からROOT座標系への変換(設計§2「共通祖先まで上がって下る」の
    /// ROOT特化版。ROOTは全フレームの共通祖先なので常にこの経路で任意の2フレーム間の
    /// 変換も合成できる)。
    pub fn transform_to_root(&self, id: FrameId) -> Transform {
        let chain = self.ancestor_chain(id); // [id, parent, ..., ROOT]
        let mut t = Transform {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        for &fid in chain.iter().rev() {
            if fid == FrameId::ROOT {
                continue;
            }
            t = t.compose(self.frame(fid).to_parent_transform());
        }
        t
    }

    /// フレームfromの座標からフレームtoの座標への変換 $T_{from\to to}$(設計§2)。
    pub fn transform_between(&self, from: FrameId, to: FrameId) -> Transform {
        let from_to_root = self.transform_to_root(from);
        let to_to_root = self.transform_to_root(to);
        to_to_root.inverse().compose(from_to_root)
    }

    /// フレームid内の位置・速度をROOT系での位置・速度に変換する(transport theorem:
    /// 各親への1ホップごとに `parent = origin_in_parent + R*child`、
    /// `v_parent = velocity_in_parent + R*v_child + omega_in_parent × (R*child)` を
    /// idからROOTまで順に適用する)。設計§3.2の跨ぎ変換式の基礎となる一般形。
    pub fn state_to_root(&self, id: FrameId, position: Vec3, velocity: Vec3) -> (Vec3, Vec3) {
        let chain = self.ancestor_chain(id); // [id, parent, ..., ROOT]
        let mut pos = position;
        let mut vel = velocity;
        for &fid in chain.iter() {
            if fid == FrameId::ROOT {
                break;
            }
            let f = self.frame(fid);
            let rotated_pos = f.rotation_in_parent.rotate(pos);
            let rotated_vel = f.rotation_in_parent.rotate(vel);
            let new_pos = f.origin_in_parent + rotated_pos;
            let new_vel = f.velocity_in_parent
                + rotated_vel
                + f.angular_velocity_in_parent.cross(rotated_pos);
            pos = new_pos;
            vel = new_vel;
        }
        (pos, vel)
    }

    /// ROOT系の位置・速度を、フレームid内の位置・速度に変換する(`state_to_root`の逆変換、
    /// ROOTからidまでの祖先鎖を逆向きにたどり各ホップの逆変換を適用する)。
    pub fn state_from_root(&self, id: FrameId, position: Vec3, velocity: Vec3) -> (Vec3, Vec3) {
        let chain = self.ancestor_chain(id); // [id, parent, ..., ROOT]
        let mut pos = position;
        let mut vel = velocity;
        for &fid in chain.iter().rev() {
            if fid == FrameId::ROOT {
                continue;
            }
            let f = self.frame(fid);
            let inv_rotation = f.rotation_in_parent.conjugate();
            let delta_pos = pos - f.origin_in_parent;
            let new_pos = inv_rotation.rotate(delta_pos);
            let delta_vel =
                vel - f.velocity_in_parent - f.angular_velocity_in_parent.cross(delta_pos);
            let new_vel = inv_rotation.rotate(delta_vel);
            pos = new_pos;
            vel = new_vel;
        }
        (pos, vel)
    }

    /// フレームfrom内の位置・速度をフレームto内の位置・速度へ厳密変換する
    /// (設計docs/20-integration/05-frame-hierarchy.md §3.2「跨ぎ時の状態変換」の式、
    /// ROOTを経由する合成として実装)。レジーム切替のAstro⇄Local状態受け渡し
    /// (docs/20-integration/06-regime-switching.md §3.2)が使う基礎変換。
    pub fn transform_state(
        &self,
        from: FrameId,
        to: FrameId,
        position: Vec3,
        velocity: Vec3,
    ) -> (Vec3, Vec3) {
        let (root_pos, root_vel) = self.state_to_root(from, position, velocity);
        self.state_from_root(to, root_pos, root_vel)
    }

    /// フレームidの合成角速度(ROOT系で表現、祖先鎖の角速度を積算)。設計§5の非慣性項計算に使う。
    pub fn angular_velocity_in_root(&self, id: FrameId) -> Vec3 {
        let chain = self.ancestor_chain(id);
        let mut omega = Vec3::ZERO;
        let mut accumulated_rotation_to_root = Quat::IDENTITY;
        // ROOTから順にomegaを積み上げ、各フレームの角速度はそのフレームの親内で表現されて
        // いるため、これまでに蓄積した回転でROOT系へ変換してから加算する。
        for &fid in chain.iter().rev() {
            if fid == FrameId::ROOT {
                continue;
            }
            let f = self.frame(fid);
            omega = omega + accumulated_rotation_to_root.rotate(f.angular_velocity_in_parent);
            accumulated_rotation_to_root = accumulated_rotation_to_root.mul(f.rotation_in_parent);
        }
        omega
    }
}

/// 設計§5の非慣性項(遠心力・コリオリ力・オイラー力)。フレームの角加速度は瞬間値として
/// 呼び出し側が渡す(既定0、静定的な回転フレームでは無視できる)。
pub struct FictitiousForces {
    pub centrifugal: Vec3,
    pub coriolis: Vec3,
    pub euler: Vec3,
}

impl FictitiousForces {
    pub fn total(&self) -> Vec3 {
        self.centrifugal + self.coriolis + self.euler
    }
}

/// フレーム角速度omega・角加速度omega_dot・質量mass・フレーム内位置r・フレーム内速度vから
/// 非慣性力を計算する(設計§5の式をそのまま実装)。
pub fn fictitious_forces(
    mass: f64,
    omega: Vec3,
    omega_dot: Vec3,
    r: Vec3,
    v: Vec3,
) -> FictitiousForces {
    FictitiousForces {
        centrifugal: omega.cross(omega.cross(r)).scale(-mass),
        coriolis: omega.cross(v).scale(-2.0 * mass),
        euler: omega_dot.cross(r).scale(-mass),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 往復変換 $T_{B\to A}\circ T_{A\to B}=\mathrm{id}$(設計§7、abs 1e-12)。
    #[test]
    fn round_trip_transform_between_frames_is_identity() {
        let mut tree = FrameTree::new();
        let a = tree.add_frame(
            FrameId::ROOT,
            Vec3::new(10.0, 0.0, 0.0),
            Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), 0.3),
            Vec3::ZERO,
            Vec3::ZERO,
        );
        let b = tree.add_frame(
            a,
            Vec3::new(0.0, 5.0, -2.0),
            Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), -0.7),
            Vec3::ZERO,
            Vec3::ZERO,
        );

        let a_to_b = tree.transform_between(a, b);
        let b_to_a = tree.transform_between(b, a);
        let round_trip = b_to_a.compose(a_to_b);

        let p = Vec3::new(1.23, -4.56, 7.89);
        let p_round_trip = round_trip.apply_point(p);
        assert!(
            (p_round_trip - p).length() < 1e-12,
            "round trip point mismatch: {p_round_trip:?} vs {p:?}"
        );

        let probe = Vec3::new(1.0, 0.0, 0.0);
        let identity_rotation_diff = round_trip.rotation.rotate(probe) - probe;
        assert!(
            identity_rotation_diff.length() < 1e-12,
            "round trip rotation mismatch: {:?}",
            round_trip.rotation
        );
    }

    /// 位置・速度の状態変換(設計§3.2「跨ぎ時の状態変換」)の往復が恒等であること
    /// (レジーム切替Astro⇄Local状態受け渡しの基礎変換、docs/20-integration/06-regime-switching.md §3.2)。
    /// 親フレームが並進速度・回転角速度の両方を持つ(公転+自転する天体のような)ケースで検証する。
    #[test]
    fn round_trip_state_transform_between_moving_rotating_frames_is_identity() {
        let mut tree = FrameTree::new();
        let planet = tree.add_frame(
            FrameId::ROOT,
            Vec3::new(1.5e8, 0.0, 0.0),
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.4),
            Vec3::new(0.0, 29.8, 0.0), // 公転速度(km/s級、単位は任意でよい)
            Vec3::new(0.0, 0.0, 7.3e-5), // 自転角速度
        );

        let position = Vec3::new(6371.0, 100.0, -50.0);
        let velocity = Vec3::new(0.1, -0.2, 0.05);

        let (root_pos, root_vel) = tree.state_to_root(planet, position, velocity);
        let (back_pos, back_vel) = tree.state_from_root(planet, root_pos, root_vel);

        assert!(
            (back_pos - position).length() < 1e-9 * position.length(),
            "position round trip mismatch: {back_pos:?} vs {position:?}"
        );
        assert!(
            (back_vel - velocity).length() < 1e-9 * velocity.length().max(1.0),
            "velocity round trip mismatch: {back_vel:?} vs {velocity:?}"
        );

        // transform_state(planet, planet, ...) も同じ恒等変換になるはず(ROOT経由の往復)。
        let (identity_pos, identity_vel) = tree.transform_state(planet, planet, position, velocity);
        assert!(
            (identity_pos - position).length() < 1e-9 * position.length(),
            "transform_state self-mapping position mismatch: {identity_pos:?}"
        );
        assert!(
            (identity_vel - velocity).length() < 1e-9 * velocity.length().max(1.0),
            "transform_state self-mapping velocity mismatch: {identity_vel:?}"
        );
    }

    /// コリオリ検算(設計§7): 回転フレーム内の自由粒子(慣性系では等速直線運動)の軌道が、
    /// 慣性系解をフレーム変換したものと一致すること(rel 1e-6)、コリオリ力の仕事が0
    /// (速度に直交するため、abs 1e-12)であることを確認する。
    #[test]
    fn coriolis_matches_inertial_frame_solution_and_does_zero_work() {
        let omega_z = 0.5; // 回転フレームの角速度(ROOT/慣性系のz軸まわり)
        let omega = Vec3::new(0.0, 0.0, omega_z);

        // 慣性系(ROOT)での厳密解: 原点を通り等速直線運動する自由粒子。
        let inertial_pos0 = Vec3::new(2.0, 0.0, 0.0);
        let inertial_vel = Vec3::new(0.0, 1.0, 0.0);
        let inertial_pos_at = |t: f64| inertial_pos0 + inertial_vel.scale(t);

        // 回転フレーム: ROOTのまわりを角速度omegaで回転する(原点は共有、姿勢のみ回転)。
        let rotating_pos_at = |t: f64| -> Vec3 {
            let rot = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), -omega_z * t);
            rot.rotate(inertial_pos_at(t))
        };

        // 回転フレーム内で、非慣性項(遠心力+コリオリ力、フレーム角加速度は0のためオイラー力は0)
        // を含めてRK4で数値積分する(初期速度は回転フレーム内速度の解析式 v = omega×r_rot + rot.rotate(inertial_vel)
        // から厳密に与える — 中心差分による近似は数値誤差の余計な混入源になるため避ける)。
        let dt = 1e-4;
        let mass = 1.0;
        let mut r = rotating_pos_at(0.0);
        // 初期速度は4次精度中心差分(h=1e-6)で求める(解析的な符号規約の取り違えを避ける)。
        let h = 1e-6;
        let mut v = (rotating_pos_at(-2.0 * h) - rotating_pos_at(2.0 * h)).scale(1.0 / (12.0 * h))
            + (rotating_pos_at(h) - rotating_pos_at(-h)).scale(8.0 / (12.0 * h));

        let accel_of = |r: Vec3, v: Vec3| -> Vec3 {
            fictitious_forces(mass, omega, Vec3::ZERO, r, v)
                .total()
                .scale(1.0 / mass)
        };

        let mut coriolis_work = 0.0;
        let steps = 2000; // t in [0, 0.2]
        for _ in 0..steps {
            let forces = fictitious_forces(mass, omega, Vec3::ZERO, r, v);
            coriolis_work += forces.coriolis.dot(v) * dt;

            // 古典的RK4(加速度はr,vのみに依存し陽に時間を含まないため、各段はその場のr,vで評価)。
            let k1_v = accel_of(r, v);
            let k1_r = v;
            let k2_v = accel_of(r + k1_r.scale(dt * 0.5), v + k1_v.scale(dt * 0.5));
            let k2_r = v + k1_v.scale(dt * 0.5);
            let k3_v = accel_of(r + k2_r.scale(dt * 0.5), v + k2_v.scale(dt * 0.5));
            let k3_r = v + k2_v.scale(dt * 0.5);
            let k4_v = accel_of(r + k3_r.scale(dt), v + k3_v.scale(dt));
            let k4_r = v + k3_v.scale(dt);

            v = v + (k1_v + k2_v.scale(2.0) + k3_v.scale(2.0) + k4_v).scale(dt / 6.0);
            r = r + (k1_r + k2_r.scale(2.0) + k3_r.scale(2.0) + k4_r).scale(dt / 6.0);
        }

        let t_final = steps as f64 * dt;
        let expected = rotating_pos_at(t_final);
        let rel_err = (r - expected).length() / expected.length();
        assert!(
            rel_err < 1e-6,
            "rotating-frame trajectory mismatch: r={r:?} expected={expected:?} rel_err={rel_err:e}"
        );
        assert!(
            coriolis_work.abs() < 1e-12,
            "coriolis work should vanish (force perpendicular to velocity): {coriolis_work:e}"
        );
    }
}
