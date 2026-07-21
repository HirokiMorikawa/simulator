//! ジョイント(拘束)。設計: docs/10-mechanics/05-joints-constraints.md。
//!
//! P3 スコープの最小実装: `DistanceJoint`(2点間距離 $|\mathbf{p}_B-\mathbf{p}_A|=L$、
//! 設計 §4.4 表「Distance | 1 | ロープ端点・スプリング」、1行拘束)と
//! `BallJoint`(アンカー一致 $\mathbf{p}_B=\mathbf{p}_A$、設計 §2.1・§4.4 表「Ball | 3 |
//! アンカー一致」、3行拘束)。どちらも `body_b: None` でワールド固定点への接続を表せる —
//! Distance は単振り子(M3/M4、質量無しの棒/紐)、Ball は固定ピボットで自由に回転できる
//! 支点(M10、独楽の歳差)を表現する。Ball の3行は設計 §4.2 が推奨する3×3ブロックソルバ
//! (コレスキー分解)ではなく、ワールド座標系のx/y/z軸に沿った3本の独立スカラー拘束として
//! PGS反復で解く(接触ソルバの摩擦円錐を2本の独立スカラー制約で近似する「箱近似」と同じ
//! 簡略化方針、docs/10-mechanics/04-friction.md §2.1)。
//! Hinge の軸直交拘束行(設計§4.4「+2」)・Slider/Fixed/Wheel・limit・ソフト拘束・
//! 真のブロックソルバは Phase 3 の残りとして未実装。
//!
//! `HingeMotorPd`(設計§4.5 位置サーボ+モーター行)は、上記の軸直交拘束行を持つ正式な
//! Hinge ジョイントとしてではなく、`BallJoint`(アンカー3行のみ)と組み合わせて使う
//! 縮約実装として追加する — 対象の動作(単一平面内の振り子的な関節、
//! docs/20-integration/03-entity-layer.md §7 静的姿勢維持テスト)では重力トルクが
//! ヒンジ軸まわりのみに生じ他の2自由度が励起されないため、軸直交拘束行を省略しても
//! 正しく振る舞う(この前提が崩れる汎用シーンでは正式なHingeジョイントが必要になる)。
//! 設計の「motor行(dθ=ω_target、|λ|≤τ_max·dt)」を、PGSの速度拘束行としてではなく、
//! 軸まわりの角速度をω_targetへ1ステップで近づけるのに必要なトルクをτ_maxでクランプして
//! 直接トルクとして印加する形で実装する(効果は同じ: 無負荷でω_targetに漸近、
//! 過負荷でτ_maxに飽和)。PD自体(ω_target = kp(θ_target-θ) - kd・θ̇)は設計§4.5が
//! 「制御ループはエンティティ層」と定めるが、`sim-entity` crateが未実装のため、この
//! 縮約実装では暫定的に物理モーターと同じ場所(本crate)に置く。

use crate::body::RigidBodySet;
use sim_math::{Quat, Vec3};

/// 設計 §9「ジョイント Baumgarte β = 0.2(接触と同じ)」。
const BAUMGARTE_BETA: f64 = 0.2;
/// 設計 §4.1「反復数も共有(N_v=10)」。
pub const JOINT_VELOCITY_ITERATIONS: u32 = 10;

/// 2点間距離拘束。`body_b = None` はワールド固定(振り子の支点等)を表す。
#[derive(Clone, Copy)]
pub struct DistanceJoint {
    pub body_a: usize,
    /// body_a ローカル座標のアンカー点。
    pub anchor_a: Vec3,
    pub body_b: Option<usize>,
    /// `body_b` が `Some` ならそのローカル座標、`None` ならワールド座標(固定点)。
    pub anchor_b: Vec3,
    /// 維持する距離 L。
    pub length: f64,
}

struct PreparedDistanceJoint {
    body_a: usize,
    body_b: Option<usize>,
    r_a: Vec3,
    r_b: Vec3,
    dir: Vec3,
    mass: f64,
    bias: f64,
}

fn point_velocity(bodies: &RigidBodySet, body: usize, r: Vec3) -> Vec3 {
    bodies.linear_velocity[body] + bodies.angular_velocity[body].cross(r)
}

/// 設計 §2.1 の $K=JM^{-1}J^T$ を単一方向 `dir` に射影したスカラー版
/// (接触ソルバ `contact::effective_mass` と同形)。`body_b=None` はワールド固定
/// (質量無限大、寄与0)として扱う。
fn effective_mass(
    bodies: &RigidBodySet,
    body_a: usize,
    r_a: Vec3,
    body_b: Option<usize>,
    r_b: Vec3,
    dir: Vec3,
) -> f64 {
    let inv_mass_a = bodies.inv_mass[body_a];
    let inv_ia = bodies.inv_inertia_world[body_a];
    let term_a = dir.dot(inv_ia.mul_vec(r_a.cross(dir)).cross(r_a));
    let (inv_mass_b, term_b) = match body_b {
        Some(b) => {
            let inv_ib = bodies.inv_inertia_world[b];
            (
                bodies.inv_mass[b],
                dir.dot(inv_ib.mul_vec(r_b.cross(dir)).cross(r_b)),
            )
        }
        None => (0.0, 0.0),
    };
    let k = inv_mass_a + inv_mass_b + term_a + term_b;
    if k > 0.0 {
        1.0 / k
    } else {
        0.0
    }
}

fn apply_impulse(bodies: &mut RigidBodySet, body: usize, impulse: Vec3, r: Vec3, sign: f64) {
    let inv_mass = bodies.inv_mass[body];
    let inv_i = bodies.inv_inertia_world[body];
    bodies.linear_velocity[body] =
        bodies.linear_velocity[body].addcarry_scaled(impulse, sign * inv_mass);
    let angular_impulse = r.cross(impulse);
    bodies.angular_velocity[body] =
        bodies.angular_velocity[body] + inv_i.mul_vec(angular_impulse).scale(sign);
}

/// body ローカルのアンカー点をワールド座標へ。`(ワールド座標, 重心からのオフセット r)`。
fn world_anchor(bodies: &RigidBodySet, body: usize, anchor_local: Vec3) -> (Vec3, Vec3) {
    let r = bodies.rotation[body].to_mat3().mul_vec(anchor_local);
    (bodies.position[body] + r, r)
}

/// `body_b=None` はワールド固定点(`anchor` をそのままワールド座標として扱う、r=0)。
fn world_anchor_or_fixed(bodies: &RigidBodySet, body: Option<usize>, anchor: Vec3) -> (Vec3, Vec3) {
    match body {
        Some(b) => world_anchor(bodies, b, anchor),
        None => (anchor, Vec3::ZERO),
    }
}

impl DistanceJoint {
    fn prepare(&self, bodies: &RigidBodySet, dt: f64) -> PreparedDistanceJoint {
        let (world_a, r_a) = world_anchor(bodies, self.body_a, self.anchor_a);
        let (world_b, r_b) = world_anchor_or_fixed(bodies, self.body_b, self.anchor_b);
        let delta = world_b - world_a;
        let current_len = delta.length();
        let dir = delta.normalize_or_zero();
        let mass = effective_mass(bodies, self.body_a, r_a, self.body_b, r_b, dir);
        // 拘束誤差 C = |p_B-p_A| - L。位置ドリフトを Baumgarte 速度バイアスで補正する
        // (設計 §9、接触ソルバと異なり split impulse 化していない — Phase 3 の精緻化課題)。
        let bias = BAUMGARTE_BETA / dt * (current_len - self.length);
        PreparedDistanceJoint {
            body_a: self.body_a,
            body_b: self.body_b,
            r_a,
            r_b,
            dir,
            mass,
            bias,
        }
    }
}

fn solve_velocity(p: &PreparedDistanceJoint, bodies: &mut RigidBodySet) {
    let v_a = point_velocity(bodies, p.body_a, p.r_a);
    let v_b = match p.body_b {
        Some(b) => point_velocity(bodies, b, p.r_b),
        None => Vec3::ZERO,
    };
    let c_dot = p.dir.dot(v_b - v_a);
    let lambda = -(c_dot + p.bias) * p.mass;
    let impulse = p.dir.scale(lambda);
    apply_impulse(bodies, p.body_a, impulse, p.r_a, -1.0);
    if let Some(b) = p.body_b {
        apply_impulse(bodies, b, impulse, p.r_b, 1.0);
    }
}

/// ジョイント解決の1ステップ分: 全ジョイントを prepare → velocity iterations(設計 §4.1、
/// 接触と同じ反復数)。処理順は「ジョイント→接触」(設計 §4.1)、呼び出し側
/// (`MechanicsSolver::step`)がその順で呼ぶ。
pub fn resolve_distance(joints: &[DistanceJoint], bodies: &mut RigidBodySet, dt: f64) {
    if joints.is_empty() {
        return;
    }
    let prepared: Vec<PreparedDistanceJoint> =
        joints.iter().map(|j| j.prepare(bodies, dt)).collect();
    for _ in 0..JOINT_VELOCITY_ITERATIONS {
        for p in &prepared {
            solve_velocity(p, bodies);
        }
    }
}

/// アンカー一致拘束(設計 §2.1)。`body_b = None` はワールド固定点(独楽の支点等、
/// M10)を表す — 剛体はその点を中心に自由に回転できる。
#[derive(Clone, Copy)]
pub struct BallJoint {
    pub body_a: usize,
    /// body_a ローカル座標のアンカー点。
    pub anchor_a: Vec3,
    pub body_b: Option<usize>,
    /// `body_b` が `Some` ならそのローカル座標、`None` ならワールド座標(固定点)。
    pub anchor_b: Vec3,
}

struct PreparedBallAxis {
    dir: Vec3,
    mass: f64,
    bias: f64,
}

struct PreparedBallJoint {
    body_a: usize,
    body_b: Option<usize>,
    r_a: Vec3,
    r_b: Vec3,
    axes: [PreparedBallAxis; 3],
}

impl BallJoint {
    fn prepare(&self, bodies: &RigidBodySet, dt: f64) -> PreparedBallJoint {
        let (world_a, r_a) = world_anchor(bodies, self.body_a, self.anchor_a);
        let (world_b, r_b) = world_anchor_or_fixed(bodies, self.body_b, self.anchor_b);
        // 拘束誤差(ズレ)C = p_B - p_A。位置ドリフトを Baumgarte 速度バイアスで補正する
        // (設計 §9)。
        let c = world_b - world_a;
        let dirs = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let axes = dirs.map(|dir| {
            let mass = effective_mass(bodies, self.body_a, r_a, self.body_b, r_b, dir);
            let bias = BAUMGARTE_BETA / dt * c.dot(dir);
            PreparedBallAxis { dir, mass, bias }
        });
        PreparedBallJoint {
            body_a: self.body_a,
            body_b: self.body_b,
            r_a,
            r_b,
            axes,
        }
    }
}

fn solve_velocity_ball(p: &PreparedBallJoint, bodies: &mut RigidBodySet) {
    for axis in &p.axes {
        let v_a = point_velocity(bodies, p.body_a, p.r_a);
        let v_b = match p.body_b {
            Some(b) => point_velocity(bodies, b, p.r_b),
            None => Vec3::ZERO,
        };
        let c_dot = axis.dir.dot(v_b - v_a);
        let lambda = -(c_dot + axis.bias) * axis.mass;
        let impulse = axis.dir.scale(lambda);
        apply_impulse(bodies, p.body_a, impulse, p.r_a, -1.0);
        if let Some(b) = p.body_b {
            apply_impulse(bodies, b, impulse, p.r_b, 1.0);
        }
    }
}

/// `resolve_distance` の Ball ジョイント版。
pub fn resolve_ball(joints: &[BallJoint], bodies: &mut RigidBodySet, dt: f64) {
    if joints.is_empty() {
        return;
    }
    let prepared: Vec<PreparedBallJoint> = joints.iter().map(|j| j.prepare(bodies, dt)).collect();
    for _ in 0..JOINT_VELOCITY_ITERATIONS {
        for p in &prepared {
            solve_velocity_ball(p, bodies);
        }
    }
}

/// PD 位置サーボ付きヒンジモーター(設計§4.5、モジュールdocの縮約理由参照)。
/// ワールド固定軸まわりの単一自由度を、`BallJoint`(アンカー)と組み合わせて表現する。
#[derive(Clone, Copy)]
pub struct HingeMotorPd {
    pub body: usize,
    /// ヒンジ軸(ワールド座標、固定、単位ベクトル)。
    pub axis: Vec3,
    /// 生成時点の`body`の姿勢(角度0の基準)。
    pub reference_rotation: Quat,
    pub theta_target: f64,
    pub kp: f64,
    pub kd: f64,
    pub torque_max: f64,
}

impl HingeMotorPd {
    /// 基準姿勢からの、軸まわりの相対回転角(swing-twist分解の簡略版 — 回転が純粋に
    /// 軸まわりである前提、モジュールdoc参照)。
    pub fn measure_angle(&self, bodies: &RigidBodySet) -> f64 {
        let q_rel = bodies.rotation[self.body].mul(self.reference_rotation.conjugate());
        let vector_part = Vec3::new(q_rel.x, q_rel.y, q_rel.z);
        2.0 * vector_part.dot(self.axis).atan2(q_rel.w)
    }

    /// PD制御(設計§4.5: ω_target = kp(θ_target-θ) - kd・θ̇)でトルクを計算し、
    /// `torque_accum`に加算する。トルクは1ステップでω_targetへ到達するのに必要な値を
    /// τ_maxでクランプ(設計の「motor行: |λ|≤τ_max・dt」と同じ飽和則)して印加する。
    /// 印加した実際のトルク(軸成分)を返す(仕事の計上に使える)。
    pub fn apply(&self, bodies: &mut RigidBodySet, dt: f64) -> f64 {
        let theta = self.measure_angle(bodies);
        let omega_axis = bodies.angular_velocity[self.body].dot(self.axis);
        let omega_target = self.kp * (self.theta_target - theta) - self.kd * omega_axis;

        let inv_inertia = bodies.inv_inertia_world[self.body];
        let inv_inertia_axis = self.axis.dot(inv_inertia.mul_vec(self.axis));
        let desired_torque = if inv_inertia_axis > 0.0 {
            (omega_target - omega_axis) / (inv_inertia_axis * dt)
        } else {
            0.0
        };
        let torque = desired_torque.clamp(-self.torque_max, self.torque_max);

        bodies.torque_accum[self.body] = bodies.torque_accum[self.body] + self.axis.scale(torque);
        torque
    }
}

/// `HingeMotorPd`一覧を全て`apply`する。
pub fn apply_hinge_motors(motors: &[HingeMotorPd], bodies: &mut RigidBodySet, dt: f64) {
    for motor in motors {
        motor.apply(bodies, dt);
    }
}
