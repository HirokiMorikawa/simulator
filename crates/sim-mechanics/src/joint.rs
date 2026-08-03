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
//! **群4で `Wheel`・`limit`・ソフト拘束を実装した**(それまで「Phase 3 の残りとして
//! 未実装」と書いていた4項目のうち3つ)。`SoftParams`(設計§4.3)がばねダンパを
//! 拘束ソルバの中で解けるようにし、それを土台に `WheelJoint`(サス+駆動+操舵)を
//! 作った——D24(車の実験場)が「新規物理待ちでスコープ外」だった原因が
//! ここに無かったことである。`HingeMotorPd` には角度制限(設計§4.4表「+ limit」、
//! 接触と同じ片側クランプ)を足した。
//! **残るのは Hinge の軸直交拘束行(§4.4「+2」)・Fixed・真のブロックソルバ**。
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
//!
//! `SliderJoint`(設計§4.4表「Slider | 5 | 軸直交並進2 + 相対回転固定3」)は、軸直交
//! 並進2行を`BallJoint`と同じ箱近似(ワールド座標軸沿いの独立スカラー拘束)で、相対回転
//! 固定3行を新設の`relative_rotation_error`(生成時の相対姿勢を基準に取り、クォータニオン
//! のベクトル部を誤差として使う小角近似)で解く。断熱圧縮(`PistonGas`結合)のピストン
//! ロッドのような、1軸並進のみを許し他の5自由度を固定する用途を対象とする。

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
    /// `true`なら解決対象から除外する(`BallJoint::disabled`と同じ理由——
    /// 密な`Vec`から取り除くと他ジョイントのindexがずれる。`World::remove_body`
    /// の連鎖削除が使う)。
    pub disabled: bool,
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
    let prepared: Vec<PreparedDistanceJoint> = joints
        .iter()
        .filter(|j| !j.disabled)
        .map(|j| j.prepare(bodies, dt))
        .collect();
    for _ in 0..JOINT_VELOCITY_ITERATIONS {
        for p in &prepared {
            solve_velocity(p, bodies);
        }
    }
}

/// アンカー一致拘束(設計 §2.1)。`body_b = None` はワールド固定点(独楽の支点等、
/// M10。`sim-world::Command::Grab`が動く目標点への拘束としても使う — ワールド座標軸
/// 沿いの3本の独立スカラー拘束はゼロ距離でも(`DistanceJoint`の方向ベクトル
/// 正規化と異なり)退化しないため、掴んだ対象を目標点ぴったりまで引き寄せる用途に
/// 適する)を表す — 剛体はその点を中心に自由に回転できる。
#[derive(Clone, Copy)]
pub struct BallJoint {
    pub body_a: usize,
    /// body_a ローカル座標のアンカー点。
    pub anchor_a: Vec3,
    pub body_b: Option<usize>,
    /// `body_b` が `Some` ならそのローカル座標、`None` ならワールド座標(固定点)。
    pub anchor_b: Vec3,
    /// `true`なら`resolve_ball`が解決対象から除外する(削除操作の代替、
    /// `sim-world::Command::Release`が使う — 密な`Vec`から実際に取り除くと他の
    /// ジョイントのindexがずれるため、`RigidBodySet`の削除と同じ「無効化に留める」
    /// 方針)。
    pub disabled: bool,
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
    let prepared: Vec<PreparedBallJoint> = joints
        .iter()
        .filter(|j| !j.disabled)
        .map(|j| j.prepare(bodies, dt))
        .collect();
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
    /// **角度制限 $[\theta_{min}, \theta_{max}]$ [rad](設計 §4.4 表「+ limit」、群4で追加)**。
    ///
    /// 設計は「不等式(接触と同じクランプ $\lambda \ge 0$)」と定める——制限角に
    /// 達したときだけ働き、制限内へ戻る向きには抵抗しない片側拘束である。
    /// `None` なら無制限(従来の挙動、既存シーンは一切変わらない)。
    ///
    /// **なぜ必要か**: これが無いと肘・膝・ドアが**逆側へ無限に曲がる**。
    /// D12(ラグドール)は BallJoint(3自由度自由)で組んであるため、
    /// 腕が肩の内側へ回り込んでも何も止めるものが無かった。
    pub limit: Option<(f64, f64)>,
    /// `true`ならトルクを一切加えない(`BallJoint::disabled`と同じ理由)。
    pub disabled: bool,
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

    /// **角度制限の速度拘束(設計 §4.4 表「+ limit」、群4で追加)**。
    ///
    /// 制限角を**超えている**ときだけ、そこから戻す向きの角速度インパルスを加える
    /// (接触と同じ片側クランプ: 押し戻す向きにしか働かない)。Baumgarte バイアスで
    /// 位置誤差も戻す。制限内では何もしない。
    ///
    /// **`apply`(モーター)と分けて呼ぶ**——モーターはトルク蓄積器へ書き込む
    /// 力生成器だが、制限は速度レベルの拘束であり、`integrate_velocities` の
    /// **後**に解かないと1ステップぶん食い込む。
    pub fn solve_limit(&self, bodies: &mut RigidBodySet, dt: f64) {
        let Some((min, max)) = self.limit else {
            return;
        };
        let theta = self.measure_angle(bodies);
        // 超過量(制限内なら 0)。正なら「max を超えた」、負なら「min を下回った」。
        let excess = if theta > max {
            theta - max
        } else if theta < min {
            theta - min
        } else {
            return;
        };

        let inv_inertia = bodies.inv_inertia_world[self.body];
        let k = self.axis.dot(inv_inertia.mul_vec(self.axis));
        if k <= 0.0 {
            return;
        }
        let mass = 1.0 / k;
        let omega_axis = bodies.angular_velocity[self.body].dot(self.axis);
        // 誤差を戻す向きのバイアス(接触の Baumgarte と同じ形)。
        let bias = BAUMGARTE_BETA / dt * excess;
        let lambda = -(omega_axis + bias) * mass;
        // **片側クランプ**: max 超過なら負(戻す)向き、min 下回りなら正向きのみ許す。
        let clamped = if excess > 0.0 {
            lambda.min(0.0)
        } else {
            lambda.max(0.0)
        };
        bodies.angular_velocity[self.body] =
            bodies.angular_velocity[self.body] + inv_inertia.mul_vec(self.axis.scale(clamped));
    }
}

/// `HingeMotorPd`の角度制限を全て解く(**群4で追加**、`solve_limit`のdoc参照)。
/// 他のジョイントと同じく複数回反復する。
pub fn resolve_hinge_limits(motors: &[HingeMotorPd], bodies: &mut RigidBodySet, dt: f64) {
    if motors.iter().all(|m| m.limit.is_none() || m.disabled) {
        return;
    }
    for _ in 0..JOINT_VELOCITY_ITERATIONS {
        for motor in motors.iter().filter(|m| !m.disabled) {
            motor.solve_limit(bodies, dt);
        }
    }
}

/// `HingeMotorPd`一覧を全て`apply`する。
pub fn apply_hinge_motors(motors: &[HingeMotorPd], bodies: &mut RigidBodySet, dt: f64) {
    for motor in motors.iter().filter(|m| !m.disabled) {
        motor.apply(bodies, dt);
    }
}

/// 純角速度拘束(r×項なし)の有効質量。`Ball`/`Distance`の並進版`effective_mass`と対に
/// なる回転版(設計§4.2の$K=JM^{-1}J^T$を単一方向`dir`に射影、`body_b=None`は
/// ワールド固定=寄与0として扱う点も並進版と同じ)。
fn angular_effective_mass(
    bodies: &RigidBodySet,
    body_a: usize,
    body_b: Option<usize>,
    dir: Vec3,
) -> f64 {
    let inv_ia = bodies.inv_inertia_world[body_a];
    let term_a = dir.dot(inv_ia.mul_vec(dir));
    let term_b = match body_b {
        Some(b) => {
            let inv_ib = bodies.inv_inertia_world[b];
            dir.dot(inv_ib.mul_vec(dir))
        }
        None => 0.0,
    };
    let k = term_a + term_b;
    if k > 0.0 {
        1.0 / k
    } else {
        0.0
    }
}

/// 角速度への直接インパルス印加(`apply_impulse`の回転版、r×項もトルクへの変換も無い —
/// 純粋な角運動量インパルス、`contact::solve_rolling_friction`と同じ経路)。
fn apply_angular_impulse(bodies: &mut RigidBodySet, body: usize, impulse: Vec3, sign: f64) {
    let inv_i = bodies.inv_inertia_world[body];
    bodies.angular_velocity[body] =
        bodies.angular_velocity[body] + inv_i.mul_vec(impulse).scale(sign);
}

/// `body_a`/`body_b`間の相対回転の、生成時基準からのズレ(誤差ベクトル)。
/// `HingeMotorPd::measure_angle`と同じ「クォータニオンのベクトル部は小角では
/// (角度/2)*軸に近似できる」性質を使うが、ここでは正確な角度への逆変換(atan2)はせず
/// ベクトル部をそのままBaumgarteバイアスの誤差項として使う(位置ドリフト補正という
/// 用途では十分、`DistanceJoint`/`BallJoint`のバイアス項も同様に厳密解ではなく
/// 線形近似)。`w<0`のとき符号反転して最短回転経路を選ぶ(二重被覆の回避)。
fn relative_rotation_error(
    bodies: &RigidBodySet,
    body_a: usize,
    body_b: Option<usize>,
    reference_relative_rotation: Quat,
) -> Vec3 {
    let rot_a = bodies.rotation[body_a];
    let rot_b = body_b.map(|b| bodies.rotation[b]).unwrap_or(Quat::IDENTITY);
    let rel = rot_b.mul(rot_a.conjugate());
    let mut err = rel.mul(reference_relative_rotation.conjugate());
    if err.w < 0.0 {
        err = Quat {
            x: -err.x,
            y: -err.y,
            z: -err.z,
            w: -err.w,
        };
    }
    Vec3::new(err.x, err.y, err.z)
}

/// **ソフト拘束のパラメータ(設計 §4.3、群4で追加)**。
///
/// 剛な拘束を「周波数 $f$ と減衰比 $\zeta$ のばねダンパ」として振る舞わせる。
/// 設計が定める式:
/// $$k = m_{eff}(2\pi f)^2,\quad c = 2 m_{eff}\zeta(2\pi f),$$
/// $$\gamma = \frac{1}{\Delta t(c + \Delta t k)},\quad
///   \beta_{soft} = \frac{\Delta t k}{c + \Delta t k}$$
/// $\gamma$ は有効質量の対角正則化($K+\gamma$)へ、$\beta_{soft}$ は
/// 位置誤差のバイアス係数へ入る。
///
/// **なぜ陽的ばねではないのか**(設計§4.3の明記): 陽的な力生成器は
/// $\Delta t < 2/\omega$ でしか安定せず、サスペンションのような硬いばねを
/// 表現できない。ソフト拘束なら硬さを上げても(拘束ソルバの中で解くので)
/// 安定なまま。**群4で `WheelJoint` のサスペンションを作るのに必要になり実装した**
/// ——それまで `sim-mechanics` にばねダンパ系の拘束は1つも無く、
/// D24(車の実験場)が「新規物理待ちでスコープ外」だった直接の原因。
#[derive(Clone, Copy, Debug)]
pub struct SoftParams {
    /// 固有振動数 [Hz]。大きいほど硬い。
    pub frequency: f64,
    /// 減衰比 $\zeta$。1.0 で臨界減衰(振動せず最短で収束)。
    pub damping_ratio: f64,
}

impl SoftParams {
    /// 乗用車のサスペンション相当(設計§9のオーダー: 固有振動数 1–2 Hz、
    /// 減衰比 0.2–0.4)。
    pub fn suspension() -> SoftParams {
        SoftParams {
            frequency: 1.5,
            damping_ratio: 0.3,
        }
    }

    /// 有効質量 `m_eff` と `dt` から $(\gamma, \beta_{soft})$ を返す(設計§4.3)。
    /// `frequency <= 0` は「剛」を意味し $(0, \beta_{Baumgarte})$ を返す
    /// ——ソフト化しない既存の拘束と同じ挙動に落ちる。
    pub fn gamma_beta(&self, effective_mass: f64, dt: f64) -> (f64, f64) {
        if self.frequency <= 0.0 || effective_mass <= 0.0 || dt <= 0.0 {
            return (0.0, BAUMGARTE_BETA);
        }
        let omega = 2.0 * std::f64::consts::PI * self.frequency;
        let k = effective_mass * omega * omega;
        let c = 2.0 * effective_mass * self.damping_ratio * omega;
        let denominator = dt * (c + dt * k);
        if denominator <= 0.0 {
            return (0.0, BAUMGARTE_BETA);
        }
        (1.0 / denominator, dt * k / (c + dt * k))
    }
}

/// スライダー拘束(設計 §4.4 表「Slider | 5 | 軸直交並進2 + 相対回転固定3」)。
/// `axis_a`(body_aローカル座標、単位ベクトル)に沿った並進1自由度のみを自由とし、
/// それに直交する並進2自由度(`Vec3::orthonormal_basis`で決定的に選ぶ接線基底、
/// 接触ソルバの摩擦基底と同じ手法)+ 相対回転3自由度(生成時の相対姿勢を基準として
/// 固定、`relative_rotation_error`)を拘束する — 合計5行、`BallJoint`(3行)と同じ
/// 「ワールド座標軸沿いの独立スカラー拘束のPGS反復」という簡略化方針(箱近似)を
/// 踏襲する。`body_b=None`はワールド固定(シリンダー側が静止、ピストンのみ動く構成、
/// 断熱圧縮の`PistonGas`結合で使う想定)を表す。
#[derive(Clone, Copy)]
pub struct SliderJoint {
    pub body_a: usize,
    /// body_a ローカル座標のアンカー点。
    pub anchor_a: Vec3,
    /// スライド軸(body_a ローカル座標、単位ベクトル)。
    pub axis_a: Vec3,
    pub body_b: Option<usize>,
    /// `body_b` が `Some` ならそのローカル座標、`None` ならワールド座標(固定点)。
    pub anchor_b: Vec3,
    /// 生成時点の相対回転(角度0の基準)。`SliderJoint::new`で自動算出する。
    reference_relative_rotation: Quat,
    /// `true`なら解決対象から除外する(`BallJoint::disabled`と同じ理由)。
    pub disabled: bool,
}

impl SliderJoint {
    /// 現在の姿勢(`body_a`/`body_b`の相対回転)を基準(角度0)として`SliderJoint`を
    /// 生成する(`HingeMotorPd::reference_rotation`と同じ「生成時点の姿勢を基準に取る」
    /// 方針)。
    pub fn new(
        bodies: &RigidBodySet,
        body_a: usize,
        anchor_a: Vec3,
        axis_a: Vec3,
        body_b: Option<usize>,
        anchor_b: Vec3,
    ) -> SliderJoint {
        let rot_a = bodies.rotation[body_a];
        let rot_b = body_b.map(|b| bodies.rotation[b]).unwrap_or(Quat::IDENTITY);
        SliderJoint {
            body_a,
            anchor_a,
            axis_a,
            body_b,
            anchor_b,
            reference_relative_rotation: rot_b.mul(rot_a.conjugate()),
            disabled: false,
        }
    }
}

struct PreparedSliderJoint {
    body_a: usize,
    body_b: Option<usize>,
    r_a: Vec3,
    r_b: Vec3,
    /// スライド軸に直交する並進2軸(`BallJoint`の`axes`と同じ`PreparedBallAxis`を流用)。
    linear_axes: [PreparedBallAxis; 2],
    /// 相対回転を固定する3軸(ワールドx/y/z、`BallJoint`と同じ箱近似)。
    angular_axes: [PreparedBallAxis; 3],
}

impl SliderJoint {
    fn prepare(&self, bodies: &RigidBodySet, dt: f64) -> PreparedSliderJoint {
        let (world_a, r_a) = world_anchor(bodies, self.body_a, self.anchor_a);
        let (world_b, r_b) = world_anchor_or_fixed(bodies, self.body_b, self.anchor_b);
        let axis_world = bodies.rotation[self.body_a]
            .to_mat3()
            .mul_vec(self.axis_a)
            .normalize_or_zero();
        let (t1, t2) = axis_world.orthonormal_basis();
        // 拘束誤差(スライド軸に直交する成分のみ) C = (p_B-p_A) - ((p_B-p_A)・axis)axis。
        // 軸方向の並進は自由なのでバイアスは直交2軸への射影のみで良い。
        let c = world_b - world_a;
        let linear_axes = [t1, t2].map(|dir| {
            let mass = effective_mass(bodies, self.body_a, r_a, self.body_b, r_b, dir);
            let bias = BAUMGARTE_BETA / dt * c.dot(dir);
            PreparedBallAxis { dir, mass, bias }
        });

        let err = relative_rotation_error(
            bodies,
            self.body_a,
            self.body_b,
            self.reference_relative_rotation,
        );
        let dirs = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let angular_axes = dirs.map(|dir| {
            let mass = angular_effective_mass(bodies, self.body_a, self.body_b, dir);
            // `HingeMotorPd::measure_angle`と同じ2倍係数(ベクトル部 ≈ (角度/2)*軸)。
            let bias = 2.0 * BAUMGARTE_BETA / dt * err.dot(dir);
            PreparedBallAxis { dir, mass, bias }
        });

        PreparedSliderJoint {
            body_a: self.body_a,
            body_b: self.body_b,
            r_a,
            r_b,
            linear_axes,
            angular_axes,
        }
    }
}

fn solve_velocity_slider(p: &PreparedSliderJoint, bodies: &mut RigidBodySet) {
    for axis in &p.linear_axes {
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
    for axis in &p.angular_axes {
        let omega_a = bodies.angular_velocity[p.body_a];
        let omega_b = match p.body_b {
            Some(b) => bodies.angular_velocity[b],
            None => Vec3::ZERO,
        };
        let c_dot = axis.dir.dot(omega_b - omega_a);
        let lambda = -(c_dot + axis.bias) * axis.mass;
        let impulse = axis.dir.scale(lambda);
        apply_angular_impulse(bodies, p.body_a, impulse, -1.0);
        if let Some(b) = p.body_b {
            apply_angular_impulse(bodies, b, impulse, 1.0);
        }
    }
}

/// `resolve_distance`のSliderジョイント版。
pub fn resolve_slider(joints: &[SliderJoint], bodies: &mut RigidBodySet, dt: f64) {
    if joints.is_empty() {
        return;
    }
    let prepared: Vec<PreparedSliderJoint> = joints
        .iter()
        .filter(|j| !j.disabled)
        .map(|j| j.prepare(bodies, dt))
        .collect();
    for _ in 0..JOINT_VELOCITY_ITERATIONS {
        for p in &prepared {
            solve_velocity_slider(p, bodies);
        }
    }
}

/// **ホイールジョイント(設計 §3 の `Joint::Wheel(WheelJoint)`「Phase 4: サス+駆動+
/// 操舵の複合」、群4で実装)**。
///
/// **なぜ長らく無かったか**: D24(車の実験場)は「実車体を `WheelJoint`(サスペンション用の
/// ばねダンパ拘束+駆動用ヒンジモーター+操舵ヒンジ)で4輪支持する」ことを要求するが、
/// `sim-mechanics` には**ばねダンパ系の拘束が1つも無かった**(既存の4種
/// `DistanceJoint`/`BallJoint`/`HingeMotorPd`/`SliderJoint` はいずれも剛)。
/// そのため D24 は「新規物理待ちでスコープ外」として閉じられていた。
/// 群4で `SoftParams`(設計§4.3のソフト拘束)を実装したことで、正面から作れるようになった。
///
/// **拘束の構成**(シャシー `chassis` と車輪 `wheel` の間):
/// - **サスペンション軸方向 1 行(ソフト)**: 取り付け点から `rest_length` の距離を
///   ばねダンパで保つ。荷重が掛かれば沈み、抜ければ戻る。
/// - **サス軸に直交する並進 2 行(剛)**: 車輪がシャシーに対して前後左右へずれない。
/// - **車軸まわり以外の回転 2 行(剛)**: 車輪が転がる以外の向きへ倒れない。
/// - **駆動モーター 1 行**: 車軸まわりの角速度を `motor_speed` へ寄せる。
///   トルクは `motor_max_torque` でクランプ(設計§4.5「速度モーター」)。
///
/// 合計6行。`SliderJoint`(5行)と同じ「ワールド座標軸沿いの独立スカラー拘束を
/// PGS 反復で解く」箱近似方針を踏襲する。
///
/// **操舵**は `steer_angle` でシャシーの上方向まわりに車軸を回す。真の操舵ヒンジ
/// (キングピン軸まわりの独立した回転自由度+その角度を制御するモーター)ではなく、
/// **車軸方向を直接回す縮約**である——操舵の慣性やセルフアライニングトルクは
/// 表現しないが、D24 が要求する「旋回」の運動そのものは再現できる。
#[derive(Clone, Copy)]
pub struct WheelJoint {
    pub chassis: usize,
    pub wheel: usize,
    /// シャシーローカルのサスペンション取り付け点。
    pub anchor_chassis: Vec3,
    /// サスペンション軸(シャシーローカル、単位ベクトル)。車輪はこの向きに沈む
    /// ——通常は下向き `(0,-1,0)`。
    pub suspension_axis: Vec3,
    /// サスペンションの自然長 [m](取り付け点から車輪中心まで)。
    pub rest_length: f64,
    /// サスペンションのばね特性(設計§4.3)。
    ///
    /// **`frequency` は「拘束の固有振動数」であって「車の乗り心地周波数」ではない**。
    /// 設計§4.3 の $k=m_{eff}(2\pi f)^2$ の $m_{eff}$ は**この2体拘束の換算質量**
    /// $(1/m_{chassis}+1/m_{wheel})^{-1}$ であり、1輪が支える**ばね上質量**とは違う
    /// (車重 600 kg・車輪 60 kg なら換算質量は 54.5 kg だが、1輪の負担は 150 kg)。
    /// そのため乗用車の乗り心地周波数(1〜1.5 Hz)をそのまま入れるとばねが柔らかすぎて
    /// **サスが底付きする**(実際に D24 のシーンを組んで踏んだ: 自然長 0.43 m に対し
    /// 沈み込みが 0.30 m になり潰れた)。狙った沈み込み $x$ から
    /// $k=W/x$、$f=\frac{1}{2\pi}\sqrt{k/m_{eff}}$ と逆算して指定する。
    pub soft: SoftParams,
    /// 操舵角 [rad]。シャシーの「上」(= `-suspension_axis`)まわりに車軸を回す。
    pub steer_angle: f64,
    /// 車軸方向(操舵角ゼロのとき、シャシーローカル、単位ベクトル)。
    /// 通常は横方向 `(1,0,0)`。
    pub axle_axis: Vec3,
    /// 駆動モーターの目標角速度 [rad/s](車軸まわり)。
    pub motor_speed: f64,
    /// 駆動モーターのトルク上限 [N·m]。0 なら空転(駆動しない)。
    pub motor_max_torque: f64,
    pub disabled: bool,
}

struct PreparedWheelJoint {
    chassis: usize,
    wheel: usize,
    r_chassis: Vec3,
    r_wheel: Vec3,
    /// サス軸方向(ワールド)の行(ソフト)。
    suspension_dir: Vec3,
    suspension_mass: f64,
    suspension_bias: f64,
    suspension_gamma: f64,
    /// サス軸直交の並進2行(剛)。
    lateral_axes: [PreparedBallAxis; 2],
    /// 車軸まわり以外の回転2行(剛)。
    tilt_axes: [PreparedBallAxis; 2],
    /// 駆動モーター(車軸方向・目標角速度・トルク上限×dt)。
    axle_dir: Vec3,
    motor_mass: f64,
    motor_target_speed: f64,
    motor_max_impulse: f64,
}

impl WheelJoint {
    /// 既定の乗用車相当の車輪(下向きサスペンション・横向き車軸・駆動なし)。
    pub fn new(chassis: usize, wheel: usize, anchor_chassis: Vec3, rest_length: f64) -> WheelJoint {
        WheelJoint {
            chassis,
            wheel,
            anchor_chassis,
            suspension_axis: Vec3::new(0.0, -1.0, 0.0),
            rest_length,
            soft: SoftParams::suspension(),
            steer_angle: 0.0,
            axle_axis: Vec3::new(1.0, 0.0, 0.0),
            motor_speed: 0.0,
            motor_max_torque: 0.0,
            disabled: false,
        }
    }

    /// 操舵を反映した車軸方向(ワールド座標)。
    fn steered_axle_world(&self, bodies: &RigidBodySet) -> Vec3 {
        let rotation = bodies.rotation[self.chassis].to_mat3();
        let axle_local = self.axle_axis.normalize_or_zero();
        let up_local = self.suspension_axis.normalize_or_zero().scale(-1.0);
        // ロドリゲスの回転公式で `up_local` まわりに `steer_angle` だけ回す。
        let (sin, cos) = self.steer_angle.sin_cos();
        let steered_local = axle_local
            .scale(cos)
            .addcarry_scaled(up_local.cross(axle_local), sin)
            .addcarry_scaled(up_local, up_local.dot(axle_local) * (1.0 - cos));
        rotation.mul_vec(steered_local).normalize_or_zero()
    }

    fn prepare(&self, bodies: &RigidBodySet, dt: f64) -> PreparedWheelJoint {
        let (anchor_world, r_chassis) = world_anchor(bodies, self.chassis, self.anchor_chassis);
        let wheel_center = bodies.position[self.wheel];
        let r_wheel = Vec3::ZERO; // 車輪の重心が接続点。

        let rotation = bodies.rotation[self.chassis].to_mat3();
        let suspension_dir = rotation
            .mul_vec(self.suspension_axis.normalize_or_zero())
            .normalize_or_zero();

        // サス軸方向: 現在長 - 自然長 が拘束誤差。ソフト拘束なので γ で正則化し、
        // β_soft で位置誤差を戻す(設計§4.3)。
        let offset = wheel_center - anchor_world;
        let current_length = offset.dot(suspension_dir);
        let raw_mass = effective_mass(
            bodies,
            self.chassis,
            r_chassis,
            Some(self.wheel),
            r_wheel,
            suspension_dir,
        );
        let (gamma, beta_soft) = self.soft.gamma_beta(raw_mass, dt);
        // K + γ の逆数が有効質量(設計§4.3「対角正則化 (K+γ1)」)。
        let suspension_mass = if raw_mass > 0.0 {
            1.0 / (1.0 / raw_mass + gamma)
        } else {
            0.0
        };
        let suspension_bias = beta_soft / dt * (current_length - self.rest_length);

        // サス軸に直交する並進2行(剛)。基底は決定的に選ぶ(接触ソルバの摩擦基底と同じ)。
        let (t1, t2) = suspension_dir.orthonormal_basis();
        let lateral_axes = [t1, t2].map(|dir| {
            let mass = effective_mass(
                bodies,
                self.chassis,
                r_chassis,
                Some(self.wheel),
                r_wheel,
                dir,
            );
            let bias = BAUMGARTE_BETA / dt * offset.dot(dir);
            PreparedBallAxis { dir, mass, bias }
        });

        // 車軸まわり以外の回転2行(剛)。車軸に直交する2方向の相対角速度を殺す。
        let axle_dir = self.steered_axle_world(bodies);
        let (a1, a2) = axle_dir.orthonormal_basis();
        let tilt_axes = [a1, a2].map(|dir| {
            let inv_i_chassis = bodies.inv_inertia_world[self.chassis];
            let inv_i_wheel = bodies.inv_inertia_world[self.wheel];
            let k = dir.dot(inv_i_chassis.mul_vec(dir)) + dir.dot(inv_i_wheel.mul_vec(dir));
            let mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            // 姿勢の位置補正はしない(車輪は転がるので基準姿勢が定義できない)
            // ——角速度だけを合わせる。
            PreparedBallAxis {
                dir,
                mass,
                bias: 0.0,
            }
        });

        let inv_i_chassis = bodies.inv_inertia_world[self.chassis];
        let inv_i_wheel = bodies.inv_inertia_world[self.wheel];
        let motor_k = axle_dir.dot(inv_i_chassis.mul_vec(axle_dir))
            + axle_dir.dot(inv_i_wheel.mul_vec(axle_dir));
        let motor_mass = if motor_k > 0.0 { 1.0 / motor_k } else { 0.0 };

        PreparedWheelJoint {
            chassis: self.chassis,
            wheel: self.wheel,
            r_chassis,
            r_wheel,
            suspension_dir,
            suspension_mass,
            suspension_bias,
            suspension_gamma: gamma,
            lateral_axes,
            tilt_axes,
            axle_dir,
            motor_mass,
            motor_target_speed: self.motor_speed,
            motor_max_impulse: self.motor_max_torque * dt,
        }
    }
}

fn solve_velocity_wheel(
    p: &mut PreparedWheelJoint,
    accumulated: &mut f64,
    bodies: &mut RigidBodySet,
) {
    // ① サスペンション(ソフト): λ = -(Ċ + bias + γ·λ_acc) * m_eff。
    //    γ·λ_acc の項がソフト拘束の本体——蓄積インパルスに比例して拘束を「緩める」。
    {
        let v_chassis = point_velocity(bodies, p.chassis, p.r_chassis);
        let v_wheel = point_velocity(bodies, p.wheel, p.r_wheel);
        let c_dot = p.suspension_dir.dot(v_wheel - v_chassis);
        let lambda =
            -(c_dot + p.suspension_bias + p.suspension_gamma * *accumulated) * p.suspension_mass;
        *accumulated += lambda;
        let impulse = p.suspension_dir.scale(lambda);
        apply_impulse(bodies, p.chassis, impulse, p.r_chassis, -1.0);
        apply_impulse(bodies, p.wheel, impulse, p.r_wheel, 1.0);
    }

    // ② サス軸直交の並進2行(剛)。
    for axis in &p.lateral_axes {
        let v_chassis = point_velocity(bodies, p.chassis, p.r_chassis);
        let v_wheel = point_velocity(bodies, p.wheel, p.r_wheel);
        let c_dot = axis.dir.dot(v_wheel - v_chassis);
        let lambda = -(c_dot + axis.bias) * axis.mass;
        let impulse = axis.dir.scale(lambda);
        apply_impulse(bodies, p.chassis, impulse, p.r_chassis, -1.0);
        apply_impulse(bodies, p.wheel, impulse, p.r_wheel, 1.0);
    }

    // ③ 車軸まわり以外の回転2行(剛)。
    for axis in &p.tilt_axes {
        let c_dot = axis
            .dir
            .dot(bodies.angular_velocity[p.wheel] - bodies.angular_velocity[p.chassis]);
        let lambda = -(c_dot + axis.bias) * axis.mass;
        let impulse = axis.dir.scale(lambda);
        apply_angular_impulse(bodies, p.chassis, impulse, -1.0);
        apply_angular_impulse(bodies, p.wheel, impulse, 1.0);
    }
}

/// 駆動モーター行(設計§4.5「速度モーター: 目標角速度・トルク上限」)。
/// 反復のたびにクランプが効くよう、蓄積インパルスを持ち回る。
fn solve_wheel_motor(p: &PreparedWheelJoint, accumulated: &mut f64, bodies: &mut RigidBodySet) {
    if p.motor_max_impulse <= 0.0 {
        return;
    }
    let relative = p
        .axle_dir
        .dot(bodies.angular_velocity[p.wheel] - bodies.angular_velocity[p.chassis]);
    let mut lambda = -(relative - p.motor_target_speed) * p.motor_mass;
    // |λ_total| ≤ τ_max·dt(設計§4.4表の motor 行のクランプ)。
    let old = *accumulated;
    let clamped = (old + lambda).clamp(-p.motor_max_impulse, p.motor_max_impulse);
    lambda = clamped - old;
    *accumulated = clamped;
    let impulse = p.axle_dir.scale(lambda);
    apply_angular_impulse(bodies, p.chassis, impulse, -1.0);
    apply_angular_impulse(bodies, p.wheel, impulse, 1.0);
}

/// `resolve_distance`のホイールジョイント版。
pub fn resolve_wheel(joints: &[WheelJoint], bodies: &mut RigidBodySet, dt: f64) {
    if joints.is_empty() {
        return;
    }
    let mut prepared: Vec<PreparedWheelJoint> = joints
        .iter()
        .filter(|j| !j.disabled)
        .map(|j| j.prepare(bodies, dt))
        .collect();
    // ソフト拘束とモーターは**蓄積インパルスを反復間で持ち回る**必要がある
    // (前者は γ·λ_acc の正則化項、後者は τ_max のクランプのため)。
    let mut suspension_impulses = vec![0.0; prepared.len()];
    let mut motor_impulses = vec![0.0; prepared.len()];
    for _ in 0..JOINT_VELOCITY_ITERATIONS {
        for (index, p) in prepared.iter_mut().enumerate() {
            solve_velocity_wheel(p, &mut suspension_impulses[index], bodies);
            solve_wheel_motor(p, &mut motor_impulses[index], bodies);
        }
    }
}

#[cfg(test)]
mod wheel_tests {
    use super::*;
    use crate::body::{BodyType, RigidBodyDesc};
    use crate::shape::Shape;
    use crate::solver::MechanicsSolver;
    use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
    use sim_math::SimRng;

    /// シャシー1体 + 車輪1体を `WheelJoint` で繋いだ最小構成。
    /// 返り値は `(solver, chassis, wheel)`。
    fn wheel_rig(
        gravity: f64,
        rest_length: f64,
        chassis_mass: f64,
        wheel_mass: f64,
    ) -> (MechanicsSolver, usize, usize) {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut solver = MechanicsSolver::new(gravity);

        let mut chassis = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.6, 0.2, 0.3),
            },
            steel,
        );
        chassis.mass_override = Some(chassis_mass);
        chassis.transform.position = Vec3::new(0.0, 1.0, 0.0);
        let chassis_index = solver.bodies.create_body(chassis, &materials);

        let mut wheel = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.3 }, steel);
        wheel.mass_override = Some(wheel_mass);
        wheel.transform.position = Vec3::new(0.0, 1.0 - rest_length, 0.0);
        let wheel_index = solver.bodies.create_body(wheel, &materials);

        // **シャシーと車輪を互いに衝突させない**(群2の衝突フィルタを使う)。
        // 実車でも当然そうする——サスペンションで繋がった2体は幾何的に必ず
        // 重なるため、接触ソルバが働くとジョイントと綱引きになる。
        // **これを入れる前は、接触ソルバが相対速度を殺してサスペンションが
        // 一切沈まなかった**(実装検証中に発見。群2の衝突フィルタが群4で
        // 実際に必要になった例)。
        solver
            .bodies
            .set_collision_filter(chassis_index, 0b01, 0b01);
        solver.bodies.set_collision_filter(wheel_index, 0b10, 0b10);

        solver.wheel_joints.push(WheelJoint::new(
            chassis_index,
            wheel_index,
            Vec3::ZERO,
            rest_length,
        ));
        (solver, chassis_index, wheel_index)
    }

    fn run(solver: &mut MechanicsSolver, dt: f64, steps: u32) {
        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(0, 0);
        let mut events = EventQueue::new();
        for _ in 0..steps {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            solver.step(dt, &mut ctx);
        }
    }

    /// **サスペンションのばね定数が設計 §4.3 の式どおりであること**。
    ///
    /// 車輪を固定(Static)してシャシーを吊ると、シャシーは
    /// **ばね力と重力が釣り合う沈み込み量 $x = m g / k$** で静止する。
    /// $k = m_{eff}(2\pi f)^2$(設計§4.3)なので、`m_eff` がシャシー質量に
    /// 等しい(車輪が無限質量)この構成では $x = g/(2\pi f)^2$ ——
    /// **質量に依存しない閉形式**になる。これを実測と突き合わせる。
    #[test]
    fn suspension_settles_at_the_static_deflection_predicted_by_the_spring_rate() {
        let gravity = 9.80665;
        let rest_length = 0.5;
        let frequency = 1.5;
        let (mut solver, chassis, wheel) = wheel_rig(gravity, rest_length, 400.0, 20.0);
        // 車輪を地面に固定した状態を模す(Static = 無限質量)。
        solver.bodies.set_body_type(wheel, BodyType::Static, 0.0);
        solver.wheel_joints[0].soft = SoftParams {
            frequency,
            damping_ratio: 1.0, // 臨界減衰で速く収束させる(振動を待たない)。
        };

        let wheel_y = solver.bodies.position[wheel].y;
        run(&mut solver, 1.0 / 240.0, 4000);

        // シャシーは車輪の上 rest_length のはずが、荷重で x だけ沈む。
        let actual_length = solver.bodies.position[chassis].y - wheel_y;
        let deflection = rest_length - actual_length;
        let omega = 2.0 * std::f64::consts::PI * frequency;
        let expected = gravity / (omega * omega);
        assert!(
            (deflection - expected).abs() / expected < 0.05,
            "static deflection must match g/(2πf)²: deflection={deflection} expected={expected}"
        );

        // **硬さを上げれば沈み込みは減る**(f を2倍にすると 1/4)。
        let (mut stiff, chassis2, wheel2) = wheel_rig(gravity, rest_length, 400.0, 20.0);
        stiff.bodies.set_body_type(wheel2, BodyType::Static, 0.0);
        stiff.wheel_joints[0].soft = SoftParams {
            frequency: frequency * 2.0,
            damping_ratio: 1.0,
        };
        let wheel2_y = stiff.bodies.position[wheel2].y;
        run(&mut stiff, 1.0 / 240.0, 4000);
        let stiff_deflection = rest_length - (stiff.bodies.position[chassis2].y - wheel2_y);
        assert!(
            (stiff_deflection - expected / 4.0).abs() / (expected / 4.0) < 0.08,
            "doubling the frequency must quarter the deflection: \
             stiff={stiff_deflection} expected={}",
            expected / 4.0
        );
    }

    /// **駆動モーターが目標角速度へ寄せ、トルク上限でクランプされること**
    /// (設計§4.5「速度モーター: 目標角速度・トルク上限」、§4.4表の
    /// `|λ| ≤ τ_max·dt` クランプ)。
    #[test]
    fn drive_motor_reaches_target_speed_and_saturates_at_the_torque_limit() {
        // 重力なし・シャシーは Static(反作用を受け止める土台)。
        let (mut solver, chassis, wheel) = wheel_rig(0.0, 0.5, 400.0, 20.0);
        solver.bodies.set_body_type(chassis, BodyType::Static, 0.0);
        solver.wheel_joints[0].motor_speed = 30.0;
        solver.wheel_joints[0].motor_max_torque = 200.0;
        let axle = solver.wheel_joints[0].axle_axis;

        run(&mut solver, 1.0 / 240.0, 600);
        let spin = solver.bodies.angular_velocity[wheel].dot(axle);
        assert!(
            (spin - 30.0).abs() < 0.5,
            "unloaded wheel must approach the target speed: spin={spin}"
        );

        // **トルク上限が効くこと**: 上限をごく小さくすると、同じ時間では
        // 目標角速度に到達できない。到達した角速度は τ·t/I で決まる。
        let (mut limited, chassis2, wheel2) = wheel_rig(0.0, 0.5, 400.0, 20.0);
        limited
            .bodies
            .set_body_type(chassis2, BodyType::Static, 0.0);
        limited.wheel_joints[0].motor_speed = 30.0;
        let torque = 0.5;
        limited.wheel_joints[0].motor_max_torque = torque;
        let steps = 600;
        let dt = 1.0 / 240.0;
        run(&mut limited, dt, steps);
        let limited_spin = limited.bodies.angular_velocity[wheel2].dot(axle);
        // 球の慣性モーメント I = 2/5 m r²(m=20, r=0.3)。
        let inertia = 0.4 * 20.0 * 0.3 * 0.3;
        let expected = torque * (steps as f64 * dt) / inertia;
        assert!(
            (limited_spin - expected).abs() / expected < 0.05,
            "torque-limited spin-up must follow τt/I: spin={limited_spin} expected={expected}"
        );
        assert!(
            limited_spin < 30.0,
            "the limited motor must not reach the target: spin={limited_spin}"
        );
    }

    /// **角度制限が制限角で止め、制限内では何もしないこと**
    /// (設計 §4.4 表「+ limit: 不等式(接触と同じクランプ λ≥0)」、群4で追加)。
    ///
    /// ワールド固定点にボールジョイントでピン留めした棒を重力で振らせ、
    /// **制限が無ければ大きく振れる**のに対し、制限を入れると制限角で止まることを見る。
    #[test]
    fn hinge_limit_stops_the_arm_at_the_limit_angle() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let limit_angle = -0.3_f64;

        let build = |limit: Option<(f64, f64)>| {
            let mut solver = MechanicsSolver::new(9.80665);
            let mut arm = RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(0.5, 0.05, 0.05),
                },
                steel,
            );
            arm.transform.position = Vec3::new(0.5, 1.0, 0.0);
            let body = solver.bodies.create_body(arm, &materials);
            // 棒の左端をワールド固定点へピン留め(振り子的な関節)。
            solver.ball_joints.push(BallJoint {
                body_a: body,
                anchor_a: Vec3::new(-0.5, 0.0, 0.0),
                body_b: None,
                anchor_b: Vec3::new(0.0, 1.0, 0.0),
                disabled: false,
            });
            solver.hinge_motors.push(HingeMotorPd {
                body,
                axis,
                reference_rotation: solver.bodies.rotation[body],
                theta_target: 0.0,
                kp: 0.0,
                kd: 0.0,
                torque_max: 0.0, // モーターは使わない——制限だけを見る。
                limit,
                disabled: false,
            });
            (solver, body)
        };

        // ① 制限なし: 重力で大きく振れ下がる。
        let (mut free, free_body) = build(None);
        run(&mut free, 1.0 / 240.0, 400);
        let free_angle = free.hinge_motors[0].measure_angle(&free.bodies);
        assert!(
            free_angle < limit_angle - 0.1,
            "制限が無ければ制限角より深く振れるはず: angle={free_angle}"
        );
        let _ = free_body;

        // ② 制限あり: 制限角付近で止まる。
        let (mut limited, _) = build(Some((limit_angle, 1.0)));
        run(&mut limited, 1.0 / 240.0, 400);
        let limited_angle = limited.hinge_motors[0].measure_angle(&limited.bodies);
        assert!(
            limited_angle >= limit_angle - 0.05,
            "制限角で止まるはず: angle={limited_angle} limit={limit_angle}"
        );
        // 制限内には入っている(=制限が「押し戻しすぎ」ていない)。
        assert!(
            limited_angle <= 0.05,
            "制限が逆向きに押し上げてはいけない: angle={limited_angle}"
        );
    }

    /// **操舵で車軸の向きが変わること**。`steer_angle` を 90° にすると、
    /// 車軸は横向き(+x)から前後向き(-z)へ回る(シャシーの上 +y まわり)。
    #[test]
    fn steering_rotates_the_axle_about_the_chassis_up_axis() {
        let (solver, _, _) = wheel_rig(0.0, 0.5, 400.0, 20.0);
        let mut joint = solver.wheel_joints[0];

        joint.steer_angle = 0.0;
        let straight = joint.steered_axle_world(&solver.bodies);
        assert!(
            (straight - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-12,
            "{straight:?}"
        );

        joint.steer_angle = std::f64::consts::FRAC_PI_2;
        let turned = joint.steered_axle_world(&solver.bodies);
        // up = -suspension_axis = +y。+y まわりに +90° 回すと x軸 → -z 軸。
        assert!(
            (turned - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-9,
            "{turned:?}"
        );
    }
}
