//! 力学ソルバ。設計: docs/10-mechanics/01-rigid-body.md §4/§9、
//!       docs/10-mechanics/02-collision-detection.md、docs/10-mechanics/03-contact-solver.md。
//!
//! P1/P2 スコープ: 重力の適用・semi-implicit Euler 積分・総当たり衝突検出・
//! sequential impulses 接触ソルバ(反発+Baumgarte+箱近似クーロン摩擦+warm starting)。
//! 最小CCD・split impulse・スリープは別増分で追加する
//! (docs/22-roadmap/01-phases.md P1/P2 ウェーブ)。

use crate::body::{BodyType, DragModel, RigidBodySet};
use crate::joint::{BallJoint, DistanceJoint, HingeMotorPd, SliderJoint};
use crate::shape::Shape;
use crate::{ccd, collision, contact, joint, sleep, RigidBodyDesc};
use sim_core::{
    Approximation, EnergyBreakdown, Event, EventKind, MaterialDb, Solver, SolverContext, SourceId,
    StateHasher,
};
use sim_fluid::{Atmosphere, StaticWaterRegion};
use sim_math::Vec3;
use std::collections::HashSet;

/// 重力場(**重力場の抽象化増分で追加**)。
///
/// **なぜ列挙にしたか**: それまで`MechanicsSolver`は重力を
/// `gravity: f64`(大きさ)と`gravity_direction: Vec3`(向き)の2つの公開
/// フィールドで持っていた。この表現は**一様場しか表せない**——空間のどこでも
/// 同じ加速度ベクトルになる場である。「小惑星の周りを回る」「惑星表面の重力が
/// 高度で弱まる」といった、点源(中心力)の場を書く手段が無く、無重力ですら
/// 「大きさ0の一様場」という遠回りな表現しか無かった。列挙にすることで、
/// 呼び出し側が読む唯一の入口を`acceleration_at`(位置を受け取る)へ一本化でき、
/// 場の種類を増やしても積分側のコードは変わらない。
///
/// **正直な適用範囲**(移行前からの制約をそのまま引き継ぐ): この場が効くのは
/// 自由体(Dynamic)への直接の重力積分(`MechanicsSolver::apply_forces`)と
/// ポテンシャルエネルギー計算のみ。浮力(`sim_fluid::StaticWaterRegion`)は
/// 水面をワールドy座標の水平面として定義するモデルのため、重力の向きや
/// 位置依存性に追従しない(`buoyancy_force`の既存実装、`sim-fluid`crateの
/// 設計上の制約——水面が重力と独立に常に水平だという簡略化は、重力の向きを
/// 可変にする前から存在した)。大気の抗力(`drag_force_sphere`)も同様に
/// 重力に依存しない。**浮力・自然対流を重力場へ追従させるのは別の計画作業**で
/// あり、本増分では踏み込まない(`MechanicsSolver::gravity`のdoc参照)。
///
/// **`sim_astro`のN体重力とは別系統**: `sim_astro::NBodySystem`は天体どうしの
/// 実際の万有引力を対ごとに計算する独立したドメインであり、この`GravityField`
/// (「普通の剛体を何が加速するか」を決める外場)とは混ぜない。天体は
/// `GravityField`を必要としない。
///
/// **`SoftBody`/`SphFluid`とも別系統**: 両者はシーンJSONから直接設定される
/// 自前の`gravity`を持つ(`soft_body.gravity`・`sph.gravity`)。ドメインごとの
/// 設定であり、本列挙の対象外。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GravityField {
    /// 一様場(移行前の唯一の挙動)。空間のどこでも`magnitude * direction`。
    Uniform {
        /// 重力加速度の大きさ [m/s^2]。既定 9.80665
        /// (docs/00-foundation/03-units-conventions.md)。
        magnitude: f64,
        /// 重力加速度の向き(単位ベクトル、既定は下向き`(0,-1,0)`)。
        /// `MechanicsSolver::set_gravity_direction`が常に正規化して保持する。
        direction: Vec3,
    },
    /// 点源(中心力)場。`center`へ向かう逆二乗の加速度
    /// $\mathbf{a} = -\mu\,\mathbf{r}/|\mathbf{r}|^3$($\mathbf{r}$は`center`
    /// からの相対位置)。
    ///
    /// **なぜ逆二乗にしたか**: 「点源の重力」が物理的に意味するのはこれであり、
    /// 円軌道速度$v=\sqrt{\mu/r}$・vis-viva・「高度が上がると重力が弱まる」と
    /// いった解析解がそのまま検証に使える。線形場や大きさ一定の中心力にすると
    /// 実装は簡単になるが、対応する既知の解析解がシーン作者にとって役に立たない。
    ///
    /// **中心の特異点**: $|\mathbf{r}|$が`POINT_SOURCE_MIN_RADIUS`未満のときは
    /// `Vec3::ZERO`を返す。厳密な中心では向きが定義できず、そのまま計算すると
    /// NaN/無限大が速度→位置→`state_hash`と伝播してシーン全体を汚染するため、
    /// 安全側へ縮退させる。
    ///
    /// **既知の限界**: ソフトニング半径は導入していない。中心へ十分近づけば
    /// 加速度は実際に発散し、固定dtの陽的(semi-implicit Euler)積分は破綻する。
    /// ソフトニングを入れると点源の解析解から系統的にずれ、円軌道テストの
    /// 検証力が落ちるため、「点源は点源のまま」にして限界をここに書く方を選んだ。
    PointSource {
        /// 場の中心(ワールド座標)。
        center: Vec3,
        /// 標準重力パラメータ $\mu = GM$ [m^3/s^2](`sim_astro::swingby`が
        /// 使う`mu`と同じ量・同じ名前)。
        mu: f64,
    },
    /// 無重力。`Uniform { magnitude: 0.0, .. }`と数値的には等価だが、
    /// 「重力が無い」という意図をシーンJSONと`Debug`出力に明示できる。
    Zero,
}

/// `GravityField::PointSource`が加速度を`Vec3::ZERO`へ縮退させる中心からの
/// 距離 [m](同variantのdoc参照)。
pub const POINT_SOURCE_MIN_RADIUS: f64 = 1e-9;

impl GravityField {
    /// **この場の唯一の入口**——位置`position`における重力加速度 [m/s^2]。
    /// 積分・力の累積・ポテンシャルはすべてこれを経由する(場の種類が増えても
    /// 呼び出し側は変わらない)。
    pub fn acceleration_at(&self, position: Vec3) -> Vec3 {
        match *self {
            GravityField::Uniform {
                magnitude,
                direction,
            } => direction.scale(magnitude),
            GravityField::PointSource { center, mu } => {
                let r = position - center;
                let distance = r.length();
                if distance < POINT_SOURCE_MIN_RADIUS {
                    return Vec3::ZERO;
                }
                // -mu * r / |r|^3。`normalize_or_zero`を経由せず直接割るのは、
                // 1/r^2 と 1/|r| を1回の除算にまとめて丸めを減らすため。
                r.scale(-mu / (distance * distance * distance))
            }
            GravityField::Zero => Vec3::ZERO,
        }
    }

    /// 単位質量あたりの重力ポテンシャル [J/kg](`MechanicsSolver::total_energy`用)。
    /// 一様場は$-\mathbf{g}\cdot\mathbf{x}$(基準 原点)、点源は$-\mu/r$
    /// (基準 無限遠)——どちらも$-\nabla\Phi = \mathbf{a}$を満たす。
    ///
    /// 点源で$r$が`POINT_SOURCE_MIN_RADIUS`未満のときは0を返す
    /// (`acceleration_at`と同じ理由の縮退。中心近傍のポテンシャルは
    /// 発散するため、エネルギー台帳へ無限大を流し込まない)。
    pub fn potential_per_mass(&self, position: Vec3) -> f64 {
        match *self {
            GravityField::Uniform { .. } | GravityField::Zero => {
                -self.acceleration_at(position).dot(position)
            }
            GravityField::PointSource { center, mu } => {
                let distance = (position - center).length();
                if distance < POINT_SOURCE_MIN_RADIUS {
                    return 0.0;
                }
                -mu / distance
            }
        }
    }

    /// 「この場を一様場1つで近似したときの大きさ」[m/s^2]
    /// (`MechanicsSolver::gravity`が返す値の実体、そちらのdocに判断根拠がある)。
    fn uniform_magnitude(&self) -> f64 {
        match *self {
            GravityField::Uniform { magnitude, .. } => magnitude,
            GravityField::PointSource { .. } | GravityField::Zero => 0.0,
        }
    }

    /// 「この場を一様場1つで近似したときの向き」(`MechanicsSolver::
    /// gravity_direction`が返す値の実体、そちらのdoc参照)。
    fn uniform_direction(&self) -> Vec3 {
        match *self {
            GravityField::Uniform { direction, .. } => direction,
            // 向きが定義できない場では既定の下向きを返す(ゼロベクトルを返すと
            // 「正規化された単位ベクトル」という呼び出し側の前提を壊す)。
            // 大きさが0なので積は正しく`Vec3::ZERO`になる。
            GravityField::PointSource { .. } | GravityField::Zero => DEFAULT_GRAVITY_DIRECTION,
        }
    }
}

/// 重力の既定の向き(下向き)。ゼロベクトルを渡されたときのフォールバックにも使う。
const DEFAULT_GRAVITY_DIRECTION: Vec3 = Vec3 {
    x: 0.0,
    y: -1.0,
    z: 0.0,
};

/// 向きを正規化し、正規化できない(ゼロベクトル)なら既定の下向きへ落とす。
/// `set_gravity_direction`/`set_gravity_field`が共有する唯一の実装。
fn normalized_or_default_direction(direction: Vec3) -> Vec3 {
    let normalized = direction.normalize_or_zero();
    if normalized.length() > 0.0 {
        normalized
    } else {
        DEFAULT_GRAVITY_DIRECTION
    }
}

#[derive(Clone)]
pub struct MechanicsSolver {
    pub bodies: RigidBodySet,
    /// 重力場(`GravityField`のdoc参照)。**公開フィールドではなく
    /// `gravity_field`/`set_gravity_field`と、一様場向けの薄いアクセサ
    /// (`gravity`/`set_gravity`/`gravity_direction`/`set_gravity_direction`)
    /// 経由で触る**——`direction`の正規化とゼロベクトルのフォールバックを
    /// 迂回されないようにするため(移行前の`pub gravity_direction`は
    /// `set_gravity_direction`を通さない代入を許していた)。
    field: GravityField,
    /// 反発を無視する接近速度の閾値(設計 §4.3・§9、既定 0.5 m/s)。ジッタ防止用の
    /// ヒューリスティクスであり、理想化された弾性衝突の検証(M5 等)では 0 に下げてよい。
    pub restitution_velocity_threshold: f64,
    /// 抗力の評価に使う周囲媒質(設計 docs/11-fluid/05-aero-hydrodynamics.md §3)。
    /// `None`(既定)は真空相当(抗力なし)。P1 は単一の一様媒質のみ(局所媒質・格子流体
    /// との排他は Phase 3、docs/11-fluid/05 §6)。
    pub atmosphere: Option<Atmosphere>,
    /// 浮力の評価に使う静的水域(設計 docs/11-fluid/04-free-surface-buoyancy.md §3)。
    /// `None`(既定)は水域なし。P1 は直立姿勢の直方体のみ対応(`sim_fluid::buoyancy` 冒頭注記)。
    pub water: Option<StaticWaterRegion>,
    /// マニフォールド持続化 + warm starting 用の永続キャッシュ(設計
    /// docs/10-mechanics/02-collision-detection.md §4.7・03-contact-solver.md §4.4)。
    /// `persistence_enabled` の切り替えは `set_manifold_persistence` から行う。
    contact_cache: contact::ManifoldCache,
    /// Box-Box 軸選択ヒステリシス用キャッシュ(設計 docs/10-mechanics/02-collision-detection.md §4.4)。
    axis_cache: collision::AxisCache,
    /// フルCCD(設計 §4.6「フルCCD(Phase 5)」)を走らせるか。既定は有効。
    /// `set_full_ccd` から切り替える(対照実験専用)。
    full_ccd_enabled: bool,
    /// Distance ジョイント一覧(設計 docs/10-mechanics/05-joints-constraints.md §3)。
    pub joints: Vec<DistanceJoint>,
    /// Ball ジョイント一覧(設計 docs/10-mechanics/05-joints-constraints.md §3)。
    pub ball_joints: Vec<BallJoint>,
    /// Slider ジョイント一覧(設計 §4.4表「Slider」、`joint`モジュールdoc参照)。
    pub slider_joints: Vec<SliderJoint>,
    /// PD位置サーボ付きヒンジモーター一覧(設計 §4.5、`joint`モジュールdoc参照)。
    pub hinge_motors: Vec<HingeMotorPd>,
    /// ホイールジョイント一覧(設計 §3「Wheel: サス+駆動+操舵の複合」、**群4で追加**)。
    /// D24(車の実験場)が要求する4輪支持の土台。
    pub wheel_joints: Vec<joint::WheelJoint>,
    /// 直近stepの接触解決(摩擦+反発)による運動エネルギー散逸量(設計
    /// docs/20-integration/01-coupling-matrix.md `DissipationToHeat`が読む、
    /// `sim-coupling`クレートのdoc参照)。接触解決の直前直後の運動エネルギー差分として
    /// 測定する(位置は変化しないためポテンシャルエネルギーは不変、速度の変化のみを見れば
    /// 十分)。抗力による散逸は含まない(抗力は保存力(重力)と共に`apply_forces`で積分
    /// されるため、この測定窓では分離できない。後続増分で抗力の仕事を個別に計測して追加
    /// する)。0にクランプしない — Baumgarte安定化・warm startingは稀に1step内で微小に
    /// 運動エネルギーを増やすことがある(PGS系接触ソルバの既知の数値アーティファクト、
    /// 物理的な現象ではない)ため、クランプすると増加分を無視し減少分だけ計上する系統的な
    /// 片側バイアスになる。実装検証中の発見: それでもなお、10秒・1200stepの滑走→静止
    /// シナリオでは、この量の累積和が実際の力学的エネルギー総損失(区間の`total_energy()`
    /// の差)より約9%大きいことを確認した — 原因は、Baumgarte位置誤差補正がこの
    /// (`contact::resolve()`呼び出し前後のみの)測定窓では運動エネルギー変化として
    /// 現れる一方、その補正効果は次stepの位置積分にも影響し、測定窓の外側で部分的に
    /// 打ち消されるため、前後差分の単純な累積が系統的に過大評価になること(PGS+
    /// Baumgarteソルバの既知の限界であり、クランプの有無では解決しない)。根本修正
    /// (Baumgarteのバイアス速度分を測定から除外する等)は接触ソルバへの踏み込んだ変更を
    /// 要するため本増分では見送り、`sim-coupling::DissipationToHeat`の受け入れテスト側で
    /// この系統誤差を踏まえた許容誤差(rel<15%)を設定して対応する。
    pub last_contact_dissipation: f64,
    /// 直近stepの接触解決で**剛体ごとに**失われた運動エネルギー [J](**群5で追加**、
    /// `body_index`で引く。総和は`last_contact_dissipation`と厳密に一致する)。
    ///
    /// `sim-coupling::DissipationToHeat`が設計 docs/12-thermal/02-heat-transfer.md §4.4
    /// 「熱浸透率比分配」を実装するために必要になった——散逸熱をどの`ThermalNode`へ
    /// 配るかは接触ペアごとに決まるので、シーン全体の合計値
    /// (`last_contact_dissipation`)だけでは配分できない。`last_manifolds`(どの剛体
    /// どうしが接触したか)と組み合わせて使う。
    ///
    /// 上記`last_contact_dissipation`の系統誤差(約9%の過大評価)はこの分解値にも
    /// そのまま含まれる(同じ測定窓の前後差分を剛体ごとに取っているだけ)。
    pub last_contact_dissipation_by_body: Vec<f64>,
    /// 直近stepで検出された接触ペア(`(body_a, body_b)`、`ContactManifold`と同じ正規化
    /// 順序)。今stepの検出結果との差分から`EventKind::ContactStarted`/`ContactEnded`
    /// (設計docs/00-foundation/04-architecture.md §1.1.2(5))を`ctx.events`へ発行する
    /// (`World`最初のイベント生産者、`subscribe`/`drain_events`のdoc参照)。
    contact_pairs: HashSet<(usize, usize)>,
    /// 直近stepで検出された接触マニフォールド(sleep判定に使う`manifolds`と同じもの、
    /// スリープ中でスキップされたペアも含む)。エディタのScene Viewオーバーレイ
    /// (設計docs/23-frontend/01-editor.md §1.2「接触点」)向けに、外部から読み取れる
    /// ようそのまま保持しておく——物理解には影響しない読み取り専用のキャッシュ。
    pub last_manifolds: Vec<collision::ContactManifold>,
}

impl MechanicsSolver {
    /// 一様重力(大きさ`gravity`・向きは既定の下向き)のソルバを作る。
    /// **シグネチャは`GravityField`導入前から変えていない**——ワークスペース全体の
    /// 約750本のテストとシーン構築経路がこの形で呼んでいるため、`GravityField`を
    /// 引数に取る形へ変えると本質と無関係な差分が全域へ広がる。非一様な場は
    /// `set_gravity_field`で後から与える(`GravityField`のdoc参照)。
    pub fn new(gravity: f64) -> MechanicsSolver {
        MechanicsSolver {
            bodies: RigidBodySet::new(),
            field: GravityField::Uniform {
                magnitude: gravity,
                direction: DEFAULT_GRAVITY_DIRECTION,
            },
            restitution_velocity_threshold: contact::DEFAULT_RESTITUTION_VELOCITY_THRESHOLD,
            atmosphere: None,
            water: None,
            contact_cache: contact::ManifoldCache::new(),
            axis_cache: collision::AxisCache::new(),
            full_ccd_enabled: true,
            joints: Vec::new(),
            ball_joints: Vec::new(),
            slider_joints: Vec::new(),
            hinge_motors: Vec::new(),
            wheel_joints: Vec::new(),
            last_contact_dissipation: 0.0,
            last_contact_dissipation_by_body: Vec::new(),
            contact_pairs: HashSet::new(),
            last_manifolds: Vec::new(),
        }
    }

    pub fn create_body(&mut self, desc: RigidBodyDesc, materials: &MaterialDb) -> usize {
        self.bodies.create_body(desc, materials)
    }

    /// マニフォールド持続化(設計 docs/10-mechanics/02-collision-detection.md §4.7)の
    /// 有効/無効。既定は有効。`false` にすると移行前の挙動(feature_id 一致だけで
    /// 無条件にインパルスを引き継ぎ、GC もしない)に戻る——対照実験専用。
    pub fn set_manifold_persistence(&mut self, enabled: bool) {
        self.contact_cache.persistence_enabled = enabled;
    }

    /// マニフォールド持続化キャッシュが保持している接触点数(GC の検証用)。
    pub fn cached_contact_point_count(&self) -> usize {
        self.contact_cache.len()
    }

    /// フルCCD(conservative advancement、設計 §4.6「フルCCD(Phase 5)」)の有効/無効。
    /// 既定は有効。`false` にすると最小CCD(speculative contact)だけが残る——
    /// 対照実験専用のスイッチ。
    pub fn set_full_ccd(&mut self, enabled: bool) {
        self.full_ccd_enabled = enabled;
    }

    /// 剛体ごとの運動エネルギー [J](並進+回転、`Solver::total_energy`の`kinetic`と
    /// 同じ式を剛体単位で評価したもの。非`Dynamic`剛体は 0)。
    /// `last_contact_dissipation_by_body`の算出に使う(**群5で追加**、同フィールドのdoc参照)。
    fn kinetic_energy_by_body(&self) -> Vec<f64> {
        (0..self.bodies.len())
            .map(|i| {
                if self.bodies.body_type[i] != BodyType::Dynamic {
                    return 0.0;
                }
                let mut ke = 0.5 * self.bodies.mass(i) * self.bodies.linear_velocity[i].length_sq();
                if let Some(inertia_world) = self.bodies.inv_inertia_world[i].inverse() {
                    let omega = self.bodies.angular_velocity[i];
                    ke += 0.5 * omega.dot(inertia_world.mul_vec(omega));
                }
                ke
            })
            .collect()
    }

    pub fn add_distance_joint(&mut self, joint: DistanceJoint) {
        self.joints.push(joint);
    }

    pub fn add_ball_joint(&mut self, joint: BallJoint) {
        self.ball_joints.push(joint);
    }

    pub fn add_slider_joint(&mut self, joint: SliderJoint) {
        self.slider_joints.push(joint);
    }

    pub fn add_hinge_motor(&mut self, motor: HingeMotorPd) {
        self.hinge_motors.push(motor);
    }

    /// 現在の重力場(`GravityField`のdoc参照)。
    pub fn gravity_field(&self) -> GravityField {
        self.field
    }

    /// 重力場をまるごと差し替える(`PointSource`/`Zero`へ到達する唯一の入口)。
    /// `Uniform`の`direction`はここでも正規化する——`set_gravity_direction`と
    /// 同じ不変条件(保持する向きは常に単位ベクトル、ゼロベクトルは既定の
    /// 下向きへフォールバック)を、場の与え方によらず成り立たせるため。
    pub fn set_gravity_field(&mut self, field: GravityField) {
        self.field = match field {
            GravityField::Uniform {
                magnitude,
                direction,
            } => GravityField::Uniform {
                magnitude,
                direction: normalized_or_default_direction(direction),
            },
            other => other,
        };
    }

    /// 重力加速度の**大きさ** [m/s^2]。
    ///
    /// **非`Uniform`な場では0.0を返す**(`Zero`は厳密に正しい。`PointSource`は
    /// 位置依存なのでスカラー1つでは表せず、代表点を発明するより0を返す方を
    /// 選んだ)。移行前の公開フィールド`gravity`の読み出しをそのまま置き換える
    /// アクセサであり、呼び出し側の書き換えは`()`を足すだけで済む。
    ///
    /// **これが隠していない挙動変化**: この値を使うのは(1)浮力
    /// (`apply_forces`の`buoyancy_force`)、(2)`sim-coupling`の
    /// Boussinesq浮力・自然対流のレイリー数、(3)Inspectorの環境パネル表示。
    /// いずれも「重力は鉛直方向に一様」を前提とする縮約モデルであり、
    /// `PointSource`場ではその前提自体が成立しない。0.0を返すことで
    /// **これらは無効化される**(水平な水面を仮定した浮力を点源場で出す方が
    /// 誤りが大きい)。浮力・自然対流を`acceleration_at`へ追従させるのは
    /// 別の計画作業であり、本増分の対象外(`GravityField`のdoc参照)。
    pub fn gravity(&self) -> f64 {
        self.field.uniform_magnitude()
    }

    /// 重力加速度の**向き**(正規化済み単位ベクトル)。
    /// **非`Uniform`な場では既定の下向き`(0,-1,0)`を返す**(向きが定義できない
    /// 場でゼロベクトルを返すと「単位ベクトルである」という呼び出し側の前提が
    /// 壊れるため)。`gravity()`が0.0を返すので、
    /// `gravity() * gravity_direction()`は`Zero`でも`PointSource`でも
    /// `Vec3::ZERO`——「この場を一様場1つで近似したベクトル」という意味で
    /// 一貫している(`Uniform`/`Zero`では厳密、`PointSource`では縮退)。
    pub fn gravity_direction(&self) -> Vec3 {
        self.field.uniform_direction()
    }

    /// 重力の大きさを設定する(移行前の`solver.gravity = g`の置き換え)。
    /// **非`Uniform`な場に対しては一様場への差し替えになる**——旧公開フィールドが
    /// 持っていた「大きさを決めれば重力が決まる」という意味をそのまま保つため
    /// (向きは`gravity_direction()`が返す値、すなわち既定の下向き)。
    /// 非一様な場を保ったまま強さだけを変えたい場合は`set_gravity_field`を使う。
    pub fn set_gravity(&mut self, magnitude: f64) {
        self.set_gravity_field(GravityField::Uniform {
            magnitude,
            direction: self.gravity_direction(),
        });
    }

    /// 重力の向きを設定する(`GravityField::Uniform::direction`のdoc参照)。
    /// ゼロベクトルは正規化できないため既定の下向きへフォールバックする
    /// (壊れた入力で重力が消えたり発散したりしないための安全側の縮退)。
    /// **移行前と挙動は完全に同一**(`Uniform`場の向きだけを差し替える)。
    ///
    /// 非`Uniform`な場に対しては、`set_gravity`と同じ理由で
    /// 「大きさ`gravity()`(=0.0)・向き`direction`の一様場」への差し替えになる。
    /// `Zero`からの遷移は数値的に無害(大きさ0のまま)だが、`PointSource`は
    /// 失われる——スカラー2つのAPIで書き込む操作は「一様場を選ぶ」ことだ、
    /// という一貫した規則を採る(黙って無視するより、規則が読める方がよい)。
    pub fn set_gravity_direction(&mut self, direction: Vec3) {
        self.set_gravity_field(GravityField::Uniform {
            magnitude: self.gravity(),
            direction,
        });
    }

    /// 位置`position`における重力加速度ベクトル [m/s^2]
    /// (`GravityField::acceleration_at`への薄い委譲)。
    pub fn gravity_at(&self, position: Vec3) -> Vec3 {
        self.field.acceleration_at(position)
    }

    /// 設計 §4 パイプラインの `apply_forces`。P1 スコープ: 重力 + 球の抗力
    /// (docs/11-fluid/05-aero-hydrodynamics.md §2.1)+ 直立直方体の浮力
    /// (docs/11-fluid/04-free-surface-buoyancy.md §2.1)。結合力は後続増分。
    fn apply_forces(&mut self) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] == BodyType::Dynamic && !self.bodies.asleep[i] {
                let mass = self.bodies.mass(i);
                // 重力は**剛体の位置で評価する**(一様場では位置に依存しないので
                // 移行前と完全に同一、点源場では剛体ごとに異なる)。剛体の広がりは
                // 無視して重心位置1点で代表する縮約——潮汐力(場の勾配が生む
                // トルク)は扱わない。
                let gravity = self.field.acceleration_at(self.bodies.position[i]);
                self.bodies.force_accum[i] = self.bodies.force_accum[i] + gravity.scale(mass);

                if let (Some(atm), DragModel::Sphere { radius }) =
                    (&self.atmosphere, self.bodies.drag[i])
                {
                    self.bodies.force_accum[i] = self.bodies.force_accum[i]
                        + sim_fluid::drag_force_sphere(radius, atm, self.bodies.linear_velocity[i]);
                }

                if let (Some(water), Shape::Box { half_extents }) =
                    (&self.water, self.bodies.shape_of(i))
                {
                    let (v_sub, _c_buoy) = sim_fluid::submerged_box_axis_aligned(
                        self.bodies.position[i],
                        *half_extents,
                        water.water_level,
                    );
                    // 浮心は直立対称箱では常に body 中心と同じ x,z を持ち、浮力は鉛直成分
                    // のみなのでトルクは厳密に0(r×F、r・Fが共にy軸方向で外積0)。
                    // トルク適用は不要(_c_buoy は式の対称性の記録として保持)。
                    if v_sub > 0.0 {
                        self.bodies.force_accum[i] = self.bodies.force_accum[i]
                            + sim_fluid::buoyancy_force(v_sub, water.density, self.gravity());
                    }
                }
            }
        }
    }

    /// `v += (F/m)dt`、`ω += I_w⁻¹(τ − ω×I_wω)dt`(ジャイロ項は既定で陽的、設計 §4/§9)。
    fn integrate_velocities(&mut self, dt: f64) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] != BodyType::Dynamic || self.bodies.asleep[i] {
                continue;
            }
            let accel = self.bodies.force_accum[i].scale(self.bodies.inv_mass[i]);
            self.bodies.linear_velocity[i] =
                self.bodies.linear_velocity[i].addcarry_scaled(accel, dt);

            let inv_iw = self.bodies.inv_inertia_world[i];
            if let Some(iw) = inv_iw.inverse() {
                let omega = self.bodies.angular_velocity[i];
                let gyro = omega.cross(iw.mul_vec(omega));
                let ang_accel = inv_iw.mul_vec(self.bodies.torque_accum[i] - gyro);
                self.bodies.angular_velocity[i] = omega.addcarry_scaled(ang_accel, dt);
            }
        }
    }

    /// `x += v dt`、`q = normalize(q + dt/2 * ω_quat ⊗ q)`(設計 §9)。
    /// Dynamic/Kinematic の両方が対象(Kinematic はスクリプトで速度が指定される)。
    fn integrate_positions(&mut self, dt: f64) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] == BodyType::Static {
                continue;
            }
            if self.bodies.body_type[i] == BodyType::Dynamic && self.bodies.asleep[i] {
                continue;
            }
            self.bodies.position[i] =
                self.bodies.position[i].addcarry_scaled(self.bodies.linear_velocity[i], dt);
            self.bodies.rotation[i] = self.bodies.rotation[i]
                .integrate_angular_velocity(self.bodies.angular_velocity[i], dt);
        }
    }

    /// ワールド慣性の相似変換キャッシュ更新 + アキュムレータのクリア(設計 §4 末尾)。
    fn update_inertia_and_clear_accum(&mut self) {
        let n = self.bodies.len();
        for i in 0..n {
            self.bodies.inv_inertia_world[i] =
                self.bodies.inv_inertia_local[i].similarity(self.bodies.rotation[i].to_mat3());
            self.bodies.force_accum[i] = sim_math::Vec3::ZERO;
            self.bodies.torque_accum[i] = sim_math::Vec3::ZERO;
        }
    }

    /// 少なくとも一方が「起きている dynamic body」なら解決対象(設計 §4「起床は新規接触・
    /// 力適用時」の反対: 両側とも寝ていれば新規に動く要素が無い)。
    fn manifold_is_active(&self, m: &collision::ContactManifold) -> bool {
        let a_awake_dynamic =
            self.bodies.body_type[m.body_a] == BodyType::Dynamic && !self.bodies.asleep[m.body_a];
        let b_awake_dynamic =
            self.bodies.body_type[m.body_b] == BodyType::Dynamic && !self.bodies.asleep[m.body_b];
        a_awake_dynamic || b_awake_dynamic
    }

    /// 前stepの接触ペア集合との差分から`ContactStarted`/`ContactEnded`イベントを
    /// `ctx.events`へ発行する(`contact_pairs`フィールドdoc参照)。`Event::step`は
    /// このソルバがワールド全体のstep_countを知らないため`0`で埋め、呼び出し側
    /// (`World::step`)がイベント排出時に正しい値へ上書きする(設計上の全体順序は
    /// 「同一stepの排出集合」として保たれるため、ここでは各イベントに一貫した
    /// プレースホルダを入れておけば十分)。
    fn emit_contact_events(
        &mut self,
        manifolds: &[collision::ContactManifold],
        ctx: &mut SolverContext,
    ) {
        let current_pairs: HashSet<(usize, usize)> =
            manifolds.iter().map(|m| (m.body_a, m.body_b)).collect();
        let source_id = |a: usize, b: usize| SourceId(((a as u64) << 32) | b as u64);
        for &(a, b) in current_pairs.difference(&self.contact_pairs) {
            ctx.events.push(Event {
                step: 0,
                source: source_id(a, b),
                kind: EventKind::ContactStarted,
            });
        }
        for &(a, b) in self.contact_pairs.difference(&current_pairs) {
            ctx.events.push(Event {
                step: 0,
                source: source_id(a, b),
                kind: EventKind::ContactEnded,
            });
        }
        self.contact_pairs = current_pairs;
    }
}

impl Solver for MechanicsSolver {
    /// sequential impulses は固定 dt 前提の速度レベル解法で明示的な CFL 条件を持たない
    /// (Box2D 系と同様)。拘束(ジョイント)導入時に硬い系の刻み制約を追加検討する。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, ctx: &mut SolverContext) {
        self.apply_forces();
        joint::apply_hinge_motors(&self.hinge_motors, &mut self.bodies, dt);
        self.integrate_velocities(dt);
        // 処理順「ジョイント→接触」(設計 docs/10-mechanics/05-joints-constraints.md §4.1)。
        joint::resolve_distance(&self.joints, &mut self.bodies, dt);
        joint::resolve_ball(&self.ball_joints, &mut self.bodies, dt);
        joint::resolve_slider(&self.slider_joints, &mut self.bodies, dt);
        joint::resolve_wheel(&self.wheel_joints, &mut self.bodies, dt);
        joint::resolve_hinge_limits(&self.hinge_motors, &mut self.bodies, dt);
        let manifolds = collision::detect(&self.bodies, &mut self.axis_cache);
        self.emit_contact_events(&manifolds, ctx);
        self.last_manifolds = manifolds.clone();
        // 両側の dynamic body が全て asleep な接触は再解決しない(収束済みで変化が無いのに
        // 毎ステップ再解決すると warm start・split impulse の数値的な揺らぎで再起床してしまう
        // ことを実装検証中に発見した — 「積分を停止」だけでは不十分で、接触解決自体も
        // 停止する必要がある、設計 docs/10-mechanics/01-rigid-body.md §4)。
        let active_manifolds: Vec<collision::ContactManifold> = manifolds
            .iter()
            .filter(|m| self.manifold_is_active(m))
            .cloned()
            .collect();
        let ke_before_contact = self.total_energy().kinetic;
        // 剛体ごとの運動エネルギーも控えておく(群5、`last_contact_dissipation_by_body`の
        // doc参照)。接触解決の前後で同じ関数を使うので、総和は必ず全体値と一致する。
        let ke_by_body_before = self.kinetic_energy_by_body();
        contact::resolve(
            &active_manifolds,
            &mut self.bodies,
            ctx.materials,
            self.restitution_velocity_threshold,
            &mut self.contact_cache,
        );
        let ke_after_contact = self.total_energy().kinetic;
        let ke_by_body_after = self.kinetic_energy_by_body();
        self.last_contact_dissipation_by_body = ke_by_body_before
            .iter()
            .zip(ke_by_body_after.iter())
            .map(|(before, after)| before - after)
            .collect();
        debug_assert!(
            ke_after_contact <= ke_before_contact + 1e-6 * ke_before_contact.max(1.0),
            "contact resolution must not increase kinetic energy beyond numerical noise: \
             before={ke_before_contact} after={ke_after_contact}"
        );
        self.last_contact_dissipation = ke_before_contact - ke_after_contact;
        // 接触が完全に消えたボディ対のエントリを捨てる(設計 §4.7、GC)。スリープで
        // ソルバをスキップしたペアも「生きている接触」として渡す(`retain_pairs` doc参照)。
        let live_pairs: std::collections::BTreeSet<(usize, usize)> =
            manifolds.iter().map(|m| (m.body_a, m.body_b)).collect();
        self.contact_cache.retain_pairs(&live_pairs);
        // 接触解決後(post-solve)の速度で静止判定する(解決前は重力積分直後でまだ抗力が
        // 相殺していないため静止判定に使えない)。島判定には(スキップした分も含め)
        // 全マニフォールドを使う。
        sleep::update_sleep_state(&mut self.bodies, &manifolds, dt);
        // 最小CCD(speculative contact、設計§4.6)。既存の実接触解決のあとに、まだ検出
        // されていない今ステップ中のすり抜けだけを速度クランプで防ぐ(P1標準機能)。
        ccd::apply_speculative_contacts(&mut self.bodies, dt);
        // フルCCD(conservative advancement、設計§4.6「フルCCD(Phase 5)」、**群9で配線**)。
        // speculative pass が原理的に扱えない「球以外の弾丸」「動的な相手」を受け持つ。
        if self.full_ccd_enabled {
            ccd::apply_conservative_advancement(&mut self.bodies, dt);
        }
        self.integrate_positions(dt);
        self.update_inertia_and_clear_accum();
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        let n = self.bodies.len();
        hasher.write_u64(n as u64);
        for i in 0..n {
            hasher.write_vec3(self.bodies.position[i]);
            hasher.write_quat(self.bodies.rotation[i]);
            hasher.write_vec3(self.bodies.linear_velocity[i]);
            hasher.write_vec3(self.bodies.angular_velocity[i]);
        }
    }

    /// Dynamic 剛体の運動エネルギー(並進+回転)+ 重力ポテンシャル
    /// (`GravityField::potential_per_mass`。一様場は基準 原点=移行前と同一、
    /// 点源場は基準 無限遠)。
    /// Kinematic の運動は外部注入エネルギーとして台帳側(World)が扱うため、ここでは対象外
    /// (docs/00-foundation/04-architecture.md §1.1.2(2))。
    fn total_energy(&self) -> EnergyBreakdown {
        let mut kinetic = 0.0;
        let mut potential = 0.0;
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] != BodyType::Dynamic {
                continue;
            }
            let mass = self.bodies.mass(i);
            kinetic += 0.5 * mass * self.bodies.linear_velocity[i].length_sq();
            if let Some(inertia_world) = self.bodies.inv_inertia_world[i].inverse() {
                let omega = self.bodies.angular_velocity[i];
                kinetic += 0.5 * omega.dot(inertia_world.mul_vec(omega));
            }
            potential += mass * self.field.potential_per_mass(self.bodies.position[i]);
        }
        EnergyBreakdown {
            kinetic,
            potential,
            ..Default::default()
        }
    }

    fn approximations(&self) -> Vec<Approximation> {
        let mut out = vec![
            Approximation {
                name: "接触: PGS + Baumgarte",
                reason: "厳密なLCPではなく逐次射影で解くため、深い積み重ねでは\
                         わずかな貫入が残る。位置補正はBaumgarteで、これが\
                         エネルギー台帳に小さな人工的増加を生む。",
                doc: "docs/10-mechanics/03-contact-solver.md",
                can_disable: false,
            },
            Approximation {
                name: "マニフォールドは最大4点",
                reason: "接触点を深い順に4点へ縮約する(設計§4.4の簡略版)。",
                doc: "docs/10-mechanics/02-collision-detection.md",
                can_disable: false,
            },
        ];
        if self.water.is_some() {
            out.push(Approximation {
                name: "浮力: 静的水域(集中定数)",
                reason: "自由表面を追跡せず、水面の高さと密度だけで浮力を出す。\
                         物体が入っても水位は変わらない。",
                doc: "docs/11-fluid/04-free-surface-buoyancy.md",
                can_disable: false,
            });
        }
        if self.atmosphere.is_some() {
            out.push(Approximation {
                name: "空気抗力: 集中定数",
                reason: "格子流体との連成ではなく、抗力係数と相対速度から直接力を出す。\
                         揚力の式は sim-fluid に無いため計上しない。",
                doc: "docs/11-fluid/05-aero-hydrodynamics.md",
                can_disable: false,
            });
        }
        out
    }
}

/// Phase 0 の `FallingBody`(回転なし・接触なし)相当を、正式な `RigidBodySet` +
/// `MechanicsSolver` 経由で再現できることを確認する M1 相当のテスト。
#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;
    use sim_core::{EventQueue, MaterialDb};
    use sim_math::{Quat, SimRng, Vec3};

    fn make_ctx<'a>(
        materials: &'a MaterialDb,
        rng: &'a mut SimRng,
        events: &'a mut EventQueue,
    ) -> SolverContext<'a> {
        SolverContext {
            materials,
            rng,
            events,
        }
    }

    /// M1: 自由落下 h=10m の到達時刻 t*=sqrt(2h/g)=1.4278s、相対誤差 0.5% 以内
    /// (docs/21-verification/01-analytic-tests.md M1)。
    #[test]
    fn m1_free_fall_matches_analytic_time_to_ground() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.transform.position = Vec3::new(0.0, 10.0, 0.0);
        let idx = solver.create_body(desc, &materials);

        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        while solver.bodies.position[idx].y > 0.0 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            t += dt;
        }

        let analytic = (2.0 * 10.0 / 9.80665_f64).sqrt();
        assert!(
            (t - analytic).abs() / analytic < 0.005,
            "t={t} analytic={analytic}"
        );
    }

    /// **残タスク完遂増分**(レビュー指摘「見送らず対応すること」への対応):
    /// `set_gravity_direction`で重力を`+x`向きへ変えると、m1
    /// (`m1_free_fall_matches_analytic_time_to_ground`)と全く同じ形の解析解
    /// (`t=sqrt(2d/g)`、軸を`y`から`x`へ入れ替えただけ)に従い、`y`は不変の
    /// ままであること——「大きさのみ可変・向きは`-y`固定」だった制約が
    /// 実際に解消されたことの直接的な検証。m1と同じ判定方法(同じ許容誤差)を
    /// 使う——数値積分(semi-implicit Euler)には既知の系統誤差があるため、
    /// 独自の許容誤差を発明せず、既にこのソルバで検証済みの手法をそのまま
    /// 軸だけ変えて再利用する。
    #[test]
    fn gravity_direction_can_be_changed_and_free_fall_follows_the_new_axis() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        solver.set_gravity_direction(Vec3::new(1.0, 0.0, 0.0));
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.transform.position = Vec3::new(0.0, 10.0, 0.0);
        let idx = solver.create_body(desc, &materials);

        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        while solver.bodies.position[idx].x < 10.0 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            t += dt;
        }

        let analytic = (2.0 * 10.0 / 9.80665_f64).sqrt();
        assert!(
            (t - analytic).abs() / analytic < 0.005,
            "t={t} analytic={analytic}"
        );
        assert!(
            (solver.bodies.position[idx].y - 10.0).abs() < 1e-9,
            "gravity along +x must not move the body along y: pos.y={}",
            solver.bodies.position[idx].y
        );
    }

    #[test]
    fn static_body_does_not_move_under_gravity() {
        let materials = MaterialDb::standard();
        let concrete = materials.find_by_name("コンクリート").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(1.0, 1.0, 1.0),
            },
            concrete,
        );
        desc.body_type = BodyType::Static;
        let idx = solver.create_body(desc, &materials);

        for _ in 0..120 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(1.0 / 120.0, &mut ctx);
        }
        assert_eq!(solver.bodies.position[idx], Vec3::ZERO);
    }

    #[test]
    fn kinematic_body_moves_at_prescribed_velocity_without_gravity() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel);
        desc.body_type = BodyType::Kinematic;
        desc.linear_velocity = Vec3::new(1.0, 0.0, 0.0);
        let idx = solver.create_body(desc, &materials);

        let dt = 1.0 / 120.0;
        for _ in 0..120 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
        }
        assert!((solver.bodies.position[idx].x - 1.0).abs() < 1e-9);
        assert!(
            (solver.bodies.position[idx].y - 0.0).abs() < 1e-12,
            "gravity must not affect kinematic bodies"
        );
    }

    /// 決定論: 同一初期条件を2回実行 → state_hash が一致する。
    #[test]
    fn determinism_same_scenario_twice_matches_hash() {
        let run = || {
            let materials = MaterialDb::standard();
            let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
            let mut rng = SimRng::new(7, 7);
            let mut events = EventQueue::new();
            let mut solver = MechanicsSolver::new(9.80665);
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
            desc.transform.position = Vec3::new(0.0, 5.0, 0.0);
            solver.create_body(desc, &materials);
            for _ in 0..300 {
                let mut ctx = make_ctx(&materials, &mut rng, &mut events);
                solver.step(1.0 / 120.0, &mut ctx);
            }
            let mut hasher = StateHasher::new();
            solver.state_hash(&mut hasher);
            hasher.finish()
        };
        assert_eq!(run(), run());
    }

    /// エンティティ層の受け入れテスト(docs/20-integration/03-entity-layer.md §7
    /// 「静的姿勢維持: 関節PDのみで外乱なしのしゃがみ姿勢を60秒維持(転倒しない、
    /// 関節角ドリフト<5°)」)。倒立平衡(バランス制御)を含まない設計の指示どおり、
    /// 完全な15剛体の人体骨格ではなく、ワールド固定ピボット(股関節)に`BallJoint`で
    /// 繋がれた単一の脚リンクが、地面(`Plane`)に足先で接地しつつ`HingeMotorPd`が
    /// 45°のしゃがみ角を保持する縮約構成(モジュールdocの`joint::HingeMotorPd`参照)で
    /// 検証する — 「関節PD × 接触ソルバの結合」という設計が明記する検証対象そのものは、
    /// この縮約構成でも(ピボット+接地の両方が同時に働くため)保たれる。設計§4.5既定の
    /// PDゲイン(kp=20 s⁻¹, kd=2)をそのまま使用したところ、60秒間の最大ドリフトは
    /// 約3.8°(基準5°以内)、足先接地点は地面にめり込まず(min_tip_yが正、接触ソルバが
    /// 支えている)であることを実装検証中に確認した。
    #[test]
    fn entity_layer_hinge_motor_maintains_crouch_pose_for_60s_with_ground_contact() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(3, 3);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);

        let mut ground = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        ground.body_type = BodyType::Static;
        solver.create_body(ground, &materials);

        let theta_target = std::f64::consts::FRAC_PI_4; // 45°(しゃがみ角)
        let half_extents = Vec3::new(0.05, 0.4, 0.05);
        let anchor_local_top = Vec3::new(0.0, half_extents.y, 0.0);
        let anchor_local_bottom = Vec3::new(0.0, -half_extents.y, 0.0);
        let rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), theta_target);

        // 45°姿勢で足先(anchor_local_bottom)がちょうど地面(y=0)に接地するようピボットを選ぶ
        // (プログラム的に算出、手計算の符号取り違えを避ける)。
        let bottom_offset_from_pivot =
            rotation.rotate(anchor_local_bottom) - rotation.rotate(anchor_local_top);
        let pivot = Vec3::new(0.0, -bottom_offset_from_pivot.y, 0.0);
        let body_center = pivot - rotation.rotate(anchor_local_top);

        let mut leg_desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, steel);
        leg_desc.transform.position = body_center;
        leg_desc.transform.rotation = rotation;
        leg_desc.mass_override = Some(5.0);
        let leg = solver.create_body(leg_desc, &materials);

        solver.add_ball_joint(BallJoint {
            body_a: leg,
            anchor_a: anchor_local_top,
            body_b: None,
            anchor_b: pivot,
            disabled: false,
        });
        solver.add_hinge_motor(HingeMotorPd {
            body: leg,
            axis: Vec3::new(0.0, 0.0, 1.0),
            reference_rotation: Quat::IDENTITY,
            theta_target,
            kp: 20.0,
            kd: 2.0,
            torque_max: 50.0,
            limit: None,
            disabled: false,
        });

        let dt = 1.0 / 120.0;
        let steps = 60 * 120;
        let mut max_drift: f64 = 0.0;
        let mut min_tip_y: f64 = f64::INFINITY;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);

            assert!(
                solver.bodies.position[leg].x.is_finite()
                    && solver.bodies.position[leg].y.is_finite()
                    && solver.bodies.position[leg].z.is_finite(),
                "solver diverged: position={:?}",
                solver.bodies.position[leg]
            );

            let theta = solver.hinge_motors[0].measure_angle(&solver.bodies);
            max_drift = max_drift.max((theta - theta_target).abs());

            let tip = solver.bodies.position[leg]
                + solver.bodies.rotation[leg].rotate(anchor_local_bottom);
            min_tip_y = min_tip_y.min(tip.y);
        }

        let max_drift_deg = max_drift.to_degrees();
        assert!(
            max_drift_deg < 5.0,
            "joint angle drift too large: {max_drift_deg:.3} deg"
        );
        assert!(
            min_tip_y > -0.02,
            "foot penetrated the ground beyond contact slop: min_tip_y={min_tip_y:.5}"
        );
    }

    /// `SliderJoint`(設計 §4.4「Slider | 5 | 軸直交並進2 + 相対回転固定3」)の受け入れ:
    /// ワールドx軸に沿って自由に滑る「ピストンロッド」(ワールド固定シリンダー、
    /// `body_b=None`)が、(1)重力下でも軸に直交するy/zへ落下・ドリフトしない
    /// (直交並進2行が拘束)、(2)姿勢が生成時の基準(恒等回転)から傾かない
    /// (相対回転固定3行が拘束)、(3)軸方向(x)には初速のまま自由に(抵抗なく)進み続ける
    /// (拘束されない1自由度)ことを確認する — 断熱圧縮の`PistonGas`結合が前提とする
    /// 「シリンダー壁は軸直交方向・回転を拘束し、軸方向のみ自由」という構成そのもの。
    #[test]
    fn slider_joint_constrains_perpendicular_translation_and_rotation_but_frees_the_axis() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(11, 11);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        let piston = solver.create_body(desc, &materials);

        solver.add_slider_joint(SliderJoint::new(
            &solver.bodies,
            piston,
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            None,
            Vec3::ZERO,
        ));

        let dt = 1.0 / 120.0;
        let steps = 240; // 2秒: 軸方向に2.0*2.0=4.0m進む間の直交ドリフト/姿勢ドリフトを見る
        let mut max_perp: f64 = 0.0;
        let mut max_tilt_deg: f64 = 0.0;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            let pos = solver.bodies.position[piston];
            max_perp = max_perp.max(pos.y.abs()).max(pos.z.abs());
            let rot = solver.bodies.rotation[piston];
            // 恒等回転からの角度 = 2*acos(|w|)(最短経路、二重被覆を考慮)。
            let tilt = 2.0 * rot.w.abs().min(1.0).acos();
            max_tilt_deg = max_tilt_deg.max(tilt.to_degrees());
        }

        assert!(
            max_perp < 0.01,
            "slider should not drift perpendicular to its axis under gravity: max_perp={max_perp:.5}"
        );
        assert!(
            max_tilt_deg < 1.0,
            "slider should not rotate relative to its fixed reference orientation: max_tilt_deg={max_tilt_deg:.3}"
        );
        let expected_x = 2.0 * (steps as f64 * dt);
        let actual_x = solver.bodies.position[piston].x;
        assert!(
            (actual_x - expected_x).abs() / expected_x < 0.01,
            "slider's free axis should move ballistically at the initial velocity: actual_x={actual_x} expected_x={expected_x}"
        );
    }

    /// 接触イベント(`emit_contact_events`、`World`最初のイベント生産者、設計
    /// docs/00-foundation/04-architecture.md §1.1.2(5))。反発係数の高い球を落下させ、
    /// 着地時に`ContactStarted`が、跳ね上がって離れた時に`ContactEnded`が
    /// (前stepとの接触ペア集合の差分として)発行されることを確認する。
    #[test]
    fn bouncing_ball_emits_contact_started_and_ended_events() {
        let mut materials = MaterialDb::standard();
        let bouncy = materials.push(sim_core::Material {
            name: "test-bouncy-for-contact-events",
            density: 1000.0,
            friction: 0.0,
            restitution: 0.8,
            youngs_modulus: None,
            specific_heat: 1000.0,
            conductivity: 1.0,
            emissivity: 0.5,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "test fixture",
            uncertainty: 0.0,
        });
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        solver.restitution_velocity_threshold = 0.0;
        let mut floor = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            bouncy,
        );
        floor.body_type = BodyType::Static;
        solver.create_body(floor, &materials);

        let mut ball_desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.5 }, bouncy);
        ball_desc.transform.position = Vec3::new(0.0, 2.0, 0.0);
        solver.create_body(ball_desc, &materials);

        let dt = 1.0 / 120.0;
        let mut started_count = 0;
        let mut ended_count = 0;
        for _ in 0..300 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            for e in events.drain_sorted() {
                match e.kind {
                    sim_core::EventKind::ContactStarted => started_count += 1,
                    sim_core::EventKind::ContactEnded => ended_count += 1,
                    _ => {}
                }
            }
        }

        assert!(
            started_count >= 1,
            "should observe at least one ContactStarted when the ball lands"
        );
        assert!(
            ended_count >= 1,
            "should observe at least one ContactEnded when the ball bounces back up"
        );
    }

    // ------------------------------------------------------------------
    // 解析解による足場(**物理コア変更の前提となる回帰ハーネス**)。
    //
    // `state_hash`の一致は決定性しか示さない——「毎回同じように間違える」
    // 実装でもハッシュは一致する。以下の群は、閉形式の解析解(トルクフリー
    // 剛体のEuler方程式・単振り子の微小振幅周期)をテスト内で独立に計算し、
    // シミュレーション結果と数値的に突き合わせることで**正しさ**を固定する。
    // ------------------------------------------------------------------

    fn child_at(position: Vec3, half: f64) -> (sim_math::Transform, Shape) {
        (
            sim_math::Transform {
                position,
                rotation: Quat::IDENTITY,
            },
            Shape::Box {
                half_extents: Vec3::new(half, half, half),
            },
        )
    }

    /// ローカル原点まわりに**対称配置**した4つの立方体からなる複合剛体。
    /// x軸上 ±0.4 と y軸上 ±0.2 に等しい部品を置くので、部品配置の重心は厳密に
    /// ローカル原点と一致する——`Shape::Compound`の慣性計算が置く「ローカル原点
    /// =重心」という仮定(`unit_mass_inertia_diagonal`のdoc「簡略化(既知の限界)」)
    /// が**この配置では厳密に正しい**。重心オフセットを`RigidBodySet`へ持ち込む
    /// 将来の変更でも、この対称配置の挙動だけは一切変わってはならない。
    ///
    /// 部品はすべて無回転なので、対角テンソルの非対角成分切り捨て近似も
    /// 誤差ゼロ(同doc)。つまり本剛体の慣性は**近似ではなく厳密**であり、
    /// 完全な`Mat3`慣性テンソルへ移行しても値は変わらない。
    fn symmetric_compound() -> Shape {
        let half = 0.05;
        Shape::Compound {
            children: vec![
                child_at(Vec3::new(0.4, 0.0, 0.0), half),
                child_at(Vec3::new(-0.4, 0.0, 0.0), half),
                child_at(Vec3::new(0.0, 0.2, 0.0), half),
                child_at(Vec3::new(0.0, -0.2, 0.0), half),
            ],
        }
    }

    /// ローカル原点まわりに**非対称**な複合剛体(群11で追加)。
    ///
    /// 部品の大きさも位置もばらばらなので、
    /// ① 重心はローカル原点から明確にずれ、
    /// ② 部品が座標軸から外れた位置(x,y 同時にオフセット)にあるため
    ///    慣性テンソルに**慣性乗積(非対角成分)が出る**。
    ///
    /// 移行前の実装はこの2点をどちらも表現できなかった(重心=ローカル原点を
    /// 決め打ちし、慣性は対角`Vec3`しか返せなかった)ので、この剛体は
    /// 「群11で新たに正しく積分できるようになったもの」そのものである。
    fn asymmetric_compound() -> Shape {
        Shape::Compound {
            children: vec![
                child_at(Vec3::new(0.5, 0.0, 0.0), 0.09),
                child_at(Vec3::new(-0.2, 0.3, 0.0), 0.05),
                child_at(Vec3::new(0.1, -0.35, 0.15), 0.07),
            ],
        }
    }

    /// ワールド系の角運動量 $L = I_{world}\,\omega$。
    fn angular_momentum(solver: &MechanicsSolver, idx: usize) -> Vec3 {
        solver.bodies.inv_inertia_world[idx]
            .inverse()
            .expect("dynamic body must have an invertible inertia tensor")
            .mul_vec(solver.bodies.angular_velocity[idx])
    }

    /// トルクフリーの自由回転を`steps`ステップ回し、
    /// `(|ΔL|/|L₀| の最大値, |ΔT|/T₀ の最大値)`を返す。
    fn torque_free_tumble_drift(omega0: Vec3, dt: f64, steps: usize) -> (f64, f64) {
        torque_free_tumble_drift_of(symmetric_compound(), omega0, dt, steps)
    }

    /// `torque_free_tumble_drift`の形状を差し替えられる版(群11で追加——
    /// 重心がローカル原点からずれた**非対称**な複合剛体でも同じ保存則が
    /// 成り立つことを確かめるため)。
    fn torque_free_tumble_drift_of(
        shape: Shape,
        omega0: Vec3,
        dt: f64,
        steps: usize,
    ) -> (f64, f64) {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        // 重力0・接触相手なし・抗力なし → 外力・外トルクは厳密にゼロ。
        let mut solver = MechanicsSolver::new(0.0);
        let mut desc = RigidBodyDesc::dynamic(shape, steel);
        desc.angular_velocity = omega0;
        let idx = solver.create_body(desc, &materials);

        let l0 = angular_momentum(&solver, idx);
        let ke0 = solver.total_energy().kinetic;
        let mut max_l_drift: f64 = 0.0;
        let mut max_ke_drift: f64 = 0.0;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            let _: Vec<Event> = events.drain_sorted();
            max_l_drift =
                max_l_drift.max((angular_momentum(&solver, idx) - l0).length() / l0.length());
            max_ke_drift = max_ke_drift.max((solver.total_energy().kinetic - ke0).abs() / ke0);
        }
        (max_l_drift, max_ke_drift)
    }

    /// **トルクフリー剛体の保存則**(Euler方程式)。外力・外トルクが厳密にゼロなら
    /// ワールド系の角運動量ベクトル $L=I_{world}\omega$ と回転運動エネルギー
    /// $T=\frac12\omega\cdot I_{world}\omega$ はどちらも**厳密な保存量**である。
    ///
    /// 主軸に一致しない初期角速度を与えて実際に歳差(tumbling)させる——主軸に
    /// 沿った回転はジャイロ項 $\omega\times I\omega$ が恒等的に0になる離散スキームの
    /// 不動点で、機械精度で保存してしまい積分器の検証にならないため。
    ///
    /// 許容誤差の根拠: ジャイロ項は陽的(explicit)に評価される
    /// (`integrate_velocities`のdoc「ジャイロ項は既定で陽的」)ので、保存量の
    /// 誤差は $O(\Delta t)$ で蓄積する。dt=1/1000・4000ステップ(4秒、|ω|≈4.3 rad/s で
    /// 約17回転)の実測は |ΔL|/|L₀| ≈ 3.7e-3、|ΔT|/T₀ ≈ 4.5e-3 なので、
    /// 3割ほどの余裕を見て 5e-3 / 6e-3 を上限とする。
    ///
    /// さらに **dt を半分にすると誤差もほぼ半分になる**(1次収束)ことを併せて
    /// 確認する——これにより残差が「積分器の刻み誤差」であって「慣性テンソルや
    /// ジャイロ項の式の誤り」ではないことまで固定できる(定数倍ずれた慣性を
    /// 使っていれば誤差は dt に依らず残る)。
    #[test]
    fn torque_free_compound_conserves_angular_momentum_and_kinetic_energy() {
        // 主軸(ローカルx/y/z)のいずれとも一致しない初期角速度。
        let omega0 = Vec3::new(1.5, 0.5, 4.0);
        let (l_drift, ke_drift) = torque_free_tumble_drift(omega0, 1.0 / 1000.0, 4000);
        assert!(
            l_drift < 5e-3,
            "angular momentum must be conserved under zero torque: |dL|/|L0|={l_drift:.3e}"
        );
        assert!(
            ke_drift < 6e-3,
            "rotational kinetic energy must be conserved under zero torque: |dT|/T0={ke_drift:.3e}"
        );

        // 刻みを半分にすれば誤差も半分になる(陽的ジャイロ項の1次収束)。
        let (l_half, ke_half) = torque_free_tumble_drift(omega0, 1.0 / 2000.0, 8000);
        assert!(
            l_half < l_drift / 1.7 && ke_half < ke_drift / 1.7,
            "halving dt should roughly halve the drift (first order): \
             l={l_drift:.3e}->{l_half:.3e} ke={ke_drift:.3e}->{ke_half:.3e}"
        );
    }

    /// **非対称な複合剛体でもトルクフリーの保存則が成り立つ**(群11で追加)。
    ///
    /// `torque_free_compound_conserves_angular_momentum_and_kinetic_energy`は
    /// **対称**な複合剛体しか見ておらず、その配置では「ローカル原点=重心」
    /// という移行前の仮定がたまたま厳密に成り立つため、重心オフセットの
    /// 実装が正しいかを一切検証できない。ここでは重心がローカル原点から
    /// ずれ、かつ慣性乗積(非対角成分)を持つ剛体で同じ保存則を要求する。
    ///
    /// 角運動量 $L=I_{world}\omega$ と回転運動エネルギーは、慣性テンソルが
    /// 非対角であっても外トルクがゼロなら厳密な保存量である。もし
    /// ① 慣性テンソルを重心まわりではなくローカル原点まわりに組んでいたり、
    /// ② 非対角成分を捨てていたりすれば、$I_{world}$ の相似変換と
    /// ジャイロ項 $\omega\times I\omega$ の整合が崩れ、保存量は $O(1)$ で
    /// 破れる(dtを細かくしても収束しない)。
    ///
    /// 許容誤差の根拠は対称版と同じ——陽的ジャイロ項の $O(\Delta t)$ 蓄積。
    /// dt=1/1000・4000ステップの実測は |ΔL|/|L₀| = 1.22e-3、|ΔT|/T₀ = 2.50e-3
    /// (対称版より小さい)。対称版と同じ上限 5e-3 / 6e-3 をそのまま使う
    /// (実測に対して4倍・2.4倍の余裕)。あわせて**dtを半分にすれば誤差も
    /// ほぼ半分**(1次収束、実測の比は 2.03 / 2.03)を確認する——これにより
    /// 残差が刻み誤差であって慣性テンソルの誤りではないことまで固定できる。
    #[test]
    fn torque_free_asymmetric_compound_also_conserves_angular_momentum_and_energy() {
        // この剛体が本当に「非対称かつ慣性乗積つき」であることを先に固定する
        // (退化した設定で保存則だけ通っても意味がないため)。
        let shape = asymmetric_compound();
        let com = shape.center_of_mass();
        assert!(
            com.length() > 0.05,
            "重心がローカル原点から有意にずれている必要がある: {com:?}"
        );
        let tensor = shape.unit_mass_inertia_tensor();
        let max_off_diagonal = [tensor.m[0][1], tensor.m[0][2], tensor.m[1][2]]
            .into_iter()
            .fold(0.0_f64, |a, b| a.max(b.abs()));
        assert!(
            max_off_diagonal > 1e-3,
            "慣性乗積が有意に出ている必要がある: {tensor:?}"
        );

        let omega0 = Vec3::new(1.5, 0.5, 4.0);
        let (l_drift, ke_drift) =
            torque_free_tumble_drift_of(shape.clone(), omega0, 1.0 / 1000.0, 4000);
        assert!(
            l_drift < 5e-3,
            "angular momentum must be conserved under zero torque: |dL|/|L0|={l_drift:.3e}"
        );
        assert!(
            ke_drift < 6e-3,
            "rotational kinetic energy must be conserved under zero torque: |dT|/T0={ke_drift:.3e}"
        );

        let (l_half, ke_half) = torque_free_tumble_drift_of(shape, omega0, 1.0 / 2000.0, 8000);
        assert!(
            l_half < l_drift / 1.7 && ke_half < ke_drift / 1.7,
            "halving dt should roughly halve the drift (first order): \
             l={l_drift:.3e}->{l_half:.3e} ke={ke_drift:.3e}->{ke_half:.3e}"
        );
    }

    /// **テニスラケット定理(中間軸定理)**。主慣性モーメントが $I_1<I_2<I_3$ と
    /// すべて異なる剛体では、最小軸・最大軸まわりの自由回転は安定だが、
    /// **中間軸**まわりの回転は不安定で、微小な擾乱が指数的に成長して回転軸が
    /// 反転する。線形化したEuler方程式から成長率は
    /// $\sigma=\omega_2\sqrt{\frac{(I_2-I_1)(I_3-I_2)}{I_1 I_3}}$、
    /// 相対擾乱 $\varepsilon$ が $O(1)$ まで育つ時刻は $t\approx\ln(1/\varepsilon)/\sigma$。
    ///
    /// これは「慣性テンソルとジャイロ項が正しく組めているか」に対する非常に強い
    /// 定性的シグナルで、たとえば主軸の取り違えや慣性の定数倍ずれがあれば
    /// 反転は起きない(あるいは起きるべきでない軸で起きる)。
    ///
    /// 許容誤差の根拠: 成長率の式は線形近似なので反転時刻の予測も概算にとどまる
    /// (実測 0.75s に対し予測 0.60s)。定数倍の安全域を見て
    /// 「予測時刻の 0.5〜2.5 倍の窓で反転する」ことのみを要求する。安定軸側は
    /// 5秒間(反転予測時刻の約8倍)まったく符号が変わらないことを要求する。
    #[test]
    fn tennis_racket_theorem_flips_only_around_the_intermediate_axis() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let shape = symmetric_compound();
        let inertia = shape.unit_mass_inertia_diagonal();
        // ローカルx=最小、y=中間、z=最大(対称配置の設計どおり)。
        assert!(
            inertia.x < inertia.y && inertia.y < inertia.z,
            "test fixture must have three distinct principal moments: {inertia:?}"
        );

        let spin = 12.0;
        let epsilon = 0.005_f64; // 主軸からの相対擾乱
        let growth_rate = spin
            * (((inertia.y - inertia.x) * (inertia.z - inertia.y)) / (inertia.x * inertia.z))
                .sqrt();
        let predicted_flip_time = (1.0 / epsilon).ln() / growth_rate;

        let dt = 1.0 / 2000.0;
        let steps = (5.0 / dt) as usize;
        // 与えた主軸まわりの(剛体ローカル系での)角速度成分が符号を変えた時刻を返す。
        let first_sign_flip = |omega0: Vec3, component: fn(Vec3) -> f64| -> Option<f64> {
            let mut rng = SimRng::new(1, 1);
            let mut events = EventQueue::new();
            let mut solver = MechanicsSolver::new(0.0);
            let mut desc = RigidBodyDesc::dynamic(shape.clone(), steel);
            desc.angular_velocity = omega0;
            let idx = solver.create_body(desc, &materials);
            for s in 0..steps {
                let mut ctx = make_ctx(&materials, &mut rng, &mut events);
                solver.step(dt, &mut ctx);
                let _: Vec<Event> = events.drain_sorted();
                // ローカル系の角速度(姿勢の共役で世界→ローカルへ戻す)。
                let omega_local = solver.bodies.rotation[idx]
                    .conjugate()
                    .rotate(solver.bodies.angular_velocity[idx]);
                if component(omega_local) < 0.0 {
                    return Some(s as f64 * dt);
                }
            }
            None
        };

        // 中間軸(y)まわり: 反転する。
        let flip = first_sign_flip(Vec3::new(epsilon * spin, spin, 0.0), |w| w.y)
            .expect("spin about the intermediate axis must flip within the simulated window");
        assert!(
            flip > 0.5 * predicted_flip_time && flip < 2.5 * predicted_flip_time,
            "flip time should be near the analytic estimate ln(1/eps)/sigma: \
             flip={flip:.3} predicted={predicted_flip_time:.3}"
        );

        // 最小軸(x)・最大軸(z)まわり: 同じ大きさの擾乱を与えても反転しない。
        assert!(
            first_sign_flip(Vec3::new(spin, epsilon * spin, 0.0), |w| w.x).is_none(),
            "spin about the minimum-inertia axis must stay stable"
        );
        assert!(
            first_sign_flip(Vec3::new(epsilon * spin, 0.0, spin), |w| w.z).is_none(),
            "spin about the maximum-inertia axis must stay stable"
        );
    }

    /// **重力の「向き」と「大きさ」の分離**(単振り子)。
    /// `gravity_direction_can_be_changed_and_free_fall_follows_the_new_axis`は
    /// 軸に平行な自由落下しか見ていない。ここでは
    /// **軸に平行でない向き(鉛直から30°)**の重力下で、ワールド固定点に
    /// `DistanceJoint`で吊るした質点振り子を振らせ、微小振幅の周期が
    /// $T=2\pi\sqrt{L/g}$ ——**$g$ は大きさだけで、向きには一切依存しない**——に
    /// 一致することを確認する。これは重力を`GravityField`抽象へ置き換える将来の
    /// 変更が壊してはならない不変量そのもの。
    ///
    /// 許容誤差の根拠: 初期振幅 $\theta_0=5°$ の有限振幅補正が
    /// $T\simeq T_0(1+\theta_0^2/16)$ = +4.76e-4(相対)で、実測の偏差
    /// (dt=1/1000・10半周期)4.76e-4 とほぼ完全に一致する。つまり残差は
    /// **物理的に正しい非線形補正**であって数値誤差ではない。よって
    /// (1) 微小振幅の式 $T_0$ に対しては補正ぶんを飲み込む 1e-3、
    /// (2) 補正込みの式に対しては 5e-5(実測 5e-7)を要求する。
    #[test]
    fn pendulum_period_under_tilted_gravity_matches_two_pi_sqrt_l_over_g() {
        let g = 9.80665;
        let length = 1.0;
        let theta0 = 5.0_f64.to_radians();
        let dt = 1.0 / 1000.0;

        // 与えた重力方向で振り子を10半周期ぶん振らせ、平均半周期の2倍を返す。
        let measure_period = |gravity_direction: Vec3| -> f64 {
            let materials = MaterialDb::standard();
            let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
            let mut rng = SimRng::new(1, 1);
            let mut events = EventQueue::new();
            let mut solver = MechanicsSolver::new(g);
            solver.set_gravity_direction(gravity_direction);
            let down = solver.gravity_direction();
            // 揺動面は「重力方向」と「それに直交する perp」が張る平面。
            let perp = Vec3::new(0.0, 0.0, 1.0).cross(down).normalize_or_zero();
            assert!(perp.length() > 0.5, "swing plane is degenerate: {down:?}");

            let anchor = Vec3::ZERO;
            let start_dir = down.scale(theta0.cos()) + perp.scale(theta0.sin());
            // アンカーを重心(ボディ原点)に置くので、ジョイント力はトルクを生まず
            // 質点振り子になる(球の自転は運動から完全に切り離される)。
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.02 }, steel);
            desc.transform.position = anchor + start_dir.scale(length);
            let bob = solver.create_body(desc, &materials);
            solver.add_distance_joint(DistanceJoint {
                body_a: bob,
                anchor_a: Vec3::ZERO,
                body_b: None,
                anchor_b: anchor,
                length,
                disabled: false,
            });

            // 揺動座標 s = (位置−アンカー)·perp のゼロ交差(=平衡点通過)を数える。
            // 交差時刻は線形補間で求める(dt の量子化誤差を周期測定へ持ち込まない)。
            let mut previous = (solver.bodies.position[bob] - anchor).dot(perp);
            let mut crossings: Vec<f64> = Vec::new();
            let mut t = 0.0;
            let mut max_length_error: f64 = 0.0;
            let mut max_out_of_plane: f64 = 0.0;
            while crossings.len() < 11 {
                let mut ctx = make_ctx(&materials, &mut rng, &mut events);
                solver.step(dt, &mut ctx);
                let _: Vec<Event> = events.drain_sorted();
                t += dt;
                assert!(t < 60.0, "pendulum did not complete enough half periods");
                let relative = solver.bodies.position[bob] - anchor;
                max_length_error = max_length_error.max((relative.length() - length).abs());
                max_out_of_plane =
                    max_out_of_plane.max(relative.dot(down.cross(perp).normalize_or_zero()).abs());
                let s = relative.dot(perp);
                if previous > 0.0 && s <= 0.0 {
                    crossings.push(t - dt + dt * previous / (previous - s));
                }
                previous = s;
            }
            // 距離拘束が保たれていること・運動が揺動面内に留まること(円錐振り子に
            // なっていないこと)を確認してから周期を返す。
            assert!(
                max_length_error < 1e-5,
                "distance joint must hold the rod length: {max_length_error:.3e}"
            );
            assert!(
                max_out_of_plane < 1e-9,
                "motion must stay planar: {max_out_of_plane:.3e}"
            );
            let last = crossings.len() - 1;
            (crossings[last] - crossings[0]) / last as f64
        };

        // 鉛直から30°傾けた重力(x成分とy成分の両方を持つ)。
        let tilt = 30.0_f64.to_radians();
        let measured = measure_period(Vec3::new(tilt.sin(), -tilt.cos(), 0.0));

        let analytic = 2.0 * std::f64::consts::PI * (length / g).sqrt();
        assert!(
            (measured - analytic).abs() / analytic < 1e-3,
            "small-angle period must match 2π√(L/g) regardless of gravity direction: \
             measured={measured} analytic={analytic}"
        );
        // 残差の正体が有限振幅補正であることまで固定する。
        let with_amplitude_correction = analytic * (1.0 + theta0 * theta0 / 16.0);
        assert!(
            (measured - with_amplitude_correction).abs() / with_amplitude_correction < 5e-5,
            "the residual must be the analytic finite-amplitude correction T0(1+θ0²/16): \
             measured={measured} corrected={with_amplitude_correction}"
        );

        // 真下向き(既定)の重力でも同じ周期になる——周期は g の**大きさ**だけで
        // 決まり、向きには依存しないという不変量の直接確認。
        let straight_down = measure_period(Vec3::new(0.0, -1.0, 0.0));
        assert!(
            (measured - straight_down).abs() / straight_down < 1e-9,
            "the period must depend only on |g|, not on its direction: \
             tilted={measured} down={straight_down}"
        );
    }

    /// **重力場の抽象化増分**: `MechanicsSolver::new`が作る既定の場が
    /// 「大きさ`gravity`・向き下向きの一様場」であること、
    /// スカラー2つのアクセサ(`gravity`/`gravity_direction`)が移行前の
    /// 公開フィールドと同じ値を返すこと、そして`set_gravity_direction`が
    /// 大きさを保ったまま向きだけを(正規化して)変えること。
    #[test]
    fn default_field_is_uniform_and_the_scalar_accessors_mirror_the_old_fields() {
        let mut solver = MechanicsSolver::new(9.80665);
        assert_eq!(
            solver.gravity_field(),
            GravityField::Uniform {
                magnitude: 9.80665,
                direction: Vec3::new(0.0, -1.0, 0.0),
            }
        );
        assert_eq!(solver.gravity(), 9.80665);
        assert_eq!(solver.gravity_direction(), Vec3::new(0.0, -1.0, 0.0));

        // 正規化される(大きさは変わらない)。
        solver.set_gravity_direction(Vec3::new(3.0, 0.0, 0.0));
        assert_eq!(solver.gravity_direction(), Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(solver.gravity(), 9.80665);
        // ゼロベクトルは既定の下向きへ縮退する(移行前と同じ安全側の挙動)。
        solver.set_gravity_direction(Vec3::ZERO);
        assert_eq!(solver.gravity_direction(), Vec3::new(0.0, -1.0, 0.0));
        assert_eq!(solver.gravity(), 9.80665);
        // `set_gravity`は向きを保ったまま大きさだけを変える。
        solver.set_gravity_direction(Vec3::new(0.0, 0.0, 1.0));
        solver.set_gravity(1.62);
        assert_eq!(
            solver.gravity_field(),
            GravityField::Uniform {
                magnitude: 1.62,
                direction: Vec3::new(0.0, 0.0, 1.0),
            }
        );
    }

    /// **重力場の抽象化増分**: 非`Uniform`な場に対する`gravity`/`gravity_direction`の
    /// 縮約(両メソッドのdoc「一様場1つで近似したときの値」)を固定する。
    #[test]
    fn scalar_accessors_degrade_to_zero_magnitude_for_non_uniform_fields() {
        let mut solver = MechanicsSolver::new(9.80665);
        for field in [
            GravityField::Zero,
            GravityField::PointSource {
                center: Vec3::new(1.0, 2.0, 3.0),
                mu: 3.986e14,
            },
        ] {
            solver.set_gravity_field(field);
            assert_eq!(solver.gravity(), 0.0);
            assert_eq!(solver.gravity_direction(), Vec3::new(0.0, -1.0, 0.0));
            // 「一様場として見たベクトル」は常に厳密にゼロ(積が定義通り)。
            assert_eq!(
                solver.gravity_direction().scale(solver.gravity()),
                Vec3::ZERO
            );
        }
    }

    /// **重力場の抽象化増分**: `GravityField::acceleration_at`の3種の分岐を、
    /// 積分器を介さず直接確認する。
    #[test]
    fn acceleration_at_follows_each_field_kind() {
        // 一様場は位置に依存しない。
        let uniform = GravityField::Uniform {
            magnitude: 2.0,
            direction: Vec3::new(0.0, -1.0, 0.0),
        };
        assert_eq!(
            uniform.acceleration_at(Vec3::new(100.0, -50.0, 7.0)),
            Vec3::new(0.0, -2.0, 0.0)
        );
        assert_eq!(
            uniform.acceleration_at(Vec3::ZERO),
            Vec3::new(0.0, -2.0, 0.0)
        );

        assert_eq!(
            GravityField::Zero.acceleration_at(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::ZERO
        );

        // 点源: 中心を向き、大きさは mu/r^2。距離を2倍にすると1/4になる。
        let mu = 4.0e14;
        let point = GravityField::PointSource {
            center: Vec3::ZERO,
            mu,
        };
        let r = 1.0e7;
        let a_near = point.acceleration_at(Vec3::new(r, 0.0, 0.0));
        assert!((a_near.y).abs() < 1e-12 && (a_near.z).abs() < 1e-12);
        assert!((a_near.x + mu / (r * r)).abs() / (mu / (r * r)) < 1e-12);
        let a_far = point.acceleration_at(Vec3::new(0.0, 2.0 * r, 0.0));
        assert!((a_far.length() * 4.0 - a_near.length()).abs() / a_near.length() < 1e-12);
        assert!(a_far.y < 0.0, "acceleration must point toward the center");

        // 中心そのものは`Vec3::ZERO`へ縮退する(NaN/無限大を出さない)。
        let at_center = point.acceleration_at(Vec3::ZERO);
        assert_eq!(at_center, Vec3::ZERO);
        assert!(point
            .acceleration_at(Vec3::new(POINT_SOURCE_MIN_RADIUS * 0.5, 0.0, 0.0))
            .length()
            .is_finite());
    }

    /// **重力場の抽象化増分・解析解テスト**: `GravityField::PointSource`(逆二乗)の
    /// 下で、半径 $r$ の円軌道の解析解速度 $v=\sqrt{\mu/r}$ を初速に与えた自由体が、
    /// **多周回にわたってほぼ一定の半径を保つ**こと。逆二乗則が本当に $1/r^2$ で
    /// あることの直接の検証(大きさ一定の中心力や線形バネ場では、この初速で円軌道に
    /// ならない)。
    ///
    /// **許容誤差がゆるい理由**: `MechanicsSolver`は`sim_astro`の軌道専用積分器では
    /// なく、接触・拘束を含む汎用剛体ソルバであり、積分は semi-implicit Euler
    /// (1次)である。シンプレクティック性はあるので**永年的な半径のドリフトは
    /// 起きない**が、1次精度ゆえ軌道は $O(\Delta t)$ の離心率を持った楕円へずれる
    /// (このずれは周回ごとに増えるのではなく、一定の振幅で振動する)。したがって
    /// ここで固定するのは「半径が有界に留まる」ことであって「軌道が高精度である」
    /// ことではない——後者は`sim_astro`の専用積分器の受け持ちである。
    /// $r=10^7$m・$\mu=4\times10^{14}$(周期 $T=2\pi\sqrt{r^3/\mu}\simeq 3140$s)を
    /// $\Delta t=1$s(周期の約1/3140)で10周させ、相対偏差 2% を要求する
    /// (実測は 1% 未満)。
    #[test]
    fn circular_orbit_in_a_point_source_field_keeps_its_radius() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mu = 4.0e14;
        let radius = 1.0e7;
        let center = Vec3::ZERO;
        let mut solver = MechanicsSolver::new(0.0);
        solver.set_gravity_field(GravityField::PointSource { center, mu });

        // 初期条件: +x に距離 r、速度は +z 向き(軌道面は xz)に v=sqrt(mu/r)。
        let speed = (mu / radius).sqrt();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        desc.transform.position = center + Vec3::new(radius, 0.0, 0.0);
        desc.linear_velocity = Vec3::new(0.0, 0.0, speed);
        let sat = solver.create_body(desc, &materials);

        let period = 2.0 * std::f64::consts::PI * (radius * radius * radius / mu).sqrt();
        let dt = 1.0;
        let steps = (10.0 * period / dt) as u32;
        let mut min_radius = f64::INFINITY;
        let mut max_radius: f64 = 0.0;
        let mut max_out_of_plane: f64 = 0.0;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            let _: Vec<Event> = events.drain_sorted();
            let relative = solver.bodies.position[sat] - center;
            min_radius = min_radius.min(relative.length());
            max_radius = max_radius.max(relative.length());
            max_out_of_plane = max_out_of_plane.max(relative.y.abs());
        }

        assert!(
            (min_radius - radius).abs() / radius < 2.0e-2,
            "orbit radius must not collapse: min={min_radius:.6e} r={radius:.6e}"
        );
        assert!(
            (max_radius - radius).abs() / radius < 2.0e-2,
            "orbit radius must not grow: max={max_radius:.6e} r={radius:.6e}"
        );
        // 中心力なので角運動量の向き(=軌道面)は厳密に保存する。面外成分は
        // 丸め誤差だけであるべき(半径に対する相対で 1e-12)。
        assert!(
            max_out_of_plane / radius < 1e-12,
            "a central force must keep the orbit planar: {max_out_of_plane:.3e}"
        );
    }

    /// **重力場の抽象化増分・解析解テスト**: `GravityField::Zero`では、外力を
    /// 何も受けない自由体の速度と位置が**厳密に**慣性運動のまま
    /// (等速直線運動)であること。「重力を消す」経路が本当に何も加えていない
    /// ことの確認——`Uniform { magnitude: 0.0, .. }`との数値的な等価性も含めて
    /// 固定する。
    #[test]
    fn zero_field_leaves_a_free_body_in_exact_inertial_motion() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        solver.set_gravity_field(GravityField::Zero);
        let velocity = Vec3::new(1.0, 2.0, -3.0);
        let start = Vec3::new(0.0, 100.0, 0.0);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.transform.position = start;
        desc.linear_velocity = velocity;
        let idx = solver.create_body(desc, &materials);

        let dt = 1.0 / 120.0;
        let steps = 2000;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            let _: Vec<Event> = events.drain_sorted();
            // 速度は1stepも変化してはならない(重力が0なら加速度も厳密に0)。
            assert_eq!(
                solver.bodies.linear_velocity[idx], velocity,
                "GravityField::Zero must not change the velocity at all"
            );
        }
        // 位置は等速直線運動そのもの。`addcarry_scaled`の逐次加算と同じ順序で
        // 期待値を作れば厳密一致するが、ここは浮動小数の加算順序に依存しない
        // 相対許容(1e-12)で十分——検証したいのは「ドリフトしないこと」。
        let expected = start + velocity.scale(dt * steps as f64);
        let drift = (solver.bodies.position[idx] - expected).length();
        assert!(
            drift / expected.length() < 1e-12,
            "position must follow exact inertial motion: drift={drift:.3e}"
        );
        // ポテンシャルは常に0、力学的エネルギーは運動エネルギーのみ。
        let energy = solver.total_energy();
        assert_eq!(energy.potential, 0.0);
        assert!(energy.kinetic > 0.0);
    }
}
