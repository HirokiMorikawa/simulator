//! 剛体状態(SoA)。設計: docs/10-mechanics/01-rigid-body.md §3。

use crate::shape::Shape;
use sim_core::{FrameId, MaterialDb, MaterialId};
use sim_math::{Mat3, Quat, Transform, Vec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct ShapeHandle(pub u32);

/// 形状プール。`RigidBodySet` から間接参照する(設計 §3 の `Vec<ShapeHandle>`)。
#[derive(Default, Clone)]
pub struct ShapeStore {
    shapes: Vec<Shape>,
}

impl ShapeStore {
    pub fn new() -> ShapeStore {
        ShapeStore { shapes: Vec::new() }
    }

    pub fn insert(&mut self, shape: Shape) -> ShapeHandle {
        let handle = ShapeHandle(self.shapes.len() as u32);
        self.shapes.push(shape);
        handle
    }

    pub fn get(&self, handle: ShapeHandle) -> &Shape {
        &self.shapes[handle.0 as usize]
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BodyType {
    /// 全法則に従う。
    Dynamic,
    /// 不動(地面・壁)。inv_mass = 0。
    Static,
    /// スクリプト駆動(速度は外部指定、力を受けない)。エンティティ制御用。
    Kinematic,
}

/// 流体抗力モデル。設計: docs/10-mechanics/01-rigid-body.md §3、
/// docs/11-fluid/05-aero-hydrodynamics.md §3。力の計算(Schiller-Naumann 補正付き
/// 抗力式)は `MechanicsSolver::apply_forces` が `sim_fluid::drag_force_sphere` を
/// 呼んで行う(P1 スコープは Sphere のみ、Cd は Re から自動決定)。
/// `Box3`(姿勢依存の投影面積補間)・`Panels`(布・翼)は Phase 3–4。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DragModel {
    None,
    Sphere { radius: f64 },
    Box3 { half_extents: Vec3, cd: f64 },
}

/// 生成記述子。設計 §3。
pub struct RigidBodyDesc {
    pub body_type: BodyType,
    pub shape: Shape,
    pub material: MaterialId,
    pub transform: Transform,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub mass_override: Option<f64>,
    pub initial_temperature: f64,
    pub drag: DragModel,
    /// 衝突フィルタのビットフラグ(設計 docs/10-mechanics/02-collision-detection.md §4.1
    /// 「`collision_group: u32` / `collision_mask: u32` のビット AND」)。
    /// 既定は「全員が同じ 1 番グループに属し、全グループと当たる」。
    pub collision_group: u32,
    pub collision_mask: u32,
}

/// 衝突フィルタの既定値。**既定同士は必ず衝突する**(`1 & !0 != 0`)ので、
/// フィルタを設定しないシーンの挙動は導入前と完全に一致する。
pub const DEFAULT_COLLISION_GROUP: u32 = 1;
pub const DEFAULT_COLLISION_MASK: u32 = u32::MAX;

/// broadphase のペアフィルタ(設計 §4.1)。**双方向 AND** を取る——片側だけが
/// 相手を無視するのは物理的に意味を成さない(A は B を押すが B は A を押さない
/// という非対称な接触になり運動量が保存しない)ため、どちらか一方でも相手を
/// マスクしていればペアを捨てる。
pub fn collision_filter_allows(group_a: u32, mask_a: u32, group_b: u32, mask_b: u32) -> bool {
    (mask_a & group_b) != 0 && (mask_b & group_a) != 0
}

impl RigidBodyDesc {
    /// 既定は「原点に静止した動的球」。テスト・簡易シーン構築の出発点。
    pub fn dynamic(shape: Shape, material: MaterialId) -> RigidBodyDesc {
        RigidBodyDesc {
            body_type: BodyType::Dynamic,
            shape,
            material,
            transform: Transform {
                position: Vec3::ZERO,
                rotation: Quat::IDENTITY,
            },
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass_override: None,
            initial_temperature: 293.15,
            drag: DragModel::None,
            collision_group: DEFAULT_COLLISION_GROUP,
            collision_mask: DEFAULT_COLLISION_MASK,
        }
    }
}

/// 剛体状態の SoA コンテナ。設計 §3。
///
/// ## `position` は「重心」、形状のローカル原点とは別(群11で分離)
///
/// 移行前は「ローカル原点 = 重心 = 回転の中心」という単一点が3役を兼ねており、
/// 重心がローカル原点からずれる形状(部品を非対称配置した`Compound`など)は
/// 原理的に扱えなかった。群11でこれを分離し:
///
/// - `position[i]` は**重心のワールド座標**。運動方程式($v\mathrel{+}=F/m\,dt$、
///   $\omega\mathrel{+}=I^{-1}\tau\,dt$)も、接触・ジョイントの腕ベクトル $r$ も
///   すべてこの点を基準にする——つまり**ソルバ側のコードは一切変わらない**
///   (`joint::world_anchor`のdocが以前から「重心からのオフセット r」と書いて
///   いたとおりの意味論に、実体がようやく追いついた)。
/// - `center_of_mass[i]` は**形状のローカル系での重心位置**。形状のローカル原点を
///   ワールドへ復元するのに使う(`shape_transform`)。
///
/// `Sphere`/`Box`/`Capsule`/`Plane` はローカル原点まわりに対称なので
/// `center_of_mass[i] == Vec3::ZERO` であり、**これらしか使わない既存シーンの
/// 挙動は数値まで完全に不変**。
#[derive(Clone)]
pub struct RigidBodySet {
    // 状態(毎ステップ更新)
    /// **重心**のワールド座標(型doc「`position` は「重心」」参照)。
    pub position: Vec<Vec3>,
    pub frame: Vec<FrameId>,
    pub rotation: Vec<Quat>,
    pub linear_velocity: Vec<Vec3>,
    pub angular_velocity: Vec<Vec3>,
    // ステップ内アキュムレータ
    pub force_accum: Vec<Vec3>,
    pub torque_accum: Vec<Vec3>,
    // 定数(生成時に確定)
    pub inv_mass: Vec<f64>,
    /// **形状のローカル系での重心**(型doc参照)。ローカル原点まわりに対称な
    /// 形状では `Vec3::ZERO`。`shape_transform`/`origin_position` が
    /// `position[i]`(=重心)との相互変換に使う。
    pub center_of_mass: Vec<Vec3>,
    /// **重心まわり**のローカル慣性テンソルの逆行列
    /// (`Shape::unit_mass_inertia_tensor` の規約に合わせる)。
    pub inv_inertia_local: Vec<Mat3>,
    pub inv_inertia_world: Vec<Mat3>,
    pub body_type: Vec<BodyType>,
    pub shape: Vec<ShapeHandle>,
    pub material: Vec<MaterialId>,
    pub drag: Vec<DragModel>,
    /// 衝突フィルタ(設計 §4.1)。broadphase の候補ペア列挙で AND を取る。
    pub collision_group: Vec<u32>,
    pub collision_mask: Vec<u32>,
    // 熱結合用
    pub temperature: Vec<f64>,
    // スリープ用(設計 docs/10-mechanics/01-rigid-body.md §4)。
    /// 島全体の速度が閾値未満の状態が続いている秒数。
    pub still_time: Vec<f64>,
    /// 積分停止中か(島単位で揃う、`crate::sleep::update_sleep_state` が管理)。
    pub asleep: Vec<bool>,
    shapes: ShapeStore,
}

impl RigidBodySet {
    pub fn new() -> RigidBodySet {
        RigidBodySet {
            position: Vec::new(),
            frame: Vec::new(),
            rotation: Vec::new(),
            linear_velocity: Vec::new(),
            angular_velocity: Vec::new(),
            force_accum: Vec::new(),
            torque_accum: Vec::new(),
            inv_mass: Vec::new(),
            center_of_mass: Vec::new(),
            inv_inertia_local: Vec::new(),
            inv_inertia_world: Vec::new(),
            body_type: Vec::new(),
            shape: Vec::new(),
            material: Vec::new(),
            drag: Vec::new(),
            collision_group: Vec::new(),
            collision_mask: Vec::new(),
            temperature: Vec::new(),
            still_time: Vec::new(),
            asleep: Vec::new(),
            shapes: ShapeStore::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.position.len()
    }

    pub fn is_empty(&self) -> bool {
        self.position.is_empty()
    }

    pub fn shape_of(&self, index: usize) -> &Shape {
        self.shapes.get(self.shape[index])
    }

    /// **形状のローカル原点**のワールド座標。`position[i]`(=重心)から
    /// 重心オフセットを引き戻したもの(型doc「`position` は「重心」」参照)。
    ///
    /// 「このボディはどこにあるか」をユーザー・エディタ・シーン保存へ見せる
    /// ときはこちらを使う——`RigidBodyDesc::transform.position` で指定した
    /// 点と同じ意味になるため、往復(生成→読み出し)が恒等になる。
    pub fn origin_position(&self, index: usize) -> Vec3 {
        self.position[index] - self.rotation[index].rotate(self.center_of_mass[index])
    }

    /// **形状を配置するためのワールド変換**(ローカル原点 + 姿勢)。
    /// narrowphase・レイキャスト・オーバーラップ判定・描画など、
    /// 「形状の幾何」を扱う側はすべてこれを使う。
    ///
    /// 一方、運動方程式・接触の腕ベクトル・ジョイントの腕ベクトルは
    /// `position[i]`(=重心)を基準にする。**幾何は原点基準・力学は重心基準**、
    /// という役割分担がこの型の設計の要。
    pub fn shape_transform(&self, index: usize) -> Transform {
        Transform {
            position: self.origin_position(index),
            rotation: self.rotation[index],
        }
    }

    /// 形状のローカル原点が `origin` に来るように `position[i]`(=重心)を
    /// 動かす。テレポート・エディタのGizmo移動・シーン読み込みなど、
    /// 「ユーザーが指定した位置へ置く」経路のための setter
    /// (`origin_position` の逆)。
    pub fn set_origin_position(&mut self, index: usize, origin: Vec3) {
        self.position[index] = origin + self.rotation[index].rotate(self.center_of_mass[index]);
    }

    /// 質量(inv_mass の逆数、静的/キネマティックは 0)。
    pub fn mass(&self, index: usize) -> f64 {
        if self.inv_mass[index] > 0.0 {
            1.0 / self.inv_mass[index]
        } else {
            0.0
        }
    }

    /// 剛体を追加する。密度→質量は `mass_override` が無ければ `shape.volume() * material.density`
    /// (設計 §3 の `RigidBodyDesc` 規約)。返り値のインデックスは push 順(世代管理は
    /// World 層の責務で、`sim_world::World::remove_body`として**実装済み**——
    /// この注記は「Phase A では未実装」と書いたまま古くなっていた)。
    pub fn create_body(&mut self, desc: RigidBodyDesc, materials: &MaterialDb) -> usize {
        let index = self.position.len();
        let material = materials.get(desc.material);

        let mass = match desc.mass_override {
            Some(m) => m,
            None => desc.shape.volume().unwrap_or(0.0) * material.density,
        };
        let is_dynamic = matches!(desc.body_type, BodyType::Dynamic);
        let inv_mass = if is_dynamic && mass > 0.0 {
            1.0 / mass
        } else {
            0.0
        };

        let inv_inertia_local = if is_dynamic && mass > 0.0 {
            desc.shape
                .unit_mass_inertia_tensor()
                .scale(mass)
                .inverse()
                .unwrap_or(Mat3::from_diagonal(Vec3::ZERO))
        } else {
            Mat3::from_diagonal(Vec3::ZERO)
        };

        // `desc.transform.position` は**形状のローカル原点**の指定
        // (作者が「ここに置く」と書いた点)。追跡するのは重心なので、
        // 姿勢で回した重心オフセットぶんだけずらして保持する(型doc参照)。
        // 重心オフセットが 0 の形状(Sphere/Box/Capsule/Plane)では
        // `desc.transform.position` そのものになり、移行前と完全に一致する。
        let center_of_mass = desc.shape.center_of_mass();
        let shape_handle = self.shapes.insert(desc.shape);

        self.position
            .push(desc.transform.position + desc.transform.rotation.rotate(center_of_mass));
        self.frame.push(FrameId::ROOT);
        self.rotation.push(desc.transform.rotation);
        self.linear_velocity.push(desc.linear_velocity);
        self.angular_velocity.push(desc.angular_velocity);
        self.force_accum.push(Vec3::ZERO);
        self.torque_accum.push(Vec3::ZERO);
        self.inv_mass.push(inv_mass);
        self.center_of_mass.push(center_of_mass);
        self.inv_inertia_local.push(inv_inertia_local);
        self.inv_inertia_world
            .push(inv_inertia_local.similarity(desc.transform.rotation.to_mat3()));
        self.body_type.push(desc.body_type);
        self.shape.push(shape_handle);
        self.material.push(desc.material);
        self.drag.push(desc.drag);
        self.collision_group.push(desc.collision_group);
        self.collision_mask.push(desc.collision_mask);
        self.temperature.push(desc.initial_temperature);
        self.still_time.push(0.0);
        self.asleep.push(false);

        index
    }

    /// エディタのScale Gizmo(縮約実装、`sim-wasm::set_body_scale_at`参照)向けに、
    /// 既存ボディの形状を置き換え、質量・慣性を`create_body`と同じ式
    /// (質量 = `shape.volume() * material.density`、`inv_inertia_local`も同様)で
    /// 再計算する。`ShapeStore`は追加専用(既存の`create_body`と同じ設計)のため、
    /// 古い形状のエントリはプールに残り続ける(Undo/Redoが古いハンドルへ戻せる
    /// 実装ではないため実害は無いが、スケールドラッグを繰り返すたびにプールが
    /// 単調に増える点は正直に記録しておく)。
    ///
    /// 静止中(`sleep`モジュール参照)のボディの形状を変えると、寸法変更後の
    /// 形状が新たに接触相手と干渉していても、asleepなボディ同士の接触は
    /// 再解決されない(`MechanicsSolver::manifold_is_active`)ため、そのまま
    /// 干渉状態で固まってしまう。形状変更は静止仮定を無効化する明らかな
    /// イベントなので、`still_time`/`asleep`をリセットして次stepで確実に
    /// 再評価させる。
    ///
    /// **形状のローカル原点は動かさない**(群11)。形状を差し替えると重心
    /// オフセットが変わりうるので、追跡している`position[i]`(=重心)のほうを
    /// 付け替える——そうしないと「Scale Gizmo を引いたらボディが横に飛ぶ」
    /// ことになる。ユーザーが掴んでいるのは形状であって重心ではない。
    pub fn set_shape(&mut self, index: usize, shape: Shape, materials: &MaterialDb) {
        let material = materials.get(self.material[index]);
        let mass = shape.volume().unwrap_or(0.0) * material.density;
        let is_dynamic = matches!(self.body_type[index], BodyType::Dynamic);
        self.inv_mass[index] = if is_dynamic && mass > 0.0 {
            1.0 / mass
        } else {
            0.0
        };
        self.inv_inertia_local[index] = if is_dynamic && mass > 0.0 {
            shape
                .unit_mass_inertia_tensor()
                .scale(mass)
                .inverse()
                .unwrap_or(Mat3::from_diagonal(Vec3::ZERO))
        } else {
            Mat3::from_diagonal(Vec3::ZERO)
        };
        self.inv_inertia_world[index] =
            self.inv_inertia_local[index].similarity(self.rotation[index].to_mat3());
        // 旧形状の原点位置を保ったまま、新形状の重心オフセットへ張り替える。
        let origin = self.origin_position(index);
        self.center_of_mass[index] = shape.center_of_mass();
        self.set_origin_position(index, origin);
        self.shape[index] = self.shapes.insert(shape);
        self.still_time[index] = 0.0;
        self.asleep[index] = false;
    }

    /// 慣性テンソルを現在の質量・形状・姿勢から張り直す。`set_mass`/`set_body_type`
    /// の共通後段。`mass <= 0` または非 Dynamic なら inv 系はすべて 0
    /// (= 無限質量)——`create_body`/`set_shape` と同じ規約。
    fn rebuild_inertia(&mut self, index: usize, mass: f64) {
        let is_dynamic = matches!(self.body_type[index], BodyType::Dynamic);
        if is_dynamic && mass > 0.0 {
            self.inv_mass[index] = 1.0 / mass;
            self.inv_inertia_local[index] = self
                .shape_of(index)
                .unit_mass_inertia_tensor()
                .scale(mass)
                .inverse()
                .unwrap_or(Mat3::from_diagonal(Vec3::ZERO));
        } else {
            self.inv_mass[index] = 0.0;
            self.inv_inertia_local[index] = Mat3::from_diagonal(Vec3::ZERO);
        }
        self.inv_inertia_world[index] =
            self.inv_inertia_local[index].similarity(self.rotation[index].to_mat3());
        // 質量や種別が変わったら静止仮定は無効(`set_shape` と同じ理由)。
        self.still_time[index] = 0.0;
        self.asleep[index] = false;
    }

    /// 質量を直接指定する(Inspector の Mass フィールド、設計
    /// docs/23-frontend/01-editor.md §1.3 の RigidBody Component)。
    ///
    /// **形状は変えずに質量だけ変える**ので、密度は暗黙に `mass / volume` へ動く。
    /// これは `RigidBodyDesc::mass_override` と同じ意味論であり、材質の密度は
    /// `MaterialDb` 側の値のまま残る(材質を共有する他のボディを巻き込まない)。
    pub fn set_mass(&mut self, index: usize, mass: f64) {
        self.rebuild_inertia(index, mass);
    }

    /// Body type を切り替える(Dynamic ⇄ Static ⇄ Kinematic)。
    ///
    /// **Dynamic 以外へ移すと質量情報が失われる**(inv_mass = 0 は無限質量を
    /// 表すため元の値を復元できない)。呼び出し側が戻したい場合に備え、
    /// 切替直前の質量を返す。Static/Kinematic へ移す際は速度も 0 にする——
    /// inv_mass = 0 のまま速度が残ると、力を受けないのに等速で飛び続ける
    /// 物理的に意味のない状態になるため。
    pub fn set_body_type(&mut self, index: usize, body_type: BodyType, mass: f64) -> f64 {
        let previous_mass = self.mass(index);
        self.body_type[index] = body_type;
        if !matches!(body_type, BodyType::Dynamic) {
            self.linear_velocity[index] = Vec3::ZERO;
            self.angular_velocity[index] = Vec3::ZERO;
        }
        self.rebuild_inertia(index, mass);
        previous_mass
    }

    /// 衝突フィルタを設定する(設計 §4.1)。
    pub fn set_collision_filter(&mut self, index: usize, group: u32, mask: u32) {
        self.collision_group[index] = group;
        self.collision_mask[index] = mask;
    }
}

impl Default for RigidBodySet {
    fn default() -> Self {
        RigidBodySet::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;

    #[test]
    fn create_body_computes_mass_from_density_and_volume() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut set = RigidBodySet::new();
        let radius = 0.5;
        let idx = set.create_body(
            RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel),
            &materials,
        );
        let expected_volume = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
        let expected_mass = expected_volume * materials.get(steel).density;
        assert!((set.mass(idx) - expected_mass).abs() / expected_mass < 1e-12);
    }

    #[test]
    fn static_body_has_zero_inv_mass() {
        let materials = MaterialDb::standard();
        let concrete = materials.find_by_name("コンクリート").unwrap();
        let mut set = RigidBodySet::new();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        desc.body_type = BodyType::Static;
        let idx = set.create_body(desc, &materials);
        assert_eq!(set.inv_mass[idx], 0.0);
        assert_eq!(set.mass(idx), 0.0);
    }

    #[test]
    fn mass_override_takes_precedence_over_density() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut set = RigidBodySet::new();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        desc.mass_override = Some(42.0);
        let idx = set.create_body(desc, &materials);
        assert_eq!(set.mass(idx), 42.0);
    }

    /// `set_mass` は質量と慣性を整合させる。球の慣性は $I = \frac{2}{5} m r^2$
    /// なので、質量を2倍にすれば inv_inertia は厳密に半分になる(解析値と比較)。
    #[test]
    fn set_mass_rebuilds_inertia_analytically() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut set = RigidBodySet::new();
        let idx = set.create_body(
            RigidBodyDesc::dynamic(Shape::Sphere { radius: 2.0 }, steel),
            &materials,
        );
        set.set_mass(idx, 10.0);
        assert_eq!(set.mass(idx), 10.0);
        // I = 2/5 · 10 · 2² = 16 → inv = 1/16
        assert!((set.inv_inertia_local[idx].m[0][0] - 1.0 / 16.0).abs() < 1e-12);

        set.set_mass(idx, 20.0);
        assert!((set.inv_inertia_local[idx].m[0][0] - 1.0 / 32.0).abs() < 1e-12);
    }

    /// Body type の切替は inv_mass を 0 にし、残留速度も消す。Dynamic へ戻せば
    /// 指定した質量で復帰する(**元の質量は復元できない**ので呼び出し側が
    /// 保持する必要がある——`set_body_type` が旧質量を返す理由)。
    #[test]
    fn set_body_type_zeroes_inverse_mass_and_velocity() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut set = RigidBodySet::new();
        let idx = set.create_body(
            RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.5 }, steel),
            &materials,
        );
        set.linear_velocity[idx] = Vec3::new(1.0, 2.0, 3.0);
        let original = set.mass(idx);
        assert!(original > 0.0);

        let returned = set.set_body_type(idx, BodyType::Static, original);
        assert_eq!(returned, original);
        assert_eq!(set.inv_mass[idx], 0.0);
        assert_eq!(set.linear_velocity[idx], Vec3::ZERO);
        assert_eq!(set.inv_inertia_world[idx], Mat3::from_diagonal(Vec3::ZERO));

        set.set_body_type(idx, BodyType::Dynamic, original);
        assert!((set.mass(idx) - original).abs() < 1e-9);
    }

    /// 衝突フィルタは双方向 AND(設計 §4.1)。片側だけがマスクしても
    /// ペアは成立しない——非対称な接触は運動量を保存しないため。
    #[test]
    fn collision_filter_is_symmetric_and_defaults_to_allow() {
        // 既定同士は必ず通る(導入前の挙動と一致)。
        assert!(collision_filter_allows(
            DEFAULT_COLLISION_GROUP,
            DEFAULT_COLLISION_MASK,
            DEFAULT_COLLISION_GROUP,
            DEFAULT_COLLISION_MASK
        ));
        // グループ 0b01 と 0b10。A は B を見るが B は A を見ない → 落ちる。
        assert!(!collision_filter_allows(0b01, 0b11, 0b10, 0b10));
        // 逆向きも同じ結果(対称性)。
        assert!(!collision_filter_allows(0b10, 0b10, 0b01, 0b11));
        // 双方向に見えていれば通る。
        assert!(collision_filter_allows(0b01, 0b10, 0b10, 0b01));
    }
}
