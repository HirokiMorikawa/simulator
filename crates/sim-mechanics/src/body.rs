//! 剛体状態(SoA)。設計: docs/10-mechanics/01-rigid-body.md §3。

use crate::shape::Shape;
use sim_core::{FrameId, MaterialDb, MaterialId};
use sim_math::{Mat3, Quat, Transform, Vec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct ShapeHandle(pub u32);

/// 形状プール。`RigidBodySet` から間接参照する(設計 §3 の `Vec<ShapeHandle>`)。
#[derive(Default)]
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
        }
    }
}

/// 剛体状態の SoA コンテナ。設計 §3。
pub struct RigidBodySet {
    // 状態(毎ステップ更新)
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
    pub inv_inertia_local: Vec<Mat3>,
    pub inv_inertia_world: Vec<Mat3>,
    pub body_type: Vec<BodyType>,
    pub shape: Vec<ShapeHandle>,
    pub material: Vec<MaterialId>,
    pub drag: Vec<DragModel>,
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
            inv_inertia_local: Vec::new(),
            inv_inertia_world: Vec::new(),
            body_type: Vec::new(),
            shape: Vec::new(),
            material: Vec::new(),
            drag: Vec::new(),
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
    /// `remove_body` を持つ World 層の責務、Phase A では未実装)。
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
            let diag = desc.shape.unit_mass_inertia_diagonal().scale(mass);
            Mat3::from_diagonal(diag)
                .inverse()
                .unwrap_or(Mat3::from_diagonal(Vec3::ZERO))
        } else {
            Mat3::from_diagonal(Vec3::ZERO)
        };

        let shape_handle = self.shapes.insert(desc.shape);

        self.position.push(desc.transform.position);
        self.frame.push(FrameId::ROOT);
        self.rotation.push(desc.transform.rotation);
        self.linear_velocity.push(desc.linear_velocity);
        self.angular_velocity.push(desc.angular_velocity);
        self.force_accum.push(Vec3::ZERO);
        self.torque_accum.push(Vec3::ZERO);
        self.inv_mass.push(inv_mass);
        self.inv_inertia_local.push(inv_inertia_local);
        self.inv_inertia_world
            .push(inv_inertia_local.similarity(desc.transform.rotation.to_mat3()));
        self.body_type.push(desc.body_type);
        self.shape.push(shape_handle);
        self.material.push(desc.material);
        self.drag.push(desc.drag);
        self.temperature.push(desc.initial_temperature);
        self.still_time.push(0.0);
        self.asleep.push(false);

        index
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
}
