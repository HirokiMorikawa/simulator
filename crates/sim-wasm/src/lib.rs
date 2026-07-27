//! wasm-bindgen バインディング。設計: docs/00-foundation/05-rust-wasm-platform.md §3。
//!
//! Phase 0 は同文書のシグネチャ例(`WasmWorld::from_scene_json/step/time/
//! body_transforms_f32/observables_json/state_hash/push_command_json`)を
//! 「箱1個が落ちる」規模に縮小したものを公開する。シーンJSON・コマンドキュー・
//! 観測値JSONはシーン記述(docs/20-integration/04-world-api.md §3)が実装され次第、
//! Phase A 以降で追加する。
//!
//! **複数ボディ対応(ワークストリームD増分)**: エディタのHierarchy/Inspector
//! (docs/23-frontend/01-editor.md §1.1・§1.3)が複数ボディを列挙・選択できることを
//! 実際に検証するため、床の静的平面(`Shape::Plane`)を追加した(箱は床の上で静止
//! するようになる、以前の「永遠に落下し続ける」挙動からの意図的な変更)。
//! `sim_world::World`自体は汎用的な「全ボディ列挙」APIを持たないため(`BodyId`は
//! 世代付きindexで、削除済みスロットとの区別に`World`内部の世代情報が必要)、
//! `WasmWorld`が自ら構築した固定2体(床・箱)+スポーンパレット(§6)で動的に
//! 追加されたボディ(`SpawnedBodyMeta`、`spawn_sphere`/`spawn_box`)をindexで
//! 列挙する縮約実装とした(シーンJSON経由で任意個のボディを構築できるように
//! なれば、`from_scenario`のボディリストをそのまま列挙する形に置き換える)。

use std::collections::VecDeque;

use js_sys::{Float32Array, Float64Array};
use sim_core::FrameId;
use sim_math::{Quat, Vec3};
use sim_mechanics::{BallJoint, BodyType, HingeMotorPd, RigidBodyDesc, Shape};
use sim_world::{BodyId, Command, ProbeTarget, World, WorldOptions};
use wasm_bindgen::prelude::*;

/// `docs/23-frontend/01-editor.md`のProbe Graphsパネル(§1.4「複数系列」)デモ用に、
/// 箱のy座標を毎step記録するプローブの履歴長。1step=dt秒、`PROBE_HISTORY_CAPACITY`
/// step分(≈`PROBE_HISTORY_CAPACITY*dt`秒)のスクロールウィンドウになる。
const PROBE_HISTORY_CAPACITY: usize = 600;

/// Timelineパネルのスナップショットリングバッファ(設計docs/00-foundation/
/// 04-architecture.md §「巻き戻しのスナップショット予算」: 既定1s間隔・
/// リングバッファN=8面・直近8s分)。1s間隔は`dt`から算出する
/// (`WasmWorld::new`で`1.0/dt`を四捨五入)。
const SNAPSHOT_RING_CAPACITY: usize = 8;

/// 分圧回路(`Command::SetSwitch`実証用、`WasmWorld::new`参照)の分圧点ノード番号。
const CIRCUIT_DIVIDER_NODE: usize = 2;

/// 熱ノード(`Command::SetHeatSource`実証用、`WasmWorld::new`参照)のindex
/// (単一ノードのみのため常に0)。
const THERMAL_HEATER_NODE: usize = 0;

/// スポーンパレット(設計docs/23-frontend/01-editor.md §6「形状×材質を選んで
/// クリック配置」)で追加したボディの記録。Shapeは`World::mechanics().bodies.
/// shape_of`で実クエリできるが(`body_shape_label_at`参照)、Materialは`World`が
/// ボディからMaterialIdを引く公開APIを持たないため、スポーンした側(この構造体)
/// が構築時の材質名をそのまま覚えておく縮約実装のまま(固定2体(床・箱)の
/// ハードコードと同じ発想)。
struct SpawnedBodyMeta {
    id: BodyId,
    label: String,
    material_label: String,
    /// Scale Gizmo(`set_body_scale_at`参照)がスケール係数を掛ける基準形状
    /// (スポーン時の寸法、以後`World`側の実形状が変わってもこの基準は不変)。
    base_shape: Shape,
    /// 振り子スポーン(`spawn_pendulum`)が追加したDistanceJointの
    /// `World::distance_joint_anchor_points`用index。球/箱スポーンでは`None`
    /// (拘束オーバーレイ対象外)。
    constraint_joint_index: Option<usize>,
    /// モーターアームスポーン(`spawn_motor_arm`)が追加した`HingeMotorPd`の
    /// `MechanicsSolver::hinge_motors`内index(`Command::SetMotorTarget`の
    /// `hinge_motor_index`引数、`set_motor_target_at`参照)。振り子/球/箱
    /// スポーンでは`None`。
    hinge_motor_index: Option<usize>,
}

#[wasm_bindgen]
pub struct WasmWorld {
    inner: World,
    ground_body: BodyId,
    box_body: BodyId,
    /// スポーンパレットで追加されたボディ(固定2体の後にindex 2, 3, ...として続く)。
    spawned: Vec<SpawnedBodyMeta>,
    y_probe: usize,
    speed_probe: usize,
    /// 分圧回路のスイッチ(`sim_em::Circuit::add_switch`が返すindex、
    /// `set_circuit_switch_closed`参照)。
    circuit_switch_index: usize,
    snapshot_interval_steps: u64,
    snapshots: VecDeque<World>,
    bookmarks: Vec<(String, World)>,
}

#[wasm_bindgen]
impl WasmWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity: f64, dt: f64, initial_height: f64) -> WasmWorld {
        let options = WorldOptions {
            gravity,
            dt,
            seed: 0,
        };
        let mut inner = World::new(options);
        let concrete = inner
            .materials()
            .find_by_name("コンクリート")
            .expect("standard DB has concrete");
        let mut ground_desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: sim_math::Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        ground_desc.body_type = BodyType::Static;
        let ground_body = inner.create_body(ground_desc);

        let steel = inner
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: sim_math::Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        desc.transform.position = sim_math::Vec3::new(0.0, initial_height, 0.0);
        let box_body = inner.create_body(desc);
        let y_probe = inner.add_probe(ProbeTarget::BodyPosY(box_body), PROBE_HISTORY_CAPACITY);
        let speed_probe = inner.add_probe(ProbeTarget::BodySpeed(box_body), PROBE_HISTORY_CAPACITY);

        // 分圧回路(`Command::SetSwitch`の実証用、`sim-world`の
        // `set_switch_command_closes_switch_and_changes_circuit_state`と同じ
        // 構成)。node 0=GND、1=電源(10V)、2=分圧点。スイッチは負荷抵抗(200Ω)と
        // 並列に接続し、閉じると分圧点をGNDへ短絡する(開: 6.67V、閉: 0V)。
        let mut circuit = sim_em::Circuit::new(3);
        circuit.add_voltage_source(1, sim_em::GROUND, 10.0);
        circuit.add_resistor(1, CIRCUIT_DIVIDER_NODE, 100.0);
        let circuit_switch_index = circuit.add_switch(CIRCUIT_DIVIDER_NODE, sim_em::GROUND, false);
        circuit.add_resistor(CIRCUIT_DIVIDER_NODE, sim_em::GROUND, 200.0);
        inner.enable_circuit(circuit);

        // 熱ノード(`Command::SetHeatSource`の実証用、`sim-world`の
        // `set_heat_source_command_raises_temperature_for_one_step_only`と
        // 同じ「1step分だけ効く」縮約セマンティクス)。ニュートン冷却
        // (`sim-thermal`のT1テストと同じc=100J/K・h=10W/(m^2K)・area=1m^2、
        // 時定数τ=c/(hA)=10s)ありの単一ノード、初期温度=周囲温度。
        let ambient_temperature = 293.15;
        let mut thermal = sim_thermal::ThermalSolver::new(ambient_temperature);
        let mut heater_node = sim_thermal::ThermalNode::new(ambient_temperature, 100.0);
        heater_node.convection_coefficient = 10.0;
        heater_node.area = 1.0;
        thermal.add_node(heater_node);
        inner.enable_thermal(thermal);

        let snapshot_interval_steps = (1.0 / dt).round().max(1.0) as u64;
        WasmWorld {
            inner,
            ground_body,
            box_body,
            spawned: Vec::new(),
            y_probe,
            speed_probe,
            circuit_switch_index,
            snapshot_interval_steps,
            snapshots: VecDeque::with_capacity(SNAPSHOT_RING_CAPACITY),
            bookmarks: Vec::new(),
        }
    }

    /// Hierarchyパネルが列挙するボディ数(固定2体+スポーンパレットで追加した分、
    /// モジュールdoc「複数ボディ対応」参照)。
    pub fn body_count(&self) -> usize {
        2 + self.spawned.len()
    }

    fn body_id_at(&self, index: usize) -> BodyId {
        match index {
            0 => self.ground_body,
            1 => self.box_body,
            _ => {
                self.spawned
                    .get(index - 2)
                    .unwrap_or_else(|| {
                        panic!(
                            "body index {index} out of range (body_count={})",
                            self.body_count()
                        )
                    })
                    .id
            }
        }
    }

    /// Hierarchyパネル表示用のラベル。
    pub fn body_label_at(&self, index: usize) -> String {
        match index {
            0 => "Ground".to_string(),
            1 => "Box_1".to_string(),
            _ => self
                .spawned
                .get(index - 2)
                .unwrap_or_else(|| panic!("body index {index} out of range"))
                .label
                .clone(),
        }
    }

    /// `index`番目のボディが静的(Static)かどうか。InspectorがTransformの速度欄を
    /// 意味のある形で表示するための補助(静的ボディは速度が常に0で自明なため)。
    /// `World::mechanics().bodies.body_type`を実クエリする(以前は「index==0
    /// (固定の床)のみ静的」という決め打ちだったが、シーンJSON Import
    /// (`import_scene_json`)で任意のindexに静的ボディが追加され得るようになった
    /// ため、実際の`BodyType`を見るクエリに置き換えた——`body_shape_label_at`が
    /// 既に辿った同じ理由)。
    pub fn body_is_static_at(&self, index: usize) -> bool {
        let id = self.body_id_at(index);
        matches!(
            self.inner.mechanics().bodies.body_type[id.index as usize],
            BodyType::Static
        )
    }

    /// Inspector表示用のShape文字列。`World::mechanics().bodies.shape_of`で
    /// 実際の現在の形状(Scale Gizmoで変更済みなら変更後の寸法)を読み、
    /// フォーマットする(以前は`SpawnedBodyMeta`にスポーン時の文字列を固定で
    /// 覚えておく縮約実装だったが、Scale Gizmoで寸法が変わりうるようになった
    /// ため、常に最新の値を返す実クエリに置き換えた)。
    pub fn body_shape_label_at(&self, index: usize) -> String {
        let id = self.body_id_at(index);
        match self.inner.mechanics().bodies.shape_of(id.index as usize) {
            Shape::Sphere { radius } => format!("Sphere({radius:.4})"),
            Shape::Box { half_extents } => format!(
                "Box({:.4},{:.4},{:.4})",
                half_extents.x, half_extents.y, half_extents.z
            ),
            Shape::Plane { normal, d } => {
                format!(
                    "Plane(normal=({},{},{}), d={d})",
                    normal.x, normal.y, normal.z
                )
            }
            other => format!("{other:?}"),
        }
    }

    /// Inspector表示用の材質名。
    pub fn body_material_label_at(&self, index: usize) -> String {
        match index {
            0 => "コンクリート".to_string(),
            1 => "鋼(炭素鋼)".to_string(),
            _ => self
                .spawned
                .get(index - 2)
                .unwrap_or_else(|| panic!("body index {index} out of range"))
                .material_label
                .clone(),
        }
    }

    /// Projectドロワー Materials タブ(設計docs/23-frontend/01-editor.md §1.6
    /// 「Materials: MaterialDbプリセット一覧」)向けに、指定した材質名の主要物性値を
    /// `[density, friction, restitution, specific_heat, conductivity]`の順で返す。
    /// 未知の名前ならパニックする(呼び出し側UIが`SPAWN_MATERIALS`等の既知の名前だけを
    /// 渡す前提、`spawn_sphere`と同じ設計)。
    pub fn material_properties_f64(&self, name: String) -> Float64Array {
        let id = self
            .inner
            .materials()
            .find_by_name(&name)
            .unwrap_or_else(|| panic!("unknown material: {name}"));
        let m = self.inner.materials().get(id);
        Float64Array::from(
            &[
                m.density,
                m.friction,
                m.restitution,
                m.specific_heat,
                m.conductivity,
            ][..],
        )
    }

    /// スポーンパレット(設計docs/23-frontend/01-editor.md §6)——球を`material_name`
    /// (`MaterialDb::standard`が持つ名前)で`(x,y,z)`に配置する。新しいボディの
    /// index(`body_count`と同じ体系)を返す。未知の材質名ならパニックする
    /// (呼び出し側UIが既知の名前だけを選択肢にするため、実行時に到達しない前提)。
    pub fn spawn_sphere(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        radius: f64,
        material_name: String,
    ) -> usize {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .unwrap_or_else(|| panic!("unknown material: {material_name}"));
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, material);
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count();
        let label = format!("Sphere_{index}");
        self.spawned.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Sphere { radius },
            constraint_joint_index: None,
            hinge_motor_index: None,
        });
        index
    }

    /// スポーンパレット——箱(半辺長`half_extent`の立方体)を`material_name`で
    /// `(x,y,z)`に配置する。`spawn_sphere`と同じ規約。
    pub fn spawn_box(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        half_extent: f64,
        material_name: String,
    ) -> usize {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .unwrap_or_else(|| panic!("unknown material: {material_name}"));
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: sim_math::Vec3::new(half_extent, half_extent, half_extent),
            },
            material,
        );
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count();
        let label = format!("Box_{index}");
        self.spawned.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Box {
                half_extents: sim_math::Vec3::new(half_extent, half_extent, half_extent),
            },
            constraint_joint_index: None,
            hinge_motor_index: None,
        });
        index
    }

    /// シーンJSON Import(設計docs/23-frontend/01-editor.md §1.6「Scenes: シーン
    /// JSON…Export/Import」——Exportは既に実装済み、これがImport側)。`json`を
    /// `sim_world::Scenario`としてパースし、`World::append_scenario_bodies`
    /// (`fluids`/`probes`セクションは対象外、そのdoc参照)で現在のワールドへ
    /// ボディを追加する。D1–D43のシーンJSONファイル(ヘッドレスランナーが使うのと
    /// 同じスキーマ)をそのままエディタへ読み込んで視覚的に確認できるようにする
    /// のが狙い(設計のワークストリームD項目13)。追加した各ボディを`spawn_sphere`/
    /// `spawn_box`と同じ`SpawnedBodyMeta`として登録するため、Hierarchy/Inspector/
    /// Scene Viewから見てスポーンパレットで追加したボディと区別が付かない。
    /// 返り値は追加したボディ数(呼び出し側はこの数だけ`body_count()`の末尾から
    /// メッシュを生成すればよい)。パース/検証エラーは`JsValue`(メッセージ文字列)
    /// として返す。
    pub fn import_scene_json(&mut self, json: String) -> Result<usize, JsValue> {
        let scenario = sim_world::Scenario::from_json(&json)
            .map_err(|e| JsValue::from_str(&format!("{e:?}")))?;
        let ids = self
            .inner
            .append_scenario_bodies(&scenario)
            .map_err(|e| JsValue::from_str(&format!("{e:?}")))?;

        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            let index = self.body_count();
            let label = body
                .name
                .clone()
                .unwrap_or_else(|| format!("Imported_{index}"));
            let shape = match body.shape {
                sim_world::ShapeJson::Box { half } => Shape::Box {
                    half_extents: Vec3::new(half[0], half[1], half[2]),
                },
                sim_world::ShapeJson::Sphere { radius } => Shape::Sphere { radius },
                sim_world::ShapeJson::Plane { normal, d } => Shape::Plane {
                    normal: Vec3::new(normal[0], normal[1], normal[2]),
                    d,
                },
            };
            self.spawned.push(SpawnedBodyMeta {
                id: *id,
                label,
                material_label: body.material.clone(),
                base_shape: shape,
                constraint_joint_index: None,
                hinge_motor_index: None,
            });
        }

        Ok(scenario.bodies.len())
    }

    /// スポーンパレット——振り子(拘束オーバーレイの実証用)。ワールド固定点
    /// `(pivot_x, pivot_y, pivot_z)`から`DistanceJoint`(`World::
    /// add_distance_joint_to_world_point`)で距離`arm_length`に保たれる球を
    /// 配置する。鉛直から30度傾いた位置(`pivot`から`arm_length`だけ離れた
    /// 点)を初期位置とすることで、静止した自明な平衡状態ではなく実際に
    /// 重力で振り子運動が始まる。
    pub fn spawn_pendulum(
        &mut self,
        pivot_x: f64,
        pivot_y: f64,
        pivot_z: f64,
        arm_length: f64,
        material_name: String,
    ) -> usize {
        const BOB_RADIUS: f64 = 0.3;
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .unwrap_or_else(|| panic!("unknown material: {material_name}"));
        let pivot = sim_math::Vec3::new(pivot_x, pivot_y, pivot_z);
        let initial_angle_from_vertical = std::f64::consts::PI / 6.0; // 30度
        let initial_offset = sim_math::Vec3::new(
            arm_length * initial_angle_from_vertical.sin(),
            -arm_length * initial_angle_from_vertical.cos(),
            0.0,
        );
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: BOB_RADIUS }, material);
        desc.transform.position = pivot + initial_offset;
        let id = self.inner.create_body(desc);
        let joint_index = self.inner.add_distance_joint_to_world_point(
            id,
            sim_math::Vec3::ZERO,
            pivot,
            arm_length,
        );
        let index = self.body_count();
        let label = format!("Pendulum_{index}");
        self.spawned.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Sphere { radius: BOB_RADIUS },
            constraint_joint_index: Some(joint_index),
            hinge_motor_index: None,
        });
        index
    }

    /// スポーンパレット——モーターアーム(`Command::SetMotorTarget`の実証用)。
    /// ワールド固定点`(pivot_x, pivot_y, pivot_z)`へ`BallJoint`でピン留めした
    /// 棒状の箱を、Z軸まわりの`HingeMotorPd`(PD位置サーボ)で角度制御する。
    /// 初期状態は目標角0(鉛直にぶら下がる姿勢)。
    pub fn spawn_motor_arm(
        &mut self,
        pivot_x: f64,
        pivot_y: f64,
        pivot_z: f64,
        material_name: String,
    ) -> usize {
        const HALF_EXTENTS: sim_math::Vec3 = sim_math::Vec3 {
            x: 0.1,
            y: 0.6,
            z: 0.1,
        };
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .unwrap_or_else(|| panic!("unknown material: {material_name}"));
        let pivot = sim_math::Vec3::new(pivot_x, pivot_y, pivot_z);
        let anchor_local_top = sim_math::Vec3::new(0.0, HALF_EXTENTS.y, 0.0);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: HALF_EXTENTS,
            },
            material,
        );
        // 目標角0(鉛直)の姿勢: body中心はpivotから真下にHALF_EXTENTS.yだけ離れた点。
        desc.transform.position = pivot - anchor_local_top;
        desc.mass_override = Some(5.0); // sim-mechanicsの参照テストと同じ質量(kp/kd/torque_maxの既定値がこの慣性で検証済み)。
        let id = self.inner.create_body(desc);
        self.inner.mechanics_mut().add_ball_joint(BallJoint {
            body_a: id.index as usize,
            anchor_a: anchor_local_top,
            body_b: None,
            anchor_b: pivot,
            disabled: false,
        });
        let hinge_motor_index = self.inner.mechanics().hinge_motors.len();
        self.inner.mechanics_mut().add_hinge_motor(HingeMotorPd {
            body: id.index as usize,
            axis: sim_math::Vec3::new(0.0, 0.0, 1.0),
            reference_rotation: sim_math::Quat::IDENTITY,
            theta_target: 0.0,
            kp: 20.0,
            kd: 2.0,
            torque_max: 50.0,
        });
        let index = self.body_count();
        let label = format!("MotorArm_{index}");
        self.spawned.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Box {
                half_extents: HALF_EXTENTS,
            },
            constraint_joint_index: None,
            hinge_motor_index: Some(hinge_motor_index),
        });
        index
    }

    /// `Command::SetMotorTarget`(モジュールdoc参照、`theta_target`は
    /// ラジアン)を、`index`番目のボディが持つヒンジモーターへ送る。
    /// モーターを持たないボディに呼ぶとパニックする(呼び出し側UIが
    /// モーターを持つボディだけに対して呼ぶ前提)。
    pub fn set_motor_target_at(&mut self, index: usize, theta_target: f64) {
        let hinge_motor_index = match index {
            0 | 1 => None,
            _ => {
                self.spawned
                    .get(index - 2)
                    .unwrap_or_else(|| panic!("body index {index} out of range"))
                    .hinge_motor_index
            }
        };
        let hinge_motor_index =
            hinge_motor_index.unwrap_or_else(|| panic!("body index {index} has no hinge motor"));
        self.inner.push_command(Command::SetMotorTarget {
            hinge_motor_index,
            theta_target,
        });
    }

    /// 分圧回路(`WasmWorld::new`参照)の分圧点電圧[V]。`Command::SetSwitch`の
    /// 効果をUIから確認するための読み取り専用クエリ。
    pub fn circuit_divider_voltage(&self) -> f64 {
        self.inner
            .circuit_probe(CIRCUIT_DIVIDER_NODE)
            .unwrap_or(0.0)
    }

    /// `Command::SetSwitch`——分圧回路のスイッチの開閉を変更する。閉じると
    /// 分圧点がGNDへ短絡され`circuit_divider_voltage`がほぼ0になる。
    pub fn set_circuit_switch_closed(&mut self, closed: bool) {
        self.inner.push_command(Command::SetSwitch {
            switch_index: self.circuit_switch_index,
            closed,
        });
    }

    /// 熱ノード(`WasmWorld::new`参照)の現在温度[K]。
    pub fn heater_node_temperature(&self) -> f64 {
        self.inner.thermal().unwrap().nodes[THERMAL_HEATER_NODE].temperature
    }

    /// `Command::SetHeatSource`——熱ノードへ`watts`ワットの熱源を1step分だけ
    /// 与える(モジュールdoc「1step分だけ効く」縮約セマンティクス参照)。
    /// 継続加熱するには呼び出し側が毎stepの直前に再度呼ぶ必要がある
    /// (`main.ts`の`frame()`ループ参照)。
    pub fn push_heat_source(&mut self, watts: f64) {
        self.inner.push_command(Command::SetHeatSource {
            node: THERMAL_HEATER_NODE,
            watts,
        });
    }

    /// Scene Viewの拘束オーバーレイ(設計docs/23-frontend/01-editor.md §1.2
    /// 「拘束」)向けに、`index`番目のボディが持つ拘束(DistanceJoint)の
    /// アンカー点2点を`[ax,ay,az,bx,by,bz]`(f32)で返す。拘束を持たない
    /// ボディ(床・箱・スポーンした球/箱)なら空配列を返す。
    pub fn constraint_anchor_points_at(&self, index: usize) -> Float32Array {
        let joint_index = match index {
            0 | 1 => None,
            _ => {
                self.spawned
                    .get(index - 2)
                    .unwrap_or_else(|| panic!("body index {index} out of range"))
                    .constraint_joint_index
            }
        };
        let Some(joint_index) = joint_index else {
            return Float32Array::new_with_length(0);
        };
        let (a, b) = self
            .inner
            .distance_joint_anchor_points(joint_index)
            .expect("constraint_joint_index recorded at spawn time must stay valid");
        Float32Array::from(
            &[
                a.x as f32, a.y as f32, a.z as f32, b.x as f32, b.y as f32, b.z as f32,
            ][..],
        )
    }

    /// フレーム軸オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「フレーム
    /// サブモード」の土台)向けに、ROOTの子として指定角速度(z軸まわり)で自転する
    /// フレームを追加する(`World::add_frame`+`sim_core::FrameTree::step`が毎step
    /// 自動的に回転を進める)。返り値はこのフレームの`FrameId`(`frame_rotation_
    /// at_f32`に渡すindex)。
    pub fn add_rotating_frame(&mut self, angular_velocity_z: f64) -> usize {
        let id = self.inner.add_frame(
            FrameId::ROOT,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, angular_velocity_z),
        );
        id.0 as usize
    }

    /// `frame_index`番目のフレームの現在の姿勢(親フレームからの相対回転)を
    /// クォータニオン`[x, y, z, w]`(f32)で返す。
    pub fn frame_rotation_at_f32(&self, frame_index: usize) -> Float32Array {
        let rotation = self
            .inner
            .frames()
            .frame(FrameId(frame_index as u32))
            .rotation_in_parent;
        Float32Array::from(
            &[
                rotation.x as f32,
                rotation.y as f32,
                rotation.z as f32,
                rotation.w as f32,
            ][..],
        )
    }

    /// 全フレーム数(ROOT含む、`sim_core::FrameTree::frame_count`の素通し)。
    /// フレーム階層ドリルインUI(Hierarchyの「Frames」サブツリー)がフレーム
    /// 一覧を列挙するために使う。
    pub fn frame_count(&self) -> usize {
        self.inner.frames().frame_count()
    }

    /// `frame_index`番目のフレームの親のindex。ROOT自身(index 0)は親を
    /// 持たないため`-1`を返す(フレーム階層ドリルインUIがツリー構造を
    /// 組み立てるための情報)。
    pub fn frame_parent_index(&self, frame_index: usize) -> i32 {
        match self
            .inner
            .frames()
            .frame(FrameId(frame_index as u32))
            .parent
        {
            Some(parent) => parent.0 as i32,
            None => -1,
        }
    }

    /// `frame_index`番目のフレームのROOT(ワールド)座標系での位置
    /// (`sim_core::FrameTree::transform_to_root`)。`frame_rotation_at_f32`
    /// (親フレームからの相対回転のみ、単一のROOT直下フレームを想定していた
    /// 旧API)と異なり、複数フレームが親子関係を持つ場合(フレーム階層
    /// ドリルインUI)でも階層を遡って合成した実際のワールド位置を返す。
    pub fn frame_world_position_f32(&self, frame_index: usize) -> Float32Array {
        let position = self
            .inner
            .frames()
            .transform_to_root(FrameId(frame_index as u32))
            .position;
        Float32Array::from(&[position.x as f32, position.y as f32, position.z as f32][..])
    }

    /// `frame_index`番目のフレームのROOT(ワールド)座標系での姿勢
    /// (`frame_world_position_f32`と同じ理由で`transform_to_root`を使う)。
    pub fn frame_world_rotation_f32(&self, frame_index: usize) -> Float32Array {
        let rotation = self
            .inner
            .frames()
            .transform_to_root(FrameId(frame_index as u32))
            .rotation;
        Float32Array::from(
            &[
                rotation.x as f32,
                rotation.y as f32,
                rotation.z as f32,
                rotation.w as f32,
            ][..],
        )
    }

    /// フレーム階層ドリルインUI(設計docs/23-frontend/01-editor.md §1.3
    /// 「フレームサブモード」)向けに、任意の既存フレーム(`parent_index`、
    /// 0=ROOT)の子として新規フレームを追加する(`add_rotating_frame`の
    /// 一般化——親をROOT固定ではなく任意に選べる)。`origin_offset_*`は
    /// 親フレーム内での原点位置(Scene View上でネストしたフレームが重ならない
    /// よう、呼び出し側が親からのオフセットを指定する)。返り値は新規フレームの
    /// index(`frame_world_position_f32`/`frame_world_rotation_f32`に渡す)。
    pub fn add_child_frame(
        &mut self,
        parent_index: usize,
        origin_offset_x: f64,
        origin_offset_y: f64,
        origin_offset_z: f64,
        angular_velocity_z: f64,
    ) -> usize {
        let id = self.inner.add_frame(
            FrameId(parent_index as u32),
            Vec3::new(origin_offset_x, origin_offset_y, origin_offset_z),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, angular_velocity_z),
        );
        id.0 as usize
    }

    /// 流体場オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「流体場」の土台)
    /// 向けに、`sim_fluid::SphFluid`を有効化し、小さな水塊(3×3×3粒子)+その直下の
    /// 床(1層の境界粒子、`SphFluid::add_boundary_particle`)を追加する。二度目以降の
    /// 呼び出しは`World::enable_sph`が新しい`SphFluid`で置き換えるため実質的に
    /// リセットとして機能する。
    pub fn spawn_fluid_block(&mut self) {
        let h: f64 = 0.15;
        let rho0: f64 = 1000.0;
        let c_s: f64 = 20.0;
        let dx: f64 = 0.1;
        let mut sph = sim_fluid::SphFluid::new(h, rho0, c_s);
        sph.mass = rho0 * dx.powi(3);

        let origin = Vec3::new(3.0, 2.0, 0.0);
        let n = 3;
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    let pos = origin + Vec3::new(ix as f64 * dx, iy as f64 * dx, iz as f64 * dx);
                    sph.add_particle(pos, Vec3::ZERO);
                }
            }
        }

        let floor_half = 0.6;
        let floor_y = -dx;
        let mut fx = origin.x - floor_half;
        while fx <= origin.x + floor_half {
            let mut fz = origin.z - floor_half;
            while fz <= origin.z + floor_half {
                sph.add_boundary_particle(Vec3::new(fx, floor_y, fz));
                fz += dx;
            }
            fx += dx;
        }

        self.inner.enable_sph(sph);
    }

    /// 流体粒子数(境界粒子は含まない、`fluid_particle_positions_f32`と同じ体系)。
    /// 流体ドメインが有効でなければ0。
    pub fn fluid_particle_count(&self) -> usize {
        self.inner.sph().map_or(0, |s| s.position.len())
    }

    /// 全流体粒子の位置をフラットな`[x0,y0,z0,x1,y1,z1,...]`(f32)で返す
    /// (毎フレーム粒子数分`body_position_at_f32`相当を個別呼び出しするのは
    /// wasm境界越えのオーバーヘッドが大きいため、1回のクエリにまとめた)。
    pub fn fluid_particle_positions_f32(&self) -> Float32Array {
        let Some(sph) = self.inner.sph() else {
            return Float32Array::new_with_length(0);
        };
        let mut flat = Vec::with_capacity(sph.position.len() * 3);
        for p in &sph.position {
            flat.push(p.x as f32);
            flat.push(p.y as f32);
            flat.push(p.z as f32);
        }
        Float32Array::from(&flat[..])
    }

    /// `index`番目のボディの位置 [x, y, z](f32)。
    pub fn body_position_at_f32(&self, index: usize) -> Float32Array {
        let id = self.body_id_at(index);
        let p = self
            .inner
            .body_position(id)
            .expect("body is created in new() and never removed");
        let out = Float32Array::new_with_length(3);
        out.set_index(0, p.x as f32);
        out.set_index(1, p.y as f32);
        out.set_index(2, p.z as f32);
        out
    }

    /// `index`番目のボディの速度 [vx, vy, vz](f32)。
    pub fn body_velocity_at_f32(&self, index: usize) -> Float32Array {
        let id = self.body_id_at(index);
        let v = self
            .inner
            .body_velocity(id)
            .expect("body is created in new() and never removed");
        let out = Float32Array::new_with_length(3);
        out.set_index(0, v.x as f32);
        out.set_index(1, v.y as f32);
        out.set_index(2, v.z as f32);
        out
    }

    /// `index`番目のボディの姿勢クォータニオン [x, y, z, w](f32)。
    pub fn body_rotation_at_f32(&self, index: usize) -> Float32Array {
        let id = self.body_id_at(index);
        let q = self
            .inner
            .body_rotation(id)
            .expect("body is created in new() and never removed");
        let out = Float32Array::new_with_length(4);
        out.set_index(0, q.x as f32);
        out.set_index(1, q.y as f32);
        out.set_index(2, q.z as f32);
        out.set_index(3, q.w as f32);
        out
    }

    /// Editモードの回転Gizmo向けの直接編集(`set_body_position_at`の姿勢版、
    /// 同じくCommandキューを経由しない直接書き換え)。
    pub fn set_body_rotation_at(&mut self, index: usize, x: f64, y: f64, z: f64, w: f64) {
        let id = self.body_id_at(index);
        self.inner.mechanics_mut().bodies.rotation[id.index as usize] =
            sim_math::Quat { x, y, z, w };
    }

    /// Scale Gizmo(縮約実装、`sim_world::World::set_body_shape`のdoc参照)——
    /// ボディの形状をスポーン時の寸法(`base_shape`、床は対象外)の`scale`倍に
    /// 置き換え、質量・慣性を再計算する。`scale`はドラッグ開始時点からの
    /// 相対値ではなく、常に基準形状からの絶対倍率(Translate/Rotate Gizmoの
    /// 「ドラッグ開始値+差分」ではなく「基準値×絶対倍率」という設計、複数回の
    /// ドラッグを重ねても誤差が蓄積しない)。
    pub fn set_body_scale_at(&mut self, index: usize, scale: f64) {
        let id = self.body_id_at(index);
        let base_shape = match index {
            0 => panic!("Ground is static and has no scale handle"),
            1 => Shape::Box {
                half_extents: sim_math::Vec3::new(0.5, 0.5, 0.5),
            },
            _ => self
                .spawned
                .get(index - 2)
                .unwrap_or_else(|| panic!("body index {index} out of range"))
                .base_shape
                .clone(),
        };
        let scaled_shape = match base_shape {
            Shape::Sphere { radius } => Shape::Sphere {
                radius: radius * scale,
            },
            Shape::Box { half_extents } => Shape::Box {
                half_extents: half_extents.scale(scale),
            },
            other => other,
        };
        self.inner.set_body_shape(id, scaled_shape);
    }

    /// 1 world step。1s相当のstep数ごとにTimelineスナップショットを
    /// リングバッファへ記録する(モジュールdoc「スナップショットリングバッファ」
    /// 参照、既存の`World::snapshot`をそのまま使う)。
    pub fn step(&mut self) {
        self.inner.step();
        if self
            .inner
            .step_count()
            .is_multiple_of(self.snapshot_interval_steps)
        {
            if self.snapshots.len() >= SNAPSHOT_RING_CAPACITY {
                self.snapshots.pop_front();
            }
            self.snapshots.push_back(self.inner.snapshot());
        }
    }

    /// Timelineスクラバが表示できるスナップショット数(モジュールdoc参照)。
    pub fn snapshot_count(&self) -> usize {
        self.snapshots.len()
    }

    /// `index`番目のスナップショットの記録時刻(秒、古い順)。
    pub fn snapshot_time_at(&self, index: usize) -> f64 {
        self.snapshots[index].time()
    }

    /// Timelineスクラバ操作: `index`番目のスナップショットへ巻き戻す(既存の
    /// `World::restore`をそのまま使う)。巻き戻した時点より後のスナップショットは
    /// もはや実際の未来を表さないため破棄する(新しいタイムラインがそこから
    /// 再開する、設計の「直前スナップショットへの巻き戻し」と同じ発想)。
    pub fn restore_snapshot(&mut self, index: usize) {
        self.inner.restore(&self.snapshots[index]);
        self.snapshots.truncate(index + 1);
    }

    /// Timelineのブックマーク(設計docs/23-frontend/01-editor.md §1.4
    /// 「ブックマーク: 任意時点にラベル付けし、後で戻れる」)。リングバッファの
    /// 退避に晒されない別領域へ、現在時点のスナップショットをラベル付きで保存する
    /// (既存の`World::snapshot`をそのまま使う)。数の上限は設けない(縮約実装、
    /// シーンJSONと一緒に出す「共有」用途は未実装)。
    pub fn add_bookmark(&mut self, label: String) {
        self.bookmarks.push((label, self.inner.snapshot()));
    }

    pub fn bookmark_count(&self) -> usize {
        self.bookmarks.len()
    }

    pub fn bookmark_label_at(&self, index: usize) -> String {
        self.bookmarks[index].0.clone()
    }

    pub fn bookmark_time_at(&self, index: usize) -> f64 {
        self.bookmarks[index].1.time()
    }

    /// ブックマークへ巻き戻す。`restore_snapshot`と異なり、ブックマーク自体は
    /// 巻き戻し後も残す(いつでも同じブックマークへ再度戻れるように)。ただし
    /// リングバッファ側のスナップショットは、もはや実際の未来を表さないため
    /// 全て破棄する(新しいタイムラインがそこから再開する)。
    pub fn restore_bookmark(&mut self, index: usize) {
        let (_, snapshot) = &self.bookmarks[index];
        self.inner.restore(snapshot);
        self.snapshots.clear();
    }

    pub fn time(&self) -> f64 {
        self.inner.time()
    }

    pub fn step_count(&self) -> u64 {
        self.inner.step_count()
    }

    /// 決定論検証・UI 表示用の状態ハッシュ(16進文字列)。
    pub fn state_hash(&self) -> String {
        format!("{:016x}", self.inner.state_hash())
    }

    /// 箱のy座標プローブの観測履歴(古い順)。エディタのProbe Graphsパネル
    /// (設計docs/23-frontend/01-editor.md §1.4)デモ用。
    pub fn y_probe_history_f64(&self) -> Float64Array {
        let probe = self
            .inner
            .probe(self.y_probe)
            .expect("y_probe is registered in new() and never removed");
        let values: Vec<f64> = probe.history().copied().collect();
        Float64Array::from(values.as_slice())
    }

    /// 箱の速さ(`ProbeTarget::BodySpeed`)プローブの観測履歴(古い順)。
    /// y座標プローブと同じProbe Graphsパネルに2系列目として表示するデモ用。
    pub fn speed_probe_history_f64(&self) -> Float64Array {
        let probe = self
            .inner
            .probe(self.speed_probe)
            .expect("speed_probe is registered in new() and never removed");
        let values: Vec<f64> = probe.history().copied().collect();
        Float64Array::from(values.as_slice())
    }

    /// エディタのPlayモード操作(設計docs/23-frontend/01-editor.md §4「介入は全て
    /// Commandとしてキューに積まれ、次ステップ先頭で適用される」)の最小デモとして、
    /// 箱に力を加えるCommandをキューに積む。重心への加力(トルク無し、`point=None`)。
    pub fn push_apply_force(&mut self, fx: f64, fy: f64, fz: f64) {
        self.inner.push_command(Command::ApplyForce {
            body: self.box_body,
            force: sim_math::Vec3::new(fx, fy, fz),
            point: None,
        });
    }

    /// Scene ViewでのドラッグでD&D的に箱をつかむ(設計§1.2「Gizmo」に相当する
    /// 最小デモ、`Command::Grab`——重心(`anchor_local=Vec3::ZERO`)をワールド座標
    /// `target`へ剛にピン留めする)。
    pub fn push_grab(&mut self, target_x: f64, target_y: f64, target_z: f64) {
        self.inner.push_command(Command::Grab {
            body: self.box_body,
            anchor_local: sim_math::Vec3::ZERO,
            target: sim_math::Vec3::new(target_x, target_y, target_z),
        });
    }

    /// ドラッグ中の`Command::MoveGrab`(既存のgrabの目標点をマウス位置へ追従させる)。
    pub fn push_move_grab(&mut self, target_x: f64, target_y: f64, target_z: f64) {
        self.inner.push_command(Command::MoveGrab {
            body: self.box_body,
            target: sim_math::Vec3::new(target_x, target_y, target_z),
        });
    }

    /// ドラッグ終了時の`Command::Release`(grabを解除、以後は通常の物理に戻る)。
    pub fn push_release(&mut self) {
        self.inner.push_command(Command::Release {
            body: self.box_body,
        });
    }

    /// Editモードの直接編集(設計docs/23-frontend/01-editor.md §4「Editモード:
    /// シーンの直接編集が可能…Scene View gizmo ドラッグ」)。Playモードのドラッグ
    /// (`push_grab`/`push_move_grab`、Command経由・シミュレーション実行中でも
    /// 決定的に記録される)とは異なり、Commandキューを経由せず`RigidBodySet`の
    /// 位置を直接書き換える——設計が「実行中の直接編集は不可」とする境界どおり、
    /// この操作はシミュレーションが進行していない(Editモード中は呼び出し側が
    /// `step()`を呼ばない)ことを前提とする。`World`自体はBodyId経由の位置setterを
    /// 公開していないため(`mechanics_mut().bodies.position`はP1設計が定める
    /// `RigidBodySet`のSoAレイアウト、`docs/10-mechanics/01-rigid-body.md` §4)、
    /// ここで直接アクセスする。
    pub fn set_body_position_at(&mut self, index: usize, x: f64, y: f64, z: f64) {
        let id = self.body_id_at(index);
        self.inner.mechanics_mut().bodies.position[id.index as usize] =
            sim_math::Vec3::new(x, y, z);
    }

    /// Scene View オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「接触点」)向けに、
    /// 直近stepの接触点ワールド座標を`[x0,y0,z0,x1,y1,z1,...]`のフラット配列で返す
    /// (既存の`World::contact_points`をそのまま使う)。
    pub fn contact_points_f32(&self) -> Float32Array {
        let points = self.inner.contact_points();
        let out = Float32Array::new_with_length((points.len() * 3) as u32);
        for (i, p) in points.iter().enumerate() {
            out.set_index((i * 3) as u32, p.x as f32);
            out.set_index((i * 3 + 1) as u32, p.y as f32);
            out.set_index((i * 3 + 2) as u32, p.z as f32);
        }
        out
    }

    /// Consoleパネル(設計docs/23-frontend/01-editor.md §1.5「`SolverDiagnostics`の
    /// 発散警告・…イベントをフィルタ表示」)向けに、前回呼び出し以降に発生した
    /// イベント(既存の`World::drain_events`)を1行1件のテキストへ整形して返す
    /// (空行区切り、`level::message`形式——フロントエンドのフィルタタブが
    /// levelで分けるための単純な区切り、JSONは使わない縮約実装)。この2体デモでは
    /// 箱が床に着地/跳ね返るたびに`ContactStarted`/`ContactEnded`が実際に発生する。
    pub fn drain_events_text(&mut self) -> String {
        self.inner
            .drain_events()
            .iter()
            .map(|e| {
                let level = match e.kind {
                    sim_core::EventKind::FuseBlown
                    | sim_core::EventKind::SolverDiverged
                    | sim_core::EventKind::JointBroken => "warnings",
                    _ => "info",
                };
                format!(
                    "{level}::step={} {:?} (source={})",
                    e.step, e.kind, e.source.0
                )
            })
            .collect::<Vec<_>>()
            .join("\n")
    }
}
