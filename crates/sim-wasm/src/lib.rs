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

use std::collections::{HashMap, VecDeque};

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
    /// コンストラクタに渡された重力・dt(`bookmark_export_scene_json`が
    /// エクスポートするシーンJSONの`world`ブロックに使う——`World`自体はこれらを
    /// 読み出す公開APIを持たないため、構築時の値をここへ複製して保持する)。
    gravity: f64,
    dt: f64,
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
    /// `spawn_fluid_block`を呼んだ回数(複数の水塊を並べてスポーンする際、
    /// 塊どうしが重ならないようX方向のオフセットを決めるのに使う。Hierarchyの
    /// 「Fluids」概要表示にも使う、`fluid_spawn_count`のdoc参照)。
    fluid_spawn_count: u32,
    /// 直近の`import_scene_json`呼び出しが`scenario.probes`から作成したプローブの
    /// ハンドル(`World::probe`用、`scenario.probes`と同じ順)。予測→実験ミニ
    /// パネル(`imported_probe_value_at`のdoc参照)がインポートしたシナリオの
    /// プローブ値を読むために使う。新規Importのたびに置き換わる(縮約実装、
    /// 複数回インポートした場合は最後のシナリオの`probes`のみ対象)。
    imported_probe_handles: Vec<usize>,
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
            gravity,
            dt,
            ground_body,
            box_body,
            spawned: Vec::new(),
            y_probe,
            speed_probe,
            circuit_switch_index,
            snapshot_interval_steps,
            snapshots: VecDeque::with_capacity(SNAPSHOT_RING_CAPACITY),
            bookmarks: Vec::new(),
            fluid_spawn_count: 0,
            imported_probe_handles: Vec::new(),
        }
    }

    /// Hierarchyパネルが列挙するボディ数(固定2体+スポーンパレットで追加した分、
    /// モジュールdoc「複数ボディ対応」参照)。
    pub fn body_count(&self) -> usize {
        2 + self.spawned.len()
    }

    /// `index`をボディIDへ解決する。範囲外なら`JsValue`(メッセージ文字列)を返す
    /// (**2026-07-27の監査で修正**: 以前は`panic!`していたが、この`index`は
    /// JS側から渡される値でありシーン再読み込み後の古い参照や単純な入力ミスで
    /// 容易に範囲外になり得る。wasmの`panic`はモジュール全体を使用不能にする
    /// ため——`console_error_panic_hook`を導入していない現状では捕捉不能な
    /// wasmトラップとしてJSに伝わり、以後同じ`WasmWorld`インスタンスへの呼び出しが
    /// 全て失敗し得る——`Result`によるエラー返却へ置き換えた。wasm-bindgenは
    /// `Result<T, JsValue>`を返すexport関数を、成功時は`T`をそのまま返し失敗時は
    /// 通常の(捕捉可能な)JS例外をthrowする形にバインドするため、TypeScript側の
    /// 呼び出し規約は変わらない)。
    fn try_body_id_at(&self, index: usize) -> Result<BodyId, JsValue> {
        match index {
            0 => Ok(self.ground_body),
            1 => Ok(self.box_body),
            _ => self
                .spawned
                .get(index - 2)
                .map(|meta| meta.id)
                .ok_or_else(|| {
                    JsValue::from_str(&format!(
                        "body index {index} out of range (body_count={})",
                        self.body_count()
                    ))
                }),
        }
    }

    /// Hierarchyパネル表示用のラベル。
    pub fn body_label_at(&self, index: usize) -> Result<String, JsValue> {
        Ok(match index {
            0 => "Ground".to_string(),
            1 => "Box_1".to_string(),
            _ => self
                .spawned
                .get(index - 2)
                .ok_or_else(|| JsValue::from_str(&format!("body index {index} out of range")))?
                .label
                .clone(),
        })
    }

    /// `index`番目のボディが静的(Static)かどうか。InspectorがTransformの速度欄を
    /// 意味のある形で表示するための補助(静的ボディは速度が常に0で自明なため)。
    /// `World::mechanics().bodies.body_type`を実クエリする(以前は「index==0
    /// (固定の床)のみ静的」という決め打ちだったが、シーンJSON Import
    /// (`import_scene_json`)で任意のindexに静的ボディが追加され得るようになった
    /// ため、実際の`BodyType`を見るクエリに置き換えた——`body_shape_label_at`が
    /// 既に辿った同じ理由)。
    pub fn body_is_static_at(&self, index: usize) -> Result<bool, JsValue> {
        let id = self.try_body_id_at(index)?;
        Ok(matches!(
            self.inner.mechanics().bodies.body_type[id.index as usize],
            BodyType::Static
        ))
    }

    /// Inspector表示用のShape文字列。`World::mechanics().bodies.shape_of`で
    /// 実際の現在の形状(Scale Gizmoで変更済みなら変更後の寸法)を読み、
    /// フォーマットする(以前は`SpawnedBodyMeta`にスポーン時の文字列を固定で
    /// 覚えておく縮約実装だったが、Scale Gizmoで寸法が変わりうるようになった
    /// ため、常に最新の値を返す実クエリに置き換えた)。
    pub fn body_shape_label_at(&self, index: usize) -> Result<String, JsValue> {
        let id = self.try_body_id_at(index)?;
        Ok(
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
            },
        )
    }

    /// Prefab機能(設計docs/23-frontend/01-editor.md §6「Prefabs: 再利用可能な
    /// Body/Joint/Circuit組(自作シーンから...「Prefabとして保存」)。他シーンへ
    /// ドラッグで再利用」の縮約実装——Bodyの形状/材質のみ対象、Joint/Circuit組は
    /// 対象外)向けに、`index`番目のボディの形状の種類を返す
    /// ("sphere"/"box"/"plane"/"other")。`body_shape_params_f64_at`と対で使う。
    pub fn body_shape_kind_at(&self, index: usize) -> Result<String, JsValue> {
        let id = self.try_body_id_at(index)?;
        Ok(
            match self.inner.mechanics().bodies.shape_of(id.index as usize) {
                Shape::Sphere { .. } => "sphere".to_string(),
                Shape::Box { .. } => "box".to_string(),
                Shape::Plane { .. } => "plane".to_string(),
                _ => "other".to_string(),
            },
        )
    }

    /// `body_shape_kind_at`と対応する数値パラメータ: sphere→`[radius]`、
    /// box→`[hx,hy,hz]`、plane→`[nx,ny,nz,d]`、other→空配列(Prefab保存用)。
    pub fn body_shape_params_f64_at(&self, index: usize) -> Result<Float64Array, JsValue> {
        let id = self.try_body_id_at(index)?;
        Ok(
            match self.inner.mechanics().bodies.shape_of(id.index as usize) {
                Shape::Sphere { radius } => Float64Array::from(&[*radius][..]),
                Shape::Box { half_extents } => {
                    Float64Array::from(&[half_extents.x, half_extents.y, half_extents.z][..])
                }
                Shape::Plane { normal, d } => {
                    Float64Array::from(&[normal.x, normal.y, normal.z, *d][..])
                }
                _ => Float64Array::from(&[][..]),
            },
        )
    }

    /// Inspector表示用の材質名。
    pub fn body_material_label_at(&self, index: usize) -> Result<String, JsValue> {
        Ok(match index {
            0 => "コンクリート".to_string(),
            1 => "鋼(炭素鋼)".to_string(),
            _ => self
                .spawned
                .get(index - 2)
                .ok_or_else(|| JsValue::from_str(&format!("body index {index} out of range")))?
                .material_label
                .clone(),
        })
    }

    /// Projectドロワー Materials タブ(設計docs/23-frontend/01-editor.md §1.6
    /// 「Materials: MaterialDbプリセット一覧」)向けに、指定した材質名の主要物性値を
    /// `[density, friction, restitution, specific_heat, conductivity]`の順で返す。
    /// 未知の名前なら`JsValue`エラーを返す(呼び出し側UIが`SPAWN_MATERIALS`等の
    /// 既知の名前だけを渡す前提だが、**2026-07-27の監査で修正**: 以前は
    /// `panic!`していた——`try_body_id_at`のdocと同じ理由でResult化した)。
    pub fn material_properties_f64(&self, name: String) -> Result<Float64Array, JsValue> {
        let id = self
            .inner
            .materials()
            .find_by_name(&name)
            .ok_or_else(|| JsValue::from_str(&format!("unknown material: {name}")))?;
        let m = self.inner.materials().get(id);
        Ok(Float64Array::from(
            &[
                m.density,
                m.friction,
                m.restitution,
                m.specific_heat,
                m.conductivity,
            ][..],
        ))
    }

    /// スポーンパレット(設計docs/23-frontend/01-editor.md §6)——球を`material_name`
    /// (`MaterialDb::standard`が持つ名前)で`(x,y,z)`に配置する。新しいボディの
    /// index(`body_count`と同じ体系)を返す。未知の材質名なら`JsValue`エラーを
    /// 返す(呼び出し側UIが既知の名前だけを選択肢にする前提だが、
    /// `material_properties_f64`のdocと同じ理由でResult化した)。
    pub fn spawn_sphere(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        radius: f64,
        material_name: String,
    ) -> Result<usize, JsValue> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| JsValue::from_str(&format!("unknown material: {material_name}")))?;
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
        Ok(index)
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
    ) -> Result<usize, JsValue> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| JsValue::from_str(&format!("unknown material: {material_name}")))?;
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
        Ok(index)
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

        // `scenario.probes`(予測→実験ミニパネルが参照する`probe_index`は
        // この配列の並びに対応する)を、`append_scenario_bodies`とは別に
        // `World::add_scenario_probes`で設定する(`append_scenario_bodies`自体は
        // probesを対象外とする設計のまま、そのdoc参照)。
        let mut body_ids_by_name: HashMap<String, BodyId> = HashMap::new();
        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            if let Some(name) = &body.name {
                body_ids_by_name.insert(name.clone(), *id);
            }
        }
        self.imported_probe_handles = self
            .inner
            .add_scenario_probes(&scenario, &body_ids_by_name)
            .map_err(|e| JsValue::from_str(&format!("{e:?}")))?;

        Ok(scenario.bodies.len())
    }

    /// 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5)向けに、直近の
    /// `import_scene_json`が`scenario.probes`から作成したプローブの現在値を返す。
    /// `probe_index`は`scenario.probes`配列内でのインデックス(`prediction_prompts
    /// [].probe_index`と同じ添字系)。範囲外、または該当プローブの履歴がまだ
    /// 1件も無い(1stepも進んでいない)場合は0.0を返す。
    pub fn imported_probe_value_at(&self, probe_index: usize) -> f64 {
        self.imported_probe_handles
            .get(probe_index)
            .and_then(|&handle| self.inner.probe(handle))
            .and_then(|probe| probe.history().last().copied())
            .unwrap_or(0.0)
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
    ) -> Result<usize, JsValue> {
        const BOB_RADIUS: f64 = 0.3;
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| JsValue::from_str(&format!("unknown material: {material_name}")))?;
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
        Ok(index)
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
    ) -> Result<usize, JsValue> {
        const HALF_EXTENTS: sim_math::Vec3 = sim_math::Vec3 {
            x: 0.1,
            y: 0.6,
            z: 0.1,
        };
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| JsValue::from_str(&format!("unknown material: {material_name}")))?;
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
        Ok(index)
    }

    /// `Command::SetMotorTarget`(モジュールdoc参照、`theta_target`は
    /// ラジアン)を、`index`番目のボディが持つヒンジモーターへ送る。
    /// モーターを持たないボディに呼ぶと`JsValue`エラーを返す(呼び出し側UIが
    /// モーターを持つボディだけに対して呼ぶ前提だが、`try_body_id_at`のdocと
    /// 同じ理由でResult化した)。
    pub fn set_motor_target_at(&mut self, index: usize, theta_target: f64) -> Result<(), JsValue> {
        let hinge_motor_index = match index {
            0 | 1 => None,
            _ => {
                self.spawned
                    .get(index - 2)
                    .ok_or_else(|| JsValue::from_str(&format!("body index {index} out of range")))?
                    .hinge_motor_index
            }
        };
        let hinge_motor_index = hinge_motor_index
            .ok_or_else(|| JsValue::from_str(&format!("body index {index} has no hinge motor")))?;
        self.inner.push_command(Command::SetMotorTarget {
            hinge_motor_index,
            theta_target,
        });
        Ok(())
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

    /// 自由配線回路エディタ(設計docs/23-frontend/01-editor.md §6「回路エディタ
    /// サブモード」(D19)の縮約実装——専用のグラフィカルなノード配線UIではなく、
    /// Circuitタブのフォームベースの操作でノード/素子を追加していく形とした)
    /// 向けに、既存の固定デモ回路(`WasmWorld::new`が構築したもの)を`num_nodes`
    /// 個のノードを持つ空の回路で置き換える。以後`circuit_editor_add_*`で
    /// ユーザーが自由に素子を追加していく。置き換え後は固定デモの
    /// `circuit_switch_index`が新回路のスイッチ数を超えて無効になり得るため、
    /// 呼び出し側(`main.ts`)は既存の「回路スイッチ(閉)」チェックボックスを
    /// 無効化する責任を負う(`set_circuit_switch_closed`をこの後に呼ぶと
    /// パニックし得る)。
    pub fn circuit_editor_reset(&mut self, num_nodes: usize) {
        self.inner.enable_circuit(sim_em::Circuit::new(num_nodes));
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間に抵抗`resistance`[Ω]を追加する。
    /// `circuit_editor_reset`より前に呼ぶと(回路が未有効化)何もしない。
    pub fn circuit_editor_add_resistor(&mut self, a: usize, b: usize, resistance: f64) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_resistor(a, b, resistance);
        }
    }

    /// 自由配線回路エディタ——ノード`a`(正極)・`b`(負極)間に独立電圧源
    /// `voltage`[V]を追加する。
    pub fn circuit_editor_add_voltage_source(&mut self, a: usize, b: usize, voltage: f64) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_voltage_source(a, b, voltage);
        }
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間に理想スイッチを追加する。返り値は
    /// このスイッチのindex(`circuit_editor_set_switch_closed`用)。回路が
    /// 未有効化なら0を返す(縮約実装、呼び出し側は`circuit_editor_reset`を
    /// 先に呼ぶ前提)。
    pub fn circuit_editor_add_switch(&mut self, a: usize, b: usize, closed: bool) -> usize {
        self.inner
            .circuit_mut()
            .map_or(0, |circuit| circuit.add_switch(a, b, closed))
    }

    /// 自由配線回路エディタ——`circuit_editor_add_switch`が返したindexのスイッチの
    /// 開閉状態を変更する(既存の`set_circuit_switch_closed`と異なりCommandキューを
    /// 経由しない即時変更——自由配線回路の構築/操作全体がEditモード的な直接操作
    /// として設計されているため、`spawn_sphere`等と同じ即時反映の扱いとした)。
    pub fn circuit_editor_set_switch_closed(&mut self, index: usize, closed: bool) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.set_switch_closed(index, closed);
        }
    }

    /// 自由配線回路エディタ——任意ノードの電圧(既存の固定デモ専用
    /// `circuit_divider_voltage`の一般化、`World::circuit_probe`をそのまま使う)。
    pub fn circuit_node_voltage(&self, node: usize) -> f64 {
        self.inner.circuit_probe(node).unwrap_or(0.0)
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
    pub fn constraint_anchor_points_at(&self, index: usize) -> Result<Float32Array, JsValue> {
        let joint_index = match index {
            0 | 1 => None,
            _ => {
                self.spawned
                    .get(index - 2)
                    .ok_or_else(|| JsValue::from_str(&format!("body index {index} out of range")))?
                    .constraint_joint_index
            }
        };
        let Some(joint_index) = joint_index else {
            return Ok(Float32Array::new_with_length(0));
        };
        let (a, b) = self
            .inner
            .distance_joint_anchor_points(joint_index)
            .expect("constraint_joint_index recorded at spawn time must stay valid");
        Ok(Float32Array::from(
            &[
                a.x as f32, a.y as f32, a.z as f32, b.x as f32, b.y as f32, b.z as f32,
            ][..],
        ))
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

    /// `frame_index`が有効な範囲内かを検証する(**2026-07-27の監査で追加**:
    /// `sim_core::FrameTree::frame`は範囲外の`FrameId`に対し生スライスindexで
    /// パニックする——`try_body_id_at`のdocと同じ理由で、フレームindexを扱う
    /// 各wasm公開メソッドの入口でここを通す)。フレームは`add_frame`/
    /// `add_child_frame`で単調増加するのみ(削除が無い)ため、
    /// `frame_index < frame_count()`が有効性の必要十分条件になる。
    fn check_frame_index(&self, frame_index: usize) -> Result<(), JsValue> {
        if frame_index < self.frame_count() {
            Ok(())
        } else {
            Err(JsValue::from_str(&format!(
                "frame index {frame_index} out of range (frame_count={})",
                self.frame_count()
            )))
        }
    }

    /// `frame_index`番目のフレームの現在の姿勢(親フレームからの相対回転)を
    /// クォータニオン`[x, y, z, w]`(f32)で返す。
    pub fn frame_rotation_at_f32(&self, frame_index: usize) -> Result<Float32Array, JsValue> {
        self.check_frame_index(frame_index)?;
        let rotation = self
            .inner
            .frames()
            .frame(FrameId(frame_index as u32))
            .rotation_in_parent;
        Ok(Float32Array::from(
            &[
                rotation.x as f32,
                rotation.y as f32,
                rotation.z as f32,
                rotation.w as f32,
            ][..],
        ))
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
    pub fn frame_parent_index(&self, frame_index: usize) -> Result<i32, JsValue> {
        self.check_frame_index(frame_index)?;
        Ok(
            match self
                .inner
                .frames()
                .frame(FrameId(frame_index as u32))
                .parent
            {
                Some(parent) => parent.0 as i32,
                None => -1,
            },
        )
    }

    /// `frame_index`番目のフレームのROOT(ワールド)座標系での位置
    /// (`sim_core::FrameTree::transform_to_root`)。`frame_rotation_at_f32`
    /// (親フレームからの相対回転のみ、単一のROOT直下フレームを想定していた
    /// 旧API)と異なり、複数フレームが親子関係を持つ場合(フレーム階層
    /// ドリルインUI)でも階層を遡って合成した実際のワールド位置を返す。
    pub fn frame_world_position_f32(&self, frame_index: usize) -> Result<Float32Array, JsValue> {
        self.check_frame_index(frame_index)?;
        let position = self
            .inner
            .frames()
            .transform_to_root(FrameId(frame_index as u32))
            .position;
        Ok(Float32Array::from(
            &[position.x as f32, position.y as f32, position.z as f32][..],
        ))
    }

    /// `frame_index`番目のフレームのROOT(ワールド)座標系での姿勢
    /// (`frame_world_position_f32`と同じ理由で`transform_to_root`を使う)。
    pub fn frame_world_rotation_f32(&self, frame_index: usize) -> Result<Float32Array, JsValue> {
        self.check_frame_index(frame_index)?;
        let rotation = self
            .inner
            .frames()
            .transform_to_root(FrameId(frame_index as u32))
            .rotation;
        Ok(Float32Array::from(
            &[
                rotation.x as f32,
                rotation.y as f32,
                rotation.z as f32,
                rotation.w as f32,
            ][..],
        ))
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
    ) -> Result<usize, JsValue> {
        // `World::add_frame`は範囲外の親`FrameId`に対し`assert!`でパニックする
        // (`sim_core::FrameTree::add_frame`)ため、`try_body_id_at`のdocと同じ
        // 理由でここも事前に検証する。
        self.check_frame_index(parent_index)?;
        let id = self.inner.add_frame(
            FrameId(parent_index as u32),
            Vec3::new(origin_offset_x, origin_offset_y, origin_offset_z),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, angular_velocity_z),
        );
        Ok(id.0 as usize)
    }

    /// 流体場オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「流体場」の土台)
    /// 向けに、小さな水塊(3×3×3粒子)+その直下の床(1層の境界粒子、
    /// `SphFluid::add_boundary_particle`)を追加する。既にSPH流体が有効なら
    /// (`World::sph_mut`)そこへ粒子を追加する(複数回スポーンすると水塊が
    /// 増えていく、`fluid_spawn_count`でX方向にずらして重なりを避ける)。
    /// まだ無効なら新規`SphFluid`を構築して有効化する(初回のみ)。
    pub fn spawn_fluid_block(&mut self) {
        let h: f64 = 0.15;
        let rho0: f64 = 1000.0;
        let c_s: f64 = 20.0;
        let dx: f64 = 0.1;
        let n = 3;
        let floor_half = 0.6;
        let floor_y = -dx;

        let spawn_index = self.fluid_spawn_count;
        self.fluid_spawn_count += 1;
        let origin = Vec3::new(3.0 + spawn_index as f64 * 1.5, 2.0, 0.0);

        let mut particle_positions = Vec::with_capacity(n * n * n);
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    particle_positions
                        .push(origin + Vec3::new(ix as f64 * dx, iy as f64 * dx, iz as f64 * dx));
                }
            }
        }
        let mut boundary_positions = Vec::new();
        let mut fx = origin.x - floor_half;
        while fx <= origin.x + floor_half {
            let mut fz = origin.z - floor_half;
            while fz <= origin.z + floor_half {
                boundary_positions.push(Vec3::new(fx, floor_y, fz));
                fz += dx;
            }
            fx += dx;
        }

        if let Some(sph) = self.inner.sph_mut() {
            for p in particle_positions {
                sph.add_particle(p, Vec3::ZERO);
            }
            for b in boundary_positions {
                sph.add_boundary_particle(b);
            }
        } else {
            let mut sph = sim_fluid::SphFluid::new(h, rho0, c_s);
            sph.mass = rho0 * dx.powi(3);
            for p in particle_positions {
                sph.add_particle(p, Vec3::ZERO);
            }
            for b in boundary_positions {
                sph.add_boundary_particle(b);
            }
            self.inner.enable_sph(sph);
        }
    }

    /// `spawn_fluid_block`を呼んだ回数(Hierarchyの「Fluids」概要表示用)。
    pub fn fluid_spawn_count(&self) -> u32 {
        self.fluid_spawn_count
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
    pub fn body_position_at_f32(&self, index: usize) -> Result<Float32Array, JsValue> {
        let id = self.try_body_id_at(index)?;
        let p = self
            .inner
            .body_position(id)
            .expect("body is created in new() and never removed");
        let out = Float32Array::new_with_length(3);
        out.set_index(0, p.x as f32);
        out.set_index(1, p.y as f32);
        out.set_index(2, p.z as f32);
        Ok(out)
    }

    /// `index`番目のボディの速度 [vx, vy, vz](f32)。
    pub fn body_velocity_at_f32(&self, index: usize) -> Result<Float32Array, JsValue> {
        let id = self.try_body_id_at(index)?;
        let v = self
            .inner
            .body_velocity(id)
            .expect("body is created in new() and never removed");
        let out = Float32Array::new_with_length(3);
        out.set_index(0, v.x as f32);
        out.set_index(1, v.y as f32);
        out.set_index(2, v.z as f32);
        Ok(out)
    }

    /// `index`番目のボディの姿勢クォータニオン [x, y, z, w](f32)。
    pub fn body_rotation_at_f32(&self, index: usize) -> Result<Float32Array, JsValue> {
        let id = self.try_body_id_at(index)?;
        let q = self
            .inner
            .body_rotation(id)
            .expect("body is created in new() and never removed");
        let out = Float32Array::new_with_length(4);
        out.set_index(0, q.x as f32);
        out.set_index(1, q.y as f32);
        out.set_index(2, q.z as f32);
        out.set_index(3, q.w as f32);
        Ok(out)
    }

    /// Editモードの回転Gizmo向けの直接編集(`set_body_position_at`の姿勢版、
    /// 同じくCommandキューを経由しない直接書き換え)。
    pub fn set_body_rotation_at(
        &mut self,
        index: usize,
        x: f64,
        y: f64,
        z: f64,
        w: f64,
    ) -> Result<(), JsValue> {
        let id = self.try_body_id_at(index)?;
        self.inner.mechanics_mut().bodies.rotation[id.index as usize] =
            sim_math::Quat { x, y, z, w };
        Ok(())
    }

    /// Scale Gizmo(縮約実装、`sim_world::World::set_body_shape`のdoc参照)——
    /// ボディの形状をスポーン時の寸法(`base_shape`、床は対象外)の`scale`倍に
    /// 置き換え、質量・慣性を再計算する。`scale`はドラッグ開始時点からの
    /// 相対値ではなく、常に基準形状からの絶対倍率(Translate/Rotate Gizmoの
    /// 「ドラッグ開始値+差分」ではなく「基準値×絶対倍率」という設計、複数回の
    /// ドラッグを重ねても誤差が蓄積しない)。
    pub fn set_body_scale_at(&mut self, index: usize, scale: f64) -> Result<(), JsValue> {
        let id = self.try_body_id_at(index)?;
        let base_shape = match index {
            0 => {
                return Err(JsValue::from_str(
                    "Ground is static and has no scale handle",
                ));
            }
            1 => Shape::Box {
                half_extents: sim_math::Vec3::new(0.5, 0.5, 0.5),
            },
            _ => self
                .spawned
                .get(index - 2)
                .ok_or_else(|| JsValue::from_str(&format!("body index {index} out of range")))?
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
        Ok(())
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

    /// `index`が有効なスナップショットindexかを検証する(**2026-07-27の監査で
    /// 追加**: `VecDeque`の生indexアクセスは範囲外でパニックする、
    /// `try_body_id_at`のdocと同じ理由)。
    fn try_snapshot_at(&self, index: usize) -> Result<&World, JsValue> {
        self.snapshots.get(index).ok_or_else(|| {
            JsValue::from_str(&format!(
                "snapshot index {index} out of range (snapshot_count={})",
                self.snapshots.len()
            ))
        })
    }

    /// `index`番目のスナップショットの記録時刻(秒、古い順)。
    pub fn snapshot_time_at(&self, index: usize) -> Result<f64, JsValue> {
        Ok(self.try_snapshot_at(index)?.time())
    }

    /// Timelineスクラバ操作: `index`番目のスナップショットへ巻き戻す(既存の
    /// `World::restore`をそのまま使う)。巻き戻した時点より後のスナップショットは
    /// もはや実際の未来を表さないため破棄する(新しいタイムラインがそこから
    /// 再開する、設計の「直前スナップショットへの巻き戻し」と同じ発想)。
    pub fn restore_snapshot(&mut self, index: usize) -> Result<(), JsValue> {
        // `try_snapshot_at`は`&self`(disjointでない全体借用)を取るため、
        // その戻り値を保持したまま`&mut self.inner`は取れない。フィールドへ
        // 直接アクセスして借用チェッカに`snapshots`と`inner`が別フィールド
        // であることを見せる。
        let snapshot = self.snapshots.get(index).ok_or_else(|| {
            JsValue::from_str(&format!(
                "snapshot index {index} out of range (snapshot_count={})",
                self.snapshots.len()
            ))
        })?;
        self.inner.restore(snapshot);
        self.snapshots.truncate(index + 1);
        Ok(())
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

    /// `index`が有効なブックマークindexかを検証する(**2026-07-27の監査で
    /// 追加**: `Vec`の生indexアクセスは範囲外でパニックする、
    /// `try_body_id_at`のdocと同じ理由)。
    fn try_bookmark_at(&self, index: usize) -> Result<&(String, World), JsValue> {
        self.bookmarks.get(index).ok_or_else(|| {
            JsValue::from_str(&format!(
                "bookmark index {index} out of range (bookmark_count={})",
                self.bookmarks.len()
            ))
        })
    }

    pub fn bookmark_label_at(&self, index: usize) -> Result<String, JsValue> {
        Ok(self.try_bookmark_at(index)?.0.clone())
    }

    pub fn bookmark_time_at(&self, index: usize) -> Result<f64, JsValue> {
        Ok(self.try_bookmark_at(index)?.1.time())
    }

    /// ブックマークのエクスポート(設計docs/23-frontend/01-editor.md §6
    /// 「保存・共有: シーンJSON+Replay+ブックマークを単一ファイルとして
    /// エクスポート」の縮約実装)。`World`自体は`Serialize`を持たない
    /// (`sim-wasm`は`serde_json`にも依存しない)ため、内部状態のバイト単位の
    /// 保存ではなく、シーンJSON Import(`import_scene_json`)へそのまま
    /// 読み込める`sim_world::Scenario`互換のJSON文字列として剛体の観測可能な
    /// 状態(位置・姿勢・速度)を書き出す——流体/熱/回路ドメインの状態や
    /// イベントログ・接触履歴は対象外(縮約実装、既知の限定)。
    ///
    /// ボディの形状/材質は現在のワールド(スケールGizmo等で変わりうる)ではなく
    /// ブックマーク取得時点の値を使うべきだが、`SpawnedBodyMeta`はスポーン時の
    /// 基準形状のみ保持し履歴を持たないため、現在の形状/材質ラベルをそのまま
    /// 使う(通常は変化しないため実用上問題にならない、既知の簡略化)。
    /// ブックマーク時点にまだ存在しなかったボディ(`body_position`が`None`を
    /// 返す)はスキップする。
    pub fn bookmark_export_scene_json(&self, index: usize) -> Result<String, JsValue> {
        let (label, snapshot) = self.try_bookmark_at(index)?;
        let mut bodies_json = Vec::new();
        for i in 0..self.body_count() {
            // `i`は`0..body_count()`の範囲内なので必ず解決できる
            // (`try_body_id_at`のdoc参照——削除が無いため常に有効)。
            let id = self
                .try_body_id_at(i)
                .expect("index within 0..body_count() is always valid");
            let (Some(position), Some(rotation), Some(velocity)) = (
                snapshot.body_position(id),
                snapshot.body_rotation(id),
                snapshot.body_velocity(id),
            ) else {
                continue; // ブックマーク時点にまだ存在しなかったボディ。
            };
            // シーンJSONの`ShapeJson`が対応する3形状のみ書き出せる(`Capsule`等は
            // `ShapeJson`自体が対象外、`scenario.rs`のdoc参照)。それ以外の形状の
            // ボディはこのエクスポートでは省略する(既知の限定)。
            let Some(shape_json) = (match self.inner.mechanics().bodies.shape_of(id.index as usize)
            {
                Shape::Sphere { radius } => Some(format!(r#"{{"sphere":{{"radius":{radius}}}}}"#)),
                Shape::Box { half_extents } => Some(format!(
                    r#"{{"box":{{"half":[{},{},{}]}}}}"#,
                    half_extents.x, half_extents.y, half_extents.z
                )),
                Shape::Plane { normal, d } => Some(format!(
                    r#"{{"plane":{{"normal":[{},{},{}],"d":{d}}}}}"#,
                    normal.x, normal.y, normal.z
                )),
                _ => None,
            }) else {
                continue;
            };
            let type_json = if self.body_is_static_at(i)? {
                r#","type":"static""#
            } else {
                ""
            };
            bodies_json.push(format!(
                r#"{{"shape":{shape_json},"material":"{material}","position":[{px},{py},{pz}],"rotation":[{qx},{qy},{qz},{qw}],"linear_velocity":[{vx},{vy},{vz}],"name":"{name}"{type_json}}}"#,
                material = self.body_material_label_at(i)?,
                px = position.x,
                py = position.y,
                pz = position.z,
                qx = rotation.x,
                qy = rotation.y,
                qz = rotation.z,
                qw = rotation.w,
                vx = velocity.x,
                vy = velocity.y,
                vz = velocity.z,
                name = self.body_label_at(i)?,
            ));
        }
        Ok(format!(
            r#"{{"name":"bookmark-{label}","world":{{"gravity":{gravity},"dt":{dt}}},"bodies":[{bodies}]}}"#,
            gravity = self.gravity,
            dt = self.dt,
            bodies = bodies_json.join(",")
        ))
    }

    /// ブックマークへ巻き戻す。`restore_snapshot`と異なり、ブックマーク自体は
    /// 巻き戻し後も残す(いつでも同じブックマークへ再度戻れるように)。ただし
    /// リングバッファ側のスナップショットは、もはや実際の未来を表さないため
    /// 全て破棄する(新しいタイムラインがそこから再開する)。
    pub fn restore_bookmark(&mut self, index: usize) -> Result<(), JsValue> {
        // `restore_snapshot`と同じ理由でフィールドへ直接アクセスする
        // (借用チェッカに`bookmarks`と`inner`が別フィールドであることを見せる)。
        let (_, snapshot) = self.bookmarks.get(index).ok_or_else(|| {
            JsValue::from_str(&format!(
                "bookmark index {index} out of range (bookmark_count={})",
                self.bookmarks.len()
            ))
        })?;
        self.inner.restore(snapshot);
        self.snapshots.clear();
        Ok(())
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
    pub fn set_body_position_at(
        &mut self,
        index: usize,
        x: f64,
        y: f64,
        z: f64,
    ) -> Result<(), JsValue> {
        let id = self.try_body_id_at(index)?;
        self.inner.mechanics_mut().bodies.position[id.index as usize] =
            sim_math::Vec3::new(x, y, z);
        Ok(())
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

/// **2026-07-27の監査で追加**: このcrate(JS/WASM境界、1280行)にテストが
/// 1本も無かった(Rustワークスペース最大の未テスト面)ため、Q5(wasm境界の
/// パニック除去)の作業とあわせて最小限のユニットテストを追加する。
///
/// **正直な制約(実際に検証した結果、当初の想定より厳しいことが判明した)**:
/// `js_sys::Float32Array`/`Float64Array`と`wasm_bindgen::JsValue`はいずれも、
/// 実際のwasmホスト(ブラウザ/Node)無しでは**値を構築すること自体ができない**。
/// 実験の結果、`Float32Array::new_with_length`はネイティブターゲットで
/// 「cannot call wasm-bindgen imported functions on non-wasm targets」と
/// (unwind可能な)パニックを起こす一方、`JsValue::from_str`は**unwindしない
/// プロセスabort(SIGABRT)** を起こすことを確認した——つまり`Result<_, JsValue>`
/// の`Err`分岐を`assert!(result.is_err())`のような形で検証しようとするテストは、
/// `Err`を構築した時点で**テストプロセスごと**abortする(該当テストの
/// `#[should_panic]`でも捕捉できない)。したがって本テストモジュールは
/// **成功パスのみ**(`Float32Array`/`Float64Array`を返す関数の戻り値自体も
/// 検証できないため、そちらは「パニックせず`Ok`を返した」ことの確認に留める)
/// に限定する。エラーパス・`Float32Array`/`Float64Array`の中身の実行時検証には
/// `wasm-bindgen-test`(`wasm-pack test --node`等)の導入が要るが、本増分の
/// スコープ外として正直に記録する(CIにもこのcrateにも現状
/// `wasm-bindgen-test`は無い、`docs/22-roadmap/02-feature-checklist.md`参照)。
#[cfg(test)]
mod tests {
    use super::*;

    fn new_world() -> WasmWorld {
        WasmWorld::new(-9.80665, 1.0 / 60.0, 5.0)
    }

    /// 固定2体(床・箱)のラベル・材質・静的判定が期待どおりであること。
    #[test]
    fn fixed_bodies_have_expected_labels_and_materials() {
        let world = new_world();
        assert_eq!(world.body_count(), 2);
        assert_eq!(world.body_label_at(0).unwrap(), "Ground");
        assert_eq!(world.body_label_at(1).unwrap(), "Box_1");
        assert_eq!(world.body_material_label_at(0).unwrap(), "コンクリート");
        assert_eq!(world.body_material_label_at(1).unwrap(), "鋼(炭素鋼)");
        assert!(world.body_is_static_at(0).unwrap());
        assert!(!world.body_is_static_at(1).unwrap());
    }

    /// `spawn_sphere`/`spawn_box`が正しい材質名で成功し、`body_count`が
    /// 増分どおりに伸び、新しいボディのラベル・形状種別・材質が読めること
    /// (Q5でResult化した成功パスの回帰テスト)。
    #[test]
    fn spawn_sphere_and_box_succeed_and_extend_body_count() {
        let mut world = new_world();
        let sphere_index = world
            .spawn_sphere(1.0, 2.0, 3.0, 0.5, "コンクリート".to_string())
            .expect("known material name must succeed");
        assert_eq!(sphere_index, 2);
        assert_eq!(world.body_count(), 3);
        assert_eq!(world.body_shape_kind_at(sphere_index).unwrap(), "sphere");
        assert_eq!(
            world.body_material_label_at(sphere_index).unwrap(),
            "コンクリート"
        );

        let box_index = world
            .spawn_box(0.0, 0.0, 0.0, 0.25, "鋼(炭素鋼)".to_string())
            .expect("known material name must succeed");
        assert_eq!(box_index, 3);
        assert_eq!(world.body_count(), 4);
        assert_eq!(world.body_shape_kind_at(box_index).unwrap(), "box");
    }

    /// フレーム階層: ROOTの子フレームを追加でき、親indexが正しく読めること。
    #[test]
    fn add_child_frame_succeeds_and_reports_correct_parent() {
        let mut world = new_world();
        assert_eq!(world.frame_count(), 1); // ROOTのみ。
        let child = world
            .add_child_frame(0, 1.0, 0.0, 0.0, 0.5)
            .expect("ROOT is always a valid parent");
        assert_eq!(child, 1);
        assert_eq!(world.frame_count(), 2);
        assert_eq!(world.frame_parent_index(child).unwrap(), 0);
        assert_eq!(world.frame_parent_index(0).unwrap(), -1); // ROOTは親を持たない。
    }

    /// ブックマークの追加・ラベル/時刻の読み取り・巻き戻しが成功パスで
    /// 期待どおり動くこと。
    #[test]
    fn bookmark_add_and_restore_round_trips_successfully() {
        let mut world = new_world();
        world.step();
        let time_at_bookmark = world.time();
        world.add_bookmark("test-bookmark".to_string());
        assert_eq!(world.bookmark_count(), 1);
        assert_eq!(world.bookmark_label_at(0).unwrap(), "test-bookmark");
        assert!((world.bookmark_time_at(0).unwrap() - time_at_bookmark).abs() < 1e-12);

        world.step();
        world.step();
        assert!(world.time() > time_at_bookmark);

        world.restore_bookmark(0).expect("bookmark 0 must exist");
        assert!((world.time() - time_at_bookmark).abs() < 1e-12);
    }

    /// 位置/姿勢の直接編集(Gizmo相当)が成功パスで正しく反映されること
    /// (`Float32Array`経由の読み出し自体はネイティブターゲットで検証できない
    /// ため、モジュールdoc「正直な制約」参照のとおりパニックせず成功した
    /// ことのみを確認する)。
    #[test]
    fn set_body_position_and_rotation_succeed_for_a_valid_body() {
        let mut world = new_world();
        world.set_body_position_at(1, 7.0, 8.0, 9.0).unwrap();
        world.set_body_rotation_at(1, 0.0, 0.0, 0.0, 1.0).unwrap();
    }

    /// Scale Gizmo(`set_body_scale_at`)がスポーンした球のスケールを
    /// 成功パスで受理すること。
    #[test]
    fn set_body_scale_at_succeeds_for_a_spawned_body() {
        let mut world = new_world();
        let sphere_index = world
            .spawn_sphere(0.0, 0.0, 0.0, 0.5, "コンクリート".to_string())
            .expect("known material name must succeed");
        world.set_body_scale_at(sphere_index, 2.0).unwrap();
        assert_eq!(world.body_shape_kind_at(sphere_index).unwrap(), "sphere");
    }
}
