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
use sim_mechanics::{BodyType, RigidBodyDesc, Shape};
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

/// スポーンパレット(設計docs/23-frontend/01-editor.md §6「形状×材質を選んで
/// クリック配置」)で追加したボディの記録。`sim-wasm`のWorld API-only制約
/// (`World`自体はShape/Materialのクエリを持たない)を、スポーンした側(この
/// 構造体)が構築時の値をそのまま覚えておくことで回避する——固定2体(床・箱)の
/// `BODY_META`ハードコードと同じ発想を、動的に増えるボディにも一般化したもの。
struct SpawnedBodyMeta {
    id: BodyId,
    label: String,
    shape_label: String,
    material_label: String,
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
        let snapshot_interval_steps = (1.0 / dt).round().max(1.0) as u64;
        WasmWorld {
            inner,
            ground_body,
            box_body,
            spawned: Vec::new(),
            y_probe,
            speed_probe,
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
    /// スポーンパレットで追加するボディは常にDynamic。
    pub fn body_is_static_at(&self, index: usize) -> bool {
        index == 0
    }

    /// Inspector表示用のShape文字列(World API-only制約により、スポーン時の
    /// 値をそのまま返す縮約実装、モジュールdoc「`SpawnedBodyMeta`」参照)。
    pub fn body_shape_label_at(&self, index: usize) -> String {
        match index {
            0 => "Plane(normal=(0,1,0), d=0)".to_string(),
            1 => "Box(0.5,0.5,0.5)".to_string(),
            _ => self
                .spawned
                .get(index - 2)
                .unwrap_or_else(|| panic!("body index {index} out of range"))
                .shape_label
                .clone(),
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
            shape_label: format!("Sphere({radius})"),
            material_label: material_name,
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
            shape_label: format!("Box({half_extent},{half_extent},{half_extent})"),
            material_label: material_name,
        });
        index
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
