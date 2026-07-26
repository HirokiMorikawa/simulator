//! wasm-bindgen バインディング。設計: docs/00-foundation/05-rust-wasm-platform.md §3。
//!
//! Phase 0 は同文書のシグネチャ例(`WasmWorld::from_scene_json/step/time/
//! body_transforms_f32/observables_json/state_hash/push_command_json`)を
//! 「箱1個が落ちる」規模に縮小したものを公開する。シーンJSON・コマンドキュー・
//! 観測値JSONはシーン記述(docs/20-integration/04-world-api.md §3)が実装され次第、
//! Phase A 以降で追加する。

use js_sys::{Float32Array, Float64Array};
use sim_mechanics::{RigidBodyDesc, Shape};
use sim_world::{BodyId, Command, ProbeTarget, World, WorldOptions};
use wasm_bindgen::prelude::*;

/// `docs/23-frontend/01-editor.md`のProbe Graphsパネル(§1.4「複数系列」)デモ用に、
/// 箱のy座標を毎step記録するプローブの履歴長。1step=dt秒、`PROBE_HISTORY_CAPACITY`
/// step分(≈`PROBE_HISTORY_CAPACITY*dt`秒)のスクロールウィンドウになる。
const PROBE_HISTORY_CAPACITY: usize = 600;

#[wasm_bindgen]
pub struct WasmWorld {
    inner: World,
    box_body: BodyId,
    y_probe: usize,
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
        WasmWorld {
            inner,
            box_body,
            y_probe,
        }
    }

    /// 1 world step。
    pub fn step(&mut self) {
        self.inner.step();
    }

    pub fn time(&self) -> f64 {
        self.inner.time()
    }

    pub fn step_count(&self) -> u64 {
        self.inner.step_count()
    }

    /// 剛体位置 [x, y, z] のビュー(描画用、f32)。
    /// 05-rust-wasm-platform.md §3 の `body_transforms_f32` の Phase 0 縮小版
    /// (回転は未実装のため位置のみ)。
    pub fn body_position_f32(&self) -> Float32Array {
        let p = self
            .inner
            .body_position(self.box_body)
            .expect("box_body is created in new() and never removed");
        let out = Float32Array::new_with_length(3);
        out.set_index(0, p.x as f32);
        out.set_index(1, p.y as f32);
        out.set_index(2, p.z as f32);
        out
    }

    /// 剛体速度 [vx, vy, vz] のビュー(エディタInspectorのTransform/RigidBody
    /// 表示用、f32)。既存の`World::body_velocity`をそのまま公開する。
    pub fn body_velocity_f32(&self) -> Float32Array {
        let v = self
            .inner
            .body_velocity(self.box_body)
            .expect("box_body is created in new() and never removed");
        let out = Float32Array::new_with_length(3);
        out.set_index(0, v.x as f32);
        out.set_index(1, v.y as f32);
        out.set_index(2, v.z as f32);
        out
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
}
