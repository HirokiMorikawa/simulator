//! wasm-bindgen バインディング。設計: docs/00-foundation/05-rust-wasm-platform.md §3。
//!
//! Phase 0 は同文書のシグネチャ例(`WasmWorld::from_scene_json/step/time/
//! body_transforms_f32/observables_json/state_hash/push_command_json`)を
//! 「箱1個が落ちる」規模に縮小したものを公開する。シーンJSON・コマンドキュー・
//! 観測値JSONはシーン記述(docs/20-integration/04-world-api.md §3)が実装され次第、
//! Phase A 以降で追加する。

use js_sys::Float32Array;
use sim_mechanics::{RigidBodyDesc, Shape};
use sim_world::{BodyId, World, WorldOptions};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct WasmWorld {
    inner: World,
    box_body: BodyId,
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
        WasmWorld { inner, box_body }
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

    /// 決定論検証・UI 表示用の状態ハッシュ(16進文字列)。
    pub fn state_hash(&self) -> String {
        format!("{:016x}", self.inner.state_hash())
    }
}
