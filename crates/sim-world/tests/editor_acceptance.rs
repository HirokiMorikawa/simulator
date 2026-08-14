//! 統合エディタの受け入れ基準を先に固定する「物差し」テスト
//! (`docs/22-roadmap/03-editor-todo.md` 冒頭・
//! `docs/reviews/2026-08-14-editor-implementation-plan.md` §2「物差し」)。
//!
//! `crates/sim-world/tests/` 配下(＝crate境界の外側でコンパイルされる統合テスト)に
//! 意図的に置いている。`src/scenario.rs` の `#[cfg(test)] mod tests` はクレート内部の
//! 単体テストなので `pub(crate)`/非公開フィールドにもアクセスできてしまい、
//! 「特権アクセスに頼っていない」ことの証明にならない。ここに置くことで、
//! **コンパイラ自身が**「`sim_world::` の `pub` API だけしか呼べない」ことを強制する
//! ——一度でも非公開項目に触れれば、この2本はビルドの時点で失敗する。
//!
//! 執筆時点の状況: この2本は「今は必ず落ちる」プレースホルダとしてではなく、
//! 直前のタスク群(`World → Scenario` 逆写像・安定ID・部品作成メソッドの実装)が
//! 既に土台を作ったため、書いた時点で両方グリーンだった。それでも「基準を
//! テスト化されていない文章のまま放置しない」という物差しタスクの目的は
//! 満たしている——以後の変更でこの基準が崩れれば、この2本が退行検知として働く。

use sim_world::{Scenario, World};

/// 基準①(`docs/23-frontend/01-editor.md` §9・`docs/20-integration/02-determinism-replay.md`
/// §5): 「Edit で編集 → Save → Load → Play で state_hash が一致する」を、
/// ジョイント(WheelJoint、操舵・駆動モータつき)を含む D24 車シーンで検証する。
///
/// 「Edit」の代替として `scenes/d24-car.json` を `World::from_scenario` で読み込み
/// (これも公開APIの一部、上記モジュールdocコメント参照)、そこから先の
/// Save→Load→Play をテストする:
/// 1. 素の world を60step走らせた state_hash を基準値とする。
/// 2. 走らせる**前**の world を `to_scenario`(Save)→JSON文字列化→`Scenario::from_json`
///    (Load)で往復させ、同じく60step走らせる。
/// 3. 両者の state_hash が一致することを確認する。
#[test]
fn d24_car_scene_survives_save_load_replay_with_matching_state_hash() {
    let scene_json = include_str!("../../../scenes/d24-car.json");
    let scenario = Scenario::from_json(scene_json).expect("d24-car.json must parse");

    let mut baseline = World::from_scenario(&scenario).expect("d24-car.json must build a World");
    for _ in 0..60 {
        baseline.step();
    }
    let baseline_hash = baseline.state_hash();

    let fresh = World::from_scenario(&scenario).expect("d24-car.json must build a World");
    let saved = sim_world::to_scenario(&fresh, "d24-car-roundtrip");
    let saved_json = serde_json::to_string(&saved).expect("saved Scenario must serialize");
    let reloaded_scenario =
        Scenario::from_json(&saved_json).expect("saved Scenario JSON must reparse");
    let mut reloaded =
        World::from_scenario(&reloaded_scenario).expect("saved Scenario must rebuild a World");
    for _ in 0..60 {
        reloaded.step();
    }

    assert_eq!(
        baseline_hash,
        reloaded.state_hash(),
        "D24 car: save→load→60step の state_hash が元の実行と一致しない"
    );
}

/// 基準②(`docs/20-integration/04-world-api.md` §5): 「API 経由のみで全デモシナリオが
/// 構築できること(特権アクセスの不在の証明)」。
///
/// `scenes/index.json` が列挙する全シーンを、`Scenario::from_json` + `World::from_scenario`
/// (設計docの「API(Rust シグネチャ)」節に載っている、`create_body`/`create_joint`等と
/// 並ぶ公開APIの一項目)だけで構築・60step実行できることを確認する。この呼び出しが
/// このファイル(crate境界の外側)から成立すること自体が、`from_scenario` の実装が
/// `World`・各ドメインcrateの非公開項目に一切依存していないことの証明になる。
#[test]
fn all_gallery_scenes_build_and_run_via_public_world_api_only() {
    let manifest_json = include_str!("../../../scenes/index.json");
    let manifest: serde_json::Value =
        serde_json::from_str(manifest_json).expect("scenes/index.json must be valid JSON");
    let scenes = manifest["scenes"]
        .as_array()
        .expect("scenes/index.json must have a top-level \"scenes\" array");
    assert!(
        !scenes.is_empty(),
        "the gallery manifest should not be empty"
    );

    for entry in scenes {
        let file = entry["file"]
            .as_str()
            .expect("each manifest entry must have a \"file\" string");
        let scene_json = std::fs::read_to_string(format!(
            "{}/../../scenes/{file}",
            env!("CARGO_MANIFEST_DIR")
        ))
        .unwrap_or_else(|e| panic!("scenes/{file} listed in the manifest must exist: {e}"));

        let scenario = Scenario::from_json(&scene_json)
            .unwrap_or_else(|e| panic!("scenes/{file} must parse as a valid Scenario: {e:?}"));
        let mut world = World::from_scenario(&scenario)
            .unwrap_or_else(|e| panic!("scenes/{file} must build a valid World: {e:?}"));
        for _ in 0..60 {
            world.step();
        }
    }
}
