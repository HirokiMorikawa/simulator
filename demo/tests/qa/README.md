# 統合エディタの QA ハーネス

エディタを**実際にマウス/キーで操作した結果**として物理量を読み出し、解析解と突き合わせる
検証スクリプト。報告書は [docs/reviews/2026-08-04-editor-qa.md](../../../docs/reviews/2026-08-04-editor-qa.md)。

## `demo/tests/smoke.spec.ts` との違い

スモークテストが守るのは「起動し、wasm が初期化され、主要な操作でクラッシュしない」という
配線の健全性だけである(そう README にも書いてある)。**物理の正しさを UI 越しに確かめる手段は
これまで無かった** — Rust 側の解析解テストは物理コアを直接叩くので、
「エディタから操作したときにその値が出るか」は誰も見ていなかった。ここがその隙間を埋める。

判定に Rust のテスト関数は一切使わない。シーンはツールバーの `Scene` 選択から読み込み、
時間はモードトグルと `⏭` で進め、値は Probe 履歴・HUD・ハッシュ表示から取る。
**UI から到達できる値だけを使う**のがこの QA の存在意義である。

Playwright のテストランナーには載せていない(拡張子が `.mjs` なので `playwright test` は収集しない)。
判定が「解析解との誤差」であり、描画 fps に依存する step 進行など CI のノイズ床と相性が悪いため、
実行は手動で行う。

## 実行

```bash
# 1. wasm と依存を用意する
wasm-pack build crates/sim-wasm --target web --out-dir ../../demo/pkg
cd demo && npm ci

# 2. 開発サーバを立てる(既定で 127.0.0.1:5199 を見に行く)
npx vite --port 5199 --strictPort --host 127.0.0.1 &

# 3. 検証を回す(demo/tests/qa から実行する — node_modules の解決に必要)
cd tests/qa
node qa-physics.mjs        # 物理法則:     19 項目
node qa-operability.mjs    # 操作性:       28 項目
node qa-coupling.mjs       # 法則の組合せ: 37 項目
node qa-defects.mjs        # 既知の不具合の再現
```

`wasm-pack` が無い環境では、同じ成果物を次の 2 コマンドで作れる。

```bash
cargo build --release --target wasm32-unknown-unknown -p sim-wasm
wasm-bindgen --target web --out-dir demo/pkg target/wasm32-unknown-unknown/release/sim_wasm.wasm
```

環境変数で上書きできる。

| 変数 | 既定 | 用途 |
|---|---|---|
| `QA_URL` | `http://127.0.0.1:5199/` | 検証対象の URL |
| `QA_OUT` | `/tmp/simulator-qa` | スクリーンショットの出力先 |
| `PLAYWRIGHT_CHROMIUM_PATH` | `/opt/pw-browsers/chromium` | Chromium の実行ファイル |

## 各スクリプト

| ファイル | 内容 |
|---|---|
| `qa-lib.mjs` | 共通部。起動待ち・シーン読み込み・`⏭` 送り・座標投影 |
| `qa-physics.mjs` | D1/D3/D5/D11/D30/D34 の解析解照合、決定論、Settings の重力反映 |
| `qa-operability.mjs` | カメラ・W/E/R/Q・Gizmo 3種・Undo/Redo・Edit/Play・Timeline・Console・ピック |
| `qa-coupling.mjs` | **ドメイン間結合**(D10/D14/D15/D17/D18b/D19/D20/D21/D23/D25/D26)の橋の両側の照合と、UI からドメインを足したとき既存の結合が拾うか([報告書](../../../docs/reviews/2026-08-04-coupling-qa.md)) |
| `qa-defects.mjs` | 2026-08-04 に見つかった不具合 9 件の再現 |

`qa-coupling.mjs` は不具合 8 件が未修正のあいだ 29/37 PASS で終わる(`qa-defects.mjs` と同じく
FAIL は「異常」ではなく「未修正」と読む)。

`qa-defects.mjs` は**不具合が残っているあいだ FAIL する**。修正が入れば PASS に転じるので、
そのまま回帰確認に使える。他の 2 本と違い、FAIL は「異常」ではなく「未修正」と読む。

## 実装上の落とし穴(踏んだもの)

- **Play モードへ入ると即座に `playing = true` になる**。モード切替だけでは自由走行が始まり、
  `⏭` は効かず(ハンドラが `!playing` を要求する)、待ち時間ぶんの step が測定に混入する。
  `enterPlayPaused()` は 2 つの click を同じ tick 内で呼んで、1 step も進めずに停止させる。
- **Probe 履歴は容量 600 のリングバッファ**(`DEFAULT_PROBE_CAPACITY`、
  crates/sim-world/src/scenario.rs:35)。600 step を超えると先頭が失われ、
  添字と時刻の対応が崩れる。D3 の「第1バウンド」を読むつもりで数回あとの極大を読んでいた、
  という取り違えを実際にやった。D34 の周期を掃過角から出しているのも同じ理由である。
- **ギズモのスケールハンドルは1個だけ**で、位置は対角の `(1.6, 1.6, 1.6)`(等方スケール)。
  軸方向を掴もうとしても当たらない。
- Chromium はソフトウェア GL で 13 fps 程度しか出ない。step 数に依存する判定は
  `⏭` の N step 送り(同期実行)で行い、経過時間には依存させない。
