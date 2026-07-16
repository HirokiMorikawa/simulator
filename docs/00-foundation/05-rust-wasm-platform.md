# 05. 実装プラットフォーム — Rust + WebAssembly

実装フェーズが従うプラットフォーム設計。コアは純 Rust、配布はブラウザ(WASM)、検証はネイティブ。

## 1. なぜ Rust + WASM か(決定済み)

- **性能**: 流体格子・SPH 粒子・気体分子デモは 10⁴〜10⁶ 要素規模になる。GC のない Rust は
  これらの安定したフレームタイムに適する。WASM は f64 をネイティブサポートし、SIMD(wasm simd128)も使える。
- **安全性と設計強制**: 所有権モデルがレイヤ依存規則([04-architecture.md](04-architecture.md) §2)の実装を助ける。
  ドメイン間の隠れた参照はコンパイルエラーになる。
- **二重ターゲット**: 同一コードをネイティブ(`cargo test`、高速な検証・ベンチ)と WASM(ブラウザデモ)でビルドできる。
  数値検証はネイティブで回し、遊びはブラウザで提供する。

## 2. ワークスペース構成

```
Cargo.toml                # [workspace]
crates/
  sim-math/               # 線形代数・場・積分器・PRNG (依存: なし。no_std 可能な範囲で)
  sim-core/               # ID・イベント・時間・MaterialDb・Solverトレイト (依存: sim-math)
  sim-mechanics/          # 剛体・衝突・拘束 (依存: sim-core)
  sim-fluid/              # 格子流体・SPH (依存: sim-core)
  sim-thermal/            # 温度場・熱伝達・相変化 (依存: sim-core)
  sim-em/                 # 回路・静場・FDTD・光学 (依存: sim-core)
  sim-quantum/            # シュレディンガーソルバ・有効モデル (依存: sim-core)
  sim-statistical/        # 分子動力学デモ・ランジュバン・MC (依存: sim-core)
  sim-coupling/           # 全 Coupling 実装 (依存: 各ドメイン crate)
  sim-world/              # World facade・Orchestrator・シナリオ (依存: sim-coupling)
  sim-wasm/               # wasm-bindgen バインディング (依存: sim-world のみ)
demo/                     # Vite + TypeScript + Three.js (sim-wasm の pkg を import)
```

- 依存方向はアーキテクチャ文書のレイヤ規則をそのまま crate 境界で強制する。
- `sim-wasm` 以外のすべての crate は `wasm-bindgen` に依存しない(ネイティブテストを純粋に保つ)。
- ドメイン crate を分けるのは、コンパイル時間の局所化と「ドメイン間非依存」の機械的保証のため。

## 3. WASM 境界の設計

原則: **境界呼び出しは粗く・少なく、データはゼロコピーで**。

- World はハンドル 1 個(`WasmWorld`)として公開。生成・シーンロード・step・クエリのみを export する。
- **状態読み出しはメモリビュー**: 剛体の位置・回転、粒子位置、格子場などは SoA バッファ
  ([04-architecture.md](04-architecture.md) §3)の `Float64Array`/`Float32Array` ビューを返す。
  JS 側はコピーせず直接 Three.js のバッファへ書く。ビューは step でメモリが動くと無効になるため、
  「step 後に毎回取り直す」規約を demo 側に置く(wasm-bindgen の view API の標準的な扱い)。

```rust
#[wasm_bindgen]
pub struct WasmWorld { inner: sim_world::World }

#[wasm_bindgen]
impl WasmWorld {
    pub fn from_scene_json(json: &str) -> Result<WasmWorld, JsError>;
    pub fn step(&mut self);                       // 1 world step (1/120 s)
    pub fn time(&self) -> f64;
    /// 剛体状態 [x,y,z, qx,qy,qz,qw] × N のビュー (描画用)
    pub fn body_transforms_f32(&self) -> js_sys::Float32Array;
    /// 観測値 (エネルギー・温度など) を JSON で
    pub fn observables_json(&self) -> String;
    pub fn state_hash(&self) -> String;
    /// 対話操作はコマンドとして積む (次stepの先頭で適用)
    pub fn push_command_json(&mut self, json: &str);
}
```

- コマンド・観測は当面 JSON 文字列(呼び出し頻度が低いので十分)。毎フレームの高頻度データのみバイナリビュー。
- 転送用 f32 変換バッファはコア側に持つ(描画精度は f32 で十分、物理は f64 のまま)。

## 4. 並列化方針

- **Phase 1〜2 はシングルスレッド**で正しさと決定論を確立する。
- その後、ドメイン内データ並列(流体格子のスライス分割、SPH の近傍バケット並列、broadphase)を
  `rayon`(ネイティブ)/ `wasm-bindgen-rayon`(ブラウザ、SharedArrayBuffer + COOP/COEP 必要)で導入。
- **決定論の制約**: 並列リダクションは加算順序を固定する(チャンク順逐次結合)。
  「並列でも同一シード→同一結果」を CI で検証してから既定有効にする。
- GPU(WebGPU)は将来オプション(大規模流体)。設計上は `FluidSolver` トレイトの別実装として隔離する。

## 5. 性能予算(60 fps インタラクティブ時)

フレーム 16.6 ms のうち物理に 10 ms を配分。ブラウザ・中位ノート PC(参考: 4 コア、WASM シングルスレッド)想定。

| ワークロード | 目標規模 | 予算 |
|---|---|---|
| 剛体(接触多数のスタック) | 500 体 | 3 ms |
| 流体格子(煙・水) | 64³ | 4 ms |
| SPH 粒子 | 2×10⁴ | 4 ms |
| 気体分子デモ | 10⁴ 粒子 | 2 ms |
| FDTD(専用シーン) | 128² (2D) | 5 ms |
| 熱・回路・結合・その他 | — | 1 ms |

- 予算超過時の方針: 精度を落とすのではなく**規模かリアルタイム性を落とす**(スロー再生・オフライン計算)。
  「検証して遊ぶ」ツールとして、静かに精度を落とすことを禁じる([01-vision.md](01-vision.md) §5)。
- ベンチは `criterion`(ネイティブ)で代表シーンを常設し、回帰を CI で検知する。

## 6. ビルド・ツールチェーン

- Rust stable、`wasm32-unknown-unknown` ターゲット、`wasm-pack`(または `wasm-bindgen-cli` + 手動 glue)。
- demo: Vite + TypeScript + Three.js。`vite-plugin-wasm` で pkg を import。
- CI: `cargo fmt --check` / `cargo clippy -D warnings` / `cargo test`(全ドメインの解析解テスト)/
  wasm ビルド / demo ビルド。決定論テスト(同一シード 2 回実行のハッシュ一致)を必須ゲートにする。
- コアで禁止するもの(clippy/lint + レビューで強制): `std::time`・`rand::thread_rng`・`HashMap` の反復順序依存
  (決定論のため。反復するマップは `BTreeMap` か挿入順 `Vec`)。

## 7. デモ層(参考、実装フェーズで詳細化)

- 各デモシナリオ([21-verification/03-demo-scenarios.md](../21-verification/03-demo-scenarios.md))= シーン JSON + UI 定義。
- 共通 UI: 再生/一時停止/1 step/スロー、パラメータスライダー、シード入力、グラフ(観測値の時系列)、
  状態ハッシュ表示、「このシーンの近似」表示([02-scale-ladder.md](02-scale-ladder.md) §5)。
- 描画: Three.js(剛体・粒子・場の可視化)。光学ドメインの結果(屈折・分光)は専用ビジュアライザ。
