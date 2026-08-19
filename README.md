# simulator

現実の物理法則を Rust で実装し、ブラウザ上で動かして遊べる物理シミュレータ。

[![CI](https://github.com/HirokiMorikawa/simulator/actions/workflows/ci.yml/badge.svg)](https://github.com/HirokiMorikawa/simulator/actions/workflows/ci.yml)
[![Rust](https://img.shields.io/badge/rust-stable-orange)](https://www.rust-lang.org/)

## 目次

- [概要](#概要)
- [特徴](#特徴)
- [必要環境](#必要環境)
- [インストール](#インストール)
- [使い方](#使い方)
- [ドキュメント](#ドキュメント)
- [開発](#開発)
- [コントリビュート](#コントリビュート)
- [ライセンス](#ライセンス)

## 概要

重力落下、流体、熱、電磁気、量子力学、天体の軌道運動など、幅広い物理現象を Rust で実装したシミュレーションエンジン。物理計算部分は WebAssembly にコンパイルされ、ブラウザ上の統合エディタからオブジェクトを配置・編集し、シミュレーションを再生しながら結果を確認できる。

教科書通りの解析解と結果を突き合わせるテストを備えており、「見た目それっぽい」ではなく実際に正しい物理計算になっているかを検証できることを重視している。

## 特徴

- **多様な物理現象を1つの世界でシミュレート** — 剛体の衝突・流体・熱伝導・電磁気回路・量子力学・天体軌道などを組み合わせて1つのシーンに配置できる。
- **正しさの検証** — 数百件の自動テストが、解析的に解ける物理問題の答えと計算結果を比較して合否を判定する。
- **再現性のある計算** — 同じ入力からは常に同じ結果が得られる決定論的なシミュレーション。リプレイや共有、回帰テストがしやすい。
- **ブラウザ上の統合エディタ** — オブジェクトの配置・編集、再生/一時停止、値のグラフ表示などをブラウザ上で行える。
- **物理ベースレンダリング対応** — 光の伝搬を物理的に計算するパストレーサーを同梱しており、静止画のレンダリングにも使える。
- **外部依存が少ない** — 数値計算まわりの主要な処理を自前実装しており、依存ライブラリを最小限に抑えている。

## 必要環境

事前に用意するのは次の2つだけ。残りの依存(`wasm32-unknown-unknown` ターゲット、wasm-pack、npm パッケージ)はセットアップコマンドが自動で揃える。

| 要件 | 補足 |
|---|---|
| [Rust](https://www.rust-lang.org/tools/install) | rustup 経由での導入を推奨。使用するバージョンとターゲットは `rust-toolchain.toml` が宣言しており自動で適用される |
| [Node.js](https://nodejs.org/) 22 以上 | ブラウザデモのビルドと起動に必要。nvm 利用時はリポジトリ直下で `nvm use` |

### 対応プラットフォーム

Linux / macOS / Windows で動作する。物理エンジン本体は OS 依存のコードを含まない。

| OS | 状態 |
|---|---|
| Linux | CI で検証(ビルド・テスト・ブラウザE2E) |
| macOS | CI で検証(セットアップ・ビルド・テスト) |
| Windows | CI で検証(セットアップ・ビルド・テスト) |

## インストール

```bash
git clone https://github.com/HirokiMorikawa/simulator.git
cd simulator
cargo xtask setup
```

`cargo xtask setup` が、wasm-pack の導入 → 物理エンジンの WebAssembly ビルド → デモの依存取得までをまとめて行う。3つのOSで同じコマンドが使える(初回は wasm-pack のビルドに数分かかる)。

完了したらデモを起動する。

```bash
cargo xtask dev
```

Vite が表示する URL をブラウザで開くとエディタが立ち上がる。

<details>
<summary>用意されているコマンド</summary>

| コマンド | 内容 |
|---|---|
| `cargo xtask setup` | セットアップ一式(wasm-pack導入 → wasmビルド → npm ci) |
| `cargo xtask build-wasm` | 物理エンジンを WebAssembly へビルドし `demo/pkg` に出力 |
| `cargo xtask dev` | ブラウザデモを起動(未セットアップなら不足分を自動で補う) |
| `cargo xtask check` | CI と同じチェック(fmt / clippy / test) |

</details>

<details>
<summary>手動でセットアップする場合</summary>

`cargo xtask setup` は下記と同等のことを行っている。個別に実行することもできる。

```bash
# 1. wasm-pack を導入する(バージョンは crates/xtask/src/main.rs で固定)
cargo install wasm-pack --version 0.13.1 --locked

# 2. WebAssembly をビルドする
#    demo/src/main.ts が demo/pkg を直接 import するため、デモのビルド・起動より
#    先に必ず通す必要がある
wasm-pack build crates/sim-wasm --target web --out-dir ../../demo/pkg

# 3. デモの依存を取得して起動する
cd demo
npm ci
npm run dev
```

`wasm32-unknown-unknown` ターゲットは `rust-toolchain.toml` により rustup が自動で導入するため、`rustup target add` は不要。

</details>

## 使い方

エディタでは、あらかじめ用意された検証用シーン([scenes/](scenes/))を選んで再生したり、オブジェクトを自分で配置してシミュレーションを試すことができる。シーンは JSON ファイルとして保存・読み込みでき、物理パラメータ(重力・材料・初期位置など)を直接編集することも可能。

Rust から直接エンジンを呼び出すこともできる。

```rust
use sim_world::{run_headless_scenario, World, WorldOptions};

// シーン JSON をヘッドレスで実行し、結果を取得する
let json = std::fs::read_to_string("scenes/d1-free-fall.json")?;
let result = run_headless_scenario(&json, 600)?;

// あるいは World を直接組み立てて使う
let mut world = World::new(WorldOptions::default());
world.step();
```

## ドキュメント

より詳しい設計や各物理領域の実装方針は [docs/](docs/) にまとめている。入口は [docs/README.md](docs/README.md)。

## 開発

```bash
# フォーマット・静的解析・テストをまとめて実行する
cargo xtask check

# デモアプリのビルド・E2Eテスト
cd demo
npm run build
npm run test:e2e
```

[.github/workflows/ci.yml](.github/workflows/ci.yml) が push・PR ごとに同じチェックを自動実行する。CI は Linux でビルド・テスト・ブラウザE2Eを、macOS と Windows で `cargo xtask setup` を起点としたセットアップとテストを検証する。

## コントリビュート

Issue / Pull Request を歓迎する。PR を送る際は CI が通ることを確認し、物理計算に関する変更には対応するテストを添えてほしい。

## ライセンス

本リポジトリには現時点でライセンスファイルが含まれていない。ライセンスを追加する場合は `LICENSE` ファイルをリポジトリ直下に置き、本セクションを更新すること。
