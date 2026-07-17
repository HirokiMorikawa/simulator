# 設計書 目次 — 現実世界の写像を目指す物理エンジン

本ディレクトリは、地球の物理法則を忠実に再現するデジタル空間シミュレータの物理エンジン設計書一式である。
成果物はまず設計のみであり、実装(Rust + WebAssembly)は本設計のレビュー承認後に
[22-roadmap/01-phases.md](22-roadmap/01-phases.md) のフェーズ計画に従って行う。

## 読み順ガイド

はじめて読む場合は次の順を推奨する。

1. **基盤** — [00-foundation/](00-foundation/) : 何を作るのか、なぜこの構造なのか
2. **数学・数値基盤** — [01-math/](01-math/) : 全ドメイン共通の道具
3. **物理ドメイン** — 10〜15 番台 : 興味のあるドメインから読んでよい(各文書は統一フォーマットで自己完結)
4. **横断設計** — [20-integration/](20-integration/) : ドメインをどう束ねるか
5. **検証・ロードマップ** — [21-verification/](21-verification/), [22-roadmap/](22-roadmap/)

## 全文書一覧

### 00-foundation — 基盤
| 文書 | 内容 |
|---|---|
| [01-vision.md](00-foundation/01-vision.md) | 目的、「現実の写像」の定義、検証して遊ぶ体験像 |
| [02-scale-ladder.md](00-foundation/02-scale-ladder.md) | スケールの階梯と有効理論。本エンジンの根本思想 |
| [03-units-conventions.md](00-foundation/03-units-conventions.md) | SI 単位系、座標系、記号・命名規約 |
| [04-architecture.md](00-foundation/04-architecture.md) | Solver / Coupling / Orchestrator、レイヤ依存、ステップパイプライン |
| [05-rust-wasm-platform.md](00-foundation/05-rust-wasm-platform.md) | crate 構成、WASM 境界、性能予算、並列化方針 |
| [06-performance-strategy.md](00-foundation/06-performance-strategy.md) | 性能 3 本柱(アルゴリズム上位化・SIMD/並列化・データ局所性)+ GPU(WebGPU, CPU 優先) |

### 01-math — 数学・数値基盤
| 文書 | 内容 |
|---|---|
| [01-linear-algebra.md](01-math/01-linear-algebra.md) | Vec3 / Quat / Mat3 / テンソル: 型定義・演算・規約 |
| [02-fields.md](01-math/02-fields.md) | 場の表現: 格子(MAC / セル中心)・粒子・補間 |
| [03-integrators.md](01-math/03-integrators.md) | 数値積分カタログと安定性解析 |
| [04-random.md](01-math/04-random.md) | 決定論的 PRNG、分布サンプリング |

### 10-mechanics — 力学
| 文書 | 内容 |
|---|---|
| [01-rigid-body.md](10-mechanics/01-rigid-body.md) | 剛体状態・慣性テンソル・力/トルク API |
| [02-collision-detection.md](10-mechanics/02-collision-detection.md) | broadphase / narrowphase、SAT、GJK/EPA |
| [03-contact-solver.md](10-mechanics/03-contact-solver.md) | sequential impulses 完全導出 |
| [04-friction.md](10-mechanics/04-friction.md) | クーロン摩擦、摩擦円錐、素材別係数表 |
| [05-joints-constraints.md](10-mechanics/05-joints-constraints.md) | 拘束の抽象、ジョイント各種、ヤコビアン導出 |
| [06-soft-body-particles.md](10-mechanics/06-soft-body-particles.md) | 粒子系・XPBD・布・ロープ |

### 11-fluid — 流体力学
| 文書 | 内容 |
|---|---|
| [01-continuum-basics.md](11-fluid/01-continuum-basics.md) | 連続体仮定、Navier-Stokes 導出、無次元数 |
| [02-eulerian-grid.md](11-fluid/02-eulerian-grid.md) | MAC 格子、semi-Lagrangian 移流、投影法 |
| [03-sph.md](11-fluid/03-sph.md) | SPH: カーネル・離散式・近傍探索 |
| [04-free-surface-buoyancy.md](11-fluid/04-free-surface-buoyancy.md) | 自由表面、アルキメデス浮力、表面張力 |
| [05-aero-hydrodynamics.md](11-fluid/05-aero-hydrodynamics.md) | 抗力・揚力、終端速度、流体⇔剛体結合 |

### 12-thermal — 熱力学
| 文書 | 内容 |
|---|---|
| [01-thermodynamics-laws.md](12-thermal/01-thermodynamics-laws.md) | 熱力学法則、状態方程式、エネルギー収支設計 |
| [02-heat-transfer.md](12-thermal/02-heat-transfer.md) | 伝導・対流・放射の離散化 |
| [03-phase-change.md](12-thermal/03-phase-change.md) | 相変化、潜熱、エンタルピー法 |
| [04-material-thermal-props.md](12-thermal/04-material-thermal-props.md) | 素材別熱物性表、温度依存物性 |

### 13-electromagnetism — 電磁気学
| 文書 | 内容 |
|---|---|
| [01-electrostatics-magnetostatics.md](13-electromagnetism/01-electrostatics-magnetostatics.md) | 静電場・静磁場、帯電・放電 |
| [02-circuits.md](13-electromagnetism/02-circuits.md) | 集中定数回路(MNA 法)、電池・モーターモデル |
| [03-maxwell-fdtd.md](13-electromagnetism/03-maxwell-fdtd.md) | マクスウェル方程式、FDTD(Yee 格子) |
| [04-light-optics.md](13-electromagnetism/04-light-optics.md) | 幾何光学、反射・屈折、放射 |
| [05-em-mechanics-coupling.md](13-electromagnetism/05-em-mechanics-coupling.md) | ローレンツ力、電磁誘導、モーター結合例 |

### 14-quantum — 量子力学
| 文書 | 内容 |
|---|---|
| [01-role-and-limits.md](14-quantum/01-role-and-limits.md) | エンジンにおける役割と、マクロ直接計算が不可能な理由 |
| [02-schrodinger-solver.md](14-quantum/02-schrodinger-solver.md) | 時間依存シュレディンガー方程式の数値解法とデモ設計 |
| [03-effective-models.md](14-quantum/03-effective-models.md) | 物性・化学・半導体・発光への橋渡し |

### 15-statistical — 統計力学
| 文書 | 内容 |
|---|---|
| [01-micro-macro-bridge.md](15-statistical/01-micro-macro-bridge.md) | アンサンブル、粗視化の方法論 |
| [02-kinetic-gas.md](15-statistical/02-kinetic-gas.md) | 気体分子運動論、マクスウェル=ボルツマン分布 |
| [03-diffusion-brownian.md](15-statistical/03-diffusion-brownian.md) | 拡散、ブラウン運動、揺動散逸定理 |
| [04-monte-carlo.md](15-statistical/04-monte-carlo.md) | メトロポリス法、イジング模型 |

### 16-astro — 天体力学
| 文書 | 内容 |
|---|---|
| [01-gravitation-nbody.md](16-astro/01-gravitation-nbody.md) | N 体重力、Barnes-Hut、シンプレクティック積分 |
| [02-orbital-mechanics.md](16-astro/02-orbital-mechanics.md) | 軌道要素・摂動・大気圏再突入・宇宙機推進 |
| [03-relativistic-corrections.md](16-astro/03-relativistic-corrections.md) | オプトイン 1PN 補正(近日点移動・GPS・光偏向) |

### 17-rendering — レンダリング(Phase D)
| 文書 | 内容 |
|---|---|
| [01-rendering-architecture.md](17-rendering/01-rendering-architecture.md) | 物理から分離した 2 経路描画(プレビュー / パストレ) |
| [02-path-tracing.md](17-rendering/02-path-tracing.md) | 物理正確スペクトル・パストレーシング |
| [03-materials-camera.md](17-rendering/03-materials-camera.md) | 光学物性 BSDF・物理カメラ・トーンマッピング |

### 20-integration — 横断設計
| 文書 | 内容 |
|---|---|
| [01-coupling-matrix.md](20-integration/01-coupling-matrix.md) | ドメイン間結合行列、operator splitting、sub-stepping |
| [02-determinism-replay.md](20-integration/02-determinism-replay.md) | 決定論、シード乱数、状態ハッシュ、リプレイ |
| [03-entity-layer.md](20-integration/03-entity-layer.md) | 物・人・生物・乗り物のエンティティ設計 |
| [04-world-api.md](20-integration/04-world-api.md) | World 公開 API、シーン記述形式 |

### 21-verification — 検証
| 文書 | 内容 |
|---|---|
| [01-analytic-tests.md](21-verification/01-analytic-tests.md) | 全ドメインの解析解テスト表・許容誤差・収束次数 |
| [02-conservation-laws.md](21-verification/02-conservation-laws.md) | 保存則チェック設計 |
| [03-demo-scenarios.md](21-verification/03-demo-scenarios.md) | 「検証して遊ぶ」デモシナリオ集 |

### 22-roadmap — ロードマップ
| 文書 | 内容 |
|---|---|
| [01-phases.md](22-roadmap/01-phases.md) | 実装フェーズ分割、完了条件、デモ対応表 |
| [02-feature-checklist.md](22-roadmap/02-feature-checklist.md) | 機能群一覧・実装チェック表(AI の中断・再開用の進行記録) |

## 文書規約

- 言語は日本語。専門用語は初出時に英語を併記する(例: 有効理論 effective theory)。
- 数式は GitHub Markdown の LaTeX 記法(`$...$` / `$$...$$`)で書く。
- コードスケッチは Rust。設計段階のシグネチャ・型定義であり、コンパイル可能性より意図の明確さを優先する。
- 物理ドメイン文書(10〜15 番台)は統一 9 節フォーマットに従う:
  1. 担う現実の現象(遊び方の例)
  2. 支配方程式(導出込み)
  3. 状態表現・Rust 型定義
  4. 数値解法(離散化の具体式・安定条件・計算量)
  5. 適用スケールと限界(何を近似するか)
  6. 他ドメインとの結合(入出力)
  7. 検証(解析解・保存則)
  8. 実装フェーズ対応
  9. パラメータ表(数値・出典)+ 擬似コード
- 数値パラメータには必ず出典(教科書・ハンドブック・標準値)を付す。
- 近似・省略の判断には必ず物理的根拠を書く。
