# 機能群一覧・実装チェック表 — 中断・再開のための進行記録

目的: 本プロジェクトは AI 主導の一気通貫開発で進めるため、トークン制限・セッション中断で
作業が途切れた際に**再開地点を機械的に特定できる**必要がある。本表はプロジェクト全体の
機能群を単一のチェック表として列挙し、進行状態の**唯一の記録**とする。

## 運用ルール

1. 項目が完了するたび `[ ]` を `[x]` にし、**その作業と同じコミットに含める**
   (表の更新が遅れると再開地点がずれる)。
2. [現在地](#現在地) 節は作業の開始時・中断時に更新する。「作業中」は常に 1 項目だけにする。
3. **再開手順**: (1) 現在地を読む → (2) 作業中項目の実状態をコード・テスト実行で確認する
   (チェック表は自己申告であり、実状態が正) → (3) 未チェックの最初の項目から続行する。
4. 項目の増減は設計書改訂と同じ扱いとする(実装が本表と乖離したら本表を先に直す)。
5. 各項目の内容定義は括弧内の参照文書が正。本表は索引であり仕様を再定義しない。

## 現在地

- **フェーズ**: 実装(Phase 0 完了。math ウェーブは線形代数・PRNG・積分器カタログの状態非依存部分が Green、場 Grid3/MacGrid が残作業)
- **作業中**: math ウェーブ残り(場 Grid3/MacGrid、下記 §3)
- **次**: 場を実装して math ウェーブを完了させたら、Phase A(全ドメインの型・トレイトのスケルトン + 全テスト記述、下記 §2)に着手。
  線形代数・PRNG・積分器カタログの汎用部分(`sim-math` の `Vec3`/`Quat`/`Mat3`/`Transform`/`SimRng`/
  `explicit_euler_step`/`semi_implicit_euler_step`/`velocity_verlet_step`/`rk4_step`(`BallisticIntegrator`)/
  `BorisPusher`)は依存が無く低リスクなため、Phase A の Red 段階を経ずに直接実装 + テストで Green 化した
  (§3 の当該行のみ先行完了)。ただし `RigidIntegrator` トレイト(P1、`RigidBodySet` に依存)・
  陰的 Euler(場が必要)・leapfrog(Yee)・split-step Fourier・XPBD・semi-Lagrangian・BAOAB は
  状態型を持つ各ドメイン crate が P1–P5 で実装する(sim-math には汎用プリミティブのみ置く)。
  他ドメインは設計通り Phase A(型スケルトン + 全テスト記述・Red確認)から進める。

## 0. 設計フェーズ残作業

決定事項:

- [x] C-1 決定論水準の決定(案 1 緩和を採用 — [20-integration/02](../20-integration/02-determinism-replay.md) §5)
- [x] C-3 位置表現(保持フレーム)の決定(フレーム ID + ローカル座標 f64 を採用 — [../00-foundation/02-scale-ladder.md](../00-foundation/02-scale-ladder.md) §2.2)
- [x] C-5 最小 CCD の方式選定(speculative contact を採用 — [../10-mechanics/02-collision-detection.md](../10-mechanics/02-collision-detection.md) §4.6)

改訂 PR:

- [x] PR-1 対応不要判断(A-1/B-1/B-3/D)の反映(vision §4、phases.md 開発体制の前提)
- [x] PR-2 テスト表の実装可能性監査・長時間級ルール・Boris pusher・Wolff 必須化・F11 注記
- [x] PR-3 決定論方針の反映・台帳再定義・sub-iteration 規則 + stiff 検出テスト行
- [x] PR-4 位置表現の決定反映・最小 CCD 標準機能化 + 検証テスト行・立位保持基準の置換
- [x] PR-5 性能構成規則・wasm 配布戦略・巻き戻しコスト
- [x] PR-6 新設文書: UI/フロントエンド設計・フレーム階層詳細設計・レジーム切替プロトコル
- [x] PR-7 実装の難所の詳細化(全ドメイン文書横断 — 難所一覧は [../00-foundation/01-vision.md](../00-foundation/01-vision.md) §4.1)
- [x] 実装開始ゲート通過(vision §4: レビュー承認。ユーザー指示により2026-07-19承認)

## 1. Phase 0 — 骨格

- [x] Cargo ワークスペース(05-rust-wasm-platform §2、sim-astro/sim-render 含む)
- [x] CI 最小構成(fmt / clippy / test / wasm ビルド / 決定論スモーク)
- [x] demo の Vite + Three.js 雛形
- [x] wasm 境界の疎通
- [x] 最小 World: 箱 1 個が落ちて cargo test 緑 + ブラウザ表示 + ハッシュ 2 回一致

## 2. Phase A — テスト先行(Red)

型・トレイトのスケルトン(中身 `todo!()`、コンパイル可):

- [ ] math(Vec3/Quat/Mat3・場・`Integrator`・SimRng)
- [ ] 力学(剛体状態・`Solver`/`Constraint`・衝突型)
- [ ] 流体(MAC 格子・SPH 粒子)
- [ ] 熱(熱ノード・相変化)
- [ ] 電磁(回路 MNA・静場・FDTD・光学)
- [ ] 量子(TDSE)
- [ ] 統計(気体分子・イジング・ランジュバン)
- [ ] 天体(N 体・軌道・フレーム階層)
- [ ] レンダリング(パストレ骨格)
- [ ] World / Coupling / 台帳 / スナップショット

テスト記述(定義は [21-verification/01-analytic-tests.md](../21-verification/01-analytic-tests.md)、
Green 管理は [§8](#8-解析解テスト-green-管理表) で行う):

- [ ] 力学 M1–M15 を記述、全 Red 確認
- [ ] 流体 F1–F11 を記述、全 Red 確認
- [ ] 熱 T1–T8 を記述、全 Red 確認
- [ ] 電磁 E1–E13 を記述、全 Red 確認
- [ ] 量子 Q1–Q6 を記述、全 Red 確認
- [ ] 統計 S1–S9 を記述、全 Red 確認
- [ ] 天体 A1–A10 を記述、全 Red 確認
- [ ] レンダリング R1–R7 を記述、全 Red 確認
- [ ] 結合 stiff 検出 X1–X2 を記述、全 Red 確認
- [ ] 各ドメイン文書 §7 のユニットテストを記述
- [ ] 保存則テスト(21-verification/02)を記述
- [ ] 決定論テスト(20-integration/02 §6)を記述
- [ ] テスト自体のレビュー完了(Phase A 完了条件)

## 3. Phase B — 実装ウェーブ(Green)

### math ウェーブ

- [x] 線形代数(Vec3/Quat/Mat3/テンソル)
- [ ] 場(MAC / セル中心格子・補間)
- [ ] 積分器カタログ: 状態非依存の汎用部分は Green
      (explicit/semi-implicit Euler・velocity Verlet・RK4=`BallisticIntegrator`・Boris pusher、
      `crates/sim-math/src/integrators.rs`)。ドメイン状態型が要る残り(XPBD・Euler–Maruyama/BAOAB・
      陰的 Euler・semi-Lagrangian・leapfrog・split-step Fourier)と `RigidIntegrator` トレイトは
      各ドメイン crate の P1–P5 実装時に追加する
- [x] 決定論 PRNG(SimRng)・分布サンプリング(PCG-XSH-RR 64/32、公式参照ベクタ一致 —
      docs/01-math/04-random.md §1/§3/§5)
- [ ] 数学基盤テスト・収束次数 ◆ Green(線形代数・PRNG は個別に Green 済み。場・積分器が残る)

### P1 — 力学基礎

- [ ] 剛体(状態・慣性テンソル・力/トルク API)
- [ ] 総当たり衝突・接触ソルバ(sequential impulses)
- [ ] 摩擦(クーロン・摩擦円錐)
- [ ] 最小 CCD(弾丸級の speculative contact)
- [ ] 位置表現 = フレーム ID + ローカル座標
- [ ] 重力・抗力・浮力
- [ ] 熱ノード(基礎)
- [ ] エネルギー台帳(残差トレンド監視)
- [ ] 担当テスト Green: M1–M9, M12, M15, F1–F6, T1, T2

### P2 — 力学拡充

- [ ] SAP / BVH(broadphase)
- [ ] Box-Box(SAT)
- [ ] split impulse・スリープ・転がり摩擦
- [ ] 担当テスト Green: M6(精度), M10, M11

### P3 — 拘束・流体・熱

- [ ] ジョイント・拘束(ヤコビアン)
- [ ] XPBD(布・ロープ)
- [ ] 格子流体(MAC・semi-Lagrangian・投影法)
- [ ] 熱伝導網・相変化(エンタルピー法)・気体区画
- [ ] 並列リダクション(同一スレッド数で決定的 — C-1 案 1)
- [ ] 担当テスト Green: M3, M4, M13, M14, F7–F9, F11, T3, T5, T7

### P4 — 電磁・光・SPH・車両・ブラウン

- [ ] 回路(MNA・非線形素子収束)
- [ ] モーター結合(sub-iteration 決定的算出)
- [ ] 静電場・静磁場
- [ ] 幾何光学
- [ ] WCSPH
- [ ] 車両(Pacejka)
- [ ] ランジュバン(ブラウン運動)
- [ ] エンティティ受け入れ: 関節 PD 静的姿勢維持
- [ ] 担当テスト Green: E1–E7, E9–E12, F10, S4–S6, T8

### P5 — 量子・統計・波動

- [ ] シュレディンガー(split-step)
- [ ] FDTD(Yee・PML)
- [ ] 気体分子運動
- [ ] イジング(Metropolis + Wolff 必須)
- [ ] GJK / EPA・フル CCD
- [ ] 担当テスト Green: Q1–Q6, E8, E13, S1–S3, S7–S9

### Pα — 天体

- [ ] N 体重力(Barnes-Hut・シンプレクティック)
- [ ] 軌道・再突入・宇宙機
- [ ] フレーム階層・floating origin
- [ ] レジーム切替(時間加速)
- [ ] 1PN 補正(オプトイン)
- [ ] 担当テスト Green: A1–A10

## 4. Phase C — 結合・全体検証

- [ ] 結合行列の実装(保存量の対記帳・排他結合 validator)
- [ ] 統合シナリオ: ブレーキ発熱
- [ ] 統合シナリオ: 手回し発電
- [ ] 統合シナリオ: 氷と飲み物
- [ ] 統合シナリオ: 断熱圧縮
- [ ] 統合シナリオ: 再突入
- [ ] CI ゲート: 決定論(2 回実行一致・スナップショット再開一致 = 階層 1、スレッド数変更・wasm⇔ネイティブは許容誤差 = 階層 2 — C-1 案 1)
- [ ] CI ゲート: 保存則 residual
- [ ] CI ゲート: 性能ベンチ回帰(構成規則)
- [ ] 全デモ D1–D39 合格([§7](#7-デモ合格管理表-d1d43))

## 5. Phase D — レンダリング

- [ ] BVH(レイ交差)
- [ ] BSDF・NEE
- [ ] 分光・屈折・コースティクス
- [ ] 参加媒質(大気・水・煙)
- [ ] 物理カメラ・トーンマッピング
- [ ] 担当テスト Green: R1–R7
- [ ] デモ D40–D43 合格

## 6. フロントエンド(設計は [../23-frontend/01-editor.md](../23-frontend/01-editor.md) が正)

Unity 風統合エディタ:

- [ ] Toolbar: 再生制御(▶/⏸/⏭)+ 時間倍率スライダー + 状態ハッシュ表示
- [ ] Scene View: Three.js 3D ビューポート + Gizmo(移動/回転/スケール)+ ピック
- [ ] Scene View オーバーレイ(接触点/速度/力/拘束/流体場/フレーム軸、切替可)
- [ ] Hierarchy: シーングラフツリー(Bodies/Joints/Circuits/Fluids/Probes/Frames)、双方向選択
- [ ] Inspector: Component ビュー(Transform/RigidBody/Joint/Circuit/FluidRegion/Coupling/Probe/近似バッジ)
- [ ] Timeline: 再生スクラバ + Play モードバッジ + ブックマーク
- [ ] Console: イベント・診断ログ(発散・CFL 警告・シーンクラス/スロー再生バッジ)+ フィルタ + クリック→時刻/オブジェクト連動
- [ ] Probe Graphs パネル: 複数系列・対数軸・CSV エクスポート
- [ ] Project ドロワー: Scenes/Materials/Prefabs/Replays
- [ ] Edit / Play モードの切替と編集ロック
- [ ] Command 系(Grab/MoveGrab/Release/SetMotorTarget/…)と入力列記録
- [ ] レイアウトプリセット(Default / Physics-focus / Circuit-focus / Astro)
- [ ] 回路エディタ(Scene View サブモード、D19 自由配線)
- [ ] フレームサブモード(L5 ドリルイン)
- [ ] 予測→実験ミニパネル(シーン側オプトイン)
- [ ] シーン編集・スポーン・材料派生
- [ ] シーン + Replay + ブックマークのエクスポート/インポート
- [ ] Undo / Redo(Edit モードのみ)
- [ ] ヘッドレスランナー(Probe assert・CI 基盤)

## 7. デモ合格管理表(D1–D43)

定義は [21-verification/03-demo-scenarios.md](../21-verification/03-demo-scenarios.md)。
「合格」= 合格基準のヘッドレステスト Green + 目視チェック。

Phase 1(P1〜P2 スモーク):

- [ ] D1 落下時計
- [ ] D2 弾道
- [ ] D3 バウンド比べ
- [ ] D4 積み木
- [ ] D5 斜面
- [ ] D6 浮き沈み
- [ ] D7 風と終端速度
- [ ] D8 散乱の再現
- [ ] D9 冷めるコーヒー
- [ ] D10 摩擦の熱

Phase 2〜3:

- [ ] D11 振り子と時計
- [ ] D12 ラグドール階段
- [ ] D13 ロープと旗
- [ ] D14 煙と渦
- [ ] D15 対流
- [ ] D16 熱伝導レース
- [ ] D17 ピストン
- [ ] D18 氷と飲み物

Phase 4:

- [ ] D19 電気工作台
- [ ] D20 モーターと発電
- [ ] D21 磁石遊び
- [ ] D22 光学ベンチ
- [ ] D23 注ぐ水(SPH)
- [ ] D24 車の実験場
- [ ] D25 ブラウン運動
- [ ] D26 帯電風船

Phase 5:

- [ ] D27 二重スリット(電子)
- [ ] D28 トンネル効果
- [ ] D29 電波の水槽
- [ ] D30 気体の箱
- [ ] D31 拡散とインク
- [ ] D32 磁石の相転移
- [ ] D33 井戸の中の電子

Pα:

- [ ] D34 太陽系儀
- [ ] D35 軌道投入
- [ ] D36 スイングバイ
- [ ] D37 再突入
- [ ] D38 潮汐
- [ ] D39 相対論 ON/OFF

Phase D:

- [ ] D40 光の実験室
- [ ] D41 材質ギャラリー
- [ ] D42 空と大気
- [ ] D43 カメラ

## 8. 解析解テスト Green 管理表

定義・許容誤差は [21-verification/01-analytic-tests.md](../21-verification/01-analytic-tests.md)。
記述(Red)の管理は §2、Green 化はここでチェックする。長時間級(通常 CI 外)の行は
PR-2 の監査で確定後、末尾に「(長時間級)」を付記すること。

力学(M、担当: P1〜P3):

- [ ] M1
- [ ] M2
- [ ] M3
- [ ] M4
- [ ] M5
- [ ] M6
- [ ] M7
- [ ] M8
- [ ] M9
- [ ] M10
- [ ] M11
- [ ] M12
- [ ] M13
- [ ] M14
- [ ] M15

流体(F、担当: P1/P3/P4):

- [ ] F1
- [ ] F2
- [ ] F3
- [ ] F4
- [ ] F5
- [ ] F6
- [ ] F7
- [ ] F8
- [ ] F9
- [ ] F10
- [ ] F11

熱(T、担当: P1/P3/P4):

- [ ] T1
- [ ] T2
- [ ] T3
- [ ] T4
- [ ] T5
- [ ] T6
- [ ] T7
- [ ] T8

電磁(E、担当: P4/P5):

- [ ] E1
- [ ] E2
- [ ] E3
- [ ] E4
- [ ] E5
- [ ] E6
- [ ] E7
- [ ] E8
- [ ] E9
- [ ] E10
- [ ] E11
- [ ] E12
- [ ] E13

量子(Q、担当: P5):

- [ ] Q1
- [ ] Q2
- [ ] Q3
- [ ] Q4
- [ ] Q5
- [ ] Q6

統計(S、担当: P4/P5):

- [ ] S1
- [ ] S2
- [ ] S3
- [ ] S4
- [ ] S5
- [ ] S6
- [ ] S7(L=256 フル版は長時間級)
- [ ] S8(L=256 フル版は長時間級)
- [ ] S9

天体(A、担当: Pα):

- [ ] A1
- [ ] A2(10⁶ 周フル版は長時間級)
- [ ] A3
- [ ] A4
- [ ] A5
- [ ] A6
- [ ] A7
- [ ] A8
- [ ] A9
- [ ] A10

レンダリング(R、担当: Phase D):

- [ ] R1
- [ ] R2
- [ ] R3
- [ ] R4
- [ ] R5
- [ ] R6
- [ ] R7

結合 stiff 検出(X、担当: P4/Phase C):

- [ ] X1
- [ ] X2
