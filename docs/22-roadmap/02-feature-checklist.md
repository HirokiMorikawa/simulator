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

- **フェーズ**: 実装(Phase 0 完了。math ウェーブは状態非依存の全項目が Green。Phase A 着手済み:
  `sim-core` の `Solver`トレイト・`MaterialDb`・`EventQueue`・`FrameId`。
  P1 力学(`sim-mechanics`)は M15(最小CCD)を除き全担当テスト Green
  (剛体・衝突検出・sequential impulses・摩擦・重力・抗力・浮力)。P2 力学は Box-Box(SAT)+
  軸選択ヒステリシス(相対5%、`collision::AxisCache`)+ warm starting
  (`contact::WarmStartCache`)+ split impulse(NGS、`contact::position_correction`)を実装し、
  M6(rel 1%)・M12(4段スタック、速度~1e-10まで収束)を Green 化(M10/M11 は未着手、
  詳細は下記 §2/§3)。`sim-thermal`(熱ノード、T1・T2)・`sim-core::EnergyLedger`
  (残差トレンド監視)・`sim-world`(`create_body` 経由の複数剛体構成)・`sim-astro`
  (N体重力、A1・A2縮約版・A3・A7)も実装済み — 詳細と各増分の設計判断・発見したバグの
  経緯は git log 参照(1コミット1増分の粒度を維持)。
  力学ドメインの角運動量・回転運動エネルギー保存則テストも追加(陽的ジャイロのドリフト率を
  実測・文書化)。M11 の解析成長率比較は簡易線形化では検証できないと判明、P2着手時の
  課題として記録)。`sim-statistical` にランジュバン方程式(BAOAB、`brownian.rs`)を実装し
  S4・S5・S6 を Green 化(S4 実装検証中に BAOAB の位置更新離散化誤差が γΔt/m に強く依存する
  ことを発見・文書化。S6=沈降平衡は床での弾性反射をテスト内で直接実装し、合成的に強めた
  重力加速度で平衡到達を高速化)。`sim-em` に静電場(`PointChargeSystem`、点電荷直接和
  クーロン力 + 一様外場合成 + Boris pusher 積分)を実装し E1・E2 を Green 化(E2 は既存の
  `sim-math::BorisPusher` テストが検証済みの物理を sim-em の公開 API 経由で改めて確認)。
  P2 力学に転がり摩擦(`contact::solve_rolling`、トルク制約を純粋な偶力として実装)を追加し、
  対応する M 番号がないため自前でエネルギー収支を導出したテストで検証(rel 2%)。`sim-em` に
  幾何光学の代数公式(`optics.rs`: スネル則・臨界角・フレネル係数・ブリュースター角・薄レンズ・
  プリズム最小偏角)を実装し E9–E12 を Green 化(フル `RayTracer` は未実装、公式のみ)。
  P2 力学に SAP broadphase(`collision::sap_candidate_pairs`、x軸掃引)を追加し総当たり版と
  結果が完全一致することを確認(BVH は未着手)。P2 力学にスリープ(`sleep::update_sleep_state`、
  接触島 union-find + 積分停止 + 接触解決停止)を実装(実装検証中に「積分停止だけでは
  不十分、接触解決自体も止めないと数値的揺らぎで再起床を繰り返す」ことを発見・修正)。
  P3 力学にジョイント(`joint::{DistanceJoint, BallJoint}`)を実装し、単振り子として
  M3・M4(Distance、大振幅の理論周期は完全楕円積分の自前AGM実装で算出)、独楽の歳差として
  M10(Ball、重心オフセット支点の固定。等方慣性の球を使ったため歳差式が近似でなく厳密になる
  ことに気づき、章動対策として速い自転+短時間平均で実測)を Green 化。`sim-em` に回路MNA
  (`circuit::Circuit`、線形素子(R/C/L/独立電圧源)のみ、後退Eulerコンパニオンモデル+
  ガウス消去)を実装し E3(RC過渡)・E4(RLC減衰)・E5(分圧)を Green 化(非線形素子・
  モーター結合は未実装)。`sim-em` に静磁場(`magnetism.rs`、磁気双極子の場・トルク(閉形式)・
  力(数値勾配))を実装し、整列2磁石の r^-4 引力則を検証(対応するE番号が無いため自前導出)。
  P3 力学にXPBDロープ(`soft_body::{SoftBody, rope}`、距離拘束のみ)を実装し、
  M13(懸垂線)・M14(伸び)を Green 化(実装検証中に、既定サブステップ数では高剛性・
  軽量シナリオで正しい剛性に収束しないことを発見、セグメント固有振動周期に対して
  十分細かいサブステップが必要と判明)。`sim-thermal` に T4(放射平衡)・T8(Antoine式の
  沸点気圧依存)を追加し Green 化 — T4 の実装検証中に既存の放射線形化コードの
  バグ(Newton線形化の補正項欠落、平衡温度が真値の1/4^0.25倍にずれる)を発見・修正した
  (T1/T2 は放射を使わない/ΔTが小さいため検出できていなかった)。`sim-astro` にホーマン遷移
  (A4)を追加し Green 化(既存の `NBodySystem` に瞬間噴射を加えるだけで表現でき、専用の
  軌道力学モジュールは追加せず済んだ)。`sim-quantum` に1D TDSE(split-step Fourier、
  `schrodinger::WaveFunction1D`)を実装し Q1(ノルム保存)・Q2(自由波束の広がり)を
  Green 化 — 自前radix-2 FFT(`sim_math::fft`)を新規実装し、小さいNでの素朴DFTとの
  検算・往復変換で正しさを確認した(量子ドメイン初着手)。続けて虚時間発展
  (`step_imaginary`・`find_eigenstates`、部分空間反復+Gram-Schmidt直交化)を実装し
  Q3(無限井戸固有値)・Q4(調和振動子固有値+コヒーレント状態のエーレンフェスト一致)を
  Green 化(無限井戸は周期境界FFTでは表現できない真の無限大障壁の代わりに有限障壁を
  使う必要があり、空間離散化誤差と split-step 時間離散化誤差が逆符号で効くため単純な
  格子細分化では改善せず、両者が打ち消し合う経験的最適な d_tau が存在することを
  スイープで確認して使用)。2D・吸収境界・検出スクリーンは未実装。`sim-statistical` に
  気体分子運動論(剛体球MD、`kinetic_gas::GasSim`)を実装し S1(MB分布収束、等確率ビンの
  χ²検定)・S2(状態方程式pV=NkT)・S3(等分配則)を Green 化 — 実装検証中に、S1に都合が
  良い密な粒子配置(φ≈0.34)ではS2で剛体球の排除体積によるvirial補正(Carnahan-Starling
  状態方程式と整合)でpVがNkTの約5倍にずれることを発見し、S2は希薄配置(φ≈0.0012)に
  分けて解決した。`sim-quantum` にトンネル効果(Q5、矩形障壁への波束入射)を実装し
  Green 化 — 波束の運動量スペクトルで重み付けした解析式の期待値との比較に切り替え、
  周期境界の一周(反射波束が透過側に誤カウントされる)前の安定した時間窓で測定する
  ことで解決した。
- **作業中**: 力学(`sim-mechanics`)P1 最後の残り — 最小CCD(M15、意図的に後回し)。
  次点候補: BVH(P2 残り)、ダイオード/モーター結合・フル RayTracer(sim-em 残り)、
  イジング(S7–S9、Metropolis + Wolff 必須)、量子の二重スリット(Q6、2D ソルバが新規必要)
- **次**: 力学 P1 の残りを詰めたら流体・熱・電磁・量子・統計・天体・レンダリングの型スケルトンへ
  → World/Coupling 拡張、の順にスケルトンと Phase A テスト記述を進める(下記 §2)。
  math ウェーブ(`sim-math` の `Vec3`/`Quat`/`Mat3`/`Transform`/`SimRng`/積分器カタログの汎用部分/
  `Grid3`/`MacGrid`/`GridSampler`/トライリニア・Catmull-Rom補間/勾配・ラプラシアン/`pcg`/`ParticleSet`/
  `SpatialHash`)は依存が無く低リスクなため、Phase A の Red 段階を経ずに直接実装 + テストで Green 化した。
  ただし `RigidIntegrator` トレイト(P1、`RigidBodySet` に依存)・陰的 Euler の具体的な線形系(場は
  揃ったが熱ドメインの温度場が必要)・IC(0)前処理(格子ラプラシアンの疎パターンが必要)・
  leapfrog(Yee)・split-step Fourier・XPBD・semi-Lagrangian・BAOAB は状態型を持つ各ドメイン crate が
  P1–P5 で実装する(sim-math には汎用プリミティブのみ置く)。
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

- [x] math(Vec3/Quat/Mat3・場・`Integrator`・SimRng)— Red を経ず直接 Green 化済み(§3参照)
- [x] 力学(剛体状態・`Solver`/`Constraint`・衝突型)— `RigidBodySet`/`BodyType`/`Shape`(Sphere/Box/
      Plane 実装、Capsule/Compound/ConvexMesh は型のみ)/`MechanicsSolver`(`Solver`実装)まで完了。
      `Constraint`(ジョイント)型は P3 で追加
- [ ] 流体(MAC 格子・SPH 粒子)
- [ ] 熱(熱ノード・相変化)
- [ ] 電磁(回路 MNA・静場・FDTD・光学)
- [ ] 量子(TDSE)
- [ ] 統計(気体分子・イジング・ランジュバン)
- [ ] 天体(N 体・軌道・フレーム階層)
- [ ] レンダリング(パストレ骨格)
- [ ] World / Coupling / 台帳 / スナップショット — `sim-core` 側の共通基盤(`Solver`トレイト・
      `SolverContext`・`EventQueue`・`MaterialDb`)は先行実装済み(`crates/sim-core/src/{solver,material}.rs`)。
      `World`本体の拡張・`Coupling`トレイト・`EnergyLedger`・スナップショットは未着手

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
- [ ] 保存則テスト(21-verification/02)を記述(力学ドメインの角運動量・回転運動エネルギー
      保存は Green 実装済み — `crates/sim-mechanics/tests/conservation.rs`。陽的ジャイロ積分の
      ドリフト率を実測・文書化(dt=1/120・1秒で |L|≈0.52%、KE≈0.79%、許容2%)。他ドメイン・
      他保存量は未記述)
- [ ] 決定論テスト(20-integration/02 §6)を記述
- [ ] テスト自体のレビュー完了(Phase A 完了条件)

## 3. Phase B — 実装ウェーブ(Green)

### math ウェーブ

- [x] 線形代数(Vec3/Quat/Mat3/テンソル)
- [x] 場(MAC / セル中心格子・補間)— `Grid3<T>`/`BoundaryRule`/`GridSampler`(Clamp/Constant/
      ZeroGradient/Periodic)、トライリニア・Catmull-Rom 補間、勾配・ラプラシアン(一様係数・流束形式の
      変係数版)、`MacGrid`+発散、PCG(`pcg`、Jacobi 前処理。IC(0) は P3 で具体的な格子ステンシルと
      併せて実装)、`ParticleSet`、`SpatialHash`(Teschner ハッシュ、総当たり一致テスト済み) —
      `crates/sim-math/src/{grid,pcg,particles}.rs`
- [x] 積分器カタログ: 状態非依存の汎用部分は Green
      (explicit/semi-implicit Euler・velocity Verlet・RK4=`BallisticIntegrator`・Boris pusher、
      `crates/sim-math/src/integrators.rs`)。ドメイン状態型が要る残り(XPBD・Euler–Maruyama/BAOAB・
      陰的 Euler・semi-Lagrangian・leapfrog・split-step Fourier)と `RigidIntegrator` トレイトは
      各ドメイン crate の P1–P5 実装時に追加する
- [x] 決定論 PRNG(SimRng)・分布サンプリング(PCG-XSH-RR 64/32、公式参照ベクタ一致 —
      docs/01-math/04-random.md §1/§3/§5)
- [x] 数学基盤テスト・収束次数 ◆ Green(sim-math 全体で47テスト。ドメイン結合が要る残りの積分器・
      IC(0) は各ドメイン crate の担当ウェーブで追加テストする)

### P1 — 力学基礎

- [x] 剛体(状態・慣性テンソル・力/トルク API)— `crates/sim-mechanics/src/{body,shape,solver}.rs`
- [x] 総当たり衝突・接触ソルバ(sequential impulses)— `crates/sim-mechanics/src/{collision,contact}.rs`。
      narrowphase は Sphere-Sphere/Sphere-Plane/Box-Plane/Sphere-Box(Phase1の4組)+
      Box-Box(SAT、15軸+Sutherland-Hodgmanクリップ、`collision.rs::box_box`)。
      軸選択のヒステリシス(相対5%、`collision::AxisCache`)+ warm starting
      (feature_idベース、`contact::WarmStartCache`。feature_idは軸選択+参照面上の象限から
      安定的に組み立てる、post-clipのインデックスは使わない)+ split impulse(NGS、§4.5、
      `contact::position_correction`)を実装。マニフォールド持続化(§4.7 の移動量2mmチェック)
      は未実装
- [x] 摩擦(クーロン・摩擦円錐)— 箱近似(2接線独立クランプ、`contact.rs::solve_tangent`)、
      `MaterialDb::friction_pair`(幾何平均+ペア表)を実接触ソルバで使用
- [ ] 最小 CCD(弾丸級の speculative contact)
- [x] 位置表現 = フレーム ID + ローカル座標(`sim_core::FrameId`、単一ルートフレームで運用中。
      フル階層は Pα)
- [x] 重力(実装済み)。抗力(球、Schiller-Naumann補正付き、`sim-fluid::aero`+
      `MechanicsSolver::apply_forces`)を実装、F1–F3 Green化。浮力(直立直方体、
      `sim-fluid::buoyancy`+`MechanicsSolver.water`)を実装、F4–F6 Green化(一般姿勢の
      凸多面体切断・球冠体積・水中抗力は Phase 3)
- [x] 熱ノード(基礎)— `crates/sim-thermal/src/lib.rs`。集中熱容量ノード網 + ニュートン冷却
      (対流)+ 放射(Newton線形化、現在温度周りの補正項込み)+ 陰的Euler(matrix-free PCG、
      `sim_math::pcg`)。Antoine式(`antoine_boiling_point_celsius`、設計12-thermal/03 §2)も追加
- [x] エネルギー台帳(残差トレンド監視)— `crates/sim-core/src/ledger.rs::EnergyLedger`
      (docs/00-foundation/04-architecture.md §1.1.2(2)、docs/21-verification/02-conservation-laws.md
      §2 の residual 式)。`sim-world::World` に配線し毎 step 後に mechanics 合計エネルギーを記帳。
      解析予測(接触なし自由落下の semi-implicit Euler 線形ドリフト)と記帳値が一致することを
      `crates/sim-world/src/lib.rs::tests::energy_ledger_residual_matches_analytic_symplectic_drift`
      で検証
- [x] 担当テスト Green: M1–M9, M12, M15除く F1–F6, T1, T2(M1・M5–M9・M12・F1–F6・T1・T2
      Green。M12 は split impulse 実装で最終的に Green 化(速度~1e-10まで収束、各接触の
      貫入もslop未満)。M15=最小CCD待ちで残存)。T4・T8 も Green 化(T4 実装検証中に
      放射線形化の欠落バグを発見・修正、詳細は §8 T4 の記録参照)

### P2 — 力学拡充

- [x] Box-Box(SAT)— `crates/sim-mechanics/src/collision.rs::box_box`。15軸分離判定
      (面3+3、辺×辺9)+ 面接触は参照面への Sutherland-Hodgman クリップ(最大4点、
      設計 §4.4 の縮約は簡易版: 面積最大化でなく深度降順で上位4点)+ 辺×辺接触は
      2線分の最近点1点。退化ケース(平行辺の軸除外・クリップ0点フォールバック)を実装
- [x] 軸選択ヒステリシス(相対5%、`collision::AxisCache`)— 設計 §4.4・§9。同一サイズの
      箱が積み重なるとA面軸/B面軸の重なり量が理論上完全一致し、浮動小数点誤差で
      ステップごとに選択軸がフリップして warm start の feature_id 対応を破壊する
      (実測: ヒステリシスなしでは warm starting がむしろ速度残差を悪化させた)ことを発見・
      修正
- [x] Warm starting(feature_idベース、`contact::WarmStartCache`)— 設計 §4.4。マニフォールド
      持続化(§4.7 の移動量チェックによる再利用判定)は未実装、feature_id 自体は軸選択+
      参照面象限から安定的に算出(post-clipインデックスは不安定なため不使用)
- [x] Split impulse(NGS、`contact::position_correction`)— 設計 §4.5。速度チャンネルから
      Baumgarte 項を除去し、位置補正を別チャンネル(Δλ=β_pos・max(δ-slop,0)・m_eff を
      位置・姿勢へ直接適用)に分離。各反復・各点で現在の body 位置から貫入量を**再計算**
      する(NGS の要点、同一bodyに複数接触点があると独立減算では過剰補正になることを
      実装中に発見・修正)。M6 を設計の目標精度(rel 1%)まで、M12 を Green 化した
- [x] SAP(broadphase、BVH は未着手)— `crates/sim-mechanics/src/collision.rs::sap_candidate_pairs`。
      x 軸への AABB 射影でソート+掃引し、総当たり $O(N^2)$ のペア列挙を削減(設計 §4.1 表)。
      結果は総当たり版と (indexA,indexB) 昇順で完全一致するようソート済み(決定論・既存の
      数値挙動を保つ)。散らばった40体シーンで総当たり列挙と一致することをテストで確認
      (`collision::tests::sap_matches_brute_force_pair_enumeration_on_scattered_scene`)
- [x] スリープ — `crates/sim-mechanics/src/sleep.rs::update_sleep_state`。dynamic-dynamic
      接触の連結成分(接触島、union-find)単位で、島内の全 dynamic body の速度が閾値
      (0.01 m/s / 0.02 rad/s)未満の状態が0.5秒続いたら asleep にし、力適用・速度積分・
      位置積分に加えて**両側とも asleep な接触の再解決**も止める(`manifold_is_active`)。
      実装検証中に、contact solve だけ止めずに毎ステップ回し続けると warm start・split
      impulse の数値的な揺らぎで凍結直後の速度が再摂動され再起床→再入眠を繰り返し、
      M12の最終速度が閾値1e-3を上回る(かえって収束が乱れる)ことを発見・修正。
      眠りに入った瞬間は残留速度を厳密に0にする。新規接触(異なる島の合流)で即座に
      起床することをテストで確認(`p2_analytic.rs::sleep_engages_after_box_settles_on_ground`,
      `sleeping_box_wakes_on_new_contact_from_falling_body`)
- [x] 転がり摩擦 — `crates/sim-mechanics/src/contact.rs::solve_rolling`。
      設計 04-friction.md §4.1 のトルク制約 $|\tau_{roll}|\le\mu_{roll}Nr$ を、線形速度を
      変えない純粋な偶力(角速度のみ更新)として `solve_tangent` と同じクランプ構造で実装
      (Sphere 形状の半径を使用、非球形接触は自動的に無効化)。
      `crates/sim-mechanics/tests/p2_analytic.rs::rolling_friction_decelerates_ball_at_designed_rate`
      で検証: 対応する M 番号が無いため設計のトルク制約から自前でエネルギー収支を導出し、
      滑りなし転がり球の並進減速度が単純な $a=\mu_{roll}g$ ではなく回転慣性を含む有効質量
      $\frac75 m$ から出る $a=\frac57\mu_{roll}g$ になることを実測で確認(rel 2%)
- [ ] 担当テスト Green: M6(精度), M10, M11(M6・M10 Green。M11 は簡易線形化で解析成長率との
      比較を試みたが、非線形フィードバック(ωx・ωzの積がωyへ2λ倍のレートで再結合)により
      線形近似が数値実験で想定より早く破綻することを確認済み — 正しい検証には Jacobi 楕円関数
      による厳密解、または慎重な多重スケール摂動法が必要で、P2着手時に再検討する)

### P3 — 拘束・流体・熱

- [x] ジョイント・拘束(ヤコビアン)— `crates/sim-mechanics/src/joint.rs::{DistanceJoint,
      BallJoint}`。設計 §4.4 表の Distance(1行、$|\mathbf{p}_B-\mathbf{p}_A|=L$)と
      Ball(3行、アンカー一致 $\mathbf{p}_B=\mathbf{p}_A$、§2.1のヤコビアン導出)を実装、
      どちらも `body_b=None` でワールド固定点への接続(振り子の支点・独楽の支点等)を表せる。
      Ball の3行は真の3×3ブロックソルバ(コレスキー)ではなくワールドx/y/z軸に沿った
      3本の独立スカラー拘束として簡略化(接触ソルバの摩擦「箱近似」と同じ方針)。
      Hinge(limit・motor)/Slider/Fixed/Wheel・ソフト拘束は未実装 — Baumgarte速度バイアス
      (β=0.2、設計§9)は使うが接触ソルバのような split impulse化はしていない
- [x] XPBD(ロープのみ、布は未実装)— `crates/sim-mechanics/src/soft_body.rs::{SoftBody,
      rope}`。距離拘束(設計§2.2)のみ実装、`MechanicsSolver` とは独立に動作する
      (`sim_statistical::BrownianParticleSet` と同様のパターン)。曲げ拘束・体積拘束・
      布/ゼリー生成ヘルパ・剛体/流体結合・自己衝突は未実装。実装検証中に、既定のサブステップ数
      (4)では特定の高剛性・軽量質点比のシナリオ(M14)で伸びが理論値の約5.6倍に収束してしまう
      ことを発見 — セグメントの固有振動周期が既定サブステップ幅より短いと粗いサブステップでは
      正しい剛性に収束しない(サブステップ数を増やして解消、設計§4「サブステップ優先」の
      実地確認)
- [ ] 格子流体(MAC・semi-Lagrangian・投影法)
- [ ] 熱伝導網・相変化(エンタルピー法)・気体区画
- [ ] 並列リダクション(同一スレッド数で決定的 — C-1 案 1)
- [ ] 担当テスト Green: M3, M4, M13, M14, F7–F9, F11, T3, T5, T7(M3・M4・M13・M14 Green)

### P4 — 電磁・光・SPH・車両・ブラウン

- [x] 回路(線形素子のMNAのみ、非線形素子収束は未実装)— `crates/sim-em/src/circuit.rs::Circuit`。
      抵抗・コンデンサ・インダクタ・独立電圧源のみ。動的素子は後退Eulerコンパニオンモデルへ
      変換、密行列を部分ピボット付きガウス消去で毎ステップ解く(トポロジ不変時のLU分解
      キャッシュは未実装)。ダイオード・モーター等の非線形素子と Newton-Raphson
      フォールバック連鎖(gmin/source stepping)・スイッチは未実装
- [ ] モーター結合(sub-iteration 決定的算出)
- [x] 静電場(点電荷直接和 + Boris pusher)— `crates/sim-em/src/electrostatics.rs::PointChargeSystem`。
      $O(N^2)$ 直接和クーロン力(設計 §4「数十源で十分」)+ 一様外場を合成し Boris pusher で積分。
      鏡像力・摩擦帯電・放電イベントは未実装
- [x] 静磁場(磁気双極子)— `crates/sim-em/src/magnetism.rs`。場は閉形式(設計§2)、トルクは
      $\tau=m\times B$、力は $F=\nabla(m\cdot B)$ を閉形式の双極子間力式ではなくポテンシャルの
      中心差分数値勾配として実装(任意の相対配置に対応する単一実装で済むため)。整列した
      2磁石の引力が $F=3\mu_0 m_1m_2/(2\pi r^4)$(設計§7の r^-4 冪則)に一致することを検証
      (対応するE番号が無いため自前導出)。多体の直接和ループ・永久磁石の剛体姿勢追従は未実装
- [x] 幾何光学(代数公式のみ)— `crates/sim-em/src/optics.rs`。スネル則・臨界角・
      フレネル反射率(s/p偏光)・ブリュースター角・薄レンズ(レンズメーカーの式 +
      近軸光線追跡)・プリズム最小偏角。フル `RayTracer`(光線束追跡・分岐・分光・
      衝突検出のray-cast再利用)は未実装
- [ ] WCSPH
- [ ] 車両(Pacejka)
- [x] ランジュバン(ブラウン運動)— `crates/sim-statistical/src/brownian.rs::BrownianParticleSet`。
      BAOAB(kick-drift-kick+OU厳密解+kick-drift-kick、設計 §4.1)を実装。濃度場の拡散
      (陰的Euler・熱伝導と共有)・移流拡散・回転ブラウン運動は Phase 5+
- [ ] エンティティ受け入れ: 関節 PD 静的姿勢維持
- [ ] 担当テスト Green: E1–E7, E9–E12, F10, S4–S6, T8(E1・E2・E3–E5・E9–E12・S4・S5・S6 Green。
      E6・E7 はモーター結合が必要で未実装)

### P5 — 量子・統計・波動

- [x] シュレディンガー(1D split-step Fourierのみ、2D・吸収境界・検出スクリーン
      サンプリングは未実装)— `crates/sim-quantum/src/schrodinger.rs::WaveFunction1D`。
      自前radix-2 FFT(`crates/sim-math/src/fft.rs`、依存最小化・決定論、設計§3)を
      新規実装し、Strang分割(半ポテンシャル→FFT→運動量空間位相回転→逆FFT→
      半ポテンシャル)で実現。原子単位($\hbar=m_e=1$)
- [x] 虚時間発展・固有状態探索 — `crates/sim-quantum/src/schrodinger.rs::{step_imaginary,
      find_eigenstates}`。$t\to-i\tau$の split-step(位相回転を実減衰に置換)を各ステップ末尾で
      再正規化しつつ反復するべき乗法(=最低エネルギー状態へ収束)。励起状態は多項式×ガウス
      包絡のシードから出発し、既知の下位状態への Gram-Schmidt 直交化(`orthogonalize_against`)
      を毎ステップ挟む部分空間反復で求める。エネルギー期待値`energy()`は運動項をParsevalの
      等式で運動量空間から評価。無限井戸(Q3)は周期境界FFTでは真の無限大障壁を表現できず
      有限障壁($V=10^6$)で近似する必要があり、空間離散化誤差(dxに起因)とsplit-step時間
      離散化誤差(d_tauに起因)が逆符号で効くため、単純に格子を細かくしても改善しない
      (両者が打ち消し合う経験的最適点 d_tau=4e-5 が存在することをスイープで確認・使用)。
      調和振動子(Q4)は滑らかなポテンシャルのためこの問題がなく、粗い格子で高精度に収束。
      続けてトンネル効果(Q5、矩形障壁への波束入射)を実装し Green 化 —
      波束は単一エネルギーでないため素朴に $T(E_0)$ と比較すると合わず(透過率がエネルギーの
      凸関数のため実測が系統的に大きくなる)、初期波束の運動量スペクトルで重み付けした
      解析式の期待値との比較に切り替えて解決。測定タイミングは、障壁通過直後の安定確率
      から反射波束が周期境界を一周して透過側に誤カウントされ始める前までの時間窓
      (プラトーを実測で確認)の中央付近を使う
- [ ] FDTD(Yee・PML)
- [x] 気体分子運動(剛体球MDのみ、Lennard-Jones・熱壁・ピストン・輸送係数測定は未実装)—
      `crates/sim-statistical/src/kinetic_gas.rs::GasSim`。空間ハッシュ(セル幅=直径)による
      broadphase + 等質量弾性衝突(法線成分の完全交換、導出済み)+ 反射壁。壁への運動量移動
      から圧力を測定。実装検証中に、S1(MB分布収束)に都合が良い密な粒子配置(充填率φ≈0.34)
      を使うとS2(pV=NkT)で剛体球の排除体積によるvirial補正(Carnahan-Starling状態方程式と
      整合する大きさのずれ)でpVがNkTの約5倍になることを発見し、S2は希薄配置(φ≈0.0012)に
      分けて解決。S1のχ²検定は等確率ビン(逆CDFを二分法で算出)を用い、期待度数を全ビンで
      均一にして検定の前提を満たした
- [ ] イジング(Metropolis + Wolff 必須)
- [ ] GJK / EPA・フル CCD
- [ ] 担当テスト Green: Q1–Q6, E8, E13, S1–S3, S7–S9(Q1・Q2・Q3・Q4・Q5・S1・S2・S3 Green)

### Pα — 天体

- [x] N 体重力(総当たり + leapfrog)— `crates/sim-astro/src/nbody.rs::NBodySystem`。
      $O(N^2)$ 総当たり(設計 §4.1: 少数体は Barnes-Hut より高精度・十分速い既定モード)+
      leapfrog(kick-drift-kick、シンプレクティック)。Barnes-Hut(N≳256 向け)・WHFast は未実装
- [x] 軌道・宇宙機(ホーマン遷移のみ、再突入・スイングバイ・軌道要素変換は未実装)—
      `crates/sim-astro/src/nbody.rs::tests::a4_hohmann_transfer_delta_v_matches_analytic_value`。
      既存の `NBodySystem`(leapfrog)に瞬間噴射(速度への直接加算)で遷移軌道を実現し、
      Δv1後の半周で遠地点が目標半径に、Δv2後の速度が目標円軌道速度に、それぞれ
      解析値と一致することを検証(専用の軌道力学モジュールは追加せず既存N体系で表現)
- [ ] フレーム階層・floating origin
- [ ] レジーム切替(時間加速)
- [ ] 1PN 補正(オプトイン)
- [ ] 担当テスト Green: A1–A10(A1・A2縮約版・A3・A4・A7 Green。A5・A6・A8–A10=J2摂動/
      再突入/1PN未実装)

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

- [x] M1
- [ ] M2
- [x] M3 — `crates/sim-mechanics/tests/p3_analytic.rs::m3_small_amplitude_pendulum_period_matches_2pi_sqrt_l_over_g`
- [x] M4 — `crates/sim-mechanics/tests/p3_analytic.rs::m4_large_amplitude_pendulum_period_matches_elliptic_integral`。
      理論周期は算術幾何平均(AGM)による完全楕円積分 $K(k)$ の自前実装で計算
- [x] M5
- [x] M6(閾値0.5m/sの固定減算により有限衝突速度では厳密1e-9は達成できないため、検証は
      反発閾値0・細かいdtで理想化した設定で実施。split impulse実装後、既定パラメータで
      設計の目標精度 rel 1% を達成)
- [x] M7
- [x] M8
- [x] M9
- [x] M10 — `crates/sim-mechanics/tests/p3_analytic.rs::m10_top_precession_rate_matches_mgr_over_i_omega`。
      重心からオフセットした支点をワールド固定する `BallJoint` で独楽を表現。等方慣性の球
      (慣性テンソルがスカラー)を使ったため歳差速度公式 $\dot\phi=mgr/(I\omega)$ は近似ではなく
      厳密になる(非等方項 $(I_1-I_3)\dot\phi^2\cos\theta$ が恒等的に消える)が、章動は残るため
      ω0=1000rad/sの速い自転+短時間平均で実測(rel<2%)
- [ ] M11
- [x] M12 — `crates/sim-mechanics/tests/p2_analytic.rs::m12_four_box_stack_settles_below_velocity_threshold`。
      Box-Box(SAT)+ warm starting + 軸選択ヒステリシス + split impulse が揃って Green 化
      (速度~1e-10まで収束、各接触の貫入もslop未満。積み上げ全体の絶対沈み込みは接触数に
      比例して累積するのが正しい挙動のため、隣接ペアごとの貫入で検査)
- [x] M13 — `crates/sim-mechanics/src/soft_body.rs::tests::m13_hanging_rope_settles_into_catenary_shape`。
      理論の懸垂線パラメータ a は全長・端点間隔から二分法で逆算
- [x] M14 — `crates/sim-mechanics/src/soft_body.rs::tests::m14_rope_stretch_under_load_matches_wl_over_ea`
- [ ] M15

流体(F、担当: P1/P3/P4):

- [x] F1 — `crates/sim-mechanics/tests/p1_analytic.rs::f1_terminal_velocity_matches_high_re_drag_formula`
- [x] F2 — `crates/sim-mechanics/tests/p1_analytic.rs::f2_raindrop_terminal_velocity_matches_gunn_kinzer_measurement`
- [x] F3 — `crates/sim-mechanics/tests/p1_analytic.rs::f3_stokes_settling_matches_analytic_formula`
      (媒質密度を無視できるほど小さく取り Δρ≈ρ_particle として隔離検証。F3 は気中沈降シナリオ
      であり `MechanicsSolver::water` を設定しないため浮力機構とは独立)
- [x] F4 — `crates/sim-mechanics/tests/p1_analytic.rs::f4_cube_waterline_depth_matches_density_ratio`
- [x] F5 — `crates/sim-mechanics/tests/p1_analytic.rs::f5_floating_body_heave_period_matches_analytic_formula`
- [x] F6 — `crates/sim-fluid/src/buoyancy.rs::tests::f6_hydrostatic_pressure_matches_rho_g_h`(代数検算)
- [ ] F7
- [ ] F8
- [ ] F9
- [ ] F10
- [ ] F11

熱(T、担当: P1/P3/P4):

- [x] T1 — `crates/sim-thermal/src/lib.rs::tests::t1_newton_cooling_matches_analytic_decay`
- [x] T2 — `crates/sim-thermal/src/lib.rs::tests::t2_two_node_equilibrium_matches_weighted_average`
- [ ] T3
- [x] T4 — `crates/sim-thermal/src/lib.rs::tests::t4_radiation_equilibrium_matches_stefan_boltzmann_formula`。
      実装検証中に、既存の放射線形化(`ThermalSolver::step` の右辺)に Newton 線形化の
      補正項 $+3\varepsilon\sigma(T^n)^4$ が欠落しているバグを発見・修正した(補正項が無いと
      「対流もどきモデル」$h_{rad}(T-T_{env})$ の平衡 $q=4\varepsilon\sigma A(T_{eq}-T_{env})T_{eq}^3$
      止まりになり、真の非線形平衡 $q=\varepsilon\sigma A(T_{eq}^4-T_{env}^4)$ から系統的に
      ずれる — $T_{env}=0$ のこのテストでは4倍の乖離として顕在化した。T1/T2 は放射を
      使わない/$T$ が $T_{env}$ に近いためこのバグを検出できていなかった)
- [ ] T5
- [ ] T6
- [ ] T7
- [x] T8 — `crates/sim-thermal/src/lib.rs::tests::t8_boiling_point_at_reduced_pressure_matches_antoine_equation`。
      設計 docs/12-thermal/03-phase-change.md §7「0.7atmで≈90°C」を直接検証

電磁(E、担当: P4/P5):

- [x] E1 — `crates/sim-em/src/electrostatics.rs::tests::e1_coulomb_force_matches_inverse_square_law_at_machine_precision`
- [x] E2 — `crates/sim-em/src/electrostatics.rs::tests::e2_cyclotron_radius_matches_mv_over_qb`
      (Boris pusher の核心的な速さ保存・回転精度自体は
      `crates/sim-math/src/integrators.rs::tests::boris_pusher_*` で既に検証済み。ここでは
      sim-em の公開 API — クーロン力との合成場 + `PointChargeSystem::step` — を通した経路として
      改めて記録)
- [x] E3 — `crates/sim-em/src/circuit.rs::tests::e3_rc_transient_time_constant_matches_rc`。
      2時刻の電圧比から時定数を逆算(指数則の形そのものを検証)
- [x] E4 — `crates/sim-em/src/circuit.rs::tests::e4_rlc_decay_angular_frequency_matches_formula`
- [x] E5 — `crates/sim-em/src/circuit.rs::tests::e5_voltage_divider_matches_analytic_solution_at_machine_precision`
- [ ] E6(モーター結合、未実装)
- [ ] E7(誘導起電力、未実装)
- [ ] E8
- [x] E9 — `crates/sim-em/src/optics.rs::tests::e9_fresnel_normal_incidence_and_brewster_angle`
- [x] E10 — `crates/sim-em/src/optics.rs::tests::e10_snell_law_and_critical_angle_totally_internally_reflect`
- [x] E11 — `crates/sim-em/src/optics.rs::tests::e11_thin_lens_focal_length_matches_paraxial_ray_trace`。
      レンズメーカーの式(閉形式)と、各球面での近軸屈折を個別に追跡した近軸光線追跡
      (reduced angle 法)が独立に一致することを確認
- [x] E12 — `crates/sim-em/src/optics.rs::tests::e12_prism_minimum_deviation_index_round_trip`
- [ ] E13

量子(Q、担当: P5):

- [x] Q1 — `crates/sim-quantum/src/schrodinger.rs::tests::q1_norm_is_conserved_to_near_machine_precision`。
      設計の目標abs 1e-12に対し実測abs<1e-10で確認(調和振動子ポテンシャル下、2000ステップ)
- [x] Q2 — `crates/sim-quantum/src/schrodinger.rs::tests::q2_free_wave_packet_spreading_matches_analytic_formula`
- [x] Q3 — `crates/sim-quantum/src/schrodinger.rs::tests::q3_infinite_well_eigenvalues_match_particle_in_a_box_formula`。
      虚時間発展+部分空間反復でn=1..5固有値を求め、rel<0.1%で確認
- [x] Q4 — `crates/sim-quantum/src/schrodinger.rs::tests::q4_harmonic_oscillator_eigenvalues_and_coherent_state_match_analytic`。
      固有値(虚時間発展、n=0..4)とコヒーレント状態(変位ガウス波束)の$\langle x\rangle(t)$の
      古典解一致(エーレンフェストの定理、実時間`step`を再利用)を両方rel<0.1%で確認
- [x] Q5 — `crates/sim-quantum/src/schrodinger.rs::tests::q5_tunneling_transmission_matches_energy_weighted_analytic_formula`。
      波束は単一エネルギーでないため素朴に$T(E_0)$と比較すると合わない(透過率がエネルギーの
      凸関数のため実測が系統的に大きくなる)ことに気づき、初期波束の運動量スペクトルで
      重み付けした解析式の期待値と比較。測定タイミングは障壁通過直後〜反射波束が周期境界を
      一周する前の安定プラトー(実測で確認)を使い、rel<2%で確認
- [ ] Q6

統計(S、担当: P4/P5):

- [x] S1 — `crates/sim-statistical/src/kinetic_gas.rs::tests::s1_speed_distribution_converges_to_maxwell_boltzmann`。
      同一速さ・ランダム方向で初期化しN2相当の剛体球衝突(数百回/粒子)で速さ分布を緩和、
      等確率ビンのχ²検定(有意水準1%)で確認
- [x] S2 — `crates/sim-statistical/src/kinetic_gas.rs::tests::s2_equation_of_state_matches_pv_equals_nkt`。
      希薄配置(φ≈0.0012)で壁への運動量移動から圧力を測定、rel<2%で確認
- [x] S3 — `crates/sim-statistical/src/kinetic_gas.rs::tests::s3_equipartition_holds_across_velocity_axes`。
      $3/\sqrt N$以内で確認
- [x] S4 — `crates/sim-statistical/src/brownian.rs::tests::s4_mean_squared_displacement_matches_6dt`。
      BAOABのA段(位置更新)離散化誤差がγΔt/mに強く依存することを実装検証中に発見
      (γΔt/m≈17で実測rel_err≈760%、≈0.17まで下げてrel_err<0.1%に収束)。O段(速度のOU
      厳密解)は大きなγΔt/mでも平衡速度分布を正確にサンプルするが、A段の精度は別問題
- [x] S5 — `crates/sim-statistical/src/brownian.rs::tests::s5_harmonic_trap_variance_matches_kbt_over_ktrap`
- [x] S6 — `crates/sim-statistical/src/brownian.rs::tests::s6_sedimentation_equilibrium_matches_boltzmann_height_distribution`。
      床(y=0)での弾性反射をテスト内で直接実装(コア API には境界条件の型を追加していない)。
      高度分布の平均 $k_BT/(mg)$ を rel 5% で検証。S5 と同じ発想で合成的に強めた重力加速度
      (g_eff=2000 m/s²)を使い、平衡到達スケールを縮めて自動テストを高速化
- [ ] S7(L=256 フル版は長時間級)
- [ ] S8(L=256 フル版は長時間級)
- [ ] S9

天体(A、担当: Pα):

- [x] A1 — `crates/sim-astro/src/nbody.rs::tests::a1_kepler_third_law_holds_across_orbital_scales`。
      実際の8惑星(水星88日〜海王星165年)は刻み解像良く高速テストするには非現実的なため、
      同一中心天体まわりの8合成衛星(幾何級数半径、周期比≈34倍)でT²∝a³を検証(法則自体は
      距離スケールに依らないため物理的に同等)。公転周期は線形補間したゼロ交差時刻で実測
- [x] A2(10⁶ 周フル版は長時間級のため縮約版(100周)で Green —
      `crates/sim-astro/src/nbody.rs::tests::a2_two_body_energy_and_angular_momentum_drift_stays_small_over_many_orbits`)
- [x] A3 — `crates/sim-astro/src/nbody.rs::tests::a3_circular_orbit_speed_matches_vis_viva_formula`
- [x] A4 — `crates/sim-astro/src/nbody.rs::tests::a4_hohmann_transfer_delta_v_matches_analytic_value`
- [ ] A5
- [ ] A6
- [x] A7 — `crates/sim-astro/src/nbody.rs::tests::a7_three_body_chaos_is_deterministic_across_runs`
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
