# 横断 01. 結合行列 — ドメイン間相互作用の全景

crate: `sim-coupling`。全ドメイン間の「誰が誰に何を渡すか」の正典。
各ドメイン文書の第 6 節はこの表の詳細化であり、矛盾した場合は本文書を修正の起点とする。

## 1. 結合行列

行 = 送り手、列 = 受け手。各セルは(渡す量 → 実装フェーズ)。空欄 = 結合なし。

| ↓送り \ 受け→ | 力学 | 流体 | 熱 | 電磁気 | 量子 | 統計 | 天体 |
|---|---|---|---|---|---|---|---|
| **力学** | — | 障害物境界・剛体表面速度 → P3 / SPH境界粒子 → P4 | 摩擦・衝突・抗力の散逸熱 → P1 / ピストン体積 → P3 | 導体の運動(誘導起電力)→ P4 / スイッチ操作 → P4 | 幾何共有のみ | ピストン(分子との運動量交換)→ P5 | 宇宙機の推力→軌道・姿勢 → Pα |
| **流体** | 浮力・抗力・揚力・圧力積分 → P1(集中)/P3(解像) | — | 対流熱伝達(h と流速)→ P3 / 移流(温度場)→ P3 | (対象外: MHD) | | 輸送係数の検証対象 → P5 | 大気抗力→軌道減衰・再突入 → Pα |
| **熱** | (将来: 熱膨張) | Boussinesq 浮力(温度場→運動量)→ P3 / 温度依存粘性 → P5 | — | 温度依存抵抗 R(T) → P5 | 温度→アレニウス速度 → P5 | 温度→ブラウン強度・熱壁 → P4/P5 | (再突入体の温度は天体←熱) |
| **電磁気** | ローレンツ力・モータートルク・磁気力 → P4 | (対象外) | ジュール熱 I²R・放電熱 → P4 | — | (バンド由来素子の式は量子→EM) | | (GPS 時刻←相対論) |
| **量子** | 物性の由来(値は実測DB) | 同左 | 比熱・反応熱の由来 | ダイオード式・輝線・プランク則 → P4/P5 | — | ボルツマン因子の共有 | (相対論補正の理論的背景) |
| **統計** | ブラウン力(微小体)→ P4 | 輸送係数の由来 | 温度・エントロピーの意味 / 分子→熱力学の実演 → P5 | | 測定統計の頻度解釈 | — | |
| **天体** | 局所重力場(一様 g の一般化)・潮汐力 → Pα | 高度依存大気プロファイルの供給 → Pα | 太陽輻射フラックス・空力加熱(再突入)→ Pα | (地磁気の位置依存)・光の重力偏向(相対論)→ Pα | 相対論オプトインの適用 | — | — |

「由来」系(量子・統計の行)はデータ・ドキュメント供給であり実行時結合ではない
([00-foundation/02](../00-foundation/02-scale-ladder.md) §2.1)。

**レンダリング**([17-rendering/](../17-rendering/), Phase D)は光学ドメイン([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md))の
スペクトル・屈折率・放射と、熱の黒体色、天体の位置・大気散乱・光の重力偏向(相対論オプトイン)を
**消費する**(一方向、状態は変えない)。物理計算には影響しない。

**実装フェーズ表記**: P1–P5 は Phase B 内の実装ウェーブ(旧フェーズ番号、実装順を維持)を指す
([22-roadmap/01-phases.md](../22-roadmap/01-phases.md) §フェーズ対応)。**Pα** = 天体ウェーブ(Phase B 末尾)。
レンダリングは **Phase D**。ドメイン各文書 §8 の「Phase 1〜5」表記も同じ Phase B 内ウェーブを指す。

## 2. 実行時結合の設計規則

1. **保存量の橋は対で記帳**: エネルギー・運動量を渡す結合は、取り出しと注入を同一
   Coupling 実装内で行い、EnergyLedger([12-thermal/01](../12-thermal/01-thermodynamics-laws.md) §4)に
   差分ゼロを検算させる。
2. **排他結合の明示**: 同じ物理を 2 経路で計算しない。
   - 浮力: 静的水域モデル XOR SPH/格子(シーン単位で選択、[11-fluid/04](../11-fluid/04-free-surface-buoyancy.md) §4.1)
   - 空気抗力: 集中定数 XOR 格子結合([11-fluid/05](../11-fluid/05-aero-hydrodynamics.md) §6)
   - コンデンサ電場エネルギー: 回路 XOR 静電場([13-electromagnetism/01](../13-electromagnetism/01-electrostatics-magnetostatics.md) §6)
3. **弱結合(operator splitting)が既定**: 各ステップで前ステップ確定値を読む
   ([00-foundation/04](../00-foundation/04-architecture.md) §1.3 の pre/post 2 相)。
   sub-iteration 化の判定と反復回数は、**実行時の収束判定ではなく状態からの決定的算出**で
   決める(反復数もリプレイ可能にするため):
   - 各結合対でステップ冒頭に**剛性指標** $\kappa = c_{\text{coupling}}\,\Delta t / m_{\text{eff}}$
     (結合係数 × dt / 実効慣性。結合種別ごとに定義式を実装時に固定)を計算し、
     **閾値表で反復回数を段階選択**する:
     $\kappa < 1$: 1 回(弱結合のまま)/ $1 \le \kappa < 10$: 2 回 / $10 \le \kappa < 100$: 4 回 /
     $\kappa \ge 100$: 8 回(**上限固定**、超過時は診断イベント + dt 側の対処を促す)。
   - 閾値表・回数はコンパイル時定数(シーン設定で上書き可、実行中は不変)。壁時計・収束測定に
     依存しないため、決定論(同一入力 → 同一反復数)が保たれる
     ([02-determinism-replay.md](02-determinism-replay.md) §2 の適応的アルゴリズム規約)。
   - 既知の要注意組(軽い剛体 × 解像流体(密度比 < 0.3)、無慣性ロータ × 回路)には
     **stiff 検出テスト**(発振・発散の既知ケース)を解析解テスト表 X1/X2 に置き、
     「兆候が出たら対処」を「このテストが Red になったら対処」に置き換える
     ([21-verification/01](../21-verification/01-analytic-tests.md))。
4. **時間スケール分離**: 独立時間軸(FDTD・量子)は結合しない。sub-stepping の倍率は
   固定または安定条件からの決定的算出のみ(適応刻みの非決定性を排除)。

## 3. Coupling 実装の一覧(Rust)

```rust
// sim-coupling が提供する実装 (Phase 順)
pub struct DissipationToHeat;      // P1: 摩擦・衝突・抗力散逸 → ThermalNode (熱浸透率比分配)
pub struct BuoyancyDrag;           // P1: 静的媒質 → 剛体力 (浮力・抗力・揚力)
pub struct GridFluidRigid;         // P3: 格子流体 ⇔ 剛体 (ボクセル化境界・圧力積分)
pub struct ConvectionLink;         // P3: 流体/媒質 ⇔ ThermalNode (相関式 h)
pub struct BoussinesqBuoyancy;     // P3: 温度場 → 流体運動量
pub struct PistonGas;              // P3: GasCompartment ⇔ Sliderジョイント
pub struct MotorCoupling;          // P4: 回路 ⇔ ヒンジ ⇔ 熱 [13-em/05]
pub struct LorentzForce;           // P4: 静場 → 帯電剛体
pub struct JouleHeat;              // P4: 回路素子 → ThermalNode
pub struct BrownianForce;          // P4: 温度・粘性 → 微小剛体のランダム力
pub struct SphRigid;               // P4: SPH ⇔ 剛体 (境界粒子)
pub struct InductionCoupling;      // P4: 導体棒・渦電流
pub struct PhaseChangeMorph;       // P3: 融解 → 剛体消滅/流体生成イベント
```

## 4. 実行順序(1 world step 内)

[00-foundation/04](../00-foundation/04-architecture.md) §1.3 のパイプラインに結合を配置:

```
Coupling(pre):  BuoyancyDrag, LorentzForce, BrownianForce, BoussinesqBuoyancy,
                MotorCoupling(電気→トルク), PistonGas(圧力→力)
Solvers:        mechanics → fluid, thermal → em(回路), (専用シーン: quantum, statistical)
Coupling(post): DissipationToHeat, JouleHeat, ConvectionLink,
                GridFluidRigid(新位置→境界), MotorCoupling(ω→逆起電力),
                PistonGas(新体積), PhaseChangeMorph(イベント)
```

順序は固定・文書化(決定論の一部)。pre は「力を作る」、post は「結果を配る」。

## 5. 検証(結合固有)

- 各 Coupling 単体のエネルギー・運動量収支テスト(ドメイン文書の §7 に個別基準)。
- **統合シナリオテスト**(複数結合の連鎖):
  1. ブレーキ: 運動 → 摩擦熱 → 温度上昇 → (P5: 抵抗変化)。台帳 residual < 10⁻³。
  2. 手回し発電: 機械仕事 → 電気 → ジュール熱 + 光(効率の収支)。
  3. 氷と飲み物: 熱伝達 + 相変化 + 浮力(質量変化)の同時進行。
  4. 断熱圧縮: ピストン仕事 = 内部エネルギー増(± 1%)。
- 排他結合の静的検査: シーンロード時に二重計上の組み合わせを拒否する validator。
