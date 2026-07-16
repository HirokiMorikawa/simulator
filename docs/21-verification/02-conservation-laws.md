# 検証 02. 保存則チェック — エネルギー・運動量・質量・電荷

解析解が無い複雑なシーンでも常に検査できる**普遍の検算**。実装は EnergyLedger
([12-thermal/01](../12-thermal/01-thermodynamics-laws.md) §4)と World の集計 API
([20-integration/04](../20-integration/04-world-api.md) §2)。

## 1. 保存量と検査条件

| 保存量 | 集計 | 保存が成立する条件 | 検査 |
|---|---|---|---|
| エネルギー(全形態) | $E_{kin} + E_{pot} + E_{elastic} + E_{thermal} + E_{em} + E_{chem} - W_{injected}$ | 常に(第 1 法則) | residual の監視(§2) |
| 運動量 $\mathbf{P}$ | $\sum m\mathbf{v}$(+流体・粒子) | 外力なし(重力・静的地形なし) | 周期境界・自由空間シーンで abs 1e-9 rel |
| 角運動量 $\mathbf{L}$ | $\sum(\mathbf{r}\times m\mathbf{v} + \mathbf{I}\boldsymbol\omega)$ | 外トルクなし | 同上(ジャイロ陽積分のドリフト率は文書化 [10-mechanics/01] §7) |
| 質量 | 剛体・粒子・相変化の総和 | 常に(蒸発は台帳に移動) | abs 機械精度 |
| 電荷 | $\sum q$ + 回路のノード電荷 | 常に(放電も移動のみ) | abs 機械精度 |
| 確率(量子) | $\int|\psi|^2$ | 吸収境界なし | abs 1e-12 |
| 第 2 法則 | エントロピー生成 $\ge 0$ | 孤立系設定 | 各熱流で violation ゼロ |

## 2. エネルギー residual の運用

$$\text{residual}(t) = \frac{|E_{total}(t) - E_{total}(0) - W_{injected}(t)|}{\max(E_{scale}, |E_{total}(0)|)}$$

- $E_{scale}$: シーンの代表エネルギー(ゼロ初期エネルギー対策)。
- **閾値の設計**(数値誤差の既知源を積み上げて設定):
  - 力学のみ(接触あり): 1e-3 / 分(Baumgarte の偽仕事が主因。split impulse で 1e-5 目標)
  - +流体(semi-Lagrangian): 1e-2 / 分(移流散逸 — これは「熱に変換されない運動エネルギー損失」
    として residual に現れる。**既知の近似**として台帳に `numerical_dissipation` 項目を設け、
    residual と区別して表示する)
  - 熱・回路のみ: 1e-6 / 分(陰解法・小系)
  - 分子動力学(Verlet・保存系): 1e-6 相対 / 10⁶ step
- CI: 全デモシナリオを既定長(60 s 相当)実行し、閾値超過を fail とする。

## 3. 変換の対応表(記帳の正しさの検査)

各結合のエネルギー移動が「出所と行き先」で二重に集計され、一致すること:

| 変換 | 出所 | 行き先 |
|---|---|---|
| 摩擦・非弾性衝突 | kinetic 減少 | thermal 増加(接触イベント経由) |
| 空気抗力(集中定数) | kinetic | thermal(媒質ノード or numerical_dissipation) |
| ジュール熱 | em(電池 chem) | thermal |
| モーター | em | kinetic(+ thermal 損失) |
| 相変化 | thermal(顕熱) | thermal(潜熱、enthalpy 内訳) |
| 蒸発 | thermal | 質量+潜熱の系外移動(台帳の系外項) |
| ピストン | kinetic/injected | 気体内部エネルギー |

テスト形式: 各変換につき「単離シーン」(その変換だけが起きる最小構成)で
移動量の一致を機械精度〜1e-6 で確認する。

## 4. 対称性との関係(ドキュメント)

ネーターの定理 — エネルギー保存 ⇔ 時間並進対称、運動量 ⇔ 空間並進、角運動量 ⇔ 回転対称。
「地面(静的ボディ)があると運動量が保存しない」のは対称性の破れとして正しい挙動であり、
バグではない — 検査条件の欄(§1)はこの理解に基づく。UI の保存量パネルにも
「このシーンで保存するはずの量 / しない量」を対称性から自動判定して表示する
(静的ボディ・重力・熱浴の有無で決まる)。

## 5. 実装フェーズ対応

Phase 1 から EnergyLedger と P/L 集計を実装し、以後の全フェーズで検査対象を拡張する
(新ドメイン追加 = 台帳項目と変換対応表への行追加、[20-integration/01](../20-integration/01-coupling-matrix.md) §2.1)。
