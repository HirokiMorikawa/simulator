# 03. 単位系・座標系・規約

全ドメイン文書と将来の実装が従う共通規約。ここに反する定義はどの文書にも作らない。

## 1. 単位系: SI(国際単位系)

内部表現はすべて SI 基本単位・組立単位とする。スケーリング(例: cm 単位への正規化)は行わない。
f64 の相対精度($\approx 2.2\times10^{-16}$)なら、$10^{-10}$ m(原子)〜 $10^{3}$ m(街区)の範囲で桁落ちは実用上問題にならない。
量子ドメインのみ例外を認める(後述 §1.2)。

| 量 | 単位 | 記号 |
|---|---|---|
| 長さ | メートル | m |
| 質量 | キログラム | kg |
| 時間 | 秒 | s |
| 温度 | ケルビン | K |
| 電流 | アンペア | A |
| 物質量 | モル | mol |
| 力 | ニュートン | N = kg·m/s² |
| エネルギー | ジュール | J = N·m |
| 仕事率 | ワット | W = J/s |
| 圧力 | パスカル | Pa = N/m² |
| 電荷 | クーロン | C = A·s |
| 電位 | ボルト | V = J/C |

**UI 表示のみ**、ユーザー向けに摂氏(°C)や km/h 等への換算を許す。内部は常に SI。

### 1.1 物理定数(CODATA 2018 / SI 定義値)

実装では単一の定数モジュール(`constants.rs` 相当)に集約し、値の重複定義を禁止する。

| 定数 | 記号 | 値 | 備考 |
|---|---|---|---|
| 標準重力加速度 | $g_0$ | $9.80665\ \mathrm{m/s^2}$ | 定義値。World デフォルト |
| 真空中の光速 | $c$ | $299\,792\,458\ \mathrm{m/s}$ | 定義値 |
| プランク定数 | $h$ | $6.62607015\times10^{-34}\ \mathrm{J\,s}$ | 定義値 |
| 換算プランク定数 | $\hbar$ | $1.054571817\times10^{-34}\ \mathrm{J\,s}$ | $h/2\pi$ |
| ボルツマン定数 | $k_B$ | $1.380649\times10^{-23}\ \mathrm{J/K}$ | 定義値 |
| アボガドロ定数 | $N_A$ | $6.02214076\times10^{23}\ \mathrm{mol^{-1}}$ | 定義値 |
| 気体定数 | $R$ | $8.314462618\ \mathrm{J/(mol\,K)}$ | $N_A k_B$ |
| 電気素量 | $e$ | $1.602176634\times10^{-19}\ \mathrm{C}$ | 定義値 |
| 真空の誘電率 | $\varepsilon_0$ | $8.8541878128\times10^{-12}\ \mathrm{F/m}$ | CODATA 2018 |
| 真空の透磁率 | $\mu_0$ | $1.25663706212\times10^{-6}\ \mathrm{N/A^2}$ | CODATA 2018 |
| シュテファン=ボルツマン定数 | $\sigma$ | $5.670374419\times10^{-8}\ \mathrm{W/(m^2K^4)}$ | 導出定義値 |
| 電子質量 | $m_e$ | $9.1093837015\times10^{-31}\ \mathrm{kg}$ | CODATA 2018 |
| 標準大気圧 | $p_0$ | $101\,325\ \mathrm{Pa}$ | 定義値 |
| 空気密度(15 °C, 1 atm) | $\rho_{air}$ | $1.225\ \mathrm{kg/m^3}$ | ISA 標準大気 |
| 水の密度(20 °C) | $\rho_{water}$ | $998.2\ \mathrm{kg/m^3}$ | CRC Handbook |

### 1.2 量子ドメインの単位の扱い

シュレディンガーソルバ([14-quantum/02-schrodinger-solver.md](../14-quantum/02-schrodinger-solver.md))では
SI のままだと $\hbar \sim 10^{-34}$ が絡む極端な指数になるため、**原子単位系 (Hartree atomic units)**
($\hbar = m_e = e = 4\pi\varepsilon_0 = 1$)での内部計算を認める。ただし:

- 変換は量子ソルバの境界(入出力)でのみ行い、変換係数は定数モジュールに置く。
- 他ドメインに原子単位の値を漏らさない。

## 2. 座標系・幾何規約

- **右手系、Y-up**: $+x$ 東、$+y$ 鉛直上向き、$+z$ 南(画面手前)。重力はデフォルトで $(0, -g_0, 0)$。
- **角度はラジアン**。角速度は rad/s。
- **回転の表現は単位クォータニオン** $q = (x, y, z, w)$、$w$ が実部。回転行列・オイラー角は入出力変換のみ。
- クォータニオンの回転は **ベクトルを回す (active rotation)**: $\mathbf{v}' = q\,\mathbf{v}\,q^{-1}$。
- 外積は右手系: $\hat{x} \times \hat{y} = \hat{z}$。
- 法線は「A から B へ」など、各 API で向きを必ず文書化する(接触法線は A→B と統一)。
- 平面は $\hat{n}\cdot\mathbf{x} = d$ で表す($\hat{n}$ 単位法線、$d$ は原点からの符号付き距離)。

## 3. 時間の規約

- シミュレーション時間 $t$ は World が管理する f64 の秒。壁時計(実時間)とは独立。
- **固定タイムステップ**。基本ステップ $\Delta t = 1/120$ s(デフォルト)。ドメインごとの sub-stepping は
  [20-integration/01-coupling-matrix.md](../20-integration/01-coupling-matrix.md) で規定する。
- コアは `Date.now()` / OS 時計 / `Math.random` 相当を参照しない(決定論、[20-integration/02-determinism-replay.md](../20-integration/02-determinism-replay.md))。

## 4. 数値の規約

- 物理量はすべて **f64**。f32 は描画層への転送でのみ使用可。
- `NaN` / `Inf` はバグとして扱う。デバッグビルドでは主要ループにアサーションを置く。
- 許容誤差の書き方: 相対誤差 $\epsilon_{rel}$ と絶対誤差 $\epsilon_{abs}$ を区別して明記する。
  比較は $|a-b| \le \epsilon_{abs} + \epsilon_{rel}\max(|a|,|b|)$。
- ゼロ除算防止の $\epsilon$(例: 正規化時の最小長)は名前付き定数にし、マジックナンバーを禁止。

## 5. 記号の規約(数式)

| 記号 | 意味 | | 記号 | 意味 |
|---|---|---|---|---|
| $\mathbf{x}$ | 位置 | | $\rho$ | 密度 |
| $\mathbf{v}, \mathbf{u}$ | 速度(剛体 / 流体場) | | $p$ | 圧力 |
| $q$ | 回転クォータニオン | | $T$ | 温度 |
| $\boldsymbol{\omega}$ | 角速度 | | $\mu$ | 摩擦係数 or 粘性係数(文脈で明記) |
| $m$ | 質量 | | $e$ | 反発係数 or 電気素量(文脈で明記) |
| $\mathbf{I}$ | 慣性テンソル | | $\mathbf{E}, \mathbf{B}$ | 電場・磁場 |
| $\mathbf{F}$ | 力 | | $\psi$ | 波動関数 |
| $\boldsymbol{\tau}$ | トルク | | $S$ | エントロピー or 作用(文脈で明記) |
| $\mathbf{j}$ | 撃力(インパルス) | | $\Delta t$ | タイムステップ |

多義的な記号($\mu$, $e$, $S$)は各文書の冒頭または初出で意味を宣言する。

## 6. 命名規約(Rust)

- crate / module: `snake_case`。型: `UpperCamelCase`。定数: `SCREAMING_SNAKE_CASE`。
- 物理量のフィールド名は完全な英単語: `linear_velocity`(`vel` 不可)、`angular_velocity`、`temperature`。
- 単位はフィールド名に含めない(すべて SI なので冗長)。ただし SI でない境界(UI・原子単位)では
  `temperature_celsius` のように必ず明記する。
- ID 型は newtype: `struct BodyId(u32);` — 生の整数の取り違えをコンパイル時に防ぐ。
- ドメイン crate 名は `sim-mechanics`, `sim-fluid`, `sim-thermal`, `sim-em`, `sim-quantum`, `sim-statistical`,
  共通基盤は `sim-math`, `sim-core`([05-rust-wasm-platform.md](05-rust-wasm-platform.md))。

## 7. 文書規約

[docs/README.md](../README.md) の「文書規約」節を正とする(統一 9 節フォーマット、LaTeX 数式、出典必須)。
