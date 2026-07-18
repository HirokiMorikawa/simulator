# 検証 01. 解析解テスト表 — 全ドメイン

実装フェーズの `cargo test` がそのまま実装すべきテスト仕様。各行は「設定 → 解析解 → 許容誤差」。
誤差の書式: rel = 相対、abs = 絶対。収束テスト(dt または h を半減して誤差比を確認)は ◆ 印。

## 0. TDD 運用規約(v2)

本エンジンは **テスト駆動開発(TDD)** で実装する([22-roadmap/01-phases.md](../22-roadmap/01-phases.md)):

1. **Phase A(Red)**: 実装前に、本表の全テスト + 各ドメイン §7 のユニットテスト + 保存則テスト
   ([02-conservation-laws.md](02-conservation-laws.md))を**先に記述**する。型/トレイトのスケルトンに対して
   コンパイルは通るが、中身は未実装(`todo!()`)なので**全テストが Red**。
2. **Phase B(Green)**: 依存順にドメインを実装し、対応するテストを Green にしていく。
   デモシナリオ([03-demo-scenarios.md](03-demo-scenarios.md))を**ドメイン内スモークテスト**として実装中に使う。
3. **Phase C(結合)**: ドメイン間結合・全体シナリオ・決定論 CI・保存則 CI を Green に。
4. 各テストは**シード固定・許容誤差明記**で、実行ごとにゆれない(決定論、[02-determinism-replay.md](../20-integration/02-determinism-replay.md))。

以下の表がその「先に書くテスト」の一覧である。

## 0.5 実装可能性監査の列定義

全行に「**担当ソルバ・担当積分器・想定実行時間**」を付記する。書かれるテストがすべて
仕様として成立していること(担当する道具が設計に存在し、CI の時間に乗ること)を、
Phase A の前提条件として本表で機械的に保証する。

- **担当ソルバ**: 行を Green にする責務を持つソルバ/モデル(仕様の正は各ドメイン文書)。
- **担当積分器**: [01-math/03-integrators.md](../01-math/03-integrators.md) のカタログ名。
  時間積分を含まない検算は「—(代数検算)」等と書く。
- **想定実行時間**: **秒級**(≲10 s)/ **分級**(≲5 min)/ **長時間級**の 3 区分。
- **長時間級ルール**: 物理的に CI に乗らない長時間級テスト(A2 二体 10⁶ 周の
  フル版、S7/S8 の L=256 フル基準など)は**通常 CI から外し、手動またはリリース前に実行する**。
  該当行は本表に長時間級と明記し、通常 CI には縮約版(規模を落とした同型テスト)を置く。
  [22-roadmap/02-feature-checklist.md](../22-roadmap/02-feature-checklist.md) §8 にも付記する。

## 力学

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| M1 | 自由落下 (h=10m) | $t^* = \sqrt{2h/g} = 1.4278$ s | rel 0.5% ◆(1次) | 剛体([10-mechanics/01](../10-mechanics/01-rigid-body.md)) | semi-implicit Euler | 秒級 |
| M2 | 斜方投射 45°(真空) | $R = v_0^2/g$ | rel 0.5% ◆ | 剛体(無衝突) | RK4(`BallisticIntegrator`) | 秒級 |
| M3 | 単振り子(小振幅) | $T = 2\pi\sqrt{L/g}$ | rel 1% | ヒンジ拘束([10-mechanics/05](../10-mechanics/05-joints-constraints.md)) | semi-implicit Euler | 秒級 |
| M4 | 単振り子(振幅 90°) | 楕円積分 $T = 4\sqrt{L/g}K(\sin^2 45°)$ | rel 1% | ヒンジ拘束 | semi-implicit Euler | 秒級 |
| M5 | 弾性衝突 1D(等質量) | 速度交換 | abs 1e-9 | 接触ソルバ([10-mechanics/03](../10-mechanics/03-contact-solver.md)、$e{=}1$) | semi-implicit Euler | 秒級 |
| M6 | 反発バウンド | 高さ比 $= e^2$ | rel 1%(split impulse) | 接触ソルバ(split impulse) | semi-implicit Euler | 秒級 |
| M7 | 斜面静止 | $\tan\theta < \mu_s$ で $v < 10^{-4}$ m/s | — | 接触 + 摩擦([10-mechanics/04](../10-mechanics/04-friction.md)) | semi-implicit Euler | 秒級 |
| M8 | 斜面滑走 | $a = g(\sin\theta - \mu_k\cos\theta)$ | rel 2% | 接触 + 摩擦 | semi-implicit Euler | 秒級 |
| M9 | 制動距離 | $d = v_0^2/(2\mu_k g)$ | rel 2% | 接触 + 摩擦 | semi-implicit Euler | 秒級 |
| M10 | こまの歳差 | $\dot\phi = mgr/(I\omega)$(速い自転極限) | rel 2% | 剛体回転(ジャイロ項) | quat 一次積分([01-math/03](../01-math/03-integrators.md) §4) | 秒級 |
| M11 | 中間軸不安定 | 摂動の指数成長率(オイラー方程式の線形化固有値) | rel 5% | 剛体回転 | quat 一次積分 + 陰的ジャイロ(検証モード) | 秒級 |
| M12 | スタック静止 | 4 段木箱 10 s: $v < 10^{-3}$、貫入 < slop | — | 接触ソルバ(スリープ) | semi-implicit Euler | 秒級 |
| M13 | カテナリー | ロープ静止形状 $y = a\cosh(x/a)$ | 最大偏差 2% | XPBD ロープ([10-mechanics/06](../10-mechanics/06-soft-body-particles.md)) | XPBD(位置ベース) | 秒級 |
| M14 | ロープの伸び | $\delta = WL/(EA)$ | rel 5% | XPBD ロープ | XPBD(位置ベース) | 秒級 |

## 流体

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| F1 | 終端速度(鋼球 1cm) | $v_t = \sqrt{2mg/(\rho C_d A)}$ | rel 1% | 抗力モデル([11-fluid/05](../11-fluid/05-aero-hydrodynamics.md)) | semi-implicit Euler | 秒級 |
| F2 | 雨滴 2mm | 6.5 m/s(Gunn-Kinzer 実測) | rel 5% | 抗力モデル | semi-implicit Euler | 秒級 |
| F3 | ストークス沈降 | $v = 2r^2\Delta\rho g/(9\mu)$ | rel 2% | 抗力モデル(ストークス域) | semi-implicit Euler | 秒級 |
| F4 | 立方体の喫水 | 密度比 × 辺長 | rel 1%(静水域)/5%(SPH) | 浮力([11-fluid/04](../11-fluid/04-free-surface-buoyancy.md))/ WCSPH 版([11-fluid/03](../11-fluid/03-sph.md)) | semi-implicit Euler / velocity Verlet(SPH) | 秒級 |
| F5 | 浮体の上下振動 | $T = 2\pi\sqrt{m/(\rho g A_{wl})}$ | rel 5% | 浮力 + 剛体 | semi-implicit Euler | 秒級 |
| F6 | 静水圧 | $p = \rho gh$ | rel 1% | 静水圧モデル([11-fluid/04](../11-fluid/04-free-surface-buoyancy.md)) | —(代数検算) | 秒級 |
| F7 | ポアズイユ流 | 放物型プロファイル | rel 2% ◆(h、2次) | 格子流体([11-fluid/02](../11-fluid/02-eulerian-grid.md)) | semi-Lagrangian + 陰的粘性(PCG) | 分級(h 4 水準) |
| F8 | Taylor-Green 渦 | 減衰率 $e^{-2\nu k^2t}$ | rel 5% | 格子流体 | semi-Lagrangian | 秒級 |
| F9 | 投影後発散 | $\nabla\cdot u = 0$ | abs 1e-6 | 格子流体(投影 PCG) | —(単段の検算) | 秒級 |
| F10 | ダム崩壊先端 | Martin-Moyce 1952 実測 | rel 10% | WCSPH([11-fluid/03](../11-fluid/03-sph.md)) | velocity Verlet | 分級 |
| F11 | カルマン渦 | $St \approx 0.2$ | rel 20% | 格子流体 | semi-Lagrangian | 分級 |

> **F11 注記(実装時確認)**: 実装時にまず 64³・渦度強化オフ・semi-Lagrangian で
> 渦離脱が自発的に立ち上がるかを**数値実験で確認**する。立ち上がらない場合は
> (i) 検証モードでも渦度強化オンを許容し、強化係数 $\varepsilon_{conf}$ を合格条件に記録する、
> または (ii) 解像度・レイノルズ数指定を変更する、のいずれかで合格条件を確定してから
> テストを Green にする([11-fluid/02](../11-fluid/02-eulerian-grid.md) §4.5)。
> 「合格基準が現象を消す」状態のまま Green を主張しない。

## 熱

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| T1 | ニュートン冷却 | $\tau = C/(hA)$ の指数減衰 | τ rel 1% ◆ | 熱ノード網([12-thermal/02](../12-thermal/02-heat-transfer.md)) | 陰的 Euler | 秒級 |
| T2 | 2 ノード平衡 | $T_{eq} = \frac{C_1T_1 + C_2T_2}{C_1+C_2}$ | abs 1e-9 | 熱ノード網 | 陰的 Euler | 秒級 |
| T3 | 1D 棒の過渡伝導 | フーリエ級数解 | rel 2% ◆(h) | 熱伝導格子([12-thermal/02](../12-thermal/02-heat-transfer.md)) | 陰的 Euler(PCG) | 秒級 |
| T4 | 放射平衡 | $T = (q/\varepsilon\sigma)^{1/4}$ | rel 2% | 熱ノード(放射項) | 陰的 Euler | 秒級 |
| T5 | 断熱圧縮 | $TV^{\gamma-1}$ 一定 | rel 1% | 気体区画([12-thermal/01](../12-thermal/01-thermodynamics-laws.md)) | —(準静的更新) | 秒級 |
| T6 | カルノー上限 | 効率 $\le 1 - T_c/T_h$(不等式) | 違反ゼロ | 気体区画(サイクル) | —(準静的更新) | 秒級 |
| T7 | 融解プラトー | プラトー長 $= mL_f/\dot Q$ | rel 1% | 相変化([12-thermal/03](../12-thermal/03-phase-change.md)、エンタルピー法) | 陰的 Euler | 秒級 |
| T8 | 沸点の気圧依存 | Antoine 式 | abs 1 °C | 相変化(Antoine 式) | —(代数検算) | 秒級 |

## 電磁気

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| E1 | クーロン力 | $kq_1q_2/r^2$ | 機械精度 | 静場直接和([13-electromagnetism/01](../13-electromagnetism/01-electrostatics-magnetostatics.md)) | —(力の代数検算) | 秒級 |
| E2 | サイクロトロン | $r = mv/(qB)$, 速さ一定 | rel 0.5% / abs 1e-9 | 静場 + ローレンツ力([13-electromagnetism/01](../13-electromagnetism/01-electrostatics-magnetostatics.md)・[05](../13-electromagnetism/05-em-mechanics-coupling.md)) | **Boris pusher**([01-math/03](../01-math/03-integrators.md) §2.6) | 秒級 |
| E3 | RC 過渡 | $\tau = RC$ | rel 0.5%(台形則)◆ | 回路 MNA([13-electromagnetism/02](../13-electromagnetism/02-circuits.md)) | 台形則 | 秒級 |
| E4 | RLC 減衰振動 | $\omega = \sqrt{1/LC - (R/2L)^2}$ | rel 1% | 回路 MNA | 台形則 | 秒級 |
| E5 | 分圧・分流 | 回路解析 | 機械精度 | 回路 MNA(DC) | —(線形解の検算) | 秒級 |
| E6 | モーター無負荷/ストール | $\omega = V/k$, $\tau = kV/R_a$ | rel 1% | MNA + モーター結合([13-electromagnetism/05](../13-electromagnetism/05-em-mechanics-coupling.md)) | 後退 Euler/台形則 + semi-implicit Euler | 秒級 |
| E7 | 誘導起電力 | $\mathcal{E} = B\ell v$ | rel 0.5% | 誘導棒結合([13-electromagnetism/05](../13-electromagnetism/05-em-mechanics-coupling.md)) | semi-implicit Euler + MNA | 秒級 |
| E8 | FDTD 伝播速度 | $c$ | rel 0.5%(20 cell/λ) | FDTD([13-electromagnetism/03](../13-electromagnetism/03-maxwell-fdtd.md)) | leapfrog(Yee) | 秒級 |
| E9 | フレネル垂直反射 | $((n_1{-}n_2)/(n_1{+}n_2))^2$ | rel 1% | 光学([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md)) | —(レイ・代数検算) | 秒級 |
| E10 | スネル・臨界角 | $n_1\sin\theta_1 = n_2\sin\theta_2$ | 機械精度 | 光学 | —(レイ・代数検算) | 秒級 |
| E11 | 薄レンズ焦点 | $1/f = (n{-}1)(1/R_1 - 1/R_2)$ | rel 1%(近軸) | 光学(近軸レイ) | —(レイ・代数検算) | 秒級 |
| E12 | プリズム最小偏角 | $n = \sin\frac{A+\delta_m}{2}/\sin\frac{A}{2}$ | rel 0.5% | 光学(分光レイ) | —(レイ・代数検算) | 秒級 |
| E13 | 空洞共振 | $f_{mn} = \frac{c}{2}\sqrt{(m/a)^2 + (n/b)^2}$ | rel 1% | FDTD | leapfrog(Yee) | 分級(共振スペクトル取得) |

> **E2 注記**: 「速さ一定 abs 1e-9」は磁場回転を厳密にノルム保存する
> **Boris pusher** が構造的に満たす。semi-implicit Euler は磁場回転で速さが系統的に増大する
> ため本行には使わない。荷電粒子・帯電点質量の運動積分は Boris を標準とする
> ([13-electromagnetism/01](../13-electromagnetism/01-electrostatics-magnetostatics.md) §4・[05](../13-electromagnetism/05-em-mechanics-coupling.md) §2.1)。

## 量子

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| Q1 | ノルム保存 | $\int \lvert\psi\rvert^2 = 1$ | abs 1e-12 | シュレディンガー([14-quantum/02](../14-quantum/02-schrodinger-solver.md)) | split-step Fourier | 秒級 |
| Q2 | 波束の広がり | $\sigma(t)$ 解析式 | rel 0.1% ◆(2次) | シュレディンガー | split-step Fourier | 秒級 |
| Q3 | 井戸固有値 n=1..5 | $E_n = n^2\pi^2\hbar^2/(2mL^2)$ | rel 0.1% | シュレディンガー | split-step Fourier | 秒級 |
| Q4 | 調和振動子 | $E_n = \hbar\omega(n+\frac12)$、コヒーレント状態の $\langle x\rangle$ | rel 0.1% | シュレディンガー | split-step Fourier | 秒級 |
| Q5 | トンネル透過率 | 矩形障壁解析式(エネルギー重み平均) | rel 2% | シュレディンガー | split-step Fourier | 秒級 |
| Q6 | 二重スリット縞間隔 | $\Delta y = \lambda_{dB}D/d$ | rel 2% | シュレディンガー(2D) | split-step Fourier | 分級(2D 格子) |

## 統計

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| S1 | MB 分布収束 | $\chi^2$ 適合(有意水準 1%) | 合格 | 気体分子([15-statistical/02](../15-statistical/02-kinetic-gas.md)) | velocity Verlet | 分級 |
| S2 | 状態方程式 | $pV = Nk_BT$ | rel 2%(N=10⁴) | 気体分子 | velocity Verlet | 分級 |
| S3 | 等分配 | $\langle v_x^2\rangle = \langle v_y^2\rangle = \langle v_z^2\rangle$ | $3/\sqrt N$ 内 | 気体分子 | velocity Verlet | 分級 |
| S4 | MSD | $\langle\Delta x^2\rangle = 6Dt$ | rel 3% | ランジュバン([15-statistical/03](../15-statistical/03-diffusion-brownian.md)) | BAOAB | 秒級 |
| S5 | 調和トラップ分散 | $k_BT/k_{trap}$ | rel 2% | ランジュバン | BAOAB | 秒級 |
| S6 | 沈降平衡 | $c(h) \propto e^{-mgh/k_BT}$ | rel 5% | ランジュバン | BAOAB | 分級 |
| S7 | イジング $T_c$ | 2.269 J/k_B | rel 2%(L=256 フル)/ rel 5%(L=64 縮約) | モンテカルロ(**Metropolis + Wolff 必須**、[15-statistical/04](../15-statistical/04-monte-carlo.md)) | —(MC サンプリング) | L=64 縮約: 分級(通常 CI)/ L=256 フル: **長時間級** |
| S8 | イジング自発磁化 | Onsager 厳密解 | rel 2%(L=256 フル)/ rel 5%(L=64 縮約) | モンテカルロ(**Metropolis + Wolff 必須**) | —(MC サンプリング) | L=64 縮約: 分級(通常 CI)/ L=256 フル: **長時間級** |
| S9 | 小系の詳細釣り合い | 4×4 厳密分配関数 | rel 1% | Metropolis(4×4 厳密照合) | —(MC サンプリング) | 秒級 |

> **S7/S8 注記**: 臨界域の L=256 は単スピン Metropolis では臨界減速により
> 現実的な時間で緩和しないため、**Wolff クラスタ法を S7/S8 を担う必須実装**とする
> ([15-statistical/04](../15-statistical/04-monte-carlo.md) §4)。通常 CI には L=64 の縮約版
> (許容は有限サイズ効果込みで rel 5%)を置き、L=256 のフル基準は長時間級
> (手動/リリース前実行)とする。

## 天体

| # | テスト | 解析解 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| A1 | ケプラー第3法則 | $T^2 \propto a^3$(太陽系 8 惑星) | rel 0.1% | N 体([16-astro/01](../16-astro/01-gravitation-nbody.md)) | leapfrog(シンプレクティック) | 秒級 |
| A2 | 二体エネルギー/角運動量保存 | 10⁶ 周のドリフト | rel 1e-6(シンプレクティック) | N 体 | leapfrog(シンプレクティック) | 10⁴ 周縮約: 分級(通常 CI)/ 10⁶ 周フル: **長時間級** |
| A3 | 円軌道速度 | $v = \sqrt{GM/r}$ | rel 0.1% | N 体 | leapfrog(シンプレクティック) | 秒級 |
| A4 | ホーマン遷移 | $\Delta v$ 解析値 | rel 0.5% | 軌道力学([16-astro/02](../16-astro/02-orbital-mechanics.md)) | leapfrog(シンプレクティック) | 秒級 |
| A5 | $J_2$ 歳差 | 昇交点の歳差率 | rel 2% | 軌道摂動($J_2$) | leapfrog / WHFast | 分級 |
| A6 | 大気減衰 | 低軌道の高度減衰傾向 | 定性 + 弾道係数依存 | 再突入(大気抗力、[16-astro/02](../16-astro/02-orbital-mechanics.md)) | leapfrog + 抗力 | 秒級 |
| A7 | 三体カオス決定論 | 同一初期条件→同一軌道 | 厳密一致 | N 体(三体) | leapfrog(シンプレクティック) | 秒級 |
| A8 | 水星近日点移動(1PN) | 42.98″/世紀 | rel 1% | 1PN 補正([16-astro/03](../16-astro/03-relativistic-corrections.md)) | leapfrog + 1PN 補正 | 分級 |
| A9 | GPS 時間差(1PN) | +38.6 μs/日 | rel 1% | 1PN 時計モデル | —(解析積算) | 秒級 |
| A10 | 光の重力偏向(1PN) | 太陽縁 1.75″ | rel 2% | 1PN 光線偏向 | —(積分公式) | 秒級 |

> **A2 注記**: 10⁶ 周のフル版は長時間級(通常 CI 外・手動/リリース前実行)。
> 通常 CI には 10⁴ 周の縮約版を置き、ドリフトが周回数に対して線形外挿で
> rel 1e-6/10⁶ 周相当以下であることを確認する。

## レンダリング(Phase D)

| # | テスト | 解析解/参照 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| R1 | 白色炉テスト | 完全拡散面が背景輝度と一致 | rel 0.1%(エネルギー保存) | パストレーサ([17-rendering/02](../17-rendering/02-path-tracing.md)) | —(MC 積分) | 分級 |
| R2 | フルネル反射率 | 解析式(誘電体・金属) | rel 1% | パストレーサ(BSDF) | —(MC 積分) | 秒級 |
| R3 | 分光/屈折 | プリズム最小偏角・虹の分散 | rel 0.5% | パストレーサ(分光) | —(MC 積分) | 分級 |
| R4 | コーネルボックス | 参照解(color bleeding) | 収束一致 | パストレーサ | —(MC 積分) | 分級(低解像度) |
| R5 | 大気レイリー | $\lambda^{-4}$ 依存(空の青) | 定性 + 波長比 | パストレーサ(大気散乱) | —(MC 積分) | 分級 |
| R6 | 被写界深度 | 錯乱円径 = 薄レンズ公式 | rel 2% | パストレーサ(物理カメラ) | —(MC 積分) | 秒級 |
| R7 | モンテカルロ収束 | ノイズ $O(1/\sqrt N)$・同一シード同一画像 | 厳密一致 | パストレーサ | —(MC 積分) | 分級 |

## 結合 — stiff 検出

弱結合の 1 ステップ遅れが破綻する既知の stiff な組に対する検出テスト。
sub-iteration 回数の決定的算出規則([20-integration/01](../20-integration/01-coupling-matrix.md) §2 規則 3)が
正しく段階選択することを、発振・発散の既知ケースで確認する。

| # | テスト | 基準 | 許容 | 担当ソルバ | 担当積分器 | 想定実行時間 |
|---|---|---|---|---|---|---|
| X1 | 無慣性ロータ × 回路 | 微小慣性ロータ(10⁻⁹ kg·m²)+ モーター結合 10 s で $\omega$・$i$ が有界、定常値は解析値(無負荷回転数)に収束 | 発散ゼロ・定常 rel 2% | MotorCoupling + sub-iteration 規則([20-integration/01](../20-integration/01-coupling-matrix.md) §2) | 後退 Euler/台形則 + semi-implicit Euler | 秒級 |
| X2 | 軽剛体 × 解像流体 | 密度比 0.1 の軽箱を 64³ 格子流体中で解放 10 s: 加速度の符号反転頻度が物理振動(浮体固有周期)の 2 倍以下(数値発振なし)、発散なし | 発振検知ゼロ | GridFluidRigid + sub-iteration 規則 | semi-implicit Euler + semi-Lagrangian | 分級 |

## 数学基盤(抜粋 — 各文書 §6/§7 の集約)

- 線形代数の恒等式(abs 1e-12)、積分器の収束次数 ◆、PCG の製造解収束、
  PRNG 参照ベクタ一致、補間の多項式再現。

## 収束次数テスト(◆)の実装規約

$\Delta t$(または $h$)を 4 水準(1, 1/2, 1/4, 1/8)で実行し、誤差の対数勾配が
公称次数 ± 0.3 に入ることを確認する。これにより「たまたま許容内」でなく
「正しい離散化」であることを保証する。
