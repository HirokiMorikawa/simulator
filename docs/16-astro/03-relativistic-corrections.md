# 天体 03. 相対論補正 — オプトイン Post-Newton 展開

crate: `sim-astro`。「地球の物理法則の忠実な再現」を完備するための、**既定オフ・オプトイン**の
一般相対論効果。日常・多くの天体シーンでは無視でき、特定の検証シーンでのみ有効化する。

## 1. 担う現実の現象

水星の近日点移動、GPS 衛星の時刻補正(相対論を入れないと 1 日 ~11 km ずれる)、
太陽縁での星の光の曲がり(1919 年エディントン)、重力赤方偏移、シャピロ遅延。
遊び方の例: 相対論 ON/OFF で水星の近日点がどれだけ回るか比べる、GPS の測位誤差の起源を見る。

## 2. 支配方程式

完全な一般相対論(アインシュタイン方程式)は解かず、弱場・低速展開である
**Post-Newton(PN)近似**を使う(太陽系は $GM/(rc^2) \sim 10^{-8}$、$v/c \sim 10^{-4}$ で PN が高精度)。

### 2.1 1PN 運動方程式(EIH 方程式の簡約)

主星まわりの 1 体の 1PN 加速度補正(Schwarzschild 項):

$$\mathbf{a}_{1PN} = \frac{GM}{c^2 r^2}\left[\left(4\frac{GM}{r} - v^2\right)\hat{\mathbf{r}} + 4(\mathbf{v}\cdot\hat{\mathbf{r}})\mathbf{v}\right]$$

これをニュートン加速度に加える。近日点移動率(解析): $\Delta\varpi = \frac{6\pi GM}{c^2 a(1-e^2)}$ /周。
水星で 42.98″/世紀(観測 43″)。

### 2.2 時間の遅れ(GPS)

固有時と座標時の比(重力ポテンシャル $\Phi$ と速度):

$$\frac{d\tau}{dt} \approx 1 + \frac{\Phi}{c^2} - \frac{v^2}{2c^2}$$

GPS 衛星(高度 20200 km)では重力効果(時計が速く進む、+45.7 μs/日)と
速度効果(遅れる、−7.1 μs/日)の差し引き +38.6 μs/日。これを時計ドメイン(観測量)に反映。

### 2.3 光の曲がり・シャピロ遅延

光線([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md))が質量近傍を通るときの偏角
$\delta = 4GM/(c^2 b)$($b$: 衝突径数)。太陽縁で 1.75″。レンダリング/光学の光線にオプトインで適用。

## 3. 状態表現

```rust
pub struct RelativitySettings {
    pub enabled: bool,                 // 既定 false
    pub pn_order: PnOrder,             // Newtonian / OnePN
    pub bodies: RelativityScope,       // All / Selected(Vec<GravBodyId>)
    pub proper_time_tracking: bool,    // GPS デモ用の固有時積算
    pub light_bending: bool,           // 光線への適用
}
// GravBody に proper_time: f64 を追加 (tracking 有効時のみ更新)
```

## 4. 数値解法

- 1PN 加速度を [01-gravitation-nbody.md](01-gravitation-nbody.md) の力計算に**加算項**として実装。
  シンプレクティック性は厳密には崩れるが、補正が $10^{-8}$ と小さいため実用上問題ない
  (長時間安定が要る場合は 1PN 対応の混合積分器を検討、Phase 後続)。
- 固有時は各天体で $d\tau/dt$ を積算(§2.2)。GPS デモは衛星と地表の固有時差を表示。
- 光の曲がりは光線トレース時に質量場の偏向を適用(既定オフ、重力レンズデモで有効化)。
- **決定論**: 補正項も決定的(追加の乱数なし)。ON/OFF はシナリオ設定に含みリプレイ再現。

## 5. 適用スケールと限界

- 1PN まで(2PN 以降・重力波・強場は対象外)。太陽系では 1PN で観測精度に達する。
- ブラックホール近傍・中性子星・宇宙論的スケールは非対象(弱場近似の外)。
- 既定オフ: 有効化はコストと意義がある特定シーン(水星・GPS・重力レンズ)に限定。
  「日常物体では $gh/c^2 \sim 10^{-15}$ で無意味」という [01-vision.md](../00-foundation/01-vision.md) §5 の
  判断は維持しつつ、天体スケールでの意義を提供する。

## 6. 他ドメインとの結合

- 天体重力([01](01-gravitation-nbody.md)): 加速度補正。
- 光学/レンダリング([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md),
  [17-rendering](../17-rendering/)): 光の曲がり(重力レンズ)。
- 観測(時計): 固有時 → GPS 測位誤差デモ([20-integration/04](../20-integration/04-world-api.md) の観測 API)。

## 7. 検証

- 水星近日点移動: 1PN で 42.98″/世紀 ± 1%(ON/OFF 差分で純相対論分を抽出)。
- GPS 時間差: +38.6 μs/日 ± 1%(重力・速度成分の内訳も一致)。
- 光の曲がり: 太陽縁 1.75″ ± 2%。
- 重力赤方偏移: $\Delta\nu/\nu = \Delta\Phi/c^2$ の一致。
- ニュートン極限: 補正を $c \to \infty$ で 0 に(実装の連続性)。

## 8. 実装フェーズ対応

Phase B 天体ドメインの後続(軌道・再突入の後)。オプトイン設計のため他天体機能に影響しない。
デモ: 水星近日点・GPS(D 群、[21-verification/03](../21-verification/03-demo-scenarios.md))。

## 9. パラメータ表

| 効果 | 値 | 出典 |
|---|---|---|
| 水星近日点移動(相対論分) | 42.98″/世紀 | 一般相対論の古典的検証 |
| GPS 正味時刻 | +38.6 μs/日 | Ashby, *Relativity in the GNSS* |
| 太陽縁の光偏向 | 1.7512″ | GR 予言・エディントン検証 |
| 光速 c | 299792458 m/s | 定義値([00-foundation/03](../00-foundation/03-units-conventions.md)) |

## 10. 性能プロファイル

- ホットスポット: 1PN 補正項の評価(有効天体のみ、通常少数)。
- 目標アルゴリズムとオーダー: ニュートン力計算に加算、$O(N_{rel})$($N_{rel}$ = 補正対象数、通常小)。
- SoA レイアウト: proper_time を GravBody 配列に追加列。
- 並列化単位: 補正対象が少ないため並列効果は限定的(N体本体に相乗り)。
- SIMD 対象カーネル: 補正が支配的でないため対象外。
- GPU 適性: 低(補正は軽量・少数)。
- ベンチ: 水星シーン・GPS シーンで ON/OFF のコスト差を測定(オーバーヘッドが無視できることの確認)。
