# レンダリング 02. スペクトル・パストレーシング — 物理正確な光輸送

crate: `sim-render`(Phase D)。写実性の設計目標 = **物理正確なフルパストレーシング**。
統計ドメインのモンテカルロ・光学ドメインの分光と数学基盤を共有する。

## 1. 担う現実の現象

正確な影・グローバルイルミネーション、色付き反射(color bleeding)、ガラス/水の屈折とコースティクス、
プリズムの分光、空の青と夕焼け(大気散乱)、被写界深度、モーションブラー、赤熱物体の色。
遊び方の例: 物理的に正しい虹・分光・水中の見え方を、光学ドメインの計算とレンダリングで一貫して見る。

## 2. 支配方程式

### 2.1 レンダリング方程式(Kajiya 1986)

点 $\mathbf{x}$ から方向 $\omega_o$ への放射輝度:

$$L_o(\mathbf{x}, \omega_o, \lambda) = L_e + \int_\Omega f_r(\mathbf{x}, \omega_i, \omega_o, \lambda)\,L_i(\mathbf{x}, \omega_i, \lambda)\,(\omega_i\cdot\mathbf{n})\,d\omega_i$$

$f_r$: BSDF、$\lambda$: 波長(**分光レンダリング** — RGB でなく波長ごとに解く)。
モンテカルロで経路をサンプリングして積分を推定する(パストレーシング)。

### 2.2 参加媒質(体積散乱)

放射伝達方程式(RTE): 吸収 $\sigma_a$・散乱 $\sigma_s$ 係数と位相関数 $p$。
大気: レイリー散乱($\sigma_s \propto \lambda^{-4}$、空の青)+ ミー散乱(エアロゾル・雲)。
煙・水も参加媒質として扱う([11-fluid](../11-fluid/) の密度場を消費)。

## 3. 状態表現

```rust
pub struct PathTracer {
    pub spp: u32,                    // samples per pixel (プログレッシブ加算)
    pub max_depth: u32,              // 経路長上限 (ロシアンルーレット併用)
    pub spectral: SpectralConfig,    // 波長サンプリング (Hero wavelength 法)
    pub bvh: Bvh,                    // シーン加速構造
    pub rng: SimRng,                 // シード付き ([01-math/04])
    pub sampler: Sampler,            // 層化 / 低食い違い列 (Sobol)
}
```

- 分光: hero wavelength sampling(1 経路で複数波長を相関サンプル)で色ノイズを抑える。
- スペクトル ⇔ RGB は光学ドメインの CIE 等色関数([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md) §4)。

## 4. 数値解法(モンテカルロ光輸送)

```text
render(scene):
  for each pixel (並列):
    for s in 0..spp:
      λ = sample_hero_wavelength(rng)
      ray = camera.generate_ray(pixel, jitter, λ, lens)   # 被写界深度
      L = trace(ray, λ)
      accumulate(pixel, L)                                 # プログレッシブ
    tone_map(pixel)                                        # [03-materials-camera.md]

trace(ray, λ, depth):
  hit = bvh.intersect(ray)                                 # 加速構造
  if none: return environment(ray, λ)                      # 空・HDR 環境
  L = hit.emission(λ)
  # 直接光 (Next Event Estimation): 光源を明示サンプル
  L += sample_lights(hit, λ) * bsdf(hit, λ)
  # 間接光: BSDF に従い次方向をサンプル (重要度サンプリング)
  (ωi, pdf) = sample_bsdf(hit, λ)
  if russian_roulette(depth): return L
  L += trace(spawn_ray(hit, ωi), λ, depth+1) * bsdf/pdf * cosθ
  # 参加媒質: 経路上で散乱イベントをサンプル (大気・水・煙)
  return L
```

- **分散低減**: 重要度サンプリング(BSDF・光源)、多重重点サンプリング(MIS)、
  低食い違い列(Sobol/blue noise)、NEE。
- **BVH**: 三角形メッシュ + 解析形状(球・平面)。SAH 構築。粒子/等値面は事前抽出。
- **決定論**: ピクセル×サンプルごとに固定サブストリーム PRNG。並列(タイル分割)でも
  ピクセル独立なので順序非依存 → 同一シード同一画像([00-foundation/06](../00-foundation/06-performance-strategy.md) §2.2)。

## 5. 適用範囲と限界

- 物理正確を目標とするが、実装は Phase D。対象現象(§1)を段階実装。
- 波動光学(回折・干渉・薄膜)は幾何光学ベースのため直接は扱わない
  — それらは FDTD([13-electromagnetism/03](../13-electromagnetism/03-maxwell-fdtd.md))・量子の領分。
  虹・分光・コースティクスは幾何光学 + 分光で表現。
- 偏光は既定オフ(フルネルは非偏光平均)。蛍光・燐光は将来。

## 6. 他ドメインとの結合

- 光学([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md)): 屈折率 $n(\lambda)$・フルネル・
  放射スペクトルの供給。レイトレーサの交差判定を共有(ray-cast は衝突検出由来
  [10-mechanics/02](../10-mechanics/02-collision-detection.md))。
- 統計([15-statistical/04](../15-statistical/04-monte-carlo.md)): モンテカルロ積分の数学基盤共有。
- 熱: 黒体放射スペクトル(温度 → 色)。
- 流体・天体: 参加媒質(煙・水・大気)の密度場、天体大気の散乱。

## 7. 検証

- **白色炉テスト(furnace test)**: 一様放射環境で完全拡散反射面が背景と同じ輝度になる
  (エネルギー保存・BSDF 正規化の厳密検証)。
- フルネル反射率が解析式と一致([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md) §7)。
- 分光: プリズム・虹の色分散が屈折率の波長依存と一致。
- コーネルボックス: 既知の参照解(color bleeding)と収束一致。
- 大気: レイリー散乱の $\lambda^{-4}$ による空の青・地平線の赤の定量。
- 収束: サンプル数 $N$ でノイズが $O(1/\sqrt N)$ 減少。決定論(同一シード同一画像)。

## 8. 実装フェーズ対応

**Phase D**。順序: BVH + 拡散/鏡面 BSDF + NEE(基本パストレ)→ 分光・屈折・コースティクス →
参加媒質(大気・水・煙)→ 被写界深度・モーションブラー。CPU 実装を先に、GPU を後
([00-foundation/06](../00-foundation/06-performance-strategy.md) §3、CPU 優先)。

## 9. パラメータ表

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| spp(プレビュー品質) | 16〜64 | プログレッシブで増加 |
| spp(最終品質) | 1024〜4096 | ノイズ許容による |
| 最大経路長 | 8〜16 + ロシアンルーレット | 無限反射の打切り |
| 波長サンプリング | hero + 3〜4 従属波長 | 色ノイズ抑制(Wilkie 2014) |
| 大気レイリー係数(海面) | $\beta_R(550nm) \approx 1.16\times10^{-5}$ /m | 標準大気散乱 |

## 10. 性能プロファイル

- ホットスポット: レイ-シーン交差(BVH 走査)、BSDF・光源サンプリング。
- 目標アルゴリズムとオーダー: BVH で $O(\log n)$ 交差、MIS/重要度サンプリングで分散低減。
- SoA レイアウト: レイのバッチ(origin/dir/throughput 別配列)、BVH ノード配列。
- 並列化単位: ピクセル/タイル(完全並列、ピクセル独立)。rayon。
- SIMD 対象カーネル: レイ-AABB/三角形交差(パケットトレース)、シェーディング。
- GPU 適性: **高**(パストレは GPU の代表的用途。WebGPU compute。CPU 参照実装は残す)。
- ベンチ: コーネルボックス・分光シーン・大気シーンで criterion(オフラインなので画像/秒)。
