# レンダリング 01. アーキテクチャ — 物理から分離した 2 経路描画

crate: 将来の `sim-render`。**実装は全物理完了後の Phase D**([22-roadmap/01](../22-roadmap/01-phases.md))。
本ドメインは設計のみを今回確定する。物理エンジンのクライアントであり、物理を書き換えない。

## 1. 役割と原則

- **描画は物理から分離**: レンダラは World の状態(剛体・粒子・場)と光学ドメインの結果を**読むだけ**。
  物理コアはレンダラを知らない([00-foundation/04](../00-foundation/04-architecture.md) のレイヤ規則)。
- **「物理としての光」と「見た目」を混同しない**: 光学ドメイン([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md))は
  物理観測データ(光線経路・スペクトル・強度)を提供する。レンダラはそれを**物理ベースで画像化**する。
- **検証可能性の維持**: レンダリングも物理的に正しいか検証する(白色炉テスト・フルネル・分光、
  [21-verification/01](../21-verification/01-analytic-tests.md))。「それらしい」絵作りではなく光輸送方程式の解。

## 2. 2 経路

| 経路 | 用途 | 手法 | 実行 |
|---|---|---|---|
| **(a) リアルタイムプレビュー** | 遊ぶ・操作・デバッグ | Three.js(ラスタライズ + PBR 近似) | ブラウザ 60fps |
| **(b) 物理正確フルパストレ** | 検証・鑑賞・スクリーンショット | スペクトル・モンテカルロ光輸送 | オフライン(数秒〜数分) |

- 同一シーン記述・同一マテリアルから両経路を駆動。プレビューで構図を決め、パストレで「正解の絵」を得る。
- パストレは物理正確([02-path-tracing.md](02-path-tracing.md)): 分光・大気散乱・コースティクス・被写界深度。
- リアルタイム経路は近似だが、光学ドメインの結果(屈折・分光・放射スペクトル)を可能な範囲で反映
  (例: 熱い物体の色は黒体スペクトルから、[13-electromagnetism/04](../13-electromagnetism/04-light-optics.md) §2.3)。

## 3. データフロー

```
World state (剛体 transform / 粒子 / 場) ─┐
光学ドメイン (光源スペクトル・屈折率・放射) ─┼─→ SceneDescription ─→ (a) Three.js
MaterialDb (n, ε_r, 放射率, 粗さ)          ─┘                    └─→ (b) パストレーサ
天体 (位置・スケール・大気)                                          → 画像 (HDR → トーンマップ)
```

```rust
pub struct SceneDescription {
    pub geometry: Vec<RenderInstance>,     // 剛体メッシュ・粒子・等値面 (流体表面)
    pub materials: Vec<OpticalMaterial>,   // [03-materials-camera.md]
    pub lights: Vec<LightSource>,          // スペクトル光源 (黒体/輝線/一様)
    pub camera: Camera,
    pub medium: Option<ParticipatingMedium>, // 大気・煙・水 (散乱)
}
```

- 流体表面: 格子は等値面抽出(marching cubes)、SPH は表面再構成 or メタボール。
- 粒子(SPH・煙・分子): 点/スプラット、または参加媒質としてボリュームレンダ。

## 4. 決定論とオフライン実行

- パストレはモンテカルロ(乱数)だが**シード付き**([01-math/04](../01-math/04-random.md))で再現可能。
  同一シーン・同一シード・同一サンプル数 → 同一画像。
- 物理の決定論([20-integration/02](../20-integration/02-determinism-replay.md))とは独立(描画は状態を変えない)。
- オフライン: 高サンプル数を時間をかけて積分。プログレッシブ表示(サンプルが増えるほど収束)。

## 5. 適用範囲と限界

- Phase D 実装。それまではプレビュー経路(Three.js)のみがデモで動く。
- パストレの計算量は GPU で緩和([02-path-tracing.md](02-path-tracing.md) §性能、[00-foundation/06](../00-foundation/06-performance-strategy.md) §3、CPU 優先)。
- 写実性の追求範囲(どこまでの光学現象を入れるか)は [02](02-path-tracing.md) の対象現象で規定。

## 6. 他ドメインとの結合

- 光学([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md)): スペクトル・屈折率・放射・光線物理の供給源。
- 熱: 黒体放射の色(赤熱・白熱)。
- 材料 DB: 光学物性([03-materials-camera.md](03-materials-camera.md))。
- 天体([16-astro](../16-astro/)): 天体位置・大気(散乱)、光の重力偏向(相対論オプトイン)。
- World API([20-integration/04](../20-integration/04-world-api.md)): 状態の読み出しのみ。

## 7. 検証

[02-path-tracing.md](02-path-tracing.md) §7 に集約(白色炉・フルネル・分光・収束)。

## 8. 実装フェーズ対応

**Phase D**(全物理完了後)。設計は本ドメイン 3 文書で完了済み。写実性のレベルは
「物理正確フルパストレ」で確定([02](02-path-tracing.md))。

## 9. パラメータ

[02-path-tracing.md](02-path-tracing.md) §9, [03-materials-camera.md](03-materials-camera.md) §9 に集約。
