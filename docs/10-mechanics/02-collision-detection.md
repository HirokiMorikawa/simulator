# 力学 02. 衝突検出 — broadphase / narrowphase / 接触マニフォールド

crate: `sim-mechanics`。

## 1. 担う現実の現象

物と物が触れる・ぶつかる・積み重なる、そのすべての幾何的判定。
遊び方の例: ビリヤードの multi-ball 衝突、積み木の接触面、斜面と箱の接触。

## 2. 問題定義(支配方程式に相当)

各ステップで、剛体集合から「互いに貫入している(または接触距離内の)形状ペア」を列挙し、
接触ソルバが必要とする幾何情報 — **接触マニフォールド** — を生成する:

- 接触法線 $\hat{\mathbf{n}}$(A→B、単位ベクトル)
- 接触点 $\mathbf{p}_k$(最大 4 点)とそれぞれの貫入深さ $\delta_k \ge 0$
- 特徴 ID(warm starting でステップ間の対応を取るため)

## 3. 状態表現・Rust 型定義

```rust
pub enum Shape {
    Sphere { radius: f64 },
    Box { half_extents: Vec3 },
    Capsule { radius: f64, half_height: f64 },     // Phase 2
    Plane { normal: Vec3, d: f64 },                // static 専用・無限平面
    Compound { children: Vec<(Transform, Shape)> },// Phase 2
    ConvexMesh { vertices: Vec<Vec3> },            // Phase 5 (GJK/EPA)
}

pub struct Aabb { pub min: Vec3, pub max: Vec3 }

pub struct ContactPoint {
    pub world_point: Vec3,     // 接触点 (A表面とB表面の中点)
    pub penetration: f64,      // >0 でめり込み
    pub feature_id: u32,       // 頂点/辺/面の組を符号化 (warm start 用)
}

pub struct ContactManifold {
    pub body_a: BodyId, pub body_b: BodyId,   // 常に body_a.index < body_b.index に正規化
    pub normal: Vec3,                          // A→B
    pub points: ArrayVec<ContactPoint, 4>,
}
```

## 4. アルゴリズム(数値解法に相当)

### 4.1 Broadphase

- **Phase 1: 総当たり** $O(n^2)$ の AABB 重なり判定。500 体 = 125k ペア判定は予算内。
  AABB は速度でわずかに膨張(speculative margin $= |\mathbf{v}|\Delta t$)させ、高速物体のすり抜けを緩和。
- **Phase 2: sweep and prune (SAP)**。x 軸で区間ソート(前ステップ順を初期値に挿入ソート —
  ほぼ整列済みで $O(n)$)、重なった区間のみ y/z を確認。
- ペア列挙順は (indexA, indexB) 昇順に固定(決定論)。
- 衝突フィルタ: `collision_group: u32` / `collision_mask: u32` のビット AND(エンティティ層が
  自己衝突制御に使う)。

### 4.2 Narrowphase(ディスパッチ)

形状ペア → 専用関数のテーブル。対称ペアは正規化(Sphere-Box は常にこの順で呼び、結果の法線を反転で対応)。

| ペア | 方法 | Phase |
|---|---|---|
| Sphere–Sphere | 中心間距離 vs 半径和(§4.3) | 1 |
| Sphere–Plane | 中心の平面距離 vs 半径 | 1 |
| Box–Plane | 8 頂点の平面距離、負の頂点を接触点(最大4点選抜) | 1 |
| Sphere–Box | ボックスローカルで最近点クランプ | 1 |
| Box–Box | SAT(分離軸: 各面法線 3+3、辺×辺 9 = 15 軸)§4.4 | 2 |
| Capsule 系 | 線分間最近点 + 球判定に帰着 | 2 |
| Convex–Convex | GJK(分離距離)+ EPA(貫入)§4.5 | 5 |

### 4.3 例: Sphere–Sphere(具体式)

中心 $\mathbf{c}_A, \mathbf{c}_B$、半径 $r_A, r_B$。$\mathbf{d} = \mathbf{c}_B - \mathbf{c}_A$、$L = |\mathbf{d}|$。
$L < r_A + r_B$ なら接触:
$\hat{\mathbf{n}} = \mathbf{d}/L$($L < \epsilon$ なら決定的フォールバック $\hat{\mathbf{n}} = \hat{y}$)、
$\delta = r_A + r_B - L$、接触点 $= \mathbf{c}_A + \hat{\mathbf{n}}(r_A - \delta/2)$。

### 4.4 Box–Box SAT(要点)

15 本の候補軸で射影区間の重なりを調べ、最小重なり軸を接触法線とする。
辺×辺軸は $|\mathbf{e}_i \times \mathbf{e}_j| < \epsilon$(平行)のとき除外。
面接触の場合は**接触面クリッピング**(参照面に対して入射面の頂点を Sutherland-Hodgman クリップ)で
最大 4 点のマニフォールドを作る(Box2D/ODE と同手法)。
数値ジッタ対策: 軸選択に前ステップの軸を優先するヒステリシス(相対 5%)を入れ、
法線のフリップ振動を防ぐ。

### 4.5 GJK/EPA(Phase 5、設計のみ)

- GJK: ミンコフスキー差のサポート写像で原点を含む単体を探索。分離時は最近距離を返す。
- EPA: 貫入時、ミンコフスキー差の境界を多面体拡張で近似し、最浅貫入方向を得る。
- 接触点は貫入方向への摂動サポート点から復元。実装の要諦(退化単体・数値許容)は
  実装フェーズで Gino van den Bergen, *Collision Detection in Interactive 3D Environments* を正とする。

### 4.6 接触マニフォールドの持続化

ステップ間でマニフォールドをキャッシュし、feature_id が一致する接触点の蓄積インパルスを引き継ぐ
(warm starting、[03-contact-solver.md](03-contact-solver.md) §4.4)。
点の再利用判定: 同一 feature_id かつ移動 < 2mm。

## 5. 適用スケールと限界

- 離散衝突検出(ステップ端点での判定)なので、**高速小物体はすり抜けうる**:
  弾丸($v=300$ m/s)は 1 ステップで 2.5 m 進む。Phase 1〜4 は speculative margin で緩和し、
  真の対策 CCD(連続衝突検出: 球は ray-cast、凸は conservative advancement)は Phase 5。
  すり抜け発生は診断イベントで可視化する(黙って壊れない)。
- 無限平面は static 専用。地形は Phase 5 の高さ場/メッシュで拡張。

## 6. 他ドメインとの結合

- 接触イベント(開始・継続・終了、法線力の大きさ)を EventQueue へ — 熱(摩擦発熱の面積按分)、
  音(将来)、エンティティ(足接地判定)が購読する。
- 流体の障害物マーキング(格子セルの solid フラグ)に剛体形状の内外判定
  `Shape::contains(local_point)` を提供する。

## 7. 検証

- 各ペア関数の幾何ユニットテスト: 手計算ケース(貫入深さ・法線・接触点、$\epsilon_{abs}=10^{-12}$)。
- 対称性: A-B と B-A で法線が正確に反転し点が一致。
- SAT: 既知の面接触・辺接触・頂点接触ケースで正しい軸が選ばれる。
- 統計テスト: 乱数配置(決定シード)で GJK 実装と総当たりサンプリングの分離判定一致(Phase 5)。
- broadphase: SAP の出力ペア集合が総当たりと完全一致。

## 8. 実装フェーズ対応

§4 の表参照。Phase 1 = 総当たり + 球/箱/平面の 4 関数 + マニフォールド持続化。

## 9. パラメータ表・擬似コード

| パラメータ | 値 | 根拠 |
|---|---|---|
| speculative margin | $\max(|\mathbf{v}|\Delta t,\ 5\,\mathrm{mm})$ | 1 ステップ移動量 |
| 接触点再利用距離 | 2 mm | Box2D 系の経験値 |
| SAT 軸ヒステリシス | 相対 5% | ジッタ抑制の経験値 |
| マニフォールド最大点数 | 4 | 面接触の安定支持に十分(3点+冗長1) |

```text
collision_detection(bodies):
  update AABBs (shape, transform, speculative margin)
  pairs = broadphase(AABBs)                       # (a<b) 昇順
  manifolds = []
  for (a, b) in pairs:
    if !filter_pass(a, b): continue
    m = narrowphase_dispatch(shape[a], xf[a], shape[b], xf[b])
    if m: manifolds.push(match_features_with_cache(m))  # warm start 引き継ぎ
  return manifolds
```
