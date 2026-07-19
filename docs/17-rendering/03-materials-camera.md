# レンダリング 03. マテリアルとカメラ — 光学物性・物理カメラ・トーンマッピング

crate: `sim-render`(Phase D)。パストレーサ([02-path-tracing.md](02-path-tracing.md))が使う
マテリアル(BSDF)と、シーンを画像化する物理カメラ・色処理。

## 1. 担う現実の現象

金属・プラスチック・ガラス・水・布の質感、濡れた路面の反射、すりガラスの拡散透過、
被写界深度(ボケ)、露出、フィルムの色再現。
遊び方の例: 同じ形の物体を材質だけ変えて見え方を比べる(物性 DB と一貫)、
絞りを変えてボケの量を見る。

## 2. 支配方程式 — BSDF

双方向散乱分布関数。物理ベースの標準モデルを採用:

- **拡散**: Lambert(または Oren-Nayar で粗面)。
- **鏡面/光沢**: マイクロファセット GGX(Trowbridge-Reitz)分布 + Smith 幾何項 +
  **フルネル項**(光学ドメインの屈折率 $n(\lambda)$ から、[13-electromagnetism/04](../13-electromagnetism/04-light-optics.md) §2.1)。
- **金属/誘電体**: 複素屈折率 $n + ik$(金属)/ 実屈折率(誘電体)で分岐。
- **透過/屈折**: 誘電体の鏡面透過(スネル則 + フルネル)。粗面透過(すりガラス)。
- **エネルギー保存**: 反射 + 透過 + 吸収 = 1 を厳密に(白色炉テスト [02](02-path-tracing.md) §7)。

BSDF は物理的に正しい正規化(相反性・エネルギー保存)を満たすものだけを使う。

## 3. 状態表現 — 物性 DB との接続

```rust
pub struct OpticalMaterial {
    pub base: MaterialId,            // MaterialDb 参照 ([12-thermal/04])
    pub model: BsdfModel,            // Diffuse / Metal / Dielectric / Glossy / Layered
    pub roughness: f64,              // GGX α
    pub ior: Spectrum,              // n(λ): MaterialDb の refractive_index を分光展開
    pub extinction: Option<Spectrum>,// k(λ): 金属の吸収
    pub albedo: Spectrum,           // 分光反射率
    pub emission: Option<EmissionSpec>, // 黒体温度 or 輝線 ([14-quantum/03])
}
```

- **MaterialDb を単一の真実源に**: 屈折率 $n$、比誘電率 $\varepsilon_r$($n=\sqrt{\varepsilon_r}$ の可視域近似)、
  放射率は熱の材料表([12-thermal/04](../12-thermal/04-material-thermal-props.md))と共有。
  「見た目の色」を物性から導くことで、物理と描画の一貫性を保つ(検証遊びの誠実さ)。
- 発光: 熱い物体は温度から黒体スペクトル([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md) §2.3)、
  ネオン/LED は輝線([14-quantum/03](../14-quantum/03-effective-models.md))。

## 4. 数値解法 — カメラと色

### 4.1 物理カメラ

- **薄レンズモデル**: 焦点距離 $f$、絞り $N$(F 値)から開口半径、被写界深度(ボケ)。
  レンズ上のサンプリングで焦点外をぼかす([02](02-path-tracing.md) のレイ生成)。
- **露出**: シャッター速度・ISO・絞りから露出値(EV)。物理的な光量 → センサー応答。
- モーションブラー: シャッター開時間内の時刻をサンプル(物理状態の補間、剛体 transform)。

### 4.2 分光 → 表示色

```text
spectrum_to_display(L_λ):
  XYZ = ∫ L_λ · CIE_xyz(λ) dλ           # 光学ドメインの等色関数
  RGB_linear = XYZ_to_sRGB(XYZ)
  RGB = tone_map(RGB_linear, exposure)   # HDR → LDR
  return gamma_encode(RGB)               # sRGB ガンマ
```

- トーンマッピング: フィルミック(ACES 等)または露出ベース。物理輝度(cd/m²)を
  ディスプレイ範囲へ。HDR 出力もオプション。

## 5. 適用範囲と限界

- Phase D 実装。層状マテリアル(クリアコート)・異方性・サブサーフェススキャタリング(肌・ミルク)は
  段階実装(後続)。
- 蛍光・偏光・薄膜干渉(シャボン玉)は既定対象外(薄膜は波動光学、[13-electromagnetism/03](../13-electromagnetism/03-maxwell-fdtd.md))。
- 物性 DB に光学値が無い材料は既定 BSDF(誘電体 $n=1.5$)+ 警告。

## 6. 他ドメインとの結合

- 材料 DB([12-thermal/04](../12-thermal/04-material-thermal-props.md)): $n, \varepsilon_r$, 放射率の供給源。
- 光学([13-electromagnetism/04](../13-electromagnetism/04-light-optics.md)): フルネル・分散・CIE 等色関数。
- 熱: 発光温度 → 黒体色。力学: 剛体 transform(モーションブラー)。

## 7. 検証

- フルネル反射率が入射角・屈折率で解析式と一致(誘電体・金属)。
- 白色炉(エネルギー保存)[02](02-path-tracing.md) §7。
- マクベスチャート的な既知反射率パッチの色再現(分光 → sRGB の正しさ)。
- 被写界深度: 錯乱円径が薄レンズ公式と一致。
- 露出: EV 変化に対する像の明るさが物理的にスケール。

## 8. 実装フェーズ対応

**Phase D**([02](02-path-tracing.md) と同時)。BSDF(拡散→誘電体→金属→粗面透過)→ 物理カメラ →
分光色処理・トーンマップ。

## 9. パラメータ表

| 材料(可視域 n) | n | model | 備考 |
|---|---|---|---|
| 水 | 1.333 | Dielectric | [12-thermal/04](../12-thermal/04-material-thermal-props.md) と共有 |
| ガラス(BK7) | 1.517 | Dielectric | 分散あり |
| 金(Au) | n+ik ≈ 0.47+2.4i(550nm) | Metal | 複素屈折率(CRC) |
| アルミ | ≈ 0.96+6.7i | Metal | 高反射 |
| ダイヤモンド | 2.417 | Dielectric | 高分散 |

| カメラ | 既定 | 備考 |
|---|---|---|
| 焦点距離 | 50 mm 相当 | 標準画角 |
| F 値 | 2.8〜8 | 被写界深度可変 |
| トーンマップ | ACES filmic | HDR→sRGB |

## 10. 性能プロファイル

- ホットスポット: BSDF 評価・サンプリング(パストレの内側)。
- 目標アルゴリズムとオーダー: 重要度サンプリング可能な解析 BSDF(GGX)で分散低減。
- SoA レイアウト: マテリアルパラメータの配列、スペクトルはサンプル配列。
- 並列化単位: [02](02-path-tracing.md) と同じ(ピクセル/タイル)。
- SIMD 対象カーネル: BSDF・フルネルのバッチ評価、分光の内積(→XYZ)。
- GPU 適性: 高([02](02-path-tracing.md) に相乗り)。
- ベンチ: 材質比較シーン(球アレイ)で画像/秒。
