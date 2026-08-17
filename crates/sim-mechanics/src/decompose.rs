//! 近似凸分解(approximate convex decomposition、V-HACD相当)。
//! 設計: docs/10-mechanics/02-collision-detection.md §4.2(narrowphase の前提)。
//!
//! ## 何を解決するか
//!
//! `Shape::ConvexMesh`は頂点列しか持たず、**常にその凸包として**扱われる
//! (`crate::hull::convex_hull`)。ユーザーが実際に描いたメッシュが非凸
//! (L字・U字・くびれのあるダンベル型など)だと、頂点だけを渡しても凸包は
//! 凹みを無視して埋めてしまい、体積・慣性が過大評価され、接触マニフォールドも
//! 実際には無いはずの領域で生成される。これを解くには**三角形の面情報**
//! (どの3頂点が実際の表面を成すか)が要る——凹みは「点の位置」ではなく
//! 「どの面が実際に存在するか」でしか表現できないため、頂点だけの入力からは
//! 原理的に復元できない。そこで本モジュールは`Shape::ConvexMesh`とは別に
//! **明示的な三角形接続を受け取る**エントリポイント(`decompose_mesh`、
//! `Shape::from_triangle_mesh`から呼ばれる)を用意する。
//!
//! ## アルゴリズム(V-HACDの簡略版)
//!
//! 1. 全頂点の凸包を張る(`crate::hull::convex_hull`)。
//! 2. **凹み具合**を、凸包の体積と実メッシュの体積の比で測る
//!    (`mesh_volume`。任意の閉曲面メッシュの体積をガウスの発散定理で
//!    直接求められるので、凸包の体積という「上から見た近似」と比較できる)。
//! 3. 比の超過が閾値未満なら**既に十分凸**とみなし、分解せず単一の凸包を返す
//!    ——凸メッシュ(既存シーンのほぼ全て)は分解が完全な no-op になる。
//! 4. 閾値を超えるなら、軸並行な平面で三角形集合を2分割する候補を複数試し、
//!    「分割後の2つの凸包の体積和」を最小化する分割を選んで再帰する。
//!    再帰は深さ上限・分割の改善が乏しい・三角形が少なすぎる、のいずれかで
//!    止まる。
//!
//! ## 本家 V-HACD との違い(意図的な簡略化)
//!
//! - **オクツリー等の高速化構造は使わない**。候補分割面の評価に毎回
//!   凸包を張り直す($O(n)$ 個の候補 × $O(n^2)$ の凸包構築)が、これは
//!   `Shape`の構築時にしか走らない(衝突検出のホットパスではない)ので、
//!   `crate::hull`のモジュールdocと同じ理由で正しさを優先した。
//! - **分割は三角形の重心をもとに丸ごと片側へ割り振る**(実際に平面で
//!   切って断面にキャップを張る厳密なクリッピングはしない)。切り口をまたぐ
//!   三角形の頂点が両側の点群に重複して現れうるが、その分はそれぞれの
//!   凸包が互いにわずかに重なる(体積をわずかに過大評価する)方向にしか
//!   効かない——本物のV-HACDの出力パーツも重なりを許容するのが普通で、
//!   物理エンジンの当たり判定用途としては実害がない。
//! - **分割面は軸並行のみ**(任意の向きの平面は探索しない)。V-HACDの
//!   「支持点サンプリングによる任意方向の平面探索」は実装コストに見合わない
//!   と判断した——このシミュレータが扱う手作りメッシュ(数十〜数百頂点)は
//!   軸並行な分割でも十分に凹みを解消できる(下記テスト参照)。
//!
//! ## パラメータ
//!
//! - `MAX_DEPTH`: 再帰の最大深さ($2^{\text{MAX\_DEPTH}}$ パーツが上限)。
//! - `GLOBAL_CONCAVITY_THRESHOLD`: 全体の凹み判定閾値(3%)。
//! - `MIN_SPLIT_IMPROVEMENT`: これ未満しか体積和が減らない分割は採用しない。
//! - `MIN_TRIANGLES_TO_SPLIT`: これ未満の三角形数のパーツはもう分割しない。
//! - `MAX_CANDIDATES_PER_AXIS`: 軸ごとに試す分割候補数の上限(性能の保険)。

use crate::hull::{convex_hull, ConvexHull};
use sim_math::Vec3;

/// 再帰の最大深さ。1ノードが最大2つに分かれるので、パーツ数は $2^3=8$ が上限
/// (課題の指示「8〜16個程度の妥当な上限」の範囲内)。
const MAX_DEPTH: u32 = 3;

/// 「凸包の体積 / 実メッシュの体積 - 1」がこれ未満なら「実質凸」として
/// 分解しない。閉じた三角形メッシュの体積計算(`mesh_volume`)は数値誤差が
/// 倍精度の丸め程度しか無いので、3%は「本物の凹み」と「三角形分割の
/// 数値誤差」を十分に区別できる余裕を持つ値。
const GLOBAL_CONCAVITY_THRESHOLD: f64 = 0.03;

/// 分割後の2凸包の体積和が、分割前の体積のこの割合未満しか減らないなら
/// 「これ以上割っても得しない」とみなして再帰を止める(無限に近い細分化の防止)。
const MIN_SPLIT_IMPROVEMENT: f64 = 0.02;

/// これ未満の三角形数のパーツはもう分割しない(四面体1つ分未満の断片を
/// 割ろうとする無意味な再帰を避ける)。
const MIN_TRIANGLES_TO_SPLIT: usize = 4;

/// 軸1本あたりに試す分割候補数の上限。三角形の重心座標を軸に射影して
/// ソートし、隣り合う値の中点を候補にする(自然な「隙間」を狙う)——
/// 候補数が三角形数に比例して増えるのを防ぐための保険。
const MAX_CANDIDATES_PER_AXIS: usize = 32;

/// 三角形メッシュ(`vertices`+`triangles`、各三角形は**外向き**の巻き順)の
/// 近似凸分解。モジュールdoc参照。
///
/// 退化入力(頂点が3次元的な広がりを持たない)は空の`Vec`を返す
/// (呼び出し側は「体積なし」として扱う——`Shape::ConvexMesh`の空入力と同じ規約)。
pub(crate) fn decompose_mesh(vertices: &[Vec3], triangles: &[[usize; 3]]) -> Vec<ConvexHull> {
    let Some(whole_hull) = convex_hull(vertices) else {
        return Vec::new();
    };
    // 三角形が無い(=頂点だけの入力)場合は凹みを判定しようが無いので、
    // 凸包そのものを返す(`Shape::ConvexMesh`と同じ挙動へのフォールバック)。
    if triangles.is_empty() {
        return vec![whole_hull];
    }
    let Some(whole_mp) = whole_hull.mass_properties() else {
        return Vec::new();
    };
    let true_volume = mesh_volume(vertices, triangles).abs();
    if true_volume <= 0.0 || whole_mp.volume / true_volume - 1.0 < GLOBAL_CONCAVITY_THRESHOLD {
        return vec![whole_hull];
    }

    let fallback = whole_hull.clone();
    let mut parts = Vec::new();
    recurse(
        vertices.to_vec(),
        triangles.to_vec(),
        whole_hull,
        whole_mp.volume,
        MAX_DEPTH,
        &mut parts,
    );
    if parts.is_empty() {
        parts.push(fallback); // 実際には到達しない防御(再帰は必ず1個以上pushする)
    }
    parts
}

/// 任意の閉じた(watertight・外向きの巻き順で一貫した)三角形メッシュの
/// 符号付き体積。ガウスの発散定理の標準的な離散化で、
/// `crate::hull::ConvexHull::mass_properties`の体積項(原点と各面が張る
/// 四面体の符号付き和)と**同じ式**を、凸性を仮定しない任意の三角形リストへ
/// 一般化しただけ——凸包はどうしても凹みを無視してしまうので、
/// 「本当の体積」を測るにはこちらが要る。
fn mesh_volume(vertices: &[Vec3], triangles: &[[usize; 3]]) -> f64 {
    triangles
        .iter()
        .map(|&[i, j, k]| {
            let (a, b, c) = (vertices[i], vertices[j], vertices[k]);
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// 三角形部分集合から、使われている頂点だけを詰め直した(頂点, 三角形)を作る。
/// `crate::hull::convex_hull`末尾の再詰め処理と同じ発想。
fn subset_from_triangles(
    vertices: &[Vec3],
    triangles: &[[usize; 3]],
) -> (Vec<Vec3>, Vec<[usize; 3]>) {
    let mut remap = vec![usize::MAX; vertices.len()];
    let mut out_vertices = Vec::new();
    let mut out_triangles = Vec::with_capacity(triangles.len());
    for tri in triangles {
        let mut nt = [0usize; 3];
        for (k, &vi) in tri.iter().enumerate() {
            if remap[vi] == usize::MAX {
                remap[vi] = out_vertices.len();
                out_vertices.push(vertices[vi]);
            }
            nt[k] = remap[vi];
        }
        out_triangles.push(nt);
    }
    (out_vertices, out_triangles)
}

/// 三角形`tri`の重心の、軸`axis`(0=x,1=y,2=z)成分。
fn centroid_axis(vertices: &[Vec3], tri: [usize; 3], axis: usize) -> f64 {
    let c = (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]).scale(1.0 / 3.0);
    match axis {
        0 => c.x,
        1 => c.y,
        _ => c.z,
    }
}

/// 三角形リストを2群に割った結果(`split_triangles_by_plane`の戻り値)。
type TriangleHalves = (Vec<[usize; 3]>, Vec<[usize; 3]>);

/// 三角形を軸並行平面(軸`axis`の座標が`cut`)で2群に割る。三角形は**重心の
/// 位置だけ**で丸ごとどちらかへ割り振る(モジュールdocの「意図的な簡略化」)。
/// どちらかが空になる分割は無効(`None`)。
fn split_triangles_by_plane(
    vertices: &[Vec3],
    triangles: &[[usize; 3]],
    axis: usize,
    cut: f64,
) -> Option<TriangleHalves> {
    let mut a = Vec::new();
    let mut b = Vec::new();
    for &tri in triangles {
        if centroid_axis(vertices, tri, axis) <= cut {
            a.push(tri);
        } else {
            b.push(tri);
        }
    }
    if a.is_empty() || b.is_empty() {
        None
    } else {
        Some((a, b))
    }
}

/// 軸`axis`上で試す分割候補(隣り合う重心座標の中点)。候補数が
/// [`MAX_CANDIDATES_PER_AXIS`]を超えたら等間隔に間引く(性能の保険、
/// モジュールdoc参照)。
fn candidate_cuts(vertices: &[Vec3], triangles: &[[usize; 3]], axis: usize) -> Vec<f64> {
    let mut values: Vec<f64> = triangles
        .iter()
        .map(|&tri| centroid_axis(vertices, tri, axis))
        .collect();
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    values.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    if values.len() < 2 {
        return Vec::new();
    }
    let mut mids: Vec<f64> = values.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
    if mids.len() > MAX_CANDIDATES_PER_AXIS {
        let stride = mids.len() as f64 / MAX_CANDIDATES_PER_AXIS as f64;
        mids = (0..MAX_CANDIDATES_PER_AXIS)
            .map(|i| mids[(((i as f64) * stride) as usize).min(mids.len() - 1)])
            .collect();
    }
    mids
}

/// 1回の分割の結果(2つの子パーツぶんの頂点・三角形・凸包・体積)。
struct Split {
    a_vertices: Vec<Vec3>,
    a_triangles: Vec<[usize; 3]>,
    a_hull: ConvexHull,
    a_volume: f64,
    b_vertices: Vec<Vec3>,
    b_triangles: Vec<[usize; 3]>,
    b_hull: ConvexHull,
    b_volume: f64,
    total_volume: f64,
}

/// 「分割後の2凸包の体積和を最小化する」軸並行分割を探す(モジュールdocの
/// アルゴリズム4番)。改善が[`MIN_SPLIT_IMPROVEMENT`]未満、または有効な
/// 分割が1つも無ければ`None`(=これ以上割らない)。
fn best_split(vertices: &[Vec3], triangles: &[[usize; 3]], parent_volume: f64) -> Option<Split> {
    let mut best: Option<Split> = None;
    for axis in 0..3 {
        for cut in candidate_cuts(vertices, triangles, axis) {
            let Some((tri_a, tri_b)) = split_triangles_by_plane(vertices, triangles, axis, cut)
            else {
                continue;
            };
            let (va, ta) = subset_from_triangles(vertices, &tri_a);
            let (vb, tb) = subset_from_triangles(vertices, &tri_b);
            let Some(ha) = convex_hull(&va) else {
                continue;
            };
            let Some(hb) = convex_hull(&vb) else {
                continue;
            };
            let (Some(mpa), Some(mpb)) = (ha.mass_properties(), hb.mass_properties()) else {
                continue;
            };
            let total = mpa.volume + mpb.volume;
            if best.as_ref().is_none_or(|s| total < s.total_volume) {
                best = Some(Split {
                    a_vertices: va,
                    a_triangles: ta,
                    a_hull: ha,
                    a_volume: mpa.volume,
                    b_vertices: vb,
                    b_triangles: tb,
                    b_hull: hb,
                    b_volume: mpb.volume,
                    total_volume: total,
                });
            }
        }
    }
    best.filter(|s| s.total_volume < parent_volume * (1.0 - MIN_SPLIT_IMPROVEMENT))
}

/// 1ノードぶんの再帰。深さが尽きた・三角形が少なすぎる・有効な改善分割が
/// 無い、のいずれかで葉として`hull`を`out`へ積む。深さが単調に減るので
/// 必ず停止する(パーツ数の上限は $2^{\text{MAX\_DEPTH}}$、モジュールdoc参照)。
fn recurse(
    vertices: Vec<Vec3>,
    triangles: Vec<[usize; 3]>,
    hull: ConvexHull,
    hull_volume: f64,
    depth_budget: u32,
    out: &mut Vec<ConvexHull>,
) {
    if depth_budget == 0 || triangles.len() < MIN_TRIANGLES_TO_SPLIT {
        out.push(hull);
        return;
    }
    match best_split(&vertices, &triangles, hull_volume) {
        Some(split) => {
            recurse(
                split.a_vertices,
                split.a_triangles,
                split.a_hull,
                split.a_volume,
                depth_budget - 1,
                out,
            );
            recurse(
                split.b_vertices,
                split.b_triangles,
                split.b_hull,
                split.b_volume,
                depth_budget - 1,
                out,
            );
        }
        None => out.push(hull),
    }
}

/// 一辺`2*half`の箱の12三角形メッシュ(2三角形×6面、外向き巻き順)。
/// `shape.rs`のテストからも再利用するため`cfg(test)`のトップレベルに置く。
#[cfg(test)]
pub(crate) fn box_mesh(half: f64) -> (Vec<Vec3>, Vec<[usize; 3]>) {
    let v = vec![
        Vec3::new(-half, -half, -half), // 0
        Vec3::new(half, -half, -half),  // 1
        Vec3::new(half, half, -half),   // 2
        Vec3::new(-half, half, -half),  // 3
        Vec3::new(-half, -half, half),  // 4
        Vec3::new(half, -half, half),   // 5
        Vec3::new(half, half, half),    // 6
        Vec3::new(-half, half, half),   // 7
    ];
    let t = vec![
        [0, 3, 2],
        [0, 2, 1], // z = -half (法線 -z)
        [4, 5, 6],
        [4, 6, 7], // z = +half (法線 +z)
        [0, 1, 5],
        [0, 5, 4], // y = -half (法線 -y)
        [3, 7, 6],
        [3, 6, 2], // y = +half (法線 +y)
        [0, 4, 7],
        [0, 7, 3], // x = -half (法線 -x)
        [1, 2, 6],
        [1, 6, 5], // x = +half (法線 +x)
    ];
    (v, t)
}

/// L字柱(モジュールdocの分割対象そのもの)。`shape.rs`の
/// `l_shaped_compound_union_volume_matches_the_analytic_value`と**同一形状**
/// (縦棒: 半寸(0.25,1.0,0.25)@(0,0.75,0)、横棒: 半寸(0.5,0.25,0.25)@(0.25,-0.25,0)、
/// 真の体積0.6875)を、2つの箱の合成としてではなく**1枚の三角形メッシュ**
/// として直接構築したもの——z方向((-0.25,0.25))へ2D L字ポリゴンを
/// 押し出した角柱。`shape.rs`のテストからも再利用するため`cfg(test)`の
/// トップレベルに置く。
///
/// 2D境界(xy平面、反時計回り、7点。P6はP5-P0の直線上・P3はP2からの
/// 段差の頂点で、どちらも三角形分割のために辺上へ足した点):
/// ```text
/// P0=(-0.25,-0.5) P1=(0.75,-0.5) P2=(0.75,0) P3=(0.25,0)
/// P4=(0.25,1.75)  P5=(-0.25,1.75) P6=(-0.25,0)
/// ```
/// 面積は靴紐公式で1.375、押し出し厚み0.5なので体積は1.375*0.5=0.6875
/// (`shape.rs`の値と一致することを`l_prism_volume_matches_the_l_shaped_compound_value`
/// で確認する)。
#[cfg(test)]
pub(crate) fn l_shaped_prism_mesh() -> (Vec<Vec3>, Vec<[usize; 3]>) {
    let p2d = [
        (-0.25, -0.5), // P0
        (0.75, -0.5),  // P1
        (0.75, 0.0),   // P2
        (0.25, 0.0),   // P3
        (0.25, 1.75),  // P4
        (-0.25, 1.75), // P5
        (-0.25, 0.0),  // P6
    ];
    let (zb, zt) = (-0.25, 0.25);
    // 頂点0..6が底面(z=zb)、7..13が同じxyの上面(z=zt)。
    let mut vertices = Vec::with_capacity(14);
    for &(x, y) in &p2d {
        vertices.push(Vec3::new(x, y, zb));
    }
    for &(x, y) in &p2d {
        vertices.push(Vec3::new(x, y, zt));
    }
    let b = |i: usize| i;
    let t = |i: usize| i + 7;

    // 上面(z=zt、法線+z): 縦棒(P6,P3,P4,P5)と横棒(P0,P1,P2,P6)へ2分割。
    // 底面(z=zb、法線-z): 上面と同じ頂点3つ組で後ろ2つを入れ替える(巻き順反転)。
    let mut triangles = vec![
        [t(6), t(3), t(4)],
        [t(6), t(4), t(5)],
        [t(0), t(1), t(2)],
        [t(0), t(2), t(6)],
        [b(6), b(4), b(3)],
        [b(6), b(5), b(4)],
        [b(0), b(2), b(1)],
        [b(0), b(6), b(2)],
    ];
    // 側面: 境界を7点の輪(0→1→2→3→4→5→6→0)としてたどり、各辺ごとに
    // 外向きの四角形(2三角形)を張る。
    for i in 0..7 {
        let j = (i + 1) % 7;
        triangles.push([b(i), b(j), t(j)]);
        triangles.push([b(i), t(j), t(i)]);
    }

    (vertices, triangles)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn l_prism_volume_matches_the_l_shaped_compound_value() {
        let (v, t) = l_shaped_prism_mesh();
        let volume = mesh_volume(&v, &t);
        assert!(
            (volume - 0.6875).abs() < 1e-12,
            "L字柱の体積は shape.rs の合成体テストと同じ0.6875のはず: {volume}"
        );
    }

    /// **すでに凸な形状(箱)は分解せず単一パーツのまま**(課題要件6)。
    /// 分解が既存挙動を壊さないことの回帰テスト。
    #[test]
    fn convex_box_mesh_is_not_decomposed() {
        let (v, t) = box_mesh(1.5);
        let parts = decompose_mesh(&v, &t);
        assert_eq!(
            parts.len(),
            1,
            "凸な箱は1パーツのままのはず: {}",
            parts.len()
        );
        let mp = parts[0].mass_properties().unwrap();
        let expected_volume = (2.0 * 1.5f64).powi(3);
        assert!(
            (mp.volume - expected_volume).abs() < 1e-9,
            "体積は箱の解析解と一致するはず: {} vs {expected_volume}",
            mp.volume
        );
    }

    /// **問題の実演**: 非凸なL字メッシュを「頂点だけの凸包」として素朴に
    /// 扱うと、真の体積(0.6875)より明確に過大な値になる(課題要件7)。
    /// 素朴な凸包の体積は2D凸包(五角形、面積1.8125)×厚み0.5=0.90625。
    #[test]
    fn naive_convex_hull_of_the_l_shape_overestimates_volume() {
        let (v, _t) = l_shaped_prism_mesh();
        let naive_hull = convex_hull(&v).unwrap();
        let naive_volume = naive_hull.mass_properties().unwrap().volume;
        let true_volume = 0.6875;
        assert!(
            (naive_volume - 0.90625).abs() < 1e-9,
            "素朴な凸包の体積は五角形×厚みの0.90625のはず: {naive_volume}"
        );
        assert!(
            naive_volume > true_volume * 1.2,
            "素朴な凸包は真の体積より20%以上過大評価するはず: naive={naive_volume} true={true_volume}"
        );
    }

    /// **本体**: L字メッシュの近似凸分解が、真の体積(0.6875)に近い値を
    /// 与え、かつ素朴な凸包(0.90625)よりも明確に真の値へ近いこと(課題要件6)。
    #[test]
    fn l_shape_decomposition_recovers_close_to_the_true_volume() {
        let (v, t) = l_shaped_prism_mesh();
        let parts = decompose_mesh(&v, &t);
        assert!(
            parts.len() >= 2,
            "凹んだL字は複数パーツへ分解されるはず: {}",
            parts.len()
        );
        assert!(
            parts.len() <= 8,
            "パーツ数は上限(2^MAX_DEPTH=8)を超えないはず: {}",
            parts.len()
        );

        let decomposed_volume: f64 = parts
            .iter()
            .map(|h| h.mass_properties().unwrap().volume)
            .sum();
        let true_volume = 0.6875;
        let naive_volume = 0.90625;

        // 真の体積からの相対誤差が10%未満(=軸並行の単純な分割でも十分な精度)。
        let rel_error = (decomposed_volume - true_volume).abs() / true_volume;
        assert!(
            rel_error < 0.10,
            "分解後の体積和は真の体積に近いはず: decomposed={decomposed_volume} \
             true={true_volume} rel_error={rel_error:.4}"
        );
        // 素朴な凸包よりは確実に真の値へ近いこと(退化した検証にしない)。
        assert!(
            (decomposed_volume - true_volume).abs() < (naive_volume - true_volume) * 0.5,
            "分解後の体積和は素朴な凸包よりも真の値へ有意に近いはず: \
             decomposed={decomposed_volume} naive={naive_volume} true={true_volume}"
        );
    }

    /// 分解結果の各パーツが元のメッシュの凸包の内部に収まっていること
    /// (=分解が「勝手に外側へはみ出た」変な形を作っていないことの健全性チェック)。
    #[test]
    fn decomposed_parts_stay_within_the_original_hull() {
        let (v, t) = l_shaped_prism_mesh();
        let whole_hull = convex_hull(&v).unwrap();
        let parts = decompose_mesh(&v, &t);
        for part in &parts {
            for &vertex in &part.vertices {
                assert!(
                    whole_hull.contains(vertex, 1e-6),
                    "分解パーツの頂点は元の凸包の内部にあるはず: {vertex:?}"
                );
            }
        }
    }

    /// 三角形が空(頂点だけの入力)なら、凹みを判定できないので凸包1つへ
    /// フォールバックする(`Shape::ConvexMesh`と同じ扱いへの後方互換)。
    #[test]
    fn vertex_only_input_falls_back_to_the_convex_hull() {
        let (v, _t) = box_mesh(1.0);
        let parts = decompose_mesh(&v, &[]);
        assert_eq!(parts.len(), 1);
    }

    /// 退化入力(3次元の広がりが無い)は空の`Vec`を返す。
    #[test]
    fn degenerate_input_returns_no_parts() {
        assert!(decompose_mesh(&[], &[]).is_empty());
        let flat = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        assert!(decompose_mesh(&flat, &[[0, 1, 2]]).is_empty());
    }
}
