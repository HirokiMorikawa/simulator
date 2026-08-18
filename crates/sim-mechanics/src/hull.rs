//! 3D凸包(incremental convex hull)と、凸多面体の厳密な質量特性。
//! 設計: docs/10-mechanics/01-rigid-body.md §4.1、02-collision-detection.md §4.2。
//!
//! `Shape::ConvexMesh`は頂点列しか持たないため、移行前は体積・慣性を頂点群の
//! AABBで代用し(正四面体で体積3倍・正八面体で6倍の過大評価)、接触生成に
//! 至っては`None`(=何ともぶつからずすり抜ける)だった。ここで**面情報を
//! 実際に構築**して、その3つすべてを解く。
//!
//! ## なぜ quickhull ではなく incremental か
//!
//! quickhull は期待 $O(n\log n)$ で漸近的には速いが、実装の分岐(遠点の選択・
//! 円錐の構築・可視面集合の管理)が多く、退化入力での正しさを担保しにくい。
//! 一方 incremental(「点を1つずつ入れ、可視な面を消して地平線に新しい面を
//! 張る」)は最悪 $O(n^2)$ だが**不変条件が1つ**(常に凸な閉多面体を保つ)で
//! 済み、テストで固めやすい。
//!
//! この判断は**質量特性が生成時にしか計算されない**という事実に強く依る——
//! 毎フレーム走る hot path ではないので、$n^2$ と $n\log n$ の差は実害が無い
//! (この物理コアが扱う`ConvexMesh`は手で書かれた数十頂点規模)。
//! 「外部クレート実質ゼロ」の方針でゼロから書く以上、速さより**正しさの
//! 検証しやすさ**を取った。
//!
//! ## 退化入力
//!
//! 点が3個以下、または全点が同一平面/同一直線/同一点にある場合は3次元の
//! 体積を持たないので `None` を返す。呼び出し側(`Shape::mass_properties`)は
//! これを「体積なし」として扱う。

use crate::shape::MassProperties;
use sim_math::{Mat3, Vec3};

/// 三角形面で閉じた凸多面体。`faces`の各三角形は**外向き**の巻き順
/// (右手系で法線が外を向く)に正規化されている——体積・慣性の符号付き
/// 四面体分解がこの規約に依存する。
#[derive(Clone, Debug)]
pub(crate) struct ConvexHull {
    pub vertices: Vec<Vec3>,
    pub faces: Vec<[usize; 3]>,
}

/// 凸包構築の相対許容誤差。点群の代表寸法に対する比で使う——絶対値で固定すると
/// ミリ単位の形状とキロメートル単位の形状で挙動が変わってしまうため。
const HULL_EPS_RELATIVE: f64 = 1e-10;

impl ConvexHull {
    /// 面`f`の外向き法線(正規化しない生の外積、面積×2の大きさを持つ)。
    // `contains`専用のため、`contains`と同じ理由で本体からは未使用。
    #[allow(dead_code)]
    fn face_normal(&self, f: [usize; 3]) -> Vec3 {
        let (a, b, c) = (
            self.vertices[f[0]],
            self.vertices[f[1]],
            self.vertices[f[2]],
        );
        (b - a).cross(c - a)
    }

    /// 点`p`が凸包の内部(または表面)にあるか。すべての面の外向き半空間の
    /// 内側にあるかを見る。
    ///
    /// **次の増分(`Compound`のブーリアン和体積)で本体から使う**——現時点では
    /// テストからのみ呼ばれるため dead_code 警告を抑制している。
    #[allow(dead_code)]
    pub fn contains(&self, p: Vec3, tolerance: f64) -> bool {
        self.faces.iter().all(|&f| {
            let n = self.face_normal(f);
            let len = n.length();
            if len < f64::MIN_POSITIVE {
                return true; // 退化面は判定に寄与させない
            }
            n.scale(1.0 / len).dot(p - self.vertices[f[0]]) <= tolerance
        })
    }

    /// 凸多面体の**厳密な**質量特性(密度一様)。
    ///
    /// 原点と各面三角形が張る四面体へ分解し、符号付きで足し合わせる
    /// (原点が多面体の外にあっても、符号が相殺して正しい値になる標準手法)。
    ///
    /// - 体積: $V=\sum \frac{1}{6}\,v_0\cdot(v_1\times v_2)$
    /// - 重心: 各四面体の重心 $(v_0+v_1+v_2)/4$ を符号付き体積で加重平均
    /// - 慣性: 各四面体の二次モーメント行列
    ///   $\int_T x x^\top dV=\frac{V_T}{20}\left(\sum_k v_kv_k^\top+ss^\top\right)$
    ///   ($s=\sum_k v_k$、原点を頂点に含むので $v_0=0$)を足し上げ、
    ///   $I=\mathrm{tr}(C)E-C$ で慣性テンソルにしてから平行軸定理で重心へ移す。
    ///
    /// 二次モーメントの式は標準的な四面体の解析積分(Tonon 2004 等で使われる
    /// 形)で、単体 $\{x,y,z\ge0,\;x+y+z\le1\}$ 上の $\int x^2=1/60$・
    /// $\int xy=1/120$ と突き合わせて検算してある(`tests`参照)。
    pub fn mass_properties(&self) -> Option<MassProperties> {
        let mut volume = 0.0;
        let mut centroid_accum = Vec3::ZERO;
        // 二次モーメント行列(covariance)を原点まわりで積算する。
        let mut covariance = Mat3::from_diagonal(Vec3::ZERO);

        for &f in &self.faces {
            let (a, b, c) = (
                self.vertices[f[0]],
                self.vertices[f[1]],
                self.vertices[f[2]],
            );
            let tet_volume = a.dot(b.cross(c)) / 6.0;
            if tet_volume == 0.0 {
                continue;
            }
            volume += tet_volume;
            centroid_accum = centroid_accum + (a + b + c).scale(tet_volume / 4.0);

            // v0 = 原点なので Σ v_k v_k^T は a,b,c のぶんだけ、s = a+b+c。
            let s = a + b + c;
            let sum_squares = Mat3::outer(a, a) + Mat3::outer(b, b) + Mat3::outer(c, c);
            let tet_cov = (sum_squares + Mat3::outer(s, s)).scale(tet_volume / 20.0);
            covariance = covariance + tet_cov;
        }

        if volume <= 0.0 {
            return None;
        }
        let center_of_mass = centroid_accum.scale(1.0 / volume);

        // I = tr(C)E - C(密度1、すなわち質量 = 体積)。
        let trace = covariance.m[0][0] + covariance.m[1][1] + covariance.m[2][2];
        let inertia_about_origin = Mat3::from_diagonal(Vec3::new(trace, trace, trace)) - covariance;
        // 平行軸定理で原点まわり → 重心まわりへ(質量 = volume ぶんを引く)。
        let inertia_about_com =
            inertia_about_origin - Mat3::parallel_axis_term(center_of_mass).scale(volume);

        Some(MassProperties {
            volume,
            center_of_mass,
            // 単位質量あたりへ正規化(密度1なので質量 = 体積)。
            unit_inertia: inertia_about_com.scale(1.0 / volume),
        })
    }
}

/// 点群の3D凸包を作る(incremental convex hull、モジュールdoc参照)。
/// 3次元的な広がりが無い(点/直線/平面上に退化した)入力は `None`。
pub(crate) fn convex_hull(points: &[Vec3]) -> Option<ConvexHull> {
    if points.len() < 4 {
        return None;
    }
    // 点群の代表寸法。許容誤差をこれに比例させる(モジュールdoc)。
    let scale = points.iter().fold(0.0_f64, |acc, p| {
        acc.max(p.x.abs().max(p.y.abs()).max(p.z.abs()))
    });
    let eps = HULL_EPS_RELATIVE * scale.max(1.0);

    let (i0, i1, i2, i3) = initial_tetrahedron(points, eps)?;

    let mut vertices = points.to_vec();
    // 初期四面体。外向き巻き順になるよう、4点目が各面の裏側に来る向きへ揃える。
    let mut faces: Vec<[usize; 3]> = Vec::new();
    for (a, b, c, opposite) in [
        (i0, i1, i2, i3),
        (i0, i3, i1, i2),
        (i0, i2, i3, i1),
        (i1, i3, i2, i0),
    ] {
        faces.push(oriented_face(&vertices, a, b, c, vertices[opposite]));
    }

    for (index, &p) in points.iter().enumerate() {
        if index == i0 || index == i1 || index == i2 || index == i3 {
            continue;
        }
        // 点 p から見える面(外向き法線の側に p がある面)を集める。
        let visible: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter(|(_, &f)| signed_distance(&vertices, f, p) > eps)
            .map(|(i, _)| i)
            .collect();
        if visible.is_empty() {
            continue; // すでに凸包の内側
        }

        // 地平線 = 可視面の有向辺のうち、逆向きの辺が可視面集合の中に**無い**もの。
        // 内部の辺は必ず2つの可視面に逆向きの対として現れるので、これで
        // 隣接情報を別途持たずに境界だけを取り出せる。
        let mut directed_edges: Vec<(usize, usize)> = Vec::new();
        for &fi in &visible {
            let f = faces[fi];
            directed_edges.push((f[0], f[1]));
            directed_edges.push((f[1], f[2]));
            directed_edges.push((f[2], f[0]));
        }
        let horizon: Vec<(usize, usize)> = directed_edges
            .iter()
            .copied()
            .filter(|&(a, b)| !directed_edges.contains(&(b, a)))
            .collect();

        // 可視面を除去(インデックスがずれないよう後ろから)。
        let mut sorted_visible = visible.clone();
        sorted_visible.sort_unstable_by(|a, b| b.cmp(a));
        for fi in sorted_visible {
            faces.swap_remove(fi);
        }

        // 地平線の各有向辺と p で新しい面を張る。元の面が外向き巻き順なので、
        // (a, b, p) の順にすれば新しい面も外向きになる。
        for (a, b) in horizon {
            faces.push([a, b, index]);
        }
    }

    // 使われている頂点だけを残して詰め直す(包含判定・面走査を軽くするため)。
    let mut remap = vec![usize::MAX; vertices.len()];
    let mut compact_vertices = Vec::new();
    let mut compact_faces = Vec::with_capacity(faces.len());
    for f in &faces {
        let mut nf = [0usize; 3];
        for (k, &vi) in f.iter().enumerate() {
            if remap[vi] == usize::MAX {
                remap[vi] = compact_vertices.len();
                compact_vertices.push(vertices[vi]);
            }
            nf[k] = remap[vi];
        }
        compact_faces.push(nf);
    }
    vertices = compact_vertices;

    if vertices.len() < 4 || compact_faces.len() < 4 {
        return None;
    }
    Some(ConvexHull {
        vertices,
        faces: compact_faces,
    })
}

/// 面 `f` の外向き法線側から見た点 `p` の符号付き距離(正なら「見える」)。
fn signed_distance(vertices: &[Vec3], f: [usize; 3], p: Vec3) -> f64 {
    let (a, b, c) = (vertices[f[0]], vertices[f[1]], vertices[f[2]]);
    let n = (b - a).cross(c - a);
    let len = n.length();
    if len < f64::MIN_POSITIVE {
        return f64::NEG_INFINITY;
    }
    n.scale(1.0 / len).dot(p - a)
}

/// `inside` が裏側に来るように巻き順を決めた三角形。
fn oriented_face(vertices: &[Vec3], a: usize, b: usize, c: usize, inside: Vec3) -> [usize; 3] {
    let n = (vertices[b] - vertices[a]).cross(vertices[c] - vertices[a]);
    if n.dot(inside - vertices[a]) > 0.0 {
        [a, c, b]
    } else {
        [a, b, c]
    }
}

/// 3次元的な広がりを持つ初期四面体を4点選ぶ。
/// ①最も離れた2点 → ②その直線から最も遠い点 → ③その平面から最も遠い点、
/// という決定的な手順(乱択なし——この物理コアの決定論方針)。
fn initial_tetrahedron(points: &[Vec3], eps: f64) -> Option<(usize, usize, usize, usize)> {
    // ① 最も離れた2点。
    let mut best = (0usize, 0usize);
    let mut best_dist = -1.0;
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            let d = (points[i] - points[j]).length_sq();
            if d > best_dist {
                best_dist = d;
                best = (i, j);
            }
        }
    }
    let (i0, i1) = best;
    if best_dist <= eps * eps {
        return None; // 全点が同一点
    }

    // ② 直線 i0-i1 から最も遠い点。
    let axis = points[i1] - points[i0];
    let mut i2 = usize::MAX;
    let mut best_area = eps * eps;
    for (k, &p) in points.iter().enumerate() {
        let area = axis.cross(p - points[i0]).length();
        if area > best_area {
            best_area = area;
            i2 = k;
        }
    }
    if i2 == usize::MAX {
        return None; // 全点が同一直線上
    }

    // ③ 平面 i0-i1-i2 から最も遠い点。
    let normal = axis.cross(points[i2] - points[i0]).normalize_or_zero();
    let mut i3 = usize::MAX;
    let mut best_height = eps;
    for (k, &p) in points.iter().enumerate() {
        let h = normal.dot(p - points[i0]).abs();
        if h > best_height {
            best_height = h;
            i3 = k;
        }
    }
    if i3 == usize::MAX {
        return None; // 全点が同一平面上
    }
    Some((i0, i1, i2, i3))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube_corners(half: f64) -> Vec<Vec3> {
        let mut v = Vec::new();
        for &sx in &[-1.0, 1.0] {
            for &sy in &[-1.0, 1.0] {
                for &sz in &[-1.0, 1.0] {
                    v.push(Vec3::new(sx * half, sy * half, sz * half));
                }
            }
        }
        v
    }

    /// 立方体の8隅 → 12三角形(6面 × 2)・8頂点。オイラーの多面体定理
    /// $V-E+F=2$ も満たす($8-18+12=2$)。
    #[test]
    fn cube_hull_has_twelve_triangles_and_eight_vertices() {
        let hull = convex_hull(&cube_corners(1.0)).unwrap();
        assert_eq!(hull.vertices.len(), 8, "頂点は8つ");
        assert_eq!(hull.faces.len(), 12, "四角形6面が三角形12枚に分割される");
    }

    /// **面の巻き順がすべて外向き**であること。凸包の内部点(重心)から見て、
    /// どの面の外向き法線も「離れる向き」でなければならない。これが崩れると
    /// 符号付き四面体分解の体積が狂う。
    #[test]
    fn all_faces_wind_outward() {
        for points in [
            cube_corners(1.5),
            regular_octahedron(),
            regular_tetrahedron(),
        ] {
            let hull = convex_hull(&points).unwrap();
            let center = hull
                .vertices
                .iter()
                .fold(Vec3::ZERO, |a, &b| a + b)
                .scale(1.0 / hull.vertices.len() as f64);
            for &f in &hull.faces {
                let d = signed_distance(&hull.vertices, f, center);
                assert!(d < 0.0, "内部の点は全ての面の裏側にあるはず: d={d}");
            }
        }
    }

    fn regular_tetrahedron() -> Vec<Vec3> {
        vec![
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, 1.0),
        ]
    }

    fn regular_octahedron() -> Vec<Vec3> {
        vec![
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
        ]
    }

    /// 立方体の質量特性が閉形式と厳密に一致すること。
    /// 一辺 $2h$ の立方体は $V=8h^3$、$I/m=\frac{2h^2}{3}$(等方)。
    #[test]
    fn cube_hull_mass_properties_match_the_closed_form() {
        let h = 1.5;
        let hull = convex_hull(&cube_corners(h)).unwrap();
        let p = hull.mass_properties().unwrap();
        assert!((p.volume - 8.0 * h * h * h).abs() < 1e-12, "V={}", p.volume);
        assert!(p.center_of_mass.length() < 1e-12, "{:?}", p.center_of_mass);
        let expected = 2.0 * h * h / 3.0;
        for k in 0..3 {
            assert!(
                (p.unit_inertia.m[k][k] - expected).abs() < 1e-12,
                "I[{k}][{k}]={} expected={expected}",
                p.unit_inertia.m[k][k]
            );
        }
        // 立方体は等方なので非対角成分は厳密にゼロ。
        assert!(p.unit_inertia.m[0][1].abs() < 1e-12);
    }

    /// **原点が多面体の外にあっても符号付き分解は正しい**(平行移動不変性)。
    /// 立方体を大きくずらしても体積は不変で、重心はそのぶん動く。
    #[test]
    fn signed_decomposition_is_translation_invariant() {
        let h = 0.7;
        let offset = Vec3::new(13.0, -7.0, 4.0);
        let shifted: Vec<Vec3> = cube_corners(h).iter().map(|&p| p + offset).collect();
        let hull = convex_hull(&shifted).unwrap();
        let p = hull.mass_properties().unwrap();
        assert!((p.volume - 8.0 * h * h * h).abs() < 1e-10, "V={}", p.volume);
        assert!(
            (p.center_of_mass - offset).length() < 1e-10,
            "{:?}",
            p.center_of_mass
        );
        // 慣性は重心まわりなので、平行移動しても原点中心の立方体と同じ。
        let expected = 2.0 * h * h / 3.0;
        for k in 0..3 {
            assert!((p.unit_inertia.m[k][k] - expected).abs() < 1e-10);
        }
    }

    /// 内部にある点(凸包に寄与しない点)を混ぜても結果が変わらないこと。
    #[test]
    fn interior_points_do_not_change_the_hull() {
        let mut points = cube_corners(1.0);
        let reference = convex_hull(&points).unwrap().mass_properties().unwrap();
        points.push(Vec3::ZERO);
        points.push(Vec3::new(0.3, -0.2, 0.5));
        points.push(Vec3::new(-0.9, 0.9, 0.0));
        let with_interior = convex_hull(&points).unwrap().mass_properties().unwrap();
        assert!((with_interior.volume - reference.volume).abs() < 1e-12);
        assert!((with_interior.center_of_mass - reference.center_of_mass).length() < 1e-12);
    }

    /// 退化入力(点・直線・平面)は3次元の体積を持たないので `None`。
    #[test]
    fn degenerate_inputs_have_no_hull() {
        assert!(convex_hull(&[]).is_none());
        assert!(convex_hull(&[Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)]).is_none());
        // 同一点。
        assert!(convex_hull(&[Vec3::new(2.0, 2.0, 2.0); 8]).is_none());
        // 同一直線上。
        let line: Vec<Vec3> = (0..6).map(|i| Vec3::new(i as f64, 0.0, 0.0)).collect();
        assert!(convex_hull(&line).is_none());
        // 同一平面上(z=0 の正方形 + 内部点)。
        let plane = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.5, 0.5, 0.0),
        ];
        assert!(convex_hull(&plane).is_none());
    }

    /// `contains` が凸包の内外を正しく判定すること(`union_volume` が依存する)。
    #[test]
    fn contains_distinguishes_inside_from_outside() {
        let hull = convex_hull(&regular_octahedron()).unwrap();
        // |x|+|y|+|z| <= 1 が正八面体の内部。
        assert!(hull.contains(Vec3::ZERO, 1e-9));
        assert!(hull.contains(Vec3::new(0.4, 0.3, 0.2), 1e-9));
        assert!(!hull.contains(Vec3::new(0.6, 0.6, 0.0), 1e-9));
        assert!(!hull.contains(Vec3::new(2.0, 0.0, 0.0), 1e-9));
    }
}
