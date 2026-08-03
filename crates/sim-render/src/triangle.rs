//! 解析形状: 三角形(+ 三角形メッシュ)。設計 docs/17-rendering/02-path-tracing.md §4
//! 「BVH: 三角形メッシュ + 解析形状(球・平面)」、**群6で追加**。
//!
//! 移行前は`Sphere`と`Quad`しか無く、三角形メッシュを「頂点バッファ・インデックス
//! バッファ・法線補間といった付随機構が一式必要になる」という理由で見送っていた。
//! 群6ではそれを実際に用意する:
//!
//! - `Triangle`は`Primitive`の一員として**3頂点を直接持つ**(インデックス参照では
//!   なく展開済み)。BVHのリーフから何度も引かれるホットパスで、間接参照を1段
//!   挟むより素直で、`Primitive`が`Copy`であるという既存の性質も保てる。
//! - `TriangleMesh`は「頂点バッファ + インデックスバッファ(+ 任意の頂点法線)」を
//!   持つ**構築時の入れ物**で、`triangles()`で`Primitive`の列へ展開する。
//!   メッシュの形で持っておきたいのは、頂点法線の平均化(`with_smooth_normals`)を
//!   共有頂点に対して行うため。
//!
//! **交差判定はMöller–Trumbore法**(行列式を使わず、スカラー三重積だけで
//! バリセントリック座標$(u,v)$と距離$t$を同時に求める標準的な手法)。
//!
//! **法線**: 頂点法線を持つ場合はバリセントリック補間した**シェーディング法線**を
//! 返し(スムーズシェーディング)、持たない場合は幾何法線(フラットシェーディング)を
//! 返す。どちらも`Quad`と同じく**常に入射レイと逆を向ける**(両面材質)——
//! メッシュの表裏を材質側で区別する機構が無いため。
//!
//! **残る縮約**: UV座標・テクスチャ・頂点カラーは持たない(材質は`SceneObject`
//! 単位で1つ)。メッシュファイル(OBJ/glTF)の読み込みも対象外——外部フォーマットの
//! パーサはレンダラの検証に必要な部分ではなく、依存を足さない方針とも合わない。
//! 代わりに解析的に生成できるメッシュ(`icosphere`・`grid`)をヘルパとして持つ。

use crate::ray::Ray;
use crate::sphere::Hit;
use sim_math::Vec3;

/// 三角形1枚。頂点は反時計回り(右手系)を表とするが、法線は常にレイ側へ向けるため
/// 順序は交差判定の結果に影響しない(モジュールdoc参照)。
#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub v0: Vec3,
    pub v1: Vec3,
    pub v2: Vec3,
    /// 頂点法線(スムーズシェーディング用)。`None`なら幾何法線を使う。
    pub vertex_normals: Option<[Vec3; 3]>,
}

impl Triangle {
    pub fn new(v0: Vec3, v1: Vec3, v2: Vec3) -> Triangle {
        Triangle {
            v0,
            v1,
            v2,
            vertex_normals: None,
        }
    }

    /// 幾何法線(正規化前の外積 = 2倍の面積ベクトル)。
    fn area_vector(&self) -> Vec3 {
        (self.v1 - self.v0).cross(self.v2 - self.v0).scale(0.5)
    }

    /// 面積。
    pub fn area(&self) -> f64 {
        self.area_vector().length()
    }

    /// 幾何法線(単位ベクトル、頂点順序が定める向き)。退化三角形ではゼロ。
    pub fn geometric_normal(&self) -> Vec3 {
        self.area_vector().normalize_or_zero()
    }

    /// Möller–Trumbore法によるレイ-三角形交差(モジュールdoc参照)。
    pub fn intersect(&self, ray: &Ray, t_min: f64) -> Option<Hit> {
        const EPSILON: f64 = 1e-12;
        let edge1 = self.v1 - self.v0;
        let edge2 = self.v2 - self.v0;
        let pvec = ray.direction.cross(edge2);
        let det = edge1.dot(pvec);
        // 両面判定なので`det`の符号は問わない。0に近い = レイが三角形の面と平行。
        if det.abs() < EPSILON {
            return None;
        }
        let inv_det = 1.0 / det;
        let tvec = ray.origin - self.v0;
        let u = tvec.dot(pvec) * inv_det;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let qvec = tvec.cross(edge1);
        let v = ray.direction.dot(qvec) * inv_det;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = edge2.dot(qvec) * inv_det;
        if t < t_min {
            return None;
        }

        let normal = match self.vertex_normals {
            // バリセントリック補間(w=1-u-v が v0 の重み)。
            Some([n0, n1, n2]) => {
                (n0.scale(1.0 - u - v) + n1.scale(u) + n2.scale(v)).normalize_or_zero()
            }
            None => self.geometric_normal(),
        };
        // 常に入射レイと逆を向ける(両面材質、モジュールdoc参照)。
        let oriented = if normal.dot(ray.direction) > 0.0 {
            -normal
        } else {
            normal
        };
        Some(Hit {
            t,
            point: ray.at(t),
            normal: oriented,
        })
    }

    /// 軸並行境界ボックス。
    pub fn bounds(&self) -> (Vec3, Vec3) {
        let min = Vec3::new(
            self.v0.x.min(self.v1.x).min(self.v2.x),
            self.v0.y.min(self.v1.y).min(self.v2.y),
            self.v0.z.min(self.v1.z).min(self.v2.z),
        );
        let max = Vec3::new(
            self.v0.x.max(self.v1.x).max(self.v2.x),
            self.v0.y.max(self.v1.y).max(self.v2.y),
            self.v0.z.max(self.v1.z).max(self.v2.z),
        );
        (min, max)
    }

    /// 重心(BVHの分割基準)。
    pub fn centroid(&self) -> Vec3 {
        (self.v0 + self.v1 + self.v2).scale(1.0 / 3.0)
    }
}

/// 三角形メッシュ(頂点バッファ + インデックスバッファ + 任意の頂点法線、
/// モジュールdoc参照)。
#[derive(Clone, Debug, Default)]
pub struct TriangleMesh {
    pub vertices: Vec<Vec3>,
    /// 3つ組で1三角形(`indices.len() % 3 == 0`)。
    pub indices: Vec<u32>,
    /// 頂点法線(`vertices`と同じ長さ)。空ならフラットシェーディング。
    pub normals: Vec<Vec3>,
}

impl TriangleMesh {
    pub fn new(vertices: Vec<Vec3>, indices: Vec<u32>) -> TriangleMesh {
        assert_eq!(indices.len() % 3, 0, "indices must come in triples");
        TriangleMesh {
            vertices,
            indices,
            normals: Vec::new(),
        }
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// **面積重み付きの頂点法線**を計算して持たせる(スムーズシェーディング)。
    /// 各頂点に、それを共有する三角形の法線を**面積に比例した重み**で足し込む
    /// ——単純平均だと細かい三角形が過剰に効くため。外積の長さがそのまま面積の
    /// 2倍なので、正規化前の外積を足すだけで面積重み付けになる。
    pub fn with_smooth_normals(mut self) -> TriangleMesh {
        let mut normals = vec![Vec3::ZERO; self.vertices.len()];
        for tri in self.indices.chunks_exact(3) {
            let (a, b, c) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
            let area_vector =
                (self.vertices[b] - self.vertices[a]).cross(self.vertices[c] - self.vertices[a]);
            for &i in &[a, b, c] {
                normals[i] = normals[i] + area_vector;
            }
        }
        self.normals = normals.into_iter().map(|n| n.normalize_or_zero()).collect();
        self
    }

    /// `Triangle`の列へ展開する(`Primitive`へ包んでBVHへ渡すための形)。
    pub fn triangles(&self) -> Vec<Triangle> {
        self.indices
            .chunks_exact(3)
            .map(|tri| {
                let (a, b, c) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
                let mut t = Triangle::new(self.vertices[a], self.vertices[b], self.vertices[c]);
                if !self.normals.is_empty() {
                    t.vertex_normals = Some([self.normals[a], self.normals[b], self.normals[c]]);
                }
                t
            })
            .collect()
    }

    /// 表面積の合計。
    pub fn surface_area(&self) -> f64 {
        self.triangles().iter().map(|t| t.area()).sum()
    }

    /// 正20面体を`subdivisions`回細分して球面へ射影したメッシュ(アイコスフィア)。
    /// 三角形の大きさが揃うので、球の解析形状(`Sphere`)と突き合わせる検証に向く。
    pub fn icosphere(center: Vec3, radius: f64, subdivisions: u32) -> TriangleMesh {
        // 正20面体の頂点(黄金比)。
        let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let mut vertices: Vec<Vec3> = vec![
            Vec3::new(-1.0, phi, 0.0),
            Vec3::new(1.0, phi, 0.0),
            Vec3::new(-1.0, -phi, 0.0),
            Vec3::new(1.0, -phi, 0.0),
            Vec3::new(0.0, -1.0, phi),
            Vec3::new(0.0, 1.0, phi),
            Vec3::new(0.0, -1.0, -phi),
            Vec3::new(0.0, 1.0, -phi),
            Vec3::new(phi, 0.0, -1.0),
            Vec3::new(phi, 0.0, 1.0),
            Vec3::new(-phi, 0.0, -1.0),
            Vec3::new(-phi, 0.0, 1.0),
        ]
        .into_iter()
        .map(|v| v.normalize_or_zero())
        .collect();
        // 正20面体の20面(5面ずつ4帯: 北極冠・北中緯度・南極冠・南中緯度)。
        let mut indices: Vec<u32> = vec![
            0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, //
            1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8, //
            3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, //
            4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1,
        ];

        for _ in 0..subdivisions {
            let mut next = Vec::with_capacity(indices.len() * 4);
            // 辺の中点をキャッシュして共有頂点を重複させない(共有しないと
            // `with_smooth_normals`が機能せず、継ぎ目が見える)。
            let mut midpoints: std::collections::HashMap<(u32, u32), u32> =
                std::collections::HashMap::new();
            let midpoint = |vertices: &mut Vec<Vec3>,
                            cache: &mut std::collections::HashMap<(u32, u32), u32>,
                            a: u32,
                            b: u32|
             -> u32 {
                let key = if a < b { (a, b) } else { (b, a) };
                if let Some(&existing) = cache.get(&key) {
                    return existing;
                }
                let m = (vertices[a as usize] + vertices[b as usize])
                    .scale(0.5)
                    .normalize_or_zero();
                vertices.push(m);
                let index = (vertices.len() - 1) as u32;
                cache.insert(key, index);
                index
            };
            for tri in indices.chunks_exact(3) {
                let (a, b, c) = (tri[0], tri[1], tri[2]);
                let ab = midpoint(&mut vertices, &mut midpoints, a, b);
                let bc = midpoint(&mut vertices, &mut midpoints, b, c);
                let ca = midpoint(&mut vertices, &mut midpoints, c, a);
                next.extend_from_slice(&[a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca]);
            }
            indices = next;
        }

        let vertices = vertices
            .into_iter()
            .map(|v| center + v.scale(radius))
            .collect();
        TriangleMesh::new(vertices, indices)
    }

    /// 平面グリッド(`nx`×`ny`分割の矩形を2三角形/セルで敷き詰める)。
    /// `origin`を隅として`edge_u`・`edge_v`が張る平行四辺形を覆う。
    pub fn grid(origin: Vec3, edge_u: Vec3, edge_v: Vec3, nx: u32, ny: u32) -> TriangleMesh {
        let mut vertices = Vec::with_capacity(((nx + 1) * (ny + 1)) as usize);
        for j in 0..=ny {
            for i in 0..=nx {
                let u = i as f64 / nx as f64;
                let v = j as f64 / ny as f64;
                vertices.push(origin + edge_u.scale(u) + edge_v.scale(v));
            }
        }
        let mut indices = Vec::with_capacity((nx * ny * 6) as usize);
        let stride = nx + 1;
        for j in 0..ny {
            for i in 0..nx {
                let a = j * stride + i;
                let b = a + 1;
                let c = a + stride;
                let d = c + 1;
                indices.extend_from_slice(&[a, c, b, b, c, d]);
            }
        }
        TriangleMesh::new(vertices, indices)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::SimRng;

    /// Möller–Trumbore法が、手で決めた三角形と交点に対して解析的に正しい
    /// $(t,$ 交点$)$ を返し、三角形の外を通るレイは確実に外すこと。
    #[test]
    fn moller_trumbore_hits_inside_and_misses_outside_the_triangle() {
        // z=5 平面上の直角三角形(0,0)-(2,0)-(0,2)。
        let tri = Triangle::new(
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(2.0, 0.0, 5.0),
            Vec3::new(0.0, 2.0, 5.0),
        );
        let shoot = |x: f64, y: f64| {
            let ray = Ray::new(Vec3::new(x, y, 0.0), Vec3::new(0.0, 0.0, 1.0));
            tri.intersect(&ray, 1e-6)
        };

        // 内部の点。
        let hit = shoot(0.5, 0.5).expect("内部の点はヒットするはず");
        assert!((hit.t - 5.0).abs() < 1e-12);
        assert!((hit.point - Vec3::new(0.5, 0.5, 5.0)).length() < 1e-12);
        // 法線はレイと逆(-z)を向く(両面材質、モジュールdoc参照)。
        assert!((hit.normal - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-12);

        // 斜辺の外側(u+v>1)、および負のバリセントリック座標。
        assert!(shoot(1.5, 1.5).is_none(), "斜辺の外は外れるはず");
        assert!(shoot(-0.1, 0.5).is_none(), "u<0 は外れるはず");
        assert!(shoot(0.5, -0.1).is_none(), "v<0 は外れるはず");

        // 面と平行なレイは行列式が0で外れる。
        let parallel = Ray::new(Vec3::new(0.5, 0.5, 5.0), Vec3::new(1.0, 0.0, 0.0));
        assert!(tri.intersect(&parallel, 1e-6).is_none());

        // 背面(z=10から-z方向)でも当たる(両面)。
        let behind = Ray::new(Vec3::new(0.5, 0.5, 10.0), Vec3::new(0.0, 0.0, -1.0));
        let back_hit = behind.direction;
        let hit = tri.intersect(&behind, 1e-6).expect("背面もヒットするはず");
        assert!((hit.t - 5.0).abs() < 1e-12);
        assert!(hit.normal.dot(back_hit) < 0.0, "法線は常にレイと逆向き");
    }

    /// 面積・重心・幾何法線が解析値と一致する。
    #[test]
    fn triangle_area_centroid_and_normal_match_analytic_values() {
        let tri = Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(0.0, 4.0, 0.0),
        );
        assert!((tri.area() - 6.0).abs() < 1e-12, "直角三角形の面積 = 3*4/2");
        assert!((tri.centroid() - Vec3::new(1.0, 4.0 / 3.0, 0.0)).length() < 1e-12);
        assert!((tri.geometric_normal() - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12);
    }

    /// **アイコスフィアが実際に球へ収束すること**: 細分を進めると全頂点が半径
    /// ちょうどの球面に乗り(構成上厳密)、表面積が $4\pi r^2$ へ単調に近づく。
    /// 「三角形メッシュを持てるようになった」ことを、解析形状との突き合わせで示す。
    #[test]
    fn icosphere_converges_to_the_analytic_sphere_surface_area() {
        let center = Vec3::new(1.0, -2.0, 3.0);
        let radius = 2.0;
        let exact = 4.0 * std::f64::consts::PI * radius * radius;

        let mut previous_error = f64::INFINITY;
        for subdivisions in 0..4 {
            let mesh = TriangleMesh::icosphere(center, radius, subdivisions);
            assert_eq!(mesh.triangle_count(), 20 * 4usize.pow(subdivisions));
            for v in &mesh.vertices {
                assert!(
                    ((*v - center).length() - radius).abs() < 1e-12,
                    "全頂点が半径ちょうどの球面上にあるはず"
                );
            }
            let error = (mesh.surface_area() - exact).abs() / exact;
            assert!(
                error < previous_error,
                "細分するほど解析値に近づくはず: subdivisions={subdivisions} error={error}"
            );
            previous_error = error;
        }
        // 3回細分(1280三角形)で相対誤差1%未満。
        assert!(
            previous_error < 0.01,
            "3回細分で rel<1% のはず: {previous_error}"
        );
    }

    /// **メッシュのレイ交差が解析形状の球と一致すること**(群6の要): 十分細分した
    /// アイコスフィアへ乱数レイを飛ばし、最近傍ヒット距離が`Sphere`の解析解と
    /// 細分に応じて縮む誤差で一致することを確認する。交差判定・法線の向き・
    /// メッシュ展開のいずれかを間違えていれば破れる。
    #[test]
    fn icosphere_ray_hits_converge_to_the_analytic_sphere_intersection() {
        use crate::sphere::Sphere;
        let center = Vec3::new(0.0, 0.0, 0.0);
        let radius = 1.0;
        let analytic = Sphere { center, radius };

        let mut previous_worst = f64::INFINITY;
        for subdivisions in 1..4 {
            let mesh = TriangleMesh::icosphere(center, radius, subdivisions);
            let triangles = mesh.triangles();
            let mut rng = SimRng::new(7, 1);
            let mut worst: f64 = 0.0;
            let mut tested = 0;
            for _ in 0..200 {
                // 球を確実に貫くレイ(球の外から中心付近へ)。
                let origin =
                    Vec3::new(rng.next_f64() * 2.0 - 1.0, rng.next_f64() * 2.0 - 1.0, -5.0);
                let target = Vec3::new(
                    (rng.next_f64() * 2.0 - 1.0) * 0.5,
                    (rng.next_f64() * 2.0 - 1.0) * 0.5,
                    0.0,
                );
                let ray = Ray::new(origin, (target - origin).normalize_or_zero());
                let Some(exact) = analytic.intersect(&ray, 1e-6) else {
                    continue;
                };
                let mesh_hit = triangles
                    .iter()
                    .filter_map(|t| t.intersect(&ray, 1e-6))
                    .min_by(|a, b| a.t.total_cmp(&b.t));
                let mesh_hit = mesh_hit.expect("解析球に当たるならメッシュにも当たるはず");
                worst = worst.max((mesh_hit.t - exact.t).abs());
                tested += 1;
            }
            assert!(tested > 100, "テストしたレイが少なすぎる: {tested}");
            assert!(
                worst < previous_worst,
                "細分するほどメッシュのヒット距離が解析球へ近づくはず: \
                 subdivisions={subdivisions} worst={worst}"
            );
            previous_worst = worst;
        }
        assert!(
            previous_worst < 0.01,
            "3回細分で最悪誤差<0.01のはず: {previous_worst}"
        );
    }

    /// 頂点法線を持たせるとシェーディング法線が滑らかに変化すること(フラット
    /// シェーディングでは三角形内で一定なのに対し、スムーズでは交点位置に応じて
    /// 変わる)。かつアイコスフィアでは補間法線が**解析球の外向き法線**へ寄る。
    #[test]
    fn smooth_normals_interpolate_toward_the_analytic_sphere_normal() {
        let center = Vec3::ZERO;
        let radius = 1.0;
        let flat = TriangleMesh::icosphere(center, radius, 2);
        let smooth = TriangleMesh::icosphere(center, radius, 2).with_smooth_normals();
        assert_eq!(smooth.normals.len(), smooth.vertices.len());
        // 単位球なら「頂点位置 = 正しい法線」。面積重み付き平均はそれに近づくが
        // **厳密には一致しない**(細分で生じる頂点は周囲の三角形が非対称なため)——
        // 実装検証中に確認した実測ずれは最大0.05程度。厳密一致を要求するテストを
        // 最初に書いて落としたが、誤っていたのは実装ではなく期待値のほうだった。
        let mut worst: f64 = 0.0;
        for (v, n) in smooth.vertices.iter().zip(smooth.normals.iter()) {
            assert!(
                (n.length() - 1.0).abs() < 1e-12,
                "頂点法線は単位ベクトルのはず"
            );
            worst = worst.max((*n - *v).length());
        }
        assert!(
            worst < 0.06,
            "頂点法線は解析法線(=頂点位置)の近傍にあるはず: worst={worst}"
        );

        let ray = Ray::new(Vec3::new(0.13, 0.07, -5.0), Vec3::new(0.0, 0.0, 1.0));
        let nearest = |mesh: &TriangleMesh| {
            mesh.triangles()
                .iter()
                .filter_map(|t| t.intersect(&ray, 1e-6))
                .min_by(|a, b| a.t.total_cmp(&b.t))
                .expect("ヒットするはず")
        };
        let flat_hit = nearest(&flat);
        let smooth_hit = nearest(&smooth);
        // 交点は同じ(幾何は変わらない)。
        assert!((flat_hit.t - smooth_hit.t).abs() < 1e-12);
        // 解析球の(レイ側を向いた)法線。レイは+z方向へ進んで球の手前側(z<0)に
        // 当たるので、**外向き法線がそのままレイと逆向き**になる(符号反転は不要)。
        let exact = smooth_hit.point.normalize_or_zero();
        assert!(
            exact.dot(ray.direction) < 0.0,
            "期待法線がレイと逆向きであること"
        );
        let flat_error = (flat_hit.normal - exact).length();
        let smooth_error = (smooth_hit.normal - exact).length();
        assert!(
            smooth_error < flat_error,
            "スムーズ法線のほうが解析法線に近いはず: smooth={smooth_error} flat={flat_error}"
        );
    }

    /// グリッドメッシュが指定の平行四辺形をちょうど覆うこと(面積が一致)。
    #[test]
    fn grid_mesh_tiles_the_requested_parallelogram_exactly() {
        let origin = Vec3::new(-1.0, 0.0, -1.0);
        let edge_u = Vec3::new(2.0, 0.0, 0.0);
        let edge_v = Vec3::new(0.0, 0.0, 3.0);
        let mesh = TriangleMesh::grid(origin, edge_u, edge_v, 4, 5);
        assert_eq!(mesh.triangle_count(), 4 * 5 * 2);
        let exact = edge_u.cross(edge_v).length();
        assert!(
            (mesh.surface_area() - exact).abs() < 1e-12,
            "三角形の総面積は平行四辺形の面積と厳密に一致するはず: {} vs {exact}",
            mesh.surface_area()
        );
    }

    /// **群6: 三角形メッシュがレンダリングパイプライン全体を通ること**。同じ位置・
    /// 同じ材質の「解析形状の球」と「細分したアイコスフィア」を実際に画像として
    /// レンダリングし、画素がほぼ一致することを確認する。`Primitive::Triangle`が
    /// BVH構築(`bounds`/`centroid`)・`Scene::closest_hit`・`trace`の全段を
    /// 正しく通っていなければ破れる(単体の交差判定テストだけでは通らない配線)。
    #[test]
    fn a_triangle_mesh_renders_the_same_image_as_the_analytic_sphere() {
        use crate::bsdf::Lambertian;
        use crate::camera::Camera;
        use crate::path_tracer::{Material, Scene, SceneObject};
        use crate::primitive::Primitive;
        use crate::render::{render_channel, RenderSettings};
        use crate::sphere::Sphere;

        let center = Vec3::new(0.0, 0.0, 4.0);
        let radius = 1.0;
        let material = Material::Lambertian(Lambertian { albedo: 0.7 });

        let analytic = Scene::new(
            vec![SceneObject::sphere(Sphere { center, radius }, material)],
            Vec::new(),
            1.0,
            None,
        );
        // 4回細分(5120三角形)なら、この解像度では解析球と区別が付かないはず。
        let mesh = TriangleMesh::icosphere(center, radius, 4).with_smooth_normals();
        let meshed = Scene::new(
            mesh.triangles()
                .into_iter()
                .map(|t| SceneObject {
                    primitive: Primitive::Triangle(t),
                    material,
                })
                .collect(),
            Vec::new(),
            1.0,
            None,
        );

        let camera = Camera {
            origin: Vec3::ZERO,
            forward: Vec3::new(0.0, 0.0, 1.0),
            right: Vec3::new(1.0, 0.0, 0.0),
            up: Vec3::new(0.0, 1.0, 0.0),
            lens_radius: 0.0,
            focus_distance: 4.0,
        };
        let settings = RenderSettings {
            spp: 8,
            max_depth: 3,
            exposure: 1.0,
            russian_roulette_after: None,
        };
        let (w, h) = (24u32, 24u32);
        let render = |scene: &Scene, stream: u64| {
            let mut rng = SimRng::new(17, stream);
            render_channel(scene, &camera, 0.9, w, h, &settings, &mut rng)
        };
        let a = render(&analytic, 0);
        let b = render(&meshed, 0);

        assert!(a.iter().any(|&v| v > 0.0), "何か写っているはず");
        // アルベド0.7の球 + 環境1.0 なので、画素は 0.7〜1.0 の範囲。形状が一致して
        // いれば、シルエットの縁を除いて画素差は小さい。
        let mean_abs_diff: f64 = a
            .iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).abs())
            .sum::<f64>()
            / a.len() as f64;
        assert!(
            mean_abs_diff < 0.02,
            "メッシュ球と解析球の画像は一致するはず: mean_abs_diff={mean_abs_diff}"
        );
        // 対照: 半径を変えたら**明確に**ずれる(上の一致が偶然でないことの確認)。
        let wrong = TriangleMesh::icosphere(center, radius * 0.7, 4);
        let wrong_scene = Scene::new(
            wrong
                .triangles()
                .into_iter()
                .map(|t| SceneObject {
                    primitive: Primitive::Triangle(t),
                    material,
                })
                .collect(),
            Vec::new(),
            1.0,
            None,
        );
        let c = render(&wrong_scene, 0);
        let wrong_diff: f64 = a
            .iter()
            .zip(c.iter())
            .map(|(x, y)| (x - y).abs())
            .sum::<f64>()
            / a.len() as f64;
        assert!(
            wrong_diff > 5.0 * mean_abs_diff,
            "半径違いは明確にずれるはず: wrong_diff={wrong_diff} match_diff={mean_abs_diff}"
        );
    }
}
