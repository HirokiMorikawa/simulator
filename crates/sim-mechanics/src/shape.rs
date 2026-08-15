//! 形状・AABB・接触マニフォールドの型。設計: docs/10-mechanics/02-collision-detection.md §3。

use sim_math::{Mat3, Vec3};

/// 剛体の幾何形状。Phase 1 は Sphere/Box/Plane のみ narrowphase 実装対象
/// (docs/10-mechanics/02-collision-detection.md §4.2)。Capsule/Compound/ConvexMesh は
/// 型として先に定義し、中身は担当フェーズ(P2/P5)で実装する。
#[derive(Clone, Debug)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    Box {
        half_extents: Vec3,
    },
    /// Phase 2。
    Capsule {
        radius: f64,
        half_height: f64,
    },
    /// static 専用・無限平面。
    Plane {
        normal: Vec3,
        d: f64,
    },
    /// Phase 2。
    Compound {
        children: Vec<(sim_math::Transform, Shape)>,
    },
    /// Phase 5(GJK/EPA)。
    ConvexMesh {
        vertices: Vec<Vec3>,
    },
}

impl Shape {
    /// 体積(質量 = 密度 × 体積の算出に使う)。Plane/Compound/ConvexMesh は
    /// static 専用または未実装フェーズのため `None`。
    pub fn volume(&self) -> Option<f64> {
        match self {
            Shape::Sphere { radius } => Some(4.0 / 3.0 * std::f64::consts::PI * radius.powi(3)),
            Shape::Box { half_extents } => {
                Some(8.0 * half_extents.x * half_extents.y * half_extents.z)
            }
            Shape::Plane { .. } => None,
            // **増分Lで実装**。円柱(半径r・高さ2h)+ 両端の半球(合わせて球1個)。
            Shape::Capsule {
                radius,
                half_height,
            } => Some(
                std::f64::consts::PI * radius * radius * 2.0 * half_height
                    + 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3),
            ),
            // **群10で実装**。各部品(ローカル座標に固定)の体積の単純和——
            // 部品どうしが重なっていても二重計上する(CSGのブーリアン和は
            // 行わない、標準的な複合剛体近似)。
            Shape::Compound { children } => {
                Some(children.iter().filter_map(|(_, s)| s.volume()).sum())
            }
            // **群10で実装**。真の凸包体積には面情報(三角形分割)が要るが
            // `ConvexMesh`は頂点列のみ持つ——`外部クレート実質ゼロ`の方針で
            // 3D凸包をゼロから実装するのは本増分の範囲外(既知の限界)。
            // 軸並行境界箱(AABB)の体積で近似する。凸包はAABBに内接するため
            // **常に過大評価**になる(密度から質量を出す用途では安全側ではない
            // ことに注意——形状を直接指定して`mass_override`を使うか、
            // 密度を実効値へ調整することを推奨、モジュールdoc参照)。
            Shape::ConvexMesh { vertices } => {
                let aabb = points_aabb(vertices)?;
                let size = aabb.max - aabb.min;
                Some(size.x * size.y * size.z)
            }
        }
    }

    /// 単位質量あたりのローカル慣性テンソル(対角、主軸がローカル軸に一致する形状のみ)。
    /// 設計: docs/10-mechanics/01-rigid-body.md §4.1。
    pub fn unit_mass_inertia_diagonal(&self) -> Vec3 {
        match self {
            Shape::Sphere { radius } => {
                let i = 2.0 / 5.0 * radius * radius;
                Vec3::new(i, i, i)
            }
            Shape::Box { half_extents } => {
                let (a, b, c) = (half_extents.x, half_extents.y, half_extents.z);
                Vec3::new(
                    (b * b + c * c) / 3.0,
                    (a * a + c * c) / 3.0,
                    (a * a + b * b) / 3.0,
                )
            }
            Shape::Plane { .. } => Vec3::ZERO,
            // **増分Lで実装**。ローカル+y軸を長軸とするカプセル。円柱部と
            // 半球部を別々に積分し、質量比で重み付けして合成する(標準的な
            // 複合剛体の慣性計算——半球は自身の重心まわりの慣性に平行軸定理で
            // 中心軸からの距離ぶんを足す)。
            Shape::Capsule {
                radius,
                half_height,
            } => {
                let (r, h) = (*radius, *half_height);
                let cylinder_volume = std::f64::consts::PI * r * r * 2.0 * h;
                let sphere_volume = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
                let total = cylinder_volume + sphere_volume;
                if total <= 0.0 {
                    return Vec3::ZERO;
                }
                // 単位質量あたりなので、体積比がそのまま質量比になる(密度一様)。
                let (mc, ms) = (cylinder_volume / total, sphere_volume / total);

                // 円柱: 長軸(y)まわり r²/2、横軸まわり (3r²+(2h)²)/12。
                let cyl_axial = 0.5 * r * r;
                let cyl_lateral = (3.0 * r * r + 4.0 * h * h) / 12.0;

                // 両端の半球を合わせた球: 長軸まわり 2r²/5。
                // 横軸まわりは半球自身の重心まわり(83/320)r² に、
                // 平行軸定理で中心からの距離 (h + 3r/8) を加える。
                let sph_axial = 2.0 / 5.0 * r * r;
                let hemisphere_offset = h + 3.0 * r / 8.0;
                let sph_lateral = 83.0 / 320.0 * r * r + hemisphere_offset * hemisphere_offset;

                let axial = mc * cyl_axial + ms * sph_axial;
                let lateral = mc * cyl_lateral + ms * sph_lateral;
                Vec3::new(lateral, axial, lateral)
            }
            // **群10で実装**。部品ごとの平行軸定理の和(設計方針: 統合エディタ
            // 実装計画の縦串①、docs/reviews/2026-08-14-editor-implementation-plan.md)。
            //
            // **簡略化(既知の限界)**: 複合剛体の重心は一般に部品配置により
            // ローカル原点からずれるが、`RigidBodySet`は単一点(`position[i]`)を
            // 並進の基準・回転の中心・(暗黙に)重心の3役で扱う設計であり、
            // 重心オフセットを持つボディは他の全形状も含めて元々サポートして
            // いない。したがってここでも**ローカル原点=重心**という既存の
            // 前提をそのまま踏襲し(シーン作成者が部品を原点まわりに対称配置する
            // ことを期待する)、平行軸定理は各部品の`transform.position`を
            // オフセットとして直接使う。
            //
            // 部品が親に対して回転(`transform.rotation`)している場合、その
            // 部品の対角テンソルは親フレームで一般に非対角になる
            // (`Mat3::similarity`で厳密に回転させた後、対角成分のみを採用する
            // 近似——`unit_mass_inertia_diagonal`の戻り値が対角`Vec3`固定の
            // ため、非対角成分は捨てる。部品が無回転なら誤差はゼロ)。
            Shape::Compound { children } => {
                let total_volume: f64 = children.iter().filter_map(|(_, s)| s.volume()).sum();
                if total_volume <= 0.0 {
                    return Vec3::ZERO;
                }
                let mut sum = Vec3::ZERO;
                for (xf, s) in children {
                    let Some(v) = s.volume() else { continue };
                    let mass_fraction = v / total_volume;
                    let local_diag = s.unit_mass_inertia_diagonal();
                    let rotated = Mat3::from_diagonal(local_diag).similarity(xf.rotation.to_mat3());
                    let rotated_diag = Vec3::new(rotated.m[0][0], rotated.m[1][1], rotated.m[2][2]);
                    let d = xf.position;
                    let parallel_axis = Vec3::new(
                        d.y * d.y + d.z * d.z,
                        d.x * d.x + d.z * d.z,
                        d.x * d.x + d.y * d.y,
                    );
                    sum = sum + (rotated_diag + parallel_axis).scale(mass_fraction);
                }
                sum
            }
            // **群10で実装**。`volume()`と同じ理由(面情報が無い)でAABBによる
            // 直方体近似(既知の限界、`volume()`のdoc参照)。
            Shape::ConvexMesh { vertices } => match points_aabb(vertices) {
                Some(aabb) => {
                    let size = aabb.max - aabb.min;
                    let half_extents = size.scale(0.5);
                    Shape::Box { half_extents }.unit_mass_inertia_diagonal()
                }
                None => Vec3::ZERO,
            },
        }
    }
}

/// 点群の軸並行境界箱(空なら`None`)。`Shape::ConvexMesh`のAABB近似
/// (`volume`/`unit_mass_inertia_diagonal`/`collision::aabb_of`が共有する)。
pub(crate) fn points_aabb(points: &[Vec3]) -> Option<Aabb> {
    let first = *points.first()?;
    let mut min = first;
    let mut max = first;
    for p in &points[1..] {
        min = Vec3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Vec3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }
    Some(Aabb { min, max })
}

/// 軸並行境界箱。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_volume_matches_formula() {
        let s = Shape::Sphere { radius: 2.0 };
        let expected = 4.0 / 3.0 * std::f64::consts::PI * 8.0;
        assert!((s.volume().unwrap() - expected).abs() < 1e-12);
    }

    #[test]
    fn box_volume_is_product_of_full_extents() {
        let b = Shape::Box {
            half_extents: Vec3::new(0.5, 1.0, 1.5),
        };
        assert!((b.volume().unwrap() - (1.0 * 2.0 * 3.0)).abs() < 1e-12);
    }

    #[test]
    fn sphere_inertia_diagonal_is_isotropic() {
        let s = Shape::Sphere { radius: 3.0 };
        let i = s.unit_mass_inertia_diagonal();
        let expected = 2.0 / 5.0 * 9.0;
        assert!((i.x - expected).abs() < 1e-12);
        assert_eq!(i.x, i.y);
        assert_eq!(i.y, i.z);
    }

    #[test]
    fn cube_inertia_diagonal_is_isotropic() {
        let cube = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let i = cube.unit_mass_inertia_diagonal();
        // 立方体は主慣性モーメントが等方(m/3*(1+1)=2m/3、単位質量なので 2/3)。
        assert!((i.x - 2.0 / 3.0).abs() < 1e-12);
        assert_eq!(i.x, i.y);
        assert_eq!(i.y, i.z);
    }

    fn identity_transform(position: Vec3) -> sim_math::Transform {
        sim_math::Transform {
            position,
            rotation: sim_math::Quat::IDENTITY,
        }
    }

    #[test]
    fn compound_volume_is_the_sum_of_its_children() {
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::ZERO),
                    Shape::Box {
                        half_extents: Vec3::new(0.5, 0.5, 0.5),
                    },
                ),
                (
                    identity_transform(Vec3::new(2.0, 0.0, 0.0)),
                    Shape::Sphere { radius: 1.0 },
                ),
            ],
        };
        let expected = 1.0 + 4.0 / 3.0 * std::f64::consts::PI;
        assert!((compound.volume().unwrap() - expected).abs() < 1e-12);
    }

    /// 「ダンベル」(x軸上 ±d に置いた微小な箱2つ)の慣性が平行軸定理の式
    /// $I_y = I_z = m_{total}\cdot d^2$(各部品の自重慣性は箱を微小にして無視できる
    /// ようにする)・$I_x \approx 0$ に一致すること。
    #[test]
    fn compound_inertia_matches_parallel_axis_theorem_for_a_dumbbell() {
        let d = 3.0;
        let tiny = Vec3::new(1e-6, 1e-6, 1e-6);
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::new(d, 0.0, 0.0)),
                    Shape::Box { half_extents: tiny },
                ),
                (
                    identity_transform(Vec3::new(-d, 0.0, 0.0)),
                    Shape::Box { half_extents: tiny },
                ),
            ],
        };
        let i = compound.unit_mass_inertia_diagonal();
        assert!(i.x.abs() < 1e-9, "x軸まわりはオフセットに寄与しない: {i:?}");
        assert!(
            (i.y - d * d).abs() < 1e-9,
            "y軸まわりは m_total(=1)*d^2 のはず: {i:?}"
        );
        assert!(
            (i.z - d * d).abs() < 1e-9,
            "z軸まわりは m_total(=1)*d^2 のはず: {i:?}"
        );
    }

    /// `ConvexMesh`はAABB近似(モジュールdoc参照)なので、頂点が立方体の
    /// 8隅そのものなら`Shape::Box`と完全に一致するはず。
    #[test]
    fn convex_mesh_of_a_cubes_corners_matches_the_equivalent_box() {
        let half = 1.5;
        let mut vertices = Vec::new();
        for &sx in &[-1.0, 1.0] {
            for &sy in &[-1.0, 1.0] {
                for &sz in &[-1.0, 1.0] {
                    vertices.push(Vec3::new(sx * half, sy * half, sz * half));
                }
            }
        }
        let mesh = Shape::ConvexMesh { vertices };
        let equivalent_box = Shape::Box {
            half_extents: Vec3::new(half, half, half),
        };
        assert!((mesh.volume().unwrap() - equivalent_box.volume().unwrap()).abs() < 1e-9);
        let mesh_i = mesh.unit_mass_inertia_diagonal();
        let box_i = equivalent_box.unit_mass_inertia_diagonal();
        assert!((mesh_i.x - box_i.x).abs() < 1e-9);
        assert!((mesh_i.y - box_i.y).abs() < 1e-9);
        assert!((mesh_i.z - box_i.z).abs() < 1e-9);
    }

    #[test]
    fn empty_convex_mesh_has_no_volume_and_no_inertia() {
        let mesh = Shape::ConvexMesh { vertices: vec![] };
        assert_eq!(mesh.volume(), None);
        assert_eq!(mesh.unit_mass_inertia_diagonal(), Vec3::ZERO);
    }

    /// **解析解の足場**(慣性テンソルを対角`Vec3`から完全な`Mat3`へ拡張する
    /// 予定変更に備えた基準点)。辺長 $w\times h\times d$ の一様直方体の主慣性
    /// モーメントは閉形式で
    /// $I=\frac{m}{12}\mathrm{diag}(h^2+d^2,\;w^2+d^2,\;w^2+h^2)$。
    /// `unit_mass_inertia_diagonal`は単位質量あたりを返すので $m=1$ とおいて比較する。
    ///
    /// 既存の`cube_inertia_diagonal_is_isotropic`は立方体(3軸が等価)しか見て
    /// おらず、軸の取り違え(x/y/zの入れ替え)を検出できない。ここでは
    /// **3辺すべてを異なる長さ**にして、どの軸にどの項が乗るかまで固定する。
    /// 許容誤差は閉形式どうしの比較(数値積分を挟まない)なので倍精度の
    /// 丸め誤差だけを見込んだ 1e-15(相対)。
    #[test]
    fn box_inertia_diagonal_matches_the_closed_form_formula() {
        let half_extents = Vec3::new(0.3, 0.7, 1.1);
        let (w, h, d) = (
            2.0 * half_extents.x,
            2.0 * half_extents.y,
            2.0 * half_extents.z,
        );
        let expected = Vec3::new(
            (h * h + d * d) / 12.0,
            (w * w + d * d) / 12.0,
            (w * w + h * h) / 12.0,
        );
        let actual = Shape::Box { half_extents }.unit_mass_inertia_diagonal();
        for (a, e) in [
            (actual.x, expected.x),
            (actual.y, expected.y),
            (actual.z, expected.z),
        ] {
            assert!(
                (a - e).abs() / e < 1e-15,
                "actual={actual:?} expected={expected:?}"
            );
        }
        // 3辺が異なるので主慣性モーメントも3つとも異なる(軸の取り違え検出用)。
        // 最も短い辺(x)まわりの慣性が最大、最も長い辺(z)まわりが最小。
        assert!(actual.x > actual.y && actual.y > actual.z, "{actual:?}");
    }

    /// **AABB近似の「カナリア」**(正しさの証明ではない、現状の**誤り方**を固定する
    /// テスト)。`ConvexMesh`は面情報を持たず、体積・慣性を頂点群のAABBで代用する
    /// (`volume()`のdoc「既知の限界」参照)。既存の
    /// `convex_mesh_of_a_cubes_corners_matches_the_equivalent_box`は、AABBが
    /// たまたま厳密になる立方体の8隅しか見ていない。
    ///
    /// ここでは**AABB近似が確実に外れる**正多面体を2つ使い、真の解析的体積との
    /// 比を固定する:
    /// - 正四面体 $(\pm1,\pm1,\pm1)$ の交互4頂点。辺長 $a=2\sqrt2$、
    ///   $V=a^3/(6\sqrt2)=8/3$。AABB は一辺2の立方体なので $V_{AABB}=8$ ——
    ///   **ちょうど3倍**の過大評価。
    /// - 正八面体 $(\pm1,0,0),(0,\pm1,0),(0,0,\pm1)$。$V=4/3$ に対し
    ///   $V_{AABB}=8$ ——**ちょうど6倍**。
    ///
    /// 慣性も同様に過大評価になる(どちらの多面体も対称性から慣性テンソルは
    /// 等方で、正四面体は $I/m=a^2/20=0.4$、正八面体は $I/m=a^2/10=0.2$。
    /// AABB近似はどちらも一辺2の立方体の $2/3$)。
    ///
    /// **将来の3D凸包(quickhull)実装が入ったらこのテストは落ちる**——その時は
    /// 比が 1.0 になったことを確認する形へ意図的に書き換える(落ちること自体が
    /// 「近似が実装に置き換わった」という通知になる)。倍数はすべて有理数で
    /// 厳密に表せるので許容誤差は丸め誤差ぶんの 1e-12(相対)。
    #[test]
    fn convex_mesh_aabb_approximation_overestimates_regular_polyhedra() {
        // 正四面体: 立方体の隅を1つおきに取ると正四面体になる。
        let edge = 2.0 * std::f64::consts::SQRT_2;
        let tetrahedron = Shape::ConvexMesh {
            vertices: vec![
                Vec3::new(1.0, 1.0, 1.0),
                Vec3::new(1.0, -1.0, -1.0),
                Vec3::new(-1.0, 1.0, -1.0),
                Vec3::new(-1.0, -1.0, 1.0),
            ],
        };
        let tetra_true_volume = edge.powi(3) / (6.0 * std::f64::consts::SQRT_2);
        assert!(
            (tetra_true_volume - 8.0 / 3.0).abs() < 1e-12,
            "解析式 a^3/(6√2) は 8/3 になるはず: {tetra_true_volume}"
        );
        let tetra_approx = tetrahedron.volume().unwrap();
        assert!(
            (tetra_approx / tetra_true_volume - 3.0).abs() < 1e-12,
            "現状のAABB近似は正四面体の体積をちょうど3倍に過大評価する: \
             approx={tetra_approx} true={tetra_true_volume}"
        );

        // 正八面体。
        let octahedron = Shape::ConvexMesh {
            vertices: vec![
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 0.0, -1.0),
            ],
        };
        let octa_true_volume = 4.0 / 3.0;
        let octa_approx = octahedron.volume().unwrap();
        assert!(
            (octa_approx / octa_true_volume - 6.0).abs() < 1e-12,
            "現状のAABB近似は正八面体の体積をちょうど6倍に過大評価する: \
             approx={octa_approx} true={octa_true_volume}"
        );

        // 慣性も過大評価(どちらも真の値は等方、AABB近似は一辺2の立方体の 2/3)。
        let cube_unit_inertia = 2.0 / 3.0;
        let tetra_true_inertia = edge * edge / 20.0; // = 0.4
        let octa_true_inertia = 2.0 / 10.0; // 辺長 a=√2 の正八面体、a²/10
        for (shape, true_inertia, label) in [
            (&tetrahedron, tetra_true_inertia, "正四面体"),
            (&octahedron, octa_true_inertia, "正八面体"),
        ] {
            let i = shape.unit_mass_inertia_diagonal();
            assert!(
                (i.x - cube_unit_inertia).abs() < 1e-12
                    && (i.y - cube_unit_inertia).abs() < 1e-12
                    && (i.z - cube_unit_inertia).abs() < 1e-12,
                "{label}: 現状は外接立方体の慣性そのもの: {i:?}"
            );
            assert!(
                i.x > true_inertia,
                "{label}: AABB近似は真の慣性 {true_inertia} を過大評価する: {i:?}"
            );
        }
    }
}
