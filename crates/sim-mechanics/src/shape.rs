//! 形状・AABB・接触マニフォールドの型。設計: docs/10-mechanics/02-collision-detection.md §3。
//!
//! ## 質量特性(体積・重心・慣性テンソル)
//!
//! 3つの量は**同じ積分から出る**ので、`MassProperties`(体積・ローカル重心・
//! **重心まわり**の単位質量慣性テンソル)として一度に計算し、`volume()` /
//! `center_of_mass()` / `unit_mass_inertia_tensor()` はその射影として提供する。
//! 別々に実装すると「重心はずれているのに慣性は原点まわり」のような
//! 内部矛盾が入り込むため(実際に移行前の`Compound`はその状態だった)。
//!
//! **重心まわり**であることが規約の要。`RigidBodySet`は`position[i]`を重心として
//! 追跡し(`RigidBodySet::center_of_mass`のdoc参照)、`inv_inertia_local`にはここで
//! 返す重心まわりのテンソルをそのまま入れる。ローカル原点まわりのテンソルが
//! 必要な場面は無い(平行軸定理の移動は`Compound`の合成の中で閉じている)。

use sim_math::{Mat3, Vec3};

/// 形状の質量特性。密度一様を仮定する(この物理コアの全体前提)。
///
/// - `volume`: 体積。
/// - `center_of_mass`: **その形状のローカル系**での重心。
/// - `unit_inertia`: **重心まわり**・**単位質量あたり**の慣性テンソル。
///   実際の慣性は質量 $m$ を掛けて $m\,I_{unit}$。
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct MassProperties {
    pub volume: f64,
    pub center_of_mass: Vec3,
    pub unit_inertia: Mat3,
}

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
    /// 質量特性(体積・ローカル重心・重心まわりの単位質量慣性テンソル)を
    /// まとめて求める。モジュールdoc「質量特性」参照。
    ///
    /// `Plane`(無限平面・static専用)と空の`ConvexMesh`は体積を持たないので
    /// `None`。`Compound`は体積を持つ子だけを集める(平面を子に混ぜても
    /// 質量には寄与しない)。
    pub(crate) fn mass_properties(&self) -> Option<MassProperties> {
        match self {
            Shape::Sphere { radius } => {
                let i = 2.0 / 5.0 * radius * radius;
                Some(MassProperties {
                    volume: 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3),
                    center_of_mass: Vec3::ZERO,
                    unit_inertia: Mat3::from_diagonal(Vec3::new(i, i, i)),
                })
            }
            Shape::Box { half_extents } => {
                let (a, b, c) = (half_extents.x, half_extents.y, half_extents.z);
                Some(MassProperties {
                    volume: 8.0 * a * b * c,
                    center_of_mass: Vec3::ZERO,
                    unit_inertia: Mat3::from_diagonal(Vec3::new(
                        (b * b + c * c) / 3.0,
                        (a * a + c * c) / 3.0,
                        (a * a + b * b) / 3.0,
                    )),
                })
            }
            Shape::Plane { .. } => None,
            // **増分Lで実装**。円柱(半径r・高さ2h)+ 両端の半球(合わせて球1個)。
            // ローカル+y軸を長軸とし、原点まわりに対称なので重心はローカル原点。
            Shape::Capsule {
                radius,
                half_height,
            } => {
                let (r, h) = (*radius, *half_height);
                let cylinder_volume = std::f64::consts::PI * r * r * 2.0 * h;
                let sphere_volume = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
                let total = cylinder_volume + sphere_volume;
                if total <= 0.0 {
                    return None;
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
                Some(MassProperties {
                    volume: total,
                    center_of_mass: Vec3::ZERO,
                    unit_inertia: Mat3::from_diagonal(Vec3::new(lateral, axial, lateral)),
                })
            }
            // **群10で実装 / 群11で重心オフセットと完全テンソルへ拡張**。
            //
            // 移行前の既知の限界は2つあり、どちらもここで解消した:
            // ①「ローカル原点=重心」を仮定して平行軸定理のオフセットに
            //   `transform.position`を直接使っていた(部品配置が非対称だと誤り)。
            //   → 体積重み付き平均で**真の重心**を出し、そこからのオフセットで移動する。
            // ②戻り値が対角`Vec3`固定だったため、部品が親に対して回転している
            //   場合の非対角成分(慣性乗積)を捨てていた。
            //   → 完全な`Mat3`を返すようになったので厳密。
            //
            // **残る近似(正直な開示)**: 部品どうしが**重なっている**場合、
            // 重なり領域の質量を二重計上する(`union_volume`のdoc参照)。
            // 重なりの無い配置(既存シーンのほぼ全て)では厳密。
            Shape::Compound { children } => {
                let parts: Vec<(sim_math::Transform, MassProperties)> = children
                    .iter()
                    .filter_map(|(xf, s)| s.mass_properties().map(|p| (*xf, p)))
                    .collect();
                let total_volume: f64 = parts.iter().map(|(_, p)| p.volume).sum();
                if total_volume <= 0.0 {
                    return None;
                }

                // 重心: 各部品の重心を親フレームへ移し、体積で重み付けて平均。
                let mut weighted = Vec3::ZERO;
                for (xf, p) in &parts {
                    weighted = weighted + xf.apply_point(p.center_of_mass).scale(p.volume);
                }
                let center_of_mass = weighted.scale(1.0 / total_volume);

                // 慣性: 各部品の「自身の重心まわりのテンソル」を親フレームへ
                // 相似変換で回し、平行軸定理で**複合剛体の重心**まで移す。
                let mut unit_inertia = Mat3::from_diagonal(Vec3::ZERO);
                for (xf, p) in &parts {
                    let rotated = p.unit_inertia.similarity(xf.rotation.to_mat3());
                    let d = xf.apply_point(p.center_of_mass) - center_of_mass;
                    let about_com = rotated + Mat3::parallel_axis_term(d);
                    unit_inertia = unit_inertia + about_com.scale(p.volume / total_volume);
                }

                Some(MassProperties {
                    volume: union_volume(children, total_volume),
                    center_of_mass,
                    unit_inertia,
                })
            }
            Shape::ConvexMesh { vertices } => convex_mesh_mass_properties(vertices),
        }
    }

    /// 体積(質量 = 密度 × 体積の算出に使う)。`Plane`は無限平面(static専用)、
    /// 空の`ConvexMesh`は実体が無いため `None`。
    ///
    /// `Compound`の合成規則は`union_volume`のdoc参照。
    pub fn volume(&self) -> Option<f64> {
        self.mass_properties().map(|p| p.volume)
    }

    /// **その形状のローカル系での重心**(群11で追加)。
    ///
    /// ローカル原点まわりに対称な形状(`Sphere`/`Box`/`Capsule`/`Plane`)は
    /// 厳密に`Vec3::ZERO`——つまり**既存の全シーンで挙動は一切変わらない**。
    /// `Compound`だけが部品配置に応じた真の重心を返し、`ConvexMesh`は
    /// 凸包の重心を返す。
    ///
    /// `RigidBodySet`はこの値を`center_of_mass[i]`に保持し、`position[i]`
    /// (=重心)と形状のローカル原点を相互変換する(`RigidBodySet::shape_transform`)。
    pub fn center_of_mass(&self) -> Vec3 {
        self.mass_properties()
            .map(|p| p.center_of_mass)
            .unwrap_or(Vec3::ZERO)
    }

    /// **重心まわり**の単位質量あたり慣性テンソル(群11で対角`Vec3`から拡張)。
    /// 設計: docs/10-mechanics/01-rigid-body.md §4.1。
    ///
    /// 実際の慣性テンソルは質量を掛けて $I = m\,I_{unit}$。主軸がローカル軸に
    /// 一致する単純形状では対角行列になり、`Compound`で部品が回転・オフセット
    /// 配置されている場合にのみ非対角成分(慣性乗積)が現れる。
    pub fn unit_mass_inertia_tensor(&self) -> Mat3 {
        self.mass_properties()
            .map(|p| p.unit_inertia)
            .unwrap_or(Mat3::from_diagonal(Vec3::ZERO))
    }

    /// `unit_mass_inertia_tensor`の対角成分のみ。
    ///
    /// **主軸がローカル軸に一致する形状(`Sphere`/`Box`/`Capsule`、および部品が
    /// 無回転で対称配置された`Compound`)でのみ完全な情報**であり、慣性乗積を
    /// 持つ形状では情報が落ちる。物理の積分には使わない(`RigidBodySet`は
    /// 完全な`Mat3`を保持する)——主慣性モーメントを直接比較したいテストや
    /// 診断表示のための便宜関数。
    pub fn unit_mass_inertia_diagonal(&self) -> Vec3 {
        let t = self.unit_mass_inertia_tensor();
        Vec3::new(t.m[0][0], t.m[1][1], t.m[2][2])
    }
}

/// `Compound`の体積。`naive_sum`は各部品の体積の単純和。
///
/// **既知の限界(群11時点では未解消、次の増分で対応)**: 部品どうしが重なって
/// いても単純和のまま返すため、重なり領域を二重計上する。CSGのブーリアン和は
/// まだ実装していない。
fn union_volume(_children: &[(sim_math::Transform, Shape)], naive_sum: f64) -> f64 {
    naive_sum
}

/// `ConvexMesh`の質量特性。
///
/// **既知の限界(群11時点では未解消、次の増分で対応)**: 真の凸包体積・慣性には
/// 面情報(三角形分割)が要るが`ConvexMesh`は頂点列のみ持つ。ここでは軸並行
/// 境界箱(AABB)による直方体近似で代用する。凸包はAABBに内接するため
/// **常に過大評価**になる(正四面体で体積3倍、正八面体で6倍——
/// `shape::tests`のカナリアテスト参照)。密度から質量を出す用途では安全側では
/// ないことに注意——形状を直接指定して`mass_override`を使うか、密度を実効値へ
/// 調整することを推奨。
///
/// 移行前と違い**重心はAABBの中心**を返す(頂点群が原点まわりに非対称なら
/// ローカル原点とはずれる)。これは`Box`近似の慣性が「AABBの中心まわり」で
/// あることと整合させるため——移行前は慣性だけAABB中心まわりで計算しつつ
/// 重心を原点と見なしており、内部矛盾していた。
fn convex_mesh_mass_properties(vertices: &[Vec3]) -> Option<MassProperties> {
    let aabb = points_aabb(vertices)?;
    let size = aabb.max - aabb.min;
    let half_extents = size.scale(0.5);
    let approximation = Shape::Box { half_extents };
    Some(MassProperties {
        volume: size.x * size.y * size.z,
        center_of_mass: (aabb.min + aabb.max).scale(0.5),
        unit_inertia: approximation.unit_mass_inertia_tensor(),
    })
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

    /// ローカル原点まわりに対称な形状の重心は**厳密に**ローカル原点。
    /// これが崩れると「既存シーンの挙動は完全に不変」という群11の前提が壊れる。
    #[test]
    fn symmetric_primitive_shapes_have_their_center_of_mass_at_the_local_origin() {
        for shape in [
            Shape::Sphere { radius: 1.7 },
            Shape::Box {
                half_extents: Vec3::new(0.3, 0.7, 1.1),
            },
            Shape::Capsule {
                radius: 0.4,
                half_height: 1.2,
            },
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
        ] {
            assert_eq!(
                shape.center_of_mass(),
                Vec3::ZERO,
                "対称形状の重心はローカル原点でなければならない: {shape:?}"
            );
        }
    }

    /// **複合剛体の重心は体積(=質量)加重平均**(群11で追加)。
    /// 体積 $V_1$ の部品を原点に、体積 $V_2$ の部品を $x=d$ に置けば
    /// 重心は $x = V_2 d/(V_1+V_2)$。移行前は「ローカル原点=重心」と
    /// 決め打ちしていたので、この値は常に0だった。
    #[test]
    fn compound_center_of_mass_is_the_volume_weighted_average() {
        let d = 2.0;
        let big = Shape::Box {
            half_extents: Vec3::new(0.5, 0.5, 0.5),
        }; // 体積 1
        let small = Shape::Box {
            half_extents: Vec3::new(0.25, 0.25, 0.25),
        }; // 体積 1/8
        let (v1, v2) = (big.volume().unwrap(), small.volume().unwrap());
        assert!((v1 - 1.0).abs() < 1e-15 && (v2 - 0.125).abs() < 1e-15);

        let compound = Shape::Compound {
            children: vec![
                (identity_transform(Vec3::ZERO), big),
                (identity_transform(Vec3::new(d, 0.0, 0.0)), small),
            ],
        };
        let com = compound.center_of_mass();
        let expected_x = v2 * d / (v1 + v2);
        assert!(
            (com.x - expected_x).abs() < 1e-15,
            "重心x = V2·d/(V1+V2) = {expected_x} のはず: {com:?}"
        );
        assert!(com.y.abs() < 1e-15 && com.z.abs() < 1e-15, "{com:?}");
    }

    /// **非対称ダンベルの慣性は「換算質量」の式に一致する**(群11)。
    ///
    /// 質量比 $f_1,f_2$($f_1+f_2=1$)の2質点を距離 $d$ 離して置くと、
    /// **重心まわり**の慣性は $I = f_1 f_2 d^2$(換算質量 × 距離²)。
    /// 移行前は慣性をローカル原点まわりに計算していたので、この配置では
    /// $f_1\cdot0^2 + f_2 d^2 = f_2 d^2$ という**別の値**になっていた
    /// ($f_1f_2d^2$ より大きい——平行軸定理のぶん過大)。
    #[test]
    fn asymmetric_dumbbell_inertia_matches_the_reduced_mass_formula() {
        let d = 3.0;
        // 微小な箱=質点。体積比 8:1 (半辺2倍で体積8倍)。
        let heavy = Shape::Box {
            half_extents: Vec3::new(2e-6, 2e-6, 2e-6),
        };
        let light = Shape::Box {
            half_extents: Vec3::new(1e-6, 1e-6, 1e-6),
        };
        let (v1, v2) = (heavy.volume().unwrap(), light.volume().unwrap());
        let (f1, f2) = (v1 / (v1 + v2), v2 / (v1 + v2));

        let compound = Shape::Compound {
            children: vec![
                (identity_transform(Vec3::ZERO), heavy),
                (identity_transform(Vec3::new(d, 0.0, 0.0)), light),
            ],
        };

        // 重心は重い側へ寄る。
        let com = compound.center_of_mass();
        assert!((com.x - f2 * d).abs() < 1e-12, "{com:?}");

        let i = compound.unit_mass_inertia_diagonal();
        let expected = f1 * f2 * d * d;
        assert!(i.x.abs() < 1e-9, "長軸まわりは実質ゼロ: {i:?}");
        assert!(
            (i.y - expected).abs() / expected < 1e-9,
            "y軸まわりは換算質量の式 f1·f2·d² = {expected} のはず: {i:?}"
        );
        assert!(
            (i.z - expected).abs() / expected < 1e-9,
            "z軸まわりも同じ: {i:?}"
        );
        // 移行前の値(原点まわり = f2·d²)とは**明確に違う**ことを固定する。
        let old_wrong = f2 * d * d;
        assert!(
            (i.y - old_wrong).abs() / old_wrong > 0.1,
            "重心まわりの値は原点まわりの値と有意に異なるはず: new={} old={old_wrong}",
            i.y
        );
    }

    /// **慣性乗積(非対角成分)が実際に出ること**(群11、完全`Mat3`化の要点)。
    ///
    /// 単位質量を2等分して $\pm(d,d,0)$ に置くと、重心は原点で、慣性テンソルは
    /// $I = |r|^2E - r\,r^\top$ を質量比で平均して
    /// $\begin{pmatrix} d^2 & -d^2 & 0\\ -d^2 & d^2 & 0\\ 0&0&2d^2\end{pmatrix}$。
    /// **$I_{xy}=-d^2$ という非対角成分がある**のがポイントで、対角`Vec3`しか
    /// 返せなかった移行前の実装はこれを捨てて $\mathrm{diag}(d^2,d^2,2d^2)$ を
    /// 返していた(= 主軸を取り違えた別の剛体を積分していた)。
    #[test]
    fn compound_inertia_tensor_has_products_of_inertia_for_diagonal_placement() {
        let d = 2.0;
        let tiny = Vec3::new(1e-6, 1e-6, 1e-6);
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::new(d, d, 0.0)),
                    Shape::Box { half_extents: tiny },
                ),
                (
                    identity_transform(Vec3::new(-d, -d, 0.0)),
                    Shape::Box { half_extents: tiny },
                ),
            ],
        };
        assert!(compound.center_of_mass().length() < 1e-12);

        let t = compound.unit_mass_inertia_tensor();
        let d2 = d * d;
        let expected = [[d2, -d2, 0.0], [-d2, d2, 0.0], [0.0, 0.0, 2.0 * d2]];
        for (i, row) in expected.iter().enumerate() {
            for (j, e) in row.iter().enumerate() {
                assert!(
                    (t.m[i][j] - e).abs() < 1e-9,
                    "I[{i}][{j}] = {} は {e} のはず(テンソル全体={t:?})",
                    t.m[i][j]
                );
            }
        }
        // 非対角成分が「実際にゼロでない」ことを明示(退化した検証にしない)。
        assert!(t.m[0][1].abs() > 1.0, "慣性乗積が出ていない: {t:?}");
    }

    /// 部品が親に対して**回転**していても、相似変換で厳密に扱えること(群11)。
    /// 立方体は等方(慣性テンソルがスカラー行列)なので、どんな回転をかけても
    /// テンソルは不変——$RIR^\top = I$ が厳密に成り立つ。移行前は回転後の
    /// 対角成分だけを拾う近似だったが、等方な場合は元々誤差ゼロなので、
    /// ここでは**非等方**な直方体を45°回した場合で差が出ることを見る。
    #[test]
    fn rotated_child_contributes_off_diagonal_inertia() {
        let half_extents = Vec3::new(1.0, 0.2, 0.2);
        let rotated = sim_math::Transform {
            position: Vec3::ZERO,
            rotation: sim_math::Quat::from_axis_angle(
                Vec3::new(0.0, 0.0, 1.0),
                std::f64::consts::FRAC_PI_4,
            ),
        };
        let compound = Shape::Compound {
            children: vec![(rotated, Shape::Box { half_extents })],
        };
        let t = compound.unit_mass_inertia_tensor();
        // 単一部品なので、部品自身のテンソルを45°回した相似変換そのもの。
        let expected = Shape::Box { half_extents }
            .unit_mass_inertia_tensor()
            .similarity(rotated.rotation.to_mat3());
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (t.m[i][j] - expected.m[i][j]).abs() < 1e-15,
                    "I[{i}][{j}]: {} vs {}",
                    t.m[i][j],
                    expected.m[i][j]
                );
            }
        }
        // z軸まわり45°回転なので xy 成分に慣性乗積が出る。
        assert!(
            t.m[0][1].abs() > 1e-3,
            "45°回した細長い箱は慣性乗積を持つはず: {t:?}"
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
