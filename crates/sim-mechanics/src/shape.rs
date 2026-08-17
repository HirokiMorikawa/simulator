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
    /// Phase 5(GJK/EPA)。**常にその頂点群の凸包として**扱われる
    /// (`crate::hull::convex_hull`)——非凸な三角形メッシュから正しい
    /// `Shape`を作りたい場合は面情報が要るので`Shape::from_triangle_mesh`
    /// (近似凸分解、`crate::decompose`)を使う。
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
            // **残る近似(正直な開示)**: **体積**は`union_volume`がブーリアン和を
            // 取るので重なっていても正しいが、**慣性テンソルと重心の質量配分**は
            // 各部品の素の体積比 `p.volume / total_volume` のままである。
            // つまり部品が重なっている領域は「密度2倍」として重心・慣性に効く。
            // 重なりの無い配置(既存シーンのほぼ全て)では厳密で、L字のように
            // 9%程度重なる構成でも重心・慣性への影響は数%に留まる。
            // 完全に正すには union 領域そのものを積分する必要があり、本増分の
            // 範囲外とした(質量=体積×密度の側だけでも正しくしたのは、
            // 質量が運動方程式へ一次で効くのに対し、質量配分の偏りは
            // 慣性テンソルへ二次的にしか効かないため)。
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

    /// 任意の(凹みを含みうる)三角形メッシュから`Shape`を作る(近似凸分解、
    /// `crate::decompose`参照、V-HACD相当)。
    ///
    /// `Shape::ConvexMesh{vertices}`は頂点だけを持ち**常にその凸包として**
    /// 扱われる——手で凸だと分かっている点群を渡す用途のための型で、
    /// 意味を変えるとその前提を壊す。非凸メッシュ(L字・U字・くびれのある
    /// ダンベル型など)を正しく扱うには面情報(`triangles`、どの3頂点が
    /// 実際の表面を成すか)が要るので、別のコンストラクタとして用意した。
    ///
    /// `vertices`はメッシュの全頂点、`triangles`は各三角形を頂点インデックス
    /// 3つ組(**外向き**の巻き順、`crate::hull`と同じ規約)で表す。
    ///
    /// 分解結果が1パーツなら(=元から実質凸)`ConvexMesh{vertices}`を
    /// **入力の頂点そのまま**で返す——`convex_hull`は同じ点群に対して
    /// 常に同じ結果になるので、これは既存の`ConvexMesh`と1ビットも
    /// 違わない(「凸メッシュには影響しない」という群の前提を保つ)。
    /// 複数パーツなら、各パーツを`ConvexMesh`として持つ`Compound`を返す
    /// ——質量特性(`union_volume`)も接触生成(collision.rsの`Compound`分解)も
    /// **既存の経路をそのまま使う**、この関数自身は分解の一度きりの実行だけを担う。
    pub fn from_triangle_mesh(vertices: Vec<Vec3>, triangles: Vec<[usize; 3]>) -> Shape {
        let parts = crate::decompose::decompose_mesh(&vertices, &triangles);
        match parts.len() {
            0 => Shape::ConvexMesh { vertices }, // 退化入力(体積なし)。元の点はそのまま残す。
            1 => Shape::ConvexMesh { vertices },
            _ => Shape::Compound {
                children: parts
                    .into_iter()
                    .map(|hull| {
                        let identity = sim_math::Transform {
                            position: Vec3::ZERO,
                            rotation: sim_math::Quat::IDENTITY,
                        };
                        (
                            identity,
                            Shape::ConvexMesh {
                                vertices: hull.vertices,
                            },
                        )
                    })
                    .collect(),
            },
        }
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

/// `Compound`の体積を**ブーリアン和(union)**として求める(群11)。
/// `naive_sum` は各部品の体積の単純和(重なりを二重計上した値)。
///
/// ## なぜ必要か
///
/// 移行前は `children.map(volume).sum()` をそのまま返していたため、部品が
/// 重なって配置されていると重なり領域を二重計上して質量を過大評価していた。
/// これは机上の心配ではない——`sim-wasm` の `spawn_compound_l_shape` が作る
/// L字は縦棒と横棒が実際に交差しており、単純和 0.75 m³ に対し真の和は
/// 0.6875 m³、**9%の過大評価**だった(`tests`で解析的に固定してある)。
///
/// ## 3段構えの実装(なぜ Monte Carlo 一本にしないか)
///
/// 1. **部品が互いに素なら単純和がそのまま厳密**。形状は自身のAABBに含まれる
///    ので、AABBが重ならない部品対は形状も重ならない。既存シーンのほとんど
///    (と既存テスト)はこの経路に落ち、**数値は1ビットも変わらない**。
///    Monte Carlo を無条件に使うと、重なりゼロの構成でも推定誤差が乗って
///    `compound_volume_is_the_sum_of_its_children` のような厳密比較のテストが
///    壊れてしまう——それは実装の後退である。
/// 2. **重なりがあり、かつ全部品が軸並行な箱なら座標圧縮で厳密**。
///    全部品の x/y/z 境界値で空間を格子に切ると、各セルは「どれかの箱に
///    完全に含まれる」か「どの箱とも交わらない」かのどちらかになる
///    (Klee の測度問題の標準解法)。含まれるセルの体積を足せば**厳密な**
///    union が出る。これが実使用の主経路(L字も車体もほぼ箱で組まれる)。
/// 3. **それ以外(球・カプセル・回転した箱が重なる場合)は層化 Monte Carlo**。
///    決定論的な固定列(下記)で union の AABB 内に点を撒き、「どれかの部品に
///    含まれる」割合から体積を推定する。
///
/// ### Monte Carlo の誤差(経路3を通る場合のみ)
///
/// サンプル数 $N=200{,}000$、包含率 $p$ の二項分布なので体積の相対標準誤差は
/// $\sqrt{(1-p)/(pN)}$。実用域の $p\gtrsim0.2$ なら **0.45% 以下**、
/// $p=0.5$ なら 0.22%。質量はこの精度で決まる(密度指定でボディを作る場合)。
/// 厳密さが要るなら `mass_override` で質量を直接指定できる。
///
/// 決定論のため乱数生成器は使わず、**加法的再帰列(Weyl列)**を3次元に
/// 拡張した低食い違い量列を使う。同じ形状には常に同じ推定値が返る
/// (この物理コアの決定論方針。`state_hash` の再現性を壊さない)。
fn union_volume(children: &[(sim_math::Transform, Shape)], naive_sum: f64) -> f64 {
    let leaves = flatten_leaves(
        children,
        sim_math::Transform {
            position: Vec3::ZERO,
            rotation: sim_math::Quat::IDENTITY,
        },
    );
    if leaves.len() < 2 {
        return naive_sum;
    }

    // 経路1: 部品が互いに素(AABBが1つも重ならない)なら単純和が厳密。
    let boxes: Vec<Aabb> = leaves.iter().map(|(xf, s)| leaf_aabb(s, *xf)).collect();
    let mut any_overlap = false;
    'outer: for i in 0..boxes.len() {
        for j in (i + 1)..boxes.len() {
            if aabbs_overlap(boxes[i], boxes[j]) {
                any_overlap = true;
                break 'outer;
            }
        }
    }
    if !any_overlap {
        return naive_sum;
    }

    // 経路2: 全部品が軸並行な箱 → 座標圧縮で厳密。
    if let Some(exact) = axis_aligned_box_union_volume(&leaves) {
        return exact;
    }

    // 経路3: 層化 Monte Carlo。
    monte_carlo_union_volume(&leaves, &boxes)
}

/// `Compound`を(入れ子も含めて)葉の形状へ平坦化し、それぞれの親ローカル系での
/// 変換を添えて返す。union の判定は「葉のどれかに含まれるか」で行うため、
/// 入れ子の`Compound`をそのまま部品として扱うと包含判定が再帰的になって
/// 扱いにくい——先に潰しておく。
fn flatten_leaves(
    children: &[(sim_math::Transform, Shape)],
    parent: sim_math::Transform,
) -> Vec<(sim_math::Transform, Shape)> {
    let mut out = Vec::new();
    for (xf, shape) in children {
        let world = parent.compose(*xf);
        match shape {
            Shape::Compound { children: inner } => out.extend(flatten_leaves(inner, world)),
            // 体積を持たない形状は union に寄与しない。
            Shape::Plane { .. } => {}
            other => out.push((world, other.clone())),
        }
    }
    out
}

/// 葉形状のローカル→親フレームでのAABB。
fn leaf_aabb(shape: &Shape, xf: sim_math::Transform) -> Aabb {
    match shape {
        Shape::Sphere { radius } => {
            let r = Vec3::new(*radius, *radius, *radius);
            Aabb {
                min: xf.position - r,
                max: xf.position + r,
            }
        }
        Shape::Capsule {
            radius,
            half_height,
        } => {
            // 芯線(ローカル+y)の両端を包む球2つのAABB。
            let axis = xf.rotation.rotate(Vec3::new(0.0, *half_height, 0.0));
            let r = Vec3::new(*radius, *radius, *radius);
            let (a, b) = (xf.position + axis, xf.position - axis);
            Aabb {
                min: Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)) - r,
                max: Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)) + r,
            }
        }
        Shape::Box { half_extents } => {
            let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for &sx in &[-1.0, 1.0] {
                for &sy in &[-1.0, 1.0] {
                    for &sz in &[-1.0, 1.0] {
                        let p = xf.apply_point(Vec3::new(
                            sx * half_extents.x,
                            sy * half_extents.y,
                            sz * half_extents.z,
                        ));
                        min = Vec3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
                        max = Vec3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
                    }
                }
            }
            Aabb { min, max }
        }
        Shape::ConvexMesh { vertices } => {
            let world: Vec<Vec3> = vertices.iter().map(|&v| xf.apply_point(v)).collect();
            points_aabb(&world).unwrap_or(Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            })
        }
        Shape::Plane { .. } | Shape::Compound { .. } => Aabb {
            min: Vec3::ZERO,
            max: Vec3::ZERO,
        },
    }
}

fn aabbs_overlap(a: Aabb, b: Aabb) -> bool {
    a.min.x < b.max.x
        && a.max.x > b.min.x
        && a.min.y < b.max.y
        && a.max.y > b.min.y
        && a.min.z < b.max.z
        && a.max.z > b.min.z
}

/// 回転していない`Box`だけで構成されている場合の**厳密な** union 体積
/// (座標圧縮 / Klee の測度問題)。1つでも箱以外・回転ありがあれば `None`。
fn axis_aligned_box_union_volume(leaves: &[(sim_math::Transform, Shape)]) -> Option<f64> {
    let mut boxes: Vec<Aabb> = Vec::with_capacity(leaves.len());
    for (xf, shape) in leaves {
        let Shape::Box { half_extents } = shape else {
            return None;
        };
        // 回転が実質恒等でなければこの経路は使えない。
        if !rotation_is_identity(xf.rotation) {
            return None;
        }
        boxes.push(Aabb {
            min: xf.position - *half_extents,
            max: xf.position + *half_extents,
        });
    }

    // 各軸の境界値を集めて昇順・重複除去(座標圧縮)。
    let axis_values = |pick: fn(Vec3) -> f64| -> Vec<f64> {
        let mut v: Vec<f64> = boxes
            .iter()
            .flat_map(|b| [pick(b.min), pick(b.max)])
            .collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.dedup();
        v
    };
    let xs = axis_values(|p| p.x);
    let ys = axis_values(|p| p.y);
    let zs = axis_values(|p| p.z);

    // セル数が過大なら諦める(部品数が多い場合の保険。O(n^3)セル)。
    if xs.len().saturating_mul(ys.len()).saturating_mul(zs.len()) > 2_000_000 {
        return None;
    }

    let mut total = 0.0;
    for i in 0..xs.len().saturating_sub(1) {
        for j in 0..ys.len().saturating_sub(1) {
            for k in 0..zs.len().saturating_sub(1) {
                // セルの中心がどれかの箱に入っていれば、セル全体が入っている
                // (境界値で切ってあるのでセルは各箱に対して「全部入り」か
                //  「全く入らない」かのどちらかしかない)。
                let c = Vec3::new(
                    0.5 * (xs[i] + xs[i + 1]),
                    0.5 * (ys[j] + ys[j + 1]),
                    0.5 * (zs[k] + zs[k + 1]),
                );
                let inside = boxes.iter().any(|b| {
                    c.x >= b.min.x
                        && c.x <= b.max.x
                        && c.y >= b.min.y
                        && c.y <= b.max.y
                        && c.z >= b.min.z
                        && c.z <= b.max.z
                });
                if inside {
                    total += (xs[i + 1] - xs[i]) * (ys[j + 1] - ys[j]) * (zs[k + 1] - zs[k]);
                }
            }
        }
    }
    Some(total)
}

fn rotation_is_identity(q: sim_math::Quat) -> bool {
    // w=±1 なら回転角0(符号の違いは同じ姿勢を表す)。
    (q.w.abs() - 1.0).abs() < 1e-12 && q.x.abs() < 1e-12 && q.y.abs() < 1e-12 && q.z.abs() < 1e-12
}

/// Monte Carlo のサンプル数(`union_volume` のdocに誤差評価あり)。
const UNION_MONTE_CARLO_SAMPLES: usize = 200_000;

/// 層化 Monte Carlo による union 体積の推定(`union_volume` のdoc参照)。
fn monte_carlo_union_volume(leaves: &[(sim_math::Transform, Shape)], boxes: &[Aabb]) -> f64 {
    // 全体のAABB(サンプリング領域)。
    let mut min = boxes[0].min;
    let mut max = boxes[0].max;
    for b in &boxes[1..] {
        min = Vec3::new(min.x.min(b.min.x), min.y.min(b.min.y), min.z.min(b.min.z));
        max = Vec3::new(max.x.max(b.max.x), max.y.max(b.max.y), max.z.max(b.max.z));
    }
    let size = max - min;
    let bounding_volume = size.x * size.y * size.z;
    if bounding_volume <= 0.0 {
        return 0.0;
    }

    // 加法的再帰(Weyl)列。3次元に均一かつ決定論的な低食い違い量列で、
    // 定数は plastic number の冪の逆数(Roberts の R_d 列)。
    const A1: f64 = 0.819_172_513_396_164_4;
    const A2: f64 = 0.671_043_606_703_789_9;
    const A3: f64 = 0.549_700_477_901_802_6;

    let hulls: Vec<Option<crate::hull::ConvexHull>> = leaves
        .iter()
        .map(|(_, s)| match s {
            Shape::ConvexMesh { vertices } => crate::hull::convex_hull(vertices),
            _ => None,
        })
        .collect();

    let mut hits = 0usize;
    for n in 0..UNION_MONTE_CARLO_SAMPLES {
        let t = (n + 1) as f64;
        let u = (t * A1).fract();
        let v = (t * A2).fract();
        let w = (t * A3).fract();
        let p = Vec3::new(min.x + u * size.x, min.y + v * size.y, min.z + w * size.z);
        let inside = leaves
            .iter()
            .zip(&hulls)
            .any(|((xf, s), hull)| leaf_contains_point(s, *xf, p, hull.as_ref()));
        if inside {
            hits += 1;
        }
    }
    bounding_volume * hits as f64 / UNION_MONTE_CARLO_SAMPLES as f64
}

/// 点`p`(親フレーム)が葉形状の内部にあるか。
fn leaf_contains_point(
    shape: &Shape,
    xf: sim_math::Transform,
    p: Vec3,
    hull: Option<&crate::hull::ConvexHull>,
) -> bool {
    // 形状ローカルへ戻してから判定する。
    let local = xf.inverse().apply_point(p);
    match shape {
        Shape::Sphere { radius } => local.length_sq() <= radius * radius,
        Shape::Box { half_extents } => {
            local.x.abs() <= half_extents.x
                && local.y.abs() <= half_extents.y
                && local.z.abs() <= half_extents.z
        }
        Shape::Capsule {
            radius,
            half_height,
        } => {
            // 芯線(ローカル+y、±half_height)への距離。
            let clamped_y = local.y.clamp(-half_height, *half_height);
            let d = local - Vec3::new(0.0, clamped_y, 0.0);
            d.length_sq() <= radius * radius
        }
        Shape::ConvexMesh { .. } => hull.is_some_and(|h| h.contains(local, 0.0)),
        Shape::Plane { .. } | Shape::Compound { .. } => false,
    }
}

/// `ConvexMesh`の質量特性。**3D凸包を実際に張って厳密に積分する**
/// (群11、`crate::hull`参照)。
///
/// 移行前は頂点群のAABBによる直方体近似で、凸包はAABBに内接するため常に
/// 過大評価だった(正四面体で体積3倍・正八面体で6倍)。いまは面三角形ごとの
/// 符号付き四面体分解で体積・重心・慣性テンソルのすべてが解析的に厳密。
///
/// 3次元的な広がりを持たない退化入力(点/直線/平面上、頂点3個以下)は
/// 体積を持たないので `None`(`Plane` と同じ扱い)。
fn convex_mesh_mass_properties(vertices: &[Vec3]) -> Option<MassProperties> {
    crate::hull::convex_hull(vertices)?.mass_properties()
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

    /// **重なりの無い`Compound`は単純和のまま厳密**(群11で union を入れても
    /// この経路の数値は1ビットも変わらないことを固定する)。
    /// `compound_volume_is_the_sum_of_its_children` と対になるテスト。
    #[test]
    fn disjoint_compound_volume_is_still_the_exact_sum() {
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::ZERO),
                    Shape::Box {
                        half_extents: Vec3::new(0.5, 0.5, 0.5),
                    },
                ),
                (
                    identity_transform(Vec3::new(10.0, 0.0, 0.0)),
                    Shape::Sphere { radius: 1.0 },
                ),
            ],
        };
        let expected = 1.0 + 4.0 / 3.0 * std::f64::consts::PI;
        // 厳密比較(Monte Carlo の推定誤差が混ざっていたらここで落ちる)。
        assert!((compound.volume().unwrap() - expected).abs() < 1e-15);
    }

    /// **重なった軸並行の箱2つの union が解析解と厳密に一致する**(群11)。
    ///
    /// 一辺2の立方体2つを x 方向に 1 だけずらして重ねる。
    /// 単純和は 8+8=16、重なりは 1×2×2=4 なので**真の union は 12**。
    /// 座標圧縮(Klee)の経路が厳密解を返すことを確認する。
    #[test]
    fn overlapping_axis_aligned_boxes_use_the_exact_union_volume() {
        let half = Vec3::new(1.0, 1.0, 1.0);
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::ZERO),
                    Shape::Box { half_extents: half },
                ),
                (
                    identity_transform(Vec3::new(1.0, 0.0, 0.0)),
                    Shape::Box { half_extents: half },
                ),
            ],
        };
        let volume = compound.volume().unwrap();
        assert!(
            (volume - 12.0).abs() < 1e-12,
            "重なり 4 を差し引いた 12 のはず(単純和は 16): {volume}"
        );
    }

    /// **実使用のL字コンパウンド**(`sim-wasm::spawn_compound_l_shape`と同一形状)
    /// の union 体積が解析解と厳密に一致すること(群11)。
    ///
    /// - 縦棒: 中心 (0, 0.75, 0)、半寸 (0.25, 1.0, 0.25) → 体積 0.5
    /// - 横棒: 中心 (0.25, -0.25, 0)、半寸 (0.5, 0.25, 0.25) → 体積 0.25
    /// - 交差領域: x∈[-0.25,0.25](幅0.5)、y∈[-0.25,0](厚み0.25)、
    ///   z∈[-0.25,0.25](幅0.5) → 0.0625
    ///
    /// よって単純和 0.75 に対し**真の union は 0.6875**。移行前はこの 0.75 を
    /// そのまま質量に使っており、**9%の質量過大評価**だった。
    #[test]
    fn l_shaped_compound_union_volume_matches_the_analytic_value() {
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::new(0.0, 0.75, 0.0)),
                    Shape::Box {
                        half_extents: Vec3::new(0.25, 1.0, 0.25),
                    },
                ),
                (
                    identity_transform(Vec3::new(0.25, -0.25, 0.0)),
                    Shape::Box {
                        half_extents: Vec3::new(0.5, 0.25, 0.25),
                    },
                ),
            ],
        };
        let volume = compound.volume().unwrap();
        assert!(
            (volume - 0.6875).abs() < 1e-12,
            "L字の真の union は 0.6875 のはず(単純和 0.75、重なり 0.0625): {volume}"
        );
        // 移行前の値へ戻っていないこと(退化した検証にしない)。
        assert!(
            (volume - 0.75).abs() > 1e-3,
            "単純和 0.75 に戻っている: {volume}"
        );
    }

    /// 完全に入れ子(小さい箱が大きい箱の内部)なら union は大きいほうの体積。
    /// 座標圧縮が「包含」を正しく扱えることの確認。
    #[test]
    fn fully_contained_child_does_not_add_volume() {
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::ZERO),
                    Shape::Box {
                        half_extents: Vec3::new(1.0, 1.0, 1.0),
                    },
                ),
                (
                    identity_transform(Vec3::ZERO),
                    Shape::Box {
                        half_extents: Vec3::new(0.3, 0.3, 0.3),
                    },
                ),
            ],
        };
        assert!((compound.volume().unwrap() - 8.0).abs() < 1e-12);
    }

    /// **Monte Carlo 経路(球どうしの重なり)が解析解に近いこと**(群11)。
    ///
    /// 半径 $r$ の等しい球を中心間距離 $d<2r$ で重ねたときのレンズ体積は
    /// $V_\cap=\frac{\pi(2r-d)^2(d^2+4dr)}{12d}$(標準的な球冠の公式)。
    /// union はその 2 球ぶんから交差を引いたもの。
    ///
    /// 許容誤差: `union_volume` の doc の誤差評価どおり、N=200,000 の
    /// 相対標準誤差は実用域で 0.5% 以下。統計的な揺らぎに 3σ 相当の余裕を
    /// 見て **2%** を上限とする(決定論的な列なので実行ごとにぶれはしないが、
    /// 「推定量として妥当な範囲」を要求する意図)。
    #[test]
    fn overlapping_spheres_union_volume_is_close_to_the_analytic_value() {
        let r = 1.0_f64;
        let d = 1.2_f64;
        let compound = Shape::Compound {
            children: vec![
                (identity_transform(Vec3::ZERO), Shape::Sphere { radius: r }),
                (
                    identity_transform(Vec3::new(d, 0.0, 0.0)),
                    Shape::Sphere { radius: r },
                ),
            ],
        };
        let sphere_volume = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
        let lens =
            std::f64::consts::PI * (2.0 * r - d).powi(2) * (d * d + 4.0 * d * r) / (12.0 * d);
        let expected = 2.0 * sphere_volume - lens;

        let volume = compound.volume().unwrap();
        let rel = (volume - expected).abs() / expected;
        assert!(
            rel < 2e-2,
            "Monte Carlo の union 推定が解析解から離れすぎ: \
             actual={volume} expected={expected} rel={rel:.4}"
        );
        // 単純和(交差を二重計上した値)よりは確実に小さいこと。
        assert!(
            volume < 2.0 * sphere_volume - 0.5 * lens,
            "重なりが差し引かれていない: {volume}"
        );
    }

    /// 同じ形状には**常に同じ体積**が返る(決定論。Monte Carlo 経路でも
    /// 乱数生成器を使わず固定の低食い違い量列を使うため)。
    #[test]
    fn union_volume_is_deterministic() {
        let make = || Shape::Compound {
            children: vec![
                (
                    identity_transform(Vec3::ZERO),
                    Shape::Sphere { radius: 1.0 },
                ),
                (
                    identity_transform(Vec3::new(1.2, 0.0, 0.0)),
                    Shape::Sphere { radius: 1.0 },
                ),
            ],
        };
        let a = make().volume().unwrap();
        let b = make().volume().unwrap();
        assert_eq!(a, b, "同じ形状なら厳密に同じ値であること");
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

    /// **かつてのAABB近似「カナリア」を、凸包実装の到達点として書き換えたもの**。
    ///
    /// 移行前このテストは、`ConvexMesh`が面情報を持たず体積・慣性を頂点群の
    /// AABBで代用していた時代の**誤り方**を固定していた——正四面体の体積を
    /// ちょうど3倍、正八面体をちょうど6倍に過大評価する、という比を
    /// assert していた。そして doc には「将来の3D凸包実装が入ったらこの
    /// テストは落ちる——その時は比が 1.0 になったことを確認する形へ意図的に
    /// 書き換える」と書かれていた。**群11でその時が来たので、宣言どおり
    /// 書き換えた**(落ちたこと自体が「近似が実装に置き換わった」という通知)。
    ///
    /// いまは `crate::hull` が実際に凸包を張り、面三角形ごとの符号付き四面体
    /// 分解で体積・重心・慣性を解析的に積分する。したがって近似ではなく
    /// **厳密な解析解との一致**を要求する:
    ///
    /// - 正四面体 $(\pm1,\pm1,\pm1)$ の交互4頂点。辺長 $a=2\sqrt2$、
    ///   $V=a^3/(6\sqrt2)=8/3$、慣性は等方で $I/m=a^2/20=0.4$。
    /// - 正八面体 $(\pm1,0,0),(0,\pm1,0),(0,0,\pm1)$。辺長 $a=\sqrt2$、
    ///   $V=4/3$、慣性は等方で $I/m=a^2/10=0.2$。
    ///
    /// どちらも正多面体(2本以上の3回対称軸を持つ点群)なので、対称性から
    /// **慣性テンソルは等方**——非対角成分は厳密にゼロ、対角3成分は等しい。
    /// これは「主軸の取り違え」に対する強い検出力を持つ。
    ///
    /// 許容誤差: 有理数・平方根の閉形式どうしの比較で、実装側も四面体分解の
    /// 有限個の積和(反復解法を含まない)なので、倍精度の丸め誤差だけを
    /// 見込んだ 1e-12(相対)。
    #[test]
    fn convex_mesh_hull_matches_the_exact_volume_and_inertia_of_regular_polyhedra() {
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

        for (shape, true_volume, true_inertia, label) in [
            (
                &tetrahedron,
                tetra_true_volume,
                edge * edge / 20.0,
                "正四面体",
            ),
            (&octahedron, octa_true_volume, 2.0 / 10.0, "正八面体"),
        ] {
            // 体積: 比が **1.0**(移行前は 3.0 / 6.0 だった)。
            let volume = shape.volume().unwrap();
            assert!(
                (volume / true_volume - 1.0).abs() < 1e-12,
                "{label}: 凸包の体積は解析解と厳密に一致するはず: \
                 actual={volume} true={true_volume} (比={})",
                volume / true_volume
            );

            // 重心: どちらも頂点の総和がゼロ = 原点対称なので厳密に原点。
            let com = shape.center_of_mass();
            assert!(com.length() < 1e-12, "{label}: 重心は原点のはず: {com:?}");

            // 慣性: 対称性から等方。対角3成分が解析値に一致し、非対角はゼロ。
            let tensor = shape.unit_mass_inertia_tensor();
            for k in 0..3 {
                assert!(
                    (tensor.m[k][k] / true_inertia - 1.0).abs() < 1e-12,
                    "{label}: I[{k}][{k}]={} は解析解 {true_inertia} と一致するはず",
                    tensor.m[k][k]
                );
            }
            for (i, j) in [(0usize, 1usize), (0, 2), (1, 2)] {
                assert!(
                    tensor.m[i][j].abs() < 1e-12 * true_inertia,
                    "{label}: 正多面体の慣性テンソルは等方なので I[{i}][{j}] はゼロのはず: {}",
                    tensor.m[i][j]
                );
            }

            // 移行前のAABB近似値(外接立方体、一辺2 → V=8、I/m=2/3)とは
            // **明確に違う**ことを固定する(退化した検証にしないため)。
            assert!(
                (volume - 8.0).abs() > 1.0,
                "{label}: AABBの体積 8 に戻っていないこと: {volume}"
            );
            assert!(
                (tensor.m[0][0] - 2.0 / 3.0).abs() > 1e-3,
                "{label}: AABBの慣性 2/3 に戻っていないこと: {}",
                tensor.m[0][0]
            );
        }
    }

    // ------------------------------------------------------------------
    // `Shape::from_triangle_mesh`(近似凸分解、`crate::decompose`)
    // ------------------------------------------------------------------

    /// **回帰**: すでに凸な三角形メッシュ(箱)は`from_triangle_mesh`を通しても
    /// 分解が起きず、`Shape::ConvexMesh{vertices}`を直接使った場合と
    /// 体積・慣性が完全に一致すること——分解の追加で既存の凸メッシュの
    /// 挙動が1ビットも変わらないことの固定(課題要件6)。
    #[test]
    fn from_triangle_mesh_of_a_convex_box_matches_plain_convex_mesh() {
        let (vertices, triangles) = crate::decompose::box_mesh(1.5);
        let decomposed = Shape::from_triangle_mesh(vertices.clone(), triangles);
        let plain = Shape::ConvexMesh { vertices };

        // 分解しても1パーツのままなので`ConvexMesh`のまま返ってくるはず。
        assert!(
            matches!(decomposed, Shape::ConvexMesh { .. }),
            "凸なメッシュはCompoundへ分解されないはず: {decomposed:?}"
        );

        let (dv, pv) = (decomposed.volume().unwrap(), plain.volume().unwrap());
        assert!(
            (dv - pv).abs() < 1e-12,
            "体積は完全一致のはず: {dv} vs {pv}"
        );
        let (dc, pc) = (decomposed.center_of_mass(), plain.center_of_mass());
        assert!(
            (dc - pc).length() < 1e-12,
            "重心は完全一致のはず: {dc:?} vs {pc:?}"
        );
        let (di, pi) = (
            decomposed.unit_mass_inertia_tensor(),
            plain.unit_mass_inertia_tensor(),
        );
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (di.m[i][j] - pi.m[i][j]).abs() < 1e-12,
                    "慣性テンソルは完全一致のはず: I[{i}][{j}]={} vs {}",
                    di.m[i][j],
                    pi.m[i][j]
                );
            }
        }
    }

    /// **本体**: 非凸なL字メッシュ(`crate::decompose::l_shaped_prism_mesh`、
    /// `l_shaped_compound_union_volume_matches_the_analytic_value`と同一形状・
    /// 真の体積0.6875)を`from_triangle_mesh`に通すと`Compound`になり、
    /// その体積が「頂点だけを渡して素朴に凸包扱いした場合」より真の値へ
    /// 明確に近いこと(課題要件5・6・7の統合テスト)。
    #[test]
    fn from_triangle_mesh_of_a_concave_l_shape_is_close_to_the_true_volume() {
        let (vertices, triangles) = crate::decompose::l_shaped_prism_mesh();
        let true_volume = 0.6875;

        // 素朴な扱い(=移行前の唯一の手段): 頂点だけ渡して凸包にする。
        let naive = Shape::ConvexMesh {
            vertices: vertices.clone(),
        };
        let naive_volume = naive.volume().unwrap();
        assert!(
            naive_volume > true_volume * 1.2,
            "素朴な凸包は真の体積より20%以上過大評価するはず: naive={naive_volume}"
        );

        // 分解あり: 非凸を認識してCompoundになるはず。
        let decomposed = Shape::from_triangle_mesh(vertices, triangles);
        assert!(
            matches!(decomposed, Shape::Compound { .. }),
            "非凸なL字はCompoundへ分解されるはず: {decomposed:?}"
        );
        let decomposed_volume = decomposed.volume().unwrap();
        let rel_error = (decomposed_volume - true_volume).abs() / true_volume;
        assert!(
            rel_error < 0.10,
            "分解後の体積は真の体積(0.6875)に近いはず: decomposed={decomposed_volume} \
             rel_error={rel_error:.4}"
        );
        assert!(
            (decomposed_volume - true_volume).abs() < (naive_volume - true_volume) * 0.5,
            "分解後の体積は素朴な凸包よりも真の値へ有意に近いはず: \
             decomposed={decomposed_volume} naive={naive_volume} true={true_volume}"
        );

        // 慣性・重心も有限で、対称形状(x=0面に対して対称なL字ではないが、
        // 有限領域内に収まる)であることの健全性チェック。
        let com = decomposed.center_of_mass();
        assert!(com.x.is_finite() && com.y.is_finite() && com.z.is_finite());
        let inertia = decomposed.unit_mass_inertia_diagonal();
        assert!(
            inertia.x > 0.0 && inertia.y > 0.0 && inertia.z > 0.0,
            "{inertia:?}"
        );
    }
}
