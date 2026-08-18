//! 自由表面・浮力(集中定数モデル)。設計: docs/11-fluid/04-free-surface-buoyancy.md。
//!
//! P1 スコープ: 直立姿勢(回転無視)の直方体のみ — F4(立方体喫水)・F5(浮体上下振動)は
//! いずれも直立浮体シナリオのため十分。球冠体積・水中抗力(§4 の `buoyancy_step`
//! 内 `F_d`)は Phase 3 に拡張する。
//!
//! 設計 §4 の表が挙げる「平面による凸多面体切断(頂点分類 + 切断多角形で閉じる)
//! → 四面体分割で体積・重心」は、**重力追従増分で切断平面の側だけ実装した**
//! (`submerged_box_below_plane`)——水面はどんな向きにも傾けられる。
//! 残るのは**剛体の姿勢**で、切られる直方体は依然ワールド軸に平行なままである
//! (下記「既知の限界」)。
//!
//! # 水域の一般化(`StaticWaterRegion` → `FluidRegion`)
//!
//! 移行前は「無限に広い静止水面」1枚(`StaticWaterRegion`)だけを表せた。設計 §3 が
//! 挙げる境界(AABB)・水温をこの型が持たなかったためで、シーンJSON側の
//! `fluids`も同じ縮約に縛られていた(`sim_world::scenario`モジュールdoc)。
//!
//! `FluidRegion`はここを3点だけ広げる:
//!
//! 1. **形状**(`FluidShape`): 移行前と同じ水平半空間(`HalfSpace`)に加え、
//!    軸並行直方体で横に閉じた水域(`Aabb`)を表せる。水槽・プールのように
//!    「そこにしか水が無い」配置を書けるようにするための最小の追加であり、
//!    球・任意凸形状は対象外(必要になった時点で変種を足す)。
//! 2. **温度**(`temperature`): 水温[K]を任意で持てる。**熱ドメインへの結合は
//!    まだ配線していない**(`FluidRegion::temperature`のdoc参照)。
//! 3. **複数領域**: 領域を持つ側(`sim_mechanics::MechanicsSolver::fluids`)が
//!    `Vec`になった。どの領域が効くかの決着規則はそちらのdocに書く。
//!
//! **浮力の式そのものは一切変えていない**——`FluidShape`が決めるのは
//! 「その領域が**どこに**効くか」だけで、水面下体積(`submerged_box_axis_aligned`)も
//! 力(`buoyancy_force`)も移行前と同一である。したがって`HalfSpace`の`FluidRegion`は
//! 移行前の`StaticWaterRegion`とビット単位で同じ挙動になる。
//!
//! # 浮力を重力場へ追従させる(**重力追従増分**)
//!
//! それまでこのモジュールは「水面はワールドy座標の水平面、浮力は`+y`向き」を
//! 固定していた。すなわち**重力の向きがワールド`-y`であること**が暗黙の前提で、
//! `sim_mechanics::GravityField`が`Uniform`の向きを可変にし`PointSource`まで
//! 表せるようになった後もそこへ追従できず、非`Uniform`な場では浮力を丸ごと
//! 無効化する(`MechanicsSolver::gravity`が0.0を返す)しかなかった。
//!
//! ここを2点だけ広げる:
//!
//! 1. **水面の向き**(`submerged_box_below_plane`): 自由表面を「上方向`up`に
//!    垂直な平面」として持つ。`water_level`の意味は移行前と同じ
//!    「原点から`up`軸に沿って測った符号付き距離」で、`up = (0,1,0)`なら
//!    移行前の「ワールドyの水平面」に**完全に一致する**。
//! 2. **力の向き**(`buoyancy_force`): 大きさ$\rho_f\,|g|\,V_\text{sub}$は
//!    そのままに、向きを`up`(=重力の逆向き)へ回す。
//!
//! **`up`は誰が決めるか**: 浮力は定義上「重力に逆らう向き」に出るので、
//! 剛体位置における**局所的な重力加速度の逆向き**を`up`とする
//! (`sim_mechanics::GravityField::up_and_magnitude_at`)。一様場では
//! 位置に依存しないので全剛体で同じ向きになり、点源場では剛体ごとに
//! 中心から外向きになる。
//!
//! **既知の限界(重力追従増分の残り)**:
//!
//! - **点源場の自由表面は球面ではなく接平面で近似する**。物理的には等ポテンシャル面
//!   (中心からの距離一定の球面)が自由表面だが、本モデルは剛体位置での局所鉛直に
//!   垂直な**平面**(原点から`up`軸に沿って`water_level`の距離)で置き換える。
//!   剛体の寸法が水域の曲率半径に対して十分小さい限りの近似であり、
//!   惑星規模の水域に小さな浮体を浮かべる用途を想定している。
//! - **重力が定義できない場(`GravityField::Zero`)では浮力は厳密に0**。
//!   「上」が無い場所に水平な自由表面は定義できず、押しのけた流体の重量も0だから
//!   ——`up`を発明して力を出すより、力が消えることを明示する方を選ぶ
//!   (`sim_mechanics::GravityField::up_and_magnitude_at`が`None`を返す)。
//! - **剛体の姿勢は依然として無視する**(モジュール冒頭の直立姿勢の縮約)。
//!   水面が傾いても切られる直方体はワールド軸に平行なままで、浮心から生じる
//!   復元トルクも積んでいない。傾いた重力の下で「浮体が姿勢を変えて落ち着く」
//!   復原性の再現は、一般姿勢の切断(設計 §4)を入れるまで対象外。

use sim_math::Vec3;

/// 流体領域の広がり(モジュールdoc「水域の一般化」参照)。
///
/// **`sim_mechanics::Aabb`を使わない理由**: `sim-mechanics`は`sim-fluid`に依存する
/// 側なので、こちらから参照すると依存が循環する。`min`/`max`の2点という中身は同じ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FluidShape {
    /// 水平半空間(無限に広い静止水面)。**移行前の`StaticWaterRegion`と同一**で、
    /// 「領域内か」の判定を一切行わない(`contains`は常に`true`)——水面より上に
    /// あるかどうかは浮力の式(`submerged_box_axis_aligned`が返す水面下体積)が
    /// 判断する、という移行前の役割分担をそのまま保つため。ここで
    /// `point.y <= water_level`のような判定を足すと、**水面をまたいで浮いている
    /// 剛体**(重心が水面より上にある浮体、D6がまさにそれ)が領域外と見なされて
    /// 浮力を失う。
    HalfSpace,
    /// 軸並行直方体(水槽・プール)。`contains`は基準点の**厳密な内外判定**。
    ///
    /// **既知の限界(正直な記録)**: 判定が基準点1点の内外なので、`max.y`を
    /// 水面(`water_level`)と同じ高さに置くと、浮上して重心が水面上へ出た剛体が
    /// 領域外に落ちて浮力を失い、沈んで戻ってはまた浮く、という不連続な
    /// チャタリングを起こす。**`max.y`は想定される浮上高さより上**(水槽の縁)に
    /// 取ること。剛体の広がりを考慮した部分的な内外判定(領域と剛体の交差体積)は
    /// 浮力の式そのものへの踏み込みになるため対象外とする。
    ///
    /// **重力追従増分の残り**: この境界箱は**ワールド軸に平行なまま**で、重力を
    /// 傾けても回らない(自由表面の向きだけが回る)。傾いた重力の下で「傾いた
    /// 水槽」を書きたい場合、この変種では表せない——領域に姿勢を持たせるのは
    /// `FluidShape`への別増分になる。
    Aabb { min: Vec3, max: Vec3 },
}

/// 流体領域。設計 §3 の `StaticWaterRegion` の一般化(モジュールdoc「水域の一般化」参照)。
///
/// 一様流(設計 §3 の流速)は引き続き対象外——`sim_fluid`側に静止水域中の
/// 相対流速を使う式が無く、持たせても読む先が無いため。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidRegion {
    /// 自由表面の位置——**「上」方向の軸に沿って原点から測った符号付き距離**。
    /// 浮力の式が読む唯一の幾何量で、`shape`が`Aabb`でも意味は変わらない
    /// (水槽が満水でない状態を書ける)。
    ///
    /// **重力追従増分で意味が広がった**: 「上」は局所的な重力の逆向きなので
    /// (モジュールdoc「浮力を重力場へ追従させる」)、重力が既定の`-y`向きなら
    /// これは移行前どおり**自由表面のワールドy座標**である。重力を傾ければ、
    /// 同じ値が傾いた軸に沿った距離を表す——水面もその向きに垂直な平面へ回る。
    pub water_level: f64,
    pub density: f64,
    /// 領域の広がり(既定は移行前と同じ`HalfSpace`)。
    pub shape: FluidShape,
    /// 水温 [K]。`None`(既定)なら温度を持たない。
    ///
    /// **正直な限界**: この値を読む結合はまだ**存在しない**。熱ドメインとの
    /// 対流結合(`sim_coupling::ConvectionLink`)は流体側も`ThermalNode`の
    /// index(`fluid_node`)で受け取る設計で、生の流体温度を熱源にする経路が
    /// 無いためである。ここは「シーンJSONで書けて、`World`から引けて、往復する」
    /// までを用意するデータ側の一歩で、実際に熱を動かすには
    /// `ConvectionLink`側へ流体領域を熱源に取る変種を足す必要がある(後続増分)。
    /// 引くには`sim_mechanics::MechanicsSolver::fluid_temperature_at`を使う。
    pub temperature: Option<f64>,
}

impl FluidRegion {
    /// 水平半空間の水域(**移行前の`StaticWaterRegion::new`と完全に同一**)。
    pub fn new(water_level: f64, density: f64) -> FluidRegion {
        FluidRegion {
            water_level,
            density,
            shape: FluidShape::HalfSpace,
            temperature: None,
        }
    }

    /// AABBで境界づけられた水域。`water_level`は自由表面の高さで、`max.y`とは
    /// 独立に与える(満水でない水槽を書けるようにするため)。
    pub fn aabb(min: Vec3, max: Vec3, water_level: f64, density: f64) -> FluidRegion {
        FluidRegion {
            water_level,
            density,
            shape: FluidShape::Aabb { min, max },
            temperature: None,
        }
    }

    /// 水温を与えた同じ領域を返す(`temperature`のdocの限界に注意)。
    pub fn with_temperature(self, temperature: f64) -> FluidRegion {
        FluidRegion {
            temperature: Some(temperature),
            ..self
        }
    }

    /// 基準点`point`がこの領域に属するか(`FluidShape`の各変種のdoc参照)。
    /// `HalfSpace`では常に`true`——判定を持たないのが移行前の挙動だから。
    pub fn contains(&self, point: Vec3) -> bool {
        match self.shape {
            FluidShape::HalfSpace => true,
            FluidShape::Aabb { min, max } => {
                point.x >= min.x
                    && point.x <= max.x
                    && point.y >= min.y
                    && point.y <= max.y
                    && point.z >= min.z
                    && point.z <= max.z
            }
        }
    }
}

/// 静水圧。設計 §2.1: p = p0 + ρ g d(d: 深さ、水面下で正)。F6。
pub fn hydrostatic_pressure(region: &FluidRegion, depth: f64, gravity: f64) -> f64 {
    region.density * gravity * depth.max(0.0)
}

/// 直立姿勢(ローカル+Y=ワールド+Y)の直方体の、**ワールドyの水平面**より下の
/// 体積と浮心。設計 §4 の一般姿勢切断アルゴリズムの直立特殊ケース(回転による
/// 姿勢依存は P1 では扱わない、モジュール冒頭注記)。
/// 戻り値: (V_sub, 浮心のワールド座標)。水面下体積が 0 なら浮心は無意味(body 中心を返す)。
///
/// **重力追従増分での位置づけ**: 任意の`up`を受ける
/// `submerged_box_below_plane`が入った後も、これは残す——`up = (0,1,0)`のときに
/// そちらが委譲する閉形式の本体であり、**移行前とビット単位で同一の演算順序**を
/// 保っているのがこの関数だからである(一般の切断アルゴリズムを`up=(0,1,0)`で
/// 走らせても数学的には同じ値になるが、丸めの経路が変わって既存シーンの
/// `state_hash`が動く)。呼び出し側が「水面はワールドyの水平面である」と
/// 決め打ってよい場面(その旨をdocに書いた検証テスト等)でも直接使える。
pub fn submerged_box_axis_aligned(
    center: Vec3,
    half_extents: Vec3,
    water_level: f64,
) -> (f64, Vec3) {
    let bottom = center.y - half_extents.y;
    let top = center.y + half_extents.y;
    let submerged_top = water_level.min(top);
    let h_sub = (submerged_top - bottom).clamp(0.0, 2.0 * half_extents.y);
    if h_sub <= 0.0 {
        return (0.0, center);
    }
    let base_area = 4.0 * half_extents.x * half_extents.z;
    let volume = base_area * h_sub;
    let centroid = Vec3::new(center.x, bottom + h_sub * 0.5, center.z);
    (volume, centroid)
}

/// 軸並行直方体のうち、自由表面 $\mathbf{x}\cdot\mathbf{up} = \texttt{water\_level}$
/// より**下**($\mathbf{x}\cdot\mathbf{up} \le \texttt{water\_level}$)にある部分の
/// 体積と浮心(**重力追従増分**、モジュールdoc「浮力を重力場へ追従させる」参照)。
///
/// `up`は**単位ベクトル**(重力の逆向き)、`water_level`は原点から`up`軸に沿って
/// 測った符号付き距離。`up = (0,1,0)`のとき`water_level`は移行前と同じ
/// 「水面のワールドy座標」になる。
///
/// # 実装
///
/// - `up == (0,1,0)`(重力が既定の`-y`向き=既存シーンすべて)は
///   `submerged_box_axis_aligned`へ委譲する。**移行前と1ビットも変えない**ための
///   分岐で、モデルとしては下の一般経路と同じ量を返す(同値であることは
///   `axis_aligned_fast_path_agrees_with_the_general_clip`が固定する)。
/// - それ以外は直方体を半空間で**厳密に**切る。6面をSutherland–Hodgmanで切り、
///   切断面に生じた辺から水面のフタを張って閉じた凸多面体を作り、
///   発散定理(原点を頂点とする四面体分割)で体積と重心を求める。近似は入らない
///   ——傾いた水面が直方体の角を切る形も、そのまま正しい体積になる。
///
/// 水面下体積が 0 なら浮心は無意味(body 中心を返す)。
pub fn submerged_box_below_plane(
    center: Vec3,
    half_extents: Vec3,
    up: Vec3,
    water_level: f64,
) -> (f64, Vec3) {
    if up.x == 0.0 && up.y == 1.0 && up.z == 0.0 {
        return submerged_box_axis_aligned(center, half_extents, water_level);
    }
    clip_box_below_plane(center, half_extents, up, water_level)
}

/// 直方体の8頂点(中心を原点に置いたローカル座標)を、面ごとの反時計回り
/// (外向き法線)の四角形として返す。`clip_box_below_plane`専用。
fn box_faces(h: Vec3) -> [[Vec3; 4]; 6] {
    let v = |sx: f64, sy: f64, sz: f64| Vec3::new(sx * h.x, sy * h.y, sz * h.z);
    let (v0, v1, v2, v3) = (
        v(-1.0, -1.0, -1.0),
        v(1.0, -1.0, -1.0),
        v(1.0, 1.0, -1.0),
        v(-1.0, 1.0, -1.0),
    );
    let (v4, v5, v6, v7) = (
        v(-1.0, -1.0, 1.0),
        v(1.0, -1.0, 1.0),
        v(1.0, 1.0, 1.0),
        v(-1.0, 1.0, 1.0),
    );
    [
        [v4, v5, v6, v7], // +z
        [v1, v0, v3, v2], // -z
        [v5, v1, v2, v6], // +x
        [v0, v4, v7, v3], // -x
        [v3, v7, v6, v2], // +y
        [v0, v1, v5, v4], // -y
    ]
}

/// 半空間 $\mathbf{x}\cdot\mathbf{up} \le \texttt{water\_level}$ による直方体の
/// 厳密な切断(`submerged_box_below_plane`の一般経路)。
fn clip_box_below_plane(
    center: Vec3,
    half_extents: Vec3,
    up: Vec3,
    water_level: f64,
) -> (f64, Vec3) {
    // 中心を原点へ移した座標で計算する(桁落ちを抑え、最後に中心を足し戻す)。
    // 中心系での水面のオフセット。
    let offset = water_level - center.dot(up);

    // 閉じた境界を張る三角形(すべて外向き)。
    let mut triangles: Vec<[Vec3; 3]> = Vec::with_capacity(24);
    // 水面のフタを扇状に張るときの要(切断面上の任意の1点でよい——符号付きの
    // 和なので、要をどこに取っても総和は変わらない)。
    let mut cap_pivot: Option<Vec3> = None;

    for face in box_faces(half_extents) {
        // Sutherland–Hodgman。`on_plane`は「その頂点が切断面上にあるか」で、
        // 距離を測り直すのではなく**生成の由来**で決める(交点として作った点は
        // 定義上 面上にあるが、再計算した距離は丸めで厳密に0にならない)。
        let mut clipped: Vec<(Vec3, bool)> = Vec::with_capacity(5);
        let mut has_strictly_outside = false;
        for i in 0..face.len() {
            let (current, next) = (face[i], face[(i + 1) % face.len()]);
            let (dc, dn) = (current.dot(up) - offset, next.dot(up) - offset);
            has_strictly_outside |= dc > 0.0;
            if dc <= 0.0 {
                clipped.push((current, dc == 0.0));
            }
            if (dc < 0.0 && dn > 0.0) || (dc > 0.0 && dn < 0.0) {
                let t = dc / (dc - dn);
                clipped.push((current + (next - current).scale(t), true));
            }
        }
        if clipped.len() < 3 {
            continue;
        }
        // 切り残った面(外向き)を扇状に三角形分割する。
        for i in 1..clipped.len() - 1 {
            triangles.push([clipped[0].0, clipped[i].0, clipped[i + 1].0]);
        }
        // 水面のフタ。**実際に切られた面だけ**(切断面より厳密に外側の頂点を
        // 持つ面だけ)がフタの辺を生む。単に「両端が切断面上にある辺」を拾うと、
        // 水面が直方体の面や稜線とちょうど重なったとき、切られてもいない元の稜線を
        // フタの辺として数えてしまう(全没なのに体積が過大になる)。
        // 逆にこの条件は「面が丸ごと切断面に乗っている」場合も落とす——その面は
        // フタそのもので既に上のループで積んであり、重ねると向きが逆の三角形で
        // 打ち消し合って穴が空く。
        //
        // なお、切られた面が元の稜線を丸ごと切断面上に持つことは無い: 面上で
        // 距離は一次関数なので、隣り合う2頂点が共に0なら残り2頂点は同符号になり、
        // その面は稜線1本(頂点2つ)しか残らず三角形分割の対象外へ落ちる。
        if !has_strictly_outside {
            continue;
        }
        for i in 0..clipped.len() {
            let (start, end) = (clipped[i], clipped[(i + 1) % clipped.len()]);
            if !(start.1 && end.1) {
                continue;
            }
            let pivot = *cap_pivot.get_or_insert(start.0);
            // 面が持つ向き(start→end)を反転して張ると、フタの法線は+up側=外向き。
            triangles.push([pivot, end.0, start.0]);
        }
    }

    // 発散定理: 原点を頂点とする四面体の符号付き体積の総和。
    let mut volume6 = 0.0;
    let mut moment = Vec3::ZERO;
    for [a, b, c] in triangles {
        let signed = a.dot(b.cross(c)); // = 6 * 四面体の符号付き体積
        volume6 += signed;
        moment = moment + (a + b + c).scale(signed);
    }
    if volume6 <= 0.0 {
        return (0.0, center);
    }
    // 重心 = Σ(V_i * 四面体重心) / ΣV_i、四面体重心 = (a+b+c+0)/4。
    (volume6 / 6.0, center + moment.scale(1.0 / (4.0 * volume6)))
}

/// アルキメデスの浮力。設計 §2.1: F_b = ρ_f |g| V_sub、向きは重力の逆
/// (`up`)。
///
/// **重力追従増分**で`up`を引数に取るようになった(モジュールdoc参照)。
/// `up = (0,1,0)`・`gravity`に重力加速度の大きさを与えれば、移行前の
/// 「常に`+y`向き」と**ビット単位で同一**の結果になる(乗算の順序も変えていない)。
pub fn buoyancy_force(volume_submerged: f64, fluid_density: f64, gravity: f64, up: Vec3) -> Vec3 {
    up.scale(fluid_density * gravity * volume_submerged)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 既定の重力(ワールド`-y`向き)に対応する上方向。
    const UP_Y: Vec3 = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };

    /// F6: 静水圧 p=ρgh(代数検算、docs/21-verification/01-analytic-tests.md F6)。
    #[test]
    fn f6_hydrostatic_pressure_matches_rho_g_h() {
        let region = FluidRegion::new(0.0, 998.2);
        let p = hydrostatic_pressure(&region, 2.0, 9.80665);
        assert!((p - 998.2 * 9.80665 * 2.0).abs() < 1e-9);
    }

    #[test]
    fn pressure_above_surface_is_zero() {
        let region = FluidRegion::new(0.0, 998.2);
        let p = hydrostatic_pressure(&region, -1.0, 9.80665);
        assert_eq!(p, 0.0);
    }

    /// F4 相当の体積検算: 密度比 r の一辺 a の立方体は水面下 r*a まで沈む
    /// (docs/11-fluid/04-free-surface-buoyancy.md §2.2)。V_sub は底面積×喫水深に一致する。
    #[test]
    fn submerged_volume_matches_waterline_depth() {
        let half = 0.5; // 一辺 1m
        let center = Vec3::new(0.0, -0.2, 0.0); // 喫水 0.5m相当まで沈める配置
        let (v, c) = submerged_box_axis_aligned(center, Vec3::new(half, half, half), 0.3);
        // bottom = -0.7, water_level=0.3 -> h_sub = 0.3-(-0.7) = 1.0 (全没)
        assert!((v - 1.0).abs() < 1e-12, "v={v}");
        assert!((c.y - (-0.2)).abs() < 1e-12);
    }

    #[test]
    fn fully_dry_box_has_zero_submerged_volume() {
        let (v, _) =
            submerged_box_axis_aligned(Vec3::new(0.0, 10.0, 0.0), Vec3::new(1.0, 1.0, 1.0), 0.0);
        assert_eq!(v, 0.0);
    }

    /// **アルキメデスの原理の閉形式解**(浮力が任意の重力場に追従するようになる
    /// 将来の変更に備えた基準点)。質量 $m$・水線面積 $A$ の直立直方体の釣り合い
    /// 喫水は $\rho_f\,g\,A\,d = m\,g$ を $d$ について解いた
    /// $d = m/(\rho_f A)$ で、**$g$ には依存しない**(両辺から消える)。
    ///
    /// ここでは `submerged_box_axis_aligned` が返す水面下体積と `buoyancy_force` が
    /// この閉形式の喫水で厳密に重量と釣り合うことを、代数計算だけで確認する
    /// (数値積分を挟まないので許容誤差は倍精度の丸め誤差ぶんの 1e-12(相対))。
    /// あわせて $g$ を変えても釣り合い喫水が動かないことも見る。
    ///
    /// **前提の明示(重力追従増分で更新)**: 本テストは**重力がワールド`-y`向きの
    /// 一様場である場合**の固定である。浮力は`up`(=重力の逆向き)へ追従する
    /// ようになったので(モジュールdoc「浮力を重力場へ追従させる」)、水面が
    /// 「ワールドy座標の水平面」で浮力が`+y`向きなのは`up = (0,1,0)`のときだけ。
    /// 傾いた重力での対応物は
    /// `sim_coupling::buoyancy_drag`の
    /// `floating_box_under_tilted_gravity_settles_along_the_tilted_up_axis`が見る。
    #[test]
    fn equilibrium_draft_matches_archimedes_closed_form() {
        let water_density = 998.2;
        let half = Vec3::new(0.5, 0.5, 0.5);
        let waterline_area = 4.0 * half.x * half.z;
        let water_level = 0.0;

        for &ratio in &[0.3, 0.6, 0.9] {
            let mass = ratio * water_density * 8.0 * half.x * half.y * half.z;
            // ρ_f g A d = m g  ⇔  d = m / (ρ_f A)。
            let draft = mass / (water_density * waterline_area);
            let center_y = water_level - draft + half.y;

            for &gravity in &[9.80665, 1.62, 24.79] {
                let (v_sub, centroid) =
                    submerged_box_axis_aligned(Vec3::new(0.0, center_y, 0.0), half, water_level);
                assert!(
                    (v_sub - waterline_area * draft).abs() / v_sub < 1e-12,
                    "V_sub = A·d のはず: v_sub={v_sub} A·d={}",
                    waterline_area * draft
                );
                // 浮心は水面下部分の中心(重量との釣り合いには効かないが、
                // 姿勢に効くので位置も固定しておく)。
                let expected_centroid_y = center_y - half.y + 0.5 * draft;
                assert!((centroid.y - expected_centroid_y).abs() < 1e-12);

                let buoyancy = buoyancy_force(v_sub, water_density, gravity, UP_Y).y;
                let weight = mass * gravity;
                assert!(
                    (buoyancy - weight).abs() / weight < 1e-12,
                    "釣り合い喫水では浮力と重量が厳密に一致する(gには依存しない): \
                     ratio={ratio} gravity={gravity} buoyancy={buoyancy} weight={weight}"
                );
            }
        }
    }

    /// `HalfSpace`は**内外判定を持たない**(常に`true`)。移行前の
    /// `StaticWaterRegion`が判定そのものを持っていなかったことの固定
    /// ——ここが`point.y <= water_level`になると、水面をまたいで浮いている
    /// 浮体(D6)が領域外になって浮力を失う(`FluidShape::HalfSpace`のdoc)。
    #[test]
    fn half_space_region_contains_every_point_including_those_above_the_waterline() {
        let region = FluidRegion::new(0.0, 998.2);
        assert!(region.contains(Vec3::new(0.0, -5.0, 0.0)));
        assert!(region.contains(Vec3::new(1e6, 5.0, -1e6)));
        assert_eq!(region.shape, FluidShape::HalfSpace);
        assert_eq!(region.temperature, None);
    }

    /// `Aabb`は基準点の厳密な内外判定(境界上は内側)。
    #[test]
    fn aabb_region_contains_only_points_inside_the_box() {
        let region = FluidRegion::aabb(
            Vec3::new(-1.0, -2.0, -1.0),
            Vec3::new(1.0, 0.5, 1.0),
            0.0,
            998.2,
        );
        assert!(region.contains(Vec3::new(0.0, -1.0, 0.0)));
        // 境界上は内側に含める(閉区間)。
        assert!(region.contains(Vec3::new(-1.0, 0.5, 1.0)));
        // 横に外れる: 「水面下の高さ」でも領域外。
        assert!(!region.contains(Vec3::new(5.0, -1.0, 0.0)));
        assert!(!region.contains(Vec3::new(0.0, -1.0, -3.0)));
        // 上下に外れる。
        assert!(!region.contains(Vec3::new(0.0, 1.0, 0.0)));
        assert!(!region.contains(Vec3::new(0.0, -3.0, 0.0)));
    }

    /// `water_level`は`max.y`とは独立(満水でない水槽を書ける)。
    /// 水温は`with_temperature`で足しても他のフィールドを変えない。
    #[test]
    fn aabb_region_keeps_water_level_and_temperature_independent_of_its_bounds() {
        let region = FluidRegion::aabb(
            Vec3::new(-1.0, -2.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            -0.5, // 縁(max.y=1.0)より下の水面 = 満水ではない水槽
            998.2,
        )
        .with_temperature(280.0);
        assert_eq!(region.water_level, -0.5);
        assert_eq!(region.temperature, Some(280.0));
        assert_eq!(
            region.shape,
            FluidShape::Aabb {
                min: Vec3::new(-1.0, -2.0, -1.0),
                max: Vec3::new(1.0, 1.0, 1.0),
            }
        );
        // 水面より上・縁より下の点も「領域内」——浮力を出すかは水面下体積が決める。
        assert!(region.contains(Vec3::new(0.0, 0.8, 0.0)));
        let (v_sub, _) = submerged_box_axis_aligned(
            Vec3::new(0.0, 0.8, 0.0),
            Vec3::new(0.1, 0.1, 0.1),
            region.water_level,
        );
        assert_eq!(v_sub, 0.0);
    }

    // ------------------------------------------------------------------
    // **重力追従増分**: 任意の`up`に対する切断(`submerged_box_below_plane`)。
    // ------------------------------------------------------------------

    /// `up = (0,1,0)`の速い経路(移行前と同一の閉形式)と、一般の切断アルゴリズムが
    /// **同じ量**を返すこと。速い経路は「丸めの経路を移行前のまま保つための分岐」で
    /// あってモデルの違いではない、という`submerged_box_below_plane`のdocの主張を
    /// ここで固定する(倍精度の丸めぶんだけずれるので厳密一致は要求しない)。
    #[test]
    fn axis_aligned_fast_path_agrees_with_the_general_clip() {
        let half = Vec3::new(0.4, 0.7, 0.25);
        for &center_y in &[-2.0, -0.5, -0.1, 0.0, 0.3, 0.69, 0.71, 2.0] {
            let center = Vec3::new(0.3, center_y, -0.8);
            let (v_fast, c_fast) = submerged_box_below_plane(center, half, UP_Y, 0.0);
            assert_eq!(
                (v_fast, c_fast),
                submerged_box_axis_aligned(center, half, 0.0),
                "速い経路は移行前の閉形式そのもの"
            );
            let (v_general, c_general) = clip_box_below_plane(center, half, UP_Y, 0.0);
            assert!(
                (v_general - v_fast).abs() <= 1e-12 * half.x * half.y * half.z * 8.0 + 1e-15,
                "center_y={center_y} v_fast={v_fast} v_general={v_general}"
            );
            if v_fast > 0.0 {
                assert!(
                    (c_general - c_fast).length() < 1e-12,
                    "center_y={center_y} c_fast={c_fast:?} c_general={c_general:?}"
                );
            }
        }
    }

    /// 中心を通る平面はどんな向きでも直方体をちょうど半分にする(点対称性)。
    /// 浮心は`up`の逆向きに、解析解どおりの位置へ来る:
    /// `up=(1,1,0)/√2`で辺長1の立方体を切ると、水面下は底面が直角二等辺三角形の
    /// 三角柱で、その重心は $(-1/6, -1/6, 0)$。
    #[test]
    fn a_plane_through_the_center_halves_the_box_for_any_up_direction() {
        let half = Vec3::new(0.5, 0.5, 0.5);
        let diagonal = Vec3::new(1.0, 1.0, 0.0).normalize_or_zero();
        let (v, c) = submerged_box_below_plane(Vec3::ZERO, half, diagonal, 0.0);
        assert!((v - 0.5).abs() < 1e-12, "v={v}");
        assert!(
            (c - Vec3::new(-1.0 / 6.0, -1.0 / 6.0, 0.0)).length() < 1e-12,
            "c={c:?}"
        );

        // 向きを変えても半分(平面が中心を通る限り)。
        for up in [
            Vec3::new(0.3, 0.9, -0.2).normalize_or_zero(),
            Vec3::new(-1.0, 2.0, 3.0).normalize_or_zero(),
            Vec3::new(1.0, 1.0, 1.0).normalize_or_zero(),
        ] {
            let center = Vec3::new(1.0, -2.0, 0.5);
            let (v, c) = submerged_box_below_plane(center, half, up, center.dot(up));
            assert!((v - 0.5).abs() < 1e-12, "up={up:?} v={v}");
            // 浮心は必ず水面より下(`-up`側)へ来る。**`up`に平行とは限らない**
            // ——直方体は球ではないので、切り口に対称性が無い向きでは浮心が
            // 横方向の成分を持つ(それが復原モーメントの源だが、姿勢を扱わない
            // 本モデルではトルクとして積んでいない、モジュールdocの既知の限界)。
            let offset = c - center;
            assert!(offset.dot(up) < 0.0, "up={up:?} offset={offset:?}");
        }
    }

    /// 傾いた水面が直方体の**側面を横切る帯**にいる間、水面下体積は喫水の
    /// 一次関数になる(断面積が一定の区間)。辺長1の立方体を`up`を30°傾けて
    /// 切ると、断面積は $1/\cos30° = 1.1547\ldots$ で一定、その帯は
    /// $|s| \le 0.1830127\ldots$。$V(s) = 0.5 + s/\cos30°$ を厳密に確認する
    /// (この線形性が、傾いた重力での浮体が単振動になる根拠でもある)。
    #[test]
    fn a_tilted_waterline_cuts_a_constant_cross_section_band_of_the_box() {
        let half = Vec3::new(0.5, 0.5, 0.5);
        let tilt = 30.0_f64.to_radians();
        let up = Vec3::new(tilt.sin(), tilt.cos(), 0.0);
        let cross_section = 1.0 / tilt.cos();
        // 帯の端 = 「上側2頂点」の`up`成分 = |0.5*sin| - |0.5*cos| の絶対値。
        let band = (0.5 * tilt.sin() - 0.5 * tilt.cos()).abs();
        for &s in &[-0.15_f64, -0.1, 0.0, 0.05, 0.1, 0.18] {
            assert!(s.abs() < band, "帯の内側であること: s={s} band={band}");
            let (v, _) = submerged_box_below_plane(Vec3::ZERO, half, up, s);
            assert!(
                (v - (0.5 + cross_section * s)).abs() < 1e-12,
                "s={s} v={v} expected={}",
                0.5 + cross_section * s
            );
        }
    }

    /// 全没・完全露出は`up`の向きによらず「全体積」「0」になる。
    /// 直方体の`up`方向の半幅は支持関数 $|u_x|a + |u_y|b + |u_z|c$。
    #[test]
    fn a_tilted_plane_still_saturates_at_full_and_zero_submersion() {
        let half = Vec3::new(0.3, 0.6, 0.45);
        let total = 8.0 * half.x * half.y * half.z;
        let up = Vec3::new(-0.4, 0.8, 0.3).normalize_or_zero();
        let reach = up.x.abs() * half.x + up.y.abs() * half.y + up.z.abs() * half.z;
        let center = Vec3::new(2.0, -1.0, 3.0);
        let s0 = center.dot(up);

        let (v_full, c_full) = submerged_box_below_plane(center, half, up, s0 + reach + 1e-9);
        assert!(
            (v_full - total).abs() < 1e-12,
            "v_full={v_full} total={total}"
        );
        assert!((c_full - center).length() < 1e-12, "全没の浮心は箱の中心");

        let (v_dry, c_dry) = submerged_box_below_plane(center, half, up, s0 - reach - 1e-9);
        assert_eq!(v_dry, 0.0);
        assert_eq!(c_dry, center);
    }

    /// 一般の切断が本当に正しい体積を返すことを、**独立な数値積分**(細かい格子の
    /// 占有率カウント)で裏取りする。解析解を書き下せない「角を斜めに切る」形も
    /// 含めて総当たりで確かめるための足場で、精度は格子の粗さで決まるので
    /// 許容誤差は 3e-3(体積比)。
    #[test]
    fn the_general_clip_matches_a_brute_force_volume_integration() {
        let half = Vec3::new(0.5, 0.35, 0.6);
        let total = 8.0 * half.x * half.y * half.z;
        let n = 60;
        for up in [
            Vec3::new(1.0, 1.0, 0.0).normalize_or_zero(),
            Vec3::new(0.2, 0.9, -0.4).normalize_or_zero(),
            Vec3::new(-1.0, 0.3, 0.5).normalize_or_zero(),
            Vec3::new(1.0, 1.0, 1.0).normalize_or_zero(),
            Vec3::new(0.0, -1.0, 0.0),
        ] {
            for &s in &[-0.4, -0.2, 0.0, 0.15, 0.35] {
                let (v, _) = submerged_box_below_plane(Vec3::ZERO, half, up, s);
                let mut inside = 0usize;
                for i in 0..n {
                    for j in 0..n {
                        for k in 0..n {
                            let p = Vec3::new(
                                (2.0 * (i as f64 + 0.5) / n as f64 - 1.0) * half.x,
                                (2.0 * (j as f64 + 0.5) / n as f64 - 1.0) * half.y,
                                (2.0 * (k as f64 + 0.5) / n as f64 - 1.0) * half.z,
                            );
                            if p.dot(up) <= s {
                                inside += 1;
                            }
                        }
                    }
                }
                let sampled = total * inside as f64 / (n * n * n) as f64;
                assert!(
                    (v - sampled).abs() / total < 3e-3,
                    "up={up:?} s={s} clip={v} sampled={sampled}"
                );
            }
        }
    }

    /// `buoyancy_force`は`up`の向きへ、大きさ $\rho_f\,|g|\,V$ で出る
    /// (**重力追従増分**)。`up=(0,1,0)`なら移行前と厳密に同一の値。
    #[test]
    fn buoyancy_force_points_along_up_with_the_archimedes_magnitude() {
        let (density, gravity, volume) = (998.2, 9.80665, 0.37);
        let magnitude = density * gravity * volume;

        let vertical = buoyancy_force(volume, density, gravity, UP_Y);
        assert_eq!(vertical, Vec3::new(0.0, magnitude, 0.0), "移行前と同一");

        for up in [
            Vec3::new(1.0, 1.0, 0.0).normalize_or_zero(),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-0.3, -0.5, 0.81).normalize_or_zero(),
        ] {
            let f = buoyancy_force(volume, density, gravity, up);
            assert!(
                (f.length() - magnitude).abs() / magnitude < 1e-15,
                "f={f:?}"
            );
            assert!(
                (f.normalize_or_zero() - up).length() < 1e-15,
                "f={f:?} up={up:?}"
            );
        }
    }

    #[test]
    fn partial_submersion_volume_is_base_area_times_depth() {
        let half = Vec3::new(0.5, 0.5, 0.5);
        // center.y=0 -> bottom=-0.5, top=0.5, water_level=0.0 -> h_sub=0.5
        let (v, c) = submerged_box_axis_aligned(Vec3::ZERO, half, 0.0);
        assert!((v - (1.0 * 0.5)).abs() < 1e-12, "v={v}");
        assert!((c.y - (-0.25)).abs() < 1e-12);
    }
}
