//! 2Dスケッチ(多角形ブーリアン・耳刈り三角形分割・押し出し)。
//! 設計: docs/22-roadmap/03-editor-todo.md「3D CADモデリング(スケッチ・
//!       押し出し・ブーリアン)を実装する」、
//!       docs/10-mechanics/02-collision-detection.md §4.2(生成された形状の行き先)。
//!
//! ## 何を解決するか
//!
//! `crate::decompose`(近似凸分解)は「任意の三角形メッシュ」を受け取れるように
//! なったが、**そのメッシュを作る手段**がエディタ側に無かった——UIから作れる形状は
//! `spawn_*`の固定レシピ(球・箱・カプセル・L字・立方体の8隅)だけで、
//! ユーザーが自分の形を作る経路が存在しない。本モジュールはCADの標準的な
//! 「スケッチ→押し出し」で、その経路を開く:
//!
//! 1. ユーザーが構築平面上に閉じた2D多角形(プロファイル)を描く。
//! 2. 複数のプロファイルを**2Dのまま**ブーリアン演算(和/差/積)で合成する。
//! 3. 合成結果(凹み・穴を含みうる)を三角形分割し、深さ方向へ押し出して
//!    角柱メッシュ(上下のキャップ+側壁)にする。
//! 4. 出来たメッシュを`Shape::from_triangle_mesh`へ渡す(=近似凸分解を経て
//!    実際に当たり判定・質量特性を持つ剛体になる)。
//!
//! ## なぜ2Dでブーリアンするのか(3D CSGを実装しない理由)
//!
//! 任意の3D三角形メッシュ同士の厳密なブーリアン(watertightを保ったまま
//! 交線で再三角形分割する)は、退化配置(面が同一平面・辺が同一直線・頂点が
//! 面上)の処理が本質的に難しく、自前実装で堅牢性を担保できる規模を超える。
//! 「外部クレート実質ゼロ」の方針でメッシュブーリアンのクレートを持ち込む
//! 選択肢も無い。
//!
//! 一方、**押し出し前の2Dプロファイル同士**のブーリアンは平面上の問題に
//! 落ちるので、辺の交点計算と内外判定だけで閉じる。しかもCADの実務では
//! これが標準的な作図手順そのもの(「四角を2つ描いて片方を引き、切り欠きの
//! ある断面を押し出す」)であり、表現力の損失は「押し出し方向に一様でない
//! 立体は作れない」に留まる。角柱に限れば、L字・コの字・穴あき板といった
//! 実用的な非凸形状はすべてこの経路で作れる。
//!
//! ## 領域(region)の表現
//!
//! 平面領域は**ループの集合**`&[Vec<Point2>]`で表す。外形ループは反時計回り
//! (CCW、符号付き面積が正)、穴ループは時計回り(CW、負)という標準的な規約で、
//! 内外判定は**巻き数(winding number)が0でなければ内側**とする。この1つの
//! 表現で「穴あき」「複数の島」「差集合で2つに割れた結果」がすべて書ける。
//!
//! ## ブーリアンのアルゴリズム(Weiler–Atherton系)
//!
//! 1. 両領域の全辺の交点を求め、**両側の辺をそこで分割**する(片側の頂点が
//!    もう片側の辺の上に乗っているT字接合も分割点に含める——ここを落とすと
//!    後段の縫い合わせで端点が一致しない)。
//! 2. 全端点を許容誤差付きで統合し、有向辺を「点index対」にする。
//! 3. 各有向辺を、**中点**が相手領域に対して内/外/境界上(同方向/逆方向)の
//!    どれかで分類する。中点を使うのが要点——端点は必ず境界上にあるので
//!    分類できない。
//! 4. 演算ごとの選択表(`keep_subject`/`keep_clip`)で残す辺を選び、
//! 5. 端点をたどって閉ループへ縫い合わせる。
//!
//! 交点の無い配置(離れている・完全に内包している)も**特別扱い無しで**正しく
//! 落ちる:全辺が「外」または「内」に分類され、選択表がそのまま答えを出す。
//!
//! ## 三角形分割・押し出し
//!
//! 耳刈り(ear clipping)。穴は**ブリッジ辺**で外形へ繋いで1本の単純多角形に
//! 畳んでから刈る(earcut等と同じ標準手法)。$O(n^2)$ だが、`crate::hull`と
//! 同じ理由で——形状の構築時にしか走らない——正しさを優先した。
//!
//! 押し出しは「下キャップ+上キャップ+側壁」をすべて外向きの巻き順で出す。
//! ここで**下流の`crate::decompose`(近似凸分解)が働くようにメッシュを作る**
//! 3つの工夫が要る(いずれも実測で必要性を確認した実バグの修正である):
//!
//! - `triangle_quality`: 耳を「最も正三角形に近いもの」から刈る。
//! - `refine_region`: 断面の全頂点が作る軸並行の格子線で辺を切り直す。
//! - `split_into_hole_free_regions`: 穴のある断面は穴を通る線で先に切り分け、
//!   **別々の`Shape`**(`Compound`の子)として渡す。
//!
//! いずれも各関数のdocに「これが無いと何がどう壊れるか」を実測値つきで書いた。
//!
//! ## 意図的な縮約
//!
//! - **穴の中の島(入れ子の深さ2以上)は扱わない**。穴ループは「それを含む
//!   最小の外形ループ」に割り当て、その先の入れ子は見ない。スケッチ2〜3枚の
//!   ブーリアンでこの配置になることは実際上まず無い。
//! - **完全に退化した重なり**(面積0の接触、同一頂点の連続など)は許容誤差
//!   `EPS`で潰す。潰した結果ループが3頂点未満になれば捨てる。
//! - **押し出し方向は平面の法線1本のみ**。テーパー(抜き勾配)・回転押し出し
//!   (revolve)・スイープは扱わない——いずれも「断面をそのまま平行移動する」
//!   という前提そのものを変えるので、別の機能として設計するのが筋である。
//! - **3Dのブーリアン(CSG)は行わない**。合成は押し出し前の2D断面に対して
//!   のみで、既に押し出した立体同士を組み合わせることはできない
//!   (上記「なぜ2Dでブーリアンするのか」)。

use sim_math::Vec3;

/// 2D点。`[x, y]`。押し出し側では`x`が世界x・`y`が世界z(構築平面は
/// 地面 y=0)に対応するが、本モジュール自体は平面の向きを知らない。
pub type Point2 = [f64; 2];

/// 平面領域の1ループ(単純多角形の頂点列、始点と終点は繋がっているものとして
/// 重複させない)。外形はCCW・穴はCW。
pub type Loop2 = Vec<Point2>;

/// 座標の許容誤差[m]。エディタのスケッチはグリッドスナップ(既定0.1m)された
/// クリック座標なので、これより近い2点は同一とみなして問題ない。
const EPS: f64 = 1e-7;

/// 面積の許容誤差[m²]。`EPS`四方の三角形の面積程度。
const AREA_EPS: f64 = 1e-12;

/// 2Dプロファイル同士のブーリアン演算。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanOp {
    /// 和(A ∪ B)。
    Union,
    /// 差(A − B)。**順序が意味を持つ**唯一の演算。
    Subtract,
    /// 積(A ∩ B)。
    Intersect,
}

fn sub(a: Point2, b: Point2) -> Point2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn cross2(a: Point2, b: Point2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn dot2(a: Point2, b: Point2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn near(a: Point2, b: Point2) -> bool {
    (a[0] - b[0]).abs() <= EPS && (a[1] - b[1]).abs() <= EPS
}

/// 単純多角形の符号付き面積(CCWで正)。
pub fn loop_signed_area(points: &[Point2]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = points[i];
        let b = points[(i + 1) % n];
        sum += cross2(a, b);
    }
    sum * 0.5
}

/// 領域(ループ集合)の符号付き面積。穴(CW)は負に効くので、
/// 「外形−穴」がそのまま出る。
pub fn region_area(loops: &[Loop2]) -> f64 {
    loops.iter().map(|l| loop_signed_area(l)).sum()
}

/// 領域の面積重心。面積が0(退化)なら`None`。
///
/// 一様な厚みで押し出した角柱の重心は、この点の真上・真下の中点に来る
/// ——押し出したメッシュのローカル原点をここへ置くことで、剛体の重心が
/// ローカル原点と一致する(`RigidBodySet`の`center_of_mass`が実質ゼロになり、
/// 「置いた座標＝見た目の中心」という直感どおりの配置になる)。
pub fn region_centroid(loops: &[Loop2]) -> Option<Point2> {
    let mut area2 = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for l in loops {
        let n = l.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let a = l[i];
            let b = l[(i + 1) % n];
            let c = cross2(a, b);
            area2 += c;
            cx += (a[0] + b[0]) * c;
            cy += (a[1] + b[1]) * c;
        }
    }
    if area2.abs() <= AREA_EPS {
        return None;
    }
    Some([cx / (3.0 * area2), cy / (3.0 * area2)])
}

/// 生のクリック列を1本の妥当なCCWループへ正規化する。
///
/// - 連続する重複点(と、始点に重なる終点)を落とす。
/// - 3頂点未満、または面積が実質0なら`None`。
/// - 時計回りに描かれていれば反転してCCWに揃える(ユーザーがどちら回りに
///   クリックしたかを気にしなくて済むようにする)。
pub fn normalize_loop(points: &[Point2]) -> Option<Loop2> {
    let mut out: Vec<Point2> = Vec::with_capacity(points.len());
    for &p in points {
        if out.last().is_some_and(|&q| near(q, p)) {
            continue;
        }
        out.push(p);
    }
    while out.len() >= 2 && near(out[0], *out.last().unwrap()) {
        out.pop();
    }
    if out.len() < 3 {
        return None;
    }
    let area = loop_signed_area(&out);
    if area.abs() <= AREA_EPS {
        return None;
    }
    if area < 0.0 {
        out.reverse();
    }
    Some(out)
}

/// 点`p`の領域に対する巻き数。0でなければ内側(非ゼロ巻き数規則)。
///
/// 標準的な交差走査。走査線が頂点をちょうど通る退化を避けるため、
/// 上向きの辺は始点を含み終点を含まない(`y0 <= py < y1`)という
/// 半開区間の規約を使う。
fn winding_number(loops: &[Loop2], p: Point2) -> i32 {
    let mut w = 0;
    for l in loops {
        let n = l.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let a = l[i];
            let b = l[(i + 1) % n];
            if a[1] <= p[1] {
                if b[1] > p[1] && cross2(sub(b, a), sub(p, a)) > 0.0 {
                    w += 1;
                }
            } else if b[1] <= p[1] && cross2(sub(b, a), sub(p, a)) < 0.0 {
                w -= 1;
            }
        }
    }
    w
}

/// 点が領域の内部(境界は含まない——境界上かどうかは呼び出し側が
/// `edge_containing`で先に判定する)にあるか。
fn inside(loops: &[Loop2], p: Point2) -> bool {
    winding_number(loops, p) != 0
}

/// 点`p`が線分`a-b`の上(距離`EPS`以内、かつ端点の外側へはみ出していない)か。
fn on_segment(a: Point2, b: Point2, p: Point2) -> bool {
    let d = sub(b, a);
    let len2 = dot2(d, d);
    if len2 <= EPS * EPS {
        return near(a, p);
    }
    let t = dot2(sub(p, a), d) / len2;
    if !(-EPS..=1.0 + EPS).contains(&t) {
        return false;
    }
    let proj = [a[0] + d[0] * t, a[1] + d[1] * t];
    let e = sub(p, proj);
    dot2(e, e) <= EPS * EPS
}

/// 点`p`を含む領域の辺の向きを返す(複数あれば最初の1本)。
fn edge_direction_at(loops: &[Loop2], p: Point2) -> Option<Point2> {
    for l in loops {
        let n = l.len();
        if n < 2 {
            continue;
        }
        for i in 0..n {
            let a = l[i];
            let b = l[(i + 1) % n];
            if on_segment(a, b, p) {
                return Some(sub(b, a));
            }
        }
    }
    None
}

/// 有向辺の、相手領域に対する分類。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EdgeClass {
    /// 相手領域の内部にある。
    Inside,
    /// 相手領域の外部にある。
    Outside,
    /// 相手領域の境界と重なり、向きも同じ。
    SharedSame,
    /// 相手領域の境界と重なり、向きが逆。
    SharedOpposite,
}

/// 交点で分割済みの有向辺(点indexの対)。
struct Segment {
    from: usize,
    to: usize,
    class: EdgeClass,
}

/// 許容誤差で統合しながら点を溜める入れ物。スケッチ由来の点数(数十〜数百)
/// では線形探索で十分で、ハッシュの丸め方に起因する取りこぼしが無い。
struct PointPool {
    points: Vec<Point2>,
}

impl PointPool {
    fn new() -> Self {
        PointPool { points: Vec::new() }
    }

    fn intern(&mut self, p: Point2) -> usize {
        for (i, &q) in self.points.iter().enumerate() {
            if near(q, p) {
                return i;
            }
        }
        self.points.push(p);
        self.points.len() - 1
    }
}

/// 2つの平面領域のブーリアン演算。モジュールdoc「ブーリアンのアルゴリズム」参照。
///
/// 入力・出力とも「外形CCW・穴CW」の規約に従うループ集合。入力が規約から
/// 外れていても(向きが揃っていなくても)結果は不定にはならないが、
/// 意図した図形にはならない——`normalize_loop`を通した外形と、
/// 本関数の出力だけを渡すこと。
pub fn polygon_boolean(a: &[Loop2], b: &[Loop2], op: BooleanOp) -> Vec<Loop2> {
    let a: Vec<Loop2> = a.iter().filter(|l| l.len() >= 3).cloned().collect();
    let b: Vec<Loop2> = b.iter().filter(|l| l.len() >= 3).cloned().collect();
    if a.is_empty() {
        return match op {
            // A が空なら A∪B = B、A−B = ∅、A∩B = ∅。
            BooleanOp::Union => b,
            _ => Vec::new(),
        };
    }
    if b.is_empty() {
        return match op {
            BooleanOp::Intersect => Vec::new(),
            _ => a,
        };
    }

    let mut pool = PointPool::new();
    let a_segments = split_region(&a, &b, &mut pool);
    let b_segments = split_region(&b, &a, &mut pool);

    let mut selected: Vec<(usize, usize)> = Vec::new();
    for s in &a_segments {
        if keep_subject(op, s.class) {
            selected.push((s.from, s.to));
        }
    }
    for s in &b_segments {
        match keep_clip(op, s.class) {
            Some(false) => selected.push((s.from, s.to)),
            Some(true) => selected.push((s.to, s.from)), // 反転して取り込む(差集合の穴)。
            None => {}
        }
    }
    stitch(&pool.points, &selected)
}

/// 主体(A)側の辺を残すか。モジュールdocの選択表。
fn keep_subject(op: BooleanOp, class: EdgeClass) -> bool {
    match op {
        // 和: Bの外にあるA、および重なっている共有辺のA側の1本だけ。
        BooleanOp::Union => matches!(class, EdgeClass::Outside | EdgeClass::SharedSame),
        // 積: Bの内にあるA、および共有辺のA側1本。
        BooleanOp::Intersect => matches!(class, EdgeClass::Inside | EdgeClass::SharedSame),
        // 差: Bの外にあるA。逆向きの共有辺は「AとBが背中合わせに接している」
        // 境界なので、A−Bの外周として残る。同方向の共有辺はBに食われて消える。
        BooleanOp::Subtract => matches!(class, EdgeClass::Outside | EdgeClass::SharedOpposite),
    }
}

/// 従体(B)側の辺を残すか。`Some(reversed)`で残す(`reversed=true`なら
/// 向きを反転して取り込む)、`None`なら捨てる。
fn keep_clip(op: BooleanOp, class: EdgeClass) -> Option<bool> {
    match op {
        BooleanOp::Union => (class == EdgeClass::Outside).then_some(false),
        BooleanOp::Intersect => (class == EdgeClass::Inside).then_some(false),
        // 差: Aの内側に入り込んだBの辺は、向きを逆にすると穴(CW)の境界になる。
        BooleanOp::Subtract => (class == EdgeClass::Inside).then_some(true),
    }
}

/// 領域`region`の全辺を、相手`other`との交点(および相手の頂点が乗っている点)で
/// 分割し、分類まで済ませた有向辺列にする。
fn split_region(region: &[Loop2], other: &[Loop2], pool: &mut PointPool) -> Vec<Segment> {
    let mut out = Vec::new();
    for l in region {
        let n = l.len();
        for i in 0..n {
            let a = l[i];
            let b = l[(i + 1) % n];
            let d = sub(b, a);
            let len2 = dot2(d, d);
            if len2 <= EPS * EPS {
                continue;
            }
            // 辺 a-b 上の分割パラメータ t を集める。
            let mut ts: Vec<f64> = vec![0.0, 1.0];
            for ol in other {
                let m = ol.len();
                for j in 0..m {
                    let c = ol[j];
                    let e = ol[(j + 1) % m];
                    // (1) 相手の頂点が a-b に乗っているならそこで割る(T字接合)。
                    if on_segment(a, b, c) {
                        ts.push(dot2(sub(c, a), d) / len2);
                    }
                    // (2) 真の交差点。
                    if let Some((t, _)) = segment_intersection(a, b, c, e) {
                        ts.push(t);
                    }
                }
            }
            ts.retain(|t| t.is_finite() && (-EPS..=1.0 + EPS).contains(t));
            ts.sort_by(|x, y| x.partial_cmp(y).unwrap());
            let mut prev = f64::NEG_INFINITY;
            let mut cuts: Vec<f64> = Vec::with_capacity(ts.len());
            for t in ts {
                let t = t.clamp(0.0, 1.0);
                if t - prev > EPS {
                    cuts.push(t);
                    prev = t;
                }
            }
            for w in cuts.windows(2) {
                let (t0, t1) = (w[0], w[1]);
                let p0 = [a[0] + d[0] * t0, a[1] + d[1] * t0];
                let p1 = [a[0] + d[0] * t1, a[1] + d[1] * t1];
                if near(p0, p1) {
                    continue;
                }
                let mid = [(p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5];
                let class = classify(other, mid, d);
                let from = pool.intern(p0);
                let to = pool.intern(p1);
                if from == to {
                    continue;
                }
                out.push(Segment { from, to, class });
            }
        }
    }
    out
}

/// 中点`mid`(進行方向`dir`)を相手領域に対して分類する。
fn classify(other: &[Loop2], mid: Point2, dir: Point2) -> EdgeClass {
    if let Some(other_dir) = edge_direction_at(other, mid) {
        return if dot2(dir, other_dir) >= 0.0 {
            EdgeClass::SharedSame
        } else {
            EdgeClass::SharedOpposite
        };
    }
    if inside(other, mid) {
        EdgeClass::Inside
    } else {
        EdgeClass::Outside
    }
}

/// 線分`a-b`と`c-e`の交点(あれば`(t, u)`、`t`は`a-b`上のパラメータ)。
/// 平行(共線含む)は`None`——共線の重なりは分割点(1)の経路で拾われる。
fn segment_intersection(a: Point2, b: Point2, c: Point2, e: Point2) -> Option<(f64, f64)> {
    let r = sub(b, a);
    let s = sub(e, c);
    let denom = cross2(r, s);
    if denom.abs() <= 1e-14 {
        return None;
    }
    let qp = sub(c, a);
    let t = cross2(qp, s) / denom;
    let u = cross2(qp, r) / denom;
    if (-EPS..=1.0 + EPS).contains(&t) && (-EPS..=1.0 + EPS).contains(&u) {
        Some((t, u))
    } else {
        None
    }
}

/// 選ばれた有向辺を閉ループへ縫い合わせる。
///
/// 1頂点から複数の候補が出ている(自己接触した図形)場合は、進入方向から見て
/// **最も左へ曲がる**辺を選ぶ——平面グラフの面走査の標準的な規則で、
/// 外形を外形として、穴を穴としてたどれる。
fn stitch(points: &[Point2], segments: &[(usize, usize)]) -> Vec<Loop2> {
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new(); points.len()];
    for (i, &(from, _)) in segments.iter().enumerate() {
        outgoing[from].push(i);
    }
    let mut used = vec![false; segments.len()];
    let mut loops = Vec::new();

    for start in 0..segments.len() {
        if used[start] {
            continue;
        }
        let mut ring: Vec<usize> = Vec::new();
        let mut current = start;
        // 閉じるまでたどる。上限は全辺数(壊れた入力でも必ず止まる)。
        for _ in 0..=segments.len() {
            if used[current] {
                break;
            }
            used[current] = true;
            let (from, to) = segments[current];
            ring.push(from);
            if !ring.is_empty() && to == segments[start].0 {
                break; // 出発点へ戻った。
            }
            let in_dir = sub(points[to], points[from]);
            let mut best: Option<(f64, usize)> = None;
            for &cand in &outgoing[to] {
                if used[cand] {
                    continue;
                }
                let (cf, ct) = segments[cand];
                let out_dir = sub(points[ct], points[cf]);
                // 進入方向からの符号付き回転角(左が正)。
                let angle = cross2(in_dir, out_dir).atan2(dot2(in_dir, out_dir));
                if best.is_none_or(|(a, _)| angle > a) {
                    best = Some((angle, cand));
                }
            }
            match best {
                Some((_, next)) => current = next,
                None => break,
            }
        }
        let ring_points: Vec<Point2> = ring.iter().map(|&i| points[i]).collect();
        if let Some(cleaned) = clean_ring(&ring_points) {
            loops.push(cleaned);
        }
    }
    loops
}

/// 縫い合わせた生のリングから重複点・共線点を落とす。向きは**変えない**
/// (CCW/CWがそのまま外形/穴の区別になっている)。
fn clean_ring(points: &[Point2]) -> Option<Loop2> {
    let mut out: Vec<Point2> = Vec::with_capacity(points.len());
    for &p in points {
        if out.last().is_some_and(|&q| near(q, p)) {
            continue;
        }
        out.push(p);
    }
    while out.len() >= 2 && near(out[0], *out.last().unwrap()) {
        out.pop();
    }
    if out.len() < 3 {
        return None;
    }
    // 共線の中間点(面積に寄与しない)を落とす。
    let mut simplified: Vec<Point2> = Vec::with_capacity(out.len());
    let n = out.len();
    for i in 0..n {
        let prev = out[(i + n - 1) % n];
        let cur = out[i];
        let next = out[(i + 1) % n];
        if cross2(sub(cur, prev), sub(next, cur)).abs() <= AREA_EPS {
            continue;
        }
        simplified.push(cur);
    }
    if simplified.len() < 3 {
        return None;
    }
    if loop_signed_area(&simplified).abs() <= AREA_EPS {
        return None;
    }
    Some(simplified)
}

// ---------------------------------------------------------------------------
// 三角形分割(耳刈り + 穴のブリッジ)
// ---------------------------------------------------------------------------

/// 領域(外形CCW+穴CW)を三角形分割する。
///
/// 返り値の index は`loops`を順に連結した頂点列(`loop[0]`の全点 →
/// `loop[1]`の全点 → …)に対するもの。呼び出し側は`flatten_region`で
/// 同じ順序の頂点列を作れる。
///
/// 穴は外形へブリッジ辺で繋いでから刈るため、**同じ index が結果の複数の
/// 三角形に現れる**(ブリッジの両端)——これは正常で、1つの三角形の中に
/// 同じ index が2回現れることはない。
pub fn triangulate_region(loops: &[Loop2]) -> Vec<[usize; 3]> {
    let points = flatten_region(loops);
    // 各ループの index 範囲を求める。
    let mut ranges: Vec<(usize, usize)> = Vec::with_capacity(loops.len());
    let mut base = 0;
    for l in loops {
        ranges.push((base, base + l.len()));
        base += l.len();
    }

    // 外形(CCW)と穴(CW)に分ける。
    let mut outers: Vec<usize> = Vec::new();
    let mut holes: Vec<usize> = Vec::new();
    for (i, l) in loops.iter().enumerate() {
        if l.len() < 3 {
            continue;
        }
        if loop_signed_area(l) >= 0.0 {
            outers.push(i);
        } else {
            holes.push(i);
        }
    }

    // 穴を「それを含む最小面積の外形」へ割り当てる(入れ子の深さ2以上は
    // 見ない——モジュールdoc「意図的な縮約」)。
    let mut assigned: Vec<Vec<usize>> = vec![Vec::new(); loops.len()];
    for &h in &holes {
        let probe = loops[h][0];
        let mut best: Option<(f64, usize)> = None;
        for &o in &outers {
            if winding_number(std::slice::from_ref(&loops[o]), probe) != 0 {
                let area = loop_signed_area(&loops[o]);
                if best.is_none_or(|(a, _)| area < a) {
                    best = Some((area, o));
                }
            }
        }
        if let Some((_, o)) = best {
            assigned[o].push(h);
        }
    }

    let mut triangles = Vec::new();
    for &o in &outers {
        let outer_ring: Vec<usize> = (ranges[o].0..ranges[o].1).collect();
        let hole_rings: Vec<Vec<usize>> = assigned[o]
            .iter()
            .map(|&h| (ranges[h].0..ranges[h].1).collect())
            .collect();
        let mut ring = bridge_holes(&points, outer_ring, &hole_rings);
        ear_clip(&points, &mut ring, &mut triangles);
    }
    triangles
}

/// `triangulate_region`の index 空間に対応する頂点列。
pub fn flatten_region(loops: &[Loop2]) -> Vec<Point2> {
    loops.iter().flat_map(|l| l.iter().copied()).collect()
}

/// 穴ループを外形ループへブリッジ辺で繋ぎ、1本の単純多角形(の index 列)にする。
///
/// 標準手法: 穴の最も右(x最大)の頂点Mから+x方向へ射線を飛ばし、最初に
/// 当たる外形の辺を見つけて、その辺の端点(または射線と辺が作る三角形の中に
/// ある凹頂点のうち最も射線に近いもの)Pと結ぶ。M,Pを複製して両リングを繋ぐ。
fn bridge_holes(points: &[Point2], outer: Vec<usize>, holes: &[Vec<usize>]) -> Vec<usize> {
    let mut ring = outer;
    // 右にある穴から順に繋ぐ(左の穴のブリッジが右の穴の射線を遮らないため)。
    let mut order: Vec<usize> = (0..holes.len()).collect();
    order.sort_by(|&i, &j| {
        let xi = holes[i]
            .iter()
            .map(|&k| points[k][0])
            .fold(f64::NEG_INFINITY, f64::max);
        let xj = holes[j]
            .iter()
            .map(|&k| points[k][0])
            .fold(f64::NEG_INFINITY, f64::max);
        xj.partial_cmp(&xi).unwrap()
    });

    for &hi in &order {
        let hole = &holes[hi];
        if hole.len() < 3 {
            continue;
        }
        // M: 穴の中で最も x が大きい頂点。
        let m_local = (0..hole.len())
            .max_by(|&i, &j| points[hole[i]][0].partial_cmp(&points[hole[j]][0]).unwrap())
            .unwrap();
        let m = hole[m_local];
        let mp = points[m];

        // 射線 (+x) が最初に当たる外形の辺と、その交点 x。
        let mut best_x = f64::INFINITY;
        let mut best_edge: Option<usize> = None;
        for i in 0..ring.len() {
            let a = points[ring[i]];
            let b = points[ring[(i + 1) % ring.len()]];
            // y が M.y を跨ぐ辺だけが射線と交わりうる。
            if (a[1] > mp[1]) == (b[1] > mp[1]) {
                continue;
            }
            let t = (mp[1] - a[1]) / (b[1] - a[1]);
            let x = a[0] + (b[0] - a[0]) * t;
            if x >= mp[0] - EPS && x < best_x {
                best_x = x;
                best_edge = Some(i);
            }
        }
        let Some(edge) = best_edge else {
            continue; // 外形に囲まれていない穴(異常入力)は無視する。
        };
        let i0 = ring[edge];
        let i1 = ring[(edge + 1) % ring.len()];
        // 候補Pは辺の端点のうち x が大きい方。
        let mut p_local = if points[i0][0] >= points[i1][0] {
            edge
        } else {
            (edge + 1) % ring.len()
        };
        let hit = [best_x, mp[1]];
        // 三角形 (M, hit, P) の中に凹頂点があるなら、そちらの方が「見える」。
        // 射線となす角が最小のものを選ぶ(同角なら近い方)。
        let mut best_angle = f64::INFINITY;
        for i in 0..ring.len() {
            let idx = ring[i];
            if idx == ring[p_local] {
                continue;
            }
            let p = points[idx];
            if !point_in_triangle(mp, hit, points[ring[p_local]], p) {
                continue;
            }
            if !is_reflex(points, &ring, i) {
                continue;
            }
            let dx = p[0] - mp[0];
            let dy = (p[1] - mp[1]).abs();
            let angle = dy.atan2(dx.max(0.0));
            if angle < best_angle {
                best_angle = angle;
                p_local = i;
            }
        }

        // 縫合: ring[..=p] + hole(Mから一周) + M + P + ring[p+1..]
        let mut merged: Vec<usize> = Vec::with_capacity(ring.len() + hole.len() + 2);
        merged.extend_from_slice(&ring[..=p_local]);
        for k in 0..hole.len() {
            merged.push(hole[(m_local + k) % hole.len()]);
        }
        merged.push(m);
        merged.push(ring[p_local]);
        merged.extend_from_slice(&ring[p_local + 1..]);
        ring = merged;
    }
    ring
}

/// リングの`i`番目の頂点が凹(reflex、CCWリングで右に曲がる)か。
fn is_reflex(points: &[Point2], ring: &[usize], i: usize) -> bool {
    let n = ring.len();
    let prev = points[ring[(i + n - 1) % n]];
    let cur = points[ring[i]];
    let next = points[ring[(i + 1) % n]];
    cross2(sub(cur, prev), sub(next, cur)) < 0.0
}

/// 点`p`が三角形`a,b,c`の内部または辺上にあるか。
fn point_in_triangle(a: Point2, b: Point2, c: Point2, p: Point2) -> bool {
    let d1 = cross2(sub(b, a), sub(p, a));
    let d2 = cross2(sub(c, b), sub(p, b));
    let d3 = cross2(sub(a, c), sub(p, c));
    let has_neg = d1 < -AREA_EPS || d2 < -AREA_EPS || d3 < -AREA_EPS;
    let has_pos = d1 > AREA_EPS || d2 > AREA_EPS || d3 > AREA_EPS;
    !(has_neg && has_pos)
}

/// 三角形の「形の良さ」(正三角形で最大、細長いほど0に近づく)。
/// $A/(a^2+b^2+c^2)$ ——正三角形で $1/(4\sqrt3)$、退化で0。
///
/// ## なぜ耳の選び方が品質に効くのか(実測に基づく)
///
/// 耳刈りは「見つけた順に刈る」だけでも**面積は必ず正しい**ので、当初は
/// 素朴に最初の耳を刈っていた。ところがそれだと凹頂点の周りで
/// **1点から全体へ張る扇状(fan)** の分割になり、L字断面では
/// 「断面を端から端まで横切る細長い三角形」が並ぶ。
///
/// これは押し出しの下流——`crate::decompose`の近似凸分解——を実際に壊した。
/// あちらは**三角形の重心の位置だけで三角形を丸ごと片側へ割り振る**
/// (`decompose`のモジュールdoc「意図的な簡略化」)ので、断面を横切る
/// 三角形は「重心は右側だが頂点は左端にも届いている」状態になり、
/// どちらへ割り振っても子パーツの凸包が元と同じ大きさになる。結果、
/// **どの分割候補も体積を減らさず、分解が一度も走らないまま凸包1個に
/// フォールバックしていた**(L字の押し出しで真の体積1.5 m³ に対し
/// 1.75 m³、U字で3.5 m³ に対し4.5 m³)。
///
/// 耳を「最も正三角形に近いもの」から刈るとコンパクトな三角形が並び、
/// 重心と広がりが一致するので分解が正しく効く(`from_triangle_mesh`まで
/// 通した回帰テストで固定してある)。計算量は $O(n^3)$ に上がるが、
/// スケッチの頂点数(数十)では体感できる差にならない。
fn triangle_quality(a: Point2, b: Point2, c: Point2) -> f64 {
    let area = cross2(sub(b, a), sub(c, a)) * 0.5;
    let ab = sub(b, a);
    let bc = sub(c, b);
    let ca = sub(a, c);
    let sum = dot2(ab, ab) + dot2(bc, bc) + dot2(ca, ca);
    if sum <= 0.0 {
        return 0.0;
    }
    area / sum
}

/// 単純多角形(CCWの index 列)の耳刈り三角形分割。
///
/// 刈る耳は**最も形の良いもの**から選ぶ(`triangle_quality`のdoc参照——
/// 順番に刈ると下流の凸分解が働かなくなる、実測に基づく設計)。
///
/// 耳が1周見つからない場合(ブリッジ辺で作った退化した並びなど)は、
/// 最も凸な頂点を強制的に刈って必ず前進する——「進めずに無限ループする」
/// より「わずかに歪んだ分割を返す」方が呼び出し側にとって遥かにましである。
fn ear_clip(points: &[Point2], ring: &mut Vec<usize>, out: &mut Vec<[usize; 3]>) {
    while ring.len() > 3 {
        let n = ring.len();
        let mut clipped = None;
        let mut best_quality = f64::NEG_INFINITY;
        for i in 0..n {
            let (ia, ib, ic) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
            if ia == ic {
                continue;
            }
            let (a, b, c) = (points[ia], points[ib], points[ic]);
            if cross2(sub(b, a), sub(c, b)) <= AREA_EPS {
                continue; // 凹 or 退化。
            }
            let quality = triangle_quality(a, b, c);
            if quality <= best_quality {
                continue; // 既に見つけた耳の方が形が良い。
            }
            let mut ok = true;
            for &other in ring.iter() {
                if other == ia || other == ib || other == ic {
                    continue;
                }
                let p = points[other];
                if near(p, a) || near(p, b) || near(p, c) {
                    continue;
                }
                if point_in_triangle(a, b, c, p) {
                    ok = false;
                    break;
                }
            }
            if ok {
                best_quality = quality;
                clipped = Some((i, [ia, ib, ic]));
            }
        }
        let clipped = clipped.inspect(|&(_, tri)| out.push(tri)).map(|(i, _)| i);
        match clipped {
            Some(i) => {
                ring.remove(i);
            }
            None => {
                // 前進保証のフォールバック。最も凸(外積最大)な頂点を刈る。
                let mut best = (f64::NEG_INFINITY, 0usize);
                for i in 0..n {
                    let (ia, ib, ic) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
                    if ia == ic {
                        continue;
                    }
                    let (a, b, c) = (points[ia], points[ib], points[ic]);
                    let v = cross2(sub(b, a), sub(c, b));
                    if v > best.0 {
                        best = (v, i);
                    }
                }
                let i = best.1;
                let (ia, ib, ic) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
                if ia != ic && best.0 > AREA_EPS {
                    out.push([ia, ib, ic]);
                }
                ring.remove(i);
            }
        }
    }
    if ring.len() == 3 {
        let (a, b, c) = (points[ring[0]], points[ring[1]], points[ring[2]]);
        if ring[0] != ring[1]
            && ring[1] != ring[2]
            && ring[0] != ring[2]
            && cross2(sub(b, a), sub(c, b)).abs() > AREA_EPS
        {
            out.push([ring[0], ring[1], ring[2]]);
        }
    }
}

// ---------------------------------------------------------------------------
// 押し出し
// ---------------------------------------------------------------------------

/// 細分後の頂点数の上限。これを超えるなら細分しない(下記`refine_region`の
/// 性能の保険。凸分解は $O(n^2)$ の凸包構築を候補ごとに回すため、
/// 頂点数が数百を超えると押し出しが体感できるほど遅くなる)。
const MAX_REFINED_VERTICES: usize = 512;

/// 領域の各辺を、**領域の全頂点が作る軸並行の格子線**で切り直す。
/// 幾何は1ミリも変わらない(元の辺の上に点を足すだけ)。
///
/// ## なぜ必要か(実測に基づく——これが無いと凹形状が凹まない)
///
/// 押し出したメッシュの行き先である`crate::decompose`は、
/// **軸並行平面で三角形を丸ごと片側へ割り振る**近似凸分解である
/// (`decompose`のモジュールdoc)。ところが素朴に押し出すと、断面の1本の
/// 長い辺がそのまま**断面の端から端まで届く1枚の側壁四角形**になる。
/// L字断面(0,0)-(2,0)-(2,1)-(1,1)-(1,2)-(0,2) の下辺 (0,0)→(2,0) がまさに
/// それで、この側壁の三角形は x=1 で割ろうとしても丸ごと片側へ行き、
/// その側の凸包は結局 x∈[0,2] 全体を覆ってしまう。**どの分割候補も体積を
/// 減らさない**ので`best_split`が`None`を返し、分解は一度も走らずに
/// 凸包1個へフォールバックしていた——真の体積 1.5 m³ に対し 1.75 m³、
/// U字では 3.5 m³ に対し 4.5 m³ で、しかも**当たり判定も凸包のまま**
/// (切り欠きを描いたのに切り欠きが無い剛体になる)。
///
/// 切り直す位置を「全頂点のx座標・z座標」に採るのは恣意的ではない——
/// 断面の凹凸が生まれる線がまさにそこであり、グリッドスナップされた
/// スケッチ(エディタの既定)ではこれが分解に必要十分な切り口になる。
/// 斜めの辺があっても、追加される点は高々「相異なる頂点座標の数」で抑えられる。
fn refine_region(loops: &[Loop2]) -> Vec<Loop2> {
    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    let mut total = 0;
    for l in loops {
        total += l.len();
        for p in l {
            xs.push(p[0]);
            ys.push(p[1]);
        }
    }
    let dedup = |v: &mut Vec<f64>| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.dedup_by(|a, b| (*a - *b).abs() <= EPS);
    };
    dedup(&mut xs);
    dedup(&mut ys);
    // 最悪でも「辺数 × (相異なるx + 相異なるy)」しか増えない。
    if total * (xs.len() + ys.len()) > MAX_REFINED_VERTICES {
        return loops.to_vec();
    }

    let mut out = Vec::with_capacity(loops.len());
    for l in loops {
        let n = l.len();
        let mut refined: Loop2 = Vec::with_capacity(n * 2);
        for i in 0..n {
            let a = l[i];
            let b = l[(i + 1) % n];
            refined.push(a);
            let d = sub(b, a);
            let mut ts: Vec<f64> = Vec::new();
            if d[0].abs() > EPS {
                for &x in &xs {
                    ts.push((x - a[0]) / d[0]);
                }
            }
            if d[1].abs() > EPS {
                for &y in &ys {
                    ts.push((y - a[1]) / d[1]);
                }
            }
            ts.retain(|t| *t > EPS && *t < 1.0 - EPS);
            ts.sort_by(|x, y| x.partial_cmp(y).unwrap());
            let mut prev = 0.0;
            for t in ts {
                if t - prev <= EPS {
                    continue;
                }
                prev = t;
                refined.push([a[0] + d[0] * t, a[1] + d[1] * t]);
            }
        }
        out.push(refined);
    }
    out
}

/// 押し出し1パーツぶんのメッシュ。三角形の巻き順はすべて**外向き**
/// (`crate::hull`/`crate::decompose`と同じ規約)。
#[derive(Clone, Debug)]
pub struct ExtrudedMesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[usize; 3]>,
}

/// 穴を開くための切断を試みる回数の上限(暴走防止)。実用上、穴は
/// 1〜2個なので2〜3回で足りる。
const MAX_HOLE_OPENING_CUTS: usize = 6;

/// 穴(CWループ)を持つ領域を、**穴を持たない部分領域の集合**へ切り分ける。
///
/// ## なぜ必要か(実測に基づく——これが無いと穴が塞がる)
///
/// `crate::decompose`は**軸並行平面による貪欲な2分割**で、「分割しても体積和が
/// 減らないならそこで止める」(`MIN_SPLIT_IMPROVEMENT`)。ところが板の**真ん中に
/// 開いた穴**は、どんな1枚の平面で2つに割っても**どちらの側の凸包も自分の側の
/// 穴を埋めてしまう**ので、1手目の改善が常にゼロになる。改善ゼロで再帰が
/// 打ち切られる結果、穴あき断面は分解されずに凸包1個へ落ち——**穴を描いたのに
/// 穴の無い(中身の詰まった)剛体になる**(実測: 外形63 m²・穴7.5 m²の板で、
/// 真の体積22.2 m³ に対し 25.2 m³、当たり判定も塞がったまま)。枠(frame)状の
/// 形は最低4パーツ必要で、貪欲な2分割の1手目では原理的に到達できない。
///
/// これは`decompose`の欠陥ではなく**入力の渡し方の問題**である——穴を通る線で
/// 先に切っておけば、各断片は「切り欠きのある単連結な形」になり、そこからは
/// 貪欲な分割がそのまま効く(同じ板で、切ってから渡すと左右とも
/// `Compound(3)`・体積は真の値と**完全一致**した)。切る位置は穴のx範囲の
/// 中央——穴を必ず横切り、かつ外形の形に依らない。
///
/// 切り分けた断片は`Shape::Compound`の子として**別々の`Shape`**になる
/// (1つのメッシュに繋いでしまうと、`decompose`から見た配置は元と変わらず
/// 同じ問題に戻る)。
fn split_into_hole_free_regions(loops: &[Loop2]) -> Vec<Vec<Loop2>> {
    let (mut min, mut max) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for l in loops {
        for p in l {
            for k in 0..2 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
    }
    if !min[0].is_finite() {
        return vec![loops.to_vec()];
    }
    let (lo, hi) = ([min[0] - 1.0, min[1] - 1.0], [max[0] + 1.0, max[1] + 1.0]);

    let mut pending = vec![loops.to_vec()];
    let mut done: Vec<Vec<Loop2>> = Vec::new();
    let mut cuts = 0;
    while let Some(region) = pending.pop() {
        let hole = region.iter().find(|l| loop_signed_area(l) < 0.0);
        let Some(hole) = hole else {
            done.push(region);
            continue;
        };
        if cuts >= MAX_HOLE_OPENING_CUTS {
            // これ以上は切らない(凸包近似へ落ちるが、必ず停止する)。
            done.push(region);
            continue;
        }
        cuts += 1;
        // 穴のx範囲の中央で縦に切る。
        let (hmin, hmax) = hole
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
                (acc.0.min(p[0]), acc.1.max(p[0]))
            });
        let cut = 0.5 * (hmin + hmax);
        let left = vec![vec![
            [lo[0], lo[1]],
            [cut, lo[1]],
            [cut, hi[1]],
            [lo[0], hi[1]],
        ]];
        let right = vec![vec![
            [cut, lo[1]],
            [hi[0], lo[1]],
            [hi[0], hi[1]],
            [cut, hi[1]],
        ]];
        let mut progressed = false;
        for half in [&left, &right] {
            let piece = polygon_boolean(&region, half, BooleanOp::Intersect);
            if piece.is_empty() {
                continue;
            }
            progressed = true;
            pending.push(piece);
        }
        if !progressed {
            done.push(region); // 切れなかった(異常入力)。そのまま渡す。
        }
    }
    done
}

/// 平面領域を厚み`depth`の角柱へ押し出す。
///
/// 平面上の`[u, v]`は3Dの`(x, z) = (u, v)`へ、押し出し方向は**+y**(上向き)で、
/// 出来上がりは`y ∈ [-depth/2, +depth/2]`に**中心を合わせて**返る。平面座標も
/// `region_centroid`ぶんだけ平行移動して原点に重心を置く——こうしておくと、
/// 剛体のローカル原点＝重心＝形状の中心になり、`(cx, depth/2, cz)`へ置けば
/// ちょうど地面に載る。
///
/// **返り値はパーツの列**。穴を持たない断面(大多数)では要素1つだが、
/// 穴があると`split_into_hole_free_regions`が切り分けた数だけ返る——
/// 呼び出し側はそれらを`Shape::Compound`の子(いずれも同じローカル原点、
/// 変換は恒等)として1つの剛体にする。
///
/// `depth`が正の有限値でない、または領域が退化している場合は`None`。
pub fn extrude_region(loops: &[Loop2], depth: f64) -> Option<Vec<ExtrudedMesh>> {
    if !(depth.is_finite() && depth > 0.0) {
        return None;
    }
    // **重心は切り分ける前の領域全体で求める**——全パーツが同じローカル原点を
    // 共有しないと、`Compound`の子として並べたときに位置がばらばらになる。
    let centroid = region_centroid(loops)?;
    let half = depth * 0.5;

    let mut parts = Vec::new();
    for region in split_into_hole_free_regions(loops) {
        // 下流の近似凸分解が効くように辺を切り直す(`refine_region`のdoc参照)。
        // 幾何は変わらないので、面積・体積・重心はどれも切り直す前と同じ。
        let region = refine_region(&region);
        let caps = triangulate_region(&region);
        if caps.is_empty() {
            continue;
        }
        let flat = flatten_region(&region);

        // 頂点は「平面上の点1つにつき下・上の2つ」。index 2k=下、2k+1=上。
        let mut vertices = Vec::with_capacity(flat.len() * 2);
        for p in &flat {
            let (x, z) = (p[0] - centroid[0], p[1] - centroid[1]);
            vertices.push(Vec3::new(x, -half, z));
            vertices.push(Vec3::new(x, half, z));
        }

        let mut triangles = Vec::with_capacity(caps.len() * 2 + flat.len() * 2);
        for &[a, b, c] in &caps {
            // 底面(y=-half、外向き法線 -y)。CCWの2D三角形をそのまま使うと -y を向く。
            triangles.push([a * 2, b * 2, c * 2]);
            // 天面(y=+half、外向き法線 +y)。巻き順を反転する。
            triangles.push([a * 2 + 1, c * 2 + 1, b * 2 + 1]);
        }

        // 側壁。ループを一周する各辺について、下→上の四角形を2枚の三角形にする。
        // 外形はCCW・穴はCWなので、同じ式でどちらも外向き(穴なら内側を向く=
        // 空洞から見て外向き)になる。
        let mut base = 0;
        for l in &region {
            let n = l.len();
            for i in 0..n {
                let ia = base + i;
                let ib = base + (i + 1) % n;
                let (bot_a, top_a) = (ia * 2, ia * 2 + 1);
                let (bot_b, top_b) = (ib * 2, ib * 2 + 1);
                triangles.push([bot_a, top_b, bot_b]);
                triangles.push([bot_a, top_a, top_b]);
            }
            base += n;
        }
        parts.push(ExtrudedMesh {
            vertices,
            triangles,
        });
    }
    if parts.is_empty() {
        return None;
    }
    Some(parts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Loop2 {
        vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    }

    /// 三角形分割が覆う総面積(符号付き)。
    fn triangulated_area(loops: &[Loop2]) -> f64 {
        let points = flatten_region(loops);
        triangulate_region(loops)
            .iter()
            .map(|&[a, b, c]| cross2(sub(points[b], points[a]), sub(points[c], points[a])) * 0.5)
            .sum()
    }

    /// 閉じたメッシュの符号付き体積(`decompose::mesh_volume`と同じ発散定理の
    /// 離散化——巻き順が全て外向きなら正になる)。
    fn mesh_volume(vertices: &[Vec3], triangles: &[[usize; 3]]) -> f64 {
        triangles
            .iter()
            .map(|&[i, j, k]| vertices[i].dot(vertices[j].cross(vertices[k])) / 6.0)
            .sum()
    }

    /// 全パーツの体積の和(穴の無い断面ではパーツは1つ)。
    fn extruded_volume(parts: &[ExtrudedMesh]) -> f64 {
        parts
            .iter()
            .map(|p| mesh_volume(&p.vertices, &p.triangles))
            .sum()
    }

    #[test]
    fn normalize_loop_orients_clockwise_input_counterclockwise() {
        let cw = vec![[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]];
        let normalized = normalize_loop(&cw).expect("面積を持つ四角形");
        assert!(
            loop_signed_area(&normalized) > 0.0,
            "時計回り入力はCCWへ揃えられる"
        );
        assert!((loop_signed_area(&normalized) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn normalize_loop_rejects_degenerate_input() {
        assert!(
            normalize_loop(&[[0.0, 0.0], [1.0, 0.0]]).is_none(),
            "2点は多角形でない"
        );
        assert!(
            normalize_loop(&[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]).is_none(),
            "共線の3点は面積0"
        );
        assert!(
            normalize_loop(&[[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]).is_some(),
            "重複点は落として三角形になる"
        );
    }

    /// **和**: 2辺2mの正方形2つを1m² ぶん重ねる。
    /// 面積は 4 + 4 − 1 = 7 m²、外形は1本の凹8角形。
    #[test]
    fn union_of_two_overlapping_squares_has_expected_area() {
        let a = vec![rect(0.0, 0.0, 2.0, 2.0)];
        let b = vec![rect(1.0, 1.0, 3.0, 3.0)];
        let result = polygon_boolean(&a, &b, BooleanOp::Union);
        assert_eq!(result.len(), 1, "和は1本の外形になる");
        assert!(
            (region_area(&result) - 7.0).abs() < 1e-9,
            "4+4-1=7 m²(実際: {})",
            region_area(&result)
        );
        assert_eq!(result[0].len(), 8, "L字を回り込む凹8角形");
    }

    /// **積**: 同じ2つの正方形の重なりはちょうど 1m² の正方形。
    #[test]
    fn intersection_of_two_overlapping_squares_is_the_overlap_square() {
        let a = vec![rect(0.0, 0.0, 2.0, 2.0)];
        let b = vec![rect(1.0, 1.0, 3.0, 3.0)];
        let result = polygon_boolean(&a, &b, BooleanOp::Intersect);
        assert_eq!(result.len(), 1);
        assert!((region_area(&result) - 1.0).abs() < 1e-9);
        assert_eq!(result[0].len(), 4, "重なりも正方形");
    }

    /// **差**: 大きい方から小さい方を引くと 4 − 1 = 3 m² のL字。
    #[test]
    fn difference_of_two_overlapping_squares_is_an_l_shape() {
        let a = vec![rect(0.0, 0.0, 2.0, 2.0)];
        let b = vec![rect(1.0, 1.0, 3.0, 3.0)];
        let result = polygon_boolean(&a, &b, BooleanOp::Subtract);
        assert_eq!(result.len(), 1);
        assert!(
            (region_area(&result) - 3.0).abs() < 1e-9,
            "4-1=3 m²(実際: {})",
            region_area(&result)
        );
        assert_eq!(result[0].len(), 6, "切り欠きのあるL字は6角形");
    }

    /// **差が穴を作る**場合。大きい正方形の内部にすっぽり入る小さい正方形を
    /// 引くと、外形(CCW)+穴(CW)の2ループになる。
    #[test]
    fn difference_with_a_fully_contained_hole_yields_two_loops() {
        let a = vec![rect(0.0, 0.0, 4.0, 4.0)];
        let b = vec![rect(1.0, 1.0, 2.0, 2.0)];
        let result = polygon_boolean(&a, &b, BooleanOp::Subtract);
        assert_eq!(result.len(), 2, "外形+穴");
        let outer = result.iter().find(|l| loop_signed_area(l) > 0.0).unwrap();
        let hole = result.iter().find(|l| loop_signed_area(l) < 0.0).unwrap();
        assert!((loop_signed_area(outer) - 16.0).abs() < 1e-9);
        assert!(
            (loop_signed_area(hole) + 1.0).abs() < 1e-9,
            "穴はCWなので -1 m²"
        );
        assert!(
            (region_area(&result) - 15.0).abs() < 1e-9,
            "16-1=15 m²(実際: {})",
            region_area(&result)
        );
    }

    /// 交わらない配置も特別扱い無しで正しく落ちる。
    #[test]
    fn disjoint_squares_behave_per_operation() {
        let a = vec![rect(0.0, 0.0, 1.0, 1.0)];
        let b = vec![rect(5.0, 5.0, 6.0, 6.0)];
        let union = polygon_boolean(&a, &b, BooleanOp::Union);
        assert_eq!(union.len(), 2, "離れていれば和は2つの島");
        assert!((region_area(&union) - 2.0).abs() < 1e-9);
        assert!(
            polygon_boolean(&a, &b, BooleanOp::Intersect).is_empty(),
            "積は空"
        );
        let diff = polygon_boolean(&a, &b, BooleanOp::Subtract);
        assert!((region_area(&diff) - 1.0).abs() < 1e-9, "差はAそのもの");
    }

    /// 辺をぴったり共有する2つの正方形(グリッドスナップされたスケッチで
    /// 実際に起きる配置)の和は、面積2m²の1本の長方形になる——共有辺は
    /// 逆向きの重なりなので両方とも捨てられる。
    #[test]
    fn union_of_edge_sharing_squares_merges_into_one_rectangle() {
        let a = vec![rect(0.0, 0.0, 1.0, 1.0)];
        let b = vec![rect(1.0, 0.0, 2.0, 1.0)];
        let result = polygon_boolean(&a, &b, BooleanOp::Union);
        assert_eq!(result.len(), 1, "接している2枚は1本の外形へ融合する");
        assert!(
            (region_area(&result) - 2.0).abs() < 1e-9,
            "1+1=2 m²(実際: {})",
            region_area(&result)
        );
    }

    /// 凸多角形(正方形)の耳刈りは n-2 = 2 枚。
    #[test]
    fn ear_clipping_a_square_yields_two_triangles() {
        let square = vec![rect(0.0, 0.0, 1.0, 1.0)];
        let tris = triangulate_region(&square);
        assert_eq!(tris.len(), 2);
        assert!((triangulated_area(&square) - 1.0).abs() < 1e-12);
    }

    /// **L字(凹多角形)の耳刈り**。6頂点なので 6-2 = 4 枚、
    /// 総面積は元のL字の面積(3 m²)と一致する。
    #[test]
    fn ear_clipping_an_l_shape_covers_the_original_area() {
        let l_shape = vec![vec![
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
        ]];
        assert!((loop_signed_area(&l_shape[0]) - 3.0).abs() < 1e-12);
        let tris = triangulate_region(&l_shape);
        assert_eq!(tris.len(), 4, "n-2 = 4 枚");
        assert!(
            (triangulated_area(&l_shape) - 3.0).abs() < 1e-12,
            "三角形の総面積が元の面積と一致する(実際: {})",
            triangulated_area(&l_shape)
        );
    }

    /// **穴あき多角形の耳刈り**(ブリッジ辺の経路)。外形4+穴4に
    /// ブリッジで2頂点ぶん増えて 10 → 8 枚、面積は 16−1 = 15 m²。
    #[test]
    fn ear_clipping_a_square_with_a_hole_covers_outer_minus_hole() {
        let region = polygon_boolean(
            &[rect(0.0, 0.0, 4.0, 4.0)],
            &[rect(1.0, 1.0, 2.0, 2.0)],
            BooleanOp::Subtract,
        );
        let tris = triangulate_region(&region);
        assert_eq!(tris.len(), 8, "外形4+穴4+ブリッジ2 = 10頂点 → 8枚");
        assert!(
            (triangulated_area(&region) - 15.0).abs() < 1e-9,
            "16-1=15 m²(実際: {})",
            triangulated_area(&region)
        );
    }

    /// 押し出した角柱の体積が「断面積 × 深さ」に一致し、巻き順が全面
    /// 外向き(=符号付き体積が正)であること。
    #[test]
    fn extruding_a_square_gives_a_box_of_the_expected_volume() {
        let square = vec![rect(-0.5, -0.5, 0.5, 0.5)];
        let parts = extrude_region(&square, 0.4).expect("押し出せる");
        assert_eq!(parts.len(), 1, "穴が無いのでパーツは1つ");
        assert_eq!(parts[0].vertices.len(), 8, "4頂点 × 上下");
        let volume = extruded_volume(&parts);
        assert!(
            (volume - 0.4).abs() < 1e-12,
            "1.0 m² × 0.4 m = 0.4 m³(実際: {volume})"
        );
    }

    /// 非凸(L字)を押し出しても体積が断面積×深さに一致する
    /// ——凸包で近似していたら 4 m² × 深さ になってしまう。
    #[test]
    fn extruding_an_l_shape_keeps_the_concave_volume() {
        let l_shape = vec![vec![
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
        ]];
        let parts = extrude_region(&l_shape, 0.5).expect("押し出せる");
        assert_eq!(parts.len(), 1, "L字に穴は無いのでパーツは1つ");
        let volume = extruded_volume(&parts);
        assert!(
            (volume - 1.5).abs() < 1e-9,
            "3 m² × 0.5 m = 1.5 m³(凸包なら 2.0 m³ になる。実際: {volume})"
        );
    }

    /// 穴あき断面を押し出すと、体積から穴の分がきちんと抜ける。
    #[test]
    fn extruding_a_holed_profile_subtracts_the_hole_volume() {
        let region = polygon_boolean(
            &[rect(0.0, 0.0, 4.0, 4.0)],
            &[rect(1.0, 1.0, 2.0, 2.0)],
            BooleanOp::Subtract,
        );
        let parts = extrude_region(&region, 0.25).expect("押し出せる");
        assert!(
            parts.len() >= 2,
            "穴があるので穴を通る線で切り分けられる(`split_into_hole_free_regions`)"
        );
        let volume = extruded_volume(&parts);
        assert!(
            (volume - 15.0 * 0.25).abs() < 1e-9,
            "(16-1) m² × 0.25 m = 3.75 m³(実際: {volume})"
        );
    }

    /// 重心が原点へ来ていること(押し出しの平行移動の規約)。
    #[test]
    fn extruded_mesh_is_centered_on_its_centroid() {
        let square = vec![rect(10.0, -4.0, 12.0, -2.0)];
        let parts = extrude_region(&square, 1.0).expect("押し出せる");
        let vertices = &parts[0].vertices;
        let n = vertices.len() as f64;
        let mean = vertices
            .iter()
            .fold(Vec3::ZERO, |acc, v| acc + *v)
            .scale(1.0 / n);
        assert!(mean.x.abs() < 1e-12 && mean.y.abs() < 1e-12 && mean.z.abs() < 1e-12);
        assert!(
            vertices.iter().all(|v| (v.y.abs() - 0.5).abs() < 1e-12),
            "深さ1.0mなら y=±0.5m"
        );
    }

    /// **回帰テスト(統合)**: 押し出したメッシュを`Shape::from_triangle_mesh`
    /// に通すと、実際に`Compound`へ**分解され**、体積が真の値に一致すること。
    ///
    /// `refine_region`と`triangle_quality`のdocが説明する2つの実バグの再発防止:
    /// どちらか一方でも欠けると`decompose`の分割候補が体積を減らせず、
    /// **分解が一度も走らないまま凸包1個**(L字で1.75 m³、U字で4.5 m³)に
    /// フォールバックしていた。凸包扱いになると質量が過大になるだけでなく、
    /// **当たり判定まで凸包**になる——切り欠きを描いたのに切り欠きの無い
    /// 剛体ができてしまい、この機能の存在意義が消える。
    #[test]
    fn extruded_concave_profiles_actually_decompose_into_convex_parts() {
        for (name, region, true_area) in [
            (
                "L字",
                vec![vec![
                    [0.0, 0.0],
                    [2.0, 0.0],
                    [2.0, 1.0],
                    [1.0, 1.0],
                    [1.0, 2.0],
                    [0.0, 2.0],
                ]],
                3.0,
            ),
            (
                "コの字",
                vec![vec![
                    [0.0, 0.0],
                    [3.0, 0.0],
                    [3.0, 3.0],
                    [2.0, 3.0],
                    [2.0, 1.0],
                    [1.0, 1.0],
                    [1.0, 3.0],
                    [0.0, 3.0],
                ]],
                7.0,
            ),
        ] {
            let depth = 0.5;
            let parts = extrude_region(&region, depth).expect("押し出せる");
            assert_eq!(parts.len(), 1, "{name}に穴は無い");
            let shape = crate::Shape::from_triangle_mesh(
                parts[0].vertices.clone(),
                parts[0].triangles.clone(),
            );
            assert!(
                matches!(shape, crate::Shape::Compound { .. }),
                "{name}は凸パーツへ分解されるはず(ConvexMesh のままなら凸包扱い): {shape:?}"
            );
            let volume = shape.volume().expect("体積を持つ");
            assert!(
                (volume - true_area * depth).abs() < 1e-6,
                "{name}の体積は 断面積{true_area} m² × {depth} m のはず(実際: {volume})"
            );
        }
    }

    /// **回帰テスト(統合)**: 穴あき断面が、切り分けた各パーツで**正しく凸分解
    /// される**こと(`split_into_hole_free_regions`のdoc参照)。
    ///
    /// 切り分けずに1つのメッシュとして渡すと、`decompose`の貪欲な2分割は
    /// 1手目で改善ゼロになって止まり、**穴の塞がった凸包1個**になる
    /// (実測: 真の体積22.2 m³ に対し25.2 m³、当たり判定も塞がったまま)。
    /// ここでは板の中央に穴を開けた断面(外形63 m²・穴7.5 m²、押し出し0.4m)で、
    /// 全パーツの体積和が真の値に一致することを固定する。
    #[test]
    fn extruded_holed_profiles_decompose_without_filling_the_hole() {
        let outer = vec![vec![[-6.0, -15.0], [6.0, -15.0], [3.0, -8.0], [-3.0, -8.0]]];
        let inner = vec![vec![[-2.0, -12.0], [2.0, -12.0], [1.0, -9.5], [-1.0, -9.5]]];
        let region = polygon_boolean(&outer, &inner, BooleanOp::Subtract);
        assert_eq!(region.len(), 2, "外形+穴");
        let true_area = region_area(&region);
        assert!((true_area - 55.5).abs() < 1e-9, "63 - 7.5 = 55.5 m²");

        let depth = 0.4;
        let parts = extrude_region(&region, depth).expect("押し出せる");
        assert!(parts.len() >= 2, "穴を通る線で切り分けられている");

        let mut total = 0.0;
        for part in &parts {
            let shape =
                crate::Shape::from_triangle_mesh(part.vertices.clone(), part.triangles.clone());
            assert!(
                matches!(shape, crate::Shape::Compound { .. }),
                "切り欠きのある各パーツは凸分解されるはず: {shape:?}"
            );
            total += shape.volume().expect("体積を持つ");
        }
        assert!(
            (total - true_area * depth).abs() < 1e-6,
            "全パーツの体積和は 55.5 m² × 0.4 m = 22.2 m³ のはず\
             (穴が塞がると 25.2 m³。実際: {total})"
        );
    }

    /// 辺の切り直し(`refine_region`)は**幾何を変えない**——面積も重心も
    /// 押し出した体積も、切り直す前と1ビットの意味で同じである。
    #[test]
    fn refining_edges_does_not_change_the_geometry() {
        let l_shape = vec![vec![
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
        ]];
        let refined = refine_region(&l_shape);
        assert!(
            refined[0].len() > l_shape[0].len(),
            "L字は格子線で切り直されて頂点が増える"
        );
        assert!((region_area(&refined) - region_area(&l_shape)).abs() < 1e-12);
        let (c0, c1) = (
            region_centroid(&l_shape).unwrap(),
            region_centroid(&refined).unwrap(),
        );
        assert!((c0[0] - c1[0]).abs() < 1e-12 && (c0[1] - c1[1]).abs() < 1e-12);
    }

    /// 押し出せない入力は`None`(呼び出し側がユーザーへエラーを返せる)。
    #[test]
    fn extrude_rejects_degenerate_input() {
        let square = vec![rect(0.0, 0.0, 1.0, 1.0)];
        assert!(extrude_region(&square, 0.0).is_none(), "深さ0");
        assert!(extrude_region(&square, f64::NAN).is_none(), "非有限の深さ");
        assert!(extrude_region(&[], 1.0).is_none(), "領域が空");
    }
}
