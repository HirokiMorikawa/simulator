//! `Coupling`トレイトと`DomainStates`(設計 docs/00-foundation/04-architecture.md §1.3)。
//!
//! **縮約実装の理由**: 設計は`DomainStates`を「全ドメインへの可変ビュー」として抽象的に
//! 示すのみで具体的な型は規定していない。本crateでは、`World`(`sim-world`)が実際に
//! 保持しているドメイン集合(mechanics・thermal・em・astro、ワークストリームBの増分で
//! `sim-world::World`に追加済み)のうち、実装済みのCouplingが必要とする組み合わせだけを
//! 持つ具体的な構造体として定義する(汎用的な型消去レジストリではない)。
//! `DissipationToHeat`はmechanics + thermal、`JouleHeat`はem_circuit + thermal、
//! `LorentzForce`はem_electrostatics + mechanics、`PistonGas`はmechanics + gas、
//! `BoussinesqBuoyancy`はthermal + grid_fluid、`SphRigid`はmechanics + sphを使う。
//! 他のCouplingが必要とする組み合わせは、そのCouplingを実装する増分で`DomainStates`に
//! フィールドを追加する。

use sim_core::DomainId;
use sim_em::{Circuit, PointChargeSystem};
use sim_fluid::{GridFluid2D, GridFluid3D, SphFluid};
use sim_mechanics::MechanicsSolver;
use sim_thermal::{GasCompartment, ThermalSolver};

/// Couplingが読み書きできる各ドメインの可変ビュー(モジュールdoc参照、現時点では
/// mechanics・thermal・em_circuit・em_electrostatics・gas・grid_fluid・sphのみ)。
pub struct DomainStates<'a> {
    pub mechanics: &'a mut MechanicsSolver,
    pub thermal: Option<&'a mut ThermalSolver>,
    pub em_circuit: Option<&'a mut Circuit>,
    pub em_electrostatics: Option<&'a mut PointChargeSystem>,
    /// 気体区画(設計 docs/12-thermal/01-thermodynamics-laws.md §3、`PistonGas`が使う)。
    pub gas: Option<&'a mut GasCompartment>,
    /// 格子流体(設計 docs/11-fluid/02-eulerian-grid.md §4.2、`BoussinesqBuoyancy`が使う)。
    pub grid_fluid: Option<&'a mut GridFluid2D>,
    /// 3D格子流体(**群9で追加**、`sim_fluid::GridFluid3D`)。2Dと同格のドメインとして
    /// 結合から到達できるようにしてある。**現時点でこれを使う結合はまだ無い**
    /// ——`GridFluidRigid`等は2D専用で、3D版の結合は設計の結合行列にも無い。
    /// ドメインを一級市民として置くための枠であることを正直に記録しておく。
    pub grid_fluid_3d: Option<&'a mut GridFluid3D>,
    /// SPH流体(設計 docs/11-fluid/03-sph.md、`SphRigid`が使う)。
    pub sph: Option<&'a mut SphFluid>,
}

/// 結合の種別(**内省層、群1で追加**)。
///
/// **なぜ必要だったか**: `Coupling`トレイトは長らく`domains()`と`apply()`しか持たず、
/// **実装が自分の種別を名乗る手段が無かった**。そのため`World::coupling_count()`は
/// 件数しか返せず、InspectorのCouplingコンポーネントは
/// 「種別: —(トレイトが名前を持たないため非表示)」という縮退表示だった。
/// 設計 docs/23-frontend/01-editor.md §1.3 は Coupling コンポーネントに
/// 「**種別**・関連する Body/Fluid/Circuit 参照」を出すことを要求しており、
/// これはその要求を満たすための型である。
///
/// **なぜ`&'static str`ではなく enum なのか**: 文字列だと実装側が任意の名前を
/// 書けてしまい、UI側が種別で分岐・フィルタできない。enumならコンパイラが
/// 網羅性を保証し、新しい結合を足したときにUI側の対応漏れが検出できる。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CouplingKind {
    BoussinesqBuoyancy,
    BrownianForce,
    BuoyancyDrag,
    ConvectionLink,
    DissipationToHeat,
    GridFluidRigid,
    ImageChargeForce,
    InductionCoupling,
    JouleHeat,
    LorentzForce,
    MotorCoupling,
    PhaseChangeMorph,
    PistonGas,
    SphRigid,
    /// `World`が pre 相で結合を一時的に取り出すときのプレースホルダ。
    Noop,
}

impl CouplingKind {
    /// 型名そのもの(UIの見出し・シーンJSONのタグと一致させる)。
    pub fn name(self) -> &'static str {
        match self {
            CouplingKind::BoussinesqBuoyancy => "BoussinesqBuoyancy",
            CouplingKind::BrownianForce => "BrownianForce",
            CouplingKind::BuoyancyDrag => "BuoyancyDrag",
            CouplingKind::ConvectionLink => "ConvectionLink",
            CouplingKind::DissipationToHeat => "DissipationToHeat",
            CouplingKind::GridFluidRigid => "GridFluidRigid",
            CouplingKind::ImageChargeForce => "ImageChargeForce",
            CouplingKind::InductionCoupling => "InductionCoupling",
            CouplingKind::JouleHeat => "JouleHeat",
            CouplingKind::LorentzForce => "LorentzForce",
            CouplingKind::MotorCoupling => "MotorCoupling",
            CouplingKind::PhaseChangeMorph => "PhaseChangeMorph",
            CouplingKind::PistonGas => "PistonGas",
            CouplingKind::SphRigid => "SphRigid",
            CouplingKind::Noop => "Noop",
        }
    }

    /// 何をする結合かの1行説明(Inspectorのツールチップ用)。
    pub fn summary(self) -> &'static str {
        match self {
            CouplingKind::BoussinesqBuoyancy => "熱ノードの温度差から格子流体へ浮力を与える",
            CouplingKind::BrownianForce => "温度と粘性から微小剛体へランダム力を与える",
            CouplingKind::BuoyancyDrag => "静的水域の浮力と大気の抗力を剛体へ与える",
            CouplingKind::ConvectionLink => "流体ノードと表面ノードを対流相関式で繋ぐ",
            CouplingKind::DissipationToHeat => "力学の散逸(接触・摩擦)を熱ノードへ移す",
            CouplingKind::GridFluidRigid => "格子流体と剛体を双方向に結合する",
            CouplingKind::ImageChargeForce => "接地平面に対する鏡像電荷の引力を与える",
            CouplingKind::InductionCoupling => "導体棒の渦電流ブレーキ(レンツ則)",
            CouplingKind::JouleHeat => "回路の抵抗損失 I^2R を熱ノードへ移す",
            CouplingKind::LorentzForce => "静電場から帯電剛体へローレンツ力を与える",
            CouplingKind::MotorCoupling => "回路とヒンジ回転を双方向に結合する(モーター/発電機)",
            CouplingKind::PhaseChangeMorph => "融解に応じて剛体の質量を減らす(相変化)",
            CouplingKind::PistonGas => "気体区画とピストン剛体を結合する",
            CouplingKind::SphRigid => "SPH流体と剛体を境界粒子経由で双方向に結合する",
            CouplingKind::Noop => "何もしない(内部用プレースホルダ)",
        }
    }
}

/// ドメイン間結合(設計 docs/00-foundation/04-architecture.md §1.3「保存量の橋」)。
/// 2つ(以上)のソルバの状態を読み、互いに作用を書き込む。取り出した量と注入した量が
/// 一致することを実装側がデバッグビルドで検算する(設計の要求、§1.1.2(2))。
pub trait Coupling: CouplingClone + AsAnyCoupling {
    /// この結合の種別(**内省層、群1で追加**。`CouplingKind`のdoc参照)。
    fn kind(&self) -> CouplingKind;

    /// 依存するソルバ(設計§1.3)。
    ///
    /// **群1で `(DomainId, DomainId)` の2-tuple から slice へ一般化した**。
    /// 2-tuple固定では**3ドメイン以上に跨る結合を宣言できない**。
    /// なお変更時点でこのメソッドの呼び出し元は**1つも無かった**——
    /// 「実行順序の決定に使う」と宣言しながら誰も使っていない死んだAPIだったので、
    /// シグネチャ変更による実挙動の変化は無い。内省層で実際に使い始める。
    fn domain_ids(&self) -> &'static [DomainId];

    /// パラメータ込みの人間可読表現(InspectorのComponent行に出す)。
    fn describe(&self) -> String;

    /// この結合が読み書きする剛体のindex(**Inspectorが選択中ボディで絞るため**)。
    /// 剛体に触れない結合(`JouleHeat`・`ConvectionLink`など)は空を返す。
    fn referenced_bodies(&self) -> Vec<usize> {
        Vec::new()
    }

    /// この結合が読み書きする熱ノードのindex(同上)。既定は空。
    fn referenced_thermal_nodes(&self) -> Vec<usize> {
        Vec::new()
    }

    /// この結合が読み書きする回路の電圧源index(同上)。既定は空。
    fn referenced_voltage_sources(&self) -> Vec<usize> {
        Vec::new()
    }

    /// 結合の適用。**pre/post を区別しない実装のための既定の入口**であり、
    /// `apply_pre`/`apply_post`のどちらもオーバーライドしなければ post 相で呼ばれる。
    fn apply(&mut self, world: &mut DomainStates, dt: f64);

    /// **pre 相**(設計 docs/20-integration/01-coupling-matrix.md §1.3
    /// 「pre/post の2相分離」、増分Jで追加)——**ドメインソルバを進める前**に
    /// 呼ばれる。今stepの積分に効かせたい力・熱・電流の**注入**はここに置く。
    ///
    /// **なぜ2相が要るのか**: 単一の`apply`しか無いと、注入型の結合は必ず
    /// 「ソルバが進んだ後」に効くため**常に1step遅れる**。
    ///
    /// **群5で移行済みの結合**: `InductionCoupling`・`MotorCoupling`(起電力を
    /// 今stepの回路solveへ間に合わせる)、`BuoyancyDrag`・`LorentzForce`・
    /// `ImageChargeForce`・`BrownianForce`(注入した速度を今stepの位置積分へ効かせる)。
    ///
    /// **post のままが正しい結合**: `DissipationToHeat`・`JouleHeat`・
    /// `ConvectionLink`・`BoussinesqBuoyancy`・`SphRigid`・`GridFluidRigid`・
    /// `PistonGas` — いずれも今stepのソルバが確定させた量(散逸熱・抵抗損失・温度・
    /// 圧力場・境界力)を読むか、次stepの積分がそのまま消費する累算器
    /// (`force_accum`)へ書くため、post 相で遅れが生じない。
    ///
    /// 既定は**何もしない**。`apply`だけを持つ実装はこの既定によって従来どおり
    /// post 相でのみ呼ばれる(挙動は変わらない)。**`apply_pre`を上書きする実装は
    /// `apply_post`も必ず上書きすること** — `apply_post`の既定は`apply`へ委譲する
    /// ので、上書きを忘れると同じ処理が1stepに2回走る。
    fn apply_pre(&mut self, _world: &mut DomainStates, _dt: f64) {}

    /// **post 相**——ドメインソルバを進めた**後**に呼ばれる。前stepではなく
    /// **今stepで確定した量を読む**結合(圧力積分の反作用、抵抗損失の集計など)は
    /// ここに置く。
    ///
    /// 既定は`apply`へ委譲する。したがって既存実装は従来どおり post 相で
    /// 1回だけ呼ばれ、**挙動は変わらない**。
    fn apply_post(&mut self, world: &mut DomainStates, dt: f64) {
        self.apply(world, dt);
    }
}

/// `Box<dyn Coupling>`をクローン可能にするdyn-safeなヘルパー(`sim_world::World`が
/// Couplingレジストリを保持しつつ`#[derive(Clone)]`(`snapshot`/`restore`が使う)を
/// 導出できるようにするため)。`T: Coupling + Clone`への下のblanket implにより、
/// 各Coupling実装は通常どおり`#[derive(Clone)]`を付けるだけでよい。
pub trait CouplingClone {
    fn clone_box(&self) -> Box<dyn Coupling>;
}

impl<T> CouplingClone for T
where
    T: 'static + Coupling + Clone,
{
    fn clone_box(&self) -> Box<dyn Coupling> {
        Box::new(self.clone())
    }
}

/// 具象型へのdowncastを`dyn Coupling`越しに可能にする(`CouplingClone`と同じ
/// blanket implパターン)。`World → Scenario`逆写像がパラメータ込みで各`Coupling`を
/// 読み戻すために使う——`describe()`は人間可読の文字列であって構造化データでは
/// ないため、これが無いと「保存すると結合のパラメータが消える」ことになる。
pub trait AsAnyCoupling {
    fn as_any(&self) -> &dyn std::any::Any;
}

impl<T> AsAnyCoupling for T
where
    T: 'static + Coupling,
{
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

impl Clone for Box<dyn Coupling> {
    fn clone(&self) -> Box<dyn Coupling> {
        self.clone_box()
    }
}

/// 何もしない`Coupling`(**増分Jで追加**)。`World`が pre 相で各結合を一時的に
/// レジストリから取り出す(`std::mem::replace`)ときのプレースホルダに使う——
/// `&mut self.couplings[i]`を保持したまま`DomainStates`が`&mut self.mechanics`等を
/// 借りることはできないため、所有権を一度取り出す必要がある。
#[derive(Clone)]
pub struct NoopCoupling;

impl Coupling for NoopCoupling {
    fn kind(&self) -> CouplingKind {
        CouplingKind::Noop
    }
    fn domain_ids(&self) -> &'static [DomainId] {
        &[]
    }
    fn describe(&self) -> String {
        "Noop".to_string()
    }
    fn apply(&mut self, _world: &mut DomainStates, _dt: f64) {}
}
