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
use sim_fluid::{GridFluid2D, SphFluid};
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
    /// SPH流体(設計 docs/11-fluid/03-sph.md、`SphRigid`が使う)。
    pub sph: Option<&'a mut SphFluid>,
}

/// ドメイン間結合(設計 docs/00-foundation/04-architecture.md §1.3「保存量の橋」)。
/// 2つ(以上)のソルバの状態を読み、互いに作用を書き込む。取り出した量と注入した量が
/// 一致することを実装側がデバッグビルドで検算する(設計の要求、§1.1.2(2))。
pub trait Coupling: CouplingClone {
    /// 依存するソルバ(実行順序の決定に使う、設計§1.3)。
    fn domains(&self) -> (DomainId, DomainId);

    /// 結合の適用。**pre/post を区別しない実装のための既定の入口**であり、
    /// `apply_pre`/`apply_post`のどちらもオーバーライドしなければ post 相で呼ばれる。
    fn apply(&mut self, world: &mut DomainStates, dt: f64);

    /// **pre 相**(設計 docs/20-integration/01-coupling-matrix.md §1.3
    /// 「pre/post の2相分離」、増分Jで追加)——**ドメインソルバを進める前**に
    /// 呼ばれる。今stepの積分に効かせたい力・熱・電流の**注入**はここに置く。
    ///
    /// **なぜ2相が要るのか**: 単一の`apply`しか無いと、注入型の結合は必ず
    /// 「ソルバが進んだ後」に効くため**常に1step遅れる**。実際
    /// `InductionCoupling`・`MotorCoupling`は「1step遅れの縮約」としてその遅れを
    /// 明示的に受け入れており、`PistonGas`も同様である。pre 相があれば、
    /// 注入を今stepに間に合わせる実装が書けるようになる。
    ///
    /// 既定は**何もしない**。既存の全実装は`apply`だけを持つので、この既定に
    /// よって**挙動は一切変わらない**(2相分離は機構として用意され、各結合が
    /// 必要に応じて移行する)。
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
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Mechanics)
    }
    fn apply(&mut self, _world: &mut DomainStates, _dt: f64) {}
}
