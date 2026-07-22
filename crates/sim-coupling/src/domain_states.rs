//! `Coupling`トレイトと`DomainStates`(設計 docs/00-foundation/04-architecture.md §1.3)。
//!
//! **縮約実装の理由**: 設計は`DomainStates`を「全ドメインへの可変ビュー」として抽象的に
//! 示すのみで具体的な型は規定していない。本crateでは、`World`(`sim-world`)が実際に
//! 保持しているドメイン集合(mechanics・thermal・em・astro、ワークストリームBの増分で
//! `sim-world::World`に追加済み)のうち、実装済みのCouplingが必要とする組み合わせだけを
//! 持つ具体的な構造体として定義する(汎用的な型消去レジストリではない)。
//! `DissipationToHeat`はmechanics + thermal、`JouleHeat`はem_circuit + thermal、
//! `LorentzForce`はem_electrostatics + mechanics、`PistonGas`はmechanics + gasを使う。
//! 他のCouplingが必要とする組み合わせは、そのCouplingを実装する増分で`DomainStates`に
//! フィールドを追加する。

use sim_core::DomainId;
use sim_em::{Circuit, PointChargeSystem};
use sim_mechanics::MechanicsSolver;
use sim_thermal::{GasCompartment, ThermalSolver};

/// Couplingが読み書きできる各ドメインの可変ビュー(モジュールdoc参照、現時点では
/// mechanics・thermal・em_circuit・em_electrostatics・gasのみ)。
pub struct DomainStates<'a> {
    pub mechanics: &'a mut MechanicsSolver,
    pub thermal: Option<&'a mut ThermalSolver>,
    pub em_circuit: Option<&'a mut Circuit>,
    pub em_electrostatics: Option<&'a mut PointChargeSystem>,
    /// 気体区画(設計 docs/12-thermal/01-thermodynamics-laws.md §3、`PistonGas`が使う)。
    pub gas: Option<&'a mut GasCompartment>,
}

/// ドメイン間結合(設計 docs/00-foundation/04-architecture.md §1.3「保存量の橋」)。
/// 2つ(以上)のソルバの状態を読み、互いに作用を書き込む。取り出した量と注入した量が
/// 一致することを実装側がデバッグビルドで検算する(設計の要求、§1.1.2(2))。
pub trait Coupling: CouplingClone {
    /// 依存するソルバ(実行順序の決定に使う、設計§1.3)。
    fn domains(&self) -> (DomainId, DomainId);

    /// 結合の適用。
    fn apply(&mut self, world: &mut DomainStates, dt: f64);
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
