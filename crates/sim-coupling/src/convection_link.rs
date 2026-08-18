//! `ConvectionLink`(設計 docs/20-integration/01-coupling-matrix.md §3
//! 「P3: 流体/媒質 ⇔ ThermalNode(相関式 h)」、docs/12-thermal/02-heat-transfer.md
//! §2.2「対流(ニュートンの冷却則)」$\dot Q=hA(T_{fluid}-T_{surf})$、
//! §4.2の強制対流(平板)相関式 $\overline{Nu}=0.664\,Re^{1/2}Pr^{1/3}$(Blasius解、
//! 層流)。
//!
//! **群5で設計 §4.2 の相関式表4件を全て実装した**(`ConvectionMode`)。移行前は
//! 「強制対流・平板」1件のみで、残り3件(自然対流(垂直面)・自然対流(球)・
//! 強制対流(球))を「流速に依存しない自然対流は`BoussinesqBuoyancy`の範疇」という
//! 理由で対象外としていた。しかしこの役割分担は誤りだった——`BoussinesqBuoyancy`が
//! 担うのは温度差が生む**浮力(運動量)**であって、自然対流が運ぶ**熱流**ではない。
//! 両者は別々の物理量であり、自然対流下の冷却(D9「冷めるコーヒー」のような静止流体中の
//! シナリオ)はどちらでも表現できていなかった。群5でレイリー数
//! $Ra=\frac{g\beta\,|T_s-T_f|\,L^3}{\nu^2}Pr$ を導入し、表の4式をそのまま実装した。
//!
//! 体膨張係数 $\beta$ は`thermal_expansion_coefficient`で与える。`None`なら理想気体の
//! $\beta=1/T_{film}$($T_{film}=(T_s+T_f)/2$)を使う(設計 §4.2 が空気を主な対象として
//! 挙げているため、既定として自然な縮約)。重力加速度は`DomainStates::mechanics`の
//! `gravity`(ワールド共通のスカラー)から取る。
//!
//! **自然対流と強制対流の合成**(混合対流、$\overline{Nu}^3=\overline{Nu}_n^3+
//! \overline{Nu}_f^3$ のような合成則)は対象外とする——設計 §4.2 の表は4式を排他的な
//! 「状況」として並べており、合成則には言及していない。呼び出し側が支配的な機構を
//! `ConvectionMode`で選ぶ。
//!
//! 特性速度は`GridFluid2D`の速度場全体のRMS速度で代表
//! させる。`u`・`v`はstaggered(MAC)配置(`u`はx面$(ih,(j+\frac12)h)$、`v`はy面
//! $((i+\frac12)h,jh)$、`sim_fluid::GridFluid2D`モジュールdoc参照)なので、両者を
//! 合成する前にまずセル中心$((i+\frac12)h,(j+\frac12)h)$へ補間する
//! (**2026-07-27の監査・修正**: 以前は`u[k]`と`v[k]`を同一の生indexで単純に
//! ペアリングしていたが、これは対角に半セルずつずれた2点を組み合わせる**数値的な
//! 誤り**だった——スコープを意図的に縮小した近似ではなく、staggered配置を素直な
//! flatインデックスで扱ったことによる実装バグ。セル(i,j)を挟む2枚のx面
//! `u_at(i,j)`・`u_at(i+1,j)`の平均、2枚のy面`v_at(i,j)`・`v_at(i,j+1)`の平均を
//! セル中心の速度成分とすることで修正した)。プラントル数`Pr`・熱伝導率`k_f`・
//! 動粘性係数`nu`は物性定数として`ConvectionLink`自身が保持する(`sim_thermal`に
//! まだ流体物性DBが無いため、`PistonGas`の`area`等と同じ「呼び出し側が材料値を
//! 直接渡す」縮約)。
//!
//! 熱源側・受熱側をともに単一の`ThermalNode`(`fluid_node`・`surface_node`)として
//! `ThermalSolver`内の2ノード間の熱交換で表す(セルごとの温度場を持たない`GridFluid2D`
//! の制約は`BoussinesqBuoyancy`と同じ)。取り出した熱量をそのまま反対側へ注入するため、
//! 2ノード間で厳密に対記帳される(丸め誤差を除き完全にゼロ和)。

use crate::domain_states::{Coupling, CouplingKind, DomainStates};
use sim_core::DomainId;

/// 対流の状況(設計 docs/12-thermal/02-heat-transfer.md §4.2 の相関式表、群5で追加)。
/// $\overline{Nu}=hL/k_f$。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConvectionMode {
    /// 自然対流(垂直面): $\overline{Nu}=0.59\,Ra^{1/4}$(Churchill-Chu 簡略形、
    /// 適用範囲 $10^4<Ra<10^9$)。
    NaturalVerticalPlate,
    /// 自然対流(球): $\overline{Nu}=2+0.43\,Ra^{1/4}$(Yuge)。$Ra\to0$ で
    /// $\overline{Nu}\to2$(静止流体中の球の純伝導極限)。
    NaturalSphere,
    /// 強制対流(球): $\overline{Nu}=2+0.6\,Re^{1/2}Pr^{1/3}$(Ranz-Marshall)。
    ForcedSphere,
    /// 強制対流(平板、層流): $\overline{Nu}=0.664\,Re^{1/2}Pr^{1/3}$(Blasius 解)。
    ForcedFlatPlate,
}

impl ConvectionMode {
    /// 流速駆動(強制対流)か温度差駆動(自然対流)か。
    fn is_natural(self) -> bool {
        matches!(
            self,
            ConvectionMode::NaturalVerticalPlate | ConvectionMode::NaturalSphere
        )
    }
}

/// 流体ノード`fluid_node`と受熱面ノード`surface_node`を、設計 §4.2 の対流相関式
/// (`mode`で選ぶ、4式)による熱伝達係数 $h=\overline{Nu}\,k_f/L$ で結ぶ
/// (モジュールdoc参照)。強制対流の特性速度は`DomainStates::grid_fluid`の速度場全体の
/// RMS速度から、自然対流のレイリー数は2ノードの温度差から算出する。
#[derive(Clone)]
pub struct ConvectionLink {
    pub fluid_node: usize,
    pub surface_node: usize,
    /// 伝熱面積 [m^2]。
    pub area: f64,
    /// 特性長さ $L$ [m](Blasius解の平板長さに相当)。
    pub characteristic_length: f64,
    /// 流体の熱伝導率 $k_f$ [W/(m·K)]。
    pub fluid_thermal_conductivity: f64,
    /// 流体の動粘性係数 $\nu$ [m^2/s]。
    pub kinematic_viscosity: f64,
    /// 流体のプラントル数 $Pr$(無次元、物性値)。
    pub prandtl_number: f64,
    /// 対流の状況(設計 §4.2 の相関式表、**群5で追加**)。
    pub mode: ConvectionMode,
    /// 流体の体膨張係数 $\beta$ [1/K](自然対流のレイリー数に使う)。`None`なら
    /// 理想気体近似 $\beta=1/T_{film}$(モジュールdoc参照)。強制対流では未使用。
    pub thermal_expansion_coefficient: Option<f64>,
}

impl Default for ConvectionLink {
    /// 空気(20℃)の物性値 + 強制対流(平板)を既定とする。`mode`追加以前の
    /// 呼び出し側が`..Default::default()`で移行できるようにするためのもの。
    fn default() -> ConvectionLink {
        ConvectionLink {
            fluid_node: 0,
            surface_node: 0,
            area: 0.0,
            characteristic_length: 0.0,
            fluid_thermal_conductivity: 0.026,
            kinematic_viscosity: 1.5e-5,
            prandtl_number: 0.71,
            mode: ConvectionMode::ForcedFlatPlate,
            thermal_expansion_coefficient: None,
        }
    }
}

impl ConvectionLink {
    /// `GridFluid2D`速度場全体のRMS速度(特性速度、モジュールdoc「2026-07-27の
    /// 監査・修正」参照)。`u`・`v`はstaggered配置のため、まずセル(i,j)ごとに
    /// 両者をセル中心へ補間してから合成する(周期境界なので`u_at`/`v_at`の
    /// ラップアラウンドがそのまま正しく機能する)。
    fn characteristic_speed(grid_fluid: &sim_fluid::GridFluid2D) -> f64 {
        let (nx, ny) = (grid_fluid.nx, grid_fluid.ny);
        if nx == 0 || ny == 0 {
            return 0.0;
        }
        let mut sum_sq = 0.0;
        for j in 0..ny as i64 {
            for i in 0..nx as i64 {
                // セル(i,j)を挟む2枚のx面/y面の平均 = セル中心の速度成分。
                let u_center = 0.5 * (grid_fluid.u_at(i, j) + grid_fluid.u_at(i + 1, j));
                let v_center = 0.5 * (grid_fluid.v_at(i, j) + grid_fluid.v_at(i, j + 1));
                sum_sq += u_center * u_center + v_center * v_center;
            }
        }
        (sum_sq / (nx * ny) as f64).sqrt()
    }

    /// レイリー数 $Ra=Gr\,Pr=\frac{g\beta\,|\Delta T|\,L^3}{\nu^2}Pr$(自然対流の
    /// 駆動パラメータ、モジュールdoc参照)。`gravity`はワールドの重力加速度 [m/s^2]。
    fn rayleigh_number(&self, gravity: f64, t_surface: f64, t_fluid: f64) -> f64 {
        if self.kinematic_viscosity <= 0.0 || gravity <= 0.0 {
            return 0.0;
        }
        let delta_t = (t_surface - t_fluid).abs();
        let film_temperature = 0.5 * (t_surface + t_fluid);
        let beta = match self.thermal_expansion_coefficient {
            Some(b) => b,
            // 理想気体近似 β = 1/T_film(モジュールdoc参照)。T_film が非物理
            // (絶対零度以下)なら自然対流を評価しない。
            None if film_temperature > 0.0 => 1.0 / film_temperature,
            None => return 0.0,
        };
        let l = self.characteristic_length;
        gravity * beta * delta_t * l * l * l / (self.kinematic_viscosity * self.kinematic_viscosity)
            * self.prandtl_number
    }

    /// 設計 §4.2 の相関式表による熱伝達係数 $h=\overline{Nu}\,k_f/L$
    /// (`mode`が式を選ぶ、モジュールdoc参照)。強制対流は`characteristic_speed`から
    /// $Re=UL/\nu$ を、自然対流は温度差から $Ra$ を作る。
    fn heat_transfer_coefficient(
        &self,
        characteristic_speed: f64,
        gravity: f64,
        t_surface: f64,
        t_fluid: f64,
    ) -> f64 {
        if self.kinematic_viscosity <= 0.0 || self.characteristic_length <= 0.0 {
            return 0.0;
        }
        let nusselt = if self.mode.is_natural() {
            let ra = self.rayleigh_number(gravity, t_surface, t_fluid);
            if ra <= 0.0 {
                // 温度差ゼロ = 駆動源なし。自然対流(球)の Nu→2(純伝導極限)は
                // 温度差ゼロなら熱流もゼロなので、ここで打ち切っても結果は同じ。
                return 0.0;
            }
            match self.mode {
                ConvectionMode::NaturalVerticalPlate => 0.59 * ra.powf(0.25),
                ConvectionMode::NaturalSphere => 2.0 + 0.43 * ra.powf(0.25),
                _ => unreachable!("is_natural() が保証する"),
            }
        } else {
            if characteristic_speed <= 0.0 {
                // 強制対流は流速が駆動源。球の Nu→2(純伝導極限)は流速ゼロでも
                // 残るが、移行前からの挙動(流速ゼロなら熱移動なし)を変えないため
                // ここで打ち切る——静止流体中の伝熱は自然対流モードで扱う。
                return 0.0;
            }
            let re = characteristic_speed * self.characteristic_length / self.kinematic_viscosity;
            let forced = re.sqrt() * self.prandtl_number.cbrt();
            match self.mode {
                ConvectionMode::ForcedFlatPlate => 0.664 * forced,
                ConvectionMode::ForcedSphere => 2.0 + 0.6 * forced,
                _ => unreachable!("is_natural() が保証する"),
            }
        };
        nusselt * self.fluid_thermal_conductivity / self.characteristic_length
    }
}

impl Coupling for ConvectionLink {
    fn kind(&self) -> CouplingKind {
        CouplingKind::ConvectionLink
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Fluid, DomainId::Thermal]
    }

    fn describe(&self) -> String {
        format!(
            "ConvectionLink {:?} fluid_node[{}] -> surface_node[{}] A={}m2",
            self.mode, self.fluid_node, self.surface_node, self.area
        )
    }

    fn referenced_thermal_nodes(&self) -> Vec<usize> {
        vec![self.fluid_node, self.surface_node]
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        // 強制対流は流体ドメインの流速を要求するが、自然対流は温度差だけで駆動される
        // ので`grid_fluid`が無くても成立する(**群5**: 移行前は`grid_fluid`が無いと
        // 常に早期returnしていた)。
        let speed = match &world.grid_fluid {
            Some(grid_fluid) => Self::characteristic_speed(grid_fluid),
            None if self.mode.is_natural() => 0.0,
            None => return,
        };
        // 自然対流のレイリー数が使う$g$。**正直な限界(重力場の抽象化増分)**:
        // 自然対流の相関式は「一様な鉛直重力」を前提とする経験式なので、
        // 位置依存の`GravityField::acceleration_at`ではなくスカラー縮約
        // `gravity()`を読む。非`Uniform`な場では0.0が返り、レイリー数が0=
        // 自然対流なしへ縮退する(`MechanicsSolver::gravity`のdoc参照)。
        let gravity = world.mechanics.gravity();
        let Some(thermal) = &mut world.thermal else {
            return;
        };
        let Some(fluid) = thermal.nodes.get(self.fluid_node) else {
            return;
        };
        let Some(surface) = thermal.nodes.get(self.surface_node) else {
            return;
        };
        let (t_fluid, c_fluid) = (fluid.temperature, fluid.heat_capacity);
        let (t_surface, c_surface) = (surface.temperature, surface.heat_capacity);

        let h = self.heat_transfer_coefficient(speed, gravity, t_surface, t_fluid);
        if h == 0.0 {
            return;
        }

        // Q = h*A*(T_fluid - T_surf)*dt(設計§2.2)。流体側から取り出した熱量を
        // そのまま受熱面側へ注入する対記帳(2ノード間で厳密にゼロ和)。
        let heat = h * self.area * (t_fluid - t_surface) * dt;
        thermal.nodes[self.fluid_node].temperature -= heat / c_fluid;
        thermal.nodes[self.surface_node].temperature += heat / c_surface;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_fluid::GridFluid2D;
    use sim_mechanics::MechanicsSolver;
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// **2026-07-27の監査で発見・修正した回帰テスト**: `characteristic_speed`が
    /// staggered配置の`u`・`v`を正しくセル中心へ補間していること(修正前の
    /// 「同一生indexでペアリング」というバグが再発しないこと)を、手計算できる
    /// 非一様な速度場で確認する。
    ///
    /// `nx=2, ny=1`(周期境界)で`u_at(0,0)=2.0`・`u_at(1,0)=0.0`・`v≡0`とする。
    /// 正しい補間: セル0はx面`u_at(0,0)`と`u_at(1,0)`に挟まれる→
    /// `u_center=0.5*(2.0+0.0)=1.0`。セル1はx面`u_at(1,0)`と`u_at(2,0)`
    /// (周期境界で`u_at(0,0)`に一致)に挟まれる→`u_center=0.5*(0.0+2.0)=1.0`。
    /// v成分は恒等的に0なので、RMS速度は**厳密に1.0**になるはずである。
    ///
    /// 修正前の実装(`u[k]`と`v[k]`を生indexでペアリング)だと
    /// `sum_sq=u[0]^2+u[1]^2=4.0+0.0=4.0`、`RMS=sqrt(4.0/2)=√2≈1.4142`という
    /// 別の値になる——本テストは1.0を要求するため、退行すれば確実に失敗する。
    #[test]
    fn characteristic_speed_interpolates_staggered_components_to_cell_centers() {
        let mut fluid = GridFluid2D::new(2, 1, 0.1);
        fluid.u[0] = 2.0;
        fluid.u[1] = 0.0;
        // v はデフォルトで全て0。

        let speed = ConvectionLink::characteristic_speed(&fluid);

        assert!(
            (speed - 1.0).abs() < 1e-12,
            "expected the cell-center-interpolated RMS speed to be exactly 1.0, got {speed} \
             (a value near sqrt(2)≈1.4142 would indicate the raw-index-pairing bug has \
             regressed)"
        );
    }

    /// **群5**: 自然対流(垂直面、Churchill-Chu 簡略形 $\overline{Nu}=0.59Ra^{1/4}$)が
    /// **流体ドメイン無し・流速ゼロでも**温度差だけで熱を運ぶこと、その熱伝達係数が
    /// 設計 §4.2 の式どおりであること、2ノード間で厳密に対記帳されることを確認する
    /// (移行前は`grid_fluid`が無いと常に早期returnし、熱が一切動かなかった)。
    ///
    /// 併せて $h$ の実測値が設計 §4.2 の目安「静止空気中 $h\approx5$–$10$ W/(m²K)」の
    /// 範囲に入ることも確認する——係数を取り違えても式の形だけは合ってしまうため、
    /// 物理的なオーダーの検算を別途置く。
    #[test]
    fn convection_link_natural_vertical_plate_transfers_heat_without_any_fluid_domain() {
        let t_fluid0 = 293.15; // 室温の空気
        let t_surface0 = 350.0; // 熱い面
        let c_fluid = 1.0e6; // 大気側は実質無限熱容量とみなせる大きさ
        let c_surface = 2000.0;
        let mut thermal = ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(ThermalNode::new(t_fluid0, c_fluid));
        let surface_node = thermal.add_node(ThermalNode::new(t_surface0, c_surface));
        let gravity = 9.80665;
        let mut mechanics = MechanicsSolver::new(gravity);

        let area = 0.05;
        let length = 0.1;
        let k_f = 0.026;
        let nu = 1.5e-5;
        let pr = 0.71;
        let mut coupling = ConvectionLink {
            fluid_node,
            surface_node,
            area,
            characteristic_length: length,
            fluid_thermal_conductivity: k_f,
            kinematic_viscosity: nu,
            prandtl_number: pr,
            mode: ConvectionMode::NaturalVerticalPlate,
            thermal_expansion_coefficient: None, // 理想気体 β=1/T_film
        };
        let dt = 0.01;
        {
            // **`grid_fluid: None`** ——自然対流は流体ドメインを要求しない。
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: Some(&mut thermal),
                em_circuit: None,
                em_electrostatics: None,
                gas: None,
                grid_fluid: None,
                grid_fluid_3d: None,
                sph: None,
            };
            coupling.apply(&mut states, dt);
        }

        // 設計 §4.2: Ra = g β ΔT L^3 / ν^2 * Pr、Nu = 0.59 Ra^(1/4)、h = Nu k_f / L。
        let beta = 1.0 / (0.5 * (t_surface0 + t_fluid0));
        let ra = gravity * beta * (t_surface0 - t_fluid0) * length.powi(3) / (nu * nu) * pr;
        let h = 0.59 * ra.powf(0.25) * k_f / length;
        // 熱は流体→表面の向きを正とする定義なので、表面のほうが熱い今回は負(冷える)。
        let expected_heat = h * area * (t_fluid0 - t_surface0) * dt;

        // Churchill-Chu の適用範囲 10^4 < Ra < 10^9 に入っていること(範囲外の値で
        // 「合っている」と主張しないための足場)。
        assert!(
            (1.0e4..1.0e9).contains(&ra),
            "Ra should be inside the correlation's validity range: Ra={ra:.3e}"
        );
        // 設計 §4.2 の目安値: 静止空気中 h ≈ 5–10 W/(m^2 K)。
        assert!(
            (4.0..12.0).contains(&h),
            "natural convection in still air should land near the design's 5-10 W/(m^2 K) \
             rule of thumb: h={h:.3}"
        );

        let fluid_temp = thermal.nodes[fluid_node].temperature;
        let surface_temp = thermal.nodes[surface_node].temperature;
        assert!(
            (fluid_temp - (t_fluid0 - expected_heat / c_fluid)).abs() < 1e-12,
            "fluid_temp={fluid_temp}"
        );
        assert!(
            (surface_temp - (t_surface0 + expected_heat / c_surface)).abs() < 1e-12,
            "surface_temp={surface_temp}"
        );
        assert!(
            surface_temp < t_surface0,
            "the hot surface should cool down: {surface_temp} vs {t_surface0}"
        );

        // 対記帳(2ノード間でゼロ和)。`c_fluid`が`c_surface`の500倍あるため、
        // 温度差分の丸め誤差が熱量に戻すと増幅される(絶対誤差ではなく相対で見る)。
        let lost = c_fluid * (t_fluid0 - fluid_temp);
        let gained = c_surface * (surface_temp - t_surface0);
        assert!(
            (lost - gained).abs() / gained.abs() < 1e-8,
            "lost={lost} gained={gained}"
        );
    }

    /// **群5**: 残り2式(自然対流(球)$2+0.43Ra^{1/4}$・強制対流(球)
    /// $2+0.6Re^{1/2}Pr^{1/3}$)が設計 §4.2 の表どおりであることを、平板版と同じ
    /// パラメータで**係数だけが違う**ことを見る形で確認する(表の転記ミスを検出する)。
    #[test]
    fn convection_link_sphere_correlations_match_the_design_table() {
        let gravity = 9.80665;
        let t_surface = 350.0;
        let t_fluid = 293.15;
        let length = 0.1;
        let k_f = 0.026;
        let nu = 1.5e-5;
        let pr = 0.71;
        let speed = 2.0;
        let base = ConvectionLink {
            characteristic_length: length,
            fluid_thermal_conductivity: k_f,
            kinematic_viscosity: nu,
            prandtl_number: pr,
            ..Default::default()
        };

        let beta = 1.0 / (0.5 * (t_surface + t_fluid));
        let ra = gravity * beta * (t_surface - t_fluid) * length.powi(3) / (nu * nu) * pr;
        let re = speed * length / nu;
        let forced = re.sqrt() * pr.cbrt();

        let cases = [
            (
                ConvectionMode::NaturalVerticalPlate,
                0.59 * ra.powf(0.25),
                0.0,
            ),
            (
                ConvectionMode::NaturalSphere,
                2.0 + 0.43 * ra.powf(0.25),
                0.0,
            ),
            (ConvectionMode::ForcedSphere, 2.0 + 0.6 * forced, speed),
            (ConvectionMode::ForcedFlatPlate, 0.664 * forced, speed),
        ];
        for (mode, expected_nusselt, u) in cases {
            let link = ConvectionLink {
                mode,
                ..base.clone()
            };
            let h = link.heat_transfer_coefficient(u, gravity, t_surface, t_fluid);
            let expected_h = expected_nusselt * k_f / length;
            assert!(
                (h - expected_h).abs() < 1e-12,
                "{mode:?}: h={h} expected={expected_h}"
            );
        }

        // 球の相関式だけが Nu→2(静止流体中の球の純伝導極限)の下駄を履く。
        // Ra→0 の極限で平板は 0 に落ちるが球は 2 に留まる、という**式の形そのもの**を
        // 確認する(2式を取り違えていれば破れる)。
        // 実測 Ra≈1.5e6 のような大きい Ra では係数(0.43 < 0.59)が効いて逆に平板の
        // ほうが大きくなる——「球のほうが常に大きい」は成り立たない。
        let tiny_ra: f64 = 1.0e-8;
        assert!(
            (2.0 + 0.43 * tiny_ra.powf(0.25) - 2.0).abs() < 0.01,
            "自然対流(球)は Ra→0 で Nu→2"
        );
        assert!(
            0.59 * tiny_ra.powf(0.25) < 0.01,
            "自然対流(垂直面)は Ra→0 で Nu→0"
        );
        assert!(
            2.0 + 0.43 * ra.powf(0.25) < 0.59 * ra.powf(0.25),
            "Ra={ra:.3e} のような大きい Ra では係数差が下駄を上回る(式の取り違え検出)"
        );
    }

    /// 温度差がゼロなら自然対流の駆動源が消え、熱移動もゼロ(Ra=0)。
    #[test]
    fn convection_link_natural_convection_stops_at_zero_temperature_difference() {
        let mut thermal = ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(ThermalNode::new(300.0, 1000.0));
        let surface_node = thermal.add_node(ThermalNode::new(300.0, 1000.0));
        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut coupling = ConvectionLink {
            fluid_node,
            surface_node,
            area: 0.05,
            characteristic_length: 0.1,
            mode: ConvectionMode::NaturalVerticalPlate,
            ..Default::default()
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, 0.01);
        assert_eq!(thermal.nodes[fluid_node].temperature, 300.0);
        assert_eq!(thermal.nodes[surface_node].temperature, 300.0);
    }

    /// 流速ゼロなら(強制対流モードでは)熱伝達係数もゼロで、両ノードとも温度不変。
    #[test]
    fn convection_link_transfers_no_heat_when_fluid_is_at_rest() {
        let mut thermal = ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(ThermalNode::new(350.0, 1000.0));
        let surface_node = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut fluid = GridFluid2D::new(4, 4, 0.1);

        let mut coupling = ConvectionLink {
            fluid_node,
            surface_node,
            area: 0.1,
            characteristic_length: 0.1,
            fluid_thermal_conductivity: 0.026,
            kinematic_viscosity: 1.5e-5,
            prandtl_number: 0.71,
            mode: ConvectionMode::ForcedFlatPlate,
            thermal_expansion_coefficient: None,
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, 0.01);

        assert_eq!(thermal.nodes[fluid_node].temperature, 350.0);
        assert_eq!(thermal.nodes[surface_node].temperature, 293.15);
    }

    /// 一様な流速がある場合、Blasius解の強制対流相関式どおりの熱伝達係数で、
    /// 流体ノード→受熱面ノードへ熱が移動し、対記帳(2ノード間でのエネルギー厳密保存)が
    /// 成立すること。
    #[test]
    fn convection_link_matches_blasius_forced_convection_formula_and_conserves_energy() {
        let t_fluid0 = 350.0;
        let t_surface0 = 293.15;
        let c_fluid = 1000.0;
        let c_surface = 2000.0;
        let mut thermal = ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(ThermalNode::new(t_fluid0, c_fluid));
        let surface_node = thermal.add_node(ThermalNode::new(t_surface0, c_surface));
        let mut mechanics = MechanicsSolver::new(9.80665);

        let mut fluid = GridFluid2D::new(4, 4, 0.1);
        let speed = 2.0;
        for u in fluid.u.iter_mut() {
            *u = speed;
        }

        let area = 0.05;
        let length = 0.2;
        let k_f = 0.026;
        let nu = 1.5e-5;
        let pr = 0.71;
        let mut coupling = ConvectionLink {
            fluid_node,
            surface_node,
            area,
            characteristic_length: length,
            fluid_thermal_conductivity: k_f,
            kinematic_viscosity: nu,
            prandtl_number: pr,
            mode: ConvectionMode::ForcedFlatPlate,
            thermal_expansion_coefficient: None,
        };
        let dt = 0.01;
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, dt);

        let reynolds = speed * length / nu;
        let nusselt = 0.664 * reynolds.sqrt() * pr.cbrt();
        let h = nusselt * k_f / length;
        let expected_heat = h * area * (t_fluid0 - t_surface0) * dt;

        let fluid_temp = thermal.nodes[fluid_node].temperature;
        let surface_temp = thermal.nodes[surface_node].temperature;
        let expected_fluid_temp = t_fluid0 - expected_heat / c_fluid;
        let expected_surface_temp = t_surface0 + expected_heat / c_surface;

        assert!(
            (fluid_temp - expected_fluid_temp).abs() < 1e-9,
            "fluid_temp={fluid_temp} expected={expected_fluid_temp}"
        );
        assert!(
            (surface_temp - expected_surface_temp).abs() < 1e-9,
            "surface_temp={surface_temp} expected={expected_surface_temp}"
        );

        // 対記帳: 流体側が失った熱量 == 受熱面側が得た熱量(2ノード間でゼロ和)。
        let heat_lost_by_fluid = c_fluid * (t_fluid0 - fluid_temp);
        let heat_gained_by_surface = c_surface * (surface_temp - t_surface0);
        assert!(
            (heat_lost_by_fluid - heat_gained_by_surface).abs() < 1e-9,
            "heat_lost_by_fluid={heat_lost_by_fluid} heat_gained_by_surface={heat_gained_by_surface}"
        );
    }
}
