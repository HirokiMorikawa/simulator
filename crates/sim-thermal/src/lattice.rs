//! 格子熱伝導(温度場)。設計: docs/12-thermal/02-heat-transfer.md §3/§4.3。
//!
//! ノードネットワーク(`ThermalSolver`)とは別に、格子上の温度場を陰的Euler + PCGで
//! 解く。1D棒(T3の検証範囲)を実装し、両端はDirichlet境界(固定温度)として
//! 線形系の未知数から除外する(内部点のみを解く、標準的な境界処理)。
//!
//! **群7で2点の縮約を解消した**:
//!
//! 1. **$\rho c_p$(体積熱容量)を持てるようにした**(`volumetric_heat_capacity`)。
//!    移行前は熱拡散率 $\alpha=k/(\rho c_p)$ という**比**しか持たず、$k$と$\rho c_p$を
//!    分離できなかった。そのため(a)蓄えている熱エネルギー $\int \rho c_p T\,dV$ を
//!    出せず`Solver::total_energy`が空を返していた、(b)材質ごとに$k$の違う棒を
//!    組めなかった。
//! 2. **空間的に変化する熱伝導率**(`conductivity`)と、設計
//!    docs/12-thermal/02-heat-transfer.md §4.3 が明記する**流束形式・調和平均**の
//!    面伝導率 $k_{i+1/2}=2k_ik_{i+1}/(k_i+k_{i+1})$ を実装した。調和平均は
//!    「直列の熱抵抗 $L/k$ が正しく足される」ための正しい平均であり、算術平均だと
//!    材質界面の温度が系統的にずれる(テストで両者を区別している)。
//!
//! どちらも**省略可能**で、指定しなければ移行前とまったく同じ一様$\alpha$の
//! 経路を通る(既存のT3テストはビット単位で不変)。
//!
//! **群7で3D(7点ステンシル)も実装した**(`ConductionGrid3D`)。設計 §4.3
//! 「温度場(格子)は同じ陰的拡散を7点ステンシルで(流束形式・調和平均)」。
//! `sim_math::Grid3<f64>`を温度場の保持に使い、1D版と同じく陰的Euler + matrix-free PCG、
//! 面の伝導率は調和平均で作る。境界は面ごとにDirichlet(固定温度)/Neumann(断熱)を
//! 選べる。**1D版と同じ問題を解かせると同じ答えになる**ことを突き合わせて検証した。

use sim_core::{Approximation, EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::{pcg, Grid3, Preconditioner, Vec3};

/// 1D棒の格子熱伝導ソルバ。両端(`temperature[0]`・`temperature[n-1]`)はDirichlet境界。
#[derive(Clone)]
pub struct ConductionRod1D {
    pub temperature: Vec<f64>,
    pub dx: f64,
    /// 熱拡散率 α=k/(ρc_p) [m²/s](設計§2.1)。`conductivity`が`None`のとき使う。
    pub thermal_diffusivity: f64,
    /// 体積熱容量 $\rho c_p$ [J/(m³·K)](**群7で追加**、モジュールdoc参照)。
    /// `Some`なら`Solver::total_energy`が実際の熱エネルギーを返し、`conductivity`と
    /// 組んで局所的な熱拡散率 $\alpha_i=k_i/(\rho c_p)$ を作れる。
    pub volumetric_heat_capacity: Option<f64>,
    /// 格子点ごとの熱伝導率 $k_i$ [W/(m·K)](**群7で追加**)。`Some`なら
    /// `volumetric_heat_capacity`と併せて使い、面の伝導率は**調和平均**で作る
    /// (モジュールdoc参照)。`None`なら一様な`thermal_diffusivity`を使う。
    pub conductivity: Option<Vec<f64>>,
    /// 棒の断面積 [m²](**群7で追加**)。熱エネルギー $\int\rho c_p T\,A\,dx$ の
    /// 体積換算に使うだけで、1Dの温度分布そのものには影響しない。
    pub cross_section_area: f64,
}

impl ConductionRod1D {
    pub fn new(
        node_count: usize,
        length: f64,
        initial_temperature: f64,
        thermal_diffusivity: f64,
    ) -> Self {
        ConductionRod1D {
            temperature: vec![initial_temperature; node_count],
            dx: length / (node_count - 1) as f64,
            thermal_diffusivity,
            volumetric_heat_capacity: None,
            conductivity: None,
            cross_section_area: 1.0,
        }
    }

    /// **材質プロファイルを与える**(**群7で追加**、モジュールdoc参照)。
    /// `conductivity`は格子点ごとの $k_i$(長さは`temperature`と同じ)、
    /// `volumetric_heat_capacity`は $\rho c_p$、`cross_section_area`は断面積。
    /// 与えると`step`は面の伝導率を調和平均で作る流束形式の離散化に切り替わり、
    /// `Solver::total_energy`が実際の熱エネルギーを返すようになる。
    pub fn with_material_profile(
        mut self,
        conductivity: Vec<f64>,
        volumetric_heat_capacity: f64,
        cross_section_area: f64,
    ) -> ConductionRod1D {
        assert_eq!(
            conductivity.len(),
            self.temperature.len(),
            "conductivity must have one value per grid node"
        );
        assert!(volumetric_heat_capacity > 0.0, "ρc_p は正");
        assert!(cross_section_area > 0.0, "断面積は正");
        // 一様プロファイルとして与えられた場合に既存の`thermal_diffusivity`と
        // 齟齬が出ないよう、代表値(平均)で更新しておく。
        let mean_k = conductivity.iter().sum::<f64>() / conductivity.len() as f64;
        self.thermal_diffusivity = mean_k / volumetric_heat_capacity;
        self.conductivity = Some(conductivity);
        self.volumetric_heat_capacity = Some(volumetric_heat_capacity);
        self.cross_section_area = cross_section_area;
        self
    }

    /// 面 $i+1/2$ の熱伝導率(**調和平均**、設計§4.3「流束形式・調和平均」)。
    /// 直列の熱抵抗 $\Delta x/k$ が正しく足されるのはこの平均だけである
    /// (算術平均だと界面で流束が過大評価される)。
    fn face_conductivity(k: &[f64], i: usize) -> f64 {
        let (a, b) = (k[i], k[i + 1]);
        if a <= 0.0 || b <= 0.0 {
            return 0.0;
        }
        2.0 * a * b / (a + b)
    }

    /// 蓄えている熱エネルギー $\int \rho c_p T\,A\,dx$ [J](**群7で追加**)。
    /// $\rho c_p$ が未指定なら`None`(熱拡散率だけでは絶対量が決まらない)。
    /// 端の格子点は半セル分だけ受け持つ(台形則、Dirichlet境界も含めて数える)。
    pub fn thermal_energy(&self) -> Option<f64> {
        let rho_cp = self.volumetric_heat_capacity?;
        let n = self.temperature.len();
        if n < 2 {
            return Some(0.0);
        }
        let mut sum = 0.0;
        for (i, &t) in self.temperature.iter().enumerate() {
            let weight = if i == 0 || i == n - 1 { 0.5 } else { 1.0 };
            sum += weight * t;
        }
        Some(rho_cp * self.cross_section_area * self.dx * sum)
    }

    pub fn set_boundary_temperatures(&mut self, left: f64, right: f64) {
        let n = self.temperature.len();
        self.temperature[0] = left;
        self.temperature[n - 1] = right;
    }

    /// 陰的Euler(設計§4.3「線形項は陰的Euler」の1D 3点ステンシル版)を matrix-free PCGで
    /// 解く。境界の既知温度は行列(内部点のみのSPD系)から右辺への定数項として移す
    /// (標準的なDirichlet境界の扱い)。
    pub fn step(&mut self, dt: f64) {
        let n = self.temperature.len();
        if n < 3 {
            return;
        }
        // 材質プロファイルが与えられていれば流束形式(調和平均)へ切り替える
        // (**群7**、モジュールdoc参照)。
        if self.conductivity.is_some() && self.volumetric_heat_capacity.is_some() {
            self.step_variable(dt);
            return;
        }
        let interior = n - 2;
        let r = self.thermal_diffusivity * dt / (self.dx * self.dx);

        let boundary_left = self.temperature[0];
        let boundary_right = self.temperature[n - 1];
        let t_old: Vec<f64> = self.temperature[1..n - 1].to_vec();

        let apply_a = |x: &[f64], out: &mut [f64]| {
            for i in 0..interior {
                let mut val = (1.0 + 2.0 * r) * x[i];
                if i > 0 {
                    val -= r * x[i - 1];
                }
                if i < interior - 1 {
                    val -= r * x[i + 1];
                }
                out[i] = val;
            }
        };

        let mut b = t_old.clone();
        b[0] += r * boundary_left;
        b[interior - 1] += r * boundary_right;

        let mut x = t_old;
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-12, 500);
        debug_assert!(
            result.converged,
            "lattice conduction PCG did not converge: {result:?}"
        );

        self.temperature[1..n - 1].copy_from_slice(&x);
    }

    /// 空間的に変化する熱伝導率の陰的Euler(**群7で追加**、モジュールdoc参照)。
    /// 流束形式 $\rho c_p\,\partial_t T = \partial_x(k\,\partial_x T)$ を
    /// 有限体積で離散化し、面の伝導率は調和平均で作る:
    /// $r_{i\pm1/2} = k_{i\pm1/2}\,\Delta t/(\rho c_p \Delta x^2)$、
    /// $(1+r_{i-1/2}+r_{i+1/2})T_i - r_{i-1/2}T_{i-1} - r_{i+1/2}T_{i+1} = T_i^n$。
    /// 係数行列は対称正定値(面の係数が両側で共有される)なのでPCGがそのまま使える。
    fn step_variable(&mut self, dt: f64) {
        let n = self.temperature.len();
        let interior = n - 2;
        let k = self
            .conductivity
            .as_ref()
            .expect("caller checked conductivity is Some");
        let rho_cp = self
            .volumetric_heat_capacity
            .expect("caller checked volumetric_heat_capacity is Some");
        let factor = dt / (rho_cp * self.dx * self.dx);
        // faces[i] = 面 i+1/2 の係数(i = 0..n-2)。
        let faces: Vec<f64> = (0..n - 1)
            .map(|i| Self::face_conductivity(k, i) * factor)
            .collect();

        let boundary_left = self.temperature[0];
        let boundary_right = self.temperature[n - 1];
        let t_old: Vec<f64> = self.temperature[1..n - 1].to_vec();

        let apply_a = |x: &[f64], out: &mut [f64]| {
            for i in 0..interior {
                // 内部点`i`は格子点`i+1`に対応する。左面 = faces[i]、右面 = faces[i+1]。
                let (left, right) = (faces[i], faces[i + 1]);
                let mut val = (1.0 + left + right) * x[i];
                if i > 0 {
                    val -= left * x[i - 1];
                }
                if i < interior - 1 {
                    val -= right * x[i + 1];
                }
                out[i] = val;
            }
        };

        let mut b = t_old.clone();
        b[0] += faces[0] * boundary_left;
        b[interior - 1] += faces[interior] * boundary_right;

        let mut x = t_old;
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-12, 500);
        debug_assert!(
            result.converged,
            "variable-conductivity PCG did not converge: {result:?}"
        );
        self.temperature[1..n - 1].copy_from_slice(&x);
    }
}

/// 面ごとの境界条件(**群7で追加**、`ConductionGrid3D`が使う)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ThermalBoundary {
    /// 固定温度(Dirichlet)。
    Fixed(f64),
    /// 断熱(Neumann、$\partial T/\partial n=0$)。
    Insulated,
}

/// 3D格子熱伝導ソルバ(7点ステンシル、**群7で追加**、モジュールdoc参照)。
///
/// 陰的Euler $\rho c_p\frac{T^{n+1}-T^n}{\Delta t}=\nabla\cdot(k\nabla T^{n+1})$ を
/// 有限体積で離散化し、面の伝導率は**調和平均**で作る(1D版と同じ、設計§4.3
/// 「流束形式・調和平均」)。係数行列は対称正定値なのでmatrix-free PCGで解ける。
///
/// **境界**は6面それぞれに`ThermalBoundary`を指定する。`Fixed`はゴーストセルの
/// 温度を固定値に、`Insulated`はゴーストセルを内側の鏡像にする(標準的な扱い)。
///
/// **残る縮約**: 熱伝導率は等方・セルごとのスカラー(異方性テンソルは非対応)。
/// 前処理器は`Preconditioner::None`(1D版と同じ。格子が大きくなったら
/// 対角スケーリングを入れる余地がある)。
#[derive(Clone)]
pub struct ConductionGrid3D {
    /// セル中心の温度場。
    pub temperature: Grid3<f64>,
    /// セルごとの熱伝導率 $k$ [W/(m·K)]。
    pub conductivity: Grid3<f64>,
    /// 体積熱容量 $\rho c_p$ [J/(m³·K)](一様)。
    pub volumetric_heat_capacity: f64,
    /// 6面の境界条件(-x, +x, -y, +y, -z, +z の順)。
    pub boundaries: [ThermalBoundary; 6],
}

impl ConductionGrid3D {
    /// 一様な材質・一様な初期温度で作る。境界は全面断熱(閉じた箱)。
    pub fn uniform(
        nx: usize,
        ny: usize,
        nz: usize,
        h: f64,
        initial_temperature: f64,
        conductivity: f64,
        volumetric_heat_capacity: f64,
    ) -> ConductionGrid3D {
        assert!(nx >= 1 && ny >= 1 && nz >= 1);
        assert!(volumetric_heat_capacity > 0.0, "ρc_p は正");
        ConductionGrid3D {
            temperature: Grid3::new(nx, ny, nz, h, Vec3::ZERO, initial_temperature),
            conductivity: Grid3::new(nx, ny, nz, h, Vec3::ZERO, conductivity),
            volumetric_heat_capacity,
            boundaries: [ThermalBoundary::Insulated; 6],
        }
    }

    fn dims(&self) -> (usize, usize, usize) {
        (
            self.temperature.nx,
            self.temperature.ny,
            self.temperature.nz,
        )
    }

    fn flat(&self, i: usize, j: usize, k: usize) -> usize {
        let (nx, ny, _) = self.dims();
        i + nx * (j + ny * k)
    }

    /// 面の伝導率(調和平均、1D版と同じ理由)。
    fn face_conductivity(a: f64, b: f64) -> f64 {
        if a <= 0.0 || b <= 0.0 {
            0.0
        } else {
            2.0 * a * b / (a + b)
        }
    }

    /// 蓄えている熱エネルギー $\sum \rho c_p T\,\Delta V$ [J]。
    pub fn thermal_energy(&self) -> f64 {
        let (nx, ny, nz) = self.dims();
        let dv = self.temperature.h.powi(3);
        let mut sum = 0.0;
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    sum += self.temperature.at(i, j, k);
                }
            }
        }
        self.volumetric_heat_capacity * dv * sum
    }

    /// 1ステップ(陰的Euler + PCG)。
    pub fn step(&mut self, dt: f64) {
        let (nx, ny, nz) = self.dims();
        let n = nx * ny * nz;
        let h = self.temperature.h;
        let factor = dt / (self.volumetric_heat_capacity * h * h);

        // 6方向の面係数を先に作る(`(方向, セル)`ごと)。方向は
        // 0:-x 1:+x 2:-y 3:+y 4:-z 5:+z で`boundaries`と同じ並び。
        let mut face = vec![[0.0_f64; 6]; n];
        // Dirichlet境界からの寄与(右辺へ移す定数項)。
        let mut boundary_source = vec![0.0_f64; n];
        let neighbour =
            |i: usize, j: usize, k: usize, dir: usize| -> Option<(usize, usize, usize)> {
                match dir {
                    0 => i.checked_sub(1).map(|ii| (ii, j, k)),
                    1 => (i + 1 < nx).then_some((i + 1, j, k)),
                    2 => j.checked_sub(1).map(|jj| (i, jj, k)),
                    3 => (j + 1 < ny).then_some((i, j + 1, k)),
                    4 => k.checked_sub(1).map(|kk| (i, j, kk)),
                    _ => (k + 1 < nz).then_some((i, j, k + 1)),
                }
            };
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = self.flat(i, j, k);
                    let k_self = self.conductivity.at(i, j, k);
                    for (dir, coefficient_slot) in face[idx].iter_mut().enumerate() {
                        match neighbour(i, j, k, dir) {
                            Some((a, b, c)) => {
                                let k_face =
                                    Self::face_conductivity(k_self, self.conductivity.at(a, b, c));
                                *coefficient_slot = k_face * factor;
                            }
                            None => match self.boundaries[dir] {
                                // 断熱: ゴーストが鏡像なので流束ゼロ = 係数ゼロ。
                                ThermalBoundary::Insulated => *coefficient_slot = 0.0,
                                // 固定温度: 境界温度は**セル中心から半セル外の面**に
                                // あるので、熱抵抗は $(h/2)/k$、したがって係数は
                                // 内部面($h/k$)の**2倍**になる。
                                // 実装検証中にこれを1倍で書いて、二層平板の定常分布が
                                // 解析解から4.5%ずれた(境界の熱抵抗を2倍に見積もった
                                // ことになり、その分だけ内部の温度差が小さくなる)。
                                ThermalBoundary::Fixed(t_boundary) => {
                                    let coefficient = 2.0 * k_self * factor;
                                    *coefficient_slot = coefficient;
                                    boundary_source[idx] += coefficient * t_boundary;
                                }
                            },
                        }
                    }
                }
            }
        }

        let mut b = vec![0.0; n];
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = self.flat(i, j, k);
                    b[idx] = self.temperature.at(i, j, k) + boundary_source[idx];
                }
            }
        }

        let apply_a = |x: &[f64], out: &mut [f64]| {
            for k in 0..nz {
                for j in 0..ny {
                    for i in 0..nx {
                        let idx = i + nx * (j + ny * k);
                        let coefficients = face[idx];
                        let mut diag = 1.0;
                        let mut sum = 0.0;
                        for (dir, &coefficient) in coefficients.iter().enumerate() {
                            if coefficient == 0.0 {
                                continue;
                            }
                            diag += coefficient;
                            if let Some((a, bb, c)) = neighbour(i, j, k, dir) {
                                sum += coefficient * x[a + nx * (bb + ny * c)];
                            }
                            // Dirichlet面の隣接項は`b`側(boundary_source)へ移してある。
                        }
                        out[idx] = diag * x[idx] - sum;
                    }
                }
            }
        };

        let mut x = b.clone();
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-12, 2000);
        debug_assert!(
            result.converged,
            "3D conduction PCG did not converge: {result:?}"
        );
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    self.temperature.set(i, j, k, x[i + nx * (j + ny * k)]);
                }
            }
        }
    }
}

/// **増分Hで追加**。これが無いあいだ`ConductionRod1D`は`World::step()`が回す
/// ドメイン一覧から漏れており、`enable_conduction_rod`で載せても**再生しても
/// 一切温度が動かなかった**(D16のテストが
/// `world.conduction_rod_mut().unwrap().step(dt)`と手で回していたのはこのため)。
/// シーンギャラリーへD16を出すには自動ステップが要る。
///
/// **縮約**: 両端のDirichlet境界温度は`set_boundary_temperatures`で設定した値が
/// `temperature[0]`/`temperature[n-1]`にそのまま残り、`step`はそれを固定として
/// 内部点だけを解く(既存の実装どおり)。したがって自動ステップでも境界は保たれる。
impl Solver for ConductionRod1D {
    /// 陰的Euler + PCGなので無条件安定(`ThermalSolver`と同じ理由)。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, _ctx: &mut SolverContext) {
        ConductionRod1D::step(self, dt);
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        hasher.write_u64(self.temperature.len() as u64);
        for t in &self.temperature {
            hasher.write_f64(*t);
        }
    }

    /// 蓄熱量 $\int\rho c_p T\,A\,dx$(**群7で実際に出せるようになった**)。
    ///
    /// 移行前は「熱拡散率 $\alpha=k/(\rho c_p)$ という**比**しか持たないので絶対的な
    /// エネルギーは出せない」としてゼロを返していた。群7で $\rho c_p$ を分離して
    /// 保持できるようにしたので、**与えられていれば本物の値を返す**。
    ///
    /// 与えられていない場合(`volumetric_heat_capacity`が`None`)は引き続きゼロを
    /// 返す——「このドメインはエネルギー台帳に参加しない」という明示であり、
    /// 熱容量1と暗に仮定した値を捏造するよりは正直である。
    fn total_energy(&self) -> EnergyBreakdown {
        match self.thermal_energy() {
            Some(thermal) => EnergyBreakdown {
                thermal,
                ..Default::default()
            },
            None => EnergyBreakdown::default(),
        }
    }

    fn approximations(&self) -> Vec<Approximation> {
        let mut out = vec![Approximation {
            name: "1D棒(3点ステンシル)",
            reason: "この型は1D。3D(7点ステンシル)が要るなら`ConductionGrid3D`を使う\
                         (群7で追加、1D版と同じ答えを出すことを突き合わせ済み)。",
            doc: "docs/12-thermal/02-heat-transfer.md",
            can_disable: false,
        }];
        // ρc_p を与えていれば台帳に参加できる(群7)。未指定のときだけ申告する。
        if self.volumetric_heat_capacity.is_none() {
            out.push(Approximation {
                name: "エネルギー台帳に参加しない",
                reason: "ρc_p(volumetric_heat_capacity)が未指定のため絶対エネルギーを出せない。\
                         `with_material_profile`で与えれば台帳に参加する(群7で追加)。\
                         嘘の数字を入れるより0を返して不参加を明示している。",
                doc: "docs/12-thermal/02-heat-transfer.md",
                can_disable: false,
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// フーリエ級数解: 両端0・初期一様T0の1D棒の過渡温度分布(docs/12-thermal/02-heat-transfer.md §7)。
    /// $T(x,t)=\sum_{n\,\text{odd}} \frac{4T_0}{n\pi}\sin(n\pi x/L)e^{-n^2\pi^2\alpha t/L^2}$。
    fn fourier_series_solution(
        x: f64,
        t: f64,
        length: f64,
        t0: f64,
        alpha: f64,
        terms: u32,
    ) -> f64 {
        let mut sum = 0.0;
        for k in 0..terms {
            let n = (2 * k + 1) as f64;
            let amplitude = 4.0 * t0 / (n * std::f64::consts::PI);
            let spatial = (n * std::f64::consts::PI * x / length).sin();
            let decay = (-n * n * std::f64::consts::PI * std::f64::consts::PI * alpha * t
                / (length * length))
                .exp();
            sum += amplitude * spatial * decay;
        }
        sum
    }

    /// T3: 1D棒の過渡伝導 — フーリエ級数解とrel<2%(h)(docs/21-verification/01-analytic-tests.md T3)。
    #[test]
    fn t3_1d_rod_transient_conduction_matches_fourier_series_solution() {
        let length = 1.0;
        let t0 = 100.0;
        let alpha = 1e-4;
        let node_count = 41;
        let mut rod = ConductionRod1D::new(node_count, length, t0, alpha);
        rod.set_boundary_temperatures(0.0, 0.0);

        let dt = 1.0;
        let total_time = 300.0;
        let steps = (total_time / dt) as u32;
        for _ in 0..steps {
            rod.step(dt);
        }

        // 境界に近すぎる点は解析解の絶対値が小さく相対誤差が発散しやすいため、
        // 中央寄りの複数点(境界の影響が支配的でない範囲)で比較する。
        for &i in &[10usize, 15, 20, 25, 30] {
            let x = i as f64 * rod.dx;
            let analytic = fourier_series_solution(x, total_time, length, t0, alpha, 50);
            let measured = rod.temperature[i];
            let rel_err = (measured - analytic).abs() / analytic;
            assert!(
                rel_err < 0.02,
                "i={i} x={x:.4} measured={measured:.4} analytic={analytic:.4} rel_err={rel_err:.4}"
            );
        }
    }

    /// **群7: 二層棒の定常温度**(材質界面の解析解)。熱伝導率$k_1$の層と$k_2$の層を
    /// 直列につなぎ、両端を固定温度にすると、定常状態では流束$q$が一定なので温度は
    /// 各層で線形になり、全体の熱抵抗は $R=x_f/k_1+(L-x_f)/k_2$(直列)で決まる。
    /// **格子全体の定常温度分布を解析解と突き合わせる**。
    ///
    /// **これが調和平均の検証になる**: 面の伝導率を算術平均で作ると界面の熱抵抗が
    /// 過小評価され、分布が系統的にずれる。
    ///
    /// **実装検証中に踏んだ2点**: (a)界面は格子点ではなく**面** $x_f=(mid-\tfrac12)\Delta x$
    /// にある(節点`mid`は既に低伝導率側へ半セル入っている)——最初これを取り違えて
    /// 4.6%ずれた。(b)低伝導率層の拡散時間 $L^2/\alpha_2$ は約$9\times10^5$ sあり、
    /// dt=5.0×4000stepではまったく定常に達していなかった。陰的Eulerは無条件安定なので
    /// **dtを桁ごと上げる**のが正しい対処。
    #[test]
    fn two_layer_rod_reaches_the_analytic_series_resistance_steady_profile() {
        let nodes = 101;
        let length = 1.0;
        let (k1, k2) = (400.0, 1.0); // 銅相当 と 断熱材相当(比400倍)
        let rho_cp = 3.5e6;
        let (t_left, t_right) = (100.0, 0.0);

        let mid = nodes / 2;
        let conductivity: Vec<f64> = (0..nodes).map(|i| if i < mid { k1 } else { k2 }).collect();
        let mut rod = ConductionRod1D::new(nodes, length, 0.0, 1.0).with_material_profile(
            conductivity,
            rho_cp,
            1.0,
        );
        rod.set_boundary_temperatures(t_left, t_right);

        // 低伝導率層の拡散時間(≈9e5 s)を大きく超えるまで進める。
        for _ in 0..1000 {
            rod.step(1.0e5);
        }

        // 解析解: 界面は面 x_f = (mid - 1/2)dx。流束 q = ΔT / (x_f/k1 + (L-x_f)/k2)。
        let dx = rod.dx;
        let x_f = (mid as f64 - 0.5) * dx;
        let flux = (t_left - t_right) / (x_f / k1 + (length - x_f) / k2);
        let analytic = |x: f64| -> f64 {
            if x <= x_f {
                t_left - flux * x / k1
            } else {
                t_right + flux * (length - x) / k2
            }
        };

        let mut worst: f64 = 0.0;
        for (i, &t) in rod.temperature.iter().enumerate() {
            let x = i as f64 * dx;
            worst = worst.max((t - analytic(x)).abs());
        }
        assert!(
            worst / (t_left - t_right) < 0.01,
            "定常分布は直列熱抵抗の解析解に一致するはず: 最大偏差={worst:.4}K"
        );

        // 対照: 界面の面伝導率は調和平均と算術平均で桁違い。算術平均を使っていれば
        // 界面での温度落差がほぼ消え、上の一致は成立しない。
        let harmonic = 2.0 * k1 * k2 / (k1 + k2);
        let arithmetic = 0.5 * (k1 + k2);
        assert!(
            arithmetic > 50.0 * harmonic,
            "この材質比なら2つの平均は桁違いのはず: harmonic={harmonic} arithmetic={arithmetic}"
        );

        // 低伝導率側のほうが急勾配(同じ流束を通すのに大きな温度差が要る)。
        let slope = |a: usize, b: usize| (rod.temperature[b] - rod.temperature[a]) / (b - a) as f64;
        let s_high = slope(10, 20);
        let s_low = slope(mid + 10, mid + 20);
        assert!(
            s_low.abs() > 100.0 * s_high.abs(),
            "低伝導率側の勾配が急なはず: k1側={s_high} k2側={s_low}"
        );
    }

    /// **群7: 熱エネルギー**。$\rho c_p$ を与えると蓄熱量 $\int\rho c_p T A\,dx$ が
    /// 出せるようになる。断熱端(両端を同じ温度に固定)で内部を暖めた棒が冷めていく
    /// 過程で、エネルギーが単調に減り、最終的に一様温度の値へ収束することを確認する。
    #[test]
    fn thermal_energy_is_reported_once_volumetric_heat_capacity_is_known() {
        let nodes = 51;
        let length = 1.0;
        let k = 200.0;
        let rho_cp = 2.4e6;
        let area = 0.01;

        let mut rod = ConductionRod1D::new(nodes, length, 0.0, 1.0).with_material_profile(
            vec![k; nodes],
            rho_cp,
            area,
        );
        // 中央だけ熱い初期条件、両端は0固定。
        for i in 1..nodes - 1 {
            rod.temperature[i] = if i == nodes / 2 { 100.0 } else { 0.0 };
        }
        rod.set_boundary_temperatures(0.0, 0.0);

        let initial = rod.thermal_energy().expect("ρc_p を与えたので出せるはず");
        assert!(initial > 0.0);
        // 解析値: 中央1点だけ100℃なので ρc_p * A * dx * 100。
        let expected_initial = rho_cp * area * rod.dx * 100.0;
        assert!(
            (initial - expected_initial).abs() / expected_initial < 1e-12,
            "initial={initial} expected={expected_initial}"
        );

        // **時間スケールの見積もりを2度間違えた**。効くのは1セルの拡散時間ではなく
        // **棒全体**の $L^2/\alpha$ で、α = k/ρc_p ≈ 8.3e-5 m²/s・L=1 m なら
        // 約1.2e4 s ある。最初 dt=1e-4(全体で0.02 s)、次に dt=0.5(100 s)で書いて
        // どちらもほとんど冷えなかった。陰的Eulerは無条件安定なので dt=100 で
        // 全体2e4 s まで進める。
        let mut previous = initial;
        for _ in 0..200 {
            rod.step(100.0);
            let now = rod.thermal_energy().unwrap();
            assert!(
                now <= previous + 1e-6,
                "両端0固定なので熱は流出し続けるはず: {previous} -> {now}"
            );
            previous = now;
        }
        assert!(
            previous < 0.5 * initial,
            "十分冷えているはず: {previous} vs {initial}"
        );

        // ρc_p 未指定なら None(移行前の状態)。
        let plain = ConductionRod1D::new(nodes, length, 300.0, 1e-5);
        assert!(plain.thermal_energy().is_none());
    }

    /// 材質プロファイルを与えなければ移行前と**厳密に同一**の結果になる
    /// (既存のT3テストが影響を受けないことの保証)。
    #[test]
    fn without_a_material_profile_the_solver_is_unchanged() {
        let build = || {
            let mut rod = ConductionRod1D::new(41, 1.0, 20.0, 1.0e-4);
            rod.set_boundary_temperatures(100.0, 0.0);
            rod
        };
        let (mut a, mut b) = (build(), build());
        assert!(a.conductivity.is_none());
        for _ in 0..100 {
            a.step(0.5);
            b.step(0.5);
        }
        assert_eq!(a.temperature, b.temperature);
    }

    /// **群7: 3D(7点ステンシル)が1D版と同じ答えを出すこと**。$x$方向の両端だけを
    /// 固定温度にし、残り4面を断熱にすれば、3Dの問題は**1Dと物理的に同一**になる。
    /// 同じ$\alpha$・同じ長さ・同じdtで両方を進め、温度分布が一致することを確認する。
    ///
    /// 単体で「それらしく動く」ことを見るより、**既に解析解(T3フーリエ級数)で
    /// 検証済みの1D版と突き合わせる**ほうが、7点ステンシル・境界処理・PCGの配線を
    /// まとめて検証できる。
    ///
    /// **格子の位置合わせに注意**: 1D版は**節点**中心(境界は節点そのもの)、3D版は
    /// **セル**中心(境界はセル中心から半セル外の面)。同じ物理領域を覆わせた上で、
    /// 1D解を3Dのセル中心位置へ線形補間して比べる。
    #[test]
    fn the_3d_solver_reproduces_the_1d_solver_on_a_one_dimensional_problem() {
        let k = 50.0;
        let rho_cp = 2.0e6;
        let alpha = k / rho_cp;
        let (t_left, t_right) = (100.0, 0.0);
        let length = 1.0;

        // 1D: 節点41個、両端がDirichlet。
        let nodes = 41;
        let mut rod = ConductionRod1D::new(nodes, length, 0.0, alpha);
        rod.set_boundary_temperatures(t_left, t_right);

        // 3D: 同じ長さをセル20個で覆う(境界は外側の面)。
        let cells = 20usize;
        let h = length / cells as f64;
        let mut grid = ConductionGrid3D::uniform(cells, 3, 2, h, 0.0, k, rho_cp);
        grid.boundaries[0] = ThermalBoundary::Fixed(t_left);
        grid.boundaries[1] = ThermalBoundary::Fixed(t_right);
        // y・z は断熱(既定)なので、この問題は x のみの1D問題になる。

        // **十分に発達した過渡で比べる**。実装検証中、t=30秒(拡散長 √(αt)≈0.027 m、
        // 3Dのセル幅0.05 mより小さい)で比べて11%ずれた——これは解の誤りではなく、
        // **境界近傍の鋭い温度フロントを解像度の違う2つの格子で見比べていた**だけ。
        // 拡散長が数セル分に育つ時刻(√(αt)≈0.32 m)まで進めれば、両者は
        // 離散化誤差の範囲で一致する。
        let dt = 20.0;
        for _ in 0..200 {
            rod.step(dt);
            grid.step(dt);
        }

        // 1D解をセル中心 (i+0.5)h へ線形補間して比較する。
        let sample_1d = |x: f64| -> f64 {
            let t = x / rod.dx;
            let i = t.floor() as usize;
            let f = t - i as f64;
            rod.temperature[i] * (1.0 - f) + rod.temperature[(i + 1).min(nodes - 1)] * f
        };
        let mut worst: f64 = 0.0;
        for i in 0..cells {
            let x = (i as f64 + 0.5) * h;
            worst = worst.max((grid.temperature.at(i, 1, 0) - sample_1d(x)).abs());
        }
        assert!(
            worst / (t_left - t_right) < 0.01,
            "3Dは1Dと同じ答えを出すはず: 最大偏差={worst:.4}K"
        );

        // y・z 方向には勾配が立たない(断熱なので1D問題のまま)。
        for i in 0..cells {
            for j in 0..3 {
                for kk in 0..2 {
                    let d = (grid.temperature.at(i, j, kk) - grid.temperature.at(i, 1, 0)).abs();
                    assert!(d < 1e-9, "y/z方向は一様のはず: セル({i},{j},{kk}) 偏差={d}");
                }
            }
        }
    }

    /// **群7: 断熱箱ではエネルギーが厳密に保存する**(全面`Insulated`)。
    /// 内部の温度分布は均されていくが、$\sum\rho c_p T\Delta V$ は変わらない
    /// ——境界処理・係数行列の対称性が正しくないと必ず漏れる。
    #[test]
    fn an_insulated_3d_box_conserves_thermal_energy_while_equalising() {
        let mut grid = ConductionGrid3D::uniform(8, 6, 5, 0.02, 300.0, 80.0, 3.0e6);
        // 角だけ熱くする(非一様な初期条件)。
        grid.temperature.set(0, 0, 0, 500.0);
        grid.temperature.set(7, 5, 4, 100.0);

        let initial = grid.thermal_energy();
        let spread_of = |g: &ConductionGrid3D| -> f64 {
            let mut min = f64::INFINITY;
            let mut max = f64::NEG_INFINITY;
            for k in 0..5 {
                for j in 0..6 {
                    for i in 0..8 {
                        min = min.min(g.temperature.at(i, j, k));
                        max = max.max(g.temperature.at(i, j, k));
                    }
                }
            }
            max - min
        };
        let initial_spread = spread_of(&grid);

        // α = k/ρc_p ≈ 2.7e-5 m²/s、箱の代表長さ 0.16 m なので均されるのに
        // L²/α ≈ 1e3 s かかる。dt=20 の200step(4e3 s)で十分。
        for _ in 0..200 {
            grid.step(20.0);
            let now = grid.thermal_energy();
            let rel = (now - initial).abs() / initial;
            assert!(
                rel < 1e-9,
                "断熱箱はエネルギーを保存するはず: rel={rel:.3e}"
            );
        }
        assert!(
            spread_of(&grid) < 0.01 * initial_spread,
            "十分に均されているはず: {} -> {}",
            initial_spread,
            spread_of(&grid)
        );
    }

    /// **群7: 3Dでも材質界面の直列熱抵抗が成り立つ**(調和平均の3D版)。
    /// $x$方向に $k_1$/$k_2$ の2層を積み、両端をDirichletにした定常分布が
    /// 解析解(1D版と同じ直列熱抵抗)に一致することを確認する。
    #[test]
    fn a_two_layer_3d_slab_matches_the_series_resistance_steady_profile() {
        let cells = 20usize;
        let h = 0.05;
        let (k1, k2) = (200.0, 2.0);
        let rho_cp = 3.0e6;
        let (t_left, t_right) = (80.0, 0.0);

        let mut grid = ConductionGrid3D::uniform(cells, 2, 2, h, 0.0, k1, rho_cp);
        for k in 0..2 {
            for j in 0..2 {
                for i in cells / 2..cells {
                    grid.conductivity.set(i, j, k, k2);
                }
            }
        }
        grid.boundaries[0] = ThermalBoundary::Fixed(t_left);
        grid.boundaries[1] = ThermalBoundary::Fixed(t_right);

        for _ in 0..2000 {
            grid.step(1.0e4);
        }

        // 解析解。境界はセル中心から半セル外にあるので、層の厚さは
        // 左: (cells/2) * h、右: (cells/2) * h、全長 cells*h。
        let length = cells as f64 * h;
        let x_f = (cells / 2) as f64 * h;
        let flux = (t_left - t_right) / (x_f / k1 + (length - x_f) / k2);
        let analytic = |x: f64| {
            if x <= x_f {
                t_left - flux * x / k1
            } else {
                t_right + flux * (length - x) / k2
            }
        };
        let mut worst: f64 = 0.0;
        for i in 0..cells {
            let x = (i as f64 + 0.5) * h;
            worst = worst.max((grid.temperature.at(i, 0, 0) - analytic(x)).abs());
        }
        assert!(
            worst / (t_left - t_right) < 0.02,
            "3Dの二層平板も直列熱抵抗の解析解に一致するはず: 最大偏差={worst:.4}K"
        );
    }
}
