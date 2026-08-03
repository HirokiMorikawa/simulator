//! 格子(Eulerian)流体ソルバ、2D。設計: docs/11-fluid/02-eulerian-grid.md。
//!
//! 完全な `GridFluid`(3D、MAC格子、Solid/Empty境界、渦度強化)ではなく、2D非圧縮流
//! (移流+粘性拡散+圧力投影)に絞った縮約実装。
//!
//! **群7で開境界(流入出)を追加した**(`GridBoundary`)。移行前は**周期境界のみ**で、
//! 「一方から流れ込んで反対側へ抜ける」という最も基本的な流路の構成すら作れなかった
//! (Taylor-Green渦(F8)・投影後発散(F9)の検証に周期境界で足りたため)。群7では
//! `GridBoundary::Channel` を追加し、
//!
//! - **x−側 = 流入**(速度Dirichlet、`inflow_speed`を固定)
//! - **x+側 = 流出**(勾配ゼロ、$\partial u/\partial x=0$ + 圧力Dirichlet $p=0$)
//! - **y側 = 自由すべり壁**($v=0$、$\partial u/\partial y=0$)
//!
//! とした。流出側に圧力Dirichletが入るのでポアソン方程式が**非特異になり**、
//! 周期境界で必要だった「右辺の平均引き」(可解性条件)が不要になる。
//!
//! **群9で渦度強化と任意形状の固体境界を追加した**:
//!
//! - **渦度強化**(設計§4.5、Fedkiw et al. 2001): `vorticity_confinement_epsilon` が
//!   既定 0.0 = 検証モードで、`> 0` のときだけ発動し**近似バッジを申告する**
//!   (設計§4.5「非物理的な補償項であることをUIの近似表示で明示し、検証モードでは
//!   無効化する」)。移流の直後・粘性の前に外力として加える(設計§4.6のステップ順)。
//! - **任意形状の固体境界**: セル種別 `CellType`(設計§3の `cell_type: Grid3<CellType>`
//!   の2D縮約)を持ち、`set_solid_cells` に「座標 → その点の固体速度」を返す閉包を渡す
//!   だけで**形状の種類に依存せず**固体を埋め込める(円柱・多角形・複数個も可)。
//!   移行前は`GridSolidBox`の単一矩形を**投影の後に速度で上書きする**マスキング方式
//!   だったが、群9で**圧力ポアソンの側で Solid 面を Neumann として扱い、Solid セルを
//!   未知数から外す**形(設計§4.4)へ変えた。`pressure_force_on_solid` も
//!   Solid–Fluid 面の一般の面積分 $\mathbf{F}=-\sum p\,\hat{n}\,h$ になった。
//!   `GridSolidBox` は既存の`sim_coupling::GridFluidRigid`結合のために残してあり、
//!   `step`の冒頭でセル種別へラスタライズされる。
//!
//! **残る縮約**: 3D化は引き続き未実装(`GridFluid3D`は群9-4)。`CellType`に `Empty`
//! (自由表面)は持たない——この縮約実装は閉領域/流路の非圧縮流に限っており、自由表面は
//! SPH側(`sph.rs`)が担う。壁は自由すべりで、粘着(no-slip)境界層は作れない
//! ——ポアズイユ流(F7)は専用の`PoiseuilleChannel1D`が別途担っている。
//!
//! 格子は staggered(MAC)配置: 圧力・スカラーはセル中心 $((i+\tfrac12)h,(j+\tfrac12)h)$、
//! `u` は x面 $(ih,(j+\tfrac12)h)$、`v` は y面 $((i+\tfrac12)h,jh)$ に置く。周期境界のため
//! 各成分の格子点数はセル数と同じ($n_x\times n_y$、境界の重複層を持たない)。

use sim_core::{Approximation, EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::Vec3;

/// 単一の矩形剛体をマスキング方式(cut-cell法ではない、`sim_fluid::GridFluidRigidBox2D`
/// (X2)と同じ縮約手法)で格子に埋め込む。`sim_coupling::GridFluidRigid`(設計
/// docs/20-integration/01-coupling-matrix.md §3「P3: 格子流体 ⇔ 剛体(ボクセル化境界・
/// 圧力積分)」)が、`World`のmechanicsボディの位置・速度から毎stepこの値を書き換える。
#[derive(Clone, Copy)]
pub struct GridSolidBox {
    pub center: (f64, f64),
    pub half_width: f64,
    pub half_height: f64,
    pub velocity: Vec3,
}

/// 境界条件(**群7で追加**、モジュールdoc参照)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GridBoundary {
    /// 全方向周期(既定、移行前の唯一の挙動)。F8/F9はこれ。
    Periodic,
    /// 流路: x−から流入・x+へ流出、y方向は自由すべり壁。
    Channel {
        /// 流入面($x=0$)で固定する $u$ [m/s]。
        inflow_speed: f64,
    },
}

/// セル種別(設計§3 `cell_type: Grid3<CellType>` の2D縮約、**群9で追加**)。
/// `Empty`(自由表面)は持たない——モジュールdocの「残る縮約」参照。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellType {
    Fluid,
    Solid,
}

/// 2D格子流体。`u`・`v` は共に長さ `nx*ny`(staggered配置、モジュールdoc参照)。
#[derive(Clone)]
pub struct GridFluid2D {
    pub nx: usize,
    pub ny: usize,
    pub h: f64,
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    /// `Solver::step`が使う既定密度(既存の`project(dt, density)`は引数で個別指定可能、
    /// このフィールドは`World`経由の自動ステップでのみ使われる)。
    pub density: f64,
    /// `Solver::step`が使う既定動粘性係数。0.0なら陽的粘性拡散をスキップする
    /// (設計§4.3: 粘性が無視できるほど小さい場合の既定分岐)。
    pub kinematic_viscosity: f64,
    /// 渦度強化の強さ $\varepsilon_{conf}$(設計§4.5、**群9で追加**)。
    /// **既定 0.0 = 検証モード**(設計§4.5「検証モードでは無効化する」)。
    /// `> 0` にすると非物理的な補償項が入るので、`approximations()` が近似バッジを申告する。
    pub vorticity_confinement_epsilon: f64,
    /// 境界条件(**群7で追加**、モジュールdoc参照)。
    boundary: GridBoundary,
    /// `GridFluidRigid`結合用の単一剛体マスク。`None`なら固体無し。
    /// `set_solid_box`(設定と同時に`cell_type`/`solid_velocity`へラスタライズする)
    /// でのみ書き換える——既存の結合コードのための後方互換経路(モジュールdoc参照)。
    /// 任意形状を使うときは`set_solid_cells`を呼ぶ(そちらは`solid`を`None`にする)。
    solid: Option<GridSolidBox>,
    /// セル種別(**群9で追加**、モジュールdoc参照)。長さ`nx*ny`。
    cell_type: Vec<CellType>,
    /// Solidセルの速度 [m/s](Fluidセルでは未使用)。長さ`nx*ny`。
    solid_velocity: Vec<Vec3>,
    /// 直近の`step`が投影した圧力場(`sim_coupling::GridFluidRigid`の圧力積分抽出専用、
    /// `boundary_force`(`sph.rs`)と同じ理由でpub)。次の`step`呼び出しの冒頭で必ず
    /// 上書きされる導出値(派生キャッシュ)のため、`state_hash`には含めない(スナップショット
    /// 復元後も次の`step`で再計算されるので決定論に影響しない)。
    pub last_pressure: Vec<f64>,
}

fn wrap(i: i64, n: usize) -> usize {
    i.rem_euclid(n as i64) as usize
}

impl GridFluid2D {
    pub fn new(nx: usize, ny: usize, h: f64) -> GridFluid2D {
        GridFluid2D {
            nx,
            ny,
            h,
            u: vec![0.0; nx * ny],
            v: vec![0.0; nx * ny],
            density: 1.0,
            kinematic_viscosity: 0.0,
            vorticity_confinement_epsilon: 0.0,
            solid: None,
            cell_type: vec![CellType::Fluid; nx * ny],
            solid_velocity: vec![Vec3::ZERO; nx * ny],
            last_pressure: vec![0.0; nx * ny],
            boundary: GridBoundary::Periodic,
        }
    }

    /// **任意形状の固体境界を設定する**(**群9で追加**、モジュールdoc参照)。
    /// `f(x, y)` がその点の固体速度を返せばそのセルは `Solid`、`None` なら `Fluid`。
    /// 判定はセル中心 $((i+\frac12)h,(j+\frac12)h)$ で行う(セル単位のラスタライズ、
    /// cut-cell法ではない縮約——半端に切れたセルは丸ごと Solid か Fluid になる)。
    /// `solid`(単一矩形の後方互換経路)はクリアされる。
    pub fn set_solid_cells(&mut self, f: impl Fn(f64, f64) -> Option<Vec3>) {
        self.solid = None;
        for j in 0..self.ny {
            for i in 0..self.nx {
                let x = (i as f64 + 0.5) * self.h;
                let y = (j as f64 + 0.5) * self.h;
                let idx = i + self.nx * j;
                match f(x, y) {
                    Some(velocity) => {
                        self.cell_type[idx] = CellType::Solid;
                        self.solid_velocity[idx] = velocity;
                    }
                    None => {
                        self.cell_type[idx] = CellType::Fluid;
                        self.solid_velocity[idx] = Vec3::ZERO;
                    }
                }
            }
        }
    }

    /// セル種別(読み取り)。範囲外は `Fluid` 扱い(周期境界では`wrap`される)。
    pub fn cell_type_at(&self, i: i64, j: i64) -> CellType {
        self.cell_type[self.idx(i, j)]
    }

    fn is_solid(&self, i: i64, j: i64) -> bool {
        self.cell_type_at(i, j) == CellType::Solid
    }

    fn has_solid(&self) -> bool {
        self.cell_type.contains(&CellType::Solid)
    }

    /// 単一矩形の固体マスクを設定する(後方互換経路、`solid`フィールドのdoc参照)。
    /// 設定と同時にセル種別へラスタライズするので、`step`を呼ぶ前でも
    /// `pressure_force_on_solid`が正しい面を選べる。
    pub fn set_solid_box(&mut self, solid: Option<GridSolidBox>) {
        self.solid = solid;
        match solid {
            Some(_) => self.rasterize_solid_box(),
            None => {
                for k in 0..self.cell_type.len() {
                    self.cell_type[k] = CellType::Fluid;
                    self.solid_velocity[k] = Vec3::ZERO;
                }
            }
        }
    }

    /// 設定されている単一矩形の固体マスク(後方互換経路)。
    pub fn solid_box(&self) -> Option<GridSolidBox> {
        self.solid
    }

    /// `solid`(単一矩形)をセル種別へラスタライズする(後方互換経路、モジュールdoc参照)。
    fn rasterize_solid_box(&mut self) {
        let Some(solid) = self.solid else {
            return;
        };
        for j in 0..self.ny {
            for i in 0..self.nx {
                let x = (i as f64 + 0.5) * self.h;
                let y = (j as f64 + 0.5) * self.h;
                let idx = i + self.nx * j;
                if Self::is_solid_at(&solid, x, y) {
                    self.cell_type[idx] = CellType::Solid;
                    self.solid_velocity[idx] = solid.velocity;
                } else {
                    self.cell_type[idx] = CellType::Fluid;
                    self.solid_velocity[idx] = Vec3::ZERO;
                }
            }
        }
    }

    /// 境界条件を差し替える(**群7で追加**、モジュールdoc参照)。`Channel`にすると
    /// 流入面の $u$ を`inflow_speed`で初期化する(そうしないと最初の数stepだけ
    /// 静止流体が流入してくる形になり、定常へ落ち着くまで余計な過渡が乗る)。
    pub fn with_boundary(mut self, boundary: GridBoundary) -> GridFluid2D {
        self.boundary = boundary;
        if let GridBoundary::Channel { inflow_speed } = boundary {
            for value in self.u.iter_mut() {
                *value = inflow_speed;
            }
            for value in self.v.iter_mut() {
                *value = 0.0;
            }
        }
        self
    }

    /// 現在の境界条件。
    pub fn boundary(&self) -> GridBoundary {
        self.boundary
    }

    /// x面の $u$(境界条件を考慮、**群7**)。`i`は面index(`0..=nx`)。
    /// `Channel`では `i=0` が流入面(固定値)、`i>=nx` は流出(勾配ゼロ = 最終列の複製)。
    fn u_face(&self, i: i64, j: i64) -> f64 {
        match self.boundary {
            GridBoundary::Periodic => self.u_at(i, j),
            GridBoundary::Channel { inflow_speed } => {
                let jj = j.clamp(0, self.ny as i64 - 1); // y壁は勾配ゼロ(自由すべり)
                if i <= 0 {
                    inflow_speed
                } else if i >= self.nx as i64 {
                    self.u[(self.nx - 1) + self.nx * jj as usize]
                } else {
                    self.u[i as usize + self.nx * jj as usize]
                }
            }
        }
    }

    /// y面の $v$(境界条件を考慮、**群7**)。`j`は面index(`0..=ny`)。
    /// `Channel`では `j=0` と `j=ny` が壁なので $v=0$(自由すべり)。
    fn v_face(&self, i: i64, j: i64) -> f64 {
        match self.boundary {
            GridBoundary::Periodic => self.v_at(i, j),
            GridBoundary::Channel { .. } => {
                if j <= 0 || j >= self.ny as i64 {
                    0.0
                } else {
                    let ii = i.clamp(0, self.nx as i64 - 1);
                    self.v[ii as usize + self.nx * j as usize]
                }
            }
        }
    }

    fn idx(&self, i: i64, j: i64) -> usize {
        wrap(i, self.nx) + self.nx * wrap(j, self.ny)
    }

    pub fn u_at(&self, i: i64, j: i64) -> f64 {
        self.u[self.idx(i, j)]
    }

    pub fn v_at(&self, i: i64, j: i64) -> f64 {
        self.v[self.idx(i, j)]
    }

    /// セル(i,j)の発散(中心差分、MAC格子の標準式、設計§4.4)。
    pub fn divergence(&self, i: i64, j: i64) -> f64 {
        (self.u_face(i + 1, j) - self.u_face(i, j)) / self.h
            + (self.v_face(i, j + 1) - self.v_face(i, j)) / self.h
    }

    /// 双線形補間(周期境界、モジュールdocのstaggered配置に対応する`offset`を使う)。
    fn sample_periodic(data: &[f64], nx: usize, ny: usize, h: f64, offset: Vec3, pos: Vec3) -> f64 {
        let local_x = (pos.x - offset.x) / h;
        let local_y = (pos.y - offset.y) / h;
        let i0f = local_x.floor();
        let j0f = local_y.floor();
        let fx = local_x - i0f;
        let fy = local_y - j0f;
        let i0 = i0f as i64;
        let j0 = j0f as i64;
        let get = |ii: i64, jj: i64| -> f64 { data[wrap(ii, nx) + nx * wrap(jj, ny)] };
        let v00 = get(i0, j0);
        let v10 = get(i0 + 1, j0);
        let v01 = get(i0, j0 + 1);
        let v11 = get(i0 + 1, j0 + 1);
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    /// 双線形補間(開境界版、**群7**)。周期のラップアラウンドの代わりに
    /// `u_face`/`v_face`(境界条件を知っている面アクセサ)から値を引く。
    /// これが無いと、流出面を跨いでバックトレースしたレイが**反対側の流入面から
    /// 値を拾ってくる**(周期の折り返し)ことになり、開境界が成立しない。
    fn sample_bounded(&self, is_u: bool, offset: Vec3, pos: Vec3) -> f64 {
        let local_x = (pos.x - offset.x) / self.h;
        let local_y = (pos.y - offset.y) / self.h;
        let i0f = local_x.floor();
        let j0f = local_y.floor();
        let fx = local_x - i0f;
        let fy = local_y - j0f;
        let (i0, j0) = (i0f as i64, j0f as i64);
        let get = |ii: i64, jj: i64| -> f64 {
            if is_u {
                self.u_face(ii, jj)
            } else {
                self.v_face(ii, jj)
            }
        };
        let v00 = get(i0, j0);
        let v10 = get(i0 + 1, j0);
        let v01 = get(i0, j0 + 1);
        let v11 = get(i0 + 1, j0 + 1);
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    fn sample_u(&self, pos: Vec3) -> f64 {
        // u[i][j] は (i*h, (j+0.5)*h) に位置する。
        let offset = Vec3::new(0.0, 0.5 * self.h, 0.0);
        match self.boundary {
            GridBoundary::Periodic => {
                Self::sample_periodic(&self.u, self.nx, self.ny, self.h, offset, pos)
            }
            GridBoundary::Channel { .. } => self.sample_bounded(true, offset, pos),
        }
    }

    fn sample_v(&self, pos: Vec3) -> f64 {
        // v[i][j] は ((i+0.5)*h, j*h) に位置する。
        let offset = Vec3::new(0.5 * self.h, 0.0, 0.0);
        match self.boundary {
            GridBoundary::Periodic => {
                Self::sample_periodic(&self.v, self.nx, self.ny, self.h, offset, pos)
            }
            GridBoundary::Channel { .. } => self.sample_bounded(false, offset, pos),
        }
    }

    fn velocity_at(&self, pos: Vec3) -> Vec3 {
        Vec3::new(self.sample_u(pos), self.sample_v(pos), 0.0)
    }

    /// semi-Lagrangian移流(RK2中点法によるバックトレース、設計§4.1)。
    pub fn advect_velocity(&mut self, dt: f64) {
        let old_u = self.u.clone();
        let old_v = self.v.clone();
        let old = GridFluid2D {
            u: old_u,
            v: old_v,
            ..self.clone()
        };

        for j in 0..self.ny as i64 {
            for i in 0..=self.nx as i64 {
                let i_wrapped = i % self.nx as i64;
                let pos = Vec3::new(i as f64 * self.h, (j as f64 + 0.5) * self.h, 0.0);
                let vel = old.velocity_at(pos);
                let mid = pos - vel.scale(0.5 * dt);
                let vel_mid = old.velocity_at(mid);
                let src = pos - vel_mid.scale(dt);
                let idx = wrap(i_wrapped, self.nx) + self.nx * wrap(j, self.ny);
                self.u[idx] = old.sample_u(src);
            }
        }
        for j in 0..=self.ny as i64 {
            for i in 0..self.nx as i64 {
                let j_wrapped = j % self.ny as i64;
                let pos = Vec3::new((i as f64 + 0.5) * self.h, j as f64 * self.h, 0.0);
                let vel = old.velocity_at(pos);
                let mid = pos - vel.scale(0.5 * dt);
                let vel_mid = old.velocity_at(mid);
                let src = pos - vel_mid.scale(dt);
                let idx = wrap(i, self.nx) + self.nx * wrap(j_wrapped, self.ny);
                self.v[idx] = old.sample_v(src);
            }
        }
    }

    /// 陽的粘性拡散(5点ラプラシアン、周期境界、設計§4.3)。
    pub fn diffuse_explicit(&mut self, dt: f64, kinematic_viscosity: f64) {
        let coeff = kinematic_viscosity * dt / (self.h * self.h);
        let old_u = self.u.clone();
        let old_v = self.v.clone();
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                let idx = self.idx(i, j);
                let lap = old_u[self.idx(i + 1, j)]
                    + old_u[self.idx(i - 1, j)]
                    + old_u[self.idx(i, j + 1)]
                    + old_u[self.idx(i, j - 1)]
                    - 4.0 * old_u[idx];
                self.u[idx] += coeff * lap;

                let lap_v = old_v[self.idx(i + 1, j)]
                    + old_v[self.idx(i - 1, j)]
                    + old_v[self.idx(i, j + 1)]
                    + old_v[self.idx(i, j - 1)]
                    - 4.0 * old_v[idx];
                self.v[idx] += coeff * lap_v;
            }
        }
    }

    fn is_solid_at(solid: &GridSolidBox, x: f64, y: f64) -> bool {
        (x - solid.center.0).abs() < solid.half_width
            && (y - solid.center.1).abs() < solid.half_height
    }

    /// Solid セルに触れる面の法線速度を固体の速度に一致させる(設計§4.4
    /// 「Solidセル面: $\mathbf{u}\cdot\hat n=\mathbf{u}_{solid}\cdot\hat n$」)。
    /// **群9でセル種別ベースへ一般化**(移行前は`GridSolidBox`の矩形内外判定だった)。
    /// x面 `i` はセル `(i-1,j)` と `(i,j)` の間にある。
    fn enforce_solid_faces(&mut self) {
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                // x面 i(セル i-1 と i の間)。
                let left_solid = self.is_solid(i - 1, j);
                let right_solid = self.is_solid(i, j);
                if left_solid || right_solid {
                    let source = if right_solid {
                        self.solid_velocity[self.idx(i, j)]
                    } else {
                        self.solid_velocity[self.idx(i - 1, j)]
                    };
                    let idx = self.idx(i, j);
                    self.u[idx] = source.x;
                }
                // y面 j(セル j-1 と j の間)。
                let below_solid = self.is_solid(i, j - 1);
                let above_solid = self.is_solid(i, j);
                if below_solid || above_solid {
                    let source = if above_solid {
                        self.solid_velocity[self.idx(i, j)]
                    } else {
                        self.solid_velocity[self.idx(i, j - 1)]
                    };
                    let idx = self.idx(i, j);
                    self.v[idx] = source.y;
                }
            }
        }
    }

    /// 固体表面の圧力積分による流体力(設計 docs/11-fluid/02-eulerian-grid.md §6
    /// 「流体→剛体: 剛体表面セルの圧力を面積分」)。固体セルが1つも無ければ `None`。
    ///
    /// **群9で任意形状に対応する一般の面積分へ置き換えた**:
    /// $\mathbf{F}=-\sum_{\text{Solid–Fluid 面}} p\,\hat n\,h$($\hat n$ は固体から
    /// 外向き)。移行前は「矩形を囲むバウンディングindexを走査する」実装で、
    /// **矩形以外の形では正しい面を選べなかった**。粘性せん断は省略(設計§6が明記する
    /// 誤差要因)。
    pub fn pressure_force_on_solid(&self) -> Option<Vec3> {
        if !self.has_solid() {
            return None;
        }
        let mut fx = 0.0;
        let mut fy = 0.0;
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                // x面 i(セル (i-1,j) と (i,j) の間)。片側だけが Solid のときが固体表面。
                let left_solid = self.is_solid(i - 1, j);
                let right_solid = self.is_solid(i, j);
                if left_solid != right_solid {
                    if right_solid {
                        // 固体は右。外向き法線は -x。dF_x = -p·(-1)·h = +p·h。
                        fx += self.last_pressure[self.idx(i - 1, j)] * self.h;
                    } else {
                        // 固体は左。外向き法線は +x。
                        fx -= self.last_pressure[self.idx(i, j)] * self.h;
                    }
                }
                // y面 j(セル (i,j-1) と (i,j) の間)。
                let below_solid = self.is_solid(i, j - 1);
                let above_solid = self.is_solid(i, j);
                if below_solid != above_solid {
                    if above_solid {
                        fy += self.last_pressure[self.idx(i, j - 1)] * self.h;
                    } else {
                        fy -= self.last_pressure[self.idx(i, j)] * self.h;
                    }
                }
            }
        }
        Some(Vec3::new(fx, fy, 0.0))
    }

    /// 圧力投影(設計§4.4): ポアソン方程式 $\nabla^2p=\frac{\rho}{\Delta t}\nabla\cdot u^*$ を
    /// matrix-free PCGで解き、$u^{n+1}=u^*-\frac{\Delta t}{\rho}\nabla p$ を適用する。
    /// 周期境界ではラプラシアンが特異(定数関数が零空間)なため、右辺の平均をあらかじめ
    /// 引いて可解性条件を満たす(標準的な周期ポアソン解法のテクニック)。圧力場自体を返す
    /// (`GridFluidRigid`の圧力積分抽出に使う、既存呼び出し元は戻り値を無視すればよい)。
    pub fn project(&mut self, dt: f64, density: f64) -> Vec<f64> {
        let n = self.nx * self.ny;
        // Solidセルは未知数から外す(恒等行 + rhs=0、設計§4.4)。**群9で追加**。
        let solid_cell: Vec<bool> = (0..n)
            .map(|k| self.cell_type[k] == CellType::Solid)
            .collect();
        let mut rhs = vec![0.0; n];
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                let idx = self.idx(i, j);
                if solid_cell[idx] {
                    continue; // 恒等行
                }
                rhs[idx] = density / dt * self.divergence(i, j);
            }
        }
        // 流出列は圧力Dirichlet p=0(上の`apply_a`の恒等行と対にする)。
        if let GridBoundary::Channel { .. } = self.boundary {
            for j in 0..self.ny {
                rhs[(self.nx - 1) + self.nx * j] = 0.0;
            }
        }
        // 周期境界ではラプラシアンが特異(定数関数が零空間)なので右辺の平均を引く。
        // **開境界(Channel)では流出面に圧力Dirichlet $p=0$ が入るため非特異**で、
        // 平均引きは不要どころか**やってはいけない**(可解な系の右辺を歪める)。
        //
        // **群9**: Solidセルを恒等行に落としたので、可解性条件の平均は**Fluidセルだけ**で
        // 取る(Solid行のrhsは0固定であり、平均引きの対象に含めると流体側の右辺を歪める)。
        if self.boundary == GridBoundary::Periodic {
            let fluid_count = solid_cell.iter().filter(|s| !**s).count();
            if fluid_count > 0 {
                let mean: f64 = rhs
                    .iter()
                    .zip(solid_cell.iter())
                    .filter(|(_, s)| !**s)
                    .map(|(r, _)| *r)
                    .sum::<f64>()
                    / fluid_count as f64;
                for (r, s) in rhs.iter_mut().zip(solid_cell.iter()) {
                    if !*s {
                        *r -= mean;
                    }
                }
            }
        }

        let nx = self.nx;
        let ny = self.ny;
        let h2 = self.h * self.h;
        let boundary = self.boundary;
        let apply_a = |x: &[f64], out: &mut [f64]| {
            for j in 0..ny as i64 {
                for i in 0..nx as i64 {
                    let idx = wrap(i, nx) + nx * wrap(j, ny);
                    // Solidセルは恒等行(未知数から外す、設計§4.4)。**群9で追加**。
                    if solid_cell[idx] {
                        out[idx] = x[idx];
                        continue;
                    }
                    match boundary {
                        GridBoundary::Periodic => {
                            // Solid隣接面はNeumann(∂p/∂n=0)なので、その方向は鏡像に
                            // なり寄与が相殺する = 対角の係数がその方向ぶん減る。
                            let mut sum = 0.0;
                            let mut diag = 0.0;
                            let mut neighbour = |ii: i64, jj: i64| {
                                let k = wrap(ii, nx) + nx * wrap(jj, ny);
                                if solid_cell[k] {
                                    return;
                                }
                                sum += x[k];
                                diag += 1.0;
                            };
                            neighbour(i + 1, j);
                            neighbour(i - 1, j);
                            neighbour(i, j + 1);
                            neighbour(i, j - 1);
                            out[idx] = (sum - diag * x[idx]) / h2;
                        }
                        GridBoundary::Channel { .. } => {
                            // 流出列(i = nx-1)は圧力Dirichlet p=0。恒等行にして
                            // 未知数から実質的に外す(rhsも0にしてある)。
                            if i == nx as i64 - 1 {
                                out[idx] = x[idx];
                                continue;
                            }
                            // 流入面(i=0の左)・y壁・Solid隣接面 はNeumann(∂p/∂n=0)なので、
                            // 隣接セルが領域外/固体なら**自分自身を鏡像として使う**
                            // ——その結果、対角の係数がその方向ぶん減る。
                            let mut sum = 0.0;
                            let mut diag = 0.0;
                            let mut neighbour = |ii: i64, jj: i64| {
                                if ii < 0 || ii >= nx as i64 || jj < 0 || jj >= ny as i64 {
                                    return; // Neumann: 鏡像なので寄与が相殺する。
                                }
                                let k = ii as usize + nx * jj as usize;
                                if solid_cell[k] {
                                    return; // Solid面もNeumann(**群9**)。
                                }
                                sum += x[k];
                                diag += 1.0;
                            };
                            neighbour(i + 1, j);
                            neighbour(i - 1, j);
                            neighbour(i, j + 1);
                            neighbour(i, j - 1);
                            out[idx] = (sum - diag * x[idx]) / h2;
                        }
                    }
                }
            }
        };

        let mut pressure = vec![0.0; n];
        let result = sim_math::pcg(
            apply_a,
            &rhs,
            &mut pressure,
            &sim_math::Preconditioner::None,
            1e-8,
            2000,
        );
        debug_assert!(
            result.converged,
            "pressure projection PCG did not converge: {result:?}"
        );

        let scale = dt / density;
        // **群9**: Solid に接する面は補正しない(そこは圧力勾配ではなく固体速度が決める。
        // 補正すると Neumann 条件を自分で壊すことになる)。補正後に `enforce_solid_faces`
        // で改めて固体速度へ戻す。
        let x_face_is_free = |i: i64, j: i64| -> bool {
            !solid_cell[wrap(i, nx) + nx * wrap(j, ny)]
                && !solid_cell[wrap(i - 1, nx) + nx * wrap(j, ny)]
        };
        let y_face_is_free = |i: i64, j: i64| -> bool {
            !solid_cell[wrap(i, nx) + nx * wrap(j, ny)]
                && !solid_cell[wrap(i, nx) + nx * wrap(j - 1, ny)]
        };
        match self.boundary {
            GridBoundary::Periodic => {
                for j in 0..self.ny as i64 {
                    for i in 0..=self.nx as i64 {
                        if !x_face_is_free(i, j) {
                            continue;
                        }
                        let ip = wrap(i, nx) + nx * wrap(j, ny);
                        let im = wrap(i - 1, nx) + nx * wrap(j, ny);
                        let dpdx = (pressure[ip] - pressure[im]) / self.h;
                        let idx = wrap(i, self.nx) + self.nx * wrap(j, self.ny);
                        self.u[idx] -= scale * dpdx;
                    }
                }
                for j in 0..=self.ny as i64 {
                    for i in 0..self.nx as i64 {
                        if !y_face_is_free(i, j) {
                            continue;
                        }
                        let jp = wrap(i, nx) + nx * wrap(j, ny);
                        let jm = wrap(i, nx) + nx * wrap(j - 1, ny);
                        let dpdy = (pressure[jp] - pressure[jm]) / self.h;
                        let idx = wrap(i, self.nx) + self.nx * wrap(j, self.ny);
                        self.v[idx] -= scale * dpdy;
                    }
                }
            }
            GridBoundary::Channel { .. } => {
                // 内部の面だけを補正する。流入面(i=0)は速度Dirichletなので触らない、
                // y壁(j=0)は v=0 のまま。
                for j in 0..self.ny as i64 {
                    for i in 1..self.nx as i64 {
                        if !x_face_is_free(i, j) {
                            continue;
                        }
                        let (i, j) = (i as usize, j as usize);
                        let dpdx = (pressure[i + nx * j] - pressure[(i - 1) + nx * j]) / self.h;
                        self.u[i + nx * j] -= scale * dpdx;
                    }
                }
                for j in 1..self.ny as i64 {
                    for i in 0..self.nx as i64 {
                        if !y_face_is_free(i, j) {
                            continue;
                        }
                        let (i, j) = (i as usize, j as usize);
                        let dpdy = (pressure[i + nx * j] - pressure[i + nx * (j - 1)]) / self.h;
                        self.v[i + nx * j] -= scale * dpdy;
                    }
                }
                // 流入面と壁を境界値へ戻す(投影が触っていないので実質的な再確認)。
                if let GridBoundary::Channel { inflow_speed } = self.boundary {
                    for j in 0..self.ny {
                        self.u[nx * j] = inflow_speed;
                        self.v[nx * j] = 0.0;
                    }
                }
            }
        }

        pressure
    }

    /// 全格子点での速度の最大値(`max_stable_dt`の移流CFL項が使う)。
    fn max_speed(&self) -> f64 {
        let mut max_sq: f64 = 0.0;
        for i in 0..self.u.len() {
            let speed_sq = self.u[i] * self.u[i] + self.v[i] * self.v[i];
            max_sq = max_sq.max(speed_sq);
        }
        max_sq.sqrt()
    }

    /// セル中心 $((i+\frac12)h,(j+\frac12)h)$ の渦度 $\omega_z=\partial v/\partial x-
    /// \partial u/\partial y$(設計§4.5)。面アクセサ経由で読むので境界条件を尊重する。
    fn vorticity_at(&self, i: i64, j: i64) -> f64 {
        let u_center = |ii: i64, jj: i64| 0.5 * (self.u_face(ii, jj) + self.u_face(ii + 1, jj));
        let v_center = |ii: i64, jj: i64| 0.5 * (self.v_face(ii, jj) + self.v_face(ii, jj + 1));
        let dvdx = (v_center(i + 1, j) - v_center(i - 1, j)) / (2.0 * self.h);
        let dudy = (u_center(i, j + 1) - u_center(i, j - 1)) / (2.0 * self.h);
        dvdx - dudy
    }

    /// **渦度強化**(設計§4.5、Fedkiw et al. 2001、**群9で追加**):
    /// $\mathbf{N}=\nabla|\omega|/|\nabla|\omega||$ として
    /// $\mathbf{f}_{conf}=\varepsilon_{conf}\,h\,(\mathbf{N}\times\boldsymbol\omega)$。
    /// 2Dでは $\boldsymbol\omega=\omega\hat z$ なので
    /// $f_x=\varepsilon h N_y\omega$、$f_y=-\varepsilon h N_x\omega$。
    ///
    /// **非物理的な補償項**なので、`epsilon > 0` のときは `approximations()` が
    /// 近似バッジを申告する(設計§4.5「UIの近似表示で明示し、検証モードでは無効化する」)。
    /// 投影より前に加えるので非圧縮性は壊れない。
    pub fn apply_vorticity_confinement(&mut self, dt: f64, epsilon: f64) {
        if epsilon == 0.0 {
            return;
        }
        let (nx, ny) = (self.nx, self.ny);
        let mut abs_omega = vec![0.0; nx * ny];
        for j in 0..ny as i64 {
            for i in 0..nx as i64 {
                abs_omega[self.idx(i, j)] = self.vorticity_at(i, j).abs();
            }
        }
        // 加算は元の速度場から独立に決まるようまとめて計算してから適用する。
        let mut du = vec![0.0; nx * ny];
        let mut dv = vec![0.0; nx * ny];
        for j in 0..ny as i64 {
            for i in 0..nx as i64 {
                if self.is_solid(i, j) {
                    continue; // 固体セルには外力を入れない
                }
                let gx = (abs_omega[self.idx(i + 1, j)] - abs_omega[self.idx(i - 1, j)])
                    / (2.0 * self.h);
                let gy = (abs_omega[self.idx(i, j + 1)] - abs_omega[self.idx(i, j - 1)])
                    / (2.0 * self.h);
                let magnitude = (gx * gx + gy * gy).sqrt();
                if magnitude < 1e-12 {
                    continue; // 勾配ゼロ(一様な渦度)では方向が定まらない
                }
                let (n_x, n_y) = (gx / magnitude, gy / magnitude);
                let omega = self.vorticity_at(i, j);
                let force_x = epsilon * self.h * n_y * omega;
                let force_y = -epsilon * self.h * n_x * omega;
                // セル中心の力を、そのセルを挟む2面へ半分ずつ配る。
                du[self.idx(i, j)] += 0.5 * force_x * dt;
                du[self.idx(i + 1, j)] += 0.5 * force_x * dt;
                dv[self.idx(i, j)] += 0.5 * force_y * dt;
                dv[self.idx(i, j + 1)] += 0.5 * force_y * dt;
            }
        }
        for k in 0..nx * ny {
            self.u[k] += du[k];
            self.v[k] += dv[k];
        }
    }

    /// `Solver::step`が呼ぶ1ステップ分の処理。設計§4.6のステップまとめの順序
    /// **移流 → 外力(渦度強化)→ 粘性 → 境界条件 → 投影** に従う。
    /// 煙/温度の移流(§4.2, §4.6)はこの縮約実装の対象外。
    /// 固体面の速度矯正は投影の前後両方に適用する(投影前は境界条件として、投影後は
    /// 丸め誤差で漏れた分の再矯正——`GridFluidRigidBox2D::step`と同じ理由)。
    pub fn step(&mut self, dt: f64) {
        self.rasterize_solid_box();
        self.advect_velocity(dt);
        self.apply_vorticity_confinement(dt, self.vorticity_confinement_epsilon);
        if self.kinematic_viscosity > 0.0 {
            self.diffuse_explicit(dt, self.kinematic_viscosity);
        }
        self.enforce_solid_faces();
        self.last_pressure = self.project(dt, self.density);
        self.enforce_solid_faces();
    }
}

impl Solver for GridFluid2D {
    /// 設計§4.3の陽的粘性の安定限界 $\nu\Delta t/h^2 \le 0.25$ と、§4.6が定める
    /// 移流のCFL規約(CFL≦5)の両方から決まる、より厳しい方を返す。半Lagrangian移流
    /// 自体は無条件安定(§4.1)なのでCFL項は「妥当な補間精度を保つための目安」であり、
    /// 厳密な安定限界ではないが、`Orchestrator`のsub-step決定に使う値として一貫させる。
    fn max_stable_dt(&self) -> f64 {
        const ADVECTION_CFL: f64 = 5.0;
        let speed = self.max_speed();
        let dt_adv = if speed > 0.0 {
            ADVECTION_CFL * self.h / speed
        } else {
            f64::INFINITY
        };
        let dt_visc = if self.kinematic_viscosity > 0.0 {
            0.25 * self.h * self.h / self.kinematic_viscosity
        } else {
            f64::INFINITY
        };
        dt_adv.min(dt_visc)
    }

    fn step(&mut self, dt: f64, _ctx: &mut SolverContext) {
        // inherent メソッド(1引数版、上の`impl GridFluid2D`ブロック)が同名のトレイト
        // メソッドより優先されるため無限再帰しない(`sim_em::Circuit`・`SphFluid`と同じ
        // パターン)。
        self.step(dt);
    }

    /// 運動エネルギーのみ(非圧縮流は圧力によるポテンシャルエネルギーを持たず、
    /// 外力由来のポテンシャルはこの縮約実装が外力自体を扱わないため対象外)。
    fn total_energy(&self) -> EnergyBreakdown {
        let cell_mass = self.density * self.h * self.h;
        let mut kinetic = 0.0;
        for i in 0..self.u.len() {
            kinetic += 0.5 * cell_mass * (self.u[i] * self.u[i] + self.v[i] * self.v[i]);
        }
        EnergyBreakdown {
            kinetic,
            ..Default::default()
        }
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        hasher.write_u64(self.u.len() as u64);
        for i in 0..self.u.len() {
            hasher.write_f64(self.u[i]);
            hasher.write_f64(self.v[i]);
        }
        // `solid`は次stepの挙動に影響する状態(`last_pressure`と異なり、次stepの冒頭で
        // 再計算される派生値ではない)なのでハッシュに含める(決定論replayの一部)。
        match self.solid {
            Some(solid) => {
                hasher.write_u64(1);
                hasher.write_f64(solid.center.0);
                hasher.write_f64(solid.center.1);
                hasher.write_f64(solid.half_width);
                hasher.write_f64(solid.half_height);
                hasher.write_f64(solid.velocity.x);
                hasher.write_f64(solid.velocity.y);
                hasher.write_f64(solid.velocity.z);
            }
            None => hasher.write_u64(0),
        }
        // 任意形状の固体境界(**群9**)。`solid`(単一矩形)を使わず`set_solid_cells`で
        // 直接設定された場合はこちらだけが状態を持つので、同様にハッシュへ含める。
        for k in 0..self.cell_type.len() {
            hasher.write_u64(if self.cell_type[k] == CellType::Solid {
                1
            } else {
                0
            });
            if self.cell_type[k] == CellType::Solid {
                hasher.write_vec3(self.solid_velocity[k]);
            }
        }
        hasher.write_f64(self.vorticity_confinement_epsilon);
    }

    fn approximations(&self) -> Vec<Approximation> {
        let mut out = vec![Approximation {
            name: "semi-Lagrangian移流",
            reason: "無条件安定だが数値拡散が大きい(低粘性域では真の粘性より拡散的)。",
            doc: "docs/11-fluid/02-eulerian-grid.md",
            can_disable: false,
        }];
        // 境界の申告は設定に応じて変える(**群9**。移行前は「2D・周期境界」固定文言で、
        // 群7で`Channel`を、群9で固体境界を足したあとも嘘のまま残っていた)。
        out.push(match self.boundary {
            GridBoundary::Periodic => Approximation {
                name: "2D・周期境界",
                reason: "全方向が周期境界のため、下流の後流が上流へ回り込む。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            },
            GridBoundary::Channel { .. } => Approximation {
                name: "2D・開境界(流路)",
                reason: "x−から流入・x+へ流出、y壁は自由すべり。粘着(no-slip)境界層は作れない。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            },
        });
        if self.has_solid() {
            out.push(Approximation {
                name: "セル単位の固体境界",
                reason: "固体はセル中心の内外判定でラスタライズする(cut-cell法ではないため、\
                         半端に切れたセルは丸ごと固体か流体になり表面が階段状になる)。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: false,
            });
        }
        if self.vorticity_confinement_epsilon > 0.0 {
            // 設計§4.5「**非物理的な補償項**であることをUIの近似表示で明示し、
            // 検証モードでは無効化する」。既定(0.0)ではこのバッジは出ない。
            out.push(Approximation {
                name: "渦度強化(非物理)",
                reason: "数値拡散で失われた小渦を補償する非物理的な外力を加えている\
                         (Fedkiw et al. 2001)。検証時は epsilon=0 にすること。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            });
        }
        if self.kinematic_viscosity == 0.0 {
            // **設定によって効いている近似が変わる例**。粘性0なら陽的粘性拡散を
            // 丸ごとスキップするので、その事実を申告する(Worldの側からドメインの
            // 有無だけで推測していたときには表現できなかった情報)。
            out.push(Approximation {
                name: "粘性拡散をスキップ",
                reason: "kinematic_viscosity=0 のため陽的粘性拡散の段を実行していない。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// F9: 投影後発散 — 任意の(非零発散の)速度場を1回投影すると|∇·u| < 1e-6になること
    /// (docs/21-verification/01-analytic-tests.md F9)。
    #[test]
    fn f9_divergence_after_single_projection_is_near_zero() {
        let nx = 16;
        let ny = 16;
        let h = 1.0 / nx as f64;
        let mut fluid = GridFluid2D::new(nx, ny, h);

        // 非発散フリーな適当な速度場(正弦波、周期境界と整合する波数)。
        for j in 0..ny as i64 {
            for i in 0..=nx as i64 {
                let idx = wrap(i, nx) + nx * wrap(j, ny);
                let x = i as f64 * h;
                let y = (j as f64 + 0.5) * h;
                fluid.u[idx] =
                    (2.0 * std::f64::consts::PI * x).sin() * (2.0 * std::f64::consts::PI * y).cos();
            }
        }
        for j in 0..=ny as i64 {
            for i in 0..nx as i64 {
                let idx = wrap(i, nx) + nx * wrap(j, ny);
                let x = (i as f64 + 0.5) * h;
                let y = j as f64 * h;
                fluid.v[idx] =
                    (2.0 * std::f64::consts::PI * x).cos() * (2.0 * std::f64::consts::PI * y).sin();
            }
        }

        fluid.project(0.01, 1.0);

        let mut max_div: f64 = 0.0;
        for j in 0..ny as i64 {
            for i in 0..nx as i64 {
                max_div = max_div.max(fluid.divergence(i, j).abs());
            }
        }
        assert!(max_div < 1e-6, "max_div={max_div:e}");
    }

    /// F8: Taylor-Green渦の減衰率が解析式 $e^{-2\nu k^2t}$ と一致すること
    /// (docs/21-verification/01-analytic-tests.md F8)。厳密解 $u=-\cos(kx)\sin(ky)e^{-2\nu k^2t}$、
    /// $v=\sin(kx)\cos(ky)e^{-2\nu k^2t}$ は非圧縮Navier-Stokesを厳密に満たす
    /// (非線形項は圧力勾配で厳密に相殺される、標準的な検証ケース)ため、圧力投影は
    /// 数値誤差の範囲で恒等的に効かないはずである。運動エネルギーの減衰率
    /// $e^{-4\nu k^2t}$(速度の2乗)から$\nu k^2$を逆算し解析値と比較する。
    #[test]
    fn f8_taylor_green_vortex_decay_matches_analytic_rate() {
        let nx = 32;
        let ny = 32;
        let length = 1.0;
        let h = length / nx as f64;
        let k = 2.0 * std::f64::consts::PI / length;
        // 実装検証中、semi-Lagrangian移流固有の数値拡散(設計§4.1・§5「渦の寿命が実際より
        // 短い」が明記する既知の限界)が、控えめな粘性(nu=0.01)では真の粘性減衰と同程度
        // かそれ以上の大きさになり、rel_err約52%(nx=32)に達することを発見した。dtを変えても
        // 変化しない(時間離散化誤差ではない)一方、解像度を上げると誤差がほぼ線形に縮小
        // (nx=64でrel_err約27%)することを確認し、空間補間由来の数値拡散と特定した。
        // 真の物理減衰が数値拡散に対して十分優勢になるよう粘性を強めに設定して解決した
        // (nu=0.2、rel_err約2.3%)。
        let nu = 0.2;
        let mut fluid = GridFluid2D::new(nx, ny, h);

        for j in 0..ny as i64 {
            for i in 0..=nx as i64 {
                let idx = wrap(i, nx) + nx * wrap(j, ny);
                let x = i as f64 * h;
                let y = (j as f64 + 0.5) * h;
                fluid.u[idx] = -(k * x).cos() * (k * y).sin();
            }
        }
        for j in 0..=ny as i64 {
            for i in 0..nx as i64 {
                let idx = wrap(i, nx) + nx * wrap(j, ny);
                let x = (i as f64 + 0.5) * h;
                let y = j as f64 * h;
                fluid.v[idx] = (k * x).sin() * (k * y).cos();
            }
        }

        let kinetic_energy = |f: &GridFluid2D| -> f64 {
            f.u.iter().map(|u| u * u).sum::<f64>() + f.v.iter().map(|v| v * v).sum::<f64>()
        };
        let ke0 = kinetic_energy(&fluid);

        let dt = 0.0005;
        let steps = 120;
        for _ in 0..steps {
            fluid.advect_velocity(dt);
            fluid.diffuse_explicit(dt, nu);
            fluid.project(dt, 1.0);
        }
        let ke1 = kinetic_energy(&fluid);
        let total_time = steps as f64 * dt;

        let measured_rate = -(ke1 / ke0).ln() / total_time;
        let analytic_rate = 4.0 * nu * k * k;
        let rel_err = (measured_rate - analytic_rate).abs() / analytic_rate;
        assert!(
            rel_err < 0.05,
            "measured_rate={measured_rate:.6} analytic_rate={analytic_rate:.6} rel_err={rel_err:.4}"
        );
    }

    /// `Solver`トレイト統合: `max_stable_dt`が粘性・移流双方の安定限界の厳しい方を
    /// 返し、`Solver::step`経由でも`step(dt)`と同じ状態遷移になること。
    #[test]
    fn solver_trait_max_stable_dt_reflects_viscous_and_advective_limits_and_step_advances_state() {
        let nx = 8;
        let ny = 8;
        let h = 1.0 / nx as f64;
        let mut fluid = GridFluid2D::new(nx, ny, h);
        fluid.kinematic_viscosity = 0.2;
        fluid.u[0] = 3.0;

        let expected_visc = 0.25 * h * h / fluid.kinematic_viscosity;
        let expected_adv = 5.0 * h / 3.0;
        let expected = expected_visc.min(expected_adv);
        assert!(
            (fluid.max_stable_dt() - expected).abs() < 1e-12,
            "max_stable_dt={} expected={}",
            fluid.max_stable_dt(),
            expected
        );

        let mut via_step = fluid.clone();
        via_step.step(0.001);

        let mut via_trait = fluid.clone();
        let materials = sim_core::MaterialDb::standard();
        let mut rng = sim_math::SimRng::new(1, 1);
        let mut events = sim_core::EventQueue::new();
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        Solver::step(&mut via_trait, 0.001, &mut ctx);

        assert_eq!(via_step.u, via_trait.u);
        assert_eq!(via_step.v, via_trait.v);
    }

    /// 静止状態(速度ゼロ・粘性ゼロ)では移流・拡散いずれも安定限界を持たないため
    /// `max_stable_dt`は`INFINITY`(`Orchestrator::sub_step_count`はこれを1に解釈する)。
    #[test]
    fn solver_trait_max_stable_dt_is_infinite_at_rest_with_no_viscosity() {
        let fluid = GridFluid2D::new(8, 8, 0.1);
        assert_eq!(fluid.max_stable_dt(), f64::INFINITY);
    }

    /// `solid`が`None`なら`pressure_force_on_solid`は`None`(`GridFluidRigid`結合の
    /// ボディ非存在ガードが依拠する)。
    #[test]
    fn pressure_force_on_solid_is_none_without_a_solid() {
        let fluid = GridFluid2D::new(8, 8, 0.5);
        assert!(fluid.pressure_force_on_solid().is_none());
    }

    /// `pressure_force_on_solid`の面積分の配線を、既知の(手で設定した)圧力場で
    /// 決定論的に検証する(`SphRigid`実装検証時に確立したパターン: 圧力場自体の物理的
    /// 妥当性は`GridFluidRigidBox2D`(X2)の既存テストが別途担うので、ここでは
    /// このメソッド自身の面積分ロジックだけを検算する)。p(i,j)=3i+2jという(非物理的だが)
    /// 既知の線形場を与え、剛体を囲む4面の圧力差積分を手計算した期待値と比較する。
    #[test]
    fn pressure_force_on_solid_integrates_a_known_linear_pressure_field() {
        let nx = 8;
        let ny = 8;
        let h = 0.5;
        let mut fluid = GridFluid2D::new(nx, ny, h);
        for j in 0..ny as i64 {
            for i in 0..nx as i64 {
                let idx = (i as usize) + nx * (j as usize);
                fluid.last_pressure[idx] = 3.0 * i as f64 + 2.0 * j as f64;
            }
        }
        // box_center=(2.0,2.0), half=0.75 => セル中心 x=1.75,2.25 (i=3,4) が箱内、
        // i_left=2, i_right=5(y方向も同型でj_below=2, j_above=5)。
        fluid.set_solid_box(Some(GridSolidBox {
            center: (2.0, 2.0),
            half_width: 0.75,
            half_height: 0.75,
            velocity: Vec3::ZERO,
        }));

        let force = fluid.pressure_force_on_solid().expect("solid is set");
        assert!(
            (force.x - (-9.0)).abs() < 1e-9,
            "force.x={} expected=-9.0",
            force.x
        );
        assert!(
            (force.y - (-6.0)).abs() < 1e-9,
            "force.y={} expected=-6.0",
            force.y
        );
        assert_eq!(force.z, 0.0);
    }

    /// `step`は`solid`が設定されている間、投影の前後どちらでもマスク領域内のセルを
    /// 厳密に剛体速度へ強制する(投影後に再度マスクをかけ直す、`GridFluidRigidBox2D::step`
    /// と同じ理由: 丸め誤差で漏れた分を再矯正する)。マスク外のセルは通常どおり移流・
    /// 投影の影響を受ける(この一様流の場合、境界近傍のセルはマスクされた剛体速度からの
    /// 圧力反力を受けて非零になり得る)。
    #[test]
    fn step_forces_masked_cells_to_the_solid_velocity_exactly() {
        let nx = 8;
        let ny = 8;
        let h = 0.5;
        let mut fluid = GridFluid2D::new(nx, ny, h);
        for i in 0..fluid.u.len() {
            fluid.u[i] = 0.3;
        }
        let solid_velocity = Vec3::new(1.5, -2.0, 0.0);
        fluid.set_solid_box(Some(GridSolidBox {
            center: (2.0, 2.0),
            half_width: 0.75,
            half_height: 0.75,
            velocity: solid_velocity,
        }));

        fluid.step(0.001);

        for j in 0..ny as i64 {
            for i in 0..=nx as i64 {
                let x = i as f64 * h;
                let y = (j as f64 + 0.5) * h;
                if (x - 2.0).abs() < 0.75 && (y - 2.0).abs() < 0.75 {
                    assert_eq!(fluid.u_at(i, j), solid_velocity.x);
                }
            }
        }
        for j in 0..=ny as i64 {
            for i in 0..nx as i64 {
                let x = (i as f64 + 0.5) * h;
                let y = j as f64 * h;
                if (x - 2.0).abs() < 0.75 && (y - 2.0).abs() < 0.75 {
                    assert_eq!(fluid.v_at(i, j), solid_velocity.y);
                }
            }
        }
    }

    /// **群7: 開境界(流路)の最も基本的な整合性**——空の流路を一様流が
    /// 「素通りする」こと。移流も粘性拡散も圧力投影も、一様流に対しては
    /// 何もしないのが正しい。周期境界の折り返しやNeumann/Dirichletの取り違えが
    /// あれば、どこかの面に必ず誤差が出る。
    #[test]
    fn a_uniform_flow_passes_through_an_open_channel_unchanged() {
        let inflow = 2.5;
        let mut fluid = GridFluid2D::new(24, 12, 0.1).with_boundary(GridBoundary::Channel {
            inflow_speed: inflow,
        });
        fluid.kinematic_viscosity = 1e-3;

        for _ in 0..200 {
            fluid.step(0.002);
        }
        let mut worst_u: f64 = 0.0;
        let mut worst_v: f64 = 0.0;
        for value in &fluid.u {
            worst_u = worst_u.max((value - inflow).abs());
        }
        for value in &fluid.v {
            worst_v = worst_v.max(value.abs());
        }
        assert!(
            worst_u < 1e-9 && worst_v < 1e-9,
            "一様流は開流路を変化せず通り抜けるはず: |u-U|max={worst_u:.3e} |v|max={worst_v:.3e}"
        );
    }

    /// **群7: 質量保存**——定常状態では流入した体積流量がそのまま流出する。
    /// 障害物を入れても(流れが曲がるだけで)総量は変わらない。
    #[test]
    fn inflow_and_outflow_volume_fluxes_balance_even_around_an_obstacle() {
        let inflow = 1.5;
        let h = 0.1;
        let (nx, ny) = (32usize, 16usize);
        let mut fluid = GridFluid2D::new(nx, ny, h).with_boundary(GridBoundary::Channel {
            inflow_speed: inflow,
        });
        // 流路の中ほどに矩形障害物を置く。
        fluid.set_solid_box(Some(GridSolidBox {
            center: (nx as f64 * h * 0.35, ny as f64 * h * 0.5),
            half_width: 2.0 * h,
            half_height: 3.0 * h,
            velocity: Vec3::ZERO,
        }));

        for _ in 0..400 {
            fluid.step(0.002);
        }

        // 流入(i=0面)と流出(i=nx-1面)の体積流量。
        let flux_at =
            |column: usize| -> f64 { (0..ny).map(|j| fluid.u[column + nx * j] * h).sum::<f64>() };
        let inflow_flux = inflow * ny as f64 * h;
        let outflow_flux = flux_at(nx - 1);
        let rel = (outflow_flux - inflow_flux).abs() / inflow_flux;
        assert!(
            rel < 0.02,
            "定常では流入流量=流出流量のはず: in={inflow_flux:.5} out={outflow_flux:.5} rel={rel:.5}"
        );

        // 障害物の脇では流れが加速する(流路が狭まるので当然)。障害物が
        // 実際に流れを曲げていることの確認。
        let obstacle_column = (nx as f64 * 0.35) as usize;
        let centre_row = ny / 2;
        let centre_speed = fluid.u[obstacle_column + nx * centre_row].abs();
        let bypass_speed = fluid.u[obstacle_column + nx].abs();
        assert!(
            bypass_speed > 1.2 * inflow,
            "障害物の脇は加速するはず: bypass={bypass_speed:.4} inflow={inflow}"
        );
        assert!(
            centre_speed < 0.5 * inflow,
            "障害物の中は流れが止まっているはず: centre={centre_speed:.4}"
        );
    }

    /// **群7: 開境界の投影が発散を落とすこと**(F9の開境界版)。
    /// 適当な発散を持つ初期速度場から1回投影して、内部セルの発散が消えることを確認する
    /// (流出列は圧力Dirichletで発散を吸うので判定から外す)。
    #[test]
    fn projection_removes_divergence_in_an_open_channel() {
        let (nx, ny) = (24usize, 16usize);
        let h = 0.05;
        let mut fluid =
            GridFluid2D::new(nx, ny, h).with_boundary(GridBoundary::Channel { inflow_speed: 1.0 });
        // 非発散フリーな摂動を乗せる。
        for j in 0..ny {
            for i in 0..nx {
                let x = i as f64 * h;
                let y = j as f64 * h;
                fluid.u[i + nx * j] += 0.3 * (3.0 * x).sin() * (2.0 * y).cos();
                fluid.v[i + nx * j] += 0.2 * (2.0 * x).cos() * (3.0 * y).sin();
            }
        }
        let before = (1..nx - 1)
            .flat_map(|i| (0..ny).map(move |j| (i, j)))
            .fold(0.0_f64, |acc, (i, j)| {
                acc.max(fluid.divergence(i as i64, j as i64).abs())
            });
        assert!(before > 0.1, "初期場は発散を持つはず: {before}");

        fluid.project(0.01, 1.0);

        let after = (1..nx - 1)
            .flat_map(|i| (0..ny).map(move |j| (i, j)))
            .fold(0.0_f64, |acc, (i, j)| {
                acc.max(fluid.divergence(i as i64, j as i64).abs())
            });
        assert!(
            after < 1e-6,
            "投影後は内部セルの発散が消えるはず: before={before:.4} after={after:.3e}"
        );
    }

    /// `Periodic`(既定)は移行前とまったく同じ挙動——F8/F9 が影響を受けないことの保証。
    #[test]
    fn the_default_boundary_is_periodic_and_unchanged() {
        let build = || {
            let mut f = GridFluid2D::new(16, 16, 0.1);
            for j in 0..16 {
                for i in 0..16 {
                    let x = i as f64 * 0.1;
                    let y = j as f64 * 0.1;
                    f.u[i + 16 * j] = (x).sin() * (y).cos();
                    f.v[i + 16 * j] = -(x).cos() * (y).sin();
                }
            }
            f
        };
        let (mut a, mut b) = (build(), build());
        assert_eq!(a.boundary(), GridBoundary::Periodic);
        for _ in 0..20 {
            a.step(0.001);
            b.step(0.001);
        }
        assert_eq!(a.u, b.u);
        assert_eq!(a.v, b.v);
    }
}
