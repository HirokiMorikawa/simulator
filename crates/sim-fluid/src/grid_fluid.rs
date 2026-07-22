//! 格子(Eulerian)流体ソルバ、2D周期境界のみ。設計: docs/11-fluid/02-eulerian-grid.md。
//!
//! 完全な `GridFluid`(3D、MAC格子、Solid/Empty境界、渦度強化)ではなく、Taylor-Green渦
//! (F8)・投影後発散(F9)の検証に必要な範囲 — 周期境界の2D非圧縮流(移流+粘性拡散+
//! 圧力投影)— に絞った縮約実装。ポアズイユ流(F7、固体境界+4解像度の収束次数)・
//! カルマン渦列(F11、円柱障害物+渦度強化の要否判断)は固体境界の扱いが別途必要な
//! ため後続増分に残す。
//!
//! 格子は staggered(MAC)配置: 圧力・スカラーはセル中心 $((i+\tfrac12)h,(j+\tfrac12)h)$、
//! `u` は x面 $(ih,(j+\tfrac12)h)$、`v` は y面 $((i+\tfrac12)h,jh)$ に置く。周期境界のため
//! 各成分の格子点数はセル数と同じ($n_x\times n_y$、境界の重複層を持たない)。

use sim_core::{EnergyBreakdown, Solver, SolverContext, StateHasher};
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

/// 周期境界の2D格子流体。`u`・`v` は共に長さ `nx*ny`(staggered配置、モジュールdoc参照)。
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
    /// `GridFluidRigid`結合用の単一剛体マスク。`None`なら従来どおり完全周期境界
    /// (既存の挙動に一切変更なし)。
    pub solid: Option<GridSolidBox>,
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
            solid: None,
            last_pressure: vec![0.0; nx * ny],
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
        (self.u_at(i + 1, j) - self.u_at(i, j)) / self.h
            + (self.v_at(i, j + 1) - self.v_at(i, j)) / self.h
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

    fn sample_u(&self, pos: Vec3) -> f64 {
        // u[i][j] は (i*h, (j+0.5)*h) に位置する。
        let offset = Vec3::new(0.0, 0.5 * self.h, 0.0);
        Self::sample_periodic(&self.u, self.nx, self.ny, self.h, offset, pos)
    }

    fn sample_v(&self, pos: Vec3) -> f64 {
        // v[i][j] は ((i+0.5)*h, j*h) に位置する。
        let offset = Vec3::new(0.5 * self.h, 0.0, 0.0);
        Self::sample_periodic(&self.v, self.nx, self.ny, self.h, offset, pos)
    }

    fn velocity_at(&self, pos: Vec3) -> Vec3 {
        Vec3::new(self.sample_u(pos), self.sample_v(pos), 0.0)
    }

    /// semi-Lagrangian移流(RK2中点法によるバックトレース、設計§4.1)。
    pub fn advect_velocity(&mut self, dt: f64) {
        let old_u = self.u.clone();
        let old_v = self.v.clone();
        let old = GridFluid2D {
            nx: self.nx,
            ny: self.ny,
            h: self.h,
            u: old_u,
            v: old_v,
            density: self.density,
            kinematic_viscosity: self.kinematic_viscosity,
            solid: self.solid,
            last_pressure: self.last_pressure.clone(),
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

    /// 剛体内部(または面上)のセルの速度を剛体の速度に強制する(`GridFluidRigidBox2D`
    /// (X2)と同じマスキング方式の縮約実装、設計 docs/11-fluid/02-eulerian-grid.md §6
    /// 「剛体→流体」)。
    fn apply_solid_mask(&mut self, solid: &GridSolidBox) {
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                let x = i as f64 * self.h;
                let y = (j as f64 + 0.5) * self.h;
                if Self::is_solid_at(solid, x, y) {
                    let idx = self.idx(i, j);
                    self.u[idx] = solid.velocity.x;
                }
            }
        }
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                let x = (i as f64 + 0.5) * self.h;
                let y = j as f64 * self.h;
                if Self::is_solid_at(solid, x, y) {
                    let idx = self.idx(i, j);
                    self.v[idx] = solid.velocity.y;
                }
            }
        }
    }

    /// 剛体表面の圧力積分による流体力(設計 docs/11-fluid/02-eulerian-grid.md §6
    /// 「流体→剛体: 剛体表面セルの圧力を面積分」)。`self.solid`が`None`なら`None`を返す。
    /// `x`・`y`成分とも、剛体を囲む4面(左右・上下)それぞれの圧力差を積分する
    /// (`GridFluidRigidBox2D::pressure_force_on_box`は鉛直方向のみだったが、こちらは
    /// 剛体が2自由度で自由運動できる一般結合のため両方向を計算する)。
    pub fn pressure_force_on_solid(&self) -> Option<Vec3> {
        let solid = self.solid?;
        let nx = self.nx as i64;
        let ny = self.ny as i64;

        let mut i_min = None;
        let mut i_max = None;
        for i in 0..nx {
            let x = (i as f64 + 0.5) * self.h;
            if Self::is_solid_at(&solid, x, solid.center.1) {
                i_min = Some(i_min.map_or(i, |m: i64| m.min(i)));
                i_max = Some(i_max.map_or(i, |m: i64| m.max(i)));
            }
        }
        let mut j_min = None;
        let mut j_max = None;
        for j in 0..ny {
            let y = (j as f64 + 0.5) * self.h;
            if Self::is_solid_at(&solid, solid.center.0, y) {
                j_min = Some(j_min.map_or(j, |m: i64| m.min(j)));
                j_max = Some(j_max.map_or(j, |m: i64| m.max(j)));
            }
        }
        let (Some(i_min), Some(i_max), Some(j_min), Some(j_max)) = (i_min, i_max, j_min, j_max)
        else {
            return Some(Vec3::ZERO);
        };
        let i_left = i_min - 1;
        let i_right = i_max + 1;
        let j_below = j_min - 1;
        let j_above = j_max + 1;

        let mut fx = 0.0;
        for j in j_min..=j_max {
            let y = (j as f64 + 0.5) * self.h;
            if (y - solid.center.1).abs() < solid.half_height {
                let p_left = self.last_pressure[self.idx(i_left, j)];
                let p_right = self.last_pressure[self.idx(i_right, j)];
                fx += self.h * (p_left - p_right);
            }
        }
        let mut fy = 0.0;
        for i in i_min..=i_max {
            let x = (i as f64 + 0.5) * self.h;
            if (x - solid.center.0).abs() < solid.half_width {
                let p_below = self.last_pressure[self.idx(i, j_below)];
                let p_above = self.last_pressure[self.idx(i, j_above)];
                fy += self.h * (p_below - p_above);
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
        let mut rhs = vec![0.0; n];
        for j in 0..self.ny as i64 {
            for i in 0..self.nx as i64 {
                rhs[self.idx(i, j)] = density / dt * self.divergence(i, j);
            }
        }
        let mean: f64 = rhs.iter().sum::<f64>() / n as f64;
        for r in rhs.iter_mut() {
            *r -= mean;
        }

        let nx = self.nx;
        let ny = self.ny;
        let h2 = self.h * self.h;
        let apply_a = |x: &[f64], out: &mut [f64]| {
            for j in 0..ny as i64 {
                for i in 0..nx as i64 {
                    let idx = wrap(i, nx) + nx * wrap(j, ny);
                    let ip = wrap(i + 1, nx) + nx * wrap(j, ny);
                    let im = wrap(i - 1, nx) + nx * wrap(j, ny);
                    let jp = wrap(i, nx) + nx * wrap(j + 1, ny);
                    let jm = wrap(i, nx) + nx * wrap(j - 1, ny);
                    out[idx] = (x[ip] + x[im] + x[jp] + x[jm] - 4.0 * x[idx]) / h2;
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
        for j in 0..self.ny as i64 {
            for i in 0..=self.nx as i64 {
                let ip = wrap(i, nx) + nx * wrap(j, ny);
                let im = wrap(i - 1, nx) + nx * wrap(j, ny);
                let dpdx = (pressure[ip] - pressure[im]) / self.h;
                let idx = wrap(i, self.nx) + self.nx * wrap(j, self.ny);
                self.u[idx] -= scale * dpdx;
            }
        }
        for j in 0..=self.ny as i64 {
            for i in 0..self.nx as i64 {
                let jp = wrap(i, nx) + nx * wrap(j, ny);
                let jm = wrap(i, nx) + nx * wrap(j - 1, ny);
                let dpdy = (pressure[jp] - pressure[jm]) / self.h;
                let idx = wrap(i, self.nx) + self.nx * wrap(j, self.ny);
                self.v[idx] -= scale * dpdy;
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

    /// `Solver::step`が呼ぶ1ステップ分の処理(設計§4.6のステップまとめから、
    /// このモジュールが実装する範囲——移流+粘性拡散+投影(+`solid`が設定されていれば
    /// 剛体マスキング)——を抜き出したもの)。外力・煙/温度移流(§4.2, §4.6)はこの
    /// 縮約実装の対象外。剛体マスクは投影の前後両方に適用する(`GridFluidRigidBox2D::step`
    /// と同じ理由: 投影前に境界条件として与え、投影後に丸め誤差で漏れた分を再度矯正する)。
    pub fn step(&mut self, dt: f64) {
        self.advect_velocity(dt);
        if self.kinematic_viscosity > 0.0 {
            self.diffuse_explicit(dt, self.kinematic_viscosity);
        }
        if let Some(solid) = self.solid {
            self.apply_solid_mask(&solid);
        }
        self.last_pressure = self.project(dt, self.density);
        if let Some(solid) = self.solid {
            self.apply_solid_mask(&solid);
        }
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
        fluid.solid = Some(GridSolidBox {
            center: (2.0, 2.0),
            half_width: 0.75,
            half_height: 0.75,
            velocity: Vec3::ZERO,
        });

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
        fluid.solid = Some(GridSolidBox {
            center: (2.0, 2.0),
            half_width: 0.75,
            half_height: 0.75,
            velocity: solid_velocity,
        });

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
}
