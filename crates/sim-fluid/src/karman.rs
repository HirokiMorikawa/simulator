//! カルマン渦列(円柱まわりの周期的渦剥離)。設計: docs/11-fluid/02-eulerian-grid.md §4.5、
//! docs/21-verification/01-analytic-tests.md F11。
//!
//! `GridFluid2D`(完全周期境界)を拡張するのではなく、流入/流出境界(x方向)+円柱障害物
//! (マスキング方式の簡易固体セル)を持つ専用のチャネル流ソルバとして実装する。y方向は
//! 周期境界(無限に長い円柱まわりの2D流れの標準的な近似)。固体セルの扱いは正式な
//! cut-cell法ではなく、各ステップ後に円柱内部の速度を0に強制する素朴なマスキング法
//! (設計§4.5がF11自体の合格条件を実装時の数値実験で確定してよいとしていることに対応する
//! 縮約実装)。

use sim_math::Vec3;

pub struct KarmanChannel2D {
    pub nx: usize,
    pub ny: usize,
    pub h: f64,
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    pub inflow_speed: f64,
    pub cylinder_center: (f64, f64),
    pub cylinder_radius: f64,
}

fn wrap(i: i64, n: usize) -> usize {
    i.rem_euclid(n as i64) as usize
}

impl KarmanChannel2D {
    pub fn new(
        nx: usize,
        ny: usize,
        h: f64,
        inflow_speed: f64,
        cylinder_center: (f64, f64),
        cylinder_radius: f64,
    ) -> KarmanChannel2D {
        let mut channel = KarmanChannel2D {
            nx,
            ny,
            h,
            u: vec![inflow_speed; (nx + 1) * ny],
            v: vec![0.0; nx * ny],
            inflow_speed,
            cylinder_center,
            cylinder_radius,
        };
        channel.apply_solid_mask();
        channel
    }

    fn u_idx(&self, i: usize, j: i64) -> usize {
        i + (self.nx + 1) * wrap(j, self.ny)
    }

    fn v_idx(&self, i: usize, j: i64) -> usize {
        i + self.nx * wrap(j, self.ny)
    }

    pub fn u_at(&self, i: usize, j: i64) -> f64 {
        self.u[self.u_idx(i, j)]
    }

    pub fn v_at(&self, i: usize, j: i64) -> f64 {
        self.v[self.v_idx(i, j)]
    }

    fn is_solid_at(&self, x: f64, y: f64) -> bool {
        let (cx, cy) = self.cylinder_center;
        let dx = x - cx;
        let dy = y - cy;
        dx * dx + dy * dy < self.cylinder_radius * self.cylinder_radius
    }

    /// 円柱内部(または面上)の速度成分を0に強制する(設計§4.5の縮約実装、モジュールdoc参照)。
    fn apply_solid_mask(&mut self) {
        for j in 0..self.ny as i64 {
            for i in 0..=self.nx {
                let x = i as f64 * self.h;
                let y = (j as f64 + 0.5) * self.h;
                if self.is_solid_at(x, y) {
                    let idx = self.u_idx(i, j);
                    self.u[idx] = 0.0;
                }
            }
        }
        for j in 0..self.ny as i64 {
            for i in 0..self.nx {
                let x = (i as f64 + 0.5) * self.h;
                let y = j as f64 * self.h;
                if self.is_solid_at(x, y) {
                    let idx = self.v_idx(i, j);
                    self.v[idx] = 0.0;
                }
            }
        }
    }

    /// 流入(x=0、Dirichlet: u=inflow_speed・v=0)・流出(x=Lx、対流的境界: 勾配0のコピー)を適用。
    fn apply_inflow_outflow(&mut self) {
        for j in 0..self.ny as i64 {
            let idx = self.u_idx(0, j);
            self.u[idx] = self.inflow_speed;
        }
        for j in 0..self.ny as i64 {
            let last = self.u_idx(self.nx, j);
            let second_last = self.u_idx(self.nx - 1, j);
            self.u[last] = self.u[second_last];
        }
        for j in 0..self.ny as i64 {
            let idx = self.v_idx(0, j);
            self.v[idx] = 0.0;
        }
    }

    fn sample_periodic_clamped_x(
        data: &[f64],
        nx_points: usize,
        ny: usize,
        h: f64,
        offset: Vec3,
        pos: Vec3,
    ) -> f64 {
        let max_x = (nx_points - 1) as f64 * h;
        let clamped_x = (pos.x - offset.x).clamp(0.0, max_x);
        let local_x = clamped_x / h;
        let local_y = (pos.y - offset.y) / h;
        let i0f = local_x.floor();
        let j0f = local_y.floor();
        let fx = (local_x - i0f).clamp(0.0, 1.0);
        let fy = local_y - j0f;
        let i0 = (i0f as i64).clamp(0, nx_points as i64 - 2).max(0);
        let j0 = j0f as i64;
        let get = |ii: i64, jj: i64| -> f64 {
            let ii = ii.clamp(0, nx_points as i64 - 1) as usize;
            data[ii + nx_points * wrap(jj, ny)]
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
        let offset = Vec3::new(0.0, 0.5 * self.h, 0.0);
        Self::sample_periodic_clamped_x(&self.u, self.nx + 1, self.ny, self.h, offset, pos)
    }

    fn sample_v(&self, pos: Vec3) -> f64 {
        let offset = Vec3::new(0.5 * self.h, 0.0, 0.0);
        Self::sample_periodic_clamped_x(&self.v, self.nx, self.ny, self.h, offset, pos)
    }

    fn velocity_at(&self, pos: Vec3) -> Vec3 {
        Vec3::new(self.sample_u(pos), self.sample_v(pos), 0.0)
    }

    /// semi-Lagrangian移流(RK2中点法、`GridFluid2D::advect_velocity`と同じ方式)。
    pub fn advect_velocity(&mut self, dt: f64) {
        let old = KarmanChannel2D {
            nx: self.nx,
            ny: self.ny,
            h: self.h,
            u: self.u.clone(),
            v: self.v.clone(),
            inflow_speed: self.inflow_speed,
            cylinder_center: self.cylinder_center,
            cylinder_radius: self.cylinder_radius,
        };

        for j in 0..self.ny as i64 {
            for i in 1..self.nx {
                let pos = Vec3::new(i as f64 * self.h, (j as f64 + 0.5) * self.h, 0.0);
                let vel = old.velocity_at(pos);
                let mid = pos - vel.scale(0.5 * dt);
                let vel_mid = old.velocity_at(mid);
                let src = pos - vel_mid.scale(dt);
                let idx = self.u_idx(i, j);
                self.u[idx] = old.sample_u(src);
            }
        }
        for j in 0..self.ny as i64 {
            for i in 1..self.nx {
                let pos = Vec3::new((i as f64 + 0.5) * self.h, j as f64 * self.h, 0.0);
                let vel = old.velocity_at(pos);
                let mid = pos - vel.scale(0.5 * dt);
                let vel_mid = old.velocity_at(mid);
                let src = pos - vel_mid.scale(dt);
                let idx = self.v_idx(i, j);
                self.v[idx] = old.sample_v(src);
            }
        }
    }

    /// 陽的粘性拡散(5点ラプラシアン、y周期・x境界はクランプ、`GridFluid2D`と同型)。
    pub fn diffuse_explicit(&mut self, dt: f64, kinematic_viscosity: f64) {
        let coeff = kinematic_viscosity * dt / (self.h * self.h);
        let old_u = self.u.clone();
        let old_v = self.v.clone();
        for j in 0..self.ny as i64 {
            for i in 1..self.nx {
                let idx = self.u_idx(i, j);
                let ip = old_u[self.u_idx((i + 1).min(self.nx), j)];
                let im = old_u[self.u_idx(i - 1, j)];
                let jp = old_u[self.u_idx(i, j + 1)];
                let jm = old_u[self.u_idx(i, j - 1)];
                let lap = ip + im + jp + jm - 4.0 * old_u[idx];
                self.u[idx] += coeff * lap;
            }
        }
        for j in 0..self.ny as i64 {
            for i in 1..self.nx - 1 {
                let idx = self.v_idx(i, j);
                let ip = old_v[self.v_idx(i + 1, j)];
                let im = old_v[self.v_idx(i - 1, j)];
                let jp = old_v[self.v_idx(i, j + 1)];
                let jm = old_v[self.v_idx(i, j - 1)];
                let lap = ip + im + jp + jm - 4.0 * old_v[idx];
                self.v[idx] += coeff * lap;
            }
        }
    }

    /// 圧力投影。流出境界(x=Lx)をDirichlet(p=0)、流入境界(x=0、速度Dirichlet)をNeumann
    /// (dp/dn=0、ミラー)として扱う(周期境界が無いため`GridFluid2D`のような特異性は生じない)。
    pub fn project(&mut self, dt: f64, density: f64) {
        let nx = self.nx;
        let ny = self.ny;
        let n = nx * ny;
        let h = self.h;

        let divergence =
            |u: &[f64], v: &[f64], i: usize, j: i64, nx: usize, ny: usize, h: f64| -> f64 {
                let u_idx = |ii: usize, jj: i64| ii + (nx + 1) * wrap(jj, ny);
                let v_idx = |ii: usize, jj: i64| ii + nx * wrap(jj, ny);
                (u[u_idx(i + 1, j)] - u[u_idx(i, j)]) / h
                    + (v[v_idx(i, j + 1)] - v[v_idx(i, j)]) / h
            };

        let mut rhs = vec![0.0; n];
        for j in 0..ny as i64 {
            for i in 0..nx {
                rhs[i + nx * wrap(j, ny)] =
                    density / dt * divergence(&self.u, &self.v, i, j, nx, ny, h);
            }
        }

        let h2 = h * h;
        let apply_a = |x: &[f64], out: &mut [f64]| {
            for j in 0..ny as i64 {
                for i in 0..nx {
                    let idx = i + nx * wrap(j, ny);
                    let p_ip = if i + 1 < nx {
                        x[i + 1 + nx * wrap(j, ny)]
                    } else {
                        0.0
                    };
                    let p_im = if i > 0 {
                        x[i - 1 + nx * wrap(j, ny)]
                    } else {
                        x[idx]
                    };
                    let p_jp = x[i + nx * wrap(j + 1, ny)];
                    let p_jm = x[i + nx * wrap(j - 1, ny)];
                    out[idx] = (p_ip + p_im + p_jp + p_jm - 4.0 * x[idx]) / h2;
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
            3000,
        );
        debug_assert!(
            result.converged,
            "Karman channel pressure projection PCG did not converge: {result:?}"
        );

        let scale = dt / density;
        for j in 0..ny as i64 {
            for i in 1..nx {
                let p_here = pressure[i + nx * wrap(j, ny)];
                let p_prev = pressure[i - 1 + nx * wrap(j, ny)];
                let dpdx = (p_here - p_prev) / h;
                let idx = self.u_idx(i, j);
                self.u[idx] -= scale * dpdx;
            }
        }
        for j in 0..ny as i64 {
            for i in 0..nx {
                let p_here = pressure[i + nx * wrap(j, ny)];
                let p_prev = pressure[i + nx * wrap(j - 1, ny)];
                let dpdy = (p_here - p_prev) / h;
                let idx = self.v_idx(i, j);
                self.v[idx] -= scale * dpdy;
            }
        }
    }

    /// セル中心の渦度 $\omega_z=\partial v/\partial x-\partial u/\partial y$(設計§4.5)。
    /// x境界(i=0・i=nx-1)は隣接セルが無いため0とする(渦度強化を境界近傍では適用しない)。
    fn vorticity_at(&self, i: usize, j: i64) -> f64 {
        if i == 0 || i == self.nx - 1 {
            return 0.0;
        }
        let dvdx = (self.v_at(i + 1, j) - self.v_at(i - 1, j)) / (2.0 * self.h);
        let u_avg =
            |ii: usize, jj: i64| -> f64 { (self.u_at(ii, jj) + self.u_at(ii + 1, jj)) * 0.5 };
        let dudy = (u_avg(i, j + 1) - u_avg(i, j - 1)) / (2.0 * self.h);
        dvdx - dudy
    }

    /// 渦度強化(設計§4.5、Fedkiw et al. 2001): 数値拡散で失われる小渦を補償する非物理的な
    /// 補償項。F11(カルマン渦)は設計§4.5が明記する例外ケースで、64³相当の解像度では
    /// semi-Lagrangian移流の数値拡散により実効レイノルズ数が渦剥離の閾値(Re≈47)を下回り、
    /// 渦度強化なしでは自発的な渦剥離が立ち上がらないことを実装検証中に確認したため、
    /// 検証モードでもこの項を有効にする(設計が許容する代替経路)。
    pub fn apply_vorticity_confinement(&mut self, dt: f64, epsilon: f64) {
        if epsilon == 0.0 {
            return;
        }
        let nx = self.nx;
        let ny = self.ny as i64;
        let mut omega = vec![0.0; nx * self.ny];
        for j in 0..ny {
            for i in 0..nx {
                omega[i + nx * wrap(j, self.ny)] = self.vorticity_at(i, j);
            }
        }
        let abs_omega: Vec<f64> = omega.iter().map(|w| w.abs()).collect();

        for j in 0..ny {
            for i in 1..nx - 1 {
                let gx = (abs_omega[i + 1 + nx * wrap(j, self.ny)]
                    - abs_omega[i - 1 + nx * wrap(j, self.ny)])
                    / (2.0 * self.h);
                let gy = (abs_omega[i + nx * wrap(j + 1, self.ny)]
                    - abs_omega[i + nx * wrap(j - 1, self.ny)])
                    / (2.0 * self.h);
                let mag = (gx * gx + gy * gy).sqrt().max(1e-12);
                let (n_x, n_y) = (gx / mag, gy / mag);
                let wz = omega[i + nx * wrap(j, self.ny)];
                let force_x = epsilon * self.h * (n_y * wz);
                let force_y = epsilon * self.h * (-n_x * wz);

                let ui = self.u_idx(i, j);
                let uip = self.u_idx(i + 1, j);
                self.u[ui] += 0.5 * force_x * dt;
                self.u[uip] += 0.5 * force_x * dt;

                let vj = self.v_idx(i, j);
                let vjp = self.v_idx(i, j + 1);
                self.v[vj] += 0.5 * force_y * dt;
                self.v[vjp] += 0.5 * force_y * dt;
            }
        }
    }

    /// 1ステップ進める(移流→拡散→固体マスク→渦度強化(オプション)→境界条件→投影→固体マスク)。
    /// `vorticity_confinement_epsilon`が0なら渦度強化は無効(設計§4.5の既定・検証モード)。
    pub fn step(
        &mut self,
        dt: f64,
        kinematic_viscosity: f64,
        density: f64,
        vorticity_confinement_epsilon: f64,
    ) {
        self.advect_velocity(dt);
        self.apply_solid_mask();
        self.diffuse_explicit(dt, kinematic_viscosity);
        self.apply_solid_mask();
        self.apply_vorticity_confinement(dt, vorticity_confinement_epsilon);
        self.apply_solid_mask();
        self.apply_inflow_outflow();
        self.project(dt, density);
        self.apply_solid_mask();
        self.apply_inflow_outflow();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// F11: カルマン渦列 — 円柱後流のストローハル数 $St=fD/U\approx0.2$、rel<20%
    /// (docs/21-verification/01-analytic-tests.md F11)。実装検証中、設計§4.5が明記する
    /// とおりまず渦度強化オフ(ε=0)で数値実験したところ、Re=100(層流渦剥離が起こる
    /// はずの領域)でも後流が非対称な定常状態に落ち着くだけで自発的な渦剥離が起こらない
    /// ことを発見した。原因を切り分けたところ、(1)完全対称なセットアップでは離散化も
    /// 対称性を保つため不安定性が成長しないこと(円柱をわずかに(0.1h)非対称配置して
    /// 対称性を破る標準的な対策で対応)、(2)semi-Lagrangian移流の数値拡散(F8のTaylor-Green
    /// 渦検証で発見したのと同じ既知の限界)がこの解像度では実効レイノルズ数を渦剥離の
    /// 閾値(Re≈47)未満まで下げてしまうこと、の2つが原因と判明した。(1)は非対称配置で
    /// 解決し、(2)は設計§4.5が明記する代替経路(検証モードでも渦度強化を許容し、強化係数を
    /// 合格条件として記録する)を採用して解決した(ε=1.0)。CI実行時間に収まる解像度・
    /// 領域サイズ・刻み幅を探索し(周期境界のy方向を狭くしすぎると円柱の周期像どうしの
    /// 干渉でストローハル数が設計値から大きくずれる(St≈0.37)ことも発見、Ly=4.8まで
    /// 広げて解決)、最終的にSt=0.2014(rel_err<1%)・debugビルドで約76秒の設定に到達した。
    #[test]
    fn f11_karman_vortex_shedding_matches_analytic_strouhal_number() {
        let h = 0.1;
        let radius = 0.5;
        let nx = 80;
        let ny = 48;
        let u0 = 1.0;
        let reynolds = 100.0;
        let nu = u0 * (2.0 * radius) / reynolds;
        let vorticity_confinement_epsilon = 1.0;

        let cx = 2.0;
        // 対称性を崩す微小な非対称配置(標準的な対策、モジュールdocのコメント参照)。
        let cy = ny as f64 * h / 2.0 + 0.1;
        let mut channel = KarmanChannel2D::new(nx, ny, h, u0, (cx, cy), radius);

        let dt = 0.08;
        let total_time = 40.0;
        let steps = (total_time / dt) as u32;

        let probe_x = cx + 2.5;
        let probe_y = ny as f64 * h / 2.0 + 0.3;
        let probe_i = (probe_x / h - 0.5).round().max(0.0) as usize;
        let probe_j = (probe_y / h).round() as i64;

        let mut samples = Vec::with_capacity(steps as usize);
        for step in 0..steps {
            channel.step(dt, nu, 1.0, vorticity_confinement_epsilon);
            let t = step as f64 * dt;
            let v = channel.v_at(probe_i.min(nx - 1), probe_j);
            samples.push((t, v));
        }

        // 前半(過渡応答が収まりきっていない可能性がある)を除いた後半で零交差を数える。
        let half = samples.len() / 2;
        let window = &samples[half..];
        let mut crossing_times = Vec::new();
        for pair in window.windows(2) {
            let (t0, v0) = pair[0];
            let (t1, v1) = pair[1];
            if v0 <= 0.0 && v1 > 0.0 {
                let frac = -v0 / (v1 - v0);
                crossing_times.push(t0 + frac * (t1 - t0));
            }
        }
        assert!(
            crossing_times.len() >= 3,
            "expected at least 2 full periods (>=3 zero crossings) in the periodic regime, got {}: {crossing_times:?}",
            crossing_times.len()
        );

        let periods: Vec<f64> = crossing_times.windows(2).map(|w| w[1] - w[0]).collect();
        let mean_period = periods.iter().sum::<f64>() / periods.len() as f64;
        let measured_strouhal = (1.0 / mean_period) * (2.0 * radius) / u0;

        let expected_strouhal = 0.2;
        let rel_err = (measured_strouhal - expected_strouhal).abs() / expected_strouhal;
        assert!(
            rel_err < 0.2,
            "measured_strouhal={measured_strouhal:.4} expected={expected_strouhal} rel_err={rel_err:.4} periods={periods:?}"
        );
    }
}
