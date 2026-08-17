//! `DissipationToHeat`(設計 docs/20-integration/01-coupling-matrix.md §3「P1: 摩擦・衝突・
//! 抗力散逸 → ThermalNode(熱浸透率比分配)」)。
//!
//! **群5で「熱浸透率比分配」(設計 docs/12-thermal/02-heat-transfer.md §4.4)を実装した**。
//! 移行前は「剛体↔熱ノードの対応表が無い」という理由で、単一の対象`ThermalNode`
//! (シーン全体の「環境」)へ全散逸熱を注入する縮約版だった。群5では
//!
//! 1. 対応表を**この結合自身が持つ**(`body_links`。`ConvectionLink`の物性値・
//!    `PistonGas`の`area`と同じ「呼び出し側が直接渡す」パターン。熱浸透率
//!    $e_t=\sqrt{k\rho c_p}$ も同様に呼び出し側が計算して渡す —
//!    `DomainStates`が`MaterialDb`を持たないため)、
//! 2. `sim_mechanics::MechanicsSolver::last_contact_dissipation_by_body`
//!    (**群5で新設**した剛体ごとの散逸)と`last_manifolds`(接触ペア)を突き合わせて
//!    ペアごとの散逸熱$\Delta Q$を作り、
//! 3. 設計どおり $Q_A/Q_B=e_{t,A}/e_{t,B}$ で2ノードへ配る。
//!
//! **残る近似**: 接触ソルバ(sequential impulses)は全マニフォールドを連立で解くため、
//! 「剛体$i$の運動エネルギー損失」を接触ペアごとに厳密に切り分ける情報が原理的に無い
//! (剛体が同時に複数の接触を持つ場合)。そこで**剛体$i$の損失をその剛体が関与する
//! マニフォールドへ均等配分**する。単一接触(床の上を滑る箱など、大半のデモ)では
//! 均等配分は恒等的に正しく、多接触の場合のみ近似になる。総熱量はどちらでも厳密に
//! 保存する(配分の内訳が変わるだけ)。
//!
//! `body_links`が空なら移行前とまったく同じ挙動(全量を`thermal_node`へ)。対応表に
//! 無い剛体は熱浸透率0として扱い、相手側が持って行く(静的な床など、熱容量を
//! モデル化していない相手にペアの熱を吸わせないため)。両方とも未登録なら
//! 既定ノード`thermal_node`へ落とす。
//!
//! 散逸源は`sim_mechanics::MechanicsSolver::last_contact_dissipation`(接触解決(摩擦+反発)
//! 直前直後の運動エネルギー差分、同crateのdoc参照)のみで、抗力による散逸は含まない
//! (抗力の仕事は保存力(重力)と共に積分されるため現在の測定窓では分離できない、
//! 後続増分で追加)。

use crate::domain_states::{Coupling, CouplingKind, DomainStates};
use sim_core::DomainId;

/// 接触解決による運動エネルギー散逸を単一の`ThermalNode`(`thermal_node`インデックス)に
/// 注入する(設計§1「保存量の橋は必ず対で書く」— 取り出した量(`last_contact_dissipation`)を
/// そのまま注入し、消費済みとしてリセットする)。
#[derive(Clone)]
pub struct DissipationToHeat {
    /// 対応表(`body_links`)に載っていない散逸の受け皿(シーン全体の「環境」ノード)。
    pub thermal_node: usize,
    /// 剛体↔熱ノード対応表(**群5で追加**、モジュールdoc参照)。空なら移行前と同じ
    /// 「全量を`thermal_node`へ」。
    pub body_links: Vec<BodyThermalLink>,
}

/// 剛体1体と`ThermalNode`の対応 + その材料の熱浸透率(**群5で追加**)。
#[derive(Clone, Copy, Debug)]
pub struct BodyThermalLink {
    pub body_index: usize,
    pub thermal_node: usize,
    /// 熱浸透率 $e_t=\sqrt{k\rho c_p}$ [J/(m²·K·s^(1/2))]。呼び出し側が材料から
    /// 計算して渡す(`effusivity`ヘルパーを使える)。
    pub effusivity: f64,
}

/// 熱浸透率 $e_t=\sqrt{k\rho c_p}$(設計 docs/12-thermal/02-heat-transfer.md §4.4、
/// 半無限体の接触理論)。`sim_core::Material`の`conductivity`・`density`・
/// `specific_heat`から作る(**群5で追加**)。
pub fn effusivity(conductivity: f64, density: f64, specific_heat: f64) -> f64 {
    (conductivity * density * specific_heat).max(0.0).sqrt()
}

impl DissipationToHeat {
    /// 対応表なし(移行前と同じ「全量を単一ノードへ」)。
    pub fn to_single_node(thermal_node: usize) -> DissipationToHeat {
        DissipationToHeat {
            thermal_node,
            body_links: Vec::new(),
        }
    }

    fn link_of(&self, body: usize) -> Option<&BodyThermalLink> {
        self.body_links.iter().find(|l| l.body_index == body)
    }
}

impl Coupling for DissipationToHeat {
    fn kind(&self) -> CouplingKind {
        CouplingKind::DissipationToHeat
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Mechanics, DomainId::Thermal]
    }

    fn describe(&self) -> String {
        format!("DissipationToHeat -> thermal_node[{}]", self.thermal_node)
    }

    fn referenced_bodies(&self) -> Vec<usize> {
        self.body_links.iter().map(|l| l.body_index).collect()
    }

    fn referenced_thermal_nodes(&self) -> Vec<usize> {
        let mut nodes = vec![self.thermal_node];
        for link in &self.body_links {
            if !nodes.contains(&link.thermal_node) {
                nodes.push(link.thermal_node);
            }
        }
        nodes
    }

    fn apply(&mut self, world: &mut DomainStates, _dt: f64) {
        let dissipated = world.mechanics.last_contact_dissipation;
        // クランプしない(sim_mechanics::MechanicsSolver::last_contact_dissipationのdoc参照:
        // 稀に負値になりうるが、それを含めて注入することで対記帳の総量が長時間で正しく
        // 相殺される)。
        if dissipated != 0.0 {
            // (node_index, heat) の並び。総和は必ず`dissipated`と一致する。
            let injections = if self.body_links.is_empty() {
                vec![(self.thermal_node, dissipated)]
            } else {
                self.distribute_by_effusivity(world, dissipated)
            };
            if let Some(thermal) = &mut world.thermal {
                for (node_index, heat) in injections {
                    if let Some(node) = thermal.nodes.get_mut(node_index) {
                        // 対記帳: mechanics側から取り出した量をそのままthermal側へ
                        // 注入する(ΔE = C・ΔT)。
                        node.temperature += heat / node.heat_capacity;
                    }
                }
            }
        }
        // 次stepで前stepの散逸を二重計上しないよう消費済みにする。
        world.mechanics.last_contact_dissipation = 0.0;
        world.mechanics.last_contact_dissipation_by_body.clear();
    }
}

impl DissipationToHeat {
    /// 設計 §4.4「熱浸透率比分配」。剛体ごとの散逸を接触ペアへ均等配分してから、
    /// ペアごとに $Q_A/Q_B=e_{t,A}/e_{t,B}$ で2ノードへ配る(モジュールdoc参照)。
    /// 対応表に無い剛体の分・どちらも未登録のペアの分は既定ノードへ落とす。
    /// 戻り値の熱量の総和は`total`と厳密に一致する(丸め誤差を除く)。
    fn distribute_by_effusivity(&self, world: &DomainStates, total: f64) -> Vec<(usize, f64)> {
        let per_body = &world.mechanics.last_contact_dissipation_by_body;
        let manifolds = &world.mechanics.last_manifolds;
        let mut out: Vec<(usize, f64)> = Vec::new();
        let mut add = |node: usize, heat: f64| {
            if let Some(entry) = out.iter_mut().find(|(n, _)| *n == node) {
                entry.1 += heat;
            } else {
                out.push((node, heat));
            }
        };

        // 1) 剛体ごとの散逸を、その剛体が関与するマニフォールドへ均等配分する。
        let mut pair_heat: Vec<(usize, usize, f64)> = manifolds
            .iter()
            .map(|m| (m.body_a, m.body_b, 0.0))
            .collect();
        let mut attributed = 0.0;
        for (body, &loss) in per_body.iter().enumerate() {
            if loss == 0.0 {
                continue;
            }
            let touching: Vec<usize> = (0..pair_heat.len())
                .filter(|&k| pair_heat[k].0 == body || pair_heat[k].1 == body)
                .collect();
            if touching.is_empty() {
                // 接触マニフォールドが無いのに散逸した(スリープ判定でスキップされた
                // ペア等)。配分先が決まらないので既定ノードへ。
                add(self.thermal_node, loss);
                attributed += loss;
                continue;
            }
            let share = loss / touching.len() as f64;
            for k in touching {
                pair_heat[k].2 += share;
            }
            attributed += loss;
        }

        // 2) ペアごとに熱浸透率比で2ノードへ配る(設計 §4.4)。
        for (a, b, heat) in pair_heat {
            if heat == 0.0 {
                continue;
            }
            let link_a = self.link_of(a);
            let link_b = self.link_of(b);
            let e_a = link_a.map_or(0.0, |l| l.effusivity.max(0.0));
            let e_b = link_b.map_or(0.0, |l| l.effusivity.max(0.0));
            let sum = e_a + e_b;
            if sum <= 0.0 {
                // 両方未登録(または熱浸透率0)。既定ノードへ落とす。
                add(self.thermal_node, heat);
                continue;
            }
            match link_a {
                Some(l) => add(l.thermal_node, heat * e_a / sum),
                None => add(self.thermal_node, heat * e_a / sum),
            }
            match link_b {
                Some(l) => add(l.thermal_node, heat * e_b / sum),
                None => add(self.thermal_node, heat * e_b / sum),
            }
        }

        // 3) 剛体ごとの内訳が取れなかった残り(`last_contact_dissipation_by_body`が
        //    空の場合など)は既定ノードへ。これで総和が必ず`total`に一致する。
        let residual = total - attributed;
        if residual != 0.0 {
            add(self.thermal_node, residual);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
    use sim_math::{SimRng, Vec3};
    use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// 摩擦で滑走→静止する箱の運動エネルギー損失が、`DissipationToHeat`経由で
    /// 単一の熱ノードの温度上昇(C・ΔT)としておおむね過不足なく計上されることを確認する
    /// (設計§1「保存量の橋」の対記帳、docs/00-foundation/04-architecture.md §1.1.2(2))。
    /// **許容誤差はrel<2%**(QA不具合1の修正で 15% から締めた)。以前は
    /// `MechanicsSolver::last_contact_dissipation`の累積和が実際の力学的エネルギー
    /// 総損失を系統的に約9%上回っていたため 15% を採っていた。原因は
    /// Baumgarte ではなく**測定窓に重力の速度増分が入っていたこと**で、
    /// 床に置かれた剛体が毎step $\frac12 m(g\Delta t)^2$ を散逸として計上して
    /// いた(`MechanicsSolver::external_velocity_hold_energy`のdoc参照)。
    /// その寄与を除いた今は誤差が桁で縮んだので、対記帳が「概ね」ではなく
    /// 実際に閉じていることを確認できる水準まで締める。
    #[test]
    fn dissipation_to_heat_pairs_kinetic_energy_loss_with_thermal_node_heat_gain() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut floor_desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        floor_desc.body_type = BodyType::Static;
        mechanics.create_body(floor_desc, &materials);

        let mut box_desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        box_desc.transform.position = Vec3::new(0.0, 0.5, 0.0);
        box_desc.linear_velocity = Vec3::new(3.0, 0.0, 0.0);
        let box_idx = mechanics.create_body(box_desc, &materials);

        // 比較対象は水平運動エネルギーの理論値(0.5*m*v0^2)ではなく、実際の力学的エネルギー
        // (運動+重力ポテンシャル)の初期値を使う — 箱の初期姿勢(底面がちょうど床に接する
        // y=0.5)ではわずかな沈み込み・跳ね(垂直方向のsettling)が生じ、その分の重力
        // ポテンシャルエネルギーも接触解決で散逸するため(実装検証中に発見: 水平KEの
        // 理論値だけと比較するとheat_gainedが約9%過大になった、settling分の散逸が
        // 加算されるため)、力学的エネルギーの総量で比較するのが正しい対記帳の検証になる。
        let mechanical_energy_0 = mechanics.total_energy().total();

        let mut thermal = ThermalSolver::new(293.15);
        let floor_node = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let mut coupling = DissipationToHeat::to_single_node(floor_node);

        let dt = 1.0 / 120.0;
        for _ in 0..1200 {
            // 10秒: 摩擦(鋼-鋼)で確実に静止するのに十分な時間。
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            mechanics.step(dt, &mut ctx);
            {
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
            let mut ctx2 = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            thermal.step(dt, &mut ctx2);
        }

        assert!(
            mechanics.bodies.linear_velocity[box_idx].length() < 0.01,
            "box should have come to rest via friction: v={:?}",
            mechanics.bodies.linear_velocity[box_idx]
        );

        let mechanical_energy_lost = mechanical_energy_0 - mechanics.total_energy().total();
        let final_temp = thermal.nodes[floor_node].temperature;
        let heat_gained = 1000.0 * (final_temp - 293.15);
        let rel_err = (heat_gained - mechanical_energy_lost).abs() / mechanical_energy_lost;
        assert!(
            rel_err < 0.02,
            "heat_gained={heat_gained:.4} mechanical_energy_lost={mechanical_energy_lost:.4} rel_err={rel_err:.4}"
        );
    }

    /// **静止した剛体は発熱しない**(QA不具合1の回帰テスト)。
    ///
    /// D10(摩擦の熱)の箱は step 61 で止まるのに、熱ノードの温度は step 121 まで
    /// 上がり続けていた(330.13 → 331.70 K、+1,573 J)。増分は 26.2 J/step で、
    /// これは $\frac12 m(g\Delta t)^2$ と一致する——**接触の法線インパルスが
    /// 毎step打ち消す重力ぶんの速度増分が散逸として計上されていた**。
    /// 発熱が止まったのは物理的な理由ではなく、0.5 秒後にスリープが接触解決ごと
    /// 止めるからでしかなかった。
    ///
    /// ここでは**スリープに入る前**の区間を見る——スリープが隠してしまう前に
    /// 「静止しているのに熱が湧く」ことを直接捕まえるため。初速 0 で床に置いた
    /// 箱を、スリープ閾値(`SLEEP_TIME_THRESHOLD` = 0.5 s)より短い 0.25 秒だけ
    /// 進めて、その間の散逸の累積が(重力ぶんの偽の寄与ではなく)ほぼ 0 で
    /// あることを確認する。修正前はここで 26.2 J/step × 30 step ≈ 786 J が出る。
    #[test]
    fn a_body_resting_on_the_floor_dissipates_no_heat_before_it_falls_asleep() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut floor_desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        floor_desc.body_type = BodyType::Static;
        mechanics.create_body(floor_desc, &materials);

        // 初速 0 で床にちょうど乗せる(滑走も落下もしない = 散逸すべき量が無い)。
        let mut box_desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        box_desc.transform.position = Vec3::new(0.0, 0.5, 0.0);
        let box_idx = mechanics.create_body(box_desc, &materials);
        let mass = mechanics.bodies.mass(box_idx);

        let dt = 1.0 / 120.0;
        // 0.25 s < SLEEP_TIME_THRESHOLD(0.5 s)なのでまだ眠らない。
        let steps = 30;
        let mut per_step: Vec<f64> = Vec::with_capacity(steps);
        for _ in 0..steps {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            mechanics.step(dt, &mut ctx);
            per_step.push(mechanics.last_contact_dissipation);
        }
        // **最初の数stepは着地の過渡**なので除く。y=0.5 は「ちょうど触れる」
        // 位置で貫入が 0 なので、1step目はまだ接触が検出されず箱は自由落下し、
        // 次のstepで実際に着地する——この着地は**本物の非弾性衝突で、散逸するのが
        // 正しい**(既存テストのコメントが言う「わずかな沈み込み・跳ね」)。
        // 見たいのはその後の「静止して乗っているだけ」の区間である。
        let settle = 10;
        let resting: f64 = per_step[settle..].iter().sum();

        // まだ眠っていない(眠っていたら接触解決が止まって当たり前に0になり、
        // この検証が意味を失う)。
        assert!(
            !mechanics.bodies.asleep[box_idx],
            "この検証はスリープ前の区間を見る前提なので、まだ起きているべき"
        );
        // 修正前の偽の寄与(1step ぶん)= 26.2 J。静止区間 20 step ぶんの累積が
        // **その 1 step ぶんにも満たない**ことを要求する(修正前はここに
        // 26.2 × 20 = 524 J が出ていた)。
        let spurious_per_step = 0.5 * mass * (9.80665 * dt).powi(2);
        assert!(
            resting < spurious_per_step,
            "床に静止した箱は発熱しないべき: step {settle}..{steps} の {} step で \
             {resting} J 散逸した(重力ぶんの偽の寄与は 1step あたり \
             {spurious_per_step} J で、修正前はこの区間に {} J 出ていた)。\
             per_step={per_step:?}",
            steps - settle,
            spurious_per_step * (steps - settle) as f64
        );
    }

    /// **群5: 熱浸透率比分配**(設計 docs/12-thermal/02-heat-transfer.md §4.4)。
    /// 鋼の箱が鋼の床の上を滑って止まるシナリオで、散逸熱が箱ノードと床ノードへ
    /// $e_{t,A}:e_{t,B}$ の比で配られること、総熱量が単一ノード版とまったく同じに
    /// なること(配分の内訳が変わるだけで総量は保存されること)を確認する。
    ///
    /// 比を検出可能にするため、床側の熱浸透率を箱側の3倍に設定する(同じ材料どうしだと
    /// 1:1 になり、比を取り違えても気付けない)。
    #[test]
    fn dissipation_to_heat_splits_between_two_nodes_by_effusivity_ratio() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

        // 単一ノード版と2ノード版で、まったく同じ力学シナリオを2回走らせる。
        let run = |links: Vec<BodyThermalLink>| -> (f64, f64, f64) {
            let mut rng = SimRng::new(1, 1);
            let mut events = EventQueue::new();
            let mut mechanics = MechanicsSolver::new(9.80665);
            let mut floor_desc = RigidBodyDesc::dynamic(
                Shape::Plane {
                    normal: Vec3::new(0.0, 1.0, 0.0),
                    d: 0.0,
                },
                steel,
            );
            floor_desc.body_type = BodyType::Static;
            let floor = mechanics.create_body(floor_desc, &materials);
            let mut box_desc = RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(0.5, 0.5, 0.5),
                },
                steel,
            );
            box_desc.transform.position = Vec3::new(0.0, 0.5, 0.0);
            box_desc.linear_velocity = Vec3::new(5.0, 0.0, 0.0);
            let boxed = mechanics.create_body(box_desc, &materials);
            assert_eq!((floor, boxed), (0, 1));

            let mut thermal = ThermalSolver::new(293.15);
            let env = thermal.add_node(ThermalNode::new(293.15, 1000.0));
            let box_node = thermal.add_node(ThermalNode::new(293.15, 1000.0));
            let floor_node = thermal.add_node(ThermalNode::new(293.15, 2000.0));
            // テスト側で対応表のノード番号を組み立て直す(引数は body_index と
            // effusivity だけを指定してもらう)。
            let links: Vec<BodyThermalLink> = links
                .into_iter()
                .map(|l| BodyThermalLink {
                    thermal_node: if l.body_index == boxed {
                        box_node
                    } else {
                        floor_node
                    },
                    ..l
                })
                .collect();
            let mut coupling = DissipationToHeat {
                thermal_node: env,
                body_links: links,
            };

            let dt = 1.0 / 120.0;
            for _ in 0..600 {
                let mut ctx = SolverContext {
                    materials: &materials,
                    rng: &mut rng,
                    events: &mut events,
                };
                mechanics.step(dt, &mut ctx);
                coupling.apply(
                    &mut DomainStates {
                        mechanics: &mut mechanics,
                        thermal: Some(&mut thermal),
                        em_circuit: None,
                        em_electrostatics: None,
                        gas: None,
                        grid_fluid: None,
                        grid_fluid_3d: None,
                        sph: None,
                    },
                    dt,
                );
            }
            let heat_of = |node: usize, c: f64| c * (thermal.nodes[node].temperature - 293.15);
            (
                heat_of(env, 1000.0),
                heat_of(box_node, 1000.0),
                heat_of(floor_node, 2000.0),
            )
        };

        // ① 対応表なし(移行前の挙動): 全量が環境ノードへ。
        let (env_only, box_only, floor_only) = run(Vec::new());
        assert!(env_only > 0.0, "摩擦で熱が出るはず: {env_only}");
        assert_eq!((box_only, floor_only), (0.0, 0.0));

        // ② 対応表あり: 箱 e=1、床 e=3 → 箱 1/4、床 3/4。
        let (env_split, box_heat, floor_heat) = run(vec![
            BodyThermalLink {
                body_index: 1, // boxed
                thermal_node: 0,
                effusivity: 1.0,
            },
            BodyThermalLink {
                body_index: 0, // floor
                thermal_node: 0,
                effusivity: 3.0,
            },
        ]);

        // 総熱量は単一ノード版と厳密に一致(力学は同一、配分先だけが違う)。
        let total_split = env_split + box_heat + floor_heat;
        assert!(
            (total_split - env_only).abs() / env_only < 1e-9,
            "総熱量は配分方法によらず同じはず: single={env_only} split={total_split}"
        );
        // 環境ノードへは(両方とも対応表にあるので)ほとんど落ちない。
        assert!(
            env_split.abs() / total_split < 1e-9,
            "両方登録済みなら環境ノードへは落ちないはず: env={env_split}"
        );
        // 設計 §4.4 の比 Q_A/Q_B = e_A/e_B = 1/3。
        let ratio = box_heat / floor_heat;
        assert!(
            (ratio - 1.0 / 3.0).abs() < 1e-9,
            "熱浸透率比 1:3 で配られるはず: box={box_heat} floor={floor_heat} ratio={ratio}"
        );
    }

    /// `effusivity`ヘルパーが $\sqrt{k\rho c_p}$ そのものであること、鋼と木で
    /// 実際に桁の違う値になること(比分配が意味を持つこと)を確認する。
    #[test]
    fn effusivity_helper_matches_the_square_root_of_k_rho_cp() {
        let materials = MaterialDb::standard();
        let steel = materials.get(materials.find_by_name("鋼(炭素鋼)").unwrap());
        let wood = materials.get(materials.find_by_name("木材(松)").unwrap());
        let e_steel = effusivity(steel.conductivity, steel.density, steel.specific_heat);
        let e_wood = effusivity(wood.conductivity, wood.density, wood.specific_heat);
        assert!(
            (e_steel - (steel.conductivity * steel.density * steel.specific_heat).sqrt()).abs()
                < 1e-9
        );
        assert!(
            e_steel > 5.0 * e_wood,
            "鋼の熱浸透率は木材よりずっと大きいはず: steel={e_steel} wood={e_wood}"
        );
    }
}
