//! 回路 — 修正節点解析(MNA)。設計: docs/13-electromagnetism/02-circuits.md。
//!
//! P4 スコープの実装: 線形素子(抵抗・コンデンサ・インダクタ・独立電圧源)のMNA(設計 §3)+
//! ダイオード(Shockley式)のNewton-Raphson反復(設計 §4)。動的素子(C・L)は後退Euler
//! (設計 §4「既定」)のコンパニオンモデルへ変換して代数化する。ダイオードも同様に
//! 各Newton反復で現在の動作点まわりの線形コンパニオンモデル(微分コンダクタンス+
//! 等価電流源、SPICE標準)へ変換する。
//!
//! **群7で「フォールバック連鎖」と「モーター」を実装した**(設計§4・§2)。
//! 移行前は段1(電圧ステップ制限つきNewton)だけで、**上限反復に達しても
//! 収束していない解をそのまま採用していた**——半波整流のテストケースが確実に
//! 収束することに依存しており、収束しない回路には黙って嘘の解を返す状態だった。
//! 群7で段2(振動ダンピング)・段3(gmin stepping)・段4(source stepping)・
//! 段5(ラッチ + `SolverDiverged`)を実装し、収束判定も設計どおり
//! 「$\Delta v<10^{-9}$V **かつ** KCL残差 $<10^{-9}$A」の2条件にした
//! (`Circuit::step`のdoc参照)。
//!
//! **設計から離れた点を2つ、実測にもとづいて明記する**:
//! 1. 電圧ステップ制限は設計の「$2nV_T$への素朴なクランプ」ではなく**SPICE標準の
//!    `pnjlim`**を使う。素朴なクランプでは冷間開始(全ノード0V)から動作点0.6Vへ
//!    52mV刻みで届かず、設計の上限10反復では**構造的に収束しない**
//!    (`pn_junction_limit`のdoc参照)。
//! 2. 反復上限は通常段が設計どおり10、**フォールバック段は100**
//!    (`FALLBACK_MAX_NEWTON_ITERATIONS`のdoc参照)。フォールバック段は連続化
//!    パラメータを動かすDC動作点解析であり、SPICEも同様に分けている。
//!
//! DCモーターは既存素子(抵抗+インダクタ+逆起電力の電圧源)の**直列等価回路**として
//! 実装した(`add_dc_motor`)。スイッチは理想スイッチの2値抵抗近似(ファイル後方の
//! `SWITCH_ON_RESISTANCE`/`SWITCH_OFF_RESISTANCE`)で実装済み。線形方程式は毎回
//! 部分ピボット付きガウス消去で解く(回路規模が小さいため十分、設計 §10「密LUで十分」)。
//!
//! `Solver`トレイト実装(`sim-coupling::JouleHeat`が`World`経由で駆動するための窓口、
//! 設計docs/00-foundation/04-architecture.md §1.2)はファイル末尾。

use sim_core::{Approximation, EnergyBreakdown, Solver, SolverContext, StateHasher};

/// ノード0は常にグラウンド(電位0、未知数に含めない)。設計 §3。
pub const GROUND: usize = 0;

/// ダイオードのNewton反復上限(設計§4「上限10反復」)。
const DIODE_MAX_NEWTON_ITERATIONS: usize = 10;
/// 収束判定: 電圧ステップがこれ未満なら収束とみなす(設計§4「Δv<10⁻⁹V」)。
const DIODE_CONVERGENCE_TOLERANCE: f64 = 1e-9;

/// 理想スイッチの近似(モーターは群7で`add_dc_motor`として実装済み、
/// `sim-world::Command::SetSwitch`が使う)。専用の未知数(電圧源のような)を追加せず、
/// 閉:低抵抗(ほぼ短絡)・開:高抵抗(ほぼ開放)の2値抵抗として`resistors`と同じ
/// `stamp_conductance`経路でスタンプする(最小の実装)。抵抗比(1e-6Ω/1e9Ω、15桁)は
/// 倍精度ガウス消去が悪条件化せずに解ける範囲で選んだ(既存のダイオード・回路規模
/// テストと同程度の条件数)。
const SWITCH_ON_RESISTANCE: f64 = 1e-6;
const SWITCH_OFF_RESISTANCE: f64 = 1e9;

/// KCL残差の収束判定の**絶対項**(設計§4「KCL残差 < 10⁻⁹ A」、**群7で追加**)。
const KCL_RESIDUAL_TOLERANCE: f64 = 1e-9;
/// KCL残差の収束判定の**相対項**(SPICEの`reltol`相当、**群7**)。
/// 悪条件な行列(理想スイッチの15桁の抵抗比など)では倍精度の丸め誤差だけで
/// 絶対項を超えるため必要になる(`kcl_residual`のdoc参照)。
const KCL_RESIDUAL_RELATIVE: f64 = 1e-9;
/// 振動と判定するΔvの符号反転回数(設計§4「符号が3回交互」、**群7**)。
const OSCILLATION_FLIP_THRESHOLD: u32 = 3;
/// 振動検出時のダンピング係数(設計§4「ダンピング0.5」、**群7**)。
const OSCILLATION_DAMPING: f64 = 0.5;
/// gmin stepping の段数(設計§4「10⁻³→10⁻¹² Sまで10段階」、**群7**)。
const GMIN_STEPPING_STAGES: usize = 10;
/// source stepping の段数(設計§4「0から10段階で連続化」、**群7**)。
const SOURCE_STEPPING_STAGES: usize = 10;
/// フォールバック段(gmin/source stepping)1段あたりのNewton反復上限(**群7**)。
///
/// 設計§4の「上限10反復」は**通常段**(前stepの解から出発する過渡解析)の値である。
/// フォールバック段は連続化パラメータを動かしながら解き直すため、実質的に
/// **DC動作点解析**であり、SPICEも同様にこちらへ大きい上限(ITL1=100)を割り当てる。
/// 実装検証で確認したこと: 冷間開始の1step目は通常段の10反復では**あと2反復**
/// 足りず(実測: max_step 3.1e-5・KCL残差 3.1e-9 まで来て打ち切り)、
/// この上限が無いとフォールバック段も同じ理由で全滅する。
const FALLBACK_MAX_NEWTON_ITERATIONS: usize = 100;

/// `Circuit::add_dc_motor`が返すハンドル(**群7で追加**)。逆起電力の更新
/// (`set_motor_speed`)と電流の読み出し(`motor_current`)に使う。
#[derive(Clone, Copy, Debug)]
pub struct MotorHandle {
    /// 逆起電力を表す電圧源のindex。
    source_index: usize,
    /// トルク定数 = 逆起電力定数 $k_e=k_t$ [V·s/rad = N·m/A]。
    pub back_emf_constant: f64,
}

/// 回路。素子はノード番号の対 `(a, b)` で接続を表す(a, b どちらも `GROUND` を含みうる)。
#[derive(Default, Clone)]
pub struct Circuit {
    num_nodes: usize,
    resistors: Vec<(usize, usize, f64)>,
    capacitors: Vec<(usize, usize, f64)>,
    inductors: Vec<(usize, usize, f64)>,
    voltage_sources: Vec<(usize, usize, f64)>,
    /// (anode, cathode, saturation_current, n・V_T)。設計§2「Shockley $i=I_s(e^{v/nV_T}-1)$」。
    diodes: Vec<(usize, usize, f64, f64)>,
    /// (a, b, closed)。理想スイッチの近似(モジュールdoc参照)。
    switches: Vec<(usize, usize, bool)>,
    /// 前ステップの端子間電圧(コンデンサの後退Eulerコンパニオンモデルの履歴項)。
    capacitor_voltage: Vec<f64>,
    /// 前ステップの枝電流(インダクタの後退Eulerコンパニオンモデルの履歴項)。
    inductor_current: Vec<f64>,
    /// ダイオードの動作点電圧(Newton反復の現在推定値、次ステップのウォームスタートにも使う)。
    diode_voltage: Vec<f64>,
    /// 直近の解(ノード電圧、`node_voltage` で参照する)。
    last_node_voltage: Vec<f64>,
    /// 直近の解(電圧源の枝電流)。
    last_source_current: Vec<f64>,
    /// 直近の`step`でフォールバック連鎖の全段が失敗したか(**群7で追加**、
    /// `step`のdoc「段5」参照)。`Solver::step`がこれを見て`SolverDiverged`を発行する。
    last_solve_diverged: bool,
}

impl Circuit {
    /// `num_nodes` はグラウンドを含むノード総数(ノード番号は `0..num_nodes`)。
    pub fn new(num_nodes: usize) -> Circuit {
        Circuit {
            num_nodes,
            last_node_voltage: vec![0.0; num_nodes],
            ..Default::default()
        }
    }

    pub fn add_resistor(&mut self, a: usize, b: usize, resistance: f64) {
        self.resistors.push((a, b, resistance));
    }

    /// 初期端子間電圧 `initial_voltage`(未充電なら0)。
    pub fn add_capacitor(&mut self, a: usize, b: usize, capacitance: f64, initial_voltage: f64) {
        self.capacitors.push((a, b, capacitance));
        self.capacitor_voltage.push(initial_voltage);
    }

    /// 初期電流 `initial_current`(a→b方向を正とする)。
    pub fn add_inductor(&mut self, a: usize, b: usize, inductance: f64, initial_current: f64) {
        self.inductors.push((a, b, inductance));
        self.inductor_current.push(initial_current);
    }

    /// 独立電圧源。`a` が正極、`b` が負極(`v_a - v_b = voltage`)。
    pub fn add_voltage_source(&mut self, a: usize, b: usize, voltage: f64) {
        self.voltage_sources.push((a, b, voltage));
    }

    /// 既存の電圧源の値を変更する(`add_voltage_source` を呼んだ順のインデックス)。
    /// 時間変化する電源(AC等)を`step`の呼び出し間で表現するために使う。
    pub fn set_voltage_source_voltage(&mut self, index: usize, voltage: f64) {
        self.voltage_sources[index].2 = voltage;
    }

    /// ダイオード(Shockley式、設計§2)。`anode`→`cathode`が順方向。
    /// `n_vt` は $nV_T$(理想係数×熱電圧、300Kで$V_T\approx25.85$mV)。
    pub fn add_diode(&mut self, anode: usize, cathode: usize, saturation_current: f64, n_vt: f64) {
        self.diodes.push((anode, cathode, saturation_current, n_vt));
        self.diode_voltage.push(0.0);
    }

    /// 理想スイッチの近似(モジュールdoc参照)。戻り値は`set_switch_closed`用のインデックス。
    /// **DCモーターを素子として追加する**(**群7で追加**、設計
    /// docs/13-electromagnetism/02-circuits.md §2 の表
    /// 「$v=Ri+L\,di/dt+k_e\omega$(逆起電力)/ $\tau=k_ti$」)。
    ///
    /// 電気側は**巻線抵抗$R$ + 巻線インダクタンス$L$ + 逆起電力$k_e\omega$の直列**で、
    /// 既存の素子(抵抗・インダクタ・電圧源)3つを内部ノードで直列につないだ
    /// **等価回路**として組む。逆起電力は電圧源として入り、機械側の角速度$\omega$が
    /// 決まるたび`set_motor_speed`で更新する。
    ///
    /// **なぜ専用の未知数を足さないのか**: MNAの行を増やすと、既存のスタンプ・
    /// 解の取り出し・`Solver`実装のすべてに分岐が増える。DCモーターの電気側は
    /// 線形素子3つの直列と**厳密に等価**なので、等価回路で組むほうが実装量も
    /// 検証の手間も小さい(トルク$\tau=k_ti$は`sim-coupling::MotorCoupling`が
    /// 電流から作るので回路側は電流を返せば足りる)。
    ///
    /// 戻り値は`MotorHandle`(逆起電力の更新と電流の読み出しに使う)。
    /// `internal_node`には**まだどの素子も繋がっていない未使用のノード番号**を
    /// 渡すこと(`Circuit::new`で確保した節点数の中から選ぶ)。
    pub fn add_dc_motor(
        &mut self,
        a: usize,
        b: usize,
        internal_nodes: (usize, usize),
        winding_resistance: f64,
        winding_inductance: f64,
        back_emf_constant: f64,
    ) -> MotorHandle {
        let (n1, n2) = internal_nodes;
        assert!(n1 != GROUND && n2 != GROUND, "内部ノードはGND以外");
        assert!(n1 != n2, "内部ノードは互いに異なる必要がある");
        assert!(
            n1 < self.num_nodes && n2 < self.num_nodes,
            "内部ノードが節点数の範囲外"
        );
        // **直列**の等価回路: a --[逆起電力源]-- n1 --[R]-- n2 --[L]-- b。
        // **内部ノードは2つ要る**。実装検証中に1つで済ませようとして R と L を
        // 同じ2ノード間に置き、**並列**になっていた(定常でLが短絡し、拘束電流が
        // R で決まらず桁違いになった)。素子の直列接続はノードを介してしか
        // 表現できない、というMNAの基本を踏み外した形。
        self.add_voltage_source(a, n1, 0.0);
        let source_index = self.voltage_sources.len() - 1;
        self.add_resistor(n1, n2, winding_resistance);
        self.add_inductor(n2, b, winding_inductance, 0.0);
        MotorHandle {
            source_index,
            back_emf_constant,
        }
    }

    /// モーターの角速度を更新する(逆起電力 $\mathcal{E}=k_e\omega$ を電圧源へ反映、
    /// **群7**、`add_dc_motor`のdoc参照)。
    pub fn set_motor_speed(&mut self, motor: MotorHandle, angular_velocity: f64) {
        let emf = motor.back_emf_constant * angular_velocity;
        self.set_voltage_source_voltage(motor.source_index, emf);
    }

    /// モーターを流れる電流(トルク $\tau=k_t i$ の元、**群7**)。
    /// 符号は**aからbへ流れる向きを正**とする(MNAの枝電流の向きがそのまま
    /// これに一致することを実測で確認した)。
    pub fn motor_current(&self, motor: MotorHandle) -> f64 {
        self.source_current(motor.source_index)
    }

    pub fn add_switch(&mut self, a: usize, b: usize, closed: bool) -> usize {
        self.switches.push((a, b, closed));
        self.switches.len() - 1
    }

    /// 開閉状態を変更する(`sim-world::Command::SetSwitch`が使う)。
    pub fn set_switch_closed(&mut self, index: usize, closed: bool) {
        self.switches[index].2 = closed;
    }

    /// 実際に配線されている素子を読み出すためのアクセサ群(**増分G2で追加**)。
    ///
    /// **追加した理由**: フロントエンドのCircuitタブが固定デモ回路の図
    /// (`Node1 (10V 電源) --[100Ω]-- Node2 --[200Ω]-- GND`)をハードコードで
    /// 描いており、シーンギャラリーから別の回路を読み込んでも**その図がそのまま
    /// 残って実際とは違う値を表示していた**。「無効です」という注記は出るものの、
    /// 数字自体は嘘のままだった。実際に載っている素子を列挙できる手段が無いこと
    /// が原因なので、ここに最小限の読み出しを足す。設計
    /// docs/18-frontend/02-panels.md のHierarchy「Circuits」サブツリーも
    /// 同じAPIを必要とする。
    pub fn num_nodes(&self) -> usize {
        self.num_nodes
    }

    /// `(a, b, resistance)`。
    pub fn resistors(&self) -> &[(usize, usize, f64)] {
        &self.resistors
    }

    /// `(a, b, capacitance)`。
    pub fn capacitors(&self) -> &[(usize, usize, f64)] {
        &self.capacitors
    }

    /// `(a, b, inductance)`。
    pub fn inductors(&self) -> &[(usize, usize, f64)] {
        &self.inductors
    }

    /// `(a, b, voltage)`。インデックスが`source_current`の引数と対応する。
    pub fn voltage_sources(&self) -> &[(usize, usize, f64)] {
        &self.voltage_sources
    }

    /// `(anode, cathode, saturation_current, n_vt)`。
    pub fn diodes(&self) -> &[(usize, usize, f64, f64)] {
        &self.diodes
    }

    /// `(a, b, closed)`。インデックスが`set_switch_closed`の引数と対応する。
    pub fn switches(&self) -> &[(usize, usize, bool)] {
        &self.switches
    }

    /// `node`が現在の回路のノード数を超える場合は0を返す(パニックしない)。
    ///
    /// **修正の経緯(シーンギャラリー増分B2で発見)**: 以前は
    /// `self.last_node_voltage[node]`の直接インデックスで、範囲外ノードを渡すと
    /// パニックしていた(`last_node_voltage`は`new()`で`vec![0.0; num_nodes]`に
    /// 初期化されるため、step()未呼び出しでは0を返せていた——docが保証していたのは
    /// そちらの経路のみで、範囲外は単に未処理だった)。シーンギャラリーでD21
    /// (銅管落下、`num_nodes=2`なのでノードは0/1のみ)を読み込むと、HUDが毎フレーム
    /// 呼ぶ`sim-wasm::circuit_divider_voltage`が固定デモ回路前提のノード番号2
    /// (`CIRCUIT_DIVIDER_NODE`)を無条件に読みに行き、この範囲外パニックを踏むことを
    /// Playwrightでの目視確認中に発見した。兄弟の`source_current`は既に
    /// `.get().copied().unwrap_or(0.0)`で範囲外を許容しており、こちらを揃えた形。
    pub fn node_voltage(&self, node: usize) -> f64 {
        if node == GROUND {
            0.0
        } else {
            self.last_node_voltage.get(node).copied().unwrap_or(0.0)
        }
    }

    /// dt 進める。MNA 行列を毎回組み立てて解く(線形素子のみなので行列自体は
    /// dt・素子値で決まり時間不変だが、キャッシュは未実装、設計 §10 の性能課題として残す)。
    /// 線形部分の MNA 行列と右辺を組み立てる(**群7で`step`から切り出した**)。
    /// `source_scale`は独立源のスケール(source stepping、`step`のdoc参照)。
    /// 通常は`1.0`。
    fn build_linear_system(&self, dt: f64, source_scale: f64) -> (Vec<Vec<f64>>, Vec<f64>) {
        let n_node_unknowns = self.num_nodes.saturating_sub(1); // GND を除く
        let n_extra = self.voltage_sources.len() + self.inductors.len();
        let n = n_node_unknowns + n_extra;

        let mut a_mat = vec![vec![0.0_f64; n]; n];
        let mut b_vec = vec![0.0_f64; n];

        let node_idx = |node: usize| -> Option<usize> {
            if node == GROUND {
                None
            } else {
                Some(node - 1)
            }
        };

        let stamp_conductance = |a_mat: &mut Vec<Vec<f64>>, a: usize, b: usize, g: f64| {
            if let Some(ia) = node_idx(a) {
                a_mat[ia][ia] += g;
            }
            if let Some(ib) = node_idx(b) {
                a_mat[ib][ib] += g;
            }
            if let (Some(ia), Some(ib)) = (node_idx(a), node_idx(b)) {
                a_mat[ia][ib] -= g;
                a_mat[ib][ia] -= g;
            }
        };

        for &(a, b, r) in &self.resistors {
            stamp_conductance(&mut a_mat, a, b, 1.0 / r);
        }

        for &(a, b, closed) in &self.switches {
            let r = if closed {
                SWITCH_ON_RESISTANCE
            } else {
                SWITCH_OFF_RESISTANCE
            };
            stamp_conductance(&mut a_mat, a, b, 1.0 / r);
        }

        // コンデンサ: 後退Eulerコンパニオンモデル(設計 §4)。等価コンダクタンス G_c=C/dt を
        // 抵抗と同じ形でスタンプし、前ステップ電圧による等価電流源 G_c・v_prev を
        // ノードaへ注入する(a→bを正方向とする電圧の定義に合わせた符号)。
        for (idx, &(a, b, c)) in self.capacitors.iter().enumerate() {
            let g_c = c / dt;
            stamp_conductance(&mut a_mat, a, b, g_c);
            let i_eq = g_c * self.capacitor_voltage[idx];
            if let Some(ia) = node_idx(a) {
                b_vec[ia] += i_eq;
            }
            if let Some(ib) = node_idx(b) {
                b_vec[ib] -= i_eq;
            }
        }

        // 電圧源・インダクタは枝電流を追加の未知数として持つ(設計 §3 の j)。
        // 行 K(拘束式): v_a - v_b - d・j = rhs。列K(KCL結合): ノードaへ+1・j、ノードbへ-1・j。
        let mut extra_idx = n_node_unknowns;
        for &(a, b, voltage) in &self.voltage_sources {
            let k = extra_idx;
            extra_idx += 1;
            if let Some(ia) = node_idx(a) {
                a_mat[ia][k] += 1.0;
                a_mat[k][ia] += 1.0;
            }
            if let Some(ib) = node_idx(b) {
                a_mat[ib][k] -= 1.0;
                a_mat[k][ib] -= 1.0;
            }
            b_vec[k] = voltage * source_scale;
        }
        for (idx, &(a, b, inductance)) in self.inductors.iter().enumerate() {
            let k = extra_idx;
            extra_idx += 1;
            if let Some(ia) = node_idx(a) {
                a_mat[ia][k] += 1.0;
                a_mat[k][ia] += 1.0;
            }
            if let Some(ib) = node_idx(b) {
                a_mat[ib][k] -= 1.0;
                a_mat[k][ib] -= 1.0;
            }
            let l_over_dt = inductance / dt;
            a_mat[k][k] -= l_over_dt;
            b_vec[k] = -l_over_dt * self.inductor_current[idx];
        }

        (a_mat, b_vec)
    }

    /// **非線形収束のフォールバック連鎖**(設計 docs/13-electromagnetism/02-circuits.md §4、
    /// **群7で実装**)。段数・刻みはすべてコンパイル時定数で、収束の速い回路でも遅い
    /// 回路でも**実行列は入力の決定的関数**になる(壁時計依存なし、設計の決定論規約)。
    ///
    /// 1. **通常**: 電圧ステップ制限つきNewton(1反復あたり$2nV_T\approx52$mV)。
    ///    収束判定は $\Delta v<10^{-9}$V **かつ** KCL残差 $<10^{-9}$A。上限10反復。
    /// 2. **振動検出**($\Delta v$の符号が3回交互): ダンピング0.5を掛けて継続。
    /// 3. **上限到達 → gmin stepping**: 各非線形素子に並列コンダクタンス $g_{min}$ を
    ///    $10^{-3}\to10^{-12}$ Sまで10段階で減らしつつ逐次解く(SPICE標準)。
    /// 4. **なお失敗 → source stepping**: 独立源を0から10段階で連続化して逐次解く。
    /// 5. **全段失敗**: 前sub-stepの解を保持(ラッチ)し`last_solve_diverged`を立てる
    ///    (`Solver::step`が`EventKind::SolverDiverged`を発行する。黙って進まない)。
    ///
    /// 移行前は段1(電圧ステップ制限つきNewton)だけで、上限に達しても**そのまま
    /// 収束していない解を採用していた**——半波整流のテストケースが確実に収束する
    /// ことに依存しており、収束しない回路を与えたときに黙って嘘の解を返す状態だった。
    pub fn step(&mut self, dt: f64) {
        self.last_solve_diverged = false;
        let (a_mat, b_vec) = self.build_linear_system(dt, 1.0);
        if self.diodes.is_empty() {
            // 線形回路: フォールバック連鎖は不要(1回のガウス消去で厳密解)。
            let x = solve_linear_system(a_mat, b_vec);
            self.commit_solution(&x);
            return;
        }

        // 段1・2: 通常のNewton(振動ダンピング込み)。
        let entry_diode_voltage = self.diode_voltage.clone();
        if let Some(x) = self.newton_attempt(&a_mat, &b_vec, 0.0, DIODE_MAX_NEWTON_ITERATIONS) {
            self.commit_solution(&x);
            return;
        }

        // 段3: gmin stepping。
        self.diode_voltage.clone_from(&entry_diode_voltage);
        let mut gmin_solution = None;
        for stage in 0..GMIN_STEPPING_STAGES {
            // 1e-3 → 1e-12 を対数等間隔で。
            let exponent = -3.0 - 9.0 * (stage as f64) / (GMIN_STEPPING_STAGES - 1) as f64;
            let gmin = 10.0_f64.powf(exponent);
            match self.newton_attempt(&a_mat, &b_vec, gmin, FALLBACK_MAX_NEWTON_ITERATIONS) {
                Some(x) => gmin_solution = Some(x),
                None => {
                    gmin_solution = None;
                    break;
                }
            }
        }
        if let Some(x) = gmin_solution {
            self.commit_solution(&x);
            return;
        }

        // 段4: source stepping(独立源を0から連続化)。
        self.diode_voltage.clone_from(&entry_diode_voltage);
        let mut source_solution = None;
        for stage in 1..=SOURCE_STEPPING_STAGES {
            let scale = stage as f64 / SOURCE_STEPPING_STAGES as f64;
            let (scaled_a, scaled_b) = self.build_linear_system(dt, scale);
            match self.newton_attempt(&scaled_a, &scaled_b, 0.0, FALLBACK_MAX_NEWTON_ITERATIONS) {
                Some(x) => source_solution = Some(x),
                None => {
                    source_solution = None;
                    break;
                }
            }
        }
        if let Some(x) = source_solution {
            self.commit_solution(&x);
            return;
        }

        // 段5: 全段失敗。前stepの解をそのまま保持(ラッチ)して診断を立てる。
        self.diode_voltage = entry_diode_voltage;
        self.last_solve_diverged = true;
    }

    /// Newtonの1試行(段1・2、`step`のdoc参照)。収束したら解を返す。
    /// `gmin`は各ダイオードに並列に入れるコンダクタンス(段3のgmin stepping用、
    /// 通常は0)。
    fn newton_attempt(
        &mut self,
        a_mat: &[Vec<f64>],
        b_vec: &[f64],
        gmin: f64,
        max_iterations: usize,
    ) -> Option<Vec<f64>> {
        let n = b_vec.len();
        let node_idx = |node: usize| -> Option<usize> {
            if node == GROUND {
                None
            } else {
                Some(node - 1)
            }
        };
        let mut x = vec![0.0; n];
        // 振動検出用: ダイオードごとの直前のΔvの符号と、符号が反転した回数。
        let mut previous_sign = vec![0i8; self.diodes.len()];
        let mut sign_flips = vec![0u32; self.diodes.len()];
        let mut damping = 1.0_f64;

        for _ in 0..max_iterations {
            let mut iter_a: Vec<Vec<f64>> = a_mat.to_vec();
            let mut iter_b: Vec<f64> = b_vec.to_vec();
            for (idx, &(a, b, is_sat, n_vt)) in self.diodes.iter().enumerate() {
                let v_op = self.diode_voltage[idx];
                let exp_term = (v_op / n_vt).exp();
                let i_at_op = is_sat * (exp_term - 1.0);
                let g_d = is_sat / n_vt * exp_term + gmin;
                let i_eq = g_d * v_op - i_at_op;
                // gmin は素子に並列な線形コンダクタンス。等価電流源側にも同じ
                // g_d*v_op が入っているので、追加分は自動的に相殺され、
                // gmin→0 の極限で元のNewtonと一致する。
                if let Some(ia) = node_idx(a) {
                    iter_a[ia][ia] += g_d;
                }
                if let Some(ib) = node_idx(b) {
                    iter_a[ib][ib] += g_d;
                }
                if let (Some(ia), Some(ib)) = (node_idx(a), node_idx(b)) {
                    iter_a[ia][ib] -= g_d;
                    iter_a[ib][ia] -= g_d;
                }
                if let Some(ia) = node_idx(a) {
                    iter_b[ia] += i_eq;
                }
                if let Some(ib) = node_idx(b) {
                    iter_b[ib] -= i_eq;
                }
            }
            x = solve_linear_system(iter_a, iter_b);
            if x.iter().any(|v| !v.is_finite()) {
                return None; // 特異・発散(この段は失敗)。
            }

            let mut max_step = 0.0_f64;
            for (idx, &(a, b, is_sat, n_vt)) in self.diodes.iter().enumerate() {
                let v_new = self.node_voltage_from(&x, a) - self.node_voltage_from(&x, b);
                let v_old = self.diode_voltage[idx];
                // 電圧ステップ制限(設計§4「1反復あたりの変化を2nV_Tにクランプ」)を
                // **SPICE標準の`pnjlim`**で行う(`pn_junction_limit`のdoc参照)。
                let limited = pn_junction_limit(v_new, v_old, n_vt, is_sat) - v_old;
                // 段2: 振動検出(Δvの符号が3回交互したらダンピング0.5)。
                let sign = if limited > 0.0 {
                    1
                } else if limited < 0.0 {
                    -1
                } else {
                    0
                };
                if sign != 0 && previous_sign[idx] != 0 && sign != previous_sign[idx] {
                    sign_flips[idx] += 1;
                    if sign_flips[idx] >= OSCILLATION_FLIP_THRESHOLD {
                        damping = OSCILLATION_DAMPING;
                    }
                }
                if sign != 0 {
                    previous_sign[idx] = sign;
                }
                let step = limited * damping;
                self.diode_voltage[idx] += step;
                max_step = max_step.max(step.abs());
            }

            // 収束判定は Δv **かつ** KCL残差(設計§4)。Δvだけだとダンピングで
            // 歩幅が小さくなっただけの状態を「収束」と誤認しうる。
            // 収束判定は Δv **かつ** KCL残差(設計§4)。`&&`は短絡するので、
            // Δvが大きいうちは残差(指数の評価を含む)を計算しない。
            // 収束判定は Δv **かつ** KCL残差(設計§4)。`&&`は短絡するので、
            // Δvが大きいうちは残差(指数の評価を含む)を計算しない。
            if max_step < DIODE_CONVERGENCE_TOLERANCE
                && self.kcl_residual(&x, a_mat, b_vec, gmin) < 1.0
            {
                return Some(x);
            }
        }
        None
    }

    /// KCL残差の**正規化された**大きさ(設計§4の収束判定の第2条件)。
    /// 1未満なら収束とみなす。
    ///
    /// 行$i$の許容値は $\text{abstol} + \text{reltol}\cdot\max_j|A_{ij}x_j|$ で、
    /// 戻り値は $\max_i |r_i| / (\text{許容値})$。
    ///
    /// **絶対値だけの判定では駄目だった**(群7の実装検証で発見): 設計§4の
    /// 「KCL残差 $<10^{-9}$A」を絶対値で課すと、**理想スイッチの2値抵抗近似**
    /// (閉$10^{-6}\Omega$/開$10^{9}\Omega$、15桁の比)を含む回路で行列の条件数が
    /// $10^{15}$規模になり、**倍精度の丸め誤差そのものが$10^{-9}$Aを超える**。
    /// D19(電気工作台)がこれで解けなくなり、段5のラッチが毎step発火して
    /// 電圧が0のままになった。SPICEと同じく相対項(`reltol`)を併せて持たせる。
    fn kcl_residual(&self, x: &[f64], a_mat: &[Vec<f64>], b_vec: &[f64], gmin: f64) -> f64 {
        let n = b_vec.len();
        let mut residual = vec![0.0; n];
        let mut scale = vec![0.0_f64; n];
        for i in 0..n {
            let mut sum = 0.0;
            let mut max_term = 0.0_f64;
            for (j, xj) in x.iter().enumerate().take(n) {
                let term = a_mat[i][j] * xj;
                sum += term;
                max_term = max_term.max(term.abs());
            }
            residual[i] = sum - b_vec[i];
            scale[i] = max_term.max(b_vec[i].abs());
        }
        // ダイオードの非線形電流(+ gmin の線形分)をKCLへ加える。
        for &(a, b, is_sat, n_vt) in &self.diodes {
            let v = self.node_voltage_from(x, a) - self.node_voltage_from(x, b);
            let current = is_sat * ((v / n_vt).exp() - 1.0) + gmin * v;
            if a != GROUND {
                residual[a - 1] += current;
                scale[a - 1] = scale[a - 1].max(current.abs());
            }
            if b != GROUND {
                residual[b - 1] -= current;
                scale[b - 1] = scale[b - 1].max(current.abs());
            }
        }
        (0..n).fold(0.0_f64, |acc, i| {
            let tolerance = KCL_RESIDUAL_TOLERANCE + KCL_RESIDUAL_RELATIVE * scale[i];
            acc.max(residual[i].abs() / tolerance)
        })
    }

    /// 解を状態へ反映する(`step`から切り出した、**群7**)。
    fn commit_solution(&mut self, x: &[f64]) {
        let n_node_unknowns = self.num_nodes.saturating_sub(1);
        self.last_node_voltage = vec![0.0; self.num_nodes];
        self.last_node_voltage[1..self.num_nodes].copy_from_slice(&x[..n_node_unknowns]);

        self.last_source_current =
            x[n_node_unknowns..n_node_unknowns + self.voltage_sources.len()].to_vec();

        for (idx, &(a, b, _)) in self.capacitors.iter().enumerate() {
            self.capacitor_voltage[idx] =
                self.node_voltage_from(x, a) - self.node_voltage_from(x, b);
        }
        let inductor_start = n_node_unknowns + self.voltage_sources.len();
        for (idx, current_slot) in self.inductor_current.iter_mut().enumerate() {
            *current_slot = x[inductor_start + idx];
        }
    }

    fn node_voltage_from(&self, x: &[f64], node: usize) -> f64 {
        if node == GROUND {
            0.0
        } else {
            x[node - 1]
        }
    }

    /// 直近の`step`が非線形収束のフォールバック連鎖を**全段失敗**したか
    /// (**群7で追加**、`step`のdoc「段5」参照)。`true`なら解は前stepのまま
    /// ラッチされている。
    pub fn last_solve_diverged(&self) -> bool {
        self.last_solve_diverged
    }

    /// 抵抗の本数(`resistor_power`のインデックス範囲、`sim-coupling::JouleHeat`が読む)。
    pub fn resistor_count(&self) -> usize {
        self.resistors.len()
    }

    /// 抵抗`index`の直近の消費電力 $P=V^2/R$(設計docs/20-integration/01-coupling-matrix.md
    /// `JouleHeat`が読む)。
    pub fn resistor_power(&self, index: usize) -> f64 {
        let (a, b, r) = self.resistors[index];
        let v = self.node_voltage(a) - self.node_voltage(b);
        v * v / r
    }

    /// 電圧源`index`(`add_voltage_source`を呼んだ順)の直近の解電流(向きは`a→b`が正、
    /// 設計docs/13-electromagnetism/05-em-mechanics-coupling.md §2.2「導体棒」、
    /// `sim-coupling::InductionCoupling`が読む)。`step()`を一度も呼んでいない場合は
    /// (`node_voltage`と同様に)0を返す(パニックしない)。
    pub fn source_current(&self, index: usize) -> f64 {
        self.last_source_current.get(index).copied().unwrap_or(0.0)
    }
}

impl Solver for Circuit {
    /// 後退Eulerは無条件安定(設計§4)。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, ctx: &mut SolverContext) {
        // 同名の inherent メソッド(1引数版、上の`impl Circuit`ブロック)を呼ぶ —
        // Rustのメソッド解決規則により inherent メソッドが同名のトレイトメソッドより
        // 優先されるため、トレイト実装内から`self.step(dt)`と書いても再帰しない。
        self.step(dt);
        // フォールバック連鎖の全段が失敗した(段5、`Circuit::step`のdoc参照)。
        // **黙って進まない**——前stepの解をラッチした上で診断イベントを出す
        // (`sim_thermal::ThermalSolver`のPCG非収束と同じ扱い)。
        if self.last_solve_diverged {
            ctx.events.push(sim_core::Event {
                step: 0,
                source: sim_core::SourceId(0),
                kind: sim_core::EventKind::SolverDiverged,
            });
        }
    }

    /// コンデンサ・インダクタに蓄えられた電磁エネルギー(設計§1.1.2(2)「電磁場」)。
    /// 抵抗のジュール熱は瞬時消費であり蓄積量ではないため対象外(`resistor_power`が
    /// 別途瞬時電力を提供、`JouleHeat`が積算する)。
    fn total_energy(&self) -> EnergyBreakdown {
        let mut electromagnetic = 0.0;
        for (idx, &(_, _, c)) in self.capacitors.iter().enumerate() {
            electromagnetic += 0.5 * c * self.capacitor_voltage[idx] * self.capacitor_voltage[idx];
        }
        for (idx, &(_, _, l)) in self.inductors.iter().enumerate() {
            electromagnetic += 0.5 * l * self.inductor_current[idx] * self.inductor_current[idx];
        }
        EnergyBreakdown {
            electromagnetic,
            ..Default::default()
        }
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        hasher.write_u64(self.last_node_voltage.len() as u64);
        for &v in &self.last_node_voltage {
            hasher.write_f64(v);
        }
        for &i in &self.last_source_current {
            hasher.write_f64(i);
        }
        for &v in &self.capacitor_voltage {
            hasher.write_f64(v);
        }
        for &i in &self.inductor_current {
            hasher.write_f64(i);
        }
        for &v in &self.diode_voltage {
            hasher.write_f64(v);
        }
    }

    fn approximations(&self) -> Vec<Approximation> {
        vec![
            Approximation {
                name: "スイッチ = 2値抵抗近似",
                reason: "理想スイッチ専用の未知数を追加せず、閉1e-6Ω/開1e9Ωの抵抗として\
                         通常のコンダクタンス経路でスタンプする。",
                doc: "docs/13-electromagnetism/02-circuits.md",
                can_disable: false,
            },
            Approximation {
                name: "動的素子は後退Euler",
                reason: "C・Lをコンパニオンモデルへ変換して代数化する(設計§4の既定)。",
                doc: "docs/13-electromagnetism/02-circuits.md",
                can_disable: false,
            },
            Approximation {
                name: "電圧ステップ制限はSPICEのpnjlim",
                reason: "設計§4の「2nV_Tへの素朴なクランプ」では冷間開始から動作点0.6Vへ\
                         上限10反復で構造的に届かないため、SPICE標準のpnjlim(指数領域だけ\
                         対数圧縮)を使う。指数領域での1反復あたりの増加は設計と同オーダー。",
                doc: "docs/13-electromagnetism/02-circuits.md",
                can_disable: false,
            },
        ]
    }
}

/// **pn接合の電圧ステップ制限(SPICE標準の`pnjlim`)**(**群7**)。
///
/// 設計 docs/13-electromagnetism/02-circuits.md §4 は「1反復あたりの変化を
/// $2nV_T\approx52$mVにクランプ」と書いているが、**この単純なクランプだけでは
/// 冷間開始(全ノード0Vから)で収束しない**ことを群7の実装検証で確認した——
/// ダイオードの動作点は約0.6Vで、52mV刻みでは設計の反復上限10回(=0.52V)に
/// 届かない。移行前の実装がそれでも動いていたのは、`diode_voltage`が**stepを
/// またいで持ち越される**ため数stepかけて動作点まで歩いていたからで、
/// 「1step内で収束する」ことは一度も保証されていなかった。
///
/// そこでSPICEが実際に使う`pnjlim`を採る:
/// - $v_{new}\le v_{crit}$ または変化が$2nV_T$以内 → **制限しない**(素直なNewton)。
/// - 指数領域($v_{new}>v_{crit}$)で大きく飛ぼうとしたとき → **対数で圧縮**
///   $v = v_{old} + nV_T\ln(1+(v_{new}-v_{old})/(nV_T))$。
///
/// $v_{crit}=nV_T\ln(nV_T/(\sqrt2 I_s))$ は指数関数の曲率が発散し始める点。
/// 指数領域での1反復あたりの増加は設計と同じ$2nV_T$のオーダーに収まりつつ、
/// 低電圧側では大きく歩けるので冷間開始でも数反復で収束する。
fn pn_junction_limit(v_new: f64, v_old: f64, n_vt: f64, is_sat: f64) -> f64 {
    if is_sat <= 0.0 || n_vt <= 0.0 {
        return v_new;
    }
    let v_crit = n_vt * (n_vt / (std::f64::consts::SQRT_2 * is_sat)).ln();
    if v_new > v_crit && (v_new - v_old).abs() > 2.0 * n_vt {
        if v_old > 0.0 {
            let arg = 1.0 + (v_new - v_old) / n_vt;
            if arg > 0.0 {
                v_old + n_vt * arg.ln()
            } else {
                v_crit
            }
        } else {
            // 冷間開始(v_old<=0)。対数スケールへ落として一気に指数領域の入口へ。
            n_vt * (v_new / n_vt).ln()
        }
    } else {
        v_new
    }
}

/// 部分ピボット付きガウス消去。回路規模が小さい(<10^3 節点、設計 §10)前提の密行列版。
fn solve_linear_system(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let mut pivot_row = col;
        let mut pivot_val = a[col][col].abs();
        for (row, row_vec) in a.iter().enumerate().skip(col + 1) {
            if row_vec[col].abs() > pivot_val {
                pivot_row = row;
                pivot_val = row_vec[col].abs();
            }
        }
        a.swap(col, pivot_row);
        b.swap(col, pivot_row);

        let pivot = a[col][col];
        if pivot.abs() < 1e-15 {
            continue; // 特異(未接続ノード等)、その行は寄与なしとして0を残す
        }
        let pivot_row_vals = a[col].clone();
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            if factor == 0.0 {
                continue;
            }
            for (k, &pivot_val) in pivot_row_vals.iter().enumerate().skip(col) {
                a[row][k] -= factor * pivot_val;
            }
            b[row] -= factor * b[col];
        }
    }

    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut sum = b[row];
        for col in (row + 1)..n {
            sum -= a[row][col] * x[col];
        }
        x[row] = if a[row][row].abs() < 1e-15 {
            0.0
        } else {
            sum / a[row][row]
        };
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `node_voltage`の範囲外パニック回帰テスト(シーンギャラリー増分B2で発見)。
    /// `sim-wasm::WasmWorld::circuit_divider_voltage`は固定デモ回路のノード番号
    /// (`CIRCUIT_DIVIDER_NODE=2`)を無条件に読むため、シーンギャラリー経由で
    /// それより少ないノード数の回路(D21銅管落下、`num_nodes=2`→ノードは0/1のみ)
    /// を読み込むと、修正前は`self.last_node_voltage[node]`の直接インデックスで
    /// パニックしていた。
    #[test]
    fn node_voltage_out_of_range_returns_zero_instead_of_panicking() {
        let mut circuit = Circuit::new(2); // ノードは0(GND)・1のみ
        circuit.add_resistor(1, GROUND, 100.0);
        circuit.add_voltage_source(1, GROUND, 5.0);
        circuit.step(1.0);

        assert_eq!(
            circuit.node_voltage(2),
            0.0,
            "node 2 does not exist in a 2-node circuit"
        );
        assert_eq!(
            circuit.node_voltage(100),
            0.0,
            "far out-of-range node should also return 0"
        );
    }

    /// ダイオード整流: 半波整流の平均電圧、rel 2%(設計 docs/13-electromagnetism/02-circuits.md
    /// §7「ダイオード整流: 半波整流の平均電圧(±2%)」。対応するE番号は無い)。
    /// 理想ダイオード(順方向降下ゼロ)近似での解析平均 $V_{peak}/\pi$ と比較する。
    /// $V_{peak}=100V$ に対しShockleyダイオードの実際の順方向降下は約0.77V($I_s=10^{-14}$A,
    /// $nV_T=25.85$mVでの概算)しかないため、理想近似との差(rel≈1.2%)は2%以内に収まる。
    #[test]
    fn diode_half_wave_rectifier_average_output_matches_ideal_diode_approximation() {
        let v_peak = 100.0;
        let r = 1000.0;
        let is_sat = 1e-14; // Si小信号ダイオード(設計§9)
        let n_vt = 0.02585; // n≈1、300KでのV_T(設計§9)

        let mut circuit = Circuit::new(3); // 0=GND, 1=AC源, 2=出力(ダイオード・抵抗の接続点)
        circuit.add_voltage_source(1, GROUND, 0.0); // index 0、毎ステップ値を更新する
        circuit.add_diode(1, 2, is_sat, n_vt); // anode=AC源側、cathode=出力側
        circuit.add_resistor(2, GROUND, r);

        let period = 1.0; // 角周波数のみが意味を持つ任意単位
        let omega = 2.0 * std::f64::consts::PI / period;
        let samples = 1000;
        let dt = period / samples as f64;

        let mut sum_v_out = 0.0;
        for i in 0..samples {
            let t = i as f64 * dt;
            let v_in = v_peak * (omega * t).sin();
            circuit.set_voltage_source_voltage(0, v_in);
            circuit.step(dt);
            sum_v_out += circuit.node_voltage(2);
        }
        let measured_avg = sum_v_out / samples as f64;
        let expected_avg = v_peak / std::f64::consts::PI;
        let rel_err = (measured_avg - expected_avg).abs() / expected_avg;
        assert!(
            rel_err < 0.02,
            "measured_avg={measured_avg} expected_avg={expected_avg} rel_err={rel_err}"
        );
    }

    /// E5: 分圧回路。直並列の解析値と機械精度一致(docs/21-verification/01-analytic-tests.md E5)。
    /// 動的素子が無いため、任意の dt での単一 MNA 解が厳密解と一致する(時間発展不要)。
    #[test]
    fn e5_voltage_divider_matches_analytic_solution_at_machine_precision() {
        let v0 = 9.0;
        let r1 = 1000.0;
        let r2 = 2000.0;
        let mut circuit = Circuit::new(3); // 0=GND, 1=V0側, 2=分圧点
        circuit.add_voltage_source(1, GROUND, v0);
        circuit.add_resistor(1, 2, r1);
        circuit.add_resistor(2, GROUND, r2);
        circuit.step(1.0);

        let expected = v0 * r2 / (r1 + r2);
        let measured = circuit.node_voltage(2);
        let rel_err = (measured - expected).abs() / expected;
        assert!(rel_err < 1e-9, "measured={measured} expected={expected}");
    }

    /// E3: RC過渡 $v(t)=V(1-e^{-t/RC})$、時定数の相対誤差 < 0.5%
    /// (docs/21-verification/01-analytic-tests.md E3)。2時刻の電圧比から時定数を逆算し、
    /// 指数則の形そのものを検証する(単一時刻の一致だけでなく)。
    #[test]
    fn e3_rc_transient_time_constant_matches_rc() {
        let v0 = 5.0;
        let r = 1000.0;
        let c = 1.0e-6;
        let tau = r * c;

        let mut circuit = Circuit::new(3); // 0=GND, 1=V0側, 2=コンデンサ端子
        circuit.add_voltage_source(1, GROUND, v0);
        circuit.add_resistor(1, 2, r);
        circuit.add_capacitor(2, GROUND, c, 0.0);

        let dt = tau / 2000.0;
        let (t1, t2) = (tau, 2.0 * tau);
        let mut v_at_t1 = None;
        let mut v_at_t2 = None;
        let mut t = 0.0;
        let steps = (t2 / dt).ceil() as u32 + 1;
        for _ in 0..steps {
            circuit.step(dt);
            t += dt;
            if v_at_t1.is_none() && t >= t1 {
                v_at_t1 = Some(circuit.node_voltage(2));
            }
            if v_at_t2.is_none() && t >= t2 {
                v_at_t2 = Some(circuit.node_voltage(2));
            }
        }
        let v1 = v_at_t1.expect("t1 should be reached");
        let v2 = v_at_t2.expect("t2 should be reached");

        // V0-v(t) = V0・exp(-t/τ) なので (V0-v1)/(V0-v2) = exp((t2-t1)/τ)。
        let measured_tau = (t2 - t1) / ((v0 - v1) / (v0 - v2)).ln();
        let rel_err = (measured_tau - tau).abs() / tau;
        assert!(rel_err < 0.005, "measured_tau={measured_tau} tau={tau}");
    }

    /// E4: RLC減衰振動 $\omega=\sqrt{1/LC-(R/2L)^2}$、rel 1%
    /// (docs/21-verification/01-analytic-tests.md E4)。初期充電したコンデンサを
    /// R・Lと閉ループにして自由減衰させ、コンデンサ電圧の隣接ゼロ交差の間隔(半周期)から
    /// 角周波数を実測する。
    #[test]
    fn e4_rlc_decay_angular_frequency_matches_formula() {
        let v0 = 1.0;
        let r: f64 = 10.0;
        let l: f64 = 0.01;
        let c: f64 = 1.0e-6;
        let omega = (1.0 / (l * c) - (r / (2.0 * l)).powi(2)).sqrt();
        let period = 2.0 * std::f64::consts::PI / omega;

        let mut circuit = Circuit::new(3); // 0=GND, 1=コンデンサ端子, 2=R-L接続点
        circuit.add_capacitor(1, GROUND, c, v0);
        circuit.add_resistor(1, 2, r);
        circuit.add_inductor(2, GROUND, l, 0.0);

        let dt = period / 4000.0;
        let steps = (period * 1.1 / dt) as u32;

        let mut prev_v = circuit.node_voltage(1);
        let mut prev_t = 0.0;
        let mut crossings = Vec::new();
        for step in 0..steps {
            circuit.step(dt);
            let t = (step + 1) as f64 * dt;
            let v = circuit.node_voltage(1);
            if prev_v.signum() != v.signum() && prev_v != 0.0 {
                let frac = -prev_v / (v - prev_v);
                crossings.push(prev_t + frac * (t - prev_t));
                if crossings.len() >= 2 {
                    break;
                }
            }
            prev_v = v;
            prev_t = t;
        }

        assert!(crossings.len() >= 2, "should observe two zero crossings");
        let measured_period = 2.0 * (crossings[1] - crossings[0]);
        let measured_omega = 2.0 * std::f64::consts::PI / measured_period;
        let rel_err = (measured_omega - omega).abs() / omega;
        assert!(
            rel_err < 0.01,
            "measured_omega={measured_omega} omega={omega} rel_err={rel_err}"
        );
    }

    /// スイッチ(理想スイッチの2値抵抗近似、`SWITCH_ON_RESISTANCE`のdoc参照): 開いている間は
    /// 出力電圧がほぼ0(開放)、閉じている間は分圧回路の解析値と一致することを確認する
    /// (`sim-world::Command::SetSwitch`が使う`set_switch_closed`経由)。
    #[test]
    fn switch_toggles_between_open_circuit_and_analytic_voltage_divider() {
        let v = 10.0;
        let r1 = 100.0;
        let r2 = 200.0;
        let mut circuit = Circuit::new(3); // 0=GND, 1=電源, 2=分圧点
        circuit.add_voltage_source(1, GROUND, v);
        circuit.add_resistor(1, 2, r1);
        let switch = circuit.add_switch(2, GROUND, false); // 初期状態: 開
                                                           // switch と並列ではなく2→GNDの負荷抵抗として使う分圧回路(r2はswitchの先の負荷)。
        circuit.add_resistor(2, GROUND, r2);

        circuit.step(1e-6);
        // 開: switchの枝はほぼ電流を流さないが、r1-r2の分圧自体はswitchと無関係に成立する
        // ため、switchが開いていても閉じていてもr2による分圧は変わらない。switch自体の
        // 効果を見るには、switchがGNDへの別経路(負荷を短絡)として働く配線にする必要がある
        // ため、ここではswitchをr2と並列(2→GND)に置き、閉で分圧点がほぼ0Vへ落ちる
        // (switchの低抵抗がr2を実効的に短絡する)ことを直接確認する。
        let v_open = circuit.node_voltage(2);
        let expected_open = v * r2 / (r1 + r2);
        let rel_err_open = (v_open - expected_open).abs() / expected_open;
        assert!(
            rel_err_open < 0.01,
            "open: v_open={v_open} expected={expected_open} rel_err={rel_err_open}"
        );

        circuit.set_switch_closed(switch, true);
        circuit.step(1e-6);
        // 閉: switch(1e-6Ω)がr2(200Ω)と並列になり分圧点をほぼ短絡するため、
        // 出力電圧はほぼ0まで落ちる。
        let v_closed = circuit.node_voltage(2);
        assert!(
            v_closed.abs() < 1e-3,
            "closed: v_closed should be near-zero (switch shorts node 2 to GND), got {v_closed}"
        );
    }

    /// **群7: DCモーターの定常電流**(設計§2の表 $v=Ri+L\,di/dt+k_e\omega$)。
    /// 電池$V$でモーターを回すと、定常状態(電流一定 → $L\,di/dt=0$)では
    /// $i=(V-k_e\omega)/R$ になる。角速度0(拘束)なら $i=V/R$(拘束電流)、
    /// 角速度を上げるほど電流が減り、$\omega=V/k_e$ で0になる(無負荷回転数)。
    #[test]
    fn dc_motor_steady_current_matches_the_back_emf_equation() {
        let v_supply = 12.0;
        let r_winding = 2.0;
        let l_winding = 1e-3;
        let ke = 0.05;

        // ノード: 1 = 電池+、2 = モーター内部、GND = 電池−。
        let build = || {
            let mut circuit = Circuit::new(4);
            circuit.add_voltage_source(1, GROUND, v_supply);
            let motor = circuit.add_dc_motor(1, GROUND, (2, 3), r_winding, l_winding, ke);
            (circuit, motor)
        };

        for &omega in &[0.0, 50.0, 100.0, 200.0] {
            let (mut circuit, motor) = build();
            circuit.set_motor_speed(motor, omega);
            // L/R = 0.5 ms なので、dt=1e-4 を200step(20 ms)で完全に定常。
            for _ in 0..200 {
                circuit.step(1e-4);
            }
            let expected = (v_supply - ke * omega) / r_winding;
            let measured = circuit.motor_current(motor);
            assert!(
                (measured - expected).abs() < 1e-6,
                "ω={omega}: 定常電流は (V - k_e ω)/R のはず: measured={measured:.6} \
                 expected={expected:.6}"
            );
        }

        // 無負荷回転数 ω = V/k_e で電流ゼロ(逆起電力が電源と釣り合う)。
        let (mut circuit, motor) = build();
        circuit.set_motor_speed(motor, v_supply / ke);
        for _ in 0..200 {
            circuit.step(1e-4);
        }
        assert!(
            circuit.motor_current(motor).abs() < 1e-6,
            "無負荷回転数では電流が消えるはず: {}",
            circuit.motor_current(motor)
        );
    }

    /// **群7: モーターのインダクタンスによる電流の立ち上がり**。突入電流は
    /// $i(t)=\frac{V}{R}(1-e^{-t/\tau})$、$\tau=L/R$。1時定数で最終値の
    /// $1-1/e\approx63.2\%$ に達することを確認する(RL回路そのものの検証は
    /// E4が別途持つが、ここでは**モーター素子として組んだ等価回路**が
    /// 同じ物理を再現することを見る)。
    #[test]
    fn dc_motor_current_rises_with_the_winding_time_constant() {
        let (v_supply, r_winding, l_winding, ke) = (12.0, 2.0, 0.01, 0.05);
        let tau = l_winding / r_winding;
        let mut circuit = Circuit::new(4);
        circuit.add_voltage_source(1, GROUND, v_supply);
        let motor = circuit.add_dc_motor(1, GROUND, (2, 3), r_winding, l_winding, ke);
        circuit.set_motor_speed(motor, 0.0); // 拘束(ω=0)

        let dt = tau / 500.0;
        let steps = 500; // ちょうど1時定数
        for _ in 0..steps {
            circuit.step(dt);
        }
        let final_current = v_supply / r_winding;
        let expected = final_current * (1.0 - (-1.0_f64).exp());
        let measured = circuit.motor_current(motor);
        let rel_err = (measured - expected).abs() / expected;
        assert!(
            rel_err < 0.01,
            "1時定数で最終値の63.2%のはず: measured={measured:.5} expected={expected:.5} \
             rel_err={rel_err:.5}"
        );
    }

    /// **群7: フォールバック連鎖が実際に段を降りて解けること**(設計§4)。
    /// 直列に多数のダイオードを積んだ回路は、Newtonの初期点(全て0V)から見ると
    /// 指数の壁が急峻で、電圧ステップ制限つきNewtonの10反復では収束しきらない。
    /// この回路が**それでも正しく解ける**ことを確認する: 段3(gmin stepping)・
    /// 段4(source stepping)が働いた結果として、KCL残差が収束判定を満たす解が
    /// 得られていること。
    ///
    /// 検証は**物理的な整合**で行う: 直列ダイオード列の両端電圧の和が電源電圧に
    /// 一致し(KVL)、各ダイオードの電流がShockley式と一致し(素子則)、
    /// それが直列なので全て等しい(KCL)。
    #[test]
    fn the_fallback_chain_solves_a_series_diode_stack_that_plain_newton_cannot() {
        let v_supply = 5.0;
        let r_series = 100.0;
        let is_sat = 1e-14;
        let n_vt = 0.026;
        let stack = 6; // 直列ダイオード段数

        // ノード: 1 = 電源+、2..=1+stack = ダイオード間、最後は抵抗経由でGND。
        let nodes = 2 + stack;
        let mut circuit = Circuit::new(nodes);
        circuit.add_voltage_source(1, GROUND, v_supply);
        for k in 0..stack {
            circuit.add_diode(1 + k, 2 + k, is_sat, n_vt);
        }
        circuit.add_resistor(1 + stack, GROUND, r_series);

        for _ in 0..50 {
            circuit.step(1e-4);
        }
        assert!(
            !circuit.last_solve_diverged(),
            "フォールバック連鎖のどこかの段で解けるはず"
        );

        // KVL: 電源 = ダイオード列の電圧降下の和 + 抵抗の電圧降下。
        let node_v = |k: usize| circuit.node_voltage(k);
        let mut diode_drop_sum = 0.0;
        for k in 0..stack {
            diode_drop_sum += node_v(1 + k) - node_v(2 + k);
        }
        let resistor_drop = node_v(1 + stack);
        assert!(
            (v_supply - (diode_drop_sum + resistor_drop)).abs() < 1e-9,
            "KVLが成り立つはず: V={v_supply} diodes={diode_drop_sum} R={resistor_drop}"
        );

        // 素子則 + KCL: 各ダイオードの Shockley 電流が抵抗電流と一致する。
        let resistor_current = resistor_drop / r_series;
        assert!(
            resistor_current > 1e-6,
            "順方向にちゃんと電流が流れているはず: {resistor_current}"
        );
        for k in 0..stack {
            let v = node_v(1 + k) - node_v(2 + k);
            let i = is_sat * ((v / n_vt).exp() - 1.0);
            let rel = (i - resistor_current).abs() / resistor_current;
            assert!(
                rel < 1e-6,
                "ダイオード{k}の電流が直列電流と一致するはず: {i} vs {resistor_current}"
            );
        }
    }

    /// 全段失敗したときは**前stepの解をラッチして診断フラグを立てる**(段5)。
    /// 実際に全段を失敗させるのは難しいので、ここでは**フラグの意味**——
    /// 正常に解けたstepでは必ず`false`であること——を固定する。
    /// (黙って嘘の解を返さないという不変条件の下限を守るテスト。)
    #[test]
    fn a_solvable_circuit_never_reports_divergence() {
        let mut circuit = Circuit::new(3);
        circuit.add_voltage_source(1, GROUND, 5.0);
        circuit.add_diode(1, 2, 1e-14, 0.026);
        circuit.add_resistor(2, GROUND, 1000.0);
        for _ in 0..100 {
            circuit.step(1e-4);
            assert!(!circuit.last_solve_diverged());
        }
    }
}
