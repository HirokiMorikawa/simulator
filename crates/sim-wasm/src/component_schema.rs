//! `WasmWorld::component_schema`が返す`apply`側スキーマの表(**Task#9**)。
//!
//! **これが無かった間の縮約(実害)**: `component_schema`はTask#8第一弾で
//! 「受け付けるkind名の平坦な文字列配列」だけを返していた。当時のdocは
//! 「パラメータのスキーマ自体は元のUIフォーム側にすでにtitleツールチップとして
//! 存在するため、二重管理を避けてここでは持たない」と書いていたが、**その結果
//! 起きていたのは二重管理の回避ではなく、意味の消失**である——フロントエンド
//! (`demo/src/main.ts`)は19種類ほどのフォームを手書きし、入力欄のラベルは
//! 「Body」「Anchor」「Axis」「Param」という汎用の文字列、**各フィールドが
//! 選択中のkindにとって何を意味するのかはtitleツールチップにしか存在しない**
//! という状態だった。`add_convection_link_coupling`の`mode`が0〜3の対流相関式で
//! あることも、`body_b`が負ならワールド固定点であることも、`water_density`の
//! 単位がkg/m^3であることも、ホバーしなければ分からない。
//!
//! そこでこのモジュールは、`apply_component_impl`の`match kind`が実際に
//! 引数へ渡している**フィールド名・型・単位・既定値・値域**を機械可読な形で
//! 宣言する。フォームを手書きする代わりにこのスキーマから生成する、という
//! 後続増分の土台である(このモジュール自体はフロントエンドに一切触れない)。
//!
//! **単一の情報源はあくまで`apply_component_impl`側**——ここはその写像であり、
//! 食い違えば生成されたフォームが無効なpayloadを送る。同期は
//! `component_schema_covers_every_apply_kind`(lib.rsのテスト)が
//! 「スキーマに載る全kindがディスパッチに存在すること」と件数の一致で守る。
//!
//! **単位について**: `unit`は`_impl`メソッド本体・その呼び出し先(`sim-mechanics`
//! /`sim-coupling`/`sim-em`等)のdocコメントが実際に宣言している物理単位を
//! そのまま写す。方向ベクトル・クォータニオン・比・ビットマスク・index・
//! 乱数シードのような無次元量は`null`。
//!
//! **`min`/`max`について**: `_impl`メソッド(またはその呼び出し先)が
//! **実際に検証して弾く**値域だけを載せる。「常識的にはこの範囲」という
//! 発明した境界は載せない——フォームが物理的に有効な入力を勝手に拒むほうが、
//! 境界を書かないより有害だからである。結果として`min`/`max`が付くのは
//! ごく少数(`set_dt`・`derive_material`・`push_set_body_mass`・
//! `set_body_scale_xyz_at`・`add_convection_link_coupling`の`mode`)。

use serde::Serialize;

/// フィールドのRust側の型(`apply_component_impl`の抽出クロージャ
/// `f`/`u`/`i`/`s`/`b`と1:1に対応する)。JSONでは小文字の型名になる。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum FieldType {
    /// `f("...")`——`as_f64`。
    F64,
    /// `u("...")`——`as_u64`。`seed`/`stream`(u64)・`group`/`mask`/`mode`
    /// (u32)もこれで表す。JSON数値の精度を超える識別子は扱わないため、
    /// 符号なし整数を1種類に畳んでいる。
    Usize,
    /// `i("...")`——`as_i64`。負値がセンチネルになる`body_b`専用。
    I32,
    /// `s("...")`——`as_str`。
    String,
    /// `b("...")`——`as_bool`。
    Bool,
}

/// フィールドの既定値——**`payload`からそのキーを省いたときに実際に渡る値**。
///
/// 抽出クロージャのフォールバック(`f`→0.0、`u`→0、`i`→0、`s`→""、
/// `b`→false)をそのまま写すのが原則で、ディスパッチが個別に別の
/// フォールバックを書いている箇所(`add_convection_link_coupling`の
/// `mode`は`unwrap_or(3)`)だけそちらを載せる。
///
/// **「意味のある既定値」ではなく「省略時に起きること」を載せる**のが要点。
/// たとえば`body_b`の既定は`0`(=ボディ0、既定シーンでは床)であって、
/// 「ワールド固定点」を意味する`-1`ではない——`body_b`を省いたフォームは
/// ワールドではなく床に繋がる、という**実際に踏む罠**をスキーマが
/// 隠さないようにするため。センチネルの存在自体は`nullable`が伝える。
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(untagged)]
pub enum FieldDefault {
    Number(f64),
    Integer(i64),
    Text(&'static str),
    Bool(bool),
}

/// `apply`側1フィールドぶんのスキーマ。
///
/// **`serde_json::json!`の入れ子ではなくstructにした理由**: 73 kind・
/// 延べ300超のフィールドを`json!`マクロで手書きすると、キー名のtypoも
/// 型の取り違えもコンパイラが一切検出しない。structなら
/// `FieldType`/`FieldDefault`が列挙で縛られ、キー名は`#[derive(Serialize)]`が
/// 生成する。
#[derive(Clone, Copy, Debug, Serialize)]
pub struct ComponentFieldSchema {
    /// JSON payloadのキー名(`apply_component_impl`が`f("ax")`等に渡す文字列と
    /// 一致する)。
    pub name: &'static str,
    /// `type`はRustの予約語なのでフィールド名を変えて`rename`する。
    #[serde(rename = "type")]
    pub field_type: FieldType,
    /// 物理単位(無次元・非物理量なら`null`、モジュールdoc参照)。
    pub unit: Option<&'static str>,
    /// 省略時に渡る値(モジュールdoc・`FieldDefault`のdoc参照)。
    pub default: Option<FieldDefault>,
    /// 負値・特定値・省略に**特別な意味**があるか。`true`なら生の数値として
    /// 扱ってはならない(例: `body_b`が負ならワールド固定点、
    /// `add_convection_link_coupling`の`thermal_expansion_coefficient`が
    /// `<=0`なら理想気体近似$\beta=1/T_{film}$)。
    pub nullable: bool,
    /// 実際に検証される下限(モジュールdoc参照)。含まない下限
    /// (`> min`)である点に注意——載せている5箇所はいずれも
    /// 「正の有限値のみ受け付ける」形の検証で、0そのものは弾かれる。
    pub min: Option<f64>,
    /// 実際に有効な上限(モジュールdoc参照)。
    pub max: Option<f64>,
}

/// `apply`側1 kindぶんのスキーマ(kind名 + フィールド一覧)。
///
/// フィールド数はkindによって0個(`clear_atmosphere`・`spawn_fluid_block`等)
/// から11個(`add_wheel_joint`)まで開きがあるが、**どれも「名前付き
/// パラメータの平坦な並び」という同じ形に収まる**。
///
/// **唯一の例外**(重力場の抽象化増分): `push_set_gravity_field`だけは
/// `kind`フィールドの値によって意味を持つフィールドが変わる。それでも表現は
/// 平坦なまま(全フィールドを並べ、`kind`が見ないものは無視される)に留めて
/// あり、事情は`apply_schema`内の同kindのコメントに書いた。
#[derive(Clone, Debug, Serialize)]
pub struct ComponentKindSchema {
    pub kind: &'static str,
    pub fields: Vec<ComponentFieldSchema>,
}

/// 単位付きのf64フィールド。
const fn f(name: &'static str, unit: &'static str) -> ComponentFieldSchema {
    ComponentFieldSchema {
        name,
        field_type: FieldType::F64,
        unit: Some(unit),
        default: Some(FieldDefault::Number(0.0)),
        nullable: false,
        min: None,
        max: None,
    }
}

/// 無次元のf64フィールド(方向ベクトル成分・クォータニオン成分・比・
/// 減衰比・プラントル数・スケール倍率など)。
const fn f_nd(name: &'static str) -> ComponentFieldSchema {
    ComponentFieldSchema {
        name,
        field_type: FieldType::F64,
        unit: None,
        default: Some(FieldDefault::Number(0.0)),
        nullable: false,
        min: None,
        max: None,
    }
}

/// usizeフィールド。index・件数・ビットマスク・乱数シードのいずれかで、
/// 物理単位を持つものは1つも無い。
const fn u(name: &'static str) -> ComponentFieldSchema {
    ComponentFieldSchema {
        name,
        field_type: FieldType::Usize,
        unit: None,
        default: Some(FieldDefault::Integer(0)),
        nullable: false,
        min: None,
        max: None,
    }
}

/// i32フィールド——**負値がセンチネルになる`body_b`のみ**が使う
/// (よって常に`nullable`)。`ComponentFieldSchema::nullable`のdoc参照。
const fn body_b() -> ComponentFieldSchema {
    ComponentFieldSchema {
        name: "body_b",
        field_type: FieldType::I32,
        unit: None,
        default: Some(FieldDefault::Integer(0)),
        nullable: true,
        min: None,
        max: None,
    }
}

/// 文字列フィールド。
///
/// **既知の限界**: このスキーマは「文字列」までしか言えず、**列挙値の
/// 集合を表現できない**。実際には
/// `push_set_body_type`の`kind`は`Dynamic`/`Static`/`Kinematic`の3値、
/// `material_name`/`base_name`は`MaterialDb`が持つ名前のいずれか、に
/// 限られる(それ以外は`WasmError::UnknownBodyType`/`UnknownMaterial`)。
/// 前者は3値固定なのでフォーム生成側が持つほかなく、後者は実行時に
/// `derive_material`で増えるためそもそも静的な表には載らない
/// (`read_component`の`material_properties_f64`で引ける)。
/// ここに`enum`欄を足すと前者しか埋められず、後者について
/// 「候補が無い＝任意の文字列でよい」という誤った含意を与えるので、
/// 型の宣言に留めてこの限界をdocに書く形を選んだ。
const fn s(name: &'static str) -> ComponentFieldSchema {
    ComponentFieldSchema {
        name,
        field_type: FieldType::String,
        unit: None,
        default: Some(FieldDefault::Text("")),
        nullable: false,
        min: None,
        max: None,
    }
}

/// boolフィールド。
const fn b(name: &'static str) -> ComponentFieldSchema {
    ComponentFieldSchema {
        name,
        field_type: FieldType::Bool,
        unit: None,
        default: Some(FieldDefault::Bool(false)),
        nullable: false,
        min: None,
        max: None,
    }
}

impl ComponentFieldSchema {
    /// 「正の有限値のみ受け付ける」検証(`set_dt`・`derive_material`・
    /// `push_set_body_mass`・`set_body_scale_xyz_at`)を写す。
    const fn positive(mut self) -> ComponentFieldSchema {
        self.min = Some(0.0);
        self
    }

    /// センチネル値・省略に特別な意味があることを立てる。
    const fn nullable(mut self) -> ComponentFieldSchema {
        self.nullable = true;
        self
    }

    /// 省略時のフォールバックがクロージャの既定と違うkind
    /// (`add_convection_link_coupling`の`mode`)で使う。
    const fn default_integer(mut self, value: i64) -> ComponentFieldSchema {
        self.default = Some(FieldDefault::Integer(value));
        self
    }

    /// 有効な整数値域(`mode`のみ)。
    const fn range(mut self, min: f64, max: f64) -> ComponentFieldSchema {
        self.min = Some(min);
        self.max = Some(max);
        self
    }
}

fn kind(name: &'static str, fields: Vec<ComponentFieldSchema>) -> ComponentKindSchema {
    ComponentKindSchema { kind: name, fields }
}

/// `apply_component`が受け付ける全kindのフィールドスキーマ
/// (`apply_component_impl`の`match kind`と同じ並び——差分レビューで
/// 突き合わせやすいように、意図的に順序まで揃えてある)。
pub fn apply_schema() -> Vec<ComponentKindSchema> {
    vec![
        // --- Joint(`sim_world::JointDesc`の薄い写像、縦串①)。アンカー点は
        // いずれも剛体ローカル座標[m]、`body_b`が負ならワールド固定点。 ---
        kind(
            "add_distance_joint",
            vec![
                u("body_a"),
                f("ax", "m"),
                f("ay", "m"),
                f("az", "m"),
                body_b(),
                f("bx", "m"),
                f("by", "m"),
                f("bz", "m"),
                f("length", "m"),
            ],
        ),
        kind(
            "add_ball_joint",
            vec![
                u("body_a"),
                f("ax", "m"),
                f("ay", "m"),
                f("az", "m"),
                body_b(),
                f("bx", "m"),
                f("by", "m"),
                f("bz", "m"),
            ],
        ),
        kind(
            "add_slider_joint",
            vec![
                u("body_a"),
                f("ax", "m"),
                f("ay", "m"),
                f("az", "m"),
                f_nd("axis_x"),
                f_nd("axis_y"),
                f_nd("axis_z"),
                body_b(),
                f("bx", "m"),
                f("by", "m"),
                f("bz", "m"),
            ],
        ),
        // `suspension_axis`/`axle_axis`は`WheelJoint::new`の既定値固定で
        // payloadに現れない(`add_wheel_joint_impl`のdocが述べる縮約)。
        // `frequency`は`SoftParams`の固有振動数[Hz]で、乗り心地周波数では
        // ないことに注意(`WheelJoint::soft`のdoc参照)。
        kind(
            "add_wheel_joint",
            vec![
                u("chassis"),
                u("wheel"),
                f("acx", "m"),
                f("acy", "m"),
                f("acz", "m"),
                f("rest_length", "m"),
                f("frequency", "Hz"),
                f_nd("damping_ratio"),
                f("steer_angle", "rad"),
                f("motor_speed", "rad/s"),
                f("motor_max_torque", "N·m"),
            ],
        ),
        // PD制御は ω_target = kp(θ_target-θ) - kd·θ̇(`HingeMotorPd::apply`)
        // なので、`kp`は[1/s]・`kd`は無次元——トルク係数[N·m/rad]ではない。
        kind(
            "add_hinge_motor_joint",
            vec![
                u("body"),
                f_nd("axis_x"),
                f_nd("axis_y"),
                f_nd("axis_z"),
                f("theta_target", "rad"),
                f("kp", "1/s"),
                f_nd("kd"),
                f("torque_max", "N·m"),
            ],
        ),
        // --- Coupling(縦串②・⑤)。 ---
        kind(
            "add_image_charge_force_coupling",
            vec![
                u("body"),
                f("charge", "C"),
                f_nd("plane_normal_x"),
                f_nd("plane_normal_y"),
                f_nd("plane_normal_z"),
                f("plane_d", "m"),
            ],
        ),
        kind(
            "add_lorentz_force_coupling",
            vec![u("body"), f("charge", "C")],
        ),
        kind(
            "add_buoyancy_drag_coupling",
            vec![
                u("body"),
                f("water_level", "m"),
                f("water_density", "kg/m^3"),
            ],
        ),
        kind("add_dissipation_to_heat_coupling", vec![u("thermal_node")]),
        kind("add_joule_heat_coupling", vec![u("thermal_node")]),
        // `viscosity`はストークス抵抗 γ=6πμr の**動粘性ではなく粘性**[Pa·s]
        // (`BrownianForce::viscosity`のdoc)。`seed`/`stream`は`SimRng`の
        // 乱数系列(同じ値なら同じ揺らぎが再現される)。
        kind(
            "add_brownian_force_coupling",
            vec![
                u("body"),
                f("radius", "m"),
                f("viscosity", "Pa·s"),
                u("thermal_node"),
                u("seed"),
                u("stream"),
            ],
        ),
        kind(
            "add_motor_coupling",
            vec![
                u("body"),
                f_nd("axis_x"),
                f_nd("axis_y"),
                f_nd("axis_z"),
                u("voltage_source_index"),
                f("torque_constant", "N·m/A"),
            ],
        ),
        kind(
            "add_induction_coupling",
            vec![
                u("body"),
                u("voltage_source_index"),
                f("length", "m"),
                f("magnetic_field", "T"),
                f_nd("axis_x"),
                f_nd("axis_y"),
                f_nd("axis_z"),
            ],
        ),
        kind(
            "add_thermal_node",
            vec![f("temperature", "K"), f("heat_capacity", "J/K")],
        ),
        // 引数を取らないドメイン有効化(冪等)。
        kind("enable_grid_fluid_2d_domain", vec![]),
        kind("enable_gas_compartment", vec![]),
        kind(
            "add_sph_rigid_coupling",
            vec![u("body"), f("radius", "m"), u("boundary_points")],
        ),
        kind(
            "add_grid_fluid_rigid_coupling",
            vec![u("body"), f("half_width", "m"), f("half_height", "m")],
        ),
        kind(
            "add_piston_gas_coupling",
            vec![
                u("body"),
                f_nd("axis_x"),
                f_nd("axis_y"),
                f_nd("axis_z"),
                f("area", "m^2"),
                f("initial_volume", "m^3"),
            ],
        ),
        // `chord_*`/`span_*`は剛体ローカル座標の方向(毎step姿勢でワールドへ
        // 回す、`LiftModel::Wing`のdoc)。`atmosphere_viscosity`は
        // `sim_fluid::Atmosphere::viscosity`と同じ動粘性係数[m^2/s]。
        kind(
            "add_wing_lift_coupling",
            vec![
                u("body"),
                f("wing_area", "m^2"),
                f_nd("chord_x"),
                f_nd("chord_y"),
                f_nd("chord_z"),
                f_nd("span_x"),
                f_nd("span_y"),
                f_nd("span_z"),
                f("atmosphere_density", "kg/m^3"),
                f("atmosphere_viscosity", "m^2/s"),
            ],
        ),
        kind(
            "add_magnus_lift_coupling",
            vec![
                u("body"),
                f("radius", "m"),
                f("atmosphere_density", "kg/m^3"),
                f("atmosphere_viscosity", "m^2/s"),
            ],
        ),
        // `coupling_index`は`World::couplings()`の登録index(`CouplingInfo::index`
        // と同じ体系)。範囲外・翼以外を指しても無言で無視される
        // (`push_set_coupling_control_surface_deflection_impl`のdoc)ため、
        // 事前に`read_component`の`"coupling_supported_params"`で確かめられる。
        kind(
            "push_set_coupling_control_surface_deflection",
            vec![u("coupling_index"), f("deflection_radians", "rad")],
        ),
        kind(
            "add_boussinesq_buoyancy_coupling",
            vec![
                u("thermal_node"),
                f("ambient_temperature", "K"),
                f("thermal_expansion_coefficient", "1/K"),
            ],
        ),
        // `mode`だけディスパッチのフォールバックが`unwrap_or(3)`
        // (=ForcedFlatPlate)で、他の`u(...)`の0とは違う。0..=3以外の値は
        // 3と区別が付かない(`_ => ForcedFlatPlate`)ので`max`は3。
        // `thermal_expansion_coefficient`は`<=0`が「理想気体近似
        // β=1/T_film」を意味するセンチネル(よって`nullable`)。
        kind(
            "add_convection_link_coupling",
            vec![
                u("fluid_node"),
                u("surface_node"),
                f("area", "m^2"),
                f("characteristic_length", "m"),
                u("mode").default_integer(3).range(0.0, 3.0),
                f("fluid_thermal_conductivity", "W/(m·K)"),
                f("kinematic_viscosity", "m^2/s"),
                f_nd("prandtl_number"),
                f("thermal_expansion_coefficient", "1/K").nullable(),
            ],
        ),
        // `initial_enthalpy`は`PhaseState::enthalpy`の初期値[J/kg]
        // (原点は融点における固相の終端H=0、負なら融点未満の固相)。
        kind(
            "add_phase_change_morph_coupling",
            vec![
                u("body"),
                u("thermal_node"),
                f("melting_temperature", "K"),
                f("latent_heat_fusion", "J/kg"),
                f("specific_heat_solid", "J/(kg·K)"),
                f("specific_heat_liquid", "J/(kg·K)"),
                f("initial_mass", "kg"),
                f("conductance", "W/K"),
                f("initial_enthalpy", "J/kg"),
            ],
        ),
        // --- 環境(縦串③)。 ---
        kind("set_gravity", vec![f("gravity", "m/s^2")]),
        // **既知の限界**: ゼロベクトル`(0,0,0)`は
        // `MechanicsSolver::set_gravity_direction`が既定の下向きへ安全に
        // フォールバックするセンチネルだが、それは**3成分にまたがる**条件で
        // あり、フィールド単位の`nullable`では表現できない(x単独が0なのは
        // ごく普通の入力なので`nullable`を立てるほうが誤りになる)。
        kind(
            "set_gravity_direction",
            vec![f_nd("x"), f_nd("y"), f_nd("z")],
        ),
        // **既知の限界(`s()`のdocに書いた列挙値の話と同じ形)**: `kind`は
        // `uniform`/`point_source`/`zero`の3値固定だが、このスキーマは
        // 「文字列」までしか言えない。さらにこのkindは**`kind`の値によって
        // 意味を持つフィールドが変わる**唯一の例外である
        // (`uniform`は`magnitude`と`x`/`y`/`z`、`point_source`は`center_*`と
        // `mu`、`zero`はどれも見ない)。`ComponentKindSchema`のdocが言う
        // 「payload形状が動的なkindは1つも無かった」はここで初めて破れる。
        // 表現形式を場合分け可能な形へ拡張するのではなく、**全フィールドを
        // 平坦に並べて余分な値は無視される**(既定値が渡っても`kind`が見ない)
        // 設計にして、この注記で限界を伝える方を選んだ——スキーマの表現力を
        // 上げると`apply_component_impl`との写像の単純さ(このモジュールの
        // 唯一の安全装置)が失われるため。
        kind(
            "push_set_gravity_field",
            vec![
                s("kind"),
                f("magnitude", "m/s^2"),
                f_nd("x"),
                f_nd("y"),
                f_nd("z"),
                f("center_x", "m"),
                f("center_y", "m"),
                f("center_z", "m"),
                f("mu", "m^3/s^2"),
            ],
        ),
        kind("set_dt", vec![f("dt", "s").positive()]),
        kind(
            "set_atmosphere",
            vec![
                f("density", "kg/m^3"),
                f("viscosity", "m^2/s"),
                f("wind_x", "m/s"),
                f("wind_y", "m/s"),
                f("wind_z", "m/s"),
            ],
        ),
        kind("clear_atmosphere", vec![]),
        kind(
            "set_water_region",
            vec![f("water_level", "m"), f("density", "kg/m^3")],
        ),
        kind("clear_water_region", vec![]),
        // --- Transform・RigidBody Componentの編集。 ---
        kind(
            "set_body_position_at",
            vec![u("index"), f("x", "m"), f("y", "m"), f("z", "m")],
        ),
        kind(
            "set_body_rotation_at",
            vec![u("index"), f_nd("x"), f_nd("y"), f_nd("z"), f_nd("w")],
        ),
        // `set_body_scale_at`は`scale`を検証しない(`set_body_scale_xyz_at`
        // だけが正の有限値を要求する)ので`min`を載せない——載せると
        // 「弾かれる」という誤った期待を与える。
        kind("set_body_scale_at", vec![u("index"), f_nd("scale")]),
        kind(
            "set_body_scale_xyz_at",
            vec![
                u("index"),
                f_nd("sx").positive(),
                f_nd("sy").positive(),
                f_nd("sz").positive(),
            ],
        ),
        kind(
            "push_apply_force",
            vec![u("body_index"), f("fx", "N"), f("fy", "N"), f("fz", "N")],
        ),
        kind(
            "push_set_body_mass",
            vec![u("body_index"), f("mass", "kg").positive()],
        ),
        kind("push_set_body_type", vec![u("body_index"), s("kind")]),
        // `group`/`mask`は衝突フィルタのビットマスク(u32)。
        kind(
            "push_set_collision_filter",
            vec![u("body_index"), u("group"), u("mask")],
        ),
        kind(
            "push_grab",
            vec![
                u("body_index"),
                f("target_x", "m"),
                f("target_y", "m"),
                f("target_z", "m"),
            ],
        ),
        kind(
            "push_move_grab",
            vec![
                u("body_index"),
                f("target_x", "m"),
                f("target_y", "m"),
                f("target_z", "m"),
            ],
        ),
        kind("push_release", vec![u("body_index")]),
        // --- スポーンパレット(設計§6)。 ---
        kind(
            "spawn_sphere",
            vec![
                f("x", "m"),
                f("y", "m"),
                f("z", "m"),
                f("radius", "m"),
                s("material_name"),
            ],
        ),
        kind(
            "spawn_capsule",
            vec![
                f("x", "m"),
                f("y", "m"),
                f("z", "m"),
                f("radius", "m"),
                f("half_height", "m"),
                s("material_name"),
            ],
        ),
        // L字形の寸法は固定(`spawn_compound_l_shape_impl`のdoc)なので
        // payloadは位置と材質だけ。
        kind(
            "spawn_compound_l_shape",
            vec![f("x", "m"), f("y", "m"), f("z", "m"), s("material_name")],
        ),
        kind(
            "spawn_convex_mesh_cube",
            vec![
                f("x", "m"),
                f("y", "m"),
                f("z", "m"),
                f("half", "m"),
                s("material_name"),
            ],
        ),
        kind(
            "spawn_box",
            vec![
                f("x", "m"),
                f("y", "m"),
                f("z", "m"),
                f("half_extent", "m"),
                s("material_name"),
            ],
        ),
        // 任意形状スポナー(`spawn_shape_json_impl`のdoc参照)。上の5つと違い
        // 寸法フィールドを持たず、形状そのものを`shape_json`
        // (`body_shape_json_at`が返すのと同じ`ShapeJson`のJSON表現)で受ける
        // ——`nullable`にしてあるのは、省略時の既定`""`が寸法の`0.0`のように
        // 「小さいが妥当な形状」にはならず`ShapeParseFailed`で弾かれる
        // 生の数値扱いできないフィールドだからである。
        kind(
            "spawn_shape_json",
            vec![
                s("shape_json").nullable(),
                f("x", "m"),
                f("y", "m"),
                f("z", "m"),
                s("material_name"),
            ],
        ),
        kind("remove_body_at", vec![u("index")]),
        kind("duplicate_body_at", vec![u("index"), f("offset", "m")]),
        kind(
            "derive_material",
            vec![
                s("base_name"),
                s("new_name"),
                f("density", "kg/m^3").positive(),
            ],
        ),
        // --- 回路(固定デモ + 自由配線回路エディタ)。 ---
        kind("set_circuit_switch_closed", vec![b("closed")]),
        kind("circuit_editor_reset", vec![u("num_nodes")]),
        kind(
            "circuit_editor_add_resistor",
            vec![u("a"), u("b"), f("resistance", "Ω")],
        ),
        kind(
            "circuit_editor_add_voltage_source",
            vec![u("a"), u("b"), f("voltage", "V")],
        ),
        kind(
            "circuit_editor_add_switch",
            vec![u("a"), u("b"), b("closed")],
        ),
        kind(
            "circuit_editor_set_switch_closed",
            vec![u("index"), b("closed")],
        ),
        kind(
            "circuit_editor_add_capacitor",
            vec![
                u("a"),
                u("b"),
                f("capacitance", "F"),
                f("initial_voltage", "V"),
            ],
        ),
        kind(
            "circuit_editor_add_inductor",
            vec![
                u("a"),
                u("b"),
                f("inductance", "H"),
                f("initial_current", "A"),
            ],
        ),
        // `n_vt`は $nV_T$(理想係数×熱電圧、300Kで≈25.85mV)なので単位はV。
        kind(
            "circuit_editor_add_diode",
            vec![
                u("anode"),
                u("cathode"),
                f("saturation_current", "A"),
                f("n_vt", "V"),
            ],
        ),
        kind(
            "circuit_editor_add_dc_motor",
            vec![
                u("a"),
                u("b"),
                f("winding_resistance", "Ω"),
                f("winding_inductance", "H"),
                f("back_emf_constant", "V·s/rad"),
            ],
        ),
        kind(
            "circuit_editor_set_motor_speed",
            vec![u("index"), f("angular_velocity", "rad/s")],
        ),
        kind("push_heat_source", vec![f("watts", "W")]),
        // --- フレーム階層・モーター。 ---
        kind("add_rotating_frame", vec![f("angular_velocity_z", "rad/s")]),
        kind(
            "add_child_frame",
            vec![
                u("parent_index"),
                f("origin_offset_x", "m"),
                f("origin_offset_y", "m"),
                f("origin_offset_z", "m"),
                f("angular_velocity_z", "rad/s"),
            ],
        ),
        kind(
            "set_motor_target_at",
            vec![u("index"), f("theta_target", "rad")],
        ),
        // --- 合成スポーン・Timeline。 ---
        kind(
            "spawn_pendulum",
            vec![
                f("pivot_x", "m"),
                f("pivot_y", "m"),
                f("pivot_z", "m"),
                f("arm_length", "m"),
                s("material_name"),
            ],
        ),
        kind(
            "spawn_motor_arm",
            vec![
                f("pivot_x", "m"),
                f("pivot_y", "m"),
                f("pivot_z", "m"),
                s("material_name"),
            ],
        ),
        // 水塊の寸法・粒子数は固定(`spawn_fluid_block_impl`)。
        kind("spawn_fluid_block", vec![]),
        kind("restore_snapshot", vec![u("index")]),
        kind("add_bookmark", vec![s("label")]),
        kind("restore_bookmark", vec![u("index")]),
    ]
}
