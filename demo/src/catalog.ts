// かんたんモード(3ステップ)のカタログ。
//
// **なぜこのファイルがあるか**: これまでエディタに最初に触れた人が目にするのは
// Unity 風の 6 パネル + 60 個以上のコントロールで、「落ちる球が見たい」だけの人が
// そこへ辿り着くには
//   Project ドロワーを開く → Scenes タブ → 43 行の一覧から `d1-free-fall.json`
//   を探す → 読み込む → Edit/Play の意味を知って Play を押す → 時間倍率が
//   足りないシーンでは何も動かないので原因を推測する
// という道のりを踏む必要があった。どれも**中の仕組みを知っている人向けの語彙**
// (Probe / Coupling / Edit モード / 時間倍率 / dt)で書かれており、知らない人は
// 最初の 1 分で詰まる。
//
// ここではその逆を用意する——**やりたいこと(見たい現象)から入って 3 手で動く**。
//   ① なにを見たい?(カテゴリ)
//   ② どれにする?(実験)
//   ③ うごかす(再生)
// そのために各実験へ、シーン JSON には無い「人間向けの情報」を持たせる:
//   - やさしい名前と一行説明(`d14b-cylinder-channel` ではなく「円柱のうしろの渦」)
//   - 見どころ(何が起きたら成功なのか)
//   - どこを見ればいいか(3D ビュー / グラフ / 場のパネル)
//   - 進める速さ(`pace`)——後述。中を知らない人が最初に踏む最大の地雷
//   - つまみ(その現象で意味のある値だけを、単位付きで 1〜3 個)
//   - いまの数値(プローブの生ラベル `BodyPosY[clock]` ではなく「ボールの高さ」)
//
// **`pace`(1 秒あたりに進める step 数)について**: 既存の「時間倍率」は
// *シミュレーション内の秒数* を実時間の何倍で進めるかを指す。ところがシーンごとに
// `dt` が 1e-12 秒(気体分子)から 31555 秒(太陽系)まで 16 桁も違うため、
// 同じ ×1 が「実時間どおり」にも「1 step 進むのに 8 時間かかる」にもなる。
// 実際 D34(太陽系儀)は上限の ×128 でも 1 step に 4 分かかり、**選んでも永遠に
// 何も起きない**。かんたんモードでは倍率ではなく「1 秒あたり何 step 進めるか」で
// 指定する——現象の時定数に合わせてここに書いた値が、そのまま「気持ちよく見える
// 速さ」になる。
//
// 数式・単位・判定基準は既存のシーン JSON と Rust 側のテストが持っているものと
// 同じ値を人間の言葉に翻訳しただけで、物理そのものは一切変えていない。

/** かんたんモードの「いまの数値」1 行。シーン JSON の `probes` の並び順を指す。 */
export type Readout = {
  /** `probes` 配列のインデックス。 */
  probe: number;
  /**
   * 複数のプローブから 1 つの値を作る場合の入力(`derive` と対で使う)。
   *
   * 天体シーンのプローブは x/y 成分に分かれているが、人が知りたいのは
   * 「太陽からどれだけ離れているか」「どれくらいの速さか」であって、
   * 成分そのものではない(`-109695037084 m` は誰にも読めない)。
   */
  probes?: number[];
  /** `probes` の値から表示する量を作る。 */
  derive?: (values: number[]) => number;
  /** 「ボールの高さ」のような、中を知らない人にも分かる名前。 */
  label: string;
  /** 表示単位。`format` を持つ場合は使われない。 */
  unit?: string;
  /** 小数点以下の桁数(既定 2)。 */
  digits?: number;
  /** 単位変換込みの整形(例: ケルビン → ℃)。 */
  format?: (value: number) => string;
};

/** つまみ 1 個。`apply` がシーン JSON(パース済みオブジェクト)を直接書き換える。 */
export type Knob = {
  id: string;
  /** 「落とす高さ」のような名前。 */
  label: string;
  kind: "range" | "choice";
  /** `range` のとき。 */
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  /** `choice` のとき。値は文字列 or 数値。 */
  options?: { label: string; value: string | number }[];
  /** 初期値。 */
  value: string | number;
  /** つまみの意味を一行で。 */
  hint?: string;
  apply: (scene: SceneJson, value: string | number) => void;
};

/** パース済みのシーン JSON。必要な部分だけを型として持つ(全体は Rust 側が検証する)。 */
export type SceneJson = {
  name?: string;
  world?: Record<string, unknown> & { gravity?: number; dt?: number };
  materials?: { extends?: string; name?: string; density?: number }[];
  bodies?: {
    name?: string;
    shape?: Record<string, unknown>;
    material?: string;
    type?: string;
    position?: number[];
    rotation?: number[];
    linear_velocity?: number[];
    mass_override?: number;
  }[];
  joints?: Record<string, unknown>[];
  thermal?: {
    ambient_temperature?: number;
    nodes?: { temperature?: number; heat_capacity?: number }[];
  };
  probes?: unknown[];
  [key: string]: unknown;
};

export type Experiment = {
  /** かんたんモード内での一意な ID(URL やテストからも指せるように安定させる)。 */
  id: string;
  /** 絵文字 1 文字。カードの視線の入口。 */
  icon: string;
  /** やさしい名前。 */
  title: string;
  /** 一行説明(カードに出る)。 */
  blurb: string;
  /** `scenes/` のファイル名。`build` を持つ実験(自作)では省略。 */
  file?: string;
  /** シーンを JS 側で組み立てる実験(「じぶんで作る」)。 */
  build?: (values: Record<string, string | number>) => SceneJson;
  /**
   * つまみを当てる前にシーンへ施す下ごしらえ。
   *
   * 検証用シーンは「解析解と一致するか」を測るための最小構成なので、床が
   * 無いものがある(D1 の落下時計・D2 の弾道)。物理としては正しいが、
   * **落ちる球がどこまでも落ち続けて着地しない**画面は、初めて見る人には
   * ただの故障に見える。かんたんモードでは床を足して「落ちて、着く」まで
   * 見せる(落下中の運動そのものは何も変えない)。
   */
  prepare?: (scene: SceneJson) => void;
  /** 見どころ。「何が起きたら成功か」を 2〜4 行で。 */
  watch: string[];
  /** どこを見れば現象が見えるか。 */
  view: "3d" | "graph" | "field";
  /** 1 秒あたりに進める step 数(モジュール冒頭の doc 参照)。 */
  pace: number;
  readouts?: Readout[];
  knobs?: Knob[];
  /**
   * グラフの凡例に出す名前(プローブ番号 → 表示名)。
   * 「いまの数値」に出さないプローブや、複数プローブから作る値のもとになった
   * プローブに名前を与えるために使う。指定が無ければ Rust 側の生ラベル。
   */
  series?: Record<number, string>;
};

export type Category = {
  id: string;
  icon: string;
  title: string;
  /** 「〜が見たい」の形で書く。選ぶ側の言葉に合わせるため。 */
  blurb: string;
  experiments: Experiment[];
};

// ---------------------------------------------------------------------------
// つまみが使う小さなヘルパ。どれも「パース済みシーン JSON を書き換えるだけ」で、
// 物理エンジンには手を触れない(読み込みは通常のシーン読み込み経路を通る)。
// ---------------------------------------------------------------------------

function body(scene: SceneJson, name: string) {
  return scene.bodies?.find((b) => b.name === name);
}

/** 重力[m/s²]。天体シーン(`astro`)は自前の万有引力を使うので対象外。 */
const GRAVITY_OPTIONS = [
  { label: "🌍 地球 (9.8)", value: 9.80665 },
  { label: "🌙 月 (1.6)", value: 1.62 },
  { label: "🔴 火星 (3.7)", value: 3.71 },
  { label: "🪐 木星 (24.8)", value: 24.79 },
  { label: "🌌 無重力 (0)", value: 0 },
];

function gravityKnob(): Knob {
  return {
    id: "gravity",
    label: "重力",
    kind: "choice",
    options: GRAVITY_OPTIONS,
    value: 9.80665,
    hint: "どの天体の上で実験するか。落ちる速さが変わります。",
    apply: (scene, value) => {
      scene.world = { ...(scene.world ?? {}), gravity: Number(value) };
    },
  };
}

/** 材質。反発・摩擦・密度がまとめて変わるので、体感の差が大きい。 */
function materialKnob(bodyName: string, label = "材質"): Knob {
  return {
    id: "material",
    label,
    kind: "choice",
    options: [
      { label: "🔩 鋼", value: "鋼(炭素鋼)" },
      { label: "🏀 ゴム", value: "ゴム(天然)" },
      { label: "🪵 木", value: "木材(松)" },
      { label: "🧊 氷", value: "氷(0°C)" },
      { label: "🫧 発泡スチロール", value: "発泡スチロール" },
      { label: "🥫 アルミ", value: "アルミニウム" },
    ],
    value: "鋼(炭素鋼)",
    hint: "重さ・跳ね返り・すべりやすさが一度に変わります。",
    apply: (scene, value) => {
      const target = body(scene, bodyName);
      if (target) target.material = String(value);
    },
  };
}

function heightKnob(bodyName: string, initial: number): Knob {
  return {
    id: "height",
    label: "落とす高さ",
    kind: "range",
    min: 1,
    max: 50,
    step: 1,
    unit: "m",
    value: initial,
    hint: "高いほど、着地までの時間も着地の速さも大きくなります。",
    apply: (scene, value) => {
      const target = body(scene, bodyName);
      if (target) target.position = [0, Number(value), 0];
    },
  };
}

/** 床(y=0 の静的な無限平面)を足す。既にあれば何もしない。 */
function addGround(scene: SceneJson): void {
  const bodies = scene.bodies ?? [];
  if (bodies.some((b) => b.shape && "plane" in b.shape)) return;
  scene.bodies = [
    {
      name: "ground",
      shape: { plane: { normal: [0, 1, 0], d: 0 } },
      type: "static",
      material: "コンクリート",
    },
    ...bodies,
  ];
}

const KELVIN = 273.15;
const celsius = (digits = 1) => (v: number) => `${(v - KELVIN).toFixed(digits)} ℃`;
/** 天体スケールの距離。m のままでは桁が読めないので億 km で書く(地球〜太陽 ≒ 1.5 億 km)。 */
const okuKm = (v: number) => `${(v / 1e11).toFixed(3)} 億 km`;
/** 天体スケールの速さ。 */
const kmPerSecond = (digits = 2) => (v: number) => `${(v / 1000).toFixed(digits)} km/s`;
const hypot = (values: number[]) => Math.hypot(...values);

// ---------------------------------------------------------------------------
// カタログ本体。
// ---------------------------------------------------------------------------

export const GUIDED_CATEGORIES: Category[] = [
  {
    id: "drop",
    icon: "🍎",
    title: "落とす・ぶつける",
    blurb: "物が落ちる、跳ねる、崩れる、揺れる。",
    experiments: [
      {
        id: "d1-free-fall",
        file: "d1-free-fall.json",
        icon: "🎯",
        title: "ボールを落とす",
        blurb: "高さ 20 m から鉄の球を落とします。",
        watch: [
          "落ちるほど速くなります(等加速度)。",
          "高さ 20 m のときで、着地はおよそ 2.0 秒後・速さ約 20 m/s(高さを変えると、どちらも変わります)。",
          "重力を月に変えると、同じ高さでも 2.5 倍ゆっくり落ちます。",
        ],
        view: "3d",
        pace: 120,
        prepare: addGround,
        readouts: [{ probe: 0, label: "ボールの高さ", unit: "m" }],
        knobs: [heightKnob("clock", 20), gravityKnob()],
      },
      {
        id: "d3-bounce",
        file: "d3-bounce.json",
        icon: "🏀",
        title: "ボールを跳ねさせる",
        blurb: "ゴムの球を落として、跳ね返る高さを見ます。",
        watch: [
          "1 回跳ねるごとに高さが決まった割合で減ります。",
          "材質を鋼や木に変えると、跳ね方がはっきり変わります。",
          "グラフの山の高さが、跳ね返るたびに低くなっていきます。",
        ],
        view: "3d",
        pace: 240,
        readouts: [{ probe: 0, label: "ボールの高さ", unit: "m" }],
        knobs: [heightKnob("ball", 2), materialKnob("ball", "ボールの材質")],
      },
      {
        id: "d2-ballistic",
        file: "d2-ballistic.json",
        icon: "🏹",
        title: "斜めに投げる",
        blurb: "秒速 20 m で 45° に投げ上げた球の軌跡。",
        watch: [
          "上りと下りが左右対称の放物線になります。",
          "45° がいちばん遠くまで飛びます(空気抵抗なしのとき)。",
          "角度を変えると、飛距離と滞空時間の関係が見えます。",
        ],
        view: "3d",
        pace: 120,
        prepare: (scene) => {
          // 射出点は原点なので、床を足す前に少しだけ持ち上げる(でないと
          // 発射の瞬間に床とめり込んだ状態から始まる)。
          const shell = body(scene, "shell");
          if (shell) shell.position = [0, 0.2, 0];
          addGround(scene);
        },
        readouts: [
          { probe: 0, label: "球の高さ", unit: "m" },
          { probe: 1, label: "速さ", unit: "m/s" },
        ],
        knobs: [
          {
            id: "angle",
            label: "投げる角度",
            kind: "range",
            min: 10,
            max: 80,
            step: 5,
            unit: "°",
            value: 45,
            hint: "45° が最も遠くまで飛びます。",
            apply: (scene, value) => {
              const target = body(scene, "shell");
              if (!target) return;
              const speed = Math.hypot(
                target.linear_velocity?.[0] ?? 20,
                target.linear_velocity?.[1] ?? 0,
              );
              const rad = (Number(value) * Math.PI) / 180;
              target.linear_velocity = [
                speed * Math.cos(rad),
                speed * Math.sin(rad),
                0,
              ];
            },
          },
          gravityKnob(),
        ],
      },
      {
        id: "d5-incline",
        file: "d5-incline-static.json",
        icon: "⛷️",
        title: "坂はすべる? 止まる?",
        blurb: "坂の傾きを変えて、箱が動き出す角度を探します。",
        watch: [
          "傾きが小さいうちは、摩擦が勝って箱は止まったままです。",
          "ある角度を超えた瞬間に滑り出します(鋼と鋼ならおよそ 30° 前後)。",
          "「箱の速さ」が 0 のままか、増えていくかで見分けられます。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 0, label: "箱の速さ", unit: "m/s" }],
        knobs: [
          {
            id: "slope",
            label: "坂のかたむき",
            kind: "range",
            min: 5,
            max: 60,
            step: 1,
            unit: "°",
            value: 10,
            hint: "少しずつ上げて、滑り出す角度を探してみてください。",
            apply: (scene, value) => {
              // 斜面は「原点を通る傾いた無限平面」+「その上に載る箱」で表す。
              // 角度を変えるときは平面の法線・箱の姿勢・箱の位置の 3 つを
              // 同時に回さないと、箱が坂へめり込む(または浮く)。
              const rad = (Number(value) * Math.PI) / 180;
              const sin = Math.sin(rad);
              const cos = Math.cos(rad);
              const plane = scene.bodies?.[0];
              if (plane?.shape?.plane) {
                (plane.shape.plane as { normal: number[] }).normal = [
                  -sin,
                  cos,
                  0,
                ];
              }
              const box = body(scene, "box");
              if (box) {
                const half = 0.5;
                box.position = [-half * sin, half * cos, 0];
                box.rotation = [0, 0, Math.sin(rad / 2), Math.cos(rad / 2)];
              }
            },
          },
          materialKnob("box", "箱の材質"),
        ],
      },
      {
        id: "d4-box-stack",
        file: "d4-box-stack.json",
        icon: "🧱",
        title: "積み木を積む",
        blurb: "積み上げた箱が崩れずに立っていられるかを見ます。",
        watch: [
          "接触の計算が安定していれば、数秒で完全に静止します。",
          "「いちばん上の箱の速さ」が 0 に落ち着けば成功です。",
          "段数を増やすほど、静まるまでに時間がかかります。",
        ],
        view: "3d",
        pace: 120,
        readouts: [
          { probe: 2, label: "いちばん上の箱の速さ", unit: "m/s", digits: 3 },
        ],
        series: { 0: "1 段目の速さ", 1: "2 段目の速さ" },
        knobs: [
          {
            id: "floors",
            label: "積む段数",
            kind: "range",
            min: 2,
            max: 8,
            step: 1,
            unit: "段",
            value: 3,
            hint: "高く積むほど崩れやすくなります。",
            apply: (scene, value) => {
              const floors = Math.max(2, Math.trunc(Number(value)));
              const ground = scene.bodies?.[0];
              const boxes = [];
              for (let i = 0; i < floors; i += 1) {
                boxes.push({
                  shape: { box: { half: [0.5, 0.5, 0.5] } },
                  material: "鋼(炭素鋼)",
                  position: [0, 0.5 + i * 1.01, 0],
                  name: `box${i + 1}`,
                });
              }
              scene.bodies = ground ? [ground, ...boxes] : boxes;
              // プローブは「1 段目・2 段目・いちばん上」を見る。
              scene.probes = [
                { body_speed: "box1" },
                { body_speed: "box2" },
                { body_speed: `box${floors}` },
              ];
            },
          },
        ],
      },
      {
        id: "d11-pendulum",
        file: "d11-pendulum.json",
        icon: "🕰️",
        title: "ふりこ",
        blurb: "ひもで吊るしたおもりが往復します。",
        watch: [
          "1 往復の時間は、おもりの重さではなく「ひもの長さ」で決まります。",
          "長さ 1 m のときで、およそ 2.0 秒に 1 往復。",
          "長さを 4 倍にすると、往復の時間は 2 倍になります。",
        ],
        view: "3d",
        pace: 120,
        prepare: (scene) => {
          // **見えないものは観察できない**。検証用の`d11`のおもりは半径 1cm で、
          // 画面では点にしかならず「ひもで吊るしたおもり」に見えなかった
          // (利用者役の観察: 真っ黒な画面に小さな点が一つ)。
          //
          // 単振り子の周期は ひもの長さ と 重力 だけで決まり、おもりの半径には
          // 依存しない(質量は`mass_override`で 1kg に固定されている)。つまり
          // 見える大きさに変えても、確かめたい関係は変わらない。
          const bob = body(scene, "bob");
          if (bob?.shape?.sphere) {
            (bob.shape.sphere as { radius: number }).radius = 0.08;
          }
          // 支点。何から吊るされているのかが分からないと、往復の意味が読めない。
          // 静的な小球なので運動には関与しない(おもりは 1m 先を回る)。
          scene.bodies = [
            {
              name: "pivot",
              shape: { sphere: { radius: 0.05 } },
              material: "鋼(炭素鋼)",
              type: "static",
              position: [0, 0, 0],
            },
            ...(scene.bodies ?? []),
          ];
        },
        readouts: [
          { probe: 0, label: "おもりの横位置", unit: "m", digits: 3 },
          { probe: 1, label: "おもりの高さ", unit: "m", digits: 3 },
        ],
        knobs: [
          {
            id: "length",
            label: "ひもの長さ",
            kind: "range",
            min: 0.25,
            max: 4,
            step: 0.25,
            unit: "m",
            value: 1,
            hint: "長いほどゆっくり揺れます。",
            apply: (scene, value) => {
              const length = Number(value);
              // 支点は原点。初期角度(約 2.9°)を保ったまま長さだけ変える。
              const angle = 0.05;
              const bob = body(scene, "bob");
              if (bob) {
                bob.position = [
                  length * Math.sin(angle),
                  -length * Math.cos(angle),
                  0,
                ];
              }
              const joint = scene.joints?.[0] as
                | { distance?: { length?: number } }
                | undefined;
              if (joint?.distance) joint.distance.length = length;
            },
          },
          gravityKnob(),
        ],
      },
      {
        id: "d8-scatter",
        file: "d8-scatter.json",
        icon: "🎲",
        title: "50 個の球をばらまく",
        blurb: "球の山が崩れて床に散らばります。",
        watch: [
          "何度やり直しても、まったく同じ散らばり方になります(決定論)。",
          "同じ入力なら結果もビット単位で同じ——それがこのエンジンの土台です。",
          "「はじめから」を押して見比べてみてください。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 1, label: "先頭の球の高さ", unit: "m" }],
      },
      {
        id: "d12-ragdoll",
        file: "d12-ragdoll.json",
        icon: "🧸",
        title: "人形が倒れる",
        blurb: "関節でつないだ人形が床へ崩れ落ちます。",
        watch: [
          "腕や頭が関節でつながったまま動きます(外れません)。",
          "床をすり抜けません。",
          "動きは必ず落ち着きます——勝手に暴れ出したら計算の破綻です。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 0, label: "胴体の高さ", unit: "m" }],
      },
      {
        id: "d13-rope",
        file: "d13-rope.json",
        icon: "🪢",
        title: "ロープが垂れる",
        blurb: "両端を留めたロープの垂れ方を見ます。",
        watch: [
          "垂れた形は「懸垂線(カテナリー)」——放物線とよく似た別の曲線です。",
          "ゆらゆら揺れたあと、静かな形に落ち着きます。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 0, label: "まん中の高さ", unit: "m", digits: 3 }],
      },
    ],
  },
  {
    id: "water",
    icon: "💧",
    title: "水・空気・流れ",
    blurb: "浮く・沈む・注ぐ・渦を巻く。",
    experiments: [
      {
        id: "d23-pouring-water",
        file: "d23-pouring-water.json",
        icon: "🚰",
        title: "水を注ぐ",
        blurb: "水のかたまりが落ちて、容器に溜まります。",
        watch: [
          "水面が波打ちながら、だんだん静かになります。",
          "粒 1 つ 1 つが水の分子ではなく「小さな水の塊」です。",
          "底の粒の密度が上がるのは、水圧で押し縮められているからです。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 0, label: "水面近くの粒の高さ", unit: "m", digits: 3 }],
      },
      {
        id: "d6-floating",
        file: "d6-floating-box-f4.json",
        icon: "🛟",
        title: "浮くか、沈むか",
        blurb: "木の箱を水に浮かべ、どこまで沈むかを見ます。",
        watch: [
          "沈む深さは「箱の重さ ÷ 水の重さ」で決まります(アルキメデス)。",
          "密度 600 kg/m³ のときで、水面下に 6 割が沈んで釣り合います。",
          "1000 を超えると沈み、超えなければ浮きます。",
        ],
        view: "graph",
        pace: 120,
        readouts: [{ probe: 0, label: "箱の高さ(水面が 0)", unit: "m", digits: 3 }],
        knobs: [
          {
            id: "density",
            label: "箱の密度",
            kind: "range",
            min: 200,
            max: 1400,
            step: 50,
            unit: "kg/m³",
            value: 600,
            hint: "水は 1000 kg/m³。これを超えると沈みます。",
            apply: (scene, value) => {
              const material = scene.materials?.[0];
              if (material) material.density = Number(value);
              // 初期位置は「だいたい釣り合う位置」に置く(沈む場合も自然に沈む)。
              const box = body(scene, "box");
              const draft = Math.min(Number(value) / 998.2, 1);
              if (box) box.position = [0, 0.5 - draft, 0];
            },
          },
        ],
      },
      {
        id: "d14-vortex",
        file: "d14-vortex.json",
        icon: "🌀",
        title: "渦ができる",
        blurb: "流れの中に柱を置くと、うしろに渦の列ができます。",
        watch: [
          "柱のうしろで流れが左右に振れ始めます(カルマン渦列)。",
          "グラフの「上下の揺れの強さ」が 0 から立ち上がります。",
          "旗がはためくのも、電線が唸るのも同じ現象です。",
        ],
        view: "graph",
        pace: 120,
        readouts: [
          { probe: 0, label: "上下の揺れの強さ", digits: 4 },
          { probe: 1, label: "平均の流れ", digits: 4 },
        ],
      },
      {
        id: "d14c-smoke-3d",
        file: "d14c-smoke-3d.json",
        icon: "💨",
        title: "煙が流れる(3D)",
        blurb: "球のまわりを煙が流れていきます。",
        watch: [
          "煙の塊が上流から下流へ運ばれます。",
          "球の裏側で巻き込まれ、まっすぐには通り抜けません。",
        ],
        view: "3d",
        pace: 60,
      },
      {
        id: "d15-convection",
        file: "d15-convection.json",
        icon: "🕯️",
        title: "あたたかい空気が昇る",
        blurb: "下から温めると、流れが自然に立ち上がります。",
        watch: [
          "温めた場所の上に、上向きの流れができます(対流)。",
          "グラフの「平均の上下の流れ」がプラス側へ動きます。",
          "エアコンや風呂の温度差も、これと同じ理屈です。",
        ],
        view: "graph",
        pace: 120,
        readouts: [
          { probe: 0, label: "平均の上下の流れ", digits: 4 },
          { probe: 1, label: "熱源の温度", format: celsius() },
        ],
      },
      {
        id: "d7-terminal",
        file: "d7-wind-high-re.json",
        icon: "🪂",
        title: "空気抵抗で速さが頭打ちになる",
        blurb: "空気の中を落ちる鉄球。速さが一定に落ち着きます。",
        watch: [
          "最初は加速しますが、途中から速さが増えなくなります(終端速度)。",
          "落ち続けているのに速さが一定——これが空気抵抗との釣り合いです。",
          "パラシュートが安全なのはこの効果です。",
        ],
        view: "graph",
        pace: 120,
        readouts: [{ probe: 0, label: "落ちる速さ", unit: "m/s" }],
      },
    ],
  },
  {
    id: "heat",
    icon: "🔥",
    title: "熱・温度",
    blurb: "冷める、伝わる、融ける、摩擦で熱くなる。",
    experiments: [
      {
        id: "d9-cooling-coffee",
        file: "d9-cooling-coffee.json",
        icon: "☕",
        title: "コーヒーが冷める",
        blurb: "熱いコーヒーが部屋の温度まで下がっていきます。",
        watch: [
          "最初は速く、あとはゆっくり冷めます(指数的な冷却)。",
          "約 10 秒で温度差が 1/e(約 37%)まで縮みます。",
          "部屋の温度より下がることはありません。",
        ],
        view: "graph",
        pace: 240,
        readouts: [{ probe: 0, label: "コーヒーの温度", format: celsius() }],
        knobs: [
          {
            id: "temperature",
            label: "淹れたての温度",
            kind: "range",
            min: 40,
            max: 95,
            step: 5,
            unit: "℃",
            value: 77,
            hint: "何度から始めても、行き着く先は部屋の温度です。",
            apply: (scene, value) => {
              const node = scene.thermal?.nodes?.[0];
              if (node) node.temperature = Number(value) + KELVIN;
            },
          },
          {
            id: "room",
            label: "部屋の温度",
            kind: "range",
            min: -10,
            max: 40,
            step: 5,
            unit: "℃",
            value: 20,
            hint: "冷たい部屋ほど速く冷めます。",
            apply: (scene, value) => {
              if (scene.thermal) {
                scene.thermal.ambient_temperature = Number(value) + KELVIN;
              }
            },
          },
        ],
      },
      {
        id: "d10-brake-heat",
        file: "d10-brake-heat.json",
        icon: "🛞",
        title: "こすれると熱くなる",
        blurb: "滑る箱が摩擦で止まり、そのぶん温度が上がります。",
        watch: [
          "箱の速さが 0 になるのと同時に、温度の上昇も止まります。",
          "消えた運動エネルギーは熱に変わっただけ——合計は保存されます。",
          "ブレーキが熱くなるのと同じ話です。",
        ],
        view: "3d",
        pace: 120,
        readouts: [
          { probe: 0, label: "箱の速さ", unit: "m/s" },
          { probe: 1, label: "ブレーキの温度", format: celsius(2) },
        ],
      },
      {
        id: "d16-conduction-race",
        file: "d16-conduction-race.json",
        icon: "🥄",
        title: "熱が棒を伝わる",
        blurb: "銅の棒の片端を 100 ℃に保つと、熱が奥へ伝わります。",
        watch: [
          "近い場所から順に温度が上がっていきます。",
          "遠い場所ほど、上がり始めるまでに時間がかかります。",
          "金属のスプーンがすぐ熱くなり、木の柄が熱くならない理由です。",
        ],
        view: "graph",
        pace: 40,
        readouts: [
          // 棒の温度は**摂氏の配列**(熱源端が 100)。熱ノードの温度(ケルビン)と
          // 同じ換算をかけると `-271.0 ℃` のような値になり、しかも文章側の
          // 「片端を 100 ℃に保つ」と食い違う。単位は測っているものごとに違う。
          { probe: 0, label: "熱源から近い点", unit: "℃", digits: 1 },
          { probe: 1, label: "まん中の点", unit: "℃", digits: 1 },
          { probe: 2, label: "遠い点", unit: "℃", digits: 1 },
        ],
      },
      {
        id: "d18-ice-in-drink",
        file: "d18-ice-in-drink.json",
        icon: "🧊",
        title: "氷が融ける",
        blurb: "飲み物に浮かべた氷が融けて小さくなります。",
        watch: [
          "融けて軽くなるにつれ、氷が浮き上がってきます。",
          "融けている間、温度はすぐには上がりません(融解に熱を使うため)。",
        ],
        view: "graph",
        pace: 240,
        readouts: [
          { probe: 0, label: "氷の高さ", unit: "m", digits: 3 },
          { probe: 1, label: "飲み物の温度", format: celsius() },
        ],
      },
      {
        id: "d17-piston",
        file: "d17-piston.json",
        icon: "🫁",
        title: "空気をばねにする",
        blurb: "閉じ込めた気体をピストンで押し縮めます。",
        watch: [
          "押し込むと押し返されます——気体はばねとして働きます。",
          "ピストンの位置が行ったり来たりを繰り返します。",
        ],
        view: "3d",
        pace: 120,
        readouts: [
          { probe: 0, label: "ピストンの位置", unit: "m", digits: 3 },
          { probe: 1, label: "ピストンの速さ", unit: "m/s", digits: 3 },
        ],
      },
    ],
  },
  {
    id: "electric",
    icon: "⚡",
    title: "電気・磁石",
    blurb: "回路に電気を流す、磁石で発電する、電波を飛ばす。",
    experiments: [
      {
        id: "d19-electric-workbench",
        file: "d19-electric-workbench.json",
        icon: "🔌",
        title: "電気の工作台",
        blurb: "電池・抵抗・コンデンサ・LED をつないだ回路。",
        watch: [
          "分圧点の電圧が、抵抗の比のとおりの値に落ち着きます。",
          "コンデンサの電圧はゆっくり下がっていきます(放電)。",
          "グラフの各線が、回路の各点の電圧です。",
        ],
        view: "graph",
        pace: 120,
        readouts: [
          { probe: 0, label: "分圧点の電圧", unit: "V", digits: 3 },
          { probe: 1, label: "コンデンサの電圧", unit: "V", digits: 3 },
        ],
      },
      {
        id: "d20-generator",
        file: "d20-hand-crank-generator.json",
        icon: "🔦",
        title: "手回し発電機",
        blurb: "クランクを回すと電気が生まれ、抵抗が熱くなります。",
        watch: [
          "回転から電圧が生まれます(電磁誘導)。",
          "流れた電流のぶんだけ、抵抗の温度が上がります。",
          "回した力が電気になり、最後は熱になる——エネルギーの旅路です。",
        ],
        view: "graph",
        pace: 120,
        readouts: [
          { probe: 0, label: "発電した電圧", unit: "V", digits: 3 },
          { probe: 1, label: "流れた電流", unit: "A", digits: 4 },
          { probe: 2, label: "抵抗の温度", format: celsius(2) },
        ],
      },
      {
        id: "d21-copper-tube",
        file: "d21-copper-tube-drop.json",
        icon: "🧲",
        title: "磁石が銅管をゆっくり落ちる",
        blurb: "磁石を銅の管に落とすと、なぜかゆっくり落ちます。",
        watch: [
          "自由落下より明らかに遅く、やがて一定の速さになります。",
          "銅の中に生まれた渦電流が、落下を邪魔しています。",
          "銅は磁石にくっつかないのに、ブレーキはかかります。",
        ],
        view: "graph",
        pace: 240,
        readouts: [{ probe: 0, label: "落ちる速さ", unit: "m/s", digits: 3 }],
      },
      {
        id: "d29-radio-tank",
        file: "d29-radio-tank.json",
        icon: "📡",
        title: "電波が広がって反射する",
        blurb: "金属の箱の中で電波のパルスを起こします。",
        watch: [
          "中心から波が輪になって広がります。",
          "壁で跳ね返り、重なり合って複雑な模様になります。",
          "画面の色が電界の強さです(場のパネル)。",
        ],
        view: "field",
        pace: 120,
        readouts: [{ probe: 0, label: "全体のエネルギー", digits: 4 }],
      },
      {
        id: "d26-balloon",
        file: "d26-balloon-qualitative.json",
        icon: "🎈",
        title: "静電気で風船がくっつく",
        blurb: "こすった風船が壁に引き寄せられます。",
        watch: [
          "壁に近づくほど引力が強くなり、加速しながら吸い寄せられます。",
          "壁の中に「鏡に映った電荷」があるかのように振る舞います。",
        ],
        view: "graph",
        pace: 120,
        readouts: [{ probe: 0, label: "壁からの距離", unit: "m", digits: 3 }],
      },
    ],
  },
  {
    id: "space",
    icon: "🪐",
    title: "宇宙",
    blurb: "惑星が回る、探査機が加速する、カプセルが帰ってくる。",
    experiments: [
      {
        id: "d34-solar-system",
        file: "d34-solar-system-single-planet.json",
        icon: "🌍",
        title: "惑星が太陽を回る",
        blurb: "地球と同じ軌道・同じ速さで 1 年を回ります。",
        watch: [
          "きれいな円を描いて戻ってきます。",
          "1 周でちょうど 1 年。グラフの波 1 つが 1 年です。",
          "太陽の引力だけで、燃料なしに回り続けます。",
        ],
        view: "graph",
        pace: 120,
        readouts: [
          {
            probe: 0,
            probes: [0, 1],
            derive: hypot,
            label: "太陽からの距離",
            format: okuKm,
          },
        ],
        series: { 0: "横の位置", 1: "縦の位置" },
      },
      {
        id: "d35-orbital-insertion",
        file: "d35-orbital-insertion.json",
        icon: "🛰️",
        title: "軌道に乗せる",
        blurb: "円軌道より 1 割遅い速さで投入するとどうなるか。",
        watch: [
          "円ではなく、つぶれた楕円になります。",
          "近いところでは速く、遠いところではゆっくり動きます(ケプラー)。",
        ],
        view: "graph",
        pace: 120,
        readouts: [
          {
            probe: 0,
            probes: [0, 1],
            derive: hypot,
            label: "中心からの距離",
            format: okuKm,
          },
          {
            probe: 2,
            probes: [2, 3],
            derive: hypot,
            label: "速さ",
            format: kmPerSecond(),
          },
        ],
        series: { 0: "横の位置", 1: "縦の位置" },
      },
      {
        id: "d36-swingby",
        file: "d36-swingby.json",
        icon: "🚀",
        title: "スイングバイで加速する",
        blurb: "惑星のそばを通るだけで、探査機が速くなります。",
        watch: [
          "燃料を使わずに速さが増えます(惑星の運動量を少し借ります)。",
          "グラフの「探査機の速さ」が、接近の前後で階段状に上がります。",
          "ボイジャーが太陽系の外へ出られたのはこの技です。",
        ],
        view: "graph",
        pace: 240,
        readouts: [
          {
            probe: 2,
            probes: [2, 3],
            derive: hypot,
            label: "探査機の速さ",
            format: kmPerSecond(),
          },
          {
            probe: 0,
            probes: [0, 1],
            derive: hypot,
            label: "惑星からの距離",
            format: (v) => `${(v / 1000).toFixed(0)} km`,
          },
        ],
        series: {
          0: "横の位置",
          1: "縦の位置",
          2: "横の速さ",
          3: "縦の速さ",
        },
      },
      {
        id: "d37-reentry",
        file: "d37-reentry.json",
        icon: "☄️",
        title: "大気圏に突入する",
        blurb: "高度 120 km のカプセルが空気に突っ込んで減速します。",
        watch: [
          "秒速 6.7 km から、秒速 72 m まで一気に落ちます。",
          "減速は薄い上層ではなく、濃い下層で急激に起きます。",
          "この減速ぶんのエネルギーが、あの高温の正体です。",
        ],
        view: "graph",
        pace: 240,
        readouts: [
          {
            probe: 2,
            probes: [2, 3],
            derive: hypot,
            label: "速さ",
            format: kmPerSecond(2),
          },
          {
            probe: 0,
            probes: [0, 1],
            derive: (v) => hypot(v) - 6_371_000, // 地球半径を引いて高度にする
            label: "高度",
            format: (v) => `${(v / 1000).toFixed(1)} km`,
          },
        ],
        series: {
          0: "横の位置",
          1: "縦の位置",
          2: "横の速さ",
          3: "縦の速さ",
        },
      },
      {
        id: "d39-relativity",
        file: "d39-relativity.json",
        icon: "🌌",
        title: "アインシュタインの補正",
        blurb: "楕円軌道の向きが、少しずつ回っていきます。",
        watch: [
          "ニュートン力学だけなら、楕円の向きは動かないはずです。",
          "一般相対論の補正を入れると、ゆっくり回ります(近日点移動)。",
          "水星で実際に観測され、相対論の証拠になった現象です。",
        ],
        view: "graph",
        pace: 240,
        readouts: [
          {
            probe: 0,
            probes: [0, 1],
            derive: hypot,
            label: "中心からの距離",
            format: okuKm,
          },
        ],
        series: { 0: "横の位置", 1: "縦の位置" },
      },
    ],
  },
  {
    id: "micro",
    icon: "🔬",
    title: "ミクロの世界",
    blurb: "電子の波、分子の運動、磁石が生まれる瞬間。",
    experiments: [
      {
        id: "d27-double-slit",
        file: "d27-double-slit.json",
        icon: "🌊",
        title: "二重スリット",
        blurb: "電子を 2 本の隙間に通すと、しま模様ができます。",
        watch: [
          "波が 2 つの隙間を抜けて、向こう側で重なります。",
          "明るい線と暗い線が交互に並びます(干渉縞)。",
          "粒のはずの電子が、波としてふるまう証拠です。",
        ],
        view: "field",
        pace: 60,
      },
      {
        id: "d28-tunneling",
        file: "d28-tunneling.json",
        icon: "🚪",
        title: "壁をすり抜ける",
        blurb: "越えられないはずの壁を、電子の波が一部通り抜けます。",
        watch: [
          "波が壁に当たると、跳ね返る波と通り抜ける波に分かれます。",
          "「通り抜けた割合」が 0 より大きくなります(トンネル効果)。",
          "全体の量は常にちょうど 1 のまま——確率は消えません。",
        ],
        view: "field",
        pace: 60,
        readouts: [
          { probe: 3, label: "通り抜けた割合", digits: 4 },
          { probe: 0, label: "全体の量(常に 1)", digits: 6 },
        ],
      },
      {
        id: "d33-electron-in-well",
        file: "d33-electron-in-well.json",
        icon: "🔒",
        title: "閉じ込められた電子",
        blurb: "箱に閉じ込めた電子の波が呼吸するように揺れます。",
        watch: [
          "波が壁の間を行ったり来たりして、形が周期的に戻ります。",
          "取れるエネルギーが飛び飛びになるのは、この閉じ込めのためです。",
        ],
        view: "field",
        pace: 60,
        readouts: [{ probe: 2, label: "エネルギー", digits: 4 }],
      },
      {
        id: "d30-gas-box",
        file: "d30-gas-box.json",
        icon: "🎱",
        title: "気体の分子を数える",
        blurb: "400 個の分子が箱の中で飛び回ります。",
        watch: [
          "1 個 1 個はでたらめに動くのに、温度と圧力は一定に落ち着きます。",
          "「たくさん集まると法則になる」——統計力学の出発点です。",
        ],
        view: "3d",
        pace: 120,
        readouts: [
          { probe: 0, label: "温度", unit: "K", digits: 1 },
          { probe: 1, label: "圧力", unit: "Pa", digits: 0 },
        ],
      },
      {
        id: "d31-diffusion-ink",
        file: "d31-diffusion-ink.json",
        icon: "🖋️",
        title: "インクが広がる",
        blurb: "1 点に落としたインクが、かき混ぜなくても広がります。",
        watch: [
          "広がりの大きさは、時間に比例して増えます(距離の 2 乗が時間に比例)。",
          "だから 2 倍の距離を広がるには 4 倍の時間がかかります。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 0, label: "広がりの大きさ", digits: 4 }],
      },
      {
        id: "d32-magnet-transition",
        file: "d32-magnet-transition.json",
        icon: "🧭",
        title: "磁石が生まれる",
        blurb: "小さな磁石の向きが、ある温度で一斉に揃います。",
        watch: [
          "「磁化」が 0 付近から離れて、大きな値へ落ち着きます。",
          "温度がある一点(キュリー温度)を下回ると起きる相転移です。",
          "鉄が磁石になれる理由そのものです。",
        ],
        view: "graph",
        pace: 60,
        readouts: [
          { probe: 0, label: "磁化(揃い具合)", digits: 4 },
          { probe: 1, label: "1 個あたりのエネルギー", digits: 4 },
        ],
      },
      {
        id: "d25-brownian",
        file: "d25-brownian.json",
        icon: "🫧",
        title: "ブラウン運動",
        blurb: "300 個の粒子が、見えない分子に叩かれて震えます。",
        watch: [
          "どの粒子もでたらめに動きますが、平均の広がり方には法則があります。",
          "アインシュタインがこの法則から分子の存在を示しました。",
        ],
        view: "3d",
        pace: 120,
        readouts: [{ probe: 2, label: "先頭の粒子の速さ", unit: "m/s", digits: 3 }],
      },
    ],
  },
  {
    id: "machine",
    icon: "🚗",
    title: "のりもの・機械",
    blurb: "走る、支える、回す。",
    experiments: [
      {
        id: "d24-car",
        file: "d24-car.json",
        icon: "🏎️",
        title: "車を走らせる",
        blurb: "4 輪のサスペンション付きの車が発進します。",
        watch: [
          "後輪が駆動して前へ進みます。",
          "発進の瞬間、車体が少し後ろへ沈みます(サスペンション)。",
          "「進んだ距離」がまっすぐ増えていきます。",
        ],
        view: "3d",
        pace: 240,
        readouts: [
          { probe: 0, label: "進んだ距離", unit: "m", digits: 2 },
          { probe: 2, label: "車の速さ", unit: "m/s", digits: 2 },
        ],
      },
      {
        id: "d18b-ice-melts",
        file: "d18b-ice-melts-into-water.json",
        icon: "💧",
        title: "氷が水に変わる",
        blurb: "融けた氷が、そのまま水の粒として現れます。",
        watch: [
          "固体が減ったぶん、液体の粒が生まれます。",
          "消えるのではなく、姿を変えるだけ——質量は保存されます。",
        ],
        view: "3d",
        pace: 240,
        readouts: [
          { probe: 0, label: "氷の高さ", unit: "m", digits: 3 },
          { probe: 1, label: "まわりの温度", format: celsius() },
        ],
      },
    ],
  },
  {
    id: "sandbox",
    icon: "✋",
    title: "じぶんで作る",
    blurb: "自分で並べて、落として、確かめる。",
    experiments: [
      {
        id: "sandbox-drop",
        icon: "⚽",
        title: "好きな物を落とす",
        blurb: "材質と高さと重力を選んで落とすだけ。",
        watch: [
          "材質を変えると、跳ね方も転がり方も変わります。",
          "重力を月にすると、ゆっくり落ちて高く跳ねます。",
          "つまみを動かして「はじめから」を押すと、すぐ試せます。",
        ],
        view: "3d",
        pace: 240,
        readouts: [{ probe: 0, label: "高さ", unit: "m", digits: 3 }],
        knobs: [
          {
            id: "shape",
            label: "かたち",
            kind: "choice",
            options: [
              { label: "⚽ ボール", value: "sphere" },
              { label: "📦 はこ", value: "box" },
            ],
            value: "sphere",
            apply: () => {},
          },
          {
            id: "material",
            label: "材質",
            kind: "choice",
            options: [
              { label: "🏀 ゴム", value: "ゴム(天然)" },
              { label: "🔩 鋼", value: "鋼(炭素鋼)" },
              { label: "🪵 木", value: "木材(松)" },
              { label: "🧊 氷", value: "氷(0°C)" },
              { label: "🫧 発泡スチロール", value: "発泡スチロール" },
            ],
            value: "ゴム(天然)",
            apply: () => {},
          },
          {
            id: "height",
            label: "高さ",
            kind: "range",
            min: 1,
            max: 30,
            step: 1,
            unit: "m",
            value: 5,
            apply: () => {},
          },
          {
            id: "gravity",
            label: "重力",
            kind: "choice",
            options: GRAVITY_OPTIONS,
            value: 9.80665,
            apply: () => {},
          },
        ],
        // 自作シーンは丸ごと組み立てる(つまみの `apply` は使わず、ここで全部読む)。
        build: (values) => {
          const shape =
            values.shape === "box"
              ? { box: { half: [0.4, 0.4, 0.4] } }
              : { sphere: { radius: 0.4 } };
          return {
            name: "sandbox-drop",
            world: { gravity: Number(values.gravity), dt: 1 / 240 },
            bodies: [
              {
                shape: { plane: { normal: [0, 1, 0], d: 0 } },
                type: "static",
                material: "コンクリート",
                name: "ground",
              },
              {
                shape,
                material: String(values.material),
                position: [0, Number(values.height), 0],
                name: "item",
              },
            ],
            probes: [{ body_pos_y: "item" }, { body_speed: "item" }],
          };
        },
      },
      {
        id: "sandbox-tower",
        icon: "🗼",
        title: "積んで、崩す",
        blurb: "好きな段数のタワーに、重い球をぶつけます。",
        watch: [
          "球が当たった瞬間、下の段から崩れていきます。",
          "段数を増やすほど、崩れ方が派手になります。",
          "球の速さを 0 にすると、崩れずに立ったままです。",
        ],
        view: "3d",
        pace: 240,
        readouts: [{ probe: 0, label: "1 段目の速さ", unit: "m/s", digits: 3 }],
        knobs: [
          {
            id: "floors",
            label: "段数",
            kind: "range",
            min: 2,
            max: 10,
            step: 1,
            unit: "段",
            value: 5,
            apply: () => {},
          },
          {
            id: "speed",
            label: "ぶつける球の速さ",
            kind: "range",
            min: 0,
            max: 30,
            step: 1,
            unit: "m/s",
            value: 12,
            apply: () => {},
          },
          {
            id: "material",
            label: "積み木の材質",
            kind: "choice",
            options: [
              { label: "🪵 木", value: "木材(松)" },
              { label: "🔩 鋼", value: "鋼(炭素鋼)" },
              { label: "🧊 氷", value: "氷(0°C)" },
            ],
            value: "木材(松)",
            apply: () => {},
          },
        ],
        build: (values) => {
          const floors = Math.max(2, Math.trunc(Number(values.floors)));
          const material = String(values.material);
          const bodies: SceneJson["bodies"] = [
            {
              shape: { plane: { normal: [0, 1, 0], d: 0 } },
              type: "static",
              material: "コンクリート",
              name: "ground",
            },
          ];
          for (let i = 0; i < floors; i += 1) {
            bodies.push({
              shape: { box: { half: [0.4, 0.4, 0.4] } },
              material,
              position: [0, 0.4 + i * 0.81, 0],
              name: `box${i + 1}`,
            });
          }
          bodies.push({
            shape: { sphere: { radius: 0.35 } },
            material: "鋼(炭素鋼)",
            position: [-6, 0.4, 0],
            linear_velocity: [Number(values.speed), 2, 0],
            name: "wrecker",
          });
          return {
            name: "sandbox-tower",
            world: { gravity: 9.80665, dt: 1 / 240 },
            bodies,
            probes: [
              { body_speed: "box1" },
              { body_speed: `box${floors}` },
              { body_pos_x: "wrecker" },
            ],
          };
        },
      },
      {
        id: "sandbox-pendulum",
        icon: "🪀",
        title: "ふりこを作る",
        blurb: "ひもの長さと重力を選んで、往復の速さを比べます。",
        watch: [
          "ひもが長いほど、ゆっくり往復します。",
          "重力が小さいほど、ゆっくり往復します。",
          "おもりの重さを変えても、往復の時間は変わりません。",
        ],
        view: "graph",
        pace: 240,
        readouts: [{ probe: 0, label: "おもりの横位置", unit: "m", digits: 3 }],
        knobs: [
          {
            id: "length",
            label: "ひもの長さ",
            kind: "range",
            min: 0.25,
            max: 6,
            step: 0.25,
            unit: "m",
            value: 2,
            apply: () => {},
          },
          {
            id: "angle",
            label: "はじめの振れ角",
            kind: "range",
            min: 5,
            max: 80,
            step: 5,
            unit: "°",
            value: 30,
            apply: () => {},
          },
          {
            id: "gravity",
            label: "重力",
            kind: "choice",
            options: GRAVITY_OPTIONS,
            value: 9.80665,
            apply: () => {},
          },
        ],
        build: (values) => {
          const length = Number(values.length);
          const angle = (Number(values.angle) * Math.PI) / 180;
          return {
            name: "sandbox-pendulum",
            world: { gravity: Number(values.gravity), dt: 1 / 240 },
            bodies: [
              {
                name: "pivot",
                shape: { sphere: { radius: 0.06 } },
                material: "鋼(炭素鋼)",
                type: "static",
                position: [0, 0, 0],
              },
              {
                shape: { sphere: { radius: 0.15 } },
                material: "鋼(炭素鋼)",
                mass_override: 1,
                position: [
                  length * Math.sin(angle),
                  -length * Math.cos(angle),
                  0,
                ],
                name: "bob",
              },
            ],
            joints: [
              {
                distance: {
                  body_a: "bob",
                  anchor_a: [0, 0, 0],
                  anchor_b: [0, 0, 0],
                  length,
                },
              },
            ],
            probes: [{ body_pos_x: "bob" }, { body_pos_y: "bob" }],
          };
        },
      },
    ],
  },
];

/** ID から実験を引く(カテゴリをまたいで探す)。 */
export function findExperiment(id: string): Experiment | undefined {
  for (const category of GUIDED_CATEGORIES) {
    const found = category.experiments.find((e) => e.id === id);
    if (found) return found;
  }
  return undefined;
}

/** ID からカテゴリを引く。 */
export function findCategory(id: string): Category | undefined {
  return GUIDED_CATEGORIES.find((c) => c.id === id);
}
