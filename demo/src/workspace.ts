// ワークスペース——**ひとつの画面のまま、粒度が連続的に変わる**操作面。
//
// **考え方**: 人は同じ対象を「遠くから眺める」ときと「一点を詰める」ときで、
// 必要な情報量が違う。しかしそれは *別のアプリに切り替わる* ことではない——
// 同じ景色に対する **見方の濃淡** でしかない。だから UI もモードで割らず、
// ひとつの画面のまま濃淡が連続して動くように作る。
//
// 濃淡は 2 つの独立した軸で表す。人の注意もこの 2 軸で動くからである。
//
//   1. **大局の粒度**(`--detail`, 0.0〜3.0 の連続値)
//      画面全体がどれだけ踏み込んだ姿になるか。みる → さわる → しらべる → つくる。
//      値は連続で、パネルは幅・高さが補間されながら現れる(スイッチではない)。
//
//   2. **局所の粒度**(カードごとの `data-expanded`)
//      「全体は眺めモードのまま、この一点だけ詰めたい」を許す。大局の粒度を
//      上げずに 1 枚のカードだけ開ける。逆に大局を上げれば、開いていなかった
//      カードも自然に開く(`data-reveal` の閾値)。
//
// この 2 軸があるので「2 モード」は要らない。眺めている途中で 1 つの値だけ
// 詰めることも、全体を作業机に変えることも、同じ画面の上で連続的に起きる。
//
// **目的地(POI)への 3 手**: どこにいても `⌘K`(または左端のパンくず)で
// パレットが開く → 打つか選ぶで絞る → 選んだ瞬間に走り出す。段階を踏ませる
// ウィザードではなく、**複雑な 46 の実験を 1 つの入口で扱えるようにする**装置。
//
// このモジュールは物理側の内部状態には触れない。`WorkspaceApi`(`main.ts` が
// 埋める小さな窓口)だけを通す。

import {
  GUIDED_CATEGORIES as CATEGORIES,
  findExperiment,
  type Category,
  type Experiment,
  type Knob,
  type SceneJson,
} from "./catalog";

/** ワークスペースが物理側へ求めることの全部。 */
export type WorkspaceApi = {
  loadSceneJson: (json: string) => void;
  play: () => void;
  pause: () => void;
  isPlaying: () => boolean;
  /**
   * 走らせておらず、直接編集できる状態(Edit)か。`playButton`が
   * Editから初めて走らせる瞬間だけ追従カメラを起こす判定に使う
   * (一時停止からの再開ではmode==="play"のままなので偽になる、
   * `playButton`のdoc参照)。
   */
  isEditing: () => boolean;
  /**
   * 1 秒あたりに進める step 数。`null` で従来の「時間倍率」方式へ戻す。
   * シーンごとに dt が 16 桁も違うため、倍率では現象ごとの速さを指定できない
   * (`catalog.ts` 冒頭の doc 参照)。
   */
  setPace: (stepsPerSecond: number | null) => void;
  probeCount: () => number;
  probeValue: (index: number) => number;
  time: () => number;
  /** 1 step の刻み [s]。時間の表示単位を決めるのに使う。 */
  stepSeconds: () => number;
  /**
   * 走らせずに、直接編集できる状態にする(いわゆる Edit)。深い粒度で
   * 実験を読み込んだときは、**勝手に走り出さない**方が正しい——作る人は
   * 置いてから動かす。
   */
  stopForEditing: () => void;
  /**
   * 「この物体は、この点から吊るされている」を線で描く指示。
   * シーンJSONの距離拘束から作る(物理には関与しない、見るための線)。
   */
  setTethers: (tethers: { bodyIndex: number; anchor: [number, number, number] }[]) => void;
  /** 追従カメラの入り切り。 */
  followCamera: (enabled: boolean) => void;
  /**
   * いま置いてある物ぜんぶが入る画角へ、**一回だけ**合わせ直す(向きは保つ)。
   *
   * `followCamera(true)` は毎フレーム追いかけ続ける追従カメラで、対象が
   * 遠いほど原点も画角に収めようとして寄りすぎず「見かけの大きさの下限」で
   * 折り合いを付ける(逃げていく物を見失わないための妥協)。「全体へ戻る」に
   * それを流用すると、止まっている1個の物が豆粒になってしまう(利用者役の
   * 報告)。ここは名前どおり「いま入れる」ことだけを約束する一回きりの処理。
   */
  frameOnContent: () => void;
  /**
   * グラフの凡例・目盛りに出す表示名と単位(プローブ番号 → 人間の言葉)。
   * 単位が分かる系列だけ `units` に入れる(無ければ数だけを書く)。
   * `convert` は**表の数字と同じ量**をグラフにも描くための変換
   * (ケルビン → ℃ など、`Readout.graph` のdoc参照)。
   */
  setProbeLabels: (
    labels: Record<number, string> | null,
    units?: Record<number, string> | null,
    convert?: Record<number, (value: number) => number> | null,
  ) => void;
  /**
   * **いまの場面をそのまま文書にする**(利用者役④の観察: 自分で組み立てた
   * 場面を保存する手段がどこにも無く、再読み込みで跡形もなく消えた)。
   * 走行中の状態も含めて書き出せるので、これを読み直せば続きから開ける。
   */
  exportSceneJson: () => string | null;
  /**
   * エディタ側(新規シーン・シーンギャラリー等)が場面を差し替えたときに
   * 呼ばれる。ワークスペースが握っている「いま見ている実験」が実物と
   * 食い違ったままになるのを防ぐためのもの——実際、新規シーンを作っても
   * パンくずと説明カードが前の実験のままだった。
   */
  onSceneReplaced: (callback: () => void) => void;
  /**
   * いま**いちばん速く動いている物の速さ** [m/s]。動く物が 1 つも無ければ 0。
   * 「もう何も動いていない」を画面から言うために使う(物理には触らない、
   * 読むだけの値)。
   */
  maxSpeed: () => number;
  /** 舞台に描くものが無いか(案内を出しているのと同じ判断)。 */
  stageIsEmpty: () => boolean;
  /**
   * 選んだ物の**材質を差し替える**。物理側は場面を組み直して反映するので、
   * 走行中はできない(`false` を返す)。
   */
  setBodyMaterial: (index: number, materialName: string) => boolean;
  /**
   * 選んだ物を**その場所へ置き直す**。組み立てるときの位置決めなので、
   * 走行中でも効く(Gizmo のドラッグと同じ扱い)。
   */
  setBodyPosition: (index: number, x: number, y: number, z: number) => boolean;
  /**
   * 選んだ物の**向き**を、度で決める(x, y, z 軸まわり)。
   *
   * 向きは輪(ギズモ)をドラッグするしか手が無く、掴む場所がわずかに違うだけで
   * どの軸が回るか変わってしまい、狙った角度の坂を作るまで何度もやり直しに
   * なっていた(利用者役④の観察)。置き場所と同じように、数値でも決められる
   * ようにする。
   */
  setBodyRotation: (index: number, degX: number, degY: number, degZ: number) => boolean;
  /**
   * 選んだ物の高さと速さを、グラフに記録し始める。
   *
   * 記録が付くのは最初に置いた物だけで、2 つ目以降を比べたくても足す手段が
   * どこにも無かった(利用者役④の観察)。
   */
  addBodyProbes: (index: number) => boolean;
  /** その物が、もうグラフに記録されているか。 */
  hasBodyProbes: (index: number) => boolean;
  /**
   * 選んだ物を場面から消す。Hierarchy右クリックの「削除」・Deleteキーと
   * 同じ経路(`false`が返るのは、床(index 0)を指定したか、既に無い場合)。
   *
   * 押し間違いで戻せなくなる操作(この後の「動かしたのを戻す(Undo)」は
   * 位置/向き/大きさしか戻せず、削除は対象外)なので、**呼ぶ前に確認を
   * 挟むのは呼び出し側の責任**——この関数自体は無条件に消す。
   */
  removeBody: (index: number) => boolean;
  /** 選べる材質の名前(スポーンパレットと同じ並び)。 */
  materialNames: () => string[];
  /**
   * その材質の摩擦係数(Rust側の材質DBが持つ実際の値、`material_properties_f64`
   * の2番目)。**課題B**: 「箱の材質」ボタンは鋼・ゴム・木・氷・発泡スチロール・
   * アルミの6択だが、どれがよく滑るかが画面のどこにも書かれておらず、利用者は
   * 「氷が滑りやすいだろう」と勘で選ぶしかなかった(利用者役の報告)。数値を
   * でっち上げず、アプリが実際に使っているこの値をそのままボタンへ添える。
   * 未知の材質名には`NaN`を返す(呼び出し側は表示を省く)。
   */
  materialFriction: (name: string) => number;
  /** 選択中の剛体(無ければ -1)。 */
  selectedBody: () => number;
  selectBody: (index: number) => void;
  bodyCount: () => number;
  /** 選択中の剛体の「いまの姿」。UI 側は読むだけ。 */
  bodyReadout: (index: number) => {
    label: string;
    shape: string;
    material: string;
    mass: number;
    position: [number, number, number];
    /** 向き(x, y, z 軸まわりの度)。 */
    rotation: [number, number, number];
    speed: number;
  } | null;
};

export type WorkspaceApiRef = { current: WorkspaceApi | null };

const DETAIL_KEY = "simulator.ui.detail";
const LAST_EXPERIMENT_KEY = "simulator.ui.last-experiment";
/// 自分で組み立てた場面の保管場所(`SavedScene[]`)。
const SAVED_SCENES_KEY = "simulator.scenes.saved";
/// 最後に開いていた自作の場面の名前。次に開いたときここへ戻る。
const LAST_OWN_SCENE_KEY = "simulator.scenes.last";

/** 自分で組み立てて名前を付けた場面。 */
type SavedScene = { name: string; savedAt: string; json: string };

function readSavedScenes(): SavedScene[] {
  try {
    const raw = localStorage.getItem(SAVED_SCENES_KEY);
    const parsed = raw ? (JSON.parse(raw) as SavedScene[]) : [];
    return Array.isArray(parsed) ? parsed : [];
  } catch {
    return [];
  }
}

function writeSavedScenes(scenes: SavedScene[]): string | null {
  try {
    localStorage.setItem(SAVED_SCENES_KEY, JSON.stringify(scenes));
    return null;
  } catch (err) {
    // 置き場が一杯なのか、そもそも使えないのかで打つ手が違う。どちらでも
    // **ファイルに落とす道**が残っていることを言う(黙って失敗しない)。
    return String(err).includes("Quota")
      ? "この端末の保管場所が一杯です。いらない場面を消すか、ファイルに書き出してください。"
      : "この端末には保存できませんでした。ファイルに書き出してください。";
  }
}

/** 大局の粒度の目盛り。連続値だが、名前が付く位置がある。 */
const GRAIN_STOPS = [
  // hint は「その粒度で**画面に何が出るか**」。以前は「つまみを動かして試す」
  // のように行為だけを書いていたので、この帯自体が実験のつまみだと読まれた
  // ——2 人続けて取り違えた(利用者役②の観察)。画面の話だと分かる書き方に
  // 統一し、現象は変わらないことを添える。
  { at: 0, key: "watch", label: "みる", hint: "現象だけを大きく(道具は隠す)" },
  { at: 1, key: "touch", label: "さわる", hint: "＋ 条件を変えるつまみ" },
  { at: 2, key: "study", label: "しらべる", hint: "＋ グラフ・一覧・数値" },
  { at: 3, key: "build", label: "つくる", hint: "＋ 自分で組み立てる道具ぜんぶ" },
];

// パネルが現れ始める粒度。CSS 側の補間と同じ値をここでも使う(表示の
// 有無を JS が決め、幅や高さの補間は CSS が受け持つ)。
// グラフが「開き始める」粒度と、「読める大きさになる」粒度は違う。自動で
// 開くときは後者を使う——閾値ちょうどでは高さ 0 で、出したつもりが出ていない。
const ANALYSIS_READABLE = 1.3;

/**
 * これより深い粒度では、実験を読み込んでも自動で走らせない。
 * 「みる/さわる」は現象が動いていることが目的だが、「つくる」で走り出すと
 * 置く前に転がっていく——同じ操作でも、粒度によって正しい既定が違う。
 */
const AUTORUN_BELOW = 2.5;

/**
 * **場から退いた剛体の目印**。
 *
 * 消えた物(融け切った氷など)は、内部では `y = -1e9 m` の遠方へ退避させられる。
 * その値を数字として出すと「-1,000,000,000.000 m」になり、壊れた画面にしか
 * 見えない(利用者役②の観察)。
 *
 * **桁の大きさで判定しない**。「1e8 を超えたら退避」と見ていたときは、太陽から
 * 1.5e11 m を回っている惑星の距離まで「もう在りません」と書いてしまい、目の前を
 * 回っている衛星を指して「消えました」と言う画面になっていた(利用者役①の
 * 観察)。退避先はただ一点(y = -1e9 m)なので、そこにいるときだけ言う。
 * 退避した物は速度も 0 にされるので、そこから動いて紛れることもない。
 */
const RETIRED_BODY_Y = -1e9;
/** 退避先とみなす幅 [m]。 */
const RETIRED_BODY_TOLERANCE = 1;

const REVEAL = {
  analysis: 1.2, // グラフ
  outline: 1.6, // シーンの一覧
  inspector: 2.0,
  toolbar: 2.4,
  console: 2.4,
  project: 2.8,
};

const sceneFiles = import.meta.glob("../../scenes/*.json", {
  query: "?raw",
  import: "default",
  eager: true,
}) as Record<string, string>;

function sceneFileContent(file: string): string | null {
  const entry = Object.entries(sceneFiles).find(([path]) =>
    path.endsWith(`/${file}`),
  );
  return entry ? entry[1] : null;
}

/**
 * 経過時間を、その桁に合った単位で書く。
 *
 * シーンの時間スケールは 1e-12 秒(気体分子)から 1e9 秒(公転)まで 21 桁に
 * わたる。秒で固定表示すると、気体の箱は永遠に「0.00 秒」、太陽系は
 * 「31554896.93 秒」になり、どちらも**動いていないのと区別が付かない**。
 */
/**
 * 数値をパネルに書く形にする。
 *
 * `toFixed` だけで書いていたので、インクの広がり(1.5e-22 → 1.9e-17)や天体の
 * 距離のように**桁が離れた量**が、動いているのに「0.0000」のまま止まって見えた
 * (利用者役③の観察)。書いた桁に何も現れない値と、桁が多すぎて読めない値は、
 * 指数で書く。
 */
export function readoutNumber(value: number, digits: number): string {
  if (!Number.isFinite(value)) return "—";
  const abs = Math.abs(value);
  if (abs !== 0 && (abs < 0.5 * 10 ** -digits || abs >= 1e7)) {
    return value.toExponential(2);
  }
  return value.toFixed(digits);
}

export function formatDuration(seconds: number, scale = 1): string {
  const t = Math.abs(seconds);
  // **単位はシーンの時間スケールで決める**。値そのもので切り替えると、同じ
  // 実験の途中で「958.33 ミリ秒 → 1.02 秒」と桁も単位も飛んで読みにくい
  // (利用者役の観察)。人の尺度で進むシーン(1 step が 1e-4 秒より粗い)は
  // 常に秒より上の単位で書き、分子や公転のように桁が離れたシーンだけ
  // それぞれの単位系に入る。
  // **0 のときも、その場面の単位で書く**。「0 秒」と決め打ちしていたので、
  // 分子の場面(ピコ秒で進む)を開いた瞬間だけ、右の「経過した時間」が
  // 「0 秒」、下の時間の帯が「0.00 ピコ秒」と食い違って見えた。値からは
  // 単位を選べないので、そのときは**時間の刻み**から選ぶ。
  const pick = t === 0 ? Math.abs(scale) : t;
  if (scale >= 1e-4) {
    if (pick < 60) return `${seconds.toFixed(2)} 秒`;
    if (pick < 3600) return `${(seconds / 60).toFixed(2)} 分`;
    if (pick < 86400) return `${(seconds / 3600).toFixed(2)} 時間`;
    if (pick < 3.155e7) return `${(seconds / 86400).toFixed(2)} 日`;
    return `${(seconds / 3.155e7).toFixed(2)} 年`;
  }
  if (pick < 1e-9) return `${(seconds * 1e12).toFixed(2)} ピコ秒`;
  if (pick < 1e-6) return `${(seconds * 1e9).toFixed(2)} ナノ秒`;
  if (pick < 1e-3) return `${(seconds * 1e6).toFixed(2)} マイクロ秒`;
  if (pick < 1) return `${(seconds * 1e3).toFixed(2)} ミリ秒`;
  if (pick < 60) return `${seconds.toFixed(2)} 秒`;
  if (pick < 3600) return `${(seconds / 60).toFixed(2)} 分`;
  if (pick < 86400) return `${(seconds / 3600).toFixed(2)} 時間`;
  if (pick < 3.155e7) return `${(seconds / 86400).toFixed(2)} 日`;
  return `${(seconds / 3.155e7).toFixed(2)} 年`;
}

/** `body_shape_label_at`(wasm)が返す先頭の型名を取り出す。
 *  `Sphere(0.4000)`のように`(`区切りの型と、`Capsule`のように専用の整形が
 *  無く`{other:?}`のRust Debugへ落ちて`Capsule { radius: 0.2, ... }`と
 *  `{`区切りになる型が混在する(`body_shape_label_at_impl`のdoc参照、Rust側は
 *  触らない縮約)。`split("(")[0]`だけだと後者で丸ごと1語になり
 *  `names`に無いキーとして弾かれ、カードにRustの生のDebug文字列が
 *  そのまま出てしまっていた(利用者役の報告で確認: カプセルだけ
 *  「選んだもの」札にも`Capsule { radius: 0.2, half_height: 0.35 }`が
 *  出ていた)。先頭の英字だけを拾う方が両方に効く。 */
function shapeHead(raw: string): string {
  return (raw.match(/^[A-Za-z_]+/)?.[0] ?? "").toLowerCase();
}

/** `Plane(normal=(0,1,0), d=0)` のような内部表記を、ひと目で分かる名前にする。 */
function friendlyShape(raw: string): string {
  const head = shapeHead(raw);
  const names: Record<string, string> = {
    plane: "ゆか(平面)",
    sphere: "球",
    box: "箱",
    capsule: "カプセル",
    compound: "組み合わせ",
    convexmesh: "凸包メッシュ",
    convex_mesh: "凸包メッシュ",
  };
  const name = names[head];
  if (!name) return raw;

  // **人が言う「大きさ」で書く**。内部表記は箱なら半分の長さ、球なら半径を
  // 出すので、`Box(0.4000, …)` の箱が 4019 kg になり「表示と重さが合わない、
  // バグでは」と読まれた(利用者役④の観察)。箱は一辺、球は直径にして、
  // 数と重さが素直に噛み合うようにする。
  const numbers = [...raw.matchAll(/-?\d+(?:\.\d+)?/g)].map((m) => Number(m[0]));
  const meters = (v: number) => `${v.toFixed(2)} m`;
  if (head === "box" && numbers.length >= 3) {
    // 単位は最後に一度だけ(「0.80 m × 0.80 m × 0.80 m」はくどい)。
    return `${name} ${numbers
      .slice(0, 3)
      .map((half) => (half * 2).toFixed(2))
      .join(" × ")} m`;
  }
  if (head === "sphere" && numbers.length >= 1) {
    return `${name}(直径 ${meters(numbers[0] * 2)})`;
  }
  if (head === "capsule" && numbers.length >= 2) {
    return `${name}(太さ ${meters(numbers[0] * 2)}・長さ ${meters(numbers[1] * 2)})`;
  }
  return name;
}

// **Inspector の「かたち (Shape)」欄で、生の値に何の値かを書き足す**。
// wasm(`body_shape_label_at`)は球なら半径、箱なら半辺(一辺の半分)の長さを
// 生のまま返す一方、「選んだもの」札は`friendlyShape`で人の言葉(直径・一辺)
// に直して見せている——**どちらも自分の役割としては正しい**が、何の値かが
// 書いていないため、球を1つ置くと札は「直径 0.80 m」、Inspectorは
// 「Sphere(0.4000)」になり、同じ球なのに数字が噛み合わないように見えた
// (利用者役の報告、`friendlyShape`のdoc参照)。
//
// **物理側(wasm)が返す値は書き換えない**——丸めも変換もせず、生の数字
// 文字列をそのまま埋め込む。変えるのは「半径」「半辺」等のラベルと単位の
// 表記だけ。Inspectorは中を知っている人向けの生の値を消さずに見せる場、
// 「選んだもの」札は中を知らない人向けの人の言葉、という役割分担のまま
// 両方を正しくする。
export function annotateInspectorShape(raw: string): string {
  const head = shapeHead(raw);
  // 数を文字列のまま取り出す(`Number(...)`を経由して`toFixed`し直すと、
  // wasmが返した桁数と表示上ずれかねない——ここは生の値を一切変えない)。
  const nums = [...raw.matchAll(/-?\d+(?:\.\d+)?/g)].map((m) => m[0]);
  if (head === "sphere" && nums.length >= 1) {
    return `Sphere(半径 ${nums[0]} m)`;
  }
  if (head === "box" && nums.length >= 3) {
    return `Box(半辺 x=${nums[0]}, y=${nums[1]}, z=${nums[2]} m)`;
  }
  if (head === "capsule" && nums.length >= 2) {
    return `Capsule(半径 ${nums[0]} m, 半分の高さ ${nums[1]} m)`;
  }
  return raw;
}

function el<T extends HTMLElement>(id: string): T {
  return document.getElementById(id) as T;
}

function readStoredDetail(): number {
  try {
    const raw = localStorage.getItem(DETAIL_KEY);
    // 初見は「さわる」から。現象が動いているのが見え、かつ**自分で変えられる**
    // ことがつまみとして目に入る位置。ここから浅くも深くも回せる。
    if (raw === null) return 1;
    const value = Number.parseFloat(raw);
    return Number.isFinite(value) ? Math.min(3, Math.max(0, value)) : 1;
  } catch {
    return 1;
  }
}

export function setUpWorkspace(apiRef: WorkspaceApiRef): void {
  const app = el<HTMLDivElement>("app");
  const dial = el<HTMLInputElement>("detail-dial");
  const dialStops = el<HTMLDivElement>("detail-stops");
  const dialHint = el<HTMLDivElement>("detail-hint");
  const crumbs = el<HTMLDivElement>("crumbs");
  const contextBody = el<HTMLDivElement>("context-body");
  const palette = el<HTMLDivElement>("palette");
  const paletteInput = el<HTMLInputElement>("palette-input");
  const paletteResults = el<HTMLDivElement>("palette-results");
  const paletteFilters = el<HTMLDivElement>("palette-filters");
  const playButton = el<HTMLButtonElement>("btn-run");
  const restartButton = el<HTMLButtonElement>("btn-restart");
  const clock = el<HTMLDivElement>("run-clock");
  const rate = el<HTMLDivElement>("run-rate");
  const actualRate = el<HTMLDivElement>("run-actual-rate");
  const speedGroup = el<HTMLDivElement>("run-speed");
  const commandbar = el<HTMLElement>("commandbar");

  // ---- 状態 -----------------------------------------------------------------
  let detail = readStoredDetail();
  /**
   * **人が自分で選んだ濃さ**。
   *
   * 「みる」にしても、実験を選び直すたびに「さわる」へ戻ってしまっていた
   * (利用者役①の観察)。舞台に何も映らない実験でグラフが読める濃さまで
   * 上げたあと、その値がそのまま**人の選択として居座って**いたため。上げるの
   * は一時的な足し算にして、次に実験を選んだらここへ戻す。
   */
  let chosenDetail = detail;
  let current: Experiment | null = null;
  let knobValues: Record<string, string | number> = {};
  let speedMultiplier = 1;
  /**
   * **実際に進んでいる速さの実測**(`run-actual-rate` のdoc参照)。
   *
   * 「実時間に対して、シミュレーション内の時間が何倍で進んでいるか」を、
   * 直前のtickとの差分(実時間の差・`api.time()`の差)から毎フレーム測る。
   * 生の瞬間値はフレームごとにばらつくので、指数移動平均で均す
   * (`main.ts`の`updateEffectiveTimeScale`と同じ考え方——ただしこちらは
   * ワールドの内部実装(step数)に触れず、`WorkspaceApi`越しに見える
   * 「実時間・シミュレーション時間」だけから測る)。
   */
  let actualRateSmoothed: number | null = null;
  let lastActualRateWallMs: number | null = null;
  let lastActualRateSimSeconds: number | null = null;
  let filterCategory: string | null = null;
  let paletteIndex = 0;
  let paletteMatches: PaletteEntry[] = [];
  let pendingStart = false;
  /** 物理側の起動待ちで、開けずにいる自分の場面。 */
  let pendingOwnScene: SavedScene | null = null;
  /**
   * **毎フレームの選択変更検出**(`tick()` 内の `selected !== lastSelection`)が
   * 前回どこを見ていたかを覚える場所。
   *
   * **不変条件**: ここより下の、選択を直接触るすべての箇所
   * (`api.selectBody(...)` を直接呼ぶ場所)は、呼んだ直後に必ず
   * `lastSelection` もその値へ合わせる。これを怠ると、`tick()` が拾う前の
   * 「本物の選択変更」を見失う。
   *
   * **これが実際に壊れていた**(Windows CI, `workspace.spec.ts`「大きさの
   * 表示が、札と Inspector で食い違わない」): 「新規シーン」は毎回まっさら
   * な世界を作り直すため、最初に置いた物は毎回同じ body index(床の次、
   * =1)になる。「新規シーン→球(index 1 を選択)→新規シーン→箱
   * (これも index 1 を選択)」という手順では、2 回目の「新規シーン」で
   * `onSceneReplaced` が `api.selectBody(-1)` を直接呼んで `renderContext()`
   * まで済ませるが、`lastSelection` は前の球の値(1)のまま残っていた。
   * その後、箱を置いて選択が index 1 に変わっても、`tick()` が一度も
   * 割り込まないまま(遅い機械ほど起きやすい——CPU を20〜40倍絞った実測で
   * 再現、詳細はコミットの本文参照)次の tick に来ると
   * `selected(1) !== lastSelection(1)` が偽になり、選んだものの札が二度と
   * 組み直されなかった。直すのは「選択を直接触ったら、その場で
   * `lastSelection` も合わせる」——そうすれば `tick()` が何回スキップされ
   * ても、次に見るときの比較は必ず正しい値どうしになる。
   */
  let lastSelection = -1;
  /** カードごとの局所的な開閉。`undefined` = 大局の粒度に従う。 */
  const cardOverrides = new Map<string, boolean>();
  /**
   * **いま開いているのが「じぶんの場面」なら、その名前**(未保存なら "")。
   * `current === null`(カタログの実験を選んでいない)ときは、自分で組み立て
   * ている場面を見ている——利用者役④が新規シーンを作ってもパンくずと説明が
   * 前の実験のままで、「自分がどこにいるのか分からない」と書いた状態を、
   * この一本の状態で言い分ける。
   */
  let ownSceneName = "";
  /**
   * 保存の名前欄に**打ちかけている文字**。カードは選択が変わるたびに組み直す
   * ので、これを持っていないと打っている途中で欄が空に戻る(CI で実際に、
   * 名前を打った直後の保存が自動命名になった)。
   */
  let sceneNameDraft = "";
  /**
   * **もう何も動いていない**と分かった時刻 [s](まだなら null)。
   *
   * 落ちて跳ねて止まった後も時計だけが回り続け、「着地しました」に当たる
   * 合図が画面のどこにも無かった。数値だけを見ていると、実際より何十倍も
   * 長い時間を「落ちるのにかかった時間」と読んでしまう——グラフを出さない
   * 限り気付けない罠だった(利用者役②の一番の不満)。
   *
   * 物理には一切触らない。**見ている値からそう読めた**というだけの表示で、
   * 判定はいちばん速い物の速さがしきい値を下回り続けたかどうかで行う。
   */
  let settledAt: number | null = null;
  /** いちばん速い物の速さが、この値を下回っていれば「止まっている」と見なす [m/s]。 */
  const SETTLED_SPEED = 0.05;
  /** 一度は動いたか(最初から動かない場面で「止まりました」と言わないため)。 */
  let everMoved = false;
  /** 止まっているように見えた連続フレーム数(取りこぼしと一瞬の静止を分ける)。 */
  let stillFrames = 0;
  /** 直前のフレームで舞台が空だったか(「どこを見るか」の追いつき用)。 */
  let lastStageEmpty: boolean | null = null;
  /** 直前に出した「ここを見る」の一行(同じなら書き直さない)。 */
  let lastWhereText = "";
  /** 「舞台が空」と決めるまでに、空のまま待つフレーム数(60fps でおよそ半秒)。 */
  const STAGE_EMPTY_FRAMES = 30;
  let stageEmptyFrames = 0;
  /**
   * **舞台に形のある物が無い実験で、グラフの段だけを強制的に開いているか**。
   *
   * 課題A(利用者役の報告): 「電気の工作台」は 3D に映る物がひとつも無く、
   * 現象そのものが数値とグラフでしかない。以前はこれを「グラフが読める
   * 濃さまでダイヤルごと上げる」(`ANALYSIS_READABLE` へ `applyDetail`)で
   * 解決していたが、それだと「変えてみる」(つまみ、reveal 0.8)まで巻き
   * 込まれて開いてしまい、「みる」を選んだ人の画面が道具だらけになった
   * (方針:「みる」で出すのは現象そのもの(グラフ)だけ)。
   *
   * そこで大局の粒度(`detail`/`chosenDetail`/ダイヤルの位置)には一切触れず、
   * グラフの段(`--row-analysis`)の開閉だけを、この旗ひとつで上書きする。
   * 「みる」は「みる」のまま、現象(グラフ)だけが追いついて出る。
   */
  let forceAnalysisOpen = false;

  // ---- 大局の粒度 -------------------------------------------------------------
  /**
   * `from` を超えたところから `span` かけて 0 → `to` まで伸びる寸法 [px]。
   * 閾値で「パッと出る」のではなく、粒度に連れて**育つ**ようにするための補間。
   */
  function grow(value: number, from: number, span: number, to: number): number {
    const t = Math.min(1, Math.max(0, (value - from) / span));
    return Math.round(t * to);
  }

  /**
   * 画面の濃さを変える。
   *
   * `byPerson` が真なら、**人が自分で選んだ濃さ**(`chosenDetail`)も更新する。
   * 実験の都合で一時的に上げるとき(舞台に何も映らない場面)は偽で呼ぶ——
   * そうしないと、一度そういう実験を開いただけで「みる」に戻れなくなる。
   */
  function applyDetail(next: number, persist = true, byPerson = true): void {
    detail = Math.min(3, Math.max(0, next));
    if (byPerson) chosenDetail = detail;
    app.style.setProperty("--detail", detail.toFixed(3));
    app.dataset.grain = nearestStop(detail).key;

    // **寸法はすべてこの 1 箇所で、粒度ひとつから決まる**。
    // 文脈の柱だけは最初から在る——「いま何を見ているか」は、どんな粒度でも
    // 必要な情報だから(消えると画面が読めなくなる)。
    const width = window.innerWidth;
    const context = Math.min(
      380,
      Math.max(272, Math.round(272 + detail * 36)),
    );
    // 一覧の柱は「使える幅」か 0 か。数十 px で顔を出すと、文字が 1 文字ずつ
    // 折り返された読めない帯になる(利用者役の観察で実際に出た)。
    // 段の高さと同じ考え方——出すなら用を成す大きさで出す。
    const outline =
      detail >= REVEAL.outline ? Math.max(168, grow(detail, REVEAL.outline, 0.6, 232)) : 0;
    const toolbar = grow(detail, REVEAL.toolbar, 0.4, 64);
    // グラフも一覧と同じで「読める大きさか 0 か」。グラフを出したいだけの人が、
    // 一覧(1.6〜)まで引き連れてこないよう、読める点を一覧より手前に置く。
    // `forceAnalysisOpen` が立っているとき(課題A: 舞台に形のある物が無い
    // 実験)は、粒度がどこにあっても最低限(150px)を渡す——他の段(つまみ・
    // 一覧・ツールバー)の閾値には一切触れない。
    let analysis =
      detail >= REVEAL.analysis || forceAnalysisOpen
        ? Math.max(150, grow(detail, REVEAL.analysis, 0.8, 210))
        : 0;
    // 時間の帯も「用を成す高さか 0 か」。中身(スクラバ)が入らない高さで
    // 顔を出すと、掴めない帯が残るだけになる。
    let timeline = detail >= 0.8 ? Math.max(56, grow(detail, 0.8, 0.6, 78)) : 0;
    let consoleRow = grow(detail, REVEAL.console, 0.6, 118);
    // Project ドロワーは「開いている」と宣言されたら、粒度に関わらず中身が
    // 入る高さを与える(タブだけ出て中身が画面外、という旧不具合の再発防止)。
    const projectBase = grow(detail, REVEAL.project, 0.2, 30);
    const project =
      app.dataset.drawer === "open" ? Math.max(projectBase, 240) : projectBase;

    // **舞台を潰さない / 出したものを見えなくしない**。
    //
    // 深い粒度では下の段(時間・グラフ・ログ・素材)が積み上がり、画面の高さに
    // よっては現象を映す場所が消える。かといって、いちど現れた段を高さ 0 まで
    // 潰すと**そこにあるのに触れない**——「Errors タブが押せない」「グラフを
    // 出したのに見えない」という、いちばん説明のつかない壊れ方になる。
    //
    // そこで 2 段階で配る:
    //   ① 現れている段には、まず**用を成す最低限**(見出しやタブが押せる高さ)を
    //      必ず渡す。舞台の最低の高さより、こちらを優先する。
    //   ② 残りを グラフ → 時間 → ログ の順に、望みの高さまで足していく。
    // 素材ドロワーを開いている間だけは、注意がそこにあるので舞台を少し譲る。
    const drawerOpen = project > projectBase;
    // グラフは 150px を切ると、線を描く場所が 60px ほどしか残らず「出したのに
    // 読めない」状態になる(実測)。用を成す最低限をここで決める。
    // 時間の段は、帯そのものに指で掴める高さ(22px)を与えたぶんだけ厚くする
    // ——最低限が薄いままだと、帯の下半分が段からはみ出して押せなくなる。
    const floor = { analysis: 150, timeline: 62, console: 34 };
    const wants = { analysis, timeline, console: consoleRow };
    const reserved =
      (wants.analysis > 0 ? floor.analysis : 0) +
      (wants.timeline > 0 ? floor.timeline : 0) +
      (wants.console > 0 ? floor.console : 0);
    const outside = commandbar.offsetHeight + toolbar + project;
    const minStage = Math.max(
      160,
      Math.min(drawerOpen ? 200 : 240, window.innerHeight - outside - reserved),
    );
    let room = Math.max(0, window.innerHeight - outside - minStage);

    // 最低限すら入らないほど窮屈なときは、**どれかを 0 にするのではなく
    // 全部を同じ割合で細くする**。ひとつを 0 にすると、そこにあるはずのタブや
    // 見出しが押せなくなる(「Errors タブが押せない」で実際に踏んだ)。
    const squeeze = reserved > 0 ? Math.min(1, room / reserved) : 1;
    analysis = wants.analysis > 0 ? Math.round(floor.analysis * squeeze) : 0;
    room -= analysis;
    timeline = wants.timeline > 0 ? Math.round(floor.timeline * squeeze) : 0;
    room -= timeline;
    consoleRow = wants.console > 0 ? Math.round(floor.console * squeeze) : 0;
    room = Math.max(0, room - consoleRow);
    const topUp = (have: number, want: number) => {
      const extra = Math.min(Math.max(0, want - have), room);
      room -= extra;
      return Math.round(have + extra);
    };
    analysis = topUp(analysis, wants.analysis);
    timeline = topUp(timeline, wants.timeline);
    consoleRow = topUp(consoleRow, wants.console);

    app.style.setProperty("--col-context", `${Math.min(context, width - 320)}px`);
    app.style.setProperty("--col-outline", `${outline}px`);
    app.style.setProperty("--row-analysis", `${analysis}px`);
    app.style.setProperty("--row-timeline", `${timeline}px`);
    app.style.setProperty("--row-toolbar", `${toolbar}px`);
    app.style.setProperty("--row-console", `${consoleRow}px`);
    app.style.setProperty("--row-project", `${project}px`);

    // 寸法 0 のパネルは中身を触らせない(見えないボタンにフォーカスが入る、
    // クリックが吸われる、を防ぐ)。
    app.dataset.analysis = String(analysis > 0);
    app.dataset.outline = String(outline > 0);
    app.dataset.timeline = String(timeline > 0);
    app.dataset.inspector = String(detail >= REVEAL.inspector);
    app.dataset.toolbar = String(toolbar > 0);
    app.dataset.console = String(consoleRow > 0);
    app.dataset.project = String(project > 0);
    dial.value = detail.toFixed(2);
    dial.setAttribute("aria-valuetext", nearestStop(detail).label);
    dialHint.textContent = `${nearestStop(detail).hint}(現象は変わりません)`;
    for (const button of dialStops.querySelectorAll("button")) {
      const at = Number((button as HTMLElement).dataset.at);
      button.classList.toggle("active", nearestStop(detail).at === at);
    }
    if (persist) {
      try {
        localStorage.setItem(DETAIL_KEY, detail.toFixed(2));
      } catch {
        /* 保存できなくても動作には影響しない */
      }
    }
    syncCards();
    // Scene View の器が変わるので three.js のキャンバスを追従させる。
    requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
  }

  function nearestStop(value: number) {
    return GRAIN_STOPS.reduce((best, stop) =>
      Math.abs(stop.at - value) < Math.abs(best.at - value) ? stop : best,
    );
  }

  for (const stop of GRAIN_STOPS) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "detail-stop";
    button.dataset.at = String(stop.at);
    button.dataset.key = stop.key;
    button.textContent = stop.label;
    button.title = stop.hint;
    button.addEventListener("click", () => applyDetail(stop.at));
    // どれを選ぶと何が起きるのかを、**選ぶ前に**読めるようにする。
    // (選んで初めて説明が出るのでは、初めての人には選びようがない)
    button.addEventListener("mouseenter", () => {
      dialHint.textContent = stop.hint;
    });
    button.addEventListener("mouseleave", () => {
      dialHint.textContent = nearestStop(detail).hint;
    });
    dialStops.appendChild(button);
  }
  dial.addEventListener("input", () => applyDetail(Number(dial.value)));
  window.addEventListener("resize", () => applyDetail(detail, false, false));
  // Project ドロワー(素材・回路・リプレイ)の開閉は、粒度とは別の局所的な
  // 操作。開いたら行の高さを与え直す必要があるので、属性の変化を見る。
  new MutationObserver(() => applyDetail(detail, false, false)).observe(app, {
    attributes: true,
    attributeFilter: ["data-drawer"],
  });

  document.addEventListener("keydown", (event) => {
    const target = event.target as HTMLElement | null;
    const typing =
      target &&
      (target.tagName === "INPUT" ||
        target.tagName === "TEXTAREA" ||
        target.isContentEditable);
    if (typing && target?.id !== "palette-input") return;

    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "k") {
      event.preventDefault();
      openPalette();
      return;
    }
    if (!palette.hidden) {
      if (event.key === "Escape") {
        event.preventDefault();
        closePalette();
      }
      return;
    }
    if (typing) return;
    // 粒度を段階的に上げ下げする。連続値だが、キーは目盛りへ寄せる
    // (細かい値はダイヤルのドラッグで作れる)。
    if (event.key === "]") {
      event.preventDefault();
      applyDetail(Math.min(3, Math.floor(detail * 2 + 1) / 2));
    } else if (event.key === "[") {
      event.preventDefault();
      applyDetail(Math.max(0, Math.ceil(detail * 2 - 1) / 2));
    }
  });

  // ---- パンくず(大局 ⇄ 局所 の行き来) ----------------------------------------
  function renderCrumbs(): void {
    crumbs.innerHTML = "";
    const root = document.createElement("button");
    root.type = "button";
    root.className = "crumb crumb-root";
    root.id = "btn-open-palette";
    root.innerHTML = `<span aria-hidden="true">◎</span> 実験をさがす<kbd>⌘K</kbd>`;
    root.title = "46 の実験から選ぶ(⌘K / Ctrl+K)";
    root.addEventListener("click", () => openPalette());
    crumbs.appendChild(root);

    if (!current) {
      // 実験を選んでいない = 自分で組み立てている場面。名前を付けてあれば
      // それを、まだなら「じぶんの場面」と名乗る。
      crumbs.appendChild(separator());
      const own = document.createElement("span");
      own.className = "crumb crumb-own";
      own.id = "crumb-own-scene";
      own.textContent = `🧱 ${ownSceneName || "じぶんの場面"}`;
      own.title = "カタログの実験ではなく、自分で組み立てている場面です";
      crumbs.appendChild(own);
    }

    if (current) {
      crumbs.appendChild(separator());
      const experiment = document.createElement("button");
      experiment.type = "button";
      experiment.className = "crumb";
      experiment.id = "crumb-experiment";
      experiment.textContent = `${current.icon} ${current.title}`;
      experiment.title = current.blurb;
      // 実験そのものへ視点を戻す = 選択を解いて全体を見る。
      experiment.addEventListener("click", () => {
        apiRef.current?.followCamera(true);
        apiRef.current?.selectBody(-1);
        // `lastSelection` の不変条件(このファイル冒頭の doc 参照)。
        lastSelection = -1;
        renderCrumbs();
        renderContext();
      });
      crumbs.appendChild(experiment);
    }

    const api = apiRef.current;
    const selected = api?.selectedBody() ?? -1;
    if (api && selected >= 0) {
      const readout = api.bodyReadout(selected);
      if (readout) {
        crumbs.appendChild(separator());
        const body = document.createElement("span");
        body.className = "crumb crumb-leaf";
        body.id = "crumb-body";
        body.textContent = readout.label;
        crumbs.appendChild(body);
      }
    }
  }

  function separator(): HTMLElement {
    const span = document.createElement("span");
    span.className = "crumb-sep";
    span.setAttribute("aria-hidden", "true");
    span.textContent = "›";
    return span;
  }

  // ---- パレット(目的地への 3 手) ----------------------------------------------
  function openPalette(): void {
    palette.hidden = false;
    paletteInput.value = "";
    filterCategory = null;
    renderFilters();
    renderResults();
    requestAnimationFrame(() => paletteInput.focus());
  }

  function closePalette(): void {
    palette.hidden = true;
  }

  function renderFilters(): void {
    paletteFilters.innerHTML = "";
    const all = document.createElement("button");
    all.type = "button";
    all.className = "palette-filter";
    all.dataset.categoryId = "";
    all.textContent = "すべて";
    all.classList.toggle("active", filterCategory === null);
    all.addEventListener("click", () => {
      filterCategory = null;
      renderFilters();
      renderResults();
    });
    paletteFilters.appendChild(all);

    // 自分の場面があるときだけ、その絞り込みも出す(空の絞り込みは出さない)。
    if (readSavedScenes().length > 0) {
      const own = document.createElement("button");
      own.type = "button";
      own.className = "palette-filter";
      own.id = "palette-filter-own";
      own.dataset.categoryId = OWN_SCENES_FILTER;
      own.textContent = "🧱 じぶんの場面";
      own.title = "自分で組み立てて保存した場面だけを出す";
      own.classList.toggle("active", filterCategory === OWN_SCENES_FILTER);
      own.addEventListener("click", () => {
        filterCategory =
          filterCategory === OWN_SCENES_FILTER ? null : OWN_SCENES_FILTER;
        renderFilters();
        renderResults();
      });
      paletteFilters.appendChild(own);
    }

    for (const category of CATEGORIES) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "palette-filter";
      button.dataset.categoryId = category.id;
      button.textContent = `${category.icon} ${category.title}`;
      button.title = category.blurb;
      button.classList.toggle("active", filterCategory === category.id);
      button.addEventListener("click", () => {
        filterCategory = filterCategory === category.id ? null : category.id;
        renderFilters();
        renderResults();
      });
      paletteFilters.appendChild(button);
    }
  }

  /**
   * **⌘K からは、用意された実験と自分で保存した場面の両方へ行ける**。
   *
   * 以前は自分の場面がここに一切載らず、名前で検索しても「見つかりません」と
   * 出た。上部のシーン選択にも載らないので、保存はできたのに開き直す道が
   * 実質どこにも無く、「二度と開けないのでは」と思わせた(利用者役④の一番の
   * 不満)。入口をひとつに保つ以上、自分の場面もこの入口から出るべきである。
   */
  function matches(): PaletteEntry[] {
    const query = paletteInput.value.trim().toLowerCase();
    const result: PaletteEntry[] = [];

    // 自分の場面が先。数は少なく、探しているのはたいていこちらだから。
    if (!filterCategory || filterCategory === OWN_SCENES_FILTER) {
      for (const scene of readSavedScenes()) {
        if (query && !scene.name.toLowerCase().includes(query)) continue;
        result.push({ kind: "saved", scene });
      }
    }
    if (filterCategory === OWN_SCENES_FILTER) return result;

    for (const category of CATEGORIES) {
      if (filterCategory && category.id !== filterCategory) continue;
      for (const experiment of category.experiments) {
        if (!query) {
          result.push({ kind: "experiment", experiment });
          continue;
        }
        const haystack =
          `${experiment.title} ${experiment.blurb} ${category.title} ` +
          `${experiment.watch.join(" ")} ${experiment.id}`.toLowerCase();
        if (haystack.includes(query)) result.push({ kind: "experiment", experiment });
      }
    }
    return result;
  }

  /** パレットの1行が指すもの。用意された実験か、自分で保存した場面か。 */
  type PaletteEntry =
    | { kind: "experiment"; experiment: Experiment }
    | { kind: "saved"; scene: SavedScene };

  /** 絞り込みの「じぶんの場面」。分野 id と衝突しない値にしてある。 */
  const OWN_SCENES_FILTER = "__own__";

  /// 場面の名前は人が付けるので、そのまま HTML へ埋めない。
  function escapeHtml(text: string): string {
    const box = document.createElement("span");
    box.textContent = text;
    return box.innerHTML;
  }

  function openEntry(entry: PaletteEntry): void {
    if (entry.kind === "experiment") start(entry.experiment);
    else openSavedScene(entry.scene);
  }

  function categoryOf(experiment: Experiment): Category | undefined {
    return CATEGORIES.find((c) => c.experiments.includes(experiment));
  }

  function renderResults(): void {
    paletteMatches = matches();
    paletteIndex = Math.min(paletteIndex, Math.max(0, paletteMatches.length - 1));
    paletteResults.innerHTML = "";
    if (paletteMatches.length === 0) {
      const empty = document.createElement("p");
      empty.className = "palette-empty";
      empty.textContent = "見つかりません。ことばを変えてみてください。";
      paletteResults.appendChild(empty);
      return;
    }
    paletteMatches.forEach((entry, index) => {
      const row = document.createElement("button");
      row.type = "button";
      row.className = "palette-row";
      row.dataset.active = String(index === paletteIndex);
      if (entry.kind === "saved") {
        row.dataset.savedScene = entry.scene.name;
        const when = entry.scene.savedAt
          ? new Date(entry.scene.savedAt).toLocaleString("ja-JP")
          : "";
        row.innerHTML =
          `<span class="palette-row-icon">🧱</span>` +
          `<span class="palette-row-main">` +
          `<span class="palette-row-title">${escapeHtml(entry.scene.name)}</span>` +
          `<span class="palette-row-blurb">${when ? `${escapeHtml(when)} に保存` : "自分で組み立てた場面"}</span>` +
          `</span>` +
          `<span class="palette-row-tag">🧱 じぶんの場面</span>`;
      } else {
        const experiment = entry.experiment;
        const category = categoryOf(experiment);
        row.dataset.experimentId = experiment.id;
        row.innerHTML =
          `<span class="palette-row-icon">${experiment.icon}</span>` +
          `<span class="palette-row-main">` +
          `<span class="palette-row-title">${experiment.title}</span>` +
          `<span class="palette-row-blurb">${experiment.blurb}</span>` +
          `</span>` +
          `<span class="palette-row-tag">${category?.icon ?? ""} ${category?.title ?? ""}</span>`;
      }
      row.addEventListener("click", () => openEntry(entry));
      row.addEventListener("mousemove", () => {
        if (paletteIndex === index) return;
        paletteIndex = index;
        for (const other of paletteResults.querySelectorAll(".palette-row")) {
          (other as HTMLElement).dataset.active = String(
            other === row,
          );
        }
      });
      paletteResults.appendChild(row);
    });
  }

  paletteInput.addEventListener("input", () => {
    paletteIndex = 0;
    renderResults();
  });
  paletteInput.addEventListener("keydown", (event) => {
    if (event.key === "ArrowDown" || event.key === "ArrowUp") {
      event.preventDefault();
      if (paletteMatches.length === 0) return;
      paletteIndex =
        (paletteIndex + (event.key === "ArrowDown" ? 1 : -1) + paletteMatches.length) %
        paletteMatches.length;
      renderResults();
      paletteResults
        .querySelector<HTMLElement>('.palette-row[data-active="true"]')
        ?.scrollIntoView({ block: "nearest" });
    } else if (event.key === "Enter") {
      event.preventDefault();
      const entry = paletteMatches[paletteIndex];
      if (entry) openEntry(entry);
    }
  });
  palette.addEventListener("click", (event) => {
    if (event.target === palette) closePalette();
  });

  // ---- 実験を走らせる ---------------------------------------------------------
  function defaultKnobValues(experiment: Experiment) {
    const values: Record<string, string | number> = {};
    for (const knob of experiment.knobs ?? []) values[knob.id] = knob.value;
    return values;
  }

  function probeLabelsFor(experiment: Experiment): Record<number, string> {
    const labels: Record<number, string> = {};
    for (const [index, entry] of Object.entries(experiment.series ?? {})) {
      labels[Number(index)] = typeof entry === "string" ? entry : entry.label;
    }
    for (const readout of experiment.readouts ?? []) {
      if (readout.derive) continue;
      labels[readout.probe] = readout.label;
    }
    return labels;
  }

  /** グラフの目盛りに添える単位。分かっているものだけ。 */
  function probeUnitsFor(experiment: Experiment): Record<number, string> {
    const units: Record<number, string> = {};
    for (const [index, entry] of Object.entries(experiment.series ?? {})) {
      if (typeof entry !== "string" && entry.unit) units[Number(index)] = entry.unit;
    }
    for (const readout of experiment.readouts ?? []) {
      if (readout.derive) continue;
      // `graph` を持つ読み値は**変換後の単位**を出す(表と同じ量を描くため)。
      const unit = readout.graph?.unit ?? readout.unit;
      if (unit) units[readout.probe] = unit;
    }
    return units;
  }

  /** グラフに描く前にかける変換(`Readout.graph` のdoc参照)。 */
  function probeConvertFor(
    experiment: Experiment,
  ): Record<number, (value: number) => number> {
    const convert: Record<number, (value: number) => number> = {};
    for (const [index, entry] of Object.entries(experiment.series ?? {})) {
      if (typeof entry !== "string" && entry.convert) {
        convert[Number(index)] = entry.convert;
      }
    }
    for (const readout of experiment.readouts ?? []) {
      if (readout.derive || !readout.graph) continue;
      convert[readout.probe] = readout.graph.convert;
    }
    return convert;
  }

  /**
   * シーンの距離拘束を「吊るし線」の指示に直す。読み込んだ JSON を持っている
   * ワークスペース側でしか作れない情報なので、ここで組み立てて渡す。
   */
  function tethersOf(scene: SceneJson) {
    const bodies = scene.bodies ?? [];
    const result: { bodyIndex: number; anchor: [number, number, number] }[] = [];
    for (const joint of scene.joints ?? []) {
      const distance = (joint as { distance?: { body_a?: string; anchor_b?: number[] } })
        .distance;
      if (!distance?.body_a) continue;
      const index = bodies.findIndex((b) => b.name === distance.body_a);
      if (index < 0) continue;
      const anchor = distance.anchor_b ?? [0, 0, 0];
      result.push({
        bodyIndex: index,
        anchor: [anchor[0] ?? 0, anchor[1] ?? 0, anchor[2] ?? 0],
      });
    }
    return result;
  }

  /**
   * 「ここを見る」の一行。**舞台の実際**を先に見て決める。
   *
   * `view` は実験の表につけた札にすぎず、実際とずれることがある——「惑星が
   * 太陽を回る」は `graph` と書いてあるのに、3D では惑星が回っていて、画面は
   * 「下のグラフを見てください」と案内していた(利用者役①の観察)。舞台に
   * 何か映っているなら、まず 3D を指す。
   */
  function stageWhereText(experiment: Experiment, stageEmpty: boolean): string {
    // **いま画面に出ているものだけを指す**。グラフがまだ出ていない濃さで
    // 「下のグラフを見てください」と書いてあって、下は真っ黒のまま——という
    // 食い違いが起きていた(利用者役①の観察)。出ていないなら、出し方を言う。
    // `forceAnalysisOpen`(課題A)で強制的に開いているときも「出ている」扱い
    // にする——実際にグラフの段が見えているのに「ダイヤルを右へ回すと出ます」
    // と書くのは、それこそ舞台の実際と食い違う。
    const graphOnScreen = detail >= REVEAL.analysis || forceAnalysisOpen;
    if (stageEmpty) {
      return graphOnScreen
        ? "📈 舞台には形のある物が出ません。下のグラフとパネルを見てください。"
        : "📈 舞台には形のある物が出ません。右のパネルの数値を見てください(右上のダイヤルを右へ回すと、グラフも出ます)。";
    }
    if (experiment.view === "field") {
      return "👀 3D の中に出る「場」のパネルに、波や分布が描かれます。";
    }
    if (experiment.view === "graph") {
      return graphOnScreen
        ? "👀 まん中の 3D と、下のグラフの両方に出ます。"
        : "👀 まん中の 3D を見てください(右上のダイヤルを右へ回すと、下にグラフも出ます)。";
    }
    return "👀 まん中の 3D を見てください。";
  }

  function sceneJsonFor(experiment: Experiment): string | null {
    if (experiment.build) return JSON.stringify(experiment.build(knobValues));
    if (!experiment.file) return null;
    const raw = sceneFileContent(experiment.file);
    if (!raw) return null;
    const scene = JSON.parse(raw) as SceneJson;
    experiment.prepare?.(scene);
    for (const knob of experiment.knobs ?? []) {
      const value = knobValues[knob.id];
      if (value !== undefined) knob.apply(scene, value);
    }
    return JSON.stringify(scene);
  }

  function start(experiment: Experiment): void {
    current = experiment;
    knobValues = defaultKnobValues(experiment);
    cardOverrides.clear();
    try {
      localStorage.setItem(LAST_EXPERIMENT_KEY, experiment.id);
    } catch {
      /* 記憶できなくても動作には影響しない */
    }
    closePalette();
    // 実験を選び直したら、**人が自分で選んだ濃さ**へ戻す。前の実験の都合で
    // 上げた分を持ち越さない(`chosenDetail` の doc 参照)。前の実験が
    // グラフの段を強制していたぶんもここで畳む——次のフレームで舞台の実際を
    // 見て、必要ならまた開く(下の毎フレーム更新)。畳まないと、3D に物が
    // 映る実験を選んでも一瞬グラフの段が居座って見える。
    if (detail !== chosenDetail || forceAnalysisOpen) {
      forceAnalysisOpen = false;
      applyDetail(chosenDetail, false, false);
    }
    lastStageEmpty = null;
    lastWhereText = "";
    stageEmptyFrames = 0;
    reload();
    renderCrumbs();
    renderContext();
  }

  /**
   * いまのつまみの値でシーンを作り直し、頭から走らせる。
   *
   * `keepPauseIntent: true` を渡すと、**「とめる」で止めていたなら止めた
   * まま**作り直す(利用者役の報告: 「とめる」で一時停止していたのに、
   * 重力のつまみを押したら勝手に再生が始まった。ツールチップ「動かすと、
   * その設定で最初からやり直します」は"最初から"としか言っておらず、
   * "止めていたのに動き出す"ことまでは書いていなかった)。
   *
   * 実験を選び直した直後(`start`)は今までどおり自動で走らせる——初めて
   * 開いた人が空白ではなく動いている現象から始まる、という原則はここでは
   * 崩さない。崩れるのは「すでに開いていて、自分で止めた」場面だけなので、
   * その呼び出し元(つまみの変更・「はじめの設定に戻す」)だけがこのフラグを
   * 渡す。
   */
  function reload(options?: { keepPauseIntent?: boolean }): void {
    const api = apiRef.current;
    if (!current) return;
    if (!api) {
      pendingStart = true;
      return;
    }
    const wasPaused = options?.keepPauseIntent === true && !api.isPlaying();
    const json = sceneJsonFor(current);
    if (!json) return;
    api.loadSceneJson(json);
    api.setTethers(tethersOf(JSON.parse(json) as SceneJson));
    // 読み込み直後は**何も選ばれていない**状態から始める。エディタ側は内部
    // 都合で先頭のボディ(たいてい床)を選ぶが、利用者から見れば自分では
    // 選んでいない——「選んだもの: ground / 重さ 0.000 kg」が出てくるのは
    // ノイズでしかない。選択は人が対象をクリックしたときだけ起きる。
    api.selectBody(-1);
    // `lastSelection` の不変条件(このファイル冒頭の doc 参照)。
    lastSelection = -1;
    api.setProbeLabels(
      probeLabelsFor(current),
      probeUnitsFor(current),
      probeConvertFor(current),
    );
    api.setPace(current.pace * speedMultiplier);
    // 作り直したら「止まった時刻」も忘れる(前回の結果が残っていると、
    // 変えた条件の結果と取り違える)。
    settledAt = null;
    everMoved = false;
    stillFrames = 0;
    // 前の実験の「実際の速さ」を引きずらない(計算の重さは実験ごとに
    // まったく違うため、切り替えた瞬間に古い実測値が一瞬出るのを避ける)。
    actualRateSmoothed = null;
    lastActualRateWallMs = null;
    lastActualRateSimSeconds = null;
    api.followCamera(true);
    if (detail < AUTORUN_BELOW) {
      if (wasPaused) api.pause();
      else api.play();
    } else {
      api.stopForEditing();
    }
    syncRun();
  }

  // ---- 走行コントロール --------------------------------------------------------
  playButton.addEventListener("click", () => {
    const api = apiRef.current;
    if (!api) return;
    // **自分で組み立てた場面も、このボタンで走る**。以前はカタログの実験を
    // 選んでいないとパレットを開いてしまい、置いたばかりの物を落とすのに
    // 「他人の実験一覧」が出てきた(利用者役④の観察)。動かすボタンは
    // 動かすためのものである。
    if (api.isPlaying()) {
      api.pause();
    } else {
      // **課題A: 自分で組み立てている場面(`current`が無い=カタログの実験を
      // 選んでいない)で、Editから初めて走らせる瞬間だけ追従カメラを起こす**。
      //
      // 「＋新規シーン」直後は、組み立て中にエディタと追従カメラが毎フレーム
      // 引っ張り合わないよう、`onSceneReplaced`側でわざと`followCamera(false)`
      // にしてある(そちらのdoc参照)。ところが「うごかす」を押しても
      // どこもそれを起こし直さないままだったので、置いた球(既定の高さ12m)は
      // 落ち始めた瞬間に追いかける者がいない画角の外へ出ていき、着地はおろか
      // 落下そのものが一切映らなかった(進行管理役の実測、`workspace.spec.ts`
      // の該当テストのdoc参照)。
      //
      // `isEditing()`(mode==="edit")で絞るのは2点のため——
      // (1) カタログの実験は`reload()`が読み込み・パラメータ変更のたびに
      //     無条件で`followCamera(true)`を呼んでおり、そちらはすでに面倒を
      //     見ている。ここでも重ねて起こすと、Editで一時停止した状態から
      //     手前でカメラを回して確かめている最中に「うごかす」を押した瞬間、
      //     その手の操作を追従カメラが上書きしてしまう(既存46実験の見え方を
      //     変えないため、`current`が無いときだけに絞る)。
      // (2) 一時停止(`pause()`)は`mode`を"play"のまま変えないので、
      //     一時停止中に自分でカメラを動かした操作を、再開のたびに奪わない
      //     ——`isEditing()`は再開時には偽になる。
      if (!current && api.isEditing()) api.followCamera(true);
      api.play();
    }
    syncRun();
  });
  restartButton.addEventListener("click", () => reload());

  for (const button of speedGroup.querySelectorAll("button")) {
    button.addEventListener("click", () => {
      speedMultiplier = Number((button as HTMLElement).dataset.speed) || 1;
      if (current) apiRef.current?.setPace(current.pace * speedMultiplier);
      syncRun();
    });
  }

  function syncRun(): void {
    const api = apiRef.current;
    const playing = api?.isPlaying() ?? false;
    playButton.dataset.playing = String(playing);
    playButton.innerHTML = playing
      ? `<span aria-hidden="true">⏸</span> とめる`
      : `<span aria-hidden="true">▶</span> うごかす`;
    playButton.setAttribute("aria-label", playing ? "とめる" : "うごかす");
    // 自分の場面には「やり直し」の元が無い(つまみで組み直す実験と違い、
    // 手で置いたものは巻き戻す先が保存した場面しかない)。
    restartButton.disabled = !current;
    for (const button of speedGroup.querySelectorAll("button")) {
      button.classList.toggle(
        "active",
        Number((button as HTMLElement).dataset.speed) === speedMultiplier,
      );
    }
    // 速さは**数として**も出す。🐢/🐇 のボタンが凹んだことは分かっても、
    // どれくらい変わったのかは絵からは読み取れない(利用者役の観察)。
    rate.textContent = `速さ ×${speedMultiplier}`;
  }

  // ---- コンテキスト(カード) ----------------------------------------------------
  //
  // カードは「大局の粒度で自動的に開く閾値(`reveal`)」を持ち、ヘッダを押せば
  // その 1 枚だけ大局と無関係に開閉できる(局所の粒度)。

  type CardSpec = {
    id: string;
    title: string;
    reveal: number;
    summary?: string;
    build: (body: HTMLElement) => void;
  };

  let readoutNodes: { readout: NonNullable<Experiment["readouts"]>[number]; node: HTMLElement }[] = [];
  /** その欄がこれまでに見せた最大の大きさ(「ほぼ 0」の判断に使う)。 */
  const readoutSeenMax = new WeakMap<HTMLElement, number>();
  let focusNodes: Record<string, HTMLElement> = {};
  /** 「選んだもの」の置き場所の入力欄(打っている最中は書き換えない)。 */
  let focusPositionInputs: HTMLInputElement[] = [];
  /**
   * **いま置き直しを頼んだ値**(まだ物理側が追いついていないぶん)。
   *
   * 毎フレームの追いつきが、頼んだ直後の 1〜2 フレームだけ**古い位置**を
   * 書き戻してしまい、打った値が元へ戻ったように見えることがある(遅い実行
   * 環境で実際に踏んだ)。頼んだ値(`want`)と、頼んだ時点で欄に出ていた
   * 古い値(`stale`)を控えておき、実際が動くまでは頼んだ値を出す。
   */
  let focusPositionPending: ({ want: number; until: number } | null)[] = [
    null,
    null,
    null,
  ];
  /** 頼んだ値を出しておく上限の時間 [ms](物理が追いつかなくても、いつかは実際へ戻す)。 */
  const POSITION_PENDING_MS = 1500;
  /** 打ちかけの値(札を組み直しても消さない)。 */
  let focusPositionDraft: (string | null)[] = [null, null, null];
  /** 向きの欄も、置き場所とまったく同じ扱いにする。 */
  let focusRotationInputs: HTMLInputElement[] = [];
  let focusRotationPending: ({ want: number; until: number } | null)[] = [
    null,
    null,
    null,
  ];
  let focusRotationDraft: (string | null)[] = [null, null, null];

  /**
   * 置き場所・向きの欄を**組み直す**ときの初期値。
   *
   * 打っている最中(`draft`)は当然それを出すが、**「決定」した直後、まだ
   * 物理側が追いついていない一瞬**(`pending`)に札の組み直しが挟まると、
   * これまでは`draft`しか見ていなかったので実際の値(古い方)を出してしまい、
   * 頼んだ値がその場で元へ戻ったように見えることがあった——毎フレームの
   * 更新(`renderContext`本体)側は`pending`を締め切りまで尊重するのに、
   * 札を**丸ごと作り直す**ときの初期値だけそれを見ていない、という食い違い。
   * 組み直しの初期値も同じ規則(締め切りまでは頼んだ値)に揃える。
   */
  function pendingOrActual(
    pending: { want: number; until: number } | null,
    actual: number,
    digits: number,
  ): string {
    if (pending !== null && performance.now() <= pending.until) {
      return pending.want.toFixed(digits);
    }
    return actual.toFixed(digits);
  }

  /**
   * **作ったものが消えない**ようにするカード(利用者役④の一番の不満:
   * 「保存に相当する言葉もボタンもどこにもなく、ページを更新しただけで
   * 自作の内容が跡形もなく消えた」)。
   *
   * 保存する中身は**いまの場面そのもの**(`api.exportSceneJson`)。読み直しは
   * 実験を読むのと同じ経路を通るので、開き直した場面は同じ物理で動く。
   * 置き場はこの端末(localStorage)と、持ち出せるファイルの 2 つ。
   */
  /** 保存できたことの知らせ(カードを組み立てるときに出す)。 */
  let sceneSaveNote: string | null = null;

  /** いま名前欄に入っている名前(まだ保存していなくても、これを使う)。 */
  function chosenSceneName(): string {
    return (sceneNameDraft || ownSceneName || "").trim();
  }

  /**
   * 場面の文書に、人が付けた名前を書き込む。
   *
   * 書き出したファイルは名前が `my-scene.json`、中の名前も `current` のままで、
   * 自分が付けた名前がどこにも残らなかった(利用者役④の観察)。読み直したとき
   * に同じ名前で戻るよう、文書そのものへ書く。壊れた文書は触らずそのまま返す。
   */
  function namedSceneJson(json: string, name: string): string {
    if (!name) return json;
    try {
      const parsed = JSON.parse(json) as Record<string, unknown>;
      parsed.name = name;
      return JSON.stringify(parsed);
    } catch {
      return json;
    }
  }

  /** 場面の文書に書いてある名前(無ければ空)。 */
  function sceneJsonName(json: string): string {
    try {
      const parsed = JSON.parse(json) as { name?: unknown };
      const name = typeof parsed.name === "string" ? parsed.name.trim() : "";
      // 既定の作業名は「名前が付いている」とは言えない。
      return name === "current" ? "" : name;
    } catch {
      return "";
    }
  }

  function savedScenesCard(): CardSpec {
    return {
      id: "my-scenes",
      title: "この場面を保存する",
      // 「つくる」に踏み込んだ人の道具。浅い粒度では出さない。
      reveal: REVEAL.toolbar,
      build: (body) => {
        const note = document.createElement("p");
        note.className = "card-note";
        note.textContent =
          "名前を付けて保存すると、次に開いたときそのまま続きから始められます。";
        body.appendChild(note);

        const row = document.createElement("div");
        row.className = "card-actions";
        const nameInput = document.createElement("input");
        nameInput.type = "text";
        nameInput.id = "input-scene-name";
        nameInput.placeholder = "場面の名前";
        nameInput.value = sceneNameDraft || ownSceneName;
        nameInput.addEventListener("input", () => {
          sceneNameDraft = nameInput.value;
        });
        const save = document.createElement("button");
        save.type = "button";
        save.id = "btn-save-scene";
        save.className = "primary";
        save.textContent = "💾 保存する";
        save.addEventListener("click", () => {
          const api = apiRef.current;
          if (!api) return;
          const json = api.exportSceneJson();
          if (!json) return;
          const name =
            (sceneNameDraft || nameInput.value).trim() ||
            `場面 ${new Date().toLocaleString("ja-JP", { month: "numeric", day: "numeric", hour: "2-digit", minute: "2-digit" })}`;
          const scenes = readSavedScenes().filter((entry) => entry.name !== name);
          scenes.unshift({
            name,
            savedAt: new Date().toISOString(),
            json: namedSceneJson(json, name),
          });
          const error = writeSavedScenes(scenes);
          if (error) {
            status.textContent = error;
            status.dataset.tone = "warn";
            return;
          }
          ownSceneName = name;
          sceneNameDraft = name;
          sceneSaveNote = `「${name}」を取っておきました。⌘K で名前を打つと、いつでも開けます。`;
          try {
            localStorage.setItem(LAST_OWN_SCENE_KEY, name);
          } catch {
            /* 覚えられなくても保存自体は済んでいる */
          }
          renderCrumbs();
          renderContext();
        });
        row.append(nameInput, save);
        body.appendChild(row);

        const status = document.createElement("p");
        status.className = "card-note";
        status.id = "scene-save-status";
        // **取っておけたことを、画面で言う**。押しても何も変わらなかったので、
        // 保存できたのか押し損ねたのか分からず、「二度と開けないのでは」と
        // 読まれた(利用者役④の観察)。カードは保存のたびに組み直されるので、
        // 知らせは変数に置いて、組み立てるときに出す。
        if (sceneSaveNote) {
          const shown = sceneSaveNote;
          status.textContent = shown;
          status.dataset.tone = "ok";
          window.setTimeout(() => {
            if (sceneSaveNote === shown) sceneSaveNote = null;
            if (status.isConnected && status.textContent === shown) {
              status.textContent = "";
              delete status.dataset.tone;
            }
          }, 6000);
        }
        body.appendChild(status);

        const scenes = readSavedScenes();
        if (scenes.length > 0) {
          const list = document.createElement("ul");
          list.className = "saved-scenes";
          for (const entry of scenes) {
            const item = document.createElement("li");
            const open = document.createElement("button");
            open.type = "button";
            open.className = "saved-scene-open";
            open.dataset.sceneName = entry.name;
            open.textContent = entry.name;
            open.title = `${new Date(entry.savedAt).toLocaleString("ja-JP")} に保存`;
            open.addEventListener("click", () => openSavedScene(entry));
            const remove = document.createElement("button");
            remove.type = "button";
            remove.className = "saved-scene-remove";
            remove.textContent = "消す";
            remove.setAttribute("aria-label", `${entry.name} を消す`);
            remove.addEventListener("click", () => {
              // 取り消せない操作を、押した瞬間に実行しない(元に戻せるのか
              // 分からず不安になった、と書かれた——利用者役④の観察)。
              if (!window.confirm(`「${entry.name}」を消します。元には戻せません。`)) {
                return;
              }
              writeSavedScenes(readSavedScenes().filter((e) => e.name !== entry.name));
              renderContext();
            });
            item.append(open, remove);
            list.appendChild(item);
          }
          body.appendChild(list);
        }

        // 端末の中だけでは、消える不安は消えない。**持ち出せる形**も要る。
        const files = document.createElement("div");
        files.className = "card-actions";
        const download = document.createElement("button");
        download.type = "button";
        download.id = "btn-scene-download";
        download.textContent = "⬇ ファイルに書き出す";
        download.addEventListener("click", () => {
          const json = apiRef.current?.exportSceneJson();
          if (!json) return;
          const blob = new Blob([namedSceneJson(json, chosenSceneName())], {
            type: "application/json",
          });
          const url = URL.createObjectURL(blob);
          const anchorElement = document.createElement("a");
          anchorElement.href = url;
          anchorElement.download = `${chosenSceneName() || "my-scene"}.json`;
          // **文書に入れてから押す**。ぶら下がったままの `<a>` を押しても
          // 名前(`download`)が効かず、保存されるファイルは中身に関係なく
          // いつも「download」——拡張子も付かず、書き出しを重ねるとどれが
          // どれだか分からなくなっていた(利用者役④の観察)。取り消し
          // (`revokeObjectURL`)も、保存が始まってからにする。
          anchorElement.style.display = "none";
          document.body.appendChild(anchorElement);
          anchorElement.click();
          anchorElement.remove();
          window.setTimeout(() => URL.revokeObjectURL(url), 0);
        });
        const upload = document.createElement("input");
        upload.type = "file";
        upload.accept = "application/json,.json";
        upload.id = "input-scene-file";
        upload.addEventListener("change", async () => {
          const file = upload.files?.[0];
          if (!file) return;
          const text = await file.text();
          openSavedScene({
            // 中に書いてある名前を優先する(付けた名前で書き出しているので、
            // 読み直したときも同じ名前で戻る)。
            name: sceneJsonName(text) || file.name.replace(/\.json$/, ""),
            savedAt: "",
            json: text,
          });
        });
        files.append(download, upload);
        body.appendChild(files);
      },
    };
  }

  /** 保存した場面を開く。実験を読むのと同じ経路(=同じ物理)を通す。 */
  function openSavedScene(entry: SavedScene): void {
    const api = apiRef.current;
    if (!api) return;
    current = null;
    ownSceneName = entry.name;
    try {
      localStorage.setItem(LAST_OWN_SCENE_KEY, entry.name);
    } catch {
      /* 覚えられなくても開くことはできる */
    }
    api.loadSceneJson(entry.json);
    api.selectBody(-1);
    // `lastSelection` の不変条件(このファイル冒頭の doc 参照)。
    lastSelection = -1;
    api.setProbeLabels(null, null);
    api.setPace(null);
    // 自分の場面は組み立てるためのものなので、カメラは追いかけない。
    api.followCamera(false);
    // 開いた直後は**止まっている**。作る人は置いてから動かす。
    api.stopForEditing();
    closePalette();
    renderCrumbs();
    renderContext();
    syncRun();
  }

  function renderContext(): void {
    // **打っている最中に組み直されても、手が離れない**ようにする。カードは
    // 選択が変わるたびに作り直すので、素朴に張り替えると入力中の欄から
    // フォーカスもカーソル位置も飛ぶ(名前を打っている途中で保存が
    // 自動命名になった、が実際に起きた)。
    const active = document.activeElement as HTMLInputElement | null;
    const activeId = active?.id ?? "";
    const activeStart = active?.selectionStart ?? null;
    const activeEnd = active?.selectionEnd ?? null;

    contextBody.innerHTML = "";
    readoutNodes = [];
    focusNodes = {};
    focusPositionInputs = [];
    focusPositionPending = [null, null, null];
    focusRotationInputs = [];
    focusRotationPending = [null, null, null];

    if (!current) {
      // カタログの実験ではなく、**いまそこにある世界**を見ている状態
      // (起動直後の作業机や、自分で組み立てた途中の世界)。ここでも文脈の柱は
      // 空にしない——何を見ているのか・次に何ができるのかは常に要る。
      const world: CardSpec[] = [
        {
          id: "world",
          title: "いまの世界",
          reveal: 0,
          build: (body) => {
            const note = document.createElement("p");
            note.className = "card-note";
            note.textContent =
              "自分で組み立てている世界です。床の上のものが落ち、ぶつかり、" +
              "止まります。用意された現象を見たいときは下から探せます。";
            body.appendChild(note);
            const actions = document.createElement("div");
            actions.className = "card-actions";
            const open = document.createElement("button");
            open.type = "button";
            open.id = "btn-empty-open";
            open.textContent = "◎ 実験をさがす";
            open.addEventListener("click", () => openPalette());
            actions.appendChild(open);
            body.appendChild(actions);
          },
        },
        {
          id: "numbers",
          title: "いまの数値",
          reveal: 0,
          build: (body) => {
            const list = document.createElement("dl");
            list.className = "readouts";
            const key = document.createElement("dt");
            key.textContent = "経過した時間";
            const value = document.createElement("dd");
            value.id = "readout-time";
            value.dataset.seconds = "0";
            // 最初の一枚から、その場面の単位で書く(`formatDuration` の doc)。
            value.textContent = formatDuration(0, apiRef.current?.stepSeconds() ?? 1);
            list.append(key, value);
            body.appendChild(list);
          },
        },
        savedScenesCard(),
      ];
      for (const spec of world) contextBody.appendChild(buildCard(spec));
      appendFocusCard();
      syncCards();
      restoreFocus(activeId, activeStart, activeEnd);
      return;
    }

    const experiment = current;
    const specs: CardSpec[] = [];

    specs.push({
      id: "watch",
      title: "ここを見る",
      // どの粒度でも開いている。「どこを見ればいいか」は、いちばん浅い
      // 見方をしている人にこそ必要な情報だから。
      reveal: 0,
      summary: experiment.watch[0],
      build: (body) => {
        const where = document.createElement("p");
        where.className = "card-where";
        // **舞台の実際と食い違わせない**。「まん中の 3D を見てください」と
        // 書いてある隣で、舞台が「形のある物は出てきません」と言っている、
        // という矛盾が起きていた(利用者役①の観察)。実際に何か描かれて
        // いるかを見てから決める。
        const stageEmpty = apiRef.current?.stageIsEmpty() ?? false;
        where.textContent = stageWhereText(experiment, stageEmpty);
        body.appendChild(where);
        const list = document.createElement("ul");
        list.className = "card-watch";
        for (const line of experiment.watch) {
          const item = document.createElement("li");
          item.textContent = line;
          list.appendChild(item);
        }
        body.appendChild(list);
      },
    });

    specs.push({
      id: "numbers",
      title: "いまの数値",
      reveal: 0, // 走っていることの唯一の手掛かりなので、いちばん薄い粒度から出す。
      build: (body) => {
        const list = document.createElement("dl");
        list.className = "readouts";
        const timeKey = document.createElement("dt");
        timeKey.textContent = "経過した時間";
        const timeValue = document.createElement("dd");
        timeValue.id = "readout-time";
        timeValue.dataset.seconds = "0";
        timeValue.textContent = formatDuration(0, apiRef.current?.stepSeconds() ?? 1);
        list.append(timeKey, timeValue);
        // 「動きが止まった時刻」は**止まったと分かってから**現れる。最初から
        // 空欄で置いておくと、埋まらない欄が気になって現象から目が離れる。
        const settledKey = document.createElement("dt");
        settledKey.id = "readout-settled-key";
        // 判定の基準は**その場に書く**。tooltip に書いてあっても読まれない
        // ——「止まった」の定義が画面から分からない、と書かれた(利用者役③)。
        settledKey.textContent = "ほぼ止まった時刻(0.05 m/s 以下)";
        settledKey.title =
          "いちばん速い物の速さが 0.05 m/s を下回ったまま続いた時点です" +
          "(物理には手を加えていません——見えている値からそう読めた、というだけの表示)。";
        settledKey.hidden = true;
        const settledValue = document.createElement("dd");
        settledValue.id = "readout-settled";
        settledValue.hidden = true;
        list.append(settledKey, settledValue);
        for (const readout of experiment.readouts ?? []) {
          const key = document.createElement("dt");
          key.textContent = readout.label;
          const value = document.createElement("dd");
          value.dataset.probe = String(readout.probe);
          value.textContent = "—";
          list.append(key, value);
          readoutNodes.push({ readout, node: value });
        }
        body.appendChild(list);
      },
    });

    if (!experiment.knobs?.length) {
      // **つまみが無いことを、黙って隠さない**。「変えてみる」欄そのものが
      // 消えていたので、条件を変えられない実験があるとは思わず、探し回った
      // 末に諦めることになった(利用者役②の観察)。無いなら無いと言う。
      specs.push({
        id: "knobs",
        title: "変えてみる",
        reveal: 0.8,
        summary: "この実験は見るだけです",
        build: (body) => {
          const note = document.createElement("p");
          note.className = "card-note";
          note.textContent =
            "この実験に変えられるつまみはありません——場の中身そのものが" +
            "記録された状態から始まるので、途中の条件を差し替えられないためです。" +
            "見どころは「ここを見る」に書いてあります。";
          body.appendChild(note);
        },
      });
    }

    if (experiment.knobs?.length) {
      specs.push({
        id: "knobs",
        title: "変えてみる",
        // 目盛り「さわる」(1.0)に届く手前から開く——ダイヤルが「さわる」と
        // 言っているのにつまみが畳まれている、という食い違いを作らない。
        reveal: 0.8,
        summary: experiment.knobs.map((k) => k.label).join(" / "),
        build: (body) => {
          const note = document.createElement("p");
          note.className = "card-note";
          note.textContent = "動かすと、その設定で最初からやり直します。";
          body.appendChild(note);
          for (const knob of experiment.knobs ?? []) body.appendChild(renderKnob(knob));
          // **戻れること**。いじった後に元へ戻す道が無く、同じ実験を選び直す
          // という遠回りを見つけるまで戻れなかった(利用者役の観察)。
          const reset = document.createElement("button");
          reset.type = "button";
          reset.id = "btn-reset-knobs";
          reset.className = "knob-reset";
          reset.textContent = "はじめの設定に戻す";
          reset.addEventListener("click", () => {
            knobValues = defaultKnobValues(experiment);
            reload({ keepPauseIntent: true });
            renderContext();
          });
          body.appendChild(reset);
        },
      });
    }

    for (const spec of specs) contextBody.appendChild(buildCard(spec));
    appendFocusCard();
    // **用意された実験の上に組み立てた場合も保存できる**。以前は「自分の場面」
    // (実験を選んでいない状態)のときしか保存の口を出しておらず、実験に物を
    // 足して衝突させた人が、それを取っておく場所を見つけられなかった
    // ——そのまま ⌘K で別の実験へ行き、戻る道が無くなった(利用者役④の観察)。
    contextBody.appendChild(buildCard(savedScenesCard()));
    contextBody.appendChild(buildCard(viewCardSpec()));
    syncCards();
    restoreFocus(activeId, activeStart, activeEnd);
  }

  /**
   * 組み直す前に触っていた欄へ、カーソル位置ごと手を戻す
   * (`renderContext` の冒頭 doc 参照)。同じ id の欄が無くなっていれば何もしない。
   */
  function restoreFocus(id: string, start: number | null, end: number | null): void {
    if (!id) return;
    const next = document.getElementById(id) as HTMLInputElement | null;
    if (!next || next === document.activeElement) return;
    next.focus();
    if (start !== null && end !== null && typeof next.setSelectionRange === "function") {
      try {
        next.setSelectionRange(start, end);
      } catch {
        /* range を持たない型(number 入力等)では何もしない */
      }
    }
  }

  /** 選択中の対象があれば、その 1 つに寄った文脈を足す(局所への踏み込み)。 */
  function appendFocusCard(): void {
    const api = apiRef.current;
    const selected = api?.selectedBody() ?? -1;
    if (api && selected >= 0) {
      const readout = api.bodyReadout(selected);
      if (readout) {
        contextBody.appendChild(buildCard({
          id: "focus",
          title: `選んだもの — ${readout.label}`,
          reveal: 0, // 選ぶ行為そのものが局所への踏み込みなので、常に開く。
          build: (body) => {
            const list = document.createElement("dl");
            list.className = "readouts";
            const rows: [string, string][] = [
              ["かたち", friendlyShape(readout.shape)],
              ["重さ", `${readout.mass.toFixed(3)} kg`],
              ["高さ", `${readout.position[1].toFixed(3)} m`],
              ["速さ", `${readout.speed.toFixed(3)} m/s`],
            ];
            for (const [key, value] of rows) {
              const dt = document.createElement("dt");
              dt.textContent = key;
              const dd = document.createElement("dd");
              dd.dataset.focus = key;
              dd.textContent = value;
              list.append(dt, dd);
              focusNodes[key] = dd;
            }
            body.appendChild(list);

            // **材質は、ここで選び直せる**。差し替える仕組みは前からあったが、
            // 置き場所が Inspector の 750px 下で、目の前のこの札には文字しか
            // 出ていなかった。「鋼で試して、次にゴムで比べる」がやりたくて
            // ここを何度も押した人が、押せないまま諦めていた(利用者役④)。
            const materials = api.materialNames();
            if (materials.length > 0) {
              const row = document.createElement("div");
              row.className = "focus-material";
              const label = document.createElement("label");
              label.textContent = "材質";
              label.htmlFor = "focus-material";
              const select = document.createElement("select");
              select.id = "focus-material";
              const names = materials.includes(readout.material)
                ? materials
                : [readout.material, ...materials];
              for (const name of names) {
                const option = document.createElement("option");
                option.value = name;
                option.textContent = name;
                option.selected = name === readout.material;
                select.appendChild(option);
              }
              select.addEventListener("change", () => {
                if (api.setBodyMaterial(selected, select.value)) {
                  renderContext();
                  return;
                }
                select.value = readout.material;
                const note = document.getElementById("focus-material-note");
                if (note) {
                  note.textContent =
                    "材質は、とめている間だけ変えられます(▶ を押す前に)。";
                }
              });
              row.append(label, select);
              body.appendChild(row);
              const note = document.createElement("p");
              note.className = "card-note";
              note.id = "focus-material-note";
              // 場面が重さを直接決めていることがある(D24 の車体は 600 kg
              // 固定)。それを知らずに「鋼なのに密度が合わない」と読まれた
              // ので、いまの重さの出どころも書いておく(利用者役③の観察)。
              note.textContent =
                "いまの重さは場面が直接決めていることがあります。材質を選び直すと、" +
                "そこからは密度で計算し直し、場面を最初から組み直します。";
              body.appendChild(note);
            }

            // **置き場所も、この札で決められる**。数値の欄は Inspector の
            // ずっと下にあり、見つけられないまま「2 つの物をぶつける」という
            // 一番やりたかったことを諦めていた(利用者役④の一番の不満)。
            const place = document.createElement("div");
            place.className = "focus-place";
            const placeLabel = document.createElement("label");
            placeLabel.textContent = "置き場所 x, y, z [m]";
            placeLabel.htmlFor = "focus-pos-x";
            place.appendChild(placeLabel);
            const fields = document.createElement("div");
            fields.className = "focus-place-fields";
            const inputs = (["x", "y", "z"] as const).map((axis, i) => {
              const input = document.createElement("input");
              input.type = "number";
              input.step = "0.1";
              input.id = `focus-pos-${axis}`;
              // **打ちかけの値は、札を組み直しても残す**。カードは選択や材質の
              // 変更で組み直されるので、打った直後に組み直しが挟まると、打った
              // 値が消えて元の位置が入り、そのまま「決定」されていた(遅い機械
              // の CI で、3 と打ったのに 1.5 のままになる形で表に出た)。
              input.value =
                focusPositionDraft[i] ??
                pendingOrActual(focusPositionPending[i], readout.position[i], 3);
              input.dataset.axis = axis;
              input.addEventListener("input", () => {
                focusPositionDraft[i] = input.value;
              });
              fields.appendChild(input);
              return input;
            });
            const push = () => {
              const [x, y, z] = inputs.map((input) => Number(input.value));
              if (![x, y, z].every((v) => Number.isFinite(v))) return;
              focusPositionPending = [x, y, z].map((want) => ({
                want,
                until: performance.now() + POSITION_PENDING_MS,
              }));
              focusPositionDraft = [null, null, null];
              api.setBodyPosition(selected, x, y, z);
            };
            for (const input of inputs) input.addEventListener("change", push);
            place.appendChild(fields);
            body.appendChild(place);
            focusPositionInputs = inputs;

            // **向きも数値で決められるようにする**(`setBodyRotation` の doc)。
            const turn = document.createElement("div");
            turn.className = "focus-place";
            const turnLabel = document.createElement("label");
            turnLabel.textContent = "向き x, y, z [度]";
            turnLabel.htmlFor = "focus-rot-x";
            turn.appendChild(turnLabel);
            const turnFields = document.createElement("div");
            turnFields.className = "focus-place-fields";
            const turnInputs = (["x", "y", "z"] as const).map((axis, i) => {
              const input = document.createElement("input");
              input.type = "number";
              input.step = "5";
              input.id = `focus-rot-${axis}`;
              input.value =
                focusRotationDraft[i] ??
                pendingOrActual(focusRotationPending[i], readout.rotation[i], 1);
              input.dataset.axis = axis;
              input.addEventListener("input", () => {
                focusRotationDraft[i] = input.value;
              });
              turnFields.appendChild(input);
              return input;
            });
            const pushTurn = () => {
              const [rx, ry, rz] = turnInputs.map((input) => Number(input.value));
              if (![rx, ry, rz].every((v) => Number.isFinite(v))) return;
              focusRotationPending = [rx, ry, rz].map((want) => ({
                want,
                until: performance.now() + POSITION_PENDING_MS,
              }));
              focusRotationDraft = [null, null, null];
              api.setBodyRotation(selected, rx, ry, rz);
            };
            for (const input of turnInputs) {
              input.addEventListener("change", pushTurn);
            }
            turn.appendChild(turnFields);
            const turnNote = document.createElement("p");
            turnNote.className = "card-note";
            turnNote.id = "focus-rotation-note";
            // 「x か z を 20〜40 度に」とだけ書いていたが、それを実際にやっても
            // 坂にならなかった(進行管理役の裏取り)。ゆか(平面)は向きを変えても
            // 傾かない(物理側がPlaneの向きをnormal/dで持ち、回転を反映しない
            // ——今回の直す範囲外)ので、その旨を正直に書く。箱などを傾けた
            // 場合も、動き方が「動く(Dynamic)」のままだと重力で転がって平らに
            // 戻るので、「動かない(Static)」にする所までを案内する
            // (課題A: 利用者役はこの手順が無いために坂を組み立てられなかった)。
            turnNote.textContent =
              shapeHead(readout.shape) === "plane"
                ? "ここ(ゆか)の向きはここでは変えられません。坂を作るには、下の「＋ 追加」で箱などを置き、向きを 20〜40 度にしたうえで、動き方を「動かない(Static)」にしてください。"
                : "坂として使うには、向きを 20〜40 度にしたうえで、下の「動き方」を「動かない(Static)」にしてください。動くままだと、重力で転がって平らに戻ります。";
            turn.appendChild(turnNote);
            body.appendChild(turn);
            focusRotationInputs = turnInputs;

            // **この物もグラフに記録する**。記録が付くのは最初に置いた物だけで、
            // 2 つ目以降を比べたくても足す手段がどこにも無かった(利用者役④)。
            if (!api.hasBodyProbes(selected)) {
              const record = document.createElement("button");
              record.type = "button";
              record.id = "btn-record-body";
              record.textContent = "📈 この物もグラフに記録する";
              record.addEventListener("click", () => {
                api.addBodyProbes(selected);
                renderContext();
              });
              body.appendChild(record);
            }
            const actions = document.createElement("div");
            actions.className = "card-actions";
            const follow = document.createElement("button");
            follow.type = "button";
            follow.id = "btn-follow-body";
            follow.textContent = "👀 これを追いかける";
            follow.addEventListener("click", () => api.followCamera(true));
            const clear = document.createElement("button");
            clear.type = "button";
            clear.id = "btn-clear-selection";
            clear.textContent = "全体へ戻る";
            clear.title = "選ぶのをやめて、ぜんぶが入る画角へ戻します";
            clear.addEventListener("click", () => {
              api.selectBody(-1);
              // `lastSelection` の不変条件(このファイル冒頭の doc 参照)。
              lastSelection = -1;
              // 名前どおり**画角も戻す**。選択を外すだけだったので、置き場所を
              // 数値で変えて物を見失った人が、押しても何も変わらないまま
              // 詰まっていた(利用者役④の観察)。ただし毎フレーム追いかける
              // `followCamera(true)`だと、止まっている物が豆粒になるまで
              // 引いてしまうことがあった(利用者役の報告、`frameOnContent`の
              // doc参照)。一回だけ、いま入れる画角に合わせ直す。
              api.frameOnContent();
              renderCrumbs();
              renderContext();
            });
            actions.append(follow, clear);
            // **消す手段を、この札から見つけられるようにする**(利用者役の
            // 報告: 置いた物を消す手段がDeleteキーしか無く、画面のどこにも
            // 書かれていなかった——自動テストで偶然見つけたと書かれた)。
            // ゆか(床)は場面の基準面なので、Hierarchy右クリックの「削除」と
            // 同じく対象から外す(`removeBody`も床は`false`を返す)。
            if (shapeHead(readout.shape) !== "plane") {
              // **押し間違いで消えると困る操作**なので、1回押しただけでは
              // 消さない——ラベルを「本当に消しますか?」に変えて、続けて
              // 押したときだけ`removeBody`を呼ぶ(数秒操作が無ければ元に戻す)。
              // ネイティブの`confirm()`はPlaywrightでの自動化やモーダル体験の
              // 一貫性の面で避け、ボタンの中で完結させる。
              const remove = document.createElement("button");
              remove.type = "button";
              remove.id = "btn-remove-body";
              remove.className = "btn-danger";
              remove.textContent = "🗑 これを消す";
              remove.title = "この物を場面から消します(元に戻せません——「動かしたのを戻す」の対象外です)";
              let armed = false;
              let armedTimer: ReturnType<typeof setTimeout> | null = null;
              const disarm = () => {
                armed = false;
                if (armedTimer !== null) clearTimeout(armedTimer);
                armedTimer = null;
                remove.textContent = "🗑 これを消す";
                remove.classList.remove("btn-danger-armed");
              };
              remove.addEventListener("click", () => {
                if (!armed) {
                  armed = true;
                  remove.textContent = "本当に消しますか?(もう一度押す)";
                  remove.classList.add("btn-danger-armed");
                  armedTimer = setTimeout(disarm, 4000);
                  return;
                }
                disarm();
                if (api.removeBody(selected)) {
                  api.selectBody(-1);
                  // `lastSelection` の不変条件(このファイル冒頭の doc 参照)。
                  lastSelection = -1;
                  api.frameOnContent();
                  renderCrumbs();
                  renderContext();
                }
              });
              actions.appendChild(remove);
            }
            body.appendChild(actions);
          },
        }));
      }
    }
  }

  function viewCardSpec(): CardSpec {
    return {
      id: "view",
      title: "見え方",
      reveal: 1.4,
      build: (body) => {
        const actions = document.createElement("div");
        actions.className = "card-actions";
        const camera = document.createElement("button");
        camera.type = "button";
        camera.id = "btn-refocus";
        camera.textContent = "👀 カメラを合わせ直す";
        camera.title = "対象が画面から外れたとき、追いかけるカメラに戻す";
        camera.addEventListener("click", () => apiRef.current?.followCamera(true));
        const graphs = document.createElement("button");
        graphs.type = "button";
        graphs.id = "btn-toggle-analysis";
        graphs.textContent =
          detail >= ANALYSIS_READABLE ? "📈 グラフをしまう" : "📈 グラフを出す";
        graphs.addEventListener("click", () =>
          applyDetail(detail >= ANALYSIS_READABLE ? 0.8 : ANALYSIS_READABLE),
        );
        actions.append(camera, graphs);
        body.appendChild(actions);
        const note = document.createElement("p");
        note.className = "card-note";
        note.textContent =
          "右上のダイヤルを右へ回すほど、一覧・グラフ・編集の道具が増えます。";
        body.appendChild(note);
      },
    };
  }

  function buildCard(spec: CardSpec): HTMLElement {
    const card = document.createElement("section");
    card.className = "card";
    card.dataset.card = spec.id;
    card.dataset.reveal = String(spec.reveal);

    const header = document.createElement("button");
    header.type = "button";
    header.className = "card-header";
    header.dataset.cardToggle = spec.id;
    header.innerHTML =
      `<span class="card-title">${spec.title}</span>` +
      (spec.summary ? `<span class="card-summary">${spec.summary}</span>` : "") +
      `<span class="card-chevron" aria-hidden="true">▾</span>`;
    header.addEventListener("click", () => {
      // 局所の粒度: この 1 枚だけ、大局と無関係に開閉する。
      const open = card.dataset.expanded !== "true";
      cardOverrides.set(spec.id, open);
      card.dataset.expanded = String(open);
      header.setAttribute("aria-expanded", String(open));
    });
    card.appendChild(header);

    const body = document.createElement("div");
    body.className = "card-body";
    spec.build(body);
    card.appendChild(body);
    return card;
  }

  /** 大局の粒度と、カードごとの上書きから、各カードの開閉を決める。 */
  function syncCards(): void {
    for (const card of contextBody.querySelectorAll<HTMLElement>(".card")) {
      const id = card.dataset.card ?? "";
      const reveal = Number(card.dataset.reveal ?? "0");
      const override = cardOverrides.get(id);
      const open = override ?? detail >= reveal;
      card.dataset.expanded = String(open);
      card.dataset.auto = String(override === undefined);
      card
        .querySelector(".card-header")
        ?.setAttribute("aria-expanded", String(open));
    }
  }

  function renderKnob(knob: Knob): HTMLElement {
    const wrap = document.createElement("div");
    wrap.className = "knob";
    wrap.dataset.knobId = knob.id;

    const label = document.createElement("label");
    label.className = "knob-label";
    label.textContent = knob.label;
    wrap.appendChild(label);

    if (knob.kind === "range") {
      const row = document.createElement("div");
      row.className = "knob-row";
      const input = document.createElement("input");
      input.type = "range";
      input.min = String(knob.min ?? 0);
      input.max = String(knob.max ?? 10);
      input.step = String(knob.step ?? 1);
      // **帯が指す値と、中身の値をずらさない**。範囲入力は目盛りに乗らない値を
      // 表示のときに丸めるので、既定値が目盛りから外れていると「帯は 75 を
      // 指しているのに、シミュレーションは 77 で走っている」という食い違いが
      // 起きる(利用者役③の観察)。表示された値を、そのまま中身にも書き戻す。
      input.value = String(knobValues[knob.id] ?? knob.value);
      const snapped = Number(input.value);
      if (Number.isFinite(snapped) && knobValues[knob.id] !== snapped) {
        knobValues[knob.id] = snapped;
      }
      input.id = `knob-${knob.id}`;
      label.htmlFor = input.id;
      const output = document.createElement("output");
      output.className = "knob-value";
      const paint = () => {
        output.textContent = `${input.value}${knob.unit ? ` ${knob.unit}` : ""}`;
      };
      paint();
      input.addEventListener("input", () => {
        knobValues[knob.id] = Number(input.value);
        paint();
      });
      input.addEventListener("change", () => reload({ keepPauseIntent: true }));
      row.append(input, output);
      wrap.appendChild(row);
    } else {
      const group = document.createElement("div");
      group.className = "knob-choice";
      // 範囲のつまみと同じように id で指せるようにしておく(選択肢のつまみだけ
      // 名前が無く、テストからも人からも「そこ」を指しにくかった)。
      group.id = `knob-${knob.id}`;
      label.htmlFor = group.id;
      // **課題B**: 材質ボタンは「材質そのものは変わったと分かるが、その材質が
      // よく滑るのかどうかは画面のどこにも書いていない」(利用者役の観察:
      // 氷が滑りやすいだろうと勘で選ぶしかなかった)。数値をでっち上げず、
      // アプリが実際に使っている摩擦係数(`api.materialFriction`、Rust側の
      // 材質DBそのもの)をボタンへ添える——材質つまみ(`knob.id === "material"`)
      // だけの上乗せで、他のchoiceつまみ(重力の天体選びなど)には影響しない。
      const friction = (value: string | number): number =>
        knob.id === "material"
          ? (apiRef.current?.materialFriction(String(value)) ?? NaN)
          : NaN;
      for (const option of knob.options ?? []) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "knob-choice-btn";
        const f = friction(option.value);
        button.textContent = Number.isFinite(f)
          ? `${option.label}(摩擦 ${f.toFixed(2)})`
          : option.label;
        if (Number.isFinite(f)) button.dataset.friction = String(f);
        button.dataset.value = String(option.value);
        button.classList.toggle(
          "active",
          String(knobValues[knob.id]) === String(option.value),
        );
        button.addEventListener("click", () => {
          knobValues[knob.id] = option.value;
          for (const sibling of group.querySelectorAll(".knob-choice-btn")) {
            sibling.classList.toggle(
              "active",
              (sibling as HTMLElement).dataset.value === String(option.value),
            );
          }
          reload({ keepPauseIntent: true });
        });
        group.appendChild(button);
      }
      wrap.appendChild(group);
    }

    if (knob.hint) {
      const hint = document.createElement("p");
      hint.className = "knob-hint";
      // 材質つまみだけ、ボタンに添えた摩擦係数の読み方を一言足す(既存の
      // 一言補足の隣に収まる分量——長い説明は増やさない)。
      hint.textContent =
        knob.id === "material"
          ? `${knob.hint}(摩擦の数字が小さいほどよく滑ります)`
          : knob.hint;
      wrap.appendChild(hint);
    }
    return wrap;
  }

  // ---- 毎フレームの更新 -------------------------------------------------------
  /// エディタ側の差し替え通知を受け取ったか(購読は物理側が立ち上がってから
  /// 一度だけ)。
  let sceneReplacedSubscribed = false;

  /** 揺れを均す指数移動平均の重み(`main.ts`の`updateEffectiveTimeScale`と同じ0.1)。 */
  const ACTUAL_RATE_SMOOTHING = 0.1;
  /**
   * 「実際に進んでいる速さ」を測って`#run-actual-rate`へ出す(`run-actual-rate`
   * のdoc参照——利用者役の報告「速さ×1と出ているのに実際は1/16の速さでしか
   * 進んでいない」への対応)。
   *
   * 走っていない・時間が戻った(やり直し等)ときは前回値との差分が意味を
   * 持たないので測らない。値が無ければ欄ごと隠す(何も無いほうが、意味の
   * 無い数字よりまし)。
   */
  function updateActualRate(api: WorkspaceApi, simSeconds: number): void {
    const nowMs = performance.now();
    const playing = api.isPlaying();
    if (!playing || lastActualRateWallMs === null || lastActualRateSimSeconds === null) {
      lastActualRateWallMs = nowMs;
      lastActualRateSimSeconds = simSeconds;
      if (!playing) {
        actualRate.hidden = true;
      }
      return;
    }
    const wallDelta = (nowMs - lastActualRateWallMs) / 1000;
    const simDelta = simSeconds - lastActualRateSimSeconds;
    lastActualRateWallMs = nowMs;
    lastActualRateSimSeconds = simSeconds;
    // 巻き戻し(やり直し・つまむと等)で simDelta が負になることがある——
    // その1フレームは測定を捨てる(次のフレームから測り直せば十分)。
    if (wallDelta <= 0 || simDelta < 0) return;
    const measured = simDelta / wallDelta;
    actualRateSmoothed =
      actualRateSmoothed === null
        ? measured
        : actualRateSmoothed + (measured - actualRateSmoothed) * ACTUAL_RATE_SMOOTHING;

    actualRate.hidden = false;
    const r = actualRateSmoothed;
    // **数字だけでなく、意味も添える**(利用者役の観察: 「実測」とだけ出ていても
    // 壊れているのか重いのか区別できない、との指摘は`main.ts`の
    // `updateEffectiveTimeScale`のdocに詳しい——ここでは中を知らない人向けに
    // さらに言葉を足す)。
    // **短い一行に留める**(利用者役の観察と同じ理由で長い注記をここへ足すと、
    // 左のパンくず「水を注ぐ」の表示幅を奪って省略記号だらけになる——
    // フルの言い分は`title`ツールチップ側(index.html)に書いてある)。
    if (r < 0.9) {
      actualRate.textContent = `実際は ×${r.toFixed(2)}(重い計算)`;
      actualRate.classList.add("slow");
    } else if (r > 1.1) {
      actualRate.textContent = `実際は ×${r.toFixed(2)}`;
      actualRate.classList.remove("slow");
    } else {
      actualRate.textContent = `実際もほぼ同じ(×${r.toFixed(2)})`;
      actualRate.classList.remove("slow");
    }
  }

  function tick(): void {
    const api = apiRef.current;
    if (api && !sceneReplacedSubscribed) {
      sceneReplacedSubscribed = true;
      // エディタ側で場面が差し替わったら(新規シーン・シーンギャラリー)、
      // 「いま見ている実験」を手放す。前の実験の名前と説明が残ったままだと、
      // 自分がどこにいるのか分からなくなる(利用者役④の観察)。
      api.onSceneReplaced(() => {
        current = null;
        ownSceneName = "";
        cardOverrides.clear();
        // **読み込み直後は何も選ばれていない状態から始める**(`reload`と
        // 同じ規則、そちらのdoc参照)。エディタ側(`sceneGalleryRef.current`)は
        // 内部都合で先頭のボディ(たいてい床)を選んだ状態でこの通知を送って
        // くるが、利用者から見れば自分では選んでいない——「選んだもの: ground」
        // の札が一瞬でも出ると、そこにある置き場所・向きの入力欄も一瞬だけ
        // 本物として存在してしまう。
        //
        // **これが実際に壊れていた**: 「＋新規シーン」の直後に物を1つ置く
        // 一連の操作(スポーン系の全テストがこの形)では、この選び直し漏れの
        // せいで「床が選ばれている」→「置いた物が選ばれている」という**本物の
        // 選択変更が2回連続で起きる**。遅い機械(macOS CIランナー)ではその
        // 間に床の札が実際に描画され、「向きも、数値で決められる」テストの
        // `#focus-rot-z`が**床の欄**を掴んでしまうことがあった——打った
        // 「30」は、そのすぐ後に置いた物へ選択が移る際に(本物の選択変更
        // なので正しく)捨てられ、新しく出てきた置いた物の欄(既定値 0)に
        // `change`が飛んで、向き0がそのまま world へ送られていた
        // (`workspace.spec.ts`の該当テストのdoc参照。実測: CPU 20倍速の
        // 抑制下で`selectionChange(-1→0=床)`→`selectionChange(0→1=箱)`の
        // 2段が観測され、床の欄が生きている間に打ち込みが割り込むと再現した)。
        // ここで先に選択を外しておけば、外から見える選択変更は「箱を選ぶ」の
        // 1回だけになり、床の欄はそもそも存在しない。
        //
        // **この「1回だけ」という前提そのものが、もう一段崩れていた**
        // (Windows CI, `workspace.spec.ts`「大きさの表示が、札と Inspector
        // で食い違わない」)。ここで直接呼ぶ`renderContext()`は選択なし
        // (-1)の札を組み、`lastSelection`はまだ前の選択(例: 前に置いた
        // 球の body index)を指したまま——ここで`tick()`が拾ってくれる
        // 前提で更新していなかった。「新規シーン」は毎回同じ並びで body
        // index を振り直すので、次に置く物(箱)が前の物(球)とまったく
        // 同じ index に着地することが普通に起こる。その間に`tick()`が
        // 一度も挟まらないと(遅い機械ほど起きやすい——CPU を20〜40倍
        // 絞った実測で再現)、`selected === lastSelection`のまま二度と
        // 更新されず、「選んだもの」の札が永久に組み直されなかった。
        // ここで`lastSelection`も直接合わせておけば、`tick()`が何回
        // 挟まろうと挟まるまいと、次に見るときの比較は必ず正しい。
        api.selectBody(-1);
        lastSelection = -1;
        api.setProbeLabels(null, null);
        api.setPace(null);
        // 実験を読み込むときと同じ規則(`reload`)。浅い粒度は「動いている
        // ところ」を見に来ているので走らせ、深い粒度は**置いてから動かす**
        // ので止めておく。前の場面の走行状態を引きずると、新しく作った場面が
        // 置いた端から転がっていく。
        if (detail < AUTORUN_BELOW) {
          api.play();
          api.followCamera(true);
        } else {
          api.stopForEditing();
          // **組み立てている間は、カメラを追いかけさせない**。追従カメラは
          // 「動くものを見る」ための道具で、置いた物へ画角を合わせようとする
          // エディタ側と毎フレーム引っ張り合いになる——置いた球が遠くの点に
          // しか見えなかったのはこれ(利用者役④の観察)。
          api.followCamera(false);
        }
        renderCrumbs();
        renderContext();
        syncRun();
      });
    }
    if (api && pendingStart) {
      pendingStart = false;
      reload();
      renderCrumbs();
      renderContext();
    }
    if (api && pendingOwnScene) {
      const scene = pendingOwnScene;
      pendingOwnScene = null;
      openSavedScene(scene);
    }
    if (api) {
      const seconds = api.time();
      const scale = api.stepSeconds();
      updateActualRate(api, seconds);
      const timeNode = document.getElementById("readout-time");
      if (timeNode) {
        timeNode.textContent = formatDuration(seconds, scale);
        timeNode.dataset.seconds = String(seconds);
      }
      // 自分で組み立てた場面でも、走っていれば時計は動く(以前は実験を
      // 選んでいるときしか出さず、「動いているのかどうか」が読めなかった)。
      clock.textContent =
        current || api.isPlaying() ? formatDuration(seconds, scale) : "";
      clock.dataset.seconds = String(seconds);

      // 「どこを見るか」は舞台の実際に従う。読み込んだ直後はまだ 1 フレームも
      // 描いていないので、ここで追いつかせる(カードを組み立てた時点では
      // 舞台が空かどうかまだ分からない)。
      // 読み込んだ直後の 1〜2 フレームは、まだ何も描いていないので**どの場面でも
      // 空に見える**。その一瞬を真に受けると、3D がちゃんと映る実験でも濃さが
      // 勝手に上がってしまい、「みる」に留まれなかった(実測で、選ぶ実験に
      // よって上がったり上がらなかったりした)。しばらく空のままのときだけ、
      // 本当に空だと決める。
      stageEmptyFrames = api.stageIsEmpty() ? stageEmptyFrames + 1 : 0;
      const stageEmptyNow = stageEmptyFrames > STAGE_EMPTY_FRAMES;
      if (stageEmptyNow !== lastStageEmpty) {
        lastStageEmpty = stageEmptyNow;
        // 舞台に形のある物が**出てこない**と分かったときだけ、グラフの段を
        // 強制的に開く——「選んだのに何も映らない」を残さないため。実験の表に
        // ついた `view` ではなく舞台の実際で決めるので、3D に何か映る実験で
        // 勝手に開くことはない(利用者役①の観察)。
        //
        // 課題A(利用者役の報告): 以前はここで大局の粒度そのもの
        // (`ANALYSIS_READABLE` へ `applyDetail`)を上げていたが、それだと
        // 「変えてみる」(つまみ)まで一緒に開いてしまい、「みる」を選んだ
        // 人の画面が道具だらけになった。`forceAnalysisOpen` はグラフの段
        // だけを開く旗なので、`detail`/`chosenDetail`/ダイヤルの位置には
        // 触れない——「みる」を選んだ人は「みる」のまま、現象(グラフ)
        // だけが追いついて出る。
        //
        // 「場」の実験(二重スリットなど)は、形のある物こそ無いものの、
        // **3D の中の場のパネルに絵が出ている**。見に行く先がそこにある以上、
        // グラフを強制して出す必要はない。
        forceAnalysisOpen = stageEmptyNow && current?.view !== "field";
        applyDetail(detail, false, false);
      }

      // 文面は濃さと `forceAnalysisOpen` でも変わる(`stageWhereText` の
      // doc)ので、舞台の空き具合だけでなく**文そのもの**が変わったかで
      // 書き直す。
      const whereNow = current ? stageWhereText(current, stageEmptyNow) : "";
      if (whereNow !== lastWhereText) {
        lastWhereText = whereNow;
        const where = document.querySelector<HTMLElement>(".card-where");
        if (where) where.textContent = whereNow;
      }

      // **もう何も動いていない**ことを見つけて、そのときの時刻を出す
      // (`settledAt` のdoc参照)。判定は「いちばん速い物の速さ」だけを見る
      // ——止まり続けた場面(振り子・惑星)では永久に出ないし、そもそも
      // 動く物が無い場面(熱・量子)でも出ない。
      if (api.isPlaying()) {
        const fastest = api.maxSpeed();
        if (fastest > SETTLED_SPEED * 4) {
          everMoved = true;
          stillFrames = 0;
          settledAt = null;
        } else if (everMoved && fastest < SETTLED_SPEED) {
          stillFrames += 1;
          // 一瞬の静止(跳ね返りの頂点、衝突の瞬間)を「止まった」と
          // 読まないだけの猶予を置く。
          if (stillFrames > 30 && settledAt === null) settledAt = seconds;
        } else {
          stillFrames = 0;
        }
      }
      const settledKey = document.getElementById("readout-settled-key");
      const settledNode = document.getElementById("readout-settled");
      if (settledKey && settledNode) {
        const show = settledAt !== null;
        settledKey.hidden = !show;
        settledNode.hidden = !show;
        if (show) {
          settledNode.textContent = formatDuration(settledAt as number, scale);
          settledNode.dataset.seconds = String(settledAt);
        }
      }

      if (current) {
        const count = api.probeCount();
        for (const { readout, node } of readoutNodes) {
          const sources = readout.probes ?? [readout.probe];
          if (sources.some((i) => i >= count)) {
            node.textContent = "—";
            continue;
          }
          const values = sources.map((i) => api.probeValue(i));
          const value = readout.derive ? readout.derive(values) : values[0];
          // **その量にとって「ほぼ 0」なら、0 と書く**。桁の離れた量を指数で
          // 書くようにしたら、こんどは止まりかけた箱の速さが「8.67e-19 m/s」と
          // 出るようになった——普通の人には壊れて見える(利用者役②の観察)。
          // その量がこれまでに取った大きさと比べて無視できるほど小さいときだけ
          // 0 に丸める。インクの広がり(常に 1e-17 前後)のように、量そのものが
          // 小さい系列は指数のまま残る。
          if (Number.isFinite(value)) {
            const seen = Math.max(readoutSeenMax.get(node) ?? 0, Math.abs(value));
            readoutSeenMax.set(node, seen);
          }
          const negligible =
            Number.isFinite(value) &&
            value !== 0 &&
            Math.abs(value) < (readoutSeenMax.get(node) ?? 0) * 1e-6;
          const shown = negligible ? 0 : value;
          // **無くなった物の値は、数字として出さない**。融け切った氷のように
          // 場から退いた剛体は、内部では遠く(-1e9 m)へ退避させられる。その値が
          // そのまま「氷の高さ -1,000,000,000.000 m」と出て、壊れたとしか読め
          // なかった(利用者役②の観察)。退避先の桁に達した値は、そう言う。
          node.textContent = !Number.isFinite(value)
            ? "—"
            : Math.abs(value - RETIRED_BODY_Y) <= RETIRED_BODY_TOLERANCE
              ? "もう在りません"
              : readout.format
                ? readout.format(shown)
                : `${readoutNumber(shown, readout.digits ?? 2)}${readout.unit ? ` ${readout.unit}` : ""}`;
        }
      }

      // 選択が変わったら、パンくずとカードを組み直す(局所へ入った/出た)。
      const selected = api.selectedBody();
      if (selected !== lastSelection) {
        lastSelection = selected;
        // 別の物を選んだら、前の物へ打ちかけていた値は捨てる。
        focusPositionDraft = [null, null, null];
        focusPositionPending = [null, null, null];
        focusRotationDraft = [null, null, null];
        focusRotationPending = [null, null, null];
        renderCrumbs();
        renderContext();
        // **選んだものの札は、選んだ瞬間に見えているべき**。カードが増えると
        // 一番下へ回るので、画面の高さによっては選んでも何も起きていないように
        // 見えた(利用者役③の観察: 900px では「選んだもの」欄が見えない)。
        if (selected >= 0) {
          document
            .querySelector('.card[data-card="focus"]')
            ?.scrollIntoView({ block: "nearest" });
        }
      } else if (selected >= 0 && Object.keys(focusNodes).length > 0) {
        const readout = api.bodyReadout(selected);
        if (readout) {
          if (focusNodes["高さ"]) {
            focusNodes["高さ"].textContent = `${readout.position[1].toFixed(3)} m`;
          }
          if (focusNodes["速さ"]) {
            focusNodes["速さ"].textContent = `${readout.speed.toFixed(3)} m/s`;
          }
          // かたちと重さも毎回書き直す。大きさを変えた直後、この札だけ古い
          // 重さのままで、選び直すまで更新されなかった(利用者役④の観察)。
          if (focusNodes["かたち"]) {
            focusNodes["かたち"].textContent = friendlyShape(readout.shape);
          }
          if (focusNodes["重さ"]) {
            focusNodes["重さ"].textContent = `${readout.mass.toFixed(3)} kg`;
          }
          // 置き場所の欄と材質の選びも、実際の値へ揃え直す(材質を変えた
          // 直後にこの札だけ前の材質を出していた——利用者役④の観察)。
          for (const [i, input] of focusPositionInputs.entries()) {
            if (document.activeElement === input) continue;
            const actual = readout.position[i];
            const pending = focusPositionPending[i];
            if (pending !== null) {
              // 物理側が追いつくまでは、頼んだ値を出したままにする。追いつかない
              // ときのために時間で切る——動く物なら、そのあとは実際の位置が
              // そのまま出るので、欄が固まったままになることはない。
              if (
                Math.abs(actual - pending.want) < 1e-6 ||
                performance.now() > pending.until
              ) {
                focusPositionPending[i] = null;
              } else {
                continue;
              }
            }
            const next = actual.toFixed(3);
            if (input.value !== next) input.value = next;
          }
          // 向きの欄も、置き場所とまったく同じ規則で揃える。
          for (const [i, input] of focusRotationInputs.entries()) {
            if (document.activeElement === input) continue;
            const actual = readout.rotation[i];
            const pending = focusRotationPending[i];
            if (pending !== null) {
              if (
                Math.abs(actual - pending.want) < 1e-3 ||
                performance.now() > pending.until
              ) {
                focusRotationPending[i] = null;
              } else {
                continue;
              }
            }
            const next = actual.toFixed(1);
            if (input.value !== next) input.value = next;
          }
          const materialSelect = document.getElementById(
            "focus-material",
          ) as HTMLSelectElement | null;
          if (
            materialSelect &&
            document.activeElement !== materialSelect &&
            materialSelect.value !== readout.material
          ) {
            materialSelect.value = readout.material;
          }
        }
      }

      if (playButton.dataset.playing !== String(api.isPlaying())) syncRun();
    }
    requestAnimationFrame(tick);
  }

  // ---- 起動 -------------------------------------------------------------------
  applyDetail(detail, false, false);
  renderCrumbs();
  renderContext();
  syncRun();
  requestAnimationFrame(tick);

  // **起動時に何を見せるか**は粒度で決まる。
  //
  // 浅い粒度(みる/さわる/しらべる)で開いた人は「何かが起きているところ」を
  // 見に来ている——空の画面や名前のない箱ではなく、名前と見どころのある現象を
  // 載せて走らせる(前回の続きがあればそれを)。
  //
  // 深い粒度(つくる)で開いた人は**自分の作業机に戻ってきた**人なので、
  // 勝手に別の世界へ差し替えない。同じ操作でも、粒度によって正しい既定が違う。
  if (detail < REVEAL.toolbar) {
    const remembered = (() => {
      try {
        const id = localStorage.getItem(LAST_EXPERIMENT_KEY);
        return id ? findExperiment(id) : undefined;
      } catch {
        return undefined;
      }
    })();
    start(remembered ?? CATEGORIES[0].experiments[0]);
  } else {
    // 「つくる」で開いた人は**自分の作業机に戻ってきた**人。前に開いていた
    // 自分の場面があれば、そこへ戻す——更新しただけで作ったものが消え、
    // 見覚えのない別の世界に置き換わるのが、いちばん堪える体験だった
    // (利用者役④の一番の不満)。
    const last = (() => {
      try {
        return localStorage.getItem(LAST_OWN_SCENE_KEY);
      } catch {
        return null;
      }
    })();
    const saved = last ? readSavedScenes().find((s) => s.name === last) : undefined;
    // 物理側はまだ立ち上がっていないかもしれないので、`tick` に開かせる。
    if (saved) pendingOwnScene = saved;
  }
}
