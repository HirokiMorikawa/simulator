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
   * グラフの凡例・目盛りに出す表示名と単位(プローブ番号 → 人間の言葉)。
   * 単位が分かる系列だけ `units` に入れる(無ければ数だけを書く)。
   */
  setProbeLabels: (
    labels: Record<number, string> | null,
    units?: Record<number, string> | null,
  ) => void;
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
    speed: number;
  } | null;
};

export type WorkspaceApiRef = { current: WorkspaceApi | null };

const DETAIL_KEY = "simulator.ui.detail";
const LAST_EXPERIMENT_KEY = "simulator.ui.last-experiment";

/** 大局の粒度の目盛り。連続値だが、名前が付く位置がある。 */
const GRAIN_STOPS = [
  { at: 0, key: "watch", label: "みる", hint: "現象だけを大きく見る" },
  { at: 1, key: "touch", label: "さわる", hint: "つまみを動かして試す" },
  { at: 2, key: "study", label: "しらべる", hint: "数値・グラフ・一覧で確かめる" },
  { at: 3, key: "build", label: "つくる", hint: "自分で組み立てる(全機能)" },
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
export function formatDuration(seconds: number, scale = 1): string {
  const t = Math.abs(seconds);
  // **単位はシーンの時間スケールで決める**。値そのもので切り替えると、同じ
  // 実験の途中で「958.33 ミリ秒 → 1.02 秒」と桁も単位も飛んで読みにくい
  // (利用者役の観察)。人の尺度で進むシーン(1 step が 1e-4 秒より粗い)は
  // 常に秒より上の単位で書き、分子や公転のように桁が離れたシーンだけ
  // それぞれの単位系に入る。
  if (t === 0) return "0 秒";
  if (scale >= 1e-4) {
    if (t < 60) return `${seconds.toFixed(2)} 秒`;
    if (t < 3600) return `${(seconds / 60).toFixed(2)} 分`;
    if (t < 86400) return `${(seconds / 3600).toFixed(2)} 時間`;
    if (t < 3.155e7) return `${(seconds / 86400).toFixed(2)} 日`;
    return `${(seconds / 3.155e7).toFixed(2)} 年`;
  }
  if (t < 1e-9) return `${(seconds * 1e12).toFixed(2)} ピコ秒`;
  if (t < 1e-6) return `${(seconds * 1e9).toFixed(2)} ナノ秒`;
  if (t < 1e-3) return `${(seconds * 1e6).toFixed(2)} マイクロ秒`;
  if (t < 1) return `${(seconds * 1e3).toFixed(2)} ミリ秒`;
  if (t < 60) return `${seconds.toFixed(2)} 秒`;
  if (t < 3600) return `${(seconds / 60).toFixed(2)} 分`;
  if (t < 86400) return `${(seconds / 3600).toFixed(2)} 時間`;
  if (t < 3.155e7) return `${(seconds / 86400).toFixed(2)} 日`;
  return `${(seconds / 3.155e7).toFixed(2)} 年`;
}

/** `Plane(normal=(0,1,0), d=0)` のような内部表記を、ひと目で分かる名前にする。 */
function friendlyShape(raw: string): string {
  const head = raw.split("(")[0].trim().toLowerCase();
  const names: Record<string, string> = {
    plane: "ゆか(平面)",
    sphere: "球",
    box: "箱",
    capsule: "カプセル",
    compound: "組み合わせ",
    convexmesh: "凸包メッシュ",
    convex_mesh: "凸包メッシュ",
  };
  return names[head] ?? raw;
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
  const speedGroup = el<HTMLDivElement>("run-speed");
  const commandbar = el<HTMLElement>("commandbar");

  // ---- 状態 -----------------------------------------------------------------
  let detail = readStoredDetail();
  let current: Experiment | null = null;
  let knobValues: Record<string, string | number> = {};
  let speedMultiplier = 1;
  let filterCategory: string | null = null;
  let paletteIndex = 0;
  let paletteMatches: Experiment[] = [];
  let pendingStart = false;
  let lastSelection = -1;
  /** カードごとの局所的な開閉。`undefined` = 大局の粒度に従う。 */
  const cardOverrides = new Map<string, boolean>();

  // ---- 大局の粒度 -------------------------------------------------------------
  /**
   * `from` を超えたところから `span` かけて 0 → `to` まで伸びる寸法 [px]。
   * 閾値で「パッと出る」のではなく、粒度に連れて**育つ**ようにするための補間。
   */
  function grow(value: number, from: number, span: number, to: number): number {
    const t = Math.min(1, Math.max(0, (value - from) / span));
    return Math.round(t * to);
  }

  function applyDetail(next: number, persist = true): void {
    detail = Math.min(3, Math.max(0, next));
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
    let analysis =
      detail >= REVEAL.analysis
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
    dialHint.textContent = nearestStop(detail).hint;
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
  window.addEventListener("resize", () => applyDetail(detail, false));
  // Project ドロワー(素材・回路・リプレイ)の開閉は、粒度とは別の局所的な
  // 操作。開いたら行の高さを与え直す必要があるので、属性の変化を見る。
  new MutationObserver(() => applyDetail(detail, false)).observe(app, {
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

  function matches(): Experiment[] {
    const query = paletteInput.value.trim().toLowerCase();
    const result: Experiment[] = [];
    for (const category of CATEGORIES) {
      if (filterCategory && category.id !== filterCategory) continue;
      for (const experiment of category.experiments) {
        if (!query) {
          result.push(experiment);
          continue;
        }
        const haystack =
          `${experiment.title} ${experiment.blurb} ${category.title} ` +
          `${experiment.watch.join(" ")} ${experiment.id}`.toLowerCase();
        if (haystack.includes(query)) result.push(experiment);
      }
    }
    return result;
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
    paletteMatches.forEach((experiment, index) => {
      const category = categoryOf(experiment);
      const row = document.createElement("button");
      row.type = "button";
      row.className = "palette-row";
      row.dataset.experimentId = experiment.id;
      row.dataset.active = String(index === paletteIndex);
      row.innerHTML =
        `<span class="palette-row-icon">${experiment.icon}</span>` +
        `<span class="palette-row-main">` +
        `<span class="palette-row-title">${experiment.title}</span>` +
        `<span class="palette-row-blurb">${experiment.blurb}</span>` +
        `</span>` +
        `<span class="palette-row-tag">${category?.icon ?? ""} ${category?.title ?? ""}</span>`;
      row.addEventListener("click", () => start(experiment));
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
      const experiment = paletteMatches[paletteIndex];
      if (experiment) start(experiment);
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
    const labels: Record<number, string> = { ...(experiment.series ?? {}) };
    for (const readout of experiment.readouts ?? []) {
      if (readout.derive) continue;
      labels[readout.probe] = readout.label;
    }
    return labels;
  }

  /** グラフの目盛りに添える単位。分かっているものだけ。 */
  function probeUnitsFor(experiment: Experiment): Record<number, string> {
    const units: Record<number, string> = {};
    for (const readout of experiment.readouts ?? []) {
      if (readout.derive || !readout.unit) continue;
      units[readout.probe] = readout.unit;
    }
    return units;
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
    // 3D に何も描かれない実験では、グラフが見える粒度まで自動的に開く——
    // 「選んだのに何も映らない」を残さないため。粒度を**下げる**ことはしない
    // (人が選んだ濃さを勝手に薄くしない)。
    if (experiment.view !== "3d" && detail < ANALYSIS_READABLE) {
      applyDetail(ANALYSIS_READABLE);
    }
    reload();
    renderCrumbs();
    renderContext();
  }

  /** いまのつまみの値でシーンを作り直し、頭から走らせる。 */
  function reload(): void {
    const api = apiRef.current;
    if (!current) return;
    if (!api) {
      pendingStart = true;
      return;
    }
    const json = sceneJsonFor(current);
    if (!json) return;
    api.loadSceneJson(json);
    api.setTethers(tethersOf(JSON.parse(json) as SceneJson));
    // 読み込み直後は**何も選ばれていない**状態から始める。エディタ側は内部
    // 都合で先頭のボディ(たいてい床)を選ぶが、利用者から見れば自分では
    // 選んでいない——「選んだもの: ground / 重さ 0.000 kg」が出てくるのは
    // ノイズでしかない。選択は人が対象をクリックしたときだけ起きる。
    api.selectBody(-1);
    api.setProbeLabels(probeLabelsFor(current), probeUnitsFor(current));
    api.setPace(current.pace * speedMultiplier);
    api.followCamera(true);
    if (detail < AUTORUN_BELOW) api.play();
    else api.stopForEditing();
    syncRun();
  }

  // ---- 走行コントロール --------------------------------------------------------
  playButton.addEventListener("click", () => {
    const api = apiRef.current;
    if (!api) return;
    if (!current) {
      openPalette();
      return;
    }
    if (api.isPlaying()) api.pause();
    else api.play();
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
    restartButton.disabled = !current;
    for (const button of speedGroup.querySelectorAll("button")) {
      button.classList.toggle(
        "active",
        Number((button as HTMLElement).dataset.speed) === speedMultiplier,
      );
    }
    // 速さは**数として**も出す。🐢/🐇 のボタンが凹んだことは分かっても、
    // どれくらい変わったのかは絵からは読み取れない(利用者役の観察)。
    rate.textContent = `×${speedMultiplier}`;
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
  let focusNodes: Record<string, HTMLElement> = {};

  function renderContext(): void {
    contextBody.innerHTML = "";
    readoutNodes = [];
    focusNodes = {};

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
            value.textContent = "0 秒";
            list.append(key, value);
            body.appendChild(list);
          },
        },
      ];
      for (const spec of world) contextBody.appendChild(buildCard(spec));
      appendFocusCard();
      syncCards();
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
        where.textContent =
          experiment.view === "3d"
            ? "👀 まん中の 3D を見てください。"
            : experiment.view === "field"
              ? "👀 3D の中に出る「場」のパネルに、波や分布が描かれます。"
              : "📈 下のグラフがいちばん分かりやすい場所です。";
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
        timeValue.textContent = "0 秒";
        list.append(timeKey, timeValue);
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
            reload();
            renderContext();
          });
          body.appendChild(reset);
        },
      });
    }

    for (const spec of specs) contextBody.appendChild(buildCard(spec));
    appendFocusCard();
    contextBody.appendChild(buildCard(viewCardSpec()));
    syncCards();
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
              ["材質", readout.material],
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
            clear.addEventListener("click", () => {
              api.selectBody(-1);
              renderCrumbs();
              renderContext();
            });
            actions.append(follow, clear);
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
      input.value = String(knobValues[knob.id] ?? knob.value);
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
      input.addEventListener("change", () => reload());
      row.append(input, output);
      wrap.appendChild(row);
    } else {
      const group = document.createElement("div");
      group.className = "knob-choice";
      for (const option of knob.options ?? []) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "knob-choice-btn";
        button.textContent = option.label;
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
          reload();
        });
        group.appendChild(button);
      }
      wrap.appendChild(group);
    }

    if (knob.hint) {
      const hint = document.createElement("p");
      hint.className = "knob-hint";
      hint.textContent = knob.hint;
      wrap.appendChild(hint);
    }
    return wrap;
  }

  // ---- 毎フレームの更新 -------------------------------------------------------
  function tick(): void {
    const api = apiRef.current;
    if (api && pendingStart) {
      pendingStart = false;
      reload();
      renderCrumbs();
      renderContext();
    }
    if (api) {
      const seconds = api.time();
      const scale = api.stepSeconds();
      const timeNode = document.getElementById("readout-time");
      if (timeNode) {
        timeNode.textContent = formatDuration(seconds, scale);
        timeNode.dataset.seconds = String(seconds);
      }
      clock.textContent = current ? formatDuration(seconds, scale) : "";
      clock.dataset.seconds = String(seconds);

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
          node.textContent = readout.format
            ? readout.format(value)
            : `${value.toFixed(readout.digits ?? 2)}${readout.unit ? ` ${readout.unit}` : ""}`;
        }
      }

      // 選択が変わったら、パンくずとカードを組み直す(局所へ入った/出た)。
      const selected = api.selectedBody();
      if (selected !== lastSelection) {
        lastSelection = selected;
        renderCrumbs();
        renderContext();
      } else if (selected >= 0 && Object.keys(focusNodes).length > 0) {
        const readout = api.bodyReadout(selected);
        if (readout) {
          if (focusNodes["高さ"]) {
            focusNodes["高さ"].textContent = `${readout.position[1].toFixed(3)} m`;
          }
          if (focusNodes["速さ"]) {
            focusNodes["速さ"].textContent = `${readout.speed.toFixed(3)} m/s`;
          }
        }
      }

      if (playButton.dataset.playing !== String(api.isPlaying())) syncRun();
    }
    requestAnimationFrame(tick);
  }

  // ---- 起動 -------------------------------------------------------------------
  applyDetail(detail, false);
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
  }
}
