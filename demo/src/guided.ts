// かんたんモード(3 ステップ)の UI。
//
// **狙い**: 中の仕組み(Edit/Play モード、Probe、Coupling、dt、時間倍率、
// シーン JSON)を一切知らない人が、**3 手で見たい現象に到達できる**こと。
//   ① なにを見たい?  — カテゴリを選ぶ
//   ② どれにする?    — 実験を選ぶ
//   ③ うごかす       — 大きなボタンを 1 回押す
// 押した瞬間にシーンが読み込まれ、その現象に合った速さで再生が始まり、
// 「どこを見ればいいか」「何が起きたら成功か」「いまの数値」が横に出る。
//
// **なぜ別モードにしたか**(既存エディタに機能を足すのではなく): 統合エディタは
// Unity 風の 6 パネル構成で、目的は「作る人が細部まで詰められること」。それは
// 残したまま、**最初に見る画面だけを入れ替える**のがいちばん副作用が小さい。
// 切り替えは相互に 1 クリック(`data-ui` 属性 1 つ)で、選択は localStorage に
// 残るので、作る人が毎回かんたんモードを通らされることもない。
//
// このモジュールは既存の `main.ts` の内部状態には触らず、`GuidedApi`(main.ts が
// 埋める小さな窓口)だけを通して操作する。

import {
  GUIDED_CATEGORIES,
  findCategory,
  findExperiment,
  type Category,
  type Experiment,
  type Knob,
  type SceneJson,
} from "./guided-catalog";

/**
 * かんたんモードが `main.ts` へ求めることの全部。
 *
 * 意図的にこれだけに絞ってある——「シーンを読む/進める/止める/いまの数値を読む」。
 * ここが太るとエディタ本体とかんたんモードが絡まって、どちらも直せなくなる。
 */
export type GuidedApi = {
  /** シーン JSON を読み込んで世界を作り直す(t=0 から)。 */
  loadSceneJson: (json: string) => void;
  /** 再生を始める(Play モードへ入る)。 */
  play: () => void;
  /** 一時停止する(状態は保つ)。 */
  pause: () => void;
  isPlaying: () => boolean;
  /**
   * 1 秒あたりに進める step 数。`null` で従来の「時間倍率」方式へ戻す。
   * シーンごとに dt が 16 桁も違うため、倍率ではなく step 数で指定する
   * (`guided-catalog.ts` 冒頭の doc 参照)。
   */
  setPace: (stepsPerSecond: number | null) => void;
  /** シーン JSON の `probes` の本数。 */
  probeCount: () => number;
  /** i 番目のプローブの現在値。 */
  probeValue: (index: number) => number;
  /** シミュレーション内の経過時間 [s]。 */
  time: () => number;
  /**
   * 追従カメラの入り切り。統合エディタへ切り替えるときは必ず切る——
   * 切らないと、エディタで自分が合わせた画角を毎フレーム上書きしてしまう。
   */
  followCamera: (enabled: boolean) => void;
  /** グラフの凡例に出す表示名(プローブ番号 → 人間の言葉)。 */
  setProbeLabels: (labels: Record<number, string> | null) => void;
};

export type GuidedApiRef = { current: GuidedApi | null };

const UI_MODE_KEY = "simulator.ui.mode";
const LAST_EXPERIMENT_KEY = "simulator.guided.last";

// シーン JSON は `scenes/` をそのまま束ねる(エディタのギャラリーと同じ実体を
// 読む——ここだけ別のコピーを持つと、片方だけ直った不整合が必ず起きる)。
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
 * **なぜ必要か**: シーンの時間スケールは 1e-12 秒(気体分子の衝突)から 1e9 秒
 * (惑星の公転)まで 21 桁にわたる。秒で固定表示すると、気体の箱は永遠に
 * 「0.00 秒」のまま(走っているのか固まっているのか分からない)、太陽系は
 * 「31554896.93 秒」(何年なのか読み取れない)になる。どちらも
 * **中を知らない人にとっては「壊れている」と区別が付かない**。
 */
function formatDuration(seconds: number): string {
  const t = Math.abs(seconds);
  if (t === 0) return "0 秒";
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

function el<T extends HTMLElement>(id: string): T {
  return document.getElementById(id) as T;
}

/** 現在の UI モード。未設定なら「かんたん」——初めて開く人が多数派だから。 */
function storedUiMode(): "guided" | "pro" {
  try {
    return localStorage.getItem(UI_MODE_KEY) === "pro" ? "pro" : "guided";
  } catch {
    return "guided";
  }
}

function storeUiMode(mode: "guided" | "pro"): void {
  try {
    localStorage.setItem(UI_MODE_KEY, mode);
  } catch {
    /* プライベートモード等では黙って諦める(機能の本質ではない) */
  }
}

export function setUpGuidedMode(apiRef: GuidedApiRef): void {
  const app = el<HTMLDivElement>("app");
  const chooser = el<HTMLDivElement>("guided-chooser");
  const chooserBody = el<HTMLDivElement>("guided-chooser-body");
  const chooserSteps = el<HTMLDivElement>("guided-steps");
  const guidedTitle = el<HTMLDivElement>("guided-title");
  const guidedBlurb = el<HTMLDivElement>("guided-blurb");
  const guidedPanelBody = el<HTMLDivElement>("guided-panel-body");
  const playButton = el<HTMLButtonElement>("btn-guided-play");
  const restartButton = el<HTMLButtonElement>("btn-guided-restart");
  const chooseButton = el<HTMLButtonElement>("btn-guided-choose");
  const graphButton = el<HTMLButtonElement>("btn-guided-graph");
  const cameraButton = el<HTMLButtonElement>("btn-guided-camera");
  const proButton = el<HTMLButtonElement>("btn-guided-pro");
  const simpleButton = el<HTMLButtonElement>("btn-simple-mode");
  const speedGroup = el<HTMLDivElement>("guided-speed");

  // ---- 状態(かんたんモードが持つのはこれだけ) -----------------------------
  let step: 1 | 2 | 3 = 1;
  let categoryId: string | null = null;
  let pendingExperiment: Experiment | null = null;
  let current: Experiment | null = null;
  let knobValues: Record<string, string | number> = {};
  let speedMultiplier = 1;
  let showGraph = false;
  // wasm の初期化が終わる前に実験が選ばれたときの持ち越し。窓口
  // (`apiRef.current`)が埋まった最初の tick で走り出す。
  let pendingStart = false;

  // ---- モード切替 -----------------------------------------------------------
  function applyUiMode(mode: "guided" | "pro"): void {
    app.dataset.ui = mode;
    storeUiMode(mode);
    // Scene View の器の大きさが変わるので、three.js のキャンバスを追従させる
    // (`main.ts` の `resize` は window の resize でしか呼ばれない)。
    requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
  }

  proButton.addEventListener("click", () => {
    applyUiMode("pro");
    closeChooser();
    // かんたんモード側の「乗っ取り」を全部返す——進める速さは統合エディタの
    // 時間倍率へ、グラフの凡例は Rust 側の生ラベルへ、カメラは手動へ。
    // 返さないと、エディタで時間倍率を変えても効かない/自分で合わせた画角が
    // 毎フレーム戻される、という説明のつかない挙動になる。
    apiRef.current?.setPace(null);
    apiRef.current?.setProbeLabels(null);
    apiRef.current?.followCamera(false);
  });
  simpleButton?.addEventListener("click", () => {
    applyUiMode("guided");
    const api = apiRef.current;
    if (api && current) {
      api.setPace(current.pace * speedMultiplier);
      api.setProbeLabels(probeLabelsFor(current));
      api.followCamera(true);
    }
    if (!current) openChooser(1);
  });

  // ---- ステップ表示 ---------------------------------------------------------
  const STEP_LABELS = ["なにを見たい?", "どれにする?", "うごかす"];

  function renderSteps(): void {
    chooserSteps.innerHTML = "";
    STEP_LABELS.forEach((label, index) => {
      const n = index + 1;
      const item = document.createElement("button");
      item.type = "button";
      item.className = "guided-step";
      item.dataset.state = n === step ? "current" : n < step ? "done" : "todo";
      item.dataset.step = String(n);
      item.innerHTML =
        `<span class="guided-step-number">${n}</span>` +
        `<span class="guided-step-label">${label}</span>`;
      // 済んだステップは押して戻れる(選び直しは「やり直し」ではなく普通の操作)。
      item.disabled = n > step;
      item.addEventListener("click", () => {
        if (n === 1) openChooser(1);
        else if (n === 2 && categoryId) openChooser(2);
        else if (n === 3 && pendingExperiment) openChooser(3);
      });
      chooserSteps.appendChild(item);
    });
  }

  // ---- ① カテゴリ ----------------------------------------------------------
  function renderCategories(): void {
    chooserBody.innerHTML = "";
    const grid = document.createElement("div");
    grid.className = "guided-card-grid";
    for (const category of GUIDED_CATEGORIES) {
      const card = document.createElement("button");
      card.type = "button";
      card.className = "guided-card";
      card.dataset.categoryId = category.id;
      card.innerHTML =
        `<span class="guided-card-icon">${category.icon}</span>` +
        `<span class="guided-card-title">${category.title}</span>` +
        `<span class="guided-card-blurb">${category.blurb}</span>` +
        `<span class="guided-card-count">${category.experiments.length} つの実験</span>`;
      card.addEventListener("click", () => {
        categoryId = category.id;
        openChooser(2);
      });
      grid.appendChild(card);
    }
    chooserBody.appendChild(grid);
  }

  // ---- ② 実験 --------------------------------------------------------------
  function renderExperiments(category: Category): void {
    chooserBody.innerHTML = "";
    const back = document.createElement("button");
    back.type = "button";
    back.className = "guided-back";
    back.textContent = "← ほかのカテゴリ";
    back.addEventListener("click", () => openChooser(1));
    chooserBody.appendChild(back);

    const grid = document.createElement("div");
    grid.className = "guided-card-grid";
    for (const experiment of category.experiments) {
      const card = document.createElement("button");
      card.type = "button";
      card.className = "guided-card";
      card.dataset.experimentId = experiment.id;
      card.innerHTML =
        `<span class="guided-card-icon">${experiment.icon}</span>` +
        `<span class="guided-card-title">${experiment.title}</span>` +
        `<span class="guided-card-blurb">${experiment.blurb}</span>`;
      card.addEventListener("click", () => {
        pendingExperiment = experiment;
        knobValues = defaultKnobValues(experiment);
        openChooser(3);
      });
      grid.appendChild(card);
    }
    chooserBody.appendChild(grid);
  }

  // ---- ③ うごかす -----------------------------------------------------------
  function renderConfirm(experiment: Experiment): void {
    chooserBody.innerHTML = "";
    const back = document.createElement("button");
    back.type = "button";
    back.className = "guided-back";
    back.textContent = "← ほかの実験";
    back.addEventListener("click", () => openChooser(2));
    chooserBody.appendChild(back);

    const card = document.createElement("div");
    card.className = "guided-confirm";
    card.innerHTML =
      `<div class="guided-confirm-head">` +
      `<span class="guided-confirm-icon">${experiment.icon}</span>` +
      `<div><h3>${experiment.title}</h3><p>${experiment.blurb}</p></div>` +
      `</div>` +
      `<h4>ここを見てください</h4>` +
      `<ul class="guided-watch">${experiment.watch
        .map((line) => `<li>${line}</li>`)
        .join("")}</ul>`;
    chooserBody.appendChild(card);

    if (experiment.knobs?.length) {
      const knobs = document.createElement("div");
      knobs.className = "guided-knobs";
      knobs.innerHTML = "<h4>先に変えておく(あとでも変えられます)</h4>";
      for (const knob of experiment.knobs) {
        knobs.appendChild(renderKnob(knob, false, "guided-pick"));
      }
      chooserBody.appendChild(knobs);
    }

    const go = document.createElement("button");
    go.type = "button";
    go.id = "btn-guided-start";
    go.className = "guided-go";
    go.textContent = "▶ うごかす";
    go.addEventListener("click", () => {
      startExperiment(experiment);
    });
    chooserBody.appendChild(go);
    // ③ に来た時点でここが唯一の主役なので、キーボードだけでも Enter で走れる。
    requestAnimationFrame(() => go.focus());
  }

  function defaultKnobValues(
    experiment: Experiment,
  ): Record<string, string | number> {
    const values: Record<string, string | number> = {};
    for (const knob of experiment.knobs ?? []) values[knob.id] = knob.value;
    return values;
  }

  /**
   * つまみ 1 個の UI。`live` なら変更が即座にシーンへ反映される。
   *
   * `idPrefix` を分けているのは、③の確認画面(チューザ)と右のガイドパネルに
   * **同じつまみが同時に存在する**ため——同じ id を 2 箇所に書くと、`<label for>`
   * が片方にしか効かない(クリックしても反応しないつまみが生まれる)。
   */
  function renderKnob(knob: Knob, live: boolean, idPrefix: string): HTMLElement {
    const wrap = document.createElement("div");
    wrap.className = "guided-knob";
    wrap.dataset.knobId = knob.id;

    const label = document.createElement("label");
    label.className = "guided-knob-label";
    label.textContent = knob.label;
    wrap.appendChild(label);

    if (knob.kind === "range") {
      const row = document.createElement("div");
      row.className = "guided-knob-row";
      const input = document.createElement("input");
      input.type = "range";
      input.min = String(knob.min ?? 0);
      input.max = String(knob.max ?? 10);
      input.step = String(knob.step ?? 1);
      input.value = String(knobValues[knob.id] ?? knob.value);
      input.id = `${idPrefix}-${knob.id}`;
      label.htmlFor = input.id;
      const readout = document.createElement("output");
      readout.className = "guided-knob-value";
      const paint = () => {
        readout.textContent = `${input.value}${knob.unit ? ` ${knob.unit}` : ""}`;
      };
      paint();
      input.addEventListener("input", () => {
        knobValues[knob.id] = Number(input.value);
        paint();
      });
      if (live) input.addEventListener("change", () => reloadCurrent());
      row.appendChild(input);
      row.appendChild(readout);
      wrap.appendChild(row);
    } else {
      const group = document.createElement("div");
      group.className = "guided-choice";
      for (const option of knob.options ?? []) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "guided-choice-btn";
        button.textContent = option.label;
        button.dataset.value = String(option.value);
        const sync = () => {
          button.classList.toggle(
            "active",
            String(knobValues[knob.id]) === String(option.value),
          );
        };
        sync();
        button.addEventListener("click", () => {
          knobValues[knob.id] = option.value;
          for (const sibling of group.querySelectorAll(".guided-choice-btn")) {
            sibling.classList.toggle(
              "active",
              (sibling as HTMLElement).dataset.value === String(option.value),
            );
          }
          if (live) reloadCurrent();
        });
        group.appendChild(button);
      }
      wrap.appendChild(group);
    }

    if (knob.hint) {
      const hint = document.createElement("p");
      hint.className = "guided-knob-hint";
      hint.textContent = knob.hint;
      wrap.appendChild(hint);
    }
    return wrap;
  }

  // ---- シーンの組み立てと読み込み --------------------------------------------

  /** グラフの凡例に出す表示名(プローブ番号 → 人間の言葉)。 */
  function probeLabelsFor(experiment: Experiment): Record<number, string> {
    const labels: Record<number, string> = { ...(experiment.series ?? {}) };
    for (const readout of experiment.readouts ?? []) {
      // 複数プローブから作る値(距離・速さ)は、もとの系列とは別物なので
      // 凡例には流用しない——`series` が明示している名前をそのまま使う。
      if (readout.derive) continue;
      labels[readout.probe] = readout.label;
    }
    return labels;
  }

  /** つまみの値を反映したシーン JSON 文字列を作る。 */
  function sceneJsonFor(
    experiment: Experiment,
    values: Record<string, string | number>,
  ): string | null {
    if (experiment.build) return JSON.stringify(experiment.build(values));
    if (!experiment.file) return null;
    const raw = sceneFileContent(experiment.file);
    if (!raw) return null;
    const scene = JSON.parse(raw) as SceneJson;
    experiment.prepare?.(scene);
    for (const knob of experiment.knobs ?? []) {
      const value = values[knob.id];
      if (value !== undefined) knob.apply(scene, value);
    }
    return JSON.stringify(scene);
  }

  function startExperiment(experiment: Experiment): void {
    current = experiment;
    pendingExperiment = experiment;
    // グラフを自動で開くのは「グラフが主役の実験」と「数値の動きを追う実験」だけ。
    // 場のパネルだけを見る実験(二重スリット等)で開くと、**空のグラフ枠**が
    // 画面の下 1/4 を占めるだけになる。
    showGraph =
      experiment.view === "graph" || (experiment.readouts?.length ?? 0) > 0;
    try {
      localStorage.setItem(LAST_EXPERIMENT_KEY, experiment.id);
    } catch {
      /* 記憶できなくても動作には影響しない */
    }
    closeChooser();
    reloadCurrent();
    renderGuidePanel();
  }

  /** いまのつまみの値でシーンを作り直し、頭から再生する。 */
  function reloadCurrent(): void {
    const api = apiRef.current;
    if (!current) return;
    if (!api) {
      pendingStart = true;
      return;
    }
    const json = sceneJsonFor(current, knobValues);
    if (!json) return;
    api.loadSceneJson(json);
    api.setProbeLabels(probeLabelsFor(current));
    api.setPace(current.pace * speedMultiplier);
    api.play();
    syncBar();
    applyGraphVisibility();
  }

  // ---- 上部バー -------------------------------------------------------------
  function syncBar(): void {
    const api = apiRef.current;
    const playing = api?.isPlaying() ?? false;
    playButton.textContent = playing ? "⏸ とめる" : "▶ うごかす";
    playButton.dataset.playing = String(playing);
    playButton.disabled = !current;
    restartButton.disabled = !current;
    graphButton.disabled = !current;
    cameraButton.disabled = !current;
    graphButton.dataset.on = String(showGraph);
    guidedTitle.textContent = current
      ? `${current.icon} ${current.title}`
      : "🔰 かんたんモード";
    guidedBlurb.textContent = current
      ? current.blurb
      : "「実験をえらぶ」から始めてください。";
    for (const button of speedGroup.querySelectorAll("button")) {
      button.classList.toggle(
        "active",
        Number((button as HTMLElement).dataset.speed) === speedMultiplier,
      );
    }
  }

  playButton.addEventListener("click", () => {
    const api = apiRef.current;
    if (!api || !current) return;
    if (api.isPlaying()) api.pause();
    else api.play();
    syncBar();
  });

  restartButton.addEventListener("click", () => reloadCurrent());
  cameraButton.addEventListener("click", () => apiRef.current?.followCamera(true));
  chooseButton.addEventListener("click", () => openChooser(1));

  graphButton.addEventListener("click", () => {
    showGraph = !showGraph;
    applyGraphVisibility();
    syncBar();
  });

  function applyGraphVisibility(): void {
    app.dataset.guidedGraph = showGraph ? "on" : "off";
    // 3D に描かれない現象(グラフが主役)では、グラフを大きく取る。
    app.dataset.guidedView = current?.view ?? "3d";
    requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
  }

  for (const button of speedGroup.querySelectorAll("button")) {
    button.addEventListener("click", () => {
      speedMultiplier = Number((button as HTMLElement).dataset.speed) || 1;
      if (current) apiRef.current?.setPace(current.pace * speedMultiplier);
      syncBar();
    });
  }

  // ---- 右の「ガイド」パネル ---------------------------------------------------
  let readoutNodes: { index: number; node: HTMLElement }[] = [];

  function renderGuidePanel(): void {
    guidedPanelBody.innerHTML = "";
    readoutNodes = [];
    if (!current) {
      const empty = document.createElement("p");
      empty.className = "guided-empty";
      empty.textContent =
        "上の「① 実験をえらぶ」を押すと、見たい現象を 3 手で選べます。";
      guidedPanelBody.appendChild(empty);
      return;
    }

    const watch = document.createElement("section");
    watch.className = "guided-section";
    watch.innerHTML =
      `<h3>ここを見てください</h3>` +
      `<ul class="guided-watch">${current.watch
        .map((line) => `<li>${line}</li>`)
        .join("")}</ul>`;
    guidedPanelBody.appendChild(watch);

    const where = document.createElement("p");
    where.className = "guided-where";
    where.textContent =
      current.view === "3d"
        ? "👀 まん中の 3D 画面を見てください。"
        : current.view === "field"
          ? "👀 3D 画面の中に出る「場」のパネルに、波や分布が描かれます。"
          : "📈 下のグラフがいちばん分かりやすい場所です。";
    guidedPanelBody.appendChild(where);

    // 「いまの数値」は**必ず出す**。プローブを持たない実験(3D や場の絵だけを
    // 見るもの)でも、経過時間が動いていること自体が「ちゃんと走っている」の
    // 唯一の手掛かりになるため。
    const numbers = document.createElement("section");
    numbers.className = "guided-section";
    numbers.innerHTML = "<h3>いまの数値</h3>";
    const list = document.createElement("dl");
    list.className = "guided-readouts";

    const timeKey = document.createElement("dt");
    timeKey.textContent = "経過した時間";
    const timeValue = document.createElement("dd");
    timeValue.id = "guided-readout-time";
    timeValue.textContent = "0 秒";
    // 表示は人間向けに単位を切り替えるので、機械(テスト)向けに生の秒も置く。
    timeValue.dataset.seconds = "0";
    list.appendChild(timeKey);
    list.appendChild(timeValue);

    for (const readout of current.readouts ?? []) {
      const key = document.createElement("dt");
      key.textContent = readout.label;
      const value = document.createElement("dd");
      value.dataset.probe = String(readout.probe);
      value.textContent = "—";
      list.appendChild(key);
      list.appendChild(value);
      readoutNodes.push({ index: readout.probe, node: value });
    }
    numbers.appendChild(list);
    guidedPanelBody.appendChild(numbers);

    if (current.knobs?.length) {
      const knobs = document.createElement("section");
      knobs.className = "guided-section guided-knobs";
      knobs.innerHTML =
        "<h3>変えてみる</h3>" +
        `<p class="guided-note">動かすと、その設定で最初からやり直します。</p>`;
      for (const knob of current.knobs) {
        knobs.appendChild(renderKnob(knob, true, "guided-knob"));
      }
      guidedPanelBody.appendChild(knobs);
    }

    const again = document.createElement("button");
    again.type = "button";
    again.className = "guided-again";
    again.id = "btn-guided-again";
    again.textContent = "ほかの実験をえらぶ";
    again.addEventListener("click", () => openChooser(2));
    guidedPanelBody.appendChild(again);
  }

  // 数値の更新。物理の 1 フレームごとに読み直す(表示だけなので軽い)。
  function tick(): void {
    const api = apiRef.current;
    if (api && pendingStart) {
      pendingStart = false;
      reloadCurrent();
    }
    if (api && current) {
      const timeNode = document.getElementById("guided-readout-time");
      if (timeNode) {
        const seconds = api.time();
        timeNode.textContent = formatDuration(seconds);
        timeNode.dataset.seconds = String(seconds);
      }
      const count = api.probeCount();
      for (const { index, node } of readoutNodes) {
        const readout = current.readouts?.find((r) => r.probe === index);
        const sources = readout?.probes ?? [index];
        if (sources.some((i) => i >= count)) {
          node.textContent = "—";
          continue;
        }
        const values = sources.map((i) => api.probeValue(i));
        const value = readout?.derive ? readout.derive(values) : values[0];
        node.textContent = readout?.format
          ? readout.format(value)
          : `${value.toFixed(readout?.digits ?? 2)}${
              readout?.unit ? ` ${readout.unit}` : ""
            }`;
      }
      if (playButton.dataset.playing !== String(api.isPlaying())) syncBar();
    }
    requestAnimationFrame(tick);
  }

  // ---- チューザ(オーバーレイ)の開閉 -----------------------------------------
  function openChooser(next: 1 | 2 | 3): void {
    step = next;
    chooser.hidden = false;
    renderSteps();
    if (step === 1) renderCategories();
    else if (step === 2) {
      const category = categoryId ? findCategory(categoryId) : undefined;
      if (category) renderExperiments(category);
      else {
        step = 1;
        renderSteps();
        renderCategories();
      }
    } else if (pendingExperiment) renderConfirm(pendingExperiment);
    else {
      step = 1;
      renderSteps();
      renderCategories();
    }
  }

  function closeChooser(): void {
    chooser.hidden = true;
  }

  chooser.addEventListener("click", (event) => {
    // 背景(オーバーレイ地)のクリックで閉じる。ただし実験を 1 つも動かして
    // いないうちは閉じても何も無い画面が残るだけなので閉じない。
    if (event.target === chooser && current) closeChooser();
  });

  document.addEventListener("keydown", (event) => {
    if (app.dataset.ui !== "guided") return;
    if (event.key === "Escape" && !chooser.hidden && current) {
      closeChooser();
      event.preventDefault();
    }
  });

  // ---- 起動 -----------------------------------------------------------------
  applyUiMode(storedUiMode());
  applyGraphVisibility();
  renderGuidePanel();
  syncBar();
  requestAnimationFrame(tick);

  if (app.dataset.ui === "guided") {
    // 初めて開いた人には、迷う余地なく ① が出ている状態から始めてもらう。
    openChooser(1);
  }
}

/** テスト・デバッグ用: カタログの実験数(index.html には出さない)。 */
export function guidedExperimentCount(): number {
  return GUIDED_CATEGORIES.reduce((sum, c) => sum + c.experiments.length, 0);
}

/** 前回選んだ実験の ID(次に開いたときの案内に使う)。 */
export function lastExperimentId(): Experiment | undefined {
  try {
    const id = localStorage.getItem(LAST_EXPERIMENT_KEY);
    return id ? findExperiment(id) : undefined;
  } catch {
    return undefined;
  }
}
