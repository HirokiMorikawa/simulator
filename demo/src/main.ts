import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { ConvexGeometry } from "three/examples/jsm/geometries/ConvexGeometry.js";
import init, {
  WasmWorld,
  run_headless_scenario_json,
  sketch_extrude_shape_json,
} from "../pkg/sim_wasm.js";
import "./style.css";
import { setUpGuidedMode, type GuidedApi, type GuidedApiRef } from "./guided";

// 統合エディタ(docs/23-frontend/01-editor.md)の骨格増分。
//
// **縮約実装の理由**: このファイルはドッキングレイアウトの骨格(§1)と、既存の
// Phase 0 デモ(床の上に箱が落ちて静止する)を Scene View パネルへ配線するところ
// までを扱う。Hierarchy/Inspector/Scene Viewは`WasmWorld`の複数ボディ列挙API
// (`body_count`/`body_label_at`/`body_position_at_f32`/`body_velocity_at_f32`)に
// 接続済みで、Hierarchyでのクリック・Scene Viewでのクリックピック(`THREE.
// Raycaster`、Alt-クリックで裏側を選択)のどちらからでも同じ選択状態
// (`selectBody`)を通じてInspectorが更新される(設計§1.2/§1.3が求める双方向
// 選択)。設計§4のEdit/Playモード分離を実装した——既定はEditモード(Unityと同じ
// 起動時挙動)で、Toolbarの Edit/Play トグルで切り替える。Editモードでは
// Scene Viewに選択中(かつ非静的)ボディのTranslate Gizmo(X赤/Y緑/Z青の
// 3軸ハンドル)が表示され、軸ハンドルをドラッグするとその軸方向にのみ
// `WasmWorld::set_body_position_at`(Commandキューを経由しない、
// `RigidBodySet`位置の直接書き換え)で位置を編集できる。同時にRotate Gizmo
// (X/Y/Z軸周りのリングハンドル)も表示され、リングをドラッグするとドラッグ
// 開始点からの画面上の角度差をそのままワールド軸周りの回転角として
// `WasmWorld::set_body_rotation_at`で適用する(Blenderのようなビュー平面
// トラックボールではなく単純な単一軸回転、スケールハンドルはこの2体デモに
// 意味のある対象が無いため未実装)。Gizmoドラッグ(位置のみ)開始のたびに
// 直前の位置/姿勢をUndoスタックへ積み、ToolbarのUndoボタン(Editモードかつ
// スタックが空でない場合のみ有効)で1件ずつ取り消せる(設計§6「Undo/Redo:
// Editモードのみ」、縮約実装によりシーンJSON差分ではなく単純な位置/姿勢スタック)。
// Redoボタンも実装済み(Undo時に取り消し前の値をRedoスタックへ積み、新規の
// Gizmoドラッグ開始でRedoスタックは破棄する標準的な意味論)。再生/ステップ/
// Nudgeボタンは無効化される(シミュレーションは
// 進行しない)。Playモードでは
// Gizmoは非表示になり、箱への直接ドラッグがCommandキュー経由
// (`Command::Grab/MoveGrab/Release`、`push_grab`/`push_move_grab`/
// `push_release`)の物理的な"つかむ"操作になり、再生/ステップ/Nudgeボタンが
// 有効になる。Shape/Materialは`world.body_shape_label_at`/
// `body_material_label_at`(スポーンパレットで追加したボディも含めて実際に
// クエリできる、`sim-wasm`側が構築時の値を覚えておく縮約実装)で取得する。
// Toolbarのスポーンパレット(設計§6「形状×材質を選んでクリック配置」)から
// 球/箱を追加できる(縮約実装によりカプセルは対象外、材質は代表的な4種のみ)。
// Scene Viewオーバーレイ(設計§1.2)は
// 選択中ボディの速度ベクトル(矢印)+ 接触点(既存の`World::contact_points`が
// 返す直近stepの接触点ワールド座標に小球マーカーを表示、この2体デモでは
// 着地/跳ね返りのたびに実際に現れる)+ 力(Nudgeボタンでキューに積む
// `Command::ApplyForce`の力ベクトルを、クリックした瞬間だけ短時間矢印表示——
// 一般の力の可視化(接触力・拘束反力の継続的な蓄積)には対応するWorld側の
// クエリが無いため対象外)を実装(いずれも切替可、Toolbarのチェックボックス。
// 拘束・流体場・フレーム軸のオーバーレイは対象外)。Consoleは
// `World::drain_events`(既存API)が返す実イベント(この2体デモでは箱の着地/
// 跳ね返りのたびに発生する`ContactStarted`/`ContactEnded`)をAll/Errors/
// Warnings/Infoタブでフィルタ表示し(設計§1.5)、イベント行(step番号を含む)を
// クリックするとその時刻に最も近いTimelineスナップショットへジャンプする
// (設計§1.5「クリックでTimeline/Scene Viewと連動」の時刻側、`jumpToStepRef`
// 経由でConsole/Scene View間を疎結合に配線)。Projectは静的な
// プレースホルダ内容のまま。TimelineはWorld::snapshot/restoreによる
// スナップショットリングバッファ(1s間隔・N=8面)でスクラブ・巻き戻しができ、
// 任意時点をブックマーク(`add_bookmark`/`restore_bookmark`、リングバッファの
// 退避を受けない別領域)として名前付きで保存・復元できる。Gizmoのスケール
// ハンドル・回転のUndo・残りのオーバーレイ種別(力・拘束・流体場・フレーム軸)・
// Commandキュー残り(SetMotorTarget/SetSwitch/SetHeatSource未配線)・
// 回路サブモードは全て後続増分。

const GRAVITY = 9.80665;
const DT = 1.0 / 120.0;
const INITIAL_HEIGHT = 10.0;
const BOX_HALF_EXTENT = 0.5;
const MAX_STEPS_PER_FRAME = 240;
const BODY_INDEX_GROUND = 0;
const BODY_INDEX_BOX = 1;

// スポーンパレット(設計docs/23-frontend/01-editor.md §6)で選べる材質。
// `sim_core::MaterialDb::standard`が持つ名前の一部(密度・反発特性が異なる
// ものを選び、着地の見た目が分かりやすいように)。
const SPAWN_MATERIALS = [
  "鋼(炭素鋼)",
  "アルミニウム",
  "木材(松)",
  "ゴム(天然)",
];
const SPAWN_HEIGHT = 12.0;
/// Hierarchy 右クリック「複製」で複製体をずらす距離 [m](群2)。
/// 同一位置に重ねると初期貫入から弾き飛ばされるため、必ず離す。
const DUPLICATE_OFFSET_M = 0.6;
const SPAWN_SPHERE_RADIUS = 0.4;
const SPAWN_BOX_HALF_EXTENT = 0.4;
const PENDULUM_PIVOT_HEIGHT = 6.0;
const PENDULUM_ARM_LENGTH = 2.0;

// **「新規シーン」ボタン**が読み込む固定シーンJSON(レビュー指摘対応、
// `docs/22-roadmap/03-editor-todo.md`参照)。既定の起動シーン(回路・熱
// ドメインの実演セットアップ込み)を消して、床の静的Planeボディ1個だけの
// まっさらな状態から始めたい時に使う。gravity/dtは既定シーンと同じ値
// (`GRAVITY`/`DT`)。
const NEW_SCENE_JSON = JSON.stringify({
  name: "new-scene",
  world: { gravity: GRAVITY, dt: DT },
  bodies: [
    {
      name: "ground",
      shape: { plane: { normal: [0, 1, 0], d: 0.0 } },
      material: "コンクリート",
      type: "static",
      collision_group: 1,
      collision_mask: 4294967295,
    },
  ],
});

function setUpLayoutPresetSwitcher() {
  const app = document.getElementById("app")!;
  const select = document.getElementById("select-layout") as HTMLSelectElement;
  select.addEventListener("change", () => {
    app.dataset.layout = select.value;
    // **プリセットが握る変数のインライン上書きを捨てる**。スプリッター
    // (`setUpPanelSplitters`)は `#app` のインラインスタイルへ `--row-console`
    // を書くが、インラインは `#app[data-layout=…]` のルールより強いので、
    // 捨てないと「レイアウトを切り替えても Console の高さが変わらない」
    // という無言の不具合になる(増分E3 の `--project-row` で踏んだのと同じ、
    // 「同じ宣言を 2 つの機能が奪い合う」問題)。列幅はプリセットが触らない
    // ので残す。
    clearPresetOwnedPanelSizes();
  });
}

// ---------------------------------------------------------------------------
// UI 基盤(増分「UI 品質の底上げ」)
//
// 設計 docs/23-frontend/01-editor.md §1 が求めていながら未実装だったもの
// (パネルのリサイズ)と、QA 報告書 docs/reviews/2026-08-04-editor-qa.md §5 が
// 「未検証」と明記していた領域(キーボードのみでの操作)を埋める層。
// どれも特定のパネルに属さないので、パネル実装より前にまとめて置く。
// ---------------------------------------------------------------------------

/// **トースト通知**。失敗の即時通知はこれまで `window.alert` だった——操作を
/// ブロックし、OK を押させ、押した瞬間に文面が消えるモーダルである。読み返す
/// ための Console Errors タブは D3 増分で用意済みなので、即時通知の側だけを
/// 非ブロッキングに置き換える。`#toast-region` は `aria-live="polite"` なので
/// 読み上げにも届く(`window.alert` はフォーカスを奪う代わりに、閉じた後に
/// 何も残さない点でスクリーンリーダー利用者にも不利だった)。
const TOAST_TIMEOUT_MS = 8000;
function showToast(
  message: string,
  kind: "error" | "success" = "error",
): void {
  const region = document.getElementById("toast-region");
  if (!region) return;
  const toast = document.createElement("div");
  toast.className = "toast";
  toast.dataset.kind = kind;
  const text = document.createElement("div");
  text.className = "toast-message";
  text.textContent = message;
  const close = document.createElement("button");
  close.className = "toast-close";
  close.type = "button";
  close.setAttribute("aria-label", "通知を閉じる");
  close.textContent = "✕";
  close.addEventListener("click", () => toast.remove());
  toast.append(text, close);
  region.appendChild(toast);
  // 積み上がりすぎないよう古いものから捨てる(連続失敗で画面が埋まらないように)。
  while (region.children.length > 4) region.firstChild?.remove();
  window.setTimeout(() => toast.remove(), TOAST_TIMEOUT_MS);
}

/// **起動オーバーレイ**。wasm の取得とコンパイルのあいだ「読み込み中」である
/// ことを示し、失敗したらその場に理由を出す(従来は HUD へ小さく出るだけで、
/// 空のパネルが並んだ画面との区別が付かなかった)。
function markBootReady(): void {
  const overlay = document.getElementById("boot-overlay");
  if (!overlay) return;
  overlay.dataset.state = "ready";
  // フェードアウト(200ms、`style.css` の transition)を待ってから DOM から外す。
  window.setTimeout(() => {
    overlay.hidden = true;
  }, 250);
}
function markBootFailed(message: string): void {
  const overlay = document.getElementById("boot-overlay");
  if (!overlay) return;
  overlay.hidden = false;
  overlay.dataset.state = "error";
  const target = overlay.querySelector(".boot-message");
  if (target) {
    target.textContent = `物理エンジンの読み込みに失敗しました。\n${message}`;
  }
}

/// **パネルのリサイズ**(設計 §1「ブラウザ 1 ページ内でリサイズ・タブ化・
/// 切り離しができる」の、リサイズの部分)。
///
/// グリッドのガター列/行そのものを掴ませる。値は `#app` のインラインスタイルへ
/// CSS 変数として書き、localStorage に残す。**タブ化・切り離しは引き続き対象外**
/// ——パネルの入れ替えはグリッドエリアの静的な割り当てを崩す必要があり、
/// 本増分の範囲を超える。
type SplitterLimits = { min: number; max: () => number; fallback: number };
const SPLITTER_LIMITS: Record<string, SplitterLimits> = {
  "--col-left": { min: 150, max: () => window.innerWidth * 0.4, fallback: 220 },
  "--col-right": { min: 190, max: () => window.innerWidth * 0.45, fallback: 268 },
  "--row-console": { min: 80, max: () => window.innerHeight * 0.6, fallback: 160 },
};
/// プリセット(`#app[data-layout=…]`)が握っている変数。`setUpLayoutPresetSwitcher`
/// はこれだけをインラインから外す。
const PRESET_OWNED_PANEL_VARS = ["--row-console"];
const PANEL_SIZE_STORAGE_KEY = "simulator.editor.panel-sizes";

function readStoredPanelSizes(): Record<string, number> {
  try {
    const raw = window.localStorage.getItem(PANEL_SIZE_STORAGE_KEY);
    if (!raw) return {};
    const parsed: unknown = JSON.parse(raw);
    if (!parsed || typeof parsed !== "object") return {};
    const out: Record<string, number> = {};
    for (const [key, value] of Object.entries(parsed as Record<string, unknown>)) {
      if (key in SPLITTER_LIMITS && typeof value === "number" && value > 0) {
        out[key] = value;
      }
    }
    return out;
    // localStorage はプライベートウィンドウ等で例外を投げ得る。保存できない
    // ことは機能の本質ではないので黙って諦める(既定サイズで動く)。
  } catch {
    return {};
  }
}
function writeStoredPanelSizes(sizes: Record<string, number>): void {
  try {
    window.localStorage.setItem(PANEL_SIZE_STORAGE_KEY, JSON.stringify(sizes));
  } catch {
    /* 保存できなくても操作自体は成立する。 */
  }
}
function clearPresetOwnedPanelSizes(): void {
  const app = document.getElementById("app");
  if (!app) return;
  const sizes = readStoredPanelSizes();
  for (const name of PRESET_OWNED_PANEL_VARS) {
    app.style.removeProperty(name);
    delete sizes[name];
  }
  writeStoredPanelSizes(sizes);
}

function setUpPanelSplitters(): void {
  const app = document.getElementById("app");
  if (!app) return;
  const stored = readStoredPanelSizes();

  function setSize(name: string, px: number, persist: boolean): number {
    const limits = SPLITTER_LIMITS[name];
    const clamped = Math.round(
      Math.min(Math.max(px, limits.min), Math.max(limits.max(), limits.min)),
    );
    app!.style.setProperty(name, `${clamped}px`);
    if (persist) {
      const sizes = readStoredPanelSizes();
      sizes[name] = clamped;
      writeStoredPanelSizes(sizes);
    }
    return clamped;
  }
  /// 今の実寸(px)。インライン上書きが無ければ CSS 側の既定を読む。
  function currentSize(name: string): number {
    const raw = getComputedStyle(app!).getPropertyValue(name).trim();
    const parsed = Number.parseFloat(raw);
    return Number.isFinite(parsed) && parsed > 0
      ? parsed
      : SPLITTER_LIMITS[name].fallback;
  }

  for (const [name, value] of Object.entries(stored)) setSize(name, value, false);

  const splitters =
    document.querySelectorAll<HTMLElement>(".splitter[data-var]");
  splitters.forEach((splitter) => {
    const name = splitter.dataset.var!;
    if (!(name in SPLITTER_LIMITS)) return;
    const axis = splitter.dataset.axis === "y" ? "y" : "x";
    // 掴んだ境界の「どちら側」のパネルを伸ばすか。Inspector と Console は
    // ガターより後ろ(右/下)にあるので、ポインタの移動方向と逆に伸びる。
    const sign = splitter.dataset.invert === "true" ? -1 : 1;

    function announce(px: number) {
      splitter.setAttribute("aria-valuenow", String(Math.round(px)));
      splitter.setAttribute("aria-valuemin", String(SPLITTER_LIMITS[name].min));
      splitter.setAttribute(
        "aria-valuemax",
        String(Math.round(SPLITTER_LIMITS[name].max())),
      );
    }
    announce(currentSize(name));

    splitter.addEventListener("pointerdown", (event) => {
      // 主ボタンのみ。右クリックで掴んだままになるのを防ぐ。
      if (event.button !== 0) return;
      event.preventDefault();
      const start = axis === "x" ? event.clientX : event.clientY;
      const startSize = currentSize(name);
      splitter.setPointerCapture(event.pointerId);
      splitter.dataset.dragging = "true";
      document.body.dataset.splitterDragging = "true";
      document.body.style.setProperty(
        "--splitter-cursor",
        axis === "x" ? "col-resize" : "row-resize",
      );

      const onMove = (move: PointerEvent) => {
        const now = axis === "x" ? move.clientX : move.clientY;
        announce(setSize(name, startSize + (now - start) * sign, false));
      };
      const onUp = () => {
        splitter.removeEventListener("pointermove", onMove);
        splitter.removeEventListener("pointerup", onUp);
        splitter.removeEventListener("pointercancel", onUp);
        delete splitter.dataset.dragging;
        delete document.body.dataset.splitterDragging;
        document.body.style.removeProperty("--splitter-cursor");
        // 確定時にだけ保存する(ドラッグ中に毎フレーム書くと無駄が大きい)。
        setSize(name, currentSize(name), true);
      };
      splitter.addEventListener("pointermove", onMove);
      splitter.addEventListener("pointerup", onUp);
      splitter.addEventListener("pointercancel", onUp);
    });

    // ダブルクリックで既定へ戻す(掴み直して探るより速い、一般的な作法)。
    splitter.addEventListener("dblclick", () => {
      app!.style.removeProperty(name);
      const sizes = readStoredPanelSizes();
      delete sizes[name];
      writeStoredPanelSizes(sizes);
      announce(currentSize(name));
    });

    // **キーボードでも動かせる**(QA 報告書 §5「キーボードのみでの操作は未検証」)。
    // マウスを持たない利用者にとって、ドラッグしか手段が無い操作は存在しないのと
    // 同じになる。
    splitter.addEventListener("keydown", (event) => {
      const step = event.shiftKey ? 48 : 16;
      let delta = 0;
      if (axis === "x" && event.key === "ArrowLeft") delta = -step;
      else if (axis === "x" && event.key === "ArrowRight") delta = step;
      else if (axis === "y" && event.key === "ArrowUp") delta = -step;
      else if (axis === "y" && event.key === "ArrowDown") delta = step;
      else if (event.key === "Home") {
        app!.style.removeProperty(name);
        const sizes = readStoredPanelSizes();
        delete sizes[name];
        writeStoredPanelSizes(sizes);
        announce(currentSize(name));
        event.preventDefault();
        return;
      } else return;
      announce(setSize(name, currentSize(name) + delta * sign, true));
      event.preventDefault();
    });
  });
}

/// **ショートカット一覧**。定義を `keydown` ハンドラと同じファイルに置く
/// (`setUpSceneView` 内のハンドラが実装、ここが一覧)。QA 不具合 7 は
/// 「`title` と README には書いてあるが `keydown` に case が無い」という
/// 食い違いだったので、一覧の側も同じファイルに置いて突き合わせやすくする。
const SHORTCUT_GROUPS: { title: string; items: [string, string][] }[] = [
  {
    title: "ツール",
    items: [
      ["W", "移動ギズモ"],
      ["E", "回転ギズモ"],
      ["R", "スケールギズモ"],
      ["Q", "選択のみ(ギズモ非表示)"],
      ["S", "スケッチツール"],
      ["X", "ギズモ座標系を World / Local で切替"],
    ],
  },
  {
    title: "再生・時間",
    items: [
      ["Space", "再生 / 一時停止(Play モード)"],
      ["F", "選択中のボディへカメラを寄せる"],
    ],
  },
  {
    title: "編集",
    items: [
      ["Ctrl / ⌘ + Z", "元に戻す"],
      ["Ctrl / ⌘ + Shift + Z", "やり直す"],
      ["Ctrl / ⌘ + Y", "やり直す"],
      ["Ctrl / ⌘ + D", "選択中のボディを複製"],
      ["Delete", "選択中のボディを削除"],
    ],
  },
  {
    title: "選択",
    items: [
      ["Ctrl / ⌘ + クリック", "Hierarchy で選択を追加・解除"],
      ["Shift + クリック", "Hierarchy で範囲選択"],
      ["Ctrl / ⌘ + A", "全ボディを選択"],
      ["↑ / ↓", "Hierarchy 内を移動(パネルにフォーカス中)"],
      ["Esc", "複数選択を解除 / メニュー・ダイアログを閉じる"],
    ],
  },
  {
    title: "スケッチ中",
    items: [
      ["Enter", "作図中の点列を 1 枚のプロファイルとして確定"],
      ["Backspace", "作図中の最後の点を取り消す"],
    ],
  },
  {
    title: "パネル",
    items: [
      ["? / F1", "この一覧を開く / 閉じる"],
      ["← → ↑ ↓", "スプリッターにフォーカス中はパネルの大きさを変える"],
      ["Home", "スプリッターにフォーカス中は既定の大きさへ戻す"],
    ],
  },
];

function setUpShortcutOverlay(): void {
  const overlay = document.getElementById("shortcut-overlay");
  const columns = document.getElementById("shortcut-columns");
  const button = document.getElementById("btn-shortcuts");
  if (!overlay || !columns) return;

  for (const group of SHORTCUT_GROUPS) {
    const section = document.createElement("div");
    section.className = "shortcut-group";
    const heading = document.createElement("h3");
    heading.textContent = group.title;
    section.appendChild(heading);
    for (const [keys, description] of group.items) {
      const row = document.createElement("div");
      row.className = "shortcut-row";
      const kbd = document.createElement("kbd");
      kbd.textContent = keys;
      const text = document.createElement("span");
      text.textContent = description;
      row.append(text, kbd);
      section.appendChild(row);
    }
    columns.appendChild(section);
  }

  let lastFocused: HTMLElement | null = null;
  function open() {
    lastFocused = document.activeElement as HTMLElement | null;
    overlay!.hidden = false;
    (overlay!.querySelector(".shortcut-dialog") as HTMLElement | null)?.focus();
  }
  function close() {
    overlay!.hidden = true;
    // 開く前にフォーカスしていた場所へ返す(キーボード利用者が迷子にならない)。
    lastFocused?.focus?.();
  }
  function toggle() {
    if (overlay!.hidden) open();
    else close();
  }

  const dialog = overlay.querySelector(".shortcut-dialog") as HTMLElement | null;
  dialog?.setAttribute("tabindex", "-1");
  button?.addEventListener("click", toggle);
  // 背景(ダイアログの外側)クリックで閉じる。
  overlay.addEventListener("click", (event) => {
    if (event.target === overlay) close();
  });

  window.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && !overlay.hidden) {
      close();
      event.preventDefault();
      return;
    }
    const target = event.target as HTMLElement | null;
    if (target && /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName)) return;
    if (event.ctrlKey || event.metaKey || event.altKey) return;
    // `?` は多くの配列で Shift + `/`。`event.key` を見れば配列差を吸収できる。
    if (event.key === "?" || event.key === "F1") {
      toggle();
      event.preventDefault();
    }
  });
}

/// **タブのキーボード操作と `aria-selected` の同期**(Console / Project)。
/// `role="tablist"` を名乗る以上、左右キーで移動できる必要がある(WAI-ARIA の
/// tabs パターン)。`aria-selected` は `.active` クラスの写しなので、ここで
/// 一括して面倒を見る——各パネルの実装は従来どおり `.active` だけを触ればよい。
function setUpTabListKeyboardNavigation(): void {
  for (const list of document.querySelectorAll<HTMLElement>(
    '[role="tablist"]',
  )) {
    const tabs = Array.from(
      list.querySelectorAll<HTMLButtonElement>('[role="tab"]'),
    );
    if (tabs.length === 0) continue;

    function syncSelected() {
      for (const tab of tabs) {
        const active = tab.classList.contains("active");
        tab.setAttribute("aria-selected", active ? "true" : "false");
        // ロービング tabindex: Tab キーは tablist 全体で 1 回だけ止まる。
        tab.tabIndex = active ? 0 : -1;
      }
    }
    syncSelected();
    // 既存のクリックハンドラ(`.active` を付け替える)より後に走るので、
    // 付け替えの結果をそのまま写せる。
    list.addEventListener("click", syncSelected);

    list.addEventListener("keydown", (event) => {
      const index = tabs.indexOf(event.target as HTMLButtonElement);
      if (index < 0) return;
      let next: number | null = null;
      if (event.key === "ArrowRight") next = (index + 1) % tabs.length;
      else if (event.key === "ArrowLeft")
        next = (index - 1 + tabs.length) % tabs.length;
      else if (event.key === "Home") next = 0;
      else if (event.key === "End") next = tabs.length - 1;
      else return;
      tabs[next].focus();
      tabs[next].click();
      event.preventDefault();
    });
  }
}

/// **Hierarchy のキーボード操作**。ツリーは `setUpHierarchy` が world の
/// 差し替えのたびに丸ごと作り直すので、個々の行にハンドラを付けると再構築の
/// たびに配線し直すことになる。`#hierarchy-tree` 自体を 1 つのフォーカス対象
/// (`tabindex="0"` + `aria-activedescendant`)にし、委譲で処理する——
/// 作り直されても配線が生き残る。
function setUpHierarchyKeyboardNavigation(): void {
  const tree = document.getElementById("hierarchy-tree");
  if (!tree) return;
  tree.tabIndex = 0;

  function visibleItems(): HTMLElement[] {
    return Array.from(
      tree!.querySelectorAll<HTMLElement>(".tree-selectable"),
    ).filter((el) => el.offsetParent !== null);
  }
  function activeIndex(items: HTMLElement[]): number {
    const selected = items.findIndex((el) => el.classList.contains("selected"));
    return selected >= 0 ? selected : 0;
  }
  function focusItem(item: HTMLElement) {
    if (!item.id) item.id = `hierarchy-item-${Math.random().toString(36).slice(2, 8)}`;
    tree!.setAttribute("aria-activedescendant", item.id);
    item.scrollIntoView({ block: "nearest" });
  }

  tree.addEventListener("keydown", (event) => {
    const items = visibleItems();
    if (items.length === 0) return;
    const index = activeIndex(items);
    let next: number | null = null;
    if (event.key === "ArrowDown") next = Math.min(index + 1, items.length - 1);
    else if (event.key === "ArrowUp") next = Math.max(index - 1, 0);
    else if (event.key === "Home") next = 0;
    else if (event.key === "End") next = items.length - 1;
    else if (event.key === "Enter" || event.key === " ") {
      items[index].click();
      focusItem(items[index]);
      event.preventDefault();
      return;
    } else return;
    // **選択はフォーカスに追従させる**(Unity の Hierarchy と同じ)。
    // 上下キーで見ている対象が Inspector と Scene View にそのまま反映される。
    items[next].click();
    focusItem(items[next]);
    event.preventDefault();
  });
}

/// ツリーの行へ WAI-ARIA の意味付けをする。`setUpHierarchy` の末尾から
/// 呼ばれ、再構築のたびに掛け直す(行の生成箇所が Bodies / Joints /
/// Materials / Frames / Probes と 5 箇所に分かれているため、1 箇所で
/// まとめて付けるほうが漏れない)。
function applyHierarchyAriaRoles(tree: HTMLElement): void {
  for (const group of tree.querySelectorAll<HTMLElement>("ul")) {
    group.setAttribute("role", "group");
  }
  for (const li of tree.querySelectorAll<HTMLElement>("li")) {
    li.setAttribute("role", "treeitem");
    if (li.classList.contains("tree-selectable")) {
      li.setAttribute(
        "aria-selected",
        li.classList.contains("selected") ? "true" : "false",
      );
    }
    const nested = li.querySelector(":scope > ul");
    if (nested) {
      li.setAttribute(
        "aria-expanded",
        (nested as HTMLElement).style.display === "none" ? "false" : "true",
      );
    }
  }
}

// Consoleパネル(設計docs/23-frontend/01-editor.md §1.5「SolverDiagnostics の
// 発散警告・…イベントをフィルタ表示」)。`world.drain_events_text`(既存の
// `World::drain_events`をそのまま使う、`sim_core::EventKind`)が返す
// `level::message`形式の行を実際のログとして追記し、タブでlevel別に絞り込む。
// 縮約実装の理由: クリックでTimeline/Scene Viewと連動させる機能・Contacts/
// Eventsタブ(設計は6タブだが本デモはAll/Errors/Warnings/Infoの4タブのみ)は
// 対象外。
const CONSOLE_LOG_CAPACITY = 200;

// クリック→時刻連動(設計docs/23-frontend/01-editor.md §1.5「クリックで
// Timeline/Scene Viewと連動」)。イベント行は`drain_events_text`が
// `step={N}`を埋め込むため、それを`data-step`属性に持たせておき、クリック時に
// `jumpToStepRef.current`(`setUpSceneView`がworld生成後に設定するコールバック)
// へその時刻へジャンプさせる。Console自体は世界(`world`)より先に構築されるため、
// 直接コールバックを渡せず可変の参照オブジェクト越しに後から配線する。
type JumpToStepRef = { current: ((step: number) => void) | null };

// Consoleのオブジェクト連動(設計§1.5「クリックでTimeline/Scene Viewと連動」の
// オブジェクト側、増分E4)。イベント行に埋め込まれた発生源ボディを選択させる。
// `JumpToStepRef`と全く同じ理由(Consoleは`world`より先に構築される)で、
// 可変の参照オブジェクト越しに`setUpSceneView`が後から`selectBody`を配線する。
/// Console の診断バッジ更新関数(増分K)。`render()`ループから毎フレーム
/// 呼ばれる(パネル構築が world 生成より先に走るため ref を介す)。
type ConsoleDiagnosticsRef = {
  current: ((residual: number, maxSpeed: number, dt: number) => void) | null;
};

type SelectBodyRef = { current: ((index: number) => void) | null };

// **エラーの永続表示(D3「Unityパリティ」増分)**。ConsoleのErrorsタブ
// (HTML側には元から存在する4タブの1つ)は、`drain_events_text`が返す
// `SolverDiagnostics`由来のイベントしか流し込んでおらず、その中に`"errors"`
// レベルへ分類される種別が1つも無い(`FuseBlown`/`SolverDiverged`/
// `JointBroken`はいずれも`"warnings"`、他は`"info"`——`setUpConsole`の`append`
// 直前のコメント参照)ため**常に空**だった。一方、Add Joint/Add Coupling・
// シーン読み込み・押し出し・材料派生などの失敗は`window.alert`の
// 一度きりのモーダルでしか伝わらず、閉じた瞬間に消えて後から見返せない
// (監査で発見した具体的な欠落——candidate gap「Console/log panel for
// warnings-errors」)。`window.alert`自体は即時のフィードバックとして
// 有用なので取り除かず、**Consoleへも同じメッセージを残す**形で埋める。
//
// `reportError`はモジュールスコープの自由関数(`wireAddJointForm`等、
// `main()`の外で定義される多数の関数から個別の引数を足さず直接呼びたい)
// なので、`JumpToStepRef`と同じ「world/Console構築より前に定義される関数から
// 呼べるよう、可変の参照変数越しに後から実体を配線する」構成を取る。ただし
// こちらは値を1個(関数)持つだけで済むため、他のRef群のような
// `{ current }`型は使わずモジュール変数そのものにした。
let consoleErrorAppend: ((message: string) => void) | null = null;

/// 失敗をユーザーへ即時に伝え、かつConsoleのErrorsタブへ恒久的に残す
/// (`consoleErrorAppend`未配線の間——起動直後の一瞬——は後者を静かにスキップする)。
///
/// **即時通知は `window.alert` からトーストへ移した(増分「UI 品質の底上げ」)**。
/// `alert` は操作をブロックし、読むために必ず OK を押させ、押した瞬間に文面が
/// 消える。連続で失敗すると数回押させられる。トーストなら操作を止めずに出せて、
/// 8 秒で自然に消え、消えても Console Errors タブに同じ文面が残る——「即時に
/// 気づける」と「後から見返せる」を両立させる形はこちらが正しい。
function reportError(message: string): void {
  consoleErrorAppend?.(message);
  showToast(message, "error");
}

/// **Inspector の編集ハンドラ(群2)**。設計 docs/23-frontend/01-editor.md §1.3 は
/// 「各 Component は World API の `Desc` 型と 1:1 対応。**編集は次ステップ先頭で
/// Command として適用される**(実行中は編集ロック — §4)」と定めているが、
/// これまで Inspector は全フィールドが読み取り専用の `<span>` だった。
///
/// `renderInspectorFor` はモジュールスコープの自由関数で `world` を引数に取る
/// (Command キューを持つクロージャの外側)ため、`SelectBodyRef` と同じ理由で
/// 可変の参照オブジェクト越しにハンドラを配線する。
type InspectorEditHandlers = {
  setMass(bodyIndex: number, mass: number): void;
  setBodyType(bodyIndex: number, kind: string): void;
  setCollisionFilter(bodyIndex: number, group: number, mask: number): void;
  /// 軸別スケール(群2、設計 §1.2 の Gizmo は Transform を編集する)。
  /// Box 以外では効かないので、適用できたかを返す。
  setScaleXyz(bodyIndex: number, sx: number, sy: number, sz: number): boolean;
  /// 等方スケール(**残タスク完遂の縦串①増分**)。既存のScale Gizmoドラッグと
  /// 同じ`set_body_scale_at`——球等、軸別スケールが効かない形状の半径を
  /// 正確な数値で指定する手段がGizmoのマウスドラッグしか無かった
  /// (D24車の車輪半径0.32mのような値をUIだけで再現できなかった)。
  /// 適用できたかを返す(Ground等は`false`)。
  setScale(bodyIndex: number, scale: number): boolean;
  /// Position の直接編集(**残タスク完遂の縦串①増分**)。Gizmo ドラッグと同じく
  /// `set_body_position_at`を直接呼ぶ(Commandを経由しない、構築時の位置決め
  /// なので次stepまで待たせる理由が無い——設計docs/20-integration/
  /// 04-world-api.md §1「シーン構築時のcreate系」に相当)。
  setPosition(bodyIndex: number, x: number, y: number, z: number): void;
};
type InspectorEditRef = { current: InspectorEditHandlers | null };
const inspectorEditRef: InspectorEditRef = { current: null };

/// 推力(**残タスク完遂の縦串⑤増分**、飛行機の物理の一部)。設計は
/// 「推力Coupling」を挙げているが、`Command::ApplyForce`(ワールド座標の力を
/// 剛体へ加える、既存の`push_apply_force`)が既にあり、ヒーターの
/// 「1step分だけ効く力を毎フレーム再送する」(`push_heat_source`と同じ
/// パターン)と同じやり方で組めるため、**新しいCoupling/Commandを物理コアへ
/// 足さずに**実装できる——ローカル軸をそのstepのボディ姿勢でワールドへ回し、
/// スロットル×最大推力を`push_apply_force`で送るだけ。物理コア変更の
/// リスクを避けつつ「エンジン」を実現する縮約(Inspectorの状態はここに
/// per-body保持、`renderInspectorFor`の再描画をまたいで生き残る必要が
/// あるため`inspectorEditRef`と同じくモジュールスコープ)。
type ThrustState = {
  enabled: boolean;
  axis: [number, number, number];
  maxThrust: number;
  throttle: number;
};
const thrustByBody = new Map<number, ThrustState>();
function thrustStateFor(bodyIndex: number): ThrustState {
  let state = thrustByBody.get(bodyIndex);
  if (!state) {
    state = { enabled: false, axis: [0, 0, 1], maxThrust: 1000, throttle: 0 };
    thrustByBody.set(bodyIndex, state);
  }
  return state;
}

/// **右クリックコンテキストメニュー(群2)**。設計 docs/23-frontend/01-editor.md は
/// Hierarchy(§1.1「右クリックでコンテキストメニュー(複製・削除・親付け・
/// プレハブ化・アイソレート表示)」)と Scene View(§1.2 のスポーンパレット)の
/// 両方で要求しているが、リポジトリ全体で `contextmenu` リスナは **0件**だった。
///
/// 1枚の `<div>` を使い回す(開くたびに中身を作り直す)。**メニューは常に
/// viewport 内に収める**——右端・下端で右クリックすると素朴な `left/top` 指定では
/// 画面外へはみ出して項目に到達できなくなる(増分E3のドロワーで踏んだのと
/// 同じクラスのバグ)。
type ContextMenuItem =
  | { label: string; onSelect: () => void; disabled?: boolean; title?: string }
  | { separator: true };

let contextMenuElement: HTMLDivElement | null = null;

function closeContextMenu(): void {
  contextMenuElement?.remove();
  contextMenuElement = null;
}

function showContextMenu(
  clientX: number,
  clientY: number,
  items: ContextMenuItem[],
): void {
  closeContextMenu();
  const menu = document.createElement("div");
  menu.id = "context-menu";
  menu.setAttribute("role", "menu");
  for (const item of items) {
    if ("separator" in item) {
      menu.appendChild(document.createElement("hr"));
      continue;
    }
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = item.label;
    button.disabled = item.disabled ?? false;
    if (item.title) button.title = item.title;
    button.addEventListener("click", () => {
      closeContextMenu();
      item.onSelect();
    });
    menu.appendChild(button);
  }
  // 一度 viewport 外の位置で貼って実寸を測り、はみ出す分だけ引き戻す。
  menu.style.left = "0px";
  menu.style.top = "0px";
  document.body.appendChild(menu);
  const rect = menu.getBoundingClientRect();
  const left = Math.max(0, Math.min(clientX, window.innerWidth - rect.width));
  const top = Math.max(0, Math.min(clientY, window.innerHeight - rect.height));
  menu.style.left = `${left}px`;
  menu.style.top = `${top}px`;
  contextMenuElement = menu;
}

// メニュー外クリック・Escape・スクロールで閉じる(Unity と同じ挙動)。
// `pointerdown` は capture 段で拾う——Scene View 側のピック処理が先に走って
// 選択が変わってしまうのを避けるため。
document.addEventListener(
  "pointerdown",
  (event) => {
    if (
      contextMenuElement &&
      !contextMenuElement.contains(event.target as Node)
    ) {
      closeContextMenu();
    }
  },
  true,
);
window.addEventListener("keydown", (event) => {
  if (event.key === "Escape") closeContextMenu();
});
window.addEventListener("blur", closeContextMenu);

// Projectドロワー Materials タブ(設計§1.6「Materials: MaterialDbプリセット一覧」)向け。
// Console/jumpToStepRefと同じ理由(worldより先にパネルが構築される)で、
// 可変の参照オブジェクト越しに`setUpSceneView`がworld生成後にコールバックを配線する。
type MaterialProperties = {
  name: string;
  density: number;
  friction: number;
  restitution: number;
  specificHeat: number;
  conductivity: number;
};
type MaterialsRef = { current: (() => MaterialProperties[]) | null };

// Projectドロワー Circuit タブ(設計docs/23-frontend/01-editor.md §1.7「回路
// エディタサブモード」の縮約実装——自由配線ではなく、既存の固定トポロジー
// (分圧回路)のトポロジー表示+ライブ電圧読み取りのみ)。MaterialsRefと同じ理由
// (worldより先にパネルが構築される)で、可変の参照オブジェクト越しにコールバックを
// 後から配線する。
type CircuitRef = { current: (() => number) | null };

// Circuitタブが「実際に配線されている素子」を読むためのref(**増分G2で追加**)。
// `CircuitRef`(分圧点の電圧を1つ返す)と同じ理由で、パネルの構築が`world`の
// 生成より先に走りうるためrefを介する。中身は`circuit_element_label_at`を
// 全素子ぶん呼んだ結果。
type CircuitElementsRef = { current: (() => string[]) | null };

// Projectドロワー Scenes タブ(設計docs/23-frontend/01-editor.md §1.6「Scenes:
// シーンJSON…Export/Import」)。Exportは現在のボディ一覧を(Inspector表示と同じ
// 人間可読なJSONへ)書き出すのみで、`sim_world::Scenario`スキーマとは形式が違う
// (Exportは表示専用、往復(round-trip)は想定しない)。Import(本増分で追加)は
// 逆に`Scenario`スキーマ(ヘッドレスランナー・D1–D43のシーンJSONと同じもの)を
// 読み、`world.import_scene_json`(`World::append_scenario_bodies`のwasm薄い
// ラッパー)で現在のワールドへボディを追加する——Exportした自分自身のJSONを
// そのままImportし直せるわけではない(スキーマが異なるため)が、設計書・
// ヘッドレスランナーのテストが使うシーンJSONファイルをエディタへ読み込んで
// 視覚的に確認できるようにする、というワークストリームDの狙い(項目13)には
// この非対称な形で十分応える。MaterialsRefと同じ理由(worldより先にパネルが
// 構築される)で、可変の参照オブジェクト越しにコールバックを後から配線する。
type SceneBodyExport = {
  index: number;
  label: string;
  shape: string;
  material: string;
  position: [number, number, number];
  isStatic: boolean;
};
type SceneExportRef = { current: (() => SceneBodyExport[]) | null };
/// Import の結果。`skipped`は**JSON に書かれていたのに取り込まなかった**
/// セクション名(QA不具合5)。Import は `materials`/`bodies`/`probes` しか
/// 見ないため、`couplings`/`thermal`/`circuit`/`fluids` などは捨てられる。
/// 捨てたことを UI とコンソールの両方へ出すためにここまで運ぶ。
type SceneImportResult = { count: number; skipped: string[] };
type SceneImportRef = { current: ((json: string) => SceneImportResult) | null };
/// 検証タブ(**残タスク完遂の縦串④増分**)が、現在のワールドを
/// `sim_world::Scenario`形式のJSON文字列として読むための口
/// (`world.read_component("export_scene_json", "")`の薄い写像、`SceneExportRef`と同じ理由で
/// refを介す——`setUpProjectDrawer`は`world`を直接持たない)。
type ValidationBaseJsonRef = { current: (() => string) | null };

/// **単一ファイル Export(群2)**。設計 docs/23-frontend/01-editor.md §1.6 は
/// Scenes / Replays / Bookmarks をそれぞれ別々に扱うが、**実際に他人へ渡したり
/// 後で自分が再現したりするときに要るのは3点セット**(どのシーンで・どんな操作を・
/// どの時点に注目したか)。これまでは Scenes タブで scene.json、Replays タブで
/// command_log.json をそれぞれ落とし、ブックマークは**そもそも書き出せなかった**。
///
/// `scene` は `WasmWorld::bookmark_export_scene_json` と同じ `sim_world::Scenario`
/// スキーマ(=ヘッドレスランナーと D1–D43 のテストが読む正典形式)なので、
/// このファイルの `scene` 部分だけを取り出せばそのままギャラリーへ読み込める。
type ProjectBundle = {
  formatVersion: 1;
  exportedAt: string;
  /// `sim_world::Scenario` スキーマのシーン JSON(文字列ではなくパース済み)。
  scene: unknown;
  /// 表示用のボディ一覧(Scenes タブが出しているのと同じ内容)。
  bodies: SceneBodyExport[];
  commandLog: CommandLogEntry[];
  bookmarks: { label: string; time: number }[];
  stateHash: string;
};
type ProjectBundleRef = { current: (() => ProjectBundle) | null };
/// 一括Export完了の通知(群2)。Project ドロワーからシーンビュー側の
/// 「未保存」フラグを下ろすための逆方向の配線。
type ProjectExportedRef = { current: (() => void) | null };

/// Replay のライブ再生(群2)。Project ドロワーの Replays タブから
/// Scene View 側の再生ドライバを操作する。
type ReplayPlaybackRef = {
  current: {
    start: () => { started: boolean; reason?: string; totalSteps?: number };
    stop: () => void;
    isPlaying: () => boolean;
    progress: () => number;
  } | null;
};

// シーンギャラリー(設計docs/23-frontend/01-editor.md §1.6「Scenes」、ワーク
// ストリームD項目13「D1–D43の全シーンをエディタから読み込み可能にし、視覚的な
// 合否確認を可能にする」)。Importが既存ワールドへボディを「追加」するのに対し、
// ギャラリーは`WasmWorld::from_scene_json`(`World::from_scenario`ベース、
// fluids/thermal/circuit/astro/joints/probesの全セクションが効く)で**ワールド
// 自体を丸ごと差し替える**——D9(熱のみ)やD34(天体のみ)のような非力学
// シーンも正しく成立させるため。
type SceneGalleryRef = { current: ((json: string) => void) | null };

// Replay再生実行(設計docs/23-frontend/01-editor.md §1.6「Replays」)。記録済みの
// `commandLog`を、既定シーン(床+箱のみ、`WasmWorld`のコンストラクタが構築する
// もの)を持つ新規`WasmWorld`へステップ番号どおりに再送し、実際に同じ入力列を
// 再実行できることを検証する(縮約実装: Scene Viewでのライブな視覚的再生では
// なく、ヘッドレスに再実行して最終状態を報告するテキストベースの検証——
// `world`をライブで差し替えるとScene View/Hierarchy/Inspector等の大部分の
// 配線をworldへの可変参照へ作り直す必要があり影響範囲が大きいため見送った)。
// スポーン/Importでボディが増えているとスポーン由来のCommand(モーター等)を
// 再現できない(`sceneChanged`で明示)。
type ReplayVerifyResult = {
  totalSteps: number;
  commandCount: number;
  sceneChanged: boolean;
  finalStateHash: string;
  finalBoxPosition: [number, number, number];
  liveStateHash: string;
  liveBoxPosition: [number, number, number];
  matches: boolean;
};
type ReplayVerifyRef = { current: (() => ReplayVerifyResult) | null };

// 自由配線回路エディタ(設計docs/23-frontend/01-editor.md §6「回路エディタ
// サブモード」(D19)の縮約実装——専用のグラフィカルなノード配線UIではなく、
// Circuitタブのフォームベースの操作でノード/素子を追加していく形とした、
// `sim-wasm`の`circuit_editor_*`メソッド群のdoc参照)向けに、`world`への
// 直接アクセスを持たない`setUpProjectDrawer`から呼べるようにするための
// コールバック群。`setSwitchClosed`以外は全て即時反映(Command経由ではない、
// `circuit_editor_set_switch_closed`のdoc参照)。
type CircuitEditorRef = {
  current: {
    reset: (numNodes: number) => void;
    addResistor: (a: number, b: number, resistance: number) => void;
    addVoltageSource: (a: number, b: number, voltage: number) => void;
    addSwitch: (a: number, b: number, closed: boolean) => number;
    setSwitchClosed: (index: number, closed: boolean) => void;
    nodeVoltage: (node: number) => number;
    // **回路素子4種をUIエディタに追加(縦串①の独立項目)**。ソルバ
    // (`sim_em::Circuit`)側は既に7種そろっており、ここまでUIから作れたのは
    // 抵抗・電圧源・スイッチの3種のみだった。
    addCapacitor: (
      a: number,
      b: number,
      capacitance: number,
      initialVoltage: number,
    ) => void;
    addInductor: (
      a: number,
      b: number,
      inductance: number,
      initialCurrent: number,
    ) => void;
    addDiode: (
      anode: number,
      cathode: number,
      saturationCurrent: number,
      nVt: number,
    ) => void;
    addDcMotor: (
      a: number,
      b: number,
      windingResistance: number,
      windingInductance: number,
      backEmfConstant: number,
    ) => number;
    setMotorSpeed: (index: number, angularVelocity: number) => void;
    motorCurrent: (index: number) => number;
  } | null;
};

// 自由配線回路が有効化されたかどうかの共有フラグ。`setUpProjectDrawer`(リセット
// ボタン押下時)がtrueにし、`setUpSceneView`の既存の固定デモ用「回路スイッチ
// (閉)」チェックボックスのハンドラがこれを見て自身を無効化する——リセット後は
// 固定デモの`circuit_switch_index`が新回路のスイッチ数を超えて無効になり得る
// (`circuit_editor_reset`のdoc参照)ため、パニックを避ける。
type CircuitFreeWiringState = { active: boolean };

// Prefabs(設計docs/23-frontend/01-editor.md §6「Prefabs: 再利用可能な
// Body/Joint/Circuit組(自作シーンから右クリック→「Prefabとして保存」)。
// 他シーンへドラッグで再利用」の縮約実装——Bodyの形状/材質のみ対象、
// Joint/Circuit組・ドラッグ&ドロップ・複数シーンをまたいだ永続化は対象外、
// ブラウザセッション内のみ保持)。`setUpProjectDrawer`から`world`への
// 直接アクセスを持たないため、他のRef同様コールバック経由で配線する。
//
// **形状は`ImportedShapeJson`(シーンJSONの`ShapeJson`と同じ形)で持つ**。
// 以前は`kind: string`+`params: number[]`(`body_shape_params_f64_at`が返す
// 平坦なf64配列)だったが、この表現ではCompound(入れ子)もConvexMesh(頂点群)
// も書けず、**両形状のボディはPrefab化しようとしても黙って何も起きなかった**
// (`captureBody`が球/箱以外を`null`で弾いていた)。`body_shape_json_at`↔
// `spawn_shape_json`という無損失に対になるwasm APIへ載せ替え、無限平面(床)を
// 除く5形状すべてをPrefab化できるようにした(平面を外す理由は`captureBody`の
// コメント参照)。`kind`はPrefabs一覧の表示ラベル専用として残す
// (形状の再構築には使わない)。
//
// **保存済みPrefabの移行は不要**——`prefabs`配列は`setUpProjectDrawer`の
// ローカル変数で、localStorage等の永続化先を一切持たない(上記「ブラウザ
// セッション内のみ保持」)。リロードすれば必ず空から始まるため、旧形式の
// `{kind, params}`がこの型に流れ込む経路そのものが存在しない。永続化を
// 足すときは、その時点でスキーマ版数を持たせること。
type PrefabDefinition = {
  name: string;
  /// Prefabs一覧の表示用(`body_shape_kind_at`が返す種別名)。
  kind: string;
  /// 形状そのもの(`body_shape_json_at`が返す`ShapeJson`をパースしたもの)。
  shape: ImportedShapeJson;
  material: string;
};
type PrefabRef = {
  current: {
    captureSelectedBody: () => Omit<PrefabDefinition, "name"> | null;
    /// 任意のボディをキャプチャする(群2: Hierarchy 右クリック→「プレハブ化」)。
    /// 選択中に限定しないのが `captureSelectedBody` との違い。
    captureBody: (index: number) => Omit<PrefabDefinition, "name"> | null;
    spawn: (prefab: PrefabDefinition) => void;
  } | null;
};

/// Prefab 登録(群2)。Prefabs タブが保持する配列へ、Hierarchy の右クリック
/// メニューから直接追加するための逆方向の配線(`PrefabRef` が Scene View →
/// ドロワーなのに対し、こちらはドロワー → 呼び出し元)。
type PrefabSaveRef = { current: ((prefab: PrefabDefinition) => void) | null };

// Import側のシーンJSONパース(`sim_world::scenario::ShapeJson`のJSON表現と同じ
// タグ付きオブジェクト形)。`world.import_scene_json`はボディの追加自体は行うが
// (返り値は追加件数のみ)、Scene Viewが各ボディに対応するThree.jsメッシュを
// 生成するには形状の種類・寸法が要る。`body_shape_label_at`は表示専用の整形
// 済み文字列(`Sphere(0.3000)`等)を返すのみで往復(round-trip)を想定していない
// ため、そこから数値を文字列パースするのではなく、Import時にJSがそもそも
// 持っている生のシーンJSONを(Rust側の`serde_json`とは独立に)そのまま
// `JSON.parse`して形状情報を読む。
type ImportedShapeJson =
  | { box: { half: [number, number, number] } }
  | { sphere: { radius: number } }
  | { capsule: { radius: number; half_height: number } }
  | { plane: { normal: [number, number, number]; d: number } }
  | {
      compound: {
        children: {
          position?: [number, number, number];
          rotation?: [number, number, number, number];
          shape: ImportedShapeJson;
        }[];
      };
    }
  | { convex_mesh: { vertices: [number, number, number][] } }
  // **`mesh`はD1(スケッチ・押し出し)で追加**。`convex_mesh`と違い面情報
  // (`triangles`)を持つ、**入力専用**のタグ(`sim_world::ShapeJson::Mesh`の
  // doc参照)。Rust側は読み込み時に近似凸分解へ通すため`body_shape_json_at`が
  // これを返すことは無い——ここで受け取るのは
  // ①スケッチ押し出しがその場で組み立てたJSON、②手書きのシーンJSON、の2つ。
  | {
      mesh: {
        vertices: [number, number, number][];
        triangles: [number, number, number][];
      };
    };
type ImportedBodyJson = { shape: ImportedShapeJson };
// 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5)向け。
// `sim_world::scenario::PredictionPromptJson`のJSON表現と同じ形(物理には
// 影響しないメタデータのため、Rust側で検証済みの値としてではなく、Importに
// 渡した生のJSONをJSが独立に読む——他のImportedShapeJson等と同じ設計)。
type ImportedPredictionPromptJson = {
  question: string;
  probe_index: number;
  expected_value: number;
};
type ImportedScenarioJson = {
  bodies?: ImportedBodyJson[];
  prediction_prompts?: ImportedPredictionPromptJson[];
};

// シーンギャラリー(`SceneGalleryRef`のdoc参照)向けのアセット読み込み。
// リポジトリ直下`scenes/`(ヘッドレスランナーのテストが読むのと同じファイル、
// `crates/sim-world/src/scenario.rs`の`all_scenes_in_the_gallery_manifest_
// parse_and_run_for_sixty_steps`が壊れたアセットの出荷を防ぐ)を`import.meta.glob`
// でバンドルする(`vite.config.ts`の`server.fs.allow: [".."]`によりdemo/外の
// ディレクトリを参照できる、ビルド後の出力にも静的にバンドルされる)。
type SceneGalleryManifestEntry = {
  file: string;
  demo: string;
  title: string;
  description: string;
  domains: string[];
};
const sceneGalleryFiles = import.meta.glob("../../scenes/*.json", {
  query: "?raw",
  import: "default",
  eager: true,
}) as Record<string, string>;

function sceneGalleryFileContent(file: string): string | null {
  const entry = Object.entries(sceneGalleryFiles).find(([path]) =>
    path.endsWith(`/${file}`),
  );
  return entry ? entry[1] : null;
}

function sceneGalleryManifest(): SceneGalleryManifestEntry[] {
  const indexJson = sceneGalleryFileContent("index.json");
  if (!indexJson) return [];
  return (JSON.parse(indexJson) as { scenes: SceneGalleryManifestEntry[] })
    .scenes;
}

function setUpConsole(
  jumpToStepRef: JumpToStepRef,
  selectBodyRef: SelectBodyRef,
  consoleDiagnosticsRef: ConsoleDiagnosticsRef,
): { append: (eventsText: string) => void; clear: () => void } {
  const log = document.getElementById("console-log")!;
  const tabs = document.querySelectorAll<HTMLButtonElement>(".console-tab");
  let activeLevel = "all";

  // **増分K**: タブは「重大度(all/errors/warnings/info)」に加えて
  // 「種別(contacts/events)」でも絞れるようにした(設計§1.5の6タブ)。
  // 種別は行の内容から決まる——接触イベントは `bodies=` を持つ(sim-wasm が
  // SourceId を復号して出す)ので、それを持つ行が contacts、それ以外で
  // `step=` を持つ実イベント行が events。
  const CATEGORY_TABS = new Set(["contacts", "events"]);
  function applyFilter() {
    for (const li of log.children) {
      const el = li as HTMLElement;
      let visible: boolean;
      if (activeLevel === "all") {
        visible = true;
      } else if (CATEGORY_TABS.has(activeLevel)) {
        visible = el.dataset.category === activeLevel;
      } else {
        visible = el.dataset.level === activeLevel;
      }
      el.style.display = visible ? "" : "none";
    }
  }

  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.toggle("active", t === tab));
      activeLevel = tab.dataset.tab!;
      applyFilter();
    });
  });

  log.addEventListener("click", (event) => {
    const li = (event.target as HTMLElement).closest("li");
    const step = li?.dataset.step;
    if (step && jumpToStepRef.current) {
      jumpToStepRef.current(Number(step));
    }
    const bodyIndex = li?.dataset.bodyIndex;
    if (bodyIndex && selectBodyRef.current) {
      selectBodyRef.current(Number(bodyIndex));
    }
  });

  const initial = document.createElement("li");
  initial.dataset.level = "info";
  initial.dataset.category = "info";
  // シーン切替時の`clear()`(QA不具合4)が消してよいのは前シーンの実行時
  // イベント(接触・ステップ等の「もう存在しないボディを指しうる」行)であって、
  // このシステム起動バナー自体はシーンに依存しないため対象外にする。
  initial.dataset.permanent = "true";
  initial.textContent =
    "[INFO] World起動 — SolverDiagnostics接続済み(ContactStarted/ContactEndedを表示)";
  log.appendChild(initial);

  // **診断バッジ(増分K)**。設計§1.5「発散・CFL警告バッジ」。
  // 判定材料は sim-wasm へ足した `energy_residual` と `max_body_speed`。
  //  - 発散: エネルギー台帳の残差が有限でない、または初期比で桁違いに増えた
  //  - CFL: 最大速度 × dt が代表長さを超える(1stepで自分の大きさ以上動く)
  // **どちらも「厳密な安定性判定」ではない**——本物のCFL数はソルバごとの
  // 空間離散化に依存する。ここは「ユーザーが画面を見て気づける粗い警告」として
  // の縮約であり、その旨をバッジのtitleにも書く。
  const badges = document.getElementById("console-badges")!;
  const CFL_CHARACTERISTIC_LENGTH = 0.5; // 既定シーンの箱の代表寸法。
  function updateDiagnostics(residual: number, maxSpeed: number, dt: number) {
    const entries: { kind: string; text: string; title: string }[] = [];
    if (!Number.isFinite(residual)) {
      entries.push({
        kind: "divergence",
        text: "発散",
        title: "エネルギー台帳の残差が有限でありません(発散)",
      });
    }
    const courant = (maxSpeed * dt) / CFL_CHARACTERISTIC_LENGTH;
    if (courant > 1.0) {
      entries.push({
        kind: "cfl",
        text: `CFL ${courant.toFixed(2)}`,
        title:
          "1ステップで代表長さ以上動いています(粗い目安であり厳密なCFL数では" +
          "ありません)。dtを小さくするか時間倍率を下げてください。",
      });
    }
    const signature = entries.map((e) => e.kind + e.text).join("|");
    if (badges.dataset.signature === signature) return;
    badges.dataset.signature = signature;
    badges.innerHTML = "";
    for (const e of entries) {
      const span = document.createElement("span");
      span.className = "console-badge";
      span.dataset.kind = e.kind;
      span.textContent = e.text;
      span.title = e.title;
      badges.appendChild(span);
    }
  }
  consoleDiagnosticsRef.current = updateDiagnostics;

  // QA不具合4: シーン切替時にConsoleがクリアされず、前シーンの接触ログが
  // 現シーンに存在しないボディindexを指したまま残っていた(クリックすると
  // 無関係なボディが選択される)。`clear`を公開し、シーン切替の入口
  // (`setUpSceneView`の`sceneGalleryRef.current`)から呼べるようにする。
  function clear() {
    for (const li of Array.from(log.children)) {
      if ((li as HTMLElement).dataset.permanent !== "true") li.remove();
    }
    applyFilter();
  }

  function append(eventsText: string) {
    if (!eventsText) return;
    for (const line of eventsText.split("\n")) {
      const [level, message] = line.split("::", 2);
      const li = document.createElement("li");
      li.dataset.level = level;
      const stepMatch = message.match(/step=(\d+)/);
      if (stepMatch) {
        li.dataset.step = stepMatch[1];
        li.classList.add("console-entry-clickable");
        li.title = "クリックでその時点のTimelineへジャンプ";
      }
      // オブジェクト連動(設計§1.5「クリックで…Scene Viewと連動」、増分E4)。
      // 接触イベントは `bodies=a,b` を持つ(`sim-wasm::drain_events_text` が
      // `SourceId` の符号化を復号して出す)。**先頭のボディを選択対象にする**
      // ——接触は2体の間で起きるが、選択は1体しか持てないため。どちらを選ぶかは
      // 任意なので、決定的になるよう常に先頭(a)にする(縮約、正直な記録)。
      const bodiesMatch = message.match(/bodies=(\d+),(\d+)/);
      // 種別タグ(増分K、Contacts/Events タブが使う)。
      li.dataset.category = bodiesMatch
        ? "contacts"
        : stepMatch
          ? "events"
          : "info";
      if (bodiesMatch) {
        li.dataset.bodyIndex = bodiesMatch[1];
        li.classList.add("console-entry-clickable");
        li.title = stepMatch
          ? `クリックでその時点のTimelineへジャンプ + ボディ${bodiesMatch[1]}を選択`
          : `クリックでボディ${bodiesMatch[1]}を選択`;
      }
      li.textContent = `[${level.toUpperCase()}] ${message}`;
      log.appendChild(li);
    }
    while (log.children.length > CONSOLE_LOG_CAPACITY) {
      log.removeChild(log.firstChild!);
    }
    applyFilter();
    log.scrollTop = log.scrollHeight;
  }

  return { append, clear };
}

// Hierarchyパネル(設計docs/23-frontend/01-editor.md §1.1「シーングラフツリー
// (Bodies/Joints/Circuits/Fluids/Probes/Frames)」)。`world.body_count`/
// `body_label_at`から実際のボディ一覧を組み立て、クリックで`onSelect`を呼ぶ
// (選択はInspector・Scene Viewと連動、設計が求める双方向選択)。戻り値の関数は
// Scene Viewピッキング(`onSelect`を経由せず見た目のハイライトだけ更新したい
// 場合)向けに、外部からハイライトだけを同期させる手段として公開する。
// Bodiesの兄弟としてJointsサブツリーも実装済み(振り子スポーンが追加した
// DistanceJointのみが対象、`constraint_anchor_points_at`で判定)。
// Fluids(概要行)・Frames(ドリルイン)・**Probes(増分E2)**は接続済み。
// **Circuitsは意図的に未対応のまま残す**: `sim_em::Circuit`は素子(抵抗・電圧源・
// スイッチ)を配列で持つがノード/素子を個別に列挙する公開APIが無く、ツリーに
// 並べるには`sim-wasm`側へ列挙APIを新設する必要がある。一方でCircuitタブの
// 自由配線エディタが既に素子一覧と各ノード電圧の表を出しており、ツリーへ
// 重複表示する実利が薄いと判断した(必要になった時点で列挙APIごと追加する)。
/// **Hierarchy の右クリック操作(群2)**。設計 §1.1「右クリックでコンテキスト
/// メニュー(複製・削除・親付け・プレハブ化・アイソレート表示)」。
///
/// **「親付け」だけは対象外にする**——`RigidBodySet` に剛体同士の親子関係が無く
/// (`FrameId` は座標系の階層であってボディの階層ではない)、UI だけ作っても
/// 何も起きない。フレーム階層のドリルイン UI が既に別途あるので、そちらが
/// この役割を担う(できないことをメニューに並べない、という判断)。
type HierarchyActions = {
  duplicate(index: number): void;
  remove(index: number): void;
  /// 選択中のボディだけを Scene View に表示する(`null` で解除)。
  isolate(index: number | null): void;
  isolatedIndex(): number | null;
  capturePrefab(index: number): void;
};

/// 折りたたみ状態(設計 §1.1「ツリーは折り畳み可」)。`setUpHierarchy` は
/// world 差し替えのたびにツリーを作り直すので、**状態はツリーの外に持つ**
/// ——さもないとシーンを読み込むたびに全部開いた状態へ戻る。
const collapsedHierarchyGroups = new Set<string>();

/// 複数選択(設計 §1.1「複数選択可」)。Inspector は単一選択前提のままなので、
/// **複数選択は Hierarchy 上の操作対象の集合**として機能する(右クリックの
/// 複製/削除がまとめて効く)。Inspector には最後にクリックしたものを出す。
const hierarchyMultiSelection = new Set<number>();

/// **Shift クリックの範囲選択の起点(D3「Unityパリティ」増分)**。監査で
/// 見つかった具体的な欠落——Ctrl/Cmd クリックのトグルは既にあったが、Unity の
/// Hierarchy(および大半のファイルマネージャ)が備える「Shift クリックで
/// 直前のクリック位置から今クリックした行までを一括選択」が無かった。
/// `hierarchyMultiSelection`と同じくツリー再構築(world差し替え)をまたいで
/// 持たせる——シーンを切り替えても直前にクリックした行番号自体に意味は
/// 無くなるため実害は無く、モジュール外の状態を増やさないほうが単純。
let hierarchyRangeAnchor: number | null = null;

function setUpHierarchy(
  world: WasmWorld,
  onSelect: (index: number) => void,
  selectedFrameIndex: number,
  onSelectFrame: (frameIndex: number) => void,
  actions: HierarchyActions | null,
  materialNames: readonly string[],
): (index: number) => void {
  const tree = document.getElementById("hierarchy-tree")!;
  tree.innerHTML = "";
  const root = document.createElement("li");
  root.textContent = "World Root";
  const bodies = document.createElement("ul");
  bodies.className = "tree-nested";

  /// グループ見出しを「折り畳み可」にする。見出しのクリックで開閉し、
  /// 状態は `collapsedHierarchyGroups` に残す。
  function makeGroup(
    key: string,
    label: string,
    contents: HTMLUListElement,
  ): HTMLLIElement {
    const item = document.createElement("li");
    item.className = "tree-group";
    const toggle = document.createElement("span");
    toggle.className = "tree-toggle";
    const apply = () => {
      const collapsed = collapsedHierarchyGroups.has(key);
      toggle.textContent = collapsed ? "▶" : "▼";
      contents.style.display = collapsed ? "none" : "";
    };
    toggle.addEventListener("click", (event) => {
      event.stopPropagation();
      if (collapsedHierarchyGroups.has(key))
        collapsedHierarchyGroups.delete(key);
      else collapsedHierarchyGroups.add(key);
      apply();
    });
    item.appendChild(toggle);
    item.appendChild(document.createTextNode(label));
    item.appendChild(contents);
    apply();
    return item;
  }

  const list = document.createElement("ul");
  list.className = "tree-nested";

  const count = readNumber(world, "body_count");
  const items: (HTMLLIElement | null)[] = [];

  function refreshSelectionClasses(primary: number) {
    items.forEach((it, i) => {
      if (!it) return;
      it.classList.toggle("selected", i === primary);
      it.classList.toggle(
        "multi-selected",
        hierarchyMultiSelection.has(i) && i !== primary,
      );
    });
  }
  function highlight(index: number) {
    refreshSelectionClasses(index);
  }

  for (let i = 0; i < count; i++) {
    // **削除済みは並べない(群2)**。`remove_body_at` は index のずれを避けるため
    // スロットを残すので、UI 側で隠す必要がある。
    if ((world.read_component("body_is_removed_at", String(i)) === "true")) {
      items.push(null);
      continue;
    }
    const item = document.createElement("li");
    item.textContent = world.read_component("body_label_at", String(i));
    // `tree-body` は **Bodies サブツリーの実体行**だけに付く(群2)。
    // Materials(参照)や Joints の行も `tree-selectable` なので、
    // 「ボディが何体あるか」を数えるにはこちらを使う。
    item.classList.add("tree-selectable", "tree-body");
    item.addEventListener("click", (event) => {
      // Shift クリックで範囲選択(D3「Unityパリティ」増分)。起点
      // (`hierarchyRangeAnchor`)から今クリックした行までの実在ボディ行
      // (削除済みでnullの行は除く)をまとめて選択に加える。Ctrl/Cmd を
      // 併用すると既存の選択へ加算(標準的な意味論)、単独なら選択を
      // 置き換える。起点が無い(初回クリック)場合は単純選択にフォールバック。
      if (event.shiftKey && hierarchyRangeAnchor !== null) {
        if (!(event.ctrlKey || event.metaKey)) hierarchyMultiSelection.clear();
        const lo = Math.min(hierarchyRangeAnchor, i);
        const hi = Math.max(hierarchyRangeAnchor, i);
        for (let k = lo; k <= hi; k++) {
          if (items[k]) hierarchyMultiSelection.add(k);
        }
        // 起点は動かさない(標準のExplorer挙動——続けてShiftクリックすると
        // 同じ起点からの範囲に置き換わる)。
      } else if (event.ctrlKey || event.metaKey) {
        // Ctrl/Cmd クリックで追加選択・解除(設計 §1.1「複数選択可」)。
        if (hierarchyMultiSelection.has(i)) hierarchyMultiSelection.delete(i);
        else hierarchyMultiSelection.add(i);
        hierarchyRangeAnchor = i;
      } else {
        hierarchyMultiSelection.clear();
        hierarchyMultiSelection.add(i);
        hierarchyRangeAnchor = i;
      }
      highlight(i);
      onSelect(i);
    });
    if (actions) {
      item.addEventListener("contextmenu", (event) => {
        event.preventDefault();
        event.stopPropagation();
        // 右クリックした行が選択に入っていなければ、そこだけを選択し直す
        // (Unity と同じ——選択外を右クリックしたら選択が移る)。
        if (!hierarchyMultiSelection.has(i)) {
          hierarchyMultiSelection.clear();
          hierarchyMultiSelection.add(i);
          highlight(i);
          onSelect(i);
        }
        const targets = [...hierarchyMultiSelection].sort((a, b) => a - b);
        const suffix = targets.length > 1 ? ` (${targets.length}件)` : "";
        const isolated = actions.isolatedIndex();
        showContextMenu(event.clientX, event.clientY, [
          {
            label: `複製${suffix}`,
            onSelect: () => targets.forEach((t) => actions.duplicate(t)),
          },
          {
            label: `削除${suffix}`,
            // index 0 は床。削除するとシーンの基準面が無くなるので禁止
            // (`remove_body_at` も Err を返す)。
            disabled: targets.includes(0),
            title: targets.includes(0) ? "床は削除できません" : undefined,
            // 降順に消す(index は詰めない実装だが、意図を明示しておく)。
            onSelect: () =>
              [...targets].reverse().forEach((t) => actions.remove(t)),
          },
          { separator: true },
          {
            label: isolated === i ? "アイソレート解除" : "アイソレート表示",
            title: "選択中のボディ以外を Scene View から隠す(物理は止めない)",
            onSelect: () => actions.isolate(isolated === i ? null : i),
          },
          {
            label: "プレハブ化",
            title: "現在の形状・材質を Project ドロワーの Prefabs へ登録する",
            onSelect: () => actions.capturePrefab(i),
          },
        ]);
      });
    }
    items.push(item);
    list.appendChild(item);
  }
  highlight(BODY_INDEX_BOX);

  bodies.appendChild(makeGroup("bodies", "Bodies", list));

  // Joints(設計§1.1「シーングラフツリー(Bodies/Joints/Circuits/Fluids/
  // Probes/Frames)」)。振り子スポーン(`spawn_pendulum`)が追加した
  // DistanceJointのみが対象(`constraint_anchor_points_at`が空でないボディ)。
  // クリックすると対応するボディを選択する(Joints専用のInspector表示は
  // 未実装のため、現状はBodies側の選択と同じ経路を再利用する)。
  const jointList = document.createElement("ul");
  jointList.className = "tree-nested";
  let jointCount = 0;
  for (let i = 0; i < count; i++) {
    if (world.constraint_anchor_points_at(i).length < 6) continue;
    jointCount += 1;
    const item = document.createElement("li");
    item.textContent = `DistanceJoint (${world.read_component("body_label_at", String(i))})`;
    item.classList.add("tree-selectable");
    item.addEventListener("click", () => {
      highlight(i);
      onSelect(i);
    });
    jointList.appendChild(item);
  }
  if (jointCount > 0) {
    bodies.appendChild(makeGroup("joints", "Joints", jointList));
  }

  // Frames(設計§1.1「シーングラフツリー(...Frames)」、フレーム階層ドリルイン
  // UI)。index 0はROOT(常に存在、`add_child_frame`の既定の親候補だが、それ自体は
  // クリック可能な項目として列挙しない)。1以上の各フレームについて、
  // `frame_parent_index`(親子関係)から再帰的にネストした`<ul>`を組み立てる——
  // 「階層ドリルイン」の名のとおり、クリックしたフレームを選択すると
  // 「+ フレーム」ボタンがそのフレームの子として次のフレームを追加するようになる。
  const frameCount = readNumber(world, "frame_count");
  if (frameCount > 1) {
    function buildFrameSubtree(parentIndex: number): HTMLUListElement {
      const ul = document.createElement("ul");
      ul.className = "tree-nested";
      for (let i = 1; i < frameCount; i++) {
        if (Number(world.read_component("frame_parent_index", String(i))) !== parentIndex) continue;
        const item = document.createElement("li");
        item.textContent = `Frame ${i}`;
        item.classList.add("tree-selectable");
        item.classList.toggle("selected", i === selectedFrameIndex);
        item.addEventListener("click", (event) => {
          event.stopPropagation();
          onSelectFrame(i);
        });
        item.appendChild(buildFrameSubtree(i));
        ul.appendChild(item);
      }
      return ul;
    }
    bodies.appendChild(makeGroup("frames", "Frames", buildFrameSubtree(0)));
  }

  // Fluids(設計§1.1「シーングラフツリー(...Fluids)」)。個々の粒子や塊単位の
  // 選択・ドリルインまでは実装せず(SPH粒子は`RigidBodySet`のような個別ID体系を
  // 持たないため)、スポーンした水塊の数+総粒子数の概要表示のみとする
  // (縮約実装、`spawn_fluid_block`が複数回スポーンで水塊を追加できるように
  // なったことを受けての最小限のHierarchy反映)。
  const fluidSpawnCount = readNumber(world, "fluid_spawn_count");
  if (fluidSpawnCount > 0) {
    const fluidItem = document.createElement("li");
    fluidItem.textContent = `Fluids (${fluidSpawnCount}塊, ${readNumber(world, "fluid_particle_count")}粒子)`;
    bodies.appendChild(fluidItem);
  }

  // 3D格子流体(**群9で追加**)。3Dの場をブラウザで可視化する経路は無いので、
  // ドメインが載っていること自体が見えるよう概要行だけを出す(縮約、
  // `grid_fluid_3d_summary`のdoc参照)。
  const gridFluid3dSummary = world.read_component("grid_fluid_3d_summary", "");
  if (gridFluid3dSummary.length > 0) {
    const item = document.createElement("li");
    item.textContent = gridFluid3dSummary;
    bodies.appendChild(item);
  }

  // Circuits(設計§1.1「シーングラフツリー(...Circuits)」、**増分G2で追加**)。
  // 増分G2で`sim-em::Circuit`へ足した素子アクセサ(それまで全フィールドが
  // privateで、載っている素子を外から数える手段が無かった)を`sim-wasm`の
  // `circuit_element_count`/`circuit_element_label_at`経由で列挙する。
  // **縮約**: Probesと同じ理由で個々の素子は選択対象にしない
  // (Inspectorに回路素子用のComponent表示が無い)。
  const circuitElementCount = readNumber(world, "circuit_element_count");
  if (circuitElementCount > 0) {
    const circuitList = document.createElement("ul");
    circuitList.className = "tree-nested";
    for (let i = 0; i < circuitElementCount; i++) {
      const item = document.createElement("li");
      item.textContent = world.read_component("circuit_element_label_at", String(i));
      circuitList.appendChild(item);
    }
    bodies.appendChild(makeGroup("circuits", "Circuits", circuitList));
  }

  // Probes(設計§1.1「シーングラフツリー(...Probes)」、増分E2で追加)。
  // シーンJSONの`probes`セクションが宣言したプローブを、増分B1で追加した
  // `imported_probe_count`/`imported_probe_label_at`(`ProbeTarget`の11変種を
  // 人間可読ラベルへ変換したもの)から列挙する。**D9(熱のみ)・D34/D35
  // (天体のみ)のようにScene Viewに何も描かれないシーンでは、これがシーンに
  // 何が定義されているかを知る唯一のツリー上の手がかりになる**。
  // 既定シーン(`WasmWorld::new`)は`scenario.probes`を持たないため0本で、
  // その場合はサブツリー自体を出さない。
  // **縮約**: プローブは選択対象にしない(Inspectorに専用のComponent表示が
  // 無いため、クリックしても見せるものが無い)。現在値はProbe Graphsパネルの
  // 凡例が既に出している。
  const probeCount = readNumber(world, "imported_probe_count");
  if (probeCount > 0) {
    const probeList = document.createElement("ul");
    probeList.className = "tree-nested";
    for (let i = 0; i < probeCount; i++) {
      const item = document.createElement("li");
      item.textContent = world.read_component("imported_probe_label_at", String(i));
      probeList.appendChild(item);
    }
    bodies.appendChild(makeGroup("probes", "Probes", probeList));
  }

  // **Materials(群2)**。設計 §1.1 は「Bodies / Joints / Circuits / Fluids /
  // Probes / Frames(フレーム階層)/ **Materials(参照)**」と列挙しているが、
  // Materials だけツリーに無かった(Project ドロワーの Materials タブには
  // 物性表があるが、それは「参照」ではなく一覧)。
  // ここでは**このシーンで実際に使われている材質**だけを出し、各材質の下に
  // それを使っているボディを並べる——これが設計の言う「参照」。
  const materialUsers = new Map<string, number[]>();
  for (let i = 0; i < count; i++) {
    if ((world.read_component("body_is_removed_at", String(i)) === "true")) continue;
    const name = world.read_component("body_material_label_at", String(i));
    const users = materialUsers.get(name);
    if (users) users.push(i);
    else materialUsers.set(name, [i]);
  }
  if (materialUsers.size > 0) {
    const materialList = document.createElement("ul");
    materialList.className = "tree-nested";
    // 表示順は `SPAWN_MATERIALS` の順(決定的)、その後に未知の材質。
    const ordered = [
      ...materialNames.filter((n) => materialUsers.has(n)),
      ...[...materialUsers.keys()].filter((n) => !materialNames.includes(n)),
    ];
    for (const name of ordered) {
      const users = materialUsers.get(name)!;
      const item = document.createElement("li");
      item.textContent = `${name} (${users.length})`;
      const userList = document.createElement("ul");
      userList.className = "tree-nested";
      for (const bodyIndex of users) {
        const userItem = document.createElement("li");
        // **「↳」を付けて Bodies の行と区別する**。同じラベルの行がツリー内に
        // 2つ現れると、見た目にどちらが実体でどちらが参照か分からないうえ、
        // ラベルでの選択(テスト・自動化)も曖昧になる(実際に Playwright の
        // strict モードが 8 本まとめて落ちて気付いた)。
        userItem.textContent = `↳ ${world.read_component("body_label_at", String(bodyIndex))}`;
        userItem.classList.add("tree-selectable");
        userItem.addEventListener("click", (event) => {
          event.stopPropagation();
          highlight(bodyIndex);
          onSelect(bodyIndex);
        });
        userList.appendChild(userItem);
      }
      item.appendChild(userList);
      materialList.appendChild(item);
    }
    bodies.appendChild(
      makeGroup("materials", "Materials (参照)", materialList),
    );
  }

  root.appendChild(bodies);
  tree.appendChild(root);

  // **検索/絞り込み(D3「Unityパリティ」増分)**。監査で見つかった具体的な
  // 欠落——ボディ数が多いシーン(散乱球群等)でHierarchyから目的の行を
  // 探す手段が無かった。ツリーの構造(Bodies/Joints/Frames/Circuits/Probes/
  // Materialsでそれぞれ行の組み立て方が違う)に依存しないよう、DOM上の
  // `<li>`をすべて舐めて「自身の直接のテキスト」が一致するか、子孫に一致が
  // あるかだけで判定する汎用フィルタにした(セクションごとの専用ロジックを
  // 増やさない)。
  const searchInput = document.getElementById(
    "hierarchy-search",
  ) as HTMLInputElement | null;
  function applyHierarchyFilter(query: string) {
    const q = query.trim().toLowerCase();
    const active = q.length > 0;
    tree.classList.toggle("hierarchy-filtering", active);
    const allLis = Array.from(tree.querySelectorAll<HTMLLIElement>("li"));
    if (!active) {
      // 折り畳み状態(`collapsedHierarchyGroups`)を管理しているのは各グループの
      // 開閉トグルが直接触る`contents.style.display`(`<ul>`側)であって、ここで
      // 触っているのは`<li>`側の`display`だけなので、消すだけで元の折り畳み
      // 表示へ戻る。
      allLis.forEach((li) => li.style.removeProperty("display"));
      return;
    }
    // 文書順(親→子)で得られる配列を逆順に辿ることで、子から先に可視判定を
    // 終わらせてから親の「子孫に一致があるか」を見られるようにする。
    for (let k = allLis.length - 1; k >= 0; k--) {
      const li = allLis[k];
      const ownText = Array.from(li.childNodes)
        .filter((n) => n.nodeType === Node.TEXT_NODE)
        .map((n) => n.textContent ?? "")
        .join("")
        .toLowerCase();
      const hasVisibleChild = Array.from(li.children).some(
        (child) =>
          child.tagName === "UL" &&
          Array.from(child.children).some(
            (c) => (c as HTMLElement).style.display !== "none",
          ),
      );
      li.style.display = ownText.includes(q) || hasVisibleChild ? "" : "none";
    }
  }
  if (searchInput) {
    applyHierarchyFilter(searchInput.value);
    // `addEventListener`だと`setUpHierarchy`が呼ばれるたび(スポーン/複製/
    // 削除のたびに再構築される)ハンドラが積み重なる——`<input>`自体はツリーの
    // 外にありツリー再構築(`tree.innerHTML = ""`)を生き延びるため。代入は
    // 前回分を置き換えるだけなので安全。
    searchInput.oninput = () => applyHierarchyFilter(searchInput.value);
  }

  applyHierarchyAriaRoles(tree);
  // `highlight` は Scene View のピッキング等から後で呼ばれるので、選択状態の
  // `aria-selected` もそこで掛け直す(class だけ変えて ARIA が古いままだと、
  // 読み上げには前の選択が残る)。
  const highlightWithAria = (index: number) => {
    highlight(index);
    applyHierarchyAriaRoles(tree);
  };
  return highlightWithAria;
}

// Inspectorパネル(設計docs/23-frontend/01-editor.md §1.3)。選択中ボディの
// Shape/Material(`world.body_shape_label_at`/`body_material_label_at`、
// スポーンパレットで追加したボディも含めて実際にクエリできる)+ Transform
// (毎フレーム実データで更新、`updateInspectorTransformFields`)を表示する。
//
// **2026-07-28のD9/D34/D35増分で追加したガード**: `index`が`readNumber(world, "body_count")`
// の範囲外(D9=熱のみ・D34/D35=天体のみのように力学ボディを1つも持たない
// ギャラリーシーンを読み込んだ直後、`index`に渡り得る`0`を含む)なら
// `world.body_label_at`等(いずれも`Result`化済みでボディが無ければ`Err`を
// throwする)を呼ばず、代わりに「選択中のボディはない」プレースホルダを表示する
// (呼ばずに済ませる以外に無害な選択肢が無いため——空文字列や`-1`を渡しても
// 同じくRust側がエラーを返す)。`updateInspectorTransformFields`は
// `#inspector-position`等のDOM要素が無ければ何もしない null-safe 実装のため、
// このプレースホルダ表示と両立する。
function renderInspectorFor(world: WasmWorld, index: number): void {
  const body = document.getElementById("inspector-body")!;
  if (index < 0 || index >= readNumber(world, "body_count")) {
    body.innerHTML = `
      <div class="empty-state">
        <p>選択中のボディはありません。</p>
        <p>このシーンには力学ボディがありません——Probe Graphs パネルや Scene View の場のパネルで観測してください。</p>
      </div>
    `;
    return;
  }
  const label = world.read_component("body_label_at", String(index));
  const staticBadge = (world.read_component("body_is_static_at", String(index)) === "true")
    ? ' <span class="badge">Static</span>'
    : "";
  // `body_position_at_f32`はWasmメモリを直接指す一時的なビューを返す(B16、
  // `crates/sim-wasm/src/lib.rs`の`HotPathViewBuffers`のdoc参照)ため、下の
  // テンプレートリテラルが他のWasm呼び出し(`read_component`)を挟んで
  // 参照する前に、ここで即座にプレーンな配列へ読み切っておく。
  const initialPosition = Array.from(world.body_position_at_f32(index));
  body.innerHTML = `
    <div class="inspector-component">
      <h3>${label}${staticBadge}</h3>
      <div class="inspector-field"><span>Shape</span><span>${world.read_component("body_shape_label_at", String(index))}</span></div>
    </div>
    <div class="inspector-component">
      <h3>Transform</h3>
      <div class="inspector-field">
        <span>Position (x,y,z)</span>
        <span class="inspector-scale-fields">
          <input type="number" id="inspector-position-x" step="0.05" value="${initialPosition[0]}" />
          <input type="number" id="inspector-position-y" step="0.05" value="${initialPosition[1]}" />
          <input type="number" id="inspector-position-z" step="0.05" value="${initialPosition[2]}" />
        </span>
      </div>
      <div class="inspector-field"><span>Rotation</span><span id="inspector-rotation">—</span></div>
      <div class="inspector-field"><span>Velocity</span><span id="inspector-velocity">—</span></div>
      <div class="inspector-field">
        <span>Scale (x,y,z)</span>
        <span class="inspector-scale-fields">
          <input type="number" id="inspector-scale-x" min="0.01" step="0.1" value="1" />
          <input type="number" id="inspector-scale-y" min="0.01" step="0.1" value="1" />
          <input type="number" id="inspector-scale-z" min="0.01" step="0.1" value="1" />
        </span>
      </div>
      <p class="inspector-note">軸別スケールは Box のみ(球・カプセルは形状表現に非等方の自由度が無い)。3欄を同じ値にすると等方スケールとして球にも効く。</p>
    </div>
    ${renderRigidBodyComponent(world, index)}
    ${renderInspectorExtraComponents(world, index)}
  `;
  wireInspectorEditFields(index);
  wireAddJointForm(world, index);
  wireAddCouplingForm(world, index);
  wireCouplingControlSurfaceInputs(world);
  wireThrustForm(index);
}

/// wasm境界を`schema`/`read`/`apply`の3メソッドへ畳む取り組み(**残タスク
/// 完遂増分**、Task#8第一弾)向けの薄いラッパー。`apply_component`は成功時
/// JSON文字列(作成系は`{"index":N}`、それ以外は`{}`)を返す——呼び出し側は
/// パース済みの値として受け取る。失敗時は他のwasmメソッドと同じく例外を投げる
/// (呼び出し元の`try`/`catch`はそのまま使える)。
function applyComponent(
  world: WasmWorld,
  kind: string,
  payload: Record<string, number | string | boolean>,
): { index?: number; applied?: boolean } {
  return JSON.parse(world.apply_component(kind, JSON.stringify(payload))) as {
    index?: number;
    applied?: boolean;
  };
}

/// `read_component`で数値スカラーを読む薄いラッパー(Task#8第二弾)。
function readNumber(world: WasmWorld, kind: string, arg = ""): number {
  return Number(world.read_component(kind, arg));
}

// ---------------------------------------------------------------------------
// 量子ドメイン(1D/2D)の追加プリセット(**B9「エディタのUIプリセットとして
// 残す」**)。
//
// `crates/sim-world/src/scenario.rs`のモジュールdocが説明するとおり、シーンJSON
// スキーマからは「よく使う3形」を列挙する構築レシピ(ガウス波束・矩形障壁・
// 調和振動子)が撤去され、`raw_state`(任意の波動関数・ポテンシャルの生配列)だけが
// 唯一の表現になった——持続する状態にとっては表現力で上回る正しい判断だが、副作用と
// して「D27/D28のようなよく使うテキストブック的初期条件をエディタから手軽に置く
// 手段」も一緒に消えた(手でpsi_re/psi_im/vをbase64エンコードして書く以外に量子
// ドメインを新規作成する経路が無い)。
//
// ここでは撤去された構築レシピと同じ発想(ガウス波束+ポテンシャル3〜4形)を
// **エディタ側だけのプリセット**として復活させる。計算は全てTypeScript側で行い、
// 結果の生配列を`enable_quantum_1d_domain`/`enable_quantum_2d_domain`
// (`apply_component`の新kind、`crates/sim-wasm/src/lib.rs`)へ渡すだけ——シーン
// JSONスキーマには一切触れない(スキーマが`raw_state`だけを唯一の真として保つ設計
// はそのまま)。
// ---------------------------------------------------------------------------

/// f64配列をLE(リトルエンディアン)+base64(RFC 4648 §4、標準アルファベット)へ
/// 符号化する。`crates/sim-world/src/raw_bytes.rs`の`encode_f64_le_base64`と同じ
/// 表現(長さヘッダ等のフレーミングは無い、素のバイト列)——量子ドメインのプリセット
/// が唯一の呼び出し元。外部パッケージを増やさない方針(README「依存が実質ゼロ」)の
/// もと、`btoa`(ブラウザ組み込み)の上に手で組む。
function encodeF64LeBase64(values: readonly number[]): string {
  const bytes = new Uint8Array(values.length * 8);
  const view = new DataView(bytes.buffer);
  values.forEach((value, i) => view.setFloat64(i * 8, value, true));
  let binary = "";
  for (let i = 0; i < bytes.length; i += 1) binary += String.fromCharCode(bytes[i]);
  return btoa(binary);
}

/// 1Dガウス波束 $\psi(x)=\exp[-(x-x_0)^2/(4\sigma^2)+ik_0x]$、離散ノルム
/// $\sum_i|\psi_i|^2dx=1$へ正規化する(`sim_quantum::WaveFunction1D::
/// set_gaussian_wave_packet`と同じ式・同じ規約、格子点は$x_i=i\cdot dx$)。
function quantum1dGaussianWavePacket(
  n: number,
  dx: number,
  x0: number,
  sigma: number,
  k0: number,
): { psiRe: number[]; psiIm: number[] } {
  const psiRe = new Array<number>(n);
  const psiIm = new Array<number>(n);
  for (let i = 0; i < n; i += 1) {
    const x = i * dx;
    const envelope = Math.exp(-((x - x0) ** 2) / (4 * sigma * sigma));
    psiRe[i] = envelope * Math.cos(k0 * x);
    psiIm[i] = envelope * Math.sin(k0 * x);
  }
  let norm = 0;
  for (let i = 0; i < n; i += 1) norm += psiRe[i] * psiRe[i] + psiIm[i] * psiIm[i];
  norm *= dx;
  const scale = 1 / Math.sqrt(norm);
  for (let i = 0; i < n; i += 1) {
    psiRe[i] *= scale;
    psiIm[i] *= scale;
  }
  return { psiRe, psiIm };
}

/// 1Dの矩形ポテンシャル。`height`に正の値を渡せば障壁、負の値を渡せば井戸になる
/// (符号は呼び出し側が決める)。中心`center`・幅`width`はいずれも[a.u.]。
function quantum1dRectangularPotential(
  n: number,
  dx: number,
  height: number,
  width: number,
  center: number,
): number[] {
  const v = new Array<number>(n).fill(0);
  const lo = center - width / 2;
  const hi = center + width / 2;
  for (let i = 0; i < n; i += 1) {
    const x = i * dx;
    if (x >= lo && x < hi) v[i] = height;
  }
  return v;
}

/// 1D調和振動子ポテンシャル $V(x)=\frac12\omega^2(x-x_c)^2$。
function quantum1dHarmonicPotential(
  n: number,
  dx: number,
  omega: number,
  center: number,
): number[] {
  const v = new Array<number>(n);
  for (let i = 0; i < n; i += 1) {
    const x = i * dx - center;
    v[i] = 0.5 * omega * omega * x * x;
  }
  return v;
}

/// 2Dガウス波束(`sim_quantum::WaveFunction2D::set_gaussian_wave_packet`と同じ式、
/// +x方向へ運動量$k_0$)。行優先(`index = iy*nx+ix`、`Quantum2dRawStateJson`の
/// docと同じ規約)。
function quantum2dGaussianWavePacket(
  nx: number,
  ny: number,
  dx: number,
  dy: number,
  x0: number,
  y0: number,
  sigmaX: number,
  sigmaY: number,
  k0: number,
): { psiRe: number[]; psiIm: number[] } {
  const psiRe = new Array<number>(nx * ny);
  const psiIm = new Array<number>(nx * ny);
  for (let iy = 0; iy < ny; iy += 1) {
    const y = iy * dy;
    for (let ix = 0; ix < nx; ix += 1) {
      const x = ix * dx;
      const envelope = Math.exp(
        -((x - x0) ** 2) / (4 * sigmaX * sigmaX) - ((y - y0) ** 2) / (4 * sigmaY * sigmaY),
      );
      const idx = iy * nx + ix;
      psiRe[idx] = envelope * Math.cos(k0 * x);
      psiIm[idx] = envelope * Math.sin(k0 * x);
    }
  }
  let norm = 0;
  for (let i = 0; i < psiRe.length; i += 1) norm += psiRe[i] * psiRe[i] + psiIm[i] * psiIm[i];
  norm *= dx * dy;
  const scale = 1 / Math.sqrt(norm);
  for (let i = 0; i < psiRe.length; i += 1) {
    psiRe[i] *= scale;
    psiIm[i] *= scale;
  }
  return { psiRe, psiIm };
}

/// 2D二重スリット障壁(`scenes/d27-double-slit.json`と同じ構成——x方向1格子点厚の
/// 壁を`ix=nx/2`に置き、y方向中心を挟んで対称な幅`slitWidth`のスリットを2本開ける、
/// 中心間隔`slitSeparation`)。
function quantum2dDoubleSlitPotential(
  nx: number,
  ny: number,
  dy: number,
  height: number,
  slitWidth: number,
  slitSeparation: number,
): number[] {
  const v = new Array<number>(nx * ny).fill(0);
  const barrierIx = Math.floor(nx / 2);
  const yCenter = (ny * dy) / 2;
  for (let iy = 0; iy < ny; iy += 1) {
    const y = iy * dy - yCenter;
    const inSlit1 = Math.abs(y - slitSeparation / 2) < slitWidth / 2;
    const inSlit2 = Math.abs(y + slitSeparation / 2) < slitWidth / 2;
    if (!inSlit1 && !inSlit2) v[iy * nx + barrierIx] = height;
  }
  return v;
}

/// 2D調和振動子ポテンシャル $V(x,y)=\frac12\omega^2[(x-c_x)^2+(y-c_y)^2]$。
function quantum2dHarmonicPotential(
  nx: number,
  ny: number,
  dx: number,
  dy: number,
  omega: number,
  cx: number,
  cy: number,
): number[] {
  const v = new Array<number>(nx * ny);
  for (let iy = 0; iy < ny; iy += 1) {
    const y = iy * dy - cy;
    for (let ix = 0; ix < nx; ix += 1) {
      const x = ix * dx - cx;
      v[iy * nx + ix] = 0.5 * omega * omega * (x * x + y * y);
    }
  }
  return v;
}

/// 2の冪のグリッドサイズだけを許す(select要素の`<option>`が既に2の冪しか
/// 提示しないが、タスク仕様「enforce/round in the UI ... rather than accepting
/// arbitrary integers that would fail Rust-side validation」に沿って呼び出し側でも
/// 明示的に検証する)。
function isPowerOfTwo(n: number): boolean {
  return Number.isInteger(n) && n > 0 && (n & (n - 1)) === 0;
}

/// 量子ドメイン(1D/2D)の「＋追加」フォームを配線する(**B9**)。Add Joint/Add
/// Coupling(B11〜B15)と違って`component_schema`駆動ではない——ここのプリセット
/// 計算(ガウス波束・ポテンシャル形状)はシーンJSONスキーマに存在しない、エディタ
/// 限定の変換だからである(モジュール冒頭のdoc参照)。設定ポップオーバー内の静的
/// フォーム(`demo/index.html`)を直接読み書きするだけの薄い配線に留める。
///
/// `enable_grid_fluid_2d_domain`と違って**冪等ではない**——ボタンを押すたびに
/// フォームの現在値で生状態を丸ごと置き換える(`sim-wasm`側`enable_quantum_1d_
/// domain_impl`のdocと同じ理由)。
///
/// **`world`を値ではなく`getWorld`(取得関数)で受け取る**: `setUpSceneView`は
/// ギャラリーシーンを読み込むたび`let world`変数を丸ごと差し替える
/// (`world = WasmWorld.from_scene_json(json)`)。この配線自体は`btn-enable-grid-
/// fluid`と同じくシーン読み込み時に1回しか呼ばれないため、`world`をそのまま値で
/// 受け取って閉包すると、シーン切り替え後もボタンが**差し替え前の破棄済み
/// `WasmWorld`**へ向かって送信し続けてしまう。クリック時に`getWorld()`を呼び直す
/// ことで、常にその時点の現在の`world`を参照する。
function wireAddQuantumDomainForms(getWorld: () => WasmWorld): void {
  // --- 1D ---
  const nSelect = document.getElementById("select-quantum1d-n") as HTMLSelectElement | null;
  const dxInput = document.getElementById("input-quantum1d-dx") as HTMLInputElement | null;
  const x0Input = document.getElementById("input-quantum1d-x0") as HTMLInputElement | null;
  const sigmaInput = document.getElementById("input-quantum1d-sigma") as HTMLInputElement | null;
  const k0Input = document.getElementById("input-quantum1d-k0") as HTMLInputElement | null;
  const potential1dSelect = document.getElementById(
    "select-quantum1d-potential",
  ) as HTMLSelectElement | null;
  const v0Input = document.getElementById("input-quantum1d-v0") as HTMLInputElement | null;
  const widthInput = document.getElementById("input-quantum1d-width") as HTMLInputElement | null;
  const centerInput = document.getElementById(
    "input-quantum1d-center",
  ) as HTMLInputElement | null;
  const omega1dInput = document.getElementById(
    "input-quantum1d-omega",
  ) as HTMLInputElement | null;
  const add1dButton = document.getElementById("btn-add-quantum-1d");

  const updateVisibility1d = () => {
    const preset = potential1dSelect?.value ?? "zero";
    document.querySelectorAll<HTMLElement>("[data-quantum1d-potential]").forEach((el) => {
      el.hidden = !(el.dataset.quantum1dPotential ?? "").split(" ").includes(preset);
    });
  };
  potential1dSelect?.addEventListener("change", updateVisibility1d);
  updateVisibility1d();

  add1dButton?.addEventListener("click", () => {
    const n = Number(nSelect?.value ?? 256);
    const dx = Number(dxInput?.value ?? 0.1);
    const x0 = Number(x0Input?.value ?? 0);
    const sigma = Number(sigmaInput?.value ?? 1);
    const k0 = Number(k0Input?.value ?? 0);
    if (!isPowerOfTwo(n)) {
      reportError("グリッド点数 n は2の冪(64/128/256/512/1024)から選んでください。");
      return;
    }
    if (!Number.isFinite(dx) || dx <= 0 || !Number.isFinite(sigma) || sigma <= 0) {
      reportError("dx・σ には正の有限値を入力してください。");
      return;
    }
    const { psiRe, psiIm } = quantum1dGaussianWavePacket(n, dx, x0, sigma, k0);
    const preset = potential1dSelect?.value ?? "zero";
    const center = Number(centerInput?.value ?? (n * dx) / 2);
    let v: number[];
    switch (preset) {
      case "barrier":
        v = quantum1dRectangularPotential(
          n,
          dx,
          Number(v0Input?.value ?? 0),
          Number(widthInput?.value ?? 0),
          center,
        );
        break;
      case "well":
        v = quantum1dRectangularPotential(
          n,
          dx,
          -Math.abs(Number(v0Input?.value ?? 0)),
          Number(widthInput?.value ?? 0),
          center,
        );
        break;
      case "harmonic":
        v = quantum1dHarmonicPotential(n, dx, Number(omega1dInput?.value ?? 1), center);
        break;
      default:
        v = new Array(n).fill(0);
    }
    try {
      applyComponent(getWorld(), "enable_quantum_1d_domain", {
        psi_re: encodeF64LeBase64(psiRe),
        psi_im: encodeF64LeBase64(psiIm),
        v: encodeF64LeBase64(v),
        dx,
      });
    } catch (err) {
      reportError(`量子ドメイン(1D)の追加に失敗しました: ${String(err)}`);
    }
  });

  // --- 2D ---
  const nxSelect = document.getElementById("select-quantum2d-nx") as HTMLSelectElement | null;
  const nySelect = document.getElementById("select-quantum2d-ny") as HTMLSelectElement | null;
  const dx2dInput = document.getElementById("input-quantum2d-dx") as HTMLInputElement | null;
  const dy2dInput = document.getElementById("input-quantum2d-dy") as HTMLInputElement | null;
  const x02dInput = document.getElementById("input-quantum2d-x0") as HTMLInputElement | null;
  const y02dInput = document.getElementById("input-quantum2d-y0") as HTMLInputElement | null;
  const sigmaX2dInput = document.getElementById(
    "input-quantum2d-sigma-x",
  ) as HTMLInputElement | null;
  const sigmaY2dInput = document.getElementById(
    "input-quantum2d-sigma-y",
  ) as HTMLInputElement | null;
  const k02dInput = document.getElementById("input-quantum2d-k0") as HTMLInputElement | null;
  const potential2dSelect = document.getElementById(
    "select-quantum2d-potential",
  ) as HTMLSelectElement | null;
  const v02dInput = document.getElementById("input-quantum2d-v0") as HTMLInputElement | null;
  const slitWidthInput = document.getElementById(
    "input-quantum2d-slit-width",
  ) as HTMLInputElement | null;
  const slitSeparationInput = document.getElementById(
    "input-quantum2d-slit-separation",
  ) as HTMLInputElement | null;
  const omega2dInput = document.getElementById(
    "input-quantum2d-omega",
  ) as HTMLInputElement | null;
  const add2dButton = document.getElementById("btn-add-quantum-2d");

  const updateVisibility2d = () => {
    const preset = potential2dSelect?.value ?? "zero";
    document.querySelectorAll<HTMLElement>("[data-quantum2d-potential]").forEach((el) => {
      el.hidden = !(el.dataset.quantum2dPotential ?? "").split(" ").includes(preset);
    });
  };
  potential2dSelect?.addEventListener("change", updateVisibility2d);
  updateVisibility2d();

  add2dButton?.addEventListener("click", () => {
    const nx = Number(nxSelect?.value ?? 64);
    const ny = Number(nySelect?.value ?? 64);
    const dx = Number(dx2dInput?.value ?? 0.2);
    const dy = Number(dy2dInput?.value ?? 0.2);
    const x0 = Number(x02dInput?.value ?? 0);
    const y0 = Number(y02dInput?.value ?? (ny * dy) / 2);
    const sigmaX = Number(sigmaX2dInput?.value ?? 1);
    const sigmaY = Number(sigmaY2dInput?.value ?? 1);
    const k0 = Number(k02dInput?.value ?? 0);
    if (!isPowerOfTwo(nx) || !isPowerOfTwo(ny)) {
      reportError("グリッド点数 nx/ny は2の冪(32/64/128/256)から選んでください。");
      return;
    }
    if (
      !Number.isFinite(dx) ||
      dx <= 0 ||
      !Number.isFinite(dy) ||
      dy <= 0 ||
      !Number.isFinite(sigmaX) ||
      sigmaX <= 0 ||
      !Number.isFinite(sigmaY) ||
      sigmaY <= 0
    ) {
      reportError("dx・dy・σx・σy には正の有限値を入力してください。");
      return;
    }
    const { psiRe, psiIm } = quantum2dGaussianWavePacket(
      nx,
      ny,
      dx,
      dy,
      x0,
      y0,
      sigmaX,
      sigmaY,
      k0,
    );
    const preset = potential2dSelect?.value ?? "zero";
    let v: number[];
    switch (preset) {
      case "double_slit":
        v = quantum2dDoubleSlitPotential(
          nx,
          ny,
          dy,
          Number(v02dInput?.value ?? 0),
          Number(slitWidthInput?.value ?? 0),
          Number(slitSeparationInput?.value ?? 0),
        );
        break;
      case "harmonic":
        v = quantum2dHarmonicPotential(
          nx,
          ny,
          dx,
          dy,
          Number(omega2dInput?.value ?? 1),
          (nx * dx) / 2,
          (ny * dy) / 2,
        );
        break;
      default:
        v = new Array(nx * ny).fill(0);
    }
    try {
      applyComponent(getWorld(), "enable_quantum_2d_domain", {
        psi_re: encodeF64LeBase64(psiRe),
        psi_im: encodeF64LeBase64(psiIm),
        v: encodeF64LeBase64(v),
        nx,
        ny,
        dx,
        dy,
      });
    } catch (err) {
      reportError(`量子ドメイン(2D)の追加に失敗しました: ${String(err)}`);
    }
  });
}

/// `component_schema`が返す`apply`側スキーマの構造(`crates/sim-wasm/src/
/// component_schema.rs`の`ComponentFieldSchema`/`ComponentKindSchema`と1:1)。
/// Add Joint / Add Coupling フォームをこのスキーマから生成する(B11/B12〜B15、
/// 「Add Joint / Add Coupling をスキーマ駆動フォームにする」)。
///
/// **これが無かった間の縮約(実害)**: 以前はAdd Joint(5種)・Add
/// Coupling(16種)とも「Body A/B・Anchor A/B・Axis・Param 1〜6」という
/// 単一の汎用フォームを全kindで共有し、選択中kindにとって各欄が実際に
/// 何を意味するかは`title`属性のツールチップにしか書かれていなかった
/// (`add_wheel_joint`の`Param 1`は`rest_length`、`add_hinge_motor_joint`の
/// `Param 1`は`theta_target[rad]`、ホバーしなければ分からない)。
/// `apply_component`側は`component_schema`(Task#9)で既に`_impl`の実引数と
/// 1:1のフィールド名・型・単位・既定値を返せるようになっていたので、
/// フロントエンドをそこへ繋ぎ直すのがこの増分。
type ApplyFieldType = "f64" | "usize" | "i32" | "string" | "bool";
type ApplyFieldSchema = {
  name: string;
  type: ApplyFieldType;
  unit: string | null;
  default: number | string | boolean | null;
  nullable: boolean;
  min: number | null;
  max: number | null;
};
type ApplyKindSchema = { kind: string; fields: ApplyFieldSchema[] };

/// `component_schema`はRustコード自体(`apply_component_impl`の`match kind`)
/// から導かれる静的な表であり、worldの実行状態には依存しない——Inspector
/// 再描画のたびに`JSON.parse`をやり直さないよう、初回結果をモジュール
/// スコープへキャッシュする。
let applySchemaCache: ApplyKindSchema[] | null = null;
function applySchema(world: WasmWorld): ApplyKindSchema[] {
  if (!applySchemaCache) {
    const parsed = JSON.parse(world.component_schema()) as {
      apply: ApplyKindSchema[];
    };
    applySchemaCache = parsed.apply;
  }
  return applySchemaCache;
}

/// `kind`名(`add_distance_joint`等)からそのフィールドスキーマを引く。
/// `component_schema_covers_every_apply_kind`(lib.rsのネイティブテスト)が
/// 「スキーマに載る全kindが`apply_component_impl`のディスパッチに実在する
/// こと」を守っているので、ここで見付からないのは呼び出し側(このファイル)の
/// kind名の綴りミスだけである。
function applyKindSchema(world: WasmWorld, kind: string): ApplyKindSchema {
  const found = applySchema(world).find((entry) => entry.kind === kind);
  if (!found) {
    throw new Error(`component_schema に kind="${kind}" が無い(呼び出し側の綴りミス)`);
  }
  return found;
}

/// フィールドの初期UI値。**スキーマの`default`(「省略時に渡る値」)を
/// そのまま初期表示に使うと踏む罠が2つある**(このフォームは常に全
/// フィールドを送るので「省略」自体は起きないが、初期値としては不適切):
///
/// ① 先頭の剛体引数(`body_a`/`body`/`chassis`)は選択中ボディを既定に
/// したい(以前のBody Aの挙動をそのまま踏襲)。
/// ② `body_b`(nullableなi32、負値がワールド固定点のセンチネル)は
/// スキーマの既定`0`のまま出すと「省略すると床に繋がる」という罠を
/// 初期表示自体が体現してしまう——ワールド固定点`-1`を初期値にする
/// (以前のBody Bの挙動をそのまま踏襲)。
///
/// この2つ以外は素直にスキーマの`default`を使う。
function initialApplyFieldValue(
  field: ApplyFieldSchema,
  selectedBodyIndex: number,
): number | string | boolean {
  if (field.type === "i32" && field.name === "body_b") return -1;
  if (
    field.type === "usize" &&
    (field.name === "body_a" || field.name === "body" || field.name === "chassis")
  ) {
    return selectedBodyIndex;
  }
  if (field.default !== null) return field.default;
  switch (field.type) {
    case "string":
      return "";
    case "bool":
      return false;
    default:
      return 0;
  }
}

function applyFieldInputId(prefix: string, field: ApplyFieldSchema): string {
  return `${prefix}-field-${field.name}`;
}

/// スキーマ1フィールドぶんの入力欄HTML。ラベルは`name`そのもの(+単位・
/// 値域・センチネルの注記)——「どのkindで何を意味するか」がラベルだけで
/// 分かるようにする(以前の`title`ツールチップだけに頼る構成をやめる)。
function renderApplyField(
  prefix: string,
  field: ApplyFieldSchema,
  selectedBodyIndex: number,
): string {
  const escape = (text: string) =>
    text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  const id = applyFieldInputId(prefix, field);
  const unitSuffix = field.unit ? ` [${field.unit}]` : "";
  const rangeSuffix =
    field.min !== null || field.max !== null
      ? ` (${field.min ?? "-∞"}〜${field.max ?? "∞"})`
      : "";
  const nullableSuffix = field.nullable ? " ※負値/特定値がセンチネル" : "";
  const label = `${escape(field.name)}${unitSuffix}${rangeSuffix}${nullableSuffix}`;
  const initial = initialApplyFieldValue(field, selectedBodyIndex);
  if (field.type === "bool") {
    return (
      `<div class="inspector-field"><label>` +
      `<input type="checkbox" id="${id}" ${initial ? "checked" : ""} /> ${label}` +
      `</label></div>`
    );
  }
  if (field.type === "string") {
    return (
      `<div class="inspector-field"><span>${label}</span>` +
      `<input type="text" id="${id}" value="${escape(String(initial))}" /></div>`
    );
  }
  const step = field.type === "f64" ? "0.1" : "1";
  const min = field.type === "usize" ? 0 : field.min;
  return (
    `<div class="inspector-field"><span>${label}</span>` +
    `<input type="number" id="${id}" step="${step}"` +
    `${min !== null ? ` min="${min}"` : ""}` +
    `${field.max !== null ? ` max="${field.max}"` : ""}` +
    ` value="${initial}" /></div>`
  );
}

/// 指定した`kind`のフィールド一覧をコンテナへ描き直す(種別セレクトの
/// `change`イベント、および初回描画で呼ぶ)。
function renderApplyFieldsInto(
  container: HTMLElement,
  prefix: string,
  fields: ApplyFieldSchema[],
  selectedBodyIndex: number,
): void {
  container.innerHTML = fields
    .map((field) => renderApplyField(prefix, field, selectedBodyIndex))
    .join("");
}

/// フォームの入力欄から`apply_component`へ渡すpayloadを組み立てる
/// (フィールド名・並び順はいずれもスキーマ側が唯一の情報源——ここでは
/// 別の対応表を持たない)。
function readApplyFieldsFrom(
  prefix: string,
  fields: ApplyFieldSchema[],
): Record<string, number | string | boolean> {
  const payload: Record<string, number | string | boolean> = {};
  for (const field of fields) {
    const el = document.getElementById(
      applyFieldInputId(prefix, field),
    ) as HTMLInputElement | null;
    if (!el) continue;
    if (field.type === "bool") {
      payload[field.name] = el.checked;
    } else if (field.type === "string") {
      payload[field.name] = el.value;
    } else {
      const n = Number(el.value);
      const value = Number.isFinite(n) ? n : 0;
      payload[field.name] = field.type === "f64" ? value : Math.trunc(value);
    }
  }
  return payload;
}

/// HUD 用の数値整形(QA不具合 7)。**固定桁だと組み合わせシーンで HUD が
/// 嘘をつく**——D20 の ΔT = 1.25×10⁻⁴ K は小数 2 桁固定ではまったく動かず、
/// 「熱が伝わっていない」ように見えていた。桁数を値の大きさで切り替え、
/// 極端に小さい値は指数表記へ落とす。
function formatHudNumber(value: number): string {
  if (!Number.isFinite(value)) return "—";
  const magnitude = Math.abs(value);
  if (magnitude === 0) return "0";
  if (magnitude < 1e-3 || magnitude >= 1e5) return value.toExponential(3);
  if (magnitude < 1) return value.toFixed(5);
  return value.toFixed(4);
}

/// HUD が読む「回路電圧」「熱ノード温度」を、**読み込んでいるシーンが宣言した
/// プローブから決める**(QA不具合 7)。
///
/// 以前は `circuit_divider_voltage`(既定シーンの分圧点 = ノード 2 固定)と
/// `heater_node_temperature`(熱ノード 0 固定)を無条件に出していた。D20 は
/// 2 ノード回路なのでノード 2 が存在せず、`circuit_probe` の `unwrap_or(0.0)`
/// がそのまま `circuit V = 0.000 V` になる——**実際の発電電圧 0.5 V は Probe
/// グラフには出るのに HUD だけが 0 と言う**状態だった。ドメインを組み合わせた
/// シーンほど HUD が嘘をつくため、シーン側の `probes` 宣言(そのシーンの作者が
/// 「これが見たい量だ」と書いたもの)を素直に第一候補にする。
///
/// プローブを宣言していないシーン(既定シーンなど)では従来の固定アクセサへ
/// 落ちる——既定シーンの分圧回路は実際にノード 2 が分圧点なので、そこでは
/// 従来表示が正しい。
/// `circuit_divider_voltage` が読むノード(sim-wasm の `CIRCUIT_DIVIDER_NODE`)。
/// 表示だけに使う——値そのものは従来どおり Rust 側のアクセサから取る。
const CIRCUIT_DIVIDER_NODE_LABEL = "2";

type HudDomainProbe = { index: number; node: string } | null;
type HudProbeSelection = {
  world: WasmWorld;
  probeCount: number;
  circuit: HudDomainProbe;
  temperature: HudDomainProbe;
  /// 温度の初期値。ΔT を出すのに使う(絶対値だけでは微小変化が読めない)。
  temperatureBaseline: number | null;
};
let hudProbeSelection: HudProbeSelection | null = null;

function selectHudProbes(world: WasmWorld): HudProbeSelection {
  const probeCount = readNumber(world, "imported_probe_count");
  // `world` の同一性と本数の両方を鍵にする——ギャラリー読み込みは `world` を
  // 差し替え、Import は同じ `world` へプローブを足すため、片方だけでは
  // キャッシュが古くなる。
  if (
    hudProbeSelection &&
    hudProbeSelection.world === world &&
    hudProbeSelection.probeCount === probeCount
  ) {
    return hudProbeSelection;
  }
  let circuit: HudDomainProbe = null;
  let temperature: HudDomainProbe = null;
  for (let i = 0; i < probeCount; i += 1) {
    // ラベルは `probe_target_label` が作る `CircuitV[2]` / `NodeTemp[0]` 形式。
    const label = world.read_component("imported_probe_label_at", String(i));
    const circuitMatch = /^CircuitV\[(\d+)\]$/.exec(label);
    if (circuitMatch && !circuit) circuit = { index: i, node: circuitMatch[1] };
    const tempMatch = /^NodeTemp\[(\d+)\]$/.exec(label);
    if (tempMatch && !temperature) temperature = { index: i, node: tempMatch[1] };
  }
  hudProbeSelection = {
    world,
    probeCount,
    circuit,
    temperature,
    temperatureBaseline: null,
  };
  return hudProbeSelection;
}

/// `couplingRow`が`BuoyancyDrag`の各行に出す「操縦面舵角」欄を配線する
/// (**残タスク完遂の縦串⑤増分**)。度→ラジアン変換して`apply_component`
/// (kind="push_set_coupling_control_surface_deflection")へ送るだけの薄い配線
/// ——Wing揚力を持たない結合では`Coupling::set_scalar_param`が無言で無視する
/// (`couplingRow`のdoc参照)ので、対象を絞り込む必要が無い。
function wireCouplingControlSurfaceInputs(world: WasmWorld): void {
  document
    .querySelectorAll<HTMLInputElement>('input[id^="coupling-deflection-"]')
    .forEach((input) => {
      const couplingIndex = Number(input.id.replace("coupling-deflection-", ""));
      input.addEventListener("change", () => {
        const degrees = Number(input.value);
        if (!Number.isFinite(degrees)) return;
        applyComponent(world, "push_set_coupling_control_surface_deflection", {
          coupling_index: couplingIndex,
          deflection_radians: (degrees * Math.PI) / 180,
        });
      });
    });
}

/// `renderInspectorExtraComponents`が生成した Thrust フォームを配線する
/// (**残タスク完遂の縦串⑤増分**)。値は`thrustByBody`(モジュールスコープ)
/// へ直接書き込むだけ——実際の`ApplyForce`送信は`setUpSceneView`のPlay
/// ループが毎step行う。
function wireThrustForm(index: number): void {
  if (index <= 0) return;
  const state = thrustStateFor(index);
  const enabledInput = document.getElementById(
    "thrust-enabled",
  ) as HTMLInputElement | null;
  enabledInput?.addEventListener("change", () => {
    state.enabled = enabledInput.checked;
  });
  (["x", "y", "z"] as const).forEach((axis, i) => {
    const input = document.getElementById(
      `thrust-axis-${axis}`,
    ) as HTMLInputElement | null;
    input?.addEventListener("change", () => {
      state.axis[i] = Number(input.value) || 0;
    });
  });
  const maxThrustInput = document.getElementById(
    "thrust-max",
  ) as HTMLInputElement | null;
  maxThrustInput?.addEventListener("change", () => {
    state.maxThrust = Math.max(0, Number(maxThrustInput.value) || 0);
  });
  const throttleInput = document.getElementById(
    "thrust-throttle",
  ) as HTMLInputElement | null;
  throttleInput?.addEventListener("change", () => {
    const value = Number(throttleInput.value);
    state.throttle = Number.isFinite(value)
      ? Math.min(1, Math.max(0, value))
      : 0;
  });
}

/// Add Jointフォームの種別セレクト値から`apply_component`のkind名への変換。
/// `component_schema.rs`の`apply_schema`が`add_${種別}_joint`という機械的な
/// 命名で全5種を並べているので、対応表を別に持つ必要が無い——`<option
/// value>`はUI表示用の短い識別子、kind名はそこから導出するだけ。
function jointApplyKind(selectValue: string): string {
  return `add_${selectValue}_joint`;
}

/// `renderInspectorExtraComponents`が生成した Add Joint フォームを配線する
/// (`renderInspectorFor`が`innerHTML`を張り替えるたび、フォームごと作り直されるので
/// 毎回呼び直す——回路エディタの`renderCircuitTab`と同じ設計)。
///
/// **スキーマ駆動フォーム(B11)**: 種別セレクトを切り替えるたびに
/// `component_schema`から該当kindのフィールド一覧を引き直し、入力欄を
/// 丸ごと描き直す。以前の「Body A/B・Anchor A/B・Axis・Param 1〜6」という
/// 全kind共通の汎用欄(意味は`title`ツールチップのみ)は無くなり、
/// 各欄が実際のフィールド名(`length`・`rest_length`・`theta_target`等)で
/// 出る。送信時も同じスキーマのフィールド一覧を辿ってpayloadを組み立てる
/// ので、フィールドの並びや意味がこことRust側でずれる余地が無い。
function wireAddJointForm(world: WasmWorld, index: number): void {
  const kindSelect = document.getElementById(
    "add-joint-kind",
  ) as HTMLSelectElement | null;
  const fieldsContainer = document.getElementById("add-joint-fields");
  const button = document.getElementById("add-joint-button");
  if (!kindSelect || !fieldsContainer || !button) return;
  const rerender = () => {
    const fields = applyKindSchema(world, jointApplyKind(kindSelect.value)).fields;
    renderApplyFieldsInto(fieldsContainer, "add-joint", fields, index);
  };
  kindSelect.addEventListener("change", rerender);
  rerender();
  button.addEventListener("click", () => {
    const kind = jointApplyKind(kindSelect.value);
    const fields = applyKindSchema(world, kind).fields;
    try {
      applyComponent(world, kind, readApplyFieldsFrom("add-joint", fields));
    } catch (err) {
      reportError(`Joint の追加に失敗しました: ${String(err)}`);
      return;
    }
    renderInspectorFor(world, index);
  });
}

/// Add Couplingフォームの種別セレクト値から`apply_component`のkind名への
/// 変換。`jointApplyKind`と同じく`add_${種別}_coupling`という機械的な命名が
/// 全16種で成り立つ(`component_schema.rs`の`apply_schema`参照)ので、
/// 対応表を別に持つ必要が無い。
function couplingApplyKind(selectValue: string): string {
  return `add_${selectValue}_coupling`;
}

/// `renderInspectorExtraComponents`が生成した Add Coupling フォームを
/// 配線する(`wireAddJointForm`と同じ設計、B12〜B15)。
///
/// 以前は16種のkindを`switch`文で手書きし、`add-coupling-axis-*`/
/// `add-coupling-p1`〜`p6`という共通の汎用欄を種別ごとに読み替えていた
/// (`add_phase_change_morph_coupling`ではAxis欄が`melting_temperature`等の
/// 材質パラメータへ、`add_convection_link_coupling`ではAxis欄が流体物性値へ
/// 流用される、といった具合)。スキーマ駆動化した今は`applyKindSchema`が
/// 返すフィールド名がそのまま入力欄のラベルと送信payloadのキーになるので、
/// この読み替えの発生源だった`switch`文自体が不要になった。
function wireAddCouplingForm(world: WasmWorld, index: number): void {
  const kindSelect = document.getElementById(
    "add-coupling-kind",
  ) as HTMLSelectElement | null;
  const fieldsContainer = document.getElementById("add-coupling-fields");
  const button = document.getElementById("add-coupling-button");
  if (!kindSelect || !fieldsContainer || !button) return;
  const rerender = () => {
    const fields = applyKindSchema(world, couplingApplyKind(kindSelect.value)).fields;
    renderApplyFieldsInto(fieldsContainer, "add-coupling", fields, index);
  };
  kindSelect.addEventListener("change", rerender);
  rerender();
  button.addEventListener("click", () => {
    const kind = couplingApplyKind(kindSelect.value);
    const fields = applyKindSchema(world, kind).fields;
    try {
      applyComponent(world, kind, readApplyFieldsFrom("add-coupling", fields));
    } catch (err) {
      reportError(`Coupling の追加に失敗しました: ${String(err)}`);
      return;
    }
    renderInspectorFor(world, index);
  });
}

/// **編集可能な RigidBody Component(群2)**。設計 §1.3 の表は RigidBody に
/// 「Shape、Mass、Material、Body type(Dynamic/Static/Kinematic)、Collision
/// group/mask」を求めている。Shape/Material は既に出ていたが、残り3つは
/// **表示すらされていなかった**(Collision group/mask に至っては
/// `RigidBodySet` に概念自体が無く、群2で `sim-mechanics` から作った)。
///
/// 編集は全て `Command` としてキューへ積み、**次 step の先頭で適用される**
/// (`World::apply_pending_commands`)。Gizmo ドラッグのような直接書き換えと
/// 違い、Play モード中でもリプレイ再現性と決定論が壊れない。
function renderRigidBodyComponent(world: WasmWorld, index: number): string {
  const mass = readNumber(world, "body_mass_at", String(index));
  const bodyType = world.read_component("body_type_at", String(index));
  const group = readNumber(world, "body_collision_group_at", String(index));
  const mask = readNumber(world, "body_collision_mask_at", String(index));
  const option = (value: string) =>
    `<option value="${value}"${value === bodyType ? " selected" : ""}>${value}</option>`;
  // 質量 0 は「無限質量」(Static/Kinematic)を意味するので、数値入力には
  // 出さずプレースホルダで示す——0 と表示すると「0 kg の物体」に見えてしまう。
  const massValue = mass > 0 ? mass.toPrecision(6) : "";
  return `
    <div class="inspector-component">
      <h3>RigidBody</h3>
      <div class="inspector-field"><span>Material</span><span>${world.read_component("body_material_label_at", String(index))}</span></div>
      <div class="inspector-field">
        <span>Mass [kg]</span>
        <input type="number" id="inspector-mass" min="0" step="0.1"
               value="${massValue}" placeholder="∞ (無限質量)"
               ${mass > 0 ? "" : "disabled"} />
      </div>
      <div class="inspector-field">
        <span>Body type</span>
        <select id="inspector-body-type">${["Dynamic", "Static", "Kinematic"].map(option).join("")}</select>
      </div>
      <div class="inspector-field">
        <span>Collision group</span>
        <input type="number" id="inspector-collision-group" min="0" step="1" value="${group}" />
      </div>
      <div class="inspector-field">
        <span>Collision mask</span>
        <input type="number" id="inspector-collision-mask" min="0" step="1" value="${mask}" />
      </div>
      <p class="inspector-note">編集は Command としてキューに積まれ、次 step の先頭で適用されます。</p>
    </div>
  `;
}

/// `renderInspectorFor` が `innerHTML` を張り替えた直後にイベントを配線する
/// (innerHTML 代入で前のリスナは要素ごと捨てられるので毎回張り直す)。
function wireInspectorEditFields(index: number): void {
  const handlers = inspectorEditRef.current;
  if (!handlers) return;
  const massInput = document.getElementById(
    "inspector-mass",
  ) as HTMLInputElement | null;
  massInput?.addEventListener("change", () => {
    const value = Number(massInput.value);
    if (!Number.isFinite(value) || value <= 0) return;
    handlers.setMass(index, value);
  });

  // Position の直接編集(**残タスク完遂の縦串①増分**)。Gizmo と同じく
  // Command を経由しない。
  const positionInputs = (["x", "y", "z"] as const).map(
    (axis) =>
      document.getElementById(
        `inspector-position-${axis}`,
      ) as HTMLInputElement | null,
  );
  const pushPosition = () => {
    const [px, py, pz] = positionInputs.map((input) => Number(input?.value));
    if (![px, py, pz].every((p) => Number.isFinite(p))) return;
    handlers.setPosition(index, px, py, pz);
  };
  positionInputs.forEach((input) =>
    input?.addEventListener("change", pushPosition),
  );

  const typeSelect = document.getElementById(
    "inspector-body-type",
  ) as HTMLSelectElement | null;
  typeSelect?.addEventListener("change", () =>
    handlers.setBodyType(index, typeSelect.value),
  );
  const groupInput = document.getElementById(
    "inspector-collision-group",
  ) as HTMLInputElement | null;
  const maskInput = document.getElementById(
    "inspector-collision-mask",
  ) as HTMLInputElement | null;
  // group/maskは`setCollisionFilter`が2つ一緒に1つのCommandとして送る必要が
  // あるため、変更時に**両方の欄をDOMから読み直す**素朴な実装だと、
  // 適用がまだ次stepまで反映されない間に`updateInspectorRigidBodyFields`の
  // 毎フレーム更新(フォーカスが外れた欄を「まだ効いていない実際の値」へ
  // 戻す、下記の同関数のdoc参照)が割り込み、片方の欄を変更してもう片方を
  // 変更するまでの間に最初の欄がフレーム更新で古い値へ戻ってしまうと、
  // 2つ目の変更が古い値と組んで送られてしまう(**残タスク完遂の縦串①増分で
  // 発見**)。ローカル変数へ「最後に自分の欄で確定した値」を保持し、
  // DOMを読み直さないことでこれを避ける。
  let pendingGroup = Number(groupInput?.value ?? 0);
  let pendingMask = Number(maskInput?.value ?? 0);
  const pushFilter = () => {
    if (
      !Number.isInteger(pendingGroup) ||
      !Number.isInteger(pendingMask) ||
      pendingGroup < 0 ||
      pendingMask < 0
    )
      return;
    handlers.setCollisionFilter(index, pendingGroup, pendingMask);
  };
  groupInput?.addEventListener("change", () => {
    pendingGroup = Number(groupInput.value);
    pushFilter();
  });
  maskInput?.addEventListener("change", () => {
    pendingMask = Number(maskInput.value);
    pushFilter();
  });

  // 軸別スケール(群2)。Gizmo ドラッグと違い**Edit モードでの直接編集**なので
  // Command を経由しない(既存の Scale Gizmo と同じ扱い)。
  const scaleInputs = (["x", "y", "z"] as const).map(
    (axis) =>
      document.getElementById(
        `inspector-scale-${axis}`,
      ) as HTMLInputElement | null,
  );
  const pushScale = (changedValue: number) => {
    const [sx, sy, sz] = scaleInputs.map((input) => Number(input?.value));
    if (![sx, sy, sz].every((s) => Number.isFinite(s) && s > 0)) return;
    if (handlers.setScaleXyz(index, sx, sy, sz)) return;
    // 軸別スケールが効かない形状(球等)では、**変更された欄の値**を等方
    // スケールとして使う(**残タスク完遂の縦串①増分**)。3欄すべての一致を
    // 条件にすると、欄を1つずつ順にfillするUI(人間がTabで移る場合も同じ)
    // では、まだ更新していない残り2欄が古い値のままなので毎回不一致になり、
    // 一度も適用されない——以前はその不一致のたびに全欄を1へ巻き戻していた。
    // 球にはX/Y/Zの区別が無い以上、変更された欄だけを信じるのが正しい。
    // (他の2欄は書き戻さない——ユーザーが続けて別の欄を編集している最中に
    // 値を書き換えると、入力中の文字列と競合して壊れる。次にInspectorが
    // 再描画されるタイミングで実際の形状に基づいた値へ揃う。)
    if (handlers.setScale(index, changedValue)) return;
    // それでも効かない(Ground等)場合のみ、入力を1に戻す——「入れたのに
    // 何も起きない」より「この形状には効かない」と見えるほうが正直。
    scaleInputs.forEach((input) => input && (input.value = "1"));
  };
  scaleInputs.forEach((input) =>
    input?.addEventListener("change", () => pushScale(Number(input.value))),
  );
}

/// **増分K: Componentビューの残り(Joint/Circuit/Coupling/Probe/近似バッジ)**。
///
/// 設計 docs/23-frontend/01-editor.md §1.3 が挙げる Component のうち、
/// Transform/RigidBody/Shape/Material だけが実データに繋がっていて、残りは
/// 未実装のままだった。それぞれ既存または本増分で足した wasm API から引く:
///
/// - **Joint**: `constraint_anchor_points_at`(選択中ボディが持つ拘束のアンカー)
/// - **Circuit**: `circuit_element_count`/`circuit_element_label_at`(増分G2)
/// - **Coupling**: `coupling_info_text`(**群1で内省層へ移行**)——増分Kの時点では
///   `coupling_count`(件数のみ)で、「`Coupling`トレイトが名前を持たないので
///   名前を捏造するより件数だけを正直に出す」と書いていた。**群1でトレイト側に
///   `kind()`/`describe()`/`referenced_bodies()` を足して前提ごと解消した**ので、
///   種別・パラメータ・跨るドメイン・作用先ボディまで出せる
/// - **Probe**: `imported_probe_count`/`imported_probe_label_at`(増分B1)
/// - **近似バッジ**: `active_approximations_text`(**群1で自己申告へ移行**)——
///   増分Kでは「どのドメインが有効か」からWorld側が推測していた。群1で
///   `Solver::approximations()` を足し、各ソルバが名前・出典・オフ可否を
///   自己申告する形にした(設定依存の近似も表現できるようになった)
///
/// FluidRegion は SPH 粒子が個別ID体系を持たないため対象外(Hierarchy の
/// 概要行と同じ既知の限界)。
function renderInspectorExtraComponents(
  world: WasmWorld,
  index: number,
): string {
  const escape = (text: string) =>
    text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  const sections: string[] = [];

  // Joint(**群1で内省層へ移行**): 以前は`constraint_anchor_points_at`が返す
  // アンカー2点だけを出しており、**種別も接続先も軸もモータ設定も見えなかった**。
  // 設計 §1.3 は「種別(Ball/Hinge/Slider/…)・接続 Body ID・軸・制限・モータ」を
  // 要求している。`joint_info_text`(タブ区切り)から全項目を出す。
  const jointLines = world
    .read_component("joint_info_text", String(index))
    .split("\n")
    .filter((l) => l.length > 0);
  if (jointLines.length > 0) {
    const rows = jointLines
      .map((line) => {
        const [kind, connection, detail, state] = line.split("\t");
        return (
          `<div class="inspector-field"><span>${escape(kind)}</span>` +
          `<span>${escape(connection)}${detail ? " / " + escape(detail) : ""}` +
          `${state === "無効" ? " (無効)" : ""}</span></div>`
        );
      })
      .join("");
    sections.push(
      `<div class="inspector-component" data-stacked><h3>Joint</h3>${rows}</div>`,
    );
  }

  // Circuit: ワールドに載っている回路素子(ボディ単位ではなくシーン単位)。
  const circuitCount = readNumber(world, "circuit_element_count");
  if (circuitCount > 0) {
    const rows: string[] = [];
    for (let k = 0; k < circuitCount; k += 1) {
      rows.push(
        `<div class="inspector-field"><span>#${k}</span><span>` +
          `${escape(world.read_component("circuit_element_label_at", String(k)))}</span></div>`,
      );
    }
    sections.push(
      `<div class="inspector-component"><h3>Circuit</h3>${rows.join("")}</div>`,
    );
  }

  // Coupling(**群1で内省層へ移行**): 以前は件数だけを出し「種別: —(トレイトが
  // 名前を持たないため非表示)」と表示していた。`Coupling`トレイトに
  // `kind()`/`describe()`/`referenced_bodies()`を足したので、**選択中ボディに
  // 作用する結合を種別・パラメータ・跨るドメイン込みで**出せるようになった。
  //
  // **ボディ単位とシーン単位を分けて出す**: `DissipationToHeat`や`JouleHeat`のように
  // 「全体の散逸/損失を読んで熱ノードへ移す」結合は特定の剛体を参照しないため、
  // 選択ボディで絞ると消えてしまう。しかしそれらもワールドに効いている以上
  // 見えないと困る(実際D10でこの問題を踏んだ)。回路と同じくシーン単位の
  // コンポーネントとして別枠で出す。
  const parseCoupling = (line: string) => {
    const [kind, description, domains, bodies, indexStr] = line.split("\t");
    return { kind, description, domains, bodies, index: Number(indexStr) };
  };
  const allCouplings = world
    .read_component("coupling_info_text", "-1")
    .split("\n")
    .filter((l) => l.length > 0)
    .map(parseCoupling);
  const forThisBody = allCouplings.filter((c) =>
    c.bodies
      .split(",")
      .filter((b) => b.length > 0)
      .includes(String(index)),
  );
  const sceneWide = allCouplings.filter((c) => c.bodies.length === 0);
  // **残タスク完遂の縦串⑤増分(操縦面)**: `BuoyancyDrag`は`LiftModel::Wing`を
  // 持つかどうかを`coupling_info_text`の文字列表現だけからは区別できない
  // (`describe()`は`water`/`atmosphere`の有無しか書かない、`lift`はモジュール
  // doc参照)。区別する専用の内省を増やすより、舵角入力を全ての`BuoyancyDrag`行に
  // 出し、Wing以外では`Coupling::set_scalar_param`が`false`を返して無言で
  // 無視される(モジュールdocの既定実装)という縮退に任せる——実害が無く、
  // wasm境界を増やさずに済む。
  const couplingRow = (c: {
    kind: string;
    description: string;
    domains: string;
    index: number;
  }) =>
    `<div class="inspector-field" title="${escape(world.read_component("coupling_kind_summary", c.kind))}">` +
    `<span>${escape(c.kind)}</span>` +
    `<span>${escape(c.description)} <em>[${escape(c.domains)}]</em></span></div>` +
    (c.kind === "BuoyancyDrag"
      ? `<div class="inspector-field">` +
        `<span>操縦面舵角 [deg]</span>` +
        `<input type="number" id="coupling-deflection-${c.index}" step="1" value="0" title="Wing揚力を持つ結合にのみ効く(Magnus/水域/大気のみの結合では無視される)" />` +
        `</div>`
      : "");
  if (forThisBody.length > 0) {
    sections.push(
      `<div class="inspector-component" data-stacked><h3>Coupling</h3>` +
        forThisBody.map(couplingRow).join("") +
        `</div>`,
    );
  }
  if (sceneWide.length > 0) {
    sections.push(
      `<div class="inspector-component" data-stacked><h3>Coupling (シーン全体)</h3>` +
        sceneWide.map(couplingRow).join("") +
        `</div>`,
    );
  }

  // Probe: シーンJSONが宣言した観測量。
  const probeCount = readNumber(world, "imported_probe_count");
  if (probeCount > 0) {
    const rows: string[] = [];
    for (let k = 0; k < probeCount; k += 1) {
      rows.push(
        `<div class="inspector-field"><span>${escape(world.read_component("imported_probe_label_at", String(k)))}</span>` +
          `<span>${readNumber(world, "imported_probe_value_at", String(k)).toFixed(4)}</span></div>`,
      );
    }
    sections.push(
      `<div class="inspector-component"><h3>Probe</h3>${rows.join("")}</div>`,
    );
  }

  // 近似バッジ(**群1で自己申告へ移行**): 以前はWorld側が「どのドメインが
  // 有効か」から推測した固定文字列を並べていた。`Solver::approximations()`で
  // 各ソルバが申告する形になり、設計 §1.3 が要求する「名前・**出典**・
  // **オフ可否**」が揃った(タブ区切りの4列)。
  const approximations = world.read_component("active_approximations_text", "");
  if (approximations.length > 0) {
    const badges = approximations
      .split("\n")
      .filter((l) => l.length > 0)
      .map((line) => {
        const [name, reason, doc, canDisable] = line.split("\t");
        // **オフ可否が false のものにトグルを出さない**——「オフにできます」と
        // いう嘘のUIを出さないため。true のものだけ操作可能に見せる。
        const suffix = canDisable === "1" ? " ⏻" : "";
        return (
          `<span class="approximation-badge" data-can-disable="${escape(canDisable)}" ` +
          `title="${escape(reason)}&#10;出典: ${escape(doc)}">${escape(name)}${suffix}</span>`
        );
      })
      .join("");
    sections.push(
      `<div class="inspector-component"><h3>近似</h3>` +
        `<div class="approximation-badges">${badges}</div></div>`,
    );
  }

  // Thrust(**残タスク完遂の縦串⑤増分**、モジュール冒頭の`ThrustState`のdoc
  // 参照)。Ground(index 0)は対象外(静的剛体に推力は意味を持たない、
  // 軸別スケールと同じ理由でindex 0を除外)。
  if (index > 0) {
    const thrust = thrustStateFor(index);
    sections.push(`
      <div class="inspector-component" data-stacked>
        <h3>Thrust(推力)</h3>
        <div class="inspector-field">
          <label><input type="checkbox" id="thrust-enabled" ${thrust.enabled ? "checked" : ""} /> エンジン有効</label>
        </div>
        <div class="inspector-field">
          <span>Axis(ローカル)</span>
          <span class="inspector-joint-row">
            <input type="number" id="thrust-axis-x" step="0.1" value="${thrust.axis[0]}" />
            <input type="number" id="thrust-axis-y" step="0.1" value="${thrust.axis[1]}" />
            <input type="number" id="thrust-axis-z" step="0.1" value="${thrust.axis[2]}" />
          </span>
        </div>
        <div class="inspector-field">
          <span>最大推力 [N]</span>
          <input type="number" id="thrust-max" step="10" min="0" value="${thrust.maxThrust}" />
        </div>
        <div class="inspector-field">
          <span>スロットル [0-1]</span>
          <input type="number" id="thrust-throttle" step="0.05" min="0" max="1" value="${thrust.throttle}" />
        </div>
        <p class="inspector-note">Playモード中、毎stepローカル軸をボディの姿勢でワールドへ回し、スロットル×最大推力を<code>ApplyForce</code>で送る(新しいCoupling/Commandを物理コアへ足さない縮約、モジュール冒頭の<code>ThrustState</code>のdoc参照)。着陸装置は既存のWheelJoint(Add Jointフォームの"Wheel"種別)をそのまま流用できる。</p>
      </div>
    `);
  }

  // Add Component(**残タスク完遂の縦串①増分**、`WasmWorld::add_*_joint`
  // 5種の薄いフォーム、**B11でスキーマ駆動化**)。以前はUnity風の
  // 「型ごとの専用フォーム」を避け、回路エディタ(`kind`セレクト+
  // `値/値2/値3`の汎用欄)と同じ縮約で「Body A/B・Anchor A/B・Axis・
  // Param1〜6」を全種別共通の汎用欄として出し、実際に使うフィールドは
  // `title`属性のツールチップでしか示していなかった——選んだ`kind`と
  // 入っている数値の対応がホバーしなければ分からない状態だった。
  //
  // `component_schema`(Task#9)は`_impl`メソッドの実引数と1:1のフィールド
  // 名・型・単位・既定値を返せるので、`<div id="add-joint-fields">`を
  // 種別セレクトの`change`ごと`wireAddJointForm`が描き直す(静的マークアップ
  // 側は種別セレクトと空のコンテナだけを持つ)。Body A相当の欄(選択中
  // ボディを既定値にする)・Body B相当の欄(既定`-1`=ワールド固定点)の
  // 挙動は`initialApplyFieldValue`が踏襲する。
  sections.push(`
    <div class="inspector-component" data-stacked>
      <h3>Add Joint</h3>
      <div class="inspector-field">
        <span>種別</span>
        <select id="add-joint-kind">
          <option value="distance">Distance(距離拘束)</option>
          <option value="ball">Ball(球面拘束)</option>
          <option value="slider">Slider(1軸並進)</option>
          <option value="wheel">Wheel(車輪、chassisがシャシー・wheelが車輪)</option>
          <option value="hinge_motor">HingeMotor(1軸ヒンジ)</option>
        </select>
      </div>
      <div id="add-joint-fields"></div>
      <button id="add-joint-button">Joint を追加</button>
      <p class="inspector-note">入力欄は種別を選ぶと実引数どおりに再構成される(Wheelの<code>suspension_axis</code>/<code>axle_axis</code>は<code>WheelJoint::new</code>の既定値固定でUIに出さない——「普通の車」を作れることを優先した縮約、<code>add_wheel_joint_impl</code>のdoc参照)。追加は即座に反映され、Command化されない(シーン構築操作のため——設計 docs/20-integration/04-world-api.md §1)。</p>
    </div>
  `);

  // Add Coupling(**残タスク完遂の縦串②増分、8種に拡張**、
  // `WasmWorld::add_*_coupling`の薄いフォーム、**B12〜B15でスキーマ駆動化**)。
  // 結合14種のうち8種を対象とする——剛体参照だけで完結する3種
  // (ImageChargeForce・LorentzForce・BuoyancyDrag)に加え、熱ノード・電圧源を
  // **indexで参照するだけ**の5種(DissipationToHeat・JouleHeat・
  // BrownianForce・MotorCoupling・InductionCoupling)。既定の起動シーンは
  // 熱ノード1個(index 0)・電圧源1個(index 0)を最初から持つため、それらを
  // 参照するだけなら対応ドメインをUIから作る手段がまだ無くても意味を持つ
  // ——ただしAdd Componentで一から組んだシーン(熱・回路ドメインが無い)では、
  // これらのindexが常に無効になり`Err`になる(wasm側`try_thermal_node_
  // _index`/`try_voltage_source_index`が明示的に拒否する、無言で無効な
  // 状態になるより失敗として伝わる方を選んだ——このスキーマ駆動化でも
  // クライアント側で先回りして検証しない、拒否はRust側の仕事のまま)。
  // 残り6種(PhaseChangeMorph・SphRigid・GridFluidRigid・ConvectionLink・
  // PistonGas・BoussinesqBuoyancy)も解禁済み——Settingsの「ドメイン」パネル
  // (熱ノード・格子流体・気体)と既存の「＋ 流体」(SPH)で対応ドメインを
  // 先に有効化すれば追加できる。加えてWingLift/MagnusLift(いずれも
  // BuoyancyDragの薄翼理論/マグヌス効果、縦串⑤で解禁)も同じフォームから
  // 追加できる。
  //
  // 以前は上記16種すべてが「Body・Axis・Param1〜6」という共通の汎用欄を
  // 種別ごとに読み替えていた——`PhaseChangeMorph`ではAxis欄が
  // `melting_temperature`等の材質パラメータへ、`ConvectionLink`では
  // Axis欄が流体物性値へ流用される、という具合で、意味は`title`
  // ツールチップの中にしか無かった。`component_schema`が`_impl`の実引数と
  // 1:1のフィールド名を返せる以上、この読み替え自体が不要になったので
  // `<div id="add-coupling-fields">`をJointと同じ設計で描き直す。
  sections.push(`
    <div class="inspector-component" data-stacked>
      <h3>Add Coupling</h3>
      <div class="inspector-field">
        <span>種別</span>
        <select id="add-coupling-kind">
          <option value="image_charge_force">ImageChargeForce(鏡像力)</option>
          <option value="lorentz_force">LorentzForce(ローレンツ力)</option>
          <option value="buoyancy_drag">BuoyancyDrag(浮力・抗力)</option>
          <option value="dissipation_to_heat">DissipationToHeat(摩擦の熱、要熱ドメイン)</option>
          <option value="joule_heat">JouleHeat(回路損失の熱、要熱・回路ドメイン)</option>
          <option value="brownian_force">BrownianForce(ブラウン運動、要熱ドメイン)</option>
          <option value="motor_coupling">MotorCoupling(モーター、要回路ドメイン)</option>
          <option value="induction_coupling">InductionCoupling(電磁誘導、要回路ドメイン)</option>
          <option value="phase_change_morph">PhaseChangeMorph(相変化、要熱ドメイン)</option>
          <option value="sph_rigid">SphRigid(SPH流体との相互作用、要SPHドメイン)</option>
          <option value="grid_fluid_rigid">GridFluidRigid(格子流体との相互作用、要格子流体ドメイン)</option>
          <option value="boussinesq_buoyancy">BoussinesqBuoyancy(温度差浮力、要熱・格子流体ドメイン)</option>
          <option value="convection_link">ConvectionLink(対流熱伝達、要熱ドメイン)</option>
          <option value="piston_gas">PistonGas(ピストン気体、要気体ドメイン)</option>
          <option value="wing_lift">WingLift(BuoyancyDrag+翼揚力、薄翼理論)</option>
          <option value="magnus_lift">MagnusLift(BuoyancyDrag+マグヌス揚力、回転球)</option>
        </select>
      </div>
      <div id="add-coupling-fields"></div>
      <button id="add-coupling-button">Coupling を追加</button>
      <p class="inspector-note">入力欄は種別を選ぶと実引数どおりに再構成される。熱ノード・電圧源を参照する5種は、対応ドメインが有効なシーン(既定の起動シーンはどちらもindex 0を1つ持つ)でのみ成功する。残り6種は対応ドメイン(熱ノード・SPH流体・格子流体・気体区画)が必要——Settingsの「ドメイン」パネル(または「＋ 流体」)で先に有効化すること。</p>
    </div>
  `);

  return sections.join("");
}

/// RigidBody Component の編集可能フィールドを、**実際にエンジンへ適用された値**で
/// 毎フレーム描き直す(群2)。編集は次stepの先頭で適用されるため、押した瞬間には
/// まだ反映されていない——ここで実データを読み直すことで「いつ適用されたか」が
/// UI 上で正直に見える。
///
/// Settings の重力入力と同じく **`document.activeElement` の欄は書き換えない**。
/// 打っている最中に値が戻ると数値を入力できなくなるため。
function updateInspectorRigidBodyFields(world: WasmWorld, index: number): void {
  const setIfIdle = (id: string, value: string) => {
    const element = document.getElementById(id) as
      | HTMLInputElement
      | HTMLSelectElement
      | null;
    if (!element || document.activeElement === element) return;
    if (element.value !== value) element.value = value;
  };
  const massInput = document.getElementById(
    "inspector-mass",
  ) as HTMLInputElement | null;
  if (massInput) {
    const mass = readNumber(world, "body_mass_at", String(index));
    // Static/Kinematic は inv_mass=0(無限質量)なので欄を空にして無効化する。
    massInput.disabled = !(mass > 0);
    if (document.activeElement !== massInput) {
      massInput.value = mass > 0 ? mass.toPrecision(6) : "";
    }
  }
  setIfIdle("inspector-body-type", world.read_component("body_type_at", String(index)));
  setIfIdle(
    "inspector-collision-group",
    world.read_component("body_collision_group_at", String(index)),
  );
  setIfIdle(
    "inspector-collision-mask",
    world.read_component("body_collision_mask_at", String(index)),
  );
}

function updateInspectorTransformFields(
  position: THREE.Vector3,
  rotation: THREE.Euler,
  velocity: THREE.Vector3,
): void {
  const rotationField = document.getElementById("inspector-rotation");
  const velocityField = document.getElementById("inspector-velocity");
  if (!rotationField || !velocityField) return; // 選択切替の再描画中は一時的に無い。
  const toDeg = (rad: number) => THREE.MathUtils.radToDeg(rad).toFixed(1);
  rotationField.textContent = `${toDeg(rotation.x)}°, ${toDeg(rotation.y)}°, ${toDeg(rotation.z)}°`;
  velocityField.textContent = `${velocity.x.toFixed(3)}, ${velocity.y.toFixed(3)}, ${velocity.z.toFixed(3)}`;

  // Position は編集可能な `<input>` になったので(残タスク完遂の縦串①増分)、
  // `updateInspectorRigidBodyFields`の`setIfIdle`と同じく、フォーカス中の欄は
  // 書き換えない(打っている最中に値が戻ると入力できなくなる)。
  (["x", "y", "z"] as const).forEach((axis, i) => {
    const input = document.getElementById(
      `inspector-position-${axis}`,
    ) as HTMLInputElement | null;
    if (!input || document.activeElement === input) return;
    const value = [position.x, position.y, position.z][i].toFixed(3);
    if (input.value !== value) input.value = value;
  });
}

// 入力列記録(設計docs/23-frontend/01-editor.md §1.6「Replays」、Command系の
// 実行記録)。Commandをキューへ積む離散的なUI操作(Nudge・Grab開始/Release・
// モーター目標切替・回路スイッチ切替)のたびに1件記録する。ヒーターは毎step
// 再送する継続的な内部動作(縮約実装、モジュールdoc参照)のため、切替の
// 瞬間だけを記録し、各subStepの再送そのものは記録しない(記録が単調に
// 膨れ上がるのを避ける、ユーザーの実際の操作単位に対応させる設計)。
//
// 各コマンドの実際のパラメータを判別共用体として構造化データのまま保持する
// (以前は呼び出し側が組み立てた表示用文字列`detail`のみを保持していたが、
// Replay再生実行(`replayCommandLogHeadless`)には実際のパラメータそのものが
// 要るため、この増分で構造化した——表示用の文字列化は`formatCommandLogDetail`
// に一本化)。
type CommandLogEntry =
  | {
      t: number;
      step: number;
      kind: "Grab";
      bodyIndex: number;
      targetX: number;
      targetY: number;
      targetZ: number;
    }
  | { t: number; step: number; kind: "Release"; bodyIndex: number }
  | {
      t: number;
      step: number;
      kind: "ApplyForce";
      bodyIndex: number;
      fx: number;
      fy: number;
      fz: number;
    }
  | {
      t: number;
      step: number;
      kind: "SetMotorTarget";
      bodyIndex: number;
      bodyLabel: string;
      targetAngle: number;
    }
  | { t: number; step: number; kind: "SetSwitch"; closed: boolean }
  | {
      t: number;
      step: number;
      kind: "SetHeatSource";
      on: boolean;
      watts: number;
    }
  // **物理パラメータの変更(群2)**。決定論の観点では重力・dtの変更も
  // 「入力」なので、Grab や SetSwitch と同じく記録しないとリプレイが
  // 再現しない(とくに dt はステップ幅そのものを変える)。
  | { t: number; step: number; kind: "SetGravity"; gravity: number }
  | {
      t: number;
      step: number;
      kind: "SetGravityDirection";
      x: number;
      y: number;
      z: number;
    }
  | { t: number; step: number; kind: "SetDt"; dt: number }
  // **Inspector の編集(群2)**。これらは `World` 側でも `Command` として
  // `command_log` に載るが、フロント側の記録は「ユーザーの操作」単位で
  // Replay タブに出すためのもの(既存の Grab/SetSwitch と同じ扱い)。
  | {
      t: number;
      step: number;
      kind: "SetBodyMass";
      bodyIndex: number;
      mass: number;
    }
  | {
      t: number;
      step: number;
      kind: "SetBodyType";
      bodyIndex: number;
      bodyType: string;
    }
  | {
      t: number;
      step: number;
      kind: "SetCollisionFilter";
      bodyIndex: number;
      group: number;
      mask: number;
    };
const commandLog: CommandLogEntry[] = [];

// `Omit<Union, K>`はTypeScriptでは判別共用体を分配せず、各variant固有の
// フィールド(targetX/fx/closed等)が消えてしまう既知の挙動のため、分配版の
// Omitを自前で定義する(`T extends any ? ... : never`は条件型がunion型の各
// メンバーへ分配して適用される性質を利用する標準的なパターン)。
type DistributiveOmit<T, K extends keyof T> = T extends unknown
  ? Omit<T, K>
  : never;

function pushCommandLog(
  world: WasmWorld,
  entry: DistributiveOmit<CommandLogEntry, "t" | "step">,
) {
  commandLog.push({
    ...entry,
    t: readNumber(world, "time"),
    step: readNumber(world, "step_count"),
  } as CommandLogEntry);
}

function formatCommandLogDetail(entry: CommandLogEntry): string {
  switch (entry.kind) {
    case "Grab":
      return `body=Box_1 anchor=(${entry.targetX.toFixed(3)},${entry.targetY.toFixed(3)},${entry.targetZ.toFixed(3)})`;
    case "Release":
      return "body=Box_1";
    case "ApplyForce":
      return `body=Box_1 force=(${entry.fx},${entry.fy},${entry.fz})`;
    case "SetMotorTarget":
      return `body=${entry.bodyLabel} theta_target=${entry.targetAngle.toFixed(3)}`;
    case "SetSwitch":
      return `closed=${entry.closed}`;
    case "SetHeatSource":
      return `toggled ${entry.on ? "on" : "off"} (${entry.watts}W)`;
    case "SetGravity":
      return `gravity = ${entry.gravity} m/s²`;
    case "SetGravityDirection":
      return `gravity direction = (${entry.x.toFixed(3)},${entry.y.toFixed(3)},${entry.z.toFixed(3)})`;
    case "SetDt":
      return `dt = ${entry.dt} s`;
    case "SetBodyMass":
      return `body=#${entry.bodyIndex} mass = ${entry.mass} kg`;
    case "SetBodyType":
      return `body=#${entry.bodyIndex} type = ${entry.bodyType}`;
    case "SetCollisionFilter":
      return `body=#${entry.bodyIndex} group=0x${entry.group.toString(16)} mask=0x${entry.mask.toString(16)}`;
  }
}

function setUpProjectDrawer(
  materialsRef: MaterialsRef,
  circuitRef: CircuitRef,
  sceneExportRef: SceneExportRef,
  projectBundleRef: ProjectBundleRef,
  projectExportedRef: ProjectExportedRef,
  sceneImportRef: SceneImportRef,
  replayVerifyRef: ReplayVerifyRef,
  replayPlaybackRef: ReplayPlaybackRef,
  circuitEditorRef: CircuitEditorRef,
  circuitFreeWiringState: CircuitFreeWiringState,
  prefabRef: PrefabRef,
  prefabSaveRef: PrefabSaveRef,
  sceneGalleryRef: SceneGalleryRef,
  circuitElementsRef: CircuitElementsRef,
  validationBaseJsonRef: ValidationBaseJsonRef,
) {
  const body = document.getElementById("project-body")!;
  const tabs = document.querySelectorAll<HTMLButtonElement>(".project-tab");
  const staticContentByTab: Record<string, string> = {};
  let circuitTabRefreshIntervalId: number | null = null;
  const prefabs: PrefabDefinition[] = [];
  // Hierarchy の右クリック「プレハブ化」からの登録口(群2、`PrefabSaveRef`のdoc参照)。
  prefabSaveRef.current = (prefab) => {
    prefabs.push(prefab);
    if (document.querySelector('.project-tab[data-tab="prefabs"].active'))
      renderPrefabsTab();
  };

  // 自由配線回路エディタの状態(タブ切替でDOMは再構築されるが、実際に構築した
  // 回路自体はwasm側に残るため、この一覧はタブ再訪時の表示復元用)。
  type FreeWiringComponent =
    | { kind: "resistor"; a: number; b: number; resistance: number }
    | { kind: "voltage_source"; a: number; b: number; voltage: number }
    | { kind: "switch"; a: number; b: number; index: number; closed: boolean }
    | { kind: "capacitor"; a: number; b: number; capacitance: number }
    | { kind: "inductor"; a: number; b: number; inductance: number }
    | { kind: "diode"; a: number; b: number }
    | { kind: "dc_motor"; a: number; b: number; index: number };
  let freeWiringNumNodes = 0;
  const freeWiringComponents: FreeWiringComponent[] = [];

  /// Import の結果表示(`importStatus`)は**再描画を跨いで残す必要がある**。
  /// `renderScenesTab()` は `body.innerHTML = ""` で作り直すため、change
  /// ハンドラ内で `importStatus.textContent` を書いた直後に `renderScenesTab()`
  /// を呼ぶと、書いたばかりの要素ごと捨てられて**メッセージが一切出なかった**
  /// (QA不具合5の「捨てたことがユーザーに伝わらない」を UI 側で直すときに
  /// 判明した。従来の「N件のボディを追加しました」も同じ理由で見えていない)。
  let lastImportStatusMessage = "";

  function renderScenesTab() {
    body.innerHTML = "";
    if (!sceneExportRef.current) {
      body.textContent = "Scenes: 読み込み中...";
      return;
    }

    // シーンギャラリー(`SceneGalleryRef`のdoc参照)。ヘッドレスランナー・
    // D1–D43のテストと同じシーンJSONをワンクリックで読み込む。ここでの
    // 読み込みは既存シーンへの追加(下のImport)ではなく、ワールド自体の
    // 差し替え(`from_scene_json`)である点が異なる。
    const galleryHeading = document.createElement("h4");
    galleryHeading.textContent =
      "シーンギャラリー(ワールドを差し替えて読み込み)";
    body.appendChild(galleryHeading);
    // **カード + 絞り込み(増分「UI 品質の底上げ」)**。従来は 43 本を
    // 1 行 2 段(説明文の下に「読み込み」ボタン)のベタなリストで並べており、
    // 目的のシーンへ行くには全体をスクロールして日本語の文章を読むしかなかった
    // ——「検証済みのデモが 43 本」は README が掲げる中心的な価値なのに、
    // その入口が一番使いにくい状態だった。
    //  - 検索欄: 番号(D27)・題名・説明・ドメイン名のいずれにも当たる
    //  - ドメインのチップ: 「熱だけ見たい」に 1 クリックで応える
    //  - カード全体がボタン: 説明を読んだ指をそのまま押せる(押す的が
    //    小さなボタン 1 個から、カード 1 枚ぶんに広がる)
    const manifest = sceneGalleryManifest();
    const galleryToolbar = document.createElement("div");
    galleryToolbar.className = "scene-gallery-toolbar";
    const search = document.createElement("input");
    search.type = "search";
    search.className = "scene-gallery-search";
    search.placeholder = "シーンを検索(番号・名前・説明・ドメイン)";
    search.setAttribute("aria-label", "シーンを検索");
    const count = document.createElement("span");
    count.className = "scene-gallery-count";
    galleryToolbar.append(search, count);
    body.appendChild(galleryToolbar);

    const domains = [...new Set(manifest.flatMap((e) => e.domains))].sort();
    let activeDomain: string | null = null;
    const chips = document.createElement("div");
    chips.className = "scene-gallery-domains";
    const chipButtons: HTMLButtonElement[] = [];
    for (const domain of ["すべて", ...domains]) {
      const chip = document.createElement("button");
      chip.className = "scene-domain-chip";
      chip.textContent = domain;
      chip.setAttribute("aria-pressed", domain === "すべて" ? "true" : "false");
      chip.addEventListener("click", () => {
        activeDomain = domain === "すべて" ? null : domain;
        for (const other of chipButtons) {
          other.setAttribute(
            "aria-pressed",
            other === chip ? "true" : "false",
          );
        }
        applyGalleryFilter();
      });
      chipButtons.push(chip);
      chips.appendChild(chip);
    }
    body.appendChild(chips);

    const galleryList = document.createElement("ul");
    galleryList.className = "scene-gallery-list";
    for (const entry of manifest) {
      const item = document.createElement("li");
      // 検索対象をあらかじめ 1 本の小文字文字列に畳んでおく(入力のたびに
      // 43 件ぶんの結合をやり直さない)。
      item.dataset.haystack =
        `${entry.demo} ${entry.title} ${entry.description} ${entry.domains.join(" ")}`.toLowerCase();
      item.dataset.domains = entry.domains.join(" ");
      const card = document.createElement("button");
      card.className = "scene-card";
      card.dataset.sceneFile = entry.file;
      card.title = `${entry.demo}: ${entry.title} を読み込む(ワールドを差し替えます)`;
      const title = document.createElement("span");
      title.className = "scene-card-title";
      const id = document.createElement("span");
      id.className = "scene-card-id";
      id.textContent = entry.demo;
      title.append(id, document.createTextNode(entry.title));
      const description = document.createElement("span");
      description.className = "scene-card-desc";
      description.textContent = entry.description;
      const tags = document.createElement("span");
      tags.className = "scene-card-tags";
      for (const domain of entry.domains) {
        const tag = document.createElement("span");
        tag.className = "scene-tag";
        tag.textContent = domain;
        tags.appendChild(tag);
      }
      card.append(title, description, tags);
      card.addEventListener("click", () => {
        const json = sceneGalleryFileContent(entry.file);
        if (!json || !sceneGalleryRef.current) return;
        sceneGalleryRef.current(json);
      });
      item.appendChild(card);
      galleryList.appendChild(item);
    }
    body.appendChild(galleryList);

    const emptyResult = document.createElement("p");
    emptyResult.className = "empty-state";
    emptyResult.textContent = "条件に合うシーンがありません。";
    emptyResult.hidden = true;
    body.appendChild(emptyResult);

    function applyGalleryFilter() {
      const query = search.value.trim().toLowerCase();
      let shown = 0;
      for (const item of Array.from(galleryList.children) as HTMLElement[]) {
        const matchesText = !query || item.dataset.haystack!.includes(query);
        const matchesDomain =
          !activeDomain || item.dataset.domains!.split(" ").includes(activeDomain);
        const visible = matchesText && matchesDomain;
        item.hidden = !visible;
        if (visible) shown += 1;
      }
      count.textContent = `${shown} / ${manifest.length} 本`;
      emptyResult.hidden = shown > 0;
    }
    search.addEventListener("input", applyGalleryFilter);
    applyGalleryFilter();

    const note = document.createElement("p");
    note.textContent =
      "現在のシーン(ボディ一覧)をJSONへエクスポートする(人間可読な表示専用の形式)。";
    body.appendChild(note);

    const bodies = sceneExportRef.current();
    const exportButton = document.createElement("button");
    exportButton.textContent = `Export current scene (${bodies.length}件, JSON)`;
    exportButton.addEventListener("click", () => {
      const latestBodies = sceneExportRef.current
        ? sceneExportRef.current()
        : bodies;
      const blob = new Blob([JSON.stringify(latestBodies, null, 2)], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "scene.json";
      a.click();
      URL.revokeObjectURL(url);
    });
    body.appendChild(exportButton);

    // **単一ファイル Export(群2、設計 §6)**。`ProjectBundle`のdoc参照。
    const bundleButton = document.createElement("button");
    bundleButton.id = "btn-export-bundle";
    bundleButton.textContent = "⬇ 一括Export (シーン+Replay+Bookmark)";
    bundleButton.title =
      "シーンJSON・記録済みコマンド列・ブックマーク一覧を1つのJSONファイルにまとめて書き出す";
    bundleButton.addEventListener("click", () => {
      if (!projectBundleRef.current) return;
      const bundle = projectBundleRef.current();
      const blob = new Blob([JSON.stringify(bundle, null, 2)], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "project_bundle.json";
      a.click();
      URL.revokeObjectURL(url);
      projectExportedRef.current?.();
    });
    body.appendChild(bundleButton);

    const list = document.createElement("ul");
    for (const b of bodies) {
      const item = document.createElement("li");
      const [x, y, z] = b.position;
      item.textContent = `${b.label}: ${b.shape} / ${b.material} @ (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})${b.isStatic ? " [static]" : ""}`;
      list.appendChild(item);
    }
    body.appendChild(list);

    const importNote = document.createElement("p");
    importNote.textContent =
      "シーンJSON(sim_world::Scenarioスキーマ、ヘッドレスランナー・D1–D43のテストと同じ形式)を読み込み、現在のシーンへボディを追加する。";
    body.appendChild(importNote);

    const importInput = document.createElement("input");
    importInput.type = "file";
    importInput.accept = "application/json,.json";
    body.appendChild(importInput);

    const importStatus = document.createElement("p");
    importStatus.id = "scene-import-status";
    importStatus.textContent = lastImportStatusMessage;
    body.appendChild(importStatus);

    importInput.addEventListener("change", () => {
      const file = importInput.files?.[0];
      if (!file || !sceneImportRef.current) return;
      file
        .text()
        .then((text) => {
          const { count, skipped } = sceneImportRef.current!(text);
          // QA不具合5: 捨てたセクションを**UI にも**出す。以前は
          // 「2件のボディを追加しました」だけで、結合や熱が落ちたことが
          // ユーザーに伝わらなかった(D10 を Import しても結合は 0 件のまま)。
          lastImportStatusMessage =
            `${count}件のボディを追加しました。` +
            (skipped.length > 0
              ? ` ただし ${skipped.join(" / ")} は取り込まれていません` +
                `(Import が扱うのは materials / bodies / probes のみ)。` +
                `これらを含めて読み込むには Scene ギャラリーから開いてください。`
              : "");
          // 再描画が `importStatus` を作り直すので、先に文言を控えてから描く
          // (`lastImportStatusMessage`のdoc参照)。
          renderScenesTab();
        })
        .catch((err: unknown) => {
          lastImportStatusMessage = `Import失敗: ${err}`;
          importStatus.textContent = lastImportStatusMessage;
        });
    });
  }

  function renderCircuitTab() {
    body.innerHTML = "";
    let circuitFreeWiringRefresh: (() => void) | null = null;
    const topology = document.createElement("pre");
    topology.className = "circuit-topology";
    // **増分G2で修正した表示バグ**: ここは以前、固定デモ回路の図
    // (`Node1 (10V 電源) --[100Ω]-- Node2 --[200Ω]-- GND`)を無条件に
    // ハードコードで描いていた。シーンギャラリーから別の回路を読み込んでも図は
    // そのまま残り、「無効です」という注記は出るものの**数字自体が実態と違う嘘の
    // まま**だった(D19を読み込むと実際は9V / 1kΩ / 2kΩ + コンデンサ + スイッチ +
    // ダイオードなのに、10V / 100Ω / 200Ω と表示され続ける)。sim-wasm へ
    // `circuit_element_count`/`circuit_element_label_at` を足し、
    // **実際に載っている素子を列挙して描く**ようにした。
    const elements = circuitElementsRef.current
      ? circuitElementsRef.current()
      : [];
    if (elements.length > 0) {
      const lines: string[] = [
        `回路の素子(実際に配線されているもの、${elements.length}件):`,
        "",
      ];
      for (const label of elements) lines.push(`  ${label}`);
      lines.push("");
      lines.push(
        circuitFreeWiringState.active
          ? "(この一覧は現在のワールドの回路そのもの。画面上部の「回路スイッチ(閉)」チェックボックスは固定デモ回路専用のため無効です)"
          : "スイッチの開閉は画面上部の「回路スイッチ(閉)」チェックボックスで操作する。",
      );
      topology.textContent = lines.join("\n");
    } else {
      topology.textContent = "このシーンには回路ドメインがありません。";
    }
    body.appendChild(topology);

    const voltageLine = document.createElement("div");
    voltageLine.id = "circuit-tab-voltage";
    voltageLine.className = "inspector-field";
    body.appendChild(voltageLine);

    const switchCheckbox = document.getElementById(
      "toggle-circuit-switch",
    ) as HTMLInputElement | null;

    // 自由配線回路エディタ(設計docs/23-frontend/01-editor.md §6「回路エディタ
    // サブモード」(D19)の縮約実装、`CircuitEditorRef`のdoc参照)。専用の
    // グラフィカルなノード配線UIではなく、ノード番号を直接指定するフォームで
    // 素子を追加していく形とした(Scene View内の別サブモードは大掛かりな
    // 追加実装が要るため見送った——`sim-em::Circuit`自体は任意のノード対応
    // 素子を既に自由に組める設計であり、本増分はそこへの配線が主眼)。
    const editorHeading = document.createElement("h4");
    editorHeading.textContent = "自由配線回路エディタ";
    body.appendChild(editorHeading);

    const resetForm = document.createElement("div");
    const nodeCountInput = document.createElement("input");
    nodeCountInput.type = "number";
    nodeCountInput.min = "2";
    nodeCountInput.value = "3";
    nodeCountInput.title = "GND(node 0)を含むノード総数";
    const resetButton = document.createElement("button");
    resetButton.textContent = "リセット(新規回路)";
    resetButton.addEventListener("click", () => {
      if (!circuitEditorRef.current) return;
      const numNodes = Math.max(
        2,
        Math.trunc(Number(nodeCountInput.value) || 2),
      );
      circuitEditorRef.current.reset(numNodes);
      circuitFreeWiringState.active = true;
      if (switchCheckbox) {
        switchCheckbox.checked = false;
        switchCheckbox.disabled = true;
      }
      freeWiringNumNodes = numNodes;
      freeWiringComponents.length = 0;
      renderCircuitTab();
    });
    resetForm.append("ノード数(GND含む): ", nodeCountInput, resetButton);
    body.appendChild(resetForm);

    if (freeWiringNumNodes > 0) {
      const addForm = document.createElement("div");
      const aInput = document.createElement("input");
      aInput.type = "number";
      aInput.min = "0";
      aInput.value = "0";
      aInput.title = "ノードA(0=GND)";
      aInput.id = "circuit-editor-node-a";
      const bInput = document.createElement("input");
      bInput.type = "number";
      bInput.min = "0";
      bInput.value = "1";
      bInput.title = "ノードB(0=GND)";
      bInput.id = "circuit-editor-node-b";
      const kindSelect = document.createElement("select");
      kindSelect.id = "circuit-editor-kind";
      for (const [value, label] of [
        ["resistor", "抵抗 [Ω]"],
        ["voltage_source", "電圧源 [V] (A=正極)"],
        ["switch", "スイッチ"],
        ["capacitor", "コンデンサ [F]"],
        ["inductor", "インダクタ [H]"],
        ["diode", "ダイオード (A=アノード)"],
        ["dc_motor", "DCモーター [Ω] (逆起電力定数は別欄)"],
      ]) {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = label;
        kindSelect.appendChild(option);
      }
      const valueInput = document.createElement("input");
      valueInput.type = "number";
      valueInput.value = "100";
      valueInput.id = "circuit-editor-value";
      valueInput.title =
        "抵抗[Ω]/電圧[V]/コンデンサ[F]/インダクタ[H]/ダイオード飽和電流[A]/DCモーター巻線抵抗[Ω]";
      // コンデンサの初期電圧・インダクタの初期電流・ダイオードのnVt・
      // DCモーターの巻線インダクタンスに使う第2値(未使用の種別では無視される)。
      const value2Input = document.createElement("input");
      value2Input.type = "number";
      value2Input.value = "0";
      value2Input.id = "circuit-editor-value2";
      value2Input.title =
        "コンデンサ初期電圧[V]/インダクタ初期電流[A]/ダイオードnVt[V]/DCモーター巻線インダクタンス[H]";
      // DCモーターの逆起電力定数のみに使う第3値。
      const value3Input = document.createElement("input");
      value3Input.type = "number";
      value3Input.value = "0.1";
      value3Input.id = "circuit-editor-value3";
      value3Input.title = "DCモーター逆起電力定数[V·s/rad]";
      const addButton = document.createElement("button");
      addButton.textContent = "素子を追加";
      addButton.addEventListener("click", () => {
        if (!circuitEditorRef.current) return;
        const a = Math.trunc(Number(aInput.value) || 0);
        const b = Math.trunc(Number(bInput.value) || 0);
        const value = Number(valueInput.value) || 0;
        const value2 = Number(value2Input.value) || 0;
        const value3 = Number(value3Input.value) || 0;
        if (kindSelect.value === "resistor") {
          circuitEditorRef.current.addResistor(a, b, value);
          freeWiringComponents.push({
            kind: "resistor",
            a,
            b,
            resistance: value,
          });
        } else if (kindSelect.value === "voltage_source") {
          circuitEditorRef.current.addVoltageSource(a, b, value);
          freeWiringComponents.push({
            kind: "voltage_source",
            a,
            b,
            voltage: value,
          });
        } else if (kindSelect.value === "capacitor") {
          circuitEditorRef.current.addCapacitor(a, b, value, value2);
          freeWiringComponents.push({
            kind: "capacitor",
            a,
            b,
            capacitance: value,
          });
        } else if (kindSelect.value === "inductor") {
          circuitEditorRef.current.addInductor(a, b, value, value2);
          freeWiringComponents.push({
            kind: "inductor",
            a,
            b,
            inductance: value,
          });
        } else if (kindSelect.value === "diode") {
          circuitEditorRef.current.addDiode(
            a,
            b,
            value,
            value2 || 0.026,
          );
          freeWiringComponents.push({ kind: "diode", a, b });
        } else if (kindSelect.value === "dc_motor") {
          const index = circuitEditorRef.current.addDcMotor(
            a,
            b,
            value,
            value2,
            value3,
          );
          freeWiringComponents.push({ kind: "dc_motor", a, b, index });
        } else {
          const index = circuitEditorRef.current.addSwitch(a, b, false);
          freeWiringComponents.push({
            kind: "switch",
            a,
            b,
            index,
            closed: false,
          });
        }
        renderCircuitTab();
      });
      addForm.append(
        "A: ",
        aInput,
        " B: ",
        bInput,
        " ",
        kindSelect,
        " 値: ",
        valueInput,
        " 値2: ",
        value2Input,
        " 値3(DCモーターのみ): ",
        value3Input,
        addButton,
      );
      body.appendChild(addForm);

      const componentList = document.createElement("ul");
      for (const c of freeWiringComponents) {
        const item = document.createElement("li");
        if (c.kind === "resistor") {
          item.textContent = `抵抗 ${c.a}-${c.b}: ${c.resistance}Ω`;
        } else if (c.kind === "voltage_source") {
          item.textContent = `電圧源 ${c.a}(+)-${c.b}(-): ${c.voltage}V`;
        } else if (c.kind === "switch") {
          const switchCheckboxItem = document.createElement("input");
          switchCheckboxItem.type = "checkbox";
          switchCheckboxItem.checked = c.closed;
          switchCheckboxItem.addEventListener("change", () => {
            c.closed = switchCheckboxItem.checked;
            circuitEditorRef.current?.setSwitchClosed(c.index, c.closed);
          });
          item.textContent = `スイッチ ${c.a}-${c.b}: `;
          item.appendChild(switchCheckboxItem);
        } else if (c.kind === "capacitor") {
          item.textContent = `コンデンサ ${c.a}-${c.b}: ${c.capacitance}F`;
        } else if (c.kind === "inductor") {
          item.textContent = `インダクタ ${c.a}-${c.b}: ${c.inductance}H`;
        } else if (c.kind === "diode") {
          item.textContent = `ダイオード ${c.a}(anode)-${c.b}(cathode)`;
        } else {
          const speedInput = document.createElement("input");
          speedInput.type = "number";
          speedInput.value = "0";
          speedInput.title = "角速度 [rad/s]";
          speedInput.addEventListener("change", () => {
            circuitEditorRef.current?.setMotorSpeed(
              c.index,
              Number(speedInput.value) || 0,
            );
          });
          item.textContent = `DCモーター ${c.a}-${c.b}: 速度[rad/s] `;
          item.appendChild(speedInput);
          const currentSpan = document.createElement("span");
          currentSpan.textContent = ` 電流: ${(circuitEditorRef.current?.motorCurrent(c.index) ?? 0).toFixed(3)}A`;
          item.appendChild(currentSpan);
        }
        componentList.appendChild(item);
      }
      body.appendChild(componentList);

      const voltageTable = document.createElement("div");
      voltageTable.id = "circuit-editor-voltages";
      body.appendChild(voltageTable);

      function refreshFreeWiringVoltages() {
        if (!circuitEditorRef.current) return;
        const lines: string[] = [];
        for (let node = 0; node < freeWiringNumNodes; node++) {
          lines.push(
            `Node${node}: ${circuitEditorRef.current.nodeVoltage(node).toFixed(3)}V`,
          );
        }
        voltageTable.textContent = lines.join(" / ");
      }
      refreshFreeWiringVoltages();
      circuitFreeWiringRefresh = refreshFreeWiringVoltages;
    } else {
      circuitFreeWiringRefresh = null;
    }

    function refresh() {
      if (!circuitRef.current) {
        voltageLine.textContent = "Node2電圧: 読み込み中...";
      } else {
        const voltage = circuitRef.current();
        const switchState = switchCheckbox?.checked ? "閉" : "開";
        voltageLine.textContent = circuitFreeWiringState.active
          ? "固定デモ回路は自由配線回路に置き換え済みです"
          : `Node2電圧: ${voltage.toFixed(3)} V (スイッチ: ${switchState})`;
      }
      circuitFreeWiringRefresh?.();
    }
    refresh();

    if (circuitTabRefreshIntervalId !== null) {
      window.clearInterval(circuitTabRefreshIntervalId);
    }
    circuitTabRefreshIntervalId = window.setInterval(refresh, 200);
  }

  function renderPrefabsTab() {
    body.innerHTML = "";
    const note = document.createElement("p");
    note.textContent =
      "選択中のボディの形状+材質をPrefabとして保存し、後で同じ形状+材質のボディを再スポーンできる。";
    body.appendChild(note);

    const saveForm = document.createElement("div");
    const nameInput = document.createElement("input");
    nameInput.type = "text";
    nameInput.placeholder = "Prefab名";
    const saveButton = document.createElement("button");
    saveButton.textContent = "選択中のボディをPrefabとして保存";
    saveButton.addEventListener("click", () => {
      if (!prefabRef.current) return;
      const captured = prefabRef.current.captureSelectedBody();
      if (!captured) return; // 未対応の形状(Plane等)は保存しない。
      const name = nameInput.value.trim() || `Prefab_${prefabs.length + 1}`;
      prefabs.push({ name, ...captured });
      nameInput.value = "";
      renderPrefabsTab();
    });
    saveForm.append(nameInput, saveButton);
    body.appendChild(saveForm);

    const list = document.createElement("ul");
    for (const prefab of prefabs) {
      const item = document.createElement("li");
      item.textContent = `${prefab.name} (${prefab.kind}, ${prefab.material}) `;
      const spawnButton = document.createElement("button");
      spawnButton.textContent = "スポーン";
      spawnButton.addEventListener("click", () => {
        prefabRef.current?.spawn(prefab);
      });
      item.appendChild(spawnButton);
      list.appendChild(item);
    }
    body.appendChild(list);
  }

  function renderMaterialsTab() {
    if (!materialsRef.current) {
      body.textContent = "Materials: 読み込み中...";
      return;
    }
    const props = materialsRef.current();
    body.innerHTML = "";
    const table = document.createElement("table");
    table.className = "materials-table";
    const header = table.insertRow();
    for (const label of [
      "Material",
      "density [kg/m^3]",
      "friction",
      "restitution",
      "specific heat [J/(kg・K)]",
      "conductivity [W/(m・K)]",
    ]) {
      const th = document.createElement("th");
      th.textContent = label;
      header.appendChild(th);
    }
    for (const p of props) {
      const row = table.insertRow();
      for (const value of [
        p.name,
        p.density.toFixed(1),
        p.friction.toFixed(3),
        p.restitution.toFixed(3),
        p.specificHeat.toFixed(1),
        p.conductivity.toFixed(3),
      ]) {
        const td = row.insertCell();
        td.textContent = value;
      }
    }
    body.appendChild(table);
  }

  function renderReplaysTab() {
    body.innerHTML = "";
    const exportButton = document.createElement("button");
    exportButton.textContent = `Export (${commandLog.length}件, JSON)`;
    exportButton.addEventListener("click", () => {
      const blob = new Blob([JSON.stringify(commandLog, null, 2)], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "command_log.json";
      a.click();
      URL.revokeObjectURL(url);
    });
    body.appendChild(exportButton);

    const replayButton = document.createElement("button");
    replayButton.textContent = "▶ Replay実行(検証)";
    replayButton.title =
      "記録済みコマンドを既定シーン(床+箱)の新規Worldへ再送し、最終状態が現在のシーンと一致するか検証する";
    const replayStatus = document.createElement("p");
    replayButton.addEventListener("click", () => {
      if (!replayVerifyRef.current) return;
      const result = replayVerifyRef.current();
      const boxText = (p: [number, number, number]) =>
        `(${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)})`;
      replayStatus.textContent =
        `${result.commandCount}件のコマンドを${result.totalSteps}stepにわたって再生。` +
        `再生後Box_1位置=${boxText(result.finalBoxPosition)} (現在のシーン: ${boxText(result.liveBoxPosition)})。` +
        (result.sceneChanged
          ? "現在のシーンはスポーン/Importで初期状態から変更されているため一致は期待できません。"
          : result.matches
            ? "state_hashが一致——決定論的に同じ結果を再現しました。"
            : "state_hashが一致しませんでした。");
    });
    body.appendChild(replayButton);

    // **ライブ再生(群2、`ReplayPlaybackRef`のdoc参照)**。上の「検証」は
    // ヘッドレスで一気に流して state_hash を比べるだけで、**記録した操作を
    // 目で見る手段が無かった**。こちらは Scene View に流し込んで再生する。
    const livePlayButton = document.createElement("button");
    livePlayButton.id = "btn-replay-live";
    livePlayButton.textContent = "▶ ライブ再生(Scene Viewで見る)";
    livePlayButton.title =
      "記録済みコマンドを別のWorldで再生し、その様子をScene Viewに映す(現在のシーンは変更しない)";
    livePlayButton.addEventListener("click", () => {
      const playback = replayPlaybackRef.current;
      if (!playback) return;
      if (playback.isPlaying()) {
        playback.stop();
        replayStatus.textContent = "ライブ再生を中断しました。";
        livePlayButton.textContent = "▶ ライブ再生(Scene Viewで見る)";
        return;
      }
      const result = playback.start();
      if (!result.started) {
        replayStatus.textContent = result.reason ?? "ライブ再生を開始できませんでした。";
        return;
      }
      replayStatus.textContent = `${commandLog.length}件のコマンドを${result.totalSteps}step分ライブ再生します(時間倍率が効きます)。`;
      livePlayButton.textContent = "⏹ ライブ再生を止める";
    });
    body.appendChild(livePlayButton);
    body.appendChild(replayStatus);

    const list = document.createElement("ul");
    for (const entry of commandLog) {
      const item = document.createElement("li");
      item.textContent = `[step ${entry.step}, t=${entry.t.toFixed(3)}s] ${entry.kind}: ${formatCommandLogDetail(entry)}`;
      list.appendChild(item);
    }
    body.appendChild(list);
  }

  /// 検証タブ(**残タスク完遂の縦串④増分**、設計docs/reviews/
  /// 2026-08-14-editor-implementation-plan.md「シーンに合格基準を書けるように
  /// し、パラメータを振って`run_headless_scenario`をN回実行、結果を重ねて
  /// グラフ・差分表示する」)。
  ///
  /// 合格基準(probe index・比較演算子・しきい値)は`Scenario::pass_criteria`
  /// としてシーンJSONスキーマの一部になっている(`prediction_prompts`と同じ
  /// 著者向けメタデータ扱い、物理には影響しない)。このタブはBase scene JSON
  /// に`pass_criteria`があれば読み込んでフォームへ反映し、「基準をシーンJSON
  /// へ書き込む」ボタンでフォームの内容を逆にJSONへ書き戻せる——UI状態と
  /// シーンJSONが往復する。
  const operatorToJson: Record<string, string> = { ">=": "ge", "<=": "le", "~=": "approx" };
  const operatorFromJson: Record<string, string> = { ge: ">=", le: "<=", approx: "~=" };

  function renderValidationTab() {
    body.innerHTML = "";

    const baseJsonArea = document.createElement("textarea");
    baseJsonArea.id = "validation-base-json";
    baseJsonArea.rows = 6;
    baseJsonArea.style.width = "100%";
    baseJsonArea.style.fontFamily = "monospace";
    baseJsonArea.style.fontSize = "11px";
    try {
      baseJsonArea.value = validationBaseJsonRef.current?.() ?? "";
    } catch {
      baseJsonArea.value = "";
    }
    const reloadButton = document.createElement("button");
    reloadButton.textContent = "現在のシーンを読み込む";
    reloadButton.addEventListener("click", () => {
      try {
        baseJsonArea.value = validationBaseJsonRef.current?.() ?? "";
      } catch (err) {
        reportError(`シーンの読み込みに失敗しました: ${String(err)}`);
      }
      loadCriteriaFromBaseJson();
    });
    baseJsonArea.addEventListener("change", () => loadCriteriaFromBaseJson());
    body.append("Base scene JSON: ", reloadButton, baseJsonArea);

    const form = document.createElement("div");
    const pathInput = document.createElement("input");
    pathInput.type = "text";
    pathInput.placeholder = "例: world.gravity";
    pathInput.title =
      "スイープするフィールドのドット区切りパス(配列は数値indexを使う、例: bodies.0.position.1)";
    const valuesInput = document.createElement("input");
    valuesInput.type = "text";
    valuesInput.placeholder = "例: 5,10,15,20";
    valuesInput.title = "パラメータの値(カンマ区切り)。この個数ぶん実行する。";
    const stepsInput = document.createElement("input");
    stepsInput.type = "number";
    stepsInput.min = "1";
    stepsInput.step = "1";
    stepsInput.value = "300";
    stepsInput.title = "各実行のstep数";
    form.append(
      "パス: ",
      pathInput,
      " 値: ",
      valuesInput,
      " step数: ",
      stepsInput,
    );
    body.appendChild(form);

    const criteriaForm = document.createElement("div");
    const probeIndexInput = document.createElement("input");
    probeIndexInput.type = "number";
    probeIndexInput.min = "0";
    probeIndexInput.step = "1";
    probeIndexInput.value = "0";
    probeIndexInput.title = "合格基準・グラフ化の対象にするprobe index(シーンのprobes配列の順)";
    const operatorSelect = document.createElement("select");
    for (const [value, label] of [
      [">=", "以上(>=)"],
      ["<=", "以下(<=)"],
      ["~=", "近似一致(|差|<0.01)"],
    ] as const) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = label;
      operatorSelect.appendChild(option);
    }
    const thresholdInput = document.createElement("input");
    thresholdInput.type = "number";
    thresholdInput.step = "0.01";
    thresholdInput.value = "0";
    thresholdInput.title = "合格基準のしきい値(probeの最終値と比較する)";
    const writeCriteriaButton = document.createElement("button");
    writeCriteriaButton.textContent = "基準をシーンJSONへ書き込む";
    criteriaForm.append(
      "合格基準: probe ",
      probeIndexInput,
      " の最終値 ",
      operatorSelect,
      " ",
      thresholdInput,
      " ",
      writeCriteriaButton,
    );
    body.appendChild(criteriaForm);

    /// Base scene JSONの`pass_criteria[0]`をフォームへ反映する(あれば)。
    function loadCriteriaFromBaseJson(): void {
      try {
        const obj = JSON.parse(baseJsonArea.value) as {
          pass_criteria?: { probe_index: number; operator: string; threshold: number }[];
        };
        const criterion = obj.pass_criteria?.[0];
        if (!criterion) return;
        probeIndexInput.value = String(criterion.probe_index);
        operatorSelect.value = operatorFromJson[criterion.operator] ?? ">=";
        thresholdInput.value = String(criterion.threshold);
      } catch {
        // Base scene JSONがまだ不正/空の場合は何もしない。
      }
    }
    loadCriteriaFromBaseJson();

    writeCriteriaButton.addEventListener("click", () => {
      let obj: Record<string, unknown>;
      try {
        obj = JSON.parse(baseJsonArea.value) as Record<string, unknown>;
      } catch (err) {
        reportError(`Base scene JSONが不正です: ${String(err)}`);
        return;
      }
      obj.pass_criteria = [
        {
          probe_index: Math.max(0, Math.trunc(Number(probeIndexInput.value) || 0)),
          operator: operatorToJson[operatorSelect.value] ?? "ge",
          threshold: Number(thresholdInput.value) || 0,
        },
      ];
      baseJsonArea.value = JSON.stringify(obj, null, 2);
    });

    const runButton = document.createElement("button");
    runButton.textContent = "スイープを実行";
    body.appendChild(runButton);

    const resultsContainer = document.createElement("div");
    body.appendChild(resultsContainer);

    /// ドット区切りパスの末端へ数値を書き込む(配列要素は数値文字列のキーで
    /// 到達する、`JSON.parse`直後のプレーンオブジェクト/配列に対して動く)。
    function setJsonPath(root: unknown, path: string, value: number): void {
      const parts = path.split(".").filter((p) => p.length > 0);
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      let cur: any = root;
      for (let i = 0; i < parts.length - 1; i++) {
        cur = cur[parts[i]];
        if (cur === undefined) {
          throw new Error(`パス "${path}" の "${parts[i]}" がシーンJSONに無い`);
        }
      }
      const lastKey = parts[parts.length - 1];
      if (cur[lastKey] === undefined) {
        throw new Error(`パス "${path}" の "${lastKey}" がシーンJSONに無い`);
      }
      cur[lastKey] = value;
    }

    function evaluateCriterion(finalValue: number): boolean {
      const threshold = Number(thresholdInput.value);
      switch (operatorSelect.value) {
        case ">=":
          return finalValue >= threshold;
        case "<=":
          return finalValue <= threshold;
        case "~=":
          return Math.abs(finalValue - threshold) < 0.01;
        default:
          return false;
      }
    }

    runButton.addEventListener("click", () => {
      resultsContainer.innerHTML = "";
      let baseObj: unknown;
      try {
        baseObj = JSON.parse(baseJsonArea.value);
      } catch (err) {
        reportError(`Base scene JSONが不正です: ${String(err)}`);
        return;
      }
      const path = pathInput.value.trim();
      const values = valuesInput.value
        .split(",")
        .map((s) => Number(s.trim()))
        .filter((n) => Number.isFinite(n));
      const steps = Math.max(1, Math.trunc(Number(stepsInput.value) || 1));
      const probeIndex = Math.max(0, Math.trunc(Number(probeIndexInput.value) || 0));
      if (values.length === 0) {
        reportError("値を1つ以上指定してください(カンマ区切り)。");
        return;
      }

      type RunResult = {
        value: number;
        finalTime: number;
        finalStateHash: string;
        probeFinal: number | null;
        probeHistory: number[];
        pass: boolean | null;
        error: string | null;
      };
      const runs: RunResult[] = [];
      for (const value of values) {
        try {
          const variant = JSON.parse(JSON.stringify(baseObj));
          if (path.length > 0) setJsonPath(variant, path, value);
          const resultJson = run_headless_scenario_json(
            JSON.stringify(variant),
            steps,
          );
          const result = JSON.parse(resultJson) as {
            final_state_hash: string;
            final_time: number;
            probe_histories: number[][];
          };
          const history = result.probe_histories[probeIndex] ?? [];
          const probeFinal = history.length > 0 ? history[history.length - 1] : null;
          runs.push({
            value,
            finalTime: result.final_time,
            finalStateHash: result.final_state_hash,
            probeFinal,
            probeHistory: history,
            pass: probeFinal === null ? null : evaluateCriterion(probeFinal),
            error: null,
          });
        } catch (err) {
          runs.push({
            value,
            finalTime: NaN,
            finalStateHash: "",
            probeFinal: null,
            probeHistory: [],
            pass: null,
            error: String(err),
          });
        }
      }

      // 結果テーブル(差分表示)。
      const table = document.createElement("table");
      table.className = "validation-results-table";
      const headerRow = table.insertRow();
      for (const label of [
        "パラメータ値",
        "final_time",
        "final_state_hash",
        `probe[${probeIndex}]最終値`,
        "合格基準",
      ]) {
        const th = document.createElement("th");
        th.textContent = label;
        headerRow.appendChild(th);
      }
      for (const run of runs) {
        const row = table.insertRow();
        row.insertCell().textContent = String(run.value);
        row.insertCell().textContent = run.error
          ? `エラー: ${run.error}`
          : run.finalTime.toFixed(4);
        row.insertCell().textContent = run.finalStateHash;
        row.insertCell().textContent =
          run.probeFinal === null ? "—" : run.probeFinal.toFixed(6);
        row.insertCell().textContent =
          run.pass === null ? "—" : run.pass ? "✓ PASS" : "✗ FAIL";
      }
      resultsContainer.appendChild(table);

      // probe履歴の重ね書きグラフ(実行ごとに色を変える)。
      const canvas = document.createElement("canvas");
      canvas.width = 480;
      canvas.height = 200;
      canvas.className = "validation-chart";
      resultsContainer.appendChild(canvas);
      const ctx = canvas.getContext("2d");
      const historiesWithData = runs.filter((r) => r.probeHistory.length > 1);
      if (ctx && historiesWithData.length > 0) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        let min = Infinity;
        let max = -Infinity;
        for (const run of historiesWithData) {
          for (const v of run.probeHistory) {
            if (v < min) min = v;
            if (v > max) max = v;
          }
        }
        if (min === max) {
          min -= 1;
          max += 1;
        }
        const colors = [
          "#6cf",
          "#f96",
          "#9f6",
          "#f6c",
          "#fc6",
          "#c9f",
          "#6fc",
          "#f66",
        ];
        historiesWithData.forEach((run, runIndex) => {
          ctx.strokeStyle = colors[runIndex % colors.length];
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          run.probeHistory.forEach((v, i) => {
            const x = (i / (run.probeHistory.length - 1)) * canvas.width;
            const y =
              canvas.height - ((v - min) / (max - min)) * canvas.height;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          });
          ctx.stroke();
        });
        const legend = document.createElement("div");
        legend.className = "validation-chart-legend";
        historiesWithData.forEach((run, runIndex) => {
          const swatch = document.createElement("span");
          swatch.className = "validation-chart-swatch";
          swatch.style.background = colors[runIndex % colors.length];
          const label = document.createElement("span");
          label.textContent = ` ${path || "run"}=${run.value}`;
          legend.append(swatch, label);
        });
        resultsContainer.appendChild(legend);
      }
    });
  }

  function show(tab: string) {
    tabs.forEach((t) => t.classList.toggle("active", t.dataset.tab === tab));
    if (tab !== "circuit" && circuitTabRefreshIntervalId !== null) {
      window.clearInterval(circuitTabRefreshIntervalId);
      circuitTabRefreshIntervalId = null;
    }
    if (tab === "scenes") {
      renderScenesTab();
      return;
    }
    if (tab === "materials") {
      renderMaterialsTab();
      return;
    }
    if (tab === "replays") {
      renderReplaysTab();
      return;
    }
    if (tab === "circuit") {
      renderCircuitTab();
      return;
    }
    if (tab === "prefabs") {
      renderPrefabsTab();
      return;
    }
    if (tab === "validation") {
      renderValidationTab();
      return;
    }
    body.textContent = staticContentByTab[tab] ?? "";
  }
  // **Project ドロワーの開閉(増分E3)**。既定のグリッド行はタブバーの高さ(28px)
  // しか無く、本体は画面外に押し出されて実ユーザーには到達不能だった(実測: viewport
  // 900px に対し `#project-body` の top が 903px)。タブをクリックしたら開き、
  // **開いている状態で同じタブをもう一度クリックしたら閉じる**(Scene View を
  // 広く使いたいときに戻せるようにする)。開閉状態は `#app` の `data-drawer` に
  // 持たせ、CSS変数 `--project-row` で高さを切り替える——レイアウトプリセットと
  // `grid-template-rows` を奪い合わないようにするため(`style.css` 参照)。
  const app = document.getElementById("app")!;
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      const name = tab.dataset.tab!;
      const isActive = tab.classList.contains("active");
      const isOpen = app.dataset.drawer === "open";
      if (isActive && isOpen) {
        delete app.dataset.drawer;
        return;
      }
      app.dataset.drawer = "open";
      show(name);
    });
  });
  show("scenes");
}

// Probe Graphsパネル(設計docs/23-frontend/01-editor.md §1.4「Probeグラフ:
// シーン定義の観測量を時系列表示」)のデモ。複数系列(箱のy座標・箱の速さ)を
// 各系列独立の自動スケーリングで重ね描きする(値のレンジが大きく異なる系列
// (m単位のy座標 vs m/s単位の速さ)を同一軸に正規化すると見づらいため、
// 系列ごとに独立してmin/maxを取り0..canvas高さへ正規化する設計)。
//
// **増分E1で対数軸とCSVエクスポートを追加した**(設計§1.4のフル仕様):
// - **対数軸**: 桁の異なる量(例: 指数減衰する温度と線形に進む座標)を比べる
//   ための表示。放射輝度・温度・エネルギー等は正だが座標・速度は負にもなり得る
//   ため、素直な`log10(v)`では大半の系列が描けなくなる。そこで**符号を保つ
//   対数変換** `sign(v)*log10(1+|v|)`(いわゆる symlog)を使う——これは
//   v=0で0、|v|が小さい領域では線形に振る舞い、大きい領域で対数になる連続かつ
//   単調な変換なので、負値・ゼロを含む系列でも破綻しない。純粋な`log10`では
//   ないことを正直に明記する(真の対数軸が要るなら正の量に限る必要がある)。
// - **CSVエクスポート**: 表示中の全系列の履歴をダウンロードする。系列ごとに
//   履歴長が異なり得る(プローブの登録タイミングが違う)ため、**最長の系列に
//   合わせて短い系列の末尾を空欄で埋める**。Probeのリングバッファは絶対時刻を
//   持たないため、時刻列ではなく**サンプル番号**を出す(縮約、下記doc参照)。
type ProbeSeries = { label: string; color: string; history: Float64Array };

/// 符号を保つ対数変換(symlog)。`type ProbeSeries`のdoc参照。
function signedLog(v: number): number {
  return Math.sign(v) * Math.log10(1 + Math.abs(v));
}

/// 表示中の全系列をCSV文字列にする。1列目はサンプル番号
/// (`sim_math::RingBuffer`は絶対時刻を保持しないため、時刻列は出せない——
/// 出すなら`World`側にサンプル時刻も積む変更が要るので後続増分の対象)。
/// 系列ごとに履歴長が違い得るので最長に合わせ、短い系列は空欄で埋める。
function probeSeriesToCsv(series: ProbeSeries[]): string {
  const rows = series.reduce((m, s) => Math.max(m, s.history.length), 0);
  const escape = (s: string) =>
    /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
  const lines = [
    ["sample", ...series.map((s) => s.label)].map(escape).join(","),
  ];
  for (let i = 0; i < rows; i++) {
    const cells = [String(i)];
    for (const s of series) {
      cells.push(i < s.history.length ? String(s.history[i]) : "");
    }
    lines.push(cells.join(","));
  }
  return lines.join("\n");
}

// シーンギャラリー読み込み時(`isGalleryScene`参照)は、`scenario.probes`が
// 宣言した本数の系列を動的に束ねる(D6=1本・D11=2本のように既定シーンの2系列
// 固定とは限らない)。乱数は決定論を重視するこのプロジェクトの流儀に反するため
// 使わず、この配列を`index % PROBE_GRAPH_COLORS.length`で決定的に巡回させる
// (`render()`内の`isGalleryScene`分岐参照)。
const PROBE_GRAPH_COLORS = ["#9cf", "#fc6", "#f9c", "#9fc", "#c9f", "#ffcc99"];

function setUpProbeGraph(): (
  series: ProbeSeries[],
  dt: number,
  currentTime: number,
) => void {
  const canvas = document.getElementById("probe-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const logToggle = document.getElementById(
    "toggle-probe-log",
  ) as HTMLInputElement;
  const csvButton = document.getElementById(
    "btn-probe-csv",
  ) as HTMLButtonElement;
  const timeRangeLabel = document.getElementById("probe-time-range")!;
  const emptyState = document.getElementById("probe-empty");

  // CSVボタンは「今まさに描かれている系列」を書き出す。`render()`が毎フレーム
  // 渡してくる最新の配列をここで覚えておく(描画とエクスポートで同じデータを
  // 使うため、別経路でクエリし直して食い違うのを避ける)。
  let latest: ProbeSeries[] = [];
  csvButton.addEventListener("click", () => {
    if (latest.length === 0) return;
    // 押しても何も起きないボタンは「壊れている」と読まれるので、下の
    // `csvButton.disabled` で先に押せなくしてある。ここは念のための保険。
    const blob = new Blob([probeSeriesToCsv(latest)], {
      type: "text/csv;charset=utf-8",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "probes.csv";
    a.click();
    URL.revokeObjectURL(url);
  });

  return (series: ProbeSeries[], dt: number, currentTime: number) => {
    latest = series;
    // **空状態**(増分「UI 品質の底上げ」)。描ける系列(サンプル 2 点以上)が
    // 1 本も無いあいだは、黒い矩形ではなく「何をすれば線が出るか」を出す。
    const drawable = series.filter((s) => s.history.length >= 2);
    if (emptyState) emptyState.hidden = drawable.length > 0;
    canvas.hidden = drawable.length === 0;
    csvButton.disabled = drawable.length === 0;
    if (drawable.length === 0) {
      timeRangeLabel.textContent = "";
      return;
    }

    const useLog = logToggle.checked;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    ctx.clearRect(0, 0, w, h);
    ctx.font = "11px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";

    // **目盛り線**(増分「UI 品質の底上げ」)。系列ごとに独立して正規化する
    // 設計(下記)なので共通の値軸は引けないが、**高さの 1/4 ごとの水平線**が
    // あるだけで「どのくらい動いたか」「振動しているのか単調なのか」が
    // 目で追えるようになる。線は地に沈む明度に抑え、データを隠さない。
    ctx.strokeStyle = "rgba(255, 255, 255, 0.07)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 1; i < 4; i++) {
      const y = Math.round((h * i) / 4) + 0.5;
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
    }
    ctx.stroke();

    // QA不具合9: グラフのどこが何秒なのか画面から分からず、当時のリングバッファの
    // 打ち切りと相まって「第1バウンドを数回あとの極大と取り違える」ような読み違いが
    // 実際に起きた。最長の系列を基準に、画面の左端(古い側)〜右端(新しい側、
    // = 現在時刻)の時刻を表示する——`ProbeSeries`自体は絶対時刻を持たない
    // (`probeSeriesToCsv`のdoc参照)ので、`currentTime`と`dt`から逆算する。
    // **打ち切り自体は解消済み**(`sim_world::Probe`のdoc参照: 履歴は可変長に
    // なり、古いサンプルが無言で捨てられることは無くなった)なので、この
    // 逆算は常に「本当の開始時刻」を指す。
    const longest = series.reduce((m, s) => Math.max(m, s.history.length), 0);
    if (longest >= 2 && dt > 0) {
      const oldestTime = currentTime - (longest - 1) * dt;
      timeRangeLabel.textContent = `t = ${oldestTime.toFixed(2)}s 〜 ${currentTime.toFixed(2)}s`;
    } else {
      timeRangeLabel.textContent = "";
    }

    let legendY = 12;
    for (const s of series) {
      if (s.history.length < 2) continue;

      // 凡例には**元の値**のmin/maxを出す(対数軸はあくまで表示上の変換であり、
      // 観測値そのものは変わらないため。ここで変換後の値を出すと読み手が
      // 実測値を誤読する)。正規化には変換後の値を使う。
      let min = Infinity;
      let max = -Infinity;
      for (const v of s.history) {
        if (v < min) min = v;
        if (v > max) max = v;
      }
      const plot = (v: number) => (useLog ? signedLog(v) : v);
      const plotMin = Math.min(plot(min), plot(max));
      const plotMax = Math.max(plot(min), plot(max));
      const range = plotMax - plotMin > 1e-12 ? plotMax - plotMin : 1.0;

      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < s.history.length; i++) {
        const x = (i / (s.history.length - 1)) * w;
        const y = h - ((plot(s.history[i]) - plotMin) / range) * h;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // 凡例は折れ線の上に重なるので、**濃い縁取りを先に引いてから**塗る
      // (以前は素の塗りだけで、線と同系色の場所では文字が読めなかった)。
      const suffix = useLog ? " [log]" : "";
      const legendText = `${s.label}: max=${max.toFixed(2)} min=${min.toFixed(2)}${suffix}`;
      ctx.lineJoin = "round";
      ctx.lineWidth = 3;
      ctx.strokeStyle = "rgba(8, 10, 13, 0.85)";
      ctx.strokeText(legendText, 4, legendY);
      ctx.fillStyle = s.color;
      ctx.fillText(legendText, 4, legendY);
      legendY += 13;
    }
  };
}

async function setUpSceneView(
  updateProbeGraph: (
    series: ProbeSeries[],
    dt: number,
    currentTime: number,
  ) => void,
  appendConsoleEntries: (eventsText: string) => void,
  clearConsole: () => void,
  jumpToStepRef: JumpToStepRef,
  materialsRef: MaterialsRef,
  circuitRef: CircuitRef,
  sceneExportRef: SceneExportRef,
  projectBundleRef: ProjectBundleRef,
  projectExportedRef: ProjectExportedRef,
  sceneImportRef: SceneImportRef,
  replayVerifyRef: ReplayVerifyRef,
  replayPlaybackRef: ReplayPlaybackRef,
  circuitEditorRef: CircuitEditorRef,
  circuitFreeWiringState: CircuitFreeWiringState,
  prefabRef: PrefabRef,
  prefabSaveRef: PrefabSaveRef,
  sceneGalleryRef: SceneGalleryRef,
  selectBodyRef: SelectBodyRef,
  circuitElementsRef: CircuitElementsRef,
  consoleDiagnosticsRef: ConsoleDiagnosticsRef,
  validationBaseJsonRef: ValidationBaseJsonRef,
  guidedApiRef: GuidedApiRef,
) {
  await init();
  let world = new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT);
  // ギャラリーシーン(`sceneGalleryRef.current`)を読み込み中かどうか
  // (増分B1「シーン定義プローブをProbe Graphsパネルへ配線」)。既定シーンでは
  // `y_probe`/`speed_probe`(BodyPosY/BodySpeedの固定2系列)をそのまま表示し、
  // ギャラリーシーンでは`readNumber(world, "imported_probe_count")`本のシーン定義プローブへ
  // 切り替える(`render()`内`updateProbeGraph`呼び出し参照)。既定シーンへ戻す
  // UI(リロード以外)は現状無いため、falseへ戻す経路は無い(正直な制約)。
  let isGalleryScene = false;
  circuitEditorRef.current = {
    reset: (numNodes: number) =>
      applyComponent(world, "circuit_editor_reset", { num_nodes: numNodes }),
    addResistor: (a, b, resistance) =>
      applyComponent(world, "circuit_editor_add_resistor", { a, b, resistance }),
    addVoltageSource: (a, b, voltage) =>
      applyComponent(world, "circuit_editor_add_voltage_source", { a, b, voltage }),
    addSwitch: (a, b, closed) =>
      applyComponent(world, "circuit_editor_add_switch", { a, b, closed }).index as number,
    setSwitchClosed: (index, closed) =>
      applyComponent(world, "circuit_editor_set_switch_closed", { index, closed }),
    nodeVoltage: (node) => readNumber(world, "circuit_node_voltage", String(node)),
    addCapacitor: (a, b, capacitance, initialVoltage) =>
      applyComponent(world, "circuit_editor_add_capacitor", {
        a,
        b,
        capacitance,
        initial_voltage: initialVoltage,
      }),
    addInductor: (a, b, inductance, initialCurrent) =>
      applyComponent(world, "circuit_editor_add_inductor", {
        a,
        b,
        inductance,
        initial_current: initialCurrent,
      }),
    addDiode: (anode, cathode, saturationCurrent, nVt) =>
      applyComponent(world, "circuit_editor_add_diode", {
        anode,
        cathode,
        saturation_current: saturationCurrent,
        n_vt: nVt,
      }),
    addDcMotor: (a, b, windingResistance, windingInductance, backEmfConstant) =>
      applyComponent(world, "circuit_editor_add_dc_motor", {
        a,
        b,
        winding_resistance: windingResistance,
        winding_inductance: windingInductance,
        back_emf_constant: backEmfConstant,
      }).index as number,
    setMotorSpeed: (index, angularVelocity) =>
      applyComponent(world, "circuit_editor_set_motor_speed", {
        index,
        angular_velocity: angularVelocity,
      }),
    motorCurrent: (index) =>
      readNumber(world, "circuit_editor_motor_current", String(index)),
  };
  materialsRef.current = () =>
    SPAWN_MATERIALS.map((name) => {
      const [density, friction, restitution, specificHeat, conductivity] = JSON.parse(
        world.read_component("material_properties_f64", name),
      ) as number[];
      return {
        name,
        density,
        friction,
        restitution,
        specificHeat,
        conductivity,
      };
    });
  circuitRef.current = () => readNumber(world, "circuit_divider_voltage");
  // Circuitタブ・Hierarchyの「Circuits」が実際の素子を読むための配線
  // (**増分G2**、`CircuitElementsRef`のdoc参照)。`world`はギャラリーからの
  // シーン読み込みで再束縛されるため、クロージャで毎回現在のものを見る。
  circuitElementsRef.current = () => {
    const count = readNumber(world, "circuit_element_count");
    const labels: string[] = [];
    for (let i = 0; i < count; i += 1)
      labels.push(world.read_component("circuit_element_label_at", String(i)));
    return labels;
  };
  sceneExportRef.current = () => {
    const count = readNumber(world, "body_count");
    const bodies: SceneBodyExport[] = [];
    for (let i = 0; i < count; i++) {
      // `body_position_at_f32`はWasmメモリを直接指す一時的なビューを返す
      // (B16、`HotPathViewBuffers`のdoc参照)ため、後続の`read_component`呼び出し
      // を挟む前にプレーンな配列へ読み切っておく(でないと`position`フィールドの
      // 評価時には`pos`がもう指しているメモリの中身が変わっている恐れがある)。
      const [px, py, pz] = world.body_position_at_f32(i);
      bodies.push({
        index: i,
        label: world.read_component("body_label_at", String(i)),
        shape: world.read_component("body_shape_label_at", String(i)),
        material: world.read_component("body_material_label_at", String(i)),
        position: [px, py, pz],
        isStatic: (world.read_component("body_is_static_at", String(i)) === "true"),
      });
    }
    return bodies;
  };
  validationBaseJsonRef.current = () => world.read_component("export_scene_json", "");

  const host = document.getElementById("scene-view-canvas-host")!;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 1000);
  camera.position.set(6, 4, 10);
  camera.lookAt(0, 3, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  host.appendChild(renderer.domElement);

  // **カメラ操作(群2で追加)**。設計 docs/23-frontend/01-editor.md §1.2
  // 「中クリック回転・右クリックパン・ホイールでズーム」。
  //
  // **これが無かったあいだ、カメラは`position.set(6,4,10)`の完全固定だった**
  // ——Unityのようなツールを名乗る上で最も基本的な欠落で、視点を変えられないため
  // 物体の裏側も、大きなシーン(D8の球50個・天体シーン)の全体も見られなかった。
  //
  // **左ボタンは割り当てない**: 左は選択・ギズモ操作・Grabに既に使われている
  // (`pointerdown`ハンドラ参照)。OrbitControlsの既定は左=回転なので、
  // そのまま入れると選択が一切できなくなる。設計が中/右を指定しているのは
  // まさにこの住み分けのためである。
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.mouseButtons = {
    LEFT: null,
    MIDDLE: THREE.MOUSE.ROTATE,
    RIGHT: THREE.MOUSE.PAN,
  };
  orbit.enableDamping = true;
  orbit.dampingFactor = 0.12;
  orbit.target.set(0, 1.5, 0);
  orbit.update();
  // Playwright からカメラ位置を観測するための露出(テスト専用、実行時の
  // 挙動には影響しない)。カメラが実際に動いたことを座標で検証するため。
  (window as unknown as { __camera: THREE.PerspectiveCamera }).__camera =
    camera;
  // 同じ理由で `world` と `scene` も露出する(群3)。**getter にする**のが要点
  // ——`world` はギャラリー読み込みで再束縛されるため、値をそのまま代入すると
  // 差し替え前の古いインスタンスを掴んだままになる。
  Object.defineProperty(window, "__world", { get: () => world, configurable: true });
  Object.defineProperty(window, "__scene", { get: () => scene, configurable: true });
  // テスト専用フック: シーンギャラリーの「ワールドを差し替えて読み込み」
  // (`sceneGalleryRef.current`)を任意のJSON文字列で直接呼べるようにする
  // (`__camera`/`__world`/`__scene`と同じテスト専用露出、実行時の挙動には
  // 影響しない)。**「新規シーン」ボタン(`btn-new-scene`、このすぐ下)は
  // 実UIから到達できる経路として別途ある**——レビュー指摘(「出来るように
  // して欲しい」)を受けて追加した。このフックは「新規シーン」の固定JSON
  // 以外の任意シーン(例: D24相当の組み立て開始点)をテストから注入する
  // 目的でなお使う。
  (window as unknown as { __loadSceneJson?: (json: string) => void }).__loadSceneJson =
    (json: string) => {
      sceneGalleryRef.current?.(json);
    };

  // **「新規シーン」ボタン**(レビュー指摘対応、`docs/22-roadmap/03-editor-todo.md`
  // 「縦串①の受け入れテストを緑にする」の項参照)。既定の起動シーンは回路・熱
  // ドメインの実演セットアップを最初から積んでおり、それを消して「床だけ」から
  // 始める手段がUIに無かった(受け入れテストがテスト専用フックに頼らざるを
  // 得なかった理由そのもの)。シーンギャラリーの差し替え経路
  // (`sceneGalleryRef.current`)を、床の静的Planeボディ1個だけを持つ固定
  // シーンJSONで呼ぶだけの薄い実装——ギャラリーからシーンを選ぶのと全く同じ
  // 「ワールド全体を差し替える」処理を再利用するため、新しい差し替えロジックは
  // 増やさない。
  document.getElementById("btn-new-scene")!.addEventListener("click", () => {
    sceneGalleryRef.current?.(NEW_SCENE_JSON);
  });

  // **ツール状態(群2)**。設計 §1.2 の W/E/R/Q に対応する。
  //
  // **`"sketch"`(S)はD1で追加**。既存の4つと同じ排他トグルの一員にした
  // ——Scene View のクリックが「選択」ではなく「頂点を置く」に変わるのは
  // まさにツールの切り替えであり、別建てのモードを増やすと W/E/R/Q との
  // 排他関係を二重に管理することになる。Gizmo の表示条件は
  // `gizmoTool === "translate"` 等の等値比較なので、変種が増えても
  // スケッチ中は3つとも自動的に隠れる(追加の分岐が要らない)。
  type GizmoTool = "translate" | "rotate" | "scale" | "none" | "sketch";
  let gizmoTool: GizmoTool = "translate";
  const toolButtons = new Map<GizmoTool, HTMLButtonElement>();
  /// スケッチツールの出入りでパネルの開閉とプレビュー描画を切り替える
  /// (実体はスケッチ機能の配線が済んだ後で差し込まれる、`*Ref`の既存パターン)。
  const sketchToolRef: { current: ((active: boolean) => void) | null } = {
    current: null,
  };
  function setGizmoTool(tool: GizmoTool) {
    gizmoTool = tool;
    for (const [t, button] of toolButtons)
      button.classList.toggle("active", t === tool);
    sketchToolRef.current?.(tool === "sketch");
  }
  for (const [id, tool] of [
    ["btn-tool-translate", "translate"],
    ["btn-tool-rotate", "rotate"],
    ["btn-tool-scale", "scale"],
    ["btn-tool-select", "none"],
    ["btn-tool-sketch", "sketch"],
  ] as [string, GizmoTool][]) {
    const button = document.getElementById(id) as HTMLButtonElement | null;
    if (!button) continue;
    toolButtons.set(tool, button);
    button.addEventListener("click", () => setGizmoTool(tool));
  }
  setGizmoTool("translate");

  // **Gizmo 座標系 World/Local(群2)**。設計 §1.2「座標系は World / Local 切替可」。
  // これまで Gizmo の軸は常に世界軸固定で、傾いた物体を「その物体の軸方向へ」
  // 動かす手段が無かった。
  type GizmoSpace = "world" | "local";
  let gizmoSpace: GizmoSpace = "world";
  const gizmoSpaceButton = document.getElementById(
    "btn-gizmo-space",
  ) as HTMLButtonElement | null;
  function setGizmoSpace(space: GizmoSpace) {
    gizmoSpace = space;
    if (gizmoSpaceButton) {
      gizmoSpaceButton.dataset.space = space;
      gizmoSpaceButton.textContent =
        space === "world" ? "🌐 World" : "📦 Local";
    }
  }
  gizmoSpaceButton?.addEventListener("click", () =>
    setGizmoSpace(gizmoSpace === "world" ? "local" : "world"),
  );

  // **Settings(⚙)ポップオーバー(群2で追加)**。設計 §2「Settings(⚙):
  // レンダリング品質・グリッド・ショートカット・PRNGシードの一括変更」。
  //
  // **オーバーレイのチェックボックス群をここへ移した**——ツールバーに
  // 直に並べていたため6個のラベルが縦積みに折り返し、ボタンが読めない
  // ほど混雑していた(実際にスクリーンショットで確認した)。
  // 常時操作するものではないので、まとめて畳む。
  const settingsButton = document.getElementById(
    "btn-settings",
  ) as HTMLButtonElement | null;
  const settingsPopover = document.getElementById(
    "settings-popover",
  ) as HTMLElement | null;
  if (settingsButton && settingsPopover) {
    settingsButton.addEventListener("click", () => {
      settingsPopover.hidden = !settingsPopover.hidden;
    });
    // ポップオーバーの外side をクリックしたら閉じる。
    document.addEventListener("pointerdown", (event) => {
      if (settingsPopover.hidden) return;
      const target = event.target as Node;
      if (!settingsPopover.contains(target) && target !== settingsButton) {
        settingsPopover.hidden = true;
      }
    });
  }

  // **物理パラメータの実行時変更(群2)**。重力を変えて挙動を見るのは
  // 「物理法則を試せるツール」の最も基本的な使い方だが、これまで
  // フロントエンドから触る手段が一切無かった(wasmにsetterが無かった)。
  const gravityInput = document.getElementById(
    "input-gravity",
  ) as HTMLInputElement | null;
  const dtInput = document.getElementById(
    "input-dt",
  ) as HTMLInputElement | null;
  const gridSnapInput = document.getElementById(
    "input-grid-snap",
  ) as HTMLInputElement | null;
  function syncSettingsInputs() {
    if (gravityInput && document.activeElement !== gravityInput) {
      gravityInput.value = Number(world.read_component("gravity", "")).toFixed(3);
    }
    const gravityDirection = JSON.parse(
      world.read_component("gravity_direction", ""),
    ) as number[];
    gravityDirectionInputs.forEach((input, i) => {
      if (input && document.activeElement !== input) {
        input.value = gravityDirection[i].toFixed(3);
      }
    });
    if (dtInput && document.activeElement !== dtInput) {
      dtInput.value = Number(world.read_component("dt", "")).toFixed(6);
    }
    if (dtInput) dtInput.disabled = mode !== "edit";

    // 環境パネル(残タスク完遂の縦串③増分)。シーンJSON(`world.atmosphere`/
    // `fluids[].static_water`)経由で設定された場合もここで拾えるよう、
    // `World::environment()`が読む同じフィールドを毎フレーム反映する
    // (フォーカス中の欄は書き換えない、`updateInspectorRigidBodyFields`と
    // 同じ理由)。
    const density = Number(world.read_component("atmosphere_density", ""));
    if (atmosphereToggle) atmosphereToggle.checked = !Number.isNaN(density);
    if (atmosphereDensityInput && document.activeElement !== atmosphereDensityInput && !Number.isNaN(density)) {
      atmosphereDensityInput.value = density.toFixed(4);
    }
    const viscosity = Number(world.read_component("atmosphere_viscosity", ""));
    if (atmosphereViscosityInput && document.activeElement !== atmosphereViscosityInput && !Number.isNaN(viscosity)) {
      atmosphereViscosityInput.value = viscosity.toFixed(8);
    }
    if (!Number.isNaN(density)) {
      const wind = JSON.parse(world.read_component("atmosphere_wind", "")) as number[];
      windInputs.forEach((input, i) => {
        if (input && document.activeElement !== input) {
          input.value = wind[i].toFixed(3);
        }
      });
    }
    const level = Number(world.read_component("water_level", ""));
    if (waterToggle) waterToggle.checked = !Number.isNaN(level);
    if (waterLevelInput && document.activeElement !== waterLevelInput && !Number.isNaN(level)) {
      waterLevelInput.value = level.toFixed(3);
    }
    const waterDensity = Number(world.read_component("water_density", ""));
    if (waterDensityInput && document.activeElement !== waterDensityInput && !Number.isNaN(waterDensity)) {
      waterDensityInput.value = waterDensity.toFixed(1);
    }
  }
  gravityInput?.addEventListener("change", () => {
    const value = Number(gravityInput.value);
    if (!Number.isFinite(value)) return;
    applyComponent(world, "set_gravity", { gravity: value });
    pushCommandLog(world, { kind: "SetGravity", gravity: value });
  });

  // **重力の向き(残タスク完遂増分、レビュー指摘「見送らず対応すること」への
  // 対応)**。3欄(x,y,z)を1つの`set_gravity_direction`呼び出しへ組み立てる
  // ——他の3欄フォーム(風、collision group/mask)と同じ「変更された欄自身の
  // 値だけをローカル変数へ確定し、他の欄はDOMから読み直さない」規約に従う
  // (`pushFilter`のdocが説明する、Playwrightの逐次`.fill()`とレース
  // する既知のバグパターンを踏まないため)。
  const gravityDirectionInputs = (["x", "y", "z"] as const).map(
    (axis) =>
      document.getElementById(`input-gravity-direction-${axis}`) as HTMLInputElement | null,
  );
  const pendingGravityDirection = gravityDirectionInputs.map((input, i) =>
    Number(input?.value ?? (i === 1 ? -1 : 0)),
  ) as [number, number, number];
  gravityDirectionInputs.forEach((input, i) =>
    input?.addEventListener("change", () => {
      pendingGravityDirection[i] = Number(input.value);
      applyComponent(world, "set_gravity_direction", {
        x: pendingGravityDirection[0],
        y: pendingGravityDirection[1],
        z: pendingGravityDirection[2],
      });
      // **決定論の観点では重力の向きの変更も「入力」**——`SetGravity`と同じ
      // 理由でReplayタブへ記録する(記録し忘れるとリプレイのstate_hashが
      // 一致しなくなる、直下の`SetGravity`のdoc参照)。
      pushCommandLog(world, {
        kind: "SetGravityDirection",
        x: pendingGravityDirection[0],
        y: pendingGravityDirection[1],
        z: pendingGravityDirection[2],
      });
    }),
  );

  dtInput?.addEventListener("change", () => {
    const value = Number(dtInput.value);
    if (!Number.isFinite(value) || value <= 0) return;
    try {
      applyComponent(world, "set_dt", { dt: value });
      pushCommandLog(world, { kind: "SetDt", dt: value });
    } catch (err) {
      reportError(`dt の変更に失敗しました: ${String(err)}`);
    }
  });

  // **環境(大気・水域、残タスク完遂の縦串③増分)**。`sim_fluid::Atmosphere`
  // (密度・動粘性・風)/`StaticWaterRegion`(水位・密度)自体は既に実装済み
  // だったが、UIから設定する手段が無かった。`BuoyancyDrag`結合(Add
  // Couplingフォーム、縦串②)を持つ剛体にのみ効く。
  //
  // **群フィールドを1つずつローカル変数へ確定する**(collision group/mask
  // と同じ理由、`pushFilter`のdoc参照)——変更のたびに他の欄をDOMから
  // 読み直すと、毎フレーム更新(該当なし、ここは即時反映なので実際は無害
  // だが将来の踏襲ミスを避けるため同じ設計に揃える)や、欄を1つずつ順に
  // fillする操作と衝突しうる。
  const atmosphereToggle = document.getElementById(
    "toggle-atmosphere",
  ) as HTMLInputElement | null;
  const atmosphereDensityInput = document.getElementById(
    "input-atmosphere-density",
  ) as HTMLInputElement | null;
  const atmosphereViscosityInput = document.getElementById(
    "input-atmosphere-viscosity",
  ) as HTMLInputElement | null;
  const windInputs = (["x", "y", "z"] as const).map(
    (axis) =>
      document.getElementById(`input-wind-${axis}`) as HTMLInputElement | null,
  );
  const pendingAtmosphere = {
    density: Number(atmosphereDensityInput?.value ?? 1.225),
    viscosity: Number(atmosphereViscosityInput?.value ?? 1.48e-5),
    wind: windInputs.map((input) => Number(input?.value ?? 0)) as [
      number,
      number,
      number,
    ],
  };
  function applyAtmosphere() {
    if (!atmosphereToggle?.checked) return;
    applyComponent(world, "set_atmosphere", {
      density: pendingAtmosphere.density,
      viscosity: pendingAtmosphere.viscosity,
      wind_x: pendingAtmosphere.wind[0],
      wind_y: pendingAtmosphere.wind[1],
      wind_z: pendingAtmosphere.wind[2],
    });
  }
  atmosphereToggle?.addEventListener("change", () => {
    if (atmosphereToggle.checked) applyAtmosphere();
    else applyComponent(world, "clear_atmosphere", {});
  });
  atmosphereDensityInput?.addEventListener("change", () => {
    pendingAtmosphere.density = Number(atmosphereDensityInput.value);
    applyAtmosphere();
  });
  atmosphereViscosityInput?.addEventListener("change", () => {
    pendingAtmosphere.viscosity = Number(atmosphereViscosityInput.value);
    applyAtmosphere();
  });
  windInputs.forEach((input, i) =>
    input?.addEventListener("change", () => {
      pendingAtmosphere.wind[i] = Number(input.value);
      applyAtmosphere();
    }),
  );

  /// 国際標準大気(ISA)対流圏近似(高度11kmまで)の気圧公式から密度を計算する
  /// (設計docs/22-roadmap/03-editor-todo.md「環境と大気の場」——ISA標準大気
  /// (高度依存密度))。$\rho(h)=\rho_0(1-Lh/T_0)^{gM/(RL)-1}$、
  /// $\rho_0$=1.225kg/m³・$T_0$=288.15K・$L$=0.0065K/m・$g$=9.80665m/s²・
  /// $M$=0.0289644kg/mol・$R$=8.3144598J/(mol·K)。物理コア(sim-fluid)には
  /// 触れない——密度という1つの数値をJS側で計算しdensity欄へ書くだけの
  /// フロントエンド機能に留める(物理コアへの変更は縦串③の対象外、
  /// docs/22-roadmap/03-editor-todo.md末尾「物理コアへの変更を再評価する」
  /// 参照)。
  function isaAirDensity(altitudeMeters: number): number {
    const rho0 = 1.225;
    const t0 = 288.15;
    const lapse = 0.0065;
    const g = 9.80665;
    const molarMass = 0.0289644;
    const gasConstant = 8.3144598;
    const exponent = (g * molarMass) / (gasConstant * lapse) - 1;
    const ratio = 1 - (lapse * altitudeMeters) / t0;
    return ratio > 0 ? rho0 * Math.pow(ratio, exponent) : 0;
  }
  document
    .getElementById("btn-apply-isa-density")
    ?.addEventListener("click", () => {
      const altitudeInput = document.getElementById(
        "input-isa-altitude",
      ) as HTMLInputElement | null;
      const altitude = Number(altitudeInput?.value ?? 0);
      if (!Number.isFinite(altitude)) return;
      const density = isaAirDensity(altitude);
      pendingAtmosphere.density = density;
      if (atmosphereDensityInput) {
        atmosphereDensityInput.value = density.toFixed(4);
      }
      applyAtmosphere();
    });

  const waterToggle = document.getElementById(
    "toggle-water",
  ) as HTMLInputElement | null;
  const waterLevelInput = document.getElementById(
    "input-water-level",
  ) as HTMLInputElement | null;
  const waterDensityInput = document.getElementById(
    "input-water-density",
  ) as HTMLInputElement | null;
  const pendingWater = {
    level: Number(waterLevelInput?.value ?? 0),
    density: Number(waterDensityInput?.value ?? 1000),
  };
  function applyWater() {
    if (!waterToggle?.checked) return;
    applyComponent(world, "set_water_region", {
      water_level: pendingWater.level,
      density: pendingWater.density,
    });
  }
  waterToggle?.addEventListener("change", () => {
    if (waterToggle.checked) applyWater();
    else applyComponent(world, "clear_water_region", {});
  });
  waterLevelInput?.addEventListener("change", () => {
    pendingWater.level = Number(waterLevelInput.value);
    applyWater();
  });
  waterDensityInput?.addEventListener("change", () => {
    pendingWater.density = Number(waterDensityInput.value);
    applyWater();
  });

  // **ドメイン作成(残タスク完遂: 結合14種の残り6種を解禁する増分)**。
  // レビュー指摘(「やり遂げて欲しい」「対応できていますか？出来ていなければ
  // 対応して」)への対応——PhaseChangeMorph/SphRigid/GridFluidRigid/
  // BoussinesqBuoyancy/ConvectionLink/PistonGasは、参照する熱ノード・
  // SPH流体・格子流体・気体区画をUIから作る手段が無く追加できなかった
  // (`docs/22-roadmap/03-editor-todo.md`に明記していた既知の欠落)。
  const thermalNodeCountDisplay = document.getElementById(
    "thermal-node-count-display",
  );
  function refreshThermalNodeCountDisplay() {
    if (thermalNodeCountDisplay) {
      const count = world.read_component("thermal_node_count", "");
      thermalNodeCountDisplay.textContent = `(現在 ${count} 個)`;
    }
  }
  refreshThermalNodeCountDisplay();
  document.getElementById("btn-add-thermal-node")?.addEventListener("click", () => {
    const temperature = Number(
      (document.getElementById("input-new-thermal-node-temperature") as HTMLInputElement | null)
        ?.value ?? 293.15,
    );
    const heatCapacity = Number(
      (document.getElementById("input-new-thermal-node-heat-capacity") as HTMLInputElement | null)
        ?.value ?? 100,
    );
    if (!Number.isFinite(temperature) || !Number.isFinite(heatCapacity) || heatCapacity <= 0) {
      reportError("温度・熱容量には正しい数値を入力してください(熱容量は正の値)。");
      return;
    }
    applyComponent(world, "add_thermal_node", { temperature, heat_capacity: heatCapacity });
    refreshThermalNodeCountDisplay();
  });
  document.getElementById("btn-enable-grid-fluid")?.addEventListener("click", () => {
    applyComponent(world, "enable_grid_fluid_2d_domain", {});
  });
  document.getElementById("btn-enable-gas")?.addEventListener("click", () => {
    applyComponent(world, "enable_gas_compartment", {});
  });
  wireAddQuantumDomainForms(() => world);

  /// グリッドスナップ幅 [m](設計 §1.2「グリッド・スナップ(既定 10 cm、変更可)」)。
  /// 0 ならスナップしない。Gizmo ドラッグの位置決めに使う。
  function gridSnapStep(): number {
    const value = Number(gridSnapInput?.value ?? "0.1");
    return Number.isFinite(value) && value > 0 ? value : 0;
  }
  function snapToGrid(value: number): number {
    const step = gridSnapStep();
    return step > 0 ? Math.round(value / step) * step : value;
  }

  // **キーボードショートカット(群2で追加)**。これまで`keydown`リスナは
  // **リポジトリ全体で0件**で、ショートカットが一切無かった。
  // テキスト入力中(input/textarea)は横取りしない——ブックマーク名や
  // 数値入力を打てなくなるため。
  //
  // QA不具合6: `event.ctrlKey || event.metaKey || event.altKey`で即returnして
  // いたため、修飾キー付きショートカット(Ctrl+Z Undo・Ctrl+Y Redo・Delete・
  // Ctrl+D 複製)が構造的に1つも通らなかった。いずれも機能自体はボタンや
  // 右クリックメニューに既に存在するので、対応するボタンの`.click()`を
  // 呼ぶだけで済む(無効化されたボタンへの`.click()`は実ブラウザでは
  // 発火しないため、モードガードは既存のクリックハンドラ側にそのまま乗る)。
  // **D3「Unityパリティ」増分でCtrl+A(全選択)・Escape(複数選択解除)を追加**
  // ——Hierarchyの複数選択(Ctrl/Cmdクリックのトグル・Shiftクリックの範囲選択)
  // と対になる標準操作が無かった監査結果への対応。
  window.addEventListener("keydown", (event) => {
    const target = event.target as HTMLElement | null;
    if (target && /^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName)) return;

    const ctrlOrCmd = event.ctrlKey || event.metaKey;
    if (ctrlOrCmd && !event.altKey) {
      switch (event.key.toLowerCase()) {
        case "z":
          (event.shiftKey ? redoButton : undoButton).click();
          break;
        case "y":
          redoButton.click();
          break;
        case "d":
          if (hasSelectedBody()) {
            hierarchyActionsRef.current?.duplicate(selectedBodyIndex);
          }
          break;
        case "a": {
          // **全選択(D3「Unityパリティ」増分)**。監査で見つかった具体的な
          // 欠落——Shift範囲選択と対になる標準の全選択が無かった。Inspector側
          // (`selectedBodyIndex`)は変えず、Hierarchyの複数選択集合
          // (`hierarchyMultiSelection`、右クリックの複製/削除がまとめて効く
          // 対象)へ現存する(削除済みでない)ボディ全件を入れる。
          const bodyCount = readNumber(world, "body_count");
          hierarchyMultiSelection.clear();
          for (let i = 0; i < bodyCount; i++) {
            if (world.read_component("body_is_removed_at", String(i)) !== "true") {
              hierarchyMultiSelection.add(i);
            }
          }
          highlightHierarchy(selectedBodyIndex);
          break;
        }
        default:
          return; // 他のCtrl/Cmd組み合わせはブラウザ既定の動作へ委ねる。
      }
      event.preventDefault();
      return;
    }
    if (event.ctrlKey || event.metaKey || event.altKey) return;
    switch (event.key.toLowerCase()) {
      case "w":
        setGizmoTool("translate");
        break;
      case "e":
        setGizmoTool("rotate");
        break;
      case "r":
        setGizmoTool("scale");
        break;
      case "q":
        setGizmoTool("none");
        break;
      case "s":
        // スケッチツール(D1)。W/E/R/Q と同じ排他トグル。
        setGizmoTool("sketch");
        break;
      case "enter":
        // スケッチ中のみ意味を持つ(作図中の点列を1枚のプロファイルとして
        // 確定する)。他のツールでは何もせずブラウザ既定へ委ねる。
        if (gizmoTool !== "sketch") return;
        document.getElementById("btn-sketch-confirm")?.click();
        break;
      case "backspace":
        if (gizmoTool !== "sketch") return;
        document.getElementById("btn-sketch-undo-point")?.click();
        break;
      case "f":
        // 選択中のボディへカメラを寄せる(Unityの F と同じ)。
        focusCameraOnSelection();
        break;
      case "delete":
        // QA不具合6: 選択中のボディを削除する(Hierarchy右クリックの「削除」と同じ経路)。
        if (hasSelectedBody()) {
          hierarchyActionsRef.current?.remove(selectedBodyIndex);
        }
        break;
      case " ":
        // QA不具合6: 再生/一時停止(Playモードでのみ意味を持つ、`playButton`の
        // 既存ガード`mode !== "play"`にそのまま乗る)。
        playButton.click();
        break;
      case "x":
        // QA不具合7: `title`とREADMEは実装済みと書いていたが、keydownの
        // switchに`x`のcaseが無く実際には効かなかった(ボタンクリックのみ有効)。
        setGizmoSpace(gizmoSpace === "world" ? "local" : "world");
        break;
      case "escape":
        // **複数選択の解除(D3「Unityパリティ」増分)**。Ctrl+A(全選択)と
        // 対になる標準操作。Inspectorに出ている1件(`selectedBodyIndex`)は
        // 変えず、そこだけの単一選択へ戻す。
        hierarchyMultiSelection.clear();
        if (hasSelectedBody()) hierarchyMultiSelection.add(selectedBodyIndex);
        highlightHierarchy(selectedBodyIndex);
        break;
      default:
        return;
    }
    event.preventDefault();
  });

  /// 選択中ボディを画面中央に収める(`F`キー)。OrbitControlsの注視点を
  /// 動かし、現在の視線方向を保ったまま一定距離まで寄る。
  function focusCameraOnSelection() {
    if (!hasSelectedBody()) return;
    const p = world.body_position_at_f32(selectedBodyIndex);
    const target = new THREE.Vector3(p[0], p[1], p[2]);
    const direction = camera.position.clone().sub(orbit.target).normalize();
    orbit.target.copy(target);
    camera.position.copy(target).addScaledVector(direction, 6);
    orbit.update();
  }

  function resize() {
    const w = host.clientWidth;
    const h = host.clientHeight;
    camera.aspect = w / Math.max(h, 1);
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  window.addEventListener("resize", resize);
  // **器の大きさが変わったら追従する**。`window` の resize だけを見ていた頃は、
  // スプリッターで Scene View の幅を変えても・かんたんモードと統合エディタを
  // 切り替えても、three.js のキャンバスが前の寸法のまま引き伸ばされて表示が
  // 歪んでいた(ウィンドウを 1px 動かすと直る、という分かりにくい症状だった)。
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(() => resize()).observe(host);
  }
  resize();

  scene.add(new THREE.AmbientLight(0xffffff, 0.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(5, 10, 5);
  scene.add(sun);

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(
      BOX_HALF_EXTENT * 2,
      BOX_HALF_EXTENT * 2,
      BOX_HALF_EXTENT * 2,
    ),
    new THREE.MeshStandardMaterial({ color: 0xffa500 }),
  );
  scene.add(box);

  // 床(静的平面、`WasmWorld::new`が`BODY_INDEX_GROUND`として構築するコンクリート面)。
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(20, 20),
    new THREE.MeshStandardMaterial({ color: 0x555555 }),
  );
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);

  // 全ボディのThree.jsメッシュ(bodyIndexで引ける、`render()`が毎フレーム
  // 位置/姿勢を反映させるために使う)。**残タスク完遂のシーンギャラリー増分**で
  // `box`/`ground`(既定シーン固有の決め打ち)とスポーンパレットで追加した
  // メッシュを統合した——シーンギャラリー(`loadScene`)が任意のシーンJSONを
  // 読み込むと、既定シーンには無いボディ構成(例: D11振り子は球1個のみで
  // index 1に「箱」は存在しない)になり得るため、「index 1は常に箱」という
  // 決め打ちを解く必要があった。
  const bodyMeshes = new Map<number, THREE.Mesh>();
  bodyMeshes.set(BODY_INDEX_GROUND, ground);
  bodyMeshes.set(BODY_INDEX_BOX, box);

  const grid = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
  scene.add(grid);

  // Scene View オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「速度ベクトル」、
  // 切替可)の最小デモ: 選択中ボディの速度ベクトルを矢印で表示する。縮約実装の
  // 理由: 接触点・力・拘束・流体場・フレーム軸のオーバーレイは対象外、速度のみ。
  const VELOCITY_OVERLAY_SCALE = 0.3; // 矢印長 = 速さ[m/s] * この係数[m]。
  const velocityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(),
    1,
    0xffee00,
  );
  velocityArrow.visible = false;
  scene.add(velocityArrow);
  const velocityOverlayToggle = document.getElementById(
    "toggle-velocity-overlay",
  ) as HTMLInputElement;
  const velocityDirection = new THREE.Vector3();

  // 接触点オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「接触点」、切替可)。
  // `World::contact_points`(既存の`MechanicsSolver::last_manifolds`をそのまま
  // 使う)が返す直近stepの接触点ワールド座標に、小さな球マーカーを重ねて表示する。
  // 縮約実装の理由: マーカーの固定プール(`CONTACT_MARKER_POOL_SIZE`個)を使い回す
  // だけで、法線・貫入量の可視化(矢印やインパルス強度の色分け等)は対象外。
  const CONTACT_MARKER_POOL_SIZE = 8;
  const CONTACT_MARKER_RADIUS = 0.06;
  const contactOverlayToggle = document.getElementById(
    "toggle-contact-overlay",
  ) as HTMLInputElement;
  const contactMarkerGeometry = new THREE.SphereGeometry(
    CONTACT_MARKER_RADIUS,
    10,
    8,
  );
  const contactMarkerMaterial = new THREE.MeshBasicMaterial({
    color: 0xff2222,
  });
  const contactMarkers: THREE.Mesh[] = [];
  for (let i = 0; i < CONTACT_MARKER_POOL_SIZE; i++) {
    const marker = new THREE.Mesh(contactMarkerGeometry, contactMarkerMaterial);
    marker.visible = false;
    scene.add(marker);
    contactMarkers.push(marker);
  }

  // 力オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「力」、切替可)。
  // 縮約実装の理由: 一般の力の可視化(接触力・拘束反力等の継続的な蓄積量)には
  // `World`側の対応するクエリが無いため対象外。Nudgeボタン(`Command::
  // ApplyForce`、1step分だけ効く既知の力)をクリックした瞬間にだけ、その
  // 力ベクトルを短時間(`FORCE_OVERLAY_DURATION_MS`)矢印表示する——実際に
  // Commandとして適用される値をそのまま可視化するため、UIの見た目と実際の
  // 物理入力が一致する。
  const FORCE_OVERLAY_DURATION_MS = 500;
  const FORCE_OVERLAY_SCALE = 1.0 / 300_000.0; // 矢印長 = 力[N] * この係数[m]。
  const forceOverlayToggle = document.getElementById(
    "toggle-force-overlay",
  ) as HTMLInputElement;
  const forceArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(),
    1,
    0xff8800,
  );
  forceArrow.visible = false;
  scene.add(forceArrow);
  let forceOverlayHideAtMs = 0;

  // 拘束オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「拘束」)。振り子
  // スポーン(`spawn_pendulum`)が追加したDistanceJointの2つのアンカー点
  // (固定ピボット・可動体側)を結ぶ線を毎フレーム描画する(`render()`内、
  // `constraintLines`ループ参照)。拘束を持たないボディ(球/箱スポーン・
  // 床・箱)は対象外。
  const constraintOverlayToggle = document.getElementById(
    "toggle-constraint-overlay",
  ) as HTMLInputElement;

  // フレーム軸オーバーレイ + 階層ドリルインUI(設計docs/23-frontend/01-editor.md
  // §1.3「フレームサブモード」)。ROOTの子としてz軸まわりに自転するフレームを
  // 1つ既定で追加し(`World::add_frame`+`sim_core::FrameTree::step`が毎step
  // 自動的に回転を進める、`sim-world`側の増分参照)、以後Hierarchyの「Frames」
  // サブツリーで選択したフレームの子として「+ フレーム」ボタンからネストした
  // フレームを追加できる(`add_child_frame`——`add_rotating_frame`の一般化、
  // 親をROOT固定ではなく任意に選べる)。各フレームは`frame_world_position_f32`/
  // `frame_world_rotation_f32`(`FrameTree::transform_to_root`、階層を遡って
  // 合成したワールド姿勢)で毎フレーム更新する専用の`THREE.AxesHelper`を持つ。
  const frameOverlayToggle = document.getElementById(
    "toggle-frame-overlay",
  ) as HTMLInputElement;
  const FRAME_AXIS_ANGULAR_VELOCITY = 1.0; // rad/s(任意値、回転が目視できる速さ)
  const FRAME_CHILD_OFFSET = 1.5; // 新規子フレームの親からのローカルオフセット(x軸方向)
  const frameAxesHelpers = new Map<number, THREE.AxesHelper>();
  let selectedFrameIndex = 0; // 0=ROOT(既定の親)。Hierarchyでフレームを選ぶと更新される。

  function createFrameAxesHelper(frameIndex: number) {
    const helper = new THREE.AxesHelper(2.0);
    scene.add(helper);
    frameAxesHelpers.set(frameIndex, helper);
  }

  const initialFrameIndex = applyComponent(world, "add_child_frame", {
    parent_index: 0,
    origin_offset_x: 0,
    origin_offset_y: 3,
    origin_offset_z: 0,
    angular_velocity_z: FRAME_AXIS_ANGULAR_VELOCITY,
  }).index as number;
  createFrameAxesHelper(initialFrameIndex);
  selectedFrameIndex = initialFrameIndex;

  // 流体場オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「流体場」の土台)。
  // 「+ 流体」ボタンでSPH流体塊(`world.spawn_fluid_block`)をスポーンすると、
  // 粒子位置をTHREE.Pointsで毎フレーム反映する(粒子数は固定なので、スポーン時に
  // 一度だけBufferAttributeを確保しrender()内で内容だけ更新する)。
  const fluidGeometry = new THREE.BufferGeometry();
  const fluidMaterial = new THREE.PointsMaterial({
    color: 0x3399ff,
    size: 0.08,
  });
  const fluidPoints = new THREE.Points(fluidGeometry, fluidMaterial);
  fluidPoints.visible = false;
  scene.add(fluidPoints);
  let fluidPositionAttribute: THREE.BufferAttribute | null = null;

  // **格子流体の速度場オーバーレイ(増分L)**。セルごとに`ArrowHelper`を作ると
  // 数百オブジェクトになるので、**1本の`LineSegments`**で全ベクトルを描く
  // (頂点2つで1本の線分。矢じりは付けない縮約——密なベクトル場では矢じりが
  // 潰れて視認性がむしろ落ちるため、長さと向きで表現する)。
  const GRID_FLUID_OVERLAY_STRIDE = 2; // 1つ飛ばし。全セルだと矢印が密すぎる。
  const GRID_FLUID_OVERLAY_SCALE = 0.05; // 速度[m/s] → 線分長[m] の表示倍率。
  const GRID_FLUID_MAX_CELLS = 4096;
  const gridFluidGeometry = new THREE.BufferGeometry();
  const gridFluidVertices = new Float32Array(GRID_FLUID_MAX_CELLS * 2 * 3);
  const gridFluidPositionAttribute = new THREE.BufferAttribute(
    gridFluidVertices,
    3,
  );
  gridFluidGeometry.setAttribute("position", gridFluidPositionAttribute);
  const gridFluidLines = new THREE.LineSegments(
    gridFluidGeometry,
    new THREE.LineBasicMaterial({ color: 0x66ccff }),
  );
  gridFluidLines.visible = false;
  scene.add(gridFluidLines);

  // **ソフトボディの描画(群3)**。**D13(ロープと旗)は Scene View に何も
  // 描かれていなかった**——ソフトボディは`RigidBodySet`の剛体ではないので
  // `bodyMeshes`の同期対象外で、Probe Graphs でしか観測できなかった。
  // 粒子を`Points`、距離拘束を`LineSegments`で描く。
  const SOFT_BODY_MAX_PARTICLES = 4096;
  const softBodyGeometry = new THREE.BufferGeometry();
  const softBodyVertices = new Float32Array(SOFT_BODY_MAX_PARTICLES * 3);
  const softBodyPositionAttribute = new THREE.BufferAttribute(softBodyVertices, 3);
  softBodyGeometry.setAttribute("position", softBodyPositionAttribute);
  const softBodyPoints = new THREE.Points(
    softBodyGeometry,
    new THREE.PointsMaterial({ color: 0xffcc66, size: 0.06 }),
  );
  softBodyPoints.visible = false;
  scene.add(softBodyPoints);

  const softBodyLinkGeometry = new THREE.BufferGeometry();
  const softBodyLinkVertices = new Float32Array(SOFT_BODY_MAX_PARTICLES * 2 * 3);
  const softBodyLinkAttribute = new THREE.BufferAttribute(softBodyLinkVertices, 3);
  softBodyLinkGeometry.setAttribute("position", softBodyLinkAttribute);
  const softBodyLines = new THREE.LineSegments(
    softBodyLinkGeometry,
    new THREE.LineBasicMaterial({ color: 0xcc9944 }),
  );
  softBodyLines.visible = false;
  scene.add(softBodyLines);
  /// 拘束ペアはシーン読み込み時にしか変わらないのでキャッシュする(毎フレーム
  /// wasm から取り直すと数千要素のコピーが走る)。
  let softBodyConstraintPairs: Uint32Array = new Uint32Array(0);

  function updateSoftBodyOverlay(currentWorld: WasmWorld) {
    const positions = currentWorld.soft_body_positions_f32();
    const count = Math.min(positions.length / 3, SOFT_BODY_MAX_PARTICLES);
    if (count === 0) {
      softBodyPoints.visible = false;
      softBodyLines.visible = false;
      return;
    }
    softBodyVertices.set(positions.subarray(0, count * 3));
    softBodyGeometry.setDrawRange(0, count);
    softBodyPositionAttribute.needsUpdate = true;
    softBodyPoints.visible = true;

    const pairs = softBodyConstraintPairs;
    const links = Math.min(pairs.length / 2, SOFT_BODY_MAX_PARTICLES);
    for (let k = 0; k < links; k += 1) {
      const i = pairs[k * 2];
      const j = pairs[k * 2 + 1];
      softBodyLinkVertices[k * 6] = positions[i * 3];
      softBodyLinkVertices[k * 6 + 1] = positions[i * 3 + 1];
      softBodyLinkVertices[k * 6 + 2] = positions[i * 3 + 2];
      softBodyLinkVertices[k * 6 + 3] = positions[j * 3];
      softBodyLinkVertices[k * 6 + 4] = positions[j * 3 + 1];
      softBodyLinkVertices[k * 6 + 5] = positions[j * 3 + 2];
    }
    softBodyLinkGeometry.setDrawRange(0, links * 2);
    softBodyLinkAttribute.needsUpdate = true;
    softBodyLines.visible = links > 0;
  }

  // **天体の描画(群3)**。**D34/D35/D36 も Scene View には何も描かれて
  // いなかった**——天体は`RigidBodySet`とは別の質点集合で、剛体メッシュの
  // 同期対象外だった。
  //
  // **座標を「そのまま」描くと何も見えない**——太陽系のスケールは 10¹¹ m
  // オーダーで、カメラは数メートルの世界にいる。最も遠い天体が画面に収まるよう
  // **毎フレーム正規化して描く**(絶対距離はProbe Graphsが出す)。
  const ASTRO_VIEW_RADIUS = 6.0; // 最遠天体をこの半径に収める。
  const astroMeshes: THREE.Mesh[] = [];
  const astroGroup = new THREE.Group();
  astroGroup.visible = false;
  scene.add(astroGroup);

  function updateAstroOverlay(currentWorld: WasmWorld) {
    // `astro_positions_f32`はWasmメモリを直接指す一時的なビューを返す(B16、
    // `HotPathViewBuffers`のdoc参照)。この下で`astro_masses_f64`という別の
    // Wasm呼び出しを挟むため、そのビューが無効化される前に自前のコピーへ
    // 読み切っておく(`positions`はこの関数の残り全体で使い続ける)。
    const positions = Float32Array.from(currentWorld.astro_positions_f32());
    const count = positions.length / 3;
    if (count === 0) {
      astroGroup.visible = false;
      return;
    }
    const masses = currentWorld.astro_masses_f64();
    // 最遠天体までの距離でスケールを決める(0 なら 1 とみなす)。
    let maxR = 0;
    for (let i = 0; i < count; i += 1) {
      const r = Math.hypot(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
      if (r > maxR) maxR = r;
    }
    const scale = maxR > 0 ? ASTRO_VIEW_RADIUS / maxR : 1;
    const maxMass = masses.length ? Math.max(...masses) : 1;
    while (astroMeshes.length < count) {
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(1, 12, 8),
        new THREE.MeshStandardMaterial({ color: 0xffdd88, emissive: 0x332200 }),
      );
      astroMeshes.push(mesh);
      astroGroup.add(mesh);
    }
    for (let i = 0; i < astroMeshes.length; i += 1) {
      const mesh = astroMeshes[i];
      if (i >= count) {
        mesh.visible = false;
        continue;
      }
      mesh.visible = true;
      mesh.position.set(
        positions[i * 3] * scale,
        positions[i * 3 + 1] * scale,
        positions[i * 3 + 2] * scale,
      );
      // 半径は質量の立方根に比例させる(密度一定の球と同じ関係)。
      // 質量が桁違いなので、太陽が画面を埋めないよう上限を掛ける。
      const ratio = maxMass > 0 ? (masses[i] ?? 0) / maxMass : 0;
      mesh.scale.setScalar(0.12 + 0.5 * Math.cbrt(Math.max(ratio, 0)));
    }
    astroGroup.visible = true;
  }

  // **気体分子・ブラウン粒子の描画(群3)**。どちらも点群として描く。
  // 分子気体は 10⁻⁷ m の箱、ブラウン粒子は 10⁻⁶ m オーダーの変位なので、
  // 天体と同じ理由で**正規化して描く**。
  const PARTICLE_VIEW_RADIUS = 4.0;
  const STAT_MAX_PARTICLES = 4096;
  function makeParticleCloud(color: number, size: number) {
    const geometry = new THREE.BufferGeometry();
    const vertices = new Float32Array(STAT_MAX_PARTICLES * 3);
    const attribute = new THREE.BufferAttribute(vertices, 3);
    geometry.setAttribute("position", attribute);
    const points = new THREE.Points(geometry, new THREE.PointsMaterial({ color, size }));
    points.visible = false;
    scene.add(points);
    return { geometry, vertices, attribute, points };
  }
  const gasCloud = makeParticleCloud(0x88ddff, 0.05);
  const brownianCloud = makeParticleCloud(0xff88cc, 0.06);

  function updateParticleCloud(
    cloud: ReturnType<typeof makeParticleCloud>,
    positions: Float32Array,
    center: [number, number, number],
  ) {
    const count = Math.min(positions.length / 3, STAT_MAX_PARTICLES);
    if (count === 0) {
      cloud.points.visible = false;
      return;
    }
    // 最遠粒子を PARTICLE_VIEW_RADIUS に収める(絶対スケールは Probe Graphs が出す)。
    let maxR = 0;
    for (let i = 0; i < count; i += 1) {
      const r = Math.hypot(
        positions[i * 3] - center[0],
        positions[i * 3 + 1] - center[1],
        positions[i * 3 + 2] - center[2],
      );
      if (r > maxR) maxR = r;
    }
    const scale = maxR > 0 ? PARTICLE_VIEW_RADIUS / maxR : 1;
    for (let i = 0; i < count; i += 1) {
      cloud.vertices[i * 3] = (positions[i * 3] - center[0]) * scale;
      cloud.vertices[i * 3 + 1] = (positions[i * 3 + 1] - center[1]) * scale + 2.0;
      cloud.vertices[i * 3 + 2] = (positions[i * 3 + 2] - center[2]) * scale;
    }
    cloud.geometry.setDrawRange(0, count);
    cloud.attribute.needsUpdate = true;
    cloud.points.visible = true;
  }

  // **場のパネル(群3)**。チェックリストが D27–D33 を閉じる際に挙げた
  // 「解禁には**専用の可視化パネル**が要る——波動関数の |ψ|² 分布・スピン格子・
  // 速度ヒストグラム等は Scene View の剛体描画では表現できない」という条件の実体。
  //
  // **3D の Scene View ではなく 2D canvas に描く**のが要点。|ψ|²・スピン格子・
  // Ez 場はいずれも「格子上のスカラー場」であり、3D 空間に浮かべるより
  // 平面に色で塗るほうが読める(実際、物理の教科書もそう描く)。
  // Probe Graphs の隣に置き、対象ドメインが無効なときは畳んで場所を取らない。
  const fieldPanel = document.getElementById("field-panel")!;
  const fieldCanvas = document.getElementById("field-canvas") as HTMLCanvasElement;
  const fieldTitle = document.getElementById("field-title")!;
  const fieldContext = fieldCanvas.getContext("2d");
  /// 気体の箱の中心(粒子描画の原点合わせに使う)。シーン読み込み時に更新する。
  let gasBoxCenter: [number, number, number] = [0, 0, 0];

  /// 発散カラーマップ(青←0→赤)。Ez のように符号を持つ場に使う。
  function divergingColor(t: number): [number, number, number] {
    const x = Math.max(-1, Math.min(1, t));
    if (x >= 0) return [255, Math.round(255 * (1 - x)), Math.round(255 * (1 - x))];
    return [Math.round(255 * (1 + x)), Math.round(255 * (1 + x)), 255];
  }

  /// 単調カラーマップ(黒→黄→白)。|ψ|² のように非負の場に使う。
  function sequentialColor(t: number): [number, number, number] {
    const x = Math.max(0, Math.min(1, t));
    return [Math.round(255 * Math.min(1, x * 2)), Math.round(255 * x), Math.round(255 * x * x)];
  }

  /// `nx × ny` のスカラー場を canvas いっぱいに描く。
  function drawScalarField(
    values: Float32Array,
    nx: number,
    ny: number,
    color: (t: number) => [number, number, number],
    normalize: "signed" | "positive",
  ) {
    if (!fieldContext || nx === 0 || ny === 0) return;
    fieldCanvas.width = nx;
    fieldCanvas.height = ny;
    const image = fieldContext.createImageData(nx, ny);
    let scale = 0;
    for (let i = 0; i < values.length; i += 1) scale = Math.max(scale, Math.abs(values[i]));
    if (scale === 0) scale = 1;
    for (let j = 0; j < ny; j += 1) {
      for (let i = 0; i < nx; i += 1) {
        const v = values[j * nx + i];
        const t = normalize === "signed" ? v / scale : Math.abs(v) / scale;
        const [r, g, b] = color(t);
        // canvas の y は下向き。物理の格子は上向きなので反転して描く。
        const px = ((ny - 1 - j) * nx + i) * 4;
        image.data[px] = r;
        image.data[px + 1] = g;
        image.data[px + 2] = b;
        image.data[px + 3] = 255;
      }
    }
    fieldContext.putImageData(image, 0, 0);
  }

  /// 1D の分布(|ψ|² とポテンシャル)を折れ線で描く。
  function drawQuantum1d(density: Float32Array, potential: Float32Array) {
    if (!fieldContext) return;
    const w = 512;
    const h = 160;
    fieldCanvas.width = w;
    fieldCanvas.height = h;
    fieldContext.fillStyle = "#111";
    fieldContext.fillRect(0, 0, w, h);
    const n = density.length;
    if (n === 0) return;
    const plot = (
      values: Float32Array,
      color: string,
      scaleOverride?: number,
    ) => {
      let max = scaleOverride ?? 0;
      if (scaleOverride === undefined) {
        for (let i = 0; i < values.length; i += 1) max = Math.max(max, values[i]);
      }
      if (max <= 0) return;
      fieldContext.strokeStyle = color;
      fieldContext.lineWidth = 1.5;
      fieldContext.beginPath();
      for (let i = 0; i < values.length; i += 1) {
        const x = (i / (values.length - 1)) * w;
        const y = h - (values[i] / max) * (h - 8) - 4;
        if (i === 0) fieldContext.moveTo(x, y);
        else fieldContext.lineTo(x, y);
      }
      fieldContext.stroke();
    };
    // ポテンシャルを先に(背景として)、確率密度を手前に。
    plot(potential, "#666");
    plot(density, "#6cf");
  }

  /// 速さのヒストグラムを棒グラフで描く(D30「ヒストグラム」)。
  function drawHistogram(counts: Float32Array) {
    if (!fieldContext) return;
    const w = 512;
    const h = 160;
    fieldCanvas.width = w;
    fieldCanvas.height = h;
    fieldContext.fillStyle = "#111";
    fieldContext.fillRect(0, 0, w, h);
    let max = 0;
    for (let i = 0; i < counts.length; i += 1) max = Math.max(max, counts[i]);
    if (max <= 0) return;
    const barWidth = w / counts.length;
    fieldContext.fillStyle = "#8df";
    for (let i = 0; i < counts.length; i += 1) {
      const barHeight = (counts[i] / max) * (h - 8);
      fieldContext.fillRect(i * barWidth, h - barHeight, Math.max(barWidth - 1, 1), barHeight);
    }
  }

  function updateFieldPanel(currentWorld: WasmWorld) {
    if (!fieldContext) return;
    // **優先順位を固定する**(決定論的な表示)。同時に複数ドメインが載っている
    // シーンでは最初に見つかったものを描く。
    const quantum2dSize = currentWorld.quantum_2d_size();
    if (quantum2dSize.length === 2) {
      fieldTitle.textContent = `量子 2D |ψ|² (${quantum2dSize[0]}×${quantum2dSize[1]})`;
      // ポテンシャル壁を暗く重ねたいが、まずは密度をそのまま出す
      // (壁は密度が 0 のまま残るので位置は読み取れる)。
      drawScalarField(
        currentWorld.quantum_2d_density_f32(),
        quantum2dSize[0],
        quantum2dSize[1],
        sequentialColor,
        "positive",
      );
      fieldPanel.hidden = false;
      return;
    }
    // `quantum_1d_density_f32`はWasmメモリを直接指す一時的なビューを返す
    // (B16、`HotPathViewBuffers`のdoc参照)。すぐ下で`quantum_1d_potential_f32`
    // という別のWasm呼び出しを挟むため、そのビューが無効化される前に自前の
    // コピーへ読み切っておく。
    const density = Float32Array.from(currentWorld.quantum_1d_density_f32());
    if (density.length > 0) {
      fieldTitle.textContent = `量子 1D |ψ|² と V(x)(格子 ${density.length} 点)`;
      drawQuantum1d(density, currentWorld.quantum_1d_potential_f32());
      fieldPanel.hidden = false;
      return;
    }
    const fdtdSize = currentWorld.fdtd_size();
    if (fdtdSize.length === 2) {
      fieldTitle.textContent = `FDTD Ez (${fdtdSize[0]}×${fdtdSize[1]}、青=負 赤=正)`;
      drawScalarField(
        currentWorld.fdtd_ez_f32(),
        fdtdSize[0],
        fdtdSize[1],
        divergingColor,
        "signed",
      );
      fieldPanel.hidden = false;
      return;
    }
    const isingSize = currentWorld.ising_size();
    if (isingSize > 0) {
      const spins = currentWorld.ising_spins_u8();
      // ±1 を ±1 の f32 に直して発散カラーマップへ渡す(上向き=赤・下向き=青)。
      const values = new Float32Array(spins.length);
      for (let i = 0; i < spins.length; i += 1) values[i] = spins[i] ? 1 : -1;
      fieldTitle.textContent = `イジング スピン格子 (${isingSize}×${isingSize})`;
      drawScalarField(values, isingSize, isingSize, divergingColor, "signed");
      fieldPanel.hidden = false;
      return;
    }
    const maxSpeed = currentWorld.kinetic_gas_max_speed();
    if (maxSpeed > 0) {
      fieldTitle.textContent = `気体分子の速さ分布(最大 ${maxSpeed.toFixed(0)} m/s)`;
      drawHistogram(currentWorld.kinetic_gas_speed_histogram_f32(32, maxSpeed));
      fieldPanel.hidden = false;
      return;
    }
    const rod = currentWorld.conduction_rod_temperatures_f32();
    if (rod.length > 0) {
      fieldTitle.textContent = `熱伝導棒の温度分布(${rod.length} 格子点)`;
      drawScalarField(rod, rod.length, 1, sequentialColor, "positive");
      fieldPanel.hidden = false;
      return;
    }
    fieldPanel.hidden = true;
  }

  /// **シーンの中身にカメラを合わせる(群3)**。
  ///
  /// **なぜ必要だったか**: ギャラリーシーンを読み込んでもカメラは起動時の
  /// 固定位置(既定シーンの箱に合わせた画角)のままだった。剛体のシーンは
  /// たまたま同じスケールなので問題にならなかったが、群3で描けるようにした
  /// ソフトボディ(D13: 1 m のロープが原点付近)や天体(正規化して半径6に描く)
  /// は画角から外れる/小さすぎて、**描画は正しく動いているのに「何も出ていない」
  /// ように見えた**(実際にこれで調査に時間を使った)。
  ///
  /// 描画対象すべてのバウンディングボックスを取り、その中心を注視点に、
  /// 対角長からカメラ距離を決める(Unity の F キーと同じ考え方)。
  /// 箱を作る部分は `contentBoundingBox` として切り出してある——
  /// かんたんモードの追従カメラ(`updateGuidedFollowCamera`)が同じ
  /// 「観察対象」の定義を使うため。
  ///
  /// 「観察対象」のバウンディングボックス。静的な床・壁は含めない(QA不具合2)。
  function contentBoundingBox(): THREE.Box3 | null {
    const box = new THREE.Box3();
    let hasContent = false;
    const expand = (object: THREE.Object3D) => {
      if (!object.visible) return;
      const objectBox = new THREE.Box3().setFromObject(object);
      if (objectBox.isEmpty()) return;
      box.union(objectBox);
      hasContent = true;
    };
    // QA不具合2: 静的な床(D4等では20m四方)を含めると、注視距離が対象では
    // なく床の大きさで決まってしまい、対象が画面の数%しか占めない、あるいは
    // (前シーンから引き継いだ視線方向によっては)カメラが床の下に潜り込んで
    // 何も見えなくなっていた。動的ボディだけを対象にする——静的な床・壁は
    // 「シーンの一部」ではあっても「観察対象」ではないため。
    for (const [bodyIndex, mesh] of bodyMeshes) {
      if ((world.read_component("body_is_static_at", String(bodyIndex)) === "true")) continue;
      expand(mesh);
    }
    expand(softBodyPoints);
    expand(astroGroup);
    expand(gasCloud.points);
    expand(brownianCloud.points);
    expand(fluidPoints);
    return hasContent ? box : null;
  }

  function frameCameraOnContent() {
    const box = contentBoundingBox();
    if (!box) return;
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(box.getSize(new THREE.Vector3()).length() * 0.5, 0.5);
    orbit.target.copy(center);
    // 現在の視線方向を保ったまま距離だけ合わせる(向きの好みを壊さない)。
    const direction = camera.position.clone().sub(center);
    if (direction.lengthSq() < 1e-9) direction.set(1, 0.7, 1.2);
    const normalizedDirection = direction.normalize();
    // QA不具合2続き: 前のシーンから引き継いだ視線方向の仰角が低い(または
    // 水平面より下を向いている)と、対象が地面近くにある場合(D11/D12/D13等)
    // カメラが計算上そのまま床の下へ潜り込んでしまう。仰角の最低ラインを
    // 設けて、床の下からは絶対に見上げない(=床の中に埋まらない)ようにする
    // ——向きの「好み」より「対象が見えること」を優先する。
    const MIN_ELEVATION = 0.25; // sin(約14.5°)。低すぎると地面すれすれで違和感が出るため床下だけを防ぐ最小限の値。
    if (normalizedDirection.y < MIN_ELEVATION) {
      normalizedDirection.y = MIN_ELEVATION;
      normalizedDirection.normalize();
    }
    camera.position.copy(center).add(normalizedDirection.multiplyScalar(radius * 2.6));
    // 仰角クランプだけでは(対象が地面近くにある・半径が小さい等の組み合わせで)
    // なお僅かに地面下へ出るケースが残ったため、最終防衛線として絶対高さも
    // 下限クランプする(このプロジェクトの地面は常にy=0の平面、モジュールdoc
    // 「床の下に潜り込む」参照)。
    camera.position.y = Math.max(camera.position.y, 0.3);
    orbit.update();
  }

  /**
   * **追従カメラ**(かんたんモード)。
   *
   * 読み込み時の 1 回だけ画角を合わせる従来のやり方は、**動くものを見る**という
   * 目的に対して成立していなかった——高さ 20m から落ちる球は、読み込み直後の
   * 球(半径 0.3m)にぴったり寄った画角から 1 秒で外へ出ていき、初めて使う人の
   * 画面には**空のグリッドだけが残る**(実際にスクリーンショットで確認した)。
   * 統合エディタなら自分でカメラを回して探せばよいが、それは「中を知っている
   * 人の操作」であって、かんたんモードが引き受けるべき仕事ではない。
   *
   * そこで毎フレーム、観察対象と原点(床・太陽など「基準」がある場所)を
   * 含む箱を作り、そこへゆっくり寄せる。ユーザーが自分でカメラを操作したら
   * 追従は止める(操作を奪わない)——「カメラを戻す」で再開できる。
   */
  let guidedFollowCamera = false;
  let guidedCameraSnap = false;
  const guidedFollowTarget = new THREE.Vector3();
  const guidedFollowDirection = new THREE.Vector3();
  function updateGuidedFollowCamera() {
    const box = contentBoundingBox();
    if (!box) return;
    // 原点を必ず含める。落下は「床(y=0)まで」、公転は「中心の星まで」が
    // 見えて初めて現象として読めるため。
    box.expandByPoint(new THREE.Vector3(0, 0, 0));
    box.getCenter(guidedFollowTarget);
    const radius = Math.max(
      box.getSize(new THREE.Vector3()).length() * 0.5,
      0.5,
    );
    // 対象の 3.6 倍まで引く。かんたんモードでは「対象が大きく映ること」より
    // **まわりが見えること**が優先——坂を滑る箱は、坂が画面に入っていなければ
    // 何が起きているのか分からない(倍率ではなく比で決めるのは、シーンの寸法が
    // 1e-7 m の分子から 1e11 m の公転まで振れるため)。
    const desired = radius * 3.6;
    const ease = guidedCameraSnap ? 1 : 0.08;
    guidedCameraSnap = false;
    guidedFollowDirection.copy(camera.position).sub(orbit.target);
    if (guidedFollowDirection.lengthSq() < 1e-12) {
      guidedFollowDirection.set(1, 0.7, 1.2);
    }
    const distance = guidedFollowDirection.length();
    guidedFollowDirection.normalize();
    if (guidedFollowDirection.y < 0.3) {
      guidedFollowDirection.y = 0.3;
      guidedFollowDirection.normalize();
    }
    orbit.target.lerp(guidedFollowTarget, ease);
    const nextDistance = distance + (desired - distance) * ease;
    camera.position
      .copy(orbit.target)
      .addScaledVector(guidedFollowDirection, nextDistance);
    camera.position.y = Math.max(camera.position.y, 0.3);
  }
  // 自分でカメラを動かしたら追従をやめる(操作を横取りしない)。
  orbit.addEventListener("start", () => {
    guidedFollowCamera = false;
  });

  function updateGridFluidOverlay(currentWorld: WasmWorld) {
    const enabled = (
      document.getElementById(
        "toggle-grid-fluid-overlay",
      ) as HTMLInputElement | null
    )?.checked;
    const field = enabled
      ? currentWorld.grid_fluid_velocity_field_f32(GRID_FLUID_OVERLAY_STRIDE)
      : new Float32Array(0);
    const cells = Math.min(field.length / 4, GRID_FLUID_MAX_CELLS);
    if (cells === 0) {
      gridFluidLines.visible = false;
      return;
    }
    for (let c = 0; c < cells; c += 1) {
      const [x, y, u, v] = [
        field[c * 4],
        field[c * 4 + 1],
        field[c * 4 + 2],
        field[c * 4 + 3],
      ];
      const base = c * 6;
      // 始点(セル中心)。格子流体は2Dなのでz=0平面に描く。
      gridFluidVertices[base] = x;
      gridFluidVertices[base + 1] = y;
      gridFluidVertices[base + 2] = 0;
      // 終点(速度ベクトルぶんだけ伸ばす)。
      gridFluidVertices[base + 3] = x + u * GRID_FLUID_OVERLAY_SCALE;
      gridFluidVertices[base + 4] = y + v * GRID_FLUID_OVERLAY_SCALE;
      gridFluidVertices[base + 5] = 0;
    }
    gridFluidGeometry.setDrawRange(0, cells * 2);
    gridFluidPositionAttribute.needsUpdate = true;
    gridFluidLines.visible = true;
  }

  function showForceOverlay(origin: THREE.Vector3, force: THREE.Vector3) {
    const magnitude = force.length();
    if (magnitude < 1e-6) return;
    forceArrow.position.copy(origin);
    forceArrow.setDirection(force.clone().divideScalar(magnitude));
    const length = magnitude * FORCE_OVERLAY_SCALE;
    forceArrow.setLength(
      Math.max(length, 0.3),
      Math.min(0.3, length * 0.3),
      Math.min(0.2, length * 0.2),
    );
    forceOverlayHideAtMs = performance.now() + FORCE_OVERLAY_DURATION_MS;
  }

  // 正式なGizmo(設計docs/23-frontend/01-editor.md §1.2「Gizmo: 選択中オブジェクトの
  // Transformを直接ドラッグで編集」、§4「Scene View gizmo ドラッグ」)。縮約実装の
  // 理由: 移動(Translate)のみ(回転/スケールはこの2体デモに意味のある対象が無い
  // ——箱は軸並行のまま、床は静的平面——ため後続増分)。X(赤)/Y(緑)/Z(青)の3本の
  // 矢印を選択中ボディの位置に表示し、Editモードでのみ表示・操作可能(設計§4
  // 「Editモード…Scene View gizmo ドラッグ」「Playモード: 直接編集は不可」の
  // 境界どおり、Playモードでは非表示)。静的ボディ(床)を選択中は編集対象として
  // 意味が無いため非表示にする。
  const GIZMO_AXIS_LENGTH = 1.2;
  const GIZMO_HEAD_LENGTH = 0.28;
  const GIZMO_SHAFT_RADIUS = 0.03;
  const GIZMO_HEAD_RADIUS = 0.09;
  const GIZMO_AXES: {
    axis: THREE.Vector3;
    color: number;
    name: "x" | "y" | "z";
  }[] = [
    { axis: new THREE.Vector3(1, 0, 0), color: 0xff4444, name: "x" },
    { axis: new THREE.Vector3(0, 1, 0), color: 0x44ff44, name: "y" },
    { axis: new THREE.Vector3(0, 0, 1), color: 0x4488ff, name: "z" },
  ];
  const gizmoGroup = new THREE.Group();
  const gizmoHandleMeshes: { mesh: THREE.Mesh; axisName: "x" | "y" | "z" }[] =
    [];
  for (const { axis, color, name } of GIZMO_AXES) {
    const shaftLength = GIZMO_AXIS_LENGTH - GIZMO_HEAD_LENGTH;
    const material = new THREE.MeshBasicMaterial({ color });
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(
        GIZMO_SHAFT_RADIUS,
        GIZMO_SHAFT_RADIUS,
        shaftLength,
        8,
      ),
      material,
    );
    shaft.position.y = shaftLength / 2;
    const head = new THREE.Mesh(
      new THREE.ConeGeometry(GIZMO_HEAD_RADIUS, GIZMO_HEAD_LENGTH, 8),
      material,
    );
    head.position.y = shaftLength + GIZMO_HEAD_LENGTH / 2;
    const axisGroup = new THREE.Group();
    axisGroup.add(shaft, head);
    axisGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
    gizmoGroup.add(axisGroup);
    gizmoHandleMeshes.push(
      { mesh: shaft, axisName: name },
      { mesh: head, axisName: name },
    );
  }
  gizmoGroup.visible = false;
  scene.add(gizmoGroup);

  // Rotate Gizmo(設計§1.2「Gizmo: 移動/回転/スケール」の回転部分)。X(赤)/Y(緑)/
  // Z(青)の3本のリングを選択中ボディの位置に表示し、Translate Gizmoと同じく
  // Editモードかつ非静的ボディ選択時のみ表示・操作可能。リングをドラッグすると、
  // ドラッグ開始点との画面上の角度差をそのままワールド軸周りの回転角として
  // 適用する単純な実装(Blenderのようなビュー平面トラックボールではなく、
  // 選択軸周りの単純回転)。
  // **リングの半径は Translate Gizmo の矢印長(`GIZMO_AXIS_LENGTH`)に合わせる**
  // (QA不具合 C3-1)。以前は 1.0 で、矢印長 1.2・スケールハンドル 1.6 と
  // 三者三様だった——ギズモを掴む距離がツールごとに違うと、ユーザは「さっき
  // 掴めた場所」を頼りにできない。
  const ROTATION_RING_RADIUS = GIZMO_AXIS_LENGTH;
  const ROTATION_RING_TUBE_RADIUS = 0.03;
  // **掴み判定用の太いリング(不可視)**。見えているリングの管半径は 0.03 で、
  // 半径 1.2 のリングに対して当たり判定が実質「線」しかない。少しでも狙いが
  // ずれるとレイは何にも当たらず、`pointerdown` はボディのピックへ落ちて
  // **回転ドラッグが黙って無反応になる**(QA不具合 C3-1 の実測: リング上を
  // 正確に掴めば回転するが、半径 1.2 の位置——見た目のリングから 0.2 m 外——
  // では一切反応しなかった)。three.js の TransformControls と同じ手法で、
  // 描画しない太い当たり判定用メッシュを重ねて掴みやすくする。
  // `material.visible = false` は描画だけを止め、レイキャストには当たる
  // (Raycaster は可視性を見ない)。
  const ROTATION_PICKER_TUBE_RADIUS = 0.16;
  const rotationGizmoGroup = new THREE.Group();
  const rotationHandleMeshes: {
    mesh: THREE.Mesh;
    axisName: "x" | "y" | "z";
  }[] = [];
  for (const { axis, color, name } of GIZMO_AXES) {
    // TorusGeometryは既定でXY平面上(穴の軸はZ)にあるため、穴の軸を`axis`へ合わせる。
    const orientation = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 0, 1),
      axis,
    );
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(
        ROTATION_RING_RADIUS,
        ROTATION_RING_TUBE_RADIUS,
        8,
        48,
      ),
      new THREE.MeshBasicMaterial({ color }),
    );
    ring.quaternion.copy(orientation);
    rotationGizmoGroup.add(ring);
    const picker = new THREE.Mesh(
      new THREE.TorusGeometry(
        ROTATION_RING_RADIUS,
        ROTATION_PICKER_TUBE_RADIUS,
        6,
        32,
      ),
      new THREE.MeshBasicMaterial({ visible: false }),
    );
    picker.quaternion.copy(orientation);
    rotationGizmoGroup.add(picker);
    // 掴み判定は太い方だけを見る(見えるリングは描画専用)。
    rotationHandleMeshes.push({ mesh: picker, axisName: name });
  }
  rotationGizmoGroup.visible = false;
  scene.add(rotationGizmoGroup);

  // Scale Gizmo(設計§1.2「Gizmo: 移動/回転/スケール」のスケール部分)。単一の
  // 立方体ハンドル(黄)を選択中ボディの対角オフセット位置に表示し、Translate/
  // Rotate Gizmoと同じくEditモードかつ非静的ボディ選択時のみ表示・操作可能。
  // 縮約実装の理由: Blenderのような軸別スケールではなく単一の一様スケールのみ
  // (X/Y/Zハンドル無し)。ドラッグ開始点からハンドルまでの画面上の距離と、
  // ドラッグ中の現在距離の比をそのまま一様スケール係数の相対変化として使う
  // (Rotate Gizmoの「角度差をそのまま回転角に使う」設計と同じ発想)。
  // `sim-wasm::set_body_scale_at`はスポーン時の寸法からの絶対倍率を受け取る
  // ため、フロント側で現在のスケール値(`currentScale`、既定1.0)をボディごとに
  // 保持し、ドラッグ開始時の値に距離比を掛けた絶対値を毎回渡す。
  const SCALE_HANDLE_OFFSET = 1.6;
  const SCALE_HANDLE_SIZE = 0.16;
  const SCALE_MIN = 0.2;
  const SCALE_MAX = 4.0;
  const scaleGizmoGroup = new THREE.Group();
  const scaleHandleMesh = new THREE.Mesh(
    new THREE.BoxGeometry(
      SCALE_HANDLE_SIZE,
      SCALE_HANDLE_SIZE,
      SCALE_HANDLE_SIZE,
    ),
    new THREE.MeshBasicMaterial({ color: 0xffff00 }),
  );
  scaleHandleMesh.position.set(
    SCALE_HANDLE_OFFSET,
    SCALE_HANDLE_OFFSET,
    SCALE_HANDLE_OFFSET,
  );
  scaleGizmoGroup.add(scaleHandleMesh);
  scaleGizmoGroup.visible = false;
  scene.add(scaleGizmoGroup);
  const currentScale = new Map<number, number>();
  /// 軸別スケール(群2)。`currentScale`(等方)と別に持つ——`render()`が毎フレーム
  /// メッシュのスケールを書き戻すため、ここに残さないと1フレームで消える
  /// (実装検証中に発見: 入力した瞬間だけ変形して即座に戻っていた)。
  const currentScaleXyz = new Map<number, [number, number, number]>();

  // モーターアーム(`Command::SetMotorTarget`の実証用、設計docs/20-integration/
  // 04-world-api.md §2「Commandキュー」)。`motorArmBodies`はスポーン時に登録
  // されたモーター付きボディのindex集合(それ以外のボディへ`set_motor_target_at`
  // を呼ぶとRust側がパニックするため、UI側でも呼び先を絞る)。
  const MOTOR_TARGET_LOW = 0.0;
  const MOTOR_TARGET_HIGH = Math.PI / 2;
  const motorArmBodies = new Set<number>();
  const currentMotorTarget = new Map<number, number>();

  let selectedBodyIndex = BODY_INDEX_BOX;
  // **2026-07-28のD9/D34/D35増分で追加**: `selectedBodyIndex`が現在の`world`の
  // 有効なボディを指しているか(D9=熱のみ・D34/D35=天体のみのギャラリー
  // シーンは力学ボディを1つも持たないため、`selectedBodyIndex`が有効な
  // ボディを指さない状態が実際に起こり得る、`sceneGalleryRef.current`の
  // doc参照)。`render()`の毎フレーム呼び出し(`body_position_at_f32`等)や
  // Nudge/Prefabキャプチャのようなユーザー操作トリガのwasm呼び出しは、
  // いずれもこのチェックで先にガードする——ボディが無ければ何も送信/表示
  // しないのが唯一の無害な選択肢(`try_body_id_at`のResult化と同じ理由で
  // 「呼べば例外」なので、呼ぶ前に弾く)。
  function hasSelectedBody(): boolean {
    return selectedBodyIndex >= 0 && selectedBodyIndex < readNumber(world, "body_count");
  }
  function selectBody(index: number) {
    selectedBodyIndex = index;
    renderInspectorFor(world, index);
    highlightHierarchy(index);
    motorToggleButton.disabled = mode === "edit" || !motorArmBodies.has(index);
  }
  // フレーム階層ドリルインUI(Hierarchyの「Frames」サブツリー): クリックした
  // フレームを以後の「+ フレーム」ボタンの親候補にする(`selectedFrameIndex`は
  // このスコープの外側で宣言済み)。
  function selectFrame(frameIndex: number) {
    selectedFrameIndex = frameIndex;
    highlightHierarchy = rebuildHierarchy();
  }
  // **Hierarchy の右クリック操作(群2)**。実体(`hierarchyActionsImpl`)は
  // メッシュ管理・プレハブ機構が揃う後段で組み立てるので、ここでは**遅延解決の
  // プロキシ**を渡す——`setUpHierarchy` は呼び出し時点の `actions` を各行の
  // リスナへ焼き込むため、後から差し替えても既に作られた行には届かない。
  const hierarchyActionsRef: { current: HierarchyActions | null } = {
    current: null,
  };
  const hierarchyActions: HierarchyActions = {
    duplicate: (i) => hierarchyActionsRef.current?.duplicate(i),
    remove: (i) => hierarchyActionsRef.current?.remove(i),
    isolate: (i) => hierarchyActionsRef.current?.isolate(i),
    isolatedIndex: () => hierarchyActionsRef.current?.isolatedIndex() ?? null,
    capturePrefab: (i) => hierarchyActionsRef.current?.capturePrefab(i),
  };
  function rebuildHierarchy(): (index: number) => void {
    return setUpHierarchy(
      world,
      selectBody,
      selectedFrameIndex,
      selectFrame,
      hierarchyActions,
      SPAWN_MATERIALS,
    );
  }
  let highlightHierarchy = rebuildHierarchy();
  // Consoleのオブジェクト連動(増分E4、`SelectBodyRef`のdoc参照)。イベント行が
  // 持つ発生源ボディを選択できるようにする。**範囲外は無視する**——イベントは
  // 過去のstepで発生したものが表示され続けるため、その後シーンギャラリーで
  // ボディ数の少ないワールドへ差し替えると古いイベントのindexが範囲外になり得る
  // (`body_position_at_f32`等がErrをthrowしてrender()ループが壊れるのを防ぐ)。
  selectBodyRef.current = (index: number) => {
    if (index < 0 || index >= readNumber(world, "body_count")) return;
    selectBody(index);
  };
  // **Inspector の編集を Command キューへ配線する(群2、`InspectorEditRef`のdoc参照)**。
  // 適用は次stepの先頭なので、押した直後に Inspector を描き直しても値は
  // まだ変わっていない。**そこで嘘の即時反映をしない**——`render()`ループが
  // 毎フレーム値を読み直して表示を更新するため、実際に適用された step で
  // 表示が変わる(Playモードが止まっていれば `step` ボタンを押すまで変わらない、
  // これは「次step先頭で適用」という設計そのものが目に見えている状態)。
  inspectorEditRef.current = {
    setMass(bodyIndex, mass) {
      if (bodyIndex < 0 || bodyIndex >= readNumber(world, "body_count")) return;
      applyComponent(world, "push_set_body_mass", { body_index: bodyIndex, mass });
      pushCommandLog(world, { kind: "SetBodyMass", bodyIndex, mass });
    },
    setBodyType(bodyIndex, kind) {
      if (bodyIndex < 0 || bodyIndex >= readNumber(world, "body_count")) return;
      applyComponent(world, "push_set_body_type", { body_index: bodyIndex, kind });
      pushCommandLog(world, { kind: "SetBodyType", bodyIndex, bodyType: kind });
    },
    setCollisionFilter(bodyIndex, group, mask) {
      if (bodyIndex < 0 || bodyIndex >= readNumber(world, "body_count")) return;
      applyComponent(world, "push_set_collision_filter", { body_index: bodyIndex, group, mask });
      pushCommandLog(world, {
        kind: "SetCollisionFilter",
        bodyIndex,
        group,
        mask,
      });
    },
    setScaleXyz(bodyIndex, sx, sy, sz) {
      if (bodyIndex <= 0 || bodyIndex >= readNumber(world, "body_count")) return false;
      const { applied } = applyComponent(world, "set_body_scale_xyz_at", {
        index: bodyIndex,
        sx,
        sy,
        sz,
      });
      if (applied) {
        // Three.js 側のメッシュは基準ジオメトリ×スケールで表示しているので、
        // 同じ倍率を掛ける(`currentScale` は等方スケール用なので触らない)。
        currentScaleXyz.set(bodyIndex, [sx, sy, sz]);
      }
      return applied ?? false;
    },
    setScale(bodyIndex, scale) {
      if (bodyIndex <= 0 || bodyIndex >= readNumber(world, "body_count")) return false;
      try {
        applyComponent(world, "set_body_scale_at", { index: bodyIndex, scale });
        currentScale.set(bodyIndex, scale);
        currentScaleXyz.delete(bodyIndex);
        return true;
      } catch {
        return false;
      }
    },
    setPosition(bodyIndex, x, y, z) {
      if (bodyIndex < 0 || bodyIndex >= readNumber(world, "body_count")) return;
      applyComponent(world, "set_body_position_at", { index: bodyIndex, x, y, z });
    },
  };
  renderInspectorFor(world, selectedBodyIndex);

  // Scene Viewピック(設計docs/23-frontend/01-editor.md §1.2「クリックでbody/
  // joint/probeを選択。Alt-クリックで下層(重なった裏)を選択」)。Playモードでは
  // 箱を直接ドラッグして`Command::Grab/MoveGrab/Release`(設計§4「ドラッグ系は
  // Commandキュー経由」)でワールド座標の目標点へ剛にピン留めする物理的な
  // "つかむ"操作(移動量が閾値未満なら通常のクリック選択、pointerdown/move/upの
  // 3イベントで判別)。Editモードでは箱本体への直接ドラッグは行わず(設計§4の
  // 「Editモード…Scene View gizmo ドラッグ」どおりGizmo経由のみ)、Gizmoの
  // 軸ハンドルをドラッグすると`set_body_position_at`でその軸方向にのみ位置を
  // 直接書き換える。
  let pickables: { mesh: THREE.Object3D; bodyIndex: number }[] = [
    { mesh: ground, bodyIndex: BODY_INDEX_GROUND },
    { mesh: box, bodyIndex: BODY_INDEX_BOX },
  ];
  // 拘束オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「拘束」)向けの
  // THREE.Line(振り子スポーンごとに1本、`world.constraint_anchor_points_at`が
  // 返す2点を毎フレーム反映する)。
  const constraintLines = new Map<number, THREE.Line>();
  const raycaster = new THREE.Raycaster();
  const pointerNdc = new THREE.Vector2();
  const dragPlane = new THREE.Plane();
  const dragPlaneHit = new THREE.Vector3();
  const cameraDirection = new THREE.Vector3();
  const DRAG_THRESHOLD_PX = 4;

  function updatePointerNdc(event: PointerEvent) {
    const rect = renderer.domElement.getBoundingClientRect();
    pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  }

  function hitTest(event: PointerEvent, wantBack: boolean) {
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    const hits = raycaster.intersectObjects(pickables.map((p) => p.mesh));
    const hit = hits[wantBack && hits.length > 1 ? 1 : 0];
    if (!hit) return null;
    const picked = pickables.find((p) => p.mesh === hit.object);
    return picked ? { picked, worldPoint: hit.point } : null;
  }

  function hitGizmo(event: PointerEvent): "x" | "y" | "z" | null {
    if (!gizmoGroup.visible) return null;
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    const hits = raycaster.intersectObjects(
      gizmoHandleMeshes.map((h) => h.mesh),
    );
    if (!hits.length) return null;
    const handle = gizmoHandleMeshes.find((h) => h.mesh === hits[0].object);
    return handle ? handle.axisName : null;
  }

  function hitRotationGizmo(event: PointerEvent): "x" | "y" | "z" | null {
    if (!rotationGizmoGroup.visible) return null;
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    const hits = raycaster.intersectObjects(
      rotationHandleMeshes.map((h) => h.mesh),
    );
    if (!hits.length) return null;
    const handle = rotationHandleMeshes.find((h) => h.mesh === hits[0].object);
    return handle ? handle.axisName : null;
  }

  function hitScaleGizmo(event: PointerEvent): boolean {
    if (!scaleGizmoGroup.visible) return false;
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    return raycaster.intersectObject(scaleHandleMesh).length > 0;
  }

  const AXIS_VECTORS: Record<"x" | "y" | "z", THREE.Vector3> = {
    x: new THREE.Vector3(1, 0, 0),
    y: new THREE.Vector3(0, 1, 0),
    z: new THREE.Vector3(0, 0, 1),
  };

  let dragStartScreen: { x: number; y: number } | null = null;
  let pointerDownHit: ReturnType<typeof hitTest> = null;
  let pointerDownGizmoAxis: "x" | "y" | "z" | null = null;
  let pointerDownRotationAxis: "x" | "y" | "z" | null = null;
  let pointerDownScaleHit = false;
  let isDragging = false;
  let dragMode: "grab" | "gizmo" | "rotate" | "scale" | null = null;
  // 現在grab中のボディ(**残タスク完遂のシーンギャラリー増分**で`BODY_INDEX_BOX`
  // 決め打ちから一般化した——ピックしたボディをそのままgrab対象にする、
  // `push_grab`/`push_move_grab`/`push_release`のdoc参照)。
  let grabbedBodyIndex = BODY_INDEX_BOX;
  const gizmoAxisDir = new THREE.Vector3();
  const gizmoDragStartPosition = new THREE.Vector3();
  let gizmoDragStartScalar = 0;
  let rotateAxisDir = new THREE.Vector3();
  let rotateStartQuat = new THREE.Quaternion();
  let rotateCenterScreen = { x: 0, y: 0 };
  let rotateStartAngle = 0;
  let scaleCenterScreen = { x: 0, y: 0 };
  let scaleDragStartDistance = 0;
  let scaleDragStartValue = 1.0;

  // Undo/Redo(Editモードのみ、設計docs/23-frontend/01-editor.md §6「Undo/Redo:
  // Editモードのみ。編集操作はシーンJSONの差分として保持」)。縮約実装の理由:
  // シーンJSON差分ではなく、Gizmoドラッグ開始直前の位置/姿勢を積むだけの単純な
  // 2本のスタック(Undo/Redo双方)。ドラッグ(Translate/Rotateいずれも)開始の
  // たびに直前の値をUndoスタックへ1件積み、新規ドラッグはRedoスタックを破棄する
  // (標準的なUndo/Redoの意味論)。
  const EDIT_UNDO_STACK_CAPACITY = 20;
  type EditUndoEntry =
    | { bodyIndex: number; kind: "position"; position: THREE.Vector3 }
    | { bodyIndex: number; kind: "rotation"; rotation: THREE.Quaternion }
    | { bodyIndex: number; kind: "scale"; scale: number };
  const editUndoStack: EditUndoEntry[] = [];
  const editRedoStack: EditUndoEntry[] = [];

  function captureCurrentEntry(
    bodyIndex: number,
    kind: "position" | "rotation" | "scale",
  ): EditUndoEntry {
    if (kind === "position") {
      const p = world.body_position_at_f32(bodyIndex);
      return {
        bodyIndex,
        kind: "position",
        position: new THREE.Vector3(p[0], p[1], p[2]),
      };
    }
    if (kind === "rotation") {
      const r = world.body_rotation_at_f32(bodyIndex);
      return {
        bodyIndex,
        kind: "rotation",
        rotation: new THREE.Quaternion(r[0], r[1], r[2], r[3]),
      };
    }
    return {
      bodyIndex,
      kind: "scale",
      scale: currentScale.get(bodyIndex) ?? 1.0,
    };
  }

  function projectToScreen(worldPos: THREE.Vector3): { x: number; y: number } {
    const ndc = worldPos.clone().project(camera);
    const rect = renderer.domElement.getBoundingClientRect();
    return {
      x: rect.left + ((ndc.x + 1) / 2) * rect.width,
      y: rect.top + ((1 - ndc.y) / 2) * rect.height,
    };
  }

  renderer.domElement.addEventListener("pointerdown", (event) => {
    dragStartScreen = { x: event.clientX, y: event.clientY };
    isDragging = false;
    dragMode = null;
    if (mode === "edit") {
      pointerDownGizmoAxis = hitGizmo(event);
      pointerDownRotationAxis = pointerDownGizmoAxis
        ? null
        : hitRotationGizmo(event);
      pointerDownScaleHit =
        !pointerDownGizmoAxis &&
        !pointerDownRotationAxis &&
        hitScaleGizmo(event);
      pointerDownHit =
        pointerDownGizmoAxis || pointerDownRotationAxis || pointerDownScaleHit
          ? null
          : hitTest(event, event.altKey);
    } else {
      pointerDownGizmoAxis = null;
      pointerDownRotationAxis = null;
      pointerDownScaleHit = false;
      pointerDownHit = hitTest(event, event.altKey);
    }
  });

  renderer.domElement.addEventListener("pointermove", (event) => {
    if (!dragStartScreen) return;
    const dx = event.clientX - dragStartScreen.x;
    const dy = event.clientY - dragStartScreen.y;
    if (!isDragging) {
      if (Math.hypot(dx, dy) < DRAG_THRESHOLD_PX) return;
      if (pointerDownRotationAxis) {
        isDragging = true;
        dragMode = "rotate";
        rotateAxisDir.copy(AXIS_VECTORS[pointerDownRotationAxis]);
        const r = world.body_rotation_at_f32(selectedBodyIndex);
        rotateStartQuat.set(r[0], r[1], r[2], r[3]);
        rotateCenterScreen = projectToScreen(rotationGizmoGroup.position);
        rotateStartAngle = Math.atan2(
          event.clientY - rotateCenterScreen.y,
          event.clientX - rotateCenterScreen.x,
        );
        editUndoStack.push({
          bodyIndex: selectedBodyIndex,
          kind: "rotation",
          rotation: rotateStartQuat.clone(),
        });
        if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY)
          editUndoStack.shift();
        editRedoStack.length = 0;
        undoButton.disabled = mode !== "edit";
        redoButton.disabled = true;
      } else if (pointerDownGizmoAxis) {
        isDragging = true;
        dragMode = "gizmo";
        gizmoAxisDir.copy(AXIS_VECTORS[pointerDownGizmoAxis]);
        // Local 座標系ならボディの姿勢で軸を回す(群2、設計 §1.2)。
        if (gizmoSpace === "local")
          gizmoAxisDir.applyQuaternion(inspectorRotationQuat).normalize();
        gizmoDragStartPosition.copy(gizmoGroup.position);
        gizmoDragStartScalar = gizmoAxisDir.dot(gizmoDragStartPosition);
        editUndoStack.push({
          bodyIndex: selectedBodyIndex,
          kind: "position",
          position: gizmoDragStartPosition.clone(),
        });
        if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY)
          editUndoStack.shift();
        editRedoStack.length = 0;
        undoButton.disabled = mode !== "edit";
        redoButton.disabled = true;
        camera.getWorldDirection(cameraDirection);
        let planeNormal = cameraDirection
          .clone()
          .sub(
            gizmoAxisDir
              .clone()
              .multiplyScalar(cameraDirection.dot(gizmoAxisDir)),
          );
        if (planeNormal.lengthSq() < 1e-9) {
          planeNormal = new THREE.Vector3().crossVectors(
            gizmoAxisDir,
            new THREE.Vector3(0, 1, 0),
          );
          if (planeNormal.lengthSq() < 1e-9) {
            planeNormal.crossVectors(gizmoAxisDir, new THREE.Vector3(1, 0, 0));
          }
        }
        planeNormal.normalize();
        dragPlane.setFromNormalAndCoplanarPoint(
          planeNormal,
          gizmoDragStartPosition,
        );
      } else if (pointerDownScaleHit) {
        isDragging = true;
        dragMode = "scale";
        scaleCenterScreen = projectToScreen(scaleGizmoGroup.position);
        scaleDragStartDistance = Math.max(
          Math.hypot(
            event.clientX - scaleCenterScreen.x,
            event.clientY - scaleCenterScreen.y,
          ),
          10,
        );
        scaleDragStartValue = currentScale.get(selectedBodyIndex) ?? 1.0;
        editUndoStack.push({
          bodyIndex: selectedBodyIndex,
          kind: "scale",
          scale: scaleDragStartValue,
        });
        if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY)
          editUndoStack.shift();
        editRedoStack.length = 0;
        undoButton.disabled = mode !== "edit";
        redoButton.disabled = true;
      } else {
        if (mode !== "play" || !pointerDownHit) return;
        isDragging = true;
        dragMode = "grab";
        grabbedBodyIndex = pointerDownHit.picked.bodyIndex;
        selectBody(grabbedBodyIndex);
        camera.getWorldDirection(cameraDirection);
        dragPlane.setFromNormalAndCoplanarPoint(
          cameraDirection,
          pointerDownHit.worldPoint,
        );
        // `body_position_at_f32`はWasmメモリを直接指す一時的なビューを返す
        // (B16、`HotPathViewBuffers`のdoc参照)。下の`applyComponent`が挟む
        // Wasm呼び出し(`apply_component`)より前に、`pushCommandLog`でも
        // 再利用する3値をここで読み切っておく(でないと2回目の参照時には
        // 無効化されている恐れがある)。
        const [px, py, pz] = world.body_position_at_f32(grabbedBodyIndex);
        applyComponent(world, "push_grab", {
          body_index: grabbedBodyIndex,
          target_x: px,
          target_y: py,
          target_z: pz,
        });
        pushCommandLog(world, {
          kind: "Grab",
          bodyIndex: grabbedBodyIndex,
          targetX: px,
          targetY: py,
          targetZ: pz,
        });
      }
    }
    if (dragMode === "rotate") {
      // 回転はドラッグ平面ではなく画面上の角度差(中心=Gizmo位置の画面投影)を
      // そのままワールド軸周りの回転角として使う(モジュールdoc参照)。
      const currentAngle = Math.atan2(
        event.clientY - rotateCenterScreen.y,
        event.clientX - rotateCenterScreen.x,
      );
      const deltaAngle = currentAngle - rotateStartAngle;
      const deltaQuat = new THREE.Quaternion().setFromAxisAngle(
        rotateAxisDir,
        deltaAngle,
      );
      const newQuat = deltaQuat.multiply(rotateStartQuat);
      applyComponent(world, "set_body_rotation_at", {
        index: selectedBodyIndex,
        x: newQuat.x,
        y: newQuat.y,
        z: newQuat.z,
        w: newQuat.w,
      });
      return;
    }
    if (dragMode === "scale") {
      // ドラッグ開始点からハンドルまでの画面上の距離との比を、そのまま
      // ドラッグ開始時点のスケール値への相対倍率として使う(モジュールdoc参照)。
      const currentDistance = Math.hypot(
        event.clientX - scaleCenterScreen.x,
        event.clientY - scaleCenterScreen.y,
      );
      const factor = Math.min(
        Math.max(
          scaleDragStartValue * (currentDistance / scaleDragStartDistance),
          SCALE_MIN,
        ),
        SCALE_MAX,
      );
      applyComponent(world, "set_body_scale_at", { index: selectedBodyIndex, scale: factor });
      currentScale.set(selectedBodyIndex, factor);
      // 等方スケール Gizmo を使ったら軸別スケールは破棄する(両方を同時に
      // 効かせると `set_body_scale_at`(基準形状×倍率)と食い違う)。
      currentScaleXyz.delete(selectedBodyIndex);
      renderInspectorFor(world, selectedBodyIndex);
      return;
    }
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    if (!raycaster.ray.intersectPlane(dragPlane, dragPlaneHit)) return;
    if (dragMode === "gizmo") {
      const t = gizmoAxisDir.dot(dragPlaneHit);
      const delta = t - gizmoDragStartScalar;
      const newPos = gizmoDragStartPosition
        .clone()
        .addScaledVector(gizmoAxisDir, delta);
      // **グリッドスナップ(群2)**。設計 §1.2「グリッド・スナップ(既定10cm、
      // 変更可)」。Settings で 0 にすると連続移動になる。
      // ドラッグしている軸の成分だけを丸める——3成分すべてを丸めると、
      // 軸に沿った移動なのに他の軸まで動いてしまう。
      //
      // **Local 座標系ではスナップしない**——グリッドは世界座標に固定された
      // 格子なので、傾いた軸に沿って動かしながら世界格子へ丸めると、動かして
      // いない軸まで格子へ引っ張られて軸拘束が壊れる(実際に試して確認した)。
      const snapped =
        gizmoSpace === "local"
          ? newPos.clone()
          : new THREE.Vector3(
              Math.abs(gizmoAxisDir.x) > 0.5 ? snapToGrid(newPos.x) : newPos.x,
              Math.abs(gizmoAxisDir.y) > 0.5 ? snapToGrid(newPos.y) : newPos.y,
              Math.abs(gizmoAxisDir.z) > 0.5 ? snapToGrid(newPos.z) : newPos.z,
            );
      applyComponent(world, "set_body_position_at", {
        index: selectedBodyIndex,
        x: snapped.x,
        y: snapped.y,
        z: snapped.z,
      });
      markUnsaved();
    } else if (dragMode === "grab") {
      applyComponent(world, "push_move_grab", {
        body_index: grabbedBodyIndex,
        target_x: dragPlaneHit.x,
        target_y: dragPlaneHit.y,
        target_z: dragPlaneHit.z,
      });
    }
  });

  renderer.domElement.addEventListener("pointerup", () => {
    // **スケッチツール中はクリック選択を行わない(D1)**。左クリックは
    // 「頂点を置く」に割り当てられており、選択が同時に走るとクリックの
    // たびに Inspector が別のボディへ飛んで作図に集中できない
    // (頂点を置く処理そのものは、下で登録するスケッチ専用リスナが担う)。
    // ドラッグによるカメラ操作は OrbitControls 側でそのまま効く。
    if (gizmoTool === "sketch") {
      isDragging = false;
      dragMode = null;
      dragStartScreen = null;
      pointerDownHit = null;
      pointerDownGizmoAxis = null;
      pointerDownRotationAxis = null;
      pointerDownScaleHit = false;
      return;
    }
    if (isDragging) {
      if (dragMode === "grab") {
        applyComponent(world, "push_release", { body_index: grabbedBodyIndex });
        pushCommandLog(world, { kind: "Release", bodyIndex: grabbedBodyIndex });
      }
    } else if (pointerDownHit) {
      selectBody(pointerDownHit.picked.bodyIndex);
    }
    isDragging = false;
    dragMode = null;
    dragStartScreen = null;
    pointerDownHit = null;
    pointerDownGizmoAxis = null;
    pointerDownRotationAxis = null;
    pointerDownScaleHit = false;
  });

  const hud = document.getElementById("hud")!;
  const hashDisplay = document.getElementById("hash-display")!;
  const timelineTime = document.getElementById("timeline-time")!;
  const timelineStep = document.getElementById("timeline-step")!;
  const playButton = document.getElementById("btn-play") as HTMLButtonElement;
  const stepButton = document.getElementById("btn-step") as HTMLButtonElement;
  const nudgeButton = document.getElementById("btn-nudge") as HTMLButtonElement;
  const motorToggleButton = document.getElementById(
    "btn-motor-toggle",
  ) as HTMLButtonElement;
  // 分圧回路のスイッチ(`Command::SetSwitch`実証用、設計docs/20-integration/
  // 04-world-api.md §2「Commandキュー」)。`WasmWorld::new`が既に分圧回路
  // (電源10V→100Ω→分圧点、分圧点→200Ω→GND、分圧点↔GNDにスイッチ)を構築
  // 済みなので、フロントエンドは切替のみを担う。HUDの`circuit V`行が
  // 実際の分圧点電圧(開: 約6.67V、閉: 約0V)を毎フレーム表示する。
  const circuitSwitchToggle = document.getElementById(
    "toggle-circuit-switch",
  ) as HTMLInputElement;
  // ヒーター(`Command::SetHeatSource`実証用)。モジュールdoc「1step分だけ効く」
  // 縮約セマンティクスのとおり、継続加熱するには毎stepの直前に再度
  // `push_heat_source`を呼ぶ必要がある(`frame()`ループ内、`world.step()`の
  // 直前で呼ぶ)。HUDの`heater T`行が熱ノードの現在温度(ニュートン冷却あり、
  // 時定数τ=10s)を毎フレーム表示する。
  const HEATER_WATTS = 2000.0;
  const heaterToggle = document.getElementById(
    "toggle-heater",
  ) as HTMLInputElement;

  /// Thrust(**残タスク完遂の縦串⑤増分**、`ThrustState`のdocコメント参照)。
  /// ヒーターと同じ「1step分だけ効く力を毎stepの直前に再送する」パターンで、
  /// エンジン有効な各ボディについてローカル軸をそのstepの姿勢でワールドへ
  /// 回し、スロットル×最大推力を`push_apply_force`で送る。
  const thrustQuat = new THREE.Quaternion();
  const thrustAxis = new THREE.Vector3();
  function applyThrustForStep(): void {
    for (const [bodyIndex, state] of thrustByBody) {
      if (!state.enabled || state.throttle <= 0) continue;
      if (bodyIndex >= readNumber(world, "body_count") || (world.read_component("body_is_removed_at", String(bodyIndex)) === "true"))
        continue;
      const rot = world.body_rotation_at_f32(bodyIndex);
      thrustQuat.set(rot[0], rot[1], rot[2], rot[3]);
      thrustAxis
        .set(state.axis[0], state.axis[1], state.axis[2])
        .applyQuaternion(thrustQuat);
      const magnitude = state.throttle * state.maxThrust;
      applyComponent(world, "push_apply_force", {
        body_index: bodyIndex,
        fx: thrustAxis.x * magnitude,
        fy: thrustAxis.y * magnitude,
        fz: thrustAxis.z * magnitude,
      });
    }
  }

  const undoButton = document.getElementById("btn-undo") as HTMLButtonElement;
  const redoButton = document.getElementById("btn-redo") as HTMLButtonElement;

  // 時間倍率(設計docs/23-frontend/01-editor.md §1.1「Toolbar: 時間倍率スライダー」)。
  // dt自体は変えず(物理の決定論性はステップ幅に依存する。dtはSettingsから
  // Editモード限定で変更できる)、1描画フレームあたりに進める実時間
  // (`frameSeconds`)をこの倍率でスケールして、見かけの再生速度のみを変える。
  //
  // **群2で範囲を ×1/8〜×128 へ広げた**(以前は ×0.5/×1/×2/×5 の4段のみ)。
  // 微速は接触の瞬間を観察するのに、高倍率は熱伝導・天体のように時定数が
  // 秒〜分の現象を待つのに要る。ただし1フレームあたりのstep数には上限
  // (`MAX_STEPS_PER_FRAME`)があるので、**高倍率では指定どおりの速度が出ない**。
  // それを黙って隠すと「×128にしたのに速くならない」という説明のつかない
  // 挙動になるため、実際に達成できている倍率を隣に出し、指定値に届かない
  // ときは赤くする。
  const timescaleSelect = document.getElementById(
    "select-timescale",
  ) as HTMLSelectElement;
  const timescaleEffective = document.getElementById("timescale-effective")!;
  let timeScale = Number.parseFloat(timescaleSelect.value);
  timescaleSelect.addEventListener("change", () => {
    timeScale = Number.parseFloat(timescaleSelect.value);
  });
  // フレームごとの実測値はばらつくので指数移動平均で均す(生値だと数字が
  // 目まぐるしく変わって読めない)。
  let effectiveTimeScale = timeScale;
  // **赤字の判定は「1フレームあたりの step 数上限に当たったか」という事実で行う**。
  // 当初は「実効倍率が指定値の9割未満なら赤」という比率で判定していたが、
  // これは機械の速さ次第で結果が変わる——実測: 60fps・dt=1/120 で ×128 は
  // 1フレーム 256 step を要求し上限 240 で頭打ちになるのに、240/256 = 93.75% は
  // 9割の閾値を超えるので**赤くならなかった**(遅い機械では赤くなる)。
  // 上限に当たったことは真偽で分かる事実なので、比率で推測せず直接見る。
  // 単発のフレーム落ちで表示がちらつかないよう、一定フレーム保持する。
  const CAPPED_INDICATOR_HOLD_FRAMES = 30;
  let cappedIndicatorFrames = 0;
  function updateEffectiveTimeScale(measured: number, capped: boolean) {
    effectiveTimeScale += (measured - effectiveTimeScale) * 0.1;
    timescaleEffective.textContent = `×${effectiveTimeScale.toFixed(2)}`;
    if (capped) cappedIndicatorFrames = CAPPED_INDICATOR_HOLD_FRAMES;
    else if (cappedIndicatorFrames > 0) cappedIndicatorFrames -= 1;
    const degraded = cappedIndicatorFrames > 0;
    timescaleEffective.classList.toggle("degraded", degraded);
    timescaleEffective.title = degraded
      ? `1フレームあたりの step 数上限(${MAX_STEPS_PER_FRAME})に当たっているため、指定した ×${timeScale} は出せていません。`
      : "実際に達成できている時間倍率。";
  }

  // Edit/Play モードの分離(設計§4「Edit モード: シーンの直接編集が可能…Play を
  // 押した瞬間の状態が実行の初期条件になる」「Play モード: 直接編集は不可。
  // 介入は全て Command」)。既定はEditモード(Unityと同じ起動時挙動、Playを
  // 押すまでシミュレーションは進まない)。
  type Mode = "edit" | "play";
  let mode: Mode = "edit";
  let playing = false;
  const modeEditButton = document.getElementById(
    "btn-mode-edit",
  ) as HTMLButtonElement;
  const modePlayButton = document.getElementById(
    "btn-mode-play",
  ) as HTMLButtonElement;

  function setMode(next: Mode) {
    mode = next;
    playing = next === "play";
    playButton.textContent = playing ? "⏸" : "▶";
    playButton.disabled = mode === "edit";
    // `stepButton.disabled` は `render()` が毎フレーム`playing`込みで
    // 同期する(QA不具合5、再生中は無効化する必要があるため)。
    nudgeButton.disabled = mode === "edit";
    motorToggleButton.disabled =
      mode === "edit" || !motorArmBodies.has(selectedBodyIndex);
    // 自由配線回路エディタでリセット済みなら、モード切替に関わらず無効のまま
    // (`circuitFreeWiringState`のdoc参照——`circuit_switch_index`が新回路の
    // スイッチ数を超えて無効になり得るため、再有効化してはならない)。
    circuitSwitchToggle.disabled =
      mode === "edit" || circuitFreeWiringState.active;
    heaterToggle.disabled = mode === "edit";
    undoButton.disabled = mode !== "edit" || editUndoStack.length === 0;
    redoButton.disabled = mode !== "edit" || editRedoStack.length === 0;
    modeEditButton.classList.toggle("active", mode === "edit");
    modePlayButton.classList.toggle("active", mode === "play");
  }
  modeEditButton.addEventListener("click", () => setMode("edit"));
  modePlayButton.addEventListener("click", () => setMode("play"));
  setMode("edit");

  function applyEditEntry(entry: EditUndoEntry) {
    if (entry.kind === "position") {
      applyComponent(world, "set_body_position_at", {
        index: entry.bodyIndex,
        x: entry.position.x,
        y: entry.position.y,
        z: entry.position.z,
      });
    } else if (entry.kind === "rotation") {
      applyComponent(world, "set_body_rotation_at", {
        index: entry.bodyIndex,
        x: entry.rotation.x,
        y: entry.rotation.y,
        z: entry.rotation.z,
        w: entry.rotation.w,
      });
    } else {
      applyComponent(world, "set_body_scale_at", {
        index: entry.bodyIndex,
        scale: entry.scale,
      });
      currentScale.set(entry.bodyIndex, entry.scale);
      renderInspectorFor(world, entry.bodyIndex);
    }
  }

  undoButton.addEventListener("click", () => {
    if (mode !== "edit") return;
    const entry = editUndoStack.pop();
    if (!entry) return;
    editRedoStack.push(captureCurrentEntry(entry.bodyIndex, entry.kind));
    if (editRedoStack.length > EDIT_UNDO_STACK_CAPACITY) editRedoStack.shift();
    applyEditEntry(entry);
    undoButton.disabled = editUndoStack.length === 0;
    redoButton.disabled = editRedoStack.length === 0;
    render();
  });

  redoButton.addEventListener("click", () => {
    if (mode !== "edit") return;
    const entry = editRedoStack.pop();
    if (!entry) return;
    editUndoStack.push(captureCurrentEntry(entry.bodyIndex, entry.kind));
    if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY) editUndoStack.shift();
    applyEditEntry(entry);
    redoButton.disabled = editRedoStack.length === 0;
    undoButton.disabled = editUndoStack.length === 0;
    render();
  });

  // スポーンパレット(設計docs/23-frontend/01-editor.md §6「形状×材質を選んで
  // クリック配置(`create_body`)」)。Toolbarの「+ 球」/「+ 箱」ボタンで、床の
  // 中心付近・上空(`SPAWN_HEIGHT`)へ新規ボディを配置する。落下位置が重ならない
  // よう、配置のたびにx/zを少しずつずらす(スポーン回数から決定的に算出)。
  const spawnMaterialSelect = document.getElementById(
    "select-spawn-material",
  ) as HTMLSelectElement;
  for (const material of SPAWN_MATERIALS) {
    const option = document.createElement("option");
    option.value = material;
    option.textContent = material;
    spawnMaterialSelect.appendChild(option);
  }

  // `sceneBaseBodyCount`: 現在の`world`が最初から持つボディ数(既定シーンは
  // 床+箱の2体、`loadScene`でギャラリーシーンへ差し替えた後はそのシーンの
  // ボディ数)。以後のスポーンパレット操作による「これまでのスポーン数」の
  // 基準点として使う。
  let sceneBaseBodyCount = 2;
  /// 未保存の変更があるか(群2、`beforeunload` のdoc参照)。
  let hasUnsavedChanges = false;
  function markUnsaved() {
    hasUnsavedChanges = true;
  }
  function nextSpawnPosition(): { x: number; z: number } {
    const n = readNumber(world, "body_count") - sceneBaseBodyCount; // これまでのスポーン数
    const angle = n * 2.4; // 黄金角に近い値、重ならないようばらけさせる
    const radius = 1.5 + n * 0.3;
    return { x: Math.cos(angle) * radius, z: Math.sin(angle) * radius };
  }

  function addSpawnedMesh(bodyIndex: number, mesh: THREE.Mesh) {
    markUnsaved();
    scene.add(mesh);
    pickables.push({ mesh, bodyIndex });
    bodyMeshes.set(bodyIndex, mesh);
    highlightHierarchy = rebuildHierarchy();
    selectBody(bodyIndex);
  }

  // シーンJSON Import(`SceneImportRef`のdoc参照)。`world.import_scene_json`が
  // 検証+ボディ追加を行い、追加件数を返す(検証エラーはJS例外として投げられる、
  // 呼び出し側のProjectドロワーが`.catch`で拾う)。形状ごとのメッシュ生成は
  // Importに渡した生のシーンJSONを(Rust側とは独立に)自前で`JSON.parse`して
  // 読む(`ImportedShapeJson`のdoc参照)。Plane形状は既存のスポーンパレットに
  // 対応物が無いため、大きな平板メッシュで代用し、位置は物理的な平面の定義
  // (`normal・p=d`、剛体の`transform.position`ではなく`normal`/`d`自体)に
  // 合わせて`normal.scale(d)`に置く。
  // 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5、`prediction_prompts`
  // を宣言したシーンJSONをImportした場合のみ表示するオプトイン——シーン側に
  // 無ければ既定でスキップされる、モジュール冒頭の設計ノート「全画面遷移を強制
  // しない」に対応)。停止条件による自動一時停止・Probeグラフへの予測線重ね描き
  // は対象外(縮約実装)——予測入力+実測/予測/解析解の比較表のみ実装する。
  let currentPredictionPrompts: ImportedPredictionPromptJson[] = [];
  const predictionPanel = document.getElementById("prediction-panel")!;

  function renderPredictionPanel() {
    predictionPanel.innerHTML = "";
    if (currentPredictionPrompts.length === 0) {
      predictionPanel.style.display = "none";
      return;
    }
    predictionPanel.style.display = "block";
    const heading = document.createElement("h3");
    heading.textContent = "予測 → 実験";
    predictionPanel.appendChild(heading);
    currentPredictionPrompts.forEach((prompt, i) => {
      const questionLine = document.createElement("div");
      questionLine.className = "inspector-field";
      questionLine.textContent = prompt.question;
      predictionPanel.appendChild(questionLine);

      const guessRow = document.createElement("div");
      guessRow.className = "inspector-field";
      const guessLabel = document.createElement("span");
      guessLabel.textContent = "あなたの予測";
      const guessInput = document.createElement("input");
      guessInput.type = "number";
      guessInput.id = `prediction-guess-${i}`;
      guessRow.append(guessLabel, guessInput);
      predictionPanel.appendChild(guessRow);

      const resultLine = document.createElement("div");
      resultLine.className = "inspector-field";
      resultLine.id = `prediction-result-${i}`;
      predictionPanel.appendChild(resultLine);
    });
  }

  function updatePredictionResults() {
    currentPredictionPrompts.forEach((prompt, i) => {
      const resultLine = document.getElementById(`prediction-result-${i}`);
      if (!resultLine) return;
      const guessInput = document.getElementById(
        `prediction-guess-${i}`,
      ) as HTMLInputElement | null;
      const actual = readNumber(world, "imported_probe_value_at", String(prompt.probe_index));
      const guessText = guessInput?.value
        ? Number(guessInput.value).toFixed(3)
        : "(未入力)";
      resultLine.textContent = `実測=${actual.toFixed(3)} / 予測=${guessText} / 解析解=${prompt.expected_value.toFixed(3)}`;
    });
  }

  /// **形状描画をShape記述に一本化(縦串①の独立項目)**。以前は
  /// `sceneImportRef.current`と`sceneGalleryRef.current`にほぼ同一の形状パーサ
  /// (plane/sphere/box の分岐)が2箇所コピーされており、`ImportedShapeJson`が
  /// `Capsule`を受け付けないままだったため**カプセルを含むシーンは0.3mの球として
  /// 描かれていた**(出荷済みシーンが未使用のため当時は未発現)。この1関数へ
  /// 集約し、Capsuleも実寸法で描く。Plane以外の未知形状はコンソール警告を出す
  /// (黙って0.3mの球を出さない)。
  ///
  /// Plane専用の位置決め(normal/dから逆算)は他の形状(ボディの現在位置/姿勢を
  /// worldへ問い合わせる)と経路が異なるため、戻り値に`isPlane`を含めて呼び出し側が
  /// 分岐する。
  function meshFromShapeJson(shape: ImportedShapeJson | undefined): {
    mesh: THREE.Mesh;
    isPlane: boolean;
  } {
    if (shape && "plane" in shape) {
      const [nx, ny, nz] = shape.plane.normal;
      const normal = new THREE.Vector3(nx, ny, nz).normalize();
      const mesh = new THREE.Mesh(
        new THREE.PlaneGeometry(20, 20),
        new THREE.MeshStandardMaterial({
          color: 0x777755,
          side: THREE.DoubleSide,
        }),
      );
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
      mesh.position.copy(normal.multiplyScalar(shape.plane.d));
      return { mesh, isPlane: true };
    }
    if (shape && "sphere" in shape) {
      return {
        mesh: new THREE.Mesh(
          new THREE.SphereGeometry(shape.sphere.radius, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        ),
        isPlane: false,
      };
    }
    if (shape && "box" in shape) {
      const [hx, hy, hz] = shape.box.half;
      return {
        mesh: new THREE.Mesh(
          new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        ),
        isPlane: false,
      };
    }
    if (shape && "capsule" in shape) {
      // THREE.CapsuleGeometryのlengthは「円柱部の長さ」= 2*half_height
      // (`spawnShapeAt`の同じ変換のdoc参照)。
      return {
        mesh: new THREE.Mesh(
          new THREE.CapsuleGeometry(
            shape.capsule.radius,
            shape.capsule.half_height * 2,
            8,
            16,
          ),
          new THREE.MeshStandardMaterial({ color: 0xcc88ff }),
        ),
        isPlane: false,
      };
    }
    if (shape && "compound" in shape) {
      // **残タスク完遂の縦串⑤前後で追加**。`THREE.Mesh`型のまま複数の子
      // メッシュを`.add()`で載せる「空ジオメトリの入れ物」トリック
      // ——`addSpawnedMesh`/`bodyMeshes`が`THREE.Mesh`型を前提にしている
      // ため(`Object3D`へ緩めるより、Meshのままの方が既存コードへの
      // 影響が小さい)。各子はローカルのposition/rotationで`carrier`の
      // 子として配置し、`carrier`自体の位置/姿勢を毎フレーム同期すれば
      // 全体が追従する(通常のメッシュ同期経路と同じ)。
      const carrier = new THREE.Mesh(new THREE.BufferGeometry());
      for (const child of shape.compound.children) {
        const { mesh: childMesh, isPlane: childIsPlane } = meshFromShapeJson(
          child.shape,
        );
        if (childIsPlane) {
          console.warn("Compoundの子にPlaneは対象外——無視します。");
          continue;
        }
        const [px, py, pz] = child.position ?? [0, 0, 0];
        childMesh.position.set(px, py, pz);
        if (child.rotation) {
          const [qx, qy, qz, qw] = child.rotation;
          childMesh.quaternion.set(qx, qy, qz, qw);
        }
        carrier.add(childMesh);
      }
      return { mesh: carrier, isPlane: false };
    }
    if (shape && "convex_mesh" in shape) {
      // **残タスク完遂の縦串⑤前後で追加**。物理側(`sim_mechanics::Shape::
      // ConvexMesh`)は面情報を持たず接触判定も`None`(すり抜け、既知の
      // 限界)のままだが、描画は`ConvexGeometry`(3点以上の点群から凸包を
      // 計算する、`three/examples/jsm`)で頂点の見た目上の凸包を描ける
      // ——Rust側の物理コアには触れない、フロントエンドの描画専用の対応。
      const points = shape.convex_mesh.vertices.map(
        ([x, y, z]) => new THREE.Vector3(x, y, z),
      );
      if (points.length < 4) {
        console.warn(
          `ConvexMesh: 頂点が${points.length}個(凸包の計算には4個以上が要る)——0.3mの球で代用します。`,
        );
      } else {
        return {
          mesh: new THREE.Mesh(
            new ConvexGeometry(points),
            new THREE.MeshStandardMaterial({ color: 0x88ccff }),
          ),
          isPlane: false,
        };
      }
    }
    if (shape && "mesh" in shape) {
      // **D1(スケッチ・押し出し)で追加**。`convex_mesh`と違い面情報を
      // 持っているので、凸包を計算し直さず**三角形をそのまま描く**
      // ——スケッチで作った切り欠きや穴が見た目にも残る(凸包で描くと
      // 埋まってしまい、物理(近似凸分解)と見た目が食い違う)。
      const { vertices, triangles } = shape.mesh;
      const positions = new Float32Array(triangles.length * 9);
      let w = 0;
      for (const [a, b, c] of triangles) {
        for (const vi of [a, b, c]) {
          const v = vertices[vi] ?? [0, 0, 0];
          positions[w++] = v[0];
          positions[w++] = v[1];
          positions[w++] = v[2];
        }
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      // インデックスを展開して三角形ごとに頂点を複製してあるので、
      // 面法線がそのまま出る(フラットシェーディングで角が立つ)。
      geometry.computeVertexNormals();
      return {
        mesh: new THREE.Mesh(
          geometry,
          new THREE.MeshStandardMaterial({
            color: 0xffcc66,
            flatShading: true,
          }),
        ),
        isPlane: false,
      };
    }
    console.warn(
      `未知の形状(${shape ? Object.keys(shape).join(",") : "undefined"})を検出——0.3mの球で代用します。`,
    );
    return {
      mesh: new THREE.Mesh(
        new THREE.SphereGeometry(0.3, 16, 12),
        new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
      ),
      isPlane: false,
    };
  }

  sceneImportRef.current = (json: string): SceneImportResult => {
    const count = world.import_scene_json(json);
    // QA不具合5: Import が取り込まなかったセクションを申告させる
    // (`skipped_import_sections`のdoc参照)。黙って落とすのをやめる。
    const skipped = JSON.parse(
      world.read_component("last_import_skipped_sections", ""),
    ) as string[];
    if (skipped.length > 0) {
      console.warn(
        `シーンJSON Import: ${skipped.join(" / ")} セクションは取り込まれませんでした` +
          `(Import が扱うのは materials / bodies / probes のみです)。` +
          `これらを含めて読み込むにはツールバーの Scene ギャラリーから開いてください。`,
      );
    }
    const parsed = JSON.parse(json) as ImportedScenarioJson;
    const bodies = parsed.bodies ?? [];
    currentPredictionPrompts = parsed.prediction_prompts ?? [];
    renderPredictionPanel();
    const total = readNumber(world, "body_count");
    const startIndex = total - count;

    for (let i = 0; i < count; i++) {
      const bodyIndex = startIndex + i;
      const { mesh, isPlane } = meshFromShapeJson(bodies[i]?.shape);
      if (isPlane) {
        addSpawnedMesh(bodyIndex, mesh);
        continue;
      }
      const pos = world.body_position_at_f32(bodyIndex);
      mesh.position.set(pos[0], pos[1], pos[2]);
      const rot = world.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(rot[0], rot[1], rot[2], rot[3]);
      addSpawnedMesh(bodyIndex, mesh);
    }
    return { count, skipped };
  };

  // シーンギャラリー(`SceneGalleryRef`のdoc参照)。Importと異なり`world`
  // そのものを`WasmWorld.from_scene_json`で差し替える——D9/D34のような
  // 非力学シーンも`fluids`/`thermal`/`circuit`/`astro`セクション込みで
  // 正しく構成されるようにするため(`import_scene_json`は`fluids`/`probes`
  // 以外を見ない設計上の非対称、モジュール冒頭のdoc参照)。
  //
  // 差し替えに伴い、旧`world`に紐づいていた以下の状態を掃除する(**残タスク
  // 完遂の増分**で発見した、旧worldのボディ/フレーム数を前提にした箇所):
  // - `bodyMeshes`/`pickables`/`constraintLines`: 旧ボディのメッシュ・拘束線。
  // - `motorArmBodies`/`currentMotorTarget`/`currentScale`: 旧ボディのindex
  //   キーが新worldでは無関係な(あるいは存在しない)ボディを指してしまう。
  // - `frameAxesHelpers`/`selectedFrameIndex`: 起動時に追加した既定の回転
  //   フレーム(`initialFrameIndex`)は新worldには存在しないため、参照すると
  //   `frame_world_position_f32`がJS例外を投げてrender()ループが壊れる
  //   ——ROOT(常に存在するindex 0)へリセットする。ギャラリーシーンの
  //   フレーム階層自体は現状スコープ外(正直な限界として記録)。
  // - `fluidPositionAttribute`: 旧worldで流体をスポーンしていた場合の粒子
  //   バッファ、新worldの流体粒子数と食い違うと壊れるため無効化する。
  // - `editUndoStack`/`commandLog`: 旧ボディのindexを記録しているため無効化。
  // - `circuitFreeWiringState.active`: 現状のギャラリーシーン(D4/D5/D6/D11)は
  //   いずれも回路を持たないため、固定デモ回路のスイッチ(`circuit_switch_index`
  //   はプレースホルダ値0、`from_scene_json`のdoc参照)を無効化する
  //   (自由配線回路エディタと同じ保護、`circuitFreeWiringState`のdoc参照)。
  // - Console(QA不具合4): クリアしないと前シーンの接触ログ(存在しない
  //   ボディindexを指す)が残り続け、クリックすると無関係なボディが
  //   選択されてしまう。
  sceneGalleryRef.current = (json: string) => {
    clearConsole();
    const parsed = JSON.parse(json) as ImportedScenarioJson;
    const bodies = parsed.bodies ?? [];

    for (const mesh of bodyMeshes.values()) {
      scene.remove(mesh);
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    }
    bodyMeshes.clear();
    for (const line of constraintLines.values()) {
      scene.remove(line);
    }
    constraintLines.clear();
    for (const helper of frameAxesHelpers.values()) {
      scene.remove(helper);
    }
    frameAxesHelpers.clear();
    selectedFrameIndex = 0; // ROOT。ギャラリーシーンのフレーム階層は対象外。
    pickables = [];
    motorArmBodies.clear();
    currentMotorTarget.clear();
    currentScale.clear();
    currentScaleXyz.clear();
    editUndoStack.length = 0;
    commandLog.length = 0;
    fluidPositionAttribute = null;
    fluidPoints.visible = false;
    circuitFreeWiringState.active = true;
    circuitSwitchToggle.disabled = true;

    world = WasmWorld.from_scene_json(json);
    isGalleryScene = true;
    // **群3で追加したドメインの描画用キャッシュを張り直す**。
    // 拘束ペアはシーン読み込み時にしか変わらないので毎フレーム取り直さない。
    softBodyConstraintPairs = world.soft_body_constraint_pairs_u32();
    // 気体の箱は原点を中心に描きたいので、箱の中心を粒子座標から推定する
    // (箱サイズは `Scenario` にしかないため、初期粒子分布の重心で代用する)。
    const gasPositions = world.kinetic_gas_positions_f32(1);
    if (gasPositions.length >= 3) {
      let sx = 0;
      let sy = 0;
      let sz = 0;
      const n = gasPositions.length / 3;
      for (let i = 0; i < n; i += 1) {
        sx += gasPositions[i * 3];
        sy += gasPositions[i * 3 + 1];
        sz += gasPositions[i * 3 + 2];
      }
      gasBoxCenter = [sx / n, sy / n, sz / n];
    } else {
      gasBoxCenter = [0, 0, 0];
    }
    // オーバーレイを一度描いてからカメラを合わせる(そうしないと
    // ソフトボディ/天体/粒子群のバウンディングボックスがまだ空)。
    updateSoftBodyOverlay(world);
    updateAstroOverlay(world);
    updateParticleCloud(gasCloud, world.kinetic_gas_positions_f32(1), gasBoxCenter);
    updateParticleCloud(brownianCloud, world.brownian_positions_f32(1), [0, 0, 0]);
    sceneBaseBodyCount = bodies.length;

    currentPredictionPrompts = parsed.prediction_prompts ?? [];
    renderPredictionPanel();

    for (let bodyIndex = 0; bodyIndex < bodies.length; bodyIndex++) {
      const { mesh, isPlane } = meshFromShapeJson(bodies[bodyIndex]?.shape);
      if (isPlane) {
        addSpawnedMesh(bodyIndex, mesh);
        continue;
      }
      const pos = world.body_position_at_f32(bodyIndex);
      mesh.position.set(pos[0], pos[1], pos[2]);
      const rot = world.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(rot[0], rot[1], rot[2], rot[3]);
      addSpawnedMesh(bodyIndex, mesh);

      if (world.constraint_anchor_points_at(bodyIndex).length >= 6) {
        const lineGeometry = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(),
          new THREE.Vector3(),
        ]);
        const line = new THREE.Line(
          lineGeometry,
          new THREE.LineBasicMaterial({ color: 0xffaa00 }),
        );
        scene.add(line);
        constraintLines.set(bodyIndex, line);
      }
    }
    // **2026-07-28のD9/D34/D35増分で追加したガード**: D9(熱のみ)・D34/D35
    // (天体のみ)は力学ボディを1つも持たないため、上のループが1回も回らず
    // `addSpawnedMesh`(内部で`selectBody`を呼ぶ)も一度も呼ばれない——
    // 無条件に`selectBody(0)`していた旧実装は、この場合ボディindex 0が
    // 存在しないままInspectorのTransform読み出し(`renderInspectorFor`→
    // `body_label_at`等)・毎フレームの`render()`(`body_position_at_f32`等)
    // へ渡り、いずれもJS例外を投げてUIが壊れる(HUD更新も止まる)——
    // ボディが無ければHierarchyだけ空の状態へ更新し、選択は行わない。
    if (readNumber(world, "body_count") > 0) {
      selectBody(0);
    } else {
      selectedBodyIndex = -1; // 有効なボディ無し(`hasSelectedBody`参照)。
      highlightHierarchy = rebuildHierarchy();
      renderInspectorFor(world, selectedBodyIndex);
    }
    // **シーンの中身にカメラを合わせる(群3、`frameCameraOnContent`のdoc参照)**。
    // 剛体・ソフトボディ・天体・粒子群がすべて配置し終わった後に呼ぶ。
    frameCameraOnContent();
  };

  // Replay再生実行(`ReplayVerifyRef`のdoc参照)。記録済み`commandLog`を、
  // 既定シーン(床+箱のみ)を持つ新規`WasmWorld`へステップ番号どおりに再送する。
  // Grab/Release/ApplyForce/SetSwitch/SetHeatSourceはWasmWorldのコンストラクタが
  // 必ず用意する固定ボディ/回路/熱ノードが対象なので常に再現できるが、
  // SetMotorTarget(スポーンしたモーターアームが対象)は新規Worldにその
  // ボディが存在しないため`bodyIndex`が範囲外なら無視する(縮約実装、既知の
  // 限定——`sceneChanged`で呼び出し側に伝える)。MoveGrab(ドラッグ中の連続更新)
  // は元々記録していないため、再生されるのはGrabの初期アンカー位置のみ。
  /// 記録済みコマンドを step 番号で引けるようにまとめ直す(検証実行と
  /// **ライブ再生**の両方が使う、群2で共通化)。
  function groupCommandsByStep(): Map<number, CommandLogEntry[]> {
    const commandsByStep = new Map<number, CommandLogEntry[]>();
    for (const entry of commandLog) {
      const list = commandsByStep.get(entry.step) ?? [];
      list.push(entry);
      commandsByStep.set(entry.step, list);
    }
    return commandsByStep;
  }

  /// ヒーターは「1step分だけ効く」縮約セマンティクスなので、再生側でも
  /// 状態として持ち回って毎step再送する必要がある(`HEATER_WATTS`のdoc参照)。
  type ReplayHeaterState = { on: boolean; watts: number };

  /// 1 step 分のコマンドを再生用ワールドへ適用する(検証実行とライブ再生で共有)。
  function applyReplayCommands(
    replayWorld: WasmWorld,
    entries: CommandLogEntry[],
    heater: ReplayHeaterState,
  ): void {
    for (const entry of entries) {
      switch (entry.kind) {
        case "Grab":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "push_grab", {
              body_index: entry.bodyIndex,
              target_x: entry.targetX,
              target_y: entry.targetY,
              target_z: entry.targetZ,
            });
          }
          break;
        case "Release":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "push_release", { body_index: entry.bodyIndex });
          }
          break;
        case "ApplyForce":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "push_apply_force", {
              body_index: entry.bodyIndex,
              fx: entry.fx,
              fy: entry.fy,
              fz: entry.fz,
            });
          }
          break;
        case "SetMotorTarget":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "set_motor_target_at", {
              index: entry.bodyIndex,
              theta_target: entry.targetAngle,
            });
          }
          break;
        case "SetSwitch":
          applyComponent(replayWorld, "set_circuit_switch_closed", { closed: entry.closed });
          break;
        case "SetHeatSource":
          heater.on = entry.on;
          heater.watts = entry.watts;
          break;
        // **群2で追加した入力も再現する**。これらを再生しないと、重力を
        // 変えた実行のリプレイが「重力9.807のまま」進んで state_hash が
        // 一致しなくなる(記録しているのに再生しないのが一番たちが悪い)。
        case "SetGravity":
          applyComponent(replayWorld, "set_gravity", { gravity: entry.gravity });
          break;
        case "SetGravityDirection":
          applyComponent(replayWorld, "set_gravity_direction", {
            x: entry.x,
            y: entry.y,
            z: entry.z,
          });
          break;
        case "SetDt":
          applyComponent(replayWorld, "set_dt", { dt: entry.dt });
          break;
        case "SetBodyMass":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "push_set_body_mass", {
              body_index: entry.bodyIndex,
              mass: entry.mass,
            });
          }
          break;
        case "SetBodyType":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "push_set_body_type", {
              body_index: entry.bodyIndex,
              kind: entry.bodyType,
            });
          }
          break;
        case "SetCollisionFilter":
          if (entry.bodyIndex < readNumber(replayWorld, "body_count")) {
            applyComponent(replayWorld, "push_set_collision_filter", {
              body_index: entry.bodyIndex,
              group: entry.group,
              mask: entry.mask,
            });
          }
          break;
      }
    }
  }

  replayVerifyRef.current = () => {
    const replayWorld = new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT);
    const totalSteps = readNumber(world, "step_count");
    const sceneChanged = readNumber(world, "body_count") !== 2;
    const commandsByStep = groupCommandsByStep();
    const heater: ReplayHeaterState = { on: false, watts: 0 };
    for (let s = 0; s < totalSteps; s++) {
      applyReplayCommands(replayWorld, commandsByStep.get(s) ?? [], heater);
      if (heater.on) applyComponent(replayWorld, "push_heat_source", { watts: heater.watts });
      replayWorld.step();
    }

    // `body_position_at_f32`はWasmメモリを直接指す一時的なビューを返す(B16、
    // `HotPathViewBuffers`のdoc参照)。しかも`replayWorld`と`world`は別々の
    // `WasmWorld`インスタンスでも同じ1個のWasmモジュールインスタンス(=同じ
    // 線形メモリ)を共有しているため、どちらか一方への後続呼び出しがもう一方の
    // ビューをも無効化しうる。この関数はこの後`world`/`replayWorld`双方へ
    // 何度も呼び出すので、ここで即座にプレーンな配列へ読み切っておく。
    const finalBoxPos = Array.from(replayWorld.body_position_at_f32(BODY_INDEX_BOX));
    // ライブ側の`world`はシーンギャラリー経由で任意のシーンに差し替わっている
    // 可能性があり、index 1(既定シーンの箱)が存在しないことがある——`sceneChanged`
    // が真の時点で`matches`は`false`確定なので、位置の意味自体が無いプレース
    // ホルダで安全に済ませる(**残タスク完遂のシーンギャラリー増分**で追加した
    // ガード、以前は既定シーン以外あり得なかったため無条件アクセスで安全だった)。
    const liveBoxPos = sceneChanged
      ? [0, 0, 0]
      : Array.from(world.body_position_at_f32(BODY_INDEX_BOX));
    const finalStateHash = replayWorld.read_component("state_hash", "");
    const liveStateHash = world.read_component("state_hash", "");
    return {
      totalSteps,
      commandCount: commandLog.length,
      sceneChanged,
      finalStateHash,
      finalBoxPosition: [finalBoxPos[0], finalBoxPos[1], finalBoxPos[2]],
      liveStateHash,
      liveBoxPosition: [liveBoxPos[0], liveBoxPos[1], liveBoxPos[2]],
      matches: !sceneChanged && finalStateHash === liveStateHash,
    };
  };

  // **Replay のライブ再生(群2)**。設計 docs/23-frontend/01-editor.md §1.6
  // 「Replays: 記録した入力列の再生」。これまでの Replay は**ヘッドレスで
  // 一気に流して最終 state_hash を比べるだけ**で、記録した操作を「見る」
  // 手段が無かった(検証としては正しいが、再生とは呼べない)。
  //
  // ライブ再生は**現在の world を壊さない**——別の `WasmWorld` を作って
  // そこを step し、その位置を Scene View のメッシュへ流し込む。再生が
  // 終われば(または中断すれば)メッシュは次フレームから現在の world の
  // 位置へ戻る(`render()` が毎フレーム同期するため、後片付けが要らない)。
  //
  // **既定シーン(床+箱)以外では再生できない**——記録済み command は
  // 既定シーンの body index を前提にしており、別シーンへ流し込むと
  // 無関係なボディが動く。`sceneChanged` と同じ判定で弾く。
  type LivePlayback = {
    replayWorld: WasmWorld;
    commandsByStep: Map<number, CommandLogEntry[]>;
    heater: ReplayHeaterState;
    step: number;
    totalSteps: number;
    accumulator: number;
  };
  let livePlayback: LivePlayback | null = null;

  replayPlaybackRef.current = {
    start: () => {
      if (readNumber(world, "body_count") !== 2) {
        return { started: false, reason: "既定シーン(床+箱)でのみライブ再生できます。" };
      }
      const totalSteps = readNumber(world, "step_count");
      if (totalSteps === 0) {
        return { started: false, reason: "再生する step がありません(まず Play で進めてください)。" };
      }
      livePlayback = {
        replayWorld: new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT),
        commandsByStep: groupCommandsByStep(),
        heater: { on: false, watts: 0 },
        step: 0,
        totalSteps,
        accumulator: 0,
      };
      return { started: true, totalSteps };
    },
    stop: () => {
      livePlayback = null;
    },
    isPlaying: () => livePlayback !== null,
    progress: () => (livePlayback ? livePlayback.step / livePlayback.totalSteps : 1),
  };

  /// ライブ再生を実時間ぶんだけ進め、再生用ワールドの位置を Scene View の
  /// メッシュへ反映する。`frame()` から毎フレーム呼ぶ。
  function advanceLivePlayback(frameSeconds: number): boolean {
    const playback = livePlayback;
    if (!playback) return false;
    const dt = readNumber(playback.replayWorld, "dt");
    playback.accumulator += frameSeconds * timeScale;
    let steps = 0;
    while (
      playback.accumulator >= dt &&
      steps < MAX_STEPS_PER_FRAME &&
      playback.step < playback.totalSteps
    ) {
      applyReplayCommands(
        playback.replayWorld,
        playback.commandsByStep.get(playback.step) ?? [],
        playback.heater,
      );
      if (playback.heater.on)
        applyComponent(playback.replayWorld, "push_heat_source", {
          watts: playback.heater.watts,
        });
      playback.replayWorld.step();
      playback.accumulator -= dt;
      playback.step += 1;
      steps += 1;
    }
    // 再生用ワールドのボディ位置をメッシュへ流し込む(床は Plane なので除く、
    // `render()` の同期と同じ理由)。
    for (const [bodyIndex, mesh] of bodyMeshes) {
      if (bodyIndex >= readNumber(playback.replayWorld, "body_count")) continue;
      if (playback.replayWorld.read_component("body_shape_kind_at", String(bodyIndex)) === "plane") continue;
      const p = playback.replayWorld.body_position_at_f32(bodyIndex);
      mesh.position.set(p[0], p[1], p[2]);
      const r = playback.replayWorld.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(r[0], r[1], r[2], r[3]);
    }
    if (playback.step >= playback.totalSteps) livePlayback = null;
    return true;
  }

  // Prefabs(`PrefabDefinition`のdoc参照)。**無限平面(床)を除く5形状すべてに
  // 対応**——キャプチャは`body_shape_json_at`、再スポーンは`spawn_shape_json`
  // (任意の`ShapeJson`をそのまま配置する汎用スポナー)という無損失な対に
  // 載っている。
  //
  // 以前は球/箱だけだった。`body_shape_params_f64_at`(平坦なf64配列)で
  // 寸法を読み、`spawn_sphere`/`spawn_box`という固定レシピのスポナーで戻す
  // 作りだったため、①Compound/ConvexMeshは配列で表現できず`captureBody`が
  // `null`を返し(ユーザーから見れば「Prefab化を押しても何も起きない」)、
  // ②箱も`spawn_box`が単一`half_extent`しか取らないので非立方体は立方体に
  // 潰れる、という2つの欠落があった。どちらも形状の表現力の問題なので、
  // 表現力で劣る経路を残さず`ShapeJson`1本へ寄せて解消した。
  prefabRef.current = {
    captureSelectedBody: () => {
      // ボディが無いシーン(D9/D34/D35のようなギャラリーシーン)で選択が
      // 無効なら何もキャプチャできない(`hasSelectedBody`のdoc参照)。
      if (!hasSelectedBody()) return null;
      return prefabRef.current!.captureBody(selectedBodyIndex);
    },
    captureBody: (index) => {
      if (index < 0 || index >= readNumber(world, "body_count")) return null;
      const kind = world.read_component("body_shape_kind_at", String(index));
      // Planeだけは対象外——無限平面を増やしても物理的に意味が無く、位置も
      // `normal`/`d`から決まるので「スポーン位置に置く」という操作自体が
      // 成り立たない(`duplicate()`が床を弾くのと同じ理由)。wasm側の
      // `spawn_shape_json`はPlaneも受け付けるが、そこで狭めず**UI側の判断**
      // として持つ(`spawn_shape_json_impl`のdoc参照)。
      if (kind === "plane") return null;
      const shape = JSON.parse(
        world.read_component("body_shape_json_at", String(index)),
      ) as ImportedShapeJson;
      const material = world.read_component("body_material_label_at", String(index));
      return { kind, shape, material };
    },
    spawn: (prefab) => {
      const { x, z } = nextSpawnPosition();
      const bodyIndex = applyComponent(world, "spawn_shape_json", {
        shape_json: JSON.stringify(prefab.shape),
        x,
        y: SPAWN_HEIGHT,
        z,
        material_name: prefab.material,
      }).index as number;
      // 見た目はシーンJSON import・複製と同じ`meshFromShapeJson`で組む
      // ——形状ごとのTHREE.Geometry組み立てをここに再掲しない。
      addSpawnedMesh(bodyIndex, meshFromShapeJson(prefab.shape).mesh);
    },
  };

  // **カプセルのスポーン(増分L)**。`sim-mechanics`側で体積・慣性・接触
  // (平面/球/カプセル)を実装したので、床へ落として寝かせるところまで動く。
  // **カプセル×箱の接触は未実装**なので箱と並べてもすり抜ける(パニックは
  // しない)——ボタンのtitleにもその制約を書いてある。
  const SPAWN_CAPSULE_RADIUS = 0.2;
  const SPAWN_CAPSULE_HALF_HEIGHT = 0.35;

  // **複合形状(L字)・凸包メッシュ(立方体)のスポーン**。`spawn_compound_l_shape`/
  // `spawn_convex_mesh_cube`(Rust側)が返す既定形状と寸法を合わせる
  // ——見た目(`meshFromShapeJson`が受け取る`ImportedShapeJson`)と物理の
  // 形状記述を二重管理しないよう、ここでも同じ`ImportedShapeJson`を組み立てて
  // `meshFromShapeJson`を再利用する(独自のTHREE.Geometry組み立てを増やさない)。
  const SPAWN_COMPOUND_L_SHAPE_JSON: ImportedShapeJson = {
    compound: {
      children: [
        {
          position: [0, 0.75, 0],
          shape: { box: { half: [0.25, 1.0, 0.25] } },
        },
        {
          position: [0.25, -0.25, 0],
          shape: { box: { half: [0.5, 0.25, 0.25] } },
        },
      ],
    },
  };
  // L字の最下点(横棒の下端): y=-0.25の中心からhalf_extents.y=0.25分下。
  const SPAWN_COMPOUND_L_SHAPE_REST_OFFSET = 0.5;
  const SPAWN_CONVEX_MESH_HALF = 0.3;
  function convexMeshCubeShapeJson(half: number): ImportedShapeJson {
    const vertices: [number, number, number][] = [];
    for (const sx of [-1, 1]) {
      for (const sy of [-1, 1]) {
        for (const sz of [-1, 1]) {
          vertices.push([sx * half, sy * half, sz * half]);
        }
      }
    }
    return { convex_mesh: { vertices } };
  }

  /// スポーンパレットが扱う形状(設計 §1.2「右クリックでコンテキストメニュー…
  /// スポーンパレット(形状×材質を選んで**クリック位置に配置**)」)。
  /// ツールバーのボタンも右クリックメニューも**この1関数を共有する**ので、
  /// 「ボタンからだと動くがメニューからだと動かない」という乖離が起きない。
  /// **`compound`/`convex_mesh`は残タスク完遂の縦串⑤前後で追加**——レビュー
  /// 指摘(「UIから作る経路がないから」を許容せず作る前提で進める)への対応。
  type SpawnShapeKind =
    | "sphere"
    | "box"
    | "capsule"
    | "compound"
    | "convex_mesh";
  function spawnShapeAt(
    kind: SpawnShapeKind,
    x: number,
    y: number,
    z: number,
  ): number {
    const material = spawnMaterialSelect.value;
    let bodyIndex: number;
    let mesh: THREE.Mesh;
    switch (kind) {
      case "sphere":
        bodyIndex = applyComponent(world, "spawn_sphere", {
          x,
          y,
          z,
          radius: SPAWN_SPHERE_RADIUS,
          material_name: material,
        }).index as number;
        mesh = new THREE.Mesh(
          new THREE.SphereGeometry(SPAWN_SPHERE_RADIUS, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0x6699ff }),
        );
        break;
      case "box":
        bodyIndex = applyComponent(world, "spawn_box", {
          x,
          y,
          z,
          half_extent: SPAWN_BOX_HALF_EXTENT,
          material_name: material,
        }).index as number;
        mesh = new THREE.Mesh(
          new THREE.BoxGeometry(
            SPAWN_BOX_HALF_EXTENT * 2,
            SPAWN_BOX_HALF_EXTENT * 2,
            SPAWN_BOX_HALF_EXTENT * 2,
          ),
          new THREE.MeshStandardMaterial({ color: 0x66cc66 }),
        );
        break;
      case "capsule":
        bodyIndex = applyComponent(world, "spawn_capsule", {
          x,
          y,
          z,
          radius: SPAWN_CAPSULE_RADIUS,
          half_height: SPAWN_CAPSULE_HALF_HEIGHT,
          material_name: material,
        }).index as number;
        // THREE.CapsuleGeometry の length は「円柱部の長さ」= 2*half_height。
        mesh = new THREE.Mesh(
          new THREE.CapsuleGeometry(
            SPAWN_CAPSULE_RADIUS,
            SPAWN_CAPSULE_HALF_HEIGHT * 2,
            8,
            16,
          ),
          new THREE.MeshStandardMaterial({ color: 0xcc88ff }),
        );
        break;
      case "compound":
        bodyIndex = applyComponent(world, "spawn_compound_l_shape", {
          x,
          y,
          z,
          material_name: material,
        }).index as number;
        mesh = meshFromShapeJson(SPAWN_COMPOUND_L_SHAPE_JSON).mesh;
        break;
      case "convex_mesh":
        bodyIndex = applyComponent(world, "spawn_convex_mesh_cube", {
          x,
          y,
          z,
          half: SPAWN_CONVEX_MESH_HALF,
          material_name: material,
        }).index as number;
        mesh = meshFromShapeJson(
          convexMeshCubeShapeJson(SPAWN_CONVEX_MESH_HALF),
        ).mesh;
        break;
    }
    addSpawnedMesh(bodyIndex, mesh);
    return bodyIndex;
  }

  for (const [id, kind] of [
    ["btn-spawn-sphere", "sphere"],
    ["btn-spawn-box", "box"],
    ["btn-spawn-capsule", "capsule"],
    ["btn-spawn-compound", "compound"],
    ["btn-spawn-convex-mesh", "convex_mesh"],
  ] as [string, SpawnShapeKind][]) {
    document.getElementById(id)!.addEventListener("click", () => {
      const { x, z } = nextSpawnPosition();
      spawnShapeAt(kind, x, SPAWN_HEIGHT, z);
    });
  }

  // **Hierarchy 右クリックメニューの実体(群2、`HierarchyActions`のdoc参照)**。
  // ここまで来て初めて `spawnShapeAt`・`bodyMeshes`・`prefabRef` が揃う。
  let isolatedBodyIndex: number | null = null;
  hierarchyActionsRef.current = {
    duplicate(index) {
      if (index < 0 || index >= readNumber(world, "body_count")) return;
      // 無限平面(床)は複製しない——2枚重ねても物理的に意味が無く、
      // 対応するメッシュも作れない(Plane の見た目は `normal`/`d` から
      // 別経路で作っている)。
      if (world.read_component("body_shape_kind_at", String(index)) === "plane") return;
      // Rust 側が形状・材質・位置を複製する(`duplicate_body_at`)。
      // フロント側は対応するメッシュを作るだけ——**形状の種類は Rust から
      // 読み直す**(元メッシュを `clone()` すると、Scale Gizmo で寸法を
      // 変えたボディで見た目と物理がずれる)。
      const newIndex = applyComponent(world, "duplicate_body_at", {
        index,
        offset: DUPLICATE_OFFSET_M,
      }).index as number;
      // **形状の読み直しは`body_shape_json_at`1本**。以前はsphere/box/capsuleを
      // `body_shape_params_f64_at`(平坦なf64配列)から手組みし、その配列で
      // 表現できないcompound/convex_meshだけ`body_shape_json_at`へ落とす、
      // という**形状の種類ごとに分かれた2経路**だった。同じ「複製後の実形状から
      // メッシュを作る」処理が2通りある必然性は無く(`meshFromShapeJson`は
      // 元から6形状すべてを描ける)、種類ごとの分岐ごと畳んだ。
      const shapeJson = JSON.parse(
        world.read_component("body_shape_json_at", String(newIndex)),
      ) as ImportedShapeJson;
      addSpawnedMesh(newIndex, meshFromShapeJson(shapeJson).mesh);
    },
    remove(index) {
      applyComponent(world, "remove_body_at", { index });
      // メッシュ・ピック対象・オーバーレイから外す(残すと y=-1e9 の
      // 退避先へ飛んだメッシュが毎フレーム同期され続ける)。
      const mesh = bodyMeshes.get(index);
      if (mesh) {
        scene.remove(mesh);
        bodyMeshes.delete(index);
      }
      const pickIndex = pickables.findIndex((p) => p.bodyIndex === index);
      if (pickIndex >= 0) pickables.splice(pickIndex, 1);
      hierarchyMultiSelection.delete(index);
      if (selectedBodyIndex === index) {
        // QA不具合6続き(Deleteキー対応中に発見した実バグ): `BODY_INDEX_BOX`を
        // 無条件のフォールバック選択先にしていたが、削除対象そのものが
        // `BODY_INDEX_BOX`だった場合(既定シーンの箱を削除する等)、
        // 存在しないボディを`selectBody`してしまい、Inspector/HUDの
        // 読み出しが例外を投げてrender()ループが壊れる。削除対象と別なら
        // `BODY_INDEX_BOX`、それも無理なら床(常に存在する`BODY_INDEX_GROUND`)
        // へ落とす。
        const boxAlive =
          index !== BODY_INDEX_BOX &&
          BODY_INDEX_BOX < readNumber(world, "body_count") &&
          !(world.read_component("body_is_removed_at", String(BODY_INDEX_BOX)) === "true");
        selectBody(boxAlive ? BODY_INDEX_BOX : BODY_INDEX_GROUND);
      }
      highlightHierarchy = rebuildHierarchy();
    },
    isolate(index) {
      isolatedBodyIndex = index;
      // **物理は止めない**——見えないだけで計算は続く(Unity の Isolate と同じ)。
      for (const [bodyIndex, mesh] of bodyMeshes) {
        mesh.visible = index === null || bodyIndex === index;
      }
      highlightHierarchy = rebuildHierarchy();
    },
    isolatedIndex: () => isolatedBodyIndex,
    capturePrefab(index) {
      const captured = prefabRef.current?.captureBody(index);
      if (!captured) {
        // 残る唯一の非対応は無限平面(床、`captureBody`のdoc参照)。
        reportError("この形状はPrefab化できません(無限平面(床)は対象外)");
        return;
      }
      prefabSaveRef.current?.({
        name: `${world.read_component("body_label_at", String(index))}_prefab`,
        ...captured,
      });
    },
  };

  // **「＋ 追加」メニュー(群2)**。8個のスポーンボタンを直接並べていたため
  // ツールバーが3行ぶんの高さに膨れ、ラベルが1文字ずつ縦に折り返して
  // 読めなくなっていた(実際にスクリーンショットで確認)。1つのメニューへ畳む。
  // **既存のボタンは `hidden` で DOM に残してある**——それぞれのクリック
  // ハンドラ(振り子・モーター・流体・フレーム・材料派生はここより後で
  // 配線される)をそのまま再利用でき、既存のテストも壊れないため。
  document.getElementById("btn-add")?.addEventListener("click", (event) => {
    const clickHidden = (id: string) => () => document.getElementById(id)?.click();
    const target = event.currentTarget as HTMLElement;
    const rect = target.getBoundingClientRect();
    showContextMenu(rect.left, rect.bottom, [
      { label: "＋ 球", onSelect: clickHidden("btn-spawn-sphere") },
      { label: "＋ 箱", onSelect: clickHidden("btn-spawn-box") },
      {
        label: "＋ カプセル",
        onSelect: clickHidden("btn-spawn-capsule"),
        title: "カプセル×箱の接触は未実装(箱とはすり抜けます)",
      },
      {
        label: "＋ 複合形状 (L字)",
        onSelect: clickHidden("btn-spawn-compound"),
        title: "Shape::Compound(Box×2の子)",
      },
      {
        label: "＋ 凸包メッシュ",
        onSelect: clickHidden("btn-spawn-convex-mesh"),
        title: "Shape::ConvexMesh(立方体の8頂点)。接触判定は未実装(すり抜けます)",
      },
      { separator: true },
      { label: "＋ 振り子 (DistanceJoint)", onSelect: clickHidden("btn-spawn-pendulum") },
      { label: "＋ モーター (BallJoint + HingeMotorPd)", onSelect: clickHidden("btn-spawn-motor") },
      { label: "＋ 流体 (SPH 水塊)", onSelect: clickHidden("btn-spawn-fluid") },
      { separator: true },
      {
        label: "＋ フレーム",
        onSelect: clickHidden("btn-add-frame"),
        title: "Hierarchy で選択中のフレーム(未選択なら ROOT)の子として追加",
      },
      {
        label: "材料派生",
        onSelect: clickHidden("btn-derive-material"),
        title: "選択中の材質から密度違いの派生材料を作る",
      },
    ]);
  });

  // **Scene View のスポーンパレット(群2)**。設計 §1.2 が求める
  // 「右クリックでコンテキストメニュー…**クリック位置に配置**」。
  //
  // 右ドラッグは OrbitControls のパンに割り当ててあるので、**パンした直後は
  // メニューを出さない**(パンのたびにメニューが開くと操作にならない)。
  // `click` とドラッグを `DRAG_THRESHOLD_PX` で判別している既存のピック処理と
  // 同じ手法で、移動量から判別する。
  //
  // **判定は `pointerup` で行う**(QA不具合 A2-2)。以前は `contextmenu` の
  // 座標と `pointerdown` の座標を比べていたが、Chromium は `contextmenu` を
  // **`pointerdown` の直後**(ボタンを離す前・カーソルが動く前)に発火するため、
  // 移動量は常に 0 で「ドラッグではない」と判定され、**右ボタンを押した瞬間に
  // 必ずパレットが開いていた**。開いたパレットはカーソル直下に残るので、
  // 次の右押しはキャンバスではなくメニュー要素へヒットテストされ、
  // `renderer.domElement` の `pointerdown` が発火しない——結果 OrbitControls が
  // PAN 状態に入らず、**2 回目以降のパンが一切効かなくなっていた**
  // (実測: 連続 3 回のパンで 0.41 → 0.015 → 0.0003 m)。
  // `pointerup` まで待てば移動量が確定するので、ドラッグとクリックを正しく
  // 分けられる。`contextmenu` はブラウザ既定メニューの抑止だけに使う。
  const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const groundHit = new THREE.Vector3();
  let rightDownScreen: { x: number; y: number } | null = null;
  renderer.domElement.addEventListener("pointerdown", (event) => {
    if (event.button === 2)
      rightDownScreen = { x: event.clientX, y: event.clientY };
  });
  // ブラウザ既定のコンテキストメニューは常に抑止する(パレットは下の
  // `pointerup` が出す)。
  renderer.domElement.addEventListener("contextmenu", (event) => {
    event.preventDefault();
  });
  // 右ドラッグが途中で失われたら押し始めの記録も捨てる(離した扱いにしない)。
  renderer.domElement.addEventListener("pointercancel", (event) => {
    if (event.button === 2) rightDownScreen = null;
  });
  renderer.domElement.addEventListener("pointerup", (event) => {
    if (event.button !== 2) return;
    const down = rightDownScreen;
    rightDownScreen = null;
    if (!down) return;
    const moved = Math.hypot(event.clientX - down.x, event.clientY - down.y);
    if (moved > DRAG_THRESHOLD_PX) return; // パンだったのでメニューは出さない。
    // クリック位置を地面(y=0)へ投影する。カメラが地面と平行に近いと交点が
    // 得られないので、その場合は既存の `nextSpawnPosition()` へ落とす。
    const rect = renderer.domElement.getBoundingClientRect();
    pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointerNdc, camera);
    const hasHit =
      raycaster.ray.intersectPlane(groundPlane, groundHit) !== null;
    const fallback = nextSpawnPosition();
    // グリッドスナップを効かせる(Settings で 0 にすれば連続)。
    const x = hasHit ? snapToGrid(groundHit.x) : fallback.x;
    const z = hasHit ? snapToGrid(groundHit.z) : fallback.z;
    const place = (kind: SpawnShapeKind, restHeight: number) => () => {
      // 地面にちょうど乗る高さで置く(めり込ませない/落とさない)。
      const bodyIndex = spawnShapeAt(kind, x, restHeight, z);
      selectBody(bodyIndex);
    };
    showContextMenu(event.clientX, event.clientY, [
      {
        label: `ここに球を配置 (${x.toFixed(2)}, ${z.toFixed(2)})`,
        onSelect: place("sphere", SPAWN_SPHERE_RADIUS),
      },
      {
        label: "ここに箱を配置",
        onSelect: place("box", SPAWN_BOX_HALF_EXTENT),
      },
      {
        label: "ここにカプセルを配置",
        onSelect: place(
          "capsule",
          SPAWN_CAPSULE_RADIUS + SPAWN_CAPSULE_HALF_HEIGHT,
        ),
        title: "カプセル×箱の接触は未実装(箱とはすり抜けます)",
      },
      {
        label: "ここに複合形状(L字)を配置",
        onSelect: place("compound", SPAWN_COMPOUND_L_SHAPE_REST_OFFSET),
        title: "Shape::Compound(Box×2の子)",
      },
      {
        label: "ここに凸包メッシュを配置",
        onSelect: place("convex_mesh", SPAWN_CONVEX_MESH_HALF),
        title: "Shape::ConvexMesh(立方体の8頂点)。接触判定は未実装(すり抜けます)",
      },
      { separator: true },
      {
        label: `材質: ${spawnMaterialSelect.value}`,
        disabled: true,
        onSelect: () => {},
        title: "材質はツールバーの材質セレクタで切り替えます",
      },
    ]);
  });

  // ---------------------------------------------------------------------------
  // **スケッチ → ブーリアン合成 → 押し出し(D1)**
  // ---------------------------------------------------------------------------
  //
  // CADの標準的な作図手順を Scene View に載せる。ユーザーは構築平面に
  // 閉じた多角形(プロファイル)を描き、複数枚を和/差/積で合成し、深さを
  // 与えて押し出す。出来た角柱メッシュは`{"mesh":{vertices,triangles}}`
  // という**新しい形状JSONタグ**になり、既存の汎用スポナー
  // `spawn_shape_json`(=シーンJSONと同じ`shape_json_to_shape`)を通って
  // `Shape::from_triangle_mesh`(近似凸分解)へ流れ、当たり判定・質量特性を
  // 持つ本物の剛体になる。
  //
  // ## 構築平面は「地面 y=0」に固定する
  //
  // 既存のスポーンパレットが右クリック位置を決めるのに使っているのと
  // **同じ平面**(`groundPlane`)・**同じグリッドスナップ**(`snapToGrid`)を
  // 使う。ここを別の平面(カメラ正対など)にすると、同じ Scene View の中で
  // 「クリックした場所」の意味が操作ごとに変わってしまう。
  // 平面の向きをユーザーが変えられるようにするのは対象外(下記「縮約」)。
  //
  // ## 幾何計算はRust側(`sketch_extrude_shape_json`)
  //
  // 多角形ブーリアン・耳刈り・穴のブリッジは数値的に厄介なので、
  // ネイティブの`cargo test`で解析的に固定できるRust側へ置いた
  // (`sim_mechanics::sketch`)。wasm呼び出しは**「押し出し」1回につき1回**
  // だけで、点を置くたびには走らない——作図中のプレビューは点列を線で
  // 結ぶだけなので、TypeScript 側だけで完結する。
  //
  // ## 意図的な縮約
  //
  // - **押し出し方向は+y固定**(構築平面の法線)。傾けたい場合は作った後に
  //   Rotate Gizmo で回す。
  // - **3Dメッシュ同士のブーリアン(CSG)は行わない**。合成は押し出し前の
  //   2D断面に対してのみ(`sim_mechanics::sketch`のモジュールdoc参照)。
  const sketchPanel = document.getElementById("sketch-panel") as HTMLElement;
  const sketchStatus = document.getElementById("sketch-status") as HTMLElement;
  const sketchOpSelect = document.getElementById(
    "select-sketch-op",
  ) as HTMLSelectElement;
  const sketchDepthInput = document.getElementById(
    "input-sketch-depth",
  ) as HTMLInputElement;

  type SketchProfile = { op: string; points: [number, number][] };
  /// 確定済みのプロファイル(先頭が土台、2枚目以降が自分の`op`で効く)。
  const sketchProfiles: SketchProfile[] = [];
  /// 作図中の点列(構築平面上の`[x, z]`)。
  let sketchPoints: [number, number][] = [];

  // プレビュー用のオブジェクト。確定済み(緑)と作図中(黄)を描き分ける。
  const SKETCH_PREVIEW_Y = 0.01; // 地面とZ-fightingしない程度に浮かせる。
  const sketchPreviewGroup = new THREE.Group();
  sketchPreviewGroup.visible = false;
  scene.add(sketchPreviewGroup);

  function clearSketchPreview() {
    for (const child of [...sketchPreviewGroup.children]) {
      sketchPreviewGroup.remove(child);
      const disposable = child as THREE.Line | THREE.Mesh;
      disposable.geometry?.dispose();
    }
  }

  function addSketchOutline(
    points: [number, number][],
    color: number,
    closed: boolean,
  ) {
    if (points.length < 2) return;
    const vertices = points.map(
      ([x, z]) => new THREE.Vector3(x, SKETCH_PREVIEW_Y, z),
    );
    if (closed) vertices.push(vertices[0].clone());
    const geometry = new THREE.BufferGeometry().setFromPoints(vertices);
    sketchPreviewGroup.add(
      new THREE.Line(geometry, new THREE.LineBasicMaterial({ color })),
    );
  }

  /// 頂点の位置を小さな点で示す(線だけだと1点目・2点目が見えない)。
  function addSketchVertexMarkers(points: [number, number][], color: number) {
    if (points.length === 0) return;
    const geometry = new THREE.BufferGeometry().setFromPoints(
      points.map(([x, z]) => new THREE.Vector3(x, SKETCH_PREVIEW_Y, z)),
    );
    sketchPreviewGroup.add(
      new THREE.Points(
        geometry,
        new THREE.PointsMaterial({ color, size: 8, sizeAttenuation: false }),
      ),
    );
  }

  function refreshSketch() {
    clearSketchPreview();
    for (const profile of sketchProfiles) {
      // 減算は赤系、それ以外は緑系——どの枚が何をするのかを色で示す。
      const color = profile.op === "subtract" ? 0xff6655 : 0x55dd88;
      addSketchOutline(profile.points, color, true);
      addSketchVertexMarkers(profile.points, color);
    }
    addSketchOutline(sketchPoints, 0xffdd55, false);
    addSketchVertexMarkers(sketchPoints, 0xffdd55);
    sketchStatus.textContent = `プロファイル ${sketchProfiles.length} 枚 / 作図中の点 ${sketchPoints.length}`;
  }

  /// スケッチツールの出入り(`setGizmoTool`から呼ばれる)。
  sketchToolRef.current = (active: boolean) => {
    sketchPanel.hidden = !active;
    sketchPreviewGroup.visible = active;
    if (active) refreshSketch();
  };

  /// 作図中の点列を1枚のプロファイルとして確定する。3点未満なら何もしない。
  function confirmSketchProfile(): boolean {
    if (sketchPoints.length < 3) return false;
    sketchProfiles.push({
      // 1枚目は土台なので`op`に意味は無い(Rust側も先頭では無視する)。
      op: sketchProfiles.length === 0 ? "union" : sketchOpSelect.value,
      points: sketchPoints,
    });
    sketchPoints = [];
    refreshSketch();
    return true;
  }

  document
    .getElementById("btn-sketch-confirm")!
    .addEventListener("click", () => {
      if (!confirmSketchProfile()) {
        reportError("プロファイルを閉じるには3点以上が要ります。");
      }
    });
  document
    .getElementById("btn-sketch-undo-point")!
    .addEventListener("click", () => {
      sketchPoints.pop();
      refreshSketch();
    });
  document.getElementById("btn-sketch-clear")!.addEventListener("click", () => {
    sketchProfiles.length = 0;
    sketchPoints = [];
    refreshSketch();
  });

  document
    .getElementById("btn-sketch-extrude")!
    .addEventListener("click", () => {
      // 描きかけの点列が3点以上あるなら、押し忘れとみなして自動で確定する
      // ——「確定を押していなかったせいで最後の1枚が消える」のは、
      // 手順を1つ増やすだけの価値が無い失敗の仕方である。
      confirmSketchProfile();
      if (sketchProfiles.length === 0) {
        reportError(
          "押し出すプロファイルがありません(地面をクリックして3点以上の多角形を描いてください)。",
        );
        return;
      }
      const depth = Number(sketchDepthInput.value);
      let result: {
        shape: ImportedShapeJson;
        origin: [number, number];
        rest_height: number;
        profile_area: number;
        volume: number;
      };
      try {
        result = JSON.parse(
          sketch_extrude_shape_json(
            JSON.stringify({ depth, profiles: sketchProfiles }),
          ),
        );
      } catch (err) {
        reportError(`押し出しに失敗しました: ${String(err)}`);
        return;
      }
      let bodyIndex: number;
      try {
        // **既存の汎用スポナーをそのまま使う**——`mesh`タグを解釈するのは
        // Rust側の`shape_json_to_shape`1箇所だけで、スポーン経路は
        // Prefab/複製と完全に共通になる。
        bodyIndex = applyComponent(world, "spawn_shape_json", {
          shape_json: JSON.stringify(result.shape),
          x: result.origin[0],
          y: result.rest_height,
          z: result.origin[1],
          material_name: spawnMaterialSelect.value,
        }).index as number;
      } catch (err) {
        reportError(`スポーンに失敗しました: ${String(err)}`);
        return;
      }
      // 見た目は**押し出した三角形そのもの**で描く(`meshFromShapeJson`の
      // `mesh`分岐)。Rust側が保持しているのは分解後の凸パーツなので、
      // 読み直すと切り欠きが凸パーツの和として近似された形になる——
      // 描画は元のメッシュを使う方が忠実である。
      addSpawnedMesh(bodyIndex, meshFromShapeJson(result.shape).mesh);
      selectBody(bodyIndex);
      // 作り終えたスケッチは片付ける(同じ断面をもう一度置きたいことより、
      // 次の形を描き始めたいことの方が多い)。
      sketchProfiles.length = 0;
      sketchPoints = [];
      refreshSketch();
    });

  // Scene View の左クリックで頂点を置く。右クリック(スポーンパレット)と
  // 同じく、**押した位置と離した位置の差**でドラッグ(カメラ操作)と
  // クリックを分ける——OrbitControls の左ドラッグ回転をそのまま残したまま、
  // 「動かさずに離した」ときだけ頂点を置く。
  const sketchGroundHit = new THREE.Vector3();
  let sketchDownScreen: { x: number; y: number } | null = null;
  renderer.domElement.addEventListener("pointerdown", (event) => {
    if (gizmoTool !== "sketch" || event.button !== 0) return;
    sketchDownScreen = { x: event.clientX, y: event.clientY };
  });
  renderer.domElement.addEventListener("pointercancel", (event) => {
    if (event.button === 0) sketchDownScreen = null;
  });
  renderer.domElement.addEventListener("pointerup", (event) => {
    if (gizmoTool !== "sketch" || event.button !== 0) return;
    const down = sketchDownScreen;
    sketchDownScreen = null;
    if (!down) return;
    if (Math.hypot(event.clientX - down.x, event.clientY - down.y) > DRAG_THRESHOLD_PX)
      return; // カメラを回しただけ。
    const rect = renderer.domElement.getBoundingClientRect();
    pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointerNdc, camera);
    if (!raycaster.ray.intersectPlane(groundPlane, sketchGroundHit)) return;
    const x = snapToGrid(sketchGroundHit.x);
    const z = snapToGrid(sketchGroundHit.z);
    // **始点をもう一度クリックしたらループを閉じる**(CADの標準的な操作)。
    // 判定はワールド距離ではなく画面上の距離で行う——カメラが遠いほど
    // 1ピクセルの表すワールド距離は大きくなるので、見た目どおりの
    // 「始点の上でクリックした」が成立する。
    if (sketchPoints.length >= 3) {
      const [sx, sz] = sketchPoints[0];
      const start = projectToScreen(
        new THREE.Vector3(sx, SKETCH_PREVIEW_Y, sz),
      );
      const CLOSE_LOOP_PX = 12;
      if (
        Math.hypot(event.clientX - start.x, event.clientY - start.y) <=
        CLOSE_LOOP_PX
      ) {
        confirmSketchProfile();
        return;
      }
    }
    sketchPoints.push([x, z]);
    refreshSketch();
  });

  // **材料派生(増分L)**。シーンJSONの`materials[].extends`と同じ仕組みを
  // 実行時に開く。**派生できるのは密度のみ**——`MaterialOverride`(シーンJSON側)
  // も密度だけを持つので、そちらと表現力を揃えた(食い違うとエディタで作った
  // 材料をシーンJSONへ書き出せなくなる)。
  document
    .getElementById("btn-derive-material")!
    .addEventListener("click", () => {
      const base = spawnMaterialSelect.value;
      const name = window.prompt(
        `「${base}」から派生する新しい材料名`,
        `${base}-軽量`,
      );
      if (!name) return;
      const densityText = window.prompt("新しい密度 [kg/m³]", "500");
      if (!densityText) return;
      const density = Number(densityText);
      try {
        applyComponent(world, "derive_material", {
          base_name: base,
          new_name: name,
          density,
        });
        const option = document.createElement("option");
        option.value = name;
        option.textContent = name;
        spawnMaterialSelect.appendChild(option);
        spawnMaterialSelect.value = name;
      } catch (err) {
        reportError(`材料派生に失敗しました: ${String(err)}`);
      }
    });

  document
    .getElementById("btn-spawn-pendulum")!
    .addEventListener("click", () => {
      const { x, z } = nextSpawnPosition();
      const material = spawnMaterialSelect.value;
      const bodyIndex = applyComponent(world, "spawn_pendulum", {
        pivot_x: x,
        pivot_y: PENDULUM_PIVOT_HEIGHT,
        pivot_z: z,
        arm_length: PENDULUM_ARM_LENGTH,
        material_name: material,
      }).index as number;
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.3, 16, 12),
        new THREE.MeshStandardMaterial({ color: 0xff66cc }),
      );
      addSpawnedMesh(bodyIndex, mesh);
      const lineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(),
        new THREE.Vector3(),
      ]);
      const line = new THREE.Line(
        lineGeometry,
        new THREE.LineBasicMaterial({ color: 0xffaa00 }),
      );
      scene.add(line);
      constraintLines.set(bodyIndex, line);
    });

  document.getElementById("btn-spawn-motor")!.addEventListener("click", () => {
    const { x, z } = nextSpawnPosition();
    const material = spawnMaterialSelect.value;
    const bodyIndex = applyComponent(world, "spawn_motor_arm", {
      pivot_x: x,
      pivot_y: PENDULUM_PIVOT_HEIGHT,
      pivot_z: z,
      material_name: material,
    }).index as number;
    motorArmBodies.add(bodyIndex);
    currentMotorTarget.set(bodyIndex, MOTOR_TARGET_LOW);
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.2, 1.2, 0.2),
      new THREE.MeshStandardMaterial({ color: 0x66ffcc }),
    );
    addSpawnedMesh(bodyIndex, mesh);
  });

  document.getElementById("btn-spawn-fluid")!.addEventListener("click", () => {
    applyComponent(world, "spawn_fluid_block", {});
    const count = readNumber(world, "fluid_particle_count");
    fluidPositionAttribute = new THREE.BufferAttribute(
      new Float32Array(count * 3),
      3,
    );
    fluidGeometry.setAttribute("position", fluidPositionAttribute);
    fluidPoints.visible = true;
    highlightHierarchy = rebuildHierarchy();
  });

  // フレーム階層ドリルインUI: Hierarchyで選択中のフレーム(既定はROOTでは
  // なく、起動時に追加した既定のフレーム——`selectedFrameIndex`の初期値)の
  // 子として新規フレームを追加する。追加した新規フレームをそのまま選択状態に
  // することで、連続クリックすると親→子→孫…と鎖状にネストしたフレームを
  // 手軽に組み立てられる(選択を変えれば任意のフレームの下に分岐させることも
  // できる)。
  document.getElementById("btn-add-frame")!.addEventListener("click", () => {
    const newFrameIndex = applyComponent(world, "add_child_frame", {
      parent_index: selectedFrameIndex,
      origin_offset_x: FRAME_CHILD_OFFSET,
      origin_offset_y: 0,
      origin_offset_z: 0,
      angular_velocity_z: FRAME_AXIS_ANGULAR_VELOCITY,
    }).index as number;
    createFrameAxesHelper(newFrameIndex);
    selectFrame(newFrameIndex);
  });

  playButton.addEventListener("click", () => {
    if (mode !== "play") return;
    playing = !playing;
    playButton.textContent = playing ? "⏸" : "▶";
  });
  // **N step 送り(群2)**。設計 §1.1 の「[▶ ⏸ ⏭]」は1step送りだが、
  // 実際に物理を観察していると「10 step だけ進めて接触の瞬間を見る」
  // 「600 step 進めて定常状態まで飛ばす」といった操作が頻繁に要る
  // (これまでは⏭を数百回押すか、Playで流して勘で止めるしかなかった)。
  // 隣の数値入力で step 数を指定する。
  const stepCountInput = document.getElementById(
    "input-step-count",
  ) as HTMLInputElement;
  stepButton.addEventListener("click", () => {
    if (mode === "play" && !playing) {
      const requested = Math.floor(Number(stepCountInput.value));
      const count = Number.isFinite(requested)
        ? Math.min(Math.max(requested, 1), 10000)
        : 1;
      for (let i = 0; i < count; i += 1) {
        // Play ループと同じくヒーターは毎 step 再送する(「1step分だけ効く」
        // 縮約セマンティクス、`HEATER_WATTS` のdoc参照)。
        if (heaterToggle.checked) applyComponent(world, "push_heat_source", { watts: HEATER_WATTS });
        applyThrustForStep();
        world.step();
      }
      appendConsoleEntries(world.drain_events_text());
      // 診断バッジ(増分K)。毎フレーム最新の残差・最大速度で更新する。
      if (consoleDiagnosticsRef.current) {
        consoleDiagnosticsRef.current(
          readNumber(world, "energy_residual"),
          readNumber(world, "max_body_speed"),
          readNumber(world, "dt"),
        );
      }
      render();
    }
  });

  // Timeline スクラバ(設計docs/00-foundation/04-architecture.md「巻き戻しの
  // スナップショット予算」既定1s間隔・リングバッファN=8面)。ドラッグ中
  // (`scrubbing`)は`render()`側からスクラバのmax/valueを触らない——そうしないと
  // 毎フレームの「最新に追従」更新がユーザーのドラッグ位置を上書きしてしまう。
  const scrubber = document.getElementById(
    "timeline-scrubber",
  ) as HTMLInputElement;
  const playModeBadge = document.getElementById("play-mode-badge")!;
  let scrubbing = false;
  scrubber.addEventListener("pointerdown", () => {
    scrubbing = true;
    playing = false;
    playButton.textContent = "▶";
  });
  scrubber.addEventListener("input", () => {
    applyComponent(world, "restore_snapshot", { index: Number(scrubber.value) });
    render();
  });
  scrubber.addEventListener("pointerup", () => {
    scrubbing = false;
  });

  // Timelineブックマーク(設計docs/23-frontend/01-editor.md §1.4「ブックマーク:
  // 任意時点にラベル付けし、後で戻れる」)。リングバッファの退避を受けない別領域
  // (`add_bookmark`/`restore_bookmark`)に保存する。縮約実装の理由: シーンJSONと
  // 一緒に出す「共有」用途(設計の記述)は未実装、ブラウザ内での往復のみ。
  const bookmarkLabelInput = document.getElementById(
    "bookmark-label",
  ) as HTMLInputElement;
  const addBookmarkButton = document.getElementById(
    "btn-add-bookmark",
  ) as HTMLButtonElement;
  const bookmarkList = document.getElementById("bookmark-list")!;

  function renderBookmarkList() {
    bookmarkList.innerHTML = "";
    const count = readNumber(world, "bookmark_count");
    for (let i = 0; i < count; i++) {
      const item = document.createElement("span");
      const chip = document.createElement("button");
      chip.className = "bookmark-chip";
      chip.textContent = `${world.read_component("bookmark_label_at", String(i))} (${readNumber(world, "bookmark_time_at", String(i)).toFixed(1)}s)`;
      chip.addEventListener("click", () => {
        playing = false;
        playButton.textContent = "▶";
        applyComponent(world, "restore_bookmark", { index: i });
        render();
      });
      item.appendChild(chip);

      // ブックマークのエクスポート(設計§6「保存・共有: シーンJSON+Replay+
      // ブックマークを単一ファイルとしてエクスポート」の縮約実装)。`World`が
      // `Serialize`を持たないため内部状態のバイト単位の保存ではなく、シーンJSON
      // Importへそのまま読み込める`Scenario`互換JSONとして剛体の観測可能な状態
      // (位置・姿勢・速度)のみを書き出す(流体/熱/回路ドメインの状態は対象外、
      // `bookmark_export_scene_json`のdoc参照)。
      const exportButton = document.createElement("button");
      exportButton.textContent = "⬇";
      exportButton.title =
        "このブックマークをシーンJSONとしてエクスポート(Importで読み込み可能)";
      exportButton.addEventListener("click", () => {
        const json = world.read_component("bookmark_export_scene_json", String(i));
        const blob = new Blob([json], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `bookmark_${world.read_component("bookmark_label_at", String(i))}.json`;
        a.click();
        URL.revokeObjectURL(url);
      });
      item.appendChild(exportButton);

      bookmarkList.appendChild(item);
    }
  }

  // **未保存の変更の確認(群2)**。設計 docs/23-frontend/01-editor.md §6
  // 「保存・共有」。エディタで小一時間かけて組んだシーンが、タブを閉じたり
  // リロードしたりした瞬間に**警告なく消える**状態だった(すべてブラウザの
  // メモリ上にしか無い)。
  //
  // **「変更あり」の判定はシーンJSONの内容比較ではなく操作の有無で行う**——
  // 内容比較には現在状態のシリアライズが要り、それ自体が重い
  // (`bookmark_export_scene_json` はブックマークを1件消費する)。
  // スポーン・Import・ギャラリー読み込み・Gizmo編集・Prefab登録のいずれかが
  // 起きたら「未保存」とみなし、一括Exportしたら解除する。
  // `beforeunload` の確認ダイアログはブラウザ側の仕様で文言を指定できないため、
  // `preventDefault()` のみを行う(表示される文言はブラウザ既定)。
  window.addEventListener("beforeunload", (event) => {
    if (!hasUnsavedChanges) return;
    event.preventDefault();
    event.returnValue = "";
  });

  // **単一ファイル Export(群2、`ProjectBundle`のdoc参照)**。設計 §6
  // 「保存・共有: シーンJSON+Replay+ブックマークを単一ファイルとして
  // エクスポート」。これまで3つが**別々のボタンで別々のファイル**に落ちており、
  // ブックマーク一覧に至っては書き出す手段が無かった(個々のブックマークを
  // シーンJSONとして落とせるだけ)。
  //
  // 現在の状態は `world.read_component("export_scene_json", "")` が `sim_world::Scenario` 形式で返す
  // (**群2で追加**——それまで「現在状態をシーンJSONにする」経路は
  // `bookmark_export_scene_json` しかなく、書き出すたびに一時ブックマークが
  // 一覧に残る実装になっていた。実装検証中に気付いて Rust 側へ
  // `export_scene_json` を切り出した)。
  projectBundleRef.current = () => {
    const sceneJson = world.read_component("export_scene_json", "");
    const bookmarks: { label: string; time: number }[] = [];
    for (let i = 0; i < readNumber(world, "bookmark_count"); i += 1) {
      bookmarks.push({
        label: world.read_component("bookmark_label_at", String(i)),
        time: readNumber(world, "bookmark_time_at", String(i)),
      });
    }
    return {
      formatVersion: 1,
      exportedAt: new Date().toISOString(),
      scene: JSON.parse(sceneJson),
      bodies: sceneExportRef.current ? sceneExportRef.current() : [],
      commandLog: [...commandLog],
      bookmarks,
      stateHash: world.read_component("state_hash", ""),
    };
  };
  // 一括Exportしたら「保存済み」とみなす(ファイルが手元に残るため)。
  projectExportedRef.current = () => {
    hasUnsavedChanges = false;
  };

  addBookmarkButton.addEventListener("click", () => {
    const label =
      bookmarkLabelInput.value.trim() || `t=${readNumber(world, "time").toFixed(1)}s`;
    applyComponent(world, "add_bookmark", { label });
    bookmarkLabelInput.value = "";
    renderBookmarkList();
  });

  // Consoleのイベント行クリック→Timelineジャンプ(設計docs/23-frontend/
  // 01-editor.md §1.5「クリックでTimeline/Scene Viewと連動」)。イベント行に
  // 埋め込まれたstep番号の時刻に最も近いスナップショットへ巻き戻す(スナップショット
  // は1s間隔のため厳密なstep一致ではなく最近傍、`restore_snapshot`と同じ挙動)。
  jumpToStepRef.current = (step: number) => {
    const count = readNumber(world, "snapshot_count");
    if (count === 0) return;
    const targetTime = step * DT;
    let bestIndex = 0;
    let bestDiff = Infinity;
    for (let i = 0; i < count; i++) {
      const diff = Math.abs(
        readNumber(world, "snapshot_time_at", String(i)) - targetTime,
      );
      if (diff < bestDiff) {
        bestDiff = diff;
        bestIndex = i;
      }
    }
    playing = false;
    playButton.textContent = "▶";
    applyComponent(world, "restore_snapshot", { index: bestIndex });
    render();
  };

  // 設計§4「Playモードでの介入は全てCommandとしてキューに積まれ、次ステップ先頭で
  // 適用される」の最小デモ: 直接オブジェクトの状態を書き換えるのではなく、
  // `push_apply_force`(Command::ApplyForceをキューに積む`sim-wasm`側の新API)を
  // 呼ぶだけで、実際の力の適用は次の`world.step()`側が担う。
  // 既定シーンの箱(鋼(炭素鋼)1m^3、密度約7850kg/m^3)を目視で分かる程度に動かす
  // (1クリックでΔv≈0.4m/s程度)よう較正した値。**残タスク完遂のシーンギャラリー
  // 増分**で`selectedBodyIndex`(選択中のボディ)へ効くよう一般化した——質量が
  // 異なるボディでは体感速度変化も変わる(質量ごとの再較正は対象外)。
  const NUDGE_FORCE_NEWTONS = 400_000.0;
  nudgeButton.addEventListener("click", () => {
    // ボディが無いシーン(D9/D34/D35のようなギャラリーシーン)では力を
    // 加える対象が無い(`hasSelectedBody`のdoc参照)。
    if (!hasSelectedBody()) return;
    applyComponent(world, "push_apply_force", {
      body_index: selectedBodyIndex,
      fx: 0.0,
      fy: NUDGE_FORCE_NEWTONS,
      fz: 0.0,
    });
    pushCommandLog(world, {
      kind: "ApplyForce",
      bodyIndex: selectedBodyIndex,
      fx: 0.0,
      fy: NUDGE_FORCE_NEWTONS,
      fz: 0.0,
    });
    if (forceOverlayToggle.checked) {
      const p = world.body_position_at_f32(selectedBodyIndex);
      showForceOverlay(
        new THREE.Vector3(p[0], p[1], p[2]),
        new THREE.Vector3(0.0, NUDGE_FORCE_NEWTONS, 0.0),
      );
    }
  });

  motorToggleButton.addEventListener("click", () => {
    if (mode !== "play" || !motorArmBodies.has(selectedBodyIndex)) return;
    const current =
      currentMotorTarget.get(selectedBodyIndex) ?? MOTOR_TARGET_LOW;
    const next =
      current === MOTOR_TARGET_LOW ? MOTOR_TARGET_HIGH : MOTOR_TARGET_LOW;
    applyComponent(world, "set_motor_target_at", {
      index: selectedBodyIndex,
      theta_target: next,
    });
    currentMotorTarget.set(selectedBodyIndex, next);
    pushCommandLog(world, {
      kind: "SetMotorTarget",
      bodyIndex: selectedBodyIndex,
      bodyLabel: world.read_component("body_label_at", String(selectedBodyIndex)),
      targetAngle: next,
    });
  });

  circuitSwitchToggle.addEventListener("change", () => {
    // 自由配線回路エディタでリセットした後は`circuit_switch_index`(固定デモの
    // スイッチ)が新回路のスイッチ数を超えて無効になり得るため、この経路は
    // 無効化する(`circuitFreeWiringState`のdoc参照、チェックボックス自体も
    // リセット時に`disabled`にする)。
    if (circuitFreeWiringState.active) return;
    applyComponent(world, "set_circuit_switch_closed", { closed: circuitSwitchToggle.checked });
    pushCommandLog(world, {
      kind: "SetSwitch",
      closed: circuitSwitchToggle.checked,
    });
  });

  heaterToggle.addEventListener("change", () => {
    // ヒーター自体の`Command::SetHeatSource`は`frame()`ループが毎subStep
    // 再送する(モジュールdoc「1step分だけ効く」縮約セマンティクス参照)ため
    // ここでは記録しない——ユーザーが行った「切替」という離散操作のみ記録する
    // (Replay再生実行はこの`on`/`watts`から再送区間を再構成する)。
    pushCommandLog(world, {
      kind: "SetHeatSource",
      on: heaterToggle.checked,
      watts: HEATER_WATTS,
    });
  });

  const inspectorPosition = new THREE.Vector3();
  const inspectorRotationQuat = new THREE.Quaternion();
  const inspectorRotation = new THREE.Euler();
  const inspectorVelocity = new THREE.Vector3();

  function render() {
    updatePredictionResults();
    // QA不具合5: 再生中は⏭(Nstep送り)を押しても無反応なのに、ボタンは
    // 有効なまま(理由の表示も無い)だった。Unityの Step は「一時停止して
    // 1フレーム進める」操作なので、実際に再生中(`playing`)は無効化して
    // 空振りを防ぐ——クリックハンドラの`mode === "play" && !playing`という
    // 条件そのものをボタンの見た目にも反映する。
    stepButton.disabled = mode === "edit" || playing;

    // 全ボディのメッシュ位置/姿勢/スケールを同期する(**残タスク完遂の
    // シーンギャラリー増分**で`box`だけの決め打ち同期を統合、`bodyMeshes`の
    // doc参照)。**Plane形状は同期から除外する**——`Shape::Plane`の世界座標での
    // 向き/位置は`normal`/`d`で定義され、剛体の`Transform.position`/
    // `rotation`とは独立(`RigidBodyDesc::dynamic`は常に`rotation:
    // Quat::IDENTITY`を設定するため、同期すればPlaneメッシュ生成時に
    // `normal`から計算した向き(`sceneImportRef`のPlane分岐参照)が単位回転で
    // 上書きされてしまう——統合の際に発見し、床メッシュの見た目が壊れる前に
    // 気付いて対処した)。Planeは静的なので同期しなくても正しい。
    for (const [bodyIndex, mesh] of bodyMeshes) {
      if (world.read_component("body_shape_kind_at", String(bodyIndex)) === "plane") continue;
      const sp = world.body_position_at_f32(bodyIndex);
      mesh.position.set(sp[0], sp[1], sp[2]);
      const sr = world.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(sr[0], sr[1], sr[2], sr[3]);
      const xyz = currentScaleXyz.get(bodyIndex);
      if (xyz) mesh.scale.set(xyz[0], xyz[1], xyz[2]);
      else mesh.scale.setScalar(currentScale.get(bodyIndex) ?? 1.0);
    }

    for (const [bodyIndex, line] of constraintLines) {
      if (!constraintOverlayToggle.checked) {
        line.visible = false;
        continue;
      }
      const anchors = world.constraint_anchor_points_at(bodyIndex);
      if (anchors.length < 6) {
        line.visible = false;
        continue;
      }
      const positions = line.geometry.attributes
        .position as THREE.BufferAttribute;
      positions.setXYZ(0, anchors[0], anchors[1], anchors[2]);
      positions.setXYZ(1, anchors[3], anchors[4], anchors[5]);
      positions.needsUpdate = true;
      line.visible = true;
    }

    if (frameOverlayToggle.checked) {
      for (const [frameIndex, helper] of frameAxesHelpers) {
        // `frame_world_position_f32`/`frame_world_rotation_f32`はいずれも
        // Wasmメモリを直接指す一時的なビューを返す(B16、`HotPathViewBuffers`の
        // doc参照)。`pos`を読み切る前に`rot`側のWasm呼び出しを挟むと`pos`の
        // ビューが無効化されうるため、他の同種呼び出し(`body_position_at_f32`
        // 等)と同じく「呼んだら即座に読み切る」順序を守る。
        const pos = world.frame_world_position_f32(frameIndex);
        helper.position.set(pos[0], pos[1], pos[2]);
        const rot = world.frame_world_rotation_f32(frameIndex);
        helper.quaternion.set(rot[0], rot[1], rot[2], rot[3]);
        helper.visible = true;
      }
    } else {
      for (const helper of frameAxesHelpers.values()) {
        helper.visible = false;
      }
    }

    if (fluidPositionAttribute) {
      const positions = world.fluid_particle_positions_f32();
      (fluidPositionAttribute.array as Float32Array).set(positions);
      fluidPositionAttribute.needsUpdate = true;
    }

    // **格子流体の速度場オーバーレイ(増分L)**。設計§1.2「流体場」のうち
    // SPHの粒子表示は実装済みだったが、**`GridFluid2D`の速度場は表示手段が
    // 無かった**——D14(渦)・D15(対流)はどちらも格子流体だけのシーンで、
    // Scene Viewに何も描かれずProbe Graphsでしか観測できなかった。
    // `grid_fluid_velocity_field_f32`(1セル4要素 `[x, y, u, v]`)を
    // LineSegmentsの頂点バッファへ直接書き込む(セルごとに`ArrowHelper`を
    // 作ると数百オブジェクトになるため、1本のジオメトリで描く)。
    updateGridFluidOverlay(world);

    // **群3で追加したドメインの描画**。それまで Scene View に一切現れず
    // Probe Graphs でしか観測できなかった(ソフトボディ・天体)、あるいは
    // ドメイン自体が World に無かった(統計)ものを描く。
    updateSoftBodyOverlay(world);
    updateAstroOverlay(world);
    updateParticleCloud(gasCloud, world.kinetic_gas_positions_f32(1), gasBoxCenter);
    updateParticleCloud(brownianCloud, world.brownian_positions_f32(1), [0, 0, 0]);
    updateFieldPanel(world);

    // **2026-07-28のD9/D34/D35増分で追加したガード**: `hasSelectedBody()`が
    // falseのとき(D9/D34/D35のように力学ボディを1つも持たないギャラリー
    // シーンを読み込んだ直後)、`body_position_at_f32`等を呼ぶとJS例外を
    // 投げてこの`render()`ループ自体が(次フレーム以降も)壊れる——このガード
    // 撤廃前は実際にここで毎フレームパニックしていた(`heater_node_temperature`/
    // `Circuit::node_voltage`と同じバグの系統、増分3-3/B2で発見した2件と同型)。
    const selectedBodyValid = hasSelectedBody();
    // `body_position_at_f32`/`body_rotation_at_f32`/`body_velocity_at_f32`は
    // いずれもWasmメモリを直接指す一時的なビューを返す(B16、
    // `HotPathViewBuffers`のdoc参照)——3つとも取得してからまとめて読むと、
    // 後段の呼び出しが前段のビューを無効化しうる。呼んだら即座に
    // `inspectorXxx`へ読み切ってから次を呼ぶ順序を守る。
    if (selectedBodyValid) {
      const selectedPosition = world.body_position_at_f32(selectedBodyIndex);
      inspectorPosition.set(
        selectedPosition[0],
        selectedPosition[1],
        selectedPosition[2],
      );
      const selectedRotation = world.body_rotation_at_f32(selectedBodyIndex);
      inspectorRotationQuat.set(
        selectedRotation[0],
        selectedRotation[1],
        selectedRotation[2],
        selectedRotation[3],
      );
      const selectedVelocity = world.body_velocity_at_f32(selectedBodyIndex);
      inspectorVelocity.set(
        selectedVelocity[0],
        selectedVelocity[1],
        selectedVelocity[2],
      );
    } else {
      inspectorPosition.set(0, 0, 0);
      inspectorRotationQuat.set(0, 0, 0, 1);
      inspectorVelocity.set(0, 0, 0);
    }
    inspectorRotation.setFromQuaternion(inspectorRotationQuat);
    if (selectedBodyValid) {
      updateInspectorTransformFields(
        inspectorPosition,
        inspectorRotation,
        inspectorVelocity,
      );
      updateInspectorRigidBodyFields(world, selectedBodyIndex);
    }
    if (isGalleryScene) {
      // シーンギャラリー読み込み中(D6/D11のような、Scene Viewに描く物が
      // 乏しい/無いデモを含む)は、シーンJSONの`probes`セクションが宣言した
      // 本数の系列を全て束ねる(0本でも`updateProbeGraph([])`が安全に空描画
      // する、`setUpProbeGraph`のdoc参照)。
      const probeCount = readNumber(world, "imported_probe_count");
      const series: ProbeSeries[] = [];
      for (let i = 0; i < probeCount; i++) {
        series.push({
          // かんたんモードでは、グラフの凡例も人間の言葉にする
          // (`NodeTemp[0]` ではなく「コーヒーの温度」)。指定が無いプローブは
          // 従来どおり Rust 側の生ラベルを出す。
          label:
            guidedProbeLabels?.[i] ??
            world.read_component("imported_probe_label_at", String(i)),
          color: PROBE_GRAPH_COLORS[i % PROBE_GRAPH_COLORS.length],
          // `imported_probe_history_f64`はWasmメモリを直接指す一時的なビューを
          // 返す(B16、`HotPathViewBuffers`のdoc参照)——このループが呼ぶたび
          // 同じ1本の永続バッファを使い回すため、`series`へ積んだ後の要素を
          // 次のイテレーションが上書きしてしまう。`updateProbeGraph`が全系列を
          // まとめて後で描く(=ループを抜けるまで読まない)以上、ここで即座に
          // 自前のコピーへ読み切っておく必要がある。
          history: Float64Array.from(world.imported_probe_history_f64(i)),
        });
      }
      updateProbeGraph(series, readNumber(world, "dt"), readNumber(world, "time"));
    } else {
      updateProbeGraph(
        [
          {
            label: "BodyPosY",
            color: "#9cf",
            // `y_probe_history_f64`/`speed_probe_history_f64`はいずれもWasm
            // メモリを直接指す一時的なビューを返す(B16、`HotPathViewBuffers`の
            // doc参照)。`updateProbeGraph`(`setUpProbeGraph`のdoc参照)は
            // 渡した`series`をCSVエクスポート用に次のフレーム以降・他のWasm
            // 呼び出しをまたいで保持し続けるため、ここで即座に自前のコピーへ
            // 読み切っておく必要がある。
            history: Float64Array.from(world.y_probe_history_f64()),
          },
          {
            label: "BodySpeed",
            color: "#fc6",
            history: Float64Array.from(world.speed_probe_history_f64()),
          },
        ],
        readNumber(world, "dt"),
        readNumber(world, "time"),
      );
    }

    const speed = inspectorVelocity.length();
    if (velocityOverlayToggle.checked && speed > 1e-6) {
      velocityDirection.copy(inspectorVelocity).divideScalar(speed);
      velocityArrow.position.copy(inspectorPosition);
      velocityArrow.setDirection(velocityDirection);
      velocityArrow.setLength(
        Math.max(speed * VELOCITY_OVERLAY_SCALE, 0.2),
        Math.min(0.2, speed * VELOCITY_OVERLAY_SCALE * 0.3),
        Math.min(0.15, speed * VELOCITY_OVERLAY_SCALE * 0.2),
      );
      velocityArrow.visible = true;
    } else {
      velocityArrow.visible = false;
    }

    if (contactOverlayToggle.checked) {
      const contactPoints = world.contact_points_f32();
      const count = Math.min(
        contactPoints.length / 3,
        CONTACT_MARKER_POOL_SIZE,
      );
      for (let i = 0; i < CONTACT_MARKER_POOL_SIZE; i++) {
        if (i < count) {
          contactMarkers[i].position.set(
            contactPoints[i * 3],
            contactPoints[i * 3 + 1],
            contactPoints[i * 3 + 2],
          );
          contactMarkers[i].visible = true;
        } else {
          contactMarkers[i].visible = false;
        }
      }
    } else {
      for (const marker of contactMarkers) marker.visible = false;
    }

    forceArrow.visible =
      forceOverlayToggle.checked && performance.now() < forceOverlayHideAtMs;

    // **ツール切替(群2で追加)**: 設計 §1.2「W(移動)/E(回転)/R(スケール)/
    // Q(選択のみ)」。以前は3つのギズモを**同時に**表示していたため、
    // 移動したいのに回転リングを掴んでしまうなど誤操作が起きやすく、
    // 何より画面が見えづらかった。1つずつ出す。
    const showGizmo =
      selectedBodyValid &&
      mode === "edit" &&
      !(world.read_component("body_is_static_at", String(selectedBodyIndex)) === "true");
    gizmoGroup.visible = showGizmo && gizmoTool === "translate";
    rotationGizmoGroup.visible = showGizmo && gizmoTool === "rotate";
    scaleGizmoGroup.visible = showGizmo && gizmoTool === "scale";
    if (showGizmo) {
      gizmoGroup.position.copy(inspectorPosition);
      rotationGizmoGroup.position.copy(inspectorPosition);
      scaleGizmoGroup.position.copy(inspectorPosition);
      // **World / Local 座標系(群2)**。設計 §1.2「座標系は World / Local 切替可」。
      // Local では Gizmo をボディの姿勢に合わせて回す——回転した箱を「その箱の
      // 長辺方向へ」動かせるようになる(World 固定では常に世界軸方向にしか
      // 動かせず、傾いた物体の扱いが著しく面倒だった)。
      if (gizmoSpace === "local") {
        gizmoGroup.quaternion.copy(inspectorRotationQuat);
        scaleGizmoGroup.quaternion.copy(inspectorRotationQuat);
      } else {
        gizmoGroup.quaternion.identity();
        scaleGizmoGroup.quaternion.identity();
      }
    }

    const hashFull = world.read_component("state_hash", "");
    // QA不具合8: 該当ドメインを持たないシーンで`heater T = NaN K`と表示され、
    // `circuit V = 0.000 V`は「回路が無い」のか「測って0Vだった」のか区別が
    // 付かなかった。熱ドメインの無は`heater_node_temperature()`が返すNaNを
    // そのまま検出できる。回路は`circuit_element_count() === 0`(=回路
    // ドメインが無効、または素子ゼロの意味を持たない状態)を「無」の判定に使う。
    const heaterTemperature = readNumber(world, "heater_node_temperature");
    const hasCircuit = readNumber(world, "circuit_element_count") > 0;
    // QA不具合7: 読み込んだシーンが宣言したプローブを第一候補にする
    // (`selectHudProbes`のdoc参照)。宣言が無ければ従来の固定アクセサへ落ちる。
    const hudProbes = selectHudProbes(world);
    let circuitLine = "circuit V = —";
    if (hudProbes.circuit) {
      const volts = readNumber(world, "imported_probe_value_at", String(hudProbes.circuit.index));
      circuitLine = `circuit V[${hudProbes.circuit.node}] = ${formatHudNumber(volts)} V`;
    } else if (hasCircuit) {
      circuitLine = `circuit V[${CIRCUIT_DIVIDER_NODE_LABEL}] = ${formatHudNumber(readNumber(world, "circuit_divider_voltage"))} V`;
    }
    let temperatureLine = "heater T = —";
    const sceneTemperature = hudProbes.temperature
      ? readNumber(world, "imported_probe_value_at", String(hudProbes.temperature.index))
      : heaterTemperature;
    if (Number.isFinite(sceneTemperature)) {
      // **基準は「プローブが最初に記録したサンプル」から取る**。
      // `imported_probe_value_at` は履歴が空だと 0 を返すので、step 0 の
      // フレームで現在値を基準にすると ΔT が「293 K も上がった」ことになる。
      // かつ「HUD が最初に描かれた時点の値」を基準にすると、停止状態で
      // `⏭` を 600 step 送ってから見たときに Δ が常に 0 になってしまう。
      // 履歴の先頭を使えばどちらの経路でも初期温度が基準になる
      // (履歴はリングバッファだが、非空になった最初のフレームで確定させる)。
      if (hudProbes.temperatureBaseline === null) {
        if (hudProbes.temperature) {
          const history = world.imported_probe_history_f64(hudProbes.temperature.index);
          if (history.length > 0) hudProbes.temperatureBaseline = history[0];
        } else {
          hudProbes.temperatureBaseline = sceneTemperature;
        }
      }
      const node = hudProbes.temperature ? hudProbes.temperature.node : "0";
      const delta =
        hudProbes.temperatureBaseline === null
          ? 0
          : sceneTemperature - hudProbes.temperatureBaseline;
      // ΔT を併記する——絶対値の桁が大きい(293 K)ので、微小変化は差分でしか
      // 読めない(D20 の ΔT = 1.25×10⁻⁴ K が「まったく動かない」と見えていた)。
      temperatureLine =
        `heater T[${node}] = ${formatHudNumber(sceneTemperature)} K` +
        (delta === 0 ? "" : ` (Δ ${delta > 0 ? "+" : ""}${formatHudNumber(delta)})`);
    }
    hud.textContent = [
      `t = ${readNumber(world, "time").toFixed(3)} s`,
      `step = ${readNumber(world, "step_count").toString()}`,
      `y = ${selectedBodyValid ? inspectorPosition.y.toFixed(4) : "—"} m`,
      circuitLine,
      temperatureLine,
    ].join("\n");
    timelineTime.textContent = `t = ${readNumber(world, "time").toFixed(3)} s`;
    timelineStep.textContent = `step = ${readNumber(world, "step_count").toString()}`;
    hashDisplay.textContent = `hash: ${hashFull.slice(0, 8)}`;
    hashDisplay.title = hashFull;
    // バッジは操作の可否を決める最重要の状態なので、文字だけでなく色でも
    // 分ける(`style.css` の `.badge[data-mode]`)。
    const badgeMode = mode === "edit" ? "edit" : playing ? "playing" : "paused";
    playModeBadge.textContent =
      badgeMode === "edit" ? "Edit" : badgeMode === "playing" ? "Playing" : "Paused";
    playModeBadge.dataset.mode = badgeMode;

    if (!scrubbing) {
      const latestIndex = Math.max(readNumber(world, "snapshot_count") - 1, 0);
      scrubber.max = String(latestIndex);
      scrubber.value = String(latestIndex);
    }

    syncSettingsInputs();
    if (guidedFollowCamera) updateGuidedFollowCamera();
    // enableDamping を使うので毎フレーム update が要る。
    orbit.update();
    renderer.render(scene, camera);
  }
  hashDisplay.addEventListener("click", () => {
    // **コピーできたことを伝える**(増分「UI 品質の底上げ」)。設計 §2 は
    // 「クリックでフル 64 bit ハッシュをコピー」と定めているが、これまでは
    // 押しても画面が一切変わらず、コピーされたのか押し損ねたのか分からなかった
    // (失敗も `.catch(() => {})` で握り潰していた——権限が無い文脈では
    // 何も起きないまま「壊れている」と読まれる)。
    const hash = world.read_component("state_hash", "");
    navigator.clipboard
      ?.writeText(hash)
      .then(() => showToast(`状態ハッシュをコピーしました: ${hash}`, "success"))
      .catch(() => showToast("クリップボードへコピーできませんでした。", "error"));
  });

  let accumulator = 0;
  // **かんたんモードの進み方**(`guided.ts` の `setPace`)。`null` なら従来どおり
  // 「時間倍率 × 実時間」で進める。数値が入っているときは *1 秒あたりの step 数*
  // として扱う——シーンごとに dt が 1e-12 秒(気体分子)〜31555 秒(太陽系)と
  // 16 桁も違い、同じ「×1」が実時間どおりにも「1 step に 4 分」にもなるため、
  // 現象ごとに見やすい速さを倍率では指定できない(D34 は上限の ×128 でも
  // 1 step 4 分かかり、選んでも永遠に何も起きなかった)。
  let guidedPace: number | null = null;
  let stepAccumulator = 0;
  /** かんたんモードが指定するプローブの表示名(index → 名前)。 */
  let guidedProbeLabels: Record<number, string> | null = null;
  let lastTimeMs = performance.now();

  function frame(nowMs: number) {
    const frameSeconds = Math.min((nowMs - lastTimeMs) / 1000, 0.25);
    lastTimeMs = nowMs;

    // ライブ再生中はそちらが Scene View を駆動する(現在の world は進めない)。
    const playingBack = advanceLivePlayback(frameSeconds);

    if (!playingBack && mode === "play" && playing) {
      // **`DT` 定数ではなく `world.dt()` を読む(群2)**。Settings で dt を
      // 変更できるようにした結果、固定の `DT` で積算すると「dt を半分にすると
      // 時間が倍速で進む」という嘘の挙動になっていた(実装検証中に発見)。
      const dt = readNumber(world, "dt");
      // このフレームで進めたい step 数(`guidedPace` の doc 参照)。
      let budget: number;
      if (guidedPace !== null) {
        stepAccumulator += frameSeconds * guidedPace;
        budget = Math.floor(stepAccumulator);
        stepAccumulator -= budget;
      } else {
        accumulator += frameSeconds * timeScale;
        budget = Math.floor(accumulator / dt);
      }
      let steps = 0;
      while (steps < budget && steps < MAX_STEPS_PER_FRAME) {
        if (heaterToggle.checked) applyComponent(world, "push_heat_source", { watts: HEATER_WATTS });
        applyThrustForStep();
        world.step();
        if (guidedPace === null) accumulator -= dt;
        steps += 1;
      }
      // **実効時間倍率(群2)**。高倍率では `MAX_STEPS_PER_FRAME` に当たって
      // 指定どおりの速度が出ない。**出ていないことを黙って隠さない**——
      // 達成できた倍率を出し、指定値に届かないときは赤で示す。
      // (届かないまま `accumulator` を溜め続けると、負荷が下がった瞬間に
      //  一気に進む「時間の借金」になるので、上限に当たったフレームでは
      //  余りを捨てる。)
      const capped = steps >= MAX_STEPS_PER_FRAME;
      if (capped) {
        accumulator = 0;
        stepAccumulator = 0;
      }
      updateEffectiveTimeScale(
        frameSeconds > 0 ? (steps * dt) / frameSeconds : timeScale,
        capped,
      );
      appendConsoleEntries(world.drain_events_text());
      // 診断バッジ(増分K)。毎フレーム最新の残差・最大速度で更新する。
      if (consoleDiagnosticsRef.current) {
        consoleDiagnosticsRef.current(
          readNumber(world, "energy_residual"),
          readNumber(world, "max_body_speed"),
          DT,
        );
      }
    }

    render();
    requestAnimationFrame(frame);
  }

  // QA不具合3: 自動フレーミング(`frameCameraOnContent`)はギャラリーシーンの
  // 読み込み時にしか呼ばれておらず、起動直後の既定シーン(箱がy=10mに立つ)は
  // カメラの固定初期位置のままだったため、開いて最初に見えるのが空の床
  // だけになっていた。ループを開始する前に一度呼ぶ——ただし`box`メッシュの
  // Three.js側の位置は`render()`が毎フレーム`world.body_position_at_f32`から
  // 反映するまで構築時の既定値(0,0,0)のままなので、先に`render()`を1回
  // 呼んでメッシュを実際の物理状態へ同期させてからでないと、存在しない
  // (0,0,0)を対象に画角を合わせてしまう。
  // **かんたんモード(`guided.ts`)へ渡す窓口**。意図的にこれだけに絞ってある
  // ——シーンを読む / 進める / 止める / いまの数値を読む。ここが太ると
  // 統合エディタとかんたんモードが互いの内部状態に依存し始め、どちらも
  // 直せなくなる。読み込みは統合エディタのシーンギャラリーと同じ経路
  // (`sceneGalleryRef.current`)を通す——別経路を作ると、片方だけ直った
  // 不整合(旧ワールドのメッシュが残る等)が必ず起きる。
  const guidedApi: GuidedApi = {
    loadSceneJson: (json) => {
      sceneGalleryRef.current?.(json);
      // 読み込み直後は「いまある物」しか無いので、落下の行き先(床)まで
      // 入る画角へ即座に合わせ直す(`updateGuidedFollowCamera` の doc 参照)。
      guidedFollowCamera = true;
      guidedCameraSnap = true;
    },
    followCamera: (enabled) => {
      guidedFollowCamera = enabled;
      guidedCameraSnap = enabled;
    },
    play: () => setMode("play"),
    pause: () => {
      playing = false;
      playButton.textContent = "▶";
    },
    isPlaying: () => mode === "play" && playing,
    setProbeLabels: (labels) => {
      guidedProbeLabels = labels;
    },
    setPace: (stepsPerSecond) => {
      guidedPace = stepsPerSecond;
      stepAccumulator = 0;
      accumulator = 0;
    },
    probeCount: () => readNumber(world, "imported_probe_count"),
    probeValue: (index) =>
      readNumber(world, "imported_probe_value_at", String(index)),
    time: () => readNumber(world, "time"),
  };
  guidedApiRef.current = guidedApi;

  render();
  frameCameraOnContent();
  requestAnimationFrame(frame);
}

function main() {
  setUpLayoutPresetSwitcher();
  // UI 基盤(増分「UI 品質の底上げ」)。world より先に立ち上げる——読み込み中
  // でもショートカット一覧は開けるし、初期化に失敗したときの通知経路(トースト)が
  // 必要になるのはまさにその瞬間だから。
  setUpPanelSplitters();
  setUpShortcutOverlay();
  setUpTabListKeyboardNavigation();
  setUpHierarchyKeyboardNavigation();
  const updateProbeGraph = setUpProbeGraph();
  const jumpToStepRef: JumpToStepRef = { current: null };
  const selectBodyRef: SelectBodyRef = { current: null };
  // **増分K: Toolbarのシーン選択ドロップダウン**(設計§1.1「Toolbar: …
  // シーン選択」)。Projectドロワーを開かずに主要シーンを切り替えられるように
  // する——ギャラリーのマニフェスト(`scenes/index.json`)をそのまま再利用し、
  // 二重管理を作らない。読み込み自体は`sceneGalleryRef`(ドロワーのギャラリーと
  // 同じ経路)へ委譲する。
  const sceneSelect = document.getElementById(
    "select-scene",
  ) as HTMLSelectElement | null;
  if (sceneSelect) {
    for (const entry of sceneGalleryManifest()) {
      const option = document.createElement("option");
      option.value = entry.file;
      option.textContent = `${entry.demo} ${entry.title}`;
      sceneSelect.appendChild(option);
    }
    sceneSelect.addEventListener("change", () => {
      const file = sceneSelect.value;
      if (!file) return;
      const json = sceneGalleryFileContent(file);
      if (json && sceneGalleryRef.current) sceneGalleryRef.current(json);
    });
  }

  const consoleDiagnosticsRef: ConsoleDiagnosticsRef = { current: null };
  const { append: appendConsoleEntries, clear: clearConsole } = setUpConsole(
    jumpToStepRef,
    selectBodyRef,
    consoleDiagnosticsRef,
  );
  // `reportError`(モジュール自由関数)からConsoleのErrorsタブへ書けるように
  // する(`errors::`プレフィクスは`append`の`level::message`分割規約に従う、
  // `consoleErrorAppend`のdoc参照)。
  consoleErrorAppend = (message) => appendConsoleEntries(`errors::${message}`);
  const materialsRef: MaterialsRef = { current: null };
  const circuitRef: CircuitRef = { current: null };
  const sceneExportRef: SceneExportRef = { current: null };
  const projectBundleRef: ProjectBundleRef = { current: null };
  const projectExportedRef: ProjectExportedRef = { current: null };
  const sceneImportRef: SceneImportRef = { current: null };
  const replayVerifyRef: ReplayVerifyRef = { current: null };
  const replayPlaybackRef: ReplayPlaybackRef = { current: null };
  const circuitEditorRef: CircuitEditorRef = { current: null };
  const circuitFreeWiringState: CircuitFreeWiringState = { active: false };
  const prefabRef: PrefabRef = { current: null };
  const prefabSaveRef: PrefabSaveRef = { current: null };
  const sceneGalleryRef: SceneGalleryRef = { current: null };
  const circuitElementsRef: CircuitElementsRef = { current: null };
  const validationBaseJsonRef: ValidationBaseJsonRef = { current: null };
  // かんたんモード(`guided.ts`)。`setUpSceneView`(wasm の初期化を含む)より
  // 先に UI を組み立てておく——読み込みが終わって起動オーバーレイが消えた
  // 瞬間に、①のカテゴリ選択が既に目の前にある状態にするため。物理側の窓口
  // (`guidedApiRef`)が埋まるのは初期化の完了時で、それまでに選ばれた実験は
  // 窓口が来た時点で自動的に走り出す(`guided.ts` の `pendingStart`)。
  const guidedApiRef: GuidedApiRef = { current: null };
  setUpGuidedMode(guidedApiRef);
  setUpProjectDrawer(
    materialsRef,
    circuitRef,
    sceneExportRef,
    projectBundleRef,
    projectExportedRef,
    sceneImportRef,
    replayVerifyRef,
    replayPlaybackRef,
    circuitEditorRef,
    circuitFreeWiringState,
    prefabRef,
    prefabSaveRef,
    sceneGalleryRef,
    circuitElementsRef,
    validationBaseJsonRef,
  );
  setUpSceneView(
    updateProbeGraph,
    appendConsoleEntries,
    clearConsole,
    jumpToStepRef,
    materialsRef,
    circuitRef,
    sceneExportRef,
    projectBundleRef,
    projectExportedRef,
    sceneImportRef,
    replayVerifyRef,
    replayPlaybackRef,
    circuitEditorRef,
    circuitFreeWiringState,
    prefabRef,
    prefabSaveRef,
    sceneGalleryRef,
    selectBodyRef,
    circuitElementsRef,
    consoleDiagnosticsRef,
    validationBaseJsonRef,
    guidedApiRef,
  )
    .then(() => {
      markBootReady();
    })
    .catch((err) => {
      const hud = document.getElementById("hud");
      if (hud) hud.textContent = `エラー: ${String(err)}`;
      markBootFailed(String(err));
      console.error(err);
    });
}

main();
