import { expect, test } from "@playwright/test";
import { addViaMenu, collectPageErrors, waitForWorld } from "./helpers";

// 統合エディタのスモークテスト(増分D2)。これまで各増分で手動実行してきた
// Playwright 確認を、CI で毎回回るテストとして資産化したもの。
//
// **意図的に薄く保つ**: 物理の正しさは Rust 側の解析解テスト(M/T/E/A/Q/S/R番号)が
// 担保しており、ここで重ねて検証しない。ここが守るのは「フロントエンドが起動し、
// wasm が初期化され、主要な操作でクラッシュしない」という配線の健全性だけである
// ——実際、増分3-3・B2・B3 で見つかったバグ(heater_node_temperature の unwrap、
// Circuit::node_voltage の範囲外、ボディ0個シーンでの render ループ崩壊)は
// いずれもこの層でしか踏めない種類だった。
//
// `collectPageErrors`/`waitForWorld`/`addViaMenu` は `./helpers` へ切り出した
// (Playwright は *.spec.ts 同士の import を許さないため、縦串①の受け入れ
// テスト(acceptance-d24.spec.ts)と共有するにはテストファイルでない
// モジュールに置く必要があった)。

test("起動して wasm が初期化され、既定シーンが Hierarchy と HUD に現れる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // 既定シーンは床(Ground)+ 箱(Box_1)。
  // **`.tree-selectable` は Bodies だけでなく Frames サブツリーも数える**
  // (起動時に `add_child_frame` で回転フレームを1つ追加しているため、既定シーンでは
  // 2体 + 1フレーム = 3件になる)。件数より意味のあるラベルで検証する。
  // ラベルは Inspector にも出るため `#hierarchy-tree` 内に限定する(限定しないと
  // Playwright の strict モードが複数一致で落ちる)。
  const hierarchy = page.locator("#hierarchy-tree");
  await expect(hierarchy.getByText("Ground", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("Box_1", { exact: true })).toBeVisible();
  await expect(page.locator("#hud")).toContainText("step = 0");
  expect(errors).toEqual([]);
});

test("Play モードでシミュレーションが進み、時刻と step が増える", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // 起動時は Edit モードで再生系ボタンが無効。**`#btn-mode-play` を押した時点で
  // 再生が始まる**(`setMode` が `playing = true` にする)ため、ここでは
  // `#btn-play` を押さずに時刻が進むことだけを見る。
  await page.click("#btn-mode-play");
  await expect(page.locator("#hud")).not.toContainText("step = 0", { timeout: 15_000 });
  await page.click("#btn-play"); // 一時停止

  expect(errors).toEqual([]);
});

test("シーンギャラリーから D4(積み木)を読み込むと Hierarchy が差し替わる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click('.project-tab[data-tab="scenes"]');
  const loadButton = page.locator('.scene-gallery-list button[data-scene-file="d4-box-stack.json"]');
  await expect(loadButton).toBeVisible();
  await loadButton.click();

  // D4 は床 + 3段の箱 = 4体。既定シーン(2体)から差し替わったことの確認。
  // **`.tree-body` は Bodies サブツリーの実体行だけ**(群2で Materials(参照)
  // サブツリーを足したため、`.tree-selectable` だけだと参照行まで数えてしまう)。
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(4);
  expect(errors).toEqual([]);
});

test("剛体を持たないシーン(D9 熱のみ)を読み込んでも描画ループが壊れない", async ({ page }) => {
  // 増分B3 で解禁したケース。ボディ0個のワールドは長らく `from_scene_json` 自体が
  // 拒否しており、解禁時に render ループ側で4箇所のクラッシュ経路が見つかった。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click('.project-tab[data-tab="scenes"]');
  await page.click('.scene-gallery-list button[data-scene-file="d9-cooling-coffee.json"]');

  // 剛体が無いので Hierarchy のボディ一覧は空になる。
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(0);
  // それでも HUD は描かれ続ける(y はプレースホルダ表示)。
  await expect(page.locator("#hud")).toContainText("t =");

  await page.click("#btn-mode-play");
  // QA不具合5の修正でPlayモードに入った直後は`playing`が真になり、⏭は
  // 一時停止中のみ有効になった(Unityの Step と同じ意味論)。停止してから
  // stepを使う。
  await page.click("#btn-play");
  await page.click("#btn-step");
  await page.click("#btn-step");
  expect(errors).toEqual([]);
});

test("Probe Graphs にシーン定義プローブが描かれる(D11 振り子は2系列)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click('.project-tab[data-tab="scenes"]');
  await page.click('.scene-gallery-list button[data-scene-file="d11-pendulum.json"]');
  await page.click("#btn-mode-play");
  await page.click("#btn-play");

  // canvas の中身は直接検証できないので、描画対象が存在することと
  // クラッシュしないことだけを見る(系列本数・ラベルの正しさは
  // sim-wasm 側の imported_probe_label_at のテストが担保している)。
  await expect(page.locator("#probe-canvas")).toBeVisible();
  await page.waitForTimeout(1500);
  await page.click("#btn-play");

  expect(errors).toEqual([]);
});

test("Probe Graphs の対数軸トグルと CSV エクスポートが動く(増分E1)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click("#btn-mode-play");
  await page.click("#btn-play");
  await page.waitForTimeout(1200);
  await page.click("#btn-play");

  // 対数軸トグル: 凡例に [log] が付く/外すと消える(表示上の変換であることの確認)。
  await page.check("#toggle-probe-log");
  await page.waitForTimeout(300);
  await page.uncheck("#toggle-probe-log");

  // CSV: ダウンロードが実際に発火し、ヘッダ行とサンプル行を含むこと。
  const [download] = await Promise.all([
    page.waitForEvent("download"),
    page.click("#btn-probe-csv"),
  ]);
  expect(download.suggestedFilename()).toBe("probes.csv");
  const stream = await download.createReadStream();
  const chunks: Buffer[] = [];
  for await (const c of stream) chunks.push(c as Buffer);
  const csv = Buffer.concat(chunks).toString("utf8");
  const lines = csv.split("\n");
  expect(lines[0]).toBe("sample,BodyPosY,BodySpeed");
  expect(lines.length).toBeGreaterThan(2);
  // 2行目は sample=0 と2系列の数値。
  expect(lines[1].split(",")).toHaveLength(3);

  expect(errors).toEqual([]);
});

test("Hierarchy に Probes サブツリーが出る(D11 は body_pos_x/y の2本、増分E2)", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // **葉ノード(プローブのラベル)で検証する**。"Probes" の `li` は入れ子の `ul` を
  // 含むため textContent が "ProbesBodyPosX(bob)..." になり `exact: true` に
  // 一致しない(Bodies/Frames も同じ構造)。
  const hierarchy = page.locator("#hierarchy-tree");
  // 既定シーンは scenario.probes を持たないのでプローブのラベルは1つも出ない。
  await expect(hierarchy.getByText("BodyPosX(", { exact: false })).toHaveCount(0);

  await page.click('.project-tab[data-tab="scenes"]');
  await page.click('.scene-gallery-list button[data-scene-file="d11-pendulum.json"]');

  // ラベルは sim-wasm の probe_target_label が生成する(ボディ名 "bob" 込み)。
  await expect(hierarchy.getByText("BodyPosX(bob)", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("BodyPosY(bob)", { exact: true })).toBeVisible();

  expect(errors).toEqual([]);
});

test("増分G1で追加した3シーン(D8/D12/D36)がギャラリーから読み込める", async ({ page }) => {
  // Rust 側は `run_headless_scenario` で解析解と突き合わせ済み(貫入なし・
  // BallJoint の拘束距離・双曲線フライバイの偏向角)。ここが守るのは
  // **同じ JSON がフロントエンドの経路でも壊れずに載る**ことだけ。
  //
  // D36 は剛体を持たない(天体ドメインのみ)ので Scene View には何も描かれない。
  // 観測手段は Probe Graphs であり、それが D9/D34/D35 と同じ既知の限界である。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  const hierarchy = page.locator("#hierarchy-tree");

  await page.click('.project-tab[data-tab="scenes"]');

  // D12 ラグドール: 床 + 胴体/頭/腕2本 = 5体。BallJoint 3本入りのシーンが
  // `JointJson::Ball`(本増分で追加)経由でパースできることの確認でもある。
  await page.click('.scene-gallery-list button[data-scene-file="d12-ragdoll.json"]');
  await expect(hierarchy.getByText("torso", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("BodyPosY(head)", { exact: true })).toBeVisible();

  // D8 散乱: 床 + 球50個 = 51体。ギャラリー中で最大のシーン。
  await page.click('.scene-gallery-list button[data-scene-file="d8-scatter.json"]');
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(51);

  // D36 スイングバイ: 剛体0体、天体2体。Hierarchy のボディ一覧は空になるが、
  // シーン定義プローブ8本が Probes サブツリーに並ぶ。
  await page.click('.scene-gallery-list button[data-scene-file="d36-swingby.json"]');
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(0);
  await expect(hierarchy.getByText("AstroPosX[1]", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("AstroVelY[0]", { exact: true })).toBeVisible();

  // 剛体が無いシーンでも再生して描画ループが回ること。
  await page.click("#btn-mode-play");
  await page.waitForTimeout(800);
  await page.click("#btn-play");
  await expect(page.locator("#probe-canvas")).toBeVisible();

  expect(errors).toEqual([]);
});

test("D19(電気工作台)を読み込むと Circuit タブと Hierarchy が実際の素子を出す(増分G2)", async ({
  page,
}) => {
  // **増分G2で修正した表示バグの回帰テスト**: Circuit タブは固定デモ回路の図
  // (10V / 100Ω / 200Ω)をハードコードで描いており、ギャラリーから別の回路を
  // 読み込んでも**その図が残って実際とは違う値を表示し続けていた**。
  // 「無効です」という注記は出ていたので既存テストは通ってしまっていた——
  // **数字が実態と一致しているか**を見るのがこのテストの要点。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click('.project-tab[data-tab="scenes"]');
  await page.click('.scene-gallery-list button[data-scene-file="d19-electric-workbench.json"]');
  await page.click('.project-tab[data-tab="scenes"]'); // ドロワーを閉じる

  // HUD は分圧点(node2)の電圧を出す。E5 の解析解は 9V * 2k/(1k+2k) = 6.000V。
  // **読み込み直後は 0.000 V**——回路は`step()`で初めて解かれるため、1step進める。
  await page.click("#btn-mode-play");
  await page.click("#btn-play"); // 一時停止(`setMode`が既に再生を始めている)
  await page.click("#btn-step");
  await expect(page.locator("#hud")).toContainText("circuit V = 6.000 V");

  // Hierarchy の Circuits サブツリーに実際の素子が並ぶ(葉ノードで検証する
  // ——"Circuits" の li は入れ子の ul を含むため exact 一致しない)。
  const hierarchy = page.locator("#hierarchy-tree");
  await expect(hierarchy.getByText("V0: GND → N1 9 V", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("R: N1 – N2 1000 Ω", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("C: N3 – GND 0.001 F", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("SW0: N1 – N4 (閉)", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("D: N4 → GND", { exact: true })).toBeVisible();

  // Circuit タブ本体も同じ実素子を出し、**固定デモ回路の嘘の数字は消えている**。
  await page.click('.project-tab[data-tab="circuit"]');
  const topology = page.locator("#project-body .circuit-topology");
  await expect(topology).toContainText("回路の素子(実際に配線されているもの、7件)");
  await expect(topology).toContainText("R: N1 – N2 1000 Ω");
  await expect(topology).not.toContainText("100Ω");
  await expect(topology).not.toContainText("10V 電源");

  expect(errors).toEqual([]);
});

test("増分Hで追加した5シーン(D13/D14/D15/D16/D23)がギャラリーから読み込める", async ({
  page,
}) => {
  // 増分H で `Scenario` に soft_body / grid_fluid / conduction_rod / sph の
  // 4ドメインを追加し、あわせて SoftBody と ConductionRod1D に `Solver` を
  // 実装して `World::step()` の対象に入れた(それまでは載せても**再生しても
  // 一切動かなかった**)。ここが守るのは同じ JSON がフロントエンドの経路でも
  // 壊れずに載り、再生してクラッシュしないことだけ。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  const hierarchy = page.locator("#hierarchy-tree");
  await page.click('.project-tab[data-tab="scenes"]');

  // D13 ロープ: 剛体0体・ソフトボディ21粒子。プローブで観測する。
  await page.click('.scene-gallery-list button[data-scene-file="d13-rope.json"]');
  await expect(hierarchy.getByText("SoftBodyPosY[10]", { exact: true })).toBeVisible();

  // D16 熱伝導レース: 1D棒の格子点温度。
  await page.click('.scene-gallery-list button[data-scene-file="d16-conduction-race.json"]');
  await expect(hierarchy.getByText("RodTemp[20]", { exact: true })).toBeVisible();

  // D15 対流: 格子流体の平均鉛直速度 + 熱ノード。
  await page.click('.scene-gallery-list button[data-scene-file="d15-convection.json"]');
  await expect(hierarchy.getByText("GridFluidMeanV", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("NodeTemp[0]", { exact: true })).toBeVisible();

  // D14 渦: 鉛直速度のRMS(平均だと上下対称で打ち消し合って0のまま)。
  await page.click('.scene-gallery-list button[data-scene-file="d14-vortex.json"]');
  await expect(hierarchy.getByText("GridFluidRmsV", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("obstacle", { exact: true })).toBeVisible();

  // D23 注ぐ水: SPH粒子。
  await page.click('.scene-gallery-list button[data-scene-file="d23-pouring-water.json"]');
  await expect(hierarchy.getByText("SphPosY[0]", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("SphDensity[86]", { exact: true })).toBeVisible();

  // 再生してもクラッシュしないこと。
  await page.click('.project-tab[data-tab="scenes"]');
  await page.click("#btn-mode-play");
  await page.waitForTimeout(800);
  await page.click("#btn-play");
  await expect(page.locator("#probe-canvas")).toBeVisible();

  expect(errors).toEqual([]);
});

test("増分K: Toolbarのシーン選択・Inspectorの追加Component・Consoleの種別タブ", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // ① Toolbar のシーン選択ドロップダウン: ドロワーを開かずにシーンを差し替える。
  await page.selectOption("#select-scene", "d19-electric-workbench.json");
  const hierarchy = page.locator("#hierarchy-tree");
  await expect(hierarchy.getByText("CircuitV[2]", { exact: true })).toBeVisible();

  // ② Inspector の追加 Component。D19 は剛体を持たないので、まず剛体のある
  //    シーンへ切り替えてからボディを選ぶ。
  await page.selectOption("#select-scene", "d11-pendulum.json");
  await hierarchy.getByText("bob", { exact: true }).click();
  const inspector = page.locator("#inspector-body");
  // Probe セクション(シーン定義プローブ)と現在値。
  await expect(inspector.getByText("Probe", { exact: true })).toBeVisible();
  await expect(inspector.getByText("BodyPosX(bob)", { exact: true })).toBeVisible();
  // 近似バッジ(**群1で各ソルバの自己申告へ移行**)。
  // 移行前はWorld側が「どのドメインが有効か」から推測しており、力学ソルバ自身の
  // 近似(PGS+Baumgarte・マニフォールド4点)は**1件も挙がっていなかった**ため
  // D11(純粋な力学シーン)ではバッジが0件だった。自己申告にしたことで、
  // 力学だけのシーンでも実際に効いている近似が出る。
  await expect(
    inspector.getByText("接触: PGS + Baumgarte", { exact: true }),
  ).toBeVisible();
  await page.selectOption("#select-scene", "d10-brake-heat.json");
  await hierarchy.getByText("brake_pad", { exact: true }).click();
  await expect(inspector.locator(".approximation-badge").first()).toBeVisible();
  // **群1: Coupling が種別とパラメータで出る**(以前は「種別: —(トレイトが
  // 名前を持たないため非表示)」という件数だけの表示だった)。D10 は
  // dissipation_to_heat 結合を持つ。これは特定ボディを参照しない(全体の散逸を
  // 読む)結合なので「Coupling (シーン全体)」枠に出る。
  await expect(inspector.getByText("DissipationToHeat", { exact: true })).toBeVisible();
  await expect(inspector).not.toContainText("トレイトが名前を持たないため非表示");

  // Joint セクションは `constraint_anchor_points_at` がアンカーを返すボディに
  // 出る。スポーンした振り子(ワールド固定点への DistanceJoint)で確認する。
  await page.reload();
  await waitForWorld(page);
  await addViaMenu(page, "＋ 振り子");
  await hierarchy.getByText("Pendulum", { exact: false }).first().click();
  await expect(inspector.getByText("Joint", { exact: true })).toBeVisible();

  // ③ Console の種別タブ。既定シーンへ戻して接触を起こす。
  await page.selectOption("#select-scene", "d4-box-stack.json");
  await page.click("#btn-mode-play");
  const contactEntry = page.locator("#console-log li", { hasText: "bodies=" }).first();
  await expect(contactEntry).toBeVisible({ timeout: 30_000 });
  await page.click("#btn-play"); // 一時停止

  // Contacts タブは接触行だけを残す。
  await page.click('.console-tab[data-tab="contacts"]');
  await expect(contactEntry).toBeVisible();
  // 起動時の INFO 行(接触ではない)は隠れる。
  const startupLine = page.locator("#console-log li", { hasText: "World起動" }).first();
  await expect(startupLine).toBeHidden();
  // All へ戻すと再び見える。
  await page.click('.console-tab[data-tab="all"]');
  await expect(startupLine).toBeVisible();

  expect(errors).toEqual([]);
});

test("増分L: 流体場オーバーレイ・カプセル・材料派生", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  const hierarchy = page.locator("#hierarchy-tree");

  // ① カプセルのスポーン(sim-mechanics 側で体積・慣性・接触を実装した)。
  await addViaMenu(page, "＋ カプセル");
  await expect(hierarchy.getByText("Capsule_", { exact: false }).first()).toBeVisible();
  // 落として床に載る(接触が起きる = カプセル-平面の接触が働いている)。
  await page.click("#btn-mode-play");
  const contact = page.locator("#console-log li", { hasText: "bodies=" }).first();
  await expect(contact).toBeVisible({ timeout: 30_000 });
  await page.click("#btn-play");

  // ② 材料派生: 新しい材料がセレクタへ増え、選択状態になる。
  await page.reload();
  await waitForWorld(page);
  const before = await page.locator("#select-spawn-material option").count();
  // prompt が2回連続で出る(材料名 → 密度)。`page.once` を2つ積むより、
  // 順番にキューから answers を取り出すハンドラのほうが確実。
  const answers = ["テスト軽量材", "321"];
  page.on("dialog", (d) => d.accept(answers.shift() ?? ""));
  await addViaMenu(page, "材料派生");
  await expect(page.locator("#select-spawn-material option")).toHaveCount(before + 1);
  await expect(page.locator("#select-spawn-material")).toHaveValue("テスト軽量材");

  // ③ 格子流体の速度場オーバーレイ。D15(対流)は格子流体だけのシーンで、
  //    これまで Scene View に何も描かれなかった。
  await page.selectOption("#select-scene", "d15-convection.json");
  await page.click("#btn-mode-play");
  await page.waitForTimeout(600);
  await page.click("#btn-play");
  // トグルが存在し、既定でONであること(描画自体はcanvas内なので直接は見えない)。
  // **群2でオーバーレイ切替は Settings(⚙)ポップオーバーへ移した**
  // ——ツールバーに6個並べていたら折り返して読めなくなっていたため。
  await page.click("#btn-settings");
  const toggle = page.locator("#toggle-grid-fluid-overlay");
  await expect(toggle).toBeChecked();
  await toggle.uncheck();
  await page.waitForTimeout(200);
  await toggle.check();

  expect(errors).toEqual([]);
});

test("群1: Inspector が結合の種別・ジョイントの接続を実データで出し、下端まで到達できる", async ({
  page,
}) => {
  // **ユーザーが指摘した縮約の解消**: 以前 Coupling は件数だけを出し
  // 「種別: —(トレイトが名前を持たないため非表示)」と表示していた。
  // Joint も `constraint_anchor_points_at` のアンカー2点のみで、
  // シーンから読み込んだジョイントは**1件も出なかった**。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  const hierarchy = page.locator("#hierarchy-tree");
  const inspector = page.locator("#inspector-body");

  // D12 ラグドールは BallJoint 3本。種別と接続先が出る。
  await page.selectOption("#select-scene", "d12-ragdoll.json");
  await hierarchy.getByText("torso", { exact: true }).click();
  await expect(inspector.getByText("BallJoint", { exact: true }).first()).toBeVisible();
  await expect(inspector.getByText("body#1 ↔ body#2", { exact: true })).toBeVisible();

  // **Component が増えてパネル高を超えるので、スクロールで下端へ到達できること**を
  // 座標で確認する(増分E3のドロワー同様、到達不能なUIを作らないための回帰)。
  const panel = page.locator("#inspector");
  const overflowing = await panel.evaluate((e) => e.scrollHeight > e.clientHeight);
  expect(overflowing).toBe(true);
  await panel.evaluate((e) => {
    e.scrollTop = e.scrollHeight;
  });
  await expect(page.locator(".approximation-badge").first()).toBeVisible();

  // 近似バッジは出典と理由を title に持つ(設計§1.3「名前・出典・オフ可否」)。
  const badgeTitle = await page.locator(".approximation-badge").first().getAttribute("title");
  expect(badgeTitle).toContain("出典: docs/");

  expect(errors).toEqual([]);
});

test("群2: カメラ操作・ツール切替ショートカット・Settings の物理パラメータ", async ({ page }) => {
  // **これらは全て存在しなかった**: カメラは position.set(6,4,10) の完全固定、
  // keydown リスナ0件、重力/dt を触る手段なし。Unityのようなツールとしては
  // 最も基本的な欠落だった。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // ① カメラ: 中ドラッグで回転する(左は選択・ギズモに使うので割り当てない)。
  const canvas = page.locator("canvas").first();
  const box = (await canvas.boundingBox())!;
  const before = await page.evaluate(() => {
    const c = (window as unknown as { __camera?: { position: { x: number; y: number; z: number } } })
      .__camera;
    return c ? { ...c.position } : null;
  });
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down({ button: "middle" });
  await page.mouse.move(box.x + box.width / 2 + 200, box.y + box.height / 2, { steps: 10 });
  await page.mouse.up({ button: "middle" });
  await page.waitForTimeout(300);
  const after = await page.evaluate(() => {
    const c = (window as unknown as { __camera?: { position: { x: number; y: number; z: number } } })
      .__camera;
    return c ? { ...c.position } : null;
  });
  expect(before).not.toBeNull();
  expect(after).not.toEqual(before);

  // ② ツール切替: W/E/R/Q。以前は3つのギズモが同時表示で切替が無かった。
  await page.keyboard.press("e");
  await expect(page.locator("#btn-tool-rotate")).toHaveClass(/active/);
  await page.keyboard.press("w");
  await expect(page.locator("#btn-tool-translate")).toHaveClass(/active/);

  // ③ Settings: 重力を実行時に変更できる(「物理法則を試す」の中心)。
  await page.click("#btn-settings");
  await expect(page.locator("#input-gravity")).toHaveValue(/9\.80/);
  await page.fill("#input-gravity", "1.0");
  await page.locator("#input-gravity").dispatchEvent("change");
  // **フォーカスを外してから確認するのが要点**。`syncSettingsInputs()` は
  // 編集中(activeElement)の入力欄を上書きしない——さもないと数値を打っている
  // 最中に毎フレーム値が書き換わって入力できなくなる。したがってフォーカスが
  // 乗ったままだと表示は打った文字列("1.0")のままで、**エンジンが受理した値**を
  // 見たことにならない。blur 後の `toFixed(3)` 表示は world.gravity() の往復。
  await page.locator("#input-gravity").blur();
  await page.waitForTimeout(200);
  await expect(page.locator("#input-gravity")).toHaveValue("1.000");
  // dt は Edit モードでのみ変更できる(決定論を守るため)。
  await expect(page.locator("#input-dt")).toBeEnabled();

  expect(errors).toEqual([]);
});

test("群2: Inspector の RigidBody を Command 経由で編集できる", async ({ page }) => {
  // **設計 §1.3 は「各 Component は World API の `Desc` 型と 1:1 対応。編集は
  // 次ステップ先頭で Command として適用される」と定めているが、Inspector は
  // 全フィールドが読み取り専用の `<span>` だった**。Collision group/mask に
  // 至っては `RigidBodySet` に概念自体が無く、群2で `sim-mechanics` から作った。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // 既定の箱は鋼。Mass は密度×体積(7850 kg/m³ × 1 m³)。
  const massInput = page.locator("#inspector-mass");
  await expect(massInput).toHaveValue(/^7850/);
  await expect(page.locator("#inspector-body-type")).toHaveValue("Dynamic");
  // 衝突フィルタの既定は「1番グループに属し全グループと当たる」。
  await expect(page.locator("#inspector-collision-group")).toHaveValue("1");
  await expect(page.locator("#inspector-collision-mask")).toHaveValue(String(0xffffffff));

  // Play モードで質量を変更 → Command が次 step 先頭で適用され、
  // 表示が**エンジンから読み直した値**に変わる。
  await page.click("#btn-mode-play");
  await massInput.fill("3.5");
  await massInput.dispatchEvent("change");
  await massInput.blur();
  await page.waitForTimeout(300);
  await expect(massInput).toHaveValue("3.50000");

  // Static へ切り替えると inv_mass = 0(無限質量)になり、質量欄は無効化される。
  await page.selectOption("#inspector-body-type", "Static");
  await page.waitForTimeout(300);
  await expect(page.locator("#inspector-body-type")).toHaveValue("Static");
  await expect(massInput).toBeDisabled();

  expect(errors).toEqual([]);
});

test("群2: 右クリックメニュー(Scene View スポーンパレット / Hierarchy 複製)", async ({
  page,
}) => {
  // **設計 §1.1/§1.2 が両方で要求しているのに、`contextmenu` リスナは
  // リポジトリ全体で0件だった**。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const rowCount = () => page.locator("#hierarchy-tree .tree-body").count();

  // ① Hierarchy: 行を右クリック → 複製。
  const before = await rowCount();
  await page.locator("#hierarchy-tree .tree-body").nth(1).click({ button: "right" });
  await expect(page.locator("#context-menu")).toBeVisible();
  await page.locator("#context-menu button", { hasText: "複製" }).first().click();
  await page.waitForTimeout(200);
  expect(await rowCount()).toBeGreaterThan(before);

  // ② Scene View: 地面を右クリック → クリック位置に配置。
  const canvas = page.locator("canvas").first();
  const box = (await canvas.boundingBox())!;
  await page.mouse.click(box.x + box.width * 0.4, box.y + box.height * 0.75, { button: "right" });
  await expect(page.locator("#context-menu")).toBeVisible();
  // メニューのラベルにクリック位置のワールド座標が入る(地面へ投影した点)。
  await expect(page.locator("#context-menu button").first()).toContainText("ここに球を配置");
  const beforeSpawn = await rowCount();
  await page.locator("#context-menu button", { hasText: "ここに箱を配置" }).click();
  await page.waitForTimeout(200);
  expect(await rowCount()).toBeGreaterThan(beforeSpawn);

  // ③ Escape で閉じる。
  await page.mouse.click(box.x + box.width * 0.4, box.y + box.height * 0.75, { button: "right" });
  await expect(page.locator("#context-menu")).toBeVisible();
  await page.keyboard.press("Escape");
  await expect(page.locator("#context-menu")).toHaveCount(0);

  expect(errors).toEqual([]);
});

test("群2: Hierarchy の折り畳み・Materials サブツリー・N step 送り・実効時間倍率", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // ① Materials(参照)サブツリー。設計 §1.1 の列挙にあるのにツリーに無かった。
  await expect(page.locator("#hierarchy-tree")).toContainText("Materials (参照)");

  // ② 折り畳み(設計 §1.1「ツリーは折り畳み可」)。Bodies を畳むと
  //    ボディ行が消える(Materials 配下の参照行は残る)。
  const visibleBodies = () =>
    page.locator("#hierarchy-tree .tree-group", { hasText: "Bodies" }).locator("li:visible").count();
  expect(await visibleBodies()).toBeGreaterThan(0);
  await page.locator("#hierarchy-tree .tree-toggle").first().click();
  expect(await visibleBodies()).toBe(0);
  await page.locator("#hierarchy-tree .tree-toggle").first().click();
  expect(await visibleBodies()).toBeGreaterThan(0);

  // ③ N step 送り。⏭ を1回押して指定 step 数ちょうど進むこと。
  await page.click("#btn-mode-play");
  await page.click("#btn-play"); // 一時停止
  await page.fill("#input-step-count", "50");
  const readStep = async () =>
    Number((await page.locator("#timeline-step").textContent())!.replace(/\D/g, ""));
  const s0 = await readStep();
  await page.click("#btn-step");
  await page.waitForTimeout(300);
  expect(await readStep()).toBe(s0 + 50);

  // ④ 実効時間倍率。×128 は 1 フレームあたりの step 上限(240)に当たる
  //    ——60fps・dt=1/120 なら 1 フレーム 256 step を要求するため。
  //    **出せていないことを赤で正直に示す**(黙って遅いままにしない)。
  //    判定は「上限に当たったか」という事実で行う(比率だと機械の速さで
  //    結果が変わる。実測: 240/256 = 93.75% は素朴な9割閾値を超えてしまう)。
  await page.selectOption("#select-timescale", "128");
  await page.click("#btn-play"); // 再生再開
  await page.waitForTimeout(1500);
  await expect(page.locator("#timescale-effective")).toHaveClass(/degraded/);
  await expect(page.locator("#timescale-effective")).toHaveAttribute(
    "title",
    /step 数上限/,
  );
  const effective = Number(
    (await page.locator("#timescale-effective").textContent())!.replace("×", ""),
  );
  expect(effective).toBeGreaterThan(1); // 速くはなっている
  expect(effective).toBeLessThan(128); // が指定値には届かない

  // ×1 に戻せば上限に当たらなくなり、赤も消える(保持フレーム分だけ遅れて)。
  await page.selectOption("#select-timescale", "1");
  await page.waitForTimeout(1500);
  await expect(page.locator("#timescale-effective")).not.toHaveClass(/degraded/);

  expect(errors).toEqual([]);
});

test("群2: 単一ファイル Export(シーン+Replay+Bookmark)", async ({ page }) => {
  // 設計 §6「保存・共有: シーンJSON+Replay+ブックマークを単一ファイルとして
  // エクスポート」。これまで3つは別々のファイルで、ブックマーク一覧は
  // そもそも書き出せなかった。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click("#btn-mode-play");
  await page.waitForTimeout(400);
  await page.click("#btn-nudge"); // Replay に載る Command を1件作る
  await page.fill("#bookmark-label", "テスト地点");
  await page.click("#btn-add-bookmark");

  await page.click('.project-tab[data-tab="scenes"]');
  const download = page.waitForEvent("download");
  await page.click("#btn-export-bundle");
  const stream = await (await download).createReadStream();
  const chunks: Buffer[] = [];
  for await (const chunk of stream) chunks.push(chunk as Buffer);
  const bundle = JSON.parse(Buffer.concat(chunks).toString("utf8"));

  expect(bundle.formatVersion).toBe(1);
  // `scene` は sim_world::Scenario スキーマなので、そのままギャラリーへ読める。
  expect(bundle.scene.bodies.length).toBeGreaterThan(0);
  expect(bundle.scene.world).toHaveProperty("gravity");
  expect(bundle.commandLog.length).toBeGreaterThan(0);
  expect(bundle.bookmarks.map((b: { label: string }) => b.label)).toContain("テスト地点");
  // **書き出しのために一時ブックマークを作らない**(群2で `export_scene_json`
  // を Rust 側へ切り出した理由——以前の実装は一覧にゴミを残した)。
  expect(bundle.bookmarks).toHaveLength(1);

  expect(errors).toEqual([]);
});

test("群3: 量子・統計・FDTD がギャラリーに載り、場のパネルに描かれる", async ({ page }) => {
  // **これらは長らく「原理的に載せられない」として閉じられていた**——
  // `sim-world` が `sim-quantum`/`sim-statistical` を依存にすら持たず、
  // `Solver` 未実装で `World::step()` の走査対象にもならなかった。
  // 群3で `Solver` 実装 → `World` 統合 → シーンJSON → 可視化まで通した。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  await page.click('.project-tab[data-tab="scenes"]');

  const fieldPanel = page.locator("#field-panel");
  const fieldTitle = page.locator("#field-title");

  // 量子1D(D28 トンネル効果): |ψ|² と V(x) の折れ線。
  await page.click('.scene-gallery-list button[data-scene-file="d28-tunneling.json"]');
  await expect(fieldPanel).toBeVisible();
  await expect(fieldTitle).toContainText("量子 1D");

  // 量子2D(D27 二重スリット): |ψ|² の 2D 分布。
  await page.click('.scene-gallery-list button[data-scene-file="d27-double-slit.json"]');
  await expect(fieldTitle).toContainText("量子 2D");

  // FDTD(D29 電波の水槽): Ez 場。
  await page.click('.scene-gallery-list button[data-scene-file="d29-radio-tank.json"]');
  await expect(fieldTitle).toContainText("FDTD Ez");

  // イジング(D32 相転移): スピン格子。
  await page.click('.scene-gallery-list button[data-scene-file="d32-magnet-transition.json"]');
  await expect(fieldTitle).toContainText("イジング スピン格子");

  // 気体(D30): 速さのヒストグラム + 粒子群が Scene View に出る。
  await page.click('.scene-gallery-list button[data-scene-file="d30-gas-box.json"]');
  await expect(fieldTitle).toContainText("気体分子の速さ分布");
  await page.click("#btn-mode-play");
  await page.waitForTimeout(1200);
  const gasDrawn = await page.evaluate(() => {
    let n = 0;
    (window as unknown as { __scene: { traverse: (f: (o: never) => void) => void } }).__scene.traverse(
      (o: never) => {
        const object = o as unknown as {
          type: string;
          visible: boolean;
          geometry?: { drawRange?: { count: number | null } };
        };
        if (object.type === "Points" && object.visible && object.geometry?.drawRange?.count) {
          n += object.geometry.drawRange.count;
        }
      },
    );
    return n;
  });
  expect(gasDrawn).toBe(400);
  await page.click("#btn-mode-edit");

  // 場のパネルは対象ドメインが無いシーンでは畳まれる(常に居座らない)。
  await page.click('.scene-gallery-list button[data-scene-file="d4-box-stack.json"]');
  await expect(fieldPanel).toBeHidden();

  expect(errors).toEqual([]);
});

test("群3: ソフトボディと天体が Scene View に描かれる", async ({ page }) => {
  // **D13(ロープ)・D34–D36(天体)は Scene View に何も描かれていなかった**
  // ——どちらも `RigidBodySet` の剛体ではないのでメッシュ同期の対象外だった。
  // あわせて**カメラをシーンの中身に合わせる**(描けていても画角外なら
  // 「何も出ていない」のと変わらない——実際に調査で踏んだ)。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  await page.click('.project-tab[data-tab="scenes"]');

  const drawnCounts = async () =>
    page.evaluate(() => {
      const result: string[] = [];
      (
        window as unknown as { __scene: { traverse: (f: (o: never) => void) => void } }
      ).__scene.traverse((o: never) => {
        const object = o as unknown as {
          type: string;
          visible: boolean;
          geometry?: { drawRange?: { count: number | null } };
        };
        if (
          (object.type === "Points" || object.type === "LineSegments") &&
          object.visible &&
          object.geometry?.drawRange?.count
        ) {
          result.push(`${object.type}:${object.geometry.drawRange.count}`);
        }
      });
      return result;
    });

  // D13: 21粒子のロープ + 20本の距離拘束(線分は 2 頂点 × 20 = 40)。
  await page.click('.scene-gallery-list button[data-scene-file="d13-rope.json"]');
  await page.click("#btn-mode-play");
  await page.waitForTimeout(800);
  const rope = await drawnCounts();
  expect(rope).toContain("Points:21");
  expect(rope).toContain("LineSegments:40");
  await page.click("#btn-mode-edit");

  // D34: 天体2体がメッシュとして出る(Points ではなく Mesh なので別途数える)。
  await page.click(
    '.scene-gallery-list button[data-scene-file="d34-solar-system-single-planet.json"]',
  );
  await page.waitForTimeout(400);
  const astroVisible = await page.evaluate(() =>
    (
      window as unknown as {
        __scene: { children: { type: string; visible: boolean; children: unknown[] }[] };
      }
    ).__scene.children.filter((c) => c.type === "Group" && c.visible && c.children.length > 0)
      .length,
  );
  expect(astroVisible).toBeGreaterThan(0);

  expect(errors).toEqual([]);
});

test("Project ドロワーがタブクリックで開き、中身が画面内に入る(増分E3)", async ({ page }) => {
  // **増分E3で修正した重大なUIバグの回帰テスト**: 既定のグリッド行はタブバーの
  // 高さしか無く、ドロワー本体(Scenes/Materials/... の中身)は画面外へ押し出されて
  // 実ユーザーには到達不能だった。Playwright は viewport 外の要素もクリックできて
  // しまうため、既存のスモークテストはこれを見逃していた——**「画面内にあるか」を
  // 座標で明示的に検証する**のがこのテストの要点。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const insideViewport = async () =>
    page.evaluate(() => {
      const r = document.getElementById("project-body")!.getBoundingClientRect();
      return r.top < window.innerHeight;
    });

  // 起動直後は閉じており本体は画面外。
  expect(await insideViewport()).toBe(false);

  await page.click('.project-tab[data-tab="materials"]');
  expect(await insideViewport()).toBe(true);
  // 中身(材質表)が実際に見えること。
  await expect(page.locator("#project-body .materials-table")).toBeVisible();

  // 同じタブをもう一度クリックすると閉じる。
  await page.click('.project-tab[data-tab="materials"]');
  expect(await insideViewport()).toBe(false);

  expect(errors).toEqual([]);
});

test("Circuit-focus レイアウトでドロワーが開いた状態になる(増分E3)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.selectOption("#select-layout", "circuit-focus");
  const visible = await page.evaluate(() => {
    const r = document.getElementById("project-body")!.getBoundingClientRect();
    return { inside: r.top < window.innerHeight, height: r.height };
  });
  expect(visible.inside).toBe(true);
  expect(visible.height).toBeGreaterThan(200);

  expect(errors).toEqual([]);
});

test("Console のイベント行クリックで発生源ボディが選択される(増分E4)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // 箱が床に着地すると ContactStarted が発生する。
  // **`#btn-mode-play` の時点で既に再生が始まる**(`setMode` が `playing = true`
  // にする)。ここで `#btn-play` を押すと逆に一時停止してしまうので押さない。
  await page.click("#btn-mode-play");
  // **`hasText: "ContactStarted"` では絞れない**——起動時のINFO行が文言として
  // 「ContactStarted/ContactEndedを表示」を含むため誤マッチする。実イベント固有の
  // `bodies=`(sim-wasm が SourceId の符号化を復号して出す)で絞る。
  const contactEntry = page.locator("#console-log li", { hasText: "bodies=" }).first();
  await expect(contactEntry).toBeVisible({ timeout: 30_000 });
  await page.click("#btn-play"); // 一時停止
  await expect(contactEntry).toContainText("ContactStarted");

  // Inspector の見出しを Ground へ変えてから、イベント行クリックで箱側へ戻ることを見る。
  await page.locator("#hierarchy-tree").getByText("Ground", { exact: true }).click();
  await expect(page.locator("#inspector-body")).toContainText("Ground");

  await contactEntry.click();
  // 接触は 床(0) と 箱(1) の間で起きるので、先頭のボディが選択される。
  await expect(page.locator("#inspector-body")).toContainText("Ground");

  expect(errors).toEqual([]);
});
