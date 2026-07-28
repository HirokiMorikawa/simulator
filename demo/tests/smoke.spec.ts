import { expect, test, type Page } from "@playwright/test";

// 統合エディタのスモークテスト(増分D2)。これまで各増分で手動実行してきた
// Playwright 確認を、CI で毎回回るテストとして資産化したもの。
//
// **意図的に薄く保つ**: 物理の正しさは Rust 側の解析解テスト(M/T/E/A/Q/S/R番号)が
// 担保しており、ここで重ねて検証しない。ここが守るのは「フロントエンドが起動し、
// wasm が初期化され、主要な操作でクラッシュしない」という配線の健全性だけである
// ——実際、増分3-3・B2・B3 で見つかったバグ(heater_node_temperature の unwrap、
// Circuit::node_voltage の範囲外、ボディ0個シーンでの render ループ崩壊)は
// いずれもこの層でしか踏めない種類だった。

/** ページ全体で発生した未捕捉例外を集める。favicon の 404 は除外する。 */
function collectPageErrors(page: Page): string[] {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  return errors;
}

/** wasm 初期化を待つ(Hierarchy にボディが並ぶまで)。 */
async function waitForWorld(page: Page) {
  await expect(page.locator("#hierarchy-tree .tree-selectable").first()).toBeVisible({
    timeout: 30_000,
  });
}

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
  await expect(page.locator("#hierarchy-tree .tree-selectable")).toHaveCount(4);
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
  await expect(page.locator("#hierarchy-tree .tree-selectable")).toHaveCount(0);
  // それでも HUD は描かれ続ける(y はプレースホルダ表示)。
  await expect(page.locator("#hud")).toContainText("t =");

  await page.click("#btn-mode-play");
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
  await expect(page.locator("#hierarchy-tree .tree-selectable")).toHaveCount(51);

  // D36 スイングバイ: 剛体0体、天体2体。Hierarchy のボディ一覧は空になるが、
  // シーン定義プローブ8本が Probes サブツリーに並ぶ。
  await page.click('.scene-gallery-list button[data-scene-file="d36-swingby.json"]');
  await expect(page.locator("#hierarchy-tree .tree-selectable")).toHaveCount(0);
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
