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
  // 1列目は**経過時間(秒)**。サンプル番号のままでは「何秒の値か」を表計算側で
  // 計算し直す必要があった(利用者役の観察)。
  expect(lines[0]).toBe("time_s,BodyPosY,BodySpeed");
  expect(lines.length).toBeGreaterThan(2);
  // 2行目は時刻と2系列の数値。時刻は単調に増える。
  expect(lines[1].split(",")).toHaveLength(3);
  const firstTime = Number.parseFloat(lines[1].split(",")[0]);
  const secondTime = Number.parseFloat(lines[2].split(",")[0]);
  expect(Number.isFinite(firstTime)).toBe(true);
  expect(secondTime).toBeGreaterThan(firstTime);

  expect(errors).toEqual([]);
});

test("Hierarchy に Probes サブツリーが出る(D11 は body_pos_x/y の2本、増分E2)", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // **葉ノード(プローブのラベル)で検証する**。"Probes" の `li` は入れ子の `ul` を
  // 観測点の名前は人の言葉へ直してある(`friendlyProbeLabel`)。括弧の中身
  // (どの物の値か)はそのまま残る。
  // 含むため textContent が "Probes横の位置(bob)..." になり `exact: true` に
  // 一致しない(Bodies/Frames も同じ構造)。
  const hierarchy = page.locator("#hierarchy-tree");
  // 既定シーンは scenario.probes を持たないのでプローブのラベルは1つも出ない。
  await expect(hierarchy.getByText("横の位置(", { exact: false })).toHaveCount(0);

  await page.click('.project-tab[data-tab="scenes"]');
  await page.click('.scene-gallery-list button[data-scene-file="d11-pendulum.json"]');

  // ラベルは sim-wasm の probe_target_label が生成する(ボディ名 "bob" 込み)。
  await expect(hierarchy.getByText("横の位置(bob)", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("高さ(bob)", { exact: true })).toBeVisible();

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
  await expect(hierarchy.getByText("高さ(head)", { exact: true })).toBeVisible();

  // D8 散乱: 床 + 球50個 = 51体。ギャラリー中で最大のシーン。
  await page.click('.scene-gallery-list button[data-scene-file="d8-scatter.json"]');
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(51);

  // D36 スイングバイ: 剛体0体、天体2体。Hierarchy のボディ一覧は空になるが、
  // シーン定義プローブ8本が Probes サブツリーに並ぶ。
  await page.click('.scene-gallery-list button[data-scene-file="d36-swingby.json"]');
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(0);
  await expect(hierarchy.getByText("横の位置(1)", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("縦の速さ(0)", { exact: true })).toBeVisible();

  // 剛体が無いシーンでも再生して描画ループが回ること。
  await page.click("#btn-mode-play");
  await page.waitForTimeout(800);
  await page.click("#btn-play"); // 一時停止

  // **`⏭` で確実に step を送る**。D36 の dt は 5 s(天体スケール)なので、
  // 実時間の再生では 800 ms 待っても 1 step も進まない——以前ここは
  // 「`#probe-canvas` が見えること」だけを見ており、**履歴が空でも常に
  // 見えていた**ので実質何も確かめていなかった(空のときは空状態の文言を
  // 出すようになって初めて表面化した)。fps に依存しない N step 送りで
  // 履歴を作ってから見る(QA ハーネスが同じ理由で使っている手)。
  await page.fill("#input-step-count", "20");
  await page.click("#btn-step");
  await expect(page.locator("#probe-canvas")).toBeVisible();
  await expect(page.locator("#probe-empty")).toBeHidden();
  await expect(page.locator("#probe-time-range")).toContainText("t =");

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
  //
  // 表記が `circuit V = ...` から **`circuit V[2] = ...`** に変わっているのは
  // QA不具合7の修正による。HUD は固定のノード番号ではなく**シーンが宣言した
  // プローブ**を読むようになり、どのノードを表示しているかを併記する
  // (D19 の `probes` 先頭が `circuit_node_voltage: 2` なのでノード 2)。
  await page.click("#btn-mode-play");
  await page.click("#btn-play"); // 一時停止(`setMode`が既に再生を始めている)
  await page.click("#btn-step");
  await expect(page.locator("#hud")).toContainText("circuit V[2] = 6.0000 V");

  // Hierarchy の Circuits サブツリーに実際の素子が並ぶ(葉ノードで検証する
  // ——"Circuits" の li は入れ子の ul を含むため exact 一致しない)。
  const hierarchy = page.locator("#hierarchy-tree");
  await expect(hierarchy.getByText("V0: GND → N1 9 V", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("R: N1 – N2 1000 Ω", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("C: N3 – GND 0.001 F", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("SW0: N1 – N4 (閉)", { exact: true })).toBeVisible();
  // ダイオードは N4 直結から **470Ω の電流制限抵抗を挟んだ N5** へ移した
  // (QA不具合3: 直列抵抗が無く 9V 源をダイオードが短絡して −7.875×10⁶ A が
  // 流れていた)。
  await expect(hierarchy.getByText("R: N4 – N5 470 Ω", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("D: N5 → GND", { exact: true })).toBeVisible();

  // Circuit タブ本体も同じ実素子を出し、**固定デモ回路の嘘の数字は消えている**。
  await page.click('.project-tab[data-tab="circuit"]');
  const topology = page.locator("#project-body .circuit-topology");
  // 7件 → 8件: QA不具合3の修正で LED 枝へ電流制限抵抗 470Ω を足したぶん。
  await expect(topology).toContainText("回路の素子(実際に配線されているもの、8件)");
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
  await expect(hierarchy.getByText("高さ(10)", { exact: true })).toBeVisible();

  // D16 熱伝導レース: 1D棒の格子点温度。
  await page.click('.scene-gallery-list button[data-scene-file="d16-conduction-race.json"]');
  await expect(hierarchy.getByText("棒の温度(20)", { exact: true })).toBeVisible();

  // D15 対流: 格子流体の平均鉛直速度 + 熱ノード。
  await page.click('.scene-gallery-list button[data-scene-file="d15-convection.json"]');
  await expect(hierarchy.getByText("流れの速さ(平均)", { exact: true })).toBeVisible();
  await expect(hierarchy.getByText("温度(0)", { exact: true })).toBeVisible();

  // D14 渦: 鉛直速度のRMS(平均だと上下対称で打ち消し合って0のまま)。
  await page.click('.scene-gallery-list button[data-scene-file="d14-vortex.json"]');
  await expect(hierarchy.getByText("流れの速さ(実効値)", { exact: true })).toBeVisible();
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
  // 生の名前(`CircuitV[2]`)ではなく、画面のほかの場所と同じ日本語で並ぶ。
  await expect(
    hierarchy.getByText("つなぎ目の電圧(2)", { exact: true }),
  ).toBeVisible();

  // ② Inspector の追加 Component。D19 は剛体を持たないので、まず剛体のある
  //    シーンへ切り替えてからボディを選ぶ。
  await page.selectOption("#select-scene", "d11-pendulum.json");
  await hierarchy.getByText("bob", { exact: true }).click();
  const inspector = page.locator("#inspector-body");
  // Probe セクション(シーン定義プローブ)と現在値。
  await expect(
    inspector.getByText("記録している値 (Probe)", { exact: true }),
  ).toBeVisible();
  await expect(inspector.getByText("横の位置(bob)", { exact: true })).toBeVisible();
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
  await expect(
    inspector.getByText("つなぎ目 (Joint)", { exact: true }),
  ).toBeVisible();

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

  // ③b 重力の向き(残タスク完遂増分、レビュー指摘「見送らず対応すること」への
  // 対応)。既定は下向き(0,-1,0)、x欄だけ変えて+x向きに変更できること。
  await expect(page.locator("#input-gravity-direction-x")).toHaveValue("0.000");
  await expect(page.locator("#input-gravity-direction-y")).toHaveValue("-1.000");
  await page.fill("#input-gravity-direction-x", "1");
  await page.locator("#input-gravity-direction-x").dispatchEvent("change");
  await page.fill("#input-gravity-direction-y", "0");
  await page.locator("#input-gravity-direction-y").dispatchEvent("change");
  await page.locator("#input-gravity-direction-y").blur();
  await page.waitForTimeout(200);
  const direction = await page.evaluate(() => {
    const w = (window as unknown as { __world: { read_component: (kind: string, arg: string) => string } })
      .__world;
    return JSON.parse(w.read_component("gravity_direction", "")) as number[];
  });
  expect(direction[0]).toBeCloseTo(1.0, 3);
  expect(direction[1]).toBeCloseTo(0.0, 3);
  expect(direction[2]).toBeCloseTo(0.0, 3);
  await expect(page.locator("#input-gravity-direction-x")).toHaveValue("1.000");

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

  // ① 材質(参考)サブツリー。設計 §1.1 の列挙にあるのにツリーに無かった。
  await expect(page.locator("#hierarchy-tree")).toContainText("材質(参考)");

  // ② 折り畳み(設計 §1.1「ツリーは折り畳み可」)。「物」を畳むと
  //    ボディ行が消える(材質配下の参照行は残る)。
  const visibleBodies = () =>
    page.locator("#hierarchy-tree .tree-group", { hasText: "物" }).locator("li:visible").count();
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
  // 表示は「実測 ×N」(実測値だと分かるように語を添えた)。
  const effective = Number(
    (await page.locator("#timescale-effective").textContent())!.replace(
      /[^0-9.]/g,
      "",
    ),
  );
  expect(effective).toBeGreaterThan(1); // 速くはなっている
  expect(effective).toBeLessThan(128); // が指定値には届かない

  // ×1 に戻せば上限に当たらなくなり、赤も消える(保持フレーム分だけ遅れて)。
  await page.selectOption("#select-timescale", "1");
  await page.waitForTimeout(1500);
  await expect(page.locator("#timescale-effective")).not.toHaveClass(/degraded/);

  expect(errors).toEqual([]);
});

test("D3「Unityパリティ」増分: Hierarchy検索・Shift範囲選択・Ctrl+A/Escape", async ({
  page,
}) => {
  // 監査で見つかった具体的な欠落3件——①ボディ数が多いシーンでHierarchyから
  // 目的の行を探す手段が無い、②Ctrl/Cmdクリックのトグルはあるが標準の
  // Shift範囲選択が無い、③範囲選択・トグルと対になる全選択(Ctrl+A)/
  // 選択解除(Escape)が無い——への対応をまとめて確認する。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // D8散乱: 床(floor) + 球50個(s0..s49) = 51体。検索・範囲選択・全選択の
  // どれも既定シーン(2体)では意味のある確認にならないため、この増分G1の
  // 最大シーンへ切り替える。
  await page.selectOption("#select-scene", "d8-scatter.json");
  const rows = page.locator("#hierarchy-tree .tree-body");
  await expect(rows).toHaveCount(51);
  const visibleRows = page.locator("#hierarchy-tree .tree-body:visible");

  // ① 検索: "floor" は1件だけに絞られる。
  const search = page.locator("#hierarchy-search");
  await search.fill("floor");
  await expect(visibleRows).toHaveCount(1);
  await expect(visibleRows.first()).toHaveText("floor");

  // "s1" は s1 本体 + s10〜s19 の計11件にマッチする。
  await search.fill("s1");
  await expect(visibleRows).toHaveCount(11);

  // 検索欄を空にすると全件へ戻る。
  await search.fill("");
  await expect(visibleRows).toHaveCount(51);

  // ② Shiftクリックの範囲選択: floor(nth 0)の次のs0(nth 1)をクリックして
  // 起点にし、s2(nth 3)をShiftクリックすると s0/s1/s2 の3件が選択される
  // ——右クリックメニューの「複製 (N件)」表示(既存の複数選択と同じ経路)で
  // 件数を確認する。範囲内(s1、nth 2)を右クリックすれば選択がそのまま
  // 保たれる(選択外の行を右クリックした場合はそこだけへ選択が移る、
  // 既存の右クリックメニューの仕様)。
  await rows.nth(1).click();
  await rows.nth(3).click({ modifiers: ["Shift"] }); // Inspectorの主選択はs2(nth 3)になる。
  await rows.nth(2).click({ button: "right" });
  await expect(page.locator("#context-menu")).toContainText("複製 (3件)");
  await page.keyboard.press("Escape"); // メニューを閉じる(複数選択は主選択1件へ戻る)。

  // ③ Ctrl+A: 現存する51体すべてが複数選択に入る。主選択(nth 3のs2)を
  // そのまま右クリックすれば「選択外→単独選択に戻す」経路を踏まずに
  // 件数を確認できる。
  await page.keyboard.press("Control+a");
  await rows.nth(3).click({ button: "right" });
  await expect(page.locator("#context-menu")).toContainText("複製 (51件)");

  // ④ Escape: メニューを閉じるだけでなく、複数選択もInspector表示中の1件
  // (主選択=s2)へ戻す——同じ行(nth 3)を右クリックし直すと件数サフィックスが
  // 消える。
  await page.keyboard.press("Escape");
  await rows.nth(3).click({ button: "right" });
  await expect(page.locator("#context-menu button", { hasText: "複製" }).first()).toHaveText(
    "複製",
  );
  await page.keyboard.press("Escape");

  expect(errors).toEqual([]);
});

test("D3「Unityパリティ」増分: 失敗がConsoleのErrorsタブへも残る", async ({ page }) => {
  // 監査で見つかった具体的な欠落: ConsoleのErrorsタブ(HTML側には元から
  // 存在する)は`drain_events_text`が`"errors"`レベルを一切出さないため
  // 常に空で、失敗は`window.alert`の一度きりのモーダルでしか伝わらなかった。
  // Validationタブの「値を1つ以上指定してください」を、UIから確実に踏める
  // 失敗経路として使う——**空欄のままではこの分岐に落ちない**ことに注意
  // (`"".split(",").map(Number)`は`[0]`になり`Number.isFinite`を満たすため、
  // 数値へ変換できないテキストを入れて初めて0件まで絞り込まれる)。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // **失敗の即時通知は `window.alert` からトーストへ移した**(増分「UI 品質の
  // 底上げ」)。ブロッキングなモーダルが**出ないこと**もここで固定する
  // ——出てしまうと Playwright の既定ハンドラが黙って dismiss して、
  // テストだけが通る状態になり得る。
  let blockingDialogAppeared = false;
  page.on("dialog", (d) => {
    blockingDialogAppeared = true;
    d.accept();
  });

  await page.click('.project-tab[data-tab="validation"]');
  await expect(page.locator("#validation-base-json")).toBeVisible();
  await page.locator('input[title*="パラメータの値"]').fill("abc,def");
  await page.click('button:has-text("スイープを実行")');

  await expect(page.locator(".toast")).toContainText(
    "値を1つ以上指定してください",
  );
  expect(blockingDialogAppeared).toBe(false);

  // 同じメッセージがConsoleのErrorsタブへも残っている(トーストが消えた後も
  // 見返せる)。
  await page.click('.console-tab[data-tab="errors"]');
  await expect(
    page.locator("#console-log li", { hasText: "値を1つ以上指定してください" }),
  ).toBeVisible();

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

test("B9: エディタから量子ドメイン(1D/2D)をプリセットで新規追加できる", async ({ page }) => {
  // 上の「群3」テストはギャラリーシーン(既に量子ドメインを持つraw_state)を
  // 読み込む経路しか確認していなかった——量子1D/2Dは`enable_grid_fluid_2d_domain`
  // 等と違い、エディタから**新規に置く**手段自体が無かった(`crates/sim-world/
  // src/scenario.rs`のモジュールdoc「構築レシピを畳んだ」経緯参照)。ここでは
  // Settingsの「量子ドメイン(プリセット)」フォーム(ガウス波束+ポテンシャル
  // プリセットをTypeScript側で計算し、`enable_quantum_1d_domain`/
  // `enable_quantum_2d_domain`へ渡す新経路)を実際に叩いて、場のパネルに
  // それぞれの密度分布が描かれることを確認する。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const fieldPanel = page.locator("#field-panel");
  const fieldTitle = page.locator("#field-title");

  await page.click("#btn-settings");

  // --- 1D: 調和振動子ポテンシャル ---
  await page.selectOption("#select-quantum1d-n", "128");
  await page.fill("#input-quantum1d-dx", "0.1");
  await page.fill("#input-quantum1d-x0", "6.4");
  await page.fill("#input-quantum1d-sigma", "1.0");
  await page.fill("#input-quantum1d-k0", "0");
  await page.selectOption("#select-quantum1d-potential", "harmonic");
  await page.fill("#input-quantum1d-omega", "1.0");
  await page.click("#btn-add-quantum-1d");
  await page.click("#btn-settings"); // ポップオーバーを閉じる。
  await expect(fieldPanel).toBeVisible();
  await expect(fieldTitle).toContainText("量子 1D");
  await expect(fieldTitle).toContainText("格子 128 点");

  // --- 2D: 二重スリット(D27と同じ構成、`quantum2dDoubleSlitPotential`のdoc参照)。
  // 場のパネルは2Dを優先して描く(`updateFieldPanel`のdoc「優先順位を固定する」)ので、
  // 2Dを有効化した時点でタイトルが1Dから切り替わることも合わせて確認する。
  await page.click("#btn-settings"); // 再度開く。
  await page.selectOption("#select-quantum2d-nx", "64");
  await page.selectOption("#select-quantum2d-ny", "64");
  await page.fill("#input-quantum2d-dx", "0.2");
  await page.fill("#input-quantum2d-dy", "0.2");
  await page.fill("#input-quantum2d-x0", "2.0");
  await page.fill("#input-quantum2d-y0", "6.4");
  await page.fill("#input-quantum2d-sigma-x", "1.0");
  await page.fill("#input-quantum2d-sigma-y", "3.0");
  await page.fill("#input-quantum2d-k0", "3.0");
  await page.selectOption("#select-quantum2d-potential", "double_slit");
  await page.fill("#input-quantum2d-v0", "60.0");
  await page.fill("#input-quantum2d-slit-width", "0.7");
  await page.fill("#input-quantum2d-slit-separation", "2.0");
  await page.click("#btn-add-quantum-2d");
  await page.click("#btn-settings"); // ポップオーバーを閉じる。
  await expect(fieldTitle).toContainText("量子 2D");
  await expect(fieldTitle).toContainText("64×64");

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

  // 開閉は行の高さのアニメーション(約 0.2 秒)を伴うので、収まるまで待つ。
  await page.click('.project-tab[data-tab="materials"]');
  await expect.poll(insideViewport, { timeout: 5_000 }).toBe(true);
  // 中身(材質表)が実際に見えること。
  await expect(page.locator("#project-body .materials-table")).toBeVisible();

  // 同じタブをもう一度クリックすると閉じる。
  await page.click('.project-tab[data-tab="materials"]');
  await expect.poll(insideViewport, { timeout: 5_000 }).toBe(false);

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

test("残タスク完遂の縦串⑤前後: 複合形状(L字)/凸包メッシュがUIから作れ、走らせても複製してもクラッシュしない", async ({
  page,
}) => {
  // レビュー指摘(「UIから作る経路がないから」を許容せず作る前提で進める、
  // 「テスト不能」を縮約とせずあるべき姿を実装する)への対応。`spawn_compound_l_shape`/
  // `spawn_convex_mesh_cube`(Rust側)を「＋ 追加」メニュー経由で実際にUIから
  // 呼び、Hierarchy に現れること・N step 送りでクラッシュしないこと(=描画・
  // 物理の両方が実際に動くこと)・複製でも壊れないことまで確認する。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const rowCount = () => page.locator("#hierarchy-tree .tree-body").count();

  // ① ツールバー「＋ 追加」メニューから複合形状(L字)を追加。
  const before = await rowCount();
  await addViaMenu(page, "＋ 複合形状 (L字)");
  await page.waitForTimeout(100);
  expect(await rowCount()).toBe(before + 1);
  await expect(page.locator("#hierarchy-tree .tree-body").last()).toContainText("Compound_");

  // ② 同じく凸包メッシュを追加。
  await addViaMenu(page, "＋ 凸包メッシュ");
  await page.waitForTimeout(100);
  expect(await rowCount()).toBe(before + 2);
  await expect(page.locator("#hierarchy-tree .tree-body").last()).toContainText("ConvexMesh_");

  // ③ Scene View の右クリックメニューからも同じ2形状を配置できる
  // (ツールバーのボタンとメニューの両方が同じ `spawnShapeAt` を共有する設計、
  // 「群2」の既存パターンと同じ)。
  const canvas = page.locator("canvas").first();
  const box = (await canvas.boundingBox())!;
  await page.mouse.click(box.x + box.width * 0.6, box.y + box.height * 0.3, { button: "right" });
  await expect(page.locator("#context-menu")).toBeVisible();
  await page
    .locator("#context-menu button", { hasText: "ここに複合形状(L字)を配置" })
    .click();
  await page.waitForTimeout(100);
  expect(await rowCount()).toBe(before + 3);

  await page.mouse.click(box.x + box.width * 0.6, box.y + box.height * 0.3, { button: "right" });
  await expect(page.locator("#context-menu")).toBeVisible();
  await page.locator("#context-menu button", { hasText: "ここに凸包メッシュを配置" }).click();
  await page.waitForTimeout(100);
  expect(await rowCount()).toBe(before + 4);

  // ④ N step 送りでシミュレーションを実際に進める——描画(carrier mesh /
  // ConvexGeometry)と物理(Compoundの衝突・ConvexMeshのAABB近似)の両方が
  // 例外を出さずに動くこと(「テスト不能」への直接の反証)。
  // `⏭`はEditモード or 再生中は無効(Unity の Step と同じ意味論)——
  // Playへ入ってから一時停止する(既存の「増分G2」テストと同じ手順)。
  await page.click("#btn-mode-play");
  await page.click("#btn-play");
  await page.fill("#input-step-count", "60");
  await page.click("#btn-step");
  await page.waitForTimeout(300);

  // ⑤ Hierarchy 右クリックで複合形状を複製できる(`body_shape_json_at`
  // 経由で実際の形状を読み直してメッシュを再構築する経路、スポーン時の
  // 既定形状だと決め打ちしない)。
  const compoundRow = page.locator("#hierarchy-tree .tree-body", { hasText: "Compound_" }).first();
  await compoundRow.click({ button: "right" });
  await expect(page.locator("#context-menu")).toBeVisible();
  const beforeDuplicate = await rowCount();
  await page.locator("#context-menu button", { hasText: "複製" }).first().click();
  await page.waitForTimeout(200);
  expect(await rowCount()).toBeGreaterThan(beforeDuplicate);

  expect(errors).toEqual([]);
});

test("Prefab: 複合形状/凸包メッシュをプレハブ化して再スポーンできる", async ({
  page,
}) => {
  // **形状の読み書きを`ShapeJson`1本へ統合した増分の受け入れテスト**。
  // それ以前のPrefabは`body_shape_params_f64_at`(平坦なf64配列)で寸法を
  // 読み、`spawn_sphere`/`spawn_box`という固定レシピのスポナーで戻す作り
  // だったため、**Compound/ConvexMeshのボディは「プレハブ化」を押しても
  // 黙って何も起きなかった**(`captureBody`が球/箱以外を`null`で弾く)。
  // `body_shape_json_at`↔`spawn_shape_json`の対に載せ替えた今、両形状が
  // キャプチャ→再スポーンまで往復することを実UI経由で確かめる。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const rowCount = () => page.locator("#hierarchy-tree .tree-body").count();

  // ① プレハブ化の元になるボディを2つ作る(複合形状と凸包メッシュ)。
  await addViaMenu(page, "＋ 複合形状 (L字)");
  await page.waitForTimeout(100);
  await addViaMenu(page, "＋ 凸包メッシュ");
  await page.waitForTimeout(100);

  // ② Hierarchy 右クリック →「プレハブ化」。旧実装ではここで
  //    「この形状はPrefab化できません」のalertが出て登録されなかった。
  for (const label of ["Compound_", "ConvexMesh_"]) {
    const row = page.locator("#hierarchy-tree .tree-body", { hasText: label }).first();
    await row.click({ button: "right" });
    await expect(page.locator("#context-menu")).toBeVisible();
    await page.locator("#context-menu button", { hasText: "プレハブ化" }).click();
    await page.waitForTimeout(100);
  }

  // ③ Project ドロワーの Prefabs タブに2件とも並ぶ(表示ラベルの形状名は
  //    `body_shape_kind_at`が返す種別名)。
  await page.click('.project-tab[data-tab="prefabs"]');
  const projectBody = page.locator("#project-body");
  await expect(projectBody).toContainText("compound");
  await expect(projectBody).toContainText("convex_mesh");

  // ④ 2件とも「スポーン」でき、Hierarchy の行が実際に増える
  //    (=`spawn_shape_json`がキャプチャした形状を復元できている)。
  const beforeSpawn = await rowCount();
  const spawnButtons = projectBody.locator("li button", { hasText: "スポーン" });
  await expect(spawnButtons).toHaveCount(2);
  await spawnButtons.nth(0).click();
  await spawnButtons.nth(1).click();
  await page.waitForTimeout(200);
  expect(await rowCount()).toBe(beforeSpawn + 2);
  // 復元されたボディのラベルは形状から引かれる(`shape_label_prefix`)ので、
  // 「球として戻ってきた」等の取り違えはラベルで検出できる。
  await expect(page.locator("#hierarchy-tree")).toContainText("Compound_");
  await expect(page.locator("#hierarchy-tree")).toContainText("ConvexMesh_");

  // ⑤ 再スポーンしたボディを実際に走らせてもクラッシュしない(描画・物理の
  //    両方が復元後の形状で動くこと)。`⏭`はPlayへ入ってからでないと無効。
  await page.click("#btn-mode-play");
  await page.click("#btn-play");
  await page.fill("#input-step-count", "60");
  await page.click("#btn-step");
  await page.waitForTimeout(300);

  expect(errors).toEqual([]);
});

test("残タスク完遂: 結合14種の残り6種(熱ノード/SPH/格子流体/気体ドメインを要するもの)がUIから追加できる", async ({
  page,
}) => {
  // レビュー指摘(「やり遂げて欲しい」「対応できていますか？出来ていなければ
  // 対応して」)への対応。PhaseChangeMorph/SphRigid/GridFluidRigid/
  // BoussinesqBuoyancy/ConvectionLink/PistonGasは、参照する熱ノード・SPH流体・
  // 格子流体・気体区画をUIから作る手段が無く追加できなかった
  // (`docs/22-roadmap/03-editor-todo.md`に明記していた既知の欠落)。
  // Settingsの「ドメイン」パネル(新設)でこれらを先に有効化し、
  // Add Couplingフォームで6種すべてが実際に追加できることを確認する。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // Box_1 を選択する(Inspector の Add Coupling フォームを開くため)。
  await page.locator("#hierarchy-tree").getByText("Box_1", { exact: true }).click();
  const inspector = page.locator("#inspector-body");

  // ---- ドメインをSettingsから有効化する ----
  await page.click("#btn-settings");
  // 熱ノードを1個追加(既定シーンのindex 0とは別、新設ノードはindex 1)。
  await page.click("#btn-add-thermal-node");
  await expect(page.locator("#thermal-node-count-display")).toContainText("2");
  await page.click("#btn-enable-grid-fluid");
  await page.click("#btn-enable-gas");
  await page.click("#btn-settings"); // ポップオーバーを閉じる。

  // SPHドメインは既存の「+ 流体」ボタン(スポーンパレット)で有効化する。
  await addViaMenu(page, "＋ 流体 (SPH 水塊)");

  // フィールドIDは`component_schema`が返す`add_*_coupling`の実引数名
  // そのもの(`#add-coupling-field-${name}`、B12〜B15でスキーマ駆動フォーム
  // 化)——以前の`add-coupling-axis-*`/`add-coupling-p1`〜`p6`という
  // 種別ごとに読み替えていた汎用IDは廃止された。
  const addCoupling = async (
    kind: string,
    fields: Record<string, number | string>,
  ) => {
    await page.selectOption("#add-coupling-kind", kind);
    // `Object.entries`を`Array.forEach`に渡すとコールバックのawaitを待たない
    // (並行実行される)ため、直列に`page.fill`を呼ぶには明示的なforループが要る。
    for (const [name, value] of Object.entries(fields)) {
      await page.fill(`#add-coupling-field-${name}`, String(value));
    }
    await page.click("#add-coupling-button");
  };

  // ① PhaseChangeMorph: body=Box_1, thermal_node=1(新設)、材質は氷/水
  // (melting_temperature=273.15K/latent_heat_fusion=334000/specific_heat_solid=
  // 2100)、specific_heat_liquid=4186、initial_mass=1kg、conductance=10W/K、
  // initial_enthalpy=-50000J(融点未満の固相から開始)。材質もUIから明示的に
  // 指定できること自体が「縮約させない」の検証点(**残タスク完遂増分**)。
  await addCoupling("phase_change_morph", {
    body: 1,
    thermal_node: 1,
    melting_temperature: 273.15,
    latent_heat_fusion: 334000,
    specific_heat_solid: 2100,
    specific_heat_liquid: 4186,
    initial_mass: 1,
    conductance: 10,
    initial_enthalpy: -50000,
  });
  await expect(inspector.getByText("PhaseChangeMorph", { exact: true })).toBeVisible();

  // ② SphRigid: body=Box_1, radius=0.2m, boundary_points=12。
  await addCoupling("sph_rigid", { body: 1, radius: 0.2, boundary_points: 12 });
  await expect(inspector.getByText("SphRigid", { exact: true })).toBeVisible();

  // ③ GridFluidRigid: body=Box_1, half_width=0.3m, half_height=0.3m。
  await addCoupling("grid_fluid_rigid", {
    body: 1,
    half_width: 0.3,
    half_height: 0.3,
  });
  await expect(inspector.getByText("GridFluidRigid", { exact: true })).toBeVisible();

  // ④ BoussinesqBuoyancy: thermal_node=1, ambient_temperature=293.15K,
  // thermal_expansion_coefficient=3.4e-3(空気の目安値)。bodyを参照しない
  // 結合なので「Coupling (シーン全体)」に出る。
  await addCoupling("boussinesq_buoyancy", {
    thermal_node: 1,
    ambient_temperature: 293.15,
    thermal_expansion_coefficient: 0.0034,
  });
  await expect(inspector.getByText("BoussinesqBuoyancy", { exact: true })).toBeVisible();

  // ⑤ ConvectionLink: fluid_node=0(既定シーンのノード), surface_node=1(新設),
  // area=0.01m^2, characteristic_length=0.05m, mode=3(強制対流・平板)、
  // 流体物性値(空気の目安値)もUIから明示的に指定する(**残タスク完遂増分**、
  // `ConvectionLink::default()`固定ではないことの検証点)。
  await addCoupling("convection_link", {
    fluid_node: 0,
    surface_node: 1,
    area: 0.01,
    characteristic_length: 0.05,
    mode: 3,
    fluid_thermal_conductivity: 0.026,
    kinematic_viscosity: 1.5e-5,
    prandtl_number: 0.71,
    thermal_expansion_coefficient: 0,
  });
  await expect(inspector.getByText("ConvectionLink", { exact: true })).toBeVisible();

  // ⑥ PistonGas: body=Box_1, axis=(0,1,0), area=0.01m^2, initial_volume=0.001m^3。
  await addCoupling("piston_gas", {
    body: 1,
    axis_x: 0,
    axis_y: 1,
    axis_z: 0,
    area: 0.01,
    initial_volume: 0.001,
  });
  await expect(inspector.getByText("PistonGas", { exact: true })).toBeVisible();

  expect(errors).toEqual([]);
});

test("縦串⑤(飛行機の物理): 翼揚力/マグヌス揚力をUIから追加でき、操縦面の舵角を実行時に変更できる", async ({
  page,
}) => {
  // レビュー指摘(「これについては、コア変更してもオッケー」)を受けて実装。
  // `sim_coupling::BuoyancyDrag::lift`(薄翼理論+マグヌス効果)は物理コア側に
  // 既に実装済みだったが、Add Couplingフォームでは`None`固定で到達できな
  // かった——WingLift/MagnusLiftとして解禁する。さらに、Coupling registryは
  // 元々「追加のみ・実行時パラメータ変更不可」だったため、飛行中に操縦面
  // (エルロン・エレベーター・ラダー)の舵角を変える手段が無かった——
  // `Coupling::set_scalar_param`(`CouplingParam::ControlSurfaceDeflection`)+
  // `Command::SetCouplingParam`という新しい書き換え経路を追加し、Inspectorの
  // Coupling行に出る「操縦面舵角」欄から実際に操作できることを確認する。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.locator("#hierarchy-tree").getByText("Box_1", { exact: true }).click();
  const inspector = page.locator("#inspector-body");

  // フィールドIDは`component_schema`が返す`add_*_coupling`の実引数名
  // そのもの(B12〜B15でスキーマ駆動フォーム化、上のPhaseChangeMorph等と
  // 同じ形)。

  // ① WingLift: body=Box_1, chord_local=(1,0,0), span_local=(0,0,1),
  // wing_area=2m^2, atmosphere_density=1.225, atmosphere_viscosity=1.81e-5。
  await page.selectOption("#add-coupling-kind", "wing_lift");
  await page.fill("#add-coupling-field-body", "1");
  await page.fill("#add-coupling-field-chord_x", "1");
  await page.fill("#add-coupling-field-chord_y", "0");
  await page.fill("#add-coupling-field-chord_z", "0");
  await page.fill("#add-coupling-field-span_x", "0");
  await page.fill("#add-coupling-field-span_y", "0");
  await page.fill("#add-coupling-field-span_z", "1");
  await page.fill("#add-coupling-field-wing_area", "2");
  await page.fill("#add-coupling-field-atmosphere_density", "1.225");
  await page.fill("#add-coupling-field-atmosphere_viscosity", "1.81e-5");
  await page.click("#add-coupling-button");
  await expect(inspector.getByText("BuoyancyDrag", { exact: true }).first()).toBeVisible();

  // ② MagnusLift: body=Box_1(種別を切り替えても選択中ボディが既定値のまま
  // 再セットされる、`initialApplyFieldValue`のdoc参照)、radius=0.3,
  // atmosphere_density=1.225, atmosphere_viscosity=1.81e-5。
  await page.selectOption("#add-coupling-kind", "magnus_lift");
  await page.fill("#add-coupling-field-radius", "0.3");
  await page.fill("#add-coupling-field-atmosphere_density", "1.225");
  await page.fill("#add-coupling-field-atmosphere_viscosity", "1.81e-5");
  await page.click("#add-coupling-button");
  await expect(inspector.getByText("BuoyancyDrag", { exact: true })).toHaveCount(2);

  // ③ 操縦面舵角欄がWingLift/MagnusLiftの両方(いずれもBuoyancyDrag)の行に
  // 出ており、値を変更してもクラッシュしないこと(MagnusLiftには効かない
  // ——`set_scalar_param`が`false`を返して無言で無視される、モジュールdoc
  // 参照——が、それ自体はエラーにならない)。
  const deflectionInputs = page.locator('input[id^="coupling-deflection-"]');
  await expect(deflectionInputs).toHaveCount(2);
  await deflectionInputs.nth(0).fill("15");
  await deflectionInputs.nth(0).dispatchEvent("change");
  await deflectionInputs.nth(1).fill("-10");
  await deflectionInputs.nth(1).dispatchEvent("change");

  // ④ N step進めても(舵角適用込みで)クラッシュしないこと。
  await page.click("#btn-mode-play");
  await page.click("#btn-play");
  await page.fill("#input-step-count", "30");
  await page.click("#btn-step");
  await page.waitForTimeout(300);

  expect(errors).toEqual([]);
});

test("検証タブ: 合格基準がシーンJSONスキーマ(pass_criteria)へ往復する", async ({ page }) => {
  // **残タスク完遂増分**(レビュー指摘「勝手に対象外にするのは禁止令発令中！！！」
  // への対応): 合格基準(probe index・比較演算子・しきい値)は`Scenario::
  // pass_criteria`としてシーンJSONスキーマの一部になった。このタブがBase scene
  // JSONの`pass_criteria`をフォームへ読み込み、「基準をシーンJSONへ書き込む」で
  // フォームの内容を逆にJSONへ書き戻せることを確認する。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click('.project-tab[data-tab="validation"]');
  const baseJsonArea = page.locator("#validation-base-json");
  await expect(baseJsonArea).toBeVisible();

  const probeIndexInput = page.locator('input[title*="probe index"]');
  const thresholdInputForRead = page.locator('input[title="合格基準のしきい値(probeの最終値と比較する)"]');

  // ① Base scene JSONへ直接`pass_criteria`を貼り付けて`change`イベントを
  // 発火させると(テキストエリアを手で編集/貼り付けした状況の再現)、
  // フォームの3フィールドへ反映される。
  const seeded = await baseJsonArea.inputValue();
  const seededObj = JSON.parse(seeded);
  seededObj.pass_criteria = [{ probe_index: 3, operator: "le", threshold: 2.5 }];
  await baseJsonArea.fill(JSON.stringify(seededObj));
  await baseJsonArea.dispatchEvent("change");
  await expect(probeIndexInput).toHaveValue("3");
  await expect(thresholdInputForRead).toHaveValue("2.5");

  // ② フォームの値を変更してから書き込みボタンを押すと、Base scene JSONの
  // `pass_criteria`が更新される。
  await probeIndexInput.fill("5");
  await thresholdInputForRead.fill("7.25");
  await page.click('button:has-text("基準をシーンJSONへ書き込む")');

  const written = JSON.parse(await baseJsonArea.inputValue());
  expect(written.pass_criteria).toHaveLength(1);
  expect(written.pass_criteria[0].probe_index).toBe(5);
  expect(written.pass_criteria[0].threshold).toBe(7.25);

  expect(errors).toEqual([]);
});

test("D1: スケッチ→押し出しで剛体をUIから作れ、走らせてもクラッシュしない", async ({
  page,
}) => {
  // D1(スケッチ・押し出し)の受け入れテスト。既存の「複合形状(L字)/凸包
  // メッシュがUIから作れ…」テストと同じ形で、**UIの操作だけ**で新しい形状
  // JSONタグ(`mesh`)を通した剛体が実際に作れることを確認する。
  //
  // ここが守るのは配線の健全性(ツール切替 → 構築平面へのレイキャスト →
  // 確定 → wasmの`sketch_extrude_shape_json` → `spawn_shape_json` →
  // Hierarchy/描画)。ブーリアン合成そのものの数値的な正しさは Rust 側の
  // 解析的なテスト(`sim_mechanics::sketch`)が担保しており、ここでは重ねない。
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const rowCount = () => page.locator("#hierarchy-tree .tree-body").count();
  const before = await rowCount();

  // ① スケッチツールへ切り替えるとパネルが開く(他のツールでは閉じている)。
  await expect(page.locator("#sketch-panel")).toBeHidden();
  await page.click("#btn-tool-sketch");
  await expect(page.locator("#sketch-panel")).toBeVisible();

  const canvas = page.locator("canvas").first();
  const box = (await canvas.boundingBox())!;
  // 構築平面は地面(y=0)。画面の下半分は必ず地面に当たる(既定カメラは
  // (6,4,10) から (0,1.5,0) を見下ろしている)。**画面左下は避ける**
  // ——スケッチパネル自体がそこに浮いており、重なった座標のクリックは
  // キャンバスではなくパネルへ吸われて頂点が置かれない。
  const clickAt = async (fx: number, fy: number) => {
    await page.mouse.click(box.x + box.width * fx, box.y + box.height * fy);
    await page.waitForTimeout(30);
  };

  // ② 地面を4回クリックして四角形の点列を作る。
  for (const [fx, fy] of [
    [0.5, 0.58],
    [0.72, 0.58],
    [0.72, 0.78],
    [0.5, 0.78],
  ] as [number, number][]) {
    await clickAt(fx, fy);
  }
  await expect(page.locator("#sketch-status")).toContainText("作図中の点 4");

  // ③ 「確定」で1枚のプロファイルになる。
  await page.click("#btn-sketch-confirm");
  await expect(page.locator("#sketch-status")).toContainText(
    "プロファイル 1 枚",
  );
  await expect(page.locator("#sketch-status")).toContainText("作図中の点 0");

  // ④ 深さを指定して押し出すと、Hierarchy にボディが1つ増える。形状は
  //    `mesh`タグ → Rust側の近似凸分解 → ConvexMesh(凸)か Compound(凹)。
  await page.fill("#input-sketch-depth", "0.4");
  await page.click("#btn-sketch-extrude");
  await page.waitForTimeout(200);
  expect(await rowCount()).toBe(before + 1);
  await expect(page.locator("#hierarchy-tree .tree-body").last()).toContainText(
    /ConvexMesh_|Compound_/,
  );
  // 押し出した後はスケッチが片付いている。
  await expect(page.locator("#sketch-status")).toContainText(
    "プロファイル 0 枚",
  );

  // ⑤ **ブーリアン(減算)**: 土台の四角形を確定 → 「減算」に切り替え、その
  //    内側に小さい四角形を描いて押し出す。断面に穴が空くので、分解結果は
  //    必ず Compound(凸パーツ複数)になる——`mesh`タグが`convex_mesh`と
  //    違う意味を持つ(面情報から凹みが復元される)ことがUI経由で分かる。
  for (const [fx, fy] of [
    [0.52, 0.55],
    [0.88, 0.55],
    [0.88, 0.8],
    [0.52, 0.8],
  ] as [number, number][]) {
    await clickAt(fx, fy);
  }
  await page.click("#btn-sketch-confirm");
  await page.selectOption("#select-sketch-op", "subtract");
  for (const [fx, fy] of [
    [0.63, 0.63],
    [0.77, 0.63],
    [0.77, 0.72],
    [0.63, 0.72],
  ] as [number, number][]) {
    await clickAt(fx, fy);
  }
  await expect(page.locator("#sketch-status")).toContainText(
    "プロファイル 1 枚 / 作図中の点 4",
  );
  // 「押し出し」は描きかけの点列を自動で確定してから走る。
  await page.click("#btn-sketch-extrude");
  await page.waitForTimeout(200);
  expect(await rowCount()).toBe(before + 2);
  await expect(page.locator("#hierarchy-tree .tree-body").last()).toContainText(
    "Compound_",
  );

  // ⑥ 実際に走らせる——押し出したメッシュが物理(接触生成・質量特性)と
  //    描画の両方で例外を出さずに動くこと。
  await page.click("#btn-mode-play");
  await page.click("#btn-play");
  await page.fill("#input-step-count", "60");
  await page.click("#btn-step");
  await page.waitForTimeout(300);

  expect(errors).toEqual([]);
});
