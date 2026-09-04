import { expect, test, type Page } from "@playwright/test";
import { collectPageErrors } from "./helpers";
import { GUIDED_CATEGORIES as CATEGORIES } from "../src/catalog";

// ワークスペース(`src/workspace.ts`)の E2E。
//
// **ここが守るもの**: 画面はひとつのまま、粒度が「大局 ⇄ 局所」に連続して
// 動くこと。具体的には
//   - 初めて開いた人が、空白ではなく**動いている現象**から始められる
//   - どこからでも 1 手で開く窓(⌘K)から、打つ/選ぶで目的の現象へ届く
//   - 見る深さのダイヤルを右へ回すほど、一覧 → グラフ → 道具が順に現れる
//   - 全体の深さを変えずに、カード 1 枚だけ深く開ける(局所の粒度)
//   - ひとつの対象を選ぶと文脈がそこへ寄り、「全体へ戻る」で戻れる
//
// 既定の storageState(`playwright.config.ts`)は深さ 3 なので、ここでは
// **初めて開いた人**を再現するために空の storageState を使う。
test.use({ storageState: { cookies: [], origins: [] } });

async function boot(page: Page) {
  await page.goto("/");
  await expect(page.locator("#boot-overlay")).toBeHidden({ timeout: 30_000 });
}

/** 「経過した時間」を秒で読む(表示は桁に合わせて単位が変わる)。 */
async function elapsedSeconds(page: Page): Promise<number> {
  const raw = await page.locator("#readout-time").getAttribute("data-seconds");
  return Number.parseFloat(raw ?? "0") || 0;
}

async function setGrain(page: Page, at: 0 | 1 | 2 | 3) {
  await page.click(`.detail-stop[data-at="${at}"]`);
}

test("初めて開くと、空白ではなく動いている現象から始まる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 画面はひとつ。上にパンくずと走行、右に文脈、まん中が舞台。
  await expect(page.locator("#commandbar")).toBeVisible();
  await expect(page.locator("#context")).toBeVisible();
  await expect(page.locator("#scene-view")).toBeVisible();
  // 何を見ているかがパンくずに出ている。
  await expect(page.locator("#crumb-experiment")).toBeVisible();
  // すでに走っている(押させない)。
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "true");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);
  // 何も選んでいないので「選んだもの」は出ない(内部都合の床が選ばれない)。
  await expect(page.locator('.card[data-card="focus"]')).toHaveCount(0);
  expect(errors).toEqual([]);
});

test("⌘K → 打つ → Enter の3手で、目的の現象が走り出す", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  await page.keyboard.press("Control+k"); // ①どこからでも開く
  await expect(page.locator("#palette")).toBeVisible();
  await page.fill("#palette-input", "コーヒー"); // ②絞る
  await expect(page.locator(".palette-row").first()).toContainText("コーヒー");
  await page.keyboard.press("Enter"); // ③選ぶ = 走り出す

  await expect(page.locator("#palette")).toBeHidden();
  await expect(page.locator("#crumb-experiment")).toContainText("コーヒー");
  await expect(page.locator("#context")).toContainText("コーヒーの温度");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);
  expect(errors).toEqual([]);
});

test("見る深さを右へ回すほど、道具が順に現れる(連続した粒度)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  await setGrain(page, 0);
  await expect(page.locator("#hierarchy")).toBeHidden();
  await expect(page.locator("#probe-graphs")).toBeHidden();
  await expect(page.locator("#toolbar")).toBeHidden();
  await expect(page.locator("#project-drawer")).toBeHidden();

  await setGrain(page, 1);
  await expect(page.locator("#timeline")).toBeVisible();
  await expect(page.locator("#hierarchy")).toBeHidden();

  await setGrain(page, 2);
  await expect(page.locator("#probe-graphs")).toBeVisible();
  await expect(page.locator("#hierarchy")).toBeVisible();
  await expect(page.locator("#inspector")).toBeVisible();
  await expect(page.locator("#toolbar")).toBeHidden();

  await setGrain(page, 3);
  await expect(page.locator("#toolbar")).toBeVisible();
  await expect(page.locator("#console-panel")).toBeVisible();
  await expect(page.locator("#project-drawer")).toBeVisible();

  // 深さは覚えている(毎回入り直させない)。
  await page.reload();
  await expect(page.locator("#boot-overlay")).toBeHidden({ timeout: 30_000 });
  await expect(page.locator("#toolbar")).toBeVisible();
  expect(errors).toEqual([]);
});

test("全体は浅いまま、カード1枚だけ深く開ける(局所の粒度)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);

  const knobs = page.locator('.card[data-card="knobs"]');
  await expect(knobs).toHaveAttribute("data-expanded", "false");
  await knobs.locator(".card-header").click();
  await expect(knobs).toHaveAttribute("data-expanded", "true");
  await expect(page.locator("#knob-height")).toBeVisible();

  // 局所を開いても、大局は「みる」のまま——一覧やグラフは出てこない。
  await expect(page.locator("#hierarchy")).toBeHidden();
  await expect(page.locator("#probe-graphs")).toBeHidden();
  expect(errors).toEqual([]);
});

test("ひとつの対象を選ぶと文脈がそこへ寄り、全体へ戻れる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);

  // 一覧から実体を選ぶ(3D のクリックでも同じ状態になる)。
  await page.locator("#hierarchy-tree .tree-body").nth(1).click();
  await expect(page.locator("#crumb-body")).toBeVisible();
  const focus = page.locator('.card[data-card="focus"]');
  await expect(focus).toBeVisible();
  await expect(focus).toContainText("かたち");

  await page.click("#btn-clear-selection");
  await expect(page.locator("#crumb-body")).toHaveCount(0);
  await expect(page.locator('.card[data-card="focus"]')).toHaveCount(0);
  expect(errors).toEqual([]);
});

test("つまみを動かすと、その設定でやり直す", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);

  const slider = page.locator("#knob-height");
  await slider.fill("5");
  await slider.dispatchEvent("change");

  const height = page.locator('#context dd[data-probe="0"]');
  await expect
    .poll(async () => Number.parseFloat((await height.textContent()) ?? "99"), {
      timeout: 10_000,
    })
    .toBeLessThan(6);
  expect(errors).toEqual([]);
});

test("dt の桁が極端なシーンでも、待たずに現象が進む", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // D34(太陽系儀)は 1 step が 31555 秒。時間倍率では上限でも 1 step 4 分。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "惑星");
  await page.keyboard.press("Enter");

  await expect
    .poll(() => elapsedSeconds(page), { timeout: 20_000 })
    .toBeGreaterThan(1_000_000);
  await expect(page.locator("#readout-time")).toContainText("日");
  expect(errors).toEqual([]);
});

test("3Dに何も描かれない実験を選ぶと、グラフが見える深さまで自動で開く", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);

  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");

  // 選んだのに何も映らない、を残さない。
  await expect(page.locator("#probe-graphs")).toBeVisible();
  await expect(page.locator("#context")).toContainText("下のグラフ");
  expect(errors).toEqual([]);
});

test("形の無い現象では、空の3Dを見せずに見る場所へ送る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "熱が棒");
  await page.keyboard.press("Enter");

  // 力学ボディが1つも無いシーン。「読み込みに失敗した」と読ませない。
  await expect(page.locator("#stage-empty-note")).toBeVisible();
  await expect(page.locator("#scene-view")).toHaveAttribute(
    "data-stage-empty",
    "true",
  );
  // 場のパネルは隅の小窓ではなく、空いた舞台の幅をもらう。
  const panel = await page.locator("#field-panel").boundingBox();
  const stage = await page.locator("#scene-view").boundingBox();
  expect(panel!.width).toBeGreaterThan(stage!.width * 0.7);
  // 題に「いま何度か」が出ている(色の帯だけで数値を当てさせない)。
  await expect(page.locator("#field-title")).toContainText("℃");

  // 物のあるシーンへ移ると、案内は引っ込む。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "ボールを落とす");
  await page.keyboard.press("Enter");
  await expect(page.locator("#stage-empty-note")).toBeHidden();
  expect(errors).toEqual([]);
});

// 「つくる」の粒度——**自分で組み立てる人**が行き止まりに当たらないこと。
// 利用者役④(粒度「つくる」)が実際に詰まった順に並べてある。
test("自分で置いた物を、そのまま「うごかす」で落とせる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await expect(page.locator("#crumb-own-scene")).toContainText("じぶんの場面");

  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.click("#btn-run");

  // カタログの実験を選んでいなくても、パレットではなく**場面が走る**。
  await expect(page.locator("#palette")).toBeHidden();
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "true");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);
  expect(errors).toEqual([]);
});

test("材質を選び直すと、重さもその材質のものになる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.locator("#hierarchy-tree .tree-body").last().click();

  const mass = () =>
    page.locator("#inspector-mass").inputValue().then((v) => Number.parseFloat(v));
  const steel = await mass();
  expect(steel).toBeGreaterThan(0);

  await page.selectOption("#inspector-material", "ゴム(天然)");
  await expect
    .poll(async () => page.locator("#inspector-material").inputValue(), { timeout: 10_000 })
    .toBe("ゴム(天然)");
  // 密度が違えば重さも違う——選び直した材質で計算し直されている。
  await expect.poll(mass, { timeout: 10_000 }).toBeLessThan(steel);

  // 名前は連番へ作り変わらない(書き出し→読み直しで消えていた)。
  await expect(page.locator("#hierarchy-tree")).toContainText("Sphere_1");
  expect(errors).toEqual([]);
});

test("打ち込んだ重さが、とめている間でもその場で効く", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.locator("#hierarchy-tree .tree-body").last().click();

  await page.fill("#inspector-mass", "10");
  await page.locator("#inspector-mass").dispatchEvent("change");
  await expect
    .poll(
      async () => Number.parseFloat(await page.locator("#inspector-mass").inputValue()),
      { timeout: 10_000 },
    )
    .toBeCloseTo(10, 3);
  expect(errors).toEqual([]);
});

test("名前を付けて保存した場面は、開き直しても残っている", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  const bodies = await page.locator("#hierarchy-tree .tree-body").count();

  await page.fill("#input-scene-name", "テストの場面");
  await page.click("#btn-save-scene");
  await expect(page.locator(".saved-scene-open")).toContainText("テストの場面");
  await expect(page.locator("#crumb-own-scene")).toContainText("テストの場面");

  // 更新しただけで作ったものが消える、を残さない。
  await page.reload();
  await expect(page.locator("#boot-overlay")).toBeHidden({ timeout: 30_000 });
  await expect(page.locator("#crumb-own-scene")).toContainText("テストの場面");
  await expect
    .poll(() => page.locator("#hierarchy-tree .tree-body").count(), { timeout: 15_000 })
    .toBe(bodies);
  expect(errors).toEqual([]);
});

test("保存した場面は、⌘K からどこにいても開き直せる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.fill("#input-scene-name", "わたしの場面");
  await page.click("#btn-save-scene");
  await expect(page.locator("#crumb-own-scene")).toContainText("わたしの場面");

  // 用意された実験へ行ってから、名前で探して戻る。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("コーヒー");

  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "わたし");
  await expect(page.locator(".palette-row").first()).toContainText("わたしの場面");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-own-scene")).toContainText("わたしの場面");
  await expect(page.locator("#palette")).toBeHidden();
  expect(errors).toEqual([]);
});

test("自分で置いた物の動きが、そのままグラフと CSV に出る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());

  // 置いた物には観測点が付く——用意された実験でだけグラフが出る、を残さない。
  await page.click("#btn-run");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0.5);
  await page.click("#btn-run");

  await expect(page.locator("#probe-empty")).toBeHidden();
  await expect(page.locator("#btn-probe-csv")).toBeEnabled();
  await expect(page.locator("#probe-time-range")).toContainText("t =");
  expect(errors).toEqual([]);
});

test("材質を変えても、場面の名前と選んでいた物は変わらない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.fill("#input-scene-name", "名前つきの場面");
  await page.click("#btn-save-scene");

  await page.locator("#hierarchy-tree .tree-body").last().click();
  await page.selectOption("#inspector-material", "ゴム(天然)");
  await expect
    .poll(async () => page.locator("#inspector-material").inputValue(), { timeout: 10_000 })
    .toBe("ゴム(天然)");

  // 組み直しは「同じ場面の編集」であって差し替えではない。
  await expect(page.locator("#crumb-own-scene")).toContainText("名前つきの場面");
  await expect(page.locator('.card[data-card="focus"]')).toContainText("Sphere_1");
  expect(errors).toEqual([]);
});

test("描かれている現象に「形では見えません」と言わない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 天体は剛体を1つも持たないが、確かに描かれている。剛体の数で判断して
  // いたときは、見えているのに「形では見えません」と出ていた。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "惑星");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("惑星");
  await expect(page.locator("#stage-empty-note")).toBeHidden();

  // 棒の温度は本当に何も描かれない——こちらでは案内を出す。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "熱が棒");
  await page.keyboard.press("Enter");
  await expect(page.locator("#stage-empty-note")).toBeVisible();
  expect(errors).toEqual([]);
});

test("動きが止まったら、その時刻が数値に出る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 落として跳ねて止まるまで待つ。時計は回り続けるが、止まった時刻は別に出る。
  const settled = page.locator("#readout-settled");
  await expect(settled).toBeVisible({ timeout: 30_000 });
  const settledSeconds = Number(await settled.getAttribute("data-seconds"));
  expect(settledSeconds).toBeGreaterThan(0);
  // 経過時間はそのあとも進む——「止まった時刻」と取り違えないための別欄。
  await expect
    .poll(() => elapsedSeconds(page), { timeout: 15_000 })
    .toBeGreaterThan(settledSeconds);

  // 回り続ける現象では出さない。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "ふりこ");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("ふりこ");
  await page.waitForTimeout(4000);
  await expect(settled).toBeHidden();
  expect(errors).toEqual([]);
});

test("グラフの単位が、表の数値と同じ量になっている", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);

  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");

  // 表は ℃。グラフだけ生のケルビン(300 台)では同じ量だと気付けない。
  const readout = page.locator('#context dd[data-probe="0"]');
  await expect.poll(async () => (await readout.textContent()) ?? "", { timeout: 15_000 })
    .toContain("℃");
  const shown = Number.parseFloat((await readout.textContent()) ?? "999");
  expect(shown).toBeLessThan(150); // ケルビンなら 300 台になる

  // 桁の小さい時刻も指数表記にしない。
  await expect(page.locator("#probe-time-range")).not.toContainText("e-");
  expect(errors).toEqual([]);
});

test("つまみが無い実験は、無いと言う", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 場の中身そのものが記録された状態から始まる実験には、変えるつまみが無い。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d27-double-slit"]');
  const knobs = page.locator('.card[data-card="knobs"]');
  await expect(knobs).toBeVisible();
  await expect(knobs).toContainText("見るだけ");

  // 逆に、つまみを足した実験では実物が出る。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d16-conduction-race"]');
  await expect(page.locator("#knob-material")).toBeVisible();
  expect(errors).toEqual([]);
});

test("棒の材質を変えると、熱の伝わり方が実際に変わる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d16-conduction-race"]');

  const near = page.locator('#context dd[data-probe="1"]');
  await expect
    .poll(async () => Number.parseFloat((await near.textContent()) ?? "0"), {
      timeout: 20_000,
    })
    .toBeGreaterThan(5);

  // 木は熱をほとんど伝えない——同じ時間でも温度が上がらない。
  await page.click("#knob-material .knob-choice-btn:nth-child(4)");
  await page.waitForTimeout(4000);
  const wood = Number.parseFloat((await near.textContent()) ?? "99");
  expect(wood).toBeLessThan(1);
  expect(errors).toEqual([]);
});

test("グラフを指すと、その時刻の値が読める", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(1);

  // 巻き戻しは 1 秒ごとの記録にしか飛べない。コンマ何秒の値は、細かく持って
  // いる側(グラフ)を指して読む。
  const canvas = page.locator("#probe-canvas");
  const box = (await canvas.boundingBox())!;
  await page.mouse.move(box.x + box.width * 0.35, box.y + box.height * 0.5);
  await page.waitForTimeout(300);

  // 十字線と読み取り値が乗ったぶん、キャンバスの絵が変わる。
  const before = await canvas.screenshot();
  await page.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.5);
  await page.waitForTimeout(300);
  const after = await canvas.screenshot();
  expect(Buffer.compare(before, after)).not.toBe(0);
  expect(errors).toEqual([]);
});

test("書き出した数値には単位が付いている", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(1);

  const [download] = await Promise.all([
    page.waitForEvent("download"),
    page.click("#btn-probe-csv"),
  ]);
  const stream = await download.createReadStream();
  const chunks: Buffer[] = [];
  for await (const c of stream) chunks.push(c as Buffer);
  const header = Buffer.concat(chunks).toString("utf8").split("\n")[0];
  // 画面には ℃ と出ているのにファイルは数字だけ、を残さない。
  expect(header).toContain("[℃]");
  expect(header.startsWith("time_s,")).toBe(true);
  expect(errors).toEqual([]);
});

test("動く物が無い実験でも、真っ黒な3Dを説明なしに残さない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("コーヒー");
  // 場のパネルすら無い(熱のノードとグラフだけの)場面でも案内は出る。
  await expect(page.locator("#stage-empty-note")).toBeVisible();
  expect(errors).toEqual([]);
});

test("選んだものの札から、材質をその場で変えられる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());

  // 目の前の札で完結する——Inspector の 750px 下まで潜らせない。
  const select = page.locator("#focus-material");
  await expect(select).toBeVisible();
  const mass = () =>
    page
      .locator('[data-focus="重さ"]')
      .textContent()
      .then((t) => Number.parseFloat(t ?? "0"));
  // 札の値が埋まるまで待つ。埋まる前に読むと 0 を掴み、「軽くなったか」の
  // 比較そのものが意味を失う(遅い実行環境で実際に踏んだ)。
  await expect.poll(mass, { timeout: 15_000 }).toBeGreaterThan(0);
  const steel = await mass();
  await select.selectOption("ゴム(天然)");
  await expect.poll(mass, { timeout: 15_000 }).toBeLessThan(steel);
  expect(errors).toEqual([]);
});

test("置いた物は、置いた瞬間に画面で見える大きさで映る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(500);

  // 置いた物のところまで画角が寄る(以前は原点を見たままで、数ピクセルの
  // 点にしか見えなかった)。
  const near = await page.evaluate(() => {
    const hud = document.getElementById("hud");
    return hud?.textContent ?? "";
  });
  expect(near).toContain("12.0000 m");
  // 走らせなくても、そこに在ることが分かる。
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");
  expect(errors).toEqual([]);
});

test("水と分子の実験が、舞台に実際に描かれる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 水の粒は「+ 流体」で置いたときしか描いておらず、水を含むシーンを
  // 読み込むと一粒も出なかった。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d23-pouring-water"]');
  await expect(page.locator("#crumb-experiment")).toContainText("水を注ぐ");
  await expect(page.locator("#stage-empty-note")).toBeHidden();

  // 分子は実寸だと 1 画素にも満たず、真っ黒に見えていた。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d25-brownian"]');
  await expect(page.locator("#stage-empty-note")).toBeHidden();
  // 実物より大きく描いていることは、隠さず書く。
  await expect(page.locator("#context")).toContainText("実物より大きく描いています");
  expect(errors).toEqual([]);
});

test("止めている間は、時間の帯をつまんだ場所に留まる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  // スナップショットが貯まるまで走らせる。
  await expect.poll(() => elapsedSeconds(page), { timeout: 30_000 }).toBeGreaterThan(5);

  const scrubber = page.locator("#timeline-scrubber");
  const box = (await scrubber.boundingBox())!;
  const state = () =>
    scrubber.evaluate((el) => ({
      value: (el as HTMLInputElement).value,
      max: (el as HTMLInputElement).max,
    }));
  const before = await state();
  expect(Number(before.max)).toBeGreaterThan(1);

  await page.mouse.move(box.x + box.width - 6, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.3, box.y + box.height / 2, { steps: 10 });
  await page.mouse.up();
  await page.waitForTimeout(400);

  const after = await state();
  // つまみは離した場所に留まり(以前は右端へ戻っていた)、
  expect(Number(after.value)).toBeLessThan(Number(before.value));
  // 記録した先の時点も消えない(以前は巻き戻した瞬間に捨てていた)。走らせて
  // いる間は記録が1つ増えることがあるので、**減っていないこと**を見る。
  expect(Number(after.max)).toBeGreaterThanOrEqual(Number(before.max));
  expect(errors).toEqual([]);
});

test("つまみは、壊れた結果しか出ない値を渡さない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d24-car"]');

  // 1.0 Hz まで下げられたときは車体が底づきして横倒しになり、走り出す前に
  // 止まっていた(進んだ距離 0.00 m のまま)。下限をその手前で止める。
  const knob = page.locator("#knob-suspension");
  await expect(knob).toHaveAttribute("min", "1.5");

  // いちばん柔らかい設定でも、車はちゃんと走る。
  await knob.fill("1.5");
  await knob.dispatchEvent("change");
  const distance = page.locator('#context dd[data-probe="0"]');
  await expect
    .poll(async () => Number.parseFloat((await distance.textContent()) ?? "0"), {
      timeout: 20_000,
    })
    .toBeGreaterThan(1);
  expect(errors).toEqual([]);
});

test("時間の帯には、何をするものか書いてある", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  // 浅い粒度では時刻も step も隠していたので、ただの飾りの線に見えていた。
  await expect(page.locator("#timeline-hint")).toBeVisible();
  await expect(page.locator("#timeline-time")).toBeVisible();
  // 深い粒度では生の値がその場所を使う。
  await setGrain(page, 3);
  await expect(page.locator("#timeline-hint")).toBeHidden();
  await expect(page.locator("#timeline-step")).toBeVisible();
  expect(errors).toEqual([]);
});

test("見えている物が選べない場面では、その理由を言う", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d34-solar-system"]');

  // 惑星は目の前を回っているのに、以前は「動く物がありません」とだけ出ていた。
  const inspector = page.locator("#inspector-body");
  await expect(inspector).toContainText("天体");
  await expect(inspector).toContainText("クリックしても選べません");
  await expect(inspector).toContainText("いまの数値");

  // 横軸は**軸ぜんぶで同じ単位**(左端が「時間」で右端が「日」になっていた)。
  const range = (await page.locator("#probe-time-range").textContent()) ?? "";
  // 単位の書き方は右パネルの「経過した時間」と同じ日本語(空白を挟む)。
  const units = [
    ...range.matchAll(/[0-9.]+\s*(年|日|時間|分|秒|ミリ秒|マイクロ秒|ナノ秒|ピコ秒)/g),
  ].map((m) => m[1]);
  expect(units.length).toBe(2);
  expect(units[0]).toBe(units[1]);
  expect(errors).toEqual([]);
});

test("選んだものの札から、置き場所を数値で決められる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());

  // 座標の欄は Inspector のずっと下にあり、見つけられないまま「2つの物を
  // ぶつける」を諦めていた。目の前の札で決められるようにした。
  const x = page.locator("#focus-pos-x");
  await expect(x).toBeVisible();
  await x.fill("3");
  await x.dispatchEvent("change");
  await expect.poll(async () => Number(await x.inputValue()), { timeout: 10_000 })
    .toBeCloseTo(3, 2);
  expect(errors).toEqual([]);
});

test("とめている間なら、材質を変えられる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());

  // 走らせて、止める。「止めているのに『とめている間だけ』と断られる」を
  // 残さない。
  await page.click("#btn-run");
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "true");
  await page.click("#btn-run");
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");

  await page.selectOption("#focus-material", "ゴム(天然)");
  await expect
    .poll(async () => page.locator("#focus-material").inputValue(), { timeout: 10_000 })
    .toBe("ゴム(天然)");
  expect(errors).toEqual([]);
});

test("グラフに、プログラムの変数名を出さない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);

  // カタログが名前を与えていない系列は、Rust 側の生ラベル(`AstroPosX[0]`、
  // `BodySpeed(chassis)`)がそのまま凡例に出ていた。やさしい日本語の画面に
  // 突然変数名が現れ、「壊れてるのかな」と読まれた。
  for (const id of ["d36-swingby", "d24-car"]) {
    await page.keyboard.press("Control+k");
    await page.click(`.palette-row[data-experiment-id="${id}"]`);
    await page.waitForTimeout(1500);
    const tree = (await page.locator("#hierarchy-tree").textContent()) ?? "";
    expect(tree).not.toMatch(/AstroPos|AstroVel|BodyPosY|BodyPosX|BodySpeed/);
  }

  // 時間の表示も、右の「経過した時間」と同じ言葉にそろえる。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d30-gas-box"]');
  await expect
    .poll(async () => (await page.locator("#timeline-time").textContent()) ?? "", {
      timeout: 15_000,
    })
    .toContain("ピコ秒");
  expect(errors).toEqual([]);
});

test("つまみが指す値と、走っている値が食い違わない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("コーヒー");

  // 範囲入力は目盛りに乗らない既定値を表示のときに丸める。帯が 75 を指して
  // いるのに中身は 77 のまま、という食い違いを残さない。
  const shown = Number(await page.locator("#knob-temperature").inputValue());
  await expect
    .poll(
      async () =>
        Number.parseFloat(
          (await page.locator('#context dd[data-probe="0"]').textContent()) ?? "0",
        ),
      { timeout: 15_000 },
    )
    .toBeLessThanOrEqual(shown + 0.5);
  expect(errors).toEqual([]);
});

test("時間の単位が、画面のどこでも同じ", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d30-gas-box"]');

  // 右は「373.00 ピコ秒」なのに下とグラフ軸は「0.4ns」、が起きていた。
  const unitOf = (text: string) =>
    text.match(/(年|日|時間|分|秒|ミリ秒|マイクロ秒|ナノ秒|ピコ秒)/)?.[1];
  await expect
    .poll(async () => (await page.locator("#timeline-time").textContent()) ?? "", {
      timeout: 15_000,
    })
    .toContain("ピコ秒");
  const elapsed = (await page.locator("#readout-time").textContent()) ?? "";
  const timeline = (await page.locator("#timeline-time").textContent()) ?? "";
  const range = (await page.locator("#probe-time-range").textContent()) ?? "";
  expect(unitOf(timeline)).toBe(unitOf(elapsed));
  expect(unitOf(range)).toBe(unitOf(elapsed));

  // まだ 1 step も進んでいない瞬間でも食い違わない。「0 秒」と決め打ちして
  // いたので、開いた直後だけ右が「0 秒」・下が「0.00 ピコ秒」になっていた
  // (遅い機械の CI で実際に踏んだ)。
  await page.click("#btn-restart");
  await expect
    .poll(async () => (await page.locator("#readout-time").textContent()) ?? "", {
      timeout: 10_000,
    })
    .toContain("ピコ秒");
  expect(errors).toEqual([]);
});

test("用意された実験に足したものも、名前を付けて取っておける", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d1-free-fall"]');
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());

  // 以前は「自分の場面」のときしか保存の口が無く、実験に物を足した人は
  // 取っておく場所を見つけられないまま別の実験へ移り、戻れなくなった。
  await page.fill("#input-scene-name", "ぶつける実験");
  await page.click("#btn-save-scene");
  await expect(page.locator(".saved-scene-open")).toContainText("ぶつける実験");

  // 別の実験へ寄り道してから、⌘K で戻ってこられる。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "跳ね");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("跳ね");

  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "ぶつける");
  await expect(page.locator(".palette-row").first()).toContainText("ぶつける実験");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-own-scene")).toContainText("ぶつける実験");
  expect(errors).toEqual([]);
});

test("「みる」を選んだら、実験を選び直しても「みる」のまま", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);

  // 「みる」にしても、実験を選ぶたびに「さわる」へ戻っていた——見るだけの
  // 人が実験ひとつごとにダイヤルを押し直す羽目になっていた(利用者役①)。
  const grain = () =>
    page.evaluate(() => document.getElementById("app")!.dataset.grain);
  const openExperiment = async (id: string) => {
    await page.keyboard.press("Control+k");
    await page.click(`.palette-row[data-experiment-id="${id}"]`);
    await page.waitForTimeout(1200);
  };

  // 3D に物が映る実験は、いくつ選び直しても「みる」のまま。
  for (const id of ["d34-solar-system", "d12-ragdoll", "d6-floating"]) {
    await openExperiment(id);
    expect(await grain()).toBe("watch");
  }

  // 舞台に形のある物が出ない実験だけは、グラフが読める濃さまで開く。
  await openExperiment("d9-cooling-coffee");
  await expect
    .poll(grain, { timeout: 10_000 })
    .not.toBe("watch");

  // それでも、次に 3D の実験へ移れば「みる」へ戻る(上げたのは一時的)。
  await openExperiment("d1-free-fall");
  await expect.poll(grain, { timeout: 10_000 }).toBe("watch");
  expect(errors).toEqual([]);
});

test("3D を引っぱっても、選んだものの札が勝手に開かない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await page.waitForTimeout(1500);

  // 画面の大半を占める床の上でドラッグすると、視点が回らずに床が「選んだ
  // もの」として開いていた——見回そうとしただけで材質や座標の欄が出てきた
  // (利用者役①)。動かせない物の上の引っぱりは、視点回しに譲る。
  const box = (await page.locator("#scene-view canvas").first().boundingBox())!;
  const startX = box.x + box.width / 2;
  const startY = box.y + box.height * 0.85;
  await page.mouse.move(startX, startY);
  await page.mouse.down();
  for (let i = 1; i <= 12; i += 1) {
    await page.mouse.move(startX + i * 14, startY - i * 2);
  }
  await page.mouse.up();
  await page.waitForTimeout(500);
  await expect(page.locator("#focus-material")).toHaveCount(0);
  expect(errors).toEqual([]);
});

test("つまみで変えた条件は、別のつまみを触っても残る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);

  // 「落とす高さ」を 50 m にしてから重力を月へ変えると、高さだけが 20 m へ
  // 戻っていた——高さのつまみが、名前を変えたあとのボディを指していなくて
  // 何も起きていなかった(利用者役②)。二つの条件を重ねて確かめられない。
  const height = page.locator("#knob-height");
  await height.fill("50");
  await height.dispatchEvent("change");
  const ballHeight = async () =>
    Number(
      (
        (await page.locator("#context .readouts dd").last().textContent()) ?? ""
      ).replace(/[^0-9.]/g, ""),
    );
  await expect.poll(ballHeight, { timeout: 10_000 }).toBeGreaterThan(40);

  await page.click("#knob-gravity button:has-text('月')");
  // 月にしても高さは 50 m のまま。落ち方だけが変わる。
  await expect.poll(ballHeight, { timeout: 10_000 }).toBeGreaterThan(40);
  await expect(height).toHaveValue("50");
  expect(errors).toEqual([]);
});

test("説明にゴムと書いてある実験は、ゴムで始まる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d3-bounce"]');

  // 「ゴムの球を落として、跳ね返る高さを見ます」と書いてある隣で、つまみの
  // 初期値が鋼を場面へ書き戻していた(利用者役②)。
  await expect(
    page.locator("#knob-material button.active"),
  ).toHaveText(/ゴム/);
  expect(errors).toEqual([]);
});

test("時間の帯は、どこまで戻れるのかを言う", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 記録は直近の数秒ぶんしか残らないので、帯の左端は 0 秒ではない。それを
  // 言わずにいたので「位置と時刻が対応していない」と読まれた(利用者役②)。
  await expect
    .poll(async () => (await page.locator("#timeline-hint").textContent()) ?? "", {
      timeout: 20_000,
    })
    .toMatch(/つまむと .+ 〜 .+ のあいだへ戻せます/);
  expect(errors).toEqual([]);
});

test("ずっと同じ値の線も、グラフの上に見える", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d20-generator"]');

  // 一定値の系列は、幅ゼロの範囲を割った結果いつも**下端**に描かれ、時刻の
  // 目盛り帯に重なって 1 本まるごと見えなかった(利用者役③:「電圧の線が
  // どこにあるのか全く見えない」)。まん中あたりに、重ならないよう引く。
  const canvas = page.locator("#probe-canvas");
  await expect(canvas).toBeVisible();
  await page.waitForTimeout(3000);

  // 一定値であることは凡例が言う(高さを値と読み違えないため)。
  // 折れ線を描く範囲の**下端**(時刻の目盛り帯のすぐ上)に、横いっぱいの
  // 明るい線が寝ていないことを見る。これが一定値の線が潰れていた場所。
  const bottomRun = await page.evaluate(() => {
    const c = document.getElementById("probe-canvas") as HTMLCanvasElement;
    const ctx = c.getContext("2d")!;
    const AXIS_BAND = 15;
    const plotBottom = c.height - AXIS_BAND;
    let worst = 0;
    for (let y = plotBottom - 3; y <= plotBottom; y += 1) {
      if (y < 0 || y >= c.height) continue;
      const row = ctx.getImageData(0, y, c.width, 1).data;
      let lit = 0;
      for (let x = 0; x < c.width; x += 1) {
        const i = x * 4;
        if (row[i] + row[i + 1] + row[i + 2] > 260) lit += 1;
      }
      worst = Math.max(worst, lit);
    }
    return { worst, width: c.width };
  });
  expect(bottomRun.worst).toBeLessThan(bottomRun.width * 0.5);
  expect(errors).toEqual([]);
});

test("場面の中身は、日本語で並ぶ", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);

  // 数値とグラフを見に来た人の目に「World Root」「Bodies」「Probes」といった
  // 中の言葉が並んでいた(利用者役③)。
  const tree = page.locator("#hierarchy-tree");
  await expect(tree).toContainText("この場面ぜんぶ");
  await expect(tree).toContainText("物");
  await expect(tree).not.toContainText("World Root");
  await expect(tree).not.toContainText("Bodies");
  expect(errors).toEqual([]);
});

test("粒が何百個もある場面でも、一覧が壁にならない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d25-brownian"]');

  // ブラウン運動は粒が 300 個あり、左が 300 行の壁になっていた(利用者役③)。
  await expect
    .poll(
      async () =>
        await page.locator("#hierarchy-tree .tree-body:visible").count(),
      { timeout: 15_000 },
    )
    .toBeLessThan(100);
  expect(errors).toEqual([]);
});

test("固定にした物でも、画面から見失わない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());
  await expect(page.locator("#focus-pos-y")).toBeVisible();

  // 動き方を「固定」にすると、カメラが画角を決める手がかりを失って一歩も
  // 動けなくなり、置き場所を数値で変えた物は画面の外へ消えたきりだった。
  // 舞台まで「形のある物は出てきません」と言い張っていた(利用者役④)。
  await page.selectOption("#inspector-body-type", "Static");
  // 選んだ値は、次の step で効くまでのあいだも欄に残る。
  await expect(page.locator("#inspector-body-type")).toHaveValue("Static");
  await page.click("#btn-run");
  await page.waitForTimeout(700);
  await page.click("#btn-run");

  const y = page.locator("#focus-pos-y");
  await y.fill("1.5");
  await y.dispatchEvent("change");
  await page.waitForTimeout(1200);
  // 形のある物が在るのだから、そうは言わない。
  await expect(page.locator("#scene-view")).toHaveAttribute(
    "data-stage-empty",
    "false",
  );
  // 「全体へ戻る」は名前どおり画角も戻す。
  await page.click("#btn-clear-selection");
  await page.waitForTimeout(1200);
  expect(errors).toEqual([]);
});

test("取っておけたことが、画面に出る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());

  // 押しても何も変わらず、保存できたのか押し損ねたのか分からなかった
  // (利用者役④)。名前は書き出す文書にも残す。
  await page.fill("#input-scene-name", "わたしの落下じっけん");
  await page.click("#btn-save-scene");
  await expect(page.locator("#scene-save-status")).toContainText(
    "わたしの落下じっけん",
  );
  const storedName = await page.evaluate(() => {
    const raw = localStorage.getItem("simulator.scenes.saved") ?? "[]";
    return JSON.parse(JSON.parse(raw)[0].json).name as string;
  });
  expect(storedName).toBe("わたしの落下じっけん");

  // 名前で呼び出せる。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "わたしの");
  await expect(page.locator(".palette-row").first()).toContainText(
    "わたしの落下じっけん",
  );
  expect(errors).toEqual([]);
});

test("用意された実験に足した物は、その場面の大きさで出てくる", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d11-pendulum"]');
  await page.waitForTimeout(1500);

  // 置く高さが 12 m 固定で、振れ幅 ±1 m ほどのふりこに足した箱が、遠い空から
  // 降ってくる豆粒にしかならなかった(利用者役④)。
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());
  await expect(page.locator("#focus-pos-y")).toBeVisible();
  const y = Number(await page.locator("#focus-pos-y").inputValue());
  expect(y).toBeLessThan(8);
  expect(y).toBeGreaterThan(0);
  expect(errors).toEqual([]);
});

test("水を注ぐ実験は、受け止める器も描く", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d23-pouring-water"]');
  await page.waitForTimeout(3000);

  // 「水のかたまりが落ちて、容器に溜まります」と書いてある隣で、真っ暗な空間に
  // 水色の塊が浮いているだけに見えた——器は物理側に境界粒子として在るのに、
  // 画面に描いていなかった(利用者役①)。
  await expect(page.locator("#scene-view")).toHaveAttribute(
    "data-fluid-boundary",
    "true",
  );
  await expect(page.locator("#scene-view")).toHaveAttribute(
    "data-stage-empty",
    "false",
  );
  expect(errors).toEqual([]);
});

test("グラフがまだ出ていない濃さでは、グラフを見ろと言わない", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d34-solar-system"]');
  await page.waitForTimeout(2000);

  // 「まん中の 3D と、下のグラフの両方に出ます」と書いてある下は真っ黒だった
  // ——「みる」ではグラフを出していないため(利用者役①)。
  const where = page.locator(".card-where");
  await expect(where).toContainText("ダイヤル");
  await setGrain(page, 2);
  await expect(where).toContainText("下のグラフ");
  expect(errors).toEqual([]);
});

test("数値は、途中で折り返さない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d12-ragdoll"]');

  // 「6.72 秒」が「6.7 / 2 / 秒」と三行に割れていた(利用者役①)。名前の列が
  // 長いぶんを値の列から奪っていたため。
  await expect
    .poll(
      async () =>
        await page.evaluate(() => {
          const dds = [
            ...document.querySelectorAll("#context .readouts dd"),
          ] as HTMLElement[];
          if (dds.length === 0) return -1;
          return Math.max(...dds.map((d) => d.getBoundingClientRect().height));
        }),
      { timeout: 15_000 },
    )
    .toBeLessThan(28);
  expect(errors).toEqual([]);
});

test("大きさの表示と重さが噛み合う", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());

  // 内部表記は半分の長さなので、`Box(0.4000, …)` の箱が 4019 kg になり
  // 「表示と重さが合わない」と読まれた。人が言う一辺の長さで書く。
  const shape = page.locator('[data-focus="かたち"]');
  await expect(shape).toContainText("0.80 × 0.80 × 0.80 m");

  // 一辺 0.8m の鋼(密度 7850)は約 4019 kg——数と重さが噛み合う。
  const mass = Number.parseFloat(
    (await page.locator('[data-focus="重さ"]').textContent()) ?? "0",
  );
  expect(mass).toBeGreaterThan(3900);
  expect(mass).toBeLessThan(4100);
  expect(errors).toEqual([]);
});

// カタログの全実験が、パレットから選んで実際に動くことを分野ごとに確認する。
for (const category of CATEGORIES) {
  test(`分野「${category.title}」の実験がすべて動く`, async ({ page }) => {
    const errors = collectPageErrors(page);
    await boot(page);

    for (const experiment of category.experiments) {
      await page.keyboard.press("Control+k");
      await expect(page.locator("#palette")).toBeVisible();
      await page.click(`.palette-row[data-experiment-id="${experiment.id}"]`);
      await expect(page.locator("#crumb-experiment")).toContainText(experiment.title);
      await expect
        .poll(() => elapsedSeconds(page), { timeout: 15_000 })
        .toBeGreaterThan(0);
    }
    expect(errors).toEqual([]);
  });
}
