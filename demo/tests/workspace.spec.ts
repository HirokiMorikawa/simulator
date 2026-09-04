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
  const units = [...range.matchAll(/[0-9.]+(年|日|時間|分|s|ms|µs)/g)].map((m) => m[1]);
  expect(units.length).toBe(2);
  expect(units[0]).toBe(units[1]);
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
