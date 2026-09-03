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
