import { expect, test, type Locator, type Page } from "@playwright/test";
import { addViaMenu, collectPageErrors } from "./helpers";
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
  await expect(page.locator("#hierarchy-tree")).toContainText("球 1");
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

test("つなぎ目のある場面も、保存して開き直せる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-pendulum")!.click());
  await expect(page.locator("#hierarchy-tree")).toContainText("つなぎ目");
  const bodies = await page.locator("#hierarchy-tree .tree-body").count();

  await page.fill("#input-scene-name", "ふりこの場面");
  await page.click("#btn-save-scene");
  await expect(page.locator("#crumb-own-scene")).toContainText("ふりこの場面");

  // 用意された実験へ行ってから、名前で探して戻る。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "コーヒー");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-experiment")).toContainText("コーヒー");

  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "ふりこの場面");
  await expect(page.locator(".palette-row").first()).toContainText("ふりこの場面");
  await page.keyboard.press("Enter");
  await expect(page.locator("#crumb-own-scene")).toContainText("ふりこの場面");
  // 物もつなぎ目も、保存したときのまま戻ってくる。書き出しがボディの名前だけを
  // 画面の名前へ書き換えていた頃は、つなぎ目が消えた名前を指したままになり、
  // 読み込みが `UnknownBodyName` で落ちて**画面が何も変わらなかった**。
  await expect
    .poll(() => page.locator("#hierarchy-tree .tree-body").count(), { timeout: 15_000 })
    .toBe(bodies);
  await expect(page.locator("#hierarchy-tree")).toContainText("つなぎ目");
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
  await expect(page.locator('.card[data-card="focus"]')).toContainText("球 1");
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

test("熱が棒を伝わるの材質ヒントに、摩擦の説明が付かない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d16-conduction-race"]');

  // `562eb88` の退行: 「材質ボタンに摩擦係数を添える」変更のヒント文が、
  // 摩擦と何の関係も無いこの実験にまで漏れていた。この実験の材質つまみは
  // 熱拡散率(数値)を選ぶだけで、ボタンにも摩擦の数字は付かない
  // ——ヒントにも付いてはいけない。
  const buttons = page.locator("#knob-material .knob-choice-btn");
  await expect(buttons).toHaveCount(4);
  for (const button of await buttons.all()) {
    expect(await button.getAttribute("data-friction")).toBeNull();
  }
  const hint = page.locator("#knob-material").locator(
    "xpath=following-sibling::p[contains(@class,'knob-hint')]",
  );
  await expect(hint).not.toContainText("摩擦");
  await expect(hint).toContainText("木はほとんど伝わりません");
  expect(errors).toEqual([]);
});

test("坂の実験の材質ヒントには、摩擦の説明が付く", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d5-incline"]');

  // こちらは材質つまみの値がそのまま材質名で、ボタンにも実際の摩擦係数が
  // 添えられる実験——摩擦の読み方の一言はここでこそ意味を持つ。
  const buttons = page.locator("#knob-material .knob-choice-btn");
  await expect(buttons).toHaveCount(6);
  for (const button of await buttons.all()) {
    expect(await button.getAttribute("data-friction")).not.toBeNull();
  }
  const hint = page.locator("#knob-material").locator(
    "xpath=following-sibling::p[contains(@class,'knob-hint')]",
  );
  await expect(hint).toContainText("摩擦の数字が小さいほどよく滑ります");
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

// 利用者役「しらべる」の観察: つまみの `step` が空(既定値の1が黙って効く)に
// 見えて、「0 と 1 の2箇所にしか止まらない/途中の時刻が拾えない」と読まれた。
// 再現すると、`value`/`max` はスナップショットの**index**(常に整数)であり、
// 記録が2つしか無い立ち上がり直後だけ実際に0と1の2択になる——記録が貯まれば
// 端でない位置(index)へも動かせ、そこに対応する**端でない時刻**が読める。
// `step` は「index が整数である」ことを画面にも明示するため 1 を明示する
// (`index.html` 側のdoc参照)。
test("つまみを途中の位置へ動かすと、その時刻の値が読める", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  // リングバッファ(最大8個)が複数貯まるまで走らせる——端点2つだけでなく
  // 「途中」と呼べる位置が実在することを確かめたい。
  await expect.poll(() => elapsedSeconds(page), { timeout: 30_000 }).toBeGreaterThan(6);

  const scrubber = page.locator("#timeline-scrubber");
  await expect(scrubber).toHaveAttribute("step", "1");
  const max = Number(await scrubber.getAttribute("max"));
  expect(max).toBeGreaterThan(2);

  const box = (await scrubber.boundingBox())!;
  // 止める。
  await page.mouse.move(box.x + box.width - 6, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.up();
  await page.waitForTimeout(200);

  // 端(0 / max)ではない、途中の位置へつまみを動かす。
  await page.mouse.move(box.x + box.width - 6, box.y + box.height / 2);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height / 2, { steps: 10 });
  await page.mouse.up();
  await page.waitForTimeout(300);

  const value = Number(await scrubber.inputValue());
  expect(value).toBeGreaterThan(0);
  expect(value).toBeLessThan(max);

  // その位置に対応する、端でない時刻が画面に読める(飾りの帯ではない)。
  const timeText = await page.locator("#timeline-time").textContent();
  const match = timeText?.match(/t = ([\d.]+) 秒/);
  expect(match).not.toBeNull();
  const shownTime = Number(match![1]);

  const [minTime, maxTime] = await Promise.all([
    scrubber.evaluate(() =>
      Number((window as any).__world.read_component("snapshot_time_at", "0")),
    ),
    scrubber.evaluate((_el, m) =>
      Number((window as any).__world.read_component("snapshot_time_at", String(m))),
      max,
    ),
  ]);
  // 案内文が約束する範囲(min〜max)の**内側**が読める——端点の使い回しではない。
  expect(shownTime).toBeGreaterThan(minTime);
  expect(shownTime).toBeLessThan(maxTime);
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

test("つまみの端でも、数値が発散しない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d17-piston"]');

  // 2.0 m/s まで押せたときは気体がほぼ 0 まで潰れて圧力が発散し、位置が
  // -5.4e7 m、速さが 2.1e7 m/s という壊れた数字になった(利用者役②)。
  const knob = page.locator("#knob-push");
  await expect(knob).toHaveAttribute("max", "1.5");
  await knob.fill("1.5");
  await knob.dispatchEvent("change");
  await page.waitForTimeout(6000);
  const values = await page.locator("#context .readouts dd").allTextContents();
  for (const text of values) {
    const n = Number.parseFloat(text.replace(/[^0-9.eE+-]/g, ""));
    if (Number.isFinite(n)) expect(Math.abs(n)).toBeLessThan(1e5);
  }
  expect(errors).toEqual([]);
});

test("無くなった物の値を、壊れた数字で出さない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d18b-ice-melts"]');
  await page.waitForTimeout(800);

  // 融け切った氷は内部で遠方(-1e9 m)へ退避する。その値がそのまま
  // 「氷の高さ -1,000,000,000.000 m」と出ていた(利用者役②)。
  await page.evaluate(() => {
    const r = document.querySelector<HTMLInputElement>(
      '.knob input[type="range"]',
    );
    if (!r) return;
    r.value = r.max;
    r.dispatchEvent(new Event("input", { bubbles: true }));
    r.dispatchEvent(new Event("change", { bubbles: true }));
  });
  await page.click('button:has-text("はやい")');
  await expect
    .poll(
      async () =>
        (await page.locator("#context .readouts").textContent()) ?? "",
      { timeout: 40_000 },
    )
    .toContain("もう在りません");
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
  // **打ちかけの値は、札が組み直されても消えない**。組み直しが打った直後に
  // 挟まると、欄が元の位置に戻り、そのまま「決定」されて別の場所へ動いていた。
  await page.waitForTimeout(1200);
  await expect(x).toHaveValue("3");
  await x.dispatchEvent("change");
  await expect.poll(async () => Number(await x.inputValue()), { timeout: 10_000 })
    .toBeCloseTo(3, 2);
  expect(errors).toEqual([]);
});

/**
 * 選んだ物が、いまの画角に**ちゃんと映っているか**を`main.ts`の
 * `isWellVisible`と同じ判定(投影した点が画角の内側にあるか)で読む。
 * `window.__camera`/`window.__world`はテスト専用に露出されている。
 */
async function bodyOnScreen(page: Page): Promise<boolean> {
  return page.evaluate(() => {
    const cam = (window as unknown as {
      __camera: {
        matrixWorldInverse: { elements: number[] };
        projectionMatrix: { elements: number[] };
      };
    }).__camera;
    const world = (window as unknown as {
      __world: {
        read_component(kind: string, arg: string): string;
        body_position_at_f32(index: number): Float32Array;
      };
    }).__world;
    const index = Number(world.read_component("body_count", "")) - 1; // 直近に置いた物
    const p = world.body_position_at_f32(index);
    const mulMat4Vec4 = (e: number[], v: number[]) => {
      const out = [0, 0, 0, 0];
      for (let r = 0; r < 4; r += 1) {
        out[r] = e[r] * v[0] + e[4 + r] * v[1] + e[8 + r] * v[2] + e[12 + r] * v[3];
      }
      return out;
    };
    const view = mulMat4Vec4(cam.matrixWorldInverse.elements, [p[0], p[1], p[2], 1]);
    const clip = mulMat4Vec4(cam.projectionMatrix.elements, view);
    if (clip[3] <= 0) return false; // カメラの後ろ
    const ndc = [clip[0] / clip[3], clip[1] / clip[3], clip[2] / clip[3]];
    return Math.abs(ndc[0]) <= 1 && Math.abs(ndc[1]) <= 1 && ndc[2] <= 1;
  });
}

test("置き場所を数値で変えても、その物は画面から消えない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(500);

  // 置いた直後は物のすぐ近くまで画角が寄っている(至近距離)。ここから
  // 数値で大きく動かすと、以前は画角が一切追随せず、物だけが動いて画面には
  // 何も映らない床だけが残った(利用者役①の報告:「数値は正しく変わって
  // いるのに、画面には何もない床だけが残る」)。
  await expect.poll(() => bodyOnScreen(page), { timeout: 5_000 }).toBe(true);

  const y = page.locator("#focus-pos-y");
  await y.fill("9");
  await y.dispatchEvent("change");
  await page.waitForTimeout(800);

  expect(await bodyOnScreen(page)).toBe(true);
  expect(errors).toEqual([]);
});

/**
 * カメラから**直近に置いた物**への向きの、水平面に対する仰角(sin)と距離。
 * `frameCameraOnContent`/`updateGuidedFollowCamera`が向きを決めるのに使って
 * いるのと同じ量(`window.__camera`/`window.__world`はテスト専用に露出)。
 */
async function cameraToLastBody(page: Page): Promise<{ elevation: number; distance: number }> {
  return page.evaluate(() => {
    const cam = (window as unknown as {
      __camera: { position: { x: number; y: number; z: number } };
    }).__camera;
    const world = (window as unknown as {
      __world: {
        read_component(kind: string, arg: string): string;
        body_position_at_f32(index: number): Float32Array;
      };
    }).__world;
    const index = Number(world.read_component("body_count", "")) - 1;
    const p = world.body_position_at_f32(index);
    const dx = cam.position.x - p[0];
    const dy = cam.position.y - p[1];
    const dz = cam.position.z - p[2];
    const distance = Math.hypot(dx, dy, dz);
    return { elevation: distance > 1e-9 ? dy / distance : 0, distance };
  });
}

/**
 * **選んだ物の見かけの直径 [px]**(課題2: 進行管理役がスクリーンショットで
 * 直接測ったのと同じ量——距離とカメラの`fov`・canvas の高さから逆算する。
 * 実測: `d1-free-fall`で粒度2から右クリックで2個目の球を置いたとき、
 * 距離47.3m・見かけの直径9.9pxだった)。直近に置いた物(`body_count - 1`)を
 * 対象にする。半径は`main.ts`の`SPAWN_SPHERE_RADIUS`と同じ0.4m決め打ち
 * ——このファイルのテストは球しか置かないため。
 */
async function apparentSphereDiameterPx(page: Page): Promise<number> {
  return page.evaluate(() => {
    const cam = (window as unknown as {
      __camera: { position: { x: number; y: number; z: number }; fov: number };
    }).__camera;
    const world = (window as unknown as {
      __world: {
        read_component(kind: string, arg: string): string;
        body_position_at_f32(index: number): Float32Array;
      };
    }).__world;
    const index = Number(world.read_component("body_count", "")) - 1;
    const p = world.body_position_at_f32(index);
    const dx = cam.position.x - p[0];
    const dy = cam.position.y - p[1];
    const dz = cam.position.z - p[2];
    const distance = Math.hypot(dx, dy, dz);
    const canvas = document.querySelector("#scene-view-canvas-host canvas") as HTMLCanvasElement;
    const heightPx = canvas.clientHeight;
    const fovRad = (cam.fov * Math.PI) / 180;
    const pxPerMeterAtDistance = heightPx / 2 / Math.tan(fovRad / 2) / distance;
    const RADIUS = 0.4; // SPAWN_SPHERE_RADIUS(main.ts)。
    return 2 * RADIUS * pxPerMeterAtDistance;
  });
}

test("置き場所を数値で高さを変えても、地平線が画角の外に消えない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(500);

  // **真上から床を覗き込む向きにならないこと**。以前は前の画角(スポーン直後の
  // 見上げ位置)をそのまま引き継いで合わせ直していたので、置き場所を数値で
  // y=3のような低い高さへ打ち替えると、仰角が0.9を超える(ほぼ真上からの
  // 見下ろし)ことがあった——地平線が画角の外へ出て、物が「宙に浮いている」
  // のか「床の上にある」のかが画面から読めなくなる(進行管理側の実測)。
  const y = page.locator("#focus-pos-y");
  await y.fill("3");
  await y.dispatchEvent("change");
  await page.waitForTimeout(500);
  expect(await bodyOnScreen(page)).toBe(true);
  const afterHigh = await cameraToLastBody(page);
  expect(afterHigh.elevation).toBeLessThan(0.6);

  // 低い高さ(y=0.5)へ変えても同じく崩れないこと。
  await y.fill("0.5");
  await y.dispatchEvent("change");
  await page.waitForTimeout(500);
  expect(await bodyOnScreen(page)).toBe(true);
  const afterLow = await cameraToLastBody(page);
  expect(afterLow.elevation).toBeLessThan(0.6);
  expect(errors).toEqual([]);
});

test("『全体へ戻る』を押しても、選んでいた物が豆粒にならない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(500);
  await expect.poll(() => bodyOnScreen(page), { timeout: 5_000 }).toBe(true);

  const clear = page.locator("#btn-clear-selection");
  await expect(clear).toBeVisible();
  await clear.click();
  await page.waitForTimeout(1000);

  // 追従カメラ(`updateGuidedFollowCamera`)をそのまま流用すると、原点(床)も
  // 画角に収めようとして大きく引いたあげく、「対象を見失わない」ための
  // 見かけの大きさの下限がそのまま効き、直前まで大きく見えていた球が豆粒に
  // なっていた(利用者役の報告、実測で再現)。半径0.4mの球なら、
  // `isWellVisible`と同じ「半径の20倍(=8m)より遠いと画面の高さの1割にも
  // 満たない粒になる」という基準に照らして、十分近いままであること。
  const after = await cameraToLastBody(page);
  expect(after.distance).toBeLessThan(6);
  expect(await bodyOnScreen(page)).toBe(true);
  expect(errors).toEqual([]);
});

test("「斜めに投げる」を開いた直後から、球が着地まで画面に映り続ける", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);

  // 秒速 20m・45°で水平に14m/s超で飛ぶ球。追従カメラの注視点(orbit.target)
  // が対象へ追いつく前の遅れが際限なく育つと、カメラの向きの都合でその遅れが
  // 真横方向に出て、球は画角の外へ出たまま二度と戻らない——案内は「まん中の
  // 3D を見てください」と言うのに、床のグリッドしか映らなかった(進行管理役
  // の実測: t=1.17〜7.12秒でカメラは球から2.3〜2.8mしか離れていないのに
  // 画面には映っていなかった)。「見え方」の「カメラを合わせ直す」を押して
  // 初めて映る、では遅い——**開いた直後から**映っていること。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d2-ballistic"]');
  await page.locator("#crumb-experiment").waitFor({ state: "visible", timeout: 10_000 });

  // 開いた直後(合わせ直しボタンに触れる前)。
  await expect.poll(() => bodyOnScreen(page), { timeout: 5_000 }).toBe(true);

  // 打ち上げ(約10m)から着地(飛行時間 約2.9秒)まで、複数時点で映り続ける。
  for (const waitMs of [400, 500, 500, 500, 500, 500, 500]) {
    await page.waitForTimeout(waitMs);
    expect(await bodyOnScreen(page)).toBe(true);
  }
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

test("とめてから重力のつまみを動かしても、止まったまま", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);

  // 利用者役の報告: 「とめる」で一時停止していたのに、重力のつまみ(選択式)
  // を押したら勝手に再生が始まった。ツールチップは「動かすと、その設定で
  // 最初からやり直します」としか言っておらず、「止めていたのに動き出す」
  // ことまでは書いていなかった——止めた意思を尊重し、止めたままやり直す
  // ようにした。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d1-free-fall"]');
  await page.locator("#crumb-experiment").waitFor({ state: "visible", timeout: 10_000 });

  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "true");
  await page.click("#btn-run");
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");

  await page.click('#knob-gravity button[data-value="1.62"]'); // 月
  await page.waitForTimeout(500);
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");

  // 止めたままでも、つまみそのものはちゃんと効いている(見た目の状態だけ
  // 差し替えて、中身が古いまま、ではないこと)。
  await expect(page.locator('#knob-gravity button[data-value="1.62"]')).toHaveClass(
    /active/,
  );
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

  // 舞台に形のある物が出ない実験(D9)でも、いまは「みる」のまま
  // ——課題A(利用者役の報告)より前は、グラフを見せるために大局の粒度
  // そのものを「さわる」まで押し上げていて、「変えてみる」(つまみ)まで
  // 一緒に開いてしまっていた。いまはグラフの段だけを個別に強制する
  // (`forceAnalysisOpen`、workspace.tsのdoc参照)ので、「みる」は「みる」の
  // まま——次の「電気の工作台」のテストで、グラフだけが開くことを見る。
  await openExperiment("d9-cooling-coffee");
  await expect.poll(grain, { timeout: 10_000 }).toBe("watch");

  // 3D の実験へ移っても、引き続き「みる」のまま。
  await openExperiment("d1-free-fall");
  await expect.poll(grain, { timeout: 10_000 }).toBe("watch");
  expect(errors).toEqual([]);
});

// 課題A(利用者役の報告・進行管理役の裏取り): 「電気の工作台」は3Dに映る物が
// ひとつも無い(回路のみのシーン)。以前は「みる」で開いても、グラフが出る
// ころには大局の粒度が「さわる」まで上がっていて、「変えてみる」(つまみ)
// まで開いてしまっていた——「みる」の画面が道具だらけになる、という別の
// 問題を生んでいた。いまは大局の粒度(ダイヤル・「変えてみる」の開閉)には
// 触れず、グラフの段だけを強制的に開く。
test("舞台に形のある物が無い実験は、「みる」のままグラフの段だけが開く(課題A)", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d19-electric-workbench"]');

  const grain = () =>
    page.evaluate(() => document.getElementById("app")!.dataset.grain);
  const analysisOpen = () =>
    page.evaluate(() => document.getElementById("app")!.dataset.analysis);
  const knobsExpanded = () =>
    page.evaluate(() =>
      document
        .querySelector('.card[data-card="knobs"]')
        ?.getAttribute("data-expanded"),
    );

  // 「舞台が空」と決まるまで数十フレーム待つ(誤検出の防止、
  // `STAGE_EMPTY_FRAMES`のdoc参照)ので、ここは`poll`で待ち合わせる。
  await expect.poll(analysisOpen, { timeout: 5_000 }).toBe("true");

  // グラフの段が開いても、「みる」のまま。つまみの「変えてみる」は畳まれた
  // ままで、大局の粒度が引きずられて開くことはない。
  expect(await grain()).toBe("watch");
  expect(await knobsExpanded()).toBe("false");

  // 「ここを見る」の文面も、実際に出ているグラフを指す(ダイヤルを回せとは
  // もう言わない)。
  await expect(page.locator(".card-where")).toContainText("下のグラフ");

  // グラフの実データ(色分けした折れ線)が実際に描かれている。
  const canvas = page.locator("#probe-canvas");
  await expect(canvas).toBeVisible();
  const canvasHeight = await canvas.evaluate((el) => el.clientHeight);
  expect(canvasHeight).toBeGreaterThan(50);

  // 3D に物が映る実験へ移れば、グラフの段は畳まれ、「みる」のまま。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d1-free-fall"]');
  await page.waitForTimeout(500);
  expect(await grain()).toBe("watch");
  await expect.poll(analysisOpen, { timeout: 5_000 }).toBe("false");
  expect(errors).toEqual([]);
});

// 課題B(利用者役の報告): 「氷が水に変わる」の氷は床が無く、永遠に落ち続けて
// いた(2.5秒で y=-17.91、進行管理役の実測)。「氷の高さ」がどんどん大きな
// 負の値になるのは、融解とは無関係のノイズでしかなかった。SPHの器の床
// (`sph.raw_state.boundary_position`、y=-0.05)に合わせた静止した床を
// シーンJSONへ足し、氷がその場に留まるようにした。
test("「氷が水に変わる」の氷は、床の上に留まって落ち続けない(課題B)", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d18b-ice-melts"]');
  await page.waitForTimeout(3_000);

  const iceHeight = await page.evaluate(() => {
    const w = window.__world as {
      body_position_at_f32: (i: number) => Float32Array;
    };
    return w.body_position_at_f32(0)[1];
  });
  // 落ち続けていれば数メートル単位の負の値になる(進行管理役の実測: 2.5秒で
  // -17.91m)。床に留まっていれば、氷の半分の厚み程度(0.05m)を大きくは
  // 超えない。
  expect(Math.abs(iceHeight)).toBeLessThan(0.2);

  // 案内文も、もう「液体の粒が生まれます」とは書かない(粒は物理には実在
  // するが、生成直後に物理側の不具合で弾け飛び、正しい姿を描ける状態では
  // ない——Rustは今回の増分の対象外なので、確実に見える範囲だけを書く)。
  const watchText = await page
    .locator(".card-watch")
    .evaluate((el) => el.textContent ?? "");
  expect(watchText).not.toContain("液体の粒が生まれます");
  expect(errors).toEqual([]);
});

// 課題C(利用者役の報告): 「手回し発電機」の3Dに映る「軸」は模様の無い灰色の
// 球で、実際には回っていても見た目に手掛かりが無かった。物理の回転自体は
// 正しく進んでいる(実測で四元数が毎秒変わることを確認済み)ので、形状・
// 質量・慣性には触れず、描画だけに取っ手を足して回転を見えるようにした。
test("「手回し発電機」の軸は、取っ手が回って見える(課題C)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d20-generator"]');
  await page.waitForTimeout(1_000);

  // 取っ手(課題C、`meshFromShapeJson`のdoc参照)は球の子として足しているので、
  // その`matrixWorld`の並進成分(4列目)を直接読む——`THREE.Vector3`を
  // 新しく作らずに済む(evaluate内で`THREE`のグローバルが要らない)。
  const handleWorldX = async () => {
    return page.evaluate(() => {
      const scene = (
        window as unknown as {
          __scene: { traverse: (fn: (obj: any) => void) => void };
        }
      ).__scene;
      let x: number | null = null;
      scene.traverse((obj: any) => {
        if (obj.type === "Mesh" && obj.children.length === 1) {
          const handle = obj.children[0];
          if (handle.type === "Mesh") {
            handle.updateWorldMatrix(true, false);
            x = handle.matrixWorld.elements[12];
          }
        }
      });
      return x;
    });
  };

  const x1 = await handleWorldX();
  expect(x1).not.toBeNull();
  await page.waitForTimeout(800);
  const x2 = await handleWorldX();
  expect(x2).not.toBeNull();
  // 回っていれば、取っ手の世界座標は時間とともに変わる(円を描く)。
  expect(Math.abs((x1 as number) - (x2 as number))).toBeGreaterThan(0.01);
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
  // 引っぱり始めは**画面の隅の床**にする。まん中は落ちてくる物が通るので、
  // そこを掴むのは「動かせる物を掴む」という別の機能(そちらは選んで正しい)。
  const box = (await page.locator("#scene-view canvas").first().boundingBox())!;
  const startX = box.x + box.width * 0.12;
  const startY = box.y + box.height * 0.88;
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

test("水を注ぐ実験は、着水後もRustの受け入れ基準どおり床を抜けない", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d23-pouring-water"]');
  await page.waitForTimeout(500);

  // 前回(127077e)は「粒が容器の外へ弾け飛ばない」ことを期待して
  // `viscosity_alpha` を 0.08→0.5 に変えたが、これは間違いだった——弱圧縮性が
  // 崩れ、内部粒子の密度が静止密度から25%も下振れし、Rustの受け入れテスト
  // (`scenario.rs`の`run_headless_scenario_pouring_water_keeps_interior_density_
  // near_rest_density`)が落ちた。`viscosity_alpha`は0.08へ戻した。
  //
  // 0.08に戻すと、実測(`cargo run --example`で手動追跡)では次のことが分かる:
  //   - 着水の瞬間に一部の粒が秒速7〜9mほどまで跳ね上がる。理論上の落下速度
  //     (この高さからの自由落下では秒速2.6〜2.8m程度)より明らかに大きく、
  //     格子状に並んだ粒がほぼ同時に着水することで起きる衝撃的な速度スパイク
  //     ——SPHの初期条件に起因する既知の限界であって、`viscosity_alpha`だけで
  //     消せるものではない(前回それを消そうとして密度テストを壊した)。
  //   - この粒は静止画ではなく実際に放物線を描いて動いている。人の目に「浮いた
  //     まま止まって見える」のは、頂点(放物運動で一瞬速さが0になる場所)を
  //     見ているだけ。
  //   - Rust側の受け入れテストが直接保証しているのは「1秒(2000ステップ)の
  //     観測窓では、どの粒も床(y<-0.04)を突き抜けない」ことだけで、横や上に
  //     器の外へ出ることは禁じていない(むしろ許容している——器の外へ出た粒が
  //     その後長い時間をかけてどこへ行くかはこのテストの範囲外)。
  //
  // そこでこのE2Eも、Rust側が実際に保証している性質と同じもの
  // ——「観測窓の中では床を抜けない」「粒は静止画ではなく動いている」
  // ——だけを検証する。「容器の外に出ない」という、実測で偽だと分かった主張は
  // 書かない。
  const { minYBefore, minYAfter, maxDelta } = await page.evaluate(() => {
    const world = (
      window as unknown as {
        __world: {
          step(): void;
          fluid_particle_positions_f32(): Float32Array;
        };
      }
    ).__world;
    const before = world.fluid_particle_positions_f32().slice();
    let minYBefore = Infinity;
    for (let i = 0; i < before.length / 3; i += 1) {
      minYBefore = Math.min(minYBefore, before[i * 3 + 1]);
    }
    // Rustの受け入れテスト(`scenario.rs`)と同じ観測窓: 2000ステップ。
    for (let s = 0; s < 2000; s += 1) world.step();
    const after = world.fluid_particle_positions_f32();
    let minYAfter = Infinity;
    let maxDelta = 0;
    for (let i = 0; i < after.length; i += 1) {
      maxDelta = Math.max(maxDelta, Math.abs(after[i] - before[i]));
      if (i % 3 === 1) minYAfter = Math.min(minYAfter, after[i]);
    }
    return { minYBefore, minYAfter, maxDelta };
  });
  // Rustの受け入れ基準と同じ: 境界の床(y=-0.04)を突き抜けた粒はいない。
  expect(minYAfter).toBeGreaterThan(-0.04);
  // 静止画ではなく、実際に(器の高さ0.34mを超えるくらい大きく)動いている。
  expect(maxDelta).toBeGreaterThan(0.3);
  expect(minYBefore).toBeLessThan(Infinity);
  expect(errors).toEqual([]);
});

test("水を注ぐ実験は、実際の遅さを隠さず見せる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d23-pouring-water"]');

  // 「速さ ×1」は人が選んだ相対倍率でしかなく、実際にどれだけ実時間より
  // 遅く進んでいるかは計算の重さ次第——それを隠さず出す(進行管理役③の
  // 実測: 「ふつう」「速さ ×1」としか出ていないのに、実時間20.9秒でも
  // シミュレーション内は1.25秒(=0.06倍)しか進まなかった)。
  const rate = page.locator("#run-actual-rate");
  await expect(rate).toBeVisible({ timeout: 10_000 });
  await expect
    .poll(async () => await rate.textContent(), { timeout: 15_000 })
    .toMatch(/実際は ×0\.\d+/);
  await expect(rate).toHaveClass(/slow/);
  // この注記のぶんツールバーが混み合っても、パンくずの実験名が省略されて
  // 消えない(横幅を奪い合って「水を注」まで切れていたのを直した)。
  await expect(page.locator("#crumb-experiment")).toContainText("水を注ぐ");
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

test("真空にしても、落ちる速さが数値で出る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d7-terminal"]');
  await page.waitForTimeout(1500);
  await page.click("#knob-air button:has-text('真空')");

  // 密度 0 では Re=0 → Cd=24/Re=∞ となり、抗力が NaN になって速さが数値で
  // なくなっていた(利用者役③)。真空なら抗力ゼロ、つまり素の自由落下。
  const speedNode = page.locator('#context dd[data-probe="0"]');
  await expect
    .poll(
      async () => Number.parseFloat((await speedNode.textContent()) ?? "NaN"),
      { timeout: 20_000 },
    )
    .toBeGreaterThan(10);

  // 自由落下 v = g t と噛み合う。
  const seconds = await elapsedSeconds(page);
  const speed = Number.parseFloat((await speedNode.textContent()) ?? "NaN");
  expect(speed).toBeGreaterThan(9.0 * seconds - 3);
  expect(speed).toBeLessThan(9.81 * seconds + 3);
  expect(errors).toEqual([]);
});

test("3D の煙が、舞台に描かれる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d14c-smoke-3d"]');

  // 煙は格子の中の数値としてしか存在せず、舞台は最初から最後まで真っ暗だった
  // ——「まん中の 3D を見てください」と案内している隣で何も映らなかった
  // (利用者役③)。
  await expect(page.locator("#scene-view")).toHaveAttribute(
    "data-smoke",
    "true",
    { timeout: 20_000 },
  );
  expect(errors).toEqual([]);
});

test("桁の離れた値が、0 に潰れない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d31-diffusion-ink"]');

  // 実際は 1.5e-22 → 1.9e-17 と 5 桁動いているのに、パネルは「0.0000」の
  // まま止まって見えた(利用者役③)。
  await expect
    .poll(
      async () =>
        (await page.locator('#context dd[data-probe="0"]').textContent()) ?? "",
      { timeout: 20_000 },
    )
    .toMatch(/e[+-]?\d/);
  expect(errors).toEqual([]);
});

test("グラフの凡例の数値が、右の「いまの数値」と同じ書式のルールで書かれる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d16-conduction-race"]');
  await page.locator('#knob-material .knob-choice-btn', { hasText: "木" }).click();

  // --- このテストが前回落ちた理由と、書き直した理由 ---
  // 元のテストは、右の「いまの数値」パネルのテキストと、グラフ凡例の
  // `max=`(=これまでの最大値)のテキストを**文字どおり同じであること**で
  // 検査していた。だがこの2つはそもそも別の量: パネルは「いまこの瞬間の
  // 値」、凡例の `max=` は「これまでに観測した最大値」で、熱が棒を伝わって
  // いく途中は一致するとは限らない。加えてパネル側だけに、`renderContext`の
  // `negligible`(その系列がこれまでに取った大きさに比べて無視できるほど
  // 小さいときだけ 0 と書く)という、凡例には無い丸めが入っている。
  // 実機でも「パネルは 0.0 ℃・凡例は max=1.24e-17 ℃」のような食い違いが
  // 普通に起きる(進行管理役の裏取り)。遅い macOS CI では、たまたまこの
  // 「パネル側だけ0に丸まった直後」を捉えて `147 passed / 1 failed` に
  // なっていた——製品の不具合ではなく、比べてはいけないものを比べていた。
  //
  // 本当に確かめたいのは値の一致ではなく**書式(桁の選び方・指数か固定
  // 小数か)が同じルールで決まっていること**。そこで、凡例が実際に使った
  // 「生の値」と「桁数」をテスト専用に露出させ(`__probeGraphLegendRaw`)、
  // パネルが使っているのと**同じ`readoutNumber`関数**(複製ではなく本物、
  // `__readoutNumberForTest`)にその生の値を通した結果と、凡例の描画済み
  // 文字列を比べる。同じ値どうしを比べるので、時間経過によるズレは起きない。

  // 「熱源から 0.25 m」は届いた温度が桁として最も大きく、指数表記から固定
  // 小数へ切り替わる境目(`readoutNumber` のdoc参照)をいちばん早く通る。
  const near = page.locator('#context dd[data-probe="1"]');
  // この瞬間の生の最大値がちょうど指数表記の境目付近にあるはずで、これが
  // まさに退行(生の指数がずれて出る)が起きた領域。ここで止めて凡例を読む
  // ——パネル側の読み取りは、もう検査には使わない(冒頭のコメント参照)。
  await expect
    .poll(async () => (await near.textContent()) ?? "", { timeout: 60_000 })
    .toMatch(/e[+-]?\d/);
  await page.click("#btn-run");
  await page.waitForTimeout(200);

  // canvas に直接ラスタライズされる凡例の文字はDOMから読めない
  // (`smoke.spec.ts` の「canvas の中身は直接検証できない」注記と同じ理由)ので、
  // テスト専用に露出した `window.__probeGraphLegend`/`__probeGraphLegendRaw`
  // (`__camera`/`__world`と同じ扱い)を読む。
  const { legendLines, legendRaw } = await page.evaluate(() => {
    const w = window as unknown as {
      __probeGraphLegend?: string[];
      __probeGraphLegendRaw?: {
        label: string;
        unit?: string;
        digits?: number;
        max: number;
        min: number;
      }[];
    };
    return {
      legendLines: w.__probeGraphLegend ?? [],
      legendRaw: w.__probeGraphLegendRaw ?? [],
    };
  });
  expect(legendLines.length).toBe(3);
  expect(legendRaw.length).toBe(3);

  // 凡例の `min=` は、生の指数(`0.0e+0`)ではなく、パネルと同じ「0.0」の
  // 書き方であること(木の実験は0.25m以外どこもまだ届いていないので、遠い
  // 2本の min は必ず、始まりのまま=0 のはず)。
  for (const line of legendLines) {
    expect(line).not.toMatch(/e\+0/);
  }

  // 本体: 元の不具合は「凡例が `readoutNumber` を使わず、桁数を知らない
  // 独自の書式(生の指数 `9.3e-67` 等)で書いていた」こと(`legendNumber`の
  // doc参照)。ここでは、凡例が実際に描いた文字列を、**同じ生の値**を
  // パネルと同じ`readoutNumber`関数に通した結果と比べることで、それが
  // 直っていることを検査する——値の一致ではなく、書式のルールの一致。
  const readoutNumberInPage = async (value: number, digits: number) =>
    page.evaluate(
      ([v, d]) =>
        (
          window as unknown as {
            __readoutNumberForTest?: (value: number, digits: number) => string;
          }
        ).__readoutNumberForTest?.(v, d) ?? "",
      [value, digits] as const,
    );

  const near25 = legendRaw.find((s) => s.label.includes("0.25 m"));
  expect(near25).toBeDefined();
  expect(near25?.digits).toBeDefined();
  const expectedMaxText = await readoutNumberInPage(near25!.max, near25!.digits!);
  const expectedMinText = await readoutNumberInPage(near25!.min, near25!.digits!);
  expect(expectedMaxText).toBeTruthy();

  const nearLine = legendLines.find((l) => l.includes("0.25 m"));
  expect(nearLine).toBeDefined();
  expect(nearLine).toContain(`max=${expectedMaxText} ℃`);
  expect(nearLine).toContain(`min=${expectedMinText} ℃`);

  // 退行時の実測を再現しないことも確かめる: 凡例の `max=` が生の指数
  // (`toExponential`のデフォルト書式や、桁数を無視した `formatTickValue`)
  // に戻っていないこと。`readoutNumber`が返す指数は必ず小数点以下2桁
  // (`toExponential(2)`)なので、それ以外の指数書式(桁数違い)は退行の
  // 兆候になる。
  const maxMatch = nearLine?.match(/max=(-?\d(?:\.\d+)?e[+-]?\d+)/);
  if (maxMatch) {
    expect(maxMatch[1]).toMatch(/^-?\d\.\d{2}e[+-]?\d+$/);
  }
  expect(errors).toEqual([]);
});

test("床より下へ落ちていく物も、画面から見失わない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d7-terminal"]');
  await page.waitForTimeout(4000);

  // 床の下へ潜らないための高さの下限を無条件に当てていたので、床の無い場面
  // (5 mm の球が y=0 から落ち続ける)では対象が -170 m まで沈んでもカメラ
  // だけが y≒0 に残り、200 m 以上離れた球を見ることになって画面が真っ黒
  // だった。潜り込む床がそもそも無い場面では当てない。
  const framed = await page.evaluate(() => {
    const canvas = document.querySelector<HTMLCanvasElement>("#scene-view canvas");
    return canvas ? canvas.width > 0 : false;
  });
  expect(framed).toBe(true);

  // 画面のまん中あたりに、背景より明るいものが映っている。
  const shot = await page
    .locator("#scene-view canvas")
    .first()
    .screenshot({ scale: "css" });
  // PNG が単色なら、ほぼ圧縮しきられて極端に小さくなる。
  expect(shot.byteLength).toBeGreaterThan(4500);
  expect(errors).toEqual([]);
});

test("向きも、数値で決められる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());

  // 向きは輪をドラッグするしか手が無く、掴む場所がわずかに違うだけでどの軸が
  // 回るか変わるので、狙った角度の坂を作れなかった(利用者役④)。
  const z = page.locator("#focus-rot-z");
  await expect(z).toBeVisible();
  await z.fill("30");
  await z.dispatchEvent("change");
  await expect
    .poll(async () => Number(await z.inputValue()), { timeout: 10_000 })
    .toBeCloseTo(30, 0);
  expect(errors).toEqual([]);
});

test("「＋新規シーン」の直後は、床が「選んだもの」として出ない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");

  // エディタ側(`main.ts`)は内部都合で先頭のボディ(床)を選んだ状態のまま
  // 「場面が差し替わった」と知らせてくる。以前はワークスペース側がそれを
  // そのまま受け取っていたため、「＋新規シーン」の直後に一瞬だけ「選んだもの
  // — ground」の札(置き場所・向きの入力欄つき)が実在した。遅い機械では、
  // 続けて物を1つ置いたときの選択の切り替わり(床→置いた物)がこの後に
  // 来るまでの間にちょうど入力が割り込むと、床の欄へ打った値が「本物の
  // 選択変更」として正しく捨てられてしまい、置いた物には既定値(0)が
  // 送られていた(「向きも、数値で決められる」がmacOS CIだけで
  // 「入力欄が0のまま」と落ちた原因)。「選んだもの」札は**人が対象を
  // クリックしたときだけ**出るべきで、床が一瞬でも出てはいけない。
  await expect(page.locator('.card[data-card="focus"]')).toHaveCount(0);

  // 続けて物を置いたときは、その物(床ではない)が選ばれて出る——選択の
  // 切り替わりが「なし→置いた物」の1回だけで済み、床を経由しない。
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());
  const focus = page.locator('.card[data-card="focus"]');
  await expect(focus).toBeVisible();
  await expect(focus).toContainText("箱 1");
  await expect(focus).not.toContainText("ground");
  expect(errors).toEqual([]);
});

test("2つ目に置いた物も、グラフに記録できる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());
  await page.waitForTimeout(600);
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());

  // 記録が付くのは最初に置いた物だけで、2 つ目以降を比べたくても足す手段が
  // どこにも無かった(利用者役④)。
  const record = page.locator("#btn-record-body");
  await expect(record).toBeVisible();
  await record.click();
  await expect(page.locator("#hierarchy-tree")).toContainText("高さ(球 2)");
  // 付いたら、そのボタンはもう出ない。
  await expect(page.locator("#btn-record-body")).toHaveCount(0);
  expect(errors).toEqual([]);
});

test("モーターは、置いて動かせば回る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-motor")!.click());

  // 目標角度を腕の初期姿勢のまま置いていたので、足して「うごかす」を押しても
  // 微動だにせず、ツールバーの「モーター切替」を見つけるまで壊れているように
  // しか見えなかった(利用者役④)。
  const z = page.locator("#focus-rot-z");
  await expect(z).toBeVisible();
  await page.click("#btn-run");
  await expect
    .poll(async () => Math.abs(Number(await z.inputValue())), { timeout: 20_000 })
    .toBeGreaterThan(30);
  expect(errors).toEqual([]);
});

test("気体の実験には、箱の枠が描かれる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d30-gas-box"]');

  // 「400 個の分子が箱の中で飛び回ります」と書いてあるのに、枠も壁も無く、
  // 点が真っ黒な空間に浮いているだけに見えた(利用者役①)。
  await expect(page.locator("#scene-view")).toHaveAttribute(
    "data-gas-box",
    "true",
    { timeout: 20_000 },
  );
  expect(errors).toEqual([]);
});

test("遠くを回っている物を「もう在りません」と言わない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 0);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d34-solar-system"]');

  // 退避した剛体の目印を「桁が大きい値」で見ていたので、太陽から 1.5e11 m を
  // 回っている惑星の距離まで「もう在りません」と書き、目の前を回っている物を
  // 指して消えたと言う画面になっていた(利用者役①)。
  const distance = page.locator('#context dd[data-probe="0"]');
  await expect
    .poll(async () => (await distance.textContent()) ?? "", { timeout: 15_000 })
    .toMatch(/[0-9]/);
  await expect(distance).not.toContainText("もう在りません");
  expect(errors).toEqual([]);
});

test("止まりかけた値は、指数ではなく 0 と書く", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 1);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d10-brake-heat"]');
  await page.waitForTimeout(1000);
  await page.evaluate(() => {
    const r = document.querySelector<HTMLInputElement>(
      '.knob input[type="range"]',
    );
    if (!r) return;
    r.value = r.max;
    r.dispatchEvent(new Event("input", { bubbles: true }));
    r.dispatchEvent(new Event("change", { bubbles: true }));
  });

  // 桁の離れた量を指数で書くようにしたら、こんどは止まりかけた箱の速さが
  // 「8.67e-19 m/s」と出るようになった(利用者役②)。その量がこれまでに
  // 取った大きさと比べて無視できるなら 0 と書く。
  const speed = page.locator('#context dd[data-probe="0"]');
  await expect
    .poll(async () => (await speed.textContent()) ?? "", { timeout: 25_000 })
    .toMatch(/^0\.00 m\/s$/);
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

test("大きさの表示が、札と Inspector で食い違わない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");

  const shape = page.locator('[data-focus="かたち"]');
  const inspector = page.locator("#inspector-body");

  // **球**: 「選んだもの」札は既定スポーン半径0.4mを「直径 0.80 m」の
  // 人の言葉で書く。Inspectorは同じ球の半径をwasmの生の値のまま出すが
  // (`body_shape_label_at`は半径を返す)、以前は「Sphere(0.4000)」と単位も
  // 「半径」の断りも無く出ていたため、札の「直径0.80m」と数字だけ見比べると
  // 半分にずれて見えた(利用者役の報告)。半径だと分かるラベルが付き、
  // 0.4000 × 2 = 0.80 と暗算できることを確かめる。
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await expect(shape).toContainText("直径 0.80 m");
  await expect(inspector).toContainText("Sphere(半径 0.4000 m)");

  // **箱**: 札は一辺の長さ(半辺の2倍)。Inspectorは半辺をそのまま返すので
  // 「半辺」と書いて区別する。
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());
  await expect(shape).toContainText("0.80 × 0.80 × 0.80 m");
  await expect(inspector).toContainText("Box(半辺 x=0.4000, y=0.4000, z=0.4000 m)");

  // **カプセル**: wasm側に専用の整形が無く(`Shape::Capsule`は
  // `body_shape_label_at_impl`で`{other:?}`のRust Debug文字列
  // `Capsule { radius: 0.2, half_height: 0.35 }`に落ちる)、以前は札側の
  // 整形(`friendlyShape`)がこの`{`区切りの形を`(`区切り前提で読み損ね、
  // 生のRust Debug文字列がそのまま「選んだもの」札にも出ていた
  // (Inspectorだけでなく人向けの札まで壊れていた、より重い食い違い)。
  // 札は「太さ・長さ」の人の言葉、Inspectorは「半径・半分の高さ」の
  // ラベル付き生値になることを確かめる。
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-capsule")!.click());
  await expect(shape).toContainText("太さ 0.40 m・長さ 0.70 m");
  await expect(shape).not.toContainText("radius:");
  await expect(inspector).toContainText("Capsule(半径 0.2 m, 半分の高さ 0.35 m)");

  expect(errors).toEqual([]);
});

test("「うごかす」を押すと、自分で置いた球が落下から着地まで画面に映り続ける", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(300);

  // 置いた直後(高さ12m)は`frameCameraOnContent`が至近距離まで寄せてくれる
  // ので映っている。問題はここから——「うごかす」を押した瞬間、以前は
  // 追従カメラ(組み立て中は`followCamera(false)`のまま、`playButton`の
  // クリックハンドラのdoc参照)が誰も起きないまま置き去りにされ、球は
  // 落ち始めた次のフレームで画角の外へ出て、着地はおろか落下そのものが
  // 一度も見えなかった(進行管理役の実測: 高さ12.0m→10.16mの間にndc.yが
  // 0→-1.61まで飛び出す)。
  await expect.poll(() => bodyOnScreen(page), { timeout: 5_000 }).toBe(true);
  await page.click("#btn-run");

  // 落ち始め・着地までの複数時点で映り続けること。
  for (const waitMs of [200, 300, 300, 300, 300, 500]) {
    await page.waitForTimeout(waitMs);
    expect(await bodyOnScreen(page)).toBe(true);
  }
  expect(errors).toEqual([]);
});

test("一時停止中に自分でカメラを動かしても、「うごかす」で再開した瞬間に戻されない", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(300);
  await page.click("#btn-run"); // 走らせる(追従カメラが起きる)。
  await page.waitForTimeout(400);
  await page.click("#btn-run"); // 一時停止。

  // 一時停止中に自分でカメラを操作する(=追従カメラを自分の意思で止める、
  // `orbit.addEventListener("start", ...)`の既存の仕組み)。左ボタンは選択・
  // ギズモ操作に割り当て済みで OrbitControls には繋がっていないため
  // (`orbit.mouseButtons`のdoc参照)、実際に視点operationに使う中ボタンで
  // 回転ドラッグする(`camera-gizmo-interaction.spec.ts`と同じ流儀)。
  const box = await page.locator("#scene-view-canvas-host").boundingBox();
  if (!box) throw new Error("scene-view-canvas-host not found");
  const cx = box.x + box.width / 2;
  const cy = box.y + box.height / 2;
  await page.mouse.move(cx, cy);
  await page.mouse.down({ button: "middle" });
  await page.mouse.move(cx + 150, cy - 60, { steps: 12 });
  await page.mouse.up({ button: "middle" });
  // `orbit.enableDamping`(慣性)がドラッグ後もしばらくカメラを動かし続ける
  // ので、その減衰が収まるまで待ってから基準位置を取る——ここで待たずに
  // 比べると、慣性による動きを「追従が起きてしまった」と誤検出する。
  await page.waitForTimeout(1500);
  const camAfterOrbit = await page.evaluate(() => {
    const cam = (window as unknown as { __camera: { position: { x: number; y: number; z: number } } }).__camera;
    return { x: cam.position.x, y: cam.position.y, z: cam.position.z };
  });

  // 再開しても、自分で動かしたカメラの位置がその場で覆されないこと
  // (`isEditing()`がmode==="play"のままなので偽になり、追従が再度起きない)。
  await page.click("#btn-run");
  await page.waitForTimeout(50);
  const camAfterResume = await page.evaluate(() => {
    const cam = (window as unknown as { __camera: { position: { x: number; y: number; z: number } } }).__camera;
    return { x: cam.position.x, y: cam.position.y, z: cam.position.z };
  });
  const moved = Math.hypot(
    camAfterResume.x - camAfterOrbit.x,
    camAfterResume.y - camAfterOrbit.y,
    camAfterResume.z - camAfterOrbit.z,
  );
  expect(moved).toBeLessThan(0.05);
  expect(errors).toEqual([]);
});

test("消した物の観測点は、左の「記録している値」に残っても消えた物だと分かる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(300);

  const probes = page.locator("#hierarchy-tree");
  await expect(probes).toContainText("高さ(球 1)");
  await expect(probes).not.toContainText("消えた物");

  // 3D と一覧からは消える一方、グラフに描いた過去データを黙って捨てるのは
  // 乱暴なので、「記録している値」には残す——ただし**もう無い物だと分かる**
  // ように注記する(課題B、`friendlyProbeLabel`のdoc参照)。
  await page.locator("#hierarchy-tree li", { hasText: "球 1" }).first().click();
  await page.keyboard.press("Delete");
  await page.waitForTimeout(300);

  await expect(probes).toContainText("高さ(球 1・消えた物)");
  await expect(probes).toContainText("速さ(球 1・消えた物)");
  // 3D・一覧からは実際に消えていること(一覧の食い違いそのものは直っている)。
  await expect(page.locator("#hierarchy-tree")).not.toContainText("↳ 球 1");
  expect(errors).toEqual([]);
});

// 課題A(進行管理役の裏取り済み): 「坂を作るなら20〜40度に」という案内どおりに
// やっても坂にならなかった(床は向きを変えても傾かない・箱は動くままだと転がって
// 平らに戻る)。案内を実態に合わせ、実際にその手順で坂ができることを確かめる。
test("床(ゆか)を選ぶと、向きはここでは変えられないと正直に書かれている", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.locator("#hierarchy-tree .tree-body").first().click();

  const note = page.locator("#focus-rotation-note");
  await expect(note).toContainText("ここ(ゆか)の向きはここでは変えられません");

  // 実際に打っても傾かないことも確かめる(利用者役の報告どおり、進行管理役の
  // 裏取りでも`Plane(normal=(0,1,0), d=0)`のままだった)。
  const rotX = page.locator("#focus-rot-x");
  await rotX.fill("30");
  await rotX.dispatchEvent("change");
  await page.waitForTimeout(500);
  await expect(page.locator("#inspector-body")).toContainText("Plane(normal=(0,1,0), d=0)");
  expect(errors).toEqual([]);
});

test("案内どおりに箱を置いて傾け、動かないようにすると、実際に坂になる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());

  const y = page.locator("#focus-pos-y");
  await y.fill("0.5");
  await y.dispatchEvent("change");
  const rotX = page.locator("#focus-rot-x");
  await rotX.fill("30");
  await rotX.dispatchEvent("change");

  // 案内が「動かない(Static)」にする所まで届いている(課題B の言葉と揃えて
  // いること自体も、この文言で確かめる)。
  const note = page.locator("#focus-rotation-note");
  await expect(note).toContainText("動かない(Static)");

  await page.selectOption("#inspector-body-type", "Static");
  await expect(page.locator("#inspector-body-type")).toHaveValue("Static");

  await page.click("#btn-run");
  await page.waitForTimeout(3000);

  // 動くままだと重力で転がって平らに戻る(直す前の実際の症状)が、
  // Static にしたので 30 度のまま——3手(置く→傾ける→動かなくする)で
  // 坂が組み上がる。
  await expect
    .poll(async () => Number(await rotX.inputValue()), { timeout: 10_000 })
    .toBeCloseTo(30, 0);
  expect(errors).toEqual([]);
});

// 課題B: プログラムの言葉(Dynamic/Static/Kinematic・DistanceJoint等)が
// 説明なしにそのまま画面へ出ていた。人の言葉を主に、元の語を括弧へ落とす。
test("動き方の選択肢とバッジ、「＋追加」メニューが人の言葉で読める", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());

  // 動き方(Body type): 主が人の言葉、従が元の語。`value`(保存データ・
  // 他のテストが参照する)は元の英単語のまま変えていない。
  const bodyTypeTexts = await page.locator("#inspector-body-type option").allTextContents();
  expect(bodyTypeTexts).toEqual([
    "動く(Dynamic)",
    "動かない(Static)",
    "決めたとおりに動く(Kinematic)",
  ]);
  await page.selectOption("#inspector-body-type", "Static");
  await expect(page.locator("#inspector-body-type")).toHaveValue("Static");

  // INSPECTOR の名前横のバッジも同じ言葉。バッジは選び直した瞬間ではなく
  // 選択の再描画で出るので、既定でStaticな床を選び直して確かめる。
  await page.locator("#hierarchy-tree .tree-body").first().click();
  await expect(page.locator("#inspector-body .badge")).toHaveText("動かない(Static)");

  // 「＋追加」メニュー: DistanceJoint 等の型名は括弧の中だけ、主たる名前は日本語。
  await page.click("#btn-add");
  const menuTexts = await page.locator("#context-menu button").allTextContents();
  expect(menuTexts).toContain("＋ 振り子 (DistanceJoint)");
  expect(menuTexts).toContain("＋ モーター (角度を指定して止まる。回り続けません)");
  expect(menuTexts).toContain("＋ 流体 (SPH 水塊)");
  await page.keyboard.press("Escape");

  // ↑ Nudge ボタンも人の言葉が主になり、内部の仕組み(Command経由)は
  // 利用者向けの説明から落ちている。
  await expect(page.locator("#btn-nudge")).toContainText("押し上げる");
  const nudgeTitle = await page.locator("#btn-nudge").getAttribute("title");
  expect(nudgeTitle ?? "").not.toContain("Command");
  expect(errors).toEqual([]);
});

// 課題C: 「Undo」は位置/向き/大きさしか戻せないのに「Undo」とだけ書かれ、
// 何でも戻せると期待させていた。また、消す手段がDeleteキーだけで画面に
// 書かれていなかった(利用者役は自動テストで偶然見つけた)。
test("「Undo」は、できること(動かしたのを戻す)に合わせた名前になっている", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await page.evaluate(() => document.getElementById("btn-spawn-box")!.click());

  await expect(page.locator("#btn-undo")).toContainText("動かしたのを戻す");
  await expect(page.locator("#btn-redo")).toContainText("戻したのをやり直す");
  expect(errors).toEqual([]);
});

/**
 * ロケータの `.click()` は要素を自前でスクロールして見える位置へ持って
 * いってから押すため、「ボタンは在るのに画面上は別パネルに隠れて実マウスで
 * 押せない」という壊れ方(課題A、進行管理役の実測)があっても素通りして
 * しまう。ここでは本物のマウス操作に近い形で、
 *   1. 要素の中心座標で `elementFromPoint` を引き、実際に**その要素自身が
 *      画面上に見えている**ことを確かめてから
 *   2. その座標へ低レベル `page.mouse` で押す
 * ことで、見た目には存在するのに他パネルに隠れて反応しない壊れ方を
 * 実際に検出できるようにする。
 */
async function realClickVerifyingVisible(page: Page, locator: Locator) {
  const box = await locator.boundingBox();
  if (!box) throw new Error("要素の位置が取れない(非表示?)");
  const x = box.x + box.width / 2;
  const y = box.y + box.height / 2;
  const atPoint = await page.evaluate(
    ({ x, y, id }) => document.elementFromPoint(x, y)?.id === id,
    { x, y, id: await locator.evaluate((el) => el.id) },
  );
  expect(
    atPoint,
    "ボタンの座標を実マウスで押しても、画面上ではその場所に別の要素が" +
      "描かれていて届かない(他パネルに隠れている可能性)",
  ).toBe(true);
  await page.mouse.move(x, y);
  await page.mouse.down();
  await page.mouse.up();
}

test("置いた物は、選んだ札の「これを消す」から見つけて消せる(押し間違い対策つき)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 球");
  const sphereRow = page.locator("#hierarchy-tree .tree-body", { hasText: "球 1" });
  await expect(sphereRow).toHaveCount(1);

  const removeBtn = page.locator("#btn-remove-body");
  await expect(removeBtn).toBeVisible();
  await expect(removeBtn).toContainText("これを消す");

  // **実マウスの座標で**1回押す。1回押しただけでは消えない
  // (押し間違い対策——確認してから消す)。
  await realClickVerifyingVisible(page, removeBtn);
  await expect(sphereRow).toHaveCount(1);
  await expect(removeBtn).toContainText("本当に消しますか");

  // 確認文言は元のラベルよりずっと長い。ここで座標を取り直さず
  // **前回と同じボタンが同じ場所にまだあること**を確かめてから続けて押す
  // ——ラベルが伸びて別のボタンを押し場所ごと動かしていたら、ここで
  // 「別の要素に隠れている」として落ちる(課題Aの回帰そのものの形)。
  await realClickVerifyingVisible(page, removeBtn);
  // 続けてもう一度押すと、実際に消える(「物」の一覧から消える。記録済みの
  // グラフは、消えた物だと分かる注記つきで残る——別のテストが確かめる対象)。
  await expect(sphereRow).toHaveCount(0);
  // 消した直後は選択も外れる(床が代わりに選ばれたままにはしない)。
  await expect(page.locator('.card[data-card="focus"]')).toHaveCount(0);

  // 床(ゆか)は場面の基準面なので、消す対象には出ない。
  await page.locator("#hierarchy-tree .tree-body").first().click();
  await expect(page.locator("#btn-remove-body")).toHaveCount(0);
  expect(errors).toEqual([]);
});

// **課題B**: 保存していない作りかけを、確認なく黙って捨てる導線が無いか。
//
// 「新規シーン」・Toolbarのシーン選択・⌘Kでの実験選び直し・保存済み場面を
// 開く、はどれも「いま見ている場面をその場で丸ごと差し替える」処理を経由
// する(`workspace.ts`の`confirmDiscardIfNeeded`のdoc参照)。**保存していない
// 自分の作りかけがあるときだけ**確認し(実験を選んだだけ・まっさらな状態
// では聞かない)、キャンセルすれば実際には何も変わらないことを確かめる。
test("「新規シーン」は、作りかけが無ければ確認なしに進む", async ({ page }) => {
  const errors = collectPageErrors(page);
  const dialogs: string[] = [];
  page.on("dialog", (d) => {
    dialogs.push(d.message());
    void d.dismiss();
  });
  await boot(page);
  await setGrain(page, 3);
  // 起動直後(実験を選んだだけ・何も編集していない)は「作りかけ」ではない。
  await page.click("#btn-new-scene");
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(1); // 床のみ
  expect(dialogs).toEqual([]);

  // 「新規シーン」を押した直後(まだ何も置いていない、まっさらな状態)も
  // 同様に確認なしで進む。
  await page.click("#btn-new-scene");
  expect(dialogs).toEqual([]);
  expect(errors).toEqual([]);
});

test("「新規シーン」は、置いた物があれば確認し、キャンセルすれば何も消えない", async ({ page }) => {
  const errors = collectPageErrors(page);
  const dialogs: string[] = [];
  page.on("dialog", (d) => {
    dialogs.push(d.message());
    void d.dismiss(); // キャンセル
  });
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 箱");
  const row = page.locator("#hierarchy-tree .tree-body", { hasText: "箱 1" });
  await expect(row).toHaveCount(1);

  await page.click("#btn-new-scene");
  expect(dialogs.length).toBe(1);
  expect(dialogs[0]).toContain("消えます");
  // キャンセルしたので、置いた箱はそのまま残っている。
  await expect(row).toHaveCount(1);
  expect(errors).toEqual([]);
});

test("「新規シーン」は、置いた物があっても確認してOKすれば実際に空になる", async ({ page }) => {
  const errors = collectPageErrors(page);
  let dialogCount = 0;
  page.on("dialog", (d) => {
    dialogCount++;
    void d.accept();
  });
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 箱");
  const row = page.locator("#hierarchy-tree .tree-body", { hasText: "箱 1" });
  await expect(row).toHaveCount(1);

  await page.click("#btn-new-scene");
  expect(dialogCount).toBe(1);
  // OKしたので、実際に空(床のみ)へ差し替わっている。
  await expect(row).toHaveCount(0);
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(1);
  expect(errors).toEqual([]);
});

test("⌘Kで別の実験を選び直すときも、作りかけがあれば確認し、キャンセルすれば場面もパンくずも食い違わない", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  let dialogCount = 0;
  page.on("dialog", (d) => {
    dialogCount++;
    void d.dismiss(); // キャンセル
  });
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 箱");
  const row = page.locator("#hierarchy-tree .tree-body", { hasText: "箱 1" });
  await expect(row).toHaveCount(1);

  await page.keyboard.press("Control+k");
  await expect(page.locator("#palette")).toBeVisible();
  await page.locator("#palette-results button").first().click();

  expect(dialogCount).toBe(1);
  // キャンセルしたので、置いた箱も「じぶんの場面」の表示も両方そのまま
  // ——パンくずだけ新しい実験名に化けて、実物(Hierarchy)は前のまま、
  // という食い違いが起きていないことを確かめる。
  await expect(row).toHaveCount(1);
  await expect(page.locator("#crumb-own-scene")).toHaveCount(1);
  await expect(page.locator("#crumb-experiment")).toHaveCount(0);
  expect(errors).toEqual([]);
});

test("保存済みの場面を開くときも、作りかけがあれば確認する", async ({ page }) => {
  const errors = collectPageErrors(page);
  const dialogs: string[] = [];
  page.on("dialog", (d) => {
    dialogs.push(d.message());
    void d.dismiss();
  });
  await boot(page);
  await setGrain(page, 3);

  // まず1つ場面を保存しておく。
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 球");
  await page.fill("#input-scene-name", "テスト保存場面A");
  await page.click("#btn-save-scene");
  await expect(page.locator("#crumb-own-scene")).toContainText("テスト保存場面A");

  // 新規シーンへ切り替え、別の物を置く(=直近の保存より後の、未保存の作りかけ)。
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 箱");
  const boxRow = page.locator("#hierarchy-tree .tree-body", { hasText: "箱 1" });
  await expect(boxRow).toHaveCount(1);

  // ⌘Kから、さっき保存した場面を開こうとする。
  await page.keyboard.press("Control+k");
  await page.fill("#palette-input", "テスト保存場面A");
  await expect(page.locator(".palette-row").first()).toContainText("テスト保存場面A");
  await page.keyboard.press("Enter");

  expect(dialogs.length).toBe(1);
  // キャンセルしたので、箱を置いた今の場面のまま(保存済み場面には切り替わらない)。
  await expect(boxRow).toHaveCount(1);
  await expect(page.locator("#crumb-own-scene")).not.toContainText("テスト保存場面A");
  expect(errors).toEqual([]);
});

test("Toolbarのシーン選択も、作りかけがあれば確認し、キャンセルすれば切り替わらない", async ({ page }) => {
  const errors = collectPageErrors(page);
  let dialogCount = 0;
  page.on("dialog", (d) => {
    dialogCount++;
    void d.dismiss();
  });
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 箱");
  const row = page.locator("#hierarchy-tree .tree-body", { hasText: "箱 1" });
  await expect(row).toHaveCount(1);

  await page.selectOption("#select-scene", { index: 1 });
  expect(dialogCount).toBe(1);
  // キャンセルしたので、置いた箱はそのまま。ドロップダウンの選択も戻る。
  await expect(row).toHaveCount(1);
  await expect(page.locator("#select-scene")).toHaveValue("");
  expect(errors).toEqual([]);
});

test("Projectドロワーの「シーン」タブから読み込むときも、作りかけがあれば確認する", async ({ page }) => {
  const errors = collectPageErrors(page);
  let dialogCount = 0;
  page.on("dialog", (d) => {
    dialogCount++;
    void d.dismiss();
  });
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "＋ 箱");
  const row = page.locator("#hierarchy-tree .tree-body", { hasText: "箱 1" });
  await expect(row).toHaveCount(1);

  await page.click('.project-tab[data-tab="scenes"]');
  await page.click('.scene-gallery-list button[data-scene-file="d4-box-stack.json"]');

  expect(dialogCount).toBe(1);
  // キャンセルしたので、D4(4体)には差し替わらず、箱を置いた今の場面のまま。
  await expect(row).toHaveCount(1);
  expect(errors).toEqual([]);
});

// **退行(進行管理役の実測)**: `51dca5c` で「保存していない作りかけを捨てる
// 前に確認する」を入れた範囲が広すぎ、「はじめから」(このアプリ自身の
// 「やり直す」ボタンで、別の場面へは移らない)まで、材質を変えた直後だけ
// 「保存していない作りかけがあります。このまま進めると消えます(元には
// 戻せません)。」という強い確認を出してしまっていた。Playwrightは
// `page.on("dialog", ...)` を登録しないと未処理のダイアログを自動で閉じる
// ため、登録しない実測(利用者役が「アプリが固まった」と読んだのと同じ状況)
// と、登録した実測の両方で確かめる。
test("「はじめから」は、材質を変えたあとでも確認を出さない(退行)", async ({ page }) => {
  const errors = collectPageErrors(page);
  const dialogs: string[] = [];
  page.on("dialog", (d) => {
    dialogs.push(d.message());
    void d.accept();
  });
  await boot(page);
  await setGrain(page, 2); // しらべる(進行管理役の実測と同じ粒度)
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d1-free-fall"]');
  await page.locator("#crumb-experiment").waitFor({ state: "visible", timeout: 10_000 });

  // とめる → 球を選ぶ → 材質を変える(`hasUnsavedWork()` が立つ、
  // `setBodyMaterial` の doc 参照)。
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "true");
  await page.click("#btn-run");
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");
  await page.locator("#hierarchy-tree .tree-body").last().click();
  await page.selectOption("#focus-material", "木材(松)");
  await expect
    .poll(async () => page.locator("#focus-material").inputValue(), { timeout: 10_000 })
    .toBe("木材(松)");

  // ある程度時間を進めてから、もう一度とめて値を固定する(「はじめから」は
  // 止めた意思を引き継がず必ず動かすので、`before` を動いたまま読むと
  // 「はじめから」後にさらに進んだ分と競合し、まれに `after` が `before` を
  // 追い越しかねない——`before` は止めて固定した値で読む)。しきい値を高めに
  // 取り、「はじめから」後 500ms 経っても追い付かない余裕を持たせる。
  await page.click("#btn-run");
  await expect.poll(() => elapsedSeconds(page), { timeout: 10_000 }).toBeGreaterThan(1.0);
  await page.click("#btn-run");
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");
  const before = await elapsedSeconds(page);
  expect(before).toBeGreaterThan(1.0);

  await page.click("#btn-restart");
  await page.waitForTimeout(500);

  // 確認は一切出ない。
  expect(dialogs).toEqual([]);
  // 「はじめから」は実際に効いている——経過時刻がいったん頭へ戻り、
  // 止めて固定した値より小さいところから再び進む(ダイアログに阻まれて
  // 無反応、ではないことの実測)。
  const after = await elapsedSeconds(page);
  expect(after).toBeLessThan(before);
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

/** `__scene` から`referenceGrid`(課題A)の`visible`を読む。無ければ`null`。 */
async function referenceGridVisible(page: Page): Promise<boolean | null> {
  return page.evaluate(() => {
    const scene = (window as unknown as { __scene: any }).__scene;
    let grid: any = null;
    scene.traverse((o: any) => {
      if (o.userData?.isReferenceGrid) grid = o;
    });
    return grid ? (grid.visible as boolean) : null;
  });
}

/**
 * 3D舞台(`#scene-view-canvas-host`)を`gapMs`あけて2枚撮り、**画素が実際に
 * どれだけ変わったか**を割合(0〜100)で返す。
 *
 * 「板を置いた」「座標上はカメラの近くにある」だけでは、線が1〜2本しか
 * 見えず実質止まって見える状態でも通ってしまう(進行管理役の実測による
 * 差し戻し: 座標ベースの裏取りだけのテストは green のまま、画面は
 * ほぼ静止していた)。**見比べて動いていると分かるか**を直接測る。
 */
async function screenshotDiffPercent(page: Page, gapMs: number): Promise<number> {
  const stage = page.locator("#scene-view-canvas-host");
  const before = await stage.screenshot();
  await page.waitForTimeout(gapMs);
  const after = await stage.screenshot();
  return page.evaluate(
    async ({ a, b }) => {
      function loadImg(dataUrl: string): Promise<HTMLImageElement> {
        return new Promise((resolve, reject) => {
          const img = new Image();
          img.onload = () => resolve(img);
          img.onerror = reject;
          img.src = dataUrl;
        });
      }
      const [imgA, imgB] = await Promise.all([
        loadImg("data:image/png;base64," + a),
        loadImg("data:image/png;base64," + b),
      ]);
      const w = imgA.width;
      const h = imgA.height;
      const draw = (img: HTMLImageElement) => {
        const c = document.createElement("canvas");
        c.width = w;
        c.height = h;
        const ctx = c.getContext("2d")!;
        ctx.drawImage(img, 0, 0);
        return ctx.getImageData(0, 0, w, h).data;
      };
      const d1 = draw(imgA);
      const d2 = draw(imgB);
      let diff = 0;
      const total = w * h;
      const THRESHOLD = 12; // 微小なAA/ノイズ差は無視する
      for (let i = 0; i < d1.length; i += 4) {
        const dr = Math.abs(d1[i] - d2[i]);
        const dg = Math.abs(d1[i + 1] - d2[i + 1]);
        const db = Math.abs(d1[i + 2] - d2[i + 2]);
        if (dr + dg + db > THRESHOLD) diff++;
      }
      return (diff / total) * 100;
    },
    { a: before.toString("base64"), b: after.toString("base64") },
  );
}

// 課題A: 磁石が銅管を落ちる・空気をばねにする・氷が水に変わるは、床も水面も
// 無いまま対象だけが動く。かんたんモードの追従カメラは対象を画面の同じ場所へ
// 置き続けるので、数値は動いていても絵が1ピクセルも変わらなかった
// (利用者役・進行管理役の実測)。板(`referenceGrid`)を対象の背後・カメラの
// 近くへ置き直し続けることで直した——ただし、板を置いただけ・座標上カメラの
// 近くにあるだけでは足りない。線の間隔が固定の1mだと、カメラが対象の
// 数十cm手前まで寄るこの3実験では視界に線が1〜2本しか入らず、実質止まって
// 見えたまま(進行管理役の差し戻し)。線の間隔を毎フレーム画角に対して
// 一定本数(`REFERENCE_GRID_CELLS_ACROSS_VIEW`)になるよう決め直すことで
// 直した(`demo/src/main.ts`の`updateReferenceGrid`のdoc参照)。ここでは
// 座標ではなく**実際に画面の画素が変わった割合**で裏取りする。
//
// 「氷が水に変わる」(d18b-ice-melts)は元はこの一覧にいたが、課題B
// (利用者役の報告: 氷が床も無く永遠に落ち続けていた)への対応でSPHの器の
// 床に合わせた静止した床(`type: "static"`の`plane`)をシーンJSONへ足した
// ——これで**本物の基準**(床)ができたので、もう「基準の無い場面」では
// ない。方眼を追加で出さないことは、下の「床のある場面では、方眼を追加で
// 出さない」に移して確かめる。
for (const id of ["d21-copper-tube", "d17-piston"]) {
  test(`基準の無い場面(${id})は、1秒で画面の5%以上の画素が変わる(課題A)`, async ({ page }) => {
    const errors = collectPageErrors(page);
    await boot(page);
    await page.keyboard.press("Control+k");
    await page.click(`.palette-row[data-experiment-id="${id}"]`);
    await page.waitForTimeout(1500);

    expect(await referenceGridVisible(page)).toBe(true);
    const diffPct = await screenshotDiffPercent(page, 1000);
    expect(diffPct).toBeGreaterThan(5);
    expect(errors).toEqual([]);
  });
}

test("床のある場面では、方眼を追加で出さない(課題Aの回帰防止)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  for (const id of ["d1-free-fall", "d2-ballistic", "d18b-ice-melts"]) {
    await page.keyboard.press("Control+k");
    await page.click(`.palette-row[data-experiment-id="${id}"]`);
    await page.waitForTimeout(1000);
    // 座標ではなく`visible`そのもの——方眼を追加で出す条件のコードパスに
    // 一切入らないことを直接確かめる(見た目が変わらないことの一番確かな
    // 保証。物理のタイミングは実行ごとにぶれるため、床のある実験は
    // ピクセル差分では比較しない)。
    expect(await referenceGridVisible(page)).toBe(false);
  }
  expect(errors).toEqual([]);
});

// 課題B: 「氷が水に変わる」(d18b-ice-melts)は氷が融ける熱の現象なのに
// 「🚗 のりもの・機械」に分類されていた。「🔥 熱・温度」へ移した。
test("「氷が水に変わる」は「熱・温度」に分類され、「のりもの・機械」からは消えている(課題B)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // カタログのデータそのものの裏取り。
  const heat = CATEGORIES.find((c) => c.id === "heat");
  const machine = CATEGORIES.find((c) => c.id === "machine");
  expect(heat?.experiments.some((e) => e.id === "d18b-ice-melts")).toBe(true);
  expect(machine?.experiments.some((e) => e.id === "d18b-ice-melts")).toBe(false);

  // 実際にそのタブから見つかることを画面で確かめる。
  await page.keyboard.press("Control+k");
  await page.click('[data-category-id="heat"]');
  await expect(
    page.locator('.palette-row[data-experiment-id="d18b-ice-melts"]'),
  ).toBeVisible();
  await page.click('[data-category-id="heat"]'); // フィルタ解除
  await page.click('[data-category-id="machine"]');
  await expect(
    page.locator('.palette-row[data-experiment-id="d18b-ice-melts"]'),
  ).toHaveCount(0);
  expect(errors).toEqual([]);
});

/**
 * ボディの中心をワールド座標から画面座標(CSSピクセル)へ投影する。
 * `bodyOnScreen`/`cameraToLastBody`と同じ、テスト専用に露出された
 * `window.__camera`/`window.__world`を使う投影計算(このファイル冒頭の
 * doc参照)。実際にクリックする画面座標を作るための版(あちらは画角内かの
 * 判定のみ)。
 */
async function screenPointForBody(
  page: Page,
  bodyIndex: number,
): Promise<{ x: number; y: number }> {
  return page.evaluate((index) => {
    const cam = (window as unknown as {
      __camera: {
        matrixWorldInverse: { elements: number[] };
        projectionMatrix: { elements: number[] };
      };
    }).__camera;
    const world = (window as unknown as {
      __world: { body_position_at_f32(index: number): Float32Array };
    }).__world;
    const p = world.body_position_at_f32(index);
    const mulMat4Vec4 = (e: number[], v: number[]) => {
      const out = [0, 0, 0, 0];
      for (let r = 0; r < 4; r += 1) {
        out[r] = e[r] * v[0] + e[4 + r] * v[1] + e[8 + r] * v[2] + e[12 + r] * v[3];
      }
      return out;
    };
    const view = mulMat4Vec4(cam.matrixWorldInverse.elements, [p[0], p[1], p[2], 1]);
    const clip = mulMat4Vec4(cam.projectionMatrix.elements, view);
    const ndcX = clip[0] / clip[3];
    const ndcY = clip[1] / clip[3];
    const canvas = document.querySelector("canvas")!;
    const rect = canvas.getBoundingClientRect();
    return {
      x: (ndcX * 0.5 + 0.5) * rect.width + rect.left,
      y: (-ndcY * 0.5 + 0.5) * rect.height + rect.top,
    };
  }, bodyIndex);
}

// 課題A: 「坂はすべる? 止まる?」の箱をクリックしても「選んだもの」が出な
// かった。実測すると、稜線を足す`addEdgeLines`が付けた`THREE.LineSegments`
// (面と同じ位置にある、見た目だけの飾り)が`raycaster.intersectObjects`の
// 既定(再帰的)に拾われ、既定の太さ判定(ワールド座標で1m)のせいで面より
// **近い**当たりとして割り込んでいた。`hitTest`はいちばん近い当たりの持ち主を
// `pickables`から探すが、稜線は登録されていないため見つからず、当たっている
// のに`null`を返して黙って何も起きなかった(利用者役の観察の再現)。
// 稜線の当たり判定を切り、`hitTest`が親を辿って持ち主を探すようにして直した。
test("坂の実験で、箱をクリックすると「選んだもの」が出る(課題A)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);

  // 再生中のクリックと、とめてからのクリックの両方(利用者役の実測の表と
  // 同じ2条件)。毎回、実験を選び直してまっさらな状態(未選択)から試す。
  for (const playing of [true, false]) {
    await page.keyboard.press("Control+k");
    await page.click('.palette-row[data-experiment-id="d5-incline"]');
    await page.waitForTimeout(800);

    const playBtn = page.locator("#btn-run");
    const isPlaying = (await playBtn.getAttribute("data-playing")) === "true";
    if (isPlaying !== playing) await playBtn.click();
    await page.waitForTimeout(150);

    const point = await screenPointForBody(page, 1); // index 1 = box
    await page.mouse.click(point.x, point.y);
    await page.waitForTimeout(300);
    await expect(page.locator('.card[data-card="focus"]')).toHaveCount(1);
    await expect(page.locator('.card[data-card="focus"]')).toContainText("box");
  }
  expect(errors).toEqual([]);
});

// 他の実験でクリック選択が壊れていないことの確認(課題Aの回帰防止)。
// ボール落下・積み木・跳ねるボール——いずれも「選んだもの」札が出る。
for (const [id, bodyLabel] of [
  ["d1-free-fall", "ball"],
  ["d4-box-stack", "box"],
  ["d3-bounce", "ball"],
] as const) {
  test(`${id} で、動く物をクリックすると「選んだもの」が出る(課題Aの回帰防止)`, async ({ page }) => {
    const errors = collectPageErrors(page);
    await boot(page);
    await page.keyboard.press("Control+k");
    await page.click(`.palette-row[data-experiment-id="${id}"]`);
    await page.waitForTimeout(800);

    const playBtn = page.locator("#btn-run");
    if ((await playBtn.getAttribute("data-playing")) === "true") await playBtn.click();
    await page.waitForTimeout(150);

    const point = await screenPointForBody(page, 1); // index 0 = 床、1 = 動く物
    await page.mouse.click(point.x, point.y);
    await page.waitForTimeout(300);
    await expect(page.locator('.card[data-card="focus"]')).toHaveCount(1);
    await expect(page.locator('.card[data-card="focus"]')).toContainText(bodyLabel);
    expect(errors).toEqual([]);
  });
}

// 課題B: 「箱の材質」ボタン(鋼・ゴム・木・氷・発泡スチロール・アルミ)は
// どれがよく滑るのかが画面のどこにも書いておらず、利用者は「氷が滑りやすい
// だろう」と勘で選ぶしかなかった。数値をでっち上げず、アプリが実際に使って
// いる摩擦係数(Rust側の材質DB、`material_properties_f64`)をボタンへ添えた。
test("材質ボタンに、実際の摩擦係数が添えてある(課題B)", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d5-incline"]');
  await page.waitForTimeout(500);

  const buttons = page.locator("#knob-material .knob-choice-btn");
  await expect(buttons).toHaveCount(6);

  // 6つとも、でっち上げでない実数(data-friction、`materialFriction`が
  // 返した値)を持っている。
  const frictions: Record<string, number> = {};
  const count = await buttons.count();
  for (let i = 0; i < count; i += 1) {
    const btn = buttons.nth(i);
    const label = (await btn.textContent()) ?? "";
    const raw = await btn.getAttribute("data-friction");
    expect(raw).not.toBeNull();
    const value = Number.parseFloat(raw ?? "NaN");
    expect(Number.isFinite(value)).toBe(true);
    // ボタンの文字にも同じ値が(小数第2位で)見えている。
    expect(label).toContain(value.toFixed(2));
    frictions[label] = value;
  }

  // 実測(進行管理役の裏取り): 氷がいちばん摩擦係数が小さく(=いちばん
  // よく滑り)、ゴムがいちばん大きい(=いちばん滑りにくい)。
  const iceEntry = Object.entries(frictions).find(([label]) => label.includes("氷"));
  const rubberEntry = Object.entries(frictions).find(([label]) => label.includes("ゴム"));
  expect(iceEntry).toBeDefined();
  expect(rubberEntry).toBeDefined();
  const allValues = Object.values(frictions);
  expect(iceEntry![1]).toBe(Math.min(...allValues));
  expect(iceEntry![1]).toBeLessThan(rubberEntry![1]);

  // 数字の読み方が、既存の一言補足の隣に短く添えてある。
  const hint = page.locator('[data-knob-id="material"] .knob-hint');
  await expect(hint).toContainText("重さ・跳ね返り・すべりやすさが一度に変わります");
  await expect(hint).toContainText("小さいほどよく滑ります");
  expect(errors).toEqual([]);
});

// 課題A(利用者役の報告): 「モーター」という名前から「回り続ける」動きを
// 期待して置いたのに、実際は目標角度まで振れてそこで止まる(サーボと同じ)
// 動きだった。物理は変えず(回り続けるようにするのは受け入れテストに
// 関わる別件)、実際の動きを置いた直後に言葉で伝える。
test("モーターを追加すると、回り続けないことがその場で分かる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "モーター");

  // 置いた直後のトーストで、実際の動き(角度まで動いて止まる)と
  // 「⟳ モーター切替」が何をするボタンなのかが分かる。
  const toast = page.locator(".toast-message");
  await expect(toast).toContainText("回り続ける");
  await expect(toast).toContainText("止まります");
  await expect(toast).toContainText("モーター切替");

  // ツールバーのボタン自体にも「回り続けない」ことが書いてある(押す前に
  // 分かる、ホバーだけに頼らない)。
  await expect(page.locator("#btn-motor-toggle")).toContainText("0°⇔90°");
  expect(errors).toEqual([]);
});

/**
 * 流体粒子(全体)のバウンディングボックスを、画面座標(CSSピクセル)での
 * 対角の大きさへ投影する。`bodyOnScreen`/`screenPointForBody`と同じ、
 * テスト専用に露出された`window.__camera`/`window.__world`を使う投影計算
 * (このファイル冒頭付近のdoc参照)——「見えているか」ではなく「どれだけの
 * 大きさに見えているか」を測る版。
 */
async function fluidProjectedDiagonalPx(page: Page): Promise<number> {
  return page.evaluate(() => {
    const cam = (window as unknown as {
      __camera: {
        matrixWorldInverse: { elements: number[] };
        projectionMatrix: { elements: number[] };
      };
    }).__camera;
    const world = (window as unknown as {
      __world: {
        read_component(kind: string, arg: string): string;
        fluid_particle_positions_f32(): Float32Array;
      };
    }).__world;
    const count = Number(world.read_component("fluid_particle_count", ""));
    const pos = world.fluid_particle_positions_f32();
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let i = 0; i < count; i += 1) {
      const x = pos[i * 3];
      const y = pos[i * 3 + 1];
      const z = pos[i * 3 + 2];
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }
    const mulMat4Vec4 = (e: number[], v: number[]) => {
      const out = [0, 0, 0, 0];
      for (let r = 0; r < 4; r += 1) {
        out[r] = e[r] * v[0] + e[4 + r] * v[1] + e[8 + r] * v[2] + e[12 + r] * v[3];
      }
      return out;
    };
    const canvas = document.querySelector("canvas")!;
    const rect = canvas.getBoundingClientRect();
    const project = (x: number, y: number, z: number) => {
      const view = mulMat4Vec4(cam.matrixWorldInverse.elements, [x, y, z, 1]);
      const clip = mulMat4Vec4(cam.projectionMatrix.elements, view);
      const ndcX = clip[0] / clip[3];
      const ndcY = clip[1] / clip[3];
      return {
        x: (ndcX * 0.5 + 0.5) * rect.width,
        y: (-ndcY * 0.5 + 0.5) * rect.height,
      };
    };
    const p0 = project(minX, minY, minZ);
    const p1 = project(maxX, maxY, maxZ);
    return Math.hypot(p1.x - p0.x, p1.y - p0.y);
  });
}

// 課題B(利用者役の報告): 「＋ 流体」を置くと物理的には生成されるが、
// 既定のカメラ距離では1〜2ピクセルの点にしか見えず、「置けたこと」が
// 画面から読めなかった。ボディのスポーンと同じ理由で、画面にちゃんと
// 入っていないときだけ画角を寄せる。
test("流体を追加すると、置いた直後から画面でちゃんと見える大きさになる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "流体");
  await page.waitForTimeout(300);

  // 直したのは「1〜2ピクセルの点」——十分大きな余裕を見て、1桁ピクセルより
  // はっきり大きいことだけを求める(画角の細かい合わせ方までは縛らない)。
  const diagonal = await fluidProjectedDiagonalPx(page);
  expect(diagonal).toBeGreaterThan(15);
  expect(errors).toEqual([]);
});

// 課題B(利用者役の報告)続き: 一覧の「Fluids」をクリックしてもInspectorが
// 「まだ何も選んでいません」のままで、押しても何も起きなかった。個々の
// 粒子や塊は(SPH流体がボディのような個別IDを持たないため)他のボディと
// 同じInspectorでは選べないが、それは「押しても無反応」の理由にはならない
// ——せめて数量が読め、選べない理由も画面で言う。
test("Fluidsの一覧行を選ぶと、数量と選べない理由が読める", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");
  await addViaMenu(page, "流体");
  await page.waitForTimeout(200);

  const fluidRow = page.locator("#hierarchy-tree li", { hasText: "Fluids" }).last();
  await fluidRow.click();

  const inspector = page.locator("#inspector-body");
  await expect(inspector).not.toContainText("まだ何も選んでいません");
  await expect(inspector).toContainText("水塊の数");
  await expect(inspector).toContainText("総粒子数");
  await expect(inspector).toContainText("選べません");
  // クリックした行自体も「選んだ」見た目になる(押しても無反応、をやめた証拠)。
  await expect(fluidRow).toHaveClass(/selected/);
  expect(errors).toEqual([]);
});

// 課題C(利用者役の報告): 「まだ何も選んでいません」に出る個数が、消した
// はずのボディを数え続けていた(`remove_body_at`はindexのずれを避けるため
// スロットを残すだけなので、生死問わず数える`body_count`をそのまま出すと
// 消した分だけ多く見える)。画面に出す個数は、いま生きている物の数にする。
test("消した物のあとの個数表示は、生きている数だけを数える", async ({ page }) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene"); // ground だけの場面。
  await addViaMenu(page, "箱");
  await page.waitForTimeout(200);

  const box = page.locator("#hierarchy-tree .tree-body", { hasText: "箱" });
  await box.click({ button: "right" });
  await page.locator("#context-menu button", { hasText: "削除" }).first().click();
  await page.waitForTimeout(200);

  // 生きているボディは ground だけ(1体)。
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(1);

  // 選択を解いて、空状態の個数表示を見る——消した箱を含めた「2」ではなく
  // 「1」でなければならない。
  await page.click("#btn-clear-selection");
  await expect(page.locator("#inspector-body")).toContainText("1 個あります");
  await expect(page.locator("#inspector-body")).not.toContainText("2 個あります");
  expect(errors).toEqual([]);
});

// **課題2(進行管理役の実測)**: 置いた物が、他の物が遠くにあると豆粒にしか
// ならない不具合。`d1-free-fall`を粒度2「しらべる」で開き、右クリックで
// 2個目の球を置いたところ、既存の球が遠く(実測: 距離47.3m)へ落ちて
// 止まっていたため、`frameCameraOnContent`が両方を入れる画角に引いてしまい、
// 新しい球は見かけの直径9.9pxにしかならなかった(「置いたのに何も起きな
// かった」と読まれて当然の大きさ)。ここでは同じ状況(遠くに他の物がある
// 場面で新しく1個置く)を、粒度3「つくる」の数値欄で確実に再現する。
test("置いた物は、他の物が遠くにあっても十分な大きさで見える(実測: 修正前は距離47.3m・直径9.9px)", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 3);
  await page.click("#btn-new-scene");

  // 1個目を置いて、進行管理役の実測と同じ桁(30〜40m)まで遠ざける
  // ——`contentBoundingBox`がこれも含めて画角を決める、「他の遠い物」役。
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await expect(page.locator("#focus-pos-x")).toBeVisible();
  await page.locator("#focus-pos-x").fill("40");
  await page.locator("#focus-pos-y").fill("0.4");
  const farZ = page.locator("#focus-pos-z");
  await farZ.fill("40");
  // **`dispatchEvent("change")` ではなく `Tab` で本物のフォーカス移動を
  // 起こして確定させる**。手で `dispatchEvent` すると、ブラウザ内部の
  // 「まだ確定していない」という印は消えないまま残り、この直後に別の要素
  // (下の2個目のスポーンボタン)をクリックしてフォーカスを奪った瞬間、
  // ブラウザが**もう一度**本物の`change`を同じ値で発火させてしまう
  // (実測で踏んだ: この2回目の`change`が`setBodyPosition`をもう一度呼び、
  // そちらの可視性フォールバックが1個目に画角を引き戻して、この後の
  // 「2個目が見える」検証を汚染していた)。`Tab`ならその場で一度だけ確定する。
  await farZ.press("Tab");
  await page.waitForTimeout(500);

  // 2個目を、進行管理役の実測と同じ状況(遠くに他の物がある場面)で置く。
  await page.evaluate(() => document.getElementById("btn-spawn-sphere")!.click());
  await page.waitForTimeout(500);

  // 直近に置いた物(2個目)が、十分な大きさで見えること。しきい値は
  // 「読める点」との境目として安全側の40pxに取る(修正後の実測は250px前後、
  // 修正前の実測は9.9px)。
  const diameter = await apparentSphereDiameterPx(page);
  expect(diameter).toBeGreaterThan(40);
  expect(errors).toEqual([]);
});

// **課題1・3・4(進行管理役の実測)**: 粒度2「しらべる」で`d1-free-fall`を
// 開いた状態の再現。ツールバー(`#btn-add`)は畳まれているのに、「選んだ
// もの」札の「🗑 これを消す」は選ぶだけで出る——**消す手段は見えるのに
// 足す手段が見えない**という非対称。右クリックすれば実は置けるが、
// 「ここに球を配置 (-20.30, -33.80)」のような生座標や「複合形状(L字)」
// 「凸包メッシュ」のような内部語彙はこの粒度の人には読めない。置いた物の
// 名前も`Sphere_2`という機械語のままだった。
test("粒度2「しらべる」でも、足す手段が見つかり、置いた物の名前が読める", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await boot(page);
  await setGrain(page, 2); // しらべる(進行管理役の実測と同じ粒度)。
  await page.keyboard.press("Control+k");
  await page.click('.palette-row[data-experiment-id="d1-free-fall"]');
  await page.locator("#crumb-experiment").waitFor({ state: "visible", timeout: 10_000 });

  // 実測どおりの前提: この粒度ではツールバーがまだ畳まれている。
  await expect(page.locator("#toolbar")).toBeHidden();

  // とめてから足す(物理が進んでいる最中の測定でぶれないように)。
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "true");
  await page.click("#btn-run");
  await expect(page.locator("#btn-run")).toHaveAttribute("data-playing", "false");

  // ①「足す」導線が、この粒度でも見つかること(課題1)。
  await expect(page.locator('.card[data-card="add-body"]')).toBeVisible();
  const addButton = page.locator("#btn-add-body-card");
  await expect(addButton).toBeVisible();
  const before = await page.locator("#hierarchy-tree .tree-body").count();
  await addButton.click();
  await page.waitForTimeout(500);
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(before + 1);

  // ②置いた物が、機械語(`Sphere_2`)ではなく読める名前で出ること——場面の
  // 中身・パンくず・「選んだもの」札の3箇所(課題3)。
  await expect(page.locator("#hierarchy-tree")).not.toContainText("Sphere_");
  const crumb = page.locator("#crumb-body");
  await expect(crumb).toBeVisible();
  await expect(crumb).not.toContainText("Sphere_");
  await expect(crumb).toContainText("球");
  const focus = page.locator('.card[data-card="focus"]');
  await expect(focus).toBeVisible();
  await expect(focus).not.toContainText("Sphere_");
  await expect(focus).toContainText("球");

  // ③置いた物が、置いた本人に見える大きさになること(課題2、単独テストは
  // 上の「置いた物は、他の物が遠くにあっても…」参照)。
  expect(await apparentSphereDiameterPx(page)).toBeGreaterThan(40);

  // ④右クリックのスポーンパレットも、この粒度では平易な言葉になっている
  // こと——生座標・「複合形状(L字)」「凸包メッシュ」のような内部語彙は
  // 出さない(「つくる」粒度専用のまま、課題4)。
  const canvas = page.locator("#scene-view-canvas-host canvas").first();
  const box = (await canvas.boundingBox())!;
  await page.mouse.click(box.x + box.width * 0.3, box.y + box.height * 0.6, {
    button: "right",
  });
  const menu = page.locator("#context-menu");
  await expect(menu).toBeVisible();
  await expect(menu).toContainText("ここに球を置く");
  await expect(menu).not.toContainText("複合形状");
  await expect(menu).not.toContainText("凸包メッシュ");
  await expect(menu).not.toContainText("配置 (");
  await page.keyboard.press("Escape");

  expect(errors).toEqual([]);
});
