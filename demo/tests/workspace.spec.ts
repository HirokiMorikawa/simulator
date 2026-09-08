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
  await expect(focus).toContainText("Box_1");
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
  await expect(page.locator("#hierarchy-tree")).toContainText("高さ(Sphere_2)");
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
  await expect(probes).toContainText("高さ(Sphere_1)");
  await expect(probes).not.toContainText("消えた物");

  // 3D と一覧からは消える一方、グラフに描いた過去データを黙って捨てるのは
  // 乱暴なので、「記録している値」には残す——ただし**もう無い物だと分かる**
  // ように注記する(課題B、`friendlyProbeLabel`のdoc参照)。
  await page.locator("#hierarchy-tree li", { hasText: "Sphere_1" }).first().click();
  await page.keyboard.press("Delete");
  await page.waitForTimeout(300);

  await expect(probes).toContainText("高さ(Sphere_1・消えた物)");
  await expect(probes).toContainText("速さ(Sphere_1・消えた物)");
  // 3D・一覧からは実際に消えていること(一覧の食い違いそのものは直っている)。
  await expect(page.locator("#hierarchy-tree")).not.toContainText("↳ Sphere_1");
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
