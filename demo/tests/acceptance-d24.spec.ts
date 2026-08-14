import { expect, test, type Page } from "@playwright/test";
import { addViaMenu, collectPageErrors, waitForWorld } from "./helpers";

// 縦串①(ジョイント)の受け入れテスト(docs/22-roadmap/03-editor-todo.md
// 「縦串①の受け入れテストを緑にする」、docs/reviews/2026-08-14-
// editor-implementation-plan.md の合格判定そのもの)。
//
// 「UIで車を組み立てて、保存して、読み直したら D24 と同じ挙動になる」を、
// D24 車(`scenes/d24-car.json`)相当の構成——シャシー(Box)+ 車輪4個(Sphere)
// + WheelJoint4本(前輪は駆動なし・後輪は駆動あり)——を **Scene View/Inspector
// のUI操作だけ**で組み立て、既存の D24 シーンJSONをそのまま実行した結果と
// `state_hash()` が一致することで検証する。
//
// **「UIのみ」で完結する**: ボディの生成(スポーンパレット)・位置
// (Position入力)・スケール(Scale入力、球にも等方フォールバックが効く——
// 本増分で追加)・質量(Massコマンド)・衝突フィルタ・Joint追加(Add Joint
// フォーム、本増分で追加)は、すべて実際にエディタが提供するUI操作を
// `page.fill`/`page.click`/`page.selectOption`で叩く。開始状態も例外では
// ない——既定の起動シーンには回路・熱ドメインの実演用セットアップが最初から
// 載っている(`WasmWorld::new`)ため、まず「新規シーン」ボタン(`#btn-new-scene`、
// レビュー指摘「出来るようにして欲しい」への対応で追加)を押して床だけの
// シーンへ差し替え、続けてSettingsの`dt`欄(Editモードのみ編集可能、
// `world.set_dt`経由)をD24シーンと同じ値へUIから書き換える。
// **テスト専用フックは一切使わない**(以前は`window.__loadSceneJson`で
// ground相当のJSONを直接注入していたが、「新規シーン」ボタンの新設により
// 不要になった)。

const D24_DT = 1 / 240;

const STEPS = 60;

/** Inspector の Position/Scale/Mass/Collision フィールドへ直接値を入れる。 */
async function setTransformAndBody(
  page: Page,
  opts: {
    position: [number, number, number];
    scale: [number, number, number];
    mass: number;
    collisionGroup: number;
    collisionMask: number;
  },
) {
  const { position, scale, mass, collisionGroup, collisionMask } = opts;
  for (const [axis, value] of [
    ["x", position[0]],
    ["y", position[1]],
    ["z", position[2]],
  ] as const) {
    await page.fill(`#inspector-position-${axis}`, String(value));
  }
  for (const [axis, value] of [
    ["x", scale[0]],
    ["y", scale[1]],
    ["z", scale[2]],
  ] as const) {
    await page.fill(`#inspector-scale-${axis}`, String(value));
  }
  await page.fill("#inspector-mass", String(mass));
  await page.fill("#inspector-collision-group", String(collisionGroup));
  await page.fill("#inspector-collision-mask", String(collisionMask));
}

/** Add Joint フォームで Wheel Joint を1本追加する。 */
async function addWheelJoint(
  page: Page,
  opts: {
    chassis: number;
    wheel: number;
    anchor: [number, number, number];
    restLength: number;
    frequency: number;
    dampingRatio: number;
    motorSpeed: number;
    motorMaxTorque: number;
  },
) {
  await page.selectOption("#add-joint-kind", "wheel");
  await page.fill("#add-joint-body-a", String(opts.chassis));
  await page.fill("#add-joint-ax", String(opts.anchor[0]));
  await page.fill("#add-joint-ay", String(opts.anchor[1]));
  await page.fill("#add-joint-az", String(opts.anchor[2]));
  await page.fill("#add-joint-body-b", String(opts.wheel));
  await page.fill("#add-joint-p1", String(opts.restLength));
  await page.fill("#add-joint-p2", String(opts.frequency));
  await page.fill("#add-joint-p3", String(opts.dampingRatio));
  await page.fill("#add-joint-p4", "0");
  await page.fill("#add-joint-p5", String(opts.motorSpeed));
  await page.fill("#add-joint-p6", String(opts.motorMaxTorque));
  await page.click("#add-joint-button");
}

test("縦串①: D24車をUIのみで組み立てるとD24シーンJSONの実行結果とstate_hashが一致する", async ({
  page,
}) => {
  const errors = collectPageErrors(page);

  // ---- 基準値: 既存のD24シーンJSONをそのまま実行 ----
  await page.goto("/");
  await waitForWorld(page);
  await page.click('.project-tab[data-tab="scenes"]');
  const loadButton = page.locator(
    '.scene-gallery-list button[data-scene-file="d24-car.json"]',
  );
  await expect(loadButton).toBeVisible();
  await loadButton.click();
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(6);
  const referenceHash: string = await page.evaluate((steps) => {
    const w = (window as unknown as { __world: any }).__world;
    for (let i = 0; i < steps; i += 1) w.step();
    return w.state_hash();
  }, STEPS);

  // ---- UIのみでD24相当を組み立てる ----
  await page.goto("/");
  await waitForWorld(page);
  // 「新規シーン」ボタン(実UI)で床だけのシーンへ差し替える。
  await page.click("#btn-new-scene");
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(1);
  // dtをD24シーンと同じ値へ、Settingsの実入力欄からUI操作で書き換える
  // (Editモードのみ編集可能——起動直後は既定でEditモードなのでそのまま使える)。
  // `change`を明示dispatchするのは既存の「群2」テスト(重力欄)と同じ理由
  // (`page.fill`だけではリスナーが拾わない)。
  await page.click("#btn-settings");
  await page.fill("#input-dt", String(D24_DT));
  await page.locator("#input-dt").dispatchEvent("change");
  await page.locator("#input-dt").blur();
  await page.waitForTimeout(100);
  await page.click("#btn-settings"); // ポップオーバーを閉じる。

  // シャシー(Box、body index 1)。既定スポーン半径0.4→半径[1.2,0.25,0.6]へ
  // 軸別スケール、質量600kg、collision_group=2 mask=4294967289(D24と同一)。
  await page.selectOption("#select-spawn-material", "鋼(炭素鋼)");
  await addViaMenu(page, "＋ 箱");
  await setTransformAndBody(page, {
    position: [0.0, 0.75, 0.0],
    scale: [1.2 / 0.4, 0.25 / 0.4, 0.6 / 0.4],
    mass: 600.0,
    collisionGroup: 2,
    collisionMask: 4294967289,
  });

  // 車輪4個(Sphere、body index 2〜5、D24のJSON順=fl,fr,rl,rrと同じ順で
  // スポーンする——WheelJointの解決順が物理結果に影響するため順序が要る)。
  // 既定スポーン半径0.4→0.32へ等方スケール(3欄とも同じ値、球へのフォール
  // バックが効く)、質量60kg、collision_group=4 mask=4294967289。
  await page.selectOption("#select-spawn-material", "ゴム(天然)");
  const wheelPositions: Record<string, [number, number, number]> = {
    fl: [0.9, 0.32, 0.65],
    fr: [0.9, 0.32, -0.65],
    rl: [-0.9, 0.32, 0.65],
    rr: [-0.9, 0.32, -0.65],
  };
  const wheelIndex: Record<string, number> = {};
  for (const key of ["fl", "fr", "rl", "rr"] as const) {
    await addViaMenu(page, "＋ 球");
    const index: number = await page.evaluate(
      () =>
        Number(
          (window as unknown as { __world: any }).__world.read_component("body_count", ""),
        ) - 1,
    );
    wheelIndex[key] = index;
    await setTransformAndBody(page, {
      position: wheelPositions[key],
      scale: [0.32 / 0.4, 0.32 / 0.4, 0.32 / 0.4],
      mass: 60.0,
      collisionGroup: 4,
      collisionMask: 4294967289,
    });
  }
  await expect(page.locator("#hierarchy-tree .tree-body")).toHaveCount(6);

  // WheelJoint4本(D24のJSON順=fl,fr,rl,rrと同じ順で追加する)。前輪は駆動
  // なし(motor_speed=0, motor_max_torque=0)、後輪は駆動あり(12rad/s, 200)。
  const chassisIndex = 1;
  const anchors: Record<string, [number, number, number]> = {
    fl: [0.9, 0.0, 0.65],
    fr: [0.9, 0.0, -0.65],
    rl: [-0.9, 0.0, 0.65],
    rr: [-0.9, 0.0, -0.65],
  };
  for (const key of ["fl", "fr", "rl", "rr"] as const) {
    const driven = key === "rl" || key === "rr";
    await addWheelJoint(page, {
      chassis: chassisIndex,
      wheel: wheelIndex[key],
      anchor: anchors[key],
      restLength: 0.43,
      frequency: 2.5,
      dampingRatio: 0.7,
      motorSpeed: driven ? 12.0 : 0.0,
      motorMaxTorque: driven ? 200.0 : 0.0,
    });
  }
  const jointText = await page.evaluate(() =>
    (window as unknown as { __world: any }).__world.read_component("joint_info_text", "-1"),
  );
  expect(jointText.split("\n").filter((l: string) => l.includes("WheelJoint")))
    .toHaveLength(4);

  const builtHash: string = await page.evaluate((steps) => {
    const w = (window as unknown as { __world: any }).__world;
    for (let i = 0; i < steps; i += 1) w.step();
    return w.state_hash();
  }, STEPS);

  expect(builtHash).toBe(referenceHash);
  expect(errors).toEqual([]);
});
