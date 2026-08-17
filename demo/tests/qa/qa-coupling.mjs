// **物理法則の組み合わせ(ドメイン間結合)を UI 操作で確かめる**。
// 報告書は docs/reviews/2026-08-04-coupling-qa.md。
//
// `qa-physics.mjs` が見ているのは単一ドメインの解析解(自由落下・振り子・ケプラー)で、
// **2つ以上の物理法則を組み合わせたときに正しいか**は誰も UI 越しに見ていなかった。
// ここがその隙間を埋める。判定は 2 種類ある。
//
// 1. **プリセットの結合**(§X): 結合を宣言したギャラリーシーンを `Scene` 選択から
//    読み込み、`⏭` で進め、Probe 履歴・HUD・エネルギー台帳から値を取って解析解と
//    突き合わせる。橋の両側(出す側のドメインと受け取る側のドメイン)を必ず両方読む
//    ——片側だけでは「結合が効いた」ことしか言えず、「保存量が正しく渡った」ことは
//    言えないため。
// 2. **UI からの組み合わせ**(§Y): ユーザーがエディタの操作だけで別ドメインを
//    足したときに、既存の結合がそれを拾うか。Settings のヒーター、Circuit タブの
//    自由配線、`＋ 追加` の流体、Project の Import が対象。
//
// Rust のテスト関数は一切呼ばない(qa-lib.mjs の方針をそのまま引き継ぐ)。
import fs from "fs";
import {
  launch,
  boot,
  results,
  loadScene,
  enterPlayPaused,
  stepN,
  probes,
  worldState,
  OUT,
} from "./qa-lib.mjs";

fs.mkdirSync(OUT, { recursive: true });
const { browser, page, errors } = await launch();
const r = results();
await boot(page);

const shot = (name) => page.screenshot({ path: `${OUT}/coupling-${name}.jpg`, quality: 70, type: "jpeg" });
const rel = (a, b) => Math.abs(a - b) / Math.abs(b);
const byLabel = (ps, frag) => ps.find((p) => p.label.includes(frag));
/// シーンを読み込んで Play(停止状態)へ入り、`steps` だけ進める。
async function run(frag, steps) {
  await page.reload();
  await boot(page);
  await loadScene(page, frag);
  await enterPlayPaused(page);
  await stepN(page, steps);
}
/// 結合の内省(Inspector の Coupling コンポーネントが読むのと同じ経路)。
const couplings = () =>
  page.evaluate(() =>
    window.__world
      .read_component("coupling_info_text", "-1")
      .split("\n")
      .filter((l) => l.length > 0)
      .map((l) => {
        const [kind, description, domains] = l.split("\t");
        return { kind, description, domains };
      }),
  );

// ======================================================== X. プリセットの結合
//
// -------- X1: 力学 → 電磁 → 熱(D20 モーターと発電、結合 2 段)
await run("d20-hand-crank-generator", 600);
{
  const cs = await couplings();
  r.check("X1-0", "D20 が 2 つの結合を積んでいる(力学+電磁 / 電磁+熱)",
    cs.length === 2 && cs[0].domains === "Mechanics+Electromagnetism" && cs[1].domains === "Electromagnetism+Thermal",
    cs.map((c) => `${c.kind}(${c.domains})`).join(" → "));

  const ps = await probes(page);
  const v = byLabel(ps, "CircuitV").h.at(-1);
  const i = byLabel(ps, "CircuitCurrent").h.at(-1);
  const temp = byLabel(ps, "NodeTemp").h;
  // クランクは ω=10 rad/s のキネマティック体、トルク定数 k=0.05 N·m/A。
  // 逆起電力 V = kω、回路は 10Ω 単独なので I = V/R。
  r.check("X1-1", "力学 → 電磁: 逆起電力 V = kω", rel(v, 0.5) < 0.01, `V = ${v.toFixed(4)} V(解析 0.5 V)`);
  r.check("X1-2", "電磁: オームの法則 I = V/R", rel(Math.abs(i), 0.05) < 0.01, `I = ${i.toFixed(5)} A(解析 −0.05 A)`);
  // 電磁 → 熱: P = VI = 25 mW を熱容量 1000 J/K のノードへ 5 s 注ぐ。
  const dT = temp.at(-1) - 293.15;
  r.check("X1-3", "電磁 → 熱: ΔT = VI·t/C(3 ドメインを貫くエネルギー)",
    rel(dT, 0.5 * 0.05 * 5.0 / 1000.0) < 0.02,
    `ΔT = ${dT.toExponential(4)} K(解析 ${(0.025 * 5 / 1000).toExponential(4)} K)`);
  await shot("d20-generator");
}

// -------- X2: 力学 → 熱(D10 摩擦の熱)
await run("d10-brake-heat", 600);
{
  const ps = await probes(page);
  const speed = byLabel(ps, "BodySpeed").h;
  const temp = byLabel(ps, "NodeTemp").h;
  const mass = await page.evaluate(() => Number(window.__world.read_component("body_mass_at", "1")));
  const heat = (i) => 1000.0 * (temp[i] - 293.15); // C = 1000 J/K
  // 「止まった」= スリープ閾値(SLEEP_LINEAR_THRESHOLD = 0.01 m/s)を下回った step。
  // 実際に速度が厳密な 0 になるのはスリープに入る 0.5 秒後なので、そこを止まった
  // 時刻と読むと下の X2-3 が見たいもの(静止中の発熱)を取りこぼす。
  const stop = speed.findIndex((v) => v < 0.01);
  const ke0 = 0.5 * mass * 3.0 ** 2; // シーンの初速 3 m/s
  r.check("X2-1", "力学: 摩擦で静止する",
    speed.at(-1) === 0,
    `v: ${speed[0].toFixed(3)} → 0 m/s(step ${stop + 1}、滑走距離 ${byLabel(ps, "BodyPosX").h.at(-1).toFixed(4)} m)`);
  r.check("X2-2", "力学 → 熱: 停止までに熱へ渡った量が失われた運動エネルギーと一致する",
    rel(heat(stop), ke0) < 0.02,
    `C·ΔT(停止時) = ${heat(stop).toFixed(0)} J / 初期 KE = ${ke0.toFixed(0)} J(差 ${(((heat(stop) - ke0) / ke0) * 100).toFixed(1)} %)`);
  // 静止後も熱が湧き続ける: 接触の法線インパルスが毎 step 打ち消す重力ぶんの
  // 速度増分 gΔt が散逸として計上される(½m(gΔt)² = 26.2 J/step)。
  const perStep = (heat(599) - heat(stop)) / (599 - stop);
  r.check("X2-3", "静止した物体は発熱しない",
    heat(599) - heat(stop) < 1.0,
    `静止後(step ${stop + 1} → 600)にさらに ${(heat(599) - heat(stop)).toFixed(0)} J = ${perStep.toFixed(1)} J/step(½m(gΔt)² = ${(0.5 * mass * (9.80665 * 0.008333333) ** 2).toFixed(1)} J/step)、T ${temp[stop].toFixed(2)} → ${temp[599].toFixed(2)} K`);
  await shot("d10-brake-heat");
}

// -------- X3: 力学 ↔ 電磁(D21 渦電流ブレーキ、双方向)
await run("d21-copper-tube-drop", 600);
{
  const ps = await probes(page);
  const v = byLabel(ps, "BodySpeed").h.at(-1);
  const { t } = await worldState(page);
  // m=0.01 kg, R=1Ω, B=0.5 T, l=0.1 m → 終端速度 v∞ = mgR/(B²l²)、時定数 τ = mR/(B²l²)。
  const k = 0.5 ** 2 * 0.1 ** 2 / 1.0; // B²l²/R = 2.5e-3
  const vInf = 0.01 * 9.80665 / k;
  const tau = 0.01 / k;
  const analytic = vInf * (1 - Math.exp(-t / tau));
  const freeFall = 9.80665 * t;
  r.check("X3-1", "力学 ↔ 電磁: 渦電流ブレーキの速度が解析解と一致",
    rel(v, analytic) < 0.01,
    `v(${t.toFixed(3)} s) = ${v.toFixed(4)} m/s(解析 ${analytic.toFixed(4)} m/s、誤差 ${(rel(v, analytic) * 100).toFixed(3)} %)`);
  r.check("X3-2", "ブレーキが実際に効いている(自由落下より遅い)",
    v < freeFall * 0.98,
    `v = ${v.toFixed(4)} m/s < 自由落下 ${freeFall.toFixed(4)} m/s(${((1 - v / freeFall) * 100).toFixed(1)} % 減speed)`);
}

// -------- X4: 回路 → 熱(D19 電気工作台)
await run("d19-electric-workbench", 600);
{
  const ps = await probes(page);
  const v2 = byLabel(ps, "CircuitV[2]").h;
  const v3 = byLabel(ps, "CircuitV[3]").h;
  const cur = byLabel(ps, "CircuitCurrent").h;
  const temp = byLabel(ps, "NodeTemp").h;
  const { dt } = await worldState(page);
  r.check("X4-1", "回路: 分圧則 V₂ = 9·2k/(1k+2k)", rel(v2.at(-1), 6.0) < 0.001, `V₂ = ${v2.at(-1).toFixed(4)} V(解析 6 V)`);
  // RC 放電(C=1 mF, R=500Ω → τ=0.5 s)。後退 Euler は減衰をわずかに過小評価する。
  const idx = 100;
  const tAt = (idx + 1) * dt;
  const analytic3 = 9.0 * Math.exp(-tAt / 0.5);
  r.check("X4-2", "回路: RC 放電 V₃ = V₀e^(−t/RC)",
    rel(v3[idx], analytic3) < 0.03,
    `V₃(${tAt.toFixed(4)} s) = ${v3[idx].toFixed(4)} V(解析 ${analytic3.toFixed(4)} V、後退 Euler の数値減衰ぶん)`);
  // 回路 → 熱: 抵抗損失 ΣV²/R を積分した値が熱ノードの ΔT·C と一致するか。
  // 抵抗は 1kΩ(N1–N2)・2kΩ(N2–GND)・500Ω(N3–GND)・470Ω(N4–N5)。
  // 470Ω は LED 枝の電流制限抵抗(不具合 3 の修正で追加)。これを足し忘れると
  // `JouleHeat` が熱へ渡す量(全抵抗の ΣV²/R)と食い違う。
  const v4 = byLabel(ps, "CircuitV[4]").h;
  const v5 = byLabel(ps, "CircuitV[5]").h;
  let joule = 0;
  for (let i = 0; i < v2.length; i += 1) {
    joule +=
      ((9.0 - v2[i]) ** 2 / 1000.0 +
        v2[i] ** 2 / 2000.0 +
        v3[i] ** 2 / 500.0 +
        (v4[i] - v5[i]) ** 2 / 470.0) *
      dt;
  }
  const dT = temp.at(-1) - 293.15;
  r.check("X4-3", "回路 → 熱: 抵抗損失の積分が熱ノードの ΔT·C と一致",
    rel(dT * 1000.0, joule) < 0.02,
    `C·ΔT = ${(dT * 1000).toFixed(5)} J / ΣV²/R の積分 = ${joule.toFixed(5)} J(差 ${((dT * 1000 - joule) / joule * 100).toFixed(2)} %)`);
  // ダイオード枝(SW0 閉 → 470Ω → D → GND)。以前は直列抵抗が無く、閉じた
  // スイッチのオン抵抗だけが 9V 源とダイオードの間にあったため −7.875×10⁶ A
  // という物理的にありえない電流が流れていた(不具合 3)。LED の電流制限抵抗
  // 470Ω を入れて、分圧枝 3 mA + LED 枝 ≈ 18 mA の妥当な値にした。
  r.check("X4-4", "ダイオード枝の電流が物理的に妥当な範囲にある",
    Math.abs(cur.at(-1)) < 1e3,
    `電源電流 = ${cur.at(-1).toExponential(3)} A(LED 枝の電流制限抵抗 470Ω、V[5] = ${v5.at(-1).toFixed(4)} V がダイオードの順方向電圧)`);
  await shot("d19-workbench");
}

// -------- X5: 熱 → 流体(D15 対流、Boussinesq 浮力)
await run("d15-convection", 300);
{
  const ps = await probes(page);
  const meanV = byLabel(ps, "GridFluidMeanV").h;
  let monotonic = true;
  for (let i = 1; i < meanV.length; i += 1) if (meanV[i] < meanV[i - 1] - 1e-12) monotonic = false;
  r.check("X5-1", "熱 → 流体: 温度差が浮力を生み、平均鉛直速度が単調に増える",
    monotonic && meanV.at(-1) > 0.1,
    `平均鉛直速度 ${meanV[0].toExponential(3)} → ${meanV.at(-1).toFixed(4)} m/s(単調 ${monotonic})`);
  await shot("d15-convection");
}

// -------- X6: 熱 → 相変化 → 流体(D18b 氷が水になる)
await page.reload();
await boot(page);
await loadScene(page, "d18b-ice-melts");
await enterPlayPaused(page);
{
  const before = await page.evaluate(() => Number(window.__world.read_component("fluid_particle_count", "")));
  await stepN(page, 10000); // dt=2 ms → 20 s(融解の潜熱を通すのにこれだけ要る)
  const after = await page.evaluate(() => ({
    particles: Number(window.__world.read_component("fluid_particle_count", "")),
    temp: Number(window.__world.read_component("heater_node_temperature", "")),
    t: Number(window.__world.read_component("time", "")),
  }));
  r.check("X6-1", "熱 → 相変化 → 流体: 融けた質量が SPH 粒子として湧く",
    before === 0 && after.particles > 0,
    `粒子数 ${before} → ${after.particles} 個(t = ${after.t.toFixed(1)} s、水槽 T = ${after.temp.toFixed(2)} K)`);
  r.check("X6-2", "融解の潜熱ぶん水槽の温度が下がる",
    after.temp < 350.0 && after.temp > 273.15,
    `T = 350.00 → ${after.temp.toFixed(2)} K`);
  await shot("d18b-ice-melt");
}

// -------- X7: 流体 ↔ 剛体(D14 煙と渦)
await run("d14-vortex", 200);
{
  const ps = await probes(page);
  const rms = byLabel(ps, "GridFluidRmsV").h;
  const x = byLabel(ps, "BodyPosX").h;
  r.check("X7-1", "流体 ↔ 剛体: 障害物まわりに渦が立ち、流体が剛体を押す",
    rms.at(-1) > 0 && Math.abs(x.at(-1) - x[0]) > 1e-4,
    `渦度 rms ${rms[0].toFixed(4)} → 最大 ${Math.max(...rms).toFixed(4)} m/s、障害物 x ${x[0].toFixed(4)} → ${x.at(-1).toFixed(4)} m`);
  await shot("d14-vortex");
}

// -------- X8: 熱 → 力学(D25 ブラウン運動、ゆらぎ散逸定理)
await page.reload();
await boot(page);
await loadScene(page, "d25-brownian");
{
  // 初期位置は原点ではなく 1 mm 間隔の列なので、**変位**を測るために先に控える
  // (Inspector が読むのと同じ `body_position_at_f32` を全ボディについて回す)。
  const positions = () =>
    page.evaluate(() => {
      const w = window.__world;
      const out = [];
      for (let i = 0; i < Number(w.read_component("body_count", "")); i += 1) out.push(Array.from(w.body_position_at_f32(i)));
      return out;
    });
  const p0 = await positions();
  await enterPlayPaused(page);
  await stepN(page, 600);
  const p1 = await positions();
  const { t } = await worldState(page);
  // ゆらぎ散逸定理の検算はエネルギー等分配 ⟨v²⟩ = 3k_BT/m で行う
  // (速度は 10⁻³ m/s オーダーなので f32 でも相対精度が足りる)。
  const v2 = await page.evaluate(() => {
    const w = window.__world;
    let sum = 0;
    for (let i = 0; i < Number(w.read_component("body_count", "")); i += 1) {
      const v = w.body_velocity_at_f32(i);
      sum += v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }
    return sum / Number(w.read_component("body_count", ""));
  });
  const mass = 4.3982297150257095e-15; // シーンの mass_override(ポリスチレン球 1 µm)
  const kT = 1.380649e-23 * 293.15;
  r.check("X8-1", "熱 → 力学: ブラウン力と Stokes 抵抗が等分配則 ⟨v²⟩ = 3k_BT/m で釣り合う",
    rel(v2, (3 * kT) / mass) < 0.1,
    `⟨v²⟩ = ${v2.toExponential(3)} m²/s²(解析 3k_BT/m = ${((3 * kT) / mass).toExponential(3)} m²/s²、比 ${(v2 / ((3 * kT) / mass)).toFixed(3)})`);
  // MSD は UI からは読めない。座標は f32(`body_position_at_f32`)で公開されており、
  // 粒子列の端(x = 0.299 m)での刻みは 3.0e-8 m。ナノメートルの変位はこの量子化
  // ノイズに埋もれる(下の実測が解析解の 4 倍を返すのは、ノイズを測っているため)。
  const msd =
    p1.reduce((sum, p, i) => sum + (p[0] - p0[i][0]) ** 2 + (p[1] - p0[i][1]) ** 2 + (p[2] - p0[i][2]) ** 2, 0) /
    p1.length;
  // Stokes–Einstein: D = kT/(6πηr)、3 次元の MSD = 6Dt。
  const D = (1.380649e-23 * 293.15) / (6 * Math.PI * 0.001002 * 1e-6);
  const analytic = 6 * D * t;
  r.check("X8-2", "アンサンブル MSD が UI から読める(Stokes–Einstein の 6Dt と一致)",
    rel(msd, analytic) < 0.3,
    `MSD(300 粒子, t=${t.toExponential(3)} s) = ${msd.toExponential(3)} m² / 解析 6Dt = ${analytic.toExponential(3)} m² — 変位 ~2 nm に対し f32 座標の刻みは端の粒子で 3.0e-8 m`);
}

// -------- X8b: 流体 × 容器(D23 注ぐ水)
//
// `Coupling` ではなく SPH 内部の固体境界(境界粒子)だが、「水と容器を組み合わせる」
// のは UI から見れば結合と同じ操作なので、ここで一緒に見る。
await run("d23-pouring-water", 2000);
{
  const sph = await page.evaluate(() => {
    const p = Array.from(window.__world.fluid_particle_positions_f32());
    const y = p.filter((_, i) => i % 3 === 1);
    return { n: y.length, below: y.filter((v) => v < -0.1).length, top: Math.max(...y) };
  });
  r.check("X8b-1", "流体 × 容器: 水が境界粒子の床に溜まる(すり抜けない)",
    sph.below === 0,
    `t=1.0 s で ${sph.n} 粒子中 ${sph.below} 個が床(y=0)より 0.1 m 下、最上の粒子でも y = ${sph.top.toFixed(3)} m`);
  await shot("d23-pouring-water");
}

// -------- X9: 力学 ↔ 気体(D17 ピストン)
await run("d17-piston", 400);
{
  const ps = await probes(page);
  const x = byLabel(ps, "BodyPosX").h;
  const speed = byLabel(ps, "BodySpeed").h;
  // 気体が膨張して押す → 速度は増えるが、圧力が下がるので加速度は落ちる。
  const a1 = speed[100] - speed[50];
  const a2 = speed[390] - speed[340];
  r.check("X9-1", "力学 ↔ 気体: 気体がピストンを押し、膨張につれ加速が鈍る",
    x.at(-1) > x[0] && speed.at(-1) > speed[0] && a2 < a1,
    `x ${x[0].toFixed(4)} → ${x.at(-1).toFixed(4)} m、Δv(50→100 step) = ${a1.toFixed(4)} > Δv(340→390 step) = ${a2.toFixed(4)} m/s`);
}

// -------- X10: 静電 → 力学(D26 帯電風船)
await run("d26-balloon", 400);
{
  const ps = await probes(page);
  const x = byLabel(ps, "BodyPosX").h;
  r.check("X10-1", "静電 → 力学: 鏡像力が風船を壁(x=0)へ引き寄せる",
    x[60] < x[0],
    `x ${x[0].toFixed(4)} → ${x[60].toFixed(4)} m(60 step 後)`);
  r.check("X10-2", "風船が壁に貼りつく(すり抜けて発散しない)",
    x.at(-1) > -0.05,
    `x(400 step) = ${x.at(-1).toFixed(3)} m — 壁の剛体がシーンに無いため x=0 を通過し、以後は無重力空間を等速で飛び去る`);
}

// ======================================================== Y. UI からの組み合わせ
//
// -------- Y1: Inspector の Coupling コンポーネント
await page.reload();
await boot(page);
await loadScene(page, "d20-hand-crank-generator");
{
  await page.locator("#hierarchy-tree .tree-body").first().click();
  await page.waitForTimeout(300);
  const text = await page.locator("#inspector-body").textContent();
  r.check("Y1-1", "Inspector が選択した剛体に効いている結合を出す",
    /Coupling/.test(text) && /MotorCoupling/.test(text),
    `Inspector に "${(text.match(/MotorCoupling[^\n]{0,40}/) ?? ["—"])[0].trim()}"`);
  const sceneWide = /シーン全体/.test(text) && /JouleHeat/.test(text);
  r.check("Y1-2", "剛体に紐づかない結合(回路→熱)も「シーン全体」として出る", sceneWide, `シーン全体セクション: ${sceneWide}`);
  await shot("inspector-coupling");
}

// -------- Y2: Settings のヒーター(UI から熱を足す)× 回路→熱シーン
await page.reload();
await boot(page);
await loadScene(page, "d19-electric-workbench");
await enterPlayPaused(page);
{
  await stepN(page, 120);
  const base = await page.evaluate(() => Number(window.__world.read_component("heater_node_temperature", "")));
  await page.locator("#btn-settings").click();
  await page.locator("#toggle-heater").check();
  await page.waitForTimeout(100);
  await stepN(page, 120);
  const heated = await page.evaluate(() => Number(window.__world.read_component("heater_node_temperature", "")));
  const dt = await page.evaluate(() => Number(window.__world.read_component("dt", "")));
  // ヒーターは 2000 W を毎 step 注ぐ(HEATER_WATTS)。C = 1000 J/K。
  const analytic = (2000.0 * 120 * dt) / 1000.0;
  r.check("Y2-1", "Settings のヒーターが、シーン側の結合が使っている熱ノードを実際に温める",
    rel(heated - base, analytic) < 0.02,
    `T ${base.toFixed(3)} → ${heated.toFixed(3)} K(ΔT = ${(heated - base).toFixed(3)} K、解析 ${analytic.toFixed(3)} K)`);
  await shot("settings-heater");
}

// -------- Y3: Settings の重力 × 渦電流ブレーキ(結合系のパラメータ変更)
await page.reload();
await boot(page);
await loadScene(page, "d21-copper-tube-drop");
{
  await page.locator("#btn-settings").click();
  await page.locator("#input-gravity").fill("1.62"); // 月面
  await page.locator("#input-gravity").dispatchEvent("change");
  await page.waitForTimeout(100);
  await enterPlayPaused(page);
  await stepN(page, 600);
  const v = (await probes(page)).find((p) => p.label.includes("BodySpeed")).h.at(-1);
  const { t } = await worldState(page);
  const k = 0.5 ** 2 * 0.1 ** 2 / 1.0;
  const analytic = (0.01 * 1.62 / k) * (1 - Math.exp(-t / (0.01 / k)));
  r.check("Y3-1", "Settings の重力変更が結合系(渦電流ブレーキ)へ正しく伝わる",
    rel(v, analytic) < 0.01,
    `g=1.62 で v(${t.toFixed(3)} s) = ${v.toFixed(4)} m/s(解析 ${analytic.toFixed(4)} m/s)`);
}

// -------- Y4: Circuit タブで回路を組み直す × シーン側の JouleHeat 結合
await page.reload();
await boot(page);
await loadScene(page, "d19-electric-workbench");
{
  await page.locator('.project-tab[data-tab="circuit"]').click();
  await page.waitForTimeout(200);
  // 「リセット(新規回路)」→ 既定 3 ノード(GND 含む)の空回路。
  await page.getByText("リセット(新規回路)").click();
  await page.waitForTimeout(200);
  // 10 V 源(N1 が正極)と 100Ω を張る → P = V²/R = 1 W。
  const addElement = async (a, b, kind, value) => {
    await page.locator("#circuit-editor-node-a").fill(String(a));
    await page.locator("#circuit-editor-node-b").fill(String(b));
    await page.locator("#circuit-editor-kind").selectOption(kind);
    await page.locator("#circuit-editor-value").fill(String(value));
    await page.getByText("素子を追加").click();
    await page.waitForTimeout(150);
  };
  await addElement(1, 0, "voltage_source", 10);
  await addElement(1, 0, "resistor", 100);
  await enterPlayPaused(page);
  const t0 = await page.evaluate(() => Number(window.__world.read_component("heater_node_temperature", "")));
  await stepN(page, 240);
  const t1 = await page.evaluate(() => Number(window.__world.read_component("heater_node_temperature", "")));
  const dt = await page.evaluate(() => Number(window.__world.read_component("dt", "")));
  const nodeText = await page.locator("#circuit-editor-voltages").textContent();
  const analytic = (10.0 ** 2 / 100.0) * 240 * dt / 1000.0;
  r.check("Y4-1", "Circuit タブで組んだ回路の電圧が UI に出る", /Node1: 10\.000V/.test(nodeText ?? ""), `"${(nodeText ?? "").trim()}"`);
  r.check("Y4-2", "UI で組んだ回路の損失を、シーン側の JouleHeat 結合が熱へ渡す",
    rel(t1 - t0, analytic) < 0.02,
    `ΔT = ${(t1 - t0).toFixed(4)} K(解析 P=1 W × ${(240 * dt).toFixed(2)} s / C=1000 J/K = ${analytic.toFixed(4)} K)`);
  await shot("circuit-editor");
}

// -------- Y5: `＋ 追加` の流体を既存シーンへ足す
await page.reload();
await boot(page);
{
  await page.locator("#btn-add").click();
  await page.getByText("＋ 流体 (SPH 水塊)").click();
  await page.waitForTimeout(200);
  const spawned = await page.evaluate(() => Number(window.__world.read_component("fluid_particle_count", "")));
  await enterPlayPaused(page);
  const y0 = await page.evaluate(() => Array.from(window.__world.fluid_particle_positions_f32()).filter((_, i) => i % 3 === 1));
  await stepN(page, 240);
  const y1 = await page.evaluate(() => Array.from(window.__world.fluid_particle_positions_f32()).filter((_, i) => i % 3 === 1));
  const fell = Math.min(...y0) - Math.min(...y1);
  const below = y1.filter((v) => v < -0.5).length;
  r.check("Y5-1", "`＋ 追加 → 流体` で既定シーン(力学)へ SPH ドメインを足せる", spawned === 27, `粒子 ${spawned} 個`);
  r.check("Y5-2", "足した流体が重力を受けて落ち、境界粒子で受け止められる",
    fell > 0 && below === 0,
    `最下粒子 y ${Math.min(...y0).toFixed(3)} → ${Math.min(...y1).toFixed(3)} m、${y1.length} 粒子中 ${below} 個が床(y=−0.1)より 0.5 m 下`);
  await shot("spawn-fluid");
}

// -------- Y6: Project の Import は結合を運ばない(制限の確認)
await page.reload();
await boot(page);
{
  // Project ドロワー Scenes タブの Import(実ファイルを file input へ渡す)。
  await page.locator('.project-tab[data-tab="scenes"]').click();
  await page.waitForTimeout(200);
  const before = await page.evaluate(() => ({
    couplings: Number(window.__world.read_component("coupling_count", "")),
    bodies: Number(window.__world.read_component("body_count", "")),
  }));
  await page.locator("#project-body input[type=file]").setInputFiles(
    new URL("../../../scenes/d10-brake-heat.json", import.meta.url).pathname,
  );
  await page.waitForTimeout(600);
  const after = await page.evaluate(() => ({
    couplings: Number(window.__world.read_component("coupling_count", "")),
    bodies: Number(window.__world.read_component("body_count", "")),
  }));
  r.check("Y6-1", "シーン JSON の Import で剛体は増える",
    after.bodies === before.bodies + 2,
    `body_count = ${before.bodies} → ${after.bodies}`);
  r.check("Y6-2", "Import した JSON の `couplings` セクションも取り込まれる",
    after.couplings > before.couplings,
    `coupling_count = ${before.couplings} → ${after.couplings}(Import は bodies/probes のみを取り込み、couplings・thermal・circuit は捨てる)`);
}

// -------- Y7: Timeline の巻き戻しが結合先ドメインの状態も戻すか
//
// D19(回路 → 熱)は温度が単調に上がり続けるので、巻き戻し先の時刻に対応する
// Probe 履歴の値と突き合わせれば「熱ドメインも一緒に戻ったか」が判定できる。
await run("d19-electric-workbench", 600);
{
  const history = (await probes(page)).find((p) => p.label.includes("NodeTemp")).h;
  const hot = await page.evaluate(() => Number(window.__world.read_component("heater_node_temperature", "")));
  await page.locator("#timeline-scrubber").fill("1"); // fill は input イベントを出す
  await page.waitForTimeout(300);
  const rewound = await page.evaluate(() => ({
    t: Number(window.__world.read_component("time", "")),
    step: Number(window.__world.read_component("step_count", "")),
    temp: Number(window.__world.read_component("heater_node_temperature", "")),
    v: Number(window.__world.read_component("circuit_node_voltage", "3")), // RC 放電の途中電圧
  }));
  const expected = history[rewound.step - 1];
  r.check("Y7-1", "Timeline のスクラブで熱ドメインの状態も一緒に巻き戻る",
    rewound.temp < hot && rel(rewound.temp - 293.15, expected - 293.15) < 1e-6,
    `t = ${rewound.t.toFixed(3)} s(step ${rewound.step})へ巻き戻して T ${hot.toFixed(6)} → ${rewound.temp.toFixed(6)} K(その時刻の Probe 値 ${expected.toFixed(6)} K)`);
  r.check("Y7-2", "回路の動的状態(コンデンサ電圧)も巻き戻る",
    rewound.v > 0.1,
    `巻き戻し後の V₃ = ${rewound.v.toFixed(4)} V(5 s 時点では ${(9 * Math.exp(-5 / 0.5)).toExponential(2)} V まで放電済み)`);
}

// -------- Y8: 結合シーンの決定論
{
  await run("d20-hand-crank-generator", 300);
  const first = await worldState(page);
  await run("d20-hand-crank-generator", 300);
  const second = await worldState(page);
  r.check("Y8-1", "結合シーンもページ再読み込みを跨いでビット一致",
    first.hash === second.hash,
    `${first.hash} = ${second.hash}`);
  const residuals = [];
  for (const [frag, steps] of [["d10-brake-heat", 300], ["d19-electric-workbench", 300], ["d14-vortex", 120]]) {
    await run(frag, steps);
    residuals.push([frag, (await worldState(page)).residual]);
  }
  r.check("Y8-2", "結合を含むシーンでもエネルギー台帳の残差が発散しない",
    residuals.every(([, v]) => Number.isFinite(v) && Math.abs(v) < 1.0),
    residuals.map(([f, v]) => `${f.split("-")[0]} ${v.toExponential(2)}`).join(" / "));
}

console.log(`\nconsole errors/warnings: ${errors.length}`);
for (const e of errors.slice(0, 5)) console.log(`  ${e}`);
const failed = r.summary();
fs.writeFileSync(`${OUT}/coupling-results.json`, JSON.stringify(r.rows, null, 2));
await browser.close();
process.exit(failed > 0 ? 1 : 0);
