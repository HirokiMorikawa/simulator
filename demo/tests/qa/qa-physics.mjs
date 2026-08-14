// 物理法則の検証(UI 操作経由)。docs/reviews/2026-08-04-editor-qa.md §3。
//
// 判定基準は docs/21-verification/03-demo-scenarios.md の合格基準に合わせる。
// 値は必ず「シーンをギャラリーから読み込み → ⏭ で step を進め → Probe 履歴 /
// ハッシュ表示から読む」経路で取る。
import fs from "fs";
import { launch, boot, results, loadScene, enterPlayPaused, stepN, probes, worldState, OUT } from "./qa-lib.mjs";

const G = 9.80665;
fs.mkdirSync(OUT, { recursive: true });
const { browser, page, errors } = await launch();
const r = results();
await boot(page);

// ---------------------------------------------------------------- 前提
// 以降の測定はすべて「停止状態からちょうど N step」に依存する。まずそれを確かめる。
await loadScene(page, "d1-free-fall");
const gate = await enterPlayPaused(page);
r.check("P0-1", "Play 移行直後の一時停止で step が 0 のまま", gate.step === 0,
  `step=${gate.step}, 再生ボタン="${gate.playButton}"`);
await stepN(page, 240);
let s = await worldState(page);
r.check("P0-2", "⏭ が要求ちょうどの step を進める", s.step === 240, `step=${s.step}(要求 240)`);

// ---------------------------------------------------------------- D1 自由落下(M1)
await stepN(page, 122); // t ≒ 3.017 s まで
s = await worldState(page);
let p = await probes(page);
const yEnd = p[0].h[p[0].h.length - 1];
const yExact = 20 - 0.5 * G * s.t * s.t;
r.check("P1-1", "D1 自由落下 y(t) = y₀ − ½gt²", Math.abs(yEnd - yExact) / 20 < 0.01,
  `t=${s.t.toFixed(4)} s: 実測 ${yEnd.toFixed(4)} m / 解析 ${yExact.toFixed(4)} m(誤差 ${(Math.abs(yEnd - yExact) / 20 * 100).toFixed(2)} % of h)`);

// 落下時間は y が 0 を跨ぐ step を線形補間して求める(履歴は容量 600 以内)。
const yh = p[0].h;
let tFall = null;
for (let i = 1; i < yh.length; i += 1) {
  if (yh[i] <= 0 && yh[i - 1] > 0) {
    tFall = (i - 1 + yh[i - 1] / (yh[i - 1] - yh[i])) * s.dt;
    break;
  }
}
const tExact = Math.sqrt((2 * 20) / G);
r.check("P1-2", "D1 落下時間 t = √(2h/g)", tFall !== null && Math.abs(tFall - tExact) / tExact < 0.01,
  `実測 ${tFall?.toFixed(4)} s / 解析 ${tExact.toFixed(4)} s(誤差 ${((tFall - tExact) / tExact * 100).toFixed(2)} %)`);

// ---------------------------------------------------------------- D3 反発(M6)
// **560 step に抑えるのが要点**: dt=1/240 で履歴容量 600 を超えると先頭が
// 失われ、「第1バウンド」だと思って読んだ極大が実は数回あとのものになる。
await loadScene(page, "d3-bounce");
await enterPlayPaused(page);
await stepN(page, 560);
s = await worldState(page);
p = await probes(page);
const RADIUS = 0.1;
const E_RUBBER = 0.8; // ゴム(天然)の反発係数(MaterialDb)
const peaks = [];
for (let i = 1; i < p[0].h.length - 1; i += 1) {
  if (p[0].h[i] > p[0].h[i - 1] && p[0].h[i] >= p[0].h[i + 1]) peaks.push(p[0].h[i] - RADIUS);
}
const ratio1 = peaks[0] / (2.0 - RADIUS);
r.check("P2-1", "D3 第1バウンドの高さ比 = e²", Math.abs(ratio1 - E_RUBBER ** 2) < 0.06,
  `落下 ${(2.0 - RADIUS).toFixed(2)} m → ${peaks[0]?.toFixed(4)} m、比 ${ratio1.toFixed(4)}(実効 e=${Math.sqrt(ratio1).toFixed(3)} / 材料値 ${E_RUBBER})`);
const ratio2 = peaks[1] / peaks[0];
r.check("P2-2", "D3 第2バウンドの高さ比 = e²", Math.abs(ratio2 - E_RUBBER ** 2) < 0.06,
  `比 ${ratio2?.toFixed(4)}(実効 e=${Math.sqrt(ratio2).toFixed(3)})`);

// ---------------------------------------------------------------- D5 斜面(M7/M8)
await loadScene(page, "d5-incline-static");
await enterPlayPaused(page);
await stepN(page, 600);
p = await probes(page);
const vStatic = p[0].h[p[0].h.length - 1];
r.check("P3-1", "D5 10° 斜面で静止(tanθ < μs)", Math.abs(vStatic) < 0.05, `5 s 後の速さ ${vStatic.toExponential(3)} m/s`);

await loadScene(page, "d5-incline-slide");
await enterPlayPaused(page);
await stepN(page, 600);
p = await probes(page);
const vh = p[0].h;
const vSlide = vh[vh.length - 1];
r.check("P3-2", "D5 45° 斜面で滑り出す(tanθ > μs)", vSlide > 0.5, `5 s 後の速さ ${vSlide.toFixed(4)} m/s`);
const aMeasured = (vh[vh.length - 1] - vh[Math.floor(vh.length / 2)]) / 2.5;
r.check("P3-3", "D5 45° の加速度が g(sinθ − μk cosθ) の範囲", aMeasured > 0 && aMeasured < G * Math.SQRT1_2,
  `実測 a ≈ ${aMeasured.toFixed(3)} m/s²(摩擦なし ${(G * Math.SQRT1_2).toFixed(3)} / μk=0.5 で ${(G * Math.SQRT1_2 * 0.5).toFixed(3)})`);

// ---------------------------------------------------------------- D11 単振り子(M3)
await loadScene(page, "d11-pendulum");
await enterPlayPaused(page);
await stepN(page, 600);
s = await worldState(page);
p = await probes(page);
const px = (p.find((q) => /x/i.test(q.label)) ?? p[0]).h;
const crossings = [];
for (let i = 1; i < px.length; i += 1) {
  if (px[i - 1] > 0 && px[i] <= 0) crossings.push((i - 1 + px[i - 1] / (px[i - 1] - px[i])) * s.dt);
}
const periods = crossings.slice(1).map((v, i) => v - crossings[i]);
const Tmeasured = periods.reduce((a, b) => a + b, 0) / Math.max(periods.length, 1);
const Texact = 2 * Math.PI * Math.sqrt(1.0 / G);
r.check("P4-1", "D11 単振り子の周期 T = 2π√(L/g)", Math.abs(Tmeasured - Texact) / Texact < 0.02,
  `実測 ${Tmeasured.toFixed(4)} s(${periods.length} 周期平均)/ 解析 ${Texact.toFixed(4)} s(誤差 ${((Tmeasured - Texact) / Texact * 100).toFixed(2)} %)`);
const amplitude = Math.max(...px.map(Math.abs));
r.check("P4-2", "D11 振幅が減衰しない(保存系)", Math.abs(amplitude - 0.04998) / 0.04998 < 0.05,
  `振幅 ${amplitude.toFixed(5)} m(初期 0.04998 m)、energy_residual = ${s.residual.toExponential(3)}`);
r.check("P9-1", "D11 エネルギー台帳の residual が小さい", Math.abs(s.residual) < 1e-3,
  `residual = ${s.residual.toExponential(3)}`);

// ---------------------------------------------------------------- D34 ケプラー(A1/A2)
await loadScene(page, "d34-solar-system-single-planet");
await enterPlayPaused(page);
await stepN(page, 600);
s = await worldState(page);
p = await probes(page);
const [ax, ay] = [p[0].h, p[1].h];
const radii = ax.map((v, i) => Math.hypot(v, ay[i]));
const [rMin, rMax] = [Math.min(...radii), Math.max(...radii)];
r.check("P5-1", "D34 円軌道の半径が一定", (rMax - rMin) / rMax < 0.01,
  `r = ${(rMin / 1.496e11).toFixed(5)}〜${(rMax / 1.496e11).toFixed(5)} au(変動 ${((rMax - rMin) / rMax * 100).toFixed(3)} %)`);
// **周期は「掃過角」から出す**。600 step では 1 周に届かず、履歴容量 600 の
// 制約でそれ以上の step は添字と時刻の対応が崩れるため、ゼロ交差では測れない。
let swept = 0;
for (let i = 1; i < ax.length; i += 1) {
  let d = Math.atan2(ay[i], ax[i]) - Math.atan2(ay[i - 1], ax[i - 1]);
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  swept += d;
}
const yearExact = 3.15576e7;
const Torbit = (2 * Math.PI * s.t) / swept;
r.check("P5-2", "D34 公転周期がケプラー第3法則どおり", Math.abs(Torbit - yearExact) / yearExact < 0.02,
  `${s.t.toExponential(3)} s で ${swept.toFixed(4)} rad 掃過 → T = ${(Torbit / 86400).toFixed(2)} 日 / 理論 ${(yearExact / 86400).toFixed(2)} 日(誤差 ${((Torbit - yearExact) / yearExact * 100).toFixed(2)} %)`);

// ---------------------------------------------------------------- D30 気体(S1/S2)
await loadScene(page, "d30-gas-box");
await enterPlayPaused(page);
await stepN(page, 600);
p = await probes(page);
const T = p[0].h[p[0].h.length - 1];
const P = p[1].h[p[1].h.length - 1];
const Pideal = (400 * 1.380649e-23 * T) / 1e-21;
r.check("P8-1", "D30 気体 p = NkT/V", Math.abs(P - Pideal) / Pideal < 0.15,
  `T=${T.toFixed(2)} K: 実測 p=${P.toExponential(4)} Pa / 理想気体 ${Pideal.toExponential(4)} Pa(差 ${((P - Pideal) / Pideal * 100).toFixed(2)} %)`);

// ---------------------------------------------------------------- 決定論(D8)
async function runHash(scene, n) {
  await loadScene(page, scene);
  await enterPlayPaused(page);
  await stepN(page, n);
  const st = await worldState(page);
  return { ...st, uiHash: await page.locator("#hash-display").getAttribute("title") };
}
const runA = await runHash("d8-scatter", 600);
const runB = await runHash("d8-scatter", 600);
r.check("P6-1", "決定論: 同シーン同 step で state_hash 一致", runA.hash === runB.hash && runA.step === runB.step,
  `A(step=${runA.step})=${runA.hash} / B(step=${runB.step})=${runB.hash}`);
r.check("P6-2", "ツールバーのハッシュ表示が World と一致", runA.uiHash === runA.hash, `UI=${runA.uiHash} / world=${runA.hash}`);
await page.reload();
await boot(page);
const runC = await runHash("d8-scatter", 600);
r.check("P6-3", "決定論: ページ再読み込みを跨いでも一致", runC.hash === runA.hash, `再読込後=${runC.hash} / 初回=${runA.hash}`);

// ---------------------------------------------------------------- Settings の重力
await loadScene(page, "d1-free-fall");
await page.locator("#btn-mode-edit").click();
await page.locator("#btn-settings").click();
await page.locator("#input-gravity").fill("1.62"); // 月面
await page.locator("#input-gravity").dispatchEvent("change");
await page.waitForTimeout(200);
await page.locator("#btn-settings").click();
const gravity = await page.evaluate(() => Number(window.__world.read_component("gravity", "")));
await enterPlayPaused(page);
await stepN(page, 240);
s = await worldState(page);
p = await probes(page);
const yMoon = p[0].h[p[0].h.length - 1];
const yMoonExact = 20 - 0.5 * 1.62 * s.t * s.t;
r.check("P7-1", "Settings の重力変更が物理に効く(月 1.62)", Math.abs(gravity - 1.62) < 1e-9 && Math.abs(yMoon - yMoonExact) < 0.2,
  `world.gravity=${gravity}, t=${s.t.toFixed(3)} s: 実測 ${yMoon.toFixed(4)} m / 解析 ${yMoonExact.toFixed(4)} m`);

console.log("\n===== ブラウザコンソール =====");
console.log([...new Set(errors)].join("\n") || "(なし)");
const failed = r.summary();
await browser.close();
process.exit(failed > 0 ? 1 : 0);
