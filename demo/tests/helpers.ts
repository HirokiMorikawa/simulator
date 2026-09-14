import { inflateSync } from "node:zlib";
import { expect, type Page } from "@playwright/test";

// スモークテスト・受け入れテストで共有するヘルパ(元は smoke.spec.ts 内に
// あったが、Playwright は *.spec.ts 同士の import を許さない
// 「should not import test file」ため、テストファイルではないこのモジュールへ
// 切り出した)。

/** ページ全体で発生した未捕捉例外を集める。favicon の 404 は除外する。 */
export function collectPageErrors(page: Page): string[] {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  return errors;
}

/** wasm 初期化を待つ(Hierarchy にボディが並ぶまで)。 */
export async function waitForWorld(page: Page) {
  await expect(page.locator("#hierarchy-tree .tree-selectable").first()).toBeVisible({
    timeout: 30_000,
  });
}

/**
 * ツールバーの「＋ 追加」メニューから項目を選ぶ(群2)。
 * スポーン系8個のボタンはツールバーを3行ぶんの高さに膨らませていたため
 * 1つのメニューへ畳んだ。個々のボタンは `hidden` で DOM に残してあるが、
 * **`hidden` な要素は Playwright からクリックできない**ので、テストも
 * 実ユーザーと同じくメニュー経由で操作する。
 */
export async function addViaMenu(page: Page, label: string) {
  await page.click("#btn-add");
  await page.locator("#context-menu button", { hasText: label }).first().click();
}

/**
 * PNG(8bit、非インターレース、RGB/RGBA)を素の画素配列へ復号する。
 *
 * **なぜ自前で持つか**: 「回っていることが画面から読み取れるか」(課題②、
 * 手回し発電機のクランク)を数値で押さえるには、キャンバスのスクリーン
 * ショットから実際のピクセルを読む必要がある。WebGL キャンバスは
 * `preserveDrawingBuffer`を立てていないため、ページ内で`getImageData`
 * しても描画済みの中身は読めない(実測: 中心ピクセルが常に黒 (0,0,0) に
 * なった)——Playwright の`locator.screenshot()`はブラウザの合成結果を
 * 直接キャプチャするため、そちらは正しく読める。だが依存に`pngjs`等は
 * このリポジトリに入っていない(`node_modules`を実測で確認)ので、
 * Node標準の`zlib`だけで最小限のPNGデコーダを持つ。Chromeの
 * `screenshot()`が出すPNG(パレット無し・インターレース無し・8bit)の
 * 範囲だけを扱う——汎用PNGデコーダを目指さない。
 */
export function decodePng(buf: Buffer): {
  width: number;
  height: number;
  /** 行優先のRGBA(各8bit)。 */
  data: Uint8Array;
} {
  let offset = 8; // 先頭8バイトのPNGシグネチャは読み飛ばす。
  let width = 0;
  let height = 0;
  let colorType = 0;
  const idatChunks: Buffer[] = [];
  while (offset < buf.length) {
    const len = buf.readUInt32BE(offset);
    const type = buf.toString("ascii", offset + 4, offset + 8);
    const data = buf.subarray(offset + 8, offset + 8 + len);
    if (type === "IHDR") {
      width = data.readUInt32BE(0);
      height = data.readUInt32BE(4);
      const bitDepth = data[8];
      colorType = data[9];
      const interlace = data[12];
      if (bitDepth !== 8) throw new Error(`decodePng: unsupported bit depth ${bitDepth}`);
      if (interlace !== 0) throw new Error("decodePng: interlaced PNG not supported");
    } else if (type === "IDAT") {
      idatChunks.push(Buffer.from(data));
    } else if (type === "IEND") {
      break;
    }
    offset += 8 + len + 4;
  }
  const channels = colorType === 6 ? 4 : colorType === 2 ? 3 : null;
  if (channels === null) throw new Error(`decodePng: unsupported color type ${colorType}`);
  const raw = inflateSync(Buffer.concat(idatChunks));
  const stride = width * channels;
  const out = new Uint8Array(width * height * 4);
  let prevRow = new Uint8Array(stride);
  let pos = 0;
  for (let y = 0; y < height; y++) {
    const filterType = raw[pos];
    pos += 1;
    const row = raw.subarray(pos, pos + stride);
    pos += stride;
    const curRow = new Uint8Array(stride);
    for (let x = 0; x < stride; x++) {
      const a = x >= channels ? curRow[x - channels] : 0;
      const b = prevRow[x];
      const c = x >= channels ? prevRow[x - channels] : 0;
      let value = row[x];
      switch (filterType) {
        case 0:
          break;
        case 1:
          value = (value + a) & 0xff;
          break;
        case 2:
          value = (value + b) & 0xff;
          break;
        case 3:
          value = (value + ((a + b) >> 1)) & 0xff;
          break;
        case 4: {
          const p = a + b - c;
          const pa = Math.abs(p - a);
          const pb = Math.abs(p - b);
          const pc = Math.abs(p - c);
          value = (value + (pa <= pb && pa <= pc ? a : pb <= pc ? b : c)) & 0xff;
          break;
        }
        default:
          throw new Error(`decodePng: unsupported filter type ${filterType}`);
      }
      curRow[x] = value;
    }
    for (let x = 0; x < width; x++) {
      const si = x * channels;
      const di = (y * width + x) * 4;
      out[di] = curRow[si];
      out[di + 1] = curRow[si + 1];
      out[di + 2] = curRow[si + 2];
      out[di + 3] = channels === 4 ? curRow[si + 3] : 255;
    }
    prevRow = curRow;
  }
  return { width, height, data: out };
}
