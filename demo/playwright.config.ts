import { defineConfig, devices } from "@playwright/test";

// フロントエンドのスモークテスト設定(増分D2)。
//
// **なぜ今まで無かったか**: これまで Playwright での確認は毎増分きちんと行って
// きたが、その場限りのスクリプトを書いて捨てる運用だったため、リポジトリにも CI にも
// 何も残っていなかった(CI の demo ジョブは `npm run build` = `tsc --noEmit` +
// `vite build` のみで、**フロントエンドのテストが1本も無い**状態だった)。
// 本増分でその確認をテストとして資産化する。
//
// `webServer` に `vite preview` を持たせて、テスト実行時に自動でビルド済み成果物を
// 配信する(CI ではその前に `npm run build` が走っている前提)。
export default defineConfig({
  testDir: "./tests",
  fullyParallel: false, // wasm の初期化を含むため直列に回す(実行時間より安定性を優先)。
  forbidOnly: !!process.env.CI,
  retries: 0,
  workers: 1,
  reporter: process.env.CI ? "list" : "html",
  timeout: 60_000,
  use: {
    baseURL: "http://127.0.0.1:4173",
    trace: "on-first-retry",
  },
  projects: [
    {
      name: "chromium",
      use: {
        ...devices["Desktop Chrome"],
        // 開発コンテナには Chromium が事前インストール済みだが、`@playwright/test`
        // が要求するビルド番号と一致しないことがある(実測: 1.62 はビルド1234を
        // 要求するのに対し環境にあるのは1194)。その場合は
        // `PLAYWRIGHT_CHROMIUM_PATH` に実行ファイルを指定すればそちらを使う。
        // CI では `npx playwright install --with-deps chromium` が正しいビルドを
        // 用意するので、この環境変数は設定しない(=既定の解決に任せる)。
        launchOptions: process.env.PLAYWRIGHT_CHROMIUM_PATH
          ? { executablePath: process.env.PLAYWRIGHT_CHROMIUM_PATH }
          : {},
      },
    },
  ],
  webServer: {
    command: "npx vite preview --port 4173 --strictPort",
    url: "http://127.0.0.1:4173",
    reuseExistingServer: !process.env.CI,
    timeout: 60_000,
  },
});
