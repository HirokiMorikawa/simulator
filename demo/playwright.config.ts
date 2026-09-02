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
    // **既定は統合エディタ(pro)で開く**。かんたんモード(`src/guided.ts`)を
    // 入れた際、初回訪問者は 3 ステップのチューザから始まるようにしたため、
    // 統合エディタを検証する既存のテスト群はモードを明示しておく必要がある
    // (実ユーザーも一度「くわしい編集画面へ」を押せば以後はこの状態になる)。
    // かんたんモード側のテストは `guided.spec.ts` が `test.use` で空の
    // storageState を指定し、**初めて開いた人**の状態を再現する。
    storageState: {
      cookies: [],
      origins: [
        {
          origin: "http://127.0.0.1:4173",
          localStorage: [{ name: "simulator.ui.mode", value: "pro" }],
        },
      ],
    },
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
    // **`--host 127.0.0.1` を明示するのが要点**。`vite preview` の既定ホストは
    // `localhost` で、GitHub Actions のランナーではこれが IPv6 (`::1`) に解決
    // される一方、Playwright は上の `url`(IPv4 の 127.0.0.1)を叩きに行くため
    // 到達できず `Timed out waiting 60000ms from config.webServer` で落ちる
    // (ローカルの開発コンテナでは `localhost` が IPv4 に解決されるため再現せず、
    // CI で初めて表面化した)。バインド先を明示して両者を一致させる。
    command: "npx vite preview --port 4173 --strictPort --host 127.0.0.1",
    url: "http://127.0.0.1:4173",
    reuseExistingServer: !process.env.CI,
    timeout: 60_000,
    // 起動に失敗したときに原因がログへ出るようにする(上記の不一致を突き止める
    // のに実際に必要だった)。
    stdout: "pipe",
    stderr: "pipe",
  },
});
