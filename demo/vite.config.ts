import { defineConfig } from "vite";

// wasm-pack (--target web) 出力の pkg/ を素の ES モジュールとして import する。
// 生成コードは `new URL("sim_wasm_bg.wasm", import.meta.url)` + fetch で自身の
// .wasm を読み込むため、Vite 標準のアセット解決だけで動作する
// (docs/00-foundation/05-rust-wasm-platform.md §7 は参考実装として
// vite-plugin-wasm を挙げるが、--target web 出力では不要)。
export default defineConfig({
  server: {
    fs: {
      allow: [".."],
    },
  },
});
