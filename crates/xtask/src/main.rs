//! 開発用タスクランナー(xtask パターン)。
//!
//! クリーンな PC からの初期セットアップを1コマンドに集約する。make / just /
//! シェルスクリプトのような追加ツールを要求せず、既に必須である Rust だけで
//! Linux / macOS / Windows のいずれでも同じコマンドが動くことを狙っている。
//!
//! ```text
//! cargo xtask setup       セットアップ一式(wasm-pack導入 → wasmビルド → npm ci)
//! cargo xtask build-wasm  物理コアを WebAssembly へビルドし demo/pkg に出力
//! cargo xtask dev         ブラウザデモを起動する(未セットアップなら自動で補う)
//! cargo xtask check       CI と同じチェック(fmt / clippy / test)
//! ```

use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Stdio};

/// 導入する wasm-pack のバージョン。環境ごとに挙動が変わるのを防ぐため固定する。
/// 更新する場合は CI(.github/workflows/ci.yml)側の検証も合わせて確認すること。
const WASM_PACK_VERSION: &str = "0.13.1";

/// 物理コアのビルドターゲット。`rust-toolchain.toml` でも宣言しており、
/// rustup 経由であれば利用者の手動追加は不要になる。
const WASM_TARGET: &str = "wasm32-unknown-unknown";

fn main() -> ExitCode {
    let task = std::env::args().nth(1);
    let result = match task.as_deref() {
        Some("setup") => setup(),
        Some("build-wasm") => build_wasm(),
        Some("dev") => dev(),
        Some("check") => check(),
        Some(other) => Err(format!("未知のサブコマンド `{other}` です。\n{}", usage())),
        None => {
            println!("{}", usage());
            return ExitCode::SUCCESS;
        }
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("\nエラー: {message}");
            ExitCode::FAILURE
        }
    }
}

fn usage() -> String {
    [
        "使い方: cargo xtask <サブコマンド>",
        "",
        "  setup       セットアップ一式(wasm-pack導入 → wasmビルド → npm ci)",
        "  build-wasm  物理コアを WebAssembly へビルドし demo/pkg に出力",
        "  dev         ブラウザデモを起動する(未セットアップなら自動で補う)",
        "  check       CI と同じチェック(fmt / clippy / test)",
    ]
    .join("\n")
}

// ---------------------------------------------------------------------------
// サブコマンド
// ---------------------------------------------------------------------------

/// クリーンな環境からブラウザデモが起動できる状態までを一括で用意する。
fn setup() -> Result<(), String> {
    println!("セットアップを開始します(初回は wasm-pack のビルドで数分かかります)");

    step(1, 5, "wasm32 ターゲットを確認");
    ensure_wasm_target();

    step(2, 5, "wasm-pack を確認");
    ensure_wasm_pack()?;

    step(3, 5, "物理コアを WebAssembly へビルド");
    build_wasm()?;

    step(4, 5, "Node.js を確認");
    ensure_node()?;

    step(5, 5, "デモの依存パッケージを取得");
    npm(&["ci"], &demo_dir())?;

    println!("\nセットアップが完了しました。次のコマンドでデモを起動できます:");
    println!("\n    cargo xtask dev\n");
    Ok(())
}

/// 物理コアを WebAssembly へビルドして `demo/pkg` に出力する。
///
/// 出力先の `demo/pkg` は .gitignore 済みで、`demo/src/main.ts` がここを直接
/// import する。そのためデモをビルド・起動する前に必ず通しておく必要がある。
fn build_wasm() -> Result<(), String> {
    ensure_wasm_pack()?;
    run(
        wasm_pack_command(),
        &[
            "build",
            "crates/sim-wasm",
            "--target",
            "web",
            "--out-dir",
            "../../demo/pkg",
        ],
        &repo_root(),
    )
}

/// ブラウザデモを起動する。未セットアップの場合は不足分だけ補ってから起動する。
fn dev() -> Result<(), String> {
    if !repo_root().join("demo/pkg/sim_wasm.js").is_file() {
        println!("demo/pkg が未生成のため、先に WebAssembly をビルドします");
        build_wasm()?;
    }
    if !demo_dir().join("node_modules").is_dir() {
        println!("node_modules が未取得のため、先に npm ci を実行します");
        ensure_node()?;
        npm(&["ci"], &demo_dir())?;
    }
    npm(&["run", "dev"], &demo_dir())
}

/// CI の native ジョブと同じ静的チェックとテストを走らせる。
fn check() -> Result<(), String> {
    let root = repo_root();
    run("cargo", &["fmt", "--all", "--", "--check"], &root)?;
    run(
        "cargo",
        &[
            "clippy",
            "--workspace",
            "--all-targets",
            "--",
            "-D",
            "warnings",
        ],
        &root,
    )?;
    run("cargo", &["test", "--workspace"], &root)
}

// ---------------------------------------------------------------------------
// 前提ツールの確認と導入
// ---------------------------------------------------------------------------

/// wasm32 ターゲットの導入を試みる。
///
/// `rust-toolchain.toml` の `targets` により rustup 経由なら既に入っているが、
/// 宣言が効かない構成(ディストリ配布の Rust など)への保険として明示的にも叩く。
/// rustup が無い環境は正常な構成のひとつなので、失敗しても警告に留める。
fn ensure_wasm_target() {
    if which("rustup").is_none() {
        println!("  rustup が見つかりません。{WASM_TARGET} が導入済みであることを確認してください");
        return;
    }
    if run("rustup", &["target", "add", WASM_TARGET], &repo_root()).is_err() {
        println!("  警告: {WASM_TARGET} の追加に失敗しました。既に導入済みなら問題ありません");
    }
}

/// wasm-pack が無ければ、バージョンを固定して導入する。
///
/// 公式の `curl | sh` インストーラは Windows で実行できないため、全OSで同じ
/// コマンドが通る `cargo install` を採用する。管理者権限も不要。
fn ensure_wasm_pack() -> Result<(), String> {
    if let Some(version) = probe(wasm_pack_command(), &["--version"]) {
        println!("  {} を検出しました", version.trim());
        return Ok(());
    }

    println!("  wasm-pack が見つかりません。v{WASM_PACK_VERSION} を導入します");
    println!("  (ソースからのビルドのため数分かかります)");
    run(
        "cargo",
        &[
            "install",
            "wasm-pack",
            "--version",
            WASM_PACK_VERSION,
            "--locked",
        ],
        &repo_root(),
    )
    .map_err(|e| format!("{e}\n\n手動で導入する場合: cargo install wasm-pack --version {WASM_PACK_VERSION} --locked"))?;

    if probe(wasm_pack_command(), &["--version"]).is_none() {
        return Err("wasm-pack の導入後も実行できません。cargo のインストール先\
             (通常 ~/.cargo/bin、Windows では %USERPROFILE%\\.cargo\\bin)が\
             PATH に含まれているか確認してください"
            .to_string());
    }
    Ok(())
}

/// Node.js の存在を確認する。デモのビルドと起動に必須。
fn ensure_node() -> Result<(), String> {
    match probe(node_command(), &["--version"]) {
        Some(version) => {
            println!("  Node.js {} を検出しました", version.trim());
            Ok(())
        }
        None => Err(
            "Node.js が見つかりません。22 以上を導入してください(https://nodejs.org/)。\n\
             nvm を使っている場合はリポジトリ直下で `nvm use` が使えます"
                .to_string(),
        ),
    }
}

// ---------------------------------------------------------------------------
// プロセス実行のヘルパ
// ---------------------------------------------------------------------------

/// リポジトリのルート。
///
/// `cargo xtask` はカレントディレクトリを変えずに起動するため、コンパイル時に
/// 確定する xtask 自身の位置(`<root>/crates/xtask`)から2つ遡って求める。
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .expect("crates/xtask の2つ上がリポジトリルートである")
        .to_path_buf()
}

fn demo_dir() -> PathBuf {
    repo_root().join("demo")
}

/// Windows では wasm-pack / node は実行可能ファイルなのでそのまま起動できる。
fn wasm_pack_command() -> &'static str {
    "wasm-pack"
}

fn node_command() -> &'static str {
    "node"
}

/// npm を実行する。
///
/// Windows の npm は `npm.cmd` というバッチファイルであり、CreateProcess は
/// PATHEXT による解決を行わないため `Command::new("npm")` は「プログラムが
/// 見つからない」で失敗する。cmd 経由で起動することで全OSの差を吸収する。
fn npm(args: &[&str], cwd: &Path) -> Result<(), String> {
    if cfg!(windows) {
        let mut full = vec!["/C", "npm"];
        full.extend_from_slice(args);
        run("cmd", &full, cwd)
    } else {
        run("npm", args, cwd)
    }
}

/// 子プロセスを実行し、失敗したらその旨をエラーとして返す。標準出力は素通しする。
fn run(program: &str, args: &[&str], cwd: &Path) -> Result<(), String> {
    println!("  $ {program} {}", args.join(" "));
    let status = Command::new(program)
        .args(args)
        .current_dir(cwd)
        .status()
        .map_err(|e| format!("`{program}` を起動できませんでした: {e}"))?;

    if status.success() {
        Ok(())
    } else {
        Err(format!("`{program} {}` が失敗しました", args.join(" ")))
    }
}

/// バージョン問い合わせなど、出力を捨てて成否だけ見たいコマンドを実行する。
/// コマンド自体が存在しない場合も `None` を返す。
fn probe(program: &str, args: &[&str]) -> Option<String> {
    let output = Command::new(program)
        .args(args)
        .stderr(Stdio::null())
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    String::from_utf8(output.stdout).ok()
}

/// コマンドが PATH 上に存在するかを、実際に起動してみることで確かめる。
fn which(program: &str) -> Option<String> {
    probe(program, &["--version"])
}

fn step(current: usize, total: usize, label: &str) {
    println!("\n[{current}/{total}] {label}");
}
