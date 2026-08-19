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

    step(1, 6, "C コンパイラを確認");
    ensure_c_toolchain()?;

    step(2, 6, "wasm32 ターゲットを確認");
    ensure_wasm_target();

    step(3, 6, "wasm-pack を確認");
    ensure_wasm_pack()?;

    step(4, 6, "物理コアを WebAssembly へビルド");
    build_wasm()?;

    step(5, 6, "Node.js を確認");
    ensure_node()?;

    step(6, 6, "デモの依存パッケージを取得");
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

/// C コンパイラ(リンカ)の有無を確認し、無ければ導入する。
///
/// **なぜ必要か**: Rust はリンクに C ツールチェーンを使うので、これが無いと
/// `cargo install wasm-pack` が `linker 'cc' not found` で落ちる。cargo が出す
/// 最後の行は `could not compile ...` という要約でしかなく、**何が足りないのか
/// も、どう直すのかも書かれていない**。クリーンな PC で最初に踏むのがここ
/// なので、着手前に検知して OS ごとの具体的なコマンドまで示す。
///
/// **なぜ黙って入れないのか**: Linux では導入に root 権限が要る。セットアップが
/// 無断で `sudo` を打つのは、たとえ善意でもツールとして信用できない挙動になる。
/// そこで「検知 → 正確なコマンドを提示 → 同意を取って実行」に留める。利用者から
/// 見れば `y` を打つだけで済み、権限昇格は必ず本人の意思を経る。
///
/// Windows は対象外(下記 `c_compiler_present` 参照)。
fn ensure_c_toolchain() -> Result<(), String> {
    if c_compiler_present() {
        println!("  C コンパイラを検出しました");
        return Ok(());
    }

    let Some(installer) = c_toolchain_installer() else {
        return Err(
            "C コンパイラが見つかりません。Rust はリンクに C ツールチェーンを使うため、\n\
             お使いのディストリビューションの方法で導入してください\n\
             (gcc または clang と、標準 C ライブラリの開発パッケージ)"
                .to_string(),
        );
    };

    println!("  C コンパイラが見つかりません。Rust のリンクに必要です");
    println!("  次のコマンドで導入できます:\n");
    println!("      {}\n", installer.command.join(" "));

    if !can_prompt() {
        return Err(format!(
            "上のコマンドを実行してから `cargo xtask setup` をやり直してください\n\
             (対話端末ではないため、ここでは自動実行しません)\n\n{}",
            installer.note
        ));
    }

    if !ask_yes_no("  今すぐ実行しますか?")? {
        return Err(format!(
            "上のコマンドを実行してから `cargo xtask setup` をやり直してください\n\n{}",
            installer.note
        ));
    }

    let args: Vec<&str> = installer.command[1..].iter().map(|s| s.as_str()).collect();
    run(&installer.command[0], &args, &repo_root())?;

    if !c_compiler_present() {
        return Err(format!(
            "導入後も C コンパイラを検出できませんでした。\n{}",
            installer.note
        ));
    }
    println!("  C コンパイラを導入しました");
    Ok(())
}

/// C コンパイラが使えるか。
///
/// Windows は判定しない(常に `true` を返す)。MSVC のリンカは PATH ではなく
/// レジストリや vswhere 経由で rustc が見つけるため、PATH を探る素朴な判定では
/// **入っているのに「無い」と誤報する**。誤報はこの機能の目的(詰まりを減らす)
/// に反するので、Windows では検知せず、失敗時の案内文で補う
/// (`ensure_wasm_pack` のエラーメッセージ参照)。
fn c_compiler_present() -> bool {
    if cfg!(windows) {
        return true;
    }
    ["cc", "gcc", "clang"]
        .iter()
        .any(|p| probe(p, &["--version"]).is_some())
}

/// C ツールチェーンの導入コマンドと補足。
struct CToolchainInstaller {
    command: Vec<String>,
    note: String,
}

/// 実行中の OS / ディストリビューションから導入コマンドを組み立てる。
fn c_toolchain_installer() -> Option<CToolchainInstaller> {
    let words = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<String>>();

    if cfg!(target_os = "macos") {
        return Some(CToolchainInstaller {
            // Command Line Tools の導入。sudo は要らないが GUI のダイアログが出る。
            command: words(&["xcode-select", "--install"]),
            note: "ダイアログが出たら「インストール」を選び、完了してから \
                   `cargo xtask setup` をやり直してください。"
                .to_string(),
        });
    }

    // /etc/os-release の ID / ID_LIKE からパッケージマネージャを決める
    // (ID が派生ディストリ名でも ID_LIKE に本家が入るため両方見る)。
    let release = std::fs::read_to_string("/etc/os-release").ok()?;
    let field = |key: &str| -> String {
        release
            .lines()
            .find_map(|line| line.strip_prefix(key))
            .unwrap_or("")
            .trim_matches(['"', '\''])
            .to_lowercase()
    };
    let ids = format!("{} {}", field("ID="), field("ID_LIKE="));
    let has = |name: &str| ids.split_whitespace().any(|id| id == name);

    let command = if has("debian") || has("ubuntu") {
        words(&["sudo", "apt-get", "install", "-y", "build-essential"])
    } else if has("fedora") || has("rhel") || has("centos") {
        words(&["sudo", "dnf", "install", "-y", "gcc"])
    } else if has("arch") {
        words(&[
            "sudo",
            "pacman",
            "-S",
            "--needed",
            "--noconfirm",
            "base-devel",
        ])
    } else if has("alpine") {
        words(&["sudo", "apk", "add", "build-base"])
    } else if has("suse") || has("opensuse") {
        words(&["sudo", "zypper", "install", "-y", "gcc"])
    } else {
        return None;
    };

    Some(CToolchainInstaller {
        command,
        note: "root 権限が必要なため、パスワードを求められることがあります。".to_string(),
    })
}

/// 対話的に尋ねてよいか(端末に繋がっていて、CI でもない)。
fn can_prompt() -> bool {
    use std::io::IsTerminal;
    std::io::stdin().is_terminal()
        && std::io::stdout().is_terminal()
        && std::env::var_os("CI").is_none()
}

/// y/N を尋ねる。既定は「いいえ」(空 Enter は実行しない)。
fn ask_yes_no(question: &str) -> Result<bool, String> {
    use std::io::Write;
    print!("{question} [y/N] ");
    std::io::stdout()
        .flush()
        .map_err(|e| format!("標準出力に書けませんでした: {e}"))?;
    let mut answer = String::new();
    std::io::stdin()
        .read_line(&mut answer)
        .map_err(|e| format!("入力を読めませんでした: {e}"))?;
    Ok(matches!(answer.trim().to_lowercase().as_str(), "y" | "yes"))
}

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
    .map_err(|e| {
        // ここで落ちる最頻の原因は C ツールチェーンの欠落である。Unix なら
        // `ensure_c_toolchain` が事前に弾いているが、Windows は誤報を避けて
        // 検知していない(同関数の doc 参照)ので、案内はここで補う。
        let toolchain_hint = if cfg!(windows) {
            "\n\nビルドが `link.exe` や `could not compile` で落ちている場合、\
             C ツールチェーンが入っていない可能性が高い。Visual Studio Build Tools の\n\
             「C++ によるデスクトップ開発」を導入してから再実行する。"
        } else {
            ""
        };
        format!(
            "{e}\n\n手動で導入する場合: \
             cargo install wasm-pack --version {WASM_PACK_VERSION} --locked{toolchain_hint}"
        )
    })?;

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
