//! 生状態スナップショット(`scenario`モジュールdoc §「生状態スナップショット
//! (`raw_state`)」)の**数値配列**を、base64(RFC 4648 §4、標準アルファベット
//! `A-Za-z0-9+/`、`=`パディングあり)+リトルエンディアン生バイトで表すための
//! 符号化ユーティリティ。設計 docs/20-integration/04-world-api.md §3。
//!
//! # なぜ生バイトなのか
//!
//! 現在の`raw_state`はどのドメインも**JSONの数値配列**(`Vec<f64>`・
//! `Vec<[f64; 3]>`・`Vec<bool>`・`Vec<i8>`)で書いている。これには2つの代償がある。
//!
//! 1. **f64の往復がJSONパーサの丸め挙動に依存する**。`serde_json`の既定の
//!    floatパーサはbest-effortで**1 ULPずれることがある**ため、`sim-world`と
//!    `sim-wasm`は`float_roundtrip`フィーチャを明示的に有効化している
//!    (両`Cargo.toml`のコメント参照)。つまり現状の往復の厳密性は
//!    「パーサがそう振る舞ってくれること」に**依存**している。
//!    バイト列を直接載せる表現なら、厳密性は*構成上*保証される——10進テキストを
//!    経由しない以上、丸めが介在する余地がそもそも無い。`float_roundtrip`は
//!    全ドメインの移行が終わった時点で外せる。
//! 2. **時間発展した配列では小さくなる**。SPHの粒子座標・量子の波動関数サンプル・
//!    イジングのスピン格子は要素数が万の単位になりうる。base64は1要素あたり
//!    **10.67バイト固定**(8バイト → 4/3倍)である一方、10進表現は
//!    `float_roundtrip`が要求する有効17桁ぶんの桁数を値に応じて使う。
//!    実測(f64 2万要素、`float_roundtrip`有効):
//!
//!    | データの性格 | 10進JSON | base64 | 比 |
//!    |---|---|---|---|
//!    | 汎用の17桁級 | 18.7 B/要素 | 10.67 B/要素 | 1.75倍**小** |
//!    | SPH座標風 | 18.9 | 10.67 | 1.78倍**小** |
//!    | 波動関数風(指数つき) | 21.6 | 10.67 | 2.02倍**小** |
//!    | 0.5刻みの「綺麗な」値 | 4.8 | 10.67 | 2.2倍**大** |
//!
//!    **「常に3倍縮む」ような単純な話ではない**ことを明記しておく。縮むのは
//!    **時間発展して仮数が埋まった配列**であって、初期化直後のゼロ埋めや粗い刻みの
//!    配列(`smoke_density`の初期値、波が来る前の`hx`/`hy`など)はむしろ**膨らむ**。
//!    実用上は前者が効く——`raw_state`をエクスポートする必要が生じるのは
//!    そもそも時間発展した状態だからである——が、サイズだけを根拠にするなら
//!    2倍弱が現実的な期待値である。**主たる動機は 1.(厳密性)のほうにある**。
//!
//!    なお`bool`のビット詰め(1要素1ビット)は話が別で、`[true,false,...]`という
//!    10進表現に対して**おおむね35倍**縮む。ここは無条件に効く。
//!
//! # なぜ base64 を自前で書くのか(外部クレートを足さない判断)
//!
//! README §「依存が実質ゼロ」が掲げるとおり、このワークスペースの外部依存は
//! 物理コアの`serde`/`serde_json`、WASM境界の`wasm-bindgen`/`js-sys`、ベンチの
//! `criterion`のみで、`Cargo.lock`に**符号化系のクレートは1つも無い**。
//!
//! `base64`クレートは小さく枯れてはいるが、**同じ判断はこのリポジトリで既に
//! 一度下されている**——`sim_render::png`は画像出力のために`png`/`flate2`を
//! 足さず、CRC32とdeflate(LZ77 + 固定ハフマン符号)を自前で書いた
//! (`sim_render::png`のモジュールdoc参照)。base64はそれより桁違いに小さい
//! 仕事である:符号表64エントリと「3バイト → 4文字」の詰め替えだけで、
//! 分岐らしい分岐は末尾のパディング処理しかない。**deflateを自前で書く規律の
//! もとで、base64のために依存を1つ増やすのは筋が通らない**。よってここも自前で書く。
//!
//! 自前実装ゆえの縮約は**しない**:符号化・復号とも RFC 4648 §4 に完全準拠する。
//! むしろ復号は仕様より**厳格**にしてある(次節)。
//!
//! # 復号を厳格にしてある理由
//!
//! 復号は以下をすべて誤りとして弾く。RFC 4648 §3.5 が実装依存としている
//! 「非正準な符号語」まで拒否する:
//!
//! - 文字列長が4の倍数でない / アルファベット外の文字が混ざる
//! - `=`が最終ブロックの3文字目・4文字目以外に現れる、`=`の後ろに実データが続く
//! - **捨てられるビットが0でない**(例: `"AA=="`は正準だが`"AB=="`は同じ1バイトへ
//!   復号できてしまう非正準形)。空白・改行も受け付けない。
//!
//! 厳格にするのは、この符号がまさに**ビット単位の同一性**のために導入されるからで
//! ある。同じバイト列に対する表現が一意でないと「エクスポート → JSON → 再インポート
//! → 再エクスポート」がテキストとして安定せず、`state_hash`の一致を診断する足場が
//! 揺らぐ。緩く受理して黙って直すより、書き手の壊れを即座に露出させるほうがよい。
//!
//! # NaN の扱い
//!
//! **符号化そのものはNaNを拒否しない**——というより、この表現は
//! `f64::NAN`・`f64::INFINITY`・`-0.0`・非正規化数をすべて**ビットパターンごと
//! 厳密に往復させる**(JSONの数値表現では`NaN`/`Inf`はそもそも書けず、
//! `serde_json`は`null`へ潰してしまう)。この点でも生バイトのほうが素直である。
//!
//! ただし**`state_hash`が覆う状態にNaNが legitimately 入ることは無い**。
//! 生状態にNaNが現れたなら、それは符号化の問題ではなく**シミュレーションが
//! 発散した**という上流の異常である。そこで方針を次の2本立てにした:
//!
//! - [`encode_f64_le_base64`] は**全域関数**(`String`を必ず返す)。低レベルの
//!   詰め替えとして値の意味を判定しない。NaNが来ればNaNのビットを忠実に書く。
//! - [`encode_f64_le_base64_finite`] は非有限値を [`RawBytesError::NonFiniteValue`]
//!   として**拒否する**検査付きの版。将来ドメインを移行する側は、発散を静かに
//!   ファイルへ焼き付けてしまわないよう**こちらを使うことを推奨する**。
//!
//! 復号側は常に全域(非有限値も素通しで復元する)。既に書かれたファイルを
//! 読めなくしても救いにならないためである。
//!
//! # このモジュールの適用範囲(**まだどのドメインにも配線していない**)
//!
//! ここにあるのは符号化・復号の**土台だけ**である。11ドメインの`raw_state`は
//! 現時点では従来どおりJSONの数値配列のままで、1バイトも変えていない。
//! 実際の移行(各`*RawStateJson`のフィールドを`Vec<f64>`から`String`へ替え、
//! `scenes/*.json`を全件書き換える)は**意図的に別作業として切ってある**——
//! 全ドメインのスキーマと既存シーン資産に同時に触る大きな変更であり、
//! この土台の正しさとは独立に検証されるべきだからである。
//!
//! 移行後の姿の素描(`GridFluidRawStateJson`の速度場を例に):
//!
//! ```text
//! pub struct GridFluidRawStateJson {
//!     /// 速度場(長さ`nx*ny`、行優先 `i + nx*j`)を
//!     /// `raw_bytes::encode_f64_le_base64`で符号化した文字列。
//!     pub u: String,
//!     pub v: String,
//!     /// 固体セルかどうか(長さ`nx*ny`)。1セル1ビットへ詰める。
//!     pub solid_cells: String,
//!     ...
//! }
//! ```
//!
//! 実際に往復するコード例は本モジュールのテスト
//! `usage_example_grid_fluid_raw_state`(`#[cfg(test)]`、説明用であって
//! 本番のスキーマではない)を参照。
//!
//! # 対応する要素型
//!
//! 11ドメインの`*RawStateJson`が実際に使っている要素型だけを用意してある
//! (使われていない型の符号化器は作らない):
//!
//! | 要素型 | 符号化 | 主な使用箇所 |
//! |---|---|---|
//! | `f64` | 8バイトLE | `u`/`v`/`w`/`ez`/`hx`/`hy`/`psi_re`/`psi_im`/`temperature`/`density` 等 |
//! | `[f64; 3]` | 24バイトLE(x,y,zの順) | `position`/`velocity`/`solid_velocity`/`boundary_position` |
//! | `bool` | **1ビット**(8個/バイト、長さ前置) | `GridFluid{,3D}RawStateJson::solid_cells` |
//! | `i8` | 1バイト | `IsingRawStateJson::spins` |

/// base64標準アルファベット(RFC 4648 §4 の表1)。
const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// `f64`1個のバイト数。
const F64_SIZE: usize = 8;
/// `[f64; 3]`1個のバイト数。
const VEC3_SIZE: usize = 24;
/// bool配列の先頭に置く要素数ヘッダ(`u64`LE)のバイト数。
const BOOL_HEADER_SIZE: usize = 8;

/// 復号(および検査付き符号化)の失敗。
///
/// **`SceneError`には意図的に依存しない**。このモジュールは`scenario`より下の
/// 低レベルな詰め替え層であり、シーン読み込み以外からも使える独立した部品に
/// しておきたいためである。シーン読み込み経路の呼び出し側が
/// `SceneError::InvalidValue(format!("...: {e}"))`のように包み直す
/// (`sim_world::scenario`が`Coupling::restore_raw_state`のエラーを包むのと同じ形)。
/// `Eq`は導出しない——[`RawBytesError::NonFiniteValue`]が`f64`を持つため
/// (`SceneError`と同じ`Clone, Debug, PartialEq`に揃えてある)。
#[derive(Clone, Debug, PartialEq)]
pub enum RawBytesError {
    /// base64文字列の長さが4の倍数でない。
    InvalidBase64Length(usize),
    /// アルファベット外の文字(空白・改行を含む)。`index`は文字列先頭からの
    /// バイト位置。
    InvalidBase64Char { index: usize, byte: u8 },
    /// `=`の位置が不正(最終ブロックの3・4文字目以外、または`=`の後ろに実データ)。
    InvalidBase64Padding,
    /// 最終ブロックで**捨てられるビットが0でない**非正準な符号語
    /// (モジュールdoc §「復号を厳格にしてある理由」)。
    NonCanonicalBase64,
    /// 復号したバイト列長が要素サイズの倍数でない。
    UnalignedByteLength { len: usize, element_size: usize },
    /// bool配列の長さヘッダ(先頭8バイト)が入りきっていない。
    MissingBoolHeader(usize),
    /// bool配列の長さヘッダが、実際に続くビットマップの長さと矛盾している。
    BoolLengthMismatch { declared: usize, payload_len: usize },
    /// bool配列の長さヘッダが`usize`に収まらない(32bit環境=wasm32での防御)。
    BoolCountOverflow(u64),
    /// bool配列の最終バイトの**未使用ビットが0でない**(非正準)。
    NonCanonicalBoolPadding,
    /// [`encode_f64_le_base64_finite`]に非有限値(NaN / ±Inf)が渡された。
    NonFiniteValue { index: usize, value: f64 },
}

impl std::fmt::Display for RawBytesError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RawBytesError::InvalidBase64Length(len) => {
                write!(f, "base64文字列長{len}が4の倍数でない")
            }
            RawBytesError::InvalidBase64Char { index, byte } => write!(
                f,
                "base64アルファベット外の文字 '{}' (0x{byte:02x}) が位置{index}にある",
                char::from(*byte).escape_default()
            ),
            RawBytesError::InvalidBase64Padding => {
                write!(f, "base64のパディング'='の位置が不正")
            }
            RawBytesError::NonCanonicalBase64 => {
                write!(f, "base64最終ブロックの余剰ビットが0でない(非正準な符号語)")
            }
            RawBytesError::UnalignedByteLength { len, element_size } => write!(
                f,
                "復号したバイト列長{len}が要素サイズ{element_size}の倍数でない"
            ),
            RawBytesError::MissingBoolHeader(len) => write!(
                f,
                "bool配列の長さヘッダ({BOOL_HEADER_SIZE}バイト)が無い(復号長{len})"
            ),
            RawBytesError::BoolLengthMismatch {
                declared,
                payload_len,
            } => write!(
                f,
                "bool配列の宣言長{declared}要素に対しビットマップが{payload_len}バイトある\
                 (期待{}バイト)",
                declared.div_ceil(8)
            ),
            RawBytesError::BoolCountOverflow(n) => {
                write!(f, "bool配列の宣言長{n}がこの環境のusizeに収まらない")
            }
            RawBytesError::NonCanonicalBoolPadding => {
                write!(f, "bool配列の最終バイトの未使用ビットが0でない(非正準)")
            }
            RawBytesError::NonFiniteValue { index, value } => {
                write!(f, "非有限値{value}がindex {index}にある(発散の疑い)")
            }
        }
    }
}

impl std::error::Error for RawBytesError {}

// ---------------------------------------------------------------------------
// base64 そのもの(RFC 4648 §4)
// ---------------------------------------------------------------------------

/// 任意のバイト列をbase64(標準アルファベット・`=`パディングあり)へ符号化する。
pub fn encode_base64(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        // 3バイトを24bitへ詰めて6bitずつ4文字に割る。端数のバイトは0で埋めた上で
        // 対応する文字を'='に置き換える(埋めた0は下の非正準判定と整合する)。
        let b0 = u32::from(chunk[0]);
        let b1 = u32::from(chunk.get(1).copied().unwrap_or(0));
        let b2 = u32::from(chunk.get(2).copied().unwrap_or(0));
        let triple = (b0 << 16) | (b1 << 8) | b2;

        out.push(char::from(ALPHABET[(triple >> 18) as usize & 0x3F]));
        out.push(char::from(ALPHABET[(triple >> 12) as usize & 0x3F]));
        if chunk.len() > 1 {
            out.push(char::from(ALPHABET[(triple >> 6) as usize & 0x3F]));
        } else {
            out.push('=');
        }
        if chunk.len() > 2 {
            out.push(char::from(ALPHABET[triple as usize & 0x3F]));
        } else {
            out.push('=');
        }
    }
    out
}

/// base64の1文字を6bit値へ写す(アルファベット外なら`None`)。
fn decode_symbol(byte: u8) -> Option<u8> {
    match byte {
        b'A'..=b'Z' => Some(byte - b'A'),
        b'a'..=b'z' => Some(byte - b'a' + 26),
        b'0'..=b'9' => Some(byte - b'0' + 52),
        b'+' => Some(62),
        b'/' => Some(63),
        _ => None,
    }
}

/// base64文字列をバイト列へ復号する。仕様より厳格に検査する
/// (モジュールdoc §「復号を厳格にしてある理由」)。
pub fn decode_base64(s: &str) -> Result<Vec<u8>, RawBytesError> {
    let bytes = s.as_bytes();
    if !bytes.len().is_multiple_of(4) {
        return Err(RawBytesError::InvalidBase64Length(bytes.len()));
    }
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3);
    for (block, chunk) in bytes.chunks(4).enumerate() {
        let is_last = (block + 1) * 4 == bytes.len();
        let mut sextets = [0u8; 4];
        let mut pad = 0usize;
        for (i, &byte) in chunk.iter().enumerate() {
            if byte == b'=' {
                // '='は最終ブロックの3・4文字目にしか立てない。
                if !is_last || i < 2 {
                    return Err(RawBytesError::InvalidBase64Padding);
                }
                pad += 1;
            } else {
                // "A=B="のように'='の後ろへ実データが続く形は不正。
                if pad > 0 {
                    return Err(RawBytesError::InvalidBase64Padding);
                }
                sextets[i] = decode_symbol(byte).ok_or(RawBytesError::InvalidBase64Char {
                    index: block * 4 + i,
                    byte,
                })?;
            }
        }
        // 捨てられるビットが0でない非正準形を弾く。pad=1なら3文字目の下位2bit、
        // pad=2なら2文字目の下位4bitが出力バイトに現れない。
        let non_canonical = match pad {
            1 => sextets[2] & 0x03 != 0,
            2 => sextets[1] & 0x0F != 0,
            _ => false,
        };
        if non_canonical {
            return Err(RawBytesError::NonCanonicalBase64);
        }

        let triple = (u32::from(sextets[0]) << 18)
            | (u32::from(sextets[1]) << 12)
            | (u32::from(sextets[2]) << 6)
            | u32::from(sextets[3]);
        let keep = 3 - pad;
        out.push((triple >> 16) as u8);
        if keep > 1 {
            out.push((triple >> 8) as u8);
        }
        if keep > 2 {
            out.push(triple as u8);
        }
    }
    Ok(out)
}

/// 復号したバイト列長が要素サイズの倍数であることを確かめる。
fn check_alignment(len: usize, element_size: usize) -> Result<(), RawBytesError> {
    if len.is_multiple_of(element_size) {
        Ok(())
    } else {
        Err(RawBytesError::UnalignedByteLength { len, element_size })
    }
}

// ---------------------------------------------------------------------------
// f64
// ---------------------------------------------------------------------------

/// `f64`配列を「各要素8バイトLE」→ base64 で符号化する(**全域関数**)。
///
/// NaN・±Inf・`-0.0`・非正規化数もビットパターンごと厳密に往復する。
/// 発散した状態を静かに書き出したくない呼び出し側は
/// [`encode_f64_le_base64_finite`]を使うこと(モジュールdoc §「NaN の扱い」)。
pub fn encode_f64_le_base64(values: &[f64]) -> String {
    let mut bytes = Vec::with_capacity(values.len() * F64_SIZE);
    for v in values {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    encode_base64(&bytes)
}

/// [`encode_f64_le_base64`]の検査付きの版。非有限値(NaN / ±Inf)が1つでもあれば
/// [`RawBytesError::NonFiniteValue`]を返す。
///
/// `state_hash`が覆う生状態にNaNが legitimately 入ることは無いので、
/// ドメイン移行側はこちらを使うのが既定であるべきである。
pub fn encode_f64_le_base64_finite(values: &[f64]) -> Result<String, RawBytesError> {
    if let Some((index, value)) = values.iter().enumerate().find(|(_, v)| !v.is_finite()) {
        return Err(RawBytesError::NonFiniteValue {
            index,
            value: *value,
        });
    }
    Ok(encode_f64_le_base64(values))
}

/// [`encode_f64_le_base64`]の逆。バイト列長が8の倍数であることを検査する。
pub fn decode_f64_le_base64(s: &str) -> Result<Vec<f64>, RawBytesError> {
    let bytes = decode_base64(s)?;
    check_alignment(bytes.len(), F64_SIZE)?;
    Ok(bytes
        .chunks_exact(F64_SIZE)
        .map(|c| f64::from_le_bytes(c.try_into().expect("chunks_exactが8バイトを保証する")))
        .collect())
}

// ---------------------------------------------------------------------------
// [f64; 3]
// ---------------------------------------------------------------------------

/// `[f64; 3]`配列を「1要素24バイト(x, y, zの順に8バイトLE)」→ base64 で
/// 符号化する。`position`/`velocity`のような3次元ベクトル列のための版。
///
/// 平坦化した`f64`列として[`encode_f64_le_base64`]に渡すのと**バイト列は同一**だが、
/// 復号時に3要素境界で長さを検証できる(24の倍数でなければ弾ける)ぶんこちらが安全。
pub fn encode_vec3_le_base64(values: &[[f64; 3]]) -> String {
    let mut bytes = Vec::with_capacity(values.len() * VEC3_SIZE);
    for v in values {
        for component in v {
            bytes.extend_from_slice(&component.to_le_bytes());
        }
    }
    encode_base64(&bytes)
}

/// [`encode_vec3_le_base64`]の逆。バイト列長が24の倍数であることを検査する。
pub fn decode_vec3_le_base64(s: &str) -> Result<Vec<[f64; 3]>, RawBytesError> {
    let bytes = decode_base64(s)?;
    check_alignment(bytes.len(), VEC3_SIZE)?;
    Ok(bytes
        .chunks_exact(VEC3_SIZE)
        .map(|c| {
            let mut v = [0.0f64; 3];
            for (slot, comp) in v.iter_mut().zip(c.chunks_exact(F64_SIZE)) {
                *slot =
                    f64::from_le_bytes(comp.try_into().expect("chunks_exactが8バイトを保証する"));
            }
            v
        })
        .collect())
}

// ---------------------------------------------------------------------------
// bool(ビット詰め)
// ---------------------------------------------------------------------------

/// `bool`配列を**1要素1ビット**(8個/バイト、各バイト内は下位ビットが先)へ詰めて
/// base64で符号化する。`solid_cells`のような格子1セル1boolの配列で効く
/// ——1バイト1boolでは10進JSON(`true,`で5バイト)の5倍しか縮まないが、
/// ビット詰めなら**40倍**縮む。
///
/// **先頭に要素数を`u64`LEで前置する**。ビット詰めは要素数を復元できない
/// (末尾バイトの余りビットと本来の値が区別できない)ためで、これが無いと
/// 復号側に`nx*ny`を外から渡してもらう必要が出る。このモジュールの他の復号器は
/// すべて文字列だけから要素数を復元できるので、**そこに揃えた**。
/// 8バイトの定数オーバーヘッドは、この符号化が効く規模の配列では無視できる。
pub fn encode_bool_bitpacked_base64(values: &[bool]) -> String {
    let n = values.len();
    let mut bytes = Vec::with_capacity(BOOL_HEADER_SIZE + n.div_ceil(8));
    bytes.extend_from_slice(&(n as u64).to_le_bytes());

    let mut acc = 0u8;
    for (i, &b) in values.iter().enumerate() {
        if b {
            acc |= 1 << (i % 8);
        }
        if i % 8 == 7 {
            bytes.push(acc);
            acc = 0;
        }
    }
    if !n.is_multiple_of(8) {
        bytes.push(acc);
    }
    encode_base64(&bytes)
}

/// [`encode_bool_bitpacked_base64`]の逆。長さヘッダとビットマップ長の整合、
/// および末尾バイトの未使用ビットが0であること(正準性)を検査する。
pub fn decode_bool_bitpacked_base64(s: &str) -> Result<Vec<bool>, RawBytesError> {
    let bytes = decode_base64(s)?;
    if bytes.len() < BOOL_HEADER_SIZE {
        return Err(RawBytesError::MissingBoolHeader(bytes.len()));
    }
    let (header, payload) = bytes.split_at(BOOL_HEADER_SIZE);
    let declared = u64::from_le_bytes(header.try_into().expect("split_atが8バイトを保証する"));
    // wasm32(usize=32bit)で宣言長が溢れる壊れた入力への防御。
    let n = usize::try_from(declared).map_err(|_| RawBytesError::BoolCountOverflow(declared))?;

    // 先に長さを検証してから確保する(壊れたヘッダで巨大確保を起こさないため)。
    if payload.len() != n.div_ceil(8) {
        return Err(RawBytesError::BoolLengthMismatch {
            declared: n,
            payload_len: payload.len(),
        });
    }
    if !n.is_multiple_of(8) {
        let used = n % 8;
        let last = payload[payload.len() - 1];
        if last >> used != 0 {
            return Err(RawBytesError::NonCanonicalBoolPadding);
        }
    }

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        out.push((payload[i / 8] >> (i % 8)) & 1 == 1);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// i8
// ---------------------------------------------------------------------------

/// `i8`配列を1要素1バイトでbase64符号化する(`IsingRawStateJson::spins`用)。
///
/// `i8`は既に1バイトなのでエンディアンの選択余地は無い。ビット詰めにしない理由は、
/// スピンが`±1`の2値であっても**型としてはi8の全域を取りうる**ためで、
/// 「2値だから1ビット」という仮定を符号化層に埋め込むと不変条件が破れたときに
/// 静かに壊れる(2値性の保証は`sim_statistical::IsingSim`側の責務)。
pub fn encode_i8_base64(values: &[i8]) -> String {
    let bytes: Vec<u8> = values.iter().map(|&v| v as u8).collect();
    encode_base64(&bytes)
}

/// [`encode_i8_base64`]の逆。1バイト1要素なので長さ検査は不要
/// (どのバイト列も妥当な`i8`列である)。
pub fn decode_i8_base64(s: &str) -> Result<Vec<i8>, RawBytesError> {
    Ok(decode_base64(s)?.into_iter().map(|b| b as i8).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ビットパターンでの一致を主張する(数値比較ではNaNが常に不一致になり、
    /// `-0.0 == 0.0`が真になってしまう。この符号の要点はビット単位の同一性である)。
    fn assert_bits_eq(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len(), "要素数が違う");
        for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
            assert_eq!(
                a.to_bits(),
                e.to_bits(),
                "index {i}: {a:?}(0x{:016x}) != {e:?}(0x{:016x})",
                a.to_bits(),
                e.to_bits()
            );
        }
    }

    // -- base64 そのもの ---------------------------------------------------

    /// RFC 4648 §10 のテストベクタ。自前実装が標準準拠であることの根拠。
    #[test]
    fn rfc4648_test_vectors() {
        let cases: &[(&str, &str)] = &[
            ("", ""),
            ("f", "Zg=="),
            ("fo", "Zm8="),
            ("foo", "Zm9v"),
            ("foob", "Zm9vYg=="),
            ("fooba", "Zm9vYmE="),
            ("foobar", "Zm9vYmFy"),
        ];
        for (plain, encoded) in cases {
            assert_eq!(
                &encode_base64(plain.as_bytes()),
                encoded,
                "符号化: {plain:?}"
            );
            assert_eq!(
                decode_base64(encoded).unwrap(),
                plain.as_bytes(),
                "復号: {encoded:?}"
            );
        }
    }

    /// 全256バイト値と、3で割った余りが0/1/2の全長さで往復する。
    #[test]
    fn base64_roundtrip_all_byte_values_and_lengths() {
        let all: Vec<u8> = (0..=255u8).collect();
        for len in 0..=all.len() {
            let src = &all[..len];
            assert_eq!(
                decode_base64(&encode_base64(src)).unwrap(),
                src,
                "len={len}"
            );
        }
    }

    #[test]
    fn base64_rejects_bad_length() {
        for bad in ["A", "AB", "ABC", "ABCDE"] {
            assert_eq!(
                decode_base64(bad),
                Err(RawBytesError::InvalidBase64Length(bad.len())),
                "{bad:?}"
            );
        }
    }

    #[test]
    fn base64_rejects_invalid_characters() {
        // '-'/'_' はURL-safeアルファベット。標準アルファベットでは受け付けない。
        assert_eq!(
            decode_base64("Zm9-"),
            Err(RawBytesError::InvalidBase64Char {
                index: 3,
                byte: b'-'
            })
        );
        // 空白・改行も受け付けない(モジュールdoc参照)。
        assert!(matches!(
            decode_base64("Zm9 "),
            Err(RawBytesError::InvalidBase64Char { .. })
        ));
        assert!(matches!(
            decode_base64("Zm9v\nZm9v"),
            Err(RawBytesError::InvalidBase64Length(_))
        ));
    }

    #[test]
    fn base64_rejects_misplaced_padding() {
        // 最終でないブロックのパディング。
        assert_eq!(
            decode_base64("Zg==Zg=="),
            Err(RawBytesError::InvalidBase64Padding)
        );
        // 1・2文字目のパディング。
        assert_eq!(
            decode_base64("=g=="),
            Err(RawBytesError::InvalidBase64Padding)
        );
        assert_eq!(
            decode_base64("Z==="),
            Err(RawBytesError::InvalidBase64Padding)
        );
        // '='の後ろに実データ。
        assert_eq!(
            decode_base64("Zm=v"),
            Err(RawBytesError::InvalidBase64Padding)
        );
    }

    #[test]
    fn base64_rejects_non_canonical_symbols() {
        // "Zg=="は正準("f")。"Zh=="は捨てられる2bitが0でないので拒否。
        assert_eq!(decode_base64("Zg==").unwrap(), b"f");
        assert_eq!(
            decode_base64("Zh=="),
            Err(RawBytesError::NonCanonicalBase64)
        );
        // "Zm8="は正準("fo")。"Zm9="は捨てられる4bitが0でないので拒否。
        assert_eq!(decode_base64("Zm8=").unwrap(), b"fo");
        assert_eq!(
            decode_base64("Zm9="),
            Err(RawBytesError::NonCanonicalBase64)
        );
    }

    // -- f64 ---------------------------------------------------------------

    #[test]
    fn f64_roundtrip_empty_and_single() {
        assert_eq!(encode_f64_le_base64(&[]), "");
        assert_bits_eq(&decode_f64_le_base64("").unwrap(), &[]);

        let single = [core::f64::consts::PI];
        assert_bits_eq(
            &decode_f64_le_base64(&encode_f64_le_base64(&single)).unwrap(),
            &single,
        );
    }

    /// ビットパターンが効く値をまとめて往復させる。**数値比較ではなくビット比較**で
    /// 主張するのが要点(`-0.0`は`0.0`と数値的に等しく、NaNは自分自身とも等しくない)。
    #[test]
    fn f64_roundtrip_preserves_exact_bit_patterns() {
        let tricky = [
            0.0,
            -0.0,
            1.0,
            -1.0,
            f64::MIN,
            f64::MAX,
            f64::MIN_POSITIVE,                     // 最小の正規化数
            f64::from_bits(1),                     // 最小の非正規化数
            f64::from_bits(0x000F_FFFF_FFFF_FFFF), // 非正規化数
            f64::EPSILON,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
            -f64::NAN,
            f64::from_bits(0x7FF0_0000_0000_0001), // シグナリングNaN
            core::f64::consts::PI,
            // `float_roundtrip`が無いと1 ULPずれる実例(sim-worldのCargo.tomlのコメント)。
            0.906_310_649_862_746_3,
        ];
        let decoded = decode_f64_le_base64(&encode_f64_le_base64(&tricky)).unwrap();
        assert_bits_eq(&decoded, &tricky);

        // 負のゼロが「符号ビットごと」戻ることを名指しで主張する。
        let neg_zero = decode_f64_le_base64(&encode_f64_le_base64(&[-0.0])).unwrap();
        assert_eq!(neg_zero[0].to_bits(), (-0.0f64).to_bits());
        assert_ne!(neg_zero[0].to_bits(), 0.0f64.to_bits());
    }

    /// 大きい配列(10000要素超)の往復。
    #[test]
    fn f64_roundtrip_large_array() {
        let values: Vec<f64> = (0..12_345)
            .map(|i| (i as f64) * 0.123_456_789_012_345_6 - 500.0)
            .collect();
        let encoded = encode_f64_le_base64(&values);
        assert_bits_eq(&decode_f64_le_base64(&encoded).unwrap(), &values);
    }

    /// サイズ特性の確認。**base64は1要素10.67バイト固定**なので、10進表現に対して
    /// 縮むか膨らむかは値の性格で決まる(モジュールdoc §「なぜ生バイトなのか」の表)。
    /// この非対称性は移行時にサイズだけを根拠にしないための歯止めとして明示しておく。
    #[test]
    fn f64_size_versus_decimal_json_depends_on_the_data() {
        let json_len = |v: &[f64]| serde_json::to_string(v).unwrap().len();

        // 仮数が埋まった「時間発展後」の配列は縮む(実測1.75倍前後)。
        let evolved: Vec<f64> = (0..20_000)
            .map(|i| (i as f64) * 0.123_456_789_012_345_6 - 500.0)
            .collect();
        let evolved_b64 = encode_f64_le_base64(&evolved).len();
        assert!(
            evolved_b64 * 3 < json_len(&evolved) * 2,
            "1.5倍以上の縮小を期待: base64 {evolved_b64} vs JSON {}",
            json_len(&evolved)
        );

        // 一方、粗い刻みの「綺麗な」値ではbase64のほうが大きい。
        let coarse: Vec<f64> = (0..20_000).map(|i| (i % 100) as f64 * 0.5).collect();
        let coarse_b64 = encode_f64_le_base64(&coarse).len();
        assert!(
            coarse_b64 > json_len(&coarse),
            "粗い値ではbase64が膨らむはず: base64 {coarse_b64} vs JSON {}",
            json_len(&coarse)
        );

        // どちらの場合もビット厳密性は変わらない(サイズと正しさは独立)。
        assert_bits_eq(
            &decode_f64_le_base64(&encode_f64_le_base64(&coarse)).unwrap(),
            &coarse,
        );
    }

    #[test]
    fn f64_rejects_unaligned_byte_length() {
        // 7バイト = base64 12文字。8の倍数でないので弾かれる。
        let seven = encode_base64(&[0u8; 7]);
        assert_eq!(
            decode_f64_le_base64(&seven),
            Err(RawBytesError::UnalignedByteLength {
                len: 7,
                element_size: 8
            })
        );
        // base64として壊れている入力もそのまま伝播する。
        assert!(matches!(
            decode_f64_le_base64("!!!!"),
            Err(RawBytesError::InvalidBase64Char { .. })
        ));
    }

    #[test]
    fn f64_finite_encoder_rejects_non_finite() {
        assert!(encode_f64_le_base64_finite(&[1.0, 2.0, -3.5]).is_ok());
        // NaNは`assert_eq!`で比べられない(NaN != NaN)ので値ごと分解して見る。
        // 下の`non_finite_error_equality_follows_f64_partial_eq_for_nan`参照。
        match encode_f64_le_base64_finite(&[1.0, f64::NAN]) {
            Err(RawBytesError::NonFiniteValue { index, value }) => {
                assert_eq!(index, 1);
                assert!(value.is_nan());
            }
            other => panic!("NaNが拒否されなかった: {other:?}"),
        }
        assert_eq!(
            encode_f64_le_base64_finite(&[f64::INFINITY]),
            Err(RawBytesError::NonFiniteValue {
                index: 0,
                value: f64::INFINITY
            })
        );
        // 最初に見つかった非有限値の位置を報告する。
        assert!(matches!(
            encode_f64_le_base64_finite(&[0.0, f64::NEG_INFINITY, f64::NAN]),
            Err(RawBytesError::NonFiniteValue { index: 1, .. })
        ));
        // 全域版はNaNを受け入れ、ビットごと往復させる(モジュールdoc §「NaN の扱い」)。
        let via_total = decode_f64_le_base64(&encode_f64_le_base64(&[f64::NAN])).unwrap();
        assert!(via_total[0].is_nan());
    }

    /// 導出した`PartialEq`は`f64`の`PartialEq`をそのまま使うので、
    /// `NonFiniteValue`が`value: NaN`を持つとき**自分自身とも等しくならない**。
    /// 上の`f64_finite_encoder_rejects_non_finite`が`assert_eq!`で通るのは
    /// `f64::NAN`同士が等しいからではなく、`assert_eq!`の左右が
    /// **同じ判定経路を通っていない**——実際にはNaNの枝は`matches!`で見ている
    /// ——という区別を明示しておく。
    #[test]
    fn non_finite_error_equality_follows_f64_partial_eq_for_nan() {
        let a = RawBytesError::NonFiniteValue {
            index: 0,
            value: f64::NAN,
        };
        assert_ne!(a, a.clone());
        // 有限値を載せた場合(実際には起きないが型としては可能)は通常どおり等しい。
        let b = RawBytesError::NonFiniteValue {
            index: 3,
            value: 1.0,
        };
        assert_eq!(b, b.clone());
    }

    // -- [f64; 3] ----------------------------------------------------------

    #[test]
    fn vec3_roundtrip() {
        assert_eq!(encode_vec3_le_base64(&[]), "");
        assert_eq!(decode_vec3_le_base64("").unwrap(), Vec::<[f64; 3]>::new());

        let values = [
            [0.0, -0.0, 1.0],
            [f64::MIN, f64::MAX, f64::EPSILON],
            [-1.5, 2.25, core::f64::consts::E],
        ];
        let decoded = decode_vec3_le_base64(&encode_vec3_le_base64(&values)).unwrap();
        assert_eq!(decoded.len(), values.len());
        for (d, v) in decoded.iter().zip(&values) {
            for (a, b) in d.iter().zip(v) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
        }
    }

    /// 平坦化した`f64`列と**バイト列が同一**であること(doc の主張の確認)、
    /// および成分順が x, y, z であること。
    #[test]
    fn vec3_is_flat_f64_in_xyz_order() {
        let values = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        let flat = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        assert_eq!(encode_vec3_le_base64(&values), encode_f64_le_base64(&flat));
        assert_bits_eq(
            &decode_f64_le_base64(&encode_vec3_le_base64(&values)).unwrap(),
            &flat,
        );
    }

    #[test]
    fn vec3_roundtrip_large_array() {
        let values: Vec<[f64; 3]> = (0..10_001)
            .map(|i| [i as f64, -(i as f64) * 0.5, 1.0 / (i as f64 + 1.0)])
            .collect();
        assert_eq!(
            decode_vec3_le_base64(&encode_vec3_le_base64(&values)).unwrap(),
            values
        );
    }

    #[test]
    fn vec3_rejects_unaligned_byte_length() {
        // 16バイト(f64 2個ぶん)は24の倍数でない。
        let sixteen = encode_base64(&[0u8; 16]);
        assert_eq!(
            decode_vec3_le_base64(&sixteen),
            Err(RawBytesError::UnalignedByteLength {
                len: 16,
                element_size: 24
            })
        );
    }

    // -- bool(ビット詰め) -------------------------------------------------

    #[test]
    fn bool_roundtrip_every_length_up_to_two_blocks() {
        // 0..=17要素で、8の倍数ちょうど・端数の両方を通す。
        for n in 0..=17usize {
            let values: Vec<bool> = (0..n).map(|i| i % 3 == 0).collect();
            let decoded = decode_bool_bitpacked_base64(&encode_bool_bitpacked_base64(&values))
                .unwrap_or_else(|e| panic!("n={n}: {e}"));
            assert_eq!(decoded, values, "n={n}");
        }
    }

    #[test]
    fn bool_roundtrip_empty_and_large() {
        assert_eq!(
            decode_bool_bitpacked_base64(&encode_bool_bitpacked_base64(&[])).unwrap(),
            Vec::<bool>::new()
        );

        // 128x128格子(=16384セル)相当。solid_cellsの現実的な規模。
        let values: Vec<bool> = (0..16_384)
            .map(|i: usize| i.count_ones().is_multiple_of(2))
            .collect();
        let encoded = encode_bool_bitpacked_base64(&values);
        assert_eq!(decode_bool_bitpacked_base64(&encoded).unwrap(), values);

        // ビット詰めが実際に効いていること: 8バイトヘッダ + 2048バイト → 2740文字前後。
        // 一方JSONの`[true,false,...]`は10万バイト級になる。
        let as_json = serde_json::to_string(&values).unwrap();
        assert!(
            encoded.len() * 20 < as_json.len(),
            "bitpacked {} bytes vs JSON {} bytes",
            encoded.len(),
            as_json.len()
        );
    }

    #[test]
    fn bool_rejects_missing_header() {
        // 4バイトしか無い(ヘッダ8バイトに足りない)。
        let short = encode_base64(&[0u8; 4]);
        assert_eq!(
            decode_bool_bitpacked_base64(&short),
            Err(RawBytesError::MissingBoolHeader(4))
        );
        assert_eq!(
            decode_bool_bitpacked_base64(""),
            Err(RawBytesError::MissingBoolHeader(0))
        );
    }

    #[test]
    fn bool_rejects_length_mismatch() {
        // 「17要素」と宣言しつつビットマップが1バイトしか無い(期待3バイト)。
        let mut bytes = 17u64.to_le_bytes().to_vec();
        bytes.push(0xFF);
        assert_eq!(
            decode_bool_bitpacked_base64(&encode_base64(&bytes)),
            Err(RawBytesError::BoolLengthMismatch {
                declared: 17,
                payload_len: 1
            })
        );
    }

    #[test]
    fn bool_rejects_non_canonical_trailing_bits() {
        // 「3要素」と宣言しつつ、使わない上位5ビットが立っている。
        let mut bytes = 3u64.to_le_bytes().to_vec();
        bytes.push(0b1111_1111);
        assert_eq!(
            decode_bool_bitpacked_base64(&encode_base64(&bytes)),
            Err(RawBytesError::NonCanonicalBoolPadding)
        );
        // 下位3ビットだけなら正準。
        let mut ok = 3u64.to_le_bytes().to_vec();
        ok.push(0b0000_0101);
        assert_eq!(
            decode_bool_bitpacked_base64(&encode_base64(&ok)).unwrap(),
            vec![true, false, true]
        );
    }

    #[test]
    fn bool_rejects_absurd_declared_length() {
        // 巨大な宣言長。usizeに収まる64bit環境では長さ不整合として、
        // 収まらない32bit環境ではオーバーフローとして弾かれる(どちらでも確保はしない)。
        let mut bytes = u64::MAX.to_le_bytes().to_vec();
        bytes.push(0);
        let err = decode_bool_bitpacked_base64(&encode_base64(&bytes)).unwrap_err();
        assert!(
            matches!(
                err,
                RawBytesError::BoolCountOverflow(_) | RawBytesError::BoolLengthMismatch { .. }
            ),
            "{err}"
        );
    }

    // -- i8 ----------------------------------------------------------------

    #[test]
    fn i8_roundtrip_full_range() {
        assert_eq!(encode_i8_base64(&[]), "");
        assert_eq!(decode_i8_base64("").unwrap(), Vec::<i8>::new());

        // i8の全域(-128..=127)を往復させる。
        let all: Vec<i8> = (i8::MIN..=i8::MAX).collect();
        assert_eq!(decode_i8_base64(&encode_i8_base64(&all)).unwrap(), all);
    }

    #[test]
    fn i8_roundtrip_ising_spins() {
        // 100x100のイジング格子相当の±1配列。
        let spins: Vec<i8> = (0..10_000)
            .map(|i| if i % 3 == 0 { 1 } else { -1 })
            .collect();
        let encoded = encode_i8_base64(&spins);
        assert_eq!(decode_i8_base64(&encoded).unwrap(), spins);

        // `[1,-1,-1,...]`の10進表現よりは小さい(1要素1バイト→1.33文字 vs 2〜3文字)。
        let as_json = serde_json::to_string(&spins).unwrap();
        assert!(encoded.len() < as_json.len());
    }

    // -- 将来の呼び出し側の姿(説明用。**本番のスキーマではない**) -----------

    /// `GridFluidRawStateJson`を生バイト表現へ移行したらどうなるか、の実物大の素描。
    ///
    /// **これは説明用のローカル型であって、`scenario::GridFluidRawStateJson`
    /// そのものには一切手を付けていない**(モジュールdoc §「このモジュールの適用範囲」)。
    /// 実際の移行は全ドメインのスキーマと`scenes/*.json`に同時に触る別作業である。
    #[test]
    fn usage_example_grid_fluid_raw_state() {
        use serde::{Deserialize, Serialize};

        #[derive(Deserialize, Serialize)]
        struct GridFluidRawStateJsonSketch {
            /// 速度場(長さ`nx*ny`、行優先 `i + nx*j`)。
            u: String,
            v: String,
            /// 固体セルかどうか(長さ`nx*ny`)。
            solid_cells: String,
            density: f64,
        }

        let (nx, ny) = (64usize, 32usize);
        let u: Vec<f64> = (0..nx * ny).map(|i| (i as f64).sin()).collect();
        let v: Vec<f64> = (0..nx * ny).map(|i| (i as f64).cos()).collect();
        let solid_cells: Vec<bool> = (0..nx * ny).map(|i| i % nx == 0).collect();

        // --- 書き出し側(`export::to_scenario`に相当) ---
        let json = serde_json::to_string(&GridFluidRawStateJsonSketch {
            // 発散を静かに焼き付けないよう検査付きの版を使う。
            u: encode_f64_le_base64_finite(&u).unwrap(),
            v: encode_f64_le_base64_finite(&v).unwrap(),
            solid_cells: encode_bool_bitpacked_base64(&solid_cells),
            density: 1000.0,
        })
        .unwrap();

        // --- 読み込み側(`World::from_scenario`に相当) ---
        let parsed: GridFluidRawStateJsonSketch = serde_json::from_str(&json).unwrap();
        let u_back = decode_f64_le_base64(&parsed.u).unwrap();
        let v_back = decode_f64_le_base64(&parsed.v).unwrap();
        let solid_back = decode_bool_bitpacked_base64(&parsed.solid_cells).unwrap();

        assert_bits_eq(&u_back, &u);
        assert_bits_eq(&v_back, &v);
        assert_eq!(solid_back, solid_cells);

        // 10進配列で書いた場合との比較(この符号化を入れる動機そのもの)。
        let decimal_json = serde_json::to_string(&(&u, &v, &solid_cells)).unwrap();
        assert!(
            json.len() * 2 < decimal_json.len(),
            "base64 {} bytes vs 10進 {} bytes",
            json.len(),
            decimal_json.len()
        );
    }
}
