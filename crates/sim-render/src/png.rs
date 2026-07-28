//! 自前の最小PNGエンコーダ。設計 docs/17-rendering/03-materials-camera.md §4.2
//! `gamma_encode`後の8bit値を実際にファイルへ書き出す最終段。
//!
//! **縮約実装の理由**: このワークスペースの依存はごく最小(各crateの`Cargo.toml`
//! 参照——`serde`/`criterion`/`wasm-bindgen`系のみ)であり、画像出力のためだけに
//! `png`/`flate2`等の外部クレートを足すのは既存の規律に反する。そのため本モジュールは
//! PNG仕様のうち**8bit・非パレット・RGB(カラータイプ2)・非インターレース**のみを
//! 実装し、圧縮は**zlibの`stored`(非圧縮)deflateブロック**で済ませる
//! (LZ77+Huffman符号化は行わない)。**したがって出力ファイルサイズは全く最適化
//! されない**(生の画素バイト数にほぼ比例して大きくなる——本増分の目的は画像出力
//! パイプラインの「正しさ」の立証であり、ファイルサイズの最適化はスコープ外)。
//! アルファチャンネル・8bit以外のビット深度・パレット・フィルタ(Sub/Up/Average/
//! Paeth、本実装は各スキャンライン先頭に常にフィルタタイプ0=Noneのみ書く)は非対応。

/// PNGファイルシグネチャ(仕様で定められた固定8バイト)。
const PNG_SIGNATURE: [u8; 8] = [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A];

/// CRC32(PNG仕様が指定するテーブル駆動実装、多項式`0xEDB88320`)。
/// 呼び出しのたびにテーブルを再生成する(256エントリのみで軽量、画像1枚あたり
/// チャンク数個分しか呼ばないためキャッシュの必要性が薄い)。
fn crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut n = 0u32;
    while n < 256 {
        let mut c = n;
        let mut k = 0;
        while k < 8 {
            if c & 1 != 0 {
                c = 0xEDB88320 ^ (c >> 1);
            } else {
                c >>= 1;
            }
            k += 1;
        }
        table[n as usize] = c;
        n += 1;
    }
    table
}

fn crc32(data: &[u8]) -> u32 {
    let table = crc32_table();
    let mut c: u32 = 0xFFFFFFFF;
    for &byte in data {
        c = table[((c ^ byte as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    c ^ 0xFFFFFFFF
}

/// Adler32(zlibストリームのフッタチェックサム、RFC 1950)。
fn adler32(data: &[u8]) -> u32 {
    const MOD_ADLER: u32 = 65521;
    let mut a: u32 = 1;
    let mut b: u32 = 0;
    for &byte in data {
        a = (a + byte as u32) % MOD_ADLER;
        b = (b + a) % MOD_ADLER;
    }
    (b << 16) | a
}

/// `data`をzlibストリーム(ヘッダ`0x78 0x01` + stored deflateブロック群 +
/// Adler32フッタ)に包む。各deflateブロックは非圧縮(`BTYPE=00`)で、最大長
/// 65535バイトごとに分割する(`LEN`がu16のため、これを超えるブロックは表現
/// できない——モジュールdoc「縮約実装の理由」参照)。
fn zlib_stored(data: &[u8]) -> Vec<u8> {
    const MAX_BLOCK: usize = 65535;
    let mut out = Vec::with_capacity(data.len() + (data.len() / MAX_BLOCK + 1) * 5 + 6);
    out.push(0x78);
    out.push(0x01);

    let mut offset = 0;
    loop {
        let remaining = data.len() - offset;
        let block_len = remaining.min(MAX_BLOCK);
        let is_final = offset + block_len >= data.len();
        // BFINAL(1bit) + BTYPE(2bit, 00=stored) + 現在バイトの残り5bitはパディング。
        // ここまでバイト境界に揃っている(直前のブロックも境界で終わる)ので、
        // このヘッダバイト自体が1バイトぴったりになる。
        out.push(if is_final { 0x01 } else { 0x00 });
        let len = block_len as u16;
        out.extend_from_slice(&len.to_le_bytes());
        out.extend_from_slice(&(!len).to_le_bytes()); // NLEN = LENの1の補数。
        out.extend_from_slice(&data[offset..offset + block_len]);
        offset += block_len;
        if is_final {
            break;
        }
    }

    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

/// PNGチャンク1個(長さ + 型 + データ + CRC32(型+データ))を`out`へ追記する。
fn write_chunk(out: &mut Vec<u8>, chunk_type: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.extend_from_slice(chunk_type);
    out.extend_from_slice(data);

    let mut crc_input = Vec::with_capacity(4 + data.len());
    crc_input.extend_from_slice(chunk_type);
    crc_input.extend_from_slice(data);
    out.extend_from_slice(&crc32(&crc_input).to_be_bytes());
}

/// `width`×`height`の8bit RGB画素列(`rgb.len() == width*height*3`、行優先・
/// 上から下)からPNGバイト列を組み立てる。
pub fn encode_rgb8(width: u32, height: u32, rgb: &[u8]) -> Vec<u8> {
    assert_eq!(
        rgb.len(),
        width as usize * height as usize * 3,
        "rgb buffer length must be width*height*3"
    );

    let mut out = Vec::new();
    out.extend_from_slice(&PNG_SIGNATURE);

    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.push(8); // bit depth = 8。
    ihdr.push(2); // color type = 2 (RGB、パレット無し)。
    ihdr.push(0); // 圧縮方式(仕様上0のみが定義される)。
    ihdr.push(0); // フィルタ方式(仕様上0のみが定義される)。
    ihdr.push(0); // インターレース無し。
    write_chunk(&mut out, b"IHDR", &ihdr);

    // 各スキャンラインの先頭にフィルタタイプ0(None)を付ける(仕様必須)。
    let stride = width as usize * 3;
    let mut raw = Vec::with_capacity((stride + 1) * height as usize);
    for row in 0..height as usize {
        raw.push(0);
        raw.extend_from_slice(&rgb[row * stride..row * stride + stride]);
    }
    let compressed = zlib_stored(&raw);
    write_chunk(&mut out, b"IDAT", &compressed);

    write_chunk(&mut out, b"IEND", &[]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CRC32はゼロ長入力・既知の短い入力に対する広く知られた基準値と一致する
    /// (PNG仕様のCRCアルゴリズムそのものの検証、"IEND"チャンクの型のみのCRCが
    /// 実際のPNGビューアで使われる既知値`0xAE426082`であることを確認)。
    #[test]
    fn crc32_matches_known_reference_values() {
        assert_eq!(crc32(b""), 0);
        // "IEND"(型のみ、データ長0)のCRCは全PNGファイルで共通の既知値。
        assert_eq!(crc32(b"IEND"), 0xAE42_6082);
    }

    /// Adler32はRFC 1950の例("Wikipedia"相当ではなく)自明な既知値
    /// (空データはa=1,b=0 → 0x00000001)と一致する。
    #[test]
    fn adler32_matches_the_definition_for_trivial_inputs() {
        assert_eq!(adler32(b""), 1);
        // 1バイト`0x01`: a=(1+1)%65521=2, b=(0+2)%65521=2 → (2<<16)|2。
        assert_eq!(adler32(&[0x01]), (2u32 << 16) | 2);
    }
}
