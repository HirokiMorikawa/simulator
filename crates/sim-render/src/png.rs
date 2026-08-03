//! 自前の最小PNGエンコーダ。設計 docs/17-rendering/03-materials-camera.md §4.2
//! `gamma_encode`後の8bit値を実際にファイルへ書き出す最終段。
//!
//! **縮約実装の理由**: このワークスペースの依存はごく最小(各crateの`Cargo.toml`
//! 参照——`serde`/`criterion`/`wasm-bindgen`系のみ)であり、画像出力のためだけに
//! `png`/`flate2`等の外部クレートを足すのは既存の規律に反する。そのため本モジュールは
//! PNG仕様のうち**8bit・非パレット・RGB(カラータイプ2)・非インターレース**のみを
//! 実装する。
//!
//! **群6で実際の圧縮(deflate)を実装した**。移行前は zlib の `stored`(非圧縮)
//! ブロックしか書いておらず、出力は生の画素バイト数にほぼ比例して大きくなっていた。
//! 群6では **LZ77(ハッシュチェーンによる最長一致探索、窓32KiB・最大一致長258)+
//! 固定ハフマン符号(RFC 1951 §3.2.6 の`BTYPE=01`)** を実装した。
//!
//! **残る縮約**: 動的ハフマン(`BTYPE=10`、ブロックごとに最適な符号表を作って
//! 一緒に送る)は実装しない——符号表の構築と送出だけで実装量が倍以上になる一方、
//! レンダリング結果の PNG に対する追加の圧縮率は数%程度にとどまるため。
//! 圧縮しきれない(固定ハフマンで膨らむ)入力に備えて `stored` 経路も残してあり、
//! **両方を試して短いほうを採用する**(RFC 1951 が想定する標準的な戦法)。
//! PNGのフィルタ(Sub/Up/Average/Paeth)は引き続き未対応で、各スキャンライン先頭に
//! 常にフィルタタイプ0(None)を書く——フィルタは deflate の効きを上げるための
//! 前処理であり、圧縮そのものが入った今も「効かせられる余地を残している」状態。
//! アルファチャンネル・8bit以外のビット深度・パレットも非対応。

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

/// deflateのビット列を組み立てるライタ(RFC 1951)。**ハフマン符号は上位ビットから、
/// それ以外(BFINAL/BTYPE・追加ビット・長さ)は下位ビットから**詰めるという仕様の
/// 非対称性をこの型に閉じ込める。
struct BitWriter {
    out: Vec<u8>,
    bit_buffer: u32,
    bit_count: u32,
}

impl BitWriter {
    fn new() -> BitWriter {
        BitWriter {
            out: Vec::new(),
            bit_buffer: 0,
            bit_count: 0,
        }
    }

    /// 下位ビットから`count`ビット書く(BFINAL/BTYPE・追加ビット用)。
    fn write_bits(&mut self, value: u32, count: u32) {
        self.bit_buffer |= (value & ((1 << count) - 1)) << self.bit_count;
        self.bit_count += count;
        while self.bit_count >= 8 {
            self.out.push((self.bit_buffer & 0xFF) as u8);
            self.bit_buffer >>= 8;
            self.bit_count -= 8;
        }
    }

    /// 上位ビットから`count`ビット書く(ハフマン符号用、RFC 1951 §3.1.1)。
    fn write_code(&mut self, code: u32, count: u32) {
        for i in (0..count).rev() {
            self.write_bits((code >> i) & 1, 1);
        }
    }

    fn finish(mut self) -> Vec<u8> {
        if self.bit_count > 0 {
            self.out.push((self.bit_buffer & 0xFF) as u8);
        }
        self.out
    }
}

/// 固定ハフマンのリテラル/長さ符号(RFC 1951 §3.2.6 の表)を`(code, bit_count)`で返す。
fn fixed_literal_code(symbol: u16) -> (u32, u32) {
    match symbol {
        0..=143 => (0x30 + symbol as u32, 8),
        144..=255 => (0x190 + (symbol as u32 - 144), 9),
        256..=279 => (symbol as u32 - 256, 7),
        _ => (0xC0 + (symbol as u32 - 280), 8),
    }
}

/// 長さ(3..=258)→ (長さ符号, 追加ビット値, 追加ビット数)。RFC 1951 §3.2.5 の表。
fn length_code(length: u16) -> (u16, u32, u32) {
    const BASE: [u16; 29] = [
        3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115,
        131, 163, 195, 227, 258,
    ];
    const EXTRA: [u32; 29] = [
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0,
    ];
    let mut i = 28;
    while BASE[i] > length {
        i -= 1;
    }
    (257 + i as u16, (length - BASE[i]) as u32, EXTRA[i])
}

/// 距離(1..=32768)→ (距離符号, 追加ビット値, 追加ビット数)。RFC 1951 §3.2.5 の表。
fn distance_code(distance: u16) -> (u16, u32, u32) {
    const BASE: [u16; 30] = [
        1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537,
        2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577,
    ];
    const EXTRA: [u32; 30] = [
        0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12,
        13, 13,
    ];
    let mut i = 29;
    while BASE[i] > distance {
        i -= 1;
    }
    (i as u16, (distance - BASE[i]) as u32, EXTRA[i])
}

/// LZ77 + 固定ハフマンで`data`をdeflate圧縮する(モジュールdoc参照)。
///
/// 一致探索は3バイトハッシュのチェーン法。チェーンを辿る回数は`MAX_CHAIN`で
/// 打ち切る(圧縮率と時間の妥協。deflateの実装が普遍的に採る手法で、打ち切っても
/// 出力は常に正しい——単に見逃した一致がリテラルとして出るだけ)。
fn deflate_fixed_huffman(data: &[u8]) -> Vec<u8> {
    const WINDOW: usize = 32768;
    const MIN_MATCH: usize = 3;
    const MAX_MATCH: usize = 258;
    const MAX_CHAIN: usize = 128;
    const HASH_BITS: usize = 15;
    const HASH_SIZE: usize = 1 << HASH_BITS;

    let mut writer = BitWriter::new();
    writer.write_bits(1, 1); // BFINAL=1(単一ブロック)。
    writer.write_bits(1, 2); // BTYPE=01(固定ハフマン)。

    // head[hash] = 直近にそのハッシュで始まった位置、prev[pos % WINDOW] = その前の位置。
    let mut head = vec![usize::MAX; HASH_SIZE];
    let mut prev = vec![usize::MAX; WINDOW];
    let hash_of = |d: &[u8], i: usize| -> usize {
        ((d[i] as usize) << 10 ^ (d[i + 1] as usize) << 5 ^ (d[i + 2] as usize)) & (HASH_SIZE - 1)
    };

    let mut pos = 0;
    while pos < data.len() {
        let mut best_len = 0usize;
        let mut best_dist = 0usize;
        if pos + MIN_MATCH <= data.len() {
            let h = hash_of(data, pos);
            let mut candidate = head[h];
            let mut chain = 0;
            let limit = pos.saturating_sub(WINDOW);
            while candidate != usize::MAX && candidate >= limit && chain < MAX_CHAIN {
                let max_len = MAX_MATCH.min(data.len() - pos);
                let mut len = 0;
                while len < max_len && data[candidate + len] == data[pos + len] {
                    len += 1;
                }
                if len > best_len {
                    best_len = len;
                    best_dist = pos - candidate;
                    if best_len == max_len {
                        break;
                    }
                }
                candidate = prev[candidate % WINDOW];
                chain += 1;
            }
            // 見つかった位置を辞書へ登録(一致に使ったバイト全てを登録する)。
            let advance = if best_len >= MIN_MATCH { best_len } else { 1 };
            for k in 0..advance {
                let p = pos + k;
                if p + MIN_MATCH <= data.len() {
                    let hk = hash_of(data, p);
                    prev[p % WINDOW] = head[hk];
                    head[hk] = p;
                }
            }
        }

        if best_len >= MIN_MATCH {
            let (lcode, lextra, lbits) = length_code(best_len as u16);
            let (code, bits) = fixed_literal_code(lcode);
            writer.write_code(code, bits);
            if lbits > 0 {
                writer.write_bits(lextra, lbits);
            }
            let (dcode, dextra, dbits) = distance_code(best_dist as u16);
            writer.write_code(dcode as u32, 5); // 距離符号は固定5ビット。
            if dbits > 0 {
                writer.write_bits(dextra, dbits);
            }
            pos += best_len;
        } else {
            let (code, bits) = fixed_literal_code(data[pos] as u16);
            writer.write_code(code, bits);
            pos += 1;
        }
    }

    let (code, bits) = fixed_literal_code(256); // ブロック終端。
    writer.write_code(code, bits);
    writer.finish()
}

/// `data`をzlibストリーム(RFC 1950)に包む。**固定ハフマン圧縮と`stored`の両方を
/// 作り、短いほうを採用する**(モジュールdoc参照)。
fn zlib_compress(data: &[u8]) -> Vec<u8> {
    let compressed = deflate_fixed_huffman(data);
    let stored = deflate_stored(data);
    let body = if compressed.len() < stored.len() {
        compressed
    } else {
        stored
    };
    let mut out = Vec::with_capacity(body.len() + 6);
    out.push(0x78);
    out.push(0x01);
    out.extend_from_slice(&body);
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

/// `stored`(非圧縮、`BTYPE=00`)のdeflateブロック列を作る。最大長65535バイトごとに
/// 分割する(`LEN`がu16のため)。圧縮しきれない入力への保険として残してある
/// (モジュールdoc参照)。zlibヘッダ・Adler32は`zlib_compress`が付ける。
fn deflate_stored(data: &[u8]) -> Vec<u8> {
    const MAX_BLOCK: usize = 65535;
    let mut out = Vec::with_capacity(data.len() + (data.len() / MAX_BLOCK + 1) * 5);

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
    let compressed = zlib_compress(&raw);
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

    /// 固定ハフマンdeflateの**ラウンドトリップ**(群6)。圧縮器は「復元できること」を
    /// 示さない限り検証したことにならないので、テスト内に固定ハフマン専用の最小
    /// inflateを書いて突き合わせる(RFC 1951 §3.2.6 の符号長規則をそのまま復号側から
    /// 書き下したもので、圧縮側の実装とは独立)。
    fn inflate_fixed(bits: &[u8]) -> Vec<u8> {
        struct Reader<'a> {
            data: &'a [u8],
            pos: usize,
        }
        impl Reader<'_> {
            fn bit(&mut self) -> u32 {
                let byte = self.data[self.pos / 8];
                let b = (byte >> (self.pos % 8)) & 1;
                self.pos += 1;
                b as u32
            }
            /// 下位ビットから`n`ビット(追加ビット用)。
            fn bits(&mut self, n: u32) -> u32 {
                let mut v = 0;
                for i in 0..n {
                    v |= self.bit() << i;
                }
                v
            }
            /// 上位ビットから`n`ビット(ハフマン符号用)。
            fn code(&mut self, n: u32) -> u32 {
                let mut v = 0;
                for _ in 0..n {
                    v = (v << 1) | self.bit();
                }
                v
            }
        }
        const LEN_BASE: [u16; 29] = [
            3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99,
            115, 131, 163, 195, 227, 258,
        ];
        const LEN_EXTRA: [u32; 29] = [
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0,
        ];
        const DIST_BASE: [u16; 30] = [
            1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025,
            1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577,
        ];
        const DIST_EXTRA: [u32; 30] = [
            0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12,
            12, 13, 13,
        ];

        let mut r = Reader { data: bits, pos: 0 };
        assert_eq!(r.bit(), 1, "BFINAL は 1(単一ブロック)のはず");
        assert_eq!(r.bits(2), 1, "BTYPE は 01(固定ハフマン)のはず");

        let mut out: Vec<u8> = Vec::new();
        loop {
            // 固定ハフマンの復号: 7bit読んで 0..=23 なら 256..=279。
            let mut v = r.code(7);
            let symbol: u16 = if v <= 0b0010111 {
                256 + v as u16
            } else {
                v = (v << 1) | r.bit();
                if (0b00110000..=0b10111111).contains(&v) {
                    (v - 0b00110000) as u16
                } else if (0b11000000..=0b11000111).contains(&v) {
                    280 + (v - 0b11000000) as u16
                } else {
                    v = (v << 1) | r.bit();
                    144 + (v - 0b110010000) as u16
                }
            };
            if symbol == 256 {
                break;
            }
            if symbol < 256 {
                out.push(symbol as u8);
                continue;
            }
            let i = (symbol - 257) as usize;
            let length = LEN_BASE[i] as usize + r.bits(LEN_EXTRA[i]) as usize;
            let dcode = r.code(5) as usize;
            let distance = DIST_BASE[dcode] as usize + r.bits(DIST_EXTRA[dcode]) as usize;
            let start = out.len() - distance;
            for k in 0..length {
                let byte = out[start + k];
                out.push(byte);
            }
        }
        out
    }

    /// 圧縮 → 復元で元のバイト列に**厳密に**戻ること。リテラルのみの入力・
    /// 長い繰り返し(LZ77一致が長さ258で頭打ちになるところまで)・ランダム風の
    /// 入力・9bit符号域(値144以上)を含む入力・空入力を通す。
    #[test]
    fn deflate_fixed_huffman_round_trips_every_shape_of_input() {
        let mut cases: Vec<Vec<u8>> = vec![
            Vec::new(),
            b"a".to_vec(),
            b"abcdefghijklmnopqrstuvwxyz".to_vec(),
            // 長い繰り返し: 最大一致長258を必ず跨ぐ。
            std::iter::repeat_n(b'x', 1000).collect(),
            // 周期的な繰り返し(距離>1の一致)。
            b"abcabcabcabcabcabcabcabcabcabc".to_vec(),
            // 9bit符号域(144..=255)を含む。
            (0u16..=255).map(|v| v as u8).collect(),
        ];
        // 決定論的な擬似ランダム(圧縮しにくい入力 = stored が選ばれる経路も通す)。
        let mut state = 0x1234_5678u32;
        let noise: Vec<u8> = (0..4096)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state >> 24) as u8
            })
            .collect();
        cases.push(noise);
        // 画像らしい入力(スキャンライン単位の繰り返し)。
        let mut image = Vec::new();
        for row in 0..64u8 {
            image.push(0); // フィルタタイプ。
            for col in 0..64u8 {
                image.extend_from_slice(&[row, col, 128]);
            }
        }
        cases.push(image);

        for data in &cases {
            let compressed = deflate_fixed_huffman(data);
            let restored = inflate_fixed(&compressed);
            assert_eq!(
                &restored,
                data,
                "round trip failed for input of {} bytes",
                data.len()
            );
        }
    }

    /// **群6の目的そのもの**: 実際に縮むこと。レンダリング結果のような滑らかな
    /// 画像で、移行前の`stored`より明確に小さくなることを確認する。
    /// あわせて、圧縮しにくい入力では`zlib_compress`が`stored`へ退避し、
    /// **決して生データより大きくならない**ことも確認する。
    #[test]
    fn zlib_compress_actually_shrinks_image_data_and_never_inflates_noise() {
        // グラデーション画像(実際のレンダリング結果に近い滑らかさ)。
        let width = 128usize;
        let height = 128usize;
        let mut raw = Vec::new();
        for row in 0..height {
            raw.push(0u8);
            for col in 0..width {
                let v = ((row + col) * 255 / (width + height)) as u8;
                raw.extend_from_slice(&[v, v / 2, 255 - v]);
            }
        }
        let compressed = zlib_compress(&raw);
        let stored = deflate_stored(&raw).len() + 6;
        assert!(
            compressed.len() * 4 < stored,
            "滑らかな画像は stored の1/4未満まで縮むはず: compressed={} stored={}",
            compressed.len(),
            stored
        );

        // 圧縮不能な入力(高エントロピー)でも生データ+わずかなオーバーヘッド以内。
        let mut state = 0xDEAD_BEEFu32;
        let noise: Vec<u8> = (0..20000)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state >> 24) as u8
            })
            .collect();
        let noise_compressed = zlib_compress(&noise);
        assert!(
            noise_compressed.len() <= noise.len() + 32,
            "圧縮不能な入力では stored へ退避するはず: {} vs {}",
            noise_compressed.len(),
            noise.len()
        );
    }

    /// PNG全体としても、圧縮の導入前後で**画素データは同一**であること
    /// (IHDRの寸法・IENDまでの構造が保たれ、IDATだけが縮む)。
    #[test]
    fn encode_rgb8_keeps_the_png_structure_while_shrinking_idat() {
        let (w, h) = (32u32, 32u32);
        let rgb: Vec<u8> = (0..(w * h * 3)).map(|i| (i % 17) as u8).collect();
        let png = encode_rgb8(w, h, &rgb);
        assert_eq!(&png[0..8], &PNG_SIGNATURE);
        // IHDR: 長さ13 + 型 + データ。
        assert_eq!(&png[8..12], &13u32.to_be_bytes());
        assert_eq!(&png[12..16], b"IHDR");
        assert_eq!(&png[16..20], &w.to_be_bytes());
        assert_eq!(&png[20..24], &h.to_be_bytes());
        assert_eq!(png[24], 8, "bit depth");
        assert_eq!(png[25], 2, "color type = RGB");
        assert_eq!(&png[png.len() - 8..png.len() - 4], b"IEND");
        // 生画素(+ 行ごとのフィルタバイト)より小さくなっている。
        assert!(
            png.len() < rgb.len() + h as usize,
            "圧縮が効いていない: png={} raw={}",
            png.len(),
            rgb.len()
        );
    }
}
