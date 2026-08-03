//! フレームバッファ(線形RGB放射輝度の画素配列)+ 出力段。設計
//! docs/17-rendering/03-materials-camera.md §4.2「spectrum_to_display」のうち
//! `tone_map(RGB_linear, exposure)` → `gamma_encode` の段を実装する(本増分より
//! 前は`sim-render`に画像出力パイプライン自体が存在しなかった、`lib.rs`モジュール
//! doc参照)。
//!
//! **縮約実装の理由**: 設計§4.2の`spectrum_to_display`は分光放射輝度$L_\lambda$を
//! CIE等色関数で積分してXYZ→線形sRGBへ変換する経路を想定するが、`sim-render`は
//! まだ分光レンダリング本体(hero wavelength法、`bsdf.rs`/`prism.rs`モジュールdoc
//! 参照)を持たないため、本モジュールは既にRGB化された線形放射輝度(`render`
//! モジュールのチャンネル別レンダリングが作る)を受け取るところから始める。
//! 露出は単純なスカラー倍率(`to_srgb8`/`write_png`の`exposure`引数)。**増分C2**で
//! この倍率をシャッター速度・ISO・絞りから物理的に求める関数
//! (`camera::relative_exposure`/`camera::exposure_value_at_iso100`)を追加した——
//! 本モジュール自体は相変わらず「渡された倍率をそのまま掛けるだけ」であり、
//! EV↔倍率の変換は`camera.rs`側の責務のまま(本モジュールのAPIは変更していない)。
//! トーンマッピングは既存の`tonemap::reinhard_tonemap_color`(色相を保つ輝度ベース版、
//! 既に単体で検証済み)をそのまま再利用する——ここで新しい圧縮アルゴリズムは
//! 実装しない(フィルミック/ACES演算子・分光→CIE→sRGBの完全な`spectrum_to_display`
//! (設計§4.2)は引き続き未実装)。

use std::io;
use std::path::Path;

use sim_math::Vec3;

use crate::png;
use crate::tonemap::reinhard_tonemap_color;

/// 線形RGB放射輝度の画素配列。`pixels`は行優先(row-major)・上から下
/// (`render`モジュールのNDC変換と対応、モジュールdoc参照)、`pixels.len() ==
/// width*height`。
pub struct Framebuffer {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<Vec3>,
}

impl Framebuffer {
    /// 全画素を黒(`Vec3::ZERO`)で初期化する。
    pub fn new(width: u32, height: u32) -> Framebuffer {
        Framebuffer {
            width,
            height,
            pixels: vec![Vec3::ZERO; width as usize * height as usize],
        }
    }

    /// sRGBガンマ符号化(設計§4.2の`gamma_encode`、IEC 61966-2-1の区分関数)。
    /// `c`は線形light(トーンマッピング後、`[0,1]`を仮定)。
    fn gamma_encode_channel(c: f64) -> f64 {
        if c <= 0.0031308 {
            12.92 * c
        } else {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        }
    }

    /// 出力段: **露出倍率 → `reinhard_tonemap_color`(色相を保つトーンマッピング)
    /// → sRGBガンマ符号化 → u8量子化**(モジュールdoc参照)の順に適用し、
    /// `width*height*3`長(行優先・RGBRGB…)のバイト列を返す。
    ///
    /// トーンマッピング後の値は理論上`[0,1)`に収まるはずだが(`tonemap.rs`
    /// モジュールdoc参照)、色相保存のためにチャンネルごとに輝度圧縮比を掛け戻す
    /// 都合上、単色に極端に偏った放射輝度では個々のチャンネルが1を僅かに超え得る
    /// ため(輝度は`[0,1)`に収まっても個々のチャンネルの保証ではない)、
    /// 量子化直前に`[0,1]`へクランプする。
    pub fn to_srgb8(&self, exposure: f64) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.pixels.len() * 3);
        for &radiance in &self.pixels {
            let exposed = radiance.scale(exposure);
            let tonemapped = reinhard_tonemap_color(exposed);
            for channel in [tonemapped.x, tonemapped.y, tonemapped.z] {
                let clamped = channel.clamp(0.0, 1.0);
                let encoded = Self::gamma_encode_channel(clamped);
                out.push((encoded * 255.0).round().clamp(0.0, 255.0) as u8);
            }
        }
        out
    }

    /// `to_srgb8`でsRGB8bit量子化した後、自前の最小PNGエンコーダ(`png`モジュール
    /// doc「縮約実装の理由」参照)でファイルへ書き出す。
    pub fn write_png(&self, path: &Path, exposure: f64) -> io::Result<()> {
        let rgb = self.to_srgb8(exposure);
        let bytes = png::encode_rgb8(self.width, self.height, &rgb);
        std::fs::write(path, bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 黒(放射輝度0)は露出・トーンマッピングによらず常に0のまま(`tonemap.rs`の
    /// `reinhard_tonemap_color_at_zero_radiance_is_zero`がゼロを保証する)。
    #[test]
    fn to_srgb8_maps_zero_radiance_to_zero() {
        let mut fb = Framebuffer::new(1, 1);
        fb.pixels[0] = Vec3::ZERO;
        let rgb = fb.to_srgb8(1.0);
        assert_eq!(rgb, vec![0, 0, 0]);
    }

    /// 十分大きい放射輝度は、Reinhardトーンマッピングが1に漸近し(`tonemap.rs`の
    /// `reinhard_tonemap_approaches_one_for_very_high_luminance`参照)、
    /// ガンマ符号化後も1に丸められるため、量子化後は255になる。
    #[test]
    fn to_srgb8_maps_very_high_radiance_to_255() {
        let mut fb = Framebuffer::new(1, 1);
        fb.pixels[0] = Vec3::new(1.0e9, 1.0e9, 1.0e9);
        let rgb = fb.to_srgb8(1.0);
        assert_eq!(rgb, vec![255, 255, 255]);
    }

    /// 既知値: グレースケール放射輝度(R=G=B、Rec.709輝度と各チャンネルが恒等的に
    /// 一致するため`reinhard_tonemap_color`は単に`reinhard_tonemap`をチャンネルへ
    /// 適用するのと同じになる)に対し、`reinhard_tonemap`の閉形式 →
    /// sRGBガンマの閉形式 → 四捨五入という手計算と厳密一致することを確認する。
    #[test]
    fn to_srgb8_matches_the_closed_form_computation_for_a_midtone_gray_value() {
        let radiance = 1.0; // 露出後もexposure=1.0でそのまま。
        let mut fb = Framebuffer::new(1, 1);
        fb.pixels[0] = Vec3::new(radiance, radiance, radiance);

        let luminance_out = radiance / (1.0 + radiance); // reinhard_tonemap(1.0) = 0.5。
        let gamma = if luminance_out <= 0.0031308 {
            12.92 * luminance_out
        } else {
            1.055 * luminance_out.powf(1.0 / 2.4) - 0.055
        };
        let expected_byte = (gamma * 255.0).round() as u8;

        let rgb = fb.to_srgb8(1.0);
        assert_eq!(rgb, vec![expected_byte, expected_byte, expected_byte]);

        // 手計算自体が非自明な値であることも確認しておく(0/255への丸めのみを
        // 確認する上2テストと役割が重ならないようにする)。
        assert!(
            expected_byte > 0 && expected_byte < 255,
            "expected_byte={expected_byte} should be a genuine midtone, not a clamp to an \
             endpoint"
        );
    }

    /// PNGラウンドトリップ: `write_png`で実際にファイルへ書き出し、読み戻した
    /// バイト列のシグネチャ・IHDRの幅/高さ・全チャンクのCRC32を自前で再計算して
    /// 検証する(`png`モジュールのCRC実装とは独立に、ここではPNG仕様の読み取り側
    /// (チャンク境界の走査)を手で書いて突き合わせる)。
    #[test]
    fn write_png_round_trips_through_a_real_file_with_valid_signature_ihdr_and_chunk_crcs() {
        let width = 4u32;
        let height = 3u32;
        let mut fb = Framebuffer::new(width, height);
        // 既知の非一様な放射輝度パターン(全画素が同じ値にならないようにする)。
        for y in 0..height {
            for x in 0..width {
                let i = (y * width + x) as usize;
                fb.pixels[i] = Vec3::new(
                    0.1 * (x as f64 + 1.0),
                    0.2 * (y as f64 + 1.0),
                    0.05 * (i as f64 + 1.0),
                );
            }
        }

        let dir = std::env::temp_dir().join("sim-render-test-scratch");
        std::fs::create_dir_all(&dir).expect("should be able to create the scratch dir");
        let path = dir.join("roundtrip_test.png");
        fb.write_png(&path, 1.0).expect("write_png should succeed");

        let bytes = std::fs::read(&path).expect("should be able to read back the written file");

        // シグネチャ。
        assert_eq!(
            &bytes[0..8],
            &[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]
        );

        // チャンクを自前で走査し、種類ごとに集計しつつ各チャンクのCRC32を
        // 再計算して照合する。
        fn crc32_reference(data: &[u8]) -> u32 {
            // PNG仕様のCRCアルゴリズムを、`png.rs`実装とは別に(コピーではなく)
            // 素朴な多項式除算そのままの定義で再実装し、独立した検証にする。
            let mut crc: u32 = 0xFFFFFFFF;
            for &byte in data {
                crc ^= byte as u32;
                for _ in 0..8 {
                    if crc & 1 != 0 {
                        crc = (crc >> 1) ^ 0xEDB88320;
                    } else {
                        crc >>= 1;
                    }
                }
            }
            crc ^ 0xFFFFFFFF
        }

        let mut offset = 8usize;
        let mut seen_ihdr = false;
        let mut seen_idat = false;
        let mut seen_iend = false;
        let mut ihdr_width = 0u32;
        let mut ihdr_height = 0u32;

        while offset < bytes.len() {
            let length = u32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap()) as usize;
            let chunk_type = &bytes[offset + 4..offset + 8];
            let data = &bytes[offset + 8..offset + 8 + length];
            let stored_crc = u32::from_be_bytes(
                bytes[offset + 8 + length..offset + 12 + length]
                    .try_into()
                    .unwrap(),
            );

            let mut crc_input = Vec::with_capacity(4 + length);
            crc_input.extend_from_slice(chunk_type);
            crc_input.extend_from_slice(data);
            assert_eq!(
                crc32_reference(&crc_input),
                stored_crc,
                "CRC mismatch for chunk type {:?}",
                std::str::from_utf8(chunk_type)
            );

            match chunk_type {
                b"IHDR" => {
                    seen_ihdr = true;
                    ihdr_width = u32::from_be_bytes(data[0..4].try_into().unwrap());
                    ihdr_height = u32::from_be_bytes(data[4..8].try_into().unwrap());
                    assert_eq!(data[8], 8, "bit depth should be 8");
                    assert_eq!(data[9], 2, "color type should be 2 (RGB)");
                }
                b"IDAT" => seen_idat = true,
                b"IEND" => {
                    seen_iend = true;
                    assert_eq!(length, 0, "IEND must carry no data");
                }
                _ => {}
            }

            offset += 12 + length; // length(4) + type(4) + data + crc(4)。
        }

        assert!(
            seen_ihdr && seen_idat && seen_iend,
            "all three chunk types must be present"
        );
        assert_eq!(ihdr_width, width);
        assert_eq!(ihdr_height, height);

        let _ = std::fs::remove_file(&path);
    }

    /// 増分C2・設計§7「露出: EV変化に対する像の明るさが物理的にスケール」の
    /// 画像レベルの検証。絞りを1段絞る(EV+1、`camera::relative_exposure`が
    /// 半分の倍率を返す)と、**トーンマップ前**の線形放射輝度は`to_srgb8`の出力段
    /// (モジュールdoc「露出倍率→トーンマップ→ガンマ→u8」)のうち露出倍率を掛ける
    /// 工程が単純な線形スケールであるため厳密に半分になる。一方トーンマップ後
    /// (Reinhard)は非線形圧縮のため出力バイト値は厳密には半分にならない――これを
    /// 閉形式で予測した値と突き合わせて確認し、設計の受け入れ基準を「トーンマップ後の
    /// 画素値が半分になる」と誤読しないための対照とする。
    #[test]
    fn stopping_down_by_one_stop_halves_pre_tonemap_radiance_but_not_the_tonemapped_byte() {
        use crate::camera::relative_exposure;

        let shutter_time = 1.0 / 60.0;
        let iso = 100.0;
        let f_number = 2.8;
        let exposure_a = relative_exposure(shutter_time, iso, f_number);
        let exposure_b = relative_exposure(shutter_time, iso, f_number * std::f64::consts::SQRT_2);

        let radiance = Vec3::new(0.6, 0.6, 0.6);
        let mut fb = Framebuffer::new(1, 1);
        fb.pixels[0] = radiance;

        // トーンマップ前(露出を掛けただけ)の値は厳密に半分。
        let pre_a = radiance.scale(exposure_a);
        let pre_b = radiance.scale(exposure_b);
        assert!(
            (pre_b.x - pre_a.x / 2.0).abs() < 1e-9,
            "pre_a={pre_a:?} pre_b={pre_b:?}"
        );

        // トーンマップ後の閉形式予測(グレースケールなので`reinhard_tonemap_color`は
        // `reinhard_tonemap`をそのまま各チャンネルへ適用するのと同じになる、
        // `tonemap.rs`のテストと同じ根拠)。
        let gamma_of = |c: f64| -> f64 {
            if c <= 0.0031308 {
                12.92 * c
            } else {
                1.055 * c.powf(1.0 / 2.4) - 0.055
            }
        };
        let expected_byte = |pre: f64| -> u8 {
            let tonemapped = pre / (1.0 + pre);
            (gamma_of(tonemapped.clamp(0.0, 1.0)) * 255.0)
                .round()
                .clamp(0.0, 255.0) as u8
        };
        let byte_a = expected_byte(pre_a.x);
        let byte_b = expected_byte(pre_b.x);

        let rgb_a = fb.to_srgb8(exposure_a);
        let rgb_b = fb.to_srgb8(exposure_b);
        assert_eq!(rgb_a[0], byte_a, "channel R at exposure_a");
        assert_eq!(rgb_b[0], byte_b, "channel R at exposure_b");

        // 非線形性そのものの確認: トーンマップ後のバイト値は厳密には半分になって
        // いない(設計の受け入れ基準の誤読を防ぐ対照実験)。
        assert!(
            (byte_b as f64 - byte_a as f64 / 2.0).abs() > 1.0,
            "post-tonemap byte should NOT simply halve (tonemap is nonlinear): \
             byte_a={byte_a} byte_b={byte_b}"
        );
    }
}
