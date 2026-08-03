//! 物性DB(`sim_core::MaterialDb`)と描画マテリアル(`crate::Material`、BSDF)の橋。
//! 設計 docs/17-rendering/03-materials-camera.md §3「物性 DB との接続 —
//! **MaterialDb を単一の真実源に**」・§5「物性 DB に光学値が無い材料は既定 BSDF
//! (誘電体 $n=1.5$)+ 警告」・§9 パラメータ表。
//!
//! デモD41「材質ギャラリー(金属/ガラス/プラスチックの球アレイ、**物性DB連動**)」の
//! 土台であり、合格基準は R1(白色炉=エネルギー保存)と R2(フレネル)。
//!
//! ## この橋を作るまで `sim-render` は `sim-core` を一度も参照していなかった
//!
//! `crates/sim-render/Cargo.toml` は以前から `sim-core` を依存に宣言していたが、
//! `crates/sim-render/src/` 内に `sim_core` の参照は1件も無く、**依存の辺だけが
//! あってコードの橋は無い**状態だった。本モジュールがその橋である。
//!
//! ## 実装中に判明した、設計の縮退規則がそのままでは使えない事情(正直な記録)
//!
//! 設計§3は「比誘電率($n=\sqrt{\varepsilon_r}$ の可視域近似)」を屈折率の
//! 導出手段として挙げるが、**`MaterialDb` が保持する `relative_permittivity` は
//! 静的(DC)比誘電率であり、可視域の値ではない**。実測すると:
//!
//! | 材質 | `relative_permittivity` | $\sqrt{\varepsilon_r}$ | 実際の可視域 $n$ |
//! |---|---:|---:|---:|
//! | 水 | 80.1 | **8.95** | **1.333** |
//! | ガラス | 6.0 | 2.449 | 1.52 |
//! | 他10材質 | 1.0(プレースホルダ) | 1.000(=真空) | — |
//!
//! 水の乖離が特に大きいのは物理的に正しい挙動である——静的比誘電率80.1は分子双極子の
//! 再配向による寄与が支配的だが、可視光($\sim10^{14}$ Hz)では双極子が追随できず
//! $\varepsilon_r(\text{光学})\approx n^2 = 1.777$ まで落ちる。つまり
//! $\sqrt{\varepsilon_r}$ は**可視域近似として成立しない**。
//!
//! さらに、`refractive_index` を持たない10材質は `relative_permittivity` が
//! 一律 1.0(実測値ではなくプレースホルダ)なので、$\sqrt{\varepsilon_r}=1.0$=真空
//! となり描画には使えない。
//!
//! **したがって解決順序を「`refractive_index` を最優先」とし、$\sqrt{\varepsilon_r}$ は
//! その次に置いた**(設計§3の規則自体は実装してあるが、標準DBでは発火しない——
//! $\varepsilon_r \ne 1$ の2材質はいずれも `refractive_index` を持つため)。
//! 発火した場合は静的比誘電率由来であることを警告に含める。
//!
//! ## 金属の複素屈折率は `MaterialDb` に無い(縮約実装)
//!
//! `sim_core::Material` は消衰係数 $k$ を持たない。設計§9のパラメータ表が
//! 金(Au) $0.47+2.4i$・アルミ $0.96+6.7i$(550nm)を挙げているので、**その2件だけを
//! 本モジュール側の光学パラメータ表として持つ**。銅・鋼も物性DB上は金属だが、
//! 設計§9にもDBにも $n+ik$ が無いため**推測値を捏造せず**、警告付きで既定BSDFへ
//! 落とす(下記 `MetalOptics` のdoc参照)。
//!
//! なお `resistivity` は金属判定に使えないことを実測で確認した:
//! ガラスが `Some(1.0e12)`(絶縁体なのに値を持つ)、**アルミニウムが `None`**
//! (金属なのに欠測)であり、有無でも大小でも一貫した判定にならない。
//! そのため判定は上記の名前キー表で行う。
//!
//! ## アルベド(反射色)は物性DBに無い
//!
//! `sim_core::Material` に反射アルベドのフィールドは無い(`emissivity` は熱赤外の
//! 放射率であって可視域の反射率ではない)。本モジュールは**アルベドを発明しない**
//! ——不透明材質も設計§5の規則どおり誘電体BSDFとして扱う(フレネル反射率は
//! 屈折率だけで決まるため、R2の検証は成立する)。拡散反射色を持つ
//! `Lambertian` へのマッピングは分光アルベドの実測値がDBへ入るまで対象外。

use sim_core::{MaterialDb, MaterialId};

use crate::bsdf::{Dielectric, Metal};
use crate::path_tracer::Material;

/// 設計§5「物性 DB に光学値が無い材料は既定 BSDF(誘電体 $n=1.5$)」の既定屈折率。
pub const DEFAULT_IOR: f64 = 1.5;

/// $\sqrt{\varepsilon_r}$ をこの値以下しか与えない材質は「光学値なし」とみなす
/// (真空 $n=1$ 相当は描画上意味を持たないため)。モジュールdocの表を参照。
const MIN_USEFUL_IOR: f64 = 1.05;

/// 設計§9パラメータ表が与える金属の複素屈折率(550nm)。
///
/// **縮約実装(正直な記録)**: `sim_core::Material` は消衰係数 $k$ を持たないため、
/// この表は `sim-render` 側が持つ。設計§9に載っている2件のみを収録し、**それ以外の
/// 金属(銅・鋼)の値は推測しない**——出典の無い数値を物性表に混ぜると、この
/// プロジェクトが `MaterialDb` に課している「`source`/`uncertainty` を伴う実測値」
/// という規律が崩れるため。収録外の材質は `from_material_db` が警告付きで
/// 既定BSDFへ落とす。
const METAL_OPTICS: &[MetalOptics] = &[
    MetalOptics {
        name: "アルミニウム",
        n: 0.96,
        k: 6.7,
    },
    MetalOptics {
        name: "金",
        n: 0.47,
        k: 2.4,
    },
];

/// 金属1件分の複素屈折率 $n+ik$(`METAL_OPTICS` の要素)。
#[derive(Clone, Copy, Debug)]
struct MetalOptics {
    name: &'static str,
    n: f64,
    k: f64,
}

/// 屈折率をどこから得たか(`from_material_db` の戻り値、警告文の生成根拠)。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IorSource {
    /// `sim_core::Material::refractive_index`(実測の可視域屈折率)。
    RefractiveIndex,
    /// `sim_core::Material::relative_permittivity` から $n=\sqrt{\varepsilon_r}$
    /// (設計§3の可視域近似。**静的比誘電率由来なので信頼できない**、モジュールdoc参照)。
    RelativePermittivity,
    /// 設計§9の金属パラメータ表(`METAL_OPTICS`)。
    MetalTable,
    /// 設計§5の既定BSDF(誘電体 $n=1.5$)。
    Default,
}

/// 物性DBの1材質を描画マテリアルへ解決した結果。
#[derive(Clone, Debug)]
pub struct OpticalMaterial {
    /// パストレーサへ渡すBSDF。
    pub material: Material,
    /// 屈折率の出所。
    pub ior_source: IorSource,
    /// 縮退が起きた場合の警告(設計§5「+ 警告」)。縮退が無ければ `None`。
    pub warning: Option<String>,
}

/// `MaterialDb` の1材質を描画マテリアル(BSDF)へ解決する。
///
/// 解決順序(モジュールdocの「設計の縮退規則がそのままでは使えない事情」参照):
/// 1. 設計§9の金属パラメータ表に名前があれば `Metal { n, k }`。
/// 2. `refractive_index` が使える値(> `MIN_USEFUL_IOR`)なら `Dielectric`。
/// 3. $\sqrt{\varepsilon_r}$ が使える値なら `Dielectric`(**警告付き**、静的比誘電率由来)。
/// 4. いずれも無ければ設計§5の既定 `Dielectric { ior: 1.5 }`(**警告付き**)。
pub fn from_material_db(db: &MaterialDb, id: MaterialId) -> OpticalMaterial {
    let material = db.get(id);
    let name = material.name;

    if let Some(metal) = METAL_OPTICS.iter().find(|m| m.name == name) {
        return OpticalMaterial {
            material: Material::Metal(Metal {
                n: metal.n,
                k: metal.k,
            }),
            ior_source: IorSource::MetalTable,
            warning: None,
        };
    }

    if let Some(ior) = material.refractive_index {
        if ior > MIN_USEFUL_IOR {
            return OpticalMaterial {
                material: Material::Dielectric(Dielectric { ior }),
                ior_source: IorSource::RefractiveIndex,
                warning: None,
            };
        }
    }

    let from_permittivity = material.relative_permittivity.max(0.0).sqrt();
    if from_permittivity > MIN_USEFUL_IOR {
        return OpticalMaterial {
            material: Material::Dielectric(Dielectric {
                ior: from_permittivity,
            }),
            ior_source: IorSource::RelativePermittivity,
            warning: Some(format!(
                "{name}: 可視域屈折率が未収録のため静的比誘電率から n=sqrt({:.4})={:.4} を用いた\
                 (静的値は分子双極子の寄与を含むため可視域では過大になり得る)",
                material.relative_permittivity, from_permittivity
            )),
        };
    }

    OpticalMaterial {
        material: Material::Dielectric(Dielectric { ior: DEFAULT_IOR }),
        ior_source: IorSource::Default,
        warning: Some(format!(
            "{name}: 可視域屈折率も有効な比誘電率も未収録のため既定BSDF(誘電体 n={DEFAULT_IOR})を用いた"
        )),
    }
}

/// 標準DBの全材質を順に解決する(D41「材質ギャラリー」が球アレイを組むときの入口)。
pub fn all_from_standard_db(db: &MaterialDb) -> Vec<OpticalMaterial> {
    (0..db.len())
        .map(|i| from_material_db(db, MaterialId(i as u32)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 標準DB13材質すべてが解決でき、解決結果が仕様どおりの出所を持つこと。
    /// **モジュールdocの表(水の$\sqrt{\varepsilon_r}$=8.95問題)の回帰テストでもある**
    /// ——水が`RefractiveIndex`経由(=1.333)で解決されなくなったら、この橋は
    /// 静的比誘電率由来の8.95を使い始めてしまう。
    #[test]
    fn every_standard_material_resolves_and_water_uses_its_measured_visible_index() {
        let db = MaterialDb::standard();
        let resolved = all_from_standard_db(&db);
        assert_eq!(resolved.len(), db.len());

        let water_id = db.find_by_name("水").expect("標準DBに水がある");
        let water = from_material_db(&db, water_id);
        assert_eq!(water.ior_source, IorSource::RefractiveIndex);
        assert!(water.warning.is_none());
        match water.material {
            Material::Dielectric(d) => assert!(
                (d.ior - 1.333).abs() < 1e-12,
                "水は可視域屈折率1.333を使うべき(静的比誘電率由来の\
                 sqrt(80.1)=8.95ではない): ior={}",
                d.ior
            ),
            other => panic!("水は誘電体になるべき: {other:?}"),
        }
    }

    /// 設計§9の金属パラメータ表が引かれること、および表に無い金属(銅・鋼)は
    /// **推測値を捏造せず**警告付きで既定BSDFへ落ちること(モジュールdoc参照)。
    #[test]
    fn metals_in_the_design_table_resolve_to_conductors_and_others_warn() {
        let db = MaterialDb::standard();

        let aluminium = from_material_db(&db, db.find_by_name("アルミニウム").unwrap());
        assert_eq!(aluminium.ior_source, IorSource::MetalTable);
        match aluminium.material {
            Material::Metal(m) => {
                assert!((m.n - 0.96).abs() < 1e-12 && (m.k - 6.7).abs() < 1e-12);
            }
            other => panic!("アルミニウムは金属BSDFになるべき: {other:?}"),
        }

        for name in ["銅", "鋼(炭素鋼)"] {
            let resolved = from_material_db(&db, db.find_by_name(name).unwrap());
            assert_eq!(
                resolved.ior_source,
                IorSource::Default,
                "{name}は設計§9の表にもDBにもn+ikが無いため既定BSDFへ落ちるべき"
            );
            assert!(
                resolved.warning.is_some(),
                "{name}は縮退したので設計§5どおり警告を返すべき"
            );
        }
    }

    /// **D41の合格基準 R2(フレネル)**: 解決した誘電体のフレネル反射率が、
    /// 全ての入射角で反射+透過=1(エネルギー保存)を厳密に満たすこと。
    /// `bsdf.rs`のR2テストと同じく`sim_em::fresnel_reflectance`が正典。
    #[test]
    fn r2_every_resolved_dielectric_conserves_energy_at_all_incidence_angles() {
        let db = MaterialDb::standard();
        for resolved in all_from_standard_db(&db) {
            let Material::Dielectric(d) = resolved.material else {
                continue;
            };
            for i in 0..=90 {
                let theta = (i as f64).to_radians();
                let r = d.reflectance(theta.cos());
                assert!(
                    (0.0..=1.0).contains(&r),
                    "フレネル反射率は[0,1]: ior={} theta={i}deg r={r}",
                    d.ior
                );
                // 全反射でなければ透過率T=1-Rが正(エネルギー保存の直接形)。
                assert!(
                    (1.0 - r) >= -1e-12,
                    "反射+透過=1を満たすべき: ior={} theta={i}deg r={r}",
                    d.ior
                );
            }
            // 垂直入射の閉形式 $R_0=((n-1)/(n+1))^2$ と厳密一致。
            let r0 = d.reflectance(1.0);
            let expected = ((d.ior - 1.0) / (d.ior + 1.0)).powi(2);
            assert!(
                (r0 - expected).abs() < 1e-9,
                "垂直入射のフレネル反射率が閉形式と一致すべき: ior={} r0={r0} expected={expected}",
                d.ior
            );
        }
    }

    /// **D41の合格基準 R2(金属側)**: 解決した金属の反射率が全入射角で[0,1]に収まり、
    /// 垂直入射で$R_0=((n-1)^2+k^2)/((n+1)^2+k^2)$の閉形式と厳密一致すること。
    #[test]
    fn r2_every_resolved_metal_matches_the_conductor_reflectance_closed_form() {
        let db = MaterialDb::standard();
        for resolved in all_from_standard_db(&db) {
            let Material::Metal(m) = resolved.material else {
                continue;
            };
            for i in 0..=89 {
                let theta = (i as f64).to_radians();
                let r = m.reflectance(theta.cos());
                assert!(
                    (0.0..=1.0).contains(&r),
                    "金属反射率は[0,1]: n={} k={} theta={i}deg r={r}",
                    m.n,
                    m.k
                );
            }
            let r0 = m.reflectance(1.0);
            let expected = ((m.n - 1.0).powi(2) + m.k * m.k) / ((m.n + 1.0).powi(2) + m.k * m.k);
            assert!(
                (r0 - expected).abs() < 1e-9,
                "垂直入射の金属反射率が閉形式と一致すべき: n={} k={} r0={r0} expected={expected}",
                m.n,
                m.k
            );
        }
    }
}
