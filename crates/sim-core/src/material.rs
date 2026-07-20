//! 材料物性データベース。設計: docs/12-thermal/04-material-thermal-props.md。
//!
//! 全ドメイン横断の物性を単一の DB に一元化する(密度→質量、摩擦・反発、
//! 熱物性、電磁気物性)。crate: sim-core(熱の列が最多のためこの章で定義、
//! [docs] 冒頭の規約通り)。

use std::collections::BTreeMap;

/// `MaterialDb` 内へのインデックス(世代なし、DB は不変)。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Hash)]
pub struct MaterialId(pub u32);

/// 相変化パラメータ(融点・凝固潜熱・沸点・気化潜熱)。P3 相変化ウェーブで使用。
#[derive(Clone, Copy, Debug)]
pub struct PhaseChangeProps {
    pub melting_point: f64,
    pub latent_heat_fusion: f64,
    pub boiling_point: f64,
    pub latent_heat_vaporization: f64,
}

/// 設計 §2 の Rust 型定義そのまま。
#[derive(Clone, Debug)]
pub struct Material {
    pub name: &'static str,
    // 力学
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
    pub youngs_modulus: Option<f64>,
    // 熱
    pub specific_heat: f64,
    pub conductivity: f64,
    pub emissivity: f64,
    pub melting: Option<PhaseChangeProps>,
    // 電磁気
    pub resistivity: Option<f64>,
    pub relative_permittivity: f64,
    pub refractive_index: Option<f64>,
    // メタ
    pub source: &'static str,
    pub uncertainty: f64,
}

/// 摩擦・反発のペア別上書き。実測値が知られる組(ゴム-アスファルト等)用
/// (docs/10-mechanics/04-friction.md)。
#[derive(Clone, Copy, Debug)]
pub struct PairOverride {
    pub friction: f64,
    pub restitution: f64,
}

fn pair_key(a: MaterialId, b: MaterialId) -> (MaterialId, MaterialId) {
    if a.0 <= b.0 {
        (a, b)
    } else {
        (b, a)
    }
}

pub struct MaterialDb {
    materials: Vec<Material>,
    friction_pairs: BTreeMap<(MaterialId, MaterialId), PairOverride>,
}

impl MaterialDb {
    pub fn empty() -> MaterialDb {
        MaterialDb {
            materials: Vec::new(),
            friction_pairs: BTreeMap::new(),
        }
    }

    /// docs/12-thermal/04-material-thermal-props.md §3 の標準物性表(CRC Handbook 103rd ed.、
    /// Incropera 付録、反発係数は代表実測)をロード順固定で構築する。
    pub fn standard() -> MaterialDb {
        let mut db = MaterialDb::empty();
        for m in standard_materials() {
            db.push(m);
        }
        db
    }

    pub fn push(&mut self, m: Material) -> MaterialId {
        let id = MaterialId(self.materials.len() as u32);
        self.materials.push(m);
        id
    }

    pub fn len(&self) -> usize {
        self.materials.len()
    }

    pub fn is_empty(&self) -> bool {
        self.materials.is_empty()
    }

    pub fn get(&self, id: MaterialId) -> &Material {
        &self.materials[id.0 as usize]
    }

    pub fn find_by_name(&self, name: &str) -> Option<MaterialId> {
        self.materials
            .iter()
            .position(|m| m.name == name)
            .map(|i| MaterialId(i as u32))
    }

    /// 温度依存物性の拡張点(P5)。当面は定数を返す
    /// (docs/12-thermal/04-material-thermal-props.md §2 の設計どおり、シグネチャを先に固定)。
    pub fn conductivity_at(&self, id: MaterialId, _temperature_k: f64) -> f64 {
        self.get(id).conductivity
    }

    pub fn set_friction_pair(&mut self, a: MaterialId, b: MaterialId, over: PairOverride) {
        self.friction_pairs.insert(pair_key(a, b), over);
    }

    /// ペア表があれば優先、無ければ単一値の幾何平均(docs/10-mechanics/04-friction.md §... の規約)。
    pub fn friction_pair(&self, a: MaterialId, b: MaterialId) -> f64 {
        if let Some(over) = self.friction_pairs.get(&pair_key(a, b)) {
            return over.friction;
        }
        (self.get(a).friction * self.get(b).friction).sqrt()
    }

    pub fn restitution_pair(&self, a: MaterialId, b: MaterialId) -> f64 {
        if let Some(over) = self.friction_pairs.get(&pair_key(a, b)) {
            return over.restitution;
        }
        (self.get(a).restitution * self.get(b).restitution).sqrt()
    }
}

impl Default for MaterialDb {
    fn default() -> Self {
        MaterialDb::empty()
    }
}

fn standard_materials() -> Vec<Material> {
    vec![
        Material {
            name: "鋼(炭素鋼)",
            density: 7850.0,
            friction: 0.6,
            restitution: 0.6,
            youngs_modulus: Some(200.0e9),
            specific_heat: 490.0,
            conductivity: 50.0,
            emissivity: 0.6,
            melting: None,
            resistivity: Some(1.4e-7),
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
        Material {
            name: "アルミニウム",
            density: 2700.0,
            friction: 0.5,
            restitution: 0.5,
            youngs_modulus: Some(69.0e9),
            specific_heat: 900.0,
            conductivity: 237.0,
            emissivity: 0.1,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
        Material {
            name: "銅",
            density: 8960.0,
            friction: 0.5,
            restitution: 0.5,
            youngs_modulus: Some(117.0e9),
            specific_heat: 385.0,
            conductivity: 401.0,
            emissivity: 0.05,
            melting: None,
            resistivity: Some(1.68e-8),
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
        Material {
            name: "ガラス",
            density: 2500.0,
            friction: 0.5,
            restitution: 0.7,
            youngs_modulus: Some(70.0e9),
            specific_heat: 840.0,
            conductivity: 1.0,
            emissivity: 0.92,
            melting: None,
            resistivity: Some(1.0e12),
            // 出典は範囲値(5-10)を示すのみ。代表値としてソーダ石灰ガラス相当を採用。
            relative_permittivity: 6.0,
            refractive_index: Some(1.52),
            source: "CRC Handbook 103rd ed. (ε_r は範囲5-10の代表値)",
            uncertainty: 0.3,
        },
        Material {
            name: "コンクリート",
            density: 2400.0,
            friction: 0.7,
            restitution: 0.2,
            youngs_modulus: Some(30.0e9),
            specific_heat: 880.0,
            conductivity: 1.4,
            emissivity: 0.9,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "Incropera Fundamentals of Heat and Mass Transfer 付録",
            uncertainty: 0.3,
        },
        Material {
            name: "木材(松)",
            density: 500.0,
            friction: 0.45,
            restitution: 0.4,
            youngs_modulus: Some(9.0e9),
            specific_heat: 1700.0,
            conductivity: 0.12,
            emissivity: 0.9,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "Incropera Fundamentals of Heat and Mass Transfer 付録",
            uncertainty: 0.3,
        },
        Material {
            name: "ゴム(天然)",
            density: 920.0,
            friction: 0.9,
            restitution: 0.8,
            youngs_modulus: Some(0.05e9),
            specific_heat: 1900.0,
            conductivity: 0.16,
            emissivity: 0.94,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
        Material {
            name: "氷(0°C)",
            density: 916.7,
            friction: 0.05,
            restitution: 0.1,
            youngs_modulus: Some(9.0e9),
            specific_heat: 2100.0,
            conductivity: 2.2,
            emissivity: 0.96,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
        Material {
            name: "水",
            density: 998.2,
            // 流体には剛体接触の摩擦・反発が意味を持たないためプレースホルダ 0.0。
            friction: 0.0,
            restitution: 0.0,
            youngs_modulus: None,
            specific_heat: 4182.0,
            conductivity: 0.60,
            emissivity: 0.96,
            melting: None,
            resistivity: Some(2.0e5),
            relative_permittivity: 80.1,
            refractive_index: Some(1.333),
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.05,
        },
        Material {
            name: "空気",
            density: 1.204,
            friction: 0.0,
            restitution: 0.0,
            youngs_modulus: None,
            specific_heat: 1005.0,
            conductivity: 0.026,
            emissivity: 0.0,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0006,
            refractive_index: Some(1.000293),
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.05,
        },
        Material {
            name: "発泡スチロール",
            density: 30.0,
            friction: 0.4,
            restitution: 0.6,
            youngs_modulus: Some(0.005e9),
            specific_heat: 1300.0,
            conductivity: 0.033,
            emissivity: 0.9,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
        Material {
            name: "人体(平均)",
            density: 1010.0,
            friction: 0.0,
            restitution: 0.0,
            youngs_modulus: None,
            specific_heat: 3500.0,
            conductivity: 0.5,
            emissivity: 0.98,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "Incropera Fundamentals of Heat and Mass Transfer 付録",
            uncertainty: 0.3,
        },
        Material {
            name: "PTFE(テフロン)",
            density: 2200.0,
            friction: 0.04,
            restitution: 0.4,
            youngs_modulus: Some(0.5e9),
            specific_heat: 1000.0,
            conductivity: 0.25,
            emissivity: 0.9,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "CRC Handbook 103rd ed.",
            uncertainty: 0.3,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_db_has_all_thirteen_materials() {
        let db = MaterialDb::standard();
        assert_eq!(db.len(), 13);
    }

    #[test]
    fn standard_db_load_order_is_deterministic() {
        let a = MaterialDb::standard();
        let b = MaterialDb::standard();
        assert_eq!(a.len(), b.len());
        for i in 0..a.len() {
            let id = MaterialId(i as u32);
            assert_eq!(a.get(id).name, b.get(id).name);
        }
    }

    #[test]
    fn find_by_name_resolves_known_material() {
        let db = MaterialDb::standard();
        let id = db.find_by_name("銅").expect("copper must exist");
        assert_eq!(db.get(id).density, 8960.0);
    }

    /// 設計 §5: 派生量での相互検証(鋼の音速 ≈ 5000 m/s)。
    #[test]
    fn steel_speed_of_sound_matches_known_value() {
        let db = MaterialDb::standard();
        let id = db.find_by_name("鋼(炭素鋼)").unwrap();
        let m = db.get(id);
        let e = m.youngs_modulus.expect("steel has Young's modulus");
        let c = (e / m.density).sqrt();
        assert!(
            (c - 5000.0).abs() / 5000.0 < 0.1,
            "speed of sound {c} far from ~5000 m/s"
        );
    }

    /// 設計 §5: 熱拡散率 α=k/(ρc_p) が文献値と桁で一致することを確認する
    /// (厳密な数値一致は物性の個体差が大きく脆いテストになるため、桁の範囲で検証する)。
    #[test]
    fn thermal_diffusivity_matches_literature_order_of_magnitude() {
        let db = MaterialDb::standard();
        let alpha = |name: &str| -> f64 {
            let id = db.find_by_name(name).unwrap();
            let m = db.get(id);
            m.conductivity / (m.density * m.specific_heat)
        };
        assert!((5e-6..5e-5).contains(&alpha("鋼(炭素鋼)")));
        assert!((5e-5..2e-4).contains(&alpha("銅")));
        assert!((5e-8..5e-7).contains(&alpha("水")));
        assert!((5e-6..1e-4).contains(&alpha("空気")));
    }

    #[test]
    fn friction_pair_override_is_symmetric() {
        let mut db = MaterialDb::standard();
        let steel = db.find_by_name("鋼(炭素鋼)").unwrap();
        let rubber = db.find_by_name("ゴム(天然)").unwrap();
        db.set_friction_pair(
            steel,
            rubber,
            PairOverride {
                friction: 0.8,
                restitution: 0.3,
            },
        );
        assert_eq!(
            db.friction_pair(steel, rubber),
            db.friction_pair(rubber, steel)
        );
        assert_eq!(db.friction_pair(steel, rubber), 0.8);
    }

    #[test]
    fn friction_pair_falls_back_to_geometric_mean_without_override() {
        let db = MaterialDb::standard();
        let steel = db.find_by_name("鋼(炭素鋼)").unwrap();
        let ice = db.find_by_name("氷(0°C)").unwrap();
        let expected = (db.get(steel).friction * db.get(ice).friction).sqrt();
        assert_eq!(db.friction_pair(steel, ice), expected);
    }
}
