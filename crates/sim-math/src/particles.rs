//! 粒子集合と近傍探索(空間ハッシュ)。設計: docs/01-math/02-fields.md §6。

use crate::Vec3;
use std::collections::BTreeMap;

/// SPH・気体分子・ブラウン粒子が共有する SoA コンテナ。設計 §6。
pub struct ParticleSet {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub mass: Vec<f64>,
    /// ドメイン固有の属性(密度・温度・電荷 …)。反復順序依存を避けるため `BTreeMap`
    /// (docs/20-integration/02-determinism-replay.md §2)。
    pub extra_f64: BTreeMap<&'static str, Vec<f64>>,
}

impl ParticleSet {
    pub fn new() -> ParticleSet {
        ParticleSet {
            position: Vec::new(),
            velocity: Vec::new(),
            mass: Vec::new(),
            extra_f64: BTreeMap::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.position.len()
    }

    pub fn is_empty(&self) -> bool {
        self.position.is_empty()
    }
}

impl Default for ParticleSet {
    fn default() -> Self {
        ParticleSet::new()
    }
}

const HASH_PX: i64 = 73856093;
const HASH_PY: i64 = 19349663;
const HASH_PZ: i64 = 83492791;

/// 空間ハッシュによる近傍探索。設計 §6.1。
/// セル幅は相互作用半径(SPH のカーネル半径・分子の衝突判定半径)に合わせる規約。
#[derive(Clone)]
pub struct SpatialHash {
    cell: f64,
    table_size: usize,
    buckets: Vec<Vec<u32>>,
    positions: Vec<Vec3>,
}

impl SpatialHash {
    pub fn new(cell: f64, table_size: usize) -> SpatialHash {
        SpatialHash {
            cell,
            table_size,
            buckets: vec![Vec::new(); table_size],
            positions: Vec::new(),
        }
    }

    fn cell_of(&self, p: Vec3) -> (i64, i64, i64) {
        (
            (p.x / self.cell).floor() as i64,
            (p.y / self.cell).floor() as i64,
            (p.z / self.cell).floor() as i64,
        )
    }

    /// Teschner らの標準ハッシュ: (i,j,k) -> (i*P1 ^ j*P2 ^ k*P3) mod table_size。
    fn hash(&self, i: i64, j: i64, k: i64) -> usize {
        let h = i.wrapping_mul(HASH_PX) ^ j.wrapping_mul(HASH_PY) ^ k.wrapping_mul(HASH_PZ);
        h.rem_euclid(self.table_size as i64) as usize
    }

    /// 再構築(毎ステップ)。バケット内は粒子インデックス昇順に安定(決定論)。
    pub fn rebuild(&mut self, positions: &[Vec3]) {
        for bucket in &mut self.buckets {
            bucket.clear();
        }
        self.positions.clear();
        self.positions.extend_from_slice(positions);
        // positions は昇順に走査するので、各バケットへの push は自然にインデックス昇順になる。
        for (idx, &p) in positions.iter().enumerate() {
            let (i, j, k) = self.cell_of(p);
            let h = self.hash(i, j, k);
            self.buckets[h].push(idx as u32);
        }
    }

    /// p から半径 r 以内の粒子インデックスを昇順で返す。
    pub fn query(&self, p: Vec3, r: f64, out: &mut Vec<u32>) {
        out.clear();
        let (ci, cj, ck) = self.cell_of(p);
        let r_sq = r * r;
        // 既定はセル幅=相互作用半径の27近傍。r がセル幅を超える場合は走査範囲を広げる。
        let span = ((r / self.cell).ceil() as i64).max(1);
        for di in -span..=span {
            for dj in -span..=span {
                for dk in -span..=span {
                    let h = self.hash(ci + di, cj + dj, ck + dk);
                    for &idx in &self.buckets[h] {
                        let q = self.positions[idx as usize];
                        if (q - p).length_sq() <= r_sq {
                            out.push(idx);
                        }
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SimRng;

    fn brute_force_query(positions: &[Vec3], p: Vec3, r: f64) -> Vec<u32> {
        let r_sq = r * r;
        let mut out: Vec<u32> = positions
            .iter()
            .enumerate()
            .filter(|(_, &q)| (q - p).length_sq() <= r_sq)
            .map(|(i, _)| i as u32)
            .collect();
        out.sort_unstable();
        out
    }

    /// 設計 §7: 総当たり結果と完全一致(乱数配置 10^3 粒子 × 決定シード)。
    #[test]
    fn matches_brute_force_for_random_particles() {
        let mut rng = SimRng::new(2024, 42);
        let n = 1000;
        let positions: Vec<Vec3> = (0..n)
            .map(|_| {
                Vec3::new(
                    rng.range_f64(-10.0, 10.0),
                    rng.range_f64(-10.0, 10.0),
                    rng.range_f64(-10.0, 10.0),
                )
            })
            .collect();

        let cell = 0.5;
        let mut hash = SpatialHash::new(cell, 4096);
        hash.rebuild(&positions);

        let mut got = Vec::new();
        for trial in 0..50 {
            let query_point = positions[trial * 17 % n];
            let r = cell; // セル幅=相互作用半径の既定ケース
            hash.query(query_point, r, &mut got);
            let expected = brute_force_query(&positions, query_point, r);
            assert_eq!(got, expected, "mismatch for trial {trial}");
        }
    }

    #[test]
    fn particle_set_starts_empty() {
        let set = ParticleSet::new();
        assert!(set.is_empty());
        assert_eq!(set.len(), 0);
    }
}
