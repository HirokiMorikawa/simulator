//! レイ。設計 docs/17-rendering/02-path-tracing.md §4。

use sim_math::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Ray {
        Ray {
            origin,
            direction: direction.normalize_or_zero(),
        }
    }

    pub fn at(&self, t: f64) -> Vec3 {
        self.origin + self.direction.scale(t)
    }
}
