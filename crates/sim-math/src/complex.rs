//! 複素数(FFT・split-step Fourier 用の最小実装)。設計: docs/14-quantum/02-schrodinger-solver.md §3。

use std::ops::{Add, Mul, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex64 {
    pub re: f64,
    pub im: f64,
}

impl Complex64 {
    pub const ZERO: Complex64 = Complex64 { re: 0.0, im: 0.0 };

    pub fn new(re: f64, im: f64) -> Complex64 {
        Complex64 { re, im }
    }

    pub fn from_polar(r: f64, theta: f64) -> Complex64 {
        Complex64::new(r * theta.cos(), r * theta.sin())
    }

    pub fn conj(self) -> Complex64 {
        Complex64::new(self.re, -self.im)
    }

    pub fn norm_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn scale(self, s: f64) -> Complex64 {
        Complex64::new(self.re * s, self.im * s)
    }
}

impl Add for Complex64 {
    type Output = Complex64;
    fn add(self, rhs: Complex64) -> Complex64 {
        Complex64::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl Sub for Complex64 {
    type Output = Complex64;
    fn sub(self, rhs: Complex64) -> Complex64 {
        Complex64::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl Mul for Complex64 {
    type Output = Complex64;
    fn mul(self, rhs: Complex64) -> Complex64 {
        Complex64::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multiplication_matches_polar_angle_addition() {
        let a = Complex64::from_polar(2.0, 0.3);
        let b = Complex64::from_polar(3.0, 0.7);
        let c = a * b;
        let expected = Complex64::from_polar(6.0, 1.0);
        assert!((c.re - expected.re).abs() < 1e-12);
        assert!((c.im - expected.im).abs() < 1e-12);
    }
}
