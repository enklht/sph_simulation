use super::SIGMA;
use macroquad::prelude::*;
use std::f32::consts::PI;

#[allow(unused)]
pub fn poly6(x: Vec2) -> f32 {
    const SIGMA2: f32 = SIGMA * SIGMA;
    const SIGMA8: f32 = SIGMA2 * SIGMA2 * SIGMA2 * SIGMA2;

    let r2 = x.length_squared();
    if r2 < SIGMA.powi(2) {
        4. / (PI * SIGMA8) * (SIGMA2 - r2).powi(3)
    } else {
        0.
    }
}

#[allow(unused)]
pub fn grad_poly6(x: Vec2) -> Vec2 {
    const SIGMA2: f32 = SIGMA * SIGMA;
    const SIGMA8: f32 = SIGMA2 * SIGMA2 * SIGMA2 * SIGMA2;

    let r2 = x.length_squared();
    if r2 < SIGMA.powi(2) {
        -24. / (PI * SIGMA8) * (SIGMA2 - r2).powi(2) * x
    } else {
        Vec2::ZERO
    }
}

#[allow(unused)]
pub fn spiky(x: Vec2) -> f32 {
    const SIGMA5: f32 = SIGMA * SIGMA * SIGMA * SIGMA * SIGMA;

    let r = x.length();
    if r < SIGMA {
        10. / (PI * SIGMA5) * (SIGMA - r).powi(3)
    } else {
        0.
    }
}

#[allow(unused)]
pub fn grad_spiky(x: Vec2) -> Vec2 {
    const SIGMA5: f32 = SIGMA * SIGMA * SIGMA * SIGMA * SIGMA;

    let r = x.length();
    if 0. < r && r < SIGMA {
        -30. / (PI * SIGMA5) * (SIGMA - r).powi(2) * (x / r)
    } else {
        Vec2::ZERO
    }
}
