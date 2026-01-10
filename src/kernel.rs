use super::H;
use macroquad::prelude::*;
use std::f32::consts::PI;

#[allow(unused)]
pub fn poly6(x: Vec2) -> f32 {
    const H2: f32 = H * H;
    const H8: f32 = H2 * H2 * H2 * H2;

    let r2 = x.length_squared();
    if r2 < H.powi(2) {
        4. / (PI * H8) * (H2 - r2).powi(3)
    } else {
        0.
    }
}

#[allow(unused)]
pub fn grad_poly6(x: Vec2) -> Vec2 {
    const H2: f32 = H * H;
    const H8: f32 = H2 * H2 * H2 * H2;

    let r2 = x.length_squared();
    if r2 < H.powi(2) {
        -24. / (PI * H8) * (H2 - r2).powi(2) * x
    } else {
        Vec2::ZERO
    }
}

#[allow(unused)]
pub fn lap_poly6(x: Vec2) -> f32 {
    const H2: f32 = H * H;
    const H8: f32 = H2 * H2 * H2 * H2;

    let r2 = x.length_squared();
    if r2 < H.powi(2) {
        48. / (PI * H8) * (H2 - r2).powi(2) * (3. * r2 - H2)
    } else {
        0.
    }
}

#[allow(unused)]
pub fn spiky(x: Vec2) -> f32 {
    const H5: f32 = H * H * H * H * H;

    let r = x.length();
    if r < H {
        10. / (PI * H5) * (H - r).powi(3)
    } else {
        0.
    }
}

#[allow(unused)]
pub fn grad_spiky(x: Vec2) -> Vec2 {
    const H5: f32 = H * H * H * H * H;

    let r = x.length();
    if 0. < r && r < H {
        -30. / (PI * H5) * (H - r).powi(2) * (x / r)
    } else {
        Vec2::ZERO
    }
}

#[allow(unused)]
pub fn lap_visc(x: Vec2) -> f32 {
    const H5: f32 = H * H * H * H * H;

    let r = x.length();
    if r < H {
        20. / (3. * PI * H5) * (H - r)
    } else {
        0.
    }
}
