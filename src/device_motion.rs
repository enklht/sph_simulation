use macroquad::prelude::*;

#[cfg(target_arch = "wasm32")]
unsafe extern "C" {
    fn get_accel_x() -> f64;
    fn get_accel_y() -> f64;
    fn get_accel_z() -> f64;
}

pub fn get_acceleration() -> Vec2 {
    let acc = {
        #[cfg(target_arch = "wasm32")]
        unsafe {
            vec2(-get_accel_x() as f32, get_accel_y() as f32)
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            vec2(0., 10.)
        }
    };

    acc
}
