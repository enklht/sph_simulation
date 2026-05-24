mod device_motion;
mod grid;
mod kernel;

use crate::device_motion::get_acceleration;
use crate::grid::SpatialHashGrid;
use crate::kernel::*;
use macroquad::prelude::*;
use rayon::prelude::*;

const PIXELS_PER_CM: f32 = 100.;

const RADIUS: f32 = 0.06;
const SPACE: f32 = RADIUS * 2.;
// kernel size
const H: f32 = RADIUS * 3.;

// rest_density
const RHO0: f32 = 0.1;
// mass
const M: f32 = RHO0 * SPACE * SPACE;
// stiffness
const K: f32 = 100.;
// viscosity
const MU: f32 = 0.1;
// xsph strength
const EPS: f32 = 0.2;

const COEF_RESTITUTION: f32 = 0.1;

const STEPS_PER_FRAME: usize = 10;
const DT: f32 = 1. / (30. * STEPS_PER_FRAME as f32);

struct Buffers {
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    p: Vec<f32>,
    force: Vec<Vec2>,
    corrections: Vec<Vec2>,
}

impl Buffers {
    fn new(n: usize) -> Self {
        Self {
            pos: vec![Vec2::ZERO; n],
            vel: vec![Vec2::ZERO; n],
            p: vec![0.; n],
            force: vec![Vec2::ZERO; n],
            corrections: vec![Vec2::ZERO; n],
        }
    }

    fn reorder(
        &mut self,
        ids: &[u32],
        pos: &mut Vec<Vec2>,
        vel: &mut Vec<Vec2>,
        p: &mut Vec<f32>,
        force: &mut Vec<Vec2>,
        corrections: &mut Vec<Vec2>,
    ) {
        for (idx, &id) in ids.iter().enumerate() {
            let i = id as usize;
            self.pos[idx] = pos[i];
            self.vel[idx] = vel[i];
            self.p[idx] = p[i];
            self.force[idx] = force[i];
            self.corrections[idx] = corrections[i];
        }

        std::mem::swap(pos, &mut self.pos);
        std::mem::swap(vel, &mut self.vel);
        std::mem::swap(p, &mut self.p);
        std::mem::swap(force, &mut self.force);
        std::mem::swap(corrections, &mut self.corrections);
    }
}

struct Scene {
    num_particles: usize,
    grid: SpatialHashGrid,
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    force: Vec<Vec2>,
    rho: Vec<f32>,
    p: Vec<f32>,
    corrections: Vec<Vec2>,
    buffers: Buffers,
}

impl Scene {
    fn new(w: f32, h: f32) -> Scene {
        let cols = ((w - 2.0 * SPACE) / 3.0 / SPACE).floor() as usize;
        let rows = ((h - 2.0 * SPACE) / SPACE).floor() as usize;

        let num_particles = cols * rows;

        let mut pos = Vec::with_capacity(num_particles);

        for row in 0..rows {
            for col in 0..cols {
                let mut x = SPACE + col as f32 * SPACE;
                let y = SPACE + row as f32 * SPACE;

                // stagger every other row
                if row % 2 == 1 {
                    x += SPACE * 0.5;
                }

                pos.push(vec2(x, y));
            }
        }

        Scene {
            num_particles,
            grid: SpatialHashGrid::new(w, h, num_particles),
            pos,
            vel: vec![Vec2::ZERO; num_particles],
            rho: vec![RHO0; num_particles],
            p: vec![0.; num_particles],
            force: vec![Vec2::ZERO; num_particles],
            corrections: vec![Vec2::ZERO; num_particles],
            buffers: Buffers::new(num_particles),
        }
    }

    fn update_density_pressure(&mut self) {
        self.rho
            .par_iter_mut()
            .zip(self.p.par_iter_mut())
            .enumerate()
            .for_each(|(i, (rho, p))| {
                *rho = 0.;

                self.grid.for_each_neighbour(i, |j| {
                    *rho += M * poly6(self.pos[i] - self.pos[j]);
                });

                *p = K * (*rho - RHO0).clamp(1e-6, 1e6);
            });
    }

    fn update_force(&mut self) {
        let gravity_accel = get_acceleration();

        self.force.par_iter_mut().enumerate().for_each(|(i, f)| {
            *f = self.rho[i] * gravity_accel;

            self.grid.for_each_neighbour(i, |j| {
                if i != j {
                    let r = self.pos[i] - self.pos[j];
                    // Pressure force
                    *f += -M * (self.p[i] + self.p[j]) / (2. * self.rho[j]) * grad_spiky(r);

                    // Viscosity force
                    *f += -MU * M * (self.vel[i] - self.vel[j]) / self.rho[j] * lap_visc(r);
                }
            });
        });
    }

    fn update_vel(&mut self) {
        self.vel.par_iter_mut().enumerate().for_each(|(i, v)| {
            *v += self.force[i] / self.rho[i] * DT;
        });
    }

    fn apply_xsph(&mut self) {
        self.corrections
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, correction)| {
                *correction = Vec2::ZERO;

                self.grid.for_each_neighbour(i, |j| {
                    if i != j {
                        let r = self.pos[i] - self.pos[j];
                        *correction += M * (self.vel[j] - self.vel[i]) / self.rho[j] * poly6(r);
                    }
                });
                *correction *= EPS;
            });

        self.vel.par_iter_mut().enumerate().for_each(|(i, v)| {
            *v += self.corrections[i];
        });
    }

    fn update_pos_with_collision(&mut self, w: f32, h: f32) {
        self.pos
            .par_iter_mut()
            .zip(self.vel.par_iter_mut())
            .enumerate()
            .for_each(|(_i, (pos, vel))| {
                const EPSILON: f32 = 1e-5;

                *pos += *vel * DT;

                if pos.x < RADIUS {
                    pos.x = RADIUS + EPSILON;
                    vel.x = vel.x.abs() * COEF_RESTITUTION;
                }
                if pos.x > w - RADIUS {
                    pos.x = w - RADIUS - EPSILON;
                    vel.x = -vel.x.abs() * COEF_RESTITUTION;
                }
                if pos.y < RADIUS {
                    pos.y = RADIUS + EPSILON;
                    vel.y = vel.y.abs() * COEF_RESTITUTION;
                }
                if pos.y > h - RADIUS {
                    pos.y = h - RADIUS - EPSILON;
                    vel.y = -vel.y.abs() * COEF_RESTITUTION;
                }
            });
    }

    fn reorder_particles(&mut self) {
        // rho will be reset on write soon after this operation, so we dont have to reorder it
        self.buffers.reorder(
            &self.grid.ids,
            &mut self.pos,
            &mut self.vel,
            &mut self.p,
            &mut self.force,
            &mut self.corrections,
        );
    }

    fn render(&self) {
        const BACKGROUND_COLOR: Color = Color::new(0.2, 0.2, 0.2, 1.0);
        const WATER_COLOR: Color = Color::new(0.3, 0.85, 0.95, 0.9);

        clear_background(BACKGROUND_COLOR);

        for i in 0..self.pos.len() {
            let p = self.pos[i] * PIXELS_PER_CM;
            draw_circle(p.x, p.y, RADIUS * 1.5 * PIXELS_PER_CM, WATER_COLOR);
        }
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "SPH Fluid Simulation".to_string(),
        window_width: 1920,
        window_height: 1280,
        platform: miniquad::conf::Platform {
            swap_interval: Some(2), // target fps = 30
            ..Default::default()
        },
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut w_px = screen_width();
    let mut h_px = screen_height();
    let mut w = w_px / PIXELS_PER_CM;
    let mut h = h_px / PIXELS_PER_CM;

    let mut scene = Scene::new(w, h);

    loop {
        let new_w_px = screen_width();
        let new_h_px = screen_height();

        if new_w_px != w_px || new_h_px != h_px {
            w_px = new_w_px;
            h_px = new_h_px;
            w = w_px / PIXELS_PER_CM;
            h = h_px / PIXELS_PER_CM;
            scene.grid = SpatialHashGrid::new(w, h, scene.num_particles);
        }

        for _ in 0..STEPS_PER_FRAME {
            scene.update_vel();

            scene.apply_xsph();

            scene.update_pos_with_collision(w, h);

            scene.grid.update(&scene.pos);
            scene.reorder_particles();

            scene.update_density_pressure();
            scene.update_force();
        }

        scene.render();

        next_frame().await;
    }
}
