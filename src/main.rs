mod device_motion;
mod kernel;

use crate::device_motion::get_acceleration;
use crate::kernel::*;
use macroquad::prelude::*;
use rayon::prelude::*;
use std::collections::HashMap;

const RADIUS: f32 = 5.;
const STEPS_PER_FRAME: usize = 5;

const G_SCALE: f32 = 20.;
// kernel size
const H: f32 = 25.;
// mass
const M: f32 = 10.;
// stiffness
const K: f32 = 1e7;
// viscosity
const MU: f32 = 1e3;
const EPS: f32 = 0.75;

const DT: f32 = 1. / (60. * STEPS_PER_FRAME as f32);

type SpatialHashGrid = HashMap<(usize, usize), Vec<usize>>;

#[inline]
fn cell_coord(pos: &Vec2) -> (usize, usize) {
    ((pos.x / H).round() as usize, (pos.y / H).round() as usize)
}

#[derive(Default)]
struct Scene {
    num_particles: usize,
    grid: SpatialHashGrid,
    rho0: f32,
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    force: Vec<Vec2>,
    rho: Vec<f32>,
    p: Vec<f32>,
}

impl Scene {
    fn new(w: f32, h: f32) -> Scene {
        let space = 2.0 * RADIUS;

        let cols = ((w - 2.0 * space) / space).floor() as usize;
        let rows = ((h - 2.0 * space) / 3.0 / space).floor() as usize;

        let num_particles = cols * rows;

        let mut pos = Vec::with_capacity(num_particles);

        for row in 0..rows {
            for col in 0..cols {
                let mut x = space + col as f32 * space;
                let y = space + row as f32 * space;

                // stagger every other row
                if row % 2 == 1 {
                    x += space * 0.5;
                }

                pos.push(vec2(x, y));
            }
        }

        Scene {
            num_particles,
            grid: HashMap::new(),
            rho0: 0.,
            pos,
            vel: vec![Vec2::ZERO; num_particles],
            force: vec![Vec2::ZERO; num_particles],
            rho: vec![1.; num_particles],
            p: vec![0.; num_particles],
        }
    }

    fn update_grid(&mut self) {
        self.grid.clear();

        for (i, p) in self.pos.iter().enumerate() {
            self.grid.entry(cell_coord(p)).or_default().push(i)
        }
    }

    fn update_density(&mut self) {
        self.rho.par_iter_mut().enumerate().for_each(|(i, rho)| {
            *rho = 0.;

            let cell = cell_coord(&self.pos[i]);

            for x in cell.0.saturating_sub(1)..=cell.0.saturating_add(1) {
                for y in cell.1.saturating_sub(1)..=cell.1.saturating_add(1) {
                    let Some(neighbours) = self.grid.get(&(x, y)) else {
                        continue;
                    };

                    for &j in neighbours {
                        *rho += M * poly6(self.pos[i] - self.pos[j]);
                    }
                }
            }
        });

        if self.rho0 == 0. {
            self.rho0 = self.rho.iter().sum::<f32>() / self.num_particles as f32;
        }
    }

    fn update_pressure(&mut self) {
        for i in 0..self.num_particles {
            self.p[i] = K * (self.rho[i] - self.rho0);
        }
    }

    fn update_force(&mut self) {
        let base_force = get_acceleration() * G_SCALE;

        self.force.par_iter_mut().enumerate().for_each(|(i, f)| {
            *f = base_force;
            let cell = cell_coord(&self.pos[i]);

            for x in cell.0.saturating_sub(1)..=cell.0.saturating_add(1) {
                for y in cell.1.saturating_sub(1)..=cell.1.saturating_add(1) {
                    let Some(neighbours) = self.grid.get(&(x, y)) else {
                        continue;
                    };

                    for &j in neighbours {
                        if i != j {
                            // Pressure force
                            *f += -M * (self.p[i] + self.p[j]) / (2. * self.rho[j])
                                * grad_spiky(self.pos[i] - self.pos[j]);

                            // Viscosity force
                            *f += MU * M * (self.vel[j] - self.vel[i]) / self.rho[j]
                                * lap_visc(self.pos[i] - self.pos[j]);
                        }
                    }
                }
            }
        });
    }

    fn apply_xsph(&mut self) {
        let dv = (0..self.num_particles)
            .into_par_iter()
            .map(|i| {
                let mut corr = Vec2::ZERO;
                let cell = cell_coord(&self.pos[i]);

                for x in cell.0.saturating_sub(1)..=cell.0.saturating_add(1) {
                    for y in cell.1.saturating_sub(1)..=cell.1.saturating_add(1) {
                        let Some(neighbours) = self.grid.get(&(x, y)) else {
                            continue;
                        };

                        for &j in neighbours {
                            if i != j {
                                let r = self.pos[i] - self.pos[j];
                                corr += M * (self.vel[j] - self.vel[i]) / self.rho[i] * poly6(r);
                            }
                        }
                    }
                }

                EPS * corr
            })
            .collect::<Vec<_>>();

        for i in 0..self.num_particles {
            self.vel[i] += dv[i];
        }
    }

    fn handle_wall_collision(&mut self, w: f32, h: f32) {
        for i in 0..self.num_particles {
            let pos = &mut self.pos[i];
            let vel = &mut self.vel[i];

            if pos.x < RADIUS {
                pos.x = RADIUS;
                vel.x = 0.;
            }
            if pos.x > w - RADIUS {
                pos.x = w - RADIUS;
                vel.x = 0.;
            }
            if pos.y < RADIUS {
                pos.y = RADIUS;
                vel.y = 0.;
            }
            if pos.y > h - RADIUS {
                pos.y = h - RADIUS;
                vel.y = 0.;
            }
        }
    }

    fn render(&self) {
        const BACKGROUND_COLOR: Color = Color::new(0.2, 0.2, 0.2, 1.0);
        const WATER_COLOR: Color = Color::new(0.3, 0.85, 0.95, 0.9);

        clear_background(BACKGROUND_COLOR);

        for pos in &self.pos {
            draw_circle(pos.x, pos.y, RADIUS * 1.5, WATER_COLOR);
        }
    }
}

#[macroquad::main("SPH")]
async fn main() {
    let w = screen_width();
    let h = screen_height();

    let mut scene = Scene::new(w, h);

    loop {
        for _ in 0..STEPS_PER_FRAME {
            let w = screen_width();
            let h = screen_height();

            for i in 0..scene.num_particles {
                scene.vel[i] += scene.force[i] / scene.rho[i] * DT / 2.;
                scene.pos[i] += scene.vel[i] * DT;
            }

            scene.update_grid();

            scene.update_density();
            scene.update_pressure();
            scene.update_force();

            scene.apply_xsph();

            for i in 0..scene.num_particles {
                scene.vel[i] += scene.force[i] / scene.rho[i] * DT / 2.;
            }

            scene.handle_wall_collision(w, h);
        }

        scene.render();
        next_frame().await;
    }
}
