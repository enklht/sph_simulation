mod device_motion;
mod kernel;

use crate::device_motion::get_acceleration;
use crate::kernel::*;
use macroquad::prelude::*;
use rayon::prelude::*;
use std::collections::HashMap;

const RADIUS: f32 = 5.;
const STEPS_PER_FRAME: usize = 10;

const SIGMA: f32 = 20.;

const GAMMA: i32 = 7;
const B: f32 = 1.5;

const M: f32 = 1.;

const EPS: f32 = 0.1;

const DT: f32 = 1. / 60.;

type SpatialHashGrid = HashMap<(usize, usize), Vec<usize>>;

fn cell_coord(pos: &Vec2) -> (usize, usize) {
    (
        (pos.x / SIGMA).round() as usize,
        (pos.y / SIGMA).round() as usize,
    )
}

#[derive(Default)]
struct Scene {
    num_particles: usize,
    grid: SpatialHashGrid,
    rho0: f32,
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    acc: Vec<Vec2>,
    rho: Vec<f32>,
    p: Vec<f32>,
}

impl Scene {
    fn new(w: f32, h: f32) -> Scene {
        let space = 2.0 * RADIUS * 1.5;
        let pad = RADIUS * 2.0;

        let cols = ((w - 2.0 * pad) / space).floor() as usize;
        let rows = ((h - 2.0 * pad) / 3.0 / space).floor() as usize;

        let num_particles = cols * rows;

        let mut pos = Vec::with_capacity(num_particles);

        for row in 0..rows {
            for col in 0..cols {
                let mut x = pad + col as f32 * space;
                let y = pad + row as f32 * space;

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
            acc: vec![Vec2::ZERO; num_particles],
            rho: vec![0.; num_particles],
            p: vec![0.; num_particles],
        }
    }

    fn update_grid(&mut self) {
        let mut grid = SpatialHashGrid::new();

        for (i, p) in self.pos.iter().enumerate() {
            grid.entry(cell_coord(p)).or_default().push(i)
        }

        self.grid = grid;
    }

    fn update_density(&mut self) {
        self.rho.par_iter_mut().enumerate().for_each(|(i, rho)| {
            *rho = 0.;

            let cell = cell_coord(&self.pos[i]);

            for x in cell.0.saturating_sub(1)..=cell.0 + 1 {
                for y in cell.1.saturating_sub(1)..=cell.1 + 1 {
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
            self.p[i] = B * ((self.rho[i] / self.rho0).powi(GAMMA) - 1.);
        }
    }

    fn update_acceleration(&mut self) {
        let base_acc = get_acceleration();

        self.acc.par_iter_mut().enumerate().for_each(|(i, acc)| {
            *acc = base_acc;

            let cell = cell_coord(&self.pos[i]);

            for x in cell.0.saturating_sub(1)..=cell.0 + 1 {
                for y in cell.1.saturating_sub(1)..=cell.1 + 1 {
                    let Some(neighbours) = self.grid.get(&(x, y)) else {
                        continue;
                    };

                    for &j in neighbours {
                        if i != j {
                            *acc += -M
                                * (self.p[i] / self.rho[i].powi(2)
                                    + self.p[j] / self.rho[j].powi(2))
                                * grad_spiky(self.pos[i] - self.pos[j])
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

                for x in cell.0.saturating_sub(1)..=cell.0 + 1 {
                    for y in cell.1.saturating_sub(1)..=cell.1 + 1 {
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
            draw_circle(pos.x, pos.y, RADIUS * 2., WATER_COLOR);
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

            scene.update_grid();

            for i in 0..scene.num_particles {
                scene.vel[i] += scene.acc[i] * DT / 2.;
                scene.pos[i] += scene.vel[i] * DT;
            }

            scene.update_density();
            scene.update_pressure();
            scene.update_acceleration();

            scene.apply_xsph();

            for i in 0..scene.num_particles {
                scene.vel[i] += scene.acc[i] * DT / 2.;
            }

            scene.handle_wall_collision(w, h);
        }

        scene.render();
        next_frame().await;
    }
}
