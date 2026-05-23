mod device_motion;
mod kernel;

use crate::device_motion::get_acceleration;
use crate::kernel::*;
use macroquad::prelude::*;
use rayon::prelude::*;

const RADIUS: f32 = 5.;
const STEPS_PER_FRAME: usize = 30;

const G_SCALE: f32 = 50.;
// kernel size
const H: f32 = 20.;
// mass
const M: f32 = 1.;
// stiffness
const K: f32 = 1e8;
// viscosity
const MU: f32 = 0.001;
const EPS: f32 = 0.5;

const DT: f32 = 1. / (30. * STEPS_PER_FRAME as f32);

#[inline]
fn cell_idx(pos: &Vec2, num_cols: usize) -> usize {
    let cx = (pos.x / H).round() as usize;
    let cy = (pos.y / H).round() as usize;
    cy * num_cols + cx
}
struct SpatialHashGrid {
    num_cols: usize,
    num_rows: usize,
    starts: Vec<u32>,
    ends: Vec<u32>,
    ids: Vec<u32>,
}

impl SpatialHashGrid {
    fn new(w: f32, h: f32, num_particles: usize) -> Self {
        let num_cols = (w / H).ceil() as usize + 1;
        let num_rows = (h / H).ceil() as usize + 1;

        let num_cells = num_cols * num_rows;

        Self {
            num_cols,
            num_rows,
            starts: vec![0; num_cells],
            ends: vec![0; num_cells],
            ids: vec![0; num_particles],
        }
    }

    fn rebuild(&mut self, pos: &[Vec2]) {
        let num_cols = self.num_cols;

        self.ends.fill(0);

        for p in pos {
            if cell_idx(p, num_cols) >= self.ends.len() {
                println!("{p:?}");
            }
            self.ends[cell_idx(p, num_cols)] += 1;
        }

        let mut total = 0;
        for i in 0..self.starts.len() {
            self.starts[i] = total;
            total += self.ends[i];
            self.ends[i] = self.starts[i];
        }

        for (i, p) in pos.iter().enumerate() {
            let c = cell_idx(p, num_cols);
            self.ids[self.ends[c] as usize] = i as u32;
            self.ends[c] += 1;
        }
    }

    fn query_neighbours<'a>(&'a self, pos: &Vec2) -> impl Iterator<Item = u32> + 'a {
        let cx = (pos.x / H).round() as isize;
        let cy = (pos.y / H).round() as isize;

        (-1..=1).flat_map(move |dy| {
            (-1..=1)
                .filter_map(move |dx| {
                    let ny: usize = (cy + dy).try_into().ok()?;
                    let nx: usize = (cx + dx).try_into().ok()?;
                    (ny < self.num_rows && nx < self.num_cols).then(|| {
                        let c = ny * self.num_cols + nx;
                        self.ids[self.starts[c] as usize..self.ends[c] as usize]
                            .iter()
                            .copied()
                    })
                })
                .flatten()
        })
    }
}

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
            grid: SpatialHashGrid::new(w, h, num_particles),
            rho0: 0.,
            pos,
            vel: vec![Vec2::ZERO; num_particles],
            force: vec![Vec2::ZERO; num_particles],
            rho: vec![1.; num_particles],
            p: vec![0.; num_particles],
        }
    }

    fn update_density(&mut self) {
        self.rho.par_iter_mut().enumerate().for_each(|(i, rho)| {
            *rho = 0.;

            for j in self.grid.query_neighbours(&self.pos[i]) {
                *rho += M * poly6(self.pos[i] - self.pos[j as usize]);
            }
        });

        if self.rho0 == 0. {
            self.rho0 = self.rho.iter().sum::<f32>() / self.num_particles as f32;
        }
    }

    fn update_pressure(&mut self) {
        for i in 0..self.num_particles {
            self.p[i] = (K * (self.rho[i] - self.rho0)).max(0.);
        }
    }

    fn update_force(&mut self) {
        let gravity_accel = get_acceleration() * G_SCALE;

        self.force.par_iter_mut().enumerate().for_each(|(i, f)| {
            *f = self.rho[i] * gravity_accel;

            for j in self.grid.query_neighbours(&self.pos[i]) {
                let j = j as usize;
                if i != j {
                    // Pressure force
                    *f += -M * (self.p[i] + self.p[j]) / (2. * self.rho[j])
                        * grad_spiky(self.pos[i] - self.pos[j]);

                    // Viscosity force
                    *f += MU * M * (self.vel[j] - self.vel[i]) / self.rho[j]
                        * lap_visc(self.pos[i] - self.pos[j]);
                }
            }
        });
    }

    fn apply_xsph(&mut self) {
        let dv = (0..self.num_particles)
            .into_par_iter()
            .map(|i| {
                let mut corr = Vec2::ZERO;

                for j in self.grid.query_neighbours(&self.pos[i]) {
                    let j = j as usize;
                    if i != j {
                        let r = self.pos[i] - self.pos[j];
                        corr += M * (self.vel[j] - self.vel[i]) / self.rho[j] * poly6(r);
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
        let COEF_RESTITUTION = 0.2;
        for i in 0..self.num_particles {
            let pos = &mut self.pos[i];
            let vel = &mut self.vel[i];

            if pos.x < RADIUS {
                pos.x = RADIUS;
                vel.x *= -COEF_RESTITUTION;
            }
            if pos.x > w - RADIUS {
                pos.x = w - RADIUS;
                vel.x *= -COEF_RESTITUTION;
            }
            if pos.y < RADIUS {
                pos.y = RADIUS;
                vel.y *= -COEF_RESTITUTION;
            }
            if pos.y > h - RADIUS {
                pos.y = h - RADIUS;
                vel.y *= -COEF_RESTITUTION;
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

fn window_conf() -> Conf {
    Conf {
        window_title: "SPH Fluid Simulation".to_string(),
        platform: miniquad::conf::Platform {
            swap_interval: Some(2), // target fps = 30
            ..Default::default()
        },
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut w = screen_width();
    let mut h = screen_height();

    let mut scene = Scene::new(w, h);

    loop {
        let new_w = screen_width();
        let new_h = screen_height();

        if new_w != w || new_h != h {
            scene.grid = SpatialHashGrid::new(new_w, new_h, scene.num_particles);
            w = new_w;
            h = new_h;
        }

        for _ in 0..STEPS_PER_FRAME {
            for i in 0..scene.num_particles {
                scene.vel[i] += scene.force[i] / scene.rho[i] * DT / 2.;
                scene.pos[i] += scene.vel[i] * DT;
            }

            scene.handle_wall_collision(w, h);

            scene.grid.rebuild(&scene.pos);

            scene.update_density();
            scene.update_pressure();
            scene.update_force();

            scene.apply_xsph();

            for i in 0..scene.num_particles {
                scene.vel[i] += scene.force[i] / scene.rho[i] * DT / 2.;
            }
        }

        scene.render();

        next_frame().await;
    }
}
