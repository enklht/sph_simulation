mod device_motion;

use std::{collections::HashMap, f32::consts::PI};

use crate::device_motion::get_acceleration;
use macroquad::prelude::*;

const RADIUS: f32 = 5.;
const NUM_PARTICLES: usize = 1000;
const STEPS_PER_FRAME: usize = 10;

const SIGMA: f32 = 20.;

const GAMMA: f32 = 7.;
const B: f32 = 1.5;

const M: f32 = 1.;

const DT: f32 = 1. / 60.;

#[derive(Default)]
struct Particle {
    pos: Vec2,
    vel: Vec2,
    rho: f32,
    p: f32,
    f: Vec2,
}

impl Particle {
    fn cell_coord(&self) -> (usize, usize) {
        (
            (self.pos.x / SIGMA).floor() as usize,
            (self.pos.y / SIGMA).floor() as usize,
        )
    }
}

type SpatialHashGrid = HashMap<(usize, usize), Vec<usize>>;

fn spawn_particles(n: usize) -> Vec<Particle> {
    const SPACE: f32 = RADIUS * 1.5;
    (0..n)
        .map(|i| {
            let mut x = 50. + (i as f32 % 50.) * 2. * SPACE;
            let y = 50. + (i as f32 / 50.).floor() * 2. * SPACE;
            if (i / 50) % 2 == 0 {
                x += SPACE;
            }

            Particle {
                pos: vec2(x, y),
                ..Default::default()
            }
        })
        .collect()
}

fn render(particles: &[Particle]) {
    const BACKGROUND_COLOR: Color = Color::new(0.2, 0.2, 0.2, 1.0);
    const WATER_COLOR: Color = Color::new(0.3, 0.85, 0.95, 0.9);

    clear_background(BACKGROUND_COLOR);

    for p in particles {
        draw_circle(p.pos.x, p.pos.y, RADIUS * 2., WATER_COLOR);
    }
}

fn poly6(x: Vec2) -> f32 {
    let r = x.length_squared();
    if r < SIGMA.powi(2) {
        4. / (PI * SIGMA.powi(8)) * (SIGMA.powi(2) - r).powi(3)
    } else {
        0.
    }
}

fn grad_poly6(x: Vec2) -> Vec2 {
    let r = x.length_squared();
    if r < SIGMA.powi(2) {
        -24. / (PI * SIGMA.powi(8)) * (SIGMA.powi(2) - r).powi(2) * x
    } else {
        Vec2::ZERO
    }
}

fn grad_spiky(x: Vec2) -> Vec2 {
    let r = x.length();
    if r < SIGMA {
        -30. / (PI * SIGMA.powi(5)) * (SIGMA - r).powi(2) * (x / r)
    } else {
        Vec2::ZERO
    }
}

fn update_density(particles: &mut [Particle], grid: &SpatialHashGrid) {
    for i in 0..NUM_PARTICLES {
        let cell = particles[i].cell_coord();
        let mut rho = 0.;

        for x in cell.0.saturating_sub(1)..=cell.0 + 1 {
            for y in cell.1.saturating_sub(1)..=cell.1 + 1 {
                let Some(neighbours) = grid.get(&(x, y)) else {
                    continue;
                };

                for &j in neighbours {
                    rho += M * poly6(particles[i].pos - particles[j].pos);
                }
            }
        }

        particles[i].rho = rho;
    }
}

fn update_pressure(particles: &mut [Particle], rho0: f32) {
    for p in particles {
        p.p = B * ((p.rho / rho0).powf(GAMMA) - 1.);
    }
}

fn update_force(particles: &mut [Particle], grid: &SpatialHashGrid) {
    for i in 0..NUM_PARTICLES {
        let cell = particles[i].cell_coord();
        let mut f = M * get_acceleration();

        for x in cell.0.saturating_sub(1)..=cell.0 + 1 {
            for y in cell.1.saturating_sub(1)..=cell.1 + 1 {
                let Some(neighbours) = grid.get(&(x, y)) else {
                    continue;
                };

                for &j in neighbours {
                    if i != j {
                        f += -M
                            * (particles[i].p / particles[i].rho.powi(2)
                                + particles[j].p / particles[j].rho.powi(2))
                            * grad_spiky(particles[i].pos - particles[j].pos)
                    }
                }
            }
        }

        particles[i].f = f;
    }
}

fn apply_xsph(particles: &mut [Particle], grid: &SpatialHashGrid) {
    const EPS: f32 = 0.1;

    let mut dv = vec![Vec2::ZERO; NUM_PARTICLES];

    for i in 0..NUM_PARTICLES {
        let cell = particles[i].cell_coord();
        let mut corr = Vec2::ZERO;

        for x in cell.0.saturating_sub(1)..=cell.0 + 1 {
            for y in cell.1.saturating_sub(1)..=cell.1 + 1 {
                let Some(neighbours) = grid.get(&(x, y)) else {
                    continue;
                };
                for &j in neighbours {
                    if i != j {
                        let r = particles[i].pos - particles[j].pos;
                        corr +=
                            M * (particles[j].vel - particles[i].vel) / particles[i].rho * poly6(r);
                    }
                }
            }
        }

        dv[i] = EPS * corr;
    }
    for i in 0..NUM_PARTICLES {
        particles[i].vel += dv[i];
    }
}

fn handle_wall_collision(particles: &mut [Particle], w: f32, h: f32) {
    for p in particles {
        if p.pos.x < RADIUS {
            p.pos.x = RADIUS;
            p.vel.x = 0.;
        }
        if p.pos.x > w - RADIUS {
            p.pos.x = w - RADIUS;
            p.vel.x = 0.;
        }
        if p.pos.y > h - RADIUS {
            p.pos.y = h - RADIUS;
            p.vel.y = 0.;
        }
    }
}

#[macroquad::main("SPH")]
async fn main() {
    let mut particles = spawn_particles(NUM_PARTICLES);
    let mut rho0 = 0.;

    loop {
        for _ in 0..STEPS_PER_FRAME {
            let mut grid = SpatialHashGrid::new();
            for (i, p) in particles.iter().enumerate() {
                grid.entry(p.cell_coord()).or_default().push(i)
            }

            let w = screen_width();
            let h = screen_height();

            for p in &mut particles {
                p.vel += p.f * DT / 2.;
                p.pos += p.vel * DT;
            }

            update_density(&mut particles, &grid);
            if rho0 == 0. {
                rho0 = particles.iter().map(|p| p.rho).sum::<f32>() / NUM_PARTICLES as f32;
            }
            update_pressure(&mut particles, rho0);
            update_force(&mut particles, &grid);

            apply_xsph(&mut particles, &grid);

            for p in &mut particles {
                p.vel += p.f * DT / 2.;
            }

            handle_wall_collision(&mut particles, w, h);
        }

        render(&particles);
        next_frame().await;
    }
}
