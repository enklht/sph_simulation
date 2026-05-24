use super::*;

pub struct SpatialHashGrid {
    num_cols: u32,
    num_rows: u32,
    starts: Vec<u32>,
    ends: Vec<u32>,
    ids: Vec<u32>,
    cxs: Vec<i32>,
    cys: Vec<i32>,
    pub neighbours: Vec<Vec<usize>>,
}

impl SpatialHashGrid {
    pub fn new(w: f32, h: f32, num_particles: usize) -> Self {
        let num_cols = (w / H).ceil() as u32 + 1;
        let num_rows = (h / H).ceil() as u32 + 1;

        let num_cells = num_cols * num_rows;

        Self {
            num_cols,
            num_rows,
            starts: vec![0; num_cells as usize],
            ends: vec![0; num_cells as usize],
            ids: vec![0; num_particles],
            cxs: vec![0; num_particles],
            cys: vec![0; num_particles],
            neighbours: vec![vec![]; num_particles],
        }
    }

    pub fn update(&mut self, pos: &[Vec2]) {
        let num_cols = self.num_cols;

        self.ends.fill(0);

        for (i, p) in pos.iter().enumerate() {
            let cx = (p.x / H).round() as i32;
            let cy = (p.y / H).round() as i32;
            self.cxs[i] = cx;
            self.cys[i] = cy;
            self.ends[(cy * num_cols as i32 + cx) as usize] += 1;
        }

        let mut total = 0;
        for i in 0..self.starts.len() {
            self.starts[i] = total;
            total += self.ends[i];
            self.ends[i] = self.starts[i];
        }

        for i in 0..pos.len() {
            let c = (self.cys[i] * num_cols as i32 + self.cxs[i]) as usize;
            self.ids[self.ends[c] as usize] = i as u32;
            self.ends[c] += 1;
        }

        self.neighbours
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, neighbours)| {
                neighbours.clear();

                let cx = self.cxs[i];
                let cy = self.cys[i];

                for dy in -1..=1 {
                    let ny = cy + dy;
                    if ny < 0 || ny >= self.num_rows as i32 {
                        continue;
                    }
                    for dx in -1..=1 {
                        let nx = cx + dx;
                        if nx < 0 || nx >= self.num_cols as i32 {
                            continue;
                        }
                        let c = ny * self.num_cols as i32 + nx;
                        for k in self.starts[c as usize]..self.ends[c as usize] {
                            neighbours.push(self.ids[k as usize] as usize)
                        }
                    }
                }
            });
    }
}
