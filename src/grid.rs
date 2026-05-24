use super::*;

pub struct SpatialHashGrid {
    pub num_cols: u32,
    pub num_rows: u32,
    pub starts: Vec<u32>,
    pub ends: Vec<u32>,
    pub ids: Vec<u32>,
    pub cxs: Vec<i32>,
    pub cys: Vec<i32>,
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
    }

    pub fn for_each_neighbour<F>(&self, i: usize, mut f: F)
    where
        F: FnMut(usize),
    {
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
                let c = (ny * self.num_cols as i32 + nx) as usize;
                for k in self.starts[c]..self.ends[c] {
                    f(self.ids[k as usize] as usize);
                }
            }
        }
    }
}
