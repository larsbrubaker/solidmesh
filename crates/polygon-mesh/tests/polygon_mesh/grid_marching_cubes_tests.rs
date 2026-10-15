//! Marching cubes over random fields gives a closed, manifold surface whatever the corner values.
//! The fields come from a seeded Knuth subtractive generator, so every run tests the same 300
//! grids and a failure always reproduces.

use std::collections::HashMap;

use polygon_mesh::sdf::{extract, DenseGrid3f};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

/// A seeded Knuth subtractive random generator: the same seed gives the same sequence on every
/// platform.
struct SeededRandom {
    seed_array: [i32; 56],
    inext: usize,
    inextp: usize,
}

impl SeededRandom {
    fn new(seed: i32) -> Self {
        const MSEED: i32 = 161_803_398;
        let mut seed_array = [0i32; 56];
        let subtraction = if seed == i32::MIN {
            i32::MAX
        } else {
            seed.abs()
        };
        let mut mj = MSEED - subtraction;
        seed_array[55] = mj;
        let mut mk = 1i32;
        for i in 1..55 {
            let ii = (21 * i) % 55;
            seed_array[ii] = mk;
            mk = mj.wrapping_sub(mk);
            if mk < 0 {
                mk = mk.wrapping_add(i32::MAX);
            }
            mj = seed_array[ii];
        }
        for _ in 1..5 {
            for i in 1..56 {
                seed_array[i] = seed_array[i].wrapping_sub(seed_array[1 + (i + 30) % 55]);
                if seed_array[i] < 0 {
                    seed_array[i] = seed_array[i].wrapping_add(i32::MAX);
                }
            }
        }
        SeededRandom {
            seed_array,
            inext: 0,
            inextp: 21,
        }
    }

    fn internal_sample(&mut self) -> i32 {
        let mut loc_inext = self.inext + 1;
        if loc_inext >= 56 {
            loc_inext = 1;
        }
        let mut loc_inextp = self.inextp + 1;
        if loc_inextp >= 56 {
            loc_inextp = 1;
        }
        let mut ret_val = self.seed_array[loc_inext].wrapping_sub(self.seed_array[loc_inextp]);
        if ret_val == i32::MAX {
            ret_val -= 1;
        }
        if ret_val < 0 {
            ret_val = ret_val.wrapping_add(i32::MAX);
        }
        self.seed_array[loc_inext] = ret_val;
        self.inext = loc_inext;
        self.inextp = loc_inextp;
        ret_val
    }

    fn next_double(&mut self) -> f64 {
        self.internal_sample() as f64 * (1.0 / i32::MAX as f64)
    }

    fn next_range(&mut self, min_value: i32, max_value: i32) -> i32 {
        let range = max_value as i64 - min_value as i64;
        (self.next_double() * range as f64) as i32 + min_value
    }
}

/// Seeded 8x8x8 grids, border outside so every surface closes: smooth random values, small
/// integers (many nodes on the iso and exact saddle ties), wildly mixed magnitudes, and a field
/// biased inside. Every directed edge is used once with its reverse present, no triangle
/// repeats a vertex, and the triangles round each vertex form one fan.
#[test]
fn random_fields_give_closed_manifold_surfaces() {
    const N: usize = 8;
    let mut failures = Vec::new();
    for mode in 0..4 {
        for seed in 0..75 {
            let mut random = SeededRandom::new(seed * 7 + mode);
            let mut grid = DenseGrid3f::new(N, N, N, 1.0);
            for k in 1..N - 1 {
                for j in 1..N - 1 {
                    for i in 1..N - 1 {
                        grid.buffer[i + j * N + k * N * N] = match mode {
                            0 => (random.next_double() * 2.0 - 1.0) as f32,
                            1 => random.next_range(-2, 3) as f32,
                            2 => {
                                if random.next_double() < 0.5 {
                                    (-1e-7 * random.next_double()) as f32
                                } else {
                                    (1e3 * random.next_double()) as f32
                                }
                            }
                            _ => (random.next_double() * 2.0 - 1.2) as f32,
                        };
                    }
                }
            }

            let mesh = extract(
                &grid,
                Vector3::new(0.0, 0.0, 0.0),
                1.0,
                0.0,
                &CancelToken::new(),
            )
            .expect("not cancelled");
            if let Some(problem) = surface_problem(&mesh) {
                failures.push(format!("mode {mode} seed {seed}: {problem}"));
            }
        }
    }

    assert_eq!(failures.join("; "), "");
}

/// Why the mesh is not a closed, consistently wound manifold, or `None`.
fn surface_problem(mesh: &Mesh) -> Option<String> {
    let corners = |f: &polygon_mesh::Face| [f.v0, f.v1, f.v2];
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for face in mesh.faces.iter() {
        let [v0, v1, v2] = corners(face);
        for (a, b) in [(v0, v1), (v1, v2), (v2, v0)] {
            if a == b {
                return Some("degenerate triangle".to_string());
            }
            *directed.entry((a, b)).or_insert(0) += 1;
        }
    }

    for (&(a, b), &count) in &directed {
        if count != 1 {
            return Some(format!("directed edge used {count} times"));
        }
        if !directed.contains_key(&(b, a)) {
            return Some("open or flipped edge".to_string());
        }
    }

    // Round each vertex, the edge after a triangle's outgoing edge leads to the next triangle;
    // one fan means one cycle visits them all.
    let mut fans: HashMap<i32, HashMap<i32, i32>> = HashMap::new();
    for face in mesh.faces.iter() {
        let [v0, v1, v2] = corners(face);
        for (a, b, c) in [(v0, v1, v2), (v1, v2, v0), (v2, v0, v1)] {
            fans.entry(a).or_default().insert(b, c);
        }
    }

    for (vertex, fan) in &fans {
        let start = *fan.keys().next().expect("a fan has an entry");
        let mut at = start;
        let mut steps = 0;
        loop {
            match fan.get(&at) {
                Some(&next) => at = next,
                None => return Some(format!("vertex {vertex} fan is open")),
            }
            steps += 1;
            if at == start || steps > fan.len() {
                break;
            }
        }
        if steps != fan.len() {
            return Some(format!(
                "vertex {vertex} has {} triangles in more than one fan",
                fan.len()
            ));
        }
    }

    None
}
