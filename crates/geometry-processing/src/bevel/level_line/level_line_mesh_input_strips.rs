//! The input-strip half of the working mesh: a strip of needles the tessellator handed in, taken
//! out ONCE by an ear cut before the build reads the mesh. Ports
//! `Bevel/LevelLine/LevelLineMesh.InputStrips.cs` (a partial of `LevelLineMesh`): so far the ear
//! cut ([`LevelLineMesh::ear_cut`]), its charge ([`LevelLineMesh::charge_strip_work`]) and small
//! readings - the first key of a map ([`first_key`]), whether an edge stands outside
//! the region ([`LevelLineMesh::edge_outside_region`]) and a triangle's least height
//! ([`LevelLineMesh::least_height`]); the strip removal lands with later packages.

use std::collections::HashSet;

use indexmap::IndexMap;
use vector_math::{cs_math, Vector3};

use super::{LevelLineMesh, LevelLineWork, WorkError};

/// An ear cut's tiles: index triples of rim vertices.
pub type Tiling = Vec<(i32, i32, i32)>;

/// The first key of a map in its insertion order, or -1 for an empty one.
pub fn first_key(map: &IndexMap<i32, i32>) -> i32 {
    map.keys().next().copied().unwrap_or(-1)
}

impl LevelLineMesh {
    /// Charges the strip removal's work to the budget in force, and counts it beside the charge
    /// so the tally still says where the work went when the budget runs out inside the removal.
    pub fn charge_strip_work(&mut self, units: i64) -> Result<(), WorkError> {
        self.input_strip_work_spent += units;
        LevelLineWork::charge(units)
    }

    /// True when a triangle outside `region` stands on edge `a b`.
    pub fn edge_outside_region(&self, a: i32, b: i32, region: &HashSet<i32>) -> bool {
        self.triangles_on(a, b)
            .iter()
            .any(|triangle| !region.contains(triangle))
    }

    /// The least height of triangle `a b c`: twice its area over its longest side; 0 when it has
    /// no extent.
    pub fn least_height(&self, a: i32, b: i32, c: i32) -> f64 {
        let pa = self.vertices[a as usize];
        let pb = self.vertices[b as usize];
        let pc = self.vertices[c as usize];
        let longest = cs_math::max_f64(
            (pb - pa).length(),
            cs_math::max_f64((pc - pb).length(), (pa - pc).length()),
        );
        if longest <= 0.0 {
            0.0
        } else {
            (pb - pa).cross(pc - pa).length() / longest
        }
    }
}

impl LevelLineMesh {
    /// Tiles a rim loop - wound counter-clockwise seen down the normal - with triangles of its own
    /// vertices, taking at each turn the ear that stands HIGHEST, and refusing any ear no higher
    /// than the floor or whose new edge the mesh already carries outside the region. `Ok(None)`
    /// where no such tiling is found.
    pub fn ear_cut(
        &mut self,
        loop_vertices: &[i32],
        origin: Vector3,
        normal: Vector3,
        region: &HashSet<i32>,
    ) -> Result<Option<Tiling>, WorkError> {
        self.longest_rim_cut = self.longest_rim_cut.max(loop_vertices.len() as i32);

        let u = normal
            .cross(if normal.x.abs() < 0.9 {
                Vector3::UNIT_X
            } else {
                Vector3::UNIT_Y
            })
            .get_normal();
        let v = normal.cross(u);
        let flat = |mesh: &LevelLineMesh, vertex: i32| {
            let d = mesh.vertices[vertex as usize] - origin;
            (d.dot(u), d.dot(v))
        };
        let turn = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
            ((b.0 - a.0) * (c.1 - a.1)) - ((b.1 - a.1) * (c.0 - a.0))
        };

        // (u, v, normal) is right-handed, so a loop wound counter-clockwise about the normal turns
        // left here. Being within the floor of the plane is not being FLAT: a ring triangle folded
        // back on itself inside the floor flattens to a clockwise triangle, the flattened rim can
        // then cross itself, and ear clipping a self-crossing rim lays overlapping tiles. So every
        // region triangle that has height must turn left once flattened, or the region is not cut.
        for &triangle in region {
            let t = self.triangles[triangle as usize];
            if self.least_height(t.a, t.b, t.c) > self.needle_floor
                && turn(flat(self, t.a), flat(self, t.b), flat(self, t.c)) <= 0.0
            {
                return Ok(None);
            }
        }

        let mut remaining = loop_vertices.to_vec();
        let mut tiles = Vec::new();
        while remaining.len() > 3 {
            // Charged for what a turn DOES: it weighs every rim vertex as an ear, and each ear that
            // survives the cheap tests scans the whole rim again (charged below). One unit a turn
            // would have let a rim of n vertices spend ~n^3 work on n units.
            self.charge_strip_work(remaining.len() as i64)?;
            let count = remaining.len();
            let mut best: Option<usize> = None;
            let mut best_height = self.needle_floor;
            for i in 0..count {
                let a = remaining[(i + count - 1) % count];
                let b = remaining[i];
                let c = remaining[(i + 1) % count];
                let pa = flat(self, a);
                let pb = flat(self, b);
                let pc = flat(self, c);
                if turn(pa, pb, pc) <= 0.0 || self.edge_outside_region(a, c, region) {
                    continue;
                }

                let height = self.least_height(a, b, c);
                if height <= best_height {
                    continue;
                }

                // An ear holds no other rim vertex - one ON its edges included, which would be a
                // T-vertex.
                self.charge_strip_work(count as i64)?;
                let empty = !remaining.iter().any(|&other| {
                    if other == a || other == b || other == c {
                        return false;
                    }

                    let p = flat(self, other);
                    turn(pa, pb, p) >= 0.0 && turn(pb, pc, p) >= 0.0 && turn(pc, pa, p) >= 0.0
                });

                if empty {
                    best = Some(i);
                    best_height = height;
                }
            }

            let Some(best) = best else {
                return Ok(None);
            };

            tiles.push((
                remaining[(best + count - 1) % count],
                remaining[best],
                remaining[(best + 1) % count],
            ));
            remaining.remove(best);
        }

        if self.least_height(remaining[0], remaining[1], remaining[2]) <= self.needle_floor {
            return Ok(None);
        }

        tiles.push((remaining[0], remaining[1], remaining[2]));
        Ok(Some(tiles))
    }
}
