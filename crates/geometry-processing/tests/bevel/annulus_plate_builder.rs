//! The construction the graze and line-to-line level-line fixtures share, written
//! once: an annulus plate's vertices welded by rounding (`At`), its planar top face wound by signed
//! area (`Top`), the hole and plate rim vertices by index (`Hole`, `Rim`), and the closing tail both
//! fixtures repeat word for word - the bottom face, the hole's wall and the plate's wall. Each
//! fixture (`graze_fixture.rs`, `line_to_line_fixture.rs`) lays its own top face between them.

#![allow(dead_code)]

use std::collections::HashMap;

use vector_math::Vector3;

use crate::level_line_room_geometry::SolidWithChains;

/// Thick enough that no chain's own wall bounds it at either fixture's size.
const THICKNESS: f64 = 3.0;

/// An annulus plate under construction.
pub struct AnnulusPlateBuilder {
    pub positions: Vec<Vector3>,
    by_position: HashMap<(i64, i64, i64), i32>,
    top_face: Vec<(i32, i32, i32)>,
    indices: Vec<(i32, i32, i32)>,
    /// The rounding a position is welded at, as a scale (1e6 welds to a micron of a unit).
    weld: f64,
    hole_radius: f64,
    plate_radius: f64,
    ring_segments: i32,
    step: f64,
    /// The bearing of plate rim vertex 0 (the hole rim's vertex 0 is always at bearing 0).
    rim_offset: f64,
}

impl AnnulusPlateBuilder {
    pub fn new(
        weld: f64,
        hole_radius: f64,
        plate_radius: f64,
        ring_segments: i32,
        rim_offset: f64,
    ) -> Self {
        AnnulusPlateBuilder {
            positions: Vec::new(),
            by_position: HashMap::new(),
            top_face: Vec::new(),
            indices: Vec::new(),
            weld,
            hole_radius,
            plate_radius,
            ring_segments,
            step: 2.0 * std::f64::consts::PI / ring_segments as f64,
            rim_offset,
        }
    }

    /// The angle between two rim vertices.
    pub fn step(&self) -> f64 {
        self.step
    }

    /// Rounding to the weld welds the pieces that meet; every region shares its boundary
    /// points by construction, so nothing that should stay apart is ever within it.
    pub fn at(&mut self, x: f64, y: f64, z: f64) -> i32 {
        // Banker's rounding, so a coordinate exactly half way between weld steps always lands on
        // the same key.
        let key = (
            (x * self.weld).round_ties_even() as i64,
            (y * self.weld).round_ties_even() as i64,
            (z * self.weld).round_ties_even() as i64,
        );
        if let Some(&index) = self.by_position.get(&key) {
            return index;
        }

        let index = self.positions.len() as i32;
        self.positions.push(Vector3::new(x, y, z));
        self.by_position.insert(key, index);
        index
    }

    /// The top face is planar at z = 0, so a triangle's winding is its signed area and
    /// the tiling can name its corners in whatever order reads best without ever getting a normal
    /// backwards.
    pub fn top(&mut self, a: i32, b: i32, c: i32) {
        let pa = self.positions[a as usize];
        let pb = self.positions[b as usize];
        let pc = self.positions[c as usize];
        let twice_area = ((pb.x - pa.x) * (pc.y - pa.y)) - ((pb.y - pa.y) * (pc.x - pa.x));
        self.top_face.push(if twice_area >= 0.0 {
            (a, b, c)
        } else {
            (a, c, b)
        });
    }

    fn wrapped(&self, i: i32) -> i32 {
        ((i % self.ring_segments) + self.ring_segments) % self.ring_segments
    }

    /// Hole rim vertex `i`, any integer taken round the ring.
    pub fn hole(&mut self, i: i32) -> i32 {
        let a = self.wrapped(i) as f64 * self.step;
        self.at(self.hole_radius * a.cos(), self.hole_radius * a.sin(), 0.0)
    }

    /// Plate rim vertex `j`, any integer taken round the ring.
    pub fn rim(&mut self, j: i32) -> i32 {
        let a = self.rim_offset + (self.wrapped(j) as f64 * self.step);
        self.at(
            self.plate_radius * a.cos(),
            self.plate_radius * a.sin(),
            0.0,
        )
    }

    fn quad(&mut self, a: i32, b: i32, c: i32, d: i32) {
        self.indices.push((a, b, c));
        self.indices.push((a, c, d));
    }

    /// The tail both fixtures share: the top face, the bottom face (the same tiling at -Thickness,
    /// wound the other way), the hole's wall facing INTO the hole - the winding the other bridge
    /// fixtures use, so the rim is a 90 degree fold on both faces - and the plate's own wall facing
    /// OUT. Hands back the positions, the triangles and the two rims (hole, then plate).
    pub fn close(mut self) -> SolidWithChains {
        let top_face = std::mem::take(&mut self.top_face);
        self.indices.extend(top_face.iter().copied());

        for &(a, b, c) in &top_face {
            let pa = self.positions[a as usize];
            let pb = self.positions[b as usize];
            let pc = self.positions[c as usize];
            let la = self.at(pa.x, pa.y, -THICKNESS);
            let lc = self.at(pc.x, pc.y, -THICKNESS);
            let lb = self.at(pb.x, pb.y, -THICKNESS);
            self.indices.push((la, lc, lb));
        }

        let n = self.ring_segments;
        let mut hole_chain = Vec::new();
        for i in 0..n {
            hole_chain.push(self.hole(i));
        }

        for i in 0..n {
            let next = self.hole(i + 1);
            let this = self.hole(i);
            let from = self.positions[next as usize];
            let to = self.positions[this as usize];
            let a = self.at(from.x, from.y, -THICKNESS);
            let b = self.at(to.x, to.y, -THICKNESS);
            self.quad(a, b, this, next);
        }

        let mut plate_chain = Vec::new();
        for j in 0..n {
            plate_chain.push(self.rim(j));
        }

        for j in 0..n {
            let this = self.rim(j);
            let next = self.rim(j + 1);
            let from = self.positions[this as usize];
            let to = self.positions[next as usize];
            let a = self.at(from.x, from.y, -THICKNESS);
            let b = self.at(to.x, to.y, -THICKNESS);
            self.quad(a, b, next, this);
        }

        (self.positions, self.indices, vec![hole_chain, plate_chain])
    }
}
