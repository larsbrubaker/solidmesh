//! The input-strip half of the working mesh: a strip of needles the tessellator handed in, taken
//! out ONCE by an ear cut before the build reads the mesh. Ports
//! `Bevel/LevelLine/LevelLineMesh.InputStrips.cs` (a partial of `LevelLineMesh`): so far the ear
//! cut's charge ([`LevelLineMesh::charge_strip_work`]) and small readings - the first key of a map ([`first_key`]), whether an edge stands outside
//! the region ([`LevelLineMesh::edge_outside_region`]) and a triangle's least height
//! ([`LevelLineMesh::least_height`]); the strip removal lands with later packages.

use std::collections::HashSet;

use indexmap::IndexMap;
use vector_math::cs_math;

use super::{LevelLineMesh, LevelLineWork, WorkError};

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
