//! Orientation repair (derived from geometry3Sharp): makes each edge-connected component's
//! triangles agree in orientation (a flood fill that flips neighbours to match), then, for the
//! global solve, casts a ray at each triangle from both sides to see whether the component is
//! mostly seen from outside or inside, and flips the components seen from inside.
//!
//! Each component's facing areas are added in triangle order on one thread, so the sums (and
//! the flip decisions that rest on them) are the same on every run. There is no progress
//! reporting: nothing that runs the repair displays it.

use std::collections::HashSet;

use super::dmesh3::{DMesh3, INVALID_ID};
use super::dmesh3_queries::find_tri_ordered_edge;
use super::ray_intersection::Ray3d;
use super::DMeshAABBTree3;

/// The single-precision zero tolerance, compared against a double area: smaller triangles
/// have no reliable normal and are left out of the vote.
const ZERO_TOLERANCE_F: f64 = 1e-6f32 as f64;

/// One edge-connected component and its inward- and outward-facing area totals.
#[derive(Debug, Clone, Default)]
struct Component {
    triangles: Vec<i32>,
    out_facing: f64,
    in_facing: f64,
}

/// The orientation repair's components, found once and then solved.
#[derive(Debug, Clone, Default)]
pub(crate) struct MeshRepairOrientation {
    components: Vec<Component>,
}

impl MeshRepairOrientation {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Flood-fills each component from its lowest triangle id,
    /// reversing every neighbour that does not run the shared edge the opposite way.
    pub(crate) fn orient_components(&mut self, mesh: &mut DMesh3) {
        self.components.clear();

        // Each flood starts from the lowest remaining triangle id; a flag per id and a cursor
        // find it without a set.
        let max_tid = mesh.max_triangle_id() as usize;
        let mut remaining = vec![false; max_tid];
        let mut remaining_count = 0;
        for tid in mesh.triangle_indices() {
            remaining[tid as usize] = true;
            remaining_count += 1;
        }
        let mut cursor = 0;
        let mut stack: Vec<i32> = Vec::new();
        while remaining_count > 0 {
            while !remaining[cursor] {
                cursor += 1;
            }
            let start = cursor as i32;
            let mut c = Component::default();
            stack.clear();
            remaining[start as usize] = false;
            remaining_count -= 1;
            c.triangles.push(start);
            stack.push(start);
            while let Some(cur) = stack.pop() {
                let tcur = mesh.get_triangle(cur);
                let nbrs = mesh.get_tri_neighbour_tris(cur);
                for j in 0..3 {
                    let nbr = nbrs[j];
                    if nbr == INVALID_ID || !remaining[nbr as usize] {
                        continue;
                    }
                    let a = tcur[j];
                    let b = tcur[(j + 1) % 3];
                    let tnbr = mesh.get_triangle(nbr);
                    if find_tri_ordered_edge(b, a, &tnbr) == INVALID_ID {
                        mesh.reverse_tri_orientation(nbr);
                    }
                    stack.push(nbr);
                    remaining[nbr as usize] = false;
                    remaining_count -= 1;
                    c.triangles.push(nbr);
                }
            }
            self.components.push(c);
        }
    }

    /// Computes every component's facing areas. Builds the tree over the
    /// mesh as it is now and drops it before returning, so the mesh can be edited again.
    fn compute_statistics(&mut self, mesh: &DMesh3) {
        let spatial = DMeshAABBTree3::new(mesh);
        let dist = 2.0 * diagonal_length(mesh);
        for c in &mut self.components {
            c.in_facing = 0.0;
            c.out_facing = 0.0;
            let tris: HashSet<i32> = c.triangles.iter().copied().collect();
            let filter = |t: i32| tris.contains(&t);
            for &ti in &c.triangles {
                let (normal, area, centroid) = get_tri_info(mesh, ti);
                if area < ZERO_TOLERANCE_F {
                    continue;
                }
                let pos_pt: [f64; 3] = std::array::from_fn(|a| centroid[a] + dist * normal[a]);
                let neg_pt: [f64; 3] = std::array::from_fn(|a| centroid[a] - dist * normal[a]);
                let neg_normal = normal.map(|v| -v);
                // The ray query refuses a direction that is not normalized; the area check
                // above keeps zero normals out, so a refusal cannot happen and counts as a miss.
                let hit_pos = spatial
                    .find_nearest_hit_triangle(
                        &Ray3d::new(pos_pt, neg_normal),
                        f64::MAX,
                        Some(&filter),
                    )
                    .unwrap_or(INVALID_ID);
                let hit_neg = spatial
                    .find_nearest_hit_triangle(&Ray3d::new(neg_pt, normal), f64::MAX, Some(&filter))
                    .unwrap_or(INVALID_ID);
                if hit_pos != ti && hit_neg != ti {
                    continue; // no evidence
                }
                if hit_pos == ti && hit_neg == ti {
                    continue; // no evidence (?)
                }
                if hit_neg == ti {
                    c.in_facing += area;
                } else if hit_pos == ti {
                    c.out_facing += area;
                }
            }
        }
    }

    /// Flips the components seen more from inside than outside.
    pub(crate) fn solve_global_orientation(&mut self, mesh: &mut DMesh3) {
        self.compute_statistics(mesh);
        for c in &self.components {
            if c.in_facing > c.out_facing {
                // Reverse each triangle (there are no vertex normals to flip here)
                for &tid in &c.triangles {
                    mesh.reverse_tri_orientation(tid);
                }
            }
        }
    }
}

/// The diagonal length of the mesh's cached bounds.
fn diagonal_length(mesh: &DMesh3) -> f64 {
    let b = mesh.cached_bounds();
    let (dx, dy, dz) = (
        b.max[0] - b.min[0],
        b.max[1] - b.min[1],
        b.max[2] - b.min[2],
    );
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// A triangle's unit normal, area and centroid.
fn get_tri_info(mesh: &DMesh3, tid: i32) -> ([f64; 3], f64, [f64; 3]) {
    let [v0, v1, v2] = mesh.get_tri_vertices(tid as usize);
    let f = 1.0 / 3.0;
    let centroid = std::array::from_fn(|a| f * (v0[a] + v1[a] + v2[a]));
    let edge1: [f64; 3] = std::array::from_fn(|a| v1[a] - v0[a]);
    let edge2: [f64; 3] = std::array::from_fn(|a| v2[a] - v0[a]);
    let c = [
        edge1[1] * edge2[2] - edge1[2] * edge2[1],
        edge1[2] * edge2[0] - edge1[0] * edge2[2],
        edge1[0] * edge2[1] - edge1[1] * edge2[0],
    ];
    // Normalize, keeping the length divided by (zero below the double epsilon)
    let length = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
    let (normal, length) = if length > 2.220_446_049_250_313e-16 {
        let inv = 1.0 / length;
        ([c[0] * inv, c[1] * inv, c[2] * inv], length)
    } else {
        ([0.0; 3], 0.0)
    };
    (normal, 0.5 * length, centroid)
}
