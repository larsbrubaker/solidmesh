//! The simple hole filler (derived from geometry3Sharp): fills a boundary loop with one
//! triangle when it has three vertices, else with a fan around a new vertex at the loop's
//! centroid, backing the vertex out again if the fan is refused.

use super::dmesh3::{DMesh3, G3Exception, INVALID_ID};
use super::edge_loop::{is_boundary_loop, EdgeLoop, ValidationStatus};
use super::mesh_editor::add_triangle_fan_ordered_vertex_loop;

/// A fill of one boundary loop and what it added.
#[derive(Debug, Clone)]
pub(crate) struct SimpleHoleFiller<'l> {
    pub(crate) lp: &'l EdgeLoop,
    /// The centroid vertex, or [`INVALID_ID`].
    pub(crate) new_vertex: i32,
    /// The fill's triangles, `None` until a fill succeeds.
    pub(crate) new_triangles: Option<Vec<i32>>,
}

impl<'l> SimpleHoleFiller<'l> {
    /// A filler for loop `lp`.
    pub(crate) fn new(lp: &'l EdgeLoop) -> Self {
        SimpleHoleFiller {
            lp,
            new_vertex: INVALID_ID,
            new_triangles: None,
        }
    }

    #[allow(dead_code)] // mesh API only the tests call so far
    /// Whether the loop is still a boundary loop of `mesh`.
    pub(crate) fn validate(&self, mesh: &DMesh3) -> ValidationStatus {
        is_boundary_loop(mesh, self.lp)
    }

    /// Fills the loop, the new triangles in group `group_id`. `Err` is a failure of the
    /// back-out `remove_vertex` (it cannot fail once the fan has removed what it added).
    pub(crate) fn fill(&mut self, mesh: &mut DMesh3, group_id: i32) -> Result<bool, G3Exception> {
        let verts = &self.lp.vertices;
        if verts.len() < 3 {
            return Ok(false);
        }

        // this just needs one triangle
        if verts.len() == 3 {
            let new_tid = mesh.append_triangle_g(verts[0], verts[2], verts[1], group_id);
            if new_tid < 0 {
                return Ok(false);
            }
            self.new_triangles = Some(vec![new_tid]);
            self.new_vertex = INVALID_ID;
            return Ok(true);
        }

        // [TODO] 4-case? could check nbr normals to figure out best internal edge...

        // compute centroid
        let mut c = [0.0f64; 3];
        for &v in verts {
            let p = mesh.vertex(v);
            c = [c[0] + p[0], c[1] + p[1], c[2] + p[2]];
        }
        let f = 1.0 / verts.len() as f64;
        c = [c[0] * f, c[1] * f, c[2] * f];

        // add centroid vtx
        self.new_vertex = mesh.append_vertex(c);

        // stitch triangles (a fan that fails to back out counts as a refusal)
        self.new_triangles =
            add_triangle_fan_ordered_vertex_loop(mesh, self.new_vertex, verts, group_id)
                .ok()
                .flatten();

        // if fill failed, back out vertex-add
        if self.new_triangles.is_none() {
            mesh.remove_vertex(self.new_vertex, true, false)?;
            self.new_vertex = INVALID_ID;
            Ok(false)
        } else {
            Ok(true)
        }
    }
}
