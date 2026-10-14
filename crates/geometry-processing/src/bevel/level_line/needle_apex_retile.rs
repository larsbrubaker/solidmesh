//! A needle's apex standing inside one of its own edges, re-tiled through: [`try_retile`].

use std::collections::{HashMap, HashSet, VecDeque};

use vector_math::Vector3;

use super::super::float_grid;
use super::refinement_splits;
use super::{LevelLineMesh, LevelLineTriangle, LevelLineWork, WorkError};

/// Where a corner of `t` stands strictly inside the opposite edge (to the float step on
/// float-stored input, otherwise the same-place backstop or coincidence), re-tiles every triangle
/// on that edge through it - once per edge (`retiled`) - and enqueues the children one level
/// deeper than `here`; true when it re-tiled.
pub fn try_retile(
    mesh: &mut LevelLineMesh,
    t: LevelLineTriangle,
    retiled: &mut HashSet<(i32, i32)>,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
    here: i32,
) -> Result<bool, WorkError> {
    for corner in 0..3 {
        let apex = t.corner(corner);
        let a = t.corner((corner + 1) % 3);
        let b = t.corner((corner + 2) % 3);
        let from = mesh.position(a);
        let along = mesh.position(b) - from;
        let length_squared = along.length_squared();
        if length_squared <= 0.0 {
            continue;
        }

        let point: Vector3 = mesh.position(apex);
        let on_the_edge = if mesh.input_is_float_stored {
            float_grid::step_at_point(point)
        } else if mesh.same_place() > 0.0 {
            mesh.same_place()
        } else {
            1e-12 * length_squared.sqrt()
        };
        let s = (point - from).dot(along) / length_squared;
        if s <= 0.0
            || s >= 1.0
            || (point - (from + (along * s))).length() > on_the_edge
            || (point - from).length() <= on_the_edge
            || (point - mesh.position(b)).length() <= on_the_edge
        {
            continue;
        }

        let key = if a < b { (a, b) } else { (b, a) };
        if !retiled.insert(key) {
            continue;
        }

        LevelLineWork::charge(1)?;
        if !mesh.retile_through(a, b, apex) {
            continue;
        }

        let mut children = mesh.triangles_on(a, apex).to_vec();
        children.extend_from_slice(mesh.triangles_on(apex, b));
        refinement_splits::enqueue(children, pending, depth, here + 1);
        return Ok(true);
    }

    Ok(false)
}
