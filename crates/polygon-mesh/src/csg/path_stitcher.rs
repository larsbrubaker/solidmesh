//! Walling two integer polygon loops into a mesh. Port of agg-sharp `Csg/PathStitcher.cs`
//! (namespace `Processors`): the start-pair search, the advance rule for an outer/inner pair and
//! the two wall builders `Stitch` uses.
//!
//! Not here yet: `Stitch` itself, whose top-only and bottom-only cases cap a loop through
//! agg-path's `VertexStorage.TriangulateFaces` (`VertexSourceToMesh`, Phase 2e); it lands with
//! that slice on top of [`create_simple_wall`] and [`stitch_2_single_walls`].

use clipper2_rust::{Path64, Point64};
use vector_math::Vector3;

use crate::Mesh;
use agg_path::clipper_library::{get_intersection, length_squared, Intersection};

fn distance_squared(a: Point64, b: Point64) -> i64 {
    length_squared(Point64::new(a.x - b.x, a.y - b.y))
}

fn at_height(point: Point64, height: f64) -> Vector3 {
    Vector3::new(point.x as f64, point.y as f64, height)
}

/// The closest pair of vertices between the two loops, first found on a tie
/// (C# `BestStartIndices`).
pub fn best_start_indices(loop_a: &Path64, loop_b: &Path64) -> (usize, usize) {
    let mut best_distance = f64::MAX;
    let mut best_index_a = 0;
    let mut best_index_b = 0;
    for (index_a, &a) in loop_a.iter().enumerate() {
        for (index_b, &b) in loop_b.iter().enumerate() {
            let distance = distance_squared(a, b) as f64;
            if distance < best_distance {
                best_distance = distance;
                best_index_a = index_a;
                best_index_b = index_b;
            }
        }
    }

    (best_index_a, best_index_b)
}

/// Walls two loops that do not have the same number of vertices, by walking both at once and
/// closing one triangle per step. Each step advances the loop whose candidate diagonal is the
/// SHORTER, which is what keeps the wall running straight up between the loops; once a loop has
/// been walked all the way round, every remaining step is taken on the other
/// (C# `Stitch2SingleWalls`).
pub fn stitch_2_single_walls(
    loop_a: &Path64,
    height_a: f64,
    loop_b: &Path64,
    height_b: f64,
) -> Mesh {
    let mut mesh = Mesh::new();
    let (start_index_a, start_index_b) = best_start_indices(loop_a, loop_b);
    let mut cur_index_a = start_index_a;
    let mut cur_index_b = start_index_b;
    let mut looped_a = false;
    let mut looped_b = false;

    loop {
        let next_index_a = (cur_index_a + 1) % loop_a.len();
        let next_index_b = (cur_index_b + 1) % loop_b.len();

        // the diagonal each candidate triangle would add, measured the same way for both
        let diagonal_if_advancing_a = distance_squared(loop_b[cur_index_b], loop_a[next_index_a]);
        let diagonal_if_advancing_b = distance_squared(loop_a[cur_index_a], loop_b[next_index_b]);

        if (!looped_a && diagonal_if_advancing_a <= diagonal_if_advancing_b) || looped_b {
            mesh.create_face(&[
                at_height(loop_a[cur_index_a], height_a),
                at_height(loop_a[next_index_a], height_a),
                at_height(loop_b[cur_index_b], height_b),
            ]);
            cur_index_a = next_index_a;
            looped_a = cur_index_a == start_index_a;
        } else {
            mesh.create_face(&[
                at_height(loop_a[cur_index_a], height_a),
                at_height(loop_b[next_index_b], height_b),
                at_height(loop_b[cur_index_b], height_b),
            ]);
            cur_index_b = next_index_b;
            looped_b = cur_index_b == start_index_b;
        }

        if cur_index_a == start_index_a && cur_index_b == start_index_b {
            break;
        }
    }

    mesh
}

/// Walls two loops with the same vertex count, two triangles per vertex pair
/// (C# `CreateSimpleWall`).
pub fn create_simple_wall(
    bottom_loop: &Path64,
    bottom_height: f64,
    top_loop: &Path64,
    top_height: f64,
) -> Mesh {
    let mut mesh = Mesh::new();
    for i in 0..bottom_loop.len() {
        let next = (i + 1) % bottom_loop.len();
        mesh.create_face(&[
            at_height(bottom_loop[i], bottom_height),
            at_height(bottom_loop[next], bottom_height),
            at_height(top_loop[i], top_height),
        ]);
        mesh.create_face(&[
            at_height(bottom_loop[next], bottom_height),
            at_height(top_loop[next], top_height),
            at_height(top_loop[i], top_height),
        ]);
    }

    mesh
}

/// Which loop an outer/inner stitch should advance on: 1 for the inner loop, 0 for the outer
/// (C# `GetPolygonToAdvance`). The inner loop advances unless its new diagonal would cross the
/// inner loop, or the outer's diagonal is both shorter and does not cross.
pub fn get_polygon_to_advance(
    outer_loop: &Path64,
    outer_index: usize,
    inner_loop: &Path64,
    inner_index: usize,
) -> i32 {
    let outer_start = outer_loop[outer_index];
    let outer_next = outer_loop[(outer_index + 1) % outer_loop.len()];

    let inner_start = inner_loop[inner_index];
    let inner_next = inner_loop[(inner_index + 1) % inner_loop.len()];

    let distance_to_inner_next = distance_squared(inner_next, outer_start);
    let distance_to_outer_next = distance_squared(inner_start, outer_next);

    let inner_advance_crosses = segment_crosses_polygon(inner_loop, outer_start, inner_next);
    let outer_advance_crosses = segment_crosses_polygon(inner_loop, inner_start, outer_next);

    if !inner_advance_crosses
        && (outer_advance_crosses || distance_to_inner_next <= distance_to_outer_next)
    {
        return 1;
    }

    0
}

/// Whether the segment meets any edge of the polygon that does not share one of its ends.
fn segment_crosses_polygon(polygon: &Path64, start: Point64, end: Point64) -> bool {
    for i in 0..polygon.len() {
        let edge_start = polygon[i];
        let edge_end = polygon[(i + 1) % polygon.len()];
        if start == edge_start || start == edge_end || end == edge_start || end == edge_end {
            continue;
        }

        if get_intersection(start, end, edge_start, edge_end) != Intersection::None {
            return true;
        }
    }

    false
}
