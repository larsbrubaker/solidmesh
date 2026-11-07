//! Walling two integer polygon loops into a mesh: [`stitch`], the start-pair search, the advance
//! rule for an outer/inner pair and the two wall builders [`stitch`] uses. Its top-only and
//! bottom-only cases cap a loop through `vertex_source_to_mesh::triangulate_faces`.

use clipper2_rust::{Path64, Paths64, Point64};
use vector_math::{Matrix4X4, Vector3};

use crate::processors::vertex_source_to_mesh::triangulate_faces;
use crate::Mesh;
use agg_path::data_converters_2d::create_vertex_storage;
use clipper_library::{get_intersection, length_squared, Intersection};

fn distance_squared(a: Point64, b: Point64) -> i64 {
    length_squared(Point64::new(a.x - b.x, a.y - b.y))
}

/// The point in units (`scaling` clipper units to the unit) at `height`, already in units. Every
/// coordinate is computed once in f64 and rounded to the mesh's f32 once, the same way the caps
/// (`create_vertex_storage` then `triangulate_faces`) and the extruder's band triangulation place
/// their points, so a ring this walls lands on exactly the vertices the faces beside it use.
/// Building in clipper units and scaling the f32 mesh afterwards rounds twice, which moved a ring
/// at a height such as 0.0999999970 mm to 0.1 here but to 0.099999994 in its neighbours, leaving
/// the extrusion open along that ring.
fn at_height(point: Point64, height: f64, scaling: f64) -> Vector3 {
    Vector3::new(point.x as f64 / scaling, point.y as f64 / scaling, height)
}

/// A cap facing up when there is only a bottom, one facing down when there is only a top, walls
/// when both have the same number of loops (paired by index, `scaling` clipper units to the unit),
/// and `None` otherwise. `None` and an empty list are the same absent loop.
pub fn stitch(
    bottom_loop: Option<&Paths64>,
    bottom_height: f64,
    top_loop: Option<&Paths64>,
    top_height: f64,
    scaling: f64,
) -> Option<Mesh> {
    let bottom_loop = bottom_loop.filter(|loops| !loops.is_empty());
    let top_loop = top_loop.filter(|loops| !loops.is_empty());
    match (bottom_loop, top_loop) {
        // only a bottom: if there is no top then we need to create a top
        (Some(bottom_loop), None) => Some(create_top(bottom_loop, bottom_height, scaling)),
        // only a top: if there is no bottom then we need to create a bottom
        (None, Some(top_loop)) => Some(create_bottom(top_loop, top_height, scaling)),
        // simple bottom and top
        (Some(bottom_loop), Some(top_loop)) if bottom_loop.len() == top_loop.len() => {
            let mut mesh = Mesh::new();
            for (bottom, top) in bottom_loop.iter().zip(top_loop) {
                let walls = if bottom.len() == top.len() {
                    create_simple_wall(bottom, bottom_height, top, top_height, scaling)
                } else {
                    stitch_2_single_walls(bottom, bottom_height, top, top_height, scaling)
                };
                mesh.copy_all_faces(&walls, Matrix4X4::IDENTITY);
            }

            Some(mesh)
        }
        _ => None,
    }
}

fn create_top(path: &Paths64, top_height: f64, scaling: f64) -> Mesh {
    triangulate_faces(
        &create_vertex_storage(path, scaling),
        None,
        None,
        top_height,
    )
}

fn create_bottom(path: &Paths64, bottom_height: f64, scaling: f64) -> Mesh {
    let mut mesh = triangulate_faces(
        &create_vertex_storage(path, scaling),
        None,
        None,
        bottom_height,
    );
    mesh.reverse_faces();
    mesh
}

/// The closest pair of vertices between the two loops, first found on a tie.
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
/// closing one triangle per step (the step rule is [`zip_steps_on_a`]): the shorter diagonal
/// keeps the wall running straight up between the loops, and every edge of both loops is used
/// exactly once. The loops are in
/// clipper units (`scaling` to the unit) and the heights in units; the mesh is in units.
pub fn stitch_2_single_walls(
    loop_a: &Path64,
    height_a: f64,
    loop_b: &Path64,
    height_b: f64,
    scaling: f64,
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

        if zip_steps_on_a(
            (cur_index_a, cur_index_b),
            (next_index_a, next_index_b),
            (start_index_a, start_index_b),
            (looped_a, looped_b),
            diagonal_if_advancing_a <= diagonal_if_advancing_b,
        ) {
            mesh.create_face(&[
                at_height(loop_a[cur_index_a], height_a, scaling),
                at_height(loop_a[next_index_a], height_a, scaling),
                at_height(loop_b[cur_index_b], height_b, scaling),
            ]);
            cur_index_a = next_index_a;
            looped_a = cur_index_a == start_index_a;
        } else {
            mesh.create_face(&[
                at_height(loop_a[cur_index_a], height_a, scaling),
                at_height(loop_b[next_index_b], height_b, scaling),
                at_height(loop_b[cur_index_b], height_b, scaling),
            ]);
            cur_index_b = next_index_b;
            looped_b = cur_index_b == start_index_b;
        }

        // Both loops walked all the way round. (Being back at both starts is not enough: a
        // small loop can be walked whole before the other has taken a step, which left the
        // other loop's wall out.)
        if looped_a && looped_b {
            break;
        }
    }

    mesh
}

/// Whether a zipper over loops a and b (at `cur`, about to step to `next`, having started at
/// `start`, with `looped` saying which loop has been walked all the way round) takes its next
/// triangle on loop a. It steps on the loop whose new diagonal is the shorter (`a_is_shorter`)
/// and on the other once a loop is done - but never back onto both starting points before the
/// other loop is done: the diagonal between the starts is the wall's seam, and passing it in the
/// middle walks it twice and ends the wall early.
pub fn zip_steps_on_a(
    cur: (usize, usize),
    next: (usize, usize),
    start: (usize, usize),
    looped: (bool, bool),
    a_is_shorter: bool,
) -> bool {
    let can_step_a = !looped.0 && !(next.0 == start.0 && cur.1 == start.1 && !looped.1);
    let can_step_b = !looped.1 && !(next.1 == start.1 && cur.0 == start.0 && !looped.0);
    can_step_a && (!can_step_b || a_is_shorter)
}

/// Walls two loops with the same vertex count, two triangles per vertex pair. The loops are in
/// clipper units (`scaling` to the unit) and the heights in units; the mesh is in units.
pub fn create_simple_wall(
    bottom_loop: &Path64,
    bottom_height: f64,
    top_loop: &Path64,
    top_height: f64,
    scaling: f64,
) -> Mesh {
    let mut mesh = Mesh::new();
    for i in 0..bottom_loop.len() {
        let next = (i + 1) % bottom_loop.len();
        mesh.create_face(&[
            at_height(bottom_loop[i], bottom_height, scaling),
            at_height(bottom_loop[next], bottom_height, scaling),
            at_height(top_loop[i], top_height, scaling),
        ]);
        mesh.create_face(&[
            at_height(bottom_loop[next], bottom_height, scaling),
            at_height(top_loop[next], top_height, scaling),
            at_height(top_loop[i], top_height, scaling),
        ]);
    }

    mesh
}

/// Which loop an outer/inner stitch should advance on: 1 for the inner loop, 0 for the outer. The
/// inner loop advances unless its new diagonal would cross the inner loop, or the outer's diagonal
/// is both shorter and does not cross.
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
