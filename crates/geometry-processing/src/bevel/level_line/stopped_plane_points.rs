//! The points the band cut MAKES in a stopped end's plane (`stopped_end_clip.rs`) go onto the plane
//! exactly. The field reads a whole thin zone about a stopped plane as 0
//! (`ChainField::on_a_stopped_plane`, the zone a float-stored seam needs), so a search for the line
//! or for the corner where the plane meets a rail can stop anywhere in it. Left there, the vertex
//! stands microns off the plane, the stopped end's run is not flat to the snap, and the fill cannot
//! cut the end flat in its plane. So each such point is moved onto the plane's trace across the
//! triangle it is put in, to the trace's nearest point: where an edge of the triangle lies in the
//! plane (a facet seam) the trace IS that edge, and the insertion then splits the edge, shared by
//! index with the triangle across it, rather than minting a sliver beside it. An edge's root goes
//! to where the edge crosses the plane. Read by the corner finder (`end_clip_corners.rs`), the
//! refinement's line points (`band_refinement_asking.rs`) and the cut's edge roots (`band_cut.rs`).

use vector_math::Vector3;

use super::{end_clip_corners, ChainField, VertexFieldReading};

/// `point`, to be put in `triangle`, moved onto the stopped end plane it stands in: to the nearest
/// point of that plane's trace across the triangle (the triangle's edge in the plane, where it has
/// one). Unmoved where it stands in no stopped plane, or where the plane only touches the triangle
/// at a corner or holds it whole.
pub fn onto_a_stopped_plane(
    reading: &VertexFieldReading,
    triangle: i32,
    point: Vector3,
) -> Vector3 {
    let Some((vertex, normal, tilt)) = reading.stopped_end_plane_at(point) else {
        return point;
    };
    let corners = {
        let mesh = reading.mesh.borrow();
        let t = mesh.triangles()[triangle as usize];
        [mesh.position(t.a), mesh.position(t.b), mesh.position(t.c)]
    };
    let on = corners.map(|c| ChainField::on_a_stopped_plane(vertex, normal, tilt, c));
    let held = (0..3).find(|&k| on[k] && on[(k + 1) % 3]);
    let segment = match (on.iter().filter(|&&o| o).count(), held) {
        (2, Some(k)) => (corners[k], corners[(k + 1) % 3]),
        _ => match end_clip_corners::trace_across(corners, vertex, normal, tilt) {
            Some(segment) => segment,
            None => return point,
        },
    };
    nearest_on(segment, point)
}

/// An edge root `point` on the edge `a`-`b`, moved onto the stopped end plane it stands in: to
/// where the edge crosses the plane. Unmoved where it stands in no stopped plane or the edge does
/// not cross it (both ends on one side, or the edge in the plane).
pub fn edge_root_onto_a_stopped_plane(
    reading: &VertexFieldReading,
    (a, b): (Vector3, Vector3),
    point: Vector3,
) -> Vector3 {
    let Some((vertex, normal, tilt)) = reading.stopped_end_plane_at(point) else {
        return point;
    };
    if ChainField::on_a_stopped_plane(vertex, normal, tilt, a)
        || ChainField::on_a_stopped_plane(vertex, normal, tilt, b)
    {
        return point;
    }

    let (da, db) = ((a - vertex).dot(normal), (b - vertex).dot(normal));
    if (da < 0.0) == (db < 0.0) {
        return point;
    }

    a + (b - a) * (da / (da - db))
}

/// The nearest point to `point` of the segment `a`-`b` (`a` where the segment has no length).
fn nearest_on((a, b): (Vector3, Vector3), point: Vector3) -> Vector3 {
    let along = b - a;
    let length_squared = along.length_squared();
    if length_squared <= 0.0 {
        return a;
    }

    // `clamp` lets NaN through, so a broken reading stays visible.
    let s = ((point - a).dot(along) / length_squared).clamp(0.0, 1.0);
    a + (along * s)
}
