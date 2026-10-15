//! The band of a bevel between two neighbouring rings that are not the same loops, built on the
//! rings' own vertices (`ring_mesh.rs`). The band walks every edge of the lower ring forward and
//! every edge of the upper ring backward, exactly once, which is what closes it against the
//! surfaces on the rings' other sides.
//!
//! The band is first triangulated in 2D: the area between the rings, with both rings' points as
//! constraints, each corner lifted to the ring it belongs to. That follows any change of shape
//! between the rings - a neck pinching apart, a spike or a hole vanishing - because the area
//! between them is simply whatever is left. Where the rings touch, at a point or along an edge,
//! the band has no width there and the wall between the rings is vertical; the band adds those
//! vertical faces itself. tess2-rust's constrained Delaunay refinement adds no points for rings
//! that do not cross. (Shewchuk's Triangle, the usual tool for this, is licensed for
//! non-commercial use only.)
//!
//! Rings a fraction of a micron apart can cross, after rounding to clipper's integer units; the 2D
//! area between them is then not a band at all. Such a pair is walled loop to loop instead, each
//! upper loop zipped to the lower loop nearest it, and any loop left without a partner capped
//! flat - a combinatorial construction that closes whatever the positions are.

use std::collections::HashSet;

use clipper2_rust::{area, Path64};

use crate::csg::path_stitcher::{best_start_indices, zip_steps_on_a};

use super::ring_mesh::{PlacedRing, RingMesh};
use crate::processors::cap_fill::{fill_triangles, loop_edges, walks_exactly};

/// Adds the band between `lower` and `upper`. `lower_is_outer` says the profile shrinks as it
/// rises here (a top bevel); a bottom bevel's profile grows.
pub(super) fn add_band(
    rings: &mut RingMesh,
    lower: &PlacedRing,
    upper: &PlacedRing,
    lower_is_outer: bool,
) {
    let mut boundary: Vec<(i32, i32)> = lower.indices.iter().flat_map(|l| loop_edges(l)).collect();
    boundary.extend(
        upper
            .indices
            .iter()
            .flat_map(|l| loop_edges(l))
            .map(|(a, b)| (b, a)),
    );

    if let Some(faces) = triangulated_band(lower, upper, lower_is_outer) {
        if walks_exactly(&faces, &boundary) {
            rings.add_all(&faces);
            return;
        }
    }
    zipped_band(rings, lower, upper);
}

/// The band as the 2D triangulation of the area between the rings, or `None` where the rings
/// cross (the tessellator had to add a point on neither ring).
fn triangulated_band(
    lower: &PlacedRing,
    upper: &PlacedRing,
    lower_is_outer: bool,
) -> Option<Vec<[i32; 3]>> {
    let loops: Vec<&Path64> = lower.loops.iter().chain(upper.loops.iter()).collect();
    let triangles = fill_triangles(&loops).ok()?;

    // Every edge of the upper ring, in both directions, for the slivers below.
    let upper_edges: HashSet<(i32, i32)> = upper
        .indices
        .iter()
        .flat_map(|l| loop_edges(l))
        .flat_map(|(a, b)| [(a, b), (b, a)])
        .collect();

    let mut faces = Vec::new();
    for triangle in triangles {
        let mut points = triangle;
        // The fill is counter clockwise, which faces outward where the profile shrinks upward.
        if !lower_is_outer {
            points.swap(0, 2);
        }
        let on_lower = points.map(|(x, y)| lower.vertex_at(x, y));
        let on_upper = points.map(|(x, y)| upper.vertex_at(x, y));
        // A point both rings pass through is one point to the 2D fill; it is lifted to the lower
        // ring, and the slivers below join it to the upper ring.
        let mut lifted = [0; 3];
        for j in 0..3 {
            lifted[j] = on_lower[j].or(on_upper[j])?;
        }
        let shared = |j: usize| on_lower[j].is_some() && on_upper[j].is_some();
        faces.push(lifted);

        // The slivers, for each edge u -> v of this face that runs along the upper ring from or
        // to a shared point. The face holds that edge at the lifted heights; the surface above
        // holds it on the upper ring, the other way round. The quad between the two, walked
        // v(lifted) -> u(lifted) -> u(upper) -> v(upper), closes the gap; a corner that is not
        // shared is already on the upper ring, and its half of the quad is empty.
        for j in 0..3 {
            let k = (j + 1) % 3;
            if !(shared(j) || shared(k)) {
                continue;
            }
            let (Some(u_upper), Some(v_upper)) = (on_upper[j], on_upper[k]) else {
                continue;
            };
            if !upper_edges.contains(&(u_upper, v_upper)) {
                continue;
            }
            if shared(j) {
                faces.push([lifted[k], lifted[j], u_upper]);
            }
            if shared(k) {
                faces.push([lifted[k], u_upper, v_upper]);
            }
        }
    }

    // An edge both rings run along in the same direction is no boundary of the 2D area at all,
    // so the fill holds no face along it; the wall there is a vertical quad from the edge on the
    // lower ring up to the same edge on the upper ring.
    let lower_edges: HashSet<(i32, i32)> =
        lower.indices.iter().flat_map(|l| loop_edges(l)).collect();
    for (path, indices) in upper.loops.iter().zip(&upper.indices) {
        for i in 0..path.len() {
            let next = (i + 1) % path.len();
            let (a, b) = (path[i], path[next]);
            let (Some(a0), Some(b0)) = (lower.vertex_at(a.x, a.y), lower.vertex_at(b.x, b.y))
            else {
                continue;
            };
            if !lower_edges.contains(&(a0, b0)) {
                continue;
            }
            let (a1, b1) = (indices[i], indices[next]);
            let mut quad = [[a0, b0, b1], [a0, b1, a1]];
            if !lower_is_outer {
                for face in &mut quad {
                    face.swap(0, 2);
                }
            }
            faces.extend(quad);
        }
    }
    Some(faces)
}

/// Walls the rings loop to loop: each upper loop is zipped to the lower loop of the same kind
/// (outline or hole) whose points come nearest its own, the nearest pairs first, and every loop
/// left without a partner - a feature that vanished or appeared between the rings - is capped flat
/// on its own ring.
fn zipped_band(rings: &mut RingMesh, lower: &PlacedRing, upper: &PlacedRing) {
    let mut candidates = Vec::new();
    for (u, upper_loop) in upper.loops.iter().enumerate() {
        for (l, lower_loop) in lower.loops.iter().enumerate() {
            if upper_loop.is_empty()
                || lower_loop.is_empty()
                || (area(upper_loop) > 0.0) != (area(lower_loop) > 0.0)
            {
                continue;
            }
            let (i, j) = best_start_indices(lower_loop, upper_loop);
            let (dx, dy) = (
                (lower_loop[i].x - upper_loop[j].x) as i128,
                (lower_loop[i].y - upper_loop[j].y) as i128,
            );
            candidates.push((dx * dx + dy * dy, u, l));
        }
    }
    // nearest first; ties go to the earlier upper loop, then the earlier lower loop
    candidates.sort_unstable();

    let mut upper_partner = vec![None; upper.loops.len()];
    let mut lower_taken = vec![false; lower.loops.len()];
    for (_, u, l) in candidates {
        if upper_partner[u].is_none() && !lower_taken[l] {
            upper_partner[u] = Some(l);
            lower_taken[l] = true;
        }
    }

    for (u, partner) in upper_partner.iter().enumerate() {
        if let Some(l) = *partner {
            zip(rings, lower, l, upper, u);
        }
    }
    let lower_alone: Vec<usize> = (0..lower.loops.len())
        .filter(|&l| !lower_taken[l])
        .collect();
    let upper_alone: Vec<usize> = (0..upper.loops.len())
        .filter(|&u| upper_partner[u].is_none())
        .collect();
    // the solid is below a lower loop that ends here, and above an upper loop that starts here
    rings.cap(lower, &lower_alone, true);
    rings.cap(upper, &upper_alone, false);
}

/// Walls lower loop `l` to upper loop `u` by walking both at once from their closest points,
/// each step closing one triangle ([`zip_steps_on_a`] picks the loop). Every edge of
/// both loops is used exactly once, lower forward and upper backward.
fn zip(rings: &mut RingMesh, lower: &PlacedRing, l: usize, upper: &PlacedRing, u: usize) {
    let (a_points, a) = (&lower.loops[l], &lower.indices[l]);
    let (b_points, b) = (&upper.loops[u], &upper.indices[u]);
    let distance = |p: clipper2_rust::Point64, q: clipper2_rust::Point64| {
        let (dx, dy) = ((p.x - q.x) as i128, (p.y - q.y) as i128);
        dx * dx + dy * dy
    };
    let (start_a, start_b) = best_start_indices(a_points, b_points);
    let (mut cur_a, mut cur_b) = (start_a, start_b);
    let (mut looped_a, mut looped_b) = (false, false);
    loop {
        let next_a = (cur_a + 1) % a.len();
        let next_b = (cur_b + 1) % b.len();
        let advancing_a = distance(b_points[cur_b], a_points[next_a]);
        let advancing_b = distance(a_points[cur_a], b_points[next_b]);
        if zip_steps_on_a(
            (cur_a, cur_b),
            (next_a, next_b),
            (start_a, start_b),
            (looped_a, looped_b),
            advancing_a <= advancing_b,
        ) {
            rings.add(a[cur_a], a[next_a], b[cur_b]);
            cur_a = next_a;
            looped_a = cur_a == start_a;
        } else {
            rings.add(a[cur_a], b[next_b], b[cur_b]);
            cur_b = next_b;
            looped_b = cur_b == start_b;
        }
        // Both loops walked all the way round. (Being back at both starts is not enough: a
        // small loop can be walked whole before the other has taken a step.)
        if looped_a && looped_b {
            break;
        }
    }
}
