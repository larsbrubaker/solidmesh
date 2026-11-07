//! The WALK across a curved landing surface: how far it is across a curved region from a chain
//! edge, measured ALONG the surface by sections square to the edge
//! ([`try_across_a_curved_region`]), and one such section's facets ([`try_walk_one_section`]). The
//! records, the region's maps and the mesh helpers are in `surface_run_across.rs`.

use std::collections::HashSet;
use std::rc::Rc;

use polygon_mesh::Mesh;
use vector_math::{ieee_math, Vector3};

use super::surface_run_across::{
    edge_key, maps_of, normal_of, vertices_of, SurfaceRunAcrossRegionMapsCache, FLAT_COSINE,
    NO_EDGE, SECTIONS_ALONG_THE_EDGE, SHORTEST_STEP, VERTEX_TOLERANCE,
};
use super::{
    SurfaceRunAcrossRegionMaps, SurfaceRunAcrossSectionStart, SurfaceRunAcrossSectionStep,
};
use crate::topology::FaceGroup;

/// How far it is across the curved `region` from the chain edge `from`..`to` (running `along`,
/// the region lying `inward`), the least of three sections square to the edge walked ALONG the
/// surface to the region's rim. `None` where a section cannot be walked or none of them curves -
/// then the straight reading is exact and answers instead.
#[allow(clippy::too_many_arguments)]
pub fn try_across_a_curved_region(
    mesh: &Mesh,
    region: &FaceGroup,
    from: Vector3,
    to: Vector3,
    along: Vector3,
    inward: Vector3,
    cache: Option<&mut SurfaceRunAcrossRegionMapsCache>,
) -> Option<f64> {
    if region.faces().len() < 2 {
        return None;
    }

    let start = try_start_at_the_edge(mesh, region, from, to, cache)?;

    let mut nearest = f64::INFINITY;
    let mut curved = false;

    for fraction in SECTIONS_ALONG_THE_EDGE {
        // A section that cannot be walked leaves this reading with nothing to say.
        let (section, curved_here) = try_section(
            mesh,
            &start,
            from + ((to - from) * fraction),
            along,
            inward,
            None,
        )?;

        curved |= curved_here;
        nearest = ieee_math::min_f64(nearest, section);
    }

    if !curved {
        // Every section stayed on one plane, so the straight reading is exact here.
        return None;
    }

    (nearest > 0.0).then_some(nearest)
}

/// The facets one section crosses at `fraction` along the chain edge, and whether it curved;
/// `None` where it cannot be started or crosses nothing.
#[allow(clippy::too_many_arguments)]
pub fn try_walk_one_section(
    mesh: &Mesh,
    region: &FaceGroup,
    from: Vector3,
    to: Vector3,
    along: Vector3,
    inward: Vector3,
    fraction: f64,
    cache: Option<&mut SurfaceRunAcrossRegionMapsCache>,
) -> Option<(Vec<SurfaceRunAcrossSectionStep>, bool)> {
    if region.faces().is_empty() {
        return None;
    }

    let start = try_start_at_the_edge(mesh, region, from, to, cache)?;

    // The run itself is of no interest here - what the caller wants is the facets.
    let mut walked = Vec::new();
    let curved = try_section(
        mesh,
        &start,
        from + ((to - from) * fraction),
        along,
        inward,
        Some(&mut walked),
    )
    .is_some_and(|(_, curved)| curved);

    (!walked.is_empty()).then_some((walked, curved))
}

/// Where a walk across `region` from the chain edge `from`..`to` begins: the facet carrying the
/// edge (by position first, by measuring after), its normal and the edge's key.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_start_at_the_edge(
    mesh: &Mesh,
    region: &FaceGroup,
    from: Vector3,
    to: Vector3,
    cache: Option<&mut SurfaceRunAcrossRegionMapsCache>,
) -> Option<SurfaceRunAcrossSectionStart> {
    let near_enough = ieee_math::max_f64((to - from).length(), 1.0) * VERTEX_TOLERANCE;
    let maps = maps_of(cache, mesh, region);

    if !maps.usable {
        return None;
    }

    let mut start_face = maps.facet_on_both(from, to);

    if start_face < 0 {
        start_face = facet_on(mesh, region, from, to, near_enough);
    }

    if start_face < 0 {
        return None;
    }

    let reference = normal_of(mesh, start_face);

    if !(reference.length_squared() > 0.0) {
        return None;
    }

    let entry = edge_key(
        vertex_at(mesh, start_face, from),
        vertex_at(mesh, start_face, to),
    );

    Some(SurfaceRunAcrossSectionStart::new(
        maps,
        start_face,
        entry,
        reference,
        near_enough,
    ))
}

/// One section from `point` across the region, square to the edge (`along`), starting `inward`:
/// facet by facet to the region's rim. Returns the run and whether any facet turned off the
/// starting plane; infinity where the section comes round to a facet it already crossed (a closed
/// loop, no bound at all). `None` where it crosses nothing. Each facet crossed is added to
/// `steps` where it is kept.
pub fn try_section(
    mesh: &Mesh,
    start: &SurfaceRunAcrossSectionStart,
    point: Vector3,
    along: Vector3,
    inward: Vector3,
    mut steps: Option<&mut Vec<SurfaceRunAcrossSectionStep>>,
) -> Option<(f64, bool)> {
    let maps: &Rc<SurfaceRunAcrossRegionMaps> = &start.maps;
    let near_enough = start.near_enough;
    let reference = start.reference;
    let mut face = start.face;
    let mut entry = start.entry;
    let mut direction = inward;
    let mut point = point;
    let mut total = 0.0;
    let mut curved = false;
    let mut crossed_already = HashSet::new();

    loop {
        if !crossed_already.insert(face) {
            // Round to a facet it has already crossed: nothing stands in front of this cut.
            return Some((f64::INFINITY, curved));
        }

        let Some((exit_edge, exit_point, crossed)) =
            try_cross_the_facet(mesh, face, entry, point, direction)
        else {
            break;
        };

        if let Some(steps) = steps.as_deref_mut() {
            steps.push(SurfaceRunAcrossSectionStep::new(
                face,
                point,
                exit_point,
                normal_of(mesh, face),
                total,
            ));
        }

        total += crossed;
        point = exit_point;

        let Some((next, next_direction, next_entry)) = try_step_on(
            mesh,
            maps,
            face,
            exit_edge,
            exit_point,
            along,
            direction,
            near_enough,
        ) else {
            // The rim of the region, which is what the run is measured to.
            return (total > 0.0).then_some((total, curved));
        };

        face = next;
        direction = next_direction;
        entry = next_entry;

        let normal = normal_of(mesh, face);

        curved |= normal.length_squared() > 0.0 && normal.dot(reference) < FLAT_COSINE;
    }

    (total > 0.0).then_some((total, curved))
}

/// The facet the section steps onto past `exit_edge` (or, through a vertex, past that vertex):
/// the first facet of the region there - other than the one it left - it can cross in its own
/// section direction, with that direction and the edge it may not cross back over.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_step_on(
    mesh: &Mesh,
    maps: &SurfaceRunAcrossRegionMaps,
    face: i32,
    exit_edge: i64,
    exit_point: Vector3,
    along: Vector3,
    coming: Vector3,
    near_enough: f64,
) -> Option<(i32, Vector3, i64)> {
    let at_a_vertex = maps.vertex_at(exit_point, exit_edge, near_enough);
    let candidates = if at_a_vertex >= 0 {
        maps.facets_of_vertex(at_a_vertex).to_vec()
    } else {
        maps.facets_of_edge(exit_edge)
    };

    // Through a vertex the section leaves no edge behind to be kept out of; across an edge it
    // does.
    let entry = if at_a_vertex >= 0 { NO_EDGE } else { exit_edge };

    for candidate in candidates {
        if candidate == face {
            continue;
        }

        let direction = section_direction(mesh, candidate, along, coming);

        if !(direction.length_squared() > 0.0)
            || try_cross_the_facet(mesh, candidate, entry, exit_point, direction).is_none()
        {
            continue;
        }

        return Some((candidate, direction, entry));
    }

    None
}

/// Where the section from `point` running `direction` leaves `face` - the nearest crossing of one
/// of its edges ahead (not the `entry` edge) - as `(exit edge, exit point, length crossed)`.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_cross_the_facet(
    mesh: &Mesh,
    face: i32,
    entry: i64,
    point: Vector3,
    direction: Vector3,
) -> Option<(i64, Vector3, f64)> {
    let normal = normal_of(mesh, face);

    if !(normal.length_squared() > 0.0) {
        return None;
    }

    let sideways = normal.cross(direction);

    if !(sideways.length_squared() > 0.0) {
        return None;
    }

    let sideways = sideways.get_normal();

    let vertices = vertices_of(mesh, face);
    let mut nearest = f64::INFINITY;
    let mut exit = NO_EDGE;
    let mut exit_point = point;

    for i in 0..3 {
        let (a_index, a_position) = vertices[i];
        let (b_index, b_position) = vertices[(i + 1) % 3];
        let key = edge_key(a_index, b_index);

        if key == entry {
            continue;
        }

        let side_a = (a_position - point).dot(sideways);
        let side_b = (b_position - point).dot(sideways);

        if (side_a > 0.0 && side_b > 0.0) || (side_a < 0.0 && side_b < 0.0) || side_a == side_b {
            continue;
        }

        let share = side_a / (side_a - side_b);
        let at = a_position + ((b_position - a_position) * share);
        let ahead = (at - point).dot(direction);

        if ahead > SHORTEST_STEP && ahead < nearest {
            nearest = ahead;
            exit = key;
            exit_point = at;
        }
    }

    if nearest.is_infinite() {
        return None;
    }

    Some((exit, exit_point, (exit_point - point).length()))
}

/// The section's direction in `face`: square to the edge (`along`) in the facet's plane, the way
/// the section was `coming`; zero for a facet of no area or one square to the edge.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn section_direction(mesh: &Mesh, face: i32, along: Vector3, coming: Vector3) -> Vector3 {
    let normal = normal_of(mesh, face);

    if !(normal.length_squared() > 0.0) {
        return Vector3::ZERO;
    }

    let direction = along.cross(normal);

    if !(direction.length_squared() > 0.0) {
        return Vector3::ZERO;
    }

    let direction = direction.get_normal();

    if direction.dot(coming) >= 0.0 {
        direction
    } else {
        -direction
    }
}

/// The first facet of `region` carrying both `from` and `to` (within `near_enough`), or -1.
pub fn facet_on(
    mesh: &Mesh,
    region: &FaceGroup,
    from: Vector3,
    to: Vector3,
    near_enough: f64,
) -> i32 {
    for &face in region.faces() {
        let mut carries_from = false;
        let mut carries_to = false;

        for (_, position) in vertices_of(mesh, face) {
            carries_from |= (position - from).length() <= near_enough;
            carries_to |= (position - to).length() <= near_enough;
        }

        if carries_from && carries_to {
            return face;
        }
    }

    -1
}

/// The vertex of `face` nearest `point`.
pub fn vertex_at(mesh: &Mesh, face: i32, point: Vector3) -> i32 {
    let mut found = -1;
    let mut nearest = f64::INFINITY;

    for (index, position) in vertices_of(mesh, face) {
        let distance = (position - point).length();

        if distance < nearest {
            nearest = distance;
            found = index;
        }
    }

    found
}
