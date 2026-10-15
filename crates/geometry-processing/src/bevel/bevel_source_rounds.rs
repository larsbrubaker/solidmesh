//! THE ROUNDS THE USER MODELLED, read straight off the working mesh: the cylinder behind a
//! tessellated bore, boss or turned face that a selected concave crease runs along, so the first
//! pass can roll its ball on that cylinder rather than on the plane of whichever facet the crease
//! edge happens to lie in. Labels the part through [`BevelRoundFaces::label`] (with
//! `read_from_the_source_mesh` set), reading surfaces off [`BevelSurfaceLabels`]; its caller is
//! `BevelGroupTools`.
//!
//! **Why it is needed.** [`BevelRoundFaces::create`] describes only the rounds the FIRST PASS cut -
//! it reads them off the resolutions, where a convex edge and the operation's size say exactly
//! what cylinder the cutter inscribed. A cylinder that was already in the part when it arrived has
//! no such record, and without one a crease where a boss meets a wall gets its section inscribed
//! against the plane of ONE facet of the boss. On the "Unexpected Gap In Fillet" fixture that
//! facet is a 0.28 mm boolean sliver, the bead arc stands 0.15 mm off the faceted wall, and the
//! trim at the wall leaves a 0.1 mm slot where the fillet should be solid.
//!
//! **What is fitted, and what is refused.** Only a CYLINDER. Every facet of a cylinder is parallel
//! to its axis, so the axis is the cross product of any two of the surface's normals and every
//! other normal has to be square to it - a sphere, a cone or a free-form surface fails that at once
//! and is left alone, which leaves the status quo standing rather than a fit that is not the
//! surface. The axis point and radius then come from a least-squares circle through the surface's
//! vertices projected square to the axis, and the fit is refused unless every one of them lies on
//! that circle: the facets are exact, so anything but a tight residual means this was not a
//! cylinder either. It is refused too where too little of the cylinder is left to have fitted
//! through - a sliver a boolean cut off a bore answers every test above while saying nothing about
//! which circle it came from (see [`MINIMUM_CHORD_GAPS_TO_FIT_THROUGH`]).
//!
//! **One descriptor per SURFACE.** Every crease edge along one bore names the same cylinder, and
//! the span the descriptor carries is the whole surface's rather than any one edge's, so the first
//! edge to reach a surface describes it and the rest read what it left. The span is measured about
//! the radial through that edge's midpoint, which puts the crease at phi = 0 and keeps a surface
//! that goes the whole way round from being cut in half by the branch of atan2.
//!
//! The circle fit itself (Kasa, its 3 by 3 solve, the chord count and the angular span) is in
//! `bevel_source_rounds_fit.rs`.

use std::collections::{HashMap, HashSet};
use std::f64::consts::{PI, TAU};

use indexmap::IndexSet;
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

use super::bevel_source_rounds_fit::{angular_span, count_the_chords, try_fit_circle};
use super::{BevelRoundDescriptor, BevelRoundFaces, BevelSurfaceLabels};
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge};

/// How far off square to the fitted axis a facet's normal may point and still be a cylinder's. It
/// is the whole of what tells a cylinder from a cone or a sphere, so it is tight: the facets of a
/// tessellated cylinder are exactly parallel to its axis, and nothing but arithmetic moves them.
const SQUARE_TO_THE_AXIS: f64 = 1e-3;

/// How near two facet normals must point before they are read as one, when counting how many
/// chords the surface was drawn with. Well inside the smallest turn any tessellation makes.
pub(super) const ONE_NORMAL_RADIANS: f64 = 1e-6;

/// How square to the first normal a partner has to be before [`try_read_the_axis`] stops looking
/// for a squarer one. The cross product of two unit normals this far apart - about 45 degrees -
/// already fixes their common perpendicular to well inside the tolerance every other test here is
/// made at, and a cylinder tessellated finely enough to be worth rolling on has plenty of pairs
/// that far apart.
const SQUARE_ENOUGH_TO_STOP_LOOKING: f64 = 0.7;

/// The fewest of the surface's own chord gaps its qualifying vertices have to span before a circle
/// is fitted through them. The chord count reads the MEDIAN gap between neighbouring normals, which
/// says how finely the surface was drawn and nothing at all about how much of the cylinder
/// survived: three facets left of a forty chord bore by a boolean report forty segments just as
/// the whole bore does, and the three or four points they leave lie on an unbounded family of
/// circles that the Kasa fit picks between on rounding alone. Four gaps is the least that leaves
/// the fit over-determined by a margin.
const MINIMUM_CHORD_GAPS_TO_FIT_THROUGH: i32 = 4;

/// The rounds already in the part that a selected concave crease runs along, with the part's faces
/// labelled against them, or `None` where there are none - which is every part with no modelled
/// round under its selection, and every chamfer.
///
/// `part` is the working mesh the tools are being built on; `resolutions` what the pass made of the
/// groups; `eligible` which of them a tool is going to be built for; `surface_labels` the
/// rebuild's reading of which faces are one smooth surface; `size` the operation's size, which
/// every labelling tolerance is a fraction of. `round_profile` is whether the tool's profile is an
/// arc: a chamfer's section is not a rolling ball's, and both readers of the result are gated on
/// the same flag, so nothing is fitted at all for one.
pub fn create(
    part: &Mesh,
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
    surface_labels: &mut BevelSurfaceLabels,
    size: f64,
    round_profile: bool,
) -> Option<BevelRoundFaces> {
    if !round_profile {
        return None;
    }

    let mut rounds = Vec::new();

    // Keyed on the tolerance the labelling was made at as well as the label, because two groups
    // may have been picked at different curve tolerances and their labellings are then different
    // arrays. Only ever asked "seen yet?", so a HashSet's order cannot leak out.
    let mut described: HashSet<(u64, i32)> = HashSet::new();

    for (i, resolution) in resolutions.iter().enumerate() {
        if !eligible[i] || !resolution.is_resolved() {
            continue;
        }

        let tolerance = resolution.group.curve_tolerance_degrees;
        let mut surfaces: Option<SurfacesAtATolerance> = None;

        for edge in &resolution.edges {
            if edge.convexity != EdgeConvexity::Concave {
                continue;
            }

            // Asked for only once a concave crease has actually turned up, for the same reason
            // BevelGroupTools asks for it lazily: labelling is a flood over every face of the part.
            if surfaces.is_none() {
                let labels = surface_labels.at(tolerance);
                let faces_of_surface = gather_surfaces(labels);
                surfaces = Some((labels, faces_of_surface));
            }

            let Some((labels, faces_of_surface)) = surfaces.as_ref() else {
                continue;
            };

            for face in [edge.face0, edge.face1] {
                try_describe(
                    part,
                    labels,
                    faces_of_surface,
                    &mut described,
                    &mut rounds,
                    tolerance,
                    edge,
                    face,
                    size,
                );
            }
        }
    }

    if rounds.is_empty() {
        None
    } else {
        Some(BevelRoundFaces::label(part, &rounds, size, true))
    }
}

/// One labelling of the part and the faces of each of its surfaces.
type SurfacesAtATolerance<'l> = (&'l [i32], HashMap<i32, Vec<i32>>);

/// A tolerance as a set key by value: -0 equals +0 and every NaN equals every other.
fn tolerance_key(tolerance: f64) -> u64 {
    if tolerance == 0.0 {
        0.0f64.to_bits()
    } else if tolerance.is_nan() {
        f64::NAN.to_bits()
    } else {
        tolerance.to_bits()
    }
}

/// Fits the surface one face of a crease belongs to, unless that surface has been fitted already.
#[allow(clippy::too_many_arguments)]
fn try_describe(
    part: &Mesh,
    labels: &[i32],
    faces_of_surface: &HashMap<i32, Vec<i32>>,
    described: &mut HashSet<(u64, i32)>,
    rounds: &mut Vec<BevelRoundDescriptor>,
    tolerance: f64,
    edge: &FeatureEdge,
    face: i32,
    size: f64,
) {
    if face < 0
        || face as usize >= labels.len()
        || !described.insert((tolerance_key(tolerance), labels[face as usize]))
    {
        return;
    }

    let faces = &faces_of_surface[&labels[face as usize]];

    if let Some(round) = try_fit(part, faces, (edge.start + edge.end) * 0.5) {
        BevelRoundFaces::add(rounds, round, size * 1e-3);
    }
}

/// Which faces carry each surface label. Only looked up, never iterated.
fn gather_surfaces(labels: &[i32]) -> HashMap<i32, Vec<i32>> {
    let mut gathered: HashMap<i32, Vec<i32>> = HashMap::new();

    for (face, &label) in labels.iter().enumerate() {
        gathered.entry(label).or_default().push(face as i32);
    }

    gathered
}

fn corners(part: &Mesh, face: i32) -> (Vector3, Vector3, Vector3, [i32; 3]) {
    let mesh_face = &part.faces[face as usize];

    (
        Vector3::from_vector3_float(part.vertices[mesh_face.v0 as usize]),
        Vector3::from_vector3_float(part.vertices[mesh_face.v1 as usize]),
        Vector3::from_vector3_float(part.vertices[mesh_face.v2 as usize]),
        [mesh_face.v0, mesh_face.v1, mesh_face.v2],
    )
}

/// The cylinder one smooth surface of the part is, or `None` where it is not one. `faces` is every
/// face of the surface; `through` the crease midpoint the span is measured about, which puts the
/// crease at phi = 0.
pub fn try_fit(part: &Mesh, faces: &[i32], through: Vector3) -> Option<BevelRoundDescriptor> {
    let mut normals = Vec::with_capacity(faces.len());
    // Insertion ordered: the circle fit's sums run over it, and their order is their rounding.
    let mut vertices: IndexSet<i32> = IndexSet::new();

    // WHICH FACETS EACH VERTEX IS ON, which is what says whether it is on the analytic cylinder. A
    // tessellated cylinder's vertices lie on the circle only where two FACETS meet: everything a
    // boolean added is somewhere on the interior of one facet, and a facet is a chord, so those
    // points are up to a whole sagitta inside the radius. Fitting a circle through them would fail
    // its own residual on a perfectly good cylinder - which is exactly what it did.
    let mut facets_at_vertex: HashMap<i32, Vec<Vector3>> = HashMap::new();
    let same_normal = ONE_NORMAL_RADIANS.cos();

    for &face in faces {
        let (a, b, c, ids) = corners(part, face);
        let normal = (b - a).cross(c - a);

        vertices.extend(ids);

        let length = normal.length();
        if length.is_nan() || length <= 0.0 {
            continue;
        }

        let normal = normal.get_normal();
        normals.push(normal);

        for vertex in ids {
            let on_it = facets_at_vertex.entry(vertex).or_default();

            if !on_it
                .iter()
                .any(|already| already.dot(normal) > same_normal)
            {
                on_it.push(normal);
            }
        }
    }

    if normals.len() < 3 || vertices.len() < 6 {
        return None;
    }

    let direction = try_read_the_axis(&normals)?;

    // The plane square to the axis, in which the surface is a circle.
    let e1 = perpendicular_to(direction);
    let e2 = direction.cross(e1).get_normal();

    // Fitted about the crease so the algebraic fit is well conditioned wherever the part sits in
    // space - a bore a metre from the origin would otherwise square coordinates a thousand times
    // its own radius and lose the fit in the rounding.
    let origin = through;
    let mut flat = Vec::with_capacity(vertices.len());

    for &vertex in &vertices {
        if facets_at_vertex.get(&vertex).map_or(0, Vec::len) < 2 {
            continue;
        }

        let relative = Vector3::from_vector3_float(part.vertices[vertex as usize]) - origin;

        flat.push(Vector2::new(relative.dot(e1), relative.dot(e2)));
    }

    if flat.len() < 3 {
        return None;
    }

    let (centre, radius) = try_fit_circle(&flat)?;

    let axis_point = origin + (e1 * centre.x) + (e2 * centre.y);

    if !points_away_from_the_axis(part, faces, axis_point, direction) {
        return None;
    }

    let segments = count_the_chords(&normals, e1, e2);

    if segments < BevelRoundFaces::MINIMUM_SEGMENTS_TO_ROLL_ON {
        return None;
    }

    // AND ENOUGH OF THE CYLINDER HAS TO BE LEFT to have fitted it through. The count above is the
    // whole cylinder's, read off the median turn between neighbouring facets, so a sliver a
    // boolean left of a finely drawn bore passes it while carrying three points - see
    // MINIMUM_CHORD_GAPS_TO_FIT_THROUGH.
    if angular_span(&flat, centre)
        < MINIMUM_CHORD_GAPS_TO_FIT_THROUGH as f64 * TAU / segments as f64
    {
        return None;
    }

    let mut first_radial = through - axis_point;
    first_radial = first_radial - (direction * first_radial.dot(direction));

    let first_length = first_radial.length();
    if first_length.is_nan() || first_length <= 1e-9 {
        return None;
    }

    let first_radial = first_radial.get_normal();

    let second_radial = direction.cross(first_radial).get_normal();
    let mut axial_from = f64::MAX;
    let mut axial_to = f64::MIN;
    let mut span_low = 0.0;
    let mut span_high = 0.0;

    for &vertex in &vertices {
        let relative = Vector3::from_vector3_float(part.vertices[vertex as usize]) - axis_point;
        let along = relative.dot(direction);

        axial_from = vector_math::ieee_math::min_f64(axial_from, along);
        axial_to = vector_math::ieee_math::max_f64(axial_to, along);

        let radial = relative - (direction * along);
        let phi = radial.dot(second_radial).atan2(radial.dot(first_radial));

        span_low = vector_math::ieee_math::min_f64(span_low, phi);
        span_high = vector_math::ieee_math::max_f64(span_high, phi);
    }

    if span_high - span_low > PI {
        // More than half a turn: the surface's own ends are no longer separable from the branch
        // cut of atan2, so it is taken as going the whole way round - which a bore or a boss does.
        span_low = -PI;
        span_high = PI;
    }

    Some(BevelRoundDescriptor::with_span(
        axis_point,
        direction,
        radius,
        first_radial,
        span_low,
        span_high,
        axial_from,
        axial_to,
        segments,
    ))
}

/// Whether the surface's material is INSIDE the cylinder - a post, a boss, a turned face - rather
/// than outside it, which is a bore.
///
/// The whole of [`BevelRoundDescriptor`] is written for a round the material is inside of: its
/// spine stands the ball's centre at `radius + ball_radius` from the axis, which is where a ball
/// rolling on the OUTSIDE of a cylinder keeps it. A ball rolling in a bore keeps its centre at
/// `radius - ball_radius`, on the other side of the surface, and handing a bore to that solve
/// gives a station that is not on the part at all - measured on the "Through Hole Slab" fixture
/// it took 27 mm^3 off the counterbore's fillet. A bore is left to the planar section it always
/// had until the descriptor can carry both signs.
fn points_away_from_the_axis(
    part: &Mesh,
    faces: &[i32],
    axis_point: Vector3,
    direction: Vector3,
) -> bool {
    for &face in faces {
        let (a, b, c, _) = corners(part, face);
        let normal = (b - a).cross(c - a);

        let length = normal.length();
        if length.is_nan() || length <= 0.0 {
            continue;
        }

        let relative = ((a + b + c) / 3.0) - axis_point;
        let radial = relative - (direction * relative.dot(direction));

        let radial_length = radial.length();
        if radial_length.is_nan()
            || radial_length <= 1e-9
            || normal.get_normal().dot(radial.get_normal()) <= 0.0
        {
            return false;
        }
    }

    true
}

/// The axis every one of these facet normals is square to, or `None` where no such direction
/// exists - which is every surface that is not a cylinder.
///
/// ONE PASS, against the FIRST normal rather than over every pair. Every facet of a cylinder is
/// parallel to its axis, so the cross of ANY two non-parallel normals of it IS the axis and there
/// is nothing a second pivot could find that this one cannot. The pair scan this replaced was
/// quadratic in the surface's facets and ran to the very end on the surfaces that fail - a planar
/// wall, whose every normal is the same one - which is most of what it is asked about: on a part
/// whose selected crease runs along a twenty thousand facet surface that is two hundred million
/// cross products to answer "not a cylinder". The most nearly square PARTNER of the pivot is still
/// the one taken, because two nearly parallel normals fix the axis only to within however parallel
/// they are, and a partner square enough that nothing better is worth looking for ends the pass
/// there.
fn try_read_the_axis(normals: &[Vector3]) -> Option<Vector3> {
    let pivot = normals[0];
    let mut best = Vector3::ZERO;
    let mut widest = 0.0;

    for normal in &normals[1..] {
        let cross = pivot.cross(*normal);
        let how_square = cross.length();

        if how_square > widest {
            widest = how_square;
            best = cross;

            if widest > SQUARE_ENOUGH_TO_STOP_LOOKING {
                break;
            }
        }
    }

    if widest < 1e-3 {
        return None;
    }

    let direction = best.get_normal();

    if normals
        .iter()
        .any(|normal| normal.dot(direction).abs() > SQUARE_TO_THE_AXIS)
    {
        return None;
    }

    Some(direction)
}

/// Any unit vector square to one direction.
fn perpendicular_to(direction: Vector3) -> Vector3 {
    let away = if direction.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };

    direction.cross(away).get_normal()
}
