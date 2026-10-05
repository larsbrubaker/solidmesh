//! A bevel ROLLED along a polygonal wall (`Bevel/Surface/SurfaceRollAcrossTheWall.cs`, a static
//! class): the profile is tangent to the flat face and to the one facet of the wall it actually
//! touches, so the centre moves with the curvature and the setback on the curved side is measured
//! ALONG the surface rather than across a chord of it. What lands here is the file's types - what a
//! chain edge's two landing surfaces turned out to be ([`SurfaceRollReading`]), which side is the
//! wall and what the roll reads of the two ([`SurfaceRollAcrossTheWallSides`]), and where one
//! facet's solve put the wall foot ([`SurfaceRollAcrossTheWallLanding`]); the roll itself lands
//! with the plan package.
//!
//! A fillet flows across a smooth mating surface staying tangent to it, and on a tessellated wall
//! "tangent to it" has to mean tangent to ONE facet: the facet k of the section walk where the
//! circle of radius `size` tangent to the flat face's plane and to plane(k) has its wall foot
//! inside facet k's own span. On a convex wall that k is unique, because the foot runs forward
//! along the section as the facets turn away and each facet claims one stretch of it. The section
//! walk is `SurfaceRunAcross`'s own - the very walk the ROOM across the same wall is measured by -
//! so a size the room allows and the facet this lands on cannot disagree about how far along the
//! surface the bevel reaches. Everything is drawn in the section plane, square to the chain edge,
//! which is why a wall whose section crosses only facet 0 comes back with exactly the unrolled
//! plan's numbers. One curved side only: where BOTH sides curve it is refused by name
//! (`SurfaceLandsOnTwoCurvedFaces`) rather than approximated, once each side has been asked whether
//! the plain ball already stands clear of it (`SurfaceWallStandsClear`).

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use vector_math::cs_math::{max_f64, min_f64};
use vector_math::Vector3;

use super::surface_run_across::{
    nothing_across_it_can_curve, SurfaceRunAcrossRegionMapsCache, VERTEX_TOLERANCE,
};
use super::surface_run_across_walk::try_walk_one_section;
use super::{
    SurfaceLine, SurfaceRolledEdge, SurfaceRunAcrossSectionStep, SurfaceVertexPool,
    SurfaceWallSection,
};
use crate::topology::{ChainEdge, FaceGroup};
use polygon_mesh::Mesh;

/// C# `TheMiddle`: where along the chain edge the section the roll is drawn in is taken.
pub const THE_MIDDLE: f64 = 0.5;

/// C# `SectionsAlongTheEdge` (this class's own copy): the sections the band is gathered over.
pub const SECTIONS_ALONG_THE_EDGE: [f64; 3] = [0.25, 0.5, 0.75];

/// C# `SurfaceRollReading`: what a chain edge's two landing surfaces turned out to be, and so how
/// it was planned.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceRollReading {
    /// Neither side curves along its section, so the plan's own two-plane construction is exact.
    #[default]
    NotCurved = 0,

    /// One side is a curved wall, and the profile was rolled along it to the facet it touches.
    Rolled,

    /// Nothing was planned, and the advisory handed back says why.
    Refused,
}

/// C# `SurfaceRollAcrossTheWall.Sides` (a readonly struct): which side of the edge is the wall, and
/// everything about the two of them the roll reads.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceRollAcrossTheWallSides {
    /// The chain edge being rolled along.
    pub chain_edge: ChainEdge,
    /// The unit direction along the chain edge.
    pub along: Vector3,
    /// Whether the wall is the edge's LEFT face (the flat face is then the right).
    pub wall_on_the_left: bool,
    /// The facets the middle section crossed, which the roll walks.
    pub steps: Vec<SurfaceRunAcrossSectionStep>,
    /// The wall's region, shared with the cache it was flooded into.
    pub wall_region: Arc<FaceGroup>,
    /// The wall's inward direction at the chain edge, square to the edge in facet 0's plane.
    pub wall_inward: Vector3,
    /// The flat face's outward normal.
    pub flat_normal: Vector3,
    /// The flat face's inward direction, square to the edge and in its plane.
    pub flat_inward: Vector3,
}

/// C# `SurfaceRollAcrossTheWall.Landing` (private): where one facet's own solve put the profile's
/// wall foot, against that facet's two ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceRollAcrossTheWallLanding {
    /// There is no circle to place at all - the two traces are parallel, or the flat foot is behind
    /// the edge.
    #[default]
    Nowhere = 0,

    /// Inside the facet, which is the facet that takes the roll.
    OnTheFacet,

    /// Short of its near end, so the profile touches the wall before this facet begins.
    BeforeIt,

    /// Past its far end, so the profile touches the wall after this facet ends.
    PastIt,
}

impl SurfaceRollAcrossTheWallSides {
    /// C#'s constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        chain_edge: ChainEdge,
        along: Vector3,
        wall_on_the_left: bool,
        steps: Vec<SurfaceRunAcrossSectionStep>,
        wall_region: Arc<FaceGroup>,
        wall_inward: Vector3,
        flat_normal: Vector3,
        flat_inward: Vector3,
    ) -> Self {
        SurfaceRollAcrossTheWallSides {
            chain_edge,
            along,
            wall_on_the_left,
            steps,
            wall_region,
            wall_inward,
            flat_normal,
            flat_inward,
        }
    }

    /// C# `Origin`: where the section the construction is drawn in meets the chain edge.
    pub fn origin(&self) -> Vector3 {
        self.steps[0].from
    }

    /// C# `Midpoint`: the middle of the chain edge, which is where a refusal is reported at.
    pub fn midpoint(&self) -> Vector3 {
        (self.chain_edge.from() + self.chain_edge.to()) / 2.0
    }
}

/// C# `CurvesAlongItsSection`: the middle section's walk across `region` when that walk turns at
/// all, or `None` when every facet lies in the one plane or the walk stays flat.
pub fn curves_along_its_section(
    mesh: &Mesh,
    region: &FaceGroup,
    chain_edge: &ChainEdge,
    along: Vector3,
    inward: Vector3,
    cache: &mut SurfaceRunAcrossRegionMapsCache,
) -> Option<Vec<SurfaceRunAcrossSectionStep>> {
    if nothing_across_it_can_curve(mesh, region, Some(&mut *cache)) {
        // Every facet of it lies in the one plane, so the section cannot leave that plane and the
        // walk would only say so the long way round.
        return None;
    }

    match try_walk_one_section(
        mesh,
        region,
        chain_edge.from(),
        chain_edge.to(),
        along,
        inward,
        THE_MIDDLE,
        Some(cache),
    ) {
        Some((steps, true)) => Some(steps),
        _ => None,
    }
}

/// C# `TryTouchTheCrease`: the profile resting on the crease between `near` and `far` instead of
/// on either facet - `(foot, flat_foot, centre)`, or `None` when no such circle stands on both.
#[allow(clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
pub fn try_touch_the_crease(
    side: &SurfaceRollAcrossTheWallSides,
    origin: Vector3,
    flat_normal: Vector3,
    near: &SurfaceRunAcrossSectionStep,
    far: &SurfaceRunAcrossSectionStep,
    far_trace: Vector3,
    offset: f64,
    on_the_trace: f64,
) -> Option<(Vector3, Vector3, Vector3)> {
    let mut near_trace = near.to - near.from;

    if !(near_trace.length_squared() > 0.0) {
        return None;
    }

    near_trace = near_trace.get_normal();

    let size = offset.abs();
    let wedge_sign = if offset >= 0.0 { 1.0 } else { -1.0 };
    let towards_the_near = out_of_the_facet(side.along, near_trace, near.normal) * wedge_sign;
    let towards_the_far = out_of_the_facet(side.along, far_trace, far.normal) * wedge_sign;
    let between = towards_the_near.dot(towards_the_far);

    let vertex = in_the_section(far.from, origin, side.along);
    let mut along_the_line = side.along.cross(flat_normal);

    if !(along_the_line.length_squared() > 0.0) || !(size > 0.0) {
        return None;
    }

    along_the_line = along_the_line.get_normal();

    let from_the_vertex = origin + (flat_normal * offset) - vertex;
    let half = from_the_vertex.dot(along_the_line);
    let under = (half * half) - from_the_vertex.length_squared() + (size * size);

    if !(under >= 0.0) {
        return None;
    }

    let root = under.sqrt();

    for at in [-half + root, -half - root] {
        let candidate = origin + (flat_normal * offset) + (along_the_line * at);
        let outward = (candidate - vertex) / size;

        if outward.dot(towards_the_near) < between - on_the_trace
            || outward.dot(towards_the_far) < between - on_the_trace
        {
            continue;
        }

        let touches_the_flat = candidate - (flat_normal * offset);

        if (touches_the_flat - origin).dot(side.flat_inward) < -on_the_trace {
            continue;
        }

        return Some((vertex, touches_the_flat, candidate));
    }

    None
}

/// C# `TheReading`: the rolled edge a landing on `facet` reads as - the wall and flat tangent lines,
/// the centre line, the band the setback covers and the section it was drawn in.
#[allow(clippy::too_many_arguments)]
pub fn the_reading(
    mesh: &Mesh,
    side: &SurfaceRollAcrossTheWallSides,
    facet: i32,
    trace: Vector3,
    foot: Vector3,
    flat_foot: Vector3,
    centre: Vector3,
    reached: f64,
    round_profile: bool,
    on_the_trace: f64,
    cache: &mut SurfaceRunAcrossRegionMapsCache,
    rests_on_the_crease: bool,
) -> SurfaceRolledEdge {
    // The wall foot's own line, which where the roll RESTS ON A CREASE is that crease: the profile
    // touches the wall along it and is tangent to neither facet's plane, so it is carried on the
    // section for the readings that would otherwise look for a facet to be tangent to.
    let wall_tangent_line = SurfaceLine::new(at_the_edge_start(foot, side), side.along);

    SurfaceRolledEdge::new(
        side.wall_on_the_left,
        facet,
        reached,
        (flat_foot - side.origin()).dot(side.flat_inward),
        trace,
        wall_tangent_line,
        SurfaceLine::new(at_the_edge_start(flat_foot, side), side.along),
        round_profile,
        SurfaceLine::new(at_the_edge_start(centre, side), side.along),
        if round_profile {
            distance_from_the_edge(centre, side)
        } else {
            0.0
        },
        Rc::new(band_across(mesh, side, reached, on_the_trace, cache)),
        Rc::new(SurfaceWallSection::new(
            side.along,
            facet,
            side.steps.clone(),
            rests_on_the_crease,
            wall_tangent_line,
        )),
    )
}

/// C# `InTheSection(Vector3 point, Vector3 origin, Vector3 along)`: `point` carried along the edge
/// into the section plane through `origin`.
pub fn in_the_section(point: Vector3, origin: Vector3, along: Vector3) -> Vector3 {
    point - (along * (point - origin).dot(along))
}

/// C# `TryCentre`: the centre of the circle of radius `offset` tangent to the flat plane and to
/// the facet's plane, drawn in the section plane, or `None` when the planes are parallel.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_centre(
    along: Vector3,
    origin: Vector3,
    flat_normal: Vector3,
    facet_point: Vector3,
    facet_normal: Vector3,
    offset: f64,
) -> Option<Vector3> {
    let across_facet = facet_normal.cross(along);
    let across_section = along.cross(flat_normal);
    let across_flat = flat_normal.cross(facet_normal);
    let determinant = flat_normal.dot(across_facet);

    if !(determinant.abs() > 1e-9) {
        return None;
    }

    let centre = ((across_facet * (origin.dot(flat_normal) + offset))
        + (across_section * (facet_point.dot(facet_normal) + offset))
        + (across_flat * origin.dot(along)))
        / determinant;

    if is_finite_vector(centre) {
        Some(centre)
    } else {
        None
    }
}

/// C# `BandAcross`: every wall facet a section walk crosses before it has run `setback` along the
/// surface, over all of [`sections_of`]'s sections.
pub fn band_across(
    mesh: &Mesh,
    side: &SurfaceRollAcrossTheWallSides,
    setback: f64,
    on_the_trace: f64,
    cache: &mut SurfaceRunAcrossRegionMapsCache,
) -> HashSet<i32> {
    let mut band = HashSet::new();

    for fraction in sections_of(side, on_the_trace) {
        let Some((steps, _)) = try_walk_one_section(
            mesh,
            &side.wall_region,
            side.chain_edge.from(),
            side.chain_edge.to(),
            side.along,
            side.wall_inward,
            fraction,
            Some(&mut *cache),
        ) else {
            continue;
        };

        for step in &steps {
            if step.run_to_entry > setback + on_the_trace {
                break;
            }

            band.insert(step.face);
        }
    }

    band
}

/// C# `OutOfTheFacet`: the unit direction square to the edge and to `trace` that leans the same
/// way as the facet's normal (the normal itself when the two are parallel).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn out_of_the_facet(along: Vector3, trace: Vector3, facet_normal: Vector3) -> Vector3 {
    let mut outward = along.cross(trace);

    if !(outward.length_squared() > 0.0) {
        return facet_normal;
    }

    outward = outward.get_normal();

    if outward.dot(facet_normal) >= 0.0 {
        outward
    } else {
        -outward
    }
}

/// C# `InTheSection(Vector3 normal, Vector3 along)`: `normal` with its component along the edge
/// taken out, re-normalised (the normal itself when nothing is left).
pub fn in_the_section_of_the_normal(normal: Vector3, along: Vector3) -> Vector3 {
    let in_it = normal - (along * normal.dot(along));

    if in_it.length_squared() > 0.0 {
        in_it.get_normal()
    } else {
        normal
    }
}

/// C# `SectionsOf`: where along the chain edge the band's sections are taken - nudged in from
/// both ends, plus the run-across's own three.
pub fn sections_of(side: &SurfaceRollAcrossTheWallSides, _on_the_trace: f64) -> Vec<f64> {
    let length = (side.chain_edge.to() - side.chain_edge.from()).length();

    let nudge = if length > 0.0 {
        min_f64(0.05, 4.0 * max_f64(length, 1.0) * VERTEX_TOLERANCE / length)
    } else {
        0.0
    };

    let mut sections = vec![nudge];
    sections.extend(SECTIONS_ALONG_THE_EDGE);
    sections.push(1.0 - nudge);
    sections
}

/// C# `AtTheEdgeStart`: `point` carried along the edge into the section through its start.
pub fn at_the_edge_start(point: Vector3, side: &SurfaceRollAcrossTheWallSides) -> Vector3 {
    point - (side.along * (point - side.chain_edge.from()).dot(side.along))
}

/// C# `DistanceFromTheEdge`: how far `point` stands from the chain edge's line.
pub fn distance_from_the_edge(point: Vector3, side: &SurfaceRollAcrossTheWallSides) -> f64 {
    (at_the_edge_start(point, side) - side.chain_edge.from()).length()
}

/// C# `ToleranceOn`: the mesh's own weld distance.
pub fn tolerance_on(mesh: &Mesh) -> f64 {
    SurfaceVertexPool::weld_distance_for(&mesh.vertices)
}

/// C# `IsFinite(double)`.
pub fn is_finite(value: f64) -> bool {
    !value.is_nan() && !value.is_infinite()
}

/// C# `IsFinite(Vector3)`.
pub fn is_finite_vector(value: Vector3) -> bool {
    is_finite(value.x) && is_finite(value.y) && is_finite(value.z)
}
