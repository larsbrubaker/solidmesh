//! A bevel ROLLED along a polygonal wall: the profile is tangent to the flat face and to the one
//! facet of the wall it actually touches, so the centre moves with the curvature and the setback on
//! the curved side is measured ALONG the surface rather than across a chord of it. What lands here
//! is the file's types - what a chain edge's two landing surfaces turned out to be
//! ([`SurfaceRollReading`]), which side is the wall and what the roll reads of the two
//! ([`SurfaceRollAcrossTheWallSides`]), and where one facet's solve put the wall foot
//! ([`SurfaceRollAcrossTheWallLanding`]) - and the roll itself, facet by facet ([`try_touch`],
//! [`try_roll`]), and the reading that chooses to roll ([`read`]).
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
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FaceGroup};
use polygon_mesh::Mesh;

/// Where along the chain edge the section the roll is drawn in is taken.
pub const THE_MIDDLE: f64 = 0.5;

/// The sections the band is gathered over.
pub const SECTIONS_ALONG_THE_EDGE: [f64; 3] = [0.25, 0.5, 0.75];

/// What a chain edge's two landing surfaces turned out to be, and so how it was planned.
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

/// Which side of the edge is the wall, and everything about the two of them the roll reads.
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

/// Where one facet's own solve put the profile's wall foot, against that facet's two ends.
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
    /// Builds the sides' reading from its parts.
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

    /// Where the section the construction is drawn in meets the chain edge.
    pub fn origin(&self) -> Vector3 {
        self.steps[0].from
    }

    /// The middle of the chain edge, which is where a refusal is reported at.
    pub fn midpoint(&self) -> Vector3 {
        (self.chain_edge.from() + self.chain_edge.to()) / 2.0
    }
}

/// The middle section's walk across `region` when that walk turns at all, or `None` when every
/// facet lies in the one plane or the walk stays flat.
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

/// The profile resting on the crease between `near` and `far` instead of on either facet - `(foot,
/// flat_foot, centre)`, or `None` when no such circle stands on both.
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

/// The rolled edge a landing on `facet` reads as - the wall and flat tangent lines, the centre
/// line, the band the setback covers and the section it was drawn in.
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

/// `point` carried along the edge into the section plane through `origin`.
pub fn in_the_section(point: Vector3, origin: Vector3, along: Vector3) -> Vector3 {
    point - (along * (point - origin).dot(along))
}

/// The centre of the circle of radius `offset` tangent to the flat plane and to the facet's plane,
/// drawn in the section plane, or `None` when the planes are parallel.
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

/// Every wall facet a section walk crosses before it has run `setback` along the surface, over all
/// of [`sections_of`]'s sections.
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

/// The unit direction square to the edge and to `trace` that leans the same way as the facet's
/// normal (the normal itself when the two are parallel).
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

/// `normal` with its component along the edge taken out, re-normalised (the normal itself when
/// nothing is left).
pub fn in_the_section_of_the_normal(normal: Vector3, along: Vector3) -> Vector3 {
    let in_it = normal - (along * normal.dot(along));

    if in_it.length_squared() > 0.0 {
        in_it.get_normal()
    } else {
        normal
    }
}

/// Where along the chain edge the band's sections are taken - nudged in from both ends, plus the
/// run-across's own three.
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

/// `point` carried along the edge into the section through its start.
pub fn at_the_edge_start(point: Vector3, side: &SurfaceRollAcrossTheWallSides) -> Vector3 {
    point - (side.along * (point - side.chain_edge.from()).dot(side.along))
}

/// How far `point` stands from the chain edge's line.
pub fn distance_from_the_edge(point: Vector3, side: &SurfaceRollAcrossTheWallSides) -> f64 {
    (at_the_edge_start(point, side) - side.chain_edge.from()).length()
}

/// The mesh's own weld distance.
pub fn tolerance_on(mesh: &Mesh) -> f64 {
    SurfaceVertexPool::weld_distance_for(&mesh.vertices)
}

/// Whether `value` is finite.
pub fn is_finite(value: f64) -> bool {
    !value.is_nan() && !value.is_infinite()
}

/// Whether every component of `value` is finite.
pub fn is_finite_vector(value: Vector3) -> bool {
    is_finite(value.x) && is_finite(value.y) && is_finite(value.z)
}

/// Where one facet's solve puts the profile - the wall foot, the flat foot and the centre (all
/// `origin` where it lands nowhere) - and whether that wall foot is on the facet, before its near
/// end or past its far one. A chamfer's foot is its own size along the section.
#[allow(clippy::too_many_arguments)]
pub fn try_touch(
    side: &SurfaceRollAcrossTheWallSides,
    origin: Vector3,
    flat_normal: Vector3,
    facet_point: Vector3,
    facet_normal: Vector3,
    trace: Vector3,
    offset: f64,
    round_profile: bool,
    size: f64,
    step: &SurfaceRunAcrossSectionStep,
    on_the_trace: f64,
) -> (SurfaceRollAcrossTheWallLanding, Vector3, Vector3, Vector3) {
    use SurfaceRollAcrossTheWallLanding as Landing;

    if !round_profile {
        let wanted = size - step.run_to_entry;
        if wanted < -on_the_trace {
            return (Landing::BeforeIt, origin, origin, origin);
        }

        if wanted > step.crossed() + on_the_trace {
            return (Landing::PastIt, origin, origin, origin);
        }

        let foot = step.from + (trace * wanted);
        let flat_foot = origin + (side.flat_inward * size);
        return (Landing::OnTheFacet, foot, flat_foot, origin);
    }

    let Some(centre) = try_centre(
        side.along,
        origin,
        flat_normal,
        facet_point,
        facet_normal,
        offset,
    ) else {
        return (Landing::Nowhere, origin, origin, origin);
    };

    let foot = centre - (facet_normal * offset);
    let flat_foot = centre - (flat_normal * offset);

    if (flat_foot - origin).dot(side.flat_inward) < -on_the_trace {
        return (Landing::Nowhere, foot, flat_foot, centre);
    }

    let along_the_trace = (foot - step.from).dot(trace);

    // Inclusive at both ends: a foot exactly on the crease between two facets is the same point
    // whichever of them claims it.
    if along_the_trace < -on_the_trace {
        return (Landing::BeforeIt, foot, flat_foot, centre);
    }

    let landing = if along_the_trace <= step.crossed() + on_the_trace {
        Landing::OnTheFacet
    } else {
        Landing::PastIt
    };
    (landing, foot, flat_foot, centre)
}

/// The profile rolled along the wall side's section walk to the one facet it touches (or the crease
/// between two it rests on), read as the rolled edge; where it runs out of wall first, the advisory
/// naming the size that fits ([`super::surface_wall_ran_out::advisory`]).
pub fn try_roll(
    mesh: &Mesh,
    side: &SurfaceRollAcrossTheWallSides,
    size: f64,
    round_profile: bool,
    convex: bool,
    cache: &mut SurfaceRunAcrossRegionMapsCache,
) -> Result<SurfaceRolledEdge, BevelAdvisory> {
    let on_the_trace = tolerance_on(mesh);
    let origin = side.origin();
    let flat_normal = in_the_section_of_the_normal(side.flat_normal, side.along);

    // The centre stands in the wedge the profile is drawn in: one sign for convexity.
    let offset = if convex { -1.0 } else { 1.0 } * size;

    // The facet whose own foot ran off its FAR end.
    let mut ran_past_the_end_of = -1;

    for k in 0..side.steps.len() {
        let step = &side.steps[k];
        let trace = step.to - step.from;
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: NaN skips
        let no_trace = !(trace.length_squared() > 0.0);
        if no_trace {
            continue;
        }

        let trace = trace.get_normal();
        let facet_normal = out_of_the_facet(side.along, trace, step.normal);
        let (landing, foot, flat_foot, centre) = try_touch(
            side,
            origin,
            flat_normal,
            step.from,
            facet_normal,
            trace,
            offset,
            round_profile,
            size,
            step,
            on_the_trace,
        );

        if landing == SurfaceRollAcrossTheWallLanding::OnTheFacet {
            let reached = step.run_to_entry + (foot - step.from).dot(trace);
            return Ok(the_reading(
                mesh,
                side,
                k as i32,
                trace,
                foot,
                flat_foot,
                centre,
                reached,
                round_profile,
                on_the_trace,
                cache,
                false,
            ));
        }

        if landing == SurfaceRollAcrossTheWallLanding::BeforeIt
            && ran_past_the_end_of == k as i32 - 1
        {
            if let Some((foot, flat_foot, centre)) = try_touch_the_crease(
                side,
                origin,
                flat_normal,
                &side.steps[k - 1],
                step,
                trace,
                offset,
                on_the_trace,
            ) {
                return Ok(the_reading(
                    mesh,
                    side,
                    k as i32,
                    trace,
                    foot,
                    flat_foot,
                    centre,
                    step.run_to_entry,
                    round_profile,
                    on_the_trace,
                    cache,
                    true,
                ));
            }
        }

        ran_past_the_end_of = if landing == SurfaceRollAcrossTheWallLanding::PastIt {
            k as i32
        } else {
            -1
        };
    }

    Err(super::surface_wall_ran_out::advisory(
        mesh,
        side,
        size,
        round_profile,
        convex,
        flat_normal,
        on_the_trace,
        Some(cache),
    ))
}

/// The face grid of a region, from whoever keeps it beside the region: asked only for a curved side
/// of a round profile.
pub type RegionGridSource<'a> =
    &'a mut dyn FnMut(&FaceGroup) -> Option<Rc<super::SurfaceRegionFaceGrid>>;

/// What the chain edge's two landing surfaces are - two planes (`NotCurved`), one side curving
/// along its section that the profile rolls across (`Rolled`, with the rolled edge), or a refusal
/// (both curved, or the roll ran out of wall). A curved side the plain ball already stands clear of
/// is planned as the plane its own facet is.
#[allow(clippy::too_many_arguments)]
pub fn read(
    mesh: Option<&Mesh>,
    chain_edge: &ChainEdge,
    left_region: &Arc<FaceGroup>,
    right_region: &Arc<FaceGroup>,
    along: Vector3,
    left_normal: Vector3,
    right_normal: Vector3,
    left_inward: Vector3,
    right_inward: Vector3,
    size: f64,
    round_profile: bool,
    convex: bool,
    cache: &mut SurfaceRunAcrossRegionMapsCache,
    grid_of: RegionGridSource,
) -> (
    SurfaceRollReading,
    Option<SurfaceRolledEdge>,
    Option<BevelAdvisory>,
) {
    let Some(mesh) = mesh else {
        return (SurfaceRollReading::NotCurved, None, None);
    };

    let left_steps =
        curves_along_its_section(mesh, left_region, chain_edge, along, left_inward, cache);
    let right_steps =
        curves_along_its_section(mesh, right_region, chain_edge, along, right_inward, cache);
    let mut left_curved = left_steps.is_some();
    let mut right_curved = right_steps.is_some();

    // A section square to the chain edge can leave the edge's own facet without leaving its PLANE:
    // a side the ball stands clear of is planned as the plane its own facet is.
    if left_curved
        && round_profile
        && super::surface_wall_stands_clear::of_the_plain_ball(
            grid_of(left_region).as_deref(),
            left_steps.as_deref().unwrap_or(&[]),
            along,
            chain_edge.from(),
            right_normal,
            size,
            convex,
        )
    {
        left_curved = false;
    }

    if right_curved
        && round_profile
        && super::surface_wall_stands_clear::of_the_plain_ball(
            grid_of(right_region).as_deref(),
            right_steps.as_deref().unwrap_or(&[]),
            along,
            chain_edge.from(),
            left_normal,
            size,
            convex,
        )
    {
        right_curved = false;
    }

    if !left_curved && !right_curved {
        return (SurfaceRollReading::NotCurved, None, None);
    }

    if left_curved && right_curved {
        let refusal = BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces,
            vec![((chain_edge.from() + chain_edge.to()) / 2.0).into()],
        );
        return (SurfaceRollReading::Refused, None, Some(refusal));
    }

    let side = SurfaceRollAcrossTheWallSides::new(
        *chain_edge,
        along,
        left_curved,
        if left_curved { left_steps } else { right_steps }.unwrap_or_default(),
        if left_curved {
            left_region.clone()
        } else {
            right_region.clone()
        },
        if left_curved {
            left_inward
        } else {
            right_inward
        },
        if left_curved {
            right_normal
        } else {
            left_normal
        },
        if left_curved {
            right_inward
        } else {
            left_inward
        },
    );

    match try_roll(mesh, &side, size, round_profile, convex, cache) {
        Ok(rolled) => (SurfaceRollReading::Rolled, Some(rolled), None),
        Err(refusal) => (SurfaceRollReading::Refused, None, Some(refusal)),
    }
}
