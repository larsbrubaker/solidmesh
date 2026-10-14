//! How much ROOM the surface construction has, and the largest size that fits in it: the 3D reading
//! of the 2D bevel's relaxation (`path_bevel::relax_radii`), where a corner's room is the run of
//! path beside it and the corners sharing that run negotiate it between them. Its nested types -
//! what kind of footprint a cut is ([`SurfaceRoomFootprint`]) and one side of one chain edge
//! ([`SurfaceRoomCut`]) - the fit of a whole pass ([`try_fit`]) and the pair rule
//! ([`largest_fitting`]).
//!
//! Room is a distance ACROSS a face region: from a chain edge, in the region's own plane, along the
//! inward direction the bevel sets back along, to the first piece of that region's rim standing in
//! front of the edge. Two cuts landing on ONE region from opposite sides share the room between
//! them: a cut's setback is `size * k` with `k` fixed by its wedge, so "each fits its share of the
//! gap" and "both setbacks fit in the gap" are the same inequality, `size * (k_a + k_b) <= gap`,
//! and the answer is the smallest of the linear constraints rather than a fixed point. And the
//! answer is a refusal, not a shrink, in the same words and advisory codes the boolean construction
//! uses.

use std::rc::Rc;

use polygon_mesh::CancelToken;
use vector_math::Vector3;

use super::{surface_room_fixed_point, surface_room_measure, SurfaceRoomCrossing};
use super::{SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan};
use crate::bevel::level_line::WorkError;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// What a cut IS, which is what the pair rule has to know about it - a band laid along an edge, or
/// one of the footprints that stand at a CORNER instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceRoomFootprint {
    /// The band one side of one chain edge takes off one region.
    #[default]
    Band = 0,

    /// The skirt a mixed corner's RUN-OUT lays on the face past the end of the bead's rim
    /// (`SurfaceRoomRunOut`). It is laid ALONG a direction as a band is, and read as one everywhere
    /// the pair rule asks how two cuts face each other - but it belongs to no EDGE, which is what
    /// tells it from a band: a chain's own notch stands between its BANDS by construction and is
    /// exempt from them, and it stands nowhere in particular with respect to its chain's skirts.
    RunOutSkirt,

    /// The section a chain's END takes out of the face beyond its corner (`SurfaceRoomEndCut`). It
    /// reaches the same distance whichever way the face runs, so it carries no
    /// [`SurfaceRoomCut::inward`], and it is never negotiated against its OWN plan's bands - the
    /// two tangent points it is bounded by STAND on them.
    EndNotch,

    /// The tip of the wedge two bands mitre into at a REFLEX corner of one region
    /// (`SurfaceRoomMitre`). It reaches one way only - along [`SurfaceRoomCut::inward`], out of the
    /// corner and into the face - and it belongs to no band, so its own plan's cuts are negotiated
    /// against it like anyone else's.
    MitreTip,
}

/// One side of one chain edge - the band it wants to take off one region, and how much of that
/// region there is to take.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceRoomCut {
    /// Which region this cut lands on (`SurfaceBevelPlan::key_of`).
    pub region_key: i32,
    /// Where the chain edge starts, which is the origin of the frame room is measured in.
    pub from: Vector3,
    /// Where the chain edge ends.
    pub to: Vector3,
    /// The unit direction along the chain edge.
    pub along: Vector3,
    /// The unit direction into the region, square to the edge and in the region's plane.
    pub inward: Vector3,
    /// The length of the chain edge.
    pub length: f64,
    /// How far this cut sets back per unit of SIZE - `1 / tan(wedge / 2)` for a round and 1 for a
    /// chamfer. The setback is linear in the size, which is what makes every constraint here a
    /// division rather than a search.
    pub setback_per_size: f64,
    /// The distance across the region along [`SurfaceRoomCut::inward`] to its far rim, or infinity
    /// where no rim edge stands in front of this one.
    pub room: f64,
    /// What kind of footprint this is, which is what the pair rule reads it as.
    pub shape: SurfaceRoomFootprint,
    /// Which plan of the pass this cut belongs to, or -1 where it belongs to none. Only the corner
    /// rule asks: an end's notch sits BETWEEN its own plan's bands by construction, so the two are
    /// not negotiated against each other.
    pub plan_index: i32,
    /// How far past [`SurfaceRoomCut::from`] this band's own material reaches, per unit of SIZE:
    /// the overhang the mitre at that end of the edge carries it by
    /// (`surface_room_mitre::overhang_per_setback`), and zero where the mitre cuts the band back
    /// instead or where the chain simply ends there. The edge's span is where the band is MEASURED
    /// from; where its material really stands is that span widened by these two.
    pub overhang_before_from: f64,
    /// The same past [`SurfaceRoomCut::to`].
    pub overhang_past_to: f64,
    /// Which edge of the plan's chain this band was taken off, or -1 where it was taken off no edge
    /// at all. With [`SurfaceRoomCut::side`] and the plan index that NAMES a band, which is what a
    /// crossing has to say to be sewn: the builder holds its strips by plan and edge, and a
    /// crossing that could only be recognised by its geometry would have to be matched back to one
    /// by measuring.
    pub edge_index: i32,
    /// Which side of that edge this band takes - 0 for the left region and 1 for the right, the
    /// order `surface_room_measure::measure` walks them in - or -1 where it is not a band.
    pub side: i32,
    /// Which MIXED CORNER of the plan's chain laid this skirt
    /// (`SurfaceBevelPlan::mixed_corner_keys`: the index of the edge the chain arrives on, or a
    /// junction sentinel), or `NO_CORNER` (`i32::MIN`) where this cut is not a run-out's skirt.
    pub corner_key: i32,
    /// Whether this footprint is the bite a run-out takes out of the WALL it turns against rather
    /// than the skirt it lays on the cap it runs across (`SurfaceRoomRunOut`). One mixed corner
    /// lays both, and they are the same KIND of footprint to the pair rule - but a band crossed on
    /// one of them is sewn along a curve of quite another shape than a band crossed on the other,
    /// so the crossing has to say which it came off.
    pub the_wall_bite: bool,
    /// One SIDE of the section an end notch takes, relative to its corner and per unit of size
    /// (`SurfaceRoomNotchSides`), or `None` where the two could not be named and the notch's reach
    /// has to be read as spreading every way. A notch's reach is DIRECTIONAL: the end plane is
    /// rarely square to the chain, so the section leans to one side of the corner, and the two
    /// sides are what bound it.
    pub section_side_a: Option<Vector3>,
    /// The other side of that section; see [`SurfaceRoomCut::section_side_a`].
    pub section_side_b: Option<Vector3>,
}

impl SurfaceRoomCut {
    /// What `corner_key` reads as where no mixed corner made this cut. Not -1, because a JUNCTION
    /// at the start of a chain is keyed -1.
    pub const NO_CORNER: i32 = i32::MIN;

    /// A [`SurfaceRoomFootprint::Band`] on no plan, edge or side, with no overhang, no corner, not
    /// the wall bite, and no section sides. Callers that need another value set that field with
    /// struct-update syntax (`SurfaceRoomCut { edge_index, ..SurfaceRoomCut::new(..) }`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        region_key: i32,
        from: Vector3,
        to: Vector3,
        along: Vector3,
        inward: Vector3,
        length: f64,
        setback_per_size: f64,
        room: f64,
    ) -> Self {
        SurfaceRoomCut {
            region_key,
            from,
            to,
            along,
            inward,
            length,
            setback_per_size,
            room,
            shape: SurfaceRoomFootprint::Band,
            plan_index: -1,
            overhang_before_from: 0.0,
            overhang_past_to: 0.0,
            edge_index: -1,
            side: -1,
            corner_key: Self::NO_CORNER,
            the_wall_bite: false,
            section_side_a: None,
            section_side_b: None,
        }
    }

    /// The same cut, told which plan of the pass it belongs to.
    pub fn on_plan(&self, plan_index: i32) -> SurfaceRoomCut {
        SurfaceRoomCut {
            plan_index,
            ..*self
        }
    }
}

/// How near a gap has to come to zero, relative to the band's length (at least 1), to be none.
pub const RELATIVE_TOLERANCE: f64 = 1e-7;

impl SurfaceRoomCut {
    /// Whether this footprint is seated at a CORNER rather than laid along an edge - a chain end's
    /// notch or a reflex corner's mitre tip.
    pub fn corner_footprint(&self) -> bool {
        self.shape == SurfaceRoomFootprint::EndNotch || self.shape == SurfaceRoomFootprint::MitreTip
    }

    /// The middle of the chain edge, which is where a refusal is reported at.
    pub fn midpoint(&self) -> Vector3 {
        (self.from + self.to) / 2.0
    }
}

/// How near the request the largest fitting size has to come to count as the request.
pub const FIT_TOLERANCE: f64 = 1e-9;

/// What [`try_fit`] reads.
#[derive(Debug, Clone)]
pub struct SurfaceRoomFit {
    /// Whether every plan fits at the requested size.
    pub fits: bool,
    /// The request where it fits, and the size that would have fitted where it does not (the
    /// request again where a reading refused by NAME).
    pub largest_fitting_size: f64,
    /// `None` where the request fits, and otherwise the advisory naming the size that does.
    pub refusal: Option<BevelAdvisory>,
    /// Every skirt over a band the build has to SEW rather than refuse. Meaningful only where
    /// [`SurfaceRoomFit::fits`].
    pub crossings: Vec<SurfaceRoomCrossing>,
}

/// The largest size every one of `plans` (drawn at `requested_size`) fits at, and - where that is
/// smaller than the request - the refusal to say so with, the size named settled by
/// [`surface_room_fixed_point::settled`]. `stretches` (when given) collects every band that runs
/// off PART of its own span. A cancel passes up as [`WorkError::Cancelled`].
pub fn try_fit(
    plans: &[Rc<SurfaceBevelPlan>],
    requested_size: f64,
    cancellation_token: &CancelToken,
    stretches: Option<&mut Vec<SurfaceBandRunsOffOverAStretch>>,
) -> Result<SurfaceRoomFit, WorkError> {
    let mut sewn = Vec::new();
    let mut named = Vec::new();

    let cuts = surface_room_measure::measure(plans, Some(&mut named), stretches);

    if let Some(first) = named.into_iter().next() {
        // A reading that refuses by NAME: no size fits such a shape, so the request is handed back
        // unchanged.
        return Ok(SurfaceRoomFit {
            fits: false,
            largest_fitting_size: requested_size,
            refusal: Some(first),
            crossings: sewn,
        });
    }

    // Whose answer it was, so the refusal is written against the guilty cut's group.
    let mut blame = Vec::new();

    let (mut largest_fitting_size, worst_at, worst_is_a_collision) = largest_fitting(
        &cuts,
        requested_size,
        cancellation_token,
        requested_size,
        Some(&mut sewn),
        Some(&mut blame),
    )?;

    if largest_fitting_size >= requested_size - (requested_size.abs() * FIT_TOLERANCE) {
        return Ok(SurfaceRoomFit {
            fits: true,
            largest_fitting_size: requested_size,
            refusal: None,
            crossings: sewn,
        });
    }

    // The size named has to be a size the user can then ASK for: re-measured until it settles.
    largest_fitting_size =
        surface_room_fixed_point::settled(plans, largest_fitting_size, cancellation_token)?;

    let refusal = BevelAdvisory::new(
        if worst_is_a_collision {
            BevelAdvisoryCode::SizeCollidesAcrossFace
        } else {
            BevelAdvisoryCode::SizeExceedsFaceWidth
        },
        vec![
            requested_size.into(),
            worst_at.into(),
            largest_fitting_size.into(),
        ],
    )
    .on_mesh_edge(the_edge_that_answered(plans, &blame));

    Ok(SurfaceRoomFit {
        fits: false,
        largest_fitting_size,
        refusal: Some(refusal),
        crossings: sewn,
    })
}

/// The one mesh edge every cut to `blame` was taken off, or `BevelAdvisory::NO_MESH_EDGE` where
/// they name two, or one names none.
pub fn the_edge_that_answered(plans: &[Rc<SurfaceBevelPlan>], blame: &[SurfaceRoomCut]) -> i32 {
    let mut named = BevelAdvisory::NO_MESH_EDGE;

    for cut in blame {
        if cut.plan_index < 0
            || cut.plan_index as usize >= plans.len()
            || cut.edge_index < 0
            || cut.edge_index as usize >= plans[cut.plan_index as usize].edges.len()
        {
            return BevelAdvisory::NO_MESH_EDGE;
        }

        let edge = plans[cut.plan_index as usize].edges[cut.edge_index as usize]
            .edge
            .edge
            .mesh_edge_index;

        if named != BevelAdvisory::NO_MESH_EDGE && named != edge {
            return BevelAdvisory::NO_MESH_EDGE;
        }

        named = edge;
    }

    named
}

/// The gap across a band's region to a CORNER footprint standing in front of it, and how far the
/// corner reaches toward the band per unit of size - `(gap, reaches at the band)` - where the
/// corner stands in front of the band within its material along the edge at the size the two would
/// meet at. `None` for a band that is itself a corner footprint, a band against its own plan's end
/// notch, or a corner off to the side, behind, or reaching nothing toward the band.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: it refuses a NaN as well.
pub fn try_gap_to_the_corner(band: &SurfaceRoomCut, corner: &SurfaceRoomCut) -> Option<(f64, f64)> {
    use vector_math::cs_math::{max_f64, min_f64};

    if band.corner_footprint()
        || (corner.shape == SurfaceRoomFootprint::EndNotch
            && band.shape == SurfaceRoomFootprint::Band
            && band.plan_index >= 0
            && band.plan_index == corner.plan_index)
        || !(band.length > 0.0)
    {
        return None;
    }

    let tolerance = max_f64(band.length, 1.0) * RELATIVE_TOLERANCE;
    let away = corner.from - band.from;
    let offset = away.dot(band.inward);

    if !(offset > tolerance) {
        return None;
    }

    let out_of_plane = away.dot(band.along.cross(band.inward)).abs();

    if out_of_plane > offset {
        return None;
    }

    let reaches_at_the_band = match (corner.section_side_a, corner.section_side_b) {
        (Some(a), Some(b)) => max_f64(0.0, max_f64(-a.dot(band.inward), -b.dot(band.inward))),
        _ => {
            if corner.inward.length_squared() > 0.0 {
                corner.setback_per_size * max_f64(0.0, -corner.inward.dot(band.inward))
            } else {
                corner.setback_per_size
            }
        }
    };

    if !(reaches_at_the_band > 0.0) {
        return None;
    }

    let at_the_size = offset / (band.setback_per_size + reaches_at_the_band);
    let reach = corner.setback_per_size * at_the_size;
    let along = away.dot(band.along);
    let (to_the_tip, from_the_tip) = if corner.inward.length_squared() > 0.0 {
        (along + (reach * corner.inward.dot(band.along)), along)
    } else {
        (along + reach, along - reach)
    };

    if max_f64(along, max_f64(to_the_tip, from_the_tip))
        < -(band.overhang_before_from * at_the_size)
        || min_f64(along, min_f64(to_the_tip, from_the_tip))
            > band.length + (band.overhang_past_to * at_the_size)
    {
        return None;
    }

    Some((offset, reaches_at_the_band))
}

/// How nearly two cuts' inward directions must oppose for the pair to face each other across one
/// region.
pub const OPPOSING_COSINE: f64 = -0.999;

/// What [`largest_fitting`] reads: the largest size every cut and every facing pair has room for,
/// where the worst limit stands, and whether it is a collision between two cuts.
pub type SurfaceRoomLargestFitting = (f64, Vector3, bool);

/// The largest size from `starting_from` down that every cut's own room and every pair of cuts
/// facing each other across one region allows - a skirt the pass sews across a band (gathered at
/// `measured_at_size`, else `starting_from`, into `crossings` where given) is no collision.
/// `blame`, where given, is left holding the cut (or pair) that set the limit.
pub fn largest_fitting(
    cuts: &[SurfaceRoomCut],
    starting_from: f64,
    cancellation_token: &CancelToken,
    measured_at_size: f64,
    crossings: Option<&mut Vec<SurfaceRoomCrossing>>,
    mut blame: Option<&mut Vec<SurfaceRoomCut>>,
) -> Result<SurfaceRoomLargestFitting, WorkError> {
    // Every reach below is a RATE; the crossing test is the one reading that needs the size.
    let at_size = if measured_at_size.is_infinite() {
        starting_from
    } else {
        measured_at_size
    };

    let mut largest_fitting_size = starting_from;
    let mut worst_at = Vector3::ZERO;
    let mut worst_is_a_collision = false;

    let blame_only = |blame: &mut Option<&mut Vec<SurfaceRoomCut>>, these: &[SurfaceRoomCut]| {
        if let Some(blame) = blame.as_deref_mut() {
            blame.clear();
            blame.extend_from_slice(these);
        }
    };

    for cut in cuts {
        if cancellation_token.is_cancelled() {
            return Err(WorkError::Cancelled);
        }

        if cut.room.is_infinite() {
            continue;
        }

        let allowed = cut.room / cut.setback_per_size;
        if allowed < largest_fitting_size {
            largest_fitting_size = allowed;
            worst_at = cut.midpoint();
            worst_is_a_collision = false;
            blame_only(&mut blame, &[*cut]);
        }
    }

    // Every skirt over a band this pass SEWS rather than refuses, read before the negotiation.
    let sewn = SurfaceRoomCrossing::gather(Some(cuts), at_size, cancellation_token)?;
    if let Some(crossings) = crossings {
        crossings.extend(sewn.iter().cloned());
    }

    // The negotiation: two cuts facing each other across one region are solved together.
    for i in 0..cuts.len() {
        for j in i + 1..cuts.len() {
            if cancellation_token.is_cancelled() {
                return Err(WorkError::Cancelled);
            }

            let here = &cuts[i];
            let other = &cuts[j];

            if here.region_key != other.region_key {
                continue;
            }

            // A run-out's skirt over another chain's band is sewn, not a collision.
            if SurfaceRoomCrossing::it_is_sewn(Some(&sewn), here, other) {
                continue;
            }

            if here.corner_footprint() || other.corner_footprint() {
                let (corner, band) = if here.corner_footprint() {
                    (here, other)
                } else {
                    (other, here)
                };

                let Some((gap, reaches_at_the_band)) = try_gap_to_the_corner(band, corner) else {
                    continue;
                };

                // A footprint that reaches only ONE way is worth what it reaches towards this band.
                let allowed_here = gap / (band.setback_per_size + reaches_at_the_band);
                if allowed_here < largest_fitting_size {
                    largest_fitting_size = allowed_here;
                    worst_at = corner.from;
                    worst_is_a_collision = true;
                    blame_only(&mut blame, &[*band, *corner]);
                }

                continue;
            }

            if here.inward.dot(other.inward) > OPPOSING_COSINE {
                continue;
            }

            let gap = surface_room_measure::offset_to_segment(
                here.from,
                here.along,
                here.length,
                here.inward,
                other.from,
                other.to,
            );
            let at = here.midpoint();

            if gap.is_infinite() {
                continue;
            }

            let allowed = gap / (here.setback_per_size + other.setback_per_size);
            if allowed < largest_fitting_size {
                largest_fitting_size = allowed;
                worst_at = at;
                worst_is_a_collision = true;
                blame_only(&mut blame, &[*here, *other]);
            }
        }
    }

    Ok((largest_fitting_size, worst_at, worst_is_a_collision))
}
