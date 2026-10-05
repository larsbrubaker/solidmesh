//! How much ROOM the surface construction has, and the largest size that fits in it
//! (`Bevel/Surface/SurfaceRoom.cs`, a static class): the 3D reading of the 2D bevel's relaxation
//! (`PathBevel.RelaxRadii`), where a corner's room is the run of path beside it and the corners
//! sharing that run negotiate it between them. Only its nested types land here - what kind of
//! footprint a cut is ([`SurfaceRoomFootprint`]) and one side of one chain edge
//! ([`SurfaceRoomCut`], with its constructor); the measuring and the pair rule land with later
//! packages.
//!
//! Room is a distance ACROSS a face region: from a chain edge, in the region's own plane, along the
//! inward direction the bevel sets back along, to the first piece of that region's rim standing in
//! front of the edge. Two cuts landing on ONE region from opposite sides share the room between
//! them: a cut's setback is `size * k` with `k` fixed by its wedge, so "each fits its share of the
//! gap" and "both setbacks fit in the gap" are the same inequality, `size * (k_a + k_b) <= gap`,
//! and the answer is the smallest of the linear constraints rather than a fixed point. And the
//! answer is a refusal, not a shrink, in the same words and advisory codes the boolean construction
//! uses.

use vector_math::Vector3;

/// C# `SurfaceRoom.Footprint`: what a cut IS, which is what the pair rule has to know about it - a
/// band laid along an edge, or one of the footprints that stand at a CORNER instead.
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

/// C# `SurfaceRoom.Cut` (a readonly struct): one side of one chain edge - the band it wants to take
/// off one region, and how much of that region there is to take.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceRoomCut {
    /// Which region this cut lands on (`SurfaceBevelPlan.KeyOf`).
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
    /// (`SurfaceRoomMitre.OverhangPerSetback`), and zero where the mitre cuts the band back instead
    /// or where the chain simply ends there. The edge's span is where the band is MEASURED from;
    /// where its material really stands is that span widened by these two.
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
    /// order `SurfaceRoomMeasure.Measure` walks them in - or -1 where it is not a band.
    pub side: i32,
    /// Which MIXED CORNER of the plan's chain laid this skirt (`SurfaceBevelPlan.MixedCornerKeys`:
    /// the index of the edge the chain arrives on, or a junction sentinel), or C#'s `NoCorner`
    /// (`i32::MIN`) where this cut is not a run-out's skirt.
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

    /// C#'s constructor with its optional arguments left at their defaults: a [`SurfaceRoomFootprint::Band`]
    /// on no plan, edge or side, with no overhang, no corner, not the wall bite, and no section
    /// sides. Callers that name an optional argument set that field with struct-update syntax
    /// (`SurfaceRoomCut { edge_index, ..SurfaceRoomCut::new(..) }`), which names it as the C# call
    /// sites do.
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

/// The one mesh edge every cut to `blame` was taken off, or `BevelAdvisory::NO_MESH_EDGE` where
/// they name two, or one names none.
pub fn the_edge_that_answered(
    plans: &[std::rc::Rc<super::SurfaceBevelPlan>],
    blame: &[SurfaceRoomCut],
) -> i32 {
    use crate::bevel::BevelAdvisory;

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
/// corner reaches toward the band per unit of size - `(gap, reaches at the band)` - where the corner
/// stands in front of the band within its material along the edge at the size the two would meet
/// at. `None` for a band that is itself a corner footprint, a band against its own plan's end
/// notch, or a corner off to the side, behind, or reaching nothing toward the band.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
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
