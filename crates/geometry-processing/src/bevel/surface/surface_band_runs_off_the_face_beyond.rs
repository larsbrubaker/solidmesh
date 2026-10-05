//! The one reading of "this band takes the WHOLE of the face it lands on": every piece of the
//! region's rim standing in front of the chain edge, over the edge's own span, lies nearer than the
//! setback, and what is beyond that rim is one plane. The band then has no landing left to walk
//! back to - it gives up the region entire, with no seam on it at all - and the strip's tangent
//! rail stands in the air over the whole edge, to be trimmed away against the plane beyond. Ports
//! `Bevel/Surface/SurfaceBandRunsOffTheFaceBeyond.cs` (C# `internal static class`; a module of free
//! functions here).
//!
//! Shared by the two halves of the same fact, as [`surface_room_standing_feature`] is. The room
//! reading must not take such a rim for a WALL: a wall is a place the setback runs out of face, and
//! this is a place the face runs out and the bevel carries on over the edge of it onto the next
//! face - which is a size that fits, sewn along the trim, rather than a size that is refused. The
//! build then trims the strip there (`SurfaceStripTrimmedByTheFaceBeyond`), which is what makes
//! the room reading honest rather than optimistic.
//!
//! **Only when the band takes ALL of it.** A tangent line that still crosses the region inside the
//! edge's span leaves a seam to sew to, and the rim beyond that seam is a wall as it always was -
//! the nose fixture's own top face at every size under 14.142 is that case. So the reading is of
//! the SETBACK against the rim, not of the rim alone, and it turns over exactly where the last of
//! the face goes.
//!
//! **One plane beyond, or a name - and the name says WHICH.** The trim is a per-rail crossing with
//! a plane, closed form, and the section the face beyond gives up is bounded by the curve that
//! crossing draws. Two things are not that, and they are told apart by the flood the surface beyond
//! is grown by (`the_surface_beyond_is_one_plane`): a flood that reaches facets out of the plane is
//! a CURVED face, because the flood only ever crosses joins under
//! [`FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES`] and so everything it reached is
//! tangent-continuous with what it came from (`SurfaceStripRunsOffACurvedFace`); a flood that IS
//! one plane while another rim of the same run-off has a different plane past it is two FLATS
//! meeting at a corner the band would cross (`SurfaceStripRunsOffAtACorner`). The two want
//! different constructions - a trim along a curve, and a trim that turns a corner - so they are
//! refused with different sentences rather than one that guesses.
//!
//! C#'s `bool Try...(..., out failure)` shape is `Result<Option<_>, BevelAdvisory>` here: `Err` is
//! the refusal by name, `Ok(None)` is "not this reading" (C# false with a null failure).

use std::cmp::Ordering;
use std::collections::HashSet;
use std::sync::Arc;

use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::surface_room_standing_feature;
use super::SurfaceBandRunsOffOverAStretch;
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use crate::topology::{EdgeConvexity, FaceGroup, FeatureEdge};

/// How near two readings of one position are the same position, per unit of edge length.
const VERTEX_TOLERANCE: f64 = 1e-5;

/// How many rulers long a stretch of run-off has to be before it is one ([`try_read_a_stretch`]).
/// Ten, because the window's two ends are plane crossings and each carries its own reading's
/// error; anything shorter is a rim the band REACHES rather than passes.
const RULERS_TO_BE_A_STRETCH: f64 = 10.0;

/// Where a whole-span run-off goes: the plane beyond and which end the trim cuts back from - the
/// `out` values of C# `TryRead`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TheFaceBeyond {
    /// A point on the plane beyond.
    pub plane_point: Vector3,

    /// That plane's outward normal, which is the far face's own.
    pub plane_normal: Vector3,

    /// Which end of the strip the trim cuts back from - the end the tangent line is furthest out
    /// at. It is the chain's WALK that decides this and nothing about the shape, so both answers
    /// are ordinary: the nose fixture's own top edge reads one way as a kernel mesh and the other
    /// way in the part the same shape was saved as.
    pub trims_the_start: bool,
}

/// The chain edge and the band read against a region: C#'s shared `from, along, length, inward,
/// setback` parameters, gathered so the three readings take them the same way.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BandSpan {
    /// The chain edge's start.
    pub from: Vector3,
    /// Its unit direction.
    pub along: Vector3,
    /// Its length, which is the span the band is read over.
    pub length: f64,
    /// The in-face direction the setback is measured along - the plan's own.
    pub inward: Vector3,
    /// How far back from the edge the band reaches on this side.
    pub setback: f64,
}

impl BandSpan {
    /// The point `distance` along the chain edge.
    fn at(&self, distance: f64) -> Vector3 {
        self.from + (self.along * distance)
    }

    /// C#'s guard `!(length > 0) || !(setback > 0)`, NaN reading as no span.
    // C#'s `!(x > 0)` is true for NaN too, which the rewritten comparison would not be.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    fn is_empty(&self) -> bool {
        !(self.length > 0.0) || !(self.setback > 0.0)
    }

    /// How near two readings of one position are, at this edge's scale.
    fn near(&self) -> f64 {
        cs_max(self.length, 1.0) * VERTEX_TOLERANCE
    }
}

/// Whether the band of one chain edge on one region takes the whole of that region over its own
/// span, and if it does, the plane beyond the rim it runs off.
///
/// `mesh` is the mesh the region was read on, for the SURFACE beyond the rim it runs off;
/// `region` the region the band lands on; `beveled_mesh_edges` the mesh edges this pass is
/// beveling, which bound nothing here; `chain_vertices` their end vertices, for the
/// standing-feature reading. `Err` where the band runs off onto something that is NOT one plane,
/// which is refused by name; `Ok(None)` otherwise, including when the band simply lands on the
/// region as usual.
pub fn try_read(
    mesh: Option<&Mesh>,
    region: Option<&FaceGroup>,
    span: &BandSpan,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
) -> Result<Option<TheFaceBeyond>, BevelAdvisory> {
    let Some(region) = region else {
        return Ok(None);
    };

    if span.is_empty() {
        return Ok(None);
    }

    let near = span.near();
    let mut found = false;
    let mut more_than_one_plane = false;
    let mut seed_beyond = -1;
    let mut plane_point = Vector3::ZERO;
    let mut plane_normal = Vector3::ZERO;

    for rim in region.boundary_edges() {
        if beveled_mesh_edges.contains(&rim.mesh_edge_index) {
            // The chain edge itself and its neighbours in the same pass: the surface runs on
            // through those, so they are no part of how far the face reaches.
            continue;
        }

        if surface_room_standing_feature::stands_on(region, rim, beveled_mesh_edges, chain_vertices)
        {
            // Something HANGING over the region, which the band passes under and the region goes
            // on behind: not the far side of the face, so not the rim anything runs off.
            continue;
        }

        let Some(in_front) = try_in_front_over_the_span(rim, span, near) else {
            continue;
        };

        if in_front.farthest >= span.setback - near {
            // The face reaches at least as far back as the band does somewhere along the span, so
            // there is still a landing here and a seam to sew to. An ordinary band, and the rim is
            // the wall the room reading has always taken it for.
            return Ok(None);
        }

        if rim.convexity != EdgeConvexity::Convex {
            // A CONCAVE rim nearer than the setback is material the band would eat into rather
            // than an edge of the solid it runs off. Refused elsewhere, by the readings that own
            // it; here it simply is not a run-off.
            return Ok(None);
        }

        let (Some((_, beyond)), Some(face_beyond)) = (
            surface_room_standing_feature::try_the_two_sides(region, rim),
            surface_room_standing_feature::try_the_face_beyond(region, rim),
        ) else {
            return Ok(None);
        };

        if !found {
            plane_normal = beyond;
            plane_point = rim.start;
            seed_beyond = face_beyond;
            found = true;
            continue;
        }

        if (beyond - plane_normal).length() > near
            || (rim.start - plane_point).dot(plane_normal).abs() > near
        {
            // More than one face beyond the rim the band runs off - two flats meeting at a corner
            // the band would cross, or one more facet of a curve the flood below will name as one.
            // NOTED rather than refused here: whether it is worth a name depends on the tangent
            // line's own reading below, and a band running off parallel is not this construction
            // at all whatever is beyond it.
            more_than_one_plane = true;
        }
    }

    if !found {
        return Ok(None);
    }

    // And the tangent line has to stand OUTSIDE that plane over the whole span, obliquely.
    //
    // Outside the whole way, because a line that crosses back into the part inside the span still
    // lands on the region there and has a seam to sew to - the rim reading above says the face is
    // gone, and this says the tangent line agrees.
    //
    // And OBLIQUELY, because taking the whole of a region is not on its own the shape this trims:
    // a band 8 deep on a wall 5 thick takes all of it too, and its tangent line runs PARALLEL to
    // the face beyond - three millimetres behind it the whole way, running off that face as well
    // and out of the part altogether. A line that crosses the plane just outside the span is cut
    // back rail by rail along it, and every rail still has a window to sew through.
    //
    // Which END the rails are cut back from is the crossing's own side: a chain walked the other
    // way round the same corner puts it at the other one, and nothing here may read the direction
    // the edge happens to have been linked in as part of the shape.
    let foot = span.from + (span.inward * span.setback);
    let slope = plane_normal.dot(span.along);

    // C#'s `!(Math.Abs(slope) > 1e-9)`, NaN reading as parallel.
    if slope.abs().partial_cmp(&1e-9) != Some(Ordering::Greater) {
        return Ok(None);
    }

    let crossing = plane_normal.dot(plane_point - foot) / slope;

    if (foot - plane_point).dot(plane_normal).partial_cmp(&0.0) != Some(Ordering::Greater)
        || (crossing > near && crossing < span.length - near)
    {
        return Ok(None);
    }

    let trims_the_start = crossing >= span.length - near;

    // Everything above says this IS the oblique run-off, and only now is what lies beyond worth a
    // name. It has to be one PLANE - not one facet of one, and not two flats meeting at a corner
    // the band would cross. Read as a SURFACE, at the tolerance a surface is read at everywhere
    // else: a wall drawn as a forty-sided prism gives every rim edge its own facet, and a strip
    // trimmed against the first of them would be cut along a line where the wall really turns.
    // Named HERE, in the reading the room makes, because a room falling through to its wall rule
    // would answer with a size for a shape this construction does not trim against at any size.
    //
    // And named for WHAT IT IS, in two sentences rather than one: the CURVE first, because the
    // flood is what proves it, and the CORNER where that flood says the surface beyond is flat and
    // some other rim of the same run-off has a different plane past it anyway.
    let middle = span.at(span.length / 2.0);

    if !the_surface_beyond_is_one_plane(mesh, seed_beyond, plane_point, plane_normal, near) {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceStripRunsOffACurvedFace,
            middle,
        ));
    }

    if more_than_one_plane {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceStripRunsOffAtACorner,
            middle,
        ));
    }

    Ok(Some(TheFaceBeyond {
        plane_point,
        plane_normal,
        trims_the_start,
    }))
}

/// The THIRD reading, in the crack between the other two: the band reaches exactly as far as the
/// face does, so the face is CONSUMED TO ITS RIM - given up whole, with the rim's own vertices
/// standing where the tangent line would have been cut. `near_enough` is the pool's own
/// [`SurfaceVertexPool::near_enough`](super::SurfaceVertexPool::near_enough).
///
/// Neither reading either side of the equality builds what a kernel builds there. Read as a
/// LANDING (`farthest >= setback - near` in [`try_read`]), the walkback cuts the face along a line
/// coincident with its rim, keeps nothing of it, and leaves the seam pick with no run over the edge
/// at all - `SurfaceStripSeamDidNotMeet` in the middle of the edge, where the face gave up over the
/// whole of it. Read as a RUN-OFF it is degenerate the other way: rail 0 lies IN the plane beyond,
/// so the per-rail crossing `SurfaceStripTrimmedByTheFaceBeyond` makes has nothing to cross. The
/// honest reading is that the rim IS the seam, which costs the build no new vertex at all - the rim
/// is in the pool already, as the part's own.
///
/// **The ruler is the pool's, not this file's.** Every other reading here measures with
/// `VERTEX_TOLERANCE` against the edge's length, but what settles this one is whether the build
/// could tell the seam from the rim if it tried - the very question `SurfaceStripEnds.MeetsTheSeam`
/// asks, and its answer is the pool's `near_enough`. A wall that keeps less than that of itself
/// leaves a sliver the run reader would sweep the rim into anyway (`SurfaceWalkbackBands.RunsAlong`
/// reads vertices within the same nearness OF THE LINE), so the two readings would disagree about
/// the same two points. On a 20 x 20 x 2 plate that ruler is about 2.8e-4, which is the width of
/// the crack.
pub fn consumes_the_face_to_its_rim(
    region: Option<&FaceGroup>,
    span: &BandSpan,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
    near_enough: f64,
) -> bool {
    let Some(region) = region else {
        return false;
    };

    if span.is_empty() {
        return false;
    }

    let near = span.near();
    let mut farthest_of_all = f64::NEG_INFINITY;

    for rim in region.boundary_edges() {
        if beveled_mesh_edges.contains(&rim.mesh_edge_index) {
            // The chain edge itself and its neighbours in the same pass: the surface runs on
            // through those, so they are no part of how far the face reaches.
            continue;
        }

        if surface_room_standing_feature::stands_on(region, rim, beveled_mesh_edges, chain_vertices)
        {
            // Something HANGING over the region, which the band passes under and the region goes
            // on behind: not the far side of the face, so no measure of how far it reaches.
            continue;
        }

        let Some(in_front) = try_in_front_over_the_span(rim, span, near) else {
            continue;
        };

        farthest_of_all = cs_max(farthest_of_all, in_front.farthest);
    }

    // The FARTHEST any rim reaches, because one rim standing at the setback while another stands
    // past it is a face with material left on it - an ordinary landing on the part beyond.
    (farthest_of_all - span.setback).abs() <= near_enough
}

/// What a chain is at the two ends of the edge a stretch is read on (C# `chainConvexity`,
/// `freeAtTheStart`, `freeAtTheEnd`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainAtTheSpan {
    /// Which way the chain edge folds, which is what tells a fillet from a cut.
    pub convexity: EdgeConvexity,
    /// Whether the chain really ENDS at the edge's from - no mitre, no junction, not closed.
    pub free_at_the_start: bool,
    /// The same at the edge's to.
    pub free_at_the_end: bool,
}

/// The other half of the same fact: the band takes the whole of the face over a STRETCH at one end
/// of the chain edge and lands on it as usual past that, which is a run-off over that stretch and
/// no wall at all. Closed form, because the rim is straight and the offset along it is linear.
/// `Ok(Some)` is where it runs off and onto what, for the build to end the strip on; `Err` where
/// the stretch is one this construction refuses by name.
///
/// A wall that DIES at one end of the span is the pyramidal recess's left wall: 0.053 mm of face
/// where the crease reaches the mouth, widening at 1.118 mm per millimetre along the crease, so a
/// 3 mm round sits on the wall over all but the last 1.887 mm of it and runs off the knife edge
/// there onto the cube's own left face. Charged as a wall - which is what the room reading did
/// before this - it names a size of 0.0735 for a part that rounds at 3.
///
/// **Anchored at ONE end, and that end free.** A stretch in the middle of the span leaves face on
/// both sides of it and the strip would have to run off and land again, which is not one trim; and
/// a stretch at an end the chain carries on past has the neighbouring strip's mitre standing where
/// the run-off would be. Both are refused by name rather than read as walls, because a wall reading
/// there answers with a SIZE for a shape this construction does not build at any size.
///
/// **And the chain edge CONCAVE.** A fillet's band lies in the pocket its two faces make and the
/// face beyond the knife edge carries it on; a chamfer's cut takes material off the outside, and a
/// cut running off part of its own landing is a different shape with a different trim. Left to the
/// wall rule rather than named, because there the wall rule's own answer - the size that keeps the
/// band on the face - is a good one.
pub fn try_read_a_stretch(
    mesh: Option<&Mesh>,
    region: Option<&FaceGroup>,
    span: &BandSpan,
    chain: ChainAtTheSpan,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
) -> Result<Option<SurfaceBandRunsOffOverAStretch>, BevelAdvisory> {
    let Some(region) = region else {
        return Ok(None);
    };

    if span.is_empty() {
        return Ok(None);
    }

    if chain.convexity != EdgeConvexity::Concave {
        // A CUT rather than a fillet, and no run-off of this reading's: a fillet's band lies in the
        // pocket its two faces make and the face beyond the knife edge carries it on, where a cut
        // takes material off the outside and what lies past its landing is a shape of its own.
        //
        // Read FIRST, so that nothing about a cut is named at all. Left to the wall rule rather
        // than refused, and that is the whole of the difference: the rim goes on being charged as
        // the wall it was, so the reading answers with a SIZE that keeps the band on the face -
        // which is an answer, and a name would take it away. Two chamfers of "Every Chamfer
        // Works.mcx" are built at exactly such a size.
        return Ok(None);
    }

    let near = span.near();
    let mut runs_off: Vec<&FeatureEdge> = Vec::new();
    let mut windows: Vec<(f64, f64)> = Vec::new();

    for rim in region.boundary_edges() {
        if beveled_mesh_edges.contains(&rim.mesh_edge_index) {
            // The chain edge itself and its neighbours in the same pass: the surface runs on
            // through those, so they are no part of how far the face reaches.
            continue;
        }

        if surface_room_standing_feature::stands_on(region, rim, beveled_mesh_edges, chain_vertices)
        {
            // Something HANGING over the region, which the band passes under: not the far side of
            // the face, so not a rim anything runs off.
            continue;
        }

        if rim.convexity != EdgeConvexity::Convex {
            // A CONCAVE rim nearer than the setback is material the band would eat into rather
            // than an edge of the solid it runs off, and it goes on being charged as the wall it
            // is. Read HERE, before the stretches are put together, so that such a rim is no part
            // of one.
            continue;
        }

        if shares_a_corner(rim, span.from, near) || shares_a_corner(rim, span.at(span.length), near)
        {
            // A rim MEETING the chain edge at a corner, which is the corner rule's business and
            // not this one: its offset is zero there by construction rather than because the face
            // ran out, so every acute corner there is - the altitude of a triangular face, the
            // recess's own lower crease at the apex - would read as a stretch of run-off at that
            // end of the span. What the corner really takes is how much of the edge's LENGTH the
            // mitre there eats (SurfaceRoomMeasure.MeasureCorner), which is a reading of its own.
            continue;
        }

        let Some(in_front) = try_in_front_over_the_span(rim, span, near) else {
            continue;
        };

        // C#'s `!(until > under + ...)`, NaN reading as too short.
        if in_front
            .under_to
            .partial_cmp(&(in_front.under_from + (near * RULERS_TO_BE_A_STRETCH)))
            != Some(Ordering::Greater)
        {
            // A window too short to be a stretch of anything: a rim that grazes the setback over a
            // rounding's worth of the span is the band REACHING it, and the strip that would be run
            // off there is thinner than the ruler the build's own points are placed with. Ten
            // rulers rather than one, because the window's ends are two plane crossings and each
            // carries its own reading's error.
            continue;
        }

        runs_off.push(rim);
        windows.push((in_front.under_from, in_front.under_to));
    }

    if runs_off.is_empty() {
        return Ok(None);
    }

    // ONE stretch, or none of this reading. A tessellated knife edge hands over a facet at a time
    // and those join into the one stretch they are; two stretches with face standing between them
    // is the band running off in two places, which is the mid-span case under another name.
    //
    // C#'s `List.Sort` is unstable, but no tie can reach an output: windows starting at the same
    // place give the same `lowest`, pass the same gap test and fold into the same `highest`
    // whichever comes first.
    let mut order: Vec<usize> = (0..windows.len()).collect();
    order.sort_by(|&left, &right| cs_compare(windows[left].0, windows[right].0));

    let lowest = windows[order[0]].0;
    let mut highest = windows[order[0]].1;

    for &at in &order[1..] {
        if windows[at].0 > highest + near {
            return Err(refusal(
                BevelAdvisoryCode::SurfaceBandRunsOffMidSpan,
                span.at((lowest + highest) / 2.0),
            ));
        }

        highest = cs_max(highest, windows[at].1);
    }

    let touches_the_start = lowest <= near;
    let touches_the_end = highest >= span.length - near;

    if touches_the_start && touches_the_end {
        // The WHOLE span, which is the reading try_read makes above - tangent line and all - and
        // not this one.
        return Ok(None);
    }

    let middle = span.at((lowest + highest) / 2.0);

    if !touches_the_start && !touches_the_end {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceBandRunsOffMidSpan,
            middle,
        ));
    }

    if !(if touches_the_start {
        chain.free_at_the_start
    } else {
        chain.free_at_the_end
    }) {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceRunOffMeetsAMitredEnd,
            middle,
        ));
    }

    // Which rim the stretch is NAMED by: the one nearest the end it is anchored at, which is the
    // one the strip really runs off first, rather than whichever the region happened to hand over
    // first. A tessellated knife edge answers with the facet at that end, and its plane speaks for
    // the rest because they were all measured against it below.
    //
    // KNOWN LIMIT: a tessellated knife edge whose first facet TOUCHES the anchored corner has that
    // facet excluded by the corner rule above, so the stretch reads as standing a facet short of
    // the end and is refused as mid-span. That is the safe direction - a refusal rather than a
    // build - but the name is the wrong one for it.
    let mut named = 0;

    for at in 1..runs_off.len() {
        let here = if touches_the_start {
            windows[at].0
        } else {
            -windows[at].1
        };
        let best = if touches_the_start {
            windows[named].0
        } else {
            -windows[named].1
        };

        if here < best
            || (here == best && runs_off[at].mesh_edge_index < runs_off[named].mesh_edge_index)
        {
            named = at;
        }
    }

    let mut plane_normal = Vector3::ZERO;
    let mut plane_point = Vector3::ZERO;
    let mut seed_beyond = -1;
    let mut more_than_one_plane = false;
    let mut rims: HashSet<i32> = HashSet::new();

    for at in 0..runs_off.len() {
        // The named rim first, so the plane the others are measured against - and the point the
        // build is handed - is the one the strip runs off at its own end.
        let rim = runs_off[if at == 0 {
            named
        } else if at <= named {
            at - 1
        } else {
            at
        }];

        let (Some((_, beyond)), Some(face_beyond)) = (
            surface_room_standing_feature::try_the_two_sides(region, rim),
            surface_room_standing_feature::try_the_face_beyond(region, rim),
        ) else {
            return Ok(None);
        };

        rims.insert(rim.mesh_edge_index);

        if seed_beyond < 0 {
            plane_normal = beyond;
            plane_point = rim.start;
            seed_beyond = face_beyond;
            continue;
        }

        // Two UNIT normals compared against a length tolerance, which is the reading the
        // whole-span rule above makes as well: at these sizes the chord between two unit normals
        // and the angle between them are the same number, so `near` reads as an angle here.
        if (beyond - plane_normal).length() > near
            || (rim.start - plane_point).dot(plane_normal).abs() > near
        {
            // More than one face beyond the stretch - two flats meeting at a corner the strip
            // would cross, or one more facet of a curve. Refused for the reason the whole-span
            // reading refuses it, and by the same two names: the trim is a crossing with ONE plane
            // in closed form, and there is no size at which it is a crossing with two.
            more_than_one_plane = true;
            break;
        }
    }

    if !the_surface_beyond_is_one_plane(mesh, seed_beyond, plane_point, plane_normal, near) {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceStripRunsOffACurvedFace,
            middle,
        ));
    }

    if more_than_one_plane {
        return Err(refusal(
            BevelAdvisoryCode::SurfaceStripRunsOffAtACorner,
            middle,
        ));
    }

    Ok(Some(SurfaceBandRunsOffOverAStretch::new(
        touches_the_start,
        if touches_the_start {
            highest
        } else {
            span.length - lowest
        },
        *runs_off[named],
        plane_normal,
        plane_point,
        Arc::new(rims),
    )))
}

/// A refusal of this reading's, which every code here words at one point of the edge.
fn refusal(code: BevelAdvisoryCode, at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(code, vec![BevelAdvisoryArg::Vector3(at)])
}

/// Whether `rim` starts or ends at `corner`.
fn shares_a_corner(rim: &FeatureEdge, corner: Vector3, near: f64) -> bool {
    (rim.start - corner).length() <= near || (rim.end - corner).length() <= near
}

/// Whether every facet of the surface `seed` belongs to lies in the one plane the trim would cut
/// against. The surface is grown at [`FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES`], which is what
/// makes a tessellated cylinder one thing to ask the question of rather than forty.
fn the_surface_beyond_is_one_plane(
    mesh: Option<&Mesh>,
    seed: i32,
    plane_point: Vector3,
    plane_normal: Vector3,
    near: f64,
) -> bool {
    let Some(mesh) = mesh else {
        return false;
    };

    if seed < 0 || seed as usize >= mesh.faces.len() {
        return false;
    }

    let Ok(beyond) = FaceGroup::create_with_graph(
        mesh,
        &mesh.get_mesh_edge_graph(),
        seed,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ) else {
        // Unreachable: the seed was range-checked above, which is the flood's only refusal.
        return false;
    };

    beyond.faces().iter().all(|&face| {
        let face = &mesh.faces[face as usize];
        [face.v0, face.v1, face.v2].iter().all(|&corner| {
            (Vector3::from_vector3_float(mesh.vertices[corner as usize]) - plane_point)
                .dot(plane_normal)
                .abs()
                <= near
        })
    })
}

/// The part of a rim in front of the chain edge within its span: how far back the farthest of it
/// reaches, and the window along the span where it stands NEARER than the setback (C#'s `out`
/// values; `under_to < under_from` where there is none).
#[derive(Debug, Clone, Copy)]
struct InFront {
    farthest: f64,
    under_from: f64,
    under_to: f64,
}

/// The part of `rim` that stands IN FRONT of the chain edge within its own span, and how far back
/// from the edge the farthest of it reaches. `None` where no stretch of it does - a rim beyond the
/// band's ends, or behind the edge's line, bounds nothing the band takes.
///
/// Read as a clip in the edge's own frame rather than as a distance, because the two questions are
/// different: a rim that only TOUCHES the span at one end (the top face's far rim, meeting the
/// x = 0 rim exactly at the end plane) is no part of what the band takes, and a point's distance
/// cannot say so. A stretch shorter than a rounding is that touch, and is let past.
fn try_in_front_over_the_span(rim: &FeatureEdge, span: &BandSpan, near: f64) -> Option<InFront> {
    let start_along = (rim.start - span.from).dot(span.along);
    let end_along = (rim.end - span.from).dot(span.along);
    let start_in = (rim.start - span.from).dot(span.inward);
    let end_in = (rim.end - span.from).dot(span.inward);

    let mut low = 0.0;
    let mut high = 1.0;

    if !narrow(start_along, end_along, 0.0, &mut low, &mut high)
        || !narrow(-start_along, -end_along, -span.length, &mut low, &mut high)
        || !narrow(start_in, end_in, 0.0, &mut low, &mut high)
    {
        return None;
    }

    let runs = (end_along - start_along).abs() + (end_in - start_in).abs();

    if (high - low) * runs <= near {
        return None;
    }

    let farthest = cs_max(
        start_in + ((end_in - start_in) * low),
        start_in + ((end_in - start_in) * high),
    );

    // And the part of that stretch the band has already eaten past: where the rim stands NEARER
    // than the setback. Closed form, because both readings are linear along the rim - a straight
    // rim in a plane frame - so the window is one interval and its ends are where the offset
    // crosses the setback.
    let mut under_low = low;
    let mut under_high = high;
    let mut under_from = 0.0;
    let mut under_to = -1.0;

    // A rim PARALLEL to the chain edge has an offset that does not rise along its own length, so
    // there is no crossing to solve for: it is wholly under the setback or wholly over it, and a
    // rim standing EXACTLY at the setback is one the band reaches and does not pass. That is the
    // reading the whole-span rule above makes as well (`farthest >= setback - near` leaves a
    // landing), and it is the RULER that settles it, because an exact reading of a degenerate
    // solve answers with the rim's whole length.
    //
    // Everywhere else the crossing is solved exactly, so that the station the stretch ends at is
    // the one where the wall really is a setback across - which is what lets the rest of the span
    // go on being charged at exactly the size that was asked for (SurfaceRoomMeasure.RoomAcross).
    let it_rises = (end_in - start_in).abs() > near;

    if if it_rises {
        narrow(
            -start_in,
            -end_in,
            -span.setback,
            &mut under_low,
            &mut under_high,
        )
    } else {
        start_in < span.setback - near
    } {
        let at_low = start_along + ((end_along - start_along) * under_low);
        let at_high = start_along + ((end_along - start_along) * under_high);

        under_from = cs_max(cs_min(at_low, at_high), 0.0);
        under_to = cs_min(cs_max(at_low, at_high), span.length);
    }

    Some(InFront {
        farthest,
        under_from,
        under_to,
    })
}

/// Narrows the parameter window [`low`, `high`] of a segment to the part of it where the reading
/// running from `at_start` to `at_end` is at least `floor`. False where nothing of it is.
fn narrow(at_start: f64, at_end: f64, floor: f64, low: &mut f64, high: &mut f64) -> bool {
    let rise = at_end - at_start;

    if rise.abs() < 1e-12 {
        return at_start >= floor;
    }

    let crosses = (floor - at_start) / rise;

    if rise > 0.0 {
        *low = cs_max(*low, crosses);
    } else {
        *high = cs_min(*high, crosses);
    }

    *low <= *high
}

/// C# `Math.Max`: NaN propagates, which `f64::max` would not.
fn cs_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}

/// C# `Math.Min`: NaN propagates, which `f64::min` would not.
fn cs_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}

/// C# `double.CompareTo`: NaN sorts first, and equal values (0.0 and -0.0 included) are equal.
fn cs_compare(a: f64, b: f64) -> Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => a.partial_cmp(&b).unwrap_or(Ordering::Equal),
    }
}
