//! WHAT THE NARROWER BEAD'S FAR FACE GIVES UP at a mitre of two UNEQUAL setbacks: the sector
//! between the wider strip's unpaired rails and the corner, which is that strip's ordinary end cut
//! on that face clipped by the narrower band's own end line. The cut itself is
//! [`super::surface_end_cut::try_cut`]; the mitre that leaves the unpaired rails is
//! `surface_strip_mitre::try_mitre`.
//!
//! **Why there is anything to give up.** Two beads of one radius mitred on a face they share sweep
//! DIFFERENT turns where their wedges differ - a 45 degree knife tip turns 135 where a 90 degree
//! edge turns 90 - so the wider of the two has rails the narrower has none to pair with. Those
//! rails are not stopped by the mitre at all: nothing of the narrower bead stands over them, and
//! what they end on is the part's own face beyond it - the narrower edge's FAR landing face, still
//! standing where it always did. That face therefore has to be opened for them, exactly as an
//! ordinary end cut opens the face a chain stops against.
//!
//! **And why it is not the ordinary cut itself.** The wider strip's end on that face is a
//! STAIRCASE: its unpaired rails land on the face, and its paired ones stand on the mitre plane,
//! off the face altogether. So the section is the polygon the strip would take alone - between its
//! profile and the corner - CLIPPED by the narrower band's end line, which is where the mitre plane
//! meets that face. Past the clip the face is already gone: the narrower band was taken off up to
//! that line and no further. The clip adds one vertex wherever the line crosses an edge bounding
//! the face, and at a three-face corner those edges are the two other faces of the corner - the one
//! the beads SHARE and the wider bead's own other face - so each crossing is one point three planes
//! meet at.
//!
//! **The tip is void, and that is right.** The piece of the far face between the two crossings and
//! the corner comes away with the section and nothing replaces it: its other two edges run along
//! the shared face and along the wider bead's other face, and both of those are bands that have
//! already gone. It is a corner of the solid the two beads between them removed entirely, so after
//! the cut its edges are the edges of no face at all, and the shell closes because nothing is left
//! standing on either side of them.
//!
//! **What it assumes.** That the far landing face is ONE planar region holding the whole profile,
//! which `SurfaceEndCut` reads for itself and refuses as `SurfaceStripEndNotSupported` where it is
//! not; and that nothing was rolled at this end - a bead rolled along a curved wall would want its
//! cap to follow the walked chords rather than run straight back to the corner, and no fixture
//! mitres unequal setbacks onto a roll.

use vector_math::Vector3;

use super::surface_end_cut;
use super::{SurfaceEdgePlan, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How much volume three unit normals have to span before their planes are read as crossing in a
/// point: a millionth, which is a thousandth of a radian off being one line - far under any real
/// corner of a part and far over the rounding of three normals read off the same mesh.
const THREE_PLANES_REALLY_CROSS: f64 = 1e-6;

/// Opens the narrower bead's far landing face for the wider strip's unpaired rails, and says what
/// the cut left standing part way along each chord of them - for the strip's station, empty on
/// every chord but the unpaired ones, which are the only ones this cut runs along.
///
/// `plan` is the WIDER strip's edge plan, whose two regions name the shared face and its other one;
/// `away_from_the_corner` runs along that edge away from the corner (the edge's own direction
/// reversed where the walk LEAVES by this strip, the direction itself where it arrives); `ring` is
/// its end ring at this mitre, rail by rail (the staircase); `shared_rail` which of its rails
/// stands on the face the two beads share (0 or the last); `paired` how many of its rails the
/// narrower bead had one each to pair with; `through`/`normal` the mitre plane;
/// `beyond_point`/`beyond_normal` the narrower edge's far landing plane; and `turn_vertex` where
/// this strip really leaves the mitre plane for the face beyond when that is not one of its own
/// rails - the NARROWER bead's outermost rail crossing, which only a mitre whose two arcs were
/// CHORDED APART has - or -1.
#[allow(clippy::too_many_arguments)]
pub fn try_give_up_the_sector(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    plan: &SurfaceEdgePlan,
    away_from_the_corner: Vector3,
    ring: &[i32],
    shared_rail: i32,
    paired: i32,
    corner: Vector3,
    through: Vector3,
    normal: Vector3,
    beyond_point: Vector3,
    beyond_normal: Vector3,
    turn_vertex: i32,
) -> Result<Vec<Vec<i32>>, BevelAdvisory> {
    let mut between = nothing_along(ring.len() as i32);
    let ring_length = ring.len() as i32;

    if paired >= ring_length {
        // The two beads swept the same turn, so the mitre stopped both strips and there is no
        // sector: this is every mitre of two equal setbacks, and the face beyond is not touched.
        return Ok(between);
    }

    let step = if shared_rail == 0 { 1 } else { -1 };

    // The turn point - the last rail that DID pair, which stands on the mitre plane and on the far
    // face at once - and the far rail, this strip's own outermost on its other face.
    let turn = shared_rail + (step * (paired - 1));
    let far = shared_rail + (step * (ring_length - 1));

    // The two faces of the corner the far face is bounded by here: the one the beads share, on the
    // shared rail's side, and this bead's own other face on the far rail's side. Both hold the
    // whole edge, so both pass through the corner.
    let shared = plan.edge.direction().cross(if shared_rail == 0 {
        plan.left_inward_at_the_edge
    } else {
        plan.right_inward_at_the_edge
    });

    let other = plan.edge.direction().cross(if shared_rail == 0 {
        plan.right_inward_at_the_edge
    } else {
        plan.left_inward_at_the_edge
    });

    let (Some(on_the_shared), Some(on_the_other)) = (
        try_three_planes(corner, shared, beyond_point, beyond_normal, through, normal),
        try_three_planes(corner, other, beyond_point, beyond_normal, through, normal),
    ) else {
        // Three planes that meet in no point: the band end line runs along one of the faces it is
        // meant to cross rather than across it, and there is no polygon to hand over.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripsDoNotMitre,
            vec![corner.into()],
        ));
    };

    // The profile, from the crossing on the SHARED side round the unpaired rails to the crossing on
    // the other: the two crossings weld onto the vertices the walkback already left where its band
    // end met those two edges, so the cut sews to them rather than beside them.
    let mut profile: Vec<i32> = Vec::with_capacity((ring_length - paired + 2) as usize);
    profile.push(pool.add(on_the_shared));

    // Where the section really begins. Ordinarily that is the last rail that paired, which stands
    // on the mitre plane and on this face at once; where the two arcs were chorded apart it is no
    // rail of this strip at all but the narrower bead's own outermost, standing part way along the
    // chord this strip leaves the mitre plane over - so the section begins there and this strip's
    // own rail at that step stays where it is, under the narrower bead and on the mitre plane.
    let begins = if turn_vertex >= 0 {
        turn_vertex
    } else {
        ring[turn as usize]
    };

    profile.push(begins);

    let mut rail = turn + step;
    loop {
        profile.push(ring[rail as usize]);

        if rail == far {
            break;
        }

        rail += step;
    }

    let die_point = pool.add(on_the_other);

    profile.push(die_point);

    // The chord into the last crossing runs ALONG the section's own straight side - that side is
    // the edge between the far face and this bead's other face, and the far rail's landing stands
    // on it too - so it names no half plane and is the cut's die point rather than a cut of its
    // own.
    let along_the_profile = surface_end_cut::try_cut(
        pool,
        faces,
        &profile,
        corner,
        false,
        away_from_the_corner,
        Vector3::ZERO,
        -1,
        die_point,
    )?;

    // Back onto the ring's own chords. The profile's first and last chords run to the two
    // crossings, which are no rails of this strip and no part of what it sews to - they bound the
    // void tip - so only the rail to rail ones in between have a station chord to land on.
    let mut at = 1usize;
    while at + 1 < profile.len() - 1 {
        let rail = turn + (step * (at as i32 - 1));
        let chord = &along_the_profile[at];

        // The transition point stands on the chord the section begins over, nearer this strip's
        // own rail than anything the cut left there, so it leads that chord's list.
        let whole = if at == 1 && turn_vertex >= 0 {
            leading(turn_vertex, chord)
        } else {
            chord.clone()
        };

        if step > 0 {
            between[rail as usize] = whole;
        } else {
            // The chord runs from the higher rail to the lower one where the shared face is the
            // last rail's, and a station's chord always runs the other way about.
            let mut backwards = whole;

            backwards.reverse();

            between[(rail - 1) as usize] = backwards;
        }

        at += 1;
    }

    Ok(between)
}

/// One vertex put in front of what a chord already carries.
fn leading(vertex: i32, rest: &[i32]) -> Vec<i32> {
    let mut whole = Vec::with_capacity(rest.len() + 1);
    whole.push(vertex);
    whole.extend_from_slice(rest);
    whole
}

/// Nothing on any chord, which is what every station but this one carries.
pub fn nothing_along(rails: i32) -> Vec<Vec<i32>> {
    vec![Vec::new(); (rails - 1).max(0) as usize]
}

/// The one point three planes meet at, or `None` where they meet in a line or not at all.
///
/// The normals are taken as given rather than as unit vectors, so they are normalised here: the
/// determinant below is the volume the three span, and only against unit normals does a small one
/// mean a shallow crossing rather than a short input.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
pub fn try_three_planes(
    first_point: Vector3,
    first_normal: Vector3,
    second_point: Vector3,
    second_normal: Vector3,
    third_point: Vector3,
    third_normal: Vector3,
) -> Option<Vector3> {
    if !(first_normal.length_squared() > 0.0)
        || !(second_normal.length_squared() > 0.0)
        || !(third_normal.length_squared() > 0.0)
    {
        return None;
    }

    let one = first_normal.get_normal();
    let two = second_normal.get_normal();
    let three = third_normal.get_normal();

    let across = two.cross(three);
    let volume = one.dot(across);

    if volume.abs() < THREE_PLANES_REALLY_CROSS {
        return None;
    }

    Some(
        ((first_point.dot(one) * across)
            + (second_point.dot(two) * three.cross(one))
            + (third_point.dot(three) * one.cross(two)))
            / volume,
    )
}
