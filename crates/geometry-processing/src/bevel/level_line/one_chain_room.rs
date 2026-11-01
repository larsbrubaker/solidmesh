//! The room of a pass of ONE chain as the band gap's build reads it, on the mesh tagged into its
//! two sides ([`read_in_the_build`], split from `band_gap_build_within.rs`), with the setbacks
//! [`the_units`] gives: the rolling ball's where it takes the pass, the angle's otherwise. The
//! build refuses the size exactly when it stands above the room, so the room has to be read off
//! the band the build cuts.
//!
//! **Why the ball's setbacks at the size asked, scaled.** The room is read as a ratio: a field
//! family `s -> field with setbacks u * s` searched at `band_room::REFERENCE_SIZE` for the largest
//! size at which the band reaches nothing it may not. The build asks it one question: does the
//! size asked fit? The ball is walked at that size (`rolling_ball_setbacks`), and its contacts
//! read per unit of that size give the family whose member at the size asked is exactly the band
//! the build cuts. So the yes or no is exact, and a free end whose round closes onto its end vertex
//! is no longer refused by a crease the level line's round, run one setback past the end, would
//! have reached. Re-walking the ball at every size the room's search tries instead would cost a
//! walk per probe, for a number only a refusal reads.
//!
//! **The number a refusal names is an estimate.** Below the size asked the family is the ball's
//! band at the size asked shrunk, not the ball at the smaller size: a contact's distance is not
//! linear in the size on a curved side, and at a free end the taper (setbacks capped by the
//! distance from the end vertex, a cap that does not shrink with the size) makes the shrunk band
//! narrower near the end than the ball's at the smaller size, so the estimate there is optimistic.
//! A size quoted below the size asked may be refused again when the ball is walked there; the size
//! ladder then falls back to its own smaller sizes. It cannot loop: each refusal is a build at a
//! smaller size. Where the ball's contact rests on a crease (a rim of its side), its setback per
//! unit is that crease's distance over the size asked, so the scaled family reaches the crease at
//! exactly the size asked: a refusal there names a room of about the size asked, the size ladder
//! builds just under it, is refused again and falls back to the percent - no loop, but a number
//! that does not help the user.
//!
//! **Each side against its own band.** Where the band's tagging divides the chain's two sides (tags
//! 1 and 2 both present, [`the_sides`]), each foreign crease on one side alone is read against that
//! side's own setbacks, and each side runs out of itself (the self room) at its own band; a crease
//! touching both sides, an end region or no band group, is read against the larger side, as
//! before. The room is the least over the sides (`foreign_edge_room_per_side`). The side of a crease
//! is read off the tags of the triangles on it, the tags the band is cut by, so the room reads the
//! same side the cut does. Cost: the foreign edges are filed once and read in up to three lists
//! (left, right, either), each with the unchanged bracket and fixed bisection, so each edge is read
//! in one list but up to three times the trial fields are built where a side's setbacks are not one
//! number; the self room builds each side's own field and samples its vertices with it. A growing
//! crease run's pierce room is read from the ball's own centres ([`the_path`],
//! `ball_pierce_room`), which stand a radius from both sides. Where the build's side
//! floods join the two sides (sides joined round a free end) and the ball takes the pass, the band
//! is tagged from the ball's own sides (`rolling_ball_band_tags`), so the room reads each side
//! there too.
//!
//! **Where it applies.** A pass of one chain, open or closed, that the ball may take - its switch
//! on, the outer pass, no topology-only probe - is neither refused nor trusted on a room the caller
//! handed in (`band_gap::build_unvalidated` sets it aside: it was read with the angle's setbacks),
//! so the build reads its own, with the ball's setbacks where the ball takes the pass and the
//! angle's where it does not, and a refusal names that room. A pass of several chains reads the
//! pick's room (`band_room_pair_room_pick`, its pair and corner readings with the angle's
//! setbacks). With the switch off nothing here changes what the build reads.

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::Vector3;

use super::ball_pierce_room::BallPath;
use super::band_gap_setback_fields::PassSetbacks;
use super::band_room;
use super::foreign_edge_room_per_side::SideRoomFields;
use super::rolling_ball_setbacks::SideSetbacks;
use super::{end_face_sense, BandProfile, ChainField, LevelLineMesh, RoomSource, WorkError};

/// The setbacks per unit of size the room of a pass of one chain is read with: the ball's larger
/// side at each chain vertex where the ball takes the pass (`ball`) and the pass is of one chain,
/// otherwise `None` (the angle's, `PassSetbacks::unit_setbacks`).
pub fn the_units(ball: Option<&[SideSetbacks]>, only: bool) -> Option<Vec<f64>> {
    match ball {
        Some([chain]) if only => Some(chain.larger()),
        _ => None,
    }
}

/// The ball's two sides of the pass, where [`the_units`] reads the ball's setbacks and the mesh is
/// tagged into two sides (`tags` holds both 1 and 2: a crease that dies out is one region holding
/// both sides, whose edges no one side's band reaches alone): the room then reads each side's
/// foreign edges against that side's own setbacks.
pub fn the_sides<'a>(
    ball: Option<&'a [SideSetbacks]>,
    only: bool,
    tags: &[i32],
) -> Option<&'a SideSetbacks> {
    let divided = tags.contains(&1) && tags.contains(&2);
    match ball {
        Some([chain]) if only && divided => Some(chain),
        _ => None,
    }
}

/// The ball's centre path for the growing crease runs' pierce room where the ball takes a pass of
/// one chain (the same condition as [`the_units`]), walked at `size` along `chain_points`; the
/// round is carried past an end only where that end carries an end ray (`end_rays`).
pub fn the_path<'a>(
    ball: Option<&'a [SideSetbacks]>,
    only: bool,
    chain_points: &'a [Vector3],
    closed: bool,
    end_rays: (bool, bool),
    size: f64,
) -> Option<BallPath<'a>> {
    match ball {
        Some([chain]) if only => BallPath {
            chain_points,
            unit_centres: &chain.centres,
            closed,
            carried: [end_rays.0, end_rays.1],
            size,
        }
        .checked(),
        _ => None,
    }
}

/// The room of chain 0 read on `mesh`, the input tagged into its two sides, with the setbacks
/// `units` per unit of size: the level line's own reading, its convex end faces read against the
/// chain without their rays (`band_room::room_of_cut_mesh` says why). With the ball's `sides`
/// ([`the_sides`]), each foreign edge on one side alone is read against that side's own setbacks;
/// with the ball's `path` ([`the_path`]), the growing crease runs' pierce room is read from it.
#[allow(clippy::too_many_arguments)]
pub fn read_in_the_build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    mesh: &Rc<RefCell<LevelLineMesh>>,
    setbacks: &PassSetbacks,
    units: &[f64],
    sides: Option<&SideSetbacks>,
    path: Option<&BallPath>,
    snap: f64,
    seam_epsilon: f64,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<(f64, RoomSource), WorkError> {
    let end_rays = setbacks.end_rays[0];
    let convex_ends = end_face_sense::convex_end_faces(positions, indices, chain, end_rays);
    let without_convex_rays = (
        end_rays.0 && convex_ends.0.is_none(),
        end_rays.1 && convex_ends.1.is_none(),
    );
    let field_at = |units: &[f64], s: f64| {
        setbacks
            .field_from(0, units, s)
            .expect("the chain's polyline built a field at the size asked")
    };
    let field_without_convex_rays = |units: &[f64], s: f64| {
        let scaled: Vec<f64> = units.iter().map(|u| u * s).collect();
        ChainField::from_polyline_bounded(
            &setbacks.chain_points[0],
            closed,
            &scaled,
            setbacks.sharp_turn_degrees(),
            without_convex_rays,
            setbacks.end_ray_bounds[0],
        )
        .expect("the chain's polyline built a field at the size asked")
    };
    let present = band_room::present(convex_ends);
    let (left, right): (&[f64], &[f64]) = match sides {
        Some(sides) => (&sides.left, &sides.right),
        None => (&[], &[]),
    };
    let (left_at, right_at) = (|s| field_at(left, s), |s| field_at(right, s));
    let (left_bare, right_bare) = (
        |s| field_without_convex_rays(left, s),
        |s| field_without_convex_rays(right, s),
    );
    let side_fields = sides.map(|_| SideRoomFields {
        units: [left, right],
        field_at: [&left_at, &right_at],
        without_convex_rays: [&left_bare, &right_bare],
    });
    band_room::room_of_cut_mesh(
        mesh,
        &setbacks.chain_edges[0],
        &|s| field_at(units, s),
        units,
        closed,
        snap,
        seam_epsilon,
        profile,
        fillet_segments,
        Some(&present),
        Some(&|s| field_without_convex_rays(units, s)),
        side_fields.as_ref(),
        path,
    )
}
