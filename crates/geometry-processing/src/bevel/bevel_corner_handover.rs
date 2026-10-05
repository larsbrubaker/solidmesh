//! The two corner treatments a run's end hands over to before it looks at any face of its own: a
//! corner patch's ball, and a mixed pair's blend. Ports `Bevel/BevelCornerHandover.cs` (C#
//! `internal static class`; a module of free functions here).
//!
//! Both are corners something else has already decided the shape of, so the end's whole job is
//! to stop on the plane that thing published rather than read the topology a second time. They
//! come first, and they come as a pair, for the reason written inside [`try_hand_over`].
//!
//! The tables are C# `IReadOnlyDictionary`s / `IReadOnlySet`, only ever looked up here, each
//! possibly absent (C# null). The dictionaries are [`IndexMap`]s as in
//! [`super::bevel_blend_seam`], because the sets that build them keep C#'s insertion order.

use std::collections::HashSet;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::{bevel_blend_seam, bevel_corner_seam, BevelEndCap};

/// The plane this end hands its corner over on, or `None` (C# false) where neither owns the
/// corner and the caller's own face scan has to decide.
///
/// `vertex` is the run's end vertex and `position` where it is; `outward` the direction the sweep
/// leaves this end by; `own_edge` the mesh edge of the run's own last edge, which is what keys a
/// blend's handover; `is_bead` whether this run is unioned on rather than cut away. The tables:
/// a corner patch's ball centre by vertex, the plane a blend hands a cut over on and the face
/// normal it caps a bead on by (vertex, mesh edge), the patched vertices whose runs may drive
/// through, and the points a patch has already put on its plane by (vertex, mesh edge).
#[allow(clippy::too_many_arguments)]
pub fn try_hand_over(
    vertex: i32,
    position: Vector3,
    outward: Vector3,
    own_edge: i32,
    is_bead: bool,
    patched_corners: Option<&IndexMap<i32, Vector3>>,
    blend_handovers: Option<&IndexMap<(i32, i32), BevelEndCap>>,
    blend_capped_corners: Option<&IndexMap<(i32, i32), Vector3>>,
    runs_may_drive_through: Option<&HashSet<i32>>,
    patch_seam_rings: Option<&IndexMap<(i32, i32), Vec<Vector3>>>,
) -> Option<BevelEndCap> {
    // These two branches are tried in an order, and what makes the order safe rather than lucky
    // is that a vertex CANNOT be in both maps: a corner patch is built from three or more
    // selected ends all of one convexity, and a blend from exactly two of opposite convexity, so
    // no vertex can present both shapes at once. The two maps carry different types - a point
    // for the patch and a whole plane for the blend. If it ever stopped holding, this order would
    // hand a run the patch's SQUARE plane while the blend went on cutting against its own leaning
    // one, and the two solids would cross tangentially at the seam rather than meeting on it,
    // which is the single failure this whole handover exists to prevent. What holds it at three
    // selected edges of mixed convexity - where a blend now builds and a patch still needs three
    // of ONE kind - is that four or more mixed edges are refused by both (`BevelMixedBlends`'
    // remarks say it there).
    if let Some(&ball_centre) = patched_corners.and_then(|corners| corners.get(&vertex)) {
        // The corner patch owns everything past here, so the chain hands the corner over on the
        // plane the patch's own wall lies in: square to the run, through the ball's centre.
        //
        // A bead at a FILLED corner reads the same line for the same reason: the two beads that
        // also end at that vertex are selected neighbours of this one's convexity, so the
        // fallbacks would send it to the overshoot on the grounds that their union resolves the
        // corner - which is exactly the thing that leaves the dimple. The patch is what resolves
        // it now, and it needs the plane. Both solids then carry the same arc on that plane and
        // the union has coplanar faces rather than the tangential crossing an overshoot would
        // leave.
        //
        // Except where the corner lets the run drive straight THROUGH it instead of stopping on
        // that wall, which is the whole of `BevelCornerSeam`. The seam points are the ones the
        // patch has already put on that plane, which this end carries rather than laying its own
        // arc a rounding away from them - see `BevelSeamRing`. `None` where the caller had no
        // patch in hand, which leaves the ring exactly as the sweep has always built it.
        let seam_points = patch_seam_rings.and_then(|rings| rings.get(&(vertex, own_edge)));

        if let Some(through) = bevel_corner_seam::try_drive_through(
            is_bead,
            vertex,
            outward,
            position,
            ball_centre,
            runs_may_drive_through,
            seam_points.map(Vec::as_slice),
        ) {
            return Some(through);
        }

        return Some(BevelEndCap::with_options(
            outward,
            outward.dot(ball_centre),
            false,
            0.0,
            seam_points.cloned(),
        ));
    }

    let seam = bevel_blend_seam::blend_seam(
        vertex,
        own_edge,
        position,
        is_bead,
        blend_handovers,
        blend_capped_corners,
    )?;

    let along_outward = seam.plane_normal.dot(outward);

    if along_outward.abs() > 0.0 {
        // Both halves of a blended corner read one line here. The cut stops on the plane the
        // blend drew its first ring in and the bead stops on the face the blend read its rim off
        // - see `BevelMixedBlends` - and each is published as the plane it is rather than as a
        // point or a normal for this to reconstruct, because reconstructing it is what used to
        // leave the two solids crossing tangentially at a corner that leaned. Only the SENSE is
        // decided here, the way every cap is: pointing out the way the sweep leaves the end.
        return Some(if along_outward < 0.0 {
            BevelEndCap::new(-seam.plane_normal, -seam.plane_distance)
        } else {
            seam
        });
    }

    None
}
