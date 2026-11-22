//! The one rule for a face's normal where its corners may be too close to a line to give one:
//! whether a triangle's corners can be trusted with a normal ([`normal_of_corners`]) and, where
//! they cannot, the walk to the face a sliver lies along ([`trusted_normal_across_slivers`]).
//! `FeatureEdge::try_classify` reads it over a `Mesh` and its edge graph, and the bevel's level
//! line over its own triangle lists (`triangle_edges::normal_an_edge_reads`,
//! `LevelLineMesh::normal_an_edge_reads`), so every reading routed through it takes a sliver the
//! same way.
//!
//! Not every reader goes through it. These level-line readers still take a face's normal straight
//! off its corners, because they are handed a triangle without its neighbours:
//! `end_face_sense::face_normal` (and through it `is_opposite_sense_crease`,
//! `side_runs_past_the_end_plane`, `end_ray_bound_at` and `convex_end_faces`) and
//! `end_face_sense::is_convex`, `end_face_sense_oblique`, `flip_corner_clip`, `band_gap_crest`,
//! `crease_run_out`, `point_star` and `point_bevel`. `hole_fill` reads the orientation of
//! triangles it is building, which are no face of the part, and keeps the raw normal on purpose.
//! The level line's other cross products (frames, arcs, fills) have not all been checked for a
//! face-normal reading, and the `sections` and `surface` constructions read their own.

use solidmesh_math::Vector3;

/// The relative precision a single-precision coordinate is placed to: half a unit in the last
/// place (2^-24).
pub const SINGLE_PRECISION_PLACEMENT: f64 = 5.960_464_477_539_063e-8;

/// The relative precision a double-precision coordinate is placed to (2^-53).
pub const DOUBLE_PRECISION_PLACEMENT: f64 = 1.110_223_024_625_156_5e-16;

/// How many faces too thin for a normal [`trusted_normal_across_slivers`] steps across before it
/// gives up: slivers lying side by side along one crease are a few faces, never more.
const SLIVER_STEPS: usize = 8;

/// The outward unit normal of the triangle wound through `corners`, or `None` where its corners
/// cannot define one. `placement` is the relative precision the corners were placed to
/// ([`SINGLE_PRECISION_PLACEMENT`] for a float-stored mesh, [`DOUBLE_PRECISION_PLACEMENT`] for
/// one built in doubles).
///
/// A face has no normal when its least height (twice its area over its longest edge) is no more
/// than rounding can move a corner, `placement (|x| + |y| + |z|)` at its farthest-out corner: its
/// third corner may as well lie on its longest edge, so which side of that edge the face lies on,
/// and with it which way its normal points, is not a fact the input holds.
pub fn normal_of_corners(corners: [Vector3; 3], placement: f64) -> Option<Vector3> {
    let [position0, position1, position2] = corners;
    let cross = (position1 - position0).cross(position2 - position0);
    let length_squared = cross.length_squared();
    // NaN is refused too, as `!(length_squared > 0)` would refuse it.
    if length_squared.is_nan() || length_squared <= 0.0 || length_squared.is_infinite() {
        return None;
    }

    let longest = (position1 - position0)
        .length()
        .max((position2 - position1).length())
        .max((position0 - position2).length());
    let reach = |p: Vector3| p.x.abs() + p.y.abs() + p.z.abs();
    let moved = placement * reach(position0).max(reach(position1)).max(reach(position2));
    if length_squared.sqrt() / longest <= moved {
        return None;
    }

    Some(cross.get_normal())
}

/// The normal an edge of face `t` reads its turn with: the face's own where `own_normal` gives
/// one, and otherwise that of the face a sliver lies along.
///
/// A sliver too thin for a normal lies along its longest edge, so the surface beside it is the
/// face across that edge - unless that face is a sliver too, lying along the same edge: two
/// slivers sharing their longest edge are the thin quad a boolean leaves split on its diagonal,
/// and each half lies along the faces across its own shorter edges, not across the other half.
/// So the walk takes the sliver's edges longest first and steps across the first whose far face
/// is neither crossed already nor a sliver sharing that edge as its own longest; only where every
/// far face is one of those does it step into such a sliver. `None` where no face with a normal is
/// reached.
///
/// Read this way a flat face with a two-sliver strip in it is flat throughout, and a strip lying
/// along a crease keeps the crease on the strip's shared edge, its two halves reading the faces on
/// either side of it.
///
/// `corners_of` names a face's corners in winding order, `position` a vertex's place, and `across`
/// the one other face standing on an edge (`None` on a boundary or non-manifold edge).
pub fn trusted_normal_across_slivers(
    t: i32,
    own_normal: &dyn Fn(i32) -> Option<Vector3>,
    corners_of: &dyn Fn(i32) -> [i32; 3],
    position: &dyn Fn(i32) -> Vector3,
    across: &dyn Fn(i32, (i32, i32)) -> Option<i32>,
) -> Option<Vector3> {
    let edges_longest_first = |face: i32| {
        let corners = corners_of(face);
        let mut edges = [
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[0]),
        ];
        let length = |(a, b): (i32, i32)| (position(b) - position(a)).length_squared();
        // Longest first; a tie keeps winding order (the sort is stable).
        edges.sort_by(|&p, &q| length(q).total_cmp(&length(p)));
        edges
    };
    let same_edge = |p: (i32, i32), q: (i32, i32)| p == q || p == (q.1, q.0);

    let mut at = t;
    let mut crossed = [t; SLIVER_STEPS];
    for step in 0..SLIVER_STEPS {
        if let Some(normal) = own_normal(at) {
            return Some(normal);
        }

        crossed[step] = at;
        let mut beside: Option<i32> = None;
        let mut sharing: Option<i32> = None;
        for edge in edges_longest_first(at) {
            let Some(face) = across(at, edge) else {
                continue;
            };
            if crossed[..=step].contains(&face) {
                continue;
            }
            let shares_its_longest =
                own_normal(face).is_none() && same_edge(edges_longest_first(face)[0], edge);
            if shares_its_longest {
                sharing = sharing.or(Some(face));
            } else {
                beside = Some(face);
                break;
            }
        }

        at = beside.or(sharing)?;
    }

    None
}
