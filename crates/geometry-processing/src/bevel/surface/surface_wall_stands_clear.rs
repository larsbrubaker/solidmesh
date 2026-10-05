//! Whether a wall that a section walk calls CURVED need not be rolled along after all, because the
//! plain two-plane ball - tangent to the flat face and to the chain edge's OWN facet's plane -
//! already stands clear of the whole wall. Ports `Bevel/Surface/SurfaceWallStandsClear.cs` (C#
//! `internal static class`; a module of free functions here). It asks its questions of the wall's
//! [`SurfaceRegionFaceGrid`] and reads the walk's first step ([`SurfaceRunAcrossSectionStep`]);
//! `SurfaceRollAcrossTheWall` decides whether to roll with it.
//!
//! **Why the section walk is not the last word.** The roll is decided by a section SQUARE TO THE
//! CHAIN EDGE: where that section leaves the chain edge's own facet before the profile's foot, the
//! wall is held to curve away. That reads a wall whose facets run ALONG the chain edge right - a
//! cone's rim, a cylinder - but on a bore drilled at a RAKE the mouth's chords run round an ellipse
//! while the wall's facets stay ruled down the drill's axis, so the section leaves the facet's SPAN
//! without ever leaving its PLANE (`SurfaceRakedBoreRollingBallTests`: 0 of 4040 stations). Rolled
//! anyway, the walkback's cut leaves no run for the strip to sew to and the build refuses.
//!
//! **So the question asked here is the ball's own**: place the plain ball where its foot really
//! meets the chain edge's facet and ask whether anything on the wall comes nearer than the radius.
//! **Rounds only** - a chamfer has no ball to stand clear.
//!
//! C#'s `OfThePlainBall(mesh, wallRegion, ..)` fetches the region's grid from its memo
//! (`SurfaceRegionFaceGrid.For`); here the caller hands in the grid it keeps (divergence 650), and
//! `None` stands for C#'s null mesh or region.

use vector_math::{cs_math, Vector3};

use super::{SurfaceRegionFaceGrid, SurfaceRunAcrossSectionStep};
use crate::topology::FeatureEdge;

/// C# `FlatEnough`: how far off a plane a point may stand per unit of its reach and still be taken
/// as on it - the sine of the angle at which the topology itself stops calling two faces coplanar
/// ([`FeatureEdge::COPLANAR_FACE_DOT`]), which is 0.51 degrees.
fn flat_enough() -> f64 {
    (1.0 - (FeatureEdge::COPLANAR_FACE_DOT * FeatureEdge::COPLANAR_FACE_DOT)).sqrt()
}

/// C# `OfThePlainBall`: whether the plain ball clears the wall, so that the roll should not run and
/// the edge is planned between two planes as it would be if nothing curved.
///
/// `grid` is the curved region the roll was about to walk, `steps` that walk (whose first step is the
/// chain edge's own facet), `along` the unit direction along the chain edge, `edge_start` a point of
/// both planes, `flat_normal` the outward unit normal of the side that does not curve, `size` the
/// fillet radius and `convex` which side the centre is on.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn of_the_plain_ball(
    grid: Option<&SurfaceRegionFaceGrid>,
    steps: &[SurfaceRunAcrossSectionStep],
    along: Vector3,
    edge_start: Vector3,
    flat_normal: Vector3,
    size: f64,
    convex: bool,
) -> bool {
    let Some(grid) = grid else {
        return false;
    };

    if steps.is_empty() || !(size > 0.0) {
        return false;
    }

    let wall_normal = steps[0].normal;

    if !(wall_normal.length_squared() > 0.0) || !(flat_normal.length_squared() > 0.0) {
        return false;
    }

    // The centre a radius from both planes, on the side the profile is drawn in: inside the material
    // on a convex edge, in the void on a concave one. Both normals are already square to the edge,
    // the edge lying in both faces, so the pair below is the whole solve.
    let offset = if convex { -size } else { size };
    let opening = 1.0 + flat_normal.dot(wall_normal);

    if !(opening.abs() > 1e-9) {
        return false;
    }

    let centre = edge_start + ((flat_normal + wall_normal) * (offset / opening));
    let foot = centre - (wall_normal * offset);

    let Some((from, to)) =
        try_the_stretch_the_foot_meets_its_facet_over(grid, &steps[0], foot, along)
    else {
        return false;
    };

    let on_the_mesh = grid.weld_distance();

    // Asked in the middle of that stretch and at both its ends: the middle is where the strip really
    // sits, and the ends are where a wall that closes in on the ball does so first.
    for at in [(from + to) / 2.0, from, to] {
        let standing = centre + (along * at);

        // Asked of the grid only as far out as the radius: anything further off cannot be inside the
        // ball, and on a region of thousands of faces looking no further is the whole of the cost.
        if grid.nearest_to(standing, size) < size - on_the_mesh {
            return false;
        }
    }

    true
}

/// C# `TryTheStretchTheFootMeetsItsFacetOver`: the stretch of the foot's own line that really lies
/// inside the chain edge's facet, as parameters along the edge from the foot itself (`(from, to)`).
/// C#'s unused `mesh` parameter is dropped; the grid carries the faces.
///
/// The facet is every face of the region lying in the first step's own plane - taken by the plane
/// rather than by flooding neighbours, because a facet split into triangles by a mesher is one plane
/// however its faces are joined. The line is infinite, so it meets a triangle exactly when that
/// triangle's three corners do not all stand on one side of it in the plane they share.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_the_stretch_the_foot_meets_its_facet_over(
    grid: &SurfaceRegionFaceGrid,
    own: &SurfaceRunAcrossSectionStep,
    foot: Vector3,
    along: Vector3,
) -> Option<(f64, f64)> {
    let on_the_plane = grid.weld_distance();
    let sideways = along.cross(own.normal);

    if !(sideways.length_squared() > 0.0) {
        return None;
    }

    let sideways = sideways.get_normal();
    let flat_enough = flat_enough();

    let mut found = false;
    let mut least = 0.0;
    let mut most = 0.0;

    let origin = own.from;
    let normal = own.normal;

    // Only the faces the foot's own line could cross: a face it does cross has its crossing point in
    // a cell the line goes through, so the grid hands back every one that could have counted.
    grid.each_face_the_line_could_cross(foot, along, |_face, a, b, c| {
        let corners = [a, b, c];
        let mut across = [0.0; 3];
        let mut down = [0.0; 3];

        for i in 0..3 {
            let reach = corners[i] - origin;

            // The ruler grows with how far the corner stands off, because a float mesh only holds a
            // plane to about an ulp of its own coordinates times that reach. Anything the topology
            // would call a CREASE is orders of magnitude further off.
            if reach.dot(normal).abs() > on_the_plane + (reach.length() * flat_enough) {
                return;
            }

            // Where each corner stands against the line, and how far along it, in the facet's plane.
            across[i] = (corners[i] - foot).dot(sideways);
            down[i] = (corners[i] - foot).dot(along);
        }

        for i in 0..3 {
            let next = (i + 1) % 3;

            if (across[i] > 0.0 && across[next] > 0.0) || (across[i] < 0.0 && across[next] < 0.0) {
                continue;
            }

            let span = across[i] - across[next];
            let at = if span.abs() > 1e-12 {
                down[i] + ((down[next] - down[i]) * (across[i] / span))
            } else {
                down[i]
            };

            if !found {
                found = true;
                least = at;
                most = at;
            } else {
                least = cs_math::min_f64(least, at);
                most = cs_math::max_f64(most, at);
            }
        }
    });

    found.then_some((least, most))
}
