//! A band's strip cut back along the FACE BEYOND the region it landed on
//! ([`SurfaceStripTrimmedByTheFaceBeyond`]). Ports the fields of
//! `Bevel/Surface/SurfaceStripTrimmedByTheFaceBeyond.cs` and the corner beyond (three planes met);
//! the reading and its ring, rim and profile questions land with another package as further
//! `impl` blocks.

use vector_math::Vector3;

use super::{SurfaceEdgePlan, SurfaceVertexPool};

/// C# `SurfaceStripTrimmedByTheFaceBeyond`: which rails the face beyond's plane crosses inside the
/// edge's span, where each of them is cut, and the one point the rails it never crosses all
/// collapse to.
///
/// The shape this is for is a band that takes the WHOLE of its landing face
/// (`SurfaceBandRunsOffTheFaceBeyond`): the tangent rail then stands off the part over the edge's
/// own length, and what bounds the strip on that side is the plane the face runs off onto. Every
/// rail meets that plane at its own parameter along the edge - a staircase - and the curve those
/// crossings draw is the ellipse the plane cuts the profile's cylinder in. **Closed form, per
/// rail**: a rail is a line, so where it meets a plane is one division, which puts the crossings
/// exactly on the face beyond.
///
/// **The rails that never cross inside the span collapse to ONE point**, put where the surface
/// really ends: on the chord between the tangent rail and the first rail that IS crossed, clipped
/// at the plane. That point is minted ONCE and is the strip's, the section's on the face beyond,
/// and the end face's at the other end of the edge - three rims through one vertex. **The corner
/// beyond** the section turns down at is a vertex the part already has, looked up rather than
/// minted, and carried at the head of the profile as the section's DIE POINT
/// (`SurfaceEndCut.TryCut`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStripTrimmedByTheFaceBeyond {
    /// Which side of the strip ran off - 0 for rail zero's, 1 for the last rail's.
    pub side: i32,
    /// Which end of the edge the rails are cut back FROM.
    pub at_the_start: bool,
    /// The parameter of the end the trim does NOT stand at, which every rail reaches.
    pub far_param: f64,
    /// Where each rail meets the plane beyond, as a parameter along the edge.
    pub trimmed_at: Vec<f64>,
    /// The first rail, counted in from the run-off side, whose crossing stands inside the span. The
    /// rails before it are air over the whole edge and collapse to
    /// [`SurfaceStripTrimmedByTheFaceBeyond::shared`].
    pub first_kept: i32,
    /// The one point the collapsed rails stand at, minted once and shared by three rims.
    pub shared: i32,
    /// The part's own corner where the plane beyond, the end plane and the landing face meet.
    pub the_corner_beyond: i32,
    /// A point on the plane beyond.
    pub plane_point: Vector3,
    /// The outward normal of the plane beyond.
    pub plane_normal: Vector3,
    /// The seam the strip sews to on the run-off side, as `(vertex, param)`: the ONE shared vertex,
    /// standing at the end the trim does not cut back. A band that landed on its face has a run of
    /// tangent vertices there; a band that ran off it has a point, which is all the stitch needs -
    /// it reads the seam's first and last vertex, and here they are the same one.
    pub seam: Vec<(i32, f64)>,
}

impl SurfaceStripTrimmedByTheFaceBeyond {
    /// The part's own vertex where the plane beyond (`plane_point`, `plane_normal`), the plane the
    /// band ENDS on and the landing face on `side` (0 left, 1 right) all meet - the corner the
    /// section turns down at. Looked up in the pool rather than minted: it is a vertex of three
    /// faces the build never touches, and a second one a rounding away is a crack.
    pub fn try_the_corner_beyond(
        pool: &SurfaceVertexPool,
        edge: &SurfaceEdgePlan,
        side: i32,
        at_the_start: bool,
        plane_point: Vector3,
        plane_normal: Vector3,
    ) -> Option<i32> {
        let along = edge.edge.direction();
        let ends_at = if at_the_start { edge.to() } else { edge.from() };
        let landing = if side == 0 {
            edge.edge.left_normal()
        } else {
            edge.edge.right_normal()
        };

        let at = Self::try_three_planes(
            plane_point,
            plane_normal,
            ends_at,
            along,
            edge.from(),
            landing,
        )?;

        pool.try_find_near(at, pool.near_enough(), None)
    }

    /// Where three planes meet, by Cramer's rule - `None` where they do not meet at a point.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
    pub fn try_three_planes(
        point_a: Vector3,
        normal_a: Vector3,
        point_b: Vector3,
        normal_b: Vector3,
        point_c: Vector3,
        normal_c: Vector3,
    ) -> Option<Vector3> {
        let volume = normal_a.dot(normal_b.cross(normal_c));

        if !(volume.abs() > 1e-9) {
            return None;
        }

        Some(
            ((normal_b.cross(normal_c) * point_a.dot(normal_a))
                + (normal_c.cross(normal_a) * point_b.dot(normal_b))
                + (normal_a.cross(normal_b) * point_c.dot(normal_c)))
                / volume,
        )
    }
}
