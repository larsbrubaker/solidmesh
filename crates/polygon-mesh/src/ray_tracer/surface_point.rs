//! What a shading ray tracer gets back about one point on a surface: [`SurfacePoint`] (barycentric
//! weights, the reconstructed position, the geometric and interpolated shading normals, the base
//! color and texture coordinate, and the displacement a continuing ray starts by), the helper
//! that turns the normals to face an incoming ray ([`facing_normals`]), and the numbers behind
//! ray continuation.
//!
//! [`crate::ray_tracer::MeshSurface`] fills a `SurfacePoint` in mesh space from a face index and
//! a hit position; [`crate::ray_tracer::PlacedPart`] carries one into world space.
//!
//! **Why a continuing ray needs its own offset.** [`crate::ray_tracer::MinimalTriangle`] holds its
//! plane in `f32` and tests a ray against it in `f32`, so whether a point counts as in front of
//! the plane is only known to the plane's rounding: a few `f32` steps of the corners' size, wherever
//! on the plane the point is. A point closer to the plane than that can land on the wrong side and
//! report its own triangle as hit again. So the origin of a continuing ray moves off the surface
//! by a multiple of `f32::EPSILON` times the largest L1 magnitude of the face's corners, measured
//! in the space the triangle is tested in. The position it moves from is the point rebuilt from
//! the corners, on the face's own plane to `f64` precision, rather than the ray's reported hit,
//! which carries the `f32` error of the hit distance as well and would need an offset growing
//! with how far the ray travelled.

use solidmesh_math::{Color, Vector2, Vector3};

/// How many `f32` steps of the face's corner magnitude a continuing ray starts off its surface.
/// Measured over meshes of 1 to 1000 mm, near and far from the origin: 1 step still leaves some
/// rays hitting their own face and 2 steps leave none (`surface_hit_continuation_tests` prints the
/// sweep); the extra is margin, kept small so the offset stays far below any corner a part really
/// has.
pub(super) const SURFACE_OFFSET_F32_STEPS: f64 = 8.0;

/// The same in `f64` steps of the world position, for parts placed so far from the origin that
/// moving a ray into the part's space rounds at about the offset's size.
pub(super) const WORLD_OFFSET_F64_STEPS: f64 = 16.0;

/// The sum of a position's coordinates' sizes, what the `f32` plane test's rounding scales with.
pub(super) fn l1_magnitude(position: Vector3) -> f64 {
    position.x.abs() + position.y.abs() + position.z.abs()
}

/// How far a continuing ray must start off a face with `corners`, in the space of the corners.
pub(super) fn surface_offset(corners: [Vector3; 3]) -> f64 {
    let largest = corners.iter().fold(0.0_f64, |largest, &corner| {
        largest.max(l1_magnitude(corner))
    });
    largest * f64::from(f32::EPSILON) * SURFACE_OFFSET_F32_STEPS
}

/// The barycentric weights of `position` on the triangle `corners`, or `None` when a corner or
/// the position is not a finite number.
///
/// The weights come from signed sub-triangle areas, which stay accurate on slivers because the
/// corners are `f32` values whose differences are exact in `f64`. A point the area ratios place
/// outside the triangle, or one a sliver's rounding has flung along its length, instead takes the
/// nearest point on the triangle's edges, so the weights are always within the triangle and the
/// point they rebuild is the closest one to `position` the triangle has. Ties keep the interior
/// answer, then the edges in order, so a given input always gives the same weights.
pub(super) fn barycentric_at(corners: [Vector3; 3], position: Vector3) -> Option<[f64; 3]> {
    if !position.is_valid() || corners.iter().any(|corner| !corner.is_valid()) {
        return None;
    }

    let [c0, c1, c2] = corners;
    let normal = (c1 - c0).cross(c2 - c0);
    let normal_squared = normal.length_squared();

    let rebuilt_distance = |weights: [f64; 3]| {
        (c0 * weights[0] + c1 * weights[1] + c2 * weights[2] - position).length_squared()
    };

    let mut best: Option<([f64; 3], f64)> = None;
    if normal_squared > 0.0 && normal_squared.is_finite() {
        let w0 = normal.dot((c1 - position).cross(c2 - position)) / normal_squared;
        let w1 = normal.dot((c2 - position).cross(c0 - position)) / normal_squared;
        let w2 = normal.dot((c0 - position).cross(c1 - position)) / normal_squared;
        let sum = w0 + w1 + w2;
        if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 && sum > 0.0 && sum.is_finite() {
            let weights = [w0 / sum, w1 / sum, w2 / sum];
            let distance = rebuilt_distance(weights);
            if distance.is_finite() {
                best = Some((weights, distance));
            }
        }
    }

    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        let edge = corners[j] - corners[i];
        let edge_squared = edge.length_squared();
        let t = if edge_squared > 0.0 {
            ((position - corners[i]).dot(edge) / edge_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let mut weights = [0.0; 3];
        weights[i] = 1.0 - t;
        weights[j] = t;
        let distance = rebuilt_distance(weights);
        if distance.is_finite() && best.is_none_or(|(_, nearest)| distance < nearest) {
            best = Some((weights, distance));
        }
    }

    best.map(|(weights, _)| weights)
}

/// One point on a surface, in mesh space or (from a placed part) world space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePoint {
    /// The face this point is on.
    pub face_index: i32,
    /// The weight of each face corner, in `[0, 1]` and summing to 1.
    pub barycentric: [f64; 3],
    /// The point rebuilt from the face's corners, on the face's own plane.
    pub position: Vector3,
    /// The unit normal of the face itself, pointing out of the side the face is wound on.
    pub geometric_normal: Vector3,
    /// The unit normal for shading: the corner normals, interpolated.
    pub shading_normal: Vector3,
    /// The face's color: [`crate::Mesh::UNCOLORED_FACE`] when the mesh has no per-face colors (a
    /// white face is one the object's own color paints) and
    /// [`crate::Mesh::UNKNOWN_FACE_COLOR`] when its color list is shorter than its face list.
    pub base_color: Color,
    /// The texture coordinate at this point, when the face has a texture.
    pub texture_coordinate: Option<Vector2>,
    /// The displacement from `position` that a ray continuing above the face starts by; negated
    /// for one going below it (see the module header).
    pub ray_offset: Vector3,
}

impl SurfacePoint {
    /// The normals turned to face a ray arriving along `incoming_direction` (see
    /// [`facing_normals`]).
    pub fn facing(&self, incoming_direction: Vector3) -> FacingNormals {
        facing_normals(
            self.geometric_normal,
            self.shading_normal,
            incoming_direction,
        )
    }

    /// Where a ray leaving this point along `outgoing_direction` starts: off the surface on the
    /// side it is heading to, so it neither finds this face again nor starts behind a wall it
    /// should hit. A direction exactly in the face's plane counts as above it.
    ///
    /// The offset grows with the size of the mesh's own coordinates, so it has a limit: a mesh
    /// baked around an L1 magnitude of 105,000 mm has an offset of about 0.1 mm and a ray from it
    /// can leak through a wall that thin. A part placed there through a matrix keeps its small
    /// mesh-space coordinates and does not.
    pub fn next_ray_origin(&self, outgoing_direction: Vector3) -> Vector3 {
        if outgoing_direction.dot(self.geometric_normal) >= 0.0 {
            self.position + self.ray_offset
        } else {
            self.position - self.ray_offset
        }
    }
}

/// A surface's normals turned toward the ray arriving at it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FacingNormals {
    /// The geometric normal, on the side the ray came from.
    pub geometric: Vector3,
    /// The shading normal, on that same side and never facing away from the ray.
    pub shading: Vector3,
    /// True when the ray arrived from behind the face (its normals were flipped).
    pub back_face: bool,
}

/// `geometric` and `shading` turned to face a ray arriving along `incoming_direction`: the
/// geometric normal flipped onto the ray's side, the shading normal flipped with it, and the
/// shading normal replaced by the geometric one where interpolation has bent it below the
/// surface or away from the ray. Bending past either makes a shader's cosine negative and turns
/// into black fringes at shadow lines, so the geometric normal, which is always right about
/// which side is which, stands in there.
pub fn facing_normals(
    geometric: Vector3,
    shading: Vector3,
    incoming_direction: Vector3,
) -> FacingNormals {
    let toward_ray = -incoming_direction;
    let back_face = geometric.dot(toward_ray) < 0.0;
    let (geometric, shading) = if back_face {
        (-geometric, -shading)
    } else {
        (geometric, shading)
    };
    let shading = if shading.dot(geometric) > 0.0 && shading.dot(toward_ray) > 0.0 {
        shading
    } else {
        geometric
    };
    FacingNormals {
        geometric,
        shading,
        back_face,
    }
}
