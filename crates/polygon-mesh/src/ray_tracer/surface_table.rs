//! [`MeshSurface`]: everything a shading ray tracer needs about a mesh's surface, built once from
//! a [`Mesh`] and then read from any thread (plain data, `Send + Sync`). It answers
//! [`MeshSurface::surface_point`]: for a hit on a face, the [`SurfacePoint`] with its barycentric
//! weights, normals, color and texture coordinate.
//!
//! **Smooth normals with sharp creases.** Each face corner gets its own normal: the unit sum of
//! the normals of the faces at that vertex that reach this corner's face across edges bent less
//! than the crease angle, each weighted by the corner's angle alone. Cutting a face into more
//! triangles around a vertex divides its angle among the pieces and leaves their sum alone, so
//! the same surface shades the same however it was tessellated, which a boolean changes all the
//! time. Faces are connected by edges rather than merely by the vertex, so one vertex with
//! several smoothing groups (a cylinder's rim, a box corner) gives each group's corners their
//! own normal. A vertex no other face uses gives its corner the face's own normal, which is
//! what unwelded triangle soup gets everywhere.
//!
//! Faces with no area or with non-finite corners contribute nothing and get the fallback normal
//! `+Z`, so every normal the table holds is finite and unit length. Faces are compared by the
//! vertex indices they share, in a fixed order, so the same mesh always gives the same table to
//! the bit.
//!
//! The table is built in `builder`, one bounded slice at a time; [`MeshSurface::new`] runs that
//! to the end in one call.

use solidmesh_math::{AxisAlignedBoundingBox, Color, Vector2, Vector2Float, Vector3, Vector3Float};

use super::surface_point::{barycentric_at, surface_offset, SurfacePoint};
use super::RayTracerError;
use crate::Mesh;

mod builder;

pub use builder::MeshSurfaceBuilder;

/// How many neighbouring faces a face is compared with across one edge. Two is a clean edge, a
/// few is a non-manifold one, and a boolean can leave a couple of dozen; every pair is compared
/// up to 257 faces on an edge. Past that the comparison would be quadratic in that count for no
/// visible gain.
const MAX_FACES_COMPARED_PER_EDGE: usize = 256;

/// How many `f32` steps of the mesh's size a tree's bounds may differ from the table's by and
/// still be the same mesh. The two are built from the same `f32` corners, so this only absorbs
/// rounding in how a caller carried them.
const TREE_BOUNDS_F32_STEPS: f64 = 8.0;

/// The normal given to a face that has none of its own.
const FALLBACK_NORMAL: Vector3 = Vector3::UNIT_Z;

/// The extent of a table's face corners, for checking that a tree was built from the same mesh.
#[derive(Debug, Clone, PartialEq)]
enum CornerBounds {
    /// The table has no faces, so it describes no tree.
    NoFaces,
    /// A face has a broken corner. A tree's box over such a face depends on which corner is the
    /// broken one, so its bounds prove nothing either way.
    Unknown,
    Known(AxisAlignedBoundingBox),
}

/// A mesh's surface data for shading.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshSurface {
    vertices: Vec<Vector3Float>,
    faces: Vec<[i32; 3]>,
    geometric_normals: Vec<Vector3>,
    corner_normals: Vec<[Vector3; 3]>,
    face_colors: Option<Vec<Color>>,
    /// Each face's corner texture coordinates; empty when no face has a texture.
    texture_corners: Vec<Option<[Vector2Float; 3]>>,
    corner_bounds: CornerBounds,
}

impl MeshSurface {
    /// The table for `mesh`, keeping edges bent by `crease_radians` or more sharp.
    ///
    /// A crease angle outside `[0, PI]` is a [`RayTracerError::InvalidArgument`].
    pub fn new(mesh: &Mesh, crease_radians: f64) -> Result<Self, RayTracerError> {
        let mut builder = MeshSurfaceBuilder::new(mesh, crease_radians)?;
        loop {
            if let Some(surface) = builder.step(usize::MAX) {
                return Ok(surface);
            }
        }
    }

    /// How many faces the table describes.
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Whether a tree with bounds `tree` can have been built from this table's mesh. A table with
    /// a broken corner cannot tell and accepts it; one with no faces describes no tree.
    pub(super) fn matches_tree_bounds(&self, tree: &AxisAlignedBoundingBox) -> bool {
        let table = match &self.corner_bounds {
            CornerBounds::NoFaces => return false,
            CornerBounds::Unknown => return true,
            CornerBounds::Known(table) => table,
        };
        let size = [table.min_xyz, table.max_xyz, tree.min_xyz, tree.max_xyz]
            .iter()
            .fold(0.0_f64, |size, corner| {
                size.max(corner.x.abs())
                    .max(corner.y.abs())
                    .max(corner.z.abs())
            });
        let tolerance = size * f64::from(f32::EPSILON) * TREE_BOUNDS_F32_STEPS;
        // Written so a NaN in the tree's box compares as different.
        (0..3).all(|axis| {
            (table.min_xyz[axis] - tree.min_xyz[axis]).abs() <= tolerance
                && (table.max_xyz[axis] - tree.max_xyz[axis]).abs() <= tolerance
        })
    }

    /// The point on face `face_index` nearest `hit_position`, or
    /// [`RayTracerError::InvalidArgument`] when that face does not exist. `hit_position` is in
    /// the mesh's own space.
    pub fn surface_point(
        &self,
        face_index: i32,
        hit_position: Vector3,
    ) -> Result<SurfacePoint, RayTracerError> {
        let face = usize::try_from(face_index)
            .ok()
            .filter(|&index| index < self.faces.len())
            .ok_or(RayTracerError::InvalidArgument(
                "the face index is not a face of this mesh",
            ))?;

        let geometric_normal = self.geometric_normals[face];
        let located = self.corners(face).and_then(|corners| {
            barycentric_at(corners, hit_position).map(|weights| (corners, weights))
        });
        let (barycentric, position, ray_offset) = match located {
            Some((corners, w)) => (
                w,
                corners[0] * w[0] + corners[1] * w[1] + corners[2] * w[2],
                geometric_normal * surface_offset(corners),
            ),
            None => {
                // A face with broken corners has no surface to place the point on; report where
                // the ray said it was so the tracer can carry on from there.
                let position = if hit_position.is_valid() {
                    hit_position
                } else {
                    Vector3::ZERO
                };
                let third = 1.0 / 3.0;
                ([third; 3], position, Vector3::ZERO)
            }
        };

        let [n0, n1, n2] = self.corner_normals[face];
        let shading_normal = unit_or(
            n0 * barycentric[0] + n1 * barycentric[1] + n2 * barycentric[2],
            geometric_normal,
        );

        Ok(SurfacePoint {
            face_index,
            barycentric,
            position,
            geometric_normal,
            shading_normal,
            base_color: self.base_color(face),
            texture_coordinate: self.texture_coordinate(face, barycentric),
            ray_offset,
        })
    }

    /// The corner positions of `face` in `f64`, `None` when an index or a coordinate is bad.
    fn corners(&self, face: usize) -> Option<[Vector3; 3]> {
        let [v0, v1, v2] = self.faces[face];
        let corner = |index: i32| {
            let position = *self.vertices.get(usize::try_from(index).ok()?)?;
            let position = Vector3::from_vector3_float(position);
            position.is_valid().then_some(position)
        };
        Some([corner(v0)?, corner(v1)?, corner(v2)?])
    }

    fn base_color(&self, face: usize) -> Color {
        match &self.face_colors {
            None => Mesh::UNCOLORED_FACE,
            Some(colors) => colors
                .get(face)
                .copied()
                .unwrap_or(Mesh::UNKNOWN_FACE_COLOR),
        }
    }

    fn texture_coordinate(&self, face: usize, barycentric: [f64; 3]) -> Option<Vector2> {
        let corners = (*self.texture_corners.get(face)?)?;
        let uv = |corner: Vector2Float| Vector2::new(f64::from(corner.x), f64::from(corner.y));
        Some(
            uv(corners[0]) * barycentric[0]
                + uv(corners[1]) * barycentric[1]
                + uv(corners[2]) * barycentric[2],
        )
    }

    /// The angle of `face` at `corner`, 0 when the face's corners are unusable.
    fn corner_angle(&self, face: usize, corner: usize) -> f64 {
        let Some(corners) = self.corners(face) else {
            return 0.0;
        };
        let at = corners[corner];
        let to_next = corners[(corner + 1) % 3] - at;
        let to_previous = corners[(corner + 2) % 3] - at;
        libm::atan2(
            to_next.cross(to_previous).length(),
            to_next.dot(to_previous),
        )
    }
}

/// The other two vertices of `face`, from `corner`.
fn other_vertices(face: [i32; 3], corner: usize) -> (i32, i32) {
    (face[(corner + 1) % 3], face[(corner + 2) % 3])
}

/// `vector` made unit length, or `fallback` when it has no usable length.
pub(super) fn unit_or(vector: Vector3, fallback: Vector3) -> Vector3 {
    let length = vector.length();
    if length > 0.0 && length.is_finite() {
        vector / length
    } else {
        fallback
    }
}
