//! Helpers over a mesh's f32 vertex list, as free functions over slices: the in-place transform
//! and the bounds `Mesh` caches.

use solidmesh_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float};

/// Transforms every f32 position in place (widened, transformed, narrowed).
pub(crate) fn transform_f32(list: &mut [Vector3Float], matrix: Matrix4X4) {
    for v in list.iter_mut() {
        *v = v.transform(matrix);
    }
}

/// The bounds of the f32 positions, widened; empty for an empty list.
pub(crate) fn bounds_f32(list: &[Vector3Float]) -> AxisAlignedBoundingBox {
    let mut bounds = AxisAlignedBoundingBox::empty();
    for &position in list {
        bounds.expand_to_include_point(Vector3::from_vector3_float(position));
    }
    bounds
}
