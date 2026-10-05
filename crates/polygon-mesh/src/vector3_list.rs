//! Helpers over vertex lists: flat double arrays, in-place transforms and bounds. Ports
//! agg-sharp `PolygonMesh/Vector3List.cs` (`Vector3ListEx`, `Vector3FloatListEx`) as free
//! functions over slices.

use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector3, Vector3Float};

/// Positions read three doubles at a time (`ToVector3List`); a trailing partial triple is
/// ignored.
pub fn to_vector3_list(v: &[f64]) -> Vec<Vector3> {
    v.chunks_exact(3)
        .map(|c| Vector3::new(c[0], c[1], c[2]))
        .collect()
}

/// Positions read three doubles at a time and narrowed to f32 (`ToVector3FloatList`).
pub fn to_vector3_float_list(v: &[f64]) -> Vec<Vector3Float> {
    v.chunks_exact(3)
        .map(|c| Vector3Float::new(c[0], c[1], c[2]))
        .collect()
}

/// The positions as a flat `x, y, z, ...` array (`List<Vector3>.ToDoubleArray`).
pub fn to_double_array(list: &[Vector3]) -> Vec<f64> {
    list.iter().flat_map(|v| [v.x, v.y, v.z]).collect()
}

/// The positions transformed by `matrix` as a flat array (`List<Vector3Float>.ToDoubleArray`).
pub fn to_double_array_transformed(list: &[Vector3Float], matrix: Matrix4X4) -> Vec<f64> {
    list.iter()
        .flat_map(|v| {
            let t = v.transform(matrix);
            [f64::from(t.x), f64::from(t.y), f64::from(t.z)]
        })
        .collect()
}

/// Transforms every position in place.
pub fn transform_f64(list: &mut [Vector3], matrix: Matrix4X4) {
    for v in list.iter_mut() {
        *v = v.transform(matrix);
    }
}

/// Transforms every f32 position in place (widened, transformed, narrowed).
pub fn transform_f32(list: &mut [Vector3Float], matrix: Matrix4X4) {
    for v in list.iter_mut() {
        *v = v.transform(matrix);
    }
}

/// The bounds of the positions; empty for an empty list.
pub fn bounds_f64(list: &[Vector3]) -> AxisAlignedBoundingBox {
    let mut bounds = AxisAlignedBoundingBox::empty();
    for &position in list {
        bounds.expand_to_include_point(position);
    }
    bounds
}

/// The bounds of the f32 positions, widened; empty for an empty list.
pub fn bounds_f32(list: &[Vector3Float]) -> AxisAlignedBoundingBox {
    let mut bounds = AxisAlignedBoundingBox::empty();
    for &position in list {
        bounds.expand_to_include_point(Vector3::from_vector3_float(position));
    }
    bounds
}
