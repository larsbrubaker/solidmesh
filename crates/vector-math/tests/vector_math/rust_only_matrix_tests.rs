// Checks for the matrix, quaternion and bounds types: the JSON shapes saved files use for
// Vector4/Matrix4X4/Quaternion/AxisAlignedBoundingBox, inversion, quaternion round trips, the
// -0.0 hash rule and the out-of-range index errors.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use vector_math::{AxisAlignedBoundingBox, MathError, Matrix4X4, Quaternion, Vector3, Vector4};

fn hash_of<T: Hash>(value: &T) -> u64 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);
    hasher.finish()
}

#[test]
fn rust_only_vector4_json_matches_newtonsoft_default() {
    let json = serde_json::to_string(&Vector4::new(3.0, 4.0, 0.0, 0.0)).unwrap();
    assert_eq!(
        json,
        "{\"X\":3.0,\"Y\":4.0,\"Z\":0.0,\"W\":0.0,\"Length\":5.0,\"LengthSquared\":25.0,\
         \"Xy\":{\"X\":3.0,\"Y\":4.0},\"Xyz\":{\"X\":3.0,\"Y\":4.0,\"Z\":0.0}}"
    );
    let back: Vector4 = serde_json::from_str(&json).unwrap();
    assert_eq!(back, Vector4::new(3.0, 4.0, 0.0, 0.0));
}

#[test]
fn rust_only_matrix_json_has_rows_and_translation() {
    let m = Matrix4X4::create_translation(1.0, 2.0, 3.0);
    let value: serde_json::Value = serde_json::to_value(m).unwrap();
    let keys: Vec<&String> = value.as_object().unwrap().keys().collect();
    assert_eq!(keys.len(), 5);
    assert_eq!(value["Translation"]["X"], 1.0);
    assert_eq!(value["Row3"]["Z"], 3.0);
    let back: Matrix4X4 = serde_json::from_value(value).unwrap();
    assert_eq!(back, m);
}

#[test]
fn rust_only_quaternion_and_aabb_json_round_trip() {
    let q = Quaternion::from_axis_angle(Vector3::UNIT_Z, 0.5);
    let json = serde_json::to_string(&q).unwrap();
    assert!(json.starts_with("{\"Xyz\":{"));
    let back: Quaternion = serde_json::from_str(&json).unwrap();
    assert_eq!(back, q);

    let bounds = AxisAlignedBoundingBox::from_coordinates(-1.0, -2.0, -3.0, 1.0, 2.0, 3.0);
    let json = serde_json::to_string(&bounds).unwrap();
    assert_eq!(
        json,
        "{\"MinXYZ\":{\"X\":-1.0,\"Y\":-2.0,\"Z\":-3.0},\"MaxXYZ\":{\"X\":1.0,\"Y\":2.0,\"Z\":3.0}}"
    );
    let back: AxisAlignedBoundingBox = serde_json::from_str(&json).unwrap();
    assert_eq!(back, bounds);
}

#[test]
fn rust_only_inverse_times_matrix_is_identity_and_singular_comes_back_unchanged() {
    let m = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_scale(2.0, 3.0, 4.0)
        * Matrix4X4::create_translation(5.0, -6.0, 7.0);
    assert!((m * m.inverted()).equals_within(Matrix4X4::IDENTITY, 1e-12));

    let singular = Matrix4X4::create_scale(1.0, 0.0, 1.0);
    assert_eq!(singular.inverted(), singular);
}

#[test]
fn rust_only_rotation_matrix_and_quaternion_agree() {
    let q = Quaternion::from_euler_angles(Vector3::new(0.4, -0.2, 1.1));
    let m = Matrix4X4::create_rotation_quaternion(q);
    let v = Vector3::new(1.0, 2.0, 3.0);
    // `create_rotation(axis, angle)` under row vectors turns the opposite way to
    // the quaternion sandwich `q v q^-1`, so the matrix of `q` moves points as `q`'s conjugate.
    assert!(v
        .transform(m)
        .equals_within(v.transform_by_quaternion(Quaternion::conjugated(q)), 1e-12));
    let back = m.get_rotation();
    let same = back.xyz().equals_within(q.xyz(), 1e-12) && (back.w - q.w).abs() < 1e-12;
    let negated = back.xyz().equals_within(-q.xyz(), 1e-12) && (back.w + q.w).abs() < 1e-12;
    assert!(same || negated);
}

#[test]
fn rust_only_negative_zero_hashes_like_zero() {
    assert_eq!(
        hash_of(&Vector3::new(-0.0, 0.0, 1.0)),
        hash_of(&Vector3::new(0.0, -0.0, 1.0))
    );
}

#[test]
fn rust_only_perspective_rejects_out_of_range_arguments() {
    assert_eq!(
        Matrix4X4::create_perspective_field_of_view(0.0, 1.0, 1.0, 2.0),
        Err(MathError::ArgumentOutOfRange("fovy"))
    );
    assert_eq!(
        Matrix4X4::create_perspective_field_of_view(1.0, 1.0, 2.0, 1.0),
        Err(MathError::ArgumentOutOfRange("zNear"))
    );
}
