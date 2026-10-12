//! Unit tests of `bevel_published_mesh`'s private improvement test (`IsAnImprovementOn`), which
//! needs private access: a volume that reads NaN (divergence 4956).

use polygon_mesh::solids::platonic_solids;
use vector_math::Vector3Float;

use super::is_an_improvement_on;

#[test]
fn rust_only_a_cube_is_an_improvement_on_itself() {
    let cube = platonic_solids::create_cube(2.0, 2.0, 2.0);

    assert!(is_an_improvement_on(&cube, &cube));
}

#[test]
fn rust_only_a_volume_that_reads_nan_is_no_improvement() {
    let cube = platonic_solids::create_cube(2.0, 2.0, 2.0);
    let mut poisoned = cube.clone();
    poisoned.vertices[0] = Vector3Float::new(f64::NAN, 1.0, 1.0);

    // C#'s Math.Sign threw on either of these; the cleanup is refused instead, so the mesh the
    // boolean gave is the one published.
    assert!(!is_an_improvement_on(&cube, &poisoned));
    assert!(!is_an_improvement_on(&poisoned, &cube));
    assert!(!is_an_improvement_on(&poisoned, &poisoned));
}
