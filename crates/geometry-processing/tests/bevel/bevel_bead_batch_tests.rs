//! Ports `Tests/GeometryProcessingTests/BevelBeadBatchTests.cs`: what the second pass may take
//! back off the bead batch, and what it must take back ALONGSIDE a rebuilt bead - a corner FILL
//! goes when either bead it continues is rebuilt.
//!
//! C# tells the meshes apart by reference; Rust meshes are values, so each box here is a cube of
//! its own size, and "the same mesh" is read off its vertices.

use std::collections::HashSet;

use geometry_processing::bevel::bevel_bead_clip::Clippable;
use geometry_processing::bevel::BevelBeadBatch;
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

#[test]
fn a_chain_bead_is_withdrawn_when_its_own_group_is_rebuilt() {
    let mut batch = BevelBeadBatch::default();

    let mine = cube(1.0);
    let theirs = cube(2.0);

    batch.add_chain_bead(0, mine.clone());
    batch.add_chain_bead(1, theirs);
    batch.add_chain_bead_clippable(1, clippable());

    batch.withdraw(&HashSet::from([1]));

    assert_equivalent(
        batch.whole(),
        &[&mine],
        "only the rebuilt group's bead comes off, whichever list it was in",
    );
    assert_eq!(batch.clippable().len(), 0, "and its clippable one with it");
}

#[test]
fn a_corner_fill_is_withdrawn_when_either_of_the_beads_it_continues_is_rebuilt() {
    let mut batch = BevelBeadBatch::default();

    let kept_bead = cube(1.0);
    let fill_between_two_kept = cube(2.0);

    batch.add_chain_bead(0, kept_bead.clone());
    batch.add_fill_patch(vec![0, 2], fill_between_two_kept.clone());
    batch.add_fill_patch(vec![0, 1], cube(3.0));
    batch.add_fill_patch_clippable(vec![1, 2], clippable());

    batch.withdraw(&HashSet::from([1]));

    assert_equivalent(
        batch.whole(),
        &[&kept_bead, &fill_between_two_kept],
        "a fill goes when EITHER of the beads it adjoins was rebuilt, and stays when neither was",
    );
    assert_eq!(
        batch.clippable().len(),
        0,
        "and the same rule applies to a fill that brought its own void cone"
    );
    assert_eq!(
        batch.fill_patches_start_at(),
        1,
        "one chain bead then one surviving fill, so the union's split index follows the withdrawal"
    );
}

#[test]
fn nothing_is_withdrawn_when_no_group_was_rebuilt() {
    let mut batch = BevelBeadBatch::default();

    batch.add_chain_bead(0, cube(1.0));
    batch.add_fill_patch(vec![0, 1], cube(2.0));

    batch.withdraw(&HashSet::new());

    assert_eq!(batch.whole().len(), 2);
    assert_eq!(batch.fill_patches_start_at(), 1);
}

/// A solid of its own size - nothing here reads the geometry, only who each mesh belongs to.
fn cube(size: f64) -> Mesh {
    platonic_solids::create_cube_vector(Vector3::new(size, size, size))
}

fn clippable() -> Clippable {
    Clippable::new(cube(5.0), cube(6.0), None, None, None)
}

/// C#'s `IsEquivalentTo`: the same meshes in any order.
fn assert_equivalent(actual: &[Mesh], expected: &[&Mesh], because: &str) {
    assert_eq!(actual.len(), expected.len(), "{because}");
    for want in expected {
        assert!(
            actual.iter().any(|mesh| mesh.vertices == want.vertices),
            "{because}"
        );
    }
}
