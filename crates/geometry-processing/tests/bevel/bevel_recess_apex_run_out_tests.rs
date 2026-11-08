//! The pyramidal recess of "UpperLeftInner should bevel.mcx" as a fixture
//! (`pyramid_recess_test_meshes`): the part the planes describe and the part the fixture's own
//! fourteen vertices make are one solid, so a reading taken on either is a reading about the same
//! part.

use vector_math::Vector3;

use crate::mesh_measurements::signed_volume;
use crate::pyramid_recess_test_meshes as recess;

/// The fixture's own triangles ARE the solid the planes describe: same volume, the cube less the
/// recess, so a reading on either builder is a reading about the same part.
#[test]
fn the_fixtures_triangles_are_the_solid_the_planes_describe() {
    let from_the_planes = signed_volume(&recess::create(false, Vector3::ZERO));
    let from_the_fixture = signed_volume(&recess::create_from_the_fixtures_own_triangles(
        Vector3::ZERO,
        false,
    ));

    assert!(
        (from_the_planes - from_the_fixture).abs() <= 0.002,
        "the planes give {from_the_planes:.4} and the fixture's triangles {from_the_fixture:.4}"
    );
    assert!(
        (from_the_fixture - 6333.343).abs() <= 0.002,
        "the regression test measures the part off the operation's own source mesh at 6333.343 - \
         got {from_the_fixture:.4}"
    );
}
