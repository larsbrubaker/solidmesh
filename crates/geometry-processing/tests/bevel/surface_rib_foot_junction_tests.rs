//! THE TWO-CREASE JUNCTION ON A SQUARE RIB - the free foot
//! of the rib-on-plate fixture at (8, 5, 5), where the rib's own vertical CUT runs into the two BEADS
//! its walls make with the plate, all three picked and all three square to one another.
//!
//! Asked of the kernel rather than of the operation because that is where the answer can be read:
//! the part builder's own closure survey refuses the pass in words and publishes nothing, so a
//! Standard test can say THAT it did not close and this one can say WHERE. Built through the shared
//! `surface_test_build`.

use solidmesh::bevel::surface::surface_mesh_closure;
use solidmesh::topology::bevel_edge_group_resolver;
use solidmesh::topology::{BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::shell_count;
use crate::surface_test_build::{try_bevel, TryBevelOptions};
use crate::topology_test_meshes::create_rib_on_plate;

/// The rib's free foot, the vertex all three picked creases meet at.
fn the_foot() -> Vector3 {
    Vector3::new(8.0, 5.0, 5.0)
}

/// One cut and two beads at a square foot build ONE CLOSED SHELL.
#[test]
fn the_ribs_square_foot_closes_as_one_shell() {
    let part = create_rib_on_plate();

    let outcome = try_bevel(
        &part,
        &the_foots_three_creases(&part),
        1.0,
        8,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let mesh = match outcome.result {
        Ok(built) => built.mesh,
        Err(reason) => panic!(
            "a corner of one cut and two beads is the junction's own shape - instead {reason:?}"
        ),
    };

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "a surface that shares every seam vertex as it makes it closes - open at {open_at:?}"
    );

    assert!(mesh.is_manifold());
    assert_eq!(
        shell_count(&mesh),
        1,
        "the rib and its plate are one solid, and rewriting a corner of it cannot make two"
    );
}

/// The three creases meeting at the foot, each as the single-edge group a user's click makes: the
/// CUT up the rib's own corner, the BEAD across the rib's end and the BEAD along its side.
fn the_foots_three_creases(part: &Mesh) -> Vec<BevelEdgeGroup> {
    let features = FeatureEdgeSet::create(part);

    let at_the_foot: Vec<i32> = (0..features.count() as i32)
        .filter(|&index| {
            let edge = &features.edges()[index as usize];
            (edge.start - the_foot()).length() < 1e-6 || (edge.end - the_foot()).length() < 1e-6
        })
        .collect();

    assert_eq!(
        at_the_foot.len(),
        3,
        "the rib fixture has lost the foot this is about: {} creases meet there",
        at_the_foot.len()
    );

    at_the_foot
        .into_iter()
        .map(|index| {
            bevel_edge_group_resolver::create_from_edge(
                &features,
                index,
                BevelSelectionKind::Edge,
                FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            )
            .expect("a crease of the part")
        })
        .collect()
}
