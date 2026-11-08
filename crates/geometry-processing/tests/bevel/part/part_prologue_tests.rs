//! Pins of a part's prologue (`bevel::part::prologue::prepare`) driven on its own, not through a
//! whole build: a part with no mesh refuses every group by code, every group that resolved to
//! edges is offered to the surface construction, and the translation out to the operation is held
//! back from the working mesh and carried as the build's own matrix.

use std::sync::Arc;

use solidmesh::bevel::part::prologue::{prepare, PreparedPart, SharedPart};
use solidmesh::bevel::part::BevelPartRequest;
use solidmesh::bevel::{BevelAdvisoryCode, BevelGroupResult};
use solidmesh::topology::bevel_edge_group_resolver::create_groups_for_all_edges;
use solidmesh::topology::BevelEdgeGroup;
use solidmesh_math::Matrix4X4;
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

/// A 1 mm fillet of `groups` on `mesh`, carried out to the operation by `source_to_operation`.
fn fillet(
    mesh: Option<Mesh>,
    source_to_operation: Matrix4X4,
    groups: Vec<BevelEdgeGroup>,
) -> BevelPartRequest {
    BevelPartRequest {
        source_to_operation,
        ..BevelPartRequest::new(mesh.map(Arc::new), groups, 1.0, 4, true, 20.0)
    }
}

/// The cube's first straight crease as one Edge group.
fn one_edge(cube: &Mesh) -> Vec<BevelEdgeGroup> {
    let groups = create_groups_for_all_edges(cube, 20.0, None).expect("a cube has edges");
    assert_eq!(12, groups.len());
    vec![groups[0].clone()]
}

fn ready(request: &BevelPartRequest) -> SharedPart {
    match prepare(request) {
        PreparedPart::Ready(shared) => *shared,
        PreparedPart::Refused(_) => panic!("the part has a mesh"),
    }
}

/// Each row's refusal code, in order.
fn codes(rows: &[Option<BevelGroupResult>]) -> Vec<Option<BevelAdvisoryCode>> {
    rows.iter()
        .map(|row| {
            row.as_ref()
                .and_then(|row| row.failure.as_ref())
                .map(|advisory| advisory.code())
        })
        .collect()
}

/// A part with no mesh (none at all, or one with no vertices) refuses every one of its groups as
/// `PartHasNoMeshToBevel`, one row per group, and says nothing about the part as a whole.
#[test]
fn a_part_with_no_mesh_refuses_every_group_by_code() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = create_groups_for_all_edges(&cube, 20.0, None).expect("edges");
    let groups = vec![groups[0].clone(), groups[1].clone()];

    for mesh in [None, Some(Mesh::new())] {
        let request = fillet(mesh, Matrix4X4::IDENTITY, groups.clone());
        let PreparedPart::Refused(outcome) = prepare(&request) else {
            panic!("a part with no mesh is refused before anything is built");
        };

        assert!(outcome.mesh.is_none());
        assert!(outcome.failures.is_none());
        assert!(outcome.advisories.is_none());
        assert_eq!(
            vec![Some(BevelAdvisoryCode::PartHasNoMeshToBevel); 2],
            codes(&outcome.group_results)
        );
        for (i, row) in outcome.group_results.iter().enumerate() {
            let row = row.as_ref().expect("every group has its row");
            assert_eq!(i as i32, row.group_index);
            assert_eq!(0, row.edge_count);
        }
    }
}

#[test]
fn the_prologue_offers_every_group_that_resolved_to_edges() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = create_groups_for_all_edges(&cube, 20.0, None).expect("edges");
    let shared = ready(&fillet(Some(cube), Matrix4X4::IDENTITY, groups));

    assert_eq!(12, shared.resolutions.len());
    assert_eq!(vec![true; 12], shared.offered());
    assert_eq!(Matrix4X4::IDENTITY, shared.build_translation);
}

#[test]
fn the_prologue_holds_the_translation_back_from_the_working_mesh() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = one_edge(&cube);
    let moved = Matrix4X4::create_translation(125.0, -40.0, 3.0);
    let shared = ready(&fillet(Some(cube), moved, groups));

    // The working mesh stays round its own origin; the shift travels as the build's matrix.
    let bounds = shared.working_mesh.get_axis_aligned_bounding_box();
    assert!((bounds.min_xyz.x + 5.0).abs() < 1e-5);
    assert!((bounds.max_xyz.x - 5.0).abs() < 1e-5);
    assert_eq!(moved, shared.build_translation);
    assert_eq!(vec![true], shared.offered());
}
