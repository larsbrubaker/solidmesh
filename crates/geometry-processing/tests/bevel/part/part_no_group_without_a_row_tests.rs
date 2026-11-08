//! A group offered to the surface construction always comes back with a row: one a pass left
//! without any is refused by name, and a row a pass did write is left as it was.

use solidmesh::bevel::part::surface_part_builder::no_group_without_a_row::refuse_the_groups_with_no_row;
use solidmesh::bevel::{BevelAdvisoryCode, BevelGroupResult, BevelResultGroup};
use solidmesh::topology::bevel_edge_group_resolver::create_groups_for_all_edges;
use solidmesh::topology::BevelEdgeGroup;
use solidmesh_mesh::solids::platonic_solids::create_cube;

fn three_groups() -> Vec<BevelEdgeGroup> {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = create_groups_for_all_edges(&cube, 20.0, None).expect("a cube has edges");
    groups.into_iter().take(3).collect()
}

#[test]
fn a_group_left_without_a_row_is_refused_by_name() {
    let groups = three_groups();
    let mut results = vec![None, None, None];

    refuse_the_groups_with_no_row(&[0, 2], &mut results, &groups, |index| index as i32 + 5);

    let row = results[2].as_ref().expect("the group has its row");
    assert_eq!(row.group_index, 2);
    assert_eq!(row.edge_count, 7);
    assert!(row.failed());
    assert_eq!(
        row.failure.as_ref().expect("a refusal").code(),
        BevelAdvisoryCode::SurfaceConstructionNotBuilt
    );
    assert!(results[0].is_some());
    assert!(
        results[1].is_none(),
        "a group not offered is not this pass's to answer"
    );
}

#[test]
fn a_row_a_pass_wrote_is_left_as_it_was() {
    let groups = three_groups();
    let built = BevelGroupResult::new(
        0,
        BevelResultGroup::Edge(groups[0].clone()),
        4,
        None,
        Vec::new(),
    );
    let mut results = vec![Some(built), None, None];

    refuse_the_groups_with_no_row(&[0], &mut results, &groups, |_| 99);

    let row = results[0].as_ref().expect("the row stays");
    assert!(!row.failed());
    assert_eq!(row.edge_count, 4);
}
