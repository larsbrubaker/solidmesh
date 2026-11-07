//! Rust-only checks of R2.P05's crease readings on the working mesh: a growing end's stretch
//! along the end face, the end face's triangle taken into a flat corner hole, the growing crease
//! runs, the foreign edge index's build, a chain growth's reads and the filing of foreign creases
//! by tag pair. Split from `rust_only_bevel_r2p05_mesh_tests.rs` by responsibility.

use geometry_processing::bevel::level_line::{end_face_sense, BandProfile, LevelLineMesh};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// A unit square as two triangles sharing the diagonal 0-2, tagged `tags`.
fn square(tags: [i32; 2]) -> LevelLineMesh {
    LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&tags),
    )
    .expect("a square builds")
}

#[test]
fn rust_only_a_growing_end_reads_the_side_to_end_face_creases_from_it() {
    use std::collections::HashSet;

    // A floor (side 1) folded square into a wall (untagged, the end face) along 0-1, plus a
    // floor triangle beyond 1 that the wall does not reach.
    let mesh = LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(0.0, 0.0, 1.0),
            v(2.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (1, 0, 3), (1, 4, 2)],
        Some(&[1, 0, 1]),
    )
    .unwrap();
    let chain = [(0, 2)];

    let from_the_start = end_face_sense::growing_crease_stretch(&mesh, &chain, true, false, None);
    assert_eq!(from_the_start, HashSet::from([(0, 1)]));

    // The chain's far end (2) has no crease at it, and neither end grows: nothing.
    assert!(end_face_sense::growing_crease_stretch(&mesh, &chain, false, true, None).is_empty());
    assert!(end_face_sense::growing_crease_stretch(&mesh, &chain, false, false, None).is_empty());

    // Read with the chain's own sides named: the same.
    let own = |tag: i32| tag == 1;
    assert_eq!(
        end_face_sense::growing_crease_stretch(&mesh, &chain, true, false, Some(&own)),
        from_the_start
    );
}

#[test]
fn rust_only_an_end_face_triangle_holding_the_hole_is_taken_into_it() {
    use geometry_processing::bevel::level_line::corner_cap_flat_ends;

    // The hole c-a-p-b lies flat in z = 0, inside the end face's triangle b-c-a at its corner c;
    // the triangle a-b-d beyond keeps the far edge walled.
    let positions = [
        v(0.0, 0.0, 0.0),
        v(0.0, 10.0, 0.0),
        v(3.0, 3.0, 0.0),
        v(10.0, 0.0, 0.0),
        v(10.0, 10.0, 0.0),
    ];
    let make = || LevelLineMesh::from_triangles(&positions, &[(3, 0, 1), (1, 3, 4)], None).unwrap();

    let mut mesh = make();
    let mut changes = Vec::new();
    let taken = corner_cap_flat_ends::with_the_end_face_triangle_taken(
        &mut mesh,
        &[0, 1, 2, 3],
        Some(&mut changes),
    );
    assert_eq!(
        taken,
        vec![1, 2, 3],
        "the corner drops out; the fan's far path is the edge a-b"
    );
    assert!(mesh.is_removed(0));
    assert!(!mesh.is_removed(1));
    assert_eq!(changes, vec![(1, 0, false), (0, 3, false), (1, 3, true)]);

    // A hole that leaves the end face's triangle is handed back as it was.
    let mut mesh = make();
    let mut outside = LevelLineMesh::from_triangles(
        &[
            positions[0],
            positions[1],
            v(12.0, 12.0, 0.0),
            positions[3],
            positions[4],
        ],
        &[(3, 0, 1), (1, 3, 4)],
        None,
    )
    .unwrap();
    assert_eq!(
        corner_cap_flat_ends::with_the_end_face_triangle_taken(&mut outside, &[0, 1, 2, 3], None),
        vec![0, 1, 2, 3]
    );
    // Too short a loop.
    assert_eq!(
        corner_cap_flat_ends::with_the_end_face_triangle_taken(&mut mesh, &[0, 1, 3], None),
        vec![0, 1, 3]
    );
    assert!(!mesh.is_removed(0));
}

#[test]
fn rust_only_growing_crease_runs_read_sides_runs_and_rooms() {
    use std::cell::RefCell;
    use std::collections::HashSet;
    use std::rc::Rc;

    use geometry_processing::bevel::level_line::GrowingCreaseRuns;

    // The square fold of floor (side 1) and wall (side 2) along the chain edge 0-1.
    let mesh = LevelLineMesh::from_triangles(
        &[
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(0.0, 0.0, 1.0),
        ],
        &[(0, 1, 2), (1, 0, 3)],
        Some(&[1, 2]),
    )
    .unwrap();
    let runs = GrowingCreaseRuns::new(
        Rc::new(RefCell::new(mesh)),
        (0, 1),
        Rc::new(|p: Vector3| p.length()),
        None,
        None,
    );

    let (a0, along, n1, t1, n2) = runs.chain_sides((0, 1)).expect("one side of each tag");
    assert_eq!((a0, along), (v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)));
    assert!((n1 - v(0.0, 0.0, 1.0)).length() < 1e-12);
    assert!(
        (t1 - v(0.0, 1.0, 0.0)).length() < 1e-12,
        "into side 1, away from the edge"
    );
    assert!(
        (n2 - v(0.0, 1.0, 0.0)).length() < 1e-12,
        "the wall triangle 1-0-3 winds to +y"
    );
    assert!(
        runs.chain_sides((0, 2)).is_none(),
        "an open edge has one side"
    );

    let adjacency = GrowingCreaseRuns::adjacency(&runs.mesh.borrow()).unwrap();
    let mut around_0 = adjacency[&0].clone();
    around_0.sort();
    assert_eq!(around_0, vec![1, 2, 3]);

    // No crease here grows (no untagged end face), so a walk stops where it starts and a run's far
    // end is the farther of its own two.
    assert!(!runs.grows((0, 2)));
    assert_eq!(runs.far_end((0, 2)).unwrap(), 2);
    assert_eq!(runs.bounds_of((0, 2), &HashSet::new()).unwrap(), None);
    assert_eq!(
        runs.bounds.borrow().get(&(0, 2)),
        Some(&None),
        "a null bound is cached too"
    );

    // No ridge: nothing pierces, and a chamfer's room is never lowered.
    assert_eq!(runs.fillet_pierce_room(&[1.0], 8), f64::INFINITY);
    assert!(!runs.is_ridge_off_the_chains_crease((0, 2)));
    assert_eq!(
        GrowingCreaseRuns::least(3.0, &[runs], &[1.0], BandProfile::Fillet, 8),
        3.0
    );
    assert_eq!(
        GrowingCreaseRuns::least(3.0, &[], &[1.0], BandProfile::Chamfer, 8),
        3.0
    );
}

#[test]
fn rust_only_a_foreign_edge_index_holds_its_edges_in_tree_order() {
    use geometry_processing::bevel::level_line::ForeignEdgeIndex;

    let mesh = square([0, 0]);
    let edges = [(0, 1), (1, 2), (2, 3), (0, 3), (0, 2)];
    let index = ForeignEdgeIndex::over(&mesh, &edges);

    assert_eq!(index.count(), 5);
    let mut held = index.edges().to_vec();
    held.sort();
    let mut given = edges.to_vec();
    given.sort();
    assert_eq!(held, given);
    for (i, &(low, high)) in index.edges().iter().enumerate() {
        assert_eq!(index.ends[i * 2], mesh.position(low));
        assert_eq!(index.ends[(i * 2) + 1], mesh.position(high));
    }
    assert!(!index.nodes.is_empty());
    assert_eq!(ForeignEdgeIndex::over(&mesh, &[]).count(), 0);
}

#[test]
fn rust_only_a_chain_growth_sets_aside_its_own_sides_and_bounds_the_rest() {
    use std::cell::RefCell;
    use std::collections::HashSet;
    use std::rc::Rc;

    use geometry_processing::bevel::level_line::{
        ForeignEdgeRegionsChainGrowth, GrowingCreaseRuns,
    };

    let mesh = square([1, 0]);
    let runs = Rc::new(GrowingCreaseRuns::new(
        Rc::new(RefCell::new(square([1, 0]))),
        (0, 1),
        Rc::new(|p: Vector3| p.length()),
        None,
        None,
    ));
    let growth = |stretch: HashSet<(i32, i32)>| {
        ForeignEdgeRegionsChainGrowth::new(runs.clone(), HashSet::from([0, 1]), stretch, true, true)
    };
    // Triangle 1 is the end face (tag 3 of the end tags); across its diagonal is side 1.
    let end_tags = [0, 3];
    let chain_keys = HashSet::from([(0, 1)]);

    let mut own = growth(HashSet::new());
    own.read_end_regions(&mesh, &end_tags, &chain_keys, &|tag| tag == 1);
    assert_eq!(own.set_aside, HashSet::from([(0, 2)]));
    assert!(own.bounding.is_empty());

    let mut foreign = growth(HashSet::new());
    foreign.read_end_regions(&mesh, &end_tags, &chain_keys, &|_| false);
    assert!(foreign.set_aside.is_empty());
    assert_eq!(
        foreign.bounding,
        vec![(0, 2)],
        "flat, so not a knife: it bounds"
    );

    // A crease on the end face's stretch is set aside; one that bounds no growing run is not.
    let mut stretched = growth(HashSet::from([(2, 3)]));
    stretched.read((2, 3)).unwrap();
    stretched.read((0, 3)).unwrap();
    assert_eq!(stretched.set_aside, HashSet::from([(2, 3)]));
    assert!(stretched.bounding.is_empty());
}

#[test]
fn rust_only_foreign_creases_are_filed_by_tag_pair_and_indexed_once() {
    use std::cell::RefCell;
    use std::rc::Rc;

    use geometry_processing::bevel::level_line::ForeignEdgeRegions;
    use indexmap::IndexSet;

    let mesh = Rc::new(RefCell::new(square([1, 2])));
    let keys: IndexSet<(i32, i32)> = IndexSet::from([(0, 1)]);
    let regions = ForeignEdgeRegions::over(mesh, &keys, None, None, None, None).unwrap();

    assert_eq!(regions.by_tag_pair.len(), 1);
    assert_eq!(regions.by_tag_pair[&(1, 2)], vec![(0, 2)]);
    assert!(regions.per_chain.is_none());

    let seeds = |tag: i32| tag == 1;
    let found = regions.for_chain(&seeds);
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].count(), 1);
    assert!(
        Rc::ptr_eq(&found[0], &regions.for_chain(&seeds)[0]),
        "indexed once and kept"
    );
    // Seeds that do not tell the pair apart read nothing.
    assert!(regions.for_chain(&|_| true).is_empty());
}

#[test]
fn rust_only_a_chains_growing_runs_keep_the_seeds_delegate_live() {
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    use geometry_processing::bevel::level_line::foreign_edge_regions_filing::Seeds;
    use geometry_processing::bevel::level_line::ForeignEdgeRegions;

    let mesh = Rc::new(RefCell::new(square([1, 2])));
    let own_tag = Rc::new(Cell::new(1));
    let reading = own_tag.clone();
    let seeds: Seeds = Rc::new(move |tag, _chain| tag == reading.get());
    let end_tags = |_: i32| vec![0, 0];

    let per_chain =
        ForeignEdgeRegions::per_chain(&mesh, &[vec![(0, 1)]], &[false], &end_tags, Some(&seeds))
            .unwrap();
    let runs = &per_chain[0].as_ref().expect("an open chain grows").runs;
    assert!((runs.is_own_side)(1));
    assert!(!(runs.is_own_side)(7));

    // A tag the build adds later is read through the same closure, so the reading stays live.
    own_tag.set(7);
    assert!((runs.is_own_side)(7));
    assert!(!(runs.is_own_side)(1));
}
