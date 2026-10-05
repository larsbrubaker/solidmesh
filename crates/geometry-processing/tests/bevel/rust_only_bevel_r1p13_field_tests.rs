//! Rust-only checks of the bevel core's R1.P13 field and room readings: the foreign-edge tree and
//! regions, the growing runs, end clips and the vertex reading with its field kinds, mesh pieces,
//! the in-turn ends, the rail pairing, the split record's text, whole-end runs, ray reaches,
//! corner pieces past a free end and the pair room's corner runs, pieces and end faces. The
//! switches and meters are in `rust_only_bevel_r1p13_switch_tests.rs`.

use std::rc::Rc;

use indexmap::IndexMap;
use vector_math::Vector3;

#[test]
fn rust_only_the_foreign_edge_tree_splits_at_the_longest_axis_median() {
    use geometry_processing::bevel::level_line::{
        foreign_edge_room, ForeignApproach, ForeignEdgeIndex, ForeignEdgeIndexNode,
    };
    // Six unit edges standing along x at x = 5, 4, 3, 2, 1, 0 (edge i at x = 5 - i).
    let mut ends = Vec::new();
    for i in 0..6 {
        let x = 5.0 - i as f64;
        ends.push(Vector3::new(x, 0.0, 0.0));
        ends.push(Vector3::new(x, 1.0, 0.0));
    }
    let mut order: Vec<i32> = (0..6).collect();
    let mut built = Vec::new();
    let root = ForeignEdgeIndex::add(&mut built, &ends, &mut order, 0, 6);
    assert_eq!(root, 0);
    assert_eq!(built.len(), 3);
    assert_eq!(order, vec![5, 4, 3, 2, 1, 0]);
    assert_eq!(
        built[0],
        ForeignEdgeIndexNode::new(
            0,
            6,
            1,
            2,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(5.0, 1.0, 0.0)
        )
    );
    assert_eq!((built[1].start, built[1].count, built[1].left), (0, 3, -1));
    assert_eq!(built[1].max, Vector3::new(2.0, 1.0, 0.0));
    assert_eq!((built[2].start, built[2].count), (3, 3));

    let edges: Vec<(i32, i32)> = (0..6).map(|i| (2 * i, 2 * i + 1)).collect();
    let index = ForeignEdgeIndex::new(edges.clone(), ends, built);
    assert_eq!(index.count(), 6);
    assert_eq!(index.edges(), &edges[..]);
    let all = foreign_edge_room::every_edge(std::slice::from_ref(&index), None);
    assert_eq!(all.len(), 6);
    assert_eq!(all[0], (0, 1, 0.0));
    let not_the_first = |edge: (i32, i32)| edge.0 == 0;
    let some = foreign_edge_room::every_edge(&[index], Some(&not_the_first));
    assert_eq!(some.len(), 5);
    assert_eq!(some[0], (2, 3, 0.0));
    let approach = ForeignApproach::new(1.5, vec![((0, 1), 1.5)], 1);
    assert_eq!(approach.read, 1);
}

#[test]
fn rust_only_foreign_edge_regions_read_each_chains_growth() {
    use std::cell::RefCell;
    use std::collections::HashSet;

    use geometry_processing::bevel::level_line::foreign_edge_regions::{chain_ends, open_chain_of};
    use geometry_processing::bevel::level_line::{
        ForeignEdgeRegions, ForeignEdgeRegionsChainGrowth, GrowingCreaseRuns, LevelLineMesh,
    };

    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    let runs = Rc::new(GrowingCreaseRuns::new(
        mesh.clone(),
        (0, 1),
        Rc::new(|p: Vector3| p.length()),
        None,
        None,
    ));
    assert_eq!(runs.chain_edges, vec![(0, 1)]);
    assert!((runs.is_own_side)(2) && !(runs.is_own_side)(3));
    let own: Rc<dyn Fn(i32) -> bool> = Rc::new(|tag| tag == 3);
    let legs = GrowingCreaseRuns::new(
        mesh.clone(),
        (0, 1),
        Rc::new(|p: Vector3| p.length()),
        Some(vec![(0, 1), (1, 2)]),
        Some(own),
    );
    assert_eq!(legs.chain_edges, vec![(0, 1), (1, 2)]);
    assert!((legs.is_own_side)(3));
    let mut neighbours = std::collections::HashMap::new();
    GrowingCreaseRuns::add(&mut neighbours, 4, 5);
    GrowingCreaseRuns::add(&mut neighbours, 4, 6);
    assert_eq!(neighbours[&4], vec![5, 6]);

    let mut growth = ForeignEdgeRegionsChainGrowth::new(
        runs.clone(),
        HashSet::from([0, 1]),
        HashSet::new(),
        false,
        true,
    );
    growth.set_aside.insert((2, 3));
    let several = ForeignEdgeRegions::new(
        mesh.clone(),
        Default::default(),
        Some(vec![Some(growth), None]),
    );
    assert!(several.set_aside_for(0, (2, 3)));
    assert!(!several.set_aside_for(1, (2, 3)));
    assert!(several.sets_aside_any(0));
    assert!(!several.sets_aside_any(1));
    assert_eq!(several.runs_out(0), (false, true));
    assert_eq!(several.runs_out(1), (true, true));
    assert!(several
        .growing_for(0)
        .is_some_and(|r| Rc::ptr_eq(&r, &runs)));
    assert!(several.growing_for(1).is_none());

    let mut one = ForeignEdgeRegions::new(mesh, Default::default(), None);
    assert!(one.growing_for(0).is_none());
    one.growing = Some(runs.clone());
    assert!(one.growing_for(5).is_some());
    assert_eq!(one.runs_out(0), (true, true));
    assert!(!one.set_aside_for(0, (2, 3)));

    let chains = vec![vec![(0, 1)], vec![], vec![(4, 5), (5, 6)]];
    assert_eq!(
        open_chain_of(Some(&chains), Some(&[false, false, false])),
        Some(&chains[0])
    );
    assert_eq!(
        open_chain_of(Some(&chains), Some(&[true, false, false])),
        Some(&chains[2])
    );
    assert_eq!(open_chain_of(None, Some(&[false])), None);
    assert_eq!(
        chain_ends(&HashSet::from([(4, 5), (5, 6)])),
        HashSet::from([4, 6])
    );
    assert!(chain_ends(&HashSet::from([(0, 1), (1, 2), (0, 2)])).is_empty());
}

#[test]
fn rust_only_end_clips_and_a_vertex_reading() {
    use std::cell::RefCell;

    use geometry_processing::bevel::level_line::{
        ChainField, EndRayBound, LevelLineMesh, ScalarField, VertexFieldReading, VertexSide,
        WorkError,
    };

    let points = [Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)];
    assert_eq!(ChainField::find_end_clips(&points, 4.0, (None, None)), None);
    let bound = EndRayBound {
        plane_normal: Vector3::new(2.0, 0.0, 0.0),
        ..EndRayBound::default()
    };
    let clips = ChainField::find_end_clips(&points, 4.0, (None, Some(bound))).expect("one clip");
    assert_eq!(clips.len(), 1);
    assert_eq!(clips[0].vertex, points[1]);
    assert_eq!(clips[0].normal, Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(clips[0].parameter, 4.0);
    let both = ChainField::find_end_clips(&points, 4.0, (Some(bound), Some(bound))).expect("two");
    assert_eq!((both[0].parameter, both[1].parameter), (0.0, 4.0));

    struct Height;
    impl ScalarField for Height {
        fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
            Ok(point.z)
        }
    }
    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    assert!(VertexFieldReading::new(mesh.clone(), Rc::new(Height), -1.0).is_err());
    assert!(VertexFieldReading::new(mesh.clone(), Rc::new(Height), f64::NAN).is_err());
    let reading = VertexFieldReading::new(mesh, Rc::new(Height), 0.5).expect("a length");
    assert_eq!(reading.snap, 0.5);
    assert_eq!(
        reading.field().value_at(Vector3::new(0.0, 0.0, 2.0)).ok(),
        Some(2.0)
    );
    assert!(reading.values.borrow().is_empty());
    assert_eq!(VertexFieldReading::side_for(0.5, 0.5), VertexSide::On);
    assert_eq!(VertexFieldReading::side_for(-0.6, 0.5), VertexSide::Inside);
    assert_eq!(VertexFieldReading::side_for(0.6, 0.5), VertexSide::Outside);
    assert_eq!(
        VertexFieldReading::side_for(f64::NAN, 0.5),
        VertexSide::Outside
    );
}

#[test]
fn rust_only_mesh_pieces_know_whether_they_have_a_closed_form() {
    use geometry_processing::bevel::level_line::ChainFieldMeshPiece;
    let a = Vector3::new(0.0, 0.0, 0.0);
    let b = Vector3::new(2.0, 0.0, 0.0);
    let c = Vector3::new(0.0, 2.0, 0.0);
    let segment = ChainFieldMeshPiece::segment(a, b);
    assert!(segment.has_closed_form() && !segment.is_triangle);
    assert_eq!(
        segment.closest_to(Vector3::new(1.0, 5.0, 0.0)),
        Vector3::new(1.0, 0.0, 0.0)
    );
    let triangle = ChainFieldMeshPiece::triangle(a, b, c);
    assert!(triangle.has_closed_form() && triangle.is_triangle);
    let custom = ChainFieldMeshPiece::custom(Rc::new(|_| Vector3::new(7.0, 7.0, 7.0)));
    assert!(!custom.has_closed_form());
    assert_eq!(custom.closest_to(a), Vector3::new(7.0, 7.0, 7.0));
}

#[test]
fn rust_only_a_chain_cut_short_by_an_earlier_band_starts_where_its_edge_enters_the_mesh() {
    use geometry_processing::bevel::level_line::band_gap_in_turn::{
        ends_on_the_mesh, nearest_on_the_edge, takes_open_chains_in_turn,
    };
    use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, GapLoop};

    // The chain 0 - 1 - 2 along x; vertex 0 was cut away, and the earlier cut left 3 at x = 0.4
    // on the first edge (and 4 off the line).
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.4, 0.0, 0.0),
        Vector3::new(0.2, 0.5, 0.0),
    ];
    let neighbours: IndexMap<i32, Vec<i32>> =
        IndexMap::from([(4, vec![3]), (1, vec![3, 2]), (2, vec![1]), (3, vec![1, 4])]);
    assert_eq!(nearest_on_the_edge(0, 1, &positions, &neighbours), 3);
    assert_eq!(nearest_on_the_edge(0, 0, &positions, &neighbours), -1);
    assert_eq!(
        ends_on_the_mesh(&[0, 1, 2], &positions, &neighbours),
        Some(vec![3, 1, 2])
    );
    assert_eq!(
        ends_on_the_mesh(&[1, 2], &positions, &neighbours),
        Some(vec![1, 2])
    );
    // With nothing on the first edge but its far end, the chain starts at that end.
    let no_cut: IndexMap<i32, Vec<i32>> = IndexMap::from([(1, vec![2]), (2, vec![1])]);
    assert_eq!(
        ends_on_the_mesh(&[0, 1, 2], &positions, &no_cut),
        Some(vec![1, 2])
    );
    assert_eq!(ends_on_the_mesh(&[0, 1], &positions, &no_cut), None);

    let open_loop = GapLoop::new(1, false, vec![0, 1], vec![0.0, 1.0], 0, false, None);
    let gap = BandGapResult {
        loops: vec![open_loop.clone()],
        ..BandGapResult::default()
    };
    assert!(takes_open_chains_in_turn(Some(&gap), &[false, false]));
    assert!(!takes_open_chains_in_turn(Some(&gap), &[false]));
    assert!(!takes_open_chains_in_turn(Some(&gap), &[false, true]));
    assert!(!takes_open_chains_in_turn(None, &[false, false]));
    let shared = BandGapResult {
        loops: vec![GapLoop::new(
            1,
            false,
            vec![0, 1],
            vec![0.0, 1.0],
            0,
            false,
            Some(vec![0, 1]),
        )],
        ..BandGapResult::default()
    };
    assert!(!takes_open_chains_in_turn(Some(&shared), &[false, false]));
    let refused = BandGapResult {
        refusal: BandGapRefusal::Size,
        loops: vec![open_loop],
        ..BandGapResult::default()
    };
    assert!(!takes_open_chains_in_turn(Some(&refused), &[false, false]));
}

#[test]
fn rust_only_the_rail_pairing_takes_the_cheaper_opposite_senses() {
    use geometry_processing::bevel::level_line::gap_fill::try_pair;
    use geometry_processing::bevel::level_line::{GapFillRailSenses, ZipRail};
    let rail = |reversed: bool| {
        Some(Rc::new(ZipRail {
            vertices: vec![0, 1],
            parameters: vec![0.0, 1.0],
            reversed,
        }))
    };
    let both = GapFillRailSenses::new(rail(false), 0.5, rail(true), 0.1, -1);
    let forward_only = GapFillRailSenses::new(rail(false), 0.0, None, 0.0, 3);
    let backward_only = GapFillRailSenses::new(None, 0.0, rail(true), 0.0, 4);
    // Both ways round on both: left backward + right forward costs 0.6, the other 0.6 too - a tie
    // goes to left forward.
    let (left, right) = try_pair(&both, &both).expect("a pairing");
    assert!(!left.reversed && right.reversed);
    let cheap = GapFillRailSenses::new(rail(false), 0.0, rail(true), 0.0, -1);
    let (left, right) = try_pair(&both, &cheap).expect("a pairing");
    assert!(left.reversed && !right.reversed);
    assert!(try_pair(&forward_only, &forward_only).is_none());
    let (left, right) = try_pair(&forward_only, &backward_only).expect("a pairing");
    assert!(!left.reversed && right.reversed);
}

#[test]
fn rust_only_a_split_record_describes_itself_in_one_line() {
    use geometry_processing::bevel::level_line::{
        LevelLineRule, PointPlace, SplitMeterSplitKind, SplitMeterSplitRecord,
    };
    let record = SplitMeterSplitRecord {
        point: Vector3::new(1.0, -2.5, 1.0 / 3.0),
        kind: SplitMeterSplitKind::OnEdge,
        rule: LevelLineRule::EdgeDip,
        triangle: 7,
        corner_a: 1,
        corner_b: 2,
        corner_c: 3,
        edge_from: 1,
        edge_to: 2,
        triangles_on_the_edge: 2,
        distance_ab: 0.0,
        distance_bc: 1.5e-7,
        distance_ca: 2.0,
        edge_tolerance: 1e-9,
        place: PointPlace::OnAnEdge,
        judged_from: 1,
        judged_to: 2,
        ..SplitMeterSplitRecord::default()
    };
    assert_eq!(
        record.describe(),
        "on-edge split of (1,2), 2 triangle(s) on it; rule EdgeDip; triangle 7 [1,2,3]; off its edges \
         AB 0.000E+000, BC 1.500E-007, CA 2.000E+000 against 1.000E-009 - stands on (1,2); \
         at (1.000000, -2.500000, 0.333333)"
    );
    let interior = SplitMeterSplitRecord {
        kind: SplitMeterSplitKind::Free,
        place: PointPlace::AtACorner,
        judged_from: 4,
        ..record
    };
    let text = interior.describe();
    assert!(
        text.starts_with("free vertex (fill); rule EdgeDip;"),
        "{text}"
    );
    assert!(text.contains(" - stands at corner 4; "), "{text}");
}

#[test]
fn rust_only_a_trimmed_run_gets_its_end_back_whole() {
    use geometry_processing::bevel::level_line::dying_crease_fill::is_one_loop_round_an_open_chain;
    use geometry_processing::bevel::level_line::end_face_fill::with_whole_end;
    use geometry_processing::bevel::level_line::{BandGapResult, GapLoop};

    let whole = GapLoop::new(
        1,
        false,
        vec![10, 11, 12, 13, 14],
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
        2,
        false,
        None,
    );
    let trimmed = GapLoop::new(
        1,
        false,
        vec![20, 11, 12],
        vec![0.5, 1.0, 2.0],
        2,
        false,
        None,
    );
    let tail = with_whole_end(&trimmed, &whole, true);
    assert_eq!(tail.vertices, vec![20, 11, 12, 13, 14]);
    assert_eq!(tail.parameters, vec![0.5, 1.0, 2.0, 3.0, 4.0]);
    assert_eq!(tail.chain, 2);
    // Its head (20) is not on the whole run, so the run comes back as it was.
    let head = with_whole_end(&trimmed, &whole, false);
    assert_eq!(head, trimmed);
    let off_the_run = GapLoop::new(1, false, vec![30, 31], vec![0.0, 1.0], 2, false, None);
    assert_eq!(with_whole_end(&off_the_run, &whole, true), off_the_run);
    let head_on = GapLoop::new(1, false, vec![12, 40], vec![2.0, 2.5], 2, false, None);
    assert_eq!(
        with_whole_end(&head_on, &whole, false).vertices,
        vec![10, 11, 12, 40]
    );

    assert!(!is_one_loop_round_an_open_chain(None));
    assert!(!is_one_loop_round_an_open_chain(Some(
        &BandGapResult::default()
    )));
}

#[test]
fn rust_only_ray_reaches_are_their_far_ends_per_segment() {
    use geometry_processing::bevel::level_line::{ChainField, ChainFieldExtension};
    assert_eq!(ChainField::reaches_of(None), None);
    let ray = ChainFieldExtension {
        origin: Vector3::new(1.0, 0.0, 0.0),
        direction: Vector3::new(0.0, 1.0, 0.0),
        reach: 2.0,
        ..ChainFieldExtension::default()
    };
    let rays = vec![None, Some(vec![ray, ray])];
    assert_eq!(
        ChainField::reaches_of(Some(&rays)),
        Some(vec![None, Some(vec![Vector3::new(1.0, 2.0, 0.0); 2])])
    );
}

#[test]
fn rust_only_a_corner_piece_keeps_its_rails_past_a_free_end() {
    use geometry_processing::bevel::level_line::corner_fill_free_ends::{rails_past_an_end, slice};
    use geometry_processing::bevel::level_line::CornerFillPiece;
    let piece = |parameters: Vec<f64>| CornerFillPiece {
        owner: 3,
        vertices: (10..10 + parameters.len() as i32).collect(),
        parameters,
    };
    // A run at the end inside the piece: down one rail, round the end, back up the other.
    let through = piece(vec![1.0, 0.5, 0.0, 0.0, 0.0, 0.5, 1.0]);
    let rails = rails_past_an_end(&through, 0.0, 1e-9);
    assert_eq!(rails.len(), 2);
    assert_eq!(rails[0].vertices, vec![10, 11, 12]);
    assert_eq!(rails[1].vertices, vec![14, 15, 16]);
    assert_eq!(rails[1].owner, 3);
    // A run at the piece's start keeps only its inner vertex.
    let starts = piece(vec![0.0, 0.0, 0.5, 1.0]);
    assert_eq!(
        rails_past_an_end(&starts, 0.0, 1e-9)[0].vertices,
        vec![11, 12, 13]
    );
    // Wholly at the end, or nowhere near it: as it was.
    let all = piece(vec![0.0, 0.0]);
    assert_eq!(rails_past_an_end(&all, 0.0, 1e-9), vec![all.clone()]);
    let none = piece(vec![0.5, 1.0]);
    assert_eq!(rails_past_an_end(&none, 0.0, 1e-9), vec![none.clone()]);
    // A single vertex at each end keeps the piece whole.
    let tips = piece(vec![0.0, 0.5, 0.0]);
    assert_eq!(rails_past_an_end(&tips, 0.0, 1e-9), vec![tips.clone()]);
    assert_eq!(slice(&through, 2, 4).parameters, vec![0.0, 0.0, 0.0]);
}

#[test]
fn rust_only_a_run_out_wrapper_is_read_through_to_its_pick() {
    use std::cell::RefCell;

    use geometry_processing::bevel::level_line::{
        band_group_fields_unwrapped, corner_cap_flat_ends, BandGroupFieldsWithRunOuts,
        ChainDistanceField, FieldKind, FieldMaximum, FieldMinimum, FieldSample, LevelLineMesh,
        PointField, ScalarField, VertexFieldReading, WorkError,
    };

    /// A test field that says it is a run-out wrapper, as `BandGroupFieldsWithRunOuts` will once
    /// it implements the field traits.
    struct Wrapped(BandGroupFieldsWithRunOuts);
    impl ScalarField for Wrapped {
        fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
            self.0.pick.value_at(point)
        }
        fn kind(&self) -> FieldKind {
            FieldKind::WithRunOuts
        }
        fn as_with_run_outs(&self) -> Option<&BandGroupFieldsWithRunOuts> {
            Some(&self.0)
        }
    }
    impl ChainDistanceField for Wrapped {
        fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
            self.0.pick.min_along_segment(a, b)
        }
        fn min_over_triangle(
            &self,
            a: Vector3,
            b: Vector3,
            c: Vector3,
        ) -> Result<FieldMinimum, WorkError> {
            self.0.pick.min_over_triangle(a, b, c)
        }
        fn max_along_segment(
            &self,
            a: Vector3,
            b: Vector3,
            tolerance: f64,
        ) -> Result<FieldMaximum, WorkError> {
            self.0.pick.max_along_segment(a, b, tolerance)
        }
        fn max_along_segment_at_least(
            &self,
            a: Vector3,
            b: Vector3,
            tolerance: f64,
            at_least: f64,
        ) -> Result<FieldMaximum, WorkError> {
            self.0
                .pick
                .max_along_segment_at_least(a, b, tolerance, at_least)
        }
        fn max_over_triangle(
            &self,
            a: Vector3,
            b: Vector3,
            c: Vector3,
            tolerance: f64,
        ) -> Result<FieldMaximum, WorkError> {
            self.0.pick.max_over_triangle(a, b, c, tolerance)
        }
        fn max_over_triangle_bounded(
            &self,
            a: Vector3,
            b: Vector3,
            c: Vector3,
            tolerance: f64,
            stop_above: f64,
            at_least: f64,
        ) -> Result<FieldMaximum, WorkError> {
            self.0
                .pick
                .max_over_triangle_bounded(a, b, c, tolerance, stop_above, at_least)
        }
    }

    let point: Rc<dyn ChainDistanceField> = Rc::new(PointField::sphere(Vector3::ZERO, 1.0));
    assert_eq!(point.kind(), FieldKind::Other);
    assert!(point.as_chain_field().is_none() && point.as_pick_field().is_none());
    let wrapper = BandGroupFieldsWithRunOuts::new(
        point.clone(),
        Rc::new(|_| Ok(FieldSample::default())),
        Some(Rc::new(|_, _| 0.0)),
        Vec::new(),
    );
    let wrapped: Rc<dyn ChainDistanceField> = Rc::new(Wrapped(wrapper));
    assert!(Rc::ptr_eq(&band_group_fields_unwrapped(&wrapped), &point));
    assert!(Rc::ptr_eq(&band_group_fields_unwrapped(&point), &point));

    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    let reading = VertexFieldReading::new(mesh, wrapped.clone(), 0.0).expect("a length");
    assert_eq!(
        reading.pick().value_at(Vector3::new(3.0, 0.0, 0.0)).ok(),
        Some(2.0)
    );
    corner_cap_flat_ends::set_use_flat_ends_override(Some(true));
    // A point field under the wrapper has no bounded ends: no known cuts.
    let cuts = reading.has_known_cuts();
    let lies = reading.edge_lies_in_a_known_cut(Vector3::ZERO, Vector3::UNIT_X);
    corner_cap_flat_ends::set_use_flat_ends_override(None);
    assert!(!cuts && !lies);
}

#[test]
fn rust_only_pair_room_corner_runs_pieces_and_end_faces() {
    use std::cell::RefCell;
    use std::collections::HashSet;

    use geometry_processing::bevel::level_line::band_room_pair_room::{
        edges_running_out_of_corners, ends_with_an_end_region, pieces_of_one_chain,
    };
    use geometry_processing::bevel::level_line::{
        EndRayBound, ForeignEdgeRegions, ForeignEdgeRegionsChainGrowth, GrowingCreaseRuns,
        LevelLineMesh,
    };
    use indexmap::IndexSet;

    // k runs along x from the corner 0; j leaves the corner up y, then turns off along x.
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(5.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(1.0, 2.0, 0.0),
    ];
    let skipped = edges_running_out_of_corners(
        &positions,
        &[0, 2, 3, 4],
        false,
        &[0, 1],
        false,
        &IndexSet::from([0]),
    );
    assert_eq!(skipped, IndexSet::from([0, 1]));
    assert!(edges_running_out_of_corners(
        &positions,
        &[0, 2, 3, 4],
        false,
        &[0, 1],
        false,
        &IndexSet::new()
    )
    .is_empty());

    let chains = vec![vec![0, 1, 2], vec![2, 3], vec![5, 6]];
    let closed = [false, false, false];
    assert_eq!(
        pieces_of_one_chain(&chains, &closed, &HashSet::from([2])),
        vec![vec![0, 1], vec![0, 1], vec![2]]
    );
    assert_eq!(
        pieces_of_one_chain(&chains, &closed, &HashSet::new()),
        vec![vec![0], vec![1], vec![2]]
    );

    let mesh = Rc::new(RefCell::new(LevelLineMesh::new()));
    let runs = Rc::new(GrowingCreaseRuns::new(
        mesh.clone(),
        (0, 1),
        Rc::new(|_| 0.0),
        None,
        None,
    ));
    let growth = |first: bool, last: bool| {
        Some(ForeignEdgeRegionsChainGrowth::new(
            runs.clone(),
            HashSet::new(),
            HashSet::new(),
            first,
            last,
        ))
    };
    let chain_edges = vec![vec![(0, 1), (1, 2)], vec![(7, 8)]];
    let regions = ForeignEdgeRegions::new(
        mesh.clone(),
        Default::default(),
        Some(vec![growth(false, true), growth(true, false)]),
    );
    assert_eq!(
        ends_with_an_end_region(&regions, &chain_edges, 0, 1, (None, None), (None, None)),
        Some((HashSet::from([0]), HashSet::from([8])))
    );
    // A same-sense bounded end stops on its face as an end region's does.
    let same = EndRayBound {
        same_sense: true,
        ..EndRayBound::default()
    };
    let one_chain = ForeignEdgeRegions::new(mesh, Default::default(), None);
    assert_eq!(
        ends_with_an_end_region(
            &one_chain,
            &chain_edges,
            0,
            1,
            (None, Some(same)),
            (Some(same), None)
        ),
        Some((HashSet::from([2]), HashSet::from([7])))
    );
    assert_eq!(
        ends_with_an_end_region(
            &one_chain,
            &chain_edges,
            0,
            1,
            (None, Some(same)),
            (None, None)
        ),
        None
    );
}
