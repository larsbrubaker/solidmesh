//! Rust-only checks of bevel-core package R1.P15's leaves: the point field's exact queries, the
//! room field's setbacks, the canonical maximum's tolerant offer and the tie-tolerant edge test,
//! the zip rail's canonical and unwrapped readings, and the corner strip's nearest rail vertex. The
//! whole-build tests also cover them.

use geometry_processing::bevel::level_line::bounded_maximum::not_shorter;
use geometry_processing::bevel::level_line::corner_rail_strip::nearest;
use geometry_processing::bevel::level_line::{
    BoundedMaximumCanonicalMaximum, ChainDistanceField, PointField, ScalarField, ZipRail,
};
use vector_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

#[test]
fn rust_only_a_point_field_answers_its_extremes_exactly() {
    let sphere = PointField::sphere(v(0.0, 0.0, 0.0), 1.0);
    assert_eq!(sphere.value_at(v(3.0, 4.0, 0.0)).ok(), Some(4.0));
    let least = sphere
        .min_along_segment(v(-2.0, 2.0, 0.0), v(2.0, 2.0, 0.0))
        .expect("uncharged");
    assert_eq!((least.value, least.point), (1.0, v(0.0, 2.0, 0.0)));
    let tri = sphere
        .min_over_triangle(v(-1.0, -1.0, 2.0), v(1.0, -1.0, 2.0), v(0.0, 1.0, 2.0))
        .expect("uncharged");
    assert_eq!((tri.value, tri.point), (1.0, v(0.0, 0.0, 2.0)));
    let most = sphere
        .max_over_triangle(v(1.0, 0.0, 0.0), v(5.0, 0.0, 0.0), v(0.0, 2.0, 0.0), 0.1)
        .expect("uncharged");
    assert_eq!((most.value, most.point), (4.0, v(5.0, 0.0, 0.0)));
    let floor = sphere
        .max_along_segment_at_least(v(1.0, 0.0, 0.0), v(2.0, 0.0, 0.0), 0.1, 9.0)
        .expect("uncharged");
    assert_eq!((floor.value, floor.point), (9.0, v(1.0, 0.0, 0.0)));

    let plane = PointField::plane(v(0.0, 0.0, 0.0), v(0.0, 0.0, -1.0), 0.5);
    assert_eq!(plane.value_at(v(7.0, 7.0, -2.0)).ok(), Some(1.5));
    let least = plane
        .min_along_segment(v(0.0, 0.0, 1.0), v(0.0, 0.0, -1.0))
        .expect("uncharged");
    assert_eq!((least.value, least.point), (-1.5, v(0.0, 0.0, 1.0)));
}

#[test]
fn rust_only_the_canonical_maximum_keeps_the_first_of_a_tie() {
    let mut best = BoundedMaximumCanonicalMaximum::new(f64::NEG_INFINITY, v(9.0, 9.0, 9.0));
    best.offer(1.0, v(1.0, 0.0, 0.0));
    best.offer(1.0 + 1e-12, v(2.0, 0.0, 0.0));
    assert_eq!(best.best_point, v(1.0, 0.0, 0.0));
    best.offer(1.1, v(3.0, 0.0, 0.0));
    assert_eq!((best.best, best.best_point), (1.1, v(3.0, 0.0, 0.0)));

    assert!(not_shorter(1.0, 1.0 + 1e-13));
    assert!(!not_shorter(1.0, 1.0 + 1e-9));
}

#[test]
fn rust_only_a_zip_rail_reads_canonically_and_past_its_wrap() {
    let rail = ZipRail {
        vertices: vec![4, 5, 6],
        parameters: vec![0.5, 1.5, 2.5],
        reversed: true,
    };
    assert_eq!(rail.count(), 3);
    let canonical = ZipRail::canonical(&rail, &[(0.4, 0.6), (1.5, 1.5), (2.0, 3.0)]);
    assert_eq!(canonical.parameters, vec![0.4, 1.5, 2.0]);
    assert!(canonical.reversed);
    assert_eq!(ZipRail::unwrapped(&rail, 1, 10.0, 0.25), 1.75);
    assert_eq!(ZipRail::unwrapped(&rail, 3, 10.0, 0.25), 10.75);

    let parameters = [0.0, 1.0, 2.0, 3.0];
    assert_eq!(nearest(&parameters, 1, 1.8, 1), 2);
    assert_eq!(nearest(&parameters, 1, 1.2, 1), 1);
    assert_eq!(nearest(&parameters, 3, 9.0, 1), 3);
}

#[test]
fn rust_only_the_bead_batch_withdraws_by_owner() {
    use geometry_processing::bevel::BevelBeadBatch;
    use polygon_mesh::solids::platonic_solids;
    use std::collections::HashSet;
    let cube = || platonic_solids::create_cube(1.0, 1.0, 1.0);
    let mut batch = BevelBeadBatch::default();
    batch.add_chain_bead(0, cube());
    batch.add_fill_patch(vec![0, 2], cube());
    batch.add_fill_patch(vec![0, 1], cube());
    assert_eq!(batch.fill_patches_start_at(), 1);
    batch.withdraw(&HashSet::from([1]));
    assert_eq!(batch.whole().len(), 2);
    assert_eq!(batch.fill_patches_start_at(), 1);
    batch.withdraw(&HashSet::from([0]));
    assert!(batch.whole().is_empty());
}

#[test]
fn rust_only_small_level_line_readings() {
    use geometry_processing::bevel::level_line::band_gap_reading::room_margin;
    use geometry_processing::bevel::level_line::consumed_to_the_rim::lies_on_a_rim;
    use geometry_processing::bevel::level_line::knife_end_overrun::on_the_plane;
    use geometry_processing::bevel::level_line::line_walk::in_triangle;
    use geometry_processing::bevel::level_line::rail_stations::pieces;
    use std::collections::HashMap;

    assert_eq!(room_margin(4.0, 1e-9), 5e-10);
    assert_eq!(room_margin(0.0, 1e-9), 0.0);
    let rims = [(v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0))];
    assert!(lies_on_a_rim(
        &rims,
        v(1.0, 1e-10, 0.0),
        v(9.0, 0.0, 0.0),
        1e-9
    ));
    assert!(!lies_on_a_rim(
        &rims,
        v(1.0, 1e-3, 0.0),
        v(9.0, 0.0, 0.0),
        1e-9
    ));
    assert!(on_the_plane(
        v(5.0, 3.0, 1e-7),
        v(0.0, 0.0, 0.0),
        v(0.0, 0.0, 1.0)
    ));
    assert!(!on_the_plane(
        v(0.5, 0.0, 1e-5),
        v(0.0, 0.0, 0.0),
        v(0.0, 0.0, 1.0)
    ));
    let (a, b, c) = (v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0));
    assert!(in_triangle(a, b, c, v(0.5, 0.5, 0.0)));
    assert!(!in_triangle(a, b, c, v(0.6, 0.6, 0.0)));
    let splits = HashMap::from([((1, 5), 3), ((3, 5), 4)]);
    assert_eq!(pieces(5, 1, &splits), vec![(5, 4), (4, 3), (3, 1)]);
}

#[test]
fn rust_only_the_last_r1p15_pieces_read_as_csharp() {
    use geometry_processing::bevel::level_line::band_refinement::{
        farthest_from_the_line3, nearest_to_the_line, span,
    };
    use geometry_processing::bevel::level_line::band_side_tagging::{
        both_sided_members_build, model_scale_of, outward_at,
    };
    use geometry_processing::bevel::level_line::corner_fill_pieces::trim_the_turn_back;
    use geometry_processing::bevel::level_line::strip_clip_valley::one_plane_run;
    use geometry_processing::bevel::level_line::virtual_rail_past_the_end::last_to_leave;
    use geometry_processing::bevel::level_line::{BandProfile, ChainField, CornerFillPiece};
    use geometry_processing::bevel::surface::surface_handover_leaves_a_strip::leaves_a_band;
    use geometry_processing::bevel::surface::{SurfaceBandEnd, SurfaceLine};
    use std::collections::HashSet;

    assert_eq!(
        span(v(0.0, 0.0, 0.0), v(3.0, 0.0, 0.0), v(0.0, 4.0, 0.0)),
        5.0
    );
    assert_eq!(nearest_to_the_line(0.05, -2.0, 1.0), 1.0);
    assert_eq!(farthest_from_the_line3(0.05, -2.0, 1.0, 0.5), 2.0);

    assert!(both_sided_members_build());
    let positions = [v(0.0, 0.0, 0.0), v(1e-6, 0.0, 0.0), v(5.0, 0.0, 0.0)];
    assert_eq!(model_scale_of(&positions), 5.0);
    assert_eq!(model_scale_of(&[]), 0.0);
    // The first step back is too short to read: the walk goes on to the next vertex.
    assert_eq!(
        outward_at(&positions, &[(0, 1), (1, 2)], true, 5.0),
        v(-1.0, 0.0, 0.0)
    );

    // A piece that runs back off the chain's start before climbing loses the turn-back.
    let field = ChainField {
        points: vec![v(0.0, 0.0, 0.0), v(4.0, 0.0, 0.0)],
        setbacks: vec![1.0, 1.0],
        parameter_at_point: vec![0.0, 4.0],
        nodes: Vec::new(),
        smooth_joints: None,
        sharp_turn_degrees: f64::INFINITY,
        end_rays: (false, false),
        end_ray_bounds: (None, None),
        is_closed: false,
        length: 4.0,
        min_setback: 1.0,
        max_setback: 1.0,
        extensions: None,
        end_clips: None,
        end_plane_reader: Default::default(),
        sharp_joints: Default::default(),
    };
    let mut piece = CornerFillPiece {
        owner: 0,
        vertices: vec![9, 8, 7, 6],
        parameters: vec![0.5, 0.0, 1.0, 2.0],
    };
    trim_the_turn_back(&field, &mut piece);
    assert_eq!(piece.vertices, vec![8, 7, 6]);

    let on: HashSet<i32> = HashSet::from([1, 2, 3, 4]);
    let planes = vec![
        (
            v(0.0, 0.0, 0.0),
            v(0.0, 0.0, 1.0),
            HashSet::from([1, 4, 2, 3]),
        ),
        (v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), HashSet::from([1, 4])),
    ];
    assert_eq!(
        one_plane_run(&[1, 2, 3, 4, 5], &on, &planes),
        Some(vec![1, 2, 3, 4])
    );

    // A chamfer row reaches behind the plane only as far as its ends.
    let behind = last_to_leave(
        v(0.0, 1.0, -1.0),
        v(0.0, -1.0, -2.0),
        v(0.0, 0.0, 0.0),
        v(0.0, 0.0, 1.0),
        v(0.0, 0.0, 1.0),
        BandProfile::Chamfer,
    );
    assert_eq!(behind, 2.0);

    let line = SurfaceLine {
        point: v(0.0, 0.0, 0.0),
        direction: v(1.0, 0.0, 0.0),
    };
    let end = |x: f64, inward: f64| SurfaceBandEnd {
        point: v(x, 0.0, 0.0),
        inward: v(inward, 0.0, 0.0),
    };
    assert!(leaves_a_band(&[end(1.0, 1.0), end(3.0, -1.0)], &line));
    assert!(!leaves_a_band(&[end(3.0, 1.0), end(1.0, -1.0)], &line));
}
