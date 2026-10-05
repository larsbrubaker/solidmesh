//! Rust-only checks of the bevel core's R1.P09 package pieces no C# test class reaches alone:
//! a run's squared-off ends and their cap-clip retries, the interrupted crease's leaning, and the
//! valley-fold mitre.

use geometry_processing::bevel::bevel_cap_clip::attempts_for;
use geometry_processing::bevel::bevel_interrupted_crease::{crosses_the_run, Leaning};
use geometry_processing::bevel::bevel_valley_fold_mitre::try_mitre;
use geometry_processing::bevel::{BevelChainEnds, BevelEndCap};
use geometry_processing::topology::{ChainEdge, FeatureEdge};
use vector_math::Vector3;

fn edge_along_x() -> ChainEdge {
    let feature = FeatureEdge {
        start_vertex: 0,
        end_vertex: 1,
        start: Vector3::new(0.0, 0.0, 0.0),
        end: Vector3::new(10.0, 0.0, 0.0),
        direction: Vector3::new(1.0, 0.0, 0.0),
        ..Default::default()
    };
    ChainEdge::new(0, feature, false)
}

#[test]
fn rust_only_a_squared_run_is_capped_square_at_its_vertex_and_retried_uncapped() {
    let run = [edge_along_x()];
    let squared = BevelChainEnds::overshoot().squared_at(&run, true, true);

    // Square to the run, out of each end, through the end's own vertex.
    assert_eq!(squared.start.plane_normal, Vector3::new(-1.0, 0.0, 0.0));
    assert_eq!(squared.start.plane_distance, 0.0);
    assert_eq!(squared.end.plane_normal, Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(squared.end.plane_distance, 10.0);

    // Each capped end in turn is retried as an overshoot, owing its cap as a clip.
    let attempts = attempts_for(&squared, &run);
    assert_eq!(attempts.len(), 2);
    assert!(!attempts[0].0.start.is_capped() && attempts[0].0.end.is_capped());
    assert_eq!(attempts[0].1[0].at, Vector3::new(0.0, 0.0, 0.0));
    assert!(attempts[1].0.start.is_capped() && !attempts[1].0.end.is_capped());
    assert_eq!(attempts[1].1[0].at, Vector3::new(10.0, 0.0, 0.0));

    // Neither end asked: the ends come back as they were.
    assert_eq!(squared.squared_at(&run, false, false), squared);
    assert!(attempts_for(&BevelChainEnds::overshoot(), &run).is_empty());
}

#[test]
fn rust_only_an_end_closes_over_the_run_only_where_nothing_falls_away() {
    assert!(crosses_the_run(0.5) && !crosses_the_run(0.1));

    let mut closing = Leaning::default();
    closing.saw(-0.9);
    assert!(closing.closes_over_the_run(true));
    assert!(!closing.closes_over_the_run(false));

    closing.saw(0.9);
    assert!(
        !closing.closes_over_the_run(true),
        "a face falling away opens the end"
    );

    let mut grazing = Leaning::default();
    grazing.saw(-0.05);
    assert!(
        !grazing.closes_over_the_run(true),
        "a face along the run does not cross it"
    );
}

#[test]
fn rust_only_a_hairpin_valley_is_mitred_and_a_gentle_turn_is_not() {
    let position = Vector3::new(0.0, 0.0, 0.0);
    let outward = Vector3::new(1.0, 0.0, 0.0);
    // Both faces of the concave run face down and back, so the void bisector points -x, -z.
    let own0 = Vector3::new(-1.0, 0.0, 0.0);
    let own1 = Vector3::new(0.0, 0.0, -1.0);

    // A partner leaving the vertex at 120 degrees from the run: folded back far enough.
    let partner = FeatureEdge {
        start_vertex: 7,
        end_vertex: 8,
        direction: Vector3::new(-0.5, 0.75f64.sqrt(), 0.0),
        ..Default::default()
    };
    let cap: Option<BevelEndCap> = try_mitre(7, position, outward, own0, own1, &partner);
    assert!(cap.is_some_and(|cap| cap.is_capped()));

    // Leaving at 30 degrees it has hardly turned: no hairpin.
    let gentle = FeatureEdge {
        direction: Vector3::new(0.75f64.sqrt(), 0.5, 0.0),
        ..partner
    };
    assert!(try_mitre(7, position, outward, own0, own1, &gentle).is_none());
}

#[test]
fn rust_only_a_straight_run_carries_one_section_along_one_line() {
    use geometry_processing::bevel::bevel_chain_sweep_mesh::SweptSegment;
    use geometry_processing::bevel::bevel_straight_run::{run_to_ring, straight_run_from};
    use vector_math::Vector2;

    let segment = |from: f64, to: f64, direction: Vector3| {
        SweptSegment::new(
            direction * from,
            direction * to,
            direction,
            Vector3::new(0.0, 0.0, 1.0),
            direction.cross(Vector3::new(0.0, 0.0, 1.0)),
            vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0)],
            1.0,
        )
    };
    let x = Vector3::new(1.0, 0.0, 0.0);
    let swept = [
        segment(0.0, 2.0, x),
        segment(2.0, 5.0, x),
        segment(0.0, 4.0, Vector3::new(0.0, 1.0, 0.0)),
    ];

    // The first two carry straight on; the third turns a right angle.
    assert_eq!(straight_run_from(&swept, 0, true), 5.0);
    assert_eq!(straight_run_from(&swept, 1, false), 5.0);
    assert_eq!(straight_run_from(&swept, 2, false), 4.0);
    assert_eq!(run_to_ring(&swept, 2), 5.0);
}

#[test]
fn rust_only_a_joint_mitres_a_square_turn_and_refuses_a_doubling_back() {
    use geometry_processing::bevel::bevel_chain_sweep_mesh::SweptSegment;
    use geometry_processing::bevel::bevel_chain_sweep_rings::try_build_joint;
    use geometry_processing::bevel::BevelAdvisoryCode;
    use vector_math::Vector2;

    let up = Vector3::new(0.0, 0.0, 1.0);
    let segment = |from: Vector3, to: Vector3| {
        let direction = (to - from).get_normal();
        SweptSegment::new(
            from,
            to,
            direction,
            up,
            direction.cross(up),
            vec![Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)],
            1.0,
        )
    };
    let corner = Vector3::new(10.0, 0.0, 0.0);
    let mut before = segment(Vector3::new(0.0, 0.0, 0.0), corner);
    let mut after = segment(corner, Vector3::new(10.0, 10.0, 0.0));

    let ring = try_build_joint(&mut before, &mut after, 2).unwrap();

    // The profile's origin stands on the corner; its other point sits on the mitre, stretched
    // by 1 / cos(45) along the turn, so sqrt(2) from the corner.
    assert!((ring[0] - corner).length() < 1e-12);
    assert!(((ring[1] - corner).length() - 2f64.sqrt()).abs() < 1e-12);
    assert!(before.end_slide.is_some() && after.start_slide.is_some());

    let mut back = segment(corner, Vector3::new(0.0, 0.0, 0.0));
    let doubled = try_build_joint(&mut before, &mut back, 2).unwrap_err();
    assert_eq!(doubled.code(), BevelAdvisoryCode::ChainDoublesBack);
}
