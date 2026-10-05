//! Rust-only checks of the bevel core's R2.P02 rail and fill readings - rail jumps, sharp-joint
//! corners and carry weights, gap fill envelopes and merges, open rails and tie spans, mitre tie
//! points, the boundary walk and the corner fill's pieces - on the chain fields of
//! `chain_field_fixtures.rs`.

use std::rc::Rc;

use geometry_processing::bevel::level_line::{
    fillet_arc, segment_box_tree, ChainField, LevelLineMesh,
};
use vector_math::Vector3;

use crate::chain_field_fixtures::{l_chain, open_chain};

#[test]
fn rust_only_a_rail_is_recut_at_a_jump_nearest_the_other_start() {
    use geometry_processing::bevel::level_line::gap_fill::within_excursion;
    use geometry_processing::bevel::level_line::ZipRail;
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let rail = ZipRail {
        vertices: vec![0, 1, 2, 3],
        parameters: vec![0.0, 1.0, 6.0, 7.0],
        reversed: false,
    };
    // From 1 to 6 of chain parameter across one unit edge: a jump; the steps either side are not.
    assert!(ZipRail::is_a_jump(&mesh, &rail, 1, 8.0));
    assert!(!ZipRail::is_a_jump(&mesh, &rail, 0, 8.0));
    assert!(!ZipRail::is_a_jump(&mesh, &rail, 3, 8.0));
    let spans: Vec<(f64, f64)> = rail.parameters.iter().map(|&p| (p, p)).collect();
    let recut = ZipRail::start_nearest(&mesh, &rail, &spans, 6.0, 8.0, 0.5);
    assert_eq!(recut.vertices, vec![2, 3, 0, 1]);
    assert_eq!(recut.parameters, vec![-2.0, -1.0, 0.0, 1.0]);
    // Within the margin the rail is left as it was.
    assert_eq!(
        ZipRail::start_nearest(&mesh, &rail, &spans, 6.0, 8.0, 3.0),
        rail
    );

    let chain = l_chain();
    // At parameter 4 the setback is 2, so three setbacks past the slack is 6.
    assert!(within_excursion(4.0, -1.9, &chain, 0.1));
    assert!(!within_excursion(4.0, -2.2, &chain, 0.1));
}

#[test]
fn rust_only_a_sharp_joints_inside_corner_and_carry_weight() {
    use geometry_processing::bevel::level_line::sharp_joint_corners::the_inside_corner;
    let chain = ChainField {
        sharp_turn_degrees: 45.0,
        ..l_chain()
    };
    assert_eq!(chain.sharp_joints(), &[(4.0, 2.0)]);
    let (corner, plane) = the_inside_corner(&chain, 4.0, 1.0).expect("the L turns");
    assert!((corner - Vector3::new(3.0, 1.0, 0.0)).length() <= 1e-9);
    assert_eq!(plane, Vector3::new(0.0, 0.0, -1.0));
    assert!(the_inside_corner(&chain, 2.0, 1.0).is_none());

    let x = Vector3::new(1.0, 0.0, 0.0);
    // 3.5 from the joint of setback 2: smooth_step(0.75).
    assert!((fillet_arc::carry_weight(&chain, 0.0, 0.5, x, x) - 0.84375).abs() <= 1e-15);
    // Over the joint, none.
    assert_eq!(fillet_arc::carry_weight(&chain, 3.9, 4.1, x, x), 0.0);
    // A row turning by the whole threshold carries none; by half of it, all that the joint allows.
    let turned = Vector3::new(1.0, 1.0, 0.0).get_normal();
    assert!(fillet_arc::carry_weight(&chain, 0.0, 0.5, x, turned).abs() <= 1e-9);
    let half = Vector3::new(1.0, 22.5f64.to_radians().tan(), 0.0).get_normal();
    assert!((fillet_arc::carry_weight(&chain, 0.0, 0.5, x, half) - 0.84375).abs() <= 1e-9);
    // No joints: the gentle weight alone.
    assert_eq!(fillet_arc::carry_weight(&l_chain(), 0.0, 0.5, x, x), 1.0);
}

#[test]
fn rust_only_a_loop_reads_as_a_rail_both_ways_and_the_rails_zip() {
    use geometry_processing::bevel::level_line::gap_fill::{merge, senses, try_envelope};
    use geometry_processing::bevel::level_line::{GapLoop, ZipRail};
    let chain = l_chain();
    // Cut open after its wrap (3 -> 0) and already monotone.
    let (rail, cost, offending) = try_envelope(
        &[10, 11, 12, 13],
        &[2.0, 3.0, 0.0, 1.0],
        1,
        false,
        &chain,
        0.0,
    );
    assert_eq!(
        rail.as_ref().map(|r| r.vertices.clone()),
        Some(vec![12, 13, 10, 11])
    );
    assert_eq!((cost, offending), (0.0, -1));
    // A shallow excursion is flattened onto the envelope.
    let (rail, cost, _) = try_envelope(
        &[10, 11, 12, 13],
        &[0.0, 2.0, 1.5, 3.0],
        3,
        false,
        &chain,
        0.0,
    );
    assert_eq!(rail.map(|r| r.parameters), Some(vec![0.0, 2.0, 2.0, 3.0]));
    assert_eq!(cost, 0.5);
    // One deeper than three setbacks is refused, naming where.
    let (rail, _, offending) = try_envelope(
        &[10, 11, 12, 13],
        &[0.0, 10.0, 1.0, 11.0],
        3,
        false,
        &chain,
        0.0,
    );
    assert!(rail.is_none());
    assert_eq!(offending, 2);

    let gap_loop = GapLoop::new(
        1,
        true,
        vec![10, 11, 12, 13],
        vec![0.0, 1.0, 2.0, 3.0],
        0,
        false,
        None,
    );
    let both = senses(&gap_loop, &chain, 0.0);
    assert_eq!(
        both.forward.as_ref().map(|r| r.vertices.clone()),
        Some(vec![10, 11, 12, 13])
    );
    assert_eq!(both.forward_cost, 0.0);
    assert_eq!(
        both.backward.as_ref().map(|r| r.vertices.clone()),
        Some(vec![12, 11, 10, 13])
    );
    assert_eq!(both.backward_cost, 3.0);
    assert_eq!(both.offending, 10);

    // Two rails along the chain's straight leg, their vertices far apart so no step is a jump.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
    ];
    let left = ZipRail {
        vertices: vec![0, 1],
        parameters: vec![0.0, 2.0],
        reversed: false,
    };
    let right = ZipRail {
        vertices: vec![2, 3],
        parameters: vec![1.0, 3.0],
        reversed: true,
    };
    let steps = merge(&mesh, &chain, &left, &right, 4.0, 0.0).expect("no budget in force");
    assert_eq!(
        steps,
        vec![(0, 0, true), (1, 0, false), (1, 1, true), (2, 1, false)]
    );
}

#[test]
fn rust_only_open_rails_extend_past_the_ends_and_tie_vertices_span_the_jump() {
    use geometry_processing::bevel::level_line::open_gap_fill::{extended, try_rail};
    use geometry_processing::bevel::level_line::{open_gap_fill_stations, rail_stations};
    let chain = open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        ],
        vec![1.0, 1.0, 1.0],
    );
    // Past the start, by the angle about the end vertex times its setback.
    assert!(
        (extended(Vector3::new(-1.0, 0.0, 0.0), 0.0, &chain) + std::f64::consts::FRAC_PI_2).abs()
            <= 1e-12
    );
    assert_eq!(extended(Vector3::new(0.0, 1.0, 0.0), 0.0, &chain), 0.0);
    assert_eq!(extended(Vector3::new(2.0, 1.0, 0.0), 2.0, &chain), 2.0);

    // The inside corner (3, 1) of the L ties the two legs: read just off it toward (3.5, 2) the
    // chain reads the second leg, so the vertex stands for 3 up to 5.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![Vector3::new(3.0, 1.0, 0.0), Vector3::new(3.5, 2.0, 0.0)];
    let (read_from, read_to) = open_gap_fill_stations::on_the_edge(
        &chain,
        mesh.vertices[0],
        mesh.vertices[1],
        3.0,
        6.0,
        1e-9,
    )
    .expect("no budget");
    assert!((read_from - 5.0).abs() <= 1e-5);
    assert_eq!(read_to, 6.0);

    let rail = try_rail(&mesh, &chain, &[0, 1], &[3.0, 6.0], false, 0.0).expect("monotone");
    assert_eq!(rail.parameters, vec![3.0, 6.0]);
    let spans = open_gap_fill_stations::tie_spans(&mesh, &chain, &rail, 1e-9).expect("no budget");
    assert_eq!(spans[0].0, 3.0);
    assert!((spans[0].1 - 5.0).abs() <= 1e-5);
    assert_eq!(spans[1], (6.0, 6.0));
    // Read backward the rail is flattened within three setbacks, refused past them.
    let back = try_rail(&mesh, &chain, &[0, 1], &[3.0, 6.0], true, 0.0).expect("within reach");
    assert_eq!(back.parameters, vec![6.0, 6.0]);
    assert_eq!(
        try_rail(&mesh, &chain, &[0, 1], &[1.0, 6.0], true, 0.0).err(),
        Some(0)
    );

    let stations = rail_stations::tie_spans(&mesh, &chain, &[0, 1], &[3.0, 6.0], 1e-9, 0.0)
        .expect("no budget");
    assert!((stations[0].1 - 5.0).abs() <= 1e-4);
    assert_eq!(stations[1], (6.0, 6.0));
    let near = rail_stations::jump_near(
        &chain,
        mesh.vertices[0],
        mesh.vertices[1],
        3.0,
        6.0,
        1e-9,
        1,
        0.0,
    )
    .expect("no budget");
    assert!((near - 5.0).abs() <= 1e-4);
    assert_eq!(
        rail_stations::jump_near(
            &chain,
            mesh.vertices[1],
            mesh.vertices[0],
            6.0,
            3.0,
            1e-9,
            -1,
            0.0
        )
        .ok(),
        Some(6.0)
    );
}

#[test]
fn rust_only_two_chains_level_lines_meet_at_the_mitre_and_a_run_is_cut_back_to_the_rail() {
    use geometry_processing::bevel::level_line::corner_tie_points::try_mitre;
    use geometry_processing::bevel::level_line::virtual_rail_past_the_end::to_the_rail;
    use geometry_processing::bevel::level_line::GapLoop;
    let along_x = open_chain(
        vec![Vector3::new(-5.0, 0.0, 0.0), Vector3::new(5.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    );
    let along_y = open_chain(
        vec![Vector3::new(0.0, -5.0, 0.0), Vector3::new(0.0, 5.0, 0.0)],
        vec![1.0, 1.0],
    );
    let found = try_mitre(
        &along_x,
        &along_y,
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
    )
    .expect("no budget");
    assert!(found.is_some_and(|p| (p - Vector3::new(1.0, 1.0, 0.0)).length() <= 1e-9));
    let outside = try_mitre(
        &along_x,
        &along_y,
        Vector3::new(2.0, 2.0, 0.0),
        Vector3::new(3.0, 2.0, 0.0),
        Vector3::new(2.0, 3.0, 0.0),
    )
    .expect("no budget");
    assert!(outside.is_none());

    // A side run along the rail y = 1 that then climbs the crease past it.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(2.0, 1.5, 0.0),
        Vector3::new(3.0, 2.0, 0.0),
    ];
    let run = GapLoop::new(
        1,
        false,
        vec![0, 1, 2, 3],
        vec![5.0, 6.0, 7.0, 8.0],
        0,
        false,
        None,
    );
    let cut = to_the_rail(&mesh, &along_x, &run, true, 1.0, false)
        .expect("no budget")
        .expect("the run climbs off the rail");
    assert_eq!((cut.keep_count, cut.at_tail), (2, true));
    assert_eq!(cut.away, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(cut.parameter_at_p, 6.0);
    // A run that stays on the rail needs no cut, unless it may end at P.
    let on_rail = GapLoop::new(1, false, vec![0, 1], vec![5.0, 6.0], 0, false, None);
    assert!(to_the_rail(&mesh, &along_x, &on_rail, true, 1.0, false)
        .expect("no budget")
        .is_none());
    assert!(to_the_rail(&mesh, &along_x, &on_rail, true, 1.0, true)
        .expect("no budget")
        .is_some());
}

#[test]
fn rust_only_a_boundary_step_follows_round_the_fan() {
    use geometry_processing::bevel::level_line::gap_loop_walk::{follows, one_of};
    use geometry_processing::bevel::level_line::LevelLineTriangle;
    // A unit square as two triangles sharing the diagonal 0-2.
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 1),
        LevelLineTriangle::new(0, 2, 3, 1),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }

    assert_eq!(one_of(&mesh, 1, 2), Some(0));
    assert_eq!(one_of(&mesh, 2, 1), None);
    // From 1 -> 2 the boundary turns round vertex 2 across the diagonal to 2 -> 3.
    assert_eq!(follows(&mesh, (1, 2)), Some((2, 3)));
    assert_eq!(follows(&mesh, (3, 0)), Some((0, 1)));
    assert_eq!(follows(&mesh, (1, 0)), None);
}

#[test]
fn rust_only_a_corner_is_where_chains_end_together_and_ties_are_read_by_value() {
    use geometry_processing::bevel::level_line::corner_fill::{
        corner_points, fields_tie, takes, ties,
    };
    use geometry_processing::bevel::level_line::{
        BandGapRefusal, BandGapResult, BandProfile, CornerFillPiece,
    };
    let x_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let y_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 4.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let chains = vec![x_leg.clone(), y_leg.clone()];
    assert_eq!(corner_points(&chains), vec![Vector3::ZERO]);
    assert!(corner_points(&chains[..1]).is_empty());
    let gap = BandGapResult {
        chains: chains.clone(),
        ..BandGapResult::default()
    };
    assert!(takes(Some(&gap), BandProfile::Fillet));
    assert!(takes(Some(&gap), BandProfile::Chamfer));
    let refused = BandGapResult {
        refusal: BandGapRefusal::Size,
        ..gap
    };
    assert!(!takes(Some(&refused), BandProfile::Fillet));
    assert!(!takes(None, BandProfile::Fillet));

    assert!(fields_tie(&x_leg, &y_leg, Vector3::new(1.0, 1.0, 0.0)).expect("no budget"));
    assert!(!fields_tie(&x_leg, &y_leg, Vector3::new(1.0, 2.0, 0.0)).expect("no budget"));
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![Vector3::new(3.0, 3.0, 0.0), Vector3::new(1.0, 2.0, 0.0)];
    let neighbour = CornerFillPiece {
        owner: 1,
        vertices: vec![0, 1],
        parameters: vec![3.0, 2.0],
    };
    assert!(ties(&mesh, &chains, 0, &neighbour, true).expect("no budget"));
    assert!(!ties(&mesh, &chains, 0, &neighbour, false).expect("no budget"));
}

#[test]
fn rust_only_corner_pieces_are_trimmed_off_the_line_and_closed_round_the_seam() {
    use geometry_processing::bevel::level_line::corner_fill::close_round_the_seam;
    use geometry_processing::bevel::level_line::corner_fill_pieces::trim_off_the_line;
    use geometry_processing::bevel::level_line::CornerFillPiece;
    let x_leg = open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    );
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 3.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(2.0, 1.0, 0.0),
        Vector3::new(3.0, 0.5, 0.0),
        Vector3::new(4.0, 4.0, 0.0),
    ];
    let mut piece = CornerFillPiece {
        owner: 0,
        vertices: vec![0, 1, 2, 3, 4],
        parameters: vec![0.0, 1.0, 2.0, 3.0, 4.0],
    };
    trim_off_the_line(&mesh, &x_leg, &mut piece).expect("no budget");
    assert_eq!(piece.vertices, vec![1, 2, 3]);
    assert_eq!(piece.parameters, vec![1.0, 2.0, 3.0]);

    // A closed square chain of side 4, its seam at (0, 0), and a loop inside it.
    let points = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(4.0, 4.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
    ];
    let setbacks = vec![1.0; 4];
    let nodes = segment_box_tree::build(&points, &setbacks, 4, None);
    let square = ChainField {
        nodes,
        parameter_at_point: vec![0.0, 4.0, 8.0, 12.0, 16.0],
        is_closed: true,
        length: 16.0,
        ..open_chain(points.clone(), setbacks.clone())
    };
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(2.0, 1.0, 0.0),
        Vector3::new(3.0, 2.0, 0.0),
        Vector3::new(2.0, 3.0, 0.0),
        Vector3::new(1.0, 2.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
    ];
    let mut round = CornerFillPiece {
        owner: 0,
        vertices: vec![0, 1, 2, 3, 4],
        parameters: vec![2.0, 6.0, 10.0, 14.0, 1.0],
    };
    close_round_the_seam(&mesh, &square, &mut round);
    // It starts at the mitre vertex nearest the seam, read at 1, and ends there read across the
    // seam at 15.
    assert_eq!(round.vertices, vec![4, 0, 1, 2, 3, 4]);
    assert_eq!(round.parameters, vec![1.0, 2.0, 6.0, 10.0, 14.0, 15.0]);
}

#[test]
fn rust_only_a_side_between_two_leaning_walls_is_swallowed() {
    use geometry_processing::bevel::level_line::swallowed_side_runs::{
        back_facing_wall, growing_walls, is_swallowed, row_span, runs_if_both_swallowed,
        ties_merge_room, virtual_run,
    };
    use geometry_processing::bevel::level_line::{BandProfile, LevelLineTriangle};
    // A chain along x from 0 to 4 with both ends rayed; walls x + 3z = 0 and 4 - x + 3z = 0 lean
    // back over it and meet at (2, y, -2/3), two thirds of a setback below the chain; side 1 is
    // the triangle under the chain reaching that crease.
    let chain = ChainField {
        end_rays: (true, true),
        ..open_chain(
            vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
    };
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, -2.0 / 3.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(-3.0, 0.0, 1.0),
        Vector3::new(7.0, 0.0, 1.0),
        Vector3::new(4.0, 1.0, 0.0),
    ];
    for (i, t) in [
        LevelLineTriangle::new(0, 1, 2, 1),
        LevelLineTriangle::new(0, 3, 4, 0),
        LevelLineTriangle::new(1, 5, 6, 0),
    ]
    .into_iter()
    .enumerate()
    {
        mesh.triangles.push(t);
        mesh.removed.push(false);
        mesh.attach(i as i32, t);
    }

    let first = back_facing_wall(&mesh, &chain, false).expect("the first wall leans back");
    assert_eq!(first.0, Vector3::ZERO);
    assert!((first.1 - Vector3::new(1.0, 0.0, 3.0).get_normal()).length() <= 1e-12);
    assert!(growing_walls(&mesh, &chain).is_some());
    assert!(is_swallowed(&mesh, &chain, 1).expect("no budget"));
    assert!(!is_swallowed(&mesh, &chain, 2).expect("no budget"));
    // Without its end rays the chain's ends are no walls.
    assert!(growing_walls(
        &mesh,
        &open_chain(chain.points.clone(), chain.setbacks.clone())
    )
    .is_none());

    // The rail one setback under the chain crosses the last wall at 1 and the first at 3.
    let run = virtual_run(&mesh, &chain, 1).expect("a run");
    assert_eq!(run.len(), 2);
    assert!((run[0].1 - 1.0).abs() <= 1e-12 && (run[1].1 - 3.0).abs() <= 1e-12);
    assert!((run[0].0 - Vector3::new(1.0, 0.0, -1.0)).length() <= 1e-12);
    // Side 2 has no triangles, so no straight-chain row span and no pair of runs.
    assert!(row_span(&mesh, &chain, BandProfile::Fillet).is_none());
    assert!(runs_if_both_swallowed(&mesh, &chain, BandProfile::Fillet)
        .expect("no budget")
        .is_none());
    let field_at = |_size: f64| ChainField {
        end_rays: (true, true),
        ..open_chain(chain.points.clone(), chain.setbacks.clone())
    };
    assert_eq!(
        ties_merge_room(&mesh, &field_at, 1, BandProfile::Chamfer, 0.5).expect("no budget"),
        None
    );
}

#[test]
fn rust_only_a_knife_end_overrun_cuts_the_climbing_rail_back_off_the_end_plane() {
    use geometry_processing::bevel::level_line::knife_end_overrun::read;
    use geometry_processing::bevel::level_line::{CornerFillPiece, EndRayBound};
    // The chain along x from 0 to 4 rays on past its last end to the plane E: x + y = 4.
    let normal = Vector3::new(1.0, 1.0, 0.0).get_normal();
    let chain = ChainField {
        end_rays: (false, true),
        end_ray_bounds: (
            None,
            Some(EndRayBound {
                plane_normal: normal,
                ..EndRayBound::default()
            }),
        ),
        ..open_chain(
            vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
    };
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = vec![
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(2.0, 1.0, 0.0),
        Vector3::new(2.5, 1.5, 0.0),
        Vector3::new(3.0, 1.0, 0.0),
        Vector3::new(6.0, -1.0, 0.0),
    ];
    // The climbing rail's last two vertices stand on E, both read at the end; the falling rail
    // runs on past the end.
    let mut climbing = CornerFillPiece {
        owner: 0,
        vertices: vec![0, 1, 2, 3],
        parameters: vec![1.0, 2.0, 4.0, 4.0],
    };
    let falling = CornerFillPiece {
        owner: 0,
        vertices: vec![4],
        parameters: vec![5.0],
    };
    let end = read(&mesh, &chain, &mut climbing, &falling, 1e-9, true)
        .expect("no budget")
        .expect("the overrun applies");
    assert_eq!(climbing.vertices, vec![0, 1, 2]);
    assert!((climbing.parameters[2] - 2.5).abs() <= 1e-12);
    assert!(end.last);
    assert_eq!(end.vertex, Vector3::new(4.0, 0.0, 0.0));
    assert!((end.across - Vector3::new(0.0, 1.0, 0.0)).length() <= 1e-12);
    // Not at the first end, which has no ray.
    let mut again = climbing.clone();
    assert!(read(&mesh, &chain, &mut again, &falling, 1e-9, false)
        .expect("no budget")
        .is_none());
}

#[test]
fn rust_only_a_piece_round_a_free_end_splits_into_its_rails() {
    use geometry_processing::bevel::level_line::corner_fill_free_ends::split_round_a_free_end;
    use geometry_processing::bevel::level_line::CornerFillPiece;
    let x_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let y_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 4.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let chains = vec![x_leg, y_leg];
    // The x leg's end at (4, 0) is free; its start meets the y leg. A piece running down one rail,
    // round that end and back up the other splits into the two rails.
    let piece = CornerFillPiece {
        owner: 0,
        vertices: vec![10, 11, 12, 13, 14],
        parameters: vec![3.0, 4.0, 4.0, 4.0, 3.0],
    };
    let rails = split_round_a_free_end(&chains, &piece);
    assert_eq!(rails.len(), 2);
    assert_eq!(rails[0].vertices, vec![10, 11]);
    assert_eq!(rails[1].vertices, vec![13, 14]);
    // A run at the joined start is not split.
    let at_start = CornerFillPiece {
        owner: 0,
        vertices: vec![10, 11, 12],
        parameters: vec![1.0, 0.0, 1.0],
    };
    assert_eq!(
        split_round_a_free_end(&chains, &at_start),
        vec![at_start.clone()]
    );
}

#[test]
fn rust_only_a_row_end_on_a_joints_seam_is_read_on_its_partners_leg() {
    let chain = ChainField {
        sharp_turn_degrees: 45.0,
        ..l_chain()
    };
    let mut mesh = LevelLineMesh::new();
    // (3, 1) is on the L's corner seam; the partners stand on the outgoing and incoming sides.
    mesh.vertices = vec![
        Vector3::new(3.0, 1.0, 0.0),
        Vector3::new(4.5, 3.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
    ];
    let end = (0, 4.0, (4.0, 4.0), 0.0);
    let toward_out = (1, 7.0, (7.0, 7.0), 0.0);
    let toward_in = (2, 1.0, (1.0, 1.0), 0.0);
    // Read on the outgoing leg at its foot there, one along it.
    assert_eq!(
        fillet_arc::on_the_partners_leg(&mesh, &chain, end, toward_out),
        (0, 5.0, (5.0, 5.0), 0.0)
    );
    // Reading on the joint already counts as the incoming leg's: as it was.
    assert_eq!(
        fillet_arc::on_the_partners_leg(&mesh, &chain, end, toward_in),
        end
    );
}

#[test]
fn rust_only_a_one_owner_loop_splits_at_another_chains_tie_run() {
    use geometry_processing::bevel::level_line::corner_fill_pieces::tie_split;
    use geometry_processing::bevel::level_line::GapLoop;
    let x_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let y_leg = Rc::new(open_chain(
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 4.0, 0.0)],
        vec![1.0, 1.0],
    ));
    let chains = vec![x_leg, y_leg];
    let mut mesh = LevelLineMesh::new();
    mesh.vertices = (0..6)
        .map(|i| Vector3::new(1.0, 1.0 + i as f64, 0.0))
        .collect();
    let mut gap_loop = GapLoop::new(
        1,
        true,
        (0..6).collect(),
        vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        0,
        false,
        None,
    );
    // Vertices 2, 3 and 4 tie with chain 1.
    gap_loop.vertex_ties = Some(vec![
        vec![0],
        vec![0],
        vec![0, 1],
        vec![0, 1],
        vec![0, 1],
        vec![0],
    ]);
    let [own, tied] = tie_split(&mesh, &chains, &gap_loop, 0)
        .expect("no budget")
        .expect("one tie run of one other chain");
    assert_eq!(tied.owner, 1);
    assert_eq!(tied.vertices, vec![2, 3, 4]);
    assert_eq!(tied.parameters, vec![3.0, 4.0, 4.0]);
    assert_eq!(own.owner, 0);
    assert_eq!(own.vertices, vec![4, 5, 0, 1, 2]);
    assert_eq!(own.parameters, vec![4.0, 5.0, 0.0, 1.0, 2.0]);
    // No ties: nothing to split.
    gap_loop.vertex_ties = None;
    assert!(tie_split(&mesh, &chains, &gap_loop, 0)
        .expect("no budget")
        .is_none());
}
