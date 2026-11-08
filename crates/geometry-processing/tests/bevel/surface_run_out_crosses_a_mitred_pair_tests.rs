//! A mixed corner's RUN-OUT crossing BOTH bands of one mitred corner - the lobed prism's barrel rim
//! and seam picked together with the whole CLOSED rim of the flat side, which is what a FACE pick
//! of that side resolves to. That rim puts a band on the bottom cap and a band on the wall the
//! run-out turns against, and the two mitre at the part's own corner between them.
//!
//! The cap band cuts the run-out's columns from the CAP inward and the wall band cuts them from the
//! WALL outward, so every column is the window between them - and the turn the two cuts meet at is
//! where the window shuts. That meeting point stands on both bands' surfaces at once, so it stands
//! on their MITRE RING (`SurfaceCrossingAtTheMitre`). Measured at `r = 3.756`: the two meet 51.205
//! degrees past the bead at `[2.6101, 0.1975, 0.1975]`, 0.4137 of the way along the ring's chord
//! from rail 6 to rail 7, 6.3101 out of the bead's axis. The lobed prism's picks are shared with
//! `surface_run_out_crosses_a_band_tests`.

use std::rc::Rc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_run_out, surface_room,
    SurfaceBevelPlan, SurfaceBuildOptions, SurfaceCrossingAtTheMitre, SurfaceEdgePlan,
    SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceStrip, SurfaceVertexPool,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge};
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_run_out_crosses_a_band_tests::{
    lobed_regions, plans_for, the_barrel, the_far_side,
};

/// What "Every Chamfer Works.mcx" is filleted at, and what the prism is measured at here.
const RADIUS: f64 = 3.756;

/// The hand-over itself, measured before anything is built with it: which chord of the mitre ring
/// it splits, how far along, where it stands and at what turn - and that it really is on the
/// run-out's own surface, which is what makes it a point of all three.
#[test]
fn the_hand_over_stands_on_one_chord_of_the_mitre_ring() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism);

    let wall = SurfaceMixedCornerWall::try_read(&corner)
        .unwrap_or_else(|why| panic!("the wall has to read - {:?}", why.code()));

    let crossing = SurfaceCrossingAtTheMitre::try_read(
        &corner,
        &wall,
        Some(&the_cap_bands_strip(&prism)),
        Some(&the_wall_bands_strip(&prism)),
        Vector3::ZERO,
        SurfaceVertexPool::near_enough_for(&prism.vertices),
    )
    .unwrap_or_else(|failure| {
        panic!(
            "the two bands mitre at the origin and the run-out reaches over them - {:?}",
            failure.code()
        )
    });

    assert_eq!(
        crossing.ring.len(),
        9,
        "one corner per rail, at eight segments to the quarter"
    );

    let given_up: Vec<usize> = (0..9).filter(|&rail| crossing.is_given_up(rail)).collect();
    assert_eq!(
        given_up,
        vec![0, 1],
        "the two rails on the CAP side of the ring stand under the run-out's own arc and are given \
         up to it; the other seven keep their mitre"
    );

    // Named by the two points rather than by the rail numbers: which end of a ring is rail zero is
    // the plan walk's business, and the chord is the same chord either way round.
    let from = crossing.ring[crossing.chord as usize];
    let to = crossing.ring[crossing.chord as usize + 1];
    let deep = Vector3::new(2.3186, 0.2859, 0.2859);
    let shallow = Vector3::new(3.0232, 0.0722, 0.0722);

    assert!(
        f64::min(
            (from - deep).length() + (to - shallow).length(),
            (to - deep).length() + (from - shallow).length()
        ) < 1e-3,
        "the ring leaves the run-out's ball on the chord between [2.3186, 0.2859, 0.2859] and \
         [3.0232, 0.0722, 0.0722] - instead {from:?} to {to:?}"
    );

    // 0.4137 of the way along it from the rail up the wall, which is 0.5863 from the one on the
    // cap.
    assert_within(
        f64::min(
            crossing.how_far_along_the_chord,
            1.0 - crossing.how_far_along_the_chord,
        ),
        0.41366,
        1e-4,
        "and 0.4137 of the way along it from the deep end",
    );

    assert!(
        (crossing.at - Vector3::new(2.6101, 0.1975, 0.1975)).length() < 1e-3,
        "which is where the three surfaces meet - instead {:?}",
        crossing.at
    );

    assert_within(
        math_helper::radians_to_degrees(crossing.turn_past),
        51.2051,
        1e-3,
        "the window shuts 51.205 degrees past the bead",
    );

    assert_within(
        corner.axis().distance_to(crossing.at),
        6.31010,
        1e-4,
        "a radius of 6.31 out of the axis, which is between the run-out's rows 1 and 2",
    );

    // On the run-out's own surface, to the arithmetic's own precision rather than to a ruler: the
    // point is a ball radius from the ball's centre at its own turn, and that is what "on all three
    // surfaces" means for the third of them.
    assert_within(
        (crossing.at - corner.ball_centre_at(corner.turn_radians() + crossing.turn_past)).length(),
        corner.size,
        1e-9,
        "the hand-over is exactly a ball radius off the sweep, not a chord's worth off it",
    );

    // And which rails are given up is the ARC's answer, worked here from the corner's own numbers
    // so the reading is checked against the geometry and not against itself: in the meridian plane
    // at one turn, the run-out takes everything below z = r - sqrt(r^2 - (rho - 2r)^2) between
    // rho = r and rho = 2r. The ring's rails 0 and 1 stand at (5.3232, 0) and (5.9118, 0.0722)
    // against arcs of 0.7039 and 0.3581; rail 2 stands at (6.5936, 0.2859) against an arc of
    // 0.1141.
    for (rail, out_of_the_axis, above, arc) in [
        (0usize, 5.32317, 0.0, 0.70389),
        (1, 5.91173, 0.0722, 0.35810),
        (2, 6.59361, 0.2859, 0.11409),
    ] {
        let at = crossing.ring[rail];

        assert_within(
            corner.axis().distance_to(at),
            out_of_the_axis,
            1e-4,
            &format!("rail {rail} of the ring stands {out_of_the_axis} out of the axis"),
        );

        assert_within(
            (at - corner.outer_rail_centre).dot(-corner.out_of_the_cap_face),
            above,
            1e-4,
            &format!("and {above} above the cap face"),
        );

        // Worked from the radius the ring really stands at rather than from the rounded number
        // above, so the arc is read off the geometry and the literal is only a witness to it.
        let over_the_touch_row = corner.axis().distance_to(at) - (2.0 * corner.size);

        assert_within(
            corner.size
                - ((corner.size * corner.size) - (over_the_touch_row * over_the_touch_row)).sqrt(),
            arc,
            1e-3,
            &format!("where the run-out's own arc stands {arc} above it"),
        );

        assert_eq!(
            crossing.is_given_up(rail),
            above < arc,
            "rail {rail} is under the run-out exactly while it stands below that arc"
        );
    }

    // Rails 4 to 8 are further out of the axis than the touch row itself, so the cap face is whole
    // out there and no meridian of the sweep reaches them at all.
    for rail in 4..crossing.ring.len() {
        assert!(
            corner.axis().distance_to(crossing.ring[rail]) > 2.0 * corner.size,
            "rail {rail} of the ring stands beyond the run-out's touch row"
        );

        assert!(
            !crossing.is_given_up(rail),
            "so rail {rail} keeps its mitre untouched"
        );
    }
}

/// A chord of the ring whose clear end is clear ONLY because its own turn is past the run-out's
/// last meridian straddles the reading exactly as a real crossing does - and the point a bisection
/// converges on there is on that meridian and nowhere on the run-out. Measured here, and refused by
/// the check that the hand-over stands a ball's radius off the sweep.
///
/// The mechanism rather than a whole fixture: the lobed prism's own mitre ring never reaches past
/// the sweep - its rails turn 45 to 53 degrees past the bead against a sweep of 60 - so the two
/// ends of the chord are made here by carrying one real ring point round the axis until its turn
/// leaves the sweep. Turning about the axis changes neither the radius nor the height, so the
/// converged point is the SAME distance off the ball as the point it came from, and that distance
/// is what the guard reads. What is NOT covered here is the guard's own refusal through
/// `SurfaceCrossingAtTheMitre::try_read`: reaching it needs a ring that really does run past the
/// sweep, which no fixture in this suite has yet.
#[test]
fn a_chord_clear_only_because_its_turn_is_past_the_sweep_is_no_hand_over() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism);

    let wall = SurfaceMixedCornerWall::try_read(&corner)
        .unwrap_or_else(|why| panic!("the wall has to read - {:?}", why.code()));
    // The rail has to meet the wall: a miss read as a turn of 0 would put every point above the
    // arc's start and let the checks below pass for the wrong reason.
    let meets_the_wall = wall
        .try_where_the_rail_meets_the_wall()
        .expect("the rail meets the wall");

    let clears = f64::min(
        meets_the_wall,
        corner.turn_radians() + surface_mixed_corner_run_out::clears_the_wall(),
    );

    // One rail of the real ring, which stands UNDER the run-out: inside the sweep, between the
    // touch row and the bead, and below the arc.
    let under = Vector3::new(3.0232, 0.0722, 0.0722);

    assert!(
        SurfaceCrossingAtTheMitre::above_the_arc(&corner, under, clears) < 0.0,
        "this rail of the ring is under the run-out's own arc"
    );

    // The same point carried round until its turn leaves the sweep. Nothing about its shape has
    // changed - same radius, same height - so it is under the arc as much as it ever was, and only
    // the turn test calls it clear.
    let the_sweep_runs = clears - corner.turn_radians();
    let its_own_turn = SurfaceCrossingAtTheMitre::turn_past_of(&corner, under);
    let past = corner.turned_about(
        under,
        the_sweep_runs + math_helper::degrees_to_radians(15.0) - its_own_turn,
    );

    assert!(
        math_helper::radians_to_degrees(SurfaceCrossingAtTheMitre::turn_past_of(&corner, past))
            > math_helper::radians_to_degrees(the_sweep_runs),
        "carried past the last meridian of the sweep"
    );

    assert!(
        {
            let above = SurfaceCrossingAtTheMitre::above_the_arc(&corner, past, clears);
            above.is_infinite() && above > 0.0
        },
        "so the reading calls it clear - and a chord from the one to the other straddles"
    );

    // Where a bisection on that straddle converges: the turn at which the sweep ends. Its radius
    // and height are the ones it started with, so it stands as far off the ball as they do - a
    // quarter of a millimetre here, which is nothing like a weld and everything like a hole.
    let on_the_last_meridian = corner.turned_about(under, the_sweep_runs - its_own_turn);

    let off_the_ball =
        (on_the_last_meridian - corner.ball_centre_at(clears)).length() - corner.size;

    assert!(
        off_the_ball.abs() > SurfaceVertexPool::near_enough_for(&prism.vertices),
        "the converged point is {off_the_ball} off the run-out's own surface, so it is no \
         hand-over and the reading has to say so rather than pool it"
    );

    assert_within(
        off_the_ball.abs(),
        0.2601,
        1e-3,
        "measured: a quarter of a millimetre at r 3.756",
    );
}

/// And the pair builds: the face pick of the flat side together with the barrel's rim and seam, at
/// the size the crossing really happens at, closes as one manifold shell the kernel takes.
#[test]
fn the_mitred_pair_is_sewn_and_closes() {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = lobed_regions(&prism);
    let plans = the_whole_pick(&prism, &regions, RADIUS);

    let fit = surface_room::try_fit(&plans, RADIUS, &CancelToken::new(), None)
        .unwrap_or_else(|_| panic!("the room is not cancelled"));
    assert!(
        fit.fits,
        "the skirt over the two bands is a pair to sew - {:?}",
        fit.refusal.as_ref().map(|refusal| refusal.code())
    );

    assert!(
        fit.crossings.iter().any(|one| !one.stands_on_the_wall()),
        "the cap band is crossed"
    );

    assert!(
        fit.crossings.iter().any(|one| one.stands_on_the_wall()),
        "and so is the band on the wall the run-out turns against"
    );

    let built = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: Some(&fit.crossings),
            ..crate::surface_test_build::build_options()
        },
    )
    .unwrap_or_else(|reason| panic!("and the pair is sewn at the mitre - {reason:?}"))
    .mesh;

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the shell closes over both crossings - open at {open_at:?}"
    );

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    let imported = manifold_import::import_strict(&built).unwrap_or_else(|rejected| {
        panic!(
            "the kernel refused the published mesh - {:?}",
            rejected.status
        )
    });

    assert_within(
        imported.volume(),
        signed_volume(&built).abs(),
        1e-6,
        "and holds the same solid the mesh measures as",
    );
}

/// The band the flat side's rim lays on the bottom CAP, which ends the run-out's rows.
fn the_cap_bands_strip(prism: &Mesh) -> SurfaceStrip {
    strip_of(prism, |edge| {
        edge.from().x.abs() < 1e-9
            && edge.to().x.abs() < 1e-9
            && edge.from().z.abs() < 1e-9
            && edge.to().z.abs() < 1e-9
    })
}

/// And the one it stands on the WALL, which ends its columns.
fn the_wall_bands_strip(prism: &Mesh) -> SurfaceStrip {
    strip_of(prism, |edge| {
        edge.from().x.abs() < 1e-9
            && edge.to().x.abs() < 1e-9
            && edge.from().y.abs() < 1e-9
            && edge.to().y.abs() < 1e-9
    })
}

fn strip_of(prism: &Mesh, which: impl Fn(&SurfaceEdgePlan) -> bool) -> SurfaceStrip {
    let regions = lobed_regions(prism);
    let plans = the_whole_pick(prism, &regions, RADIUS);
    let found: Vec<&SurfaceEdgePlan> = plans
        .iter()
        .flat_map(|plan| plan.edges.iter())
        .filter(|edge| which(edge))
        .collect();
    assert_eq!(found.len(), 1, "one band answers the pick");

    SurfaceStrip::try_create(found[0], 8)
        .unwrap_or_else(|why| panic!("the band has to rule: {:?}", why.code()))
}

/// The mixed corner the barrel's rim turns at the bottom cap's left-hand crossing.
fn the_bottom_caps_corner(prism: &Mesh) -> Rc<SurfaceMixedCorner> {
    let regions = lobed_regions(prism);

    for plan in the_whole_pick(prism, &regions, RADIUS) {
        for edge in 0..plan.edges.len() as i32 {
            if let Some(mixed) = plan.try_mixed_corner_after(edge) {
                if mixed.corner.z.abs() < 1e-6 && mixed.corner.x < lobed_prism_meshes::WIDTH / 2.0 {
                    return mixed;
                }
            }
        }
    }

    panic!("the bottom cap's corner has to read");
}

/// The pick: the barrel's whole rim - which carries the seam and both mixed corners - and the
/// WHOLE closed rim of the flat side, which is what a face pick of it resolves to and is what puts
/// two mitred bands in the run-out's way.
fn the_whole_pick(
    prism: &Mesh,
    regions: &std::sync::Arc<FaceRegionCache>,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    let mut seeds: Vec<FeatureEdge> = the_barrel(prism).boundary_edges().to_vec();

    seeds.extend(the_far_side(prism).boundary_edges().iter().cloned());

    plans_for(regions, size, seeds)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}
