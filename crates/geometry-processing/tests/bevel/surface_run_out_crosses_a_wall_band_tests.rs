//! A mixed corner's RUN-OUT crossing a band standing on the WALL it turns against, sewn along the
//! curve the two meet in - the lobed prism's barrel rim and seam picked together with the single
//! VERTICAL edge of the flat side, which is "Every Chamfer Works.mcx"'s own seam with the cube's
//! (-21, 28, z) edge (`lobed_prism_meshes`).
//!
//! The dual of `surface_run_out_crosses_a_band_tests`. A band on the CAP the run-out travels across
//! ends its ROWS, each at its own turn; a band on the WALL it turns against ends its COLUMNS, each
//! at its own radius - the band simply stands in front of the wall from the turn the two first
//! meet, and the column ends on it instead. Measured: the band's rails cross the torus at 45.12,
//! 49.64,
//! 52.19 and 53.16 degrees past the bead, at heights 0.7037, 0.3579, 0.1140 and 0.0049 over the
//! cap, and the curve leaves the patch through the CAP at about 53.2. The first of them, on the
//! rail that stands in the wall itself, is [3.756, 0, 0.7037] - exactly where the run-out's own
//! foot crosses that band's tangent line. The lobed prism's picks are shared with the cap band's
//! class.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_run_out, surface_room,
    SurfaceBandEnd, SurfaceBevelPlan, SurfaceBuildOptions, SurfaceEdgePlan, SurfaceMixedCorner,
    SurfaceMixedCornerWall, SurfaceRunOutColumnWindow, SurfaceRunOutMeetsAWallBand, SurfaceStrip,
    SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FaceRegionCache, FeatureEdge};
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_run_out_crosses_a_band_tests::{
    build_with, lobed_regions, plans_for, the_barrel, the_far_side,
};

/// What "Every Chamfer Works.mcx" is filleted at.
const RADIUS: f64 = 3.756;

/// The reading itself: one crossing per rail of the band that stands inside the run-out's own
/// quarter of the torus, each in closed form off that rail's own radius.
#[test]
fn each_rail_of_the_band_ends_a_column_at_its_own_radius() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let band = the_wall_bands_strip(&prism, RADIUS);
    let meets = meets_of(&corner, &band);

    let turns = &meets.rail_turns_past_the_bead;

    assert_eq!(
        turns.len(),
        4,
        "four of the nine rails stand between r and 2r of the axis - instead {}",
        turns.len()
    );

    let measured = [45.1230, 49.6429, 52.1932, 53.1624];

    for (rail, &expected) in measured.iter().enumerate() {
        assert_within(
            math_helper::radians_to_degrees(turns[rail]),
            expected,
            1e-3,
            &format!("rail {rail} of the crossing stands where its own circle reaches it"),
        );

        // And the column really ends there, on that rail: the radius the band stands off at that
        // turn is the rail's own, and the point is a point of the rail's line.
        let (stands_off, at) = meets
            .try_stands_off_at(turns[rail])
            .expect("the band stands off at its own rail's turn");

        assert!(
            (at - band.point_on(0, 0.0))
                .dot(band.along().cross(Vector3::UNIT_Z))
                .abs()
                < 1.0,
            "the point stands in the band's own profile"
        );

        assert_within(corner.axis().distance_to(at), stands_off, 1e-9, "");
    }

    // The first of them is the foot's own crossing of that band's tangent line in the wall, which
    // is the one place the crossing curve touches the wall at all.
    let (_, the_foot) = meets
        .try_stands_off_at(turns[0])
        .expect("the band stands off at its first rail's turn");

    assert!(
        (the_foot - Vector3::new(3.756, 0.0, 0.7037)).length() < 1e-3,
        "the curve starts where the run-out's foot meets the band on the wall - instead {the_foot:?}"
    );
}

/// A band whose edge does NOT run along the bead's axis is refused by name rather than searched
/// for: the closed form above is the whole of this reading.
#[test]
fn a_band_across_the_axis_is_refused_by_name() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let across = the_cap_bands_strip(&prism, RADIUS);

    assert!(
        across.along().dot(corner.axis().direction).abs() < 0.001,
        "the cap's own band runs square to the axis, which is the case this refuses"
    );

    let Err(why) = SurfaceRunOutMeetsAWallBand::try_read(Some(&corner), Some(&across)) else {
        panic!("refused, with no reading handed back");
    };

    assert_eq!(
        why.code(),
        BevelAdvisoryCode::SurfaceCrossingBandNotAlongTheAxis
    );
}

/// And why the two are not sewn yet, measured: seen from the bead's axis the band's own profile
/// turns back on itself, and it does so while its far side still stands INSIDE the circle the touch
/// point traces. Between those two turns the rows of the run-out that still stand are a middle band
/// of the column rather than a prefix of it - a lens of material both removals leave - and a
/// run-out's columns are prefixes by construction.
///
/// The numbers on this fixture: the profile's furthest turn is the last crossed rail's own, 53.162
/// degrees past the bead, where the two crossings of the ray meet at radius 7.3205 - well inside
/// the
/// 7.512 the touch row stands at. The far crossing comes inside that circle at 53.099, so the lens
/// is the 0.064 degrees between them. It is not a chording artefact: the ideal circle turns back at
/// 53.2 with its crossings meeting near 7.6, which leaves the same lens 0.03 degrees wide.
#[test]
fn the_curve_turns_back_before_its_far_side_reaches_the_touch_row() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let band = the_wall_bands_strip(&prism, RADIUS);
    let meets = meets_of(&corner, &band);

    let furthest = *meets
        .rail_turns_past_the_bead
        .last()
        .expect("a crossed rail");

    assert_within(
        math_helper::radians_to_degrees(furthest),
        53.1624,
        1e-3,
        "the profile's own furthest turn is the last rail the crossing reaches",
    );

    let (meet_at, _) = meets
        .try_stands_off_at(furthest)
        .expect("the band stands off at its furthest turn");

    assert!(
        meet_at < 2.0 * RADIUS,
        "and there the two crossings meet at {meet_at}, inside the {} the touch row stands at",
        2.0 * RADIUS
    );

    // Which leaves a stretch of turn where the far crossing has come INSIDE the touch row: there
    // the rows that stand are a middle band of the column and the curve has two branches.
    let mut below = math_helper::degrees_to_radians(52.0);
    let mut above = furthest;

    for _ in 0..60 {
        let middle = (below + above) / 2.0;

        if meets
            .try_the_far_side_at(middle)
            .is_some_and(|far| far < 2.0 * RADIUS)
        {
            above = middle;
        } else {
            below = middle;
        }
    }

    let lens_opens_at = (below + above) / 2.0;

    assert_within(
        math_helper::radians_to_degrees(lens_opens_at),
        53.0986,
        1e-3,
        "the far side comes inside the touch row there",
    );

    assert!(
        lens_opens_at < furthest,
        "so the lens is open before the curve turns back, and the column is no longer a prefix"
    );
}

/// And where the wall itself RUNS OUT under the blend as well, the band still says where the
/// columns end: its own rails are the stations, and the wall's end plane does not put its crossings
/// back among them.
///
/// The two stationings cannot both hold. A band on the wall ends every column ON one of its rails,
/// which is what makes each crossing one vertex both surfaces are drawn to; the wall's end plane
/// stations the turns its own rows leave at, which fall between those rails. Run together, the
/// columns end part way along a quad of the strip and the shell is left open with nothing named.
#[test]
fn a_wall_that_runs_out_under_the_blend_does_not_restation_the_bands_own_crossings() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let band = the_wall_bands_strip(&prism, RADIUS);
    let mut pool = SurfaceVertexPool::new(&prism.vertices);

    let wall =
        SurfaceMixedCornerWall::try_read(&corner).unwrap_or_else(|why| panic!("{:?}", why.code()));
    let meets = meets_of(&corner, &band);

    let seed = the_seed_column(&mut pool, &corner);
    let step = corner.turn_radians() / 8.0;

    // The plane across a wall's own end, put far enough along the turn to have crossings of its
    // own.
    let ends_on = SurfaceBandEnd::new(
        corner.outer_rail_point(
            corner.turn_radians() + (surface_mixed_corner_run_out::clears_the_wall() / 2.0),
        ),
        corner
            .out_of_the_wall
            .cross(corner.out_of_the_cap_face)
            .get_normal(),
    );

    let alone = surface_mixed_corner_run_out::try_columns(
        &mut pool,
        &corner,
        &wall,
        &seed,
        step,
        None,
        None,
        Some(&meets),
        None,
        None,
    )
    .unwrap_or_else(|no| panic!("{:?}", no.code()));

    let together = surface_mixed_corner_run_out::try_columns(
        &mut pool,
        &corner,
        &wall,
        &seed,
        step,
        Some(ends_on),
        None,
        Some(&meets),
        None,
        None,
    )
    .unwrap_or_else(|nope| panic!("{:?}", nope.code()));

    assert_eq!(
        together.turns.len(),
        alone.turns.len(),
        "the band's own rails are the stations either way - instead {} against {}",
        together.turns.len(),
        alone.turns.len()
    );

    for station in 0..alone.turns.len() {
        assert_within(
            together.turns[station],
            alone.turns[station],
            1e-12,
            &format!("station {station} stands at the same turn either way"),
        );

        assert_eq!(
            together.columns[station].len(),
            alone.columns[station].len(),
            "and ends on the same vertex, so the column is the same one"
        );
    }
}

/// The patch's own last column, as the ideal ring stands it: the profile from the cap face inward,
/// on the circle the bead's axis turns it about.
fn the_seed_column(pool: &mut SurfaceVertexPool, corner: &SurfaceMixedCorner) -> Vec<i32> {
    let count = 9;
    let out_of_the_axis = corner.out_of_the_axis(corner.turn_radians());

    (0..count)
        .map(|row| {
            let turn = std::f64::consts::PI / 2.0 * row as f64 / (count - 1) as f64;
            let radius = (2.0 * corner.size) - (corner.size * turn.sin());
            let under = corner.size * (1.0 - turn.cos());

            pool.add(
                corner.outer_rail_centre + (out_of_the_axis * radius)
                    - (corner.out_of_the_cap_face * under),
            )
        })
        .collect()
}

/// The far side of the band's profile is only the SAME as the near side at the one rail the profile
/// turns back at. At every other crossed rail the ray meets the profile twice and the far side is a
/// real crossing of its own - which is what says whether the touch row still stands there.
///
/// Answered from the rail itself at the turn-back, because there the ray touches the profile
/// without crossing it and a crossing test reads no crossing at all. Answered from the rail
/// ANYWHERE would hand back the near side as the far one, and the cap exit would then be bisected
/// against a bound that is not the turn-back at all.
#[test]
fn the_far_side_is_the_near_one_only_where_the_profile_turns_back() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let band = the_wall_bands_strip(&prism, RADIUS);
    let meets = meets_of(&corner, &band);

    let turns = &meets.rail_turns_past_the_bead;
    let the_turn_back = turns[turns.len() - 1];

    let at_the_turn_back = meets
        .try_the_far_side_at(the_turn_back)
        .expect("the far side reads at the turn-back");
    let (near_there, _) = meets
        .try_stands_off_at(the_turn_back)
        .expect("the near side reads at the turn-back");

    assert_within(
        at_the_turn_back,
        near_there,
        1e-9,
        "the two sides meet at the turn-back",
    );

    for (rail, &turn) in turns[..turns.len() - 1].iter().enumerate() {
        let far = meets.try_the_far_side_at(turn).expect("the far side reads");
        let (near, _) = meets.try_stands_off_at(turn).expect("the near side reads");

        assert!(
            far > near + 1e-6,
            "at rail {rail} the ray crosses the profile twice - instead {far} and {near}"
        );

        assert!(
            far > 2.0 * RADIUS,
            "and stands outside the touch row there, which is why the column is still a prefix - \
             instead {far}"
        );
    }
}

/// The whole shell: the run-out's columns ended on the band and the band's strip cut back to the
/// same curve, so the part closes, at a size of 3.756.
#[test]
fn the_run_out_and_the_walls_band_are_sewn_along_the_curve_they_cross_in() {
    let size = 3.756;
    let built = the_crossing_built(size);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the shell closes over the crossing - open at {open_at:?}"
    );

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    // And the kernel takes it, which is what a Combine after a Fillet really asks of it.
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

/// And every corner of the curve is ONE vertex both surfaces are drawn to - a triangle of the
/// run-out and a triangle of the band's strip alike - rather than two points a rounding apart.
#[test]
fn every_corner_of_the_crossing_is_one_vertex_both_surfaces_meet_at() {
    let built = the_crossing_built(RADIUS);

    let the_foot = built
        .vertices
        .iter()
        .map(|vertex| Vector3::from_vector3_float(*vertex))
        .filter(|at| (*at - Vector3::new(3.756, 0.0, 0.7037)).length() < 1e-3)
        .count();

    assert_eq!(
        the_foot, 1,
        "the crossing's first corner is one vertex and not two"
    );

    for corner in the_corners_of_the_curve() {
        let around = built
            .faces
            .iter()
            .filter(|face| {
                [face.v0, face.v1, face.v2].iter().any(|&vertex| {
                    (Vector3::from_vector3_float(built.vertices[vertex as usize]) - corner).length()
                        < 1e-3
                })
            })
            .count();

        assert!(
            around >= 4,
            "the corner at {corner:?} is a place the two surfaces meet, not a rim either of them \
             ends on"
        );
    }

    // And nothing of the two rims is left standing between them: the stretch of sharp edge the
    // run-out used to die along carries no vertex at all now, because the sector's bite and the
    // band's own section close on the one point they share there.
    let between_the_rims = built
        .vertices
        .iter()
        .map(|vertex| Vector3::from_vector3_float(*vertex))
        .filter(|at| at.y.abs() < 1e-6 && at.z.abs() < 1e-6 && at.x > 0.5 && at.x < 3.5)
        .count();

    assert_eq!(
        between_the_rims, 0,
        "neither rim mints a corner of its own along that edge - instead {between_the_rims}"
    );
}

/// And a band that reaches only ONE of its own rails into the run-out is refused by name: the
/// hairpin where its profile turns back stands on the turn the run-out dies at anyway, so what the
/// two share is a point rather than a line and there is nothing either side of it to sew.
///
/// On this fixture that is `r = 2.96`: the one rail crosses 59.869 degrees past the bead - against
/// the 60 the run-out clears the wall at - at 0.00009 along the band's own edge.
#[test]
fn a_crossing_that_only_reaches_the_run_outs_death_is_refused_by_name() {
    const SIZE: f64 = 2.96;

    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, SIZE);
    let band = the_wall_bands_strip(&prism, SIZE);

    let Err(why) = SurfaceRunOutMeetsAWallBand::try_read(Some(&corner), Some(&band)) else {
        panic!("refused, with no reading handed back");
    };

    assert_eq!(
        why.code(),
        BevelAdvisoryCode::SurfaceCrossingAtTheRunOutsDeath
    );

    // The numbers it is refused over, read off the one rail the band really does reach.
    let only_one: Vec<Vector3> = (0..band.rail_count())
        .map(|rail| band.point_on(rail, 0.0))
        .filter(|&at| {
            corner.axis().distance_to(at) > SIZE && corner.axis().distance_to(at) < 2.0 * SIZE
        })
        .collect();

    assert_eq!(
        only_one.len(),
        1,
        "one rail of the band stands inside the run-out's own quarter of the torus and no more"
    );

    assert_within(
        band.param_of(only_one[0]),
        0.0,
        1e-3,
        "and it stands at the very end of the band's edge",
    );

    // And the build says the same rather than drawing a crossing with nothing either side of it.
    let regions = lobed_regions(&prism);
    let plans = the_seam_and_the_vertical_edge(&prism, &regions, SIZE);

    let fit = surface_room::try_fit(&plans, SIZE, &CancelToken::new(), None)
        .unwrap_or_else(|_| panic!("the room is not cancelled"));
    assert!(fit.fits);

    let Err(reason) = surface_bevel_builder::try_build(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: Some(&fit.crossings),
            ..SurfaceBuildOptions::default()
        },
    ) else {
        panic!("the build refuses the crossing too");
    };

    assert_eq!(
        reason.advisory().map(|reason| reason.code()),
        Some(BevelAdvisoryCode::SurfaceCrossingAtTheRunOutsDeath),
        "refused by name - instead {reason:?}"
    );
}

/// A column end the band mints part way along one of its own quads is moved onto that quad's
/// chord, and the STAND-OFF the walk admits rows against is read from the moved point rather than
/// from the curve it came off. The two are not the same number: on the mitred pair the curve stands
/// 0.0023 and 0.0066 inside the chord at r 3.756, against a pool that welds at 0.00046 - so a
/// stand-off left on the curve admits a row the column's own end already stands outside of, which
/// is a row drawn under the trim point and a fin between the two surfaces.
///
/// Driven at `SurfaceRunOutColumnWindow::onto_the_bands_staircase` rather than through a whole
/// build, because no fixture in this suite stations the run-out BETWEEN two rails of the wall band
/// on its own: that needs a band on the cap as well, whose rows' crossings are stations too (the
/// mitred pair). The stations here are the real rails and the turn half way between them, and
/// every reading is the production one.
#[test]
fn a_column_end_moved_onto_the_chord_is_read_at_its_own_radius() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let band = the_wall_bands_strip(&prism, RADIUS);
    let meets = meets_of(&corner, &band);

    let rails = &meets.rail_turns_past_the_bead;
    let stations = [rails[0], (rails[0] + rails[1]) / 2.0, rails[1]];

    assert_eq!(
        meets.the_rail_standing_at(stations[1]),
        -1,
        "the middle station is a turn of the run-out's own, part way along one of the band's quads"
    );

    let mut scallops = [Vector3::ZERO; 3];
    let mut stands_off_at = [0.0; 3];

    for station in 0..stations.len() {
        let (stands_off, scallop) = meets
            .try_stands_off_at(stations[station])
            .expect("the band stands off at every station");
        stands_off_at[station] = stands_off;
        scallops[station] = scallop;

        assert_within(
            corner.axis().distance_to(scallops[station]),
            stands_off_at[station],
            1e-9,
            "the band's own reading hands back a point and the radius it stands at, together",
        );
    }

    let on_the_curve = stands_off_at[1];

    SurfaceRunOutColumnWindow::onto_the_bands_staircase(
        &corner,
        &stations,
        &[true, true, true],
        &meets,
        &mut scallops,
        &mut stands_off_at,
    );

    let near_enough = SurfaceVertexPool::near_enough_for(&prism.vertices);
    let moved = corner.axis().distance_to(scallops[1]) - on_the_curve;

    assert!(
        moved.abs() > near_enough,
        "the move is a real move: the chord stands {moved} further out of the axis than the \
         curve, against a pool that welds at {near_enough}"
    );

    assert_within(
        moved,
        0.01542,
        1e-4,
        "measured on this fixture, half way between the first two rails - the mitred pair's own \
         stations sit closer to the rails and read 0.0023 and 0.0066",
    );

    // The invariant: admission reads the MOVED end.
    assert_within(
        stands_off_at[1],
        corner.axis().distance_to(scallops[1]),
        1e-9,
        "the stand-off rows are admitted against has to be the moved end's own",
    );

    // And the rails themselves are untouched: they are vertices of the band, not points part way
    // along a quad, and nothing moves them.
    for station in [0, 2] {
        assert_within(
            corner.axis().distance_to(scallops[station]),
            stands_off_at[station],
            1e-9,
            &format!("station {station} stands on a rail and is read exactly"),
        );
    }
}

/// The corners of the crossing curve: one per crossed rail, and then the cap exit.
fn the_corners_of_the_curve() -> Vec<Vector3> {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, RADIUS);
    let band = the_wall_bands_strip(&prism, RADIUS);
    let meets = meets_of(&corner, &band);

    let mut corners: Vec<Vector3> = meets
        .rail_turns_past_the_bead
        .iter()
        .map(|&turn| {
            meets
                .try_stands_off_at(turn)
                .expect("the band stands off at its own rail's turn")
                .1
        })
        .collect();

    let (_, exit) = meets
        .try_the_cap_exit()
        .expect("the curve leaves through the cap");
    corners.push(exit);

    corners
}

/// The wall band's reading, which every rung but the refusals expects to read.
fn meets_of(corner: &Rc<SurfaceMixedCorner>, band: &SurfaceStrip) -> SurfaceRunOutMeetsAWallBand {
    SurfaceRunOutMeetsAWallBand::try_read(Some(corner), Some(band))
        .unwrap_or_else(|why| {
            panic!(
                "the band runs along the bead's own axis, so its rails keep their radius - {:?}",
                why.code()
            )
        })
        .expect("the band stands on the wall the run-out turns against")
}

/// The crossing's own shell.
fn the_crossing_built(size: f64) -> Mesh {
    let prism = lobed_prism_meshes::create();
    let regions = lobed_regions(&prism);
    let plans = the_seam_and_the_vertical_edge(&prism, &regions, size);

    let fit = surface_room::try_fit(&plans, size, &CancelToken::new(), None)
        .unwrap_or_else(|_| panic!("the room is not cancelled"));
    assert!(
        fit.fits,
        "the pair has to fit: {:?}",
        fit.refusal.as_ref().map(|refusal| refusal.code())
    );

    build_with(&prism, &regions, &plans, &fit.crossings)
        .unwrap_or_else(|reason| panic!("the crossing has to build: {reason}"))
}

/// The strip of the band standing on the wall: the flat side's own vertical edge.
fn the_wall_bands_strip(prism: &Mesh, size: f64) -> SurfaceStrip {
    let regions = lobed_regions(prism);
    let plans = the_seam_and_the_vertical_edge(prism, &regions, size);
    let found: Vec<&SurfaceEdgePlan> = plans
        .iter()
        .flat_map(|plan| plan.edges.iter())
        .filter(|edge| {
            edge.from().x.abs() < 1e-9
                && edge.to().x.abs() < 1e-9
                && edge.from().y.abs() < 1e-9
                && edge.to().y.abs() < 1e-9
        })
        .collect();
    assert_eq!(found.len(), 1, "one band answers the pick");

    SurfaceStrip::try_create(found[0], 8)
        .unwrap_or_else(|why| panic!("the band has to rule: {:?}", why.code()))
}

/// And the strip of a band on the CAP, which runs square to the axis.
fn the_cap_bands_strip(prism: &Mesh, size: f64) -> SurfaceStrip {
    let regions = lobed_regions(prism);
    let seeds: Vec<FeatureEdge> = the_far_side(prism)
        .boundary_edges()
        .iter()
        .filter(|edge| {
            edge.start.x.abs() < 1e-9
                && edge.end.x.abs() < 1e-9
                && edge.start.z.abs() < 1e-9
                && edge.end.z.abs() < 1e-9
        })
        .cloned()
        .collect();

    let plans = plans_for(&regions, size, seeds);
    assert_eq!(plans.len(), 1);

    SurfaceStrip::try_create(&plans[0].edges[0], 8)
        .unwrap_or_else(|why| panic!("the band has to rule: {:?}", why.code()))
}

/// The mixed corner the barrel's rim turns at the bottom cap's left-hand crossing.
fn the_bottom_caps_corner(prism: &Mesh, size: f64) -> Rc<SurfaceMixedCorner> {
    let regions = lobed_regions(prism);

    for plan in the_seam_and_the_vertical_edge(prism, &regions, size) {
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

/// The pick: the barrel's whole rim - which carries the seam and both mixed corners - and the ONE
/// vertical edge of the flat side, which is what a chain pick of that edge resolves to.
fn the_seam_and_the_vertical_edge(
    prism: &Mesh,
    regions: &Arc<FaceRegionCache>,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    let mut seeds: Vec<FeatureEdge> = the_barrel(prism).boundary_edges().to_vec();

    seeds.extend(
        the_far_side(prism)
            .boundary_edges()
            .iter()
            .filter(|edge| {
                edge.start.x.abs() < 1e-9
                    && edge.end.x.abs() < 1e-9
                    && edge.start.y.abs() < 1e-9
                    && edge.end.y.abs() < 1e-9
            })
            .cloned(),
    );

    plans_for(regions, size, seeds)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}
