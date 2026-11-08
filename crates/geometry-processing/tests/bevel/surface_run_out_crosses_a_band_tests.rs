//! A mixed corner's RUN-OUT crossing another chain's BAND on the same cap, sewn along the curve
//! where the two meet instead of refused - the lobed prism's barrel rim and seam picked together
//! with the ONE bottom edge of the flat side 10.228 mm away, which is "Every Chamfer Works.mcx"'s
//! own seam with its cube's `(-21, 38, 0)` edge (`lobed_prism_meshes`).
//!
//! A single FREE ENDED band: the band's strip ends in the plane of the wall the run-out's own foot
//! travels in, so the three corners of the crossing curve - the touch row's, the row below it, and
//! the foot's - all stand on that one strip. The touch row leaves the band 30.14 degrees past the
//! bead, the row below it at 41.65, no row below that reaches the band before the wall has
//! swallowed it, and the foot enters the section the band's end cut takes out of the wall at 53.18
//! degrees - well before the 60 the run-out would otherwise die at. `the_crossing_built` is shared
//! with `surface_result_imports_into_manifold_tests.rs`.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_bevel_crossings, surface_mesh_closure,
    surface_mixed_corner_run_out, surface_room, SurfaceBevelPlan, SurfaceBuildOptions,
    SurfaceEdgePlan, SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceRunOutMeetsTheBand,
    SurfaceStrip, SurfaceStripTrimmedAtTheCrossing, SurfaceStripTrims, SurfaceVertexPool,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{FaceGroup, FaceRegionCache, FeatureEdge};
use solidmesh_math::{Vector2, Vector3};
use solidmesh_mesh::{CancelToken, Mesh};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_test_chains::join_and_open;

const SIZE: f64 = 3.756;

/// The two crossings the rows are stationed at, against the closed form: row j's circle of radius
/// `rho_j` about the bead's axis reaches its own rail's plane where `sin(turn) = (axis - rail) /
/// rho_j`, which is one `asin` and owes the construction nothing.
#[test]
fn each_row_leaves_the_band_where_its_own_circle_reaches_its_own_rail() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, SIZE);
    let band = Rc::new(the_bands_strip(&prism, SIZE, false));
    let rows = the_ideal_seed_rows(&corner, &band);

    let meets = read_meets(&corner, &band, &rows, 1e-6);

    for (row, &seed) in rows.iter().enumerate() {
        // sin(turn) = how far the rail's own plane stands in front of the axis, over the radius the
        // row turns on - which is the whole of it.
        let radius = corner.axis().distance_to(seed);
        let ratio = (corner.outer_rail_centre - the_rail_beside(&corner, &band, seed))
            .dot(across(&corner, &band))
            / radius;

        if ratio.abs() > 1.0 {
            assert!(
                meets.where_the_row_crosses(row as i32) == f64::INFINITY,
                "row {row}'s circle never reaches the band at all"
            );
            continue;
        }

        assert_within(
            meets.where_the_row_crosses(row as i32).to_degrees(),
            ratio.asin().to_degrees(),
            1e-6,
            &format!("row {row} crosses where its own circle reaches its own rail"),
        );
    }

    // And the two the fixture really has, named rather than only derived.
    assert_within(
        meets.where_the_row_crosses(0).to_degrees(),
        30.1427,
        1e-3,
        "row 0",
    );
    assert_within(
        meets.where_the_row_crosses(1).to_degrees(),
        41.6462,
        1e-3,
        "row 1",
    );
    assert!(
        meets.where_the_row_crosses(3) == f64::INFINITY,
        "no row this far down the profile ever reaches the band"
    );
}

/// The same crossings off a band whose chain was walked the OTHER way, which puts rail zero on the
/// wall instead of on the cap: rows and rails are paired by the height and the offset they stand
/// at, so nothing about the answer moves.
#[test]
fn the_band_walked_either_way_is_crossed_at_the_same_turns() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, SIZE);
    let one_way = Rc::new(the_bands_strip(&prism, SIZE, false));
    let the_other = Rc::new(the_bands_strip(&prism, SIZE, true));

    assert!(
        one_way.along().dot(the_other.along()) < -0.999,
        "the two picks really do walk that edge in opposite directions"
    );

    assert!(
        (one_way.point_on(0, 0.0) - the_other.point_on(0, 0.0)).length() > 1.0,
        "and so rail zero is a different end of the profile in each"
    );

    let rows = the_ideal_seed_rows(&corner, &one_way);

    let here = read_meets(&corner, &one_way, &rows, 1e-6);
    let there = read_meets(&corner, &the_other, &rows, 1e-6);

    for row in 0..rows.len() as i32 {
        if here.where_the_row_crosses(row) == f64::INFINITY {
            assert!(there.where_the_row_crosses(row) == f64::INFINITY);
            continue;
        }

        assert_within(
            there.where_the_row_crosses(row),
            here.where_the_row_crosses(row),
            1e-12,
            &format!(
                "row {row} is paired with the rail it STANDS beside, not with the rail of its own \
                 number"
            ),
        );
    }
}

/// The foot's crossing: the run-out's own trim point dies where it enters the section the band's
/// end cut takes out of the wall, which is a point ON the band's profile and IN the wall's plane -
/// the last corner of the curve the two surfaces are sewn along.
#[test]
fn the_foot_dies_on_the_bands_own_profile_and_not_at_sixty_degrees() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, SIZE);
    let band = Rc::new(the_bands_strip(&prism, SIZE, false));
    let rows = the_ideal_seed_rows(&corner, &band);

    let wall = SurfaceMixedCornerWall::try_read(&corner)
        .unwrap_or_else(|why_not| panic!("the wall has to read - {:?}", why_not.code()));

    let meets = read_meets(&corner, &band, &rows, 1e-6);

    let clears = surface_mixed_corner_run_out::clears_the_wall();
    let turn = meets
        .try_where_the_foot_crosses(&corner, &wall, clears)
        .expect("the foot really does run into the band before the wall clears it");

    assert!(
        turn.to_degrees() < clears.to_degrees(),
        "the run-out dies on the band, not on the sharp edge"
    );

    // The CHORDED reading, which is a hair earlier than the true arc's 53.1819: the strip is ruled
    // by its rails and the surface really stands on the chords between them, so the crossing is
    // asked of those and not of the arc they approximate.
    assert_within(
        turn.to_degrees(),
        53.0368,
        1e-3,
        "the chorded foot crossing",
    );

    let at = surface_mixed_corner_run_out::try_scallop_point(&corner, &wall, turn)
        .expect("the scallop point at the crossing");

    // On the wall the run-out's foot travels in, which is the plane the band's strip ends in.
    assert!(at.y.abs() < 1e-9);

    // And on the band's own profile: the ball that swept the strip stands one size from every point
    // of it, so the crossing is a point of the arc rather than a point near it.
    let centre = the_ball_centre_of(&band, SIZE);

    // Within one sagitta of the chording - at eight segments to the quarter that is 0.018 - because
    // the point stands on the CHORD between two rails rather than on the arc itself.
    assert_within(
        (Vector2::new(at.x, at.z) - centre).length(),
        SIZE,
        0.019,
        "the foot dies on the profile the band's end cut left in the wall",
    );
}

/// The whole shell: the corner's run-out and the band's strip trimmed at the curve they cross in
/// and sewn along it, so the part closes and every vertex of the crossing is shared by both
/// surfaces.
#[test]
fn the_skirt_and_the_band_are_sewn_along_the_curve_they_cross_in() {
    let prism = lobed_prism_meshes::create();
    let regions = lobed_regions(&prism);
    let plans = the_seam_and_the_one_edge(&prism, &regions, SIZE);

    let fit = surface_room::try_fit(&plans, SIZE, &CancelToken::new(), None)
        .unwrap_or_else(|_| panic!("the room is not cancelled"));
    assert!(
        fit.fits,
        "a skirt over a band is a pair to sew - {:?}",
        fit.refusal.as_ref().map(|refusal| refusal.code())
    );

    assert_eq!(
        fit.crossings.len(),
        1,
        "one band, on the one cap it stands on"
    );

    let built = build_with(&prism, &regions, &plans, &fit.crossings)
        .unwrap_or_else(|reason| panic!("and the build sews it - {reason}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the shell closes over the crossing - open at {open_at:?}"
    );

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// Where the crossing itself STANDS in the shell: the touch row's corner on the band's own cap side
/// tangent line, the foot's on the wall the run-out travels in, and every one of them a vertex both
/// surfaces meet at rather than two points a rounding apart.
#[test]
fn every_corner_of_the_crossing_is_one_vertex_both_surfaces_meet_at() {
    let built = the_crossing_built();

    // The band's cap side tangent line: x = r on the bottom cap, which is where the walkback left
    // the cap's own rim and where the touch row has to stop.
    let on_the_tangent_line: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|at| {
            (at.x - 3.756).abs() < 1e-6 && at.z.abs() < 1e-6 && at.y > 1e-6 && at.y < 20.0 - 1e-6
        })
        .collect();

    assert!(
        on_the_tangent_line
            .iter()
            .any(|at| (at.y - 2.7404).abs() < 1e-3),
        "the touch row's crossing stands ON the tangent line, 2.7404 along it"
    );

    // And the foot, in the plane of the wall the run-out's own trim curve travels in.
    let the_foot: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|at| at.y.abs() < 1e-9 && (at.x - 2.5371).abs() < 1e-3)
        .collect();

    assert_eq!(
        the_foot.len(),
        1,
        "the foot's crossing is one vertex and not two"
    );

    assert_within(the_foot[0].z, 0.2196, 1e-3, "the foot's height");

    // Every edge of the curve used by exactly two faces is what "sewn" means, and the shell's own
    // closure says it of every edge at once - so what is left to say here is that the curve's
    // corners are used by faces on BOTH sides, which a vertex only one surface reached would not
    // be.
    for corner in [Vector3::new(3.756, 2.7404, 0.0), the_foot[0]] {
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
}

/// Two rails inside one row's own ruler: whichever of them is nearer, a row with two rails under it
/// has no ONE rail to be sewn to, so the pair is refused rather than resolved by the order the rows
/// happened to be read in.
#[test]
fn two_rails_under_one_row_is_refused_rather_than_picked_between() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, SIZE);
    let band = Rc::new(the_bands_strip(&prism, SIZE, false));
    let rows = the_ideal_seed_rows(&corner, &band);

    // One millimetre of ruler, where the profile's own rails stand tenths apart: every row now has
    // several of them under it, which is exactly what a mismatched chording looks like.
    let why = SurfaceRunOutMeetsTheBand::try_read(
        Some(&corner),
        Some(&band),
        the_bands_edge(),
        Some(&rows),
        1.0,
    )
    .expect_err("no row has one rail of its own at that ruler");

    assert_eq!(
        why.code(),
        BevelAdvisoryCode::SurfaceCrossingProfilesDoNotMatch
    );
}

/// Two profiles of DIFFERENT lengths, which is the plainest mismatch there is: refused before any
/// height is compared at all.
#[test]
fn a_band_chorded_differently_is_refused_by_name() {
    let prism = lobed_prism_meshes::create();
    let corner = the_bottom_caps_corner(&prism, SIZE);
    let band = Rc::new(the_bands_strip(&prism, SIZE, false));
    let mut rows = the_ideal_seed_rows(&corner, &band);
    rows.truncate((band.rail_count() - 1) as usize);

    let why = SurfaceRunOutMeetsTheBand::try_read(
        Some(&corner),
        Some(&band),
        the_bands_edge(),
        Some(&rows),
        1e-6,
    )
    .expect_err("refused");

    assert_eq!(
        why.code(),
        BevelAdvisoryCode::SurfaceCrossingProfilesDoNotMatch
    );
}

/// A crossing standing in the MIDDLE of a band rather than running off one of its ends would leave
/// that strip in two pieces with the curve between them, which this construction does not build.
#[test]
fn a_crossing_in_the_middle_of_a_band_is_refused_by_name() {
    let prism = lobed_prism_meshes::create();
    let band = the_bands_strip(&prism, SIZE, false);
    let edge = the_bands_edge_plan(&prism, SIZE);
    let mut pool = SurfaceVertexPool::new(&prism.vertices);

    // The same staircase the real crossing makes, moved bodily five millimetres in from the end.
    let rails = band.rail_count();
    let crossing = vec![
        pool.add(band.point_on(rails - 1, 12.2604)),
        pool.add(band.point_on(rails - 2, 13.6901)),
        pool.add(band.point_on(rails - 3, 15.0)),
    ];

    let why = SurfaceStripTrimmedAtTheCrossing::try_read(
        Some(&pool),
        Some(&band),
        0,
        &edge,
        Some(&crossing),
        None,
    )
    .expect_err("neither end of the band is where this curve stands");

    assert_eq!(why.code(), BevelAdvisoryCode::SurfaceCrossingLeavesTheBand);
}

/// One END of one band crossed by TWO corners' run-outs would have to be cut back at two curves at
/// once, which is named rather than left to the later crossing to overwrite the earlier. The two
/// ENDS of one band are two staircases and are each other's business no more than the two ends of
/// any strip are: a band running between two corners is crossed at each of them.
#[test]
fn one_end_of_a_band_crossed_twice_is_seen_before_the_second_overwrites_the_first() {
    let mut already: SurfaceStripTrims = vec![[None, None], [None, None], [None, None]];
    let (_, trim) = the_real_crossings_trim(&lobed_prism_meshes::create());
    let at_the_start = trim.at_the_start;

    assert!(
        !SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(
            Some(&already),
            1,
            at_the_start
        )
    );

    already[1][if at_the_start { 0 } else { 1 }] = Some(Rc::new(trim));

    assert!(
        SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(
            Some(&already),
            1,
            at_the_start
        ),
        "that end has been claimed and the second crossing has to be told so"
    );

    assert!(
        !SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(
            Some(&already),
            1,
            !at_the_start
        ),
        "but the OTHER end of the same band is still free to be crossed by its own corner"
    );

    assert!(
        !SurfaceStripTrimmedAtTheCrossing::the_band_is_already_crossed(
            Some(&already),
            2,
            at_the_start
        )
    );
}

/// The seam of a crossed strip is cut back to the crossing itself - and only what stands on the
/// TRIMMED side of it comes out. A later split the cap face still uses can stand a hair on the kept
/// side, and dropping it would leave that face a vertex part way along an edge nothing sews to.
#[test]
fn trimming_a_seam_at_the_crossing_keeps_what_stands_a_hair_on_the_kept_side() {
    let prism = lobed_prism_meshes::create();
    let band = Rc::new(the_bands_strip(&prism, SIZE, false));
    let (pool, trim) = the_real_crossings_trim(&prism);

    trim.closed_the_end.set(true);

    let rail = band.rail_count() - 1;
    let cut = trim.param_on(rail).expect("the last rail is crossed");

    let mut seams = vec![[
        Some(vec![(0, 0.0), (1, 20.0)]),
        Some(vec![
            (2, 0.0),
            // A hair on the KEPT side - half a station, which the old reading dropped for being
            // near.
            (3, cut - (pool.weld_distance() / 2.0)),
            // And a hair on the trimmed side, which really has gone.
            (4, cut + (pool.weld_distance() / 2.0)),
            (5, 20.0),
        ]),
    ]];

    let the_last = trim.vertex_on(rail);
    let mut trims: SurfaceStripTrims = vec![[None, None]];
    let at_the_start = trim.at_the_start;
    trims[0][if at_the_start { 0 } else { 1 }] = Some(Rc::new(trim));

    surface_bevel_crossings::trim_the_seams_at_the_crossings(&[Some(band)], &trims, &mut seams);

    let kept: Vec<i32> = seams[0][1]
        .as_ref()
        .expect("the seam is still there")
        .iter()
        .map(|one| one.0)
        .collect();

    assert!(
        !kept.contains(&4),
        "that one stands on the side of the cut the crossing took"
    );

    assert!(
        kept.contains(&3),
        "and that one stands on the side the cap face still carries, a hair though it is"
    );

    assert_eq!(
        kept.last().copied(),
        the_last,
        "the crossing itself is the last vertex the stitch has to reach on that line"
    );
}

/// The crossing's own shell, which the boolean kernel is asked to take as
/// well.
pub fn the_crossing_built() -> Mesh {
    let prism = lobed_prism_meshes::create();
    let regions = lobed_regions(&prism);
    let plans = the_seam_and_the_one_edge(&prism, &regions, SIZE);

    let fit = surface_room::try_fit(&plans, SIZE, &CancelToken::new(), None)
        .unwrap_or_else(|_| panic!("the room is not cancelled"));
    assert!(
        fit.fits,
        "the pair has to fit: {:?}",
        fit.refusal.as_ref().map(|refusal| refusal.code())
    );

    build_with(&prism, &regions, &plans, &fit.crossings)
        .unwrap_or_else(|reason| panic!("the crossing has to build: {reason}"))
}

pub fn build_with(
    prism: &Mesh,
    regions: &FaceRegionCache,
    plans: &[Rc<SurfaceBevelPlan>],
    crossings: &[solidmesh::bevel::surface::SurfaceRoomCrossing],
) -> Result<Mesh, String> {
    surface_bevel_builder::try_build(
        prism,
        &prism.get_mesh_edge_graph(),
        regions,
        plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: Some(crossings),
            ..crate::surface_test_build::build_options()
        },
    )
    .map(|built| built.mesh)
    .map_err(|reason| format!("{reason:?}"))
}

fn read_meets(
    corner: &SurfaceMixedCorner,
    band: &Rc<SurfaceStrip>,
    rows: &[Vector3],
    near_enough: f64,
) -> SurfaceRunOutMeetsTheBand {
    SurfaceRunOutMeetsTheBand::try_read(
        Some(corner),
        Some(band),
        the_bands_edge(),
        Some(rows),
        near_enough,
    )
    .unwrap_or_else(|why| {
        panic!(
            "one arc at one chording crosses the other - {:?}",
            why.code()
        )
    })
    .expect("one arc at one chording crosses the other")
}

/// The trim the real crossing makes, which the guards above are asked about, with the pool its
/// vertices stand in.
fn the_real_crossings_trim(prism: &Mesh) -> (SurfaceVertexPool, SurfaceStripTrimmedAtTheCrossing) {
    let band = the_bands_strip(prism, SIZE, false);
    let mut pool = SurfaceVertexPool::new(&prism.vertices);

    let rails = band.rail_count();
    let crossing = vec![
        pool.add(band.point_on(rails - 1, 17.2604)),
        pool.add(band.point_on(rails - 2, 18.6901)),
        pool.add(band.point_on(rails - 3, 20.0)),
    ];

    let trimmed = SurfaceStripTrimmedAtTheCrossing::try_read(
        Some(&pool),
        Some(&band),
        0,
        &the_bands_edge_plan(prism, SIZE),
        Some(&crossing),
        None,
    )
    .unwrap_or_else(|why| panic!("the real crossing has to read: {:?}", why.code()))
    .expect("the real crossing has to read");

    (pool, trimmed)
}

/// The planned edge of the band the skirt crosses.
fn the_bands_edge_plan(prism: &Mesh, size: f64) -> SurfaceEdgePlan {
    let regions = lobed_regions(prism);

    the_one_bands_edge(&the_seam_and_the_one_edge(prism, &regions, size))
}

fn the_one_bands_edge(plans: &[Rc<SurfaceBevelPlan>]) -> SurfaceEdgePlan {
    let found: Vec<&SurfaceEdgePlan> = plans
        .iter()
        .flat_map(|plan| plan.edges.iter())
        .filter(|edge| it_is_the_bands_edge(edge))
        .collect();
    assert_eq!(found.len(), 1);

    found[0].clone()
}

/// The two axes the band's section is read in, borrowed for the closed form above.
fn across(corner: &SurfaceMixedCorner, band: &SurfaceStrip) -> Vector3 {
    let across = band.along().cross(-corner.out_of_the_cap_face).get_normal();

    if across.dot(corner.corner - band.point_on(0, 0.0)) < 0.0 {
        -across
    } else {
        across
    }
}

/// The rail standing at the same height over the cap as a row does, which is the one it crosses
/// at.
fn the_rail_beside(corner: &SurfaceMixedCorner, band: &SurfaceStrip, row: Vector3) -> Vector3 {
    let up = -corner.out_of_the_cap_face;
    let height = (row - corner.outer_rail_centre).dot(up);

    for rail in 0..band.rail_count() {
        if ((band.point_on(rail, 0.0) - corner.outer_rail_centre).dot(up) - height).abs() < 1e-9 {
            return band.point_on(rail, 0.0);
        }
    }

    panic!("every row of the seed stands at some rail's height");
}

/// A point on the band's own edge, which is the apex of the section its end cut takes.
fn the_bands_edge() -> Vector3 {
    Vector3::ZERO
}

/// The centre the band's own ball rolled on, in the wall's plane: a size in from each of the two
/// faces it was tangent to, which on this fixture is `(r, r)` in `(x, z)`.
fn the_ball_centre_of(band: &SurfaceStrip, size: f64) -> Vector2 {
    let last = band.rail_count() - 1;
    let on_the_cap = if band.point_on(0, 0.0).z < band.point_on(last, 0.0).z {
        band.point_on(0, 0.0)
    } else {
        band.point_on(last, 0.0)
    };

    Vector2::new(on_the_cap.x, on_the_cap.z + size)
}

/// The run-out's seed rows as the IDEAL ring stands: the band's own profile, height for height,
/// laid on the circle the bead's axis turns it about. Built here rather than read off a patch so
/// that the crossing is asked of rows whose radii are named rather than measured.
fn the_ideal_seed_rows(corner: &SurfaceMixedCorner, band: &SurfaceStrip) -> Vec<Vector3> {
    let up = -corner.out_of_the_cap_face;
    let across = across(corner, band);
    let mut on_the_cap = band.point_on(0, 0.0);

    for rail in 0..band.rail_count() {
        if (band.point_on(rail, 0.0) - corner.outer_rail_centre).dot(up)
            < (on_the_cap - corner.outer_rail_centre).dot(up)
        {
            on_the_cap = band.point_on(rail, 0.0);
        }
    }

    let out_of_the_axis = corner.out_of_the_axis(corner.turn_radians());

    let mut rows: Vec<Vector3> = (0..band.rail_count())
        .map(|rail| {
            let height = (band.point_on(rail, 0.0) - on_the_cap).dot(up);
            let offset = (band.point_on(rail, 0.0) - on_the_cap).dot(across).abs();

            corner.outer_rail_centre
                + (out_of_the_axis * (corner.outer_rail_radius() - offset))
                + (up * height)
        })
        .collect();

    // From the cap face inward, which is the order a column is read in. Every rail stands at its
    // own height, so the sort has no ties to order.
    rows.sort_by(|first, second| {
        (*first - corner.outer_rail_centre)
            .dot(up)
            .total_cmp(&(*second - corner.outer_rail_centre).dot(up))
    });

    rows
}

/// The mixed corner the barrel's rim turns at the bottom cap's left-hand crossing.
fn the_bottom_caps_corner(prism: &Mesh, size: f64) -> Rc<SurfaceMixedCorner> {
    let regions = lobed_regions(prism);

    for plan in the_seam_and_the_one_edge(prism, &regions, size) {
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

/// The strip of the band the skirt crosses, off whichever pick walks its edge the named way.
fn the_bands_strip(prism: &Mesh, size: f64, walked_the_other_way: bool) -> SurfaceStrip {
    let regions = lobed_regions(prism);

    let plans = if walked_the_other_way {
        plans_for(
            &regions,
            size,
            the_far_side(prism).boundary_edges().to_vec(),
        )
    } else {
        the_seam_and_the_one_edge(prism, &regions, size)
    };

    let edge = the_one_bands_edge(&plans);

    SurfaceStrip::try_create(&edge, 8)
        .unwrap_or_else(|why| panic!("the band has to rule: {:?}", why.code()))
}

/// The bottom edge of the flat side, which is the one band the skirt crosses.
fn it_is_the_bands_edge(edge: &SurfaceEdgePlan) -> bool {
    edge.from().x.abs() < 1e-9
        && edge.to().x.abs() < 1e-9
        && edge.from().z.abs() < 1e-9
        && edge.to().z.abs() < 1e-9
}

/// The pick: the barrel's whole rim - which carries the seam and both mixed corners - and the ONE
/// bottom edge of the flat side, which is what a chain pick of that edge resolves to.
fn the_seam_and_the_one_edge(
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
                    && edge.start.z.abs() < 1e-9
                    && edge.end.z.abs() < 1e-9
            })
            .cloned(),
    );

    plans_for(regions, size, seeds)
}

/// The picked edges, joined into chains and planned at one size.
pub fn plans_for(
    regions: &Arc<FaceRegionCache>,
    size: f64,
    seeds: Vec<FeatureEdge>,
) -> Vec<Rc<SurfaceBevelPlan>> {
    join_and_open(&bevel_chain_linker::link(&seeds))
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, size, true).unwrap_or_else(|reason| {
                panic!("the pick has to plan at {size}: {:?}", reason.code())
            })
        })
        .collect()
}

/// The lobe picked whole - one smooth region of 9 degree facets, which is the cylinder's barrel.
pub fn the_barrel(prism: &Mesh) -> Arc<FaceGroup> {
    region_at(prism, |face| {
        let normal = Vector3::from_vector3_float(prism.faces[face].normal);

        normal.dot(-Vector3::UNIT_Y) > 0.99
            && normal.z.abs() < 1e-6
            && middle_of(prism, face).y < -1e-3
    })
}

/// The flat side the cap runs across to, which is the cube's own side rim in the fixture.
pub fn the_far_side(prism: &Mesh) -> Arc<FaceGroup> {
    region_at(prism, |face| {
        (Vector3::from_vector3_float(prism.faces[face].normal) + Vector3::UNIT_X).length() < 1e-6
    })
}

fn middle_of(prism: &Mesh, face: usize) -> Vector3 {
    let at = prism.faces[face];

    (Vector3::from_vector3_float(prism.vertices[at.v0 as usize])
        + Vector3::from_vector3_float(prism.vertices[at.v1 as usize])
        + Vector3::from_vector3_float(prism.vertices[at.v2 as usize]))
        / 3.0
}

/// The region flooded from the first face answering `seeds`.
fn region_at(prism: &Mesh, seeds: impl Fn(usize) -> bool) -> Arc<FaceGroup> {
    let regions = lobed_regions(prism);

    for face in 0..prism.faces.len() {
        if seeds(face) {
            return regions.region(face as i32).expect("a region");
        }
    }

    panic!("no face answers the seed");
}

pub fn lobed_regions(prism: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ))
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}
