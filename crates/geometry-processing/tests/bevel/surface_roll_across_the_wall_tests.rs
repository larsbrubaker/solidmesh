//! The bevel ROLLED along a polygonal wall, on the half
//! disc's upright seam - the fillet is tangent to the chord face and to the one facet of the arc it
//! actually touches, which past the first facet is not the facet the seam is on
//! (`SurfaceRollAcrossTheWall`).
//!
//! The kernel reading of "Every Chamfer Works.mcx"'s two cylinder-to-cube seams: the cut is made in
//! the plane of the facet the profile really touches, so the kept triangles there have their cut
//! edge ON the tangent line and the strip has a wall run to sew to. Also the crease case on the
//! lobed prism's concave seam (`lobed_prism_meshes`), where the profile rests on a crease and the
//! foot only ever moves forward through the sizes.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use crate::path_solids::extrude;
use crate::path_solids::Outline;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_face_walkback, surface_room, surface_room_measure,
    SurfaceBevelPlan, SurfaceBuildOptions, SurfaceEdgePlan, SurfaceLine, SurfaceStrip,
};
use geometry_processing::bevel::{
    bevel_chain_linker, bevel_edge_tool_builder, BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Vector2, Vector3};

use crate::lobed_prism_meshes;
use crate::topology_test_meshes::create_arc_prism;

/// The bend threshold the fixture is read at, which the arc's facet joins are under.
const THRESHOLD_DEGREES: f64 = 10.0;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 20.0;

/// How near a facet a point has to come to be read as standing on it. A mesh keeps its vertices as
/// floats, so a point worked out in doubles from the shape the mesh was built to lands this far off
/// the triangles that were made of it.
const ON_THE_FACET: f64 = 1e-4;

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The seam of a half disc filleted 3: the profile circle stands 3 from the chord face and 3 from
/// the wall, and where it touches the wall is past the facet the seam is on - so the setback there
/// is measured along the surface and is nothing like the 3 the flat side takes. Worked on paper on
/// the tessellation: the chord face gives up 3.69937 and the wall two whole chords of 1.56918 and
/// 1.14509 of the third, 4.28345 along its surface.
#[test]
fn the_profile_touches_the_facet_it_reaches_and_not_the_one_the_seam_is_on() {
    const SIZE: f64 = 3.0;

    let prism = create_arc_prism(RADIUS, HEIGHT, 20);
    let (plan, _) = seam_plan_of(&prism, SIZE);
    let edge = &plan.edges[0];
    let wall_on_the_left = is_the_wall(edge.left_inward);

    let wall_setback = if wall_on_the_left {
        edge.left_setback
    } else {
        edge.right_setback
    };
    let flat_setback = if wall_on_the_left {
        edge.right_setback
    } else {
        edge.left_setback
    };
    let wall_foot = (if wall_on_the_left {
        edge.left_tangent_line
    } else {
        edge.right_tangent_line
    })
    .closest_point_to(edge.from());
    let centre = edge.center_line.closest_point_to(edge.from());

    let chord = 2.0 * RADIUS * degrees_to_radians(180.0 / 20.0 / 2.0).sin();

    assert!(
        wall_setback > chord,
        "a foot only {wall_setback} along the wall would be on the seam's own facet, whose chord is {chord}"
    );

    // To a thousandth, which is what a mesh holding its vertices as floats can say about 4 mm.
    assert_within(
        wall_setback,
        4.28345,
        0.002,
        &format!("the foot stands two chords and 1.14509 of the third round, and it measured {wall_setback}"),
    );

    assert_within(
        flat_setback,
        3.69937,
        0.002,
        &format!("the flat side is the same circle's other tangency, 10 - 6.30063 from the seam, and it measured {flat_setback}"),
    );

    assert!(
        (wall_setback - flat_setback).abs() > 0.5,
        "the two sides of a rolled edge take DIFFERENT setbacks, which is the whole of what rolling changes"
    );

    // The circle itself: a radius from the chord face, a radius from the facet it touches, and its
    // foot standing inside that facet rather than on the line the facet's plane extends to.
    assert_within(
        centre.y.abs(),
        SIZE,
        1e-9,
        &format!("the chord face is the plane y = 0, and the centre stands at {centre:?}"),
    );

    let touched = facet_under(&prism, wall_foot);

    assert!(
        touched >= 0,
        "the wall foot {wall_foot:?} has to stand on a facet of the wall, inside it and not merely on its plane"
    );

    assert_within(
        distance_to_plane_of(&prism, touched, centre),
        SIZE,
        1e-9,
        "the centre is a radius from the plane of the facet it touches, which is what makes it tangent there",
    );

    assert!(
        distance_to_plane_of(&prism, facet_at_the_seam(&prism), centre) > SIZE + 0.05,
        "and it is NOT a radius from the facet the seam is on, which is the facet the unrolled plan would have used"
    );
}

/// Where the section stays on ONE facet the roll has to come back with today's numbers to the last
/// bit. A half disc tessellated 200 ways has facets 0.157 mm across, so a 0.05 mm fillet touches the
/// facet the seam is on - k = 0 - and every number is the two-plane construction's own closed form.
#[test]
fn a_foot_landing_on_the_seams_own_facet_is_the_unrolled_plan_exactly() {
    const SIZE: f64 = 0.05;

    let prism = create_arc_prism(RADIUS, HEIGHT, 200);
    let (plan, _) = seam_plan_of(&prism, SIZE);
    let edge = &plan.edges[0];

    let half_wedge = degrees_to_radians(edge.wedge_angle_degrees) / 2.0;
    let setback = bevel_edge_tool_builder::tangent_distance(SIZE, edge.wedge_angle_degrees, true);

    assert_within(
        edge.left_setback,
        setback,
        1e-12,
        &format!(
            "the left side is r / tan(half the wedge), and it read {} against {setback}",
            edge.left_setback
        ),
    );

    assert_within(
        edge.right_setback,
        setback,
        1e-12,
        &format!(
            "so is the right, since at k = 0 the wedge's apex IS the edge: it read {}",
            edge.right_setback
        ),
    );

    assert_within(
        edge.center_distance,
        SIZE / half_wedge.sin(),
        1e-12,
        &format!(
            "and the centre stands r / sin(half the wedge) off the edge, at {}",
            edge.center_distance
        ),
    );

    let edge_line = SurfaceLine::new(edge.from(), edge.edge.direction());

    assert_within(
        edge_line.distance_to(edge.left_tangent_line.point),
        setback,
        1e-12,
        "left",
    );
    assert_within(
        edge_line.distance_to(edge.right_tangent_line.point),
        setback,
        1e-12,
        "right",
    );
    assert_within(
        edge_line.distance_to(edge.center_line.point),
        SIZE / half_wedge.sin(),
        1e-12,
        "centre",
    );
}

/// What the cut then does to the wall: the facets the band paced out come away, the facet the
/// profile touches is split at the foot, every other facet of the wall is left exactly as it was,
/// and the strip has a run on the wall to sew to, which is what it never had before.
#[test]
fn the_cut_takes_the_band_it_paced_out_and_leaves_the_rest_of_the_wall() {
    const SIZE: f64 = 3.0;

    let prism = create_arc_prism(RADIUS, HEIGHT, 20);
    let (plan, regions) = seam_plan_of(&prism, SIZE);
    let edge = &plan.edges[0];
    let wall_on_the_left = is_the_wall(edge.left_inward);
    let band = if wall_on_the_left {
        edge.left_band.clone()
    } else {
        edge.right_band.clone()
    };
    let wall_region = if wall_on_the_left {
        &edge.left_region
    } else {
        &edge.right_region
    };

    let band = band.expect(
        "a rolled cut names the facets it may touch, since the half space alone reaches round the far side",
    );

    assert!(
        band.len() > 2,
        "the band reaches past the seam's own two triangles, and it named {} of them",
        band.len()
    );

    assert!(
        band.len() < wall_region.faces().len(),
        "and it is a stretch of the wall rather than the whole of it"
    );

    let walked =
        surface_face_walkback::try_apply_one(&prism, &prism.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|failure| panic!("the walkback refused the seam: {failure}"));

    let survived = triangles_of(&walked.mesh);

    for &facet in wall_region.faces() {
        if band.contains(&facet) {
            continue;
        }

        assert!(
            survived.contains(&name(&corners_of(&prism, facet))),
            "facet {facet} of the wall is outside the band, so the cut may not have touched it"
        );
    }

    // The run the strip sews to: kept triangles of the wall with an edge standing ON the tangent
    // line, which on a wall cut in the plane of the facet it touches is exactly what facet k leaves.
    let runs = walked
        .tangent_runs
        .get(&(0, wall_on_the_left))
        .expect("the wall gathered a run");
    assert!(
        !runs.is_empty(),
        "the wall gathered no run at all before the roll, which is what the strip could not meet"
    );
    assert!(
        runs[0].len() >= 2,
        "a run is an edge at least, and the wall's had {} vertices in it",
        runs[0].len()
    );
}

/// And so the build gets PAST the seam: the strip meets the walkback on both sides, and the ends
/// are cut too (`SurfaceEndFollowsTheWalkedChords`), so the whole seam builds.
#[test]
fn the_strip_meets_the_walkback_and_the_ends_are_cut_too() {
    const SIZE: f64 = 3.0;

    let prism = create_arc_prism(RADIUS, HEIGHT, 20);
    let (plan, regions) = seam_plan_of(&prism, SIZE);

    let built = surface_bevel_builder::try_build(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        &[plan],
        8,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    );

    // The closure and the volume of what it now builds are held by
    // SurfaceEndNotchFollowsTheWalkedChordsTests.
    assert!(
        built.is_ok(),
        "the strip meets the wall and the ends are cut to the wall's own chords: {:?}",
        built.err()
    );
}

/// What the ROOM has to believe about a rolled edge: exactly what the roll takes. It comes off the
/// plan's own two setbacks, one per side, and the walk that measures the region still starts where
/// the EDGE is.
#[test]
fn the_room_reads_what_the_roll_really_takes_on_each_side() {
    const SIZE: f64 = 3.0;

    let prism = create_arc_prism(RADIUS, HEIGHT, 20);
    let (plan, _) = seam_plan_of(&prism, SIZE);
    let edge = &plan.edges[0];

    // One edge, so two bands, and measure makes them left then right. Its two free ends notch the
    // caps beyond them too, which are corner footprints and no band's reading.
    let cuts: Vec<_> = surface_room_measure::measure(&[Rc::clone(&plan)], None, None)
        .into_iter()
        .filter(|cut| !cut.corner_footprint())
        .collect();

    assert_eq!(cuts.len(), 2);

    assert_within(
        cuts[0].setback_per_size * SIZE,
        edge.left_setback,
        1e-12,
        &format!(
            "the left side gives up {}, and the room believed {}",
            edge.left_setback,
            cuts[0].setback_per_size * SIZE
        ),
    );

    assert_within(
        cuts[1].setback_per_size * SIZE,
        edge.right_setback,
        1e-12,
        &format!(
            "the right side gives up {}, and the room believed {}",
            edge.right_setback,
            cuts[1].setback_per_size * SIZE
        ),
    );

    assert!(
        (cuts[0].setback_per_size - cuts[1].setback_per_size).abs() > 0.1,
        "a rolled edge's two sides give up different widths, so one ratio for both of them cannot be right"
    );

    // And the walk still starts at the edge: the wall's cut points along facet k, but the room's
    // own direction is facet 0's, so it measures across the wall rather than from part way round.
    let wall_on_the_left = is_the_wall(edge.left_inward);
    let wall = if wall_on_the_left { cuts[0] } else { cuts[1] };
    let at_the_edge = (if wall_on_the_left {
        edge.left_inward_at_the_edge
    } else {
        edge.right_inward_at_the_edge
    })
    .get_normal();

    assert_within(
        wall.inward.dot(at_the_edge),
        1.0,
        1e-9,
        &format!(
            "the room walks from the edge along facet 0, and it walked along {:?}",
            wall.inward
        ),
    );

    let across_the_wall = 20.0 * 2.0 * RADIUS * degrees_to_radians(4.5).sin();

    assert_within(
        wall.room,
        across_the_wall,
        1e-3,
        &format!(
            "which is the whole half turn of wall, {across_the_wall}, and it measured {}",
            wall.room
        ),
    );
}

/// And the size a refusal names is one that really does fit: it plans, and the room check lets it
/// through rather than turning round and refusing it again. A hair more than it does not plan. On
/// this fixture it is the TANGENCY that binds rather than the room.
#[test]
fn the_size_a_refusal_names_plans_and_passes_the_room() {
    let prism = create_arc_prism(RADIUS, HEIGHT, 20);

    let Err(refusal) = try_seam_plan_of(&prism, 40.0) else {
        panic!("40 does not plan");
    };
    assert_eq!(refusal.code(), BevelAdvisoryCode::SizeExceedsFaceWidth);

    let fits = the_size_it_names(&refusal);

    assert_within(
        fits,
        RADIUS / 2.0,
        0.05,
        &format!("the disc's own inscribed circle is 5 mm, and the refusal named {fits}"),
    );

    let plan = try_seam_plan_of(&prism, fits * 0.999).unwrap_or_else(|reason| {
        panic!("the size a refusal names has to be one that plans, and it said {reason}")
    });

    let fit = surface_room::try_fit(&[plan], fits * 0.999, &CancelToken::new(), None)
        .expect("nothing cancels the fit");
    assert!(
        fit.fits,
        "and one the room lets through, which allowed only {}: {:?}",
        fit.largest_fitting_size,
        fit.refusal.as_ref().map(|reason| reason.code())
    );

    assert!(
        try_seam_plan_of(&prism, fits * 1.01).is_err(),
        "a hair over it is past the tangency, which is what makes the named size the largest"
    );
}

/// Which way the strip faces has to follow the ROLL. A reference taken from the two original face
/// normals is still the bisector of the old sector, and once the roll turns more than half the
/// wedge the far end of the arc leaves the hemisphere it names. The fixture is a 90 degree segment
/// of the same disc; every rail of the strip stands outward of the rolling ball's centre, and the
/// reference must be within a right angle of every one of them.
#[test]
fn a_narrow_wedge_rolled_several_facets_still_faces_outward() {
    const SIZE: f64 = 1.3;

    let segment = segment_prism(RADIUS, HEIGHT, 90.0, 20);
    let (plan, _) = seam_plan_of(&segment, SIZE);
    let edge = &plan.edges[0];

    let wall_setback = if is_the_wall(edge.left_inward) {
        edge.left_setback
    } else {
        edge.right_setback
    };
    let chord = 2.0 * RADIUS * degrees_to_radians(90.0 / 20.0 / 2.0).sin();

    assert!(
        wall_setback > chord * 3.0,
        "the fixture is only worth anything while the profile really rolls, and it went {wall_setback} where a facet is {chord}"
    );

    let strip = SurfaceStrip::try_create(edge, 8).unwrap_or_else(|refused| {
        panic!(
            "the strip has to rule before its winding can be read: {:?}",
            refused.code()
        )
    });

    let centre = edge.center_line.closest_point_to(edge.from());
    let as_the_faces_read = (edge.edge.left_normal() + edge.edge.right_normal()).get_normal();
    let mut any_left_the_old_hemisphere = false;

    for rail in 0..strip.rail_count() {
        let outward = (strip.point_on(rail, 0.0) - centre).get_normal();

        assert!(
            outward.dot(strip.outward_reference) > 0.0,
            "rail {rail} faces {outward:?}, and a strip wound to face {:?} would be inside out there",
            strip.outward_reference
        );

        any_left_the_old_hemisphere |= outward.dot(as_the_faces_read) <= 0.0;
    }

    assert!(
        any_left_the_old_hemisphere,
        "the fixture has to bite: some rail must be more than a right angle off the two faces' own bisector {as_the_faces_read:?}, or this proves nothing"
    );
}

/// The profile resting ON A CREASE of the wall, tangent to neither facet either side of it. On a
/// CONCAVE seam running along a CONVEX wall - the lobed prism's own - at `r = 1.6` facet 0's own
/// solve puts the foot past the part chord's far end and facet 1's puts it before facet 1 begins,
/// and the true circle touches the wall on the vertex between them.
#[test]
fn a_profile_resting_on_a_crease_rolls_rather_than_refusing_the_corner() {
    const SIZE: f64 = 1.6;

    let prism = lobed_prism_meshes::create();
    let plan = lobe_seam_plan_of(&prism, SIZE);
    let edge = &plan.edges[0];
    let wall_on_the_left = the_lobe_is_on_the_left(edge);
    let wall_setback = if wall_on_the_left {
        edge.left_setback
    } else {
        edge.right_setback
    };
    let wall_foot = (if wall_on_the_left {
        edge.left_tangent_line
    } else {
        edge.right_tangent_line
    })
    .closest_point_to(edge.from());
    let centre = edge.center_line.closest_point_to(edge.from());

    let part_chord = (lobed_prism_meshes::disc_corner(first_corner_outside())
        - lobed_prism_meshes::left_crossing())
    .length();

    assert_within(
        wall_setback,
        part_chord,
        1e-3,
        &format!("the foot rests on the crease at the end of the part chord, {part_chord} along, and it read {wall_setback}"),
    );

    // The circle is a radius from the flat face and a radius from the CREASE, and closer than a
    // radius to both facets' planes - which is what "tangent to neither of them" means.
    assert_within(
        (centre - wall_foot).length(),
        SIZE,
        1e-3,
        &format!(
            "the crease vertex {wall_foot:?} is on the circle, and it stands {} from {centre:?}",
            (centre - wall_foot).length()
        ),
    );

    let first = first_corner_outside();

    assert!(
        distance_to_the_chord(
            centre,
            lobed_prism_meshes::left_crossing(),
            lobed_prism_meshes::disc_corner(first)
        ) < SIZE - 1e-3,
        "the seam's own facet is not tangent to it - the circle crosses that plane past the crease"
    );

    assert!(
        distance_to_the_chord(
            centre,
            lobed_prism_meshes::disc_corner(first),
            lobed_prism_meshes::disc_corner(first + 1)
        ) < SIZE - 1e-3,
        "and neither is the next facet round, which the profile has not reached"
    );
}

/// And so the roll is CONTINUOUS in the size across a crease: every radius over the band plans, and
/// the wall foot only ever moves forward - it sits still on the crease while the size grows through
/// the stretch that rests there, then walks on along the next facet.
#[test]
fn the_wall_foot_moves_forward_through_every_size_across_a_crease() {
    let prism = lobed_prism_meshes::create();
    let mut reached = 0.0;
    let mut rested = 0;

    // An accumulating step, so the radii are the same doubles on every run.
    let mut size = 1.0;
    while size <= 5.0001 {
        let plan = try_lobe_seam_plan_of(&prism, size).unwrap_or_else(|reason| {
            panic!("radius {size} is between two that roll, so it has to roll: {reason}")
        });

        let edge = &plan.edges[0];
        let now = if the_lobe_is_on_the_left(edge) {
            edge.left_setback
        } else {
            edge.right_setback
        };

        assert!(
            now >= reached - 1e-9,
            "the foot ran back from {reached} to {now} between two radii a twentieth apart"
        );

        rested += if (now - reached).abs() < 1e-9 { 1 } else { 0 };
        reached = now;
        size += 0.05;
    }

    assert!(
        rested > 5,
        "the sweep only proves anything while it really crosses creases, and the foot stood still on one for {rested} of its steps"
    );
}

/// A CONVEX seam has no such crossing: its facets' windows tile, so the half disc rolls at every
/// size up to the one the tangency itself runs out at.
#[test]
fn a_convex_seam_rolls_at_every_size_below_the_one_it_runs_out_at() {
    let prism = create_arc_prism(RADIUS, HEIGHT, 20);
    let mut reached = 0.0;

    let mut size = 0.1;
    while size <= 4.9001 {
        let plan = try_seam_plan_of(&prism, size).unwrap_or_else(|reason| {
            panic!("a convex seam's windows tile, so radius {size} has to roll: {reason}")
        });

        let edge = &plan.edges[0];
        let now = if is_the_wall(edge.left_inward) {
            edge.left_setback
        } else {
            edge.right_setback
        };

        assert!(
            now > reached,
            "the foot walks forward along a convex wall without ever resting - {reached} then {now}"
        );

        reached = now;
        size += 0.05;
    }
}

/// And where the wall really does run out, the refusal names the size that lands on its LAST facet
/// rather than the largest any facet's own PLANE could take. The sizes are absurd because the lobe
/// is a wall that turns through 157 degrees: it takes a radius of 2598 before the foot reaches the
/// far rim. The named size has to be one that really rolls.
#[test]
fn the_size_a_refusal_names_on_a_curved_wall_is_one_that_rolls() {
    let prism = lobed_prism_meshes::create();

    let Err(refusal) = try_lobe_seam_plan_of(&prism, 2700.0) else {
        panic!("a radius past the lobe's own far rim is one the wall cannot hold");
    };

    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "and what has run out is the wall's width, which is what the reader is told"
    );

    let fits = the_size_it_names(&refusal);

    assert!(
        try_lobe_seam_plan_of(&prism, fits).is_ok(),
        "the size a refusal names has to be one that plans, and {fits} did not"
    );

    assert!(
        try_lobe_seam_plan_of(&prism, fits * 1.001).is_err(),
        "and a hair more than it does not, which is what makes it the largest"
    );
}

/// The size a `SizeExceedsFaceWidth` refusal names, its third argument.
fn the_size_it_names(refusal: &BevelAdvisory) -> f64 {
    match refusal.args().get(2) {
        Some(BevelAdvisoryArg::Double(fits)) => *fits,
        other => panic!("the refusal names a size as its third argument - instead {other:?}"),
    }
}

/// The lobed prism's left seam planned whole, which panics where it cannot be.
fn lobe_seam_plan_of(prism: &Mesh, size: f64) -> Rc<SurfaceBevelPlan> {
    try_lobe_seam_plan_of(prism, size)
        .unwrap_or_else(|reason| panic!("the lobe's seam has to plan at {size}: {reason}"))
}

/// The CONCAVE seam where the lobe pokes through the flat side, planned on its own: a chain edge
/// with the flat wall on one side and the lobe's facets on the other.
fn try_lobe_seam_plan_of(prism: &Mesh, size: f64) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));
    let features = FeatureEdgeSet::create_with_angle(prism, lobed_prism_meshes::THRESHOLD_DEGREES);
    let seam = lobed_prism_meshes::left_crossing();

    let edges: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| at_the_crossing(edge.start, seam) && at_the_crossing(edge.end, seam))
        .copied()
        .collect();

    let mut chains = bevel_chain_linker::link(&edges);
    assert_eq!(chains.len(), 1, "the seam links into one chain");

    SurfaceBevelPlan::try_create(&chains.remove(0), &regions, size, true)
}

/// Whether a point stands on the upright seam the lobe makes where it crosses the flat side.
fn at_the_crossing(point: Vector3, seam: Vector2) -> bool {
    (point.x - seam.x).abs() < 1e-3 && (point.y - seam.y).abs() < 1e-3
}

/// Whether the LOBE is the left side of that seam's walk - the side whose inward leaves y = 0.
fn the_lobe_is_on_the_left(edge: &SurfaceEdgePlan) -> bool {
    edge.left_inward_at_the_edge.get_normal().y.abs() > 0.5
}

/// The first corner of the disc standing OUTSIDE the flat side, which the part chord runs to.
fn first_corner_outside() -> i32 {
    (0..lobed_prism_meshes::SIDES)
        .filter(|&corner| lobed_prism_meshes::disc_corner(corner).y < 0.0)
        .min()
        .expect("the disc crosses the flat side")
}

/// How far a point stands off the PLANE of one chord of the lobe, which on a prism is the chord's
/// own line in the outline read at any height.
fn distance_to_the_chord(point: Vector3, from: Vector2, to: Vector2) -> f64 {
    let along = (to - from).get_normal();
    let across = Vector2::new(-along.y, along.x);

    (Vector2::new(point.x, point.y) - from).dot(across).abs()
}

/// A prism on a SEGMENT of a disc: the tessellated arc through `sweep_degrees` closed by the chord
/// back to where it started. `create_arc_prism` is this at 180 degrees; a shallower sweep is what
/// makes the seam a NARROW wedge.
fn segment_prism(radius: f64, height: f64, sweep_degrees: f64, segments: i32) -> Mesh {
    let mut path = Outline::new();
    let sweep = degrees_to_radians(sweep_degrees);

    path.move_to(radius, 0.0);

    for i in 1..=segments {
        let angle = sweep * f64::from(i) / f64::from(segments);

        path.line_to(radius * angle.cos(), radius * angle.sin());
    }

    path.close_polygon();

    extrude(&path, height).expect("the segment extrudes")
}

/// Whether an in-face inward direction is the WALL's - it leaves the chord plane, and the chord
/// face's does not.
fn is_the_wall(inward: Vector3) -> bool {
    inward.y.abs() > 0.5
}

/// The plan for the seam at (10, 0), flooded at the fixture's own 10 degree threshold, with the
/// regions it was planned on.
fn seam_plan_of(mesh: &Mesh, size: f64) -> (Rc<SurfaceBevelPlan>, Arc<FaceRegionCache>) {
    let regions = regions_of(mesh);
    let plan = try_seam_plan_on(mesh, &regions, size)
        .unwrap_or_else(|reason| panic!("the seam has to plan: {reason}"));

    (plan, regions)
}

/// The same, where the refusal is what is being read.
fn try_seam_plan_of(mesh: &Mesh, size: f64) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
    try_seam_plan_on(mesh, &regions_of(mesh), size)
}

fn regions_of(mesh: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        THRESHOLD_DEGREES,
    ))
}

fn try_seam_plan_on(
    mesh: &Mesh,
    regions: &Arc<FaceRegionCache>,
    size: f64,
) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
    let features = FeatureEdgeSet::create_with_angle(mesh, THRESHOLD_DEGREES);

    let seam: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| at_the_seam(edge.start) && at_the_seam(edge.end))
        .copied()
        .collect();

    let mut chains = bevel_chain_linker::link(&seam);
    assert_eq!(chains.len(), 1, "the seam links into one chain");

    SurfaceBevelPlan::try_create(&chains.remove(0), regions, size, true)
}

/// Whether a point stands on the half of the chord plane the seam is on.
fn at_the_seam(point: Vector3) -> bool {
    point.y.abs() < 1e-3 && point.x > 1e-3
}

/// The facet of the mesh the point stands INSIDE, or -1 where it stands on none of them.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
fn facet_under(mesh: &Mesh, point: Vector3) -> i32 {
    for face in 0..mesh.faces.len() as i32 {
        let corners = corners_of(mesh, face);
        let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);

        if !(normal.length_squared() > 0.0) {
            continue;
        }

        let normal = normal.get_normal();

        if (point - corners[0]).dot(normal).abs() > ON_THE_FACET {
            continue;
        }

        let mut inside = true;

        for i in 0..3 {
            inside &= (corners[(i + 1) % 3] - corners[i])
                .cross(point - corners[i])
                .dot(normal)
                >= -ON_THE_FACET;
        }

        if inside {
            return face;
        }
    }

    -1
}

/// The facet of the wall the seam itself lies in, which is the one the unrolled plan would cut in.
fn facet_at_the_seam(mesh: &Mesh) -> i32 {
    // The middle of the first chord, which is on the facet rather than out on the circle it spans.
    let step = std::f64::consts::PI / 20.0;

    facet_under(
        mesh,
        Vector3::new(
            RADIUS * (1.0 + step.cos()) / 2.0,
            RADIUS * step.sin() / 2.0,
            HEIGHT / 2.0,
        ),
    )
}

/// How far a point stands off the plane of one facet.
fn distance_to_plane_of(mesh: &Mesh, face: i32, point: Vector3) -> f64 {
    let corners = corners_of(mesh, face);
    let normal = (corners[1] - corners[0])
        .cross(corners[2] - corners[0])
        .get_normal();

    (point - corners[0]).dot(normal).abs()
}

fn corners_of(mesh: &Mesh, face: i32) -> [Vector3; 3] {
    let facet = &mesh.faces[face as usize];

    [facet.v0, facet.v1, facet.v2].map(|at| Vector3::from_vector3_float(mesh.vertices[at as usize]))
}

/// Every triangle of a mesh by its three corner POSITIONS, since the rewrite renumbers.
fn triangles_of(mesh: &Mesh) -> HashSet<String> {
    (0..mesh.faces.len() as i32)
        .map(|face| name(&corners_of(mesh, face)))
        .collect()
}

/// One triangle's three corners, rounded and sorted, as the name it answers to either side of a
/// rewrite.
fn name(corners: &[Vector3; 3]) -> String {
    let mut named: Vec<String> = corners
        .iter()
        .map(|corner| format!("({:.4}, {:.4}, {:.4})", corner.x, corner.y, corner.z))
        .collect();

    named.sort();

    named.join(" ")
}
