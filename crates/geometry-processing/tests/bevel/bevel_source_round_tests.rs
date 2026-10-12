//! THE ROUND THE USER MODELLED, read off the source mesh and rolled on: the two pieces that let a
//! fillet on the crease where a 40 sided post meets a flat wall stand on the POST rather than on
//! the plane of the one facet of it the crease edge happens to lie in. Ports
//! `Tests/GeometryProcessingTests/BevelSourceRoundTests.cs`, over `bevel_source_rounds::try_fit`,
//! `BevelRoundFaces` and `BevelRoundDescriptor`; its meshes are `source_round_test_meshes.rs`.
//!
//! The fixture is the "Unexpected Gap In Fillet" part reduced to numbers: a radius 10 prism drawn
//! with 40 chords, its axis along z through (5, -4), standing in a wall whose face is the plane
//! y = -5. The crease is therefore PARALLEL to the post's own axis, which is the case
//! `BevelRoundDescriptor::try_find_station` could not answer at all before - its scan walks the
//! spine round the axis, and here the spine is a straight line running along it.
//!
//! Where the numbers come from. The ball of radius 2 keeps its centre 2 out from the wall, so at
//! y = -7, and 10 + 2 from the axis, so 12 from (5, -4): the two constraints are a line and a
//! circle in the plane square to z, meeting at x = 5 +- sqrt(12^2 - 3^2) = 5 +- 11.6190. The
//! crease the section is anchored at is the one at x = 5 - sqrt(10^2 - 1^2) = -4.9499, so the
//! root wanted is the near one, x = -6.6190. The contact is that same radial at a distance of 10
//! rather than 12, which is (-4.6825, -6.5).

use std::f64::consts::PI;
use std::time::Instant;

use geometry_processing::bevel::bevel_source_rounds;
use geometry_processing::bevel::{BevelRoundDescriptor, BevelRoundFaces};
use geometry_processing::topology::{EdgeConvexity, FaceGroup, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{math_helper, Matrix4X4, Vector2, Vector3};

use crate::source_round_test_meshes;

const SIDES: i32 = 40;
const POST_RADIUS: f64 = 10.0;
const BALL_RADIUS: f64 = 2.0;

/// The wall's face, with its material below it.
const WALL_Y: f64 = -5.0;

/// Where the post's axis runs, in x and y.
const AXIS: Vector2 = Vector2::new(5.0, -4.0);

/// Where the post's TRUE cylinder crosses the wall on the near side.
fn crease_x() -> f64 {
    AXIS.x - ((POST_RADIUS * POST_RADIUS) - ((AXIS.y - WALL_Y) * (AXIS.y - WALL_Y))).sqrt()
}

/// The ball centre's distance across from the axis, square to the wall.
fn across() -> f64 {
    (((POST_RADIUS + BALL_RADIUS) * (POST_RADIUS + BALL_RADIUS))
        - (((AXIS.y - WALL_Y) + BALL_RADIUS) * ((AXIS.y - WALL_Y) + BALL_RADIUS)))
        .sqrt()
}

/// The prism's own wall is fitted to the cylinder it was drawn from, on a mesh no boolean has been
/// near - so the fixture below can be read as measuring the fit rather than the union.
///
/// A ten thousandth rather than anything finer because `Mesh` stores its vertices as SINGLE
/// precision: a coordinate of ten already carries about a millionth of slop before anything is
/// asked of it, and the algebraic fit spreads that over forty of them. It is four orders inside
/// the residual the fit refuses at, which is what matters.
#[test]
fn the_prism_wall_is_fitted_to_the_cylinder_it_was_drawn_from() {
    let mut post = source_round_test_meshes::create_polygon_prism(SIDES, POST_RADIUS, 40.0);
    post.translate(Vector3::new(AXIS.x, AXIS.y, 0.0));

    let surfaces = FaceGroup::label_surfaces(
        &post,
        &post.get_mesh_edge_graph(),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    );

    // The wall is the one surface with a facet for every chord; the two caps are one face each.
    // (LINQ's GroupBy keeps first-appearance order and its OrderByDescending is stable, so the
    // first label to reach the largest count wins.)
    let mut counts: Vec<(i32, usize)> = Vec::new();
    for &label in &surfaces {
        match counts.iter_mut().find(|(seen, _)| *seen == label) {
            Some((_, count)) => *count += 1,
            None => counts.push((label, 1)),
        }
    }
    let wall_label = counts
        .iter()
        .fold(None::<(i32, usize)>, |best, &(label, count)| match best {
            Some((_, most)) if most >= count => best,
            _ => Some((label, count)),
        })
        .map(|(label, _)| label)
        .unwrap();

    let round = bevel_source_rounds::try_fit(
        &post,
        &faces_of(&surfaces, wall_label),
        Vector3::new(crease_x(), WALL_Y, 0.0),
    )
    .expect("the prism's wall fits");

    assert!(round.axis_direction.z.abs() > 1.0 - 1e-9);
    assert!(round.axis_direction.x.abs() + round.axis_direction.y.abs() < 1e-9);
    assert!((round.axis_point.x - AXIS.x).abs() < 1e-4);
    assert!((round.axis_point.y - AXIS.y).abs() < 1e-4);
    assert!((round.radius - POST_RADIUS).abs() < 1e-4);
    assert_eq!(round.segments, SIDES);
}

/// The post really is read as the cylinder it is - axis, centre, radius and chord count - and the
/// labelling puts its facets on the round while leaving the wall's own face off it, which is what
/// makes the crease between them a round against a plane for the sweep.
///
/// Held to a thousandth rather than the millionth the raw prism above is held to, because the
/// union that made this fixture snaps its output to a grid: the fitted radius comes back 6e-5 wide
/// of the prism it was cut from, and no amount of care in the fit can put that back.
#[test]
fn the_post_in_the_wall_is_read_as_the_cylinder_it_is() {
    let part = source_round_test_meshes::create_post_in_a_wall(SIDES, POST_RADIUS, AXIS, WALL_Y);
    let (crease, surfaces) = the_crease(&part);
    let midpoint = (crease.start + crease.end) * 0.5;

    let post = bevel_source_rounds::try_fit(
        &part,
        &faces_of(&surfaces, surfaces[crease.face0 as usize]),
        midpoint,
    );
    let wall = bevel_source_rounds::try_fit(
        &part,
        &faces_of(&surfaces, surfaces[crease.face1 as usize]),
        midpoint,
    );

    // Face0 and Face1 come off the mesh in whichever order the triangulation left them, so
    // whichever of the two fitted is the post - and exactly one of them may.
    assert!(!(post.is_some() && wall.is_some()));
    let round_is_post = post.is_some();
    let round = post.or(wall).expect("one side of the crease fits");

    // Along z either way round: the fit has no way to know which end of its own axis is which, and
    // nothing downstream cares.
    assert!(round.axis_direction.z.abs() > 1.0 - 1e-9);
    assert!(round.axis_direction.x.abs() + round.axis_direction.y.abs() < 1e-9);

    // The axis point may sit anywhere ALONG the axis, so it is the square offset that is the
    // answer.
    assert!((round.axis_point.x - AXIS.x).abs() < 1e-3);
    assert!((round.axis_point.y - AXIS.y).abs() < 1e-3);
    assert!((round.radius - POST_RADIUS).abs() < 1e-3);
    assert_eq!(round.segments, SIDES);

    // AND THE LABELLING, which is what the sweep actually asks of all this.
    let labelled = BevelRoundFaces::label(&part, std::slice::from_ref(&round), BALL_RADIUS, false);
    let (post_face, wall_face) = if round_is_post {
        (crease.face0, crease.face1)
    } else {
        (crease.face1, crease.face0)
    };

    let (_, is_tangent_plane) = labelled
        .try_read_face(post_face)
        .expect("the post's facet is on the round");
    assert!(!is_tangent_plane);
    assert!(labelled.try_read_face(wall_face).is_none());
}

/// The station for a crease PARALLEL to the axis, which is a line and a circle rather than a
/// scan: the ball stands 2 off the wall and 12 from the axis, on the crease's own side of the
/// post.
#[test]
fn the_station_for_a_crease_along_the_axis_is_the_rolling_ball() {
    let round = the_post_as_a_round();
    let anchor = Vector3::new(crease_x(), WALL_Y, 3.0);

    let station = round
        .try_find_station(
            anchor,
            Vector3::new(0.0, WALL_Y, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            BALL_RADIUS,
        )
        .expect("the ball has a station");

    assert!(
        (station.centre - Vector3::new(AXIS.x - across(), WALL_Y - BALL_RADIUS, anchor.z)).length()
            < 1e-9
    );

    // The ball touches the post where its own radial crosses the true cylinder.
    let radial =
        Vector3::new(station.centre.x - AXIS.x, station.centre.y - AXIS.y, 0.0).get_normal();
    assert!(
        (station.contact - (Vector3::new(AXIS.x, AXIS.y, anchor.z) + (radial * POST_RADIUS)))
            .length()
            < 1e-9
    );

    // The spine is straight and runs along the axis.
    assert!((station.tangent.z.abs() - 1.0).abs() < 1e-9);

    // And the station's own angle really is where that radial points, in the round's basis.
    assert!((round.radial(station.phi) - radial).length() < 1e-9);
}

/// The FAR root is not taken. Both intersections of the wall's offset plane with the spine
/// cylinder are real, and the one wanted is on the side of the post the crease is on - which is
/// the whole of what stops a fillet on one crease being built against the other.
#[test]
fn the_station_is_taken_on_the_crease_own_side_of_the_post() {
    let round = the_post_as_a_round();
    let far = (2.0 * AXIS.x) - crease_x();

    let station = round
        .try_find_station(
            Vector3::new(far, WALL_Y, 3.0),
            Vector3::new(0.0, WALL_Y, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            BALL_RADIUS,
        )
        .expect("the ball has a station");

    assert!(station.centre.x > AXIS.x);
}

/// THE PART MAY SIT ANYWHERE. The same post in the same wall, turned 17 degrees about z and
/// carried out to (100, 100), still gives the station the analytic rolling ball gives.
///
/// Not a redundant re-run of the fixture above: every number in the fit - the algebraic circle
/// most of all - is quadratic in the coordinates it is handed, so a part a hundred millimetres out
/// and turned off the axes is where a fit that only works at the origin gives itself away. The
/// station itself is held to a ten thousandth of the analytic answer, which is the arithmetic and
/// not the placement.
#[test]
fn the_station_is_found_wherever_the_part_sits_in_space() {
    let turn = Matrix4X4::create_rotation_z(math_helper::degrees_to_radians(17.0));
    let place = turn * Matrix4X4::create_translation(100.0, 100.0, 0.0);

    let mut part =
        source_round_test_meshes::create_post_in_a_wall(SIDES, POST_RADIUS, AXIS, WALL_Y);
    part.transform(place);

    let graph = part.get_mesh_edge_graph();
    let surfaces =
        FaceGroup::label_surfaces(&part, &graph, FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES);
    let features =
        FeatureEdgeSet::create_with_graph(&part, &graph, FeatureEdge::default_flat_angle_degrees());
    let crease_foot = Vector3::new(crease_x(), WALL_Y, 0.0).transform(place);

    // The long vertical crease nearest where the near-side one was before the part was moved.
    // Found rather than assumed, as above: the boolean decides which mesh edges are there.
    // (LINQ's OrderBy is stable, so the first of equal distances wins.)
    let crease = features
        .edges()
        .iter()
        .filter(|edge| {
            edge.convexity == EdgeConvexity::Concave
                && (edge.direction.z.abs() - 1.0).abs() < 1e-6
                && (edge.end - edge.start).length() > 10.0
        })
        .fold(None::<(&FeatureEdge, f64)>, |best, edge| {
            let distance =
                Vector2::new(edge.start.x - crease_foot.x, edge.start.y - crease_foot.y).length();
            match best {
                Some((_, nearest)) if nearest <= distance => best,
                _ => Some((edge, distance)),
            }
        })
        .map(|(edge, _)| *edge)
        .expect("the part has a long vertical crease");

    let midpoint = (crease.start + crease.end) * 0.5;
    let round = bevel_source_rounds::try_fit(
        &part,
        &faces_of(&surfaces, surfaces[crease.face0 as usize]),
        midpoint,
    )
    .or_else(|| {
        bevel_source_rounds::try_fit(
            &part,
            &faces_of(&surfaces, surfaces[crease.face1 as usize]),
            midpoint,
        )
    })
    .expect("one side of the crease fits");

    let station = round
        .try_find_station(
            Vector3::new(crease_x(), WALL_Y, 3.0).transform(place),
            Vector3::new(0.0, WALL_Y, 0.0).transform(place),
            Vector3::new(0.0, -1.0, 0.0)
                .transform_vector(turn)
                .get_normal(),
            BALL_RADIUS,
        )
        .expect("the ball has a station");

    let expected_centre =
        Vector3::new(AXIS.x - across(), WALL_Y - BALL_RADIUS, 3.0).transform(place);
    assert!((station.centre - expected_centre).length() < 1e-4);

    // And the contact is a ball radius in from it, on the post.
    assert!(((station.centre - station.contact).length() - BALL_RADIUS).abs() < 1e-4);
}

/// AN AXIS A FEW PARTS IN A MILLION OUT OF SQUARE still answers. The wall of a real part is
/// parallel to the bore's axis by intent, but the axis here is FITTED - through vertices a `Mesh`
/// stores in single precision, and a fit is accepted while its facets are square to it only to
/// 1e-3 - so the number that comes back is rarely the exact zero the closed form was gated on.
///
/// What made this worth a test of its own rather than a tolerance nudge: the failure was SILENT
/// and total. Past the exact gate the solve fell into the scan, which slides the spine along the
/// axis by one over that same tiny number - millions of radii - so it bracketed nothing, refused
/// the station, and the crease quietly went back to the planar section the source rounds exist to
/// replace. Nothing downstream can tell that apart from a crease that never touched a round.
#[test]
fn the_station_survives_an_axis_slightly_out_of_square_with_the_wall() {
    const OUT_OF_SQUARE: f64 = 5e-6;

    let axis_point = Vector3::new(AXIS.x, AXIS.y, 0.0);
    let direction = Vector3::new(0.0, OUT_OF_SQUARE, 1.0).get_normal();
    let wall_normal = Vector3::new(0.0, -1.0, 0.0);

    // Square to the fitted axis, as every radial of the descriptor's own basis is.
    let mut crease_radial = Vector3::new(crease_x(), WALL_Y, 0.0) - axis_point;
    crease_radial = crease_radial - (direction * crease_radial.dot(direction));

    let round = BevelRoundDescriptor::with_span(
        axis_point,
        direction,
        POST_RADIUS,
        crease_radial.get_normal(),
        -PI,
        PI,
        -20.0,
        20.0,
        SIDES,
    );

    let station = round.try_find_station(
        Vector3::new(crease_x(), WALL_Y, 3.0),
        Vector3::new(0.0, WALL_Y, 0.0),
        wall_normal,
        BALL_RADIUS,
    );

    let station = station.unwrap_or_else(|| {
        panic!(
            "a wall {:e} off parallel to the axis is the ordinary case, not a crease that has \
             left the round",
            direction.dot(wall_normal).abs()
        )
    });

    assert!(
        (station.centre - Vector3::new(AXIS.x - across(), WALL_Y - BALL_RADIUS, 3.0)).length()
            < 1e-4
    );

    // The spine still runs along the axis, which is what the section is drawn square to.
    assert!(station.tangent.dot(direction).abs() > 1.0 - 1e-9);
}

/// TWO SURFACES OF ONE CYLINDER MERGE TO A SPAN COVERING BOTH. A boss wall cut in two by a slot
/// is two smooth surfaces of the same cylinder, each described about its own crease, and the
/// merged descriptor has to keep the whole of both spans - a ball anchored on either half must
/// still find its station.
///
/// The span of a round read off the SOURCE mesh runs either side of its first radial, because that
/// radial is the crease the surface was found through. Rebuilding a merge through the two-radial
/// constructor - which reads a span as running between the two radials it is handed - threw half
/// of that away, and the half thrown away was refused a station without a word.
#[test]
fn two_surfaces_of_one_cylinder_merge_to_a_span_covering_both() {
    let axis_point = Vector3::new(AXIS.x, AXIS.y, 0.0);
    let direction = Vector3::new(0.0, 0.0, 1.0);

    // The same 120 degree arc described twice, about two creases 30 degrees apart, over two
    // stretches of the axis that meet.
    let lower = BevelRoundDescriptor::with_span(
        axis_point,
        direction,
        POST_RADIUS,
        at(0.0),
        math_helper::degrees_to_radians(-60.0),
        math_helper::degrees_to_radians(60.0),
        0.0,
        10.0,
        SIDES,
    );
    let upper = BevelRoundDescriptor::with_span(
        axis_point,
        direction,
        POST_RADIUS,
        at(30.0),
        math_helper::degrees_to_radians(-90.0),
        math_helper::degrees_to_radians(30.0),
        10.0,
        20.0,
        SIDES,
    );

    let mut rounds = Vec::new();
    BevelRoundFaces::add(&mut rounds, lower, 1e-3);
    BevelRoundFaces::add(&mut rounds, upper, 1e-3);

    assert_eq!(rounds.len(), 1);
    let merged = &rounds[0];

    // The axial extent covers both, which is what a merge is for.
    assert!(merged.contains(
        axis_point + (at(0.0) * POST_RADIUS) + Vector3::new(0.0, 0.0, 18.0),
        1e-6,
        1e-6
    ));

    // AND BOTH HALVES OF THE SPAN ANSWER. The wall's face stands out from the axis along the first
    // radial far enough that the post breaks through it either side of that radial, and a ball of
    // radius 2 rolling in either of those two creases stands at +-50 degrees - one either side of
    // the first radial, which is the whole point.
    let stand_off = (POST_RADIUS + BALL_RADIUS) * math_helper::degrees_to_radians(50.0).cos();

    for side in [1.0, -1.0] {
        let crease_angle = side * ((stand_off - BALL_RADIUS) / POST_RADIUS).acos();

        let station = merged
            .try_find_station(
                axis_point + (radial(crease_angle) * POST_RADIUS) + Vector3::new(0.0, 0.0, 4.0),
                axis_point + (at(0.0) * (stand_off - BALL_RADIUS)),
                at(0.0),
                BALL_RADIUS,
            )
            .unwrap_or_else(|| {
                panic!(
                    "the ball anchored at {:.1} degrees is on the merged round's span and has to \
                     have a station on it",
                    math_helper::radians_to_degrees(crease_angle)
                )
            });

        assert!((math_helper::radians_to_degrees(station.phi) - (side * 50.0)).abs() <= 1e-6);
    }
}

/// A SLIVER OF A CYLINDER IS REFUSED. Three or four surviving facets of a forty chord post report
/// forty segments just as the whole post does - the count is read off the MEDIAN turn between
/// neighbouring facets, which says how finely the surface was drawn and nothing about how much of
/// it is left - and the few points they leave lie on an unbounded family of circles.
#[test]
fn a_sliver_of_a_cylinder_is_too_little_to_fit() {
    let mut post = source_round_test_meshes::create_polygon_prism(SIDES, POST_RADIUS, 40.0);
    post.translate(Vector3::new(AXIS.x, AXIS.y, 0.0));

    for facets in [3, 4] {
        let sliver = wall_facets(&post, facets);

        assert!(
            bevel_source_rounds::try_fit(&post, &sliver, through(&post, &sliver)).is_none(),
            "{facets} facets of a {SIDES} chord post do not say what cylinder they came from"
        );
    }

    // NOT VACUOUS: the very same selection, taken wide enough to have fitted through, still fits.
    let enough = wall_facets(&post, 10);
    assert!(bevel_source_rounds::try_fit(&post, &enough, through(&post, &enough)).is_some());
}

/// THE FIT DOES NOT WALK EVERY PAIR OF A SURFACE'S FACETS. A twenty thousand facet planar surface
/// is refused - it is a plane, not a cylinder - and refusing it is one pass over the normals rather
/// than the two hundred million cross products every PAIR of them would be.
///
/// A planar wall is the worst case on purpose: it is what most of the surfaces this is asked about
/// are, and the pair scan this stands against could only refuse one after examining all of them.
/// Two seconds is far above the few milliseconds the one pass takes and far below the pair scan,
/// so the bound says which of the two is running without being a stopwatch on the machine's mood.
/// (C#'s `[Timeout(120_000)]` has no libtest equivalent; the two-second bound is the test.)
#[test]
fn a_huge_surface_is_answered_in_one_pass_over_its_facets() {
    const ACROSS: i32 = 100;

    let mut wall = Mesh::new();

    for x in 0..ACROSS {
        for y in 0..ACROSS {
            let corner = Vector3::new(x as f64, y as f64, 0.0);
            wall.create_face(&[
                corner,
                corner + Vector3::new(1.0, 0.0, 0.0),
                corner + Vector3::new(1.0, 1.0, 0.0),
            ]);
            wall.create_face(&[
                corner,
                corner + Vector3::new(1.0, 1.0, 0.0),
                corner + Vector3::new(0.0, 1.0, 0.0),
            ]);
        }
    }

    let faces: Vec<i32> = (0..wall.faces.len() as i32).collect();
    assert_eq!(faces.len(), (2 * ACROSS * ACROSS) as usize);

    let stopwatch = Instant::now();
    let fitted = bevel_source_rounds::try_fit(&wall, &faces, Vector3::new(50.0, 50.0, 0.0));
    let elapsed = stopwatch.elapsed().as_secs_f64();

    assert!(fitted.is_none());
    assert!(
        elapsed < 2.0,
        "{} facets took {elapsed:.3}s to refuse, which is the shape of a scan over every PAIR of \
         their normals",
        faces.len()
    );
}

/// The unit radial at an angle from the post's axis, in degrees.
fn at(degrees: f64) -> Vector3 {
    radial(math_helper::degrees_to_radians(degrees))
}

/// The unit radial at an angle from the post's axis, in radians.
fn radial(angle: f64) -> Vector3 {
    Vector3::new(angle.cos(), angle.sin(), 0.0)
}

/// A contiguous run of the prism's wall facets, taken as the two triangles of each of the chords
/// nearest the +x radial - which is a boolean's sliver in everything but how it was made.
fn wall_facets(post: &Mesh, chords: usize) -> Vec<i32> {
    let mut wall: Vec<(i32, f64)> = Vec::new();

    for (face, mesh_face) in post.faces.iter().enumerate() {
        let a = Vector3::from_vector3_float(post.vertices[mesh_face.v0 as usize]);
        let b = Vector3::from_vector3_float(post.vertices[mesh_face.v1 as usize]);
        let c = Vector3::from_vector3_float(post.vertices[mesh_face.v2 as usize]);
        let normal = (b - a).cross(c - a).get_normal();

        if normal.z.abs() > 1e-6 {
            // A cap, not the wall.
            continue;
        }

        wall.push((face as i32, normal.y.atan2(normal.x)));
    }

    // LINQ's OrderBy(|angle|).ThenBy(face): a total order, so the stable sort is the same one.
    wall.sort_by(|one, other| {
        one.1
            .abs()
            .total_cmp(&other.1.abs())
            .then(one.0.cmp(&other.0))
    });

    let mut taken: Vec<i32> = wall.iter().take(2 * chords).map(|facet| facet.0).collect();
    taken.sort();

    taken
}

/// A point of the surface, standing in for the crease a fit would be measured about.
fn through(post: &Mesh, faces: &[i32]) -> Vector3 {
    Vector3::from_vector3_float(
        post.vertices[post.faces[faces[faces.len() / 2] as usize].v0 as usize],
    )
}

/// The post as the descriptor a fit of it produces, without going through a mesh for it.
fn the_post_as_a_round() -> BevelRoundDescriptor {
    let axis_point = Vector3::new(AXIS.x, AXIS.y, 0.0);
    let first_radial = (Vector3::new(crease_x(), WALL_Y, 0.0) - axis_point).get_normal();

    BevelRoundDescriptor::with_span(
        axis_point,
        Vector3::new(0.0, 0.0, 1.0),
        POST_RADIUS,
        first_radial,
        -PI,
        PI,
        -20.0,
        20.0,
        SIDES,
    )
}

/// The vertical crease on the near side, where the post meets the wall, with the part's surface
/// labels. Found rather than assumed, because the boolean decides which mesh edges are there.
/// (LINQ's OrderByDescending is stable, so the first of equal lengths wins.)
fn the_crease(part: &Mesh) -> (FeatureEdge, Vec<i32>) {
    let graph = part.get_mesh_edge_graph();
    let surfaces =
        FaceGroup::label_surfaces(part, &graph, FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES);
    let features =
        FeatureEdgeSet::create_with_graph(part, &graph, FeatureEdge::default_flat_angle_degrees());

    let crease = features
        .edges()
        .iter()
        .filter(|edge| {
            edge.convexity == EdgeConvexity::Concave
                && (edge.direction.z.abs() - 1.0).abs() < 1e-6
                && edge.start.x < AXIS.x
        })
        .fold(None::<(&FeatureEdge, f64)>, |best, edge| {
            let length = (edge.end - edge.start).length();
            match best {
                Some((_, longest)) if longest >= length => best,
                _ => Some((edge, length)),
            }
        })
        .map(|(edge, _)| *edge)
        .expect("the part has a near-side vertical crease");

    (crease, surfaces)
}

/// Every face carrying one surface label.
fn faces_of(surfaces: &[i32], label: i32) -> Vec<i32> {
    surfaces
        .iter()
        .enumerate()
        .filter(|(_, &surface)| surface == label)
        .map(|(face, _)| face as i32)
        .collect()
}
