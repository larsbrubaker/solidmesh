//! Port of `SurfaceRakedBoreRollingBallTests.cs`: where the rolling ball really runs round the
//! RAKED bore's mouth (`bore_test_meshes::create_the_raked_fixture`), read against the wall itself
//! and contrasted with the straight bore. Nothing here reads the plan: the wall is taken straight
//! off the fixture's own geometry - forty half strips ruled down the drill's axis - and the ball is
//! placed by halving rather than by any closed form, so the numbers are a measurement of the PART
//! and not downstream of the construction they are used to choose.

use std::collections::{HashMap, HashSet};

use geometry_processing::topology::{FeatureEdge, FeatureEdgeSet};
use vector_math::Vector3;

use crate::bore_test_meshes::{
    create_plate_with_a_round_bore, create_the_raked_fixture, raked_drill_axis, RAKED_RADIUS,
    RAKED_SIDES, RAKED_THICKNESS,
};

/// The size the whole raked reading is taken at, which is the one the fixture is built for.
const SIZE: f64 = 0.4;

/// The bend threshold "Chain and Face Should Both work.mcx" carries, at which a forty sided bore's
/// nine degree creases are not feature edges and the mouth walks all the way round as one chain.
const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// The straight bore this reading is contrasted against - `surface_bore_mouth_tests`' own.
const STRAIGHT_RADIUS: f64 = 3.0;

const STRAIGHT_THICKNESS: f64 = 10.0;

/// The arc at a mouth vertex is the ball PIVOTING ABOUT THE CREASE, and it is exact: every station
/// of it stands a radius from the crease LINE and a radius below the top face. Read against the
/// rail of the facet the ball actually TOUCHES, the true contact stands off it by nothing at all.
/// And the pivot criterion does not choose: it holds at every crease station of both bores; what
/// separates them is the DRIFT - 1 on the straight bore and 2 on the raked one.
fn the_arc_at_a_mouth_vertex_is_the_ball_pivoting_about_that_crease(
    raked: bool,
    crease_stations: i32,
    corners_carrying_an_arc: usize,
    worst_drift: i32,
) {
    let bore = the_mouth_of(raked);
    let corners = &bore.corners;
    let top = bore.top;

    let stations = 7200;
    let mut on_a_crease = 0;
    let mut worst_off_the_ellipse: f64 = 0.0;
    let mut worst_off_the_height: f64 = 0.0;
    let mut both_feet_outside = 0;
    let mut either_foot_inside = 0;
    let mut worst_off_its_own_facets_rail: f64 = 0.0;
    let mut drift = 0;

    let mut carrying: HashSet<usize> = HashSet::new();

    for s in 0..stations {
        let centre = the_rolling_balls_centre_at(
            &bore,
            2.0 * std::f64::consts::PI * s as f64 / stations as f64,
        );

        let reading = to_the_raked_wall(&bore, centre);
        let feature = reading.facet;

        let over = the_chord_the_top_contact_stands_over(&bore, centre);

        drift = drift.max(how_far_round(
            corners.len() as i32,
            feature as i32,
            over as i32,
        ));

        if !reading.rests_on_a_crease {
            // (the straight stretches) the contact is on the rail of the facet it touches, exactly.
            worst_off_its_own_facets_rail =
                worst_off_its_own_facets_rail.max(how_far_off_its_own_chords_tangent_line(
                    &bore,
                    feature,
                    centre - (the_raked_facets_normal(&bore, feature) * SIZE),
                ));

            continue;
        }

        on_a_crease += 1;
        carrying.insert(feature);

        // (a) on the ellipse: a radius from the crease LINE and a radius below the top face.
        let on_the_crease =
            corners[feature] + (bore.axis * (centre - corners[feature]).dot(bore.axis));

        worst_off_the_ellipse =
            worst_off_the_ellipse.max(((centre - on_the_crease).length() - SIZE).abs());
        worst_off_the_height = worst_off_the_height.max((centre.z - (top - SIZE)).abs());

        // (c) both neighbouring facets' perpendicular feet stand outside those facets.
        let (before, before_down, before_length) = the_foot_on_the_facets_plane(
            &bore,
            (feature + corners.len() - 1) % corners.len(),
            centre,
        );
        let (after, after_down, after_length) =
            the_foot_on_the_facets_plane(&bore, feature, centre);

        if (before < 0.0 || before > before_length || before_down > 0.0)
            && (after < 0.0 || after > after_length || after_down > 0.0)
        {
            both_feet_outside += 1;
        } else {
            either_foot_inside += 1;
        }
    }

    assert_eq!(
        on_a_crease, crease_stations,
        "how much of the loop the ball spends resting on a crease - instead {on_a_crease}"
    );

    assert!(
        worst_off_the_ellipse < 1e-9,
        "(a) every one of those stations is a radius from the crease LINE, which is what makes the \
         arc an ellipse in this plane - instead one is out by {worst_off_the_ellipse}"
    );

    assert!(
        worst_off_the_height < 1e-9,
        "(a) and a radius below the top face - instead out by {worst_off_the_height}"
    );

    assert_eq!(
        either_foot_inside, 0,
        "(c) the criterion holds at every crease station - instead it broke at {either_foot_inside}"
    );

    assert_eq!(
        both_feet_outside, crease_stations,
        "(c) and it holds at ALL of them, on the straight bore as much as on the raked one, which \
         is why it cannot be what tells a pivot vertex from a mitred one"
    );

    assert_eq!(
        carrying.len(),
        corners_carrying_an_arc,
        "(d) how many of the mouth's vertices carry an arc - instead {}",
        carrying.len()
    );

    assert!(
        worst_off_its_own_facets_rail < 1e-9,
        "over every straight stretch the contact lies ON the rail of the facet it touches, which is \
         the rail SurfaceStrip already draws - instead out by {worst_off_its_own_facets_rail}"
    );

    assert_eq!(
        drift, worst_drift,
        "how far round the loop the feature touched stands from the chord above it, which IS what \
         separates the two fixtures - instead {drift}"
    );
}

#[test]
fn the_arc_at_a_mouth_vertex_is_the_ball_pivoting_about_that_crease_true_1512_40_2() {
    the_arc_at_a_mouth_vertex_is_the_ball_pivoting_about_that_crease(true, 1512, 40, 2);
}

#[test]
fn the_arc_at_a_mouth_vertex_is_the_ball_pivoting_about_that_crease_false_840_40_1() {
    the_arc_at_a_mouth_vertex_is_the_ball_pivoting_about_that_crease(false, 840, 40, 1);
}

/// (b) The straight stretch and the arc meet TANGENTIALLY: the transition stands a radius from the
/// crease line in the facet's own outward normal, and there the ellipse's tangent comes out
/// parallel to that facet's mouth chord, so a strip can be ended at the transition and the arc
/// carried on from it with no kink.
fn the_arc_meets_each_stright_stretch_tangentially(raked: bool) {
    let bore = the_mouth_of(raked);
    let corners = &bore.corners;
    let top = bore.top;
    let axis = bore.axis;

    let mut worst_tangency: f64 = 0.0;
    let mut worst_off_the_wall: f64 = 0.0;

    for corner in 0..corners.len() {
        for facet in [(corner + corners.len() - 1) % corners.len(), corner] {
            let normal = the_raked_facets_normal(&bore, facet);
            let down = ((top - SIZE) - corners[corner].z - (normal.z * SIZE)) / axis.z;
            let on_the_crease = corners[corner] + (axis * down);
            let transition = on_the_crease + (normal * SIZE);

            let tangent = Vector3::new(0.0, 0.0, 1.0)
                .cross(transition - on_the_crease)
                .get_normal();
            let chord = (corners[(facet + 1) % corners.len()] - corners[facet]).get_normal();

            worst_tangency = worst_tangency.max((tangent.dot(chord).abs() - 1.0).abs());
            worst_off_the_wall =
                worst_off_the_wall.max((to_the_raked_wall(&bore, transition).reach - SIZE).abs());
        }
    }

    assert!(
        worst_tangency < 1e-12,
        "the arc leaves the crease in the direction the strip's own rails run - instead the cosine \
         is off one by {worst_tangency}"
    );

    assert!(
        worst_off_the_wall < 1e-9,
        "and the transition itself stands a radius from the wall, so the two constructions hand \
         over on the curve and not near it - instead out by {worst_off_the_wall}"
    );
}

#[test]
fn the_arc_meets_each_stright_stretch_tangentially_true() {
    the_arc_meets_each_stright_stretch_tangentially(true);
}

#[test]
fn the_arc_meets_each_stright_stretch_tangentially_false() {
    the_arc_meets_each_stright_stretch_tangentially(false);
}

/// TODAY'S MITRE ALREADY STANDS IN FOR THE PIVOT, at every vertex of both bores, and every strip's
/// own stretch touches its own facet: over the whole of each strip's mitre-bounded extent the ball
/// tangent to the top face and to THAT CHORD'S OWN FACET'S PLANE has its contact inside that facet
/// and stands exactly a radius from the wall. The mitre's apex stands `r (sec 4.5 degrees - 1)`
/// = 0.001237 past the pivot's round on both bores, and each strip's tangency on the pivot's
/// cylinder lies 0.13 of a chord inside the apex on the raked bore and 0.07 on the straight one.
fn the_mitre_already_stands_in_for_the_pivot_and_every_strip_touches_its_own_facet(
    raked: bool,
    shift_in_chords: f64,
) {
    let bore = the_mouth_of(raked);
    let corners = &bore.corners;
    let count = corners.len();

    let mut needing_a_pivot = 0;
    let mut empty_extents = 0;
    let mut worst_apex: f64 = 0.0;
    let mut worst_shift: f64 = 0.0;

    for corner in 0..count {
        let before = (corner + count - 1) % count;

        let apex = where_the_two_centre_lines_cross(&bore, before, corner);
        let on_the_crease = corners[corner] + (bore.axis * (apex - corners[corner]).dot(bore.axis));

        // (i) the apex stands OUTSIDE the pivot's round, which is what a mitre tip is.
        let beyond = (apex - on_the_crease).length() - SIZE;

        worst_apex = worst_apex.max((beyond - 0.001237).abs());

        // (ii) each strip's tangency on the pivot's cylinder lies inside the apex, so the strip
        // covers its own share of the round.
        let coming_in = the_tangency_on(&bore, before, corner);
        let going_out = the_tangency_on(&bore, corner, corner);

        let into_the_apex = (coming_in - apex).dot(the_chords_direction(&bore, before));
        let out_of_the_apex = (going_out - apex).dot(the_chords_direction(&bore, corner));

        if beyond < 0.0 || into_the_apex > 1e-12 || out_of_the_apex < -1e-12 {
            needing_a_pivot += 1;
        }

        worst_shift = worst_shift.max(
            (into_the_apex.abs() / (corners[corner] - corners[before]).length()).max(
                out_of_the_apex.abs() / (corners[(corner + 1) % count] - corners[corner]).length(),
            ),
        );
    }

    let mut leaving_its_own_facet = 0;
    let mut worst_off_the_wall: f64 = 0.0;

    for chord in 0..count {
        let from = the_tangency_on(&bore, chord, chord);
        let to = the_tangency_on(&bore, chord, (chord + 1) % count);
        let length = (corners[(chord + 1) % count] - corners[chord]).length();

        if (to - from).dot(the_chords_direction(&bore, chord)) <= 0.0 {
            empty_extents += 1;
        }

        for station in 0..=100 {
            let at = from + ((to - from) * (station as f64 / 100.0));

            let (along, down, _) = the_foot_on_the_facets_plane(&bore, chord, at);
            let reading = to_the_raked_wall(&bore, at);

            if along < -1e-9
                || along > length + 1e-9
                || down > 1e-9
                || (!reading.rests_on_a_crease && reading.facet != chord)
            {
                leaving_its_own_facet += 1;
            }

            worst_off_the_wall = worst_off_the_wall.max((reading.reach - SIZE).abs());
        }
    }

    assert!(
        worst_apex < 1e-6,
        "the mitre's apex stands 0.001237 past the pivot's round at every vertex of both bores, \
         that being r (sec 4.5 degrees - 1) - instead one is out by {worst_apex}"
    );

    assert_eq!(
        needing_a_pivot, 0,
        "so no vertex needs a pivot patch to stand in for it - instead {needing_a_pivot} of {count} \
         do, which would be the construction this reading was looking for"
    );

    assert!(
        (worst_shift - shift_in_chords).abs() <= 0.005,
        "and each strip's tangency lies that fraction of a chord inside the apex - instead \
         {worst_shift}"
    );

    assert_eq!(
        empty_extents, 0,
        "no strip is wholly taken by its neighbours - instead {empty_extents} are"
    );

    assert_eq!(
        leaving_its_own_facet,
        0,
        "over its own extent every strip's ball touches its own facet, which is the unrolled \
         construction - instead {leaving_its_own_facet} of {} stations do not",
        count * 101
    );

    assert!(
        worst_off_the_wall < 1e-9,
        "and stands exactly a radius from the wall while it does - instead out by \
         {worst_off_the_wall}"
    );
}

#[test]
fn the_mitre_already_stands_in_for_the_pivot_and_every_strip_touches_its_own_facet_true_0_13() {
    the_mitre_already_stands_in_for_the_pivot_and_every_strip_touches_its_own_facet(true, 0.13);
}

#[test]
fn the_mitre_already_stands_in_for_the_pivot_and_every_strip_touches_its_own_facet_false_0_07() {
    the_mitre_already_stands_in_for_the_pivot_and_every_strip_touches_its_own_facet(false, 0.07);
}

/// WHERE THE ROLLING BALL ACTUALLY RUNS round the raked mouth, measured against the WALL itself:
/// its centre runs on straight stretches - each parallel to its own chord and a radius off that
/// chord's own facet plane - joined by ELLIPTICAL ARCS, because over 156 of 720 stations the
/// nearest thing on the wall is not a facet but a CREASE. No chord's stretch crosses more than two
/// facets, and the true wall contact leaves the tangent line of the chord it stands UNDER by up to
/// 0.13565.
#[test]
fn the_rolling_ball_round_the_raked_mouth_rests_on_a_crease_over_a_fifth_of_the_loop() {
    let bore = the_mouth_of(true);
    let corners = &bore.corners;

    assert_eq!(
        corners.len(),
        RAKED_SIDES as usize,
        "the raked mouth is {RAKED_SIDES} chords - instead {}",
        corners.len()
    );

    let stations = 720;
    let mut on_a_crease = 0;
    let mut worst_from_the_wall: f64 = 0.0;
    let mut worst_departure: f64 = 0.0;
    let mut most_facets_over_one_chord = 0;
    let mut most_features_over_one_chord = 0;

    // Keyed by whether the feature is a crease as well as by its index, because a crease and a
    // facet both count from the same corner and are not the same thing on the wall.
    let mut facets_over: HashMap<usize, HashSet<(bool, usize)>> = HashMap::new();

    for s in 0..stations {
        let centre = the_rolling_balls_centre_at(
            &bore,
            2.0 * std::f64::consts::PI * s as f64 / stations as f64,
        );
        let reading = to_the_raked_wall(&bore, centre);

        worst_from_the_wall = worst_from_the_wall.max((reading.reach - SIZE).abs());

        if reading.rests_on_a_crease {
            on_a_crease += 1;
        }

        let over = the_chord_the_top_contact_stands_over(&bore, centre);

        let seen = facets_over.entry(over).or_default();
        seen.insert((reading.rests_on_a_crease, reading.facet));
        most_facets_over_one_chord =
            most_facets_over_one_chord.max(seen.iter().filter(|feature| !feature.0).count());
        most_features_over_one_chord = most_features_over_one_chord.max(seen.len());

        worst_departure = worst_departure.max(how_far_off_its_own_chords_tangent_line(
            &bore,
            over,
            reading.foot,
        ));
    }

    assert!(
        worst_from_the_wall < 1e-6,
        "every station of the curve stands a radius from the wall, which is what makes it the \
         rolling ball's own centre - instead one is out by {worst_from_the_wall}"
    );

    assert_eq!(
        on_a_crease, 156,
        "156 of {stations} stations rest on a crease rather than on a facet, measured 2026-09-16 - \
         instead {on_a_crease}, which would move the whole reading"
    );

    assert_eq!(
        most_facets_over_one_chord, 2,
        "and no chord's own stretch crosses more than two facets - instead \
         {most_facets_over_one_chord}"
    );

    assert_eq!(
        most_features_over_one_chord, 4,
        "nor more than four wall features counting the creases between them - instead \
         {most_features_over_one_chord}"
    );

    // 0.13565 at this suite's 720 stations and 0.138533 at ten times as many - the curve's own
    // worst point sits between stations, so the number is a floor on the departure and not its
    // maximum.
    assert!(
        (worst_departure - 0.13565).abs() <= 1e-5,
        "the furthest the true wall contact stands off the tangent line of the chord it stands \
         UNDER - instead {worst_departure}"
    );
}

/// C# `Bore`: one bore's mouth - its corners wound the way the loop goes, the drill's own
/// direction, and the face the mouth is cut in.
struct Bore {
    corners: Vec<Vector3>,
    axis: Vector3,
    top: f64,
}

/// C# `ToTheRakedWall`'s answer and its three `out`s.
struct WallReading {
    reach: f64,
    facet: usize,
    foot: Vector3,
    rests_on_a_crease: bool,
}

/// The line the rolling ball's centre runs along over `chord`: parallel to that chord, a radius
/// below the top face and a radius off that chord's OWN facet's plane.
fn the_centre_line_of(bore: &Bore, chord: usize) -> Vector3 {
    let normal = the_raked_facets_normal(bore, chord);
    let middle = (bore.corners[chord] + bore.corners[(chord + 1) % bore.corners.len()]) / 2.0;
    let drop = SIZE / (1.0 - normal.z);

    middle - Vector3::new(0.0, 0.0, drop) + (normal * (SIZE + (drop * normal.z)))
}

fn the_chords_direction(bore: &Bore, chord: usize) -> Vector3 {
    (bore.corners[(chord + 1) % bore.corners.len()] - bore.corners[chord]).get_normal()
}

/// Where two neighbouring centre lines cross, which is the apex today's mitre is cut at.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
fn where_the_two_centre_lines_cross(bore: &Bore, before: usize, after: usize) -> Vector3 {
    let from = the_centre_line_of(bore, before);
    let along = the_chords_direction(bore, before);
    let to = the_centre_line_of(bore, after);
    let across = the_chords_direction(bore, after);

    let turn = along.cross(across);

    if !(turn.length_squared() > 1e-18) {
        return from;
    }

    from + (along * ((to - from).cross(across).dot(turn) / turn.length_squared()))
}

/// Where `chord`'s centre line touches the cylinder of a radius about `crease` - the point on it
/// nearest that crease line, which is where the strip hands over to the pivot's arc.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > y)`, which refuses a NaN as well.
fn the_tangency_on(bore: &Bore, chord: usize, crease: usize) -> Vector3 {
    let from = the_centre_line_of(bore, chord);
    let along = the_chords_direction(bore, chord);
    let apart = from - bore.corners[crease];

    let leaning = along.dot(bore.axis);
    let denominator = 1.0 - (leaning * leaning);

    if !(denominator.abs() > 1e-14) {
        return from;
    }

    from + (along * (((leaning * apart.dot(bore.axis)) - apart.dot(along)) / denominator))
}

/// How many corners apart two features of the mouth stand, the short way round.
fn how_far_round(corners: i32, one: i32, other: i32) -> i32 {
    let apart = ((one - other) % corners + corners) % corners;

    apart.min(corners - apart)
}

/// The raked fixture's upper mouth, or the straight one's, at the sizes both are read at.
fn the_mouth_of(raked: bool) -> Bore {
    let axis = if raked {
        raked_drill_axis()
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };
    let top = if raked {
        RAKED_THICKNESS / 2.0
    } else {
        STRAIGHT_THICKNESS / 2.0
    };
    let radius = if raked { RAKED_RADIUS } else { STRAIGHT_RADIUS };

    let part = if raked {
        create_the_raked_fixture()
    } else {
        create_plate_with_a_round_bore(RAKED_SIDES, STRAIGHT_RADIUS, STRAIGHT_THICKNESS, 6.0)
    };

    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );

    let on_the_mouth = |at: Vector3| {
        (at.z - top).abs() < 1e-6 && ((at - (axis * at.dot(axis))).length() - radius).abs() < 1e-4
    };

    let mouth: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| on_the_mouth(edge.start) && on_the_mouth(edge.end))
        .copied()
        .collect();

    assert_eq!(
        mouth.len(),
        RAKED_SIDES as usize,
        "the mouth is {} chords, not {RAKED_SIDES}",
        mouth.len()
    );

    Bore {
        corners: the_mouth_wound_round(&mouth, axis),
        axis,
        top,
    }
}

/// The mouth's forty vertices in the order the loop goes round, taken off the fixture's own
/// feature edges so that nothing here depends on where the fixture started drilling.
fn the_mouth_wound_round(mouth: &[FeatureEdge], axis: Vector3) -> Vec<Vector3> {
    let mut wound = vec![mouth[0].start, mouth[0].end];
    let mut taken: HashSet<usize> = HashSet::from([0]);

    while taken.len() < mouth.len() {
        let at = wound[wound.len() - 1];

        let next = (0..mouth.len())
            .find(|&i| {
                !taken.contains(&i)
                    && ((mouth[i].start - at).length() < 1e-6
                        || (mouth[i].end - at).length() < 1e-6)
            })
            .expect("the raked mouth does not close");

        taken.insert(next);

        if taken.len() < mouth.len() {
            wound.push(if (mouth[next].start - at).length() < 1e-6 {
                mouth[next].end
            } else {
                mouth[next].start
            });
        }
    }

    // Wound the way the fixture builds it, so facet i is the one chord i rims.
    let turn = (wound[1] - wound[0]).cross(wound[2] - wound[1]);

    if turn.dot(axis) < 0.0 {
        wound.reverse();
    }

    wound
}

/// Where the rolling ball's centre stands on the ray at `turn` about the drill's own axis, in the
/// plane a radius below the top face, found by halving because the wall is a polyhedron and its
/// offset has no closed form.
fn the_rolling_balls_centre_at(bore: &Bore, turn: f64) -> Vector3 {
    let in_the_plane = bore.axis * ((bore.top - SIZE) / bore.axis.z);
    let ray = Vector3::new(turn.cos(), turn.sin(), 0.0);

    let mut near = 0.0;
    let mut far = RAKED_RADIUS * 8.0;

    for _ in 0..80 {
        let middle = (near + far) / 2.0;
        let at = in_the_plane + (ray * middle);

        if in_the_bore(bore, at) || to_the_raked_wall(bore, at).reach < SIZE {
            near = middle;
        } else {
            far = middle;
        }
    }

    in_the_plane + (ray * far)
}

/// Whether a point stands in the drilled void, which every facet's plane holds it inside of.
fn in_the_bore(bore: &Bore, at: Vector3) -> bool {
    (0..bore.corners.len())
        .all(|j| (at - bore.corners[j]).dot(the_raked_facets_normal(bore, j)) <= 0.0)
}

/// Facet j's outward unit normal - away from the drill's axis, which is where its material is.
fn the_raked_facets_normal(bore: &Bore, j: usize) -> Vector3 {
    let next = (j + 1) % bore.corners.len();
    let chord = (bore.corners[next] - bore.corners[j]).get_normal();
    let normal = chord.cross(bore.axis).get_normal();
    let middle = (bore.corners[j] + bore.corners[next]) / 2.0;

    if normal.dot(middle - (bore.axis * middle.dot(bore.axis))) < 0.0 {
        -normal
    } else {
        normal
    }
}

/// Where the perpendicular foot of a point onto facet j's PLANE stands, in that facet's own two
/// directions: how far along its mouth chord, and how far down the drill's axis - with the chord's
/// length (C#'s three `out`s).
fn the_foot_on_the_facets_plane(bore: &Bore, facet: usize, at: Vector3) -> (f64, f64, f64) {
    let next = (facet + 1) % bore.corners.len();
    let chord = bore.corners[next] - bore.corners[facet];
    let length = chord.length();

    let direction = chord / length;
    let normal = the_raked_facets_normal(bore, facet);
    let relative = at - bore.corners[facet];

    // relative = along * direction + down * axis + out * normal, by the triple product.
    let volume = direction.cross(bore.axis).dot(normal);

    let along = relative.cross(bore.axis).dot(normal) / volume;
    let down = direction.cross(relative).dot(normal) / volume;

    (along, down, length)
}

/// How far a point in the plate stands from the bore's wall, and where: the nearest point on it is
/// a foot inside one of the forty half strips or a point on the CREASE two of them share. Where the
/// foot leaves its facet the nearest point is found on that facet's own BOUNDARY - the two crease
/// rays and the mouth chord - not by clamping the foot's two oblique coordinates.
fn to_the_raked_wall(bore: &Bore, at: Vector3) -> WallReading {
    let mut reading = WallReading {
        reach: f64::MAX,
        facet: usize::MAX,
        foot: at,
        rests_on_a_crease: false,
    };

    for j in 0..bore.corners.len() {
        let next = (j + 1) % bore.corners.len();
        let chord = bore.corners[next] - bore.corners[j];
        let length = chord.length();
        let direction = chord / length;

        let (along, down, _) = the_foot_on_the_facets_plane(bore, j, at);

        if along >= 0.0 && along <= length && down <= 0.0 {
            let normal = the_raked_facets_normal(bore, j);
            let reach = (at - bore.corners[j]).dot(normal).abs();

            if reach < reading.reach {
                reading = WallReading {
                    reach,
                    facet: j,
                    foot: at - (normal * (at - bore.corners[j]).dot(normal)),
                    rests_on_a_crease: false,
                };
            }

            continue;
        }

        for corner in [j, next] {
            let under = (at - bore.corners[corner]).dot(bore.axis).min(0.0);
            let on = bore.corners[corner] + (bore.axis * under);
            let reach = (at - on).length();

            if reach < reading.reach {
                reading = WallReading {
                    reach,
                    facet: corner,
                    foot: on,
                    rests_on_a_crease: true,
                };
            }
        }

        let at_the_mouth = bore.corners[j]
            + (direction * length.min((at - bore.corners[j]).dot(direction).max(0.0)));
        let mouth_reach = (at - at_the_mouth).length();

        if mouth_reach < reading.reach {
            reading = WallReading {
                reach: mouth_reach,
                facet: j,
                foot: at_the_mouth,
                rests_on_a_crease: false,
            };
        }
    }

    reading
}

/// Which chord of the mouth the ball's contact on the TOP face stands over.
fn the_chord_the_top_contact_stands_over(bore: &Bore, centre: Vector3) -> usize {
    let top = Vector3::new(centre.x, centre.y, bore.top);

    let mut over = usize::MAX;
    let mut nearest = f64::MAX;

    for j in 0..bore.corners.len() {
        let next = (j + 1) % bore.corners.len();
        let chord = bore.corners[next] - bore.corners[j];
        let at = chord
            .length()
            .min((top - bore.corners[j]).dot(chord.get_normal()).max(0.0));
        let gap = (top - (bore.corners[j] + (chord.get_normal() * at))).length();

        if gap < nearest {
            nearest = gap;
            over = j;
        }
    }

    over
}

/// How far a point stands off the chord-parallel tangent line the claim draws on `chord` - the
/// line in that chord's own facet's plane that the ball tangent to that plane touches it along.
fn how_far_off_its_own_chords_tangent_line(bore: &Bore, chord: usize, at: Vector3) -> f64 {
    let next = (chord + 1) % bore.corners.len();
    let direction = (bore.corners[next] - bore.corners[chord]).get_normal();
    let normal = the_raked_facets_normal(bore, chord);
    let middle = (bore.corners[chord] + bore.corners[next]) / 2.0;

    let drop = SIZE / (1.0 - normal.z);
    let step_out = SIZE + (drop * normal.z);
    let touched_at = middle - Vector3::new(0.0, 0.0, drop) + (normal * (step_out - SIZE));

    let relative = at - touched_at;

    (relative - (direction * relative.dot(direction))).length()
}
