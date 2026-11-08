//! THE OWNER FILE "Top Face Should stil bevel.mcx", on the
//! part's own mesh - the raked bore cube of [`crate::inner_face_test_mesh`] with its TOP (+Z) face
//! picked. One closed chain of 45 edges whose two JUNCTIONS (vertices 19 and 59, where the straight
//! -X edge meets the mouth arc, turning 86.81 degrees) share one flat side at UNEQUAL WEDGES (90 and
//! 93.18 degrees): the mitre is still a plane and both canals cut it in one ellipse, but the two
//! wall ends stand 0.0278 apart on it, no point of the fold closes them, and the leaving canal cuts
//! no notch into the wall. The class's helpers are in
//! [`crate::section_pierced_face_readings`]; its other parts (`.Junction`, `.Pass`, `.Seam`,
//! `.Takeover`, `.TopJoin`) are their own files.
//!
//! `TheTopFaceTakesItsFilletThroughTheSections` is not run: the junction's wedge is still laid
//! twice.

use solidmesh::bevel::sections::section_corner;
use solidmesh_math::math_helper::degrees_to_radians;
use solidmesh_math::Vector3;

use crate::inner_face_test_mesh;
use crate::section_pierced_face_readings::{
    distance_to_line, meet_of_lines, slid_onto_the_plane, the_axis_at, the_fold_at, the_plans,
    the_turn_at, wedge, RADIUS,
};
use crate::section_turn_readings::the_sharp_turns_of;

/// THE READING at the two junctions: one flat side shared, two wedges that differ, and the mitre
/// declining on the wedge rather than on the shared side.
#[test]
fn the_junction_is_one_flat_side_at_two_wedges() {
    let part = inner_face_test_mesh::the_part();
    let (plans, route, reason) = the_plans(&part, None);

    assert_eq!(plans.len(), 1, "the top face's rim is one chain");
    assert!(plans[0].is_closed);
    assert_eq!(
        plans[0].edges.len(),
        45,
        "three straight edges of the square top and the mouth's chords"
    );

    // DUTY FOUR IS WIRED: the turn clause no longer sends this pick to legacy on the wedge. The pass
    // is ROUTED to the sections and refused there instead, by the smooth-side cut.
    assert_eq!(
        route.as_deref(),
        Some("sections"),
        "and the sections then refuse it: {:?}",
        reason.as_ref().map(|reason| reason.code())
    );

    let turns = the_sharp_turns_of(&plans);

    let mut vertices: Vec<i32> = turns
        .iter()
        .map(|(_, leaving)| leaving.from_vertex())
        .collect();
    vertices.sort();
    assert_eq!(
        vertices,
        vec![1, 19, 59, 70, 121, 163],
        "the cube's four square corners and the mouth's two junctions"
    );

    for vertex in [19, 59] {
        let (arriving, leaving) = the_turn_at(&plans, vertex);

        assert!(
            (section_corner::turn_degrees(&arriving, &leaving) - 86.8145).abs() <= 0.002,
            "the chain turns off the straight edge onto the mouth arc at vertex {vertex}"
        );

        // ONE FLAT SIDE IS SHARED - the top face - which is what makes this a different reading
        // from the bore wall's own pick, where neither side was flat.
        assert!(
            (arriving
                .right_normal()
                .get_normal()
                .dot(leaving.right_normal().get_normal())
                - 1.0)
                .abs()
                <= 1e-9,
            "both edges stand on the picked top face"
        );

        let mut wedges = [wedge(&arriving), wedge(&leaving)];
        wedges.sort_by(|a, b| a.total_cmp(b));

        assert!(
            (wedges[0] - 90.0).abs() <= 1e-3,
            "the square top/-X cube edge"
        );
        assert!(
            (wedges[1] - 93.1802).abs() <= 1e-3,
            "the raked bore's mouth chord"
        );

        assert!(
            (RADIUS * (degrees_to_radians(wedges[1]) / 2.0).tan() - 0.528553).abs() <= 1e-5,
            "so the two setbacks on the one shared face are 0.5 and 0.5286"
        );

        assert!(
            !section_corner::looks_like_a_corner(&arriving, &leaving),
            "and the wedge screen turns the pair away - see the file header for what the \
             measured construction then says"
        );
    }
}

/// THE MITRE IS STILL A PLANE at unequal wedges, and the two canals cut it in ONE curve: the plane
/// through the axes' meet with normal `d0 + d1`, whose flat side end is the offset-meet.
#[test]
fn the_two_canals_cut_one_curve_in_the_plane_through_their_axes_meet() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for vertex in [19, 59] {
        let (arriving, leaving) = the_turn_at(&plans, vertex);
        let station = leaving.from();

        let (through, normal) = section_corner::try_the_mitre_plane(&arriving, &leaving, RADIUS)
            .expect("the two axes lie in one plane under the shared face, so they meet");

        // The axes meet exactly, which is what makes the plane a plane at all.
        assert!(
            distance_to_line(
                through,
                the_axis_at(&arriving, station, RADIUS),
                arriving.direction().get_normal()
            ) < 1e-9,
            "the meet stands on the arriving edge's canal axis"
        );

        assert!(
            distance_to_line(
                through,
                the_axis_at(&leaving, station, RADIUS),
                leaving.direction().get_normal()
            ) < 1e-9,
            "and on the leaving edge's, so the two axes are not skew"
        );
        assert!(
            (through.z - (20.0 - RADIUS)).abs() <= 1e-6,
            "both axes ride the radius under the top face"
        );

        // The VERTEX is off the plane - that is the whole of what unequal wedges change.
        assert!(
            ((station - through).dot(normal).abs() - 0.0207804).abs() <= 1e-5,
            "where the wedges agree the plane passes through the vertex, and here it does not"
        );

        let arriving_profile = slid_onto_the_plane(&arriving, station, through, normal, RADIUS, 9);
        let leaving_profile = slid_onto_the_plane(&leaving, station, through, normal, RADIUS, 9);

        // ONE CURVE: every point of the arriving edge's slid profile stands the radius from the
        // LEAVING edge's axis, so it lies on that canal too.
        let off_the_other_canal = arriving_profile
            .iter()
            .map(|point| {
                (distance_to_line(*point, through, leaving.direction().get_normal()) - RADIUS).abs()
            })
            .fold(f64::MIN, f64::max);

        assert!(
            off_the_other_canal < 1e-9,
            "two equal cylinders whose axes meet cut each other in a plane ellipse"
        );

        let arriving_flat = arriving_profile[arriving_profile.len() - 1];
        let leaving_flat = leaving_profile[leaving_profile.len() - 1];

        // The FLAT side ends are one point, the offset-meet, and nothing computes it twice.
        assert!(
            (arriving_flat - leaving_flat).length() < 1e-9,
            "both profiles end on the top face's offset-meet"
        );

        assert!(
            (arriving_flat.z - 20.0).abs() <= 1e-9,
            "which stands in the shared face"
        );

        assert!(
            (distance_to_line(arriving_flat, station, arriving.direction().get_normal())
                - RADIUS * (degrees_to_radians(wedge(&arriving)) / 2.0).tan())
            .abs()
                <= 1e-6,
            "on the arriving edge's own offset line"
        );

        assert!(
            (distance_to_line(arriving_flat, station, leaving.direction().get_normal())
                - RADIUS * (degrees_to_radians(wedge(&leaving)) / 2.0).tan())
            .abs()
                <= 1e-6,
            "and on the leaving edge's, which is what an offset-meet is"
        );
    }
}

/// WHAT IS STILL OPEN: the two profiles' WALL ends are different points of that one curve, and
/// neither of them is on the fold - so one shared profile does not yet serve both strips.
#[test]
fn the_two_wall_ends_stand_apart_on_the_one_curve() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for vertex in [19, 59] {
        let (arriving, leaving) = the_turn_at(&plans, vertex);
        let station = leaving.from();

        let (through, normal) = section_corner::try_the_mitre_plane(&arriving, &leaving, RADIUS)
            .expect("the mitre plane");

        let arriving_profile = slid_onto_the_plane(&arriving, station, through, normal, RADIUS, 9);
        let leaving_profile = slid_onto_the_plane(&leaving, station, through, normal, RADIUS, 9);

        let apart = (arriving_profile[0] - leaving_profile[0]).length();

        assert!(
            (apart - 0.027766).abs() <= 1e-5,
            "each profile ends at the tangency with its OWN wall, and the walls differ"
        );

        assert!(
            apart > 1e-3 * RADIUS,
            "which is fifty-five times what the measured check allows one shared profile"
        );

        let fold = the_fold_at(&part, &arriving, &leaving);

        // Named as a pair because the two junctions are mirrors: the straight edge arrives at 19
        // and leaves at 59, so which profile is the 90 degree one swaps between them.
        let mut off_the_fold = [
            distance_to_line(arriving_profile[0], station, fold),
            distance_to_line(leaving_profile[0], station, fold),
        ];
        off_the_fold.sort_by(|a, b| a.total_cmp(b));

        assert!(
            (off_the_fold[0] - 0.056340).abs() <= 1e-5,
            "the square edge's wall end stands off the fold"
        );

        assert!(
            (off_the_fold[1] - 0.058614).abs() <= 1e-5,
            "and so does the mouth chord's - neither rides it"
        );
    }
}

/// WHY THE WALL ENDS CANNOT BE CLOSED BY ONE POINT ON THE FOLD: the fold is in the cube's -X wall
/// and in NEITHER the bore facet the mouth chord stands on NOR the mitre plane, so the mouth chord's
/// canal never touches it and the plane crosses it on the far side of the vertex.
///
/// The reading this refutes is the tidy one: that each canal's tangency line with its own wall
/// crosses the fold in one point, that the two points are the same point because the mitre ellipse
/// lies on both canals, and that the corner is therefore the ellipse down to that point. The square
/// edge's canal does cross the fold, exactly, at `F1`; the mouth chord's canal misses it by 0.0586
/// mm, because the bore is TESSELLATED and the facet the fold folds against is not the facet the
/// chord stands on - they lean 6.36 degrees apart.
#[test]
fn the_fold_lies_in_one_wall_only_and_not_in_the_mitre_plane() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for vertex in [19, 59] {
        let (arriving, leaving) = the_turn_at(&plans, vertex);
        let station = leaving.from();
        let fold = the_fold_at(&part, &arriving, &leaving);

        // The straight top/-X cube edge stands at 90 degrees and the raked mouth chord at 93.18;
        // which of the two ARRIVES swaps between the two junctions, so name them by their wedge.
        let (square, chord) = if wedge(&arriving) < 91.0 {
            (arriving, leaving)
        } else {
            (leaving, arriving)
        };

        let (through, normal) = section_corner::try_the_mitre_plane(&arriving, &leaving, RADIUS)
            .expect("the mitre plane");

        assert!(
            fold.dot(square.left_normal().get_normal()).abs() < 1e-9,
            "the fold runs down the cube's -X wall, which is the square edge's own wall"
        );

        assert!(
            (fold.dot(chord.left_normal().get_normal()).abs() - 0.110796).abs() <= 1e-5,
            "but it leans 6.36 degrees out of the bore FACET the mouth chord stands on - the facet \
             the fold folds against is the next one round the tessellated bore"
        );

        let square_axis = the_axis_at(&square, station, RADIUS);
        let chord_axis = the_axis_at(&chord, station, RADIUS);
        let square_along = square.direction().get_normal();
        let chord_along = chord.direction().get_normal();

        let (f1, square_skew) = meet_of_lines(
            square_axis + (square.left_normal().get_normal() * RADIUS),
            square_along,
            station,
            fold,
        );
        let (f2, chord_skew) = meet_of_lines(
            chord_axis + (chord.left_normal().get_normal() * RADIUS),
            chord_along,
            station,
            fold,
        );

        assert!(
            square_skew < 1e-9,
            "so the square edge's canal DOES end its contact with that wall on the fold"
        );

        assert!(
            (distance_to_line(f1, square_axis, square_along) - RADIUS).abs() <= 1e-9,
            "F1 standing the radius off its own axis"
        );

        assert!(
            (distance_to_line(f1, chord_axis, chord_along) - (RADIUS + 0.056340)).abs() <= 1e-5,
            "and 0.0563 OUTSIDE the mouth chord's canal, so it is not a point of the mitre curve"
        );

        assert!(
            (chord_skew - 0.058561).abs() <= 1e-5,
            "and the mouth chord's canal never meets the fold at all - its tangency line is skew \
             to it, which is the whole of why one point cannot close both wall ends"
        );

        assert!(
            ((f1 - f2).length() - 0.038201).abs() <= 1e-5,
            "the two readings stand this far apart, not together"
        );

        // And the fold is not in the mitre plane either - at equal wedges it is, which is what lets
        // today's corner slide its wall end down it.
        assert!(
            (fold.dot(normal).abs() - 0.040364).abs() <= 1e-5,
            "the fold leans out of the plane the two canals cut in one curve"
        );

        let crossing = station + (fold * ((through - station).dot(normal) / fold.dot(normal)));

        assert!(
            ((crossing - station).dot(fold) - -0.514837).abs() <= 1e-4,
            "so the plane crosses the fold's LINE half a millimetre the wrong side of the vertex, \
             out of the part"
        );

        assert!(
            ((distance_to_line(crossing, square_axis, square_along) - RADIUS) - 0.630604).abs()
                <= 1e-4,
            "and that crossing is nowhere near either canal"
        );
    }
}

/// AND THE LEAVING CANAL CUTS NOTHING OUT OF THE -X WALL: forward of the junction it stands clear
/// of the wall's plane, the loop it would cut there lies wholly BEHIND the junction, and it never
/// reaches the fold - so there is no notch for the leaving strip to end against.
///
/// The reading this refutes is the End-section one: that the leaving edge's canal pokes through the
/// cube's -X wall and that the strip should therefore be ENDED against `x = -19`. Measured, that
/// curve is a closed ellipse standing at `t` in [-0.0572, -0.0016] ALONG THE CHORD - every point of
/// it behind the junction, out past the wall, where the canal does not exist.
#[test]
fn the_leaving_canal_cuts_no_notch_into_the_wall() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    let (arriving, leaving) = the_turn_at(&plans, 19);
    let station = leaving.from();
    let chord = if wedge(&arriving) < 91.0 {
        leaving
    } else {
        arriving
    };
    let axis = the_axis_at(&chord, station, RADIUS);
    let along = chord.direction().get_normal();

    // The canal's own cross-section frame: one across the wall's normal, one straight down.
    let across = along.cross(Vector3::new(0.0, 0.0, 1.0)).get_normal();

    assert!(
        along.x > 0.0,
        "the chord leaves the junction INTO the part, away from the -X wall"
    );

    // Forward of the junction the canal's least x: the cross-section reaches furthest back at t = 0.
    assert!(
        ((axis.x - (RADIUS * across.x.abs()) + 19.0) - 0.001588).abs() <= 1e-5,
        "so the leaving canal never reaches the wall's plane - it stands a micron and a half \
         inside it at its nearest, which is the foot of P2"
    );

    // Where the canal WOULD cut x = -19, in its own parameter along the chord: wholly behind.
    let nearest = (-19.0 - (axis.x + (RADIUS * across.x.abs()))) / along.x;
    let furthest = (-19.0 - (axis.x - (RADIUS * across.x.abs()))) / along.x;

    assert!((nearest - -0.057239).abs() <= 1e-5);
    assert!(
        (furthest - -0.001591).abs() <= 1e-5,
        "both ends of that loop stand BEHIND the junction, out past the wall, where the canal does \
         not exist - so there is no notch and nothing to end a strip against"
    );

    // And the fold never enters the canal either, anywhere along its line.
    let fold = the_fold_at(&part, &arriving, &leaving);
    let mut clearance = f64::MAX;

    // Steps of 1e-4 accumulated by addition, so the samples land on the same values every run.
    let mut s = 0.0;
    while s <= 1.0 {
        clearance = clearance.min(distance_to_line(station + (fold * s), axis, along) - RADIUS);
        s += 1e-4;
    }

    assert!(
        (clearance - 0.055571).abs() <= 1e-4,
        "the fold runs down the wall OUTSIDE the leaving canal the whole way - closest at s = \
         0.47, still 0.0556 clear - so the strip's rim cannot slide down it either"
    );
}

/// THE CORNER SWALLOWS NOTHING on this pick: every chord leaving a junction is a whole millimetre
/// against a reach of 0.5286.
#[test]
fn no_station_stands_inside_the_corners_reach() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for vertex in [19, 59] {
        let (_, leaving) = the_turn_at(&plans, vertex);
        let reach = RADIUS * (degrees_to_radians(wedge(&leaving)) / 2.0).tan();

        assert!((leaving.direction().length() - 1.0).abs() <= 1e-6);
        assert!(
            leaving.direction().length() > reach,
            "so the first station past the corner stands clear of it"
        );
    }
}
