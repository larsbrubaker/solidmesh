//! A partnered bead's station profile clipped out of its partners' tubes.

use geometry_processing::bevel::bevel_edge_tool_builder as builder;
use geometry_processing::bevel::bevel_partner_tube_clip::try_clip;
use geometry_processing::bevel::BevelPartnerTube;
use geometry_processing::topology::EdgeConvexity;
use vector_math::{Vector2, Vector3};

const RADIUS: f64 = 5.0;

const SEGMENTS: i32 = 8;

/// A concave crease running along +z between the faces x = 0 and y = 0 of an inside corner.
fn profile() -> (Vec<Vector2>, Vector3, Vector3, Vector3) {
    let (normal0, normal1) = builder::try_get_profile_normals(
        EdgeConvexity::Concave,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let (x_axis, y_axis, z_axis) =
        builder::try_build_frame(normal0, normal1, Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let (points, _) = builder::try_build_cross_section(
        normal0, normal1, x_axis, y_axis, z_axis, RADIUS, SEGMENTS, true, 0.0,
    )
    .expect("the fixture's own profile has to build before anything is clipped out of it");

    (points, x_axis, y_axis, z_axis)
}

fn ball_centre_depth() -> f64 {
    -RADIUS / (std::f64::consts::PI / 4.0).sin()
}

fn tube(
    frame_x: f64,
    frame_y: f64,
    x_axis: Vector3,
    y_axis: Vector3,
    radius: f64,
) -> BevelPartnerTube {
    BevelPartnerTube::new(
        (x_axis * frame_x) + (y_axis * frame_y),
        x_axis.cross(y_axis),
        radius,
    )
}

fn in_part(point: Vector2, x_axis: Vector3, y_axis: Vector3) -> Vector3 {
    (x_axis * point.x) + (y_axis * point.y)
}

/// With no partner the profile is handed back point for point.
#[test]
fn with_no_partner_the_profile_is_handed_back_point_for_point() {
    let (profile, x_axis, y_axis, z_axis) = profile();

    let clipped =
        try_clip(&profile, Vector3::default(), x_axis, y_axis, z_axis, &[]).expect("no refusal");

    assert_eq!(clipped.len(), profile.len());
    for i in 0..profile.len() {
        assert_eq!(
            (clipped[i] - profile[i]).length(),
            0.0,
            "point {i} may not move at all"
        );
    }
}

/// A partner that does not reach this station changes nothing.
#[test]
fn a_partner_that_does_not_reach_this_station_changes_nothing() {
    let (profile, x_axis, y_axis, z_axis) = profile();
    let partner = tube(40.0, 40.0, x_axis, y_axis, RADIUS);

    let clipped = try_clip(
        &profile,
        Vector3::default(),
        x_axis,
        y_axis,
        z_axis,
        &[partner],
    )
    .expect("no refusal");

    assert_eq!(clipped.len(), profile.len());
}

/// A partner tube reaching into the fill takes its bite.
#[test]
fn a_partner_tube_reaching_into_the_fill_takes_its_bite() {
    let (profile, x_axis, y_axis, z_axis) = profile();
    let partner = tube(6.0, ball_centre_depth(), x_axis, y_axis, RADIUS);

    let clipped = try_clip(
        &profile,
        Vector3::default(),
        x_axis,
        y_axis,
        z_axis,
        &[partner],
    )
    .unwrap_or_else(|refusal| panic!("a tube part way into the fill leaves one region: {refusal}"));

    assert_eq!(
        clipped.len(),
        profile.len(),
        "the bite is SPENT on the seam, not inserted"
    );

    for point in &clipped {
        assert!(
            partner.depth(in_part(*point, x_axis, y_axis)) >= -1e-6,
            "({}, {}) is inside the partner's tube",
            point.x,
            point.y
        );
    }

    let intruding = profile
        .iter()
        .filter(|&&point| partner.depth(in_part(point, x_axis, y_axis)) < -1e-6)
        .count();
    assert!(intruding > 0, "the unclipped profile has to enter the tube");

    let on_the_tube = clipped
        .iter()
        .filter(|&&point| partner.depth(in_part(point, x_axis, y_axis)).abs() < 1e-6)
        .count();
    assert_eq!(
        on_the_tube, intruding,
        "every point the tube swallowed is spent on the seam and no other point moves"
    );
}

/// A partner tube that swallows the station is refused in words.
#[test]
fn a_partner_tube_that_swallows_the_station_is_refused_in_words() {
    let (profile, x_axis, y_axis, z_axis) = profile();
    let partner = tube(0.0, ball_centre_depth(), x_axis, y_axis, 40.0);

    assert!(
        try_clip(
            &profile,
            Vector3::default(),
            x_axis,
            y_axis,
            z_axis,
            &[partner]
        )
        .is_err(),
        "a station with nothing left of it has to be refused in words"
    );
}

/// Two partners are both taken out.
#[test]
fn two_partners_are_both_taken_out() {
    let (profile, x_axis, y_axis, z_axis) = profile();
    let partners = [
        tube(6.0, ball_centre_depth(), x_axis, y_axis, RADIUS),
        tube(-6.0, ball_centre_depth(), x_axis, y_axis, RADIUS),
    ];

    let clipped = try_clip(
        &profile,
        Vector3::default(),
        x_axis,
        y_axis,
        z_axis,
        &partners,
    )
    .unwrap_or_else(|refusal| panic!("both bites are ordinary ones: {refusal}"));

    for partner in &partners {
        for point in &clipped {
            assert!(partner.depth(in_part(*point, x_axis, y_axis)) >= -1e-6);
        }
    }
}

fn inside(partner: &BevelPartnerTube, point: Vector2, x_axis: Vector3, y_axis: Vector3) -> bool {
    partner.depth(in_part(point, x_axis, y_axis)) < -1e-6
}

/// A deep bite keeps the count and lays the seam in one direction round the tube.
#[test]
fn a_deep_bite_keeps_the_count_and_lays_the_seam_in_one_direction_round_the_tube() {
    let (profile, x_axis, y_axis, z_axis) = profile();
    let partner = tube(2.5, ball_centre_depth(), x_axis, y_axis, RADIUS);

    let clipped = try_clip(
        &profile,
        Vector3::default(),
        x_axis,
        y_axis,
        z_axis,
        &[partner],
    )
    .unwrap_or_else(|refusal| panic!("a deep bite is still one bite: {refusal}"));

    assert_eq!(clipped.len(), profile.len());

    let mut moved = 0;

    for i in 0..profile.len() {
        if inside(&partner, profile[i], x_axis, y_axis) {
            moved += 1;
            assert!(
                partner.depth(in_part(clipped[i], x_axis, y_axis)).abs() < 1e-6,
                "point {i} was swallowed, so its slot lands on the tube"
            );
            continue;
        }

        assert_eq!(
            (clipped[i] - profile[i]).length(),
            0.0,
            "point {i} may not move"
        );
    }

    assert!(
        moved >= 3,
        "the bite has to be deep enough to have a seam of several points"
    );

    let count = profile.len();
    let begins = (0..count)
        .find(|&slot| {
            inside(&partner, profile[slot], x_axis, y_axis)
                && !inside(
                    &partner,
                    profile[(slot + count - 1) % count],
                    x_axis,
                    y_axis,
                )
        })
        .unwrap();

    let mut angles = Vec::new();

    for i in 0..count {
        let slot = (begins + i) % count;

        if !inside(&partner, profile[slot], x_axis, y_axis) {
            break;
        }

        angles.push(partner.angle_of(in_part(clipped[slot], x_axis, y_axis)));
    }

    let mut steps = Vec::new();

    for i in 1..angles.len() {
        let mut step = angles[i] - angles[i - 1];

        while step > std::f64::consts::PI {
            step -= std::f64::consts::PI * 2.0;
        }

        while step < -std::f64::consts::PI {
            step += std::f64::consts::PI * 2.0;
        }

        steps.push(step);
    }

    for step in &steps {
        assert!(
            step * steps[0] >= 0.0,
            "the seam may not double back on itself"
        );
    }
}

fn off_the_line(point: Vector2, from: Vector2, to: Vector2) -> f64 {
    let along = to - from;

    if along.length() <= 0.0 {
        return (point - from).length();
    }

    let normal = Vector2::new(-along.y, along.x).get_normal();

    (point - from).dot(normal).abs()
}

fn off_the_edge(point: Vector2, from: Vector2, to: Vector2) -> f64 {
    let along = to - from;

    if along.length() <= 0.0 {
        return (point - from).length();
    }

    let at = ((point - from).dot(along) / along.length_squared()).clamp(0.0, 1.0);

    (point - (from + (along * at))).length()
}

fn frame(angle: f64, x_axis: Vector3, y_axis: Vector3) -> Vector3 {
    (x_axis * angle.cos()) + (y_axis * angle.sin())
}

fn flat(point: Vector3, x_axis: Vector3, y_axis: Vector3) -> Vector2 {
    Vector2::new(point.dot(x_axis), point.dot(y_axis))
}

/// A deep bite spends its end slots on the crossings themselves.
#[test]
fn a_deep_bite_spends_its_end_slots_on_the_crossings_themselves() {
    let (profile, x_axis, y_axis, z_axis) = profile();
    let partner = tube(2.5, ball_centre_depth(), x_axis, y_axis, RADIUS);

    let clipped = try_clip(
        &profile,
        Vector3::default(),
        x_axis,
        y_axis,
        z_axis,
        &[partner],
    )
    .unwrap_or_else(|refusal| panic!("a deep bite is still one bite: {refusal}"));

    let count = profile.len();
    let is_inside = |slot: usize| inside(&partner, profile[slot], x_axis, y_axis);
    let begins = (0..count)
        .find(|&slot| is_inside(slot) && !is_inside((slot + count - 1) % count))
        .unwrap();

    let mut swallowed = 0;

    while is_inside((begins + swallowed) % count) {
        swallowed += 1;
    }

    let ends = (begins + swallowed - 1) % count;
    let before = (begins + count - 1) % count;
    let after = (ends + 1) % count;

    assert!(
        swallowed >= 3,
        "two end slots and at least one between them"
    );
    assert!(
        off_the_edge(clipped[begins], profile[before], profile[begins]) < 1e-6,
        "the first spent slot lands on the crossing the profile entered by"
    );
    assert!(
        off_the_edge(clipped[ends], profile[after], profile[ends]) < 1e-6,
        "the last spent slot lands on the crossing the profile left by"
    );
}

/// A chorded partner leaves the seam on its own facets and a smooth one does not.
#[test]
fn a_chorded_partner_leaves_the_seam_on_its_own_facets_and_a_smooth_one_does_not() {
    let (profile, x_axis, y_axis, z_axis) = profile();

    let axis = x_axis.cross(y_axis);
    let centre = (x_axis * 2.5) + (y_axis * ball_centre_depth());

    let first_tangent = frame(std::f64::consts::PI / 4.0, x_axis, y_axis);
    let second_tangent = frame(std::f64::consts::PI * 3.0 / 4.0, x_axis, y_axis);

    let chorded = BevelPartnerTube::around_the_chorded_arc(
        centre,
        axis,
        RADIUS,
        -first_tangent,
        -second_tangent,
        SEGMENTS,
        f64::NEG_INFINITY,
        f64::INFINITY,
    );
    let smooth = BevelPartnerTube::new(centre, axis, RADIUS);

    assert!(
        chorded.is_chorded(),
        "a partner with an arc is swept chorded"
    );

    let quarter = std::f64::consts::PI / 2.0;
    let facets: Vec<(Vector2, Vector2)> = (0..SEGMENTS)
        .map(|i| {
            let at = |k: i32| {
                flat(
                    centre
                        + (frame(
                            (std::f64::consts::PI / 4.0)
                                + (quarter * f64::from(k) / f64::from(SEGMENTS)),
                            x_axis,
                            y_axis,
                        ) * RADIUS),
                    x_axis,
                    y_axis,
                )
            };
            (at(i), at(i + 1))
        })
        .collect();

    let mut off_the_facets = Vec::new();
    let mut off_the_facets_smooth = Vec::new();

    for (partner, off) in [
        (&chorded, &mut off_the_facets),
        (&smooth, &mut off_the_facets_smooth),
    ] {
        let clipped = try_clip(
            &profile,
            Vector3::default(),
            x_axis,
            y_axis,
            z_axis,
            &[*partner],
        )
        .unwrap_or_else(|refusal| panic!("this is one ordinary bite: {refusal}"));

        for i in 0..profile.len() {
            if partner.depth(in_part(profile[i], x_axis, y_axis)) < -1e-6 {
                off.push(
                    facets
                        .iter()
                        .map(|&(from, to)| off_the_line(clipped[i], from, to))
                        .fold(f64::INFINITY, f64::min),
                );
            }
        }
    }

    let worst = |values: &[f64]| values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    assert!(
        off_the_facets.len() >= 3,
        "a bite deep enough to lay several points"
    );
    assert!(
        worst(&off_the_facets) < 1e-6,
        "clipped against the chorded partner, every seam point lies on its facets: {}",
        worst(&off_the_facets)
    );
    assert!(
        worst(&off_the_facets_smooth) > 0.005,
        "clipped against the smooth cylinder, the seam stands off the facets by the sagitta: {}",
        worst(&off_the_facets_smooth)
    );
}
