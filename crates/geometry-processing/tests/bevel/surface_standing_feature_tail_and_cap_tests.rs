//! The readings the notched end is sewn from, asked ON THEIR OWN rather than through a part - where
//! the strip stops, the bands the tail draws past it, what the split cap leaves on the ring's own
//! chords, and how an extended face that no fan covers is cut up
//! (`surface_strip_tail_around_a_feature`, `surface_end_cap_around_a_feature`,
//! `surface_standing_feature_faces`, `surface_ring_ears`).

use std::collections::HashSet;

use geometry_processing::bevel::surface::{
    surface_end_cap_around_a_feature, surface_ring_ears, surface_standing_feature_faces,
    surface_strip_tail_around_a_feature, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use vector_math::{Vector3, Vector3Float};

/// The tail's own rule, read off the production reading rather than off a copy of it: every corner
/// of every band it sews is a point that was ALREADY standing - a row point of the cross section, a
/// rail's crossing, the exit, or a Q - and there is nothing else in any of them.
#[test]
fn the_tail_is_sewn_from_points_that_were_already_standing() {
    let station = [100, 101, 102, 103, 104];
    let notched = [false, true, true, true, false];
    let crossing = [-1, 201, 202, 203, -1];
    let q_on = [400, -1, -1, 401];

    let rings = surface_strip_tail_around_a_feature::try_the_rings(
        &station, &notched, &crossing, 1, 300, &q_on,
    )
    .unwrap_or_else(|refused_at| {
        panic!("every band of this notch is one the tail draws - refused at {refused_at}")
    });

    // One band per pair of rails with anything past the cross section: three notched rails give two
    // boundary bands and two between them.
    assert_eq!(rings.len(), 4);

    // Each walked in boundary order: out along the higher rail, back along the far rim, down the
    // lower rail - and the band the ridge leaves through is the one pentagon.
    let read = rings
        .iter()
        .map(|ring| {
            ring.iter()
                .map(|vertex| vertex.to_string())
                .collect::<Vec<_>>()
                .join(",")
        })
        .collect::<Vec<_>>()
        .join(" | ");
    assert_eq!(
        read,
        "100,101,201,400 | 101,102,202,300,201 | 102,103,203,202 | 103,104,401,203"
    );

    let used: HashSet<i32> = rings.iter().flatten().copied().collect();

    // Five row points, three crossings, the exit and two Qs - eleven points, and the tail mints
    // none of its own.
    assert_eq!(used.len(), 11);
    // And every one of them is a point the notch or the trim had already stood.
    assert!(used.iter().all(|vertex| station.contains(vertex)
        || crossing.contains(vertex)
        || q_on.contains(vertex)
        || *vertex == 300));
}

/// What the split cap leaves standing on the end RING's own chords is carried into the station, in
/// the station's own order, and anything standing where the ring has no rail pair is refused by
/// name. The walk runs DOWN the rails here, which is the half of it a forward walk would never
/// show: the cap reads each chord from its higher rail and a station is indexed from the lower one,
/// so the points have to come back reversed.
#[test]
fn the_caps_own_betweens_are_carried_into_the_station() {
    let pool = SurfaceVertexPool::new(&[Vector3Float::new(1.0, 2.0, 3.0)]);

    // Five rails walked from the top down, the notch taking the last two of the walk: the head is
    // rails 4, 3, 2, then the feature's Q and its crease vertex.
    let walk = [4, 3, 2, 1, 0];
    let head_step = [0, 1, 2];
    let profile = [104, 103, 102, 900, 901];

    let along_the_profile = [
        Some(vec![700, 701]),
        Some(vec![]),
        Some(vec![]),
        Some(vec![]),
    ];

    let between = surface_end_cap_around_a_feature::try_the_betweens(
        &pool,
        &walk,
        &head_step,
        &profile,
        &along_the_profile,
        5,
    )
    .unwrap_or_else(|refusal| {
        panic!("both ends of that chord are rails the walk took one after the other - {refusal}")
    });

    // One gap per pair of neighbouring rails.
    assert_eq!(between.len(), 4);
    // The chord from rail 4 to rail 3 is the station's gap 3, and its two points come back in the
    // LOWER rail's order - reversed, because the walk read them from the higher.
    let read = between
        .iter()
        .map(|one| {
            one.iter()
                .map(|vertex| vertex.to_string())
                .collect::<Vec<_>>()
                .join(",")
        })
        .collect::<Vec<_>>()
        .join(" | ");
    assert_eq!(read, " |  |  | 701,700");

    // And the same point standing on the chord into the feature's Q, which is no chord of the ring.
    let off_the_ring = [Some(vec![]), Some(vec![]), Some(vec![0]), Some(vec![])];

    let named = surface_end_cap_around_a_feature::try_the_betweens(
        &pool,
        &walk,
        &head_step,
        &profile,
        &off_the_ring,
        5,
    )
    .expect_err("the stitch has no rail pair to sew a point on that chord to");
    assert_eq!(named.code(), BevelAdvisoryCode::SurfaceStripSeamDidNotMeet);
}

/// Where the strip stops, read on its own: clear of the notch, clear of everything the other end
/// reaches, and refused where the two leave no room between them.
#[test]
fn the_section_stands_clear_of_every_end_the_strip_has() {
    // The notch is deepest at 8.76 and the other end reaches 2, so there is room between them.
    let clear = surface_strip_tail_around_a_feature::try_the_section(8.76, &[0.0, 2.0], 1e-6)
        .expect("there is room between them");
    // Half way between the furthest the other end reaches and the notch's own deepest point.
    assert!((clear - 5.38).abs() <= 1e-9, "{clear}");

    // A crossing at the other end reaching PAST the notch leaves no section between the two -
    // measured against every end the strip has, not against the staircase alone.
    assert!(
        surface_strip_tail_around_a_feature::try_the_section(8.76, &[0.0, 9.5], 1e-6).is_none()
    );

    // And an other end standing exactly where the notch is deepest leaves no strip to stitch.
    assert!(surface_strip_tail_around_a_feature::try_the_section(2.0, &[2.0], 1e-6).is_none());

    // A notch at the START end is read the same way, the other end being above it.
    let down = surface_strip_tail_around_a_feature::try_the_section(2.0, &[14.0, 10.0], 1e-6)
        .expect("a notch at the start end is read the same way");
    // Half way between the NEAREST the other end comes and the notch - 10, not 14, which is the
    // seeding that used to be taken from zero and stitched the strip through its own notch.
    assert!((down - 6.0).abs() <= 1e-9, "{down}");
}

/// An extended face no fan can cover once is cut into EARS instead - not refused, and not fanned:
/// the fan winds every triangle it lays to the normal it is given, so a ring that caves in would be
/// covered twice over with nothing said about it.
#[test]
fn an_extended_face_no_fan_covers_is_cut_into_ears() {
    // A square, and the same outline with a channel cut into its top - the smallest shape no corner
    // of which sees the whole of it, since either arm hides the other.
    let vertices = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(6.0, 0.0, 0.0),
        Vector3Float::new(6.0, 6.0, 0.0),
        Vector3Float::new(4.0, 6.0, 0.0),
        Vector3Float::new(4.0, 2.0, 0.0),
        Vector3Float::new(2.0, 2.0, 0.0),
        Vector3Float::new(2.0, 6.0, 0.0),
        Vector3Float::new(0.0, 6.0, 0.0),
    ];

    let pool = SurfaceVertexPool::new(&vertices);
    let up = Vector3::new(0.0, 0.0, 1.0);

    // A convex ring is covered exactly once from any corner of it.
    assert!(!surface_standing_feature_faces::no_apex_covers_it(
        &[0, 1, 2, 7],
        &pool,
        up
    ));
    let channel = [0, 1, 2, 3, 4, 5, 6, 7];

    // The channel between the two arms is seen from neither of them, so a fan of this ring covers
    // part of it twice - which is why the ears are asked for instead.
    assert!(surface_standing_feature_faces::no_apex_covers_it(
        &channel, &pool, up
    ));

    let mut faces = Vec::new();

    // And a simple planar ring always has an ear, however it caves in.
    assert!(surface_ring_ears::try_triangulate_the_ring(
        &channel, &pool, &mut faces, up, false
    ));

    // A triangulation of an n corner polygon is n - 2 triangles, so none of them is spare.
    assert_eq!(faces.len(), channel.len() - 2);

    let mut area = 0.0;

    for &(v0, v1, v2) in &faces {
        let first = pool.position_of(v0);
        let second = pool.position_of(v1);
        let third = pool.position_of(v2);
        let normal = (second - first).cross(third - first);

        // Every triangle lies the way the ring does - a backwards one is a fold.
        assert!(normal.dot(up) > 0.0);

        let middle = (first + second + third) / 3.0;

        // And stands inside the polygon, not across the channel.
        assert!(
            is_inside_the_channel(middle),
            "{middle} stands across the channel"
        );

        area += normal.length() / 2.0;

        for corner in [v0, v1, v2] {
            // Out of the ring's OWN corners: the ears mint nothing.
            assert!(channel.contains(&corner));
        }
    }

    // And they cover the 6 by 6 square less the 2 by 4 channel exactly once - 28 mm^2.
    assert!((area - 28.0).abs() <= 1e-9, "{area}");

    // With not one new vertex in the pool.
    assert_eq!(pool.vertices().len(), 8);
}

/// Whether a point of the plane stands inside the channel shaped ring above - read here, in the
/// test, so that what the triangulation is measured against is the SHAPE and not another reading
/// of it.
fn is_inside_the_channel(at: Vector3) -> bool {
    if at.x <= 0.0 || at.x >= 6.0 || at.y <= 0.0 || at.y >= 6.0 {
        return false;
    }

    // The channel itself: the 2 by 4 slot between the two arms, open at the top.
    !(at.x > 2.0 && at.x < 4.0 && at.y > 2.0)
}
