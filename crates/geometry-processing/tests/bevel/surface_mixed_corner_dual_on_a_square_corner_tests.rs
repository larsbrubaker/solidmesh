//! Port of `SurfaceMixedCornerDualOnASquareCornerTests.cs`: what the DUAL canal does on the L
//! prism's SQUARE mixed corner, measured and recorded - not judged. The cap-riding canal's ball
//! centres run on the circle of radius 2 about (11, 11, 9) in z = 9; the dual's on the circle of
//! radius 2 about (9, 11, 9) in y = 11. Both span the same two end rings; they differ by up to a
//! whole radius in between, sweep the same Pappus volume, and are exact mirrors about the part.

use geometry_processing::bevel::bevel_working_mesh::signed_volume;
use geometry_processing::bevel::surface::SurfaceMixedCornerSpine;
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::surface_mixed_corner_reading_tests::the_mixed_corner;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

const EXACT: f64 = 1e-9;

/// The cell the volumes below are counted on.
const CELL: f64 = 0.025;

fn top_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

/// The dual READS on a square corner, and these are its frame numbers.
#[test]
fn the_dual_reads_on_the_square_corner_too_and_turns_the_cuts_own_sweep() {
    let corner = the_mixed_corner();

    let dual = corner
        .try_read_the_dual_canal()
        .unwrap_or_else(|why| panic!("instead {why:?}"));

    assert!(
        (dual.along_the_cut_out_of_the_wall.abs() - 1.0).abs() <= EXACT,
        "the cut runs square out of the wall's plane here - instead {}",
        dual.along_the_cut_out_of_the_wall
    );
    assert!(
        dual.rail_centre
            .equals_within(Vector3::new(9.0, 10.0, 9.0), EXACT),
        "the cut's centre line crosses the wall at (9, 10, 9) - instead {:?}",
        dual.rail_centre
    );
    assert!(
        (radians_to_degrees(dual.turn_radians) - 90.0).abs() <= EXACT,
        "and it turns the CUT's own sweep - instead {}",
        radians_to_degrees(dual.turn_radians)
    );

    // Which here is also the bead's, the corner being square.
    assert!((radians_to_degrees(corner.turn_radians()) - 90.0).abs() <= EXACT);
}

/// The two canals are two different circles of ball centres, each passing through the other's
/// centre.
#[test]
fn the_two_canals_are_two_different_circles_of_ball_centres() {
    let corner = the_mixed_corner();
    let dual = corner.try_read_the_dual_canal().expect("the dual reads");

    let (cap_centre, cap_radius, cap_normal) = the_circle_of(&corner.spine, corner.turn_radians());
    let (wall_centre, wall_radius, wall_normal) = the_circle_of(&dual.spine, dual.turn_radians);

    assert!(
        cap_centre.equals_within(Vector3::new(11.0, 11.0, 9.0), EXACT),
        "instead {cap_centre:?}"
    );
    assert!((cap_radius - 2.0 * RADIUS).abs() <= EXACT);
    assert!(
        (cap_normal.dot(Vector3::new(0.0, 0.0, 1.0)).abs() - 1.0).abs() <= EXACT,
        "it lies in the plane z = 9, a radius under the cap - instead its normal is {cap_normal:?}"
    );

    assert!(
        wall_centre.equals_within(Vector3::new(9.0, 11.0, 9.0), EXACT),
        "instead {wall_centre:?}"
    );
    assert!((wall_radius - 2.0 * RADIUS).abs() <= EXACT);
    assert!(
        (wall_normal.dot(Vector3::new(0.0, 1.0, 0.0)).abs() - 1.0).abs() <= EXACT,
        "and it lies in the plane y = 11, a radius outside the wall - instead its normal is {wall_normal:?}"
    );

    // Each circle's centre is a point of the other circle.
    assert!(
        ((cap_centre - wall_centre).length() - 2.0 * RADIUS).abs() <= EXACT,
        "so each passes through the other's centre"
    );

    // The two defining conditions of each, at every station, straight off production.
    for station in 0..=32 {
        let at = corner
            .spine
            .centre_at(station as f64 * corner.turn_radians() / 32.0);

        assert!(
            (corner.axis().distance_to(at) - 2.0 * RADIUS).abs() <= EXACT,
            "the cap-riding centre stands 2r from the BEAD's axis - {at:?} does not"
        );
        assert!(
            ((at - top_corner()).dot(corner.out_of_the_cap_face) + RADIUS).abs() <= EXACT,
            "and a radius INSIDE the cap - {at:?} does not"
        );

        let also_at = dual
            .spine
            .centre_at(station as f64 * dual.turn_radians / 32.0);

        assert!(
            (corner.cut_center_line().distance_to(also_at) - 2.0 * RADIUS).abs() <= EXACT,
            "the dual's centre stands 2r from the CUT's centre line - {also_at:?} does not"
        );
        assert!(
            ((also_at - top_corner()).dot(corner.out_of_the_wall) - RADIUS).abs() <= EXACT,
            "and a radius OUTSIDE the wall - {also_at:?} does not"
        );
    }
}

/// Both canals span the same two end rings, so the dual is closed in principle here.
#[test]
fn the_dual_spans_the_same_two_end_rings_the_cap_riding_canal_does() {
    let corner = the_mixed_corner();
    let dual = corner.try_read_the_dual_canal().expect("the dual reads");

    // Turn zero: a ball of the bead, so the bead's square end section is G1 into it.
    assert!(
        corner.axis().distance_to(dual.spine.centre_at(0.0)) < EXACT,
        "the dual's first centre stands ON the bead's axis - instead {:?}",
        dual.spine.centre_at(0.0)
    );
    assert!(
        dual.spine
            .centre_at(0.0)
            .equals_within(Vector3::new(11.0, 11.0, 9.0), EXACT),
        "instead {:?}",
        dual.spine.centre_at(0.0)
    );
    assert!(
        dual.cut_contact_at(0.0)
            .equals_within(corner.tangent_point, EXACT),
        "and it touches the cut's tube at the point the two cylinders touch - instead {:?}",
        dual.cut_contact_at(0.0)
    );

    // The cut's ring: the dual's own ring plane IS the square handover plane the cut stops on.
    let ring_plane = dual.cut_ring_plane();
    assert!(
        ring_plane
            .point
            .equals_within(corner.cut_stops_at.point, EXACT),
        "instead {:?}",
        ring_plane.point
    );
    assert!(
        (ring_plane.inward.dot(corner.cut_stops_at.inward).abs() - 1.0).abs() <= EXACT,
        "instead {:?} against {:?}",
        ring_plane.inward,
        corner.cut_stops_at.inward
    );

    for station in 0..=32 {
        let at = dual.cut_contact_at(station as f64 * dual.turn_radians / 32.0);

        assert!(
            corner.cut_stops_at.signed_distance(at).abs() < EXACT,
            "every station of the dual's cut contact lies in that one plane - {at:?} does not"
        );
        assert!(
            (corner.cut_center_line().distance_to(at) - RADIUS).abs() <= EXACT,
            "and on the cut's own cylinder - {at:?} does not"
        );
    }

    // The far end: the ball kisses the cap from outside, on the cut strip's own last rail.
    let last = dual.spine.centre_at(dual.turn_radians);

    assert!(
        last.equals_within(Vector3::new(9.0, 11.0, 11.0), EXACT),
        "instead {last:?}"
    );
    let above = (last - top_corner()).dot(corner.out_of_the_cap_face);
    assert!(
        (above - RADIUS).abs() <= EXACT,
        "tangent to the cap from OUTSIDE it, removing nothing of it - instead {above}"
    );
    assert!(
        dual.cut_contact_at(dual.turn_radians)
            .equals_within(Vector3::new(9.0, 11.0, 10.0), EXACT),
        "on the cut's own T-spring line - instead {:?}",
        dual.cut_contact_at(dual.turn_radians)
    );

    // And the cap-riding canal's first ball is the mirror of the dual's: a ball of the CUT.
    assert!(
        corner
            .cut_center_line()
            .distance_to(corner.spine.centre_at(0.0))
            < EXACT,
        "instead {:?}",
        corner.spine.centre_at(0.0)
    );
}

/// How far apart the two surfaces stand, on the corner they both span.
#[test]
fn how_far_apart_the_two_surfaces_stand_on_the_corner_they_both_span() {
    let corner = the_mixed_corner();
    let dual = corner.try_read_the_dual_canal().expect("the dual reads");

    let cap_riding = the_arc_of(&corner.spine, corner.turn_radians());
    let wall_riding = the_arc_of(&dual.spine, dual.turn_radians);

    let mut furthest: f64 = 0.0;
    let mut nearest = f64::MAX;

    for mine in 0..=512 {
        for theirs in 0..=512 {
            let apart = (cap_riding.at(mine as f64 * cap_riding.sweep / 512.0)
                - wall_riding.at(theirs as f64 * wall_riding.sweep / 512.0))
            .length();

            furthest = furthest.max(apart);
            nearest = nearest.min(apart);
        }
    }

    assert!(
        (furthest - 2.0 * 3f64.sqrt() * RADIUS).abs() <= 1e-4,
        "the two centre curves stand 2 root 3 apart at their furthest - instead {furthest:.5}"
    );
    assert!(
        (nearest - 2.0 * RADIUS).abs() <= 1e-4,
        "and never come nearer than the ball's own DIAMETER - instead {nearest:.5}"
    );

    // Point for point off the other's surface, both ways round.
    let off_the_dual: Vec<f64> = the_surface_of(
        &corner.spine,
        corner.turn_radians(),
        corner.out_of_the_cap_face,
    )
    .into_iter()
    .map(|at| off_the_surface(at, &wall_riding).abs())
    .collect();

    let off_the_cap_riding: Vec<f64> =
        the_surface_of(&dual.spine, dual.turn_radians, corner.out_of_the_wall)
            .into_iter()
            .map(|at| off_the_surface(at, &cap_riding).abs())
            .collect();

    let max_of = |them: &[f64]| them.iter().cloned().fold(f64::MIN, f64::max);

    assert!(
        (max_of(&off_the_dual) - 1.01388).abs() <= 1e-4,
        "the cap-riding surface stands that far off the dual at worst - instead {:.5}",
        max_of(&off_the_dual)
    );
    assert!(
        (max_of(&off_the_cap_riding) - 1.01388).abs() <= 1e-4,
        "and the dual exactly as far off it, the two being mirrors - instead {:.5}",
        max_of(&off_the_cap_riding)
    );

    assert!(
        (the_median_of(&off_the_dual) - 0.08594).abs() <= 1e-4,
        "while half of it stands within that, the two tubes being TANGENT along a curve - instead {:.5}",
        the_median_of(&off_the_dual)
    );
    assert!(
        (the_median_of(&off_the_cap_riding) - 0.08594).abs() <= 1e-4,
        "instead {:.5}",
        the_median_of(&off_the_cap_riding)
    );
}

/// What each canal's ball sweep measures, and how it sits in the part, counted on a deterministic
/// grid of `CELL` mm cells over the corner's own box.
#[test]
fn the_volumes_each_canal_sweeps() {
    let corner = the_mixed_corner();
    let dual = corner.try_read_the_dual_canal().expect("the dual reads");

    // The predicate the volumes are counted against, checked against the fixture it stands for.
    let part = create_l_shaped_grid_prism();

    assert!(
        (signed_volume(&part).abs() - 3000.0).abs() <= 1e-6,
        "the L prism is 20 x 20 x 10 less the 10 x 10 x 10 bite, which is what in_the_part reads it as"
    );

    let cap_riding = the_arc_of(&corner.spine, corner.turn_radians());
    let wall_riding = the_arc_of(&dual.spine, dual.turn_radians);

    let mut in_the_cap_riding = 0i64;
    let mut in_the_wall_riding = 0i64;
    let mut cap_riding_in_the_part = 0i64;
    let mut wall_riding_in_the_part = 0i64;
    let mut in_both = 0i64;

    // C#'s `for (var x = 8 + Cell / 2; x < 12; x += Cell)`, accumulated the same way.
    let mut x = 8.0 + (CELL / 2.0);
    while x < 12.0 {
        let mut y = 8.0 + (CELL / 2.0);
        while y < 12.0 {
            let mut z = 8.0 + (CELL / 2.0);
            while z < 12.0 {
                let at = Vector3::new(x, y, z);
                let mine = off_the_surface(at, &cap_riding) <= 0.0;
                let theirs = off_the_surface(at, &wall_riding) <= 0.0;
                let inside = in_the_part(at);

                if mine {
                    in_the_cap_riding += 1;
                    if inside {
                        cap_riding_in_the_part += 1;
                    }
                }

                if theirs {
                    in_the_wall_riding += 1;
                    if inside {
                        wall_riding_in_the_part += 1;
                    }
                }

                if mine && theirs {
                    in_both += 1;
                }

                z += CELL;
            }
            y += CELL;
        }
        x += CELL;
    }

    let cell = CELL * CELL * CELL;
    let exactly =
        (std::f64::consts::PI * std::f64::consts::PI) + (4.0 * std::f64::consts::PI / 3.0);

    assert!(
        (exactly - 14.0584).abs() <= 1e-4,
        "Pappus: a disc of area pi swept round a quarter of a circle of radius 2, capped both ends - instead {exactly}"
    );

    let measure = |count: i64| count as f64 * cell;

    assert!(
        (measure(in_the_cap_riding) - 14.06238).abs() <= 1e-4,
        "which is what the grid counts of the cap-riding sweep - instead {:.5}",
        measure(in_the_cap_riding)
    );
    assert!(
        (measure(in_the_wall_riding) - 14.06238).abs() <= 1e-4,
        "and of the dual's, to the CELL - instead {:.5}",
        measure(in_the_wall_riding)
    );

    assert!(
        (measure(cap_riding_in_the_part) - 13.87422).abs() <= 1e-4,
        "the cap-riding ball rolls INSIDE the material - instead {:.5}",
        measure(cap_riding_in_the_part)
    );
    assert!(
        (measure(wall_riding_in_the_part) - 0.18816).abs() <= 1e-4,
        "and the dual's void ball outside it - instead {:.5}",
        measure(wall_riding_in_the_part)
    );

    assert!(
        measure(in_both).abs() <= 1e-9,
        "and the two sweeps share NO volume at all - instead {:.5}",
        measure(in_both)
    );
}

/// Whether `at` is in the L prism: the 20 cube 10 tall, less the quarter at x, y > 10.
fn in_the_part(at: Vector3) -> bool {
    if at.x < 0.0 || at.x > 20.0 || at.y < 0.0 || at.y > 20.0 || at.z < 0.0 || at.z > 10.0 {
        return false;
    }

    !(at.x > 10.0 && at.y > 10.0)
}

/// How far `at` stands off the surface of `canal`: the distance to its centre ARC, less the ball's
/// radius. Negative inside the sweep, zero on its surface.
fn off_the_surface(at: Vector3, canal: &CentreArc) -> f64 {
    canal.distance_to(at) - RADIUS
}

/// The arc a canal's ball centres run on, as a circle and the two ends of the sweep.
struct CentreArc {
    centre: Vector3,
    radius: f64,
    normal: Vector3,
    from: Vector3,
    to: Vector3,
    sweep: f64,
}

impl CentreArc {
    /// Where the arc stands at `turn` of its own sweep.
    fn at(&self, turn: f64) -> Vector3 {
        let out = (self.from - self.centre) / self.radius;
        let across = self.normal.cross(out);

        self.centre + (out * (self.radius * turn.cos())) + (across * (self.radius * turn.sin()))
    }

    /// How far `at` stands from the arc: from the circle where its own angle falls inside the
    /// sweep, and from the nearer end where it does not.
    fn distance_to(&self, at: Vector3) -> f64 {
        let from = at - self.centre;
        let off = from.dot(self.normal);
        let in_the_plane = from - (self.normal * off);
        let out = (self.from - self.centre) / self.radius;
        let across = self.normal.cross(out);
        let round = in_the_plane.dot(across).atan2(in_the_plane.dot(out));

        if round >= 0.0 && round <= self.sweep {
            let radial = in_the_plane.length() - self.radius;
            return ((radial * radial) + (off * off)).sqrt();
        }

        (at - self.from).length().min((at - self.to).length())
    }
}

/// The arc a canal's centres run on, read off three of production's own stations and then CHECKED
/// against thirty-three of them.
fn the_arc_of(spine: &SurfaceMixedCornerSpine, turn: f64) -> CentreArc {
    let (centre, radius, normal) = the_circle_of(spine, turn);
    let arc = CentreArc {
        centre,
        radius,
        normal,
        from: spine.centre_at(0.0),
        to: spine.centre_at(turn),
        sweep: turn,
    };

    for station in 0..=32 {
        let at = station as f64 * turn / 32.0;

        assert!(
            (arc.at(at) - spine.centre_at(at)).length() <= 1e-9,
            "the arc read off the spine is not the spine at {at}: {:?} against {:?}",
            arc.at(at),
            spine.centre_at(at)
        );
    }

    arc
}

/// A dense sample of the canal's own SURFACE over the patch it spans: at every station, the ring
/// of the ball from the face it rides round to the tube it is pressed against.
fn the_surface_of(
    spine: &SurfaceMixedCornerSpine,
    turn: f64,
    _out_of_the_ridden_face: Vector3,
) -> Vec<Vector3> {
    let mut points = Vec::new();

    for station in 0..=64 {
        let at = station as f64 * turn / 64.0;
        let centre = spine.centre_at(at);
        let from_the_face = (spine.t_contact_at(at) - centre).get_normal();
        let from_the_tube = (spine.bead_contact_at(at) - centre).get_normal();
        let across =
            (from_the_tube - (from_the_face * from_the_tube.dot(from_the_face))).get_normal();
        let sweep = from_the_face.dot(from_the_tube).clamp(-1.0, 1.0).acos();

        for rail in 0..=32 {
            let round = rail as f64 * sweep / 32.0;

            points.push(
                centre
                    + (from_the_face * (RADIUS * round.cos()))
                    + (across * (RADIUS * round.sin())),
            );
        }
    }

    points
}

/// The circle a canal's centres run on, read off three of its own stations: the plane they span
/// and the centre equidistant from all three.
fn the_circle_of(spine: &SurfaceMixedCornerSpine, turn: f64) -> (Vector3, f64, Vector3) {
    let a = spine.centre_at(0.0);
    let b = spine.centre_at(turn / 2.0);
    let c = spine.centre_at(turn);

    let normal = (b - a).cross(c - a).get_normal();

    // The centre is where the two chords' perpendicular bisectors cross, solved in the plane.
    let across = (b - a).cross(normal).get_normal();
    let along_to = (c - b).cross(normal).get_normal();
    let from = (a + b) / 2.0;
    let to = (b + c) / 2.0;

    let denominator = across.cross(along_to).length();
    let step = (to - from).cross(along_to).length() / denominator;
    let sign = if (to - from).cross(along_to).dot(across.cross(along_to)) > 0.0 {
        step
    } else {
        -step
    };
    let centre = from + (across * sign);

    (centre, (a - centre).length(), normal)
}

fn the_median_of(them: &[f64]) -> f64 {
    let mut sorted = them.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));

    sorted[sorted.len() / 2]
}
