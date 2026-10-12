//! Port of `SurfaceMixedCornerOverACurvedSharedFaceTests.cs`: the mixed corner the other way round,
//! where the face the two edges SHARE is the chorded wall and the wall nobody picked is the flat face.
//! That is the LOBE's own wall picked as a face, "Every Chamfer Works.mcx"'s cylinder barrel, read
//! off the merged outline alone (`lobed_prism_meshes`), whose rim turns over at each seam.
//!
//! The convex side is a tangent CHAIN of 9 degree chords along the cap, and the bead's ball is not
//! tangent to the chord arriving at the corner: the bead rolls across the wall's facets until it is
//! tangent to facet `k`, so the cut is the chord standing on THAT facet and the chords between are
//! swallowed by the blend (`surface_mixed_corner_cut_edge`). The blend is the same torus about the
//! same axis, the turn still 67.5 degrees. The rim reading is the one
//! `surface_mixed_corner_resting_on_a_crease_tests.rs` shares. C# `[Arguments]` rows are a loop.

use std::rc::Rc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_hands_over_at_the_mitre,
    surface_mixed_corner_ring_schedule, surface_mixed_corner_sector, SurfaceBandEnd,
    SurfaceBevelPlan, SurfaceBuildOptions, SurfaceEdgeRoles, SurfaceLine, SurfaceMixedCorner,
    SurfaceMixedCornerFaces, SurfaceStrip, SurfaceVertexPool,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::math_helper::{degrees_to_radians, radians_to_degrees};
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mixed_corner_resting_on_a_crease_tests::{apart, the_walls_rim, Rim};

/// The fixture's own fillet radius, which is what "Every Chamfer Works.mcx" is filleted at.
const RADIUS: f64 = 3.756;

/// The turn facet k names here, which is the flat side's own rim's turn read the other way about.
const TURN_DEGREES: f64 = 67.5;

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The corner at the top of the right seam the rim reading names.
fn the_corner() -> Vector3 {
    Vector3::new(
        lobed_prism_meshes::right_crossing().x,
        lobed_prism_meshes::right_crossing().y,
        lobed_prism_meshes::HEIGHT,
    )
}

/// The bead's own index in the chain: the edge after the corner where the cut arrives, else the
/// corner's own edge.
fn bead_index(read: &Rim) -> i32 {
    if read.cut_is_arriving {
        (read.corner_index + 1) % read.planned.len() as i32
    } else {
        read.corner_index
    }
}

/// The cut is the chord the bead ROLLED to and not the chord arriving at the corner, and the chord
/// between the two is swallowed - it stands nearer the axis than the ball's own reach.
#[test]
fn the_cut_is_the_chord_the_bead_rolled_to_and_the_chord_between_is_swallowed() {
    let read = the_walls_rim(RADIUS);

    assert_eq!(
        read.swallowed.len(),
        1,
        "the ball rolls over the part chord the merge left at the seam - instead {}",
        read.swallowed.len()
    );

    let swallowed = &read.planned[read.swallowed[0] as usize];

    assert!(
        near(swallowed.from(), Vector3::new(29.511, -1.09, 20.0)),
        "which is the crossing chord's own rim edge - instead {:?} -> {:?}",
        swallowed.from(),
        swallowed.to()
    );
    assert!(
        near(swallowed.to(), the_corner()),
        "and it is the one arriving at the corner - instead {:?} -> {:?}",
        swallowed.from(),
        swallowed.to()
    );

    let cut = &read.planned[read.cut_index as usize];

    assert!(
        near(cut.from(), Vector3::new(28.91, -2.54, 20.0)),
        "the cut is the next chord along - instead {:?} -> {:?}",
        cut.from(),
        cut.to()
    );
    assert!(
        near(cut.to(), swallowed.from()),
        "which runs into the swallowed one - instead {:?} -> {:?}",
        cut.from(),
        cut.to()
    );

    // And why: the two cylinders are tangent exactly where the centre lines stand 2r apart, which
    // is the reading itself rather than a count of facets.
    assert_within(
        apart(cut.center_line, read.bead.center_line),
        2.0 * RADIUS,
        1e-9,
        "the cut's cylinder touches the bead's",
    );

    assert_within(
        apart(swallowed.center_line, read.bead.center_line),
        7.2563,
        1e-3,
        "while the swallowed chord's stands inside it, which is what swallowed means",
    );
}

/// And the corner then READS, with the same frame the flat side's own rim gives: the axis a radius
/// off facet k and a radius off the flat face, the touch point the bead's foot on facet k lifted a
/// radius under the cap, and 67.5 degrees of turn between them.
#[test]
fn the_corner_reads_with_the_beads_own_foot_on_the_facet_it_rolled_to() {
    let read = the_walls_rim(RADIUS);
    let corner = &read.corner1;

    // The axis: under the lobe, a radius outside the flat face it also stands against.
    assert_within(
        corner.axis().point.x,
        32.4718,
        1e-3,
        &format!("instead {:?}", corner.axis().point),
    );
    assert_within(
        corner.axis().point.y,
        -RADIUS,
        1e-6,
        &format!("a radius outside y = 0 - instead {:?}", corner.axis().point),
    );

    // Turn zero is the way out of the axis to the bead's foot on the SHARED face, which on a rolled
    // one is facet k's own inward normal - 22.5 degrees round from -x here.
    assert_within(
        radians_to_degrees(
            (-corner
                .towards_the_shared_face()
                .dot(Vector3::new(1.0, 0.0, 0.0)))
            .clamp(-1.0, 1.0)
            .acos(),
        ),
        22.5,
        0.02,
        &format!("instead {:?}", corner.towards_the_shared_face()),
    );

    // The touch point: a radius out of the axis that way, in the plane a radius under the cap.
    assert_within(
        corner.tangent_point.z,
        lobed_prism_meshes::HEIGHT - RADIUS,
        1e-6,
        &format!("instead {:?}", corner.tangent_point),
    );
    assert!(
        (corner.tangent_point
            - Vector3::new(29.0021, -2.3178, lobed_prism_meshes::HEIGHT - RADIUS))
        .length()
            < 1e-3,
        "which is the bead's foot on facet k lifted under the cap - instead {:?}",
        corner.tangent_point
    );

    assert_within(
        corner.axis().distance_to(corner.tangent_point),
        RADIUS,
        1e-4,
        "on the axis's own radius",
    );
    assert_within(
        corner.cut_center_line().distance_to(corner.tangent_point),
        RADIUS,
        1e-4,
        "and on the cut's",
    );

    // The turn ends on the flat face nobody picked, so the blend's last station stands on the
    // bead's foot there and the cap corner is that foot in the cap face.
    assert_within(
        radians_to_degrees(corner.turn_radians()),
        TURN_DEGREES,
        0.02,
        &format!("instead {}", radians_to_degrees(corner.turn_radians())),
    );

    assert!(
        (corner.out_of_the_wall - Vector3::new(0.0, -1.0, 0.0)).length() < 1e-6,
        "the wall nobody picked is the flat face - instead {:?}",
        corner.out_of_the_wall
    );

    assert_within(
        corner.outer_rail_centre.z,
        lobed_prism_meshes::HEIGHT,
        1e-9,
        "the outer rail runs in the cap face",
    );

    assert_within(
        corner.cap_corner().y,
        0.0,
        1e-6,
        &format!(
            "the blend dies on the flat face - instead {:?}",
            corner.cap_corner()
        ),
    );

    // The handover plane is the meridian through the axis and that foot, and its normal is the
    // CUT edge's own direction - the chord on facet k, not the chord arriving at the corner.
    let along_the_cut = read.planned[read.cut_index as usize].edge.direction();

    assert_within(
        corner.cut_stops_at.inward.dot(along_the_cut).abs(),
        1.0,
        1e-6,
        &format!(
            "instead {:?} against {along_the_cut:?}",
            corner.cut_stops_at.inward
        ),
    );
}

/// WHY there is no room, in one reading: the cut chord's own rail on the cap is TANGENT to the
/// circle the ball's touch point traces, and it touches it at the handover - so the blend's
/// footprint never crosses that rail, while the ring the blend hands over on stands 0.0547 outside
/// it, past the chord's mitre with the chord before it.
#[test]
fn the_cut_chords_cap_rail_only_touches_the_balls_circle_at_the_handover() {
    let read = the_walls_rim(RADIUS);
    let corner = &read.corner1;
    let cut = &read.planned[read.cut_index as usize];

    let on_the_cap = on_the_cap_of(cut, corner);

    assert_within(
        apart(on_the_cap, corner.axis()),
        corner.outer_rail_radius(),
        1e-3,
        &format!(
            "the rail stands 2r from the axis, which is tangency - instead {}",
            apart(on_the_cap, corner.axis())
        ),
    );

    // And the one point it touches at IS the ball's touch point at turn zero.
    assert!(
        on_the_cap.distance_to(corner.outer_rail_point(0.0)) < 1e-3,
        "the outer rail at turn zero stands on the cut's own cap rail - instead {:?}",
        corner.outer_rail_point(0.0)
    );

    // Which stands outside the chord's band: the mitre with the chord before it starts the band
    // 0.0547 further along.
    let handover = (corner.outer_rail_point(0.0) - on_the_cap.point).dot(on_the_cap.direction);
    let count = read.planned.len();
    let before = &read.planned[(read.cut_index as usize + count - 1) % count];
    let before_on_the_cap = on_the_cap_of(before, corner);

    assert_within(
        where_they_cross(on_the_cap, before_on_the_cap, corner.out_of_the_cap_face) - handover,
        0.0547,
        1e-3,
        "so the blend's ring stands that far outside the band it would hand over to",
    );
}

/// And no edge that HAS a strip is ever asked to mitre with one the blend swallowed: a swallowed
/// edge's two neighbours are the corner's own cut and bead, and each of them meets the corner at
/// that end rather than a mitre plane (`SurfaceEdgeRoles`).
#[test]
fn a_swallowed_edges_neighbours_meet_the_corner_rather_than_a_mitre() {
    let read = the_walls_rim(RADIUS);
    let mut roles = SurfaceEdgeRoles::default();

    roles.read(
        read.corner_index,
        read.cut_index,
        read.cut_is_arriving,
        bead_index(&read),
        &read.swallowed,
    );

    let count = read.planned.len() as i32;

    for &swallowed in &read.swallowed {
        let before = (swallowed + count - 1) % count;
        let after = (swallowed + 1) % count;

        let leaving = roles.try_corner_at(before, true).unwrap_or_else(|| {
            panic!("the edge before the swallowed one meets the corner there - instead a mitre with edge {swallowed}")
        });
        assert_eq!(leaving, read.corner_index);

        let arriving = roles.try_corner_at(after, false).unwrap_or_else(|| {
            panic!("and so does the edge after it - instead a mitre with edge {swallowed}")
        });
        assert_eq!(arriving, read.corner_index);
    }
}

/// And WHY the handover had to move at all: the blend's handover plane stands BEYOND the cut
/// chord's own mitre with the chord before it, so that chord has no band left on the cap and no
/// strip to hand over to. The rim plans on the mitre ring instead.
#[test]
fn the_handover_stands_past_the_cut_chords_own_mitre_so_the_ring_moves_to_it() {
    let read = the_walls_rim(RADIUS);
    let corner = &read.corner1;
    let cut = &read.planned[read.cut_index as usize];

    // The cut chord's tangent line on the CAP - the side the two edges do not share, which is the
    // side a handover end is written on.
    let on_the_cap = on_the_cap_of(cut, corner);

    let handover = (corner.tangent_point - on_the_cap.point).dot(on_the_cap.direction);

    assert_within(
        handover,
        0.2405,
        1e-3,
        &format!("the bead's foot stands that far along the chord - instead {handover}"),
    );

    // And the mitre with the chord BEFORE it - away from the corner - which is where the chord's
    // own band starts. The two tangent lines cross there, in the cap's own plane.
    let count = read.planned.len();
    let before = &read.planned[(read.cut_index as usize + count - 1) % count];
    let before_on_the_cap = on_the_cap_of(before, corner);

    let mitre = where_they_cross(on_the_cap, before_on_the_cap, corner.out_of_the_cap_face);

    assert_within(
        mitre,
        0.2952,
        1e-3,
        &format!("a 9 degree turn carries a rail a radius out back by r tan 4.5 - instead {mitre}"),
    );

    assert!(
        mitre > handover,
        "which leaves the chord no band between the two planes, and so no strip"
    );

    // So the chord owes no strip at all: the plan comes back, with that chord swallowed and the
    // ring taken from the mitre before it.
    let planned = SurfaceBevelPlan::try_create(&read.chain, &read.regions, RADIUS, true);

    assert!(
        planned.is_ok(),
        "the handover moved to the mitre ring rather than asking that chord for a strip - instead {:?}",
        planned.err().map(|refusal| refusal.code())
    );
}

/// How far along `line` it crosses `other`, both lying in the plane `facing` names.
fn where_they_cross(line: SurfaceLine, other: SurfaceLine, facing: Vector3) -> f64 {
    let across = line.direction.cross(other.direction).dot(facing);

    (other.point - line.point)
        .cross(other.direction)
        .dot(facing)
        / across
}

/// So the corner hands over on the MITRE ring the chain already has just before the tangency
/// chord: that chord goes under the blend with the others, the chord before it keeps its strip,
/// and the ring is the very plane its band already stops on.
#[test]
fn the_ring_is_the_mitre_the_chain_already_has_just_before_the_tangency() {
    let read = the_walls_rim(RADIUS);
    let (carries_the_ring, ring_stops_at, corner) = the_mitre_handover(&read);

    let carries = &read.planned[carries_the_ring];

    assert!(
        near(carries.to(), read.planned[read.cut_index as usize].from()),
        "the chord before the tangency one carries it - instead {:?} -> {:?}",
        carries.from(),
        carries.to()
    );

    let stops_at =
        ring_stops_at.expect("which means the ring is cut at a plane the chain already has");

    assert!(
        near(stops_at.point, Vector3::new(28.91, -2.54, 20.0)),
        "the mitre stands at the corner between the two chords - instead {:?}",
        stops_at.point
    );

    // And the corner re-read on that ring keeps the tangency it was read from: the ball is where
    // it is, and what changed is only which ring the patch starts from.
    assert!(corner.hands_over_at_a_mitre);
    assert!(
        (corner.tangent_point - read.corner1.tangent_point).length() < 1e-9,
        "the two cylinders touch where they touch"
    );
    assert_within(
        radians_to_degrees(corner.turn_radians()),
        TURN_DEGREES,
        0.02,
        "the same turn",
    );
}

/// And the ring LEANS: its cap vertex stands a little PAST the tangency round the axis and its rail
/// on the shared face well short of it, so the patch's stations cannot all turn by one angle
/// (`surface_mixed_corner_ring_schedule`). Every vertex of it is still on the ideal torus to inside
/// what the wall's own chords already sag.
#[test]
fn the_mitre_ring_leans_but_stays_on_the_ideal_torus() {
    let read = the_walls_rim(RADIUS);
    let (carries_the_ring, ring_stops_at, corner) = the_mitre_handover(&read);

    let strip =
        SurfaceStrip::try_create(&read.planned[carries_the_ring], 8).unwrap_or_else(|refused| {
            panic!(
                "the chord that carries the ring has to rule: {:?}",
                refused.code()
            )
        });

    let stops_at = ring_stops_at.expect("the ring is cut at the mitre");
    let mut on_the_cap = f64::NAN;
    let mut on_the_shared_face = f64::NAN;
    let mut worst = 0.0f64;

    for rail in 0..strip.rail_count() {
        let at = strip
            .try_cross(rail, stops_at.point, stops_at.inward)
            .expect("every rail of the ring crosses the plane it is cut at");

        let point = strip.point_on(rail, at);
        let angle =
            radians_to_degrees(surface_mixed_corner_ring_schedule::angle_of(&corner, point));

        // Where it stands off the ideal torus: the ball's centre runs at 2r from the axis, a radius
        // under the cap, and the surface is a radius from that circle.
        let out_of_the_axis = (point - corner.axis().closest_point_to(point)).length();
        let under_the_cap = (corner.outer_rail_centre - point).dot(corner.out_of_the_cap_face);
        let from_the_centre_line = ((out_of_the_axis - corner.outer_rail_radius()).powi(2)
            + (under_the_cap - RADIUS).powi(2))
        .sqrt();

        worst = worst.max((from_the_centre_line - RADIUS).abs());

        if under_the_cap.abs() < 1e-6 {
            on_the_cap = angle;
        }

        if (under_the_cap - RADIUS).abs() < 1e-6 {
            on_the_shared_face = angle;
        }
    }

    assert_within(
        on_the_cap,
        0.4177,
        0.005,
        &format!("the cap vertex stands that far PAST the tangency round the axis - instead {on_the_cap}"),
    );

    assert_within(
        on_the_shared_face,
        -3.6634,
        0.005,
        &format!(
            "and the rail on the shared face that far short of it - instead {on_the_shared_face}"
        ),
    );

    assert_within(
        on_the_cap - on_the_shared_face,
        4.0811,
        0.005,
        "which is the lean, and why one angle for every station would carry the cap rail past the wall",
    );

    assert_within(worst, 0.0077, 1e-3, &format!("instead {worst}"));

    assert!(
        worst < the_walls_sagitta(),
        "which is inside the sagitta the wall's own chords already stand off their circle"
    );
}

/// And with the touching chord swallowed, the BEAD's own band on the shared face has to stop at its
/// own height too: on the plane a radius under the cap, at the very point the two cylinders touch,
/// which is the corner the walkback on that face is left with.
#[test]
fn the_beads_band_on_the_shared_face_stops_at_its_own_height() {
    let read = the_walls_rim(RADIUS);
    let (_, _, corner) = the_mitre_handover(&read);
    let bead = &read.bead;

    assert!(
        corner.both_bands_stop_on_the_shared_face(),
        "the edge that would have ended the bead's band on the barrel is swallowed, as the cut's ring already stands short of the bead's"
    );

    let stops_at = corner.band_end_for(bead);

    assert!(
        (stops_at.point - corner.tangent_point).length() < 1e-9,
        "the plane stands where the two cylinders touch - instead {:?}",
        stops_at.point
    );

    assert_within(
        stops_at.point.z,
        lobed_prism_meshes::HEIGHT - RADIUS,
        1e-9,
        &format!(
            "which is a radius under the cap - instead {:?}",
            stops_at.point
        ),
    );

    assert!(
        (stops_at.point - Vector3::new(29.0021, -2.3178, lobed_prism_meshes::HEIGHT - RADIUS))
            .length()
            < 1e-3,
        "and that is P, the bead's foot on facet k lifted under the cap - instead {:?}",
        stops_at.point
    );

    // Facing DOWN the seam, away from the corner: the band the bead takes off is the stretch below
    // the stop, and the 3.756 above it is what is left standing for the blend's footprint.
    let away_from_the_corner = ((if near(bead.from(), the_corner()) {
        bead.to()
    } else {
        bead.from()
    }) - the_corner())
    .get_normal();

    assert_within(
        stops_at.inward.dot(away_from_the_corner),
        1.0,
        1e-9,
        &format!(
            "instead {:?} against {away_from_the_corner:?}",
            stops_at.inward
        ),
    );

    assert_within(
        lobed_prism_meshes::HEIGHT - stops_at.point.z,
        RADIUS,
        1e-9,
        "so that much of the seam stands above the stop, under the blend",
    );

    // And the bead's own end ring meets the walkback there: one of its rails crosses the stopping
    // plane at P itself.
    let strip = SurfaceStrip::try_create(bead, 8)
        .unwrap_or_else(|refused| panic!("the bead has to rule: {:?}", refused.code()));

    let mut nearest = f64::MAX;

    for rail in 0..strip.rail_count() {
        if let Some(at) = strip.try_cross(rail, stops_at.point, stops_at.inward) {
            nearest = nearest.min((strip.point_on(rail, at) - corner.tangent_point).length());
        }
    }

    assert!(
        nearest < 1e-9,
        "the ring's rail on the shared face stands on P - instead {nearest} away"
    );
}

/// The corner's two faces, told apart: the bead ROLLED along the face the two edges share - the
/// barrel - so the strip under the blend comes off THAT face, facet by facet, while the face nobody
/// picked is the flat side of the cube and reads as the one plane it is. The strip reaches the
/// RING's own rail and not the tangency: the whole of the chord that carries the ring.
#[test]
fn the_strip_under_the_blend_comes_off_the_face_the_bead_rolled_along() {
    let read = the_walls_rim(RADIUS);
    let (_, _, corner) = the_mitre_handover(&read);

    assert!(
        corner.rolled_is_the_shared_face(),
        "the bead rolls along the barrel, which is the face the two edges share"
    );

    assert!(
        corner.unpicked_section().is_none(),
        "so the face nobody picked is the flat one, and a bead that rolls has only one curved face"
    );

    let faces = SurfaceMixedCornerFaces::try_read(&corner)
        .unwrap_or_else(|refused| panic!("both faces have to read: {:?}", refused.code()));

    assert_eq!(
        faces.unpicked_face.count(),
        1,
        "the flat side of the cube is one plane - instead {} facets",
        faces.unpicked_face.count()
    );

    assert!(
        (faces.unpicked_face.normal_of(0) - Vector3::new(0.0, -1.0, 0.0)).length() < 1e-6,
        "instead {:?}",
        faces.unpicked_face.normal_of(0)
    );

    assert_eq!(
        faces.rolled_section.touched_facet, 3,
        "the bead rolls to the second chord of the barrel, its fourth triangle - instead {}",
        faces.rolled_section.touched_facet
    );

    // The strip on the SHARED face, facet by facet: from the seam out to the ring's own rail.
    let rolled = faces.rolled_section.strip_under_the_blend();
    let mut across = 0.0;

    for (piece, normal) in &rolled {
        for at in piece {
            assert!(
                (*at - piece[0]).dot(*normal).abs() < 1e-9,
                "{at:?} stands off the facet it is bitten from"
            );
        }

        assert_within(
            (piece[0] - piece[3]).length(),
            RADIUS,
            1e-9,
            "the strip is a radius deep - the bead's band stops that far under the cap",
        );

        across += (piece[1] - piece[0]).length();
    }

    assert_eq!(
        rolled.len(),
        4,
        "two triangles of the part chord and two of the chord that carries the ring - instead {}",
        rolled.len()
    );

    assert!(
        (rolled[0].0[0] - the_corner()).length() < 1e-3,
        "the walk starts at the seam itself - instead {:?}",
        rolled[0].0[0]
    );

    assert!(
        (rolled[rolled.len() - 1].0[1] - corner.shared_face_foot()).length() < 1e-9,
        "and ends on the ring's own rail, lifted into the cap - instead {:?}",
        rolled[rolled.len() - 1].0[1]
    );

    assert!(
        (corner.shared_face_foot() - Vector3::new(28.91, -2.54, lobed_prism_meshes::HEIGHT))
            .length()
            < 1e-3,
        "which is the mitre corner between the two chords - instead {:?}",
        corner.shared_face_foot()
    );

    assert_within(
        across,
        2.6904,
        1e-3,
        &format!(
            "1.1208 of the part chord and the whole 1.5696 of the one that carries the ring - instead {across}"
        ),
    );

    // And the flat face's own strip stands beside it rather than instead of it.
    assert_eq!(
        faces.strips_under_the_blend().len(),
        rolled.len() + 1,
        "the face nobody picked gives up its square as well, because its band stopped at the same height"
    );

    let flat = faces.unpicked_face.strip_under_the_blend();

    assert!(
        (flat[0].0[1] - corner.cap_corner()).length() < 1e-9,
        "the flat face's strip ends at the bead's foot on it - instead {:?}",
        flat[0].0[1]
    );
}

/// The cap's own sector takes the WEDGE the leaning ring stands off by: its first side is the trace
/// of the plane the ring was cut at rather than the radius through the ring's cap vertex
/// (`surface_mixed_corner_sector`). 4.0769 degrees of wedge, and 2.0737 square millimetres between
/// the two lines.
#[test]
fn the_sector_takes_the_wedge_its_leaning_ring_stands_off_by() {
    let read = the_walls_rim(RADIUS);
    let (_, _, corner) = the_mitre_handover(&read);

    let ring_on_the_cap = the_rings_own_vertex_on_the_cap(&corner);
    let last_on_the_cap = corner.outer_rail_point(corner.turn_radians());

    let apex = surface_mixed_corner_sector::try_apex(&corner, ring_on_the_cap, last_on_the_cap)
        .unwrap_or_else(|refused| panic!("the apex has to read: {:?}", refused.code()));

    // On the last station's own radius, so that the side the runout's sector shares is the same
    // line it always was - and off the axis along it, which is the whole of what the wedge costs.
    let along_the_radius = (last_on_the_cap - corner.outer_rail_centre).get_normal();
    let from_the_centre = apex - corner.outer_rail_centre;

    assert!(
        (from_the_centre - (along_the_radius * from_the_centre.dot(along_the_radius))).length()
            < 1e-9,
        "the apex stays on that radius - instead {apex:?}"
    );

    assert_within(
        from_the_centre.dot(along_the_radius),
        -0.5995,
        1e-3,
        &format!(
            "and stands that far back along it, on the far side of the axis - instead {apex:?}"
        ),
    );

    // The first side now runs along the ring's own stopping plane, which is what gives it the
    // wedge.
    let along_the_first_side = (ring_on_the_cap - apex).get_normal();

    assert!(
        along_the_first_side.dot(corner.cut_stops_at.inward).abs() < 1e-9,
        "instead {along_the_first_side:?} against {:?}",
        corner.cut_stops_at.inward
    );

    // And what the sector gained is the triangle between the two lines: the axis, the apex and the
    // ring's own vertex on the cap.
    let wedge = from_the_centre
        .cross(ring_on_the_cap - corner.outer_rail_centre)
        .length()
        / 2.0;

    assert_within(wedge, 2.0737, 1e-3, &format!("instead {wedge}"));

    // And the wedge is the lean itself, read at the ring's own cap vertex.
    let off_the_radius = radians_to_degrees(
        (ring_on_the_cap - corner.outer_rail_centre)
            .get_normal()
            .dot(along_the_first_side)
            .clamp(-1.0, 1.0)
            .acos(),
    );

    assert_within(
        off_the_radius,
        4.0769,
        0.01,
        &format!("instead {off_the_radius}"),
    );
}

/// The ring's own vertex on the cap face: the cut's tangent line there, met with the plane the ring
/// is cut at - the same two readings the patch's first column is built between.
fn the_rings_own_vertex_on_the_cap(corner: &SurfaceMixedCorner) -> Vector3 {
    let on_the_cap = on_the_cap_of(&corner.cut, corner);

    let across = on_the_cap.direction.dot(corner.cut_stops_at.inward);

    on_the_cap.point
        + (on_the_cap.direction
            * ((corner.cut_stops_at.point - on_the_cap.point).dot(corner.cut_stops_at.inward)
                / across))
}

/// And the whole rim BUILDS: closed, manifold, one shell, and taking less off the finer it is
/// chorded - the last face of "Every Chamfer Works" that did not.
#[test]
fn the_walls_whole_rim_closes_and_comes_down_on_the_smooth_answer() {
    for (segments, takes_off, coarser) in [
        (4, 182.32437, f64::MAX),
        (8, 175.40976, 182.32437),
        (16, 173.62329, 175.40976),
        (32, 173.18687, 173.62329),
    ] {
        let read = the_walls_rim(RADIUS);
        let built = the_rim_built(&read, segments);

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(open, 0, "open at {open_at:?} ({segments} segments)");
        assert!(built.is_manifold(), "{segments} segments");
        assert_eq!(shell_count(&built), 1, "{segments} segments");

        let took = signed_volume(&read.prism) - signed_volume(&built);

        assert_within(took, takes_off, 1e-3, &format!("instead {took}"));

        // And each reading takes LESS off than the one before it.
        assert!(
            took < coarser,
            "at {segments} segments it took {took}, and half the stations took {coarser}"
        );
    }
}

/// And the blend the build put there is on the ideal torus about the bead's axis: every vertex of
/// the part standing anywhere near that torus, within the turn the blend sweeps, is on it to inside
/// the sagitta the barrel's own chords already stand off their circle. Read off the PUBLISHED mesh.
#[test]
fn every_vertex_of_the_blend_stands_on_the_ideal_torus_about_the_axis() {
    let read = the_walls_rim(RADIUS);
    let (_, _, corner) = the_mitre_handover(&read);
    let built = the_rim_built(&read, 16);

    // What the mesh already carries there: the sagitta of one of the barrel's own chords.
    let sagitta = the_walls_sagitta();

    let mut on_the_torus = 0;
    let mut worst = 0.0f64;

    for &stored in &built.vertices {
        let at = Vector3::from_vector3_float(stored);
        let turn = surface_mixed_corner_ring_schedule::angle_of(&corner, at);

        if turn < -0.05 || turn > corner.turn_radians() + 0.05 {
            continue;
        }

        // How far it stands off the ideal torus: the ball's centre runs at 2r from the axis, a
        // radius under the cap, and the surface is a radius from that circle.
        let out_of_the_axis = (at - corner.axis().closest_point_to(at)).length();
        let under_the_cap = (corner.outer_rail_centre - at).dot(corner.out_of_the_cap_face);
        let from_the_centre_line = ((out_of_the_axis - corner.outer_rail_radius()).powi(2)
            + (under_the_cap - RADIUS).powi(2))
        .sqrt();

        let off = (from_the_centre_line - RADIUS).abs();

        if off > 0.2 || out_of_the_axis > corner.outer_rail_radius() + 1e-6 {
            // Past the outer rail is the cap's own material, which the blend never reached: the
            // torus is TANGENT to the cap there.
            continue;
        }

        on_the_torus += 1;
        worst = worst.max(off);
    }

    assert!(
        on_the_torus > 20,
        "the blend has to BE there to be measured - instead {on_the_torus} vertices near the torus"
    );

    assert!(
        worst < sagitta,
        "the worst stands {worst} off the torus, where the barrel's own chords sag {sagitta}"
    );
}

/// The sagitta of one of the barrel's own chords.
fn the_walls_sagitta() -> f64 {
    lobed_prism_meshes::RADIUS
        * (1.0 - degrees_to_radians(180.0 / f64::from(lobed_prism_meshes::SIDES)).cos())
}

/// The wall region's rim, planned whole and built at `segments` stations.
fn the_rim_built(read: &Rim, segments: i32) -> Mesh {
    let plan = SurfaceBevelPlan::try_create(&read.chain, &read.regions, RADIUS, true)
        .unwrap_or_else(|why| panic!("the rim has to plan: {why}"));

    surface_bevel_builder::try_build(
        &read.prism,
        &read.prism.get_mesh_edge_graph(),
        &read.regions,
        &[plan],
        segments,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .map(|built| built.mesh)
    .unwrap_or_else(|reason| panic!("the rim has to build: {reason:?}"))
}

/// Where the corner hands over, read off the plans as the plan itself reads it: the edge that
/// carries the ring, the plane the ring stops at, and the corner re-read on that ring.
fn the_mitre_handover(read: &Rim) -> (usize, Option<SurfaceBandEnd>, Rc<SurfaceMixedCorner>) {
    let (carries, stops_at) = surface_mixed_corner_hands_over_at_the_mitre::try_read(
        &read.planned,
        read.chain.is_closed,
        read.cut_index as usize,
        read.cut_is_arriving,
        &read.corner1,
    );

    let count = read.planned.len();
    let corner = SurfaceMixedCorner::try_read_carried(
        &read.planned[carries],
        &read.planned[read.cut_index as usize],
        stops_at,
        &read.planned[read.corner_index as usize],
        &read.planned[(read.corner_index as usize + 1) % count],
        the_corner(),
        RADIUS,
        SurfaceVertexPool::near_enough_for(&read.prism.vertices),
    )
    .unwrap_or_else(|refusal| {
        panic!(
            "the corner has to read on the mitre ring: {:?}",
            refusal.code()
        )
    });

    (carries, stops_at, Rc::new(corner))
}

/// The edge's tangent line on the CAP - the side the corner's two edges do not share.
fn on_the_cap_of(
    edge: &geometry_processing::bevel::surface::SurfaceEdgePlan,
    corner: &SurfaceMixedCorner,
) -> SurfaceLine {
    if edge.left_region_key() == corner.shared_region_key {
        edge.right_tangent_line
    } else {
        edge.left_tangent_line
    }
}

/// Whether two points are the same place on this fixture's scale.
fn near(first: Vector3, second: Vector3) -> bool {
    (first - second).length() < 1e-3
}
