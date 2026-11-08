//! The patch that closes a TWO-crease junction - one cut and two beads
//! (`surface_two_crease_junction_patch`), read on the nose-in-notch fixture the rest of the
//! two-crease reading is measured on (`surface_two_crease_junction_reading_tests`,
//! `surface_two_crease_junction_rings_tests`).

use std::collections::HashMap;
use std::sync::Arc;

use geometry_processing::bevel::surface::{surface_bevel_builder, SurfaceMixedJunction};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes;
use crate::surface_two_crease_junction_reading_tests::{
    the_junction, the_plans_of_the_right_ridge_end, RADIUS,
};
use crate::surface_two_crease_junction_rings_tests::the_rings;

/// What ARRIVES at a two-crease junction is the cut ONCE and both beads - not, as a one-crease
/// junction's assembly would have it, both arms' cuts, which here are the same edge twice, and
/// only the first arm's bead.
#[test]
fn the_arrivals_are_the_cut_once_and_both_beads() {
    let (junction, _, plans) = the_junction();
    let arriving = junction.the_arriving_ends();

    assert_eq!(arriving.len(), 3, "three chains meet at the corner");

    // The cut, at its own end of its own chain - the edge BOTH arms were read off.
    assert_eq!(arriving[0].plan, junction.first.plan);
    assert_eq!(arriving[0].edge, junction.first.cut_index);
    assert_eq!(arriving[0].at_the_start, junction.first.corner_at_the_start);

    assert_eq!(
        SurfaceMixedJunction::edge_of(&plans, arriving[0]).convexity(),
        EdgeConvexity::Convex,
        "the one arrival that is not a crease is the ridge"
    );

    // And then a bead each, which is what the one-crease assembly had no room for.
    for (arrival, bead) in [
        (arriving[1], junction.first.bead_end),
        (arriving[2], junction.second.bead_end),
    ] {
        assert_eq!(arrival.plan, bead.plan);
        assert_eq!(arrival.edge, bead.edge);
        assert_eq!(arrival.at_the_start, bead.at_the_start);

        assert_eq!(
            SurfaceMixedJunction::edge_of(&plans, arrival).convexity(),
            EdgeConvexity::Concave,
            "both of the other two arrivals are creases"
        );
    }

    // Three DIFFERENT chains, which is what says the cut was not handed in twice.
    assert_ne!(arriving[1].plan, arriving[0].plan);
    assert_ne!(arriving[2].plan, arriving[0].plan);
    assert_ne!(arriving[2].plan, arriving[1].plan);
}

/// Under each arm the wall gave up exactly the stretch the patch now covers: the wedge between the
/// bead's own band end, its foot on the crease and row zero. Its inside is gone, and the two
/// corners of the outline it was bounded by - the band end's own edge and the crease beyond the
/// crossing - are gone with it.
#[test]
fn the_wall_gives_up_one_convex_bite_under_each_arm() {
    let built = the_build(None);

    // One point well inside each bite, taken off the coverage the wall had before the patch was
    // laid: at z = 12.5 the wall reached from x = 11.92 outwards and row zero stood at x = 13.4.
    for in_the_bite in [Vector3::new(12.5, 7.5, 12.5), Vector3::new(12.5, 7.5, 7.5)] {
        assert_eq!(
            the_wall_faces_covering(&built, in_the_bite),
            0,
            "the patch covers {in_the_bite:?} now, so no wall face may still stand under it"
        );
    }

    // And the two corners the bite turned at are off the wall's outline altogether: both sides of
    // each were the bite's own rim, so neither is anybody's boundary any more.
    let outline = the_walls_outline_near_the_tip(&built);

    for foot in [
        Vector3::new(11.800611, 8.199389, 12.132926),
        Vector3::new(11.800611, 8.199389, 7.867074),
    ] {
        assert!(
            !outline
                .iter()
                .any(|(from, to)| (*from - foot).length() < 1e-5 || (*to - foot).length() < 1e-5),
            "the bead's foot at {foot:?} was a corner of what the wall gave up, not of what it kept"
        );
    }
}

/// And the notch between the two crossings (wall the two bands took and the patch does not reach)
/// is filled from the corner itself, so what the wall ends on near the tip is ROW ZERO, rail for
/// rail, and nothing else.
#[test]
fn the_noses_root_is_filled_from_the_apex() {
    let built = the_build(None);
    let apex = nose_in_notch_test_meshes::right_ridge_end();

    // Inside the notch and off the corner's own height, where the fan's triangles all meet: before
    // the fill the wall reached only to x = 14.25 at this height, and row zero stands at 14.0.
    assert_eq!(
        the_wall_faces_covering(&built, Vector3::new(14.1, 5.9, 10.5)),
        1,
        "the notch's own middle stood on no face at all before the fill, and stands on one now"
    );

    // The fill is fanned from the corner, so the corner is a vertex of the wall again - and every
    // triangle of it reaches row zero.
    let (rings, _, _) = the_rings();

    let row_zero: Vec<Vector3> = (0..rings.station_count())
        .map(|station| rings.at(0, station))
        .collect();

    // The fan's own triangles are the wall faces at the corner whose other two corners BOTH stand
    // on row zero - on the CHORDED row and not merely near one of its rails, because the two ends
    // of the fan are the crossings, which stand part way along a chord. The rest of the wall's
    // faces at the corner run away across the part.
    let fanned: Vec<[Vector3; 3]> = the_wall_faces_at(&built, apex)
        .into_iter()
        .filter(|face| {
            face.iter()
                .filter(|at| (**at - apex).length() > 1e-9)
                .all(|at| off_row_zero(*at, &row_zero) < 1e-5)
        })
        .collect();

    assert!(
        fanned.len() > 1,
        "the notch is fanned from the corner over more than one chord of row zero - instead {} \
         triangles of it stand there",
        fanned.len()
    );

    // And it runs CREASE TO CREASE: one of its corners stands on each of the two lines the wall's
    // own boundary came in to the corner along, which is where row zero left the wall.
    for (name, along) in [
        ("upper", Vector3::new(12.503849, 7.496151, 11.664101) - apex),
        ("lower", Vector3::new(12.503849, 7.496151, 8.335899) - apex),
    ] {
        let corners: Vec<Vector3> = fanned
            .iter()
            .flatten()
            .copied()
            .filter(|at| (*at - apex).length() > 1e-9)
            .collect();
        let direction = along.get_normal();

        assert!(
            corners.iter().any(|at| {
                ((*at - apex) - (direction * (*at - apex).dot(direction))).length() < 1e-5
            }),
            "the fan ends on the {name} crease - instead its corners are {corners:?}"
        );
    }

    // And what the wall ENDS on near the tip is row zero the whole way, rail for rail: the two
    // bites took it out to the crossings and the fill carried it between them.
    let outline = the_walls_outline_near_the_tip(&built);

    for (from, to) in outline {
        for at in [from, to] {
            assert!(
                off_row_zero(at, &row_zero) < 1e-4,
                "instead the wall still ends at {at:?}, which stands on no chord of row zero"
            );
        }
    }
}

/// A crease CHORDED beside the corner - two mesh edges where this fixture has one, which is what
/// the boolean under the corpus parts leaves - answers exactly what the whole crease answers: the
/// canal's first ball stands 5.002 mm along the crease and the chord runs out at 4.690, so the
/// chord has no band left to carry the ring and is SWALLOWED, the way a mixed corner's cut swallows
/// the chords between it and its corner.
///
/// Vertex for vertex, because chording a crease is a triangulation of the same solid and not a
/// different part: the chord's own point stands inside the band the walkback takes off, so it is
/// not in the answer at all. Without the swallowing the pass dies before any of this - the walkback
/// leaves the chord no tangent run on either side and the seam over it cannot be picked, which is
/// exactly where both corpus fixtures stood.
#[test]
fn the_creases_chord_beside_the_corner_is_swallowed() {
    let whole = the_build(None);
    let chorded = the_build(Some(the_chord_on_the_upper_crease()));

    assert!(
        chorded.is_manifold(),
        "the chorded crease closes as the whole one does"
    );
    assert_eq!(shell_count(&chorded), 1, "and nothing of it comes away");

    let here: Vec<Vector3> = whole
        .vertices
        .iter()
        .map(|vertex| Vector3::from_vector3_float(*vertex))
        .collect();
    let there: Vec<Vector3> = chorded
        .vertices
        .iter()
        .map(|vertex| Vector3::from_vector3_float(*vertex))
        .collect();

    // The same SOLID: the chord is a triangulation of the part and not a different part, and the
    // blend over it is the same canal on the same creases.
    let wholly_volume = signed_volume(&whole);
    let chorded_volume = signed_volume(&chorded);

    // A ten-thousandth of a mm^3 out of 6042.76, which is 2e-8 of it. Not zero, and it should not
    // be: the chorded slope leaves the walkback a few extra tangent vertices, the strip puts a
    // station at every one of them, and a round chorded at more stations encloses a hair more of
    // its own arc. Measured, the two stand 3.8e-5 apart.
    assert!(
        (chorded_volume - wholly_volume).abs() <= 1e-4,
        "instead the chorded answer measures {chorded_volume:.6} against {wholly_volume:.6}"
    );

    // And the same SURFACE, both ways round. Not the same vertices: the chorded slope's own fan
    // leaves the walkback tangent vertices the whole one has not, a strip puts a station at every
    // one of them, and neither set of points contains the other. What has to hold is that every
    // point of each answer lies ON the other answer, which is what says the blend landed in the
    // same place and was chorded differently rather than drawn differently.
    for (name, from, onto) in [
        ("the whole crease's", &here, &chorded),
        ("the chorded one's", &there, &whole),
    ] {
        let worst = from
            .iter()
            .map(|at| off_the_surface(*at, onto))
            .fold(f64::MIN, f64::max);

        assert!(
            worst <= 1e-3,
            "every vertex of {name} answer stands on the other's surface - instead one is \
             {worst:e} off it"
        );
    }
}

/// How far `at` stands off the nearest triangle of `mesh`.
fn off_the_surface(at: Vector3, mesh: &Mesh) -> f64 {
    let mut off = f64::MAX;

    for face in mesh.faces.iter() {
        off = off.min(off_the_triangle(
            at,
            Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]),
        ));
    }

    off
}

/// How far a point stands off one triangle: its distance to the nearest point of the closed
/// triangle, which is either inside the face or on one of its three edges. (This test's own
/// measure, which reads a degenerate triangle by its edges rather than skipping it as
/// `mesh_surface_distance` does.)
fn off_the_triangle(at: Vector3, first: Vector3, second: Vector3, third: Vector3) -> f64 {
    let mut normal = (second - first).cross(third - first);

    if normal.length_squared() > 0.0 {
        normal = normal.get_normal();

        let on_the_plane = at - (normal * (at - first).dot(normal));

        if inside(on_the_plane, first, second, third, normal)
            && inside(on_the_plane, second, third, first, normal)
            && inside(on_the_plane, third, first, second, normal)
        {
            return (at - on_the_plane).length();
        }
    }

    off_the_segment(at, first, second)
        .min(off_the_segment(at, second, third).min(off_the_segment(at, third, first)))
}

fn inside(at: Vector3, from: Vector3, to: Vector3, opposite: Vector3, normal: Vector3) -> bool {
    let outward = (to - from).cross(normal);

    (at - from).dot(outward) * (opposite - from).dot(outward) >= -1e-18
}

fn off_the_segment(at: Vector3, from: Vector3, to: Vector3) -> f64 {
    let along = to - from;

    if along.length_squared() == 0.0 {
        return (at - from).length();
    }

    let how = ((at - from).dot(along) / along.length_squared()).clamp(0.0, 1.0);

    (at - (from + (along * how))).length()
}

/// Where the boolean under "Center Flows Through.mcx" chords the upper right crease - at its
/// `t = 0.6`, 4.690 mm along a crease 7.817 long.
fn the_chord_on_the_upper_crease() -> Vector3 {
    Vector3::new(12.0, 8.0, 12.0)
}

/// How far a point stands off the CHORDED row zero - its nearest point on any chord of it.
fn off_row_zero(at: Vector3, row_zero: &[Vector3]) -> f64 {
    let mut off = f64::MAX;

    for rail in 0..row_zero.len().saturating_sub(1) {
        let along = row_zero[rail + 1] - row_zero[rail];
        let how = ((at - row_zero[rail]).dot(along) / along.length_squared()).clamp(0.0, 1.0);

        off = off.min((at - (row_zero[rail] + (along * how))).length());
    }

    off
}

/// The published surface of the fixture's own pass, which every measurement here is of.
/// `the_upper_crease_chorded_at` is where to chord the upper crease into two mesh edges, or `None`
/// to leave it whole - which is the difference between this fixture and the boolean the corpus
/// parts carry.
fn the_build(the_upper_crease_chorded_at: Option<Vector3>) -> Mesh {
    let mesh = Arc::new(match the_upper_crease_chorded_at {
        Some(split) => nose_in_notch_test_meshes::create_with_upper_right_crease_split_at(split),
        None => nose_in_notch_test_meshes::create(),
    });
    let features = FeatureEdgeSet::create(&mesh);
    let regions = Arc::new(FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let plans =
        the_plans_of_the_right_ridge_end(&features, &regions, RADIUS, the_upper_crease_chorded_at);

    surface_bevel_builder::try_build(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        &plans,
        16,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .unwrap_or_else(|refused| panic!("the pass refused: {refused:?}"))
    .mesh
}

/// The corners of every published face standing wholly in the WALL's own plane, x + y = 20.
fn the_wall_faces(built: &Mesh) -> Vec<[Vector3; 3]> {
    built
        .faces
        .iter()
        .map(|face| {
            [face.v0, face.v1, face.v2]
                .map(|vertex| Vector3::from_vector3_float(built.vertices[vertex as usize]))
        })
        .filter(|corners| corners.iter().all(|at| (at.x + at.y - 20.0).abs() < 1e-5))
        .collect()
}

/// How many of those cover `probe`, read in the wall's own plane.
fn the_wall_faces_covering(built: &Mesh, probe: Vector3) -> i32 {
    let side = |from: Vector3, to: Vector3| {
        ((to.x - from.x) * (probe.z - from.z)) - ((to.z - from.z) * (probe.x - from.x))
    };

    the_wall_faces(built)
        .iter()
        .filter(|face| {
            let first = side(face[0], face[1]);
            let second = side(face[1], face[2]);
            let third = side(face[2], face[0]);

            (first >= -1e-9 && second >= -1e-9 && third >= -1e-9)
                || (first <= 1e-9 && second <= 1e-9 && third <= 1e-9)
        })
        .count() as i32
}

/// Those of them with a corner AT `at`.
fn the_wall_faces_at(built: &Mesh, at: Vector3) -> Vec<[Vector3; 3]> {
    the_wall_faces(built)
        .into_iter()
        .filter(|face| face.iter().any(|corner| (*corner - at).length() < 1e-6))
        .collect()
}

/// What the wall ENDS on near the tip: the edges used by exactly one face of the wall's own plane,
/// both ends of which stand within reach of the corner. Corners are keyed rounded to six places,
/// with banker's rounding (ties to even).
fn the_walls_outline_near_the_tip(built: &Mesh) -> Vec<(Vector3, Vector3)> {
    type Key = (i64, i64, i64);
    let rounded = |at: Vector3| -> Key {
        (
            (at.x * 1e6).round_ties_even() as i64,
            (at.y * 1e6).round_ties_even() as i64,
            (at.z * 1e6).round_ties_even() as i64,
        )
    };
    let back = |key: Key| Vector3::new(key.0 as f64 / 1e6, key.1 as f64 / 1e6, key.2 as f64 / 1e6);

    // Kept in insertion order, so the outline reads the same on every run.
    let mut order: Vec<(Key, Key)> = Vec::new();
    let mut used: HashMap<(Key, Key), i32> = HashMap::new();

    for face in the_wall_faces(built) {
        for side in 0..3 {
            let from = rounded(face[side]);
            let to = rounded(face[(side + 1) % 3]);
            let key = if from.0 < to.0 || (from.0 == to.0 && from.2 < to.2) {
                (from, to)
            } else {
                (to, from)
            };

            let count = used.entry(key).or_insert(0);
            if *count == 0 {
                order.push(key);
            }
            *count += 1;
        }
    }

    let apex = nose_in_notch_test_meshes::right_ridge_end();

    order
        .into_iter()
        .filter(|key| used[key] == 1)
        .map(|(from, to)| (back(from), back(to)))
        .filter(|(from, to)| (*from - apex).length() < 5.0 && (*to - apex).length() < 5.0)
        .collect()
}
