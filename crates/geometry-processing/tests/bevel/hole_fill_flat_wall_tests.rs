//! Ports `Tests/GeometryProcessingTests/LevelLine/HoleFillFlatWallTests.cs`: a fillet's corner
//! hole on a flat-faceted wall closed by the hole fill, kept inside the wall facets its rail runs
//! on, and the crease readings the fill turns its cross-tangents by.

use std::collections::HashSet;

use geometry_processing::bevel::level_line::corner_cap::{close, CornerCapLastRow};
use geometry_processing::bevel::level_line::corner_cap_hole_fill::{
    inside_the_creases, leaves_through_convex_crease, out_of_the_planes,
};
use geometry_processing::bevel::level_line::{GapFillRefusal, LevelLineMesh};
use vector_math::Vector3;

const CYLINDER_CENTRE_X: f64 = 5.0;
const CYLINDER_CENTRE_Y: f64 = -4.0;
const CYLINDER_RADIUS: f64 = 10.0;
const FILLET_RADIUS: f64 = 2.0;
const FILLET_SEGMENTS: i32 = 8;

const ROW_TOP: usize = 8;
const ROW_WALL: usize = 16;

#[allow(clippy::excessive_precision)]
fn hole_loop() -> Vec<Vector3> {
    vec![
        Vector3::new(-3.910064697265625, -8.539905548095703, 18.759972195191658),
        Vector3::new(-3.784527719547049, -8.744763214484179, 19.142565283341135),
        Vector3::new(-3.68276939489067, -8.910817657593668, 20.0),
        Vector3::new(-3.490791869404615, -8.810654269600368, 20.0),
        Vector3::new(-3.2219342407744644, -8.619690808288603, 20.0),
        Vector3::new(-2.989032144109456, -8.388222617599713, 20.0),
        Vector3::new(-2.7973069964078365, -8.122036977796755, 20.0),
        Vector3::new(-2.6515471590752466, -7.827770613256137, 20.0),
        Vector3::new(-2.555718745018957, -7.512754444189912, 20.0),
        Vector3::new(-2.9114285009210374, -7.435859845364316, 19.966610736833168),
        Vector3::new(-3.2752872373661535, -7.357203662608852, 19.859576602124935),
        Vector3::new(-3.6254295749021073, -7.281512585459719, 19.673991022437477),
        Vector3::new(-3.9380036495548025, -7.213942726396543, 19.414213562373416),
        Vector3::new(-4.191916121574134, -7.1590538813514355, 19.09441950677078),
        Vector3::new(-4.37331174110926, -7.11984117165923, 18.73618941912444),
        Vector3::new(-4.47792936931886, -7.097225737536932, 18.36392610482519),
        Vector3::new(-4.510564804077148, -7.090170860290527, 18.000000000001098),
        Vector3::new(-4.450698773176372, -7.234700162839531, 18.00612755196175),
        Vector3::new(-4.264197913389476, -7.684952815039999, 18.106448368414835),
        Vector3::new(-3.951037810699587, -8.440987757509474, 18.635366989307727),
    ]
}

#[test]
fn the_fill_stays_inside_the_flat_wall_facets_its_rail_runs_on() {
    let (mut mesh, open, row, corner) = the_hole_on_its_surround();
    let mut added = Vec::new();
    let mut added_vertices = Vec::new();
    let (refusal, _) = close(
        &mut mesh,
        &open,
        &[corner],
        Some(std::slice::from_ref(&row)),
        FILLET_SEGMENTS,
        &mut added,
        &mut added_vertices,
        None,
    )
    .expect("closed");
    assert_eq!(refusal, GapFillRefusal::None);
    assert!(!added_vertices.is_empty());

    // a fillet row's chord stands this far inside its arc - the most a faceted round is ever off
    // its surface
    let chord_error =
        FILLET_RADIUS * (1.0 - (std::f64::consts::PI / (4.0 * FILLET_SEGMENTS as f64)).cos());
    for (from_degrees, to_degrees) in [(198.0, 207.0), (207.0, 216.0)] {
        let from = rim_vertex(from_degrees);
        let outward = wall_outward(from_degrees, to_degrees);
        // C#'s `OrderByDescending(...).First()`: the first of the farthest out.
        let mut worst = mesh.position(added_vertices[0]);
        for &v in &added_vertices[1..] {
            let p = mesh.position(v);
            if outward.dot(p - from) > outward.dot(worst - from) {
                worst = p;
            }
        }
        assert!(
            outward.dot(worst - from) <= chord_error,
            "fill vertex {worst:?} stands {:.4} outside the wall facet {from_degrees}-{to_degrees} its rail runs on",
            outward.dot(worst - from)
        );
    }
}

#[test]
fn a_convex_crease_is_read_from_the_holes_side() {
    let wall = Vector3::new(-1.0, 0.0, 0.0);
    let into_hole = Vector3::new(-0.5, 0.5, 0.0);

    // toward the hole the chain leaves out through the wall, whichever way the row's normal is signed
    assert!(leaves_through_convex_crease(
        Vector3::new(-1.0, 1.0, 0.0).get_normal(),
        into_hole,
        wall
    ));
    assert!(leaves_through_convex_crease(
        Vector3::new(1.0, -1.0, 0.0).get_normal(),
        into_hole,
        wall
    ));

    // concave: toward the hole the chain runs behind the wall, into the air the fill belongs in
    assert!(!leaves_through_convex_crease(
        Vector3::new(1.0, 1.0, 0.0).get_normal(),
        Vector3::new(0.5, 0.5, 0.0),
        wall
    ));

    // a chain square to the hole's side gives no way in to read, so it is no crease - even
    // pointing through the wall
    assert!(!leaves_through_convex_crease(
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        wall
    ));
}

#[test]
fn two_crease_faces_at_a_slant_are_left_together() {
    let n1 = Vector3::new(1.0, 0.0, 0.0);
    let n2 = Vector3::new(0.5, 0.75f64.sqrt(), 0.0);
    let inside = out_of_the_planes(Vector3::new(0.6, 0.3, 0.74).get_normal(), &[n1, n2]);

    // removed one after the other, n2's removal puts some n1 back; jointly the result lies in both
    // planes
    assert!(inside.dot(n1).abs() < 1e-12);
    assert!(inside.dot(n2).abs() < 1e-12);
    assert!((inside.dot(n1.cross(n2).get_normal()).abs() - 1.0).abs() <= 1e-12);
}

#[test]
fn the_far_end_of_a_row_keeps_its_own_cross_tangent() {
    // a crease at the first end only, the far end's own face overhanging: the far end is not turned
    // toward the crease face at all, so it meets its own face as before - no bump, no dent there
    let cross_tangent = Vector3::new(0.0, 1.0, 0.0);
    let crease_face = [Vector3::new(-0.8, 0.6, 0.0)];
    let none: [Vector3; 0] = [];
    let far_end = inside_the_creases(cross_tangent, 8, 9, &crease_face, &none);
    assert!((far_end - cross_tangent).length() < 1e-12);

    let crease_end = inside_the_creases(cross_tangent, 0, 9, &crease_face, &none);
    assert!(crease_end.dot(crease_face[0]).abs() < 1e-12);

    // creases at both ends: each fades out by the row's middle
    let middle = inside_the_creases(cross_tangent, 4, 9, &crease_face, &crease_face);
    assert!((middle - cross_tangent).length() < 1e-12);
}

fn the_hole_on_its_surround() -> (
    LevelLineMesh,
    HashSet<(i32, i32)>,
    CornerCapLastRow,
    Vector3,
) {
    let hole = hole_loop();
    let mut positions = hole.clone();
    let mut triangles = Vec::new();
    let mut sum = Vector3::ZERO;
    for &p in &hole {
        sum += p;
    }
    let centroid = sum / hole.len() as f64;
    let n = hole.len();
    for i in 0..n {
        if (ROW_TOP..ROW_WALL).contains(&i) {
            continue;
        }

        let j = (i + 1) % n;
        let middle = (hole[i] + hole[j]) / 2.0;
        let on_the_top = hole[i].z == 20.0 && hole[j].z == 20.0;
        let mut away = middle - centroid;
        away.z = 0.0;
        positions.push(if on_the_top {
            middle + (away.get_normal() * 0.5)
        } else {
            middle - Vector3::new(0.0, 0.0, 1.0)
        });

        // the kept surface runs the loop backwards
        triangles.push((j as i32, i as i32, positions.len() as i32 - 1));
    }

    let mesh = LevelLineMesh::from_triangles(&positions, &triangles, None).expect("a mesh");
    let open: HashSet<(i32, i32)> = (0..n as i32).map(|i| (i, (i + 1) % n as i32)).collect();
    let chain_point = Vector3::new(hole[ROW_WALL].x, hole[ROW_WALL].y, hole[ROW_TOP].z);
    let row = CornerCapLastRow::new(
        (ROW_TOP as i32..=ROW_WALL as i32).collect(),
        chain_point,
        &mesh,
        false,
        Vector3::ZERO,
    );

    // the corner G3's chain runs toward, a millimetre along the chain from its end
    let chain = (hole[ROW_TOP] - row.centre)
        .cross(hole[ROW_WALL] - row.centre)
        .get_normal();
    (mesh, open, row, chain_point + chain)
}

fn wall_outward(from_degrees: f64, to_degrees: f64) -> Vector3 {
    let from = rim_vertex(from_degrees);
    let to = rim_vertex(to_degrees);
    let outward = Vector3::new(to.y - from.y, from.x - to.x, 0.0).get_normal();
    if outward.dot(from - Vector3::new(CYLINDER_CENTRE_X, CYLINDER_CENTRE_Y, 0.0)) < 0.0 {
        outward * -1.0
    } else {
        outward
    }
}

fn rim_vertex(degrees: f64) -> Vector3 {
    let radians = degrees * std::f64::consts::PI / 180.0;
    Vector3::new(
        CYLINDER_CENTRE_X + (CYLINDER_RADIUS * radians.cos()),
        CYLINDER_CENTRE_Y + (CYLINDER_RADIUS * radians.sin()),
        0.0,
    )
}
