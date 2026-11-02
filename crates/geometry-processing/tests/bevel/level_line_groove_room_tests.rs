//! The level line's own build of a few grooves, held to answers worked out by hand: the groove
//! with one wall bending steeper past a narrow first face, leaning off square at both ends, whose
//! leaning end faces are capped over the round with the round's own section in each face
//! (`end_face_section_row`); the groove whose band's side floods join round its free ends; a plain
//! V-groove on a thin floor, picked short of its square ends, whose room is the distance from the
//! end vertex to the floor's crease; the one-wall-bending groove's ring, with a room handed in; a
//! groove whose walls bend steeper, set back by the angle; and a plate's top edge past and within
//! its room.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult, LevelLineMesh,
    RoomReadingKind,
};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

type Triangles = Vec<(i32, i32, i32)>;

/// The groove's walls past the first bend run at this angle to the horizontal, in degrees.
const STEEP_DEGREES: f64 = 65.0;

/// The leaning groove's chain: the floor from (0, 0, 0) to (0, 6, 0).
const LENGTH: f64 = 6.0;

/// The leaning groove's fillet size.
const RADIUS: f64 = 2.0;

/// How near a point must stand to a hand answer: the field's root precision.
const NEAR: f64 = 1e-6;

/// The sliced grooves' length along y.
const SLICED_LENGTH: f64 = 16.0;

/// How many slices the sliced grooves are cut into along y, so the floor is a chain of edges
/// [`SLICED_LENGTH`] / [`SLICES`] = 0.25 long.
const SLICES: i32 = 64;

/// The one-wall-bending groove's ring: the profile revolved about the z axis at this mean radius.
const RING_RADIUS: f64 = 20.0;

/// How many facets the ring has round the axis.
const RING_SEGMENTS: i32 = 256;

/// The thin-floor groove's floor depth below its line: the end faces' bottom crease runs 0.5 under
/// the floor.
const FLOOR: f64 = 0.5;

/// How far each free end of the thin-floor groove stands short of its square end: the picked
/// stretch runs from slice 8 to slice 56 of 64, y = 2 to 14 of 16.
const SHORT: f64 = 2.0;

/// The size asked of the thin-floor groove: above the level line's room.
const THIN_FLOOR_SIZE: f64 = 2.3;

/// The pick built at `size` with the creases as its feature edges, the room `handed` handed in
/// where given, as the model hands the room it read before the build.
fn built(
    (positions, triangles): &(Vec<Vector3>, Triangles),
    chains: &[Vec<i32>],
    closed: bool,
    size: f64,
    handed: Option<f64>,
) -> BandGapResult {
    build_pick(
        positions,
        triangles,
        chains,
        &vec![closed; chains.len()],
        Some(creases(positions, triangles)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            known_room: handed,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick's arguments are good")
}

/// `chain` of `positions` filleted at [`RADIUS`] with a 30 degree sharp turn: the band and the
/// fill.
fn filleted(
    positions: &[Vector3],
    triangles: &Triangles,
    chain: [i32; 2],
) -> (BandGapResult, GapFillResult) {
    let band = build_pick(
        positions,
        triangles,
        &[chain.to_vec()],
        &[false],
        Some(creases(positions, triangles)),
        RADIUS,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            sharp_turn_degrees: 30.0,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick's arguments are good");
    let fill = gap_fill::fill(&band, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS);
    (band, fill.expect("the fill runs"))
}

/// The level line's build of the groove leaning `degrees`: both sides set back by
/// the angle's 2, so its rails end 2 from the floor's line, the straight one at (-sqrt 2, sqrt 2)
/// and the steeper one where its wall, from (1, 1) at 65 degrees, stands 2 from the line; its rows
/// are drawn about the end vertex, so in (x, z) each end's last row is the circle tangent to the
/// lines from the end vertex to the two rail ends, at those ends.
fn the_level_lines_groove_caps_each_leaning_end_face(degrees: f64) {
    let (positions, triangles, slope) = leaning(a_groove_with_one_wall_bending_steeper(), degrees);
    let (band, fill) = filleted(&positions, &triangles, [0, 8]);
    assert_eq!(band.refusal, BandGapRefusal::None);
    assert_eq!(fill.refusal, GapFillRefusal::None, "{:?}", fill.open_edge);

    // The steeper rail end (1 + s cos 65, 1 + s sin 65) at distance 2: s^2 + 2 s (cos + sin) + 2
    // = 4.
    let steep = STEEP_DEGREES.to_radians();
    let sum = steep.cos() + steep.sin();
    let s = -sum + ((sum * sum) + 2.0).sqrt();
    let q = (1.0 + (s * steep.cos()), 1.0 + (s * steep.sin()));
    let p = (-std::f64::consts::SQRT_2, std::f64::consts::SQRT_2);
    // Both legs are 2, so the circle's centre is on their bisector, 2 / cos(half the angle
    // between them) from the end vertex, and its radius 2 tan(half that angle).
    let cosine = ((p.0 * q.0) + (p.1 * q.1)) / 4.0;
    let half = cosine.acos() / 2.0;
    let bisector = (p.0 + q.0, p.1 + q.1);
    let length = bisector.0.hypot(bisector.1);
    let reach = 2.0 / half.cos();
    let centre = (reach * bisector.0 / length, reach * bisector.1 / length);

    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    closed_flat_ended_and_not_crossing(&mesh, slope);
    assert_eq!(
        rows_on_the_end_faces_stand_on(&mesh, &fill, slope, centre, 2.0 * half.tan()),
        [DEFAULT_FILLET_SEGMENTS - 1; 2],
        "the vertices each end's row lays on its end face"
    );
}

/// Leaning one way: the level line caps the groove's leaning end faces over the round.
#[test]
fn the_level_lines_groove_leaning_one_way_caps_each_leaning_end_face_over_the_round() {
    the_level_lines_groove_caps_each_leaning_end_face(8.0);
}

/// Leaning the other way.
#[test]
fn the_level_lines_groove_leaning_the_other_way_caps_each_leaning_end_face_over_the_round() {
    the_level_lines_groove_caps_each_leaning_end_face(-8.0);
}

/// The one-wall-bending groove sliced along its length, picked from y = 2 to y = 14, free at both
/// ends, built at radius 2. Its feature edges are its creases but the floor's line past the pick,
/// so the band's side floods run round each free end and join the two sides into one region. The
/// level line's room, 2 less the margin, is set where the floor's crease runs on past the first
/// free end, 2 from it at the origin, so a size of 2 is refused.
#[test]
fn the_groove_whose_sides_join_round_its_free_ends_is_refused_where_the_floors_crease_runs_on() {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, tz),
        (-tz, tz),
    ];
    let n = profile.len() as i32;
    let ((positions, triangles), chain) = sliced(&profile, (0.0, -2.0), (8, 56));
    let sharp = creases(&positions, &triangles);
    let floor = move |a: i32, b: i32| a % n == 0 && b % n == 0;
    let level_line = build_pick(
        &positions,
        &triangles,
        &[chain],
        &[false],
        Some(Rc::new(move |a, b| sharp(a, b) && !floor(a, b))),
        2.0,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick's arguments are good");
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert_eq!(level_line.room, 1.999999998);
    assert_eq!(level_line.room_set_by.kind, RoomReadingKind::Foreign);
    assert_eq!(level_line.room_set_by.crease_at, Some(Vector3::ZERO));
}

/// The plain 90 degree V-groove, its walls 4 high, on a floor [`FLOOR`] thick, sliced, picked from
/// y = 2 to y = 14, free at both ends.
fn the_groove_on_a_thin_floor() -> ((Vec<Vector3>, Triangles), Vec<i32>) {
    sliced(
        &[
            (0.0, 0.0),
            (4.0, 4.0),
            (6.0, 4.0),
            (6.0, -FLOOR),
            (-6.0, -FLOOR),
            (-6.0, 4.0),
            (-4.0, 4.0),
        ],
        (0.0, -FLOOR / 2.0),
        (8, 56),
    )
}

/// The level line's room by hand. Its round runs one setback (the size, at 90 degrees) past the
/// free end, so it reaches the square end's bottom crease, nearest at (0, 0, -0.5), at the size
/// equal to that crease's distance from the end vertex: sqrt(2^2 + 0.5^2).
fn the_level_lines_room() -> f64 {
    SHORT.hypot(FLOOR)
}

/// The level line's round reaches the crease past the free end and the size is refused, naming
/// that room.
#[test]
fn a_free_end_short_of_a_crease_is_refused_naming_the_distance_to_it() {
    let (part, chain) = the_groove_on_a_thin_floor();
    let level_line = built(
        &part,
        std::slice::from_ref(&chain),
        false,
        THIN_FLOOR_SIZE,
        None,
    );
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert!(
        (level_line.room - the_level_lines_room()).abs() < 1e-6,
        "the level line's room {}, by hand {}",
        level_line.room,
        the_level_lines_room()
    );
}

/// The thin-floor groove handed the level line's room, which refuses the size: the build refuses
/// it, quoting the handed room.
#[test]
fn a_handed_room_that_refuses_the_size_is_quoted() {
    let (part, chain) = the_groove_on_a_thin_floor();
    let handed = the_level_lines_room();
    let level_line = built(&part, &[chain], false, THIN_FLOOR_SIZE, Some(handed));
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert_eq!(level_line.room, handed, "the handed room is quoted");
}

/// How far the ring's steeper wall's top crease stands from the groove's floor in the profile:
/// the wall runs from (0, 0) to (1, 1), then 4 at [`STEEP_DEGREES`].
fn the_steeper_walls_top() -> f64 {
    let steep = STEEP_DEGREES.to_radians();
    (1.0 + (4.0 * steep.cos())).hypot(1.0 + (4.0 * steep.sin()))
}

/// The ring at 3.9: the level line's own room is the steeper wall's top crease, 5.35 from the
/// floor at a setback of the size, which clears the size; handed back in, it is trusted and
/// nothing is refused by size.
#[test]
fn a_handed_room_that_clears_the_size_is_trusted() {
    let size = 3.9;
    let (positions, triangles, chain) = a_ring_with_one_groove_wall_bending_steeper();
    let part = (positions, triangles);
    let unhanded = built(&part, std::slice::from_ref(&chain), false, size, None);
    let handed = unhanded.room;
    assert!(
        (handed - the_steeper_walls_top()).abs() < 0.01 && handed > size,
        "the level line's room {handed}"
    );
    let level_line = built(&part, &[chain], true, size, Some(handed));
    assert_ne!(level_line.refusal, BandGapRefusal::Size);
}

/// The whole ring, a closed chain, at radius 2 handed a room of 1: refused up front on that room.
#[test]
fn a_closed_chain_handed_a_refusing_room_is_refused_on_it() {
    let (positions, triangles, chain) = a_ring_with_one_groove_wall_bending_steeper();
    let part = (positions, triangles);
    let level_line = built(&part, &[chain], true, 2.0, Some(1.0));
    assert_eq!(level_line.refusal, BandGapRefusal::Size);
    assert_eq!(level_line.room, 1.0);
}

/// Radius 2 in the groove whose walls bend steeper: the first wall is sqrt 2 wide, less than the
/// radius the 90 degree corner asks, and the level line's angle formula sets every setback back by
/// the radius, and builds.
#[test]
fn where_the_wall_bends_away_the_band_takes_the_angles_setback() {
    let radius = 2.0;
    let level_line = built(
        &a_groove_whose_walls_bend_steeper(),
        &[vec![0, 9]],
        false,
        radius,
        None,
    );
    assert_eq!(level_line.chains.len(), 1);
    for setback in &level_line.chains[0].setbacks {
        assert!(
            (setback - radius).abs() < 1e-9,
            "the angle formula: {setback}"
        );
    }
    assert_eq!(level_line.refusal, BandGapRefusal::None);
}

/// A plate 10 by 10 and 1 thick, and its top edge along x at y = 5 (corners 7 to 6): the side
/// face below the edge is 1 tall, so a round larger than that has no face to touch on that side.
fn a_plate() -> (Vec<Vector3>, Triangles) {
    a_box(Vector3::new(-5.0, -5.0, 0.0), Vector3::new(5.0, 5.0, 1.0))
}

/// The plate's top edge.
fn the_top_edge() -> Vec<Vec<i32>> {
    vec![vec![7, 6]]
}

/// Far past the plate's thickness the size is refused by its size.
#[test]
fn a_size_past_the_plates_room_is_refused_by_size() {
    let off = built(&a_plate(), &the_top_edge(), false, 3.0, None);
    assert_eq!(off.refusal, BandGapRefusal::Size, "room {}", off.room);
}

/// The same edge at 0.6, within its room, over a sealed cavity just under the edge, builds.
#[test]
fn a_size_within_the_plates_room_over_a_sealed_cavity_builds() {
    let (mut positions, mut triangles) = a_plate();
    let (cavity, cavity_triangles) =
        a_box(Vector3::new(-4.9, 4.7, 0.6), Vector3::new(4.9, 4.85, 0.85));
    let base = positions.len() as i32;
    positions.extend(cavity);
    triangles.extend(
        cavity_triangles
            .iter()
            .map(|&(a, b, c)| (base + a, base + c, base + b)),
    );
    let off = built(&(positions, triangles), &the_top_edge(), false, 0.6, None);
    assert_eq!(
        off.refusal,
        BandGapRefusal::None,
        "within the level line's room"
    );
}

/// Whether the triangles cross themselves, by the boolean operand check.
fn crosses_itself(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> bool {
    let mut mesh = Mesh::new();
    for p in positions {
        mesh.vertices.push(Vector3Float::from_vector3(*p));
    }
    for &(a, b, c) in triangles {
        mesh.faces.add(a, b, c, &mesh.vertices);
    }
    classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false)
        == BooleanOperandVerdict::SelfIntersecting
}

/// `part` leaning `degrees` off square at both ends (each vertex moved along the chain by its x
/// times the lean's tangent), with the lean's slope.
fn leaning(part: (Vec<Vector3>, Triangles), degrees: f64) -> (Vec<Vector3>, Triangles, f64) {
    let slope = degrees.to_radians().tan();
    let (positions, triangles) = part;
    let positions = positions
        .iter()
        .map(|p| Vector3::new(p.x, p.y + (slope * p.x), p.z))
        .collect();
    (positions, triangles, slope)
}

/// The filled `mesh` is closed (every directed edge of a live triangle run once, and once back)
/// and does not cross itself; each end face, leaning by `slope`, stays one flat face (no vertex
/// stands just off it, as a row bowed out of it would).
fn closed_flat_ended_and_not_crossing(mesh: &LevelLineMesh, slope: f64) {
    for v in 0..mesh.vertex_count() {
        let p = mesh.position(v);
        for at in [0.0, LENGTH] {
            let off = (p.y - (at + (slope * p.x))).abs();
            assert!(
                off <= NEAR || off >= 0.01,
                "{p:?} stands {off} off the end face through y = {at}"
            );
        }
    }

    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    let mut live = Vec::new();
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }
        let face = mesh.triangles()[t as usize];
        live.push((face.a, face.b, face.c));
        for (a, b) in [(face.a, face.b), (face.b, face.c), (face.c, face.a)] {
            *directed.entry((a, b)).or_default() += 1;
        }
    }
    for (&(a, b), &count) in &directed {
        assert_eq!(count, 1, "edge {a}-{b} is run {count} times one way");
        assert_eq!(
            directed.get(&(b, a)),
            Some(&1),
            "edge {a}-{b} has no triangle running it back: the mesh is open"
        );
    }
    let all: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();
    assert!(
        !crosses_itself(&all, &live),
        "the filleted part crosses itself"
    );
}

/// Every vertex `fill` added on an end face (leaning by `slope`) stands `radius` from the axis
/// (x, z) = `centre`, read in (x, z): the lean slides points along the chain only, so a row that is
/// the round's section in the end face keeps the square section's (x, z). Answers how many there
/// were at each end.
fn rows_on_the_end_faces_stand_on(
    mesh: &LevelLineMesh,
    fill: &GapFillResult,
    slope: f64,
    centre: (f64, f64),
    radius: f64,
) -> [i32; 2] {
    let mut on_the_section = [0; 2];
    for &v in &fill.added_vertices {
        let p = mesh.position(v);
        for (end, at) in [(0, 0.0), (1, LENGTH)] {
            if (p.y - (at + (slope * p.x))).abs() > NEAR {
                continue;
            }
            on_the_section[end] += 1;
            let from_the_axis = (p.x - centre.0).hypot(p.z - centre.1);
            assert!(
                (from_the_axis - radius).abs() < NEAR,
                "end {end}'s row vertex {p:?} stands {from_the_axis} from the axis, not {radius}"
            );
        }
    }
    on_the_section
}

/// A convex solid's triangles, each wound to face away from the solid's centre.
fn outward(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> Triangles {
    let mut centre = Vector3::ZERO;
    for p in positions {
        centre += *p;
    }
    centre *= 1.0 / positions.len() as f64;
    triangles
        .iter()
        .map(|&(a, b, c)| {
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            let normal = (pb - pa).cross(pc - pa);
            if normal.dot(pa - centre) < 0.0 {
                (a, c, b)
            } else {
                (a, b, c)
            }
        })
        .collect()
}

/// A box from `min` to `max`: bottom corners 0 to 3, top 4 to 7, counterclockwise from (-, -).
fn a_box(min: Vector3, max: Vector3) -> (Vec<Vector3>, Triangles) {
    let positions = vec![
        Vector3::new(min.x, min.y, min.z),
        Vector3::new(max.x, min.y, min.z),
        Vector3::new(max.x, max.y, min.z),
        Vector3::new(min.x, max.y, min.z),
        Vector3::new(min.x, min.y, max.z),
        Vector3::new(max.x, min.y, max.z),
        Vector3::new(max.x, max.y, max.z),
        Vector3::new(min.x, max.y, max.z),
    ];
    let quads = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
    ];
    let mut triangles = Vec::new();
    for q in quads {
        triangles.push((q[0], q[1], q[2]));
        triangles.push((q[0], q[2], q[3]));
    }
    let triangles = outward(&positions, &triangles);
    (positions, triangles)
}

/// The mesh's creases: every edge whose two triangles' normals turn by more than 30 degrees.
fn creases(positions: &[Vector3], triangles: &[(i32, i32, i32)]) -> Rc<dyn Fn(i32, i32) -> bool> {
    let normal = |t: (i32, i32, i32)| {
        let a = positions[t.0 as usize];
        (positions[t.1 as usize] - a)
            .cross(positions[t.2 as usize] - a)
            .get_normal()
    };
    let key = |a: i32, b: i32| if a < b { (a, b) } else { (b, a) };
    let mut on: Vec<((i32, i32), Vector3)> = Vec::new();
    for &t in triangles {
        for (a, b) in [(t.0, t.1), (t.1, t.2), (t.2, t.0)] {
            on.push((key(a, b), normal(t)));
        }
    }
    let mut sharp = HashSet::new();
    for (i, (edge, n)) in on.iter().enumerate() {
        for (other, m) in &on[i + 1..] {
            if edge == other && n.dot(*m) < (30.0_f64).to_radians().cos() {
                sharp.insert(*edge);
            }
        }
    }
    Rc::new(move |a, b| sharp.contains(&key(a, b)))
}

/// The profile, in (x, z), extruded along y from 0 to `length`: each profile point i is vertex i
/// at y = 0 and vertex n + i at y = `length`. Each end cap is a fan from `centre` (vertices 2n
/// and 2n + 1) where given, or else from profile point 0; the profile must be star-shaped from
/// it. Wound so the solid's volume is positive.
fn extruded(
    profile: &[(f64, f64)],
    length: f64,
    centre: Option<(f64, f64)>,
) -> (Vec<Vector3>, Triangles) {
    let n = profile.len() as i32;
    let mut positions: Vec<Vector3> = Vec::new();
    for y in [0.0, length] {
        for &(x, z) in profile {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let mut triangles = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push((i, j, n + j));
        triangles.push((i, n + j, n + i));
    }
    match centre {
        Some((x, z)) => {
            positions.push(Vector3::new(x, 0.0, z));
            positions.push(Vector3::new(x, length, z));
            for i in 0..n {
                let j = (i + 1) % n;
                triangles.push((2 * n, j, i));
                triangles.push((2 * n + 1, n + i, n + j));
            }
        }
        None => {
            for i in 1..n - 1 {
                triangles.push((0, i + 1, i));
                triangles.push((n, n + i, n + i + 1));
            }
        }
    }
    let mut volume = 0.0;
    for &(a, b, c) in &triangles {
        let (pa, pb, pc) = (
            positions[a as usize],
            positions[b as usize],
            positions[c as usize],
        );
        volume += pa.dot(pb.cross(pc));
    }
    if volume < 0.0 {
        triangles = triangles.iter().map(|&(a, b, c)| (a, c, b)).collect();
    }
    (positions, triangles)
}

/// A 90 degree V-groove along y whose walls run 45 degrees for sqrt 2 (to (+-1, 1)), then bend 20
/// degrees steeper (less than the 30 degree crease, so each wall is one side) for 4, then turn out
/// flat to the block's top. The chain is the groove's floor, vertices 0 and 9.
fn a_groove_whose_walls_bend_steeper() -> (Vec<Vector3>, Triangles) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-5.0, -3.0),
        (-5.0, tz),
        (-tx, tz),
        (-1.0, 1.0),
    ];
    extruded(&profile, 6.0, Some((0.0, -2.0)))
}

/// The groove with only its right wall (x > 0) bending steeper past the narrow first face; the
/// left wall runs straight at 45 degrees to the block's top. The chain is the floor, vertices 0
/// and 8.
fn a_groove_with_one_wall_bending_steeper() -> (Vec<Vector3>, Triangles) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, tz),
        (-tz, tz),
    ];
    extruded(&profile, 6.0, Some((0.0, -2.0)))
}

/// The one-wall-bending groove's profile (straight wall at x < 0, steeper wall at x > 0) revolved
/// about the z axis: profile point (x, z) at segment j is vertex j n + i at radius
/// [`RING_RADIUS`] + x, angle 2 pi j / [`RING_SEGMENTS`]. A closed ring, with no caps; the groove's
/// floor is the closed chain of vertices j n.
fn a_ring_with_one_groove_wall_bending_steeper() -> (Vec<Vector3>, Triangles, Vec<i32>) {
    let steep = STEEP_DEGREES.to_radians();
    let (tx, tz) = (1.0 + (4.0 * steep.cos()), 1.0 + (4.0 * steep.sin()));
    let profile = [
        (0.0, 0.0),
        (1.0, 1.0),
        (tx, tz),
        (5.0, tz),
        (5.0, -3.0),
        (-7.0, -3.0),
        (-7.0, tz),
        (-tz, tz),
    ];
    let n = profile.len() as i32;
    let mut positions = Vec::new();
    for j in 0..RING_SEGMENTS {
        let angle = std::f64::consts::TAU * f64::from(j) / f64::from(RING_SEGMENTS);
        for &(x, z) in &profile {
            let rho = RING_RADIUS + x;
            positions.push(Vector3::new(rho * angle.cos(), rho * angle.sin(), z));
        }
    }
    let mut triangles: Triangles = Vec::new();
    for j in 0..RING_SEGMENTS {
        let next = (j + 1) % RING_SEGMENTS;
        for i in 0..n {
            let k = (i + 1) % n;
            let (a, b, c, d) = (j * n + i, j * n + k, next * n + k, next * n + i);
            triangles.push((a, b, c));
            triangles.push((a, c, d));
        }
    }
    let mut volume = 0.0;
    for &(a, b, c) in &triangles {
        let (pa, pb, pc) = (
            positions[a as usize],
            positions[b as usize],
            positions[c as usize],
        );
        volume += pa.dot(pb.cross(pc));
    }
    if volume < 0.0 {
        triangles = triangles.iter().map(|&(a, b, c)| (a, c, b)).collect();
    }
    let chain = (0..RING_SEGMENTS).map(|j| j * n).collect();
    (positions, triangles, chain)
}

/// `profile`, in (x, z), extruded along y from 0 to [`SLICED_LENGTH`] in [`SLICES`] equal slices: profile
/// point i at slice j is vertex j n + i; each end is a fan from `centre` (the last two vertices),
/// the profile star-shaped from it. Wound so the solid's volume is positive. Returns the part and
/// the chain of profile point 0 over the slices `picked`.
fn sliced(
    profile: &[(f64, f64)],
    centre: (f64, f64),
    picked: (i32, i32),
) -> ((Vec<Vector3>, Triangles), Vec<i32>) {
    let n = profile.len() as i32;
    let mut positions = Vec::new();
    for j in 0..=SLICES {
        let y = SLICED_LENGTH * f64::from(j) / f64::from(SLICES);
        for &(x, z) in profile {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let mut triangles: Triangles = Vec::new();
    for j in 0..SLICES {
        for i in 0..n {
            let k = (i + 1) % n;
            let (a, b, c, d) = (j * n + i, j * n + k, (j + 1) * n + k, (j + 1) * n + i);
            triangles.push((a, b, c));
            triangles.push((a, c, d));
        }
    }
    let (first, last) = (positions.len() as i32, positions.len() as i32 + 1);
    positions.push(Vector3::new(centre.0, 0.0, centre.1));
    positions.push(Vector3::new(centre.0, SLICED_LENGTH, centre.1));
    let top = SLICES * n;
    for i in 0..n {
        let k = (i + 1) % n;
        triangles.push((first, k, i));
        triangles.push((last, top + i, top + k));
    }
    let mut volume = 0.0;
    for &(a, b, c) in &triangles {
        let (pa, pb, pc) = (
            positions[a as usize],
            positions[b as usize],
            positions[c as usize],
        );
        volume += pa.dot(pb.cross(pc));
    }
    if volume < 0.0 {
        triangles = triangles.iter().map(|&(a, b, c)| (a, c, b)).collect();
    }
    let chain = (picked.0..=picked.1).map(|j| j * n).collect();
    ((positions, triangles), chain)
}
