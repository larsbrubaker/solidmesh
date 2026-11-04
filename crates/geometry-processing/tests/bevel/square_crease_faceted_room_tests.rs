//! A square crease end whose concave crease is a FACETED curve: a quarter round's top edge, convex
//! between the round's first facet and its flat front, ends square on a taller block's end wall,
//! which the round meets in a concave crease that follows the round's facets down. The block's
//! front carries on past the end in the front's plane. The round's section in the end wall meets
//! the crease at a setback from the end vertex, so its foot rides the crease from facet to facet
//! for as long as the crease runs on, and the room the crease leaves is the farthest the crease
//! reaches from the end vertex, not the first chord's length. Facets folding exactly 30 degrees (a
//! 12-gon's) are still one curve, 31 degrees a corner (`end_face_sense::carries_on`).

use geometry_processing::bevel::level_line::square_crease_end_clips;
use vector_math::Vector3;

/// The round's radius.
const RADIUS: f64 = 10.0;

/// A triangle wound so its normal faces along `outward`.
fn faced(positions: &[Vector3], t: (i32, i32, i32), outward: Vector3) -> (i32, i32, i32) {
    let [a, b, c] = [t.0, t.1, t.2].map(|i| positions[i as usize]);
    if (b - a).cross(c - a).dot(outward) >= 0.0 {
        t
    } else {
        (t.0, t.2, t.1)
    }
}

/// The quarter round of `facets` facets along x (axis at y = 0, z = -RADIUS, the solid y <= 0), from x = -5 to the end
/// wall x = 0; the chain is its top edge 1 - 0 on y = 0, z = 0, ending at vertex 0. The block past
/// the wall stands to z = 5 with the same front y = 0 and runs to x = 5.
fn quarter_round_onto_a_wall(facets: usize) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    round_onto_a_wall(facets, std::f64::consts::FRAC_PI_2 / facets as f64)
}

/// [`quarter_round_onto_a_wall`] with each of its `facets` facets turning `step` radians off the
/// last, so the round spans `facets * step`.
fn round_onto_a_wall(facets: usize, step: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    round_onto_a_leaning_wall(facets, step, 0.0)
}

/// [`quarter_round_onto_a_wall`] with the end wall turned `lean` radians about the vertical through
/// the end vertex.
fn quarter_round_onto_a_leaning_wall(
    facets: usize,
    lean: f64,
) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    round_onto_a_leaning_wall(facets, std::f64::consts::FRAC_PI_2 / facets as f64, lean)
}

/// The round of `facets` facets, each turning `step` radians off the last, onto an end wall turned
/// `lean` radians about the vertical through the end vertex: the plane x = -y tan(lean), still
/// meeting the front y = 0 at x = 0, so the round's facets run on under it to the wall.
fn round_onto_a_leaning_wall(
    facets: usize,
    step: f64,
    lean: f64,
) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let on_the_wall = |y: f64| -y * lean.tan();
    let length = 5.0;
    let height = 5.0;
    let mut positions = Vec::new();
    // At x = 0 (index 2k) and x = -length (index 2k + 1), k = 0..=facets; vertex 0 is the end and
    // vertex 1 the chain's other end.
    for k in 0..=facets {
        let angle = step * k as f64;
        let (y, z) = (-RADIUS * angle.sin(), RADIUS * angle.cos() - RADIUS);
        positions.push(Vector3::new(on_the_wall(y), y, z));
        positions.push(Vector3::new(-length, y, z));
    }
    let base = positions.len() as i32;
    // The wall's top over each crease vertex, then the front's and the block's corners.
    for k in 0..=facets {
        let y = positions[2 * k].y;
        positions.push(Vector3::new(on_the_wall(y), y, height));
    }
    let front = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, -RADIUS)); // front + 0: below the end vertex
    positions.push(Vector3::new(-length, 0.0, -RADIUS)); // front + 1
    positions.push(Vector3::new(length, 0.0, -RADIUS)); // front + 2
    positions.push(Vector3::new(length, 0.0, 0.0)); // front + 3

    let mut indices = Vec::new();
    for k in 0..facets as i32 {
        let (a, b, c, d) = (2 * k, 2 * k + 2, 2 * k + 3, 2 * k + 1);
        let mid = (positions[a as usize] + positions[c as usize]) * 0.5;
        let outward = Vector3::new(0.0, mid.y, mid.z + RADIUS);
        indices.push(faced(&positions, (a, b, c), outward));
        indices.push(faced(&positions, (a, c, d), outward));
        // The wall above this facet's crease chord, facing back over the round.
        let wall = Vector3::new(-1.0, -lean.tan(), 0.0);
        let (top_a, top_b) = (base + k, base + k + 1);
        indices.push(faced(&positions, (a, b, top_b), wall));
        indices.push(faced(&positions, (a, top_b, top_a), wall));
    }
    // The flat front, y = 0, under the round and on past the end under the block.
    let out = Vector3::new(0.0, 1.0, 0.0);
    indices.push(faced(&positions, (1, front + 1, front), out));
    indices.push(faced(&positions, (1, front, 0), out));
    indices.push(faced(&positions, (0, front, front + 2), out));
    indices.push(faced(&positions, (0, front + 2, front + 3), out));
    (positions, indices)
}

/// The end clip at the chain's end on the wall.
fn the_wall_end_clip(facets: usize) -> geometry_processing::bevel::level_line::EndRayBound {
    the_end_clip_on(quarter_round_onto_a_wall(facets))
}

/// The end clip at the chain's end on the wall of `solid`.
fn the_end_clip_on(
    solid: (Vec<Vector3>, Vec<(i32, i32, i32)>),
) -> geometry_processing::bevel::level_line::EndRayBound {
    let (positions, indices) = solid;
    let (clip, other) =
        square_crease_end_clips(&positions, &indices, &[1, 0], false, (false, false));
    assert_eq!(clip, None, "the chain's far end is not on the wall");
    other.expect("the end on the wall takes the square crease clip")
}

/// A 40-gon's quarter: each facet folds 9 degrees off the last, one curve.
#[test]
fn a_faceted_crease_leaves_the_room_it_reaches() {
    let facets = 10;
    let clip = the_wall_end_clip(facets);

    // The crease is the quarter circle's chords, each farther from the end vertex than the last; its
    // last vertex, a quarter turn round, stands RADIUS * sqrt(2) from it.
    let reach = RADIUS * std::f64::consts::SQRT_2;
    assert!(
        (clip.crease_room - reach).abs() <= 1e-9 * reach,
        "room {} against the crease's reach {reach} (its first chord is {})",
        clip.crease_room,
        2.0 * RADIUS * (std::f64::consts::FRAC_PI_4 / facets as f64).sin()
    );
}

/// Two facets over the quarter fold 45 degrees apart: no curve carrying on but a corner, where the
/// section's foot leaves the crease, so the room stays the first chord's.
#[test]
fn a_crease_stops_at_a_corner() {
    let clip = the_wall_end_clip(2);
    let chord = 2.0 * RADIUS * std::f64::consts::FRAC_PI_8.sin();
    assert!(
        (clip.crease_room - chord).abs() <= 1e-9 * chord,
        "room {} against the first chord {chord}",
        clip.crease_room
    );
}

/// An end wall leaning 35 degrees off square still holds the round's section, so it stops the
/// round flush whatever its lean: the end takes the clip, on the wall's own plane, and its room is
/// the reach of the crease the wall makes with the round, whose last vertex stands at
/// (R tan 35, -R, -R) from the end vertex.
#[test]
fn a_steeply_leaning_wall_still_clips_its_end() {
    let lean = 35.0_f64.to_radians();
    let (positions, indices) = quarter_round_onto_a_leaning_wall(10, lean);
    let (clip, other) =
        square_crease_end_clips(&positions, &indices, &[1, 0], false, (false, false));
    assert_eq!(clip, None, "the chain's far end is not on the wall");
    let clip = other.expect("the leaning wall's end takes the clip");
    let wall = Vector3::new(1.0, lean.tan(), 0.0).get_normal();
    assert!(
        clip.plane_normal.get_normal().dot(wall) >= 1.0 - 1e-12,
        "clip plane {:?} against the wall {wall:?}",
        clip.plane_normal
    );
    let reach = (2.0 * RADIUS * RADIUS + (RADIUS * lean.tan()).powi(2)).sqrt();
    assert!(
        (clip.crease_room - reach).abs() <= 1e-9 * reach,
        "room {} against the crease's reach {reach}",
        clip.crease_room
    );
}

/// A 12-gon's quarter: three facets, each folding exactly 30 degrees off the last, which
/// `end_face_sense::carries_on` reads as one curve whichever way the coordinates round, so the
/// room follows the whole crease.
#[test]
fn a_twelve_gon_crease_leaves_the_room_it_reaches() {
    let clip = the_wall_end_clip(3);
    let reach = RADIUS * std::f64::consts::SQRT_2;
    assert!(
        (clip.crease_room - reach).abs() <= 1e-9 * reach,
        "room {} against the crease's reach {reach}",
        clip.crease_room
    );
}

/// Two facets folding 31 degrees apart, a degree past the limit: a corner, so the room stays the
/// first chord's.
#[test]
fn a_crease_folding_past_thirty_degrees_stops_at_the_first_chord() {
    let step = 31.0_f64.to_radians();
    let clip = the_end_clip_on(round_onto_a_wall(2, step));
    let chord = 2.0 * RADIUS * (step / 2.0).sin();
    assert!(
        (clip.crease_room - chord).abs() <= 1e-9 * chord,
        "room {} against the first chord {chord}",
        clip.crease_room
    );
}

/// A mesh, its triangles and a two-vertex chain on it.
type SolidAndChain = (Vec<Vector3>, Vec<(i32, i32, i32)>, [i32; 2]);

/// An L section extruded along x from 0 to 10: the solid below z = 10 for y < 5, and below the
/// floor z = 3 + `slope` (y - 5) for y from 5 to 10. The chain is the convex vertical edge where
/// the notch wall y = 5 meets the end cap x = 10, from (10, 5, 10) down to the floor at (10, 5, 3):
/// there the floor is the end face, meeting the notch wall in a concave crease, and the cap
/// carries on BELOW the floor's plane across the convex crease the two make. Answers the mesh and
/// the chain (top, bottom).
fn edge_down_to_a_leaning_floor(slope: f64) -> SolidAndChain {
    let section = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 3.0 + slope * 5.0),
        (5.0, 3.0),
        (5.0, 10.0),
        (0.0, 10.0),
    ];
    let n = section.len() as i32;
    let mut positions = Vec::new();
    for x in [0.0, 10.0] {
        for &(y, z) in &section {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let near = |i: i32| i % n;
    let far = |i: i32| n + i % n;
    let mut indices = Vec::new();
    // The two caps, fanned from the section's first corner, which sees every other.
    for i in 1..n - 1 {
        indices.push(faced(
            &positions,
            (near(0), near(i), near(i + 1)),
            Vector3::new(-1.0, 0.0, 0.0),
        ));
        indices.push(faced(
            &positions,
            (far(0), far(i), far(i + 1)),
            Vector3::new(1.0, 0.0, 0.0),
        ));
    }
    // The sides: the section runs counter-clockwise in (y, z), so each edge faces (dz, -dy).
    for i in 0..n {
        let (a, b) = (section[i as usize], section[((i + 1) % n) as usize]);
        let outward = Vector3::new(0.0, b.1 - a.1, -(b.0 - a.0));
        indices.push(faced(
            &positions,
            (near(i), near(i + 1), far(i + 1)),
            outward,
        ));
        indices.push(faced(&positions, (near(i), far(i + 1), far(i)), outward));
    }
    (positions, indices, [far(4), far(3)])
}

/// A floor leaning 11 degrees off square to the edge: the cap carries on under it across a convex
/// crease (`carries_on_under`), so the end takes the leaning clip on the floor's own plane.
#[test]
fn an_edge_onto_a_leaning_floor_its_side_carries_on_under_takes_the_clip() {
    let slope = 0.2;
    let (positions, indices, chain) = edge_down_to_a_leaning_floor(slope);
    let (top, bottom) =
        square_crease_end_clips(&positions, &indices, &chain, false, (false, false));
    assert_eq!(top, None, "the top end stands on no floor");
    let clip = bottom.expect("the end on the leaning floor takes the clip");
    let floor = Vector3::new(0.0, slope, -1.0).get_normal();
    assert!(
        clip.plane_normal.get_normal().dot(floor).abs() >= 1.0 - 1e-12,
        "clip plane {:?} against the floor {floor:?}",
        clip.plane_normal
    );
}

/// The edge onto the leaning floor, rounded at 1 and read either way along (so each rail meets
/// the floor's clip plane in turn, `end_face_fill::rails_to_the_clip_corner`'s left and right
/// rails): the band builds and fills closed, one shell, flush on the floor.
#[test]
fn an_edge_onto_a_leaning_floor_rounds_closed_read_either_way() {
    use geometry_processing::bevel::level_line::band_gap_entry_points::{
        self, BandGapBuildOptions,
    };
    use geometry_processing::bevel::level_line::{gap_fill, BandProfile, GapFillRefusal};

    let (positions, indices, chain) = edge_down_to_a_leaning_floor(0.2);
    let features = crate::level_line_feature_edges::feature_edges_over(&positions, &indices, 10.0);
    for picked in [vec![chain[0], chain[1]], vec![chain[1], chain[0]]] {
        let gap = band_gap_entry_points::build(
            &positions,
            &indices,
            &picked,
            false,
            Some(std::rc::Rc::clone(&features)),
            1.0,
            BandProfile::Fillet,
            &BandGapBuildOptions::default(),
        )
        .expect("the pick is well formed");
        assert!(
            gap.built(),
            "{picked:?}: band {:?}, room {}",
            gap.refusal,
            gap.room
        );
        let filled = gap_fill::fill(&gap, BandProfile::Fillet, gap_fill::DEFAULT_FILLET_SEGMENTS)
            .expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None, "{picked:?}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = crate::sweep_mesh_measures::kept(&mesh);
        assert!(
            crate::sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "{picked:?}: open"
        );
    }
}
