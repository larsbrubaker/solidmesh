//! `LevelLineMesh::remove_input_needle_strips` on a plate whose top face carries a STRIP of needles -
//! a row of nearly collinear vertices zigzag-tiled the way a CAD tessellator leaves one, each
//! needle's apex standing on the next one's long edge - the shape that made the per-needle repair
//! cascade on Many Holes. The tests share the process-wide meter, so a `threads-required` override
//! in `.config/nextest.toml` keeps them apart.

use std::collections::HashSet;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{band_gap_reading, gap_fill};
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandProfile, GapFillRefusal, LevelLineMesh, LevelLineWork, WorkBudget,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{is_watertight, open_edges, signed_volume, signed_volume_of};

/// The floor the unit tests measure against - the strip's needles are three decades under it.
const FLOOR: f64 = 1e-3;

const FOLD_DEGREES: f64 = 20.0;

/// The work a strip-plate build may spend per input triangle - see the rim-pick test.
const WORK_PER_TRIANGLE: i64 = 1000;

/// The length of the fixture's row of nearly collinear vertices.
const STRIP_LENGTH: f64 = 10.0;

/// The strip is removed in one bounded pass: closed, manifold, one shell, no needle left, no
/// vertex minted, and the part's volume unmoved beyond the floor over the strip's length.
///
/// RED, measured 2026-09-22 before the removal existed: the whole-mesh per-needle repair on this
/// fixture's 15 input needles makes 64 repairs and spends 143 units of work. The removal takes the
/// strip out in one visit: 16 units of work, 40 live triangles become 20. The per-needle numbers
/// are printed, not asserted - pinning the OLD repair's cascade would fail a later improvement.
#[test]
fn a_strip_of_input_needles_is_removed_once_without_a_vertex_or_a_needle() {
    let (positions, indices, _) = strip_plate(10, 1e-6);
    let before = signed_volume_of(&positions, &indices);

    // What the per-needle repair alone does to the strip - the red half, printed so the remark
    // stays honest.
    let mut repaired = from_triangles(&positions, &indices);
    repaired.needle_floor = FLOOR;
    let repair_budget = Rc::new(WorkBudget::new(1_000_000).expect("a positive limit"));
    {
        let _work = LevelLineWork::begin(Some(Rc::clone(&repair_budget)));
        repaired.repair_needles().expect("inside its budget");
    }

    println!(
        "per-needle repair: needles in {}, repaired {}, left alone {}, live needles after {}, \
         triangles {} -> {}, work {}",
        needle_count(&from_triangles(&positions, &indices), FLOOR),
        repaired.needles_repaired,
        repaired.needles_left_alone(),
        needle_count(&repaired, FLOOR),
        indices.len(),
        live(&repaired).len(),
        repair_budget.spent.get()
    );

    let mut mesh = from_triangles(&positions, &indices);
    mesh.needle_floor = FLOOR;
    let needles_in = needle_count(&mesh, FLOOR);
    let budget = Rc::new(WorkBudget::new(1_000_000).expect("a positive limit"));
    {
        let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
        mesh.remove_input_needle_strips()
            .expect("inside its budget");
    }

    let after = signed_volume(&mesh);
    println!(
        "strip removal: needles in {needles_in}, strips removed {}, left {}, live needles after \
         {}, triangles {} -> {}, work {}, volume {before} -> {after}",
        mesh.input_strips_removed,
        mesh.input_strips_left,
        needle_count(&mesh, FLOOR),
        indices.len(),
        live(&mesh).len(),
        budget.spent.get()
    );

    assert!(needles_in >= 8, "the fixture must carry a real strip");
    assert_eq!(mesh.input_strips_removed, 1);
    assert_eq!(mesh.input_strips_left, 0);
    assert_eq!(
        needle_count(&mesh, FLOOR),
        0,
        "no no-height triangle may stand after the strip goes"
    );
    assert_eq!(
        mesh.vertex_count() as usize,
        positions.len(),
        "the removal mints no vertex"
    );
    assert!(is_watertight(&mesh), "every edge on exactly two triangles");
    assert_eq!(shells(&mesh), 1);
    assert!(
        (after - before).abs() <= FLOOR * STRIP_LENGTH,
        "the dropped row lay within the floor of a flat face"
    );
    assert!(
        budget.spent.get() <= 4 * indices.len() as i64,
        "each strip is visited once: the work is bounded by the strip, not by a cascade"
    );
}

/// A level-line rim pick on the strip plate's top face, at a quarter of its room, builds and fills
/// closed with the strip removed.
#[test]
fn a_rim_pick_on_the_strip_plate_builds_closed() {
    let (positions, indices, top_rim) = strip_plate(10, 1e-6);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let room = band_gap_reading::room_of(
        &positions,
        &indices,
        &top_rim,
        true,
        Some(Rc::clone(&feature)),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("the reading");
    let budget = Rc::new(
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit"),
    );
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::clone(&budget)),
        ..BandGapBuildOptions::default()
    };
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &[top_rim],
        &[true],
        Some(feature),
        0.25 * room,
        BandProfile::Chamfer,
        &options,
    )
    .expect("the pick is well formed");

    {
        let mesh = gap.mesh.as_ref().map(|m| m.borrow());
        println!(
            "room {room}, refusal {:?}, strips removed {:?}, left {:?}, needle floor {:?}, work {} \
             for {} triangles",
            gap.refusal,
            mesh.as_ref().map(|m| m.input_strips_removed),
            mesh.as_ref().map(|m| m.input_strips_left),
            mesh.as_ref().map(|m| m.needle_floor),
            budget.spent.get(),
            indices.len()
        );
    }
    assert_eq!(gap.refusal, BandGapRefusal::None);

    // The build call is what is under test here, not the fallback: a refused region would leave
    // the build to the per-needle repair and still build, so the removal itself is required.
    {
        let mesh = gap.mesh.as_ref().expect("the band's mesh").borrow();
        assert!(mesh.input_strips_removed >= 1);
        assert_eq!(mesh.input_strips_left, 0);
        assert_eq!(
            needle_count(&mesh, mesh.needle_floor),
            0,
            "no no-height triangle may stand after the strip goes"
        );
    }

    // Measured 2026-09-22: 12,270 units for 40 input triangles, about 307 a triangle - most of it
    // the room reading and the refinement, not the strip. 1000 a triangle leaves over three times
    // that as headroom.
    assert!(
        budget.spent.get() <= WORK_PER_TRIANGLE * indices.len() as i64,
        "a strip is visited once: the build's work is a small multiple of the part, not a cascade \
         ({} spent)",
        budget.spent.get()
    );

    let fill = gap_fill::fill(&gap, BandProfile::Chamfer, 1).expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None);
    let filled = fill.mesh.as_ref().expect("a filled mesh").borrow();
    assert!(open_edges(&filled).is_empty());
    assert_eq!(shells(&filled), 1);
    assert!(signed_volume(&filled) > 0.0);
}

fn from_triangles(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> LevelLineMesh {
    LevelLineMesh::from_triangles(positions, indices, None).expect("a well-formed mesh")
}

type Plate = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>);

/// A 10 x 10 x 2 plate whose top face is cut in two by a row running from the middle of its left
/// side to the middle of its right side: `interior` vertices, alternately `offset` ABOVE and BELOW
/// the line, zigzag-tiled into needles. The half above the row is fanned from L, the row's own
/// end, so those fan triangles are needles too; the half below is a fan from the bottom-left
/// corner, with height.
fn strip_plate(interior: i32, offset: f64) -> Plate {
    const H: f64 = 2.0;
    let mut positions = vec![
        Vector3::new(0.0, 0.0, H),
        Vector3::new(10.0, 0.0, H),
        Vector3::new(10.0, 5.0, H),
        Vector3::new(10.0, 10.0, H),
        Vector3::new(0.0, 10.0, H),
        Vector3::new(0.0, 5.0, H),
    ];
    const C0: i32 = 0;
    const C1: i32 = 1;
    const R: i32 = 2;
    const C2: i32 = 3;
    const C3: i32 = 4;
    const L: i32 = 5;

    // The row in x order: L, the interior points, R.
    let mut row = vec![L];
    for k in 1..=interior {
        row.push(positions.len() as i32);
        positions.push(Vector3::new(
            10.0 * k as f64 / (interior + 1) as f64,
            5.0 + if k % 2 == 1 { offset } else { -offset },
            H,
        ));
    }

    row.push(R);
    let last = row.len() - 1;
    let upper: Vec<i32> = row
        .iter()
        .enumerate()
        .filter(|&(i, _)| i == 0 || i == last || i % 2 == 1)
        .map(|(_, &v)| v)
        .collect();
    let lower: Vec<i32> = row
        .iter()
        .enumerate()
        .filter(|&(i, _)| i == 0 || i == last || i % 2 == 0)
        .map(|(_, &v)| v)
        .collect();

    let mut top = Vec::new();
    for i in 0..row.len() - 2 {
        top.push((row[i], row[i + 1], row[i + 2]));
    }

    // Above the row, the fan a tessellator leaves when it ear-cuts from the row's own end: every
    // fan triangle over the row stands on L, which is ON the line, so each is a needle too.
    for i in 1..upper.len() - 1 {
        top.push((L, upper[i], upper[i + 1]));
    }

    top.push((L, R, C2));
    top.push((L, C2, C3));
    for i in 0..lower.len() - 1 {
        top.push((lower[i], lower[i + 1], C0));
    }

    top.push((C0, C1, R));

    let at = |positions: &[Vector3], v: i32| positions[v as usize];
    let mut indices = Vec::new();
    for (a, b, c) in top {
        // Wound to face +z, read off the (exact, all at z = h) cross product.
        let z = (at(&positions, b) - at(&positions, a))
            .cross(at(&positions, c) - at(&positions, a))
            .z;
        indices.push(if z > 0.0 { (a, b, c) } else { (a, c, b) });
    }

    // The bottom: the rim's six corners, fanned from R (a fan from a corner would put L on the
    // line from C0 to C3 and make a needle of its own), facing -z.
    let rim = vec![C0, C1, R, C2, C3, L];
    let bottom = positions.len() as i32;
    for &v in &rim {
        let p = at(&positions, v);
        positions.push(Vector3::new(p.x, p.y, 0.0));
    }

    let n = rim.len() as i32;
    for i in 1..n - 1 {
        let a = bottom + 2;
        let b = bottom + ((2 + i) % n);
        let c = bottom + ((3 + i) % n);
        indices.push((a, c, b));
    }

    // A wall quad under each rim edge; the rim runs counter-clockwise seen from +z.
    for i in 0..n {
        let a = rim[i as usize];
        let b = rim[((i + 1) % n) as usize];
        let a_low = bottom + i;
        let b_low = bottom + ((i + 1) % n);
        indices.push((a, a_low, b_low));
        indices.push((a, b_low, b));
    }

    (positions, indices, rim)
}

fn live(mesh: &LevelLineMesh) -> Vec<i32> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect()
}

/// Live triangles no higher than the floor over their longest edge.
fn needle_count(mesh: &LevelLineMesh, floor: f64) -> usize {
    live(mesh)
        .into_iter()
        .filter(|&t| {
            let tri = mesh.triangles()[t as usize];
            let a = mesh.position(tri.a);
            let b = mesh.position(tri.b);
            let c = mesh.position(tri.c);
            let longest = (b - a).length().max((c - b).length().max((a - c).length()));
            longest <= 0.0 || (b - a).cross(c - a).length() / longest <= floor
        })
        .count()
}

/// How many edge-connected shells the live triangles make.
fn shells(mesh: &LevelLineMesh) -> i32 {
    let mut seen = HashSet::new();
    let mut shells = 0;
    for start in live(mesh) {
        if !seen.insert(start) {
            continue;
        }

        shells += 1;
        let mut pending = vec![start];
        while let Some(next_triangle) = pending.pop() {
            let t = mesh.triangles()[next_triangle as usize];
            for (a, b) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
                for &next in mesh.triangles_on(a, b) {
                    if seen.insert(next) {
                        pending.push(next);
                    }
                }
            }
        }
    }

    shells
}
