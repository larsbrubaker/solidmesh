//! Every path extrudes to clean geometry: a closed solid (each edge walked once in each direction
//! on shared vertices), wound outward, with no face of zero area, whose caps lie flat facing out
//! and cover exactly the area the path fills, no more (no overlapping slivers) and no less (no
//! holes in the cap). The fixtures are a traced picture's outline (`test-data/ControlVertexSources/
//! Image Silhouette Trace.svgd`, the path Image to Path saved in `test-data/mcx/Image Silhouette
//! Extrude.mcx`) and small paths that each stress one way a loop set can be awkward to fill.
//!
//! The filled area each solid is held to is measured here from the path itself, independently of
//! the extruder: the even-odd area (the path display's fill), integrated over horizontal slabs.

use std::collections::HashMap;

use agg_path::clipper_library::ClipperError;
use agg_path::vertex_source::VertexSource;
use agg_path::VertexStorage;
use polygon_mesh::processors::vertex_source_to_mesh::extrude;
use polygon_mesh::Mesh;

const HEIGHT: f64 = 3.0;

/// An edge of a loop, from one point to the next.
type Edge = ((f64, f64), (f64, f64));

/// The path's loops as point lists, each closed back to its first point implicitly.
fn loops_of(path: &VertexStorage) -> Vec<Vec<(f64, f64)>> {
    let mut loops = Vec::new();
    let mut current: Vec<(f64, f64)> = Vec::new();
    for vertex in path.vertices() {
        if vertex.is_move_to() && !current.is_empty() {
            loops.push(std::mem::take(&mut current));
        }
        if vertex.is_vertex() {
            current.push((vertex.position.x, vertex.position.y));
        }
    }
    if !current.is_empty() {
        loops.push(current);
    }
    loops
}

/// The length of the horizontal line at `y` the loops fill under the even-odd rule.
fn covered_length(edges: &[Edge], y: f64) -> f64 {
    let mut xs: Vec<f64> = edges
        .iter()
        .filter(|((_, y0), (_, y1))| (*y0 <= y && y < *y1) || (*y1 <= y && y < *y0))
        .map(|((x0, y0), (x1, y1))| x0 + (y - y0) / (y1 - y0) * (x1 - x0))
        .collect();
    xs.sort_by(f64::total_cmp);
    xs.as_chunks::<2>()
        .0
        .iter()
        .map(|[low, high]| high - low)
        .sum()
}

/// The covered length integrated from `y0` to `y1`, split in half while the length is not
/// linear across the slab (two edges cross inside it).
fn slab_area(edges: &[Edge], y0: f64, y1: f64, depth: u32) -> f64 {
    let (low, mid, high) = (
        covered_length(edges, y0 + (y1 - y0) * 1e-9),
        covered_length(edges, (y0 + y1) / 2.0),
        covered_length(edges, y1 - (y1 - y0) * 1e-9),
    );
    if depth == 0 || (mid - (low + high) / 2.0).abs() < 1e-12 {
        return mid * (y1 - y0);
    }
    let middle = (y0 + y1) / 2.0;
    slab_area(edges, y0, middle, depth - 1) + slab_area(edges, middle, y1, depth - 1)
}

/// The area the path fills under the even-odd rule: between each two neighbouring heights at
/// which an edge starts or ends, the covered length is linear in y unless edges cross.
fn even_odd_area(path: &VertexStorage) -> f64 {
    let mut edges = Vec::new();
    for points in loops_of(path) {
        for i in 0..points.len() {
            let (a, b) = (points[i], points[(i + 1) % points.len()]);
            if a.1 != b.1 {
                edges.push((a, b));
            }
        }
    }
    let mut heights: Vec<f64> = edges.iter().flat_map(|(a, b)| [a.1, b.1]).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup();
    let mut area = 0.0;
    for pair in heights.windows(2) {
        // only the edges spanning this slab can cross it
        let spanning: Vec<_> = edges
            .iter()
            .copied()
            .filter(|((_, y0), (_, y1))| y0.min(*y1) <= pair[0] && y0.max(*y1) >= pair[1])
            .collect();
        area += slab_area(&spanning, pair[0], pair[1], 12);
    }
    area
}

/// The directed edges not walked exactly once each way: empty for a closed solid whose faces
/// are all wound the same way round.
fn unpaired_edges(mesh: &Mesh) -> Vec<(i32, i32, u32)> {
    let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
    for face in mesh.faces.iter() {
        for edge in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }
    let mut unpaired: Vec<(i32, i32, u32)> = walks
        .iter()
        .filter(|&(&(a, b), &count)| count != 1 || walks.get(&(b, a)) != Some(&1))
        .map(|(&(a, b), &count)| (a, b, count))
        .collect();
    unpaired.sort_unstable();
    unpaired
}

/// A face's corners in f64.
fn corners(mesh: &Mesh, v: [i32; 3]) -> [[f64; 3]; 3] {
    v.map(|i| {
        let p = mesh.vertices[i as usize];
        [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
    })
}

/// Twice the face's area projected on the xy plane, signed: positive counter clockwise from
/// above (facing up).
fn doubled_xy_area([a, b, c]: [[f64; 3]; 3]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// How far a solid's cap area may stray from the area its path fills, as a fraction of it. The
/// profile's clean-up merges points less than 1.415 clipper units (1.4 microns) apart, which moves
/// a finely traced outline's area by a few millionths of itself.
const AREA_FRACTION: f64 = 5e-6;

/// Asserts the solid is clean and that its caps fill `filled_area`, within `tolerance` mm².
fn assert_clean_solid(mesh: &Mesh, filled_area: f64, height: f64, tolerance: f64, what: &str) {
    assert!(!mesh.faces.is_empty(), "{what}: no faces");
    let unpaired = unpaired_edges(mesh);
    assert!(
        unpaired.is_empty(),
        "{what}: {} edges are not walked once each way, e.g. {:?}",
        unpaired.len(),
        &unpaired[..unpaired.len().min(8)]
    );
    let (mut top_up, mut top_down, mut bottom_down, mut bottom_up) = (0.0, 0.0, 0.0, 0.0);
    let mut degenerate = 0;
    for face in mesh.faces.iter() {
        let points = corners(mesh, [face.v0, face.v1, face.v2]);
        let [a, b, c] = points;
        let (ab, ac) = (
            [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
            [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
        );
        let normal = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        if normal.iter().map(|n| n * n).sum::<f64>() == 0.0 {
            degenerate += 1;
        }
        let flat_at = |z: f64| points.iter().all(|p| (p[2] - z).abs() < 1e-6);
        let doubled = doubled_xy_area(points);
        if flat_at(height) {
            if doubled > 0.0 {
                top_up += doubled / 2.0;
            } else {
                top_down -= doubled / 2.0;
            }
        } else if flat_at(0.0) {
            if doubled < 0.0 {
                bottom_down -= doubled / 2.0;
            } else {
                bottom_up += doubled / 2.0;
            }
        }
    }
    assert_eq!(degenerate, 0, "{what}: faces with no area");
    assert!(
        top_down <= tolerance && bottom_up <= tolerance,
        "{what}: cap faces facing into the solid (top {top_down} mm², bottom {bottom_up} mm²)"
    );
    assert!(
        (top_up - filled_area).abs() <= tolerance && (bottom_down - filled_area).abs() <= tolerance,
        "{what}: the caps cover top {top_up} mm² and bottom {bottom_down} mm², the path fills \
         {filled_area} mm²"
    );
    let volume = mesh.get_volume();
    assert!(
        (volume - filled_area * height).abs() <= tolerance * height,
        "{what}: volume {volume}, expected {}",
        filled_area * height
    );
}

/// Extrudes the path and holds it to the rule, against the path's own even-odd area, within
/// `tolerance` mm² beyond what the clean-up's point merging moves (see [`AREA_FRACTION`]). Every
/// loop of the path is also on the top of the solid by at least one of its own points, so no
/// part of the shape was dropped, however small.
fn assert_extrudes_within(
    path: &VertexStorage,
    tolerance: f64,
    what: &str,
) -> Result<(), ClipperError> {
    let mesh = extrude(path, HEIGHT)?;
    let filled_area = even_odd_area(path);
    let allowed = filled_area * AREA_FRACTION + tolerance;
    assert_clean_solid(&mesh, filled_area, HEIGHT, allowed, what);
    let on_top: Vec<(f64, f64)> = mesh
        .vertices
        .iter()
        .filter(|v| (f64::from(v.z) - HEIGHT).abs() < 1e-6)
        .map(|v| (f64::from(v.x), f64::from(v.y)))
        .collect();
    for (index, points) in loops_of(path).iter().enumerate() {
        let kept = points.iter().any(|&(x, y)| {
            on_top
                .iter()
                .any(|&(vx, vy)| (vx - x).abs() < 1e-4 && (vy - y).abs() < 1e-4)
        });
        assert!(kept, "{what}: loop {index} of the path is not on the solid");
    }
    Ok(())
}

/// [`assert_extrudes_within`] for a path whose points are all on clipper's 1 micron grid, so
/// nothing but the clean-up moves the area.
fn assert_extrudes_cleanly(path: &VertexStorage, what: &str) -> Result<(), ClipperError> {
    assert_extrudes_within(path, 1e-9, what)
}

fn traced_silhouette() -> VertexStorage {
    let file = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test-data/ControlVertexSources/Image Silhouette Trace.svgd"
    );
    let text = std::fs::read_to_string(file).expect("the traced path fixture");
    VertexStorage::from_svg_d_string(text.trim())
}

#[test]
fn a_traced_picture_outline_extrudes_to_a_clean_solid() -> Result<(), ClipperError> {
    assert_extrudes_cleanly(&traced_silhouette(), "traced silhouette")
}

/// A path from point lists, each one closed loop.
fn path_of(loops: &[&[(f64, f64)]]) -> VertexStorage {
    let mut path = VertexStorage::new();
    for points in loops {
        path.move_to(points[0].0, points[0].1);
        for &(x, y) in &points[1..] {
            path.line_to(x, y);
        }
        path.close_polygon();
    }
    path
}

/// A square of `side` at (x, y), counter clockwise, or clockwise when `reversed`.
fn square_at(x: f64, y: f64, side: f64, reversed: bool) -> Vec<(f64, f64)> {
    let mut points = vec![(x, y), (x + side, y), (x + side, y + side), (x, y + side)];
    if reversed {
        points.reverse();
    }
    points
}

#[test]
fn deeply_nested_loops_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    // five squares inside each other, alternately wound: solid, hole, solid, hole, solid
    let squares: Vec<Vec<(f64, f64)>> = (0..5)
        .map(|i| {
            let inset = f64::from(i) * 2.0;
            square_at(inset, inset, 20.0 - 2.0 * inset, i % 2 == 1)
        })
        .collect();
    let loops: Vec<&[(f64, f64)]> = squares.iter().map(Vec::as_slice).collect();
    assert_extrudes_cleanly(&path_of(&loops), "nested squares")
}

#[test]
fn loops_wound_either_way_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    // the same nesting with every loop wound the same way: the even-odd fill still alternates
    let squares: Vec<Vec<(f64, f64)>> = (0..4)
        .map(|i| {
            let inset = f64::from(i) * 2.0;
            square_at(inset, inset, 20.0 - 2.0 * inset, false)
        })
        .collect();
    let loops: Vec<&[(f64, f64)]> = squares.iter().map(Vec::as_slice).collect();
    assert_extrudes_cleanly(&path_of(&loops), "same-wound nested squares")?;
    let reversed: Vec<Vec<(f64, f64)>> = (0..4)
        .map(|i| {
            let inset = f64::from(i) * 2.0;
            square_at(inset, inset, 20.0 - 2.0 * inset, true)
        })
        .collect();
    let loops: Vec<&[(f64, f64)]> = reversed.iter().map(Vec::as_slice).collect();
    assert_extrudes_cleanly(&path_of(&loops), "clockwise nested squares")
}

#[test]
fn loops_touching_at_a_point_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    let a = square_at(0.0, 0.0, 10.0, false);
    let b = square_at(10.0, 10.0, 10.0, false);
    assert_extrudes_cleanly(&path_of(&[&a, &b]), "squares touching at a corner")?;
    // a hole touching its outline at a point
    let outline = square_at(0.0, 0.0, 20.0, false);
    let hole = [(0.0, 10.0), (10.0, 5.0), (10.0, 15.0)];
    assert_extrudes_cleanly(&path_of(&[&outline, &hole]), "hole touching its outline")
}

#[test]
fn a_self_crossing_loop_extrudes_to_a_clean_solid() -> Result<(), ClipperError> {
    let bowtie = [(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)];
    let path = path_of(&[&bowtie]);
    // two triangles of base 10 and height 5
    assert!((even_odd_area(&path) - 50.0).abs() < 1e-9);
    assert_extrudes_cleanly(&path, "bowtie")?;
    // A star drawn in one stroke, its points on the micron grid: under even-odd its centre is
    // empty. Each of the five crossings lands on the grid too, a fraction of a micron from where
    // the lines meet, which moves the area by up to about a micron times the edges beside it.
    let star: Vec<(f64, f64)> = (0..5)
        .map(|i| {
            let angle = std::f64::consts::TAU * f64::from(i * 2) / 5.0;
            let on_grid = |v: f64| (v * 1000.0).round_ties_even() / 1000.0;
            (on_grid(10.0 * angle.cos()), on_grid(10.0 * angle.sin()))
        })
        .collect();
    assert_extrudes_within(&path_of(&[&star]), 0.01, "pentagram")
}

#[test]
fn duplicate_consecutive_points_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    let points = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
        (10.0, 10.0),
        (10.0, 10.0),
        (0.0, 10.0),
        (0.0, 0.0),
    ];
    assert_extrudes_cleanly(&path_of(&[&points]), "repeated points")
}

#[test]
fn a_loop_too_small_to_hold_drops_out_cleanly() -> Result<(), ClipperError> {
    let outline = square_at(0.0, 0.0, 10.0, false);
    // smaller than a clipper unit: it rounds to nothing
    let speck = square_at(20.0, 20.0, 0.0002, false);
    let mesh = extrude(&path_of(&[&outline, &speck]), HEIGHT)?;
    assert_clean_solid(&mesh, 100.0, HEIGHT, 1e-9, "outline with a speck");
    // A sliver one unit tall inside a hole of the outline: its tip is within the clean-up's
    // 1.415 units of its base, so the sliver goes and the hole is left empty.
    let hole = square_at(2.0, 2.0, 6.0, true);
    let sliver = [(4.0, 4.0), (6.0, 4.0), (6.0, 4.001)];
    let mesh = extrude(&path_of(&[&outline, &hole, &sliver]), HEIGHT)?;
    assert_clean_solid(&mesh, 64.0, HEIGHT, 1e-9, "sliver in a hole");
    // the same sliver two units tall is kept
    let sliver = [(4.0, 4.0), (6.0, 4.0), (6.0, 4.002)];
    assert_extrudes_cleanly(
        &path_of(&[&outline, &hole, &sliver]),
        "thin triangle in a hole",
    )
}
