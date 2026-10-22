//! The clean-solid checker every path-to-solid operation is held to (Linear Extrude, Revolve,
//! Sweep): a closed solid (each edge walked once in each direction on shared vertices), wound
//! outward, with no face of zero area, and flat caps that cover exactly the area the path fills,
//! measured here from the path itself, independently of any builder: the even-odd area (the path
//! display's fill), integrated over horizontal slabs. Also the paths the checks run on: the traced
//! picture outline fixture and small loop sets built from point lists. Shared by the polygon_mesh
//! area and mattercad-model's paths_curves area (through a `#[path]` module), so not every binary
//! uses every helper.
#![allow(dead_code)]

use std::collections::HashMap;

use agg_path::vertex_source::VertexSource;
use agg_path::VertexStorage;
use polygon_mesh::Mesh;

/// An edge of a loop, from one point to the next.
type Edge = ((f64, f64), (f64, f64));

/// The path's loops as point lists, each closed back to its first point implicitly.
pub fn loops_of(path: &VertexStorage) -> Vec<Vec<(f64, f64)>> {
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
pub fn even_odd_area(path: &VertexStorage) -> f64 {
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
pub fn unpaired_edges(mesh: &Mesh) -> Vec<(i32, i32, u32)> {
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
pub const AREA_FRACTION: f64 = 5e-6;

/// Asserts the solid is clean and that its caps fill `filled_area`, within `tolerance` mm².
pub fn assert_clean_solid(mesh: &Mesh, filled_area: f64, height: f64, tolerance: f64, what: &str) {
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

/// The traced picture outline fixture (`test-data/ControlVertexSources/Image Silhouette
/// Trace.svgd`).
pub fn traced_silhouette() -> VertexStorage {
    let file = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../test-data/ControlVertexSources/Image Silhouette Trace.svgd"
    );
    let text = std::fs::read_to_string(file).expect("the traced path fixture");
    VertexStorage::from_svg_d_string(text.trim())
}

/// A path from point lists, each one closed loop.
pub fn path_of(loops: &[&[(f64, f64)]]) -> VertexStorage {
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
pub fn square_at(x: f64, y: f64, side: f64, reversed: bool) -> Vec<(f64, f64)> {
    let mut points = vec![(x, y), (x + side, y), (x + side, y + side), (x, y + side)];
    if reversed {
        points.reverse();
    }
    points
}

/// Asserts the solid is closed (every edge walked once each way), has no face of zero area and
/// encloses a positive volume, which with the faces all wound alike means they face outward.
pub fn assert_closed_outward(mesh: &Mesh, what: &str) {
    assert!(!mesh.faces.is_empty(), "{what}: no faces");
    let unpaired = unpaired_edges(mesh);
    assert!(
        unpaired.is_empty(),
        "{what}: {} edges are not walked once each way, e.g. {:?}",
        unpaired.len(),
        &unpaired[..unpaired.len().min(8)]
    );
    let degenerate = mesh
        .faces
        .iter()
        .filter(|face| {
            let [a, b, c] = corners(mesh, [face.v0, face.v1, face.v2]);
            let (ab, ac) = (
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
            );
            let normal = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];
            normal.iter().map(|n| n * n).sum::<f64>() == 0.0
        })
        .count();
    assert_eq!(degenerate, 0, "{what}: faces with no area");
    let volume = mesh.get_volume();
    assert!(
        volume > 0.0,
        "{what}: the solid faces inward (volume {volume})"
    );
}

/// The faces lying flat in the plane where coordinate `axis` is `at`, as the area facing the +axis
/// way and the area facing the other way.
pub fn flat_area(mesh: &Mesh, axis: usize, at: f64) -> (f64, f64) {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let (mut facing_up, mut facing_down) = (0.0, 0.0);
    for face in mesh.faces.iter() {
        let points = corners(mesh, [face.v0, face.v1, face.v2]);
        if !points.iter().all(|p| (p[axis] - at).abs() < 1e-5) {
            continue;
        }
        let [a, b, c] = points;
        let doubled = (b[u] - a[u]) * (c[v] - a[v]) - (b[v] - a[v]) * (c[u] - a[u]);
        if doubled > 0.0 {
            facing_up += doubled / 2.0;
        } else {
            facing_down -= doubled / 2.0;
        }
    }
    (facing_up, facing_down)
}

/// The path moved by (dx, dy).
pub fn moved(path: &VertexStorage, dx: f64, dy: f64) -> VertexStorage {
    let mut out = VertexStorage::new();
    for points in loops_of(path) {
        out.move_to(points[0].0 + dx, points[0].1 + dy);
        for &(x, y) in &points[1..] {
            out.line_to(x + dx, y + dy);
        }
        out.close_polygon();
    }
    out
}

/// The small loop sets whose loops touch: each one a way the cleaned loops can meet themselves or
/// each other, with a name for failure messages. All are on the micron grid and lie in x, y >= 0.
pub fn touching_loop_cases() -> Vec<(&'static str, VertexStorage)> {
    let a = square_at(0.0, 0.0, 10.0, false);
    let b = square_at(10.0, 10.0, 10.0, false);
    let outline = square_at(0.0, 0.0, 20.0, false);
    // a hole whose corner sits partway along the outline's left edge
    let touching_hole = [(0.0, 10.0), (10.0, 5.0), (10.0, 15.0)];
    // a hole touching its outline from inside at a corner point of both
    let corner_hole = [(10.0, 0.0), (15.0, 10.0), (5.0, 10.0)];
    // a hole sharing a stretch of its outline's bottom edge, run the other way
    let seam_hole = [(5.0, 0.0), (5.0, 8.0), (15.0, 8.0), (15.0, 0.0)];
    let bowtie = [(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)];
    vec![
        ("squares touching at a corner", path_of(&[&a, &b])),
        (
            "hole touching its outline's edge",
            path_of(&[&outline, &touching_hole]),
        ),
        (
            "hole touching its outline's corner",
            path_of(&[&outline, &corner_hole]),
        ),
        (
            "hole sharing its outline's edge",
            path_of(&[&outline, &seam_hole]),
        ),
        ("bowtie", path_of(&[&bowtie])),
    ]
}
