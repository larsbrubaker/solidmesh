//! The flat fill that closes a solid across a set of loops: a cap of an extrusion, the end of a
//! partial revolve, a band between two bevel rings, the end of a sweep. Every builder places each
//! loop point once in its mesh and hands the fill the loops with their vertex indices; the fill
//! joins only those vertices, so the cap meets the faces beside it on the same vertices and the
//! solid is closed by construction, with no welding of equal positions afterwards.
//!
//! [`fill_triangles`] is the 2D constrained Delaunay fill on the loops' integer points,
//! [`walks_exactly`] the check that a surface's open boundary is exactly the edges it must close,
//! and [`cap_faces_by_corner`] the two together with a fan for any loop set the fill cannot close
//! (`ring_vertices.rs` calls it with each ring's vertices, a vertex per pass where the loops pass
//! a point more than once).

use std::collections::HashMap;

use clipper2_rust::Path64;
use tess2_rust::{ElementType, TessOption, Tessellator, WindingRule};

/// The faces of a flat cap across `loops` (clipper loops, outlines counter clockwise and holes
/// clockwise), `indices[i][k]` the vertex of point `k` of loop `i` and `corner_vertex(triangle, k)`
/// the vertex of corner `k` of a fill triangle (where the loops pass a point more than once, the
/// vertex of the pass whose wedge the triangle lies in: `ring_vertices.rs`). A cap that walks the
/// loops' edges in loop order (`walk_forward`) closes a solid lying on the side the +z normal
/// points away from (a top, or the start of a sweep); one that walks them backward closes the
/// other side.
///
/// The fill only joins the loops' own points: clipper's loops never cross. Should a fill still
/// not close exactly the loops' edges, or leave a triangle with no area, each loop is fanned from
/// its first point instead, which walks every edge once whatever the loop's shape, so the solid
/// stays closed.
pub fn cap_faces_by_corner(
    loops: &[&Path64],
    indices: &[&[i32]],
    corner_vertex: impl Fn(Triangle, usize) -> Option<i32>,
    walk_forward: bool,
) -> Vec<[i32; 3]> {
    let orient = |[a, b, c]: [i32; 3]| if walk_forward { [a, b, c] } else { [c, b, a] };
    let boundary: Vec<(i32, i32)> = indices
        .iter()
        .flat_map(|l| loop_edges(l))
        .map(|(a, b)| if walk_forward { (a, b) } else { (b, a) })
        .collect();

    let filled = fill_triangles(loops).ok().and_then(|triangles| {
        triangles
            .iter()
            .map(|&triangle| {
                if doubled_area(triangle) == 0 {
                    return None;
                }
                // The tessellator lays triangles out counter clockwise, walking a loop's edges
                // in loop order (outlines run counter clockwise, holes clockwise).
                let [a, b, c] = [0, 1, 2].map(|k| corner_vertex(triangle, k));
                Some(orient([a?, b?, c?]))
            })
            .collect::<Option<Vec<[i32; 3]>>>()
    });
    match filled.filter(|faces| walks_exactly(faces, &boundary)) {
        Some(faces) => faces,
        None => indices
            .iter()
            .flat_map(|l| {
                (1..l.len().saturating_sub(1)).map(move |k| orient([l[0], l[k], l[k + 1]]))
            })
            .filter(|&[a, b, c]| a != b && b != c && c != a)
            .collect(),
    }
}

/// A loop's directed edges, as vertex pairs.
pub fn loop_edges(indices: &[i32]) -> impl Iterator<Item = (i32, i32)> + '_ {
    (0..indices.len()).map(move |i| (indices[i], indices[(i + 1) % indices.len()]))
}

/// Whether `faces` is a surface whose open boundary is exactly `boundary`: each of those directed
/// edges walked once and never back, every other edge walked once each way, and no face with a
/// repeated corner. This is what lets a surface be added beside the ones already built (which
/// walk `boundary` the other way) and leave the solid closed - checked on vertex indices, so it
/// says nothing about where the points are.
pub fn walks_exactly(faces: &[[i32; 3]], boundary: &[(i32, i32)]) -> bool {
    let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
    for &[a, b, c] in faces {
        if a == b || b == c || c == a {
            return false;
        }
        for edge in [(a, b), (b, c), (c, a)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }
    // the surfaces beside this one walk its boundary the other way
    for &(a, b) in boundary {
        *walks.entry((b, a)).or_insert(0) += 1;
    }
    walks
        .iter()
        .all(|(&(a, b), &count)| count == 1 && walks.get(&(b, a)) == Some(&1))
}

/// The tessellator had to add a point that is on none of the loops: two loops it was given
/// cross.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InventedPoint;

/// One triangle of a fill, as the integer positions of its corners, counter clockwise.
pub type Triangle = [(i64, i64); 3];

/// The constrained Delaunay fill of `loops` under the odd rule (outlines and the holes inside
/// them), each corner one of the loops' own points. Delaunay refinement keeps the triangles off
/// the zero-area slivers a plain sweep leaves along straight runs of points.
pub fn fill_triangles(loops: &[&Path64]) -> Result<Vec<Triangle>, InventedPoint> {
    let mut tess = Tessellator::new();
    tess.set_option(TessOption::ConstrainedDelaunayTriangulation, true);
    let mut any = false;
    for path in loops {
        if path.len() >= 3 {
            let coordinates: Vec<f64> =
                path.iter().flat_map(|p| [p.x as f64, p.y as f64]).collect();
            tess.add_contour(2, &coordinates);
            any = true;
        }
    }
    if !any {
        return Ok(Vec::new());
    }
    // the +z normal makes every triangle counter clockwise in 2D
    if !tess.tessellate(
        WindingRule::Odd,
        ElementType::Polygons,
        3,
        2,
        Some([0.0, 0.0, 1.0]),
    ) {
        return Err(InventedPoint);
    }

    let vertices = tess.vertices();
    let original = tess.vertex_indices();
    let mut triangles = Vec::new();
    for triangle in tess.elements().as_chunks::<3>().0 {
        if triangle.contains(&tess2_rust::tess::TESS_UNDEF) {
            continue;
        }
        let mut corners = [(0, 0); 3];
        for (corner, &vertex) in corners.iter_mut().zip(triangle) {
            // A point the tessellator made where two edges cross is on no loop.
            if original[vertex as usize] == tess2_rust::tess::TESS_UNDEF {
                return Err(InventedPoint);
            }
            let i = vertex as usize * 2;
            // The tessellator carries the integer inputs through unchanged (exactly
            // representable in f64), so the cast recovers the loop point.
            *corner = (vertices[i] as i64, vertices[i + 1] as i64);
        }
        triangles.push(corners);
    }
    flip_flat_triangles(&mut triangles);
    Ok(triangles)
}

/// Twice the signed area of a triangle, exactly.
pub fn doubled_area([a, b, c]: Triangle) -> i128 {
    let (abx, aby) = ((b.0 - a.0) as i128, (b.1 - a.1) as i128);
    let (acx, acy) = ((c.0 - a.0) as i128, (c.1 - a.1) as i128);
    abx * acy - aby * acx
}

/// Replaces each flat triangle - three points on one line, which the tessellator can leave where
/// a point sits on the far side of a long straight edge - by flipping its long edge: the flat
/// triangle (m, x, y), m between x and y, and its neighbour (y, x, r) across x -> y become
/// (m, x, r) and (y, m, r). The two cover the same area and walk the same outer edges, so the
/// fill still closes exactly its loops. A flat triangle whose long edge is a loop edge has no
/// neighbour to flip with and stays.
fn flip_flat_triangles(triangles: &mut [Triangle]) {
    let mut budget = triangles.len() * 4;
    let mut index = 0;
    while index < triangles.len() && budget > 0 {
        let triangle = triangles[index];
        if doubled_area(triangle) != 0 {
            index += 1;
            continue;
        }
        // m is the corner between the other two
        let middle = (0..3).find(|&j| {
            let (m, x, y) = (triangle[j], triangle[(j + 1) % 3], triangle[(j + 2) % 3]);
            let along = |p: (i64, i64)| {
                (p.0 - x.0) as i128 * (y.0 - x.0) as i128
                    + (p.1 - x.1) as i128 * (y.1 - x.1) as i128
            };
            along(m) > 0 && along(m) < along(y)
        });
        let Some(j) = middle else {
            index += 1;
            continue;
        };
        let (m, x, y) = (triangle[j], triangle[(j + 1) % 3], triangle[(j + 2) % 3]);
        let neighbour = triangles
            .iter()
            .position(|other| (0..3).any(|k| other[k] == y && other[(k + 1) % 3] == x));
        let Some(n) = neighbour else {
            index += 1;
            continue;
        };
        let other = triangles[n];
        let k = (0..3).find(|&k| other[k] == y).unwrap_or(0);
        let r = other[(k + 2) % 3];
        triangles[index] = [m, x, r];
        triangles[n] = [y, m, r];
        budget -= 1;
        // look at both new triangles again from the start of the earlier one
        index = index.min(n);
    }
}
