//! The indexed mesh a bevelled extrusion is built in. Every bevel ring is placed once, as one
//! vertex per distinct point, and every face - the caps, the straight walls and the bands between
//! rings (`band.rs`) - is made of those vertex indices. Faces that meet along a ring meet on the
//! same vertices by construction, so the solid is closed without any welding of equal positions
//! afterwards.

use std::collections::HashMap;
use std::rc::Rc;

use clipper2_rust::{Path64, Paths64};
use tess2_rust::{ElementType, TessOption, Tessellator, WindingRule};
use vector_math::{Vector3, Vector3Float};

use crate::mesh::Mesh;

/// Clipper units per mm, the scale every ring's integer points are in.
const UNITS_PER_MM: f64 = agg_path::data_converters_2d::DEFAULT_SCALING;

/// One ring placed in the mesh: its loops and the vertex of every loop point.
pub(super) struct PlacedRing {
    pub loops: Rc<Paths64>,
    /// `indices[loop][point]` is the vertex of that point.
    pub indices: Vec<Vec<i32>>,
    /// The vertex at each 2D position. Two loops of one ring that touch at a point share it.
    pub by_position: HashMap<(i64, i64), i32>,
}

impl PlacedRing {
    /// The vertex at a 2D position, if the ring passes through it.
    pub fn vertex_at(&self, x: i64, y: i64) -> Option<i32> {
        self.by_position.get(&(x, y)).copied()
    }
}

/// The mesh being built and the rings placed in it.
pub(super) struct RingMesh {
    pub mesh: Mesh,
}

impl RingMesh {
    pub fn new() -> Self {
        Self { mesh: Mesh::new() }
    }

    /// Creates the ring's vertices: each distinct point once, at `height`. The position is
    /// computed in f64 and rounded to the mesh's f32 once.
    pub fn place(&mut self, loops: &Rc<Paths64>, height: f64) -> PlacedRing {
        let mut by_position = HashMap::new();
        let indices = loops
            .iter()
            .map(|path| {
                path.iter()
                    .map(|point| {
                        *by_position.entry((point.x, point.y)).or_insert_with(|| {
                            self.mesh
                                .vertices
                                .push(Vector3Float::from_vector3(Vector3::new(
                                    point.x as f64 / UNITS_PER_MM,
                                    point.y as f64 / UNITS_PER_MM,
                                    height,
                                )));
                            self.mesh.vertices.len() as i32 - 1
                        })
                    })
                    .collect()
            })
            .collect();
        PlacedRing {
            loops: Rc::clone(loops),
            indices,
            by_position,
        }
    }

    /// Adds a triangle unless two of its corners are the same vertex.
    pub fn add(&mut self, a: i32, b: i32, c: i32) {
        if a != b && b != c && c != a {
            self.mesh.faces.add(a, b, c, &self.mesh.vertices);
        }
    }

    /// The straight wall between two placements of the same loops: a quad per edge, walking the
    /// lower ring's edges forward and the upper ring's backward, which is how every band meets
    /// the rings it joins (see [`RingMesh::cap`]). Each quad is split from its lower start corner
    /// to its upper end corner. Either diagonal makes the same flat wall, but a mesh built from
    /// the solid (a boolean, a bevel) follows the triangles it is given, and this is the split
    /// the walls of saved designs were made with.
    pub fn straight_wall(&mut self, lower: &PlacedRing, upper: &PlacedRing) {
        for (lower_loop, upper_loop) in lower.indices.iter().zip(&upper.indices) {
            let count = lower_loop.len();
            for i in 0..count {
                let next = (i + 1) % count;
                self.add(lower_loop[i], lower_loop[next], upper_loop[next]);
                self.add(lower_loop[i], upper_loop[next], upper_loop[i]);
            }
        }
    }

    /// Fills `loops` (some or all of `ring`'s loops, given as loop numbers) flat at the ring's
    /// height. Every surface above a ring walks its edges in loop order and every surface below
    /// walks them backward, so a cap that closes the solid below the ring (`walk_forward`, a top)
    /// walks them forward and one that closes the solid above it (a bottom) walks them backward.
    ///
    /// The tessellator only joins the ring's own points: clipper's loops never cross, and points
    /// where loops touch are one vertex of the ring. Should a fill still not close exactly the
    /// loops' edges, each loop is fanned from its first point instead, which walks every edge once
    /// whatever the loop's shape, so the solid stays closed.
    pub fn cap(&mut self, ring: &PlacedRing, loops: &[usize], walk_forward: bool) {
        let paths: Vec<&Path64> = loops.iter().map(|&i| &ring.loops[i]).collect();
        let boundary: Vec<(i32, i32)> = loops
            .iter()
            .flat_map(|&i| loop_edges(&ring.indices[i]))
            .map(|(a, b)| if walk_forward { (a, b) } else { (b, a) })
            .collect();

        let filled = fill_triangles(&paths).ok().and_then(|triangles| {
            triangles
                .iter()
                .map(|triangle| {
                    let [a, b, c] = triangle.map(|(x, y)| ring.vertex_at(x, y));
                    // The tessellator lays triangles out counter clockwise, walking a loop's
                    // edges in loop order (outlines run counter clockwise, holes clockwise).
                    let (a, b, c) = (a?, b?, c?);
                    Some(if walk_forward { [a, b, c] } else { [c, b, a] })
                })
                .collect::<Option<Vec<[i32; 3]>>>()
        });
        match filled.filter(|faces| walks_exactly(faces, &boundary)) {
            Some(faces) => self.add_all(&faces),
            None => {
                for &i in loops {
                    let indices = &ring.indices[i];
                    for k in 1..indices.len().saturating_sub(1) {
                        let (a, b, c) = (indices[0], indices[k], indices[k + 1]);
                        if walk_forward {
                            self.add(a, b, c);
                        } else {
                            self.add(c, b, a);
                        }
                    }
                }
            }
        }
    }

    pub fn add_all(&mut self, faces: &[[i32; 3]]) {
        for &[a, b, c] in faces {
            self.add(a, b, c);
        }
    }
}

/// A loop's directed edges, as vertex pairs.
pub(super) fn loop_edges(indices: &[i32]) -> impl Iterator<Item = (i32, i32)> + '_ {
    (0..indices.len()).map(move |i| (indices[i], indices[(i + 1) % indices.len()]))
}

/// Whether `faces` is a surface whose open boundary is exactly `boundary`: each of those directed
/// edges walked once and never back, every other edge walked once each way, and no face with a
/// repeated corner. This is what lets a surface be added beside the ones already built (which
/// walk `boundary` the other way) and leave the solid closed - checked on vertex indices, so it
/// says nothing about where the points are.
pub(super) fn walks_exactly(faces: &[[i32; 3]], boundary: &[(i32, i32)]) -> bool {
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
pub(super) struct InventedPoint;

/// One triangle of a fill, as the integer positions of its corners, counter clockwise.
pub(super) type Triangle = [(i64, i64); 3];

/// The constrained Delaunay fill of `loops` under the odd rule (outlines and the holes inside
/// them), each corner one of the loops' own points. Delaunay refinement keeps the triangles off
/// the zero-area slivers a plain sweep leaves along straight runs of points.
pub(super) fn fill_triangles(loops: &[&Path64]) -> Result<Vec<Triangle>, InventedPoint> {
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
fn doubled_area([a, b, c]: Triangle) -> i128 {
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
