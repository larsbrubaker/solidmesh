//! The planar region simplifier: merges connected
//! coplanar triangles into one region each and retriangulates the region from its boundary
//! vertices alone. The marching-cubes surface of a distance field over a mechanical part is mostly
//! flat walls tiled with grid-sized triangles, which this collapses to a few large ones without
//! moving the surface.
//!
//! Every boundary vertex of a region is kept, so each edge a region shares with its neighbour stays
//! exactly as it was: the mesh stays closed with no T-junctions, and non-planar regions are
//! untouched. Only interior vertices go, and each lies within the distance tolerance of its
//! region's plane. The retriangulation is manifold-rust's ear clipper
//! (`polygon::triangulate_idx`, built for retriangulating coplanar faces, holes included, never
//! adding a vertex). A region is left as
//! it was when its boundary pinches (one vertex starts two boundary edges), when the new triangles
//! do not tile the old area exactly, or when a new diagonal duplicates an edge elsewhere in the
//! mesh.
//!
//! The ear clipper does not fail with an error, so there is nothing to catch; a bad result is
//! caught by the triangle-count and area checks after it, which leave the region as it was.
//!
//! Iteration order is fixed wherever it reaches the output, so the same mesh always simplifies
//! the same way: boundary loops are traced in the order their start vertices were first seen,
//! the seeds are ordered by an introsort with a fixed placement of ties, and clashing regions
//! fall back in ascending order. Tests are in `planar_region_simplifier_tests.rs`.

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet, VecDeque};

use manifold_rust::linalg::Vec2;
use manifold_rust::polygon::triangulate_idx;
use manifold_rust::types::{PolyVert, PolygonsIdx};
use solidmesh_math::fixed_tie_sort::introspective_sort;
use solidmesh_math::ieee_math::max_f64;
use solidmesh_math::Vector3;

use crate::Mesh;

/// The default angle tolerance of [`simplify`]: radians a face normal may differ from its
/// region's.
pub const DEFAULT_ANGLE_TOLERANCE: f64 = 1e-4;
/// The default relative distance tolerance of [`simplify`]: how far, as a fraction of the
/// largest bounding box extent, a vertex may sit off its region's plane.
pub const DEFAULT_RELATIVE_DISTANCE_TOLERANCE: f64 = 1e-7;

/// A face not yet in any region.
const NO_REGION: usize = usize::MAX;

/// A directed edge (from, to) by vertex index.
type EdgeKey = (usize, usize);

#[derive(Default)]
struct Region {
    faces: Vec<usize>,
    normal: Vector3,
    offset: f64,
    /// The boundary loops, outer and holes, in the faces' winding.
    loops: Vec<Vec<usize>>,
}

/// The per-face data every step reads.
struct Faces<'a> {
    verts: Vec<usize>,
    positions: &'a [Vector3],
}

impl Faces<'_> {
    fn position(&self, h: usize) -> Vector3 {
        self.positions[self.verts[h]]
    }
}

/// Simplifies `mesh` with the default tolerances.
pub fn simplify(mesh: &Mesh) -> Mesh {
    simplify_with(
        mesh,
        DEFAULT_ANGLE_TOLERANCE,
        DEFAULT_RELATIVE_DISTANCE_TOLERANCE,
    )
}

/// Returns a copy of `mesh` (a welded mesh: faces share vertex indices) with every coplanar region
/// retriangulated, or the mesh's own faces where nothing could be merged. `angle_tolerance` is in
/// radians; `relative_distance_tolerance` is a fraction of the largest bounding box extent.
pub fn simplify_with(mesh: &Mesh, angle_tolerance: f64, relative_distance_tolerance: f64) -> Mesh {
    simplify_counting_work(mesh, angle_tolerance, relative_distance_tolerance).0
}

/// `simplify_with`, also counting its inner-loop steps (faces grown, boundary vertices scanned and
/// triangulated, edges checked) so a test can guard that the cost stays linear in the mesh.
pub(crate) fn simplify_counting_work(
    mesh: &Mesh,
    angle_tolerance: f64,
    relative_distance_tolerance: f64,
) -> (Mesh, i64) {
    let mut work = 0i64;
    let face_count = mesh.faces.len();
    let positions: Vec<Vector3> = mesh
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();

    let bounds = mesh.get_axis_aligned_bounding_box();
    let distance_tolerance = relative_distance_tolerance
        * max_f64(bounds.x_size(), max_f64(bounds.y_size(), bounds.z_size()));
    let cos_tolerance = angle_tolerance.cos();

    let mut verts = vec![0usize; face_count * 3];
    let mut normals = vec![Vector3::ZERO; face_count];
    let mut areas = vec![0.0f64; face_count];
    for (f, face) in mesh.faces.iter().enumerate() {
        verts[3 * f] = face.v0 as usize;
        verts[3 * f + 1] = face.v1 as usize;
        verts[3 * f + 2] = face.v2 as usize;
        let p0 = positions[face.v0 as usize];
        let cross = (positions[face.v1 as usize] - p0).cross(positions[face.v2 as usize] - p0);
        areas[f] = cross.length() / 2.0;
        normals[f] = if areas[f] > 0.0 {
            cross / (2.0 * areas[f])
        } else {
            Vector3::ZERO
        };
    }
    let faces = Faces {
        verts,
        positions: &positions,
    };

    // Directed half-edge -> its index (face * 3 + corner). A repeated directed edge is a
    // non-manifold spot; its faces are kept out of every region.
    let mut half_edges: HashMap<EdgeKey, usize> = HashMap::with_capacity(face_count * 3);
    let mut eligible = vec![true; face_count];
    for h in 0..face_count * 3 {
        let key = (faces.verts[h], faces.verts[next(h)]);
        if let Some(&existing) = half_edges.get(&key) {
            eligible[h / 3] = false;
            eligible[existing / 3] = false;
        } else {
            half_edges.insert(key, h);
        }
    }

    // Seed from the largest faces first: their normals carry the least rounding noise.
    let mut order: Vec<usize> = (0..face_count).collect();
    introspective_sort(&mut order, &mut |&a, &b| {
        nan_first_compare(areas[b], areas[a])
    });

    // Degenerate slivers inside a flat wall have no normal to compare; they join on distance.
    let degenerate_area = distance_tolerance * distance_tolerance;
    let joins = |f: usize, region: &Region| -> bool {
        for corner in 0..3 {
            if (region.normal.dot(faces.position(3 * f + corner)) - region.offset).abs()
                > distance_tolerance
            {
                return false;
            }
        }
        areas[f] <= degenerate_area || normals[f].dot(region.normal) >= cos_tolerance
    };

    let mut region_of = vec![NO_REGION; face_count];
    let mut regions: Vec<Region> = Vec::new();
    let mut stack: Vec<usize> = Vec::new();
    for &seed in &order {
        if region_of[seed] != NO_REGION || !eligible[seed] || areas[seed] <= degenerate_area {
            continue;
        }

        let mut region = Region {
            normal: normals[seed],
            offset: normals[seed].dot(faces.position(3 * seed)),
            ..Region::default()
        };
        let region_index = regions.len();
        region_of[seed] = region_index;
        stack.push(seed);
        while let Some(f) = stack.pop() {
            work += 1;
            region.faces.push(f);
            for corner in 0..3 {
                let h = 3 * f + corner;
                let Some(&opposite) = half_edges.get(&(faces.verts[next(h)], faces.verts[h]))
                else {
                    continue;
                };

                let g = opposite / 3;
                if region_of[g] == NO_REGION && eligible[g] && joins(g, &region) {
                    region_of[g] = region_index;
                    stack.push(g);
                }
            }
        }

        regions.push(region);
    }

    // A region with a pinched boundary can not be traced; it keeps its faces, as do lone faces.
    let mut active = vec![false; regions.len()];
    for (r, region) in regions.iter_mut().enumerate() {
        active[r] = region.faces.len() >= 2
            && trace_boundary(region, r, &faces.verts, &region_of, &half_edges);
    }

    // Retriangulate every active region. One that fails keeps its faces and takes back the
    // vertices it dropped; only the neighbour that dropped them with it is redone, so a failure
    // costs its own region and one neighbour, never a fresh pass. A new diagonal can also join
    // two boundary vertices a neighbour already joins, giving an edge four faces; the regions
    // that made such an edge fall back the same way.
    let mut state = Retriangulation {
        replacements: vec![None; regions.len()],
        dropped: Vec::new(),
        first_region: Vec::new(),
        second_region: Vec::new(),
        queued: vec![false; regions.len()],
        pending: VecDeque::new(),
        active,
    };
    let (dropped, first_region, second_region) = find_dropped_vertices(
        &regions,
        &state.active,
        &region_of,
        &faces,
        distance_tolerance,
        &mut work,
    );
    state.dropped = dropped;
    state.first_region = first_region;
    state.second_region = second_region;
    for r in 0..regions.len() {
        if state.active[r] {
            state.queued[r] = true;
            state.pending.push_back(r);
        }
    }

    loop {
        while let Some(r) = state.pending.pop_front() {
            state.queued[r] = false;
            if state.active[r] {
                state.replacements[r] = retriangulate(
                    &regions[r],
                    &state.dropped,
                    &faces,
                    distance_tolerance,
                    &mut work,
                );
                if state.replacements[r].is_none() {
                    state.fall_back(r, &regions);
                }
            }
        }

        let clashing =
            find_clashing_regions(&faces.verts, &region_of, &state.replacements, &mut work);
        if clashing.is_empty() {
            let result = build_mesh(mesh, &faces.verts, &region_of, &state.replacements);
            return (result, work);
        }

        for r in clashing {
            if state.active[r] {
                state.fall_back(r, &regions);
            }
        }
    }
}

/// The retriangulation pass's mutable state, shared with its fall-back step.
struct Retriangulation {
    active: Vec<bool>,
    replacements: Vec<Option<Vec<[usize; 3]>>>,
    dropped: Vec<bool>,
    first_region: Vec<usize>,
    second_region: Vec<usize>,
    queued: Vec<bool>,
    pending: VecDeque<usize>,
}

impl Retriangulation {
    fn fall_back(&mut self, r: usize, regions: &[Region]) {
        self.active[r] = false;
        self.replacements[r] = None;
        for lp in &regions[r].loops {
            for &vertex in lp {
                if self.dropped[vertex] {
                    self.dropped[vertex] = false;
                    let other = if self.first_region[vertex] == r {
                        self.second_region[vertex]
                    } else {
                        self.first_region[vertex]
                    };
                    if self.active[other] && !self.queued[other] {
                        self.queued[other] = true;
                        self.pending.push_back(other);
                    }
                }
            }
        }
    }
}

fn next(h: usize) -> usize {
    if h % 3 == 2 {
        h - 2
    } else {
        h + 1
    }
}

/// NaN sorts below every number and equal to itself.
fn nan_first_compare(a: f64, b: f64) -> Ordering {
    match a.partial_cmp(&b) {
        Some(ordering) => ordering,
        None => match (a.is_nan(), b.is_nan()) {
            (true, true) => Ordering::Equal,
            (true, false) => Ordering::Less,
            _ => Ordering::Greater,
        },
    }
}

/// Traces the region's boundary loops into `Region::loops`, in the faces' winding; false when one
/// vertex starts two boundary edges (a pinch), which would make it ambiguous.
fn trace_boundary(
    region: &mut Region,
    region_index: usize,
    verts: &[usize],
    region_of: &[usize],
    half_edges: &HashMap<EdgeKey, usize>,
) -> bool {
    // `starts` keeps the order start vertices were first seen, so loops are traced in a fixed
    // order (nothing is removed).
    let mut next_on_boundary: HashMap<usize, usize> = HashMap::new();
    let mut starts: Vec<usize> = Vec::new();
    for &f in &region.faces {
        for corner in 0..3 {
            let h = 3 * f + corner;
            let a = verts[h];
            let b = verts[next(h)];
            if let Some(&opposite) = half_edges.get(&(b, a)) {
                if region_of[opposite / 3] == region_index {
                    continue;
                }
            }

            if next_on_boundary.contains_key(&a) {
                return false;
            }
            next_on_boundary.insert(a, b);
            starts.push(a);
        }
    }

    let mut visited: HashSet<usize> = HashSet::new();
    for &start in &starts {
        if !visited.insert(start) {
            continue;
        }

        let mut lp = Vec::new();
        let mut current = start;
        loop {
            lp.push(current);
            match next_on_boundary.get(&current) {
                Some(&n) => current = n,
                None => return false,
            }
            if !(current != start && visited.insert(current)) {
                break;
            }
        }

        if current != start {
            return false;
        }

        region.loops.push(lp);
    }

    true
}

/// The boundary vertices two merged regions may both leave out: a vertex touched only by the
/// faces of exactly two active regions, on the straight run of boundary they share. A run is
/// dropped only when all of it lies on the segment between the kept vertices at its ends, and
/// both regions see the same run between the same ends, so they drop the same vertices.
fn find_dropped_vertices(
    regions: &[Region],
    active: &[bool],
    region_of: &[usize],
    faces: &Faces<'_>,
    distance_tolerance: f64,
    work: &mut i64,
) -> (Vec<bool>, Vec<usize>, Vec<usize>) {
    let positions = faces.positions;
    let vertex_count = positions.len();
    let mut first = vec![NO_REGION; vertex_count];
    let mut second = vec![NO_REGION; vertex_count];
    let mut keep = vec![false; vertex_count];
    for (h, &vertex) in faces.verts.iter().enumerate() {
        let r = region_of[h / 3];
        if r == NO_REGION || !active[r] {
            keep[vertex] = true;
        } else if first[vertex] == NO_REGION {
            first[vertex] = r;
        } else if first[vertex] != r {
            if second[vertex] == NO_REGION {
                second[vertex] = r;
            } else if second[vertex] != r {
                keep[vertex] = true;
            }
        }
    }

    let mut dropped: Vec<bool> = (0..vertex_count)
        .map(|vertex| !keep[vertex] && second[vertex] != NO_REGION)
        .collect();

    // First each candidate against its own two neighbours, which both regions share.
    for (r, region) in regions.iter().enumerate() {
        if !active[r] {
            continue;
        }

        for lp in &region.loops {
            let n = lp.len();
            *work += n as i64;
            for i in 0..n {
                let vertex = lp[i];
                if dropped[vertex]
                    && distance_to_segment(
                        positions[vertex],
                        positions[lp[(i + n - 1) % n]],
                        positions[lp[(i + 1) % n]],
                    ) > distance_tolerance
                {
                    dropped[vertex] = false;
                }
            }
        }
    }

    // Then each whole run against the segment that will replace it, so a gentle curve can not
    // be flattened a vertex at a time.
    for (r, region) in regions.iter().enumerate() {
        if !active[r] {
            continue;
        }

        for lp in &region.loops {
            let n = lp.len();
            *work += n as i64;
            let Some(anchor) = lp.iter().position(|&vertex| !dropped[vertex]) else {
                for &vertex in lp {
                    dropped[vertex] = false;
                }

                continue;
            };

            let mut step = 0;
            while step < n {
                let start = (anchor + step) % n;
                let mut length = 1;
                while dropped[lp[(start + length) % n]] {
                    length += 1;
                }

                if length > 1 {
                    let a = positions[lp[start]];
                    let b = positions[lp[(start + length) % n]];
                    let mut straight = true;
                    let mut i = 1;
                    while i < length && straight {
                        straight = distance_to_segment(positions[lp[(start + i) % n]], a, b)
                            <= distance_tolerance;
                        i += 1;
                    }

                    if !straight {
                        for i in 1..length {
                            dropped[lp[(start + i) % n]] = false;
                        }
                    }
                }

                step += length;
            }
        }
    }

    (dropped, first, second)
}

fn distance_to_segment(point: Vector3, a: Vector3, b: Vector3) -> f64 {
    let ab = b - a;
    let length_squared = ab.length_squared();
    let t = if length_squared > 0.0 {
        ((point - a).dot(ab) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (point - (a + (ab * t))).length()
}

fn retriangulate(
    region: &Region,
    dropped: &[bool],
    faces: &Faces<'_>,
    distance_tolerance: f64,
    work: &mut i64,
) -> Option<Vec<[usize; 3]>> {
    let positions = faces.positions;
    // Project onto the plane; the faces wind counter-clockwise about the normal, so the
    // outer loop comes out counter-clockwise and the holes clockwise, as the ear clipper wants.
    let normal = region.normal;
    let axis = if normal.x.abs() < 0.9 {
        Vector3::UNIT_X
    } else {
        Vector3::UNIT_Y
    };
    let u = normal.cross(axis).get_normal();
    let v = normal.cross(u);
    let mut loops: PolygonsIdx = Vec::new();
    let mut boundary_count = 0usize;
    for lp in &region.loops {
        *work += lp.len() as i64;
        let mut kept = Vec::with_capacity(lp.len());
        for &vertex in lp {
            if !dropped[vertex] {
                let p = positions[vertex];
                kept.push(PolyVert {
                    pos: Vec2::new(p.dot(u), p.dot(v)),
                    idx: vertex as i32,
                });
            }
        }

        boundary_count += kept.len();
        loops.push(kept);
    }

    // A disk with h holes and V boundary vertices takes V + 2h - 2 triangles.
    let expected = boundary_count as i64 + 2 * (loops.len() as i64 - 1) - 2;
    if expected > region.faces.len() as i64 {
        return None;
    }

    let triangles = triangulate_idx(&loops, distance_tolerance, true);
    if triangles.len() as i64 != expected {
        return None;
    }

    // The new triangles must all face the region's way and cover exactly its old area - with
    // every one positive, an equal total rules out overlaps and gaps.
    let mut old_area = 0.0;
    for &f in &region.faces {
        old_area += projected_area(
            faces.position(3 * f),
            faces.position(3 * f + 1),
            faces.position(3 * f + 2),
            normal,
        );
    }

    let mut new_area = 0.0;
    let mut result = Vec::with_capacity(triangles.len());
    for triangle in &triangles {
        let corners = [
            triangle.x as usize,
            triangle.y as usize,
            triangle.z as usize,
        ];
        let area = projected_area(
            positions[corners[0]],
            positions[corners[1]],
            positions[corners[2]],
            normal,
        );
        // Not `area <= 0.0`: a NaN area must fail too.
        if area.partial_cmp(&0.0) != Some(Ordering::Greater) {
            return None;
        }

        new_area += area;
        result.push(corners);
    }

    if (new_area - old_area).abs()
        > 1e-9 * max_f64(1.0, old_area) + (distance_tolerance * distance_tolerance)
    {
        return None;
    }

    Some(result)
}

fn projected_area(a: Vector3, b: Vector3, c: Vector3, normal: Vector3) -> f64 {
    (b - a).cross(c - a).dot(normal) / 2.0
}

/// The merged regions whose new triangles use an edge that is not used exactly once each way in
/// the result - a diagonal that repeats an edge elsewhere. Returned in ascending order, so the
/// fall-back order is fixed.
fn find_clashing_regions(
    verts: &[usize],
    region_of: &[usize],
    replacements: &[Option<Vec<[usize; 3]>>],
    work: &mut i64,
) -> Vec<usize> {
    // Per directed edge: its use count, and whether a new triangle is one of the users.
    let mut directed: HashMap<EdgeKey, (i32, bool)> = HashMap::with_capacity(verts.len());
    let mut use_edge = |a: usize, b: usize, is_new: bool| {
        let entry = directed.entry((a, b)).or_insert((0, false));
        entry.0 += 1;
        entry.1 = entry.1 || is_new;
    };

    for (f, &r) in region_of.iter().enumerate() {
        if r == NO_REGION || replacements[r].is_none() {
            use_edge(verts[3 * f], verts[3 * f + 1], false);
            use_edge(verts[3 * f + 1], verts[3 * f + 2], false);
            use_edge(verts[3 * f + 2], verts[3 * f], false);
        }
    }

    for triangles in replacements.iter().flatten() {
        for t in triangles {
            use_edge(t[0], t[1], true);
            use_edge(t[1], t[2], true);
            use_edge(t[2], t[0], true);
        }
    }

    *work += directed.len() as i64;
    let mut bad_edges: HashSet<EdgeKey> = HashSet::new();
    for (&(a, b), &(count, is_new)) in &directed {
        let (reverse_count, reverse_new) = directed.get(&(b, a)).copied().unwrap_or((0, false));
        if (is_new || reverse_new) && (count != 1 || reverse_count != 1) {
            bad_edges.insert((a, b));
        }
    }

    let mut clashing = Vec::new();
    if bad_edges.is_empty() {
        return clashing;
    }

    let bad = |a: usize, b: usize| bad_edges.contains(&(a, b)) || bad_edges.contains(&(b, a));
    for (r, replacement) in replacements.iter().enumerate() {
        if let Some(triangles) = replacement {
            if triangles
                .iter()
                .any(|t| bad(t[0], t[1]) || bad(t[1], t[2]) || bad(t[2], t[0]))
            {
                clashing.push(r);
            }
        }
    }

    clashing
}

/// Builds the mesh from the kept vertices only, renumbered in their old order.
fn build_mesh(
    mesh: &Mesh,
    verts: &[usize],
    region_of: &[usize],
    replacements: &[Option<Vec<[usize; 3]>>],
) -> Mesh {
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for (f, &r) in region_of.iter().enumerate() {
        if r == NO_REGION || replacements[r].is_none() {
            triangles.push([verts[3 * f], verts[3 * f + 1], verts[3 * f + 2]]);
        }
    }

    for replacement in replacements.iter().flatten() {
        triangles.extend(replacement.iter().copied());
    }

    let mut new_index = vec![-1i32; mesh.vertices.len()];
    let mut kept_vertices = Vec::new();
    for triangle in &triangles {
        for &vertex in triangle {
            if new_index[vertex] == -1 {
                new_index[vertex] = kept_vertices.len() as i32;
                kept_vertices.push(mesh.vertices[vertex]);
            }
        }
    }

    let mut result = Mesh::new();
    result.vertices = kept_vertices;
    for [a, b, c] in triangles {
        result
            .faces
            .add(new_index[a], new_index[b], new_index[c], &result.vertices);
    }

    result
}

#[cfg(test)]
#[path = "planar_region_simplifier_tests.rs"]
mod tests;
