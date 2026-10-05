//! Split chords: a vertex a boolean left standing part way along a straight crease, reading as a
//! corner of nothing (`Bevel/BevelSplitChords.cs`, a C# `internal static class`; a module of free
//! functions here). What is here is the READING of one such vertex - whether it would be removed
//! ([`would_remove`]) and the re-cut of the two fans either side of it - with the mesh's own lattice
//! ([`lattice_for`]) and the straight-line test the split blend shares ([`stands_on_the_line_between`]).
//! The repair pass that takes every such vertex out lands with a later package.
//!
//! The faces-on-each-edge table is only looked up, so a `HashMap`'s order cannot reach an output; the
//! vertex's link (`next_round`) is walked for its creases in insertion order, so it is an [`IndexMap`].

use std::collections::HashMap;

use indexmap::IndexMap;
use polygon_mesh::{Face, Mesh};
use vector_math::{cs_math, Vector3};

/// C# `FloatStorageEpsilon`: the step between neighbouring floats at magnitude one, which is how a
/// mesh stores a point.
const FLOAT_STORAGE_EPSILON: f64 = 1.1920929e-7;

/// C# `LatticeFactor`: how many of those steps still count as one straight line. Eight rather than
/// one because a vertex a boolean wrote has been through several roundings on its way here, and
/// rather than thirty-two because the smallest turn these rims really make is thousands of steps wide.
const LATTICE_FACTOR: f64 = 8.0;

/// The faces on each edge, by the edge's two vertices (low first).
pub type FacesOnEachEdge = HashMap<(i32, i32), Vec<i32>>;

/// C# `LatticeFor`: the mesh's own lattice - [`LATTICE_FACTOR`] float steps at its furthest
/// coordinate - or zero for a mesh with no extent (or an infinite one).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn lattice_for(mesh: &Mesh) -> f64 {
    let bounds = mesh.get_axis_aligned_bounding_box();

    let furthest = cs_math::max_f64(
        cs_math::max_f64(bounds.min_xyz.x.abs(), bounds.max_xyz.x.abs()),
        cs_math::max_f64(
            cs_math::max_f64(bounds.min_xyz.y.abs(), bounds.max_xyz.y.abs()),
            cs_math::max_f64(bounds.min_xyz.z.abs(), bounds.max_xyz.z.abs()),
        ),
    );

    if !(furthest > 0.0) || furthest.is_infinite() {
        return 0.0;
    }

    furthest * FLOAT_STORAGE_EPSILON * LATTICE_FACTOR
}

/// C# `WouldRemove`: whether `vertex` reads as a split chord the repair would take out - `Ok(())`
/// where it does, `Err(decline)` with the reason (C#'s `out string decline`) where it does not.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn would_remove(mesh: &Mesh, vertex: i32) -> Result<(), String> {
    let lattice = lattice_for(mesh);

    if !(lattice > 0.0) {
        return Err("no lattice".to_string());
    }

    let at_each_vertex = faces_at_each_vertex(mesh);
    let fan = at_each_vertex[vertex as usize].as_deref().unwrap_or(&[]);

    if fan.len() < 2 {
        return Err(format!("fan of {} faces", fan.len()));
    }

    try_read_split_chord(mesh, vertex, fan, &faces_on_each_edge(mesh), lattice).map(|_| ())
}

/// C# `TryReadSplitChord`: the vertex's fan read as two flat sides of one straight crease through it,
/// re-cut without it - `Ok(replacement)` with the new faces, `Err(decline)` with why not.
fn try_read_split_chord(
    mesh: &Mesh,
    vertex: i32,
    fan: &[i32],
    faces_on_edge: &FacesOnEachEdge,
    lattice: f64,
) -> Result<Vec<(i32, i32, i32)>, String> {
    // The LINK of the vertex: for each face round it, the edge across from it, in the direction that
    // face winds. A fan that is one closed ring has each of those edges leaving exactly one vertex.
    let mut next_round: IndexMap<i32, i32> = IndexMap::with_capacity(fan.len());

    for &face_index in fan {
        match try_read_across(&mesh.faces[face_index as usize], vertex) {
            Some((from, to)) if from != to && !next_round.contains_key(&from) => {
                next_round.insert(from, to);
            }
            _ => {
                return Err(format!(
                    "link: face {face_index} of the fan of {} does not leave one edge per corner",
                    fan.len()
                ));
            }
        }
    }

    let mut creases = Vec::new();

    for &from in next_round.keys() {
        let (one_surface, readable) =
            are_one_surface_across(mesh, vertex, from, faces_on_edge, lattice);

        if !one_surface {
            if !readable {
                let key = if vertex < from {
                    (vertex, from)
                } else {
                    (from, vertex)
                };
                let on_it = faces_on_edge.get(&key).map_or(0, Vec::len);

                return Err(format!("unreadable edge to {from}, {on_it} faces on it"));
            }

            creases.push(from);
        }
    }

    if creases.len() != 2 {
        return Err(format!(
            "creases: {} of the {} edges at the vertex are creases",
            creases.len(),
            next_round.len()
        ));
    }

    let a = creases[0];
    let b = creases[1];

    if !stands_on_the_line_between_vertices(mesh, vertex, a, b, lattice) {
        let at = position(mesh, vertex);
        let one_end = position(mesh, a);
        let other = position(mesh, b);
        let span = other - one_end;

        return Err(format!(
            "the line: off it by {} against a lattice of {}, ends dot {}",
            to_nine_places(span.cross(at - one_end).length() / span.length()),
            to_nine_places(lattice),
            to_nine_places((at - one_end).dot(at - other))
        ));
    }

    let first = the_way_round(&next_round, a, b);
    let second = the_way_round(&next_round, b, a);

    let (first, second) = match (first, second) {
        (Some(first), Some(second)) if first.len() + second.len() == fan.len() + 2 => {
            (first, second)
        }
        (first, second) => {
            // Not one closed ring of faces round the vertex, so there is no pair of fans to rebuild.
            let count = |side: &Option<Vec<i32>>| {
                side.as_ref()
                    .map_or("none".to_string(), |side| side.len().to_string())
            };

            return Err(format!(
                "the ring: sides of {} and {} over a fan of {}",
                count(&first),
                count(&second),
                fan.len()
            ));
        }
    };

    let (first_is_flat, first_out_of_plane) = is_flat(mesh, vertex, &first, lattice);
    let (second_is_flat, second_out_of_plane) = is_flat(mesh, vertex, &second, lattice);

    if !first_is_flat || !second_is_flat {
        return Err(format!(
            "flatness: sides out of plane by {} and {} against a lattice of {}",
            to_nine_places(first_out_of_plane),
            to_nine_places(second_out_of_plane),
            to_nine_places(lattice)
        ));
    }

    let mut replacement = Vec::new();

    if !try_recut_star(mesh, &first, lattice, &mut replacement)
        || !try_recut_star(mesh, &second, lattice, &mut replacement)
    {
        // The star cannot be re-cut without laying a triangle with no height or one outside the
        // outline the file drew, so the vertex stays where the file put it.
        return Err(
            "the star of the vertex has no re-cut free of degenerate or flipped triangles"
                .to_string(),
        );
    }

    Ok(replacement)
}

/// C# `TryRecutStar`: one side's outline (the path round the vertex from one crease end to the other)
/// re-cut into triangles by clipping its tallest ear each time, appended to `into` - false where some
/// remainder has no ear with height above the lattice that lies the way the outline does.
fn try_recut_star(
    mesh: &Mesh,
    path: &[i32],
    lattice: f64,
    into: &mut Vec<(i32, i32, i32)>,
) -> bool {
    if path.len() < 3 {
        // One face, whose two corners other than the vertex are an edge and no triangle at all.
        return true;
    }

    let normal = outline_normal(mesh, path);

    if normal == Vector3::ZERO {
        return false;
    }

    let mut corners = path.to_vec();

    while corners.len() > 3 {
        let mut best_ear = -1;
        let mut best_altitude = 0.0;
        let count = corners.len();

        for i in 0..count {
            let before = corners[(i + count - 1) % count];
            let at = corners[i];
            let after = corners[(i + 1) % count];

            let Some(altitude) = is_an_ear(mesh, &corners, before, at, after, normal, lattice)
            else {
                continue;
            };

            if altitude > best_altitude {
                best_altitude = altitude;
                best_ear = i as i32;
            }
        }

        if best_ear < 0 {
            return false;
        }

        let best = best_ear as usize;
        into.push((
            corners[(best + count - 1) % count],
            corners[best],
            corners[(best + 1) % count],
        ));

        corners.remove(best);
    }

    if is_an_ear(
        mesh, &corners, corners[0], corners[1], corners[2], normal, lattice,
    )
    .is_none()
    {
        return false;
    }

    into.push((corners[0], corners[1], corners[2]));

    true
}

/// C# `IsAnEar`: the corner `at` between `before` and `after` as an ear of the outline - its height
/// (twice its area over its longest side) where it turns the way the outline does, stands taller than
/// the lattice, and holds no other corner of the outline.
#[allow(clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
fn is_an_ear(
    mesh: &Mesh,
    corners: &[i32],
    before: i32,
    at: i32,
    after: i32,
    normal: Vector3,
    lattice: f64,
) -> Option<f64> {
    let a = position(mesh, before);
    let b = position(mesh, at);
    let c = position(mesh, after);

    let cross = (b - a).cross(c - a);
    let twice_area = cross.dot(normal);

    if twice_area <= 0.0 {
        // Reflex or collinear: cutting here would lay a triangle outside the side, or none at all.
        return None;
    }

    let longest = cs_math::max_f64(
        (b - a).length(),
        cs_math::max_f64((c - b).length(), (a - c).length()),
    );

    if !(longest > 0.0) || twice_area / longest <= lattice {
        return None;
    }

    for &corner in corners {
        if corner == before || corner == at || corner == after {
            continue;
        }

        if is_inside(position(mesh, corner), a, b, c, normal) {
            return None;
        }
    }

    Some(twice_area / longest)
}

/// C# `IsInside`: whether a point stands inside the triangle or on its rim, read about `normal`.
fn is_inside(point: Vector3, a: Vector3, b: Vector3, c: Vector3, normal: Vector3) -> bool {
    (b - a).cross(point - a).dot(normal) >= 0.0
        && (c - b).cross(point - b).dot(normal) >= 0.0
        && (a - c).cross(point - c).dot(normal) >= 0.0
}

/// C# `OutlineNormal`: the unit Newell normal of an outline, or zero for one with no area.
fn outline_normal(mesh: &Mesh, path: &[i32]) -> Vector3 {
    let mut sum = Vector3::ZERO;

    for i in 0..path.len() {
        let at = position(mesh, path[i]);
        let next = position(mesh, path[(i + 1) % path.len()]);

        sum += at.cross(next);
    }

    if sum.length() > 0.0 {
        sum / sum.length()
    } else {
        Vector3::ZERO
    }
}

/// C# `TryReadAcross`: the edge of `face` across from `vertex`, in the direction the face winds.
fn try_read_across(face: &Face, vertex: i32) -> Option<(i32, i32)> {
    if face.v0 == vertex {
        Some((face.v1, face.v2))
    } else if face.v1 == vertex {
        Some((face.v2, face.v0))
    } else if face.v2 == vertex {
        Some((face.v0, face.v1))
    } else {
        None
    }
}

/// C# `AreOneSurfaceAcross`: whether the edge from `vertex` to `other` divides one flat surface -
/// both its faces within the lattice of one plane - as `(one_surface, readable)`; an edge with other
/// than two faces, or a first face with no plane, is not readable.
fn are_one_surface_across(
    mesh: &Mesh,
    vertex: i32,
    other: i32,
    faces_on_edge: &FacesOnEachEdge,
    lattice: f64,
) -> (bool, bool) {
    let key = if vertex < other {
        (vertex, other)
    } else {
        (other, vertex)
    };
    let Some(faces) = faces_on_edge.get(&key) else {
        return (false, false);
    };

    if faces.len() != 2 {
        return (false, false);
    }

    let (normal, through) = plane_of(mesh, &mesh.faces[faces[0] as usize]);

    if normal == Vector3::ZERO {
        return (false, false);
    }

    let across = &mesh.faces[faces[1] as usize];
    let off = |v: i32| (position(mesh, v) - through).dot(normal).abs() <= lattice;

    (off(across.v0) && off(across.v1) && off(across.v2), true)
}

/// C# `StandsOnTheLineBetween(Mesh, ..)`: [`stands_on_the_line_between`] for three mesh vertices.
fn stands_on_the_line_between_vertices(
    mesh: &Mesh,
    vertex: i32,
    a: i32,
    b: i32,
    lattice: f64,
) -> bool {
    stands_on_the_line_between(
        position(mesh, vertex),
        position(mesh, a),
        position(mesh, b),
        lattice,
    )
}

/// C# `StandsOnTheLineBetween`: whether `point` stands on the straight line from `from` to `to`,
/// within the lattice, and BETWEEN them - a chord a boolean split carries its extra vertex INSIDE the
/// chord, and one standing past an end would make the merged edge longer than either piece.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn stands_on_the_line_between(
    point: Vector3,
    from: Vector3,
    to: Vector3,
    lattice: f64,
) -> bool {
    let along = to - from;
    let length = along.length();

    if !(length > 0.0) {
        return false;
    }

    along.cross(point - from).length() / length <= lattice && (point - from).dot(point - to) < 0.0
}

/// C# `TheWayRound`: the link walked from `from` to `to`, both ends included, or `None` where it does
/// not get there.
fn the_way_round(next_round: &IndexMap<i32, i32>, from: i32, to: i32) -> Option<Vec<i32>> {
    let mut path = vec![from];
    let mut at = from;

    for _ in 0..next_round.len() {
        let next = *next_round.get(&at)?;

        path.push(next);

        if next == to {
            return Some(path);
        }

        at = next;
    }

    None
}

/// C# `IsFlat`: whether one side of the fan (its path round the vertex) lies within the lattice of
/// one plane through the vertex, as `(flat, worst_out_of_plane)`; a side of one face is flat.
fn is_flat(mesh: &Mesh, vertex: i32, path: &[i32], lattice: f64) -> (bool, f64) {
    if path.len() < 3 {
        // One face, which is flat by itself and has nothing to be compared with.
        return (true, 0.0);
    }

    let through = position(mesh, vertex);
    let mut normal = Vector3::ZERO;
    let mut widest = 0.0;

    for pair in path.windows(2) {
        let cross = (position(mesh, pair[0]) - through).cross(position(mesh, pair[1]) - through);

        if cross.length() > widest {
            widest = cross.length();
            normal = cross / cross.length();
        }
    }

    if normal == Vector3::ZERO {
        return (false, f64::INFINITY);
    }

    let mut worst_out_of_plane = 0.0;

    for &corner in path {
        worst_out_of_plane = cs_math::max_f64(
            worst_out_of_plane,
            (position(mesh, corner) - through).dot(normal).abs(),
        );
    }

    (worst_out_of_plane <= lattice, worst_out_of_plane)
}

/// C# `PlaneOf`: a face's unit normal and a point it passes through, the normal zero for a face with
/// no area.
fn plane_of(mesh: &Mesh, face: &Face) -> (Vector3, Vector3) {
    let through = position(mesh, face.v0);
    let cross = (position(mesh, face.v1) - through).cross(position(mesh, face.v2) - through);

    let normal = if cross.length() > 0.0 {
        cross / cross.length()
    } else {
        Vector3::ZERO
    };

    (normal, through)
}

/// C# `FacesOnEachEdge`: every face on each edge, by the edge's two vertices (low first).
pub fn faces_on_each_edge(mesh: &Mesh) -> FacesOnEachEdge {
    let mut on_the_edge = FacesOnEachEdge::with_capacity(mesh.faces.len() * 3);

    for (face_index, face) in mesh.faces.iter().enumerate() {
        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            on_the_edge.entry(key).or_default().push(face_index as i32);
        }
    }

    on_the_edge
}

/// C# `FacesAtEachVertex`: every face at each vertex, once each even where a face names a vertex
/// twice; `None` for a vertex no face uses (C#'s null entry).
pub fn faces_at_each_vertex(mesh: &Mesh) -> Vec<Option<Vec<i32>>> {
    let mut at_the_vertex: Vec<Option<Vec<i32>>> = vec![None; mesh.vertices.len()];

    for (face_index, face) in mesh.faces.iter().enumerate() {
        let mut note = |vertex: i32| {
            at_the_vertex[vertex as usize]
                .get_or_insert_with(Vec::new)
                .push(face_index as i32);
        };

        note(face.v0);

        if face.v1 != face.v0 {
            note(face.v1);
        }

        if face.v2 != face.v0 && face.v2 != face.v1 {
            note(face.v2);
        }
    }

    at_the_vertex
}

/// C#'s custom numeric format `0.#########`: up to nine decimal places, trailing zeros dropped, and
/// .NET's spellings of the infinities and NaN.
fn to_nine_places(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_string();
    }

    if value.is_infinite() {
        return if value > 0.0 { "∞" } else { "-∞" }.to_string();
    }

    let text = format!("{value:.9}");
    let text = text.trim_end_matches('0').trim_end_matches('.');

    if text == "-0" {
        "0".to_string()
    } else {
        text.to_string()
    }
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
