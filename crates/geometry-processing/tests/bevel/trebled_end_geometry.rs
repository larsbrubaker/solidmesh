//! Port of the test fixture `LevelLine/TrebledEndGeometry.cs`: the two shapes a real exported
//! part carries AT a chain's end, built as closed manifold geometry - a vertex SPLIT into
//! coincident-to-epsilon copies joined by degenerate slivers, and a small real feature standing
//! microns from that end but joined to the part by ordinary edges. `trebled_end_seam_tests.rs`
//! reads both. Both are built by rewriting a plain block, never by moving a vertex of one: the
//! topology is the whole subject.

use std::collections::HashMap;

use vector_math::{Vector2, Vector3};

/// A triangle list's positions and index triples.
pub type Solid = (Vec<Vector3>, Vec<(i32, i32, i32)>);

/// C# `PlainBlock`: a block `half` either side of the origin, cut into `slices` bands along x, so
/// the top-front crease is a run of edges that can be picked one at a time.
pub fn plain_block(half: f64, slices: i32) -> Solid {
    // The cross-section in (y, z), counter-clockwise seen down -x, from the top-front crease round.
    let section = [
        Vector2::new(-half, half),  // 0 the picked crease
        Vector2::new(-half, -half), // 1 the front wall's foot
        Vector2::new(half, -half),  // 2
        Vector2::new(half, half),   // 3
    ];

    let mut positions = Vec::new();
    for slice in 0..=slices {
        let x = -half + (2.0 * half * slice as f64 / slices as f64);
        for point in &section {
            positions.push(Vector3::new(x, point.x, point.y));
        }
    }

    let mut indices = Vec::new();
    let ring = section.len() as i32;
    for slice in 0..slices {
        let a = slice * ring;
        let b = (slice + 1) * ring;
        for i in 0..ring {
            let next = (i + 1) % ring;
            indices.push((a + i, b + i, b + next));
            indices.push((a + i, b + next, a + next));
        }
    }

    let last = slices * ring;
    for i in 1..ring - 1 {
        indices.push((0, i, i + 1));
        indices.push((last, last + i + 1, last + i));
    }

    (positions, indices)
}

/// One step of a vertex's fan: the triangle, read as (vertex, out, in).
#[derive(Clone, Copy)]
struct FanStep {
    triangle: usize,
    out: i32,
    into: i32,
}

/// C# `SplitVertexAtSpokes`: splits `vertex` into one copy per arc of its triangle fan, the arcs
/// cut at `spokes`, and sews the slits shut with degenerate slivers and one cap triangle - so the
/// result is still closed, still consistently wound, and still manifold. The arc holding `anchor`
/// keeps the ORIGINAL vertex index and position; the remaining arcs take `offsets` in fan order.
pub fn split_vertex_at_spokes(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    vertex: i32,
    spokes: &[i32],
    anchor: i32,
    offsets: &[Vector3],
) -> Solid {
    let mut points = positions.to_vec();
    let mut triangles = indices.to_vec();

    // The fan, each triangle read as (vertex, out, in): its directed boundary leaves the vertex
    // along "out" and comes back along "in", which is what lets the fan be walked in order.
    let mut fan: HashMap<i32, FanStep> = HashMap::new();
    for (t, &(a, b, c)) in triangles.iter().enumerate() {
        let spoke = if a == vertex {
            Some((b, c))
        } else if b == vertex {
            Some((c, a))
        } else if c == vertex {
            Some((a, b))
        } else {
            None
        };
        if let Some((out, into)) = spoke {
            let previous = fan.insert(
                out,
                FanStep {
                    triangle: t,
                    out,
                    into,
                },
            );
            assert!(
                previous.is_none(),
                "C#'s Dictionary.Add: one fan step per spoke"
            );
        }
    }

    let mut order = Vec::new();
    let mut walk = fan[&spokes[0]];
    for _ in 0..fan.len() {
        order.push(walk);
        walk = fan[&walk.into];
    }

    // One arc per spoke, in the order the walk meets them.
    let mut arcs: Vec<Vec<FanStep>> = Vec::new();
    for step in order {
        if arcs.is_empty() || spokes.contains(&step.out) {
            arcs.push(Vec::new());
        }

        arcs.last_mut().expect("an arc").push(step);
    }

    assert_eq!(
        arcs.len(),
        spokes.len(),
        "the fan cut into {} arcs, not {}",
        arcs.len(),
        spokes.len()
    );

    // The copy each arc gets: the anchor's arc keeps the original, the others are new points.
    let mut copies = vec![0i32; arcs.len()];
    let mut taken = 0;
    for k in 0..arcs.len() {
        if arcs[k].iter().any(|s| s.out == anchor || s.into == anchor) {
            copies[k] = vertex;
            continue;
        }

        points.push(positions[vertex as usize] + offsets[taken]);
        taken += 1;
        copies[k] = points.len() as i32 - 1;
    }

    for k in 0..arcs.len() {
        for step in &arcs[k] {
            let (a, b, c) = triangles[step.triangle];
            let swap = |v: i32| if v == vertex { copies[k] } else { v };
            triangles[step.triangle] = (swap(a), swap(b), swap(c));
        }
    }

    // Each arc opens a slit along the spoke it starts at, between the previous arc's copy and its
    // own.
    for k in 0..arcs.len() {
        let previous = copies[(k + arcs.len() - 1) % arcs.len()];
        triangles.push((previous, arcs[k][0].out, copies[k]));
    }

    // And the slivers leave one micro-triangle's worth of directed edges over: the copies' own cap.
    triangles.push((copies[0], copies[1], copies[2]));

    (points, triangles)
}

/// C# `TentInside`: raises a tiny tent inside one triangle - three points inside it, lifted along
/// its normal, with the triangle retriangulated as an annulus around them. The tent's base edges
/// are real feature edges joined to the rest of the part by ORDINARY edges. `corners` are turned
/// round if they are not wound as the triangle is.
pub fn tent_inside(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    triangle: usize,
    corners: &[Vector3],
    height: f64,
) -> Solid {
    let mut points = positions.to_vec();
    let mut triangles = indices.to_vec();
    let (a, b, c) = triangles[triangle];
    let normal = (points[b as usize] - points[a as usize])
        .cross(points[c as usize] - points[a as usize])
        .get_normal();

    let mut wound = corners.to_vec();
    if (wound[1] - wound[0]).cross(wound[2] - wound[0]).dot(normal) < 0.0 {
        wound.reverse();
    }

    let mut base_ring = Vec::new();
    for corner in wound {
        points.push(corner);
        base_ring.push(points.len() as i32 - 1);
    }

    let apex = (points[base_ring[0] as usize]
        + points[base_ring[1] as usize]
        + points[base_ring[2] as usize])
        / 3.0;
    points.push(apex + (normal * height));
    let peak = points.len() as i32 - 1;

    // The annulus, each of the parent's sides bridged to the base ring's matching side.
    let outer = [a, b, c];
    triangles.remove(triangle);
    for i in 0..3 {
        let next = (i + 1) % 3;
        triangles.push((outer[i], outer[next], base_ring[next]));
        triangles.push((outer[i], base_ring[next], base_ring[i]));
    }

    for i in 0..3 {
        let next = (i + 1) % 3;
        triangles.push((base_ring[i], base_ring[next], peak));
    }

    (points, triangles)
}
