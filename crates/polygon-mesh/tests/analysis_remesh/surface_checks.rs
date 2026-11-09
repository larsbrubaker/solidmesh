//! Checks on a returned surface: crossing triangles and survival of sharp edges, computed from
//! the coordinates alone so the remesher's own bookkeeping cannot vouch for itself. A
//! self-intersecting surface cannot bound a volume mesh, so any crossing is a failure.

use std::collections::HashMap;

use solidmesh_mesh::Mesh;

type P = [f64; 3];

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: P, b: P) -> P {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `mesh` as coordinates and faces.
pub fn soup_of(mesh: &Mesh) -> (Vec<P>, Vec<[i32; 3]>) {
    let vertices = mesh
        .vertices
        .iter()
        .map(|p| [f64::from(p.x), f64::from(p.y), f64::from(p.z)])
        .collect();
    let faces = mesh.faces.iter().map(|f| [f.v0, f.v1, f.v2]).collect();
    (vertices, faces)
}

/// Sign of the volume of tetrahedron `a b c d`: which side of plane `a b c` the point `d` is on.
fn orient(a: P, b: P, c: P, d: P) -> f64 {
    dot(cross(sub(b, a), sub(c, a)), sub(d, a))
}

/// Whether the open segment `p`-`q` properly crosses triangle `a b c`.
fn segment_crosses_triangle(p: P, q: P, a: P, b: P, c: P) -> bool {
    let (d_p, d_q) = (orient(a, b, c, p), orient(a, b, c, q));
    if d_p == 0.0 || d_q == 0.0 || (d_p > 0.0) == (d_q > 0.0) {
        return false;
    }
    let s0 = orient(p, q, a, b);
    let s1 = orient(p, q, b, c);
    let s2 = orient(p, q, c, a);
    (s0 > 0.0 && s1 > 0.0 && s2 > 0.0) || (s0 < 0.0 && s1 < 0.0 && s2 < 0.0)
}

fn triangles_cross(a: &[P; 3], b: &[P; 3]) -> bool {
    let edges = |t: &[P; 3]| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])];
    edges(a)
        .iter()
        .any(|&(p, q)| segment_crosses_triangle(p, q, b[0], b[1], b[2]))
        || edges(b)
            .iter()
            .any(|&(p, q)| segment_crosses_triangle(p, q, a[0], a[1], a[2]))
}

/// The edges whose two faces bend by more than `degrees`, as point pairs.
pub fn sharp_edges(vertices: &[P], faces: &[[i32; 3]], degrees: f64) -> Vec<(P, P)> {
    let face_normal = |t: usize| {
        let [a, b, c] = faces[t].map(|v| vertices[v as usize]);
        let n = cross(sub(b, a), sub(c, a));
        let len = dot(n, n).sqrt();
        [n[0] / len, n[1] / len, n[2] / len]
    };
    // Lookup only, so map order cannot reach the result.
    let mut edge_faces: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (t, f) in faces.iter().enumerate() {
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            edge_faces.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let cos_sharp = degrees.to_radians().cos();
    let mut out = Vec::new();
    for (&(a, b), around) in &edge_faces {
        if around.len() == 2 && dot(face_normal(around[0]), face_normal(around[1])) < cos_sharp {
            out.push((vertices[a as usize], vertices[b as usize]));
        }
    }
    out.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    out
}

fn segment_distance(p: P, a: P, b: P) -> f64 {
    let ab = sub(b, a);
    let span = dot(ab, ab);
    let t = if span > 0.0 {
        (dot(sub(p, a), ab) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let d = sub(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t]);
    dot(d, d).sqrt()
}

/// How many of `input`'s sharp edges have no sharp edge of `output` within `reach` of their
/// midpoint.
pub fn lost_sharp_edges(
    input: &(Vec<P>, Vec<[i32; 3]>),
    output: &(Vec<P>, Vec<[i32; 3]>),
    degrees: f64,
    reach: f64,
) -> usize {
    let input_edges = sharp_edges(&input.0, &input.1, degrees);
    let output_edges = sharp_edges(&output.0, &output.1, degrees);
    input_edges
        .iter()
        .filter(|&&(a, b)| {
            let mid = [
                (a[0] + b[0]) * 0.5,
                (a[1] + b[1]) * 0.5,
                (a[2] + b[2]) * 0.5,
            ];
            !output_edges
                .iter()
                .any(|&(p, q)| segment_distance(mid, p, q) <= reach)
        })
        .count()
}

/// Pairs of triangles sharing no vertex whose interiors cross, in input order.
pub fn self_intersections(vertices: &[P], faces: &[[i32; 3]]) -> Vec<(usize, usize)> {
    let tris: Vec<[P; 3]> = faces
        .iter()
        .map(|f| f.map(|v| vertices[v as usize]))
        .collect();
    let bounds: Vec<(P, P)> = tris
        .iter()
        .map(|t| {
            let mut lo = t[0];
            let mut hi = t[0];
            for p in &t[1..] {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
            (lo, hi)
        })
        .collect();
    // Sweep along x so only triangles overlapping in x are compared.
    let count = faces.len();
    let mut order: Vec<usize> = (0..count).collect();
    order.sort_by(|&a, &b| bounds[a].0[0].total_cmp(&bounds[b].0[0]).then(a.cmp(&b)));
    let mut crossings = Vec::new();
    for (i, &t) in order.iter().enumerate() {
        let verts = faces[t];
        for &other in &order[i + 1..] {
            if bounds[other].0[0] > bounds[t].1[0] {
                break;
            }
            let overlaps_box = (1..3).all(|k| {
                bounds[other].0[k] <= bounds[t].1[k] && bounds[other].1[k] >= bounds[t].0[k]
            });
            if !overlaps_box {
                continue;
            }
            if faces[other].iter().any(|v| verts.contains(v)) {
                continue;
            }
            if triangles_cross(&tris[t], &tris[other]) {
                crossings.push((t.min(other), t.max(other)));
            }
        }
    }
    crossings.sort_unstable();
    crossings
}
