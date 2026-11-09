//! Curved and hard-input parts for the analysis remesh tests: icosphere shells with thin walls,
//! a plate with a smooth bump, and the point-to-mesh distance the tests measure with. The measure
//! samples far more densely than the remesher's own guards, so it can catch a surface that only
//! looks within its bound at the remesher's own sample points.

use std::collections::HashMap;

use solidmesh_math::Vector3Float;
use solidmesh_mesh::Mesh;

pub type P = [f64; 3];

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// A mesh of the given points and faces.
pub fn mesh_of(points: &[P], faces: &[[usize; 3]]) -> Mesh {
    let mut mesh = Mesh::new();
    for p in points {
        mesh.vertices.push(Vector3Float::new(p[0], p[1], p[2]));
    }
    for f in faces {
        mesh.faces
            .add(f[0] as i32, f[1] as i32, f[2] as i32, &mesh.vertices);
    }
    mesh
}

/// A sphere of `radius` made by subdividing an octahedron `level` times, wound outward.
pub fn icosphere(radius: f64, level: u32) -> (Vec<P>, Vec<[usize; 3]>) {
    let mut points: Vec<P> = vec![
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ];
    let mut faces: Vec<[usize; 3]> = vec![
        [0, 2, 4],
        [2, 1, 4],
        [1, 3, 4],
        [3, 0, 4],
        [2, 0, 5],
        [1, 2, 5],
        [3, 1, 5],
        [0, 3, 5],
    ];
    for _ in 0..level {
        let mut middle: HashMap<(usize, usize), usize> = HashMap::new();
        let mut finer = Vec::with_capacity(faces.len() * 4);
        for t in &faces {
            let mut mid = [0; 3];
            for i in 0..3 {
                let (a, b) = (t[i], t[(i + 1) % 3]);
                mid[i] = *middle.entry((a.min(b), a.max(b))).or_insert_with(|| {
                    let s = [
                        points[a][0] + points[b][0],
                        points[a][1] + points[b][1],
                        points[a][2] + points[b][2],
                    ];
                    let l = dot(s, s).sqrt();
                    points.push([s[0] / l, s[1] / l, s[2] / l]);
                    points.len() - 1
                });
            }
            finer.push([t[0], mid[0], mid[2]]);
            finer.push([t[1], mid[1], mid[0]]);
            finer.push([t[2], mid[2], mid[1]]);
            finer.push([mid[0], mid[1], mid[2]]);
        }
        faces = finer;
    }
    (
        points
            .iter()
            .map(|p| [p[0] * radius, p[1] * radius, p[2] * radius])
            .collect(),
        faces,
    )
}

/// A closed spherical shell: outer radius `radius`, wall `wall`, inner surface wound inward.
pub fn hollow_sphere(radius: f64, wall: f64, level: u32) -> Mesh {
    let (mut points, mut faces) = icosphere(radius, level);
    let (inner_points, inner_faces) = icosphere(radius - wall, level);
    let offset = points.len();
    points.extend(inner_points);
    for t in inner_faces {
        faces.push([t[0] + offset, t[2] + offset, t[1] + offset]);
    }
    mesh_of(&points, &faces)
}

/// A 30 mm plate 1 mm thick with a smooth 0.6 mm bump on its top, finely triangulated.
pub fn bump_plate() -> Mesh {
    let n = 120_usize;
    let h = 0.25;
    let bump = |x: f64, y: f64| {
        let d2 = (x - 7.3) * (x - 7.3) + (y - 11.1) * (y - 11.1);
        0.6 * (-d2 / (2.0 * 0.8 * 0.8)).exp()
    };
    let mut points: Vec<P> = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            let (x, y) = (i as f64 * h, j as f64 * h);
            points.push([x, y, 1.0 + bump(x, y)]);
        }
    }
    for j in 0..=n {
        for i in 0..=n {
            points.push([i as f64 * h, j as f64 * h, 0.0]);
        }
    }
    let id =
        |i: usize, j: usize, top: bool| (if top { 0 } else { (n + 1) * (n + 1) }) + j * (n + 1) + i;
    let mut faces = Vec::new();
    for j in 0..n {
        for i in 0..n {
            faces.push([id(i, j, true), id(i + 1, j, true), id(i + 1, j + 1, true)]);
            faces.push([id(i, j, true), id(i + 1, j + 1, true), id(i, j + 1, true)]);
            faces.push([
                id(i, j, false),
                id(i + 1, j + 1, false),
                id(i + 1, j, false),
            ]);
            faces.push([
                id(i, j, false),
                id(i, j + 1, false),
                id(i + 1, j + 1, false),
            ]);
        }
    }
    for i in 0..n {
        faces.push([id(i, 0, false), id(i + 1, 0, false), id(i + 1, 0, true)]);
        faces.push([id(i, 0, false), id(i + 1, 0, true), id(i, 0, true)]);
        faces.push([id(i + 1, n, false), id(i, n, false), id(i, n, true)]);
        faces.push([id(i + 1, n, false), id(i, n, true), id(i + 1, n, true)]);
        faces.push([id(0, i + 1, false), id(0, i, false), id(0, i, true)]);
        faces.push([id(0, i + 1, false), id(0, i, true), id(0, i + 1, true)]);
        faces.push([id(n, i, false), id(n, i + 1, false), id(n, i + 1, true)]);
        faces.push([id(n, i, false), id(n, i + 1, true), id(n, i, true)]);
    }
    mesh_of(&points, &faces)
}

/// A long thin cone: its apex fan has corners far under any useful angle floor, and the edges
/// meeting there bend too little to be creases.
pub fn needle_cone() -> Mesh {
    cone(1.0, 30.0, 24)
}

/// A faceted cone standing on its base with its apex first in the point list.
pub fn cone(radius: f64, height: f64, sides: usize) -> Mesh {
    let n = sides;
    let mut points: Vec<P> = vec![[0.0, 0.0, height], [0.0, 0.0, 0.0]];
    for i in 0..n {
        let a = std::f64::consts::TAU * i as f64 / n as f64;
        points.push([radius * a.cos(), radius * a.sin(), 0.0]);
    }
    let mut faces = Vec::new();
    for i in 0..n {
        let (x, y) = (2 + i, 2 + (i + 1) % n);
        faces.push([0, x, y]);
        faces.push([1, y, x]);
    }
    mesh_of(&points, &faces)
}

fn closest_on_triangle(p: P, a: P, b: P, c: P) -> P {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let along = |o: P, d: P, t: f64| [o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t];
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return along(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return along(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return along(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    along(along(a, ab, vb * denom), ac, vc * denom)
}

/// Nearest-triangle queries over a triangle list through a uniform grid of its bounding boxes.
struct Grid {
    triangles: Vec<[P; 3]>,
    cell: f64,
    origin: P,
    cells: HashMap<[i32; 3], Vec<u32>>,
}

impl Grid {
    fn new(triangles: Vec<[P; 3]>) -> Self {
        let mut low = triangles[0][0];
        let mut high = low;
        let mut total_edge = 0.0;
        for t in &triangles {
            total_edge += dot(sub(t[1], t[0]), sub(t[1], t[0])).sqrt();
            for p in t {
                for k in 0..3 {
                    low[k] = low[k].min(p[k]);
                    high[k] = high[k].max(p[k]);
                }
            }
        }
        let cell = (total_edge / triangles.len() as f64 * 2.0).max(1e-6);
        let mut grid = Self {
            triangles,
            cell,
            origin: low,
            cells: HashMap::new(),
        };
        for t in 0..grid.triangles.len() {
            let tri = grid.triangles[t];
            let mut lo = tri[0];
            let mut hi = tri[0];
            for p in &tri[1..] {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
            let (a, b) = (grid.key(lo), grid.key(hi));
            for x in a[0]..=b[0] {
                for y in a[1]..=b[1] {
                    for z in a[2]..=b[2] {
                        grid.cells.entry([x, y, z]).or_default().push(t as u32);
                    }
                }
            }
        }
        grid
    }

    fn key(&self, p: P) -> [i32; 3] {
        [0, 1, 2].map(|k| ((p[k] - self.origin[k]) / self.cell).floor() as i32)
    }

    /// The distance from `p` to the nearest triangle, walking outward in rings until no closer
    /// triangle can exist.
    fn distance(&self, p: P) -> f64 {
        let key = self.key(p);
        let mut best = f64::MAX;
        let mut ring = 0;
        loop {
            for x in key[0] - ring..=key[0] + ring {
                for y in key[1] - ring..=key[1] + ring {
                    for z in key[2] - ring..=key[2] + ring {
                        let on_shell = (x - key[0]).abs() == ring
                            || (y - key[1]).abs() == ring
                            || (z - key[2]).abs() == ring;
                        if !on_shell {
                            continue;
                        }
                        if let Some(list) = self.cells.get(&[x, y, z]) {
                            for &t in list {
                                let [a, b, c] = self.triangles[t as usize];
                                let q = closest_on_triangle(p, a, b, c);
                                let s = sub(p, q);
                                best = best.min(dot(s, s));
                            }
                        }
                    }
                }
            }
            let reached = ring as f64 * self.cell;
            if best.sqrt() <= reached || ring > 4096 {
                return best.sqrt();
            }
            ring += 1;
        }
    }
}

/// The largest distance from `from`'s surface, sampled on a barycentric grid of `n` per triangle
/// side, to `to`'s surface. Nearest is across every sheet, so a sheet-confused surface only reads
/// as closer than it should, never farther.
pub fn dense_deviation(from: &[[P; 3]], to: &[[P; 3]], n: usize) -> f64 {
    let grid = Grid::new(to.to_vec());
    let mut worst = 0.0_f64;
    for tri in from {
        for i in 0..=n {
            for j in 0..=(n - i) {
                let (u, v, w) = (
                    i as f64 / n as f64,
                    j as f64 / n as f64,
                    (n - i - j) as f64 / n as f64,
                );
                let p = [0, 1, 2].map(|k| tri[0][k] * u + tri[1][k] * v + tri[2][k] * w);
                worst = worst.max(grid.distance(p));
            }
        }
    }
    worst
}

/// `mesh`'s triangles as corner points.
pub fn triangles_of(mesh: &Mesh) -> Vec<[P; 3]> {
    mesh.faces
        .iter()
        .map(|f| {
            [f.v0, f.v1, f.v2].map(|v| {
                let p = mesh.vertices[v as usize];
                [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
            })
        })
        .collect()
}

/// The vertex indices of each connected component of `faces`, largest first.
pub fn components(vertex_count: usize, faces: &[[i32; 3]]) -> Vec<Vec<usize>> {
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let mut parent: Vec<usize> = (0..vertex_count).collect();
    for f in faces {
        for (a, b) in [(f[0], f[1]), (f[1], f[2])] {
            let (ra, rb) = (find(&mut parent, a as usize), find(&mut parent, b as usize));
            parent[ra] = rb;
        }
    }
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..vertex_count {
        let root = find(&mut parent, i);
        groups.entry(root).or_default().push(i);
    }
    let mut out: Vec<Vec<usize>> = groups.into_values().collect();
    out.sort_by_key(|g| std::cmp::Reverse(g.len()));
    out
}
