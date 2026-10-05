//! The part of geometry3Sharp's `DMesh3` (`mesh/DMesh3.cs`) the signed distance grid reaches:
//! appending vertices and triangles with g3's refusals, triangle corners, centroids and bounds,
//! and the cached bounds, and the `AxisAlignedBox3d` arithmetic the implicits and marching
//! cubes use. Plus agg-sharp's `MeshExtensions.ToDMesh3` and `ToMesh`, which convert to and from
//! a [`Mesh`], and g3's point-triangle distance (`MeshQueries.TriDistanceSqr` and
//! `DistPoint3Triangle3.DistanceSqr`, which are the same computation).
//!
//! Nothing is ever removed, so triangle ids are dense (0..count in append order) and every
//! vertex stays live, as in a g3 mesh that has only been appended to.

use std::collections::HashMap;

use crate::Mesh;

/// g3 `DMesh3`, append-only: positions as doubles, triangles as vertex-id triples.
#[derive(Debug, Clone, Default)]
pub(crate) struct DMesh3 {
    vertices: Vec<[f64; 3]>,
    triangles: Vec<[i32; 3]>,
    /// Triangles on each undirected edge, for g3's non-manifold refusal.
    edge_triangles: HashMap<(i32, i32), u8>,
}

/// g3 `AxisAlignedBox3d`, the subset used here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AxisAlignedBox3d {
    pub(crate) min: [f64; 3],
    pub(crate) max: [f64; 3],
}

impl AxisAlignedBox3d {
    /// C# `DistanceSquared(Vector3d)`: zero inside.
    pub(crate) fn distance_squared(&self, v: [f64; 3]) -> f64 {
        let axis = |a: usize| {
            if v[a] < self.min[a] {
                self.min[a] - v[a]
            } else if v[a] > self.max[a] {
                v[a] - self.max[a]
            } else {
                0.0
            }
        };
        let (dx, dy, dz) = (axis(0), axis(1), axis(2));
        dx * dx + dy * dy + dz * dz
    }

    /// C# `AxisAlignedBox3d.Empty`: inverted to the extremes, so any point grows it.
    pub(crate) const EMPTY: AxisAlignedBox3d = AxisAlignedBox3d {
        min: [f64::MAX; 3],
        max: [f64::MIN; 3],
    };

    /// C# `Width` (never negative).
    pub(crate) fn width(&self) -> f64 {
        (self.max[0] - self.min[0]).max(0.0)
    }

    /// C# `Height` (never negative).
    pub(crate) fn height(&self) -> f64 {
        (self.max[1] - self.min[1]).max(0.0)
    }

    /// C# `Depth` (never negative).
    pub(crate) fn depth(&self) -> f64 {
        (self.max[2] - self.min[2]).max(0.0)
    }

    /// C# `MaxDim`.
    pub(crate) fn max_dim(&self) -> f64 {
        self.width().max(self.height().max(self.depth()))
    }

    /// C# `Expand(double)`: grows every side by `radius`.
    pub(crate) fn expand(&mut self, radius: f64) {
        for a in 0..3 {
            self.min[a] -= radius;
            self.max[a] += radius;
        }
    }

    /// C# `Contain(AxisAlignedBox3d)`: grows to cover `other`.
    pub(crate) fn contain(&mut self, other: &AxisAlignedBox3d) {
        for a in 0..3 {
            self.min[a] = self.min[a].min(other.min[a]);
            self.max[a] = self.max[a].max(other.max[a]);
        }
    }

    /// C# `Intersect(AxisAlignedBox3d)`: the overlap, or [`AxisAlignedBox3d::EMPTY`] when it
    /// has no volume.
    pub(crate) fn intersect(&self, other: &AxisAlignedBox3d) -> AxisAlignedBox3d {
        let overlap = AxisAlignedBox3d {
            min: std::array::from_fn(|a| self.min[a].max(other.min[a])),
            max: std::array::from_fn(|a| self.max[a].min(other.max[a])),
        };
        if overlap.height() <= 0.0 || overlap.width() <= 0.0 || overlap.depth() <= 0.0 {
            AxisAlignedBox3d::EMPTY
        } else {
            overlap
        }
    }
}

impl DMesh3 {
    /// C# `AppendVertex`.
    pub(crate) fn append_vertex(&mut self, v: [f64; 3]) -> i32 {
        self.vertices.push(v);
        (self.vertices.len() - 1) as i32
    }

    /// C# `AppendTriangle(v0, v1, v2)`: refuses (returns -1, g3's `InvalidID`, or -2,
    /// `NonManifoldID`) a triangle with a missing or repeated vertex, or one that would put a
    /// third triangle on an edge.
    pub(crate) fn append_triangle(&mut self, v0: i32, v1: i32, v2: i32) -> i32 {
        let vertex_count = self.vertices.len() as i32;
        let is_vertex = |v: i32| v >= 0 && v < vertex_count;
        if !is_vertex(v0) || !is_vertex(v1) || !is_vertex(v2) {
            return -1;
        }
        if v0 == v1 || v0 == v2 || v1 == v2 {
            return -1;
        }
        let edges = [edge_key(v0, v1), edge_key(v1, v2), edge_key(v2, v0)];
        if edges.iter().any(|edge| {
            self.edge_triangles
                .get(edge)
                .is_some_and(|&count| count >= 2)
        }) {
            return -2;
        }
        for edge in edges {
            *self.edge_triangles.entry(edge).or_insert(0) += 1;
        }
        self.triangles.push([v0, v1, v2]);
        (self.triangles.len() - 1) as i32
    }

    /// C# `TriangleCount`.
    pub(crate) fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// C# `VertexCount`.
    pub(crate) fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// C# `GetVertex`.
    pub(crate) fn get_vertex(&self, vid: usize) -> [f64; 3] {
        self.vertices[vid]
    }

    /// C# `GetTriVertices`.
    pub(crate) fn get_tri_vertices(&self, tid: usize) -> [[f64; 3]; 3] {
        let [a, b, c] = self.triangles[tid];
        [
            self.vertices[a as usize],
            self.vertices[b as usize],
            self.vertices[c as usize],
        ]
    }

    /// C# `GetTriCentroid`.
    pub(crate) fn get_tri_centroid(&self, tid: usize) -> [f64; 3] {
        let [a, b, c] = self.get_tri_vertices(tid);
        let f = 1.0 / 3.0;
        [
            (a[0] + b[0] + c[0]) * f,
            (a[1] + b[1] + c[1]) * f,
            (a[2] + b[2] + c[2]) * f,
        ]
    }

    /// C# `GetTriBounds`.
    pub(crate) fn get_tri_bounds(&self, tid: usize) -> AxisAlignedBox3d {
        let corners = self.get_tri_vertices(tid);
        bounds_of(corners.iter().copied())
    }

    /// C# `CachedBounds` (`GetBounds`): the box of every vertex, zero when there are none.
    pub(crate) fn cached_bounds(&self) -> AxisAlignedBox3d {
        if self.vertices.is_empty() {
            return AxisAlignedBox3d {
                min: [0.0; 3],
                max: [0.0; 3],
            };
        }
        bounds_of(self.vertices.iter().copied())
    }
}

/// g3's min/max scan: seeded with the first point, each axis tested `<` then `else if >`.
fn bounds_of(mut points: impl Iterator<Item = [f64; 3]>) -> AxisAlignedBox3d {
    let first = points.next().unwrap_or([0.0; 3]);
    let (mut min, mut max) = (first, first);
    for p in points {
        for a in 0..3 {
            if p[a] < min[a] {
                min[a] = p[a];
            } else if p[a] > max[a] {
                max[a] = p[a];
            }
        }
    }
    AxisAlignedBox3d { min, max }
}

fn edge_key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

impl Mesh {
    /// agg-sharp `MeshExtensions.ToDMesh3`: every vertex, then every face in order (g3 drops
    /// the ones it refuses).
    pub(crate) fn to_dmesh3(&self) -> DMesh3 {
        let mut out = DMesh3::default();
        for vertex in &self.vertices {
            out.append_vertex([vertex.x as f64, vertex.y as f64, vertex.z as f64]);
        }
        for face in self.faces.iter() {
            out.append_triangle(face.v0, face.v1, face.v2);
        }
        out
    }
}

impl DMesh3 {
    /// agg-sharp `MeshExtensions.ToMesh(DMesh3)`: every vertex, then every triangle, in id
    /// order (ids are dense here, so the vertex map is the identity).
    pub(crate) fn to_mesh(&self) -> Mesh {
        let mut out = Mesh::new();
        for v in &self.vertices {
            out.vertices
                .push(vector_math::Vector3Float::new(v[0], v[1], v[2]));
        }
        for t in &self.triangles {
            out.faces.add(t[0], t[1], t[2], &out.vertices);
        }
        out
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// g3 `MeshQueries.TriDistanceSqr` / `DistPoint3Triangle3.DistanceSqr` (Eberly's region walk):
/// the squared distance from `point` to the triangle `v`.
pub(crate) fn tri_distance_sqr(v: &[[f64; 3]; 3], point: [f64; 3]) -> f64 {
    let diff = sub(v[0], point);
    let edge0 = sub(v[1], v[0]);
    let edge1 = sub(v[2], v[0]);
    let a00 = dot(edge0, edge0);
    let a01 = dot(edge0, edge1);
    let a11 = dot(edge1, edge1);
    let b0 = dot(diff, edge0);
    let b1 = dot(diff, edge1);
    let c = dot(diff, diff);
    let det = (a00 * a11 - a01 * a01).abs();
    let mut s = a01 * b1 - a11 * b0;
    let mut t = a01 * b0 - a00 * b1;
    let sqr_distance;

    if s + t <= det {
        if s < 0.0 {
            if t < 0.0 {
                // region 4
                if b0 < 0.0 {
                    if -b0 >= a00 {
                        sqr_distance = a00 + 2.0 * b0 + c;
                    } else {
                        s = -b0 / a00;
                        sqr_distance = b0 * s + c;
                    }
                } else if b1 >= 0.0 {
                    sqr_distance = c;
                } else if -b1 >= a11 {
                    sqr_distance = a11 + 2.0 * b1 + c;
                } else {
                    t = -b1 / a11;
                    sqr_distance = b1 * t + c;
                }
            } else {
                // region 3
                if b1 >= 0.0 {
                    sqr_distance = c;
                } else if -b1 >= a11 {
                    sqr_distance = a11 + 2.0 * b1 + c;
                } else {
                    t = -b1 / a11;
                    sqr_distance = b1 * t + c;
                }
            }
        } else if t < 0.0 {
            // region 5
            if b0 >= 0.0 {
                sqr_distance = c;
            } else if -b0 >= a00 {
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = -b0 / a00;
                sqr_distance = b0 * s + c;
            }
        } else {
            // region 0: minimum at an interior point
            let inv_det = 1.0 / det;
            s *= inv_det;
            t *= inv_det;
            sqr_distance =
                s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
        }
    } else if s < 0.0 {
        // region 2
        let tmp0 = a01 + b0;
        let tmp1 = a11 + b1;
        if tmp1 > tmp0 {
            let numer = tmp1 - tmp0;
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = numer / denom;
                t = 1.0 - s;
                sqr_distance =
                    s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
            }
        } else if tmp1 <= 0.0 {
            sqr_distance = a11 + 2.0 * b1 + c;
        } else if b1 >= 0.0 {
            sqr_distance = c;
        } else {
            t = -b1 / a11;
            sqr_distance = b1 * t + c;
        }
    } else if t < 0.0 {
        // region 6
        let tmp0 = a01 + b1;
        let tmp1 = a00 + b0;
        if tmp1 > tmp0 {
            let numer = tmp1 - tmp0;
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                sqr_distance = a11 + 2.0 * b1 + c;
            } else {
                t = numer / denom;
                s = 1.0 - t;
                sqr_distance =
                    s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
            }
        } else if tmp1 <= 0.0 {
            sqr_distance = a00 + 2.0 * b0 + c;
        } else if b0 >= 0.0 {
            sqr_distance = c;
        } else {
            s = -b0 / a00;
            sqr_distance = b0 * s + c;
        }
    } else {
        // region 1
        let numer = a11 + b1 - a01 - b0;
        if numer <= 0.0 {
            sqr_distance = a11 + 2.0 * b1 + c;
        } else {
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = numer / denom;
                t = 1.0 - s;
                sqr_distance =
                    s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
            }
        }
    }

    // C# `if (sqrDistance < 0) sqrDistance = 0` (NaN passes through, as in C#).
    if sqr_distance < 0.0 {
        0.0
    } else {
        sqr_distance
    }
}
