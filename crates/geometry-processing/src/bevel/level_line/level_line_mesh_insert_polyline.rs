//! The polyline insertion half of the working mesh. Ports
//! `Bevel/LevelLine/LevelLineMesh.InsertPolyline.cs` (a partial of `LevelLineMesh`): the
//! insertion of a polyline through a flat region as a chain of edges
//! ([`LevelLineMesh::insert_polyline`]) and the removal of the region it cuts off
//! ([`LevelLineMesh::remove_enclosed`]).
//!
//! The fixed rule: a point is ON a corner, an edge or the segment only where the orientation reads
//! exactly zero (no tolerance); the mesh's own same-place backstop in the splits is the only
//! nearness. A piece that does not come out as a mesh edge refuses (`None`) rather than leave a
//! T-junction.

use std::collections::HashSet;

use vector_math::Vector3;

use super::{LevelLineMesh, LevelLineMeshError};

/// C#'s `Math.Sign` on a double: -1, 0 or 1. Never handed a NaN: `insert_polyline` refuses a
/// polyline or plane that is not finite before any orientation is read (divergence 1073).
fn sign(x: f64) -> i32 {
    if x > 0.0 {
        1
    } else if x < 0.0 {
        -1
    } else {
        0
    }
}

/// The orientation of c against the line a-b in the plane's 2D frame.
fn orient(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    ((b.0 - a.0) * (c.1 - a.1)) - ((b.1 - a.1) * (c.0 - a.0))
}

impl LevelLineMesh {
    /// C# `LevelLineMesh.InsertPolyline`: inserts `points` as a chain of edges through the flat
    /// region of kept triangles `in_region` accepts (asked of the mesh as it stands, as C#'s
    /// closure over it reads it), all in the plane of `plane_normal`; answers
    /// the chain's vertices in order (every point of the polyline and every crossing), or `None`
    /// where it cannot. Each segment's first point is made a vertex, then every region edge the
    /// open segment crosses is split where it crosses, nearest first, then its last point.
    ///
    /// An empty polyline, or one whose points or plane are not finite, is refused (`None`) with
    /// the mesh untouched (divergence 1073: C#'s `Math.Sign` threw on the NaN orientations).
    pub fn insert_polyline(
        &mut self,
        points: &[Vector3],
        plane_normal: Vector3,
        in_region: &dyn Fn(&LevelLineMesh, i32) -> bool,
    ) -> Result<Option<Vec<i32>>, LevelLineMeshError> {
        let n = plane_normal.get_normal();
        let u = (if n.x.abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        })
        .cross(n)
        .get_normal();
        let w = n.cross(u);
        let finite = |v: Vector3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        if points.is_empty() || !finite(u) || !finite(w) || !points.iter().all(|&p| finite(p)) {
            return Ok(None);
        }

        let origin = points[0];
        let flat = |p: Vector3| ((p - origin).dot(u), (p - origin).dot(w));

        let mut chain: Vec<i32> = Vec::new();
        let Some(start) = self.make_vertex(points[0], &flat, in_region)? else {
            return Ok(None);
        };

        chain.push(start);
        for i in 0..points.len().saturating_sub(1) {
            let q3 = points[i + 1];
            let fq = flat(q3);
            loop {
                // The nearest region edge the OPEN segment from the chain's last vertex to q
                // crosses properly, or the nearest region vertex standing exactly on it.
                let from = chain[chain.len() - 1];
                let f_from = flat(self.vertices[from as usize]);
                let mut best = f64::INFINITY;
                let mut crossed = (-1, -1);
                let mut on_vertex = -1;
                let mut seen = HashSet::new();
                for t in self.region(in_region) {
                    let tri = self.triangles[t as usize];
                    for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
                        if !seen.insert(Self::key(a, b)) || a == from || b == from {
                            continue;
                        }

                        let fa = flat(self.vertices[a as usize]);
                        let fb = flat(self.vertices[b as usize]);
                        let sa = orient(f_from, fq, fa);
                        let sb = orient(f_from, fq, fb);
                        let ta = orient(fa, fb, f_from);
                        let tq = orient(fa, fb, fq);
                        if sa == 0.0 || sb == 0.0 {
                            for (v, fv, sv) in [(a, fa, sa), (b, fb, sb)] {
                                let d = (fq.0 - f_from.0, fq.1 - f_from.1);
                                let along = ((fv.0 - f_from.0) * d.0 + (fv.1 - f_from.1) * d.1)
                                    / (d.0 * d.0 + d.1 * d.1);
                                if sv == 0.0 && along > 0.0 && along < 1.0 && along < best {
                                    best = along;
                                    on_vertex = v;
                                    crossed = (-1, -1);
                                }
                            }

                            continue;
                        }

                        if sign(sa) != sign(sb) && ta != 0.0 && tq != 0.0 && sign(ta) != sign(tq) {
                            let along = ta / (ta - tq);
                            if along < best {
                                best = along;
                                crossed = (a, b);
                                on_vertex = -1;
                            }
                        }
                    }
                }

                if best == f64::INFINITY {
                    break;
                }

                let next = if on_vertex >= 0 {
                    on_vertex
                } else {
                    let at = self.vertices[from as usize];
                    self.split_edge(crossed.0, crossed.1, at + ((q3 - at) * best), true)?
                };
                if next == from || !self.has_edge(from, next) {
                    return Ok(None);
                }

                chain.push(next);
            }

            let end = self.make_vertex(q3, &flat, in_region)?;
            let last = chain[chain.len() - 1];
            let Some(end) = end.filter(|&e| e == last || self.has_edge(last, e)) else {
                return Ok(None);
            };

            if end != last {
                chain.push(end);
            }
        }

        Ok(Some(chain))
    }

    /// The live region triangles, in index order.
    fn region(&self, in_region: &dyn Fn(&LevelLineMesh, i32) -> bool) -> Vec<i32> {
        (0..self.triangle_count())
            .filter(|&t| !self.removed[t as usize] && in_region(self, t))
            .collect()
    }

    /// The polyline insertion's `MakeVertex`: the vertex at `p` in the region - a corner it stands
    /// on, an edge it stands on (split), or the inside of a triangle (split) - or `None` where no
    /// region triangle holds it.
    fn make_vertex(
        &mut self,
        p: Vector3,
        flat: &dyn Fn(Vector3) -> (f64, f64),
        in_region: &dyn Fn(&LevelLineMesh, i32) -> bool,
    ) -> Result<Option<i32>, LevelLineMeshError> {
        let q = flat(p);
        for t in self.region(in_region) {
            let tri = self.triangles[t as usize];
            let a = flat(self.vertices[tri.a as usize]);
            let b = flat(self.vertices[tri.b as usize]);
            let c = flat(self.vertices[tri.c as usize]);
            let s = sign(orient(a, b, c)) as f64;
            let o = [
                orient(b, c, q) * s,
                orient(c, a, q) * s,
                orient(a, b, q) * s,
            ];
            if o.iter().any(|&x| x < 0.0) {
                continue;
            }

            let zeros = o.iter().filter(|&&x| x == 0.0).count();
            if zeros >= 2 {
                // On a corner: the corner the two zero edges share.
                return Ok(Some(if o[0] != 0.0 {
                    tri.a
                } else if o[1] != 0.0 {
                    tri.b
                } else {
                    tri.c
                }));
            }

            if zeros == 1 {
                let (e0, e1) = if o[0] == 0.0 {
                    (tri.b, tri.c)
                } else if o[1] == 0.0 {
                    (tri.c, tri.a)
                } else {
                    (tri.a, tri.b)
                };
                return Ok(Some(self.split_edge(e0, e1, p, true)?));
            }

            return Ok(Some(self.split_triangle(t, p)?));
        }

        Ok(None)
    }

    /// Removes the region triangles reached from those standing at `seed` without crossing an
    /// edge of `chain` - the part of the flat region the inserted polyline cuts off round the
    /// seed. Returns how many were removed. (The removal order does not reach the mesh: each
    /// removal only detaches its own entries.)
    pub fn remove_enclosed(
        &mut self,
        seed: i32,
        chain: &[i32],
        in_region: &dyn Fn(i32) -> bool,
    ) -> i32 {
        let mut wall = HashSet::new();
        for pair in chain.windows(2) {
            wall.insert(Self::key(pair[0], pair[1]));
        }

        let mut stack: Vec<i32> = (0..self.triangle_count())
            .filter(|&t| {
                let tri = self.triangles[t as usize];
                !self.removed[t as usize]
                    && in_region(t)
                    && (tri.a == seed || tri.b == seed || tri.c == seed)
            })
            .collect();
        let mut reached = HashSet::new();
        while let Some(t) = stack.pop() {
            if !reached.insert(t) {
                continue;
            }

            let tri = self.triangles[t as usize];
            for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
                if wall.contains(&Self::key(a, b)) {
                    continue;
                }

                for &other in self.triangles_on(a, b) {
                    if other != t
                        && !self.removed[other as usize]
                        && in_region(other)
                        && !reached.contains(&other)
                    {
                        stack.push(other);
                    }
                }
            }
        }

        let count = reached.len() as i32;
        for t in reached {
            self.remove_triangle(t);
        }

        count
    }
}
