//! The readings of [`GrowingCreaseRuns`] past its constructor: whether a crease grows, the
//! straight run it belongs to and that run's far end,
//! the far creases that bound a growing run (less the convex seams onto the next facet of one wall,
//! [`GrowingCreaseRuns::read_bounds`]), the ridges off the chain's crease, and the size at
//! which a ridge first pierces the round (the fillet pierce room, read from the angle's centre).
//! Split from
//! `growing_crease_runs.rs` by responsibility.

use std::collections::{HashMap, HashSet, VecDeque};

use solidmesh_math::{ieee_math, Vector3};

use super::{
    end_face_sense, triangle_edges, BandProfile, GrowingCreaseRuns, LevelLineMesh, LevelLineWork,
    WorkError,
};

/// The two sides of a chain edge as the pierce room reads them: a point of the edge, its unit
/// direction, side 1's normal, side 1's in-face direction away from the edge, and side 2's normal.
pub type ChainSides = (Vector3, Vector3, Vector3, Vector3, Vector3);

impl GrowingCreaseRuns {
    /// The least size at which a ridge rising out of the chain's crease
    /// pierces the round (as built of `fillet_segments` chords, or the true round for 0), over every
    /// unit setback the chain carries - infinity where no ridge does.
    pub fn fillet_pierce_room(&self, unit_setback: &[f64], fillet_segments: i32) -> f64 {
        let inscribed = if fillet_segments > 0 {
            libm::cos(std::f64::consts::PI / (2.0 * f64::from(fillet_segments)))
        } else {
            1.0
        };
        let ridges = self.ridges.borrow().clone();
        if ridges.is_empty() {
            return f64::INFINITY;
        }

        let mut room = f64::INFINITY;
        let mut seen_ridges = Vec::new();
        for (near, far) in ridges {
            if seen_ridges.contains(&(near, far)) {
                continue;
            }
            seen_ridges.push((near, far));

            let (start, far_point) = {
                let mesh = self.mesh.borrow();
                (mesh.position(near), mesh.position(far))
            };

            // Against the leg the ridge runs from, never the first edge alone.
            let leg_edge = self.nearest_chain_edge(start);
            let Some((a0, along, n1, t1, n2)) = self.chain_sides(leg_edge) else {
                continue;
            };

            let chain_length = (self.mesh.borrow().position(leg_edge.1) - a0).length();
            let direction = (start - far_point).get_normal();
            let mut common = direction.cross(along);
            if common.length() < 1e-12 {
                continue;
            }

            common = common.get_normal();
            let mut seen_setbacks = Vec::new();
            for &u in unit_setback {
                if seen_setbacks.contains(&u) {
                    continue;
                }
                seen_setbacks.push(u);

                // The centre's side of side 1: whichever normal puts it one radius from side 2.
                let m1 = if (((t1 * u) - n1).dot(n2).abs() - 1.0).abs()
                    <= (((t1 * u) + n1).dot(n2).abs() - 1.0).abs()
                {
                    -n1
                } else {
                    n1
                };
                let per_size = (t1 * u) + m1;
                let c0 = (a0 - start).dot(common);
                let c1 = per_size.dot(common);
                for sign in [1.0, -1.0] {
                    let denominator = (sign * inscribed) - c1;
                    if (sign * c1) - inscribed <= Self::GRAZING_MARGIN {
                        continue;
                    }

                    // A root at the scale of rounding is a ridge that starts ON the chain line.
                    let s = c0 / denominator;
                    if s > Self::ROOT_FLOOR * chain_length {
                        room = ieee_math::min_f64(room, s);
                    }
                }
            }
        }

        room
    }

    /// `room`, lowered for a fillet to every growing run's pierce room.
    pub fn least(
        room: f64,
        growing: &[GrowingCreaseRuns],
        unit_setback: &[f64],
        profile: BandProfile,
        fillet_segments: i32,
    ) -> f64 {
        if profile != BandProfile::Fillet {
            return room;
        }

        growing.iter().fold(room, |least, g| {
            let pierce = g.fillet_pierce_room(unit_setback, fillet_segments);
            ieee_math::min_f64(least, pierce)
        })
    }

    /// The chain edge's two sides (one triangle tagged 1, one tagged 2), or
    /// `None` where it has not exactly one of each.
    pub fn chain_sides(&self, chain_edge: (i32, i32)) -> Option<ChainSides> {
        let mesh = self.mesh.borrow();
        let on_chain = mesh.triangles_on(chain_edge.0, chain_edge.1);
        let one: Vec<i32> = on_chain
            .iter()
            .copied()
            .filter(|&t| mesh.triangles[t as usize].tag == 1)
            .collect();
        let two: Vec<i32> = on_chain
            .iter()
            .copied()
            .filter(|&t| mesh.triangles[t as usize].tag == 2)
            .collect();
        if one.len() != 1 || two.len() != 1 {
            return None;
        }

        let a0 = mesh.position(chain_edge.0);
        let along = (mesh.position(chain_edge.1) - a0).get_normal();
        let face = mesh.triangles[one[0] as usize];
        let second = mesh.triangles[two[0] as usize];
        drop(mesh);
        let n1 = self.normal(face);
        let mut t1 = along.cross(n1).get_normal();
        let third = [face.a, face.b, face.c]
            .into_iter()
            .find(|&v| v != chain_edge.0 && v != chain_edge.1)
            .expect("a triangle has a third corner");
        if (self.mesh.borrow().position(third) - a0).dot(t1) < 0.0 {
            t1 = -t1;
        }

        Some((a0, along, n1, t1, self.normal(second)))
    }

    /// Whether `edge` is a crease of the opposite sense to the chain's first leg.
    pub fn grows(&self, edge: (i32, i32)) -> bool {
        end_face_sense::is_opposite_sense_crease_on(&self.mesh.borrow(), self.chain_edge, edge)
    }

    /// The end of `edge`'s straight growing run farther from the chain, cached for
    /// every piece of the run.
    pub fn far_end(&self, edge: (i32, i32)) -> Result<i32, WorkError> {
        let key = triangle_edges::key(edge.0, edge.1);
        if let Some(&cached) = self.far_end.borrow().get(&key) {
            return Ok(cached);
        }

        self.ensure_adjacency()?;
        let mut pieces = vec![key];
        let a = self.walk(edge.1, edge.0, &mut pieces)?;
        let b = self.walk(edge.0, edge.1, &mut pieces)?;
        let (pa, pb) = {
            let mesh = self.mesh.borrow();
            (mesh.position(a), mesh.position(b))
        };
        let far = if (self.distance_to_chain)(pa) >= (self.distance_to_chain)(pb) {
            a
        } else {
            b
        };

        let mut far_end = self.far_end.borrow_mut();
        for piece in pieces {
            far_end.insert(piece, far);
        }

        Ok(far)
    }

    /// The far creases bounding `edge`'s growing run (cached, `None` included), each
    /// noted as a far bound.
    pub fn bounds_of(
        &self,
        edge: (i32, i32),
        chain_ends: &HashSet<i32>,
    ) -> Result<Option<Vec<(i32, i32)>>, WorkError> {
        let key = triangle_edges::key(edge.0, edge.1);
        if let Some(cached) = self.bounds.borrow().get(&key) {
            return Ok(cached.clone());
        }

        let result = self.read_bounds(edge, chain_ends)?;
        self.bounds.borrow_mut().insert(key, result.clone());
        let mut bound_edges = self.bound_edges.borrow_mut();
        for bound in result.iter().flatten() {
            bound_edges.insert(triangle_edges::key(bound.0, bound.1));
        }

        Ok(result)
    }

    /// For a growing crease on one of the chain's own sides whose run ends at no
    /// chain end, the boundary creases of the far triangle's coplanar untagged region - less the
    /// run itself, the ridges out of its NEAR end (noted as ridges instead), the CONVEX seams out
    /// of its FAR end onto a face that carries the run on as the next facet of one wall (a faceted
    /// cylinder's next facet, which grows its own cap: `faces_carrying_the_run_on`) and creases at
    /// a chain end.
    pub fn read_bounds(
        &self,
        edge: (i32, i32),
        chain_ends: &HashSet<i32>,
    ) -> Result<Option<Vec<(i32, i32)>>, WorkError> {
        let (side_triangle, far_triangle, side) = {
            let mesh = self.mesh.borrow();
            let standing = mesh.triangles_on(edge.0, edge.1);
            if standing.len() != 2 {
                return Ok(None);
            }
            let side_triangle = if mesh.triangles[standing[0] as usize].tag != 0 {
                standing[0]
            } else {
                standing[1]
            };
            let far_triangle = if side_triangle == standing[0] {
                standing[1]
            } else {
                standing[0]
            };
            (
                side_triangle,
                far_triangle,
                mesh.triangles[side_triangle as usize].tag,
            )
        };
        let _ = side_triangle;

        if !self.grows(edge) || !(self.is_own_side)(side) {
            return Ok(None);
        }

        self.ensure_adjacency()?;
        let mut run = vec![triangle_edges::key(edge.0, edge.1)];
        let a = self.walk(edge.1, edge.0, &mut run)?;
        let b = self.walk(edge.0, edge.1, &mut run)?;
        if chain_ends.contains(&a) || chain_ends.contains(&b) {
            return Ok(None);
        }

        let mesh = self.mesh.borrow();
        let near = if (self.distance_to_chain)(mesh.position(a))
            <= (self.distance_to_chain)(mesh.position(b))
        {
            a
        } else {
            b
        };

        // C: the far triangle's coplanar untagged region, flooded to its own creases.
        let unit = |t: i32| {
            let f = mesh.triangles[t as usize];
            (mesh.position(f.b) - mesh.position(f.a))
                .cross(mesh.position(f.c) - mesh.position(f.a))
                .get_normal()
        };
        let normal = unit(far_triangle);
        let mut in_c = HashSet::from([far_triangle]);
        let mut work = VecDeque::from([far_triangle]);
        let mut boundary = Vec::new();
        while let Some(t) = work.pop_front() {
            LevelLineWork::charge(1)?;
            let face = mesh.triangles[t as usize];
            for (p, q) in triangle_edges::corners((face.a, face.b, face.c)) {
                let across: Vec<i32> = mesh
                    .triangles_on(p, q)
                    .iter()
                    .copied()
                    .filter(|t| !in_c.contains(t))
                    .collect();
                let flat = across.len() == 1
                    && mesh.triangles[across[0] as usize].tag == 0
                    && unit(across[0]).dot(normal) >= 1.0 - 1e-9;
                if flat {
                    in_c.insert(across[0]);
                    work.push_back(across[0]);
                } else if !across.is_empty() {
                    boundary.push(triangle_edges::key(p, q));
                }
            }
        }

        let at_a_chain_end = |e: (i32, i32)| chain_ends.contains(&e.0) || chain_ends.contains(&e.1);
        let run_keys: HashSet<(i32, i32)> = run.iter().copied().collect();
        let mut distinct = Vec::new();
        for e in boundary {
            if !distinct.contains(&e) {
                distinct.push(e);
            }
        }

        // C's seams out of its FAR end onto the next facet of the same wall, whose own cap meets
        // C's along the seam: they do not bound either.
        let far = if near == a { b } else { a };
        let carrying_on = self.faces_carrying_the_run_on(&mesh, near, far, &run_keys, normal)?;
        let seams: HashSet<(i32, i32)> = distinct
            .iter()
            .copied()
            .filter(|e| (e.0 == far || e.1 == far) && !run_keys.contains(e))
            .filter(|e| {
                let on = mesh.triangles_on(e.0, e.1);
                let Some(&own) = on.iter().find(|t| in_c.contains(t)) else {
                    return false;
                };
                on.iter().any(|&t| {
                    !in_c.contains(&t)
                        && mesh.triangles[t as usize].tag == 0
                        && carrying_on.iter().any(|n| unit(t).dot(*n) >= 1.0 - 1e-9)
                        // A CONVEX seam only: the strip clip drops the strip where every face of
                        // the neighbour stands over it, which is the neighbour's inside only where
                        // its faces meet convexly. A concave seam (a V-notch, a wall bending back
                        // into the part) stays a bound.
                        && end_face_sense::is_convex(
                            &mesh.vertices,
                            corners_of(mesh.triangles[own as usize]),
                            corners_of(mesh.triangles[t as usize]),
                            e.0,
                        )
                })
            })
            .collect();

        // C's ridges out of the run's NEAR end are what the cap extends past: they do not bound.
        let mut ridges = self.ridges.borrow_mut();
        for &(p, q) in distinct
            .iter()
            .filter(|e| !run_keys.contains(e) && (e.0 == near || e.1 == near))
        {
            ridges.push((near, if p == near { q } else { p }));
        }

        Ok(Some(
            distinct
                .into_iter()
                .filter(|e| {
                    !run_keys.contains(e)
                        && e.0 != near
                        && e.1 != near
                        && !at_a_chain_end(*e)
                        && !seams.contains(e)
                })
                .collect(),
        ))
    }

    /// The normals of the faces that carry a growing run on past its far end `far` as facets of one
    /// wall: the untagged face of each growing crease of this chain's own sides leaving `far`
    /// (other than the run's own pieces) that carries the run's line on and whose face carries C's
    /// on (normal `c_normal`), by the kernel's one reading of "the curve carries on"
    /// (`end_face_sense::carries_on`, under which a 12-gon is one wall), as a stopped end and an
    /// oblique end read a faceted curve's next facet. Such a
    /// face grows its own cap over the band, meeting C's along the seam between them, so the seam
    /// is no far crease. A crease turning sharper is a corner of the neighbour (a rib's back), past
    /// which C's cap would have to turn: that seam still bounds.
    fn faces_carrying_the_run_on(
        &self,
        mesh: &LevelLineMesh,
        near: i32,
        far: i32,
        run: &HashSet<(i32, i32)>,
        c_normal: Vector3,
    ) -> Result<Vec<Vector3>, WorkError> {
        let along = (mesh.position(far) - mesh.position(near)).get_normal();
        let around = self
            .neighbours
            .borrow()
            .as_ref()
            .and_then(|n| n.get(&far))
            .cloned()
            .unwrap_or_default();
        let mut normals = Vec::new();
        for w in around {
            LevelLineWork::charge(1)?;
            if run.contains(&triangle_edges::key(far, w)) {
                continue;
            }

            let on = mesh.triangles_on(far, w);
            if on.len() != 2 {
                continue;
            }
            let (first, second) = (
                mesh.triangles[on[0] as usize],
                mesh.triangles[on[1] as usize],
            );
            let (side, face) = match (first.tag, second.tag) {
                (0, 0) => continue,
                (0, _) => (second, first),
                (_, 0) => (first, second),
                _ => continue,
            };
            if !(self.is_own_side)(side.tag) || !self.grows((far, w)) {
                continue;
            }

            let step = mesh.position(w) - mesh.position(far);
            if step.length() == 0.0 || !end_face_sense::carries_on(along.dot(step.get_normal())) {
                continue;
            }

            let face_normal = (mesh.position(face.b) - mesh.position(face.a))
                .cross(mesh.position(face.c) - mesh.position(face.a))
                .get_normal();
            if !end_face_sense::carries_on(face_normal.dot(c_normal)) {
                continue;
            }

            normals.push(face_normal);
        }

        Ok(normals)
    }

    /// Whether `edge` is a noted ridge whose near end stands on
    /// the chain's crease (an edge between sides 1 and 2).
    pub fn is_ridge_off_the_chains_crease(&self, edge: (i32, i32)) -> bool {
        let key = triangle_edges::key(edge.0, edge.1);
        let neighbours = self.neighbours.borrow();
        let mesh = self.mesh.borrow();
        for &(near, far) in self.ridges.borrow().iter() {
            if triangle_edges::key(near, far) != key {
                continue;
            }

            let Some(around) = neighbours.as_ref().and_then(|n| n.get(&near)) else {
                continue;
            };

            for &other in around {
                let tags: Vec<i32> = mesh
                    .triangles_on(near, other)
                    .iter()
                    .map(|&t| mesh.triangles[t as usize].tag)
                    .collect();
                if tags.len() == 2 && tags.contains(&1) && tags.contains(&2) {
                    return true;
                }
            }
        }

        false
    }

    /// From `at` (arrived at from `from`), on along growing creases that run straight
    /// on, adding each to `pieces`; the vertex it stops at.
    pub fn walk(
        &self,
        mut from: i32,
        mut at: i32,
        pieces: &mut Vec<(i32, i32)>,
    ) -> Result<i32, WorkError> {
        loop {
            LevelLineWork::charge(1)?;
            let next = {
                let mesh = self.mesh.borrow();
                let along = (mesh.position(at) - mesh.position(from)).get_normal();
                let neighbours = self.neighbours.borrow();
                let around = neighbours
                    .as_ref()
                    .and_then(|n| n.get(&at))
                    .cloned()
                    .unwrap_or_default();
                drop(neighbours);
                let mut next = -1;
                for candidate in around {
                    if candidate == from {
                        continue;
                    }

                    let step = mesh.position(candidate) - mesh.position(at);
                    let length = step.length();
                    if length == 0.0
                        || along.dot(step / length) < 1.0 - 1e-12
                        || !end_face_sense::is_opposite_sense_crease_on(
                            &mesh,
                            self.chain_edge,
                            (at, candidate),
                        )
                    {
                        continue;
                    }

                    next = candidate;
                    break;
                }
                next
            };

            if next < 0 {
                return Ok(at);
            }

            pieces.push(triangle_edges::key(at, next));
            (from, at) = (at, next);
        }
    }

    /// Every vertex's neighbours along the mesh's edges, each edge charged.
    pub fn adjacency(mesh: &super::LevelLineMesh) -> Result<HashMap<i32, Vec<i32>>, WorkError> {
        let mut neighbours = HashMap::new();
        for (low, high) in mesh.edges() {
            LevelLineWork::charge(1)?;
            GrowingCreaseRuns::add(&mut neighbours, low, high);
            GrowingCreaseRuns::add(&mut neighbours, high, low);
        }

        Ok(neighbours)
    }

    /// Builds the vertex-to-edge adjacency the first time a walk needs it, and only then.
    fn ensure_adjacency(&self) -> Result<(), WorkError> {
        if self.neighbours.borrow().is_none() {
            let built = GrowingCreaseRuns::adjacency(&self.mesh.borrow())?;
            *self.neighbours.borrow_mut() = Some(built);
        }

        Ok(())
    }
}

/// A working-mesh triangle's three corners.
fn corners_of(t: super::LevelLineTriangle) -> (i32, i32, i32) {
    (t.a, t.b, t.c)
}
