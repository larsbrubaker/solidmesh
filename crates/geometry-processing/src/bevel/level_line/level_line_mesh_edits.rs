//! The working mesh's edits: building it from a triangle list, the conforming edge split and the
//! interior split, adding a free vertex and a face, and the triangle bookkeeping under them. Ports
//! `FromTriangles`, `SplitEdge`, `SplitTriangle`, `AddVertex`, `AddFace`, `AddTriangle` and
//! `ReplaceTriangle` of `Bevel/LevelLine/LevelLineMesh.cs`; the type and its readings are in
//! `level_line_mesh.rs`.

use vector_math::{cs_math, Vector3};

use super::super::float_grid;
use super::{
    FieldCounters, LevelLineMesh, LevelLineTriangle, LevelLineWork, NeedleMeter, SplitMeter,
    WorkError,
};

/// Why a working-mesh edit refused (C# throws `ArgumentException`), or the build's own exit.
#[derive(Debug, Clone)]
pub enum LevelLineMeshError {
    /// The work budget ran out or the build was cancelled.
    Work(WorkError),
    /// An edge split was asked of a vertex pair that is no edge (C#'s message).
    NoEdge(String),
    /// A triangle repeats a vertex (C#'s message).
    RepeatsAVertex(String),
    /// A tag list that is not one tag per triangle (C#'s message).
    TagCount(String),
}

impl From<WorkError> for LevelLineMeshError {
    fn from(error: WorkError) -> Self {
        LevelLineMeshError::Work(error)
    }
}

impl std::fmt::Display for LevelLineMeshError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LevelLineMeshError::Work(error) => write!(f, "{error}"),
            LevelLineMeshError::NoEdge(message)
            | LevelLineMeshError::RepeatsAVertex(message)
            | LevelLineMeshError::TagCount(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for LevelLineMeshError {}

impl LevelLineMesh {
    /// Builds a working mesh from positions and index triples, with an optional tag per triangle.
    /// An input vertex is its own ancestry: it stands on every input edge that names it.
    pub fn from_triangles(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        face_tags: Option<&[i32]>,
    ) -> Result<LevelLineMesh, LevelLineMeshError> {
        if let Some(tags) = face_tags {
            if tags.len() != indices.len() {
                return Err(LevelLineMeshError::TagCount(
                    "one tag per triangle, or none at all".to_string(),
                ));
            }
        }

        let mut mesh = LevelLineMesh::new();
        mesh.vertices.extend_from_slice(positions);
        mesh.descends_from
            .extend((0..positions.len() as i32).map(|v| (v, v)));

        for (i, &(a, b, c)) in indices.iter().enumerate() {
            // Uncharged, and a whole part's worth of triangles; a cancel still has to reach it.
            if i & 1023 == 0 {
                LevelLineWork::poll_cancel()?;
            }

            if a == b || b == c || c == a {
                return Err(LevelLineMeshError::RepeatsAVertex(format!(
                    "triangle {i} repeats a vertex"
                )));
            }

            let tag = face_tags.map_or(0, |tags| tags[i]);
            mesh.add_triangle(LevelLineTriangle::new(a, b, c, tag));
        }

        Ok(mesh)
    }

    /// Inserts a vertex on edge a-b and splits EVERY triangle standing on that edge, so the mesh
    /// is never left with a T-junction; returns the new vertex - or, where the point stands at a
    /// vertex the mesh already carries there (an end of the edge to the same-place backstop, a
    /// triangle's opposite corner, or a T-vertex inside the edge one hop through a needle, at
    /// coincidence - widened to the float step on float-stored input unless `widen_for_float_noise`
    /// is off), that vertex, with nothing split. Each parent is replaced in place by the child
    /// holding the a end, the b-end child appended; both keep the parent's winding and tag.
    pub fn split_edge(
        &mut self,
        a: i32,
        b: i32,
        point: Vector3,
        widen_for_float_noise: bool,
    ) -> Result<i32, LevelLineMeshError> {
        // A split is the one thing a runaway refinement does to the mesh, so it is charged here.
        LevelLineWork::charge(1)?;
        let key = Self::key(a, b);
        let Some(standing) = self.edge_triangles.get(&key) else {
            return Err(LevelLineMeshError::NoEdge(format!(
                "no edge between {a} and {b} to split"
            )));
        };
        let standing = standing.clone();

        // The edge is about to stop existing, so whatever a rule concluded about it stops applying.
        if let Some(clean) = &mut self.answered_clean {
            clean.remove(&key);
        }

        // The narrowest test that still catches a duplicate at an END: the build's own snap where
        // one was given, otherwise coincidence to the last bits the edge's length can carry.
        let same_place = if self.same_place > 0.0 {
            self.same_place
        } else {
            1e-12 * self.length(a, b)
        };
        if (point - self.vertices[a as usize]).length() <= same_place {
            return Ok(a);
        }

        if (point - self.vertices[b as usize]).length() <= same_place {
            return Ok(b);
        }

        // An INTERIOR vertex already on the edge (a needle's apex become a T-vertex) is asked for at
        // COINCIDENCE, not at the snap: a point a snap away from it is an honest point. Float-stored
        // input widens it to the float step, but only away from the edge's ends - a near-twin of an
        // end keeps the exact test.
        let exact = 1e-12 * self.length(a, b);
        let mut coincident = exact;
        if widen_for_float_noise && self.input_is_float_stored {
            coincident = cs_math::max_f64(coincident, float_grid::step_at_point(point));
        }

        let (pa, pb) = (self.vertices[a as usize], self.vertices[b as usize]);
        let is_that_place = |mesh: &LevelLineMesh, vertex: i32| {
            let at = mesh.vertices[vertex as usize];
            let distance = (point - at).length();
            if distance <= exact {
                return true;
            }

            distance <= coincident
                && (at - pa).length() > coincident
                && (at - pb).length() > coincident
        };

        for &parent in &standing {
            let t = self.triangles[parent as usize];
            let opposite = Self::opposite_corner(t, a, b);
            if is_that_place(self, opposite) {
                return Ok(opposite);
            }

            // One hop further, and only THROUGH A NEEDLE: a vertex can lie inside an edge it is not
            // an end of only where the strip has no area.
            if !self.has_no_height(t, coincident) {
                continue;
            }

            for corner in 0..3 {
                let u = t.corner(corner);
                let w = t.corner((corner + 1) % 3);
                if (u == a && w == b) || (u == b && w == a) {
                    continue;
                }

                for &across in self.triangles_on(u, w) {
                    let third = Self::opposite_corner(self.triangles[across as usize], u, w);
                    if is_that_place(self, third) {
                        return Ok(third);
                    }
                }
            }
        }

        let mid = self.vertices.len() as i32;
        self.vertices.push(point);
        let behind = Self::merge(
            self.descends_from[a as usize],
            self.descends_from[b as usize],
        );
        self.descends_from.push(behind);
        if SplitMeter::on() {
            SplitMeter::note_edge_split(self, mid, point, a, b, &standing);
        }

        for parent in standing {
            let t = self.triangles[parent as usize];

            // Rotate the triangle so its corners read (x, y, other) with x -> y being this edge in
            // the triangle's own direction; then both children wind the same way the parent did.
            let (x, y, other) = if (t.a == a && t.b == b) || (t.a == b && t.b == a) {
                (t.a, t.b, t.c)
            } else if (t.b == a && t.c == b) || (t.b == b && t.c == a) {
                (t.b, t.c, t.a)
            } else {
                (t.c, t.a, t.b)
            };

            self.replace_triangle(parent, LevelLineTriangle::new(x, mid, other, t.tag));
            self.add_triangle(LevelLineTriangle::new(mid, y, other, t.tag));
        }

        Ok(mid)
    }

    /// Inserts a vertex INSIDE a triangle and replaces it with the three triangles round that
    /// point; returns the new vertex - or, where the point stands at one of the triangle's own
    /// corners (to the same-place backstop), that corner.
    pub fn split_triangle(&mut self, triangle: i32, point: Vector3) -> Result<i32, WorkError> {
        LevelLineWork::charge(1)?;
        let t = self.triangles[triangle as usize];
        let mut longest = 0.0;
        for corner in 0..3 {
            longest = cs_math::max_f64(
                longest,
                self.length(t.corner(corner), t.corner((corner + 1) % 3)),
            );
        }

        let same_place = if self.same_place > 0.0 {
            self.same_place
        } else {
            1e-12 * longest
        };
        for corner in 0..3 {
            if (point - self.vertices[t.corner(corner) as usize]).length() <= same_place {
                return Ok(t.corner(corner));
            }
        }

        let inserted = self.vertices.len() as i32;
        self.vertices.push(point);

        // A point in a triangle's middle stands on no input edge at all.
        self.descends_from.push((-1, -1));
        if SplitMeter::on() {
            SplitMeter::note_triangle_split(self, inserted, point, triangle);
        }

        self.replace_triangle(triangle, LevelLineTriangle::new(t.a, t.b, inserted, t.tag));
        self.add_triangle(LevelLineTriangle::new(t.b, t.c, inserted, t.tag));
        self.add_triangle(LevelLineTriangle::new(t.c, t.a, inserted, t.tag));
        Ok(inserted)
    }

    /// Adds a free vertex (the fill's), standing on no input edge.
    pub fn add_vertex(&mut self, point: Vector3) -> i32 {
        let index = self.vertices.len() as i32;
        self.vertices.push(point);
        self.descends_from.push((-1, -1));
        if SplitMeter::on() {
            SplitMeter::note_free_vertex(self, index, point);
        }

        index
    }

    /// Adds a triangle (C#'s optional `tag = 0` passed explicitly); returns its index. Refuses one
    /// that repeats a vertex.
    pub fn add_face(
        &mut self,
        a: i32,
        b: i32,
        c: i32,
        tag: i32,
    ) -> Result<i32, LevelLineMeshError> {
        if a == b || b == c || c == a {
            return Err(LevelLineMeshError::RepeatsAVertex(format!(
                "a triangle ({a}, {b}, {c}) repeats a vertex"
            )));
        }

        let index = self.triangles.len() as i32;
        self.add_triangle(LevelLineTriangle::new(a, b, c, tag));
        Ok(index)
    }

    /// Appends a triangle and stands it on its edges.
    pub fn add_triangle(&mut self, t: LevelLineTriangle) {
        let index = self.triangles.len() as i32;
        self.triangles.push(t);
        self.removed.push(false);
        self.attach(index, t);
        if FieldCounters::meter_on() {
            NeedleMeter::note_added(index);
        }
    }

    /// Rewrites triangle `index` in place, moving it off its old edges onto its new ones.
    pub fn replace_triangle(&mut self, index: i32, t: LevelLineTriangle) {
        self.detach(index, self.triangles[index as usize]);
        self.triangles[index as usize] = t;
        self.attach(index, t);
        if FieldCounters::meter_on() {
            NeedleMeter::note_written(index);
        }
    }
}
