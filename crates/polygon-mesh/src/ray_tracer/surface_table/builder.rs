//! [`MeshSurfaceBuilder`]: the build behind [`MeshSurface::new`], in slices. A caller with no
//! threads, such as the browser's UI thread, calls [`MeshSurfaceBuilder::step`] each frame with as
//! many work units as it has measured it can afford, and stops by dropping the builder. There is
//! no clock here and no thread: what a slice does depends only on the mesh and the units given,
//! so the finished table is the same bits however the work was sliced.
//!
//! **A work unit** is one small, fixed piece of work: copying one element, or one vertex or
//! corner of the normal smoothing, or one pair of faces compared. A step performs at most its
//! units, plus at most [`MeshSurfaceBuilder::STEP_OVERRUN_UNITS`] for a short sort it keeps whole.
//! The whole mesh costs a handful of units per face, however it is shaped: a vertex with a
//! million faces is split across steps like everything else.
//!
//! **How the sort stays sliced.** Corners are grouped by vertex with a counting sort: count each
//! vertex's corners, turn the counts into bucket starts, then place the corners face by face.
//! Vertices are valid here (only faces with usable corners are counted) and the corners go in
//! face order, so the order is the ascending (vertex, face, corner) order that sorting by that
//! key would give. The corners of one vertex are grouped by neighbour with a sort that is short
//! for an ordinary vertex and a bottom-up merge, resumable at any element, for a large one.
//!
//! The header of [`MeshSurface`](super::MeshSurface) says what the smoothing computes.

use std::sync::Arc;

use solidmesh_math::{AxisAlignedBoundingBox, Vector3};

use super::{other_vertices, unit_or, CornerBounds, MeshSurface, FALLBACK_NORMAL};
use super::{RayTracerError, MAX_FACES_COMPARED_PER_EDGE};
use crate::Mesh;

/// A neighbour list this short is sorted in one piece; a longer one goes through the merge.
const WHOLE_SORT_LENGTH: usize = 64;

/// A face's corner as the smoothing walk sees it.
#[derive(Clone, Copy)]
struct IncidentCorner {
    vertex: i32,
    face: u32,
    corner: u8,
}

/// The mesh being built from: borrowed for the one-call [`MeshSurface::new`], shared so a builder
/// can live across frames beside the mesh it reads.
enum MeshSource<'a> {
    Borrowed(&'a Mesh),
    Shared(Arc<Mesh>),
}

impl MeshSource<'_> {
    fn get(&self) -> &Mesh {
        match self {
            MeshSource::Borrowed(mesh) => mesh,
            MeshSource::Shared(mesh) => mesh,
        }
    }
}

/// The smoothing groups of the faces around one vertex: a disjoint-set forest over the vertex's
/// corners, kept to reuse its allocations from one vertex to the next. Roots are the lowest
/// member, so a group's normal sums its faces in face order, the same every time.
#[derive(Default)]
struct Groups {
    parents: Vec<usize>,
    sums: Vec<Vector3>,
    neighbours: Vec<(i32, usize)>,
    merged: Vec<(i32, usize)>,
}

impl Groups {
    fn find(&mut self, mut node: usize) -> usize {
        while self.parents[node] != node {
            let grandparent = self.parents[self.parents[node]];
            self.parents[node] = grandparent;
            node = grandparent;
        }
        node
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.parents[a.max(b)] = a.min(b);
        }
    }
}

/// Where a bottom-up merge of `neighbours` into `merged` is up to: the width of the runs being
/// merged, and the left and right positions of the current pair of runs.
#[derive(Clone, Copy)]
struct MergeCursor {
    width: usize,
    run_left: usize,
    left: usize,
    right: usize,
}

/// Where the smoothing of one vertex is up to.
#[derive(Clone, Copy)]
enum Phase {
    Begin,
    Reset { done: usize },
    Gather { next: usize },
    Sort,
    Merge(MergeCursor),
    FindRun { run_start: usize, cursor: usize },
    Compare { run_end: usize, i: usize, j: usize },
    Sum { next: usize },
    Write { next: usize },
}

/// Where the whole build is up to.
#[derive(Clone, Copy)]
enum Stage {
    Vertices,
    Faces,
    FaceColors,
    TextureSlots,
    Textures { after: Option<i32> },
    Bounds { face: usize },
    GeometricNormals { face: usize },
    CountCorners { face: usize },
    BucketStarts { vertex: usize },
    IncidentSlots,
    PlaceCorners { face: usize },
    SeedCornerNormals { face: usize },
    Smooth { start: usize, phase: Phase },
    Finish,
    Finished,
}

/// A [`MeshSurface`] being built a slice at a time.
pub struct MeshSurfaceBuilder<'a> {
    mesh: MeshSource<'a>,
    cosine_of_crease: f64,
    surface: MeshSurface,
    stage: Stage,
    cross_lengths: Vec<f64>,
    bounds: Option<(Vector3, Vector3)>,
    bounds_unknown: bool,
    /// One count per vertex plus one: corners per vertex, then the start of each vertex's bucket,
    /// then, once the corners are placed, the end of each.
    bucket_starts: Vec<usize>,
    incident: Vec<IncidentCorner>,
    groups: Groups,
    work_done: u64,
}

impl<'a> MeshSurfaceBuilder<'a> {
    /// The most a [`MeshSurfaceBuilder::step`] does above its `work_units`: a whole short sort.
    pub const STEP_OVERRUN_UNITS: usize = WHOLE_SORT_LENGTH;

    /// A builder of the table for `mesh`, keeping edges bent by `crease_radians` or more sharp.
    /// A crease angle outside `[0, PI]` is a [`RayTracerError::InvalidArgument`]. Nothing is built
    /// until the first [`MeshSurfaceBuilder::step`].
    pub fn new(mesh: &'a Mesh, crease_radians: f64) -> Result<Self, RayTracerError> {
        Self::from_source(MeshSource::Borrowed(mesh), crease_radians)
    }

    /// A builder that shares `mesh`, so it can be kept across frames with nothing to borrow.
    pub fn from_shared(
        mesh: Arc<Mesh>,
        crease_radians: f64,
    ) -> Result<MeshSurfaceBuilder<'static>, RayTracerError> {
        MeshSurfaceBuilder::from_source(MeshSource::Shared(mesh), crease_radians)
    }

    fn from_source(mesh: MeshSource<'a>, crease_radians: f64) -> Result<Self, RayTracerError> {
        if !(0.0..=core::f64::consts::PI).contains(&crease_radians) {
            return Err(RayTracerError::InvalidArgument(
                "the crease angle must be between 0 and PI radians",
            ));
        }
        Ok(MeshSurfaceBuilder {
            mesh,
            cosine_of_crease: libm::cos(crease_radians),
            surface: MeshSurface {
                vertices: Vec::new(),
                faces: Vec::new(),
                geometric_normals: Vec::new(),
                corner_normals: Vec::new(),
                face_colors: None,
                texture_corners: Vec::new(),
                corner_bounds: CornerBounds::NoFaces,
            },
            stage: Stage::Vertices,
            cross_lengths: Vec::new(),
            bounds: None,
            bounds_unknown: false,
            bucket_starts: Vec::new(),
            incident: Vec::new(),
            groups: Groups::default(),
            work_done: 0,
        })
    }

    /// The work units done so far, so a caller or test can see what a slice cost.
    pub fn work_done(&self) -> u64 {
        self.work_done
    }

    /// Does up to `work_units` units (at least one; see the module header for the cap), and
    /// returns the finished table once there is nothing left to do. After that it returns `None`.
    /// Stop early by dropping the builder.
    pub fn step(&mut self, work_units: usize) -> Option<MeshSurface> {
        let mut budget = work_units.max(1);
        loop {
            if matches!(self.stage, Stage::Finished) {
                return None;
            }
            if let Some(surface) = self.advance(&mut budget) {
                return Some(surface);
            }
            if budget == 0 {
                return None;
            }
        }
    }

    /// Charges `amount` units against the step's budget and the running total.
    fn charge(&mut self, budget: &mut usize, amount: usize) {
        *budget = budget.saturating_sub(amount);
        // A work count stays far below 2^63, so the cast keeps it.
        self.work_done += amount as u64;
    }

    /// Runs the current stage as far as `budget` allows, returning the table when it is done.
    fn advance(&mut self, budget: &mut usize) -> Option<MeshSurface> {
        match self.stage {
            Stage::Vertices => {
                let source = self.mesh.get().vertices.as_slice();
                let done = self.surface.vertices.len();
                let take = (*budget).min(source.len() - done);
                self.surface
                    .vertices
                    .extend_from_slice(&source[done..done + take]);
                let finished = done + take == source.len();
                self.charge(budget, take);
                if finished {
                    self.stage = Stage::Faces;
                }
            }
            Stage::Faces => {
                let source = self.mesh.get().faces.as_slice();
                let done = self.surface.faces.len();
                let take = (*budget).min(source.len() - done);
                self.surface
                    .faces
                    .extend(source[done..done + take].iter().map(|f| [f.v0, f.v1, f.v2]));
                let finished = done + take == source.len();
                self.charge(budget, take);
                if finished {
                    self.stage = Stage::FaceColors;
                }
            }
            Stage::FaceColors => {
                let Some(source) = self.mesh.get().face_colors.as_ref() else {
                    self.stage = Stage::TextureSlots;
                    return None;
                };
                let target = self.surface.face_colors.get_or_insert_with(Vec::new);
                let done = target.len();
                let take = (*budget).min(source.len() - done);
                target.extend_from_slice(&source[done..done + take]);
                let finished = done + take == source.len();
                self.charge(budget, take);
                if finished {
                    self.stage = Stage::TextureSlots;
                }
            }
            Stage::TextureSlots => {
                if self.mesh.get().face_textures.is_empty() {
                    self.stage = Stage::Bounds { face: 0 };
                    return None;
                }
                let want = self.surface.faces.len();
                let done = self.surface.texture_corners.len();
                let take = (*budget).min(want - done);
                self.surface.texture_corners.resize(done + take, None);
                let finished = done + take == want;
                self.charge(budget, take);
                if finished {
                    self.stage = Stage::Textures { after: None };
                }
            }
            Stage::Textures { after } => {
                let textures = &self.mesh.get().face_textures;
                let lower = match after {
                    Some(key) => std::ops::Bound::Excluded(key),
                    None => std::ops::Bound::Unbounded,
                };
                let mut last = after;
                let mut taken = 0;
                for (&face, texture) in textures.range((lower, std::ops::Bound::Unbounded)) {
                    if taken == *budget {
                        break;
                    }
                    if let Some(slot) = usize::try_from(face)
                        .ok()
                        .and_then(|face| self.surface.texture_corners.get_mut(face))
                    {
                        *slot = Some([texture.uv0, texture.uv1, texture.uv2]);
                    }
                    last = Some(face);
                    taken += 1;
                }
                let exhausted = taken < *budget;
                self.charge(budget, taken);
                self.stage = if exhausted {
                    Stage::Bounds { face: 0 }
                } else {
                    Stage::Textures { after: last }
                };
            }
            Stage::Bounds { face } => self.bounds_step(face, budget),
            Stage::GeometricNormals { face } => self.geometric_normals_step(face, budget),
            Stage::CountCorners { face } => self.count_corners_step(face, budget),
            Stage::BucketStarts { vertex } => self.bucket_starts_step(vertex, budget),
            Stage::IncidentSlots => {
                let want = self.bucket_starts.last().copied().unwrap_or(0);
                let done = self.incident.len();
                let take = (*budget).min(want - done);
                self.incident.resize(
                    done + take,
                    IncidentCorner {
                        vertex: 0,
                        face: 0,
                        corner: 0,
                    },
                );
                let finished = done + take == want;
                self.charge(budget, take);
                if finished {
                    self.stage = Stage::PlaceCorners { face: 0 };
                }
            }
            Stage::PlaceCorners { face } => self.place_corners_step(face, budget),
            Stage::SeedCornerNormals { face } => self.seed_corner_normals_step(face, budget),
            Stage::Smooth { start, phase } => self.smooth_step(start, phase, budget),
            Stage::Finish => {
                self.surface.corner_bounds = if self.bounds_unknown {
                    CornerBounds::Unknown
                } else {
                    match self.bounds {
                        Some((low, high)) => {
                            CornerBounds::Known(AxisAlignedBoundingBox::new(low, high))
                        }
                        None => CornerBounds::NoFaces,
                    }
                };
                self.stage = Stage::Finished;
                return Some(std::mem::replace(
                    &mut self.surface,
                    MeshSurface {
                        vertices: Vec::new(),
                        faces: Vec::new(),
                        geometric_normals: Vec::new(),
                        corner_normals: Vec::new(),
                        face_colors: None,
                        texture_corners: Vec::new(),
                        corner_bounds: CornerBounds::NoFaces,
                    },
                ));
            }
            Stage::Finished => {}
        }
        None
    }

    fn bounds_step(&mut self, mut face: usize, budget: &mut usize) {
        let end = face + (*budget).min(self.surface.faces.len() - face);
        let (begin, surface) = (face, &self.surface);
        let mut unknown = false;
        let mut bounds = self.bounds;
        while face < end {
            let Some(corners) = surface.corners(face) else {
                unknown = true;
                break;
            };
            for corner in corners {
                let (low, high) = bounds.unwrap_or((corner, corner));
                bounds = Some((
                    Vector3::component_min(low, corner),
                    Vector3::component_max(high, corner),
                ));
            }
            face += 1;
        }
        self.bounds = bounds;
        self.charge(budget, face - begin);
        if unknown {
            self.bounds_unknown = true;
            self.stage = Stage::GeometricNormals { face: 0 };
        } else if face == self.surface.faces.len() {
            self.stage = Stage::GeometricNormals { face: 0 };
        } else {
            self.stage = Stage::Bounds { face };
        }
    }

    fn geometric_normals_step(&mut self, mut face: usize, budget: &mut usize) {
        let end = face + (*budget).min(self.surface.faces.len() - face);
        let begin = face;
        while face < end {
            let (normal, cross_length) = match self.surface.corners(face) {
                Some([c0, c1, c2]) => {
                    let cross = (c1 - c0).cross(c2 - c0);
                    let length = cross.length();
                    if length > 0.0 && length.is_finite() {
                        (cross / length, length)
                    } else {
                        (FALLBACK_NORMAL, 0.0)
                    }
                }
                None => (FALLBACK_NORMAL, 0.0),
            };
            self.surface.geometric_normals.push(normal);
            self.cross_lengths.push(cross_length);
            face += 1;
        }
        self.charge(budget, face - begin);
        self.stage = if face == self.surface.faces.len() {
            self.bucket_starts = vec![0; self.surface.vertices.len() + 1];
            Stage::CountCorners { face: 0 }
        } else {
            Stage::GeometricNormals { face }
        };
    }

    fn count_corners_step(&mut self, mut face: usize, budget: &mut usize) {
        let end = face + (*budget).min(self.surface.faces.len() - face);
        let begin = face;
        while face < end {
            // A face with no area was never placed; a face with area has valid vertex indices.
            if self.cross_lengths[face] != 0.0 {
                for vertex in self.surface.faces[face] {
                    // `corners` said this index is in range, so it is not negative.
                    self.bucket_starts[vertex as usize + 1] += 1;
                }
            }
            face += 1;
        }
        self.charge(budget, face - begin);
        self.stage = if face == self.surface.faces.len() {
            Stage::BucketStarts { vertex: 1 }
        } else {
            Stage::CountCorners { face }
        };
    }

    fn bucket_starts_step(&mut self, mut vertex: usize, budget: &mut usize) {
        let end = vertex + (*budget).min(self.bucket_starts.len().saturating_sub(vertex));
        let begin = vertex;
        while vertex < end {
            self.bucket_starts[vertex] += self.bucket_starts[vertex - 1];
            vertex += 1;
        }
        self.charge(budget, vertex - begin);
        self.stage = if vertex >= self.bucket_starts.len() {
            Stage::IncidentSlots
        } else {
            Stage::BucketStarts { vertex }
        };
    }

    fn place_corners_step(&mut self, mut face: usize, budget: &mut usize) {
        let end = face + (*budget).min(self.surface.faces.len() - face);
        let begin = face;
        while face < end {
            if self.cross_lengths[face] != 0.0 {
                for (corner, &vertex) in self.surface.faces[face].iter().enumerate() {
                    // `corners` said this index is in range, so it is not negative.
                    let start = self.bucket_starts[vertex as usize];
                    self.incident[start] = IncidentCorner {
                        vertex,
                        // A face list fits an i32 index, so the cast keeps it.
                        face: face as u32,
                        corner: corner as u8,
                    };
                    self.bucket_starts[vertex as usize] = start + 1;
                }
            }
            face += 1;
        }
        self.charge(budget, face - begin);
        self.stage = if face == self.surface.faces.len() {
            Stage::SeedCornerNormals { face: 0 }
        } else {
            Stage::PlaceCorners { face }
        };
    }

    fn seed_corner_normals_step(&mut self, mut face: usize, budget: &mut usize) {
        let end = face + (*budget).min(self.surface.faces.len() - face);
        let begin = face;
        while face < end {
            let normal = self.surface.geometric_normals[face];
            self.surface.corner_normals.push([normal; 3]);
            face += 1;
        }
        self.charge(budget, face - begin);
        self.stage = if face == self.surface.faces.len() {
            Stage::Smooth {
                start: 0,
                phase: Phase::Begin,
            }
        } else {
            Stage::SeedCornerNormals { face }
        };
    }

    /// The end of the corner bucket beginning at `start` of the sorted incident list.
    fn bucket_end(&self, start: usize) -> usize {
        // A placed corner's vertex is a valid index; see `count_corners_step`.
        self.bucket_starts[self.incident[start].vertex as usize]
    }

    fn smooth_step(&mut self, start: usize, phase: Phase, budget: &mut usize) {
        if start >= self.incident.len() {
            self.stage = Stage::Finish;
            return;
        }
        let end = self.bucket_end(start);
        let count = end - start;
        let next_phase = match phase {
            Phase::Begin => {
                self.groups.parents.clear();
                self.groups.sums.clear();
                self.groups.neighbours.clear();
                self.charge(budget, 1);
                Phase::Reset { done: 0 }
            }
            Phase::Reset { done } => {
                let take = (*budget).min(count - done);
                self.groups.parents.extend(done..done + take);
                self.groups.sums.resize(done + take, Vector3::ZERO);
                self.charge(budget, take);
                if done + take == count {
                    Phase::Gather { next: 0 }
                } else {
                    Phase::Reset { done: done + take }
                }
            }
            Phase::Gather { next } => {
                let take = (*budget).min(count - next);
                for index in next..next + take {
                    let corner = self.incident[start + index];
                    let face = self.surface.faces[corner.face as usize];
                    let (a, b) = other_vertices(face, usize::from(corner.corner));
                    self.groups.neighbours.push((a, index));
                    self.groups.neighbours.push((b, index));
                }
                self.charge(budget, take);
                if next + take == count {
                    Phase::Sort
                } else {
                    Phase::Gather { next: next + take }
                }
            }
            Phase::Sort => {
                let length = self.groups.neighbours.len();
                if length <= WHOLE_SORT_LENGTH {
                    self.groups.neighbours.sort_unstable();
                    self.charge(budget, length);
                    Phase::FindRun {
                        run_start: 0,
                        cursor: 0,
                    }
                } else {
                    self.groups.merged.clear();
                    Phase::Merge(MergeCursor {
                        width: 1,
                        run_left: 0,
                        left: 0,
                        right: 1.min(length),
                    })
                }
            }
            Phase::Merge(cursor) => self.merge_step(cursor, budget),
            Phase::FindRun { run_start, cursor } => {
                if run_start >= self.groups.neighbours.len() {
                    Phase::Sum { next: 0 }
                } else {
                    let other = self.groups.neighbours[run_start].0;
                    let mut cursor = cursor.max(run_start);
                    let mut scanned = 0;
                    while cursor < self.groups.neighbours.len()
                        && self.groups.neighbours[cursor].0 == other
                        && scanned < *budget
                    {
                        cursor += 1;
                        scanned += 1;
                    }
                    self.charge(budget, scanned);
                    if cursor < self.groups.neighbours.len()
                        && self.groups.neighbours[cursor].0 == other
                    {
                        Phase::FindRun { run_start, cursor }
                    } else {
                        Phase::Compare {
                            run_end: cursor,
                            i: run_start,
                            j: run_start + 1,
                        }
                    }
                }
            }
            Phase::Compare { run_end, i, j } => self.compare_step(start, run_end, i, j, budget),
            Phase::Sum { next } => {
                let take = (*budget).min(count - next);
                for index in next..next + take {
                    let corner = self.incident[start + index];
                    let face = corner.face as usize;
                    let root = self.groups.find(index);
                    let angle = self.surface.corner_angle(face, usize::from(corner.corner));
                    self.groups.sums[root] += self.surface.geometric_normals[face] * angle;
                }
                self.charge(budget, take);
                if next + take == count {
                    Phase::Write { next: 0 }
                } else {
                    Phase::Sum { next: next + take }
                }
            }
            Phase::Write { next } => {
                let take = (*budget).min(count - next);
                for index in next..next + take {
                    let corner = self.incident[start + index];
                    let face = corner.face as usize;
                    let root = self.groups.find(index);
                    self.surface.corner_normals[face][usize::from(corner.corner)] =
                        unit_or(self.groups.sums[root], self.surface.geometric_normals[face]);
                }
                self.charge(budget, take);
                if next + take == count {
                    self.stage = Stage::Smooth {
                        start: end,
                        phase: Phase::Begin,
                    };
                    return;
                }
                Phase::Write { next: next + take }
            }
        };
        self.stage = Stage::Smooth {
            start,
            phase: next_phase,
        };
    }

    /// One slice of the bottom-up merge of `neighbours`, which holds runs of `cursor.width`; the
    /// merged runs of twice that go to `merged` in order, and the two lists swap at the end of a
    /// pass. A comparison of two runs' heads moves one element, so the slice is one per unit.
    fn merge_step(&mut self, mut cursor: MergeCursor, budget: &mut usize) -> Phase {
        let n = self.groups.neighbours.len();
        loop {
            let mid = (cursor.run_left + cursor.width).min(n);
            let run_end = (cursor.run_left + 2 * cursor.width).min(n);
            let groups = &mut self.groups;
            let mut moved = 0;
            while moved < *budget {
                let take_left = cursor.left < mid
                    && (cursor.right >= run_end
                        || groups.neighbours[cursor.left] <= groups.neighbours[cursor.right]);
                if take_left {
                    groups.merged.push(groups.neighbours[cursor.left]);
                    cursor.left += 1;
                } else if cursor.right < run_end {
                    groups.merged.push(groups.neighbours[cursor.right]);
                    cursor.right += 1;
                } else {
                    break;
                }
                moved += 1;
            }
            self.charge(budget, moved);
            if cursor.left < mid || cursor.right < run_end {
                return Phase::Merge(cursor);
            }

            cursor.run_left += 2 * cursor.width;
            cursor.left = cursor.run_left;
            cursor.right = (cursor.run_left + cursor.width).min(n);
            if cursor.run_left >= n {
                std::mem::swap(&mut self.groups.neighbours, &mut self.groups.merged);
                self.groups.merged.clear();
                cursor.width *= 2;
                if cursor.width >= n {
                    return Phase::FindRun {
                        run_start: 0,
                        cursor: 0,
                    };
                }
                cursor.run_left = 0;
                cursor.left = 0;
                cursor.right = cursor.width.min(n);
            }
        }
    }

    /// One slice of comparing the faces sharing an edge: compared with each other within a run
    /// of the same other vertex, up to `MAX_FACES_COMPARED_PER_EDGE` apart.
    fn compare_step(
        &mut self,
        start: usize,
        run_end: usize,
        mut i: usize,
        mut j: usize,
        budget: &mut usize,
    ) -> Phase {
        loop {
            if i >= run_end {
                return Phase::FindRun {
                    run_start: run_end,
                    cursor: run_end,
                };
            }
            if *budget == 0 {
                return Phase::Compare { run_end, i, j };
            }
            let last = (i + 1 + MAX_FACES_COMPARED_PER_EDGE).min(run_end);
            self.charge(budget, 1);
            if j >= last {
                i += 1;
                j = i + 1;
                continue;
            }
            let first = self.groups.neighbours[i].1;
            let second = self.groups.neighbours[j].1;
            let corner_a = self.incident[start + first];
            let corner_b = self.incident[start + second];
            let normal_a = self.surface.geometric_normals[corner_a.face as usize];
            let normal_b = self.surface.geometric_normals[corner_b.face as usize];
            // Strictly less than the crease: an edge bent exactly the crease angle stays sharp.
            if normal_a.dot(normal_b) > self.cosine_of_crease {
                self.groups.join(first, second);
            }
            j += 1;
        }
    }
}
