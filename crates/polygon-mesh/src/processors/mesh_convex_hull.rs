//! The convex hull of a mesh's vertices, cached on the mesh, built by
//! `mi_convex_hull` (derived from MIConvexHull). Each hull point is one vertex of the hull mesh and
//! its faces share them, so the hull is a closed solid by construction.
//! `TransformedAabbCache` (`crate::transformed_aabb_cache`) measures a transformed mesh's bounds over
//! the vertices on or outside this hull (`hull_bounds_points`).
//!
//! The mesh owns a typed [`ConvexHullCache`] whose entry carries the [`Mesh::changed_count`] it
//! was built at, so any [`Mesh::mark_as_changed`] makes it stale, without a property bag or an
//! event. [`ConvexHullSteps`] builds the same hull a piece at a time for a thread that must keep
//! drawing frames, and caches it the same way. Every call is synchronous (a background hull of a
//! mesh the owner may be editing would race), so there is no in-flight marker either.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};

use vector_math::{AxisAlignedBoundingBox, Vector3, Vector3Float};

use super::mi_convex_hull;
use crate::mesh::f32_key;
use crate::Mesh;

/// What [`Mesh::get_convex_hull`] hands back: the cached hull, or - when the mesh is flat or
/// empty - the mesh itself, which is then its own best hull.
#[derive(Debug, Clone)]
pub enum ConvexHull<'a> {
    Hull(Arc<Mesh>),
    Mesh(&'a Mesh),
}

impl ConvexHull<'_> {
    /// The hull mesh.
    pub fn mesh(&self) -> &Mesh {
        match self {
            ConvexHull::Hull(hull) => hull,
            ConvexHull::Mesh(mesh) => mesh,
        }
    }
}

/// The hull cached on a mesh, tagged with the `changed_count` it was built at. Cloning a mesh
/// starts the clone with an empty cache.
#[derive(Debug, Default)]
pub(crate) struct ConvexHullCache(Mutex<Option<(i32, Arc<Mesh>)>>);

impl Clone for ConvexHullCache {
    fn clone(&self) -> Self {
        Self::default()
    }
}

impl ConvexHullCache {
    fn get(&self, changed_count: i32) -> Option<Arc<Mesh>> {
        let guard = self.0.lock().unwrap_or_else(|e| e.into_inner());
        match &*guard {
            Some((count, hull)) if *count == changed_count => Some(hull.clone()),
            _ => None,
        }
    }

    fn set(&self, changed_count: i32, hull: Arc<Mesh>) {
        // last wins - two threads hulling the same (unchanged) mesh produce equivalent hulls
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = Some((changed_count, hull));
    }
}

impl Mesh {
    /// Gets the convex hull of the mesh, caching it on the mesh (and dropping the cache when the
    /// mesh changes). `None` when the mesh has fewer than 4 faces or the hull cannot be built.
    ///
    /// `_generate_async` is ignored and the hull is always built synchronously: a background
    /// task walking a mesh the owning thread may still be editing would race with the edit.
    pub fn get_convex_hull(&self, _generate_async: bool) -> Option<ConvexHull<'_>> {
        if self.faces.len() < 4 {
            return None;
        }
        let changed_count = self.changed_count();
        if let Some(hull) = self.convex_hull_cache.get(changed_count) {
            return Some(ConvexHull::Hull(hull));
        }
        self.create_hull_mesh(changed_count)
    }

    /// Whether a hull built at the mesh's current [`Mesh::changed_count`] is cached.
    pub fn has_cached_convex_hull(&self) -> bool {
        self.convex_hull_cache.get(self.changed_count()).is_some()
    }

    fn create_hull_mesh(&self, changed_count: i32) -> Option<ConvexHull<'_>> {
        let mut steps = ConvexHullSteps::building(self, changed_count);
        loop {
            if let Some(hull) = steps.step(usize::MAX) {
                return hull;
            }
        }
    }
}

/// [`Mesh::get_convex_hull`] built a piece at a time ([`step`](Self::step)), so a thread that must
/// keep drawing frames (the browser's) can spread a large mesh's hull over them. The pieces run
/// the one build in its own order, so the hull, and the hull cached on the mesh at the end, are
/// the ones `get_convex_hull` builds; a hull already cached is answered at the first step.
pub struct ConvexHullSteps<'a> {
    mesh: &'a Mesh,
    changed_count: i32,
    phase: Phase,
    /// The vertex visits done so far ([`work`](Self::work)).
    work: usize,
}

/// Where a [`ConvexHullSteps`] build is.
enum Phase {
    /// Collecting the distinct vertices, `next` the first vertex not read yet.
    Distinct {
        next: usize,
        seen: HashSet<(u32, u32, u32)>,
        points: Vec<[f64; 3]>,
        bounds: AxisAlignedBoundingBox,
    },
    /// The quickhull over those points.
    Hull {
        build: Box<mi_convex_hull::HullBuild>,
        points: Vec<[f64; 3]>,
    },
    /// The answer was handed out.
    Done,
}

/// The distance tolerance the hull is built with.
const TOLERANCE: f64 = 0.01;

/// What reading one vertex while collecting the distinct ones counts, in vertex visits (a vertex
/// tested against a hull face): the set lookup takes about six times as long, measured natively
/// on a 96,000-face mesh.
const DISTINCT_READ_VISITS: usize = 6;

impl<'a> ConvexHullSteps<'a> {
    /// The hull of `mesh`, nothing done yet.
    pub fn new(mesh: &'a Mesh) -> Self {
        Self::building(mesh, mesh.changed_count())
    }

    fn building(mesh: &'a Mesh, changed_count: i32) -> Self {
        Self {
            mesh,
            changed_count,
            work: 0,
            phase: Phase::Distinct {
                next: 0,
                seen: HashSet::new(),
                points: Vec::new(),
                bounds: AxisAlignedBoundingBox::empty(),
            },
        }
    }

    /// The vertex visits the next [`step`](Self::step) of `work` does at least: `work`, except
    /// that the hull's first piece reads every distinct vertex twice in one go, one vertex read
    /// is the least a piece collecting the distinct vertices does, and the piece that reads the
    /// last vertices also copies every distinct vertex into the hull's setup (counted here over
    /// the ones found so far; the few the piece itself adds are recorded once it is done).
    pub fn next_work(&self, work: usize) -> usize {
        match &self.phase {
            Phase::Hull { build, .. } => build.next_work(work),
            Phase::Distinct { next, points, .. } => {
                let reads = (work / DISTINCT_READ_VISITS).max(1);
                let reading = work.max(DISTINCT_READ_VISITS);
                if next.saturating_add(reads) >= self.mesh.vertices.len() {
                    reading.saturating_add(points.len())
                } else {
                    reading
                }
            }
            Phase::Done => work.max(DISTINCT_READ_VISITS),
        }
    }

    /// The vertex visits the build has done so far, which [`step`](Self::step) may take past the
    /// `work` it was asked for: a caller that charges each step's work up front adds what a step
    /// did beyond it.
    pub fn work(&self) -> usize {
        self.work
    }

    /// Does about `work` vertex visits of the build (a vertex read, or tested against a hull
    /// face); one iteration of the hull's main loop may pass it, and the hull's first piece does
    /// [`next_work`](Self::next_work). `Some` with what `get_convex_hull` answers once the build is over,
    /// `None` while there is more to do. Not called again once it has answered `Some`.
    pub fn step(&mut self, work: usize) -> Option<Option<ConvexHull<'a>>> {
        let mesh = self.mesh;
        match &mut self.phase {
            Phase::Distinct {
                next,
                seen,
                points,
                bounds,
            } => {
                if *next == 0 {
                    if mesh.faces.len() < 4 {
                        self.phase = Phase::Done;
                        return Some(None);
                    }
                    if let Some(hull) = mesh.convex_hull_cache.get(self.changed_count) {
                        self.phase = Phase::Done;
                        return Some(Some(ConvexHull::Hull(hull)));
                    }
                }
                // Get the convex hull for the mesh over the distinct vertices (float ==, first
                // kept)
                let reads = (work / DISTINCT_READ_VISITS).max(1);
                let end = next.saturating_add(reads).min(mesh.vertices.len());
                for &position in &mesh.vertices[*next..end] {
                    let has_nan = position.x.is_nan() || position.y.is_nan() || position.z.is_nan();
                    // NaN never equals itself, so every NaN position is kept
                    if has_nan
                        || seen.insert((
                            f32_key(position.x),
                            f32_key(position.y),
                            f32_key(position.z),
                        ))
                    {
                        points.push([position.x as f64, position.y as f64, position.z as f64]);
                        bounds.expand_to_include_point(Vector3::from_vector3_float(position));
                    }
                }
                self.work = self
                    .work
                    .saturating_add((end - *next).saturating_mul(DISTINCT_READ_VISITS));
                *next = end;
                if end < mesh.vertices.len() {
                    return None;
                }
                let points = std::mem::take(points);
                if points.is_empty()
                    || bounds.x_size() <= TOLERANCE
                    || bounds.y_size() <= TOLERANCE
                    || bounds.z_size() <= TOLERANCE
                    || points[0][0].is_nan()
                {
                    // degenerate (flat or empty) - the mesh is its own best hull, but don't
                    // cache it
                    self.phase = Phase::Done;
                    return Some(Some(ConvexHull::Mesh(mesh)));
                }
                // Setting the quickhull up copies every point once.
                self.work = self.work.saturating_add(points.len());
                let Some(build) = mi_convex_hull::HullBuild::new(&points, TOLERANCE) else {
                    self.phase = Phase::Done;
                    return Some(None);
                };
                self.phase = Phase::Hull {
                    build: Box::new(build),
                    points,
                };
                None
            }
            Phase::Hull { build, points } => {
                let before = build.work();
                let finished = build.step(work);
                self.work = self.work.saturating_add(build.work() - before);
                let faces = finished?;
                let points = std::mem::take(points);
                self.phase = Phase::Done;
                Some(faces.map(|faces| self.cache(&points, faces)))
            }
            Phase::Done => Some(None),
        }
    }

    /// The hull mesh of `faces` over `points`, cached on the mesh.
    fn cache(&self, points: &[[f64; 3]], faces: Vec<[usize; 3]>) -> ConvexHull<'a> {
        // Create the mesh from the hull data: each hull point becomes one vertex, the first time
        // a face uses it, and every face is made of those vertices, so neighbouring faces share
        // their corners and the hull is a closed solid by construction.
        let mut hull_mesh = Mesh::new();
        let mut vertex_of_point: Vec<Option<i32>> = vec![None; points.len()];
        for face in faces {
            let [a, b, c] = face.map(|point| {
                *vertex_of_point[point].get_or_insert_with(|| {
                    let p = points[point];
                    hull_mesh
                        .vertices
                        .push(Vector3Float::from_vector3(Vector3::new(p[0], p[1], p[2])));
                    hull_mesh.vertices.len() as i32 - 1
                })
            });
            hull_mesh.faces.add(a, b, c, &hull_mesh.vertices);
        }

        let hull = Arc::new(hull_mesh);
        self.mesh
            .convex_hull_cache
            .set(self.changed_count, hull.clone());
        ConvexHull::Hull(hull)
    }
}
