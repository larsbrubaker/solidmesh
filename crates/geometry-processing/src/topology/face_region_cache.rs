//! The face regions of one mesh at one tolerance, flooded once each and kept. Each region is a
//! [`FaceGroup`]; picking asks for them at mouse-move rate. A reader that keeps answers per cache
//! (the bevel's chain-end reading) keeps them here, with the cache
//! ([`FaceRegionCache::attached`]).

use std::any::{Any, TypeId};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};

use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::face_group::FaceGroup;

/// The face regions of one mesh at one tolerance, flooded once each and kept.
///
/// A flood is a walk out from a seed face over the whole surface it can reach, and picking asks
/// for them at mouse-move rate: a hover over an edge wants the region on each side of it, and a
/// hover over a face wants that face's region for the reading and the highlight. On a mesh of any
/// size that is the most expensive thing a hover does, and the same question is asked again for
/// every pixel the mouse travels across one flat face.
///
/// Cached against a SNAPSHOT of a mesh: the cache shares ownership of the mesh (`Arc`), so the
/// mesh it floods cannot change underneath it.
///
/// Safe to use from more than one thread. Two threads racing for the same region flood it twice
/// and keep the first answer, which costs a flood and cannot give a wrong one.
#[derive(Debug)]
pub struct FaceRegionCache {
    mesh: Arc<Mesh>,
    graph: MeshEdgeGraph,
    barrier_faces: Option<Vec<i32>>,
    barriers: Option<HashSet<i32>>,
    // Looked up by face only, never iterated, so a HashMap's order cannot leak out.
    regions: Mutex<HashMap<i32, Arc<FaceGroup>>>,
    floods: AtomicI32,
    graph_scans: AtomicI32,
    // One value per type, looked up by type only and never iterated (see `attached`).
    attached: Mutex<HashMap<TypeId, Arc<dyn Any + Send + Sync>>>,
    flat_angle_degrees: f64,
}

impl FaceRegionCache {
    /// A cache over `mesh` with no fence. `graph` is its edge graph, or `None` to build one here.
    /// `flat_angle_degrees` is the pick's flat tolerance, so that "this face" means the same
    /// thing here as everywhere else in a pick.
    pub fn new(mesh: Arc<Mesh>, graph: Option<MeshEdgeGraph>, flat_angle_degrees: f64) -> Self {
        Self::with_barriers(mesh, graph, flat_angle_degrees, None)
    }

    /// A cache whose every region is FENCED against `barrier_faces` - see
    /// [`FaceGroup::create_fenced`]. The fence belongs to the cache rather than to the ask,
    /// because the regions are shared out per face and two asks with different fences would
    /// disagree about which region a face is in. `None` (or empty) is no fence.
    pub fn with_barriers(
        mesh: Arc<Mesh>,
        graph: Option<MeshEdgeGraph>,
        flat_angle_degrees: f64,
        barrier_faces: Option<Vec<i32>>,
    ) -> Self {
        let graph = graph.unwrap_or_else(|| mesh.get_mesh_edge_graph());
        let barriers = barrier_faces
            .as_ref()
            .filter(|b| !b.is_empty())
            .map(|b| b.iter().copied().collect());

        FaceRegionCache {
            mesh,
            graph,
            barrier_faces,
            barriers,
            regions: Mutex::new(HashMap::new()),
            floods: AtomicI32::new(0),
            graph_scans: AtomicI32::new(0),
            attached: Mutex::new(HashMap::new()),
            flat_angle_degrees,
        }
    }

    /// The tolerance every region here was flooded at.
    pub fn flat_angle_degrees(&self) -> f64 {
        self.flat_angle_degrees
    }

    /// The edge graph the regions were flooded over - the topology that goes with this reading of
    /// the mesh, so that a caller holding a cache does not have to rebuild it.
    pub fn graph(&self) -> &MeshEdgeGraph {
        &self.graph
    }

    /// The mesh the regions were flooded over.
    pub fn mesh(&self) -> &Arc<Mesh> {
        &self.mesh
    }

    /// How many floods have actually run - what a test asks to prove that hovering one face twice
    /// costs one walk of the mesh rather than two.
    pub fn flood_count(&self) -> i32 {
        self.floods.load(Ordering::SeqCst)
    }

    /// How many times a reader has scanned EVERY edge of [`Self::graph`] - the other walk of the
    /// whole mesh a plan can make, which a flood count cannot see. A reader that indexes the graph
    /// says so with [`Self::note_graph_scan`]; a test asks, as it asks [`Self::flood_count`], so
    /// that what planning costs is a count that reads the same on any machine and under any load.
    pub fn graph_scan_count(&self) -> i32 {
        self.graph_scans.load(Ordering::SeqCst)
    }

    /// Records one scan of every edge of [`Self::graph`] (see [`Self::graph_scan_count`]).
    pub fn note_graph_scan(&self) {
        self.graph_scans.fetch_add(1, Ordering::SeqCst);
    }

    /// The one `T` kept WITH this cache: made (`T::default()`) on the first ask and handed to
    /// every later one, for as long as the cache lives. Kept on the cache itself, answers read off
    /// a cache go when the cache does, with no global table that outlives it. `T` guards its
    /// own contents (a `Mutex`) when readers on more than one thread fill it.
    pub fn attached<T: Any + Default + Send + Sync>(&self) -> Arc<T> {
        let mut attached = self.attached.lock().unwrap_or_else(|e| e.into_inner());
        let value = attached
            .entry(TypeId::of::<T>())
            .or_insert_with(|| Arc::new(T::default()))
            .clone();
        drop(attached);

        // Stored under T's own TypeId, so the downcast always holds.
        value
            .downcast::<T>()
            .unwrap_or_else(|_| Arc::new(T::default()))
    }

    /// The region a face belongs to, flooded on the first ask and remembered after it - for
    /// EVERY face of it, not just the one asked for. `None` for a face index this mesh does not
    /// have.
    ///
    /// The fill crosses a flat edge both ways, so the faces it reaches are one connected
    /// component and the same component whichever of them is the seed: one flood answers for all
    /// of them, and the one group they share is the answer each of them gets.
    pub fn region(&self, face_index: i32) -> Option<Arc<FaceGroup>> {
        if face_index < 0 || face_index as usize >= self.mesh.faces.len() {
            return None;
        }

        if let Some(already) = self.lock_regions().get(&face_index) {
            return Some(already.clone());
        }

        self.floods.fetch_add(1, Ordering::SeqCst);

        // Flooded outside the lock so a flood never blocks other readers: two threads may both
        // flood, and the first to publish names the region.
        let flooded = FaceGroup::create_fenced(
            &self.mesh,
            &self.graph,
            face_index,
            self.flat_angle_degrees,
            self.barrier_faces.as_deref(),
        )
        .ok()?;

        let mut regions = self.lock_regions();
        let shared = regions
            .entry(face_index)
            .or_insert_with(|| Arc::new(flooded))
            .clone();

        // A region flooded from INSIDE the fence is not the region any of its other faces would
        // flood - the fence is lifted for the seed alone - so that one is not shared out.
        if !self
            .barriers
            .as_ref()
            .is_some_and(|b| b.contains(&face_index))
        {
            for &face in shared.faces() {
                regions.entry(face).or_insert_with(|| shared.clone());
            }
        }

        Some(shared)
    }

    /// Floods now every region of at least `min_faces` faces, so that the first ask for one of
    /// them is a lookup rather than a walk of the region - work for a background build, done once,
    /// so a hover onto a huge smooth surface answers inside a frame. Small regions are left to be
    /// flooded on demand, where a flood costs microseconds. A cache with a fence floods nothing
    /// here: the surface labelling that finds the regions crosses no fence.
    ///
    /// The readings do not change: a region's faces and rim are sorted into mesh order, so the
    /// region a face belongs to is the same whichever of its faces seeded the flood. Only
    /// [`FaceGroup::seed_face`] says which one did - here the region's lowest face - and it shows
    /// nowhere but in the text of a `EdgeDoesNotBoundFaceGroup` error, which names the face the
    /// region was flooded from.
    pub fn flood_regions_of_at_least(&self, min_faces: usize) {
        if self.barriers.is_some() {
            return;
        }
        let labels = FaceGroup::label_surfaces(&self.mesh, &self.graph, self.flat_angle_degrees);
        // Labels are dense from zero in ascending face order, so the first face seen with a label
        // is that region's lowest face.
        let mut first_face: Vec<i32> = Vec::new();
        let mut size: Vec<usize> = Vec::new();
        for (face, &label) in labels.iter().enumerate() {
            let label = label as usize;
            if label >= size.len() {
                size.resize(label + 1, 0);
                first_face.resize(label + 1, face as i32);
            }
            size[label] += 1;
        }
        for (label, &count) in size.iter().enumerate() {
            if count >= min_faces {
                self.region(first_face[label]);
            }
        }
    }

    fn lock_regions(&self) -> std::sync::MutexGuard<'_, HashMap<i32, Arc<FaceGroup>>> {
        // A poisoned lock only means another thread panicked mid-insert; every entry in the map
        // is a complete region, so the map is still sound to read and extend.
        self.regions.lock().unwrap_or_else(|e| e.into_inner())
    }
}
