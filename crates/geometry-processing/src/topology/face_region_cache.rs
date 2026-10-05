//! The face regions of one mesh at one tolerance, flooded once each and kept. Ports
//! `Topology/FaceRegionCache.cs`. Each region is a [`FaceGroup`]; picking asks for them at
//! mouse-move rate.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};

use polygon_mesh::{Mesh, MeshEdgeGraph};

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
/// mesh it floods cannot change underneath it (C# asks the caller to promise that).
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

        // Flooded outside the lock, as C# floods outside the ConcurrentDictionary: two threads
        // may both flood, and the first to publish names the region.
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

    fn lock_regions(&self) -> std::sync::MutexGuard<'_, HashMap<i32, Arc<FaceGroup>>> {
        // A poisoned lock only means another thread panicked mid-insert; every entry in the map
        // is a complete region, so the map is still sound to read and extend.
        self.regions.lock().unwrap_or_else(|e| e.into_inner())
    }
}
