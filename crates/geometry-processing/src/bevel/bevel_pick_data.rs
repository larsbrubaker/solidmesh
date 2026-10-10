//! One state of one mesh, prepared for picking. Ports `Bevel/BevelPickData.cs`: the welded
//! working copy ([`super::bevel_working_mesh`]), a BVH over it, its edge graph, and the feature
//! edge set of the operation's own bend threshold. The construction plane's face picker
//! (mattercad-model `construction_plane_pick_model`) is built on it; C#'s `BevelPickDataSource`,
//! which keys and publishes it for edge picking, ports with the bevel pick model.

use std::sync::{Arc, OnceLock};

use polygon_mesh::ray_tracer::TraceableRef;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::bevel_working_mesh::{self, StampedVerdict};
use crate::topology::bevel_edge_group_resolver::{self, MatchTolerances};
use crate::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FaceRegionCache, FeatureEdgeSet};

/// How far two flat tolerances may differ and still be the same rung when a stored group is
/// matched to a cached feature edge set. Only wide enough to absorb arithmetic on the tolerances
/// themselves - the resolver refuses a real mismatch, and this must never be the thing that lets
/// one through.
pub const SAME_RUNG_DEGREES: f64 = 1e-6;

/// C# `BevelPickData`. Immutable once built, which is what lets a background build hand it to the
/// UI thread whole and lets anything ask it a question from any thread.
pub struct BevelPickData {
    source_mesh: Arc<Mesh>,
    source_changed_count: i32,
    flat_angle_degrees: f64,
    working_mesh: Arc<Mesh>,
    bvh: Option<TraceableRef>,
    graph: MeshEdgeGraph,
    pick_features: Arc<FeatureEdgeSet>,
    ladder: Vec<Arc<FeatureEdgeSet>>,
    face_regions: OnceLock<FaceRegionCache>,
}

impl std::fmt::Debug for BevelPickData {
    // The BVH is a `dyn Traceable`, which has no `Debug`; the mesh state it was built for says
    // which snapshot this is.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BevelPickData")
            .field("source_changed_count", &self.source_changed_count)
            .field("flat_angle_degrees", &self.flat_angle_degrees)
            .field("working_faces", &self.working_mesh.faces.len())
            .field("has_bvh", &self.bvh.is_some())
            .finish()
    }
}

impl BevelPickData {
    /// C# `Build`: prepares one state of one mesh for picking - a mesh copy, a weld check over it,
    /// a BVH and a feature edge scan. `build_bvh` builds the tree over the working mesh (passed in
    /// because the BVH policy lives with the app, C#'s `m => m.CreateBVHData()`); `None` is C#'s
    /// null hierarchy for a mesh with nothing to trace.
    pub fn build(
        source_mesh: &Arc<Mesh>,
        source_changed_count: i32,
        build_bvh: impl FnOnce(&Mesh) -> Option<TraceableRef>,
        flat_angle_degrees: f64,
    ) -> BevelPickData {
        let working_mesh =
            bevel_working_mesh::create_from(source_mesh, &mut StampedVerdict::default());
        let graph = working_mesh.get_mesh_edge_graph();

        let pick_features = Arc::new(FeatureEdgeSet::create_with_graph(
            &working_mesh,
            &graph,
            flat_angle_degrees,
        ));

        // The threshold's own rung, and nothing else. A finer rung would offer the user a reading
        // made of edges the threshold has just said are not edges of the shape - the cylinder
        // wall's facet joins - and a coarser one would offer a reading whose group could then
        // disagree with the operation about what "this face" is.
        let ladder = vec![Arc::clone(&pick_features)];

        let bvh = build_bvh(&working_mesh);
        BevelPickData {
            source_mesh: Arc::clone(source_mesh),
            source_changed_count,
            flat_angle_degrees,
            working_mesh: Arc::new(working_mesh),
            bvh,
            graph,
            pick_features,
            ladder,
            face_regions: OnceLock::new(),
        }
    }

    /// C# `WorkingMesh`: the welded copy of the source mesh everything about this snapshot is
    /// measured against.
    pub fn working_mesh(&self) -> &Arc<Mesh> {
        &self.working_mesh
    }

    /// C# `Bvh`: the ray tracing tree over [`Self::working_mesh`], in that mesh's own space.
    pub fn bvh(&self) -> Option<&TraceableRef> {
        self.bvh.as_ref()
    }

    /// C# `Graph`: the edge graph of [`Self::working_mesh`].
    pub fn graph(&self) -> &MeshEdgeGraph {
        &self.graph
    }

    /// C# `PickFeatures`: the set a pick runs in, built at the operation's bend threshold.
    pub fn pick_features(&self) -> &Arc<FeatureEdgeSet> {
        &self.pick_features
    }

    /// C# `Ladder`: the feature edge sets the readings of a pick are built from -
    /// [`Self::pick_features`], and only it.
    pub fn ladder(&self) -> &[Arc<FeatureEdgeSet>] {
        &self.ladder
    }

    /// C# `FaceRegions`: the face regions of [`Self::working_mesh`], each flooded once as it is
    /// first asked for. Built lazily, so a part nobody hovers a face on never pays for it.
    pub fn face_regions(&self) -> &FaceRegionCache {
        self.face_regions.get_or_init(|| {
            FaceRegionCache::new(
                Arc::clone(&self.working_mesh),
                Some(self.graph.clone()),
                self.pick_features.flat_angle_degrees(),
            )
        })
    }

    /// C# `Resolve`: one group against this snapshot, re-read at the threshold the snapshot was
    /// built at, so the resolve costs an anchor match rather than a fresh scan of the mesh.
    pub fn resolve(&self, group: &BevelEdgeGroup) -> BevelEdgeGroupResolution {
        bevel_edge_group_resolver::resolve_in(
            &group.read_at(self.flat_angle_degrees),
            &self.working_mesh,
            Some(&self.graph),
            Arc::clone(&self.pick_features),
            MatchTolerances::default(),
        )
    }

    /// C# `IsFor`: whether this was built from a given state of a given mesh. Reference identity
    /// is not enough on its own - a source can be edited in place and come back the same object
    /// with different geometry, which the change count is the counter for - and the tolerance is
    /// in the key as well, because moving the threshold changes which edges are candidates.
    pub fn is_for(&self, mesh: &Arc<Mesh>, changed_count: i32, flat_angle_degrees: f64) -> bool {
        Arc::ptr_eq(&self.source_mesh, mesh)
            && self.source_changed_count == changed_count
            && (self.flat_angle_degrees - flat_angle_degrees).abs() <= SAME_RUNG_DEGREES
    }
}
