//! One state of one mesh, prepared for picking: the welded working copy
//! ([`super::bevel_working_mesh`]), a BVH over it, its edge graph, the feature edge set of the
//! operation's own bend threshold and its face regions. A caller that hovers it readies it first
//! ([`BevelPickData::prepare_for_hover`]: every tangent chain walked, the big regions flooded);
//! a one-off face pick does not. The construction plane's face picker (mattercad-model
//! `construction_plane_pick_model`) is built on it; the source that keys and publishes it for edge
//! picking lives with the bevel pick model.

use std::sync::{Arc, OnceLock};

use polygon_mesh::ray_tracer::TraceableRef;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::bevel_working_mesh::{self, StampedVerdict};
use crate::topology::bevel_edge_group_resolver::{self, MatchTolerances};
use crate::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, FaceRegionCache, FeatureEdgeChain, FeatureEdgeSet,
};

/// How far two flat tolerances may differ and still be the same rung when a stored group is
/// matched to a cached feature edge set. Only wide enough to absorb arithmetic on the tolerances
/// themselves - the resolver refuses a real mismatch, and this must never be the thing that lets
/// one through.
pub const SAME_RUNG_DEGREES: f64 = 1e-6;

/// Regions this big are flooded when the data is readied for hovering
/// ([`BevelPickData::prepare_for_hover`]) rather than on the first hover onto them: a
/// flood of a few thousand faces takes well under a millisecond, and one of a hundred thousand
/// tens of milliseconds, more than a frame.
pub const REGION_FLOODED_UP_FRONT_FACES: usize = 2048;

/// One mesh state ready to pick on. Immutable once built, which is what lets a background build
/// hand it to the UI thread whole and lets anything ask it a question from any thread.
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
    /// Prepares one state of one mesh for picking - a mesh copy, a weld check over it, a BVH and a
    /// feature edge scan. `build_bvh` builds the tree over the working mesh (passed in because the
    /// BVH policy lives with the app); it answers `None` for a mesh with nothing to trace.
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

    /// Readies the data for hovering, on whatever thread built it: walks every tangent chain of
    /// the pick set (so a hover onto each new edge of a long rim reads its chain rather than
    /// walking the whole rim again) and floods every region of at least
    /// [`REGION_FLOODED_UP_FRONT_FACES`] faces (so the first hover onto a smooth surface of a
    /// hundred thousand faces is a lookup). Only for a caller that will hover or list many edges:
    /// the bevel's pick source and the AI's edge list. A one-off face pick (Align to Face, the
    /// construction plane) skips it; at a fine flat tolerance nearly every edge of a curved part
    /// is a feature edge, and walking them all would cost far more than its one answer.
    pub fn prepare_for_hover(&self) {
        self.pick_features
            .prepare_tangent_chains(FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES);
        self.face_regions()
            .flood_regions_of_at_least(REGION_FLOODED_UP_FRONT_FACES);
    }

    /// The welded copy of the source mesh everything about this snapshot is measured against.
    pub fn working_mesh(&self) -> &Arc<Mesh> {
        &self.working_mesh
    }

    /// The ray tracing tree over [`Self::working_mesh`], in that mesh's own space.
    pub fn bvh(&self) -> Option<&TraceableRef> {
        self.bvh.as_ref()
    }

    /// The edge graph of [`Self::working_mesh`].
    pub fn graph(&self) -> &MeshEdgeGraph {
        &self.graph
    }

    /// The set a pick runs in, built at the operation's bend threshold.
    pub fn pick_features(&self) -> &Arc<FeatureEdgeSet> {
        &self.pick_features
    }

    /// The feature edge sets the readings of a pick are built from - [`Self::pick_features`], and
    /// only it.
    pub fn ladder(&self) -> &[Arc<FeatureEdgeSet>] {
        &self.ladder
    }

    /// The face regions of [`Self::working_mesh`], each flooded once as it is first asked for, or
    /// already, for the big ones, by [`Self::prepare_for_hover`]. Built lazily, so data nobody
    /// hovers a face on never pays for it.
    pub fn face_regions(&self) -> &FaceRegionCache {
        self.face_regions.get_or_init(|| {
            FaceRegionCache::new(
                Arc::clone(&self.working_mesh),
                Some(self.graph.clone()),
                self.pick_features.flat_angle_degrees(),
            )
        })
    }

    /// One group against this snapshot, re-read at the threshold the snapshot was built at, so the
    /// resolve costs an anchor match rather than a fresh scan of the mesh.
    pub fn resolve(&self, group: &BevelEdgeGroup) -> BevelEdgeGroupResolution {
        bevel_edge_group_resolver::resolve_in(
            &group.read_at(self.flat_angle_degrees),
            &self.working_mesh,
            Some(&self.graph),
            Arc::clone(&self.pick_features),
            MatchTolerances::default(),
        )
    }

    /// Whether this was built from a given state of a given mesh. Reference identity is not enough
    /// on its own - a source can be edited in place and come back the same object with different
    /// geometry, which the change count is the counter for - and the tolerance is in the key as
    /// well, because moving the threshold changes which edges are candidates.
    pub fn is_for(&self, mesh: &Arc<Mesh>, changed_count: i32, flat_angle_degrees: f64) -> bool {
        Arc::ptr_eq(&self.source_mesh, mesh)
            && self.source_changed_count == changed_count
            && (self.flat_angle_degrees - flat_angle_degrees).abs() <= SAME_RUNG_DEGREES
    }
}
