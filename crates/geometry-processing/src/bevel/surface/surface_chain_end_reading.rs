//! The one question a chain's free END asks of the mesh that its own two faces cannot answer
//! ([`SurfaceChainEndReading`]): does the crease it stops at carry on past that corner, in an edge
//! nobody picked? The reading's fields, its constructor and its surface and vertex indexes; the
//! reading per cache, whose memos are kept on the cache itself
//! ([`SurfaceChainEndReading::for_regions`], [`FaceRegionCache::attached`]) so they live exactly as
//! long as the cache; and the reading itself
//! ([`SurfaceChainEndReading::runs_out_at`]).

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use super::SurfaceBevelPlan;
use crate::topology::{ChainEdge, FaceRegionCache};

/// Whether a chain's end corner is one where the crease carries on.
///
/// This is what decides what the end of a bevel IS. A cube's picked top edge ends at a corner where
/// the crease genuinely finishes - every other edge there is between a different pair of faces -
/// and the bevel runs out onto the end face, which gives up the section the profile cuts from its
/// corner. One rim facet of a 64 sided cylinder ends at a corner where the rim goes straight on
/// into the next facet, and there is no end face at all. There the bevel has to stop against a wall
/// of its own making, square to the edge, and the band it takes off has to stop there too
/// ([`super::SurfaceBandEnd`]).
///
/// Asked of the CURVE tolerance reading, because that is what makes the two wall facets one surface
/// and so makes the next rim edge the same crease as this one. Asked of the flat reading it would
/// be a different pair of regions and every cylinder rim would look like a cube corner.
///
/// **An object rather than a static, and one per cache rather than one per plan, and both of those
/// are about cost.** Which edges stand at a corner is a scan of every edge in the graph, so it is
/// indexed once instead (lazily: a closed chain never asks). Which surface a face belongs to floods
/// the region, and [`FaceRegionCache`] remembers a flood against the face it was ASKED for rather
/// than against every face in what it found - so a dozen faces of one cylinder wall are a dozen
/// floods of the whole wall. Recording the answer for every face of a region as it comes back turns
/// those into one.
///
/// A cache may be read from more than one thread, so the two memos sit behind one `Mutex`, held
/// across a whole reading, and every reading of one cache shares them: they are kept on the cache
/// ([`FaceRegionCache::attached`]). The graph is the cache's own, so it is reached through
/// [`SurfaceChainEndReading::curve_regions`] rather than held twice.
#[derive(Debug)]
pub struct SurfaceChainEndReading {
    /// The CURVE tolerance regions the reading floods, shared with every other reader of the mesh.
    pub curve_regions: Arc<FaceRegionCache>,
    /// The two memos, guarded together and shared by every reading of
    /// [`Self::curve_regions`].
    pub memos: Arc<Mutex<SurfaceChainEndReadingMemos>>,
}

/// The two memos a [`SurfaceChainEndReading`] keeps, guarded together by its `Mutex` so a reading
/// sees both in step.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceChainEndReadingMemos {
    /// Which surface each face belongs to, by region key, filled a whole region at a time. Only
    /// ever looked up by face, never iterated, so a `HashMap`'s order cannot reach an output.
    pub surface_of_face: HashMap<i32, i32>,
    /// The graph edges standing at each vertex, indexed on the first ask; `None` until then, since
    /// a chain that never asks - every closed one - never pays for the scan. Looked up only.
    pub edges_at_vertex: Option<HashMap<i32, Vec<i32>>>,
}

impl SurfaceChainEndReading {
    /// The two surfaces an edge lies between, as their region keys in a fixed order (low, high),
    /// or `None` where either face has no region or where both are one surface - which is no
    /// crease to carry on. Called with the memos locked.
    pub fn try_surface_pair(
        &self,
        memos: &mut SurfaceChainEndReadingMemos,
        face0: i32,
        face1: i32,
    ) -> Option<(i32, i32)> {
        let low = self.try_surface_of(memos, face0)?;
        let high = self.try_surface_of(memos, face1)?;
        if low == high {
            return None;
        }

        Some((low.min(high), low.max(high)))
    }

    /// Which surface a face belongs to, flooding at most once per surface rather than once per
    /// face.
    pub fn try_surface_of(
        &self,
        memos: &mut SurfaceChainEndReadingMemos,
        face_index: i32,
    ) -> Option<i32> {
        if let Some(&key) = memos.surface_of_face.get(&face_index) {
            return Some(key);
        }

        let region = self.curve_regions.region(face_index)?;
        let key = SurfaceBevelPlan::key_of(&region);

        for &face in region.faces() {
            memos.surface_of_face.insert(face, key);
        }

        Some(key)
    }
}

impl SurfaceChainEndReading {
    /// A reading of `curve_regions`,
    /// with the memos the cache keeps for its readings - empty the first time it is read.
    pub fn new(curve_regions: Arc<FaceRegionCache>) -> Self {
        let memos = curve_regions.attached::<Mutex<SurfaceChainEndReadingMemos>>();
        SurfaceChainEndReading {
            curve_regions,
            memos,
        }
    }

    /// The graph edges standing at `vertex`, indexed on the first ask. A chain that never asks -
    /// every closed one - pays nothing for it. Called with the memos locked.
    pub fn edges_at<'m>(
        &self,
        memos: &'m mut SurfaceChainEndReadingMemos,
        vertex: i32,
    ) -> &'m [i32] {
        let edges_at_vertex = memos.edges_at_vertex.get_or_insert_with(|| {
            // Counted on the cache, so what planning costs reads as a count (SurfacePlanCostTests).
            self.curve_regions.note_graph_scan();
            let graph = self.curve_regions.graph();
            let mut edges_at_vertex = HashMap::with_capacity(graph.edge_count().max(0) as usize);

            for edge in 0..graph.edge_count() {
                Self::record(&mut edges_at_vertex, graph.get_vertex0(edge), edge);
                Self::record(&mut edges_at_vertex, graph.get_vertex1(edge), edge);
            }

            edges_at_vertex
        });

        edges_at_vertex
            .get(&vertex)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    /// Records `edge` as standing at `vertex`.
    pub fn record(edges_at_vertex: &mut HashMap<i32, Vec<i32>>, vertex: i32, edge: i32) {
        edges_at_vertex
            .entry(vertex)
            .or_insert_with(|| Vec::with_capacity(6))
            .push(edge);
    }
}

impl SurfaceChainEndReading {
    /// The reading of `curve_regions`, or `None` where there is no cache. Every reading of a
    /// cache shares the memos the cache keeps ([`FaceRegionCache::attached`]), so they are
    /// filled once per cache however many plans are read against it, and live exactly as long as
    /// the cache, with no process-wide table shared between rebuild workers.
    pub fn for_regions(curve_regions: Option<&Arc<FaceRegionCache>>) -> Option<Self> {
        curve_regions.map(|cache| Self::new(cache.clone()))
    }
}

impl SurfaceChainEndReading {
    /// Whether the chain merely STOPS at `corner_vertex` - another edge stands
    /// there, between the same two surfaces as `chain_edge`, carrying the crease on past the end of
    /// what was picked. Asked with the memos locked for the whole reading; a
    /// lock another reader panicked under still holds memos that are only ever added to, so it is
    /// taken as it stands.
    pub fn runs_out_at(&self, chain_edge: &ChainEdge, corner_vertex: i32) -> bool {
        let mut memos = self
            .memos
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());

        self.asks_the_mesh(&mut memos, chain_edge, corner_vertex)
    }

    /// The reading itself, with the memos locked.
    pub fn asks_the_mesh(
        &self,
        memos: &mut SurfaceChainEndReadingMemos,
        chain_edge: &ChainEdge,
        corner_vertex: i32,
    ) -> bool {
        let Some(picked) =
            self.try_surface_pair(memos, chain_edge.left_face(), chain_edge.right_face())
        else {
            return false;
        };

        // Copied out of the memo, which the surface readings below fill as they go.
        let edges = self.edges_at(memos, corner_vertex).to_vec();
        let graph = self.curve_regions.graph();

        for edge in edges {
            if edge == chain_edge.edge.mesh_edge_index || graph.get_face_count(edge) != 2 {
                continue;
            }

            let carried_on =
                self.try_surface_pair(memos, graph.get_face(edge, 0), graph.get_face(edge, 1));

            if carried_on == Some(picked) {
                return true;
            }
        }

        false
    }
}
