//! The one question a chain's free END asks of the mesh that its own two faces cannot answer
//! ([`SurfaceChainEndReading`]): does the crease it stops at carry on past that corner, in an edge
//! nobody picked? Ports the instance fields of `Bevel/Surface/SurfaceChainEndReading.cs`, its
//! constructor and its surface and vertex indexes; the per-cache table (C#'s static
//! `ConditionalWeakTable`, `For`) and the reading itself land with later packages.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use super::SurfaceBevelPlan;
use crate::topology::FaceRegionCache;

/// C# `SurfaceChainEndReading`: whether a chain's end corner is one where the crease carries on.
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
/// A cache may be read from more than one thread, so the two memos sit behind one `Mutex` - C#'s
/// `readingLock`, held across a whole reading. C#'s `graph` field is the cache's own graph
/// (`curveRegions.Graph`), which the Rust cache owns, so it is reached through
/// [`SurfaceChainEndReading::curve_regions`] rather than held twice.
#[derive(Debug)]
pub struct SurfaceChainEndReading {
    /// The CURVE tolerance regions the reading floods, shared with every other reader of the mesh.
    pub curve_regions: Arc<FaceRegionCache>,
    /// The two memos, guarded together (C# `readingLock`).
    pub memos: Mutex<SurfaceChainEndReadingMemos>,
}

/// The two memos a [`SurfaceChainEndReading`] keeps, guarded together by its `Mutex` as C#'s
/// `readingLock` guards them.
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
    /// crease to carry on. Called with the memos locked (C# under `readingLock`).
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
    /// C#'s private constructor (with its field initializers): a reading of `curve_regions` with
    /// both memos empty. The per-cache table that hands one reading out per cache (C#'s `For`
    /// over a static `ConditionalWeakTable`) is not ported yet.
    pub fn new(curve_regions: Arc<FaceRegionCache>) -> Self {
        SurfaceChainEndReading {
            curve_regions,
            memos: Mutex::new(SurfaceChainEndReadingMemos::default()),
        }
    }

    /// The graph edges standing at `vertex`, indexed on the first ask. A chain that never asks -
    /// every closed one - pays nothing for it. Called with the memos locked (C# under
    /// `readingLock`).
    pub fn edges_at<'m>(
        &self,
        memos: &'m mut SurfaceChainEndReadingMemos,
        vertex: i32,
    ) -> &'m [i32] {
        let edges_at_vertex = memos.edges_at_vertex.get_or_insert_with(|| {
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
