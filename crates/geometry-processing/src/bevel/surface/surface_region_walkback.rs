//! One face region and the cuts the chain asks of it ([`SurfaceRegionWalkback`]): its constructor
//! and its fields, which the walkback (`SurfaceWalkbackBands`, `SurfaceBandOutline`) fills in as it
//! reads each chain edge reaching the region.

use std::collections::HashSet;
use std::sync::Arc;

use indexmap::IndexMap;

use super::SurfaceFacetCut;
use crate::topology::FaceGroup;

/// One face region and the cuts the chain asks of it.
#[derive(Debug, Clone)]
pub struct SurfaceRegionWalkback {
    /// The region's lowest face index, which is what names it here.
    pub key: i32,
    /// The region itself, which is what its rim is read from.
    pub region: Arc<FaceGroup>,
    /// The faces in it.
    pub faces: HashSet<i32>,
    /// The tangent lines it is cut by, one per chain edge that reaches it.
    pub cuts: Vec<SurfaceFacetCut>,
    /// Which of the region's facets each of [`Self::cuts`] is allowed to take material off, in the
    /// same order - see `SurfaceWalkbackBands`.
    pub reaches: Vec<HashSet<i32>>,
    /// For each of [`Self::cuts`] in the same order, the facets it reaches across a CREASE and the
    /// sibling cut whose own rail lies in each of those facets' planes - the line it draws its
    /// band's far side along there instead of its own (`SurfaceBandOutline`). `None` for a cut with
    /// nothing to borrow, which is every cut of a planar region. An `IndexMap` because a reader may
    /// walk it, and insertion order keeps that walk the same on every run.
    pub rail_borrowed_on: Vec<Option<IndexMap<i32, i32>>>,
}

impl SurfaceRegionWalkback {
    /// The region named by `key`, with its faces gathered and nothing cut yet.
    pub fn new(key: i32, region: Arc<FaceGroup>) -> Self {
        let faces = region.faces().iter().copied().collect();

        SurfaceRegionWalkback {
            key,
            region,
            faces,
            cuts: Vec::new(),
            reaches: Vec::new(),
            rail_borrowed_on: Vec::new(),
        }
    }
}
