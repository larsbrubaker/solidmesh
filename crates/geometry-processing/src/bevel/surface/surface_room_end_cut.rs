//! The notch a chain END cuts into the faces standing across its corner, as the room reads it
//! (`Bevel/Surface/SurfaceRoomEndCut.cs`, a static class: free functions here). Which surfaces
//! stand across a region's rim at a corner lands here first; the end cut itself lands with a later
//! package.

use std::sync::Arc;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::SurfaceBevelPlan;
use crate::topology::{FaceGroup, FaceRegionCache};

/// The surfaces standing on the far side of `region`'s rim at `corner` (within `tolerance`),
/// keyed by region key, less the chain edge's own two (`left`, `right`) and the chain edge itself
/// (`picked_mesh_edge`). In the order the rim names them (C#'s `Dictionary`, so an `IndexMap`).
#[allow(clippy::too_many_arguments)]
pub fn across_the_rim(
    regions: &FaceRegionCache,
    region: &FaceGroup,
    region_key: i32,
    corner: Vector3,
    tolerance: f64,
    picked_mesh_edge: i32,
    left: i32,
    right: i32,
) -> IndexMap<i32, Arc<FaceGroup>> {
    let mut across = IndexMap::new();

    for rim in region.boundary_edges() {
        if rim.mesh_edge_index == picked_mesh_edge
            || ((rim.start - corner).length() > tolerance
                && (rim.end - corner).length() > tolerance)
        {
            continue;
        }

        let (Some((side0, group0)), Some((side1, group1))) = (
            try_key_of(regions, rim.face0),
            try_key_of(regions, rim.face1),
        ) else {
            continue;
        };

        let other = if side0 == region_key { side1 } else { side0 };

        if other != left && other != right {
            across.insert(other, if side0 == region_key { group1 } else { group0 });
        }
    }

    across
}

/// Which region a face belongs to, as `(its key, the region)`, or `None` where the cache has none
/// for it.
pub fn try_key_of(regions: &FaceRegionCache, face_index: i32) -> Option<(i32, Arc<FaceGroup>)> {
    let region = regions.region(face_index)?;
    let key = SurfaceBevelPlan::key_of(&region);

    Some((key, region))
}

/// How near two points have to be to be the same corner of the rim, relative to the chain edge
/// (at least 1).
pub const VERTEX_TOLERANCE: f64 = 1e-5;
