//! The records of the walk across a CURVED landing surface (`Bevel/Surface/SurfaceRunAcross.cs`):
//! one facet a section crossed ([`SurfaceRunAcrossSectionStep`]), where a walk begins
//! ([`SurfaceRunAcrossSectionStart`]) and the region's facets indexed the two ways the walk asks
//! for them ([`SurfaceRunAcrossRegionMaps`]). C#'s `SurfaceRunAcross` is a static class - how far
//! it is across a curved region from a chain edge, measured ALONG the surface by sections square
//! to the edge - so it lands nothing itself; its walk, and these types' methods, land with the
//! surface package's later slices.

use std::collections::HashMap;
use std::rc::Rc;

use vector_math::Vector3;

/// C# `SurfaceRunAcross.SectionStep` (a readonly struct): one facet a section crossed, as the
/// stretch of the section that lies in it. What the ROLLING construction needs of the same walk
/// the run is measured by: a fillet on a polygonal wall is tangent to the facet it actually
/// touches, so the plan has to know which facets the section passes through, where it enters and
/// leaves each of them, and how far along the surface that is (`SurfaceRollAcrossTheWall`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceRunAcrossSectionStep {
    /// The facet, by index into the mesh.
    pub face: i32,
    /// Where the section entered it - on the chain edge itself for the first facet.
    pub from: Vector3,
    /// Where the section left it, which is on the facet's far edge.
    pub to: Vector3,
    /// The facet's unit normal, read off its vertex POSITIONS (`NormalOf`).
    pub normal: Vector3,
    /// How far along the surface [`SurfaceRunAcrossSectionStep::from`] is from the chain edge.
    pub run_to_entry: f64,
}

/// C# `SurfaceRunAcross.SectionStart` (a private readonly struct): where a walk across a region
/// begins - the facet the chain edge lies in, and the region's maps.
#[derive(Debug, Clone)]
pub struct SurfaceRunAcrossSectionStart {
    /// The region's facet maps, built once per region and shared by every cut of one reading.
    pub maps: Rc<SurfaceRunAcrossRegionMaps>,
    /// The facet the chain edge lies in, by index into the mesh.
    pub face: i32,
    /// The chain edge itself (as its edge key), which the first facet may not be crossed back out
    /// through.
    pub entry: i64,
    /// The first facet's normal, which curvature is judged against.
    pub reference: Vector3,
    /// How near two points have to be to be read as one: the chain edge's length (at least 1)
    /// times the vertex tolerance.
    pub near_enough: f64,
}

/// C# `SurfaceRunAcross.RegionMaps` (a sealed class): the region's facets, indexed the two ways a
/// walk across them asks for them.
///
/// Built over the region's OWN facets only, so an edge with one facet on it IS the region's rim and
/// the walk stops there with nothing else to ask. Built once per region and shared by every cut of
/// one reading, because a rim picked whole lands a dozen cuts on the same region - hence held
/// through `Rc` by [`SurfaceRunAcrossSectionStart`].
#[derive(Debug, Clone, Default)]
pub struct SurfaceRunAcrossRegionMaps {
    /// The region's facets on each of its edges, by edge key: the first facet and the second (-1
    /// where the edge is the region's rim). Looked up only, never iterated into an answer.
    pub by_edge: HashMap<i64, (i32, i32)>,
    /// The region's facets around each of its vertices, by vertex index.
    pub by_vertex: HashMap<i32, Vec<i32>>,
    /// Each of the region's vertices' positions, by vertex index.
    pub positions: HashMap<i32, Vector3>,
    /// The region's facets standing at each corner POSITION, so two vertices welded in space but
    /// not in index still find each other. C# keys this by `Vector3` value equality; here the key
    /// is the position's three coordinates as bits, built with `-0.0` read as `0.0` so the two
    /// agree.
    pub facets_at_corner: HashMap<(u64, u64, u64), Vec<i32>>,
    /// Whether the region lies flat enough that no section across it can read as curved, so the
    /// answer is known without walking one. What that saves is the whole walk: a cut landing on a
    /// six hundred faceted cap crossed a hundred and thirty eight of them to find out that a plane
    /// is flat, once per chain edge (`SurfacePlanCostTests`).
    pub cannot_curve: bool,
    /// Whether the region is manifold within itself, so that a walk across it means anything.
    pub usable: bool,
}

impl SurfaceRunAcrossSectionStep {
    /// How far the section runs inside this facet.
    pub fn crossed(&self) -> f64 {
        (self.to - self.from).length()
    }
}
