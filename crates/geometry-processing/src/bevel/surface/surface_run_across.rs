//! The records of the walk across a CURVED landing surface: one facet a section crossed
//! ([`SurfaceRunAcrossSectionStep`]), where a walk begins ([`SurfaceRunAcrossSectionStart`]) and
//! the region's facets indexed the two ways the walk asks for them
//! ([`SurfaceRunAcrossRegionMaps`]). The walk measures how far it is across a curved region from a
//! chain edge, ALONG the surface by sections square to the edge. Here are its constants, the
//! records' constructors and readings, and the mesh helpers; the walk itself is in
//! `surface_run_across_walk.rs`.
//!
//! There is no process-wide memo of a region's maps, which would be shared mutable state across
//! rebuild workers: the caller's cache ([`SurfaceRunAcrossRegionMapsCache`]) is the only memo,
//! keyed by the region's face list.

use std::collections::HashMap;
use std::rc::Rc;

use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::topology::FaceGroup;

/// One facet a section crossed, as the stretch of the section that lies in it. What the ROLLING
/// construction needs of the same walk the run is measured by: a fillet on a polygonal wall is
/// tangent to the facet it actually touches, so the plan has to know which facets the section
/// passes through, where it enters and leaves each of them, and how far along the surface that is
/// (`SurfaceRollAcrossTheWall`).
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

/// Where a walk across a region begins - the facet the chain edge lies in, and the region's maps.
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

/// The region's facets, indexed the two ways a walk across them asks for them.
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
    /// not in index still find each other. The key is the position's three coordinates as bits,
    /// built with `-0.0` read as `0.0` so the two zeros are one position.
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

/// How near flat two facet normals have to be, as a cosine, for a section to read as not curved.
pub const FLAT_COSINE: f64 = 1.0 - 1e-7;

/// How near one plane every facet of a region has to be for no section across it to curve.
pub const NONE_OF_IT_LEANS_COSINE: f64 = 1.0 - 2.5e-8;

/// How near two points have to be to be read as one, relative to the chain edge's length (at
/// least 1).
pub const VERTEX_TOLERANCE: f64 = 1e-5;

/// The shortest step a section takes across a facet.
pub const SHORTEST_STEP: f64 = 1e-9;

/// Where along the chain edge the three sections are taken.
pub const SECTIONS_ALONG_THE_EDGE: [f64; 3] = [0.25, 0.5, 0.75];

/// No edge (a walk entering through a vertex).
pub const NO_EDGE: i64 = -1;

/// The caller's memo of each region's maps, keyed by the region's face list.
pub type SurfaceRunAcrossRegionMapsCache = HashMap<Vec<i32>, Rc<SurfaceRunAcrossRegionMaps>>;

impl SurfaceRunAcrossSectionStep {
    /// One facet a section crossed, with where along the run it was entered.
    pub fn new(face: i32, from: Vector3, to: Vector3, normal: Vector3, run_to_entry: f64) -> Self {
        SurfaceRunAcrossSectionStep {
            face,
            from,
            to,
            normal,
            run_to_entry,
        }
    }
}

impl SurfaceRunAcrossSectionStart {
    /// Where a walk begins.
    pub fn new(
        maps: Rc<SurfaceRunAcrossRegionMaps>,
        face: i32,
        entry: i64,
        reference: Vector3,
        near_enough: f64,
    ) -> Self {
        SurfaceRunAcrossSectionStart {
            maps,
            face,
            entry,
            reference,
            near_enough,
        }
    }
}

/// A position as a hashable key, `-0.0` read as `0.0` so the two zeros are one position.
fn corner_key(position: Vector3) -> (u64, u64, u64) {
    (
        (position.x + 0.0).to_bits(),
        (position.y + 0.0).to_bits(),
        (position.z + 0.0).to_bits(),
    )
}

impl SurfaceRunAcrossRegionMaps {
    /// Indexes `region`'s own facets of `mesh` by edge, by vertex and by corner position; reads
    /// whether every facet lies on one plane (so nothing across it can curve), and stands the
    /// whole reading down (`usable` false) where a third facet of the region shares one of its
    /// edges.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: it refuses a NaN as well.
    pub fn new(mesh: &Mesh, region: &FaceGroup) -> Self {
        let capacity = region.faces().len() * 3;
        let mut by_edge: HashMap<i64, (i32, i32)> = HashMap::with_capacity(capacity);
        let mut by_vertex: HashMap<i32, Vec<i32>> = HashMap::with_capacity(capacity);
        let mut positions: HashMap<i32, Vector3> = HashMap::with_capacity(capacity);
        let mut facets_at_corner: HashMap<(u64, u64, u64), Vec<i32>> =
            HashMap::with_capacity(capacity);
        let mut leaning = false;
        let mut first = Vector3::ZERO;
        let mut usable = true;

        for &face in region.faces() {
            let normal = normal_of(mesh, face);

            if !(normal.length_squared() > 0.0) {
                leaning = true;
            } else if !(first.length_squared() > 0.0) {
                first = normal;
            } else if normal.dot(first) < NONE_OF_IT_LEANS_COSINE {
                leaning = true;
            }

            let vertices = vertices_of(mesh, face);

            for i in 0..3 {
                let (index, position) = vertices[i];
                positions.insert(index, position);

                facets_at_corner
                    .entry(corner_key(position))
                    .or_insert_with(|| Vec::with_capacity(6))
                    .push(face);

                by_vertex
                    .entry(index)
                    .or_insert_with(|| Vec::with_capacity(6))
                    .push(face);

                let key = edge_key(index, vertices[(i + 1) % 3].0);

                match by_edge.get(&key).copied() {
                    None => {
                        by_edge.insert(key, (face, -1));
                    }
                    Some((already, -1)) => {
                        by_edge.insert(key, (already, face));
                    }
                    Some(_) => {
                        // A third facet of this region on one of its own edges: the whole reading
                        // stands down and the straight one answers instead.
                        usable = false;
                    }
                }
            }
        }

        SurfaceRunAcrossRegionMaps {
            by_edge,
            by_vertex,
            positions,
            facets_at_corner,
            cannot_curve: !leaning,
            usable,
        }
    }

    /// A facet standing at both `from` and `to` (by position), or -1.
    pub fn facet_on_both(&self, from: Vector3, to: Vector3) -> i32 {
        let (Some(at_from), Some(at_to)) = (
            self.facets_at_corner.get(&corner_key(from)),
            self.facets_at_corner.get(&corner_key(to)),
        ) else {
            return -1;
        };

        at_from
            .iter()
            .copied()
            .find(|face| at_to.contains(face))
            .unwrap_or(-1)
    }

    /// The region's two facets on `edge`, or none where the edge is the region's rim.
    pub fn facets_of_edge(&self, edge: i64) -> Vec<i32> {
        match self.by_edge.get(&edge) {
            Some(&(first, second)) if second >= 0 => vec![first, second],
            _ => Vec::new(),
        }
    }

    /// The region's facets around `vertex`.
    pub fn facets_of_vertex(&self, vertex: i32) -> &[i32] {
        self.by_vertex.get(&vertex).map_or(&[], Vec::as_slice)
    }

    /// Which end of `edge` (if either) stands within `near_enough` of `point`, or -1.
    pub fn vertex_at(&self, point: Vector3, edge: i64, near_enough: f64) -> i32 {
        if edge == NO_EDGE {
            return -1;
        }

        for vertex in [(edge >> 32) as i32, (edge & 0xFFFF_FFFF) as i32] {
            if let Some(&position) = self.positions.get(&vertex) {
                if (position - point).length() <= near_enough {
                    return vertex;
                }
            }
        }

        -1
    }
}

/// Whether `region` lies flat enough that no section across it can read as curved.
pub fn nothing_across_it_can_curve(
    mesh: &Mesh,
    region: &FaceGroup,
    cache: Option<&mut SurfaceRunAcrossRegionMapsCache>,
) -> bool {
    if region.faces().is_empty() {
        return false;
    }

    maps_of(cache, mesh, region).cannot_curve
}

/// `region`'s maps, from the caller's cache where it has them (and remembered there otherwise).
pub fn maps_of(
    cache: Option<&mut SurfaceRunAcrossRegionMapsCache>,
    mesh: &Mesh,
    region: &FaceGroup,
) -> Rc<SurfaceRunAcrossRegionMaps> {
    match cache {
        Some(cache) => cache
            .entry(region.faces().to_vec())
            .or_insert_with(|| Rc::new(SurfaceRunAcrossRegionMaps::new(mesh, region)))
            .clone(),
        None => Rc::new(SurfaceRunAcrossRegionMaps::new(mesh, region)),
    }
}

/// The key of the edge between vertices `a` and `b`, the lower index first.
pub fn edge_key(a: i32, b: i32) -> i64 {
    if a < b {
        ((a as i64) << 32) | (b as u32 as i64)
    } else {
        ((b as i64) << 32) | (a as u32 as i64)
    }
}

/// `face`'s three vertices, as (index, position).
pub fn vertices_of(mesh: &Mesh, face: i32) -> [(i32, Vector3); 3] {
    let facet = mesh.faces[face as usize];

    [
        (
            facet.v0,
            Vector3::from_vector3_float(mesh.vertices[facet.v0 as usize]),
        ),
        (
            facet.v1,
            Vector3::from_vector3_float(mesh.vertices[facet.v1 as usize]),
        ),
        (
            facet.v2,
            Vector3::from_vector3_float(mesh.vertices[facet.v2 as usize]),
        ),
    ]
}

/// `face`'s unit normal from its vertex positions, or zero for a facet of no area.
pub fn normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let [(_, position0), (_, position1), (_, position2)] = vertices_of(mesh, face);

    let cross = (position1 - position0).cross(position2 - position0);

    if cross.length_squared() > 0.0 {
        cross.get_normal()
    } else {
        Vector3::ZERO
    }
}
