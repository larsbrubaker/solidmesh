//! Cutting the SMOOTH side of a run away from the part: the main module of the smooth-side cut, a
//! set of free functions. What is here is the cut's own types - why a side would not cut
//! ([`SmoothSideRefusal`]), where a boundary vertex lies ([`SmoothSidePlace`]), one cut facet
//! ([`SmoothSideFacetCut`]) and what the whole cut came to ([`SmoothSideCut`]) - the cut itself
//! ([`try_cut_the_smooth_side`]), and the readings of the side the cut is laid on: its faces, its
//! flat pieces and the facets each boundary vertex stands on. The boundary is laid in
//! [`super::section_smooth_side_boundary`] (a junction's rim in
//! `super::section_smooth_side_junction_rim`), the facet split is in
//! `super::section_smooth_side_facets`, the kept rings' triangulation in
//! [`super::section_smooth_side_triangulate`], and a piece with holes is read as one ring in
//! `super::section_smooth_side_holes`.
//!
//! The cut is TOPOLOGICAL, not clipped: a facet is a disc bounded by one outline ring, the boundary
//! enters it at a point ON that ring, runs through points interior to it and leaves at another point
//! ON the ring, and splitting a ring by such a path gives exactly two rings - which is which is read
//! off DIRECTION, never a measurement. Every boundary vertex is one [`SectionPool`] index, so a
//! crossing on a fold is the SAME index in both facets that fold joins, and the two cut facets meet
//! without a T junction.
//!
//! [`SectionPool`]: super::SectionPool

use indexmap::{IndexMap, IndexSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};
use std::collections::VecDeque;

use super::section_fan_walk::{self, pair_key, EdgeAtPair};
use super::section_smooth_side_boundary::{try_lay_the_boundary, SmoothSideFailure};
use super::section_smooth_side_facets::try_cut_one_facet;
use super::{SectionKind, SectionPool, SectionRun};

/// Why a smooth side would not cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SmoothSideRefusal {
    /// It did.
    #[default]
    None = 0,
    /// Both sides of the run are smooth, so the fillet would stand between two cuts and neither
    /// side's fan is the rebuild's to keep. Not a refusal of the cut itself - the cut is never asked -
    /// but it is the same answer to the same question, so it is named alongside the rest.
    BothSidesAreSmooth = 1,
    /// An open run whose two ends are not [`SectionKind::End`] sections: only an end section puts the
    /// run's last boundary point ON a crease of the part, which is what closes the band.
    TheRunIsNotClosed = 2,
    /// An end section's boundary point is not on any chord of the crease it slid down.
    TheEndIsNotOnItsCrease = 3,
    /// No facet of a station's own fan holds the direction its offset sets out in.
    NoFanFaceHoldsTheOffset = 4,
    /// A station's setback ran into a feature of the part before it was spent.
    TheWalkWasStopped = 5,
    /// Two consecutive boundary points are not joined along the surface.
    TheJoinWasRefused = 6,
    /// A facet's once-used edges are not one simple ring, so it is not a disc to cut.
    TheOutlineIsNotOneRing = 7,
    /// The boundary never leaves one facet, so there is no through cut to make.
    TheBoundaryNeverLeavesOneFacet = 8,
    /// A piece of the boundary ends somewhere that is not on the facet's outline.
    APieceEndsOffTheOutline = 9,
    /// No single current ring holds both ends of a piece, so the cut cannot be applied.
    APieceEndsOnNoOneRing = 10,
    /// One ring runs boundary edges both ways, so which side of it is band is not agreed.
    TheBandSideIsNotAgreed = 11,
    /// A ring the cut produced has no boundary edge at all, so it cannot be sided.
    ARingHasNoBoundaryEdge = 12,
    /// A kept ring will not triangulate in its facet's plane.
    ARingWillNotTriangulate = 13,
    /// Lifting a ring's collinear runs left too little of it to triangulate at all.
    ARingIsNothingButACollinearRun = 14,
    /// No triangle of the reduced ring holds the edge a collinear run must be fanned back over.
    ACollinearRunHasNoTriangleToFan = 15,
}

/// Where one boundary vertex lies on the part.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SmoothSidePlace {
    /// The face the point lies strictly inside, or -1 where it lies on a mesh edge.
    pub face: i32,
    /// The mesh edge the point lies on, or -1 where it lies inside a face.
    pub edge: i32,
    /// Where along that edge, 0 at the graph's vertex 0 and 1 at its vertex 1.
    pub parameter: f64,
    /// The mesh VERTEX the point stands on, or -1 where it stands on no corner. A point at a corner
    /// lies on every edge of that vertex's fan, so `edge` there names one fold out of several by
    /// rounding; this names the one thing that is topological. Carried from the walk's end vertex,
    /// never re-derived from a position.
    pub vertex: i32,
}

impl SmoothSidePlace {
    /// A place on `face`; `vertex` is -1 where the place is not on a vertex of the part.
    pub fn new(face: i32, edge: i32, parameter: f64, vertex: i32) -> Self {
        Self {
            face,
            edge,
            parameter,
            vertex,
        }
    }
}

/// One flat piece of the smooth side, cut by the boundary that crosses it.
#[derive(Debug, Clone, Default)]
pub struct SmoothSideFacetCut {
    /// The facet's own mesh faces - every one of them is replaced.
    pub faces: Vec<i32>,
    /// The plane the facet lies in, taken from its first face's corners.
    pub normal: Vector3,
    /// The rings left standing, wound the way the facet's outline is.
    pub kept_rings: Vec<Vec<i32>>,
    /// The rings the fillet replaces - the band between the chain and the boundary.
    pub band_rings: Vec<Vec<i32>>,
    /// The kept rings triangulated in the facet's plane.
    pub triangles: Vec<(i32, i32, i32)>,
}

/// What cutting one smooth side of a run came to.
#[derive(Debug, Clone)]
pub struct SmoothSideCut {
    /// Station `i`'s own boundary points, in walk order - ONE for every ordinary station, and THREE
    /// at a junction, which owns two wall ends on two walls with the fold between them: P1 on the
    /// arriving wall, F1 on the fold, P2 on the leaving wall. This is the map the cut is indexed on;
    /// `station_boundary` is it flattened.
    pub station_points: Vec<Vec<i32>>,
    /// Every station's boundary points in the run's order, flattened out of `station_points` - one
    /// entry per station except at a junction, which contributes its three. The joins and
    /// `crossings_after` are indexed on THIS list.
    pub station_boundary: Vec<i32>,
    /// The fold crossings between station `i`'s boundary point and station `i + 1`'s, in the order
    /// the boundary meets them. The loft's last row stitches through these.
    pub crossings_after: Vec<Vec<i32>>,
    /// The whole boundary, in chain order: B0, its crossings, B1, its crossings, ...
    pub boundary: Vec<i32>,
    /// Whether that boundary is a CYCLE. An open run's is a path: it starts at the first end
    /// section's boundary point and stops at the last's, both of which lie on a crease of the part,
    /// and the band is closed off by those creases rather than by the boundary running back round.
    pub boundary_is_closed: bool,
    /// The faces of the side that station `i`'s own boundary point may be read as lying in - one face
    /// for a walked landing, and for a landing that lies on a crease every face of that crease which
    /// is on this side. This is the reading a join out of that station is offered, and it is kept so
    /// that a join can be laid from outside this module exactly as the cut lays it.
    pub station_faces: Vec<Vec<i32>>,
    /// Where each boundary vertex lies on the part, in the order the vertices were placed.
    pub places: IndexMap<i32, SmoothSidePlace>,
    /// The station boundary points the BUILDER placed along the join between their two neighbours
    /// (`BevelSection::left_boundary_is_join_placed`) rather than walking from their own station.
    /// Carried, never re-derived from positions.
    pub join_placed: IndexSet<i32>,
    /// Where `TheOutlineIsNotOneRing` was answered, the sizes of the rings that facet's outline
    /// actually came to, largest first - empty where the piece was not a surface at all. MEASURED on
    /// the owner's Many Holes barrel (2026-09-17): the facet is 178 faces and its outline is
    /// `[64, 64, 40]` - an outer ring of 40 edges about a patch holding TWO WHOLE BORE MOUTHS of 64
    /// edges as inner rings. The flat piece is legitimately a disc WITH HOLES, and the cut owes it a
    /// bridge from each hole out to the outer ring, so the piece reads as one ring again.
    pub rings_of_the_refused_facet: Vec<i32>,
    /// Every facet the boundary cut.
    pub facets: Vec<SmoothSideFacetCut>,
    /// Every face of every cut facet - all of them come out of the part.
    pub removed_faces: Vec<i32>,
    /// The triangles that replace them.
    pub triangles: Vec<(i32, i32, i32)>,
}

impl Default for SmoothSideCut {
    fn default() -> Self {
        Self {
            station_points: Vec::new(),
            station_boundary: Vec::new(),
            crossings_after: Vec::new(),
            boundary: Vec::new(),
            // A boundary is closed until an open run's cut says otherwise.
            boundary_is_closed: true,
            station_faces: Vec::new(),
            places: IndexMap::new(),
            join_placed: IndexSet::new(),
            rings_of_the_refused_facet: Vec::new(),
            facets: Vec::new(),
            removed_faces: Vec::new(),
            triangles: Vec::new(),
        }
    }
}

impl SmoothSideCut {
    /// The crossings of the join the LOFT's row between `station` and the
    /// next runs through - the join from this station's LAST boundary point to the next station's
    /// FIRST. `None` where the run is open and this is its last station.
    ///
    /// The loft is indexed on STATIONS and `crossings_after` on the FLATTENED boundary, so the two
    /// agree only while every station hands one point. A junction hands three, so this maps the
    /// station onto its group's LAST point and reads the join out of that.
    pub fn crossings_after_station(&self, station: i32) -> Option<&Vec<i32>> {
        let last = self.first_point_of(station) + self.points_at(station) - 1;

        if last < self.crossings_after.len() as i32 {
            Some(&self.crossings_after[last as usize])
        } else {
            None
        }
    }

    /// The crossings of a junction station's OWN rim joins - P1 -> F1 and
    /// F1 -> P2 - in walk order. Empty for an ordinary station, which has one point and so no internal
    /// join.
    ///
    /// These are NOT the loft's to lay: the rim chords are laid as faces by the junction's own
    /// strips, so a loft row through them would lay the same band twice. Their crossings are consumed
    /// by the CUT instead - minted into `places` and carried in `boundary`, which splits the part's
    /// facets - so every crossing is laid somewhere exactly once.
    pub fn internal_crossings_of(&self, station: i32) -> Vec<i32> {
        let first = self.first_point_of(station);
        let mut internal_joins = Vec::new();

        let mut join = first;
        while join < first + self.points_at(station) - 1 && join < self.crossings_after.len() as i32
        {
            internal_joins.extend_from_slice(&self.crossings_after[join as usize]);
            join += 1;
        }

        internal_joins
    }

    /// Where `station`'s first point sits in the FLATTENED boundary.
    fn first_point_of(&self, station: i32) -> i32 {
        let mut at = 0;

        for i in 0..station {
            at += self.points_at(i);
        }

        at
    }

    /// How many boundary points `station` handed. A cut assembled without the station
    /// map reads as one point per station, which is what the flattened boundary then is.
    fn points_at(&self, station: i32) -> i32 {
        if station < self.station_points.len() as i32 {
            self.station_points[station as usize].len() as i32
        } else {
            1
        }
    }
}

/// Lays the smooth side's boundary on the surface and cuts every facet it
/// crosses. The cut comes back whether or not it was made, beside `Ok` or why not, so a caller can
/// read how far a refused cut got.
///
/// `is_a_feature_edge` is the OPERATION's own feature reading - never a coplanarity test, which would
/// call every fold of a tessellated wall a feature and stop the boundary at the first one.
///
/// AN OPEN RUN CUTS TOO (2026-09-17, on the raked bore cube's top mouth arc). The band an open run
/// leaves is closed off at each end by the CREASE that end section's boundary slid down, whose chords
/// are already ring edges of the wall's facets: the end point lies ON one of those chords and is
/// inserted into the ring exactly as a fold crossing is. What the open case changes is only the
/// WRAPPING: the joins stop at the last station, the boundary's own directed edges do not wrap, and a
/// facet's boundary pieces are found without the modulo.
#[allow(clippy::too_many_arguments)]
pub fn try_cut_the_smooth_side(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    run: &SectionRun,
    smooth_side_is_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> (SmoothSideCut, Result<(), SmoothSideFailure>) {
    let mut cut = SmoothSideCut {
        boundary_is_closed: run.is_closed,
        ..Default::default()
    };

    let done = cut_the_smooth_side(
        mesh,
        graph,
        edge_at_pair,
        pool,
        run,
        smooth_side_is_left,
        is_a_feature_edge,
        &mut cut,
    );

    (cut, done)
}

/// The body of [`try_cut_the_smooth_side`], writing into the cut it hands back.
#[allow(clippy::too_many_arguments)]
fn cut_the_smooth_side(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    run: &SectionRun,
    smooth_side_is_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    cut: &mut SmoothSideCut,
) -> Result<(), SmoothSideFailure> {
    if !run.is_closed && !both_ends_are_end_sections(run) {
        return Err(SmoothSideFailure::Refused(
            SmoothSideRefusal::TheRunIsNotClosed,
            Vector3::ZERO,
        ));
    }

    let side = the_sides_faces(
        mesh,
        graph,
        edge_at_pair,
        run,
        smooth_side_is_left,
        is_a_feature_edge,
    );

    let mut facet_of: IndexMap<i32, i32> = IndexMap::new();
    let mut facets: Vec<Vec<i32>> = Vec::new();

    for &face in &side {
        if !facet_of.contains_key(&face) {
            let piece = the_flat_piece(mesh, graph, edge_at_pair, &side, face);

            for &held in &piece {
                facet_of.insert(held, facets.len() as i32);
            }

            facets.push(piece);
        }
    }

    try_lay_the_boundary(
        mesh,
        graph,
        edge_at_pair,
        pool,
        run,
        smooth_side_is_left,
        is_a_feature_edge,
        &side,
        &facet_of,
        cut,
    )?;

    let visits = the_facets_each_vertex_is_on(graph, cut, &facet_of);

    let mut visited: Vec<i32> = visits
        .values()
        .flatten()
        .copied()
        .collect::<IndexSet<i32>>()
        .into_iter()
        .collect();

    if visits.values().all(|on| on.len() == 1) && visited.len() == 1 {
        return Err(SmoothSideFailure::Refused(
            SmoothSideRefusal::TheBoundaryNeverLeavesOneFacet,
            pool.position_of(cut.boundary[0]),
        ));
    }

    visited.sort_unstable();

    for facet in visited {
        let facet_cut = try_cut_one_facet(
            mesh,
            graph,
            pool,
            cut,
            &facets[facet as usize],
            facet,
            &visits,
            smooth_side_is_left,
        )
        .map_err(|(refusal, at)| SmoothSideFailure::Refused(refusal, at))?;

        let Some(facet_cut) = facet_cut else {
            continue;
        };

        cut.removed_faces.extend_from_slice(&facet_cut.faces);
        cut.triangles.extend_from_slice(&facet_cut.triangles);
        cut.facets.push(facet_cut);
    }

    Ok(())
}

/// Whether an open run stops at an [`SectionKind::End`] section at BOTH
/// ends. Only then does the boundary start and stop on a crease of the part, which is what an open
/// band is closed off by.
pub fn both_ends_are_end_sections(run: &SectionRun) -> bool {
    run.sections.len() >= 2
        && run.sections[0].kind == SectionKind::End
        && run.sections[run.sections.len() - 1].kind == SectionKind::End
}

/// The faces of the smooth side - the side face of every chain edge, grown over
/// the part across every edge the operation does NOT call a feature. The features bound the side,
/// so this is the whole of the surface the boundary may be laid on and no more.
pub fn the_sides_faces(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    run: &SectionRun,
    smooth_side_is_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> IndexSet<i32> {
    let seeds: Vec<i32> = run
        .edges
        .iter()
        .map(|edge| {
            if smooth_side_is_left {
                edge.left_face()
            } else {
                edge.right_face()
            }
        })
        .collect();

    flood(mesh, graph, edge_at_pair, &seeds, None, &|edge| {
        is_a_feature_edge.is_some_and(|is_a_feature| is_a_feature(edge))
    })
}

/// One flat piece of the side - the faces around `from` that are coplanar with it.
pub fn the_flat_piece(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    side: &IndexSet<i32>,
    from: i32,
) -> Vec<i32> {
    flood(mesh, graph, edge_at_pair, &[from], Some(side), &|edge| {
        section_fan_walk::is_a_crease(mesh, graph, edge)
    })
    .into_iter()
    .collect()
}

/// The faces reached from `seeds` across every two-faced edge `barred` lets through,
/// staying `within` the given faces where there are any - in the order the flood reached them, so
/// the faces come back the same on every run.
pub fn flood(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    seeds: &[i32],
    within: Option<&IndexSet<i32>>,
    barred: &dyn Fn(i32) -> bool,
) -> IndexSet<i32> {
    let mut reached = IndexSet::new();
    let mut queue = VecDeque::new();

    for &seed in seeds {
        if seed >= 0 && within.is_none_or(|within| within.contains(&seed)) && reached.insert(seed) {
            queue.push_back(seed);
        }
    }

    while let Some(face) = queue.pop_front() {
        for edge in edges_of(mesh, edge_at_pair, face) {
            if graph.get_face_count(edge) != 2 || barred(edge) {
                continue;
            }

            let other = if graph.get_face(edge, 0) == face {
                graph.get_face(edge, 1)
            } else {
                graph.get_face(edge, 0)
            };

            if within.is_none_or(|within| within.contains(&other)) && reached.insert(other) {
                queue.push_back(other);
            }
        }
    }

    reached
}

/// Which facets each boundary vertex stands on - two where it lies on a
/// fold between them. Keyed in the boundary's order.
pub fn the_facets_each_vertex_is_on(
    graph: &MeshEdgeGraph,
    cut: &SmoothSideCut,
    facet_of: &IndexMap<i32, i32>,
) -> IndexMap<i32, Vec<i32>> {
    let mut on = IndexMap::new();

    for &vertex in &cut.boundary {
        // Every boundary vertex is given a place where it is laid, so a missing one is a broken cut
        // and reads as no facet.
        let mut facets = Vec::new();

        if let Some(place) = cut.places.get(&vertex) {
            if place.face >= 0 {
                if let Some(&facet) = facet_of.get(&place.face) {
                    facets.push(facet);
                }
            } else {
                for i in 0..graph.get_face_count(place.edge) {
                    if let Some(&facet) = facet_of.get(&graph.get_face(place.edge, i)) {
                        if !facets.contains(&facet) {
                            facets.push(facet);
                        }
                    }
                }
            }
        }

        on.insert(vertex, facets);
    }

    on
}

/// The graph edges of a face's three sides, in corner order, each looked up by its
/// vertex pair (a side with no edge is skipped).
pub fn edges_of(mesh: &Mesh, edge_at_pair: &EdgeAtPair, face: i32) -> Vec<i32> {
    let corners = corners(mesh, face);
    let mut edges = Vec::with_capacity(3);

    for i in 0..3 {
        if let Some(&edge) = edge_at_pair.get(&pair_key(corners[i], corners[(i + 1) % 3])) {
            edges.push(edge);
        }
    }

    edges
}

/// A face's three corners in its own order.
pub fn corners(mesh: &Mesh, face: i32) -> [i32; 3] {
    let drawn = &mesh.faces[face as usize];
    [drawn.v0, drawn.v1, drawn.v2]
}

/// A face's normal read off its corners (not its stored normal).
pub fn normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let drawn = &mesh.faces[face as usize];
    let a = Vector3::from_vector3_float(mesh.vertices[drawn.v0 as usize]);

    (Vector3::from_vector3_float(mesh.vertices[drawn.v1 as usize]) - a)
        .cross(Vector3::from_vector3_float(mesh.vertices[drawn.v2 as usize]) - a)
        .get_normal()
}
