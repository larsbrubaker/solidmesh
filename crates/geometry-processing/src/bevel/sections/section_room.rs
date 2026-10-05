//! How much room a chain's sections have on the part (`Bevel/Sections/SectionRoom.cs`, a C#
//! `internal static class`, so a module here): where one side's boundary goes per unit of radius
//! ([`Boundary`]), a region of the surface as one flood read it ([`Region`]), and everything a room
//! reading knows about the PART rather than about a chain ([`SectionRoomCache`]). Here are the
//! region and crease readings (the flood, its rim and planarity, the reach across a face and the
//! length of a crease) and the room itself ([`try_fit`], [`try_fit_with`], [`try_fit_edges`],
//! with each vertex's two sides read by [`try_read_vertex`] and [`try_side`]).

use std::collections::HashSet;

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, Vector3};

use super::section_builder;
use super::section_fan_walk::{self, EdgeAtPair};
use super::section_room_smooth_side;
use super::section_split_blend;
use crate::bevel::bevel_split_chords;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FeatureEdge, FeatureEdgeChain};

/// C# `SectionRoom.Boundary`: where one side's boundary goes per unit of radius,
/// `V + r * reach * along`, with [`Boundary::largest_that_fits`] the radius at which it runs out
/// of the part it is standing on.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Boundary {
    /// The unit direction the boundary leaves the station along.
    pub along: Vector3,
    /// How far it goes that way per unit of radius.
    pub reach: f64,
    /// The radius at which it reaches the end of its fold or of its fan.
    pub largest_that_fits: f64,
    /// Whether the boundary steps SQUARE off the chain into a flat fan - no fold leaves the
    /// station this side - which is what `SectionSide::Offset` means to the builder.
    pub is_offset: bool,
}

/// C# `SectionRoom.Region`: a region of the surface as one flood read it.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Region {
    /// The edges the flood stopped at.
    pub rim: Vec<i32>,
    /// Whether every face of the region lies in the seed face's plane - the only case in which a
    /// straight ray drawn in that plane is a path along the surface.
    pub is_one_plane: bool,
}

/// C# `SectionRoomCache`: everything a room reading knows about the PART rather than about a
/// chain, so a pass measuring several chains reads it once - the edge table the fan walk is
/// handed, the part's lattice, and the flat regions' rims as they are flooded.
///
/// The C# `Dictionary`s are [`IndexMap`]s here, filled in the C# order, except the pair table,
/// which is only ever looked up and is the fan walk's own [`EdgeAtPair`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SectionRoomCache {
    /// The mesh edge filed under each vertex pair.
    pub edge_at_pair: EdgeAtPair,
    /// The margin every reach limit is pulled back by.
    pub lattice: f64,
    /// How far a reach is ever asked to travel - the part's own bounding diagonal. Nothing on the
    /// part can stand further away, so a walk that spends it whole has met no limit at all.
    pub span: f64,
    /// The lattice a split vertex is read on - `BevelSplitChords.LatticeFor`.
    pub chord_lattice: f64,
    /// The mesh edges touching each vertex, in edge order - what a slide's walk along a crease
    /// steps by, and so a table of the part rather than of a chain.
    pub edges_at_vertex: IndexMap<i32, Vec<i32>>,
    /// Each face's SMOOTH region as flooded with the operation's feature reading - everything
    /// reachable without crossing a feature - filled as the regions are walked. Filed by face
    /// alone, so a cache is one pass's and one feature reading's.
    pub feature_regions_by_face: IndexMap<i32, Region>,
    /// Each face's FLAT region as flooded by coplanarity - the reading a caller with no operation
    /// in hand has to take. Held apart from the feature reading so one pass's two answers cannot
    /// be confused.
    pub coplanar_regions_by_face: IndexMap<i32, Region>,
}

/// One crease's two sides: the normals of the two faces it divides, and whether it divides two at
/// all (false for an open rim, whose normals are then zero).
pub type TwoSides = (Vector3, Vector3, bool);

/// The region and crease readings of the room (C# `PairKey`, `TheTwoSides`, `SameTwoSides`,
/// `IsACrease`, `EveryFaceIsInTheSeedsPlane`, `Region`'s constructor, `FacesOfTheFlatRegion`,
/// `RegionOfTheFace`, `TheRegionIsOnePlane`, `ReachAcrossTheFace`, `LengthOfTheCrease`).
impl Region {
    /// C#'s `Region` constructor: the edges the flood stopped at, and whether every face of the
    /// region lies in the seed face's plane - the only case in which a straight ray drawn in that
    /// plane is a path along the surface.
    pub fn new(rim: Vec<i32>, is_one_plane: bool) -> Self {
        Self { rim, is_one_plane }
    }
}

/// C# `PairKey`: the key `SectionFanWalk.EdgesByVertexPair` files a mesh edge under - the same one,
/// so the flood reads the same table the fan walk was handed.
pub fn pair_key(one: i32, other: i32) -> i64 {
    if one < other {
        ((one as i64) << 32) | (other as u32 as i64)
    } else {
        ((other as i64) << 32) | (one as u32 as i64)
    }
}

/// C# `TheTwoSides`: the normals of the two faces a crease divides, or nothing where it is an open
/// rim.
pub fn the_two_sides(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32) -> TwoSides {
    if graph.get_face_count(edge) != 2 {
        return (Vector3::ZERO, Vector3::ZERO, false);
    }

    (
        face_normal(mesh, graph.get_face(edge, 0)),
        face_normal(mesh, graph.get_face(edge, 1)),
        true,
    )
}

/// C# `SameTwoSides`: whether two creases fold the same two flat sides of the part, in either order.
pub fn same_two_sides(here: TwoSides, there: TwoSides) -> bool {
    if !here.2 || !there.2 {
        return false;
    }

    let flat = FeatureEdge::COPLANAR_FACE_DOT;

    (here.0.dot(there.0) >= flat && here.1.dot(there.1) >= flat)
        || (here.0.dot(there.1) >= flat && here.1.dot(there.0) >= flat)
}

/// C# `IsACrease`: whether a crossed edge is a crease - a fold the part itself has, which a boundary
/// may not pass - read by the OPERATION's own feature set where one is in hand, and by coplanarity
/// only where there is none. Non-manifold either way: an edge with anything but two faces is a rim.
/// The same reading the build's `TryCreasesInTheFan` makes, so the room and the build cannot drift
/// apart.
pub fn is_a_crease(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge: i32,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> bool {
    match is_a_feature_edge {
        None => section_fan_walk::is_a_crease(mesh, graph, edge),
        Some(is_a_feature_edge) => graph.get_face_count(edge) != 2 || is_a_feature_edge(edge),
    }
}

/// C# `EveryFaceIsInTheSeedsPlane`: whether every face of a flooded region lies in the seed face's
/// plane. One fold inside it, however small, and a ray drawn in the seed face's plane is not a path
/// along this surface.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn every_face_is_in_the_seeds_plane<'a>(
    mesh: &Mesh,
    in_the_region: impl IntoIterator<Item = &'a i32>,
    seed: i32,
) -> bool {
    let normal = face_normal(mesh, seed);

    in_the_region
        .into_iter()
        .all(|&member| !(face_normal(mesh, member).dot(normal) < FeatureEdge::COPLANAR_FACE_DOT))
}

/// C# `FacesOfTheFlatRegion`: the FLAT REGION one face stands in - itself and every face reachable
/// from it across edges that are not folds of the part, and, where `held_back` is given, not picked
/// chain edges either (by pair key), so a region never floods over the very crease the fillet is
/// being built on. `rim` is appended with the edges the flood stopped at, where wanted.
/// `is_a_feature_edge` makes this the SMOOTH region rather than the flat one.
///
/// The faces come back as an [`IndexSet`] in the order the flood reached them, which is C#'s
/// `HashSet<int>` enumeration order (nothing is removed).
pub fn faces_of_the_flat_region(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    seed: i32,
    held_back: Option<&HashSet<i64>>,
    mut rim: Option<&mut Vec<i32>>,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> IndexSet<i32> {
    let mut in_the_region = IndexSet::new();
    in_the_region.insert(seed);
    let mut to_walk = vec![seed];

    while let Some(standing) = to_walk.pop() {
        let drawn = &mesh.faces[standing as usize];
        let corners = [drawn.v0, drawn.v1, drawn.v2];

        for i in 0..3 {
            let key = pair_key(corners[i], corners[(i + 1) % 3]);

            let Some(&edge) = edge_at_pair.get(&key) else {
                continue;
            };

            if is_a_crease(mesh, graph, edge, is_a_feature_edge)
                || held_back.is_some_and(|held_back| held_back.contains(&key))
            {
                if let Some(rim) = rim.as_deref_mut() {
                    rim.push(edge);
                }

                continue;
            }

            let beyond = if graph.get_face(edge, 0) == standing {
                graph.get_face(edge, 1)
            } else {
                graph.get_face(edge, 0)
            };

            if in_the_region.insert(beyond) {
                to_walk.push(beyond);
            }
        }
    }

    in_the_region
}

/// C# `RegionOfTheFace`: the region `face` belongs to - its rim, and whether the whole of it lies in
/// one plane - flooded across the edges that are not creases and cached in `regions_by_face` for
/// every face of the region at once.
pub fn region_of_the_face(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    regions_by_face: &mut IndexMap<i32, Region>,
    face: i32,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Region {
    if let Some(known) = regions_by_face.get(&face) {
        return known.clone();
    }

    let mut rim = Vec::new();
    let in_the_region = faces_of_the_flat_region(
        mesh,
        graph,
        edge_at_pair,
        face,
        None,
        Some(&mut rim),
        is_a_feature_edge,
    );
    let region = Region::new(
        rim,
        every_face_is_in_the_seeds_plane(mesh, &in_the_region, face),
    );

    for &member in &in_the_region {
        regions_by_face.insert(member, region.clone());
    }

    region
}

/// C# `TheRegionIsOnePlane`: whether the region `seed` stands in lies wholly in one plane - the same
/// flood and measure [`region_of_the_face`] makes, for readers that want the planarity and not the
/// rim. `answer_by_face` is appended with the answer for EVERY face of the region.
pub fn the_region_is_one_plane(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    seed: i32,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    answer_by_face: Option<&mut IndexMap<i32, bool>>,
) -> bool {
    let in_the_region = faces_of_the_flat_region(
        mesh,
        graph,
        edge_at_pair,
        seed,
        None,
        None,
        is_a_feature_edge,
    );
    let is_one_plane = every_face_is_in_the_seeds_plane(mesh, &in_the_region, seed);

    if let Some(answer_by_face) = answer_by_face {
        for &member in &in_the_region {
            answer_by_face.insert(member, is_one_plane);
        }
    }

    is_one_plane
}

/// C# `ReachAcrossTheFace`: how far the ray from `station` along `away` reaches before it leaves
/// the FLAT REGION the fan stands in - the nearest crossing of one of that region's own rim edges,
/// read in the region's plane. The honest limit is the region's rim rather than the fan's outline,
/// because a fan whose substitution folded it is re-meshed in its own plane. A ray that crosses no
/// rim edge at all leaves the reach infinite (`f64::MAX`).
pub fn reach_across_the_face(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    region: &Region,
    face: i32,
    vertex: i32,
    station: Vector3,
    away: Vector3,
) -> f64 {
    let normal = face_normal(mesh, face);
    let mut nearest = f64::MAX;

    for &edge in &region.rim {
        let one = graph.get_vertex0(edge);
        let other = graph.get_vertex1(edge);

        if one == vertex || other == vertex {
            // The rim edges that touch the station itself cross the ray at its own origin, which is
            // not the ray leaving anything.
            continue;
        }

        let from = position(mesh, one);
        let span = position(mesh, other) - from;
        let divisor = away.cross(span).dot(normal);

        if divisor.abs() < 1e-12 {
            continue;
        }

        let to_the_edge = from - station;
        let along_the_ray = to_the_edge.cross(span).dot(normal) / divisor;
        let along_the_edge = to_the_edge.cross(away).dot(normal) / divisor;

        if along_the_ray > 0.0 && (-1e-9..=1.0 + 1e-9).contains(&along_the_edge) {
            nearest = cs_math::min_f64(nearest, along_the_ray);
        }
    }

    nearest
}

/// C# `LengthOfTheCrease`: how far along `along` from `station` the crease the slide rides on
/// carries on - the fold edge, then onward crease by crease while each one runs on along the same
/// line (within `lattice`) and folds the same two flat sides of the part. What counts as a crease
/// onward is the OPERATION's own feature reading where one is in hand.
#[allow(clippy::too_many_arguments)]
pub fn length_of_the_crease(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edges_at_vertex: &IndexMap<i32, Vec<i32>>,
    fold: i32,
    fold_end: i32,
    station: Vector3,
    along: Vector3,
    lattice: f64,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> f64 {
    let sides = the_two_sides(mesh, graph, fold);
    let mut standing = fold;
    let mut at = fold_end;
    let mut reached = (position(mesh, at) - station).length();

    // Bounded by the edges of the graph, since every step takes a new edge and never returns to one.
    for _ in 0..graph.edge_count() {
        let mut onward = -1;

        let Some(touching) = edges_at_vertex.get(&at) else {
            break;
        };

        for &candidate in touching {
            if candidate == standing || !is_a_crease(mesh, graph, candidate, is_a_feature_edge) {
                continue;
            }

            if onward >= 0 {
                // Two creases onward: whichever way the surface folds here, the part has a corner and
                // the slide has nothing unambiguous to ride past it.
                return reached;
            }

            onward = candidate;
        }

        if onward < 0 {
            return reached;
        }

        let beyond = if graph.get_vertex0(onward) == at {
            graph.get_vertex1(onward)
        } else {
            graph.get_vertex0(onward)
        };
        let to_the_end = position(mesh, beyond) - station;
        let farther = to_the_end.dot(along);

        if farther <= reached
            || (to_the_end - (along * farther)).length() > lattice
            || !same_two_sides(sides, the_two_sides(mesh, graph, onward))
        {
            return reached;
        }

        reached = farther;
        standing = onward;
        at = beyond;
    }

    reached
}

/// A face's stored normal, read as doubles.
fn face_normal(mesh: &Mesh, face: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.faces[face as usize].normal)
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}

/// C# `SmallestWedgeCosine`: how nearly two normals may line up before the vertex has no wedge to draw
/// a section in - the same bound `SectionBuilder` refuses at, because a radius the section rule cannot
/// draw at all is not a question of room.
pub const SMALLEST_WEDGE_COSINE: f64 = -0.999999;

impl Boundary {
    /// C#'s `Boundary` constructor: where one side's boundary goes per unit of radius (`along`,
    /// `reach`), the largest radius that fits it, and whether the side is an offset.
    pub fn new(along: Vector3, reach: f64, largest_that_fits: f64, is_offset: bool) -> Self {
        Self {
            along,
            reach,
            largest_that_fits,
            is_offset,
        }
    }

    /// C# `PerRadius`: where the boundary moves per unit of radius.
    pub fn per_radius(&self) -> Vector3 {
        self.along * self.reach
    }
}

/// C# `TheChordCarriesItsRoom`: whether station `i` is a split vertex the blend will redraw from its
/// neighbours - both sides offsets, standing on the straight chord between them on the blend's own
/// ruler - so that its room is the chord's rather than its own.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn the_chord_carries_its_room(
    stations: &[Vector3],
    left: &[Boundary],
    right: &[Boundary],
    i: usize,
    chord_lattice: f64,
) -> bool {
    if !(chord_lattice > 0.0) {
        return false;
    }

    let count = stations.len();

    section_split_blend::reads_as_a_split_vertex(
        left[i].is_offset,
        right[i].is_offset,
        stations[i],
        stations[(i + count - 1) % count],
        stations[(i + 1) % count],
        chord_lattice,
    )
}

/// C# `KeepsItsDirection`: the largest radius at which the boundary segment from one station's side
/// to the next keeps the chain edge's own direction, or `f64::MAX` where it keeps it at every radius.
/// `(span + r (u_next - u_here)) . d > 0` is linear in `r`: where the bracket's dot is not negative
/// the segment only ever lengthens along the edge, and where it is negative the root is the radius at
/// which it stands still, one step before the strip there folds back on itself.
pub fn keeps_its_direction(
    span: Vector3,
    direction: Vector3,
    here: Boundary,
    next: Boundary,
) -> f64 {
    let closing = -(next.per_radius() - here.per_radius()).dot(direction);

    if closing <= 1e-12 {
        return f64::MAX;
    }

    span.dot(direction) / closing
}

impl SectionRoomCache {
    /// C#'s `SectionRoomCache` constructor, with its initializers (`<init>`: empty region caches):
    /// the mesh's edges by vertex pair and by vertex, its own lattice - eight float steps of its
    /// bounding diagonal - and the ruler the split blend reads a split vertex with.
    pub fn new(mesh: &Mesh, graph: &MeshEdgeGraph) -> Self {
        let span = mesh.get_axis_aligned_bounding_box().size().length();
        let mut edges_at_vertex: IndexMap<i32, Vec<i32>> = IndexMap::new();

        for edge in 0..graph.edge_count() {
            edges_at_vertex
                .entry(graph.get_vertex0(edge))
                .or_default()
                .push(edge);
            edges_at_vertex
                .entry(graph.get_vertex1(edge))
                .or_default()
                .push(edge);
        }

        Self {
            edge_at_pair: section_fan_walk::edges_by_vertex_pair(graph),
            lattice: 8.0 * span * 1.1920929e-7,
            span,
            chord_lattice: bevel_split_chords::lattice_for(mesh),
            edges_at_vertex,
            feature_regions_by_face: IndexMap::new(),
            coplanar_regions_by_face: IndexMap::new(),
        }
    }
}

/// What a room reading answers (C#'s `bool` with `out largestThatFits` and `out failure`): whether
/// `radius` fits, the largest radius that does (read even where the asked one does not), and why
/// not.
pub type SectionRoomFit = (bool, f64, Option<BevelAdvisory>);

/// C# `TryFit(mesh, graph, chain, radius, ..)`: the room of a closed chain, counting creases by
/// coplanarity.
pub fn try_fit(
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    chain: Option<&FeatureEdgeChain>,
    radius: f64,
) -> SectionRoomFit {
    try_fit_with(mesh, graph, chain, radius, None)
}

/// C# `TryFit(mesh, graph, chain, radius, isAFeatureEdge, ..)`: the same, with the operation's own
/// feature reading deciding which crossed edges are creases.
pub fn try_fit_with(
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    chain: Option<&FeatureEdgeChain>,
    radius: f64,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> SectionRoomFit {
    try_fit_edges(
        mesh,
        graph,
        chain.map(|c| c.edges()),
        chain.is_some_and(|c| c.is_closed()),
        radius,
        None,
        is_a_feature_edge,
    )
}

/// C# `TryFit(mesh, graph, edges, isClosed, radius, cache, isAFeatureEdge, ..)`: the largest radius
/// a closed chain's sections have room for - each station's two sides' own limits (a station the
/// chord carries is stepped over), and along the chain each side's boundary keeping its edge's
/// direction - and whether `radius` is within it. `cache` (C#'s nullable `SectionRoomCache`) is
/// built here when not handed in.
pub fn try_fit_edges(
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    edges: Option<&[ChainEdge]>,
    is_closed: bool,
    radius: f64,
    cache: Option<&mut SectionRoomCache>,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> SectionRoomFit {
    let (Some(mesh), Some(graph), Some(edges)) = (mesh, graph, edges) else {
        return (
            false,
            0.0,
            Some(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceConstructionNotBuilt,
                Vec::new(),
            )),
        );
    };
    if !is_closed || edges.len() < 3 {
        return (
            false,
            0.0,
            Some(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceConstructionNotBuilt,
                Vec::new(),
            )),
        );
    }

    let mut built;
    let readings: &mut SectionRoomCache = match cache {
        Some(cache) => cache,
        None => {
            built = SectionRoomCache::new(mesh, graph);
            &mut built
        }
    };
    let lattice = readings.lattice;
    let span = readings.span;
    let chord_lattice = readings.chord_lattice;
    let edge_at_pair = &readings.edge_at_pair;
    let edges_at_vertex = &readings.edges_at_vertex;
    let regions_by_face = if is_a_feature_edge.is_none() {
        &mut readings.coplanar_regions_by_face
    } else {
        &mut readings.feature_regions_by_face
    };

    let count = edges.len();
    let mut stations = vec![Vector3::ZERO; count];
    let mut left = vec![Boundary::default(); count];
    let mut right = vec![Boundary::default(); count];

    let mut least = f64::MAX;
    let mut limiting_at = Vector3::ZERO;
    let mut limiting_code = BevelAdvisoryCode::SizeExceedsFaceWidth;
    let mut take = |limit: f64, at: Vector3, code: BevelAdvisoryCode| {
        if limit < least {
            least = limit;
            limiting_at = at;
            limiting_code = code;
        }
    };

    for i in 0..count {
        let leaving = &edges[i];
        let arriving = &edges[(i + count - 1) % count];

        if arriving.to_vertex() != leaving.from_vertex() {
            return (
                false,
                0.0,
                Some(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceConstructionNotBuilt,
                    vec![leaving.from_vertex().into()],
                )),
            );
        }

        match try_read_vertex(
            mesh,
            graph,
            edge_at_pair,
            arriving,
            leaving,
            lattice,
            regions_by_face,
            edges_at_vertex,
            is_a_feature_edge,
            span,
        ) {
            Ok((station, l, r)) => {
                stations[i] = station;
                left[i] = l;
                right[i] = r;
            }
            Err(failure) => return (false, 0.0, Some(failure)),
        }
    }

    // Which stations the construction does not place on their own.
    let carried: Vec<bool> = (0..count)
        .map(|i| the_chord_carries_its_room(&stations, &left, &right, i, chord_lattice))
        .collect();
    let placed_on_its_own = carried.iter().filter(|&&c| !c).count();

    // Each side's own limit, at the stations that have one.
    for i in 0..count {
        if carried[i] {
            continue;
        }

        take(
            left[i].largest_that_fits,
            stations[i],
            BevelAdvisoryCode::SizeExceedsFaceWidth,
        );
        take(
            right[i].largest_that_fits,
            stations[i],
            BevelAdvisoryCode::SizeExceedsFaceWidth,
        );
    }

    // Along the chain: each side's boundary segment keeps its edge's own direction, a carried
    // station stepped over.
    if placed_on_its_own > 1 {
        for i in 0..count {
            if carried[i] {
                continue;
            }

            let mut next = (i + 1) % count;
            while carried[next] {
                next = (next + 1) % count;
            }

            let direction = edges[i].direction().get_normal();
            let span = stations[next] - stations[i];

            take(
                keeps_its_direction(span, direction, left[i], left[next]),
                stations[i],
                BevelAdvisoryCode::SizeCollidesAcrossFace,
            );
            take(
                keeps_its_direction(span, direction, right[i], right[next]),
                stations[i],
                BevelAdvisoryCode::SizeCollidesAcrossFace,
            );
        }
    }

    let largest_that_fits = cs_math::max_f64(least, 0.0);

    if radius > largest_that_fits {
        let failure = BevelAdvisory::new(
            limiting_code,
            vec![radius.into(), limiting_at.into(), largest_that_fits.into()],
        );
        return (false, largest_that_fits, Some(failure));
    }

    (true, largest_that_fits, None)
}

/// C# `TryReadVertex`: the station at `leaving`'s start and its two sides' boundaries, read
/// across the wedge the two chain edges' averaged face normals make there; refused where the faces
/// are coplanar or fold back.
#[allow(clippy::too_many_arguments)]
pub fn try_read_vertex(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    lattice: f64,
    regions_by_face: &mut IndexMap<i32, Region>,
    edges_at_vertex: &IndexMap<i32, Vec<i32>>,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    asked_reach: f64,
) -> Result<(Vector3, Boundary, Boundary), BevelAdvisory> {
    let station = leaving.from();
    let left_normal = (arriving.left_normal() + leaving.left_normal()).get_normal();
    let right_normal = (arriving.right_normal() + leaving.right_normal()).get_normal();

    let cosine = left_normal.dot(right_normal);
    if cosine > FeatureEdge::COPLANAR_FACE_DOT || cosine < SMALLEST_WEDGE_COSINE {
        let code = if cosine > 0.0 {
            BevelAdvisoryCode::FacesCoplanar
        } else {
            BevelAdvisoryCode::FacesFoldBack
        };
        return Err(BevelAdvisory::new(code, vec![leaving.from_vertex().into()]));
    }

    let wedge = cosine.clamp(-1.0, 1.0).acos();
    let setback = (wedge / 2.0).tan();

    let across = cs_math::max_f64(1.0 - (cosine * cosine), 0.0).sqrt();
    let left_away = ((left_normal * cosine) - right_normal) / across;
    let right_away = ((right_normal * cosine) - left_normal) / across;

    let left = try_side(
        mesh,
        graph,
        edge_at_pair,
        arriving,
        leaving,
        true,
        left_away,
        setback,
        lattice,
        regions_by_face,
        edges_at_vertex,
        is_a_feature_edge,
        asked_reach,
    )?;
    let right = try_side(
        mesh,
        graph,
        edge_at_pair,
        arriving,
        leaving,
        false,
        right_away,
        setback,
        lattice,
        regions_by_face,
        edges_at_vertex,
        is_a_feature_edge,
        asked_reach,
    )?;

    Ok((station, left, right))
}

/// C# `TrySide`: one side's boundary at a station - an OFFSET where the fan crosses no crease (the
/// limit where the reach leaves the part), a SLIDE down the one crease it crosses (the limit where
/// that crease ends); refused at a corner of more than one crease, or a crease that does not lean
/// away.
#[allow(clippy::too_many_arguments)]
pub fn try_side(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    on_the_left: bool,
    away: Vector3,
    setback: f64,
    lattice: f64,
    regions_by_face: &mut IndexMap<i32, Region>,
    edges_at_vertex: &IndexMap<i32, Vec<i32>>,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    asked_reach: f64,
) -> Result<Boundary, BevelAdvisory> {
    let vertex = leaving.from_vertex();
    let station = leaving.from();

    // THE ROOM COUNTS ITS CREASES THE WAY THE BUILD DOES - the same call.
    let (creases, faces) = section_builder::try_creases_in_the_fan_reading(
        vertex,
        arriving,
        leaving,
        on_the_left,
        mesh,
        graph,
        edge_at_pair,
        is_a_feature_edge,
    )?;

    if creases.len() > 1 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        ));
    }

    if creases.is_empty() {
        // OFFSET: the limit is where the reach leaves the part.
        let reach = section_room_smooth_side::reach_of_an_offset_side(
            mesh,
            graph,
            edge_at_pair,
            regions_by_face,
            &faces,
            vertex,
            station,
            away,
            is_a_feature_edge,
            asked_reach,
        );

        return Ok(Boundary::new(
            away,
            setback,
            cs_math::max_f64(reach - lattice, 0.0) / setback,
            true,
        ));
    }

    let fold = creases[0];
    let other = if graph.get_vertex0(fold) == vertex {
        graph.get_vertex1(fold)
    } else {
        graph.get_vertex0(fold)
    };
    let to_the_end = Vector3::from_vector3_float(mesh.vertices[other as usize]) - station;
    let fold_length = to_the_end.length();
    let along = to_the_end / fold_length;
    let lean = along.dot(away);

    let fold_length = length_of_the_crease(
        mesh,
        graph,
        edges_at_vertex,
        fold,
        other,
        station,
        along,
        lattice,
        is_a_feature_edge,
    );

    if lean < 1e-9 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        ));
    }

    // SLIDE: the boundary stands setback / cos(beta) down the fold, and the fold ends at its far
    // vertex.
    Ok(Boundary::new(
        along,
        setback / lean,
        cs_math::max_f64(fold_length - lattice, 0.0) * lean / setback,
        false,
    ))
}
