//! Splitting the facets a smooth-side boundary crosses (`Bevel/Sections/SectionSmoothSide.Facets.cs`,
//! part of the C# `internal static partial class SectionSmoothSide`; free functions here beside
//! [`super::section_smooth_side`]'s types). A facet's outline rings are read off the mesh
//! ([`the_outline_rings`]), the boundary's pieces across the facet are found ([`the_pieces`]), each
//! piece end is inserted into the ring edge it lies on ([`try_insert_into_the_rings`]), each piece
//! splits the ring holding both its ends ([`try_split`]), and which side of the boundary each ring is
//! is read off direction ([`try_side`]). A piece with holes is bridged to one ring in
//! [`super::section_smooth_side_holes`]; the kept rings are triangulated in
//! [`super::section_smooth_side_triangulate`].

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::section_smooth_side::{normal_of, SmoothSideCut, SmoothSideFacetCut, SmoothSideRefusal};
use super::section_smooth_side_holes::{
    signed_area_of_pool_ring, the_edges_the_crossings_stand_on, try_bridge_the_holes,
    try_place_the_loose_rings,
};
use super::section_smooth_side_triangulate::{
    the_polyline_neighbours, try_triangulate_a_kept_ring,
};
use super::SectionPool;

/// The boundary's own directed edges, `(from, to)`.
pub type BoundaryEdges = IndexSet<(i32, i32)>;

/// A refusal and where on the part it stands (C#'s `out refusal, out where` pair).
pub type RefusedAt = (SmoothSideRefusal, Vector3);

/// C# `TryCutOneFacet`: cuts one facet - its outline ring is read, the boundary vertices that lie ON
/// that ring are inserted into it in order, and each piece of the boundary that runs across the
/// facet splits a ring in two. `Ok(None)` (C#'s true with a null cut) where no piece crosses it.
// `!(area > 0.0)` keeps C#'s reading of a NaN area as not holding material.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_cut_one_facet(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &SectionPool,
    cut: &mut SmoothSideCut,
    facet_faces: &[i32],
    facet: i32,
    visits: &IndexMap<i32, Vec<i32>>,
    smooth_side_is_left: bool,
) -> Result<Option<SmoothSideFacetCut>, RefusedAt> {
    let mut outlines = the_outline_rings(mesh, facet_faces);
    let pieces = the_pieces(cut, facet, visits);
    let mut on_the_outline: IndexSet<i32> = IndexSet::new();

    for piece in &pieces {
        on_the_outline.insert(piece[0]);
        on_the_outline.insert(piece[piece.len() - 1]);
    }

    // A piece with HOLES in it is bridged back to one ring along its own edges - see
    // section_smooth_side_holes - so everything below reads one outline whatever the piece is.
    if let Some(rings) = &outlines {
        if rings.len() > 1 {
            let boundaries: Vec<Vec<Vector3>> = pieces
                .iter()
                .map(|piece| piece.iter().map(|&at| pool.position_of(at)).collect())
                .collect();

            if let Some(one_ring) = try_bridge_the_holes(
                mesh,
                facet_faces,
                rings,
                normal_of(mesh, facet_faces[0]),
                &the_edges_the_crossings_stand_on(graph, cut, &on_the_outline),
                &boundaries,
            ) {
                outlines = Some(vec![one_ring]);
            }
        }
    }

    let outlines = match outlines {
        Some(outlines) if !outlines.is_empty() => outlines,
        _ => {
            let first = mesh.faces[facet_faces[0] as usize].v0;
            cut.rings_of_the_refused_facet.clear();

            return Err((
                SmoothSideRefusal::TheOutlineIsNotOneRing,
                Vector3::from_vector3_float(mesh.vertices[first as usize]),
            ));
        }
    };

    if pieces.is_empty() {
        return Ok(None);
    }

    // A piece with holes that no bridge reached is cut from its rings AS THEY STAND: each crossing
    // goes into whichever ring holds the mesh edge it was placed on - a boundary ringing a bore mouth
    // inside the facet puts every one of its ends on that mouth's ring - and the split below then
    // splits THAT ring rather than the outer one.
    let Some(mut rings) = try_insert_into_the_rings(graph, cut, &outlines, &on_the_outline) else {
        return Err((
            SmoothSideRefusal::APieceEndsOffTheOutline,
            pool.position_of(on_the_outline[0]),
        ));
    };

    for piece in &pieces {
        if !try_split(&mut rings, piece) {
            return Err((
                SmoothSideRefusal::APieceEndsOnNoOneRing,
                pool.position_of(piece[0]),
            ));
        }
    }

    let mut facet_cut = SmoothSideFacetCut {
        normal: normal_of(mesh, facet_faces[0]),
        ..Default::default()
    };
    facet_cut.faces.extend_from_slice(facet_faces);

    let forward = the_boundarys_own_edges(cut);
    let along_the_boundary = the_polyline_neighbours(cut);
    let mut loose: Vec<Vec<i32>> = Vec::new();

    for left in &rings {
        if left.len() < 3 {
            continue;
        }

        let kept = match try_side(left, &forward, smooth_side_is_left, outlines.len() > 1) {
            Ok(RingSide::Untouched) => {
                // A ring the cut never reached says nothing about its own side; which region holds
                // it is read below, by which region's ring encloses it.
                loose.push(left.clone());

                continue;
            }
            Ok(side) => side == RingSide::Kept,
            Err(refused) => {
                let refusal = match refused {
                    RingSideRefusal::BothWays => SmoothSideRefusal::TheBandSideIsNotAgreed,
                    RingSideRefusal::NoBoundaryEdge => SmoothSideRefusal::ARingHasNoBoundaryEdge,
                };

                return Err((refusal, pool.position_of(left[0])));
            }
        };

        if !(signed_area_of_pool_ring(pool, left, facet_cut.normal) > 0.0) {
            // A SIDED REGION IS WOUND THE WAY THE PIECE IS. The outline this split came from holds
            // its material to the left, and chords cut across it leave pieces that do the same, so a
            // region reading zero or negative area is not a piece of the facet at all: it is a ring
            // that SWALLOWED one of the facet's holes, running the hole's rim inside itself in the
            // hole's own direction. On the owner's Many Holes barrel that is the fillet reaching into
            // the bore BESIDE the one being filleted - measured on the mouth at (-111.2, 97.5, -74.9)
            // at 0.5, whose band region came to -5.04 = a strip of about 1.8 less the neighbouring
            // mouth's 6.8 - and a band is never triangulated, so emitting it drops that mouth's 64
            // rim faces and puts nothing back. Refuse by name rather than emit the open rim.
            cut.rings_of_the_refused_facet.clear();
            let mut sizes: Vec<i32> = rings.iter().map(|ring_of| ring_of.len() as i32).collect();
            // C# `OrderByDescending` (stable; equal sizes are indistinguishable anyway).
            sizes.sort_by(|a, b| b.cmp(a));
            cut.rings_of_the_refused_facet.extend(sizes);

            return Err((
                SmoothSideRefusal::TheOutlineIsNotOneRing,
                pool.position_of(left[0]),
            ));
        }

        if kept {
            facet_cut.kept_rings.push(left.clone());
        } else {
            facet_cut.band_rings.push(left.clone());
        }
    }

    // Each untouched ring goes to the region enclosing it, and a kept region that is an outer ring
    // WITH inner rings in it is spliced back to one ring along a visible straight segment per hole -
    // see section_smooth_side_holes - so the ears read one ring here whatever the piece was.
    if !loose.is_empty()
        && !try_place_the_loose_rings(
            pool,
            cut,
            &along_the_boundary,
            facet_cut.normal,
            &mut facet_cut.kept_rings,
            &mut facet_cut.band_rings,
            &loose,
        )
    {
        // The reading the refusal is worth keeping: on the owner's Many Holes barrel every one of
        // these is a flat piece with HOLES in it - see SmoothSideCut::rings_of_the_refused_facet.
        cut.rings_of_the_refused_facet.clear();
        let mut sizes: Vec<i32> = facet_cut
            .kept_rings
            .iter()
            .chain(&facet_cut.band_rings)
            .chain(&loose)
            .map(|ring_of| ring_of.len() as i32)
            .collect();
        sizes.sort_by(|a, b| b.cmp(a));
        cut.rings_of_the_refused_facet.extend(sizes);

        return Err((
            SmoothSideRefusal::TheOutlineIsNotOneRing,
            pool.position_of(loose[0][0]),
        ));
    }

    for kept in &facet_cut.kept_rings {
        if let Err(refusal) = try_triangulate_a_kept_ring(
            pool,
            cut,
            &along_the_boundary,
            kept,
            facet_cut.normal,
            &mut facet_cut.triangles,
        ) {
            return Err((refusal, pool.position_of(kept[0])));
        }
    }

    Ok(Some(facet_cut))
}

/// C# `ThePieces`: the pieces of the boundary that cross one facet - each a maximal run of
/// consecutive boundary vertices standing on it. A run of one is the boundary touching a fold and
/// turning back, which cuts nothing, and is not a piece.
///
/// An OPEN boundary's first and last vertices are ends in their own right, so the runs are found
/// straight through from 0 and the modulo would splice the two end pieces into one that ran the
/// wrong way round the facet. A closed one is walked from a vertex whose predecessor is off the
/// facet, so that no run is cut in half by the list's own start.
pub fn the_pieces(
    cut: &SmoothSideCut,
    facet: i32,
    visits: &IndexMap<i32, Vec<i32>>,
) -> Vec<Vec<i32>> {
    let boundary = &cut.boundary;
    let on: Vec<bool> = boundary
        .iter()
        .map(|v| visits.get(v).is_some_and(|facets| facets.contains(&facet)))
        .collect();
    let mut pieces = Vec::new();
    let mut start: i32 = if cut.boundary_is_closed { -1 } else { 0 };

    let count = boundary.len();
    let mut i = 0;
    while cut.boundary_is_closed && i < count {
        if on[i] && !on[(i + count - 1) % count] {
            start = i as i32;
        }
        i += 1;
    }

    if start < 0 {
        return pieces;
    }

    let mut piece: Option<Vec<i32>> = None;

    for step in 0..count {
        let i = if cut.boundary_is_closed {
            (start as usize + step) % count
        } else {
            step
        };

        if on[i] {
            piece.get_or_insert_with(Vec::new).push(boundary[i]);
        } else if let Some(done) = piece.take() {
            if done.len() > 1 {
                pieces.push(done);
            }
        }
    }

    if let Some(done) = piece {
        if done.len() > 1 {
            pieces.push(done);
        }
    }

    pieces
}

/// The points inserted into one ring edge, each with how far along that edge it stands.
type EdgeInserts = Vec<(f64, i32)>;

/// C# `TryInsertIntoTheRings`: the facet's outline rings with every piece end inserted into the ring
/// edge it lies on, in order along that edge; `None` where an end is on no ring's edge at all, which
/// is not a cut this facet can take.
///
/// Where the piece has holes each end goes into the ONE ring holding the mesh edge it was placed on -
/// a boundary ringing a bore mouth inside the facet puts every one of its ends on that mouth's ring -
/// so the rings come back in the outlines' own order, each carrying the crossings standing on it.
pub fn try_insert_into_the_rings(
    graph: &MeshEdgeGraph,
    cut: &SmoothSideCut,
    outlines: &[Vec<i32>],
    ends: &IndexSet<i32>,
) -> Option<Vec<Vec<i32>>> {
    let mut inserts: Vec<Vec<Option<EdgeInserts>>> = outlines
        .iter()
        .map(|outline| vec![None; outline.len()])
        .collect();

    for &end in ends {
        // C# indexes the dictionary (a throw on a missing place); every boundary vertex has one.
        let place = cut.places.get(&end)?;

        if place.edge < 0 {
            return None;
        }

        let one = graph.get_vertex0(place.edge);
        let other = graph.get_vertex1(place.edge);
        let mut on_ring = -1;
        let mut at = -1;

        for (r, outline) in outlines.iter().enumerate() {
            for i in 0..outline.len() {
                let from = outline[i];
                let to = outline[(i + 1) % outline.len()];

                if (from == one && to == other) || (from == other && to == one) {
                    on_ring = r as i32;
                    at = i as i32;
                }
            }
        }

        if at < 0 {
            return None;
        }

        let (on_ring, at) = (on_ring as usize, at as usize);
        let along = if outlines[on_ring][at] == one {
            place.parameter
        } else {
            1.0 - place.parameter
        };

        inserts[on_ring][at]
            .get_or_insert_with(Vec::new)
            .push((along, end));
    }

    let mut rings = Vec::with_capacity(outlines.len());

    for (r, outline) in outlines.iter().enumerate() {
        let mut ring = Vec::new();

        for (i, &corner) in outline.iter().enumerate() {
            ring.push(corner);

            let Some(here) = &inserts[r][i] else {
                continue;
            };

            // C# `OrderBy` is a STABLE sort, so equal parameters keep their insertion order.
            let mut ordered = here.clone();
            ordered.sort_by(|a, b| a.0.total_cmp(&b.0));

            for (_, vertex) in ordered {
                ring.push(vertex);
            }
        }

        rings.push(ring);
    }

    Some(rings)
}

/// C# `TrySplit`: splits whichever ring holds both ends of `piece` into the two rings that path
/// makes - one running the path forward, one running it backward. Which of those the part keeps is
/// read off the direction afterwards, by [`try_side`].
pub fn try_split(rings: &mut Vec<Vec<i32>>, piece: &[i32]) -> bool {
    let from = piece[0];
    let to = piece[piece.len() - 1];

    for r in 0..rings.len() {
        let ring = &rings[r];
        let a = ring.iter().position(|&v| v == from);
        let b = ring.iter().position(|&v| v == to);

        let (Some(a), Some(b)) = (a, b) else {
            continue;
        };

        if a == b {
            continue;
        }

        let through = &piece[1..piece.len() - 1];
        let mut backward = arc(ring, a, b);
        let mut forward = arc(ring, b, a);

        backward.extend(through.iter().rev());
        forward.extend_from_slice(through);

        rings.remove(r);
        rings.push(backward);
        rings.push(forward);

        return true;
    }

    false
}

/// C# `Arc`: the ring from one index to another, going the way the ring is wound, both ends included.
pub fn arc(ring: &[i32], from: usize, to: usize) -> Vec<i32> {
    let mut arc = Vec::new();
    let mut i = from;

    loop {
        arc.push(ring[i]);

        if i == to {
            return arc;
        }

        i = (i + 1) % ring.len();
    }
}

/// C# `TheBoundarysOwnEdges`: the boundary's own directed edges, in chain order - wrapping only where
/// the boundary is closed.
pub fn the_boundarys_own_edges(cut: &SmoothSideCut) -> BoundaryEdges {
    let mut edges = IndexSet::new();
    let count = cut.boundary.len();
    let last = if cut.boundary_is_closed {
        count as i32
    } else {
        count as i32 - 1
    };

    for i in 0..last.max(0) as usize {
        edges.insert((cut.boundary[i], cut.boundary[(i + 1) % count]));
    }

    edges
}

/// What [`try_side`] read of one ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingSide {
    /// The ring is the part's to keep (`kept` true).
    Kept,
    /// The ring is the band the fillet replaces (`kept` false).
    Band,
    /// The ring carries no boundary edge at all and that is allowed (C#'s `untouched`).
    Untouched,
}

/// Why [`try_side`] could not side a ring (C#'s false return, `kept` telling the two apart).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingSideRefusal {
    /// The ring runs boundary edges both ways (C#'s `kept` true alongside false).
    BothWays,
    /// The ring runs no boundary edge at all (C#'s `kept` false alongside false).
    NoBoundaryEdge,
}

/// C# `TrySide`: which side of the boundary a ring is. The facet's outline is wound counter-clockwise
/// about its normal, so a ring's inside is to the LEFT of every edge it runs. A ring that runs a
/// boundary edge in the boundary's own order therefore holds what stands to the boundary's left,
/// which is the part the fillet keeps when the smooth side is the chain's left, and the band when it
/// is its right.
///
/// `untouched_rings_are_allowed`: whether a ring carrying no boundary edge at all is allowed rather
/// than refused. On a piece WITH HOLES that is what a bore mouth the cut never reached is: the
/// boundary did not separate it, and which region holds it is a question of enclosure, not side.
pub fn try_side(
    ring: &[i32],
    boundary: &BoundaryEdges,
    smooth_side_is_left: bool,
    untouched_rings_are_allowed: bool,
) -> Result<RingSide, RingSideRefusal> {
    let mut runs_forward = false;
    let mut runs_backward = false;

    for i in 0..ring.len() {
        let from = ring[i];
        let to = ring[(i + 1) % ring.len()];

        runs_forward |= boundary.contains(&(from, to));
        runs_backward |= boundary.contains(&(to, from));
    }

    if runs_forward == runs_backward {
        if !runs_forward && untouched_rings_are_allowed {
            return Ok(RingSide::Untouched);
        }

        // Both ways round is a ring the cut did not separate; neither way is a ring with no boundary
        // on it at all, which no ring a split produced can be.
        return Err(if runs_forward {
            RingSideRefusal::BothWays
        } else {
            RingSideRefusal::NoBoundaryEdge
        });
    }

    Ok(if runs_forward == smooth_side_is_left {
        RingSide::Kept
    } else {
        RingSide::Band
    })
}

/// C# `TheOutlineRings`: the boundary rings of one flat piece - its once-used edges, chained up,
/// wound the way its faces are. A disc has ONE; a piece with a hole in it - a bore mouth standing
/// wholly inside the flat piece - has its outer ring and one more per hole. `None` where the piece is
/// not a surface at all (a side drawn twice, or a vertex the outline pinches through).
///
/// This mirrors `SectionFanRemesh`'s own outline walk, which reads the REBUILT faces; this one reads
/// the mesh, because nothing has been rebuilt when the smooth side is cut.
pub fn the_outline_rings(mesh: &Mesh, piece: &[i32]) -> Option<Vec<Vec<i32>>> {
    // Insertion-ordered sets and maps: C#'s HashSet and Dictionary iterate in insertion order here
    // (nothing is removed), and that order decides which ring comes first.
    let mut drawn: IndexSet<(i32, i32)> = IndexSet::new();

    for &face in piece {
        let triple = &mesh.faces[face as usize];

        for side in [
            (triple.v0, triple.v1),
            (triple.v1, triple.v2),
            (triple.v2, triple.v0),
        ] {
            if !drawn.insert(side) {
                return None;
            }
        }
    }

    let mut next: IndexMap<i32, i32> = IndexMap::new();

    for &(from, to) in &drawn {
        if drawn.contains(&(to, from)) {
            continue;
        }

        if next.contains_key(&from) {
            return None;
        }

        next.insert(from, to);
    }

    if next.is_empty() {
        return None;
    }

    let mut rings = Vec::new();
    let mut walked: IndexSet<i32> = IndexSet::new();

    for &start in next.keys() {
        if walked.contains(&start) {
            continue;
        }

        let mut ring = Vec::new();
        let mut here = start;

        // Every vertex of the outline has exactly one once-used edge out of it, so the walk can only
        // stop by coming back to a vertex it has already stood on. Coming back anywhere but the start
        // is a vertex two rings run through - a pinch - and no set of simple rings.
        while walked.insert(here) {
            ring.push(here);
            // Every `to` is some `from` of a closed outline; one that is not is an open outline,
            // where C#'s dictionary lookup would throw - read here as not a surface.
            here = *next.get(&here)?;
        }

        if here != start {
            return None;
        }

        rings.push(ring);
    }

    Some(rings)
}
