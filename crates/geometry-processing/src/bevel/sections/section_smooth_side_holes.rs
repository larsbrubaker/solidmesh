//! A flat piece WITH HOLES read as one ring: free functions beside
//! [`super::section_smooth_side_facets`], which calls them. Two ways in: every inner ring BRIDGED
//! out to the outer one along the piece's OWN mesh edges ([`try_bridge_the_holes`]), and - where
//! the cut's own boundary bars every such edge, as it does round an enclosed bore mouth - each
//! UNTOUCHED ring handed to the region enclosing it and a kept region spliced back to one ring
//! along a VISIBLE STRAIGHT SEGMENT of the facet's plane ([`try_place_the_loose_rings`]).
//!
//! Either bridge adds no vertex and no geometry: it is a zero width slit whose two directed edges
//! cancel, so the ring encloses exactly the area the piece does and every triangle the ears cut is a
//! triangle of the piece. Both are chosen by TOPOLOGY - lowest vertex index first at every tie - so
//! the same mesh always bridges the same way.

use std::collections::VecDeque;

use indexmap::{IndexMap, IndexSet};
use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_smooth_side::SmoothSideCut;
use super::section_smooth_side_triangulate::{
    the_ring_without_its_collinear_runs, PolylineNeighbours,
};
use super::SectionPool;
use crate::bevel::surface::surface_ring_ears::they_cross_in;
use crate::bevel::surface::SurfaceVertexPool;

/// Undirected mesh edges as sorted vertex pairs.
pub type VertexPairs = IndexSet<(i32, i32)>;

/// The facet's outline rings - one outer, the rest holes - read as ONE ring by
/// bridging every hole out to the outer ring along the piece's own edges, the bridge run there and
/// back; `None` where the rings are not one outer with holes, or a hole cannot be reached.
///
/// The edges a boundary crossing stands on (`barred`) are barred to the bridge: a crossing is inserted
/// into ONE position of the ring, and a bridge edge stands in the ring twice. So are the edges that
/// cross a wall of the piece - the facet's outline rings and the open `boundaries` the cut runs across
/// the facet ([`bar_the_edges_that_cross_the_outline`]). A bore mouth standing WHOLLY inside a facet is
/// ringed by the cut's own setback boundary, so every edge radiating from it is barred and no bridge
/// is found here; that piece is answered by [`try_place_the_loose_rings`] instead.
pub fn try_bridge_the_holes(
    mesh: &Mesh,
    facet_faces: &[i32],
    outlines: &[Vec<i32>],
    normal: Vector3,
    barred: &VertexPairs,
    boundaries: &[Vec<Vector3>],
) -> Option<Vec<i32>> {
    let mut outer = -1;

    for (i, outline) in outlines.iter().enumerate() {
        if signed_area_of_mesh_ring(mesh, outline, normal) > 0.0 {
            if outer >= 0 {
                // Two rings that both hold material to their left are two pieces, not a piece with a
                // hole, and nothing bridges them.
                return None;
            }

            outer = i as i32;
        }
    }

    if outer < 0 {
        return None;
    }

    let outer = outer as usize;

    // A bridge may only run where the piece IS, and the piece is what the CUT leaves: the facet's own
    // outline is one wall of it and the boundary running across the facet is the other. The facet's
    // edge graph reaches the facet's original corners, which stand outside the cut piece, and the
    // shortest path in edge count is happy to leave through the outline and come back - a slit drawn
    // across the boundary, which is no slit of the piece at all and is what the ears then read as a
    // crossing.
    let mut off_the_piece = barred.clone();

    bar_the_edges_that_cross_the_outline(
        mesh,
        facet_faces,
        outlines,
        boundaries,
        normal,
        &mut off_the_piece,
    );

    let neighbours = the_edges_of_the_piece(mesh, facet_faces, &off_the_piece);
    let mut ring = outlines[outer].clone();

    for (i, hole) in outlines.iter().enumerate() {
        if i == outer {
            continue;
        }

        let path = try_the_shortest_bridge(&neighbours, &ring, hole)?;

        splice(&mut ring, hole, &path);
    }

    Some(ring)
}

/// The mesh edges the boundary's ends on this facet stand on, as
/// sorted vertex pairs - the edges no bridge may run along, because a crossing is inserted into one
/// position of the ring.
pub fn the_edges_the_crossings_stand_on(
    graph: &MeshEdgeGraph,
    cut: &SmoothSideCut,
    ends: &IndexSet<i32>,
) -> VertexPairs {
    let mut barred = IndexSet::new();

    for end in ends {
        let Some(place) = cut.places.get(end) else {
            continue;
        };

        if place.edge < 0 {
            continue;
        }

        let one = graph.get_vertex0(place.edge);
        let other = graph.get_vertex1(place.edge);

        barred.insert((one.min(other), one.max(other)));
    }

    barred
}

/// Bars every edge of the facet that CROSSES a wall of the piece,
/// so no bridge can be drawn through one. The walls are the facet's outline rings - outer and inner
/// alike - and the open chains the CUT runs across the facet on, which the outline knows nothing of
/// yet and which a facet corner's shortest path to a bore mouth walks straight through. An edge that
/// merely ENDS on the outline shares a point with the sides there rather than straddling them, so it
/// is not read as crossing.
pub fn bar_the_edges_that_cross_the_outline(
    mesh: &Mesh,
    facet_faces: &[i32],
    outlines: &[Vec<i32>],
    boundaries: &[Vec<Vector3>],
    normal: Vector3,
    barred: &mut VertexPairs,
) {
    // The PIECE's ruler, not the whole mesh's: the ears read this same outline on its own extent, and
    // a ruler taken off a part ten times bigger reads a shallow crossing as a touch.
    let outline_points: Vec<Vector3Float> = outlines
        .iter()
        .flatten()
        .map(|&at| mesh.vertices[at as usize])
        .collect();
    let near_enough = SurfaceVertexPool::near_enough_for(&outline_points);
    let mut edges: IndexSet<(i32, i32)> = IndexSet::new();

    for &face in facet_faces {
        let triple = &mesh.faces[face as usize];

        for (from, to) in [
            (triple.v0, triple.v1),
            (triple.v1, triple.v2),
            (triple.v2, triple.v0),
        ] {
            edges.insert((from.min(to), from.max(to)));
        }
    }

    let rings: Vec<Vec<Vector3>> = outlines
        .iter()
        .map(|outline| {
            outline
                .iter()
                .map(|&at| Vector3::from_vector3_float(mesh.vertices[at as usize]))
                .collect()
        })
        .collect();
    let walls: Vec<(&[Vector3], bool)> = rings
        .iter()
        .map(|points| (points.as_slice(), true))
        .chain(boundaries.iter().map(|chain| (chain.as_slice(), false)))
        .collect();

    for edge in edges {
        if barred.contains(&edge) {
            continue;
        }

        let from = Vector3::from_vector3_float(mesh.vertices[edge.0 as usize]);
        let to = Vector3::from_vector3_float(mesh.vertices[edge.1 as usize]);

        'walls: for &(points, closed) in &walls {
            let sides = if closed {
                points.len() as i32
            } else {
                points.len() as i32 - 1
            };

            for side in 0..sides.max(0) as usize {
                let at = points[side];
                let next = points[(side + 1) % points.len()];

                if they_cross_in(normal, from, to, at, next, near_enough)
                    || it_stands_on_the_interior(at, from, to, near_enough)
                    || (!closed
                        && side as i32 == sides - 1
                        && it_stands_on_the_interior(next, from, to, near_enough))
                {
                    barred.insert(edge);

                    break 'walls;
                }
            }
        }
    }
}

/// Whether a wall point stands ON the INTERIOR of an edge - near the
/// segment, and near neither of its two ends. Such an edge is barred exactly as a crossing bars one.
///
/// A wall that passes from one side of an edge to the other AT ONE OF ITS OWN VERTICES straddles that
/// edge with neither of the two sides meeting there, so the crossing test reads a touch on each side
/// and lets the edge through. Reading the POINT rather than the two sides closes that hole.
pub fn it_stands_on_the_interior(
    at: Vector3,
    from: Vector3,
    to: Vector3,
    near_enough: f64,
) -> bool {
    let along = to - from;
    let length = along.length();

    if length <= near_enough {
        return false;
    }

    let how_far = (at - from).dot(along) / (length * length);

    if how_far * length <= near_enough || (1.0 - how_far) * length <= near_enough {
        return false;
    }

    (at - (from + (along * how_far))).length() <= near_enough
}

/// The piece's own undirected edges as an adjacency, with the `barred` edges
/// left out. Neighbour lists are sorted, so a walk that takes the lowest index first is the same every
/// run.
pub fn the_edges_of_the_piece(
    mesh: &Mesh,
    facet_faces: &[i32],
    barred: &VertexPairs,
) -> IndexMap<i32, Vec<i32>> {
    let mut neighbours: IndexMap<i32, IndexSet<i32>> = IndexMap::new();

    let mut join = |from: i32, to: i32| {
        if barred.contains(&(from.min(to), from.max(to))) {
            return;
        }

        neighbours.entry(from).or_default().insert(to);
    };

    for &face in facet_faces {
        let triple = &mesh.faces[face as usize];

        for (from, to) in [
            (triple.v0, triple.v1),
            (triple.v1, triple.v2),
            (triple.v2, triple.v0),
        ] {
            join(from, to);
            join(to, from);
        }
    }

    neighbours
        .into_iter()
        .map(|(at, held)| {
            let mut sorted: Vec<i32> = held.into_iter().collect();
            sorted.sort_unstable();
            (at, sorted)
        })
        .collect()
}

/// The shortest path in edge count from `ring` - what has been bridged so
/// far - to `hole`, given outer end first. Ties go to the lowest vertex index, at the end it reaches
/// and at every step back, so the bridge is a property of the mesh and not of the walk.
pub fn try_the_shortest_bridge(
    neighbours: &IndexMap<i32, Vec<i32>>,
    ring: &[i32],
    hole: &[i32],
) -> Option<Vec<i32>> {
    let on_the_ring: IndexSet<i32> = ring.iter().copied().collect();
    let in_the_hole: IndexSet<i32> = hole.iter().copied().collect();
    let mut steps: IndexMap<i32, i32> = IndexMap::new();
    let mut queue = VecDeque::new();

    let mut starts: Vec<i32> = on_the_ring.iter().copied().collect();
    starts.sort_unstable();

    for at in starts {
        steps.insert(at, 0);
        queue.push_back(at);
    }

    let mut reached = -1;
    let mut reached_at = i32::MAX;

    while let Some(here) = queue.pop_front() {
        let here_steps = steps[&here];

        if in_the_hole.contains(&here)
            && (reached < 0
                || here_steps < reached_at
                || (here_steps == reached_at && here < reached))
        {
            reached = here;
            reached_at = here_steps;

            continue;
        }

        if reached >= 0 && here_steps >= reached_at {
            break;
        }

        let Some(next) = neighbours.get(&here) else {
            continue;
        };

        for &there in next {
            if !steps.contains_key(&there) {
                steps.insert(there, here_steps + 1);
                queue.push_back(there);
            }
        }
    }

    if reached < 0 {
        return None;
    }

    // Walked back the way it came: from the hole out to the ring, each step to the lowest indexed
    // neighbour one step nearer, which is what makes the path itself tie-free.
    let mut back = vec![reached];
    let mut walk = reached;

    while steps[&walk] > 0 {
        let wanted = steps[&walk] - 1;
        // A vertex `steps` reached at n > 0 was reached FROM a neighbour at n - 1, so one is always
        // found, and its neighbour lists are symmetric.
        let nearer = neighbours
            .get(&walk)?
            .iter()
            .copied()
            .filter(|there| steps.get(there).is_some_and(|&was| was == wanted))
            .min()?;

        back.push(nearer);
        walk = nearer;
    }

    if !on_the_ring.contains(&walk) {
        return None;
    }

    back.reverse();

    Some(back)
}

/// Slits the ring open at the bridge's outer end and threads the hole onto it - out along
/// the bridge, once round the hole its own way, and back along the bridge. The two ends of the bridge
/// each stand in the ring twice afterwards, which is what a zero width slit is.
pub fn splice(ring: &mut Vec<i32>, hole: &[i32], path: &[i32]) {
    // A missing end would insert at the ring's start; both ends are always found (the path was
    // walked from the ring to the hole).
    let at = ring
        .iter()
        .position(|&v| v == path[0])
        .map_or(0, |at| at + 1);
    let inner = hole
        .iter()
        .position(|&v| v == path[path.len() - 1])
        .unwrap_or(usize::MAX);
    let inner = if inner == usize::MAX {
        -1
    } else {
        inner as i64
    };
    let mut threaded = Vec::new();

    // The bridge outward, its outer end already standing in the ring at "at".
    threaded.extend_from_slice(&path[1..]);

    // Once round the hole, in the hole's own winding, ending where it started.
    for step in 1..=hole.len() as i64 {
        threaded.push(hole[((inner + step) % hole.len() as i64) as usize]);
    }

    // And back out, leaving the ring to carry on from the vertex it was slit at.
    threaded.extend(path.iter().rev().skip(1));

    ring.splice(at..at, threaded);
}

/// Hands every UNTOUCHED ring of a facet - a bore mouth the cut never
/// reached - to the region whose ring encloses it, and splices the kept regions that gained one back
/// to a single ring; false where a loose ring holds material of its own or no region encloses it.
///
/// Side cannot answer which region holds a hole - it carries no boundary edge - so it is read by
/// enclosure: the smallest region ring that contains the hole's own first point takes it. A band
/// region simply carries its hole (a band ring is never triangulated); a kept one is spliced by
/// [`try_splice_the_kept_region`] so the ears see one ring.
// `!(area > 0.0)` reads a NaN area as not holding material.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_place_the_loose_rings(
    pool: &SectionPool,
    cut: &SmoothSideCut,
    along_the_boundary: &PolylineNeighbours,
    normal: Vector3,
    kept: &mut [Vec<i32>],
    band: &mut Vec<Vec<i32>>,
    loose: &[Vec<i32>],
) -> bool {
    let mut holes_of: IndexMap<usize, Vec<Vec<i32>>> = IndexMap::new();

    for ring in loose {
        if signed_area_of_pool_ring(pool, ring, normal) > 0.0 {
            // A ring holding material of its own is a second PIECE, not a hole in this one, and the
            // facet is not one region the cut can answer for.
            return false;
        }

        let at = pool.position_of(ring[0]);
        let mut holder = -1;
        let mut smallest = f64::MAX;

        for i in 0..kept.len() + band.len() {
            let region = if i < kept.len() {
                &kept[i]
            } else {
                &band[i - kept.len()]
            };
            let area = signed_area_of_pool_ring(pool, region, normal);

            if !(area > 0.0) || area >= smallest || !it_stands_inside(pool, region, at, normal) {
                continue;
            }

            holder = i as i32;
            smallest = area;
        }

        if holder < 0 {
            return false;
        }

        let holder = holder as usize;

        if holder >= kept.len() {
            band.push(ring.clone());

            continue;
        }

        holes_of.entry(holder).or_default().push(ring.clone());
    }

    for (holder, holes) in holes_of {
        let mut region = vec![kept[holder].clone()];

        region.extend(holes);

        let Some(one) = try_splice_the_kept_region(pool, cut, along_the_boundary, &region, normal)
        else {
            return false;
        };

        kept[holder] = one;
    }

    true
}

/// Whether a point of the facet's plane stands inside a ring - the ring's winding
/// about it, counted by the angle it turns through, is a whole turn rather than none. The angle sum
/// needs no axis chosen in the plane and no special case for a ray grazing a vertex.
// `!(x > 0.0)` reads a NaN length as nothing to turn through.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn it_stands_inside(pool: &SectionPool, ring: &[i32], at: Vector3, normal: Vector3) -> bool {
    let facing = normal.get_normal();
    let mut turned = 0.0;

    for i in 0..ring.len() {
        let from = pool.position_of(ring[i]) - at;
        let to = pool.position_of(ring[(i + 1) % ring.len()]) - at;

        if !(from.length_squared() > 0.0) || !(to.length_squared() > 0.0) {
            continue;
        }

        turned += from.cross(to).dot(facing).atan2(from.dot(to));
    }

    f64::abs(turned) > std::f64::consts::PI
}

/// A KEPT REGION - an outer ring with inner rings standing in it - taken
/// back to ONE ring by splicing each inner ring into the outer along a VISIBLE STRAIGHT SEGMENT of the
/// facet's plane: a zero width slit drawn where no mesh edge can reach.
///
/// Deterministic by TOPOLOGY first: the inner end is the ring's lowest pool index, and the outer end
/// is the lowest pool index vertex of the ring merged so far whose segment to it crosses no edge of
/// any ring still to be joined ([`the_segment_is_visible`]). Inner rings go in lowest-index order,
/// each against the ring merged so far, so a second hole may slit to the first hole's own vertices.
pub fn try_splice_the_kept_region(
    pool: &SectionPool,
    cut: &SmoothSideCut,
    along_the_boundary: &PolylineNeighbours,
    region: &[Vec<i32>],
    normal: Vector3,
) -> Option<Vec<i32>> {
    let mut outer = -1;

    for (i, ring) in region.iter().enumerate() {
        if signed_area_of_pool_ring(pool, ring, normal) > 0.0 {
            if outer >= 0 {
                // Two rings that both hold material are two regions, not a region with holes.
                return None;
            }

            outer = i as i32;
        }
    }

    if outer < 0 {
        return None;
    }

    let outer = outer as usize;

    let points: Vec<Vector3Float> = region
        .iter()
        .flatten()
        .map(|&at| Vector3Float::from_vector3(pool.position_of(at)))
        .collect();
    let near_enough = SurfaceVertexPool::near_enough_for(&points);

    let mut holes: Vec<&Vec<i32>> = region
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != outer)
        .map(|(_, ring)| ring)
        .collect();
    // Holes in order of their lowest vertex; `sort_by_key` is stable, so ties keep their order.
    holes.sort_by_key(|ring| ring.iter().copied().min().unwrap_or(i32::MAX));

    let mut merged = region[outer].clone();

    for h in 0..holes.len() {
        let hole = holes[h];

        // The walls are the rings AS THE EARS WILL SEE THEM - with every collinear run lifted out and
        // spanned by its chord - and the slit's two ends are taken from those same reduced rings. A
        // slit drawn to a lifted point, or across a chord, is one the ears then refuse.
        let mut walls =
            vec![the_ring_without_its_collinear_runs(cut, along_the_boundary, &merged).0];

        for other in &holes[h..] {
            walls.push(the_ring_without_its_collinear_runs(cut, along_the_boundary, other).0);
        }

        // A ring of a region is never empty, so it always has a lowest vertex.
        let inner = walls[1].iter().copied().min()?;

        let mut candidates: Vec<i32> = walls[0]
            .iter()
            .copied()
            .collect::<IndexSet<i32>>()
            .into_iter()
            .collect();
        candidates.sort_unstable();

        let outer_end = candidates
            .into_iter()
            .find(|&at| the_segment_is_visible(pool, &walls, at, inner, normal, near_enough))?;

        splice(&mut merged, hole, &[outer_end, inner]);
    }

    Some(merged)
}

/// Whether the straight segment between two ring vertices runs clear of
/// every wall - crossing no side of any of them, and with no wall vertex other than its own two ends
/// standing on its interior.
pub fn the_segment_is_visible(
    pool: &SectionPool,
    walls: &[Vec<i32>],
    outer_end: i32,
    inner: i32,
    normal: Vector3,
    near_enough: f64,
) -> bool {
    let from = pool.position_of(outer_end);
    let to = pool.position_of(inner);

    for wall in walls {
        for i in 0..wall.len() {
            let at = wall[i];
            let next = wall[(i + 1) % wall.len()];

            if at != outer_end
                && at != inner
                && it_stands_on_the_interior(pool.position_of(at), from, to, near_enough)
            {
                return false;
            }

            if at == outer_end || at == inner || next == outer_end || next == inner {
                // A side meeting the segment at one of its own ends touches it rather than crossing.
                continue;
            }

            if they_cross_in(
                normal,
                from,
                to,
                pool.position_of(at),
                pool.position_of(next),
                near_enough,
            ) {
                return false;
            }
        }
    }

    true
}

/// Twice the ring's area about `normal` - positive where it holds
/// material.
pub fn signed_area_of_pool_ring(pool: &SectionPool, ring: &[i32], normal: Vector3) -> f64 {
    let mut twice = Vector3::ZERO;

    for i in 0..ring.len() {
        twice += pool
            .position_of(ring[i])
            .cross(pool.position_of(ring[(i + 1) % ring.len()]));
    }

    twice.dot(normal)
}

/// Twice the ring's area about `normal` - positive where it holds material.
pub fn signed_area_of_mesh_ring(mesh: &Mesh, ring: &[i32], normal: Vector3) -> f64 {
    let mut twice = Vector3::ZERO;

    for i in 0..ring.len() {
        let from = Vector3::from_vector3_float(mesh.vertices[ring[i] as usize]);
        let to = Vector3::from_vector3_float(mesh.vertices[ring[(i + 1) % ring.len()] as usize]);

        twice += from.cross(to);
    }

    twice.dot(normal)
}
