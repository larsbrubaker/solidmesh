//! The fallback for the faces that substitution alone would fold: the REGION they stand in is
//! drawn again from its own moved OUTLINE with the ear clipper
//! ([`crate::bevel::surface::surface_ring_ears`]), and the region's faces are replaced by the
//! ears'. `SectionFaceRebuild` gathers the regions it is handed; `SectionEndSplice` draws its
//! rewritten rings through [`try_draw_the_ring`]. Crease reading is [`section_fan_walk::is_a_crease`].
//!
//! **Why the outline is the right thing to keep.** A region of this kind is a disc of triangles
//! whose only boundary is its outline ring, so any triangulation of that ring covers exactly the
//! same piece of the part. The triangulation the modeller happened to arrive with is not part of
//! the part's shape; a corner of it that the boundary has walked past is a fact about that
//! triangulation only, and is answered by drawing the disc again rather than by refusing the size.
//!
//! **Why a UNION and not one fan.** A thin triangle that folds when a chain vertex moves out to its
//! boundary usually carries the NEIGHBOURING chain vertex as a corner too - it sits between the
//! chord in front of one mouth vertex and its neighbour - so both of those fans fold and they SHARE
//! that face. Drawing them one at a time would read the second one's outline out of faces the first
//! had already replaced, so the fans that fold and share faces are gathered as ONE region by
//! `SectionFaceRebuild` before they reach this box.
//!
//! **The ring is read off the REBUILT faces, not the mesh.** A region's outline neighbours can be
//! chain vertices themselves, and those have already been moved to their own boundaries in these
//! very faces. Reading the ring from the rebuilt triples takes those moves with it, which reading
//! the mesh would not.
//!
//! **One plane at a time.** The ears triangulate IN A PLANE, so the region is split at its folds:
//! two of its faces are in the same flat piece when the mesh edge between them is not a crease, so
//! an OFFSET side's fans come out as one flat piece and a SLIDE side's split either side of the fold
//! it slides down. This is the only place in the section construction where the face COUNT may
//! change.
//!
//! **Interior points.** A point of the piece its outline does not carry is one of two things. If
//! it still stands inside the ring, the ears would drop it and the part would lose a vertex it has,
//! so it is put back by splitting the ear that holds it into three. If it stands OUTSIDE the ring,
//! the boundary has moved over it - the fillet has eaten the sliver it sat on - and leaving it out
//! is the answer rather than a refusal: in a FLAT piece an interior point carries no shape, so the
//! surface the ears draw is the same surface either way, and a point the fillet covers is not a
//! point of the result.
//!
//! **When it refuses.** Where the moved outline is no longer one simple ring the boundary has
//! crossed the region's own edge - the size genuinely does not fit the face - and that is a refusal
//! at the station, not a re-mesh.

use std::collections::{HashMap, HashSet};

use indexmap::{IndexMap, IndexSet};
use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk;
use super::SectionPool;
use crate::bevel::surface::{surface_ring_ears, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// A triangle of the rebuilt faces, by pool index.
pub type Triple = (i32, i32, i32);

fn corner_not_supported(at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceCornerNotSupported,
        vec![at.into()],
    )
}

/// Replaces every face of one region with the ears' triangulation of its
/// moved outline, one flat piece of the region at a time.
///
/// `mesh` is the part, for the planes the region's faces were drawn in; `graph` its edge graph,
/// for reading which shared edges are folds; `pool` the pool the sections minted their boundaries
/// into, seeded with the mesh; `region` the faces to draw again - the union of the folding fans
/// that share them - in the order the caller gathered them (which orders the flat pieces);
/// `station` where to name a refusal, on the part; `rebuilt` the faces as they stand after every
/// substitution; `added` the triangles drawn to replace the region, appended to. The error is why
/// not, where the moved outline cannot be triangulated.
#[allow(clippy::too_many_arguments)]
pub fn try_remesh_the_region(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &SectionPool,
    region: &[i32],
    station: Vector3,
    rebuilt: &[Triple],
    added: &mut Vec<Triple>,
) -> Result<(), BevelAdvisory> {
    let held: HashSet<i32> = region.iter().copied().collect();

    for piece in flat_pieces_of(mesh, graph, region) {
        try_remesh_one_flat_piece(mesh, pool, &piece, &held, station, rebuilt, added)?;
    }

    Ok(())
}

/// The region split into the pieces that are flat: its faces gathered across every shared mesh
/// edge that is NOT a crease, so each piece can be triangulated in one plane.
fn flat_pieces_of(mesh: &Mesh, graph: &MeshEdgeGraph, region: &[i32]) -> Vec<Vec<i32>> {
    let held: HashSet<i32> = region.iter().copied().collect();
    let mut joined: HashMap<i32, i32> = HashMap::new();

    fn root(joined: &HashMap<i32, i32>, mut face: i32) -> i32 {
        while let Some(&up) = joined.get(&face) {
            if up == face {
                break;
            }
            face = up;
        }

        face
    }

    for &face in region {
        joined.insert(face, face);
    }

    for edge in 0..graph.edge_count() {
        if graph.get_face_count(edge) != 2 {
            continue;
        }

        let one = graph.get_face(edge, 0);
        let other = graph.get_face(edge, 1);

        if !held.contains(&one)
            || !held.contains(&other)
            || section_fan_walk::is_a_crease(mesh, graph, edge)
        {
            continue;
        }

        let from = root(&joined, one);
        let to = root(&joined, other);
        joined.insert(from, to);
    }

    // Insertion ordered, so the pieces come out in region order.
    let mut pieces: IndexMap<i32, Vec<i32>> = IndexMap::new();

    for &face in region {
        pieces.entry(root(&joined, face)).or_default().push(face);
    }

    pieces.into_values().collect()
}

/// Draws the ring of one flat piece of the region again. The piece's faces are left in `rebuilt`
/// for the caller to drop; the new triangles go to `added`.
fn try_remesh_one_flat_piece(
    mesh: &Mesh,
    pool: &SectionPool,
    piece: &[i32],
    region: &HashSet<i32>,
    station: Vector3,
    rebuilt: &[Triple],
    added: &mut Vec<Triple>,
) -> Result<(), BevelAdvisory> {
    let ring = match try_the_outline(rebuilt, piece) {
        Ok(ring) => ring,
        Err(pinched_at) => {
            // Named where the outline actually stuck, when it stuck at a point rather than at the
            // count of its rings, so the advisory points at the place and not at the station it
            // was handed.
            return Err(corner_not_supported(if pinched_at >= 0 {
                pool.position_of(pinched_at)
            } else {
                station
            }));
        }
    };

    try_draw_the_ring(
        mesh,
        pool,
        piece,
        region,
        &ring,
        Vector3::from_vector3_float(mesh.faces[piece[0] as usize].normal),
        station,
        rebuilt,
        added,
        None,
    )
}

/// Draws one flat piece again from a ring already in hand - the outline as
/// read, or one an end's SPLICE has rewritten (`SectionEndSplice`) - and puts back the interior
/// points that still stand inside it.
///
/// `ring` is the ring to ear, wound the way the piece's faces are; `plane` the plane the piece is
/// drawn in; `gone` the points of the piece that were DELIBERATELY dropped - the chain's end vertex
/// and the crease vertices a splice swallowed - which are not to be put back and are not a crack in
/// the part, because the splice knows nothing else uses them any more.
#[allow(clippy::too_many_arguments)]
pub fn try_draw_the_ring(
    mesh: &Mesh,
    pool: &SectionPool,
    piece: &[i32],
    region: &HashSet<i32>,
    ring: &[i32],
    plane: Vector3,
    station: Vector3,
    rebuilt: &[Triple],
    added: &mut Vec<Triple>,
    gone: Option<&HashSet<i32>>,
) -> Result<(), BevelAdvisory> {
    // The ears work in a pool of their own, holding just this ring, so the section pool - which
    // the strips are lofted out of by index - is never asked to weld anything.
    let mut at = Vec::with_capacity(ring.len());
    let mut indices = Vec::with_capacity(ring.len());

    for (i, &point) in ring.iter().enumerate() {
        at.push(Vector3Float::from_vector3(pool.position_of(point)));
        indices.push(i as i32);
    }

    let mut drawn: Vec<Triple> = Vec::new();

    // ROUND TIPS FIRST, so the answer does not move with the placement. A region's outline carries
    // the fillet's own boundary samples, which run very nearly straight: a tip of one standing a
    // micron off the chord its neighbours span has a convexity whose SIGN is float noise, and a
    // walk that takes whichever ear comes first in ring order therefore cuts the same region
    // differently once the part is moved. Measured on the raked bore's top mouth arc, where it was
    // the only thing left moving between placements.
    if !surface_ring_ears::try_triangulate_the_ring(
        &indices,
        &SurfaceVertexPool::new(&at),
        &mut drawn,
        plane,
        true,
    ) {
        // The boundary has crossed the region's own outline: the size does not fit this face.
        return Err(corner_not_supported(station));
    }

    let mut eared: Vec<Triple> = Vec::with_capacity(drawn.len());

    for face in &drawn {
        eared.push((
            ring[face.0 as usize],
            ring[face.1 as usize],
            ring[face.2 as usize],
        ));
    }

    try_keep_the_interior_points_still_inside(
        mesh, pool, piece, region, rebuilt, ring, plane, &mut eared, gone,
    )?;

    added.extend(eared);

    Ok(())
}

/// Puts back every point of the piece that its outline does not carry AND that still stands
/// inside the ring, by splitting the ear that holds it into three. A point the boundary has moved
/// over is left out - see the module's remarks on why that is the answer and not a refusal - BUT
/// only where the region being drawn again is every face that uses it. A mesh vertex a face
/// OUTSIDE the region still stands on cannot be dropped: that face would keep a corner nothing
/// else carries and the part would crack there, so it is refused at the point itself.
#[allow(clippy::too_many_arguments)]
fn try_keep_the_interior_points_still_inside(
    mesh: &Mesh,
    pool: &SectionPool,
    piece: &[i32],
    region: &HashSet<i32>,
    rebuilt: &[Triple],
    ring: &[i32],
    plane: Vector3,
    eared: &mut Vec<Triple>,
    gone: Option<&HashSet<i32>>,
) -> Result<(), BevelAdvisory> {
    let on_the_ring: HashSet<i32> = ring.iter().copied().collect();
    let mut inside: Vec<i32> = Vec::new();

    for &face in piece {
        let triple = rebuilt[face as usize];
        for corner in [triple.0, triple.1, triple.2] {
            if gone.is_some_and(|gone| gone.contains(&corner)) {
                continue;
            }

            if !on_the_ring.contains(&corner) && !inside.contains(&corner) {
                inside.push(corner);
            }
        }
    }

    for &point in &inside {
        let at = pool.position_of(point);
        let holder = eared.iter().position(|&ear| holds(pool, ear, at, plane));

        let Some(holder) = holder else {
            if point < mesh.vertices.len() as i32
                && !every_face_on_it_is_in_the_region(mesh, region, point)
            {
                return Err(corner_not_supported(at));
            }

            continue;
        };

        let split = eared[holder];

        eared[holder] = (split.0, split.1, point);
        eared.push((split.1, split.2, point));
        eared.push((split.2, split.0, point));
    }

    Ok(())
}

/// Whether every face of the part standing on `vertex` - by INDEX, so two mesh vertices the
/// boolean left at one position are two different points here - is one of the faces being drawn
/// again.
fn every_face_on_it_is_in_the_region(mesh: &Mesh, region: &HashSet<i32>, vertex: i32) -> bool {
    for (face, drawn) in mesh.faces.iter().enumerate() {
        if (drawn.v0 == vertex || drawn.v1 == vertex || drawn.v2 == vertex)
            && !region.contains(&(face as i32))
        {
            return false;
        }
    }

    true
}

/// Whether a triangle of the piece's plane holds `at`, read scale free against the triangle's own
/// area. A point ON an edge does not count as held: splitting there would leave the triangle on
/// the other side of that edge with a T junction in it.
fn holds(pool: &SectionPool, face: Triple, at: Vector3, plane: Vector3) -> bool {
    let a = pool.position_of(face.0);
    let b = pool.position_of(face.1);
    let c = pool.position_of(face.2);

    let twice_the_area = (b - a).cross(c - a).dot(plane);

    if twice_the_area <= 0.0 {
        return false;
    }

    for (from, to) in [(a, b), (b, c), (c, a)] {
        if (to - from).cross(at - from).dot(plane) <= 1e-9 * twice_the_area {
            return false;
        }
    }

    true
}

/// The boundary ring of one flat piece - its once-used edges, read off the
/// REBUILT faces and chained up, wound the way those faces are. More than one ring, or a point the
/// outline leaves two ways, is not a disc and is refused by the caller.
///
/// The error is the pool point the outline left two ways, so the caller can
/// name where the walk stuck; -1 where it stuck on something with no one point to it, such as more
/// than one ring.
pub fn try_the_outline(rebuilt: &[Triple], piece: &[i32]) -> Result<Vec<i32>, i32> {
    // Insertion ordered, because which once-used edge
    // comes first decides where the ring starts, and so how the ears cut it.
    let mut drawn: IndexSet<(i32, i32)> = IndexSet::new();

    for &face in piece {
        let triple = rebuilt[face as usize];

        if triple.0 == triple.1 || triple.1 == triple.2 || triple.2 == triple.0 {
            // A triangle two of whose corners were substituted by the SAME point - a station the
            // corner beside it swallowed, whose boundary point IS the corner's - has collapsed onto
            // a segment. Its two real sides run there and back and cancel, so it carries no outline
            // at all: read past it rather than let its zero length side pinch the ring.
            continue;
        }

        for side in [
            (triple.0, triple.1),
            (triple.1, triple.2),
            (triple.2, triple.0),
        ] {
            if !drawn.insert(side) {
                // The same directed edge twice is two faces of the piece wound against each other.
                return Err(-1);
            }
        }
    }

    let mut next: IndexMap<i32, i32> = IndexMap::new();

    for &(from, to) in &drawn {
        if drawn.contains(&(to, from)) {
            continue;
        }

        if next.contains_key(&from) {
            // Two ways out of one point: the outline pinches there and is not a simple ring.
            return Err(from);
        }

        next.insert(from, to);
    }

    let Some((&start, _)) = next.first() else {
        return Err(-1);
    };

    let mut walked = Vec::new();
    let mut here = start;

    loop {
        walked.push(here);
        // Every once-used edge's end has a once-used edge out of it (each face adds one way in and
        // one way out at each corner, and a cancelled pair removes one of each), so this lookup
        // always answers; a miss is refused rather than trusted.
        let Some(&step) = next.get(&here) else {
            return Err(-1);
        };
        here = step;
        if here == start || walked.len() > next.len() {
            break;
        }
    }

    if walked.len() != next.len() {
        // The once-used edges make more than one ring: the piece is not a disc.
        return Err(-1);
    }

    Ok(walked)
}
