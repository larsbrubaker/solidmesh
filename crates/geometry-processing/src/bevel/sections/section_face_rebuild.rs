//! Opens the room the fillet will stand in by REWRITING the part's own faces: at
//! every chain vertex, each of the two fans around it has that vertex replaced by the boundary
//! point the section handed that side ([`try_rebuild`]). The junction station's wall side, whose
//! fan the fold cuts in two, is [`super::section_face_rebuild_fold`]; a fan that folds is drawn
//! again by [`super::section_fan_remesh`], and an open run's transverse face is spliced by
//! [`super::section_end_splice`].
//!
//! **Substitution, not clipping.** Nothing is cut here. Every triangle of the part comes out as a
//! triangle, the face COUNT does not change, and no point of the part moves except the chain's own
//! vertices, each of which is read as two different points - one per side - by the faces that see
//! it from that side. The triangles on the chain edge itself keep their places and simply get
//! shorter. That is what the edge-centric construction could not do: it flooded a band off each
//! face, re-triangulated its outline, and then had to find the strip's rail among the points that
//! flood had minted.
//!
//! **The fan.** A side's fan at a chain vertex is every face around that vertex between the
//! ARRIVING chain edge and the LEAVING one on that side, found by turning face to face across the
//! edges at the vertex - the same walk `SectionBuilder` makes to find the one crease a boundary
//! slides down, kept here so the substitution rides exactly the faces the boundary was computed
//! for. Because a routed chain vertex carries exactly two picked edges, its left and right fans
//! together are ALL of the faces around it, each in exactly one of them; so after the rebuild no
//! face anywhere carries a chain vertex.
//!
//! **What is checked.** Only that no rewritten triangle turned itself inside out: its new normal
//! must still face the way its old one did, and it must still have area. A vertex pulled past the
//! far side of its own triangle is the size not fitting the face, and this box REFUSES it;
//! re-meshing that fan's outline (`SurfaceRingEars`) is the fallback the next box builds.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::section_face_rebuild_fold::try_substitute_either_side_of_the_fold;
use super::section_fan_remesh::{try_remesh_the_region, Triple};
use super::section_fan_walk::{self, EdgeAtPair};
use super::section_smooth_seam::SectionSmoothSeam;
use super::{
    section_end_splice, section_room, section_split_blend, BevelSection, JunctionSection,
    SectionKind, SectionPool, SectionRun,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FeatureEdge};

/// One side's fan as it was walked, kept so a fan that folds can be drawn again from its outline.
/// The edges crossed and the boundary point are not kept: nothing reads either.
#[derive(Debug, Clone)]
pub(super) struct WalkedFan {
    pub faces: Vec<i32>,
    pub station: Vector3,
}

/// The rebuild's state as the substitutions write it (the rebuilt faces, the touched faces, the
/// fans, the junction rims and the swallowed faces), with the part they read.
pub(super) struct Rebuild<'a> {
    pub mesh: &'a Mesh,
    pub graph: &'a MeshEdgeGraph,
    pub pool: &'a SectionPool,
    pub edge_at_pair: &'a EdgeAtPair,
    /// Every face as pool indices, with the substitutions applied so far.
    pub rebuilt: Vec<Triple>,
    /// Every face a substitution reached, and the station of the section that reached it.
    pub touched: Vec<(i32, Vector3)>,
    /// Each side's fan as it was walked.
    pub fans: Vec<WalkedFan>,
    /// The two rim triangles each junction's fold owes.
    pub junction_rims: Vec<Triple>,
    /// The faces a junction's fold SWALLOWED.
    pub swallowed_faces: HashSet<i32>,
}

/// The `SurfaceCornerNotSupported` refusal named at `station`.
pub(super) fn corner_not_supported(station: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceCornerNotSupported,
        vec![station.into()],
    )
}

/// Rewrites the part's faces for one run of sections.
pub fn try_rebuild_run(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    run: &SectionRun,
    pool: &SectionPool,
) -> Result<Vec<Triple>, BevelAdvisory> {
    try_rebuild(mesh, graph, std::slice::from_ref(run), pool, None)
}

/// The whole mesh's faces as pool indices with the substitutions applied, for
/// SEVERAL runs at once - both mouths of a bore, or every port mouth of a barrel - in ONE pass over
/// the face list, with the SMOOTH sides already cut (`seams`, one per run that has a smooth side,
/// `None` where none has): a seam's side is not substituted at all - the cut has replaced every
/// facet of it - and the faces the cut removed are dropped here, in the one place that compacts the
/// face list.
///
/// The runs never see each other here, because a substitution only ever rewrites the ONE corner of
/// a face that stands at its own chain vertex. A wall triangle spanning from the top mouth of a
/// thin plate to the bottom mouth carries a corner of each, and comes out with BOTH rewritten: the
/// fan walks are read off the ORIGINAL mesh, per run and per vertex, so a second run walks exactly
/// the faces its own sections were computed for whatever the first run has already written into
/// them. What they must not do is meet ON a corner: two runs sharing a chain vertex is refused up
/// front at that vertex's station; two runs wanting the same corner of the same face for different
/// vertices is refused by [`Rebuild::rewrite`], which finds the corner already given away.
///
/// The fold check, the region re-mesh and the drop are all over the UNION: a region growing out of
/// one run's folded fan may take in faces another run has rewritten, and re-meshing them one run
/// at a time would read the second one's outline out of faces the first had already replaced.
pub fn try_rebuild(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    runs: &[SectionRun],
    pool: &SectionPool,
    seams: Option<&[SectionSmoothSeam]>,
) -> Result<Vec<Triple>, BevelAdvisory> {
    let not_built = || BevelAdvisory::new(BevelAdvisoryCode::SurfaceConstructionNotBuilt, vec![]);

    if runs.is_empty() {
        return Err(not_built());
    }

    for run in runs {
        if !run.is_closed
            && (run.sections.len() as i32 != run.count() + 1
                || run.sections[0].kind != SectionKind::End
                || run.sections[run.sections.len() - 1].kind != SectionKind::End)
        {
            // An open run is rebuilt only where both its ends ARE sections: an end vertex with no
            // end section has no boundary for the faces around it to be carried onto.
            return Err(not_built());
        }
    }

    every_chain_vertex_is_its_own(runs)?;

    let rebuilt: Vec<Triple> = mesh.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(graph);

    let mut state = Rebuild {
        mesh,
        graph,
        pool,
        edge_at_pair: &edge_at_pair,
        rebuilt,
        touched: Vec::new(),
        fans: Vec::new(),
        junction_rims: Vec::new(),
        swallowed_faces: HashSet::new(),
    };

    // Every face the cuts took out of the part. A substitution may not touch one: a face belongs
    // to exactly ONE side of a chain, so a face the smooth side's cut removed and the other side's
    // fan also claimed would mean the two sides overlap, which is refused by name rather than
    // silently rewritten into a face that is about to be dropped.
    let mut cut_away: IndexSet<i32> = IndexSet::new();

    if let Some(seams) = seams {
        for seam in seams {
            for &face in &seam.cut.removed_faces {
                cut_away.insert(face);
            }
        }
    }

    for (run_index, run) in runs.iter().enumerate() {
        let smooth_side = smooth_side_of(seams, run_index);
        let count = run.count();

        for i in 0..run.sections.len() as i32 {
            let section = &run.sections[i as usize];

            // On an OPEN run the outer sections stand at the chain's two END vertices, where there
            // is only one picked edge: the fan on each side is bounded by that edge and by the
            // crease the end recorded its boundary as sliding down, and is walked by those bounds.
            let is_an_end = !run.is_closed && (i == 0 || i == count);
            let leaving = &run.edges[if is_an_end && i == count {
                count - 1
            } else {
                i % count
            } as usize];
            let arriving = if run.is_closed {
                &run.edges[((i - 1 + count) % count) as usize]
            } else {
                &run.edges[if i == 0 { 0 } else { i - 1 } as usize]
            };

            // An ABSORBED station's corner is substituted by the point the absorbed reading gives
            // it, which on a corner-swallowed side is the CORNER's own offset-meet - the same index
            // the loft laid - so the rebuilt ring meets the band by identity rather than position.
            let left_at = boundary_of(run, section, true);
            let right_at = boundary_of(run, section, false);

            // A JUNCTION station's WALL side has TWO boundaries, one each side of the fold, so the
            // side that is not the shared flat one is handed the junction and splits its own fan.
            // The flat side is substituted as any other.
            let junction = run.junctions.get(&i);

            if smooth_side != Some(true) {
                state.try_substitute(
                    section,
                    arriving,
                    leaving,
                    is_an_end,
                    true,
                    left_at,
                    junction.filter(|j| !j.flat_on_the_left),
                )?;
            }

            if smooth_side != Some(false) {
                state.try_substitute(
                    section,
                    arriving,
                    leaving,
                    is_an_end,
                    false,
                    right_at,
                    junction.filter(|j| j.flat_on_the_left),
                )?;
            }
        }
    }

    for &(face, station) in &state.touched {
        if cut_away.contains(&face) {
            return Err(corner_not_supported(station));
        }
    }

    let mut folded: HashSet<i32> = HashSet::new();

    for &(face, _) in &state.touched {
        if state.swallowed_faces.contains(&face) {
            // A swallowed face has no shape left to read - it is being dropped, not folded.
            continue;
        }

        if !still_faces_the_same_way(mesh, pool, face, state.rebuilt[face as usize]) {
            folded.insert(face);
        }
    }

    let mut dropped: HashSet<i32> = cut_away.iter().copied().collect();

    for &face in &state.swallowed_faces {
        dropped.insert(face);
    }

    let mut added: Vec<Triple> = Vec::new();
    let chain_edges = chain_edge_pairs(runs);

    for region in regions_to_draw_again(
        mesh,
        graph,
        &edge_at_pair,
        &chain_edges,
        &state.fans,
        &folded,
    ) {
        let before_the_region = added.len();

        if try_remesh_the_region(
            mesh,
            graph,
            pool,
            &region.faces,
            region.station,
            &state.rebuilt,
            &mut added,
        )
        .is_err()
        {
            // The whole flat face's own once-used edges did not make one simple ring - a hole's
            // mouth inside the face, say, which leaves it a disc with a second boundary the ears
            // cannot read. Fall back to exactly the partial region the edge-grown reading used to
            // hand over. GAP: such a face is still drawn again in pieces, and a point stranded
            // outside a PIECE's ring is what the whole-face reading was built to answer.
            added.truncate(before_the_region);

            try_remesh_the_region(
                mesh,
                graph,
                pool,
                &region.partial,
                region.station,
                &state.rebuilt,
                &mut added,
            )?;

            for &face in &region.partial {
                dropped.insert(face);
            }

            continue;
        }

        for &face in &region.faces {
            dropped.insert(face);
        }
    }

    // An OPEN run dies on a wall, and that wall's own rim loses the end vertex and gains the end
    // section's whole profile - a splice, not a substitution, so it is drawn here after every fan.
    section_end_splice::try_rebuild_the_transverse_faces(
        mesh,
        graph,
        &edge_at_pair,
        runs,
        pool,
        &chain_edges,
        &state.rebuilt,
        &mut added,
        &mut dropped,
    )?;

    for &(face, station) in &state.touched {
        if !folded.contains(&face) || dropped.contains(&face) {
            continue;
        }

        // The boundary was pulled past the far side of this triangle and no fan holding it was
        // drawn again - the fillet is wider than the face it has to stand on at this vertex. Named
        // at the station so the application can point at the place rather than at the pick.
        return Err(corner_not_supported(station));
    }

    // THE JUNCTIONS' OWN FACES. The patch across the fold and the two rim chords that carry each
    // wall from its own wall end onto F1 - faces of the PART, not of the blend, so they are laid
    // here beside the re-meshed regions rather than by the loft.
    for run in runs {
        for junction in run.junctions.values() {
            for triangle in junction.patch_triangles() {
                added.push((triangle[0], triangle[1], triangle[2]));
            }
        }
    }

    added.extend_from_slice(&state.junction_rims);

    let mut rebuilt = state.rebuilt;

    if !dropped.is_empty() || !added.is_empty() {
        let mut kept: Vec<Triple> = Vec::with_capacity(rebuilt.len() + added.len());

        for (face, &drawn) in rebuilt.iter().enumerate() {
            if !dropped.contains(&(face as i32)) {
                kept.push(drawn);
            }
        }

        kept.extend(added);
        rebuilt = kept;
    }

    Ok(rebuilt)
}

/// The pool point one station's corner is substituted by on one side: its own boundary, unless the
/// run absorbed that station, in which case the absorbed reading names the point - the same index
/// the loft laid there.
fn boundary_of(run: &SectionRun, section: &BevelSection, is_left: bool) -> i32 {
    let own = if is_left {
        section.left_boundary
    } else {
        section.right_boundary
    };

    let Some(absorbed) = run.absorbed.as_ref() else {
        return own;
    };

    if !section_split_blend::absorb_split_vertices() {
        return own;
    }

    let absorbed = absorbed.point_at(section.chain_vertex, is_left);

    if absorbed >= 0 {
        absorbed
    } else {
        own
    }
}

/// Which side of the run at `run_index` a cut has already replaced - `Some(true)` for the LEFT,
/// `Some(false)` for the RIGHT, `None` where neither side is smooth and both are substituted.
/// (A seam names its run by index.)
fn smooth_side_of(seams: Option<&[SectionSmoothSeam]>, run_index: usize) -> Option<bool> {
    seams?
        .iter()
        .find(|seam| seam.run == run_index as i32)
        .map(|seam| seam.is_left)
}

/// Whether no two runs stand on the same chain vertex - refused at that vertex's station, since a
/// vertex carrying two picked chains has four picked edges and its fans are not "the two sides of
/// one chain" at all.
fn every_chain_vertex_is_its_own(runs: &[SectionRun]) -> Result<(), BevelAdvisory> {
    if runs.len() < 2 {
        return Ok(());
    }

    let mut taken: HashSet<i32> = HashSet::new();

    for run in runs {
        for section in &run.sections {
            if !taken.insert(section.chain_vertex) {
                return Err(corner_not_supported(section.station));
            }
        }
    }

    Ok(())
}

/// One region the ears draw again: the whole flat faces, the partial fallback, and the station
/// that names it.
struct RegionToDraw {
    faces: Vec<i32>,
    partial: Vec<i32>,
    station: Vector3,
}

/// The regions the ears have to draw again: every fan holding a folded face, grown to the WHOLE
/// FLAT FACE those folded ones stand in, and then the ones that SHARE a face gathered into one
/// region.
///
/// A thin triangle that folds usually has two chain vertices as corners, so both their fans fold
/// and they share it; drawing them one at a time would read the second one's outline out of faces
/// the first had already replaced. Hence the union.
///
/// **Why the whole face and not a band of it.** The fold means the boundary has moved OUT of the
/// fan it came from, into the faces just past it, so a region stopping at the fan's own outline
/// has an outline the boundary crosses - no ring to ear. Growing over the faces a folded one shares
/// a corner with is not enough either: the picture hanger's top face carries a corner a tenth of a
/// millimetre INSIDE its own rim, the tip of a fifty millimetre sliver fan, and a 0.4 rim offset
/// leaves that corner outside the moved rim. The whole flat face has ONE outline, the picked rim
/// itself, and offsetting that rim is exactly the question the fillet asks. Corners interior to the
/// flat face carry no shape and are dropped by `SectionFanRemesh`, which checks first that no face
/// outside the region uses them.
///
/// The flood stops at the picked chain edges as well as at the part's folds, so a face is never
/// drawn again together with the face on the far side of the very crease being filleted.
fn regions_to_draw_again(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    chain_edges: &HashSet<i64>,
    fans: &[WalkedFan],
    folded: &HashSet<i32>,
) -> Vec<RegionToDraw> {
    let mut joined: Vec<usize> = (0..fans.len()).collect();

    fn root(joined: &[usize], mut fan: usize) -> usize {
        while joined[fan] != fan {
            fan = joined[fan];
        }

        fan
    }

    let mut holding: Vec<usize> = Vec::new();
    let mut by: HashMap<i32, usize> = HashMap::new();
    let mut grown: HashMap<usize, Vec<i32>> = HashMap::new();
    let mut partial: HashMap<usize, Vec<i32>> = HashMap::new();
    let around = faces_by_corner(mesh);
    let mut region_of_face: HashMap<i32, Rc<IndexSet<i32>>> = HashMap::new();

    for (i, fan) in fans.iter().enumerate() {
        if !fan.faces.iter().any(|face| folded.contains(face)) {
            continue;
        }

        holding.push(i);
        let whole = whole_flat_faces_of(
            mesh,
            graph,
            edge_at_pair,
            chain_edges,
            &mut region_of_face,
            &fan.faces,
            folded,
        );
        partial.insert(i, grown_over(mesh, &around, &fan.faces, folded));

        for &face in &whole {
            if let Some(&already) = by.get(&face) {
                let from = root(&joined, already);
                joined[from] = root(&joined, i);
            } else {
                by.insert(face, i);
            }
        }

        grown.insert(i, whole);
    }

    let mut regions: IndexMap<usize, RegionToDraw> = IndexMap::new();

    for &fan in &holding {
        let root = root(&joined, fan);

        let region = regions.entry(root).or_insert_with(|| RegionToDraw {
            faces: Vec::new(),
            partial: Vec::new(),
            station: fans[fan].station,
        });

        for &face in &grown[&fan] {
            if !region.faces.contains(&face) {
                region.faces.push(face);
            }
        }

        for &face in &partial[&fan] {
            if !region.partial.contains(&face) {
                region.partial.push(face);
            }
        }
    }

    regions.into_values().collect()
}

/// One fan's faces together with the WHOLE flat face each of its folded ones stands in - the flood
/// `SectionRoom.FacesOfTheFlatRegion` makes, cached per face so a rim whose vertices all stand on
/// one face floods it once and not once per vertex.
fn whole_flat_faces_of(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    chain_edges: &HashSet<i64>,
    region_of_face: &mut HashMap<i32, Rc<IndexSet<i32>>>,
    fan: &[i32],
    folded: &HashSet<i32>,
) -> Vec<i32> {
    let mut region: Vec<i32> = fan.to_vec();
    let mut held: HashSet<i32> = fan.iter().copied().collect();

    for &face in fan {
        if !folded.contains(&face) {
            continue;
        }

        let flat = match region_of_face.get(&face) {
            Some(flat) => Rc::clone(flat),
            None => {
                let flat = Rc::new(section_room::faces_of_the_flat_region(
                    mesh,
                    graph,
                    edge_at_pair,
                    face,
                    Some(chain_edges),
                    None,
                    None,
                ));

                for &member in flat.iter() {
                    region_of_face.insert(member, Rc::clone(&flat));
                }

                flat
            }
        };

        for &member in flat.iter() {
            if held.insert(member) {
                region.push(member);
            }
        }
    }

    region
}

/// The pair keys of every picked edge of every run, which a flat region may not flood over.
fn chain_edge_pairs(runs: &[SectionRun]) -> HashSet<i64> {
    let mut picked = HashSet::new();

    for run in runs {
        for edge in &run.edges {
            picked.insert(section_room::pair_key(edge.from_vertex(), edge.to_vertex()));
        }
    }

    picked
}

/// One fan's faces together with the faces its FOLDED ones share a corner with, kept to the plane
/// the folded face was drawn in so the grown region is still flat where the fan was.
fn grown_over(
    mesh: &Mesh,
    around: &HashMap<i32, Vec<i32>>,
    fan: &[i32],
    folded: &HashSet<i32>,
) -> Vec<i32> {
    let mut region: Vec<i32> = fan.to_vec();
    let mut held: HashSet<i32> = fan.iter().copied().collect();

    for &face in fan {
        if !folded.contains(&face) {
            continue;
        }

        let drawn = &mesh.faces[face as usize];
        let plane = Vector3::from_vector3_float(drawn.normal);

        for corner in [drawn.v0, drawn.v1, drawn.v2] {
            for &near in around.get(&corner).map(Vec::as_slice).unwrap_or(&[]) {
                if held.contains(&near)
                    || Vector3::from_vector3_float(mesh.faces[near as usize].normal).dot(plane)
                        < FeatureEdge::COPLANAR_FACE_DOT
                {
                    continue;
                }

                held.insert(near);
                region.push(near);
            }
        }
    }

    region
}

/// Every face of the mesh, looked up by each corner it stands on.
fn faces_by_corner(mesh: &Mesh) -> HashMap<i32, Vec<i32>> {
    let mut found: HashMap<i32, Vec<i32>> = HashMap::with_capacity(mesh.vertices.len());

    for (face, drawn) in mesh.faces.iter().enumerate() {
        for corner in [drawn.v0, drawn.v1, drawn.v2] {
            let at = found.entry(corner).or_default();

            if !at.contains(&(face as i32)) {
                at.push(face as i32);
            }
        }
    }

    found
}

impl Rebuild<'_> {
    /// Rewrites the chain vertex to `boundary` in every face of one side's fan.
    #[allow(clippy::too_many_arguments)]
    fn try_substitute(
        &mut self,
        section: &BevelSection,
        arriving: &ChainEdge,
        leaving: &ChainEdge,
        at_an_end: bool,
        on_the_left: bool,
        boundary: i32,
        junction: Option<&JunctionSection>,
    ) -> Result<(), BevelAdvisory> {
        let walked = if at_an_end {
            section_fan_walk::try_walk_to(
                self.mesh,
                self.graph,
                self.edge_at_pair,
                section.chain_vertex,
                if on_the_left {
                    leaving.left_face()
                } else {
                    leaving.right_face()
                },
                leaving.edge.mesh_edge_index,
                if on_the_left {
                    section.left_slide_edge
                } else {
                    section.right_slide_edge
                },
                section.station,
            )?
        } else {
            section_fan_walk::try_walk(
                self.mesh,
                self.graph,
                self.edge_at_pair,
                section.chain_vertex,
                arriving,
                leaving,
                on_the_left,
                section.station,
            )?
        };

        if let Some(junction) = junction {
            return try_substitute_either_side_of_the_fold(
                self,
                junction,
                walked.faces,
                walked.crossed,
            );
        }

        for &face in &walked.faces {
            if !self.rewrite(face, section.chain_vertex, boundary, section.station) {
                // This face has already given its chain corner away - the two fans overlap, which
                // they cannot do on a vertex carrying exactly two picked edges.
                return Err(corner_not_supported(section.station));
            }
        }

        self.fans.push(WalkedFan {
            faces: walked.faces,
            station: section.station,
        });

        Ok(())
    }

    /// Puts `boundary` where `vertex` stood in one face, answering false where that face no longer
    /// carries the vertex at all.
    pub(super) fn rewrite(
        &mut self,
        face: i32,
        vertex: i32,
        boundary: i32,
        station: Vector3,
    ) -> bool {
        let drawn = self.rebuilt[face as usize];

        self.rebuilt[face as usize] = if drawn.0 == vertex {
            (boundary, drawn.1, drawn.2)
        } else if drawn.1 == vertex {
            (drawn.0, boundary, drawn.2)
        } else if drawn.2 == vertex {
            (drawn.0, drawn.1, boundary)
        } else {
            return false;
        };

        self.touched.push((face, station));

        true
    }
}

/// Whether a rewritten triangle still has area and still faces the way the face it came from did.
fn still_faces_the_same_way(mesh: &Mesh, pool: &SectionPool, face: i32, drawn: Triple) -> bool {
    let was = &mesh.faces[face as usize];

    let before = cross(
        Vector3::from_vector3_float(mesh.vertices[was.v0 as usize]),
        Vector3::from_vector3_float(mesh.vertices[was.v1 as usize]),
        Vector3::from_vector3_float(mesh.vertices[was.v2 as usize]),
    );
    let now = cross(
        pool.position_of(drawn.0),
        pool.position_of(drawn.1),
        pool.position_of(drawn.2),
    );

    // Twice the area, so "has area" and "faces the same way" are one reading of one vector: a
    // triangle whose corner has been pulled onto the line of the other two has a zero cross, and
    // one pulled past it has a cross pointing back at the old one.
    if now.length_squared() <= 0.0 {
        return false;
    }

    now.get_normal().dot(before.get_normal()) > 0.0
}

/// Twice the signed area vector of the triangle `a b c`.
pub(super) fn cross(a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    (b - a).cross(c - a)
}
