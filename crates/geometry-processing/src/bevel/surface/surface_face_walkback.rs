//! Phase 2 of the surface bevel (free functions): every face region the plans touch is walked back
//! to its tangent lines. The sliver between each line and the chain edge is taken off, what is left
//! is re-triangulated with vertices standing ON the line, and everything else in the mesh is
//! carried over unchanged - except the unselected faces the tangent lines ran out across, which are
//! split at the crossing so no T junction is left behind.
//!
//! This is the decision - which region is cut where, and in what order the pieces are asked for.
//! The mechanics are its neighbours: the band each cut reaches and the guard against anything
//! standing in it (`surface_walkback_bands`), the rails borrowed across a crease
//! (`surface_band_outline`), the standing features held back (`surface_standing_feature_stretch`,
//! `surface_walkback_standing_features`), the two-pass rewrite (`surface_walkback_rewrite`) and the
//! tangent runs each cut leaves (`surface_walkback_tangent_runs`).
//!
//! **Per facet, not per region polygon.** The obvious reading of "cut the region's polygon by the
//! tangent line" is to order the rim into a loop, project it into the region's plane, clip the
//! loop, and re-triangulate. This does the same cut one triangle at a time against the PLANE
//! through the tangent line ([`SurfaceFacetCut`]), which comes out the same on a planar region and
//! is better behaved everywhere else: existing triangles that the cut misses are copied with their
//! own vertex indices, so nothing but the cut band is re-triangulated and nothing that was welded
//! comes apart; a region with a hole or a reflex rim needs no special case, because a triangle has
//! neither; and a slightly warped region is cut on its own facets instead of on a mean plane the
//! facets do not lie in.
//!
//! **The rim still has to close.** A region whose rim does not close is one whose neighbours
//! cannot be trusted to be split where this splits them, so the walkback refuses it
//! (`FaceRegionRimIsPartial`) rather than quietly leaving a crack. The check is
//! `bevel_chain_linker::link` over the region's own boundary edges.
//!
//! **Two passes.** The first finds every crossing of a tangent plane with a mesh edge and welds it
//! into the vertex pool; the second rebuilds each face with the crossings on its edges inserted
//! before it is cut. A crossing belongs to BOTH faces on its edge, and the second of them may be in
//! another region, or in no region at all.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::sync::Arc;

use indexmap::IndexMap;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::surface_walkback_tangent_runs::KeptFaces;
use super::{
    surface_band_outline, surface_fan, surface_mesh_closure, surface_ring,
    surface_standing_feature_stretch, surface_walkback_bands, surface_walkback_rewrite,
    surface_walkback_standing_features, surface_walkback_tangent_runs, SurfaceBevelPlan,
    SurfaceFacetCut, SurfaceRegionWalkback, SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
    SurfaceWalkbackResult, SurfaceWalkbackRuns,
};
use crate::bevel::{bevel_chain_linker, BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{FaceGroup, FaceRegionCache};

/// What the strip's OWN chain is, as `surface_room_standing_feature::stands_on`
/// wants it - the mesh edges the pass is beveling (first), and the vertices those edges end at
/// (second). Every reading that has to tell the chain from everything else the bands reach asks
/// for the same two sets, so they are gathered here once rather than rebuilt at each of them.
pub fn the_chains_own(plans: &[Rc<SurfaceBevelPlan>]) -> (HashSet<i32>, HashSet<i32>) {
    let mut beveled_mesh_edges = HashSet::new();
    let mut chain_vertices = HashSet::new();

    for plan in plans {
        for edge in &plan.edges {
            beveled_mesh_edges.insert(edge.edge.edge.mesh_edge_index);
            chain_vertices.insert(edge.edge.edge.start_vertex);
            chain_vertices.insert(edge.edge.edge.end_vertex);
        }
    }

    (beveled_mesh_edges, chain_vertices)
}

/// Walks every
/// region one plan touches back to its tangent lines.
pub fn try_apply_one(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    regions: &FaceRegionCache,
    plan: &Rc<SurfaceBevelPlan>,
) -> Result<SurfaceWalkbackResult, BevelAdvisory> {
    try_apply(mesh, graph, regions, std::slice::from_ref(plan))
}

/// Walks every region EVERY plan of one part touches back to its tangent lines, in one pass.
///
/// One pass because a face two chains both reach has to give up both their bands at once: cut for
/// one chain and then cut again for the other and the second cut is being made against a mesh the
/// second chain was never planned on. Gathering the cuts per region and clipping each facet by all
/// of them is what a closed rim's shared face already does, so more than one chain asks nothing new
/// of the cut itself - only of the bookkeeping, which numbers the chains' edges end to end so that
/// one tangent run can be named whichever chain asked for it.
///
/// `mesh` is the welded mesh the plans were made on (not modified), `graph` its own edge graph and
/// `regions` the region cache the plans were made with, at the same tolerance. An empty plan list
/// (or plans with no edges) is refused as `NoEdgesResolved`: there is nothing to bevel.
pub fn try_apply(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    regions: &FaceRegionCache,
    plans: &[Rc<SurfaceBevelPlan>],
) -> Result<SurfaceWalkbackResult, BevelAdvisory> {
    let planned: usize = plans.iter().map(|plan| plan.edges.len()).sum();

    if planned == 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::NoEdgesResolved,
            vec![],
        ));
    }

    let mut walkbacks = try_collect_regions(plans)?;

    let mut pool = SurfaceVertexPool::new(&mesh.vertices);

    // Which region each face is being walked back as part of, or -1 for the rest of the mesh. The
    // cache is asked for the same answer as a guard: a plan made against another cache - or the
    // same mesh at another tolerance - would name regions this mesh does not have, and the cut
    // would land on faces nobody planned.
    let mut region_of_face = vec![-1i32; mesh.faces.len()];

    for walkback in walkbacks.values() {
        // The walkback's faces are the region's faces in order and never change, so the region is
        // read here instead.
        for &face in walkback.region.faces() {
            if !walkback.faces.contains(&face) {
                continue;
            }

            let key = regions
                .region(face)
                .map(|region| SurfaceBevelPlan::key_of(&region));

            if key != Some(walkback.key) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::FaceRegionHasNoBevelableBoundary,
                    vec![
                        pool.position_of(mesh.faces[face as usize].v0).into(),
                        regions.flat_angle_degrees().into(),
                    ],
                ));
            }

            region_of_face[face as usize] = walkback.key;
        }
    }

    surface_walkback_bands::limit_each_cut_to_the_band_its_own_edge_reaches(
        mesh,
        &mut walkbacks,
        &pool,
    );

    surface_band_outline::borrow_the_rail_of_each_facet_reached_across_a_crease(
        mesh,
        &mut walkbacks,
        &pool,
    );

    surface_walkback_bands::try_check_nothing_is_standing_in_the_band(mesh, &walkbacks, &pool)?;

    let splits_on_edge =
        surface_walkback_rewrite::try_find_crossings(mesh, graph, &walkbacks, &mut pool)?;

    let mut faces: Vec<(i32, i32, i32)> = Vec::with_capacity(mesh.faces.len());
    let mut faces_of_region: IndexMap<i32, KeptFaces> = IndexMap::new();
    let mut kept_range_of_facet: IndexMap<i32, (i32, i32)> = IndexMap::new();

    // The band's own corners, gathered as the facets are clipped because that is the only place
    // the line a cut was DRAWN on is known (SurfaceWalkbackResult::band_corners). Keyed by the cut
    // and then by which of its ends the corner stands on.
    let mut band_corners_of_cut: IndexMap<(i32, bool), Vec<Vec<i32>>> = IndexMap::new();

    // Anything the bands passed UNDER, read before a facet is emitted: those facets are held back
    // rather than carried over, because each of them has to be extended through the corner the
    // band took (SurfaceStandingFeatureWithheld). The plans' own edges and their end vertices are
    // what tells a feature from the strip's own chain.
    let (beveled_mesh_edges, chain_vertices) = the_chains_own(plans);

    let mut withheld = surface_standing_feature_stretch::try_withhold(
        mesh,
        graph,
        &walkbacks,
        &beveled_mesh_edges,
        &chain_vertices,
        &pool,
    )?;

    surface_walkback_standing_features::none_of_the_feature_is_also_being_cut(
        &withheld,
        &region_of_face,
        &pool,
    )?;

    // ONE FEATURE, HOWEVER MANY BANDS REACHED IT. The stretches are read a region at a time, so a
    // feature standing out of two of them - the nose of the nose-in-notch part, with both knife
    // tips beveled - arrives here as two records over the very same facets. Merged into one before
    // anything is indexed by facet, because what follows hands each facet ONE ring and each face
    // has to carry both of the crease runs it lost its footing on.
    surface_walkback_standing_features::try_merge_what_two_bands_both_reached(
        &mut withheld,
        &pool,
    )?;

    let held_back = try_hold_back(&withheld, &pool)?;

    for face_index in 0..mesh.faces.len() as i32 {
        let ring = surface_walkback_rewrite::ring_of(mesh, face_index, &splits_on_edge, &pool);

        if let Some(standing) = held_back.get(&face_index) {
            // Held back, with its ring: the band took the material under this face's crease, so
            // what goes in the mesh is the whole region's outline MERGED with its extension out to
            // the fillet, and that is built once the strip is there to meet
            // (SurfaceStandingFeatureFaces). The rings are gathered facet by facet here because
            // this is where the crossings on their edges are known.
            standing.rings.borrow_mut().push(ring);
            continue;
        }

        let region = region_of_face[face_index as usize];

        if region < 0 {
            // Not walked back, but a tangent line may have run out across it: the crossings on its
            // edges are already in the ring, and fanning the ring is what keeps its neighbour's
            // new vertex a vertex of this face too. Nothing crossed it means nothing to
            // re-triangulate, and it is carried over verbatim rather than re-fanned - a fan drops a
            // triangle with no area in it, and a source mesh is allowed to hold one (both caps of
            // the L prism fixture do). Dropping it takes the surface's bookkeeping open along its
            // three edges while changing no volume at all, which reads downstream as a crack that
            // is not there.
            if ring.len() == 3 {
                faces.push((ring[0], ring[1], ring[2]));
            } else {
                surface_fan::emit(&ring, &mut pool, &mut faces);
            }

            continue;
        }

        let walkback = &walkbacks[&region];

        let pieces = clip_the_facet(
            walkback,
            face_index,
            ring,
            &mut pool,
            &mut band_corners_of_cut,
        );

        let before = faces.len();

        for piece in &pieces {
            surface_fan::emit(piece, &mut pool, &mut faces);
        }

        let kept = faces_of_region.entry(walkback.key).or_default();

        let first_of_this_facet = kept.len() as i32;

        kept.extend_from_slice(&faces[before..]);

        // Which stretch of the region's kept faces came from THIS facet. A cut that borrowed
        // another rail borrowed it facet by facet (SurfaceBandOutline), so reading the borrowed
        // line's own segments back off the region as a whole would sweep in every other facet's;
        // `continuations_along` asks only the facets that borrowed.
        kept_range_of_facet.insert(
            face_index,
            (first_of_this_facet, kept.len() as i32 - first_of_this_facet),
        );
    }

    // Each withheld region as ONE outline. The triangulation's own diagonals are used twice over
    // between the facets of a region and cancel, which is what leaves the face the part meant
    // rather than the triangles it happens to be cut into - and it is those diagonals, kept as
    // edges, that would otherwise stand on the extension's new material at one end and on nothing
    // at the other.
    for feature in &withheld {
        for face in &feature.faces {
            let outline = surface_ring::try_boundary_of_the_union(&face.rings.borrow());

            let Some(outline) = outline else {
                // Not one simple loop: a region with a hole in it, or pinched to a point, has no
                // one ring for the extension to replace a side of.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                    vec![pool.position_of(face.crease().0).into()],
                ));
            };

            *face.ring.borrow_mut() = Some(outline);
        }
    }

    // The walkback's own answer, read for folds before anything is built on it. Not for CLOSURE:
    // the gap this just opened is rimmed with edges one face stands on, and every one of them gets
    // its second when the strips are sewn in. A fold is the reading that means something here -
    // nothing downstream takes a face away - and read at the end instead it comes back as a corner
    // refusing at a place the corner never touched (surface_mesh_closure::folded_edge_count).
    let (folds, folded) = surface_mesh_closure::folded_edge_count(&faces);

    if folds > 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceResultNotClosed,
            vec![((pool.position_of(folded.0) + pool.position_of(folded.1)) / 2.0).into()],
        ));
    }

    let mut rail_continuations = SurfaceWalkbackRuns::new();

    let mut tangent_runs = surface_walkback_tangent_runs::gather_tangent_runs(
        &walkbacks,
        &faces_of_region,
        &pool,
        Some(&beveled_mesh_edges),
        Some(&chain_vertices),
        Some(&kept_range_of_facet),
        Some(&mut rail_continuations),
    );

    let mut band_corners: SurfaceWalkbackRuns = band_corners_of_cut;

    let assembled = surface_walkback_rewrite::assemble(
        &pool,
        &faces,
        &mut tangent_runs,
        &withheld,
        Some(&mut rail_continuations),
        Some(&mut band_corners),
    );

    Ok(SurfaceWalkbackResult::new(
        Arc::new(assembled),
        tangent_runs,
        withheld,
        Some(rail_continuations),
        Some(band_corners),
    ))
}

/// Indexes every withheld feature's faces by the facets they are made of,
/// refusing a facet held back for two features at once.
fn try_hold_back(
    withheld: &[Rc<super::SurfaceStandingFeatureWithheld>],
    pool: &SurfaceVertexPool,
) -> Result<HashMap<i32, Rc<SurfaceStandingFeatureWithheldFace>>, BevelAdvisory> {
    let mut held_back: HashMap<i32, Rc<SurfaceStandingFeatureWithheldFace>> = HashMap::new();

    for feature in withheld {
        for face in &feature.faces {
            for &facet in &face.mesh_faces {
                if held_back.contains_key(&facet) {
                    // ONE facet held back for two features at once that the merge could NOT make
                    // one - two records whose face sets differ, so the same facet is asked to be
                    // extended as part of two different features. Only one of them would be handed
                    // the ring, and the other would go on to read a ring that is not there - a null
                    // reference in the middle of the walkback rather than a word about the part. It
                    // is the same shape the stretch reader names when it can see both cuts from one
                    // region; here the two come from different regions and only the facet they
                    // share says so.
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts,
                        vec![pool.position_of(face.crease().0).into()],
                    ));
                }

                held_back.insert(facet, Rc::clone(face));
            }
        }
    }

    Ok(held_back)
}

/// One walked-back facet's pieces: its ring clipped by every cut of its region that reaches it,
/// with the band corners each cut left recorded against that cut.
fn clip_the_facet(
    walkback: &SurfaceRegionWalkback,
    face_index: i32,
    ring: Vec<i32>,
    pool: &mut SurfaceVertexPool,
    band_corners_of_cut: &mut IndexMap<(i32, bool), Vec<Vec<i32>>>,
) -> Vec<Vec<i32>> {
    // Pieces, not one ring: a cut with ends takes a BOX out of the facet rather than a half space,
    // and what that leaves is more than one convex piece (SurfaceFacetCut::clip_to_pieces).
    let mut pieces = vec![ring];

    for cut in 0..walkback.cuts.len() {
        if !walkback.reaches[cut].contains(&face_index) {
            continue;
        }

        let mut clipped = Vec::with_capacity(pieces.len());
        let mut corners: Vec<(usize, i32)> = Vec::new();

        // Drawn to the rail of the facet it has reached where that is not its own facet
        // (SurfaceBandOutline), so the band's outline on a chorded wall steps down a crease between
        // two rails' depths rather than leaning past it.
        let drawn = surface_band_outline::as_drawn_on(walkback, cut as i32, face_index);

        for piece in &pieces {
            for left in drawn.clip_to_pieces(piece, pool, Some(&mut corners)) {
                if left.len() >= 3 {
                    clipped.push(left);
                }
            }
        }

        if !corners.is_empty() {
            let key = (walkback.cuts[cut].edge_index, walkback.cuts[cut].left);
            let of_the_cut = band_corners_of_cut.entry(key).or_default();

            for (end, vertex) in corners {
                while of_the_cut.len() <= end {
                    of_the_cut.push(Vec::new());
                }

                if !of_the_cut[end].contains(&vertex) {
                    of_the_cut[end].push(vertex);
                }
            }
        }

        // Knitted across the cuts, not just within one of them: clip_to_pieces knits the pieces
        // its OWN ends and tangent plane left, but it is handed this facet's pieces one at a time,
        // so a crossing the next cut lands on a segment an earlier cut created goes into the piece
        // being clipped and into nothing else - the sibling piece keeps the segment whole, which is
        // a T junction in the middle of the face. The facet's whole piece list is the walkback's,
        // so knitting it is too, and it is done after EVERY cut so that a third cut clips rings
        // that already carry the second's crossings. It costs nothing where it cannot matter: knit
        // returns at once for a facet left in one piece.
        pieces = surface_ring::knit(clipped, pool);

        if pieces.is_empty() {
            break;
        }
    }

    pieces
}

/// The regions the plans touch, each with the cuts every chain's edges ask
/// of it. A region reached from two of its faces is two `FaceGroup` objects with two different
/// seeds, so they are gathered under the region's lowest face index - the one name a region always
/// answers to.
fn try_collect_regions(
    plans: &[Rc<SurfaceBevelPlan>],
) -> Result<IndexMap<i32, SurfaceRegionWalkback>, BevelAdvisory> {
    let mut walkbacks = IndexMap::new();

    // The chains' edges numbered end to end, which is the numbering the strip builder reads the
    // tangent runs back by: chain after chain, in the order the plans were handed over.
    let mut edge_index = 0;

    // And which plan each cut came from, which is a different question: the guard's chain
    // exemption is one plan's to claim of its own chain, and a region holds every plan's cuts at
    // once.
    let mut plan_index = 0;

    for plan in plans {
        for (i, edge) in plan.edges.iter().enumerate() {
            // An edge the blend SWALLOWS asks for nothing: it has no strip, so it has nothing to
            // sew to what its band would leave, and the band would take material the CUT's own
            // strip still needs - the bead's stop plane and the cap's sector are what clear that
            // stretch instead (SurfaceEdgeRoles, SurfaceMixedCornerPatch). Its number is still
            // spent, so the edges downstream are numbered as they always were.
            if plan.roles().is_swallowed(i as i32) {
                edge_index += 1;
                continue;
            }

            // Every edge's band is bounded along its own direction, by the mitre plane where the
            // chain turns and by the square plane where it runs out
            // (SurfaceBevelPlan::band_ends_of). Per side, because at a corner where the convexity
            // turns over the face the two edges share is bounded by the other edge's band rather
            // than by a plane of its own.
            let left_ends = plan.band_ends_of(i, true);
            let right_ends = plan.band_ends_of(i, false);

            // The in-face direction away from the edge is the plan's own - the very one it drew the
            // tangent lines from - rather than worked out again here. The chain edge itself travels
            // with each cut, because the standing-feature guard has to be able to tell the crease
            // being bevelled from anything else the band reaches.
            try_add_cut(
                &mut walkbacks,
                &edge.left_region,
                SurfaceFacetCut::with_ends(
                    edge_index,
                    true,
                    edge.left_tangent_line,
                    edge.left_inward,
                    edge.edge.left_face(),
                    left_ends,
                    edge.from(),
                    edge.to(),
                    plan_index,
                    edge.left_band.clone(),
                ),
            )?;

            try_add_cut(
                &mut walkbacks,
                &edge.right_region,
                SurfaceFacetCut::with_ends(
                    edge_index,
                    false,
                    edge.right_tangent_line,
                    edge.right_inward,
                    edge.edge.right_face(),
                    right_ends,
                    edge.from(),
                    edge.to(),
                    plan_index,
                    edge.right_band.clone(),
                ),
            )?;

            edge_index += 1;
        }

        plan_index += 1;
    }

    Ok(walkbacks)
}

/// Files `cut` under its region, opening the region's walkback (after checking
/// its rim closes) the first time it is reached.
fn try_add_cut(
    walkbacks: &mut IndexMap<i32, SurfaceRegionWalkback>,
    region: &Arc<FaceGroup>,
    cut: SurfaceFacetCut,
) -> Result<(), BevelAdvisory> {
    let key = SurfaceBevelPlan::key_of(region);

    if !walkbacks.contains_key(&key) {
        try_check_the_rim_closes(region)?;

        walkbacks.insert(key, SurfaceRegionWalkback::new(key, Arc::clone(region)));
    }

    walkbacks[&key].cuts.push(cut);

    Ok(())
}

/// Whether the region is rimmed all the way round by edges two faces
/// share. An open or non-manifold stretch, or a rim that will not link into closed loops, both say
/// the same thing: there is a boundary here whose other side the walkback cannot split to match,
/// so it does not cut.
fn try_check_the_rim_closes(region: &FaceGroup) -> Result<(), BevelAdvisory> {
    let open = region.open_boundary_mesh_edges().len() as i32;

    if open > 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::FaceRegionRimIsPartial,
            vec![open.into()],
        ));
    }

    let mut loose = 0i32;

    for chain in bevel_chain_linker::link(region.boundary_edges()) {
        if !chain.is_closed {
            loose += chain.edges.len() as i32;
        }
    }

    if loose > 0 {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::FaceRegionRimIsPartial,
            vec![loose.into()],
        ));
    }

    Ok(())
}
