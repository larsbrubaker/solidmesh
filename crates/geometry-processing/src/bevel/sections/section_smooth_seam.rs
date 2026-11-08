//! One run's SMOOTH side ([`SectionSmoothSeam`]) and the reading of which side that is. The cut
//! itself is [`super::section_smooth_side`]'s; this reads which side of
//! each run the cut is asked of, and cuts every run's side that is ([`try_cut_every_smooth_side`]).
//!
//! The seam is the object the three stages of the assembly agree over. The face rebuild leaves this
//! side alone entirely (its faces are the cut's, not the substitution's); the cut hands back the rings
//! the part keeps; the loft ends its last row on the very polyline the cut ran along, pool index for
//! pool index. That identity - one list of integers read by both - is the whole of the watertightness
//! here.
//!
//! **The test.** A side is smooth when ANY of its stations' reaches crossed a FOLD - an edge whose two
//! faces are not coplanar ([`BevelSection::left_walk_crossed_a_fold`], read at the same tolerance the
//! facets' flood uses). That is exactly the condition the substitution cannot answer - it would have to
//! pull a corner of a triangle onto a triangle in another plane - and exactly the one the cut can.
//!
//! **What it is NOT.** "The walk ended outside the station's own fan" is a weaker question and reads a
//! FLAT face as smooth: a boolean tessellates a flat face into long thin triangles, a half-millimetre
//! reach crosses a coplanar diagonal into a triangle that does not carry the chain vertex, and nothing
//! folded. The cube's top face was cut that way on the raked bore's mouth arc and its joins ran off the
//! strip.
//!
//! It is recorded by the walk that crossed the folds rather than re-read off positions afterwards, and
//! a run neither side of which folded reads as smooth on neither side and takes today's path untouched.
//!
//! [`BevelSection::left_walk_crossed_a_fold`]: super::BevelSection::left_walk_crossed_a_fold

use std::collections::HashSet;

use indexmap::IndexMap;
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk::{self, EdgeAtPair};
use super::section_room;
use super::section_smooth_side::{self, SmoothSideCut, SmoothSideRefusal};
use super::section_smooth_side_boundary::SmoothSideFailure;
use super::{NanSectionPoint, SectionPool, SectionRun};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One run's SMOOTH side - which side of it is smooth, and the cut that side
/// made in the part.
pub struct SectionSmoothSeam {
    /// The run whose side this is, by its index in the runs [`try_cut_every_smooth_side`] was handed
    /// (its readers match it by that index).
    pub run: i32,
    /// Whether the smooth side is the LEFT of the chain's walk.
    pub is_left: bool,
    /// What the boundary did to the part's facets on that side.
    pub cut: SmoothSideCut,
}

impl SectionSmoothSeam {
    /// The seam of run `run`, smooth on the left or right, with its cut.
    pub fn new(run: i32, is_left: bool, cut: SmoothSideCut) -> Self {
        SectionSmoothSeam { run, is_left, cut }
    }
}

/// Why [`try_cut_every_smooth_side`] cut nothing: the refusal it answers with, or a point the cut
/// minted that came out not a number, which the pool refuses.
#[derive(Debug, Clone, PartialEq)]
pub enum SmoothSeamFailure {
    /// A side would not cut, named by [`BevelAdvisoryCode::SurfaceSmoothSideNotCut`].
    Refused(BevelAdvisory),
    /// A point minted into the pool was NaN.
    NotANumber(NanSectionPoint),
}

/// Whether the named side of `run` is smooth, read BY ITS REGION where the
/// operation has a feature reading in hand, and by the walks' fold crossings where it has none.
///
/// A side whose region is not one plane CAN be cut and cannot be substituted onto, whatever this run's
/// own walks did. It is only actually cut where some run's seam is on that region
/// ([`try_cut_every_smooth_side`]): a lone run on a curved wall whose every reach stayed inside its own
/// facet has nothing to collide with, and the substitution answers it as it always has.
///
/// `region_is_one_plane` is a memo over seed faces, so the barrel's six runs flood the one wall region
/// once and not twelve times. Pass a fresh map per mesh.
pub fn side_is_smooth(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    run: &SectionRun,
    on_the_left: bool,
    region_is_one_plane: &mut IndexMap<i32, bool>,
) -> bool {
    a_walk_crossed_a_fold(run, on_the_left)
        || (run.is_a_feature_edge.is_some()
            && !side_stands_on_one_plane(
                mesh,
                graph,
                edge_at_pair,
                run,
                on_the_left,
                region_is_one_plane,
            ))
}

/// Whether every region the named side stands in lies in one plane.
fn side_stands_on_one_plane(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    run: &SectionRun,
    on_the_left: bool,
    region_is_one_plane: &mut IndexMap<i32, bool>,
) -> bool {
    for seed in the_sides_seed_faces(run, on_the_left) {
        let one_plane = match region_is_one_plane.get(&seed) {
            Some(&known) => known,
            None => section_room::the_region_is_one_plane(
                mesh,
                graph,
                edge_at_pair,
                seed,
                run.is_a_feature_edge.as_deref(),
                Some(region_is_one_plane),
            ),
        };

        if !one_plane {
            return false;
        }
    }

    true
}

/// The side face of every chain edge - the faces the side's region is
/// flooded out from.
fn the_sides_seed_faces(run: &SectionRun, on_the_left: bool) -> Vec<i32> {
    let mut seeds = Vec::new();

    for edge in &run.edges {
        let seed = if on_the_left {
            edge.left_face()
        } else {
            edge.right_face()
        };

        if seed >= 0 {
            seeds.push(seed);
        }
    }

    seeds
}

/// Whether any station's reach on that side crossed a fold. This is the whole
/// reading where the operation has no feature set, and the SEED of it where it has one: a side that
/// folded is cut, and its region is then what promotes the sides of other runs standing on it.
pub fn a_walk_crossed_a_fold(run: &SectionRun, on_the_left: bool) -> bool {
    for section in &run.sections {
        if if on_the_left {
            section.left_walk_crossed_a_fold
        } else {
            section.right_walk_crossed_a_fold
        } {
            return true;
        }
    }

    false
}

/// Cuts every run's smooth side, where it has one. A run smooth on BOTH
/// sides is not built: the fillet would then stand between two cuts and neither side's fan is the
/// rebuild's to keep, which is a shape this construction has no reading for yet.
///
/// The seams come back only on success, the only time a caller reads them.
pub fn try_cut_every_smooth_side(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
    runs: &[SectionRun],
) -> Result<Vec<SectionSmoothSeam>, SmoothSeamFailure> {
    let mut seams = Vec::new();

    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(graph);
    let mut region_is_one_plane = IndexMap::new();
    let cut_regions =
        the_regions_some_seam_cuts(mesh, graph, &edge_at_pair, runs, &mut region_is_one_plane);

    for (index, run) in runs.iter().enumerate() {
        if run.is_a_feature_edge.is_none() {
            continue;
        }

        let left = side_is_cut(
            mesh,
            graph,
            &edge_at_pair,
            run,
            true,
            &mut region_is_one_plane,
            &cut_regions,
        );
        let right = side_is_cut(
            mesh,
            graph,
            &edge_at_pair,
            run,
            false,
            &mut region_is_one_plane,
            &cut_regions,
        );

        if !left && !right {
            continue;
        }

        if left && right {
            return Err(SmoothSeamFailure::Refused(refused(
                run.sections[0].station,
                SmoothSideRefusal::BothSidesAreSmooth,
            )));
        }

        let (cut, done) = section_smooth_side::try_cut_the_smooth_side(
            mesh,
            graph,
            &edge_at_pair,
            pool,
            run,
            left,
            run.is_a_feature_edge.as_deref(),
        );

        match done {
            Ok(()) => {}
            Err(SmoothSideFailure::Refused(refusal, place)) => {
                return Err(SmoothSeamFailure::Refused(refused(place, refusal)));
            }
            Err(SmoothSideFailure::NotANumber(nan)) => {
                return Err(SmoothSeamFailure::NotANumber(nan));
            }
        }

        seams.push(SectionSmoothSeam::new(index as i32, left, cut));
    }

    Ok(seams)
}

/// Whether the named side is CUT: it must be smooth, and some run's seam must run on
/// the region it stands in.
///
/// THE COLLISION THIS ANSWERS (measured on the conditioned tapered barrel's wall pick, 2026-09-17). A
/// wall facet there is one ruled quad running the barrel's whole height, so a bore mouth's cut takes
/// the WHOLE of it - including the strip at z = -15 an end rim stands on. The rim's own walks cross no
/// fold (its reach runs along the rulings inside one facet), so by the walk reading alone it was called
/// flat and SUBSTITUTED onto faces that cut had already removed, which is what the face rebuild's
/// cutAway/touched guard was catching. A side standing on a region some other run cuts must be cut too.
fn side_is_cut(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    run: &SectionRun,
    on_the_left: bool,
    region_is_one_plane: &mut IndexMap<i32, bool>,
    cut_regions: &HashSet<i32>,
) -> bool {
    if a_walk_crossed_a_fold(run, on_the_left) {
        return true;
    }

    if run.is_a_feature_edge.is_none()
        || side_stands_on_one_plane(
            mesh,
            graph,
            edge_at_pair,
            run,
            on_the_left,
            region_is_one_plane,
        )
    {
        // A flat region is substituted onto, and a cut never removes one, so there is nothing to
        // collide with however many runs stand on it.
        return false;
    }

    for seed in the_sides_seed_faces(run, on_the_left) {
        if cut_regions.contains(&seed) {
            return true;
        }
    }

    false
}

/// Every face of every region a run's own fold-crossing walks already
/// send to the cut. Read before any side is settled, because the side that must join them may belong
/// to a run read later. (The memo is passed through unread.)
fn the_regions_some_seam_cuts(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    runs: &[SectionRun],
    _region_is_one_plane: &mut IndexMap<i32, bool>,
) -> HashSet<i32> {
    let mut cut = HashSet::new();

    for run in runs {
        if run.is_a_feature_edge.is_none() {
            continue;
        }

        for side in 0..2 {
            let on_the_left = side == 0;

            if !a_walk_crossed_a_fold(run, on_the_left) {
                continue;
            }

            for seed in the_sides_seed_faces(run, on_the_left) {
                for face in section_room::faces_of_the_flat_region(
                    mesh,
                    graph,
                    edge_at_pair,
                    seed,
                    None,
                    None,
                    run.is_a_feature_edge.as_deref(),
                ) {
                    cut.insert(face);
                }
            }
        }
    }

    cut
}

/// The advisory a refused smooth side answers - the place the application points at,
/// and the cut's OWN reason by name.
///
/// These used to be collapsed into `SurfaceCornerNotSupported` with the refusal dropped into a
/// `Debug.WriteLine`, which told a user that a facet with a hole in it was "a corner this bevel cannot
/// turn" and told a test nothing at all. The name is an argument, so it is part of the advisory's
/// value: a test may assert the exact refusal, and a log carries it without a debugger. The user's
/// sentence does not quote it - see `BevelAdvisoryText`.
fn refused(place: Vector3, refusal: SmoothSideRefusal) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceSmoothSideNotCut,
        // The refusal by its member name, which the derived `Debug` writes for these field-less
        // variants; the advisory text names the reason that way.
        vec![place.into(), format!("{refusal:?}").into()],
    )
}
