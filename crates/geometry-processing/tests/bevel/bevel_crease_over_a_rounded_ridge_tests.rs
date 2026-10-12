//! Port of `R/Tests/GeometryProcessingTests/BevelCreaseOverARoundedRidgeTests.cs`: the second pass
//! of a mixed bevel on the nose-in-notch part (`nose_in_notch_cut`) - round the ridge FIRST, then
//! read the two concave creases that died on it as the ONE curved crease the cut mesh now carries
//! (18 mesh edges: both straight pieces and eight arc facets either side of the crest), and roll one
//! bead along the whole of it.
//!
//! The subtract that makes the round leaves the cut shredded where the round runs TANGENT into the
//! faces it meets - slivers, micron-apart duplicate vertices and zero-area faces - so the cut is put
//! through `bevel_published_mesh::clean` before anything is read off it, and a crease walked over
//! the untidied mesh stops at the tangency points. And the crease's radius of curvature over the
//! round IS the round's radius, so the sweep pulls the bead's back in to what the tightest turn
//! allows (`BevelSweepBackDepth`) and keeps the run whole.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::BevelFaceFit;
use geometry_processing::bevel::{bevel_published_mesh, bevel_source_hull, bevel_working_mesh};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, FeatureEdgeChain, FeatureEdgeSet, MatchTolerances,
};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::math_helper::radians_to_degrees;
use vector_math::{Matrix4X4, Vector3};

use crate::nose_in_notch_cut::{cut_ridge, RidgeCut};
use crate::nose_in_notch_test_meshes as meshes;

/// The one radius, used for both the ridge round and the beads, as the fixtures ask for it.
const RADIUS: f64 = 2.0;

const SEGMENTS: i32 = 16;

/// The ridge's own dihedral turn, which the round spans in `SEGMENTS` facets.
const RIDGE_TURN_DEGREES: f64 = 112.62;

/// How far the crease may turn from one edge to the next: the round's own facet step, and a margin.
fn facet_step() -> f64 {
    RIDGE_TURN_DEGREES / f64::from(SEGMENTS)
}

/// Where the crease crosses the ridge plane, on the crest of the round.
fn crest() -> Vector3 {
    Vector3::new(13.3945, 6.6056, 10.0)
}

/// The 18 edges: one straight crease each side, and eight facets of the arc each side of the
/// crest.
const CREASE_EDGES: usize = 18;

/// Where the round runs tangent into the LOWER nose face, which is where the lower crease stops
/// being straight and becomes the arc. Measured off the cut part.
fn lower_tangency() -> Vector3 {
    Vector3::new(12.5039, 7.4962, 8.3359)
}

/// The same on the upper face.
fn upper_tangency() -> Vector3 {
    Vector3::new(12.5039, 7.4962, 11.6641)
}

/// How close to a tangency point a sliver has to be to be counted as one of ITS slivers.
const NEAR_TANGENCY: f64 = 1e-2;

/// How far a crease may swing from one edge to the next, as a tangent chain reads it.
const TANGENT_TOLERANCE_DEGREES: f64 = FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES;

/// What the facet step is allowed for arithmetic and for the ends of the arc.
const MARGIN: f64 = 2.0;

#[test]
fn the_two_creases_read_as_one_crease_over_the_round() {
    let RidgeCut {
        cut, upper, lower, ..
    } = cut_ridge_default();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &cut,
        &cut.get_mesh_edge_graph(),
        upper.flat_angle_degrees,
    ));

    let from_the_upper_pick = resolve_crease(&upper, features.clone());
    let from_the_lower_pick = resolve_crease(&lower, features);

    assert!(
        from_the_upper_pick.is_resolved(),
        "the upper crease still exists on the cut mesh, trimmed by the round"
    );
    assert!(
        from_the_lower_pick.is_resolved(),
        "and so does the lower one"
    );

    let chain = from_the_upper_pick.chain.as_ref().expect("resolved");
    assert_eq!(
        chain.count(),
        CREASE_EDGES,
        "the crease is the two straight pieces plus the eight arc facets either side of the crest"
    );

    let mut lower_indices: Vec<i32> = from_the_lower_pick
        .chain
        .as_ref()
        .expect("resolved")
        .edges()
        .iter()
        .map(|edge| edge.edge_index)
        .collect();
    let mut upper_indices: Vec<i32> = chain.edges().iter().map(|edge| edge.edge_index).collect();
    lower_indices.sort();
    upper_indices.sort();
    assert_eq!(
        lower_indices, upper_indices,
        "both picks name the same one crease, which is what lets a rebuild bead them as one run"
    );

    let ends = ends(&from_the_upper_pick);

    assert!(
        ends.iter()
            .any(|&end| (end - meshes::lower_crease_end()).length() < 1e-3),
        "the crease still dies on the notch's centre crease below the nose"
    );
    assert!(
        ends.iter()
            .any(|&end| (end - meshes::upper_crease_end()).length() < 1e-3),
        "and above it"
    );

    assert!(
        chain
            .edges()
            .iter()
            .any(|edge| (edge.to() - crest()).length() < 1e-3),
        "it runs over the crest of the round rather than stopping at the tangency point"
    );
}

/// What the subtract actually leaves along the tangency, and what the tidy takes away - because
/// the second pass reads a SURFACE off the cut part, and this is the difference between a surface
/// and a boolean's bookkeeping.
#[test]
fn the_boolean_shreds_the_cut_where_the_round_runs_tangent_into_a_face() {
    let RidgeCut { raw, .. } = cut_ridge_default();

    let tolerance = bevel_working_mesh::seam_tolerance_for(&raw);

    assert!(
        short_edge_count(&raw, tolerance, None) > 0,
        "the round crosses the faces it is tangent to at a vanishing angle, so the kernel cuts \
         slivers there rather than one edge"
    );

    assert!(
        short_edge_count(&raw, tolerance, Some(lower_tangency())) > 0,
        "and one of those fans sits on the lower crease's tangency point"
    );
    assert!(
        short_edge_count(&raw, tolerance, Some(upper_tangency())) > 0,
        "and another on the upper one"
    );

    assert!(
        duplicate_vertex_pairs(&raw, tolerance) > 0,
        "the slivers are pairs of vertices a seam tolerance apart, which is what makes them slivers"
    );
    assert!(
        zero_area_face_count(&raw, tolerance) > 0,
        "and the triangles between them have no area to speak of"
    );

    let cleaned = tidy(&raw);

    assert!(
        cleaned.faces.len() < raw.faces.len(),
        "the tidy is what takes them out - it is not a no-op on this mesh"
    );
    assert!(
        cleaned.is_manifold(),
        "and what it leaves is still a closed solid, or it would have handed the raw mesh back"
    );

    assert_eq!(
        short_edge_count(&cleaned, tolerance, None),
        0,
        "nothing shorter than a seam survives the tidy"
    );
    assert_eq!(
        duplicate_vertex_pairs(&cleaned, tolerance),
        0,
        "nor any pair of vertices that were always one vertex"
    );
    assert_eq!(
        zero_area_face_count(&cleaned, tolerance),
        0,
        "nor any triangle with no area"
    );
}

/// The dependency the second pass has on that tidy, stated as the thing that breaks without it:
/// the crease walk stops dead at the tangency the slivers sit on.
///
/// The untidied walk still RESOLVES - it is not that the pick is lost - which is what makes this
/// worth pinning. What comes back is the 16 facets of the arc and neither of the two straight
/// creases, ended on the tangency points instead of on the notch's centre crease, so a rebuild that
/// read it would bead the round and leave the creases either side of it bare.
///
/// `bevel_working_mesh::create_from` happens to weld this particular mesh as well, since the kernel
/// refuses the shredded operand - but that weld is CONDITIONAL on the kernel's verdict, so it is
/// not what the second pass may rely on. The unconditional tidy is (`BevelFeatureMeshBuilder`
/// cleans the cut part before it re-reads a crease on it).
#[test]
fn the_crease_walk_needs_the_cut_tidied_first() {
    let RidgeCut {
        raw, upper, lower, ..
    } = cut_ridge_default();

    let tidied = tidy(&raw);

    let over_the_tidied = walk(&tidied, &upper);

    assert!(
        over_the_tidied.is_resolved(),
        "the crease is there to be read on the tidied cut"
    );
    assert_eq!(
        over_the_tidied.chain.as_ref().expect("resolved").count(),
        CREASE_EDGES,
        "and it is the whole of it: both straight creases and the arc between them"
    );
    assert!(
        ends(&over_the_tidied)
            .iter()
            .any(|&end| (end - meshes::lower_crease_end()).length() < 1e-3),
        "ending where the crease itself does, on the notch's centre crease below the nose"
    );
    assert!(
        ends(&over_the_tidied)
            .iter()
            .any(|&end| (end - meshes::upper_crease_end()).length() < 1e-3),
        "and above it"
    );

    let over_the_raw = walk(&raw, &upper);

    assert!(
        over_the_raw.is_resolved(),
        "the pick still resolves on the untidied cut - the fault is in what it reaches, not in \
         whether it reads"
    );
    let raw_count = over_the_raw.chain.as_ref().expect("resolved").count();
    assert!(
        raw_count < CREASE_EDGES,
        "the sliver fan at the tangency reads as feature edges and the walk stops on it"
    );
    assert!(
        ends(&over_the_raw)
            .iter()
            .any(|&end| (end - lower_tangency()).length() < NEAR_TANGENCY),
        "what comes back is the arc, stopped at the lower tangency point"
    );
    assert!(
        ends(&over_the_raw)
            .iter()
            .any(|&end| (end - upper_tangency()).length() < NEAR_TANGENCY),
        "and at the upper one"
    );
    assert!(
        !ends(&over_the_raw).iter().any(|&end| {
            (end - meshes::lower_crease_end()).length() < 1e-3
                || (end - meshes::upper_crease_end()).length() < 1e-3
        }),
        "neither straight crease is on it, which is exactly the pair the second pass exists to join"
    );

    assert_eq!(
        walk(&raw, &lower).chain.as_ref().expect("resolved").count(),
        raw_count,
        "and the other pick is stopped by the same slivers"
    );
}

#[test]
fn every_joint_of_it_turns_no_more_than_a_facet_of_the_round() {
    let RidgeCut { cut, upper, .. } = cut_ridge_default();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &cut,
        &cut.get_mesh_edge_graph(),
        upper.flat_angle_degrees,
    ));
    let resolution = resolve_crease(&upper, features);
    let chain = resolution.chain.as_ref().expect("resolved");

    let mut worst_face: f64 = 0.0;
    let mut worst_direction: f64 = 0.0;

    for i in 1..chain.count() {
        let before = &chain.edges()[i - 1];
        let after = &chain.edges()[i];

        worst_face = worst_face.max(
            between(before.left_normal(), after.left_normal())
                .max(between(before.right_normal(), after.right_normal())),
        );
        worst_direction = worst_direction.max(between(before.direction(), after.direction()));
    }

    assert!(
        worst_face <= facet_step() + MARGIN,
        "one surface along the crease is the slope and never turns, and the other steps one facet \
         of the round at a time (worst {worst_face})"
    );

    // The crease is a curve cut ACROSS the round rather than around it, so its chord swings faster
    // than the round's own facet step where the curvature is highest - at the crest. That is why
    // the walk's direction tolerance is the tangent chain's number and the face tolerance is the
    // facet's.
    assert!(
        worst_direction < TANGENT_TOLERANCE_DEGREES,
        "the crease never turns a corner, but at the crest its chord swings half again as far as a \
         facet (worst {worst_direction})"
    );
}

#[test]
fn the_whole_crease_sweeps_as_one_bead_the_kernel_will_take() {
    let RidgeCut { cut, upper, .. } = cut_ridge_default();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &cut,
        &cut.get_mesh_edge_graph(),
        upper.flat_angle_degrees,
    ));
    let crease = resolve_crease(&upper, features);

    let fits = BevelFaceFit::create(
        cut.clone(),
        Rc::new(vec![crease.clone()]),
        true,
        RADIUS,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    )
    .try_check_group(0);

    assert!(
        fits.is_ok(),
        "a radius {RADIUS} bead fits this crease on the cut part ({:?})",
        fits.err().map(|does_not_fit| does_not_fit.code())
    );

    let built = bevel_group_tools::try_build(
        &crease,
        true,
        RADIUS,
        SEGMENTS,
        BevelGroupToolsOptions::default(),
    );

    let tools = match built {
        Ok(built) => built.tools,
        Err(failure) => panic!("the crease sweeps ({:?})", failure.reason.code()),
    };

    assert_eq!(
        tools.len(),
        1,
        "ONE bead over the whole crease - the run must not be chopped into a piece per facet, \
         which is what left the boolean a pile of near tangent beads to settle"
    );

    let bead = &tools[0];

    assert!(bead.is_bead, "a concave crease is filled, not cut");

    assert_eq!(
        classify_boolean_operand(&bead.mesh, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::Clean,
        "a bead that folds through itself is one the union will refuse"
    );

    let hull = bevel_source_hull::build(Some(&cut));
    assert!(
        bevel_source_hull::lies_inside(hull.as_ref(), Some(&bead.mesh)),
        "nothing of a bead may stand outside the part it is added to"
    );
}

/// C# `ResolveCrease(group, features, TangentToleranceDegrees, FacetStep + Margin)` at the
/// default match tolerances.
fn resolve_crease(
    group: &BevelEdgeGroup,
    features: Arc<FeatureEdgeSet>,
) -> BevelEdgeGroupResolution {
    bevel_edge_group_resolver::resolve_crease(
        group,
        features,
        TANGENT_TOLERANCE_DEGREES,
        facet_step() + MARGIN,
        MatchTolerances::default(),
    )
}

/// The crease this group anchors, as the walk reads it off `mesh`.
fn walk(mesh: &Mesh, group: &BevelEdgeGroup) -> BevelEdgeGroupResolution {
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        mesh,
        &mesh.get_mesh_edge_graph(),
        group.flat_angle_degrees,
    ));

    resolve_crease(group, features)
}

fn ends(resolution: &BevelEdgeGroupResolution) -> [Vector3; 2] {
    let chain = resolution.chain.as_ref().expect("resolved");

    [
        chain.edges()[0].from(),
        chain.edges()[chain.count() - 1].to(),
    ]
}

/// C# `BevelPublishedMesh.Clean`, which hands the mesh it was given back where there is nothing
/// to tidy (`None` in Rust).
fn tidy(raw: &Mesh) -> Mesh {
    bevel_published_mesh::clean(raw, &CancelToken::new())
        .expect("not cancelled")
        .unwrap_or_else(|| raw.clone())
}

/// How many of the mesh's edges are shorter than a seam - all of them, or only those with an end
/// on `near` when one is given.
fn short_edge_count(mesh: &Mesh, tolerance: f64, near: Option<Vector3>) -> usize {
    let graph = mesh.get_mesh_edge_graph();
    let mut count = 0;

    for i in 0..graph.edge_count() {
        let from = mesh.vertices[graph.get_vertex0(i) as usize].as_vector3();
        let to = mesh.vertices[graph.get_vertex1(i) as usize].as_vector3();

        if (from - to).length() < tolerance
            && near.is_none_or(|near| {
                (from - near).length() < NEAR_TANGENCY || (to - near).length() < NEAR_TANGENCY
            })
        {
            count += 1;
        }
    }

    count
}

fn duplicate_vertex_pairs(mesh: &Mesh, tolerance: f64) -> usize {
    let mut count = 0;

    for i in 0..mesh.vertices.len() {
        for j in (i + 1)..mesh.vertices.len() {
            if (mesh.vertices[i].as_vector3() - mesh.vertices[j].as_vector3()).length() < tolerance
            {
                count += 1;
            }
        }
    }

    count
}

/// How many faces are smaller than the tidy's own idea of a triangle worth keeping.
fn zero_area_face_count(mesh: &Mesh, tolerance: f64) -> usize {
    let smallest = bevel_working_mesh::minimum_face_area_for(tolerance);
    let mut count = 0;

    for face in mesh.faces.iter() {
        let a = mesh.vertices[face.v0 as usize].as_vector3();
        let b = mesh.vertices[face.v1 as usize].as_vector3();
        let c = mesh.vertices[face.v2 as usize].as_vector3();

        if (b - a).cross(c - a).length() / 2.0 < smallest {
            count += 1;
        }
    }

    count
}

fn between(a: Vector3, b: Vector3) -> f64 {
    radians_to_degrees(a.dot(b).clamp(-1.0, 1.0).acos())
}

/// The part with its ridge rounded and the two creases of its right end - see
/// `nose_in_notch_cut::cut_ridge`, which is where those steps live so that the second pass's own
/// tests can take the same cut part.
fn cut_ridge_default() -> RidgeCut {
    cut_ridge(RADIUS, SEGMENTS)
}
