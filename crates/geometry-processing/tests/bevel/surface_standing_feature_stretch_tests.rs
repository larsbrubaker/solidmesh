//! What the standing-feature reader will not read, each refused by name - a root that FORKS
//! (`surface_standing_feature_stretch::try_read`), a feature under two bands of one region, a
//! finely tessellated CURVED wall that is not one face (`try_the_region_of`) and a face of the
//! feature that another band is cutting
//! (`surface_walkback_standing_features::none_of_the_feature_is_also_being_cut`) - and the one
//! feature two CHAINS' bands both reach, withheld once with both reaches (`surface_face_walkback`).

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::surface_face_walkback;
use geometry_processing::bevel::surface::surface_standing_feature_stretch::{
    standing_stretches, try_read, try_the_region_of,
};
use geometry_processing::bevel::surface::surface_walkback_standing_features::none_of_the_feature_is_also_being_cut;
use geometry_processing::bevel::surface::{
    SurfaceBandEnd, SurfaceBevelPlan, SurfaceFacetCut, SurfaceLine, SurfaceRegionWalkback,
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector3, Vector3Float};

use crate::nose_in_notch_test_meshes;
use crate::surface_walkback_checks::{centre_of, regions_of};
use crate::topology_test_meshes;
use crate::topology_test_meshes::create_forked_fins_on_a_slab;

/// A rim that FORKS is refused by name rather than closed on a chord. Two fins whose roots touch
/// at one tread vertex make four standing creases meet there, and no polygon can be drawn round
/// that: a walk takes whichever legs it meets first and what is left is neither a path nor a
/// ring, so closing it would shut the rest with a chord straight across the part.
///
/// Asked of the stretch reader with a cut made by hand, because the only shapes that fork a
/// region's rim also put a HOLE in it, and the walkback's own standing-feature guard turns those
/// down before the reader is ever reached. The fork guard is what stands behind that one, and
/// this is where it can be read.
#[test]
fn a_forked_root_is_refused_by_name_rather_than_closed_on_a_chord() {
    let part = Arc::new(create_forked_fins_on_a_slab());
    let (regions, _) = regions_of(&part);

    // The tread, and a cut 6 mm in off its left edge - far enough to have the pinch at
    // (4, 5, 20) inside the band.
    let tread_face = (0..part.faces.len() as i32)
        .find(|&face| {
            (part.faces[face as usize].normal.z as f64) > 0.99
                && centre_of(&part, face).z > 19.9
                && centre_of(&part, face).y < 10.0
        })
        .expect("a tread facet");
    let tread = regions.region(tread_face).expect("the tread's region");

    let mut walkback = SurfaceRegionWalkback::new(SurfaceBevelPlan::key_of(&tread), tread.clone());

    // Ended on the riser's own plane, which is where an unforked root on this part DOES close -
    // so the fork is the only thing left for the refusal to be about.
    walkback.cuts.push(SurfaceFacetCut::with_ends(
        0,
        true,
        SurfaceLine::new(Vector3::new(6.0, 0.0, 20.0), Vector3::new(0.0, 1.0, 0.0)),
        Vector3::new(1.0, 0.0, 0.0),
        tread.faces()[0],
        vec![SurfaceBandEnd::new(
            Vector3::new(0.0, 10.0, 20.0),
            Vector3::new(0.0, -1.0, 0.0),
        )],
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(0.0, 10.0, 20.0),
        0,
        None,
    ));
    walkback
        .reaches
        .push(tread.faces().iter().copied().collect::<HashSet<i32>>());

    let pool = SurfaceVertexPool::new(&part.vertices);

    let branching: Vec<_> = standing_stretches(&walkback, &HashSet::new(), &HashSet::new())
        .into_iter()
        .filter(|stretch| stretch.is_branching)
        .collect();
    assert_eq!(branching.len(), 1, "one forked stretch");

    let reason = try_read(&branching[0], false, &pool).expect_err(
        "the rim forks at the shared apex, so there is no telling which facet stands across which \
         crease",
    );
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotClose
    );
}

/// And one feature under TWO of the pass's bands is refused by name too. Both bands uncover part of
/// the same root and the two parts overlap, so closing them one band at a time would lay the same
/// triangle down twice - a fold, not an island.
#[test]
fn a_feature_under_two_bands_is_refused_by_name_rather_than_closed_twice() {
    let part = Arc::new(topology_test_meshes::create_fin_on_a_slab());
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);

    // The tread's left edge and its front edge, picked as one mitred pair: both bands land on the
    // tread and at 7 mm both of them reach the fin's root.
    let pair = [
        features.edges()[top_edge_of(
            &features,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(0.0, 10.0, 20.0),
        )],
        features.edges()[top_edge_of(
            &features,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(20.0, 0.0, 20.0),
        )],
    ];

    let chains = bevel_chain_linker::link(&pair);
    assert_eq!(chains.len(), 1);

    let plan =
        SurfaceBevelPlan::try_create(&chains[0], &regions, 7.0, false).unwrap_or_else(|reason| {
            panic!("the pair has to plan before it can be walked back: {reason}")
        });

    let reason =
        surface_face_walkback::try_apply_one(&part, &part.get_mesh_edge_graph(), &regions, &plan)
            .expect_err("both bands of the pair reach the one fin");
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts
    );
}

/// ONE FEATURE UNDER TWO CHAINS. The nose of the nose-in-notch part stands out of both slopes, and
/// at 3.4 the two knife tips' own vertical bands each reach it - the nose's ridge ends stand 7.071
/// along each slope from its tip edge and the band reaches 8.208 - so the walkback holds the nose
/// back ONCE and records that TWO bands reached it, rather than refusing because one facet would be
/// held back twice.
#[test]
fn a_feature_under_two_chains_tip_bands_is_withheld_once_with_both_reaches() {
    let part = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);
    let size = nose_in_notch_test_meshes::SIZE;

    let plans = [
        plan_of(
            &regions,
            &features,
            top_edge_of(
                &features,
                Vector3::new(size, 0.0, 0.0),
                Vector3::new(size, 0.0, size),
            ),
            3.4,
            true,
        ),
        plan_of(
            &regions,
            &features,
            top_edge_of(
                &features,
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, size),
            ),
            3.4,
            true,
        ),
    ];

    let walked =
        surface_face_walkback::try_apply(&part, &part.get_mesh_edge_graph(), &regions, &plans)
            .unwrap_or_else(|reason| {
                panic!(
            "one nose reached by two tip bands is one feature with two reaches - instead {reason}"
        )
            });

    assert_eq!(
        walked.standing_features.len(),
        1,
        "the nose is ONE feature however many bands undercut it"
    );

    let nose = &walked.standing_features[0];
    let reaches = nose.reaches.borrow();

    assert_eq!(
        reaches.len(),
        2,
        "and both tip bands reached it, which is what the record has to remember"
    );

    let edges: HashSet<i32> = reaches.iter().map(|reach| reach.0).collect();
    assert_eq!(
        edges.len(),
        2,
        "the two reaches are two different chains' edges, not one edge counted twice"
    );

    assert_eq!(
        nose.faces.len(),
        2,
        "the nose is two faces - the upper and the lower - however many bands reached them"
    );

    for face in &nose.faces {
        let creases = face.creases.borrow();

        assert_eq!(
            creases.len(),
            2,
            "each face lost its footing on one run of crease per band: its right end and its left"
        );
        assert_eq!(
            face.withins.borrow().len(),
            2,
            "and what stands inside each of those runs is that run's own"
        );
        assert_ne!(
            creases[0], creases[1],
            "the two runs are at opposite ends of the nose and cannot be the same two vertices"
        );
    }

    // The four crease ends the two reaches name between them are the nose's own two ridge ends and
    // the two places the creases die on the notch's centre crease - measured, so that a merge which
    // appended the WRONG record's run would be seen rather than counted.
    let named: Vec<Vector3> = nose
        .faces
        .iter()
        .flat_map(|face| face.creases.borrow().clone())
        .flat_map(|(from, to)| [from, to])
        .map(|vertex| walked.mesh.vertices[vertex as usize].as_vector3())
        .collect();

    for end in [
        nose_in_notch_test_meshes::left_ridge_end(),
        nose_in_notch_test_meshes::right_ridge_end(),
        nose_in_notch_test_meshes::upper_crease_end(),
        nose_in_notch_test_meshes::lower_crease_end(),
    ] {
        assert!(
            named.iter().any(|&at| (at - end).length() < 1e-4),
            "the four runs between them stand on {end:?}"
        );
    }
}

/// A finely tessellated CURVED wall is not one face, however small its steps are. The flood that
/// reads a feature's face crosses anything flat to within half a degree, so a wall stepping 0.3
/// degrees a facet is crossed at every step and the whole curve arrives as one "face" - and a face
/// is extended as a PLANE and published as one flat outline, which would silently straighten the
/// curve. Refused by name instead. Asked of the region reader with a wall built by hand, so the
/// guard working cannot be confused with the part being out of reach.
#[test]
fn a_curved_wall_tessellated_finely_enough_to_flood_is_not_one_face() {
    let flat = a_strip_of_wall(0.0);
    let curved = a_strip_of_wall(0.3);

    let flat_pool = SurfaceVertexPool::new(&flat.vertices);

    let whole = try_the_region_of(
        &flat,
        &flat.get_mesh_edge_graph(),
        (flat.faces.len() / 2) as i32,
        &flat_pool,
    )
    .expect("a FLAT wall cut into triangles is one face however many pieces it is in");

    assert_eq!(
        whole.len(),
        flat.faces.len(),
        "and the flood takes the whole of it - that is what makes the region the unit"
    );

    let curved_pool = SurfaceVertexPool::new(&curved.vertices);

    let failure = try_the_region_of(
        &curved,
        &curved.get_mesh_edge_graph(),
        (curved.faces.len() / 2) as i32,
        &curved_pool,
    )
    .expect_err(
        "a wall turning 0.3 degrees a facet is crossed at every step and would arrive as one \
         face, twenty times further from its seed than the flat bound allows",
    );

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
        "and it is named rather than flattened"
    );
}

/// And a face of the feature that another band is cutting is refused by name. A reading of the
/// check itself, with the two answers handed to it: what has to be pinned here is that the
/// walkback asks the question at all, and that it names what it finds.
#[test]
fn a_face_of_the_feature_that_another_band_is_cutting_is_refused_by_name() {
    let pool = SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(1.0, 0.0, 0.0),
        Vector3Float::new(0.0, 1.0, 0.0),
    ]);

    let standing = vec![Rc::new(SurfaceStandingFeatureWithheld::new(
        (0, true),
        Vector3::UNIT_Z,
        vec![Rc::new(SurfaceStandingFeatureWithheldFace::new(
            vec![3, 4],
            (0, 1),
            Vec::new(),
            Vector3::ZERO,
            Vector3::UNIT_Z,
        ))],
    ))];

    // Face 4 of the region is inside the region region 7 is being walked back as.
    let cut = [-1, -1, -1, -1, 7, -1];
    let clear = [-1, -1, -1, -1, -1, -1];

    assert!(
        none_of_the_feature_is_also_being_cut(&standing, &clear, &pool).is_ok(),
        "a feature whose faces no band lands on is held back and extended, as every one measured \
         so far is"
    );

    let failure = none_of_the_feature_is_also_being_cut(&standing, &cut, &pool).expect_err(
        "but one of its faces is the very ground another band is taking, and it cannot be both \
         carried out whole and cut back",
    );

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
}

/// A strip of wall 20 facet pairs long, each pair turned `turn_per_facet_degrees` from the last
/// about the y axis - which is flat at zero and a tessellated cylinder otherwise. Open at the rim,
/// which is all the flood needs: the edges it walks across are the inner ones.
fn a_strip_of_wall(turn_per_facet_degrees: f64) -> Mesh {
    let mut mesh = Mesh::new();
    let radius = 40.0;

    let at = |step: i32, y: f64| {
        let angle = math_helper::degrees_to_radians(turn_per_facet_degrees * step as f64);

        if turn_per_facet_degrees == 0.0 {
            Vector3::new(step as f64, y, 0.0)
        } else {
            Vector3::new(radius * angle.sin(), y, radius * (1.0 - angle.cos()))
        }
    };

    for step in 0..20 {
        mesh.create_face(&[at(step, 0.0), at(step + 1, 0.0), at(step + 1, 5.0)]);
        mesh.create_face(&[at(step, 0.0), at(step + 1, 5.0), at(step, 5.0)]);
    }

    mesh.clean_and_merge();

    mesh
}

/// One picked edge, planned on both the regions it lands on.
pub fn plan_of(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    pick: usize,
    size: f64,
    round_profile: bool,
) -> Rc<SurfaceBevelPlan> {
    let picked: Vec<FeatureEdge> = vec![features.edges()[pick]];
    let chains = bevel_chain_linker::link(&picked);
    assert_eq!(chains.len(), 1);

    SurfaceBevelPlan::try_create(&chains[0], regions, size, round_profile)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason}"))
}

/// The edge between two named points, whichever way round the mesh happens to hold
/// it.
pub fn top_edge_of(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&index| {
            let edge = &features.edges()[index];

            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6)
        })
        .collect();

    assert_eq!(found.len(), 1, "one edge between the two points");

    found[0]
}
