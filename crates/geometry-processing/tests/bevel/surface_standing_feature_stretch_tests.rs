//! Port of `SurfaceStandingFeatureStretchTests.cs`: what the standing-feature reader will not read,
//! each refused by name - a root that FORKS (`surface_standing_feature_stretch::try_read`), a
//! finely tessellated CURVED wall that is not one face (`try_the_region_of`), and a face of the
//! feature that another band is cutting
//! (`surface_walkback_standing_features::none_of_the_feature_is_also_being_cut`).

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::surface_standing_feature_stretch::{
    standing_stretches, try_read, try_the_region_of,
};
use geometry_processing::bevel::surface::surface_walkback_standing_features::none_of_the_feature_is_also_being_cut;
use geometry_processing::bevel::surface::{
    SurfaceBandEnd, SurfaceBevelPlan, SurfaceFacetCut, SurfaceLine, SurfaceRegionWalkback,
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector3, Vector3Float};

use crate::surface_walkback_checks::{centre_of, regions_of};
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
