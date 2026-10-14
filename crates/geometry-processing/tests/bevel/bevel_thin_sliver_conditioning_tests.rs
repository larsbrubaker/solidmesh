//! The THIN SLIVER conditioning a working copy gets on its way in (`bevel_thin_slivers`), read on
//! the poke-through that needs it - `gusset_test_meshes::create_plate_with_gusset_poking_through`,
//! whose gusset tip the boolean wrote three times over within two microns.
//!
//! Two vertices nearer than the weld distance ARE one vertex - that is what
//! `bevel_working_mesh::seam_tolerance_for` means, and what the kernel's own seam weld does with
//! them - so the edge between them is collapsed and every triangle hanging on it goes. The part is
//! unchanged to within two microns; what changes is that its top stops being three surfaces with a
//! knife edge between them and goes back to being one.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_chain_join, surface_chain_opens_at_the_junction, surface_room, SurfaceBevelPlan,
};
use geometry_processing::bevel::{
    bevel_chain_linker, bevel_t_vertex_slivers, bevel_thin_slivers, bevel_working_mesh,
};
use geometry_processing::topology::{FaceGroup, FaceRegionCache};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::gusset_test_meshes;
use crate::mesh_measurements;

const RADIUS: f64 = 0.4;

/// The bend threshold the fixture's own operation carries.
const FLAT_ANGLE_DEGREES: f64 = 10.0;

/// The pass does its own job: every edge shorter than the weld distance is collapsed, the
/// triangles hanging on them go, and the part is the same part to within two microns.
///
/// Driven directly rather than through `bevel_working_mesh::create_from`, because on THIS fixture
/// the seam weld gets there first: the raw boolean output is one the kernel would refuse, so the
/// weld runs and closes the tip on its way past. What is held here is the pass's own reading of a
/// real one.
#[test]
fn the_sliver_edges_are_collapsed_as_the_one_point_they_are() {
    let mut part = gusset_test_meshes::create_plate_with_gusset_poking_through();
    let weld_distance = bevel_working_mesh::seam_tolerance_for(&part);
    let was = mesh_measurements::signed_volume(&part);
    let faces = part.faces.len() as i32;

    assert!(
        thinnest_face(&part) < weld_distance,
        "the fixture is the one with the sliver in it, {} thick against a weld distance of {weld_distance}",
        thinnest_face(&part)
    );

    let went = bevel_thin_slivers::collapsed_from(&mut part);

    assert_eq!(
        went,
        faces - part.faces.len() as i32,
        "what it reports is what it dropped"
    );

    assert_eq!(
        went, 6,
        "the gusset tip written three times over hangs six triangles between the readings"
    );

    assert!(
        thinnest_face(&part) < weld_distance,
        "and what is left for the pass AFTER it is a sliver of the other kind: collapsing the tip turns the long triangle beside it into an honestly collinear one, which is BevelTVertexSlivers' own subject - {} thin",
        thinnest_face(&part)
    );

    bevel_t_vertex_slivers::dropped_from(&mut part);

    assert!(
        thinnest_face(&part) > weld_distance,
        "and with both passes run, which is the order BevelWorkingMesh runs them in, nothing sliver-thin is left at all - the thinnest is {}",
        thinnest_face(&part)
    );

    assert!(
        part.is_manifold(),
        "collapsing an edge may not leave the part open"
    );

    assert_eq!(
        mesh_measurements::shell_count(&part),
        1,
        "nor break it into pieces"
    );

    assert!(
        (mesh_measurements::signed_volume(&part) - was).abs() < 1e-4,
        "and it is the same part: what moved, moved two microns"
    );
}

/// And the top is ONE surface again: the ten degree flood from the middle of the plate's top
/// reaches the gusset's top, which the sliver's knife edge stood across.
#[test]
fn the_top_floods_as_one_surface_once_the_sliver_is_gone() {
    let part = the_conditioned_fixture();
    let top = FaceGroup::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        the_top_seed_face(&part),
        FLAT_ANGLE_DEGREES,
    )
    .expect("the seed is a face of the part");

    assert!(
        top.faces().contains(&the_gussets_top_face(&part)),
        "the plate's top and the gusset's top are one plane to a thousandth of a degree, so one is the other's own surface and the pick has to reach it"
    );
}

/// And with the sliver gone the top face's rim FITS at the 0.4 it asks for: the band down the
/// plate's y = -6 wall has the whole 20 mm of that wall, and the band down the gusset's hypotenuse
/// the whole 20 of its own. (This was a pin at 0.3999824; `surface_room_corner_wall_tests` holds
/// both of its fixes on a block built to say so.)
#[test]
fn the_top_face_fits_at_the_size_it_asks_for_once_the_sliver_is_gone() {
    let part = the_conditioned_fixture();
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(part.get_mesh_edge_graph()),
        FLAT_ANGLE_DEGREES,
    ));
    let top =
        FaceGroup::create_with_graph(&part, &graph, the_top_seed_face(&part), FLAT_ANGLE_DEGREES)
            .expect("the seed is a face of the part");

    let chains = surface_chain_opens_at_the_junction::open(surface_chain_join::join(
        &bevel_chain_linker::link(top.boundary_edges()),
    ));

    let mut plans: Vec<Rc<SurfaceBevelPlan>> = Vec::new();

    for chain in &chains {
        let plan = SurfaceBevelPlan::try_create(chain, &regions, RADIUS, true);

        assert!(
            plan.is_ok(),
            "the rim plans at {RADIUS} - the sliver is what used to stop it: {:?}",
            plan.as_ref().err().map(|why| why.code())
        );

        plans.push(plan.expect("asserted above"));
    }

    let fit =
        surface_room::try_fit(&plans, RADIUS, &CancelToken::new(), None).expect("nothing cancels");

    assert!(
        fit.fits,
        "every face the rim lands on is 20 mm across: instead {:?} at {}",
        fit.refusal, fit.largest_fitting_size
    );

    assert!(
        (fit.largest_fitting_size - RADIUS).abs() <= 1e-9,
        "and what fits is what was asked for: instead {}",
        fit.largest_fitting_size
    );
}

/// The fixture with both source conditioning passes run over it, in the order
/// `bevel_working_mesh::create_from` runs them - the thin slivers collapsed and then the collinear
/// ones that leaves dropped. Not `CreateFrom` itself, because this fixture is raw boolean output
/// the kernel would refuse, so the seam weld would run first and do part of the work.
fn the_conditioned_fixture() -> Mesh {
    let mut part = gusset_test_meshes::create_plate_with_gusset_poking_through();

    bevel_thin_slivers::collapsed_from(&mut part);
    bevel_t_vertex_slivers::dropped_from(&mut part);

    part
}

/// The thinnest triangle of a mesh, as the shortest altitude any of them has.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` skips NaN too.
fn thinnest_face(mesh: &Mesh) -> f64 {
    let mut thinnest = f64::INFINITY;

    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        let longest = (b - a).length().max((c - b).length().max((a - c).length()));

        if !(longest > 0.0) {
            continue;
        }

        thinnest = thinnest.min((b - a).cross(c - a).length() / longest);
    }

    thinnest
}

/// A face in the middle of the plate's own top, well away from anything the gusset did.
fn the_top_seed_face(part: &Mesh) -> i32 {
    nearest_upward_face_to(part, Vector3::new(0.0, -8.0, 20.0))
}

/// A face of the gusset's own top, which the sliver's knife edge stood between.
fn the_gussets_top_face(part: &Mesh) -> i32 {
    nearest_upward_face_to(part, Vector3::new(29.0, -4.0, 20.0))
}

fn nearest_upward_face_to(part: &Mesh, want: Vector3) -> i32 {
    let mut best = -1;
    let mut nearest = f64::INFINITY;

    for (i, face) in part.faces.iter().enumerate() {
        if (face.normal.z as f64) < 0.99 {
            continue;
        }

        let centroid = (Vector3::from_vector3_float(part.vertices[face.v0 as usize])
            + Vector3::from_vector3_float(part.vertices[face.v1 as usize])
            + Vector3::from_vector3_float(part.vertices[face.v2 as usize]))
            / 3.0;

        let away = (centroid - want).length();

        if away < nearest {
            nearest = away;
            best = i as i32;
        }
    }

    best
}
