//! What a rim edge leaving the chain edge's own END
//! CORNER is worth to the room across the face (`surface_room_measure::measure_corner`). At a
//! square corner it is worth nothing. The block is a plain box whose one vertical rim carries an
//! extra vertex 0.4 below the top - the way a boolean leaves one where another solid's face arrived
//! at the rim - which a wrong reading takes for a face 0.4 wide.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{surface_room, surface_room_measure, SurfaceBevelPlan};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3Float;

/// A mesh keeps its vertices as floats, so a distance of 10 is a distance of 10 to about this.
const NEAR: f64 = 1e-5;

/// How far below the top the stray vertex splits the vertical rim.
const SPLIT_BELOW_THE_TOP: f64 = 0.4;

/// And how far INTO the face it misses that rim by - the order a boolean writes a point that
/// should have been on a line.
const SPLIT_MISSES_THE_RIM_BY: f64 = 1e-5;

/// The band down the front wall has the whole 10 of it, not the 0.4 of the rim edge that leaves
/// its end corner.
#[test]
fn a_rim_leaving_the_end_corner_square_is_no_wall_in_front_of_the_band() {
    let block = block_with_a_split_rim();
    let plan = plan_of(&block, top_front_edge(&block), 2.0);

    for cut in surface_room_measure::measure(std::slice::from_ref(&plan), None, None)
        .iter()
        .filter(|cut| !cut.corner_footprint())
    {
        assert!(
            (cut.room - 10.0).abs() <= NEAR,
            "both the top and the front wall are 10 across, and this one measured {}",
            cut.room
        );
    }

    let fit = surface_room::try_fit(&[plan], 2.0, &CancelToken::new(), None).expect("no cancel");
    assert!(
        fit.fits,
        "a 2 mm round has 10 mm of face to land in: instead {:?} at {}",
        fit.refusal, fit.largest_fitting_size
    );
    assert!((fit.largest_fitting_size - 2.0).abs() <= NEAR);
}

/// And the wall is still the wall it is: a size wider than the face is refused with the 10 the
/// face really has.
#[test]
fn the_face_beyond_it_is_still_measured_at_its_own_width() {
    let block = block_with_a_split_rim();
    let plan = plan_of(&block, top_front_edge(&block), 12.0);

    let fit = surface_room::try_fit(&[plan], 12.0, &CancelToken::new(), None).expect("no cancel");
    assert!(
        !fit.fits,
        "12 mm of setback does not fit across 10 mm of face"
    );
    assert_eq!(
        fit.refusal.as_ref().expect("a refusal").code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth
    );
    assert!(
        (fit.largest_fitting_size - 10.0).abs() <= NEAR,
        "the far side of the face is where it always was: instead {}",
        fit.largest_fitting_size
    );
}

/// A block 20 x 10 x 10 with one vertical rim - the one at x = 20 on the front wall - split by a
/// vertex 0.4 below the top. Built by index and deliberately not cleaned.
fn block_with_a_split_rim() -> Mesh {
    let vertices = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(20.0, 0.0, 0.0),
        Vector3Float::new(20.0, 10.0, 0.0),
        Vector3Float::new(0.0, 10.0, 0.0),
        Vector3Float::new(0.0, 0.0, 10.0),
        Vector3Float::new(20.0, 0.0, 10.0),
        Vector3Float::new(20.0, 10.0, 10.0),
        Vector3Float::new(0.0, 10.0, 10.0),
        Vector3Float::new(
            (20.0 - SPLIT_MISSES_THE_RIM_BY) as f32 as f64,
            0.0,
            (10.0 - SPLIT_BELOW_THE_TOP) as f32 as f64,
        ),
    ];

    let triangles = [
        // z = 0 and z = 10
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        // y = 0, fanned from the corner away from the split so that no triangle is collinear
        (0, 1, 8),
        (0, 8, 5),
        (0, 5, 4),
        // x = 20, fanned the same way about the split
        (6, 5, 8),
        (6, 8, 1),
        (6, 1, 2),
        // y = 10 and x = 0
        (3, 6, 2),
        (3, 7, 6),
        (0, 4, 7),
        (0, 7, 3),
    ];

    let mut mesh = Mesh::new();

    mesh.vertices.extend_from_slice(&vertices);

    for (a, b, c) in triangles {
        mesh.faces.add(a, b, c, &mesh.vertices);
    }

    mesh
}

/// The top edge of the front wall: (0, 0, 10) to (20, 0, 10), whose end corner is split.
fn top_front_edge(mesh: &Mesh) -> usize {
    let features = FeatureEdgeSet::create(mesh);

    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];
            (edge.start.z - 10.0).abs() < 1e-4
                && (edge.end.z - 10.0).abs() < 1e-4
                && edge.start.y.abs() < 1e-4
                && edge.end.y.abs() < 1e-4
        })
        .collect();
    assert_eq!(found.len(), 1, "exactly one feature edge matches");
    found[0]
}

fn plan_of(mesh: &Mesh, edge_index: usize, size: f64) -> Rc<SurfaceBevelPlan> {
    let features = FeatureEdgeSet::create(mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let mut chains = bevel_chain_linker::link(&[features.edges()[edge_index]]);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    SurfaceBevelPlan::try_create(&chain, &regions, size, true).unwrap_or_else(|reason| {
        panic!("the fixture has to plan before its room can be measured: {reason:?}")
    })
}
