//! Chains picked one at a time that turn out to be a whole
//! RIM - joined end to end they come back to where they started, and `SurfaceChainJoin` has to
//! publish that as the CLOSED loop it is.
//!
//! The join works pairwise over the ends standing at each vertex, and the last pair of a ring is the
//! pair whose two ends belong to the same chain by then - which it skips. Left open, the ring is
//! published with `from_vertex(first) == to_vertex(last)`, and everything downstream reads that one
//! vertex as TWO free chain ends. Four picked edges around a face are the smallest shape that says
//! it. The cube rim and edge readers are the ones `surface_two_face_rims_tests.rs` and
//! `surface_chain_opens_at_the_junction_tests.rs` share.

use std::sync::Arc;

use geometry_processing::bevel::surface::{surface_bevel_builder, SurfaceBevelPlan};
use geometry_processing::bevel::{bevel_chain_linker, BevelSweptChain};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::signed_volume;
use crate::surface_chain_opens_at_the_junction_tests::edge_between;
use crate::surface_test_chains::join_and_open;
use crate::surface_two_face_rims_tests::{assert_it_is_a_sound_solid, rim_of};
use crate::topology_test_meshes::{create_l_shaped_grid_prism, create_pyramid_pocket_prism};

const CUBE_SIZE: f64 = 20.0;

const SIZE: f64 = 3.0;

/// The radius the L prism's step wall is rounded at - the mixed-corner fixture's own.
const WALL_SIZE: f64 = 1.0;

/// The pocket prism's rim circumradius, its height, and how far its dimple drops.
const POCKET_RADIUS: f64 = 6.0;

const POCKET_HEIGHT: f64 = 8.0;

const POCKET_APEX: f64 = 4.0;

/// The four edges of one face of a cube, each picked on its own, join into ONE chain - and that
/// chain is closed, with no vertex standing as an end of it.
#[test]
fn four_chains_around_one_face_join_into_one_closed_ring() {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let separately = each_edge_on_its_own(&cube, Vector3::UNIT_Z);

    assert_eq!(
        separately.len(),
        4,
        "a cube face has four edges and nobody shares one with anybody here"
    );

    let joined = join_and_open(&separately);

    assert_eq!(joined.len(), 1, "the four run end to end around the face");

    assert_eq!(joined[0].edges.len(), 4);

    assert!(
        joined[0].is_closed,
        "the ring the join itself assembled comes back to where it started, which is a loop and not a run"
    );

    assert_eq!(
        joined[0].edges[0].from_vertex(),
        joined[0].edges[3].to_vertex(),
        "a closed chain's ends are the same vertex, which is why leaving it open reads as two free ends there"
    );
}

/// The same ring chamfered, held to the closed form: four swept sections less the four mitres that
/// would each be counted twice - `surface_two_face_rims_tests`'s terms, one rim's worth.
#[test]
fn the_joined_ring_chamfers_to_its_closed_form() {
    let built = build(false, 1);

    assert_it_is_a_sound_solid(&built);

    let removed = (4.0 * CUBE_SIZE * (SIZE * SIZE / 2.0)) - (4.0 * (SIZE * SIZE * SIZE / 3.0));

    assert!((removed - 324.0).abs() <= 1e-9, "instead {removed}");

    let volume = signed_volume(&built);
    assert!(
        (volume - (8000.0 - removed)).abs() <= 8000.0 * 1e-6,
        "four prisms and four mitres, which is the same solid whether the rim was picked at once or \
         an edge at a time - instead {volume}"
    );
}

/// The ring filleted is the SAME solid as the face pick's own closed loop. No closed form is needed
/// to say it: how the rim was picked cannot change what the fillet takes off it.
#[test]
fn the_joined_ring_fillets_to_what_the_face_pick_builds() {
    let built = build(true, 8);

    assert_it_is_a_sound_solid(&built);

    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let as_one_loop = bevel_chain_linker::link(&rim_of(&cube, Vector3::UNIT_Z));

    assert_eq!(as_one_loop.len(), 1);
    assert!(
        as_one_loop[0].is_closed,
        "picked as a face, the rim arrives closed and the join has nothing to do"
    );

    let at_once = built_from(&cube, &as_one_loop, true, 8);

    let (volume, at_once) = (signed_volume(&built), signed_volume(&at_once));
    assert!(
        (volume - at_once).abs() <= 1e-9,
        "the ring the join assembled is the loop the face pick hands over - instead {volume} against {at_once}"
    );
}

/// A ring with a CONCAVE edge in it, which is where the wrap pair has to be read as a corner rather
/// than passed over: the four edges round the L prism's step wall, picked one at a time. The mixed
/// corner at each end of the reflex edge is a blend the construction only reaches by reading the
/// pair that wraps from the last planned edge back to the first.
#[test]
fn a_mixed_ring_joined_from_four_chains_reads_its_wrap_corner() {
    let prism = create_l_shaped_grid_prism();
    let features = FeatureEdgeSet::create(&prism);

    let wall = [
        edge_between(
            &features,
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(10.0, 20.0, 10.0),
        ),
        edge_between(
            &features,
            Vector3::new(10.0, 20.0, 10.0),
            Vector3::new(10.0, 20.0, 0.0),
        ),
        edge_between(
            &features,
            Vector3::new(10.0, 20.0, 0.0),
            Vector3::new(10.0, 10.0, 0.0),
        ),
        edge_between(
            &features,
            Vector3::new(10.0, 10.0, 0.0),
            Vector3::new(10.0, 10.0, 10.0),
        ),
    ];

    assert_eq!(
        wall.iter()
            .filter(|edge| edge.convexity == EdgeConvexity::Concave)
            .count(),
        1,
        "the wall's rim runs down the L's one reflex edge and round three convex ones"
    );

    let separately: Vec<BevelSweptChain> = wall
        .iter()
        .flat_map(|edge| bevel_chain_linker::link(&[*edge]))
        .collect();
    let joined = join_and_open(&separately);

    assert_eq!(joined.len(), 1);
    assert!(
        joined[0].is_closed,
        "mixed convexity does not stop a ring being a ring - what it needs is the wrap pair read as the corner it is"
    );

    let graph = prism.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    let plan = SurfaceBevelPlan::try_create(&joined[0], &regions, WALL_SIZE, true)
        .unwrap_or_else(|reason| panic!("the plan refused the ring: {reason}"));

    assert!(
        plan.try_mixed_corner_after(plan.edges.len() as i32 - 1)
            .is_some(),
        "the corner between the last planned edge and the first is a mixed one, and an open chain would never have looked at it"
    );

    let built = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &[plan],
        8,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the ring: {reason:?}"))
    .mesh;

    assert_it_is_a_sound_solid(&built);
}

/// A ring that comes back to its start with its two surfaces SWAPPED there is NOT a loop, and the
/// join has to say so however the ring was picked. The pocket prism's rim carries the pocket wall
/// on one side of a vertical edge and on the other side of the crease it meets at the rim, so the
/// run cannot carry one frame round - which is exactly why the linker, handed all five edges as one
/// group, breaks them into two open runs rather than a closed one.
#[test]
fn a_ring_whose_surfaces_swap_where_it_closes_stays_open() {
    let pocket = create_pyramid_pocket_prism(
        4,
        POCKET_RADIUS,
        POCKET_HEIGHT,
        Vector3::new(0.0, 0.0, POCKET_APEX),
    );
    let features = FeatureEdgeSet::create(&pocket);

    let ring = vec![
        edge_between(
            &features,
            Vector3::new(0.0, 6.0, 0.0),
            Vector3::new(6.0, 0.0, 0.0),
        ),
        edge_between(
            &features,
            Vector3::new(6.0, 0.0, 0.0),
            Vector3::new(6.0, 0.0, 8.0),
        ),
        edge_between(
            &features,
            Vector3::new(6.0, 0.0, 8.0),
            Vector3::new(0.0, 0.0, 4.0),
        ),
        edge_between(
            &features,
            Vector3::new(0.0, 0.0, 4.0),
            Vector3::new(0.0, 6.0, 8.0),
        ),
        edge_between(
            &features,
            Vector3::new(0.0, 6.0, 8.0),
            Vector3::new(0.0, 6.0, 0.0),
        ),
    ];

    let as_one_group = bevel_chain_linker::link(&ring);

    assert_eq!(
        as_one_group.len(),
        2,
        "picked at once, the linker stops where the surfaces swap and hands back two runs"
    );

    assert!(
        as_one_group.iter().all(|chain| !chain.is_closed),
        "neither piece is a loop, and the ring they lie on is not one either"
    );

    let joined = join_and_open(&as_one_group);

    assert_eq!(
        joined.len(),
        1,
        "the two runs still meet end to end, and joining them is what the join is for"
    );

    assert_eq!(joined[0].edges.len(), 5);

    assert!(
        !joined[0].is_closed,
        "the joint that closes it swaps the surfaces over, so the same edges read the same whether \
         they were picked as one group or a piece at a time"
    );
}

/// That rim's edges as the chains FOUR separate picks make of them - which is what
/// `SurfaceSharedEdgeNetwork` hands over for four groups that share no edge with each other.
fn each_edge_on_its_own(cube: &Mesh, normal: Vector3) -> Vec<BevelSweptChain> {
    rim_of(cube, normal)
        .iter()
        .flat_map(|edge| bevel_chain_linker::link(&[*edge]))
        .collect()
}

fn build(round_profile: bool, segments_per_quarter: i32) -> Mesh {
    let cube = create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);

    built_from(
        &cube,
        &each_edge_on_its_own(&cube, Vector3::UNIT_Z),
        round_profile,
        segments_per_quarter,
    )
}

/// The chains joined, planned and built exactly as `BevelSurfacePartBuilder` does it.
fn built_from(
    cube: &Mesh,
    chains: &[BevelSweptChain],
    round_profile: bool,
    segments_per_quarter: i32,
) -> Mesh {
    let graph = cube.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(cube.clone()),
        Some(graph.clone()),
        FeatureEdge::default_flat_angle_degrees(),
    ));

    let plans: Vec<_> = join_and_open(chains)
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, &regions, SIZE, round_profile)
                .unwrap_or_else(|reason| panic!("the plan refused a chain: {reason}"))
        })
        .collect();

    surface_bevel_builder::try_build(
        cube,
        &graph,
        &regions,
        &plans,
        segments_per_quarter,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .unwrap_or_else(|reason| panic!("the build refused: {reason:?}"))
    .mesh
}
