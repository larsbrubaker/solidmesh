//! TWO face rims that share an edge, bevelled together. In polygon space one mesh edge is ONE strip
//! between its two faces, whichever groups name it, so the two rims are one network of chains
//! rather than two selections to be told apart.
//!
//! The boolean construction cannot have it that way - two coincident swept tools cannot be unioned,
//! so `bevel_shared_edges::award_once` gives each doubly named edge to one group and takes it out
//! of the other. The surface path unions the groups' edges and links the union instead, which is
//! what the other tests here measure - and
//! `taking_the_shared_edge_out_of_one_rim_leaves_the_same_solid` is what the award costs it. The
//! union is a real network and not just a longer list: the two ends of the shared edge are vertices
//! where THREE selected edges meet, which is the three-edge corner the kernel already builds
//! (`surface_corner_tests`, whose closed-form profile terms are shared here).

use std::collections::HashSet;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::bevel::{bevel_chain_linker, BevelSweptChain};
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdge};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_corner_tests::{fillet_profile, mitred_corner_overlap, section_area};
use crate::surface_test_chains::join_and_open;

const CUBE_SIZE: f64 = 20.0;

const SIZE: f64 = 3.0;

/// The union of the two rims is the seven distinct edges of the two faces, and it links into three
/// runs meeting at the two ends of the edge they share.
#[test]
fn two_adjacent_face_rims_union_into_one_network_of_chains() {
    let cube = the_cube();

    let top = rim_of(&cube, Vector3::UNIT_Z);
    let front = rim_of(&cube, -Vector3::UNIT_Y);

    assert_eq!(top.len(), 4);
    assert_eq!(front.len(), 4);

    let front_indices: Vec<i32> = front.iter().map(|edge| edge.mesh_edge_index).collect();
    let shared: Vec<i32> = top
        .iter()
        .map(|edge| edge.mesh_edge_index)
        .filter(|index| front_indices.contains(index))
        .collect();

    assert_eq!(
        shared.len(),
        1,
        "the top and the front of a cube meet along exactly one edge, which is the case this \
         class is about"
    );

    let union = union(&[&top, &front]);

    assert_eq!(
        union.len(),
        7,
        "four and four with the shared edge counted once"
    );

    let chains = bevel_chain_linker::link(&union);

    assert_eq!(
        chains.len(),
        3,
        "the linker will not walk a vertex three selected edges meet at, so the shared edge and \
         the two rims' remainders are three runs"
    );

    assert_eq!(
        chains.iter().filter(|chain| chain.edges.len() == 1).count(),
        1,
        "the shared edge stands alone between the two junctions"
    );
    assert_eq!(
        chains.iter().filter(|chain| chain.edges.len() == 3).count(),
        2,
        "each rim's other three edges walk through their own two corners and stop at the junctions"
    );
}

/// The two rims chamfered together, held to the closed form: seven swept sections, less what the
/// four ordinary mitres would count twice, less what the two three-edge corners would.
///
/// Every term is `surface_corner_tests`'s, applied once per corner of this network. A mitre between
/// two edges sharing a face stops `d^3 / 3` being counted twice. At a three-edge corner the three
/// prisms would take `1.5 d^3` of the corner cell between them and the cell really gives up
/// `d^3 - d^3 / 4`, so `0.75 d^3` of it is counted twice. That comes to
/// `7 * 20 * 4.5 - 4 * 9 - 2 * 20.25 = 553.5`.
#[test]
fn two_adjacent_face_rims_chamfer_to_their_closed_form() {
    let built = build(both_rims, false, 1);

    assert_it_is_a_sound_solid(&built);

    let removed = (7.0 * CUBE_SIZE * (SIZE * SIZE / 2.0))
        - (4.0 * (SIZE * SIZE * SIZE / 3.0))
        - (2.0 * (0.75 * SIZE * SIZE * SIZE));

    assert!((removed - 553.5).abs() <= 1e-9);

    let volume = signed_volume(&built);
    assert!(
        (volume - (8000.0 - removed)).abs() <= 8000.0 * 1e-6,
        "seven prisms, four mitres and two three-edge corners, each of them a term the corner tests \
         already measure on its own - instead {volume}"
    );
}

/// The same network filleted. The two junctions take the sphere octant rather than the mitre
/// point, which is the only term that changes shape, and every vertex of both patches stands a
/// radius from its own ball centre.
#[test]
fn two_adjacent_face_rims_fillet_into_two_sphere_octants() {
    const SEGMENTS: i32 = 8;

    let built = build(both_rims, true, SEGMENTS);

    assert_it_is_a_sound_solid(&built);

    let profile = fillet_profile(SIZE, std::f64::consts::PI / 2.0, SEGMENTS);
    let section = section_area(&profile);

    let removed = (7.0 * CUBE_SIZE * section)
        - (4.0 * mitred_corner_overlap(&profile))
        - (2.0
            * ((3.0 * SIZE * section) - (SIZE * SIZE * SIZE)
                + (std::f64::consts::PI * SIZE * SIZE * SIZE / 6.0)));

    let volume = signed_volume(&built);
    assert!(
        (volume - (8000.0 - removed)).abs() <= 1.0,
        "the chamfer's argument over the chorded profile, with each junction cell keeping the \
         ball's own octant - instead {volume} against {}",
        8000.0 - removed
    );

    for centre in [
        Vector3::new(
            (CUBE_SIZE / 2.0) - SIZE,
            (-CUBE_SIZE / 2.0) + SIZE,
            (CUBE_SIZE / 2.0) - SIZE,
        ),
        Vector3::new(
            (-CUBE_SIZE / 2.0) + SIZE,
            (-CUBE_SIZE / 2.0) + SIZE,
            (CUBE_SIZE / 2.0) - SIZE,
        ),
    ] {
        let patch = faces_within(&built, centre, SIZE + 1e-5);

        assert_eq!(
            patch.len() as i32,
            (4 * SEGMENTS * SEGMENTS) - (3 * SEGMENTS),
            "the octant at {centre:?} is gridded on its own arcs' sample count - n squared \
             triangles - and one level of INTERIOR refinement cuts each of them in four but for the \
             edges of the RIM, which may not be split: one child fewer for every rim edge a triangle \
             carries, and the three at the corners carry two apiece - 4T less R, which for three \
             rings of n chords is 4n squared less 3n"
        );

        for at in patch.iter().flatten() {
            assert!(
                ((*at - centre).length() - SIZE).abs() <= 1e-6,
                "every point of the patch stands a radius from the ball centre, and {at:?} does not"
            );
        }
    }
}

/// What the shared-edge AWARD leaves the surface construction: give the shared edge to the top rim
/// only and the front rim becomes an open run of three whose two ends stand at two corners of the
/// top rim. Those corners YIELD their mitre - three convex edges at a vertex are the ball's octant
/// however they are grouped - so the same seven edges leave the same solid whichever rim was given
/// the edge they share.
///
/// This used to be the reason the union exists: the open run's ends fell in the middle of the
/// closed rim's own strip, the ring mitred its own pair before anything noticed them, and the build
/// refused. The union is still what the surface path does - it is how two groups become one set of
/// edges - but it is no longer the only way this pick can be built, and that is the point of
/// measuring both.
#[test]
fn taking_the_shared_edge_out_of_one_rim_leaves_the_same_solid() {
    let cube = the_cube();

    let top = rim_of(&cube, Vector3::UNIT_Z);
    let awarded: HashSet<i32> = top.iter().map(|edge| edge.mesh_edge_index).collect();

    let front: Vec<FeatureEdge> = rim_of(&cube, -Vector3::UNIT_Y)
        .into_iter()
        .filter(|edge| !awarded.contains(&edge.mesh_edge_index))
        .collect();

    assert_eq!(
        front.len(),
        3,
        "the award takes the shared edge out of whichever group named it second"
    );

    // Linked per GROUP, which is what the boolean hand-over does: the top rim is still a closed
    // loop and the front rim is the open run the award made of it.
    let mut chains = bevel_chain_linker::link(&top);

    chains.extend(bevel_chain_linker::link(&front));

    let built = what_the_build_says_about(&cube, &chains, false, 1).unwrap_or_else(|complaint| {
        panic!("the same seven edges are the same pick however they were grouped: {complaint}")
    });

    assert_it_is_a_sound_solid(&built);

    let both = signed_volume(&build(both_rims, false, 1));
    let volume = signed_volume(&built);
    assert!(
        (volume - both).abs() <= 8000.0 * 1e-9,
        "the award moves an edge from one group to the other and nothing else, so the solid cannot \
         move with it - instead {volume} against {both}"
    );
}

fn the_cube() -> Mesh {
    create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)
}

/// The two rims' edges as one list, the edge they share appearing once.
fn both_rims(cube: &Mesh) -> Vec<FeatureEdge> {
    union(&[
        &rim_of(cube, Vector3::UNIT_Z),
        &rim_of(cube, -Vector3::UNIT_Y),
    ])
}

/// The rim of the face region whose outward normal is `normal`, which is what clicking that face
/// of the cube selects.
pub(crate) fn rim_of(cube: &Mesh, normal: Vector3) -> Vec<FeatureEdge> {
    let seed = (0..cube.faces.len())
        .find(|&face| {
            (Vector3::from_vector3_float(cube.faces[face].normal) - normal).length() < 1e-6
        })
        .expect("a face of the cube faces that way");

    FaceGroup::create(cube, seed as i32)
        .unwrap_or_else(|why| panic!("the face group has to read: {why:?}"))
        .boundary_edges()
        .to_vec()
}

/// Two groups' edges as one selection, deduped by mesh edge index. One orientation is kept for a
/// doubly named edge, which costs nothing: a boundary edge is classified off the MESH
/// (`FeatureEdge::try_classify`), so both groups describe it identically.
fn union(groups: &[&[FeatureEdge]]) -> Vec<FeatureEdge> {
    let mut seen = HashSet::new();
    let mut union = Vec::new();

    for edge in groups.iter().flat_map(|group| group.iter()) {
        if seen.insert(edge.mesh_edge_index) {
            union.push(*edge);
        }
    }

    union
}

/// The cube with the named selection bevelled by the surface construction.
fn build(
    picked: impl Fn(&Mesh) -> Vec<FeatureEdge>,
    round_profile: bool,
    segments_per_quarter: i32,
) -> Mesh {
    let cube = the_cube();
    let chains = bevel_chain_linker::link(&picked(&cube));

    what_the_build_says_about(&cube, &chains, round_profile, segments_per_quarter)
        .unwrap_or_else(|complaint| panic!("{complaint}"))
}

/// The chains joined, planned and built exactly as `BevelSurfacePartBuilder` does it: the built
/// mesh, or whatever the plan or the build refused with.
fn what_the_build_says_about(
    cube: &Mesh,
    chains: &[BevelSweptChain],
    round_profile: bool,
    segments_per_quarter: i32,
) -> Result<Mesh, String> {
    let graph = cube.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(cube.clone()),
        Some(graph.clone()),
        FeatureEdge::default_flat_angle_degrees(),
    ));
    let mut plans = Vec::new();

    // Joined and then OPENED, which is the pass the builder makes (BevelSurfacePartBuilder): a
    // closed rim another chain ends on is cut there, so the three edges at that vertex are three
    // chain ends whichever way they were picked.
    for chain in join_and_open(chains) {
        plans.push(
            SurfaceBevelPlan::try_create(&chain, &regions, SIZE, round_profile)
                .map_err(|reason| format!("the plan refused a chain: {reason}"))?,
        );
    }

    let built = surface_bevel_builder::try_build(
        cube,
        &graph,
        &regions,
        &plans,
        segments_per_quarter,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .map_err(|reason| format!("the build refused: {reason:?}"))?
    .mesh;

    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    if !closed {
        return Err(format!("left open at {open_at:?}"));
    }

    Ok(built)
}

/// The faces of the built mesh whose every corner stands within `reach` of a point.
fn faces_within(mesh: &Mesh, of: Vector3, reach: f64) -> Vec<[Vector3; 3]> {
    mesh.faces
        .iter()
        .map(|face| {
            [face.v0, face.v1, face.v2]
                .map(|vertex| Vector3::from_vector3_float(mesh.vertices[vertex as usize]))
        })
        .filter(|corners| corners.iter().all(|at| (*at - of).length() <= reach))
        .collect()
}

pub(crate) fn assert_it_is_a_sound_solid(built: &Mesh) {
    assert!(
        built.is_manifold(),
        "a bevel of a closed solid stays a closed solid"
    );
    let (closed, open_at) = surface_mesh_closure::is_closed(built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert_eq!(shell_count(built), 1);
}
