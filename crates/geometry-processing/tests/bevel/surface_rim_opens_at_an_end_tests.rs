//! Port of `SurfaceRimOpensAtAnEndTests.cs`: a CLOSED rim taken whole, with one more edge standing
//! at one of its corners. Three convex edges meet at that vertex, which is the ball's octant - what
//! the build makes of them whenever the three arrive as three chains (`surface_corner_tests`). Two
//! of them arriving as one plan's consecutive pair may not change the answer: the rim is CUT OPEN
//! at that vertex before anything is planned (`surface_chain_opens_at_the_junction`), so the three
//! arrive as three chain ends there and close the octant together.
//!
//! Before the opening, the ring's own corner was mitred before anything noticed the third plan's
//! end, and the mitre was refused (`SurfaceStripSeamDidNotMeet`) - Lars's own five-edge case on
//! "Every Chamfer Works.mcx" measured down to a cube. The control is the SAME FIVE EDGES linked as
//! the app links a hand-picked set: one pick is a ring and one is a run, and the solid is the same.
//! C# `[Arguments]` rows are a loop.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildOptions,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_chains::join_and_open;

const CUBE_SIZE: f64 = 20.0;

const SIZE: f64 = 3.0;

const SEGMENTS: i32 = 8;

/// One pick: the feature edges a user's selection names.
type Pick = fn(&FeatureEdgeSet) -> Vec<FeatureEdge>;

/// The corner of the top rim the fifth edge also stands at.
fn the_shared_corner() -> Vector3 {
    Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, CUBE_SIZE / 2.0)
}

/// The whole thing, chamfered, measured against a volume worked out by hand: the rim as one closed
/// plan and the vertical edge as another take exactly what five prisms take, less what the ring's
/// three ordinary mitres stop counting twice and less what the three-edge corner keeps.
///
/// Each of the five edges sweeps its right triangle `d^2 / 2` the whole 20 mm. At each of the rim's
/// three PLAIN corners two prisms overlap in `d^3 / 3`, which the mitre counts once. In the cell at
/// the shared corner `3 d^3 / 4` was counted twice. At d = 3 that is `450 - 27 - 20.25` removed,
/// and the solid is 7597.25.
#[test]
fn the_rim_with_an_edge_at_its_corner_chamfers_to_the_volume_its_five_prisms_leave() {
    let built = build(&[the_top_rim, the_edge_at_the_corner], false, 1);

    assert_it_is_a_sound_solid(&built);

    let swept = 5.0 * CUBE_SIZE * (SIZE * SIZE / 2.0);
    let mitred = 3.0 * (SIZE * SIZE * SIZE / 3.0);
    let the_cell = 3.0 * (SIZE * SIZE * SIZE / 4.0);

    let expected = 8000.0 - swept + mitred + the_cell;
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= 8000.0 * 1e-6,
        "five prisms, three mitres and one three-edge corner, which is 7597.25 and nothing else - \
         {measured}"
    );

    assert!(
        (expected - 7597.25).abs() <= 1e-9,
        "the closed form is worth writing down"
    );
}

/// The same pick filleted: what stands in the corner cell is the ball's OCTANT - the patch three
/// chains at a vertex get - sampled on its own three arcs and every point of it a radius from the
/// one centre. A mitre would have put two flat triangles there, or refused.
#[test]
fn the_opened_corner_carries_the_balls_octant() {
    let built = build(&[the_top_rim, the_edge_at_the_corner], true, SEGMENTS);

    assert_it_is_a_sound_solid(&built);

    let centre = the_shared_corner() + Vector3::new(-SIZE, SIZE, -SIZE);
    let patch = faces_in_the_corner_cell(&built);

    assert_eq!(
        patch.len() as i32,
        (4 * SEGMENTS * SEGMENTS) - (3 * SEGMENTS),
        "the grid on its own arcs' sample count is n squared triangles, and one level of INTERIOR \
         refinement cuts each of them in four but for the edges of the RIM, which may not be split: \
         4T less R, which for three rings of n chords is 4n squared less 3n"
    );

    for face in &patch {
        for &at in face {
            assert!(
                ((at - centre).length() - SIZE).abs() <= 1e-6,
                "every point of the patch stands a radius from the ball centre, and {at:?} does not"
            );
        }
    }
}

/// And nothing is minted twice. The octant's rims are the seam vertices the walkback already made,
/// so no two vertices of the answer stand at one place. Said of both profiles, and said whichever
/// edge of the ring the plan happens to start at: the same five edges picked so that the linker
/// walks the ring from another edge build the same solid, vertex for vertex.
#[test]
fn nothing_is_minted_twice_and_the_rings_starting_edge_does_not_matter() {
    for (round_profile, segments) in [(false, 1), (true, SEGMENTS)] {
        let built = build(
            &[the_top_rim, the_edge_at_the_corner],
            round_profile,
            segments,
        );
        let from_another_edge = build(&[the_five_edges_together], round_profile, segments);

        let positions: Vec<Vector3> = built
            .vertices
            .iter()
            .map(|&at| Vector3::from_vector3_float(at))
            .collect();

        let doubled: Vec<Vector3> = positions
            .iter()
            .enumerate()
            .filter(|&(index, &one)| {
                positions.iter().enumerate().any(|(other_index, &other)| {
                    other_index != index && (other - one).length() < 1e-6
                })
            })
            .map(|(_, &at)| at)
            .collect();

        assert!(
            doubled.is_empty(),
            "round {round_profile}: a rim the octant made its own vertex of would stand at a seam \
             vertex the walkback already made, and {:?} does",
            doubled.first()
        );

        assert_eq!(
            built.vertices.len(),
            from_another_edge.vertices.len(),
            "round {round_profile}: the ring is the same ring whichever of its edges the linker \
             starts walking at"
        );
        assert_eq!(
            built.faces.len(),
            from_another_edge.faces.len(),
            "round {round_profile}"
        );
        assert!(
            (signed_volume(&built) - signed_volume(&from_another_edge)).abs() <= 8000.0 * 1e-9,
            "round {round_profile}"
        );
    }
}

/// The sanity: a fifth edge that stands at NO corner of the rim leaves every corner of the rim a
/// mitre. The ring is still one plan and the yield is not a licence to stop mitring.
#[test]
fn a_rim_with_an_edge_it_does_not_touch_still_mitres_every_corner() {
    let built = build(&[the_top_rim, the_edge_across_the_cube], false, 1);

    assert_it_is_a_sound_solid(&built);

    // Four rim chords and one loose one, each taking its own swept section, less the four corners
    // the rim's mitres stop counting twice. The loose edge is nowhere near any of them.
    let swept = 5.0 * CUBE_SIZE * (SIZE * SIZE / 2.0);
    let corners = 4.0 * (SIZE * SIZE * SIZE / 3.0);

    let expected = 8000.0 - swept + corners;
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= 8000.0 * 1e-6,
        "five prisms less the four mitred corners of the ring, which is the ring mitring as it \
         always did - {measured}, expected {expected}"
    );
}

/// And the refusal by name: two rings whose corners stand at ONE vertex. The ends there are not
/// three but four, which is no octant, so it is named rather than built or crashed into.
#[test]
fn two_rings_meeting_at_one_vertex_are_refused_by_name() {
    let pyramid = the_double_pyramid();
    let features = FeatureEdgeSet::create(&pyramid);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(pyramid.clone()),
        Some(pyramid.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    let mut plans: Vec<Rc<SurfaceBevelPlan>> = Vec::new();

    for ring in [the_north_face_rim(&features), the_south_face_rim(&features)] {
        for chain in bevel_chain_linker::link(&ring) {
            plans.push(
                SurfaceBevelPlan::try_create(&chain, &regions, SIZE / 4.0, false).unwrap_or_else(
                    |reason| panic!("the plan refused the ring: {:?}", reason.code()),
                ),
            );
        }
    }

    let reason = surface_bevel_builder::try_build(
        &pyramid,
        &pyramid.get_mesh_edge_graph(),
        &regions,
        &plans,
        1,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .map(|_| ())
    .expect_err(
        "two corners at one vertex are four ends there, and nothing in this pass closes four",
    );

    assert_eq!(
        reason.advisory().map(|advisory| advisory.code()),
        Some(BevelAdvisoryCode::SurfaceTwoCornersAtOneVertex),
        "it is named for what it is rather than found out as a mitre that would not meet - \
         {reason:?}"
    );
}

fn assert_it_is_a_sound_solid(built: &Mesh) {
    assert!(
        built.is_manifold(),
        "a bevel of a closed solid stays a closed solid"
    );
    let (open, open_at) = surface_mesh_closure::open_edge_count(built);
    assert_eq!(
        open, 0,
        "nothing may be left open, and something is at {open_at:?}"
    );
    let (folds, folded) = surface_mesh_closure::folded_edge_count(&faces_of(built));
    assert_eq!(
        folds, 0,
        "nothing may be folded, and the edge {folded:?} is"
    );
    assert_eq!(shell_count(built), 1);
}

/// The faces of the built mesh lying wholly inside the CORNER CELL - the cube of the bevel's own
/// size at [`the_shared_corner`] - which is where the patch is and where nothing else can be: all
/// three strips stop on its walls.
fn faces_in_the_corner_cell(mesh: &Mesh) -> Vec<[Vector3; 3]> {
    let mut found = Vec::new();
    let least = the_shared_corner() + Vector3::new(-SIZE, 0.0, -SIZE);
    let most = the_shared_corner() + Vector3::new(0.0, SIZE, 0.0);

    for face in mesh.faces.iter() {
        let corners = [
            Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]),
            Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]),
        ];

        let mut inside = true;

        for at in corners {
            inside &= at.x >= least.x - 1e-5
                && at.x <= most.x + 1e-5
                && at.y >= least.y - 1e-5
                && at.y <= most.y + 1e-5
                && at.z >= least.z - 1e-5
                && at.z <= most.z + 1e-5;
        }

        if inside {
            found.push(corners);
        }
    }

    found
}

/// The built mesh's faces as the closure reading takes them.
fn faces_of(mesh: &Mesh) -> Vec<(i32, i32, i32)> {
    mesh.faces
        .iter()
        .map(|face| (face.v0, face.v1, face.v2))
        .collect()
}

/// The four edges of the cube's top face, which link into one closed ring.
fn the_top_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.z - (CUBE_SIZE / 2.0)).abs() < 1e-6
                && (edge.end.z - (CUBE_SIZE / 2.0)).abs() < 1e-6
        })
        .copied()
        .collect()
}

/// The vertical edge standing at [`the_shared_corner`].
fn the_edge_at_the_corner(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    vec![edge_between(
        features,
        the_shared_corner(),
        Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
    )]
}

/// An edge of the BOTTOM face, which stands at no corner of the top rim at all.
fn the_edge_across_the_cube(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    vec![edge_between(
        features,
        Vector3::new(CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
        Vector3::new(CUBE_SIZE / 2.0, CUBE_SIZE / 2.0, -CUBE_SIZE / 2.0),
    )]
}

/// The same five edges as ONE pick, which is how they arrive when they are picked loose: the
/// linker will not walk the vertex three of them meet at, so the rim comes back as an open run of
/// four.
fn the_five_edges_together(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    let mut found = the_top_rim(features);

    found.extend(the_edge_at_the_corner(features));

    found
}

/// The one feature edge running between two given points, either way about.
fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.start - to).length() < 1e-6 && (edge.end - from).length() < 1e-6)
        })
        .copied()
        .collect();

    assert_eq!(found.len(), 1, "one edge from {from:?} to {to:?}");
    found[0]
}

/// The cube with each PICK linked on its own - a face rim is one group and a crease is another,
/// and the linker never walks from one group into the next - and all of their plans built in one
/// pass.
fn build(picks: &[Pick], round_profile: bool, segments_per_quarter: i32) -> Mesh {
    let cube = platonic_solids::create_cube(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE);
    let features = FeatureEdgeSet::create(&cube);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(cube.clone()),
        Some(cube.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    let mut chains: Vec<BevelSweptChain> = Vec::new();

    for pick in picks {
        chains.extend(bevel_chain_linker::link(&pick(&features)));
    }

    // Joined and then OPENED, which is the pass the builder makes (BevelSurfacePartBuilder): a
    // closed rim another chain ends on is cut there, so the three edges at that vertex are three
    // chain ends whichever way they were picked.
    let plans: Vec<Rc<SurfaceBevelPlan>> = join_and_open(&chains)
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, &regions, SIZE, round_profile)
                .unwrap_or_else(|reason| panic!("the plan refused the chain: {:?}", reason.code()))
        })
        .collect();

    surface_bevel_builder::try_build(
        &cube,
        &cube.get_mesh_edge_graph(),
        &regions,
        &plans,
        segments_per_quarter,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the corner: {reason:?}"))
    .mesh
}

/// Two square pyramids base to base, which is the cheapest solid with a vertex FOUR creases meet
/// at: the apex. Two of the four faces around it touch each other only there, so their rims are
/// two closed rings sharing one vertex and nothing else.
fn the_double_pyramid() -> Mesh {
    let mut mesh = Mesh::new();

    let top = Vector3::new(0.0, 0.0, 10.0);
    let bottom = Vector3::new(0.0, 0.0, -10.0);

    let waist = [
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(-10.0, 10.0, 0.0),
        Vector3::new(-10.0, -10.0, 0.0),
        Vector3::new(10.0, -10.0, 0.0),
    ];

    for i in 0..4 {
        let here = waist[i];
        let next = waist[(i + 1) % 4];

        mesh.create_face(&[top, here, next]);
        mesh.create_face(&[bottom, next, here]);
    }

    mesh.clean_and_merge();

    mesh
}

/// One face of the upper pyramid, as its own three creases.
fn the_north_face_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    rim_of(
        features,
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(-10.0, 10.0, 0.0),
    )
}

/// The face across the apex from it, which shares the apex and no edge.
fn the_south_face_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    rim_of(
        features,
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(-10.0, -10.0, 0.0),
        Vector3::new(10.0, -10.0, 0.0),
    )
}

/// The three creases bounding one triangular face.
fn rim_of(features: &FeatureEdgeSet, a: Vector3, b: Vector3, c: Vector3) -> Vec<FeatureEdge> {
    vec![
        edge_between(features, a, b),
        edge_between(features, b, c),
        edge_between(features, c, a),
    ]
}
