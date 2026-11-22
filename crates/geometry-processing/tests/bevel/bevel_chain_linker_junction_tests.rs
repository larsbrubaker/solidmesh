//! A run carried on through a junction of picked creases (`bevel_chain_linker::link_on_the_mesh`):
//! a tube joined to a box across the box's convex edge meets it in one closed concave loop, with the
//! box edge ending on that loop from either side; and at a channel's two inner corners the end
//! face's rim carries on round each corner while the floor's concave creases end on it, a group's
//! row counts the loops the build links, and two rims touching at a corner stay two loops.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker::{self, BevelSweptChain};
use solidmesh::bevel::bevel_working_mesh::{self, StampedVerdict};
use solidmesh::bevel::part::surface_part_builder::BevelSurfacePartBuilder;
use solidmesh::bevel::BevelRebuildAdvisories;
use solidmesh::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge, FeatureEdgeSet,
};
use solidmesh_math::{Color, Matrix4X4, Vector3};
use solidmesh_mesh::csg::csg_operations;
use solidmesh_mesh::solids::platonic_solids::create_cube_vector;
use solidmesh_mesh::{CancelToken, Mesh};

/// The bend threshold the picks are read at, as a saved fillet's is.
const BEND_DEGREES: f64 = 13.0;

/// A closed tube of `sides` facets, radius `radius`, along unit `axis` from `centre + from * axis`
/// to `centre + to * axis`, wound outward. `across` and `up` finish a right-handed frame with
/// `axis` (`across x up = axis`).
fn tube(
    centre: Vector3,
    frame: [Vector3; 3],
    radius: f64,
    (from, to): (f64, f64),
    sides: i32,
) -> Mesh {
    let [across, up, axis] = frame;
    let mut vertices = Vec::new();
    for t in [from, to] {
        for i in 0..sides {
            let angle = std::f64::consts::TAU * i as f64 / sides as f64;
            let at = centre + axis * t + (across * angle.cos() + up * angle.sin()) * radius;
            vertices.extend([at.x, at.y, at.z]);
        }
    }
    for t in [from, to] {
        let at = centre + axis * t;
        vertices.extend([at.x, at.y, at.z]);
    }
    let (bottom_centre, top_centre) = (2 * sides, 2 * sides + 1);
    let mut faces = Vec::new();
    for i in 0..sides {
        let next = (i + 1) % sides;
        let (b0, b1, t0, t1) = (i, next, sides + i, sides + next);
        faces.extend([b0, b1, t1, b0, t1, t0]);
        faces.extend([top_centre, t0, t1]);
        faces.extend([bottom_centre, b1, b0]);
    }
    Mesh::from_arrays(&vertices, &faces)
}

/// A box from `low` to `high`.
fn block(low: Vector3, high: Vector3) -> Mesh {
    let mut mesh = create_cube_vector(high - low);
    let middle = (low + high) * 0.5;
    mesh.translate(middle);
    mesh
}

/// `a` and `b` combined, made the working copy a bevel reads.
fn joined(a: &Mesh, b: &Mesh) -> Mesh {
    let union = csg_operations::union(a, b, &CancelToken::new()).expect("the union runs");
    bevel_working_mesh::create_from(&union, &mut StampedVerdict::default())
}

/// The picked edges linked as a pick is, on the mesh.
fn linked(mesh: &Mesh, pick: impl Fn(&FeatureEdge) -> bool) -> Vec<BevelSweptChain> {
    let features = FeatureEdgeSet::create_with_angle(mesh, BEND_DEGREES);
    let picked: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(|edge| pick(edge))
        .collect();
    let graph = mesh.get_mesh_edge_graph();
    bevel_chain_linker::link_on_the_mesh(&picked, mesh, &graph, &features)
}

fn ends(chain: &BevelSweptChain) -> [i32; 2] {
    [
        chain.edges[0].from_vertex(),
        chain.edges[chain.edges.len() - 1].to_vertex(),
    ]
}

fn on_the_line(edge: &FeatureEdge, y: f64, z: f64) -> bool {
    [edge.start, edge.end]
        .iter()
        .all(|p| (p.y - y).abs() < 1e-5 && (p.z - z).abs() < 1e-5)
}

/// A U-shaped channel along x: a floor with a wall either side.
fn the_channel() -> Mesh {
    let floor = block(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 0.0));
    let near_wall = block(Vector3::new(-1.0, -1.0, -0.5), Vector3::new(1.0, -0.4, 1.0));
    let far_wall = block(Vector3::new(-1.0, 0.4, -0.5), Vector3::new(1.0, 1.0, 1.0));
    joined(&joined(&floor, &near_wall), &far_wall)
}

/// The channel's end face rim at x = -1 and its two floor creases.
fn the_channel_pick(edge: &FeatureEdge) -> bool {
    let on_the_end = [edge.start, edge.end]
        .iter()
        .all(|p| (p.x + 1.0).abs() < 1e-5);
    (on_the_end && edge.convexity == EdgeConvexity::Convex)
        || (edge.convexity == EdgeConvexity::Concave
            && (on_the_line(edge, -0.4, 0.0) || on_the_line(edge, 0.4, 0.0)))
}

/// A tube through the top front edge of a box, picked with that edge: the tube's join is one
/// closed concave loop, and the box edge is two open convex runs, each from a box corner to the
/// loop - the edge truly ends where the tube cuts it.
#[test]
fn a_tube_joined_across_a_box_edge_meets_it_in_one_closed_loop() {
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let frame = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, half, half),
        Vector3::new(0.0, -half, half),
    ];
    let pipe = tube(Vector3::new(0.0, -1.0, 1.0), frame, 0.3, (-0.8, 1.0), 32);
    let mesh = joined(&create_cube_vector(Vector3::new(2.0, 2.0, 2.0)), &pipe);
    let chains = linked(&mesh, |edge| {
        edge.convexity == EdgeConvexity::Concave || on_the_line(edge, -1.0, 1.0)
    });

    let concave: Vec<&BevelSweptChain> = chains
        .iter()
        .filter(|chain| chain.edges[0].edge.convexity == EdgeConvexity::Concave)
        .collect();
    let convex: Vec<&BevelSweptChain> = chains
        .iter()
        .filter(|chain| chain.edges[0].edge.convexity == EdgeConvexity::Convex)
        .collect();
    assert_eq!(concave.len(), 1, "the join is one loop: {chains:?}");
    assert!(concave[0].is_closed, "the tube's join closes");
    assert_eq!(convex.len(), 2, "the box edge either side of the tube");

    let on_the_loop: Vec<i32> = concave[0]
        .edges
        .iter()
        .map(|edge| edge.from_vertex())
        .collect();
    for run in convex {
        assert!(!run.is_closed);
        let [a, b] = ends(run);
        assert!(
            on_the_loop.contains(&a) != on_the_loop.contains(&b),
            "a box edge run goes from a box corner to the loop: {a} {b}"
        );
    }
}

/// A U-shaped channel's end face picked round its rim with the channel's two inner creases: at each
/// inner corner two convex edges bound the end face and a concave crease runs between a wall and
/// the floor. The end face's rim goes on round both corners as one closed loop - its outline is
/// what a first-time user sees as one edge - and each inner crease ends on it. Two corners, so a
/// walk that stops at every junction leaves the rim in two open pieces; one corner alone would close
/// either way, the walk coming back round to the corner it stopped at.
#[test]
fn a_channel_end_rim_runs_round_both_inner_corners_where_the_floor_creases_end() {
    let mesh = the_channel();
    let chains = linked(&mesh, the_channel_pick);

    assert_eq!(chains.len(), 3, "{chains:?}");
    let rims: Vec<&BevelSweptChain> = chains
        .iter()
        .filter(|chain| chain.edges[0].edge.convexity == EdgeConvexity::Convex)
        .collect();
    assert_eq!(rims.len(), 1, "the end face's rim is one run");
    assert!(
        rims[0].is_closed,
        "the end face's rim closes round both inner corners"
    );
    let on_the_rim: Vec<i32> = rims[0]
        .edges
        .iter()
        .map(|edge| edge.from_vertex())
        .collect();
    for crease in chains
        .iter()
        .filter(|chain| chain.edges[0].edge.convexity == EdgeConvexity::Concave)
    {
        assert!(!crease.is_closed);
        let [a, b] = ends(crease);
        assert!(
            on_the_rim.contains(&a) != on_the_rim.contains(&b),
            "a floor crease ends on the rim at one end: {a} {b}"
        );
    }
}

/// The channel's end rim and floor creases picked as ONE group: the group's row counts the loops
/// the build links - the closed rim and the two creases, three - and not the four pieces a walk
/// stopping at every junction leaves.
#[test]
fn a_groups_row_counts_the_loops_the_build_links() {
    let mesh = the_channel();
    let features = Arc::new(FeatureEdgeSet::create_with_angle(&mesh, BEND_DEGREES));
    let edges: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .copied()
        .filter(the_channel_pick)
        .collect();
    let graph = mesh.get_mesh_edge_graph();
    let built = bevel_chain_linker::link_on_the_mesh(&edges, &mesh, &graph, &features).len();
    let resolution = BevelEdgeGroupResolution {
        group: BevelEdgeGroup::default(),
        features,
        failure_reason: None,
        seed_edge_index: 0,
        edges,
        chain: None,
        face_group: None,
    };
    let mut builder = BevelSurfacePartBuilder::new(
        vec![BevelEdgeGroup::default()],
        vec![resolution],
        vec![true],
        Rc::new(BevelRebuildAdvisories::new(None::<Vec<_>>)),
        Arc::new(mesh),
        graph,
        0.1,
        8,
        true,
        Matrix4X4::IDENTITY,
        Color::default(),
        None,
        None,
        None,
    );

    assert_eq!(built, 3);
    assert_eq!(builder.the_rim_loops_of_the_group(0).len(), built);
}

/// A sheet with two square pockets whose rims touch at one corner, each pocket an upside-down
/// pyramid so every edge is manifold: at the shared corner four picked rim edges meet, and the
/// sheet's top is pinched there into two sectors.
fn two_pockets_touching_at_a_corner() -> Mesh {
    let index = |x: i32, y: i32| (x + 1) + (y + 1) * 5;
    let mut vertices = Vec::new();
    for y in -1..=3 {
        for x in -1..=3 {
            vertices.extend([x as f64, y as f64, 0.0]);
        }
    }
    let mut faces = Vec::new();
    for y in -1..3 {
        for x in -1..3 {
            let corners = [
                index(x, y),
                index(x + 1, y),
                index(x + 1, y + 1),
                index(x, y + 1),
            ];
            if (x, y) == (0, 0) || (x, y) == (1, 1) {
                // The pocket: four walls down to an apex under the cell's middle.
                let apex = (vertices.len() / 3) as i32;
                vertices.extend([x as f64 + 0.5, y as f64 + 0.5, -1.0]);
                for i in 0..4 {
                    faces.extend([corners[i], corners[(i + 1) % 4], apex]);
                }
            } else {
                faces.extend([corners[0], corners[1], corners[2]]);
                faces.extend([corners[0], corners[2], corners[3]]);
            }
        }
    }
    Mesh::from_arrays(&vertices, &faces)
}

/// The sheet's rim picked round both pockets: the two rims touch at one corner but are two closed
/// loops. Carried across the pinch they would close as one run through that corner twice, which
/// the build refuses as a run touching itself.
#[test]
fn two_rims_touching_at_a_corner_stay_two_loops() {
    let mesh = two_pockets_touching_at_a_corner();
    let chains = linked(&mesh, |edge| edge.convexity == EdgeConvexity::Convex);
    let runs: Vec<(bool, usize)> = chains
        .iter()
        .map(|chain| (chain.is_closed, chain.edges.len()))
        .collect();
    assert_eq!(runs, vec![(true, 4), (true, 4)], "{chains:?}");
}
