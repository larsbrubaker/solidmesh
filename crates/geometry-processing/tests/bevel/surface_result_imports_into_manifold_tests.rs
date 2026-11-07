//! A Surface bevel result is only finished when
//! the boolean kernel will take it. A fillet is normally followed by a Combine, and the kernel is the
//! next thing to read the mesh after the builder publishes it. Every other Surface test measures with
//! `mesh_measurements`, which reads the triangles the way the builder wrote them; this one hands the
//! same mesh to the kernel and asks what IT makes of it, which is a stricter reading and the one the
//! workflow actually depends on.
//!
//! What the kernel asks for is not what `surface_mesh_closure::open_edge_count` asks for. The kernel
//! welds vertex ids by EXACT position, DROPS every triangle that is degenerate at that welding, and
//! then requires the directed edges of what is LEFT to balance on every welded edge. So a mesh whose
//! own indices pair up perfectly is still `NotClosed` if any face it carries is degenerate by
//! position. The volume is checked against the mesh's own signed volume so a mesh that imports as
//! some OTHER solid does not read as a pass.
//!
//! The import is `manifold_import::import_strict`, with no weld-and-retry, so a seam the build
//! left open is not healed on the way in. The kernel status
//! is read by its `Debug` name, as `manifold_rust_backend_tests` reads it.

use std::collections::HashMap;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildOptions,
};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::csg::{boolean_processing, manifold_import, BooleanOperandVerdict};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Face, Mesh};
use vector_math::{Matrix4X4, Vector3};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mitre_consumes_a_chord_tests::the_eaten_chord_built;
use crate::surface_run_out_crosses_a_band_tests::the_crossing_built;
use crate::surface_test_chains::join_and_open;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

const SEGMENTS: i32 = 8;

/// A cube edge filleted: the simplest Surface result there is.
#[test]
fn a_filleted_cube_edge_imports_into_the_boolean_kernel() {
    assert_imports(&a_cube_edge_fillet());
}

/// The L prism's mixed corner, where a convex cut runs into a concave bead.
#[test]
fn the_mixed_corner_imports_into_the_boolean_kernel() {
    assert_imports(&the_mixed_corner());
}

/// The L prism's JUNCTION: both top rims picked as well as the crease, where two cuts and a bead
/// meet at one vertex and one torus fills between them.
#[test]
fn the_junction_imports_into_the_boolean_kernel() {
    assert_imports(&the_junction());
}

/// The lobed prism's own JUNCTION: the flat side's top rim, the barrel's top rim and the seam
/// between them all picked, so one arm of the corner hands over on a leaning mitre ring.
#[test]
fn the_chorded_junction_imports_into_the_boolean_kernel() {
    assert_imports(&the_chorded_junction());
}

/// The lobed prism's flat side, whose two mixed corners have a CURVED wall: the blend's footprint
/// is bitten facet by facet there, and the pieces that leaves are the thinnest the construction
/// makes.
#[test]
fn a_mixed_corner_over_a_curved_wall_imports_into_the_boolean_kernel() {
    assert_imports(&the_rim_over_the_lobe());
}

/// The lobed prism's BARREL picked as a face, whose rim turns over at both seams: the corner hands
/// over on a mitre ring there, the strip under the blend comes off the face the two edges share,
/// and the runout dies into the flat face nobody picked.
#[test]
fn the_barrels_own_rim_imports_into_the_boolean_kernel() {
    assert_imports(&the_rim_of_the_barrel(&lobed_prism_meshes::create()));
}

/// The same barrel with the prism's far side brought IN, so the flat wall the blend's runout dies
/// into runs out under it and the blend is trimmed by the face across that edge.
#[test]
fn a_barrel_whose_wall_runs_out_imports_into_the_boolean_kernel() {
    assert_imports(&the_rim_of_the_barrel(
        &lobed_prism_meshes::create_with_a_short_wall(),
    ));
}

/// A mixed corner's RUN-OUT crossing another chain's BAND, trimmed at the curve the two meet in and
/// sewn along it (`surface_run_out_crosses_a_band_tests`): the crossing's own corners are made once
/// and shared, so the kernel has to read them as one vertex each.
#[test]
fn a_run_out_crossing_a_band_imports_into_the_boolean_kernel() {
    assert_imports(&the_crossing_built());
}

/// A mitre that EATS a whole chord on the rails nearest one face
/// (`surface_mitre_consumes_a_chord_tests`): four strip ends stand on the one virtual corner there
/// and the consumed rails cover nothing between them, which is exactly the shape - a shared vertex
/// and a collapsed column - that the kernel's exact weld and degenerate-face drop are strict about.
#[test]
fn a_chord_a_mitre_eats_imports_into_the_boolean_kernel() {
    assert_imports(&the_eaten_chord_built());
}

/// One triangle of a closed solid wound the other way round: the case our three gates cannot see
/// and the kernel refuses. Every edge is still used by exactly two faces, so the undirected
/// readings - `Mesh::is_manifold` and the shell count - say the mesh is closed; the kernel pairs
/// halfedges by DIRECTION, finds the flipped triangle's three edges unpaired and reports
/// `NotClosed`. The closure gate is the directed reading, and this test says the two agree.
#[test]
fn a_triangle_wound_the_other_way_is_refused_by_both_the_gate_and_the_kernel() {
    let mut cube = create_cube(20.0, 20.0, 20.0);

    let flipped = cube.faces[0];
    cube.faces[0] = Face::new(flipped.v0, flipped.v2, flipped.v1, &cube.vertices);

    // Every edge is still used by exactly two faces, which is all the other two readings ask.
    assert!(cube.is_manifold());
    assert_eq!(shell_count(&cube), 1);

    let rejected = manifold_import::import_strict(&cube)
        .err()
        .expect("the kernel pairs by direction and refuses the flipped triangle");
    assert_eq!(
        format!("{:?}", rejected.status),
        "NotClosed",
        "the kernel pairs by direction - {}",
        diagnosis(&cube)
    );

    assert_eq!(
        surface_mesh_closure::open_edge_count(&cube).0,
        3,
        "the flipped triangle's three edges are the ones with no partner running the other way"
    );
}

/// The mesh imports with a valid status, as the solid it measures as, and the boolean path that a
/// Combine after a Fillet would take accepts it as an operand.
fn assert_imports(built: &Mesh) {
    // Our own gates first, so a failure below is known to be the kernel's reading and not ours.
    let (open, open_at) = surface_mesh_closure::open_edge_count(built);
    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(built), 1);

    let imported = manifold_import::import_strict(built).unwrap_or_else(|rejected| {
        panic!(
            "the kernel refused the published mesh - {:?}: {}",
            rejected.status,
            diagnosis(built)
        )
    });

    let measured = signed_volume(built).abs();
    assert!(
        (imported.volume() - measured).abs() <= 1e-6,
        "the kernel holds the same solid the mesh measures as - {} against {measured}",
        imported.volume()
    );

    assert_eq!(
        boolean_processing::classify_boolean_operand(built, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::Clean,
        "a Combine after a Fillet hands this very mesh over as an operand"
    );
}

/// Why the kernel might have refused, said in its own terms: how many vertices share a position
/// with another, how many faces it would drop as degenerate, and how many welded edges are left
/// unbalanced once those faces are gone.
fn diagnosis(mesh: &Mesh) -> String {
    let mut at_position: HashMap<(u32, u32, u32), i32> = HashMap::new();
    let mut welded = vec![0i32; mesh.vertices.len()];
    let mut duplicates = 0;

    for (i, vertex) in mesh.vertices.iter().enumerate() {
        // Keyed on the float triple's bits: the same as float equality, NaN aside.
        let key = (vertex.x.to_bits(), vertex.y.to_bits(), vertex.z.to_bits());

        let first = match at_position.get(&key) {
            Some(&first) => {
                duplicates += 1;
                first
            }
            None => {
                at_position.insert(key, i as i32);
                i as i32
            }
        };

        welded[i] = first;
    }

    let mut repeated_index = 0;
    let mut collapsed_by_weld = 0;
    let mut zero_area = 0;
    let mut balance: HashMap<(i32, i32), i32> = HashMap::new();

    for face in mesh.faces.iter() {
        let a = welded[face.v0 as usize];
        let b = welded[face.v1 as usize];
        let c = welded[face.v2 as usize];

        if face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0 {
            repeated_index += 1;
        }

        let at = |v: i32| Vector3::from_vector3_float(mesh.vertices[v as usize]);
        let along = at(face.v1) - at(face.v0);
        let across = at(face.v2) - at(face.v0);
        let area = along.cross(across).length() / 2.0;

        if area == 0.0 {
            zero_area += 1;
        }

        if a == b || b == c || c == a {
            collapsed_by_weld += 1;
            continue;
        }

        if area == 0.0 {
            continue;
        }

        for (u, v) in [(a, b), (b, c), (c, a)] {
            *balance.entry((u.min(v), u.max(v))).or_insert(0) += if u < v { 1 } else { -1 };
        }
    }

    let unbalanced = balance.values().filter(|&&running| running != 0).count();

    format!(
        "{} vertices ({duplicates} sharing a position with an earlier one), {} faces \
         ({repeated_index} with a repeated index, {collapsed_by_weld} collapsed by the weld, \
         {zero_area} of zero area), {unbalanced} welded edges left unbalanced",
        mesh.vertices.len(),
        mesh.faces.len()
    )
}

/// Plans one chain, failing with the plan's reason.
fn plan_of(
    chain: &BevelSweptChain,
    regions: &Arc<FaceRegionCache>,
    size: f64,
    what: &str,
) -> std::rc::Rc<SurfaceBevelPlan> {
    SurfaceBevelPlan::try_create(chain, regions, size, true)
        .unwrap_or_else(|reason| panic!("the plan refused {what}: {reason}"))
}

/// Builds every plan in one pass, failing with the build's reason.
fn build_all(
    part: &Mesh,
    regions: &FaceRegionCache,
    plans: &[std::rc::Rc<SurfaceBevelPlan>],
    what: &str,
) -> Mesh {
    surface_bevel_builder::try_build(
        part,
        &part.get_mesh_edge_graph(),
        regions,
        plans,
        SEGMENTS,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .map(|built| built.mesh)
    .unwrap_or_else(|reason| panic!("the build refused {what}: {reason:?}"))
}

/// The top front edge of a 20 cube, filleted - `surface_strip_tests`' own fixture.
fn a_cube_edge_fillet() -> Mesh {
    let cube = create_cube(20.0, 20.0, 20.0);
    let (regions, features) = regions_of(&cube);

    let edge: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.y + 10.0).abs() < 1e-5
                && (edge.end.y + 10.0).abs() < 1e-5
                && edge.start.z > 9.9
                && edge.end.z > 9.9
        })
        .cloned()
        .collect();
    assert_eq!(edge.len(), 1);

    let chain = single(bevel_chain_linker::link(&edge));
    let plan = plan_of(&chain, &regions, 3.0, "the edge");

    build_all(&cube, &regions, &[plan], "the edge")
}

/// The barrel's own wall as a face pick, rim linked and built, on the prism handed in.
fn the_rim_of_the_barrel(prism: &Mesh) -> Mesh {
    let regions = lobed_regions(prism);

    let mut on_the_barrel = -1;

    for face in 0..prism.faces.len() {
        if on_the_barrel >= 0 {
            break;
        }

        let facet = prism.faces[face];
        let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
            / 3.0;

        if middle.y < -lobed_prism_meshes::RADIUS / 2.0
            && middle.z > 1.0
            && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        {
            on_the_barrel = face as i32;
        }
    }

    let rim = regions
        .region(on_the_barrel)
        .expect("the barrel is a region")
        .boundary_edges()
        .to_vec();

    let chain = single(bevel_chain_linker::link(&rim));
    let plan = plan_of(&chain, &regions, 3.756, "the barrel's rim");

    build_all(prism, &regions, &[plan], "the barrel's rim")
}

/// The lobed prism with all three edges at the seam's cap corner picked.
fn the_chorded_junction() -> Mesh {
    let prism = lobed_prism_meshes::create();
    let regions = lobed_regions(&prism);
    let features = FeatureEdgeSet::create_with_angle(&prism, lobed_prism_meshes::THRESHOLD_DEGREES);
    let seam = lobed_prism_meshes::left_crossing();
    let height = lobed_prism_meshes::HEIGHT;

    let picked: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            let on_the_cap =
                (edge.start.z - height).abs() < 1e-4 && (edge.end.z - height).abs() < 1e-4;
            let on_the_flat_side = edge.start.y.abs() < 1e-4 && edge.end.y.abs() < 1e-4;
            let on_the_lobe = edge.start.y < -1e-4 || edge.end.y < -1e-4;

            ((edge.start.x - seam.x).abs() < 1e-3
                && (edge.end.x - seam.x).abs() < 1e-3
                && on_the_flat_side)
                || (on_the_cap && on_the_lobe)
                || (on_the_cap
                    && on_the_flat_side
                    && edge.start.x < seam.x + 0.01
                    && edge.end.x < seam.x + 0.01)
        })
        .cloned()
        .collect();

    let plans: Vec<_> = join_and_open(&bevel_chain_linker::link(&picked))
        .iter()
        .map(|chain| plan_of(chain, &regions, 3.756, "a chain"))
        .collect();

    build_all(&prism, &regions, &plans, "the chorded junction")
}

/// The rim of the flat side the lobe pokes through, filleted at the fixture's own 3.756 - the same
/// build `surface_mixed_corner_over_a_curved_wall_tests` reads, handed to the kernel.
fn the_rim_over_the_lobe() -> Mesh {
    let prism = lobed_prism_meshes::create();
    let regions = lobed_regions(&prism);
    let features = FeatureEdgeSet::create_with_angle(&prism, lobed_prism_meshes::THRESHOLD_DEGREES);
    let seam = lobed_prism_meshes::left_crossing();

    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            edge.start.y.abs() < 1e-4
                && edge.end.y.abs() < 1e-4
                && edge.start.x < seam.x + 0.01
                && edge.end.x < seam.x + 0.01
        })
        .cloned()
        .collect();

    let chain = single(bevel_chain_linker::link(&rim));
    let plan = plan_of(&chain, &regions, 3.756, "the rim");

    build_all(&prism, &regions, &[plan], "the rim")
}

/// The same L with the void wall's own top rim picked as well - the junction.
fn the_junction() -> Mesh {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = regions_of(&mesh);
    let at = Vector3::new(10.0, 10.0, 10.0);

    let plans: Vec<_> = (0..features.count())
        .filter(|&i| {
            (features.edges()[i].start - at).length() <= 1e-6
                || (features.edges()[i].end - at).length() <= 1e-6
        })
        .map(|i| plan_of(&chain_of(&features, i), &regions, RADIUS, "the edge"))
        .collect();

    build_all(&mesh, &regions, &plans, "the junction")
}

/// The L prism's mixed corner - `surface_mixed_corner_tests`' own fixture, built whole.
fn the_mixed_corner() -> Mesh {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = regions_of(&mesh);

    let concave: Vec<usize> = (0..features.count())
        .filter(|&i| features.edges()[i].convexity == EdgeConvexity::Concave)
        .collect();
    assert_eq!(concave.len(), 1);

    let chains = join_and_open(&[
        chain_of(&features, concave[0]),
        chain_of(&features, the_step_walls_top_rim(&features)),
    ]);

    assert_eq!(
        chains.len(),
        1,
        "the two picked edges are end to end, so they are one chain"
    );

    let plan = plan_of(&chains[0], &regions, RADIUS, "the chain");

    build_all(&mesh, &regions, &[plan], "the corner")
}

fn the_step_walls_top_rim(features: &FeatureEdgeSet) -> usize {
    let corner = Vector3::new(10.0, 10.0, 10.0);
    let far = Vector3::new(10.0, 20.0, 10.0);

    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];

            ((edge.start - corner).length() < 1e-6 && (edge.end - far).length() < 1e-6)
                || ((edge.end - corner).length() < 1e-6 && (edge.start - far).length() < 1e-6)
        })
        .collect();
    assert_eq!(found.len(), 1);

    found[0]
}

fn chain_of(features: &FeatureEdgeSet, edge_index: usize) -> BevelSweptChain {
    single(bevel_chain_linker::link(&[features.edges()[edge_index]]))
}

fn single(mut chains: Vec<BevelSweptChain>) -> BevelSweptChain {
    assert_eq!(chains.len(), 1, "one chain");
    chains.remove(0)
}

fn regions_of(mesh: &Mesh) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    (regions, features)
}

fn lobed_regions(prism: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ))
}
