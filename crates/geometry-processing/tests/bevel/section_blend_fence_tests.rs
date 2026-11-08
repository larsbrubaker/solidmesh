//! THE BLEND FACES a sections build names, and the FENCE they
//! make for a second pick. On the straight bore's mouth the blend is the loft and nothing else; on
//! the picture hanger's raked mouth the blend comes out of the built mesh along two closed rows; and
//! a 10 degree flood from the bore wall of that build leaks over the fillet unless the blend fences
//! it. The hanger is built through the shared `surface_test_build` with the sections flag pinned
//! on.

use std::collections::{HashMap, HashSet};

use solidmesh::bevel::sections::{
    section_assembly, section_builder_run, section_routing, SectionPool,
};
use solidmesh::topology::{FaceGroup, FeatureEdgeChain, FeatureEdgeSet};
use solidmesh_math::{Vector2, Vector3};
use solidmesh_mesh::{CancelToken, Mesh, MeshEdgeGraph};

use crate::bore_test_meshes::create_plate_with_a_round_bore;
use crate::picture_hanger_test_mesh as hanger;
use crate::surface_test_build::{pinned_to_the_sections, try_bevel, TryBevelOptions};

/// The flat angle a pick's rows read a region at - the number the leak is measured at.
const FLAT_ANGLE_DEGREES: f64 = 10.0;

/// THE STRAIGHT BORE'S MOUTH: the blend is the loft and nothing else. Read by taking the blend faces
/// OUT of the published mesh - what is left is open along exactly two closed loops, which are the
/// fillet's two boundary rows and the only places the blend was ever joined to the part.
#[test]
fn the_straight_bores_blend_faces_are_the_loft_and_nothing_else() {
    // The fixture's default land is 6.
    let part = create_plate_with_a_round_bore(24, 3.0, 10.0, 6.0);
    let graph = part.get_mesh_edge_graph();

    let (built, blend_faces) = the_mouth_assembled(&part, &graph);

    assert!(!blend_faces.is_empty());
    assert_eq!(
        blend_faces.iter().collect::<HashSet<_>>().len(),
        blend_faces.len()
    );
    assert!(blend_faces
        .iter()
        .all(|&f| f >= 0 && (f as usize) < built.faces.len()));

    // A blend face is made of section samples, so none of its corners stands where a vertex of the
    // part it was cut into stands: a rebuilt or re-meshed original face would.
    for &face in &blend_faces {
        assert!(
            !any_corner_stands_on_the_part(&built, face, &part),
            "face {face} was called blend and one of its corners is a vertex of the part"
        );
    }

    let (loops, why) = open_loops_without(&built, &blend_faces.iter().copied().collect());

    assert_eq!(
        loops, 2,
        "one run leaves its two boundary rows and nothing else open - {why}"
    );
}

/// THE HANGER'S RAKED MOUTH, the same reading through the whole pipeline: the pass routes to the
/// sections, and the blend it names comes out of the built mesh along two closed rows.
#[test]
fn the_raked_mouths_blend_faces_come_out_along_two_rows() {
    let (built, blend_faces) = the_hangers_mouth_built();

    assert!(!blend_faces.is_empty());

    let (loops, why) = open_loops_without(&built, &blend_faces.iter().copied().collect());

    assert_eq!(
        loops, 2,
        "the mouth is one run, so its blend leaves two rows open - {why}"
    );
}

/// THE LEAK AND THE FENCE. On the hanger's own output, a 10 degree flood from a face of the bore
/// wall runs over the blend; fenced with the blend faces it is the wall alone, and its rim closes.
#[test]
fn a_wall_region_on_the_built_mesh_leaks_over_the_blend_unless_it_is_fenced() {
    let (built, blend_faces) = the_hangers_mouth_built();
    let blend: HashSet<i32> = blend_faces.iter().copied().collect();
    let graph = built.get_mesh_edge_graph();
    let seed = a_wall_face_beside_the_blend(&built, &graph, &blend);

    let leaked = FaceGroup::create_with_graph(&built, &graph, seed, FLAT_ANGLE_DEGREES)
        .expect("the seed is a face of the build");
    let leaked_blend = leaked
        .faces()
        .iter()
        .filter(|face| blend.contains(face))
        .count();

    // The leak, pinned: 798 of the built mesh's 1086 faces, 600 of them blend - the flood takes in
    // all but the 56 blend faces on the far side of the fillet's other boundary row and carries on
    // into the plate. This is the number the fence has to change.
    assert_eq!(leaked.faces().len(), 798);
    assert_eq!(
        leaked_blend, 600,
        "an unfenced flood crosses the fillet's own facets, which turn less than 10 degrees"
    );

    let fenced =
        FaceGroup::create_fenced(&built, &graph, seed, FLAT_ANGLE_DEGREES, Some(&blend_faces))
            .expect("the seed is a face of the build");

    assert_eq!(
        fenced
            .faces()
            .iter()
            .filter(|face| blend.contains(face))
            .count(),
        0,
        "the fence is not crossed"
    );

    // 102: the very count the SOURCE mesh's wall region reads, which is the whole claim - the
    // fenced flood on the built mesh reads the same wall the pick read before anything was built.
    assert_eq!(
        fenced.faces().len(),
        102,
        "the fence leaves the wall alone - {} where the leak read {}",
        fenced.faces().len(),
        leaked.faces().len()
    );

    assert_eq!(
        fenced.open_boundary_mesh_edges().len(),
        0,
        "a fenced edge has a face on both sides of it, so it is a rim edge and not an open one"
    );

    let (rim_loops, why_not) = closed_loops(
        &fenced
            .boundary_edges()
            .iter()
            .map(|e| {
                (
                    graph.get_vertex0(e.mesh_edge_index),
                    graph.get_vertex1(e.mesh_edge_index),
                )
            })
            .collect::<Vec<_>>(),
    );

    // Three, every one of them closed: the fillet's wall-side boundary row at the mouth, and the two
    // rims the raked bore's far end leaves in the plate's under side. `closed_loops` answers zero
    // for a rim that does not close, which is the reading the failing row was refused on.
    assert_eq!(rim_loops, 3, "the fenced region's rim closes - {why_not}");
}

/// The bore mouth of the plate fixture, sectioned and assembled, blend faces
/// and all.
fn the_mouth_assembled(part: &Mesh, graph: &MeshEdgeGraph) -> (Mesh, Vec<i32>) {
    let features = FeatureEdgeSet::create_with_graph(part, graph, 20.0);
    let chain = FeatureEdgeChain::propagate(&features, the_upper_mouth(&features))
        .expect("the seed is an edge of the set");
    let mut pool = SectionPool::new(&part.vertices);

    let run =
        section_builder_run::try_build_chain(Some(&chain), part, graph, &mut pool, 0.4, 8, true)
            .unwrap_or_else(|failure| panic!("the bore's mouth sections - instead {failure:?}"));

    section_assembly::try_assemble_with_blend(part, graph, std::slice::from_ref(&run), &mut pool)
        .unwrap_or_else(|failure| panic!("the bore's mouth assembles - instead {failure:?}"))
}

/// The index of the bore's upper mouth among the feature edges.
fn the_upper_mouth(set: &FeatureEdgeSet) -> i32 {
    for (i, edge) in set.edges().iter().enumerate() {
        if (edge.start.z - 5.0).abs() < 1e-9
            && (edge.end.z - 5.0).abs() < 1e-9
            && (Vector2::new(edge.start.x, edge.start.y).length() - 3.0).abs() < 1e-6
        {
            return i as i32;
        }
    }

    panic!("the fixture has no upper mouth");
}

/// The hanger's raked mouth alone, built through the sections route,
/// with the blend faces the build names.
fn the_hangers_mouth_built() -> (Mesh, Vec<i32>) {
    let _sections = pinned_to_the_sections();

    let part = hanger::the_part();

    let outcome = try_bevel(
        &part,
        &[hanger::the_raked_mouth()],
        hanger::SIZE,
        hanger::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let route = section_routing::last_route();

    match outcome.result {
        Ok(built) if route.as_deref() == Some("sections") => (built.mesh, built.blend_faces),
        other => panic!(
            "the raked mouth builds through the sections - instead {:?} by {route:?}",
            other.err()
        ),
    }
}

/// A face of the BORE WALL standing against the blend - the first
/// non-blend face sharing an edge with a blend face whose own plane is not the plate's flat top.
/// The raked wall's facets lean at most 45 degrees off vertical, so their normals never reach the
/// plate's.
fn a_wall_face_beside_the_blend(mesh: &Mesh, graph: &MeshEdgeGraph, blend: &HashSet<i32>) -> i32 {
    for edge_index in 0..graph.edge_count() {
        if graph.get_face_count(edge_index) != 2 {
            continue;
        }

        let faces = graph.get_faces(edge_index);

        for side in 0..2 {
            let face = faces[side];

            if blend.contains(&face) || !blend.contains(&faces[1 - side]) {
                continue;
            }

            if (mesh.faces[face as usize].normal.z as f64).abs() < 0.9 {
                return face;
            }
        }
    }

    panic!("no wall face stands against the blend");
}

/// Whether any corner of a face of the built mesh stands where a
/// vertex of the part does.
fn any_corner_stands_on_the_part(built: &Mesh, face_index: i32, part: &Mesh) -> bool {
    let face = &built.faces[face_index as usize];

    [face.v0, face.v1, face.v2].iter().any(|&corner| {
        let at = Vector3::from_vector3_float(built.vertices[corner as usize]);

        part.vertices
            .iter()
            .any(|vertex| (Vector3::from_vector3_float(*vertex) - at).length() < 1e-6)
    })
}

/// How many closed loops the mesh is left open along when `removed` is taken
/// out of it - zero where what is left open does not close, with why.
fn open_loops_without(mesh: &Mesh, removed: &HashSet<i32>) -> (i32, String) {
    // The open edges are kept in first-met order, so the listing reads the same on every run.
    let mut order: Vec<(i32, i32)> = Vec::new();
    let mut used: HashMap<(i32, i32), i32> = HashMap::new();

    for (i, face) in mesh.faces.iter().enumerate() {
        if removed.contains(&(i as i32)) {
            continue;
        }

        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            let count = used.entry(key).or_insert(0);

            if *count == 0 {
                order.push(key);
            }

            *count += 1;
        }
    }

    let open: Vec<(i32, i32)> = order.into_iter().filter(|key| used[key] == 1).collect();

    closed_loops(&open)
}

/// How many closed loops a set of undirected edges makes, or zero where any vertex
/// of it is not stood on by exactly two of them - which is a rim that does not close.
fn closed_loops(edges: &[(i32, i32)]) -> (i32, String) {
    if edges.is_empty() {
        return (0, "there are no edges at all".to_string());
    }

    // Insertion-ordered, so the walk visits vertices the same way on every run.
    let mut order: Vec<i32> = Vec::new();
    let mut neighbours: HashMap<i32, Vec<i32>> = HashMap::new();

    let mut add = |from: i32, to: i32| {
        neighbours
            .entry(from)
            .or_insert_with(|| {
                order.push(from);
                Vec::new()
            })
            .push(to);
    };

    for &(a, b) in edges {
        add(a, b);
        add(b, a);
    }

    for vertex in &order {
        let count = neighbours[vertex].len();

        if count != 2 {
            return (0, format!("vertex {vertex} stands on {count} of them"));
        }
    }

    let mut seen = HashSet::new();
    let mut loops = 0;

    for &vertex in &order {
        if !seen.insert(vertex) {
            continue;
        }

        loops += 1;

        let mut walk = vec![vertex];

        while let Some(at) = walk.pop() {
            for &next in &neighbours[&at] {
                if seen.insert(next) {
                    walk.push(next);
                }
            }
        }
    }

    (
        loops,
        format!("{loops} closed loops over {} edges", edges.len()),
    )
}
