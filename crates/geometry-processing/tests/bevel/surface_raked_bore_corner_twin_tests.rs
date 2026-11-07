//! The one row of `SectionEveryPickMatrixTests` that is not a refusal - the raked bore cube's -Z/-Y
//! cube edge (`inner_face_test_mesh`), picked ALONE. At 0.5 the legacy construction leaves it open
//! because the +X face's re-mesh and the band each land their own point at the pick's corner, two
//! vertices 1.4e-4 apart where one belongs; at 2 the two agree inside the weld and it closes. Both
//! halves: the contract the user is owed (closed, or refused by name as the app's own closure gate
//! has it), and the twin as a ratchet. The parameter rows are a loop.

use geometry_processing::bevel::surface::{surface_mesh_closure, SurfaceVertexPool};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroupResolution, BevelSelectionKind, FaceGroup,
    FeatureEdgeSet,
};
use indexmap::IndexMap;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector2, Vector3, Vector3Float};

use crate::inner_face_test_mesh::{the_part, BEND_THRESHOLD_DEGREES, SEGMENTS};
use crate::mesh_measurements::shell_count;
use crate::surface_test_build::{try_bevel_resolutions, TryBevelOptions, TryBevelOutcome};

/// The -Z/-Y cube edge's midpoint, which is how the matrix names this pick.
fn the_pick() -> Vector3 {
    Vector3::new(-9.0, -29.0, 0.0)
}

/// THE CONTRACT, at both sizes: this pick is either built closed, manifold and in one shell, or
/// refused by name - as the app's own gate has it. Never published open.
#[test]
fn the_minus_z_minus_y_edge_is_built_closed_or_refused_by_name() {
    for size in [2.0, 0.5] {
        let Some(mesh) = try_build_as_the_app_does(size) else {
            continue;
        };

        let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
        assert_eq!(
            open, 0,
            "a published bevel is closed, and this one is open at {open_at:?}"
        );
        assert!(mesh.is_manifold());
        assert_eq!(shell_count(&mesh), 1);
    }
}

/// THE MEASUREMENT, as a ratchet: at 0.5 the ungated build is open, and it is open because two
/// vertices 1.1e-4 apart stand where one point belongs.
#[test]
fn at_half_a_millimetre_two_vertices_stand_where_one_belongs() {
    let outcome = try_build_the_kernel_way(0.5);
    assert!(
        outcome.built(),
        "the construction still gets as far as a mesh at 0.5"
    );
    let mesh = outcome.result.expect("built").mesh;

    let corner = Vector3Float::new(1.0, -29.163, 0.1328);
    let twins: Vec<usize> = (0..mesh.vertices.len())
        .filter(|&vertex| (mesh.vertices[vertex] - corner).length() < 1e-3)
        .collect();

    assert_eq!(
        twins.len(),
        2,
        "the +X face's re-mesh and the band each land their own point at the pick's corner: {:?}",
        twins
            .iter()
            .map(|&vertex| mesh.vertices[vertex])
            .collect::<Vec<_>>()
    );

    // Wider than the pool's weld and narrower than its ruler: the two landings disagree, they are
    // not one landing rounded twice.
    let apart = f64::from((mesh.vertices[twins[0]] - mesh.vertices[twins[1]]).length());
    let weld = SurfaceVertexPool::weld_distance_for(&mesh.vertices);

    assert!(
        apart > weld,
        "{apart} apart against a {weld} weld - a wider weld would only hide this"
    );
    assert!(apart < weld * 10.0);

    // Neither twin is a station of the profile: the round profile of a 0.5 bevel on this convex
    // edge runs on the circle centred at (y, z) = (-29.5, 0.5), and both stand a chord's sag
    // inside it.
    for &twin in &twins {
        let at = mesh.vertices[twin].as_vector3();
        let from_centre = Vector2::new(at.y + 29.5, at.z - 0.5).length();

        assert!(
            from_centre < 0.5,
            "a point between two stations lies inside the arc, and this one is {from_centre} out"
        );
        assert!(
            from_centre > 0.49,
            "it is still ON the profile, one chord in from the radius"
        );
    }

    // And that twin is the whole of what is open: every unmatched edge has one of the two as an end.
    assert!(surface_mesh_closure::open_edge_count(&mesh).0 > 0);
    assert!(
        unmatched_edges(&mesh)
            .iter()
            .all(|&(a, b)| twins.contains(&(a as usize)) || twins.contains(&(b as usize))),
        "nothing else about this build is open"
    );

    // The same pick at 2 lands the two computations inside the weld and the mesh closes.
    let at_two = try_build_the_kernel_way(2.0);
    assert!(at_two.built());
    assert_eq!(
        surface_mesh_closure::open_edge_count(&at_two.result.expect("built").mesh).0,
        0
    );
}

/// The pick, resolved: the one straight cube edge whose midpoint is [`the_pick`].
fn resolve(part: &Mesh) -> BevelEdgeGroupResolution {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(part, &graph, BEND_THRESHOLD_DEGREES);

    let index = (0..features.count())
        .find(|&index| {
            let edge = &features.edges()[index];

            (((edge.start + edge.end) / 2.0) - the_pick()).length() < 1e-6
        })
        .expect("the part no longer has the -Z/-Y cube edge");

    bevel_edge_group_resolver::resolve(
        &bevel_edge_group_resolver::create_from_edge(
            &features,
            index as i32,
            BevelSelectionKind::Edge,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .expect("the edge is in the set"),
        part,
    )
}

/// The kernel's build, with no closure gate - what the matrix reads.
fn try_build_the_kernel_way(size: f64) -> TryBevelOutcome {
    let part = the_part();
    let resolution = resolve(&part);

    try_bevel_resolutions(
        &part,
        &[resolution],
        size,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
}

/// The same build WITH the gate the app applies after the builder - `BevelSurfacePartBuilder` turns
/// an open answer into `SurfaceResultNotClosed` rather than publishing it. `None` is a refusal,
/// and every refusal names itself.
fn try_build_as_the_app_does(size: f64) -> Option<Mesh> {
    let outcome = try_build_the_kernel_way(size);

    let built = match outcome.result {
        Ok(built) => built.mesh,
        Err(failure) => {
            assert!(
                failure.advisory().is_some(),
                "a refusal has to name itself - {failure:?}"
            );
            return None;
        }
    };

    let (closed, _) = surface_mesh_closure::is_closed(&built);

    if !closed {
        // The app's own answer: refused by name, `SurfaceResultNotClosed`.
        return None;
    }

    Some(built)
}

/// Every half-edge of the mesh that is not paired with exactly one other.
fn unmatched_edges(mesh: &Mesh) -> Vec<(i32, i32)> {
    let mut counts: IndexMap<(i32, i32), i32> = IndexMap::new();

    for face in mesh.faces.iter() {
        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *counts.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }

    counts
        .into_iter()
        .filter(|&(_, count)| count != 2)
        .map(|(key, _)| key)
        .collect()
}
