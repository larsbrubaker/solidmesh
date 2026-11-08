//! ROW G1 OF THE PICTURE HANGER, on the hanger's own mesh
//! ([`crate::picture_hanger_test_mesh`]) - the top face's ten edge rim at z = 20, the face that rim
//! goes round, and the eleventh corner standing a tenth of a millimetre inside it - taking its 0.4
//! fillet through the sections, closed, manifold and in one shell, built through the shared
//! `surface_test_build` with the sections flag pinned on.

use std::collections::{BTreeSet, HashMap, HashSet};

use solidmesh_math::{Vector2, Vector3};
use solidmesh_mesh::Mesh;

use solidmesh::bevel::sections::section_routing;
use solidmesh::topology::bevel_edge_group_resolver::{self, BevelEdgeGroupResolution};
use solidmesh_mesh::CancelToken;

use crate::picture_hanger_test_mesh as fixture;
use crate::surface_test_build::{
    pinned_to_the_sections, the_closed_readable_mesh, try_bevel_resolutions, TryBevelOptions,
    TryBevelOutcome,
};

/// THE CLAIM: the hanger's top rim takes its 0.4 fillet through the sections, closed, manifold and
/// in one shell.
#[test]
fn the_top_rim_takes_its_fillet_through_the_sections() {
    let _sections = pinned_to_the_sections();

    the_closed_readable_mesh(&the_top_rim_at(fixture::SIZE));
}

/// It goes through the sections, not the edge-centric fallback: the rim passes every clause of the
/// router's reading and its room fits the 0.4.
#[test]
fn the_top_rim_is_routed_to_the_sections() {
    let _sections = pinned_to_the_sections();

    the_top_rim_at(fixture::SIZE);

    assert_eq!(section_routing::last_route().as_deref(), Some("sections"));
}

/// The group the file carries really is the ten edge rim of the top face, on this copy too.
#[test]
fn the_stored_pick_resolves_to_the_top_faces_ten_edge_rim() {
    let part = fixture::the_part();
    let resolution = the_rim(&part);

    assert!(resolution.is_resolved(), "{:?}", resolution.failure_reason);
    assert_eq!(resolution.edges.len(), 10);

    for edge in &resolution.edges {
        assert!((edge.start.z - 20.0).abs() < 1e-4);
        assert!((edge.end.z - 20.0).abs() < 1e-4);
    }
}

/// THE TOP FACE AS THE PART HOLDS IT, read off the fixture before anything moves: what the re-mesh
/// is handed, and the three points of it that a 0.4 offset must walk over.
///
/// The face is ten triangles on eleven corners, and its outline IS the picked rim. The eleventh
/// corner, (25.5, -6.1, 20), is INSIDE the ring: the tip of a sliver fan the boolean left where the
/// 50 mm front edge steps 0.1 mm in. It and two rim corners, (30.5, -6.1, 20) and (30.5, -6, 20),
/// stand 0.1 from the boundary edges they are not ends of; every other corner stands 3.9 or more.
/// The boolean left three vertices at that interior position, of which the top face uses one, and
/// that index is used by faces of the top face alone - the precondition `SectionFanRemesh` checks
/// before it drops such a point.
#[test]
fn the_top_face_carries_a_point_a_tenth_of_a_millimetre_from_its_own_rim() {
    let part = fixture::the_part();
    let face = the_top_face(&part);

    assert_eq!(face.len(), 10, "the top face is ten triangles");

    let mut used = BTreeSet::new();

    for &triangle in &face {
        used.insert(part.faces[triangle].v0);
        used.insert(part.faces[triangle].v1);
        used.insert(part.faces[triangle].v2);
    }

    assert_eq!(used.len(), 11, "on eleven corners");

    let outline = the_once_used_edges_of(&part, &face);

    assert_eq!(outline.len(), 10, "whose outline is ten edges");

    let mut on_the_outline = BTreeSet::new();

    for &(from, to) in &outline {
        on_the_outline.insert(from);
        on_the_outline.insert(to);
    }

    assert_eq!(
        on_the_outline.len(),
        10,
        "chaining into one ring of ten - the picked rim itself"
    );

    let mut close: Vec<Vector3> = Vec::new();

    for &corner in &used {
        let mut room = f64::MAX;

        for &(from, to) in &outline {
            if corner != from && corner != to {
                room = room.min(distance_to_the_segment(&part, corner, from, to));
            }
        }

        if room < fixture::SIZE {
            close.push(Vector3::from_vector3_float(part.vertices[corner as usize]));

            assert!(
                (room - 0.1).abs() <= 1e-3,
                "the close corners are a tenth away, and {corner} is {room}"
            );
        } else {
            assert!(
                room > 3.8,
                "and every other corner has ten times the room - {corner} has {room}"
            );
        }
    }

    assert_eq!(
        close.len(),
        3,
        "three corners stand inside the 0.4 the row asks: {close:?}"
    );

    let the_interior_point = Vector3::new(25.5, -6.1, 20.0);
    let mut at_that_position = 0;

    for vertex in &part.vertices {
        if (Vector3::from_vector3_float(*vertex) - the_interior_point).length() < 1e-4 {
            at_that_position += 1;
        }
    }

    assert_eq!(
        at_that_position, 3,
        "and the boolean left three separate vertices at the interior one"
    );

    // AND THE DROP IS SAFE: the interior corner's INDEX is used by faces of the top face alone, so
    // leaving it out when the face is drawn again cannot strand a corner some other face still
    // stands on. This is the precondition SectionFanRemesh checks before it drops such a point.
    let on_the_top_face: HashSet<usize> = face.iter().copied().collect();

    for &corner in &used {
        if on_the_outline.contains(&corner) {
            continue;
        }

        for (i, triangle) in part.faces.iter().enumerate() {
            if triangle.v0 == corner || triangle.v1 == corner || triangle.v2 == corner {
                assert!(
                    on_the_top_face.contains(&i),
                    "the interior corner {corner} is used by face {i}, outside the top face"
                );
            }
        }
    }
}

/// The part's upward triangles at z = 20 - the face the row's rim goes round.
fn the_top_face(part: &Mesh) -> Vec<usize> {
    // The mesh stores f32 coordinates; the subtraction happens in f32 so the 1e-4 band reads them
    // at their own precision.
    let at_twenty = |vertex: i32| ((part.vertices[vertex as usize].z - 20.0) as f64).abs() < 1e-4;
    let mut face = Vec::new();

    for (i, triangle) in part.faces.iter().enumerate() {
        if (triangle.normal.z as f64) > 0.99
            && at_twenty(triangle.v0)
            && at_twenty(triangle.v1)
            && at_twenty(triangle.v2)
        {
            face.push(i);
        }
    }

    face
}

/// The boundary of a set of triangles - the edges exactly one of them uses,
/// in the order they were first met, so the rim reads the same on every run.
fn the_once_used_edges_of(part: &Mesh, face: &[usize]) -> Vec<(i32, i32)> {
    let mut order: Vec<(i32, i32)> = Vec::new();
    let mut uses: HashMap<(i32, i32), i32> = HashMap::new();

    for &triangle in face {
        let at = &part.faces[triangle];

        for (from, to) in [(at.v0, at.v1), (at.v1, at.v2), (at.v2, at.v0)] {
            let key = if from < to { (from, to) } else { (to, from) };
            let count = uses.entry(key).or_insert(0);

            if *count == 0 {
                order.push(key);
            }

            *count += 1;
        }
    }

    order.into_iter().filter(|key| uses[key] == 1).collect()
}

/// How far a corner stands from a boundary edge, in the face's own
/// plane.
fn distance_to_the_segment(part: &Mesh, corner: i32, from: i32, to: i32) -> f64 {
    let flat = |vertex: i32| {
        let at = part.vertices[vertex as usize];
        Vector2::new(at.x as f64, at.y as f64)
    };
    let at = flat(corner);
    let start = flat(from);
    let end = flat(to);

    let along = end - start;
    let length_squared = along.length_squared();

    let t = if length_squared == 0.0 {
        0.0
    } else {
        0f64.max(1f64.min((at - start).dot(along) / length_squared))
    };

    (at - (start + (along * t))).length()
}

/// The rim alone, built the part builder's own way - see `surface_test_build`.
fn the_top_rim_at(size: f64) -> TryBevelOutcome {
    let part = fixture::the_part();

    try_bevel_resolutions(
        &part,
        &[the_rim(&part)],
        size,
        fixture::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    )
}

/// The top face's rim, resolved off the fixture's stored group.
fn the_rim(part: &Mesh) -> BevelEdgeGroupResolution {
    let (mut resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&[fixture::the_top_face_rim()], part);
    resolutions.remove(0)
}
