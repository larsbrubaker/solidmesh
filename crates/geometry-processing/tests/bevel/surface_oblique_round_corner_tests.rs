//! Port of `SurfaceObliqueRoundCornerTests.cs`: a ROUND corner where the three wedges are all
//! different. `skew_ridge_solid`'s skew ends at 112, 142 and 146 degrees, whose three rim arcs sweep
//! 68, 38 and 34 degrees and so are chorded 6, 3 and 3 times at eight segments to the quarter. The
//! square corner's grid needs all three counts equal; this corner has no such grid and is built as
//! parametric rows zipped instead.
//!
//! The rows are laid from the apex opposite the LONGEST side; two neighbouring rows are sewn as a
//! zipper, which lets a row of three chords hand over to a row of four without a sliver.
//! `surface_corner_tests.rs` holds the square corner, which this construction has to leave exactly
//! as it was, and `the_octant_is_unchanged` is that promise written down as the octant's own face
//! list. C# `[Arguments]` rows are a loop.

use std::collections::HashMap;
use std::fmt::Write;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildOptions,
};
use geometry_processing::bevel::{bevel_arc_chording, bevel_chain_linker};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::csg::{boolean_processing, manifold_import, BooleanOperandVerdict};
use polygon_mesh::{CancelToken, Face, Mesh};
use sha2::{Digest, Sha256};
use vector_math::{math_helper, Matrix4X4, Vector3};

use crate::half_space_solids::{self, Wall};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::skew_ridge_solid;
use crate::surface_corner_tests::the_filleted_cube_corner;

const SIZE: f64 = 3.0;

const RIDGE_LENGTH: f64 = 25.0;

/// The face list of the cube corner, taken as a hash of every face's three corners in the order
/// they are published, rounded to a nanometre. A change of winding, of vertex order, or of any
/// position beyond that rounding moves it.
const THE_OCTANT_AS_IT_WAS: &str =
    "fd7519522a45f1c2117c3f75120938b02225441f70af1dc471132267aa00c98f/348";

/// The patch is part of the ball. Every vertex the corner cell holds stands a radius from the ball
/// centre, which is what makes the rim and the patch one surface rather than two.
#[test]
fn every_patch_vertex_stands_a_radius_from_the_centre() {
    let built = rounded(RIDGE_LENGTH, 8);
    let centre = ball_centre_at_the_near_corner(RIDGE_LENGTH);
    let in_the_cell = faces_near_the_centre(&built, centre);

    assert!(
        !in_the_cell.is_empty(),
        "the corner cell holds the patch, and an empty cell means the patch went somewhere else"
    );

    for face in &in_the_cell {
        for &at in face {
            assert!(
                ((at - centre).length() - SIZE).abs() <= 1e-6,
                "the patch is the ball's own surface, and {at:?} is not on it"
            );
        }
    }
}

/// The patch's rim is the strips' OWN end columns, shared by index rather than minted again: every
/// edge of every face in the corner cell is used by exactly two faces of the whole mesh.
#[test]
fn the_rim_is_the_strips_own_end_columns() {
    let built = rounded(RIDGE_LENGTH, 8);
    let used = edge_use(&built);
    let centre = ball_centre_at_the_near_corner(RIDGE_LENGTH);
    let mut in_the_cell = 0;

    for face in built.faces.iter() {
        let corners = corners_of(&built, face);

        if !corners
            .iter()
            .all(|&at| (at - centre).length() < SIZE + 1e-6)
        {
            continue;
        }

        in_the_cell += 1;

        for edge in edges_of(face) {
            assert_eq!(
                used[&edge], 2,
                "the patch hands over to the strips along their own end columns, and the edge at \
                 {:?} is used {} times",
                corners[0], used[&edge]
            );
        }
    }

    assert!(
        in_the_cell > 0,
        "the corner cell holds the patch, and an empty cell means the patch went somewhere else"
    );
}

/// The same corner at eight and at sixteen segments to the quarter, where the three sides carry 6,
/// 3 and 3 chords and then 12, 7 and 6. Nothing the zipper emits may have no area in it.
#[test]
fn three_different_chord_counts_zip_without_a_sliver() {
    for segments_per_quarter in [8, 16] {
        let built = rounded(RIDGE_LENGTH, segments_per_quarter);
        let centre = ball_centre_at_the_near_corner(RIDGE_LENGTH);
        let in_the_cell = faces_near_the_centre(&built, centre);

        assert!(
            !in_the_cell.is_empty(),
            "the corner cell holds the patch, and an empty cell means the patch went somewhere else"
        );

        for face in &in_the_cell {
            let spread = (face[1] - face[0]).cross(face[2] - face[0]).length() / 2.0;

            assert!(
                spread > 1e-9,
                "a zipper that advances the same polyline twice leaves a sliver, and {:?} is on one \
                 ({segments_per_quarter} segments)",
                face[0]
            );
        }
    }
}

/// The square corner is untouched by the ROWS. Three arcs of one count still lay the same rows the
/// grid laid - the same order, the same winding, connectivity for connectivity.
#[test]
fn the_octant_is_unchanged() {
    assert_eq!(
        the_face_list_of(&the_filleted_cube_corner()),
        THE_OCTANT_AS_IT_WAS,
        "the rows reduce to the grid when the three counts agree - refined alike - and any other \
         reading of them is a change to the case the ladder was measured on"
    );
}

/// Nothing is built twice. The bottom face of the fixture carries slivers lying along the very
/// lines the end cuts are bounded by; such a facet used to be kept by BOTH of that side's clips and
/// emitted twice over.
#[test]
fn no_face_is_emitted_twice() {
    for segments_per_quarter in [2, 8, 16] {
        let built = rounded(RIDGE_LENGTH, segments_per_quarter);

        // C#'s GroupBy keeps the groups in the order their first face was met.
        let mut order: Vec<(i32, i32, i32)> = Vec::new();
        let mut groups: HashMap<(i32, i32, i32), Vec<&Face>> = HashMap::new();

        for face in built.faces.iter() {
            groups
                .entry(sorted(face))
                .or_insert_with(|| {
                    order.push(sorted(face));
                    Vec::new()
                })
                .push(face);
        }

        let doubled: Vec<&Vec<&Face>> = order
            .iter()
            .map(|key| &groups[key])
            .filter(|group| group.len() > 1)
            .collect();

        assert_eq!(
            doubled.len(),
            0,
            "the face at {:?} is emitted {} times ({segments_per_quarter} segments)",
            doubled.first().map(|group| corners_of(&built, group[0])[0]),
            doubled.first().map_or(0, |group| group.len())
        );
    }
}

/// The whole fixture rounded is a closed, manifold, single shell - and one the BOOLEAN KERNEL
/// takes, which is the reading a Combine after a Fillet depends on.
#[test]
fn the_rounded_skew_ridge_is_one_closed_shell_the_kernel_takes() {
    let built = rounded(RIDGE_LENGTH, 8);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);

    assert_eq!(
        open, 0,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    // C# imports with `Manifold.FromMeshGL64Robust` on `SurfaceResultImportsIntoManifoldTests
    // .AsMeshGL64`: the strict import, the same marshalling with no weld-and-retry.
    let imported = manifold_import::import_strict(&built).unwrap_or_else(|rejected| {
        panic!(
            "the kernel is the next thing to read a fillet after the builder publishes it - \
             {rejected:?}"
        )
    });

    let kernel_volume = imported.volume();
    let measured = signed_volume(&built).abs();

    assert!(
        (kernel_volume - measured).abs() <= 1e-6,
        "the kernel holds the same solid the mesh measures as - {kernel_volume} against {measured}"
    );

    assert_eq!(
        boolean_processing::classify_boolean_operand(&built, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::Clean,
        "a Combine after a Fillet hands this very mesh over as an operand"
    );
}

/// And it is the RIGHT solid, measured against the polytope of its own chords: a round bevel of a
/// convex solid is that solid intersected with the profile's chord planes, one per chord per edge.
/// The two differ only by the corner cells' chording - two hundredths of a cubic millimetre in
/// twenty-three thousand.
#[test]
fn it_keeps_exactly_what_its_own_chord_planes_keep() {
    for segments_per_quarter in [2, 4, 8, 16] {
        let built = rounded(RIDGE_LENGTH, segments_per_quarter);

        let kept =
            half_space_solids::volume(&walls_with_the_rounds(RIDGE_LENGTH, segments_per_quarter));
        let volume = signed_volume(&built);

        assert!(
            (volume - kept).abs() <= 0.05,
            "rounding a convex solid is intersecting it with its profile's own chord planes, and \
             all that may differ is how the two read the corner cells - {volume} against {kept} \
             ({segments_per_quarter} segments)"
        );
    }
}

/// C# `TheFaceListOf`: the mesh's faces as one string - each face's three corners in published
/// order, rounded to a nanometre - hashed, with the face count after a slash.
pub fn the_face_list_of(mesh: &Mesh) -> String {
    let mut said = String::new();

    for face in mesh.faces.iter() {
        for index in [face.v0, face.v1, face.v2] {
            let at = Vector3::from_vector3_float(mesh.vertices[index as usize]);

            let _ = write!(said, "{:.9},{:.9},{:.9};", at.x, at.y, at.z);
        }

        said.push('|');
    }

    let hash = Sha256::digest(said.as_bytes());
    let hex: String = hash.iter().map(|b| format!("{b:02x}")).collect();

    format!("{hex}/{}", mesh.faces.len())
}

/// The ball centre at the fixture's +x corner: the one point standing the radius in from all three
/// faces there, read off the fixture's own walls rather than off the build.
fn ball_centre_at_the_near_corner(ridge_length: f64) -> Vector3 {
    let walls = skew_ridge_solid::walls(ridge_length);
    let normals = [
        walls[0].facing.get_normal(),
        walls[1].facing.get_normal(),
        walls[2].facing.get_normal(),
    ];

    let determinant = normals[0].dot(normals[1].cross(normals[2]));

    let first = normals[1].cross(normals[2]);
    let second = normals[2].cross(normals[0]);
    let third = normals[0].cross(normals[1]);

    skew_ridge_solid::near_corner(ridge_length)
        + (((first * -SIZE) + (second * -SIZE) + (third * -SIZE)) / determinant)
}

/// The faces standing wholly within a radius and a rounding of the ball centre, which at a corner
/// this size is the patch and nothing else.
fn faces_near_the_centre(mesh: &Mesh, centre: Vector3) -> Vec<[Vector3; 3]> {
    mesh.faces
        .iter()
        .map(|face| corners_of(mesh, face))
        .filter(|corners| {
            corners
                .iter()
                .all(|&at| (at - centre).length() < SIZE + 1e-6)
        })
        .collect()
}

/// How many faces of the mesh carry each of its edges, the edge taken as its two vertices.
fn edge_use(mesh: &Mesh) -> HashMap<(i32, i32), i32> {
    let mut used = HashMap::new();

    for face in mesh.faces.iter() {
        for edge in edges_of(face) {
            *used.entry(edge).or_insert(0) += 1;
        }
    }

    used
}

/// A face's three edges, each with its two vertices in order so that a shared one matches.
fn edges_of(face: &Face) -> [(i32, i32); 3] {
    [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)].map(|(from, to)| {
        if from < to {
            (from, to)
        } else {
            (to, from)
        }
    })
}

/// A face's three vertices as one key, so that the same triangle emitted twice is one group.
fn sorted(face: &Face) -> (i32, i32, i32) {
    let mut corners = [face.v0, face.v1, face.v2];

    corners.sort();

    (corners[0], corners[1], corners[2])
}

fn corners_of(mesh: &Mesh, face: &Face) -> [Vector3; 3] {
    [face.v0, face.v1, face.v2].map(|v| Vector3::from_vector3_float(mesh.vertices[v as usize]))
}

/// The fixture's five edges rounded by the surface construction, or the refusal named.
fn rounded(ridge_length: f64, segments_per_quarter: i32) -> Mesh {
    let mesh = skew_ridge_solid::create(ridge_length);
    let graph = mesh.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create(&mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));

    let picked = picked_edges(&features, ridge_length);
    let mut plans = Vec::new();

    for chain in bevel_chain_linker::link(&picked) {
        plans.push(
            SurfaceBevelPlan::try_create(&chain, &regions, SIZE, true)
                .unwrap_or_else(|reason| panic!("the plan refused the chain: {reason}")),
        );
    }

    surface_bevel_builder::try_build(
        &mesh,
        &graph,
        &regions,
        &plans,
        segments_per_quarter,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the corner: {reason:?}"))
    .mesh
}

/// The fixture's own five walls and the chord planes of all five rounded edges: the walls of the
/// rounded solid, chorded exactly as the surface strip chords its rails.
fn walls_with_the_rounds(ridge_length: f64, segments_per_quarter: i32) -> Vec<Wall> {
    let walls = skew_ridge_solid::walls(ridge_length);
    let mut found = walls.clone();

    // The same five edges picked_edges finds, named by the two walls each runs between.
    for (first, second) in [(0, 1), (0, 2), (1, 2), (0, 3), (1, 3)] {
        found.extend(the_chords_of(
            walls[first],
            walls[second],
            segments_per_quarter,
        ));
    }

    found
}

/// The chord planes of one edge's round profile: the arc from the foot on one wall to the foot on
/// the other, sampled at the operation's own count for its sweep, with a plane through each chord
/// parallel to the edge. Each faces out of the material, away from the ball.
fn the_chords_of(first: Wall, second: Wall, segments_per_quarter: i32) -> Vec<Wall> {
    let along = first.facing.cross(second.facing).get_normal();

    // The ball touching both walls from inside: a radius in from each of them, on the plane through
    // the origin square to the edge - three planes, one point.
    let centre = the_one_point(
        first.facing,
        first.offset - SIZE,
        second.facing,
        second.offset - SIZE,
        along,
        0.0,
    );

    let from_centre = first.facing * SIZE;
    let sweep = first.facing.dot(second.facing).clamp(-1.0, 1.0).acos();
    let turning = if from_centre.cross(second.facing * SIZE).dot(along) < 0.0 {
        -1.0
    } else {
        1.0
    };
    let samples = 1.max(bevel_arc_chording::segments_for(
        math_helper::radians_to_degrees(sweep),
        segments_per_quarter,
    ));

    let points: Vec<Vector3> = (0..=samples)
        .map(|i| {
            centre
                + turned(
                    from_centre,
                    along,
                    turning * sweep * i as f64 / samples as f64,
                )
        })
        .collect();

    points
        .windows(2)
        .map(|pair| {
            let facing = (pair[1] - pair[0]).cross(along);

            half_space_solids::through(
                pair[0],
                if facing.dot(pair[0] - centre) < 0.0 {
                    -facing
                } else {
                    facing
                },
            )
        })
        .collect()
}

/// The one point three planes meet at, each given as its normal and its offset.
fn the_one_point(
    first: Vector3,
    first_at: f64,
    second: Vector3,
    second_at: f64,
    third: Vector3,
    third_at: f64,
) -> Vector3 {
    ((second.cross(third) * first_at)
        + (third.cross(first) * second_at)
        + (first.cross(second) * third_at))
        / first.dot(second.cross(third))
}

/// Rodrigues: `vector` turned about `axis`.
fn turned(vector: Vector3, axis: Vector3, radians: f64) -> Vector3 {
    let cos = radians.cos();

    (vector * cos) + (axis.cross(vector) * radians.sin()) + (axis * axis.dot(vector) * (1.0 - cos))
}

fn picked_edges(features: &FeatureEdgeSet, ridge_length: f64) -> Vec<FeatureEdge> {
    let picked: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| skew_ridge_solid::is_selected(edge.start, edge.end, ridge_length))
        .cloned()
        .collect();

    assert_eq!(
        picked.len(),
        5,
        "the fixture should offer the ridge and its four wings"
    );

    picked
}
