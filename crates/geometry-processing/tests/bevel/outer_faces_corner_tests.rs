//! The owner file "Outer faces should still do the bevel" as the app route hands it to the kernel
//! (size 0.4, 10 chains: an L-shaped top face, the -X end face and the +X wall, four hole mouths,
//! and one MIXED corner at (-24.5, -6, 20) where chain 4's first edge is concave). Built through
//! the corner path it published MORE volume than the source; every convex pick must take material.
//! Parameter rows are loops over the same-named test.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{gap_fill, BandProfile, GapFillRefusal};
use solidmesh_math::Vector3;

use crate::corner_pick_with_mouth_tests::concave_bound;
use crate::end_face_builds::feature_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{
    distance_to_surface, signed_volume, signed_volume_of, winding_number,
};
use crate::sweep_mesh_measures;

const CONCAVE_TOP: Vector3 = Vector3::new(-24.5, -6.0, 20.0);

const CONCAVE_FOOT: Vector3 = Vector3::new(-24.5, -6.0, 0.0);

#[test]
fn the_owner_files_pick_takes_material_only_beside_its_concave_edge() {
    for profile in [BandProfile::Fillet, BandProfile::Chamfer] {
        takes_material_only_beside_its_concave_edge(profile);
    }
}

fn takes_material_only_beside_its_concave_edge(profile: BandProfile) {
    let fixture = Rc::new(LevelLineFixture::load("outer-faces-r04"));
    let positions = &fixture.positions;
    let indices = &fixture.indices;
    let before = signed_volume_of(positions, indices);
    let gap = band_gap_entry_points::build_pick(
        positions,
        indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(&fixture)),
        fixture.size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");
    assert!(
        gap.built(),
        "{profile:?}: band {:?}, room {}",
        gap.refusal,
        gap.room
    );

    let filled = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{profile:?}: chain {}, vertex {}",
        filled.offending_chain,
        filled.offending_vertex
    );

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let volume = signed_volume(&mesh);

    // Material is added on purpose beside the one concave edge, within the size of it. Everywhere
    // else an added vertex lies inside or on the part - on it to within the faceting of the input:
    // the four hole mouths are slanted 48-gon bores, and a true arc between two facets stands up to
    // 0.01 proud of them.
    let outside: Vec<Vector3> = filled
        .added_vertices
        .iter()
        .map(|&v| mesh.position(v))
        .filter(|&p| {
            winding_number(positions, indices, p) < 0.45
                && distance_to_surface(positions, indices, p) > 0.02
        })
        .collect();
    let stray: Vec<Vector3> = outside
        .iter()
        .copied()
        .filter(|&p| from_the_concave_edge(p) > fixture.size * 1.5)
        .collect();
    let worst = outside
        .iter()
        .map(|&p| distance_to_surface(positions, indices, p))
        .fold(0.0, f64::max);

    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    assert_eq!(
        stray.len(),
        0,
        "{profile:?}: {:?}",
        stray.iter().take(30).collect::<Vec<_>>()
    );
    assert!(
        volume < before,
        "{profile:?}: volume {volume:?} of {before:?}"
    );

    // Beside the concave edge no more than the profile adds there (see
    // CornerPickWithMouthTests.ConcaveBound).
    let bound = concave_bound(profile, fixture.size, half_opening(positions, indices));
    assert!(
        worst <= bound * (1.0 + 1e-6),
        "{profile:?}: worst {worst} against bound {bound}"
    );
}

/// Half the angle the two faces on the concave edge open by, read from their normals.
fn half_opening(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> f64 {
    let at = |v: i32| positions[v as usize];
    let mut normals: Vec<Vector3> = Vec::new();
    for &(a, b, c) in indices {
        let on_edge = [a, b, c]
            .iter()
            .filter(|&&i| from_the_concave_edge(at(i)) < 1e-9)
            .count();
        if on_edge < 2 {
            continue;
        }

        let normal = (at(b) - at(a)).cross(at(c) - at(a)).get_normal();
        if normals.iter().all(|&n| (n - normal).length() > 1e-6) {
            normals.push(normal);
        }
    }

    // Two faces meet on the edge; the air between them opens by pi minus the angle between the
    // normals.
    let between = normals[0].dot(normals[1]).clamp(-1.0, 1.0).acos();
    (std::f64::consts::PI - between) / 2.0
}

fn from_the_concave_edge(p: Vector3) -> f64 {
    let axis = CONCAVE_FOOT - CONCAVE_TOP;
    let t = ((p - CONCAVE_TOP).dot(axis) / axis.length_squared()).clamp(0.0, 1.0);
    (p - (CONCAVE_TOP + axis * t)).length()
}
