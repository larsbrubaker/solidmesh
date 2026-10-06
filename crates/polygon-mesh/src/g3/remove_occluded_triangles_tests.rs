//! Rust-only tests for `RemoveOccludedTriangles` (`remove_occluded_triangles`), the fast winding
//! number (`fast_winding`) and the Advanced repair's remove modes (`advanced_repair`).
//! geometry3Sharp and MatterCAD ship no tests for them; these pin the geometric meaning: a closed
//! cube winds once around its inside and not at all outside, and both remove modes drop a shell
//! sealed inside another while keeping the outer one.

use super::dmesh3::DMesh3;
use super::dmesh_aabb_tree3::DMeshAABBTree3;
use super::fast_winding::{tri_solid_angle, FastWinding};
use super::remove_occluded_triangles::{CalculationMode, RemoveOccludedTriangles};
use crate::solids::platonic_solids::create_cube;
use crate::{AdvancedRepairOptions, CancelToken, Mesh, RemoveModes};

/// A 20 mm cube with a 10 mm cube sealed inside it, both wound outward.
fn cube_in_cube() -> Mesh {
    let mut mesh = create_cube(20.0, 20.0, 20.0);
    let inner = create_cube(10.0, 10.0, 10.0);
    mesh.copy_faces(&inner);
    mesh
}

#[test]
fn rust_only_tri_solid_angle_of_a_cube_sums_to_four_pi_inside() {
    let cube = create_cube(10.0, 10.0, 10.0).to_dmesh3();
    let total: f64 = cube
        .triangle_indices()
        .map(|t| {
            let [a, b, c] = cube.get_tri_vertices(t as usize);
            tri_solid_angle(a, b, c, [1.0, 2.0, -0.5])
        })
        .sum();
    assert!((total - 4.0 * std::f64::consts::PI).abs() < 1e-9);
}

#[test]
fn rust_only_fast_winding_number_is_one_inside_and_zero_outside() {
    // Subdivided enough that the tree has internal boxes with far-field caches.
    let mesh: DMesh3 = {
        let mut m = Mesh::new();
        for i in 0..4 {
            let mut c = create_cube(10.0, 10.0, 10.0);
            c.translate_xyz(f64::from(i) * 30.0, 0.0, 0.0);
            m.copy_faces(&c);
        }
        m.to_dmesh3()
    };
    let tree = DMeshAABBTree3::new(&mesh);
    let winding = FastWinding::new(&tree);
    for i in 0..4 {
        let x = f64::from(i) * 30.0;
        assert!((winding.fast_winding_number([x + 1.0, -2.0, 3.0]) - 1.0).abs() < 1e-3);
        assert!(winding.fast_winding_number([x + 15.0, 0.0, 0.0]).abs() < 1e-3);
    }
    assert!(winding.fast_winding_number([500.0, 40.0, -70.0]).abs() < 1e-3);
}

#[test]
fn rust_only_both_modes_remove_the_sealed_inner_cube() {
    for mode in [
        CalculationMode::FastWindingNumber,
        CalculationMode::SimpleOcclusionTest,
    ] {
        let mut mesh = cube_in_cube().to_dmesh3();
        let mut remove = RemoveOccludedTriangles::new(mode);
        remove.per_vertex = true;
        assert!(remove.apply(&mut mesh));
        assert_eq!(remove.removed_t, (12..24).collect::<Vec<_>>(), "{mode:?}");
        assert!(!remove.remove_failed);
        assert_eq!(mesh.triangle_count(), 12, "{mode:?}");
        assert_eq!(mesh.vertex_count(), 8, "{mode:?}");
        assert!(mesh.is_closed());
    }
}

#[test]
fn rust_only_centroid_test_removes_the_sealed_inner_cube() {
    let mut mesh = cube_in_cube().to_dmesh3();
    let mut remove = RemoveOccludedTriangles::new(CalculationMode::FastWindingNumber);
    assert!(remove.apply(&mut mesh));
    assert_eq!(mesh.triangle_count(), 12);
}

#[test]
fn rust_only_advanced_repair_remove_modes_keep_only_the_outer_cube() {
    for remove_mode in [RemoveModes::Interior, RemoveModes::Occluded] {
        let options = AdvancedRepairOptions {
            weld_vertices: false,
            remove_mode,
            ..AdvancedRepairOptions::default()
        };
        let repaired = cube_in_cube()
            .advanced_repaired(&options, &CancelToken::new())
            .expect("the repair runs");
        assert_eq!(repaired.faces.len(), 12, "{remove_mode:?}");
        assert_eq!(repaired.vertices.len(), 8, "{remove_mode:?}");
        assert!((repaired.get_volume() - 8000.0).abs() < 1e-6);
    }
}

#[test]
fn rust_only_a_cancelled_advanced_repair_returns_the_source() {
    let cancel = CancelToken::new();
    cancel.cancel();
    let options = AdvancedRepairOptions {
        weld_vertices: false,
        face_orientation: true,
        ..AdvancedRepairOptions::default()
    };
    let source = cube_in_cube();
    let repaired = source
        .advanced_repaired(&options, &cancel)
        .expect("a cancel is not an error");
    assert_eq!(repaired.faces.len(), source.faces.len());
}
