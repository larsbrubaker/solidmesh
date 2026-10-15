//! Tests for the remeshing pieces (`mesh_constraints`, `mesh_refiner_base`, `mesh_util`,
//! `remesher`, `remesher_pro`). They pin the behavior `MinimalHoleFill` relies on: a fixed
//! boundary stays put, a flat patch stays flat and reaches the target edge lengths, and the
//! flip/collapse checks and constraint rules give the expected answers.

use std::rc::Rc;

use super::dmesh3::DMesh3;
use super::mesh_constraints::{
    fix_all_boundary_edges, same_target, MeshConstraints, ProjectionTarget, TargetRef,
    VertexConstraint,
};
use super::mesh_refiner_base::MeshRefinerBase;
use super::mesh_util::{
    check_if_collapse_creates_flip, check_if_edge_flip_creates_flip, discrete_gauss_curvature,
    get_edge_flip_normals, get_edge_flip_tris, uniform_smooth,
};
use super::remesher::TargetProjectionMode;
use super::remesher_pro::RemesherPro;
use super::vector3d::{cross, sub};

/// An `n` x `n` grid of unit cells in z = 0, each split along one diagonal, wound +z.
fn grid(n: i32, size: f64) -> DMesh3 {
    let mut m = DMesh3::default();
    for j in 0..=n {
        for i in 0..=n {
            m.append_vertex([i as f64 * size, j as f64 * size, 0.0]);
        }
    }
    let id = |i: i32, j: i32| j * (n + 1) + i;
    for j in 0..n {
        for i in 0..n {
            m.append_triangle(id(i, j), id(i + 1, j), id(i + 1, j + 1));
            m.append_triangle(id(i, j), id(i + 1, j + 1), id(i, j + 1));
        }
    }
    m
}

/// Two triangles sharing edge [0,2]: a unit square in z = 0, wound +z.
fn square() -> DMesh3 {
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ] {
        m.append_vertex(p);
    }
    m.append_triangle(0, 1, 2);
    m.append_triangle(0, 2, 3);
    m
}

fn boundary_positions(m: &DMesh3) -> Vec<(i32, [f64; 3])> {
    let mut out: Vec<(i32, [f64; 3])> = m
        .vertex_indices()
        .filter(|&v| m.is_boundary_vertex(v))
        .map(|v| (v, m.vertex(v)))
        .collect();
    out.sort_by_key(|p| p.0);
    out
}

fn total_area(m: &DMesh3) -> f64 {
    m.triangle_indices()
        .map(|t| {
            let [a, b, c] = m.get_tri_vertices(t as usize);
            0.5 * cross(sub(b, a), sub(c, a))[2]
        })
        .sum()
}

/// The expected values come from an independent sequential run of the reference remesher on
/// the same grid with the same settings: the counts, the one folded corner triangle, and a
/// position checksum all match bit for bit.
#[test]
fn rust_only_remesh_flat_patch_matches_csharp_and_keeps_boundary() {
    let mut m = grid(2, 1.0);
    let before = boundary_positions(&m);
    {
        let mut r = RemesherPro::new(&mut m);
        r.remesher.smooth_speed_t = 1.0;
        r.fix_all_boundary_edges();
        r.remesher.set_target_edge_length(0.25);
        r.fastest_remesh(25, true).expect("no flip exception");
    }

    assert_eq!(m.triangle_count(), 206);
    assert_eq!(m.vertex_count(), 108);
    assert_eq!(m.edge_count(), 313);
    assert_eq!(m.max_vertex_id(), 709);
    for v in m.vertex_indices() {
        assert_eq!(m.vertex(v)[2], 0.0, "vertex {v} left the plane");
    }
    // The remesher (normal-flip prevention off, as MinimalHoleFill leaves it) folds one sliver
    // at the (2,2) corner; the rest face +z and the patch keeps its area
    let folded: Vec<i32> = m
        .triangle_indices()
        .filter(|&t| {
            let [a, b, c] = m.get_tri_vertices(t as usize);
            cross(sub(b, a), sub(c, a))[2] <= 0.0
        })
        .collect();
    assert_eq!(folded, vec![291]);
    assert_eq!(m.get_triangle(291), [296, 8, 31]);
    assert_eq!(
        m.get_tri_vertices(291),
        [
            [1.598306613209306, 1.9460105710754432, 0.0],
            [2.0, 2.0, 0.0],
            [1.3808657514245506, 1.8880631240203851, 0.0]
        ]
    );
    assert!((total_area(&m) - 4.0).abs() < 1e-9);
    let mut hash = 0.0;
    for v in m.vertex_indices() {
        let p = m.vertex(v);
        hash += p[0] * (v + 1) as f64 + p[1] * (v + 7) as f64;
    }
    assert_eq!(hash, 61745.14515216437);

    assert_eq!(boundary_positions(&m), before, "the fixed boundary moved");
}

#[test]
fn rust_only_fixed_boundary_constraints_pin_edges_and_vertices() {
    let m = grid(2, 1.0);
    let mut cons = MeshConstraints::default();
    fix_all_boundary_edges(&mut cons, &m);
    for e in m.edge_indices() {
        let ec = cons.get_edge_constraint(e);
        if m.is_boundary_edge(e) {
            assert!(ec.no_modifications());
            assert!(!ec.can_flip() && !ec.can_split() && !ec.can_collapse());
        } else {
            assert!(ec.is_unconstrained());
            assert!(!cons.has_edge_constraint(e));
        }
    }
    // the center vertex (4) is the only interior one
    for v in m.vertex_indices() {
        assert_eq!(cons.get_vertex_constraint(v).fixed, v != 4, "vertex {v}");
    }
    assert!(cons.has_constraints());
}

#[test]
fn rust_only_collapse_resolves_vertex_constraints() {
    let mut m = grid(2, 1.0);
    let mut base = MeshRefinerBase::new(&mut m);
    // no constraints: anything may collapse, to the midpoint
    assert_eq!(base.can_collapse_vtx(0, 0, 4), (true, -1));

    let mut cons = MeshConstraints::default();
    cons.set_or_update_vertex_constraint(0, VertexConstraint::pinned());
    base.set_external_constraints(cons);
    // a pinned vertex is kept
    assert_eq!(base.can_collapse_vtx(0, 0, 4), (true, 0));
    assert_eq!(base.can_collapse_vtx(0, 4, 0), (true, 0));

    // two pinned vertices with no shared set id cannot collapse
    let cons = base.constraints.as_mut().expect("set above");
    cons.set_or_update_vertex_constraint(4, VertexConstraint::pinned());
    assert_eq!(base.can_collapse_vtx(0, 0, 4), (false, -1));

    // ... but may when they share a valid set id
    let cons = base.constraints.as_mut().expect("set above");
    let id = cons.allocate_set_id();
    cons.set_or_update_vertex_constraint(0, VertexConstraint::new_fixed(true, id));
    cons.set_or_update_vertex_constraint(4, VertexConstraint::new_fixed(true, id));
    assert_eq!(base.can_collapse_vtx(0, 0, 4), (true, -1));
    assert!(base.vertex_is_fixed(0) && base.vertex_is_constrained(4));
    assert!(!base.vertex_is_constrained(1));
}

struct ZPlane;
impl ProjectionTarget for ZPlane {
    fn project(&self, p: [f64; 3], _identifier: i32) -> [f64; 3] {
        [p[0], p[1], 0.0]
    }
}

#[test]
fn rust_only_projection_targets_compare_by_reference() {
    let a: TargetRef = Some(Rc::new(ZPlane));
    let b: TargetRef = Some(Rc::new(ZPlane));
    assert!(same_target(&None, &None));
    assert!(same_target(&a, &a.clone()));
    assert!(!same_target(&a, &b));
    assert!(!same_target(&a, &None));
}

#[test]
fn rust_only_remesh_projects_interior_onto_target() {
    // a grid whose interior vertex is lifted; projection after refinement flattens it
    let mut m = grid(2, 1.0);
    m.set_vertex(4, [1.0, 1.0, 0.5]);
    {
        let mut r = RemesherPro::new(&mut m);
        r.fix_all_boundary_edges();
        r.remesher.set_projection_target(Some(Rc::new(ZPlane)));
        assert_eq!(
            r.remesher.projection_mode,
            TargetProjectionMode::AfterRefinement
        );
        r.remesher.set_target_edge_length(0.5);
        r.fastest_remesh(25, true).expect("no flip exception");
    }
    for v in m.vertex_indices() {
        assert_eq!(m.vertex(v)[2], 0.0, "vertex {v} not projected");
    }
    // the reference sequential run of the same remesh (see the flat-patch test)
    assert_eq!(
        (
            m.triangle_count(),
            m.vertex_count(),
            m.edge_count(),
            m.max_vertex_id()
        ),
        (40, 25, 64, 41)
    );
    let mut hash = 0.0;
    for v in m.vertex_indices() {
        let p = m.vertex(v);
        hash += p[0] * (v + 1) as f64 + p[1] * (v + 7) as f64 + p[2] * (v + 3) as f64;
    }
    assert_eq!(hash, 1215.0993063138017);
}

#[test]
fn rust_only_mesh_util_flip_and_collapse_checks() {
    let m = square();
    // edge [0,2] is the shared diagonal
    let e = m.find_edge(0, 2);
    assert!(!m.is_boundary_edge(e));
    let (t0, t1, f0, f1) = get_edge_flip_tris(&m, e);
    // triangle 0 = [0,1,2] holds the edge as 2->0, with 1 opposite; 3 is opposite in [0,2,3]
    assert_eq!((t0, t1), ([2, 0, 1], [0, 2, 3]));
    assert_eq!((f0, f1), ([1, 3, 0], [3, 1, 2]));
    let (n1, n2, on1, on2) = get_edge_flip_normals(&m, e);
    for n in [n1, n2, on1, on2] {
        assert_eq!(n, [0.0, 0.0, 1.0]);
    }
    // flipping a convex quad's diagonal keeps the normals
    assert!(!check_if_edge_flip_creates_flip(&m, e, 0.0));

    // the square has no triangles beyond the diagonal's two, so no collapse can flip one
    assert!(!check_if_collapse_creates_flip(&m, e, [0.5, 0.5, 0.0]));

    // in the 2x2 grid, collapsing the center edge [0,4] inside the one-ring is fine; past
    // the edge [1,5] it turns triangle [1,5,4] over
    let g = grid(2, 1.0);
    let e04 = g.find_edge(0, 4);
    assert!(!check_if_collapse_creates_flip(&g, e04, [0.5, 0.5, 0.0]));
    assert!(check_if_collapse_creates_flip(&g, e04, [3.0, 0.5, 0.0]));

    // a non-convex quad (reflex at vertex 0): the flipped diagonal [1,3] runs outside it,
    // folding a triangle over
    let mut k = square();
    k.set_vertex(0, [0.6, 0.6, 0.0]);
    let ek = k.find_edge(0, 2);
    assert!(check_if_edge_flip_creates_flip(&k, ek, 0.0));
}

#[test]
fn rust_only_tri_angles_curvature_and_smoothing() {
    let m = square();
    // triangle 0 is [0,1,2]: right angle at 1, 45 degrees at 0 and 2
    let q = std::f64::consts::FRAC_PI_4;
    assert!((m.get_tri_internal_angle_r(0, 0) - q).abs() < 1e-12);
    assert!((m.get_tri_internal_angle_r(0, 1) - 2.0 * q).abs() < 1e-12);
    assert!((m.get_tri_internal_angle_r(0, 2) - q).abs() < 1e-12);

    let g = grid(2, 1.0);
    // flat interior vertex: zero curvature; a lifted one is not
    assert!(discrete_gauss_curvature(&g, 4).abs() < 1e-12);
    let mut h = g.clone();
    h.set_vertex(4, [1.0, 1.0, 0.5]);
    assert!(discrete_gauss_curvature(&h, 4) < -1e-3);

    // uniform smoothing of the lifted center toward its flat one-ring
    assert_eq!(h.vtx_one_ring_centroid(4), [1.0, 1.0, 0.0]);
    assert_eq!(uniform_smooth(&h, 4, 1.0), [1.0, 1.0, 0.0]);
    assert_eq!(uniform_smooth(&h, 4, 0.5), [1.0, 1.0, 0.25]);
}
