//! Tests for the mesh boundary loops, editor, selection and iterators
//! (`mesh_boundary_loops`, `mesh_editor`, `mesh_face_selection`, `mesh_iterators`,
//! `edge_loop`, `dmesh3_set_triangle`).

use super::dmesh3::{DMesh3, INVALID_ID};
use super::edge_loop::{is_boundary_loop, EdgeLoop, ValidationStatus};
use super::mesh_boundary_loops::MeshBoundaryLoops;
use super::mesh_editor::{
    add_triangle_fan_ordered_vertex_loop, disconnect_all_bowties, remove_isolated_triangles,
    remove_triangles,
};
use super::mesh_face_selection::MeshFaceSelection;
use super::mesh_iterators::{boundary_edges, bowtie_vertices, interior_vertices};

/// A unit cube wound outward, without its two top (z = 1) triangles.
fn open_box() -> DMesh3 {
    let mut m = DMesh3::default();
    for z in [0.0, 1.0] {
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)] {
            m.append_vertex([x, y, z]);
        }
    }
    let quads = [
        [0, 3, 2, 1], // bottom
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
    ];
    for q in quads {
        assert!(m.append_triangle(q[0], q[1], q[2]) >= 0);
        assert!(m.append_triangle(q[0], q[2], q[3]) >= 0);
    }
    m
}

/// Two triangles meeting only at vertex 0.
fn bowtie() -> DMesh3 {
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ] {
        m.append_vertex(p);
    }
    m.append_triangle(0, 1, 2);
    m.append_triangle(0, 3, 4);
    m
}

#[test]
fn rust_only_open_box_has_one_boundary_loop_wound_with_the_boundary() {
    let m = open_box();
    let loops = MeshBoundaryLoops::new(&m);
    assert_eq!(loops.loops.len(), 1);
    assert!(loops.spans.is_empty() && !loops.saw_open_spans);
    let lp = &loops.loops[0];
    assert_eq!(lp.vertex_count(), 4);
    let mut sorted = lp.vertices.clone();
    sorted.sort();
    assert_eq!(sorted, vec![4, 5, 6, 7]);
    assert_eq!(is_boundary_loop(&m, lp), ValidationStatus::Ok);
    assert_eq!(boundary_edges(&m).len(), 4);
    assert_eq!(interior_vertices(&m), vec![0, 1, 2, 3]);
    // a closed mesh has none
    let mut closed = open_box();
    closed.append_triangle(4, 5, 6);
    closed.append_triangle(4, 6, 7);
    assert!(MeshBoundaryLoops::new(&closed).loops.is_empty());
}

#[test]
fn rust_only_a_bowtie_boundary_splits_into_two_simple_loops() {
    let m = bowtie();
    assert_eq!(bowtie_vertices(&m), vec![0]);
    let loops = MeshBoundaryLoops::new(&m);
    assert!(loops.spans.is_empty());
    assert_eq!(loops.loops.len(), 2);
    for lp in &loops.loops {
        assert_eq!(lp.vertex_count(), 3);
        assert!(lp.vertices.contains(&0));
        assert_eq!(lp.bowtie_vertices, vec![0]);
        assert!(!lp.edges.contains(&INVALID_ID));
        assert_eq!(is_boundary_loop(&m, lp), ValidationStatus::Ok);
    }
}

#[test]
fn rust_only_fan_fill_closes_the_box_and_a_refused_fan_backs_out() {
    let mut m = open_box();
    let lp: EdgeLoop = MeshBoundaryLoops::new(&m).loops.remove(0);
    let center = m.append_vertex([0.5, 0.5, 1.0]);
    let tris = add_triangle_fan_ordered_vertex_loop(&mut m, center, &lp.vertices, -1)
        .unwrap()
        .unwrap();
    assert_eq!(tris.len(), 4);
    assert!(m.is_closed());

    // the same fan again would put a third triangle on every edge: refused, nothing kept
    let before = m.triangle_count();
    let center2 = m.append_vertex([0.5, 0.5, 2.0]);
    let refused = add_triangle_fan_ordered_vertex_loop(&mut m, center2, &lp.vertices, -1).unwrap();
    assert!(refused.is_none());
    assert_eq!(m.triangle_count(), before);
}

#[test]
fn rust_only_disconnecting_a_bowtie_gives_the_smaller_fan_its_own_vertex() {
    let mut m = bowtie();
    assert_eq!(disconnect_all_bowties(&mut m, 10), 0);
    assert_eq!(m.vertex_count(), 6);
    assert_eq!(m.get_triangle(0), [0, 1, 2]);
    assert_eq!(m.get_triangle(1), [5, 3, 4]);
    assert_eq!(m.vertex(5), [0.0, 0.0, 0.0]);
    assert!(bowtie_vertices(&m).is_empty());
    assert_eq!(MeshBoundaryLoops::new(&m).loops.len(), 2);
}

#[test]
fn rust_only_loners_and_selected_boundary_triangles_are_removed() {
    let mut m = open_box();
    let a = m.append_vertex([5.0, 5.0, 5.0]);
    let b = m.append_vertex([6.0, 5.0, 5.0]);
    let c = m.append_vertex([5.0, 6.0, 5.0]);
    m.append_triangle(a, b, c);
    assert!(remove_isolated_triangles(&mut m));
    assert_eq!(m.triangle_count(), 10);
    assert!(!m.is_vertex(a));

    // erosion: every triangle on a boundary edge goes
    let mut selection = MeshFaceSelection::new();
    for eid in boundary_edges(&m) {
        selection.select_edge_tris(&m, eid);
    }
    assert_eq!(selection.count(), 4);
    let selected: Vec<i32> = selection.iter().collect();
    assert!(remove_triangles(&mut m, selected, true));
    assert_eq!(m.triangle_count(), 6);
    assert_eq!(MeshBoundaryLoops::new(&m).loops.len(), 1);
}

#[test]
fn rust_only_simple_hole_filler_closes_a_quad_hole_with_a_centroid_fan() {
    use super::simple_hole_filler::SimpleHoleFiller;
    let mut m = open_box();
    let lp = MeshBoundaryLoops::new(&m).loops.remove(0);
    let mut filler = SimpleHoleFiller::new(&lp);
    assert_eq!(filler.validate(&m), ValidationStatus::Ok);
    assert!(filler.fill(&mut m, -1).unwrap());
    assert!(m.is_closed());
    assert_eq!(m.vertex(filler.new_vertex), [0.5, 0.5, 1.0]);
    assert_eq!(filler.new_triangles.as_ref().map(Vec::len), Some(4));

    // filling it again is refused and leaves no stray vertex
    let vertex_count = m.vertex_count();
    let mut again = SimpleHoleFiller::new(&lp);
    assert!(!again.fill(&mut m, -1).unwrap());
    assert_eq!(again.new_vertex, INVALID_ID);
    assert_eq!(m.vertex_count(), vertex_count);
}

#[test]
fn rust_only_simple_hole_filler_closes_a_triangle_hole_with_one_triangle() {
    use super::simple_hole_filler::SimpleHoleFiller;
    // a tetrahedron missing one face
    let mut m = DMesh3::default();
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ] {
        m.append_vertex(p);
    }
    m.append_triangle(0, 2, 1);
    m.append_triangle(0, 1, 3);
    m.append_triangle(1, 2, 3);
    let lp = MeshBoundaryLoops::new(&m).loops.remove(0);
    assert_eq!(lp.vertex_count(), 3);
    let mut filler = SimpleHoleFiller::new(&lp);
    assert!(filler.fill(&mut m, -1).unwrap());
    assert!(m.is_closed());
    assert_eq!(filler.new_vertex, INVALID_ID);
    assert_eq!(m.vertex_count(), 4);
}

/// The open box closed with its top, wound outward.
fn closed_box() -> DMesh3 {
    let mut m = open_box();
    m.append_triangle(4, 5, 6);
    m.append_triangle(4, 6, 7);
    m
}

/// Signed volume (positive when wound outward).
fn signed_volume(m: &DMesh3) -> f64 {
    m.triangle_indices()
        .map(|t| {
            let [a, b, c] = m.get_tri_vertices(t as usize);
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

#[test]
fn rust_only_orientation_makes_a_component_consistent_then_turns_it_outward() {
    use super::mesh_repair_orientation::MeshRepairOrientation;
    let mut m = closed_box();
    // wind the whole box inward, then flip two triangles back so it is inconsistent too;
    // triangle 0 seeds the flood fill, so the fill makes everything agree with it (inward)
    m.reverse_orientation();
    m.reverse_tri_orientation(3);
    m.reverse_tri_orientation(8);
    let mut orient = MeshRepairOrientation::new();
    orient.orient_components(&mut m);
    assert!((signed_volume(&m) + 1.0).abs() < 1e-12);
    orient.solve_global_orientation(&mut m);
    assert!((signed_volume(&m) - 1.0).abs() < 1e-12);
    assert!(m.is_closed());
}

#[test]
fn rust_only_ray_hit_query_finds_the_nearest_face_and_honours_the_filter() {
    use super::ray_intersection::Ray3d;
    use super::DMeshAABBTree3;
    let m = closed_box();
    let tree = DMeshAABBTree3::new(&m);
    // straight down through the middle: the top (triangles 10, 11) first
    let ray = Ray3d::new([0.25, 0.5, 5.0], [0.0, 0.0, -2.0]);
    assert_eq!(ray.direction, [0.0, 0.0, -1.0]);
    let hit = tree
        .find_nearest_hit_triangle(&ray, f64::MAX, None)
        .unwrap();
    assert!(hit == 10 || hit == 11);
    // with the top filtered out, the bottom (triangles 0, 1)
    let not_top = |t: i32| t < 10;
    let hit = tree
        .find_nearest_hit_triangle(&ray, f64::MAX, Some(&not_top))
        .unwrap();
    assert!(hit == 0 || hit == 1);
    // a miss
    let away = Ray3d::new([5.0, 5.0, 5.0], [0.0, 0.0, 1.0]);
    assert_eq!(
        tree.find_nearest_hit_triangle(&away, f64::MAX, None),
        Some(-1)
    );
}
