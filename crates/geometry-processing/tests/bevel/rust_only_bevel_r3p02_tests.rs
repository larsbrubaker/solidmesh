//! Rust-only checks of the bevel core's R3.P02 cut pieces on a hand-built square: the vertex
//! reading's per-vertex cache, the edge roots, the band cut's partition and line, and the side
//! roots' settling. The tests that read these pieces in a whole build are in their own files.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_cut::{cut, side_of_triangle, BandCutError};
use solidmesh::bevel::level_line::edge_root::{find, root_on};
use solidmesh::bevel::level_line::side_roots_settling::{root_the_side, settle_every_side};
use solidmesh::bevel::level_line::{
    LevelLineMesh, ScalarField, VertexFieldReading, VertexSide, WorkError,
};
use solidmesh_math::Vector3;

/// The signed distance to the plane x = 0.5: negative on the x < 0.5 half.
struct HalfPlane;
impl ScalarField for HalfPlane {
    fn value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        Ok(point.x - 0.5)
    }
}

/// A unit square in z = 0 of two triangles, tagged `tag`.
fn square(tag: i32) -> Rc<RefCell<LevelLineMesh>> {
    let mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&[tag, tag]),
    )
    .expect("a square");
    Rc::new(RefCell::new(mesh))
}

#[test]
fn rust_only_a_half_plane_cuts_the_square_along_one_open_line() {
    let reading = VertexFieldReading::new(square(0), Rc::new(HalfPlane), 1e-9).expect("a snap");
    assert_eq!(reading.value_at(1).ok(), Some(0.5));
    assert_eq!(reading.side_of(0).ok(), Some(VertexSide::Inside));
    assert_eq!(reading.side_of(2).ok(), Some(VertexSide::Outside));
    // Read once: only the vertices asked about are in the cache.
    assert_eq!(reading.values.borrow().len(), 3);

    let root = root_on(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        &reading,
    )
    .expect("a root");
    assert!((root - Vector3::new(0.5, 0.5, 0.0)).length() <= 1e-12);
    assert_eq!(reading.remembered_roots.borrow().len(), 1);

    let roots = find(&reading.mesh.borrow(), &reading).expect("the roots");
    // Edges 0-1, 0-2 and 2-3 cross; 0-3 and 1-2 do not. Canonical edge order.
    assert_eq!(
        roots.iter().map(|r| (r.a, r.b)).collect::<Vec<_>>(),
        vec![(0, 1), (0, 2), (2, 3)]
    );

    let result = cut(&reading, &roots).expect("the cut");
    assert_eq!(reading.mesh.borrow().vertex_count(), 7);
    assert_eq!(
        result.inside_triangles.len() + result.outside_triangles.len(),
        6
    );
    assert_eq!(result.lines.len(), 1);
    assert!(!result.lines[0].closed);
    let mut on_line = result.lines[0].vertices.clone();
    on_line.sort();
    assert_eq!(on_line, vec![4, 5, 6]);
    for v in on_line {
        assert_eq!(reading.side_of(v).ok(), Some(VertexSide::On));
        assert!((reading.mesh.borrow().position(v).x - 0.5).abs() <= 1e-12);
    }
    for &t in &result.inside_triangles {
        assert!(reading.mesh.borrow().centroid(t).x < 0.5);
    }

    // A root whose edge is gone is refused by name.
    assert!(matches!(
        cut(&reading, &roots[..1]),
        Err(BandCutError::RootEdgeGone(_))
    ));
}

#[test]
fn rust_only_a_triangle_still_straddling_the_line_is_refused() {
    let reading = VertexFieldReading::new(square(0), Rc::new(HalfPlane), 1e-9).expect("a snap");
    assert!(matches!(
        side_of_triangle(&reading, 0),
        Err(BandCutError::Straddles(_))
    ));
}

#[test]
fn rust_only_settling_roots_each_side_until_a_pass_roots_nothing() {
    let reading = VertexFieldReading::new(square(3), Rc::new(HalfPlane), 1e-9).expect("a snap");
    assert!(!root_the_side(&reading, 5).expect("no such side"));
    assert!(root_the_side(&reading, 3).expect("rooted"));
    assert_eq!(reading.mesh.borrow().vertex_count(), 7);
    assert!(!root_the_side(&reading, 3).expect("settled"));

    // Two sides of one mesh: the pass repeats until neither roots anything.
    let mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&[1, 2]),
    )
    .expect("a square");
    let mesh = Rc::new(RefCell::new(mesh));
    let readings: HashMap<i32, VertexFieldReading> = [1, 2]
        .into_iter()
        .map(|tag| {
            let reading =
                VertexFieldReading::new(mesh.clone(), Rc::new(HalfPlane), 1e-9).expect("a snap");
            (tag, reading)
        })
        .collect();
    settle_every_side(&[1, 2], &readings).expect("settled");
    assert_eq!(mesh.borrow().vertex_count(), 7);
    for tag in [1, 2] {
        assert!(!root_the_side(&readings[&tag], tag).expect("settled"));
    }
}

#[test]
fn rust_only_a_rules_point_is_inserted_where_it_stands() {
    use solidmesh::bevel::level_line::refinement_splits::NOTHING_TO_INSERT;
    use solidmesh::bevel::level_line::refinement_splits_enqueue::{
        insert_on_the_line, insert_where_it_stands,
    };
    use std::collections::VecDeque;
    let mesh = square(0);
    let mut mesh = mesh.borrow_mut();
    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    // At a corner nothing is minted and nothing is queued.
    let at = Vector3::new(0.0, 0.0, 0.0);
    assert_eq!(
        insert_where_it_stands(&mut mesh, 0, at, &mut pending, &mut depth, None).ok(),
        Some(0)
    );
    assert!(pending.is_empty());
    // On edge 0-1 the one triangle there is split, its children queued one deeper.
    let on_edge = Vector3::new(0.5, 0.0, 0.0);
    assert_eq!(
        insert_where_it_stands(&mut mesh, 0, on_edge, &mut pending, &mut depth, None).ok(),
        Some(4)
    );
    assert_eq!(pending.len(), 2);
    assert!(pending.iter().all(|t| depth[t] == 1));

    // On the line: a corner within the snap is answered with; one past the snap but within the
    // triangle's millionth has nothing to insert; within the snap of an edge the edge is split.
    let t = 1;
    assert_eq!(
        insert_on_the_line(
            &mut mesh,
            t,
            Vector3::new(0.0, 1.0 - 1e-4, 0.0),
            1e-3,
            &mut pending,
            &mut depth
        )
        .ok(),
        Some(3)
    );
    assert_eq!(
        insert_on_the_line(
            &mut mesh,
            t,
            Vector3::new(0.0, 1.0 - 1e-7, 0.0),
            1e-9,
            &mut pending,
            &mut depth
        )
        .ok(),
        Some(NOTHING_TO_INSERT)
    );
    let before = mesh.vertex_count();
    assert_eq!(
        insert_on_the_line(
            &mut mesh,
            t,
            Vector3::new(0.0, 0.5, 0.0),
            1e-3,
            &mut pending,
            &mut depth
        )
        .ok(),
        Some(before)
    );
    assert!(!mesh.has_edge(0, 3));
}

#[test]
fn rust_only_a_sides_witness_is_read_and_inserted_at_the_snap() {
    use solidmesh::bevel::level_line::side_witness::{holds_an_outside_vertex, insert};
    use std::collections::VecDeque;
    let reading = VertexFieldReading::new(square(0), Rc::new(HalfPlane), 1e-9).expect("a snap");
    // Triangle 1 (0, 2, 3) has corner 2 outside; reading stops there.
    assert!(holds_an_outside_vertex(&reading.mesh.borrow(), &reading, &[1]).expect("read"));
    assert!(!reading.values.borrow().contains_key(&3));

    let mut pending = VecDeque::new();
    let mut depth = HashMap::new();
    let mut mesh = reading.mesh.borrow_mut();
    // A point a hair (under the snap) off corner 1 is that corner: nothing is added.
    let near = Vector3::new(1.0 - 1e-4, 0.0, 0.0);
    insert(&mut mesh, 0, near, 1e-3, &mut pending, &mut depth).expect("inserted");
    assert_eq!(mesh.vertex_count(), 4);
    // Past the snap it is a new vertex, on the edge it stands on.
    insert(&mut mesh, 0, near, 1e-5, &mut pending, &mut depth).expect("inserted");
    assert_eq!(mesh.vertex_count(), 5);
    assert!(!mesh.has_edge(0, 1));
}

#[test]
fn rust_only_a_sides_band_is_its_inside_triangles_reaching_the_chain() {
    use solidmesh::bevel::level_line::band_triangles::band_of_side;
    use std::collections::HashSet;
    let reading = VertexFieldReading::new(square(2), Rc::new(HalfPlane), 1e-9).expect("a snap");
    let roots = find(&reading.mesh.borrow(), &reading).expect("the roots");
    let result = cut(&reading, &roots).expect("the cut");
    // The chain runs up x = 0's corner 0: the band is every inside triangle, reached through
    // inside triangles from the one at 0.
    let band = band_of_side(&reading, 2, &HashSet::from([0])).expect("the band");
    let mut sorted = band.clone();
    sorted.sort();
    let mut inside = result.inside_triangles.clone();
    inside.sort();
    assert_eq!(sorted, inside);
    // Another side, or a chain no inside triangle touches, has no band.
    assert!(band_of_side(&reading, 9, &HashSet::from([0]))
        .expect("none")
        .is_empty());
    assert!(band_of_side(&reading, 2, &HashSet::from([1]))
        .expect("none")
        .is_empty());
}

#[test]
fn rust_only_the_line_walk_finds_the_lines_point_down_the_slope() {
    use solidmesh::bevel::level_line::line_walk::try_find_line_point_near;
    let reading = VertexFieldReading::new(square(0), Rc::new(HalfPlane), 1e-9).expect("a snap");
    let mesh = reading.mesh.borrow();
    // From inside the band the walk goes up +x to the line x = 0.5.
    let point = try_find_line_point_near(&mesh, &reading, 0, Vector3::new(0.3, 0.1, 0.0), 1.0)
        .expect("read")
        .expect("a point");
    assert!((point - Vector3::new(0.5, 0.1, 0.0)).length() <= 1e-9);
    // In the other triangle it walks up +x to the same line.
    let point = try_find_line_point_near(&mesh, &reading, 1, Vector3::new(0.2, 0.7, 0.0), 1.0)
        .expect("read")
        .expect("a point");
    assert!((point - Vector3::new(0.5, 0.7, 0.0)).length() <= 1e-9);
    drop(mesh);

    // A triangle with no area has no plane to walk in.
    let flat = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a needle");
    assert_eq!(
        try_find_line_point_near(&flat, &reading, 0, Vector3::new(0.3, 0.0, 0.0), 1.0).ok(),
        Some(None)
    );
}

#[test]
fn rust_only_two_chains_mitre_point_is_made_a_vertex() {
    use solidmesh::bevel::level_line::corner_tie_points::insert;
    use solidmesh::bevel::level_line::{ChainField, PickField};
    let leg = |to: Vector3| {
        Rc::new(
            ChainField::from_polyline(&[Vector3::new(0.0, 0.0, 0.0), to], false, &[1.0, 1.0])
                .expect("a chain"),
        )
    };
    let chains = vec![
        leg(Vector3::new(4.0, 0.0, 0.0)),
        leg(Vector3::new(0.0, 4.0, 0.0)),
    ];
    let pick = PickField::new(chains.clone(), &[0, 1]).expect("two members");
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.5, 0.5, 0.0),
            Vector3::new(2.0, 0.5, 0.0),
            Vector3::new(0.5, 2.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a triangle");
    let triangles = insert(&mut mesh, &pick, vec![0], 1e-9).expect("inserted");
    // The two level lines y = 1 and x = 1 meet at (1, 1): that point is now a vertex, and the
    // candidates are the triangles round it.
    assert_eq!(mesh.vertex_count(), 4);
    assert!((mesh.position(3) - Vector3::new(1.0, 1.0, 0.0)).length() <= 1e-9);
    assert_eq!(triangles.len(), 3);
    assert!(triangles.iter().all(|&t| !mesh.is_removed(t)));

    // One member has no mitre: the candidates come back as they were.
    let one = PickField::new(chains, &[0]).expect("one member");
    assert_eq!(
        insert(&mut mesh, &one, vec![2, 0], 1e-9).ok(),
        Some(vec![2, 0])
    );
}

#[test]
fn rust_only_a_sharp_joints_inside_corner_is_made_a_vertex_on_the_line() {
    use solidmesh::bevel::level_line::sharp_joint_corners::insert;
    use solidmesh::bevel::level_line::ChainField;
    // An L whose joint at (4, 0) turns a right angle: its inside corner, a setback from both
    // legs, is (3, 1).
    let chain = ChainField::from_polyline(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 4.0, 0.0),
        ],
        false,
        &[1.0, 1.0, 1.0],
    )
    .expect("a chain");
    let mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(2.0, 0.5, 0.0),
            Vector3::new(3.6, 0.5, 0.0),
            Vector3::new(2.0, 2.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("a triangle");
    // Joints sharper than 45 degrees are sharp (a plain polyline's threshold is infinite).
    let mut chain = chain;
    chain.sharp_turn_degrees = 45.0;
    let chain = Rc::new(chain);
    let field: Rc<dyn ScalarField> = chain.clone();
    let reading =
        VertexFieldReading::new(Rc::new(RefCell::new(mesh)), field.clone(), 1e-9).expect("a snap");
    let triangles =
        insert(&reading, field.as_ref(), [chain.as_ref()], vec![0], 1e-9).expect("inserted");
    assert_eq!(reading.mesh.borrow().vertex_count(), 4);
    assert!((reading.mesh.borrow().position(3) - Vector3::new(3.0, 1.0, 0.0)).length() <= 1e-9);
    assert_eq!(reading.side_of(3).ok(), Some(VertexSide::On));
    assert_eq!(triangles.len(), 3);

    // A straight chain has no sharp joint, and no end clip without a same-sense end plane: the
    // candidates come back as they were.
    let straight = ChainField::from_polyline(
        &[Vector3::new(0.0, 0.0, 0.0), Vector3::new(4.0, 0.0, 0.0)],
        false,
        &[1.0, 1.0],
    )
    .expect("a chain");
    assert_eq!(
        insert(&reading, field.as_ref(), [&straight], vec![2, 0], 1e-9).ok(),
        Some(vec![2, 0])
    );
    assert_eq!(
        solidmesh::bevel::level_line::end_clip_corners::insert(
            &reading,
            &straight,
            [&straight],
            vec![2, 0],
            1e-9
        )
        .ok(),
        Some(vec![2, 0])
    );
}
