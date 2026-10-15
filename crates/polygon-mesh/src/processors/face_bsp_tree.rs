//! A BSP tree over a mesh's faces for back-to-front rendering without a depth buffer: the
//! tree's nodes, building it, and walking it in visibility order. Faces are never cut; each
//! node splits on the sampled face that crosses the least.

use vector_math::Matrix4X4;

use crate::mesh::Mesh;

/// Faces within this distance of a splitting plane count as on it.
const CONSIDER_COPLANAR: f64 = 0.1;

/// One node: the face it splits on (-1 for an empty leaf) and the faces behind and in front.
/// A node with a face always has both children (an empty side is a -1 leaf).
#[derive(Debug, Clone, PartialEq)]
pub struct BspNode {
    pub back_node: Option<Box<BspNode>>,
    pub front_node: Option<Box<BspNode>>,
    pub index: i32,
}

impl Default for BspNode {
    fn default() -> Self {
        BspNode {
            back_node: None,
            front_node: None,
            index: -1,
        }
    }
}

/// A node as seen from the camera: its face and its children swapped so `back` is the far side
/// (a fresh view sharing the node's children, so the tree is never reordered in place).
#[derive(Debug, Clone, Copy)]
struct RenderNode<'a> {
    index: i32,
    back: Option<&'a BspNode>,
    front: Option<&'a BspNode>,
}

impl BspNode {
    fn render_order(
        &self,
        mesh: &Mesh,
        mesh_to_view_transform: Matrix4X4,
        inv_mesh_to_view_transform: Matrix4X4,
    ) -> RenderNode<'_> {
        let face = mesh.faces[self.index as usize];
        let face_normal_in_view_space = face
            .normal
            .transform_normal_inverse(inv_mesh_to_view_transform);
        let point_on_face_in_view_space =
            mesh.vertices[face.v0 as usize].transform(mesh_to_view_transform);
        let infront_of_face = face_normal_in_view_space.dot(point_on_face_in_view_space) < 0.0;

        if infront_of_face {
            RenderNode {
                index: self.index,
                back: self.back_node.as_deref(),
                front: self.front_node.as_deref(),
            }
        } else {
            RenderNode {
                index: self.index,
                back: self.front_node.as_deref(),
                front: self.back_node.as_deref(),
            }
        }
    }
}

/// Builds the tree, testing at most about `max_faces_to_test` faces per node; with
/// `try_to_balance_tree` a non-crossing face does not end the search early.
pub fn create(mesh: &Mesh, max_faces_to_test: usize, try_to_balance_tree: bool) -> BspNode {
    let mut root = BspNode::default();
    let faces: Vec<i32> = (0..mesh.faces.len() as i32).collect();
    create_no_splitting_fast(
        mesh,
        &mut root,
        &faces,
        max_faces_to_test,
        try_to_balance_tree,
    );
    root
}

/// [`create`] with the defaults (10 faces tested, no balancing).
pub fn create_default(mesh: &Mesh) -> BspNode {
    create(mesh, 10, false)
}

/// The face indices back to front for the given view. An empty tree (root index -1) yields
/// nothing.
pub fn get_faces_in_visibilty_order(
    mesh: &Mesh,
    root: &BspNode,
    mesh_to_view_transform: Matrix4X4,
    inv_mesh_to_view_transform: Matrix4X4,
) -> Vec<i32> {
    let mut result = Vec::new();
    if root.index == -1 {
        return result;
    }

    let mut render_order: Vec<RenderNode> =
        vec![root.render_order(mesh, mesh_to_view_transform, inv_mesh_to_view_transform)];

    while let Some(top) = render_order.last() {
        let mut last_back = top.back;
        while let Some(back) = last_back.filter(|b| b.index != -1) {
            render_order.last_mut().expect("non-empty").back = None;
            render_order.push(back.render_order(
                mesh,
                mesh_to_view_transform,
                inv_mesh_to_view_transform,
            ));
            last_back = render_order.last().expect("non-empty").back;
        }

        let node = render_order.pop().expect("non-empty");
        if node.index != -1 {
            result.push(node.index);
        }

        if let Some(front) = node.front.filter(|f| f.index != -1) {
            render_order.push(front.render_order(
                mesh,
                mesh_to_view_transform,
                inv_mesh_to_view_transform,
            ));
        }
    }

    result
}

fn signed_distance(mesh: &Mesh, plane_face: i32, point_on_plane: usize, vertex: i32) -> f64 {
    let normal = mesh.faces[plane_face as usize].normal;
    f64::from(normal.dot(mesh.vertices[vertex as usize] - mesh.vertices[point_on_plane]))
}

fn calculate_crossing_area(
    mesh: &Mesh,
    face_index: usize,
    faces: &[i32],
    smallest_crossing_arrea: f64,
) -> (f64, i32) {
    let mut negative_distance = 0.0;
    let mut positive_distance = 0.0;
    let mut negative_side_count: i32 = 0;
    let mut positive_side_count: i32 = 0;

    let check_face = faces[face_index];
    let point_on_check_face = mesh.faces[check_face as usize].v0 as usize;

    for (i, &face) in faces.iter().enumerate() {
        if i != face_index {
            let i_face = mesh.faces[face as usize];
            for vertex_index in [i_face.v0, i_face.v1, i_face.v2] {
                let distance_to_plan =
                    signed_distance(mesh, check_face, point_on_check_face, vertex_index);
                if distance_to_plan.abs() > CONSIDER_COPLANAR {
                    // Take the square of this distance to penalize far away points
                    if distance_to_plan < 0.0 {
                        negative_distance += distance_to_plan * distance_to_plan;
                    } else {
                        positive_distance += distance_to_plan * distance_to_plan;
                    }

                    if negative_distance > smallest_crossing_arrea
                        && positive_distance > smallest_crossing_arrea
                    {
                        return (f64::MAX, i32::MAX);
                    }
                }
            }

            if negative_distance > positive_distance {
                negative_side_count += 1;
            } else {
                positive_side_count += 1;
            }
        }
    }

    // return whatever side is small as our rating of badness (0 being good)
    (
        vector_math::cs_math::min_f64(negative_distance, positive_distance),
        (negative_side_count - positive_side_count).abs(),
    )
}

fn create_back_and_front_face_lists(
    mesh: &Mesh,
    face_index: usize,
    faces: &[i32],
) -> (Vec<i32>, Vec<i32>) {
    let mut back_faces = Vec::new();
    let mut front_faces = Vec::new();
    let check_face_index = faces[face_index];
    let point_on_splitting_face = mesh.faces[check_face_index as usize].v0 as usize;

    for (i, &face) in faces.iter().enumerate() {
        if i != face_index {
            let i_face = mesh.faces[face as usize];
            let back_face = [i_face.v0, i_face.v1, i_face.v2].iter().all(|&v| {
                let distance_to_plane =
                    signed_distance(mesh, check_face_index, point_on_splitting_face, v);
                // Any vertex above the plane means it's not a back face
                !(distance_to_plane.abs() > CONSIDER_COPLANAR && distance_to_plane > 0.0)
            });

            if back_face {
                back_faces.push(face);
            } else {
                front_faces.push(face);
            }
        }
    }

    (back_faces, front_faces)
}

fn create_no_splitting_fast(
    mesh: &Mesh,
    node: &mut BspNode,
    faces: &[i32],
    max_faces_to_test: usize,
    try_to_balance_tree: bool,
) {
    if faces.is_empty() {
        return;
    }

    let mut best_face_index: Option<usize> = None;
    let mut smallest_crossing_arrea = f64::MAX;
    let mut best_balance = i32::MAX;

    // find the first face that does not split anything
    let step = (faces.len() / max_faces_to_test).max(1);
    for i in (0..faces.len()).step_by(step) {
        let (crossing_arrea, balance) =
            calculate_crossing_area(mesh, i, faces, smallest_crossing_arrea);
        if crossing_arrea < smallest_crossing_arrea {
            smallest_crossing_arrea = crossing_arrea;
            best_balance = balance;
            best_face_index = Some(i);
            if crossing_arrea == 0.0 && !try_to_balance_tree {
                break;
            }
        } else if crossing_arrea == smallest_crossing_arrea && balance < best_balance {
            // the crossing area is the same but the tree balance is better
            best_balance = balance;
            best_face_index = Some(i);
        }
    }

    // When every candidate's crossing area overflows to infinity none is picked; the first
    // candidate is as good a splitter as any then.
    let best_face_index = best_face_index.unwrap_or(0);

    // the source faces are 0..n, so a face's index in them is the face index itself
    node.index = faces[best_face_index];

    let (back_faces, front_faces) = create_back_and_front_face_lists(mesh, best_face_index, faces);

    let mut back = BspNode::default();
    create_no_splitting_fast(
        mesh,
        &mut back,
        &back_faces,
        max_faces_to_test,
        try_to_balance_tree,
    );
    node.back_node = Some(Box::new(back));
    let mut front = BspNode::default();
    create_no_splitting_fast(
        mesh,
        &mut front,
        &front_faces,
        max_faces_to_test,
        try_to_balance_tree,
    );
    node.front_node = Some(Box::new(front));
}
