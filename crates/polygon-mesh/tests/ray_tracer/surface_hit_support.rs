//! Shared fixtures for the surface-hit tests: the meshes with known surfaces (a sphere and a
//! cylinder wound outward), the tree over them, the spread of test directions and the hit helper.

use std::f64::consts::{PI, TAU};
use std::sync::Arc;

use solidmesh_math::{Ray, Vector3, Vector3Float};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, IntersectInfo, MinimalTriangle, TraceableRef,
    VertexFunc,
};
use solidmesh_mesh::Mesh;

pub(crate) const CREASE_30: f64 = 30.0 * PI / 180.0;

pub(crate) fn tree_of(mesh: &Mesh) -> TraceableRef {
    let corners: Arc<Vec<[Vector3Float; 3]>> = Arc::new(
        mesh.faces
            .iter()
            .map(|face| {
                [
                    mesh.vertices[face.v0 as usize],
                    mesh.vertices[face.v1 as usize],
                    mesh.vertices[face.v2 as usize],
                ]
            })
            .collect(),
    );
    let vertex_func: VertexFunc =
        Arc::new(move |face, corner| corners[face as usize][corner.min(2) as usize]);
    let leaves = (0..mesh.faces.len() as i32)
        .map(|face| Arc::new(MinimalTriangle::new(vertex_func.clone(), face)) as TraceableRef)
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(leaves, BvhCreationOptions::default())
        .expect("the mesh has faces")
}

/// Faces of `mesh` swapped where needed so each one's normal points away from `inside`.
pub(crate) fn wind_outward(vertices: &[f64], faces: Vec<[i32; 3]>, inside: Vector3) -> Mesh {
    let corner = |i: i32| {
        let i = i as usize * 3;
        Vector3::new(vertices[i], vertices[i + 1], vertices[i + 2])
    };
    let mut flat = Vec::with_capacity(faces.len() * 3);
    for [a, b, c] in faces {
        let (pa, pb, pc) = (corner(a), corner(b), corner(c));
        let centroid = (pa + pb + pc) / 3.0;
        let outward = (pb - pa).cross(pc - pa).dot(centroid - inside) > 0.0;
        flat.extend_from_slice(&if outward { [a, b, c] } else { [a, c, b] });
    }
    Mesh::from_arrays(vertices, &flat)
}

/// A latitude-longitude sphere of `radius` at `center`, wound outward.
pub(crate) fn sphere(radius: f64, center: Vector3, segments: usize, rings: usize) -> Mesh {
    let mut vertices = vec![center.x, center.y, center.z + radius];
    vertices.extend_from_slice(&[center.x, center.y, center.z - radius]);
    let ring_vertex = |ring: usize, segment: usize| {
        2 + (ring - 1) as i32 * segments as i32 + (segment % segments) as i32
    };
    for ring in 1..rings {
        let polar = PI * ring as f64 / rings as f64;
        for segment in 0..segments {
            let azimuth = TAU * segment as f64 / segments as f64;
            vertices.push(center.x + radius * polar.sin() * azimuth.cos());
            vertices.push(center.y + radius * polar.sin() * azimuth.sin());
            vertices.push(center.z + radius * polar.cos());
        }
    }
    let mut faces = Vec::new();
    for segment in 0..segments {
        faces.push([0, ring_vertex(1, segment), ring_vertex(1, segment + 1)]);
        faces.push([
            1,
            ring_vertex(rings - 1, segment + 1),
            ring_vertex(rings - 1, segment),
        ]);
        for ring in 1..(rings - 1) {
            let (a, b) = (ring_vertex(ring, segment), ring_vertex(ring, segment + 1));
            let (c, d) = (
                ring_vertex(ring + 1, segment),
                ring_vertex(ring + 1, segment + 1),
            );
            faces.push([a, c, d]);
            faces.push([a, d, b]);
        }
    }
    wind_outward(&vertices, faces, center)
}

/// A closed cylinder along z, `radius` and `half_height`, the rim vertices shared by the side
/// and the caps.
pub(crate) fn cylinder(radius: f64, half_height: f64, segments: usize) -> Mesh {
    let mut vertices = vec![0.0, 0.0, half_height, 0.0, 0.0, -half_height];
    let at = |top: bool, segment: usize| 2 + (segment % segments) as i32 * 2 + i32::from(!top);
    for segment in 0..segments {
        let azimuth = TAU * segment as f64 / segments as f64;
        for z in [half_height, -half_height] {
            vertices.push(radius * azimuth.cos());
            vertices.push(radius * azimuth.sin());
            vertices.push(z);
        }
    }
    let mut faces = Vec::new();
    for segment in 0..segments {
        faces.push([0, at(true, segment), at(true, segment + 1)]);
        faces.push([1, at(false, segment + 1), at(false, segment)]);
        let (a, b) = (at(true, segment), at(true, segment + 1));
        let (c, d) = (at(false, segment), at(false, segment + 1));
        faces.push([a, c, d]);
        faces.push([a, d, b]);
    }
    wind_outward(&vertices, faces, Vector3::ZERO)
}

/// `count` points spread over the unit sphere, a repeatable spiral.
pub(crate) fn fibonacci_directions(count: usize) -> Vec<Vector3> {
    let golden = PI * (3.0 - 5.0_f64.sqrt());
    (0..count)
        .map(|i| {
            let z = 1.0 - (2.0 * i as f64 + 1.0) / count as f64;
            let radial = (1.0 - z * z).sqrt();
            let angle = golden * i as f64;
            Vector3::new(radial * angle.cos(), radial * angle.sin(), z)
        })
        .collect()
}

/// A ray from outside a body at `center` aimed at it along `direction`.
pub(crate) fn inward_ray(center: Vector3, direction: Vector3, distance: f64) -> Ray {
    let unit = direction.get_normal();
    Ray::new(center + unit * distance, -unit)
}

/// The closest hit of `ray` on `tree` and the face it landed on.
pub(crate) fn trace_face<'a>(
    tree: &'a TraceableRef,
    ray: &mut Ray,
) -> Option<(i32, IntersectInfo<'a>)> {
    let hit = tree.get_closest_intersection(ray)?;
    let face = hit.closest_hit_as::<MinimalTriangle>()?.face_index;
    Some((face, hit))
}

pub(crate) fn assert_unit(vector: Vector3, what: &str) {
    assert!(vector.is_valid(), "{what} is not finite: {vector:?}");
    assert!(
        (vector.length() - 1.0).abs() < 1e-9,
        "{what} is not unit length: {vector:?}"
    );
}

pub(crate) fn angle_between(a: Vector3, b: Vector3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}
