//! Continuing a ray from a surface: the origin a continuing ray starts from must not find the
//! face it left and must not carry the ray through a wall. The measurements print the rate of
//! self-hits at the offset [`solidmesh_mesh::ray_tracer::SurfacePoint`] reports and at the old
//! fixed `Ray::SAME_SURFACE_OFFSET`, on parts from 1 mm to 1000 mm.

use std::f64::consts::PI;
use std::sync::Arc;

use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::{
    MeshSurface, MinimalTriangle, PlacedPart, PlacedParts, SurfacePoint, TraceableRef,
};
use solidmesh_mesh::Mesh;

use crate::surface_hit_support::{
    fibonacci_directions, inward_ray, sphere, trace_face, tree_of, CREASE_30,
};

/// How often a ray leaving a real hit point finds the face it left: `offset` builds the origin
/// from the hit and direction, and the ray looks at both sides of every face so the test sees
/// the plane rounding a one-sided query can hide.
fn self_hit_rate(
    tree: &TraceableRef,
    surface: &MeshSurface,
    center: Vector3,
    radius: f64,
    offset: &dyn Fn(&SurfacePoint, Vector3, Vector3) -> Vector3,
) -> (u32, u32) {
    let (mut rays, mut self_hits) = (0, 0);
    for direction in fibonacci_directions(120) {
        let mut ray = inward_ray(center, direction, radius * 3.0);
        let Some((face, hit)) = trace_face(tree, &mut ray) else {
            continue;
        };
        let point = surface.surface_point(face, hit.hit_position).unwrap();
        for outgoing in fibonacci_directions(40) {
            let origin = offset(&point, hit.hit_position, outgoing);
            let mut next =
                Ray::with_limits(origin, outgoing, 0.0, f64::INFINITY, IntersectionType::Both);
            rays += 1;
            if let Some(next_hit) = tree.get_closest_intersection(&mut next) {
                if next_hit
                    .closest_hit_as::<MinimalTriangle>()
                    .map(|t| t.face_index)
                    == Some(face)
                {
                    self_hits += 1;
                }
            }
        }
    }
    (rays, self_hits)
}

/// The sizes the self-hit tests run at: parts from 1 mm to 1000 mm, near the origin and baked far
/// from it, where the f32 vertices are coarse.
fn baked_sphere_cases() -> Vec<(f64, Vector3)> {
    let mut cases = Vec::new();
    for radius in [0.5, 5.0, 50.0, 500.0] {
        for center in [
            Vector3::ZERO,
            Vector3::new(30.0, -70.0, 15.0),
            Vector3::new(100_000.0, 100_000.0, -100_000.0),
        ] {
            cases.push((radius, center));
        }
    }
    cases
}

/// A continuing ray from a real hit point never finds the triangle it just left, at any size or
/// placement, and the old fixed offset is shown failing where it does fail.
///
/// The sweep of the reported offset's size pins the landed value of 8 `f32` steps from both sides:
/// it is the threshold of about 2 steps with a margin. Measured over these cases (about 4,800
/// rays each): 0 x gives about 50% self-hits, 0.05 x (0.4 steps) 1 to 20%, 0.125 x (1 step) a few
/// (20 of 4,800 at radius 5 around the origin, 21 at radius 0.5 around (30, -70, 15)), and
/// 0.25 x (2 steps) and above none. So 1 step must still fail somewhere and 2 steps and up must
/// not, which fails if the constant is halved to 4 or doubled to 16.
#[test]
fn continuing_rays_do_not_find_the_face_they_left() {
    let mut old_offset_failed = false;
    let (mut one_step_hits, mut two_steps_and_up_hits) = (0_u32, 0_u32);
    for (radius, center) in baked_sphere_cases() {
        let mesh = sphere(radius, center, 48, 24);
        let tree = tree_of(&mesh);
        let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();

        // Sweep the size of the offset this module reports, to measure the margin.
        let mut rows = Vec::new();
        for scale in [0.0, 0.05, 0.125, 0.25, 0.5, 1.0] {
            let (rays, hits) =
                self_hit_rate(&tree, &surface, center, radius, &|point, _hit, out| {
                    let direction = if out.dot(point.geometric_normal) >= 0.0 {
                        1.0
                    } else {
                        -1.0
                    };
                    point.position + point.ray_offset * (direction * scale)
                });
            rows.push((scale, rays, hits));
            if scale == 0.125 {
                one_step_hits += hits;
            } else if scale >= 0.25 {
                two_steps_and_up_hits += hits;
            }
        }
        let (rays, new_hits) =
            self_hit_rate(&tree, &surface, center, radius, &|point, _hit, out| {
                point.next_ray_origin(out)
            });
        let (_, old_hits) = self_hit_rate(&tree, &surface, center, radius, &|point, hit, out| {
            let direction = if out.dot(point.geometric_normal) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            hit + point.geometric_normal * (Ray::SAME_SURFACE_OFFSET * direction)
        });
        eprintln!(
            "self-hits radius {radius} center {:?}: offset-scale rows {rows:?}; \
             ray_offset {new_hits}/{rays}; SAME_SURFACE_OFFSET {old_hits}/{rays} ({:.3}%)",
            center.to_array(),
            f64::from(old_hits) * 100.0 / f64::from(rays)
        );
        assert_eq!(new_hits, 0, "radius {radius} center {center:?}");
        old_offset_failed |= old_hits > 0;
    }
    assert!(
        old_offset_failed,
        "the fixed offset should fail somewhere, or this test no longer measures anything"
    );
    assert!(
        one_step_hits > 0,
        "one step of the offset no longer shows any self-hit, so the offset is not what this \
         sweep pinned"
    );
    assert_eq!(
        two_steps_and_up_hits, 0,
        "two steps of the offset or more must leave no self-hit"
    );
}

/// A placed part: the continuation is safe through a transform, including a scale, and a part
/// placed very far away keeps working because the offset is measured in the part's own space.
#[test]
fn continuing_rays_are_safe_on_placed_parts_at_any_placement() {
    let mesh = sphere(5.0, Vector3::new(11.0, 7.0, -3.0), 48, 24);
    let tree = tree_of(&mesh);
    let surface = Arc::new(MeshSurface::new(&mesh, CREASE_30).unwrap());

    let matrices = [
        Matrix4X4::create_translation(100_000.0, -100_000.0, 50_000.0),
        Matrix4X4::create_scale_uniform(0.001) * Matrix4X4::create_translation(3.0, 4.0, 5.0),
        Matrix4X4::create_scale_uniform(1000.0) * Matrix4X4::create_rotation_z(1.2),
        Matrix4X4::create_scale(0.2, 7.0, 90.0)
            * Matrix4X4::create_rotation_x(0.4)
            * Matrix4X4::create_translation(1e9, 0.0, 0.0),
        Matrix4X4::create_scale(-3.0, 3.0, 3.0) * Matrix4X4::create_translation(-77.0, 1.0, 1e6),
    ];

    for matrix in matrices {
        let part = PlacedPart::new(tree.clone(), surface.clone(), matrix).unwrap();
        let scene = PlacedParts::new(vec![part]);
        let world_center = Vector3::new(11.0, 7.0, -3.0).transform_position(matrix);
        let world_radius = matrix
            .column0()
            .length()
            .max(matrix.column1().length())
            .max(matrix.column2().length())
            * 5.0;

        let (mut rays, mut self_hits, mut below_surface_misses) = (0_u32, 0_u32, 0_u32);
        for direction in fibonacci_directions(120) {
            let start = world_center + direction * (world_radius * 3.0);
            let mut ray = Ray::new(start, (world_center - start).get_normal());
            let Some(hit) = scene.trace(&mut ray) else {
                continue;
            };
            let point = hit.surface_point().unwrap();
            for outgoing in fibonacci_directions(40) {
                let origin = point.next_ray_origin(outgoing);
                let mut next =
                    Ray::with_limits(origin, outgoing, 0.0, f64::INFINITY, IntersectionType::Both);
                rays += 1;
                if let Some(next_hit) = scene.trace(&mut next) {
                    let face = next_hit.surface_point().unwrap().face_index;
                    if face == point.face_index {
                        self_hits += 1;
                    }
                }
                // A ray heading into the solid must still find the far wall.
                if outgoing.dot(point.geometric_normal) < -0.3 {
                    let mut through = Ray::new(origin, outgoing);
                    through.intersection_type = IntersectionType::Both;
                    if scene.trace(&mut through).is_none() {
                        below_surface_misses += 1;
                    }
                }
            }
        }
        eprintln!("placed {matrix:?}: {self_hits}/{rays} self-hits, {below_surface_misses} leaks");
        assert_eq!(self_hits, 0, "matrix {matrix:?}");
        assert_eq!(
            below_surface_misses, 0,
            "a continuing ray escaped the solid {matrix:?}"
        );
    }
}

/// The offset never carries a ray across a tight corner: at the tip of a sharp wedge, a ray from
/// the floor still hits the wall that is really there, and the wall stays in the ray's way only
/// because the floor's offset is smaller than the gap. On a 500 mm part with a corner at the
/// origin (largest L1 corner 1000 mm, so the landed offset is about 9.5e-4 mm), a real 0.05 mm
/// wall clears it by about 52 times; a gap of about 4 offsets (3.8e-3 mm) bounds the offset from
/// above at about 32 steps, 4 times what it is.
#[test]
fn a_ray_into_a_tight_concave_corner_still_hits_the_wall() {
    assert_a_ray_from_the_floor_hits_the_wall(0.05);
    assert_a_ray_from_the_floor_hits_the_wall(3.8e-3);
}

/// A floor and a wall hinged along y at the origin, opening 20 degrees, 500 mm each way, with the
/// floor `gap` below the wall at the point tested. The wall's front looks at the floor.
fn assert_a_ray_from_the_floor_hits_the_wall(gap: f64) {
    let tan_20 = (20.0 * PI / 180.0_f64).tan();
    let height = 500.0 * tan_20;
    let mesh = Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, 500.0, 0.0, 0.0, 500.0, 500.0, 0.0, 0.0, 500.0, 0.0, 500.0, 0.0, height,
            500.0, 500.0, height,
        ],
        &[0, 1, 2, 0, 2, 3, 0, 5, 4, 0, 3, 5],
    );
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();

    let x = gap / tan_20;
    let mut down = Ray::new(Vector3::new(x, 250.0, 3.0), -Vector3::UNIT_Z);
    let (face, hit) = trace_face(&tree, &mut down).expect("the floor");
    let point = surface.surface_point(face, hit.hit_position).unwrap();
    assert!(face == 0 || face == 1, "hit the floor, not face {face}");

    let toward_wall = Vector3::new(-0.2, 0.0, 1.0).get_normal();
    let origin = point.next_ray_origin(toward_wall);
    assert!(
        x * tan_20 - origin.z > 0.0,
        "the offset itself crossed a {gap} mm wall"
    );
    let mut next = Ray::new(origin, toward_wall);
    let (wall, wall_hit) = trace_face(&tree, &mut next)
        .expect("the wall is right there, the ray must not leak past it");
    assert!(wall >= 2, "the ray hit face {wall}, not the wall");
    assert!((wall_hit.hit_position.z - wall_hit.hit_position.x * tan_20).abs() < 1e-3);
}

/// The same on a 10 mm part, near the hinge, where the corners are small and the offset far
/// under a micron.
#[test]
fn a_ray_into_a_small_parts_corner_still_hits_the_wall() {
    // A floor and a wall hinged at the x axis, opening 20 degrees: the wall's front looks at
    // the floor.
    let tan_20 = (20.0 * PI / 180.0_f64).tan();
    let mesh = Mesh::from_arrays(
        &[
            0.0,
            -5.0,
            0.0,
            10.0,
            -5.0,
            0.0,
            10.0,
            5.0,
            0.0,
            0.0,
            5.0,
            0.0,
            10.0,
            -5.0,
            10.0 * tan_20,
            10.0,
            5.0,
            10.0 * tan_20,
        ],
        &[0, 1, 2, 0, 2, 3, 0, 5, 4, 0, 3, 5],
    );
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();

    // A point on the floor where the wall above it is a fixed 0.3 um away: wide of the floor's
    // offset (about 14 nm here) but far inside any offset much larger than it, so a ray that
    // starts on the wrong side of the wall here means the offset is what carried it through.
    let x = 3e-4 / tan_20;

    let mut down = Ray::new(Vector3::new(x, 0.3, 3.0), -Vector3::UNIT_Z);
    let (face, hit) = trace_face(&tree, &mut down).expect("the floor");
    let point = surface.surface_point(face, hit.hit_position).unwrap();
    assert!(face == 0 || face == 1, "hit the floor, not face {face}");

    let toward_wall = Vector3::new(-0.2, 0.0, 1.0).get_normal();
    let origin = point.next_ray_origin(toward_wall);
    let expected_gap = x * tan_20 - origin.z;
    assert!(expected_gap > 0.0, "the offset itself crossed the wall");
    let mut next = Ray::new(origin, toward_wall);
    let (wall, wall_hit) = trace_face(&tree, &mut next)
        .expect("the wall is right there, the ray must not leak past it");
    assert!(wall >= 2, "the ray hit face {wall}, not the wall");
    assert!((wall_hit.hit_position.z - wall_hit.hit_position.x * tan_20).abs() < 1e-3);
}
