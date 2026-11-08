//! What a shading ray tracer reads at a hit: smooth normals with sharp creases, barycentric
//! weights, face colors and parts placed under transforms. Every hit here comes from tracing
//! real rays through a real tree; meshes are built only to give the normals a known truth. The
//! origin of a continuing ray is tested in `surface_hit_continuation_tests`.

use std::f64::consts::{PI, TAU};
use std::sync::Arc;

use solidmesh_math::{Color, Matrix4X4, Ray, Vector2, Vector2Float, Vector3, Vector3Float};
use solidmesh_mesh::ray_tracer::{
    facing_normals, MeshSurface, PlacedPart, PlacedParts, TraceableRef, Transform,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{FaceTextureData, Mesh};

use crate::surface_hit_support::{
    angle_between, assert_unit, cylinder, fibonacci_directions, inward_ray, sphere, trace_face,
    tree_of, CREASE_30,
};

/// A sphere at low detail reads smooth: the shading normal sits close to the true normal, and far
/// closer than the facet does, which is the whole reason shading normals exist.
#[test]
fn a_spheres_shading_normal_is_close_to_true_and_far_better_than_the_facet() {
    let center = Vector3::new(3.0, -2.0, 7.0);
    let radius = 10.0;
    let mesh = sphere(radius, center, 24, 12);
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();

    let (mut shading_sum, mut facet_sum, mut shading_max, mut count) = (0.0, 0.0, 0.0_f64, 0);
    for direction in fibonacci_directions(600) {
        let mut ray = inward_ray(center, direction, 30.0);
        let (face, hit) = trace_face(&tree, &mut ray).expect("the sphere is in the way");
        let point = surface.surface_point(face, hit.hit_position).unwrap();
        let truth = (point.position - center).get_normal();
        let shading_error = angle_between(point.shading_normal, truth);
        let facet_error = angle_between(point.geometric_normal, truth);
        shading_sum += shading_error;
        facet_sum += facet_error;
        shading_max = shading_max.max(shading_error);
        count += 1;
    }

    let (shading_mean, facet_mean) = (shading_sum / count as f64, facet_sum / count as f64);
    eprintln!(
        "sphere 24x12: shading mean {:.3} deg max {:.3} deg, facet mean {:.3} deg",
        shading_mean.to_degrees(),
        shading_max.to_degrees(),
        facet_mean.to_degrees()
    );
    assert!(
        shading_max < 3.0_f64.to_radians(),
        "shading max {shading_max}"
    );
    assert!(
        shading_mean * 3.0 < facet_mean,
        "shading {shading_mean} should beat facet {facet_mean} by three times"
    );
}

/// A cylinder's side is smooth around its circumference, and the rim stays sharp: the cap stays
/// flat right to the edge and the side does not bend toward the cap.
#[test]
fn a_cylinders_side_is_smooth_and_its_rim_stays_sharp() {
    let (radius, half_height) = (5.0, 4.0);
    let mesh = cylinder(radius, half_height, 16);
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();

    for i in 0..40 {
        let azimuth = TAU * (i as f64 + 0.31) / 40.0;
        let outward = Vector3::new(azimuth.cos(), azimuth.sin(), 0.0);
        // Near the top rim on the side, almost at the crease: the side must stay radial.
        let mut ray = Ray::new(outward * 30.0 + Vector3::UNIT_Z * 3.9, -outward);
        let (face, hit) = trace_face(&tree, &mut ray).expect("the side is there");
        let point = surface.surface_point(face, hit.hit_position).unwrap();
        let radial = Vector3::new(point.position.x, point.position.y, 0.0).get_normal();
        assert!(
            point.shading_normal.z.abs() < 1e-9,
            "the rim bent the side: {:?}",
            point.shading_normal
        );
        assert!(
            angle_between(point.shading_normal, radial) < 3.0_f64.to_radians(),
            "the side is not smooth: {:?} against {:?}",
            point.shading_normal,
            radial
        );

        // On the top cap, a hair inside the polygon's rim: the cap's own flat normal.
        let apothem = radius * (PI / 16.0).cos();
        let near_rim = outward * (apothem - 0.02) + Vector3::UNIT_Z * 20.0;
        let mut ray = Ray::new(near_rim, -Vector3::UNIT_Z);
        let (face, hit) = trace_face(&tree, &mut ray).expect("the cap is there");
        let point = surface.surface_point(face, hit.hit_position).unwrap();
        assert_eq!(
            point.shading_normal,
            Vector3::UNIT_Z,
            "the cap must stay flat"
        );
        assert_eq!(point.geometric_normal, Vector3::UNIT_Z);
    }
}

/// Without creases the same cylinder really does blend across the rim, so the sharp-rim test
/// above is measuring the crease rather than some accident of the mesh.
#[test]
fn a_crease_of_pi_smooths_a_cylinders_rim_too() {
    let mesh = cylinder(5.0, 4.0, 16);
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, PI).unwrap();
    let mut ray = Ray::new(Vector3::new(4.98, 0.0, 20.0), -Vector3::UNIT_Z);
    let (face, hit) = trace_face(&tree, &mut ray).expect("the cap is there");
    let point = surface.surface_point(face, hit.hit_position).unwrap();
    assert!(point.shading_normal.z < 0.99, "{:?}", point.shading_normal);
    assert!(point.shading_normal.x > 0.1, "{:?}", point.shading_normal);
}

/// The normal at `vertex` on `face` of a mesh, read through the table at that exact corner.
fn normal_at_vertex(mesh: &Mesh, face: i32, vertex: Vector3) -> Vector3 {
    let surface = MeshSurface::new(mesh, CREASE_30).unwrap();
    surface.surface_point(face, vertex).unwrap().shading_normal
}

/// Cutting a face into more triangles must not change how the surface shades: MatterCAD
/// re-tessellates surfaces in every boolean, and the same surface has to look the same after.
/// Two planes tilted 10 degrees each way meet at a spine; the left plane's two triangles at the
/// vertex are then each split at the midpoint of their outer edge, which is the same surface
/// with twice the triangles on one side.
#[test]
fn cutting_a_face_into_more_triangles_does_not_change_the_normal() {
    let tilt = 10.0_f64.to_radians();
    let (length, reach) = (20.0, 20.0);
    let drop = reach * tilt.tan();
    let spine_top = [0.0, length, 0.0];
    let spine_bottom = [0.0, -length, 0.0];
    let left = [-reach, 0.0, -drop];
    let right = [reach, 0.0, -drop];
    let origin = [0.0, 0.0, 0.0];

    let midpoint = |a: [f64; 3], b: [f64; 3]| {
        [
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        ]
    };
    let vertices = |points: &[[f64; 3]]| points.iter().flatten().copied().collect::<Vec<f64>>();

    // 0 origin, 1 top, 2 left, 3 bottom, 4 right.
    let whole = Mesh::from_arrays(
        &vertices(&[origin, spine_top, left, spine_bottom, right]),
        &[0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1],
    );
    // The left plane's triangles cut at their outer edges: 5 and 6 are those midpoints.
    let cut = Mesh::from_arrays(
        &vertices(&[
            origin,
            spine_top,
            left,
            spine_bottom,
            right,
            midpoint(spine_top, left),
            midpoint(left, spine_bottom),
        ]),
        &[0, 1, 5, 0, 5, 2, 0, 2, 6, 0, 6, 3, 0, 3, 4, 0, 4, 1],
    );

    let whole_normal = normal_at_vertex(&whole, 0, Vector3::ZERO);
    let cut_normal = normal_at_vertex(&cut, 0, Vector3::ZERO);
    assert!(
        (whole_normal - Vector3::UNIT_Z).length() < 1e-6,
        "a symmetric vertex must point straight up, got {whole_normal:?}"
    );
    assert!(
        (cut_normal - whole_normal).length() < 1e-6,
        "the split moved the normal from {whole_normal:?} to {cut_normal:?}"
    );
}

/// Faces on a non-manifold edge are still compared with each other when many share it: pages of
/// a 20-face fan on one edge, where only the first and the last lean the same way, smooth with
/// each other. A boolean artifact can leave an edge like this.
#[test]
fn faces_on_a_crowded_non_manifold_edge_still_smooth_together() {
    let spine_end = [0.0, 10.0, 0.0];
    let radius = 10.0;
    let mut vertices = vec![0.0, 0.0, 0.0];
    vertices.extend_from_slice(&spine_end);
    let mut faces = Vec::new();
    // Page 0 and page 19 are 20 degrees apart; the 18 between them lean 60 to 300 degrees away.
    for page in 0..20 {
        let angle = match page {
            0 => 0.0,
            19 => 20.0,
            middle => 60.0 + (f64::from(middle) - 1.0) * (240.0 / 17.0),
        }
        .to_radians();
        vertices.extend_from_slice(&[radius * angle.cos(), 5.0, radius * angle.sin()]);
        faces.extend_from_slice(&[0, 1, page + 2]);
    }
    let mesh = Mesh::from_arrays(&vertices, &faces);

    let normal = normal_at_vertex(&mesh, 0, Vector3::ZERO);
    let (sin, cos) = (10.0_f64.to_radians().sin(), 10.0_f64.to_radians().cos());
    let expected = Vector3::new(sin, 0.0, -cos);
    assert!(
        (normal - expected).length() < 1e-2,
        "page 0 should blend with page 19 only, got {normal:?} expected {expected:?}"
    );
    let far_page = normal_at_vertex(&mesh, 10, Vector3::ZERO);
    assert!(
        far_page.dot(expected) < 0.9,
        "a far page joined: {far_page:?}"
    );
}

/// A cube is flat on every face and across every triangle at a 30-degree crease, to the bit.
#[test]
fn a_cube_at_a_thirty_degree_crease_stays_exactly_flat() {
    let mesh = create_cube(20.0, 20.0, 20.0);
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();
    for axis_direction in [
        Vector3::UNIT_X,
        Vector3::UNIT_Y,
        Vector3::UNIT_Z,
        -Vector3::UNIT_X,
        -Vector3::UNIT_Y,
        -Vector3::UNIT_Z,
    ] {
        for offset in [
            Vector3::new(3.1, 2.3, 1.7),
            Vector3::new(-6.1, -8.3, 4.7),
            Vector3::new(9.1, 2.3, -7.7),
        ] {
            let tangential = offset - axis_direction * offset.dot(axis_direction);
            let mut ray = Ray::new(tangential + axis_direction * 40.0, -axis_direction);
            let (face, hit) = trace_face(&tree, &mut ray).expect("a face of the cube");
            let point = surface.surface_point(face, hit.hit_position).unwrap();
            assert_eq!(point.shading_normal, axis_direction);
            assert_eq!(point.geometric_normal, axis_direction);
        }
    }
}

/// Barycentric weights rebuild the hit: within float rounding on a whole sphere, and on slivers
/// where the signed areas are tiny and a naive solve scatters along the triangle.
#[test]
fn barycentric_weights_rebuild_the_hit_position_including_on_slivers() {
    let mesh = sphere(7.0, Vector3::new(1.0, 1.0, 1.0), 32, 16);
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();
    for direction in fibonacci_directions(400) {
        let mut ray = inward_ray(Vector3::new(1.0, 1.0, 1.0), direction, 25.0);
        let (face, hit) = trace_face(&tree, &mut ray).unwrap();
        let point = surface.surface_point(face, hit.hit_position).unwrap();
        let [w0, w1, w2] = point.barycentric;
        assert!(
            w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0,
            "{:?}",
            point.barycentric
        );
        assert!((w0 + w1 + w2 - 1.0).abs() < 1e-12);
        assert!((point.position - hit.hit_position).length() < 1e-4);
    }

    // A needle (one tiny edge), a cap (one corner almost on the long edge) and a thin wedge, each
    // queried at weights only rebuilding can recover exactly, with the hit a whisker off the
    // plane as it comes from an f32 distance.
    let slivers = [
        [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [5.0, 1e-5, 0.0]],
        [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [500.0, 1e-4, 0.0]],
        [[0.0, 0.0, 0.0], [0.003, 0.0, 0.0], [0.001, 8.0, 0.0]],
    ];
    for corners in slivers {
        let vertices: Vec<f64> = corners.iter().flatten().copied().collect();
        let mesh = Mesh::from_arrays(&vertices, &[0, 1, 2]);
        let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();
        let positions = corners.map(|[x, y, z]| Vector3::new(x, y, z));
        for weights in [
            [0.5, 0.25, 0.25],
            [0.1, 0.1, 0.8],
            [0.8, 0.1, 0.1],
            [0.0, 0.5, 0.5],
            [0.999, 0.0005, 0.0005],
        ] {
            let exact =
                positions[0] * weights[0] + positions[1] * weights[1] + positions[2] * weights[2];
            let hit = exact + Vector3::new(0.0, 0.0, 2e-6);
            let point = surface.surface_point(0, hit).unwrap();
            assert!(
                (point.position - exact).length() < 1e-5,
                "sliver {corners:?} at {weights:?} rebuilt {:?} from {exact:?}",
                point.position
            );
            let sum: f64 = point.barycentric.iter().sum();
            assert!((sum - 1.0).abs() < 1e-9);
            assert!(point.barycentric.iter().all(|w| *w >= 0.0 && *w <= 1.0));
        }
    }

    // A point past the triangle clamps to it rather than reaching outside.
    let mesh = Mesh::from_arrays(&[0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 4.0, 0.0], &[0, 1, 2]);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();
    let point = surface
        .surface_point(0, Vector3::new(-1.0, -1.0, 0.0))
        .unwrap();
    assert!((point.barycentric[0] - 1.0).abs() < 1e-12);
    assert_eq!(point.position, Vector3::ZERO);
}

fn assert_every_face_is_finite_and_unit(mesh: &Mesh) {
    let surface = MeshSurface::new(mesh, CREASE_30).unwrap();
    for face in 0..mesh.faces.len() as i32 {
        for position in [
            Vector3::new(0.3, 0.4, 0.1),
            Vector3::new(f64::NAN, 0.0, 0.0),
        ] {
            let point = surface.surface_point(face, position).unwrap();
            assert_unit(point.geometric_normal, "geometric normal");
            assert_unit(point.shading_normal, "shading normal");
            assert!(point.position.is_valid(), "{:?}", point.position);
            assert!(point.barycentric.iter().all(|w| w.is_finite()));
            assert!(point.ray_offset.is_valid());
        }
    }
}

/// Real meshes bring degenerate faces, soup, shared-too-many edges and NaN corners. None of those
/// panics and each gives a usable normal.
#[test]
fn messy_meshes_never_panic_and_give_finite_unit_normals() {
    // A zero-area face and a zero-length edge beside a good face.
    let degenerate = Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0,
        ],
        &[0, 1, 2, 0, 1, 3, 1, 4, 3],
    );
    assert_every_face_is_finite_and_unit(&degenerate);

    // Unwelded soup: nothing shares a vertex, so each face gets its own normal at every corner.
    let soup = Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            1.0,
        ],
        &[0, 1, 2, 3, 4, 5],
    );
    assert_every_face_is_finite_and_unit(&soup);
    let surface = MeshSurface::new(&soup, PI).unwrap();
    let flat = surface
        .surface_point(0, Vector3::new(0.25, 0.25, 0.0))
        .unwrap();
    assert_eq!(flat.shading_normal, flat.geometric_normal);
    assert_eq!(flat.geometric_normal, Vector3::UNIT_Z);

    // Three faces on one edge, wound every which way, with a wall meeting at a shared rim.
    let non_manifold = Mesh::from_arrays(
        &[
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.5, -1.0, 0.0, 0.5, 0.0, 1.0,
        ],
        &[0, 1, 2, 1, 0, 3, 0, 1, 4],
    );
    assert_every_face_is_finite_and_unit(&non_manifold);

    // NaN corners: a face that names one has no surface.
    let mut with_nan = Mesh::from_arrays(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0],
        &[0, 1, 2, 1, 3, 2],
    );
    with_nan.vertices[3] = Vector3Float::new_f32(f32::NAN, 0.5, 0.0);
    assert_every_face_is_finite_and_unit(&with_nan);

    let empty = MeshSurface::new(&Mesh::new(), CREASE_30).unwrap();
    assert_eq!(empty.face_count(), 0);
    assert!(empty.surface_point(0, Vector3::ZERO).is_err());
}

/// A bad face index is an answer, not a crash.
#[test]
fn a_face_index_out_of_range_is_an_error() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();
    let last = mesh.faces.len() as i32;
    assert!(surface.surface_point(last, Vector3::ZERO).is_err());
    assert!(surface.surface_point(-1, Vector3::ZERO).is_err());
    assert!(surface.surface_point(i32::MIN, Vector3::ZERO).is_err());
    assert!(surface.surface_point(last - 1, Vector3::ZERO).is_ok());
}

#[test]
fn a_crease_angle_that_is_not_an_angle_is_refused() {
    let mesh = create_cube(10.0, 10.0, 10.0);
    assert!(MeshSurface::new(&mesh, f64::NAN).is_err());
    assert!(MeshSurface::new(&mesh, -0.1).is_err());
    assert!(MeshSurface::new(&mesh, 4.0).is_err());
    assert!(MeshSurface::new(&mesh, 0.0).is_ok());
    assert!(MeshSurface::new(&mesh, PI).is_ok());
}

/// A mesh with no colors reads white, a mesh with some reads each face's own, a too-short list
/// reads the grey placeholder, and textures come through as interpolated UVs.
#[test]
fn face_colors_and_texture_coordinates_come_through() {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let hit = Vector3::new(0.0, 0.0, 5.0);

    let plain = MeshSurface::new(&mesh, CREASE_30).unwrap();
    let point = plain.surface_point(0, hit).unwrap();
    assert_eq!(point.base_color, Mesh::UNCOLORED_FACE);
    assert_eq!(point.texture_coordinate, None);

    let colors: Vec<Color> = (0..mesh.faces.len())
        .map(|face| Color::rgb(face as u8, 20, 30))
        .collect();
    mesh.face_colors = Some(colors.clone());
    let painted = MeshSurface::new(&mesh, CREASE_30).unwrap();
    for (face, color) in colors.iter().enumerate() {
        let point = painted.surface_point(face as i32, hit).unwrap();
        assert_eq!(point.base_color, *color);
    }

    mesh.face_colors = Some(vec![Color::RED; 2]);
    let short = MeshSurface::new(&mesh, CREASE_30).unwrap();
    assert_eq!(short.surface_point(0, hit).unwrap().base_color, Color::RED);
    assert_eq!(
        short.surface_point(11, hit).unwrap().base_color,
        Mesh::UNKNOWN_FACE_COLOR
    );

    // UVs interpolate with the weights: halfway along a corner edge gets halfway between UVs.
    let triangle_vertices = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0, 0.0];
    let mut textured = Mesh::from_arrays(&triangle_vertices, &[0, 1, 2]);
    textured.face_textures.insert(
        0,
        FaceTextureData::new(
            None,
            Vector2Float::new(0.0, 0.0),
            Vector2Float::new(1.0, 0.0),
            Vector2Float::new(0.0, 0.5),
        ),
    );
    let surface = MeshSurface::new(&textured, CREASE_30).unwrap();
    let point = surface
        .surface_point(0, Vector3::new(5.0, 2.5, 0.0))
        .unwrap();
    let uv = point.texture_coordinate.expect("the face has a texture");
    assert!((uv - Vector2::new(0.5, 0.125)).length() < 1e-9, "{uv:?}");
}

/// A world-space placed sphere, for checking normals against the analytic ellipsoid.
fn analytic_ellipsoid_normal(
    world: Vector3,
    center: Vector3,
    scale: Vector3,
    rotation: Matrix4X4,
) -> Vector3 {
    let unrotation = rotation.inverted();
    let local = (world - center).transform_vector(unrotation);
    // The gradient of (x/a)^2 + (y/b)^2 + (z/c)^2, the textbook ellipsoid normal.
    let gradient = Vector3::new(
        local.x / (scale.x * scale.x),
        local.y / (scale.y * scale.y),
        local.z / (scale.z * scale.z),
    );
    gradient.get_normal().transform_vector(rotation)
}

/// Placed parts report the right normal at every hit: translated, rotated, stretched, mirrored
/// and through a nested chain of transforms. Compares against the textbook ellipsoid normal, so
/// a wrong transform of the normal shows up rather than agreeing with itself.
#[test]
fn placed_parts_have_correct_normals_under_every_kind_of_placement() {
    let mesh = sphere(1.0, Vector3::ZERO, 96, 48);
    let tree = tree_of(&mesh);
    let surface = Arc::new(MeshSurface::new(&mesh, CREASE_30).unwrap());

    let rotation = Matrix4X4::create_rotation_x(0.7)
        * Matrix4X4::create_rotation_y(-1.1)
        * Matrix4X4::create_rotation_z(2.3);
    let translation = Vector3::new(40.0, -25.0, 60.0);
    let placements: [(&str, Vector3, Matrix4X4); 5] = [
        (
            "translation",
            Vector3::new(1.0, 1.0, 1.0),
            Matrix4X4::create_translation_vector(translation),
        ),
        (
            "rotation",
            Vector3::new(1.0, 1.0, 1.0),
            rotation * Matrix4X4::create_translation_vector(translation),
        ),
        (
            "non-uniform scale",
            Vector3::new(4.0, 1.0, 0.25),
            Matrix4X4::create_scale(4.0, 1.0, 0.25)
                * rotation
                * Matrix4X4::create_translation_vector(translation),
        ),
        (
            "mirror",
            Vector3::new(1.0, 1.0, 1.0),
            Matrix4X4::create_scale(-1.0, 1.0, 1.0)
                * rotation
                * Matrix4X4::create_translation_vector(translation),
        ),
        (
            "mirror with stretch",
            Vector3::new(-2.0, 1.0, 3.0),
            Matrix4X4::create_scale(-2.0, 1.0, 3.0)
                * rotation
                * Matrix4X4::create_translation_vector(translation),
        ),
    ];

    for (name, scale, matrix) in placements {
        // The part as a chain of nested nodes: the mesh's rotation inside its scale, the same
        // matrix as a single node would get.
        let flat = PlacedParts::new(vec![
            PlacedPart::new(tree.clone(), surface.clone(), matrix).unwrap()
        ]);
        // Three nodes with nothing commuting, so composing them in the wrong order shows up.
        let outer =
            Matrix4X4::create_rotation_z(0.4) * Matrix4X4::create_translation(3.0, -2.0, 9.0);
        let middle =
            Matrix4X4::create_rotation_x(-0.9) * Matrix4X4::create_translation(1.0, 5.0, 0.0);
        let inner = matrix * outer.inverted() * middle.inverted();
        let nested = Transform::with_matrix(
            Arc::new(Transform::with_matrix(
                Arc::new(Transform::with_matrix(tree.clone(), inner)) as TraceableRef,
                middle,
            )) as TraceableRef,
            outer,
        );
        let nested = PlacedParts::new(vec![
            PlacedPart::from_transform(&nested, surface.clone()).unwrap()
        ]);

        let (mut shading_max, mut flat_mean) = (0.0_f64, 0.0_f64);
        let mut count = 0;
        for direction in fibonacci_directions(400) {
            // A ray aimed at the part's centre.
            let start = translation + direction * 40.0;
            let mut ray = Ray::new(start, (translation - start).get_normal());
            let hit = flat.trace(&mut ray).expect("the part is in the way");
            let point = hit.surface_point().unwrap();
            let expected = if name == "translation" {
                (point.position - translation).get_normal()
            } else {
                analytic_ellipsoid_normal(point.position, translation, scale, rotation)
            };

            assert!(
                point.shading_normal.dot(point.position - translation) > 0.0,
                "{name}: the shading normal does not point out of the solid"
            );
            assert!(
                point.geometric_normal.dot(point.position - translation) > 0.0,
                "{name}: the geometric normal does not point out of the solid"
            );
            shading_max = shading_max.max(angle_between(point.shading_normal, expected));
            flat_mean += angle_between(point.geometric_normal, expected);
            count += 1;
            assert_unit(point.shading_normal, name);
            assert_unit(point.geometric_normal, name);

            let mut same = Ray::new(start, (translation - start).get_normal());
            let hit = nested
                .trace(&mut same)
                .expect("the nested part is there too");
            let point_nested = hit.surface_point().unwrap();
            assert!(
                (point_nested.shading_normal - point.shading_normal).length() < 1e-9,
                "{name}: a nested chain gave a different normal"
            );
            assert!((point_nested.position - point.position).length() < 1e-6);
            assert_eq!(point_nested.face_index, point.face_index);
        }
        let flat_mean = flat_mean / count as f64;
        eprintln!(
            "{name}: shading max {:.3} deg, flat mean {:.3} deg",
            shading_max.to_degrees(),
            flat_mean.to_degrees()
        );
        assert!(
            shading_max < 1.0_f64.to_radians(),
            "{name}: shading normal is {shading_max} rad off the ellipsoid's"
        );
    }
}

/// Many parts share one tree and each reports its own surface: its place in the set, its own
/// position and its own colors from its own surface table.
#[test]
fn many_parts_sharing_a_tree_each_report_their_own_surface() {
    let mesh = sphere(1.0, Vector3::ZERO, 32, 16);
    let tree = tree_of(&mesh);
    let plain = Arc::new(MeshSurface::new(&mesh, CREASE_30).unwrap());
    let mut red = mesh.clone();
    red.face_colors = Some(vec![Color::RED; mesh.faces.len()]);
    let red = Arc::new(MeshSurface::new(&red, CREASE_30).unwrap());

    let centers: Vec<Vector3> = (0..64)
        .map(|i| Vector3::new((i % 8) as f64 * 5.0, (i / 8) as f64 * 5.0, 0.0))
        .collect();
    let parts = centers
        .iter()
        .enumerate()
        .map(|(i, &center)| {
            let surface = if i % 2 == 0 {
                plain.clone()
            } else {
                red.clone()
            };
            PlacedPart::new(
                tree.clone(),
                surface,
                Matrix4X4::create_translation_vector(center),
            )
            .unwrap()
        })
        .collect();
    let scene = PlacedParts::new(parts);

    for (i, &center) in centers.iter().enumerate() {
        let start = center + Vector3::UNIT_Z * 20.0;
        let mut ray = Ray::new(start, -Vector3::UNIT_Z);
        let hit = scene.trace(&mut ray).expect("the part under the ray");
        assert_eq!(hit.part.index(), i);
        let point = hit.surface_point().unwrap();
        assert!((point.position - (center + Vector3::UNIT_Z)).length() < 0.01);
        assert!(point.shading_normal.dot(Vector3::UNIT_Z) > 0.9);
        let expected = if i % 2 == 0 { Color::WHITE } else { Color::RED };
        assert_eq!(point.base_color, expected);
    }

    // A ray along a row finds the nearest part, and the window narrows as a tree's does.
    let mut ray = Ray::new(Vector3::new(-20.0, 0.0, 0.0), Vector3::UNIT_X);
    let hit = scene.trace(&mut ray).expect("the row is there");
    assert_eq!(hit.part.index(), 0);
    assert_eq!(ray.max_distance_to_consider, hit.info.distance_to_hit);

    // A miss and an empty set give no hit.
    assert!(scene
        .trace(&mut Ray::new(
            Vector3::new(0.0, 0.0, 500.0),
            Vector3::UNIT_Z
        ))
        .is_none());
    let empty = PlacedParts::new(Vec::new());
    assert!(empty
        .trace(&mut Ray::new(Vector3::ZERO, Vector3::UNIT_Z))
        .is_none());
}

#[test]
fn a_part_whose_matrix_cannot_be_inverted_is_refused() {
    let mesh = create_cube(2.0, 2.0, 2.0);
    let tree = tree_of(&mesh);
    let surface = Arc::new(MeshSurface::new(&mesh, CREASE_30).unwrap());
    let flat = Matrix4X4::create_scale(1.0, 1.0, 0.0);
    assert!(PlacedPart::new(tree.clone(), surface.clone(), flat).is_err());
    let mut broken = Matrix4X4::IDENTITY;
    broken.row3.x = f64::NAN;
    assert!(PlacedPart::new(tree, surface, broken).is_err());
}

/// The normals a shader reads always face the ray: a back-face hit flips both, and an
/// interpolated normal that has bent past the surface or away from the ray gives way to the
/// geometric one.
#[test]
fn facing_normals_turn_both_normals_toward_the_ray() {
    let geometric = Vector3::UNIT_Z;
    let shading = Vector3::new(0.3, 0.0, 0.95).get_normal();

    let from_above = facing_normals(geometric, shading, -Vector3::UNIT_Z);
    assert!(!from_above.back_face);
    assert_eq!(from_above.geometric, geometric);
    assert_eq!(from_above.shading, shading);

    let from_below = facing_normals(geometric, shading, Vector3::UNIT_Z);
    assert!(from_below.back_face);
    assert_eq!(from_below.geometric, -geometric);
    assert_eq!(from_below.shading, -shading);

    // A shading normal bent so far the viewer is behind it: the geometric normal stands in.
    let bent = Vector3::new(0.95, 0.0, 0.1).get_normal();
    let grazing = Vector3::new(0.8, 0.0, -0.6).get_normal();
    let facing = facing_normals(geometric, bent, grazing);
    assert_eq!(facing.shading, geometric);
    assert!(facing.shading.dot(-grazing) > 0.0);

    // A shading normal on the wrong side of the surface altogether: the geometric one again.
    let flipped = facing_normals(geometric, -shading, -Vector3::UNIT_Z);
    assert_eq!(flipped.shading, geometric);

    // On a real sphere, every hit's facing shading normal agrees with the ray.
    let mesh = sphere(5.0, Vector3::ZERO, 24, 12);
    let tree = tree_of(&mesh);
    let surface = MeshSurface::new(&mesh, CREASE_30).unwrap();
    for direction in fibonacci_directions(300) {
        let mut ray = inward_ray(Vector3::ZERO, direction, 20.0);
        let incoming = ray.direction_normal;
        let (face, hit) = trace_face(&tree, &mut ray).unwrap();
        let point = surface.surface_point(face, hit.hit_position).unwrap();
        let facing = point.facing(incoming);
        assert!(facing.geometric.dot(-incoming) >= 0.0);
        assert!(facing.shading.dot(-incoming) > 0.0);
        assert!(facing.shading.dot(facing.geometric) > 0.0);
    }
}

/// The tables and the placed parts go to a worker and back: plain data that a tracer reads from
/// any thread.
#[test]
fn surface_tables_and_placed_parts_can_be_shared_across_threads() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<MeshSurface>();
    assert_send_sync::<PlacedPart>();
    assert_send_sync::<PlacedParts>();
}
