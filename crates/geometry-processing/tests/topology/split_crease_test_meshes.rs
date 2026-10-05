//! Fixture builders ported from `GeometryProcessingTests/Fixtures/SplitCreaseTestMeshes.cs`:
//! meshes whose straight crease is CUT INTO SEVERAL mesh edges by vertices that are not corners
//! of the shape - what a boolean, or one solid stood against another, leaves behind. The
//! fixtures for the collinear run. `CreateUnsplitCreaseBar`, `CreateCubeWithAPostPunchedThrough`
//! and `CreateBarWithAWallMissing` join with the test classes that use them.

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Matrix4X4, Vector3};

/// A 30 x 10 x 10 bar whose top front crease is cut into three mesh edges by vertices at
/// `x = 10` and `x = 20` where nothing else happens. The crease runs along `y = 0, z = 10`.
///
/// Three cells end to end, each contributing only its OUTER quads - no wall between cells:
/// two boxes sharing a face would leave every edge round it on four faces, manifold nowhere.
pub fn create_split_crease_bar() -> Mesh {
    let mut mesh = Mesh::new();
    let v = Vector3::new;

    for i in 0..3 {
        let x0 = f64::from(i) * 10.0;
        let x1 = x0 + 10.0;

        mesh.create_face(&[
            v(x0, 0.0, 10.0),
            v(x1, 0.0, 10.0),
            v(x1, 10.0, 10.0),
            v(x0, 10.0, 10.0),
        ]);
        mesh.create_face(&[
            v(x0, 10.0, 0.0),
            v(x1, 10.0, 0.0),
            v(x1, 0.0, 0.0),
            v(x0, 0.0, 0.0),
        ]);
        mesh.create_face(&[
            v(x1, 0.0, 0.0),
            v(x1, 0.0, 10.0),
            v(x0, 0.0, 10.0),
            v(x0, 0.0, 0.0),
        ]);
        mesh.create_face(&[
            v(x0, 10.0, 0.0),
            v(x0, 10.0, 10.0),
            v(x1, 10.0, 10.0),
            v(x1, 10.0, 0.0),
        ]);
    }

    mesh.create_face(&[
        v(0.0, 0.0, 10.0),
        v(0.0, 10.0, 10.0),
        v(0.0, 10.0, 0.0),
        v(0.0, 0.0, 0.0),
    ]);
    mesh.create_face(&[
        v(30.0, 0.0, 0.0),
        v(30.0, 10.0, 0.0),
        v(30.0, 10.0, 10.0),
        v(30.0, 0.0, 10.0),
    ]);

    mesh.clean_and_merge();

    mesh
}

/// A 10 mm cube with further bricks stood against its right face, each FLUSH with the cube's
/// top and front planes so the cube's top front crease carries straight on into the brick's.
/// The bricks meet the cube at the single vertex `(10, 0, 10)`, which carries six feature edges
/// or more. Each brick is YAWED about the vertical axis through that vertex by its angle, so the
/// yaw is exactly how far from collinear the continuation is.
pub fn create_flush_brick_stack(brick_yaw_degrees: &[f64]) -> Mesh {
    let mut mesh = Mesh::new();

    add_brick(
        &mut mesh,
        Vector3::ZERO,
        Vector3::new(10.0, 10.0, 10.0),
        Vector3::ZERO,
        0.0,
    );

    let pivot = Vector3::new(10.0, 0.0, 10.0);

    for &yaw_degrees in brick_yaw_degrees {
        add_brick(
            &mut mesh,
            Vector3::new(10.0, 0.0, 2.0),
            Vector3::new(20.0, 8.0, 10.0),
            pivot,
            yaw_degrees,
        );
    }

    mesh.clean_and_merge();

    mesh
}

/// The same two flush bricks put through a REAL boolean union, so the split vertex on the
/// crease and the triangles around it are the CSG kernel's own work. The bricks OVERLAP (the
/// second starts at `x = 8`); both are flush at `y = 0` and `z = 10`, so the union's top front
/// crease is one straight 20 mm edge of the SHAPE from `(0, 0, 10)` to `(20, 0, 10)`.
///
/// C# passes `ProcessingModes.Polygons` and `ProcessingResolution._64`, which the manifold
/// kernel path ignores; polygon-mesh's `do_array` does not take them yet.
pub fn create_booleaned_flush_bricks() -> Mesh {
    let mut first = platonic_solids::create_cube(10.0, 10.0, 10.0);
    first.transform(Matrix4X4::create_translation(5.0, 5.0, 5.0));

    let mut second = platonic_solids::create_cube(12.0, 8.0, 8.0);
    second.transform(Matrix4X4::create_translation(14.0, 4.0, 6.0));

    do_array(
        &[
            (&first, Matrix4X4::IDENTITY),
            (&second, Matrix4X4::IDENTITY),
        ],
        CsgModes::Union,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .expect("the union of two overlapping boxes succeeds")
}

/// One axis aligned box, optionally turned about a vertical axis through `pivot`, with every
/// face wound outward.
fn add_brick(mesh: &mut Mesh, low: Vector3, high: Vector3, pivot: Vector3, yaw_degrees: f64) {
    let yaw = math_helper::degrees_to_radians(yaw_degrees);
    let cos = yaw.cos();
    let sin = yaw.sin();

    let at = |x: f64, y: f64, z: f64| {
        let dx = x - pivot.x;
        let dy = y - pivot.y;

        Vector3::new(
            pivot.x + (dx * cos) - (dy * sin),
            pivot.y + (dx * sin) + (dy * cos),
            z,
        )
    };

    let (x0, x1) = (low.x, high.x);
    let (y0, y1) = (low.y, high.y);
    let (z0, z1) = (low.z, high.z);

    mesh.create_face(&[
        at(x1, y0, z0),
        at(x1, y1, z0),
        at(x1, y1, z1),
        at(x1, y0, z1),
    ]);
    mesh.create_face(&[
        at(x0, y0, z1),
        at(x0, y1, z1),
        at(x0, y1, z0),
        at(x0, y0, z0),
    ]);
    mesh.create_face(&[
        at(x0, y1, z0),
        at(x0, y1, z1),
        at(x1, y1, z1),
        at(x1, y1, z0),
    ]);
    mesh.create_face(&[
        at(x1, y0, z0),
        at(x1, y0, z1),
        at(x0, y0, z1),
        at(x0, y0, z0),
    ]);
    mesh.create_face(&[
        at(x0, y0, z1),
        at(x1, y0, z1),
        at(x1, y1, z1),
        at(x0, y1, z1),
    ]);
    mesh.create_face(&[
        at(x0, y1, z0),
        at(x1, y1, z0),
        at(x1, y0, z0),
        at(x0, y0, z0),
    ]);
}
