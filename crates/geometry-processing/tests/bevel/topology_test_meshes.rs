//! Fixture builders ported from `GeometryProcessingTests/Fixtures/TopologyTestMeshes.cs`, added
//! one builder at a time as the test classes that use them land. So far:
//! `CreateIsoscelesPrism` (`BevelArcChordingTests`) and `CreateArcPrism` (`BevelSweepBackDepthTests`).

use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector2, Vector3};

/// A triangular prism whose apex at the origin has the given interior angle, the two long sides
/// running `length` along +x, standing `height` tall centred on z = 0. Welded, so the fixture is
/// about acute angles rather than the repair path.
pub fn create_isosceles_prism(apex_angle_degrees: f64, length: f64, height: f64) -> Mesh {
    let spread = length * (math_helper::degrees_to_radians(apex_angle_degrees) / 2.0).tan();

    // clockwise in xy, which is what makes the side face winding below come out facing outward
    let outline = [
        Vector2::new(0.0, 0.0),
        Vector2::new(length, spread),
        Vector2::new(length, -spread),
    ];

    let bottom = -height / 2.0;
    let top = height / 2.0;

    let mut mesh = Mesh::new();
    let at = Vector3::from_vector2;

    mesh.create_face(&[
        at(outline[0], bottom),
        at(outline[1], bottom),
        at(outline[2], bottom),
    ]);

    mesh.create_face(&[
        at(outline[0], top),
        at(outline[2], top),
        at(outline[1], top),
    ]);

    for i in 0..outline.len() {
        let from = outline[i];
        let to = outline[(i + 1) % outline.len()];

        mesh.create_face(&[at(from, bottom), at(from, top), at(to, top), at(to, bottom)]);
    }

    // CreateFace adds fresh vertices every call, so what is built above is triangle soup.
    // Welding it here keeps this fixture about acute angles - an unwelded one would go down the
    // repair path instead.
    mesh.clean_and_merge();

    mesh
}

/// A half disc of `radius` (its arc chorded into `segments`, closed by its diameter on the x axis),
/// extruded `height` along z.
pub fn create_arc_prism(radius: f64, height: f64, segments: i32) -> Mesh {
    let mut path = agg_path::vertex_storage::VertexStorage::new();
    path.move_to(radius, 0.0);
    for i in 1..=segments {
        let angle = std::f64::consts::PI * f64::from(i) / f64::from(segments);
        path.line_to(radius * angle.cos(), radius * angle.sin());
    }

    path.close_polygon();

    polygon_mesh::processors::vertex_source_to_mesh::extrude(&path, height)
        .expect("a half disc extrudes")
}

/// C# `TopologyTestMeshes.CreateGridSolid`: the solid cells of an axis-aligned grid, each exposed
/// cell face one quad.
pub fn create_grid_solid(
    xs: &[f64],
    ys: &[f64],
    zs: &[f64],
    is_solid: impl Fn(i32, i32, i32) -> bool,
) -> Mesh {
    let mut mesh = Mesh::new();
    let v = Vector3::new;

    for i in 0..xs.len() as i32 - 1 {
        for j in 0..ys.len() as i32 - 1 {
            for k in 0..zs.len() as i32 - 1 {
                if !is_solid(i, j, k) {
                    continue;
                }

                let (x0, x1) = (xs[i as usize], xs[i as usize + 1]);
                let (y0, y1) = (ys[j as usize], ys[j as usize + 1]);
                let (z0, z1) = (zs[k as usize], zs[k as usize + 1]);

                if !is_solid(i + 1, j, k) {
                    mesh.create_face(&[v(x1, y0, z0), v(x1, y1, z0), v(x1, y1, z1), v(x1, y0, z1)]);
                }
                if !is_solid(i - 1, j, k) {
                    mesh.create_face(&[v(x0, y0, z1), v(x0, y1, z1), v(x0, y1, z0), v(x0, y0, z0)]);
                }
                if !is_solid(i, j + 1, k) {
                    mesh.create_face(&[v(x0, y1, z0), v(x0, y1, z1), v(x1, y1, z1), v(x1, y1, z0)]);
                }
                if !is_solid(i, j - 1, k) {
                    mesh.create_face(&[v(x1, y0, z0), v(x1, y0, z1), v(x0, y0, z1), v(x0, y0, z0)]);
                }
                if !is_solid(i, j, k + 1) {
                    mesh.create_face(&[v(x0, y0, z1), v(x1, y0, z1), v(x1, y1, z1), v(x0, y1, z1)]);
                }
                if !is_solid(i, j, k - 1) {
                    mesh.create_face(&[v(x0, y1, z0), v(x1, y1, z0), v(x1, y0, z0), v(x0, y0, z0)]);
                }
            }
        }
    }

    mesh.clean_and_merge();

    mesh
}

/// C# `SurfaceTestMeshes.CreateSquareBossOnPlate`: a 20 x 20 x 4 plate with an 8 x 8 boss standing
/// 8 tall on its middle.
pub fn create_square_boss_on_plate() -> Mesh {
    create_grid_solid(
        &[0.0, 6.0, 14.0, 20.0],
        &[0.0, 6.0, 14.0, 20.0],
        &[0.0, 4.0, 12.0],
        |i, j, k| {
            (0..3).contains(&i)
                && (0..3).contains(&j)
                && (0..2).contains(&k)
                && (k == 0 || (i == 1 && j == 1))
        },
    )
}

/// C# `TopologyTestMeshes.CreateCornerPocketBlock`: a `size` cube with a `pocket` cube bitten out
/// of its far corner.
pub fn create_corner_pocket_block(size: f64, pocket: f64) -> Mesh {
    let split = size - pocket;
    let grid = [0.0, split, size];
    create_grid_solid(&grid, &grid, &grid, |i, j, k| {
        (0..2).contains(&i)
            && (0..2).contains(&j)
            && (0..2).contains(&k)
            && !(i == 1 && j == 1 && k == 1)
    })
}

/// C# `SurfaceTestMeshes.CreateRibOnPlate`: a 20 x 20 x 5 plate with a rib 4 wide standing 10 on
/// it, its walls at x = 8 and 12 and its free end at y = 5.
pub fn create_rib_on_plate() -> Mesh {
    create_grid_solid(
        &[0.0, 8.0, 12.0, 20.0],
        &[0.0, 5.0, 20.0],
        &[0.0, 5.0, 15.0],
        |i, j, k| {
            (0..3).contains(&i)
                && (0..2).contains(&j)
                && (0..2).contains(&k)
                && (k == 0 || (i == 1 && j == 1))
        },
    )
}
