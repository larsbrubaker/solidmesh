//! Fixture builders ported from `GeometryProcessingTests/Fixtures/TopologyTestMeshes.cs`, added
//! one builder at a time as the test classes that use them land. So far:
//! `CreateIsoscelesPrism` (`BevelArcChordingTests`), `CreateArcPrism` (`BevelSweepBackDepthTests`)
//! and `CreateFrustum` (`TaperedBarrelTestMeshes`), among others below.

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

/// C# `TopologyTestMeshes.CreateCylinder`: a `sides`-sided cylinder of `radius` and `height` about
/// z, centred on the origin.
pub fn create_cylinder(radius: f64, height: f64, sides: i32) -> Mesh {
    let mut path = agg_path::vertex_storage::VertexStorage::new();
    path.move_to(0.0, -height / 2.0);
    path.line_to(radius, -height / 2.0);
    path.line_to(radius, height / 2.0);
    path.line_to(0.0, height / 2.0);
    polygon_mesh::processors::vertex_source_to_mesh::revolve(
        &path,
        sides,
        0.0,
        std::f64::consts::TAU,
        true,
    )
    .expect("the cylinder revolves")
}

/// C# `TopologyTestMeshes.CreateBoreBlockWithChordWall`: a 40 x 40 x 20 block with a bore of
/// radius 8 through it and a flat-walled lobe cut across the bore's side.
pub fn create_bore_block_with_chord_wall(sides: i32) -> Mesh {
    use polygon_mesh::csg::boolean_processing::do_array;
    use polygon_mesh::csg::CsgModes;
    use vector_math::Matrix4X4;

    let block = polygon_mesh::solids::platonic_solids::create_cube(40.0, 40.0, 20.0);
    let bore = create_cylinder(8.0, 30.0, sides);

    let mut lobe = agg_path::vertex_storage::VertexStorage::new();
    lobe.move_to(0.0, -6.0);
    lobe.line_to(12.0, -6.0);
    lobe.line_to(12.0, 6.0);
    lobe.line_to(0.0, 6.0);
    lobe.close_polygon();
    let mut lobe_mesh = polygon_mesh::processors::vertex_source_to_mesh::extrude(&lobe, 30.0)
        .expect("the lobe extrudes");
    lobe_mesh.translate(Vector3::new(0.0, 0.0, -15.0));

    do_array(
        &[
            (&block, Matrix4X4::IDENTITY),
            (&bore, Matrix4X4::IDENTITY),
            (&lobe_mesh, Matrix4X4::IDENTITY),
        ],
        CsgModes::Subtract,
        &polygon_mesh::CancelToken::new(),
        &polygon_mesh::csg::BooleanOptions::default(),
    )
    .expect("the bore and lobe cut the block")
}

/// C# `TopologyTestMeshes.CreateShallowRidgeBlock`: a 20 x 20 block whose top is two triangles
/// folded `step_degrees` along a diagonal ridge, one of them flat.
pub fn create_shallow_ridge_block(step_degrees: f64) -> Mesh {
    let half = 10.0;
    let top = 5.0;
    let bottom = -5.0;
    let far = top - (2f64.sqrt() * half * math_helper::degrees_to_radians(step_degrees).tan());

    let flat_far = Vector3::new(-half, half, top);
    let on_ridge = Vector3::new(-half, -half, top);
    let corner = Vector3::new(half, half, top);
    let dipped = Vector3::new(half, -half, far);
    let b = |x: f64, y: f64| Vector3::new(x, y, bottom);

    let mut mesh = Mesh::new();
    mesh.create_face(&[flat_far, on_ridge, corner]);
    mesh.create_face(&[corner, on_ridge, dipped]);
    mesh.create_face(&[
        b(-half, half),
        b(half, half),
        b(half, -half),
        b(-half, -half),
    ]);
    mesh.create_face(&[b(-half, half), flat_far, corner, b(half, half)]);
    mesh.create_face(&[b(half, half), corner, dipped, b(half, -half)]);
    mesh.create_face(&[flat_far, b(-half, half), b(-half, -half), on_ridge]);
    mesh.create_face(&[on_ridge, b(-half, -half), b(half, -half), dipped]);
    mesh.clean_and_merge();
    mesh
}

/// C# `TopologyTestMeshes.CreatePyramidPocketPrism`: a `sides`-sided prism of circumradius
/// `radius` and `height` whose top is a pyramid pocket sunk to `apex`.
pub fn create_pyramid_pocket_prism(sides: i32, radius: f64, height: f64, apex: Vector3) -> Mesh {
    let rim: Vec<Vector3> = (0..sides)
        .map(|i| {
            let angle = 2.0 * std::f64::consts::PI * f64::from(i) / f64::from(sides);
            Vector3::new(radius * angle.cos(), radius * angle.sin(), height)
        })
        .collect();
    let floor = |p: Vector3| Vector3::new(p.x, p.y, 0.0);

    let mut mesh = Mesh::new();
    for i in 1..(sides - 1) as usize {
        mesh.create_face(&[floor(rim[0]), floor(rim[i + 1]), floor(rim[i])]);
    }

    for i in 0..sides as usize {
        let next = (i + 1) % sides as usize;
        mesh.create_face(&[floor(rim[i]), floor(rim[next]), rim[next], rim[i]]);
        mesh.create_face(&[rim[i], rim[next], apex]);
    }

    mesh.clean_and_merge();
    mesh
}

/// C# `TopologyTestMeshes.CreateFrustum`: a `sides`-sided frustum about z, `bottom_radius` at
/// z = -height / 2 tapering to `top_radius` at +height / 2, closed at both ends.
pub fn create_frustum(bottom_radius: f64, top_radius: f64, height: f64, sides: i32) -> Mesh {
    let mut path = agg_path::vertex_storage::VertexStorage::new();
    path.move_to(0.0, -height / 2.0);
    path.line_to(bottom_radius, -height / 2.0);
    path.line_to(top_radius, height / 2.0);
    path.line_to(0.0, height / 2.0);
    polygon_mesh::processors::vertex_source_to_mesh::revolve(
        &path,
        sides,
        0.0,
        std::f64::consts::TAU,
        true,
    )
    .expect("the frustum revolves")
}

/// C# `LPrismTestMeshes.CreateLShapedGridPrism`: an L of three 10 mm cells, 10 tall - one concave
/// edge up its inside corner, meeting the top's convex rim at (10, 10, 10).
pub fn create_l_shaped_grid_prism() -> Mesh {
    let across = [0.0, 10.0, 20.0];
    create_grid_solid(&across, &across, &[0.0, 10.0], |i, j, k| {
        (0..2).contains(&i) && (0..2).contains(&j) && k == 0 && !(i == 1 && j == 1)
    })
}

/// C# `LPrismTestMeshes.CreateLShapedPrism`: an L cross section extruded 10 in z - five square
/// corners and one reflex corner at (10, 10), which is the one concave edge of the solid.
pub fn create_l_shaped_prism() -> Mesh {
    let mut path = agg_path::vertex_storage::VertexStorage::new();
    path.move_to(0.0, 0.0);
    path.line_to(20.0, 0.0);
    path.line_to(20.0, 10.0);
    path.line_to(10.0, 10.0);
    path.line_to(10.0, 20.0);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    polygon_mesh::processors::vertex_source_to_mesh::extrude(&path, 10.0)
        .expect("the L outline extrudes")
}

/// C# `LPrismTestMeshes.CreateLPrismWithFadingInnerCorner`: the L prism whose inner corner FADES -
/// sharp for the bottom half, then opened out into a fillet that grows from nothing at the middle
/// to `top_radius` at the top face. The blend is the exact rolling-ball surface, which for a
/// linearly growing radius is a CONE: each of the `blend_facets` sectors is one planar triangle
/// standing on the apex at `(10, 10, height / 2)`, so the only angles in the fade are the arc's
/// own facet steps (`90 / blend_facets` between sectors and half that where the first leaves the
/// wall) and at any flat tolerance above those the apex has exactly ONE feature edge on it, the
/// 90 degree crease below.
pub fn create_l_prism_with_fading_inner_corner(
    top_radius: f64,
    blend_facets: i32,
    height: f64,
) -> Mesh {
    // The L's outline, counter clockwise, with the reflex corner at (10, 10)
    let outline = [
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 10.0),
        Vector2::new(10.0, 10.0),
        Vector2::new(10.0, 20.0),
        Vector2::new(0.0, 20.0),
    ];

    let corner = Vector2::new(10.0, 10.0);
    let apex = Vector3::from_vector2(corner, height / 2.0);

    // The rolling ball's centre at the top face, and the arc it leaves, from the tangent point on
    // the y = 10 wall round to the tangent point on the x = 10 wall
    let centre = corner + Vector2::new(top_radius, top_radius);
    let arc: Vec<Vector3> = (0..=blend_facets)
        .map(|i| {
            let angle =
                math_helper::degrees_to_radians(270.0 - (90.0 * i as f64 / blend_facets as f64));

            Vector3::from_vector2(
                centre + (Vector2::new(angle.cos(), angle.sin()) * top_radius),
                height,
            )
        })
        .collect();

    let mut mesh = Mesh::new();
    let at = |point: Vector2, z: f64| Vector3::from_vector2(point, z);

    // The top face: the L with the corner filled in to the arc, walked from a vertex every other
    // one is visible from, because CreateFace fans from the first point it is given
    let mut top = vec![
        at(outline[0], height),
        at(outline[1], height),
        at(outline[2], height),
    ];
    top.extend_from_slice(&arc);
    top.push(at(outline[4], height));
    top.push(at(outline[5], height));
    mesh.create_face(&top);

    // The bottom face, reversed and started at the reflex corner - the one vertex the whole L is
    // visible from
    mesh.create_face(&[
        at(outline[3], 0.0),
        at(outline[2], 0.0),
        at(outline[1], 0.0),
        at(outline[0], 0.0),
        at(outline[5], 0.0),
        at(outline[4], 0.0),
    ]);

    // The four walls that never touch the corner
    for i in [0, 1, 4, 5] {
        let from = outline[i];
        let to = outline[(i + 1) % outline.len()];

        mesh.create_face(&[at(from, 0.0), at(to, 0.0), at(to, height), at(from, height)]);
    }

    // and the two that do, each a pentagon that gives up its top corner to the blend
    let last = blend_facets as usize;
    mesh.create_face(&[
        at(outline[2], 0.0),
        at(outline[3], 0.0),
        apex,
        arc[0],
        at(outline[2], height),
    ]);
    mesh.create_face(&[
        at(outline[3], 0.0),
        at(outline[4], 0.0),
        at(outline[4], height),
        arc[last],
        apex,
    ]);

    // The blend itself: one planar sector per arc chord, all standing on the apex
    for i in 0..last {
        mesh.create_face(&[apex, arc[i + 1], arc[i]]);
    }

    // CreateFace makes fresh vertices every call, so what is above is triangle soup until it is
    // welded
    mesh.clean_and_merge();

    mesh
}

/// C# `TopologyTestMeshes.CreateRakedLBlock`: a 40 x 40 x 20 block with one quadrant cut out of it
/// so two walls meet at the origin, the `x = 0` wall square and the other raked `rake_degrees`
/// under the part. The two walls present their top edges to the top face at DIFFERENT angles, and
/// the two fillets there have different setbacks and different overshoots.
pub fn create_raked_l_block(rake_degrees: f64) -> Mesh {
    use polygon_mesh::csg::boolean_processing::do_array;
    use polygon_mesh::csg::CsgModes;
    use vector_math::Matrix4X4;

    let mut block = polygon_mesh::solids::platonic_solids::create_cube(40.0, 40.0, 20.0);

    block.translate(Vector3::new(0.0, 0.0, -10.0));

    // Big enough in every direction that its own far faces never touch the block, so the only thing
    // it contributes is the two planes through the origin: x = 0 and, after the rotation,
    // y = z tan(rake).
    let mut quadrant = polygon_mesh::solids::platonic_solids::create_cube(200.0, 200.0, 400.0);

    quadrant.transform(
        Matrix4X4::create_translation(100.0, 100.0, 0.0)
            * Matrix4X4::create_rotation_x(math_helper::degrees_to_radians(-rake_degrees)),
    );

    do_array(
        &[
            (&block, Matrix4X4::IDENTITY),
            (&quadrant, Matrix4X4::IDENTITY),
        ],
        CsgModes::Subtract,
        &polygon_mesh::CancelToken::new(),
        &polygon_mesh::csg::BooleanOptions::default(),
    )
    .expect("the quadrant cuts the block")
}

/// C# `SurfaceTestMeshes.CreateBossOnSlab`: the classic boss fillet fixture - a cylinder standing
/// on a slab, unioned into one solid, so the circle where the two meet is a closed CONCAVE loop.
/// The post runs well down into the slab rather than resting on its face, because two coincident
/// faces are a boolean's worst input and the fixture is not what is under test.
pub fn create_boss_on_slab(post_radius: f64, sides: i32, slab_top: f64) -> Mesh {
    use polygon_mesh::csg::boolean_processing::do_array;
    use polygon_mesh::csg::CsgModes;
    use vector_math::Matrix4X4;

    let slab = polygon_mesh::solids::platonic_solids::create_cube(30.0, 30.0, slab_top * 2.0);
    let mut post = create_cylinder(post_radius, 12.0, sides);

    post.translate(Vector3::new(0.0, 0.0, slab_top + 3.0));

    do_array(
        &[(&slab, Matrix4X4::IDENTITY), (&post, Matrix4X4::IDENTITY)],
        CsgModes::Union,
        &polygon_mesh::CancelToken::new(),
        &polygon_mesh::csg::BooleanOptions::default(),
    )
    .expect("the post and slab union")
}
