//! Fixture builders for the topology and bevel tests: prisms, grids, frustums, plates with bosses,
//! ribs and walls, each a small closed solid whose creases a test picks. Among them
//! `create_isosceles_prism` (arc chording), `create_arc_prism` (sweep-back depth) and
//! `create_frustum` (`tapered_barrel_test_meshes.rs`).

use solidmesh_math::{math_helper, Vector2, Vector3};
use solidmesh_mesh::Mesh;

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
    let mut path = crate::path_solids::Outline::new();
    path.move_to(radius, 0.0);
    for i in 1..=segments {
        let angle = std::f64::consts::PI * f64::from(i) / f64::from(segments);
        path.line_to(radius * angle.cos(), radius * angle.sin());
    }

    path.close_polygon();

    crate::path_solids::extrude(&path, height).expect("a half disc extrudes")
}

/// The solid cells of an axis-aligned grid, each exposed
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

/// A 28 x 30 x 5 plate carrying a 4 x 30 x 20
/// wall, 30 mm of it, from x = 12 to x = 16 and up to z = 25. Its two feet are a pair of concave
/// creases - (12, y, 5) and (16, y, 5) - facing away from each other across a wall thinner than
/// the radius they are filleted at. The ledges are 12 mm so a radius-8 setback has face to land on
/// at both feet.
// Used by the thin wall bead clip tests in mattercad-model; the kernel `bevel` binary has no user.
#[allow(dead_code)]
pub fn create_thin_wall_between_ledges() -> Mesh {
    // The whole base, and above it only the middle column, which is the wall
    create_grid_solid(
        &[0.0, 12.0, 16.0, 28.0],
        &[0.0, 30.0],
        &[0.0, 5.0, 25.0],
        |i, j, k| (0..3).contains(&i) && j == 0 && (0..2).contains(&k) && (k == 0 || i == 1),
    )
}

/// A 20 x 20 x 4 plate with an 8 x 8 boss standing
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

/// Two 12 x 12 blocks on (or sunk into) a
/// 4 mm plate, `land` of plate between them, their near foot creases collinear along `y = 4`.
///
/// **Sunk**: each pocket's floor is a region of its own, the two blocks' footprints lying between
/// them, so the crease that resumes does not ride the pair this one rides. **Proud**: at a block's
/// foot the wall bounds the block itself, so a face FALLS AWAY there, the corner a crease dies in.
pub fn create_two_aligned_blocks_on_a_plate(standing_proud: bool, land: f64) -> Mesh {
    let xs = [0.0, 4.0, 16.0, 16.0 + land, 28.0 + land, 32.0 + land];
    let ys = [0.0, 4.0, 16.0, 20.0];
    let zs = [0.0, 6.0, 10.0];

    // The two cells the blocks are: the upper layer's middle row, first and third columns.
    let is_a_block = |i: i32, j: i32, k: i32| k == 1 && j == 1 && (i == 1 || i == 3);

    create_grid_solid(&xs, &ys, &zs, |i, j, k| {
        if !(0..=4).contains(&i) || !(0..=2).contains(&j) || !(0..=1).contains(&k) {
            return false;
        }

        // The plate is the whole lower layer either way; the upper layer is the two blocks alone,
        // or everything BUT them, which is the same two shapes the other way round.
        k == 0
            || if standing_proud {
                is_a_block(i, j, k)
            } else {
                !is_a_block(i, j, k)
            }
    })
}

/// Where the first block's foot crease runs,
/// along the near side of the plate.
pub fn the_first_blocks_near_crease() -> (Vector3, Vector3) {
    (Vector3::new(4.0, 4.0, 6.0), Vector3::new(16.0, 4.0, 6.0))
}

/// And the second's, collinear with it across
/// the land.
pub fn the_second_blocks_near_crease(land: f64) -> (Vector3, Vector3) {
    (
        Vector3::new(16.0 + land, 4.0, 6.0),
        Vector3::new(28.0 + land, 4.0, 6.0),
    )
}

/// A `size` cube with a `pocket` cube bitten out
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

/// A 20 x 20 x 5 plate with a rib 4 wide standing 10 on
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

/// A `sides`-sided cylinder of `radius` and `height` about
/// z, centred on the origin.
pub fn create_cylinder(radius: f64, height: f64, sides: i32) -> Mesh {
    let mut path = crate::path_solids::Outline::new();
    path.move_to(0.0, -height / 2.0);
    path.line_to(radius, -height / 2.0);
    path.line_to(radius, height / 2.0);
    path.line_to(0.0, height / 2.0);
    crate::path_solids::revolve(&path, sides, 0.0, std::f64::consts::TAU, true)
        .expect("the cylinder revolves")
}

/// A `sides`-sided prism of circumradius
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

/// A `sides`-sided frustum about z, `bottom_radius` at
/// z = -height / 2 tapering to `top_radius` at +height / 2, closed at both ends.
pub fn create_frustum(bottom_radius: f64, top_radius: f64, height: f64, sides: i32) -> Mesh {
    let mut path = crate::path_solids::Outline::new();
    path.move_to(0.0, -height / 2.0);
    path.line_to(bottom_radius, -height / 2.0);
    path.line_to(top_radius, height / 2.0);
    path.line_to(0.0, height / 2.0);
    crate::path_solids::revolve(&path, sides, 0.0, std::f64::consts::TAU, true)
        .expect("the frustum revolves")
}

/// An L of three 10 mm cells, 10 tall - one concave
/// edge up its inside corner, meeting the top's convex rim at (10, 10, 10).
pub fn create_l_shaped_grid_prism() -> Mesh {
    let across = [0.0, 10.0, 20.0];
    create_grid_solid(&across, &across, &[0.0, 10.0], |i, j, k| {
        (0..2).contains(&i) && (0..2).contains(&j) && k == 0 && !(i == 1 && j == 1)
    })
}

/// The same L prism LEANED OVER as it rises,
/// about its own top face - that face is still `z = 10` and its rim is exactly where it was, but
/// every vertical edge, the concave one included, now runs `lean_degrees` off square to it. Leaning
/// in x tilts the face the two picked edges SHARE and leaves the unpicked wall the plane `y = 10`;
/// leaning in y (`along_the_rim`) tilts that wall instead. The grid cap is sheared rather than the
/// extruded one because the kernel wants the grid cap for anything that lands on the cap face.
pub fn create_sheared_l_shaped_grid_prism(lean_degrees: f64, along_the_rim: bool) -> Mesh {
    let mut mesh = create_l_shaped_grid_prism();
    let shear = math_helper::degrees_to_radians(lean_degrees).tan();

    mesh.transform(solidmesh_math::Matrix4X4::new(
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        if along_the_rim { 0.0 } else { shear },
        if along_the_rim { shear } else { 0.0 },
        1.0,
        0.0,
        if along_the_rim { 0.0 } else { -10.0 * shear },
        if along_the_rim { -10.0 * shear } else { 0.0 },
        0.0,
        1.0,
    ));

    mesh
}

/// An L cross section extruded 10 in z - five square
/// corners and one reflex corner at (10, 10), which is the one concave edge of the solid.
pub fn create_l_shaped_prism() -> Mesh {
    let mut path = crate::path_solids::Outline::new();
    path.move_to(0.0, 0.0);
    path.line_to(20.0, 0.0);
    path.line_to(20.0, 10.0);
    path.line_to(10.0, 10.0);
    path.line_to(10.0, 20.0);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    crate::path_solids::extrude(&path, 10.0).expect("the L outline extrudes")
}

/// A square prism with a triangular notch cut into its
/// +x side. The notch's apex at (16, 10) is a reflex corner, so the vertical edge there is concave
/// and its VOID wedge is 2 atan(1.5) = 112.62 degrees, at an angle chosen so every outline
/// coordinate is an integer and nothing rounds on the way through Clipper.
pub fn create_notched_prism() -> Mesh {
    let mut path = crate::path_solids::Outline::new();
    path.move_to(0.0, 0.0);
    path.line_to(20.0, 0.0);
    path.line_to(20.0, 4.0);
    path.line_to(16.0, 10.0);
    path.line_to(20.0, 16.0);
    path.line_to(20.0, 20.0);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    crate::path_solids::extrude(&path, 10.0).expect("the notched outline extrudes")
}

/// A square prism whose top edge is kinked INWARD
/// by a hair, so the vertical edge at the kink is concave but folds by only `turn_degrees`. The
/// fixture for the band between the two tolerances that matter: a fold of more than
/// `FeatureEdge::default_flat_angle_degrees()` is a feature edge and can be selected, and a fold of
/// less than a degree is too little for a blend to be carried round. The kink is
/// `10 tan(turn / 2)` deep over a 10 mm run, which at 0.8 degrees is 70 microns - seventy of
/// agg's own 0.001 mm grid steps, so the extrusion does not round it away. The usual `height` is
/// 10.
// Read by mattercad-model's mixed corner tests; the geometry-processing binaries that mount this
// file do not use it.
#[allow(dead_code)]
pub fn create_shallow_reflex_prism(turn_degrees: f64, height: f64) -> Mesh {
    let kink = 10.0 * (math_helper::degrees_to_radians(turn_degrees) / 2.0).tan();

    let mut path = crate::path_solids::Outline::new();
    path.move_to(0.0, 0.0);
    path.line_to(20.0, 0.0);
    path.line_to(20.0, 20.0);
    path.line_to(10.0, 20.0 - kink);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    crate::path_solids::extrude(&path, height).expect("the shallow reflex outline extrudes")
}

/// The L prism whose inner corner FADES -
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

/// A 40 x 40 x 20 block with one quadrant cut out of it
/// so two walls meet at the origin, the `x = 0` wall square and the other raked `rake_degrees`
/// under the part. The two walls present their top edges to the top face at DIFFERENT angles, and
/// the two fillets there have different setbacks and different overshoots.
pub fn create_raked_l_block(rake_degrees: f64) -> Mesh {
    use solidmesh_math::Matrix4X4;
    use solidmesh_mesh::csg::boolean_processing::do_array;
    use solidmesh_mesh::csg::CsgModes;

    let mut block = solidmesh_mesh::solids::platonic_solids::create_cube(40.0, 40.0, 20.0);

    block.translate(Vector3::new(0.0, 0.0, -10.0));

    // Big enough in every direction that its own far faces never touch the block, so the only thing
    // it contributes is the two planes through the origin: x = 0 and, after the rotation,
    // y = z tan(rake).
    let mut quadrant = solidmesh_mesh::solids::platonic_solids::create_cube(200.0, 200.0, 400.0);

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
        &solidmesh_mesh::CancelToken::new(),
        &solidmesh_mesh::csg::BooleanOptions::default(),
    )
    .expect("the quadrant cuts the block")
}

/// The classic boss fillet fixture - a cylinder standing
/// on a slab, unioned into one solid, so the circle where the two meet is a closed CONCAVE loop.
/// The post runs well down into the slab rather than resting on its face, because two coincident
/// faces are a boolean's worst input and the fixture is not what is under test.
pub fn create_boss_on_slab(post_radius: f64, sides: i32, slab_top: f64) -> Mesh {
    use solidmesh_math::Matrix4X4;
    use solidmesh_mesh::csg::boolean_processing::do_array;
    use solidmesh_mesh::csg::CsgModes;

    let slab = solidmesh_mesh::solids::platonic_solids::create_cube(30.0, 30.0, slab_top * 2.0);
    let mut post = create_cylinder(post_radius, 12.0, sides);

    post.translate(Vector3::new(0.0, 0.0, slab_top + 3.0));

    do_array(
        &[(&slab, Matrix4X4::IDENTITY), (&post, Matrix4X4::IDENTITY)],
        CsgModes::Union,
        &solidmesh_mesh::CancelToken::new(),
        &solidmesh_mesh::csg::BooleanOptions::default(),
    )
    .expect("the post and slab union")
}

/// Everything of the step that neither fin fixture changes -
/// the two sides, the bottom, the riser's top and back, and the tread's own front.
fn a_steps_shell(mesh: &mut Mesh) {
    // The two sides, each an L fanned from its own reflex corner - the corner that makes the
    // band's end a plane of its own.
    let side = [
        Vector2::new(10.0, 30.0),
        Vector2::new(20.0, 30.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(0.0, 0.0),
        Vector2::new(0.0, 20.0),
    ];

    for i in 0..side.len() - 1 {
        mesh.create_face(&[
            Vector3::new(0.0, 10.0, 20.0),
            Vector3::new(0.0, side[i].x, side[i].y),
            Vector3::new(0.0, side[i + 1].x, side[i + 1].y),
        ]);
        mesh.create_face(&[
            Vector3::new(20.0, 10.0, 20.0),
            Vector3::new(20.0, side[i + 1].x, side[i + 1].y),
            Vector3::new(20.0, side[i].x, side[i].y),
        ]);
    }

    let quad = |mesh: &mut Mesh, q: [[f64; 3]; 4]| {
        let corners: Vec<Vector3> = q.iter().map(|c| Vector3::new(c[0], c[1], c[2])).collect();
        mesh.create_face(&corners);
    };
    quad(
        mesh,
        [
            [0.0, 0.0, 0.0],
            [0.0, 20.0, 0.0],
            [20.0, 20.0, 0.0],
            [20.0, 0.0, 0.0],
        ],
    );
    quad(
        mesh,
        [
            [0.0, 10.0, 30.0],
            [20.0, 10.0, 30.0],
            [20.0, 20.0, 30.0],
            [0.0, 20.0, 30.0],
        ],
    );
    quad(
        mesh,
        [
            [0.0, 20.0, 0.0],
            [0.0, 20.0, 30.0],
            [20.0, 20.0, 30.0],
            [20.0, 20.0, 0.0],
        ],
    );
    quad(
        mesh,
        [
            [0.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],
            [20.0, 0.0, 20.0],
            [0.0, 0.0, 20.0],
        ],
    );
}

/// The step with a riser fin narrowed to (2, 10)..
/// (6, 10) and its apex at (4, 5, 20), and a second, four-sided tent on (4, 5), (5, 1), (9, 1),
/// (8, 4) sharing that apex and nothing else - so four standing creases meet there, the tread's
/// rim pinches, and a walk of the rim that starts down the fin's own tail leaves the tent's ring
/// broken open behind it: a rim that FORKS. A band 6 mm in off the tread's left edge has the pinch
/// and one corner of the ring inside it.
pub fn create_forked_fins_on_a_slab() -> Mesh {
    let mut mesh = Mesh::new();

    let shared = Vector3::new(4.0, 5.0, 20.0);
    let left = Vector3::new(2.0, 10.0, 20.0);
    let right = Vector3::new(6.0, 10.0, 20.0);
    let peak = Vector3::new(4.0, 10.0, 23.0);

    let near_tent = Vector3::new(5.0, 1.0, 20.0);
    let far_tent = Vector3::new(9.0, 1.0, 20.0);
    let high_tent = Vector3::new(8.0, 4.0, 20.0);
    let tent_peak = Vector3::new(6.5, 2.75, 23.0);

    // The riser fin's own two faces FIRST, so that its two creases are the older edges at the
    // shared apex: a walk of the rim there takes them before the tent's, which is what leaves the
    // tent's ring broken open behind it - the shape this fixture is for.
    mesh.create_face(&[left, shared, peak]);
    mesh.create_face(&[shared, right, peak]);

    // The tread, less that notch and the tent's own ring, in the four convex pieces the pinch
    // leaves.
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(20.0, 0.0, 20.0),
        far_tent,
        near_tent,
    ]);
    mesh.create_face(&[
        Vector3::new(20.0, 0.0, 20.0),
        Vector3::new(20.0, 10.0, 20.0),
        high_tent,
        far_tent,
    ]);
    mesh.create_face(&[Vector3::new(20.0, 10.0, 20.0), right, shared, high_tent]);
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 20.0),
        near_tent,
        shared,
        left,
        Vector3::new(0.0, 10.0, 20.0),
    ]);

    // The riser, with the one notch its own fin makes on its foot.
    mesh.create_face(&[
        Vector3::new(0.0, 10.0, 20.0),
        left,
        peak,
        Vector3::new(0.0, 10.0, 30.0),
    ]);
    mesh.create_face(&[
        right,
        Vector3::new(20.0, 10.0, 20.0),
        Vector3::new(20.0, 10.0, 30.0),
        peak,
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 10.0, 30.0),
        peak,
        Vector3::new(20.0, 10.0, 30.0),
    ]);

    a_steps_shell(&mut mesh);

    // And the tent's four.
    mesh.create_face(&[shared, near_tent, tent_peak]);
    mesh.create_face(&[near_tent, far_tent, tent_peak]);
    mesh.create_face(&[far_tent, high_tent, tent_peak]);
    mesh.create_face(&[high_tent, shared, tent_peak]);

    mesh.clean_and_merge();

    mesh
}

/// A step with ONE fin standing on its tread, against the riser. The step
/// is 20 wide: the tread is `y` in [0, 10] at `z` = 20 and the riser stands behind it to `z` = 30.
/// The fin is the tetrahedron on `(2, 10, 20)`, `(10, 10, 20)`, `(6, 5, 20)` and the peak
/// `(6, 10, 24)`, standing on the tread and buried in the riser behind it. The solid is
/// 10000 + 80/3 mm^3.
pub fn create_fin_on_a_slab() -> Mesh {
    let mut mesh = Mesh::new();

    let a = Vector3::new(2.0, 10.0, 20.0);
    let b = Vector3::new(10.0, 10.0, 20.0);
    let c = Vector3::new(6.0, 5.0, 20.0);
    let peak = Vector3::new(6.0, 10.0, 24.0);

    // The tread, in five triangles rather than a fan: the fin's root bites a notch out of it whose
    // apex is a reflex corner, and no corner of the tread can see past that apex.
    let corner = Vector3::new(0.0, 0.0, 20.0);

    mesh.create_face(&[
        corner,
        Vector3::new(20.0, 0.0, 20.0),
        Vector3::new(20.0, 10.0, 20.0),
    ]);
    mesh.create_face(&[Vector3::new(20.0, 10.0, 20.0), b, c]);
    mesh.create_face(&[corner, Vector3::new(20.0, 10.0, 20.0), c]);
    mesh.create_face(&[corner, c, a]);
    mesh.create_face(&[corner, a, Vector3::new(0.0, 10.0, 20.0)]);

    // The riser, which carries the fin's own notch on its foot: three convex pieces rather than a
    // fan, because every fan of this shape runs a diagonal through the notch's peak.
    mesh.create_face(&[
        Vector3::new(0.0, 10.0, 20.0),
        a,
        peak,
        Vector3::new(0.0, 10.0, 30.0),
    ]);
    mesh.create_face(&[
        b,
        Vector3::new(20.0, 10.0, 20.0),
        Vector3::new(20.0, 10.0, 30.0),
        peak,
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 10.0, 30.0),
        peak,
        Vector3::new(20.0, 10.0, 30.0),
    ]);

    a_steps_shell(&mut mesh);

    // The fin's two sloping faces, each wound to face out and away from the peak.
    mesh.create_face(&[a, c, peak]);
    mesh.create_face(&[c, b, peak]);

    mesh.clean_and_merge();

    mesh
}

/// A 30 x 30 plate, its top at z = 20 with the front
/// rim broken at x = 8 and 22, and a triangular boss on `(5, 6)`, `(25, 6)`, `(15, 20)` standing to
/// z = 28 - its long foot edge running alongside the picked middle third of the front rim.
pub fn create_plate_with_a_boss_foot_spanning_the_rim() -> Mesh {
    let top = 20.0;
    let boss_top = 28.0;

    // The plate's top rim, with the two extra corners on the front that make its middle third an
    // edge of its own to pick
    let o0 = Vector3::new(0.0, 0.0, top);
    let o1 = Vector3::new(8.0, 0.0, top);
    let o2 = Vector3::new(22.0, 0.0, top);
    let o3 = Vector3::new(30.0, 0.0, top);
    let o4 = Vector3::new(30.0, 30.0, top);
    let o5 = Vector3::new(0.0, 30.0, top);

    // The boss's foot, and the same triangle at its top
    let a = Vector3::new(5.0, 6.0, top);
    let b = Vector3::new(25.0, 6.0, top);
    let c = Vector3::new(15.0, 20.0, top);
    let a_up = Vector3::new(a.x, a.y, boss_top);
    let b_up = Vector3::new(b.x, b.y, boss_top);
    let c_up = Vector3::new(c.x, c.y, boss_top);

    let mut mesh = Mesh::new();

    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 30.0, 0.0),
        Vector3::new(30.0, 30.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(30.0, 0.0, 0.0),
        o3,
        o2,
        o1,
        o0,
        Vector3::new(0.0, 0.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 30.0, 0.0),
        o5,
        o4,
        Vector3::new(30.0, 30.0, 0.0),
    ]);
    mesh.create_face(&[
        o0,
        o5,
        Vector3::new(0.0, 30.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(30.0, 30.0, 0.0),
        o4,
        o3,
    ]);

    // The top, triangle by triangle rather than as one polygon, so that the foot edge from a to b
    // is a single edge of it and nothing else crosses the band
    mesh.create_face(&[a, o0, o1]);
    mesh.create_face(&[a, o1, o2]);
    mesh.create_face(&[a, o2, o3]);
    mesh.create_face(&[a, o3, b]);
    mesh.create_face(&[o5, o0, a]);
    mesh.create_face(&[o5, a, c]);
    mesh.create_face(&[c, b, o3]);
    mesh.create_face(&[c, o3, o4]);
    mesh.create_face(&[o5, c, o4]);

    // The boss: each wall traverses its foot edge the other way round from the top, which is what
    // makes the two of them agree
    mesh.create_face(&[c, a, a_up, c_up]);
    mesh.create_face(&[b, c, c_up, b_up]);
    mesh.create_face(&[a, b, b_up, a_up]);
    mesh.create_face(&[a_up, b_up, c_up]);

    mesh.clean_and_merge();

    mesh
}
