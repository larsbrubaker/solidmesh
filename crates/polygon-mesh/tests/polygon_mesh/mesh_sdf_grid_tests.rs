//! The drag preview's distance field. Offsets of a 20 mm cube land on the exact rounded box and
//! the shrunk cube, an inside-out cube reads as the same solid, one field serves every step, and
//! offsets keep the part's topology.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;

use polygon_mesh::csg::minkowski_processing::sphere_mesh;
use polygon_mesh::csg::minkowski_shell_orientation::mesh_may_need_repair;
use polygon_mesh::processors::outline_to_mesh::extrude_contours;
use polygon_mesh::sdf::{MeshSdfGrid, SdfGridError};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector2};

const SIDE: f64 = 20.0;
const RADIUS: f64 = 2.0;

fn none() -> CancelToken {
    CancelToken::new()
}

fn grid(mesh: Mesh, max_radius: f64, resolution: i32) -> MeshSdfGrid {
    MeshSdfGrid::new(mesh, max_radius, resolution).expect("valid grid arguments")
}

fn offset(sdf: &MeshSdfGrid, iso: f64) -> Mesh {
    sdf.extract(iso, &none()).expect("extract")
}

fn inside_out_cube(x: f64, y: f64, z: f64) -> Mesh {
    let mut cube = create_cube(x, y, z);
    cube.reverse_faces();
    cube
}

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not within {tolerance} of {expected}"
    );
}

#[test]
fn dilated_cube_matches_rounded_box() {
    let sdf = grid(create_cube(SIDE, SIDE, SIDE), RADIUS, 128);
    let volume = closed_volume(&offset(&sdf, RADIUS));
    assert_within(
        volume,
        rounded_box_volume(SIDE, RADIUS),
        tolerance(SIDE + 2.0 * RADIUS, sdf.cell_size()),
    );
}

#[test]
fn eroded_cube_matches_shrunk_cube() {
    let sdf = grid(create_cube(SIDE, SIDE, SIDE), RADIUS, 128);
    let shrunk = SIDE - 2.0 * RADIUS;
    assert_within(
        closed_volume(&offset(&sdf, -RADIUS)),
        shrunk * shrunk * shrunk,
        tolerance(shrunk, sdf.cell_size()),
    );
}

#[test]
fn inside_out_cube_gives_the_same_offsets() {
    let sdf = grid(inside_out_cube(SIDE, SIDE, SIDE), RADIUS, 128);
    let shrunk = SIDE - 2.0 * RADIUS;
    assert_within(
        closed_volume(&offset(&sdf, RADIUS)),
        rounded_box_volume(SIDE, RADIUS),
        tolerance(SIDE + 2.0 * RADIUS, sdf.cell_size()),
    );
    assert_within(
        closed_volume(&offset(&sdf, -RADIUS)),
        shrunk * shrunk * shrunk,
        tolerance(shrunk, sdf.cell_size()),
    );
}

#[test]
fn only_a_part_that_may_be_inverted_goes_through_the_kernel_orientation() {
    // A closed, right-way-out shell is taken as is; inside out, or holding an inward-wound
    // cavity, it takes the kernel's repair.
    let right = grid(create_cube(SIDE, SIDE, SIDE), RADIUS, 32);
    right.build(&none()).expect("build");
    let inside_out = grid(inside_out_cube(SIDE, SIDE, SIDE), RADIUS, 32);
    inside_out.build(&none()).expect("build");
    let mut with_cavity = create_cube(SIDE, SIDE, SIDE);
    with_cavity.copy_all_faces(&inside_out_cube(8.0, 8.0, 8.0), Matrix4X4::IDENTITY);

    assert_eq!(right.orient_repairs(), 0);
    assert_eq!(inside_out.orient_repairs(), 1);
    assert!(mesh_may_need_repair(&with_cavity));
    assert_eq!(wrong_sign_voxels(&inside_out), 0);
}

#[test]
fn field_is_computed_once_for_every_step() {
    let sdf = grid(create_cube(SIDE, SIDE, SIDE), RADIUS, 48);
    let mut iso = -RADIUS;
    while iso <= RADIUS {
        offset(&sdf, iso);
        iso += 0.5;
    }
    assert_eq!(sdf.grid_builds(), 1);
}

#[test]
fn separate_bodies_one_inside_out_offset_like_both_right_way_out() {
    let mut part = create_cube(10.0, 10.0, 10.0);
    part.copy_all_faces(
        &inside_out_cube(10.0, 10.0, 10.0),
        Matrix4X4::create_translation(30.0, 0.0, 0.0),
    );
    let mut right_way_out = create_cube(10.0, 10.0, 10.0);
    right_way_out.copy_all_faces(
        &create_cube(10.0, 10.0, 10.0),
        Matrix4X4::create_translation(30.0, 0.0, 0.0),
    );
    let sdf = grid(part, 1.0, 128);
    let reference = grid(right_way_out, 1.0, 128);

    for iso in [1.0, -1.0] {
        assert_within(
            closed_volume(&offset(&sdf, iso)),
            closed_volume(&offset(&reference, iso)),
            1e-6,
        );
    }

    // Both land on the analytic offsets. Two 10 mm cubes 30 mm apart make a coarse grid (0.37
    // mm cells), and marching cubes bevels the eroded cubes' sharp edges by a fraction of a
    // cell, which dominates here: 1% of 1024 measured.
    let grown = 2.0 * rounded_box_volume(10.0, 1.0);
    assert_within(closed_volume(&offset(&sdf, 1.0)), grown, 0.02 * grown);
    let shrunk = 2.0 * 8.0 * 8.0 * 8.0;
    assert_within(closed_volume(&offset(&sdf, -1.0)), shrunk, 0.02 * shrunk);
}

#[test]
fn cavity_grows_when_eroded_and_closes_when_dilated() {
    let mut part = create_cube(SIDE, SIDE, SIDE);
    part.copy_all_faces(&inside_out_cube(8.0, 8.0, 8.0), Matrix4X4::IDENTITY);
    let sdf = grid(part, 5.0, 128);

    // Eroded by 1: the outside shrinks to 18 and the cavity grows to a rounded 10.
    let eroded = 18.0 * 18.0 * 18.0 - rounded_box_volume(8.0, 1.0);
    let eroded_volume = closed_volume(&offset(&sdf, -1.0));
    assert_within(
        eroded_volume,
        eroded,
        tolerance(18.0, sdf.cell_size()) + tolerance(10.0, sdf.cell_size()),
    );

    // Dilated by 5: the cavity's 8 mm shrinks past nothing and the part is solid.
    assert_within(
        closed_volume(&offset(&sdf, 5.0)),
        rounded_box_volume(SIDE, 5.0),
        tolerance(SIDE + 10.0, sdf.cell_size()),
    );
}

#[test]
fn small_open_hole_still_offsets_like_the_closed_part() {
    let closed = create_cube(SIDE, SIDE, SIDE);
    let mut holed = create_cube(SIDE, SIDE, SIDE);
    holed.faces.remove(0);
    let closed_sdf = grid(closed, RADIUS, 128);
    let holed_sdf = grid(holed, RADIUS, 128);

    // Every voxel keeps its sign (the vote outruns the one wrong axis through the hole), but
    // the distance near the hole is to its rim, so both offsets bend into it.
    holed_sdf.build(&none()).expect("build");
    assert_eq!(wrong_sign_voxels(&holed_sdf), 0);

    let shrunk = closed_volume(&offset(&closed_sdf, -RADIUS));
    assert_within(
        closed_volume(&offset(&holed_sdf, -RADIUS)),
        shrunk,
        0.05 * shrunk,
    );

    let grown = closed_volume(&offset(&closed_sdf, RADIUS));
    assert_within(
        closed_volume(&offset(&holed_sdf, RADIUS)),
        grown,
        0.05 * grown,
    );
}

#[test]
fn grid_on_the_faces_leaves_no_wrong_sign_voxel() {
    // With a 2 mm range at 52 voxels the cell is exactly 0.5 mm and the grid origin sits 3 mm
    // below the cube's minimum, so grid lines run along its faces, edges and face diagonals.
    let sdf = grid(create_cube(SIDE, SIDE, SIDE), RADIUS, 52);
    sdf.build(&none()).expect("build");
    assert_eq!(sdf.cell_size(), 0.5);
    assert_eq!(wrong_sign_voxels(&sdf), 0);
}

#[test]
fn cancelled_build_stops_promptly() {
    // A 256 grid around an 8k-triangle ball: a cancel at 20000 lands mid-pass.
    const CANCEL_AT: i64 = 20000;
    let ball = sphere_mesh(10.0, 128).expect("sphere");
    let mut sdf = grid(ball, 5.0, MeshSdfGrid::MAX_RESOLUTION);
    let cancel = CancelToken::new();
    let trigger = cancel.clone();
    sdf.set_flood_progress(Some(Arc::new(move |processed| {
        if processed == CANCEL_AT {
            trigger.cancel();
        }
    })));

    let clock = Instant::now();
    assert!(matches!(sdf.build(&cancel), Err(SdfGridError::Cancelled)));

    // Work, not time: the flood polls the token at every 64th voxel, so it stops at most 64
    // voxels past the cancel, plus one per worker (one here: the flood runs on one thread).
    let workers = std::thread::available_parallelism().map_or(1, |n| n.get()) as i64;
    assert!(sdf.flood_work() - CANCEL_AT <= 64 + workers);
    assert_eq!(sdf.grid_builds(), 0);

    // A hang guard only.
    assert!(clock.elapsed().as_secs_f64() < 60.0);
}

/// A 12-tooth gear eroded by 1.0479 mm stays one connected ring: the diagonal teeth's tips
/// must not come off as floating shells.
#[test]
fn eroded_gear_tooth_tips_stay_joined_to_the_body() {
    const TEETH: i32 = 12;
    let (tip_radius, root_radius) = (15.0, 12.5);
    let fraction = [0.0, 0.25, 0.4, 0.6, 0.75];
    let radius = [
        root_radius,
        root_radius,
        tip_radius,
        tip_radius,
        root_radius,
    ];
    let mut gear = Vec::new();
    for tooth in 0..TEETH {
        for k in 0..fraction.len() {
            let angle = (tooth as f64 + fraction[k]) * std::f64::consts::PI * 2.0 / TEETH as f64;
            gear.push(Vector2::new(
                radius[k] * angle.cos(),
                radius[k] * angle.sin(),
            ));
        }
    }
    let mut bore = vec![Vector2::new(2.5, 0.0)];
    for i in 1..32 {
        let angle = -(i as f64) * std::f64::consts::PI * 2.0 / 32.0;
        bore.push(Vector2::new(2.5 * angle.cos(), 2.5 * angle.sin()));
    }
    const DEPTH: f64 = 1.0479;
    let sdf = grid(
        extrude_contours(&[gear, bore], 8.0).expect("extrude"),
        DEPTH,
        130,
    );

    let eroded = offset(&sdf, -DEPTH);
    closed_volume(&eroded);
    assert_eq!(shell_count(&eroded), 1);
    assert_eq!(euler_characteristic(&eroded), 0);
}

/// A torus and an L bracket offset both ways come out closed, manifold, one shell, and of the
/// part's own topology.
#[test]
fn offsets_of_torus_and_bracket_keep_their_topology() {
    let bracket = [
        (0.0, 0.0),
        (40.0, 0.0),
        (40.0, 4.0),
        (4.0, 4.0),
        (4.0, 30.0),
        (0.0, 30.0),
    ]
    .map(|(x, y)| Vector2::new(x, y))
    .to_vec();

    for (part, euler) in [
        (torus(10.0, 3.0, 64, 32), 0),
        (extrude_contours(&[bracket], 20.0).expect("extrude"), 2),
    ] {
        let sdf = grid(part, 1.5, 96);
        for iso in [1.5, 0.7, -0.7, -1.5] {
            let result = offset(&sdf, iso);
            closed_volume(&result);
            assert_eq!(shell_count(&result), 1);
            assert_eq!(euler_characteristic(&result), euler);
        }
    }
}

/// A closed torus about Z, outward wound.
fn torus(major_radius: f64, minor_radius: f64, around: usize, across: usize) -> Mesh {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for i in 0..around {
        let u = i as f64 * std::f64::consts::PI * 2.0 / around as f64;
        for j in 0..across {
            let v = j as f64 * std::f64::consts::PI * 2.0 / across as f64;
            let ring = major_radius + minor_radius * v.cos();
            vertices.extend([ring * u.cos(), ring * u.sin(), minor_radius * v.sin()]);
        }
    }
    for i in 0..around {
        for j in 0..across {
            let a = (i * across + j) as i32;
            let b = (((i + 1) % around) * across + j) as i32;
            let c = (((i + 1) % around) * across + (j + 1) % across) as i32;
            let d = (i * across + (j + 1) % across) as i32;
            faces.extend([a, b, c, a, c, d]);
        }
    }
    let mut torus = Mesh::from_arrays(&vertices, &faces);
    if signed_volume(&torus) < 0.0 {
        torus.reverse_faces();
    }
    torus
}

/// V - E + F, counting each undirected edge once.
fn euler_characteristic(mesh: &Mesh) -> i64 {
    let mut edges = HashSet::new();
    for face in mesh.faces.iter() {
        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            edges.insert(if a < b { (a, b) } else { (b, a) });
        }
    }
    mesh.vertices.len() as i64 - edges.len() as i64 + mesh.faces.len() as i64
}

/// The number of edge-connected pieces.
fn shell_count(mesh: &Mesh) -> usize {
    let mut parent: Vec<usize> = (0..mesh.vertices.len()).collect();
    fn find(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }
    for face in mesh.faces.iter() {
        let (a, b) = (
            find(&mut parent, face.v0 as usize),
            find(&mut parent, face.v1 as usize),
        );
        parent[a] = b;
        let (b, c) = (
            find(&mut parent, face.v1 as usize),
            find(&mut parent, face.v2 as usize),
        );
        parent[b] = c;
    }
    let mut roots = HashSet::new();
    for face in mesh.faces.iter() {
        roots.insert(find(&mut parent, face.v0 as usize));
    }
    roots.len()
}

/// The signed volume, after asserting a closed, consistently wound, manifold surface.
fn closed_volume(mesh: &Mesh) -> f64 {
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for face in mesh.faces.iter() {
        for edge in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *directed.entry(edge).or_insert(0) += 1;
        }
    }
    let bad = directed
        .iter()
        .filter(|(edge, &count)| {
            count != 1 || directed.get(&(edge.1, edge.0)).copied().unwrap_or(0) != 1
        })
        .count();
    assert_eq!(bad, 0);
    assert!(mesh.is_manifold());
    signed_volume(mesh)
}

/// Samples of a centred cube's field whose sign disagrees with the cube, ignoring the ones on
/// its faces, where either sign is right.
fn wrong_sign_voxels(sdf: &MeshSdfGrid) -> usize {
    let field = sdf.field().expect("built");
    let origin = sdf.field_origin();
    let mut wrong = 0;
    for k in 0..field.nk {
        for j in 0..field.nj {
            for i in 0..field.ni {
                let p = [
                    origin.x + sdf.cell_size() * i as f64,
                    origin.y + sdf.cell_size() * j as f64,
                    origin.z + sdf.cell_size() * k as f64,
                ];
                let outside = p[0].abs().max(p[1].abs().max(p[2].abs())) - SIDE / 2.0;
                let value = field.buffer[i + field.ni * (j + field.nj * k)];
                if outside.abs() > 1e-6 && (outside < 0.0) != (value < 0.0) {
                    wrong += 1;
                }
            }
        }
    }
    wrong
}

/// A cube of side a grown by r: the cube, six slabs, twelve quarter cylinders and eight sphere
/// octants.
pub(crate) fn rounded_box_volume(side: f64, radius: f64) -> f64 {
    use std::f64::consts::PI;
    side * side * side
        + 6.0 * side * side * radius
        + 3.0 * PI * side * radius * radius
        + 4.0 / 3.0 * PI * radius * radius * radius
}

/// A fiftieth of a voxel of drift over the surface of a box of this side.
fn tolerance(side: f64, cell_size: f64) -> f64 {
    0.02 * 6.0 * side * side * cell_size
}

pub(crate) fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;
    for face in mesh.faces.iter() {
        let a = mesh.vertices[face.v0 as usize];
        let b = mesh.vertices[face.v1 as usize];
        let c = mesh.vertices[face.v2 as usize];
        // In float: the stored `Vector3Float` components are multiplied before the sum is
        // widened and divided by 6.0.
        let six_volume = (a.x * ((b.y * c.z) - (b.z * c.y))) - (a.y * ((b.x * c.z) - (b.z * c.x)))
            + (a.z * ((b.x * c.y) - (b.y * c.x)));
        total += six_volume as f64 / 6.0;
    }
    total
}
