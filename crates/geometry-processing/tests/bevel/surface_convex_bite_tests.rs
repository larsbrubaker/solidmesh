//! Port of `SurfaceConvexBiteTests.cs`: a convex bite takes its own ring out of the faces and
//! nothing else, a ring that turns back is refused, a face wound against the plane keeps its
//! facing, and the ring's own clean-ups keep the part's corners.

use indexmap::IndexSet;

use geometry_processing::bevel::surface::surface_convex_bite::{
    try_take, with_the_vertices_standing_on_it, without_the_straight_through,
};
use geometry_processing::bevel::surface::SurfaceVertexPool;
use geometry_processing::bevel::BevelAdvisoryCode;
use polygon_mesh::solids::platonic_solids::create_cube_vector;
use vector_math::{Vector3, Vector3Float};

const MEASURED: f64 = 1e-6;

type Faces = Vec<(i32, i32, i32)>;

fn cube_and_pool() -> (SurfaceVertexPool, Faces) {
    let cube = create_cube_vector(Vector3::new(20.0, 20.0, 20.0));
    let pool = SurfaceVertexPool::new(&cube.vertices);
    let faces = cube.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    (pool, faces)
}

fn square_rim() -> Vec<Vector3> {
    vec![
        Vector3::new(-5.0, -5.0, 10.0),
        Vector3::new(5.0, -5.0, 10.0),
        Vector3::new(5.0, 5.0, 10.0),
        Vector3::new(-5.0, 5.0, 10.0),
    ]
}

/// C# `ASquareBiteTakesItsOwnRingOutOfTheFaceAndNothingElse`.
#[test]
fn a_square_bite_takes_its_own_ring_out_of_the_face_and_nothing_else() {
    let (mut pool, mut faces) = cube_and_pool();
    let split_from = pool.vertices().len() as i32;
    try_take(
        &mut pool,
        &mut faces,
        &square_rim(),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 10.0),
        &[],
        split_from,
    )
    .unwrap_or_else(|failure| panic!("{failure:?}"));
    assert!(
        (area_on(&pool, &faces, |at| (at.z - 10.0).abs() < 1e-6) - (400.0 - 100.0)).abs()
            <= MEASURED
    );
    assert!((area_on(&pool, &faces, |at| (at.z + 10.0).abs() < 1e-6) - 400.0).abs() <= MEASURED);
}

/// C# `ARingThatTurnsBackOnItselfIsRefusedRatherThanBitten`.
#[test]
fn a_ring_that_turns_back_on_itself_is_refused_rather_than_bitten() {
    let (mut pool, mut faces) = cube_and_pool();
    let before = faces.clone();
    let rim = [
        Vector3::new(-5.0, -5.0, 10.0),
        Vector3::new(5.0, -5.0, 10.0),
        Vector3::new(5.0, 0.0, 10.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(0.0, 5.0, 10.0),
        Vector3::new(-5.0, 5.0, 10.0),
    ];
    let split_from = pool.vertices().len() as i32;
    let failure = try_take(
        &mut pool,
        &mut faces,
        &rim,
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 10.0),
        &[],
        split_from,
    )
    .expect_err("an L-shaped ring is not convex");
    assert_eq!(failure.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);
    assert_eq!(
        faces, before,
        "a refused bite leaves the faces as they were"
    );
}

/// C# `AFaceWoundAgainstTheBitesPlaneKeepsItsOwnFacing`.
#[test]
fn a_face_wound_against_the_bites_plane_keeps_its_own_facing() {
    let (mut pool, mut faces) = cube_and_pool();
    let first = pool.add(Vector3::new(-8.0, -8.0, 10.0));
    let second = pool.add(Vector3::new(8.0, -8.0, 10.0));
    let third = pool.add(Vector3::new(-8.0, 8.0, 10.0));
    faces.push((first, third, second));
    let split_from = pool.vertices().len() as i32;
    try_take(
        &mut pool,
        &mut faces,
        &square_rim(),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 10.0),
        &[],
        split_from,
    )
    .unwrap_or_else(|failure| panic!("{failure:?}"));
    let on_top = |at: Vector3| (at.z - 10.0).abs() < 1e-6;
    assert!(
        (area_facing(&pool, &faces, Vector3::new(0.0, 0.0, -1.0), on_top) - (128.0 - 50.0)).abs()
            <= MEASURED
    );
    assert!(
        (area_facing(&pool, &faces, Vector3::new(0.0, 0.0, 1.0), on_top) - (400.0 - 100.0)).abs()
            <= MEASURED
    );
}

/// C# `AnOriginalCornerOfThePartIsKeptWhereACrossingWouldBeDropped`.
#[test]
fn an_original_corner_of_the_part_is_kept_where_a_crossing_would_be_dropped() {
    let mut pool = SurfaceVertexPool::new(&to_storage(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(5.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ]));
    let split_from = pool.vertices().len() as i32;
    let crossing = pool.add(Vector3::new(0.0, 5.0, 0.0));
    let kept = without_the_straight_through(&pool, &[0, 1, 2, 3, crossing], split_from);
    assert!(
        kept.contains(&1),
        "the part's own corner stays in the ring - instead {kept:?}"
    );
    assert!(
        !kept.contains(&crossing),
        "and the crossing goes - instead {kept:?}"
    );
}

/// C# `ACornerTheRingStopsCarryingGoesBackIn`.
#[test]
fn a_corner_the_ring_stops_carrying_goes_back_in() {
    let mut pool = SurfaceVertexPool::new(&to_storage(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ]));
    let split_from = pool.vertices().len() as i32;
    let ruler = pool.near_enough();
    let nearer = pool.add(Vector3::new(3.0, ruler * 1.35, 0.0));
    let further = pool.add(Vector3::new(6.0, ruler * 0.9, 0.0));
    let kept = without_the_straight_through(&pool, &[0, nearer, further, 1, 2], split_from);
    assert!(
        !kept.contains(&further),
        "the one the surviving side carries goes - instead {kept:?}"
    );
    assert!(
        kept.contains(&nearer),
        "and the one it no longer carries is back in - instead {kept:?}"
    );
}

/// C# `AVertexTheTwoSidesOfACornerBothCarryGoesInOnce`.
#[test]
fn a_vertex_the_two_sides_of_a_corner_both_carry_goes_in_once() {
    let mut pool = SurfaceVertexPool::new(&to_storage(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ]));
    let ruler = pool.near_enough();
    let on_both = pool.add(Vector3::new(ruler * 0.3, ruler * 0.3, 0.0));
    let grown = with_the_vertices_standing_on_it(
        &pool,
        vec![0, 1, 2],
        &IndexSet::from([on_both]),
        None,
        -1,
    );
    assert_eq!(
        grown.iter().filter(|&&at| at == on_both).count(),
        1,
        "instead the ring is {grown:?}"
    );
}

fn to_storage(corners: &[Vector3]) -> Vec<Vector3Float> {
    corners
        .iter()
        .map(|&at| Vector3Float::from_vector3(at))
        .collect()
}

fn area_facing(
    pool: &SurfaceVertexPool,
    faces: &Faces,
    facing: Vector3,
    on_the_plane: impl Fn(Vector3) -> bool,
) -> f64 {
    faces
        .iter()
        .map(|&(v0, v1, v2)| {
            (
                pool.position_of(v0),
                pool.position_of(v1),
                pool.position_of(v2),
            )
        })
        .filter(|&(a, b, c)| on_the_plane(a) && on_the_plane(b) && on_the_plane(c))
        .map(|(a, b, c)| (b - a).cross(c - a))
        .filter(|normal| normal.dot(facing) > 0.0)
        .map(|normal| normal.length() / 2.0)
        .sum()
}

fn area_on(pool: &SurfaceVertexPool, faces: &Faces, on_the_plane: impl Fn(Vector3) -> bool) -> f64 {
    faces
        .iter()
        .map(|&(v0, v1, v2)| {
            (
                pool.position_of(v0),
                pool.position_of(v1),
                pool.position_of(v2),
            )
        })
        .filter(|&(a, b, c)| on_the_plane(a) && on_the_plane(b) && on_the_plane(c))
        .map(|(a, b, c)| (b - a).cross(c - a).length() / 2.0)
        .sum()
}
