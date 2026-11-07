//! A clip reads each point one way only -
//! the second bite takes the vertex the first left on the edge, and a crossing where a vertex,
//! a ring corner or a profile vertex already stands is that vertex.

use geometry_processing::bevel::surface::surface_convex_bite::try_take;
use geometry_processing::bevel::surface::surface_cut_clip::clip_by;
use geometry_processing::bevel::surface::SurfaceVertexPool;
use vector_math::{Vector3, Vector3Float};

const TALL: f64 = 20.0;
const WIDE: f64 = 0.15;

#[test]
fn the_second_bite_takes_the_vertex_the_first_left_on_the_edge_rather_than_one_beside_it() {
    let corners = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(WIDE, 0.0, 0.0),
        Vector3::new(WIDE, 0.0, TALL),
        Vector3::new(0.0, 0.0, TALL),
    ];
    let mut pool = SurfaceVertexPool::new(&to_storage(&corners));
    let mut faces = vec![(0, 1, 2), (0, 2, 3)];
    let hair = pool.near_enough() / 2.0;
    let split_from = pool.vertices().len() as i32;
    let first = bite(0.0, 0.08, TALL / 2.0);
    let second = bite(
        0.08,
        WIDE,
        (TALL / 2.0) - (hair * TALL / ((TALL * TALL) + (WIDE * WIDE)).sqrt()),
    );

    let first_at = vertices_of(&mut pool, &first);
    try_take(
        &mut pool,
        &mut faces,
        &first,
        Vector3::new(0.0, -1.0, 0.0),
        corners[0],
        &first_at,
        split_from,
    )
    .unwrap_or_else(|refused| panic!("{refused:?}"));
    let second_at = vertices_of(&mut pool, &second);
    try_take(
        &mut pool,
        &mut faces,
        &second,
        Vector3::new(0.0, -1.0, 0.0),
        corners[0],
        &second_at,
        split_from,
    )
    .unwrap_or_else(|refused| panic!("{refused:?}"));

    let mut standing: Vec<Vector3> = Vec::new();
    for &(v0, v1, v2) in &faces {
        for corner in [v0, v1, v2] {
            let at = pool.position_of(corner);
            if !standing.contains(&at) {
                standing.push(at);
            }
        }
    }

    let mut too_near = Vec::new();
    for one in 0..standing.len() {
        for other in one + 1..standing.len() {
            if (standing[one] - standing[other]).length() < pool.near_enough() {
                too_near.push(format!("{:?} and {:?}", standing[one], standing[other]));
            }
        }
    }
    assert!(too_near.is_empty(), "{}", too_near.join("; "));

    let on_the_diagonal = standing
        .iter()
        .filter(|at| {
            at.z > 1e-9 && at.z < TALL - 1e-9 && ((at.x * TALL) - (at.z * WIDE)).abs() < 1e-6
        })
        .count();
    assert_eq!(on_the_diagonal, 1);
}

#[test]
fn a_crossing_where_a_vertex_already_stands_is_that_vertex() {
    let (mut pool, ring) = the_side_crossed_at_the_origin();
    let near = pool.near_enough();
    let already = pool.add(Vector3::new(near * 0.4, near * 0.3, 0.0));
    let piece = clip_by(
        &ring,
        &mut pool,
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
        None,
        None,
    );
    assert!(piece.contains(&already));
    assert_eq!(standing_near(&pool, Vector3::ZERO), 1);
}

#[test]
fn a_crossing_where_a_corner_of_the_ring_already_stands_adds_nothing() {
    let (mut pool, ring) = the_side_crossed_at_the_origin();
    let near = pool.near_enough();
    let corner = pool.add(Vector3::new(near * 0.4, near * 0.3, 0.0));
    let mut with_the_corner = ring.clone();
    with_the_corner.push(corner);
    let piece = clip_by(
        &with_the_corner,
        &mut pool,
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
        None,
        None,
    );
    assert_eq!(standing_near(&pool, Vector3::ZERO), 1);
    assert!(piece.contains(&corner));

    let mut too_near = Vec::new();
    for one in 0..piece.len() {
        for other in one + 1..piece.len() {
            if (pool.position_of(piece[one]) - pool.position_of(piece[other])).length()
                < pool.near_enough()
            {
                too_near.push(format!(
                    "{:?} and {:?}",
                    pool.position_of(piece[one]),
                    pool.position_of(piece[other])
                ));
            }
        }
    }
    assert!(too_near.is_empty(), "{}", too_near.join("; "));
}

#[test]
fn a_crossing_landing_on_the_cuts_own_profile_is_that_profile_vertex() {
    let (mut pool, ring) = the_side_crossed_at_the_origin();
    let near = pool.near_enough();
    let off_the_side = pool.add(Vector3::new(0.0, near * 1.8, 0.0));
    let piece = clip_by(
        &ring,
        &mut pool,
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
        Some(&[off_the_side]),
        None,
    );
    assert!(piece.contains(&off_the_side));
    assert_eq!(standing_near(&pool, Vector3::ZERO), 0);
}

#[test]
fn a_crossing_a_little_way_along_the_cut_from_a_sides_own_end_is_that_end() {
    let (mut pool, ring) = the_side_crossed_at_the_origin();
    let was = pool.vertices().len();
    let end = pool.position_of(ring[1]);
    let near = pool.near_enough();
    let piece = clip_by(
        &ring,
        &mut pool,
        end - Vector3::new(near * 1.5, 0.0, 0.0),
        Vector3::new(-0.3, (1.0f64 - (0.3 * 0.3)).sqrt(), 0.0),
        None,
        None,
    );
    assert!(piece.contains(&ring[1]));
    assert_eq!(standing_near(&pool, end), 1);
    assert_eq!(pool.vertices().len(), was);
}

#[test]
fn a_crossing_a_little_way_across_the_cut_from_a_sides_own_end_is_a_new_point() {
    let (mut pool, ring) = the_side_crossed_at_the_origin();
    let end = pool.position_of(ring[1]);
    let near = pool.near_enough();
    let at = end - Vector3::new(near * 1.5, 0.0, 0.0);
    let piece = clip_by(
        &ring,
        &mut pool,
        at,
        Vector3::new(-1.0, 0.0, 0.0),
        None,
        None,
    );
    assert!(!piece.contains(&ring[1]));
    assert_eq!(standing_near(&pool, at), 1);
}

fn standing_near(pool: &SurfaceVertexPool, at: Vector3) -> usize {
    (0..pool.vertices().len() as i32)
        .filter(|&vertex| (pool.position_of(vertex) - at).length() < pool.near_enough())
        .count()
}

fn the_side_crossed_at_the_origin() -> (SurfaceVertexPool, Vec<i32>) {
    let pool = SurfaceVertexPool::new(&to_storage(&[
        Vector3::new(-10.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
    ]));
    (pool, vec![0, 1, 2])
}

fn bite(left: f64, right: f64, from: f64) -> Vec<Vector3> {
    vec![
        Vector3::new(left, 0.0, from),
        Vector3::new(right, 0.0, from),
        Vector3::new(right, 0.0, TALL + 1.0),
        Vector3::new(left, 0.0, TALL + 1.0),
    ]
}

fn vertices_of(pool: &mut SurfaceVertexPool, rim: &[Vector3]) -> Vec<i32> {
    rim.iter().map(|&corner| pool.add(corner)).collect()
}

fn to_storage(corners: &[Vector3]) -> Vec<Vector3Float> {
    corners
        .iter()
        .map(|&at| Vector3Float::from_vector3(at))
        .collect()
}
