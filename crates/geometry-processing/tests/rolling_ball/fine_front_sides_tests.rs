//! A chain whose two sides the kernel's flood joins round the part, read where one side is cut
//! into many fine strips: the joined surface is divided by distance over the surface, so how
//! finely a side is triangulated does not move the division toward it. Held to a 2-cube whose
//! back edges are blended, its front cut into strips 0.005 tall below the picked top front edge:
//! without regions the front is side B's whole (side A's way round the back to the ball's front
//! contact is about 7.75 long, side B's 0.25), and the ball takes the pass as it does with the
//! caller's face regions.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick_with_side_regions, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use vector_math::Vector3;

use super::band_route_tests::{creases, Triangles};

/// The fillet's radius.
const RADIUS: f64 = 0.25;

/// How tall each fine strip of the front is.
const STRIP: f64 = 0.005;

/// The cube from (-1, -1, -1) to (1, 1, 1) as a profile in (y, z) carried along x: the top back
/// corner first, then the top front corner (the picked edge), `fine` strips down the front, the
/// bottom front and the bottom back. The ends are fanned from the top back corner. Returns the
/// mesh and the profile's length: profile point i is vertex i at x = -1 and vertex n + i at x = 1.
fn fine_front_cube(fine: usize) -> (Vec<Vector3>, Triangles, i32) {
    let mut profile = vec![(1.0, 1.0), (-1.0, 1.0)];
    for k in 1..=fine {
        profile.push((-1.0, 1.0 - STRIP * k as f64));
    }
    profile.push((-1.0, -1.0));
    profile.push((1.0, -1.0));
    let n = profile.len() as i32;
    let positions: Vec<Vector3> = [-1.0, 1.0]
        .iter()
        .flat_map(|&x| profile.iter().map(move |&(y, z)| Vector3::new(x, y, z)))
        .collect();

    // Wound outward: walking the profile turns counter-clockwise seen from +x, so each side quad
    // runs (near i, near j, far j, far i), the near end faces -x and the far end +x.
    let mut triangles = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push((i, j, n + j));
        triangles.push((i, n + j, n + i));
    }
    for i in 1..n - 1 {
        triangles.push((0, i + 1, i));
        triangles.push((n, n + i, n + i + 1));
    }
    (positions, triangles, n)
}

/// The cube's creases but its three blended back lines (top back, bottom back, bottom front), so
/// the top's flood runs round the back into the front without crossing the chain.
fn blended_back(
    positions: &[Vector3],
    triangles: &Triangles,
    n: i32,
) -> Rc<dyn Fn(i32, i32) -> bool> {
    let sharp = creases(positions, triangles);
    let blended = [0, n - 2, n - 1];
    Rc::new(move |a, b| {
        let along_x = a % n == b % n && a != b;
        sharp(a, b) && !(along_x && blended.contains(&(a % n)))
    })
}

/// Each triangle's face of the cube as a region: its normal's largest axis and that axis's sign.
fn faces(positions: &[Vector3], triangles: &Triangles) -> Vec<u32> {
    triangles
        .iter()
        .map(|&(a, b, c)| {
            let p = positions[a as usize];
            let normal = (positions[b as usize] - p).cross(positions[c as usize] - p);
            let axes = [normal.x, normal.y, normal.z];
            let axis = (0..3)
                .max_by(|&i, &j| axes[i].abs().total_cmp(&axes[j].abs()))
                .unwrap_or(0);
            2 * axis as u32 + u32::from(axes[axis] > 0.0)
        })
        .collect()
}

/// The top front edge of the cube with `fine` strips filleted at [`RADIUS`] with the switch on.
fn the_top_front_edge(fine: usize, with_regions: bool) -> (BandGapResult, Option<String>) {
    let (positions, triangles, n) = fine_front_cube(fine);
    let regions = faces(&positions, &triangles);
    let feature = blended_back(&positions, &triangles, n);
    set_use_rolling_ball(true);
    set_last_route(None);
    let result = build_pick_with_side_regions(
        &positions,
        &triangles,
        &[vec![1, n + 1]],
        &[false],
        Some(feature),
        RADIUS,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
        with_regions.then_some(&regions[..]),
    )
    .expect("the pick's arguments are good");
    let route = last_route();
    reset_to_the_default();
    (result, route)
}

/// The cube is a closed solid wound outward, the fixture's premise.
#[test]
fn the_fine_front_cube_is_a_solid() {
    for fine in [0, 20, 60] {
        let (positions, triangles, _) = fine_front_cube(fine);
        let volume: f64 = triangles
            .iter()
            .map(|&(a, b, c)| {
                positions[a as usize].dot(positions[b as usize].cross(positions[c as usize]))
            })
            .sum::<f64>()
            / 6.0;
        assert!((volume - 8.0).abs() < 1e-12, "{fine}: {volume}");
    }
}

/// With the caller's regions or without them, coarse or with the front cut into 20 or 60 fine
/// strips, the ball rolls in the square corner with each contact the radius from the edge: the
/// division of the joined surface keeps the whole front on side B however finely it is cut.
#[test]
fn a_finely_cut_side_keeps_its_own_surface_where_the_floods_join() {
    for fine in [0, 20, 60] {
        for with_regions in [true, false] {
            let (result, route) = the_top_front_edge(fine, with_regions);
            let what = format!("{fine} strips, regions {with_regions}: {route:?}");
            assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE), "{what}");
            assert_eq!(result.refusal, BandGapRefusal::None, "{what}");
            for setback in &result.chains[0].setbacks {
                assert!((setback - RADIUS).abs() < 1e-8, "{what}: {setback}");
            }
        }
    }
}
