//! Port of the test fixture `LevelLine/HourglassBesideSquareFixture.cs`: a plate carrying TWO
//! bosses - one an hourglass in plan, one a square - and the pick is both bosses' rims at the top
//! of their walls. One chain of the pick pinches WELL BELOW the room the pick reads, so the size
//! refusal can only name a size a probe actually built at.

#![allow(dead_code)]

use std::collections::HashMap;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile, RoomReading};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChains;
use crate::several_chains_fixtures;

/// C# `FoldDegrees`: a fold threshold that reads a 90 degree corner as a feature.
pub const FOLD_DEGREES: f64 = several_chains_fixtures::FOLD_DEGREES;

/// C# `NeckHalfWidth`: half the width of the hourglass's neck - the size at which its face is cut
/// in two.
pub const NECK_HALF_WIDTH: f64 = 1.0;

/// C# `BossHeight`: the plate's top is at z = 0, the bosses stand this high above it.
const BOSS_HEIGHT: f64 = 3.0;

const PLATE_THICKNESS: f64 = 1.0;

/// C# `Xs`: the plate's two edges, the hourglass's four (lobe, neck, neck, lobe) and the square
/// boss's two.
const XS: [f64; 8] = [-14.0, -11.0, -7.0, -5.0, -1.0, 5.0, 9.0, 14.0];

/// C# `Ys`: the plate's two, the hourglass's lobe and neck lines, the square's two.
const YS: [f64; 8] = [-6.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 6.0];

/// C# `HourglassCells`: both lobes over the full height of their columns, and the neck between.
const HOURGLASS_CELLS: [(usize, usize); 11] = [
    (1, 1),
    (1, 2),
    (1, 3),
    (1, 4),
    (1, 5),
    (2, 3),
    (3, 1),
    (3, 2),
    (3, 3),
    (3, 4),
    (3, 5),
];

/// C# `SquareCells`: the square boss, 4 x 4.
const SQUARE_CELLS: [(usize, usize); 3] = [(5, 2), (5, 3), (5, 4)];

/// C# `HourglassCorners`: the hourglass's plan, counter-clockwise seen from +z.
const HOURGLASS_CORNERS: [(f64, f64); 12] = [
    (-11.0, -3.0),
    (-7.0, -3.0),
    (-7.0, -1.0),
    (-5.0, -1.0),
    (-5.0, -3.0),
    (-1.0, -3.0),
    (-1.0, 3.0),
    (-5.0, 3.0),
    (-5.0, 1.0),
    (-7.0, 1.0),
    (-7.0, 3.0),
    (-11.0, 3.0),
];

/// C# `SquareCorners`: the square boss's plan, counter-clockwise seen from +z.
const SQUARE_CORNERS: [(f64, f64); 4] = [(5.0, -2.0), (9.0, -2.0), (9.0, 2.0), (5.0, 2.0)];

/// The welding vertex list `Pick`'s local `At` and `Quad` build into, in insertion order.
struct Builder {
    positions: Vec<Vector3>,
    by_position: HashMap<(i64, i64, i64), i32>,
    indices: Vec<(i32, i32, i32)>,
}

impl Builder {
    fn at(&mut self, x: f64, y: f64, z: f64) -> i32 {
        // The grid's lines are exact in binary, so rounding to a nanometre welds the pieces that
        // meet and never welds two that do not.
        let key = (
            (x * 1e6).round_ties_even() as i64,
            (y * 1e6).round_ties_even() as i64,
            (z * 1e6).round_ties_even() as i64,
        );
        if let Some(&index) = self.by_position.get(&key) {
            return index;
        }

        let index = self.positions.len() as i32;
        self.positions.push(Vector3::new(x, y, z));
        self.by_position.insert(key, index);
        index
    }

    fn quad(&mut self, a: i32, b: i32, c: i32, d: i32) {
        self.indices.push((a, b, c));
        self.indices.push((a, c, d));
    }
}

/// C# `Pick()` with its default arguments: `square_first` and `moved` both false.
pub fn pick() -> SolidWithChains {
    pick_with(false, false)
}

/// C# `Pick`: the plate with both bosses on it, and the two bosses' top rims as one pick, the
/// hourglass first unless `square_first` asks for the other order.
pub fn pick_with(square_first: bool, moved: bool) -> SolidWithChains {
    let mut g = Builder {
        positions: Vec::new(),
        by_position: HashMap::new(),
        indices: Vec::new(),
    };

    let is_boss_cell = |column: usize, row: usize| {
        HOURGLASS_CELLS.contains(&(column, row)) || SQUARE_CELLS.contains(&(column, row))
    };

    for column in 0..XS.len() - 1 {
        for row in 0..YS.len() - 1 {
            let (x0, x1, y0, y1) = (XS[column], XS[column + 1], YS[row], YS[row + 1]);

            // The plate's top face, wound +z, everywhere a boss does not stand.
            if !is_boss_cell(column, row) {
                let q = [
                    g.at(x0, y0, 0.0),
                    g.at(x1, y0, 0.0),
                    g.at(x1, y1, 0.0),
                    g.at(x0, y1, 0.0),
                ];
                g.quad(q[0], q[1], q[2], q[3]);
            }

            // The plate's bottom face, the same grid wound -z so the two edges of every cell line up.
            let q = [
                g.at(x0, y0, -PLATE_THICKNESS),
                g.at(x0, y1, -PLATE_THICKNESS),
                g.at(x1, y1, -PLATE_THICKNESS),
                g.at(x1, y0, -PLATE_THICKNESS),
            ];
            g.quad(q[0], q[1], q[2], q[3]);
        }
    }

    // The plate's four sides, one quad per grid interval so the top and bottom rings meet them.
    for column in 0..XS.len() - 1 {
        let (x0, x1, low, high) = (XS[column], XS[column + 1], YS[0], YS[YS.len() - 1]);
        let q = [
            g.at(x0, low, -PLATE_THICKNESS),
            g.at(x1, low, -PLATE_THICKNESS),
            g.at(x1, low, 0.0),
            g.at(x0, low, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
        let q = [
            g.at(x1, high, -PLATE_THICKNESS),
            g.at(x0, high, -PLATE_THICKNESS),
            g.at(x0, high, 0.0),
            g.at(x1, high, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
    }

    for row in 0..YS.len() - 1 {
        let (y0, y1, left, right) = (YS[row], YS[row + 1], XS[0], XS[XS.len() - 1]);
        let q = [
            g.at(left, y1, -PLATE_THICKNESS),
            g.at(left, y0, -PLATE_THICKNESS),
            g.at(left, y0, 0.0),
            g.at(left, y1, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
        let q = [
            g.at(right, y0, -PLATE_THICKNESS),
            g.at(right, y1, -PLATE_THICKNESS),
            g.at(right, y1, 0.0),
            g.at(right, y0, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
    }

    let hourglass = boss(&HOURGLASS_CORNERS, &HOURGLASS_CELLS, &mut g);
    let square = boss(&SQUARE_CORNERS, &SQUARE_CELLS, &mut g);

    let placed = if moved {
        let motion = several_chains_fixtures::motion();
        g.positions.iter().map(|p| p.transform(motion)).collect()
    } else {
        g.positions
    };

    let chains = if square_first {
        vec![square, hourglass]
    } else {
        vec![hourglass, square]
    };

    (placed, g.indices, chains)
}

/// C# `HourglassChain`: which chain of the pick is the hourglass, in the order `pick` was asked for.
pub fn hourglass_chain(square_first: bool) -> i32 {
    if square_first {
        1
    } else {
        0
    }
}

/// C# `Boss`: one boss - its walls from the plate's top up to its face, and its face. Answers the
/// face's rim, counter-clockwise seen from +z.
fn boss(corners: &[(f64, f64)], cells: &[(usize, usize)], g: &mut Builder) -> Vec<i32> {
    let plan = subdivide_on_the_grid(corners);

    // The wall under each rim edge. The rim runs counter-clockwise with the boss on its left, so the
    // quad base-to-base then up faces OUT of the boss.
    for i in 0..plan.len() {
        let (x0, y0) = plan[i];
        let (x1, y1) = plan[(i + 1) % plan.len()];
        let q = [
            g.at(x0, y0, 0.0),
            g.at(x1, y1, 0.0),
            g.at(x1, y1, BOSS_HEIGHT),
            g.at(x0, y0, BOSS_HEIGHT),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
    }

    // The boss's face, the same grid cells the plate's top gave up, wound +z.
    for &(column, row) in cells {
        let (cx0, cx1, cy0, cy1) = (XS[column], XS[column + 1], YS[row], YS[row + 1]);
        let q = [
            g.at(cx0, cy0, BOSS_HEIGHT),
            g.at(cx1, cy0, BOSS_HEIGHT),
            g.at(cx1, cy1, BOSS_HEIGHT),
            g.at(cx0, cy1, BOSS_HEIGHT),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
    }

    plan.iter().map(|p| g.at(p.0, p.1, BOSS_HEIGHT)).collect()
}

/// C# `SubdivideOnTheGrid`: a rectilinear plan with a vertex wherever a grid line crosses it - what
/// keeps the wall's quads and the neighbouring cells edge for edge.
fn subdivide_on_the_grid(corners: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut plan = Vec::new();
    for i in 0..corners.len() {
        let from = corners[i];
        let to = corners[(i + 1) % corners.len()];
        plan.push(from);

        if from.0 == to.0 {
            let mut ys: Vec<f64> = YS
                .iter()
                .copied()
                .filter(|&y| (y - from.1) * (y - to.1) < 0.0)
                .collect();
            // C# OrderBy is a stable sort.
            ys.sort_by(|a, b| {
                ((a - from.1) * (to.1 - from.1))
                    .partial_cmp(&((b - from.1) * (to.1 - from.1)))
                    .unwrap()
            });
            for y in ys {
                plan.push((from.0, y));
            }
        } else {
            let mut xs: Vec<f64> = XS
                .iter()
                .copied()
                .filter(|&x| (x - from.0) * (x - to.0) < 0.0)
                .collect();
            xs.sort_by(|a, b| {
                ((a - from.0) * (to.0 - from.0))
                    .partial_cmp(&((b - from.0) * (to.0 - from.0)))
                    .unwrap()
            });
            for x in xs {
                plan.push((x, from.1));
            }
        }
    }

    plan
}

/// C# `FeatureEdges`: the pick's feature edges, read at the several-chain fold threshold.
pub fn feature_edges(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

/// C# `Read`: the whole pick's room, read on the input mesh.
pub fn read(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    profile: BandProfile,
) -> RoomReading {
    let closed = vec![true; chains.len()];
    band_gap_reading::read_room_of_pick(
        positions,
        indices,
        chains,
        &closed,
        Some(feature_edges(positions, indices)),
        profile,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads")
}

/// C# `Build`: a build of the whole pick at one size.
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
    profile: BandProfile,
) -> BandGapResult {
    let closed = vec![true; chains.len()];
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        &closed,
        Some(feature_edges(positions, indices)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name")
}
