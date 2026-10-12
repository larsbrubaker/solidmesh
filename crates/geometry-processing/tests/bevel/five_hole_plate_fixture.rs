//! Port of the test fixture `LevelLine/FiveHolePlateFixture.cs`: a plate whose top face carries a
//! ROW OF FIVE square through-holes, the pick being that face's whole rim - the outer loop plus one
//! chain per hole, six chains. Two of the holes stand 0.6 from the plate's edge, too near for
//! `SIZE`; the other three have room to spare.

#![allow(dead_code)]

use std::collections::HashMap;

use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandProfile, RoomReading};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChains;
use crate::several_chains_fixtures;

/// C# `FoldDegrees`: the several-chain fold threshold, so a 90 degree corner is a feature edge.
pub const FOLD_DEGREES: f64 = several_chains_fixtures::FOLD_DEGREES;

/// C# `Size`: over the 0.3 the two end holes' shared gap buys, under the 0.6 either side of that
/// gap buys alone, and well under everything the other three holes have.
pub const SIZE: f64 = 0.5;

/// C# `Thickness`: thick enough that no chain's own wall bounds it at `SIZE`.
const THICKNESS: f64 = 3.0;

/// C# `Xs`: the plate's two edges and each hole's two, the end holes 0.6 inside the edge and the
/// middle three at least 3 from their neighbours.
const XS: [f64; 12] = [
    -12.0, -11.4, -9.4, -6.0, -4.0, -1.0, 1.0, 4.0, 6.0, 9.4, 11.4, 12.0,
];

/// C# `Ys`: the plate's two and the holes' two, every hole spanning y in [-1, 1].
const YS: [f64; 4] = [-4.0, -1.0, 1.0, 4.0];

/// C# `HoleColumns`: the grid columns a hole stands in, left to right.
const HOLE_COLUMNS: [usize; 5] = [1, 3, 5, 7, 9];

/// C# `HoleRow`: the row every hole stands in.
const HOLE_ROW: usize = 1;

/// C# `TooNearTheEdge`: which chains of `pick` cannot be built at `SIZE` - the two end holes.
pub const TOO_NEAR_THE_EDGE: [i32; 2] = [1, 5];

/// C# `TooNearTheEdgeWhenHolesComeFirst`: the same two holes when the holes are given FIRST.
pub const TOO_NEAR_THE_EDGE_WHEN_HOLES_COME_FIRST: [i32; 2] = [0, 4];

/// The welding vertex list `Pick`'s local `At` and `Quad` build into, in insertion order.
struct Builder {
    positions: Vec<Vector3>,
    by_position: HashMap<(i64, i64, i64), i32>,
    indices: Vec<(i32, i32, i32)>,
}

impl Builder {
    fn at(&mut self, x: f64, y: f64, z: f64) -> i32 {
        // The grid's lines are exact in binary to a nanometre, so rounding welds the pieces that
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

/// C# `Pick()` with its default arguments: `holes_first` and `moved` both false.
pub fn pick() -> SolidWithChains {
    pick_with(false, false)
}

/// C# `Pick`: the plate and the six chains of its top face's rim - the outer loop first, then the
/// five holes left to right, unless `holes_first` asks for the holes before the outer loop.
pub fn pick_with(holes_first: bool, moved: bool) -> SolidWithChains {
    let mut g = Builder {
        positions: Vec::new(),
        by_position: HashMap::new(),
        indices: Vec::new(),
    };

    let is_hole_cell =
        |column: usize, row: usize| row == HOLE_ROW && HOLE_COLUMNS.contains(&column);

    for column in 0..XS.len() - 1 {
        for row in 0..YS.len() - 1 {
            if is_hole_cell(column, row) {
                continue;
            }

            let (x0, x1, y0, y1) = (XS[column], XS[column + 1], YS[row], YS[row + 1]);
            let q = [
                g.at(x0, y0, 0.0),
                g.at(x1, y0, 0.0),
                g.at(x1, y1, 0.0),
                g.at(x0, y1, 0.0),
            ];
            g.quad(q[0], q[1], q[2], q[3]);
            let q = [
                g.at(x0, y0, -THICKNESS),
                g.at(x0, y1, -THICKNESS),
                g.at(x1, y1, -THICKNESS),
                g.at(x1, y0, -THICKNESS),
            ];
            g.quad(q[0], q[1], q[2], q[3]);
        }
    }

    // The plate's four sides, one quad per grid interval so the top and bottom rings meet them.
    for column in 0..XS.len() - 1 {
        let (x0, x1, low, high) = (XS[column], XS[column + 1], YS[0], YS[YS.len() - 1]);
        let q = [
            g.at(x0, low, -THICKNESS),
            g.at(x1, low, -THICKNESS),
            g.at(x1, low, 0.0),
            g.at(x0, low, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
        let q = [
            g.at(x1, high, -THICKNESS),
            g.at(x0, high, -THICKNESS),
            g.at(x0, high, 0.0),
            g.at(x1, high, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
    }

    for row in 0..YS.len() - 1 {
        let (y0, y1, left, right) = (YS[row], YS[row + 1], XS[0], XS[XS.len() - 1]);
        let q = [
            g.at(left, y1, -THICKNESS),
            g.at(left, y0, -THICKNESS),
            g.at(left, y0, 0.0),
            g.at(left, y1, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
        let q = [
            g.at(right, y0, -THICKNESS),
            g.at(right, y1, -THICKNESS),
            g.at(right, y1, 0.0),
            g.at(right, y0, 0.0),
        ];
        g.quad(q[0], q[1], q[2], q[3]);
    }

    let mut holes: Vec<Vec<i32>> = Vec::new();
    for &column in &HOLE_COLUMNS {
        let (x0, x1, y0, y1) = (XS[column], XS[column + 1], YS[HOLE_ROW], YS[HOLE_ROW + 1]);
        let plan = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];

        // A hole's wall faces INTO the hole, so its quads run the plan BACKWARDS - the mirror of the
        // plate's own sides, which run forwards and face out.
        for i in 0..plan.len() {
            let from = plan[(i + 1) % plan.len()];
            let to = plan[i];
            let q = [
                g.at(from.0, from.1, -THICKNESS),
                g.at(to.0, to.1, -THICKNESS),
                g.at(to.0, to.1, 0.0),
                g.at(from.0, from.1, 0.0),
            ];
            g.quad(q[0], q[1], q[2], q[3]);
        }

        holes.push(plan.iter().map(|p| g.at(p.0, p.1, 0.0)).collect());
    }

    let outer: Vec<i32> = outer_loop()
        .into_iter()
        .map(|p| g.at(p.0, p.1, 0.0))
        .collect();

    let placed = if moved {
        let motion = several_chains_fixtures::motion();
        g.positions.iter().map(|p| p.transform(motion)).collect()
    } else {
        g.positions
    };

    let mut chains = Vec::new();
    if holes_first {
        chains.extend(holes);
        chains.push(outer);
    } else {
        chains.push(outer);
        chains.extend(holes);
    }

    (placed, g.indices, chains)
}

/// C# `OuterLoop`: the plate's outer boundary counter-clockwise seen from +z, carrying a vertex at
/// every grid line that meets it so the rim's edges are exactly the top face's cell edges.
fn outer_loop() -> Vec<(f64, f64)> {
    let (low, high, left, right) = (YS[0], YS[YS.len() - 1], XS[0], XS[XS.len() - 1]);
    let mut out = Vec::new();
    for &x in &XS[..XS.len() - 1] {
        out.push((x, low));
    }

    for &y in &YS[..YS.len() - 1] {
        out.push((right, y));
    }

    for i in (1..XS.len()).rev() {
        out.push((XS[i], high));
    }

    for i in (1..YS.len()).rev() {
        out.push((left, YS[i]));
    }

    out
}

/// C# `FeatureEdges`: the pick's feature edges, read at the several-chain fold threshold.
pub fn feature_edges(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

/// C# `Read`: one room reading of the pick, optionally asked to name every candidate under a size
/// (C#'s default is `NaN`, naming none).
pub fn read(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    name_every_candidate_under: f64,
) -> RoomReading {
    let closed = vec![true; chains.len()];
    band_gap_reading::read_room_of_pick(
        positions,
        indices,
        chains,
        &closed,
        Some(feature_edges(positions, indices)),
        BandProfile::Chamfer,
        1e-9,
        None,
        name_every_candidate_under,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads")
}
