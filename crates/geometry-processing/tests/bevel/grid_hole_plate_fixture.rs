//! A plate whose top face carries an n x n GRID of square through-holes, the pick being that face's
//! whole rim - the outer loop plus one chain per hole, so n = 2, 4, 6 give 5, 17 and 37 chains. The
//! scalable stand-in for Many Holes, built as one rectilinear grid of cells so nothing is
//! hand-triangulated and no edge carries a T-vertex.

#![allow(dead_code)]

use std::collections::{HashMap, HashSet};

use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChains;
use crate::several_chains_fixtures;

/// The several-chain fold threshold, so a 90 degree corner is a feature edge.
pub const FOLD_DEGREES: f64 = several_chains_fixtures::FOLD_DEGREES;

/// Well under everything: a hole's nearest neighbour is 2.0 away, shared between two bands.
pub const SIZE: f64 = 0.5;

/// Thick enough that no chain's own wall bounds it at [`SIZE`].
const THICKNESS: f64 = 3.0;

/// A hole is exactly one cell, and this is that cell's width.
const HOLE: f64 = 2.0;

/// The default width of a cell of the FACE.
pub const DEFAULT_SPAN: f64 = 2.0;

/// How many cells across the plate is for a grid of `holes` holes per axis.
pub fn cells_across(holes: i32) -> i32 {
    (3 * holes) + 1
}

/// The chain count of [`pick`]: the outer loop plus one per hole.
pub fn chain_count(holes: i32) -> i32 {
    1 + (holes * holes)
}

/// The pick at the default span.
pub fn pick(holes: i32) -> SolidWithChains {
    pick_with_span(holes, DEFAULT_SPAN)
}

/// The plate and the chains of its top face's rim, the outer loop first,
/// then the holes in row-major order.
pub fn pick_with_span(holes: i32, span: f64) -> SolidWithChains {
    assert!(holes >= 1, "holes out of range");

    let across = cells_across(holes) as usize;
    let hole_lines: Vec<usize> = (0..holes as usize).map(|i| (3 * i) + 1).collect();
    let mut is_hole = vec![false; across];
    for &line in &hole_lines {
        is_hole[line] = true;
    }

    // The holes stay the same size whatever the face's cells do, so the grid's lines are a running
    // sum of two widths rather than a multiple of one.
    let mut lines = vec![0.0; across + 1];
    for i in 0..across {
        lines[i + 1] = lines[i] + if is_hole[i] { HOLE } else { span };
    }

    let mut positions: Vec<Vector3> = Vec::new();
    let mut by_position: HashMap<(i64, i64, i64), i32> = HashMap::new();
    let mut indices: Vec<(i32, i32, i32)> = Vec::new();

    // The grid's lines are exact in binary, so rounding welds the pieces that meet and never welds
    // two that do not.
    let mut at = |x: f64, y: f64, z: f64| -> i32 {
        let key = (
            (x * 1e6).round_ties_even() as i64,
            (y * 1e6).round_ties_even() as i64,
            (z * 1e6).round_ties_even() as i64,
        );
        *by_position.entry(key).or_insert_with(|| {
            positions.push(Vector3::new(x, y, z));
            positions.len() as i32 - 1
        })
    };

    let quad = |indices: &mut Vec<(i32, i32, i32)>, a: i32, b: i32, c: i32, d: i32| {
        indices.push((a, b, c));
        indices.push((a, c, d));
    };

    // Every third cell from the second is a hole.
    let hole_cells: HashSet<(usize, usize)> = hole_lines
        .iter()
        .flat_map(|&column| hole_lines.iter().map(move |&row| (column, row)))
        .collect();

    for column in 0..across {
        for row in 0..across {
            if hole_cells.contains(&(column, row)) {
                continue;
            }

            let (x0, x1, y0, y1) = (lines[column], lines[column + 1], lines[row], lines[row + 1]);
            let (a, b, c, d) = (
                at(x0, y0, 0.0),
                at(x1, y0, 0.0),
                at(x1, y1, 0.0),
                at(x0, y1, 0.0),
            );
            quad(&mut indices, a, b, c, d);
            let t = -THICKNESS;
            let (a, b, c, d) = (at(x0, y0, t), at(x0, y1, t), at(x1, y1, t), at(x1, y0, t));
            quad(&mut indices, a, b, c, d);
        }
    }

    // The plate's four sides, one quad per grid interval so the top and bottom rings meet them.
    let (low, high) = (lines[0], lines[across]);
    let t = -THICKNESS;
    for i in 0..across {
        let (a, b) = (lines[i], lines[i + 1]);
        let q = (
            at(a, low, t),
            at(b, low, t),
            at(b, low, 0.0),
            at(a, low, 0.0),
        );
        quad(&mut indices, q.0, q.1, q.2, q.3);
        let q = (
            at(b, high, t),
            at(a, high, t),
            at(a, high, 0.0),
            at(b, high, 0.0),
        );
        quad(&mut indices, q.0, q.1, q.2, q.3);
        let q = (
            at(low, b, t),
            at(low, a, t),
            at(low, a, 0.0),
            at(low, b, 0.0),
        );
        quad(&mut indices, q.0, q.1, q.2, q.3);
        let q = (
            at(high, a, t),
            at(high, b, t),
            at(high, b, 0.0),
            at(high, a, 0.0),
        );
        quad(&mut indices, q.0, q.1, q.2, q.3);
    }

    let mut chains: Vec<Vec<i32>> = vec![outer_loop(&lines, across)
        .into_iter()
        .map(|(x, y)| at(x, y, 0.0))
        .collect()];

    for &row in &hole_lines {
        for &column in &hole_lines {
            let (x0, x1, y0, y1) = (lines[column], lines[column + 1], lines[row], lines[row + 1]);
            let plan = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];

            // A hole's wall faces INTO the hole, so its quads run the plan BACKWARDS.
            for i in 0..plan.len() {
                let from = plan[(i + 1) % plan.len()];
                let to = plan[i];
                let q = (
                    at(from.0, from.1, t),
                    at(to.0, to.1, t),
                    at(to.0, to.1, 0.0),
                    at(from.0, from.1, 0.0),
                );
                quad(&mut indices, q.0, q.1, q.2, q.3);
            }

            chains.push(plan.iter().map(|&(x, y)| at(x, y, 0.0)).collect());
        }
    }

    drop(at);
    (positions, indices, chains)
}

/// The pick's feature edges, read at the several-chain fold threshold.
pub fn feature_edges(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> FeatureEdgeTest {
    feature_edges_over(positions, indices, FOLD_DEGREES)
}

/// The plate's outer boundary counter-clockwise seen from +z, carrying a vertex at every grid line
/// that meets it so the rim's edges are exactly the top face's cell edges.
fn outer_loop(lines: &[f64], across: usize) -> Vec<(f64, f64)> {
    let (low, high) = (lines[0], lines[across]);
    let mut points = Vec::new();
    for &x in &lines[..across] {
        points.push((x, low));
    }

    for &y in &lines[..across] {
        points.push((high, y));
    }

    for i in (1..=across).rev() {
        points.push((lines[i], high));
    }

    for i in (1..=across).rev() {
        points.push((low, lines[i]));
    }

    points
}
