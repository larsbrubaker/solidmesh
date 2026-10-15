//! A loop re-read as a cycle running in increasing chain parameter ([`ZipRail`]), with its count,
//! its canonical reading at tie vertices and its
//! reading past the wrap, whether a step is a jump ([`ZipRail::is_a_jump`]) and the rail re-cut to
//! start nearest the other's start ([`ZipRail::start_nearest`]).

use vector_math::ieee_math;

use super::{gap_fill, LevelLineMesh};

/// A loop re-read as a cycle running in increasing chain parameter - one of the two
/// rails `GapFill`'s per-chain zip merges. Its properties are set once, as public fields.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ZipRail {
    /// The rail's mesh vertices, in increasing parameter.
    pub vertices: Vec<i32>,
    /// The chain parameter each vertex reads.
    pub parameters: Vec<f64>,
    /// True when the rail runs against its loop's walk - so its fill edges run WITH the rail.
    pub reversed: bool,
}

impl ZipRail {
    /// How many vertices the rail has.
    pub fn count(&self) -> i32 {
        self.vertices.len() as i32
    }

    /// The rail with each vertex read at the low end of the span it stands for: a tie vertex's
    /// incoming leg's reading, any other vertex's own. A vertex on a sharp joint's inner bisector
    /// is on both legs, and which of the two its own reading names is float noise; the merge, its
    /// jump test and the rail start all read this one value, so no choice they make turns on the
    /// noise. The incoming leg is the convention the fillet arc takes for a reading on a joint.
    pub fn canonical(rail: &ZipRail, spans: &[(f64, f64)]) -> ZipRail {
        ZipRail {
            vertices: rail.vertices.clone(),
            parameters: spans.iter().map(|span| span.0).collect(),
            reversed: rail.reversed,
        }
    }

    /// The parameter of rail position `advance`, read on past the wrap: a position at or past the
    /// count is the first vertex again, a chain `length` further on; `offset` shifts every reading.
    pub fn unwrapped(rail: &ZipRail, advance: i32, length: f64, offset: f64) -> f64 {
        if advance < rail.count() {
            rail.parameters[advance as usize] + offset
        } else {
            rail.parameters[0] + offset + length
        }
    }
}

impl ZipRail {
    /// Whether the rail's step from vertex `at` to the next climbs more chain
    /// parameter than twice its own length (plus a tie): a jump is the inside of a concave turn, or a
    /// small region between legs - Around Center's inner diamond is four rail vertices each reading
    /// a different leg. Across a jump the merge takes the shorter of the two spokes it could make.
    pub fn is_a_jump(mesh: &LevelLineMesh, rail: &ZipRail, at: i32, length: f64) -> bool {
        let from = rail.vertices[at as usize];
        let to = rail.vertices[((at + 1) % rail.count()) as usize];
        let climb =
            Self::unwrapped(rail, at + 1, length, 0.0) - Self::unwrapped(rail, at, length, 0.0);
        climb
            > 2.0 * (mesh.position(to) - mesh.position(from)).length()
                + (gap_fill::PARAMETER_TIE_RELATIVE * length)
    }

    /// The rail rotated to begin at its vertex nearest `other_start` in
    /// chain parameter (round the wrap), when its own start is further than `margin` beyond that -
    /// else the rail as it was, so a rail that begins beside the other is never re-cut. Each vertex
    /// is measured by the whole span it stands for (`spans`), as the merge's jump rule is, and the
    /// rail is re-cut only at a jump, read canonically and on either side of the new start.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn start_nearest(
        mesh: &LevelLineMesh,
        rail: &ZipRail,
        spans: &[(f64, f64)],
        other_start: f64,
        length: f64,
        margin: f64,
    ) -> ZipRail {
        let apart = |p: f64| {
            let d = (p - other_start).abs() % length;
            ieee_math::min_f64(d, length - d)
        };
        let span_apart = |i: usize| {
            let (low, high) = spans[i];
            if high - low >= length {
                return 0.0;
            }

            // The other start inside the span, a lap either way, is no distance at all.
            let mut lapped = other_start + (((low - other_start) / length).floor() * length);
            if lapped < low {
                lapped += length;
            }

            if lapped <= high {
                0.0
            } else {
                ieee_math::min_f64(apart(low), apart(high))
            }
        };

        let count = rail.count();
        let mut best = 0usize;
        for i in 1..count as usize {
            if span_apart(i) < span_apart(best) {
                best = i;
            }
        }

        // The rail is re-cut only at a jump, read canonically and on either side of the new start,
        // so a tie vertex whose noisy reading named the other leg is judged the same either way.
        let read = Self::canonical(rail, spans);
        let best_i = best as i32;
        if best == 0
            || span_apart(0) - span_apart(best) <= margin
            || !(Self::is_a_jump(mesh, &read, (best_i + count - 1) % count, length)
                || Self::is_a_jump(mesh, &read, best_i, length))
        {
            return rail.clone();
        }

        // The vertices before the new start move to the end; read in one climbing frame, the new
        // start's stretch is a lap earlier.
        let n = count as usize;
        let mut vertices = Vec::with_capacity(n);
        let mut parameters = Vec::with_capacity(n);
        for i in 0..n {
            let from = (best + i) % n;
            vertices.push(rail.vertices[from]);
            parameters.push(if from >= best {
                rail.parameters[from] - length
            } else {
                rail.parameters[from]
            });
        }

        ZipRail {
            vertices,
            parameters,
            reversed: rail.reversed,
        }
    }
}
