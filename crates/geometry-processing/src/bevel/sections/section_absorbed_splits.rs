//! A run's split vertices read as POINTS OF ITS STRIPS rather than as stations of their own
//! (`Bevel/Sections/SectionAbsorbedSplits.cs`): the per-gap, per-side table
//! ([`SectionAbsorbedSplits`]) and one absorbed point in it ([`AbsorbedPoint`]), with its reading
//! off a run ([`SectionAbsorbedSplits::from`]); the corner pass (`BehindTheCorners`) lands with a
//! later package as a further `impl` block.
//!
//! A split vertex stands on the straight chain edge between two unsplit stations A and B with the
//! same two surfaces on its sides the whole way, so the strip between A and B is exactly the strip
//! an unsplit mesh would get, and the split vertex is merely a point of that strip's two boundary
//! rows - a strip never subdivided there cannot reverse there.

use super::section_split_blend::SplitPlacement;
use super::BevelSection;

/// C# `SectionAbsorbedSplits`: for each gap between two consecutive unsplit stations, and for
/// each side, the boundary points the split vertices in that gap contribute to that strip's
/// boundary row, in chain order.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SectionAbsorbedSplits {
    /// Per gap, the absorbed points on the LEFT side, in chain order.
    pub on_the_left: Vec<Vec<AbsorbedPoint>>,
    /// Per gap, the absorbed points on the RIGHT side, in chain order.
    pub on_the_right: Vec<Vec<AbsorbedPoint>>,
    /// The run's unsplit station indices in chain order. Gap `g` runs from `stations[g]` to
    /// `stations[g + 1]` - and, on a CLOSED run only, the last gap wraps back to `stations[0]`.
    pub stations: Vec<i32>,
    /// Whether the run this was read off closes on itself.
    pub is_closed: bool,
}

/// C# `SectionAbsorbedSplits.AbsorbedPoint`: one absorbed split vertex on one side of one gap -
/// where its boundary point already stands in the pool, which mesh chain vertex it replaces, and
/// how far along the gap it is.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AbsorbedPoint {
    /// The pool index of the boundary point itself, minted once by the builder.
    pub pool_index: i32,
    /// The mesh chain vertex this point stands at, whose corners the face rebuild substitutes.
    pub chain_vertex: i32,
    /// Where along the gap the split vertex stands - the blend's own parameter, which is also the
    /// order the absorbed points of one gap merge into that gap's boundary row.
    pub parameter: f64,
    /// Whether the point was placed ON the join between the two ends' boundary points - true on a
    /// walked side, where the point is a vertex of the boundary the smooth cut runs along.
    pub is_join_placed: bool,
}

impl AbsorbedPoint {
    /// C#'s `AbsorbedPoint` constructor.
    pub fn new(pool_index: i32, chain_vertex: i32, parameter: f64, is_join_placed: bool) -> Self {
        Self {
            pool_index,
            chain_vertex,
            parameter,
            is_join_placed,
        }
    }
}

/// The table's constructor, its reading off a run and its questions (C# constructor, `From`,
/// `Count`, `GapCount`, `LeftOf`, `RightOf`, `PointAt`, `Next`). The corner pass
/// (`BehindTheCorners`) lands with a later package.
impl SectionAbsorbedSplits {
    /// C#'s private constructor.
    pub fn new(
        stations: Vec<i32>,
        is_closed: bool,
        on_the_left: Vec<Vec<AbsorbedPoint>>,
        on_the_right: Vec<Vec<AbsorbedPoint>>,
    ) -> Self {
        Self {
            on_the_left,
            on_the_right,
            stations,
            is_closed,
        }
    }

    /// C# `Count`: how many unsplit stations there are.
    pub fn count(&self) -> i32 {
        self.stations.len() as i32
    }

    /// C# `GapCount`: how many gaps there are - one per unsplit station on a closed run, one FEWER on
    /// an open one, whose first and last stations are its two end sections.
    pub fn gap_count(&self) -> i32 {
        if self.is_closed {
            self.count()
        } else {
            self.count() - 1
        }
    }

    /// C# `LeftOf`: the points absorbed into gap `gap`'s LEFT boundary row, in chain order.
    pub fn left_of(&self, gap: i32) -> &[AbsorbedPoint] {
        &self.on_the_left[gap as usize]
    }

    /// C# `RightOf`: the points absorbed into gap `gap`'s RIGHT boundary row, in chain order.
    pub fn right_of(&self, gap: i32) -> &[AbsorbedPoint] {
        &self.on_the_right[gap as usize]
    }

    /// C# `From`: reads the finished sections and the blend's placements as gaps and absorbed points,
    /// or `None` where nothing is split (in which case every pass downstream keeps today's plain
    /// path).
    ///
    /// An OPEN run's gap list does NOT carry the wrap-around gap from its last station back to its
    /// first: those two stations are its end sections. `swallowed_left` / `swallowed_right` are, per
    /// station, the pool index its boundary takes where a corner swallowed it, or -1 - the reading
    /// `BehindTheCorners` makes; `None` where no corner pass ran.
    pub fn from(
        sections: &[BevelSection],
        placed: &[SplitPlacement],
        is_closed: bool,
        swallowed_left: Option<&[i32]>,
        swallowed_right: Option<&[i32]>,
    ) -> Option<Self> {
        if sections.len() != placed.len() {
            return None;
        }

        let stations: Vec<i32> = (0..sections.len())
            .filter(|&i| !placed[i].is_split() && !swallowed_left.is_some_and(|left| left[i] >= 0))
            .map(|i| i as i32)
            .collect();

        if stations.len() == sections.len() || stations.len() < 3 {
            // Nothing split, or so much split that there is no ring of unsplit stations left to hang
            // the strips off - either way this reading has nothing to say and the caller keeps
            // today's run.
            return None;
        }

        let gaps = if is_closed {
            stations.len()
        } else {
            stations.len() - 1
        };
        let count = sections.len() as i32;

        let mut left = Vec::with_capacity(gaps);
        let mut right = Vec::with_capacity(gaps);

        for g in 0..gaps {
            let mut left_row = Vec::new();
            let mut right_row = Vec::new();

            let from = stations[g];
            let to = stations[(g + 1) % stations.len()];

            let mut at = Self::next(from, count);
            while at != to {
                let section = &sections[at as usize];
                let index = at as usize;

                // A corner-swallowed station's point on the swallowed side is the CORNER's own point, by
                // index - the loft and the rebuild then both see one point where the fold would have
                // been.
                let left_at = match swallowed_left {
                    Some(swallowed) if swallowed[index] >= 0 => swallowed[index],
                    _ => section.left_boundary,
                };
                let right_at = match swallowed_right {
                    Some(swallowed) if swallowed[index] >= 0 => swallowed[index],
                    _ => section.right_boundary,
                };

                left_row.push(AbsorbedPoint::new(
                    left_at,
                    section.chain_vertex,
                    placed[index].parameter,
                    section.left_boundary_is_join_placed,
                ));
                right_row.push(AbsorbedPoint::new(
                    right_at,
                    section.chain_vertex,
                    placed[index].parameter,
                    section.right_boundary_is_join_placed,
                ));

                at = Self::next(at, count);
            }

            left.push(left_row);
            right.push(right_row);
        }

        Some(Self::new(stations, is_closed, left, right))
    }

    /// C# `PointAt`: the pool index the station at `chain_vertex` takes on one side where it was
    /// absorbed, or -1 where that vertex is not an absorbed station at all.
    pub fn point_at(&self, chain_vertex: i32, is_left: bool) -> i32 {
        let rows = if is_left {
            &self.on_the_left
        } else {
            &self.on_the_right
        };

        rows.iter()
            .flatten()
            .find(|point| point.chain_vertex == chain_vertex)
            .map_or(-1, |point| point.pool_index)
    }

    /// C# `Next`: the station after `at` round a run of `count`.
    fn next(at: i32, count: i32) -> i32 {
        (at + 1) % count
    }
}
