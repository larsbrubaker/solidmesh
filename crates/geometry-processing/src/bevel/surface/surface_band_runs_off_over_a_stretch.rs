//! What one side of one chain edge runs off over PART of its own span: the stretch at one end of
//! the edge where the face it lands on has already died away under the setback, and the knife edge
//! it runs off there onto the one plane beyond.
//!
//! The partial half of the whole-span run-off. There the band takes the WHOLE of the region over
//! the edge's span; here the face runs out only over a stretch - the pyramidal recess's left wall,
//! 0.053 mm wide where the crease reaches the mouth and widening at 1.118 mm per millimetre, so a
//! 3 mm round eats the whole sliver over the first 1.887 mm and lands on the wall past that.
//!
//! Recorded rather than refused, and the rim it runs off charges no room: the old reading charged
//! that rim as a WALL and named a size of 0.0735 for a part that rounds at 3.

use std::collections::HashSet;
use std::sync::Arc;

use vector_math::Vector3;

use crate::topology::FeatureEdge;

/// A side's run-off over a stretch of its span. The rim set is shared, never mutated, so
/// [`SurfaceBandRunsOffOverAStretch::on`] copies no set.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceBandRunsOffOverAStretch {
    /// Which end of the chain edge's span the stretch is anchored at - true for the edge's own
    /// FROM. A stretch touching neither end, or both, is no part of this reading: the first is
    /// refused as `SurfaceBandRunsOffMidSpan` and the second is the whole-span run-off.
    pub at_the_start: bool,

    /// How far along the span the stretch reaches, measured from the end `at_the_start` names: the
    /// stretch is [0, until] at the start and [length - until, length] at the end.
    pub until: f64,

    /// The rim the band runs off over that stretch - the knife edge where the face it lands on ends
    /// and the plane beyond begins. Where a tessellated rim contributed more than one edge this is
    /// the first of them; every one of them is in `rims_it_runs_off`.
    pub knife_line: FeatureEdge,

    /// The outward normal of the one plane beyond that rim, which is the far face's own.
    pub plane_beyond: Vector3,

    /// A point on that plane.
    pub plane_beyond_at: Vector3,

    /// The mesh edges of the rim the stretch runs off, which are the ones the room reading must NOT
    /// charge as walls - the whole point of the reading is that the band goes over them rather
    /// than stopping at them. Only ever asked whether it contains an edge.
    pub rims_it_runs_off: Arc<HashSet<i32>>,

    /// Which plan of the pass this stretch belongs to, or -1 before it has been stamped.
    pub plan_index: i32,

    /// Which edge of that plan's chain it was read on, or -1.
    pub edge_index: i32,

    /// Which side of that edge ran off - 0 for the left region and 1 for the right, the order the
    /// room measure walks them in - or -1.
    pub side: i32,
}

impl SurfaceBandRunsOffOverAStretch {
    /// A run-off not yet stamped onto a band (`plan_index`,
    /// `edge_index` and `side` all -1).
    pub fn new(
        at_the_start: bool,
        until: f64,
        knife_line: FeatureEdge,
        plane_beyond: Vector3,
        plane_beyond_at: Vector3,
        rims_it_runs_off: Arc<HashSet<i32>>,
    ) -> Self {
        SurfaceBandRunsOffOverAStretch {
            at_the_start,
            until,
            knife_line,
            plane_beyond,
            plane_beyond_at,
            rims_it_runs_off,
            plan_index: -1,
            edge_index: -1,
            side: -1,
        }
    }

    /// The same stretch, told which band of the pass it was read on.
    pub fn on(&self, plan_index: i32, edge_index: i32, side: i32) -> Self {
        SurfaceBandRunsOffOverAStretch {
            plan_index,
            edge_index,
            side,
            ..self.clone()
        }
    }
}
