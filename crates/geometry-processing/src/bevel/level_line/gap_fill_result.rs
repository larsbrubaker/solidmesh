//! What the fill left behind ([`GapFillResult`]), with its derived reading
//! ([`GapFillResult::filled`]).

use std::cell::RefCell;
use std::rc::Rc;

use super::{GapFillRefusal, LevelLineMesh, LevelLineWorkPhase};

/// What the fill left behind. The `-1` defaults mean "no vertex, edge or chain named", which is
/// why `Default` is written out rather than derived.
#[derive(Debug, Clone)]
pub struct GapFillResult {
    /// True when an open chain's two ends were closed by END-FACE caps - the run stops square on
    /// its end faces rather than fading out. The app reads it to say, as the existing construction
    /// does, where a run stops square while its crease carries on past the pick.
    pub ends_capped_by_end_faces: bool,
    /// Why the fill refused, or [`GapFillRefusal::None`] when it went in.
    pub refusal: GapFillRefusal,
    /// The mesh vertex a rail-is-not-monotone refusal is about, else -1.
    pub offending_vertex: i32,
    /// The first edge a fill-would-not-close refusal found open, as the two mesh vertices it runs
    /// between; (-1, -1) otherwise.
    pub open_edge: (i32, i32),
    /// The gap's mesh, now with the fill laid into it - the same object the gap carried.
    pub mesh: Option<Rc<RefCell<LevelLineMesh>>>,
    /// The triangles the fill added, in the order it added them.
    pub added_triangles: Vec<i32>,
    /// The vertices the fill added - a fillet's interior arc points; none for a chamfer.
    pub added_vertices: Vec<i32>,
    /// Which chain of the pick a refusal is about, as an index into `BandGapResult::chains`, or -1
    /// when it is about none of them in particular. A pick of one chain names chain 0.
    pub offending_chain: i32,
    /// How many triangles each chain's own fill laid, in the pick's order. Empty on a refusal,
    /// which adds nothing at all.
    pub triangles_of_chain: Vec<i32>,
    /// How many vertices each chain's own fill added, in the pick's order.
    pub vertices_of_chain: Vec<i32>,
    /// What was running when the work budget ran out; [`LevelLineWorkPhase::None`] otherwise.
    pub phase: LevelLineWorkPhase,
    /// How many units of work the fill spent.
    pub work_spent: i64,
    /// How many units it was allowed.
    pub work_limit: i64,
    /// Why the rolling ball refused the band this fill was refused for
    /// (`BandGapResult::rolling_ball_refusal`), where a build of several passes - chains in turn,
    /// rounds first - stopped at a pass the ball refused; `None` otherwise.
    pub rolling_ball_refusal: Option<String>,
}

impl Default for GapFillResult {
    fn default() -> Self {
        GapFillResult {
            ends_capped_by_end_faces: false,
            refusal: GapFillRefusal::default(),
            offending_vertex: -1,
            open_edge: (-1, -1),
            mesh: None,
            added_triangles: Vec::new(),
            added_vertices: Vec::new(),
            offending_chain: -1,
            triangles_of_chain: Vec::new(),
            vertices_of_chain: Vec::new(),
            phase: LevelLineWorkPhase::default(),
            work_spent: 0,
            work_limit: 0,
            rolling_ball_refusal: None,
        }
    }
}

impl GapFillResult {
    /// True when the fill went in.
    pub fn filled(&self) -> bool {
        self.refusal == GapFillRefusal::None
    }
}
