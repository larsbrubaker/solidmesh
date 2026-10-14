//! Where a built surface bevel stopped SHORT of the part rather than short of its size, and the one
//! line that says so on the group's own row: [`read`] and the end gathering it is read with.
//!
//! This is the surface construction's half of what the boolean path says through
//! `bevel_chain_end_budget::held_back`, and it is owed for the same reason: a user picks one facet
//! of a tessellated rim, the cut is made, and it ends in a little wall part way round with nothing
//! anywhere saying why. The two constructions arrive there differently - the boolean end WANTED to
//! overshoot and was bounded to half its own segment; the surface end never overshoots at all,
//! because the crease carries on past the corner in an edge nobody picked and there is no face
//! beyond it to run out onto (`SurfaceChainEndReading`, `SurfaceFaceCarriesOn`) - so the sentence
//! is its own. What they share is the advice, which is the only thing a user can act on: read more
//! of the edge.
//!
//! Owed only to a FREE end. An end another pick's run also stands at is CONSUMED by the corner
//! reading between them - a junction blend (`SurfaceMixedJunction`), a ball corner
//! (`SurfaceCornerPatch`), a mitre with the edge beside it - and a consumed end does not stop
//! square at all: what closes it is the neighbouring bevel, not a wall of its own. Saying it there
//! would put the sentence on every row of a corner that came out exactly as asked, which is what
//! three mixed edges at one vertex of an L did.
//!
//! A WARNING and never a refusal. Nothing was given up: the bevel is the full size that was asked
//! for, every ring of it, and the wall at the end is the correct answer for the selection as made.
//! Read off the plans rather than off the built mesh because the plans are where the decision was
//! taken, and because the mesh cannot tell a wall the user chose from one they did not.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use vector_math::Vector3;

use super::SurfaceBevelPlan;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One line per plan whose free ends stop square, or `None` where none of them do (and where there
/// are no plans at all).
///
/// `plans` are the plans one pass built, in the order they were planned (the pass's list holds
/// holds no missing plan). `edge_count` is how much the GROUP being written to picked, which is the
/// other thing the user can change - the group's own count and not the chain's, because the row the
/// sentence lands on is the group's. `the_groups_mesh_edges` names which mesh edges that group
/// picked, keyed the way the plans carry them (`FeatureEdge::mesh_edge_index`); only a plan
/// carrying one of them has anything to say on that group's row, and `None` says every plan is the
/// group's, which is the whole of one pass again.
pub fn read(
    plans: Option<&[Rc<SurfaceBevelPlan>]>,
    edge_count: i32,
    the_groups_mesh_edges: Option<&HashSet<i32>>,
) -> Option<Vec<BevelAdvisory>> {
    let plans = plans?;
    let mut said: Option<Vec<BevelAdvisory>> = None;

    // Two runs can stop square at the SAME two corners - a region's rim opened at two junctions
    // comes back as two arcs between one pair of vertices - and the same sentence about the same
    // two points twice reads as two problems rather than one (bevel_chain_end_budget::held_back
    // says the same of one run's two ends). Keyed on the pair unordered, since the two arcs walk it
    // opposite ways. A list compared with `==` (component for component), since `Vector3` is no
    // `Eq`; a pass says a handful of these at most.
    let mut already_said: Vec<(Vector3, Vector3)> = Vec::new();

    // Which ends a CORNER closed, read the way the build's own end gathering reads them: by the
    // mesh's welded vertex index, an end another open plan's end stands at being one of the two or
    // three the corner takes together (surface_strip_ends::try_close_the_ends). Exact rather than a
    // nearness test, for the reason that gathering gives - a nearness test is not transitive, so
    // which ends came out together would depend on which one was asked first.
    let ends_at = the_ends_gathered_by_vertex(plans);

    for plan in plans {
        if plan.is_closed || plan.edges.is_empty() || !this_groups_own(plan, the_groups_mesh_edges)
        {
            continue;
        }

        let first = &plan.edges[0];
        let last = &plan.edges[plan.edges.len() - 1];

        // "RunsOut" reads the other way round than it sounds: it is TRUE where the chain STOPS at
        // that corner against a wall of its own (`SurfaceBevelPlan::start_runs_out`). Free on top
        // of that is the end no other pick stands at.
        let start_stops_free =
            plan.start_runs_out && ends_at.get(&first.edge.from_vertex()) == Some(&1);
        let end_stops_free = plan.end_runs_out && ends_at.get(&last.edge.to_vertex()) == Some(&1);

        if !start_stops_free && !end_stops_free {
            continue;
        }

        let start = first.from();
        let end = last.to();

        let where_ = if start_stops_free && end_stops_free {
            (first_of(start, end), second_of(start, end))
        } else {
            // One end keyed as itself twice, which no two-ended pair can be: a run whose two ends
            // are the same vertex is a closed one, and closed runs never get here.
            let one = if start_stops_free { start } else { end };
            (one, one)
        };

        if already_said.contains(&where_) {
            continue;
        }

        already_said.push(where_);

        // One line for the run and not one per end, for the reason
        // bevel_chain_end_budget::held_back gives: two ends of one short run are one story, and
        // saying it twice reads as two problems.
        said.get_or_insert_with(Vec::new)
            .push(if start_stops_free && end_stops_free {
                BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceRunStopsSquareAtBothEnds,
                    vec![start.into(), end.into(), edge_count.into()],
                )
            } else {
                BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceRunStopsSquareAtOneEnd,
                    vec![
                        (if start_stops_free { start } else { end }).into(),
                        edge_count.into(),
                    ],
                )
            });
    }

    said
}

/// Whether a plan is one the group being written to asked for, which is where its sentence may
/// land.
///
/// One PASS plans every contributing group's chains together, and two groups that named the same
/// edge are linked into one chain (`SurfaceSharedEdgeNetwork`), so a plan can belong to more than
/// one group and each of them hears about it. What this stops is the other thing: a group hearing
/// where somebody ELSE's run stopped, which reads on its row as a short cut of its own that it
/// cannot find.
fn this_groups_own(plan: &SurfaceBevelPlan, the_groups_mesh_edges: Option<&HashSet<i32>>) -> bool {
    let Some(the_groups_mesh_edges) = the_groups_mesh_edges else {
        return true;
    };

    plan.edges
        .iter()
        .any(|edge| the_groups_mesh_edges.contains(&edge.edge.edge.mesh_edge_index))
}

/// How many open plans END at each mesh vertex any of them ends at. One is an end standing alone,
/// which is the only kind the build closes against a wall; more than one is a corner or a junction,
/// and the bevel beside it is what closes it.
fn the_ends_gathered_by_vertex(plans: &[Rc<SurfaceBevelPlan>]) -> HashMap<i32, i32> {
    let mut ends_at = HashMap::new();

    for plan in plans {
        if plan.is_closed || plan.edges.is_empty() {
            continue;
        }

        for vertex in [
            plan.edges[0].edge.from_vertex(),
            plan.edges[plan.edges.len() - 1].edge.to_vertex(),
        ] {
            *ends_at.entry(vertex).or_insert(0) += 1;
        }
    }

    ends_at
}

/// Whichever of two points sorts first, so a pair can be keyed however it was walked.
fn first_of(a: Vector3, b: Vector3) -> Vector3 {
    if before(a, b) {
        a
    } else {
        b
    }
}

/// The other one.
fn second_of(a: Vector3, b: Vector3) -> Vector3 {
    if before(a, b) {
        b
    } else {
        a
    }
}

/// X, then y, then z.
#[allow(clippy::float_cmp)]
fn before(a: Vector3, b: Vector3) -> bool {
    if a.x != b.x {
        return a.x < b.x;
    }

    if a.y != b.y {
        return a.y < b.y;
    }

    a.z < b.z
}
