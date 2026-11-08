//! The clauses the sections router asks of a pass's PLANS (the feature test, round closed chains,
//! buildable ends, two edges at every vertex, turns under the curve tolerance, one convexity, no
//! mixed corner or junction). Each answers with the sentence the
//! route log and the tests read; [`super::section_routing`] asks them clause-major for a whole
//! pass and plan-major for its partition.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use solidmesh_math::math_helper;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_run::IsAFeatureEdge;
use super::{section_corner, section_end, section_fan_walk, section_junction_build};
use crate::bevel::level_line::stopwatch::g4;
use crate::bevel::surface::SurfaceBevelPlan;
use crate::topology::{EdgeConvexity, FaceGroup, FeatureEdgeSet};

/// A clause's answer: `Ok` where the plan passes it, the first failing sentence otherwise.
pub type Clause = Result<(), String>;

/// The OPERATION's own reading of which mesh edges are features, as the
/// junction screen wants it - `None` where the pass carries no feature set, which leaves the
/// reading to the coplanarity fallback exactly as it was before this was carried at all.
///
/// One table per pass, built beside [`picked_edges_by_vertex`]: on a TESSELLATED wall the
/// fallback calls every facet fold a crease, and the wall-side fan at the pierced face's junctions
/// then reads TWO of them - so a junction routed without this reads as no junction at all and the
/// pass goes to legacy on the wedge.
pub fn the_feature_test(features: Option<&FeatureEdgeSet>) -> Option<IsAFeatureEdge> {
    let features = features?;

    let mut mesh_edges = HashSet::new();

    for feature in features.edges() {
        mesh_edges.insert(feature.mesh_edge_index);
    }

    Some(Rc::new(move |edge| mesh_edges.contains(&edge)))
}

/// Every clause the PLANS and the mesh can answer on their own, in the order the plan
/// states them.
///
/// The loops are CLAUSE-major and not plan-major: the sentence a failing pass carries is the first
/// clause any plan fails, which is what the route log and the tests have always read.
/// `Partition` asks the same clauses plan-major, since there the answer belongs to a plan.
pub fn reads(
    plans: &[Rc<SurfaceBevelPlan>],
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Clause {
    // A missing mesh or graph answers with the same sentence; the caller asks that first.
    if plans.is_empty() {
        return Err("there is nothing to route".to_string());
    }

    for plan in plans {
        is_a_round_closed_chain(plan, mesh, graph)?;
    }

    let carried = picked_edges_by_vertex(plans);

    for plan in plans {
        carries_two_edges_at_every_vertex(plan, &carried)?;
    }

    for plan in plans {
        turns_under_the_curve_tolerance(plan, mesh, graph, is_a_feature_edge)?;
    }

    for plan in plans {
        is_one_convexity(plan)?;
    }

    for plan in plans {
        no_corner_is_mixed_and_no_end_is_a_junction(plan)?;
    }

    Ok(())
}

/// Every clause of one plan, in the order [`reads`] asks them of the pass.
pub fn reads_plan(
    plan: &SurfaceBevelPlan,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    carried_over_the_pass: &HashMap<i32, i32>,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Clause {
    is_a_round_closed_chain(plan, mesh, graph)?;
    carries_two_edges_at_every_vertex(plan, carried_over_the_pass)?;
    turns_under_the_curve_tolerance(plan, mesh, graph, is_a_feature_edge)?;
    is_one_convexity(plan)?;
    no_corner_is_mixed_and_no_end_is_a_junction(plan)
}

/// A round profile, a closed chain, and at least three edges to close it
/// with.
pub fn is_a_round_closed_chain(
    plan: &SurfaceBevelPlan,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
) -> Clause {
    if !plan.round_profile {
        // A chamfer's section is a chord and not an arc, and nothing draws it yet. The profile is
        // one operation's, so the first plan answers for the pass - it is read per plan only to
        // keep every clause a question about a plan.
        return Err("the operation is a chamfer".to_string());
    }

    if !plan.is_closed && !both_ends_can_be_built(plan, mesh, graph) {
        return Err("a chain is not closed".to_string());
    }

    if plan.edges.len() < 3 {
        return Err(format!("a chain walks only {} edges", plan.edges.len()));
    }

    Ok(())
}

/// Whether an OPEN plan's two ends are both ones `SectionEnd` can draw -
/// the side beyond each end one flat region, and one crease shared with each side for the boundary
/// to slide down ([`section_end::can_end_at`]). An open chain is routed exactly when both answer
/// yes.
///
/// The predicate is `SectionEnd`'s own, not a copy: the same reading the build makes before it
/// draws anything. So the only open chains the router lets through are ones whose ends the build
/// is about to succeed at, and every other open chain keeps the sentence it has always carried - a
/// plain cube corner, where the chain edge's own two sides are ALL the faces at the vertex and
/// there is no transverse region at all, is one of them.
///
/// The mesh and graph are required: every caller has both.
pub fn both_ends_can_be_built(plan: &SurfaceBevelPlan, mesh: &Mesh, graph: &MeshEdgeGraph) -> bool {
    if plan.edges.len() < 3 {
        return false;
    }

    // One table for both ends - the fan walk's own lookup, which is the whole cost of the reading.
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(graph);

    section_end::can_end_at(mesh, graph, &plan.edges[0].edge, true, Some(&edge_at_pair)).is_ok()
        && section_end::can_end_at(
            mesh,
            graph,
            &plan.edges[plan.edges.len() - 1].edge,
            false,
            Some(&edge_at_pair),
        )
        .is_ok()
}

/// How many picked edges of the PASS end at each vertex - the one
/// reading the router makes across plans rather than within one. Only looked up, never iterated.
pub fn picked_edges_by_vertex(plans: &[Rc<SurfaceBevelPlan>]) -> HashMap<i32, i32> {
    let mut carried: HashMap<i32, i32> = HashMap::new();

    for plan in plans {
        for planned in &plan.edges {
            *carried.entry(planned.edge.from_vertex()).or_insert(0) += 1;
            *carried.entry(planned.edge.to_vertex()).or_insert(0) += 1;
        }
    }

    carried
}

/// Exactly two picked edges at every vertex of this chain,
/// counted across ALL the plans - a vertex three picked edges meet at is a junction whatever chain
/// each of them came from, and one edge is a free end. Version 1 has no treatment for either.
pub fn carries_two_edges_at_every_vertex(
    plan: &SurfaceBevelPlan,
    carried_over_the_pass: &HashMap<i32, i32>,
) -> Clause {
    // An OPEN chain that got past the closed clause ends at two vertices its own chain touches
    // ONCE, and those are the vertices the End sections are drawn at - a free end there is the
    // shape of the run, not a junction. Every other vertex still owes exactly two.
    let starts_at = if plan.is_closed {
        -1
    } else {
        plan.edges[0].edge.from_vertex()
    };
    let finishes_at = if plan.is_closed {
        -1
    } else {
        plan.edges[plan.edges.len() - 1].edge.to_vertex()
    };

    for planned in &plan.edges {
        for vertex in [planned.edge.from_vertex(), planned.edge.to_vertex()] {
            let owed = if vertex == starts_at || vertex == finishes_at {
                1
            } else {
                2
            };

            if let Some(&count) = carried_over_the_pass.get(&vertex) {
                if count != owed {
                    return Err(format!("vertex {vertex} carries {count} picked edges"));
                }
            }
        }
    }

    Ok(())
}

/// The turn between consecutive chain edges, under the curve
/// tolerance at every vertex - OR a turn that reads as a CORNER, which has its own section
/// (`SectionCorner`). The reading here is the cheap half, the one the two chain edges can answer
/// alone: the same flat side at the same wedge. Whether the fold leaving the vertex lets both edges
/// slide the same distance down it wants the fan, so it is left to the build, in the same spirit
/// as the last clause.
///
/// The tolerance is the default 30 degrees rather than the pick's own: a plan does not carry the
/// `CurveToleranceDegrees` its group was resolved at (only a `FaceRegionCache` flooded at it,
/// which does not publish the angle back), so the router reads the same default the resolver hands
/// a group that does not name one.
pub fn turns_under_the_curve_tolerance(
    plan: &SurfaceBevelPlan,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Clause {
    let count = plan.edges.len();

    // An open chain has no turn at index 0: the edge before it is the LAST one, half the part
    // away, and reading the wrap there would answer for a corner the chain does not have.
    let first = if plan.is_closed { 0 } else { 1 };
    for i in first..count {
        let arriving = plan.edges[(i + count - 1) % count]
            .edge
            .direction()
            .get_normal();
        let leaving = plan.edges[i].edge.direction().get_normal();

        let turn = math_helper::radians_to_degrees(arriving.dot(leaving).clamp(-1.0, 1.0).acos());

        let arrived = &plan.edges[(i + count - 1) % count];
        let leaves = &plan.edges[i];

        // The junction screen is asked only where BOTH edges are convex. The junction section is a
        // convex construction, so a sharp turn off a concave edge is not one whatever the wedges
        // read - and the pass's sentence must stay the convexity clause's, which is the one that
        // owns it (the handover corner, SectionHandoverCornerTests, turns 90 degrees at a vertex
        // whose edge is Concave).
        let could_be_a_junction = arrived.convexity() == EdgeConvexity::Convex
            && leaves.convexity() == EdgeConvexity::Convex;

        if turn > FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES
            && !section_corner::looks_like_a_corner(&arrived.edge, &leaves.edge)
            && !(could_be_a_junction
                && section_junction_build::is_a_junction_station(
                    mesh,
                    graph,
                    &arrived.edge,
                    &leaves.edge,
                    leaves.edge.from_vertex(),
                    is_a_feature_edge,
                    None,
                ))
        {
            return Err(format!(
                "the chain turns {} degrees at vertex {}",
                g4(turn),
                plan.edges[i].edge.from_vertex()
            ));
        }
    }

    Ok(())
}

/// One convexity per chain, and CONVEX. `SectionBuilder`'s centre is the
/// convex formula and a concave stretch reaching the loft is refused by its winding check, so a
/// concave chain is named here rather than found out there.
pub fn is_one_convexity(plan: &SurfaceBevelPlan) -> Clause {
    for planned in &plan.edges {
        if planned.convexity() != EdgeConvexity::Convex {
            // The sentence names the convexity by its member name, which is what `Debug` writes.
            return Err(format!(
                "edge at vertex {} is {:?}",
                planned.edge.from_vertex(),
                planned.convexity()
            ));
        }
    }

    Ok(())
}

/// No mixed corner and no junction. Both are readings the
/// legacy construction makes its own geometry for, and neither has a section.
///
/// **A ROLLED WALL is no longer one of them.** This clause used to decline any plan carrying a
/// `SurfaceEdgePlan.WallSection` - a side whose faces curve away under the chain rather than
/// standing flat - on the grounds that only the legacy roll drew that. The sections' smooth-side
/// cut IS that construction now: a station whose fan walk crosses a fold of the tessellation is
/// read on the surface, and the tapered barrel's six loops - four bore mouths and two end rims,
/// every one of them with the picked wall as its curving side - assemble closed, manifold and in
/// one shell through `SectionBuilder.TryBuildLoop` and `SectionAssembly`
/// (SectionTaperedBarrelMouthReadingTests). The clause predated that and was declining passes the
/// sections can cut.
///
/// The reading is left to the BUILD rather than replaced by a cheaper plan-time one. The router
/// reads plans before anything is walked, so it cannot tell a wall that is one smooth region from
/// a wall with a crease inside it; the build can, and refuses by name where the cut cannot be
/// made. Dropping the clause outright regresses nothing: the whole kernel suite - every owner-file
/// fixture, the robustness ratchet and both routing suites - is bit-identical without it, so no
/// pass that used to reach the legacy roll through this clause was relying on getting there.
pub fn no_corner_is_mixed_and_no_end_is_a_junction(plan: &SurfaceBevelPlan) -> Clause {
    for i in 0..plan.edges.len() {
        if plan.try_mixed_corner_after(i as i32).is_some() {
            return Err(format!("a mixed corner follows edge {i}"));
        }
    }

    if plan.try_junction_at(true).is_some() || plan.try_junction_at(false).is_some() {
        return Err("the chain ends at a junction".to_string());
    }

    Ok(())
}
