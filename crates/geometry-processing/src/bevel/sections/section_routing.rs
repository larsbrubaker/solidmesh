//! The gate between the legacy edge-centric Surface construction and the vertex-centric sections
//! one: whether a whole
//! pass of plans is SIMPLE in the sections' sense, and the run to build it from when it is. The
//! clauses it asks of each plan are in `super::section_routing_clauses`.
//!
//! **Whole builds, never half.** The router answers for the pass and not for a chain, because the
//! two constructions publish different meshes at the same place - the sections' own documented
//! trade is about a quarter of a percent of the radius on the wall - and a pass that took one
//! chain down each path would have to reconcile them at a seam neither of them draws. Either every
//! plan of the pass is simple and the sections build all of it, or the legacy construction builds
//! all of it untouched.
//!
//! **The last clause is asked by building.** Every clause above it is a question the plans and the
//! mesh can already answer, but "each side at each vertex is a SLIDE with at most one fold or an
//! OFFSET" is `SectionBuilder`'s own reading of the fan, and a second copy of it here would be a
//! second thing to keep in step. So the run is built, and a router that says yes hands the run it
//! built to the caller rather than making it twice.
//!
//! **Every plan of the pass, over ONE pool.** `SectionAssembly` takes a LIST of runs and rebuilds
//! the part's faces once over the union of their fans, so a pass of several chains routes as one:
//! every plan must pass every clause on its own, each is built into its own run against the
//! ORIGINAL mesh over the one pool, and the list is assembled in a single pass. What is still read
//! ACROSS the pass rather than per plan is the picked-edge count at a vertex - three picked edges
//! meeting there is a junction whichever chains they came from.
//!
//! **The room is read before anything is assembled.** A routed pass asks `SectionRoom` whether
//! the size fits every one of its chains, and a chain that has no room refuses the PASS with the
//! legacy size advisory carrying the most that fits. It does not fall back to the legacy
//! construction: the two paths refuse at different numbers, so falling back would build a mesh the
//! panel has just told the user does not fit.
//!
//! Whether the sections are asked at all is the caller's routing (`BevelRouting::sections`, carried
//! down as `SurfaceBuildOptions::use_sections`), never read here. What lives here besides the
//! router is its diagnostics, a test seam: the last route and its observer belong to the thread
//! that asked for the build, not the process, because tests run on threads of one process and a
//! test that reads the route its own build took must not see another test's build beside it. They
//! live in a record each thread starts with its own of; work handed to another thread carries the
//! record along ([`SectionRouteRecord::current`] on the submitting thread,
//! [`SectionRouteRecord::enter`] on the worker; the model's compute pool does this for every job),
//! so the route a build takes is written where its asker reads it.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex, MutexGuard};

use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_builder_run::{try_build_loop, try_build_plan};
use super::section_builder_walked_side::SectionBuildRefusal;
use super::section_room::{try_fit_edges, SectionRoomCache};
use super::section_routing_clauses::{picked_edges_by_vertex, reads, reads_plan, the_feature_test};
use super::{NanSectionPoint, SectionPool, SectionRun};
use crate::bevel::surface::{SurfaceBevelPlan, SurfaceBevelPlanlessLoop};
use crate::bevel::BevelAdvisory;
use crate::topology::FeatureEdgeSet;

/// A route observer, told each route as it is written.
pub type RouteObserver = Arc<dyn Fn(Option<&str>) + Send + Sync>;

/// The last route ([`last_route`]) and its observer ([`route_observed`]), for one thread and the
/// work it hands on. Shared behind a
/// mutex because a carried record is written on a worker thread and read on the asking one.
#[derive(Default)]
struct RouteRecord {
    last: Option<String>,
    observer: Option<RouteObserver>,
}

type SharedRouteRecord = Arc<Mutex<RouteRecord>>;

thread_local! {
    // The record this thread's routes are written to: its own, or one carried in.
    static ROUTES: RefCell<SharedRouteRecord> = RefCell::new(SharedRouteRecord::default());
}

fn routes() -> SharedRouteRecord {
    ROUTES.with(|routes| Arc::clone(&routes.borrow()))
}

fn locked(record: &Mutex<RouteRecord>) -> MutexGuard<'_, RouteRecord> {
    // It holds two whole values, complete after any write, so a panic elsewhere cannot leave it
    // half-made: a poisoned lock's data is still good.
    record
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// The record one thread's routes go to, captured so work run on another thread reports its route
/// where the thread that asked for it reads it. A test seam, like the record itself.
#[derive(Clone)]
pub struct SectionRouteRecord {
    routes: SharedRouteRecord,
}

impl std::fmt::Debug for SectionRouteRecord {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SectionRouteRecord").finish_non_exhaustive()
    }
}

impl SectionRouteRecord {
    /// This thread's route record right now.
    pub fn current() -> Self {
        Self { routes: routes() }
    }

    /// Writes routes to the captured record, on this thread until the returned guard drops.
    pub fn enter(&self) -> SectionRouteRecordScope {
        let outer_routes =
            ROUTES.with(|routes| std::mem::replace(&mut *routes.borrow_mut(), self.routes.clone()));
        SectionRouteRecordScope {
            outer_routes: Some(outer_routes),
            _not_send: std::marker::PhantomData,
        }
    }
}

/// While alive, this thread writes its routes where a [`SectionRouteRecord`] carried in says;
/// dropping it puts back what the thread had before. It belongs to the thread that made it.
#[must_use = "the carried record ends as soon as the scope is dropped"]
pub struct SectionRouteRecordScope {
    outer_routes: Option<SharedRouteRecord>,
    // A scope moved to another thread would restore that thread's record wrongly.
    _not_send: std::marker::PhantomData<*const ()>,
}

impl std::fmt::Debug for SectionRouteRecordScope {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SectionRouteRecordScope")
            .finish_non_exhaustive()
    }
}

impl Drop for SectionRouteRecordScope {
    fn drop(&mut self) {
        if let Some(outer) = self.outer_routes.take() {
            ROUTES.with(|routes| *routes.borrow_mut() = outer);
        }
    }
}

/// Which way the last pass through [`is_simple`] that this
/// thread asked for went - "sections", or "legacy: " and the first clause that failed. A test
/// seam: routing is invisible in the answer, so without it a test could only read the mesh and
/// guess.
pub fn last_route() -> Option<String> {
    locked(&routes()).last.clone()
}

/// Records the route and tells [`route_observed`]'s
/// observer.
pub fn set_last_route(value: Option<String>) {
    let record = routes();
    // Cloned out of the lock so an observer may itself read or write the route.
    let observer = {
        let mut record = locked(&record);
        record.last = value.clone();
        record.observer.clone()
    };
    if let Some(observer) = observer {
        observer(value.as_deref());
    }
}

/// Told every time this thread's last route is written, so that
/// a caller which builds MANY passes - the app's own drop-and-retry, which runs
/// `SurfaceBevelBuilder.TryBuild` once per turn - can keep each pass's route instead of only the
/// last one's. `None` in production; a test seam. Every `TryBuild` opens by writing "legacy: the
/// flag is off", so that sentence also delimits one call from the next in whatever the observer
/// collects.
pub fn route_observed() -> Option<RouteObserver> {
    locked(&routes()).observer.clone()
}

/// Sets (or clears) [`route_observed`] for this thread and the work it hands
/// on.
pub fn set_route_observed(observer: Option<RouteObserver>) {
    locked(&routes()).observer = observer;
}

fn route(value: String) {
    set_last_route(Some(value));
}

/// What a routed pass is built from: one run per plan (the plans' own order, then the plan-less
/// loops' own order), and the pool they were all minted into, which the assembly must be given as
/// well.
pub struct SectionRoute {
    /// The runs, one per plan and then one per plan-less loop.
    pub runs: Vec<SectionRun>,
    /// The one pool every run was minted into.
    pub pool: SectionPool,
}

/// What [`is_simple`] answers: the routed pass, or the first failing
/// clause (for the log and for the tests) and - for a declined plan-less loop - the advisory to
/// answer the pass with.
pub enum SectionRouteAnswer {
    /// The pass is simple: build it from these runs.
    Sections(SectionRoute),
    /// The pass goes to the legacy construction (or, for a declined plan-less loop, is refused).
    Legacy {
        /// The first failing clause.
        why_not: String,
        /// The plan-less loop's own planner refusal, where a loop was declined; `None` otherwise.
        refusal: Option<BevelAdvisory>,
    },
}

impl SectionRouteAnswer {
    /// Whether the pass is simple.
    pub fn is_simple(&self) -> bool {
        matches!(self, SectionRouteAnswer::Sections(_))
    }

    /// The first failing clause; `None` where the pass is simple.
    pub fn why_not(&self) -> Option<&str> {
        match self {
            SectionRouteAnswer::Sections(_) => None,
            SectionRouteAnswer::Legacy { why_not, .. } => Some(why_not),
        }
    }

    fn legacy(why_not: String) -> Self {
        SectionRouteAnswer::Legacy {
            why_not,
            refusal: None,
        }
    }
}

/// The sentence for a refused run: its advisory code by name, or "with no code".
fn code_of(failure: &SectionBuildRefusal) -> String {
    failure
        .advisory()
        .map(|advisory| advisory.code().to_string())
        .unwrap_or_else(|| "with no code".to_string())
}

/// Whether this pass of plans (already known to be one size and one profile, planned on the
/// welded `mesh`)
/// is one the sections can build, and the runs and pool to build it from.
///
/// `features` is the feature set the PICK was resolved in, carried down the plan path so a planned
/// run's offsets - and the room read over them - walk the surface by the OPERATION's own reading
/// rather than stopping at every fold of a tessellated wall. `None` leaves a planned run with no
/// reading.
///
/// A section point that is not a number is an error here.
pub fn is_simple(
    plans: &[Rc<SurfaceBevelPlan>],
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    segments_per_quarter: i32,
    features: Option<&FeatureEdgeSet>,
) -> Result<SectionRouteAnswer, NanSectionPoint> {
    // A missing mesh or graph is asked first, so the clauses take the mesh and graph as they
    // are, with the same sentence.
    let (Some(mesh), Some(graph)) = (mesh, graph) else {
        let why_not = "there is nothing to route".to_string();
        route(format!("legacy: {why_not}"));

        return Ok(SectionRouteAnswer::legacy(why_not));
    };

    let is_a_feature_edge = the_feature_test(features);
    if let Err(why_not) = reads(plans, mesh, graph, is_a_feature_edge.as_deref()) {
        route(format!("legacy: {why_not}"));

        return Ok(SectionRouteAnswer::legacy(why_not));
    }

    let mut pool = SectionPool::new(&mesh.vertices);

    let mut built = Vec::with_capacity(plans.len());

    // The last clause, asked by building it - see the module's notes. Every plan is built against
    // the ORIGINAL mesh over the one pool, so no run sees another's substitutions.
    for plan in plans {
        match try_build_plan(
            Some(plan.clone()),
            mesh,
            graph,
            &mut pool,
            plan.size,
            segments_per_quarter,
            features,
        ) {
            Ok(run) => built.push(run),
            Err(SectionBuildRefusal::NotANumber(nan)) => return Err(nan),
            Err(failure) => {
                let why_not = format!("the sections refused the run: {}", code_of(&failure));
                route(format!("legacy: {why_not}"));

                return Ok(SectionRouteAnswer::legacy(why_not));
            }
        }
    }

    route("sections".to_string());

    Ok(SectionRouteAnswer::Sections(SectionRoute {
        runs: built,
        pool,
    }))
}

/// The same pass, with loops the PLANNER would
/// not make a plan for carried alongside the plans (see [`SurfaceBevelPlanlessLoop`]), built after
/// the planned ones, in their own order.
///
/// A plan-less loop is read through the loop entry ([`try_build_loop`]), which takes a bare
/// vertex ring and the pick's features and so asks none of the clauses - those are questions about
/// a PLAN, and there is none. What it does still share is the pool: every run of the pass, planned
/// or not, is minted into one, because the assembly rebuilds the part's faces once over the union
/// of their fans.
///
/// **A declined loop is the planner's refusal again.** There is no legacy construction to fall
/// back to for a plan that was never made, so the answer carries the loop's own
/// `planner_refusal` back out.
pub fn is_simple_with_planless(
    plans: &[Rc<SurfaceBevelPlan>],
    planless: &[SurfaceBevelPlanlessLoop],
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    segments_per_quarter: i32,
    features: Option<&FeatureEdgeSet>,
) -> Result<SectionRouteAnswer, NanSectionPoint> {
    if planless.is_empty() {
        return is_simple(plans, mesh, graph, segments_per_quarter, features);
    }

    let (Some(mesh), Some(graph)) = (mesh, graph) else {
        let why_not = "there is nothing to route".to_string();
        route(format!("legacy: {why_not}"));

        return Ok(SectionRouteAnswer::legacy(why_not));
    };

    let mut built = Vec::new();

    let mut pool = if !plans.is_empty() {
        match is_simple(
            plans,
            Some(mesh),
            Some(graph),
            segments_per_quarter,
            features,
        )? {
            SectionRouteAnswer::Sections(planned) => {
                built.extend(planned.runs);
                planned.pool
            }
            declined => return Ok(declined),
        }
    } else {
        SectionPool::new(&mesh.vertices)
    };

    for planless_loop in planless {
        match try_build_loop(
            &planless_loop.ring,
            Some(&planless_loop.features),
            mesh,
            graph,
            &mut pool,
            planless_loop.size,
            segments_per_quarter,
            true,
        ) {
            Ok(run) => built.push(run),
            Err(SectionBuildRefusal::NotANumber(nan)) => return Err(nan),
            Err(failure) => {
                let why_not = format!(
                    "the sections declined a plan-less loop: {}",
                    code_of(&failure)
                );
                route(format!("legacy: {why_not}"));

                return Ok(SectionRouteAnswer::Legacy {
                    why_not,
                    refusal: Some(planless_loop.planner_refusal.clone()),
                });
            }
        }
    }

    route("sections".to_string());

    Ok(SectionRouteAnswer::Sections(SectionRoute {
        runs: built,
        pool,
    }))
}

/// Whether the pass's size fits every one of its chains, read in
/// closed form off the part (`SectionRoom`) before anything is assembled. `Err` carries the
/// refusal of the first chain that has no room - the legacy size advisory, already carrying the
/// most that fits (the room reading always names it).
pub fn has_room(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    runs: &[SectionRun],
    radius: f64,
) -> Result<(), Option<BevelAdvisory>> {
    // One cache for the pass: several chains standing on one face flood that face's region once.
    let mut cache = SectionRoomCache::new(mesh, graph);

    for run in runs {
        if !run.is_closed {
            // SectionRoom reads a CLOSED chain: every station's neighbours are taken modulo the
            // count and an open run's two ends have no such neighbour, so it refuses one outright
            // rather than measuring it. An open run's size is answered by the build instead - the
            // End sections and the smooth-side cut refuse by name where the size does not fit -
            // which is a refusal about the very geometry rather than a closed-form advisory.
            continue;
        }

        // The run's OWN feature reading goes with it: a smooth side's reach is read along the
        // surface the run's offsets were laid on, not across one facet of it.
        let (fits, _, failure) = try_fit_edges(
            Some(mesh),
            Some(graph),
            Some(&run.edges),
            run.is_closed,
            radius,
            Some(&mut cache),
            run.is_a_feature_edge.as_deref(),
        );
        if !fits {
            return Err(failure);
        }
    }

    Ok(())
}

/// [`partition`]'s answer: the indices into the plans the sections accept, in the
/// plans' own order, and one entry per remaining plan - its index and the first clause it fails,
/// the same sentence [`is_simple`] would name for it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SectionPartition {
    /// The plans the sections accept.
    pub accepted: Vec<usize>,
    /// The plans they decline, each with its first failing clause.
    pub declined: Vec<(usize, String)>,
}

/// Which plans of the pass the sections can build and which cannot,
/// read one plan at a time.
///
/// **Why a partition at all.** [`is_simple`] answers for the whole pass, so one open run in a face
/// pick sends chains the sections would have built - the raked hole mouths in a wall - down the
/// legacy path, which then cannot close them and drops them. The caller that builds a pass in two
/// (the sections first, legacy afterwards on their output) needs the split, not the verdict.
///
/// **The picked-edge count still reads the WHOLE pass, not the accepted subset.** A vertex where a
/// mouth touches another chain of the same pick is a junction on the part whether or not that
/// other chain is itself buildable: the sections would round one chain through a vertex the legacy
/// pass is about to cut its own geometry at. So a plan is declined if any of its vertices carries a
/// picked edge of another plan, accepted or declined. On a plan that has already passed the closed
/// clause its own edges contribute exactly two at each of its vertices, so any other count IS a
/// touch (or a chain that crosses itself), which is why one count answers both.
///
/// **No room reading here.** The size is read against the built runs by [`has_room`], and it
/// refuses the pass rather than trimming it - a chain with no room is an advisory to the user, not
/// a plan to hand to the legacy construction.
pub fn partition(
    plans: &[Rc<SurfaceBevelPlan>],
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    features: Option<&FeatureEdgeSet>,
) -> SectionPartition {
    let mut answer = SectionPartition::default();

    let (Some(mesh), Some(graph)) = (mesh, graph) else {
        // Nothing to route, and no plan to name a reason against.
        return answer;
    };
    if plans.is_empty() {
        return answer;
    }

    let carried = picked_edges_by_vertex(plans);
    let is_a_feature_edge = the_feature_test(features);

    for (i, plan) in plans.iter().enumerate() {
        match reads_plan(plan, mesh, graph, &carried, is_a_feature_edge.as_deref()) {
            Ok(()) => answer.accepted.push(i),
            Err(why_not) => answer.declined.push((i, why_not)),
        }
    }

    answer
}
