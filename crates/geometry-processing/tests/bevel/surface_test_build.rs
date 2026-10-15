//! A surface-mode build of one part's groups, by the part builder's own steps in the part
//! builder's own order - `BevelSurfacePartBuilder::build_the_groups_together`, which lives above
//! the kernel crate.
//!
//! One copy for every kernel test that needs the whole pipeline rather than one stage of it. It is
//! the ORDER that matters and is easy to get wrong - the interrupted-crease reading before the
//! plans, the room before the builder - so a second hand-written copy of it in another test file
//! would be a second thing to keep in step with production.
//!
//! **Two of the part builder's gates are left out, on purpose.** It short-circuits on no plans -
//! nothing here ever has no plan, and a test that did would want to see the advisory rather than
//! nothing - and it reads `surface_mesh_closure::is_closed` on the ANSWER, turning an open result
//! into `SurfaceResultNotClosed`. That last one is deliberately not taken here: closure is what
//! most of these tests are about, and a helper that refused an open mesh would hand them a code
//! where they want the mesh and the place it is open. Every caller asserts closure itself.

// The kernel tests that build through here land a class at a time.
#![allow(dead_code)]

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::sections::section_routing;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_interrupted_crease, surface_mesh_closure, surface_room,
    surface_shared_edge_network, SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan,
    SurfaceBevelPlanlessLoop, SurfaceBuildFailure, SurfaceBuildOptions, SurfaceBuilt,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{BevelEdgeGroup, BevelEdgeGroupResolution, FaceRegionCache};
use polygon_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::shell_count;
use crate::surface_test_chains::join_and_open;

/// Pins `section_routing::use_sections` for as long as the handle lives, then puts back the value
/// it found, even when the test fails part way.
///
/// The flag is set for this thread only, so a test pinning it may run beside one that sets it the
/// other way.
pub struct UseSectionsPin {
    found_use_sections: bool,
}

/// The pin's name in the classes that read the legacy construction.
pub type LegacyConstructionPin = UseSectionsPin;

/// The flag OFF, for the classes that read the LEGACY
/// construction - their passes are cases the sections would route if the flag were on, so what
/// they assert is only true of the legacy path and they must not route.
pub fn pinned_to_the_legacy_construction() -> LegacyConstructionPin {
    pinned_use_sections(false)
}

/// The flag ON, as the section classes set it in their `try` blocks.
pub fn pinned_to_the_sections() -> UseSectionsPin {
    pinned_use_sections(true)
}

fn pinned_use_sections(on: bool) -> UseSectionsPin {
    let found_use_sections = section_routing::use_sections();
    section_routing::set_use_sections(on);

    UseSectionsPin { found_use_sections }
}

impl Drop for UseSectionsPin {
    fn drop(&mut self) {
        section_routing::set_use_sections(self.found_use_sections);
    }
}

/// Where an advisory says the build went wrong, for the message of an assertion that is not met.
pub fn station(reason: Option<&BevelAdvisory>) -> String {
    match reason.and_then(|reason| reason.args().first()) {
        Some(BevelAdvisoryArg::Vector3(at)) => format!("{at:?}"),
        _ => "nowhere named".to_string(),
    }
}

/// The optional settings of a test build.
#[derive(Clone, Copy)]
pub struct TryBevelOptions {
    /// A fillet where true, a chamfer where false.
    pub round_profile: bool,
    /// Hands the pick's own feature set down the PLAN path. A seam
    /// of the ladder, off by default.
    pub carry_the_feature_reading: bool,
}

impl Default for TryBevelOptions {
    fn default() -> Self {
        TryBevelOptions {
            round_profile: true,
            carry_the_feature_reading: false,
        }
    }
}

/// What a build that ran hands back besides its answer: the plans the build made (before the
/// builder ran), for a test that has to read what was planned.
pub struct TryBevelOutcome {
    /// The plans made, whether or not the build then succeeded.
    pub plans_built: Vec<Rc<SurfaceBevelPlan>>,
    /// The build's own answer.
    pub result: Result<SurfaceBuilt, SurfaceBuildFailure>,
}

/// The build from GROUPS.
pub fn try_bevel(
    part: &Mesh,
    groups: &[BevelEdgeGroup],
    size: f64,
    segments: i32,
    cancellation_token: &CancelToken,
    options: TryBevelOptions,
) -> TryBevelOutcome {
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(groups, part);

    try_bevel_resolutions(
        part,
        &resolutions,
        size,
        segments,
        cancellation_token,
        options,
    )
}

/// The same build from RESOLUTIONS rather than groups - for
/// a test that has to hand one group more than one edge, which only the resolution carries.
pub fn try_bevel_resolutions(
    part: &Mesh,
    resolutions: &[BevelEdgeGroupResolution],
    size: f64,
    segments: i32,
    cancellation_token: &CancelToken,
    options: TryBevelOptions,
) -> TryBevelOutcome {
    let mut plans_built = Vec::new();
    let result = build(
        part,
        resolutions,
        size,
        segments,
        cancellation_token,
        options,
        &mut plans_built,
    );

    TryBevelOutcome {
        plans_built,
        result,
    }
}

impl TryBevelOutcome {
    /// Whether the build succeeded.
    pub fn built(&self) -> bool {
        self.result.is_ok()
    }

    /// The advisory a refused build names, `None` where it built or declined
    /// with nothing named.
    pub fn reason(&self) -> Option<&BevelAdvisory> {
        self.result
            .as_ref()
            .err()
            .and_then(|failure| failure.advisory())
    }

    /// The built mesh, or the advisory the build named (`None` for a decline, a cancel or a NaN -
    /// a failed build that names no reason).
    pub fn mesh_or_reason(&self) -> Result<&Mesh, Option<&BevelAdvisory>> {
        match &self.result {
            Ok(built) => Ok(&built.mesh),
            Err(failure) => Err(failure.advisory()),
        }
    }
}

/// The assertions the section owner-file classes make of every build they expect to publish: built,
/// closed, at most two faces on an edge, one shell, and readable by a second pass.
pub fn the_closed_readable_mesh(outcome: &TryBevelOutcome) -> &Mesh {
    let mesh = match outcome.mesh_or_reason() {
        Ok(mesh) => mesh,
        Err(reason) => panic!(
            "{:?} at {}",
            reason.map(BevelAdvisory::code),
            station(reason)
        ),
    };

    let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);
    assert_eq!(open, 0, "open at {open_at:?}");

    assert!(crate::rigid_motion_meshes::most_faces_on_an_edge(mesh) <= 2);
    assert_eq!(shell_count(mesh), 1);

    let (unreadable, unreadable_at) = crate::mesh_measurements::unclassifiable_edges(Some(mesh));
    assert_eq!(
        unreadable, 0,
        "a second pass reads {unreadable_at} as a torn face"
    );

    mesh
}

/// The mesh a build that has to succeed hands back, or a test failure that names the reason,
/// with `because`.
pub fn expect_built(outcome: TryBevelOutcome, because: &str) -> Mesh {
    match outcome.result {
        Ok(built) => built.mesh,
        Err(failure) => panic!("{because} - instead {failure:?}"),
    }
}

/// The three checks nearly every built answer is held to: closed
/// (`surface_mesh_closure::is_closed`), manifold, and one shell.
pub fn assert_sound(mesh: &Mesh) {
    let (closed, open_at) = surface_mesh_closure::is_closed(mesh);

    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert!(mesh.is_manifold());
    assert_eq!(shell_count(mesh), 1);
}

fn refused(reason: BevelAdvisory) -> Result<SurfaceBuilt, SurfaceBuildFailure> {
    Err(SurfaceBuildFailure::Refused(reason))
}

fn build(
    part: &Mesh,
    resolutions: &[BevelEdgeGroupResolution],
    size: f64,
    segments: i32,
    cancellation_token: &CancelToken,
    options: TryBevelOptions,
    plans_built: &mut Vec<Rc<SurfaceBevelPlan>>,
) -> Result<SurfaceBuilt, SurfaceBuildFailure> {
    let graph = part.get_mesh_edge_graph();
    let first = &resolutions[0];
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        first.features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        first.group.curve_tolerance_degrees,
    ));

    let chains = join_and_open(&surface_shared_edge_network::link_together(resolutions));

    if let Some(reason) = surface_interrupted_crease::try_read(
        &chains,
        &first.features,
        &regions,
        size,
        options.round_profile,
    ) {
        return refused(reason);
    }

    let mut plans = Vec::new();
    let mut planless = Vec::new();

    for chain in &chains {
        match SurfaceBevelPlan::try_create_with_curve_regions(
            chain,
            &regions,
            Some(&curve_regions),
            size,
            options.round_profile,
        ) {
            Ok(plan) => plans.push(plan),
            Err(reason) => {
                // A closed loop whose two sides BOTH read curved is one the planner will not plan
                // and the sections can still walk - see SurfaceBevelPlanlessLoop. It is carried to
                // the builder as the bare ring the loop entry takes; with the flag off nothing is
                // carried and the refusal stands exactly where it always did.
                if section_routing::use_sections()
                    && reason.code() == BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces
                    && chain.is_closed
                    && chain.edges.len() >= 3
                {
                    planless.push(SurfaceBevelPlanlessLoop {
                        ring: chain.edges.iter().map(|edge| edge.from_vertex()).collect(),
                        features: first.features.clone(),
                        size,
                        planner_refusal: reason,
                    });

                    continue;
                }

                return refused(reason);
            }
        }
    }

    plans_built.extend(plans.iter().cloned());

    let mut stretches: Vec<SurfaceBandRunsOffOverAStretch> = Vec::new();
    let mut crossings = None;

    // The legacy room read cannot speak for a pass that carries plan-less loops: it reads bands
    // rolled off PLANS and there is no plan for those loops, so what it would answer is a verdict
    // on part of the pick. A routed pass reads its own room (section_routing::has_room) over every
    // run it built.
    if !plans.is_empty() && planless.is_empty() {
        let fit = surface_room::try_fit(&plans, size, cancellation_token, Some(&mut stretches))
            .map_err(|_| SurfaceBuildFailure::Cancelled)?;

        if !fit.fits {
            return match fit.refusal {
                Some(reason) => refused(reason),
                None => Err(SurfaceBuildFailure::Declined),
            };
        }

        crossings = Some(fit.crossings);
    }

    surface_bevel_builder::try_build(
        part,
        &graph,
        &regions,
        &plans,
        segments,
        cancellation_token,
        SurfaceBuildOptions {
            crossings: crossings.as_deref(),
            ran_off_over_a_stretch: Some(&stretches),
            lay_the_oblique_rows_anyway: false,
            planless_loops: &planless,
            features: if options.carry_the_feature_reading {
                Some(&first.features)
            } else {
                None
            },
        },
    )
}
