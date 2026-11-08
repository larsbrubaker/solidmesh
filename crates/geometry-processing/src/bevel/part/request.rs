//! What [`bevel_part`](super::bevel_part) is asked ([`BevelPartRequest`]: one part's mesh, where
//! it sits, its picks and the operation's settings) and what stops it outright
//! ([`BevelPartError`]: a cancel, or a build that could not run). A refusal is neither: it is a
//! row of the [`BevelPartOutcome`](super::BevelPartOutcome) with a code.

use std::sync::Arc;

use polygon_mesh::Mesh;
use vector_math::{Color, Matrix4X4};

use super::surface_part_builder::{BevelSurfacePartBuilder, SurfacePartBuildError};
use super::{BevelPickMemo, BevelRouting};
use crate::bevel::level_line::point_bevel::PointBevelFault;
use crate::bevel::level_line::WorkError;
use crate::topology::BevelEdgeGroup;

/// One part's bevel, as plain data: the build reads nothing else.
#[derive(Clone)]
pub struct BevelPartRequest {
    /// The part's mesh, in the part's own space; `None` (or a mesh with no vertices) is a part
    /// with nothing to bevel, and every group is refused with `PartHasNoMeshToBevel`.
    pub mesh: Option<Arc<Mesh>>,
    /// The part's space carried into the operation's, the space `size` is quoted in and the
    /// outcome's mesh is returned in (row-major, row vectors). [`Matrix4X4::IDENTITY`] where the
    /// two are the same.
    pub source_to_operation: Matrix4X4,
    /// The part's color as the caller shows it, which the outcome falls back to where the mesh
    /// carries no face colors of its own.
    pub color: Color,
    /// The picks on this part: edge chains, single edges, face rims and (with
    /// `BevelSelectionKind::Point`) corner points, anchored in the part's own space. The
    /// outcome's rows are indexed into this list.
    pub groups: Vec<BevelEdgeGroup>,
    /// The fillet radius or chamfer setback, one for every group.
    pub size: f64,
    /// How many segments a round is made of.
    pub segments: i32,
    /// A fillet's tangent arc (true) or a chamfer's straight cut (false).
    pub profile_is_round: bool,
    /// The bend threshold in degrees: the flat angle every group is read at, and the corner a
    /// point pick must be.
    pub bend_threshold_degrees: f64,
    /// What the caller's operation already knows about these picks across rebuilds, which is what
    /// lets a drag of the size measure the room once (`None`: a memo private to this build).
    pub pick_memo: Option<Arc<BevelPickMemo>>,
    /// How long one search for the size that fits may take, in milliseconds
    /// ([`BevelPartRequest::DEFAULT_SIZE_SEARCH_BUDGET_MS`] unless the caller says otherwise).
    pub size_search_budget_ms: i32,
    /// Which constructions the build may take ([`BevelRouting::default`], MatterCAD's routes,
    /// unless the caller says otherwise).
    pub routing: BevelRouting,
}

impl std::fmt::Debug for BevelPartRequest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BevelPartRequest")
            .field("mesh", &self.mesh)
            .field("source_to_operation", &self.source_to_operation)
            .field("color", &self.color)
            .field("groups", &self.groups)
            .field("size", &self.size)
            .field("segments", &self.segments)
            .field("profile_is_round", &self.profile_is_round)
            .field("bend_threshold_degrees", &self.bend_threshold_degrees)
            .field("pick_memo", &self.pick_memo.is_some())
            .field("size_search_budget_ms", &self.size_search_budget_ms)
            .field("routing", &self.routing)
            .finish()
    }
}

impl BevelPartRequest {
    /// The size search budget a request starts from.
    pub const DEFAULT_SIZE_SEARCH_BUDGET_MS: i32 =
        BevelSurfacePartBuilder::DEFAULT_SIZE_SEARCH_BUDGET_MS;

    /// A request for `groups` on `mesh`, whose own space is the operation's, with no color, no
    /// pick memo, the default size search budget and the default routes (MatterCAD's).
    pub fn new(
        mesh: Option<Arc<Mesh>>,
        groups: Vec<BevelEdgeGroup>,
        size: f64,
        segments: i32,
        profile_is_round: bool,
        bend_threshold_degrees: f64,
    ) -> Self {
        BevelPartRequest {
            mesh,
            source_to_operation: Matrix4X4::IDENTITY,
            color: Color::default(),
            groups,
            size,
            segments,
            profile_is_round,
            bend_threshold_degrees,
            pick_memo: None,
            size_search_budget_ms: Self::DEFAULT_SIZE_SEARCH_BUDGET_MS,
            routing: BevelRouting::default(),
        }
    }
}

/// What stops a part's build outright: a cancel, or a build that could not run (a work budget
/// that ran out, a surface construction that could not start, a Point group's fault). Only the
/// cancel is the caller's to pass on; for the others the part's rows are
/// [`BevelPartOutcome::every_group_refused`](super::BevelPartOutcome::every_group_refused).
#[derive(Debug)]
pub enum BevelPartError {
    /// The cancellation token was cancelled.
    Cancelled,
    /// A walk ran out of its budget.
    Work(WorkError),
    /// The surface construction could not run.
    Surface(SurfacePartBuildError),
    /// A Point group's bevel left by a fault.
    Point(PointBevelFault),
}

impl std::fmt::Display for BevelPartError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BevelPartError::Cancelled => f.write_str("The operation was canceled."),
            BevelPartError::Work(error) => write!(f, "{error}"),
            BevelPartError::Surface(error) => write!(f, "{error:?}"),
            BevelPartError::Point(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for BevelPartError {}

impl From<PointBevelFault> for BevelPartError {
    fn from(error: PointBevelFault) -> Self {
        match error {
            PointBevelFault::Work(WorkError::Cancelled) => BevelPartError::Cancelled,
            other => BevelPartError::Point(other),
        }
    }
}

impl From<SurfacePartBuildError> for BevelPartError {
    fn from(error: SurfacePartBuildError) -> Self {
        match error {
            SurfacePartBuildError::Cancelled => BevelPartError::Cancelled,
            other => BevelPartError::Surface(other),
        }
    }
}

impl From<WorkError> for BevelPartError {
    fn from(error: WorkError) -> Self {
        match error {
            WorkError::Cancelled => BevelPartError::Cancelled,
            other => BevelPartError::Work(other),
        }
    }
}
