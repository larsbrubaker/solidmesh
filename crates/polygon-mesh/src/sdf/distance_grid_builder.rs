//! agg-sharp `PolygonMesh/Sdf/MeshSdfGrid.cs`'s `DistanceGridBuilder` delegate: a fill that
//! computes a grid's unsigned narrow-band distances in place of g3's CPU flood. `MeshSdfGrid`
//! (`mesh_sdf_grid.rs`) lays the grid out exactly as the flood would and hands the fill the
//! triangles; the sign stays its CPU crossing vote. polygon-mesh has no GPU, so the production fill
//! (MatterCAD's WebGPU `GpuMeshSdf`) lives in the UI layer (mattercad-renderer) and is injected
//! through this trait.
//!
//! C# awaits the builder (the browser's GPU read-back finishes on a later turn of the event loop);
//! here the builder blocks the calling thread, which is a rebuild's worker.

/// One fill's input: C#'s `DistanceGridBuilder(triangles, ni, nj, nk, cellSize, band, cap)`
/// arguments, plus `GpuMeshSdf.ComputeDistances`'s `stageDone`.
pub struct DistanceGridRequest<'a> {
    /// Nine floats per triangle (three corners), relative to the grid's sample (0, 0, 0).
    pub triangles: &'a [f32],
    /// Samples along x.
    pub ni: i32,
    /// Samples along y.
    pub nj: i32,
    /// Samples along z.
    pub nk: i32,
    /// Sample spacing; sample (i, j, k) sits at (i, j, k) * `cell_size`.
    pub cell_size: f32,
    /// Distances out to this are exact ...
    pub band: f32,
    /// ... the rest get this value.
    pub cap: f32,
    /// Called with 1 when the triangles are bucketed, 2 when the dispatch has finished and 3 when
    /// the distances are read back, so a caller can pace a progress bar across a pass it cannot
    /// otherwise watch (C# `stageDone`; `None` for the plain delegate).
    pub stage_done: Option<&'a (dyn Fn(i32) + Sync)>,
}

/// Why a fill gave no distances.
#[derive(Debug, Clone, PartialEq)]
pub enum DistanceBuildError {
    /// The caller's token was cancelled (C# `OperationCanceledException`).
    Cancelled,
    /// The fill failed (C# any other exception), with its message.
    Failed(String),
}

impl std::fmt::Display for DistanceBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DistanceBuildError::Cancelled => write!(f, "The operation was canceled."),
            DistanceBuildError::Failed(message) => write!(f, "{message}"),
        }
    }
}

impl std::error::Error for DistanceBuildError {}

/// C# `DistanceGridBuilder`: returns `ni * nj * nk` unsigned distances, index
/// `i + ni * (j + nj * k)`.
pub trait DistanceGridBuilder: Send + Sync {
    /// Computes the request's distances.
    fn compute_distances(
        &self,
        request: &DistanceGridRequest<'_>,
    ) -> Result<Vec<f32>, DistanceBuildError>;
}
