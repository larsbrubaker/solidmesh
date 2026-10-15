//! The [`DistanceGridBuilder`] hook of `MeshSdfGrid`: a fill that computes a grid's unsigned
//! narrow-band distances in place of the CPU flood. `MeshSdfGrid` (`mesh_sdf_grid.rs`) lays the
//! grid out exactly as the flood would and hands the fill the triangles; the sign stays its CPU
//! crossing vote. polygon-mesh has no GPU, so the production fill (the WebGPU one in
//! mattercad-renderer's `gpu_mesh_sdf`) lives in the UI layer and is injected through this
//! trait.
//!
//! The builder blocks the calling thread, which is a rebuild's worker, so the UI never waits
//! on the GPU read-back.

/// One fill's input: the triangles, the grid's size and spacing, the band and cap, and an
/// optional progress callback.
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
    /// otherwise watch (`None` when nobody watches).
    pub stage_done: Option<&'a (dyn Fn(i32) + Sync)>,
}

/// Why a fill gave no distances.
#[derive(Debug, Clone, PartialEq)]
pub enum DistanceBuildError {
    /// The caller's token was cancelled.
    Cancelled,
    /// The fill failed, with its message.
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

/// A distance fill: returns `ni * nj * nk` unsigned distances, index `i + ni * (j + nj * k)`.
pub trait DistanceGridBuilder: Send + Sync {
    /// Computes the request's distances.
    fn compute_distances(
        &self,
        request: &DistanceGridRequest<'_>,
    ) -> Result<Vec<f32>, DistanceBuildError>;
}
