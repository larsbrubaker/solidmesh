//! A solid's signed distance field, sampled once on a grid, from which the offset surface at any
//! radius up to `max_radius` is pulled by marching cubes: the preview a Dilate / Erode radius
//! drag shows before the exact kernel result replaces it. Port of agg-sharp
//! `PolygonMesh/Sdf/MeshSdfGrid.cs`.
//!
//! The field is negative inside. Distances come from g3's `MeshSignedDistanceGrid` narrow-band
//! flood (`crate::g3`), exact out to the band (`max_radius` plus two cells) and never visiting
//! voxels beyond it; those keep a large magnitude. The sign is a vote: every grid line along each
//! axis counts triangle crossings (orientation free), and a voxel is inside when at least two of
//! its three parities say so, so one axis grazing an edge or a small hole cannot flip it.
//!
//! A grid given a `DistanceGridBuilder` (`distance_grid_builder.rs`; MatterCAD's GPU fill) lays
//! the grid out as the flood would and takes the unsigned distances from it instead
//! (`build_with_injected_distances`); the CPU flood is the reference that fill is tested against.
//! C# builds such a grid only through `BuildAsync`, which awaits the builder; here the builder
//! blocks, so `build` serves both. C# runs the crossing count with `Parallel.For` and atomic increments; the counts and votes do not
//! depend on order, so here it runs on one thread.

use std::sync::atomic::{AtomicI32, AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use vector_math::Vector3;

use super::distance_grid_builder::{DistanceBuildError, DistanceGridBuilder, DistanceGridRequest};
use super::grid_marching_cubes::{extract, DenseGrid3f};
use crate::csg::minkowski_processing::{orient_shells_as_solid, MinkowskiError};
use crate::csg::minkowski_shell_orientation::mesh_may_need_repair;
use crate::g3::mesh_signed_distance_grid::{point_in_triangle_2d, NarrowBandDistanceGrid};
use crate::g3::{DMesh3, DMeshAABBTree3};
use crate::{CancelToken, Mesh};

pub use crate::g3::mesh_signed_distance_grid::FloodProgress;

/// Why a grid was not built or an offset not extracted. C# throws these as exceptions.
#[derive(Debug, Clone)]
pub enum SdfGridError {
    /// An argument was out of range (C# `ArgumentOutOfRangeException`).
    OutOfRange(String),
    /// The caller's token was cancelled (C# `OperationCanceledException`); the field stays
    /// unbuilt, so a later call starts over.
    Cancelled,
    /// Orienting the part's shells failed other than by the kernel refusing the part (which the
    /// grid absorbs, as C# does).
    Orientation(MinkowskiError),
    /// The injected `DistanceGridBuilder` failed (C# the exception it threw), with its message.
    DistanceFill(String),
}

impl std::fmt::Display for SdfGridError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SdfGridError::OutOfRange(message) => write!(f, "{message}"),
            SdfGridError::Cancelled => write!(f, "The operation was canceled."),
            SdfGridError::Orientation(error) => write!(f, "{error}"),
            SdfGridError::DistanceFill(message) => write!(f, "{message}"),
        }
    }
}

impl std::error::Error for SdfGridError {}

/// The built field and the position of its sample (0, 0, 0).
#[derive(Debug)]
struct Built {
    field: DenseGrid3f,
    origin: Vector3,
}

/// C# `MeshSdfGrid`.
pub struct MeshSdfGrid {
    source: Mesh,
    max_radius: f64,
    resolution: i32,
    orient_shells: bool,
    flood_progress: Option<FloodProgress>,
    distance_builder: Option<Arc<dyn DistanceGridBuilder>>,
    build_lock: Mutex<()>,
    built: OnceLock<Built>,
    cell_size_bits: AtomicU64,
    grid_builds: AtomicI32,
    orient_repairs: AtomicI32,
    flood_work: AtomicI64,
}

impl MeshSdfGrid {
    /// The most voxels along the longest axis: 256^3 floats is 64 MB, and the step time grows
    /// with the cube of it.
    pub const MAX_RESOLUTION: i32 = 256;

    /// C#'s default `resolution`.
    pub const DEFAULT_RESOLUTION: i32 = 128;

    /// C# `new MeshSdfGrid(solid, maxRadius, resolution)`. `max_radius` is the largest |iso|
    /// the caller will extract (the band holds exact distances this far both sides, and the
    /// grid is padded by it); `resolution` is the voxel count along the padded longest axis,
    /// clamped to [`MeshSdfGrid::MAX_RESOLUTION`].
    pub fn new(solid: Mesh, max_radius: f64, resolution: i32) -> Result<Self, SdfGridError> {
        if max_radius.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return Err(SdfGridError::OutOfRange(
                "The offset range must be positive.".to_string(),
            ));
        }
        if resolution < 16 {
            return Err(SdfGridError::OutOfRange(
                "A grid needs at least 16 voxels along its longest axis.".to_string(),
            ));
        }
        Ok(MeshSdfGrid {
            source: solid,
            max_radius,
            resolution: resolution.min(Self::MAX_RESOLUTION),
            orient_shells: true,
            flood_progress: None,
            distance_builder: None,
            build_lock: Mutex::new(()),
            built: OnceLock::new(),
            cell_size_bits: AtomicU64::new(0f64.to_bits()),
            grid_builds: AtomicI32::new(0),
            orient_repairs: AtomicI32::new(0),
            flood_work: AtomicI64::new(0),
        })
    }

    /// C# `OrientShells` (init): whether the build orients the part's shells first. Leave it
    /// true unless the caller already ran [`orient_shells_as_solid`] on the mesh.
    pub fn with_orient_shells(mut self, orient_shells: bool) -> Self {
        self.orient_shells = orient_shells;
        self
    }

    /// C# `new MeshSdfGrid(.., distanceBuilder)`: computes the unsigned band distances with
    /// `builder` instead of g3's CPU flood; the sign is still the crossing vote here.
    pub fn with_distance_builder(mut self, builder: Arc<dyn DistanceGridBuilder>) -> Self {
        self.distance_builder = Some(builder);
        self
    }

    /// C# `FloodProgress`: called with the flood's running work count, once per voxel.
    pub fn set_flood_progress(&mut self, progress: Option<FloodProgress>) {
        self.flood_progress = progress;
    }

    /// How many times the distance field has been computed; one per instance.
    pub fn grid_builds(&self) -> i32 {
        self.grid_builds.load(Ordering::SeqCst)
    }

    /// The largest |iso| the band holds exact distances for.
    pub fn max_radius(&self) -> f64 {
        self.max_radius
    }

    /// The voxel count along the longest padded axis.
    pub fn resolution(&self) -> i32 {
        self.resolution
    }

    /// The voxel edge length, available once a build has started.
    pub fn cell_size(&self) -> f64 {
        f64::from_bits(self.cell_size_bits.load(Ordering::SeqCst))
    }

    /// How many builds sent the part through the kernel's shell orientation; for tests.
    pub fn orient_repairs(&self) -> i32 {
        self.orient_repairs.load(Ordering::SeqCst)
    }

    /// The voxels the band flood processed in the last build, finished or cancelled.
    pub fn flood_work(&self) -> i64 {
        self.flood_work.load(Ordering::SeqCst)
    }

    /// The sampled field (negative inside), for tests; `None` until built.
    pub fn field(&self) -> Option<&DenseGrid3f> {
        self.built.get().map(|built| &built.field)
    }

    /// The position of the field's sample (0, 0, 0), for tests; zero until built.
    pub fn field_origin(&self) -> Vector3 {
        self.built
            .get()
            .map_or(Vector3::new(0.0, 0.0, 0.0), |built| built.origin)
    }

    /// C# `Build` (and `BuildAsync` for a grid with a `DistanceGridBuilder`): computes the field
    /// if it has not been computed yet. Safe from several threads: one builds, the others wait.
    pub fn build(&self, cancel: &CancelToken) -> Result<(), SdfGridError> {
        if self.built.get().is_some() {
            return Ok(());
        }
        let _guard = self
            .build_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if self.built.get().is_some() {
            return Ok(());
        }
        let dmesh = self.prepare(cancel)?;
        let (mut field, origin) = match &self.distance_builder {
            Some(builder) => {
                self.build_with_injected_distances(&dmesh, builder.as_ref(), cancel)?
            }
            None => self.build_cpu_distances(&dmesh, cancel)?,
        };
        apply_crossing_signs(&dmesh, &mut field, origin, self.cell_size(), cancel)?;
        let _ = self.built.set(Built { field, origin });
        self.grid_builds.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    /// C# `Prepare`: orients the part (unless it is provably fine as is) and sets the cell size.
    fn prepare(&self, cancel: &CancelToken) -> Result<DMesh3, SdfGridError> {
        // Orient first, as the kernel does, so the preview is of the same solid the exact
        // Dilate / Erode will build; its repair also joins split seams the crossing count would
        // otherwise see as doubled walls.
        let mut oriented: Option<Mesh> = None;
        if self.orient_shells && mesh_may_need_repair(&self.source) {
            self.orient_repairs.fetch_add(1, Ordering::SeqCst);
            match orient_shells_as_solid(&self.source, cancel) {
                Ok(mesh) => oriented = Some(mesh),
                // The kernel refuses an open part, but the crossing count needs no orientation
                // and the vote outruns a small hole, so the part is used as it is.
                Err(MinkowskiError::ImportRejected { .. }) => {}
                Err(MinkowskiError::Cancelled) => return Err(SdfGridError::Cancelled),
                Err(other) => return Err(SdfGridError::Orientation(other)),
            }
        }

        let dmesh = oriented.as_ref().unwrap_or(&self.source).to_dmesh3();
        let bounds = dmesh.cached_bounds();
        let width = bounds.max[0] - bounds.min[0];
        let height = bounds.max[1] - bounds.min[1];
        let depth = bounds.max[2] - bounds.min[2];
        let longest = width.max(height.max(depth));

        // The band (max_radius plus two cells) is also the padding each side; this cell size
        // makes the padded longest axis `resolution` voxels.
        let cell_size = (longest + (2.0 * self.max_radius)) / (self.resolution - 4) as f64;
        self.cell_size_bits
            .store(cell_size.to_bits(), Ordering::SeqCst);
        Ok(dmesh)
    }

    /// C# `BuildCpuDistances`: the unsigned band distances and the grid origin.
    fn build_cpu_distances(
        &self,
        dmesh: &DMesh3,
        cancel: &CancelToken,
    ) -> Result<(DenseGrid3f, Vector3), SdfGridError> {
        let cell_size = self.cell_size();
        let spatial = DMeshAABBTree3::new(dmesh);
        let cancel_f = || cancel.is_cancelled();
        let mut distances = NarrowBandDistanceGrid {
            mesh: dmesh,
            spatial: &spatial,
            cell_size: cell_size as f32,
            narrow_band_max_distance: self.max_radius + (2.0 * cell_size),
            pad_width: self.max_radius + (2.0 * cell_size),
            cancel: &cancel_f,
            flood_progress: self.flood_progress.clone(),
            flood_voxels_processed: 0,
            grid_origin: [0.0; 3],
        };
        let grid = distances.compute();
        self.flood_work
            .store(distances.flood_voxels_processed, Ordering::SeqCst);
        if cancel.is_cancelled() {
            return Err(SdfGridError::Cancelled);
        }
        let origin = distances.grid_origin;
        Ok((
            grid,
            Vector3::new(origin[0] as f64, origin[1] as f64, origin[2] as f64),
        ))
    }

    /// C# `BuildWithInjectedDistances`: the grid laid out exactly as g3's `Compute` does with
    /// `PadWidth` set (f32 origin, f32 cell size, the same truncating counts), so the injected
    /// distances land on exactly the samples the CPU flood would have written.
    fn build_with_injected_distances(
        &self,
        dmesh: &DMesh3,
        builder: &dyn DistanceGridBuilder,
        cancel: &CancelToken,
    ) -> Result<(DenseGrid3f, Vector3), SdfGridError> {
        let cell_size = self.cell_size();
        let bounds = dmesh.cached_bounds();
        let dx = cell_size as f32;
        let pad = (self.max_radius + (2.0 * cell_size)) as f32;
        let origin: [f32; 3] = std::array::from_fn(|a| bounds.min[a] as f32 - pad);
        let max: [f32; 3] = std::array::from_fn(|a| bounds.max[a] as f32 + pad);
        let ni = ((max[0] - origin[0]) / dx) as i32 + 1;
        let nj = ((max[1] - origin[1]) / dx) as i32 + 1;
        let nk = ((max[2] - origin[2]) / dx) as i32 + 1;
        let grid_origin = Vector3::new(origin[0] as f64, origin[1] as f64, origin[2] as f64);

        let mut triangles = Vec::with_capacity(dmesh.triangle_count() * 9);
        for tid in dmesh.triangle_indices() {
            for p in dmesh.get_tri_vertices(tid as usize) {
                triangles.push((p[0] - grid_origin.x) as f32);
                triangles.push((p[1] - grid_origin.y) as f32);
                triangles.push((p[2] - grid_origin.z) as f32);
            }
        }

        if cancel.is_cancelled() {
            return Err(SdfGridError::Cancelled);
        }

        // g3's upper bound for samples the band never reached.
        let cap = (ni + nj + nk) as f32 * dx;
        let distances = builder
            .compute_distances(&DistanceGridRequest {
                triangles: &triangles,
                ni,
                nj,
                nk,
                cell_size: dx,
                band: (self.max_radius + (2.0 * cell_size)) as f32,
                cap,
                stage_done: None,
            })
            .map_err(|error| match error {
                DistanceBuildError::Cancelled => SdfGridError::Cancelled,
                DistanceBuildError::Failed(message) => SdfGridError::DistanceFill(message),
            })?;
        let mut field = DenseGrid3f::new(
            ni.max(0) as usize,
            nj.max(0) as usize,
            nk.max(0) as usize,
            0.0,
        );
        let size = field.size();
        // C# `Array.Copy(distances, field.Buffer, field.size)` throws on a short answer.
        if distances.len() < size {
            return Err(SdfGridError::DistanceFill(format!(
                "The distance fill returned {} distances for a grid of {size} samples.",
                distances.len()
            )));
        }
        field.buffer.copy_from_slice(&distances[..size]);
        Ok((field, grid_origin))
    }

    /// C# `Extract`: the surface at signed distance `iso` from the part (positive grows it,
    /// negative shrinks it), building the field first if needed.
    pub fn extract(&self, iso: f64, cancel: &CancelToken) -> Result<Mesh, SdfGridError> {
        if iso.abs() > self.max_radius {
            return Err(SdfGridError::OutOfRange(format!(
                "This field holds offsets up to {} either way; build it with maxRadius at least |iso| (the largest Dilate radius or Erode / Hollow depth of the drag).",
                self.max_radius
            )));
        }
        self.build(cancel)?;
        let built = self.built.get().ok_or(SdfGridError::Cancelled)?;
        extract(&built.field, built.origin, self.cell_size(), iso, cancel)
            .ok_or(SdfGridError::Cancelled)
    }
}

/// C# `ApplyCrossingSigns`: negates every voxel that at least two of the three axis crossing
/// parities put inside.
fn apply_crossing_signs(
    mesh: &DMesh3,
    field: &mut DenseGrid3f,
    origin: Vector3,
    dx: f64,
    cancel: &CancelToken,
) -> Result<(), SdfGridError> {
    let size = [field.ni, field.nj, field.nk];
    let stride = [1, field.ni, field.ni * field.nj];
    let origin = [origin.x, origin.y, origin.z];
    let total = field.size();
    let mut crossings = vec![0i32; total];
    let mut inside_votes = vec![0u8; total];

    for axis in 0..3 {
        if cancel.is_cancelled() {
            return Err(SdfGridError::Cancelled);
        }
        let b = (axis + 1) % 3;
        let c = (axis + 2) % 3;
        crossings.iter_mut().for_each(|count| *count = 0);

        // A crossing at line coordinate t lands in the first voxel at or past it, so a running
        // parity along the line is odd exactly for the voxels inside.
        for t in 0..mesh.triangle_count() {
            if cancel.is_cancelled() {
                return Err(SdfGridError::Cancelled);
            }
            let [p0, p1, p2] = mesh.get_tri_vertices(t);
            let scaled = |p: [f64; 3], a: usize| (p[a] - origin[a]) / dx;
            let (a0, a1, a2) = (scaled(p0, axis), scaled(p1, axis), scaled(p2, axis));
            let (b0, b1, b2) = (scaled(p0, b), scaled(p1, b), scaled(p2, b));
            let (c0, c1, c2) = (scaled(p0, c), scaled(p1, c), scaled(p2, c));
            let b_min = 0.max(b0.min(b1.min(b2)).ceil() as i32);
            let b_max = (size[b] as i32 - 1).min(b0.max(b1.max(b2)).floor() as i32);
            let c_min = 0.max(c0.min(c1.min(c2)).ceil() as i32);
            let c_max = (size[c] as i32 - 1).min(c0.max(c1.max(c2)).floor() as i32);
            for jc in c_min..=c_max {
                for jb in b_min..=b_max {
                    // Batty's exact orientation test with its tie-break: a line through a shared
                    // edge or vertex is counted for exactly one of the triangles.
                    let Some((w0, w1, w2)) =
                        point_in_triangle_2d(jb as f64, jc as f64, b0, c0, b1, c1, b2, c2)
                    else {
                        continue;
                    };
                    let hit = (w0 * a0 + w1 * a1 + w2 * a2) / (w0 + w1 + w2);
                    let mut ia = hit.ceil() as i32;
                    if ia < 0 {
                        ia = 0;
                    } else if ia >= size[axis] as i32 {
                        continue;
                    }
                    crossings[ia as usize * stride[axis]
                        + jb as usize * stride[b]
                        + jc as usize * stride[c]] += 1;
                }
            }
        }

        for line in 0..size[b] * size[c] {
            let jb = line % size[b];
            let jc = line / size[b];
            let line_start = jb * stride[b] + jc * stride[c];
            let mut parity = 0;
            for ia in 0..size[axis] {
                let index = line_start + ia * stride[axis];
                parity ^= crossings[index] & 1;
                inside_votes[index] += parity as u8;
            }
        }
    }

    if cancel.is_cancelled() {
        return Err(SdfGridError::Cancelled);
    }
    for (value, &votes) in field.buffer.iter_mut().zip(&inside_votes) {
        if votes >= 2 {
            *value = -*value;
        }
    }
    Ok(())
}
