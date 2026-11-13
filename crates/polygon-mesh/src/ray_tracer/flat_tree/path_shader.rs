//! The GPU path loop's source: [`PATH_WGSL`] is `trace.wgsl`, `scatter.wgsl`, `path.wgsl`,
//! `see_through.wgsl` (the parts one straight line has gone through), `sun.wgsl` (the sky, the sun
//! and the shadow ray toward it), `catcher.wgsl` and `seam.wgsl` (glass lying on glass)
//! concatenated, with the compute entry point [`PATH_ENTRY_POINT`]. A caller compiles it once with
//! the three bind groups `path.wgsl`'s header lists; a [`super::GpuRenderScene`] packs the buffers
//! and [`super::pixel_slice_bytes`] the per-dispatch uniform. `TRACE_WGSL` and `SCATTER_WGSL` keep
//! their own entry-point-free or check-only uses and are the same text. WGSL declarations at
//! module scope may come in any order, so the order only keeps each file's header true.

/// The WGSL source of the path loop: traversal, scattering model and integrator in one module.
pub const PATH_WGSL: &str = concat!(
    include_str!("trace.wgsl"),
    "\n",
    include_str!("scatter.wgsl"),
    "\n",
    include_str!("path.wgsl"),
    "\n",
    include_str!("see_through.wgsl"),
    "\n",
    include_str!("sun.wgsl"),
    "\n",
    include_str!("catcher.wgsl"),
    "\n",
    include_str!("seam.wgsl"),
);

/// The compute entry point of [`PATH_WGSL`].
pub const PATH_ENTRY_POINT: &str = "path_trace_main";

/// The workgroup size of [`PATH_ENTRY_POINT`]: pixels per workgroup a dispatch covers.
pub const PATH_WORKGROUP_PIXELS: u32 = 64;
