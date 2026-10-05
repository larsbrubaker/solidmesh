//! Where a strip STOPS: the ring at an open end and the notch it cuts in the face beyond, and the
//! gathering of every free end of every chain into the corners they stand at. Ports
//! `Bevel/Surface/SurfaceStripEnds.cs` (C# `internal static class`; a module of free functions
//! here). What is here is the one ruler every end reads its seam with ([`meets_the_seam`]); the
//! open end and the corner gathering land with later packages. The third way a strip stops -
//! against the next strip of its own chain - is `SurfaceStripMitre`.

use super::SurfaceVertexPool;

/// C# `MeetsTheSeam`: how near two points of the seam have to come to be the same point - the
/// pool's own [`SurfaceVertexPool::near_enough`], which is ten times its weld because these are
/// float positions read back and crossed with planes, and follows the part's size because a float's
/// ulp does. A seam that is a rounding out is a crack either way.
pub fn meets_the_seam(pool: &SurfaceVertexPool) -> f64 {
    pool.near_enough()
}
