//! Where a strip STOPS: the ring at an open end and the notch it cuts in the face beyond, and the
//! gathering of every free end of every chain into the corners they stand at. Ports
//! `Bevel/Surface/SurfaceStripEnds.cs` (C# `internal static class`; a module of free functions
//! here). What is here is the one ruler every end reads its seam with ([`meets_the_seam`]), the
//! run-off stretch at an end ([`try_the_stretch_at`]) and the junction at a vertex
//! ([`the_junction_at`]); the open end and the corner gathering land with later packages. The third way a strip stops -
//! against the next strip of its own chain - is `SurfaceStripMitre`.

use super::SurfaceVertexPool;

/// C# `MeetsTheSeam`: how near two points of the seam have to come to be the same point - the
/// pool's own [`SurfaceVertexPool::near_enough`], which is ten times its weld because these are
/// float positions read back and crossed with planes, and follows the part's size because a float's
/// ulp does. A seam that is a rounding out is a crack either way.
pub fn meets_the_seam(pool: &SurfaceVertexPool) -> f64 {
    pool.near_enough()
}

/// The one run-off stretch at edge `edge_index`'s start (or end) - from either side's entry of
/// `over_a_stretch` (`None`, C#'s null, is none) - or `None` where neither side runs off there.
/// Refused (at `corner`) where both sides run off at that end.
pub fn try_the_stretch_at(
    over_a_stretch: Option<&[[Option<super::SurfaceBandRunsOffOverAStretch>; 2]]>,
    edge_index: usize,
    at_the_start: bool,
    corner: vector_math::Vector3,
) -> Result<Option<super::SurfaceBandRunsOffOverAStretch>, crate::bevel::BevelAdvisory> {
    let Some(over_a_stretch) = over_a_stretch else {
        return Ok(None);
    };

    let mut stretch = None;

    for side in &over_a_stretch[edge_index] {
        let Some(here) = side
            .as_ref()
            .filter(|here| here.at_the_start == at_the_start)
        else {
            continue;
        };

        if stretch.is_some() {
            return Err(crate::bevel::BevelAdvisory::new(
                crate::bevel::BevelAdvisoryCode::SurfaceStripRunsOffBothSides,
                vec![corner.into()],
            ));
        }

        stretch = Some(here.clone());
    }

    Ok(stretch)
}

/// The junction standing at `vertex`, where one of `junctions` does.
pub fn the_junction_at(
    junctions: Option<&[std::rc::Rc<super::SurfaceMixedJunction>]>,
    vertex: i32,
) -> Option<std::rc::Rc<super::SurfaceMixedJunction>> {
    junctions?
        .iter()
        .find(|junction| junction.vertex == vertex)
        .cloned()
}
