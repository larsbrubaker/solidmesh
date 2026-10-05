//! The fixed point the room's largest size is stepped down to (`Bevel/Surface/SurfaceRoomFixedPoint.cs`,
//! a static class: free functions here). Its rulers and steps land here first; the iteration
//! (planning every chain again at each size) lands with a later package.

use std::rc::Rc;

use super::{SurfaceBevelPlan, SurfaceVertexPool};

/// The most times the size is stepped down.
pub const MOST_STEPS: i32 = 5;

/// How near a size has to fit to count as fitting.
pub const FIT_TOLERANCE: f64 = 1e-9;

/// `size` one `ruler` down, where that is still a size; `size` itself otherwise.
pub fn stepped_down(size: f64, ruler: f64) -> f64 {
    if size - ruler > 0.0 {
        size - ruler
    } else {
        size
    }
}

/// The part's weld ruler, read off the first plan with a mesh; zero where none has one.
pub fn ruler_of(plans: &[Rc<SurfaceBevelPlan>]) -> f64 {
    plans
        .iter()
        .find_map(|plan| plan.mesh())
        .map_or(0.0, |mesh| {
            SurfaceVertexPool::near_enough_for(&mesh.vertices)
        })
}
