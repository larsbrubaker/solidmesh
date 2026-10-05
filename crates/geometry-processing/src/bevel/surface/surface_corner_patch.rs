//! The corner where THREE ROUND strips meet at one vertex of the part
//! (`Bevel/Surface/SurfaceCornerPatch.cs`, a static class): each one ends on the plane through the
//! ball centre square to its own edge, and the spherical triangle between the three end rings is
//! stitched in as the patch - the rung 3 reading of `BevelCornerPatches`, built as surface and
//! shared by vertex rather than built as a solid and subtracted. Here are its nested record, one
//! strip arriving at the corner ([`SurfaceCornerPatchArrival`]), and the cut of one strip's end at
//! the ball's plane ([`try_cut_the_end`]); the rest of the patch's construction lands with later
//! packages.

use std::rc::Rc;

use vector_math::Vector3;

use super::{SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// C# `SurfaceCornerPatch.Arrival` (a readonly struct): one strip arriving at the corner - which
/// strip, what it was planned as, and the two vertices the walkback left where its own two tangent
/// lines ended.
#[derive(Debug, Clone)]
pub struct SurfaceCornerPatchArrival {
    /// The strip arriving.
    pub strip: Rc<SurfaceStrip>,
    /// The edge plan the strip was built from (a C# struct, so held by value).
    pub edge: SurfaceEdgePlan,
    /// The last vertex of the left face's tangent run - rail 0 of the end ring.
    pub on_the_left: i32,
    /// The same for the right face, which is the ring's last rail.
    pub on_the_right: i32,
}

/// C# `TryCutTheEnd`: one strip cut at the plane through the ball centre square to its own edge,
/// as the ring of vertices across it and where along the edge each rail meets that plane (C#'s
/// `ring` and `meetsAt`); `Err` is C#'s `failure`. The ring's two outer rails have to land on the
/// vertices the walkback left at the ends of that strip's tangent runs - the ball's own touch points
/// on the two faces - or the patch would not be rimmed by the faces it stands between. The inner
/// rails' points are added to `pool`.
pub fn try_cut_the_end(
    pool: &mut SurfaceVertexPool,
    arrival: &SurfaceCornerPatchArrival,
    centre: Vector3,
    corner: Vector3,
) -> Result<(Vec<i32>, Vec<f64>), BevelAdvisory> {
    let rail_count = arrival.strip.rail_count();
    let mut ring = vec![0i32; rail_count as usize];
    let mut meets_at = vec![0.0; rail_count as usize];

    for rail in 0..rail_count {
        let Some(param) = arrival.strip.try_cross(rail, centre, arrival.strip.along()) else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![corner.into()],
            ));
        };

        meets_at[rail as usize] = param;

        let at = arrival.strip.point_on(rail, param);

        if rail == 0 || rail == rail_count - 1 {
            let vertex = if rail == 0 {
                arrival.on_the_left
            } else {
                arrival.on_the_right
            };

            if (pool.position_of(vertex) - at).length() > pool.near_enough() {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                ));
            }

            ring[rail as usize] = vertex;
            continue;
        }

        ring[rail as usize] = pool.add(at);
    }

    Ok((ring, meets_at))
}
