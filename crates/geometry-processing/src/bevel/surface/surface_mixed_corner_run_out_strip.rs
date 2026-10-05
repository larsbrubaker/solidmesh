//! One piece of a runout, between two neighbouring stations, drawn as the STRIP it is rather than
//! triangulated as a ring. Ports `Bevel/Surface/SurfaceMixedCornerRunOutStrip.cs` (C# `internal
//! static class`; a module of free functions here). What is here is the one triangle the strip is
//! drawn with ([`draw`]); the strip's walk lands with a later package.
//!
//! The two columns are the same rows of the patch's own profile, turned to their two meridians,
//! each cut short where the wall swallowed it and each ending at its trim point on the wall. So the
//! piece has a structure: quads down the rows both columns still have, and then a fan over what
//! the shorter column has given up, out of that column's own trim point. These pieces are warped -
//! they lie on one turn of a torus, not in a plane - so no ear clip's projection serves them
//! ([`super::surface_ring_ears`] stays for the end sections that really are general rings).
//!
//! **No ruler for slivers.** Only a triangle whose corners are the same VERTEX twice is dropped - an
//! index coincidence, which is the pool having welded two of them into one - so nothing is decided
//! by a tolerance here beyond the fan's own smallest triangle.

use vector_math::Vector3;

use super::{surface_fan, SurfaceVertexPool};

/// C# `Draw`: one triangle of the strip, unless two of its corners are the same VERTEX - which is
/// the trim point welded onto the row above it, or a column that ended where the other did, and
/// has no area to draw either way.
pub fn draw(
    pool: &SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    apex: i32,
    from: i32,
    to: i32,
    outward: Vector3,
) {
    if apex == from || apex == to || from == to {
        return;
    }

    surface_fan::emit_triangle_facing(pool, faces, apex, from, to, outward);
}
