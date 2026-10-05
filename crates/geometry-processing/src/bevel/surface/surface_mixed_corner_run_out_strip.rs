//! One piece of a runout, between two neighbouring stations, drawn as the STRIP it is rather than
//! triangulated as a ring. Ports `Bevel/Surface/SurfaceMixedCornerRunOutStrip.cs` (C# `internal
//! static class`; a module of free functions here). What is here is the one triangle the strip is
//! drawn with ([`draw`]) and the strip's walk ([`emit`]).
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

use super::{surface_fan, surface_ring, SurfaceVertexPool};

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

/// C# `Emit`: the piece between two neighbouring stations' columns `here` and `there` (each its
/// rows from `*_first_row` on, ending in its trim point on the wall), drawn as a strip facing
/// `outward`: the top rim (if any) fanned out of the further column's top, quads down the rows both
/// columns hold, the rows the PLANE took off one of them fanned to the other's first vertex, and
/// what the WALL took off the bottom fanned out of the shorter column's trim point.
#[allow(clippy::too_many_arguments)]
pub fn emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    here: &[i32],
    there: &[i32],
    here_first_row: i32,
    there_first_row: i32,
    top_rim: Option<&[i32]>,
    outward: Vector3,
) {
    // The top rim first, fanned out of the further column's own top.
    if let Some(top_rim) = top_rim {
        if !top_rim.is_empty() && !here.is_empty() && !there.is_empty() {
            let apex_of_the_rim = there[0];
            for at in 0..top_rim.len() {
                let to = if at + 1 < top_rim.len() {
                    top_rim[at + 1]
                } else {
                    here[0]
                };
                draw(pool, faces, apex_of_the_rim, top_rim[at], to, outward);
            }
        }
    }

    // Which rows each column holds, lined up by row NUMBER.
    let here_last = here_first_row + here.len() as i32 - 2;
    let there_last = there_first_row + there.len() as i32 - 2;
    let first_shared = here_first_row.max(there_first_row);
    let last_shared = here_last.min(there_last);

    let h = |row: i32| here[(row - here_first_row) as usize];
    let t = |row: i32| there[(row - there_first_row) as usize];
    for row in first_shared..last_shared {
        // Without repeats: two columns can hold the SAME vertex in one row's slot.
        let quad = surface_ring::without_repeats(&[h(row), h(row + 1), t(row + 1), t(row)]);
        surface_fan::emit_facing(&quad, pool, faces, outward);
    }

    // What the PLANE has taken off the top of one of them, closed to the other's first vertex.
    if here_first_row != there_first_row {
        let higher = if here_first_row < there_first_row {
            here
        } else {
            there
        };
        let higher_first = here_first_row.min(there_first_row);
        let apex = if here_first_row < there_first_row {
            there
        } else {
            here
        }[0];

        let mut row = higher_first;
        while row < first_shared && ((row + 1 - higher_first) as usize) < higher.len() {
            draw(
                pool,
                faces,
                apex,
                higher[(row - higher_first) as usize],
                higher[(row + 1 - higher_first) as usize],
                outward,
            );
            row += 1;
        }
    }

    // And what the WALL has taken off the bottom, fanned out of the shorter column's trim point.
    let here_is_deeper = here_last >= there_last;
    let deeper = if here_is_deeper { here } else { there };
    let deeper_first = if here_is_deeper {
        here_first_row
    } else {
        there_first_row
    };
    let shallower = if here_is_deeper { there } else { here };
    let shallower_last = here_last.min(there_last);
    let foot = shallower[shallower.len() - 1];

    let mut rim = Vec::with_capacity(deeper.len() + 1);
    if last_shared >= first_shared {
        let shallower_first = if here_is_deeper {
            there_first_row
        } else {
            here_first_row
        };
        rim.push(shallower[(shallower_last - shallower_first) as usize]);
    }

    let mut row = last_shared.max(first_shared);
    while row <= deeper_first + deeper.len() as i32 - 2 {
        rim.push(deeper[(row - deeper_first) as usize]);
        row += 1;
    }

    rim.push(deeper[deeper.len() - 1]);

    for at in 0..rim.len() - 1 {
        draw(pool, faces, foot, rim[at], rim[at + 1], outward);
    }
}
