//! Port of the test fixture `Fixtures/BevelObstructionTestMeshes.cs`: parts built around a fillet's
//! setback, each putting a feature INSIDE the band of the 20 mm cube's top edge at x = 0 (or, for
//! the blocked crease, across an L's concave foot) to ask what happens to it.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::topology_test_meshes::create_grid_solid;

/// C# `BlockedAt`: where the block starts, which is where the selected crease has to stop.
pub const BLOCKED_AT: f64 = 10.0;

/// C# `BlockedSpan`: how long the block is, and how much of the part is left past the crease's
/// end.
pub const BLOCKED_SPAN: f64 = 2.0;

/// C# `CreateRibNearATopEdge`: a 20 mm cube with a 2 wide, 4 tall RIB on its top face running the
/// full length in y, parallel to the top edge at x = 0 and `rib_distance` in from it - a feature
/// that ENCROACHES on a fillet's tangent band from the material side.
pub fn create_rib_near_a_top_edge(rib_distance: f64) -> Mesh {
    let across_x = [0.0, rib_distance, rib_distance + 2.0, 20.0];
    let along_y = [0.0, 20.0];
    let upward = [0.0, 20.0, 24.0];

    // The whole cube, and the one cell above it that is the rib
    create_grid_solid(&across_x, &along_y, &upward, |i, j, k| {
        (0..3).contains(&i) && j == 0 && (0..2).contains(&k) && (k == 0 || i == 1)
    })
}

/// C# `CreateLBossNearATopEdge`: the same cube with an L-shaped boss on its top - x in [1, 2] over
/// the full depth, plus x in [2, 4] for y in [0, 10] - so the notch, x in [2, 4], y in [10, 20],
/// is inside the 5 mm setback and is material the fillet is entitled to cut.
pub fn create_l_boss_near_a_top_edge() -> Mesh {
    let across_x = [0.0, 1.0, 2.0, 4.0, 20.0];
    let along_y = [0.0, 10.0, 20.0];
    let upward = [0.0, 20.0, 24.0];

    create_grid_solid(&across_x, &along_y, &upward, |i, j, k| {
        if !(0..=3).contains(&i) || !(0..=1).contains(&j) || !(0..=1).contains(&k) {
            return false;
        }

        // The whole cube, and above it the L: the long arm at x in [1, 2], and the short one that
        // reaches out to x = 4 over the near half of the depth only.
        k == 0 || i == 1 || (i == 2 && j == 0)
    })
}

/// C# `CreateRibAndBridgeNearATopEdge`: the same cube carrying BOTH a rib standing on the top face
/// inside the band (x in [1, 3]) AND a deck (z in [26, 30]) passing over the band on a post at
/// x in [12, 16], well outside it. The rib's column is the shadow clip's; the bridge's is not.
pub fn create_rib_and_bridge_near_a_top_edge() -> Mesh {
    let across_x = [0.0, 1.0, 3.0, 12.0, 16.0, 20.0];
    let along_y = [0.0, 20.0];
    let upward = [0.0, 20.0, 24.0, 26.0, 30.0];

    create_grid_solid(&across_x, &along_y, &upward, |i, j, k| {
        if !(0..=4).contains(&i) || j != 0 || !(0..=3).contains(&k) {
            return false;
        }

        k == 0 || (k == 1 && (i == 1 || i == 3)) || (k == 2 && i == 3) || k == 3
    })
}

/// C# `CreateBlockedCreaseEnd`: an L prism (wall x in [0, 4] 20 tall, floor x in [4, 12] 4 thick)
/// whose concave foot crease is BLOCKED by a block filling the corner from [`BLOCKED_AT`] to the
/// end, the part stopping [`BLOCKED_SPAN`] mm later with nothing on the far side.
pub fn create_blocked_crease_end() -> Mesh {
    let across_x = [0.0, 4.0, 12.0];
    let along_y = [0.0, BLOCKED_AT, BLOCKED_AT + BLOCKED_SPAN];
    let upward = [0.0, 4.0, 8.0, 20.0];

    create_grid_solid(&across_x, &along_y, &upward, |i, j, k| {
        if !(0..=1).contains(&i) || !(0..=1).contains(&j) || !(0..=2).contains(&k) {
            return false;
        }

        // The wall is the whole of its column; the floor is the bottom of the other one; and the
        // block is the middle of that column over the last span only.
        i == 0 || k == 0 || (k == 1 && j == 1)
    })
}

/// C# `CreateRibEndingAt`.
pub fn create_rib_ending_at(rib_ends_at: f64) -> Mesh {
    let size = 20.0;
    let near = 1.0;
    let far = 3.0;
    let rib_top = 24.0;
    let v = Vector3::new;

    let mut mesh = Mesh::new();

    // The top, z = size, around the rib's foot.
    mesh.create_face(&[
        v(0.0, 0.0, size),
        v(near, 0.0, size),
        v(near, rib_ends_at, size),
    ]);
    mesh.create_face(&[
        v(0.0, 0.0, size),
        v(near, rib_ends_at, size),
        v(0.0, size, size),
    ]);
    mesh.create_face(&[
        v(0.0, size, size),
        v(near, rib_ends_at, size),
        v(near, size, size),
    ]);
    mesh.create_face(&[
        v(near, 0.0, size),
        v(far, 0.0, size),
        v(far, rib_ends_at, size),
        v(near, rib_ends_at, size),
    ]);
    mesh.create_face(&[
        v(far, 0.0, size),
        v(size, 0.0, size),
        v(far, rib_ends_at, size),
    ]);
    mesh.create_face(&[
        v(far, rib_ends_at, size),
        v(size, 0.0, size),
        v(size, size, size),
    ]);
    mesh.create_face(&[
        v(far, rib_ends_at, size),
        v(size, size, size),
        v(far, size, size),
    ]);

    // The front, y = 0.
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(size, 0.0, 0.0), v(size, 0.0, size)]);
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(size, 0.0, size), v(far, 0.0, size)]);
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(far, 0.0, size), v(near, 0.0, size)]);
    mesh.create_face(&[v(0.0, 0.0, 0.0), v(near, 0.0, size), v(0.0, 0.0, size)]);

    // The back, y = size, with the rib's end rising from it.
    mesh.create_face(&[v(0.0, size, 0.0), v(0.0, size, size), v(near, size, size)]);
    mesh.create_face(&[
        v(0.0, size, 0.0),
        v(near, size, size),
        v(near, size, rib_top),
    ]);
    mesh.create_face(&[
        v(0.0, size, 0.0),
        v(near, size, rib_top),
        v(far, size, rib_top),
    ]);
    mesh.create_face(&[v(0.0, size, 0.0), v(far, size, rib_top), v(far, size, size)]);
    mesh.create_face(&[v(0.0, size, 0.0), v(far, size, size), v(size, size, size)]);
    mesh.create_face(&[v(0.0, size, 0.0), v(size, size, size), v(size, size, 0.0)]);

    // The left, right and bottom.
    mesh.create_face(&[
        v(0.0, 0.0, 0.0),
        v(0.0, 0.0, size),
        v(0.0, size, size),
        v(0.0, size, 0.0),
    ]);
    mesh.create_face(&[
        v(size, 0.0, 0.0),
        v(size, size, 0.0),
        v(size, size, size),
        v(size, 0.0, size),
    ]);
    mesh.create_face(&[
        v(0.0, 0.0, 0.0),
        v(0.0, size, 0.0),
        v(size, size, 0.0),
        v(size, 0.0, 0.0),
    ]);

    // The rib.
    mesh.create_face(&[
        v(near, rib_ends_at, size),
        v(near, rib_ends_at, rib_top),
        v(near, size, rib_top),
        v(near, size, size),
    ]);
    mesh.create_face(&[
        v(far, rib_ends_at, size),
        v(far, size, size),
        v(far, size, rib_top),
        v(far, rib_ends_at, rib_top),
    ]);
    mesh.create_face(&[
        v(near, rib_ends_at, size),
        v(far, rib_ends_at, size),
        v(far, rib_ends_at, rib_top),
        v(near, rib_ends_at, rib_top),
    ]);
    mesh.create_face(&[
        v(near, rib_ends_at, rib_top),
        v(far, rib_ends_at, rib_top),
        v(far, size, rib_top),
        v(near, size, rib_top),
    ]);

    mesh.clean_and_merge();
    mesh
}
