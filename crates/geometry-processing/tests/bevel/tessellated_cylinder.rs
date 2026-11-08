//! A closed cylinder with fan caps, shared by the tests that pick its top rim
//! (`topology_only_probe_tests.rs` and `chain_field_speed_tests.rs` among them).

#![allow(dead_code)]

use solidmesh_math::Vector3;

use crate::level_line_room_geometry::SolidWithChain;

/// A closed cylinder with fan caps; the returned rim is the top cap's
/// edge, in order.
pub fn tessellated_cylinder(sides: i32, stacks: i32, radius: f64, height: f64) -> SolidWithChain {
    let mut positions = Vec::new();
    let mut indices = Vec::new();

    for s in 0..=stacks {
        let z = (height * s as f64 / stacks as f64) - (height / 2.0);
        for i in 0..sides {
            let angle = (i as f64 / sides as f64) * std::f64::consts::PI * 2.0;
            positions.push(Vector3::new(radius * angle.cos(), radius * angle.sin(), z));
        }
    }

    for s in 0..stacks {
        for i in 0..sides {
            let next = (i + 1) % sides;
            let a = (s * sides) + i;
            let b = (s * sides) + next;
            let c = ((s + 1) * sides) + next;
            let d = ((s + 1) * sides) + i;
            indices.push((a, b, c));
            indices.push((a, c, d));
        }
    }

    let bottom_centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, -height / 2.0));
    let top_centre = positions.len() as i32;
    positions.push(Vector3::new(0.0, 0.0, height / 2.0));

    let mut rim = Vec::new();
    for i in 0..sides {
        let next = (i + 1) % sides;
        indices.push((bottom_centre, next, i));
        indices.push((top_centre, (stacks * sides) + i, (stacks * sides) + next));
        rim.push((stacks * sides) + i);
    }

    (positions, indices, rim)
}
