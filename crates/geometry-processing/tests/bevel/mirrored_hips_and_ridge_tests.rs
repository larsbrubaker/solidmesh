//! "Full Front Edges Should Work" - a hip roof's ridge picked with the two hips at one of its ends,
//! as the app hands it to the kernel. The part and the pick are mirror-symmetric across z = 10, so
//! the rounded result must be too; each hip's end stays on the part's side of the base it meets
//! obliquely; and the Boolean kernel does not read the answer SelfIntersecting. The app-route build
//! and the mirror measure are `mirrored_pick_tests`'.

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::level_line::LevelLineMesh;
use solidmesh_math::Vector3;
use solidmesh_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::level_line_fixture::LevelLineFixture;
use crate::mirrored_pick_tests::{build_as_the_app_route, describe, mirror_distances};
use crate::sweep_mesh_measures;

#[test]
fn the_ridge_and_both_hips_round_the_same_on_both_sides() {
    let mesh = build();
    let worst = mirror_distances(&mesh.borrow());

    assert!(worst[0].1 <= 0.025, "{}", describe(&worst));
}

/// Each hip ends at a base corner, where the base (y = 0, the part below it) is a third face the
/// hip's end meets obliquely. The strip's rows there must stay on the part's side of the base: no
/// vertex past its plane and no edge through a base triangle.
#[test]
fn the_hip_ends_stay_on_the_part_side_of_the_base() {
    let mesh = build();
    let mesh = mesh.borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = |v: i32| mesh.position(v);

    let mut used: Vec<i32> = Vec::new();
    for &(a, b, c) in &kept {
        for v in [a, b, c] {
            if !used.contains(&v) {
                used.push(v);
            }
        }
    }
    let mut above: Vec<Vector3> = used.iter().map(|&v| at(v)).filter(|v| v.y > 1e-4).collect();
    above.sort_by(|p, q| q.y.total_cmp(&p.y));
    assert_eq!(
        above.len(),
        0,
        "past the base: {:?}",
        above.iter().take(5).collect::<Vec<_>>()
    );

    let on_base: Vec<(i32, i32, i32)> = kept
        .iter()
        .copied()
        .filter(|&(a, b, c)| [a, b, c].iter().all(|&v| at(v).y.abs() <= 1e-4))
        .collect();
    let mut piercings: Vec<String> = Vec::new();
    for &(a, b, c) in &kept {
        let corners = [a, b, c];
        for e in 0..3 {
            let (p, q) = (corners[e], corners[(e + 1) % 3]);
            for &(ba, bb, bc) in on_base
                .iter()
                .filter(|&&(ba, bb, bc)| ![ba, bb, bc].iter().any(|v| corners.contains(v)))
            {
                if pierces(at(p), at(q), at(ba), at(bb), at(bc)) {
                    piercings.push(format!(
                        "{:?}-{:?} through {:?} {:?} {:?}",
                        at(p),
                        at(q),
                        at(ba),
                        at(bb),
                        at(bc)
                    ));
                }
            }
        }
    }

    assert_eq!(
        piercings.len(),
        0,
        "{}",
        piercings
            .iter()
            .take(3)
            .cloned()
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// The app publishes the one-pick answer only if the Boolean kernel does not read it
/// SelfIntersecting.
#[test]
fn the_kernel_does_not_read_it_self_intersecting() {
    let mesh = build();
    assert_ne!(
        boolean_kernel_gate::verdict(&mesh.borrow()),
        BooleanOperandVerdict::SelfIntersecting
    );
}

/// Whether segment p-q passes strictly through the interior of triangle a, b, c.
fn pierces(p: Vector3, q: Vector3, a: Vector3, b: Vector3, c: Vector3) -> bool {
    let n = (b - a).cross(c - a);
    let dp = (p - a).dot(n);
    let dq = (q - a).dot(n);
    let noise = 1e-9 * n.length();
    if dp * dq >= 0.0 || dp.abs() <= noise || dq.abs() <= noise {
        return false;
    }

    let x = p + ((q - p) * (dp / (dp - dq)));
    (b - a).cross(x - a).dot(n) > 0.0
        && (c - b).cross(x - b).dot(n) > 0.0
        && (a - c).cross(x - c).dot(n) > 0.0
}

fn build() -> Rc<RefCell<LevelLineMesh>> {
    let fixture = Rc::new(LevelLineFixture::load("full-front-ridge-and-hips"));
    let chains = fixture.chains.clone();
    build_as_the_app_route(&fixture, &chains)
}
