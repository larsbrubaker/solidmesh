//! The convex cell bound over a field with an END CLIP. A clipped sample's `distance - value` is
//! NOT a setback, and the plane raises the field above every union-of-balls term - either one alone
//! would let the bound stand under the field and retire the cell holding the peak. Also holds
//! `greatest_by_sampling`, which `segment_cell_bound_tests` shares.

use std::rc::Rc;

use solidmesh::bevel::level_line::{
    convex_cell_bound, ChainField, EndRayBound, PickField, ScalarField,
};
use solidmesh_math::seeded_random::SeededRandom;
use solidmesh_math::{ieee_math, Vector3};

// A chain along -x ending at the origin, setback 1, its last end cut by a plane leaning 80 degrees
// off the chain's own direction - so at (-2, 1) the plane reads 0.638 against the chain's 0 and the
// sample is clipped, with distance - value = 1.598 against the real setback of 1.
fn normal() -> Vector3 {
    let angle = std::f64::consts::PI * 80.0 / 180.0;
    Vector3::new(angle.cos(), angle.sin(), 0.0)
}

fn clipped() -> ChainField {
    ChainField::from_polyline_bounded(
        &[Vector3::new(-10.0, 0.0, 0.0), Vector3::ZERO],
        false,
        &[1.0, 1.0],
        f64::INFINITY,
        (false, true),
        (
            None,
            Some(EndRayBound {
                reach_per_setback: 4.0,
                plane_normal: normal(),
                ..EndRayBound::default()
            }),
        ),
    )
    .expect("a valid chain")
}

#[test]
fn a_cell_with_a_clipped_corner_is_not_bound_under_its_peak() {
    let field = clipped();
    let p = Vector3::new(-2.0, 1.0, 0.0);
    let sample = field.evaluate(p).expect("no budget");
    assert!(
        sample.distance - sample.value > 1.5,
        "a clipped sample's distance - value is not the setback of 1"
    );

    // Found by the random sweep below: every corner reads the plane, and the bound stood 0.2 under
    // the field.
    let a = Vector3::new(-3.4984, -0.229, 0.0);
    let b = Vector3::new(-4.2183, -0.0166, 0.0);
    let c = Vector3::new(-3.6323, 0.1864, 0.0);
    assert!(
        chain_bound(&field, a, b, c)
            >= greatest_by_sampling(&|q| value(&field, q), a, b, c, 40) - 1e-9
    );
}

/// Seeded random cells round the clipped end, of a lone clipped chain and of a pick holding it.
#[test]
fn random_cells_round_a_clipped_end_are_never_bound_under_the_field() {
    let clipped = Rc::new(clipped());
    let other = Rc::new(
        ChainField::from_polyline(
            &[Vector3::new(-8.0, 4.0, 0.0), Vector3::new(1.0, 5.0, 0.0)],
            false,
            &[0.6, 1.4],
        )
        .expect("a valid chain"),
    );
    let pick = PickField::new(vec![clipped.clone(), other], &[0, 1]).expect("a valid pick");
    let mut random = SeededRandom::new(20260924);
    for _ in 0..400 {
        let a = {
            let x = (random.next_double() * 8.0) - 6.0;
            let y = (random.next_double() * 6.0) - 1.0;
            Vector3::new(x, y, 0.0)
        };
        let b = {
            let x = (random.next_double() * 4.0) - 2.0;
            let y = (random.next_double() * 4.0) - 2.0;
            a + Vector3::new(x, y, 0.0)
        };
        let c = {
            let x = (random.next_double() * 4.0) - 2.0;
            let y = (random.next_double() * 4.0) - 2.0;
            a + Vector3::new(x, y, 0.0)
        };
        assert!(
            chain_bound(&clipped, a, b, c)
                >= greatest_by_sampling(&|q| value(clipped.as_ref(), q), a, b, c, 40) - 1e-9,
            "the lone clipped chain over ({a:?}), ({b:?}), ({c:?})"
        );
        assert!(
            pick_bound(&pick, a, b, c)
                >= greatest_by_sampling(&|q| value(&pick, q), a, b, c, 40) - 1e-9,
            "the pick over ({a:?}), ({b:?}), ({c:?})"
        );
    }
}

fn value(field: &dyn ScalarField, point: Vector3) -> f64 {
    field.value_at(point).expect("no budget")
}

fn chain_bound(field: &ChainField, a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let reader = field.end_plane_reader();
    convex_cell_bound::over_triangle(
        &field.evaluate(a).expect("no budget"),
        &field.evaluate(b).expect("no budget"),
        &field.evaluate(c).expect("no budget"),
        a,
        b,
        c,
        reader.as_deref(),
    )
}

fn pick_bound(field: &PickField, a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let reader = field.end_plane_reader();
    convex_cell_bound::over_triangle(
        &field.evaluate(a).expect("no budget"),
        &field.evaluate(b).expect("no budget"),
        &field.evaluate(c).expect("no budget"),
        a,
        b,
        c,
        reader.as_deref(),
    )
}

/// The field's greatest value over a dense barycentric grid of the cell, corners and edges
/// included (40 steps a side is the usual density).
pub fn greatest_by_sampling(
    value_at: &dyn Fn(Vector3) -> f64,
    a: Vector3,
    b: Vector3,
    c: Vector3,
    steps: i32,
) -> f64 {
    let mut greatest = f64::NEG_INFINITY;
    for i in 0..=steps {
        for j in 0..=(steps - i) {
            let q =
                a + ((b - a) * (i as f64 / steps as f64)) + ((c - a) * (j as f64 / steps as f64));
            greatest = ieee_math::max_f64(greatest, value_at(q));
        }
    }

    greatest
}
