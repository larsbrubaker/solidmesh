//! The two rulers a facet cut
//! reads a band end with - the WELD its removal test uses and the pool's NEARNESS its clip uses -
//! and the facet that stands in the gap between them, removed by one and kept whole by the other
//! (held as a measurement, a queue entry rather than a fix).

use geometry_processing::bevel::surface::{
    surface_cut_clip, SurfaceBandEnd, SurfaceFacetCut, SurfaceLine, SurfaceVertexPool,
};
use vector_math::{Vector3, Vector3Float};

#[test]
fn a_facet_inside_the_band_by_between_a_weld_and_a_nearness_is_removed_and_kept_at_once() {
    let mut pool = a_pool_twenty_across();

    let weld = pool.weld_distance();

    assert!(
        (pool.near_enough() - weld * 10.0).abs() <= weld * 1e-9,
        "the whole of this fixture is the gap between the two, and it is a factor of ten"
    );

    // The cut: everything with x below 1 is the band, bounded at y = 0 with the band on the y > 0
    // side.
    let cut = a_cut_keeping_x_above(
        1.0,
        vec![SurfaceBandEnd::new(
            Vector3::ZERO,
            Vector3::new(0.0, 1.0, 0.0),
        )],
    );

    // A triangle lying mostly beyond that end, poking into the band by five welds - half a
    // nearness, squarely between the two rulers.
    let pokes_in_by = weld * 5.0;

    let facet = vec![
        pool.add(Vector3::new(0.5, pokes_in_by, 0.0)),
        pool.add(Vector3::new(0.8, -1.0, 0.0)),
        pool.add(Vector3::new(0.2, -1.0, 0.0)),
    ];

    let tip = pool.position_of(facet[0]);

    assert!(
        cut.removes(tip, surface_cut_clip::on_the_cut(&pool)),
        "read on the weld, {tip:?} is inside every end of the band and past the tangent plane"
    );

    let pieces = cut.clip_to_pieces(&facet, &mut pool, None);

    // The disagreement itself: read on the nearness the facet is wholly beyond the end, so it
    // comes back exactly as it went in.
    assert_eq!(pieces.len(), 1);

    let mut came_back = pieces[0].clone();
    came_back.sort_unstable();
    let mut went_in = facet.clone();
    went_in.sort_unstable();
    assert_eq!(
        came_back, went_in,
        "the clip hands back the very triangle it was given, having taken nothing off it, while the \
         cut says it removes {tip:?} - the two rulers disagreeing"
    );
}

#[test]
fn either_side_of_the_gap_the_two_rulers_agree() {
    let mut pool = a_pool_twenty_across();
    let weld = pool.weld_distance();
    let on_the_cut = surface_cut_clip::on_the_cut(&pool);
    let cut = a_cut_keeping_x_above(
        1.0,
        vec![SurfaceBandEnd::new(
            Vector3::ZERO,
            Vector3::new(0.0, 1.0, 0.0),
        )],
    );

    // BEYOND the end by more than a weld, which is where the cut's own reading finally stops.
    let beyond = a_triangle_reaching_to(&mut pool, -weld * 2.0);

    assert!(
        !cut.removes(pool.position_of(beyond[0]), on_the_cut),
        "two welds past the end is past it by the cut's own ruler as well"
    );
    assert_eq!(
        cut.clip_to_pieces(&beyond, &mut pool, None).len(),
        1,
        "and the clip gives it up whole, which is the two agreeing"
    );

    // Past the nearness: both take material.
    let well_in = a_triangle_reaching_to(&mut pool, weld * 20.0);

    assert!(cut.removes(pool.position_of(well_in[0]), on_the_cut));
    assert!(
        cut.clip_to_pieces(&well_in, &mut pool, None)
            .iter()
            .any(|piece| !piece.contains(&well_in[0])),
        "twenty welds in is past the nearness too, so the clip splits it and the corner standing in \
         the band is no longer a corner of every piece"
    );
}

fn a_triangle_reaching_to(pool: &mut SurfaceVertexPool, into: f64) -> Vec<i32> {
    vec![
        pool.add(Vector3::new(0.5, into, 0.0)),
        pool.add(Vector3::new(0.8, -1.0, 0.0)),
        pool.add(Vector3::new(0.2, -1.0, 0.0)),
    ]
}

fn a_cut_keeping_x_above(at: f64, ends: Vec<SurfaceBandEnd>) -> SurfaceFacetCut {
    SurfaceFacetCut::with_ends(
        0,
        true,
        SurfaceLine::new(Vector3::new(at, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        Vector3::new(1.0, 0.0, 0.0),
        0,
        ends,
        Vector3::ZERO,
        Vector3::new(0.0, 1.0, 0.0),
        0,
        None,
    )
}

fn a_pool_twenty_across() -> SurfaceVertexPool {
    SurfaceVertexPool::new(&[
        Vector3Float::new(-10.0, -10.0, -10.0),
        Vector3Float::new(10.0, 10.0, 10.0),
    ])
}
