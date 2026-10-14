//! Where a cut's TANGENT RUN stops. The run is the stretch of the cut's own line the region's kept
//! triangles hold (`surface_walkback_tangent_runs::runs_along`), and it has to stop at the band's
//! ends rather than carry on down a line that happens to be an edge of the part for its whole
//! length - the crease a profile RESTING on one lands on. The cut never split that edge, so the
//! reading is made at the VERTICES: an edge is in the run while both of its ends are inside the
//! band.

use geometry_processing::bevel::surface::surface_walkback_tangent_runs::runs_along;
use geometry_processing::bevel::surface::{
    SurfaceBandEnd, SurfaceFacetCut, SurfaceLine, SurfaceVertexPool,
};
use vector_math::{Vector3, Vector3Float};

/// The run stops at the last vertex inside the band even where the straddling edge's MIDDLE is
/// inside it - an edge read at its midpoint is kept or dropped by up to half its own length.
#[test]
fn a_run_stops_at_the_last_vertex_inside_the_band_and_not_at_the_straddling_edges_middle() {
    // The line is the part's own crease, running up z through the band and well past it; the band
    // is -6..6 of it. The long edge runs from -5 to 16, so its middle stands at 5.5 - INSIDE the
    // end - while its far vertex stands ten millimetres beyond.
    let below = Vector3::new(0.0, 0.0, -5.0);
    let inside = Vector3::new(0.0, 0.0, 5.0);
    let beyond = Vector3::new(0.0, 0.0, 16.0);

    let pool = SurfaceVertexPool::new(&[
        Vector3Float::from_vector3(below),
        Vector3Float::from_vector3(inside),
        Vector3Float::from_vector3(beyond),
        Vector3Float::from_vector3(Vector3::new(1.0, 0.0, 0.0)),
        Vector3Float::from_vector3(Vector3::new(1.0, 0.0, 5.0)),
    ]);

    let kept = vec![(0, 1, 3), (0, 2, 4)];

    let runs = runs_along(&the_cut(the_band()), &kept, &pool, None);

    assert_eq!(
        runs.len(),
        1,
        "one stretch of the line is held by these two triangles - instead {}",
        runs.len()
    );

    let zs: Vec<f64> = runs[0].iter().map(|&v| pool.position_of(v).z).collect();
    assert_eq!(
        zs,
        vec![-5.0, 5.0],
        "the run reaches the last vertex inside the band and stops, though the edge that would \
         carry it to 16 has its own middle at 5.5, inside the end"
    );
}

/// And with no ends at all - the ordinary cut, whose line the facets are split along - the same run
/// takes the whole stretch, so what stops it above is the BAND and not the reading being cautious.
#[test]
fn with_no_ends_the_run_takes_the_whole_stretch_of_the_line() {
    let pool = SurfaceVertexPool::new(&[
        Vector3Float::from_vector3(Vector3::new(0.0, 0.0, -5.0)),
        Vector3Float::from_vector3(Vector3::new(0.0, 0.0, 5.0)),
        Vector3Float::from_vector3(Vector3::new(0.0, 0.0, 16.0)),
        Vector3Float::from_vector3(Vector3::new(1.0, 0.0, 0.0)),
        Vector3Float::from_vector3(Vector3::new(1.0, 0.0, 5.0)),
    ]);

    let kept = vec![(0, 1, 3), (0, 2, 4)];

    let runs = runs_along(&the_cut(Vec::new()), &kept, &pool, None);

    assert_eq!(runs.len(), 1);

    let zs: Vec<f64> = runs[0].iter().map(|&v| pool.position_of(v).z).collect();
    assert_eq!(
        zs,
        vec![-5.0, 5.0, 16.0],
        "nothing bounds the line, so every edge standing on it is the strip's seam"
    );
}

/// The band this fixture's cut is bounded by: from -6 to 6 along the line.
fn the_band() -> Vec<SurfaceBandEnd> {
    vec![
        SurfaceBandEnd::new(Vector3::new(0.0, 0.0, 6.0), -Vector3::UNIT_Z),
        SurfaceBandEnd::new(Vector3::new(0.0, 0.0, -6.0), Vector3::UNIT_Z),
    ]
}

/// One cut along z through the origin, bounded by `ends`.
fn the_cut(ends: Vec<SurfaceBandEnd>) -> SurfaceFacetCut {
    SurfaceFacetCut::with_ends(
        0,
        true,
        SurfaceLine::new(Vector3::ZERO, Vector3::UNIT_Z),
        Vector3::UNIT_X,
        0,
        ends,
        Vector3::new(0.0, 0.0, -5.0),
        Vector3::new(0.0, 0.0, 5.0),
        0,
        None,
    )
}
