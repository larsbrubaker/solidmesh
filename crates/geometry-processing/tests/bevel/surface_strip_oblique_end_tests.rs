//! The strip against an OBLIQUE end cut, where the middle rails meet the end further along than the
//! flanks do. A column clamped to zero width still owes the points the end cut left on its chord,
//! and they have to be carried to the first column that has width rather than covered where the
//! chord is not there. Exercises `surface_bevel_stitch::try_stitch` on "Center Flows Through.mcx"'s
//! shape at the size of a cube.

use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::surface_bevel_stitch;
use solidmesh::bevel::surface::surface_mitre_joint_step::SideSeams;
use solidmesh::bevel::surface::{
    SurfaceBevelPlan, SurfaceStrip, SurfaceStripStation, SurfaceVertexPool,
};
use solidmesh::topology::{FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::solids::platonic_solids;
use solidmesh_mesh::Mesh;

/// A column of the strip that collapses to zero width still owes the points the END CUT left on
/// its chord: they are carried forward to the first column that has width, which takes them as
/// extra corners of its own ring. What must not happen is the chord being covered where it is not
/// there - a needle whose three corners are one line, whose normal is whatever the floats say, and
/// which reads against the face beside it as a fold in the published surface.
///
/// The shape is "Center Flows Through.mcx"'s, at the size of a cube: an OBLIQUE end cut, met by the
/// middle rails further along than by the flanks, so a seam's tangent vertex can stand behind it on
/// rails away from its own side. The station there is clamped rail by rail, the column between two
/// clamped rails has no width, and the break on its chord - where the end face's own rim crosses
/// this strip's chord - was coming out as a needle reading 155.6 degrees against the trim fan.
#[test]
fn a_collapsed_column_carries_its_break_to_the_column_that_has_width() {
    let (mut read, mut pool, strip) = a_strip_with_an_oblique_end(false);

    // The break the end cut left between rails 1 and 2, a hair off the chord - the drift the real
    // one has, which is inside the pool's ruler and so tells nobody anything by measuring it.
    let chord_from = read.start.rail[1];
    let chord_to = read.start.rail[2];
    let break_at = pool.add(
        Vector3::lerp(
            pool.position_of(chord_from),
            pool.position_of(chord_to),
            0.97,
        ) + (Vector3::new(0.0, 0.0, 1.0) * 8e-5),
    );

    read.start.between[1] = vec![break_at];

    let mut faces = Vec::new();

    let stitched = surface_bevel_stitch::try_stitch(
        &mut pool,
        &mut faces,
        &[],
        &strip,
        &read.seams,
        0,
        &read.start,
        &read.end,
    );

    assert!(
        stitched.is_ok(),
        "the fixture's own strip has to stitch: {stitched:?}"
    );

    for face in &faces {
        assert!(
            twice_the_area(&pool, *face) > 1e-5,
            "({},{},{}) covers nothing and faces wherever the floats put it, which is a fold \
             against whatever stands beside it",
            face.0,
            face.1,
            face.2
        );
    }

    assert!(
        faces
            .iter()
            .any(|face| corners(*face).contains(&break_at) && twice_the_area(&pool, *face) > 1e-5),
        "the break is a corner of a triangle with width in it, not of a needle"
    );

    assert_eq!(
        uses(&faces, chord_from, chord_to),
        0,
        "nothing may span the break: the face across the chord stops at it, and a triangle \
         spanning it is the T junction a shell does not close over"
    );

    assert_eq!(uses(&faces, chord_from, break_at), 1);
    assert_eq!(
        uses(&faces, break_at, chord_to),
        1,
        "the two halves of the chord are each used once, so they pair with the trim ring"
    );
}

/// A carry and the END ring's own breaks meeting in one column: the carry arrives on the ring the
/// column starts from, the end ring's own on the side the quad walks back along, and both come out
/// corners of triangles with width rather than of needles.
///
/// The two never meet in one LIST - only an end ring has anything between its rails, and the start
/// ring is the only one a carry can leave from - so what this pins is that they meet in one RING,
/// each on its own side of it, and that neither is spanned.
#[test]
fn a_carry_and_the_end_rings_own_break_both_enter_the_last_column() {
    let (mut read, mut pool, strip) = a_strip_with_an_oblique_end(false);

    let chord_from = read.start.rail[1];
    let chord_to = read.start.rail[2];
    let carried = pool.add(Vector3::lerp(
        pool.position_of(chord_from),
        pool.position_of(chord_to),
        0.9,
    ));

    read.start.between[1] = vec![carried];

    let on_the_end = pool.add(Vector3::lerp(
        pool.position_of(read.end.rail[1]),
        pool.position_of(read.end.rail[2]),
        0.3,
    ));

    read.end.between[1] = vec![on_the_end];

    let mut faces = Vec::new();

    let stitched = surface_bevel_stitch::try_stitch(
        &mut pool,
        &mut faces,
        &[],
        &strip,
        &read.seams,
        0,
        &read.start,
        &read.end,
    );

    assert!(
        stitched.is_ok(),
        "the fixture's own strip has to stitch: {stitched:?}"
    );

    for face in &faces {
        assert!(
            twice_the_area(&pool, *face) > 1e-5,
            "({},{},{}) covers nothing",
            face.0,
            face.1,
            face.2
        );
    }

    assert_eq!(uses(&faces, chord_from, carried), 1);
    assert_eq!(
        uses(&faces, carried, chord_to),
        1,
        "the carried break broke the start ring's chord and still does"
    );

    assert_eq!(uses(&faces, read.end.rail[1], on_the_end), 1);
    assert_eq!(
        uses(&faces, on_the_end, read.end.rail[2]),
        1,
        "and the end ring's own break broke the end ring's chord"
    );

    assert_eq!(
        uses(&faces, chord_from, chord_to),
        0,
        "neither chord may be spanned: both ends of the column stop at their own break"
    );

    assert_eq!(uses(&faces, read.end.rail[1], read.end.rail[2]), 0);
}

/// A column with NO width anywhere - the two end rings standing on one pair of vertices - has no
/// column to carry its breaks to. That ring is still emitted, as needles whose edges are exactly
/// its own, because those edges are what close the shell across it.
#[test]
fn a_column_collapsed_all_the_way_through_is_still_closed_through() {
    let (mut read, mut pool, strip) = a_strip_with_an_oblique_end(true);

    let chord_from = read.start.rail[1];
    let chord_to = read.start.rail[2];
    let break_at = pool.add(Vector3::lerp(
        pool.position_of(chord_from),
        pool.position_of(chord_to),
        0.6,
    ));

    read.start.between[1] = vec![break_at];

    let mut faces = Vec::new();

    let stitched = surface_bevel_stitch::try_stitch(
        &mut pool,
        &mut faces,
        &[],
        &strip,
        &read.seams,
        0,
        &read.start,
        &read.end,
    );

    assert!(
        stitched.is_ok(),
        "the fixture's own strip has to stitch: {stitched:?}"
    );

    assert_eq!(
        uses(&faces, chord_from, break_at),
        1,
        "the break still has to be zipped in - there is no column with width to carry it to"
    );

    assert_eq!(uses(&faces, break_at, chord_to), 1);

    assert_eq!(
        uses(&faces, chord_from, chord_to),
        1,
        "the needle's one diagonal is the chord itself, which cancels against nothing else"
    );
}

/// The two rings and the seams of one crafted strip.
struct ObliqueEndReading {
    start: SurfaceStripStation,
    end: SurfaceStripStation,
    seams: Vec<SideSeams>,
}

/// The part's feature edges and its region cache at their flat angle.
fn regions_of(mesh: &Arc<Mesh>) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = FaceRegionCache::new(
        mesh.clone(),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );

    (Arc::new(regions), features)
}

/// One cube edge's fillet strip, with an OBLIQUE start ring - the two
/// middle rails reaching a whole station further along than the flanks - and a left seam whose one
/// interior vertex stands behind that ring on both of them. Stationed there, the two middle rails
/// are clamped and the column between them collapses; the flanking columns keep their width.
///
/// `collapse_column_one_end_to_end`: whether the middle column has no width at EITHER ring, which
/// leaves it nothing to carry to.
fn a_strip_with_an_oblique_end(
    collapse_column_one_end_to_end: bool,
) -> (ObliqueEndReading, SurfaceVertexPool, SurfaceStrip) {
    let cube = Arc::new(platonic_solids::create_cube_vector(Vector3::new(
        20.0, 20.0, 20.0,
    )));
    let (regions, features) = regions_of(&cube);

    let matching: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];

            (edge.start.y + 10.0).abs() < 1e-5
                && (edge.end.y + 10.0).abs() < 1e-5
                && edge.start.z > 9.9
                && edge.end.z > 9.9
        })
        .collect();

    assert_eq!(matching.len(), 1, "exactly one top edge at y = -10");

    let mut chains = bevel_chain_linker::link(&[features.edges()[matching[0]]]);

    assert_eq!(chains.len(), 1, "exactly one chain");

    let chain = chains.remove(0);

    let plan = SurfaceBevelPlan::try_create(&chain, &regions, 3.0, true)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason:?}"));

    let strip = SurfaceStrip::try_create(&plan.edges[0], 3)
        .unwrap_or_else(|reason| panic!("the strip refused to rule: {reason:?}"));

    let mut pool = SurfaceVertexPool::new(&cube.vertices);

    let rails = strip.rail_count() as usize;
    let mut start_param = vec![0.0; rails];
    let mut end_param = vec![0.0; rails];

    for rail in 0..rails {
        // The oblique cut: the two middle rails a station and a half ahead of the flanks.
        start_param[rail] = if rail == 1 || rail == 2 { 1.5 } else { 0.0 };
        end_param[rail] = if collapse_column_one_end_to_end && (rail == 1 || rail == 2) {
            1.5
        } else {
            12.0
        };
    }

    let start = ring_at(&mut pool, &strip, &start_param);
    let end = ring_at(&mut pool, &strip, &end_param);

    // One interior vertex on the left seam, a station BEHIND the cut on the middle rails.
    let interior = pool.add(strip.point_on(0, 1.0));
    let left = vec![
        (start.rail[0], start_param[0]),
        (interior, 1.0),
        (end.rail[0], end_param[0]),
    ];
    let right = vec![
        (start.rail[rails - 1], start_param[rails - 1]),
        (end.rail[rails - 1], end_param[rails - 1]),
    ];

    let seams = vec![[Some(left), Some(right)]];

    (ObliqueEndReading { start, end, seams }, pool, strip)
}

/// A station on every rail at its own parameter, with nothing between the rails yet.
fn ring_at(
    pool: &mut SurfaceVertexPool,
    strip: &SurfaceStrip,
    param: &[f64],
) -> SurfaceStripStation {
    let mut rail = Vec::with_capacity(param.len());

    for (i, &at) in param.iter().enumerate() {
        rail.push(pool.add(strip.point_on(i as i32, at)));
    }

    let between = vec![Vec::new(); param.len() - 1];

    SurfaceStripStation::new(rail, param.to_vec(), between)
}

fn twice_the_area(pool: &SurfaceVertexPool, face: (i32, i32, i32)) -> f64 {
    let at = pool.position_of(face.0);

    (pool.position_of(face.1) - at)
        .cross(pool.position_of(face.2) - at)
        .length()
}

fn corners(face: (i32, i32, i32)) -> [i32; 3] {
    [face.0, face.1, face.2]
}

/// How many of the triangles use the edge between two vertices, either way round.
fn uses(faces: &[(i32, i32, i32)], first: i32, second: i32) -> i32 {
    let mut uses = 0;

    for face in faces {
        for (from, to) in [(face.0, face.1), (face.1, face.2), (face.2, face.0)] {
            if (from == first && to == second) || (from == second && to == first) {
                uses += 1;
            }
        }
    }

    uses
}
