//! Rung 4 of the polygon-space bevel plan - ONE rim facet edge
//! of a 64 sided cylinder, where the chain stops in the middle of a run that goes on without it.
//! Neither end has a face to notch - the cap's neighbours are the cap, and the wall's neighbour is
//! another wall facet five degrees round - so the bevel has to run out against a wall it makes
//! itself, square to the edge. The section's closed forms are `surface_strip_tests`' (rung 1's).

use std::f64::consts::PI;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceLine,
};
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_strip_tests::{chord_excess_area, wedge_area};
use crate::surface_walkback_checks::{area_of_where, area_on, corners};
use crate::topology_test_meshes::create_cylinder;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 10.0;

const SIDES: i32 = 64;

/// Areas and volumes are summed over float vertices, so they are read to a ten thousandth. Every
/// expectation below is a closed form, so there is nowhere for a wrong answer to hide in that.
const MEASURED: f64 = 1e-4;

/// One rim facet filleted 2. The section is rung 1's - the rim's dihedral is a right angle, the cap
/// being flat and the facet vertical - and the run is prismatic over the facet's own chord, because
/// both ends are square to the edge. So the volume removed is that section times the chord, exactly,
/// and the two walls that close the groove are one section each.
///
/// The cap is what this rung is really about. Walked back to an unbounded half space, a straight
/// tangent line takes a whole circular SEGMENT off a disc - everything on its far side, which on a
/// 10 mm cap set back 2 mm is a third of the rim - and no strip goes anywhere near that. Bounded by
/// the planes the chain stops at, it takes the 2 by 0.98 rectangle over the edge and nothing else,
/// which is what the cap's area says here.
///
/// The closure reading is also the T junction reading: the end planes split facets down the middle,
/// so the tangent line ends part way along an edge two of the cap's own pieces share, and a piece
/// that did not know about it would leave those pieces' edges used once each rather than twice.
#[test]
fn a_rim_facet_fillet_runs_out_square_at_both_ends() {
    const SIZE: f64 = 2.0;
    const SEGMENTS: i32 = 8;

    let built = build(SIZE, true, SEGMENTS);
    let section = wedge_area(SIZE, PI / 2.0) + chord_excess_area(SIZE, PI / 2.0, SEGMENTS);

    assert_it_closed(&built);

    assert_within(
        signed_volume(&built),
        cylinder_volume() - (section * chord_length()),
        MEASURED,
        "the section is analytic, tessellation term and all, and a square ended run is a prism of it",
    );

    assert_the_cap_kept_everything_but_the_band(&built, SIZE);
    assert_the_run_out_walls_are_one_section_each(&built, section);

    // The whole of what a fillet is: every vertex of the blend stands a radius from the line the
    // rolling ball's centre runs along - a radius in from the facet's own plane and a radius down
    // from the cap, measured on the facet's apothem rather than on the cylinder's nominal radius.
    let centre = centre_line(SIZE);

    for face in blend_faces(&built) {
        for corner in corners(&built, face) {
            assert_within(
                centre.distance_to(corner),
                SIZE,
                1e-5,
                &format!(
                    "{corner:?} is on the blend, so it stands a radius from {:?}",
                    centre.point
                ),
            );
        }
    }

    let blend = blend_faces(&built);
    assert_within(
        area_of_where(&built, |face| blend.contains(&face)),
        chord_length()
            * 2.0
            * SIZE
            * SEGMENTS as f64
            * (PI / 2.0 / SEGMENTS as f64 / 2.0).sin(),
        MEASURED,
        "the blend is the chorded quarter arc carried the length of the facet's chord and no further",
    );
}

/// The same rim facet chamfered 2, where the profile is one chord and the run-out walls are the
/// right triangle on the two setbacks. Nothing about the ends is different - which is the point of
/// running the rung in both profiles.
#[test]
fn a_rim_facet_chamfer_runs_out_square_at_both_ends() {
    const SIZE: f64 = 2.0;

    let built = build(SIZE, false, 1);
    let section = SIZE * SIZE / 2.0;

    assert_it_closed(&built);

    assert_within(
        signed_volume(&built),
        cylinder_volume() - (section * chord_length()),
        MEASURED,
        "",
    );

    assert_the_cap_kept_everything_but_the_band(&built, SIZE);
    assert_the_run_out_walls_are_one_section_each(&built, section);

    let blend = blend_faces(&built);
    assert_within(
        area_of_where(&built, |face| blend.contains(&face)),
        chord_length() * SIZE * 2.0_f64.sqrt(),
        MEASURED,
        "a 90 degree chamfer's band is root two setbacks across, over the facet's chord",
    );
}

fn assert_it_closed(built: &Mesh) {
    assert!(
        built.is_manifold(),
        "the strip and its two end walls closed the gap the walkback opened"
    );
    assert_eq!(shell_count(built), 1);
    let (open, open_at) = surface_mesh_closure::open_edge_count(built);
    assert_eq!(
        open, 0,
        "every edge of the answer is shared by two faces, and the first that is not is at {open_at:?}"
    );
}

/// That the cap gave up the rectangle over the edge and nothing else: its area is the polygon's
/// less the setback times the facet's chord.
fn assert_the_cap_kept_everything_but_the_band(built: &Mesh, setback: f64) {
    let top = HEIGHT / 2.0;

    assert_within(
        area_on(built, |vertex| (f64::from(vertex.z) - top).abs() < 1e-5),
        polygon_area() - (setback * chord_length()),
        MEASURED,
        "an unbounded tangent plane would take a circular segment off the cap instead",
    );
}

/// That each end of the groove is closed by one wall standing square to the edge, of exactly the
/// section's area. Read ON the plane of the corner it stands at rather than merely near it: the
/// facet's chord is only a millimetre long, so the two walls are neighbours, and a facet a quarter
/// of the way round the cylinder faces the same way as both of them.
fn assert_the_run_out_walls_are_one_section_each(built: &Mesh, section: f64) {
    let (from, to) = the_rim_facet();
    let along = (to - from).get_normal();

    for corner in [from, to] {
        let area = area_of_where(built, |face| {
            normal_of(built, face).dot(along).abs() > 1.0 - 1e-6
                && corners(built, face)
                    .iter()
                    .all(|at| (*at - corner).dot(along).abs() < 1e-5)
        });

        assert_within(
            area,
            section,
            MEASURED,
            &format!("the groove's wall at {corner:?} is the section itself, standing square to the edge"),
        );
    }
}

/// The faces of the blend: the ones facing both the cap and the facet at once. Everything the
/// walkback left behind faces one of them and not the other, and the two run-out walls face
/// neither.
fn blend_faces(built: &Mesh) -> Vec<i32> {
    let outward = facet_normal();

    (0..built.faces.len() as i32)
        .filter(|&face| {
            let normal = normal_of(built, face);

            normal.dot(Vector3::UNIT_Z) > 1e-6 && normal.dot(outward) > 1e-6
        })
        .collect()
}

/// The line the rolling ball's centre runs along for the picked facet: a radius in from the
/// facet's plane, which stands at the polygon's apothem, and a radius down from the cap.
fn centre_line(size: f64) -> SurfaceLine {
    let (from, to) = the_rim_facet();
    let outward = facet_normal();

    SurfaceLine::new(
        (outward * ((RADIUS * (PI / SIDES as f64).cos()) - size))
            + Vector3::new(0.0, 0.0, (HEIGHT / 2.0) - size),
        (to - from).get_normal(),
    )
}

fn facet_normal() -> Vector3 {
    let (from, to) = the_rim_facet();
    let midpoint = (from + to) / 2.0;

    Vector3::new(midpoint.x, midpoint.y, 0.0).get_normal()
}

/// The two ends of the rim edge the tests pick, read the same way `build` does.
fn the_rim_facet() -> (Vector3, Vector3) {
    let cylinder = create_cylinder(RADIUS, HEIGHT, SIDES);
    let features = FeatureEdgeSet::create(&cylinder);
    let edge = features.edges()[rim_edge_index(&features)];

    (edge.start, edge.end)
}

fn rim_edge_index(features: &FeatureEdgeSet) -> usize {
    let top = HEIGHT / 2.0;

    (0..features.count())
        .find(|&i| {
            (features.edges()[i].start.z - top).abs() < 1e-6
                && (features.edges()[i].end.z - top).abs() < 1e-6
        })
        .expect("a rim edge")
}

/// The cylinder's own volume, as the prism on its 64 sided polygon.
fn cylinder_volume() -> f64 {
    polygon_area() * HEIGHT
}

fn polygon_area() -> f64 {
    SIDES as f64 / 2.0 * RADIUS * RADIUS * (2.0 * PI / SIDES as f64).sin()
}

/// How long one facet's rim edge is: the chord of a 64th of the circle.
fn chord_length() -> f64 {
    2.0 * RADIUS * (PI / SIDES as f64).sin()
}

fn build(size: f64, round_profile: bool, segments_per_quarter: i32) -> Mesh {
    let cylinder = Arc::new(create_cylinder(RADIUS, HEIGHT, SIDES));
    let features = FeatureEdgeSet::create(&cylinder);
    let graph = cylinder.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        cylinder.clone(),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        cylinder.clone(),
        Some(graph.clone()),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ));

    let mut chains = bevel_chain_linker::link(&[features.edges()[rim_edge_index(&features)]]);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create_with_curve_regions(
        &chains.remove(0),
        &regions,
        Some(&curve_regions),
        size,
        round_profile,
    )
    .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason}"));

    surface_bevel_builder::try_build_one(
        &cylinder,
        &graph,
        &regions,
        plan,
        segments_per_quarter,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the edge: {reason:?}"))
}

fn normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let corners = corners(mesh, face);

    (corners[1] - corners[0])
        .cross(corners[2] - corners[0])
        .get_normal()
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}
