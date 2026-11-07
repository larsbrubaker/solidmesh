//! The strip and its ends, phase 3 of the polygon-space bevel plan, measured on the one shape whose
//! every number can be worked out on paper. The walkback left a gap bounded by the two tangent
//! lines and by a step round the cube's own corner at each end; what is checked here is that the
//! strip fills exactly that gap - the right surface, in the right place, sewn to the vertices
//! already standing there - and that the end faces gave up exactly the section the strip's end
//! profile cuts out of them.
//!
//! Everything is measured rather than counted wherever a count would pin a triangulation: area,
//! volume and distance are what the construction promises.

use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceLine,
};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector2, Vector3, Vector3Float};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::topology_test_meshes::{create_cylinder, create_l_shaped_prism};

/// Areas and volumes are summed over float vertices, so they are read to a ten thousandth. Every
/// expectation below is a closed form, so there is nowhere for a wrong answer to hide in that.
const MEASURED: f64 = 1e-4;

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// One edge of a cube, chamfered 3. The strip is the flat 45 degree band between the two tangent
/// lines - 20 long and 3 root 2 across - and it is the ONLY new surface: the two end faces lose the
/// 3 by 3 right triangle the profile's single chord cuts off their corner, and nothing else moves.
#[test]
fn a_chamfered_cube_edge_is_filled_by_one_flat_band_and_notches_both_end_faces() {
    let built = build(3.0, false, 1);

    assert!(
        built.is_manifold(),
        "the strip closed the gap the walkback opened"
    );
    assert_eq!(shell_count(&built), 1);
    assert_eq!(boundary_edge_count(&built), 0);

    // The section removed is the right triangle on the two setbacks, and the edge is prismatic.
    assert_within(
        signed_volume(&built),
        8000.0 - (20.0 * 4.5),
        MEASURED,
        "the volume",
    );

    let strip = strip_faces(&built);

    assert_within(
        area_of(&built, &strip),
        20.0 * 3.0 * 2f64.sqrt(),
        MEASURED,
        "a 20 long band 3 root 2 across is the whole of a 90 degree chamfer's surface",
    );

    // What the triangle count was standing in for, said directly: the strip's cross sections stand
    // at the union of the two tangent lines' own vertices, and the end profile carries whatever the
    // end face's triangulation left on it, so that no vertex anywhere ends up part way along
    // another triangle's edge.
    assert_nothing_stands_part_way_along_an_edge(&built);

    for &face in &strip {
        for corner in corners(&built, face) {
            assert!(
                (corner.z - corner.y - 17.0).abs() < 1e-5,
                "{corner:?} stands on the 45 degree chamfer plane through (0, -7, 10) and (0, -10, 7)"
            );
        }
    }

    for end in [10.0, -10.0] {
        assert_within(
            area_on(&built, |v| (v.x as f64 - end).abs() < 1e-6),
            400.0 - 4.5,
            MEASURED,
            &format!("the x = {end} face gave up the 3 by 3 triangle at its corner"),
        );
    }
}

/// The same edge filleted 3 with 8 segments. The strip is now eight flat bands, and what says they
/// are the right eight is that every vertex of them stands exactly a radius from the center line.
///
/// The volume is checked too, and it is NOT the analytic fillet's: the profile is sampled with
/// inscribed chords, so the cut takes the circular segment under each chord with it.
#[test]
fn a_filleted_cube_edge_stands_a_radius_from_its_center_line_all_the_way_round() {
    const RADIUS: f64 = 3.0;
    const SEGMENTS: i32 = 8;

    let built = build(RADIUS, true, SEGMENTS);

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
    assert_eq!(boundary_edge_count(&built), 0);

    // The center line of a 90 degree edge filleted 3: a radius in from each of the two faces.
    let center_line = SurfaceLine::new(Vector3::new(0.0, -7.0, 7.0), Vector3::new(1.0, 0.0, 0.0));

    let strip = strip_faces(&built);

    for &face in &strip {
        for corner in corners(&built, face) {
            assert!(
                (center_line.distance_to(corner) - RADIUS).abs() < 1e-5,
                "{corner:?} is on the rolling ball"
            );
        }
    }

    // Eight chords of a quarter turn, each 2 r sin(sweep / 16) long, carried 20 mm.
    let chord = 2.0 * RADIUS * (PI / 2.0 / SEGMENTS as f64 / 2.0).sin();

    assert_within(
        area_of(&built, &strip),
        20.0 * SEGMENTS as f64 * chord,
        MEASURED,
        "the strip",
    );

    let section = wedge_area(RADIUS, PI / 2.0) + chord_excess_area(RADIUS, PI / 2.0, SEGMENTS);

    assert_within(
        signed_volume(&built),
        8000.0 - (20.0 * section),
        MEASURED,
        "the volume",
    );

    for end in [10.0, -10.0] {
        assert_within(
            area_on(&built, |v| (v.x as f64 - end).abs() < 1e-6),
            400.0 - section,
            MEASURED,
            &format!(
                "the x = {end} face gave up the same section the strip carries along the edge"
            ),
        );
    }
}

/// Rung 2's shape at the kernel: the whole top rim of a cube chamfered 3, four edges closing on
/// themselves, mitred at each corner. By inclusion and exclusion `4 * 20 * 4.5 - 4 * 9 = 324`
/// comes off the 8000.
#[test]
fn the_top_rim_of_a_cube_mitres_its_four_corners_into_the_analytic_solid() {
    let cube = create_cube(20.0, 20.0, 20.0);
    let built = build_the_top_rim(&cube, 3.0, false, 1);

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
    assert_eq!(boundary_edge_count(&built), 0);

    assert_within(
        signed_volume(&built),
        8000.0 - 324.0,
        MEASURED,
        "the volume",
    );

    // Each mitre is a real corner of the surface, where the two strips meet at the four points the
    // tangent lines cross - and it is ONE point, shared, not two a weld would have to find.
    for mitre in [
        Vector3::new(7.0, 7.0, 10.0),
        Vector3::new(-7.0, 7.0, 10.0),
        Vector3::new(7.0, -7.0, 10.0),
        Vector3::new(-7.0, -7.0, 10.0),
    ] {
        let at = built
            .vertices
            .iter()
            .filter(|&&v| (Vector3::from_vector3_float(v) - mitre).length() < 1e-5)
            .count();
        assert_eq!(at, 1, "the mitre at {mitre:?} is one vertex of the mesh");
    }
}

/// The same rim FILLETED, with one vertex of the front wall standing inside a mitre's setback
/// window: on the wall's own tangent line, 1.5 mm short of the corner, where the setback is 3. A
/// station the wall's seam asks for there is already PAST the end of the rails nearer the top, and
/// a cross section taken at it steps backward on them while it steps forward on the rest - which
/// came back as a doubled-over flap. So it is measured by AREA and by that pair rather than by
/// manifoldness, which the fold satisfies.
#[test]
fn a_vertex_inside_a_mitres_setback_window_does_not_double_the_strip_back_over_itself() {
    const RADIUS: f64 = 3.0;
    const SEGMENTS: i32 = 8;

    let cube = with_an_extra_vertex_in_a_face(
        &create_cube(20.0, 20.0, 20.0),
        Vector3::new(8.5, -10.0, 7.0),
    );
    let built = build_the_top_rim(&cube, RADIUS, true, SEGMENTS);

    assert!(built.is_manifold());
    assert_eq!(boundary_edge_count(&built), 0);
    assert_eq!(
        doubled_face_count(&built),
        0,
        "two triangles on the same three vertices are one surface folded back over itself"
    );

    // The rails: the arc about the centre line at (y, z) = (-7, 7), swung from the foot on the top
    // face to the foot on the wall, and each one reaching -2 y along the edge between its two
    // mitres.
    let mut expected = 0.0;

    for chord in 0..SEGMENTS {
        let here = -7.0 - (RADIUS * (PI / 2.0 * chord as f64 / SEGMENTS as f64).sin());
        let there = -7.0 - (RADIUS * (PI / 2.0 * (chord + 1) as f64 / SEGMENTS as f64).sin());

        expected += ((-2.0 * here) + (-2.0 * there)) / 2.0
            * (2.0 * RADIUS * (PI / 2.0 / SEGMENTS as f64 / 2.0).sin());
    }

    assert_within(
        area_of(&built, &strip_faces(&built)),
        4.0 * expected,
        MEASURED,
        "the four bevelled edges carry the same chorded profile between their own mitre planes, and no more",
    );
}

/// What a whole cylinder rim comes to TODAY: 64 facet edges and 64 wall facets, each planned
/// against its own flat wall, mitred facet to facet. The faceted answer IS the analytic one for the
/// solid it builds - Pappus on the inradius, because every mitre plane of a regular rim passes
/// through the axis - and the tolerance is left at 2 per cent because what this test is FOR is
/// noticing when rung 5 changes the answer.
#[test]
fn a_cylinder_rim_builds_facet_to_facet_today() {
    const RADIUS: f64 = 10.0;
    const HEIGHT: f64 = 10.0;
    const SIDES: i32 = 64;
    const FILLET: f64 = 3.0;
    const SEGMENTS: i32 = 8;

    let cylinder = create_cylinder(RADIUS, HEIGHT, SIDES);
    let (regions, features) = regions_of(&cylinder);
    let top = features
        .edges()
        .iter()
        .map(|edge| edge.start.z)
        .fold(f64::MIN, f64::max);

    let rim: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-6 && (edge.end.z - top).abs() < 1e-6)
        .cloned()
        .collect();

    assert_eq!(rim.len(), SIDES as usize);

    let built = build_chain(&cylinder, &regions, &rim, FILLET, true, SEGMENTS);

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
    assert_eq!(boundary_edge_count(&built), 0);

    let removed = signed_volume(&cylinder) - signed_volume(&built);
    let analytic = rim_swept_volume(&the_removed_section(FILLET, SEGMENTS), RADIUS, SIDES);

    assert_within(
        removed,
        analytic,
        analytic * 0.02,
        "the mitred facet-to-facet sweep of the section round the rim is the solid this builds",
    );
}

/// The same one cube edge chamfered at a tenth of the cube, on a 20 mm cube, a 200 mm one and a
/// 2000 mm one. Nothing about the construction is scale dependent, but the arithmetic underneath
/// is: a mesh stores floats, and any tolerance written as a fixed micron stops meaning anything
/// there. The setbacks are deliberately not round numbers, so the tangent lines land where a float
/// cannot hold them exactly.
#[test]
fn a_chamfered_cube_edge_closes_at_every_scale() {
    for (across, setback) in [
        (20.0, 3.0),
        (200.0, 30.0),
        (2000.0, 300.0),
        (20.0, 4.758),
        (200.0, 47.58),
        (2000.0, 475.8),
    ] {
        let built = build_on(across, setback, false, 1);

        assert_eq!(
            boundary_edge_count(&built),
            0,
            "a {across} mm cube's chamfer has to close as surely as a 20 mm one's"
        );
        assert!(built.is_manifold());
        assert_eq!(shell_count(&built), 1);

        assert_within(
            signed_volume(&built),
            (across * across * across) - (across * setback * setback / 2.0),
            MEASURED * across * across * across / 8000.0,
            "the chamfer takes the right triangle on the two setbacks, carried the whole edge",
        );
    }
}

/// The concave inner edge of an L prism, filleted. The bead is ADDED rather than cut, and its two
/// ends stand on the top and bottom caps - where the corner is a REENTRANT vertex of the face, so
/// the section between the profile and the corner is the bead's own cap.
#[test]
fn the_concave_edge_of_an_l_prism_adds_its_bead_and_caps_both_ends() {
    const RADIUS: f64 = 3.0;
    const SEGMENTS: i32 = 8;

    let prism = create_l_shaped_prism();
    let (regions, features) = regions_of(&prism);

    let inner: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .cloned()
        .collect();
    assert_eq!(inner.len(), 1);

    let built = build_chain(&prism, &regions, &inner, RADIUS, true, SEGMENTS);

    assert_eq!(
        boundary_edge_count(&built),
        0,
        "a bead on a closed solid leaves a closed solid, caps and all"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    let section = wedge_area(RADIUS, PI / 2.0) + chord_excess_area(RADIUS, PI / 2.0, SEGMENTS);

    assert_within(
        signed_volume(&built),
        3000.0 + (10.0 * section),
        MEASURED,
        "the inner corner gains exactly the material a ball of that radius cannot roll into",
    );
}

/// The section a 90 degree fillet takes out of a rim, as the polygon between the corner and the
/// chorded arc: `x` in from the rim chord, `y` down from the top face.
fn the_removed_section(radius: f64, segments: i32) -> Vec<Vector2> {
    let mut found = vec![Vector2::new(0.0, 0.0), Vector2::new(radius, 0.0)];

    for i in 1..=segments {
        let turn = PI / 2.0 * i as f64 / segments as f64;

        found.push(Vector2::new(
            radius - (radius * turn.sin()),
            radius - (radius * turn.cos()),
        ));
    }

    found
}

/// The volume a section mitred round the rim of a regular prism removes: Pappus on the inradius.
fn rim_swept_volume(section: &[Vector2], circumradius: f64, sides: i32) -> f64 {
    let mut twice_area = 0.0;
    let mut six_times_moment = 0.0;

    for i in 0..section.len() {
        let here = section[i];
        let next = section[(i + 1) % section.len()];
        let cross = (here.x * next.y) - (next.x * here.y);

        twice_area += cross;
        six_times_moment += cross * (here.x + next.x);
    }

    let area = f64::abs(twice_area) / 2.0;
    let moment = f64::abs(six_times_moment) / 6.0;

    let perimeter = sides as f64 * 2.0 * circumradius * (PI / sides as f64).sin();
    let inradius = circumradius * (PI / sides as f64).cos();

    perimeter / inradius * ((inradius * area) - moment)
}

/// The same cube with one more vertex in it: the triangle `at` falls inside is fanned into three
/// about it, which adds a vertex and changes no geometry.
fn with_an_extra_vertex_in_a_face(cube: &Mesh, at: Vector3) -> Mesh {
    let mut split = Mesh::new();
    let mut vertices = cube.vertices.clone();
    let added = vertices.len() as i32;
    vertices.push(Vector3Float::from_vector3(at));

    let mut found = false;

    for face in 0..cube.faces.len() {
        let corners = corners(cube, face);
        let mesh = &cube.faces[face];

        if !found
            && (at - corners[0])
                .dot(
                    (corners[1] - corners[0])
                        .cross(corners[2] - corners[0])
                        .get_normal(),
                )
                .abs()
                < 1e-9
            && holds(&corners, at)
        {
            found = true;

            split.faces.add(mesh.v0, mesh.v1, added, &vertices);
            split.faces.add(mesh.v1, mesh.v2, added, &vertices);
            split.faces.add(mesh.v2, mesh.v0, added, &vertices);
            continue;
        }

        split.faces.add(mesh.v0, mesh.v1, mesh.v2, &vertices);
    }

    assert!(found, "no face of the fixture holds {at:?}");

    split.vertices = vertices;
    split
}

/// Whether a point stands strictly inside a triangle, read in the plane the three lie in.
fn holds(corners: &[Vector3; 3], at: Vector3) -> bool {
    let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);

    (0..3).all(|i| {
        let from = corners[i];
        let along = corners[(i + 1) % 3] - from;

        (at - from).cross(along).dot(normal) < 0.0
    })
}

/// How many triangles of the mesh stand on the same three vertices as another one - the reading
/// that catches a surface folded back over itself, which manifoldness and shell count both accept.
fn doubled_face_count(mesh: &Mesh) -> i32 {
    let mut seen: HashMap<[i32; 3], i32> = HashMap::new();

    for face in mesh.faces.iter() {
        let mut corners = [face.v0, face.v1, face.v2];
        corners.sort();

        *seen.entry(corners).or_insert(0) += 1;
    }

    seen.values().filter(|&&count| count > 1).sum()
}

/// The one cube edge every single-edge case is built on: the top front edge of a 20 cube.
fn build(size: f64, round_profile: bool, segments_per_quarter: i32) -> Mesh {
    build_on(20.0, size, round_profile, segments_per_quarter)
}

/// The same, on a cube of any size - the top front edge of an `across` cube.
fn build_on(across: f64, size: f64, round_profile: bool, segments_per_quarter: i32) -> Mesh {
    let cube = create_cube(across, across, across);
    let (regions, features) = regions_of(&cube);

    let half = across / 2.0;

    let edge: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.y + half).abs() < 1e-6 * across
                && (edge.end.y + half).abs() < 1e-6 * across
                && edge.start.z > half * 0.99
                && edge.end.z > half * 0.99
        })
        .cloned()
        .collect();
    assert_eq!(edge.len(), 1);

    build_chain(
        &cube,
        &regions,
        &edge,
        size,
        round_profile,
        segments_per_quarter,
    )
}

/// The whole top rim of `cube`, linked into its one closed chain and built.
fn build_the_top_rim(cube: &Mesh, size: f64, round_profile: bool, segments: i32) -> Mesh {
    let (regions, features) = regions_of(cube);

    let rim: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| edge.start.z > 9.9 && edge.end.z > 9.9)
        .cloned()
        .collect();

    build_chain(cube, &regions, &rim, size, round_profile, segments)
}

/// Links `edges` into their one chain, plans it and builds it, failing with the plan's or the
/// build's reason.
fn build_chain(
    part: &Mesh,
    regions: &Arc<FaceRegionCache>,
    edges: &[geometry_processing::topology::FeatureEdge],
    size: f64,
    round_profile: bool,
    segments: i32,
) -> Mesh {
    let chains = bevel_chain_linker::link(edges);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create(&chains[0], regions, size, round_profile)
        .unwrap_or_else(|reason| panic!("the plan refused the edge: {reason}"));

    surface_bevel_builder::try_build_one(
        part,
        &part.get_mesh_edge_graph(),
        regions,
        plan,
        segments,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the edge: {reason:?}"))
}

/// The kite between the edge and its two tangent points less the sector of the rolling ball.
pub fn wedge_area(radius: f64, wedge_radians: f64) -> f64 {
    (radius * radius / (wedge_radians / 2.0).tan()) - (radius * radius * (PI - wedge_radians) / 2.0)
}

/// How much more an inscribed chording takes than the arc it stands in for.
pub fn chord_excess_area(radius: f64, sweep_radians: f64, segments: i32) -> f64 {
    radius * radius / 2.0
        * (sweep_radians - (segments as f64 * (sweep_radians / segments as f64).sin()))
}

/// The faces of the strip: the ones standing on none of the cube's own six planes.
fn strip_faces(mesh: &Mesh) -> Vec<usize> {
    (0..mesh.faces.len())
        .filter(|&face| {
            let corners = corners(mesh, face);
            let on_a_cube_plane = |read: fn(&Vector3) -> f64| {
                corners
                    .iter()
                    .all(|corner| (read(corner).abs() - 10.0).abs() < 1e-5)
            };

            !on_a_cube_plane(|c| c.x) && !on_a_cube_plane(|c| c.y) && !on_a_cube_plane(|c| c.z)
        })
        .collect()
}

/// How many edges of the mesh are used by some number of faces other than two - the construction's
/// own reading of whether its answer closes.
fn boundary_edge_count(mesh: &Mesh) -> i32 {
    surface_mesh_closure::open_edge_count(mesh).0
}

/// That no vertex of the mesh stands part way along an edge of a face that does not use it - the T
/// junction the whole sewing scheme exists to avoid.
fn assert_nothing_stands_part_way_along_an_edge(mesh: &Mesh) {
    for face in mesh.faces.iter() {
        for (low, high) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let from = Vector3::from_vector3_float(mesh.vertices[low as usize]);
            let run = Vector3::from_vector3_float(mesh.vertices[high as usize]) - from;

            for vertex in 0..mesh.vertices.len() as i32 {
                if vertex == low || vertex == high {
                    continue;
                }

                let at = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);
                let where_ = (at - from).dot(run) / run.length_squared();

                let standing_on_it = where_ > 1e-6
                    && where_ < 1.0 - 1e-6
                    && (from + (run * where_) - at).length() <= 1e-5;

                assert!(
                    !standing_on_it,
                    "{at:?} stands part way along the edge from {from:?} to {:?}, which no face using that edge knows about",
                    from + run
                );
            }
        }
    }
}

fn area_of(mesh: &Mesh, faces: &[usize]) -> f64 {
    let mut area = 0.0;

    for &face in faces {
        let corners = corners(mesh, face);

        area += (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .length()
            / 2.0;
    }

    area
}

fn area_on(mesh: &Mesh, on_the_plane: impl Fn(Vector3Float) -> bool) -> f64 {
    let faces: Vec<usize> = (0..mesh.faces.len())
        .filter(|&face| {
            let f = &mesh.faces[face];
            on_the_plane(mesh.vertices[f.v0 as usize])
                && on_the_plane(mesh.vertices[f.v1 as usize])
                && on_the_plane(mesh.vertices[f.v2 as usize])
        })
        .collect();

    area_of(mesh, &faces)
}

fn corners(mesh: &Mesh, face_index: usize) -> [Vector3; 3] {
    let face = &mesh.faces[face_index];

    [
        Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]),
        Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]),
        Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]),
    ]
}

/// The part's feature edges and its region cache at their flat angle.
pub fn regions_of(mesh: &Mesh) -> (Arc<FaceRegionCache>, FeatureEdgeSet) {
    let features = FeatureEdgeSet::create(mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    (regions, features)
}
