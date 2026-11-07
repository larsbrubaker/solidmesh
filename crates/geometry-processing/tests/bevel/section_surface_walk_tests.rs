//! The surface walk's ROOF tests: `section_surface_walk` - a straight line
//! laid ON a faceted surface - read against a closed form. Two 10 x 10 panels hinged along the y
//! axis with 7.5 degrees between their normals (the tapered barrel's own fold angle,
//! `section_surface_walk_roof`); unfolded, the pair is one flat plane, so a walk of distance d from
//! a point a back from the hinge in a direction making theta with the up-slope ends at hinge
//! distance `d cos(theta) - a` and hinge offset `y + d sin(theta)`, compared at 1e-12. The barrel's
//! bore mouths and the pyramid's corner are the class's other half, in
//! `section_surface_walk_mouth_tests`.

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{math_helper, Vector3};

use geometry_processing::bevel::sections::section_fan_walk;
use geometry_processing::bevel::sections::section_surface_walk::{
    grid_of, SurfaceWalkCrossing, SurfaceWalkResult,
};
use geometry_processing::bevel::sections::section_surface_walk_faces::try_face_holding;
use geometry_processing::bevel::sections::section_surface_walk_join::try_join;
use geometry_processing::bevel::sections::section_surface_walk_try::try_walk;
use geometry_processing::bevel::sections::SurfaceWalkRefusal;
use geometry_processing::bevel::surface::SurfaceVertexPool;

use crate::section_surface_walk_roof::{
    a_roof, a_roof_at, corners, face_holding, frame, is_the_hinge, normal_of, off_the_face,
    triangle,
};

/// THE CLOSED FORM: unfolding the roof's one hinge is exactly flattening it, at 1e-12, for walks
/// that cross the hinge at a range of angles.
fn walking_over_the_roofs_fold_is_the_flattened_straight_line(toward_degrees: f64, distance: f64) {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let theta = math_helper::degrees_to_radians(toward_degrees);
    let back = 2.0;
    let offset = 0.5;

    let start = frame.hinge(offset) + (frame.down_a * back);
    let direction = (-frame.down_a * theta.cos()) + (frame.along * theta.sin());
    let start_face = face_holding(&roof, start);

    let walk = try_walk(
        &roof,
        &graph,
        &edge_at_pair,
        start,
        start_face,
        direction,
        distance,
        None,
    )
    .unwrap_or_else(|walk| panic!("{:?}", walk.refusal));
    assert!((walk.travelled - distance).abs() <= 1e-12);

    // Flattened: the hinge is the origin of a plain (across, along) plane.
    let across = -back + (distance * theta.cos());
    let along = offset + (distance * theta.sin());
    let wanted = if across <= 0.0 {
        frame.hinge(along) + (frame.down_a * -across)
    } else {
        frame.hinge(along) + (frame.down_b * across)
    };

    assert!((walk.end - wanted).length() < 1e-12 * wanted.length().max(1.0));

    assert_eq!(
        walk.crossings.len(),
        if across > 0.0 { 1 } else { 0 },
        "the hinge is crossed exactly when the reach passes it"
    );

    if walk.crossings.len() == 1 {
        let crossing = &walk.crossings[0];
        let hinge = offset + (back * theta.tan());

        assert!((crossing.at - frame.hinge(hinge)).length() < 1e-12 * 10.0);
        assert!(crossing.parameter >= 0.0);
        assert!(crossing.parameter <= 1.0);
        assert!(on_its_edge(&roof, &graph, crossing) < 1e-12 * 10.0);
    }

    assert!(off_the_plane_of(&roof, walk.end_face, walk.end) < 1e-12 * 10.0);
    assert!(inside(&roof, walk.end_face, walk.end));
    assert!((piece_lengths(start, &walk).iter().sum::<f64>() - distance).abs() <= 1e-12);
}

#[test]
fn walking_over_the_roofs_fold_is_the_flattened_straight_line_0_3() {
    walking_over_the_roofs_fold_is_the_flattened_straight_line(0.0, 3.0);
}

#[test]
fn walking_over_the_roofs_fold_is_the_flattened_straight_line_20_3() {
    walking_over_the_roofs_fold_is_the_flattened_straight_line(20.0, 3.0);
}

#[test]
fn walking_over_the_roofs_fold_is_the_flattened_straight_line_minus_35_4_5() {
    walking_over_the_roofs_fold_is_the_flattened_straight_line(-35.0, 4.5);
}

#[test]
fn walking_over_the_roofs_fold_is_the_flattened_straight_line_60_5() {
    walking_over_the_roofs_fold_is_the_flattened_straight_line(60.0, 5.0);
}

#[test]
fn walking_over_the_roofs_fold_is_the_flattened_straight_line_minus_55_5() {
    walking_over_the_roofs_fold_is_the_flattened_straight_line(-55.0, 5.0);
}

/// A REFUSAL IS NAMED AND CARRIES HOW FAR IT GOT: a walk told the hinge is a crease stops on the
/// hinge, having gone exactly the flattened distance to it.
#[test]
fn a_walk_stopped_at_the_fold_says_so_and_how_far_it_got() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let start = frame.hinge(0.5) + (frame.down_a * 2.0);
    let start_face = face_holding(&roof, start);
    let the_hinge = |edge: i32| is_the_hinge(&roof, &graph, edge);

    let walk = try_walk(
        &roof,
        &graph,
        &edge_at_pair,
        start,
        start_face,
        -frame.down_a,
        5.0,
        Some(&the_hinge),
    )
    .expect_err("the hinge is a crease here");

    assert_eq!(walk.refusal, SurfaceWalkRefusal::CreaseReached);
    assert!((walk.travelled - 2.0).abs() <= 1e-12);

    // And walked off the roof's far edge instead, it is the boundary that stops it.
    let over = try_walk(
        &roof,
        &graph,
        &edge_at_pair,
        start,
        start_face,
        frame.down_a,
        20.0,
        None,
    )
    .expect_err("the roof ends ten down its slope");

    assert_eq!(over.refusal, SurfaceWalkRefusal::BoundaryReached);
    assert!((over.travelled - (frame.length - 2.0)).abs() <= 1e-12);
}

/// A REACH THAT RUNS OUT WITHIN THE MESH'S OWN LATTICE OF A FOLD LANDS ON IT. Where the end falls -
/// a hair short of the fold, or a hair past it - is float noise; what is not good is a mesh vertex
/// a lattice from another, which is what a join into a near-miss end mints. So this is a
/// CLASSIFICATION, on the mesh's own ruler (`SurfaceVertexPool::weld_distance_for`, a millionth of
/// the part's diagonal): inside it the end IS the crossing, outside it the end lies strictly inside
/// the face and no edge is claimed.
#[test]
fn a_reach_running_out_within_the_lattice_of_a_fold_lands_on_it() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let offset = 0.5;
    let back = 2.0;
    let start = frame.hinge(offset) + (frame.down_a * back);
    let start_face = face_holding(&roof, start);
    let lattice = SurfaceVertexPool::weld_distance_for(&roof.vertices);

    assert!(
        lattice > 1e-7 && lattice < 1e-3,
        "the roof's lattice has to straddle the two reaches below for them to say anything"
    );

    let on_it = walk_or_say(&roof, &graph, start, start_face, -frame.down_a, back - 1e-7);

    assert!(on_it.end_edge >= 0);
    assert!(is_the_hinge(&roof, &graph, on_it.end_edge));
    assert!(
        (on_it.end - frame.hinge(offset)).length() < 1e-12 * 10.0,
        "landing on the fold puts the end exactly on it, not a lattice off it"
    );
    assert_eq!(on_it.end_face, start_face, "the face the walk came from");
    assert!(
        (on_it.travelled - back).abs() <= 1e-12,
        "the true distance walked, the shortfall being the classification's own rounding"
    );

    // A reach that runs out a thousandth short of the fold is short of it, and says so.
    let inside = walk_or_say(&roof, &graph, start, start_face, -frame.down_a, back - 1e-3);

    assert_eq!(inside.end_edge, -1);
    assert!((inside.travelled - (back - 1e-3)).abs() <= 1e-12);
}

/// THE LATTICE IS THE MESH'S OWN FLOAT GRID, WHICH GROWS WITH WHERE THE PART STANDS. A vertex is
/// stored single precision, so the same roof a thousand from the origin repeats a computed point
/// only to about 6e-5 where at the origin it repeats to 6e-7. Read on the mesh's grid, a reach 1e-4
/// short of the fold is inside the grid far from the origin and outside it at the origin.
#[test]
fn the_lattice_is_the_meshs_float_grid() {
    let offset = 0.5;
    let back = 2.0;
    let shortfall = 1e-4;

    let at_the_origin = a_roof();
    let far_out = a_roof_at(Vector3::new(0.0, 0.0, 1000.0));

    assert!(
        grid_of(&at_the_origin) < shortfall,
        "at the origin the roof's grid is its diagonal's millionth, about 2e-5"
    );
    assert!(
        grid_of(&far_out) > shortfall,
        "a thousand out, one float ulp is 6e-5 and the grid is a few of them"
    );

    for (name, roof, lands) in [
        ("at the origin", &at_the_origin, false),
        ("a thousand out", &far_out, true),
    ] {
        let frame = frame(roof);
        let graph = roof.get_mesh_edge_graph();
        let start = frame.hinge(offset) + (frame.down_a * back);

        let walk = walk_or_say(
            roof,
            &graph,
            start,
            face_holding(roof, start),
            -frame.down_a,
            back - shortfall,
        );

        assert_eq!(
            walk.end_edge >= 0,
            lands,
            "{name}: a reach {shortfall} short of the fold against a grid of {}",
            grid_of(roof)
        );
    }
}

/// THE LANDING READS BOTH SIDES OF THE FOLD. An end that ran a few ulps PAST a fold is on it as
/// much as one that stopped a few ulps short, so a reach that overshoots the hinge by less than the
/// mesh's grid stops ON the hinge, its end being the crossing itself.
#[test]
fn a_reach_running_out_just_past_a_fold_lands_on_it_too() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let offset = 0.5;
    let back = 2.0;
    let start = frame.hinge(offset) + (frame.down_a * back);
    let start_face = face_holding(&roof, start);

    let on_it = walk_or_say(&roof, &graph, start, start_face, -frame.down_a, back + 1e-7);

    assert!(on_it.end_edge >= 0);
    assert!(is_the_hinge(&roof, &graph, on_it.end_edge));
    assert!(
        (on_it.end - frame.hinge(offset)).length() < 1e-12 * 10.0,
        "landing on the fold puts the end exactly on it, not a lattice past it"
    );
    assert_eq!(on_it.end_face, start_face, "the face the walk came from");
    assert_eq!(
        on_it.crossings.len(),
        1,
        "the line did reach the fold, so anything laying a polyline through this walk's crossings \
         still turns on it - what the landing changes is only where the end is said to be"
    );
    assert!((on_it.travelled - back).abs() <= 1e-12);

    // A reach that runs a thousandth PAST the fold is past it, and says so.
    let over = walk_or_say(&roof, &graph, start, start_face, -frame.down_a, back + 1e-3);

    assert_eq!(over.end_edge, -1);
    assert_eq!(over.crossings.len(), 1);
    assert!((over.travelled - (back + 1e-3)).abs() <= 1e-12);
}

/// A REACH THAT RUNS OUT AT ITS EXIT STANDS ON THE EDGE IT NAMES: the exit landing moves its end
/// onto the crossing, exactly as the fold landing beside it does, rather than recording the raw
/// `at + direction * remaining` a billionth of the reach off the edge it names.
#[test]
fn a_reach_running_out_at_its_exit_stands_on_the_edge_it_names() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let offset = 0.5;
    let back = 2.0;
    let shortfall = 1e-9;
    let start = frame.hinge(offset) + (frame.down_a * back);
    let start_face = face_holding(&roof, start);

    assert!(
        shortfall < 1e-9 * back.max(1.0),
        "the reach has to run out AT its exit - the exit reading, not the grid's"
    );

    let walk = walk_or_say(
        &roof,
        &graph,
        start,
        start_face,
        -frame.down_a,
        back - shortfall,
    );

    assert!(walk.end_edge >= 0);
    assert!(is_the_hinge(&roof, &graph, walk.end_edge));

    let v0 = vertex(&roof, graph.get_vertex0(walk.end_edge));
    let v1 = vertex(&roof, graph.get_vertex1(walk.end_edge));
    let along = (v1 - v0).get_normal();
    let off = walk.end - v0;
    let perpendicular = (off - (along * off.dot(along))).length();

    assert!(
        perpendicular < 1e-12,
        "the end stands ON the edge it names, not a billionth of the reach off it"
    );
    assert!(
        (walk.end - frame.hinge(offset)).length() < 1e-12,
        "and the point it stands on is the crossing itself"
    );
}

/// A WALK THAT RUNS ALONGSIDE A FOLD LANDS ON IT TOO. A line running nearly ALONG a fold ends a few
/// microns off a fold its end face is bounded by; the landing reads the end's PERPENDICULAR to
/// every fold bounding its end face, not just the distance still to run.
#[test]
fn a_walk_ending_alongside_a_fold_lands_on_it() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let aside = 2e-6;
    let from = -4.0;
    let distance = 3.0;
    let start = frame.hinge(from) + (frame.down_a * aside);
    let start_face = face_holding(&roof, start);

    assert!(
        aside < grid_of(&roof),
        "the end has to stand INSIDE the mesh's grid of the fold for the reading to be one"
    );

    let walk = walk_or_say(&roof, &graph, start, start_face, frame.along, distance);

    assert!((walk.travelled - distance).abs() <= 1e-12);
    assert_eq!(
        walk.crossings.len(),
        0,
        "the line never leaves the facet it set out in - the exit ahead is nine away"
    );

    assert!(
        walk.end_edge >= 0,
        "the end stands a couple of microns off the hinge, inside the mesh's grid of it"
    );
    assert!(is_the_hinge(&roof, &graph, walk.end_edge));

    // The parameter is the FOOT's - where the perpendicular from the raw end meets the fold - and
    // the end is MOVED onto it, exactly as the exit landing moves its end onto the crossing.
    let v0 = vertex(&roof, graph.get_vertex0(walk.end_edge));
    let v1 = vertex(&roof, graph.get_vertex1(walk.end_edge));
    let along = v1 - v0;
    let wanted = (frame.hinge(from + distance) - v0).dot(along) / along.length_squared();

    assert!((walk.end_parameter - wanted).abs() <= 1e-9);
    assert!(
        (v0 + (along * walk.end_parameter) - walk.end).length() < 1e-12,
        "the end IS the foot, not a couple of microns off it"
    );
    let moved = (walk.end - (start + (frame.along * distance))).length();
    assert!(
        moved > aside * 0.5 && moved < grid_of(&roof),
        "the move onto the fold is the classification's own rounding, under one grid ({moved})"
    );

    // And a line the same length run a thousandth off the fold is not alongside it: outside the
    // grid the end lies strictly inside the face and claims nothing, as it always did.
    let inside = walk_or_say(
        &roof,
        &graph,
        frame.hinge(from) + (frame.down_a * 1e-3),
        start_face,
        frame.along,
        distance,
    );

    assert_eq!(inside.end_edge, -1);
}

/// THE FAN FACE A WALK SETS OUT IN: `try_face_holding` picks the facet whose corner wedge holds the
/// direction, and refuses one that holds nothing.
#[test]
fn the_fan_face_holding_a_direction_is_the_picked_one() {
    let roof = a_roof();
    let frame = frame(&roof);
    let mut apex = 0;

    for (i, v) in roof.vertices.iter().enumerate() {
        if (v.x as f64).abs() < 1e-9 && v.y < 0.0 {
            apex = i as i32;
        }
    }

    let fan: Vec<i32> = (0..roof.faces.len())
        .filter(|&f| corners(&roof, f).contains(&apex))
        .map(|f| f as i32)
        .collect();

    assert!(fan.len() > 1);

    let (face_a, in_a) =
        try_face_holding(&roof, apex, frame.down_a + frame.along, &fan).expect("panel A holds it");
    let (face_b, in_b) =
        try_face_holding(&roof, apex, frame.down_b + frame.along, &fan).expect("panel B holds it");

    assert_ne!(face_a, face_b);
    assert!(in_a.dot(normal_of(&roof, face_a)).abs() <= 1e-12);
    assert!(in_b.dot(normal_of(&roof, face_b)).abs() <= 1e-12);

    // Straight out of the roof's edge, away from every face of the fan.
    assert!(try_face_holding(&roof, apex, -frame.along, &fan).is_err());
}

/// SETTING OUT FROM A POINT ON AN EDGE. A point on the hinge lies in BOTH panels, so the FACE a
/// walk runs in is decided by the DIRECTION, not by the label the caller happened to carry. Told
/// panel A both times, a walk into A stays in A and a walk into B sets out in B, and each lands
/// where the flattened straight line says.
#[test]
fn a_walk_setting_out_from_a_point_on_the_hinge_goes_into_the_face_its_direction_names() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let (on_a, on_b) = the_hinges_faces(&roof, &graph);
    let start = frame.hinge(0.5);
    let reach = 3.0;

    for told in [on_a, on_b] {
        let a = walk_or_say(&roof, &graph, start, told, frame.down_a, reach);

        assert!((a.end - (start + (frame.down_a * reach))).length() < 1e-12 * 10.0);
        assert_eq!(a.end_face, on_a);
        assert_eq!(a.crossings.len(), 0, "it never re-crosses the hinge");

        let b = walk_or_say(&roof, &graph, start, told, frame.down_b, reach);

        assert!((b.end - (start + (frame.down_b * reach))).length() < 1e-12 * 10.0);
        assert_eq!(b.end_face, on_b);
        assert_eq!(b.crossings.len(), 0);
    }
}

/// A JOIN BETWEEN TWO POINTS ON THE HINGE runs ALONG the hinge: the direction lies on the edge
/// itself, which belongs to both panels alike, so the answer is picked deterministically and the
/// line arrives.
#[test]
fn a_join_between_two_points_on_the_hinge_runs_along_it() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (on_a, on_b) = the_hinges_faces(&roof, &graph);
    let from = frame.hinge(-2.0);
    let to = frame.hinge(2.0);

    let crossings = try_join(&roof, &graph, &edge_at_pair, from, on_a, to, on_b, None)
        .unwrap_or_else(|refusal| panic!("{refusal:?}"));

    for crossing in &crossings {
        assert!(crossing.at.x.abs() < 1e-9, "every piece stays on the hinge");
    }
}

/// A JOIN FROM A POINT ON THE HINGE INTO THE OTHER PANEL. The end is labelled with panel A, the
/// line runs into panel B, and the first thing it must do is leave A across the very edge it stands
/// on - which is the step that used to report `JoinLeftTheStrip`.
#[test]
fn a_join_from_a_point_on_the_hinge_into_the_other_panel_is_laid() {
    let roof = a_roof();
    let frame = frame(&roof);
    let graph = roof.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (on_a, _) = the_hinges_faces(&roof, &graph);
    let from = frame.hinge(0.5);
    let to = frame.hinge(1.5) + (frame.down_b * 3.0);

    let crossings = try_join(
        &roof,
        &graph,
        &edge_at_pair,
        from,
        on_a,
        to,
        face_holding(&roof, to),
        None,
    )
    .unwrap_or_else(|refusal| panic!("{refusal:?}"));

    assert_eq!(
        crossings.len(),
        0,
        "the line sets out IN panel B from the hinge; it crosses no fold on the way"
    );

    // And the same line the other way about, from the panel back to the hinge point.
    if let Err(refusal) = try_join(
        &roof,
        &graph,
        &edge_at_pair,
        to,
        face_holding(&roof, to),
        from,
        on_a,
        None,
    ) {
        panic!("{refusal:?}");
    }
}

/// A walk that must succeed, failing the test with its refusal otherwise.
fn walk_or_say(
    roof: &Mesh,
    graph: &MeshEdgeGraph,
    start: Vector3,
    start_face: i32,
    direction: Vector3,
    distance: f64,
) -> SurfaceWalkResult {
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(graph);

    try_walk(
        roof,
        graph,
        &edge_at_pair,
        start,
        start_face,
        direction,
        distance,
        None,
    )
    .unwrap_or_else(|walk| panic!("{:?}", walk.refusal))
}

/// The two faces the roof's hinge joins, panel A's first.
fn the_hinges_faces(roof: &Mesh, graph: &MeshEdgeGraph) -> (i32, i32) {
    for edge in 0..graph.edge_count() {
        if is_the_hinge(roof, graph, edge) && graph.get_face_count(edge) == 2 {
            let one = graph.get_face(edge, 0);
            let other = graph.get_face(edge, 1);
            let (a, b, c) = triangle(roof, one);
            let centre = a + b + c;

            return if centre.x < 0.0 {
                (one, other)
            } else {
                (other, one)
            };
        }
    }

    (-1, -1)
}

fn vertex(mesh: &Mesh, index: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[index as usize])
}

fn off_the_plane_of(mesh: &Mesh, face: i32, at: Vector3) -> f64 {
    let (a, _, _) = triangle(mesh, face);

    (at - a).dot(normal_of(mesh, face)).abs()
}

fn inside(mesh: &Mesh, face: i32, at: Vector3) -> bool {
    off_the_face(mesh, face, at) < 1e-9
}

fn on_its_edge(mesh: &Mesh, graph: &MeshEdgeGraph, crossing: &SurfaceWalkCrossing) -> f64 {
    let a = vertex(mesh, graph.get_vertex0(crossing.edge));
    let b = vertex(mesh, graph.get_vertex1(crossing.edge));

    (a + ((b - a) * crossing.parameter) - crossing.at).length()
}

fn piece_lengths(start: Vector3, walk: &SurfaceWalkResult) -> Vec<f64> {
    let mut points = vec![start];
    points.extend(walk.crossings.iter().map(|c| c.at));
    points.push(walk.end);

    points.windows(2).map(|w| (w[1] - w[0]).length()).collect()
}
