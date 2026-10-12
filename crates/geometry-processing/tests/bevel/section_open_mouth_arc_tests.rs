//! Port of `SectionOpenMouthArcTests.cs`: THE TOP MOUTH ARC of `InnerFaceTestMesh` - the part of
//! the raked bore's top mouth that lies on the cube's TOP face, 40 edges, an OPEN chain whose two
//! ends sit where the mouth crosses the cube's top/-X edge. The arc readers the class shares with
//! its neighbours (`TheTopMouthArc`, `TheTopMouthArcWalk`, `TheSameArcOn`) live in
//! [`crate::open_mouth_arc_walk`]. C# `[Arguments]` rows are loops over the same-named test.
//!
//! **The reading** (measured, not assumed). The arc's two ends are `(-19, -44, 20)` and
//! `(-19, -34, 20)`, both ON the -X wall's plane `x = -19` to the precision the fixture's float
//! coordinates carry. At each end THREE creases meet: the PICKED arc's own last edge (0.9681), the
//! cube's top/-X edge (5), and the mouth's UNPICKED continuation down the -X wall, whose FIRST CHORD
//! IS ONLY 0.1214 LONG - so the end's wall-side landing is a walk down a polyline. The arrival is
//! nearly square (86.81 degrees to the -X plane), so the profile slid onto that plane is barely
//! stretched.
//!
//! Skipped as in C# (`[Skip]`):
//! - `TheEndSectionsDieOnTheWall` (both rows): "the transverse face has no rebuild: at 0.5 the mesh
//!   is open along the -X wall, at 2 the smooth cut refuses first".
//! - `TheOpenArcAssemblesAtTwo`: "far alone still mints the twin" - the station's offset direction
//!   is tilted 1.610e-5 off the fold it runs along by the normals of the 95:1 needle facets at
//!   vertex 71, read on float vertices quantised at 1.526e-5 there; owed is `SectionBuilder` taking
//!   such a boundary off the fold.

use std::collections::HashSet;

use polygon_mesh::Mesh;
use vector_math::Vector3;

use geometry_processing::bevel::sections::section_builder_run::try_build_run;
use geometry_processing::bevel::sections::section_fan_walk::edges_by_vertex_pair;
use geometry_processing::bevel::sections::section_smooth_side::try_cut_the_smooth_side;
use geometry_processing::bevel::sections::{
    section_assembly, section_loft, SectionKind, SectionPool,
};
use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{FeatureEdge, FeatureEdgeSet};

use crate::open_mouth_arc_walk::{
    the_same_arc_on, the_top_mouth_arc, the_top_mouth_arc_walk, MINUS_X_PLANE,
};
use crate::{inner_face_test_mesh, mesh_measurements, rigid_motion_meshes};

/// What the fixture's coordinates are stored to - single precision, so a point that IS on the -X
/// plane reads 2e-6 off it. Nothing measured off this part may be asked for to 1e-9.
const FLOAT_COORDINATE_TOLERANCE: f64 = 1e-5;

/// The bore's tessellated wall is the arc's LEFT side; see `SectionOpenMouthArcCutTests`.
const THE_BORE_WALL_IS_THE_LEFT: bool = true;

fn vertex(part: &Mesh, index: i32) -> Vector3 {
    Vector3::from_vector3_float(part.vertices[index as usize])
}

#[test]
fn the_arcs_geometry_reads() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );

    let arc = the_top_mouth_arc(&part, &features);

    assert_eq!(arc.len(), 40);

    let mut said = String::new();

    let first_vertex = arc[0].from_vertex();
    let last_vertex = arc[arc.len() - 1].to_vertex();

    said.push_str(&format!(
        "ends {} and {}\n",
        show(vertex(&part, first_vertex)),
        show(vertex(&part, last_vertex))
    ));

    for (label, edge, end_vertex) in [
        ("start", &arc[0], first_vertex),
        ("finish", &arc[arc.len() - 1], last_vertex),
    ] {
        // The direction the chain runs INTO the end, which is the axis the end profile is slid along.
        let direction = if label == "start" {
            (edge.from() - edge.to()).get_normal()
        } else {
            (edge.to() - edge.from()).get_normal()
        };

        let end = vertex(&part, end_vertex);

        // A raked mouth arrives at the wall obliquely: this is the angle between the chain's
        // direction and the -X plane (90 degrees would be square onto it).
        let to_plane =
            90.0 - (1.0_f64.min(direction.x.abs()).acos() * 180.0 / std::f64::consts::PI);

        said.push_str(&format!(
            "{label}: at {} direction {} meets the -X plane at {to_plane:.3} degrees\n",
            show(end),
            show(direction)
        ));

        assert!(
            (86.8..=86.83).contains(&to_plane),
            "the arc arrives very nearly square onto the wall, so the profile slid onto that plane \
             is barely stretched - {to_plane}"
        );

        // The creases leaving the end vertex, which is what the end's two boundary points slide
        // down.
        for &index in features.edges_at_vertex(end_vertex) {
            let feature = &features.edges()[index as usize];
            let other = if feature.start_vertex == end_vertex {
                feature.end
            } else {
                feature.start
            };

            said.push_str(&format!(
                "    crease to {} length {:.4}\n",
                show(other),
                (other - end).length()
            ));
        }
    }

    println!("{said}");

    // Both ends stand ON the -X wall's plane, which is what makes the end section a slide onto
    // that plane rather than a cut across open air.
    assert!((vertex(&part, first_vertex).x - MINUS_X_PLANE).abs() < FLOAT_COORDINATE_TOLERANCE);
    assert!((vertex(&part, last_vertex).x - MINUS_X_PLANE).abs() < FLOAT_COORDINATE_TOLERANCE);

    // The three creases at one end, by length: the picked arc (0.9681), the cube's top/-X edge (5)
    // and the mouth's continuation down the wall, whose first chord is the short one the wall-side
    // slide will run straight past.
    let mut lengths: Vec<f64> = features
        .edges_at_vertex(first_vertex)
        .iter()
        .map(|&index| {
            length_at(
                &features.edges()[index as usize],
                vertex(&part, first_vertex),
            )
        })
        .collect();
    lengths.sort_by(f64::total_cmp);

    assert_eq!(lengths.len(), 3);
    assert!(
        (0.12..=0.123).contains(&lengths[0]),
        "the wall crease's first chord is far shorter than any radius worth filleting, so the \
         end's wall-side landing is a walk down a polyline and not a point on one chord - {}",
        lengths[0]
    );
    assert!((0.967..=0.969).contains(&lengths[1]), "{}", lengths[1]);
    assert!((4.999..=5.001).contains(&lengths[2]), "{}", lengths[2]);
}

/// A PLACE NAMES THE FACE ITS POINT IS IN. Every station of the bore wall's cut carries a place -
/// either one face of the part or one mesh edge of it - and the point that place belongs to STANDS
/// there: inside the named face, or on the named edge's own segment. Read at both radii in every
/// rigid placement. The bar is a billionth ABSOLUTE: a place is a topological reading.
#[test]
fn every_stations_place_names_the_face_its_point_is_in() {
    for radius in [2.0, 0.5] {
        let mut said = String::new();
        let mut wrong = 0;
        let mut read = 0;
        let mut worst = 0.0_f64;
        let walk = the_top_mouth_arc_walk();

        for (name, motion) in rigid_motion_meshes::the_motions() {
            let mut part = inner_face_test_mesh::the_part();

            rigid_motion_meshes::move_mesh(&mut part, motion);

            let graph = part.get_mesh_edge_graph();
            let features = FeatureEdgeSet::create_with_graph(
                &part,
                &graph,
                inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
            );
            let edge_at_pair = edges_by_vertex_pair(&graph);
            let feature_edges: HashSet<i32> =
                features.edges().iter().map(|f| f.mesh_edge_index).collect();
            let arc = the_same_arc_on(&features, &walk);
            let mut pool = SectionPool::new(&part.vertices);

            let Ok(run) = try_build_run(
                &arc,
                false,
                &part,
                &graph,
                &mut pool,
                radius,
                inner_face_test_mesh::SEGMENTS,
                true,
                Some(&features),
            ) else {
                continue;
            };

            let is_a_feature = |edge: i32| feature_edges.contains(&edge);
            let (cut, _) = try_cut_the_smooth_side(
                &part,
                &graph,
                &edge_at_pair,
                &mut pool,
                &run,
                THE_BORE_WALL_IS_THE_LEFT,
                Some(&is_a_feature),
            );

            for &station in &cut.station_boundary {
                read += 1;

                let at = pool.position_of(station);
                let place = &cut.places[&station];
                let off = if place.face >= 0 {
                    let face = &part.faces[place.face as usize];
                    off_the_triangle(
                        vertex(&part, face.v0),
                        vertex(&part, face.v1),
                        vertex(&part, face.v2),
                        at,
                    )
                } else {
                    off_the_segment(
                        vertex(&part, graph.get_vertex0(place.edge)),
                        vertex(&part, graph.get_vertex1(place.edge)),
                        at,
                    )
                };

                worst = worst.max(off);

                if off > 1e-9 {
                    wrong += 1;
                    said.push_str(&format!(
                        "{name} station at {}: place names {}, which the point stands {off:.6} off\n",
                        show(at),
                        if place.face >= 0 {
                            format!("face {}", place.face)
                        } else {
                            format!("edge {}", place.edge)
                        }
                    ));
                }
            }
        }

        println!(
            "radius {radius}: {read} places read, {wrong} name something their point is not in, \
             worst {worst:.3e} off"
        );
        println!("{said}");

        assert!(read > 0);

        assert_eq!(
            wrong, 0,
            "a place is written only by a walk or a join that LANDED, so the point is in it: {said}"
        );
    }
}

/// The whole OPEN run: 40 edges and 41 sections, the two outer ones `SectionKind::End` and every
/// one between them a station of the arc.
#[test]
fn the_open_run_builds() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );

    let arc = the_top_mouth_arc(&part, &features);
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_run(
        &arc,
        false,
        &part,
        &graph,
        &mut pool,
        inner_face_test_mesh::SIZE,
        inner_face_test_mesh::SEGMENTS,
        true,
        None,
    )
    .unwrap_or_else(|failure| panic!("an open run's ends are sections now: {failure:?}"));

    assert!(!run.is_closed);
    assert_eq!(run.sections.len(), arc.len() + 1);
    assert_eq!(run.sections[0].kind, SectionKind::End);
    assert_eq!(run.sections[run.sections.len() - 1].kind, SectionKind::End);

    for i in 1..run.sections.len() - 1 {
        assert_ne!(
            run.sections[i].kind,
            SectionKind::End,
            "only the two vertices the chain STOPS at are ends"
        );
    }

    // Every edge's strip now has a section at each of its two vertices, which is what the loft
    // asks.
    for i in 0..run.count() {
        assert_eq!(run.start_of(i).chain_vertex, arc[i as usize].from_vertex());
        assert_eq!(
            run.end_of(i)
                .expect("an open run's edge has an end section")
                .chain_vertex,
            arc[i as usize].to_vertex()
        );
    }
}

/// THE OPEN RUN LOFTS. Its INTERIOR is read exactly as a closed run's, and its two END stations
/// are never split, never walked, and stand as ordinary unsplit chord ends for the station beside
/// them. So the boundary points are ON the bore's tessellated wall, and the ladder is laid all the
/// way.
#[test]
fn the_open_run_lofts_along_the_surface() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );

    let arc = the_top_mouth_arc(&part, &features);
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_run(
        &arc,
        false,
        &part,
        &graph,
        &mut pool,
        inner_face_test_mesh::SIZE,
        inner_face_test_mesh::SEGMENTS,
        true,
        Some(&features),
    )
    .unwrap_or_else(|why_not| panic!("the open run builds: {why_not:?}"));

    let mut faces = Vec::new();

    // Every section IS drawn, and all in the same number of segments - the ladder has rungs to
    // lay.
    assert!(run
        .sections
        .iter()
        .all(|section| section.segments() == inner_face_test_mesh::SEGMENTS));

    let lofted = section_loft::try_emit(&run, &pool, &mut faces, None);

    println!(
        "lofted {} with {} faces{}",
        lofted.is_ok(),
        faces.len(),
        match &lofted {
            Ok(()) => String::new(),
            Err(failure) => format!(", refused {:?} at {:?}", failure.code(), failure.args()),
        }
    );

    if let Err(failure) = &lofted {
        panic!(
            "every gap of the open run lays its ladder: {:?}",
            failure.code()
        );
    }

    assert!(!faces.is_empty());

    // EVERY INTERIOR BOUNDARY POINT, both sides, stands ON the part - the bore's wall side because
    // it was laid along that wall's facets, the top side because that face is flat.
    let mut off = 0.0_f64;
    let mut backwards = 0;
    let mut previous: [Option<Vector3>; 2] = [None, None];

    for i in 1..run.sections.len() - 1 {
        let section = &run.sections[i];

        for (side, before_on_this_side) in previous.iter_mut().enumerate() {
            let at = pool.position_of(if side == 0 {
                section.left_boundary
            } else {
                section.right_boundary
            });

            off = off.max(distance_to_the_nearest_face(&part, at));

            // In order ALONG THE ARC: each side's boundary row advances with the chain, measured
            // against the station's own tangent rather than any one axis.
            if let Some(before) = *before_on_this_side {
                if (at - before).dot(section.tangent) <= 0.0 {
                    backwards += 1;
                }
            }

            *before_on_this_side = Some(at);
        }
    }

    println!("interior boundaries up to {off:.9} off the part, {backwards} backwards");

    assert!(
        off < FLOAT_COORDINATE_TOLERANCE,
        "a boundary laid ALONG the surface stands on a real facet of it - {off}"
    );

    assert_eq!(
        backwards, 0,
        "each side's boundary points run in order along the arc"
    );
}

#[test]
fn the_open_arc_assembles_at_a_half() {
    assemble_the_open_arc_in_every_placement(0.5);
}

fn assemble_the_open_arc_in_every_placement(radius: f64) {
    let mut signature = None;

    // The pick, read once where the fixture was made: a rigid motion moves no index, so every
    // motion below fillets the very same forty edges.
    let walk = the_top_mouth_arc_walk();

    for (name, motion) in rigid_motion_meshes::the_motions() {
        let mut part = inner_face_test_mesh::the_part();

        rigid_motion_meshes::move_mesh(&mut part, motion);

        let graph = part.get_mesh_edge_graph();
        let features = FeatureEdgeSet::create_with_graph(
            &part,
            &graph,
            inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
        );
        let arc = the_same_arc_on(&features, &walk);

        assert_eq!(
            arc.len(),
            40,
            "{name}: the same 40 edges wherever it stands"
        );
        let mut pool = SectionPool::new(&part.vertices);

        let run = try_build_run(
            &arc,
            false,
            &part,
            &graph,
            &mut pool,
            radius,
            inner_face_test_mesh::SEGMENTS,
            true,
            Some(&features),
        )
        .unwrap_or_else(|why_not| panic!("{name}: the open run builds: {why_not:?}"));

        let mesh = section_assembly::try_assemble(&part, &graph, &run, &mut pool)
            .unwrap_or_else(|failure| panic!("{name}: the open arc assembles: {failure:?}"));

        assert_closed_and_smaller(&part, &mesh, name);

        let moved = rigid_motion_meshes::signature_of(Some(&mesh));

        match &signature {
            None => signature = Some(moved),
            Some(first) => assert_eq!(
                &moved, first,
                "{name}: the same part filleted the same way wherever it stands"
            ),
        }
    }
}

#[test]
fn the_open_arc_assembles_where_the_part_was_made() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let arc = the_top_mouth_arc(&part, &features);
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_run(
        &arc,
        false,
        &part,
        &graph,
        &mut pool,
        0.5,
        inner_face_test_mesh::SEGMENTS,
        true,
        Some(&features),
    )
    .unwrap_or_else(|why_not| panic!("the open run builds: {why_not:?}"));

    let mesh = section_assembly::try_assemble(&part, &graph, &run, &mut pool)
        .unwrap_or_else(|failure| panic!("the open arc assembles: {failure:?}"));

    assert_closed_and_smaller(&part, &mesh, "where it was made");
}

/// The readings both assembly tests take: closed, manifold, one shell, readable by a second pass,
/// and smaller than the part - a fillet on a convex mouth only ever takes material away.
fn assert_closed_and_smaller(part: &Mesh, mesh: &Mesh, name: &str) {
    let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);
    assert_eq!(open, 0, "{name}: open at {open_at:?}");
    assert!(rigid_motion_meshes::most_faces_on_an_edge(mesh) <= 2);
    assert_eq!(mesh_measurements::shell_count(mesh), 1);

    let (unreadable, unreadable_at) = mesh_measurements::unclassifiable_edges(Some(mesh));
    assert_eq!(
        unreadable, 0,
        "{name}: a second pass reads {unreadable_at} as a torn face"
    );

    assert!(
        mesh_measurements::signed_volume(mesh).abs() < mesh_measurements::signed_volume(part).abs()
    );
}

fn distance_to_the_nearest_face(part: &Mesh, point: Vector3) -> f64 {
    let mut nearest = f64::MAX;

    for face in part.faces.iter() {
        nearest = nearest.min(off_the_triangle(
            vertex(part, face.v0),
            vertex(part, face.v1),
            vertex(part, face.v2),
            point,
        ));

        if nearest < 1e-12 {
            break;
        }
    }

    nearest
}

fn off_the_triangle(a: Vector3, b: Vector3, c: Vector3, point: Vector3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let mut normal = ab.cross(ac);
    let area = normal.length();

    // C#'s `!(area > 0)`: a NaN area falls here too.
    if area.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return (point - a).length();
    }

    normal /= area;

    let in_plane = point - (normal * (point - a).dot(normal));

    // Barycentric coordinates of the projected point, clamped onto the triangle's edges.
    let u = ab.cross(in_plane - a).dot(normal) / area;
    let v = (in_plane - c).cross(ac).dot(normal) / area;
    let w = 1.0 - u - v;

    if u >= 0.0 && v >= 0.0 && w >= 0.0 {
        return (point - in_plane).length();
    }

    off_the_segment(a, b, point).min(off_the_segment(b, c, point).min(off_the_segment(c, a, point)))
}

fn off_the_segment(from: Vector3, to: Vector3, point: Vector3) -> f64 {
    let along = to - from;
    let span = along.length_squared();
    let at = if span > 0.0 {
        ((point - from).dot(along) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };

    (point - (from + (along * at))).length()
}

fn length_at(edge: &FeatureEdge, end: Vector3) -> f64 {
    (if (edge.start - end).length() < (edge.end - end).length() {
        edge.end - edge.start
    } else {
        edge.start - edge.end
    })
    .length()
}

fn show(point: Vector3) -> String {
    format!("({:.4}, {:.4}, {:.4})", point.x, point.y, point.z)
}
