//! `SectionSmoothSide` - the fillet's boundary laid on a smooth
//! tessellated side and every facet it crosses cut in two - read on the tapered barrel's bore mouths
//! ([`crate::tapered_barrel_test_meshes`]).
//!
//! The cut is TOPOLOGICAL: no tolerance, no clipping library. So what is measured here is not "is
//! the answer near the right shape" but the things a topological cut either has or has not got.
//! Area in the facet's own plane must be conserved exactly (1e-9 relative): the kept rings and the
//! band rings together are the facet, because the boundary only ever splits a ring into two. Every
//! new triangle must lie in the facet's plane and face its normal. Every crossing must be ONE pool
//! index used by both of the facets its fold joins - which is what makes the two cut facets meet
//! along that fold with the same vertex sequence read from either side.
//!
//! The one measured number is the size of the band: the fillet eats a strip of the wall one setback
//! wide all the way round the mouth, so the dropped area is the setback times the mouth's perimeter,
//! to within the few percent the rim's curvature and the mitre at each station account for.

use std::collections::{HashMap, HashSet};

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use geometry_processing::bevel::bevel_chain_linker::{self, BevelSweptChain};
use geometry_processing::bevel::sections::section_builder_run::try_build_loop;
use geometry_processing::bevel::sections::section_fan_walk::{edges_by_vertex_pair, EdgeAtPair};
use geometry_processing::bevel::sections::section_run::SectionRun;
use geometry_processing::bevel::sections::section_smooth_side::{
    try_cut_the_smooth_side, SmoothSideCut, SmoothSideFacetCut, SmoothSideRefusal,
};
use geometry_processing::bevel::sections::section_smooth_side_boundary::SmoothSideFailure;
use geometry_processing::bevel::sections::{SectionPool, SectionSide};
use geometry_processing::topology::BevelEdgeGroupResolution;

use crate::mesh_surface_distance;
use crate::tapered_barrel_test_meshes;

const RADIUS: f64 = 0.5;

const SEGMENTS: i32 = 8;

/// THE HIGH MOUTH: the 39-station loop cuts every facet its boundary crosses, and every one of those
/// cuts conserves its facet's area exactly.
#[test]
fn the_high_mouths_smooth_side_cuts_every_facet_it_crosses() {
    // The high mouth: 32 stations with the drill exactly level (39 under float Pi's tilt).
    let mouths = the_mouths();
    let read = mouths
        .iter()
        .find(|mouth| mouth.stations == 32)
        .expect("a mouth of 32 stations");

    assert_eq!(
        read.refusal,
        SmoothSideRefusal::None,
        "the cut is refused at {}",
        read.where_
    );

    let cut = read.cut.as_ref().expect("the cut");

    println!(
        "high mouth: {} facets cut, {} faces out, {} in, boundary of {}, setback {:.4} round a \
         perimeter of {:.4}, band {:.5} against {:.5}, off the plane {:.3e} against the facets' own {:.3e}",
        cut.facets.len(),
        cut.removed_faces.len(),
        cut.triangles.len(),
        cut.boundary.len(),
        read.setback,
        read.perimeter,
        read.band_area,
        read.band_wanted,
        read.worst_off_the_plane,
        read.facet_flatness
    );

    assert!(
        cut.facets.len() > 1,
        "the boundary crosses the wall's folds, so it cuts more than one facet"
    );

    assert!(
        read.worst_area_slip < 1e-9,
        "a topological split turns one ring into two and loses nothing: kept + band = facet"
    );

    assert!(
        read.worst_off_the_plane <= read.facet_flatness + 1e-9,
        "every corner of every new triangle is a point of the facet's OWN faces, so the cut adds \
         nothing to how far that piece already stands off one plane"
    );

    assert_eq!(
        read.backwards, 0,
        "every new triangle is wound the way the facet it replaces was, worst {:.3e}",
        read.worst_backwards
    );

    assert_eq!(
        read.rings_that_repeat_a_point, 0,
        "every kept ring is simple"
    );

    assert_eq!(
        read.crossings_shared_wrong, 0,
        "a crossing on a fold is ONE pool index, standing in both of the facets that fold joins"
    );

    assert_eq!(
        read.unmatched_fold_edges, 0,
        "the kept rings of two facets meeting on a fold run the same vertices in opposite \
         directions, so there is no T junction between them"
    );

    let ratio = read.band_area / read.band_wanted;
    assert!(
        (0.95..=1.05).contains(&ratio),
        "the band is one setback wide all the way round the mouth: {:.5} against {:.5}",
        read.band_area,
        read.band_wanted
    );
}

/// AND NOT JUST THE ONE: ALL FOUR bore mouths cut their smooth side the same way, every invariant
/// holding - area conserved, nothing added off the facet's plane, no backwards ear, every kept ring
/// simple, every fold crossing one shared pool index, no T junction, and a band one setback wide.
#[test]
fn every_mouth_cuts_its_smooth_side() {
    let mouths = the_mouths();

    assert_eq!(mouths.len(), 4);

    for mouth in &mouths {
        println!(
            "mouth of {}: refusal {:?} at {}",
            mouth.stations, mouth.refusal, mouth.where_
        );
    }

    assert_eq!(
        mouths
            .iter()
            .filter(|mouth| mouth.refusal == SmoothSideRefusal::None)
            .count(),
        4,
        "every one of the barrel's four mouths cuts its smooth side"
    );

    for mouth in &mouths {
        println!(
            "mouth of {}: refusal {:?}, {:?} facets, band {:.5} of {:.5}, off the plane {:.3e} of {:.3e}",
            mouth.stations,
            mouth.refusal,
            mouth.cut.as_ref().map(|cut| cut.facets.len()),
            mouth.band_area,
            mouth.band_wanted,
            mouth.worst_off_the_plane,
            mouth.facet_flatness
        );

        assert!(mouth.worst_area_slip < 1e-9);
        assert!(mouth.worst_off_the_plane <= mouth.facet_flatness + 1e-9);
        assert_eq!(mouth.backwards, 0);
        assert_eq!(mouth.rings_that_repeat_a_point, 0);
        assert_eq!(mouth.crossings_shared_wrong, 0);
        assert_eq!(mouth.unmatched_fold_edges, 0);
        let ratio = mouth.band_area / mouth.band_wanted;
        assert!((0.95..=1.05).contains(&ratio), "band ratio {ratio}");
    }
}

/// ONE POINT, MADE ONCE: the run's own wall-side boundary point is ON the wall - inside the facet
/// the builder walked it into, to a billionth - and the cut uses THAT pool index rather than
/// minting a second point for the same thing. This is what lets the loft's last row meet the cut
/// wall.
#[test]
fn every_mouths_wall_side_boundary_is_the_runs_own_walked_point() {
    let mouths = the_mouths();

    assert_eq!(mouths.len(), 4);

    for mouth in &mouths {
        println!(
            "mouth of {}: {} of {} stations walked, worst off its own facet {:.3e}, {} boundary \
             points the cut minted for itself",
            mouth.stations,
            mouth.walked_stations,
            mouth.stations,
            mouth.worst_off_its_face,
            mouth.boundary_points_not_the_runs
        );

        assert_eq!(mouth.refusal, SmoothSideRefusal::None);

        assert!(
            mouth.walked_stations > 0,
            "a mouth's wall-side reach crosses the wall's folds, so it is walked"
        );

        assert!(
            mouth.worst_off_its_face < 1e-9,
            "every wall-side boundary point stands in the facet the run walked it into - the \
             one-plane offset stood 0.0446 off the wall"
        );

        assert_eq!(
            mouth.boundary_points_not_the_runs, 0,
            "the cut consumes the run's boundary point; two points for one thing is exactly what \
             the loft could not then meet"
        );
    }
}

/// A PLACE NAMES THE FACE ITS POINT IS IN, read on ALL SIX of the barrel's closed loops - the four
/// mouths and the two 48-edge rims. Each station of a cut carries a place, one face of the part or
/// one mesh edge of it, and the point stands there: inside the named face or on the named edge's
/// segment.
///
/// A join out of a station is laid IN the face its place names, so a place naming a face the point
/// is not in cannot be walked out of. Only a walk or a join that LANDED writes a face; a station
/// whose walk and join were both refused carries none (-1) and `SectionSmoothSide.TryLayTheBoundary`
/// walks it itself. All six loops cut: a rim station's offset up its wall fold sets out along the
/// fold itself rather than a hair outside the face.
#[test]
fn every_stations_place_names_the_face_its_point_is_in() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let feature_edges = the_feature_edges(&resolution);
    let loops: Vec<BevelSweptChain> = bevel_chain_linker::link(&resolution.edges)
        .into_iter()
        .filter(|chain| chain.is_closed)
        .collect();
    let mut said = String::new();
    let mut wrong = 0;
    let mut read = 0;
    let mut cut_loops = 0;
    let mut worst = 0.0f64;

    assert_eq!(loops.len(), 6);

    for one_loop in &loops {
        let ring: Vec<i32> = one_loop
            .edges
            .iter()
            .map(|edge| edge.from_vertex())
            .collect();
        let mut pool = SectionPool::new(&part.vertices);

        let run = try_build_loop(
            &ring,
            Some(&resolution.features),
            &part,
            &graph,
            &mut pool,
            RADIUS,
            SEGMENTS,
            true,
        )
        .unwrap_or_else(|why| {
            panic!(
                "the loop of {} builds its run on the conditioned part: {why:?}",
                ring.len()
            )
        });

        let left_is_smooth = left_is_smooth(&run);
        let is_a_feature = |edge: i32| feature_edges.contains(&edge);

        let (cut, done) = try_cut_the_smooth_side(
            &part,
            &graph,
            &edge_at_pair,
            &mut pool,
            &run,
            left_is_smooth,
            Some(&is_a_feature),
        );

        if let Err(failure) = done {
            println!(
                "loop of {}: refused {failure:?}, no place written",
                ring.len()
            );

            continue;
        }

        cut_loops += 1;

        for &station in &cut.station_boundary {
            read += 1;

            let at = pool.position_of(station);
            let place = &cut.places[&station];
            let off = if place.face >= 0 {
                off_the_face(&part, place.face, at)
            } else {
                off_the_edge(&part, &graph, place.edge, at)
            };

            worst = worst.max(off);

            if off > 1e-9 {
                wrong += 1;
                let named = if place.face >= 0 {
                    format!("face {}", place.face)
                } else {
                    format!("edge {}", place.edge)
                };
                said.push_str(&format!(
                    "loop of {}, station at {at:?}: place names {named}, which the point stands {off:.6} off\n",
                    ring.len()
                ));
            }
        }
    }

    println!(
        "{cut_loops} of {} loops cut, {read} places read, {wrong} name something their point is not \
         in, worst {worst:.3e} off",
        loops.len()
    );
    println!("{said}");

    assert_eq!(
        cut_loops, 6,
        "the four mouths and both 48-edge rims cut: a rim station's offset up its wall fold sets \
         out along the fold itself rather than a hair outside the face"
    );

    assert_eq!(
        wrong, 0,
        "a place is written only by a walk or a join that LANDED, so the point is in it: {said}"
    );
}

/// How far a point stands from a face of the part, clamped onto the triangle.
fn off_the_face(part: &Mesh, face: i32, at: Vector3) -> f64 {
    let (a, b, c) = corners(part, face);

    (mesh_surface_distance::closest_point_on_triangle(at, a, b, c) - at).length()
}

/// How far a point stands from the SEGMENT of one mesh edge - not from its infinite
/// line.
fn off_the_edge(part: &Mesh, graph: &MeshEdgeGraph, edge: i32, at: Vector3) -> f64 {
    let a = vertex(part, graph.get_vertex0(edge));
    let along = vertex(part, graph.get_vertex1(edge)) - a;
    let t = if along.length_squared() < 1e-300 {
        0.0
    } else {
        ((at - a).dot(along) / along.length_squared()).clamp(0.0, 1.0)
    };

    (a + (along * t) - at).length()
}

/// How far each wall-side boundary point stands from the facet the
/// run says it is in, and whether the cut used the run's own pool index for it.
fn measure_the_boundary_is_the_runs(
    part: &Mesh,
    run: &SectionRun,
    pool: &SectionPool,
    cut: &SmoothSideCut,
    wall_is_left: bool,
    read: &mut MouthCut,
) {
    for (i, section) in run.sections.iter().enumerate() {
        let boundary = if wall_is_left {
            section.left_boundary
        } else {
            section.right_boundary
        };
        let face = if wall_is_left {
            section.left_walk_face
        } else {
            section.right_walk_face
        };

        if cut.station_boundary[i] != boundary {
            read.boundary_points_not_the_runs += 1;
        }

        if face < 0 {
            continue;
        }

        read.walked_stations += 1;

        let at = pool.position_of(boundary);
        let (a, b, c) = corners(part, face);

        read.worst_off_its_face = read
            .worst_off_its_face
            .max((mesh_surface_distance::closest_point_on_triangle(at, a, b, c) - at).length());
    }
}

fn vertex(part: &Mesh, index: i32) -> Vector3 {
    Vector3::from_vector3_float(part.vertices[index as usize])
}

fn corners(part: &Mesh, face: i32) -> (Vector3, Vector3, Vector3) {
    let at = &part.faces[face as usize];

    (
        vertex(part, at.v0),
        vertex(part, at.v1),
        vertex(part, at.v2),
    )
}

fn the_feature_edges(resolution: &BevelEdgeGroupResolution) -> HashSet<i32> {
    resolution
        .features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect()
}

/// The WALL is the side of the mouth's walk whose stations offset into a smooth tessellated face;
/// the bore side is the ruled one.
fn left_is_smooth(run: &SectionRun) -> bool {
    run.sections
        .iter()
        .filter(|section| section.left_kind == SectionSide::Offset)
        .count()
        >= run
            .sections
            .iter()
            .filter(|section| section.right_kind == SectionSide::Offset)
            .count()
}

/// Every closed loop of the wall's rim that is not one of the two 48-edge rims, cut.
fn the_mouths() -> Vec<MouthCut> {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let feature_edges = the_feature_edges(&resolution);

    bevel_chain_linker::link(&resolution.edges)
        .iter()
        .filter(|chain| chain.is_closed && chain.edges.len() != 48)
        .map(|one_loop| {
            cut_mouth(
                &part,
                &resolution,
                &graph,
                &edge_at_pair,
                &feature_edges,
                one_loop,
            )
        })
        .collect()
}

/// One mouth's run built and its smooth side cut and measured.
fn cut_mouth(
    part: &Mesh,
    resolution: &BevelEdgeGroupResolution,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    feature_edges: &HashSet<i32>,
    one_loop: &BevelSweptChain,
) -> MouthCut {
    let ring: Vec<i32> = one_loop
        .edges
        .iter()
        .map(|edge| edge.from_vertex())
        .collect();
    let mut pool = SectionPool::new(&part.vertices);

    let run = try_build_loop(
        &ring,
        Some(&resolution.features),
        part,
        graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        true,
    )
    .unwrap_or_else(|why| panic!("every mouth builds its run on the conditioned part: {why:?}"));

    let mut read = MouthCut {
        stations: run.sections.len(),
        band_wanted: 1.0,
        ..MouthCut::default()
    };

    // The WALL is the side of the mouth's walk whose stations offset into a smooth tessellated face;
    // the bore side is the ruled one, and is not this box's business.
    let left_is_smooth = left_is_smooth(&run);
    let is_a_feature = |edge: i32| feature_edges.contains(&edge);

    let (cut, done) = try_cut_the_smooth_side(
        part,
        graph,
        edge_at_pair,
        &mut pool,
        &run,
        left_is_smooth,
        Some(&is_a_feature),
    );

    match done {
        Ok(()) => {}
        Err(SmoothSideFailure::Refused(refusal, at)) => {
            read.refusal = refusal;
            read.where_ = format!("{at:?}");

            return read;
        }
        Err(SmoothSideFailure::NotANumber(nan)) => panic!("the cut minted a NaN point: {nan}"),
    }

    measure(part, graph, &pool, &cut, &mut read);
    measure_the_boundary_is_the_runs(part, &run, &pool, &cut, left_is_smooth, &mut read);

    read.setback = run.sections.iter().map(|s| s.setback).sum::<f64>() / run.sections.len() as f64;
    read.perimeter = run
        .edges
        .iter()
        .map(|edge| (vertex(part, edge.to_vertex()) - vertex(part, edge.from_vertex())).length())
        .sum();

    // Steiner: the band is the mouth's rim swept one setback OUTWARD over the wall, and a closed
    // convex rim's outward offset carries a curvature term of pi d squared beyond perimeter times d.
    read.band_wanted =
        (read.perimeter * read.setback) + (std::f64::consts::PI * read.setback * read.setback);

    read.cut = Some(cut);

    read
}

/// The invariants of one cut, accumulated into `read`.
fn measure(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &SectionPool,
    cut: &SmoothSideCut,
    read: &mut MouthCut,
) {
    let mut on_a_fold_edge: HashMap<(i32, i32), i32> = HashMap::new();

    for facet in &cut.facets {
        let whole = facet
            .faces
            .iter()
            .map(|&face| twice_the_area(part, face, facet.normal))
            .sum::<f64>()
            / 2.0;
        let kept: f64 = facet
            .kept_rings
            .iter()
            .map(|ring_of| ring_area(pool, ring_of, facet.normal))
            .sum();
        let band: f64 = facet
            .band_rings
            .iter()
            .map(|ring_of| ring_area(pool, ring_of, facet.normal))
            .sum();

        read.worst_area_slip = read
            .worst_area_slip
            .max((kept + band - whole).abs() / 1e-12f64.max(whole));
        read.band_area += band;

        for ring_of in &facet.kept_rings {
            let distinct: HashSet<i32> = ring_of.iter().copied().collect();

            if distinct.len() != ring_of.len() {
                read.rings_that_repeat_a_point += 1;
            }

            for i in 0..ring_of.len() {
                let side = (ring_of[i], ring_of[(i + 1) % ring_of.len()]);

                *on_a_fold_edge.entry(side).or_insert(0) += 1;
            }
        }

        for &(v0, v1, v2) in &facet.triangles {
            let a = pool.position_of(v0);
            let b = pool.position_of(v1);
            let c = pool.position_of(v2);
            let anchor = vertex(part, part.faces[facet.faces[0] as usize].v0);

            for corner in [a, b, c] {
                read.worst_off_the_plane = read
                    .worst_off_the_plane
                    .max((corner - anchor).dot(facet.normal).abs());
            }

            read.facet_flatness = read.facet_flatness.max(flatness(part, facet));

            let twice = (b - a).cross(c - a).dot(facet.normal);

            // An ear of no area at all is wound neither way: the ring's own points are the mesh's,
            // in single precision, so three near-collinear ones sign at the lattice and not above it.
            if twice / 2.0 < -1e-9 * whole {
                read.backwards += 1;
            }

            read.worst_backwards = read.worst_backwards.min(twice / 2.0);
        }
    }

    // A directed edge of a kept ring that lies on a fold between two CUT facets must be run once each
    // way - once by each facet - or the two sides of that fold do not meet vertex for vertex.
    for (&side, &times) in &on_a_fold_edge {
        if (times != 1 || !on_a_fold_edge.contains_key(&(side.1, side.0)))
            && on_a_fold_between_cut_facets(part, graph, cut, side)
        {
            read.unmatched_fold_edges += 1;
        }
    }

    measure_the_crossings(graph, cut, read);
}

/// Whether a ring edge runs along a fold that two CUT facets share -
/// the only place a T junction could be left. An edge on the outer rim of the cut region has nothing
/// on the other side to match.
fn on_a_fold_between_cut_facets(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    cut: &SmoothSideCut,
    side: (i32, i32),
) -> bool {
    let cut_faces: HashSet<i32> = cut.removed_faces.iter().copied().collect();

    for facet in &cut.facets {
        for &face in &facet.faces {
            let at = &part.faces[face as usize];
            let corners = [at.v0, at.v1, at.v2];

            for i in 0..3 {
                let one = corners[i];
                let other = corners[(i + 1) % 3];

                if (one != side.0 && one != side.1) || (other != side.0 && other != side.1) {
                    continue;
                }

                for e in 0..graph.edge_count() {
                    if (graph.get_vertex0(e) == one && graph.get_vertex1(e) == other)
                        || (graph.get_vertex0(e) == other && graph.get_vertex1(e) == one)
                    {
                        return graph.get_face_count(e) == 2
                            && cut_faces.contains(&graph.get_face(e, 0))
                            && cut_faces.contains(&graph.get_face(e, 1));
                    }
                }
            }
        }
    }

    false
}

/// Every boundary vertex that stands on a fold between two cut facets must
/// be carried by BOTH of them and by no others.
fn measure_the_crossings(graph: &MeshEdgeGraph, cut: &SmoothSideCut, read: &mut MouthCut) {
    let mut facet_of: HashMap<i32, usize> = HashMap::new();

    for (i, facet) in cut.facets.iter().enumerate() {
        for &face in &facet.faces {
            facet_of.insert(face, i);
        }
    }

    for vertex in &cut.boundary {
        let place = &cut.places[vertex];

        if place.edge < 0 || graph.get_face_count(place.edge) != 2 {
            continue;
        }

        let (Some(one), Some(other)) = (
            facet_of.get(&graph.get_face(place.edge, 0)),
            facet_of.get(&graph.get_face(place.edge, 1)),
        ) else {
            continue;
        };

        if one == other {
            continue;
        }

        let carriers = cut
            .facets
            .iter()
            .filter(|facet| {
                facet
                    .kept_rings
                    .iter()
                    .chain(facet.band_rings.iter())
                    .any(|ring_of| ring_of.contains(vertex))
            })
            .count();

        if carriers != 2 {
            read.crossings_shared_wrong += 1;
        }
    }
}

/// How far a facet's OWN corners already stand off the plane of its first face. A
/// flat piece is gathered across edges the half-degree coplanarity test calls flat, over vertices
/// the mesh stores in SINGLE precision, so it is not exactly planar and there is no 1e-9 to hold
/// the new triangles to. What must hold is that the cut adds nothing.
fn flatness(part: &Mesh, facet: &SmoothSideFacetCut) -> f64 {
    let anchor = vertex(part, part.faces[facet.faces[0] as usize].v0);
    let mut worst = 0.0f64;

    for &face in &facet.faces {
        let at = &part.faces[face as usize];

        for corner in [at.v0, at.v1, at.v2] {
            worst = worst.max((vertex(part, corner) - anchor).dot(facet.normal).abs());
        }
    }

    worst
}

pub(crate) fn twice_the_area(part: &Mesh, face: i32, normal: Vector3) -> f64 {
    let (a, b, c) = corners(part, face);

    (b - a).cross(c - a).dot(normal)
}

pub(crate) fn ring_area(pool: &SectionPool, ring: &[i32], normal: Vector3) -> f64 {
    let mut twice = Vector3::ZERO;

    for i in 0..ring.len() {
        twice += pool
            .position_of(ring[i])
            .cross(pool.position_of(ring[(i + 1) % ring.len()]));
    }

    twice.dot(normal) / 2.0
}

/// One mouth's cut and every reading taken of it.
#[derive(Default)]
struct MouthCut {
    stations: usize,
    cut: Option<SmoothSideCut>,
    refusal: SmoothSideRefusal,
    where_: String,
    worst_area_slip: f64,
    worst_off_the_plane: f64,
    backwards: i32,
    worst_backwards: f64,
    rings_that_repeat_a_point: i32,
    crossings_shared_wrong: i32,
    unmatched_fold_edges: i32,
    setback: f64,
    perimeter: f64,
    facet_flatness: f64,
    band_area: f64,
    band_wanted: f64,
    walked_stations: i32,
    worst_off_its_face: f64,
    boundary_points_not_the_runs: i32,
}
