//! THE THREE ROWS OF "Edge Loop And Face Should Match.mcx",
//! on the part's own mesh ([`crate::edge_loop_and_face_test_mesh`]) - the face group pick and the
//! Edge-plus-TangentChain pair, both routed to the sections and both built, and the corner where
//! the 20 mm diameter meets the arc's 1.6516 mm first chord, read.
//!
//! The corner's setback of 2 reaches past the station at the far end of that chord, whose flat-side
//! boundary stands 0.686 BEHIND the corner's offset-meet; that station is ABSORBED by the corner
//! (`SectionAbsorbedSplits`), which is what lets both picks build. Built through the shared
//! `surface_test_build` with the sections flag pinned on.

use std::rc::Rc;

use geometry_processing::bevel::sections::{
    section_builder_run, section_corner, section_routing, SectionPool, SectionSide,
};
use geometry_processing::bevel::surface::{surface_mesh_closure, SurfaceBevelPlan};
use geometry_processing::bevel::BevelAdvisory;
use geometry_processing::topology::{BevelEdgeGroup, EdgeConvexity, FeatureEdge};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::edge_loop_and_face_test_mesh as fixture;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::rigid_motion_meshes::most_faces_on_an_edge;
use crate::section_turn_readings::{
    chain_dihedral_degrees, feature_dihedral_degrees, the_creases_at, the_sharp_turns_of,
};
use crate::surface_test_build::{pinned_to_the_sections, station, try_bevel, TryBevelOptions};

/// The station the face group pick's refusal named.
const THE_MINUS_X_CORNER: Vector3 = Vector3::new(-10.0, -20.0, 0.0);

/// The station the Edge plus TangentChain pair's refusal named - the same corner mirrored.
const THE_PLUS_X_CORNER: Vector3 = Vector3::new(10.0, -20.0, 0.0);

/// THE ROUTING, in the kernel: both of the file's operations are routed to the sections - one rim
/// of twenty edges either way - and both then BUILD, each publishing a mesh, where they once
/// refused at the corner named here.
#[test]
fn both_picks_are_routed_to_the_sections_and_build() {
    for (name, groups, corner) in [
        (
            "the end face",
            vec![fixture::the_end_face()],
            THE_MINUS_X_CORNER,
        ),
        (
            "the edge and the arc",
            vec![fixture::the_straight_edge(), fixture::the_arc()],
            THE_PLUS_X_CORNER,
        ),
    ] {
        let result = {
            let _sections = pinned_to_the_sections();

            let (result, plans) = build(&groups, fixture::SIZE);

            assert_eq!(
                section_routing::last_route().as_deref(),
                Some("sections"),
                "'{name}' is a pick the router accepts, which is why its refusal is a loss"
            );

            assert_eq!(plans.len(), 1, "one rim");
            assert!(plans[0].is_closed, "the rim is a closed loop");
            assert_eq!(
                plans[0].edges.len(),
                20,
                "the diameter and the arc's nineteen chords"
            );

            result
        };

        let reason = result.as_ref().err().cloned().flatten();
        assert!(
            result.is_ok(),
            "'{name}' builds through the sections - {:?} at {}, the corner {corner:?} the absorbed \
             reading now answers",
            reason.as_ref().map(BevelAdvisory::code),
            station(reason.as_ref())
        );
    }
}

/// THE READING of the corner both refusals named. Every number the reading quotes is asserted
/// here.
#[test]
fn the_corner_is_the_diameter_meeting_the_arc_inside_one_flat_face() {
    let part = fixture::the_part();

    let plans = {
        let _sections = pinned_to_the_sections();
        build(&[fixture::the_end_face()], fixture::SIZE).1
    };

    let turns = the_sharp_turns_of(&plans);

    assert_eq!(
        turns.len(),
        2,
        "the rim turns sharply exactly where the diameter meets the arc, at both its ends"
    );

    for (arriving, leaving) in &turns {
        let corner = leaving.from();

        assert!((corner.x.abs() - 10.0).abs() <= 1e-6);
        assert!((corner.y - -20.0).abs() <= 1e-6);
        assert!(corner.z.abs() < 1e-6);

        assert!(
            (section_corner::turn_degrees(arriving, leaving) - 94.7233).abs() <= 0.001,
            "the diameter and the arc's first chord meet at a real corner"
        );

        // THE SHARED SIDE IS THE PICKED FACE, and it is genuinely flat - which is why the corner
        // rule has no quarrel with this corner at all.
        assert!(
            (arriving
                .left_normal()
                .get_normal()
                .dot(leaving.left_normal().get_normal())
                - 1.0)
                .abs()
                <= 1e-9,
            "both edges' left side is the y = -20 end face, one plane"
        );

        assert!(
            (arriving
                .right_normal()
                .get_normal()
                .dot(leaving.right_normal().get_normal())
                - -0.082344)
                .abs()
                <= 1e-5,
            "and the two other sides - the floor and the arc's first facet - fold 94.72 degrees"
        );

        assert!(
            section_corner::looks_like_a_corner(arriving, leaving),
            "so the corner rule ACCEPTS it: one flat shared side, two sides that differ"
        );

        // THE CHORD THE MITRE EATS: shorter than the size, so the corner's reach passes over the
        // station that would stand at its far end.
        let arriving_length = (arriving.to() - arriving.from()).length();
        let leaving_length = (leaving.to() - leaving.from()).length();
        let the_short_way = arriving_length.min(leaving_length);
        let the_long_way = arriving_length.max(leaving_length);

        assert!(
            (the_short_way - 1.6516).abs() <= 1e-3,
            "the arc's first chord"
        );
        assert!((the_long_way - 20.0).abs() <= 1e-6, "the diameter");
        assert!(
            the_short_way < fixture::SIZE,
            "and the chord is SHORTER than the band's own setback, so the corner's reach passes \
             over the station at its far end - 'the mitre eats the arc's first chord'"
        );

        // THE THIRD CREASE: the part's own long edge, running back along y to the far end.
        let creases: Vec<FeatureEdge> = the_creases_at(
            &part,
            leaving.from_vertex(),
            fixture::BEND_THRESHOLD_DEGREES,
        )
        .into_iter()
        .filter(|crease| {
            crease.start_vertex != arriving.from_vertex()
                && crease.end_vertex != arriving.from_vertex()
                && crease.start_vertex != leaving.to_vertex()
                && crease.end_vertex != leaving.to_vertex()
        })
        .collect();

        assert_eq!(
            creases.len(),
            1,
            "exactly one crease leaves the corner that is not the rim"
        );

        assert!(
            (creases[0].direction.get_normal().y.abs() - 1.0).abs() <= 1e-4,
            "running along Y - the part's own long edge, where the floor meets the arc's first \
             facet, and the fold the corner's setback would have to ride"
        );

        assert!((feature_dihedral_degrees(&creases[0]) - 94.7233).abs() <= 0.001);
        assert_eq!(creases[0].convexity, EdgeConvexity::Convex);

        // And the rim's own two edges are square folds, the end face against floor and against
        // facet.
        assert!((chain_dihedral_degrees(arriving) - 90.0).abs() <= 1e-3);
        assert!((chain_dihedral_degrees(leaving) - 90.0).abs() <= 1e-3);
    }
}

/// WHAT THE CORNER'S REACH SWALLOWS, measured off the run the builder actually draws: on the FLAT
/// side - the end face - the station at the far end of the arc's first chord stands BEHIND the
/// corner's offset-meet along the outline, so the region's ring folds; on the WALL side nothing
/// folds at all.
///
/// At (10, -20, 0) the setback is 2 and the arc's first chord is 1.6516. The corner's flat end is
/// the offset-meet (7.8279, -20, 2); the next station's flat boundary is 0.686 BACKWARDS along the
/// chord's own direction, and the station beyond it a positive 0.643 along the SECOND chord: exactly
/// ONE station is swallowed. On the WALL side the corner's end slides 2 mm down the part's own long
/// y edge and the next station's wall boundary stands the whole chord length ahead of it.
#[test]
fn the_corners_reach_swallows_the_station_at_the_chords_far_end() {
    let part = fixture::the_part();

    let plans = {
        let _sections = pinned_to_the_sections();
        build(&[fixture::the_end_face()], fixture::SIZE).1
    };

    let mut pool = SectionPool::new(&part.vertices);

    let run = section_builder_run::try_build_plan(
        Some(Rc::clone(&plans[0])),
        &part,
        &part.get_mesh_edge_graph(),
        &mut pool,
        fixture::SIZE,
        fixture::SEGMENTS,
        None,
    )
    .unwrap_or_else(|failure| {
        panic!("the sections themselves draw - {failure:?} - it is the face rebuild that refuses")
    });

    // The corner the arc LEAVES from, the one whose leaving edge is the short chord.
    let count = run.count() as usize;
    let mut at: i32 = -1;

    for i in 0..count {
        if section_corner::is_sharp_turn(&run.edges[(i + count - 1) % count], &run.edges[i])
            && (run.edges[i].to() - run.edges[i].from()).length() < fixture::SIZE
        {
            at = i as i32;
        }
    }

    assert!(at >= 0, "one corner leaves along the short chord");

    let at = at as usize;
    let corner = &run.sections[at];
    let next = &run.sections[(at + 1) % run.sections.len()];
    let beyond = &run.sections[(at + 2) % run.sections.len()];

    let first_chord = run.edges[at].direction().get_normal();
    let second_chord = run.edges[(at + 1) % count].direction().get_normal();

    assert!(
        (corner.setback - 2.0).abs() <= 1e-5,
        "a square fold at radius 2, read off float normals"
    );
    assert!(((run.edges[at].to() - run.edges[at].from()).length() - 1.6516).abs() <= 1e-3);

    // THE FLAT SIDE FOLDS. Both corner and station read their flat end as an offset in the end
    // face.
    assert_eq!(corner.left_kind, SectionSide::Offset);
    assert_eq!(next.left_kind, SectionSide::Offset);

    let corner_meet = pool.position_of(corner.left_boundary);

    assert!(
        ((pool.position_of(next.left_boundary) - corner_meet).dot(first_chord) - -0.68618).abs()
            <= 1e-4,
        "the swallowed station's boundary stands BEHIND the corner's offset-meet"
    );

    assert!(
        ((pool.position_of(beyond.left_boundary) - corner_meet).dot(second_chord) - 0.64335).abs()
            <= 1e-4,
        "and the station beyond it is the first standing clear of the reach"
    );

    // THE WALL SIDE DOES NOT. Its end slides down the part's own long edge and the band runs on.
    assert_eq!(corner.right_kind, SectionSide::Slide);

    assert!(
        ((pool.position_of(next.right_boundary) - pool.position_of(corner.right_boundary))
            .dot(first_chord)
            - 1.65161)
            .abs()
            <= 1e-4,
        "the wall boundaries stand the whole chord apart, in order"
    );
}

/// LARS'S CLAIM ON THIS FILE, MET: both picks - the end face alone, and the straight edge with the
/// arc - take their 2 mm fillet through the sections, closed, manifold, one shell and smaller than
/// the part they were cut out of, as Surface/legacy already does.
#[test]
fn both_picks_build_through_the_sections() {
    for groups in [
        vec![fixture::the_end_face()],
        vec![fixture::the_straight_edge(), fixture::the_arc()],
    ] {
        let result = {
            let _sections = pinned_to_the_sections();
            build(&groups, fixture::SIZE).0
        };

        let mesh = match result {
            Ok(mesh) => mesh,
            Err(reason) => panic!(
                "{:?} at {}",
                reason.as_ref().map(BevelAdvisory::code),
                station(reason.as_ref())
            ),
        };

        let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
        assert_eq!(open, 0, "open at {open_at:?}");

        assert!(most_faces_on_an_edge(&mesh) <= 2);
        assert_eq!(shell_count(&mesh), 1);

        assert!(
            signed_volume(&mesh) < signed_volume(&fixture::the_part()),
            "a fillet only ever takes material away"
        );
    }
}

/// The picks built the part builder's own way - see `surface_test_build` - as the mesh
/// or the advisory (`Err` with the reason, `None` where nothing was named), with the plans
/// made.
#[allow(clippy::type_complexity)]
fn build(
    groups: &[BevelEdgeGroup],
    size: f64,
) -> (
    Result<Mesh, Option<BevelAdvisory>>,
    Vec<Rc<SurfaceBevelPlan>>,
) {
    let part = fixture::the_part();

    let outcome = try_bevel(
        &part,
        groups,
        size,
        fixture::SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    (
        outcome
            .result
            .map(|built| built.mesh)
            .map_err(|failure| failure.advisory().cloned()),
        outcome.plans_built,
    )
}
