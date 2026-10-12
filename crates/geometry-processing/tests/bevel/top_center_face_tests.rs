//! Ports `Tests/GeometryProcessingTests/LevelLine/TopCenterFaceTests.cs`: "Top Center Face.mcx"'s
//! top face (f6 in the app sweep), three facets at z = 20 whose rim turns in at a reflex corner
//! (-10, -108.5, 20). Filleted at 0.1 the level line once left a vertex 0.88 r off the source
//! surface, out of reach of any fillet band. The class's `ToTriangle` lives in
//! `end_face_measures.rs`, which the other classes reading it share.

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{gap_fill, GapFillRefusal};

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::end_face_measures::to_triangle;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures::{is_closed_and_consistent, kept, shell_count};

/// C# `TheTopFaceFilletStaysOnTheBand`.
#[test]
fn the_top_face_fillet_stays_on_the_band() {
    let fixture = Rc::new(LevelLineFixture::load("top-center-face-f6-r01"));
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(
        &fixture,
        &all,
        fixture.size,
        &BandGapBuildOptions {
            budget: Some(default_budget(&fixture, false)),
            ..as_saved(&fixture, fixture.known_room)
        },
    );
    assert!(gap.built(), "band {:?}", gap.refusal);

    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments)
        .expect("the fill fills or refuses by name");
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = kept(&mesh);
    assert!(is_closed_and_consistent(&mesh.vertices, &kept));
    assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);

    // Every chain edge of this pick is a right angle, so along an edge a fillet arc stands at most
    // r(1 - cos 45) = 0.29 r off the faces it joins; where three faces meet a blend stands up to
    // r(1 - 1/sqrt 3) = 0.42 r off. Half r is past every blend this pick has; the defect stood
    // 0.88 r off.
    let reach = fixture.size * 0.5;
    let used: HashSet<i32> = kept.iter().flat_map(|t| [t.0, t.1, t.2]).collect();
    let mut worst = 0.0;
    let mut worst_vertex = -1;
    for &v in &used {
        let p = mesh.vertices[v as usize];
        let off = fixture
            .indices
            .iter()
            .map(|t| {
                to_triangle(
                    p,
                    fixture.positions[t.0 as usize],
                    fixture.positions[t.1 as usize],
                    fixture.positions[t.2 as usize],
                )
            })
            .fold(f64::INFINITY, f64::min);
        if off > worst {
            worst = off;
            worst_vertex = v;
        }
    }

    if worst_vertex >= 0 {
        println!(
            "worst vertex {worst_vertex} at {:?} stands {worst:.4} off the source",
            mesh.vertices[worst_vertex as usize]
        );
        for t in kept
            .iter()
            .filter(|t| t.0 == worst_vertex || t.1 == worst_vertex || t.2 == worst_vertex)
        {
            println!(
                "  on {} {:?} | {} {:?} | {} {:?}",
                t.0,
                mesh.vertices[t.0 as usize],
                t.1,
                mesh.vertices[t.1 as usize],
                t.2,
                mesh.vertices[t.2 as usize]
            );
        }
    }

    assert!(worst <= reach);
}
