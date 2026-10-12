//! Ports `Tests/GeometryProcessingTests/LevelLine/AroundCenterMitreKnifeTests.cs`: "5mm Bevel Around
//! Center" with the mitre on (the app's default bend threshold, 10 degrees) - the tip's mitre runs
//! to a convex 45-degree KNIFE crease, where the band must stop with zero width and the crease
//! stretch become the rail, which is what the Onshape control does. `the_rails_read_in_walk_order`
//! is C#'s measurement, gated on `BEVEL_AROUND_CENTER_MEASURE=1` as there:
//! `BEVEL_AROUND_CENTER_MEASURE=1 cargo nextest run --workspace --test bevel --no-capture
//! around_center_mitre_knife_tests::the_rails_read_in_walk_order`.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::{gap_fill, BandGapRefusal, BandGapResult, ChainField};
use vector_math::Vector3;

use crate::end_face_builds::build_chains;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

const MITRE_DEGREES: f64 = 10.0;

/// The fixture's whole pick at its size: C#'s `chordTolerance: 0, snap: 1e-9, knownRoom: null,
/// seamEpsilon: 0, sharpTurnDegrees: MitreDegrees`.
fn build(fixture: &Rc<LevelLineFixture>) -> BandGapResult {
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    build_chains(
        fixture,
        &all,
        fixture.size,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            seam_epsilon: 0.0,
            sharp_turn_degrees: MITRE_DEGREES,
            ..BandGapBuildOptions::default()
        },
    )
}

/// The control surface's distance from `q` (C#'s `control.Indices.Min(...)`).
fn off_the_control(control: &LevelLineFixture, q: Vector3) -> f64 {
    control
        .indices
        .iter()
        .map(|&(a, b, c)| {
            (ChainField::closest_on_triangle(
                q,
                control.positions[a as usize],
                control.positions[b as usize],
                control.positions[c as usize],
            ) - q)
                .length()
        })
        .fold(f64::INFINITY, f64::min)
}

/// C# `TheMitredTipStopsOnTheKnifeAndTheCreaseIsTheRail`.
#[test]
fn the_mitred_tip_stops_on_the_knife_and_the_crease_is_the_rail() {
    let fixture = Rc::new(LevelLineFixture::load("around-center-r5"));

    let gap = build(&fixture);

    // Before the knife rule the knife was a foreign edge and bounded the room under the size
    // (4.563).
    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "room {} set by {:?}",
        gap.room,
        gap.room_set_by
    );

    // The wall side's loop runs ALONG the crease x = -20, y = -118.5 between the two crossings the
    // control has (z 9.6811 and 10.3189), through a crease vertex at z 10 between them.
    let mut on_the_crease: Vec<f64> = {
        let mesh = gap.mesh.as_ref().expect("a built band").borrow();
        gap.loops
            .iter()
            .flat_map(|l| l.vertices.iter())
            .map(|&v| mesh.position(v))
            .filter(|p| (p.x + 20.0).abs() < 1e-9 && (p.y + 118.5).abs() < 1e-9)
            .map(|p| p.z)
            .collect()
    };
    on_the_crease.sort_by(f64::total_cmp);
    assert_eq!(on_the_crease.len(), 3, "{on_the_crease:?}");
    assert!((on_the_crease[0] - 9.6811).abs() <= 1e-4);
    assert!((on_the_crease[1] - 10.0).abs() <= 1e-4);
    assert!((on_the_crease[2] - 10.3189).abs() <= 1e-4);

    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    assert!(filled.filled(), "fill {:?}", filled.refusal);
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let live = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &live
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &live),
        1
    );
}

/// C# `TheZipDoesNotSpokeAcrossTheInnerRailsJumps`: the zip across a JUMP - the inner rail is four
/// vertices each reading a different leg, and the merge's usual rule spoked the tip's outer
/// vertices to the inner vertex 11.8 of parameter away (0.776 off the control). With the
/// shorter-spoke rule across a jump the worst is 0.272 - a ratchet, not the goal.
#[test]
fn the_zip_does_not_spoke_across_the_inner_rails_jumps() {
    let fixture = Rc::new(LevelLineFixture::load("around-center-r5"));
    let control = LevelLineFixture::load("around-center-control");
    let gap = build(&fixture);
    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let mut seen = HashSet::new();
    let worst = sweep_mesh_measures::kept(&mesh)
        .iter()
        .flat_map(|&(a, b, c)| [a, b, c])
        .filter(|&v| seen.insert(v))
        .map(|v| off_the_control(&control, mesh.position(v)))
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(worst < 0.3, "{worst}");
}

/// C# `TheRailsReadInWalkOrder`: a MEASUREMENT, gated on `BEVEL_AROUND_CENTER_MEASURE=1` - both
/// sides' loops in walk order, each vertex's parameter and the chain point it reads, so a jump in a
/// rail's parameter can be read for what it is.
#[test]
fn the_rails_read_in_walk_order() {
    if std::env::var("BEVEL_AROUND_CENTER_MEASURE").as_deref() != Ok("1") {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load("around-center-r5"));
    let gap = build(&fixture);
    let field = &gap.chains[0];
    let read: Vec<String> = fixture.chains[0]
        .iter()
        .map(|&v| {
            format!(
                "{:.3}",
                field
                    .evaluate(fixture.positions[v as usize])
                    .expect("no budget")
                    .parameter
            )
        })
        .collect();
    println!(
        "chain length {:.3}; chain vertices read {}",
        field.length,
        read.join(", ")
    );
    let band_vertices = {
        let mesh = gap.mesh.as_ref().expect("a built band").borrow();
        for l in &gap.loops {
            println!(
                "side {} closed {} {} vertices",
                l.side,
                l.is_closed,
                l.vertices.len()
            );
            for (i, &v) in l.vertices.iter().enumerate() {
                let q = mesh.position(v);
                let sm = field.evaluate(q).expect("no budget");
                println!(
                    "  {i} v{v} ({:.3}, {:.3}, {:.3}) param {:.3} dist {:.3} nearest ({:.3}, {:.3}, {:.3})",
                    q.x, q.y, q.z, l.parameters[i], sm.distance, sm.nearest.x, sm.nearest.y, sm.nearest.z
                );
            }
        }

        mesh.vertex_count()
    };

    // Each published arc vertex's row: the fill adds (segments - 1) interior vertices per spoke, in
    // spoke order, after the band's own vertices; a row's rail ends are the loop vertices its first
    // and last interior vertices stand beside.
    let control = LevelLineFixture::load("around-center-control");
    let filled =
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let rails: HashSet<i32> = gap
        .loops
        .iter()
        .flat_map(|l| l.vertices.iter().copied())
        .collect();
    let mut neighbours: HashMap<i32, HashSet<i32>> = HashMap::new();
    for (a, b, c) in sweep_mesh_measures::kept(&mesh) {
        for (x, y) in [(a, b), (b, c), (c, a), (b, a), (c, b), (a, c)] {
            neighbours.entry(x).or_default().insert(y);
        }
    }

    let inner = fixture.segments - 1;
    let first = band_vertices;
    let rows = (mesh.vertex_count() - first) / inner;
    let rail_ends = |v: i32| -> String {
        neighbours
            .get(&v)
            .map(|n| {
                n.iter()
                    .filter(|x| rails.contains(x))
                    .map(|x| format!("v{x}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            })
            .unwrap_or_default()
    };
    for row in 0..rows {
        let ids: Vec<i32> = (first + (row * inner)..first + ((row + 1) * inner)).collect();
        let worst = ids
            .iter()
            .map(|&v| off_the_control(&control, mesh.position(v)))
            .fold(f64::NEG_INFINITY, f64::max);
        println!(
            "row {row} v{}..v{} worst {worst:.3}: ends [{}] .. [{}]",
            ids[0],
            ids[ids.len() - 1],
            rail_ends(ids[0]),
            rail_ends(ids[ids.len() - 1])
        );
    }
}
