//! The case pieces of `Tests/GeometryProcessingTests/SurfaceBevelRobustnessTests.cs` that
//! `SectionRobustnessTests` takes by reference (C# widened them to `internal` for exactly this):
//! the twelve-case matrix, the hanger's own splits, the bore's numbers, the case names and the
//! upper-mouth pick. Kept in one place so a case cannot drift between the two tables.
//!
//! Also the class's own test, `ABoreMouthBevelsTheSameWhereverThePartStands`, which reads the
//! LEGACY construction (pinned through `pinned_to_the_legacy_construction`): each case built where
//! it was made and at each rigid motion, every invariant read, and the answers held to the recorded
//! ratchet table of today's construction.

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::CancelToken;
use vector_math::{math_helper, Matrix4X4, Vector3};

use crate::bore_test_meshes::{
    create_plate_with_a_raked_bore, create_plate_with_a_raked_bore_whose_mouth_chords_are_split,
};
use crate::mesh_measurements::shell_count;
use crate::rigid_motion_meshes::{self, TopologySignature};
use crate::section_robustness_tests::how_it_differs_from;
use crate::surface_test_build::{pinned_to_the_legacy_construction, try_bevel, TryBevelOptions};

pub const RADIUS: f64 = 1.5;

pub const THICKNESS: f64 = 4.0;

pub const SEGMENTS: i32 = 8;

pub const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// The eight splits the hanger's own boolean left on its mouth, as
/// `SurfaceRakedBoreSplitChordTests` read them - the chord index and how far along it the extra
/// vertex stands. Reused so the split rows carry the part's own sub-chords and not some tidier
/// invention.
const THE_PARTS_OWN_SPLITS: [(i32, f64); 8] = [
    (9, 0.365755),
    (10, 0.354150),
    (12, 0.331480),
    (13, 0.320984),
    (24, 0.299840),
    (25, 0.307971),
    (29, 0.350692),
    (30, 0.362577),
];

/// The twelve cases: rake degrees, facet count, bevel size, and whether the mouth's chords are
/// split - a sample that moves one axis at a time off the hanger's own row (45 degrees, forty
/// sided, 0.4, split).
pub fn the_matrix() -> Vec<(f64, i32, f64, bool)> {
    vec![
        (0.0, 24, 0.2, false),
        (0.0, 40, 0.4, false),
        (0.0, 40, 0.2, true),
        (0.0, 48, 0.4, true),
        (30.0, 24, 0.4, false),
        (30.0, 24, 0.4, true),
        (30.0, 40, 0.2, true),
        (30.0, 48, 0.4, true),
        (45.0, 24, 0.2, false),
        (45.0, 40, 0.2, false),
        (45.0, 40, 0.4, true),
        (45.0, 48, 0.2, true),
    ]
}

/// The hanger's eight splits, taken to a facet count that may be smaller than the chord indices
/// they were read at: each wraps onto its own chord, and a facet that would take two of them keeps
/// the first, so no chord is split twice.
pub fn splits_within(sides: i32) -> Vec<(i32, f64)> {
    let mut taken: Vec<(i32, f64)> = Vec::new();

    for (chord, at) in THE_PARTS_OWN_SPLITS {
        let wrapped = ((chord % sides) + sides) % sides;

        if !taken.iter().any(|already| already.0 == wrapped) {
            taken.push((wrapped, at));
        }
    }

    taken
}

/// C# `NameOf`: `rake{rake:0} sides{sides} size{size:0.0} split|whole`.
pub fn name_of(rake: f64, sides: i32, size: f64, split: bool) -> String {
    format!(
        "rake{rake:.0} sides{sides} size{size:.1} {}",
        if split { "split" } else { "whole" }
    )
}

/// The first edge of the upper mouth, found by the bore's own geometry, with each end brought
/// back through `back_to_where_it_was_made` first, so a part stood somewhere else is picked by the
/// SAME geometric question and not by an index that happens to have survived.
pub fn the_upper_mouth(
    features: &FeatureEdgeSet,
    back_to_where_it_was_made: Matrix4X4,
    rake_degrees: f64,
) -> i32 {
    for i in 0..features.count() {
        let edge = &features.edges()[i];

        if on_the_mouth(
            edge.start.transform_position(back_to_where_it_was_made),
            rake_degrees,
        ) && on_the_mouth(
            edge.end.transform_position(back_to_where_it_was_made),
            rake_degrees,
        ) {
            return i as i32;
        }
    }

    panic!("the fixture has no upper mouth");
}

/// On the upper mouth: on the top face, and a bore radius from the drill's own axis.
fn on_the_mouth(at: Vector3, rake_degrees: f64) -> bool {
    let rake = math_helper::degrees_to_radians(rake_degrees);
    let axis = Vector3::new(rake.sin(), 0.0, rake.cos());

    // A thousandth rather than the bore fixtures' ten-thousandth: a part stood 250 mm out and
    // turned is held on a float lattice of about 2e-5, so the question has to be asked a little
    // more loosely than at the origin. Still two orders inside the nearest thing it could confuse
    // the mouth with - the far face 4 mm away and the rim 6 mm out.
    (at.z - (THICKNESS / 2.0)).abs() < 1e-3
        && ((at - (axis * at.dot(axis))).length() - RADIUS).abs() < 1e-3
}

/// WHAT TODAY'S LEGACY CONSTRUCTION FAILS: the key is "`case | motion`", the value every invariant
/// the row breaks. A row not listed passes every invariant.
const THE_FAILURES_OF_TODAYS_CONSTRUCTION: &[(&str, &str)] = &[
    ("rake0 sides24 size0.2 whole | far", "moved valences"),
    (
        "rake0 sides24 size0.2 whole | rotated and moved",
        "moved valences",
    ),
    ("rake0 sides24 size0.2 whole | rotated", "moved valences"),
    ("rake0 sides24 size0.2 whole | stand-off", "moved valences"),
    (
        "rake0 sides40 size0.2 split | far",
        "refused SurfaceStripSeamDidNotMeet; moved v0/1352 f0/2704 valences",
    ),
    (
        "rake0 sides40 size0.2 split | rotated and moved",
        "refused SurfaceHullEndOnThreePlanes; moved v0/1352 f0/2704 valences",
    ),
    ("rake0 sides40 size0.2 split | rotated", "moved valences"),
    (
        "rake0 sides40 size0.2 split | stand-off",
        "moved v1356/1352 f2712/2704 valences",
    ),
    (
        "rake0 sides40 size0.4 whole | far",
        "moved v1153/1120 f2306/2240 valences",
    ),
    (
        "rake0 sides40 size0.4 whole | rotated and moved",
        "refused SurfaceHullEndCrossesTheCorner; moved v0/1120 f0/2240 valences",
    ),
    ("rake0 sides40 size0.4 whole | rotated", "moved valences"),
    ("rake0 sides40 size0.4 whole | stand-off", "moved valences"),
    (
        "rake0 sides48 size0.4 split | far",
        "refused SurfaceStripSeamDidNotMeet; moved v0/1576 f0/3152 valences",
    ),
    (
        "rake0 sides48 size0.4 split | rotated and moved",
        "refused SurfaceHullEndCrossesTheCorner; moved v0/1576 f0/3152 valences",
    ),
    (
        "rake0 sides48 size0.4 split | rotated",
        "refused SurfaceStripSeamDidNotMeet; moved v0/1576 f0/3152 valences",
    ),
    (
        "rake0 sides48 size0.4 split | stand-off",
        "moved v1585/1576 f3170/3152 valences",
    ),
    ("rake30 sides24 size0.4 split | far", "moved valences"),
    (
        "rake30 sides24 size0.4 split | rotated and moved",
        "moved valences",
    ),
    ("rake30 sides24 size0.4 split | rotated", "moved valences"),
    ("rake30 sides24 size0.4 split | stand-off", "moved valences"),
    ("rake30 sides24 size0.4 whole | far", "moved valences"),
    (
        "rake30 sides24 size0.4 whole | rotated and moved",
        "moved valences",
    ),
    ("rake30 sides24 size0.4 whole | rotated", "moved valences"),
    ("rake30 sides24 size0.4 whole | stand-off", "moved valences"),
    (
        "rake30 sides40 size0.2 split | far",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2246 f0/4492 valences",
    ),
    (
        "rake30 sides40 size0.2 split | rotated and moved",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2246 f0/4492 valences",
    ),
    (
        "rake30 sides40 size0.2 split | rotated",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2246 f0/4492 valences",
    ),
    (
        "rake30 sides40 size0.2 split | stand-off",
        "open 5; fold 3; moved v2282/2246 f4563/4492 open5/0 valences",
    ),
    (
        "rake30 sides48 size0.4 split | far",
        "open 5; fold 3; moved v2619/2639 f5237/5278 open5/0 valences",
    ),
    (
        "rake30 sides48 size0.4 split | rotated and moved",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2639 f0/5278 valences",
    ),
    (
        "rake30 sides48 size0.4 split | rotated",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2639 f0/5278 valences",
    ),
    (
        "rake30 sides48 size0.4 split | stand-off",
        "open 10; fold 3; moved v2670/2639 f5338/5278 open10/0 valences",
    ),
    ("rake45 sides24 size0.2 whole | far", "moved valences"),
    (
        "rake45 sides24 size0.2 whole | rotated and moved",
        "moved valences",
    ),
    ("rake45 sides24 size0.2 whole | rotated", "moved valences"),
    ("rake45 sides40 size0.2 whole | far", "moved valences"),
    (
        "rake45 sides40 size0.2 whole | rotated and moved",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2066 f0/4132 valences",
    ),
    (
        "rake45 sides40 size0.2 whole | rotated",
        "refused SurfaceHullEndCrossesTheCorner; moved v0/2066 f0/4132 valences",
    ),
    (
        "rake45 sides40 size0.2 whole | stand-off",
        "open 5; moved v2088/2066 f4175/4132 open5/0 valences",
    ),
    (
        "rake45 sides40 size0.4 split | far",
        "moved v2211/2265 f4422/4530 valences",
    ),
    (
        "rake45 sides40 size0.4 split | rotated and moved",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2265 f0/4530 valences",
    ),
    (
        "rake45 sides40 size0.4 split | rotated",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2265 f0/4530 valences",
    ),
    (
        "rake45 sides40 size0.4 split | stand-off",
        "open 5; fold 3; moved v2287/2265 f4573/4530 open5/0 valences",
    ),
    (
        "rake45 sides48 size0.2 split | far",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2783 f0/5562 open0/16 valences",
    ),
    ("rake45 sides48 size0.2 split | identity", "open 16; fold 3"),
    (
        "rake45 sides48 size0.2 split | rotated and moved",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2783 f0/5562 open0/16 valences",
    ),
    (
        "rake45 sides48 size0.2 split | rotated",
        "refused SurfaceStripSeamDidNotMeet; moved v0/2783 f0/5562 open0/16 valences",
    ),
    (
        "rake45 sides48 size0.2 split | stand-off",
        "open 10; fold 3; moved v2793/2783 f5584/5562 open10/16 valences",
    ),
];

/// Each case, built where it was made and at each of `rigid_motion_meshes::the_motions`, with
/// every invariant read on each and the answers held to the recorded table. C#'s
/// `[MethodDataSource]` rows are a loop; every motion of every case is read before anything is
/// asserted, so one row's disagreement does not hide the ones behind it.
#[test]
fn a_bore_mouth_bevels_the_same_wherever_the_part_stands() {
    let _pinned = pinned_to_the_legacy_construction();
    let mut disagreements = Vec::new();

    for (rake, sides, size, split) in the_matrix() {
        let case_name = name_of(rake, sides, size, split);
        let mut at_the_origin: Option<TopologySignature> = None;

        for (motion_name, motion) in rigid_motion_meshes::the_motions() {
            let (mut broken, signature) = what_is_broken_by(rake, sides, size, split, motion);

            if motion_name == "identity" {
                at_the_origin = Some(signature.clone());
            } else if let Some(origin) = &at_the_origin {
                if signature != *origin {
                    // The harness's own invariant, read last so a row that is already broken is
                    // reported by what it broke rather than by the difference that follows from it.
                    broken.push(how_it_differs_from(&signature, origin));
                }
            }

            let key = format!("{case_name} | {motion_name}");
            let reading = broken.join("; ");
            let recorded = THE_FAILURES_OF_TODAYS_CONSTRUCTION
                .iter()
                .find(|(k, _)| *k == key)
                .map(|(_, v)| *v);

            // How the table above was written, and how it is rewritten whole when the construction
            // changes: run with BEVEL_ROBUSTNESS_BASELINE set to a file and every row's reading
            // lands in it table-ready.
            if let Ok(dump) = std::env::var("BEVEL_ROBUSTNESS_BASELINE") {
                use std::io::Write;

                if let Ok(mut file) = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(dump)
                {
                    let _ = writeln!(file, "    (\"{key}\", \"{reading}\"),");
                }
            }

            if reading != recorded.unwrap_or("") {
                disagreements.push(format!(
                    "{key}: reads \"{reading}\", table records \"{}\"\n    (\"{key}\", \"{reading}\"),",
                    recorded.unwrap_or("(nothing - it passes)")
                ));
            }
        }
    }

    assert_eq!(
        disagreements.join("\n"),
        "",
        "UPDATE THE RATCHET - what this case does and what is recorded for it disagree"
    );
}

/// C# `WhatIsBrokenBy`: builds one case stood by one motion through the legacy construction and
/// reads every invariant of the answer, with the answer's topology signature.
fn what_is_broken_by(
    rake: f64,
    sides: i32,
    size: f64,
    split: bool,
    motion: Matrix4X4,
) -> (Vec<String>, TopologySignature) {
    let mut broken = Vec::new();

    let mut part = if split {
        create_plate_with_a_raked_bore_whose_mouth_chords_are_split(
            sides,
            RADIUS,
            THICKNESS,
            rake,
            &splits_within(sides),
            6.0,
            Vector3::ZERO,
        )
    } else {
        create_plate_with_a_raked_bore(sides, RADIUS, THICKNESS, rake, 6.0)
    };

    rigid_motion_meshes::move_mesh(&mut part, motion);

    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_upper_mouth(&features, motion.inverted(), rake),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the upper mouth is one of the part's feature edges");

    let outcome = try_bevel(
        &part,
        &[group],
        size,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let mesh = match outcome.result {
        Ok(built) => built.mesh,
        Err(failure) => {
            broken.push(format!(
                "refused {}",
                failure
                    .advisory()
                    .map_or("with no code".to_string(), |reason| format!(
                        "{:?}",
                        reason.code()
                    ))
            ));

            return (broken, rigid_motion_meshes::signature_of(None));
        }
    };

    let signature = rigid_motion_meshes::signature_of(Some(&mesh));

    let (open, _) = surface_mesh_closure::open_edge_count(&mesh);
    if open != 0 {
        broken.push(format!("open {open}"));
    }

    let most = rigid_motion_meshes::most_faces_on_an_edge(&mesh);
    if most > 2 {
        broken.push(format!("fold {most}"));
    }

    let shells = shell_count(&mesh);
    if shells != 1 {
        broken.push(format!("shells {shells}"));
    }

    let needles = rigid_motion_meshes::two_cornered_triangle_count(&mesh);
    if needles != 0 {
        broken.push(format!("two-cornered {needles}"));
    }

    (broken, signature)
}
