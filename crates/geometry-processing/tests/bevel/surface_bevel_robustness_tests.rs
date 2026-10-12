//! The case pieces of `Tests/GeometryProcessingTests/SurfaceBevelRobustnessTests.cs` that
//! `SectionRobustnessTests` takes by reference (C# widened them to `internal` for exactly this):
//! the twelve-case matrix, the hanger's own splits, the bore's numbers, the case names and the
//! upper-mouth pick. Kept in one place so a case cannot drift between the two tables.
//!
//! The class's own test, `ABoreMouthBevelsTheSameWhereverThePartStands`, reads the LEGACY
//! construction through `SurfaceTestBuild.PinnedToTheLegacyConstruction`, and lands here with
//! `SurfaceTestBuild` (S28).

use geometry_processing::topology::FeatureEdgeSet;
use vector_math::{math_helper, Matrix4X4, Vector3};

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
