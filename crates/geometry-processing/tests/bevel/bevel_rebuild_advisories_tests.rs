//! Ports `GeometryProcessingTests/BevelRebuildAdvisoriesTests.cs`: the once-per-operation rule
//! `BevelRebuildAdvisories` exists to keep. What is at risk is the COUNT: each of these is raised
//! from inside a loop over chains, so without the flag eight chains tripping one fallback would put
//! eight copies of one sentence in the panel.

use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelRebuildAdvisories};

#[test]
fn each_advisory_is_said_once_however_many_chains_trip_it() {
    let repaired = BevelAdvisory::new(BevelAdvisoryCode::SourceMeshWasInsideOut, vec![]);
    let mut advisories = BevelRebuildAdvisories::new(Some(vec![repaired.clone()]));

    advisories.corner_went_unrounded();
    advisories.corner_went_unrounded();
    advisories.bead_went_on_unclipped();
    advisories.corner_went_unrounded();
    advisories.shadow_clip_was_refused();
    advisories.shadow_clip_was_refused();

    assert_eq!(
        advisories.messages().len(),
        4,
        "the repair's own line, plus one line each for the three fallbacks however many times \
         they were hit"
    );

    assert_eq!(
        advisories.messages()[0],
        repaired,
        "whatever repairing the working copy had to say travels with the rebuild"
    );

    assert_eq!(
        advisories.messages()[1].code(),
        BevelAdvisoryCode::CornerWentUnrounded,
        "the user has to be told a corner may still carry a ridge"
    );

    assert_eq!(
        advisories.messages()[3].code(),
        BevelAdvisoryCode::ShadowClipWasRefused,
        "a cutter whose shadow clip the kernel would not build goes on to cut through whatever \
         was standing on the face, which has to be said out loud"
    );
}

#[test]
fn nothing_to_say_is_an_empty_list() {
    let advisories = BevelRebuildAdvisories::new(None::<Vec<BevelAdvisory>>);

    assert!(
        advisories.messages().is_empty(),
        "a rebuild that needed no repair and hit no fallback - which is nearly all of them - must \
         not put a panel in front of the user"
    );
}
