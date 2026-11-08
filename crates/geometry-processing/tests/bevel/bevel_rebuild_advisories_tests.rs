//! What a rebuild says about itself beyond its rows (`BevelRebuildAdvisories`): the repair the
//! working copy needed travels with the rebuild, and a rebuild with nothing to say says nothing.

use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelRebuildAdvisories};

#[test]
fn the_repair_travels_with_the_rebuild() {
    let repaired = BevelAdvisory::new(BevelAdvisoryCode::SourceMeshWasInsideOut, vec![]);
    let advisories = BevelRebuildAdvisories::new(Some(vec![repaired.clone()]));

    assert_eq!(
        advisories.messages()[0],
        repaired,
        "whatever repairing the working copy had to say travels with the rebuild"
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
