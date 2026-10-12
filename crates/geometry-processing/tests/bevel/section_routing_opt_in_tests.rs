//! Port of `SectionRoutingOptInTests.cs`: the developer opt-in that seeds the sections switch
//! from `MATTERCAD_BEVEL_SECTIONS` - only "1" and "true" in any casing turn it on. The reading is
//! tested and not the switch, because the switch is seeded once when first touched and the process
//! environment is shared by every test running beside these. C# `[Arguments]` rows are loops.

use geometry_processing::bevel::sections::section_routing::reads_as_on;

/// C# `ValuesThatTurnTheSectionsOn`.
#[test]
fn values_that_turn_the_sections_on() {
    for value in ["1", "true", "TRUE"] {
        assert!(reads_as_on(Some(value)), "{value}");
    }
}

/// C# `EverythingElseLeavesThemOff` (the `null` row is `None`).
#[test]
fn everything_else_leaves_them_off() {
    for value in [None, Some(""), Some("0"), Some("no")] {
        assert!(!reads_as_on(value), "{value:?}");
    }
}
