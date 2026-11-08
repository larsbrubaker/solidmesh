//! Rust-only checks of the tenth bevel slice: a record whose initial values are not its field
//! types' defaults (the level line's `PickFault`).

// ---------------------------------------------------------------- Bevel core round 0 records

#[test]
fn rust_only_a_pick_fault_defaults_to_naming_nothing() {
    use geometry_processing::bevel::level_line::{BandGapRefusal, PickFault};

    // Its initial values: every index -1 (none yet), the refusal its first member.
    let fault = PickFault::default();
    assert_eq!(fault.refusal, BandGapRefusal::default());
    assert_eq!(
        (fault.chain, fault.other, fault.vertex, fault.edge),
        (-1, -1, -1, (-1, -1))
    );
}
