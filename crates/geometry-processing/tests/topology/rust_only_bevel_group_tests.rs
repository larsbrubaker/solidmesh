//! Rust-only checks of the gp2 records: the saved JSON shape of `BevelEdgeGroup`, the advisory
//! equality rules, and the copy-not-mutate contract of `bevel_group_edits`.

use std::sync::Arc;

use geometry_processing::bevel::{bevel_advisory_codes, BevelAdvisory, BevelAdvisoryCode};
use geometry_processing::topology::{bevel_group_edits, BevelEdgeGroup, BevelSelectionKind};
use vector_math::Vector3;

#[test]
fn rust_only_edge_group_json_matches_newtonsoft_shape() {
    let group = BevelEdgeGroup {
        selection_kind: BevelSelectionKind::TangentChain,
        anchor_midpoint: Vector3::new(1.0, 2.0, 3.0),
        ..BevelEdgeGroup::default()
    };

    let json = serde_json::to_string(&group).unwrap();
    assert!(json.starts_with(r#"{"SelectionKind":"TangentChain","AnchorMidpoint":{"X":1.0,"Y":2.0,"Z":3.0},"AnchorDirection":"#));
    assert!(!json.contains("SourceItemID"), "null is ignored: {json}");
    assert!(
        json.contains(
            r#""AnchorPlaneDistance1":0.0,"CurveToleranceDegrees":30.0,"FlatAngleDegrees":"#
        ),
        "{json}"
    );

    let named = BevelEdgeGroup {
        source_item_id: Some("abc".to_string()),
        ..group.clone()
    };
    let json = serde_json::to_string(&named).unwrap();
    assert!(
        json.contains(r#","SourceItemID":"abc","CurveToleranceDegrees":"#),
        "{json}"
    );

    let back: BevelEdgeGroup = serde_json::from_str(&json).unwrap();
    assert_eq!(back, named);
}

#[test]
fn rust_only_edge_group_json_defaults_missing_members_and_ignores_old_sizes() {
    let back: BevelEdgeGroup =
        serde_json::from_str(r#"{"SelectionKind":"FaceGroup","Size":2.0,"Segments":4}"#).unwrap();

    assert_eq!(back.selection_kind, BevelSelectionKind::FaceGroup);
    assert_eq!(back.curve_tolerance_degrees, 30.0);
    assert_eq!(back.source_item_id, None);
}

#[test]
fn rust_only_advisory_equality_ignores_mesh_edge_and_tells_int_from_double() {
    let a = BevelAdvisory::new(BevelAdvisoryCode::FaceRegionRimIsPartial, vec![3.into()]);
    assert_eq!(a, a.on_mesh_edge(7));
    assert_eq!(a.on_mesh_edge(7).mesh_edge_index(), 7);
    assert_ne!(
        a,
        BevelAdvisory::new(BevelAdvisoryCode::FaceRegionRimIsPartial, vec![3.0.into()])
    );
    assert_eq!(a.to_string(), "FaceRegionRimIsPartial(3)");

    let fits = BevelAdvisory::new(
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        vec![2.0.into(), Vector3::default().into(), 1.5.into()],
    );
    assert_eq!(
        bevel_advisory_codes::try_the_most_that_fits(Some(&fits)),
        Some(1.5)
    );
    assert!(bevel_advisory_codes::is_a_note_rather_than_a_complaint(
        BevelAdvisoryCode::BuiltWithTheOtherConstruction
    ));
}

#[test]
fn rust_only_group_edits_copy_rather_than_mutate() {
    let first = Arc::new(BevelEdgeGroup::default());
    let second = Arc::new(BevelEdgeGroup::default());
    let groups = vec![Arc::clone(&first), Arc::clone(&second)];

    // equal by value but not the same group: the slot does not hold it
    assert!(bevel_group_edits::without(&groups, 0, &second).is_none());

    let removed = bevel_group_edits::without(&groups, 0, &first).unwrap();
    assert_eq!(removed.len(), 1);
    assert!(Arc::ptr_eq(&removed[0], &second));
    assert_eq!(groups.len(), 2);

    let replacement = Arc::new(first.read_at(10.0));
    let replaced =
        bevel_group_edits::replacing(&groups, 1, &second, Arc::clone(&replacement)).unwrap();
    assert!(Arc::ptr_eq(&replaced[1], &replacement));
    assert!(Arc::ptr_eq(&groups[1], &second));
}
