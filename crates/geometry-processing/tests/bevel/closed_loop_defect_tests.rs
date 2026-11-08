//! The closed-loop picks the SWEEP found the level line taking and failing, carried into the kernel
//! as the app route's own arguments. A closed chain either builds a closed solid with a real volume
//! change, or refuses BY SIZE with a room that is true - and nothing else. Each test's cases run as
//! a loop inside it.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{gap_fill, BandGapRefusal, GapFillRefusal};

use crate::end_face_builds::feature_of;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{open_edges, signed_volume, signed_volume_of};

/// The rim of the top face of "Simple Internal Fillet", at
/// r1 and r2. The sweep's rows read "the fill refused, SideDoesNotLeaveOneLoop" and "the fill
/// refused, OpenChainEnds".
#[test]
fn a_closed_rim_builds_or_refuses_by_size() {
    for name in [
        "internal-fillet-face8-r1",
        "internal-fillet-face8-r2",
        "internal-fillet-face9-r1",
        "internal-fillet-face9-r2",
    ] {
        a_closed_rim_builds_or_refuses_by_size_row(name);
    }
}

fn a_closed_rim_builds_or_refuses_by_size_row(name: &str) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let profile = profile_of(&fixture);
    let build = |size: f64| {
        band_gap_entry_points::build(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains[0],
            true,
            Some(feature_of(&fixture)),
            size,
            profile,
            &BandGapBuildOptions::default(),
        )
        .expect("the pick builds or refuses by name")
    };

    let before = signed_volume_of(&fixture.positions, &fixture.indices);

    let gap = build(fixture.size);

    if gap.refusal == BandGapRefusal::Size {
        // An honest size refusal is an answer, but only if the room it names is true: just under
        // it the very same pick must build.
        assert!(gap.room > 0.0, "{name}");
        assert!(gap.room < fixture.size, "{name}");

        let under = build(gap.room * 0.9);

        assert_eq!(under.refusal, BandGapRefusal::None, "{name}");

        let fitted_under =
            gap_fill::fill(&under, profile, fixture.segments).expect("the fill runs");

        assert_eq!(fitted_under.refusal, GapFillRefusal::None, "{name}");
        let mesh = fitted_under.mesh.as_ref().expect("a filled mesh").borrow();
        assert!(open_edges(&mesh).is_empty(), "{name}");

        return;
    }

    assert_eq!(gap.refusal, BandGapRefusal::None, "{name}");

    let filled = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");

    assert_eq!(filled.refusal, GapFillRefusal::None, "{name}");
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    assert!(open_edges(&mesh).is_empty(), "{name}");

    let after = signed_volume(&mesh);

    // A fillet or chamfer of a convex rim takes material away; of a concave one it adds. Either
    // way the change is real and bounded by the band the size could have reached.
    assert!((after - before).abs() > 0.0, "{name}");
}
