//! Port of `LevelLine/RoundsFirstGrowOverEndTests.cs`: Center Right and Left Side built rounds first
//! (`BandGap.BuildRoundsFirst`): pass B's joined concave crease 5 - ... - 8 ends at v5 (-10, -108.5,
//! 6.667) and v8 (-10, -108.5, 13.333) on the notch wall x - y = 98.5, a side of the picked rim. A
//! concave bead's end is grown over, not cut: it closes against the wall with the round's section
//! lying in the wall plane, so the crease's end vertex is not left in the part.

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_rounds_first::build_rounds_first;
use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::{BandProfile, GapFillRefusal};
use vector_math::Vector3;

use crate::level_line_fixture::LevelLineFixture;

const NAME: &str = "center-right-and-left-side-op0";

/// The fixture came from float storage: its plane offsets carry ~1e-6 noise at 100 mm.
const FLOAT_NOISE: f64 = 1e-5;

fn off_the_wall(p: Vector3) -> f64 {
    (p.x - p.y - 98.5) / 2f64.sqrt()
}

/// Neither crease end vertex is left in the built part, and at each the bead's end is the round's
/// section in the wall: its interior points are added vertices on the wall plane within the
/// bevel's reach of the end.
#[test]
fn the_bead_grows_over_both_end_vertices() {
    let fixture = Rc::new(LevelLineFixture::load(NAME));
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let feature = {
        let fixture = Rc::clone(&fixture);
        Rc::new(move |a, b| fixture.is_feature_edge(a, b))
    };
    let built = build_rounds_first(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(feature),
        fixture.size,
        profile,
        fixture.segments,
        None,
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        None,
        Some(&fixture.closed),
    )
    .expect("the build runs");
    assert_eq!(
        built.refusal,
        GapFillRefusal::None,
        "site {}",
        ends_site().unwrap_or_default()
    );
    let mesh = built.mesh.as_ref().expect("the built mesh").borrow();
    let at = mesh.vertices();
    let kept: Vec<_> = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| mesh.triangles()[t as usize])
        .collect();
    let used: HashSet<i32> = kept.iter().flat_map(|t| [t.a, t.b, t.c]).collect();
    for end in [5, 8] {
        let mut neighbours: Vec<i32> = Vec::new();
        for v in kept
            .iter()
            .filter(|t| t.a == end || t.b == end || t.c == end)
            .flat_map(|t| [t.a, t.b, t.c])
        {
            if v != end && !neighbours.contains(&v) {
                neighbours.push(v);
            }
        }
        let named: Vec<String> = neighbours
            .iter()
            .map(|&v| {
                let p = at[v as usize];
                format!(
                    "{v} ({:.3},{:.3},{:.3}) wall {:.3e}",
                    p.x,
                    p.y,
                    p.z,
                    off_the_wall(p)
                )
            })
            .collect();
        assert!(
            !used.contains(&end),
            "v{end} still joins {}",
            named.join("; ")
        );

        let section = used
            .iter()
            .filter(|&&v| {
                v >= fixture.positions.len() as i32
                    && off_the_wall(at[v as usize]).abs() <= FLOAT_NOISE
                    && (at[v as usize] - fixture.positions[end as usize]).length()
                        <= 2.0 * fixture.size
            })
            .count() as i32;
        assert!(
            section >= fixture.segments - 1,
            "at v{end}: {section} added vertices on the wall"
        );
    }
}
