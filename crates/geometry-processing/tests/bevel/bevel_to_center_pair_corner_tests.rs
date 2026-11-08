//! "Bevel to Center Feature" as the app hands it to the kernel - five groups at the saved 3.4
//! fillet, which the route joins into FOUR open chains. The right tip's vertical edge and the top
//! edge along its side meet end to end and are one chain, 10 - 12 - 19, turning at the pair corner
//! v12 (0, -118.5, 20); the band splits it there into two members, the tip (member 0) and the top
//! edge (member 1). This fixture's frame is the app's shifted by (-20, -118.5, 0).
//!
//! The pair corner's end-face field (`PairCornerEndFaceField`, on by default) makes the top face's
//! group read the least of its pick field and the tip's round carried through the top, so the top
//! face is cut along the arc about the tip's axis; the step-corner carry alone keeps the top face
//! within the tip's round as well. Turning the pair-corner end-face field off rides on the
//! build's budget, so it reaches no other build. Each test's cases run as a loop inside it.

use std::collections::HashSet;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use solidmesh::bevel::level_line::{
    chain_on_the_mesh, gap_fill, triangle_edges, BandGapRefusal, BandGapResult, ChainDistanceField,
    GapFillRefusal, LevelLineWork, PairCornerEndFaceField,
};
use solidmesh_math::seeded_random::SeededRandom;
use solidmesh_math::{Vector2, Vector3};

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

const FIXTURE: &str = "bevel-to-center-feature-r34";

/// The right tip plus its top edge: v10 up the tip to the pair corner v12, then back along x = 0.
const RIGHT_TIP_CHAIN: usize = 1;

/// Every level-line vertex the cut leaves on the
/// TOP face in front of the tip's round lies on that round - the tip's fillet axis stands at
/// (-3.4, -110.292) in plan, so nothing of the top face may stand more than 3.4 from it ahead of
/// y = -110.292. The control is 15.192 deep; without the cut the rebuild's 16.6 is exactly the
/// vertex this finds (the top edge's rail turning on the unselected slope top edge v12 - v18 at
/// (-3.4, -115.1, 20), 4.808 from the tip's axis).
///
/// v12 is a step corner of the one picked chain, so StepCornerCarry joins the tip to the top
/// edge's side group and its field cuts the top face round the corner as well: with the end-face
/// field turned off the spike no longer comes back, so the turned-off reading checks the carry.
#[test]
fn the_top_face_stops_at_the_tips_round_at_the_pair_corner() {
    let fixture = Rc::new(LevelLineFixture::load(FIXTURE));
    assert_eq!(fixture.chains[RIGHT_TIP_CHAIN], vec![10, 12, 19]);

    let carried_only = farthest_ahead(&fixture, true);
    assert!(
        carried_only <= fixture.size + 1e-3,
        "the step-corner carry alone keeps a vertex {carried_only:.3} from the tip's axis"
    );

    let ahead = farthest_ahead(&fixture, false);
    assert!(
        ahead <= fixture.size + 1e-3,
        "the top face keeps a vertex {ahead:.3} from the tip's axis"
    );
}

/// The right tip's chain alone, open, at `size` - with the pair-corner end-face field turned off
/// for this build when `field_off`.
fn build_right_tip(fixture: &Rc<LevelLineFixture>, size: f64, field_off: bool) -> BandGapResult {
    let budget = default_budget(fixture, false);
    let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
    let _off = field_off.then(PairCornerEndFaceField::turn_off);
    let options = BandGapBuildOptions {
        budget: Some(Rc::clone(&budget)),
        ..as_saved(fixture, None)
    };
    // The chain is built open, as it is in the fixture.
    assert!(!fixture.closed[RIGHT_TIP_CHAIN]);
    build_chains(fixture, &[RIGHT_TIP_CHAIN], size, &options)
}

/// How far from the tip's axis the farthest level-line vertex the build leaves
/// on the top face ahead of it stands.
fn farthest_ahead(fixture: &Rc<LevelLineFixture>, field_off: bool) -> f64 {
    let gap = build_right_tip(fixture, fixture.size, field_off);
    let mesh = gap.mesh.as_ref().unwrap_or_else(|| {
        panic!(
            "{:?}, room {} set by {:?}",
            gap.refusal, gap.room, gap.room_set_by
        )
    });
    let mesh = mesh.borrow();

    let axis = tip_axis(fixture, fixture.size);
    gap.loops
        .iter()
        .flat_map(|l| l.vertices.iter())
        .map(|&v| mesh.vertices()[v as usize])
        .filter(|p| (p.z - 20.0).abs() < 1e-6 && p.y < axis.y)
        .map(|p| (Vector2::new(p.x, p.y) - axis).length())
        .fold(f64::NEG_INFINITY, f64::max)
}

/// The same spike in a BUILT part, at a size
/// under the pinch the saved 3.4 is refused by - the right tip's chain alone fills at 2.5, and its
/// top face must end on the tip's round there too. Without the end-face cut and the step-corner
/// carry it kept (-2.5, -116, 20), 3.536 (size * sqrt 2) from the tip's axis where the round is 2.5.
#[test]
fn under_the_pinch_the_filled_top_face_stops_at_the_tips_round() {
    const SIZE: f64 = 2.5;
    let fixture = Rc::new(LevelLineFixture::load(FIXTURE));

    let carried_only = filled_farthest_ahead(&fixture, SIZE, true);
    assert!(
        carried_only <= SIZE + 1e-3,
        "the step-corner carry alone keeps a vertex {carried_only:.3} from the tip's axis"
    );

    let worst = filled_farthest_ahead(&fixture, SIZE, false);
    assert!(
        worst <= SIZE + 1e-3,
        "the filled top face keeps a vertex {worst:.3} from the tip's axis"
    );
}

/// The filled right tip's farthest top-face vertex ahead of its axis,
/// from that axis, after checking the fill is one good shell.
fn filled_farthest_ahead(fixture: &Rc<LevelLineFixture>, size: f64, field_off: bool) -> f64 {
    let gap = build_right_tip(fixture, size, field_off);
    assert!(
        gap.built(),
        "{:?}, room {} set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );
    let filled =
        gap_fill::fill(&gap, profile_of(fixture), fixture.segments).expect("the fill runs");
    assert_eq!(filled.refusal, GapFillRefusal::None);
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    assert!(sweep_mesh_measures::smallest_area(mesh.vertices(), &kept) > 0.0);

    let axis = tip_axis(fixture, size);
    let mut seen = HashSet::new();
    kept.iter()
        .flat_map(|&(a, b, c)| [a, b, c])
        .filter(|&v| seen.insert(v))
        .map(|v| mesh.vertices()[v as usize])
        // The right half only (x = -10 is the crease); the left tip is not in this pick.
        .filter(|p| (p.z - 20.0).abs() < 1e-6 && p.y < axis.y && p.x > -10.0)
        .map(|p| (Vector2::new(p.x, p.y) - axis).length())
        .fold(f64::NEG_INFINITY, f64::max)
}

/// For the right tip alone ({1}) and the whole pick ({0, 1, 2, 3}): at the saved 3.4 the right tip
/// alone and the whole pick cut, fill as one shell, and the right tip's top face still ends on its
/// round - the Onshape control builds all five runs at this size.
///
/// Each knife tip's band sets back 3.4 / tan(22.5) = 8.21 on its slope and the nose meets that
/// slope 7.07 from the tip, so from about 2.93 up the tip's front on the slope is cut in two by the
/// nose: z 0..9.464 and 10.536..20 at 3.4, both open, both the tip's alone. A one-chain build of an
/// open chain reads no loop count and builds exactly these pieces.
#[test]
fn at_the_saved_size_the_pick_cuts_and_fills() {
    let fixture = Rc::new(LevelLineFixture::load(FIXTURE));
    assert!((fixture.size - 3.4).abs() <= 1e-9);
    for pick in [vec![RIGHT_TIP_CHAIN], vec![0, 1, 2, 3]] {
        let gap = build_chains(&fixture, &pick, fixture.size, &as_saved(&fixture, None));
        assert_eq!(
            gap.refusal,
            BandGapRefusal::None,
            "{pick:?}: room {} set by {:?}",
            gap.room,
            gap.room_set_by
        );

        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None, "{pick:?}");
        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert!(sweep_mesh_measures::smallest_area(mesh.vertices(), &kept) > 0.0);

        // The right tip's top face ends on its round. "In front of the round" is the plan kite
        // between the tip and the round's two tangent points, 3.4 / tan(22.5) up the x = 0 face and
        // up the slope: the whole pick's top face also keeps vertices on the slope's top edge PAST
        // the slope's tangent point (9.5 and 10.7 from the tip), which are behind the round and
        // more than 3.4 from its axis.
        let axis = tip_axis(&fixture, fixture.size);
        let tip = Vector2::new(fixture.positions[12].x, fixture.positions[12].y);
        let setback = fixture.size / (std::f64::consts::PI / 8.0).tan();
        let up_the_face = Vector2::new(0.0, 1.0);
        let up_the_slope = Vector2::new(-1.0, 1.0).get_normal();
        let mut seen = HashSet::new();
        let worst = kept
            .iter()
            .flat_map(|&(a, b, c)| [a, b, c])
            .filter(|&v| seen.insert(v))
            .map(|v| mesh.vertices()[v as usize])
            .filter(|p| (p.z - 20.0).abs() < 1e-6)
            .map(|p| Vector2::new(p.x, p.y))
            .filter(|&p| {
                (p - tip).dot(up_the_face) < setback
                    && (p - tip).dot(up_the_slope) < setback
                    && p.x > -10.0
            })
            .map(|p| (p - axis).length())
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            worst <= fixture.size + 1e-3,
            "{pick:?}: the filled top face keeps a vertex {worst:.3} from the tip's axis"
        );
    }
}

/// A face rim's corners are not pair corners - they
/// share the picked face and turn their walls through a convex crease as the knife tip does, but
/// the rim is closed and its next edge cuts that wall itself. The knife tip's pick still finds its
/// corner.
#[test]
fn only_an_open_chains_mitred_corner_is_a_pair_corner() {
    for (name, has_one) in [
        ("fifteen-mm-f4-r005", false),
        ("fifteen-mm-f4-c27-r005", false),
        ("center-flows-through-f3-r005", false),
        (FIXTURE, true),
    ] {
        let fixture = LevelLineFixture::load(name);
        let feature = |a: i32, b: i32| fixture.is_feature_edge(a, b);
        let tags = vec![0; fixture.indices.len()];
        let found = PairCornerEndFaceField::find(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            &feature,
            fixture.size,
            &tags,
            None,
        );
        assert_eq!(found.is_some(), has_one, "{name}");

        // And as the build hands them over: cut into open pieces where their setbacks step.
        let (chains, closed) = chain_on_the_mesh::split_at_setback_steps(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            profile_of(&fixture),
            fixture.sharp_turn_degrees,
            Some(&feature),
        )
        .expect("every chain edge stands on two triangles");
        let found_split = PairCornerEndFaceField::find(
            &fixture.positions,
            &fixture.indices,
            &chains,
            &closed,
            &feature,
            fixture.size,
            &tags,
            None,
        );
        assert_eq!(
            found_split.is_some(),
            has_one,
            "{name}: {} pieces after the split",
            chains.len()
        );
    }
}

/// The corner field's least and greatest over a
/// triangle are exact - never above (least) or below (greatest) what a dense grid reads, and within
/// the grid's spacing of it.
#[test]
fn the_corner_fields_least_and_greatest_are_exact() {
    let field = PairCornerEndFaceField::of(
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, -1.0, 0.0).get_normal(),
        Vector3::new(0.0, 0.0, 1.0),
        2.5,
    );
    let mut random = SeededRandom::new(7);
    let mut any = || {
        let x = (random.next_double() * 12.0) - 9.0;
        let y = (random.next_double() * 12.0) - 9.0;
        let z = 23.0 - (random.next_double() * 9.0);
        Vector3::new(x, y, z)
    };
    const STEPS: i32 = 200;
    for i in 0..200 {
        let (a, b, c) = (any(), any(), any());
        let mut sampled_min = f64::INFINITY;
        let mut sampled_max = f64::NEG_INFINITY;
        for s in 0..=STEPS {
            for t in 0..=(STEPS - s) {
                let value = field.read(
                    a + ((b - a) * (s as f64 / STEPS as f64))
                        + ((c - a) * (t as f64 / STEPS as f64)),
                );
                sampled_min = sampled_min.min(value);
                sampled_max = sampled_max.max(value);
            }
        }

        let least = field.min_over_triangle(a, b, c).expect("no budget").value;
        let greatest = field
            .max_over_triangle(a, b, c, 1e-6)
            .expect("no budget")
            .value;
        let spacing = 0.1;
        assert!(least <= sampled_min + 1e-9, "triangle {i}");
        assert!(least >= sampled_min - spacing, "triangle {i}");
        assert!(greatest >= sampled_max - 1e-9, "triangle {i}");
        assert!(greatest <= sampled_max + spacing, "triangle {i}");
    }
}

/// The field cuts only in the corner's end
/// region. Its round's prism runs on along the member's first segment forever, and it is keyed to
/// the end face's whole side group, so without bounds it cut far down the member - past a partial
/// member's free end, or where a curved member has left the straight prism - and on a notch of the
/// end face standing off the end face's plane. In front of the round on the end face it still cuts.
#[test]
fn the_corner_field_cuts_only_in_the_corners_end_region() {
    const SIZE: f64 = 2.5;
    let corner = Vector3::new(0.0, 0.0, 20.0);
    let field = PairCornerEndFaceField::of(
        corner,
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, -1.0, 0.0).get_normal(),
        Vector3::new(0.0, 0.0, 1.0),
        SIZE,
    );
    let axis = field.axis_point();

    // Straight out from the axis toward the edge, between the tangent lines, past the round: in
    // front of it.
    let ahead = Vector3::new(axis.x, axis.y, 0.0)
        + ((Vector3::new(corner.x, corner.y, 0.0) - Vector3::new(axis.x, axis.y, 0.0)) * 0.9);
    assert!(
        field.read(Vector3::new(ahead.x, ahead.y, 20.0)) < 0.0,
        "the end face in front of the round is cut"
    );

    // (a), (b): the same plan point far down the member.
    assert!(
        field.read(Vector3::new(ahead.x, ahead.y, -30.0)) >= 0.0,
        "past the member's far end"
    );
    assert!(
        field.read(Vector3::new(ahead.x, ahead.y, 20.0 - (3.0 * SIZE))) >= 0.0,
        "down a curved member"
    );

    // (c): a notch of the end face's group, off the end face's plane by more than the size.
    assert!(
        field.read(Vector3::new(ahead.x, ahead.y, 20.0 - (1.5 * SIZE))) >= 0.0,
        "a notch below the end face"
    );
    assert!(
        field
            .min_over_triangle(
                Vector3::new(ahead.x, ahead.y, 10.0),
                Vector3::new(ahead.x + 1.0, ahead.y, 10.0),
                Vector3::new(ahead.x, ahead.y + 1.0, 10.0)
            )
            .expect("no budget")
            .value
            >= 0.0,
        "a whole notch face in the prism, 10 below the end face"
    );
}

/// At another dihedral - normals 120 degrees
/// apart, a 60 degree edge - the axis stands the size inside both faces and the field's zero set is
/// the round's cylinder in front of it.
#[test]
fn at_a_sixty_degree_edge_the_axis_and_zero_set_are_the_rounds() {
    const SIZE: f64 = 2.0;
    let corner = Vector3::new(0.0, 0.0, 0.0);
    let shared = Vector3::new(1.0, 0.0, 0.0);
    let own = Vector3::new(
        (std::f64::consts::PI * 2.0 / 3.0).cos(),
        (std::f64::consts::PI * 2.0 / 3.0).sin(),
        0.0,
    );
    let field = PairCornerEndFaceField::of(
        corner,
        Vector3::new(0.0, 0.0, -1.0),
        shared,
        own,
        Vector3::new(0.0, 0.0, 1.0),
        SIZE,
    );
    let axis = field.axis_point();
    assert!(((axis - corner).dot(shared) - -SIZE).abs() <= 1e-9);
    assert!(((axis - corner).dot(own) - -SIZE).abs() <= 1e-9);

    // On the cylinder, straight toward the edge: zero. Just inside: the round (positive); just
    // outside: cut.
    let toward = (corner - axis).get_normal();
    assert!(field.read(axis + (toward * SIZE)).abs() <= 1e-9);
    assert!(field.read(axis + (toward * (SIZE - 0.1))) > 0.0);
    assert!(field.read(axis + (toward * (SIZE + 0.1))) < 0.0);

    // Behind the axis, away from the edge: never cut.
    assert!(field.read(axis - (toward * (SIZE + 1.0))) > 0.0);
}

/// A face rim with a picked TAIL is still a rim - its
/// corners are on a cycle of picked edges, so they are not pair corners, though the tail makes one
/// rim corner a three-way junction and the rim no longer a pure ring.
#[test]
fn a_rim_with_a_picked_tail_has_no_pair_corners() {
    let fixture = LevelLineFixture::load("fifteen-mm-f4-r005");
    let rim = &fixture.chains[0];
    assert!(fixture.closed[0]);
    let rim_edges: HashSet<(i32, i32)> = rim
        .iter()
        .enumerate()
        .map(|(i, &v)| triangle_edges::key(v, rim[(i + 1) % rim.len()]))
        .collect();
    let tail = fixture
        .indices
        .iter()
        .flat_map(|&(a, b, c)| [(a, b), (b, c), (c, a)])
        .find(|&(u, v)| {
            rim.contains(&u)
                && !rim.contains(&v)
                && fixture.is_feature_edge(u, v)
                && !rim_edges.contains(&triangle_edges::key(u, v))
        })
        .expect("a tail edge");

    let mut chains = fixture.chains.clone();
    chains.push(vec![tail.0, tail.1]);
    let mut closed = fixture.closed.clone();
    closed.push(false);
    let feature = |a: i32, b: i32| fixture.is_feature_edge(a, b);
    let found = PairCornerEndFaceField::find(
        &fixture.positions,
        &fixture.indices,
        &chains,
        &closed,
        &feature,
        fixture.size,
        &vec![0; fixture.indices.len()],
        None,
    );
    assert!(found.is_none());
}

/// A drafted box edge is not a knife (88 degrees: no pair corner; 80: one): only an edge clearly
/// sharper than square is a knife - a box whose vertical edge is drafted to 88 degrees (normals 92
/// apart, shared . own = -0.035) has no pair corner where that edge meets the front face's top
/// edge, and the same corner with the edge at 80 degrees (shared . own = -0.17) has one.
#[test]
fn a_drafted_box_edge_is_not_a_knife() {
    for (edge_degrees, is_a_pair_corner) in [(88.0, false), (80.0, true)] {
        // A prism over a parallelogram with a vertex of edge_degrees at P0 = (0, 0): the front face
        // runs along y = 0 to P1 = (-10, 0), the right face from P0 to P3 at edge_degrees from it;
        // z from 0 to 10.
        let angle = edge_degrees * std::f64::consts::PI / 180.0;
        let to_p3 = Vector3::new(-angle.cos(), -angle.sin(), 0.0) * 10.0;
        let plan = [
            Vector3::ZERO,
            Vector3::new(-10.0, 0.0, 0.0),
            Vector3::new(-10.0, 0.0, 0.0) + to_p3,
            to_p3,
        ];
        let mut positions = plan.to_vec();
        positions.extend(plan.iter().map(|&p| p + Vector3::new(0.0, 0.0, 10.0)));

        // Bottom 0..3, top 4..7, the plan counter-clockwise, every triangle wound outward.
        let mut indices = vec![(4, 5, 6), (4, 6, 7), (0, 2, 1), (0, 3, 2)];
        for i in 0..4 {
            let j = (i + 1) % 4;
            indices.push((i, j, j + 4));
            indices.push((i, j + 4, i + 4));
        }

        // The drafted vertical edge up to the corner T0, then the front face's top edge: shared
        // face the front, the other sides the right face and the top, joined through T0 by the
        // unselected top-right edge.
        let chains = vec![vec![0, 4, 5]];
        let found = PairCornerEndFaceField::find(
            &positions,
            &indices,
            &chains,
            &[false],
            &|_, _| true,
            1.0,
            &vec![0; indices.len()],
            None,
        );
        assert_eq!(found.is_some(), is_a_pair_corner, "{edge_degrees}");
    }
}

/// The right tip's fillet axis in plan at `size` - `size` in from both the x = 0 face
/// and the 45 degree slope that meet at the tip v12, so (tip.x - size, tip.y + size (1 + sqrt 2)).
fn tip_axis(fixture: &LevelLineFixture, size: f64) -> Vector2 {
    let tip = fixture.positions[12];
    Vector2::new(tip.x - size, tip.y + size * (1.0 + 2f64.sqrt()))
}
