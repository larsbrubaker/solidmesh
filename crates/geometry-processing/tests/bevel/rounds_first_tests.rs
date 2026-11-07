//! Inner Right Sides and Center's knife tip built rounds first (`BandGap.RoundsFirst`): the convex
//! ridge 9 - 1 alone (pass A), then its two concave beads 5 - 9 and 9 - 11 - 8 read again on what
//! that left. In the kernel's frame the notch wall is the plane x - y = 98.5 (v0, v1, v2, v5, v8)
//! and the slope x + y = -118.5 (v4, v5, v9, v11, v8); the ridge runs from v1 on the wall to v9 on
//! the slope, and the part is symmetric about z = 10.

use std::collections::HashSet;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap::BandGapBuildError;
use geometry_processing::bevel::level_line::band_gap_after_the_rounds::{
    creases_after_the_rounds, feature_edges_after_the_rounds,
};
use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_in_turn::BuildAndFillError;
use geometry_processing::bevel::level_line::band_gap_rounds_first::{
    build_rounds_first, build_the_rounds, chain_senses, takes_rounds_first,
};
use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandProfile, GapFillRefusal, GapFillResult, WorkBudget,
};
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

const NAME: &str = "inner-right-sides-and-center-saved";

/// The app's control comparison tolerance (BevelControlMeshComparison.ChordTolerance in
/// MatterCADTests).
const CONTROL_TOLERANCE: f64 = 0.05;

/// The fixture came from float storage: its plane offsets carry ~1e-6 noise at 100 mm (v12 is
/// 1.3e-6 off the slope).
const FLOAT_NOISE: f64 = 1e-5;

fn off_the_wall(p: Vector3) -> f64 {
    (p.x - p.y - 98.5) / 2f64.sqrt()
}

fn off_the_slope(p: Vector3) -> f64 {
    (p.x + p.y + 118.5) / 2f64.sqrt()
}

fn load() -> Rc<LevelLineFixture> {
    Rc::new(LevelLineFixture::load(NAME))
}

fn profile(fixture: &LevelLineFixture) -> BandProfile {
    if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    }
}

/// The fixture's feature-edge list as the build's feature test.
fn feature_of(fixture: &Rc<LevelLineFixture>) -> FeatureEdgeTest {
    let fixture = Rc::clone(fixture);
    Rc::new(move |a, b| fixture.is_feature_edge(a, b))
}

/// Builds the fixture's rounds.
fn the_rounds(fixture: &Rc<LevelLineFixture>) -> GapFillResult {
    build_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        Some(feature_of(fixture)),
        fixture.size,
        profile(fixture),
        fixture.segments,
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        None,
    )
    .expect("pass A runs")
}

/// The rounds-first build of the fixture, closed as the fixture records.
fn rounds_first(fixture: &Rc<LevelLineFixture>) -> GapFillResult {
    build_rounds_first(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(feature_of(fixture)),
        fixture.size,
        profile(fixture),
        fixture.segments,
        None,
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        None,
        Some(&fixture.closed),
    )
    .expect("the build runs")
}

fn site() -> String {
    format!("site {}", ends_site().unwrap_or_default())
}

#[test]
fn the_ridge_alone_runs_out_through_both_end_planes() {
    let fixture = load();
    assert_eq!(
        chain_senses(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            Some(&fixture.closed)
        ),
        vec![Some(false), Some(true), Some(false)]
    );
    assert!(takes_rounds_first(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed)
    ));

    let rounds = the_rounds(&fixture);
    assert_eq!(rounds.refusal, GapFillRefusal::None, "{}", site());
    let mesh = rounds.mesh.as_ref().expect("the rounds' mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();
    assert!(sweep_mesh_measures::is_closed_and_consistent(at, &kept));
    assert!(
        !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
            at,
            &kept,
            &fixture.positions,
            &fixture.indices
        ),
        "self-intersection"
    );

    // The ridge's ends are rounded away: v1 and v9 may stay only as corners of the end faces, which
    // grow over the round's section; and the round itself (off both side faces, which meet the end
    // planes only in straight lines) reaches each end plane.
    let used: HashSet<i32> = kept.iter().flat_map(|&(a, b, c)| [a, b, c]).collect();
    let off_their_end_face = kept
        .iter()
        .filter(|&&(a, b, c)| {
            let corners = [a, b, c];
            (corners.contains(&1)
                && corners
                    .iter()
                    .any(|&v| off_the_wall(at[v as usize]).abs() > FLOAT_NOISE))
                || (corners.contains(&9)
                    && corners
                        .iter()
                        .any(|&v| off_the_slope(at[v as usize]).abs() > FLOAT_NOISE))
        })
        .count();
    assert_eq!(
        off_their_end_face, 0,
        "v1 stays only in the wall and v9 only in the slope"
    );
    let lower = plane(&fixture, 1, 5, 9);
    let upper = plane(&fixture, 1, 9, 11);
    let on_the_round = |v: i32| {
        v >= fixture.positions.len() as i32
            && (lower.0.dot(at[v as usize]) - lower.1).abs() > 1e-3
            && (upper.0.dot(at[v as usize]) - upper.1).abs() > 1e-3
    };
    let on_the_wall = used
        .iter()
        .filter(|&&v| on_the_round(v) && off_the_wall(at[v as usize]).abs() < FLOAT_NOISE)
        .count();
    let on_the_slope = used
        .iter()
        .filter(|&&v| on_the_round(v) && off_the_slope(at[v as usize]).abs() < FLOAT_NOISE)
        .count();
    assert!(on_the_wall > 0, "the round runs out through the wall at v1");
    assert!(
        on_the_slope > 0,
        "the round runs out through the slope at v9"
    );
}

/// Pass A's mesh has no two kept vertices standing closer than 1e-6 of the size: a vertex a hair
/// from another leaves a zero-area triangle, and pass B's cut reads it as a crease or crosses it
/// with a zero-length rail edge.
#[test]
fn the_rounds_leave_no_twin_vertices() {
    let fixture = load();
    let rounds = the_rounds(&fixture);
    let mesh = rounds.mesh.as_ref().expect("the rounds' mesh").borrow();
    let at = mesh.vertices();
    let mut used: Vec<i32> = sweep_mesh_measures::kept(&mesh)
        .iter()
        .flat_map(|&(a, b, c)| [a, b, c])
        .collect();
    used.sort();
    used.dedup();
    let mut twins = Vec::new();
    for &a in &used {
        for &b in used
            .iter()
            .filter(|&&b| b > a && (at[b as usize] - at[a as usize]).length() < 1e-6 * fixture.size)
        {
            let p = at[a as usize];
            twins.push(format!(
                "{a}/{b} at ({:.4},{:.4},{:.4}) {:.3e} apart",
                p.x,
                p.y,
                p.z,
                (at[b as usize] - p).length()
            ));
        }
    }

    assert!(twins.is_empty(), "{}", twins.join("; "));
}

/// A convex edge meeting ONE concave edge end to end - a stepped corner, here the ridge 9 - 1 with
/// only the bead 5 - 9 - is not a knife tip: the crease re-read needs the two concave chains that
/// join over the round, so such a pick must not pay for pass A on every trial size.
#[test]
fn a_stepped_corner_is_not_rounds_first() {
    let fixture = load();
    let stepped = vec![fixture.chains[0].clone(), fixture.chains[1].clone()];
    assert_eq!(
        chain_senses(
            &fixture.positions,
            &fixture.indices,
            &stepped,
            Some(&[false, false])
        ),
        vec![Some(false), Some(true)]
    );
    assert!(!takes_rounds_first(
        &fixture.positions,
        &fixture.indices,
        &stepped,
        Some(&[false, false])
    ));
}

/// A rounds-first build answers its caller's budget in both passes: a drag cancelled while a
/// knife-tip pick builds leaves at once, as every other build does.
#[test]
fn a_cancelled_build_leaves_at_once() {
    let fixture = load();
    let cancel = CancelToken::default();
    cancel.cancel();
    let budget_for = |_count: i32| {
        Rc::new(
            WorkBudget::with_cancellation(i64::MAX, cancel.clone(), None)
                .expect("a positive limit"),
        )
    };
    let whole = build_rounds_first(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(feature_of(&fixture)),
        fixture.size,
        profile(&fixture),
        fixture.segments,
        None,
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        Some(&budget_for),
        Some(&fixture.closed),
    );
    assert!(matches!(
        whole,
        Err(BuildAndFillError::Build(BandGapBuildError::Cancelled))
    ));

    // Pass A on its own too: it is the whole cost of a trial size where pass B never starts.
    let pass_a = build_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        Some(feature_of(&fixture)),
        fixture.size,
        profile(&fixture),
        fixture.segments,
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        Some(&budget_for),
    );
    assert!(matches!(
        pass_a,
        Err(BuildAndFillError::Build(BandGapBuildError::Cancelled))
    ));
}

#[test]
fn the_creases_join_over_the_round() {
    let fixture = load();
    let rounds = the_rounds(&fixture);
    assert_eq!(rounds.refusal, GapFillRefusal::None, "{}", site());
    let mesh = rounds.mesh.as_ref().expect("the rounds' mesh").borrow();
    let creases = creases_after_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        &mesh,
    )
    .expect("the creases");
    assert_eq!(creases.len(), 1);
    let crease = &creases[0];
    assert_eq!(crease[0], 5);
    assert_eq!(*crease.last().expect("a crease"), 8);
    assert!(crease.contains(&11));

    let at = mesh.vertices();
    assert!(
        crease
            .iter()
            .all(|&v| off_the_slope(at[v as usize]).abs() < FLOAT_NOISE),
        "the whole crease lies on the slope"
    );
    let crossings = crease
        .windows(2)
        .filter(|w| (at[w[0] as usize].z - 10.0) * (at[w[1] as usize].z - 10.0) <= 0.0)
        .count();
    assert!(crossings > 0, "the crease crosses z = 10 over the round");

    drop(mesh);
    let feature = feature_edges_after_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        Some(feature_of(&fixture)),
        &rounds,
    )
    .expect("the feature reading");
    let not_features: Vec<String> = crease
        .windows(2)
        .filter(|w| !feature(w[0], w[1]))
        .map(|w| format!("({}, {})", w[0], w[1]))
        .collect();
    assert!(not_features.is_empty(), "{}", not_features.join(", "));
}

/// Under pass B's feature predicate the notch wall is fenced off from the crease's side: every edge
/// between a wall triangle and any other face of the rounds' mesh is a feature. The round's run-out
/// left a wall triangle (v1, 168, 22) tilted 2.5e-5 by float noise; read by plane offsets it stood
/// 1.2e-3 off the wall, counted as a facet of the round, and its edge with the round's first facet
/// was no feature - the side flooded over the wall, and the end section at v5 found no untagged
/// wall to insert into.
#[test]
fn the_wall_stays_off_the_creases_side() {
    let fixture = load();
    let rounds = the_rounds(&fixture);
    let feature = feature_edges_after_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        Some(feature_of(&fixture)),
        &rounds,
    )
    .expect("the feature reading");
    let mesh = rounds.mesh.as_ref().expect("the rounds' mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();
    let on_the_wall = |t: (i32, i32, i32)| {
        [t.0, t.1, t.2]
            .iter()
            .all(|&v| off_the_wall(at[v as usize]).abs() < FLOAT_NOISE)
    };
    let mut open: Vec<String> = sweep_mesh_measures::standing(&kept)
        .iter()
        .filter(|(e, on)| {
            on.len() == 2
                && on_the_wall(kept[on[0] as usize]) != on_the_wall(kept[on[1] as usize])
                && !feature(e.0, e.1)
        })
        .map(|(e, _)| format!("{}-{}", e.0, e.1))
        .collect();
    open.sort();
    assert!(open.is_empty(), "{}", open.join(" "));
}

/// Pass B's cut clears the saved size. Both ends of the joined crease stand on the notch wall, and
/// the round's last facets meet that wall too, so the wall was read as a GROWING neighbour across
/// the band whose far creases (8 - 18 among them) run out of the chain's own end at v8: room 0,
/// refused Size at every size. The face a chain ends on is its end face, not a neighbour across the
/// band (GrowingCreaseRuns.ReadBounds).
#[test]
fn the_joined_crease_cuts_at_the_saved_size() {
    let fixture = load();
    let rounds = the_rounds(&fixture);
    let feature = feature_edges_after_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        Some(feature_of(&fixture)),
        &rounds,
    )
    .expect("the feature reading");
    let mesh = rounds.mesh.as_ref().expect("the rounds' mesh").borrow();
    let creases = creases_after_the_rounds(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        Some(&fixture.closed),
        &mesh,
    )
    .expect("the creases");
    let kept = sweep_mesh_measures::kept(&mesh);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: None,
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    let gap = band_gap_entry_points::build_pick(
        mesh.vertices(),
        &kept,
        &creases,
        &[false],
        Some(feature),
        fixture.size,
        profile(&fixture),
        &options,
    )
    .expect("the cut runs");
    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "room {} set by {:?}",
        gap.room,
        gap.room_set_by
    );

    // Both ends' planes are the one wall. A rail vertex at the START (v5) reads the start's
    // parameter, not the far end's: v8 stands 6.7e-7 off the wall through v5 (float storage), so
    // the last end's plane read a hair higher at every wall point and v5's rail corner read 12.231 -
    // the trim then found no rail to cut at end 0 and the fill refused.
    let start = mesh.vertices()[5];
    let gap_mesh = gap.mesh.as_ref().expect("the cut mesh").borrow();
    let length = gap.chain().expect("the chain").length;
    let misread: Vec<String> = gap
        .loops
        .iter()
        .flat_map(|l| l.vertices.iter().zip(&l.parameters))
        .filter(|&(&v, &t)| (gap_mesh.position(v) - start).length() < 1.0 && t > length / 2.0)
        .map(|(v, t)| format!("{v}@{t:.3}"))
        .collect();
    assert!(misread.is_empty(), "{}", misread.join(" "));

    let repeats: Vec<String> = gap
        .loops
        .iter()
        .flat_map(|l| l.vertices.windows(2).map(|w| (w[0], w[1])))
        .filter(|&(a, b)| {
            a == b || (gap_mesh.position(a) - gap_mesh.position(b)).length() < 1e-6 * fixture.size
        })
        .map(|(a, b)| format!("{a}-{b}"))
        .collect();
    assert!(
        repeats.is_empty(),
        "no rail steps nowhere: {}",
        repeats.join(" ")
    );
}

/// Pass B, the one crease 5 - ... - 8 built on the rounds' mesh, closes into one consistent shell
/// that does not cross itself.
#[test]
fn the_joined_crease_builds_one_shell() {
    let fixture = load();
    let built = rounds_first(&fixture);
    assert_eq!(built.refusal, GapFillRefusal::None, "{}", site());
    let mesh = built.mesh.as_ref().expect("the built mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();
    assert!(sweep_mesh_measures::is_closed_and_consistent(at, &kept));
    assert_eq!(sweep_mesh_measures::shell_count(at.len() as i32, &kept), 1);
    assert!(
        !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
            at,
            &kept,
            &fixture.positions,
            &fixture.indices
        ),
        "self-intersection"
    );
}

/// The rounds-first build keeps the part's mirror symmetry about z = 10: every vertex the bevel
/// added, mirrored, lies on the built surface to the app's control tolerance
/// (BevelControlMeshComparison.ChordTolerance, 0.05).
///
/// Not tighter: 0.0202 at worst (1274 at (-7.691, -111.424, 11.451); 524 at z 8.526, 0.019), from
/// the open strip's quad diagonals, which all lean one way. The slope cuts them beside each round
/// row end of pass A always at the higher z (8.482, 8.658, 8.863 below 10; 11.548, 11.411, 11.252
/// above, against mirrors 11.518, 11.342, 11.137), so pass B's two halves are swept along
/// differently sampled creases and their rails differ (lower 2.822, 3.101, 3.392 against mirrored
/// upper 2.815, 3.072, 3.348). Laying the diagonals mirror-symmetrically is a general strip change
/// (docs/modeling/bevel_plan.md).
#[test]
fn the_rounds_first_build_is_mirror_symmetric() {
    let fixture = load();
    let built = rounds_first(&fixture);
    assert_eq!(built.refusal, GapFillRefusal::None, "{}", site());
    let mesh = built.mesh.as_ref().expect("the built mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();

    // The part's own triangulation is not mirror-symmetric (v17 at z 8.333 has no twin at 11.667),
    // so symmetry is read on the SURFACE: every vertex the bevel added, mirrored about z = 10, lies
    // on the built mesh.
    let mut added: Vec<i32> = Vec::new();
    for v in kept.iter().flat_map(|&(a, b, c)| [a, b, c]) {
        if v >= fixture.positions.len() as i32 && !added.contains(&v) {
            added.push(v);
        }
    }
    let mut off_mirror: Vec<(i32, f64)> = added
        .iter()
        .map(|&v| {
            let p = at[v as usize];
            (
                v,
                distance_to_mesh(Vector3::new(p.x, p.y, 20.0 - p.z), at, &kept),
            )
        })
        .collect();
    // Worst first; `sort_by` is stable, so ties keep their vertex order.
    off_mirror.sort_by(|a, b| b.1.total_cmp(&a.1));
    let worst: Vec<String> = off_mirror
        .iter()
        .take(5)
        .map(|&(v, d)| {
            let p = at[v as usize];
            format!("{v} ({:.3},{:.3},{:.3}) mirror {d:.3e} off", p.x, p.y, p.z)
        })
        .collect();
    assert!(off_mirror[0].1 <= CONTROL_TOLERANCE, "{}", worst.join("; "));
}

/// The rounds-first build publishes no triangle standing less than 3.6e-4 mm on its own longest
/// edge - the app's weld tolerance for this part (BevelPublishedTriangulationRegressionTests reads
/// the same thing on the published mesh). Pass A alone has none. Pass B laid them where the outer
/// rail's vertices crowd over the round - one smooth joint's parameter for a run of them
/// (ChainField.AcrossSmoothJoints), a row for every one of them 1e-4 apart
/// (OpenGapFill.RowsWhereTheRoundNeedsThem) - and at the end face, five end-clip corners on one
/// crease edge (EndClipCorners).
#[test]
fn the_rounds_first_build_has_no_slivers() {
    let fixture = load();
    let built = rounds_first(&fixture);
    assert_eq!(built.refusal, GapFillRefusal::None, "{}", site());
    let mesh = built.mesh.as_ref().expect("the built mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let at = mesh.vertices();
    let mut thin: Vec<((i32, i32, i32), f64)> = kept
        .iter()
        .map(|&t| {
            (
                t,
                altitude(at[t.0 as usize], at[t.1 as usize], at[t.2 as usize]),
            )
        })
        .filter(|e| e.1 < 3.6e-4)
        .collect();
    thin.sort_by(|a, b| a.1.total_cmp(&b.1));
    let named: Vec<String> = thin
        .iter()
        .take(6)
        .map(|&(t, h)| {
            let p = at[t.0 as usize];
            format!(
                "{}/{}/{} {h:.3e} high at ({:.4},{:.4},{:.4})",
                t.0, t.1, t.2, p.x, p.y, p.z
            )
        })
        .collect();
    assert!(
        thin.is_empty(),
        "{} of {}: {}",
        thin.len(),
        kept.len(),
        named.join("; ")
    );
}

/// The height of a triangle on its own longest edge.
fn altitude(a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let longest = f64::max(
        (b - a).length(),
        f64::max((c - b).length(), (a - c).length()),
    );
    if longest == 0.0 {
        0.0
    } else {
        (b - a).cross(c - a).length() / longest
    }
}

/// The distance from `p` to the nearest of `triangles`.
pub fn distance_to_mesh(p: Vector3, at: &[Vector3], triangles: &[(i32, i32, i32)]) -> f64 {
    let mut best = f64::INFINITY;
    for &(a, b, c) in triangles {
        best = f64::min(
            best,
            (p - closest_on_triangle(p, at[a as usize], at[b as usize], at[c as usize])).length(),
        );
    }

    best
}

/// The closest point of triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_triangle(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = (d1 * d4) - (d3 * d2);
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + (ab * (d1 / (d1 - d3)));
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = (d5 * d2) - (d1 * d6);
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + (ac * (d2 / (d2 - d6)));
    }

    let va = (d3 * d6) - (d5 * d4);
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + ((c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }

    let denom = 1.0 / (va + vb + vc);
    a + (ab * (vb * denom)) + (ac * (vc * denom))
}

/// The unit normal and offset of the plane through three fixture vertices.
fn plane(fixture: &LevelLineFixture, a: usize, b: usize, c: usize) -> (Vector3, f64) {
    let p = &fixture.positions;
    let normal = (p[b] - p[a]).cross(p[c] - p[a]).get_normal();
    (normal, normal.dot(p[a]))
}
