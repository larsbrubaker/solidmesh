//! A MEASUREMENT of "5mm Bevel Around Center.mcx" as the app route hands it to the kernel
//! (`around-center-r5.txt`, one closed 5-edge concave chain with four sharp kinks where an unpicked
//! crease runs in) against the file's Onshape control (`around-center-control.txt`, same frame):
//! per kink the turn, the dihedrals, the unpicked crease and the side tags; the setback along the
//! chain; and at the level line's worst published vertex, where it is against the control. The flow
//! design's reading (`the_flow_design_reads_against_the_control`) is the `flow` module beside this
//! file, so both tests sit under this class's path.
//!
//! Asserts nothing. Returns at once unless `BEVEL_AROUND_CENTER_MEASURE=1`; writes its reading to
//! the file named by `BEVEL_AROUND_CENTER_MEASURE_LOG` (the flow reading to that name plus
//! `.flow.txt`), and to stdout: `BEVEL_AROUND_CENTER_MEASURE=1 cargo nextest run --workspace --test
//! bevel --profile measure --no-capture around_center_kink_measure_tests`. It runs alone (a
//! `threads-required` override in `.config/nextest.toml`), so its timings read undisturbed.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_side_groups::group_sides;
use geometry_processing::bevel::level_line::band_side_tagging_ends::{tag_sides, SideReach};
use geometry_processing::bevel::level_line::{
    chain_on_the_mesh, chain_setback, gap_fill, triangle_edges, BandGapResult, ChainField,
};
use indexmap::IndexSet;
use vector_math::Vector3;

use crate::encroaching_side_room_tests::up_to;
use crate::end_face_builds::feature_of;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;

#[path = "around_center_kink_measure_tests_flow.rs"]
mod flow;

/// The run variable both measurements are gated on.
pub(super) const RUN_VARIABLE: &str = "BEVEL_AROUND_CENTER_MEASURE";

/// The file the reading is written to, when set.
pub(super) const LOG_VARIABLE: &str = "BEVEL_AROUND_CENTER_MEASURE_LOG";

/// A point as `(x, y, z)`, each coordinate to at most three decimals with trailing zeros dropped.
pub(super) fn p(v: Vector3) -> String {
    format!("({}, {}, {})", up_to(v.x, 3), up_to(v.y, 3), up_to(v.z, 3))
}

/// The fixture's whole pick at its size and profile, every optional argument at its default.
pub(super) fn build(fixture: &Rc<LevelLineFixture>) -> BandGapResult {
    band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(fixture)),
        fixture.size,
        profile_of(fixture),
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
}

/// The control surface's distance from `q` and the control point it
/// is read at (the first triangle reaching the least distance).
pub(super) fn to_control(control: &LevelLineFixture, q: Vector3) -> (f64, Vector3) {
    let mut best = f64::INFINITY;
    let mut at = q;
    for &(a, b, c) in &control.indices {
        let on = ChainField::closest_on_triangle(
            q,
            control.positions[a as usize],
            control.positions[b as usize],
            control.positions[c as usize],
        );
        let d = (on - q).length();
        if d < best {
            best = d;
            at = on;
        }
    }

    (best, at)
}

/// Writes the reading to `LOG_VARIABLE`'s file (plus `suffix`) when it is set, and to stdout.
pub(super) fn publish(log: &str, suffix: &str) {
    if let Ok(path) = std::env::var(LOG_VARIABLE) {
        if !path.is_empty() {
            std::fs::write(format!("{path}{suffix}"), log).expect("the log file is writable");
        }
    }

    println!("{log}");
}

fn degrees(cos: f64) -> f64 {
    cos.clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI
}

/// The kinked chain reads against the control.
#[test]
fn the_kinked_chain_reads_against_the_control() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load("around-center-r5"));
    let control = LevelLineFixture::load("around-center-control");
    let positions = &fixture.positions;
    let indices = &fixture.indices;
    let chain = &fixture.chains[0];
    let profile = profile_of(&fixture);
    let mut log = String::new();

    let standing = triangle_edges::standing_map(indices);
    let n = |t: i32| triangle_edges::normal(positions, indices[t as usize]);

    // Signed turn between an edge's two triangles: positive concave (the second triangle rises
    // above the first's plane), in degrees of normal turn.
    let dihedral = |a: i32, b: i32| {
        let two = &standing[&triangle_edges::key(a, b)];
        let n0 = n(two[0]);
        let (p0, q0, r0) = indices[two[1] as usize];
        let opposite = [p0, q0, r0]
            .into_iter()
            .find(|&v| v != a && v != b)
            .expect("a triangle has a third corner");
        let turn = degrees(n0.dot(n(two[1])));
        if n0.dot(positions[opposite as usize] - positions[a as usize]) < 0.0 {
            -turn
        } else {
            turn
        }
    };

    let count = chain.len();
    let chain_edges: Vec<(i32, i32)> = (0..count)
        .map(|i| (chain[i], chain[(i + 1) % count]))
        .collect();
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(f, t)| triangle_edges::key(f, t))
        .collect();
    let is_feature = |a: i32, b: i32| fixture.is_feature_edge(a, b);
    let (tags, divides) = tag_sides(
        indices,
        &chain_edges,
        &is_feature,
        true,
        Some(positions),
        (false, false),
        SideReach::default(),
    )
    .expect("the chain has two sides");
    let (_, groups, undivided) = group_sides(
        indices,
        std::slice::from_ref(&chain_edges),
        &is_feature,
        true,
        Some(positions),
        None,
        None,
    )
    .expect("the sides group");
    let chain_text: Vec<String> = chain.iter().map(i32::to_string).collect();
    let _ = writeln!(
        log,
        "chain [{}] closed, size {}, {profile:?}",
        chain_text.join(" "),
        fixture.size
    );
    let mut tag_values: Vec<i32> = tags.clone();
    tag_values.sort_unstable();
    tag_values.dedup();
    let tag_counts: Vec<String> = tag_values
        .iter()
        .map(|&g| format!("tag {g} x{}", tags.iter().filter(|&&t| t == g).count()))
        .collect();
    let _ = writeln!(
        log,
        "TagSides divides {}; tag counts: {}",
        if divides { "True" } else { "False" },
        tag_counts.join(", ")
    );
    let _ = writeln!(
        log,
        "GroupSides: {} groups, undivided chain {undivided}",
        groups.len()
    );

    let unit =
        chain_on_the_mesh::unit_setbacks(positions, indices, chain, true, &chain_edges, profile)
            .expect("every chain edge stands on two triangles");
    for (e, &(from, to)) in chain_edges.iter().enumerate() {
        let (left, right) = chain_on_the_mesh::sides_of(indices, &standing, from, to)
            .expect("the chain edge stands on two triangles");
        let own = chain_setback::for_fillet(fixture.size, n(left), n(right));
        let _ = writeln!(
            log,
            "edge {e} v{from}->v{to} {}->{} len {}: dihedral {}, own setback {}, left t{left} n{} tag {}, right t{right} n{} tag {}",
            p(positions[from as usize]),
            p(positions[to as usize]),
            up_to((positions[to as usize] - positions[from as usize]).length(), 3),
            up_to(dihedral(from, to), 2),
            up_to(own, 3),
            p(n(left)),
            tags[left as usize],
            p(n(right)),
            tags[right as usize]
        );
    }

    let _ = writeln!(
        log,
        "per chain vertex (setback = mean of the two edges' own, x size):"
    );
    for i in 0..count {
        let v = chain[i];
        let at = positions[v as usize];
        let in_dir = (at - positions[chain[(i + count - 1) % count] as usize]).get_normal();
        let out_dir = (positions[chain[(i + 1) % count] as usize] - at).get_normal();
        let turn = degrees(in_dir.dot(out_dir));
        let _ = writeln!(
            log,
            "  v{v} {}: turn {}deg, vertex setback {}",
            p(at),
            up_to(turn, 2),
            up_to(unit[i] * fixture.size, 3)
        );
        for key in standing
            .keys()
            .filter(|k| (k.0 == v || k.1 == v) && !chain_keys.contains(k))
        {
            let other = if key.0 == v { key.1 } else { key.0 };
            let dir = (positions[other as usize] - at).get_normal();
            let feature = fixture.is_feature_edge(key.0, key.1);
            if dihedral(key.0, key.1).abs() > 1.0 {
                let fan_tags: Vec<String> = standing[key]
                    .iter()
                    .map(|&t| format!("t{t}:{}", tags[t as usize]))
                    .collect();
                let _ = writeln!(
                    log,
                    "    unpicked {} crease to v{other} dir {} dihedral {}, angle to in {}, to out {}; triangles tags {}",
                    if feature { "FEATURE" } else { "plain" },
                    p(dir),
                    up_to(dihedral(key.0, key.1), 2),
                    up_to(degrees(-in_dir.dot(dir)), 1),
                    up_to(degrees(out_dir.dot(dir)), 1),
                    fan_tags.join("/")
                );
            }
        }

        let fan: Vec<String> = (0..indices.len())
            .filter(|&t| {
                let (a, b, c) = indices[t];
                a == v || b == v || c == v
            })
            .map(|t| format!("t{t} tag {} n{}", tags[t], p(n(t as i32))))
            .collect();
        let _ = writeln!(log, "    fan: {}", fan.join(", "));
    }

    let gap = build(&fixture);
    let _ = writeln!(log, "band {:?}, room {}", gap.refusal, up_to(gap.room, 3));
    if gap.built() {
        read_the_band(&mut log, &fixture, &control, &gap);
    }

    publish(&log, "");
}

/// The rest of `the_kinked_chain_reads_against_the_control` on a built band: the field's setbacks,
/// the fill's worst published vertices, the rail, and the control's own blend boundary.
fn read_the_band(
    log: &mut String,
    fixture: &LevelLineFixture,
    control: &LevelLineFixture,
    gap: &BandGapResult,
) {
    let positions = &fixture.positions;
    let indices = &fixture.indices;
    let n = |t: usize| triangle_edges::normal(positions, indices[t]);
    let field = gap.chain().expect("a built band has its chain");
    let evaluate = |q: Vector3| field.evaluate(q).expect("the field reads");
    let _ = writeln!(
        log,
        "field setbacks min {} max {}, length {}",
        up_to(field.min_setback, 3),
        up_to(field.max_setback, 3),
        up_to(field.length, 3)
    );
    for s in 0..=40 {
        let t = field.length * s as f64 / 40.0;
        let _ = writeln!(
            log,
            "  setback({}) = {} at {}",
            up_to(t, 2),
            up_to(field.setback_at(t), 3),
            p(field.point_at(t))
        );
    }

    let filled = gap_fill::fill(gap, profile_of(fixture), fixture.segments).expect("the fill runs");
    let _ = writeln!(log, "fill {:?}", filled.refusal);
    if let Some(mesh) = filled.mesh.as_ref() {
        let mesh = mesh.borrow();
        let used: HashSet<i32> = mesh
            .triangles()
            .iter()
            .enumerate()
            .filter(|&(i, _)| !mesh.is_removed(i as i32))
            .flat_map(|(_, t)| [t.a, t.b, t.c])
            .collect();
        let mut worst_list: Vec<(i32, Vector3, f64, Vector3)> = (0..mesh.vertex_count())
            .filter(|v| used.contains(v))
            .map(|v| {
                let q = mesh.position(v);
                let (d, at) = to_control(control, q);
                (v, q, d, at)
            })
            .collect();
        // A stable sort keeps ties in vertex order, so the reading is the same every run.
        worst_list.sort_by(|a, b| b.2.total_cmp(&a.2));
        let _ = writeln!(log, "worst published vertices to the control:");
        for &(v, q, d, at) in worst_list.iter().take(12) {
            let sample = evaluate(q);
            let _ = writeln!(
                log,
                "  v{v} {} off {} (control at {}), chain param {} nearest {} dist {} setback there {} value {}",
                p(q),
                up_to(d, 3),
                p(at),
                up_to(sample.parameter, 2),
                p(sample.nearest),
                up_to(sample.distance, 3),
                up_to(field.setback_at(sample.parameter), 3),
                up_to(sample.value, 3)
            );
        }
    }

    // The rail: band-mesh vertices on the level line (value ~0), on the input's faces.
    let _ = writeln!(
        log,
        "rail vertices (|value| < 1e-6) with their distance to the chain and the control:"
    );
    if let Some(band) = gap.mesh.as_ref() {
        let band = band.borrow();
        for v in 0..band.vertex_count() {
            let q = band.position(v);
            let sample = evaluate(q);
            if sample.value.abs() < 1e-6 {
                let _ = writeln!(
                    log,
                    "  r{v} {} param {} dist {} to control {}",
                    p(q),
                    up_to(sample.parameter, 2),
                    up_to(sample.distance, 3),
                    up_to(to_control(control, q).0, 3)
                );
            }
        }
    }

    // The control's own blend boundary: control vertices on a source face plane with a control
    // neighbour off every plane - with their distance to the chain (the control's setback there).
    let planes: Vec<(Vector3, f64)> = (0..indices.len())
        .map(|t| (n(t), n(t).dot(positions[indices[t].0 as usize])))
        .collect();
    let on_source_of = |q: Vector3| {
        planes.iter().any(|&(pn, pd)| (pn.dot(q) - pd).abs() < 1e-3)
            && indices.iter().any(|&(a, b, c)| {
                (ChainField::closest_on_triangle(
                    q,
                    positions[a as usize],
                    positions[b as usize],
                    positions[c as usize],
                ) - q)
                    .length()
                    < 1e-3
            })
    };
    let on_source: Vec<bool> = control.positions.iter().map(|&q| on_source_of(q)).collect();
    let _ = writeln!(
        log,
        "control blend boundary (on the source, beside a blend vertex):"
    );
    // An insertion-ordered set, so the boundary prints in the order the triangles meet it.
    let mut boundary: IndexSet<i32> = IndexSet::new();
    for &(a, b, c) in &control.indices {
        let vs = [a, b, c];
        if vs.iter().any(|&x| !on_source[x as usize]) {
            for &x in vs.iter().filter(|&&x| on_source[x as usize]) {
                boundary.insert(x);
            }
        }
    }

    let parameter_of = |x: i32| evaluate(control.positions[x as usize]).parameter;
    let mut boundary: Vec<i32> = boundary.into_iter().collect();
    boundary.sort_by(|&a, &b| parameter_of(a).total_cmp(&parameter_of(b)));
    for x in boundary {
        let q = control.positions[x as usize];
        let sample = evaluate(q);
        let _ = writeln!(
            log,
            "  c{x} {} param {} dist {} (field setback {})",
            p(q),
            up_to(sample.parameter, 2),
            up_to(sample.distance, 3),
            up_to(field.setback_at(sample.parameter), 3)
        );
    }

    let _ = writeln!(
        log,
        "control blend vertices: {} of {}",
        on_source.iter().filter(|&&o| !o).count(),
        on_source.len()
    );
    let mut blend: Vec<i32> = (0..control.positions.len() as i32)
        .filter(|&x| !on_source[x as usize])
        .collect();
    blend.sort_by(|&a, &b| parameter_of(a).total_cmp(&parameter_of(b)));
    for x in blend {
        let q = control.positions[x as usize];
        let sample = evaluate(q);
        let _ = writeln!(
            log,
            "  b{x} {} param {} dist {}",
            p(q),
            up_to(sample.parameter, 2),
            up_to(sample.distance, 3)
        );
    }
}
