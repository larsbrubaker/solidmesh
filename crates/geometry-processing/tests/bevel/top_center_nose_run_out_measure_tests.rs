//! A MEASUREMENT of the Top Center Face nose pick (`top-center-face-nose-r1`) at its two ridge
//! ends, where the ridge round's rail stops on the unpicked lower crease - the side groups and tags
//! of the triangles there, the room and what set it, how far along that crease the band cut's rail
//! reaches, and the corner hole the corner fill leaves open.
//!
//! Asserts nothing. Returns at once unless `BEVEL_NOSE_RUNOUT_MEASURE=1`; writes its reading to the
//! file named by `BEVEL_NOSE_RUNOUT_MEASURE_LOG`, and to the console:
//! `BEVEL_NOSE_RUNOUT_MEASURE=1 cargo nextest run --workspace --test bevel --no-capture
//! top_center_nose_run_out_measure_tests`.

use std::cell::RefCell;
use std::collections::{HashSet, VecDeque};
use std::fmt::Write as _;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use solidmesh::bevel::level_line::corner_fill_fill::WATCH_OPEN_HOLES;
use solidmesh::bevel::level_line::{band_side_groups, chain_on_the_mesh, gap_fill, triangle_edges};
use solidmesh_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;

fn p(v: Vector3) -> String {
    format!("({:.4}, {:.4}, {:.4})", v.x, v.y, v.z)
}

/// Clears the corner fill's open-hole watcher on drop, so a failing test leaves it cleared too.
struct WatchGuard;

impl Drop for WatchGuard {
    fn drop(&mut self) {
        WATCH_OPEN_HOLES.with(|w| *w.borrow_mut() = None);
    }
}

/// The ridge ends read as the run out will see them.
#[test]
fn the_ridge_ends_read_as_the_run_out_will_see_them() {
    if std::env::var("BEVEL_NOSE_RUNOUT_MEASURE").as_deref() != Ok("1") {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load("top-center-face-nose-r1"));
    let positions = &fixture.positions;
    let indices = &fixture.indices;
    let mut log = String::new();
    let n = |t: usize| triangle_edges::normal(positions, indices[t]);
    let feature = |a: i32, b: i32| fixture.is_feature_edge(a, b);

    let chain_edges: Vec<Vec<(i32, i32)>> = fixture
        .chains
        .iter()
        .enumerate()
        .map(|(k, chain)| chain_on_the_mesh::chain_edges(chain, fixture.closed[k]))
        .collect();
    let (tags, groups, undivided) = band_side_groups::group_sides(
        indices,
        &chain_edges,
        &feature,
        true,
        Some(positions),
        None,
        None,
    )
    .expect("the sides group");
    let chains: Vec<String> = fixture
        .chains
        .iter()
        .map(|c| {
            c.iter()
                .map(|v| v.to_string())
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect();
    let _ = writeln!(
        log,
        "chains: {}; size {}, {:?}",
        chains.join(" | "),
        fixture.size,
        profile_of(&fixture)
    );
    let _ = writeln!(
        log,
        "GroupSides: {} groups, undivided chain {undivided}",
        groups.len()
    );
    for group in &groups {
        let sides: Vec<String> = group
            .side_of_chain
            .iter()
            .map(|(k, s)| format!("chain {k} side {s}"))
            .collect();
        let _ = writeln!(
            log,
            "  group tag {}: members {:?}, sides {}, both-sided {:?}, end of {}",
            group.tag,
            group.members,
            sides.join(", "),
            group.both_sided,
            group.end_of
        );
    }

    // The two ridge ends and the lower crease each leaves along: v1 and v9 to the lower crease end
    // v5.
    const LOWER_CREASE_END: i32 = 5;
    let standing = triangle_edges::standing_map(indices);
    for end in [1, 9] {
        let _ = writeln!(log, "ridge end v{end} {}:", p(positions[end as usize]));
        for (t, &(a, b, c)) in indices.iter().enumerate() {
            if a == end || b == end || c == end {
                let _ = writeln!(log, "  t{t} ({a} {b} {c}) n{} tag {}", p(n(t)), tags[t]);
            }
        }

        let crease_triangles = &standing[&triangle_edges::key(end, LOWER_CREASE_END)];
        let distinct: HashSet<i32> = crease_triangles.iter().map(|&t| tags[t as usize]).collect();
        let listed: Vec<String> = crease_triangles
            .iter()
            .map(|&t| format!("t{t} tag {}", tags[t as usize]))
            .collect();
        let _ = writeln!(
            log,
            "  lower crease v{end}-v{LOWER_CREASE_END} (feature {}): {}{}",
            fixture.is_feature_edge(end, LOWER_CREASE_END),
            listed.join(", "),
            if distinct.len() == 1 {
                " - ONE group"
            } else {
                " - different groups"
            }
        );
    }

    type Holes = Rc<RefCell<VecDeque<Vec<(i32, i32)>>>>;
    let holes: Holes = Rc::new(RefCell::new(VecDeque::new()));
    let seen = Rc::clone(&holes);
    let _guard = WatchGuard;
    WATCH_OPEN_HOLES.with(|w| {
        *w.borrow_mut() = Some(Rc::new(move |open: &HashSet<(i32, i32)>| {
            seen.borrow_mut().push_back(open.iter().copied().collect());
        }));
    });

    let options = BandGapBuildOptions {
        budget: Some(default_budget(&fixture, false)),
        ..as_saved(&fixture, fixture.known_room)
    };
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(&fixture, &all, fixture.size, &options);
    let _ = writeln!(
        log,
        "band {:?}, room {:.4} set by {:?}, unflowed room {}",
        gap.refusal,
        gap.room,
        gap.room_set_by,
        gap.unflowed_room
            .map_or("-".to_string(), |r| format!("{r:.4}"))
    );
    if gap.built() {
        {
            let mesh = gap.mesh.as_ref().expect("a built band").borrow();
            for end in [1, 9] {
                let start = positions[end];
                let down = (positions[LOWER_CREASE_END as usize] - start).get_normal();
                let mut on_crease: Vec<(i32, f64)> = (0..mesh.vertex_count())
                    .map(|v| {
                        let from = mesh.position(v) - start;
                        let along = from.dot(down);
                        (v, along, (from - (down * along)).length())
                    })
                    .filter(|&(_, along, off)| along > 1e-9 && off < 1e-4)
                    .map(|(v, along, _)| (v, along))
                    .collect();
                on_crease.sort_by(|a, b| a.1.total_cmp(&b.1));
                let listed: Vec<String> = on_crease
                    .iter()
                    .map(|(v, along)| format!("m{v} at {along:.4}"))
                    .collect();
                let _ = writeln!(
                    log,
                    "  band vertices on the lower crease from v{end}: {}",
                    listed.join(", ")
                );
            }
        }

        let loops: Vec<String> = gap
            .loops
            .iter()
            .map(|l| {
                format!(
                    "{} vertices{}",
                    l.vertices.len(),
                    if l.is_closed { " closed" } else { " open" }
                )
            })
            .collect();
        let _ = writeln!(log, "  loops: {}", loops.join(" | "));
        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        let _ = writeln!(log, "fill {:?}", filled.refusal);
        if let Some(mesh) = &filled.mesh {
            let mesh = mesh.borrow();
            for hole in holes.borrow().iter() {
                let _ = writeln!(log, "corner hole: {} open directed edges", hole.len());
                for &(a, b) in hole {
                    let _ = writeln!(
                        log,
                        "  m{a} {} -> m{b} {}",
                        p(mesh.position(a)),
                        p(mesh.position(b))
                    );
                }
            }
        }
    }

    if let Ok(path) = std::env::var("BEVEL_NOSE_RUNOUT_MEASURE_LOG") {
        if !path.is_empty() {
            std::fs::write(&path, &log).expect("the log writes");
        }
    }

    println!("{log}");
}
