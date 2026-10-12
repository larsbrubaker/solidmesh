//! Ports `Tests/GeometryProcessingTests/LevelLine/EncroachingSideRoomTests.cs`: the owner file
//! "Encroaching Side" as the app route hands it to the kernel (size 3.4, fillet, one open chain),
//! read for which reading names the room and what stands there. A reading, not a verdict: it
//! returns at once unless `BEVEL_ENCROACHING_READ=1`, and asserts nothing; `BEVEL_ENCROACHING_LOG`
//! names a file the reading goes to instead of standard output.

use std::collections::HashMap;
use std::io::Write;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::BandProfile;
use vector_math::Vector3;

use crate::level_line_fixture::LevelLineFixture;

/// C# `TheRoomAndWhatNamesIt`.
#[test]
fn the_room_and_what_names_it() {
    if std::env::var("BEVEL_ENCROACHING_READ").as_deref() != Ok("1") {
        return;
    }

    let mut out: Box<dyn Write> = match std::env::var("BEVEL_ENCROACHING_LOG") {
        Ok(path) if !path.is_empty() => {
            Box::new(std::fs::File::create(&path).unwrap_or_else(|e| panic!("{path}: {e}")))
        }
        _ => Box::new(std::io::stdout()),
    };
    read(&mut out).expect("the reading is written");
}

/// C# `Read`.
fn read(out: &mut dyn Write) -> std::io::Result<()> {
    let fixture = Rc::new(LevelLineFixture::load("encroaching-side-r34"));
    let positions = &fixture.positions;
    let indices = &fixture.indices;
    let chain = &fixture.chains[0];
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let lookup = Rc::clone(&fixture);
    let feature: Rc<dyn Fn(i32, i32) -> bool> = Rc::new(move |a, b| lookup.is_feature_edge(a, b));
    let chain_text: Vec<String> = chain.iter().map(|v| v.to_string()).collect();
    writeln!(
        out,
        "size {}, profile {profile:?}, chain {}",
        fixture.size,
        chain_text.join(" ")
    )?;

    for sharp_turn in [f64::INFINITY, 30.0] {
        let reading = band_gap_reading::read_room_of_pick(
            positions,
            indices,
            &fixture.chains,
            &fixture.closed,
            Some(Rc::clone(&feature)),
            profile,
            1e-9,
            None,
            100.0,
            0.0,
            sharp_turn,
        )
        .expect("the room reads");
        writeln!(
            out,
            "sharp turn {sharp_turn}: room {}, set by {:?}, offending edge {:?}, vertex {}",
            reading.room, reading.room_set_by, reading.offending_edge, reading.offending_vertex
        )?;
        for candidate in &reading.candidates {
            writeln!(
                out,
                "    candidate {:?} room {}",
                candidate.source, candidate.room
            )?;
        }

        let gap = band_gap_entry_points::build_pick(
            positions,
            indices,
            &fixture.chains,
            &fixture.closed,
            Some(Rc::clone(&feature)),
            fixture.size,
            profile,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                sharp_turn_degrees: sharp_turn,
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the pick builds or refuses by name");
        writeln!(
            out,
            "    build at {}: {:?}, room {}, set by {:?}, edge {:?}, vertex {}",
            fixture.size,
            gap.refusal,
            gap.room,
            gap.room_set_by,
            gap.offending_edge,
            gap.offending_vertex
        )?;
    }

    // Every feature edge not on the chain, by its distance from the picked edge and the turn it
    // carries. The faces are kept in first-seen order (C#'s Dictionary), and the sort is stable
    // (C#'s OrderBy).
    let a = positions[chain[0] as usize];
    let b = positions[chain[chain.len() - 1] as usize];
    let mut faces: Vec<((i32, i32), Vec<usize>)> = Vec::new();
    let mut slot: HashMap<(i32, i32), usize> = HashMap::new();
    for (t, &(i, j, k)) in indices.iter().enumerate() {
        for (p, q) in [(i, j), (j, k), (k, i)] {
            let key = (p.min(q), p.max(q));
            let at = *slot.entry(key).or_insert_with(|| {
                faces.push((key, Vec::new()));
                faces.len() - 1
            });
            faces[at].1.push(t);
        }
    }

    let distance_of =
        |key: (i32, i32)| distance(positions[key.0 as usize], positions[key.1 as usize], a, b);
    faces.sort_by(|x, y| distance_of(x.0).total_cmp(&distance_of(y.0)));

    for ((low, high), tris) in &faces {
        let (low, high) = (*low, *high);
        if !fixture.is_feature_edge(low, high) || tris.len() != 2 {
            continue;
        }

        let n0 = normal(positions, indices[tris[0]]);
        let n1 = normal(positions, indices[tris[1]]);
        let turn = n0.dot(n1).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI;
        let (p0, q0) = (positions[low as usize], positions[high as usize]);
        let other = other_vertex(indices[tris[1]], low, high);
        let convex = normal(positions, indices[tris[0]]).dot(positions[other as usize] - p0) < 0.0;
        writeln!(
            out,
            "    feature {low}-{high} {p0:?} .. {q0:?}: {} from the pick, turn {} deg, {}",
            up_to(distance(p0, q0, a, b), 4),
            up_to(turn, 1),
            if convex { "convex" } else { "concave" }
        )?;
    }

    Ok(())
}

/// C#'s `0.####` / `0.#` formats: at most `places` decimals, trailing zeros dropped.
fn up_to(value: f64, places: usize) -> String {
    let text = format!("{value:.places$}");
    if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    }
}

fn normal(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    let pa = positions[t.0 as usize];
    (positions[t.1 as usize] - pa)
        .cross(positions[t.2 as usize] - pa)
        .get_normal()
}

fn other_vertex(t: (i32, i32, i32), low: i32, high: i32) -> i32 {
    [t.0, t.1, t.2]
        .into_iter()
        .find(|&v| v != low && v != high)
        .expect("a triangle has a third vertex")
}

/// C# `Distance`: the least distance between two segments, sampled - enough for a reading.
fn distance(p: Vector3, q: Vector3, a: Vector3, b: Vector3) -> f64 {
    let mut best = f64::MAX;
    let axis = b - a;
    for s in 0..=200 {
        let x = p + (q - p) * (s as f64 / 200.0);
        let u = ((x - a).dot(axis) / axis.length_squared()).clamp(0.0, 1.0);
        best = best.min((x - (a + axis * u)).length());
    }

    best
}
