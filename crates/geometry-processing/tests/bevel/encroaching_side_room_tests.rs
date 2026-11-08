//! The owner file "Encroaching Side" as the app route hands it to the kernel (size 3.4, fillet, one
//! open chain), read for which reading names the room and what stands there. A reading, not a
//! verdict: it returns at once unless `BEVEL_ENCROACHING_READ=1`, and asserts nothing;
//! `BEVEL_ENCROACHING_LOG` names a file the reading goes to instead of standard output. Beside it,
//! a Rust-only test derives the pick's room from the fixture's geometry and asserts the kernel
//! reads it.

use std::collections::HashMap;
use std::io::Write;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::{BandGapRefusal, BandProfile, RoomReadingKind};
use solidmesh_math::Vector3;

use crate::level_line_fixture::LevelLineFixture;

/// The room and what names it.
///
/// What it reads:
/// room 3.53553393056 (5 / sqrt 2) set by the FOREIGN reading, the same with the sharp turn at
/// infinity or 30 degrees, and the build at 3.4 succeeds. The wedge at the pick is 45 degrees, so
/// a 3.4 fillet sets back 3.4 / tan 22.5 = 8.21 on the notch wall, past the web's tip v9
/// (-5, -113.5, 10) 7.0711 from the pick. The band reaching that tip would read 2.9289 (7.0711 tan
/// 22.5, refused by size) if concave neighbours were not grown; but the web's two
/// faces meet the wall in CONCAVE creases (5-9, 9-11, 66.9 degrees) and grow over the cut, so
/// what bounds the room is the web's ridge 1-9 leaving the round. The derivation is pinned by
/// `rust_only_encroaching_side_room_is_where_the_grown_web_ridge_leaves_the_round`.
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
    // carries. The faces are kept in first-seen order and the sort is stable, so the listing reads
    // the same every run.
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

/// At most `places` decimals, trailing zeros dropped, so readings stay short.
pub fn up_to(value: f64, places: usize) -> String {
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

/// The least distance between two segments, sampled - enough for a reading.
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

/// The room Encroaching Side's pick truly has, derived from the fixture's own geometry and held
/// against the kernel's reading: 5 / sqrt 2 = 3.5355, set by the web's ridge, and the saved 3.4
/// builds.
///
/// The pick is the vertical edge x = 0, y = -118.5 where the x = 0 face meets the 45-degree notch
/// wall x + y = -118.5 (a 45-degree wedge of material). A web bridges the notch: its two faces
/// meet the notch wall in CONCAVE creases (5-9, 9-11) and each other in the convex ridge 1-9
/// (y = -113.5, z = 10), whose tip v9 stands 7.0711 from the pick on the notch wall. A concave
/// neighbour is GROWN over the cut (the flow decision: the web's faces are extended down to the
/// round, as Onshape does), so the band reaching v9 - at 7.0711 tan 22.5 = 2.9289, the room the
/// reading named before growing creases existed - is not a limit. What is: the grown web's ridge
/// must still pierce the round. Past the size where the whole round passes beyond the ridge line,
/// the grown web would cut clean across the band to the x = 0 face, which no tie can close. In
/// the cross-section (edge at the origin) the round's centre is (-r, r (1 + sqrt 2)) and the
/// ridge is the line y = 5, so it pierces while r (1 + sqrt 2) - 5 <= r, up to r = 5 / sqrt 2.
#[test]
fn rust_only_encroaching_side_room_is_where_the_grown_web_ridge_leaves_the_round() {
    let fixture = Rc::new(LevelLineFixture::load("encroaching-side-r34"));
    let positions = &fixture.positions;
    let profile = BandProfile::Fillet;
    assert!(!fixture.chamfer);

    // Brute force, from the mesh alone: grow the radius and find the largest one at which the
    // ridge line 1-9 still meets the round's cylinder.
    let origin = positions[10];
    let along = (positions[12] - origin).get_normal();
    let in_face = |p: Vector3| {
        let d = p - origin;
        (d - along * d.dot(along)).get_normal()
    };
    let (side_a, side_b) = (in_face(positions[19]), in_face(positions[9]));
    let half_angle = side_a.dot(side_b).clamp(-1.0, 1.0).acos() / 2.0;
    let bisector = (side_a + side_b).get_normal();
    let ridge_start = positions[9];
    let ridge = (positions[1] - positions[9]).get_normal();
    let common = ridge.cross(along).get_normal();
    let pierces = |r: f64| {
        let axis_point = origin + bisector * (r / half_angle.sin());
        (ridge_start - axis_point).dot(common).abs() <= r
    };
    let step = 1e-4;
    let mut last_piercing = 0.0;
    let mut r = step;
    while r < 20.0 {
        if pierces(r) {
            last_piercing = r;
        }
        r += step;
    }
    assert!(
        last_piercing > 0.0,
        "the ridge meets the round at some size"
    );
    let (mut low, mut high) = (last_piercing, last_piercing + step);
    for _ in 0..60 {
        let mid = (low + high) / 2.0;
        if pierces(mid) {
            low = mid;
        } else {
            high = mid;
        }
    }
    let derived = low;
    assert!(
        (derived - 5.0 / 2f64.sqrt()).abs() < 1e-6,
        "brute force {derived} vs closed form 5 / sqrt 2"
    );

    // The band first reaches the web's tip well below that - the wall reading, not the room.
    let tip = (positions[9] - origin) - along * (positions[9] - origin).dot(along);
    let band_meets_the_tip = tip.length() * half_angle.tan();
    // 5 sqrt 2 tan 22.5 = 10 - 5 sqrt 2 = 2.9289.
    assert!((band_meets_the_tip - (10.0 - 5.0 * 2f64.sqrt())).abs() < 1e-6);
    assert!(band_meets_the_tip < fixture.size && fixture.size < derived);

    let lookup = Rc::clone(&fixture);
    let feature: Rc<dyn Fn(i32, i32) -> bool> = Rc::new(move |a, b| lookup.is_feature_edge(a, b));
    for sharp_turn in [f64::INFINITY, 30.0] {
        let reading = band_gap_reading::read_room_of_pick(
            positions,
            &fixture.indices,
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
        assert!(
            (reading.room - derived).abs() < 1e-6,
            "sharp turn {sharp_turn}: the kernel reads {} where the ridge leaves the round at {derived}",
            reading.room
        );
        assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign);

        let build = |size: f64| {
            band_gap_entry_points::build_pick(
                positions,
                &fixture.indices,
                &fixture.chains,
                &fixture.closed,
                Some(Rc::clone(&feature)),
                size,
                profile,
                &BandGapBuildOptions {
                    chord_tolerance: 0.0,
                    snap: 1e-9,
                    sharp_turn_degrees: sharp_turn,
                    ..BandGapBuildOptions::default()
                },
            )
            .expect("the pick builds or refuses by name")
        };
        let saved = build(fixture.size);
        assert_eq!(saved.refusal, BandGapRefusal::None, "the saved 3.4 builds");
        let past = build(derived * 1.1);
        assert_eq!(
            past.refusal,
            BandGapRefusal::Size,
            "1.1 x the room is refused by size"
        );
        assert!((past.room - derived).abs() < 1e-6);
    }
}
