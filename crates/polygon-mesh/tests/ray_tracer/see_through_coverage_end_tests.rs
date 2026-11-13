//! The coverage of a camera line ended at the floor where the line's end lies inside a part the
//! walk did not meet before the end, on the CPU (`RenderScene::coverage_along`) and on the GPU
//! route's mirror (`coverage_along_f32`), line for line: a see-through part resting on an opaque
//! part sunk through the floor, with a see-through part inside that one below the floor, on an
//! inside-out opaque part, and on two overlapping opaque parts; parts wholly below the floor,
//! parts straddling it, parts whose base or top lies in its plane. Each count of lines by coverage
//! is the one measured on the CPU in solidmesh 0.3.0 unless the case says why it differs.

use std::collections::BTreeMap;

use solidmesh_mesh::ray_tracer::ScatteringSurface;
use solidmesh_mesh::Mesh;

use crate::glass_tray_support::box_meshes;
use crate::see_through_coverage_tests::{both_coverages, camera_lines, scene_and_pack};

/// A part: its mesh in place, its surface and its opacity.
type Part = (Mesh, ScatteringSurface, f64);

const SIZE: [f64; 3] = [40.0, 40.0, 20.0];

fn cube(size: [f64; 3], center: [f64; 3]) -> Mesh {
    box_meshes(&[(size, center)])[0].clone()
}

/// `mesh` with every face wound the other way: the same closed surface, its normals inward.
fn inside_out(mesh: &Mesh) -> Mesh {
    let vertices: Vec<f64> = mesh
        .vertices
        .iter()
        .flat_map(|p| [f64::from(p.x), f64::from(p.y), f64::from(p.z)])
        .collect();
    let faces: Vec<i32> = mesh
        .faces
        .iter()
        .flat_map(|face| [face.v0, face.v2, face.v1])
        .collect();
    Mesh::from_arrays(&vertices, &faces)
}

fn half() -> ScatteringSurface {
    ScatteringSurface::matte_plastic([0.6; 3])
}

fn grey() -> ScatteringSurface {
    ScatteringSurface::matte_plastic([0.3; 3])
}

/// The see-through part (40 by 40 by 20 at opacity 0.5) resting on a support whose top is at
/// z = 20, lifted `lift` off it.
fn top_part(lift: f64) -> Part {
    (cube(SIZE, [0.0, 0.0, 30.0 + lift]), half(), 0.5)
}

/// Every camera line's coverage of `parts`, on the CPU and on the card's mirror.
fn coverages(parts: &[Part]) -> (Vec<f64>, Vec<f64>) {
    let (scene, pack) = scene_and_pack(parts);
    camera_lines()
        .iter()
        .map(|line| both_coverages(&scene, &pack, line))
        .unzip()
}

/// How many lines have each coverage, by its shortest text.
fn counts(values: &[f64]) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    for value in values {
        *counts.entry(format!("{value}")).or_insert(0) += 1;
    }
    counts
}

fn expected(pairs: &[(&str, usize)]) -> BTreeMap<String, usize> {
    pairs
        .iter()
        .map(|&(value, count)| (value.to_string(), count))
        .collect()
}

/// Measures `parts` and records in `failures` where the CPU and the card's mirror differ on a
/// line, where the CPU's counts are not `want`, and, given `lifted`, where the CPU or the card
/// differs from that scene on a line.
fn judge(
    failures: &mut Vec<String>,
    case: &str,
    parts: &[Part],
    lifted: Option<&[Part]>,
    want: &[(&str, usize)],
) {
    let (cpu, card) = coverages(parts);
    let differing = cpu.iter().zip(&card).filter(|(a, b)| a != b).count();
    println!(
        "{case}: cpu {:?}, card {:?}, lines differing {differing}",
        counts(&cpu),
        counts(&card)
    );
    if differing != 0 {
        failures.push(format!(
            "{case}: the card and the CPU differ on {differing} lines"
        ));
    }
    if counts(&cpu) != expected(want) {
        failures.push(format!("{case}: cpu {:?}, want {want:?}", counts(&cpu)));
    }
    if let Some(lifted) = lifted {
        let (cpu_lifted, card_lifted) = coverages(lifted);
        let cpu_apart = cpu.iter().zip(&cpu_lifted).filter(|(a, b)| a != b).count();
        let card_apart = card
            .iter()
            .zip(&card_lifted)
            .filter(|(a, b)| a != b)
            .count();
        if (cpu_apart, card_apart) != (0, 0) {
            failures.push(format!(
                "{case}: resting and lifted differ on {cpu_apart} lines on the CPU, \
                 {card_apart} on the card"
            ));
        }
    }
}

/// What 0.3.0 measured for the see-through part resting on a 40 by 40 support from the floor
/// up: lines missing both, lines through the see-through part only, and lines reaching the support.
const ON_A_SUPPORT: [(&str, usize); 3] = [("0", 706), ("0.5", 253), ("1", 1345)];

#[test]
fn a_line_ending_inside_an_opaque_part_it_entered_past_a_resting_part_is_covered() {
    let mut failures = Vec::new();
    let sunk = (cube([40.0, 40.0, 30.0], [0.0, 0.0, 5.0]), grey(), 1.0);
    let inner = (cube([30.0, 30.0, 4.0], [0.0, 0.0, -5.0]), grey(), 0.5);
    let overlapping = (cube([30.0, 30.0, 10.0], [0.0, 0.0, -10.0]), grey(), 1.0);
    let inverted = (inside_out(&cube(SIZE, [0.0, 0.0, 10.0])), grey(), 1.0);
    let cases: [(&str, Vec<Part>); 4] = [
        ("sunk through the floor", vec![sunk.clone()]),
        (
            "sunk, a see-through part inside it below the floor",
            vec![sunk.clone(), inner],
        ),
        (
            "sunk, an opaque part overlapping it below the floor",
            vec![sunk, overlapping],
        ),
        ("an inside-out box on the floor", vec![inverted]),
    ];
    for (case, under) in cases {
        for support_first in [false, true] {
            let arrange = |lift: f64| {
                let mut parts = vec![top_part(lift)];
                parts.extend(under.iter().cloned());
                if support_first {
                    parts.reverse();
                }
                parts
            };
            judge(
                &mut failures,
                &format!("{case}, support first {support_first}"),
                &arrange(0.0),
                Some(&arrange(0.01)),
                &ON_A_SUPPORT,
            );
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
fn a_part_wholly_below_the_floor_counts_nothing() {
    // 0.3.0 traced every stretch past a see-through part to infinity and counted these parts on
    // every line through the see-through part (468 lines at 1 under the small box, 1,044 at 1 under
    // the inside-out one and 1,044 at 0.75 over the see-through one); 0.3.1 ends the line at the
    // floor.
    let mut failures = Vec::new();
    let on_floor = (cube(SIZE, [0.0, 0.0, 10.0]), half(), 0.5);
    let cases = [
        (
            "an opaque box under a small see-through box",
            vec![
                (cube([20.0; 3], [0.0, 0.0, 10.0]), half(), 0.5),
                (cube([40.0, 40.0, 10.0], [0.0, 0.0, -20.0]), grey(), 1.0),
            ],
            [("0", 1836), ("0.5", 468)].as_slice(),
        ),
        (
            "an inside-out opaque box",
            vec![
                on_floor.clone(),
                (inside_out(&cube(SIZE, [0.0, 0.0, -20.0])), grey(), 1.0),
            ],
            [("0", 959), ("0.5", 1345)].as_slice(),
        ),
        (
            "a see-through box",
            vec![on_floor, (cube(SIZE, [0.0, 0.0, -20.0]), grey(), 0.5)],
            [("0", 959), ("0.5", 1345)].as_slice(),
        ),
    ];
    for (case, parts, want) in cases {
        judge(&mut failures, case, &parts, None, want);
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
fn parts_straddling_the_floor_or_lying_on_it_count_as_they_did() {
    let mut failures = Vec::new();
    let on_floor = (cube(SIZE, [0.0, 0.0, 10.0]), half(), 0.5);
    // A see-through part straddling the floor counts once; one whose base lies on the floor is
    // decided once, at its top or at its base; a see-through part inside another on the floor
    // counts once more and never twice (0.75, not more): each as in 0.3.0.
    judge(
        &mut failures,
        "a see-through box straddling the floor",
        &[(cube(SIZE, [0.0, 0.0, 5.0]), half(), 0.5)],
        None,
        &[("0", 1084), ("0.5", 1220)],
    );
    judge(
        &mut failures,
        "a see-through box on the floor",
        std::slice::from_ref(&on_floor),
        None,
        &[("0", 959), ("0.5", 1345)],
    );
    judge(
        &mut failures,
        "a see-through box on the floor inside another",
        &[
            on_floor.clone(),
            (cube([20.0, 20.0, 10.0], [0.0, 0.0, 5.0]), grey(), 0.5),
        ],
        None,
        &[("0", 959), ("0.5", 969), ("0.75", 376)],
    );
    // An opaque box straddling the floor under a see-through box on the floor, overlapping it: a
    // line meets the opaque top above the floor.
    judge(
        &mut failures,
        "an opaque box straddling the floor under a see-through box on it",
        &[
            on_floor.clone(),
            (cube(SIZE, [0.0, 0.0, -5.0]), grey(), 1.0),
        ],
        None,
        &[("0", 959), ("0.5", 225), ("1", 1120)],
    );
    // A scene of fully there parts stops each line at its first face, as before: an opaque box on
    // a clear glass box, and an opaque box sunk through the floor.
    judge(
        &mut failures,
        "an opaque box on a clear glass box",
        &[
            (cube(SIZE, [0.0, 0.0, 30.0]), grey(), 1.0),
            (
                cube(SIZE, [0.0, 0.0, 10.0]),
                ScatteringSurface::glass(),
                1.0,
            ),
        ],
        None,
        &[("0", 706), ("1", 1598)],
    );
    judge(
        &mut failures,
        "an opaque box sunk through the floor",
        &[(cube(SIZE, [0.0, 0.0, 0.0]), grey(), 1.0)],
        None,
        &[("0", 1149), ("1", 1155)],
    );
    // An opaque box wholly below the floor with its top in the floor's plane, under a see-through
    // box on the floor: the line's end lies on both. A line through both counts the opaque box,
    // line for line as in 0.3.0, whichever face the trace returns first at the end: a face within
    // its surface offset of the end lies at the end, so the opaque top met there covers, and a walk
    // past the end that passed the top with the see-through base crosses only the opaque bottom.
    for under_first in [false, true] {
        let mut parts = vec![
            on_floor.clone(),
            (cube(SIZE, [0.0, 0.0, -10.0]), grey(), 1.0),
        ];
        if under_first {
            parts.reverse();
        }
        judge(
            &mut failures,
            &format!("an opaque box with its top in the floor's plane, under first {under_first}"),
            &parts,
            None,
            &[("0", 959), ("0.5", 225), ("1", 1120)],
        );
    }
    // A see-through box on a see-through box sunk through the floor, with a third inside that one
    // below the floor: the sunk box counts as it does lifted, the one below the floor not at all
    // (0.3.0 counted that one too: 0.875 on 728 lines).
    let sunk = (cube([40.0, 40.0, 30.0], [0.0, 0.0, 5.0]), grey(), 0.5);
    let inner = (cube([30.0, 30.0, 4.0], [0.0, 0.0, -5.0]), grey(), 0.5);
    for support_first in [false, true] {
        let arrange = |lift: f64| {
            let mut parts = vec![top_part(lift), sunk.clone(), inner.clone()];
            if support_first {
                parts.reverse();
            }
            parts
        };
        judge(
            &mut failures,
            &format!("see-through on see-through sunk, support first {support_first}"),
            &arrange(0.0),
            Some(&arrange(0.01)),
            &[("0", 706), ("0.5", 253), ("0.75", 1345)],
        );
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
