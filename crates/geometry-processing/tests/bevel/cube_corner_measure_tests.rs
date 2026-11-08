//! The second MEASUREMENT taken before the corner construction was built - two adjacent cube faces
//! picked as the app links them (the shared top-front edge, the rest of the top rim and the rest of
//! the front rim: three open chains meeting at two corners of three ends), which the corner reading
//! now lets through, so what the rest of the build does at a corner can be read.
//!
//! Asserts nothing. Returns at once unless `STAGE3_MEASURE=1`; appends its reading to the file
//! named by `STAGE3_MEASURE_LOG`, or to the system temp folder, and prints it:
//! `STAGE3_MEASURE=1 cargo nextest run --workspace --test bevel --no-capture
//! cube_corner_measure_tests`. It runs alone (a `threads-required` override in
//! `.config/nextest.toml`).

use std::fmt::Write as _;
use std::io::Write as _;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{
    band_gap_reading, band_side_groups, chain_on_the_mesh, gap_loop_walk, BandProfile, ChainField,
    PickField,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::cube_solid;
use crate::several_chains_fixtures;

const SNAP: f64 = 1e-9;

/// Two adjacent cube faces read at their corners.
#[test]
fn two_adjacent_cube_faces_read_at_their_corners() {
    if std::env::var("STAGE3_MEASURE").as_deref() != Ok("1") {
        return;
    }

    // Half 1, so the edge is 2 and a tenth of it is 0.2. Top face z=+1 is 4,5,6,7; front y=-1 is
    // 0,1,5,4.
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
    let chains = vec![vec![4, 5], vec![5, 6, 7, 4], vec![5, 1, 0, 4]];
    let closed = [false, false, false];
    const SIZE: f64 = 0.2;

    let mut log = String::new();
    let _ = writeln!(
        log,
        "=== {:?} cube half 1 (edge 2), size {SIZE}; chains 0 = 4-5 (top-front), 1 = 5-6-7-4 (top rest), 2 = 5-1-0-4 (front rest)",
        std::time::SystemTime::now()
    );

    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let _ = writeln!(log, "##### profile {profile:?}");
        measure(
            &positions, &indices, &chains, &closed, &feature, SIZE, profile, &mut log,
        );
    }

    let path = match std::env::var("STAGE3_MEASURE_LOG") {
        Ok(path) if !path.is_empty() => std::path::PathBuf::from(path),
        _ => std::env::temp_dir().join("stage3-measure.log"),
    };
    std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .and_then(|mut file| file.write_all(log.as_bytes()))
        .expect("the log appends");
    println!("{log}");
}

fn p(v: Vector3) -> String {
    format!("({:.4}, {:.4}, {:.4})", v.x, v.y, v.z)
}

#[allow(clippy::too_many_arguments)]
fn measure(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    feature: &FeatureEdgeTest,
    size: f64,
    profile: BandProfile,
    log: &mut String,
) {
    // The side groups, read the way the build reads them.
    let chain_edges: Vec<Vec<(i32, i32)>> = chains
        .iter()
        .enumerate()
        .map(|(k, c)| chain_on_the_mesh::chain_edges(c, closed[k]))
        .collect();
    let (_, groups, undivided) = band_side_groups::group_sides(
        indices,
        &chain_edges,
        feature.as_ref(),
        true,
        None,
        None,
        None,
    )
    .expect("the sides group");
    let _ = writeln!(
        log,
        "GroupSides: {} groups, undivided {undivided}",
        groups.len()
    );
    for g in &groups {
        let sides: Vec<String> = g
            .side_of_chain
            .iter()
            .map(|(k, s)| format!("chain {k} side {s}"))
            .collect();
        let _ = writeln!(
            log,
            "  group tag {}: members {:?}, sides {{{}}}",
            g.tag,
            g.members,
            sides.join(", ")
        );
    }

    // The room, every candidate named.
    let reading = band_gap_reading::read_room_of_pick(
        positions,
        indices,
        chains,
        closed,
        Some(Rc::clone(feature)),
        profile,
        SNAP,
        None,
        100.0,
        0.0,
        f64::INFINITY,
    )
    .expect("the room reads");
    let _ = writeln!(
        log,
        "ReadRoom: room {}, set by {:?}, size statement {}, apart {}, divides {}",
        reading.room,
        reading.room_set_by,
        reading.is_a_size_statement(),
        reading.chains_are_apart,
        reading.chain_divides_sides
    );
    let mut candidates: Vec<_> = reading.candidates.iter().collect();
    candidates.sort_by(|a, b| a.room.total_cmp(&b.room));
    for c in candidates {
        let _ = writeln!(
            log,
            "  candidate {:?} room {}{}",
            c.source,
            c.room,
            if c.room <= 1e-9 { "   <-- ZERO" } else { "" }
        );
    }

    // The ordinary build, its own room read.
    let plain = band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        Some(Rc::clone(feature)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");
    let _ = writeln!(
        log,
        "Build (room read by the build): {:?}, room {}, set by {:?}, pinch probes {}, pinches everywhere {}",
        plain.refusal,
        plain.room,
        plain.room_set_by,
        plain.pinch_probes,
        plain.pinches_at_every_size_tried
    );

    // The build with the room handed in, so the cut, the walk and the owned-loop test are reached.
    let built = band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        Some(Rc::clone(feature)),
        size,
        profile,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: SNAP,
            known_room: Some(1.0),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick");
    let _ = writeln!(
        log,
        "Build (knownRoom 1.0): {:?}, room {}, pinch probes {}, pinches everywhere {}, loops {}",
        built.refusal,
        built.room,
        built.pinch_probes,
        built.pinches_at_every_size_tried,
        built.loops.len()
    );
    for row in &built.side_loops {
        let _ = writeln!(
            log,
            "  side row: chain {} side {} group {} loops {} closed {} mixed {} met {:?} owed {}",
            row.chain,
            row.side,
            row.group_tag,
            row.loops,
            row.closed_loops,
            row.mixed_loops,
            row.met_chains,
            row.owed
        );
    }

    let Some(mesh) = built.mesh.as_ref() else {
        return;
    };
    if built.loops.is_empty() {
        return;
    }

    let _ = writeln!(
        log,
        "  EveryChainSideLeavesOneOwnedLoop: {}",
        gap_loop_walk::every_chain_side_leaves_one_owned_loop(
            &groups,
            closed,
            &built.loops,
            Some(chains)
        )
    );

    // Every chain's own field at this size, so the owner at a loop vertex is read as the build
    // reads it.
    let fields: Vec<Rc<ChainField>> = chains
        .iter()
        .enumerate()
        .map(|(k, c)| {
            let points: Vec<Vector3> = c.iter().map(|&v| positions[v as usize]).collect();
            let setbacks: Vec<f64> = chain_on_the_mesh::unit_setbacks(
                positions,
                indices,
                c,
                closed[k],
                &chain_edges[k],
                profile,
            )
            .expect("every chain edge stands on two triangles")
            .into_iter()
            .map(|u| u * size)
            .collect();
            Rc::new(ChainField::from_polyline(&points, closed[k], &setbacks).expect("a chain"))
        })
        .collect();

    let mesh = mesh.borrow();
    let mut sides: Vec<i32> = built.loops.iter().map(|l| l.side).collect();
    sides.sort_unstable();
    sides.dedup();
    for side in sides {
        let group_loops: Vec<_> = built.loops.iter().filter(|l| l.side == side).collect();
        let members: Vec<i32> = groups
            .iter()
            .find(|g| g.tag == side)
            .map(|g| g.members.clone())
            .unwrap_or_default();
        let _ = writeln!(
            log,
            "  group tag {side} {members:?}: {} loop(s)",
            group_loops.len()
        );
        let Ok(pick) = PickField::new(fields.clone(), &members) else {
            // A group with no members has no pick field.
            continue;
        };
        for l in group_loops {
            let verts = &l.vertices;
            let owners: Vec<i32> = verts
                .iter()
                .map(|&v| pick.evaluate(mesh.position(v)).expect("no budget").chain)
                .collect();
            let mut rle = Vec::new();
            let mut i = 0;
            while i < owners.len() {
                let mut j = i;
                while j < owners.len() && owners[j] == owners[i] {
                    j += 1;
                }

                rle.push(format!("{}x{}", owners[i], j - i));
                i = j;
            }

            let _ = writeln!(
                log,
                "    loop: {}, {} vertices, owner (build) {}, mixed {}, owners {:?}, sequence {}",
                if l.is_closed { "closed" } else { "open" },
                verts.len(),
                l.chain,
                l.has_mixed_owners,
                l.owners,
                rle.join(" ")
            );

            // Each switch: the segment between two consecutive vertices of different owners, and at
            // each end how far apart the two chains' values are - a tie at a vertex puts the switch
            // ON it.
            let steps = if l.is_closed {
                verts.len()
            } else {
                verts.len() - 1
            };
            for i in 0..steps {
                let a = verts[i];
                let b = verts[(i + 1) % verts.len()];
                let oa = owners[i];
                let ob = owners[(i + 1) % verts.len()];
                if oa == ob {
                    continue;
                }

                let end = |v: i32| {
                    let at = mesh.position(v);
                    let va = fields[oa as usize].evaluate(at).expect("no budget").value;
                    let vb = fields[ob as usize].evaluate(at).expect("no budget").value;
                    format!(
                        "v{v}{} {} f{oa}={va:.3e} f{ob}={vb:.3e} |diff|={:.3e}",
                        if (v as usize) < positions.len() {
                            " (INPUT)"
                        } else {
                            ""
                        },
                        p(at),
                        (va - vb).abs()
                    )
                };

                let _ = writeln!(
                    log,
                    "      switch {oa}->{ob} on segment [{}] -> [{}]",
                    end(a),
                    end(b)
                );
            }

            // Bisector ties anywhere on the loop: a loop vertex where two members read the same
            // value.
            for &v in verts {
                let at = mesh.position(v);
                let mut values: Vec<(i32, f64)> = members
                    .iter()
                    .map(|&k| (k, fields[k as usize].evaluate(at).expect("no budget").value))
                    .collect();
                // A stable sort: equal values keep the members' order.
                values.sort_by(|a, b| a.1.total_cmp(&b.1));
                if values.len() >= 2 && (values[0].1 - values[1].1).abs() <= 1e-9 {
                    let _ = writeln!(
                        log,
                        "      TIE at v{v}{} {}: chains {},{} value {:.3e}",
                        if (v as usize) < positions.len() {
                            " (INPUT)"
                        } else {
                            ""
                        },
                        p(at),
                        values[0].0,
                        values[1].0,
                        values[0].1
                    );
                }
            }
        }
    }
}
