//! Measured before the corner filler became the default - where a fillet pick's volume goes. Every
//! added triangle carries one tag, so the fill is split from the strips by the corner cap's
//! hole-fill watch and held between two bounds on its own loop: a fan to the loop's centroid (a
//! sharp-ish cap) and the same patch pushed radially onto the nearest strip end's extended cylinder
//! (the strips carried on into the corner). Each chain is also built alone and read against its
//! exact straight prism (length x r^2 (tan(phi/2) - phi/2) per edge), so the difference is what
//! that chain's ends do.
//!
//! Gated: `BEVEL_CORNER_VOLUME=1` (`BEVEL_CORNER_VOLUME_OUT` appends to a file). Prints only:
//! `BEVEL_CORNER_VOLUME=1 cargo nextest run --workspace --test bevel --profile measure
//! --no-capture corner_volume_measure_tests`. It runs alone (a `threads-required` override in
//! `.config/nextest.toml`). Each test's cases run as a loop inside it.

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{
    band_gap_reading, BandGapResult, BandProfile, CornerCapLastRow, GapFillRefusal,
    HoleFillTriangle, LevelLineMesh,
};
use solidmesh_math::Vector3;

use crate::end_face_builds::feature_of;
use crate::hole_fill_corner_tests::{fill_with_the_filler, fill_with_the_filler_watched};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{cube_solid, signed_volume, signed_volume_of};
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

const RUN_VARIABLE: &str = "BEVEL_CORNER_VOLUME";

/// One filled hole: its fill's points, its loop's length, its triangles and its strip ends.
type Hole = (Vec<Vector3>, usize, Vec<HoleFillTriangle>, Vec<Row>);

/// The pick's build at the measured size, for any of its chains.
struct Pick {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chains: Vec<Vec<i32>>,
    closed: Vec<bool>,
    feature: FeatureEdgeTest,
    size: f64,
    segments: i32,
    fixture: Option<Rc<LevelLineFixture>>,
}

impl Pick {
    fn build(&self, chains: &[Vec<i32>], closed: &[bool]) -> BandGapResult {
        let options = match &self.fixture {
            None => BandGapBuildOptions::default(),
            Some(fixture) => BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                known_room: if chains.len() == fixture.chains.len() {
                    fixture.known_room
                } else {
                    None
                },
                budget: None,
                seam_epsilon: fixture.seam_epsilon,
                sharp_turn_degrees: fixture.sharp_turn_degrees,
                fillet_segments: fixture.segments,
            },
        };
        band_gap_entry_points::build_pick(
            &self.positions,
            &self.indices,
            chains,
            closed,
            Some(Rc::clone(&self.feature)),
            self.size,
            BandProfile::Fillet,
            &options,
        )
        .expect("a valid pick")
    }
}

fn pick(name: &str) -> Pick {
    if name == "cube-two-faces" {
        let (positions, indices) = cube_solid(1.0);
        let feature =
            feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
        return Pick {
            positions,
            indices,
            chains: vec![vec![4, 5], vec![5, 6, 7, 4], vec![5, 1, 0, 4]],
            closed: vec![false, false, false],
            feature,
            size: 0.5,
            segments: 16,
            fixture: None,
        };
    }

    let fixture = Rc::new(LevelLineFixture::load(name));
    let feature = feature_of(&fixture);
    let size = if name == "outer-faces-r04" {
        0.5 * band_gap_reading::room_of_pick(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            &fixture.closed,
            Some(Rc::clone(&feature)),
            BandProfile::Fillet,
            1e-9,
        )
        .expect("the room reads")
    } else {
        fixture.size
    };
    Pick {
        positions: fixture.positions.clone(),
        indices: fixture.indices.clone(),
        chains: fixture.chains.clone(),
        closed: fixture.closed.clone(),
        feature,
        size,
        segments: fixture.segments,
        fixture: Some(fixture),
    }
}

/// Where the volume goes, on inner-right-sides-and-center-op0, outer-faces-r04 and
/// cube-two-faces.
#[test]
fn where_the_volume_goes() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        return;
    }

    for name in [
        "inner-right-sides-and-center-op0",
        "outer-faces-r04",
        "cube-two-faces",
    ] {
        where_the_volume_goes_row(name);
    }
}

fn where_the_volume_goes_row(name: &str) {
    let p = pick(name);
    let (positions, indices, chains, closed, size) =
        (&p.positions, &p.indices, &p.chains, &p.closed, p.size);
    let source = signed_volume_of(positions, indices);
    let holes: Rc<RefCell<Vec<Hole>>> = Rc::new(RefCell::new(Vec::new()));
    let seen = Rc::clone(&holes);
    let gap = p.build(chains, closed);
    if !gap.built() {
        log(&format!("{name} at {size:.3}: band {:?}", gap.refusal));
        return;
    }

    let filled = fill_with_the_filler_watched(
        &gap,
        BandProfile::Fillet,
        p.segments,
        Some(Rc::new(
            move |mesh: &LevelLineMesh,
                  fill: &[Vector3],
                  n: i32,
                  triangles: &[HoleFillTriangle],
                  rows: &[CornerCapLastRow]| {
                seen.borrow_mut().push((
                    fill.to_vec(),
                    n as usize,
                    triangles.to_vec(),
                    rows.iter().map(|r| Row::new(mesh, r)).collect(),
                ))
            },
        )),
    );

    if filled.refusal != GapFillRefusal::None {
        log(&format!("{name} at {size:.3}: fill {:?}", filled.refusal));
        return;
    }

    let filled_mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let removed = source - signed_volume(&filled_mesh);
    let holes = holes.borrow();
    log(&format!(
        "{name} fillet {size:.3}: source {source:.3}, removed {removed:.4}, {} filled holes",
        holes.len()
    ));

    // Signed volumes of the patch against each reference over the same loop: positive keeps MORE
    // material than the reference (stands proud of it), so removes less.
    let mut patch_over_fan = 0.0;
    let mut cylinder_over_fan = 0.0;
    for (h, (fill, n, triangles, rows)) in holes.iter().enumerate() {
        let n = *n;
        let centroid = centroid_of(&fill[..n]);
        let patch: f64 = triangles
            .iter()
            .map(|t| tet(fill[t.a as usize], fill[t.b as usize], fill[t.c as usize]))
            .sum();
        let fan: f64 = (0..n)
            .map(|i| tet(centroid, fill[i], fill[(i + 1) % n]))
            .sum();
        let on_cylinders: Vec<Vector3> = fill
            .iter()
            .enumerate()
            .map(|(i, &q)| {
                if i < n {
                    q
                } else {
                    onto_nearest_cylinder(q, rows)
                }
            })
            .collect();
        let cylinder: f64 = triangles
            .iter()
            .map(|t| {
                tet(
                    on_cylinders[t.a as usize],
                    on_cylinders[t.b as usize],
                    on_cylinders[t.c as usize],
                )
            })
            .sum();
        patch_over_fan += patch - fan;
        cylinder_over_fan += cylinder - fan;
        let radius = if rows.is_empty() {
            f64::NAN
        } else {
            rows.iter().map(|r| r.radius).sum::<f64>() / rows.len() as f64
        };
        let per_row: Vec<String> = rows
            .iter()
            .map(|r| {
                let proud: Vec<f64> = if fill.len() > n {
                    fill[n..].iter().map(|&q| r.proud(q)).collect()
                } else {
                    vec![0.0]
                };
                format!(
                    "{} r {:.3} proud max {:.4} min {:.4}",
                    if r.concave { "concave" } else { "convex" },
                    r.radius,
                    max_of(&proud),
                    min_of(&proud)
                )
            })
            .collect();
        let nearest_each: Vec<f64> = if fill.len() > n {
            fill[n..]
                .iter()
                .map(|&q| {
                    if rows.is_empty() {
                        f64::NAN
                    } else {
                        min_of(&rows.iter().map(|r| r.proud(q).abs()).collect::<Vec<_>>())
                    }
                })
                .collect()
        } else {
            vec![0.0]
        };
        let nearest = max_of(&nearest_each);
        let place = if (cylinder - fan).abs() < 1e-12 {
            f64::NAN
        } else {
            (patch - fan) / (cylinder - fan)
        };

        // The loop's own plane (Newell, wound as the cap lays it, so it points out of the part):
        // how far the loop leaves it, and how far the inner vertices stand out of it (positive keeps
        // material).
        let mut newell = Vector3::ZERO;
        for i in 0..n {
            newell += (fill[i] - centroid).cross(fill[(i + 1) % n] - centroid);
        }
        let newell = newell.get_normal();
        let loop_off = max_of(
            &fill[..n]
                .iter()
                .map(|&q| newell.dot(q - centroid).abs())
                .collect::<Vec<_>>(),
        );
        let inner_out: Vec<f64> = if fill.len() > n {
            fill[n..]
                .iter()
                .map(|&q| newell.dot(q - centroid))
                .collect()
        } else {
            vec![0.0]
        };
        let span = max_of(
            &fill[..n]
                .iter()
                .map(|&q| (q - centroid).length())
                .collect::<Vec<_>>(),
        );
        log(&format!(
            "  hole {h}: loop leaves its plane by {loop_off:.4}; inner vertices stand out of it {:.4} / {:.4}; loop span {span:.3}",
            max_of(&inner_out),
            min_of(&inner_out)
        ));
        log(&format!(
            "  hole {h}: loop {n}, {} inner, {} strip ends; patch-fan {:.4}, cylinders-fan {:.4}, patch at {place:.2} of fan->cylinders; worst inner vertex off its nearest cylinder {nearest:.4} ({:.1}% of r); {}",
            fill.len() - n,
            rows.len(),
            patch - fan,
            cylinder - fan,
            nearest / radius * 100.0,
            per_row.join("; ")
        ));
    }

    log(&format!(
        "  removed with fans {:.4}, with the patches {removed:.4}, with patches on the cylinders {:.4}",
        removed + patch_over_fan,
        removed + patch_over_fan - cylinder_over_fan
    ));

    let mut prism_sum = 0.0;
    for k in 0..chains.len() {
        let prism = prism(positions, indices, &chains[k], closed[k], size);
        prism_sum += prism;
        let one = p.build(&[chains[k].clone()], &[closed[k]]);
        let one_fill = one
            .built()
            .then(|| fill_with_the_filler(&one, BandProfile::Fillet, p.segments));
        let alone = match &one_fill {
            Some(f) if f.refusal == GapFillRefusal::None => {
                source - signed_volume(&f.mesh.as_ref().expect("a filled mesh").borrow())
            }
            _ => f64::NAN,
        };
        log(&format!(
            "  chain {k} ({} vertices, {}): alone removes {alone:.4} ({}), exact prism {prism:.4}, alone - prism {:.4}",
            chains[k].len(),
            if closed[k] { "closed" } else { "open" },
            match &one_fill {
                Some(f) => format!("{:?}", f.refusal),
                None => format!("{:?}", one.refusal),
            },
            alone - prism
        ));
    }

    log(&format!(
        "  prisms summed {prism_sum:.4} (corner overlaps counted twice); pick removed {removed:.4}"
    ));

    // The strips' approach into the corner: the area removed in planes square to the longest
    // convex chain, from the end nearest the biggest hole, against the exact fillet's
    // cross-section.
    let mut longest: Option<usize> = None;
    for k in 0..chains.len() {
        if closed[k] || prism(positions, indices, &chains[k], false, size) <= 0.0 {
            continue;
        }

        let span = |k: usize| {
            (positions[chains[k][chains[k].len() - 1] as usize] - positions[chains[k][0] as usize])
                .length()
        };
        if longest.is_none_or(|best| span(k) > span(best)) {
            longest = Some(k);
        }
    }

    let longest = longest.expect("a convex open chain");
    let chain = &chains[longest];
    let mut corner = &holes[0];
    for hole in holes.iter() {
        if hole.1 > corner.1 {
            corner = hole;
        }
    }

    let corner_at = centroid_of(&corner.0[..corner.1]);
    let (first, last) = (
        positions[chain[0] as usize],
        positions[chain[chain.len() - 1] as usize],
    );
    let (from, to) = if (first - corner_at).length() < (last - corner_at).length() {
        (first, last)
    } else {
        (last, first)
    };
    let along = (to - from).get_normal();
    let length = (to - from).length();
    let per_length =
        prism(positions, indices, chain, false, size) / polyline_length(positions, chain);
    let kept = sweep_mesh_measures::kept(&filled_mesh);
    let mut stations = Vec::new();
    for fraction in [
        0.02, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 0.98,
    ] {
        let at = from + (along * (fraction * length));
        let cut = section_area(positions, indices, at, along)
            - section_area(filled_mesh.vertices(), &kept, at, along);
        stations.push(format!(
            "{:.2}:{:.0}%",
            fraction * length,
            cut / per_length * 100.0
        ));
    }

    log(&format!(
        "  chain {longest} (length {length:.3}, exact section {per_length:.4}) removed section / exact, by distance from the corner end: {}",
        stations.join(" ")
    ));
}

/// On inner-right-sides-and-center-op0: each chain of the fixture
/// built alone, its removed cross-section (negative where it adds) read every 0.5 mm along each of
/// its edges, from 3 mm before the edge to 3 mm past it, against the exact straight prism's
/// section - so an excess is placed on the whole strip or at the ends.
#[test]
fn chain_alone_section_profile() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load("inner-right-sides-and-center-op0"));
    let size = fixture.size;
    let source = signed_volume_of(&fixture.positions, &fixture.indices);
    for (k, chain) in fixture.chains.iter().enumerate() {
        let prism = prism(
            &fixture.positions,
            &fixture.indices,
            chain,
            fixture.closed[k],
            size,
        );
        let gap = band_gap_entry_points::build_pick(
            &fixture.positions,
            &fixture.indices,
            std::slice::from_ref(chain),
            &[fixture.closed[k]],
            Some(feature_of(&fixture)),
            size,
            BandProfile::Fillet,
            &BandGapBuildOptions {
                seam_epsilon: fixture.seam_epsilon,
                sharp_turn_degrees: fixture.sharp_turn_degrees,
                fillet_segments: fixture.segments,
                ..BandGapBuildOptions::default()
            },
        )
        .expect("a valid pick");
        if !gap.built() {
            log(&format!("chain {k}: band {:?}", gap.refusal));
            continue;
        }

        let filled = fill_with_the_filler(&gap, BandProfile::Fillet, fixture.segments);
        log(&format!(
            "  chain {k} end rays {}/{}",
            gap.chains[0].end_rays.0, gap.chains[0].end_rays.1
        ));
        let tip = fixture.positions[chain[chain.len() - 1] as usize];
        let onward = (tip - fixture.positions[chain[chain.len() - 2] as usize]).get_normal();
        if filled.refusal != GapFillRefusal::None {
            log(&format!(
                "chain {k}: fill {:?} at vertex {}",
                filled.refusal, filled.offending_vertex
            ));
            let mesh = gap.mesh.as_ref().expect("a built band").borrow();
            for l in &gap.loops {
                let read: Vec<String> = l
                    .vertices
                    .iter()
                    .map(|&v| {
                        format!(
                            "{:.2}/{:.2}",
                            onward.dot(mesh.position(v) - tip),
                            (mesh.position(v) - tip).length()
                        )
                    })
                    .collect();
                log(&format!(
                    "  rail side {}, along past the far end: {}",
                    l.side,
                    read.join(" ")
                ));
            }

            continue;
        }

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let alone = source - signed_volume(&mesh);
        log(&format!(
            "chain {k} {chain:?} size {size}: alone removes {alone:.4}, prism {prism:.4}, alone - prism {:.4}",
            alone - prism
        ));
        let kept = sweep_mesh_measures::kept(&mesh);
        let loops: Vec<String> = gap
            .loops
            .iter()
            .map(|l| {
                format!(
                    "side {} {} {}",
                    l.side,
                    if l.is_closed { "closed" } else { "open" },
                    l.vertices.len()
                )
            })
            .collect();
        log(&format!("  loops: {}", loops.join("; ")));

        // How far each rail vertex stands past the chain's far end along its last edge: an end ray
        // carries a rail straight on, a plain end curls it round within a setback.
        for l in &gap.loops {
            let read: Vec<String> = l
                .vertices
                .iter()
                .map(|&v| format!("{:.2}", onward.dot(mesh.position(v) - tip)))
                .collect();
            log(&format!(
                "  rail side {}, along past the far end: {}",
                l.side,
                read.join(" ")
            ));
        }

        for e in 0..chain.len() - 1 {
            let from = fixture.positions[chain[e] as usize];
            let to = fixture.positions[chain[e + 1] as usize];
            let along = (to - from).get_normal();
            let length = (to - from).length();
            let per_length = prism_of_edge(&fixture, chain[e], chain[e + 1], size) / length;
            let mut stations = Vec::new();
            let mut d = -3.0;
            while d <= length + 3.0 {
                let at = from + (along * d);
                let cut = section_area(&fixture.positions, &fixture.indices, at, along)
                    - section_area(mesh.vertices(), &kept, at, along);
                stations.push(format!("{d:.1}:{cut:.3}"));
                d += 0.5;
            }

            log(&format!(
                "  edge {}-{} length {length:.3} exact section {per_length:.4}; removed section by distance: {}",
                chain[e],
                chain[e + 1],
                stations.join(" ")
            ));
        }
    }
}

fn prism_of_edge(fixture: &LevelLineFixture, a: i32, b: i32, size: f64) -> f64 {
    prism(&fixture.positions, &fixture.indices, &[a, b], false, size)
}

fn centroid_of(points: &[Vector3]) -> Vector3 {
    let mut sum = Vector3::ZERO;
    for &q in points {
        sum += q;
    }

    sum / points.len() as f64
}

fn max_of(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

fn min_of(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::INFINITY, f64::min)
}

/// To the console, and appended to `BEVEL_CORNER_VOLUME_OUT` when set.
fn log(line: &str) {
    use std::io::Write as _;

    println!("{line}");
    if let Ok(output) = std::env::var(format!("{RUN_VARIABLE}_OUT")) {
        if !output.is_empty() {
            std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&output)
                .and_then(|mut file| writeln!(file, "{line}"))
                .expect("the log appends");
        }
    }
}

fn polyline_length(positions: &[Vector3], chain: &[i32]) -> f64 {
    (0..chain.len() - 1)
        .map(|i| (positions[chain[i + 1] as usize] - positions[chain[i] as usize]).length())
        .sum()
}

/// A closed mesh's cross-section area in the plane through `origin` square to
/// `normal` - each crossing triangle's segment, oriented along normal x its own normal, summed as a
/// signed polygon area.
fn section_area(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    origin: Vector3,
    normal: Vector3,
) -> f64 {
    let mut area = 0.0;
    for &(ta, tb, tc) in triangles {
        let p = [
            positions[ta as usize],
            positions[tb as usize],
            positions[tc as usize],
        ];
        let d: Vec<f64> = p.iter().map(|&x| normal.dot(x - origin)).collect();
        let mut hits = Vec::new();
        for i in 0..3 {
            let j = (i + 1) % 3;
            if (d[i] < 0.0) != (d[j] < 0.0) {
                hits.push(p[i] + ((p[j] - p[i]) * (d[i] / (d[i] - d[j]))));
            }
        }

        if hits.len() != 2 {
            continue;
        }

        let face_normal = (p[1] - p[0]).cross(p[2] - p[0]);
        let (a, b) = if (hits[1] - hits[0]).dot(normal.cross(face_normal)) > 0.0 {
            (hits[0], hits[1])
        } else {
            (hits[1], hits[0])
        };
        area += (a - origin).cross(b - origin).dot(normal) / 2.0;
    }

    area.abs()
}

fn tet(a: Vector3, b: Vector3, c: Vector3) -> f64 {
    a.dot(b.cross(c)) / 6.0
}

fn onto_nearest_cylinder(p: Vector3, rows: &[Row]) -> Vector3 {
    let Some(mut row) = rows.first() else {
        return p;
    };
    for r in &rows[1..] {
        if r.proud(p).abs() < row.proud(p).abs() {
            row = r;
        }
    }

    let off = p - row.centre;
    let foot = row.centre + (row.axis * off.dot(row.axis));
    let radial = p - foot;
    if radial.length_squared() < 1e-24 {
        p
    } else {
        foot + (radial.get_normal() * row.radius)
    }
}

/// The exact straight fillet's removed prism along a chain - per edge, length x r^2
/// (tan(phi/2) - phi/2), signed (a concave edge adds).
pub(crate) fn prism(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    r: f64,
) -> f64 {
    let mut total = 0.0;
    let steps = if closed { chain.len() } else { chain.len() - 1 };
    let has = |t: &(i32, i32, i32), x: i32| t.0 == x || t.1 == x || t.2 == x;
    let normal = |t: (i32, i32, i32)| {
        let a = positions[t.0 as usize];
        (positions[t.1 as usize] - a)
            .cross(positions[t.2 as usize] - a)
            .get_normal()
    };
    for i in 0..steps {
        let a = chain[i];
        let b = chain[(i + 1) % chain.len()];
        let on: Vec<(i32, i32, i32)> = indices
            .iter()
            .filter(|t| has(t, a) && has(t, b))
            .copied()
            .collect();
        if on.len() != 2 {
            continue;
        }

        let n1 = normal(on[0]);
        let phi = n1.dot(normal(on[1])).clamp(-1.0, 1.0).acos();
        let t = on[1];
        let third = if t.0 != a && t.0 != b {
            t.0
        } else if t.1 != a && t.1 != b {
            t.1
        } else {
            t.2
        };
        let convex = n1.dot(positions[third as usize] - positions[a as usize]) < 0.0;
        total += if convex { 1.0 } else { -1.0 }
            * (positions[b as usize] - positions[a as usize]).length()
            * r
            * r
            * ((phi / 2.0).tan() - (phi / 2.0));
    }

    total
}

/// A strip end's circle, carried on as the cylinder its strip would continue as past the
/// end.
struct Row {
    centre: Vector3,
    axis: Vector3,
    radius: f64,
    concave: bool,
}

impl Row {
    fn new(mesh: &LevelLineMesh, row: &CornerCapLastRow) -> Row {
        // LastRow's centre and radius are the fillet's circle only on a right-angled edge; the
        // arc's own circumcircle is the fillet's circle at any angle.
        let a = mesh.position(row.arc[0]);
        let b = mesh.position(row.arc[row.arc.len() / 2]);
        let c = mesh.position(row.arc[row.arc.len() - 1]);
        let ab = b - a;
        let ac = c - a;
        let normal = ab.cross(ac);
        let centre = a
            + ((normal.cross(ab) * ac.length_squared()) + (ac.cross(normal) * ab.length_squared()))
                / (2.0 * normal.length_squared());
        Row {
            centre,
            axis: normal.get_normal(),
            radius: (a - centre).length(),
            concave: row.concave,
        }
    }

    /// How far `p` stands out of the cylinder on the side that keeps more material than the rolling
    /// ball: for a convex row outside the cylinder (toward the removed edge), for a concave row
    /// inside it (in the air).
    fn proud(&self, p: Vector3) -> f64 {
        let off = p - self.centre;
        let d = (off - (self.axis * off.dot(self.axis))).length() - self.radius;
        if self.concave {
            -d
        } else {
            d
        }
    }
}
