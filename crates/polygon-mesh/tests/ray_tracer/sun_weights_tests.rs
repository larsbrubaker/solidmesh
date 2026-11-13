//! The GPU route's weights for sunlight through glass (`sun_weights.rs`, the Rust mirror of
//! `path_sample_sun`, `path_sun_toward`, `path_sun_after` and `path_escaped` in `sun.wgsl`)
//! against the CPU's own rules (`sun_toward_at`, `bent_sun_at` and `SunPathAt`, which run
//! `sun_toward.rs` and `glass_chain.rs`), case by case to `f32` rounding: the two shares and how
//! far the line is walked, the shadow term with a clear line and through glass under each
//! strategy, a receiver inside glass with and without a first face, every row of the record's
//! table after an event, and the sun's weight on escape. `sun_weights_text_tests` pins the
//! shader's text to the mirror's steps.

use solidmesh_mesh::ray_tracer::flat_tree::{
    arrival_pdf_f32, escaped_sun_weight_f32, sample_sun_f32, sun_after_f32, sun_bent_f32,
    sun_toward_f32, SunAtF32, SunEventF32, SunLineF32, SunStateF32, SunViewF32,
};
use solidmesh_mesh::ray_tracer::{bent_sun_at, sun_toward_at, LightStrategy, SunPathAt};

const STRATEGIES: [LightStrategy; 3] = [
    LightStrategy::Combined,
    LightStrategy::NextEventOnly,
    LightStrategy::SurfaceSamplingOnly,
];

/// A 2.5 degree sun's density, about 1 / (pi * (2.5 degrees)^2).
const SUN_PDF: f64 = 1680.0;
const SUN_RADIANCE: [f64; 3] = [30.0, 29.0, 27.0];

fn narrow3(v: [f64; 3]) -> [f32; 3] {
    v.map(|x| x as f32)
}

fn close(a: f32, b: f64, what: &str) {
    let tolerance = 4e-6 * b.abs().max(1e-30);
    assert!(
        (f64::from(a) - b).abs() <= tolerance,
        "{what}: GPU {a}, CPU {b}"
    );
}

fn close3(a: [f32; 3], b: [f64; 3], what: &str) {
    for (a, b) in a.into_iter().zip(b) {
        close(a, b, what);
    }
}

/// One direction at a vertex: `wo_z`, `wi_z`, the two density parts, the BSDF, the chain.
struct Case {
    wo_z: f64,
    wi_z: f64,
    densities: (f64, f64),
    bsdf: [f64; 3],
    chain: f64,
}

fn cases() -> Vec<Case> {
    let bsdf = [0.19, 0.17, 0.12];
    let case = |wo_z, wi_z, densities, chain| Case {
        wo_z,
        wi_z,
        densities,
        bsdf,
        chain,
    };
    vec![
        // A matte floor: other lobes only.
        case(0.8, 0.9, (0.29, 0.0), 1.0),
        // A sharp glossy floor, its density above the sun's.
        case(0.8, 0.9, (5200.0, 0.0), 1.0),
        // Half glass, half plastic, toward the sun on its own side.
        case(0.6, 0.7, (0.2, 3.1), 0.4),
        // Frosted glass, toward the sun on its own side: the glass lobe alone.
        case(0.6, 0.7, (0.0, 2.5), 0.7),
        // Through the surface: all glass, whatever the densities say.
        case(0.6, -0.7, (0.3, 2.5), 0.25),
        // A delta-only glass lobe beside plastic: nothing toward a sampled direction is glass's.
        case(0.5, 0.5, (0.0, 0.0), 1.0),
        // Glass with a chain that has kept nothing.
        case(0.6, 0.7, (0.0, 2.5), 0.0),
    ]
}

#[test]
fn the_shares_and_the_shadow_term_are_the_cpus() {
    let reaches = [
        None,
        Some(([1.0; 3], 0)),
        Some(([0.83, 0.86, 0.9], 2)),
        Some(([0.92; 3], 1)),
    ];
    let mut walks = Vec::new();
    for strategy in STRATEGIES {
        for (index, case) in cases().iter().enumerate() {
            for reach in reaches {
                let cpu = sun_toward_at(
                    strategy,
                    case.wo_z,
                    case.wi_z,
                    case.bsdf,
                    SUN_PDF,
                    SUN_RADIANCE,
                    case.densities,
                    case.chain,
                    reach,
                );
                let gpu = sun_toward_f32(
                    strategy,
                    case.wo_z as f32,
                    case.wi_z as f32,
                    [case.densities.0 as f32, case.densities.1 as f32],
                    narrow3(case.bsdf),
                    SUN_PDF as f32,
                    narrow3(SUN_RADIANCE),
                    case.chain as f32,
                );
                let what = format!("{strategy:?} case {index} reach {reach:?}");
                close(gpu.other, cpu.other_share, &what);
                close(gpu.glass, cpu.glass_share, &what);
                close(gpu.glass_weight, cpu.glass_weight, &what);
                assert_eq!(gpu.walk(), cpu.walk, "{what}");
                if !walks.contains(&cpu.walk) {
                    walks.push(cpu.walk);
                }
                if let Some((transmittance, crossed)) = reach {
                    let light = gpu.light(strategy, narrow3(transmittance), crossed);
                    close3(light, cpu.light.unwrap_or([0.0; 3]), &what);
                }
            }
        }
    }
    // Every way of walking the line came up: straight, stopped at any surface, not at all.
    assert_eq!(walks.len(), 3, "{walks:?}");
}

/// The model's values toward a direction, from the CPU's values for it.
fn at_of(case: &Case) -> SunAtF32 {
    SunAtF32 {
        sides_agree: true,
        wi_z: case.wi_z as f32,
        bsdf: narrow3(case.bsdf),
        parts: [case.densities.0 as f32, case.densities.1 as f32],
        pdf: (case.densities.0 + case.densities.1) as f32,
    }
}

fn view(straight: bool, strategy: LightStrategy) -> SunViewF32 {
    SunViewF32 {
        straight,
        has_sun: true,
        strategy,
        sun_pdf: SUN_PDF as f32,
        sun_radiance: narrow3(SUN_RADIANCE),
    }
}

fn line(reached: bool, crossed: u32, first_face: Option<([f32; 3], f32)>) -> SunLineF32 {
    SunLineF32 {
        reached,
        transmittance: if crossed == 0 {
            [1.0; 3]
        } else {
            [0.9, 0.91, 0.93]
        },
        crossed,
        first_face,
    }
}

#[test]
fn a_vertex_in_open_air_takes_the_cpus_two_routes_and_exact_without_the_flag() {
    let sun = [0.0, 0.0, 1.0];
    for strategy in STRATEGIES {
        for (index, case) in cases().iter().enumerate() {
            for crossed in [0, 2] {
                let what = format!("{strategy:?} case {index} crossed {crossed}");
                // With the flag: the shares decide the walk, the CPU's term the light.
                let mut asked = None;
                let shadow = sample_sun_f32(
                    &view(true, strategy),
                    case.wo_z as f32,
                    case.chain as f32,
                    false,
                    false,
                    sun,
                    &mut |_| at_of(case),
                    &mut |straight| {
                        asked = Some(straight);
                        line(true, crossed, None)
                    },
                );
                let reach = Some((
                    line(true, crossed, None).transmittance.map(f64::from),
                    crossed,
                ));
                let cpu = sun_toward_at(
                    strategy,
                    case.wo_z,
                    case.wi_z,
                    case.bsdf,
                    SUN_PDF,
                    SUN_RADIANCE,
                    case.densities,
                    case.chain,
                    reach,
                );
                assert_eq!(asked, cpu.walk, "{what}");
                assert_eq!(shadow.walked, cpu.walk, "{what}");
                close3(shadow.light, cpu.light.unwrap_or([0.0; 3]), &what);
                assert_eq!(shadow.receiver_pdf, SUN_PDF as f32, "{what}");
                // Without it: Exact's shadow ray, at the whole density, none under surface sampling.
                let exact = sample_sun_f32(
                    &view(false, strategy),
                    case.wo_z as f32,
                    case.chain as f32,
                    false,
                    false,
                    sun,
                    &mut |_| at_of(case),
                    &mut |straight| {
                        assert!(straight, "Exact walks the plain shadow ray");
                        line(crossed == 0, 0, None)
                    },
                );
                let pdf = case.densities.0 + case.densities.1;
                let weight = match strategy {
                    LightStrategy::Combined => {
                        let ratio = pdf / SUN_PDF;
                        1.0 / (1.0 + ratio * ratio)
                    }
                    LightStrategy::NextEventOnly => 1.0,
                    LightStrategy::SurfaceSamplingOnly => 0.0,
                };
                let expected = if crossed == 0 && weight > 0.0 {
                    case.bsdf.map(|f| f * case.wi_z.abs() / SUN_PDF * weight)
                } else {
                    [0.0; 3]
                };
                let expected = [0, 1, 2].map(|c| expected[c] * SUN_RADIANCE[c]);
                close3(exact.light, expected, &format!("Exact {what}"));
            }
        }
    }
}

#[test]
fn a_receiver_inside_glass_reads_the_sun_bent_at_the_first_face_as_the_cpu_does() {
    // The line leaves a 1.5 glass block's top face, tilted 20 degrees off its normal.
    let tilt = 20.0_f64.to_radians();
    let sun = [tilt.sin(), 0.0, tilt.cos()];
    let outward = [0.0_f32, 0.0, 1.0];
    let ior = 1.5_f32;
    let (inside, cos_out, cos_in) =
        sun_bent_f32(outward, ior, narrow3(sun)).expect("the line leaves the face");
    let (cpu_pdf, cpu_radiance) = bent_sun_at(
        SUN_PDF,
        SUN_RADIANCE,
        f64::from(ior),
        f64::from(cos_in),
        f64::from(cos_out),
    );
    // A matte part inside: the model's values at the bent direction differ from the line's.
    let along_line = Case {
        wo_z: 0.9,
        wi_z: sun[2],
        densities: (0.3, 0.0),
        bsdf: [0.19; 3],
        chain: 0.6,
    };
    let along_bent = Case {
        wo_z: 0.9,
        wi_z: f64::from(inside[2]),
        densities: (0.31, 0.0),
        bsdf: [0.2; 3],
        chain: 0.6,
    };
    for strategy in STRATEGIES {
        for (first, reached) in [(true, true), (false, true), (true, false)] {
            let what = format!("{strategy:?} first face {first} reached {reached}");
            let mut asked = Vec::new();
            let shadow = sample_sun_f32(
                &view(true, strategy),
                0.9,
                0.6,
                true,
                false,
                narrow3(sun),
                &mut |direction| {
                    asked.push(direction);
                    if direction == inside {
                        at_of(&along_bent)
                    } else {
                        at_of(&along_line)
                    }
                },
                &mut |straight| {
                    assert!(straight, "a receiver inside glass walks its line straight");
                    line(reached, 1, first.then_some((outward, ior)))
                },
            );
            let (pdf, radiance, case) = if first {
                (cpu_pdf, cpu_radiance, &along_bent)
            } else {
                (SUN_PDF, SUN_RADIANCE, &along_line)
            };
            close(shadow.receiver_pdf, pdf, &what);
            let cpu = sun_toward_at(
                strategy,
                case.wo_z,
                case.wi_z,
                case.bsdf,
                pdf,
                radiance,
                case.densities,
                case.chain,
                reached.then_some((line(true, 1, None).transmittance.map(f64::from), 1)),
            );
            close3(shadow.light, cpu.light.unwrap_or([0.0; 3]), &what);
            // The line first, then, once it reached the sun, the direction read.
            let expected: Vec<[f32; 3]> = match (first, reached) {
                (_, false) => vec![narrow3(sun)],
                (true, true) => vec![narrow3(sun), inside],
                (false, true) => vec![narrow3(sun), narrow3(sun)],
            };
            assert_eq!(asked, expected, "{what}");
        }
    }
    // At a seam or on a contact the receiver is not inside glass: the unbent line's shares.
    let shadow = sample_sun_f32(
        &view(true, LightStrategy::Combined),
        0.9,
        0.6,
        true,
        true,
        narrow3(sun),
        &mut |_| at_of(&along_line),
        &mut |_| line(true, 1, Some((outward, ior))),
    );
    assert_eq!(shadow.receiver_pdf, SUN_PDF as f32);
}

/// One drawn event: delta or not, from the glass lobe or not, its whole density, its two parts and
/// its direction's local `z`.
type Drawn = (bool, bool, f64, (f64, f64), f64);

/// Every event the path loop can draw, as the record reads it: sampled from another lobe, a delta
/// of another lobe, sampled from glass, a delta of glass.
fn events() -> Vec<Drawn> {
    vec![
        (false, false, 0.5, (0.3, 0.2), 0.7),
        (false, false, 9000.0, (9000.0, 0.0), 0.4),
        (true, false, 0.0, (0.0, 0.0), 0.8),
        (false, true, 2.2, (0.1, 2.1), -0.6),
        (false, true, 2.2, (0.1, 2.1), 0.6),
        (true, true, 0.0, (0.0, 0.0), -0.9),
        (true, true, 0.0, (0.0, 0.0), 0.9),
    ]
}

fn state_of(path: SunPathAt) -> SunStateF32 {
    SunStateF32 {
        chain: path.chain as f32,
        in_chain: path.in_chain,
        last_glass: path.last_glass,
        in_glass: path.in_glass,
    }
}

#[test]
fn the_record_after_every_event_and_the_escape_weight_are_the_cpus() {
    // Walk every sequence of three events from the camera, at a seam, on glass and on an opaque
    // surface, so each row of the table starts from every state the rows before it leave.
    let surfaces = [(false, false), (false, true), (true, true)];
    let receiver_pdfs = [SUN_PDF, SUN_PDF * 2.4];
    let mut seen = Vec::new();
    let mut frontier = vec![SunPathAt::camera()];
    for _ in 0..3 {
        let mut next_frontier = Vec::new();
        for cpu_before in &frontier {
            for (is_specular, from_glass, pdf, densities, direction_z) in events() {
                for (at_seam, transmissive) in surfaces {
                    for receiver_pdf in receiver_pdfs {
                        for straight in [true, false] {
                            let cpu = cpu_before.after(
                                straight,
                                is_specular,
                                from_glass,
                                pdf,
                                densities,
                                direction_z,
                                at_seam,
                                transmissive,
                                receiver_pdf,
                            );
                            let drawn = SunEventF32 {
                                is_specular,
                                from_glass,
                                pdf: pdf as f32,
                                other_pdf: densities.0 as f32,
                                glass_pdf: densities.1 as f32,
                                direction_z: direction_z as f32,
                            };
                            let what = format!(
                                "{cpu_before:?} then {drawn:?} seam {at_seam} glass \
                                 {transmissive} straight {straight}"
                            );
                            assert_eq!(
                                arrival_pdf_f32(straight, &drawn).map(f64::from),
                                cpu.arrival.map(|pdf| f64::from(pdf as f32)),
                                "{what}"
                            );
                            if !straight {
                                // Exact keeps no record beside the arrival; the shader writes none.
                                assert_eq!(cpu.chain, cpu_before.chain, "{what}");
                                continue;
                            }
                            let gpu = sun_after_f32(
                                state_of(*cpu_before),
                                &drawn,
                                at_seam,
                                transmissive,
                                receiver_pdf as f32,
                            );
                            assert_eq!(gpu.in_glass, cpu.in_glass, "{what}");
                            assert_eq!(gpu.in_chain, cpu.in_chain, "{what}");
                            assert_eq!(gpu.last_glass, cpu.last_glass, "{what}");
                            close(gpu.chain, cpu.chain, &what);
                            for strategy in STRATEGIES {
                                let weight = escaped_sun_weight_f32(
                                    strategy,
                                    arrival_pdf_f32(straight, &drawn),
                                    SUN_PDF as f32,
                                    &gpu,
                                );
                                close(weight, cpu.sun_weight(strategy, SUN_PDF), &what);
                            }
                            let row = (cpu.in_chain, cpu.last_glass, cpu.in_glass);
                            if !seen.contains(&row) {
                                seen.push(row);
                            }
                            next_frontier.push(cpu);
                        }
                    }
                }
            }
        }
        next_frontier.dedup();
        next_frontier.truncate(64);
        frontier = next_frontier;
    }
    // In a chain or not, after glass or not, inside glass or not: all four rows the table can leave came up
    // (a glass event always leaves the path in a chain).
    seen.sort();
    assert_eq!(
        seen,
        vec![
            (false, false, false),
            (true, false, true),
            (true, true, false),
            (true, true, true),
        ]
    );
}
