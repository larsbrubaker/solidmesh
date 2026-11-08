//! The surface scattering model's pointwise physics: reciprocity, the density `sample` draws
//! against `pdf`, the closed-form values, wild input and the local frame. Energy and the albedo
//! table are measured in `scattering_energy_tests`.
//!
//! Monte Carlo here uses a fixed seed and sample count, so each run computes the same numbers.
//! Each statistical check gives its tolerance as five standard errors of the estimate (the
//! standard error measured from the same samples, so it follows the sample count `N` as
//! `sqrt(variance / N)`), at which a true bound is falsely refused in fewer than one in a million
//! trials; plus any fixed margin named beside it for the quadrature the estimate is compared to.

use std::f64::consts::PI;

use crate::scattering_support::{direction, sphere_grid, Rng, Stats};
use solidmesh_math::Vector3;
use solidmesh_mesh::ray_tracer::{LocalFrame, ScatteringSurface};

const WHITE: [f64; 3] = [1.0, 1.0, 1.0];

fn opaque_surfaces() -> Vec<ScatteringSurface> {
    let mut surfaces = Vec::new();
    for &metallic in &[0.0, 0.3, 1.0] {
        for &roughness in &[0.0, 0.1, 0.3, 0.6, 1.0] {
            for &ior in &[1.0, 1.5, 2.5] {
                for &base in &[[1.0, 1.0, 1.0], [0.9, 0.5, 0.05]] {
                    surfaces.push(ScatteringSurface::new(base, metallic, roughness, 0.0, ior));
                }
            }
        }
    }
    surfaces
}

fn upper_directions() -> Vec<Vector3> {
    let mut directions = Vec::new();
    for &cos_theta in &[0.02, 0.2, 0.5, 0.8, 1.0] {
        for slice in 0..4 {
            directions.push(direction(cos_theta, 0.4 + slice as f64 * 1.4));
        }
    }
    directions
}

#[test]
fn opaque_surfaces_are_reciprocal() {
    for surface in opaque_surfaces() {
        for &a in &upper_directions() {
            for &b in &upper_directions() {
                let forward = surface.evaluate(a, b);
                let backward = surface.evaluate(b, a);
                for channel in 0..3 {
                    let scale = forward[channel].abs().max(1.0);
                    assert!(
                        (forward[channel] - backward[channel]).abs() <= 1e-9 * scale,
                        "{surface:?} {a:?} {b:?}: {forward:?} against {backward:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn rough_glass_is_reciprocal_after_the_radiance_factor() {
    let glass = ScatteringSurface::new(WHITE, 0.0, 0.4, 1.0, 1.5);
    for &above in &upper_directions() {
        for &below in &upper_directions() {
            let below = Vector3::new(below.x, below.y, -below.z);
            let forward = glass.evaluate(above, below)[0];
            let backward = glass.evaluate(below, above)[0];
            // wo in air, wi in glass, then the reverse: divide out each direction's eta_wo^2.
            assert!(
                (forward / 1.0 - backward / 2.25).abs() <= 1e-9 * (forward + backward).max(1.0),
                "{above:?} {below:?}: {forward} against {backward}"
            );
        }
    }
}

/// The surfaces whose sampling has no delta events, so a density comparison applies.
fn rough_surfaces() -> Vec<ScatteringSurface> {
    vec![
        ScatteringSurface::new(WHITE, 1.0, 0.4, 0.0, 1.5),
        ScatteringSurface::new(WHITE, 1.0, 1.0, 0.0, 1.5),
        ScatteringSurface::new([0.7, 0.4, 0.2], 0.0, 0.5, 0.0, 1.5),
        ScatteringSurface::new([0.7, 0.4, 0.2], 0.5, 0.7, 0.0, 2.0),
        ScatteringSurface::new(WHITE, 0.0, 0.5, 1.0, 1.5),
        ScatteringSurface::new(WHITE, 0.0, 0.8, 0.6, 1.33),
    ]
}

#[test]
fn pdf_integrates_to_the_mass_sample_does_not_refuse() {
    let mut rng = Rng(0xfeed_0002);
    let samples = 200_000;
    let (sphere, cell) = sphere_grid(720, 360);
    let mut report = Vec::new();
    for surface in rough_surfaces() {
        for &cos_theta in &[0.15, 0.5, 0.95, -0.5] {
            if cos_theta < 0.0 && surface.transmission() == 0.0 {
                continue;
            }
            let wo = direction(cos_theta, 0.7);
            let mut integral = 0.0;
            for &wi in &sphere {
                integral += surface.pdf(wo, wi) * cell;
            }
            let mut kept = Stats::default();
            for _ in 0..samples {
                kept.add(f64::from(u8::from(surface.sample(wo, rng.u3()).is_some())));
            }
            // A midpoint rule on this grid is good to about a thousandth on lobes this wide.
            let tolerance = kept.tolerance() + 2e-3;
            assert!(
                (integral - kept.mean()).abs() <= tolerance,
                "{surface:?} at cos {cos_theta}: pdf integrates to {integral}, sample keeps {}",
                kept.mean()
            );
            report.push(format!(
                "{surface:?} cos {cos_theta}: integral {integral:.4}, refused mass {:.4}",
                1.0 - kept.mean()
            ));
        }
    }
    println!("{}", report.join("\n"));
}

#[test]
fn the_density_of_sample_is_what_pdf_says() {
    let mut rng = Rng(0xfeed_0004);
    let draws: usize = 400_000;
    let bands = 16;
    let slices = 24;
    let sub = 32;
    let cell_area = 4.0 * PI / (bands * slices) as f64;
    let sub_area = cell_area / (sub * sub) as f64;
    for surface in rough_surfaces() {
        for &cos_theta in &[0.25, 0.8, -0.6] {
            if cos_theta < 0.0 && surface.transmission() == 0.0 {
                continue;
            }
            let wo = direction(cos_theta, 0.9);
            let mut counts = vec![0u64; bands * slices];
            for _ in 0..draws {
                if let Some(event) = surface.sample(wo, rng.u3()) {
                    assert!(!event.is_specular);
                    let cos = event.direction.z.clamp(-1.0, 1.0);
                    let band = (((cos + 1.0) * 0.5 * bands as f64) as usize).min(bands - 1);
                    let phi = event.direction.y.atan2(event.direction.x);
                    let turn = (phi + PI) / (2.0 * PI);
                    let slice = ((turn * slices as f64) as usize).min(slices - 1);
                    counts[band * slices + slice] += 1;
                }
            }
            for band in 0..bands {
                for slice in 0..slices {
                    let mut expected = 0.0;
                    for sub_band in 0..sub {
                        let cos = -1.0
                            + 2.0 * (band as f64 + (sub_band as f64 + 0.5) / sub as f64)
                                / bands as f64;
                        for sub_slice in 0..sub {
                            let turn = (slice as f64 + (sub_slice as f64 + 0.5) / sub as f64)
                                / slices as f64;
                            let phi = 2.0 * PI * turn - PI;
                            expected += surface.pdf(wo, direction(cos, phi)) * sub_area;
                        }
                    }
                    let observed = counts[band * slices + slice] as f64 / draws as f64;
                    let sigma = (expected * (1.0 - expected) / draws as f64).sqrt();
                    // Five sigma of the bin count, 2% for the sub-grid quadrature of the cell,
                    // and two counts for bins with almost nothing in them.
                    let tolerance = 5.0 * sigma + 0.02 * expected + 2.0 / draws as f64;
                    assert!(
                        (observed - expected).abs() <= tolerance,
                        "{surface:?} cos {cos_theta} bin {band},{slice}: observed {observed}, pdf \
                         integrates to {expected}"
                    );
                }
            }
        }
    }
}

#[test]
fn a_mirror_returns_all_of_the_light() {
    let metal = ScatteringSurface::new(WHITE, 1.0, 0.0, 0.0, 1.5);
    let event = metal
        .sample(direction(0.6, 0.0), [0.5, 0.5, 0.5])
        .expect("a mirror always reflects");
    assert!(event.is_specular);
    assert!((event.weight[0] - 1.0).abs() < 1e-12, "{event:?}");
}

/// The smooth surface's specular event, which a `pick` of 0 selects first.
fn specular_reflectance(surface: &ScatteringSurface, wo: Vector3) -> [f64; 3] {
    let event = surface
        .sample(wo, [0.0, 0.5, 0.5])
        .expect("a specular event at pick 0");
    assert!(event.is_specular, "{surface:?}");
    event.weight.map(|weight| weight * event.pdf)
}

#[test]
fn fresnel_is_f0_at_normal_incidence_and_one_at_grazing() {
    let surface = ScatteringSurface::new([0.5, 0.5, 0.5], 0.0, 0.0, 0.0, 1.5);
    let normal = specular_reflectance(&surface, Vector3::new(0.0, 0.0, 1.0));
    assert!((normal[0] - 0.04).abs() < 1e-12, "{normal:?}");
    let grazing = specular_reflectance(&surface, direction(1e-7, 0.0));
    assert!((grazing[0] - 1.0).abs() < 1e-5, "{grazing:?}");
    let metal = ScatteringSurface::new([0.3, 0.6, 0.9], 1.0, 0.0, 0.0, 1.5);
    let metal_normal = specular_reflectance(&metal, Vector3::new(0.0, 0.0, 1.0));
    for (reflectance, base) in metal_normal.iter().zip([0.3, 0.6, 0.9]) {
        assert!((reflectance - base).abs() < 1e-12, "{metal_normal:?}");
    }
}

#[test]
fn a_metal_goes_to_white_at_grazing_whatever_its_color() {
    let metal = ScatteringSurface::new([0.9, 0.01, 0.001], 1.0, 0.0, 0.0, 1.5);
    let grazing = specular_reflectance(&metal, direction(1e-7, 0.0));
    for channel in grazing {
        assert!((channel - 1.0).abs() < 1e-5, "{grazing:?}");
    }
}

#[test]
fn ior_one_is_pure_diffuse_with_a_cosine_pdf() {
    let base = [0.7, 0.3, 0.1];
    for &roughness in &[0.0, 0.4, 1.0] {
        let surface = ScatteringSurface::new(base, 0.0, roughness, 0.0, 1.0);
        for &wo in &upper_directions() {
            for &wi in &upper_directions() {
                let f = surface.evaluate(wo, wi);
                for (value, base) in f.iter().zip(base) {
                    assert!((value - base / PI).abs() < 1e-12, "{surface:?}: {f:?}");
                }
                assert!((surface.pdf(wo, wi) - wi.z / PI).abs() < 1e-12);
            }
        }
        let event = surface
            .sample(direction(0.5, 0.0), [0.5, 0.3, 0.8])
            .expect("diffuse never refuses here");
        assert!(!event.is_specular);
        for (weight, base) in event.weight.iter().zip(base) {
            assert!((weight - base).abs() < 1e-12, "{event:?}");
        }
    }
}

#[test]
fn a_smooth_metal_has_no_diffuse() {
    let metal = ScatteringSurface::new([0.9, 0.8, 0.7], 1.0, 0.0, 0.0, 1.5);
    for &wo in &upper_directions() {
        for &wi in &upper_directions() {
            assert_eq!(metal.evaluate(wo, wi), [0.0; 3]);
            assert_eq!(metal.pdf(wo, wi), 0.0);
        }
    }
    // Rough metal reflects, but away from the mirror direction it has no diffuse to add: at a
    // direction a broad lobe barely reaches, the value is far below a Lambert surface's `base / pi`.
    let rough = ScatteringSurface::new([0.9, 0.9, 0.9], 1.0, 0.6, 0.0, 1.5);
    let off_lobe = rough.evaluate(direction(0.7, 0.0), direction(0.7, 0.0));
    assert!(off_lobe[0] < 0.5 * 0.9 / PI, "{off_lobe:?}");
}

#[test]
fn smooth_glass_at_normal_incidence_reflects_four_percent() {
    let glass = ScatteringSurface::glass();
    let event = glass
        .sample(Vector3::new(0.0, 0.0, 1.0), [0.0, 0.5, 0.5])
        .expect("reflects");
    assert!(event.is_specular);
    assert!((event.pdf - 0.04).abs() < 1e-12, "{event:?}");
    assert!((event.direction.z - 1.0).abs() < 1e-12);
    assert!((event.weight[0] - 1.0).abs() < 1e-12, "{event:?}");
    let through = glass
        .sample(Vector3::new(0.0, 0.0, 1.0), [0.5, 0.5, 0.5])
        .expect("refracts");
    assert!((through.pdf - 0.96).abs() < 1e-12, "{through:?}");
    assert!((through.direction.z + 1.0).abs() < 1e-12, "{through:?}");
    // Entering the denser medium divides radiance by ior^2 (tracing from the camera).
    assert!(
        (through.weight[0] - 1.0 / 2.25).abs() < 1e-12,
        "{through:?}"
    );
}

#[test]
fn glass_obeys_snell_and_total_internal_reflection() {
    let glass = ScatteringSurface::glass();
    let eta = 1.5;
    // Entering at 40 degrees from the normal.
    let wo = direction(40.0_f64.to_radians().cos(), 0.8);
    let event = glass
        .sample(wo, [0.9999, 0.5, 0.5])
        .expect("refracts into the glass");
    assert!(event.direction.z < 0.0, "{event:?}");
    let sin_o = (1.0 - wo.z * wo.z).sqrt();
    let sin_i = (1.0 - event.direction.z * event.direction.z).sqrt();
    assert!(
        (sin_o - eta * sin_i).abs() < 1e-12,
        "{sin_o} against {eta}*{sin_i}"
    );
    assert!(wo.x * event.direction.x + wo.y * event.direction.y < 0.0);
    // Leaving the glass: past the critical angle everything reflects, weight 1 and pdf 1.
    let inside = direction(-0.3, 0.2);
    assert!((1.0 - inside.z * inside.z).sqrt() > 1.0 / eta);
    for &pick in &[0.0, 0.5, 0.9999] {
        let reflected = glass.sample(inside, [pick, 0.5, 0.5]).expect("reflects");
        assert!(reflected.is_specular);
        assert!((reflected.pdf - 1.0).abs() < 1e-12, "{reflected:?}");
        assert!((reflected.weight[0] - 1.0).abs() < 1e-12);
        assert!((reflected.direction.z - inside.z).abs() < 1e-12);
        assert!((reflected.direction.x + inside.x).abs() < 1e-12);
    }
    // Inside the critical angle, light leaves, and the radiance factor is the reverse.
    let leaving = direction(-0.95, 0.2);
    let out = glass
        .sample(leaving, [0.9999, 0.5, 0.5])
        .expect("refracts out");
    assert!(out.direction.z > 0.0, "{out:?}");
    assert!((out.weight[0] - 2.25).abs() < 1e-12, "{out:?}");
}

#[test]
fn glass_of_ior_one_passes_everything_through_unchanged() {
    for &roughness in &[0.0, 0.5, 1.0] {
        let slab = ScatteringSurface::new(WHITE, 0.0, roughness, 1.0, 1.0);
        for &wo in &upper_directions() {
            let event = slab.sample(wo, [0.5, 0.5, 0.5]).expect("passes through");
            assert!(event.is_specular);
            let through = Vector3::new(-wo.x, -wo.y, -wo.z);
            assert!(event.direction.equals_within(through, 1e-12), "{event:?}");
            assert!((event.weight[0] - 1.0).abs() < 1e-12, "{event:?}");
            assert_eq!(slab.pdf(wo, event.direction), 0.0);
            assert_eq!(slab.evaluate(wo, event.direction), [0.0; 3]);
        }
    }
}

#[test]
fn wild_input_gives_only_finite_non_negative_values() {
    let wild = [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        0.0,
        -0.0,
        1e-320,
        -1e-320,
        1e-300,
        1e-9,
        1e-8,
        -1e-9,
        1e300,
        -1e300,
        1.0,
        -1.0,
        1.0000000001,
        0.9999999999,
        0.5,
    ];
    let mut rng = Rng(0xfeed_0007);
    let pick = |rng: &mut Rng, list: &[f64]| list[rng.next_u64() as usize % list.len()];
    let wild_vector = |rng: &mut Rng| {
        // Half the time a real direction just off unit length, the rest anything at all.
        if rng.unit() < 0.5 {
            let scale = 1.0 + (rng.unit() - 0.5) * 1e-8;
            direction(rng.unit() * 2.0 - 1.0, rng.unit() * 6.3) * scale
        } else {
            Vector3::new(pick(rng, &wild), pick(rng, &wild), pick(rng, &wild))
        }
    };
    let check = |value: f64, what: &str| {
        assert!(value.is_finite() && value >= 0.0, "{what} = {value}");
    };
    for _ in 0..200_000 {
        let base = [pick(&mut rng, &wild), rng.unit(), pick(&mut rng, &wild)];
        let surface = ScatteringSurface::new(
            base,
            pick(&mut rng, &[0.0, 1.0, 0.5, -3.0, f64::NAN]),
            pick(
                &mut rng,
                &[0.0, 1.0, 0.5, 1e-200, 1e-6, -1.0, f64::INFINITY],
            ),
            pick(&mut rng, &[0.0, 1.0, 0.5, 7.0, f64::NEG_INFINITY]),
            pick(&mut rng, &[1.0, 1.0000001, 1.5, 4.0, 1e6, -2.0, f64::NAN]),
        );
        let wo = wild_vector(&mut rng);
        let wi = wild_vector(&mut rng);
        let u = [
            pick(&mut rng, &wild),
            pick(&mut rng, &wild),
            pick(&mut rng, &wild),
        ];
        for value in surface.evaluate(wo, wi) {
            check(value, "evaluate");
        }
        check(surface.pdf(wo, wi), "pdf");
        if let Some(event) = surface.sample(wo, u) {
            check(event.pdf, "sample.pdf");
            assert!(event.pdf > 0.0, "{event:?}");
            for value in event.weight {
                check(value, "sample.weight");
            }
            let length = event.direction.length();
            assert!((length - 1.0).abs() < 1e-9, "{event:?}");
        }
    }
}

#[test]
fn presets_are_what_their_names_say() {
    assert_eq!(ScatteringSurface::polished_metal(WHITE).metallic(), 1.0);
    assert_eq!(ScatteringSurface::brushed_metal(WHITE).metallic(), 1.0);
    assert_eq!(ScatteringSurface::matte_plastic(WHITE).metallic(), 0.0);
    assert_eq!(ScatteringSurface::rubber(WHITE).transmission(), 0.0);
    assert!(
        ScatteringSurface::glossy_plastic(WHITE).roughness()
            < ScatteringSurface::matte_plastic(WHITE).roughness()
    );
    assert!(
        ScatteringSurface::polished_metal(WHITE).roughness()
            < ScatteringSurface::brushed_metal(WHITE).roughness()
    );
    let glass = ScatteringSurface::glass();
    assert_eq!(glass.transmission(), 1.0);
    assert_eq!(glass.roughness(), 0.0);
    assert_eq!(glass.ior(), 1.5);
}

#[test]
fn out_of_range_settings_are_clamped() {
    let surface = ScatteringSurface::new([-1.0, 2.0, 0.5], 9.0, -1.0, 4.0, 0.5);
    assert_eq!(surface.base_color(), [0.0, 1.0, 0.5]);
    assert_eq!(surface.metallic(), 1.0);
    assert_eq!(surface.roughness(), 0.0);
    assert_eq!(surface.transmission(), 1.0);
    assert_eq!(surface.ior(), 1.0);
    let blank = ScatteringSurface::new([f64::NAN; 3], f64::NAN, f64::NAN, f64::NAN, f64::NAN);
    assert_eq!(blank.base_color(), [0.0; 3]);
    assert_eq!(blank.metallic(), 0.0);
    assert_eq!(blank.roughness(), 1.0);
    assert_eq!(blank.ior(), 1.5);
}

#[test]
fn local_frames_are_orthonormal_at_every_axis() {
    let mut normals = vec![
        Vector3::UNIT_X,
        -Vector3::UNIT_X,
        Vector3::UNIT_Y,
        -Vector3::UNIT_Y,
        Vector3::UNIT_Z,
        -Vector3::UNIT_Z,
        Vector3::new(0.0, 0.0, -0.0),
        Vector3::new(1e-16, 1e-16, -1.0),
        Vector3::new(1e-300, 0.0, 1.0),
        Vector3::new(1e300, 1.0, 1e-300),
        Vector3::new(3.0, 4.0, 12.0),
    ];
    let mut rng = Rng(0xfeed_0008);
    for _ in 0..2000 {
        normals.push(direction(rng.unit() * 2.0 - 1.0, rng.unit() * 6.3));
    }
    for n in normals {
        let Some(frame) = LocalFrame::from_normal(n) else {
            assert!(n.length() == 0.0, "{n:?} refused");
            continue;
        };
        let check = |a: Vector3, b: Vector3, expected: f64| {
            assert!((a.dot(b) - expected).abs() < 1e-12, "{n:?}: {frame:?}");
        };
        check(frame.tangent, frame.tangent, 1.0);
        check(frame.bitangent, frame.bitangent, 1.0);
        check(frame.tangent, frame.bitangent, 0.0);
        check(frame.tangent, frame.normal, 0.0);
        check(frame.tangent.cross(frame.bitangent), frame.normal, 1.0);
        let v = Vector3::new(0.3, -0.7, 0.2);
        assert!(frame.to_world(frame.to_local(v)).equals_within(v, 1e-12));
    }
    assert!(LocalFrame::from_normal(Vector3::new(f64::NAN, 0.0, 1.0)).is_none());
}
