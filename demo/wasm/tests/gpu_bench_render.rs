// The GPU page's render check: its comparison catches a biased render, accepts one that only
// differs by the sampling noise it measures, holds a closed-form scene to its answer in both
// renders, finds a leak along a single line of pixels that a region would dilute, fails on
// samples the GPU drops, and refuses GPU output of the wrong size. The scenes that reach each
// branch build, and the CPU reference runs the closed forms exactly.
// A GPU is what really runs the shader; this is everything else.

use solidmesh::mesh::ray_tracer::flat_tree::ACCUMULATION_STRIDE;
use solidmesh_wasm::gpu_bench_render::{
    check_scene, compare_groups, gpu_passes, layout_of, render_cpu, with_dropped_samples,
    GroupLayout, PassGroups, SCENE_COUNT, Z_THRESHOLD,
};
use solidmesh_wasm::shapes::Lcg;

const LAYOUT: GroupLayout = GroupLayout {
    width: 48,
    height: 48,
};
const PASSES: usize = 64;

/// A sum of twelve uniforms, close to a Gaussian of unit variance, times `noise`.
fn gaussian(random: &mut Lcg, noise: f64) -> f64 {
    let sum: f64 = (0..12).map(|_| random.next_f64()).sum::<f64>() - 6.0;
    sum * noise
}

/// Passes of `layout` from per-pixel values `mean` plus independent noise, with `tweak` applied
/// to each pixel's value before the groups are formed.
fn noisy_groups(
    layout: GroupLayout,
    mean: f64,
    noise: f64,
    seed: u32,
    tweak: impl Fn(u32, u32, [f64; 3]) -> [f64; 3],
) -> PassGroups {
    let mut random = Lcg::new(seed);
    (0..PASSES)
        .map(|_| {
            let pixels: Vec<[f64; 3]> = (0..layout.height)
                .flat_map(|y| (0..layout.width).map(move |x| (x, y)))
                .map(|(x, y)| tweak(x, y, [0, 1, 2].map(|_| mean + gaussian(&mut random, noise))))
                .collect();
            layout.aggregate(&pixels)
        })
        .collect()
}

fn no_tweak(_: u32, _: u32, value: [f64; 3]) -> [f64; 3] {
    value
}

#[test]
fn two_renders_that_differ_only_by_noise_agree_at_every_scale() {
    let gpu = noisy_groups(LAYOUT, 1.0, 0.1, 1, no_tweak);
    let cpu = noisy_groups(LAYOUT, 1.0, 0.1, 2, no_tweak);
    let result = compare_groups(&gpu, &cpu, &LAYOUT, None);
    assert_eq!(result.regions.exceeded, 0, "{:?}", result.regions);
    assert_eq!(result.lines.exceeded, 0, "{:?}", result.lines);
    assert_eq!(result.pooled.exceeded, 0, "{:?}", result.pooled);
    assert!(result.passed);
    assert!(result.detectable.whole_image > 0.0);
    assert!(result.detectable.whole_image < result.detectable.region_median);
}

#[test]
fn a_deliberately_biased_region_is_caught_in_the_right_channel_and_region() {
    // Noise of 0.1 per sample over the 36 pixels and 64 passes of a region is an error of 0.0021
    // each, 0.003 combined. Region 20 is x 24 to 29, y 12 to 17 of a 48 by 48 image. A 30% bias is
    // far past the threshold however the noise falls.
    let cpu = noisy_groups(LAYOUT, 1.0, 0.1, 1, no_tweak);
    let gpu = noisy_groups(LAYOUT, 1.0, 0.1, 2, |x, y, mut value| {
        if (24..30).contains(&x) && (12..18).contains(&y) {
            value[2] *= 1.3;
        }
        value
    });
    let result = compare_groups(&gpu, &cpu, &LAYOUT, None);
    assert_eq!(result.regions.exceeded, 1, "only region 20: {result:?}");
    assert!(
        result.regions.worst.contains("region 20 channel 2"),
        "names the region and channel: {}",
        result.regions.worst
    );
    assert!(result.regions.max_z > Z_THRESHOLD);
    assert!(!result.passed);
}

#[test]
fn a_leak_along_one_pixel_wide_column_is_caught_where_a_region_would_dilute_it() {
    // A 3% brightening of only the single column x = 17. In its region that is a sixth of it, 0.5%,
    // against a combined error of 0.27%, about two standard errors. In its own column the full 3%
    // stands against a combined error of 0.25%, about twelve.
    let cpu = noisy_groups(LAYOUT, 1.0, 0.1, 1, no_tweak);
    let gpu = noisy_groups(LAYOUT, 1.0, 0.1, 2, |x, _, mut value| {
        if x == 17 {
            value[0] *= 1.03;
        }
        value
    });
    let result = compare_groups(&gpu, &cpu, &LAYOUT, None);
    assert_eq!(
        result.lines.exceeded, 1,
        "only that column: {:?}",
        result.lines
    );
    assert!(
        result.lines.worst.contains("column 17 channel 0"),
        "names the line: {}",
        result.lines.worst
    );
    assert_eq!(
        result.regions.exceeded, 0,
        "the leak is diluted below the region threshold, which is what the line tier is for: {:?}",
        result.regions
    );
}

#[test]
fn a_small_uniform_bias_is_caught_pooled_where_no_region_could_tell() {
    // A 1% bias over the whole image. Noise of 0.5 per sample is a combined error of 1.5% in a
    // region of 36 pixels and 64 passes, so no region can see it, and of 0.18% pooled over all
    // 2304 pixels, about five and a half standard errors.
    let cpu = noisy_groups(LAYOUT, 1.0, 0.5, 1, no_tweak);
    let gpu = noisy_groups(LAYOUT, 1.01, 0.5, 2, no_tweak);
    let result = compare_groups(&gpu, &cpu, &LAYOUT, None);
    assert!(
        result.regions.max_z < Z_THRESHOLD,
        "no single region can see it: {:?}",
        result.regions
    );
    assert!(result.pooled.exceeded >= 1, "{:?}", result.pooled);
    assert!(
        result.pooled.worst.contains("whole image"),
        "the whole image is where it adds up: {:?}",
        result.pooled
    );
    assert!(!result.passed);
}

#[test]
fn a_uniform_bias_hidden_by_one_noisy_spot_is_found_by_the_weighted_fit() {
    // One region with noise forty times the rest (a highlight seen through glass): its error
    // dominates the plain pooled mean, but the weighted fit discounts it and sees the rest.
    let noise = |x: u32, y: u32| {
        if (24..30).contains(&x) && (24..30).contains(&y) {
            20.0
        } else {
            0.5
        }
    };
    let mut random = Lcg::new(7);
    let mut render = |mean: f64| -> PassGroups {
        (0..PASSES)
            .map(|_| {
                let pixels: Vec<[f64; 3]> = (0..LAYOUT.height)
                    .flat_map(|y| (0..LAYOUT.width).map(move |x| (x, y)))
                    .map(|(x, y)| [0, 1, 2].map(|_| mean + gaussian(&mut random, noise(x, y))))
                    .collect();
                LAYOUT.aggregate(&pixels)
            })
            .collect()
    };
    let cpu = render(1.0);
    let gpu = render(1.02);
    let result = compare_groups(&gpu, &cpu, &LAYOUT, None);
    assert!(
        result.uniform.exceeded >= 1 && result.uniform.max_z > 4.5,
        "the fit sees the 2%: {:?}",
        result.uniform
    );
    assert!(
        result.pooled.groups > 0 && result.detectable.uniform_bias < result.detectable.whole_image,
        "and it can see more than the plain mean could: {:?}",
        result.detectable
    );
    assert!(!result.passed);
}

#[test]
fn closed_forms_hold_each_render_to_the_exact_answer() {
    let exact = vec![vec![[1.0_f64; 3]; LAYOUT.group_count()]; PASSES];
    let clean = compare_groups(&exact, &exact, &LAYOUT, Some([1.0; 3]));
    assert_eq!(clean.closed_form_exceeded, 0);
    assert_eq!(clean.cpu_closed_form_exceeded, 0);
    assert!(clean.passed);

    // The GPU off by 0.2%: noiseless, so the arithmetic allowance alone decides it, and it is
    // past that allowance everywhere.
    let bad = vec![vec![[1.002_f64; 3]; LAYOUT.group_count()]; PASSES];
    let result = compare_groups(&bad, &exact, &LAYOUT, Some([1.0; 3]));
    assert_eq!(result.regions.exceeded, 64, "every region of the furnace");
    assert_eq!(
        result.closed_form_exceeded as usize,
        LAYOUT.group_count() * 3,
        "every channel of every group"
    );
    assert_eq!(result.cpu_closed_form_exceeded, 0);
    assert!(!result.passed);

    // An error in both renders, which comparing them can't see, is caught by the closed form.
    let shared = compare_groups(&bad, &bad, &LAYOUT, Some([1.0; 3]));
    assert_eq!(
        shared.regions.exceeded, 0,
        "the two renders agree with each other"
    );
    assert_eq!(
        shared.closed_form_exceeded as usize,
        LAYOUT.group_count() * 3
    );
    assert_eq!(
        shared.cpu_closed_form_exceeded as usize,
        LAYOUT.group_count() * 3,
        "and the CPU's half of the error is reported as its own"
    );
    assert!(!shared.passed);
}

#[test]
fn a_sample_the_gpu_drops_that_the_cpu_did_not_fails_the_check() {
    let exact = vec![vec![[1.0_f64; 3]; LAYOUT.group_count()]; PASSES];
    let compared = compare_groups(&exact, &exact, &LAYOUT, None);
    assert!(compared.passed);
    assert!(with_dropped_samples(compared.clone(), 0, 0).passed);
    assert!(
        !with_dropped_samples(compared.clone(), 1, 0).passed,
        "a dropped GPU sample the CPU kept fails"
    );
    assert!(
        with_dropped_samples(compared, 3, 3).passed,
        "the CPU dropped as many"
    );
}

#[test]
fn gpu_output_of_the_wrong_size_is_refused() {
    let error = gpu_passes(&[0u8; 8], LAYOUT, 64).err().expect("refused");
    assert!(error.contains("bytes of accumulation"), "{error}");
    let pixels = (LAYOUT.width * LAYOUT.height) as usize;
    let exact = vec![0u8; pixels * ACCUMULATION_STRIDE * PASSES];
    let render = gpu_passes(&exact, LAYOUT, 64).expect("the right size");
    assert_eq!(render.passes.len(), PASSES);
    assert_eq!(render.dropped, (pixels * PASSES) as u64);
}

#[test]
fn the_scenes_reach_every_branch_and_the_closed_forms_are_exact_on_the_cpu() {
    let names: Vec<&str> = (0..SCENE_COUNT)
        .map(|i| check_scene(i).expect("each scene builds").name)
        .collect();
    for wanted in [
        "mirror",
        "Smooth glass",
        "non-uniform scale",
        "Orthographic",
        "next-event",
        "surface sampling",
        "1e7",
        "concave lossless",
        "concave index-matched",
    ] {
        assert!(
            names.iter().any(|n| n.contains(wanted)),
            "{wanted}: {names:?}"
        );
    }
    for index in 0..SCENE_COUNT {
        let check = check_scene(index).expect("a scene");
        let Some(answer) = check.closed_form else {
            continue;
        };
        let layout = layout_of(&check);
        let cpu = render_cpu(&check).expect("renders");
        let result = compare_groups(&cpu.passes, &cpu.passes, &layout, Some(answer));
        assert_eq!(
            result.cpu_closed_form_exceeded, 0,
            "{} on the CPU is exactly its closed form: {:?}",
            check.name, result.regions
        );
        assert_eq!(cpu.dropped, 0);
    }
}
