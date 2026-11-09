//! The render check's statistics: from per-pass, per-pixel radiance, the mean of every group of
//! pixels (the coarse grid of regions, each one-pixel-wide row and column, coarse bands, and the
//! whole image), and how two renders' group means compare against the standard error measured
//! from the passes themselves. `gpu_bench_render` feeds it and the demo page reports it.
//!
//! **Why three tiers.** A region alone is both blunt and diluted: a uniform darkening of 3% hides
//! inside the noise of a glossy region, and a leak along one row of pixels is diluted sixfold by a
//! six-pixel region. So every comparison is tested at three scales:
//!
//! - *Regions*, an 8 by 8 grid: where in the picture a difference is.
//! - *Lines*, every one-pixel-wide row and column: a leak along an edge or a seam keeps its full
//!   size in the line that holds it, not a sixth of it.
//! - *Pooled*, the whole image and four horizontal and four vertical bands: a small bias over much
//!   of the picture adds up here, against an error that shrinks as more pixels are pooled.
//!
//! **The thresholds**, fixed here before any result and never widened to pass. A group's z is
//! the difference over the two renders' combined standard error. With the measured error having
//! about 126 degrees of freedom at the smallest sample count used (the two renders' 64 passes each,
//! combined), the chance of `|t| > 5` is about 2e-6 and of `|t| > 4.5` about 1.5e-5. Regions and
//! lines use [`Z_THRESHOLD`] = 5 and pooled groups [`POOLED_Z_THRESHOLD`] = 4.5, with fewer tests
//! and a stronger reason to look. Over the page's fifteen scenes that is about 2,900 region tests,
//! 4,300 line tests and 400 pooled tests, so under an exact match about 1 run in 50 would see a
//! single false alarm, almost all from the lines. The seed is fixed, so a given GPU sees the same
//! answer every time.
//!
//! [`ARITHMETIC_ALLOWANCE`] adds the part of a difference that is arithmetic and not noise: a
//! path's value passes through the same f32 chain the scattering check budgeted at `256 *
//! f32::EPSILON` (`gpu_bench_scatter`). A furnace has zero sample noise, so this is all its
//! comparison and its closed-form check rest on.
//!
//! **A uniform bias, weighted by what each region can say.** The pooled mean weighs every pixel
//! alike, so one bright, noisy spot (a specular highlight seen through glass) dominates its error
//! and hides a uniform 2% everywhere else. So each channel also fits the single relative bias `b`
//! that best explains every region's difference `d = b * m`, weighting each region by what its own
//! standard error `s` allows: the best linear estimate is `sum(m d / s^2) / sum(m^2 / s^2)`, whose
//! z is `sum(m d / s^2) / sqrt(sum(m^2 / s^2))`. No single region can dominate its error, because
//! each `s` is floored at a quarter of the median (a measured `s` that happens to be small is not
//! allowed to claim a huge weight) and at the arithmetic allowance, so a noiseless furnace is well
//! defined. It runs at [`POOLED_Z_THRESHOLD`], three tests per scene.
//!
//! **Dropped samples.** A sample the GPU dropped for being not finite is a sample the estimator
//! lost, so it fails the verdict unless the CPU dropped as many on the same scene: the two may
//! legitimately overflow together, but a GPU that drops what the CPU keeps is wrong.

/// See the module header.
pub const Z_THRESHOLD: f64 = 5.0;

/// See the module header.
pub const POOLED_Z_THRESHOLD: f64 = 4.5;

/// See the module header: the scattering check's own f32 budget.
pub const ARITHMETIC_ALLOWANCE: f64 = 256.0 * f32::EPSILON as f64;

/// How many regions across and down the comparison grid has.
pub const REGION_GRID: u32 = 8;

/// A region's standard error is floored at this fraction of the median, before weighting.
const WEIGHT_FLOOR_OF_MEDIAN: f64 = 0.25;

/// How many horizontal bands and how many vertical bands.
const BAND_COUNT: u32 = 4;

/// Per pass, per group, the mean linear RGB: see [`GroupLayout`] for the order of the groups.
pub type PassGroups = Vec<Vec<[f64; 3]>>;

/// Which pixels form which groups, for an image of a given size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroupLayout {
    pub width: u32,
    pub height: u32,
}

impl GroupLayout {
    fn regions(&self) -> usize {
        (REGION_GRID * REGION_GRID) as usize
    }

    fn lines(&self) -> usize {
        (self.height + self.width) as usize
    }

    fn bands(&self) -> usize {
        (BAND_COUNT * 2) as usize
    }

    /// How many groups the layout has: regions, rows, columns, bands, and the whole image last.
    pub fn group_count(&self) -> usize {
        self.regions() + self.lines() + self.bands() + 1
    }

    fn band_base(&self) -> usize {
        self.regions() + self.lines()
    }

    /// The groups pixel (`x`, `y`) belongs to, one of each kind.
    fn groups_of(&self, x: u32, y: u32) -> [usize; 6] {
        let (w, h) = (self.width, self.height);
        let region = ((y * REGION_GRID / h) * REGION_GRID + x * REGION_GRID / w) as usize;
        let row = self.regions() + y as usize;
        let column = self.regions() + h as usize + x as usize;
        let horizontal = self.band_base() + (y * BAND_COUNT / h) as usize;
        let vertical = self.band_base() + (BAND_COUNT + x * BAND_COUNT / w) as usize;
        [
            region,
            row,
            column,
            horizontal,
            vertical,
            self.group_count() - 1,
        ]
    }

    /// A group's name in a report.
    pub fn group_name(&self, group: usize) -> String {
        let regions = self.regions();
        let height = self.height as usize;
        let band_base = self.band_base();
        if group < regions {
            format!("region {group}")
        } else if group < regions + height {
            format!("row {}", group - regions)
        } else if group < band_base {
            format!("column {}", group - regions - height)
        } else if group < band_base + BAND_COUNT as usize {
            format!("horizontal band {}", group - band_base)
        } else if group < band_base + (2 * BAND_COUNT) as usize {
            format!("vertical band {}", group - band_base - BAND_COUNT as usize)
        } else {
            "whole image".to_string()
        }
    }

    /// The mean of every group for one pass, from `pixels` in row order. A pixel's group means
    /// divide by the count of pixels that group holds.
    pub fn aggregate(&self, pixels: &[[f64; 3]]) -> Vec<[f64; 3]> {
        let mut sums = vec![[0.0_f64; 3]; self.group_count()];
        let mut counts = vec![0_u32; self.group_count()];
        for y in 0..self.height {
            for x in 0..self.width {
                let value = pixels[(y * self.width + x) as usize];
                for group in self.groups_of(x, y) {
                    counts[group] += 1;
                    for (sum, channel) in sums[group].iter_mut().zip(value) {
                        *sum += channel;
                    }
                }
            }
        }
        for (sum, count) in sums.iter_mut().zip(counts) {
            *sum = sum.map(|channel| channel / f64::from(count.max(1)));
        }
        sums
    }
}

/// How one tier of the comparison came out.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TierResult {
    pub groups: u32,
    pub exceeded: u32,
    pub max_z: f64,
    /// The group and channel with the largest z, in words.
    pub worst: String,
}

/// The smallest bias each check of a scene could tell from noise, as a fraction of the mean: the
/// threshold in standard errors, over the mean of the group.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DetectableBias {
    /// The whole image, the worst channel.
    pub whole_image: f64,
    /// A uniform relative bias, by the weighted fit, the worst channel.
    pub uniform_bias: f64,
    pub region_median: f64,
    pub region_p75: f64,
    pub region_max: f64,
}

/// What comparing one scene's two renders found.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RenderCheckResult {
    pub name: String,
    pub regions: TierResult,
    pub lines: TierResult,
    pub pooled: TierResult,
    /// The weighted fit of one uniform relative bias per channel: see the module header.
    pub uniform: TierResult,
    /// Channel-groups where the GPU differs from the exact answer.
    pub closed_form_exceeded: u32,
    /// Channel-groups where the CPU reference differs from the exact answer, so an error the two
    /// share is caught too.
    pub cpu_closed_form_exceeded: u32,
    pub mean_relative_difference: f64,
    pub detectable: DetectableBias,
    pub gpu_dropped: u64,
    pub cpu_dropped: u64,
    /// Every tier is clean and the GPU dropped no more samples than the CPU did.
    pub passed: bool,
}

#[derive(Debug, Clone, Copy)]
struct GroupMean {
    mean: f64,
    standard_error: f64,
}

/// The mean over passes of one group's channel, and the standard error of that mean from the
/// passes' own spread.
fn group_mean(passes: &PassGroups, group: usize, channel: usize) -> GroupMean {
    let n = passes.len();
    let mean = passes.iter().map(|p| p[group][channel]).sum::<f64>() / n as f64;
    let variance = passes
        .iter()
        .map(|p| (p[group][channel] - mean) * (p[group][channel] - mean))
        .sum::<f64>()
        / (n - 1).max(1) as f64;
    GroupMean {
        mean,
        standard_error: (variance / n as f64).sqrt(),
    }
}

/// One group and channel of two renders against each other.
struct ChannelTest {
    z: f64,
    exceeded: bool,
    words: String,
    detectable: f64,
    relative: f64,
}

fn test_channel(
    a: &PassGroups,
    b: &PassGroups,
    layout: &GroupLayout,
    group: usize,
    channel: usize,
    threshold: f64,
) -> ChannelTest {
    let first = group_mean(a, group, channel);
    let second = group_mean(b, group, channel);
    let difference = (first.mean - second.mean).abs();
    let se = (first.standard_error.powi(2) + second.standard_error.powi(2)).sqrt();
    let magnitude = first.mean.abs().max(second.mean.abs());
    let allowance = ARITHMETIC_ALLOWANCE * magnitude;
    // A combined error of zero (the furnace) has no z of its own: the allowance decides.
    let z = if se > 0.0 {
        difference / se
    } else if difference <= allowance {
        0.0
    } else {
        f64::INFINITY
    };
    ChannelTest {
        z,
        exceeded: difference > threshold * se + allowance,
        words: format!(
            "{} channel {channel}: GPU {:.5}, CPU {:.5}, difference {:.5}, combined standard error {:.5}",
            layout.group_name(group),
            first.mean,
            second.mean,
            difference,
            se
        ),
        detectable: if magnitude > 0.0 {
            (threshold * se + allowance) / magnitude
        } else {
            0.0
        },
        relative: if magnitude > 0.0 {
            difference / magnitude
        } else {
            0.0
        },
    }
}

/// The tier of groups `range`, and each channel's detectable bias.
fn compare_tier(
    a: &PassGroups,
    b: &PassGroups,
    layout: &GroupLayout,
    range: std::ops::Range<usize>,
    threshold: f64,
    detectable: &mut Vec<f64>,
    relative: &mut (f64, u64),
) -> TierResult {
    let mut tier = TierResult {
        groups: range.len() as u32,
        ..TierResult::default()
    };
    for group in range {
        let mut exceeded = false;
        for channel in 0..3 {
            let test = test_channel(a, b, layout, group, channel, threshold);
            exceeded |= test.exceeded;
            detectable.push(test.detectable);
            relative.0 += test.relative;
            relative.1 += 1;
            if test.z > tier.max_z || tier.worst.is_empty() {
                tier.max_z = test.z;
                tier.worst = test.words;
            }
        }
        if exceeded {
            tier.exceeded += 1;
        }
    }
    tier
}

/// The tier fitting one uniform relative bias per channel over the regions, and its detectable
/// size, the worst channel.
fn compare_uniform_bias(
    gpu: &PassGroups,
    cpu: &PassGroups,
    layout: &GroupLayout,
) -> (TierResult, f64) {
    let mut tier = TierResult {
        groups: 3,
        ..TierResult::default()
    };
    let mut worst_detectable = 0.0_f64;
    for channel in 0..3 {
        let measured: Vec<(f64, f64, f64)> = (0..layout.regions())
            .map(|region| {
                let first = group_mean(gpu, region, channel);
                let second = group_mean(cpu, region, channel);
                let se = (first.standard_error.powi(2) + second.standard_error.powi(2)).sqrt();
                (
                    first.mean.abs().max(second.mean.abs()),
                    first.mean - second.mean,
                    se,
                )
            })
            .filter(|(magnitude, _, _)| *magnitude > 0.0)
            .collect();
        let mut errors: Vec<f64> = measured.iter().map(|(_, _, se)| *se).collect();
        let floor = percentile(&mut errors, 0.5) * WEIGHT_FLOOR_OF_MEDIAN;
        let (mut numerator, mut denominator) = (0.0_f64, 0.0_f64);
        for (magnitude, difference, se) in &measured {
            let se = se.max(floor).max(ARITHMETIC_ALLOWANCE * magnitude);
            numerator += magnitude * difference / (se * se);
            denominator += magnitude * magnitude / (se * se);
        }
        if denominator <= 0.0 {
            continue;
        }
        let z = numerator / denominator.sqrt();
        let detectable = POOLED_Z_THRESHOLD / denominator.sqrt();
        worst_detectable = worst_detectable.max(detectable);
        if z.abs() > POOLED_Z_THRESHOLD {
            tier.exceeded += 1;
        }
        if z.abs() > tier.max_z || tier.worst.is_empty() {
            tier.max_z = z.abs();
            tier.worst = format!(
                "channel {channel}: estimated uniform bias {:.3}%, z {:.2}",
                numerator / denominator * 100.0,
                z
            );
        }
    }
    (tier, worst_detectable)
}

/// The value at `fraction` of the sorted `values`, zero when there are none.
fn percentile(values: &mut [f64], fraction: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    // The fraction is in 0..=1 and the length a slice's, so the cast stays in range.
    let index = (fraction * (values.len() - 1) as f64).round() as usize;
    values[index]
}

/// How many channel-groups of `passes` differ from the exact `answer` by more than the region
/// threshold in standard errors plus the arithmetic allowance.
fn closed_form_misses(passes: &PassGroups, layout: &GroupLayout, answer: [f64; 3]) -> u32 {
    let mut misses = 0;
    for group in 0..layout.group_count() {
        for (channel, exact) in answer.into_iter().enumerate() {
            let measured = group_mean(passes, group, channel);
            let limit = Z_THRESHOLD * measured.standard_error + ARITHMETIC_ALLOWANCE * exact;
            if (measured.mean - exact).abs() > limit {
                misses += 1;
            }
        }
    }
    misses
}

/// `gpu` against `cpu` over every group at the thresholds, and each against `closed_form` when
/// the exact answer is known.
pub fn compare_groups(
    gpu: &PassGroups,
    cpu: &PassGroups,
    layout: &GroupLayout,
    closed_form: Option<[f64; 3]>,
) -> RenderCheckResult {
    let band_base = layout.band_base();
    let regions_end = layout.regions();
    let lines_end = band_base;
    let mut relative = (0.0_f64, 0_u64);

    let mut region_detectable = Vec::new();
    let regions = compare_tier(
        gpu,
        cpu,
        layout,
        0..regions_end,
        Z_THRESHOLD,
        &mut region_detectable,
        &mut relative,
    );
    let mut line_detectable = Vec::new();
    let lines = compare_tier(
        gpu,
        cpu,
        layout,
        regions_end..lines_end,
        Z_THRESHOLD,
        &mut line_detectable,
        &mut relative,
    );
    let mut pooled_detectable = Vec::new();
    let pooled = compare_tier(
        gpu,
        cpu,
        layout,
        band_base..layout.group_count(),
        POOLED_Z_THRESHOLD,
        &mut pooled_detectable,
        &mut relative,
    );

    let whole_group = layout.group_count() - 1;
    let whole_image = (0..3)
        .map(|channel| test_channel(gpu, cpu, layout, whole_group, channel, POOLED_Z_THRESHOLD))
        .map(|test| test.detectable)
        .fold(0.0_f64, f64::max);
    let (uniform, uniform_bias) = compare_uniform_bias(gpu, cpu, layout);
    let detectable = DetectableBias {
        whole_image,
        uniform_bias,
        region_median: percentile(&mut region_detectable, 0.5),
        region_p75: percentile(&mut region_detectable, 0.75),
        region_max: percentile(&mut region_detectable, 1.0),
    };

    let (mut closed_form_exceeded, mut cpu_closed_form_exceeded) = (0, 0);
    if let Some(answer) = closed_form {
        closed_form_exceeded = closed_form_misses(gpu, layout, answer);
        cpu_closed_form_exceeded = closed_form_misses(cpu, layout, answer);
    }

    RenderCheckResult {
        name: String::new(),
        passed: regions.exceeded == 0
            && lines.exceeded == 0
            && pooled.exceeded == 0
            && uniform.exceeded == 0
            && closed_form_exceeded == 0
            && cpu_closed_form_exceeded == 0,
        regions,
        lines,
        pooled,
        uniform,
        closed_form_exceeded,
        cpu_closed_form_exceeded,
        mean_relative_difference: if relative.1 > 0 {
            relative.0 / relative.1 as f64
        } else {
            0.0
        },
        detectable,
        gpu_dropped: 0,
        cpu_dropped: 0,
    }
}

/// `result` once the dropped-sample counts are known: a GPU that dropped more than the CPU fails
/// (see the module header).
pub fn with_dropped_samples(
    mut result: RenderCheckResult,
    gpu_dropped: u64,
    cpu_dropped: u64,
) -> RenderCheckResult {
    result.gpu_dropped = gpu_dropped;
    result.cpu_dropped = cpu_dropped;
    result.passed = result.passed && gpu_dropped <= cpu_dropped;
    result
}
