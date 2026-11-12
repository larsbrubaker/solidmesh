//! Glass parts against glass parts, beyond the touching tray: glass parts that don't touch
//! (apart, nested with a clear gap, a sheet a few millimetres over a box, two boxes a modelled
//! gap apart) and same-glass parts that overlap in volume render the bits they always did, while
//! two boxes stacked on a shared face gain nothing in a white furnace whether their glasses are
//! equal, differ in index either way round, or differ only in colour.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PathTracer, PlacedParts, RenderCamera, RenderScene,
    RenderWork, ScatteringSurface,
};

use crate::glass_tray_support::{box_meshes, part_of, rig_lights};
use crate::path_tracer_golden_tests::render_digest;
use crate::path_tracer_support::flat_patch;
use crate::scattering_support::Stats;

/// How far above 1 a furnace sample may come out from rounding alone.
pub(crate) const FURNACE_SLACK: f64 = 1e-9;

pub(crate) fn furnace() -> LightSetup {
    LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap())
}

fn matte() -> ScatteringSurface {
    ScatteringSurface::new([0.6, 0.55, 0.5], 0.0, 1.0, 0.0, 1.0)
}

fn glass_of(color: [f64; 3], roughness: f64, ior: f64) -> ScatteringSurface {
    ScatteringSurface::new(color, 0.0, roughness, 1.0, ior)
}

/// Boxes of (size, center, surface) as parts, over a matte floor when `floor`.
pub(crate) fn boxes_scene(
    boxes: &[([f64; 3], [f64; 3], ScatteringSurface)],
    floor: bool,
    lights: LightSetup,
) -> RenderScene {
    let shapes: Vec<_> = boxes
        .iter()
        .map(|&(size, center, _)| (size, center))
        .collect();
    let mut parts = Vec::new();
    let mut surfaces = Vec::new();
    if floor {
        parts.push(part_of(&flat_patch(30.0, 0.0), Matrix4X4::IDENTITY));
        surfaces.push(matte());
    }
    for (mesh, &(_, _, surface)) in box_meshes(&shapes).iter().zip(boxes) {
        parts.push(part_of(mesh, Matrix4X4::IDENTITY));
        surfaces.push(surface);
    }
    RenderScene::new(PlacedParts::new(parts), surfaces, lights).unwrap()
}

/// Three glass boxes apart on a floor: clear, rough, and of index 1.7.
fn three_separate_boxes() -> RenderScene {
    boxes_scene(
        &[
            (
                [5.0, 5.0, 5.0],
                [-7.0, 0.0, 2.5],
                ScatteringSurface::glass(),
            ),
            (
                [5.0, 5.0, 4.0],
                [0.0, 5.0, 2.0],
                glass_of([1.0; 3], 0.4, 1.5),
            ),
            (
                [4.0, 6.0, 6.0],
                [7.0, -3.0, 3.0],
                glass_of([0.9, 1.0, 0.8], 0.0, 1.7),
            ),
        ],
        true,
        rig_lights(),
    )
}

/// A box of index 1.5 holding a tinted box of index 1.7 with a clear gap of 2 all round.
fn nested_with_a_gap() -> RenderScene {
    boxes_scene(
        &[
            (
                [16.0, 16.0, 8.0],
                [0.0, 0.0, 5.0],
                ScatteringSurface::glass(),
            ),
            (
                [8.0, 8.0, 4.0],
                [0.0, 0.0, 5.0],
                glass_of([0.8, 0.9, 1.0], 0.0, 1.7),
            ),
        ],
        true,
        rig_lights(),
    )
}

/// An open glass sheet 7 above the top of a closed glass box of the same glass.
fn sheet_over_a_box() -> RenderScene {
    let sheet = flat_patch(10.0, 12.0);
    let boxes = box_meshes(&[([10.0, 10.0, 4.0], [0.0, 0.0, 3.0])]);
    let parts = vec![
        part_of(&flat_patch(30.0, 0.0), Matrix4X4::IDENTITY),
        part_of(&sheet, Matrix4X4::IDENTITY),
        part_of(&boxes[0], Matrix4X4::IDENTITY),
    ];
    let glass = ScatteringSurface::glass();
    RenderScene::new(
        PlacedParts::new(parts),
        vec![matte(), glass, glass],
        rig_lights(),
    )
    .unwrap()
}

/// Two boxes of the same glass stacked with a modelled gap of 0.05 between them.
fn boxes_a_gap_apart() -> RenderScene {
    let glass = ScatteringSurface::glass();
    boxes_scene(
        &[
            ([10.0, 10.0, 4.0], [0.0, 0.0, 3.0], glass),
            ([10.0, 10.0, 4.0], [0.0, 0.0, 7.05], glass),
        ],
        true,
        rig_lights(),
    )
}

/// Two boxes of the same glass overlapping in volume.
fn overlapping_boxes() -> RenderScene {
    let glass = ScatteringSurface::glass();
    boxes_scene(
        &[
            ([10.0, 10.0, 10.0], [-4.0, 0.0, 6.0], glass),
            ([10.0, 10.0, 10.0], [4.0, 0.0, 6.0], glass),
        ],
        true,
        rig_lights(),
    )
}

/// Pinned from the path tracer before glass parts could touch, where no glass part lies on
/// another: these scenes keep the same bits. The three separate boxes rest on the floor, so their
/// bits are those of glass leaving through its own surface onto the floor (`glass_rest`).
const THREE_SEPARATE_BOXES_DIGEST: u64 = 0xe03a_f854_e0e9_c1be;
const NESTED_WITH_A_GAP_DIGEST: u64 = 0x1886_fe58_a0e1_7412;
const SHEET_OVER_A_BOX_DIGEST: u64 = 0xb4e0_2094_9cb1_ac19;
const BOXES_A_GAP_APART_DIGEST: u64 = 0xeb76_7a2a_0972_c151;
const OVERLAPPING_BOXES_DIGEST: u64 = 0x5570_a464_01b9_78fa;

#[test]
fn separate_glass_parts_render_as_they_always_did() {
    let digests = [
        render_digest(&three_separate_boxes()),
        render_digest(&nested_with_a_gap()),
    ];
    println!("separate glass digests {digests:#018x?}");
    assert_eq!(
        digests,
        [THREE_SEPARATE_BOXES_DIGEST, NESTED_WITH_A_GAP_DIGEST],
        "got {digests:#018x?}"
    );
}

/// An open sheet a few millimetres over a box is not in contact with it: the box stays.
#[test]
fn a_glass_sheet_over_a_glass_box_renders_as_it_always_did() {
    let digest = render_digest(&sheet_over_a_box());
    assert_eq!(digest, SHEET_OVER_A_BOX_DIGEST, "got {digest:#018x}");
}

/// A gap of 0.05 is far wider than any rounding: it is a real gap of air and stays one.
#[test]
fn glass_boxes_a_modelled_gap_apart_render_as_they_always_did() {
    let digest = render_digest(&boxes_a_gap_apart());
    assert_eq!(digest, BOXES_A_GAP_APART_DIGEST, "got {digest:#018x}");
}

/// Same-glass parts overlapping in volume enter twice and leave twice, so they gain nothing,
/// and their inner faces still show, as they always have.
#[test]
fn overlapping_glass_boxes_gain_nothing_and_render_as_they_always_did() {
    let glass = ScatteringSurface::glass();
    let scene = boxes_scene(
        &[
            ([10.0, 10.0, 10.0], [-4.0, 0.0, 0.0], glass),
            ([10.0, 10.0, 10.0], [4.0, 0.0, 0.0], glass),
        ],
        false,
        furnace(),
    );
    let camera = RenderCamera::orthographic(
        Vector3::new(-20.0, -20.0, 20.0),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        14.0,
    )
    .unwrap();
    let (most, _) = furnace_samples(&scene, camera);
    assert!(
        most <= 1.0 + FURNACE_SLACK,
        "a furnace sample through overlapping boxes came out at {most}"
    );
    let digest = render_digest(&overlapping_boxes());
    assert_eq!(digest, OVERLAPPING_BOXES_DIGEST, "got {digest:#018x}");
}

/// The brightest channel of any sample of a 16 by 16 view of `scene` with roulette off, and the
/// statistics of every channel of every sample.
pub(crate) fn furnace_samples(scene: &RenderScene, camera: RenderCamera) -> (f64, Stats) {
    let settings = PathTraceSettings {
        russian_roulette_after: None,
        ..PathTraceSettings::default()
    };
    let tracer = PathTracer::new(scene, camera, 16, 16, settings).unwrap();
    let mut work = RenderWork::default();
    let mut stats = Stats::default();
    let mut most = 0.0_f64;
    for y in 0..16 {
        for x in 0..16 {
            for sample in 0..16 {
                for channel in tracer.sample_pixel(x, y, sample, &mut work).unwrap() {
                    most = most.max(channel);
                    stats.add(channel);
                }
            }
        }
    }
    (most, stats)
}

/// Two 10 by 10 by 4 boxes, `lower` under `upper`, sharing the face at z = 4, seen straight down
/// in a white furnace: no sample carries more than the light it started in, and the mean is 1
/// less only what the bounce limit and a colour cut off.
fn check_stacked(lower: ScatteringSurface, upper: ScatteringSurface, what: &str) {
    let scene = boxes_scene(
        &[
            ([10.0, 10.0, 4.0], [0.0, 0.0, 2.0], lower),
            ([10.0, 10.0, 4.0], [0.0, 0.0, 6.0], upper),
        ],
        false,
        furnace(),
    );
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 40.0),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        8.0,
    )
    .unwrap();
    let (most, stats) = furnace_samples(&scene, camera);
    assert!(
        most <= 1.0 + FURNACE_SLACK,
        "{what}: a furnace sample came out at {most}, above the light it started in"
    );
    // A tint of 0.999 a pass loses a few thousandths over the passes a path makes.
    let tolerance = 5.0 * stats.standard_error() + 0.005;
    assert!(
        (stats.mean() - 1.0).abs() <= tolerance,
        "{what}: the furnace's mean came out at {} (tolerance {tolerance})",
        stats.mean()
    );
}

#[test]
fn stacked_glasses_of_index_1_5_over_1_9_gain_nothing_in_a_furnace() {
    let clear = [1.0; 3];
    check_stacked(
        glass_of(clear, 0.0, 1.9),
        glass_of(clear, 0.0, 1.5),
        "1.5 over 1.9",
    );
}

#[test]
fn stacked_glasses_of_index_1_9_over_1_5_gain_nothing_in_a_furnace() {
    let clear = [1.0; 3];
    check_stacked(
        glass_of(clear, 0.0, 1.5),
        glass_of(clear, 0.0, 1.9),
        "1.9 over 1.5",
    );
}

#[test]
fn stacked_glasses_differing_only_in_colour_gain_nothing_in_a_furnace() {
    check_stacked(
        glass_of([1.0, 1.0, 0.999], 0.0, 1.5),
        glass_of([1.0; 3], 0.0, 1.5),
        "a colour of [1, 1, 0.999] under clear",
    );
}

#[test]
fn stacked_glasses_alike_gain_nothing_in_a_furnace() {
    let glass = ScatteringSurface::glass();
    check_stacked(glass, glass, "the same glass");
}
