// The GPU Ray Tracing page's Rust half: its ray sets are valid for both tracers, the comparison
// against the reference is exact on the reference's own answers and finds a wrong one, the size
// refusal reaches the page in plain words, and the picture is drawn from hits. A browser with
// WebGPU is the only real test of the shader's results; this is everything that does not need one.

use solidmesh_wasm::gpu_bench::{Bench, RayKind, IMAGE_HEIGHT, IMAGE_WIDTH};
use solidmesh_wasm::gpu_bench_compare::{classify_closest, HitMatch, TIE_TOLERANCE};
use solidmesh_wasm::ray_trace_scene::DETAIL_TRIANGLES;

use solidmesh::mesh::ray_tracer::flat_tree::{
    hit_from_le_bytes, hit_to_le_bytes, FlatHit, HIT_STRIDE, RAY_STRIDE,
    WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};

const TORUS: u32 = 1;

fn counting_clock() -> impl FnMut() -> f64 {
    let mut ms = 0.0;
    move || {
        ms += 1.0;
        ms
    }
}

fn small_bench() -> Bench {
    Bench::new(
        TORUS,
        0,
        WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        &mut counting_clock(),
    )
    .expect("a small scene fits")
}

/// The reference's own answers as GPU hit records.
fn reference_hits(bench: &Bench, kind: RayKind) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(bench.rays(kind).len() * HIT_STRIDE);
    for ray in bench.rays(kind) {
        let hit = if kind.is_any_hit() {
            bench.tree().trace_any(ray).then_some(FlatHit {
                t: 1.0,
                triangle: 0,
                source_index: 0,
                barycentric: [1.0, 0.0, 0.0],
                front_face: true,
            })
        } else {
            bench.tree().trace_closest(ray)
        };
        bytes.extend_from_slice(&hit_to_le_bytes(hit.as_ref()));
    }
    bytes
}

#[test]
fn the_scene_has_the_promised_size_and_one_camera_ray_per_pixel() {
    let bench = small_bench();
    assert!(bench.info.triangles as f64 >= DETAIL_TRIANGLES[0] * 0.7);
    let width_height = u64::from(IMAGE_WIDTH) * u64::from(IMAGE_HEIGHT);
    assert_eq!(bench.rays(RayKind::Camera).len() as u64, width_height);
    assert_eq!(
        bench.ray_bytes(RayKind::Camera).len(),
        bench.rays(RayKind::Camera).len() * RAY_STRIDE
    );
    // Most of the view is the part and its floor, so shadow and bounce rays are plentiful.
    assert!(bench.rays(RayKind::Bounce).len() > 100_000);
    assert!(bench.rays(RayKind::Shadow).len() > 10_000);
}

#[test]
fn the_reference_agrees_with_itself_on_every_ray_set() {
    let bench = small_bench();
    for kind in RayKind::ALL {
        let agreement = bench
            .compare(kind, &reference_hits(&bench, kind))
            .expect("the byte count is right");
        assert_eq!(agreement.compared, bench.rays(kind).len() as u64);
        assert_eq!(agreement.exact, agreement.compared, "{kind:?}");
        assert_eq!(agreement.disagreed, 0, "{kind:?}");
    }
}

#[test]
fn a_wrong_hit_is_reported_with_the_ray_and_both_answers() {
    let bench = small_bench();
    let mut hits = reference_hits(&bench, RayKind::Camera);
    let first_hit_index = (0..bench.rays(RayKind::Camera).len())
        .find(|i| {
            let mut record = [0u8; HIT_STRIDE];
            record.copy_from_slice(&hits[i * HIT_STRIDE..(i + 1) * HIT_STRIDE]);
            hit_from_le_bytes(&record).is_some()
        })
        .expect("the camera sees something");
    // The GPU says it found nothing where the reference found a triangle.
    hits[first_hit_index * HIT_STRIDE..(first_hit_index + 1) * HIT_STRIDE]
        .copy_from_slice(&hit_to_le_bytes(None));
    let agreement = bench
        .compare(RayKind::Camera, &hits)
        .expect("the byte count is right");
    assert_eq!(agreement.disagreed, 1);
    assert_eq!(agreement.examples.len(), 1);
    assert!(agreement.examples[0].contains(&format!("ray {first_hit_index}")));
    assert!(agreement.examples[0].contains("GPU nothing"));
}

#[test]
fn a_wrong_byte_count_is_an_error_not_a_panic() {
    let bench = small_bench();
    assert!(bench.compare(RayKind::Camera, &[0u8; 28]).is_err());
}

#[test]
fn a_different_triangle_at_the_same_distance_is_a_tie_and_elsewhere_a_disagreement() {
    let hit = |t: f32, triangle: u32| FlatHit {
        t,
        triangle,
        source_index: triangle,
        barycentric: [1.0, 0.0, 0.0],
        front_face: true,
    };
    let near = 10.0;
    assert_eq!(
        classify_closest(Some(&hit(near, 1)), Some(&hit(near, 1))),
        HitMatch::Exact
    );
    assert_eq!(
        classify_closest(
            Some(&hit(near, 1)),
            Some(&hit(near * (1.0 + TIE_TOLERANCE * 0.5), 2))
        ),
        HitMatch::Tie
    );
    assert_eq!(
        classify_closest(
            Some(&hit(near, 1)),
            Some(&hit(near * (1.0 + TIE_TOLERANCE * 10.0), 2))
        ),
        HitMatch::Differ
    );
    assert_eq!(
        classify_closest(Some(&hit(near, 1)), None),
        HitMatch::Differ
    );
    assert_eq!(classify_closest(None, None), HitMatch::Exact);
}

#[test]
fn a_scene_past_the_binding_limit_says_so_in_the_trees_own_words() {
    let refusal = Bench::new(TORUS, 0, 1024, &mut counting_clock())
        .err()
        .expect("too big for a tiny limit");
    assert!(refusal.contains("buffer would need"), "{refusal}");
    assert!(refusal.contains("simplify the scene"), "{refusal}");
}

#[test]
fn the_picture_is_one_opaque_pixel_per_camera_ray_drawn_from_hits() {
    let bench = small_bench();
    let pixels = bench
        .picture(
            &reference_hits(&bench, RayKind::Camera),
            &reference_hits(&bench, RayKind::Shadow),
        )
        .expect("the byte counts are right");
    assert_eq!(
        pixels.len() as u64,
        u64::from(IMAGE_WIDTH) * u64::from(IMAGE_HEIGHT) * 4
    );
    let (rgba, _) = pixels.as_chunks::<4>();
    assert!(rgba.iter().all(|p| p[3] == 255));
    // Not one flat colour: the part, its shading and the floor show up.
    let distinct: std::collections::BTreeSet<[u8; 3]> =
        rgba.iter().map(|p| [p[0], p[1], p[2]]).collect();
    assert!(distinct.len() > 100, "{} colours", distinct.len());
}

#[test]
fn the_reference_rate_counts_every_ray_it_traced() {
    let bench = small_bench();
    let (traced, ms) = bench.reference_run(RayKind::Shadow, 0.5, &mut counting_clock());
    assert_eq!(traced, bench.rays(RayKind::Shadow).len() as u64);
    assert!(ms > 0.0);
    // A longer budget traces the whole set repeatedly.
    let (more, _) = bench.reference_run(RayKind::Shadow, 3.5, &mut counting_clock());
    assert_eq!(more % traced, 0);
    assert!(more > traced);
}
