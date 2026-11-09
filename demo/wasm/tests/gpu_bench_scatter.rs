// The GPU Ray Tracing page's scattering check: the fixed batch has every kind of surface the CPU
// model has, the comparison accepts the CPU's own answers rounded to f32 (what a correct shader
// writes) and finds a wrong one in any output, and a shader answer the wrong size is refused in
// plain words. A browser with WebGPU is what really runs the shader; this is everything else.

use solidmesh::mesh::ray_tracer::flat_tree::{
    ScatterCheckCase, ScatterCheckResult, SAMPLE_FOUND, SCATTER_RESULT_STRIDE,
};
use solidmesh_wasm::gpu_bench_scatter::{
    classify, cpu_answer, result_of, sensitivity, standard_batch, ScatterBatch, ScatterMatch,
    RELATIVE_TOLERANCE,
};

fn reference_results(batch: &ScatterBatch) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(batch.cases.len() * SCATTER_RESULT_STRIDE);
    for case in &batch.cases {
        let surface = &batch.surfaces[case.surface as usize].surface;
        bytes.extend_from_slice(&result_of(&cpu_answer(surface, case)).to_le_bytes());
    }
    bytes
}

fn surface_names(batch: &ScatterBatch) -> Vec<&str> {
    batch.surfaces.iter().map(|s| s.name.as_str()).collect()
}

#[test]
fn the_batch_has_every_kind_of_surface_the_model_has() {
    let batch = standard_batch();
    let names = surface_names(&batch);
    for kind in [
        "colored metal",
        "white metal",
        "colored plastic",
        "black plastic",
        "diffuse, no interface",
        "plastic, ior 4",
        "clear glass",
        "tinted glass",
        "glass, ior 2.4",
        "glass, ior 4",
        "index-matched glass",
        "glass, ior 1,",
        "glass, ior 1.0011",
        "glass, ior 1.002",
        "glass, ior 1.03",
        "metal, plastic and glass mixed",
    ] {
        assert!(
            names.iter().any(|name| name.contains(kind)),
            "the batch covers {kind}"
        );
    }
    // Rough enough to be a microfacet lobe, and smooth enough to be a delta lobe, for each.
    let rough: usize = names.iter().filter(|n| n.contains("roughness 1")).count();
    let smooth: usize = names
        .iter()
        .filter(|n| n.contains("roughness 0,") || n.ends_with("roughness 0"))
        .count();
    assert_eq!(rough, 16, "one rough entry per kind");
    assert_eq!(smooth, 16, "one smooth entry per kind");
    assert!(
        batch.cases.len() >= 6000,
        "a broad batch ({})",
        batch.cases.len()
    );
    assert!(
        batch.cases.iter().any(|c| c.wo[2] > 0.0 && c.wo[2] < 2e-6),
        "views right at grazing are included"
    );
    assert!(
        batch.cases.iter().any(|c| c.wo[2] < 0.0),
        "views from below are included"
    );
    assert!(
        batch.cases.iter().any(|c| c.wo == [0.0, 0.0, 0.0]),
        "a view with no direction is included"
    );
}

#[test]
fn the_cpu_answers_rounded_to_f32_are_exact_agreement() {
    let batch = standard_batch();
    let agreement = batch
        .compare(&reference_results(&batch))
        .expect("the reference has one result per case");
    assert_eq!(agreement.compared, batch.cases.len() as u64);
    assert_eq!(agreement.disagreed, 0, "{:?}", agreement.examples);
    assert_eq!(agreement.ties, 0, "the CPU cannot tie with itself");
}

/// An output of a rough lobe with a found, nonspecular sample, so every field has a value.
fn rough_case(batch: &ScatterBatch) -> (&ScatterCheckCase, usize, ScatterCheckResult) {
    let case = batch
        .cases
        .iter()
        .find(|case| {
            let surface = &batch.surfaces[case.surface as usize].surface;
            let answer = cpu_answer(surface, case);
            surface.roughness() > 0.3
                && case.wo[2] > 0.2
                && answer.pdf > 0.0
                && answer.sample.is_some_and(|s| s.pdf > 0.0 && !s.is_specular)
                && answer.evaluate.iter().all(|&v| v > 0.0)
                && answer
                    .sample
                    .is_some_and(|s| s.weight.iter().all(|&w| w > 0.0))
        })
        .expect("a rough case with a sample and color");
    let surface = &batch.surfaces[case.surface as usize].surface;
    (
        case,
        case.surface as usize,
        result_of(&cpu_answer(surface, case)),
    )
}

fn classify_with(
    batch: &ScatterBatch,
    case: &ScatterCheckCase,
    result: &ScatterCheckResult,
) -> ScatterMatch {
    let surface = &batch.surfaces[case.surface as usize].surface;
    let answer = cpu_answer(surface, case);
    let margin = sensitivity(surface, case, &answer);
    classify(case, &answer, &margin, result)
}

#[test]
fn a_wrong_value_in_any_output_is_found_and_named() {
    let batch = standard_batch();
    let (case, _, truth) = rough_case(&batch);
    assert_eq!(classify_with(&batch, case, &truth), ScatterMatch::Agree);

    let wrong_by = 1.0 + 10.0 * RELATIVE_TOLERANCE as f32;
    let mut evaluate = truth;
    evaluate.evaluate[1] *= wrong_by;
    let mut pdf = truth;
    pdf.pdf *= wrong_by;
    let mut weight = truth;
    weight.sample_weight[2] *= wrong_by;
    let mut sample_pdf = truth;
    sample_pdf.sample_pdf *= wrong_by;
    let mut direction = truth;
    direction.sample_direction[0] += 0.01;
    let mut missing = truth;
    missing.flags = 0;
    for (label, bad) in [
        ("evaluate", evaluate),
        ("pdf", pdf),
        ("sample weight", weight),
        ("sample pdf", sample_pdf),
        ("sample direction", direction),
        ("sample:", missing),
    ] {
        match classify_with(&batch, case, &bad) {
            ScatterMatch::Differ(wording) => assert!(
                wording.contains(label) || label == "sample:",
                "{label}: {wording}"
            ),
            other => panic!("{label} corrupted reads as {other:?}"),
        }
    }
    let mut no_found_flag = truth;
    no_found_flag.flags &= !SAMPLE_FOUND;
    assert!(matches!(
        classify_with(&batch, case, &no_found_flag),
        ScatterMatch::Differ(_)
    ));
}

#[test]
fn a_full_compare_counts_a_corrupted_case_and_names_it() {
    let batch = standard_batch();
    let mut bytes = reference_results(&batch);
    let (case, _, _) = rough_case(&batch);
    let index = batch.cases.iter().position(|c| c == case).expect("found");
    let mut record = [0u8; SCATTER_RESULT_STRIDE];
    record.copy_from_slice(
        &bytes[index * SCATTER_RESULT_STRIDE..(index + 1) * SCATTER_RESULT_STRIDE],
    );
    let mut wrong = ScatterCheckResult::from_le_bytes(&record);
    wrong.evaluate[0] *= 1.5;
    bytes[index * SCATTER_RESULT_STRIDE..(index + 1) * SCATTER_RESULT_STRIDE]
        .copy_from_slice(&wrong.to_le_bytes());
    let agreement = batch.compare(&bytes).expect("right size");
    assert_eq!(agreement.disagreed, 1);
    assert!(agreement.examples[0].contains(&format!("case {index} ")));
}

#[test]
fn results_of_the_wrong_size_are_refused_in_plain_words() {
    let batch = standard_batch();
    let error = batch.compare(&[0u8; 7]).expect_err("wrong size");
    assert!(error.contains("7 bytes"), "{error}");
    assert!(error.contains("run the benchmark again"), "{error}");
}
