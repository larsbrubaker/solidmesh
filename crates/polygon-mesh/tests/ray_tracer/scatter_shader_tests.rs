//! The scattering model's WGSL: both sources parse and validate (a shader that does not compile
//! fails here and not only in a browser), its structs are byte for byte the Rust records, its
//! constants are the Rust model's, and the surface and albedo buffers hold what the shader reads.
//! Whether its answers match the CPU is the demo's GPU page, which runs it.

use naga::valid::{Capabilities, ValidationFlags, Validator};
use naga::{ArraySize, Expression, Literal, MathFunction, Module, ShaderStage, TypeInner};
use solidmesh_mesh::ray_tracer::flat_tree::{
    albedo_table_bytes, ScatterCheckCase, ScatterCheckResult, SurfaceRecord, ALBEDO_ENTRY_COUNT,
    ALBEDO_ENTRY_STRIDE, SAMPLE_FOUND, SAMPLE_SPECULAR, SCATTER_CASE_STRIDE, SCATTER_CHECK_WGSL,
    SCATTER_RESULT_STRIDE, SCATTER_WGSL, SURFACE_HAS_GLASS, SURFACE_INDEX_MATCHED,
    SURFACE_RECORD_STRIDE, SURFACE_SMOOTH,
};
use solidmesh_mesh::ray_tracer::ScatteringSurface;

use super::flat_tree_shader_tests::{assert_layout, literal};

fn parsed(source: &str) -> Module {
    naga::front::wgsl::parse_str(source).expect("the shader parses as WGSL")
}

fn validate(module: &Module) {
    Validator::new(ValidationFlags::all(), Capabilities::empty())
        .validate(module)
        .expect("the shader validates");
}

#[test]
fn the_library_validates_and_exposes_the_three_operations() {
    let module = parsed(SCATTER_WGSL);
    validate(&module);
    assert!(module.entry_points.is_empty(), "a library, no entry point");
    let names: Vec<&str> = module
        .functions
        .iter()
        .filter_map(|(_, f)| f.name.as_deref())
        .collect();
    for wanted in ["surface_evaluate", "surface_pdf", "surface_sample"] {
        assert!(names.contains(&wanted), "the library has {wanted}");
    }
}

#[test]
fn the_check_validates_with_its_entry_point() {
    let module = parsed(SCATTER_CHECK_WGSL);
    validate(&module);
    let names: Vec<&str> = module
        .entry_points
        .iter()
        .filter(|e| e.stage == ShaderStage::Compute)
        .map(|e| e.name.as_str())
        .collect();
    assert_eq!(names, ["scatter_check_main"]);
}

#[test]
fn the_records_match_the_buffers_byte_for_byte() {
    let module = parsed(SCATTER_CHECK_WGSL);
    assert_layout(
        &module,
        "Surface",
        &[
            ("base_r", 0),
            ("base_g", 4),
            ("base_b", 8),
            ("metallic", 12),
            ("roughness", 16),
            ("transmission", 20),
            ("ior", 24),
            ("ior_minus_one", 28),
            ("flags", 32),
        ],
        SURFACE_RECORD_STRIDE as u32,
    );
    assert_layout(
        &module,
        "AlbedoEntry",
        &[("a", 0), ("b", 4), ("loss", 8)],
        ALBEDO_ENTRY_STRIDE as u32,
    );
    assert_layout(
        &module,
        "ScatterCase",
        &[
            ("surface", 0),
            ("wo_x", 4),
            ("wo_y", 8),
            ("wo_z", 12),
            ("wi_x", 16),
            ("wi_y", 20),
            ("wi_z", 24),
            ("u0", 28),
            ("u1", 32),
            ("u2", 36),
        ],
        SCATTER_CASE_STRIDE as u32,
    );
    assert_layout(
        &module,
        "ScatterResult",
        &[
            ("eval_r", 0),
            ("eval_g", 4),
            ("eval_b", 8),
            ("pdf", 12),
            ("sample_x", 16),
            ("sample_y", 20),
            ("sample_z", 24),
            ("sample_pdf", 28),
            ("weight_r", 32),
            ("weight_g", 36),
            ("weight_b", 40),
            ("flags", 44),
        ],
        SCATTER_RESULT_STRIDE as u32,
    );
    let stride_of = |var: &str| {
        let (_, global) = module
            .global_variables
            .iter()
            .find(|(_, g)| g.name.as_deref() == Some(var))
            .unwrap_or_else(|| panic!("the shader declares {var}"));
        match module.types[global.ty].inner {
            TypeInner::Array {
                size: ArraySize::Dynamic,
                stride,
                ..
            } => stride,
            ref other => panic!("{var} is {other:?}"),
        }
    };
    assert_eq!(stride_of("surfaces"), SURFACE_RECORD_STRIDE as u32);
    assert_eq!(stride_of("albedo_table"), ALBEDO_ENTRY_STRIDE as u32);
    assert_eq!(stride_of("cases"), SCATTER_CASE_STRIDE as u32);
    assert_eq!(stride_of("results"), SCATTER_RESULT_STRIDE as u32);
}

#[test]
fn the_shader_constants_are_the_rust_constants() {
    let module = parsed(SCATTER_WGSL);
    assert_eq!(literal(&module, "MIN_COS"), Literal::F32(1e-9));
    assert_eq!(literal(&module, "MIN_ALPHA"), Literal::F32(1e-3));
    assert_eq!(
        literal(&module, "MEAN_SCHLICK_GRAZING"),
        Literal::F32((1.0_f64 / 21.0) as f32)
    );
    assert_eq!(
        literal(&module, "MIN_SINGLE_SCATTER_LOSS"),
        Literal::F32(1e-5)
    );
    assert_eq!(
        literal(&module, "ALMOST_ONE"),
        Literal::F32(1.0 - f32::EPSILON / 2.0)
    );
    assert_eq!(
        literal(&module, "SCATTER_PI"),
        Literal::F32(std::f32::consts::PI)
    );
    assert_eq!(
        literal(&module, "ALBEDO_TABLE_SIZE"),
        Literal::U32(solidmesh_mesh::ray_tracer::ALBEDO_TABLE_SIZE as u32)
    );
    assert_eq!(
        literal(&module, "ROW_LOWEST"),
        Literal::F32(1e-3_f64.sqrt().sqrt() as f32)
    );
    assert_eq!(
        literal(&module, "SURFACE_SMOOTH"),
        Literal::U32(SURFACE_SMOOTH)
    );
    assert_eq!(
        literal(&module, "SURFACE_INDEX_MATCHED"),
        Literal::U32(SURFACE_INDEX_MATCHED)
    );
    assert_eq!(
        literal(&module, "SURFACE_HAS_GLASS"),
        Literal::U32(SURFACE_HAS_GLASS)
    );
    let check = parsed(SCATTER_CHECK_WGSL);
    assert_eq!(literal(&check, "SAMPLE_FOUND"), Literal::U32(SAMPLE_FOUND));
    assert_eq!(
        literal(&check, "SAMPLE_SPECULAR"),
        Literal::U32(SAMPLE_SPECULAR)
    );
}

fn word(bytes: &[u8], index: usize) -> u32 {
    u32::from_le_bytes(bytes[index * 4..index * 4 + 4].try_into().expect("a word"))
}

#[test]
fn a_surface_record_holds_its_fields_and_the_branches_the_cpu_takes() {
    let surface = ScatteringSurface::new([0.25, 0.5, 0.75], 0.0, 0.5, 0.3, 1.5);
    let record = SurfaceRecord::from_surface(&surface);
    let bytes = record.to_le_bytes();
    assert_eq!(word(&bytes, 0), 0.25_f32.to_bits());
    assert_eq!(word(&bytes, 1), 0.5_f32.to_bits());
    assert_eq!(word(&bytes, 2), 0.75_f32.to_bits());
    assert_eq!(word(&bytes, 3), 0.0_f32.to_bits());
    assert_eq!(word(&bytes, 4), 0.5_f32.to_bits());
    assert_eq!(word(&bytes, 5), 0.3_f32.to_bits());
    assert_eq!(word(&bytes, 6), 1.5_f32.to_bits());
    assert_eq!(word(&bytes, 7), 0.5_f32.to_bits());
    assert_eq!(word(&bytes, 8), SURFACE_HAS_GLASS);

    // The index step near 1 is kept whole: `ior` as an f32 minus 1 has lost most of its digits.
    let near_one =
        SurfaceRecord::from_surface(&ScatteringSurface::new([1.0; 3], 0.0, 0.5, 1.0, 1.0011));
    assert_eq!(near_one.ior_minus_one, (1.0011_f64 - 1.0) as f32);
    assert_ne!(near_one.ior_minus_one, near_one.ior - 1.0);

    let smooth = ScatteringSurface::new([1.0; 3], 0.0, 0.0, 0.0, 1.5);
    assert_eq!(SurfaceRecord::from_surface(&smooth).flags, SURFACE_SMOOTH);

    let matched = ScatteringSurface::new([1.0; 3], 0.0, 0.5, 1.0, 1.0005);
    assert_eq!(
        SurfaceRecord::from_surface(&matched).flags,
        SURFACE_INDEX_MATCHED | SURFACE_HAS_GLASS
    );
}

#[test]
fn a_flag_decided_in_f64_can_differ_from_what_f32_rounds_to() {
    // Just above the smooth threshold in f64, but its nearest f32 is below it, so a shader that
    // squared the stored roughness would call this smooth and take the other branch.
    let roughness = 1e-3_f64.sqrt() * (1.0 + 1e-9);
    let surface = ScatteringSurface::new([1.0; 3], 0.0, roughness, 0.0, 1.5);
    let record = SurfaceRecord::from_surface(&surface);
    assert_eq!(
        record.flags & SURFACE_SMOOTH,
        0,
        "the CPU's own branch: rough"
    );
    let rounded = f64::from(record.roughness);
    assert!(
        rounded * rounded < 1e-3,
        "the f32 roughness rounds down across the threshold ({rounded})"
    );
}

#[test]
fn a_case_and_a_result_record_hold_their_fields_where_the_shader_reads_them() {
    let case = ScatterCheckCase {
        surface: 7,
        wo: [0.1, 0.2, 0.3],
        wi: [0.4, 0.5, 0.6],
        u: [0.7, 0.8, 0.9],
    };
    let bytes = case.to_le_bytes();
    assert_eq!(word(&bytes, 0), 7);
    assert_eq!(word(&bytes, 3), 0.3_f32.to_bits());
    assert_eq!(word(&bytes, 4), 0.4_f32.to_bits());
    assert_eq!(word(&bytes, 9), 0.9_f32.to_bits());

    let result = ScatterCheckResult {
        evaluate: [1.0, 2.0, 3.0],
        pdf: 4.0,
        sample_direction: [5.0, 6.0, 7.0],
        sample_pdf: 8.0,
        sample_weight: [9.0, 10.0, 11.0],
        flags: SAMPLE_FOUND | SAMPLE_SPECULAR,
    };
    assert_eq!(
        ScatterCheckResult::from_le_bytes(&result.to_le_bytes()),
        result
    );
}

#[test]
fn the_albedo_buffer_holds_the_grid_then_the_row_means_with_their_loss() {
    use solidmesh_mesh::ray_tracer::{stored_albedo_cell, stored_mean_albedo, ALBEDO_TABLE_SIZE};
    let bytes = albedo_table_bytes();
    assert_eq!(bytes.len(), ALBEDO_ENTRY_COUNT * ALBEDO_ENTRY_STRIDE);
    assert_eq!(bytes.len(), 12 * (32 * 32 + 32));
    let entry = |index: usize| {
        let offset = index * ALBEDO_ENTRY_STRIDE;
        let f = |at: usize| {
            f32::from_le_bytes(
                bytes[offset + at..offset + at + 4]
                    .try_into()
                    .expect("a word"),
            )
        };
        (f(0), f(4), f(8))
    };
    for (row, column) in [(0, 0), (5, 3), (31, 31)] {
        let [a, b] = stored_albedo_cell(row, column);
        assert_eq!(
            entry(row * ALBEDO_TABLE_SIZE + column),
            (a as f32, b as f32, (1.0 - a - b) as f32),
            "cell {row},{column}"
        );
    }
    let [mean_a, mean_b] = stored_mean_albedo(31);
    assert_eq!(
        entry(ALBEDO_TABLE_SIZE * ALBEDO_TABLE_SIZE + 31),
        (mean_a as f32, mean_b as f32, (1.0 - mean_a - mean_b) as f32),
    );
}

#[test]
fn the_sampled_azimuth_does_not_trust_the_built_in_sin_and_cos() {
    // WGSL only asks those built-ins for 2^-11 accuracy, which would show as a sampled direction
    // up to 1e-3 off on a device at that limit. `cos_sin_of_turn` is accurate everywhere.
    let module = parsed(SCATTER_WGSL);
    let names: Vec<&str> = module
        .functions
        .iter()
        .filter_map(|(_, f)| f.name.as_deref())
        .collect();
    assert!(names.contains(&"cos_sin_of_turn"));
    for (_, function) in module.functions.iter() {
        for (_, expression) in function.expressions.iter() {
            if let Expression::Math { fun, .. } = expression {
                assert!(
                    !matches!(fun, MathFunction::Sin | MathFunction::Cos),
                    "{:?} calls a built-in trig function",
                    function.name
                );
            }
        }
    }
    assert_eq!(
        literal(&module, "QUARTER_TURN"),
        Literal::F32(std::f32::consts::FRAC_PI_2)
    );
}
