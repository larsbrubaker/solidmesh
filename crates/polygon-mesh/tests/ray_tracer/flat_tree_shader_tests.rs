//! The flat tree's WGSL traversal: it parses and validates (a shader that does not compile fails
//! here and not only in a browser), its structs are byte for byte the Rust records, its constants
//! are the Rust constants, and the ray and hit bytes round-trip.

use naga::valid::{Capabilities, ValidationFlags, Validator};
use naga::{ArraySize, Literal, Module, ShaderStage, StructMember, TypeInner};
use solidmesh_mesh::ray_tracer::flat_tree::{
    child_code, hit_from_le_bytes, hit_to_le_bytes, FlatHit, FlatRay, FACE_BACK, FACE_FRONT,
    HIT_STRIDE, NODE_STRIDE, NO_HIT, RAY_STRIDE, STACK_CAPACITY, TRACE_WGSL, TRIANGLE_STRIDE,
};

fn parsed() -> Module {
    naga::front::wgsl::parse_str(TRACE_WGSL).expect("the shader parses as WGSL")
}

fn struct_members<'m>(module: &'m Module, name: &str) -> (&'m Vec<StructMember>, u32) {
    let (_, found) = module
        .types
        .iter()
        .find(|(_, ty)| ty.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("the shader has a struct {name}"));
    match &found.inner {
        TypeInner::Struct { members, span } => (members, *span),
        other => panic!("{name} is {other:?}"),
    }
}

pub(super) fn assert_layout(module: &Module, name: &str, fields: &[(&str, u32)], stride: u32) {
    let (members, span) = struct_members(module, name);
    let found: Vec<(&str, u32)> = members
        .iter()
        .map(|m| (m.name.as_deref().unwrap_or(""), m.offset))
        .collect();
    assert_eq!(found, fields, "the fields and offsets of {name}");
    assert_eq!(span, stride, "the stride of {name}");
}

#[test]
fn the_shader_validates_with_both_entry_points() {
    let module = parsed();
    Validator::new(ValidationFlags::all(), Capabilities::empty())
        .validate(&module)
        .expect("the shader validates");
    let names: Vec<&str> = module
        .entry_points
        .iter()
        .filter(|e| e.stage == ShaderStage::Compute)
        .map(|e| e.name.as_str())
        .collect();
    assert_eq!(names, ["trace_closest_main", "trace_any_main"]);
}

#[test]
fn the_shader_records_match_the_buffers_byte_for_byte() {
    let module = parsed();
    assert_layout(
        &module,
        "Node",
        &[
            ("min_x", 0),
            ("max_x", 16),
            ("min_y", 32),
            ("max_y", 48),
            ("min_z", 64),
            ("max_z", 80),
            ("child", 96),
        ],
        NODE_STRIDE as u32,
    );
    assert_layout(
        &module,
        "Triangle",
        &[("v0", 0), ("v1", 12), ("v2", 24), ("source_index", 36)],
        TRIANGLE_STRIDE as u32,
    );
    assert_layout(
        &module,
        "Ray",
        &[
            ("origin", 0),
            ("t_min", 12),
            ("direction", 16),
            ("t_max", 28),
            ("face_mask", 32),
        ],
        RAY_STRIDE as u32,
    );
    assert_layout(
        &module,
        "Hit",
        &[
            ("t", 0),
            ("triangle", 4),
            ("source_index", 8),
            ("front_face", 12),
            ("u", 16),
            ("v", 20),
            ("w", 24),
        ],
        HIT_STRIDE as u32,
    );
}

#[test]
fn the_shader_binds_tightly_packed_buffers() {
    let module = parsed();
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
    assert_eq!(stride_of("nodes"), NODE_STRIDE as u32);
    assert_eq!(stride_of("triangles"), TRIANGLE_STRIDE as u32);
    assert_eq!(stride_of("rays"), RAY_STRIDE as u32);
    assert_eq!(stride_of("hits"), HIT_STRIDE as u32);
}

/// The value of a named `const` of a scalar type.
pub(super) fn literal(module: &Module, name: &str) -> Literal {
    let (_, constant) = module
        .constants
        .iter()
        .find(|(_, c)| c.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("the shader declares {name}"));
    match module.global_expressions[constant.init] {
        naga::Expression::Literal(value) => value,
        ref other => panic!("{name} is {other:?}"),
    }
}

#[test]
fn the_shader_constants_are_the_rust_constants() {
    let module = parsed();
    assert_eq!(literal(&module, "EMPTY"), Literal::U32(child_code::EMPTY));
    assert_eq!(
        literal(&module, "LEAF_BIT"),
        Literal::U32(child_code::LEAF_BIT)
    );
    assert_eq!(
        literal(&module, "COUNT_BITS"),
        Literal::U32(child_code::COUNT_BITS)
    );
    assert_eq!(
        literal(&module, "COUNT_MASK"),
        Literal::U32(child_code::COUNT_MASK)
    );
    assert_eq!(literal(&module, "FACE_FRONT"), Literal::U32(FACE_FRONT));
    assert_eq!(literal(&module, "FACE_BACK"), Literal::U32(FACE_BACK));
    assert_eq!(literal(&module, "NO_HIT"), Literal::U32(NO_HIT));
    assert_eq!(
        literal(&module, "STACK_CAPACITY"),
        Literal::U32(STACK_CAPACITY as u32)
    );
    assert_eq!(literal(&module, "MIN_DIRECTION"), Literal::F32(1e-18));
    assert_eq!(literal(&module, "PARALLEL_SPAN"), Literal::F32(1e37));
    // The shader's own tolerances, wider than the reference's (the header of trace.wgsl gives the
    // budget): traverse.rs widens by 4 epsilons and has no edge snap.
    assert_eq!(
        literal(&module, "SLAB_WIDEN"),
        Literal::F32(8.0 * f32::EPSILON)
    );
    assert_eq!(
        literal(&module, "EDGE_SNAP"),
        Literal::F32(4.0 * f32::EPSILON)
    );
}

#[test]
fn the_shader_has_a_function_for_each_of_the_references_steps() {
    let module = parsed();
    let names: Vec<&str> = module
        .functions
        .iter()
        .filter_map(|(_, f)| f.name.as_deref())
        .collect();
    for wanted in [
        "safe_inverse",
        "slab",
        "shear_of",
        "hit_triangle",
        "trace_closest",
        "trace_any",
        "trace",
    ] {
        assert!(names.contains(&wanted), "the shader mirrors {wanted}");
    }
}

#[test]
fn a_ray_record_holds_its_fields_where_the_shader_reads_them() {
    let ray = FlatRay {
        origin: [1.5, -2.5, 3.5],
        direction: [0.0, 0.6, -0.8],
        t_min: 0.25,
        t_max: f32::MAX,
        face_mask: FACE_FRONT,
    };
    let bytes = ray.to_le_bytes();
    let f =
        |offset: usize| f32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("a word"));
    let u =
        |offset: usize| u32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("a word"));
    assert_eq!([f(0), f(4), f(8)], ray.origin);
    assert_eq!(f(12), ray.t_min);
    assert_eq!([f(16), f(20), f(24)], ray.direction);
    assert_eq!(f(28), ray.t_max);
    assert_eq!(u(32), FACE_FRONT);
    assert_eq!(&bytes[36..RAY_STRIDE], &[0u8; 12]);
}

#[test]
fn a_hit_record_round_trips_and_a_miss_reads_as_none() {
    let hit = FlatHit {
        t: 12.5,
        triangle: 7,
        source_index: 33,
        barycentric: [0.25, 0.25, 0.5],
        front_face: false,
    };
    assert_eq!(hit_from_le_bytes(&hit_to_le_bytes(Some(&hit))), Some(hit));
    assert_eq!(hit_from_le_bytes(&hit_to_le_bytes(None)), None);
}
