//! The path loop's seam piece (`flat_tree/seam.wgsl`): its constants are the pack's
//! (`gpu_contacts.rs`), the word offsets it reads included, and the CPU's skip; the part buffer's
//! word reading matches the part record; and the contact block costs no binding: the path entry point still uses 7 storage buffers and
//! group 2 still holds bindings 0 to 6, so a caller binds exactly what it bound before. Whether the
//! seam's picture matches the CPU's needs a graphics card and is checked where one runs.

use naga::valid::{Capabilities, ValidationFlags, Validator};
use naga::{AddressSpace, Literal, Module};
use solidmesh_mesh::ray_tracer::flat_tree::{
    CONTACT_COVER_DENSER, CONTACT_EDGE_ALLOWANCE, CONTACT_EDGE_DIRECTION, CONTACT_EDGE_START,
    CONTACT_EDGE_WORDS, CONTACT_ENTRY_WORDS, CONTACT_FULLY_COVERED, CONTACT_HIT_DENSER,
    CONTACT_NO_SEAM, CONTACT_SAME_GLASS, CONTACT_SKIP_TOLERANCES, CONTACT_WORD_COLOR,
    CONTACT_WORD_DROP, CONTACT_WORD_FLAGS, CONTACT_WORD_PART, CONTACT_WORD_SURFACE,
    CONTACT_WORD_TOLERANCE, PART_STRIDE, PATH_ENTRY_POINT, PATH_WGSL,
};

use super::flat_tree_shader_tests::literal;

fn parsed() -> Module {
    naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses as WGSL")
}

#[test]
fn the_seam_constants_are_the_pack_constants() {
    let module = parsed();
    let words = |name: &str| literal(&module, name);
    assert_eq!(
        words("CONTACT_ENTRY_WORDS"),
        Literal::U32(CONTACT_ENTRY_WORDS as u32)
    );
    assert_eq!(
        words("CONTACT_FULLY_COVERED"),
        Literal::U32(CONTACT_FULLY_COVERED)
    );
    assert_eq!(
        words("CONTACT_SAME_GLASS"),
        Literal::U32(CONTACT_SAME_GLASS)
    );
    assert_eq!(
        words("CONTACT_HIT_DENSER"),
        Literal::U32(CONTACT_HIT_DENSER)
    );
    assert_eq!(
        words("CONTACT_COVER_DENSER"),
        Literal::U32(CONTACT_COVER_DENSER)
    );
    assert_eq!(words("CONTACT_NO_SEAM"), Literal::U32(CONTACT_NO_SEAM));
    assert_eq!(
        words("CONTACT_SKIP_TOLERANCES"),
        Literal::F32(CONTACT_SKIP_TOLERANCES as f32)
    );
    for (name, value) in [
        ("CONTACT_EDGE_WORDS", CONTACT_EDGE_WORDS),
        ("CONTACT_EDGE_START", CONTACT_EDGE_START),
        ("CONTACT_EDGE_DIRECTION", CONTACT_EDGE_DIRECTION),
        ("CONTACT_EDGE_ALLOWANCE", CONTACT_EDGE_ALLOWANCE),
        ("CONTACT_WORD_DROP", CONTACT_WORD_DROP),
        ("CONTACT_WORD_TOLERANCE", CONTACT_WORD_TOLERANCE),
        ("CONTACT_WORD_PART", CONTACT_WORD_PART),
        ("CONTACT_WORD_FLAGS", CONTACT_WORD_FLAGS),
        ("CONTACT_WORD_COLOR", CONTACT_WORD_COLOR),
        ("CONTACT_WORD_SURFACE", CONTACT_WORD_SURFACE),
    ] {
        assert_eq!(words(name), Literal::U32(value as u32), "{name}");
    }
    assert_eq!(
        words("PATH_PART_WORDS"),
        Literal::U32((PART_STRIDE / 4) as u32)
    );
}

#[test]
fn the_contact_block_takes_no_binding_of_its_own() {
    let module = parsed();
    let info = Validator::new(ValidationFlags::all(), Capabilities::empty())
        .validate(&module)
        .expect("the path loop validates");
    let (index, _) = module
        .entry_points
        .iter()
        .enumerate()
        .find(|(_, e)| e.name == PATH_ENTRY_POINT)
        .expect("the path entry point");
    let uses = info.get_entry_point(index);
    let mut storage = Vec::new();
    let mut group_two = Vec::new();
    for (handle, global) in module.global_variables.iter() {
        if uses[handle].is_empty() {
            continue;
        }
        if matches!(global.space, AddressSpace::Storage { .. }) {
            storage.push(global.name.clone().unwrap_or_default());
        }
        if let Some(binding) = &global.binding {
            if binding.group == 2 {
                group_two.push(binding.binding);
            }
        }
    }
    assert_eq!(
        storage.len(),
        7,
        "storage buffers the path loop uses: {storage:?}"
    );
    group_two.sort_unstable();
    assert_eq!(group_two, (0..7).collect::<Vec<u32>>());
}

#[test]
fn the_seam_is_decided_from_one_place_in_the_path_loop() {
    let module = parsed();
    for name in [
        "seam_at",
        "seam_covering",
        "seam_covers",
        "line_starts_past",
    ] {
        assert!(
            module
                .functions
                .iter()
                .any(|(_, f)| f.name.as_deref() == Some(name)),
            "the shader defines {name}"
        );
    }
    assert_eq!(PATH_WGSL.matches("seam_at(point,").count(), 1);
}
