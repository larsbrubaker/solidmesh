//! Booleans of operands with no face textures give the very mesh they gave before textures were
//! carried through the kernel: every vertex bit, every face index and every face colour. The
//! hashes were taken on solidmesh 0.2.8, so any change to the untextured path shows up here:
//! two-operand booleans, and three-operand ones through the n-ary batch and the winding-rule
//! fold with orientation repair.

use solidmesh_math::{Color, Matrix4X4, Vector3};
use solidmesh_mesh::csg::{boolean_processing, BooleanOptions, CsgModes, WindingRule};
use solidmesh_mesh::solids::platonic_solids::{create_cube, create_icosahedron};
use solidmesh_mesh::{CancelToken, Mesh};

const RED: Color = Color::rgb(0xE0, 0x30, 0x30);
const BLUE: Color = Color::rgb(0x30, 0x40, 0xE0);
const YELLOW: Color = Color::rgb(0xF0, 0xD0, 0x20);

/// Every bit of the result: positions, face indices and, when present, face colours.
fn full_hash(mesh: &Mesh) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    let mut mix = |bits: u32| {
        for byte in bits.to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
    };
    for v in &mesh.vertices {
        mix(v.x.to_bits());
        mix(v.y.to_bits());
        mix(v.z.to_bits());
    }
    for f in mesh.faces.iter() {
        mix(f.v0 as u32);
        mix(f.v1 as u32);
        mix(f.v2 as u32);
    }
    match &mesh.face_colors {
        None => mix(u32::MAX),
        Some(colors) => {
            mix(colors.len() as u32);
            for c in colors {
                mix(u32::from_le_bytes([c.red, c.green, c.blue, c.alpha]));
            }
        }
    }
    hash
}

/// How the case's operands are coloured.
#[derive(Clone, Copy)]
enum Colouring {
    /// No colour tracking at all.
    None,
    /// One colour per operand.
    PerOperand,
    /// The first operand already wears two face colours (a previous boolean's result).
    FaceColoured,
}

fn run(operation: CsgModes, colouring: Colouring) -> Mesh {
    let mut base = create_cube(10.0, 10.0, 10.0);
    let tool = create_icosahedron(7.0);
    let tool_matrix = Matrix4X4::create_rotation_z(0.5)
        * Matrix4X4::create_translation_vector(Vector3::new(3.0, 2.0, 1.5));
    let mesh_colors = match colouring {
        Colouring::None => None,
        Colouring::PerOperand => Some(vec![RED, BLUE]),
        Colouring::FaceColoured => {
            let colors = (0..base.faces.len())
                .map(|face| if face % 3 == 0 { YELLOW } else { RED })
                .collect();
            base.face_colors = Some(colors);
            Some(vec![RED, BLUE])
        }
    };
    let options = BooleanOptions {
        mesh_colors,
        ..BooleanOptions::default()
    };
    boolean_processing::do_boolean(
        &base,
        Matrix4X4::IDENTITY,
        &tool,
        tool_matrix,
        operation,
        &CancelToken::new(),
        &options,
    )
    .expect("the boolean runs")
}

#[test]
fn untextured_booleans_give_the_same_mesh_as_before() {
    let mut hashes = Vec::new();
    for operation in [CsgModes::Union, CsgModes::Subtract, CsgModes::Intersect] {
        for colouring in [
            Colouring::None,
            Colouring::PerOperand,
            Colouring::FaceColoured,
        ] {
            hashes.push(full_hash(&run(operation, colouring)));
        }
    }
    let printed: Vec<String> = hashes.iter().map(|h| format!("{h:#018x}")).collect();
    assert_eq!(hashes, PINNED, "hashes now: {printed:?}");
}

/// Union, Subtract, Intersect, each uncoloured, coloured per operand and face-coloured.
const PINNED: [u64; 9] = [
    0x3f00_be3f_7b7e_4b07,
    0x581d_b6cf_ef0b_700f,
    0x8874_4fb9_2713_843d,
    0x54f9_6c9b_1f9d_ffba,
    0x284d_fc2d_e7b0_9a56,
    0x1b6d_7609_ac6f_cdf8,
    0xbc64_8345_07ec_6f2c,
    0x258e_6ff4_7bc4_a8a4,
    0xf826_17e8_d483_f94f,
];

#[test]
fn untextured_three_operand_booleans_with_repair_give_the_same_mesh_as_before() {
    let base = create_cube(10.0, 10.0, 10.0);
    let tool = create_icosahedron(7.0);
    let second = create_cube(4.0, 4.0, 14.0);
    let items = [
        (&base, Matrix4X4::IDENTITY),
        (
            &tool,
            Matrix4X4::create_rotation_z(0.5)
                * Matrix4X4::create_translation_vector(Vector3::new(3.0, 2.0, 1.5)),
        ),
        (
            &second,
            Matrix4X4::create_rotation_x(0.2) * Matrix4X4::create_translation(-3.0, -1.0, 0.5),
        ),
    ];
    let mut hashes = Vec::new();
    // The n-ary batch, then the pairwise fold a non-default winding rule takes.
    for (operation, winding_rule) in [
        (CsgModes::Union, WindingRule::Positive),
        (CsgModes::Subtract, WindingRule::Nonzero),
    ] {
        let options = BooleanOptions {
            mesh_colors: Some(vec![RED, BLUE, YELLOW]),
            repair_orientation: true,
            winding_rule,
            ..BooleanOptions::default()
        };
        let result = boolean_processing::do_array(&items, operation, &CancelToken::new(), &options)
            .expect("the boolean runs");
        hashes.push(full_hash(&result));
    }
    let printed: Vec<String> = hashes.iter().map(|h| format!("{h:#018x}")).collect();
    assert_eq!(hashes, PINNED_THREE_OPERAND, "hashes now: {printed:?}");
}

/// The n-ary union and the folded subtract, both coloured per operand and repaired.
const PINNED_THREE_OPERAND: [u64; 2] = [0xba78_ee6c_5efd_9363, 0x50c0_322e_6453_31f6];
