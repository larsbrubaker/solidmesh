//! The Booleans page: union, difference or intersection of two primitives (`shapes`) through
//! `csg::boolean_processing::do_boolean`, the library's two-operand boolean on manifold-rust.
//!
//! "Messy" inputs show the robust path: operand B turned inside out (every face reversed), and
//! operand A made self-intersecting (a second copy of it overlapping the first in the same mesh,
//! never unioned). `BooleanOptions` then decides what counts as solid: the nonzero winding rule
//! keeps an inside-out shell as material, and `repair_orientation` rewinds it before the combine.
//! The note reports what `classify_boolean_operand` makes of each operand, so the page can show
//! that a self-intersecting or inside-out operand is taken rather than refused.

use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, do_boolean};
use polygon_mesh::csg::{BooleanOperandVerdict, BooleanOptions, CsgModes, WindingRule};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Matrix4X4;

use crate::shapes::Shape;
use crate::Scene;

/// The operand size in mm.
const SIZE: f64 = 20.0;

/// Segments around each curved operand.
const SEGMENTS: u32 = 48;

/// What the page's controls send.
pub struct BooleanParams {
    pub shape_a: u32,
    pub shape_b: u32,
    /// 0 union, 1 difference (A - B), 2 intersection.
    pub op: u32,
    /// Where B sits, in mm from A's centre.
    pub offset: [f64; 3],
    /// B's turn about z, in degrees.
    pub rotate_deg: f64,
    /// 0 clean, 1 B inside out, 2 A self-intersecting, 3 both.
    pub messy: u32,
    pub nonzero: bool,
    pub repair: bool,
}

fn verdict_text(verdict: BooleanOperandVerdict) -> &'static str {
    match verdict {
        BooleanOperandVerdict::Clean => "clean",
        BooleanOperandVerdict::SelfIntersecting => "self-intersecting (robust engine)",
        BooleanOperandVerdict::Refused => "refused",
    }
}

/// A plus a second copy of itself turned and shifted, in one mesh: two closed shells that cut
/// through each other, which the kernel only takes on its robust path.
fn self_intersecting(mesh: &Mesh) -> Mesh {
    let mut doubled = mesh.copy();
    let shift = Matrix4X4::create_rotation_z(0.6)
        * Matrix4X4::create_translation(SIZE * 0.3, SIZE * 0.2, SIZE * 0.15);
    doubled.copy_all_faces(mesh, shift);
    doubled
}

/// The boolean scene: the result, with both operands (as placed and messed up) as the ghost.
pub fn boolean_scene(params: &BooleanParams) -> Result<Scene, String> {
    let mut a = Shape::from_index(params.shape_a)?.build(SIZE, SEGMENTS)?;
    let mut b = Shape::from_index(params.shape_b)?.build(SIZE, SEGMENTS)?;
    if params.messy & 1 != 0 {
        b.reverse_faces();
    }
    if params.messy & 2 != 0 {
        a = self_intersecting(&a);
    }
    let [x, y, z] = params.offset;
    let matrix_b = Matrix4X4::create_rotation_z(params.rotate_deg.to_radians())
        * Matrix4X4::create_translation(x, y, z);
    let operation = match params.op {
        0 => CsgModes::Union,
        1 => CsgModes::Subtract,
        2 => CsgModes::Intersect,
        other => return Err(format!("There is no boolean {other}.")),
    };
    let options = BooleanOptions {
        winding_rule: if params.nonzero {
            WindingRule::Nonzero
        } else {
            WindingRule::Positive
        },
        repair_orientation: params.repair,
        ..BooleanOptions::default()
    };
    let verdict_a = classify_boolean_operand(&a, Matrix4X4::IDENTITY, params.repair);
    let verdict_b = classify_boolean_operand(&b, matrix_b, params.repair);
    let mesh = do_boolean(
        &a,
        Matrix4X4::IDENTITY,
        &b,
        matrix_b,
        operation,
        &CancelToken::new(),
        &options,
    )
    .map_err(|e| format!("The boolean refused: {e}"))?;
    let mut ghost = a.copy();
    ghost.copy_all_faces(&b, matrix_b);
    let note = format!(
        "Operand A: {}. Operand B: {}.",
        verdict_text(verdict_a),
        verdict_text(verdict_b)
    );
    Ok(Scene {
        mesh,
        ghost: Some(ghost),
        note,
    })
}
