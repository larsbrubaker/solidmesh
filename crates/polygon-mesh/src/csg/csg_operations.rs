//! Two-mesh boolean conveniences: union, subtract, intersect, and both halves of a cut. Each call
//! is one [`super::boolean_processing::do_boolean`] with identity transforms and default options.
//!
//! An operand with no faces is answered without the kernel, as `do_boolean` answers it: a union
//! is the other operand, a subtract from nothing or an intersect with nothing is an empty mesh, and
//! a subtract of nothing is the first operand. They take no progress reporter: each is a single
//! quick boolean.

use solidmesh_math::Matrix4X4;

use super::boolean_processing::do_boolean;
use super::{BooleanError, BooleanOptions, CsgModes};
use crate::{CancelToken, Mesh};

fn run(
    a: &Mesh,
    b: &Mesh,
    operation: CsgModes,
    cancel: &CancelToken,
) -> Result<Mesh, BooleanError> {
    do_boolean(
        a,
        Matrix4X4::IDENTITY,
        b,
        Matrix4X4::IDENTITY,
        operation,
        cancel,
        &BooleanOptions::default(),
    )
}

/// `a` and `b` combined.
pub fn union(a: &Mesh, b: &Mesh, cancel: &CancelToken) -> Result<Mesh, BooleanError> {
    if a.faces.is_empty() {
        return Ok(b.clone());
    }
    if b.faces.is_empty() {
        return Ok(a.clone());
    }
    run(a, b, CsgModes::Union, cancel)
}

/// `b` removed from `a`. Nothing minus anything is nothing (an empty mesh), and `a` minus nothing
/// is `a`.
pub fn subtract(a: &Mesh, b: &Mesh, cancel: &CancelToken) -> Result<Mesh, BooleanError> {
    if a.faces.is_empty() {
        return Ok(Mesh::new());
    }
    if b.faces.is_empty() {
        return Ok(a.clone());
    }
    run(a, b, CsgModes::Subtract, cancel)
}

/// The volume `a` and `b` share: nothing (an empty mesh) when either has no faces.
pub fn intersect(a: &Mesh, b: &Mesh, cancel: &CancelToken) -> Result<Mesh, BooleanError> {
    if a.faces.is_empty() || b.faces.is_empty() {
        return Ok(Mesh::new());
    }
    run(a, b, CsgModes::Intersect, cancel)
}

/// Both halves of cutting `a` with `b`: (`a` minus `b`, `a` intersect `b`). No empty-operand
/// shortcut.
pub fn intersect_and_subtract(
    a: &Mesh,
    b: &Mesh,
    cancel: &CancelToken,
) -> Result<(Mesh, Mesh), BooleanError> {
    let subtract = run(a, b, CsgModes::Subtract, cancel)?;
    let intersect = run(a, b, CsgModes::Intersect, cancel)?;
    Ok((subtract, intersect))
}
