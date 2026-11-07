//! Two-mesh boolean conveniences: union, subtract, intersect, and both halves of a cut. Each call
//! is one [`super::boolean_processing::do_boolean`] with identity transforms and default options.
//!
//! Union/subtract/intersect hand back the other operand when one has no faces (so an intersect with
//! an empty mesh returns the non-empty one here, unlike `do_boolean`). They take no progress
//! reporter: each is a single quick boolean.

use vector_math::Matrix4X4;

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

/// `b` removed from `a`.
pub fn subtract(a: &Mesh, b: &Mesh, cancel: &CancelToken) -> Result<Mesh, BooleanError> {
    if a.faces.is_empty() {
        return Ok(b.clone());
    }
    if b.faces.is_empty() {
        return Ok(a.clone());
    }
    run(a, b, CsgModes::Subtract, cancel)
}

/// The volume `a` and `b` share.
pub fn intersect(a: &Mesh, b: &Mesh, cancel: &CancelToken) -> Result<Mesh, BooleanError> {
    if a.faces.is_empty() {
        return Ok(b.clone());
    }
    if b.faces.is_empty() {
        return Ok(a.clone());
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
