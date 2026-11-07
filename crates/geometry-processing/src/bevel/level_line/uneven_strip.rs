//! A cell between two arcs of DIFFERENT segment counts: zipped by arc-length share, so no
//! triangle spans more of one arc than the other. The even strip and the triangle lay are
//! `gap_fill`'s.

use super::gap_fill::{lay, strip};
use super::{LevelLineMesh, LevelLineMeshError};

/// The cell between arcs `u` and `v` - the ordinary strip where they have
/// the same count, the zip ([`zip`]) where they do not.
pub fn cell(
    mesh: &mut LevelLineMesh,
    u: &[i32],
    v: &[i32],
    added: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    if u.len() == v.len() {
        strip(mesh, u, v, added, None)
    } else {
        zip(mesh, u, v, added)
    }
}

/// Advances along whichever arc's next vertex has the smaller share of its
/// arc's length, ties broken by the shorter diagonal, laying one triangle per step.
pub fn zip(
    mesh: &mut LevelLineMesh,
    u: &[i32],
    v: &[i32],
    added: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    let shares = |mesh: &LevelLineMesh, arc: &[i32]| {
        let mut at = vec![0.0; arc.len()];
        for k in 1..arc.len() {
            at[k] = at[k - 1] + mesh.length(arc[k - 1], arc[k]);
        }

        let total = at[at.len() - 1];
        for (k, share) in at.iter_mut().enumerate() {
            *share = if total > 0.0 {
                *share / total
            } else {
                k as f64 / (arc.len() - 1) as f64
            };
        }

        at
    };

    let us = shares(mesh, u);
    let vs = shares(mesh, v);
    let mut i = 0;
    let mut j = 0;
    while i < u.len() - 1 || j < v.len() - 1 {
        let advance_u = if i == u.len() - 1 {
            false
        } else if j == v.len() - 1 {
            true
        } else if (us[i + 1] - vs[j + 1]).abs() > 1e-9 {
            us[i + 1] < vs[j + 1]
        } else {
            mesh.length(u[i + 1], v[j]) <= mesh.length(u[i], v[j + 1])
        };

        if advance_u {
            lay(mesh, u[i], v[j], u[i + 1], added)?;
            i += 1;
        } else {
            lay(mesh, u[i], v[j], v[j + 1], added)?;
            j += 1;
        }
    }

    Ok(())
}
