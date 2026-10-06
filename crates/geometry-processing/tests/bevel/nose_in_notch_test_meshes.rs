//! Port of the test fixture `Fixtures/NoseInNotchTestMeshes.cs` (its `Create` with the default
//! arguments, and the named points and normals): a 20 mm cube with a triangular notch cut down its
//! back, and a two-faced nose standing in the notch.

#![allow(dead_code)]

use geometry_processing::bevel::BevelSweptChain;
use geometry_processing::topology::{ChainEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

/// The cube's edge.
pub const SIZE: f64 = 20.0;
/// The part's volume: the cube less the notch, plus the nose.
pub const VOLUME: f64 = 8000.0 - 2000.0 + (500.0 / 9.0);
/// Where the nose's ridge stands.
pub const RIDGE_Y: f64 = 5.0;
/// The ridge's height.
pub const RIDGE_Z: f64 = 10.0;

pub fn left_ridge_end() -> Vector3 {
    Vector3::new(5.0, RIDGE_Y, RIDGE_Z)
}

pub fn right_ridge_end() -> Vector3 {
    Vector3::new(15.0, RIDGE_Y, RIDGE_Z)
}

pub fn upper_crease_end() -> Vector3 {
    Vector3::new(10.0, 10.0, 40.0 / 3.0)
}

pub fn lower_crease_end() -> Vector3 {
    Vector3::new(10.0, 10.0, 20.0 / 3.0)
}

/// The notch's floor corner.
pub fn notch_floor() -> Vector3 {
    Vector3::new(10.0, 10.0, 0.0)
}

pub fn left_slope_normal() -> Vector3 {
    Vector3::new(1.0, -1.0, 0.0).get_normal()
}

pub fn right_slope_normal() -> Vector3 {
    Vector3::new(-1.0, -1.0, 0.0).get_normal()
}

pub fn upper_nose_normal() -> Vector3 {
    Vector3::new(0.0, -2.0, 3.0).get_normal()
}

/// C# `Create()` with its defaults (unscaled, the upper face one triangle).
pub fn create() -> Mesh {
    create_scaled(1.0)
}

/// C# `Create(scale)`, the upper face one triangle.
pub fn create_scaled(scale: f64) -> Mesh {
    let mut mesh = Mesh::new();
    let centre = Vector2::new(SIZE / 2.0, SIZE / 2.0);

    fan(&mut mesh, 0.0, false, centre);
    fan(&mut mesh, SIZE, true, centre);

    mesh.create_face(&[
        Vector3::new(0.0, SIZE, 0.0),
        Vector3::new(0.0, SIZE, SIZE),
        Vector3::new(SIZE, SIZE, SIZE),
        Vector3::new(SIZE, SIZE, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, SIZE),
        Vector3::new(0.0, SIZE, SIZE),
        Vector3::new(0.0, SIZE, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(SIZE, 0.0, 0.0),
        Vector3::new(SIZE, SIZE, 0.0),
        Vector3::new(SIZE, SIZE, SIZE),
        Vector3::new(SIZE, 0.0, SIZE),
    ]);

    slope(&mut mesh, false);
    slope(&mut mesh, true);

    mesh.create_face(&[left_ridge_end(), right_ridge_end(), upper_crease_end()]);
    mesh.create_face(&[left_ridge_end(), lower_crease_end(), right_ridge_end()]);

    mesh.clean_and_merge();

    if scale != 1.0 {
        mesh.transform(vector_math::Matrix4X4::create_scale_uniform(scale));
    }

    mesh
}

fn fan(mesh: &mut Mesh, z: f64, up: bool, centre: Vector2) {
    let corner = Vector3::new(centre.x, centre.y, z);
    let around = [
        Vector3::new(0.0, 0.0, z),
        Vector3::new(0.0, SIZE, z),
        Vector3::new(SIZE, SIZE, z),
        Vector3::new(SIZE, 0.0, z),
    ];

    for i in 0..around.len() - 1 {
        if up {
            mesh.create_face(&[corner, around[i + 1], around[i]]);
        } else {
            mesh.create_face(&[corner, around[i], around[i + 1]]);
        }
    }
}

fn slope(mesh: &mut Mesh, mirrored: bool) {
    let at = |x: f64, y: f64, z: f64| {
        if mirrored {
            Vector3::new(SIZE - x, y, z)
        } else {
            Vector3::new(x, y, z)
        }
    };

    let apex = at(5.0, RIDGE_Y, RIDGE_Z);
    let around = [
        at(0.0, 0.0, 0.0),
        at(0.0, 0.0, SIZE),
        at(10.0, 10.0, SIZE),
        at(10.0, 10.0, 40.0 / 3.0),
        at(10.0, 10.0, 20.0 / 3.0),
        at(10.0, 10.0, 0.0),
    ];

    for i in 0..around.len() - 1 {
        if i == 3 {
            continue;
        }

        if mirrored {
            mesh.create_face(&[apex, around[i], around[i + 1]]);
        } else {
            mesh.create_face(&[apex, around[i + 1], around[i]]);
        }
    }

    if mirrored {
        mesh.create_face(&[apex, around[around.len() - 1], around[0]]);
    } else {
        mesh.create_face(&[apex, around[0], around[around.len() - 1]]);
    }
}

/// C# `Creases(part, out names)` at scale 1: the nose's four creases, each a one-edge run from
/// its ridge end to its crease end, with their names.
pub fn creases(part: &Mesh) -> (Vec<BevelSweptChain>, Vec<String>) {
    creases_scaled(part, 1.0)
}

/// C# `Creases(part, out names, scale)`.
pub fn creases_scaled(part: &Mesh, scale: f64) -> (Vec<BevelSweptChain>, Vec<String>) {
    let features = FeatureEdgeSet::create(part);

    let wanted = [
        (
            "upper left",
            left_ridge_end() * scale,
            upper_crease_end() * scale,
        ),
        (
            "lower left",
            left_ridge_end() * scale,
            lower_crease_end() * scale,
        ),
        (
            "upper right",
            right_ridge_end() * scale,
            upper_crease_end() * scale,
        ),
        (
            "lower right",
            right_ridge_end() * scale,
            lower_crease_end() * scale,
        ),
    ];

    let mut runs = Vec::new();
    let mut names = Vec::new();

    for (name, from, to) in wanted {
        let mut found = false;

        for (i, edge) in features.edges().iter().enumerate() {
            if found {
                break;
            }

            if (edge.start - from).length() < 1e-6 * scale
                && (edge.end - to).length() < 1e-6 * scale
            {
                runs.push(BevelSweptChain::new(
                    vec![ChainEdge::new(i as i32, *edge, false)],
                    false,
                ));
                found = true;
            } else if (edge.end - from).length() < 1e-6 * scale
                && (edge.start - to).length() < 1e-6 * scale
            {
                runs.push(BevelSweptChain::new(
                    vec![ChainEdge::new(i as i32, *edge, true)],
                    false,
                ));
                found = true;
            }
        }

        assert!(
            found,
            "the fixture has no {name} crease from {from:?} to {to:?}"
        );
        names.push(name.to_string());
    }

    (runs, names)
}
