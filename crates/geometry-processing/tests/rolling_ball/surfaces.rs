//! The surfaces the rolling-ball tests roll between, as one shared vertex list with each side's
//! triangles: a flat rectangle split into a grid, a cylinder's outer wall and a UV sphere. Each
//! builder appends to the list it is given and returns the triangles it made, so two sides of one
//! part share the part's numbering as the kernel's sides do.
#![allow(dead_code)]

use std::f64::consts::PI;

use vector_math::Vector3;

/// A flat rectangle from `corner` spanning `along_u` and `along_v`, split into `nu` by `nv` cells of
/// two triangles each, wound so its normal is `along_u x along_v`.
pub fn rectangle(
    positions: &mut Vec<Vector3>,
    corner: Vector3,
    along_u: Vector3,
    along_v: Vector3,
    nu: usize,
    nv: usize,
) -> Vec<[i32; 3]> {
    let base = positions.len() as i32;
    for j in 0..=nv {
        for i in 0..=nu {
            let u = i as f64 / nu as f64;
            let v = j as f64 / nv as f64;
            positions.push(corner + (along_u * u) + (along_v * v));
        }
    }

    let at = |i: usize, j: usize| base + ((j * (nu + 1)) + i) as i32;
    let mut triangles = Vec::new();
    for j in 0..nv {
        for i in 0..nu {
            triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }

    triangles
}

/// The outer wall of a cylinder about the z axis, radius `radius`, from z = 0 to `height`, with
/// `sides` facets whose vertex columns stand at angles `(k + offset) 2 pi / sides`, wound outward.
pub fn cylinder_wall(
    positions: &mut Vec<Vector3>,
    radius: f64,
    height: f64,
    sides: usize,
    offset: f64,
    rows: usize,
) -> Vec<[i32; 3]> {
    let base = positions.len() as i32;
    for j in 0..=rows {
        let z = height * (j as f64 / rows as f64);
        for k in 0..sides {
            let angle = ((k as f64) + offset) * (2.0 * PI / sides as f64);
            positions.push(Vector3::new(radius * angle.cos(), radius * angle.sin(), z));
        }
    }

    let at = |k: usize, j: usize| base + ((j * sides) + (k % sides)) as i32;
    let mut triangles = Vec::new();
    for j in 0..rows {
        for k in 0..sides {
            triangles.push([at(k, j), at(k + 1, j), at(k + 1, j + 1)]);
            triangles.push([at(k, j), at(k + 1, j + 1), at(k, j + 1)]);
        }
    }

    triangles
}

/// A UV sphere of `radius` about `centre`, `slices` around and `stacks` from pole to pole, wound
/// outward.
pub fn sphere(
    positions: &mut Vec<Vector3>,
    centre: Vector3,
    radius: f64,
    slices: usize,
    stacks: usize,
) -> Vec<[i32; 3]> {
    let base = positions.len() as i32;
    for j in 0..=stacks {
        let polar = PI * (j as f64 / stacks as f64);
        for k in 0..slices {
            let around = 2.0 * PI * (k as f64 / slices as f64);
            positions.push(
                centre
                    + Vector3::new(
                        radius * polar.sin() * around.cos(),
                        radius * polar.sin() * around.sin(),
                        radius * polar.cos(),
                    ),
            );
        }
    }

    let at = |k: usize, j: usize| base + ((j * slices) + (k % slices)) as i32;
    let mut triangles = Vec::new();
    for j in 0..stacks {
        for k in 0..slices {
            if j > 0 {
                triangles.push([at(k, j), at(k, j + 1), at(k + 1, j)]);
            }
            if j + 1 < stacks {
                triangles.push([at(k + 1, j), at(k, j + 1), at(k + 1, j + 1)]);
            }
        }
    }

    triangles
}
