//! Outlines to meshes: flat fills, extrusions and revolves of 2D outlines.
//!
//! Extrude and revolve build on integer polygons, clipper2-rust's `Paths64` at
//! [`DEFAULT_SCALING`] (clipper units per mm: a one-micron grid), whose clean-up and offsets run
//! on clipper-library. [`extrude`], [`extrude_bevelled`] and [`revolve`] take those polygons;
//! [`extrude_contours`], [`extrude_bevelled_contours`] and [`revolve_contours`] are the same
//! operations on contours in mm (each a list of points, closed back to its first), snapped onto
//! the grid by [`contours_to_polygons`] - each point scaled by [`DEFAULT_SCALING`] and rounded
//! half to even - which is the conversion a polyline path gets on its way to a solid. A caller with
//! curves flattens them first; how finely is its own choice (the app's paths flatten by their size
//! and convert in mattercad-geometry).
//!
//! The flat fill every other caller builds on ([`triangulate_faces`], over [`CachedTesselator`])
//! takes contours in their own units, unsnapped, and is in this file; revolving is in [`revolve`]
//! and extruding with its bevel rings in [`extrude`], split by entry point for the file-size
//! limit.

use clipper2_rust::{Path64, Paths64};
use clipper_library::{int_point, ClipperError, JoinType, DEFAULT_SCALING};
use vector_math::{Matrix4X4, Vector2, Vector3};

mod extrude;
mod revolve;

pub use extrude::{extrude, extrude_bevelled};
pub use revolve::{revolve, RevolveError};

use crate::data_converters_2d::CachedTesselator;
use crate::mesh::Mesh;

/// Contours in mm as integer polygons at [`DEFAULT_SCALING`]: one polygon per non-empty
/// contour, every point scaled onto the grid and rounded half to even.
pub fn contours_to_polygons(contours: &[Vec<Vector2>]) -> Paths64 {
    contours
        .iter()
        .filter(|contour| !contour.is_empty())
        .map(|contour| {
            contour
                .iter()
                .map(|point| int_point(point.x * DEFAULT_SCALING, point.y * DEFAULT_SCALING))
                .collect::<Path64>()
        })
        .collect()
}

/// [`extrude`] of contours in mm ([`contours_to_polygons`]).
pub fn extrude_contours(
    contours: &[Vec<Vector2>],
    z_height_top: f64,
) -> Result<Mesh, ClipperError> {
    extrude(&contours_to_polygons(contours), z_height_top)
}

/// [`extrude_bevelled`] of contours in mm ([`contours_to_polygons`]).
pub fn extrude_bevelled_contours(
    contours: &[Vec<Vector2>],
    z_height_top: f64,
    bevel: Option<&[(f64, f64)]>,
    join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Mesh, ClipperError> {
    extrude_bevelled(
        &contours_to_polygons(contours),
        z_height_top,
        bevel,
        join_type,
        bottom_bevel,
        bottom_join_type,
    )
}

/// [`revolve`] of contours in mm ([`contours_to_polygons`]).
pub fn revolve_contours(
    contours: &[Vec<Vector2>],
    angle_steps: i32,
    angle_start: f64,
    angle_end: f64,
    revolve_around_z: bool,
) -> Result<Mesh, RevolveError> {
    revolve(
        &contours_to_polygons(contours),
        angle_steps,
        angle_start,
        angle_end,
        revolve_around_z,
    )
}

/// Fills the contours' outline (non-zero winding) at `z_height`, added to `mesh_to_add_to` or a
/// new mesh (see [`triangulate_transformed`]).
pub fn triangulate_faces(
    contours: &[Vec<Vector2>],
    tesselated_source: Option<&mut CachedTesselator>,
    mesh_to_add_to: Option<Mesh>,
    z_height: f64,
) -> Mesh {
    triangulate_transformed(contours, tesselated_source, mesh_to_add_to, z_height, None)
}

/// Fills the contours' outline at `z_height`, skipping triangles with two equal corners,
/// and transforms each corner by `in_matrix` unless it is absent or the identity. A caller
/// that passes `tesselated_source` gets the tessellation back in it.
pub fn triangulate_transformed(
    contours: &[Vec<Vector2>],
    tesselated_source: Option<&mut CachedTesselator>,
    mesh_to_add_to: Option<Mesh>,
    z_height: f64,
    in_matrix: Option<Matrix4X4>,
) -> Mesh {
    let matrix = in_matrix.filter(|m| *m != Matrix4X4::IDENTITY);
    let mut own_tesselator = CachedTesselator::new();
    let tesselator = tesselated_source.unwrap_or(&mut own_tesselator);
    tesselator.send_contours(contours);

    let mut mesh = mesh_to_add_to.unwrap_or_default();
    for_each_triangle(tesselator, |v0, v1, v2| {
        let corners = [v0, v1, v2].map(|v| {
            let position = Vector3::from_vector2(v, z_height);
            match matrix {
                Some(m) => position.transform(m),
                None => position,
            }
        });
        mesh.create_face(&corners);
    });
    mesh
}

/// Calls `f` with the three corners of every tessellated triangle that has three distinct
/// corners (the `v0 == v1 || v1 == v2 || v2 == v0` skip every reader of the cache needs).
fn for_each_triangle(tesselator: &CachedTesselator, mut f: impl FnMut(Vector2, Vector2, Vector2)) {
    let cache = &tesselator.indices_cache;
    for i in (0..cache.len().saturating_sub(2)).step_by(3) {
        let position = |k: usize| tesselator.vertices_cache[cache[i + k].index as usize];
        let (v0, v1, v2) = (position(0), position(1), position(2));
        if v0 == v1 || v1 == v2 || v2 == v0 {
            continue;
        }
        f(v0, v1, v2);
    }
}
