//! Spinning a 2D profile into a solid: C# `VertexSourceToMesh.Revolve` with its helpers
//! (`fixCloseAngles`, `RevolvedPosition`, `AddRevolveCap`, `AddRevolveStrip`). The profile goes
//! through agg-path's clipper conversions (`CreatePolygons`, `GetCorrectedWinding`,
//! `CreateVertexStorage`) so its winding is normalized and anything left of the axis is
//! mirrored onto the right before it is swept.

use agg_path::clipper_library::{int_point, PolygonsExtensions};
use agg_path::data_converters_2d::{
    create_vertex_storage, get_corrected_winding, VertexSourceToClipperPolygons, DEFAULT_SCALING,
};
use agg_path::vertex_source::VertexSource;
use clipper2_rust::{FillRule, Path64};
use vector_math::math_helper::{range_0_to_tau, TAU};
use vector_math::{MathError, Matrix4X4, Vector2, Vector3};

use super::for_each_triangle;
use crate::data_converters_2d::CachedTesselator;
use crate::mesh::Mesh;

/// C# `fixCloseAngles`: snaps an angle within 0.001 of 0 or Tau onto it.
fn fix_close_angles(angle: f64) -> f64 {
    if angle.abs() < 0.001 {
        return 0.0;
    }

    if angle < TAU + 0.001 && angle > TAU - 0.001 {
        return TAU;
    }

    angle
}

/// C# `Revolve(this IVertexSource, angleSteps = 30, angleStart = 0, angleEnd = Tau,
/// revolveAroundZ = true)`: sweeps the profile (x out from the axis, y along it) from
/// `angle_start` to `angle_end` in `angle_steps` strips, capping both ends of a partial sweep.
/// A profile that encloses no area revolves to an empty mesh.
///
/// The angles are brought into `[0, Tau]` first; one that cannot be (below `-Tau`, or NaN) is
/// the error C#'s `Range0ToTau` throws.
pub fn revolve(
    source: &dyn VertexSource,
    angle_steps: i32,
    angle_start: f64,
    angle_end: f64,
    revolve_around_z: bool,
) -> Result<Mesh, MathError> {
    let mut angle_steps = angle_steps;
    let angle_start = range_0_to_tau(angle_start)?;
    let angle_end = range_0_to_tau(angle_end)?;

    // make sure we close 360 shapes
    let angle_start = fix_close_angles(angle_start);
    let angle_end = fix_close_angles(angle_end);

    if angle_start == 0.0 && angle_end == TAU {
        angle_steps = angle_steps.max(3);
    } else {
        angle_steps = angle_steps.max(1);
    }

    // convert to clipper polygons and scale so we can ensure good shapes
    let mut polygons = source.create_polygons(DEFAULT_SCALING);

    // A profile enclosing no area - flat, or with no points at all - is what a primitive's
    // editor holds between one keystroke and the next. It cannot make a solid, so it is an
    // empty one.
    let profile_bounds = polygons.get_bounds();
    if profile_bounds.width() <= 0.0 || profile_bounds.height() <= 0.0 {
        return Ok(Mesh::new());
    }

    // Every profile needs its winding normalized, whichever side of the axis it is on: a
    // clockwise one revolves inside out.
    polygons = get_corrected_winding(&polygons);
    if polygons.is_empty() {
        // winding correction drops what encloses no area
        return Ok(Mesh::new());
    }

    // Only the profiles with material left of the axis need clipping and mirroring.
    if polygons.iter().any(|poly| poly.iter().any(|pos| pos.x < 0)) {
        let mut bounds = polygons.get_bounds();
        bounds.inflate(10.0);
        // clip against x=0 left and right
        let left_clip: Path64 = vec![
            int_point(0.0, bounds.bottom),
            int_point(0.0, bounds.top),
            int_point(bounds.left, bounds.top),
            int_point(bounds.left, bounds.bottom),
        ];
        let right_stuff = polygons.subtract_polygon(&left_clip);

        let right_clip: Path64 = vec![
            int_point(0.0, bounds.top),
            int_point(0.0, bounds.bottom),
            int_point(bounds.right, bounds.bottom),
            int_point(bounds.right, bounds.top),
        ];
        let left_stuff = polygons.subtract_polygon(&right_clip);
        // mirror left material across the origin
        let left_add = left_stuff.scale(-1.0, 1.0);
        polygons = if !left_add.is_empty() {
            if !right_stuff.is_empty() {
                right_stuff.union(&left_add, FillRule::EvenOdd)
            } else {
                left_add
            }
        } else {
            // there is nothing on the left
            right_stuff
        };
        polygons = get_corrected_winding(&polygons);
    }

    // convert the data back to PathStorage
    let cleaned_path = create_vertex_storage(&polygons, DEFAULT_SCALING);

    let mut mesh = Mesh::new();

    let has_start_and_end_faces = angle_start > 0.000001 || angle_end < TAU - 0.000001;
    // check if we need to make closing faces
    if has_start_and_end_faces {
        // make a face for the start
        add_revolve_cap(
            &cleaned_path,
            &mut mesh,
            angle_start,
            revolve_around_z,
            false,
        );
    }

    // make the outside shell
    let angle_delta = (angle_end - angle_start) / f64::from(angle_steps);
    let mut current_angle = angle_start;
    if !has_start_and_end_faces {
        angle_steps -= 1;
    }

    for _ in 0..angle_steps {
        add_revolve_strip(
            &cleaned_path,
            &mut mesh,
            current_angle,
            current_angle + angle_delta,
            revolve_around_z,
        );
        current_angle += angle_delta;
    }

    if !has_start_and_end_faces {
        if ((angle_end - angle_start) < 0.0000001 || (angle_end - TAU - angle_start) < 0.0000001)
            && (angle_end - current_angle) > 0.0000001
        {
            // make sure we close the shape exactly
            add_revolve_strip(
                &cleaned_path,
                &mut mesh,
                current_angle,
                angle_start,
                revolve_around_z,
            );
        }
    } else {
        // The end cap closes against the last strip, so it sits on the angle that strip
        // actually ended on - the accumulated angle, which is only approximately angle_end.
        add_revolve_cap(
            &cleaned_path,
            &mut mesh,
            current_angle,
            revolve_around_z,
            true,
        );
    }

    mesh.clean_and_merge();

    Ok(mesh)
}

/// C# `RevolvedPosition`: the single place a profile point becomes a point on the solid, at
/// `angle` around the axis. Every part of the solid comes through here, because
/// `clean_and_merge` welds on exact float equality and a second way of computing a shared
/// point would leave the seam open.
fn revolved_position(profile_point: Vector2, angle: f64, revolve_around_z: bool) -> Vector3 {
    if revolve_around_z {
        return Vector3::new(profile_point.x, 0.0, profile_point.y)
            .transform(Matrix4X4::create_rotation_z(angle));
    }

    Vector3::new(profile_point.x, profile_point.y, 0.0)
        .transform(Matrix4X4::create_rotation_y(angle))
}

/// C# `AddRevolveCap`: the flat face closing a partial revolve at `angle`, wound outward
/// (`reverse_winding` flips it for the end of the sweep). It tessellates directly rather than
/// through `triangulate_faces` so its corners are placed by [`revolved_position`].
fn add_revolve_cap(
    profile: &dyn VertexSource,
    mesh: &mut Mesh,
    angle: f64,
    revolve_around_z: bool,
    reverse_winding: bool,
) {
    let mut tesselator = CachedTesselator::new();
    tesselator.send_vertex_source(profile);

    for_each_triangle(&tesselator, |v0, v1, v2| {
        let (v0, v2) = if reverse_winding { (v2, v0) } else { (v0, v2) };
        mesh.create_face(&[
            revolved_position(v0, angle, revolve_around_z),
            revolved_position(v1, angle, revolve_around_z),
            revolved_position(v2, angle, revolve_around_z),
        ]);
    });
}

/// C# `AddRevolveStrip`: one quad per profile segment between the two angles.
fn add_revolve_strip(
    vertex_source: &dyn VertexSource,
    mesh: &mut Mesh,
    start_angle: f64,
    end_angle: f64,
    revolve_around_z: bool,
) {
    let mut last_point = Vector2::ZERO;
    let mut first_point = Vector2::ZERO;

    for vertex_data in vertex_source.vertices() {
        if vertex_data.is_stop() {
            break;
        }

        if vertex_data.is_move_to() {
            first_point = vertex_data.position;
            last_point = first_point;
        }

        if vertex_data.is_line_to() || vertex_data.is_close() {
            let current_point = if vertex_data.is_close() {
                first_point
            } else {
                vertex_data.position
            };

            // a segment lying on the axis sweeps no area
            if current_point.x != 0.0 || last_point.x != 0.0 {
                mesh.create_face(&[
                    revolved_position(current_point, end_angle, revolve_around_z),
                    revolved_position(current_point, start_angle, revolve_around_z),
                    revolved_position(last_point, start_angle, revolve_around_z),
                    revolved_position(last_point, end_angle, revolve_around_z),
                ]);
            }

            last_point = current_point;
        }
    }
}
