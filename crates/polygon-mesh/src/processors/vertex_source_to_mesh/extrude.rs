//! Lifting a 2D profile into a prism, square-ended or with bevel rings at either end.
//!
//! A bevelled extrusion is a stack of rings (the profile and its insets), each placed in the
//! mesh once (`extrude/ring_mesh.rs`); the caps, the straight walls and the bands between rings
//! that differ (`extrude/band.rs`) are all built on those vertices. A bevel ring inset by less
//! than one clipper unit is the profile itself.

use std::rc::Rc;

use agg_path::clipper_library::{ClipperError, JoinType, PolygonsExtensions};
use agg_path::data_converters_2d::{
    create_vertex_storage, get_corrected_winding, VertexSourceToClipperPolygons, DEFAULT_SCALING,
};
use agg_path::vertex_source::VertexSource;
use clipper2_rust::{Path64, Paths64};
use vector_math::Vector3;

use super::triangulate_faces;
use crate::data_converters_2d::CachedTesselator;
use crate::mesh::Mesh;
use band::add_band;
use ring_mesh::{PlacedRing, RingMesh};

/// The profile from z = 0 up to `z_height_top` with square ends: [`extrude_bevelled`] with no
/// bevel at either end. A clipper failure in the profile's clean-up comes back as the error.
pub fn extrude(
    vertex_source_in: &dyn VertexSource,
    z_height_top: f64,
) -> Result<Mesh, ClipperError> {
    let bottom_polygons = vertex_source_in.create_polygons(DEFAULT_SCALING);

    // ensure good winding and consistent shapes
    let bottom_polygons = get_corrected_winding(&bottom_polygons)?;
    Ok(extrude_square_ended(&bottom_polygons, z_height_top))
}

/// Extrudes a closed profile from z = 0 up to `z_height_top`, optionally rounding either end.
///
/// `bevel` (`(height, inset_amount)` pairs) is the top bevel ordered from the bottom of the
/// bevel UPWARD. Each `height` is an absolute z and each `inset_amount` is in mm from the
/// ORIGINAL profile (negative shrinks it) - insets never compound. The wall rises un-inset to
/// `bevel[0].height`; from there entry i's inset is reached at `bevel[i + 1].height`, and the
/// last entry's at `z_height_top`, the profile the top cap is built from. `bottom_bevel` is
/// its mirror image, ordered from the top of the bevel DOWNWARD, its last inset reached at
/// z = 0. The join types say how the clipper offset rounds corners while insetting each bevel.
/// With no bevel at either end the prism is square-ended, built from the profile's fill with a
/// wall along each of its edges and the seams welded. With a bevel, every ring is placed in the
/// mesh once and every face is built on its vertices, so the result is a closed solid whatever
/// the profile.
/// A clipper failure in the profile's clean-up comes back as the error.
pub fn extrude_bevelled(
    vertex_source_in: &dyn VertexSource,
    z_height_top: f64,
    bevel: Option<&[(f64, f64)]>,
    join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Mesh, ClipperError> {
    let bottom_polygons = vertex_source_in.create_polygons(DEFAULT_SCALING);

    // ensure good winding and consistent shapes
    let bottom_polygons = get_corrected_winding(&bottom_polygons)?;

    if bevel.is_none() && bottom_bevel.is_none() {
        return Ok(extrude_square_ended(&bottom_polygons, z_height_top));
    }
    get_loop_mesh(
        z_height_top,
        bevel,
        &bottom_polygons,
        join_type,
        bottom_bevel,
        bottom_join_type,
    )
}

/// The square-ended prism: the profile's fill at the top and (reversed) at the bottom, and a
/// wall quad along every edge of the fill.
fn extrude_square_ended(bottom_polygons: &Paths64, z_height_top: f64) -> Mesh {
    let mut bottom_teselated_source = CachedTesselator::new();

    // add the top polygon
    let vertex_source_bottom = create_vertex_storage(bottom_polygons, DEFAULT_SCALING);
    let mut mesh = triangulate_faces(
        &vertex_source_bottom,
        Some(&mut bottom_teselated_source),
        Some(Mesh::new()),
        0.0,
    );
    mesh.translate(Vector3::new(0.0, 0.0, z_height_top));

    let cache = &bottom_teselated_source.indices_cache;
    let position = |i: usize| bottom_teselated_source.vertices_cache[cache[i].index as usize];
    let num_indicies = cache.len();

    // then the outside edge
    for i in (0..num_indicies.saturating_sub(2)).step_by(3) {
        let (v0, v1, v2) = (position(i), position(i + 1), position(i + 2));
        if v0 == v1 || v1 == v2 || v2 == v0 {
            continue;
        }

        let bottom = [v0, v1, v2].map(|v| Vector3::from_vector2(v, 0.0));
        let top = [v0, v1, v2].map(|v| Vector3::from_vector2(v, z_height_top));

        for corner in 0..3 {
            if cache[i + corner].is_edge {
                let next = (corner + 1) % 3;
                mesh.create_face(&[bottom[corner], bottom[next], top[next], top[corner]]);
            }
        }
    }

    // then the bottom
    for i in (0..num_indicies.saturating_sub(2)).step_by(3) {
        let (v0, v1, v2) = (position(i), position(i + 1), position(i + 2));
        if v0 == v1 || v1 == v2 || v2 == v0 {
            continue;
        }

        mesh.create_face(&[
            Vector3::from_vector2(v2, 0.0),
            Vector3::from_vector2(v1, 0.0),
            Vector3::from_vector2(v0, 0.0),
        ]);
    }

    mesh.clean_and_merge();

    mesh
}

/// One profile ring of a bevelled extrusion and the height it sits at. The un-inset profile is
/// shared, so [`Rc::ptr_eq`] recognises the straight wall.
type Ring = (Rc<Paths64>, f64);

/// Caps and walls each outline-with-holes group through its bevel rings. Each ring is placed in
/// the mesh once and every face is built on its vertices (`ring_mesh.rs`), so the solid is closed
/// by construction.
fn get_loop_mesh(
    z_height_top: f64,
    bevel: Option<&[(f64, f64)]>,
    input_polygons: &Paths64,
    join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Mesh, ClipperError> {
    let bottom_polygons_sets = input_polygons.separate_polygon_groups()?;

    let mut ring_mesh = RingMesh::new();
    for bottom_polygons in bottom_polygons_sets {
        let rings = build_bevel_rings(
            bottom_polygons,
            z_height_top,
            bevel,
            join_type,
            bottom_bevel,
            bottom_join_type,
        )?;
        // Each ring with whether the profile shrinks on the way up to it: the bands of the
        // bottom bevel are where it GROWS as it rises.
        let bottom_bevel_band_count = bottom_bevel.map_or(0, <[_]>::len);
        let mut rings: Vec<(Ring, bool)> = rings
            .into_iter()
            .enumerate()
            .map(|(i, ring)| (ring, i > bottom_bevel_band_count))
            .collect();
        // A top and bottom bevel that exactly meet leave no straight wall between them: the one
        // ring they share is placed once.
        rings.dedup_by(|(upper, _), (lower, _)| {
            Rc::ptr_eq(&upper.0, &lower.0) && upper.1 == lower.1
        });
        // A bevel that insets the whole outline away leaves nothing beyond it; the last ring
        // with loops is capped instead.
        rings.retain(|((loops, _), _)| !loops.is_empty());
        let Some(((first_loops, first_height), _)) = rings.first() else {
            continue;
        };

        let mut lower = ring_mesh.place(first_loops, *first_height);
        let all_loops = |ring: &PlacedRing| (0..ring.loops.len()).collect::<Vec<_>>();
        ring_mesh.cap(&lower, &all_loops(&lower), false);
        for ((loops, height), shrinks) in rings.iter().skip(1) {
            let upper = ring_mesh.place(loops, *height);
            if Rc::ptr_eq(&lower.loops, &upper.loops) {
                ring_mesh.straight_wall(&lower, &upper);
            } else {
                add_band(&mut ring_mesh, &lower, &upper, *shrinks);
            }
            lower = upper;
        }
        ring_mesh.cap(&lower, &all_loops(&lower), true);
    }

    Ok(ring_mesh.mesh)
}

/// The stack of profile rings the extrusion passes through, bottom to
/// top, the first at z = 0 and the last at `z_height_top`. Both bevels inset the ORIGINAL
/// profile, and every ring shares the one arc tolerance the profile's size earns.
fn build_bevel_rings(
    profile: Paths64,
    z_height_top: f64,
    top_bevel: Option<&[(f64, f64)]>,
    top_join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Vec<Ring>, ClipperError> {
    let mut rings: Vec<Ring> = Vec::new();
    let arc_tolerance = profile.arc_tolerance_for(DEFAULT_SCALING);
    let profile = Rc::new(without_straight_points(profile));
    // An inset of less than one clipper unit cannot move the outline by a representable amount:
    // it only rounds some points a unit inward and leaves the rest where they were, a ring that
    // differs from the profile by rounding alone and pinches the band to nothing all round. Such a
    // ring is the profile itself, and the wall up to it is straight.
    let inset = |inset_amount: f64, join: JoinType| {
        let delta = inset_amount * DEFAULT_SCALING;
        if delta.abs() < 1.0 {
            return Ok(Rc::clone(&profile));
        }
        profile
            .offset_with_arc_tolerance(delta, join, arc_tolerance)
            .map(|ring| Rc::new(without_straight_points(ring)))
    };

    match bottom_bevel {
        Some(bottom_bevel) if !bottom_bevel.is_empty() => {
            // the list runs downward, so its last entry is the narrowest ring - the z = 0 one
            let last = bottom_bevel.len() - 1;
            rings.push((inset(bottom_bevel[last].1, bottom_join_type)?, 0.0));
            for i in (1..=last).rev() {
                rings.push((
                    inset(bottom_bevel[i - 1].1, bottom_join_type)?,
                    bottom_bevel[i].0,
                ));
            }

            // where the bevel gives way to the straight wall the profile is back to full size
            rings.push((Rc::clone(&profile), bottom_bevel[0].0));
        }
        _ => rings.push((Rc::clone(&profile), 0.0)),
    }

    match top_bevel {
        Some(top_bevel) if !top_bevel.is_empty() => {
            rings.push((Rc::clone(&profile), top_bevel[0].0));
            for i in 0..top_bevel.len() - 1 {
                rings.push((inset(top_bevel[i].1, top_join_type)?, top_bevel[i + 1].0));
            }

            let last = top_bevel.len() - 1;
            rings.push((inset(top_bevel[last].1, top_join_type)?, z_height_top));
        }
        _ => rings.push((Rc::clone(&profile), z_height_top)),
    }

    Ok(rings)
}

/// The loops with every point that lies exactly on the line through its neighbours removed
/// (integer cross product zero: the outline runs straight on through it, turns straight back
/// in a needle of no width, or repeats it), and any loop left with fewer than three points
/// dropped. Such a point adds no area to the shape, but a cap or band could close it off only
/// with a zero-area triangle.
fn without_straight_points(loops: Paths64) -> Paths64 {
    loops
        .into_iter()
        .map(|path| {
            let mut points = path;
            // Each removal can straighten a neighbour, so repeat until nothing changes.
            loop {
                let count = points.len();
                if count < 3 {
                    break;
                }
                let kept: Path64 = (0..count)
                    .filter(|&i| {
                        let (prev, here, next) = (
                            points[(i + count - 1) % count],
                            points[i],
                            points[(i + 1) % count],
                        );
                        let (ax, ay) = ((here.x - prev.x) as i128, (here.y - prev.y) as i128);
                        let (bx, by) = ((next.x - here.x) as i128, (next.y - here.y) as i128);
                        ax * by - ay * bx != 0
                    })
                    .map(|i| points[i])
                    .collect();
                if kept.len() == count {
                    break;
                }
                points = kept;
            }
            points
        })
        .filter(|path| path.len() >= 3)
        .collect()
}

mod band;
mod ring_mesh;

#[cfg(test)]
mod tests;
