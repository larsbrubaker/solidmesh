//! Lifting a 2D profile into a prism, square-ended or with bevel rings at either end.
//!
//! Every extrusion is a stack of rings (the profile at the bottom and the top, and its insets
//! where a bevel rounds an end), each placed in the mesh once (`extrude/ring_mesh.rs`); the caps,
//! the straight walls and the bands between rings that differ (`extrude/band.rs`) are all built
//! on those vertices, so every extrusion is a closed solid by construction. A bevel ring inset by
//! less than one clipper unit is the profile itself. A profile is first resolved into the area it
//! fills (the even-odd area, as the path display fills it), then into loops a ring can be built
//! on (`processors/ring_loops.rs`): a hole touching its outline, a seam of no width between two
//! loops, a bowtie or a figure eight all extrude to a closed solid of exactly that area.

use std::rc::Rc;

use agg_path::data_converters_2d::{VertexSourceToClipperPolygons, DEFAULT_SCALING};
use agg_path::vertex_source::VertexSource;
use clipper2_rust::Paths64;
use clipper_library::{get_corrected_winding, ClipperError, JoinType, PolygonsExtensions};

use crate::mesh::Mesh;
use crate::processors::ring_loops::{simple_loops, without_crossings};
use band::add_band;
use meeting_ring::cut_meeting_ring;
use ring_mesh::{PlacedRing, RingMesh};

/// The profile from z = 0 up to `z_height_top` with square ends: [`extrude_bevelled`] with no
/// bevel at either end, a closed solid by construction. A clipper failure in the profile's
/// clean-up comes back as the error.
pub fn extrude(
    vertex_source_in: &dyn VertexSource,
    z_height_top: f64,
) -> Result<Mesh, ClipperError> {
    extrude_bevelled(
        vertex_source_in,
        z_height_top,
        None,
        JoinType::Miter,
        None,
        JoinType::Miter,
    )
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
/// With no bevel at either end the prism is square-ended: the profile placed at z = 0 and at
/// `z_height_top`, capped and joined by a straight wall. Every ring is placed in the mesh once
/// and every face is built on its vertices, so the result is a closed solid whatever the profile.
/// A clipper failure in the profile's clean-up comes back as the error.
///
/// A zero height encloses nothing and gives an empty mesh. A negative height extrudes downward:
/// the solid from `z_height_top` up to z = 0, faces pointing out, built as the extrusion of the
/// height's magnitude (bevels and all) moved down by it, so the top bevel still rounds the
/// upper end.
pub fn extrude_bevelled(
    vertex_source_in: &dyn VertexSource,
    z_height_top: f64,
    bevel: Option<&[(f64, f64)]>,
    join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Mesh, ClipperError> {
    if z_height_top == 0.0 {
        return Ok(Mesh::new());
    }
    let bottom_polygons = vertex_source_in.create_polygons(DEFAULT_SCALING);

    // ensure good winding and consistent shapes: the area the outline fills, as the path
    // display shows it, as loops a ring can be built on, so each loop is grouped with the holes
    // inside it before any ring is built
    let bottom_polygons =
        simple_loops(without_crossings(get_corrected_winding(&bottom_polygons)?)?);

    // the solid's lowest z: every ring is placed this far below the height it is built at
    let base = z_height_top.min(0.0);
    get_loop_mesh(
        base,
        z_height_top.abs(),
        bevel,
        &bottom_polygons,
        join_type,
        bottom_bevel,
        bottom_join_type,
    )
}

/// One profile ring of a bevelled extrusion and the height it sits at. The un-inset profile is
/// shared, so [`Rc::ptr_eq`] recognises the straight wall.
type Ring = (Rc<Paths64>, f64);

/// Caps and walls each outline-with-holes group through its bevel rings, from z = `base` up to
/// `base + z_height_top`. Each ring is placed in the mesh once and every face is built on its vertices (`ring_mesh.rs`), so the solid is closed
/// by construction.
fn get_loop_mesh(
    base: f64,
    z_height_top: f64,
    bevel: Option<&[(f64, f64)]>,
    input_polygons: &Paths64,
    join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Mesh, ClipperError> {
    let bottom_polygons_sets = input_polygons.separate_polygon_groups()?;

    let mut ring_mesh = RingMesh::with_base(base);
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
        // ring they share is placed once, with the area both its bands would cover flat cut out
        // of it (`extrude/meeting_ring.rs`).
        let meeting = rings.windows(2).position(|pair| {
            let ((lower, lower_height), (upper, upper_height)) = (&pair[0].0, &pair[1].0);
            Rc::ptr_eq(lower, upper) && lower_height == upper_height
        });
        rings.dedup_by(|(upper, _), (lower, _)| {
            Rc::ptr_eq(&upper.0, &lower.0) && upper.1 == lower.1
        });
        if let Some(m) = meeting.filter(|&m| m > 0 && m + 1 < rings.len()) {
            let cut = cut_meeting_ring(&rings[m - 1].0 .0, &rings[m].0 .0, &rings[m + 1].0 .0);
            rings[m].0 .0 = cut;
        }
        // A bevel that insets the whole outline away leaves nothing beyond it; the last ring
        // with loops is capped instead.
        rings.retain(|((loops, _), _)| !loops.is_empty());
        // Where both bevels inset a small outline away and meet with no straight wall between
        // them, the one ring left encloses nothing: capping it from both sides would make a sheet
        // of no thickness, so the outline adds nothing to the solid.
        if rings.len() < 2 {
            continue;
        }
        let ((first_loops, first_height), _) = &rings[0];

        let all_loops = |ring: &PlacedRing| (0..ring.loops.len()).collect::<Vec<_>>();
        if let [_, ((top_loops, top_height), _)] = rings.as_slice() {
            if Rc::ptr_eq(first_loops, top_loops) {
                // A square-ended prism is built top first: its top cap, its walls, then its
                // bottom, the order of the faces the prisms of saved designs hold. The solid is
                // the same in any order, but what is built from it afterwards (a bevel walks
                // faces in order and keeps the first of equal choices) follows that order.
                let upper = ring_mesh.place(top_loops, *top_height);
                ring_mesh.cap(&upper, &all_loops(&upper), true);
                let lower = ring_mesh.place(first_loops, *first_height);
                ring_mesh.straight_wall(&lower, &upper);
                ring_mesh.cap(&lower, &all_loops(&lower), false);
                continue;
            }
        }
        let mut lower = ring_mesh.place(first_loops, *first_height);
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
    let profile = Rc::new(simple_loops(profile));
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
            .and_then(without_crossings)
            .map(|ring| Rc::new(simple_loops(ring)))
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

mod band;
mod meeting_ring;
mod ring_mesh;

#[cfg(test)]
mod tests;
