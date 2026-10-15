//! Lifting a 2D profile into a prism, square-ended or with bevel rings at either end.
//!
//! Where two bevel rings do not correspond point for point, the band between them is
//! triangulated by tess2-rust with its constrained Delaunay refinement, which adds no points
//! for rings that do not cross. (Shewchuk's Triangle, the usual tool for this, is licensed
//! for non-commercial use only.) Where the rings touch, at a point or along an edge, the band
//! has no width there and the wall between the rings is vertical; the skinning adds those
//! vertical faces itself. A bevel ring inset by less than one clipper unit is the profile itself.

use std::collections::HashSet;
use std::rc::Rc;

use agg_path::clipper_library::{ClipperError, JoinType, PolygonsExtensions};
use agg_path::data_converters_2d::{
    create_vertex_storage, get_corrected_winding, VertexSourceToClipperPolygons, DEFAULT_SCALING,
};
use agg_path::vertex_source::VertexSource;
use clipper2_rust::{Path64, Paths64};
use tess2_rust::{ElementType, TessOption, Tessellator, WindingRule};
use vector_math::{Vector2, Vector3};

use super::triangulate_faces;
use crate::csg::path_stitcher;
use crate::data_converters_2d::CachedTesselator;
use crate::mesh::Mesh;

/// The profile from z = 0 up to `z_height_top`
/// with square ends. A clipper failure in the profile's clean-up comes back as the error.
pub fn extrude(
    vertex_source_in: &dyn VertexSource,
    z_height_top: f64,
) -> Result<Mesh, ClipperError> {
    extrude_bevelled(
        vertex_source_in,
        z_height_top,
        None,
        JoinType::Round,
        None,
        JoinType::Round,
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

    if bevel.is_some() || bottom_bevel.is_some() {
        return get_loop_mesh(
            z_height_top,
            bevel,
            &bottom_polygons,
            join_type,
            bottom_bevel,
            bottom_join_type,
        );
    }

    let mut bottom_teselated_source = CachedTesselator::new();

    // add the top polygon
    let vertex_source_bottom = create_vertex_storage(&bottom_polygons, DEFAULT_SCALING);
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

    Ok(mesh)
}

/// One profile ring of a bevelled extrusion and the height it sits at. The un-inset profile is
/// shared, so [`Rc::ptr_eq`] recognises the straight wall.
type Ring = (Rc<Paths64>, f64);

/// Caps and walls each outline-with-holes group through its bevel rings.
fn get_loop_mesh(
    z_height_top: f64,
    bevel: Option<&[(f64, f64)]>,
    input_polygons: &Paths64,
    join_type: JoinType,
    bottom_bevel: Option<&[(f64, f64)]>,
    bottom_join_type: JoinType,
) -> Result<Mesh, ClipperError> {
    let bottom_polygons_sets = input_polygons.separate_polygon_groups()?;

    let mut mesh = Mesh::new();
    for bottom_polygons in bottom_polygons_sets {
        let rings = build_bevel_rings(
            bottom_polygons,
            z_height_top,
            bevel,
            join_type,
            bottom_bevel,
            bottom_join_type,
        )?;

        // The first wall segments are the bottom bevel, where the profile GROWS as it rises.
        // The triangulation lays its triangles out counter clockwise in 2D, which only faces
        // outward for a profile that shrinks as it rises; those segments wind back.
        let bottom_bevel_wall_count = bottom_bevel.map_or(0, <[_]>::len);

        // create the bottom polygon from whatever profile the walls actually start at
        if let Some(bottom) =
            path_stitcher::stitch(None, 0.0, Some(&rings[0].0), rings[0].1, DEFAULT_SCALING)
        {
            mesh.copy_faces(&bottom);
        }

        // create all the walls by skinning each neighbouring pair of rings
        for i in 0..rings.len() - 1 {
            let (bottom_loops, bottom_height) = &rings[i];
            let (top_loops, top_height) = &rings[i + 1];

            // A top and bottom bevel that exactly meet leave no straight wall between them.
            if Rc::ptr_eq(bottom_loops, top_loops) && bottom_height == top_height {
                continue;
            }

            let is_side = !bottom_loops.is_empty()
                && bottom_loops.len() == top_loops.len()
                && !bottom_loops[0].is_empty()
                && bottom_loops[0].len() == top_loops[0].len()
                && bottom_loops[0][0] == top_loops[0][0];

            if is_side {
                if let Some(walls) = path_stitcher::stitch(
                    Some(bottom_loops),
                    *bottom_height,
                    Some(top_loops),
                    *top_height,
                    DEFAULT_SCALING,
                ) {
                    mesh.copy_faces(&walls);
                }
            } else {
                create_triangulation(
                    &mut mesh,
                    bottom_loops,
                    *bottom_height,
                    top_loops,
                    *top_height,
                    i < bottom_bevel_wall_count,
                )?;
            }
        }

        // create the top polygon
        let last_ring = &rings[rings.len() - 1];
        if let Some(top) =
            path_stitcher::stitch(Some(&last_ring.0), last_ring.1, None, 0.0, DEFAULT_SCALING)
        {
            mesh.copy_faces(&top);
        }
    }

    mesh.clean_and_merge();
    Ok(mesh)
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
    let profile = Rc::new(profile);
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
            .map(Rc::new)
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

/// The exact position of a point as a set key (`+ 0.0` folds -0 onto 0, as `Vector2`
/// equality does).
fn position_key(x: f64, y: f64) -> (u64, u64) {
    ((x + 0.0).to_bits(), (y + 0.0).to_bits())
}

/// Skins the wall between two rings that do not correspond point for
/// point by triangulating the area between them in 2D and lifting each vertex to the ring it
/// came from. `flip_winding` reverses the faces for a profile that grows as it rises.
fn create_triangulation(
    mesh: &mut Mesh,
    bottom_loops: &Paths64,
    bottom_height: f64,
    top_loops: &Paths64,
    top_height: f64,
    flip_winding: bool,
) -> Result<(), ClipperError> {
    let rings = BandRings::new(bottom_loops, top_loops);

    let mut outline_loops = bottom_loops.clone();
    outline_loops.extend(top_loops.iter().cloned());
    let poly_groups = outline_loops.separate_into_outlines_and_contained_holes()?;

    for poly_group in poly_groups {
        if let Some((outline, holes)) = poly_group.split_first() {
            create_mesh_loop(
                mesh,
                outline,
                holes,
                (bottom_height, top_height),
                &rings,
                flip_winding,
            );
        }
    }

    // An edge both rings run along (rounding to whole clipper units can leave a short edge of a
    // small inset exactly where it was) is no boundary of the 2D band at all, so the band holds no face along it; the wall there is a
    // vertical quad from the edge on the bottom ring up to the same edge on the top ring. Rings
    // run outlines counter clockwise and holes clockwise, so the band (on the left of every face
    // edge) walks the bottom ring forward and the top ring backward; the quad does the same.
    for top_loop in top_loops {
        for (index, &a) in top_loop.iter().enumerate() {
            let b = top_loop[(index + 1) % top_loop.len()];
            if a == b || !rings.bottom_edges.contains(&(point_key(&a), point_key(&b))) {
                continue;
            }
            let at = |point: clipper2_rust::Point64, height: f64| {
                Vector3::new(point.x as f64 / 1000.0, point.y as f64 / 1000.0, height)
            };
            let (a0, b0) = (at(a, bottom_height), at(b, bottom_height));
            let (a1, b1) = (at(a, top_height), at(b, top_height));
            let mut faces = [[a0, b0, b1], [a0, b1, a1]];
            if flip_winding {
                for face in &mut faces {
                    face.swap(0, 2);
                }
            }
            for face in &faces {
                mesh.create_face(face);
            }
        }
    }
    Ok(())
}

/// A clipper point's [`position_key`].
fn point_key(point: &clipper2_rust::Point64) -> (u64, u64) {
    position_key(point.x as f64, point.y as f64)
}

/// What the band skinning needs to know about the two rings: which points belong to the bottom
/// ring, which to the top, and which pairs of points are neighbours along the top ring.
struct BandRings {
    bottom_points: HashSet<(u64, u64)>,
    top_points: HashSet<(u64, u64)>,
    /// Every edge of the top ring, in both directions.
    top_edges: HashSet<((u64, u64), (u64, u64))>,
    /// Every edge of the bottom ring, in its own direction.
    bottom_edges: HashSet<((u64, u64), (u64, u64))>,
}

impl BandRings {
    fn new(bottom_loops: &Paths64, top_loops: &Paths64) -> Self {
        let key = point_key;
        let mut top_edges = HashSet::new();
        for top_loop in top_loops {
            for (index, point) in top_loop.iter().enumerate() {
                let next = &top_loop[(index + 1) % top_loop.len()];
                top_edges.insert((key(point), key(next)));
                top_edges.insert((key(next), key(point)));
            }
        }
        let mut bottom_edges = HashSet::new();
        for bottom_loop in bottom_loops {
            for (index, point) in bottom_loop.iter().enumerate() {
                let next = &bottom_loop[(index + 1) % bottom_loop.len()];
                bottom_edges.insert((key(point), key(next)));
            }
        }
        Self {
            bottom_edges,
            bottom_points: bottom_loops.iter().flatten().map(key).collect(),
            top_points: top_loops.iter().flatten().map(key).collect(),
            top_edges,
        }
    }

    /// A point both rings pass through: the band pinches to nothing there, and the wall is
    /// vertical between the point on the bottom ring and the same point on the top ring.
    fn is_shared(&self, point: (u64, u64)) -> bool {
        self.bottom_points.contains(&point) && self.top_points.contains(&point)
    }
}

/// Triangulates one outline minus its holes (clipper units) and adds the
/// triangles, each corner at the bottom height if it is a bottom-ring point and the top height
/// otherwise, scaled back to mm.
///
/// A point both rings pass through (a small inset can leave a corner exactly where it was) is one
/// point to the 2D triangulation, lifted to the bottom height. The top ring still passes through
/// it at the top height, where the wall above starts, so every top-ring edge the band ends on at
/// such a point gets the vertical sliver that joins the two heights; without it the extrusion is
/// open along those edges.
fn create_mesh_loop(
    mesh: &mut Mesh,
    outline: &Path64,
    holes: &[Path64],
    (bottom_height, top_height): (f64, f64),
    rings: &BandRings,
    flip_winding: bool,
) {
    if outline.is_empty() {
        return;
    }

    let contour = |loop_: &Path64| -> Vec<f64> {
        loop_
            .iter()
            .flat_map(|p| [p.x as f64, p.y as f64])
            .collect()
    };

    // TriangleNet is told which contours are holes; tess2 is given the outline and its holes
    // under the odd rule, which fills the same region for holes inside their outline.
    let mut tess = Tessellator::new();
    tess.set_option(TessOption::ConstrainedDelaunayTriangulation, true);
    tess.add_contour(2, &contour(outline));
    for hole in holes {
        if !hole.is_empty() {
            tess.add_contour(2, &contour(hole));
        }
    }

    // the +z normal makes every triangle counter clockwise in 2D, as TriangleNet's are
    if !tess.tessellate(
        WindingRule::Odd,
        ElementType::Polygons,
        3,
        2,
        Some([0.0, 0.0, 1.0]),
    ) {
        return;
    }

    let vertices = tess.vertices();
    let vertex = |index: u32| {
        let i = index as usize * 2;
        Vector2::new(vertices[i], vertices[i + 1])
    };
    let at = |v: Vector2, height: f64| Vector3::new(v.x / 1000.0, v.y / 1000.0, height);

    for triangle in tess.elements().as_chunks::<3>().0 {
        if triangle.contains(&tess2_rust::tess::TESS_UNDEF) {
            continue;
        }

        let mut points = [0, 1, 2].map(|j| vertex(triangle[j]));
        if flip_winding {
            points.swap(0, 2);
        }
        let keys = points.map(|v| position_key(v.x, v.y));
        let lifted = |j: usize| {
            if rings.bottom_points.contains(&keys[j]) {
                bottom_height
            } else {
                top_height
            }
        };

        mesh.create_face(&[0, 1, 2].map(|j| at(points[j], lifted(j))));

        // The slivers, for each edge u -> v of this face that runs along the top ring from or
        // to a shared point. The face holds that edge at the lifted heights; the wall above holds
        // it at the top height, the other way round. The quad between the two, walked
        // v(lifted) -> u(lifted) -> u(top) -> v(top), closes the gap with the windings both
        // neighbours expect; a corner that is not shared is already at the top height, and
        // its half of the quad is empty.
        for j in 0..3 {
            let k = (j + 1) % 3;
            let (u, v) = (keys[j], keys[k]);
            if !(rings.is_shared(u) || rings.is_shared(v)) || !rings.top_edges.contains(&(u, v)) {
                continue;
            }
            let u_lifted = at(points[j], lifted(j));
            let v_lifted = at(points[k], lifted(k));
            let u_top = at(points[j], top_height);
            let v_top = at(points[k], top_height);
            if rings.is_shared(u) {
                mesh.create_face(&[v_lifted, u_lifted, u_top]);
            }
            if rings.is_shared(v) {
                mesh.create_face(&[v_lifted, u_top, v_top]);
            }
        }
    }
}

#[cfg(test)]
mod tests;
