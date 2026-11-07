//! Plan 6's spherical corner, both ways up: one corner tool and the groups meeting at it
//! ([`BevelCornerPatch`]), every patch a selection asked for with where each ball sits and what an
//! unpatched corner has to say ([`BevelCornerPatchSet`]), and what has been selected at one mesh
//! vertex ([`Corner`]), with the corner functions: the corner's own constants, building one
//! corner's tool ([`try_build`]), placing its ball ([`try_get_inset_point`]), collecting a
//! selection's corners ([`collect`]) and the whole selection's tools ([`create`]).
//!
//! Where three or more fillet chains of one radius meet at a vertex, the swept tools leave the
//! intersection of their cylinders where a rolling ball would leave a sphere: on an OUTSIDE corner
//! a tricylinder octant of material standing where a ball octant belongs, on an INSIDE one the
//! same shape of void left unfilled. A patch is the solid that settles the difference, to
//! subtract in the first case and to union on in the second.
//!
//! The set's tables are [`IndexMap`]s and [`IndexSet`]s because [`collect`] fills them in a fixed
//! order, which reaches the output.

use std::collections::HashSet;

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3};

use super::{
    bevel_corner_face_ring, bevel_corner_patch_mesher, bevel_edge_tool_builder, BevelAdvisory,
    BevelAdvisoryCode,
};
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge};

/// One corner tool and the groups whose selections meet at it. A patch that built is always used,
/// even where one of those groups then failed to sweep its chain: every chain ending here was laid
/// on this patch's own plane, so withholding the patch would leave them stopped short around a
/// hole.
#[derive(Debug, Clone, Default)]
pub struct BevelCornerPatch {
    /// The corner solid, in the source item's own local space: one to SUBTRACT at an outside
    /// corner and one to UNION on at an inside one - see [`BevelCornerPatch::is_fill`].
    pub mesh: Mesh,
    /// The void this corner owes a FILL, as a solid to clip that fill against, or `None` where
    /// none could be built. Always `None` on a scoop, which takes material away.
    pub void_cone: Option<Mesh>,
    /// The groups whose selections meet at this corner. Also what decides whether the patch
    /// SURVIVES the second pass: `BevelBeadBatch` carries it as a fill's owners and takes the
    /// fill back whenever any one of these groups was re-beaded on the cut part.
    pub groups: Vec<i32>,
    /// Whether this solid FILLS an inside corner rather than scooping an outside one. A fill
    /// joins the BEADS rather than the cutters.
    pub is_fill: bool,
}

/// Every corner patch a selection asked for, where each patched corner's ball sits (which is where
/// the chains ending there stop), and what any corner that could not be patched has to say.
/// `Default` is the empty set ([`BevelCornerPatchSet::empty`]).
#[derive(Debug, Clone, Default)]
pub struct BevelCornerPatchSet {
    /// The corner solids, one per patched vertex - to subtract or to union on as each says.
    pub patches: Vec<BevelCornerPatch>,
    /// By mesh vertex, where the rolling ball sits in each patched OUTSIDE corner. A convex chain
    /// ending at one of these stops on the plane through that point square to its own last edge.
    pub ball_centres: IndexMap<i32, Vector3>,
    /// The same for each filled INSIDE corner, where the ball sits out in the void. Kept apart so
    /// a chain reads only the corners its own convexity built.
    pub fill_ball_centres: IndexMap<i32, Vector3>,
    /// The warnings per group, in the resolutions' own order, or `None` for a group with no
    /// corner to complain about.
    pub warnings: Vec<Option<Vec<BevelAdvisory>>>,
    /// The patched vertices a run may overshoot straight THROUGH rather than stop on: the
    /// all-convex corners of exactly three runs (see `BevelChainEnds`).
    pub runs_may_drive_through: IndexSet<i32>,
    /// The points each patch put on each run's ball-centre plane, by (vertex, mesh edge) - the
    /// SEAM, which the run carries literally rather than computing its own arc there
    /// (`BevelSeamRing`). By the edge as well as the vertex because one corner hands a different
    /// arc to each run meeting at it. Both convexities share the map: a vertex cannot be both a
    /// patched outside corner and a filled inside one.
    pub seam_rings: IndexMap<(i32, i32), Vec<Vector3>>,
}

/// What has been selected at one mesh vertex.
#[derive(Debug, Clone, Default)]
pub struct Corner {
    /// The mesh vertex this corner is, which is how a chain end recognizes it.
    pub vertex: i32,
    /// Where that vertex is.
    pub position: Vector3,
    /// The selected feature edges at the vertex, kept sorted by mesh edge index so the face ring
    /// and its triangulation do not depend on selection order.
    pub edges: Vec<FeatureEdge>,
    /// The groups those edges were picked into, each once.
    pub groups: Vec<i32>,
    /// The mesh edges already taken in, so one edge picked into two groups is one edge. Only
    /// asked for membership, never iterated.
    pub mesh_edges: HashSet<i32>,
}

/// How many selected fillet chains have to end at a vertex before their cylinders leave anything to
/// patch. Two meeting chains are resolved by the union alone.
pub const MINIMUM_EDGES_FOR_A_PATCH: i32 = 3;

/// A one segment fillet is a chamfer chord, and a chamfer corner is the planar cutback the union
/// already gives; below this there is no arc for the patch to be tangent to.
pub const MINIMUM_SEGMENTS_FOR_A_PATCH: i32 = 2;

/// How far a face may be from the ball, as a fraction of the radius, and still count as touching
/// it - what an apex of four or more faces is tested against. Loose enough for the single precision
/// the mesh stores its positions in, and a tenth of a micron on a millimetre fillet is still far
/// tighter than any corner a user could mean.
const TOUCH_AGREEMENT: f64 = 1e-4;

/// How far from the vertex the ball's centre may sit, in radii, before the corner is too sharp to
/// patch. That distance is `r / sin(half the narrowest wedge)`, so twenty is a wedge of about six
/// degrees, and the outer radius is measured from it.
const MAXIMUM_INSET_RADII: f64 = 20.0;

/// One corner's tool as [`try_build`] hands it back: the shell, the void cone a fill is clipped to
/// (`None` on a scoop, or where none could be built), the ball's centre, and each run's seam
/// published against its own mesh edge.
pub type BuiltCornerTool = (Mesh, Option<Mesh>, Vector3, Vec<(i32, Vec<Vector3>)>);

impl BevelCornerPatch {
    /// A corner tool, its void cone, the groups it belongs to and whether it fills.
    pub fn new(mesh: Mesh, void_cone: Option<Mesh>, groups: Vec<i32>, is_fill: bool) -> Self {
        BevelCornerPatch {
            mesh,
            void_cone,
            groups,
            is_fill,
        }
    }
}

impl BevelCornerPatchSet {
    /// A set from its parts.
    pub fn new(
        patches: Vec<BevelCornerPatch>,
        ball_centres: IndexMap<i32, Vector3>,
        fill_ball_centres: IndexMap<i32, Vector3>,
        warnings: Vec<Option<Vec<BevelAdvisory>>>,
        runs_may_drive_through: IndexSet<i32>,
        seam_rings: IndexMap<(i32, i32), Vec<Vector3>>,
    ) -> Self {
        BevelCornerPatchSet {
            patches,
            ball_centres,
            fill_ball_centres,
            warnings,
            runs_may_drive_through,
            seam_rings,
        }
    }

    /// No patches, no centres, no warnings. A fresh value each call rather than a shared static,
    /// which is the same set to every reader.
    pub fn empty() -> Self {
        BevelCornerPatchSet::new(
            Vec::new(),
            IndexMap::new(),
            IndexMap::new(),
            Vec::new(),
            IndexSet::new(),
            IndexMap::new(),
        )
    }
}

impl Corner {
    /// A corner at `vertex`, with nothing selected there yet.
    pub fn new(vertex: i32, position: Vector3) -> Self {
        Corner {
            vertex,
            position,
            edges: Vec::new(),
            groups: Vec::new(),
            mesh_edges: HashSet::new(),
        }
    }

    /// The groups whose edges meet here, in first-seen order.
    pub fn groups(&self) -> &[i32] {
        &self.groups
    }

    /// Adds `edge` (once per mesh edge) and its group. The edges stay sorted by mesh edge so the
    /// face ring and its triangulation never depend on selection order.
    pub fn add(&mut self, edge: FeatureEdge, group_index: i32) {
        if !self.mesh_edges.insert(edge.mesh_edge_index) {
            return;
        }

        self.edges.push(edge);
        self.edges.sort_by_key(|e| e.mesh_edge_index);

        if !self.groups.contains(&group_index) {
            self.groups.push(group_index);
        }
    }

    /// The selected feature edges at the vertex.
    pub fn edges(&self) -> &[FeatureEdge] {
        &self.edges
    }
}

/// Every vertex where eligible groups' convex edges meet (the OUTSIDE corners), every vertex where
/// concave ones meet and nothing convex does (the INSIDE corners), and every vertex any concave
/// edge ends at - `(outside, inside, concave vertices)`, in the order the vertices were first met.
pub fn collect(
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
) -> (Vec<Corner>, Vec<Corner>, HashSet<i32>) {
    let mut convex_at_vertex: IndexMap<i32, Corner> = IndexMap::new();
    let mut concave_at_vertex: IndexMap<i32, Corner> = IndexMap::new();

    for (group_index, resolution) in resolutions.iter().enumerate() {
        if !eligible[group_index] {
            continue;
        }

        for edge in &resolution.edges {
            let at_vertex = match edge.convexity {
                EdgeConvexity::Convex => &mut convex_at_vertex,
                EdgeConvexity::Concave => &mut concave_at_vertex,
                // Neither, which is an edge flat enough that there is no corner here to patch.
                _ => continue,
            };

            record(
                at_vertex,
                edge.start_vertex,
                edge.start,
                edge,
                group_index as i32,
            );
            record(
                at_vertex,
                edge.end_vertex,
                edge.end,
                edge,
                group_index as i32,
            );
        }
    }

    let concave_vertices: HashSet<i32> = concave_at_vertex.keys().copied().collect();
    let inside_corners = concave_at_vertex
        .into_values()
        .filter(|corner| !convex_at_vertex.contains_key(&corner.vertex))
        .collect();
    let outside_corners = convex_at_vertex.into_values().collect();

    (outside_corners, inside_corners, concave_vertices)
}

/// Adds `edge` to the corner at `vertex`, making the corner on first ask.
pub fn record(
    at_vertex: &mut IndexMap<i32, Corner>,
    vertex: i32,
    position: Vector3,
    edge: &FeatureEdge,
    group_index: i32,
) {
    at_vertex
        .entry(vertex)
        .or_insert_with(|| Corner::new(vertex, position))
        .add(*edge, group_index);
}

/// One corner's tool, or the reason there is none. `Err(None)` is a corner this was never asked to
/// patch - a partial selection, whose ends are capped by the surface as before.
///
/// `fill` says this is an INSIDE corner, whose ball sits in the void: the mirror derived on the
/// class, which is the same construction with the face normals turned round.
pub fn try_build(
    corner: &Corner,
    fill: bool,
    radius: f64,
    segments: i32,
) -> Result<BuiltCornerTool, Option<BevelAdvisory>> {
    let Some(ring) = bevel_corner_face_ring::try_order(corner.edges()) else {
        return Err(None);
    };
    let mut normals = ring.normals;
    let edge_of_sector = ring.edge_of_sector;

    // The ring as the SURFACE presents it, kept back before the mirror below negates it in place. The
    // void cone is the corner's own OUTWARD half spaces and its rays have to be signed against them
    // (bevel_corner_patch_mesher::try_build_void_cone), so the fill leg cannot hand over the mirrored
    // copy it draws its own shell from - the two describe opposite cones.
    let outward = if fill { Some(normals.clone()) } else { None };

    if fill {
        // The whole of the mirror. A ball that stands a radius OUTSIDE every face is a ball that
        // stands a radius inside every face of the corner these normals describe turned round, and
        // the ring the walk above just closed is the same ring either way.
        for normal in normals.iter_mut() {
            *normal = -*normal;
        }
    }

    let edge_count = corner.edges().len() as i32;

    let Some((center, from_center)) = try_get_inset_point(&normals, corner.position, radius) else {
        return Err(Some(BevelAdvisory::new(
            if fill {
                BevelAdvisoryCode::CornerBallDoesNotTouchAllInsideEdges
            } else {
                BevelAdvisoryCode::CornerBallDoesNotTouchAllEdges
            },
            vec![edge_count.into(), corner.position.into()],
        )));
    };

    let outer_radius = from_center.length() + radius;

    let shell =
        bevel_corner_patch_mesher::build_shell(center, &normals, radius, outer_radius, segments)
            .filter(|(mesh, _)| {
                classify_boolean_operand(mesh, Matrix4X4::IDENTITY, false)
                    == BooleanOperandVerdict::Clean
            });

    let Some((mesh, seam_rings)) = shell else {
        return Err(Some(BevelAdvisory::new(
            if fill {
                BevelAdvisoryCode::CornerPatchNotSolidToAdd
            } else {
                BevelAdvisoryCode::CornerPatchNotSolidToCut
            },
            vec![edge_count.into(), corner.position.into()],
        )));
    };

    // The wall over each sector is the plane one run stops on, so the arc the shell just drew there
    // is that run's seam - published against its own mesh edge for the sweep to carry
    // (BevelSeamRing).
    let seams: Vec<(i32, Vec<Vector3>)> = seam_rings
        .into_iter()
        .enumerate()
        .map(|(sector, seam)| (corner.edges()[edge_of_sector[sector]].mesh_edge_index, seam))
        .collect();

    // Only a FILL has anything to be clipped to. A scoop takes material away, so reaching too far
    // costs it nothing it was not already entitled to take, and there is no void to name.
    let void_cone = outward.and_then(|outward| {
        bevel_corner_patch_mesher::try_build_void_cone(
            corner.position,
            &outward,
            bevel_edge_tool_builder::CONTAINMENT_MARGIN * outer_radius,
            segments,
            true,
        )
    });

    Ok((mesh, void_cone, center, seams))
}

/// Where the rolling ball sits when it touches every face of the corner at once - the point
/// `radius` from all of them - as the centre and the vector from the centre back out to the vertex.
/// Solved from the triple of faces that is furthest from degenerate and then checked against the
/// rest, so an apex of four or more faces is patched only when one ball really does touch all of
/// them.
pub fn try_get_inset_point(
    normals: &[Vector3],
    vertex: Vector3,
    radius: f64,
) -> Option<(Vector3, Vector3)> {
    let mut from_center = Vector3::ZERO;
    let mut best_determinant: f64 = 0.0;

    for a in 0..normals.len() {
        for b in a + 1..normals.len() {
            for c in b + 1..normals.len() {
                let determinant = normals[a].dot(normals[b].cross(normals[c]));

                if determinant.abs() <= best_determinant.abs() {
                    continue;
                }

                best_determinant = determinant;

                from_center = (normals[b].cross(normals[c])
                    + normals[c].cross(normals[a])
                    + normals[a].cross(normals[b]))
                    * (radius / determinant);
            }
        }
    }

    if best_determinant.abs() < 1e-6 || from_center.length() > radius * MAXIMUM_INSET_RADII {
        return None;
    }

    for normal in normals {
        if (normal.dot(from_center) - radius).abs() > radius * TOUCH_AGREEMENT {
            return None;
        }
    }

    Some((vertex - from_center, from_center))
}

/// The corner tools for a whole selection, read across every group at once - a vertex does not care
/// which group each edge was picked into. `resolutions` is every group's resolved selection and
/// `eligible` which of them resolved and fit; a chamfer (`round_profile` false), or a fillet of
/// fewer than [`MINIMUM_SEGMENTS_FOR_A_PATCH`] segments, needs no patch and gets none. The corners
/// a run may drive straight through are the three-edge outside ones with no concave edge at the
/// vertex and a ball that built (see `BevelChainEnds`).
pub fn create(
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
    round_profile: bool,
    radius: f64,
    segments: i32,
) -> BevelCornerPatchSet {
    let mut warnings: Vec<Option<Vec<BevelAdvisory>>> = vec![None; resolutions.len()];
    let mut patches = Vec::new();
    let mut ball_centres = IndexMap::new();
    let mut fill_ball_centres = IndexMap::new();
    let mut seam_rings = IndexMap::new();

    if !round_profile || segments < MINIMUM_SEGMENTS_FOR_A_PATCH {
        // A one segment fillet is a chamfer chord, and a chamfer corner is the union's job.
        return BevelCornerPatchSet::new(
            patches,
            ball_centres,
            fill_ball_centres,
            warnings,
            IndexSet::new(),
            seam_rings,
        );
    }

    let mut patch = |corner: &Corner,
                     fill: bool,
                     centres: &mut IndexMap<i32, Vector3>,
                     patches: &mut Vec<BevelCornerPatch>,
                     warnings: &mut Vec<Option<Vec<BevelAdvisory>>>| {
        if (corner.edges.len() as i32) < MINIMUM_EDGES_FOR_A_PATCH {
            return;
        }

        match try_build(corner, fill, radius, segments) {
            Ok((mesh, void_cone, ball_centre, seams)) => {
                patches.push(BevelCornerPatch::new(
                    mesh,
                    void_cone,
                    corner.groups.clone(),
                    fill,
                ));
                centres.insert(corner.vertex, ball_centre);

                for (mesh_edge, points) in seams {
                    seam_rings.insert((corner.vertex, mesh_edge), points);
                }
            }
            Err(Some(warning)) => {
                for &group in &corner.groups {
                    warnings[group as usize]
                        .get_or_insert_with(Vec::new)
                        .push(warning.clone());
                }
            }
            Err(None) => {}
        }
    };

    let (outside_corners, inside_corners, concave_vertices) = collect(resolutions, eligible);

    for corner in &outside_corners {
        patch(
            corner,
            false,
            &mut ball_centres,
            &mut patches,
            &mut warnings,
        );
    }

    for corner in &inside_corners {
        patch(
            corner,
            true,
            &mut fill_ball_centres,
            &mut patches,
            &mut warnings,
        );
    }

    // Which of those scoops the runs may drive straight through rather than stop on.
    let through_corners: IndexSet<i32> = outside_corners
        .iter()
        .filter(|corner| {
            corner.edges.len() as i32 == MINIMUM_EDGES_FOR_A_PATCH
                && !concave_vertices.contains(&corner.vertex)
                && ball_centres.contains_key(&corner.vertex)
        })
        .map(|corner| corner.vertex)
        .collect();

    BevelCornerPatchSet::new(
        patches,
        ball_centres,
        fill_ball_centres,
        warnings,
        through_corners,
        seam_rings,
    )
}
