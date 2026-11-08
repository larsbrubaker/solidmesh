//! Picking on the Fillet & Chamfer page: one part prepared for picking ([`PickSession`], over the
//! kernel's `BevelPickData`: the welded working copy, a ray tracing tree over it, its feature
//! edges at the bend threshold and its face regions) and what a ray means in each pick mode
//! ([`PickMode`]).
//!
//! - **Chain**: the edge nearest the hit, read as the tangent chain it lies in
//!   (`pick::interpretations_of_edge`, then `index_of_preferred_chain`);
//! - **Edge**: that edge alone (the first reading, the straight crease it is part of);
//! - **Face rim**: the rim of the smooth region the hit face belongs to (`pick::interpretations_of_face`);
//! - **Point**: the sharp corner nearest the hit (`pick::corner_candidates`), as a Point group.
//!
//! A pick answers the [`BevelEdgeGroup`] a click would store, which is a geometric signature in the
//! part's own space, and the polylines to draw it with. The page keeps the group as numbers
//! ([`group_to_numbers`] / [`group_from_numbers`]) and hands it back with each build.

use std::sync::Arc;

use solidmesh::math::{Matrix4X4, Ray, Vector3, Vector3Float};
use solidmesh::mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, MinimalTriangle, TraceableRef, VertexFunc,
};
use solidmesh::mesh::Mesh;
use solidmesh::pick::{self, pick_feature_edge_measured, BevelPickData};
use solidmesh::{BevelEdgeGroup, BevelSelectionKind, FeatureEdge, FeatureEdgeChain};

/// The bend threshold every pick and build on the page reads the part at, in degrees: an edge is
/// an edge where the surface turns more than this.
pub const BEND_THRESHOLD_DEGREES: f64 = 20.0;

/// How far from the cursor, in pixels, an edge or corner still counts as under it.
pub const PICK_PIXELS: f64 = 8.0;

/// What a click picks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickMode {
    Chain,
    Edge,
    FaceRim,
    Point,
}

impl PickMode {
    /// The mode a menu index names (0 chain, 1 edge, 2 face rim, 3 point).
    pub fn from_index(index: u32) -> Result<PickMode, String> {
        Ok(match index {
            0 => PickMode::Chain,
            1 => PickMode::Edge,
            2 => PickMode::FaceRim,
            3 => PickMode::Point,
            _ => return Err(format!("There is no pick mode {index}.")),
        })
    }
}

/// What one pick came to: the group a click stores, the segments it selects (pairs of points;
/// empty for a point), and a short label for the list.
#[derive(Debug, Clone)]
pub struct Pick {
    pub group: BevelEdgeGroup,
    pub segments: Vec<[Vector3; 2]>,
    pub point: Option<Vector3>,
    pub label: String,
}

/// The ray tracing tree over every face of `mesh`, each leaf naming its face.
fn build_bvh(mesh: &Mesh) -> Option<TraceableRef> {
    let corners: Arc<Vec<[Vector3Float; 3]>> = Arc::new(
        mesh.faces
            .iter()
            .map(|face| {
                [
                    mesh.vertices[face.v0 as usize],
                    mesh.vertices[face.v1 as usize],
                    mesh.vertices[face.v2 as usize],
                ]
            })
            .collect(),
    );
    let vertex_func: VertexFunc =
        Arc::new(move |face, corner| corners[face as usize][corner.min(2) as usize]);
    let leaves = (0..mesh.faces.len() as i32)
        .map(|face| Arc::new(MinimalTriangle::new(vertex_func.clone(), face)) as TraceableRef)
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(leaves, BvhCreationOptions::default())
}

/// One part, ready to pick on.
pub struct PickSession {
    data: BevelPickData,
    /// The sharp corners of the working mesh, and whether each can take a fillet.
    corners: Vec<(Vector3, bool)>,
}

impl PickSession {
    /// Prepares `mesh` for hovering: the working copy, its tree, its feature edges with every
    /// tangent chain walked, and its sharp corners.
    pub fn new(mesh: &Arc<Mesh>) -> PickSession {
        let data = BevelPickData::build(mesh, 0, build_bvh, BEND_THRESHOLD_DEGREES);
        data.prepare_for_hover();
        let working = data.working_mesh();
        let positions: Vec<Vector3> = working
            .vertices
            .iter()
            .map(|v| Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z)))
            .collect();
        let triangles: Vec<(i32, i32, i32)> =
            working.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
        let corners = pick::corner_candidates(&positions, &triangles, BEND_THRESHOLD_DEGREES)
            .into_iter()
            .map(|c| (positions[c.vertex as usize], c.allows_fillet))
            .collect();
        PickSession { data, corners }
    }

    /// Every feature edge of the part, as segments.
    pub fn feature_segments(&self) -> Vec<[Vector3; 2]> {
        self.data
            .pick_features()
            .edges()
            .iter()
            .map(|e| [e.start, e.end])
            .collect()
    }

    /// The sharp corners a Point pick can take, with whether each can take a fillet (a corner
    /// a sharp edge meets takes only a chamfer).
    pub fn corners(&self) -> &[(Vector3, bool)] {
        &self.corners
    }

    /// The face a ray meets first and where, or `None` on a miss.
    pub fn hit(&self, origin: Vector3, direction: Vector3) -> Option<(i32, Vector3)> {
        let bvh = self.data.bvh()?;
        let mut ray = Ray::new(origin, direction.get_normal());
        let hit = bvh.get_closest_intersection(&mut ray)?;
        let face = hit.closest_hit_as::<MinimalTriangle>()?.face_index;
        Some((face, hit.hit_position))
    }

    /// What a click along the ray would pick in `mode`. `pixel_size` is how many mm one pixel
    /// spans at one mm from the eye, which scales the pick tolerance with distance; `round` hides
    /// corners only a chamfer can take, since a fillet there would be refused.
    pub fn pick(
        &self,
        origin: Vector3,
        direction: Vector3,
        mode: PickMode,
        pixel_size: f64,
        round: bool,
    ) -> Result<Option<Pick>, String> {
        let Some((face, at)) = self.hit(origin, direction) else {
            return Ok(None);
        };
        let reach = (at - origin).length() * pixel_size * PICK_PIXELS;
        match mode {
            PickMode::Point => Ok(self.pick_corner(at, reach * 1.5, round)),
            PickMode::FaceRim => self.pick_face(face),
            PickMode::Chain | PickMode::Edge => self.pick_edge(face, at, reach, mode),
        }
    }

    fn pick_corner(&self, at: Vector3, reach: f64, round: bool) -> Option<Pick> {
        let mut best: Option<(f64, Vector3)> = None;
        for &(corner, allows_fillet) in &self.corners {
            if round && !allows_fillet {
                continue;
            }
            let distance = (corner - at).length();
            if distance <= reach && best.is_none_or(|(d, _)| distance < d) {
                best = Some((distance, corner));
            }
        }
        best.map(|(_, corner)| Pick {
            group: BevelEdgeGroup {
                selection_kind: BevelSelectionKind::Point,
                anchor_midpoint: corner,
                ..BevelEdgeGroup::default()
            },
            segments: Vec::new(),
            point: Some(corner),
            label: "corner".to_string(),
        })
    }

    fn pick_face(&self, face: i32) -> Result<Option<Pick>, String> {
        let (readings, _) = pick::interpretations_of_face(
            self.data.working_mesh(),
            Some(self.data.graph()),
            self.data.pick_features(),
            face,
            FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
            Some(self.data.face_regions()),
        )
        .map_err(|e| format!("{e:?}"))?;
        Ok(readings.first().map(|reading| {
            pick_of(
                reading.group().clone(),
                reading.edges(),
                "face rim".to_string(),
            )
        }))
    }

    fn pick_edge(
        &self,
        face: i32,
        at: Vector3,
        reach: f64,
        mode: PickMode,
    ) -> Result<Option<Pick>, String> {
        let pick = pick_feature_edge_measured(
            self.data.working_mesh(),
            self.data.pick_features(),
            face,
            at,
            reach,
            Matrix4X4::IDENTITY,
        )
        .map_err(|e| format!("{e:?}"))?;
        if !pick.is_hit() {
            return Ok(None);
        }
        let readings = pick::interpretations_of_edge(
            self.data.working_mesh(),
            Some(self.data.graph()),
            self.data.pick_features(),
            pick.edge_index(),
            FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
            self.data.ladder(),
            Some(self.data.face_regions()),
        )
        .map_err(|e| format!("{e:?}"))?;
        let index = match mode {
            PickMode::Chain => pick::index_of_preferred_chain(&readings).max(0),
            _ => 0,
        } as usize;
        let Some(reading) = readings.get(index) else {
            return Ok(None);
        };
        let tangent = reading.group().selection_kind == BevelSelectionKind::TangentChain;
        let label = match (tangent, reading.chain().is_some_and(|c| c.is_closed())) {
            (true, true) => "loop",
            (true, false) => "chain",
            _ => "edge",
        };
        Ok(Some(pick_of(
            reading.group().clone(),
            reading.edges(),
            label.to_string(),
        )))
    }
}

fn pick_of(group: BevelEdgeGroup, edges: &[FeatureEdge], kind: String) -> Pick {
    let count = edges.len();
    Pick {
        group,
        segments: edges.iter().map(|e| [e.start, e.end]).collect(),
        point: None,
        label: if count == 1 {
            kind
        } else {
            format!("{kind}, {count} edges")
        },
    }
}

/// How many numbers [`group_to_numbers`] writes.
pub const GROUP_NUMBERS: usize = 17;

/// A group as the flat numbers the page keeps: kind (0 edge, 1 chain, 2 face, 3 point), the
/// anchor's midpoint, direction, both normals with their plane offsets, and the curve and flat
/// tolerances.
pub fn group_to_numbers(group: &BevelEdgeGroup) -> [f64; GROUP_NUMBERS] {
    let kind = match group.selection_kind {
        BevelSelectionKind::Edge => 0.0,
        BevelSelectionKind::TangentChain => 1.0,
        BevelSelectionKind::FaceGroup => 2.0,
        BevelSelectionKind::Point => 3.0,
    };
    let (m, d, n0, n1) = (
        group.anchor_midpoint,
        group.anchor_direction,
        group.anchor_normal0,
        group.anchor_normal1,
    );
    [
        kind,
        m.x,
        m.y,
        m.z,
        d.x,
        d.y,
        d.z,
        n0.x,
        n0.y,
        n0.z,
        group.anchor_plane_distance0,
        n1.x,
        n1.y,
        n1.z,
        group.anchor_plane_distance1,
        group.curve_tolerance_degrees,
        group.flat_angle_degrees,
    ]
}

/// The group [`group_to_numbers`] wrote, or an error for a slice of the wrong length.
pub fn group_from_numbers(n: &[f64]) -> Result<BevelEdgeGroup, String> {
    if n.len() != GROUP_NUMBERS {
        return Err(format!(
            "A pick is {GROUP_NUMBERS} numbers, not {}.",
            n.len()
        ));
    }
    let selection_kind = match n[0] as i32 {
        0 => BevelSelectionKind::Edge,
        1 => BevelSelectionKind::TangentChain,
        2 => BevelSelectionKind::FaceGroup,
        3 => BevelSelectionKind::Point,
        other => return Err(format!("There is no pick kind {other}.")),
    };
    Ok(BevelEdgeGroup {
        selection_kind,
        anchor_midpoint: Vector3::new(n[1], n[2], n[3]),
        anchor_direction: Vector3::new(n[4], n[5], n[6]),
        anchor_normal0: Vector3::new(n[7], n[8], n[9]),
        anchor_plane_distance0: n[10],
        anchor_normal1: Vector3::new(n[11], n[12], n[13]),
        anchor_plane_distance1: n[14],
        source_item_id: None,
        curve_tolerance_degrees: n[15],
        flat_angle_degrees: n[16],
    })
}
