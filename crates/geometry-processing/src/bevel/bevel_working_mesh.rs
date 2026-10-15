//! The copy of a source mesh that a fillet or chamfer works against - welded if it needed welding,
//! turned right way out, freed of any face folded back over itself, and never the source's own
//! object: the remembered verdict on whether a source mesh is usable as it stands
//! ([`StampedVerdict`], held by whoever holds the source mesh as an owned value passed in, per the
//! bevel-core rule for per-object readings), carrying the working mesh into the source's placement
//! ([`carry_into`]), the folded-flap drop, the weld the kernel needs, the shared seam and openness
//! readings, and the working copy itself ([`create_from`], [`create_from_with_advisories`]).

use std::collections::{HashMap, HashSet};

use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::{Face, FaceList, Mesh};
use vector_math::{ieee_math, AxisAlignedBoundingBox, Matrix4X4, Vector3};

use super::{bevel_t_vertex_slivers, bevel_thin_slivers, BevelAdvisory, BevelAdvisoryCode};
use crate::topology::FeatureEdgeSet;

/// The least a working copy can be worth repairing for, as a fraction of its bounding box.
/// Anything flatter than this encloses nothing to speak of, so the sign of its volume is
/// arithmetic noise rather than a winding.
pub const MINIMUM_VOLUME_FRACTION_OF_BOUNDS: f64 = 1e-6;

/// How far apart two vertices may be and still be one vertex, as a fraction of the greater of the
/// part's size and its distance from the origin - the kernel's own number for the same question,
/// about a micron on a 20 mm part.
pub const SEAM_TOLERANCE_OF_THE_PARTICULAR_PART: f64 = 1e-5;

/// How far from a picked chain a feature edge may stand and still be that chain's own SEAM rather
/// than a wall beside it - fifty times the weld's tolerance, read off the part's DIAGONAL alone.
/// Bracketed by measurement on `Issue with even a single edge.mcx` (32 mm across), where 163 of
/// 179 size refusals are a crease doubled by a ribbon 2.5 to 6 microns wide: 16 microns there,
/// over twice the widest of them, while a ribbon a user could see stays three times outside it.
/// The scale is a trade, not a proof: 1.5 mm on a part three metres across.
pub const SEAM_EPSILON_OF_THE_PARTICULAR_PART: f64 = 5e-4;

/// Whether a source mesh can be used without the weld repair, stamped with the mesh's
/// `changed_count` when the verdict was taken, so a mesh edited in place answers afresh rather than
/// from a verdict about what it used to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StampedVerdict {
    /// The source mesh's `changed_count` when the verdict was taken, or `None` on a fresh entry.
    /// Optional because 0 is the very count a never-edited mesh carries: a plain zero read as a
    /// verdict already taken, and its default `is_usable` of false sent a good operand down the
    /// weld-repair path.
    pub changed_count: Option<i32>,
    /// Whether the mesh is usable as it stands.
    pub is_usable: bool,
}

/// Carries the working mesh by `matrix` (nothing for the identity), turning it right way out again
/// when the matrix mirrors.
pub fn carry_into(working_mesh: &mut Mesh, matrix: Matrix4X4) {
    if matrix == Matrix4X4::IDENTITY {
        return;
    }

    working_mesh.transform(matrix);

    if matrix.determinant() < 0.0 {
        turn_right_way_out(working_mesh);
    }
}

/// Reverses every face's winding.
pub fn turn_right_way_out(working_mesh: &mut Mesh) {
    working_mesh.reverse_faces();
}

/// Drops every pair of faces on the same three vertices wound opposite ways - a flap folded back
/// over itself - where cancelling them leaves no edge open; returns how many pairs went.
pub fn folded_flaps_dropped(mesh: &mut Mesh) -> i32 {
    // First-met order, so the pairing walks the faces in a fixed order.
    let mut faces_on_the_same_vertices: indexmap::IndexMap<(i32, i32, i32), Vec<usize>> =
        indexmap::IndexMap::new();
    for (face_index, face) in mesh.faces.iter().enumerate() {
        if face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0 {
            continue;
        }

        faces_on_the_same_vertices
            .entry(in_ascending_order(face))
            .or_default()
            .push(face_index);
    }

    let mut flap_faces: HashSet<usize> = HashSet::new();

    // Built once, and only if there is something to weigh against it - the counts are of the mesh
    // as it arrived, before any drop.
    let mut faces_on_the_edge: Option<HashMap<(i32, i32), i32>> = None;
    for (key, faces) in &faces_on_the_same_vertices {
        if faces.len() < 2 {
            continue;
        }

        let on_edges = faces_on_the_edge.get_or_insert_with(|| faces_on_each_edge(mesh));
        if !cancelling_leaves_no_open_edge(*key, on_edges) {
            continue;
        }

        // Paired off rather than dropped wholesale, so three copies of one triangle lose the two
        // that cancel and keep the one that is really the part's surface.
        for i in 0..faces.len() {
            if flap_faces.contains(&faces[i]) {
                continue;
            }

            for j in i + 1..faces.len() {
                if !flap_faces.contains(&faces[j])
                    && wound_opposite_ways(&mesh.faces[faces[i]], &mesh.faces[faces[j]])
                {
                    flap_faces.insert(faces[i]);
                    flap_faces.insert(faces[j]);
                    break;
                }
            }
        }
    }

    if flap_faces.is_empty() {
        return 0;
    }

    let mut kept = FaceList::new();
    let mut source_face_index = Vec::with_capacity(mesh.faces.len() - flap_faces.len());
    for (face_index, face) in mesh.faces.iter().enumerate() {
        if !flap_faces.contains(&face_index) {
            kept.add(face.v0, face.v1, face.v2, &mesh.vertices);
            source_face_index.push(face_index as i32);
        }
    }

    mesh.faces = kept;
    carry_face_data_onto(mesh, &source_face_index);
    mesh.mark_as_changed();
    (flap_faces.len() / 2) as i32
}

/// How many faces use each undirected edge.
pub fn faces_on_each_edge(mesh: &Mesh) -> HashMap<(i32, i32), i32> {
    let mut counts = HashMap::with_capacity(mesh.faces.len() * 3);
    let mut count = |a: i32, b: i32| {
        let edge = if a < b { (a, b) } else { (b, a) };
        *counts.entry(edge).or_insert(0) += 1;
    };
    for face in mesh.faces.iter() {
        count(face.v0, face.v1);
        count(face.v1, face.v2);
        count(face.v2, face.v0);
    }

    counts
}

/// True when dropping two copies of the (ascending) triangle opens none of its edges: no edge of
/// it is used by exactly three faces.
pub fn cancelling_leaves_no_open_edge(
    triangle: (i32, i32, i32),
    faces_on_the_edge: &HashMap<(i32, i32), i32>,
) -> bool {
    let (v0, v1, v2) = triangle;
    faces_on_the_edge[&(v0, v1)] != 3
        && faces_on_the_edge[&(v1, v2)] != 3
        && faces_on_the_edge[&(v0, v2)] != 3
}

/// Re-indexes the mesh's per-face colours and textures onto a new face list, where new face i was
/// old face `source_face_index[i]`.
pub fn carry_face_data_onto(mesh: &mut Mesh, source_face_index: &[i32]) {
    if let Some(colors) = &mesh.face_colors {
        let kept: Vec<_> = source_face_index
            .iter()
            .map(|&source| {
                colors
                    .get(source as usize)
                    .copied()
                    .unwrap_or(Mesh::UNCOLORED_FACE)
            })
            .collect();
        mesh.face_colors = Some(kept);
    }

    if !mesh.face_textures.is_empty() {
        let mut kept = polygon_mesh::FaceTextures::new();
        for (i, source) in source_face_index.iter().enumerate() {
            if let Some(texture) = mesh.face_textures.get(source) {
                kept.insert(i as i32, texture.clone());
            }
        }

        mesh.face_textures = kept;
    }
}

/// A face's three vertices in ascending order.
pub fn in_ascending_order(face: &Face) -> (i32, i32, i32) {
    let low = face.v0.min(face.v1.min(face.v2));
    let high = face.v0.max(face.v1.max(face.v2));
    (low, face.v0 + face.v1 + face.v2 - low - high, high)
}

/// True when two faces on the same vertices run them in opposite directions.
pub fn wound_opposite_ways(face: &Face, other: &Face) -> bool {
    (other.v0 == face.v0 && other.v1 == face.v2)
        || (other.v1 == face.v0 && other.v2 == face.v2)
        || (other.v2 == face.v0 && other.v0 == face.v2)
}

/// Whether the working mesh can be used as modelled: a usable, manifold operand. The verdict is
/// remembered against the source mesh's change count in `remembered`, which the source mesh's
/// holder keeps; with no source mesh it is taken afresh.
pub fn can_be_used_as_modelled(
    source_mesh: Option<&Mesh>,
    remembered: &mut StampedVerdict,
    working_mesh: &Mesh,
) -> bool {
    let Some(source_mesh) = source_mesh else {
        return is_usable_operand(Some(working_mesh), None) && working_mesh.is_manifold();
    };

    if remembered.changed_count != Some(source_mesh.changed_count()) {
        remembered.is_usable =
            is_usable_operand(Some(working_mesh), None) && working_mesh.is_manifold();
        remembered.changed_count = Some(source_mesh.changed_count());
    }

    remembered.is_usable
}

/// The working mesh, or a welded copy of it where the kernel would refuse it as modelled and the
/// weld makes it usable.
pub fn welded_if_the_kernel_would_refuse_it(
    source_mesh: Option<&Mesh>,
    remembered: &mut StampedVerdict,
    working_mesh: Mesh,
) -> Mesh {
    if can_be_used_as_modelled(source_mesh, remembered, &working_mesh) {
        return working_mesh;
    }

    let tolerance = seam_tolerance_for(&working_mesh);
    // A NaN reads as no extent, as zero does.
    if tolerance.is_nan() || tolerance <= 0.0 {
        // Nothing to scale a tolerance against, and a mesh with no extent is not one welding helps.
        return working_mesh;
    }

    let minimum_face_area = minimum_face_area_for(tolerance);
    let mut welded = working_mesh.copy();
    welded.merge_vertices(tolerance, minimum_face_area, None);
    welded.remove_degenerate_faces(minimum_face_area);
    welded.remove_unused_vertices();

    // A weld that did not make it usable is no improvement on the mesh the user modelled, and the
    // stages below still have their own fallbacks for it.
    if is_usable_operand(Some(&welded), None) {
        welded
    } else {
        working_mesh
    }
}

/// How far apart two vertices may be and still be one vertex on this mesh: the greater of its
/// diagonal and its distance from the origin, times [`SEAM_TOLERANCE_OF_THE_PARTICULAR_PART`]; 0
/// for a mesh with no extent. Shared with the published mesh's cleanup so the weld on the way IN
/// and the cleanup on the way OUT agree about which two points are the same point.
pub fn seam_tolerance_for(mesh: &Mesh) -> f64 {
    let bounds = mesh.get_axis_aligned_bounding_box();
    let diagonal = bounds.size().length();
    if diagonal.is_nan() || diagonal <= 0.0 || diagonal.is_infinite() {
        return 0.0;
    }

    ieee_math::max_f64(diagonal, furthest_from_the_origin(&bounds))
        * SEAM_TOLERANCE_OF_THE_PARTICULAR_PART
}

/// How far a feature edge may stand from a picked chain and still be read as that chain's own seam;
/// a mesh with no extent has no seams.
pub fn seam_epsilon_for(mesh: &Mesh) -> f64 {
    seam_epsilon_of_diagonal(mesh.get_axis_aligned_bounding_box().size().length())
}

/// The same number off bare vertices, so a caller holding the kernel's own positions reads what the
/// app reads.
pub fn seam_epsilon_for_positions(positions: &[Vector3]) -> f64 {
    let Some(&first) = positions.first() else {
        return 0.0;
    };

    let mut low = first;
    let mut high = first;
    for &point in positions {
        low = Vector3::component_min(low, point);
        high = Vector3::component_max(high, point);
    }

    seam_epsilon_of_diagonal((high - low).length())
}

/// [`SEAM_EPSILON_OF_THE_PARTICULAR_PART`] of a diagonal; 0 for none.
pub fn seam_epsilon_of_diagonal(diagonal: f64) -> f64 {
    if diagonal.is_nan() || diagonal <= 0.0 || diagonal.is_infinite() {
        return 0.0;
    }

    diagonal * SEAM_EPSILON_OF_THE_PARTICULAR_PART
}

/// How many undirected edges of a face list are used by exactly one face - how OPEN it is. Shared
/// by both source conditioning passes, which each weigh their own repair by it.
pub fn open_edge_count(faces: &FaceList) -> i32 {
    let mut used: HashMap<(i32, i32), i32> = HashMap::with_capacity(faces.len() * 3);
    let mut count = |a: i32, b: i32| {
        let edge = if a < b { (a, b) } else { (b, a) };
        *used.entry(edge).or_insert(0) += 1;
    };
    for face in faces.iter() {
        count(face.v0, face.v1);
        count(face.v1, face.v2);
        count(face.v2, face.v0);
    }

    used.values().filter(|&&count| count == 1).count() as i32
}

/// The smallest triangle worth keeping at a given seam tolerance: well under the tolerance
/// squared, so only triangles a weld at that tolerance itself collapsed are dropped.
pub fn minimum_face_area_for(seam_tolerance: f64) -> f64 {
    seam_tolerance * seam_tolerance / 10.0
}

/// How far the furthest corner of a box is from the origin, on its furthest axis.
pub fn furthest_from_the_origin(bounds: &AxisAlignedBoundingBox) -> f64 {
    let (min, max) = (bounds.min_xyz, bounds.max_xyz);
    ieee_math::max_f64(
        ieee_math::max_f64(min.x.abs(), max.x.abs()),
        ieee_math::max_f64(
            ieee_math::max_f64(min.y.abs(), max.y.abs()),
            ieee_math::max_f64(min.z.abs(), max.z.abs()),
        ),
    )
}

/// Whether a mesh may be handed to the kernel as an operand: present, not empty, and classified
/// clean under the transform it will be booleaned in with (`None` for the identity) - a mirroring
/// one turns every triangle inside out on its own.
pub fn is_usable_operand(mesh: Option<&Mesh>, matrix: Option<Matrix4X4>) -> bool {
    mesh.is_some_and(|mesh| {
        !mesh.vertices.is_empty()
            && classify_boolean_operand(mesh, matrix.unwrap_or(Matrix4X4::IDENTITY), false)
                == BooleanOperandVerdict::Clean
    })
}

/// Whether the whole mesh is wound the wrong way round: a closed manifold surface enclosing a real
/// volume (more than [`MINIMUM_VOLUME_FRACTION_OF_BOUNDS`] of its box), with its faces pointing in.
pub fn is_inside_out(mesh: &Mesh) -> bool {
    let volume = signed_volume(mesh);
    if volume >= 0.0 {
        // The common case, and the cheap one - the manifold walk below never runs for it.
        return false;
    }

    let bounds = mesh.get_axis_aligned_bounding_box();
    let bounds_volume = bounds.x_size() * bounds.y_size() * bounds.z_size();
    // Flat in at least one direction (or empty, or NaN): no inside, no winding worth flipping.
    if bounds_volume.is_nan() || bounds_volume <= 0.0 {
        return false;
    }

    -volume > bounds_volume * MINIMUM_VOLUME_FRACTION_OF_BOUNDS && mesh.is_manifold()
}

/// The volume the mesh's faces enclose, signed by which way they wind, from the divergence
/// theorem. Positive when they face out.
pub fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;
    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let v1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let v2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);
        total += v0.dot(v1.cross(v2));
    }

    total / 6.0
}

/// [`create_from_with_advisories`] with its advisories put aside.
pub fn create_from(source_mesh: &Mesh, remembered: &mut StampedVerdict) -> Mesh {
    create_from_with_advisories(source_mesh, remembered).0
}

/// A working copy of `source_mesh` - never the source's own mesh, which a design must get back
/// exactly as it went in - repaired if feature detection finds nothing in it, turned the right way
/// out if it arrived inside out, and freed of folded flaps, thin slivers and T-vertex slivers; with
/// what had to be repaired, for the caller to put in front of the user. The weld stays quiet by
/// product decision: it changes nothing about what the part IS. `remembered` is the source's
/// stamped verdict.
///
/// The split-chord pass (`BevelSplitChords`) would stand between the thin slivers and the
/// T-vertices and is deliberately NOT called: wired in, it moved six Onshape-control pins and the
/// face-edge survey and cleared neither part it was written for.
pub fn create_from_with_advisories(
    source_mesh: &Mesh,
    remembered: &mut StampedVerdict,
) -> (Mesh, Vec<BevelAdvisory>) {
    let mut working_mesh = source_mesh.copy();

    if FeatureEdgeSet::create(&working_mesh).count() == 0 {
        working_mesh.clean_and_merge();
    }

    // Turned the right way out BEFORE the weld is considered: an inside-out mesh is one the kernel
    // refuses for a reason no welding fixes, so asked the other way round every seam the weld closed
    // was thrown away.
    let mut advisories = Vec::new();

    if is_inside_out(&working_mesh) {
        turn_right_way_out(&mut working_mesh);
        advisories.push(BevelAdvisory::new(
            BevelAdvisoryCode::SourceMeshWasInsideOut,
            Vec::new(),
        ));
    }

    let mut repaired =
        welded_if_the_kernel_would_refuse_it(Some(source_mesh), remembered, working_mesh);

    // After the weld, because a weld is what puts a flap's two faces on the same three vertices.
    let flap_pairs = folded_flaps_dropped(&mut repaired);

    if flap_pairs > 0 {
        advisories.push(BevelAdvisory::new(
            BevelAdvisoryCode::SourceMeshHadFoldedFlaps,
            vec![flap_pairs.into()],
        ));
    }

    // The seams the weld would have closed, ahead of the all-or-nothing T-vertex pass.
    let thin = bevel_thin_slivers::collapsed_from(&mut repaired);

    if thin > 0 {
        advisories.push(BevelAdvisory::new(
            BevelAdvisoryCode::SourceMeshHadThinSlivers,
            vec![thin.into()],
        ));
    }

    let slivers = bevel_t_vertex_slivers::dropped_from(&mut repaired);

    if slivers > 0 {
        advisories.push(BevelAdvisory::new(
            BevelAdvisoryCode::SourceMeshHadTVertexSlivers,
            vec![slivers.into()],
        ));
    }

    (repaired, advisories)
}
