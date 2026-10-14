//! Recognising the unselected crease that lies BETWEEN two selected runs at a 5.3 corner - the one
//! edge whose vote `BevelChainEnds` has to pass over, as against a third crease that genuinely
//! bounds the corner and still caps - and, of the creases it does recognise, which ones the pair's
//! rolling ball can go round ([`takes_the_partners_ball`]).
//!
//! It lives apart from the cap scan that asks it because it is a different question from the one
//! that file is about. Everything there weighs a plane - how square is it to the run, can the
//! segment absorb the slide onto it - and this weighs nothing: it asks whether two faces are the
//! same SURFACE, which has an exact answer that only floating point blurs. Keeping the two apart is
//! also what keeps the tolerance below from drifting into the lean guards, which it looks like and
//! is not.

use vector_math::{math_helper, Vector3};

use crate::topology::{EdgeConvexity, FeatureEdge};

/// How closely two faces meeting at one vertex have to agree on their outward normal to count as
/// the same SURFACE - two triangles of one wall rather than two sides of a corner. Half a degree:
/// this is asking an exact question that arithmetic has rounded, not weighing a lean. A WARPED
/// wall's two triangles are one surface and two planes, and no tolerance that admitted them would
/// still refuse a genuine third crease - so that case is settled by surface id instead.
fn same_surface_agreement() -> f64 {
    math_helper::degrees_to_radians(0.5).cos()
}

/// The run's own two tangent faces and those of every selected neighbour of its convexity departing
/// from the vertex, with no faces (`-1`, `-1`, `None`) as [`Default`].
#[derive(Debug, Clone, Copy)]
pub struct PairFaces<'a> {
    /// The face `own_normal0` came off.
    pub own_face0: i32,
    /// The face `own_normal1` came off.
    pub own_face1: i32,
    /// The faces the neighbour normals came off, in the same order and the same count.
    pub neighbour_faces: Option<&'a [i32]>,
}

impl Default for PairFaces<'_> {
    fn default() -> Self {
        Self {
            own_face0: -1,
            own_face1: -1,
            neighbour_faces: None,
        }
    }
}

/// Whether an unselected feature edge at a 5.3 corner is the one lying BETWEEN the two selected
/// runs - bounded on both sides by surfaces one or other of them is already tangent to - rather
/// than a third crease that genuinely bounds the corner.
///
/// `edge` departs from the end vertex; `own_normal0`/`own_normal1` are the outward normals of the
/// two surfaces this run's bevel is tangent to; `neighbour_normals` are the same two for every
/// selected edge of this run's convexity departing from the vertex. `surface_of_face` is every
/// face's logical surface at the group's own curve tolerance (`FaceGroup::label_surfaces`), as
/// something to ASK rather than to have: the labelling is a flood over the whole mesh and most
/// corners are settled by the normals without it, so it is read once and only on the way into the
/// branch that wants it. `None` asks the exact normal question alone.
///
/// Why by surface and not by face index: faces are triangles and a flat wall is at least two of
/// them, so comparing outward normals (sense included) compares the SURFACES; every face in
/// question passes through the end vertex, so agreeing on a normal is agreeing on a plane. What the
/// normals cannot read is a WARPED wall, so the surface is then asked for by name - but only for an
/// edge that is a boundary BETWEEN two surfaces at the labelling's own tolerance
/// ([`is_crease_between_surfaces`]): an edge whose own two faces flood together is a fold inside one
/// surface (a shallow step a run ending there must stop on) and keeps its vote.
pub fn lies_between_selected_runs<'s>(
    edge: &FeatureEdge,
    own_normal0: Vector3,
    own_normal1: Vector3,
    neighbour_normals: &[Vector3],
    faces: PairFaces<'_>,
    surface_of_face: Option<&mut dyn FnMut() -> Option<&'s [i32]>>,
) -> bool {
    let by_normal0 =
        is_pair_surface_by_normal(edge.normal0, own_normal0, own_normal1, neighbour_normals);
    let by_normal1 =
        is_pair_surface_by_normal(edge.normal1, own_normal0, own_normal1, neighbour_normals);

    if by_normal0 && by_normal1 {
        // Every flat wall and every prismatic facetted one, settled without a lookup.
        return true;
    }

    let Some(surface_of_face) = surface_of_face else {
        return false;
    };

    let Some(surfaces) = surface_of_face() else {
        return false;
    };

    if !is_crease_between_surfaces(edge, surfaces) {
        return false;
    }

    (by_normal0 || is_pair_surface_by_id(edge.face0, faces, surfaces))
        && (by_normal1 || is_pair_surface_by_id(edge.face1, faces, surfaces))
}

/// Whether the crease [`lies_between_selected_runs`] just recognised is one the pair's rolling
/// ball can go ROUND - the only shape of 5.3 corner where the trim by the partner's ball tube
/// (`BevelPairCornerTrim`) is the finish, as against the plain union of the two overshooting
/// cutters.
///
/// Round an INSIDE crease the ball rolls from one run into the other, staying in contact, and
/// leaves the HIGHER of the two cylinders: the trim. Over an OUTSIDE edge there is nothing on the
/// far side for the ball to stay against, so the two fillets MITRE - the lower of the two
/// cylinders, which the union of the overshooting cutters already cuts; trimming there would dome
/// the corner a full radius above the mitre. Only the trim is refused for a convex crease, not the
/// recognition: its cap vote is still passed over and both cutters still overshoot.
pub fn takes_the_partners_ball(between_runs: &FeatureEdge) -> bool {
    // Concave and nothing else. A flat edge never reaches here - FeatureEdgeSet drops those - so
    // this is the whole of the distinction, said as the geometry means it rather than as a
    // negation.
    between_runs.convexity == EdgeConvexity::Concave
}

/// Whether this edge is a boundary between two surfaces at the labelling's own tolerance, rather
/// than a fold inside one of them - the gate on the whole id reading.
fn is_crease_between_surfaces(edge: &FeatureEdge, surface_of_face: &[i32]) -> bool {
    let count = surface_of_face.len() as i64;
    edge.face0 >= 0
        && (edge.face0 as i64) < count
        && edge.face1 >= 0
        && (edge.face1 as i64) < count
        && surface_of_face[edge.face0 as usize] != surface_of_face[edge.face1 as usize]
}

/// Whether one face's outward normal is that of a surface this run, or a selected neighbour of
/// its own convexity ending here, is tangent to.
fn is_pair_surface_by_normal(
    normal: Vector3,
    own_normal0: Vector3,
    own_normal1: Vector3,
    neighbour_normals: &[Vector3],
) -> bool {
    let agreement = same_surface_agreement();

    if normal.dot(own_normal0) >= agreement || normal.dot(own_normal1) >= agreement {
        return true;
    }

    neighbour_normals
        .iter()
        .any(|&neighbour| normal.dot(neighbour) >= agreement)
}

/// The same question asked of the labelling: whether one face belongs to a SURFACE this run, or
/// such a neighbour, is tangent to - which is what a warped wall's two triangles cannot say by
/// normal.
fn is_pair_surface_by_id(face: i32, faces: PairFaces<'_>, surface_of_face: &[i32]) -> bool {
    if face < 0 || face as usize >= surface_of_face.len() {
        return false;
    }

    let surface = surface_of_face[face as usize];

    if is_same_surface(surface, faces.own_face0, surface_of_face)
        || is_same_surface(surface, faces.own_face1, surface_of_face)
    {
        return true;
    }

    if let Some(neighbour_faces) = faces.neighbour_faces {
        for &neighbour in neighbour_faces {
            if is_same_surface(surface, neighbour, surface_of_face) {
                return true;
            }
        }
    }

    false
}

/// Whether a labelled face belongs to the surface already read off another one.
fn is_same_surface(surface: i32, other_face: i32, surface_of_face: &[i32]) -> bool {
    other_face >= 0
        && (other_face as usize) < surface_of_face.len()
        && surface_of_face[other_face as usize] == surface
}
