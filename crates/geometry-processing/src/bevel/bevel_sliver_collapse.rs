//! Takes the triangles a boolean leaves lying IN a flat face with all but no ALTITUDE out of a
//! mesh, by recutting each of them together with the face it leans on
//! (`Bevel/BevelSliverCollapse.cs`, a C# `internal static class`, so a module of free functions
//! here). `BevelPublishedMesh` runs it on the mesh a bevel publishes.
//!
//! **Why altitude and not area.** `Mesh.RemoveDegenerateFaces` drops triangles by AREA, and the
//! ones a near-tangent union leaves are not small in area because they are small: they are small
//! because they are THIN. Measured on "Inner Right Sides and Center.mcx", 24 of them lie in one
//! corner face's own plane with edges up to 7.8 mm and areas around 7e-5 mm^2, and two more span a
//! whole 22.6 and 28.3 mm face diagonal. An area threshold that reached those would eat every
//! genuinely small facet of a fillet with it. What is actually wrong with them is the ratio: twice
//! the area over the longest edge - the height of the triangle standing on that edge - is about
//! 2e-5 mm.
//!
//! **Why they have to go rather than be shaded better.** Nothing about them is wrong in POSITION;
//! every vertex is on the surface. But a triangle whose three points are collinear to within a
//! hundredth of a micron has no normal to compute - the cross product of its two nearly parallel
//! edges is float noise, and it comes out 83 to 94 degrees off the plane the triangle demonstrably
//! lies in. The viewport flat-shades with that stored normal and the outline pass draws an edge
//! wherever two stored normals differ by more than 45 degrees, so what the user sees is thin
//! triangle outlines drawn across a face that should read as one flat plane. Recomputing the
//! normal cannot help, because the recomputed one is the same noise.
//!
//! **The operation.** A RECUT of an edge, not a vertex move: the sliver and the face across one of
//! its edges are the two halves of a quadrilateral, and that quadrilateral is cut along its other
//! diagonal instead. No vertex moves and no vertex is lost, every edge of the surface still has
//! exactly two faces on it, and the only material that changes hands is the tetrahedron the four
//! corners span - whose height, since the apex lies on the edge to within the altitude, is that
//! altitude: at most a third of the neighbour's area times a weld tolerance. `BevelPublishedMesh`
//! measures the whole mesh's volume across the pass anyway and publishes the uncleaned mesh if it
//! moved.

use std::collections::{HashMap, HashSet};

use polygon_mesh::{CancelToken, Face, FaceList, Mesh};
use vector_math::{Vector3, Vector3Float};

use super::level_line::work_budget::WorkError;

/// How many times the whole mesh is swept. A sweep recuts every sliver whose neighbour no other
/// recut in the same sweep has claimed, so a run of them hanging off each other needs one sweep
/// per link - measured, three sweeps to finish "Inner Right Sides and Center.mcx" and nine to
/// finish "5mm Bevel Around Center.mcx", where 542 of them start. Termination does not rest on
/// this number, because no recut is taken unless it leaves both its triangles thicker than the one
/// it removed; the bound is here so that the worst a mesh nobody has seen yet can cost is a known
/// amount of work.
pub const MAXIMUM_PASSES: i32 = 16;

/// An undirected edge, its lower vertex first.
type EdgeKey = (i32, i32);

/// The height of the triangle standing on its own longest edge - twice its area over that edge -
/// or zero when it has no edge to stand on.
///
/// This is the reading that says how near to a straight line the three points are, in the units
/// the part is measured in, and it is the one thing a "sliver" means that neither area nor edge
/// length says on its own.
pub fn altitude_of(p0: Vector3Float, p1: Vector3Float, p2: Vector3Float) -> f64 {
    measure(p0, p1, p2).0
}

/// Collapses every face thinner than `altitude_tolerance`, and returns whether it changed
/// anything. `mesh` is cleaned in place (only the caller's own copy is ever passed);
/// `BevelPublishedMesh` passes the mesh's WELD tolerance, and the reasoning for that is there. The
/// token is checked once per sweep; a cancel comes back as [`WorkError::Cancelled`].
pub fn collapse(
    mesh: &mut Mesh,
    altitude_tolerance: f64,
    cancellation_token: &CancelToken,
) -> Result<bool, WorkError> {
    let mut changed_anything = false;

    for _pass in 0..MAXIMUM_PASSES {
        if cancellation_token.is_cancelled() {
            return Err(WorkError::Cancelled);
        }

        if !collapse_one_pass(mesh, altitude_tolerance) {
            break;
        }

        changed_anything = true;
    }

    Ok(changed_anything)
}

/// One sweep of the mesh: collapse every sliver that can be collapsed without touching a face
/// another collapse in this same sweep already claimed, and rebuild the face list once at the end.
fn collapse_one_pass(mesh: &mut Mesh, altitude_tolerance: f64) -> bool {
    let slivers = slivers_by_thinnest_first(mesh, altitude_tolerance);

    if slivers.is_empty() {
        return false;
    }

    let face_count = mesh.faces.len();
    let edges = faces_by_edge(&mesh.faces);

    // A face is claimed the moment a collapse reads or rewrites it, so two collapses can never
    // both be reasoning about the same triangle from opposite sides. What one sweep leaves the
    // next takes.
    let mut claimed = vec![false; face_count];
    let mut dropped = vec![false; face_count];
    // Lookup only, never iterated, so a HashMap cannot leak an order into the result.
    let mut split_in_two: HashMap<usize, (Face, Face)> = HashMap::new();

    // The edges the sweep's own recuts have introduced. The map above is the mesh as the sweep
    // found it, and two recuts that each want to cut along the same new edge would leave four
    // faces on it.
    let mut cut_along: HashSet<EdgeKey> = HashSet::new();

    for (altitude, sliver_index) in slivers {
        if claimed[sliver_index] {
            continue;
        }

        let Some((neighbour_index, first, second)) =
            best_cut(mesh, &edges, &cut_along, &claimed, sliver_index, altitude)
        else {
            continue;
        };

        cut_along.insert(edge_key(first.v1, first.v2));
        claimed[sliver_index] = true;
        claimed[neighbour_index] = true;
        dropped[sliver_index] = true;
        split_in_two.insert(neighbour_index, (first, second));
    }

    if split_in_two.is_empty() {
        return false;
    }

    rebuild(mesh, &dropped, &split_in_two);

    true
}

/// The best of the three edges of a sliver to recut it across, as the neighbour it shares that
/// edge with and the two triangles the pair becomes, or `None` when no edge of it can be recut into
/// anything better than it already is.
///
/// Recutting an edge is the flip of the quadrilateral the two faces on it make: the sliver goes,
/// and the face it leaned on is cut instead from the sliver's apex to its own far corner. Every
/// directed edge the pair carried comes back except the shared one, so the surface is closed after
/// it if it was closed before.
///
/// **Why all three edges and not just the long one.** The obvious cut is across the longest edge,
/// the one the apex is lying on. But a boolean leaves whole RUNS of these - a line of vertices
/// along a bead's straight tangency, with thin triangles on both sides of it - and there the face
/// across the long edge is another sliver, so that cut hands back two more of them and the next
/// sweep undoes it again. Measured on "Inner Right Sides and Center.mcx" that is a fixed point: 4
/// recut and 4 recreated, for as many sweeps as it is given. So every edge is tried and the one
/// that leaves the THICKEST worst triangle wins.
///
/// **And why it has to be an improvement.** A cut is only taken when both triangles it leaves
/// stand higher than the sliver did. That is what makes the sweep terminate rather than cycle:
/// every cut strictly raises the mesh's own thinnest face, which cannot go on forever.
fn best_cut(
    mesh: &Mesh,
    edges: &HashMap<EdgeKey, Vec<usize>>,
    cut_along: &HashSet<EdgeKey>,
    claimed: &[bool],
    sliver_index: usize,
    sliver_altitude: f64,
) -> Option<(usize, Face, Face)> {
    let faces = &mesh.faces;
    let vertices = &mesh.vertices;
    let sliver = faces[sliver_index];

    let mut best = None;
    let mut best_altitude = sliver_altitude;

    for edge in 0..3 {
        let (apex, from, to) = corners_against(&sliver, edge);

        let Some(on_that_edge) = edges.get(&edge_key(from, to)).filter(|on| on.len() == 2) else {
            // Not a manifold edge, so there is no one face across it and no way to stay closed.
            continue;
        };

        let neighbour_index = if on_that_edge[0] == sliver_index {
            on_that_edge[1]
        } else {
            on_that_edge[0]
        };

        if neighbour_index == sliver_index || claimed[neighbour_index] {
            continue;
        }

        let opposite = third_corner(&faces[neighbour_index], to, from);

        if opposite < 0 || opposite == apex {
            // Either the neighbour does not carry this edge the other way round - so the two do
            // not agree which way the surface faces - or the pair is a fin standing on all three of
            // its edges, which no recut can take apart.
            continue;
        }

        if edges.contains_key(&edge_key(apex, opposite))
            || cut_along.contains(&edge_key(apex, opposite))
        {
            // The cut would be along an edge the mesh already has somewhere else, which would leave
            // that edge carrying four faces.
            continue;
        }

        // The neighbour carries the edge as to -> from, so it reads round as (to, from, opposite).
        let first = Face::new(to, apex, opposite, vertices);
        let second = Face::new(apex, from, opposite, vertices);

        if !faces_the_same_way(&first, &faces[neighbour_index])
            || !faces_the_same_way(&second, &faces[neighbour_index])
        {
            // The four corners make a REFLEX quadrilateral, and recutting one of those along its
            // other diagonal folds it: the diagonal is outside the shape, so one of the two
            // triangles comes out wound the other way and the pair covers the same material twice.
            // A mesh that does that is one the kernel refuses as an operand - measured, it is what
            // BevelPairCornerTrimTests sees the moment this check is taken out - so the reading is
            // taken against the NEIGHBOUR's normal, which is the only one of the four faces
            // involved that is thick enough to have a trustworthy one.
            continue;
        }

        let worst = f64::min(
            altitude_of(
                vertices[to as usize],
                vertices[apex as usize],
                vertices[opposite as usize],
            ),
            altitude_of(
                vertices[apex as usize],
                vertices[from as usize],
                vertices[opposite as usize],
            ),
        );

        if worst > best_altitude {
            best_altitude = worst;
            best = Some((neighbour_index, first, second));
        }
    }

    best
}

/// Whether a face is wound the same way round as another one it is being cut from.
fn faces_the_same_way(piece: &Face, parent: &Face) -> bool {
    Vector3::from_vector3_float(piece.normal).dot(Vector3::from_vector3_float(parent.normal)) > 0.0
}

/// Every face thinner than the tolerance, thinnest first.
///
/// Thinnest first so that a sweep spends its one claim per face on the worst offenders. Which face
/// a contested pair goes to is otherwise arbitrary, and arbitrary here would mean the answer
/// depended on face order. C#'s `List.Sort` is unstable; equal altitudes are broken by face index
/// here so the order is fixed.
fn slivers_by_thinnest_first(mesh: &Mesh, altitude_tolerance: f64) -> Vec<(f64, usize)> {
    let mut found = Vec::new();

    for (face_index, face) in mesh.faces.iter().enumerate() {
        if face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0 {
            // A repeated corner is dropped by the weld pass ahead of this one; there is no apex
            // here to split anything at.
            continue;
        }

        let (altitude, _, longest_length) = measure(
            mesh.vertices[face.v0 as usize],
            mesh.vertices[face.v1 as usize],
            mesh.vertices[face.v2 as usize],
        );

        if longest_length > 0.0 && altitude < altitude_tolerance {
            found.push((altitude, face_index));
        }
    }

    found.sort_by(|left, right| left.0.total_cmp(&right.0).then(left.1.cmp(&right.1)));

    found
}

/// The altitude, which edge the triangle stands on, and how long that edge is.
fn measure(p0: Vector3Float, p1: Vector3Float, p2: Vector3Float) -> (f64, i32, f64) {
    let v0 = Vector3::from_vector3_float(p0);
    let v1 = Vector3::from_vector3_float(p1);
    let v2 = Vector3::from_vector3_float(p2);

    let lengths = [(v1 - v0).length(), (v2 - v1).length(), (v0 - v2).length()];

    let mut longest_edge = 0;

    for edge in 1..3 {
        if lengths[edge] > lengths[longest_edge] {
            longest_edge = edge;
        }
    }

    let longest_length = lengths[longest_edge];

    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(longest_length > 0.0) {
        return (0.0, longest_edge as i32, 0.0);
    }

    let area = (v1 - v0).cross(v2 - v0).length() / 2.0;

    (
        2.0 * area / longest_length,
        longest_edge as i32,
        longest_length,
    )
}

/// The corner opposite the given edge, and that edge's two ends in the face's own order.
fn corners_against(face: &Face, longest_edge: i32) -> (i32, i32, i32) {
    match longest_edge {
        0 => (face.v2, face.v0, face.v1),
        1 => (face.v0, face.v1, face.v2),
        _ => (face.v1, face.v2, face.v0),
    }
}

/// The corner of the face that is not on the directed edge from -> to, or -1 when the face does
/// not carry that edge in that direction.
fn third_corner(face: &Face, from: i32, to: i32) -> i32 {
    if face.v0 == from && face.v1 == to {
        return face.v2;
    }

    if face.v1 == from && face.v2 == to {
        return face.v0;
    }

    if face.v2 == from && face.v0 == to {
        return face.v1;
    }

    -1
}

/// Which faces sit on each undirected edge.
fn faces_by_edge(faces: &FaceList) -> HashMap<EdgeKey, Vec<usize>> {
    let mut edges: HashMap<EdgeKey, Vec<usize>> = HashMap::with_capacity(faces.len() * 3);

    for (face_index, face) in faces.iter().enumerate() {
        for key in [
            edge_key(face.v0, face.v1),
            edge_key(face.v1, face.v2),
            edge_key(face.v2, face.v0),
        ] {
            edges
                .entry(key)
                .or_insert_with(|| Vec::with_capacity(2))
                .push(face_index);
        }
    }

    edges
}

fn edge_key(a: i32, b: i32) -> EdgeKey {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// Writes the sweep's result back: the dropped faces gone, the split ones as their two pieces, and
/// every other face where it was.
///
/// Per-face colors and textures are keyed by face INDEX, so a rebuild that drops and appends faces
/// leaves them painted onto whatever triangle lands on the index. They are carried over here from
/// the face each new one came from - the same thing `Mesh.RemapFaceData` does for the operations
/// inside PolygonMesh, which is not visible from C#'s assembly.
fn rebuild(mesh: &mut Mesh, dropped: &[bool], split_in_two: &HashMap<usize, (Face, Face)>) {
    let was = std::mem::take(&mut mesh.faces);
    let mut rebuilt = FaceList::new();
    let mut came_from: Vec<usize> = Vec::with_capacity(was.len());

    for (face_index, face) in was.iter().enumerate() {
        if dropped[face_index] {
            continue;
        }

        if let Some((first, second)) = split_in_two.get(&face_index) {
            rebuilt.push(*first);
            rebuilt.push(*second);
            came_from.push(face_index);
            came_from.push(face_index);
        } else {
            rebuilt.push(*face);
            came_from.push(face_index);
        }
    }

    mesh.faces = rebuilt;

    if let Some(colors) = &mesh.face_colors {
        let rebuilt_colors = came_from
            .iter()
            .map(|&source| {
                if source < colors.len() {
                    colors[source]
                } else {
                    Mesh::UNKNOWN_FACE_COLOR
                }
            })
            .collect();

        mesh.face_colors = Some(rebuilt_colors);
    }

    if !mesh.face_textures.is_empty() {
        let mut rebuilt_textures = std::collections::BTreeMap::new();

        for (face_index, &source) in came_from.iter().enumerate() {
            if let Some(texture) = mesh.face_textures.get(&(source as i32)) {
                rebuilt_textures.insert(face_index as i32, texture.clone());
            }
        }

        mesh.face_textures = rebuilt_textures;
    }

    mesh.mark_as_changed();
}
