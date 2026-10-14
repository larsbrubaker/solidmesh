//! The surfaces of `BevelCornerPatches`: one spherical polygon builder and the two solids drawn
//! over it - the SHELL a corner is patched with ([`build_shell`]), and the VOID CONE a fill is
//! clipped to ([`try_build_void_cone`]).
//!
//! Split from `BevelCornerPatches` ([`super::bevel_corner_patch`]) along the seam between the two
//! jobs that file was doing. What stays there is the READING of a corner - which faces meet at it,
//! which convexity, whether one ball touches them all - and every rule and refusal that goes with
//! it. What is here takes those answers as numbers and turns them into triangles.
//!
//! The two solids share `fan_directions` because they are the same surface at different radii:
//! the shell reads it twice, at the ball and past the vertex, and the cone reads it once over an
//! apex. That sharing is what makes the shell's own walls planar and the cone's walls exactly the
//! corner's faces. The seam arcs come from [`bevel_seam_ring`], and the ball's surface is refined
//! once inside them by [`bevel_patch_refinement`].

use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use super::{bevel_patch_refinement, bevel_seam_ring};

/// The fanned spherical polygon: its directions (the middle first), its triangles, and the boundary
/// arc of each sector as indices into the directions.
type Fan = (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<Vec<i32>>);

/// How square two adjacent faces have to be to each other before the edge they share is a
/// direction at all: a wedge of about a twentieth of a degree, there only to keep a normalize from
/// dividing by nothing.
const MINIMUM_EDGE_RAY_SINE: f64 = 1e-3;

/// How far outside a face plane an edge ray may sit and still count as lying in it. Two of the dots
/// tested are zero by construction and come back as rounding, so this only has to be above the
/// noise of a normalized cross product of unit vectors.
const RAY_OUTSIDE_FACE_TOLERANCE: f64 = 1e-9;

/// How near the apex the chorded cap may come, as a fraction of its own radius, before the cone is
/// not worth building - a hundredth is a cone a hundred times longer than the shell it bounds, and a
/// sign that the corner's faces are very nearly one plane.
const MINIMUM_CAP_COVERAGE: f64 = 1e-2;

/// How far the void cone's apex is set back into the corner, as a fraction of the cone's own
/// length, so that its lateral walls do not lie exactly in the part's face planes - a thousandth,
/// 0.013 mm on the corner that made it necessary. It makes the clip's `part + K_v` a pair of solids
/// that CROSS rather than share a wall over an area, which is the difference between a union the
/// kernel calls clean and one it calls self-intersecting.
const WALLS_CLEAR_THE_FACES_BY: f64 = 1e-3;

/// The shell itself - the spherical polygon whose corners are the face normals, drawn at the inner
/// and the outer radius, and closed by one planar wall over each of its boundary arcs - with the
/// seam rings. `None` where the normals have no middle.
///
/// One fanned polygon read twice. What keeps the walls between them planar is that both surfaces
/// are bounded by the SAME boundary arcs, and the wall over each arc lies in the plane those two
/// face normals span. Every triangle's winding is decided against the direction the solid faces
/// there (`add_oriented`), so the inner surface - the ball's own, with the material on the far side
/// of it - is simply the same triangle wound the other way. The inner surface is refined one level
/// inside its arcs ([`bevel_patch_refinement`]); the outer cap, which bounds nothing but air past
/// the corner, is left as the fan drew it.
pub fn build_shell(
    center: Vector3,
    normals: &[Vector3],
    inner_radius: f64,
    outer_radius: f64,
    segments: i32,
) -> Option<(Mesh, Vec<Vec<Vector3>>)> {
    let count = normals.len();
    let (mut directions, patch, arcs) = fan_directions(normals, segments)?;

    let middle = directions[0];
    let mut mesh = Mesh::new();

    // The BALL's own surface is refined one level inside its boundary - the seam arcs are left
    // exactly as they are. The refinement APPENDS to `directions`, so every index the coarse polygon
    // handed out still means what it did.
    let coarse = directions.len();
    let refined = bevel_patch_refinement::refine(&mut directions, &patch, Some(&arcs));

    for &direction in &directions {
        mesh.vertices.push(Vector3Float::from_vector3(
            center + (direction * inner_radius),
        ));
    }

    // The OUTER surface is left coarse and keeps only the directions the fan started with.
    let outer = directions.len() as i32;

    for &direction in &directions[..coarse] {
        mesh.vertices.push(Vector3Float::from_vector3(
            center + (direction * outer_radius),
        ));
    }

    // The boundary arcs are the SEAM, so they are not this mesh's own points: they are taken from
    // the one routine that says where the seam is, and the runs that end on these planes take their
    // arcs from the same arrays (BevelSeamRing). Writing them FROM the shared arrays is what makes
    // the agreement a fact rather than a coincidence two constructions have to keep.
    let mut seam_rings = Vec::with_capacity(count);

    for sector in 0..count {
        let seam = bevel_seam_ring::points(
            center,
            inner_radius,
            normals[sector],
            normals[(sector + 1) % count],
            segments,
        );
        let arc = &arcs[sector];

        for (step, &point) in seam.iter().enumerate() {
            mesh.vertices[arc[step] as usize] = Vector3Float::from_vector3(point);
        }

        seam_rings.push(seam);
    }

    for &(a, b, c) in &refined {
        let facing = directions[a as usize] + directions[b as usize] + directions[c as usize];

        // The inner surface is the ball's own, so the solid is on the far side of it from the centre
        add_oriented(&mut mesh, a, b, c, -facing);
    }

    for &(a, b, c) in &patch {
        let facing = directions[a as usize] + directions[b as usize] + directions[c as usize];

        add_oriented(&mut mesh, outer + a, outer + b, outer + c, facing);
    }

    for sector in 0..count {
        let arc = &arcs[sector];

        // Square to the plane the wall lies in, and pointing out of the patch rather than into it
        let mut facing = normals[sector].cross(normals[(sector + 1) % count]);

        if facing.dot(middle) > 0.0 {
            facing = -facing;
        }

        for step in 0..segments as usize {
            add_oriented(
                &mut mesh,
                arc[step],
                arc[step + 1],
                outer + arc[step + 1],
                facing,
            );
            add_oriented(
                &mut mesh,
                arc[step],
                outer + arc[step + 1],
                outer + arc[step],
                facing,
            );
        }
    }

    Some((mesh, seam_rings))
}

/// The spherical polygon over a ring of unit directions, as directions rather than as points - the
/// middle direction first, then every sample of the fan, with the triangles that join them and the
/// boundary arc of each sector. `None` where the ring has no middle - directions that sum to
/// nothing span no cone.
///
/// The polygon is fanned into one spherical triangle per boundary arc and each of those is
/// subdivided `segments` deep. The ribs from the middle out to each corner are built once and
/// shared by the two triangles either side of them, so the fan has no crack down it; the boundary
/// arcs are slerped at equal angles, which is what puts them on the swept fillets exactly.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn fan_directions(corners: &[Vector3], segments: i32) -> Option<Fan> {
    let count = corners.len();
    let segments = segments as usize;

    let mut middle = Vector3::ZERO;

    for &corner in corners {
        middle += corner;
    }

    if !(middle.length() > 1e-9) {
        return None;
    }

    middle = middle.get_normal();

    let mut directions = vec![middle];

    let mut ribs: Vec<Vec<i32>> = Vec::with_capacity(count);

    for &corner in corners {
        let mut rib = vec![0i32; segments + 1];
        rib[0] = 0;
        rib[segments] = add(&mut directions, corner);

        for (step, slot) in rib.iter_mut().enumerate().take(segments).skip(1) {
            *slot = add(
                &mut directions,
                slerp(middle, corner, step as f64 / segments as f64),
            );
        }

        ribs.push(rib);
    }

    let mut patch = Vec::new();
    let mut arcs = Vec::with_capacity(count);

    for sector in 0..count {
        let next = (sector + 1) % count;
        let mut rows: Vec<Vec<i32>> = Vec::with_capacity(segments + 1);

        for row in 0..=segments {
            let mut cells = vec![0i32; row + 1];
            cells[0] = ribs[sector][row];
            cells[row] = ribs[next][row];

            for step in 1..row {
                let from = directions[cells[0] as usize];
                let to = directions[cells[row] as usize];
                cells[step] = add(&mut directions, slerp(from, to, step as f64 / row as f64));
            }

            rows.push(cells);
        }

        for row in 0..segments {
            for step in 0..=row {
                patch.push((
                    rows[row][step],
                    rows[row + 1][step],
                    rows[row + 1][step + 1],
                ));
            }

            for step in 0..row {
                patch.push((
                    rows[row][step],
                    rows[row + 1][step + 1],
                    rows[row][step + 1],
                ));
            }
        }

        // The last row IS the boundary arc between the two corner directions, sampled at equal
        // angles over the same segment count the fillet's own profile is - which is the seam.
        arcs.push(rows.swap_remove(segments));
    }

    Some((directions, patch, arcs))
}

/// The void the corner owes its FILL, as a solid to clip that fill against - the cone of the
/// corner's own outward half spaces, bounded by a ball of radius `length` about the vertex. `None`
/// where it cannot be read, which leaves the fill to go on whole and say so.
///
/// `normals` are the corner's OUTWARD face normals in ring order - not the mirrored ring the fill's
/// shell is drawn over, which gives the same rays but the opposite cone. `clear_the_faces`
/// (normally false) sets the apex back into the corner so the walls do not lie in the part's own
/// face planes: taken by a corner FILL, whose cone is unioned with the part on its own, and not by
/// the void a 5.3 corner opens for two beads.
///
/// **One sign for the whole ring, and it is not the ball's.** The rays are `u_i = n_i x n_(i+1)`,
/// which fixes their axes but not their ends; signing each by the ball's centre goes wrong on a
/// wide cone. A ray of the cone is a direction lying in EVERY one of the corner's half spaces, so
/// both ends of the ring are tried and a saddle, where neither works, is refused
/// (`try_orient_out_of_the_corner`).
///
/// **The far end is a spherical cap, grown to hold the ball.** The cap is the same spherical
/// polygon the shell is built from, over the rays; it is chorded, so the reach is divided by how
/// near the apex the chorded cap actually comes (`smallest_plane_distance`), which makes it
/// circumscribe the ball whatever shape the cone is. Erring long is free: the cone is an UPPER bound
/// on what the fill may keep.
///
/// **The apex is set back a hair.** The lateral walls are the corner's own face planes, and two
/// solids that share a wall over an AREA are what an exact kernel handles worst ("5mm Bevel Center
/// Bottom.mcx" came back self-intersecting). Set back by [`WALLS_CLEAR_THE_FACES_BY`] of the length
/// along the rays' sum, every half space grows at once and the walls cross the part's faces instead;
/// the ball of radius `length` can then be missed by at most `d (1 - coverage)`, under `d` itself.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_build_void_cone(
    vertex: Vector3,
    normals: &[Vector3],
    length: f64,
    segments: i32,
    clear_the_faces: bool,
) -> Option<Mesh> {
    let count = normals.len();
    let mut rays = Vec::with_capacity(count);

    for i in 0..count {
        let ray = normals[i].cross(normals[(i + 1) % count]);

        if ray.length() < MINIMUM_EDGE_RAY_SINE {
            // Two adjacent faces flat enough to each other that the edge between them is no direction
            return None;
        }

        rays.push(ray.get_normal());
    }

    if !try_orient_out_of_the_corner(normals, &mut rays) {
        return None;
    }

    let (directions, patch, arcs) = fan_directions(&rays, segments)?;

    let middle = directions[0];

    // How near the apex the chorded cap comes anywhere, as a fraction of its own radius: the
    // smallest distance from the apex to any of the cap's triangle PLANES, a lower bound on the
    // closest the surface itself gets.
    let coverage = smallest_plane_distance(&directions, &patch);

    if !(coverage > MINIMUM_CAP_COVERAGE) {
        return None;
    }

    // Set back into the corner so the cone's walls are buried in material rather than lying in the
    // part's own face planes. The whole cone travels with it, so the reach is measured from the
    // moved apex.
    let mut apex = vertex;
    let mut set_back = 0.0;

    if clear_the_faces {
        let mut axis = Vector3::ZERO;

        for &ray in &rays {
            axis += ray;
        }

        if !(axis.length() > 1e-9) {
            // The rays cancel, so the cone has no axis to set its apex back along. Refused only HERE,
            // inside the branch that needs the axis.
            return None;
        }

        set_back = WALLS_CLEAR_THE_FACES_BY * length;
        apex = vertex - (axis.get_normal() * set_back);
    }

    let reach = (length / coverage) + set_back;

    let mut mesh = Mesh::new();

    // The apex, and then the cap - so every direction's vertex is one past its index in the fan
    mesh.vertices.push(Vector3Float::from_vector3(apex));

    for &direction in &directions {
        mesh.vertices
            .push(Vector3Float::from_vector3(apex + (direction * reach)));
    }

    for &(a, b, c) in &patch {
        let facing = directions[a as usize] + directions[b as usize] + directions[c as usize];

        add_oriented(&mut mesh, 1 + a, 1 + b, 1 + c, facing);
    }

    for sector in 0..count {
        let arc = &arcs[sector];

        // Square to the face plane the wall lies in, and pointing out of the cone rather than into it
        let mut facing = rays[sector].cross(rays[(sector + 1) % count]);

        if facing.dot(middle) > 0.0 {
            facing = -facing;
        }

        for step in 0..segments as usize {
            add_oriented(&mut mesh, 0, 1 + arc[step], 1 + arc[step + 1], facing);
        }
    }

    // Validated like every other solid built here: a cone the kernel would refuse has to be found
    // now, while there is still a fallback, rather than inside the clip.
    if classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false) == BooleanOperandVerdict::Clean {
        Some(mesh)
    } else {
        None
    }
}

/// Turns a ring of edge rays to point out of the corner along the VOID, in place, and says whether
/// that could be done at all. The two candidate answers are the rays as they came and the rays all
/// reversed - one sign for the ring rather than one per ray.
///
/// BOTH signs passing is refused too: every ray is then orthogonal to every face plane to within
/// [`RAY_OUTSIDE_FACE_TOLERANCE`] - the corner has flattened out - and the one taken would be
/// decided by which way the loop happened to run. Pick the wrong one and the cone points into the
/// material and clips the solid it was meant to bound away to nothing.
fn try_orient_out_of_the_corner(normals: &[Vector3], rays: &mut [Vector3]) -> bool {
    let forward = lies_in_every_face(normals, rays, 1);
    let backward = lies_in_every_face(normals, rays, -1);

    if forward == backward {
        // Neither sign bounds a convex cone, or the corner is flat enough that both do and there is
        // nothing to choose between them. Refused either way.
        return false;
    }

    if backward {
        for ray in rays.iter_mut() {
            *ray = -*ray;
        }
    }

    true
}

/// Whether every ray taken with the given sign lies in every one of the corner's half spaces, which
/// is the definition of the ring pointing out of the corner.
fn lies_in_every_face(normals: &[Vector3], rays: &[Vector3], sign: i32) -> bool {
    for &ray in rays {
        for &normal in normals {
            // Slack enough for the two dots that are zero by construction to come back a rounding
            // either side of it, and far tighter than any wedge a corner could be built from
            if normal.dot(ray * sign as f64) < -RAY_OUTSIDE_FACE_TOLERANCE {
                return false;
            }
        }
    }

    true
}

/// The nearest any of a fanned polygon's triangle PLANES comes to the origin, with the polygon's
/// directions read as unit vectors - a lower bound on the closest the SURFACE comes, so scaling by
/// it never leaves the ball uncovered. A cap NO triangle of which had a plane comes back as zero,
/// which fails the caller's coverage floor and refuses the corner out loud, rather than infinity,
/// which would collapse the cone to the apex.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn smallest_plane_distance(directions: &[Vector3], patch: &[(i32, i32, i32)]) -> f64 {
    let mut closest = f64::MAX;
    let mut measured = false;

    for &(a, b, c) in patch {
        let corner = directions[a as usize];
        let facet = (directions[b as usize] - corner).cross(directions[c as usize] - corner);
        let area = facet.length();

        if !(area > 0.0) {
            // A triangle with no plane says nothing about where its plane is
            continue;
        }

        measured = true;
        closest = closest.min(facet.dot(corner).abs() / area);
    }

    if measured {
        closest
    } else {
        0.0
    }
}

/// Appends a direction and answers its index.
fn add(directions: &mut Vec<Vector3>, direction: Vector3) -> i32 {
    directions.push(direction);

    directions.len() as i32 - 1
}

/// The point `fraction` of the way along the great circle from one unit vector to another. It lives
/// on [`bevel_seam_ring`] because the boundary arcs this fan ends on are the seam, and one formula
/// in one place keeps the patch's sampling and the published seam the same arithmetic.
fn slerp(from: Vector3, to: Vector3, fraction: f64) -> Vector3 {
    bevel_seam_ring::slerp(from, to, fraction)
}

/// One triangle, wound so its normal agrees with the direction the solid faces there. Deciding it
/// per triangle rather than by construction is what lets the faces be walked in whichever way round
/// the vertex the mesh happened to order them.
fn add_oriented(mesh: &mut Mesh, index0: i32, index1: i32, index2: i32, facing: Vector3) {
    let position0 = Vector3::from_vector3_float(mesh.vertices[index0 as usize]);
    let normal = (Vector3::from_vector3_float(mesh.vertices[index1 as usize]) - position0)
        .cross(Vector3::from_vector3_float(mesh.vertices[index2 as usize]) - position0);

    if normal.dot(facing) < 0.0 {
        mesh.faces.add(index0, index2, index1, &mesh.vertices);
    } else {
        mesh.faces.add(index0, index1, index2, &mesh.vertices);
    }
}
