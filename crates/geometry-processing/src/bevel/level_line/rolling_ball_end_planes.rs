//! Where a one-chain band tagged from the rolling ball's sides (`rolling_ball_band_tags`) ends at an
//! open end that carries no end ray: the end face's own plane. Read twice from the same tags: by
//! `band_gap_build_sides::tag_the_sides`, right after the sweep (`rolling_ball_sweep`), to take out
//! of each side what lies wholly past such a plane ([`leave_what_lies_past`]); and by
//! `one_chain_room::read_in_the_build`, so a foreign edge wholly past one sets no room
//! (`foreign_edge_room_per_side`).
//!
//! **Why.** At an end on an end face with no end ray the ball's last station stands on the face's
//! plane and the round is cut flat in it: past that plane, on the side away from the chain, the
//! band does not exist, so nothing there can be cut by it or crossed by it. The sides read from the
//! caller's regions can run on there all the same (the T Handle's two back-to-back pockets on a
//! web 6 thick: every pocket wall is one region and the crossbar round them another, so the far
//! pocket's junction is a crease between the chain's own two sides, 6 from the chain's web end
//! through the web's solid). The field past an open end with no ray is the distance to the end
//! vertex less the end's setback, a reading that does not know the face is there; read against it,
//! that crease set the room at 6 over the end's setback per unit (4.65 at a size of 12), though no
//! ball rolled along the chain reaches it. The sweep cannot take those triangles out by its own
//! planes: past an open end no station's cross-section holds them, so it reads them as neither.
//!
//! **The rule.** An end's plane ([`end_face_planes`]) is the plane of the end face round the end
//! vertex: the triangles there in neither side (untagged, or an end region), all in one plane to
//! [`COPLANAR_TOLERANCE`], not holding the chain's direction; faced toward the chain. An end with an
//! end ray, an open end with no end face (a free end, whose band tapers onto its end vertex), a
//! curved end face or several end faces has none, and nothing changes there. A point is past a
//! plane only by more than the snap ([`is_wholly_past`]); a triangle or an edge straddling the
//! plane, or lying in it, still counts. A leaning end face reads its own plane, not one square to
//! the chain.

use vector_math::Vector3;

use super::end_face_sense::{PointPlane, COPLANAR_TOLERANCE};

/// One triangle by its three corners and its band tag.
pub type TaggedTriangle = ((i32, i32, i32), i32);

fn has(t: (i32, i32, i32), v: i32) -> bool {
    t.0 == v || t.1 == v || t.2 == v
}

/// The plane of the end face at `end`, the chain running on to `next`, faced toward the chain:
/// `None` where the triangles round `end` in neither side (tag 0, or an end region's 3 and 4) are
/// none, do not share one plane, or hold the chain's direction.
fn end_face_plane(
    position: &dyn Fn(i32) -> Vector3,
    triangles: &[TaggedTriangle],
    end: i32,
    next: i32,
) -> Option<PointPlane> {
    let v = position(end);
    let into = (position(next) - v).get_normal();
    let mut normal: Option<Vector3> = None;
    for &(t, tag) in triangles {
        if !has(t, end) || tag == 1 || tag == 2 {
            continue;
        }
        let n = (position(t.1) - position(t.0))
            .cross(position(t.2) - position(t.0))
            .get_normal();
        match normal {
            None => normal = Some(n),
            Some(first) if first.dot(n) < 1.0 - COPLANAR_TOLERANCE => return None,
            Some(_) => {}
        }
    }
    let n = normal?;
    let slant = n.dot(into);
    if slant.abs() <= COPLANAR_TOLERANCE {
        return None;
    }
    Some((v, if slant > 0.0 { n } else { -n }))
}

/// Each open end's end face plane where that end carries no end ray (`end_rays`), faced toward the
/// chain `chain` (its vertices in order); `None` at a closed chain's, a rayed end's, an end with no
/// single flat end face, or where any chain vertex stands past the plane by more than `snap` (a
/// chain bending back behind its own end plane).
pub fn end_face_planes(
    position: &dyn Fn(i32) -> Vector3,
    triangles: &[TaggedTriangle],
    chain: &[i32],
    closed: bool,
    end_rays: (bool, bool),
    snap: f64,
) -> [Option<PointPlane>; 2] {
    if closed || chain.len() < 2 {
        return [None, None];
    }
    let last = chain.len() - 1;
    let planes = [
        (!end_rays.0)
            .then(|| end_face_plane(position, triangles, chain[0], chain[1]))
            .flatten(),
        (!end_rays.1)
            .then(|| end_face_plane(position, triangles, chain[last], chain[last - 1]))
            .flatten(),
    ];
    // A chain that bends back behind its own end plane has band past that plane round its own
    // points: the plane does not bound its band, so it gives none.
    planes.map(|plane| {
        plane.filter(|&(origin, toward_the_chain)| {
            chain
                .iter()
                .all(|&v| (position(v) - origin).dot(toward_the_chain) >= -snap)
        })
    })
}

/// The vertices of the chain whose edges are `edges`, in order.
pub fn chain_vertices(edges: &[(i32, i32)]) -> Vec<i32> {
    let mut vertices: Vec<i32> = edges.iter().map(|e| e.0).collect();
    if let Some(last) = edges.last() {
        vertices.push(last.1);
    }
    vertices
}

/// Whether every one of `points` stands past one of `planes` (away from the chain) by more than
/// `snap`: a set straddling or lying in a plane is not past it.
pub fn is_wholly_past(planes: &[Option<PointPlane>; 2], points: &[Vector3], snap: f64) -> bool {
    planes.iter().flatten().any(|&(origin, toward_the_chain)| {
        points
            .iter()
            .all(|&p| (p - origin).dot(toward_the_chain) < -snap)
    })
}

/// Takes every side triangle (tag 1 or 2) wholly past one of `planes` out of its side (tag 0),
/// never one with a chain edge (`chain_edges`); how many it took out. End regions (tags 3 and 4)
/// keep their tags: the cap is cut from its own field, which this does not change, so the room
/// keeps reading their edges (`foreign_edge_room_per_side`).
pub fn leave_what_lies_past(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    tags: &mut [i32],
    planes: &[Option<PointPlane>; 2],
    snap: f64,
) -> usize {
    if planes.iter().all(Option::is_none) {
        return 0;
    }
    let on_the_chain = |t: &(i32, i32, i32)| {
        [(t.0, t.1), (t.1, t.2), (t.2, t.0)].iter().any(|&(a, b)| {
            chain_edges
                .iter()
                .any(|&(c, d)| (a == c && b == d) || (a == d && b == c))
        })
    };
    let mut taken = 0;
    for (t, tag) in indices.iter().zip(tags.iter_mut()) {
        if (*tag != 1 && *tag != 2) || on_the_chain(t) {
            continue;
        }
        let corners = [
            positions[t.0 as usize],
            positions[t.1 as usize],
            positions[t.2 as usize],
        ];
        if is_wholly_past(planes, &corners, snap) {
            *tag = 0;
            taken += 1;
        }
    }
    taken
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A floor at z = 0 round the end vertex 0, the chain rising from it to vertex 1.
    fn floor() -> (Vec<Vector3>, Vec<TaggedTriangle>) {
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 3.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        ];
        let triangles = vec![((0, 2, 4), 0), ((0, 4, 3), 3), ((0, 1, 2), 1)];
        (positions, triangles)
    }

    #[test]
    fn an_unrayed_end_on_a_flat_face_reads_that_face_faced_toward_the_chain() {
        let (positions, triangles) = floor();
        let at = |v: i32| positions[v as usize];
        let planes = end_face_planes(&at, &triangles, &[0, 1], false, (false, false), 1e-6);
        let (origin, normal) = planes[0].expect("the floor is the first end's face");
        assert_eq!(origin, positions[0]);
        assert!((normal.z - 1.0).abs() < 1e-12, "{normal:?}");
        assert!(planes[1].is_none(), "the far end has no face round it");
        let rayed = end_face_planes(&at, &triangles, &[0, 1], false, (true, false), 1e-6);
        assert!(rayed[0].is_none(), "a rayed end keeps its ray's reading");
    }

    /// The chain leaves the floor's end vertex, rises and bends back down through the floor's
    /// plane (vertex 2 below it): its band runs past that plane round its own points, so the plane
    /// bounds nothing and the end gives none.
    #[test]
    fn a_chain_bending_back_behind_its_own_end_plane_gives_that_end_no_plane() {
        let (mut positions, triangles) = floor();
        positions.push(Vector3::new(-2.0, 0.0, -1.0));
        let below = positions.len() as i32 - 1;
        let at = |v: i32| positions[v as usize];
        let planes = end_face_planes(&at, &triangles, &[0, 1, below], false, (false, true), 1e-6);
        assert!(planes[0].is_none(), "{planes:?}");
        let straight = end_face_planes(&at, &triangles, &[0, 1], false, (false, true), 1e-6);
        assert!(
            straight[0].is_some(),
            "the same end on a straight chain reads the floor"
        );
    }

    /// A side triangle past the plane leaves its side; one holding a chain edge never does, nor
    /// an end region's.
    #[test]
    fn past_the_plane_only_side_triangles_off_the_chain_leave_their_side() {
        let positions = vec![
            Vector3::new(0.0, 0.0, -1.0),
            Vector3::new(1.0, 0.0, -1.0),
            Vector3::new(0.0, 1.0, -1.0),
            Vector3::new(1.0, 1.0, -2.0),
        ];
        let indices = [(0, 1, 2), (1, 3, 2), (0, 2, 3)];
        let mut tags = vec![1, 2, 3];
        let plane = [Some((Vector3::ZERO, Vector3::new(0.0, 0.0, 1.0))), None];
        let taken = leave_what_lies_past(&positions, &indices, &[(0, 1)], &mut tags, &plane, 1e-6);
        assert_eq!((taken, tags), (1, vec![1, 0, 3]));
    }

    #[test]
    fn only_what_lies_wholly_past_the_plane_by_more_than_the_snap_is_past() {
        let plane = [Some((Vector3::ZERO, Vector3::new(0.0, 0.0, 1.0))), None];
        let below = Vector3::new(0.0, 0.0, -1.0);
        let above = Vector3::new(0.0, 0.0, 1.0);
        assert!(is_wholly_past(&plane, &[below, below * 2.0], 1e-6));
        assert!(!is_wholly_past(&plane, &[below, above], 1e-6));
        assert!(!is_wholly_past(&plane, &[below, Vector3::ZERO], 1e-6));
    }
}
