//! The crest fill: two strips whose rounds overlap across the face between their edges, each
//! built from its far rail up its round to the seam where it meets the other round
//! ([`super::crest_seam_rows`]), the seam's vertices shared by both strips, and each end face's
//! hole closed flat through the seam's end vertex.
//!
//! It reads the gap the band build leaves when handed a room past the size
//! ([`super::band_gap_crest`]): each chain's far rail is a loop owned by that chain alone, and each
//! end face's cut is a loop owned by both, running from one far rail's end to the other's. A gap
//! of any other shape is not this fill's (`None`), and the caller keeps its refusal.
//!
//! A row is the round's arc at a far-rail vertex, in the plane across its edge: centre `F - r m`
//! (`F` the far-rail vertex, `m` the edge's other face's outward normal), the full arc's even
//! split into the fillet's segments kept below the row's seam end, and the last chord ending on
//! that seam vertex. A chord point within the seam epsilon of the seam vertex is that vertex, so no
//! chord shorter than the epsilon is made. Between two rows of a strip the rows are zipped, and
//! the last chord's band fans to every seam vertex between the two rows' ends - the other strip's
//! included - which is what makes the seam shared by construction. Where the two bands' cuts across
//! an end face just touch (the face's exact width), the sliver of end face they leave ringed by a
//! closed loop is removed, the end's flat fill laying it again, and the cut's own vertex on the
//! input edge joining the two chains' ends there - found by index, never by distance - is the seam
//! vertex there, the end's hole closed as two, one each side of it.
//! Each triangle faces away from its round's axis; each end hole faces past its chains' ends, and
//! is read flat to the tolerance the pair's edges were read straight to.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::crest_seam_rows::{seam_rows, RowEnd, SeamRowsPair, SeamStrip};
use super::flat_hole_fill;
use super::gap_fill::FILL_TAG;
use super::{GapFillRefusal, GapFillResult, GapLoop, LevelLineMesh, LevelLineMeshError};

/// One picked straight edge as the crest fill reads it.
pub struct CrestEdge {
    /// The edge's first point.
    pub start: Vector3,
    /// The edge's last point.
    pub end: Vector3,
    /// The outward unit normal of its face that is not the shared one.
    pub other_normal: Vector3,
}

/// What the fill is handed.
pub struct CrestFillInput<'a> {
    /// The cut mesh, the band removed.
    pub mesh: &'a Rc<RefCell<LevelLineMesh>>,
    /// The gap's loops.
    pub loops: &'a [GapLoop],
    /// The two edges, chain 0 then chain 1.
    pub edges: [&'a CrestEdge; 2],
    /// The shared face's outward unit normal.
    pub face_normal: Vector3,
    /// The rounds' radius.
    pub radius: f64,
    /// The fillet's segments over a whole arc.
    pub segments: i32,
    /// Below this length two points are one.
    pub seam_epsilon: f64,
    /// The kernel's snap.
    pub snap: f64,
    /// The length within which the edges were read straight and on the shared face: the seam
    /// reads them, and each end hole is read flat, to the same tolerance, so a part stored in
    /// single precision (its faces flat only to the stored bits) is read as it was picked.
    pub tolerance: f64,
    /// Each chain's first and last input vertex, chain 0 then chain 1.
    pub chain_ends: [[i32; 2]; 2],
}

/// One built row: its vertex indices from the far rail to its seam vertex, and its arc centre.
struct Row {
    vertices: Vec<i32>,
    seam: usize,
    centre: Vector3,
}

fn facing_away(
    mesh: &mut LevelLineMesh,
    corners: [i32; 3],
    away_from: impl Fn(Vector3) -> Vector3,
    added: &mut Vec<i32>,
) -> Result<(), LevelLineMeshError> {
    let [a, b, c] = corners;
    let (pa, pb, pc) = (mesh.position(a), mesh.position(b), mesh.position(c));
    let normal = (pb - pa).cross(pc - pa);
    let centroid = (pa + pb + pc) * (1.0 / 3.0);
    let triangle = if normal.dot(away_from(centroid)) < 0.0 {
        mesh.add_face(a, c, b, FILL_TAG)?
    } else {
        mesh.add_face(a, b, c, FILL_TAG)?
    };
    added.push(triangle);
    Ok(())
}

/// Removes the kept piece a closed loop owned by both chains rings: where the two bands' cuts
/// across an end face just touch, they leave a sliver of that face standing alone inside the hole
/// the end is closed with, and the hole's flat fill lays that face again. False, removing nothing,
/// where the kept side of the loop reaches any open loop - then it is no island.
fn remove_the_island(
    mesh: &mut LevelLineMesh,
    island: &GapLoop,
    open_vertices: &HashSet<i32>,
) -> bool {
    if island.owners != [0, 1] {
        return false;
    }
    let count = island.vertices.len();
    // The ring's edges in the loop's own order, so the piece is gathered in an order that never
    // depends on hashing; the set only answers membership.
    let ring_edges: Vec<(i32, i32)> = (0..count)
        .map(|i| {
            let (a, b) = (island.vertices[i], island.vertices[(i + 1) % count]);
            (a.min(b), a.max(b))
        })
        .collect();
    let ring: HashSet<(i32, i32)> = ring_edges.iter().copied().collect();
    let mut piece: Vec<i32> = Vec::new();
    let mut seen: HashSet<i32> = HashSet::new();
    for &(a, b) in &ring_edges {
        for &t in mesh.triangles_on(a, b) {
            if !mesh.is_removed(t) && seen.insert(t) {
                piece.push(t);
            }
        }
    }
    let mut next = 0;
    while next < piece.len() {
        let t = mesh.triangles()[piece[next] as usize];
        next += 1;
        for (a, b) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            if open_vertices.contains(&a) {
                return false;
            }
            if ring.contains(&(a.min(b), a.max(b))) {
                continue;
            }
            for &u in mesh.triangles_on(a, b) {
                if !mesh.is_removed(u) && seen.insert(u) {
                    piece.push(u);
                }
            }
        }
    }
    for t in piece {
        mesh.remove_triangle(t);
    }
    true
}

/// The crest fill of `input`, or `None` where the gap is not this fill's shape.
pub fn fill(input: &CrestFillInput) -> Result<Option<GapFillResult>, LevelLineMeshError> {
    let r = input.radius;
    let n = input.face_normal;
    let epsilon = input.seam_epsilon.max(input.snap);

    // Each chain's far rail: its one loop owned by it alone; the end cuts: the loops owned by both.
    let mut rails: [Option<&GapLoop>; 2] = [None, None];
    let mut ends: Vec<&GapLoop> = Vec::new();
    let mut islands: Vec<&GapLoop> = Vec::new();
    for gap_loop in input.loops {
        if gap_loop.is_closed {
            islands.push(gap_loop);
            continue;
        }
        // An open loop runs between two ends; one of fewer vertices is no rail and no cut.
        if gap_loop.vertices.len() < 2 {
            return Ok(None);
        }
        // `owners` is ascending, so a loop owned by both chains reads `[0, 1]`.
        match gap_loop.owners.as_slice() {
            [only] if (0..2).contains(only) && !gap_loop.has_mixed_owners => {
                if rails[*only as usize].replace(gap_loop).is_some() {
                    return Ok(None);
                }
            }
            [0, 1] => ends.push(gap_loop),
            _ => return Ok(None),
        }
    }
    let (Some(rail_0), Some(rail_1)) = (rails[0], rails[1]) else {
        return Ok(None);
    };
    if ends.len() != 2 {
        return Ok(None);
    }

    let mut mesh = input.mesh.borrow_mut();
    let open_vertices: HashSet<i32> = input
        .loops
        .iter()
        .filter(|gap_loop| !gap_loop.is_closed)
        .flat_map(|gap_loop| gap_loop.vertices.iter().copied())
        .collect();
    for island in islands {
        if !remove_the_island(&mut mesh, island, &open_vertices) {
            return Ok(None);
        }
    }
    let direction = |edge: &CrestEdge| Vector3::normalized(edge.end - edge.start);
    let station = |edge: &CrestEdge, p: Vector3| (p - edge.start).dot(direction(edge));
    let strips: Vec<SeamStrip> = [rail_0, rail_1]
        .iter()
        .zip(input.edges)
        .map(|(rail, edge)| SeamStrip {
            start: edge.start,
            end: edge.end,
            other_normal: edge.other_normal,
            row_stations: rail
                .vertices
                .iter()
                .map(|&v| station(edge, mesh.position(v)))
                .collect(),
        })
        .collect();
    let Ok(seam) = seam_rows(&SeamRowsPair {
        strip_k: &strips[0],
        strip_j: &strips[1],
        face_normal: n,
        radius: r,
        seam_epsilon: epsilon,
        tolerance: input.tolerance,
    }) else {
        return Ok(None);
    };

    // At the face's exact width the band's cut across an end face runs up to the point where the
    // two rounds meet, on the shared face's edge between the two chains' ends; that cut vertex IS
    // the seam's vertex there, read by index - the cut's vertex standing on the input edge joining
    // the two chain ends at that end - so the end's hole meets itself on one vertex.
    let mut on_a_cut: Vec<Option<i32>> = vec![None; seam.vertices.len()];
    for end in &ends {
        // Which chain's rail, which end of it, and that end's seam vertex and chain vertex.
        let end_at = |v: i32| {
            (0..2).find_map(|strip| {
                let rail = [rail_0, rail_1][strip];
                let ends_of = [&seam.ends_k, &seam.ends_j][strip];
                let row = if rail.vertices[0] == v {
                    0
                } else if rail.vertices[rail.vertices.len() - 1] == v {
                    rail.vertices.len() - 1
                } else {
                    return None;
                };
                let edge = input.edges[strip];
                let at_start = station(edge, mesh.position(v)) < station(edge, edge.end) / 2.0;
                let chain_vertex = input.chain_ends[strip][usize::from(!at_start)];
                match ends_of[row] {
                    RowEnd::Seam(index) => Some((index, chain_vertex)),
                    RowEnd::FaceRail => None,
                }
            })
        };
        let first = end.vertices[0];
        let last = end.vertices[end.vertices.len() - 1];
        let (Some((seam_a, chain_a)), Some((seam_b, chain_b))) = (end_at(first), end_at(last))
        else {
            return Ok(None);
        };
        if seam_a != seam_b || !seam.snapped[seam_a] {
            continue;
        }
        let between = LevelLineMesh::key(chain_a, chain_b);
        let inner = &end.vertices[1..end.vertices.len() - 1];
        match inner.iter().copied().find(|&v| {
            let (a, b) = mesh.input_edge_behind(v);
            LevelLineMesh::key(a, b) == between
        }) {
            Some(cut) => on_a_cut[seam_a] = Some(cut),
            None => return Ok(None),
        }
    }
    let mut added_vertices: Vec<i32> = Vec::new();
    let seam_vertices: Vec<i32> = seam
        .vertices
        .iter()
        .zip(&on_a_cut)
        .map(|(&p, cut)| {
            cut.unwrap_or_else(|| {
                let v = mesh.add_vertex(p);
                added_vertices.push(v);
                v
            })
        })
        .collect();
    let mut added_triangles: Vec<i32> = Vec::new();

    // The rows, strip by strip, in far-rail order.
    let mut rows: [Vec<Row>; 2] = [Vec::new(), Vec::new()];
    for (strip, (rail, ends_of)) in [(rail_0, &seam.ends_k), (rail_1, &seam.ends_j)]
        .into_iter()
        .enumerate()
    {
        let m = input.edges[strip].other_normal;
        let w = Vector3::normalized(n - m * m.dot(n));
        let turn = libm::acos(m.dot(n).clamp(-1.0, 1.0));
        for (&far, end) in rail.vertices.iter().zip(ends_of.iter()) {
            let RowEnd::Seam(seam_index) = *end else {
                return Ok(None);
            };
            let far_point = mesh.position(far);
            let centre = far_point - m * r;
            let to_seam = seam.vertices[seam_index] - centre;
            let end_angle = libm::atan2(to_seam.dot(w), to_seam.dot(m));
            let mut vertices = vec![far];
            for i in 1..input.segments {
                let angle = turn * f64::from(i) / f64::from(input.segments);
                if angle >= end_angle {
                    break;
                }
                let point = centre + m * (r * libm::cos(angle)) + w * (r * libm::sin(angle));
                if (seam.vertices[seam_index] - point).length() <= epsilon {
                    break;
                }
                let v = mesh.add_vertex(point);
                added_vertices.push(v);
                vertices.push(v);
            }
            vertices.push(seam_vertices[seam_index]);
            rows[strip].push(Row {
                vertices,
                seam: seam_index,
                centre,
            });
        }
    }

    // Between consecutive rows: zip below the last chords, then fan the last band over the seam.
    for (strip, strip_rows) in rows.iter().enumerate() {
        let d = direction(input.edges[strip]);
        for pair in strip_rows.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let axis_point = a.centre;
            let away = |p: Vector3| {
                let offset = p - axis_point;
                offset - d * offset.dot(d)
            };
            let below_b = &b.vertices[..b.vertices.len() - 1];
            let (mut i, mut j) = (0usize, 0usize);
            let (last_a, last_b) = (a.vertices.len() - 1, below_b.len() - 1);
            while i < last_a || j < last_b {
                let advance_a = j == last_b
                    || (i < last_a
                        && (i + 1) as f64 / last_a as f64 <= (j + 1) as f64 / last_b.max(1) as f64);
                let corners = if advance_a {
                    i += 1;
                    [a.vertices[i - 1], below_b[j], a.vertices[i]]
                } else {
                    j += 1;
                    [a.vertices[i], below_b[j - 1], below_b[j]]
                };
                facing_away(&mut mesh, corners, away, &mut added_triangles)?;
            }
            let apex = below_b[last_b];
            let path: Vec<usize> = if a.seam <= b.seam {
                (a.seam..=b.seam).collect()
            } else {
                (b.seam..=a.seam).rev().collect()
            };
            for step in path.windows(2) {
                let corners = [apex, seam_vertices[step[0]], seam_vertices[step[1]]];
                facing_away(&mut mesh, corners, away, &mut added_triangles)?;
            }
        }
    }

    // Each end cut, closed through the two strips' end rows and their shared seam vertex.
    for end in ends {
        let (first, last) = (end.vertices[0], end.vertices[end.vertices.len() - 1]);
        let row_from = |v: i32| {
            rows.iter()
                .enumerate()
                .flat_map(|(strip, rs)| rs.iter().map(move |row| (strip, row)))
                .find(|(_, row)| row.vertices[0] == v)
        };
        let (Some((strip_up, up)), Some((_, down))) = (row_from(last), row_from(first)) else {
            return Ok(None);
        };
        if up.seam != down.seam {
            return Ok(None);
        }
        // The hole: the cut, up the one row and down the other. Where the cut reaches the seam
        // vertex itself, the hole is two, one each side of that vertex.
        let crest = seam_vertices[up.seam];
        let mut holes: Vec<Vec<i32>> = Vec::new();
        match end.vertices.iter().position(|&v| v == crest) {
            Some(pinch) => {
                let mut below: Vec<i32> = end.vertices[..=pinch].to_vec();
                below.extend(down.vertices[1..down.vertices.len() - 1].iter().rev());
                let mut beyond: Vec<i32> = end.vertices[pinch..].to_vec();
                beyond.extend(&up.vertices[1..up.vertices.len() - 1]);
                holes.push(below);
                holes.push(beyond);
            }
            None => {
                let mut hole: Vec<i32> = end.vertices.clone();
                hole.extend(&up.vertices[1..]);
                hole.extend(down.vertices[1..down.vertices.len() - 1].iter().rev());
                holes.push(hole);
            }
        }
        // Each hole faces past the end of the chain whose row climbs it.
        let edge = input.edges[strip_up];
        let d = direction(edge);
        let past = if station(edge, mesh.position(end.vertices[0])) > station(edge, edge.end) / 2.0
        {
            d
        } else {
            d * -1.0
        };
        for hole in holes {
            let points: Vec<Vector3> = hole.iter().map(|&v| mesh.position(v)).collect();
            let Some((triangles, normal)) = flat_hole_fill::fill(&points, input.tolerance, None)
            else {
                return Ok(None);
            };
            let flip = normal.dot(past) < 0.0;
            for t in triangles {
                let (a, b, c) = (hole[t.a as usize], hole[t.b as usize], hole[t.c as usize]);
                let triangle = if flip {
                    mesh.add_face(a, c, b, FILL_TAG)?
                } else {
                    mesh.add_face(a, b, c, FILL_TAG)?
                };
                added_triangles.push(triangle);
            }
        }
    }

    drop(mesh);
    Ok(Some(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: Some(Rc::clone(input.mesh)),
        added_triangles,
        added_vertices,
        ..GapFillResult::default()
    }))
}
