//! Where each strip's rows end when two rounds overlap across the flat face between their edges:
//! on the other round.
//!
//! Two straight convex edges both bound one flat face (outward normal `n`); each is rounded at
//! radius `r` between that face and its own other face (outward normal `m`). A round tangent to
//! both faces has its axis `r` inside each, at `edge - r (n + m) / (1 + n.m)`; its face rail (the
//! axis's foot on the face) stands its setback in from the edge along the face. Both axes lie in
//! the plane `r` below the face, so the points `r` from both are on a plane normal to the face that
//! bisects the two axes - the plane halfway between them where the edges run parallel, the
//! bisector of their angle where they do not. That plane is the seam.
//!
//! A row is the round's arc across its edge at a station, from its far rail up to its face rail.
//! Its line on the face meets the seam at `q`, `sigma` in from the edge. Past its face rail
//! (`sigma` over the setback) the row is untouched: it ends on its face rail. Short of it, the row
//! ends where its arc crosses the seam, `q - h n` with `h = r - sqrt(r^2 - delta^2)`, `delta` the
//! setback less `sigma` - a point `r` from both axes. Within the seam epsilon of its face rail the
//! row SNAPS: it ends on its face rail, and that point is the seam's vertex there. A seam that
//! runs past the whole arc (`delta` beyond the arc's reach) is a true limit: one round swallows the
//! other's, which the outside-first rule decides, not this seam.
//!
//! Every row end on the seam of either strip is made ONCE: the ends are ordered along the seam and
//! two within the seam epsilon are one vertex, which both strips' rows index - so the fill that
//! fans each strip's last chords to the seam vertices between its rows shares them by
//! construction. On parallel edges every row of one strip ends where the other's does, at the
//! closed form `h = r - sqrt(r^2 - (D / 2)^2)`, `D` the setbacks' overlap. The level line's crest
//! fill (`gap_fill_crest`) builds the strips on these rows.

use solidmesh_math::Vector3;

/// One picked edge's strip: the edge and where its rows stand.
pub struct SeamStrip {
    /// The edge's first point.
    pub start: Vector3,
    /// The edge's last point.
    pub end: Vector3,
    /// The outward unit normal of the edge's face that is NOT the shared one.
    pub other_normal: Vector3,
    /// The rows' stations, as distances along the edge from `start`.
    pub row_stations: Vec<f64>,
}

/// The two strips the seam is asked of.
pub struct SeamRowsPair<'a> {
    /// The first strip (`k`).
    pub strip_k: &'a SeamStrip,
    /// The second strip (`j`).
    pub strip_j: &'a SeamStrip,
    /// The shared face's outward unit normal.
    pub face_normal: Vector3,
    /// The rounds' radius.
    pub radius: f64,
    /// Below this length two seam points, or a row end and its face rail, are one point.
    pub seam_epsilon: f64,
    /// The flatness and parallelism tolerance, as a length.
    pub tolerance: f64,
}

/// Where one row ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowEnd {
    /// On its face rail, as a row of a round alone does.
    FaceRail,
    /// On the seam vertex of this index.
    Seam(usize),
}

/// The seam and every row's end.
#[derive(Debug, Clone, PartialEq)]
pub struct SeamRows {
    /// The seam vertices, in order along the seam, each made once.
    pub vertices: Vec<Vector3>,
    /// Which seam vertices are face-rail points the rows snapped to (on the face).
    pub snapped: Vec<bool>,
    /// Strip `k`'s row ends, one per station.
    pub ends_k: Vec<RowEnd>,
    /// Strip `j`'s row ends, one per station.
    pub ends_j: Vec<RowEnd>,
}

/// Why two strips have no seam rows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeamRowsRefusal {
    /// An edge does not lie in the shared face's plane.
    NotOnTheFace,
    /// An edge's faces fold back onto each other: it has no round.
    NotRoundable,
    /// No row of either strip reaches the seam: the rounds do not overlap.
    BandsDoNotOverlap,
    /// The seam runs past a row's whole arc: one round swallows the other's.
    OverlapPastTheRounds,
}

/// One strip read for the seam: its edge line, its face rail's in-face direction and setback,
/// and how far its arc reaches across the face.
struct Round {
    start: Vector3,
    direction: Vector3,
    into_face: Vector3,
    setback: f64,
    reach: f64,
    length: f64,
}

fn round_of(strip: &SeamStrip, pair: &SeamRowsPair) -> Result<Round, SeamRowsRefusal> {
    let n = pair.face_normal;
    let r = pair.radius;
    let off = |p: Vector3| (p - strip.start).dot(n).abs() > pair.tolerance;
    if off(strip.end) {
        return Err(SeamRowsRefusal::NotOnTheFace);
    }
    let m = strip.other_normal;
    let c = n.dot(m);
    if 1.0 + c <= pair.tolerance {
        return Err(SeamRowsRefusal::NotRoundable);
    }
    // The axis offset from the edge, then its foot on the face.
    let to_axis = (n + m) * (-r / (1.0 + c));
    let to_foot = to_axis + n * r;
    let setback = to_foot.length();
    // The arc's farthest reach back across the face from its face rail: the whole radius once it
    // turns past a quarter, else as far as its far rail stands.
    let reach = if c <= 0.0 {
        r
    } else {
        r * (1.0 - c * c).sqrt()
    };
    Ok(Round {
        start: strip.start,
        direction: Vector3::normalized(strip.end - strip.start),
        length: (strip.end - strip.start).length(),
        into_face: to_foot * (1.0 / setback),
        setback,
        reach,
    })
}

/// The seam plane, as a point on it and its unit normal (in the face).
fn seam_plane(k: &Round, j: &Round, n: Vector3, tolerance: f64) -> (Vector3, Vector3) {
    let foot_k = k.start + k.into_face * k.setback;
    let foot_j = j.start + j.into_face * j.setback;
    let across = k.direction.cross(j.direction);
    if across.length() <= tolerance {
        // Parallel: halfway between the two face rails, facing from one edge to the other (the
        // rails themselves coincide at the face's exact width, so they cannot give the facing).
        let gap = j.start - k.start;
        let normal = Vector3::normalized(gap - k.direction * gap.dot(k.direction));
        return ((foot_k + foot_j) * 0.5, normal);
    }

    // The face rails' lines cross at x (both lie in the face): foot_k + a d_k = foot_j + b d_j.
    let w = foot_j - foot_k;
    let a = w.cross(j.direction).dot(across) / across.dot(across);
    let x = foot_k + k.direction * a;
    // Of the two bisectors, the one whose plane has the two face rails' middles on opposite
    // sides: the seam stands between the strips.
    let middle_k = foot_k + k.direction * (k.length / 2.0) - x;
    let middle_j = foot_j + j.direction * (j.length / 2.0) - x;
    let normals = [
        Vector3::normalized((k.direction + j.direction).cross(n)),
        Vector3::normalized((k.direction - j.direction).cross(n)),
    ];
    let normal = normals
        .into_iter()
        .find(|normal| normal.dot(middle_k) * normal.dot(middle_j) < 0.0)
        .unwrap_or(normals[0]);
    (x, normal)
}

/// Where a row of `round` at `station` meets the seam, as `sigma` in from the edge, or `None`
/// where the row runs along the seam.
fn sigma_at(round: &Round, station: f64, plane: (Vector3, Vector3)) -> Option<(Vector3, f64)> {
    let edge_point = round.start + round.direction * station;
    let toward = round.into_face.dot(plane.1);
    if toward == 0.0 {
        return None;
    }
    Some((edge_point, (plane.0 - edge_point).dot(plane.1) / toward))
}

/// The seam rows of `pair`, or why it has none.
pub fn seam_rows(pair: &SeamRowsPair) -> Result<SeamRows, SeamRowsRefusal> {
    let n = pair.face_normal;
    let r = pair.radius;
    let k = round_of(pair.strip_k, pair)?;
    let j = round_of(pair.strip_j, pair)?;
    if (pair.strip_j.start - pair.strip_k.start).dot(n).abs() > pair.tolerance {
        return Err(SeamRowsRefusal::NotOnTheFace);
    }
    let plane = seam_plane(&k, &j, n, pair.tolerance);
    let along_seam = n.cross(plane.1);

    // Every row end on the seam, as (place along the seam, strip, row, point, snapped).
    let mut on_seam: Vec<(f64, usize, usize, Vector3, bool)> = Vec::new();
    for (strip, round, stations) in [
        (0, &k, &pair.strip_k.row_stations),
        (1, &j, &pair.strip_j.row_stations),
    ] {
        for (row, &station) in stations.iter().enumerate() {
            let Some((edge_point, sigma)) = sigma_at(round, station, plane) else {
                continue;
            };
            let delta = round.setback - sigma;
            if delta < -pair.seam_epsilon {
                continue;
            }
            let (point, snapped) = if delta <= pair.seam_epsilon {
                (edge_point + round.into_face * round.setback, true)
            } else {
                if delta >= round.reach {
                    return Err(SeamRowsRefusal::OverlapPastTheRounds);
                }
                let depth = r - (r * r - delta * delta).sqrt();
                (edge_point + round.into_face * sigma - n * depth, false)
            };
            on_seam.push((
                (point - plane.0).dot(along_seam),
                strip,
                row,
                point,
                snapped,
            ));
        }
    }
    if on_seam.is_empty() {
        return Err(SeamRowsRefusal::BandsDoNotOverlap);
    }

    // Ordered along the seam, ties by strip then row so the order never depends on the sort.
    on_seam.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let mut rows = SeamRows {
        vertices: Vec::new(),
        snapped: Vec::new(),
        ends_k: vec![RowEnd::FaceRail; pair.strip_k.row_stations.len()],
        ends_j: vec![RowEnd::FaceRail; pair.strip_j.row_stations.len()],
    };
    for (_, strip, row, point, snapped) in on_seam {
        let same = rows
            .vertices
            .last()
            .is_some_and(|&last| (point - last).length() <= pair.seam_epsilon);
        if !same {
            rows.vertices.push(point);
            rows.snapped.push(snapped);
        }
        let vertex = rows.vertices.len() - 1;
        let ends = if strip == 0 {
            &mut rows.ends_k
        } else {
            &mut rows.ends_j
        };
        ends[row] = RowEnd::Seam(vertex);
    }
    Ok(rows)
}
