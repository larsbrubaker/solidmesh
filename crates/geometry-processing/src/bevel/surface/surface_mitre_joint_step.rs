//! The STEP at a joint: the one triangle nobody lays where two consecutive cuts of a chain meet on
//! a side no sliver was sewn on - the near terminal of one cut's tangent run, the band corner the
//! neighbour's rim starts at, and the corner the two rims converge on at the far end.
//!
//! Ground no phase claims: the run belongs to one cut, the corner list to its neighbour, and the
//! strip stitch sews a column to a run rather than across a joint, so the little wedge between the
//! two rails at the joint is left standing open. Measured at the raked bore's hanger stand-off,
//! that is SIX of the seven holes the finished mesh still had - each a triangle whose short side is
//! 6.30 to 20.37 welds and whose two long sides run up to the shared corner at 24 to 1517
//! (`SurfaceRakedBoreSplitChordTests`, "The 22 unsewn rim segments").
//!
//! The gate is the HOLE'S OWN SIGNATURE rather than a length: all three sides already standing OPEN
//! BY ONE in the faces laid so far - net use +1 or -1 - with the three unmatched uses running one
//! way round. A step a few welds across and a long rim segment are told apart by nothing else, and
//! a length gate over a range as wide as 6 to 20 welds would be a guess; three sides open one way
//! round IS an open triangular hole, whatever its size. The triangle is laid wound OPPOSITE that
//! walk, which is what makes each of the three edges' matching use.
//!
//! The NET and not an exact count of one, which is what the finished mesh reads: at joints 42/43
//! and 43/44 the short step is used once forward and twice backward at mitre time - odd, so open,
//! but not once-used, because a later phase uses it twice more - and asking for exactly one closed
//! four of the six holes where the net closes all six.
//!
//! The directed-use map is only looked up, never iterated, so a `HashMap` cannot leak its order.

use std::collections::HashMap;

/// One chain edge's tangent run on each side as `(vertex, parameter)` pairs, indexed
/// `[edge][side]`, each run possibly absent.
pub type SideSeams = [Option<Vec<(i32, f64)>>; 2];

/// One chain edge's band corners, indexed
/// `[edge][side][end]` where end 0 is the arriving end and 1 the leaving one.
pub type SideBandCorners = [[Option<Vec<i32>>; 2]; 2];

type DirectedUses = HashMap<(i32, i32), i32>;

/// Lays the joint's step on each side of the joint between `edge_index` and `next_index` that
/// still has one open. Silent where there is none - which is every joint whose side a sliver's
/// patch and ear already closed - and where either table is absent.
pub fn sew(
    faces: &mut Vec<(i32, i32, i32)>,
    seams: Option<&[SideSeams]>,
    band_corners: Option<&[SideBandCorners]>,
    edge_index: usize,
    next_index: usize,
) {
    let (Some(seams), Some(band_corners)) = (seams, band_corners) else {
        return;
    };

    for side in 0..2 {
        // The walk carries the chain INTO this joint along edge_index and OUT of it along
        // next_index, so the run's terminal is the LAST vertex of the one and the FIRST of the
        // other, and the band corners are the leaving end's of the one and the arriving end's of
        // the other (the builder keys that last index 0 arriving, 1 leaving).
        let mut terminals = Vec::new();

        terminal(seams[edge_index][side].as_deref(), true, &mut terminals);
        terminal(seams[next_index][side].as_deref(), false, &mut terminals);

        let mut corners = Vec::new();

        add_corners(band_corners[edge_index][side][1].as_deref(), &mut corners);
        add_corners(band_corners[next_index][side][0].as_deref(), &mut corners);

        if terminals.is_empty() || corners.len() < 2 {
            continue;
        }

        let used = the_directed_uses(faces);
        let mut sewn = false;

        for &terminal in &terminals {
            if try_sew_one(faces, &used, terminal, &corners) {
                // One polygon per joint per side: with the hole closed the other terminal's own
                // triple no longer reads once-used all round anyway.
                sewn = true;
                break;
            }
        }

        if !sewn && terminals.len() == 2 {
            try_sew_the_quad(faces, &used, terminals[0], terminals[1], &corners);
        }
    }
}

fn try_sew_one(
    faces: &mut Vec<(i32, i32, i32)>,
    used: &DirectedUses,
    terminal: i32,
    corners: &[i32],
) -> bool {
    for first in 0..corners.len() {
        for second in first + 1..corners.len() {
            let near = corners[first];
            let far = corners[second];

            if near == far || near == terminal || far == terminal {
                continue;
            }

            let (Some(along_the_step), Some(along_theirs), Some(along_mine)) = (
                open_by_one(used, terminal, near),
                open_by_one(used, near, far),
                open_by_one(used, far, terminal),
            ) else {
                continue;
            };

            if along_the_step != along_theirs || along_theirs != along_mine {
                // Three once-used sides that do NOT run one way round bound no single hole - they
                // are edges of three different pieces that happen to share vertices.
                continue;
            }

            faces.push(if along_the_step {
                (terminal, far, near)
            } else {
                (terminal, near, far)
            });

            return true;
        }
    }

    false
}

/// The joint's step where it is a QUADRILATERAL rather than a triangle: BOTH runs' terminals
/// stand on the hole, with the two cuts' band corners between them.
///
/// Where a sliver's patch stood the joint's step is a triangle, because the patch owns one of the
/// two run terminals. Where the patch is DECLINED - both its crossings taken as their own rims, so
/// the whole of it stands on the crease and there is nothing to sew
/// (`surface_mitre_sliver_sew::try_sew`) - nobody owns either terminal and the hole is the four
/// sided one between them: at the hanger stand-off, joint 25/26 walks terminal, corner, corner,
/// terminal all one way round. The gate is the triangle's, one side longer, and the two triangles
/// are laid across the diagonal wound opposite that walk.
fn try_sew_the_quad(
    faces: &mut Vec<(i32, i32, i32)>,
    used: &DirectedUses,
    first: i32,
    second: i32,
    corners: &[i32],
) -> bool {
    for (from, to) in [(first, second), (second, first)] {
        for &near in corners {
            for &far in corners {
                if near == far || near == from || near == to || far == from || far == to {
                    continue;
                }

                let (Some(along_the_step), Some(along_theirs), Some(along_their_step)) = (
                    open_by_one(used, from, near),
                    open_by_one(used, near, far),
                    open_by_one(used, far, to),
                ) else {
                    continue;
                };

                if along_the_step != along_theirs || along_theirs != along_their_step {
                    continue;
                }

                // The fourth side runs between the two RUN TERMINALS, and at mitre time it is
                // ordinarily not drawn at all: the phases that sew across a joint's crease come
                // after the mitre, and at the hanger stand-off's joint 25/26 the three sides above
                // read once each while terminal to terminal reads nothing either way. So the gate
                // there is UNDRAWN as well as open by one - the quad may hand that later use its
                // partner, but it may not be the second face of a side two faces already own.
                let refused = match open_by_one(used, to, from) {
                    None => the_crease_is_drawn(used, to, from),
                    Some(along_the_crease) => along_the_crease != along_the_step,
                };

                if refused {
                    continue;
                }

                if along_the_step {
                    faces.push((from, to, far));
                    faces.push((from, far, near));
                } else {
                    faces.push((from, far, to));
                    faces.push((from, near, far));
                }

                return true;
            }
        }
    }

    false
}

/// Whether anything has drawn the side between two vertices, either way round.
fn the_crease_is_drawn(used: &DirectedUses, from: i32, to: i32) -> bool {
    used.contains_key(&(from, to)) || used.contains_key(&(to, from))
}

/// Every directed side of the faces laid so far, counted.
fn the_directed_uses(faces: &[(i32, i32, i32)]) -> DirectedUses {
    let mut used = DirectedUses::new();

    for &(v0, v1, v2) in faces {
        for side in [(v0, v1), (v1, v2), (v2, v0)] {
            *used.entry(side).or_insert(0) += 1;
        }
    }

    used
}

/// Whether the edge between two vertices stands OPEN by one use as the mesh now is, and if so
/// which way round that unmatched use walks it (`true` from `from` to `to`).
fn open_by_one(used: &DirectedUses, from: i32, to: i32) -> Option<bool> {
    let along = used.get(&(from, to)).copied().unwrap_or(0);
    let against = used.get(&(to, from)).copied().unwrap_or(0);

    let net = along - against;

    (net == 1 || net == -1).then_some(net > 0)
}

fn terminal(run: Option<&[(i32, f64)]>, last: bool, into: &mut Vec<i32>) {
    let Some(run) = run else {
        return;
    };

    let end = if last { run.last() } else { run.first() };

    if let Some(&(vertex, _)) = end {
        if !into.contains(&vertex) {
            into.push(vertex);
        }
    }
}

fn add_corners(of_the_end: Option<&[i32]>, into: &mut Vec<i32>) {
    for &corner in of_the_end.unwrap_or_default() {
        if !into.contains(&corner) {
            into.push(corner);
        }
    }
}
