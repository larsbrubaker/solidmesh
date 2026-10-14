//! The END-FACE cap of the corner cap: the hole between a strip's last row and the end run the cut left
//! on the end face, both running from S to T, zipped by ANGLE about the chain's end vertex V
//! ([`zip_end`]), with its row/run watcher ([`WATCH_END_HOLE`]); the mesh-and-V watcher is
//! [`super::corner_cap::WATCH_END_HOLE_AT`].

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::corner_cap::{fans_about_the_end, zips_about_the_rows_axis, WATCH_END_HOLE_AT};
use super::corner_cap_flat_ends::{
    closes_flat, fans_from_the_end_vertex_on_the_run, use_flat_ends,
};
use super::corner_cap_hole_fill_end::end_hole_by_the_one_filler;
use super::{corner_fill, GapFillRefusal, LevelLineMesh, LevelLineMeshError, LevelLineWork};

/// A watcher of the end holes: the (row, run) pair [`zip_end`] receives.
pub type WatchEndHole = Rc<dyn Fn(&[i32], &[i32])>;

thread_local! {
    /// A read-only observer of the end holes, handed
    /// the (row, run) pair [`zip_end`] receives, before it reads them. Per thread, so a measuring
    /// test never sees another test's fills. `None` by default.
    pub static WATCH_END_HOLE: RefCell<Option<WatchEndHole>> = const { RefCell::new(None) };
}

/// How many snaps of placement an angle tie in [`zip_end`]
/// allows for (see `placed_tie` there).
const PLACEMENT_TIE_SNAPS: f64 = 8.0;

/// The END-FACE cap - the hole between a strip's last row and the end run
/// the cut left on the end face, both running from S to T, zipped by ANGLE about the chain's end
/// vertex V. No vertex is added, so the end face's plane is kept: the angles are measured in the
/// plane of V, S and T, which is planar, since the rails end on that face's edges and V is its
/// corner. The hole does not surround V - it lies between two curves from S to T - which is why
/// the corner cap's full-turn test cannot take it. `row` is traversed in the direction the cap
/// must lay it and `run` in the direction the KEPT surface walks it; either not rising in angle
/// from S to T refuses [`GapFillRefusal::CornerCapNotStarShaped`] - unless the run turns the other
/// way round V to close the full turn, which [`fans_about_the_end`] takes. Answers the refusal and
/// the offending vertex (-1 on success). `flowed` sends a refused hole of a
/// flowed band to [`end_hole_by_the_one_filler`].
#[allow(clippy::too_many_arguments)]
pub fn zip_end(
    mesh: &mut LevelLineMesh,
    row: &[i32],
    run: &[i32],
    v: Vector3,
    added: &mut Vec<i32>,
    added_vertices: &mut Vec<i32>,
    flowed: bool,
) -> Result<(GapFillRefusal, i32), LevelLineMeshError> {
    let watch = WATCH_END_HOLE.with(|w| w.borrow().clone());
    if let Some(watch) = watch {
        watch(row, run);
    }
    let watch_at = WATCH_END_HOLE_AT.with(|w| w.borrow().clone());
    if let Some(watch_at) = watch_at {
        watch_at(mesh, row, run, v);
    }
    let mut offending = -1;

    let ends_shared = row[0] == run[0] && row[row.len() - 1] == run[run.len() - 1];

    // flat first: a planar hole needs no angle about V, so a row starting AT V (UpperLeftInner) is closed too
    if use_flat_ends() && ends_shared && closes_flat(mesh, row, run, added)? {
        return Ok((GapFillRefusal::None, offending));
    }

    if ends_shared && fans_from_the_end_vertex_on_the_run(mesh, row, run, v, added)? {
        return Ok((GapFillRefusal::None, offending));
    }

    let s = mesh.position(row[0]) - v;
    let t = mesh.position(row[row.len() - 1]) - v;
    let n = s.cross(t);
    if !ends_shared || n.length_squared() == 0.0 {
        offending = row[0];
        return Ok((GapFillRefusal::CornerCapNotStarShaped, offending));
    }

    let u = s.get_normal();
    let w = n.get_normal().cross(u);
    let angle_of = |mesh: &LevelLineMesh, vertex: i32| {
        let d = mesh.position(vertex) - v;
        d.dot(w).atan2(d.dot(u))
    };

    let a: Vec<f64> = row.iter().map(|&i| angle_of(mesh, i)).collect();
    let mut b: Vec<f64> = run.iter().map(|&i| angle_of(mesh, i)).collect();

    // The run may pass THROUGH V: a concave crease cut on a face square to it (a boss's foot on the
    // block's bottom) leaves the end face's kept edge running S -> V -> T along the two faces the crease
    // joins. V has no angle about itself, and every row vertex past the run's last angle before it sees V
    // across the hole, so V takes that angle: the zip steps onto V at once and fans the rest of the row
    // to it, and the run's last step V -> T is the row's own last triangle.
    let mut i = 1;
    while i + 1 < run.len() {
        if corner_fill::same(mesh.position(run[i]), v) {
            b[i] = b[i - 1];
        }
        i += 1;
    }

    let tie = 1e-12;
    for is_run in [false, true] {
        let (angles, vertices) = if is_run { (&b, run) } else { (&a, row) };
        let mut i = 0;
        while i + 1 < angles.len() {
            if angles[i + 1] < angles[i] - tie {
                if is_run
                    && fans_about_the_end(
                        mesh,
                        row,
                        run,
                        &a,
                        &b,
                        v,
                        n.get_normal(),
                        added,
                        added_vertices,
                    )?
                {
                    return Ok((GapFillRefusal::None, offending));
                }

                if is_run && zips_about_the_rows_axis(mesh, row, run, added)? {
                    return Ok((GapFillRefusal::None, offending));
                }

                // A flowed band's end hole goes to the one filler. Elsewhere the refusal still stands: it
                // sends an oblique end on to the construction that matches its control
                // (ObliqueEndFaceOwnerTests.BottomCenterEdgeRefusesByNameAtItsApexEnd), until the one
                // filler is judged against those ends too.
                offending = vertices[i + 1];
                let refusal = if flowed {
                    end_hole_by_the_one_filler(
                        mesh,
                        row,
                        run,
                        v,
                        added,
                        added_vertices,
                        &mut offending,
                    )?
                } else {
                    GapFillRefusal::CornerCapNotStarShaped
                };
                return Ok((refusal, offending));
            }
            i += 1;
        }
    }

    // Two vertices at the SAME angle about V (a fillet's middle arc point and the end run's vertex on the
    // ridge line, both on the symmetry plane) are a tie the arithmetic cannot break: the run's vertex is a
    // root placed only to the snap, so its angle wobbles by snap / distance with the part's placement, and
    // a tight tie laid the other diagonal after a translation. A tie is therefore read to the placement
    // accuracy - a few snaps over the nearer vertex's distance from V - and broken by rule: the row first.
    let placed_tie = |mesh: &LevelLineMesh, row_vertex: i32, run_vertex: i32| {
        let nearer = cs_math::min_f64(
            (mesh.position(row_vertex) - v).length(),
            (mesh.position(run_vertex) - v).length(),
        );
        if nearer > 0.0 {
            cs_math::max_f64(tie, PLACEMENT_TIE_SNAPS * mesh.same_place() / nearer)
        } else {
            tie
        }
    };

    let (mut x, mut y) = (0, 0);
    while x < row.len() - 1 || y < run.len() - 1 {
        LevelLineWork::charge(1)?;
        let advances_row = y >= run.len() - 1
            || (x < row.len() - 1
                && a[x + 1] <= b[y + 1] + placed_tie(mesh, row[x + 1], run[y + 1]));
        if advances_row {
            corner_fill::lay(mesh, row[x], row[x + 1], run[y], added)?;
            x += 1;
        } else {
            corner_fill::lay(mesh, run[y + 1], run[y], row[x], added)?;
            y += 1;
        }
    }

    Ok((GapFillRefusal::None, offending))
}
