//! Clipping a cutter whose run ended on a cap plane the sweep could not carry: one cap a boolean
//! owes ([`ClippedCap`]), one cutter with the caps to take off it ([`Candidate`]) and a cutter that
//! could not be clipped and was withdrawn ([`Withdrawal`]). The sweep with its cap-clip fallback
//! ([`try_create`]), the demotions it retries with ([`attempts_for`]), the half-space box a cap
//! plane is clipped by ([`half_space`]) and the clip itself ([`apply`]) land here.

use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::{cs_math, AxisAlignedBoundingBox, Matrix4X4, Vector3};

use super::bevel_bead_clip::CombineMeshes;
use super::bevel_chain_sweep::{self, BevelSweptSection};
use super::bevel_working_mesh;
use super::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelChainEnds, BevelEndCap};
use crate::topology::ChainEdge;

/// One cap the boolean owes - the plane to subtract, and the vertex the run ends at, which is what
/// the refusal has to name if the subtraction cannot be made.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClippedCap {
    /// The half space the tool must stay out of, in the cutter's own space.
    pub plane: BevelEndCap,
    /// Where the run ends, for the message.
    pub at: Vector3,
}

/// One cutter and the cap planes to take off it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Candidate {
    /// Where this cutter is in the builder's cutter list.
    pub cutter_index: i32,
    /// Which group to fail if the clip cannot be made.
    pub group_index: i32,
    /// The operation's bevel size, which the refusal quotes.
    pub size: f64,
    /// The caps to subtract. Never empty.
    pub clips: Vec<ClippedCap>,
}

/// A cutter that could not be clipped and has been taken out of the batch, with what to say on its
/// group's row.
#[derive(Debug, Clone)]
pub struct Withdrawal {
    /// The group whose cutter was withdrawn.
    pub group_index: i32,
    /// The refusal to put on that group's row.
    pub reason: BevelAdvisory,
}

impl ClippedCap {
    /// A cap owed on `plane`, at the run's end `at`.
    pub fn new(plane: BevelEndCap, at: Vector3) -> Self {
        ClippedCap { plane, at }
    }
}

impl Candidate {
    /// A cutter and the caps to take off it.
    pub fn new(cutter_index: i32, group_index: i32, size: f64, clips: Vec<ClippedCap>) -> Self {
        Candidate {
            cutter_index,
            group_index,
            size,
            clips,
        }
    }
}

/// The retries a run capped at either end is offered: each capped end in turn demoted to an
/// overshoot (and its cap owed as a clip at the run's first or last vertex), the other end's cap
/// and overshoot bound carried across unchanged. Exit planes are not carried: a demoted end
/// overshoots, so it has no squared exit to clip to.
pub fn attempts_for(
    ends: &BevelChainEnds,
    run: &[ChainEdge],
) -> Vec<(BevelChainEnds, Vec<ClippedCap>)> {
    let mut attempts = Vec::with_capacity(2);

    let carried =
        |start: BevelEndCap, end: BevelEndCap, start_max: f64, end_max: f64| BevelChainEnds {
            start_pair_corner: ends.start_pair_corner,
            end_pair_corner: ends.end_pair_corner,
            start_maximum_overshoot: start_max,
            end_maximum_overshoot: end_max,
            start_overshoot_corner: ends.start_overshoot_corner,
            end_overshoot_corner: ends.end_overshoot_corner,
            ..BevelChainEnds::new(start, end)
        };

    // The OTHER end's overshoot bound is carried across unchanged; the demoted end takes none -
    // it is meant to run past and be cut back by the plane.
    if ends.start.is_capped() {
        attempts.push((
            carried(
                BevelEndCap::default(),
                ends.end.clone(),
                f64::INFINITY,
                ends.end_maximum_overshoot,
            ),
            vec![ClippedCap::new(ends.start.clone(), run[0].from())],
        ));
    }

    if ends.end.is_capped() {
        attempts.push((
            carried(
                ends.start.clone(),
                BevelEndCap::default(),
                ends.start_maximum_overshoot,
                f64::INFINITY,
            ),
            vec![ClippedCap::new(ends.end.clone(), run[run.len() - 1].to())],
        ));
    }

    attempts
}

/// The run's tool, swept with its own `ends`; where that fails on an open run capped at either
/// end, swept again with each capped end in turn demoted to an overshoot ([`attempts_for`]),
/// returning the caps the boolean then owes as clips. The first refusal stands where every retry
/// fails too.
#[allow(clippy::too_many_arguments)]
pub fn try_create(
    run: &[ChainEdge],
    is_closed: bool,
    ends: Option<&BevelChainEnds>,
    size: f64,
    segments: i32,
    round_profile: bool,
    warnings: Option<&mut Vec<BevelAdvisory>>,
    back_depth: f64,
) -> Result<(Mesh, Vec<ClippedCap>), BevelAdvisory> {
    let sweep = |ends: Option<&BevelChainEnds>, warnings: Option<&mut Vec<BevelAdvisory>>| {
        bevel_chain_sweep::try_create(
            run,
            is_closed,
            ends,
            size,
            segments,
            round_profile,
            BevelSweptSection::Tool,
            0.0,
            0.0,
            warnings,
            back_depth,
            None,
        )
    };

    let failure = match sweep(ends, warnings) {
        Ok(tool) => return Ok((tool, Vec::new())),
        Err(failure) => failure,
    };

    let Some(ends) = ends else {
        return Err(failure);
    };
    if is_closed || run.is_empty() || !(ends.start.is_capped() || ends.end.is_capped()) {
        return Err(failure);
    }

    for (attempt_ends, clips) in attempts_for(ends, run) {
        if let Ok(retried) = sweep(Some(&attempt_ends), None) {
            return Ok((retried, clips));
        }
    }

    Err(failure)
}

/// How far the half-space box reaches from the plane, in diagonals of the cutter's own bounding
/// box. Two would already cover a cutter whose every point lies within one diagonal of the plane;
/// four is that with room to spare, and the box costs the kernel nothing for being large.
pub const SPAN_IN_DIAGONALS: f64 = 4.0;

impl Withdrawal {
    /// A withdrawn cutter's group and the reason it was withdrawn.
    pub fn new(group_index: i32, reason: BevelAdvisory) -> Self {
        Self {
            group_index,
            reason,
        }
    }
}

/// A box standing on the cap plane's far side - the half-space it removes - sized from `bounds`:
/// [`SPAN_IN_DIAGONALS`] of their diagonal on a side, its near face on the plane where the
/// bounds' centre drops onto it.
pub fn half_space(cap: &BevelEndCap, bounds: &AxisAlignedBoundingBox) -> Mesh {
    // A degenerate box would be an operand the kernel refuses rather than a clip that does
    // nothing, so the span has a floor under it however small the cutter turns out to be.
    let diagonal = cs_math::max_f64(bounds.size().length(), 1e-6);
    let span = SPAN_IN_DIAGONALS * diagonal;

    // The cutter's centre dropped onto the plane, which is where the box's near face is put.
    // Taking the plane's own closest point to the ORIGIN instead would sit the box wherever the
    // part happens to be modelled.
    let centre = bounds.center();
    let on_plane =
        centre - (cap.plane_normal * (cap.plane_normal.dot(centre) - cap.plane_distance));

    let mut cube = platonic_solids::create_cube(span, span, span);
    cube.transform(
        Matrix4X4::create_rotation_between(Vector3::new(0.0, 0.0, 1.0), cap.plane_normal)
            * Matrix4X4::create_translation_vector(on_plane + (cap.plane_normal * (span / 2.0))),
    );
    cube
}

/// Takes the clipped caps off the cutters they belong to, and withdraws any cutter whose clip the
/// kernel would not run. `cutters` is the builder's list, in the source item's own space: clipped
/// entries are replaced in place and withdrawn ones REMOVED, so this runs once every index written
/// while the tools were built has been read. Returns the groups whose cutter was withdrawn, for the
/// caller to fail on their own rows; a cancel of the boolean passes up as an error, any other
/// refusal withdraws the cutter.
pub fn apply(
    cutters: &mut Vec<Mesh>,
    candidates: &[Candidate],
    combine: &CombineMeshes<'_>,
) -> Result<Vec<Withdrawal>, BooleanError> {
    let mut withdrawals = Vec::new();
    // Marked rather than removed while the loop runs: it is still reading candidates by index, and
    // closing a gap now would renumber the ones it has not reached.
    let mut withdrawn = vec![false; cutters.len()];

    for candidate in candidates {
        let index = candidate.cutter_index as usize;
        let cutter = &cutters[index];
        let bounds = cutter.get_axis_aligned_bounding_box();
        let mut operands = Vec::with_capacity(candidate.clips.len() + 1);
        operands.push((cutter.clone(), Matrix4X4::IDENTITY));
        for clip in &candidate.clips {
            operands.push((half_space(&clip.plane, &bounds), Matrix4X4::IDENTITY));
        }

        // The kernel took some operands and not others, or refused one outright: either way the
        // plane this was asked to stop the tool on is not the plane it stopped it on, which is
        // treated exactly like a refusal rather than as a partial success worth keeping. Letting
        // the error out instead would fail the whole rebuild over a cap.
        let clipped = match combine(&operands, CsgModes::Subtract) {
            Ok(mesh) => Some(mesh),
            Err(BooleanError::Cancelled) => return Err(BooleanError::Cancelled),
            Err(_) => None,
        };

        if let Some(clipped) =
            clipped.filter(|mesh| bevel_working_mesh::is_usable_operand(Some(mesh), None))
        {
            cutters[index] = clipped;
            continue;
        }

        withdrawn[index] = true;
        withdrawals.push(Withdrawal::new(
            candidate.group_index,
            BevelAdvisory::new(
                BevelAdvisoryCode::CapClipCouldNotCut,
                vec![
                    BevelAdvisoryArg::Vector3(candidate.clips[0].at),
                    BevelAdvisoryArg::Double(candidate.size),
                ],
            ),
        ));
    }

    if !withdrawals.is_empty() {
        let mut index = 0;
        cutters.retain(|_| {
            let keep = !withdrawn[index];
            index += 1;
            keep
        });
    }

    Ok(withdrawals)
}
