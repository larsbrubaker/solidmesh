//! Redrawing a run's SPLIT stations - vertices on the straight chain between two unsplit
//! neighbours - from those neighbours rather than from their own fans
//! (`Bevel/Sections/SectionSplitBlend.cs`, a C# `internal static class`, so a module here): what
//! the blend did at one station ([`SplitPlacement`]) and whether it split it, the switch that turns
//! absorbing split vertices on and off, and the blend's small readings (the nearest unsplit
//! neighbour, a point or number part way between two). And the blend itself
//! ([`apply`], [`try_blend`]) with its reading of a split vertex ([`reads_as_a_split_vertex`]).

use std::sync::atomic::{AtomicBool, Ordering};

use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::{BevelSection, NanSectionPoint, SectionKind, SectionPool, SectionSide};
use crate::bevel::bevel_split_chords;

/// C# `SectionSplitBlend.SplitPlacement`: what the blend did at one station, kept so the passes
/// AFTER it do not have to read the split a second time. A later pass that treats a split vertex
/// differently - the surface walk, which may not walk one from its own station - asks this rather
/// than re-deriving the reading. A station is split exactly where both neighbours are set.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitPlacement {
    /// The nearest unsplit station before this one, or -1 where this station is not split.
    pub before: i32,
    /// The nearest unsplit station after this one, or -1 where this station is not split.
    pub after: i32,
    /// Where along the chord from `before`'s station to `after`'s this station stands - the very
    /// parameter the blend drew the whole section at.
    pub parameter: f64,
}

impl SplitPlacement {
    /// C# `IsSplit`: whether the blend redrew this station as a split vertex.
    pub fn is_split(&self) -> bool {
        self.before >= 0 && self.after >= 0
    }
}

/// C# `AbsorbSplitVertices` (a static property, initializer `true`): whether a run's split vertices
/// are absorbed into their neighbours' strips. A process-wide switch, as C#'s static is, kept
/// thread-safe because the kernel runs on worker threads (bevel README, "Decided").
static ABSORB_SPLIT_VERTICES: AtomicBool = AtomicBool::new(true);

/// Reads [`ABSORB_SPLIT_VERTICES`].
pub fn absorb_split_vertices() -> bool {
    ABSORB_SPLIT_VERTICES.load(Ordering::Relaxed)
}

/// Sets [`ABSORB_SPLIT_VERTICES`].
pub fn set_absorb_split_vertices(absorb: bool) {
    ABSORB_SPLIT_VERTICES.store(absorb, Ordering::Relaxed);
}

impl SplitPlacement {
    /// C#'s `SplitPlacement` constructor.
    pub fn new(before: i32, after: i32, parameter: f64) -> Self {
        Self {
            before,
            after,
            parameter,
        }
    }
}

/// C# `NearestThatIsNotSplit`: the nearest station `step` (±1) round the run from `from` that is not
/// split, or -1 where every other station is.
pub fn nearest_that_is_not_split(is_split: &[bool], from: i32, step: i32) -> i32 {
    let count = is_split.len() as i32;
    let mut at = from;

    for _ in 0..count {
        at = ((at + step) % count + count) % count;

        if at == from {
            return -1;
        }

        if !is_split[at as usize] {
            return at;
        }
    }

    -1
}

/// C# `HowFarAlong`: how far along the chain's `tangent` the `wanted` point stands from the `drawn`
/// one.
pub fn how_far_along(wanted: Vector3, drawn: Vector3, tangent: Vector3) -> f64 {
    (wanted - drawn).dot(tangent)
}

/// C# `Between` (points): the point `parameter` of the way from `from` to `to`.
pub fn between(from: Vector3, to: Vector3, parameter: f64) -> Vector3 {
    from + ((to - from) * parameter)
}

/// C# `Between` (numbers): the number `parameter` of the way from `from` to `to`.
pub fn between_values(from: f64, to: f64, parameter: f64) -> f64 {
    from + ((to - from) * parameter)
}

/// C# `Apply`: rewrites in place every section that reads as a split vertex, blended from its
/// nearest unsplit neighbours either side, and says what it did at each station (C#'s `placed`;
/// `SplitPlacement::new(-1, -1, 0.0)` where it did nothing). A run of under three stations, or a mesh
/// with no lattice, is left alone. `Err` where a blended point is not a number (where C# throws).
pub fn apply(
    sections: &mut [BevelSection],
    mesh: &Mesh,
    pool: &mut SectionPool,
) -> Result<Vec<SplitPlacement>, NanSectionPoint> {
    let mut placed = vec![SplitPlacement::new(-1, -1, 0.0); sections.len()];

    if sections.len() < 3 {
        return Ok(placed);
    }

    let lattice = bevel_split_chords::lattice_for(mesh);

    if lattice.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return Ok(placed);
    }

    let is_split: Vec<bool> = (0..sections.len())
        .map(|i| reads_as_a_split_vertex_at(sections, i, lattice))
        .collect();

    let mut blended: Vec<Option<BevelSection>> = vec![None; sections.len()];

    for i in 0..sections.len() {
        if !is_split[i] {
            continue;
        }

        let before = nearest_that_is_not_split(&is_split, i as i32, -1);
        let after = nearest_that_is_not_split(&is_split, i as i32, 1);

        if before < 0 || after < 0 {
            // A chord with no unsplit vertex either side of it - a whole chain read as one straight
            // line - has no pair of sections to blend between.
            continue;
        }

        if let Some((section, parameter)) = try_blend(
            &sections[before as usize],
            &sections[after as usize],
            &sections[i],
            pool,
        )? {
            blended[i] = Some(section);
            placed[i] = SplitPlacement::new(before, after, parameter);
        }
    }

    for (section, blend) in sections.iter_mut().zip(blended) {
        if let Some(blend) = blend {
            *section = blend;
        }
    }

    Ok(placed)
}

/// C# `ReadsAsASplitVertex(BevelSection[], int, double)`: whether the SMOOTH section at `i` stands
/// on the straight line between its two neighbours' stations with both its sides offsets.
pub fn reads_as_a_split_vertex_at(sections: &[BevelSection], i: usize, lattice: f64) -> bool {
    let section = &sections[i];

    if section.kind != SectionKind::Smooth {
        return false;
    }

    let count = sections.len();
    let before = &sections[(i + count - 1) % count];
    let after = &sections[(i + 1) % count];

    reads_as_a_split_vertex(
        section.left_kind == SectionSide::Offset,
        section.right_kind == SectionSide::Offset,
        section.station,
        before.station,
        after.station,
        lattice,
    )
}

/// C# `ReadsAsASplitVertex(bool, bool, ..)`: a station with both sides offsets standing on the
/// straight line between its neighbours. A side that SLIDES has a fold of the part to ride and its
/// own placement is the right one; blending it would take its boundary off the very edge the
/// substitution keeps.
pub fn reads_as_a_split_vertex(
    left_is_offset: bool,
    right_is_offset: bool,
    station: Vector3,
    before: Vector3,
    after: Vector3,
    lattice: f64,
) -> bool {
    left_is_offset
        && right_is_offset
        && bevel_split_chords::stands_on_the_line_between(station, before, after, lattice)
}

/// C# `TryBlend`: the split section redrawn from its two neighbours - each end slid ALONG the chain
/// to stand where the line between the neighbours' boundaries stands at the split's own parameter,
/// every sample between moved by the share of that slide its place across the profile gives it - with
/// that parameter. `Ok(None)` where the profiles differ in length or the split does not stand strictly
/// between its neighbours along the chord.
///
/// The ENDS of the chord are all this reads, so a CORNER standing at one end is a perfectly good
/// end: its shared-side boundary rides the same fold the smooth neighbour's would.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_blend(
    before: &BevelSection,
    after: &BevelSection,
    split: &BevelSection,
    pool: &mut SectionPool,
) -> Result<Option<(BevelSection, f64)>, NanSectionPoint> {
    if before.profile.len() != split.profile.len() || after.profile.len() != split.profile.len() {
        return Ok(None);
    }

    let along = after.station - before.station;
    let span = along.length_squared();

    if !(span > 0.0) {
        return Ok(None);
    }

    let parameter = (split.station - before.station).dot(along) / span;

    if !(parameter > 0.0) || !(parameter < 1.0) {
        return Ok(None);
    }

    // How far each end of the section has to travel ALONG the chord to stand where the line between
    // its two neighbours' boundaries stands at this parameter. Only the along-chord coordinate is
    // taken: sliding along the chord keeps every sample's offset from its own centre what the
    // section drew, radius and all.
    let tangent = split.tangent;
    let last = split.profile.len() - 1;

    let to_the_left = how_far_along(
        between(
            pool.position_of(before.profile[0]),
            pool.position_of(after.profile[0]),
            parameter,
        ),
        pool.position_of(split.profile[0]),
        tangent,
    );

    let to_the_right = how_far_along(
        between(
            pool.position_of(before.profile[before.profile.len() - 1]),
            pool.position_of(after.profile[after.profile.len() - 1]),
            parameter,
        ),
        pool.position_of(split.profile[last]),
        tangent,
    );

    let mut profile = Vec::with_capacity(split.profile.len());

    for k in 0..split.profile.len() {
        let across = k as f64 / last as f64;
        let moved = pool.position_of(split.profile[k])
            + (tangent * between_values(to_the_left, to_the_right, across));

        profile.push(pool.add(moved)?);
    }

    let section = BevelSection::new(
        split.chain_vertex,
        split.station,
        tangent,
        profile[0],
        profile[last],
        profile.clone(),
        split.centre_from + (tangent * to_the_left),
        split.centre_to + (tangent * to_the_right),
        split.left_kind,
        split.right_kind,
        split.left_slide_edge,
        split.right_slide_edge,
        split.setback,
    );

    // The profile runs from its own first point to its own last by construction.
    Ok(section.ok().map(|section| (section, parameter)))
}
