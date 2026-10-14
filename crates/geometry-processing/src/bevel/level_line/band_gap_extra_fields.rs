//! The extra fields a band's gap reads beyond its pick: the crease run-outs
//! ([`band_group_run_outs`](super::band_group_run_outs)) and
//! the mitred pair corners' end-face fields ([`PairCornerEndFaceField::find`]), gathered by group.

use vector_math::Vector3;

use super::{band_group_run_outs, triangle_edges, BandGroupExtraFields, PairCornerEndFaceField};

/// The run-outs' fields by group, with the pair corners' end-face
/// fields appended to each group's list - `None` where neither found anything.
#[allow(clippy::too_many_arguments)]
pub fn gather(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    size: f64,
    tags: &[i32],
    input_standing: Option<&triangle_edges::StandingMap>,
) -> Option<BandGroupExtraFields> {
    let mut run_outs = band_group_run_outs(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        size,
        tags,
        input_standing,
    );

    // A round running out through its end face at a mitred pair corner reads the same way.
    if let Some(pair_corners) = PairCornerEndFaceField::find(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        size,
        tags,
        input_standing,
    ) {
        let gathered = run_outs.get_or_insert_with(BandGroupExtraFields::new);
        for (tag, list) in pair_corners {
            gathered.entry(tag).or_default().extend(list);
        }
    }

    run_outs
}
