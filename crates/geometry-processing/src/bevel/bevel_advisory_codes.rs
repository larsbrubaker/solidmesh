//! What a [`BevelAdvisoryCode`] is ABOUT, where something has to tell one sort from another.
//! Crate-visible free functions.

use super::bevel_advisory::{BevelAdvisory, BevelAdvisoryArg};
use super::bevel_advisory_code::BevelAdvisoryCode;

/// Whether a code describes SOURCE CONDITIONING - something `BevelWorkingMesh` had to do to its
/// copy of the INPUT before any construction ran, rather than anything the construction did or
/// could not do.
///
/// A test asking whether a case BUILT must not read these as complaints, and a panel may show
/// them as notes about the file. `SourceMeshWasInsideOut` is deliberately NOT one of these:
/// turning a part right way out changes which way every surface of the answer faces.
pub fn is_source_conditioning(code: BevelAdvisoryCode) -> bool {
    matches!(
        code,
        BevelAdvisoryCode::SourceMeshHadFoldedFlaps
            | BevelAdvisoryCode::SourceMeshHadTVertexSlivers
            | BevelAdvisoryCode::SourceMeshHadThinSlivers
            | BevelAdvisoryCode::SourceMeshHadSplitChords
    )
}

/// Whether an advisory is a NOTE about a bevel that built rather than news of something given up
/// on: the source conditioning notes plus `BuiltWithTheOtherConstruction` (the pick came back
/// BUILT, by the construction the user did not set).
pub fn is_a_note_rather_than_a_complaint(code: BevelAdvisoryCode) -> bool {
    is_source_conditioning(code) || code == BevelAdvisoryCode::BuiltWithTheOtherConstruction
}

/// The largest size a refusal about the SIZE already worked out would have fitted, where the
/// refusal carries one (`None` otherwise).
///
/// These are the refusals whose own sentence ends "The most that fits there is about x". The
/// codes are listed rather than a convention being assumed about argument three, because an
/// advisory's arguments mean whatever its own sentence says they mean.
pub fn try_the_most_that_fits(advisory: Option<&BevelAdvisory>) -> Option<f64> {
    let advisory = advisory?;
    if advisory.args().len() < 3 {
        return None;
    }

    match advisory.code() {
        BevelAdvisoryCode::CapPullsToolBackPastSegment
        | BevelAdvisoryCode::SizeFoldsToolAtCurve
        | BevelAdvisoryCode::SizeCollidesAcrossFace
        | BevelAdvisoryCode::SizeExceedsFaceWidth
        | BevelAdvisoryCode::SizeReachesPastCrease => {}
        _ => return None,
    }

    match advisory.args()[2] {
        BevelAdvisoryArg::Double(found) if found.is_finite() && found > 0.0 => Some(found),
        _ => None,
    }
}
