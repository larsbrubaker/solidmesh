//! Whether a level-line answer crosses itself where the part it was built on did not
//! ([`the_answer_intersects_itself_where_the_source_did_not`]) - the check every level-line answer
//! (rounds first, the pick-wide fill, the build past the room) must pass before it publishes. The
//! source is turned into a mesh by `answer_mesh`; the verdicts are the booleans' own operand
//! classification.

use std::sync::atomic::{AtomicU64, Ordering};

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use solidmesh_mesh::{CancelToken, Cancelled, Mesh};
use web_time::Instant;

use super::answer_mesh::the_mesh_of;

/// Total time spent in the self-intersection check, for measuring its cost (measurement tests read
/// it). In nanoseconds.
pub static SELF_INTERSECTION_CHECK_NANOS: AtomicU64 = AtomicU64::new(0);

/// Whether the answer crosses itself where the source (`positions`, `indices`) did not.
/// `Err(Cancelled)` when the token is cancelled during the check.
pub fn the_answer_intersects_itself_where_the_source_did_not(
    answer: &Mesh,
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    cancellation_token: &CancelToken,
) -> Result<bool, Cancelled> {
    let started = Instant::now();
    let result = (|| {
        let check = || {
            if cancellation_token.is_cancelled() {
                Err(Cancelled)
            } else {
                Ok(())
            }
        };

        // Each classify is a whole-part pass that takes no token, so Cancel is read on either side
        // of it.
        check()?;
        let answer_crosses = classify_boolean_operand(answer, Matrix4X4::IDENTITY, false)
            == BooleanOperandVerdict::SelfIntersecting;
        check()?;
        if !answer_crosses {
            return Ok(false);
        }

        let source = the_mesh_of(positions, indices);
        check()?;
        let source_clean = classify_boolean_operand(&source, Matrix4X4::IDENTITY, false)
            != BooleanOperandVerdict::SelfIntersecting;
        check()?;
        Ok(source_clean)
    })();

    // Counted whether the check finished, answered early or was cancelled.
    let elapsed = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
    SELF_INTERSECTION_CHECK_NANOS.fetch_add(elapsed, Ordering::Relaxed);

    result
}
