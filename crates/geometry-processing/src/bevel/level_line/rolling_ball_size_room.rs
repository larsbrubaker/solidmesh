//! The room a pass of ONE chain names where the rolling ball refused it at the size asked
//! (strict mode, `band_gap_build_within.rs`): the ball's own room, read off a walk at a smaller
//! size the ball does take, so a size far past the part (a sheared prism's top edge at 100, a
//! block's edge whose round would cross a bore) is refused by its size, naming a number the user
//! can ask for, instead of by the ball with no number at all.
//!
//! **Why the ball's room and not the angle's.** The level line's own setbacks read a room the
//! ball's band need not have: on the T Handle they read the pocket wall's room through the web,
//! past the end face the ball's round is cut flat by (`rolling_ball_end_planes`). So the room is
//! read exactly as the build reads it for a pass the ball takes (`one_chain_room`): with the
//! ball's setbacks per unit of the walked size, its own sides and its end planes, on the mesh
//! tagged by them. Nothing is cut; the reading stops at the room.
//!
//! **The rule.**
//! 1. Try the powers of two below the size asked, largest first, until the ball walks, at most
//!    [`HALVINGS`] of them. No size walks: the ball's own refusal stands. The search goes on down
//!    only while the walk's refusal says the size is too big (its kind, `rolling_ball_refusal`:
//!    the round reaching past the part, or an end station's contact resting on its side's rim
//!    without touching the side square there); any other refusal at a power of two ends it
//!    there, since halving the size does not lift it (the T Handle's junction chains, whose rails
//!    miss their end faces at every size down to 1/128 of R12, paid eight walks per rung of the
//!    size ladder for none). Powers of two and not halves of the size asked, so every size past
//!    the ball's limit walks at the same size and reads the same room to the bit: the room is a
//!    fact about the part, not about the size asked (a contact solved at another size moves in
//!    its last bits, and so would the room).
//! 2. Read the room at that size. A room at or above the size asked does not explain the refusal
//!    (the ball stopped for something no room sees), so the ball's own refusal stands.
//! 3. The room is the size the refusal names where the ball walks there and its own reading there
//!    admits it, so the size named builds. On flat sides the ball's setbacks per unit do not
//!    change with the size and the room is the same number whatever size it was read at; on a
//!    curved side it is the estimate `one_chain_room`'s header describes, checked here by that
//!    one walk. A walked size whose own room is below it, with a room the ball does not take,
//!    leaves no size known to build: the ball's own refusal stands.
//! 4. Otherwise bisect between the walked size and the room, at most [`BISECTIONS`] times, for the
//!    largest size that walks and fits its own room, and name that: a size found to build, never
//!    the room just found not to.
//!
//! **Cost.** At most `HALVINGS + 1 + BISECTIONS` walks (14) and `2 + BISECTIONS` room readings
//! (7), each reading the build's steps 1 and 2 and its room on a fresh copy of the mesh, and only
//! for a one-chain pass the ball refused; one walk where the first power of two is refused for a
//! reason a smaller size does not lift.
//!
//! The walks write the route as they go; the ball's refusal at the size asked is written back on
//! every way out of the search, so the route a refused pass reports is still its own
//! (`rolling_ball_routing`). A trial reading that runs out of work leaves the ball's refusal, not
//! a build that ran out; a cancel or a fault leaves as it came.

use vector_math::Vector3;

use super::band_gap::BuildWithinArgs;
use super::band_gap_build_fault::BuildFault;
use super::band_gap_build_within;
use super::band_gap_setback_fields::PassSetbacks;
use super::rolling_ball_refusal::BallRefusal;
use super::rolling_ball_setbacks::BallPass;
use super::seam_edges::FeatureEdgeTest;
use super::{square_crease_end_clip, BandGapRefusal, BandGapResult, RoomSource, WorkError};
use crate::bevel::rolling_ball::rolling_ball_routing;

/// How many powers of two below the size asked are walked, looking for a size the ball takes: down
/// to about 1/256 of it, which covers a size asked two orders of magnitude past the part.
pub const HALVINGS: usize = 8;

/// How many bisections narrow the largest size that walks and fits, where the room read at a
/// smaller size is not one the ball takes: 1/32 of the gap between them.
pub const BISECTIONS: usize = 5;

/// What the build's room reading says at one size (`band_gap_build_within`'s room-only pass).
#[derive(Clone, Debug)]
pub struct RoomAtSize {
    /// The room read at that size.
    pub room: f64,
    /// What set it.
    pub set_by: RoomSource,
    /// Whether that size fits its room (or its band is consumed exactly to a rim).
    pub fits: bool,
}

impl RoomAtSize {
    /// The room-only pass's answer read back: `None` where it refused for anything but the size.
    pub fn of(result: &BandGapResult) -> Option<RoomAtSize> {
        let fits = match result.refusal {
            BandGapRefusal::None => true,
            BandGapRefusal::Size => false,
            _ => return None,
        };
        Some(RoomAtSize {
            room: result.room,
            set_by: result.room_set_by,
            fits,
        })
    }
}

/// A pass the rolling ball was asked to take and cannot, refused with `why`.
pub fn refused_by_the_ball(why: String) -> BandGapResult {
    BandGapResult {
        refusal: BandGapRefusal::RollingBall,
        rolling_ball_refusal: Some(why),
        ..BandGapResult::default()
    }
}

/// What the build reads a room at another size with: the pass as the build was handed it.
pub struct ReadAt<'a> {
    pub positions: &'a [Vector3],
    pub indices: &'a [(i32, i32, i32)],
    pub chains: &'a [Vec<i32>],
    pub closed: &'a [bool],
    pub is_feature_edge: &'a Option<FeatureEdgeTest>,
    pub args: &'a BuildWithinArgs<'a>,
    pub setbacks: &'a PassSetbacks,
}

impl ReadAt<'_> {
    /// The build's room at `size` on the ball's walk there (`pass`), nothing cut.
    fn room_at(&self, size: f64, pass: BallPass) -> Result<Option<RoomAtSize>, BuildFault> {
        let at = BuildWithinArgs { size, ..*self.args };
        let reading = band_gap_build_within::build_on_the_walk(
            self.positions,
            self.indices,
            self.chains,
            self.closed,
            self.is_feature_edge.clone(),
            &at,
            &mut None,
            &mut None,
            self.setbacks,
            Some(pass),
            true,
        )?;
        Ok(RoomAtSize::of(&reading))
    }
}

/// The refusal of a one-chain pass the ball refused at `size` (`refused`, see the file header):
/// by size naming the ball's own room where a smaller size walks and that room explains the
/// refusal, by the ball otherwise. `walk` walks the ball at a size; `read` reads the build's room
/// at a size on that walk. Whatever leaves the search, the route is the ball's refusal at the size
/// asked again.
pub fn refusal(
    size: f64,
    refused: BallRefusal,
    walk: &mut dyn FnMut(f64) -> Result<BallPass, BallRefusal>,
    read: &ReadAt,
) -> Result<BandGapResult, BuildFault> {
    let named = named_room(size, walk, &mut |s, pass| read.room_at(s, pass));
    // The walks wrote their own routes; the pass's is the ball's refusal at the size asked.
    rolling_ball_routing::record_refusal(&refused.why);
    Ok(match named {
        Ok(Some((room, room_set_by))) => BandGapResult {
            refusal: BandGapRefusal::Size,
            room,
            room_set_by,
            ..BandGapResult::default()
        },
        Ok(None) => refused_by_the_ball(refused.why),
        // A trial reading that ran out of work found no room: the size asked is still the ball's
        // refusal, not a build that ran out.
        Err(BuildFault::Work(WorkError::BudgetExceeded(_))) => refused_by_the_ball(refused.why),
        Err(fault) => return Err(fault),
    })
}

/// The room to name and what set it, or `None` where no size is known to build (the rule's steps).
fn named_room(
    size: f64,
    walk: &mut dyn FnMut(f64) -> Result<BallPass, BallRefusal>,
    room_at: &mut dyn FnMut(f64, BallPass) -> Result<Option<RoomAtSize>, BuildFault>,
) -> Result<Option<(f64, RoomSource)>, BuildFault> {
    if !size.is_finite() {
        return Ok(None);
    }
    // 1. A size the ball walks: the powers of two below the size asked, largest first.
    let mut walked = None;
    let mut tried = 2f64.powi(size.log2().floor() as i32);
    while tried * 2.0 < size {
        tried *= 2.0;
    }
    while tried >= size {
        tried *= 0.5;
    }
    for _ in 0..HALVINGS {
        match walk(tried) {
            Ok(pass) => {
                walked = Some((tried, pass));
                break;
            }
            // Only a refusal that says the size is too big goes on down (see the file header).
            Err(refused) if !refused.kind.says_the_size_is_too_big() => return Ok(None),
            Err(_) => {}
        }
        tried *= 0.5;
    }
    let Some((walked_size, pass)) = walked else {
        return Ok(None);
    };

    // 2. Its room, which must explain the refusal.
    let Some(reading) = room_at(walked_size, pass)? else {
        return Ok(None);
    };
    let room = reading.room;
    if !(room.is_finite() && room > 0.0 && room < size) {
        return Ok(None);
    }

    // 3. The room itself, where the ball takes it.
    let mut fits_at = |s: f64| -> Result<Option<RoomAtSize>, BuildFault> {
        match walk(s) {
            Ok(pass) => Ok(room_at(s, pass)?.filter(|at| at.fits)),
            Err(_) => Ok(None),
        }
    };
    if fits_at(room)?.is_some() {
        return Ok(Some((room, reading.set_by)));
    }

    // 4. The largest size between a walked size that fits and the room that walks and fits.
    if !reading.fits || walked_size >= room {
        return Ok(None);
    }
    let (mut low, mut high) = (walked_size, room);
    let mut low_set_by = reading.set_by;
    for _ in 0..BISECTIONS {
        let middle = 0.5 * (low + high);
        match fits_at(middle)? {
            Some(at) => {
                low = middle;
                low_set_by = at.set_by;
            }
            None => high = middle,
        }
    }
    Ok(Some((low, low_set_by)))
}

/// A room-only pass's answer (`band_gap_build_within`'s `build_on_the_walk`): the room read,
/// capped where a square crease end's section leaves its crease as step 5 caps it, and whether
/// the size fits it.
#[allow(clippy::too_many_arguments)]
pub fn room_only_answer(
    args: &BuildWithinArgs,
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    read_room: Option<f64>,
    mut room_set_by: RoomSource,
    ends_on_the_rim: bool,
) -> BandGapResult {
    let Some(mut room) = read_room else {
        return BandGapResult {
            refusal: BandGapRefusal::RollingBall,
            ..BandGapResult::default()
        };
    };
    square_crease_end_clip::cap_room(
        positions,
        indices,
        chains,
        closed,
        args.sharp_turn_degrees,
        &mut room,
        &mut room_set_by,
    );
    let fits = args.size <= room || ends_on_the_rim;
    BandGapResult {
        refusal: if fits {
            BandGapRefusal::None
        } else {
            BandGapRefusal::Size
        },
        room,
        room_set_by,
        ..BandGapResult::default()
    }
}
