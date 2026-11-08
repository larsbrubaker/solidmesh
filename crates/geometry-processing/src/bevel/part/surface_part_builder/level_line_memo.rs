// The level line's memo keys - the working mesh's
// content stamp and the string a pick's findings are remembered against in the operation's
// `BevelPickMemo` (`crate::bevel::part::pick_memo`). The level-line route
// (`level_line`) and `level_line_rows` read and write the memo through these keys.

use crate::bevel::level_line::{BandProfile, RoomSource};
use solidmesh_math::number_format::format_f64;

use super::BevelSurfacePartBuilder;
use crate::bevel::part::BevelPickMemo;

/// The snap every reading on the level-line route is taken at (the kernel's default snap).
pub const THE_ROUTES_SNAP: f64 = 1e-9;

impl BevelSurfacePartBuilder {
    /// The one string a pick's findings are remembered against - the working
    /// mesh's content, the chains and their closedness, the profile, the snap every reading on
    /// this route is taken at ([`THE_ROUTES_SNAP`] is the default), the sharp turn and whether
    /// stopped ends are on.
    ///
    /// The mesh enters as EVERY position and EVERY face index, hashed to 64 bits
    /// ([`BevelPickMemo::the_stamp_of`]), so the same part rebuilt hits and a part whose source
    /// changed anywhere at all misses. The snap is a constant today; it is in the key because a
    /// reading taken at another one is another reading. The numbers are written in their
    /// shortest round-trip form, so equal numbers always make equal keys.
    pub fn the_memo_key_of(
        &mut self,
        chains: &[Vec<i32>],
        closed: &[bool],
        profile: BandProfile,
        snap: f64,
    ) -> String {
        // The sharp-turn angle is in the key because it moves the room: a mitred corner reads wider.
        // So do stopped ends, while a caller can turn them on (`turn_stopped_ends_on`).
        format!(
            "{}|{}|{}|{}|{}{}",
            self.the_stamp(),
            profile_name(profile),
            format_f64(snap),
            format_f64(self.the_sharp_turn_degrees),
            Self::the_key_of(chains, closed),
            if crate::bevel::level_line::flow_stopped_ends_on() {
                "|stopped"
            } else {
                ""
            }
        )
    }

    /// The working mesh's whole content, folded once per builder.
    ///
    /// The CONTENT and not the mesh's identity: the working mesh is copied fresh at every rebuild,
    /// so a reference comparison never hits across the steps of a drag, which is the whole thing
    /// the memo exists for.
    pub fn the_stamp(&mut self) -> u64 {
        *self
            .the_mesh_stamp
            .get_or_insert_with(|| BevelPickMemo::the_stamp_of(&self.working_mesh))
    }

    /// Every chain of the pick in order, as the one string the room is remembered
    /// against. Each chain carries its CLOSEDNESS into the key, because the same run of vertices
    /// read open and read closed are two different picks with two different rooms.
    pub fn the_key_of(chains: &[Vec<i32>], closed: &[bool]) -> String {
        chains
            .iter()
            .enumerate()
            .map(|(index, chain)| {
                let vertices: Vec<String> = chain.iter().map(i32::to_string).collect();
                format!(
                    "{}{}",
                    if closed[index] { "closed:" } else { "open:" },
                    vertices.join(",")
                )
            })
            .collect::<Vec<_>>()
            .join(";")
    }

    /// Lower the pick's remembered room to a room a refusal named, when
    /// that room is smaller. Only ever DOWN, and only onto an entry for this very pick - the same
    /// mesh content, chains, profile and snap - so a pinch on one pick can never lower another's.
    /// `pinched_chain` is -1 where none was named.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn remember_a_smaller_room(
        &mut self,
        chains: &[Vec<i32>],
        closed: &[bool],
        profile: BandProfile,
        room: f64,
        set_by: RoomSource,
        pinched_chain: i32,
        refused_at_size: Option<f64>,
    ) {
        // A room that is not positive, or NaN or infinite, is no room to remember.
        if room <= 0.0 || !room.is_finite() {
            return;
        }

        let stamp = self.the_stamp();
        let key = self.the_memo_key_of(chains, closed, profile, THE_ROUTES_SNAP);
        self.memo.remember_a_smaller_room(
            stamp,
            &key,
            room,
            set_by,
            pinched_chain,
            refused_at_size,
        );
    }
}

/// The profile's name, as the key carries it.
fn profile_name(profile: BandProfile) -> &'static str {
    match profile {
        BandProfile::Fillet => "Fillet",
        BandProfile::Chamfer => "Chamfer",
    }
}
