//! What one bevel operation has already found out about a pick that does not depend on the size
//! ([`BevelPickMemo`]) - the pick's room and how a many-loop rim settles - kept across that
//! operation's rebuilds. The caller keys it (a pick's chains, profile and the mesh's content
//! stamp, [`BevelPickMemo::the_stamp_of`]), owns one per operation and hands it to every rebuild;
//! the settlings it holds are [`SettledPick`]s.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

use solidmesh_mesh::Mesh;

use super::settled_pick::SettledPick;
use crate::bevel::level_line::RoomSource;

/// What one bevel operation remembers about its picks across rebuilds. A caller makes one per
/// operation ([`BevelPickMemo::new`]) and hands the same `Arc` to every rebuild in
/// [`BevelPartRequest::pick_memo`](super::BevelPartRequest::pick_memo), which lets a drag of the
/// size measure a pick's room once; everything else on it is the kernel's own.
///
/// The room of a pick is a statement about the PART and the PICK: how much space the sides of each
/// chain have to give. The size the user is dragging does not enter it. Reading it is the longest
/// single thing on the level-line route - most of a minute on a rim of forty-five chains - and a
/// live drag of the size rebuilds at every step and is never skipped for being slow, so the only
/// honest way to make the drag keep up is to stop re-paying what the size did not change.
///
/// It belongs to the OPERATION and not to the process. The builder is constructed fresh for every
/// rebuild, so a memo on the builder would never hit; a static one hit across rebuilds but also
/// across unrelated operations, and leaked one test's reading into the next test's count of how
/// many readings a route took. A fresh operation - and so every test - starts with an empty memo,
/// and a copied, cloned or undone operation is a new object with a new empty memo, which is always
/// safe: an empty memo only means the next rebuild reads for itself.
///
/// No entry holds a mesh. The mesh is in the key as a content stamp, so an entry can neither pin a
/// working mesh alive nor be handed to a build of a mesh it was not read on. One mutex guards it;
/// the memo is shared between rebuilds (which may run on worker threads) by `Arc`.
#[derive(Default)]
pub struct BevelPickMemo {
    the_lock: Mutex<MemoState>,
}

/// Everything kept under the lock. Looked up by key only and never iterated, so the maps'
/// order cannot reach an output.
#[derive(Default)]
struct MemoState {
    /// Room by key; the value is itself an option, because "no size statement" is worth keeping.
    rooms: HashMap<String, Option<f64>>,

    settlings: HashMap<String, TheSettlingRemembered>,

    /// The lowest room a BUILD has proved for a pick - a pinch refusal's number - which is a
    /// ceiling nothing may raise, and which is consulted whatever kind of entry the pick is
    /// answered from. Kept apart from `rooms` because a SETTLED pick never writes a room entry at
    /// all: its room comes out of the settling. A pinch that reached only the room entry therefore
    /// left the settling saying "nobody condemned, fits" at a size the build had already refused,
    /// and the pinch ladder was paid again at every step of the drag below it.
    floors: HashMap<String, f64>,

    /// The largest size a build has actually carried for each pick, which no floor may sit under.
    /// A pinch refusal's number is a size KNOWN to fit, found by halving below the size that
    /// pinched - up to about 12% under the true limit - so as a ceiling it can sit under a size
    /// that builds: on a plate's rim beside a 0.5 mm block, 0.494 built, 0.5 pinched and named
    /// 0.469, and the drag back down to 0.494 was refused by the memo without building anything.
    built: HashMap<String, f64>,

    /// The smallest size a build was actually REFUSED at for each pick, where the refusal said it -
    /// the sizes its floor is a ceiling for. A floor is a size known to fit, and a size between it
    /// and the size that was refused is unknown until built; held under the floor there, a drag to
    /// 0.499 was handed 0.494 where a fresh build at 0.499 builds every loop. The result at a size
    /// must not depend on the drag that reached it, so below the refused size the floor says
    /// nothing.
    refused_at: HashMap<String, f64>,

    /// Which reading named each remembered room and each floor, so a refusal answered from the
    /// memo can still say whether the room was two bevels meeting or one bevel reaching the far
    /// side of its face.
    room_sources: HashMap<String, RoomSource>,

    floor_sources: HashMap<String, RoomSource>,

    /// The chain the PINCH that set each pick's floor named - see
    /// [`BevelPickMemo::the_pinched_chain_under_the_floor`].
    floor_pinched_chains: HashMap<String, i32>,

    /// The mesh stamp everything held here was read on, or `None` while the memo is empty. One
    /// stamp at a time, because every key carries the stamp and a key from another one can never
    /// be read again.
    the_stamp: Option<u64>,

    rooms_read: i32,

    settlings_run: i32,
}

/// One remembered settling of a many-loop rim.
struct TheSettlingRemembered {
    /// The size the settling was asked at.
    size: f64,

    settled: Arc<SettledPick>,

    /// The survivors' room, as the settling's last reading read it, or `None` where it made no
    /// size statement.
    room: Option<f64>,
}

impl BevelPickMemo {
    /// A fresh, empty memo.
    pub fn new() -> Self {
        Self::default()
    }

    fn lock(&self) -> MutexGuard<'_, MemoState> {
        // A panic while holding the lock leaves maps that are only ever written whole entries.
        self.the_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// How many room readings were actually PAID for against this memo - the drag's own count.
    #[doc(hidden)]
    pub fn rooms_read(&self) -> i32 {
        self.lock().rooms_read
    }

    /// How many settlings were actually paid for against this memo.
    #[doc(hidden)]
    pub fn settlings_run(&self) -> i32 {
        self.lock().settlings_run
    }

    /// How many keys are held, which is what a long drag must not grow without bound.
    #[doc(hidden)]
    pub fn entries(&self) -> i32 {
        let state = self.lock();
        (state.rooms.len() + state.settlings.len() + state.floors.len() + state.built.len()) as i32
    }

    /// A stamp of the mesh's whole CONTENT - every vertex position's raw bits and
    /// every face's indices, folded into one 64-bit FNV-1a.
    ///
    /// Taken here and not from `Mesh::get_long_hash_code`, which SAMPLES: on a big part an upstream
    /// edit that moves a small feature can move only vertices between the samples, and the
    /// operation would be handed the room read BEFORE the edit for the rest of its life. A 64-bit
    /// hash can in principle collide; every byte of the geometry enters it, so a collision needs
    /// an adversary rather than an edit.
    #[doc(hidden)]
    pub fn the_stamp_of(mesh: &Mesh) -> u64 {
        let mut hash: u64 = 14695981039346656037;

        // FNV-1a's step taken a 32-bit WORD at a time rather than a byte at a time: a quarter of
        // the multiplies for the same bytes. The byte-at-a-time order is what gives FNV its
        // avalanche, so the word form is finished with a strong mix below rather than trusted raw.
        let mut fold = |bits: u32| {
            hash = (hash ^ bits as u64).wrapping_mul(1099511628211);
        };

        let vertices = &mesh.vertices;

        // `as u32` reinterprets an `i32`'s bits, so a negative index folds as its two's complement.
        fold(vertices.len() as u32);

        for vertex in vertices.iter() {
            fold(vertex.x.to_bits());
            fold(vertex.y.to_bits());
            fold(vertex.z.to_bits());
        }

        let faces = &mesh.faces;

        fold(faces.len() as u32);

        for face in faces.iter() {
            fold(face.v0 as u32);
            fold(face.v1 as u32);
            fold(face.v2 as u32);
        }

        // splitmix64's finisher: spreads the word-wise fold's bits so two meshes differing in one
        // float do not land near each other.
        hash ^= hash >> 30;
        hash = hash.wrapping_mul(0xBF58476D1CE4E5B9);
        hash ^= hash >> 27;
        hash = hash.wrapping_mul(0x94D049BB133111EB);

        hash ^ (hash >> 31)
    }

    /// Forget everything, so the next rebuild reads for itself - for a caller that must begin from
    /// a VIRGIN operation although this one has already been built (the route tests' setup).
    pub fn forget(&self) {
        let mut state = self.lock();
        state.empty();
        state.rooms_read = 0;
        state.settlings_run = 0;
    }

    /// The room remembered for this key, where one is - never above a floor a
    /// build has proved. `None` where nothing is remembered; `Some(None)` is a remembered "no size
    /// statement". `size` is the size the room is asked for, or `None` to take the floor at every
    /// size.
    #[doc(hidden)]
    pub fn try_the_room(&self, key: &str, size: Option<f64>) -> Option<Option<f64>> {
        let state = self.lock();
        let room = *state.rooms.get(key)?;

        Some(state.the_lower_of(key, room, size))
    }

    /// Remember what a reading just paid for came to, on the mesh it was read on. Kept as READ and
    /// laid under the floor only when it is read back ([`Self::try_the_room`]), so a reading
    /// cannot raise a room a build has already refused at. Every caller names the reading that set
    /// it (`set_by`).
    #[doc(hidden)]
    pub fn remember_the_room(&self, stamp: u64, key: &str, room: Option<f64>, set_by: RoomSource) {
        let mut state = self.lock();
        state.only_this_mesh(stamp);

        state.room_sources.insert(key.to_string(), set_by);
        state.rooms.insert(key.to_string(), room);
        state.rooms_read += 1;
    }

    /// Lower this pick's remembered room to a room a refusal named, when that room is smaller.
    ///
    /// Only ever DOWN: the entry is a ceiling on what builds, and a pinch refusal has proved a
    /// smaller one. Written as a FLOOR of its own and not only onto a room entry, because the pick
    /// this matters most for - a many-loop rim - is answered from a SETTLING and has no room entry
    /// at all. `pinched_chain` is -1 where the pinch named none;
    /// `refused_at_size` is the size the refusal was at, or `None` where it is not known and the
    /// floor holds every size.
    #[doc(hidden)]
    pub fn remember_a_smaller_room(
        &self,
        stamp: u64,
        key: &str,
        room: f64,
        set_by: RoomSource,
        pinched_chain: i32,
        refused_at_size: Option<f64>,
    ) {
        // `is_finite` also refuses NaN, so a NaN room is never remembered.
        if room <= 0.0 || !room.is_finite() {
            return;
        }

        let mut state = self.lock();
        state.only_this_mesh(stamp);

        let mut room = room;

        // Never under a size that has built: the pinch's number is a size known to fit, not the
        // most.
        if let Some(&carried) = state.built.get(key) {
            room = room.max(carried);
        }

        let lower = state.floors.get(key).is_none_or(|&known| room < known);
        if lower {
            state.floors.insert(key.to_string(), room);
            state.floor_sources.insert(key.to_string(), set_by);

            if pinched_chain >= 0 {
                state
                    .floor_pinched_chains
                    .insert(key.to_string(), pinched_chain);
            } else {
                state.floor_pinched_chains.remove(key);
            }
        }

        // Only ever down, and a refusal whose size is not known holds the floor at every size.
        let at = match refused_at_size {
            Some(value) if value.is_finite() => value,
            _ => 0.0,
        };
        if state.refused_at.get(key).is_none_or(|&before| at < before) {
            state.refused_at.insert(key.to_string(), at);
        }
    }

    /// Remember that this pick built at this size, and lift a floor standing under it.
    #[doc(hidden)]
    pub fn remember_a_size_that_built(&self, stamp: u64, key: &str, size: f64) {
        // `is_finite` also refuses NaN, so a NaN size is never remembered.
        if size <= 0.0 || !size.is_finite() {
            return;
        }

        let mut state = self.lock();
        state.only_this_mesh(stamp);

        if state.built.get(key).is_none_or(|&carried| size > carried) {
            state.built.insert(key.to_string(), size);
        }

        if state.floors.get(key).is_some_and(|&floor| floor < size) {
            state.floors.insert(key.to_string(), size);

            // The whole pick fitted at this size, so where that is at or above the size the pinch
            // refused at, no one chain is to blame for the floor now. Below it the pinch still
            // stands and still names its chain: on the plate rim beside a 0.5 mm block, 0.499
            // built after 0.5 pinched, and the drag on to 0.6 - refused from the floor - blamed
            // nobody and refused every loop, where a fresh 0.6 leaves the short block's foot
            // alone. The source stays, as it still says which reading the room came from.
            if state.refused_at.get(key).is_none_or(|&at| size >= at) {
                state.floor_pinched_chains.remove(key);
            }
        }

        // The remembered ROOM is left as read, even under a size that built: only one open chain
        // can build past its reading (it flows over a narrow end face, BandGap.Build), and a fresh
        // build handed that same reading flows past it too, so raising it would only make a drag
        // differ from a fresh build.
    }

    /// Which reading named the pick's OWN remembered room, never a floor's source - what the
    /// pick's room is about, whatever a later refusal lowered it to.
    #[doc(hidden)]
    pub fn the_readings_own_source(&self, key: &str) -> RoomSource {
        self.lock()
            .room_sources
            .get(key)
            .copied()
            .unwrap_or_else(RoomSource::nothing)
    }

    /// This room, or the floor a build has proved for the pick where that is lower. `size` is the
    /// size the room is asked for, or `None` to take the floor at every size.
    #[doc(hidden)]
    pub fn the_room_under_its_floor(
        &self,
        key: &str,
        room: Option<f64>,
        size: Option<f64>,
    ) -> Option<f64> {
        self.lock().the_lower_of(key, room, size)
    }

    /// Which reading named the room this memo answers for the key - the floor's where the floor
    /// is what holds the room down, the remembered room's otherwise. The zero source where
    /// neither is known ([`zero_room_source`]).
    #[doc(hidden)]
    pub fn the_room_source(&self, key: &str, size: Option<f64>) -> RoomSource {
        let state = self.lock();
        let room = state.rooms.get(key).copied().flatten();
        let set_by = state
            .room_sources
            .get(key)
            .copied()
            .unwrap_or_else(zero_room_source);

        state.the_source_of(key, room, set_by, size)
    }

    /// Which reading named this room once the pick's floor is laid under it - the pair to
    /// [`Self::the_room_under_its_floor`].
    #[doc(hidden)]
    pub fn the_source_under_its_floor(
        &self,
        key: &str,
        room: Option<f64>,
        set_by: RoomSource,
        size: Option<f64>,
    ) -> RoomSource {
        self.lock().the_source_of(key, room, set_by, size)
    }

    /// The chain the pinch that set this pick's floor named, where that floor is what holds this
    /// room - otherwise -1.
    ///
    /// A build handed the floor refuses at once, before it can pinch again, so it has no chain of
    /// its own to name; the chain the pinch itself named is remembered beside the floor, and a warm
    /// refusal names the loop the cold one did. Only where the floor holds at the size asked:
    /// below the size the pinch refused at, a refusal is the build's own.
    #[doc(hidden)]
    pub fn the_pinched_chain_under_the_floor(
        &self,
        key: &str,
        room: Option<f64>,
        size: Option<f64>,
    ) -> i32 {
        let state = self.lock();

        match state.the_floor_at(key, size) {
            Some(floor) if room.is_none_or(|room| room >= floor) => {
                state.floor_pinched_chains.get(key).copied().unwrap_or(-1)
            }
            _ => -1,
        }
    }

    /// The settling this pick may re-use at `size`, or `None` where it must settle again.
    ///
    /// The same pick asked at the SAME size is the same question, so its answer is re-used whole.
    /// Asked at another size it is re-used only when the remembered settling condemned NOBODY and
    /// its room is at or above the new size: the kernel condemns a chain on `candidate < size`, so
    /// nobody can be condemned. Both rules are read against the settling's room taken UNDER any
    /// floor a build has since proved.
    #[doc(hidden)]
    pub fn the_settling_to_reuse(&self, key: &str, size: f64) -> Option<Arc<SettledPick>> {
        let state = self.lock();
        let known = state.settlings.get(key)?;

        let room = state.the_lower_of(key, known.room, Some(size));

        // The FLOOR first, and before the same-size rule too. A settling that fitted at the size
        // it was asked at is stale the moment a build refuses this pick below that size. Read off
        // the floor itself and not off the merged room, because a settling that CONDEMNED somebody
        // honestly carries a survivors' room below the size it was asked at.
        if known.settled.fits
            && state
                .the_floor_at(key, Some(size))
                .is_some_and(|floor| floor < size)
        {
            return None;
        }

        if known.size == size {
            return Some(known.settled.clone());
        }

        if known.settled.condemned.is_empty() && room.is_some_and(|room| room >= size) {
            Some(known.settled.clone())
        } else {
            None
        }
    }

    /// Remember a settling just paid for, with the size it was asked at and the survivors' room.
    #[doc(hidden)]
    pub fn remember_the_settling(
        &self,
        stamp: u64,
        key: &str,
        size: f64,
        settled: Arc<SettledPick>,
        room: Option<f64>,
    ) {
        let mut state = self.lock();
        state.only_this_mesh(stamp);

        state.settlings.insert(
            key.to_string(),
            TheSettlingRemembered {
                size,
                settled,
                room,
            },
        );
        state.settlings_run += 1;
    }

    /// Remember, under ANOTHER key, a settling already known to hold for it - the survivors of a
    /// verdict, whose room the settling's last reading is a reading of. Counted as no work,
    /// because none was done.
    #[doc(hidden)]
    pub fn carry_the_settling_to(
        &self,
        stamp: u64,
        key: &str,
        size: f64,
        settled: Arc<SettledPick>,
        room: Option<f64>,
    ) {
        let mut state = self.lock();
        state.only_this_mesh(stamp);

        state.settlings.insert(
            key.to_string(),
            TheSettlingRemembered {
                size,
                settled,
                room,
            },
        );
    }
}

impl MemoState {
    fn the_source_of(
        &self,
        key: &str,
        room: Option<f64>,
        set_by: RoomSource,
        size: Option<f64>,
    ) -> RoomSource {
        if let Some(floor) = self.the_floor_at(key, size) {
            if room.is_none_or(|room| floor < room) {
                if let Some(&floor_set_by) = self.floor_sources.get(key) {
                    return floor_set_by;
                }
            }
        }

        set_by
    }

    fn the_lower_of(&self, key: &str, room: Option<f64>, size: Option<f64>) -> Option<f64> {
        let Some(floor) = self.the_floor_at(key, size) else {
            return room;
        };

        // The floor wins unless a room is known and the floor is not below it; a NaN floor wins.
        match room {
            Some(room) if floor.partial_cmp(&room) != Some(std::cmp::Ordering::Less) => Some(room),
            _ => Some(floor),
        }
    }

    /// The pick's floor, where it holds at this size - at every size where no size is asked, and
    /// otherwise only at or above the size a build was refused at.
    fn the_floor_at(&self, key: &str, size: Option<f64>) -> Option<f64> {
        let floor = *self.floors.get(key)?;
        let holds = match size {
            None => true,
            Some(size) => self.refused_at.get(key).is_none_or(|&at| size >= at),
        };

        holds.then_some(floor)
    }

    /// A write on a new mesh empties what was read on the old one.
    fn only_this_mesh(&mut self, stamp: u64) {
        if self.the_stamp.is_some_and(|known| known != stamp) {
            self.empty();
        }

        self.the_stamp = Some(stamp);
    }

    fn empty(&mut self) {
        self.rooms.clear();
        self.settlings.clear();
        self.floors.clear();
        self.built.clear();
        self.room_sources.clear();
        self.floor_sources.clear();
        self.floor_pinched_chains.clear();
        self.refused_at.clear();
        self.the_stamp = None;
    }
}

/// The zero room source - the enum's first kind and chain 0, other chain 0 - which is NOT
/// `RoomSource::nothing` (-1, -1). A lookup that finds nothing answers with it.
pub fn zero_room_source() -> RoomSource {
    RoomSource::new(Default::default(), 0, 0)
}
