//! The beads a rebuild has gathered so far, each remembered with WHOSE it is, so the second pass
//! can take back the ones whose crease it re-read, with its adding, reading and withdrawing.
//!
//! Two lists rather than one, because a bead that brought a void wedge with it is trimmed to that
//! wedge before it joins the batch ([`super::bevel_bead_clip`]) and a bead that did not goes on
//! whole. A chain bead belongs to the one group whose crease it was swept along; a corner FILL
//! belongs to a vertex, so it names the groups whose beads adjoin it instead - which is what makes
//! it withdrawable when either of those beads is rebuilt.

use std::collections::HashSet;

use polygon_mesh::Mesh;

use super::bevel_bead_clip::Clippable;

/// The beads of one rebuild and whose each is. `Default` is an empty batch: every list empty.
#[derive(Debug, Clone, Default)]
pub struct BevelBeadBatch {
    /// The beads that go on with no trim of their own, fill patches last.
    pub whole: Vec<Mesh>,
    /// The beads that brought a void wedge, each to be trimmed to what it may occupy.
    pub clippable: Vec<Clippable>,
    /// Whose each entry of [`BevelBeadBatch::whole`] is, in step with it.
    pub whole_owners: Vec<Owner>,
    /// Whose each entry of [`BevelBeadBatch::clippable`] is, in step with it.
    pub clippable_owners: Vec<Owner>,
    /// How many of the trailing entries of [`BevelBeadBatch::whole`] are corner fills - the split
    /// the batch union's middle fallback rung wants, recovered from a count rather than
    /// remembered as an index because withdrawals renumber the list.
    pub fill_patches_in_whole: i32,
}

/// Whose one bead is - a group's, or a corner's.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Owner {
    /// The group whose crease a chain bead was swept along; negative for a corner fill.
    pub group: i32,
    /// The groups whose beads meet at a fill's corner (`BevelCornerPatch::groups`), or `None` on
    /// a chain bead.
    pub adjoining_groups: Option<Vec<i32>>,
}

impl BevelBeadBatch {
    /// The beads that go on whole, chain beads first and fill patches after.
    pub fn whole(&self) -> &[Mesh] {
        &self.whole
    }

    /// The beads trimmed to their void wedge before they join.
    pub fn clippable(&self) -> &[Clippable] {
        &self.clippable
    }

    /// Where the fill patches start among the whole beads.
    pub fn fill_patches_start_at(&self) -> i32 {
        self.whole.len() as i32 - self.fill_patches_in_whole
    }

    /// The whole beads, for the clip to trim in place.
    pub fn whole_for_clipping(&mut self) -> &mut Vec<Mesh> {
        &mut self.whole
    }

    /// Adds a chain bead that goes on whole, owned by `group`.
    pub fn add_chain_bead(&mut self, group: i32, bead: Mesh) {
        self.whole.push(bead);
        self.whole_owners.push(Owner::new(group, None));
    }

    /// Adds a chain bead to be trimmed to its void wedge, owned by `group`.
    pub fn add_chain_bead_clippable(&mut self, group: i32, bead: Clippable) {
        self.clippable.push(bead);
        self.clippable_owners.push(Owner::new(group, None));
    }

    /// Adds a corner fill that goes on whole, owned by the groups whose beads adjoin it.
    pub fn add_fill_patch(&mut self, adjoining_groups: Vec<i32>, patch: Mesh) {
        self.whole.push(patch);
        self.whole_owners
            .push(Owner::new(-1, Some(adjoining_groups)));
        self.fill_patches_in_whole += 1;
    }

    /// Adds a corner fill to be trimmed to its void wedge.
    pub fn add_fill_patch_clippable(&mut self, adjoining_groups: Vec<i32>, patch: Clippable) {
        self.clippable.push(patch);
        self.clippable_owners
            .push(Owner::new(-1, Some(adjoining_groups)));
    }

    /// Takes back every bead a rebuilt group owns, and every fill patch adjoining one.
    pub fn withdraw(&mut self, rebuilt_groups: &HashSet<i32>) {
        for i in (0..self.whole.len()).rev() {
            if self.whole_owners[i].withdrawn_by(rebuilt_groups) {
                if self.whole_owners[i].is_fill_patch() {
                    self.fill_patches_in_whole -= 1;
                }

                self.whole.remove(i);
                self.whole_owners.remove(i);
            }
        }

        for i in (0..self.clippable.len()).rev() {
            if self.clippable_owners[i].withdrawn_by(rebuilt_groups) {
                self.clippable.remove(i);
                self.clippable_owners.remove(i);
            }
        }
    }
}

impl Owner {
    /// A chain bead's group, or -1 and the adjoining groups for a fill patch.
    pub fn new(group: i32, adjoining_groups: Option<Vec<i32>>) -> Self {
        Self {
            group,
            adjoining_groups,
        }
    }

    /// True for a corner fill patch, which no single group owns.
    pub fn is_fill_patch(&self) -> bool {
        self.group < 0
    }

    /// Whether rebuilding `rebuilt_groups` withdraws this bead: its own group's, or for a fill
    /// patch any adjoining group's.
    pub fn withdrawn_by(&self, rebuilt_groups: &HashSet<i32>) -> bool {
        if !self.is_fill_patch() {
            return rebuilt_groups.contains(&self.group);
        }

        self.adjoining_groups
            .as_ref()
            .is_some_and(|groups| groups.iter().any(|group| rebuilt_groups.contains(group)))
    }
}
