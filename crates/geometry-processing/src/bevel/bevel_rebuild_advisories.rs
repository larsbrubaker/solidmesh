//! Everything a bevel rebuild has to SAY about itself beyond each group's own row: what repairing
//! the working mesh had to report (`BevelWorkingMesh.CreateFrom`). The repair is made on a copy and
//! never on the design, so this list is the only trace of it the user ever gets; the prologue
//! starts it and the surface part builder publishes it.

use super::BevelAdvisory;

/// The advisories one rebuild has gathered, in the order they were raised.
#[derive(Debug, Clone, Default)]
pub struct BevelRebuildAdvisories {
    messages: Vec<BevelAdvisory>,
}

impl BevelRebuildAdvisories {
    /// Starts from whatever repairing the working mesh had to report (`None` for nothing).
    pub fn new<I: IntoIterator<Item = BevelAdvisory>>(source_advisories: Option<I>) -> Self {
        Self {
            messages: source_advisories
                .map(|advisories| advisories.into_iter().collect())
                .unwrap_or_default(),
        }
    }

    /// What to publish, live: one list for the whole rebuild, so every way out of the builder
    /// publishes whatever had been added by the time it took its way out.
    pub fn messages(&self) -> &[BevelAdvisory] {
        &self.messages
    }
}
