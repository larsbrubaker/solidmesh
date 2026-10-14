//! Which of the two constructions of a bevel builds it: the solid tools and booleans that shipped
//! first, or the direct rewrite of the surface that replaces them.
//!
//! The enum exists so the test ladder can run every rung in BOTH constructions until the new one
//! carries the whole corpus; it and `Boolean` are deleted together at the end of the
//! polygon-space bevel plan, and nothing outside that plan should grow a dependency on being able
//! to choose.
//!
//! Serialized by NAME rather than by ordinal, so a member added in the middle later does not
//! re-interpret the bevels already saved in people's files - the same rule `BevelSelectionKind`
//! carries, for the same reason. serde's default unit-variant form is the variant name, which is
//! what Newtonsoft writes.

use std::fmt;

use serde::{Deserialize, Serialize};

/// Which construction builds a bevel. `Default` is the first member.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum BevelConstructionMode {
    /// Sweep a solid tool per chain - a cutter for a convex edge, a bead for a concave one - and
    /// combine it with the part through the boolean kernel. The construction on its way out: it
    /// is the one that leaves the slivers and self-touching surfaces the kernel cannot be asked
    /// to fix.
    #[default]
    Boolean,

    /// Rewrite the mesh directly - walk each face region back to its tangent line and stitch the
    /// bevel strip between them - the 3D reading of what the 2D bevel does to a contour. No
    /// kernel is called at all, and the result is watertight by construction.
    Surface,
}

impl fmt::Display for BevelConstructionMode {
    /// The member name, the same text the setting is saved as.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BevelConstructionMode::Boolean => f.write_str("Boolean"),
            BevelConstructionMode::Surface => f.write_str("Surface"),
        }
    }
}
