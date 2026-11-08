//! The chain sweep's two rulers: the sharpest joint a mitre may stand at and the fewest segments a
//! closed run may have. The sweep's sections, joint rings and segments are its sibling files
//! ([`super::bevel_chain_sweep_sections`], [`super::bevel_chain_sweep_rings`],
//! [`super::bevel_chain_sweep_mesh`]).

/// The least cosine a joint's mitre may stand at against either segment before the joint is too
/// sharp to mitre.
pub const MINIMUM_MITER_COSINE: f64 = 0.1;

/// The fewest segments a closed run may have.
pub const MINIMUM_CLOSED_SEGMENTS: i32 = 3;
