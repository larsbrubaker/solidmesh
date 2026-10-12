//! Port of `Fixtures/SurfaceTestChains.cs`: the one way a test turns linked chains into the chains a
//! plan is made from - the same two steps, in the same order, that `BevelSurfacePartBuilder` takes.
//! A test that joined without opening was asking the kernel for a shape production can no longer
//! hand it, so every kernel test comes through here instead.

// The kernel tests that join through here land with the slices their construction needs.
#![allow(dead_code)]

use geometry_processing::bevel::surface::{
    surface_chain_join, surface_chain_opens_at_the_junction,
};
use geometry_processing::bevel::BevelSweptChain;

/// C# `JoinAndOpen`: chains joined end to end, then cut open at every junction vertex they pass
/// through - exactly `SurfaceChainOpensAtTheJunction.Open(SurfaceChainJoin.Join(chains))`, which
/// is what the builder does.
pub fn join_and_open(chains: &[BevelSweptChain]) -> Vec<BevelSweptChain> {
    surface_chain_opens_at_the_junction::open(surface_chain_join::join(chains))
}
