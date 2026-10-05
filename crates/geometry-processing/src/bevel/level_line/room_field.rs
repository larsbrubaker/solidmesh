//! The one field a room reading asks ([`RoomField`]): a single chain's, or a side group's
//! [`PickField`] over several. Ports the type of `Bevel/LevelLine/RoomField.cs`; its readings and
//! conversions land with the room-reading method package.

use std::rc::Rc;

use super::{ChainField, PickField};

/// C# `RoomField`: the one field a room reading asks - a single chain's, or a side group's
/// [`PickField`] over several. Only what the self-room reading needs, so a group's side is read by
/// the very code a single chain's side is. Exactly one of the two is set.
#[derive(Debug, Clone, Default)]
pub struct RoomField {
    /// The single chain's field, or `None` when this reads a group.
    pub one: Option<Rc<ChainField>>,
    /// The side group's field, or `None` when this reads a single chain.
    pub several: Option<Rc<PickField>>,
}
