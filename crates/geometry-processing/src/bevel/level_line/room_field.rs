//! The one field a room reading asks ([`RoomField`]): a single chain's, or a side group's
//! [`PickField`] over several. Ports the type of `Bevel/LevelLine/RoomField.cs` with its
//! constructors, setback readings and the conversion from a chain's field; its field readings land
//! with the room-reading method package.

use std::rc::Rc;

use super::chain_setback;
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

impl RoomField {
    /// One chain's field.
    pub fn from_chain(field: Rc<ChainField>) -> Self {
        Self {
            one: Some(field),
            several: None,
        }
    }

    /// A side group's field over several chains.
    pub fn from_pick(field: Rc<PickField>) -> Self {
        Self {
            one: None,
            several: Some(field),
        }
    }

    /// The smallest setback the field asks for. Panics on an empty room field, where C# reads
    /// through a null.
    pub fn min_setback(&self) -> f64 {
        match (&self.one, &self.several) {
            (Some(one), _) => one.min_setback,
            (None, Some(several)) => several.min_setback,
            (None, None) => panic!("a room field holds a chain's or a group's field"),
        }
    }

    /// The largest setback the field asks for. Panics on an empty room field, as
    /// [`RoomField::min_setback`] does.
    pub fn max_setback(&self) -> f64 {
        match (&self.one, &self.several) {
            (Some(one), _) => one.max_setback,
            (None, Some(several)) => several.max_setback,
            (None, None) => panic!("a room field holds a chain's or a group's field"),
        }
    }

    /// Whether the setback changes over the field.
    pub fn setback_varies(&self) -> bool {
        !chain_setback::is_uniform(self.min_setback(), self.max_setback())
    }
}

/// C#'s `implicit operator RoomField(ChainField)`.
impl From<Rc<ChainField>> for RoomField {
    fn from(field: Rc<ChainField>) -> Self {
        Self::from_chain(field)
    }
}
