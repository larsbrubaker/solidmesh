//! The one field a room reading asks ([`RoomField`]): a single chain's, or a side group's
//! [`PickField`] over several. Ports the type of `Bevel/LevelLine/RoomField.cs` with its
//! constructors, setback readings, the conversion from a chain's field and its field readings
//! ([`RoomField::evaluate`], [`RoomField::setback_of`], [`RoomField::max_over_triangle`]); the
//! swallow scale lands with its method package.

use std::rc::Rc;

use vector_math::Vector3;

use super::chain_setback;
use super::{ChainField, FieldMaximum, FieldSample, PickField, WorkError};

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

    /// The chain's or the group's field, whichever this holds. Panics on an empty room field, as
    /// [`RoomField::min_setback`] does.
    fn either(&self) -> Result<&Rc<ChainField>, &Rc<PickField>> {
        match (&self.one, &self.several) {
            (Some(one), _) => Ok(one),
            (None, Some(several)) => Err(several),
            (None, None) => panic!("a room field holds a chain's or a group's field"),
        }
    }

    /// C# `RoomField.Evaluate`: the point query of whichever field this holds.
    pub fn evaluate(&self, point: Vector3) -> Result<FieldSample, WorkError> {
        match self.either() {
            Ok(one) => one.evaluate(point),
            Err(several) => several.evaluate(point),
        }
    }

    /// C# `RoomField.SetbackOf`: the setback a sample stands at - the MINIMISING chain's, where
    /// several are read.
    pub fn setback_of(&self, sample: &FieldSample) -> f64 {
        match self.either() {
            Ok(one) => one.setback_at(sample.parameter),
            Err(several) => several.setback_of(sample),
        }
    }

    /// C# `RoomField.MaxOverTriangle`: the bounded maximum over a triangle of whichever field this
    /// holds.
    pub fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        match self.either() {
            Ok(one) => one.max_over_triangle_bounded(a, b, c, tolerance, stop_above, at_least),
            Err(several) => {
                several.max_over_triangle_bounded(a, b, c, tolerance, stop_above, at_least)
            }
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

impl RoomField {
    /// C# `RoomField.WithSetbacksScaled`: the same room with its chain's - or its group's member
    /// chains' - setbacks times `factor`.
    pub fn with_setbacks_scaled(&self, factor: f64) -> Result<RoomField, WorkError> {
        Ok(match (&self.one, &self.several) {
            (Some(one), _) => RoomField::from_chain(Rc::new(one.with_setbacks_scaled(factor)?)),
            (None, Some(several)) => {
                RoomField::from_pick(Rc::new(several.with_setbacks_scaled(factor)?))
            }
            (None, None) => RoomField::default(),
        })
    }
}
