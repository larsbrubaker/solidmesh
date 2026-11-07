//! Colours: a mesh face's colour and the colours callers keep beside their meshes (MatterCAD's
//! themes and its objects' `Color`). [`color8`] is the 8-bit `Color`, [`color_f`] the float
//! `ColorF`, [`color_type`] the `IColorType` trait with `ColorExtensionMethods`, `color_names` the
//! LAB naming helpers and `color_serde` the JSON shape saved files hold. MatterCAD's 2D path crate
//! (agg-path) re-exports this module's items under `agg_path::color`, beside its own OKLab space
//! and candidate-colour engine.

pub mod color8;
pub mod color_f;
mod color_names;
mod color_serde;
pub mod color_type;

pub use color8::Color;
pub use color_f::ColorF;
pub use color_type::ColorType;
