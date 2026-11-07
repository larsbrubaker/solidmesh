//! JSON shapes of `Vector4`, `Matrix4X4` and `Quaternion` as saved JSON holds them: the stored
//! components, then the derived values that are written, in a fixed order.
//! (The vectors with only fields, and `AxisAlignedBoundingBox`, derive theirs in their own
//! files.) MatterCAD's `.mcx` writes matrices through its own converter, in the model.
//!
//! - `Vector4`: `{"X","Y","Z","W","Length","LengthSquared","Xy":{..},"Xyz":{..}}`; reading takes
//!   X/Y/Z/W, then the settable `Xy` and `Xyz` if present (applied after the components, so they
//!   win); the read-only lengths are ignored.
//! - `Matrix4X4`: `{"Row0".."Row3": Vector4, "Translation": Vector4}` (`Translation` is the only
//!   derived value written); reading takes the rows and ignores `Translation`.
//! - `Quaternion`: only its derived values appear, not its stored fields:
//!   `{"Xyz":{..},"X","Y","Z","W","Length","LengthSquared"}`; reading applies `Xyz`, then X, Y,
//!   Z and W.

use crate::{Matrix4X4, Quaternion, Vector2, Vector3, Vector4};
use serde::ser::SerializeStruct;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

impl Serialize for Vector4 {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut s = serializer.serialize_struct("Vector4", 8)?;
        s.serialize_field("X", &self.x)?;
        s.serialize_field("Y", &self.y)?;
        s.serialize_field("Z", &self.z)?;
        s.serialize_field("W", &self.w)?;
        s.serialize_field("Length", &self.length())?;
        s.serialize_field("LengthSquared", &self.length_squared())?;
        s.serialize_field("Xy", &self.xy())?;
        s.serialize_field("Xyz", &self.xyz())?;
        s.end()
    }
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct Vector4Json {
    #[serde(rename = "X")]
    x: f64,
    #[serde(rename = "Y")]
    y: f64,
    #[serde(rename = "Z")]
    z: f64,
    #[serde(rename = "W")]
    w: f64,
    #[serde(rename = "Xy")]
    xy: Option<Vector2>,
    #[serde(rename = "Xyz")]
    xyz: Option<Vector3>,
}

impl<'de> Deserialize<'de> for Vector4 {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let json = Vector4Json::deserialize(deserializer)?;
        let mut result = Vector4::new(json.x, json.y, json.z, json.w);
        if let Some(xy) = json.xy {
            result.set_xy(xy);
        }
        if let Some(xyz) = json.xyz {
            result.set_xyz(xyz);
        }
        Ok(result)
    }
}

impl Serialize for Matrix4X4 {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut s = serializer.serialize_struct("Matrix4X4", 5)?;
        s.serialize_field("Row0", &self.row0)?;
        s.serialize_field("Row1", &self.row1)?;
        s.serialize_field("Row2", &self.row2)?;
        s.serialize_field("Row3", &self.row3)?;
        s.serialize_field("Translation", &self.translation())?;
        s.end()
    }
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct Matrix4X4Json {
    #[serde(rename = "Row0")]
    row0: Vector4,
    #[serde(rename = "Row1")]
    row1: Vector4,
    #[serde(rename = "Row2")]
    row2: Vector4,
    #[serde(rename = "Row3")]
    row3: Vector4,
}

impl<'de> Deserialize<'de> for Matrix4X4 {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let json = Matrix4X4Json::deserialize(deserializer)?;
        Ok(Matrix4X4::from_rows(
            json.row0, json.row1, json.row2, json.row3,
        ))
    }
}

impl Serialize for Quaternion {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut s = serializer.serialize_struct("Quaternion", 7)?;
        s.serialize_field("Xyz", &self.xyz())?;
        s.serialize_field("X", &self.x)?;
        s.serialize_field("Y", &self.y)?;
        s.serialize_field("Z", &self.z)?;
        s.serialize_field("W", &self.w)?;
        s.serialize_field("Length", &self.length())?;
        s.serialize_field("LengthSquared", &self.length_squared())?;
        s.end()
    }
}

#[derive(Deserialize, Default)]
#[serde(default)]
struct QuaternionJson {
    #[serde(rename = "Xyz")]
    xyz: Option<Vector3>,
    #[serde(rename = "X")]
    x: Option<f64>,
    #[serde(rename = "Y")]
    y: Option<f64>,
    #[serde(rename = "Z")]
    z: Option<f64>,
    #[serde(rename = "W")]
    w: Option<f64>,
}

impl<'de> Deserialize<'de> for Quaternion {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let json = QuaternionJson::deserialize(deserializer)?;
        // A member the JSON leaves out stays zero: reading starts from an all-zero quaternion.
        let mut result = Quaternion::default();
        if let Some(xyz) = json.xyz {
            result.set_xyz(xyz);
        }
        if let Some(x) = json.x {
            result.x = x;
        }
        if let Some(y) = json.y {
            result.y = y;
        }
        if let Some(z) = json.z {
            result.z = z;
        }
        if let Some(w) = json.w {
            result.w = w;
        }
        Ok(result)
    }
}
