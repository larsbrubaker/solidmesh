//! `Color`'s JSON shape, shared by theme files, the themeset and `.mcx` (`Object3D.Color`).
//! Writing: C# `ColorJsonConverter.CanWrite` is false, so Newtonsoft falls back to the
//! `[TypeConverter(typeof(ColorTypeConverter))]` string contract, whose `ConvertTo(string)` is
//! `ToString()` = `GetAsHTMLString()`: `"#RRGGBB"` when opaque, `"#RRGGBBAA"` otherwise (the
//! form MatterCAD's theme files and test scenes hold). Reading ports
//! `ColorJsonConverter.ReadJson`: a string through the `Html` setter (empty or unrecognized is
//! transparent), an object through its `Html` property, and null or anything else transparent.

use std::fmt;

use serde::de::{self, IgnoredAny, MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use super::color8::Color;

impl Serialize for Color {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.get_as_html_string())
    }
}

struct ColorVisitor;

impl<'de> Visitor<'de> for ColorVisitor {
    type Value = Color;

    fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("an HTML colour string or an object with an Html property")
    }

    fn visit_str<E: de::Error>(self, v: &str) -> Result<Color, E> {
        // The Html setter reads named colors ("red") as well as "#RRGGBB", and leaves the color
        // at its default (transparent) for anything it does not recognize.
        Ok(Color::from_html_or_transparent(v))
    }

    fn visit_unit<E: de::Error>(self) -> Result<Color, E> {
        Ok(Color::TRANSPARENT)
    }

    fn visit_none<E: de::Error>(self) -> Result<Color, E> {
        Ok(Color::TRANSPARENT)
    }

    fn visit_some<D: Deserializer<'de>>(self, d: D) -> Result<Color, D::Error> {
        d.deserialize_any(ColorVisitor)
    }

    fn visit_bool<E: de::Error>(self, _: bool) -> Result<Color, E> {
        Ok(Color::TRANSPARENT)
    }

    fn visit_i64<E: de::Error>(self, _: i64) -> Result<Color, E> {
        Ok(Color::TRANSPARENT)
    }

    fn visit_u64<E: de::Error>(self, _: u64) -> Result<Color, E> {
        Ok(Color::TRANSPARENT)
    }

    fn visit_f64<E: de::Error>(self, _: f64) -> Result<Color, E> {
        Ok(Color::TRANSPARENT)
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Color, A::Error> {
        // C# reader.Skip(): consume the whole container.
        while seq.next_element::<IgnoredAny>()?.is_some() {}
        Ok(Color::TRANSPARENT)
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Color, A::Error> {
        // JObject["Html"] is the last "Html" property; its string value, if any, is parsed.
        let mut html: Option<Option<String>> = None;
        while let Some(key) = map.next_key::<String>()? {
            if key == "Html" {
                html = Some(map.next_value::<HtmlValue>()?.0);
            } else {
                map.next_value::<IgnoredAny>()?;
            }
        }
        Ok(match html {
            Some(Some(text)) => Color::from_html_or_transparent(&text),
            _ => Color::TRANSPARENT,
        })
    }
}

/// An `Html` property's value: its text when a string, `None` for anything else (C#
/// `Value<string>()` of a number or bool never parses as a colour either).
struct HtmlValue(Option<String>);

impl<'de> Deserialize<'de> for HtmlValue {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let value = string_or_skip(d)?;
        Ok(HtmlValue(value))
    }
}

/// Reads any value, keeping it only when it is a string.
fn string_or_skip<'de, D: Deserializer<'de>>(d: D) -> Result<Option<String>, D::Error> {
    struct StringOrSkip;
    impl<'de> Visitor<'de> for StringOrSkip {
        type Value = Option<String>;
        fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
            f.write_str("any value")
        }
        fn visit_str<E: de::Error>(self, v: &str) -> Result<Self::Value, E> {
            Ok(Some(v.to_string()))
        }
        fn visit_unit<E: de::Error>(self) -> Result<Self::Value, E> {
            Ok(None)
        }
        fn visit_none<E: de::Error>(self) -> Result<Self::Value, E> {
            Ok(None)
        }
        fn visit_bool<E: de::Error>(self, _: bool) -> Result<Self::Value, E> {
            Ok(None)
        }
        fn visit_i64<E: de::Error>(self, _: i64) -> Result<Self::Value, E> {
            Ok(None)
        }
        fn visit_u64<E: de::Error>(self, _: u64) -> Result<Self::Value, E> {
            Ok(None)
        }
        fn visit_f64<E: de::Error>(self, _: f64) -> Result<Self::Value, E> {
            Ok(None)
        }
        fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
            while seq.next_element::<IgnoredAny>()?.is_some() {}
            Ok(None)
        }
        fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Self::Value, A::Error> {
            while map.next_entry::<IgnoredAny, IgnoredAny>()?.is_some() {}
            Ok(None)
        }
    }
    d.deserialize_any(StringOrSkip)
}

impl<'de> Deserialize<'de> for Color {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(ColorVisitor)
    }
}
