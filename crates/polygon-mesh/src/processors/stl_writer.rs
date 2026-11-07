//! STL writing (ASCII and binary) and the six-decimal number format ASCII STL uses;
//! `stl_processing` is the load half.

use std::io::Write;
use std::path::Path;

use vector_math::{Vector3, Vector3Float};

use crate::cancel::CancelToken;
use crate::mesh::Mesh;
use crate::processors::mesh_output_settings::{MeshOutputSettings, OutputType};
use crate::processors::stl_processing::StlError;

/// Writes `mesh` to a new file at `file_name`.
pub fn save_file(
    mesh: &Mesh,
    file_name: &Path,
    cancel: &CancelToken,
    output_info: &MeshOutputSettings,
) -> Result<(), StlError> {
    let file = std::fs::File::create(file_name).map_err(|e| StlError::Io(e.to_string()))?;
    let mut writer = std::io::BufWriter::new(file);
    save(mesh, &mut writer, cancel, output_info)?;
    writer.flush().map_err(|e| StlError::Io(e.to_string()))
}

/// Writes `mesh` as an ASCII or binary STL. Cancellation is polled before every face; a
/// cancelled save returns [`StlError::Cancelled`] with whatever was already written.
pub fn save<W: Write>(
    mesh: &Mesh,
    writer: &mut W,
    cancel: &CancelToken,
    output_info: &MeshOutputSettings,
) -> Result<(), StlError> {
    let io = |e: std::io::Error| StlError::Io(e.to_string());
    match output_info.output_type_setting {
        OutputType::Ascii => {
            // Lines always end with "\n", on every platform: one output for one mesh, and
            // every STL reader accepts it.
            writeln!(writer, "solid Default").map_err(io)?;
            for face in mesh.faces.iter() {
                if cancel.is_cancelled() {
                    return Err(StlError::Cancelled);
                }
                let mut text = String::new();
                text.push_str("  facet normal ");
                text.push_str(&format_for_stl_f32(face.normal));
                text.push_str("\n    outer loop\n");
                for v in [face.v0, face.v1, face.v2] {
                    text.push_str("      vertex ");
                    text.push_str(&format_for_stl_f32(mesh.vertices[v as usize]));
                    text.push('\n');
                }
                text.push_str("    endloop\n  endfacet\n");
                writer.write_all(text.as_bytes()).map_err(io)?;
            }
            writeln!(writer, "endsolid Default").map_err(io)?;
        }
        OutputType::Binary => {
            // 80 bytes of nothing
            writer.write_all(&[0u8; 80]).map_err(io)?;
            // the number of triangles (the face count, written up front so the writer never
            // has to seek back)
            writer
                .write_all(&(mesh.faces.len() as i32).to_le_bytes())
                .map_err(io)?;
            let mut record = Vec::with_capacity(50);
            for face in mesh.faces.iter() {
                if cancel.is_cancelled() {
                    return Err(StlError::Cancelled);
                }
                record.clear();
                for p in [
                    face.normal,
                    mesh.vertices[face.v0 as usize],
                    mesh.vertices[face.v1 as usize],
                    mesh.vertices[face.v2 as usize],
                ] {
                    record.extend_from_slice(&p.x.to_le_bytes());
                    record.extend_from_slice(&p.y.to_le_bytes());
                    record.extend_from_slice(&p.z.to_le_bytes());
                }
                // and the attribute
                record.extend_from_slice(&0u16.to_le_bytes());
                writer.write_all(&record).map_err(io)?;
            }
        }
    }
    writer.flush().map_err(io)
}

/// "x y z" with six decimals, for doubles.
pub fn format_for_stl(value: Vector3) -> String {
    format!(
        "{} {} {}",
        format_fixed6(value.x, 15),
        format_fixed6(value.y, 15),
        format_fixed6(value.z, 15)
    )
}

/// "x y z" with six decimals, for floats.
pub fn format_for_stl_f32(value: Vector3Float) -> String {
    format!(
        "{} {} {}",
        format_fixed6(f64::from(value.x), 7),
        format_fixed6(f64::from(value.y), 7),
        format_fixed6(f64::from(value.z), 7)
    )
}

/// The fixed "0.000000" format: the value is first rounded to `precision` significant digits
/// (7 for float, 15 for double: the digits each type holds reliably), then that digit string
/// is rounded half-up to six decimals, so a float's noise digits never reach the file. A
/// negative value keeps its sign even when it rounds to zero.
pub(crate) fn format_fixed6(value: f64, precision: usize) -> String {
    if value.is_nan() {
        return "NaN".to_string();
    }
    if value.is_infinite() {
        return if value > 0.0 { "∞" } else { "-∞" }.to_string();
    }
    let negative = value.is_sign_negative();
    // Exact decimal digits (Rust prints the exact binary value to any precision), then
    // round half-up to `precision` significant digits.
    let sci = format!("{:.40e}", value.abs());
    let (mantissa, exponent) = sci.split_once('e').unwrap_or((&sci, "0"));
    let mut exponent: i32 = exponent.parse().unwrap_or(0);
    let all_digits: Vec<u8> = mantissa
        .bytes()
        .filter(u8::is_ascii_digit)
        .map(|b| b - b'0')
        .collect();
    let mut digits = round_half_up(&all_digits, precision, &mut exponent);
    if digits.iter().all(|&d| d == 0) {
        exponent = 0;
    }

    // digits[0] is the 10^exponent place; keep through the 10^-6 place.
    let keep = exponent + 1 + 6;
    let mut int_part = String::new();

    let frac: String = if keep <= 0 {
        // Everything is below the sixth decimal; round on the first digit's position.
        let rounds_up = keep == 0 && digits[0] >= 5;
        int_part.push('0');
        if rounds_up {
            "000001".to_string()
        } else {
            "000000".to_string()
        }
    } else {
        let mut exp = exponent;
        digits = round_half_up(&digits, keep as usize, &mut exp);
        // Lay the digits out on a place-value grid from max(exp, 0) down to -6.
        let top = exp.max(0);
        let mut text = String::new();
        for place in (-6..=top).rev() {
            let index = exp - place;
            let digit = if index >= 0 && (index as usize) < digits.len() {
                digits[index as usize]
            } else {
                0
            };
            text.push((b'0' + digit) as char);
            if place == 0 {
                text.push('.');
            }
        }
        let (i, f) = text.split_once('.').unwrap_or((&text, ""));
        int_part.push_str(i);
        f.to_string()
    };
    format!("{}{}.{}", if negative { "-" } else { "" }, int_part, frac)
}

/// The first `count` of `digits`, rounded half-up on the next digit; a carry out of the first
/// digit bumps `exponent`.
fn round_half_up(digits: &[u8], count: usize, exponent: &mut i32) -> Vec<u8> {
    let mut kept: Vec<u8> = digits.iter().copied().take(count).collect();
    while kept.len() < count {
        kept.push(0);
    }
    if digits.get(count).is_some_and(|&d| d >= 5) {
        let mut i = kept.len();
        loop {
            if i == 0 {
                kept.insert(0, 1);
                kept.pop();
                *exponent += 1;
                break;
            }
            i -= 1;
            if kept[i] == 9 {
                kept[i] = 0;
            } else {
                kept[i] += 1;
                break;
            }
        }
    }
    kept
}

#[cfg(test)]
#[path = "stl_writer_tests.rs"]
mod tests;
