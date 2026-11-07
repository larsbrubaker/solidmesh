//! Number text for the vector types' text form and everything that writes numbers into saved
//! files, expressions and labels: the default double/float text (shortest round-trip, general
//! layout), 17 significant digits ("G17", object-reference literals), fixed, grouped and
//! percent text ("F"/"N"/"P", the bevel meters' reports), the `"0.####"` layout vectors print
//! with, the scientific `"0.###e0"`, and the lenient float parser. Saved designs hold numbers in
//! exactly these forms, so the text must not drift.
//!
//! All text is culture-invariant: "NaN", "Infinity", "-Infinity", '.' as the decimal point.

/// The default text of a double: the shortest digits that round-trip, written fixed-point
/// unless the decimal exponent is >= max(digit count, 15) or < -4, in which case it is
/// `d.dddE+XX`. Negative zero prints "-0".
pub fn format_f64(value: f64) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    let shortest = format!("{:e}", value.abs());
    // The shortest digits must be the correctly rounded ones, half to even at an exact tie
    // ("-159.60000610351562", "-8.847000122070312"); Rust's shortest formatter rounds such a tie
    // up. `{:.Ne}` is exact with ties to even; the correctly rounded digits of the same length
    // are at least as close as the shortest ones, so they round-trip too (checked, to be safe
    // at a power-of-two boundary, where the round-trip interval is lopsided).
    let length = split_scientific(&shortest).0.len();
    let rounded = format!("{:.*e}", length - 1, value.abs());
    let digits = if rounded.parse::<f64>() == Ok(value.abs()) {
        rounded
    } else {
        shortest
    };
    general_layout(&digits, value.is_sign_negative(), 15)
}

/// "G17" text: 17 significant digits, correctly rounded,
/// trailing zeros dropped, fixed-point unless the decimal exponent is >= 17 or < -4.
pub fn format_f64_g17(value: f64) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    general_layout(
        &format!("{:.16e}", value.abs()),
        value.is_sign_negative(),
        17,
    )
}

/// The default text of a float: as [`format_f64`] with the shortest `f32`
/// round-trip digits and a 7-digit fixed-point threshold.
pub fn format_f32(value: f32) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    general_layout(&format!("{:e}", value.abs()), value.is_sign_negative(), 7)
}

/// The custom format `"0.###…"` with `max_decimals` `#`s. The value is first reduced to 15
/// significant digits (round-half-even on the exact value), then rounded half away from zero
/// to `max_decimals`; trailing fractional zeros and a bare '.' are dropped. A negative value
/// that rounds to zero keeps its sign ("-0").
pub fn format_f64_max_decimals(value: f64, max_decimals: usize) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    let negative = value.is_sign_negative();
    let (digits, exponent) = split_scientific(&format!("{:.14e}", value.abs()));
    // Decimal digits as (integer part, fraction part) of the 15-significant-digit value.
    let point = exponent + 1;
    let (mut int_part, frac_part): (Vec<u8>, Vec<u8>) = if point <= 0 {
        let mut frac = vec![b'0'; (-point) as usize];
        frac.extend_from_slice(&digits);
        (Vec::new(), frac)
    } else if point as usize >= digits.len() {
        let mut int = digits.clone();
        int.resize(point as usize, b'0');
        (int, Vec::new())
    } else {
        (
            digits[..point as usize].to_vec(),
            digits[point as usize..].to_vec(),
        )
    };
    let mut kept: Vec<u8> = frac_part.iter().take(max_decimals).copied().collect();
    let round_up = frac_part.get(max_decimals).is_some_and(|&d| d >= b'5');
    if round_up {
        // Carry through the kept fraction, then the integer part.
        let mut carry = true;
        for d in kept.iter_mut().rev() {
            if *d == b'9' {
                *d = b'0';
            } else {
                *d += 1;
                carry = false;
                break;
            }
        }
        if carry {
            for d in int_part.iter_mut().rev() {
                if *d == b'9' {
                    *d = b'0';
                } else {
                    *d += 1;
                    carry = false;
                    break;
                }
            }
            if carry {
                int_part.insert(0, b'1');
            }
        }
    }
    while kept.last() == Some(&b'0') {
        kept.pop();
    }
    while int_part.len() > 1 && int_part[0] == b'0' {
        int_part.remove(0);
    }
    let mut text = String::new();
    if negative {
        text.push('-');
    }
    if int_part.is_empty() {
        text.push('0');
    } else {
        text.push_str(&String::from_utf8_lossy(&int_part));
    }
    if !kept.is_empty() {
        text.push('.');
        text.push_str(&String::from_utf8_lossy(&kept));
    }
    text
}

/// The parsed value, or 0 where [`try_parse_f64`] fails.
pub fn parse_f64(text: &str) -> f64 {
    try_parse_f64(text).unwrap_or(0.0)
}

/// A culture-invariant float, or `None` where the text is not one. Accepts surrounding white
/// space (U+0009–U+000D, U+0020), a leading sign, a decimal point, an exponent, and the
/// symbols "NaN", "Infinity", "-Infinity" (case-insensitive; not the "∞" sign). No thousands
/// separators, so "12,5" is `None` rather than a misread 125.
pub fn try_parse_f64(text: &str) -> Option<f64> {
    let trimmed = text.trim_matches(|c: char| ('\u{9}'..='\u{d}').contains(&c) || c == ' ');
    let (negative, body) = match trimmed.as_bytes().first() {
        Some(b'-') => (true, &trimmed[1..]),
        Some(b'+') => (false, &trimmed[1..]),
        _ => (false, trimmed),
    };
    let lower = body.to_ascii_lowercase();
    let magnitude = if lower == "infinity" {
        f64::INFINITY
    } else if lower == "nan" {
        f64::NAN
    } else if body
        .bytes()
        .all(|b| b.is_ascii_digit() || matches!(b, b'.' | b'e' | b'E' | b'+' | b'-'))
        && body
            .bytes()
            .next()
            .is_some_and(|b| b.is_ascii_digit() || b == b'.')
    {
        // Rust's float grammar accepts every form saved text uses ("1.", ".5", "1e5", "1E+05").
        body.parse::<f64>().ok()?
    } else {
        return None;
    };
    Some(if negative { -magnitude } else { magnitude })
}

/// `"F<decimals>"` text: fixed-point with `decimals` places, rounded from the EXACT binary value
/// with ties to even (`2.675` is "2.67", `0.125` is "0.12"), no grouping; a negative value that
/// rounds to zero keeps its sign.
pub fn format_f64_fixed(value: f64, decimals: usize) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    format!("{value:.decimals$}")
}

/// `"N<decimals>"` text: [`format_f64_fixed`] with the integer
/// part grouped in threes by ','.
pub fn format_f64_number(value: f64, decimals: usize) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    let text = format!("{:.decimals$}", value.abs());
    let (whole, fraction) = text.split_once('.').unwrap_or((&text, ""));
    signed_grouped(value.is_sign_negative(), whole, fraction, "")
}

/// "N0" text of an integer: the count grouped in threes by ','.
pub fn format_i64_number(value: i64) -> String {
    signed_grouped(value < 0, &value.unsigned_abs().to_string(), "", "")
}

/// `"P<decimals>"` text: the value times 100 - taken off the
/// exact decimal expansion by moving the point, never by a floating multiply (`0.0135` is
/// "1.3 %", though `0.0135 * 100` is 1.35000000000000008) - grouped like "N", then " %"
/// (the invariant pattern "n %" / "-n %"). No value can tie at the rounding digit (a tie
/// would need a fifth power in a binary fraction), so ties are moot.
pub fn format_f64_percent(value: f64, decimals: usize) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    let places = decimals + 2;
    let text = format!("{:.places$}", value.abs());
    let (whole, fraction) = text.split_once('.').unwrap_or((&text, ""));
    let shifted = format!("{whole}{}", &fraction[..2]);
    let shifted = shifted.trim_start_matches('0');
    let shifted = if shifted.is_empty() { "0" } else { shifted };
    signed_grouped(value.is_sign_negative(), shifted, &fraction[2..], " %")
}

/// `whole` grouped in threes by ',', then `.fraction` where there is one, then `suffix`, with a
/// leading '-' where `negative`.
fn signed_grouped(negative: bool, whole: &str, fraction: &str, suffix: &str) -> String {
    let mut text = String::with_capacity(whole.len() + whole.len() / 3 + fraction.len() + 4);
    if negative {
        text.push('-');
    }
    for (i, digit) in whole.chars().enumerate() {
        if i > 0 && (whole.len() - i).is_multiple_of(3) {
            text.push(',');
        }
        text.push(digit);
    }
    if !fraction.is_empty() {
        text.push('.');
        text.push_str(fraction);
    }
    text.push_str(suffix);
    text
}

/// The custom scientific format `"0.<max_decimals #>e0"`, e.g. `"0.###e0"`:
/// one integer digit, up to `max_decimals` fractional digits with trailing zeros (and a bare '.')
/// dropped, and an exponent of at least one digit with only a minus sign ("1.235e-4", "1e1",
/// "0e0"). The value is first reduced to 15 significant digits (round-half-even on the
/// exact value), then rounded half away from zero; a carry past 9 moves the exponent ("9.9995" is
/// "1e1"). Zero (either sign) prints "0e0".
pub fn format_f64_scientific_e0(value: f64, max_decimals: usize) -> String {
    if let Some(special) = special_text(value.is_nan(), value.is_infinite(), value < 0.0) {
        return special;
    }
    if value == 0.0 {
        return "0e0".to_string();
    }

    let (mut digits, mut exponent) = split_scientific(&format!("{:.14e}", value.abs()));
    let keep = 1 + max_decimals;
    if digits.len() > keep {
        let round_up = digits[keep] >= b'5';
        digits.truncate(keep);
        if round_up {
            let mut i = keep;
            loop {
                if i == 0 {
                    digits.insert(0, b'1');
                    digits.pop();
                    exponent += 1;
                    break;
                }
                i -= 1;
                if digits[i] == b'9' {
                    digits[i] = b'0';
                } else {
                    digits[i] += 1;
                    break;
                }
            }
        }
    }
    while digits.len() > 1 && digits.last() == Some(&b'0') {
        digits.pop();
    }

    let mut text = String::new();
    if value < 0.0 {
        text.push('-');
    }
    text.push(digits[0] as char);
    if digits.len() > 1 {
        text.push('.');
        text.extend(digits[1..].iter().map(|&d| d as char));
    }
    text.push('e');
    text.push_str(&exponent.to_string());
    text
}

fn special_text(is_nan: bool, is_infinite: bool, is_negative: bool) -> Option<String> {
    if is_nan {
        Some("NaN".to_string())
    } else if is_infinite {
        Some(if is_negative { "-Infinity" } else { "Infinity" }.to_string())
    } else {
        None
    }
}

/// Splits Rust's `{:e}` text ("1.2345e-7", "5e0") into its digits (no '.') and exponent.
fn split_scientific(text: &str) -> (Vec<u8>, i32) {
    let (mantissa, exponent) = text.split_once('e').unwrap_or((text, "0"));
    let digits: Vec<u8> = mantissa.bytes().filter(|b| *b != b'.').collect();
    (digits, exponent.parse().unwrap_or(0))
}

/// The general ("G") layout for shortest round-trip digits.
fn general_layout(scientific: &str, negative: bool, min_precision: usize) -> String {
    let (mut digits, exponent) = split_scientific(scientific);
    while digits.len() > 1 && digits.last() == Some(&b'0') {
        digits.pop();
    }
    let mut text = String::new();
    if negative {
        text.push('-');
    }
    if digits == b"0" {
        text.push('0');
        return text;
    }
    let max_digits = digits.len().max(min_precision) as i32;
    let digits_text = String::from_utf8_lossy(&digits).into_owned();
    if exponent >= max_digits || exponent < -4 {
        text.push_str(&digits_text[..1]);
        if digits.len() > 1 {
            text.push('.');
            text.push_str(&digits_text[1..]);
        }
        text.push('E');
        text.push(if exponent < 0 { '-' } else { '+' });
        text.push_str(&format!("{:02}", exponent.abs()));
    } else if exponent < 0 {
        text.push_str("0.");
        for _ in 0..(-exponent - 1) {
            text.push('0');
        }
        text.push_str(&digits_text);
    } else {
        let point = exponent as usize + 1;
        if digits.len() <= point {
            text.push_str(&digits_text);
            for _ in digits.len()..point {
                text.push('0');
            }
        } else {
            text.push_str(&digits_text[..point]);
            text.push('.');
            text.push_str(&digits_text[point..]);
        }
    }
    text
}

#[cfg(test)]
mod tests;
