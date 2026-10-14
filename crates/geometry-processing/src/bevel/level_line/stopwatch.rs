//! The level-line meters' clock: a timestamp and its tick rate, as the
//! diagnostic meters (`FieldCounters`, `NeedleMeter`, `SplitMeter`, `ForeignRoomCounters`) and
//! the work budget's phase tally and progress throttle read them: nanoseconds since the first
//! reading, from `web_time::Instant`, which reads `performance.now()` in the browser and the
//! system's monotonic clock natively. Also the number formats the meters write their reports in,
//! `G4` ([`g4`]) and `E3` ([`e3`]); `N0` is `vector_math::number_format::format_i64_number`.

use std::sync::OnceLock;

use web_time::Instant;

/// Ticks per second of [`get_timestamp`].
pub const FREQUENCY: i64 = 1_000_000_000;

/// The current timestamp in ticks: never 0, so a meter may keep 0 as "not running".
pub fn get_timestamp() -> i64 {
    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    let origin = *ORIGIN.get_or_init(Instant::now);
    // Saturating: i64 nanoseconds run out after 292 years of process time.
    1 + i64::try_from(origin.elapsed().as_nanos()).unwrap_or(i64::MAX - 1)
}

/// Ticks as seconds (`ticks / FREQUENCY`).
pub fn seconds(ticks: i64) -> f64 {
    ticks as f64 / FREQUENCY as f64
}

/// A number in the `G4` format: four significant digits, trailing zeros dropped,
/// scientific (`1.235E+05`, two exponent digits at least) when the exponent is below -5 or at
/// least 4; `∞`, `-∞` and `NaN` for the values that are not finite.
pub fn g4(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_string();
    }

    if value.is_infinite() {
        return if value > 0.0 { "∞" } else { "-∞" }.to_string();
    }

    if value == 0.0 {
        return "0".to_string();
    }

    // Rust's `{:e}` rounds the exact binary value correctly, so the digits are the true ones.
    let scientific = format!("{value:.3e}");
    let (mantissa, exponent) = scientific.split_once('e').unwrap_or((&scientific, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let negative = mantissa.starts_with('-');
    let digits: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    let digits = digits.trim_end_matches('0');
    let digits = if digits.is_empty() { "0" } else { digits };
    let sign = if negative { "-" } else { "" };

    if !(-5..4).contains(&exponent) {
        let (lead, rest) = digits.split_at(1);
        let fraction = if rest.is_empty() {
            String::new()
        } else {
            format!(".{rest}")
        };
        let exponent_sign = if exponent < 0 { '-' } else { '+' };
        return format!(
            "{sign}{lead}{fraction}E{exponent_sign}{:02}",
            exponent.abs()
        );
    }

    let point = exponent + 1;
    let text = if point <= 0 {
        format!("0.{}{digits}", "0".repeat((-point) as usize))
    } else if point as usize >= digits.len() {
        format!("{digits}{}", "0".repeat(point as usize - digits.len()))
    } else {
        let (whole, fraction) = digits.split_at(point as usize);
        format!("{whole}.{fraction}")
    };
    format!("{sign}{text}")
}

/// A number in the `E3` format: one digit, three decimals, `E`, a sign and three
/// exponent digits at least (`1.235E+004`); `∞`, `-∞` and `NaN` for the values that are not finite.
pub fn e3(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_string();
    }

    if value.is_infinite() {
        return if value > 0.0 { "∞" } else { "-∞" }.to_string();
    }

    let scientific = format!("{value:.3e}");
    let (mantissa, exponent) = scientific.split_once('e').unwrap_or((&scientific, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let exponent_sign = if exponent < 0 { '-' } else { '+' };
    format!("{mantissa}E{exponent_sign}{:03}", exponent.abs())
}
