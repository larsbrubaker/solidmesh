//! The level-line meters' clock: C#'s `Stopwatch.GetTimestamp` and `Stopwatch.Frequency`, as the
//! diagnostic meters (`FieldCounters`, `NeedleMeter`, `SplitMeter`, `ForeignRoomCounters`) and
//! the work budget's phase tally read them. Native builds count nanoseconds since the first
//! reading; `wasm32-unknown-unknown` has no clock std can read, so there every timestamp is 0 and
//! every metered time reads zero (`docs/CSHARP_DIVERGENCES.md` 653). Also the number formats
//! the meters write their reports in, C#'s `N0` ([`n0`]), `G4` ([`g4`]) and `E3` ([`e3`]).

/// Ticks per second of [`get_timestamp`] (C# `Stopwatch.Frequency`).
pub const FREQUENCY: i64 = 1_000_000_000;

/// The current timestamp in ticks (C# `Stopwatch.GetTimestamp`): never 0 natively, so a meter
/// may keep 0 as "not running" the way the C# does.
#[cfg(not(target_arch = "wasm32"))]
pub fn get_timestamp() -> i64 {
    use std::sync::OnceLock;
    use std::time::Instant;

    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    let origin = *ORIGIN.get_or_init(Instant::now);
    // Saturating: i64 nanoseconds run out after 292 years of process time.
    1 + i64::try_from(origin.elapsed().as_nanos()).unwrap_or(i64::MAX - 1)
}

/// The current timestamp in ticks: always 0 in the browser, where std has no clock (divergence
/// 653), so the meters' clocks never start there.
#[cfg(target_arch = "wasm32")]
pub fn get_timestamp() -> i64 {
    0
}

/// Ticks as seconds (C# `FieldCounters.Seconds`, `ticks / Stopwatch.Frequency`).
pub fn seconds(ticks: i64) -> f64 {
    ticks as f64 / FREQUENCY as f64
}

/// A count with thousands separators, as C#'s `N0` format writes it (invariant/en-US grouping).
pub fn n0(value: i64) -> String {
    let digits = value.unsigned_abs().to_string();
    let mut grouped = String::with_capacity(digits.len() + digits.len() / 3 + 1);
    if value < 0 {
        grouped.push('-');
    }

    for (i, digit) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            grouped.push(',');
        }

        grouped.push(digit);
    }

    grouped
}

/// A number as C#'s `G4` format writes it: four significant digits, trailing zeros dropped,
/// scientific (`1.235E+05`, two exponent digits at least) when the exponent is below -5 or at
/// least 4; `∞`, `-∞` and `NaN` as .NET Core writes them.
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

    // Rust's `{:e}` rounds the exact binary value correctly, as .NET Core 3+ does.
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

/// A number as C#'s `E3` format writes it: one digit, three decimals, `E`, a sign and three
/// exponent digits at least (`1.235E+004`); `∞`, `-∞` and `NaN` as .NET Core writes them.
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
