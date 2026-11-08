//! The shared pieces of the demo's sentences (`fillet_words`, `fillet_words_blend`): where a
//! position is said (`describe`), the argument readers, the shared held-back, stops-square and
//! face-angle sentences, the five size refusals that offer a number to type (`size_sentence`),
//! and the template filler (`format`: `{n}` and `{n:0.##}` holes filled from the advisory's
//! values, numbers to at most as many decimals as the hole has `#`s).

use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use vector_math::{Vector2, Vector3};

/// The English the sentences are written in: the demo shows them as they are.
pub fn localize(sentence: &str) -> String {
    sentence.to_string()
}

/// `value` to at most `decimals` decimals, trailing zeros dropped and no "-0".
pub fn max_decimals(value: f64, decimals: usize) -> String {
    let fixed = format!("{value:.decimals$}");
    let trimmed = if fixed.contains('.') {
        fixed
            .trim_end_matches('0')
            .trim_end_matches('.')
            .to_string()
    } else {
        fixed
    };
    if trimmed == "-0" {
        "0".to_string()
    } else {
        trimmed
    }
}

/// A position as the user reads it off the part: "(x, y, z)" to two decimals.
pub fn describe(position: Vector3) -> String {
    format!(
        "({})",
        [part(position.x), part(position.y), part(position.z)].join(", ")
    )
}

/// A point on a path, read the same way as a position on a part.
pub fn describe_2d(position: Vector2) -> String {
    format!("({})", [part(position.x), part(position.y)].join(", "))
}

fn part(value: f64) -> String {
    max_decimals(value, 2)
}

/// Argument `i`, boxed as it came. An index the advisory does not carry reads as empty text rather
/// than failing; every code's arguments are pinned by the advisory text tests.
pub fn at(args: &[BevelAdvisoryArg], index: usize) -> BevelAdvisoryArg {
    args.get(index)
        .cloned()
        .unwrap_or_else(|| BevelAdvisoryArg::String(String::new()))
}

/// Argument `i` as a count.
pub fn int_at(args: &[BevelAdvisoryArg], index: usize) -> i32 {
    match args.get(index) {
        Some(BevelAdvisoryArg::Int(value)) => *value,
        Some(BevelAdvisoryArg::Double(value)) => *value as i32,
        _ => 0,
    }
}

/// The position the argument holds, said as
/// [`describe`] says it. Any other kind is written as it is.
pub fn describe_at(args: &[BevelAdvisoryArg], index: usize) -> String {
    match args.get(index) {
        Some(BevelAdvisoryArg::Vector3(position)) => describe(*position),
        Some(BevelAdvisoryArg::Vector2(position)) => describe_2d(*position),
        Some(other) => arg_text(other, None),
        None => String::new(),
    }
}

/// How much was picked, in the words the group list uses for a selection.
fn selection(edge_count: i32) -> String {
    if edge_count == 1 {
        localize("1 edge")
    } else {
        format(&localize("{0} edges"), &[BevelAdvisoryArg::Int(edge_count)])
    }
}

/// The line a run that stopped SQUARE puts on its own row - where it
/// stopped and how much was picked. The size is not named, because nothing about the size was given
/// up - the cut is the full size asked for, and
/// what stops short is how far along the part it runs. So the only thing offered is the other one:
/// read more of the edge and it goes further.
pub fn stops_square_sentence(stopped_at: String, edge_count: i32) -> String {
    format(
        &localize("The bevel stops square where {0}: the crease carries straight on past there in an edge that was not picked, so the cut ends against a wall of its own rather than running out onto a face beyond the corner. This selection is {1}; choose a longer reading from the row's dropdown or from the badge beside the edge to cut further."),
        &[text(stopped_at), text(selection(edge_count))],
    )
}

/// The shared sentence about the angle between two faces.
pub fn face_angle_sentence(interior_angle_degrees: BevelAdvisoryArg, too_what: String) -> String {
    format(
        &localize("The faces along this edge meet at {0:0.#} degrees, which is too {1} to bevel."),
        &[interior_angle_degrees, text(too_what)],
    )
}

/// The sentence for one of the three size codes, or `None` for any other code. The room a row
/// offers is `args[2]`; where it is not a double nothing was confirmed to build and the sentence
/// names no size.
pub fn size_sentence(code: BevelAdvisoryCode, args: &[BevelAdvisoryArg]) -> Option<String> {
    let offers_room = matches!(args.get(2), Some(BevelAdvisoryArg::Double(_)));

    Some(match code {
        BevelAdvisoryCode::SizeCollidesAcrossFace if !offers_room => format(
            &localize("A size of {0:0.###} runs into the bevel coming the other way across the face at {1}, so the two of them do not both fit."),
            &[at(args, 0), text(describe_at(args, 1))],
        ),
        BevelAdvisoryCode::SizeCollidesAcrossFace => format(
            &localize("A size of {0:0.###} runs into the bevel coming the other way across the face at {1}, so the two of them do not both fit. The most that fits there is about {2:0.###}."),
            &[at(args, 0), text(describe_at(args, 1)), at(args, 2)],
        ),
        BevelAdvisoryCode::SizeExceedsFaceWidth if !offers_room => format(
            &localize("A size of {0:0.###} reaches further across the face at {1} than the face is wide."),
            &[at(args, 0), text(describe_at(args, 1))],
        ),
        BevelAdvisoryCode::SizeExceedsFaceWidth => format(
            &localize("A size of {0:0.###} reaches further across the face at {1} than the face is wide. The most that fits there is about {2:0.###}."),
            &[at(args, 0), text(describe_at(args, 1)), at(args, 2)],
        ),
        BevelAdvisoryCode::SizeReachesPastCrease if !offers_room => format(
            &localize("A size of {0:0.###} reaches past the crease at {1}, where another face meets this one."),
            &[at(args, 0), text(describe_at(args, 1))],
        ),
        BevelAdvisoryCode::SizeReachesPastCrease => format(
            &localize("A size of {0:0.###} reaches past the crease at {1}, where another face meets this one. The most that fits there is about {2:0.###}."),
            &[at(args, 0), text(describe_at(args, 1)), at(args, 2)],
        ),
        _ => return None,
    })
}

pub fn arg_text(arg: &BevelAdvisoryArg, spec: Option<&str>) -> String {
    match arg {
        BevelAdvisoryArg::Int(value) => value.to_string(),
        BevelAdvisoryArg::Double(value) => match spec {
            None => value.to_string(),
            Some(spec) => max_decimals(*value, spec.chars().filter(|c| *c == '#').count()),
        },
        BevelAdvisoryArg::String(value) => value.clone(),
        BevelAdvisoryArg::Vector2(value) => value.to_string(),
        BevelAdvisoryArg::Vector3(value) => value.to_string(),
    }
}

/// Fills `template` from `args`: every `{n}` / `{n:spec}` hole replaced in one pass, so an
/// argument that itself contains braces is never read as a hole. A hole naming no argument is left
/// as written, so a translation's typo shows rather than losing the sentence (every template here
/// names only arguments it is given, which the advisory text tests pin for every code).
pub fn format(template: &str, args: &[BevelAdvisoryArg]) -> String {
    let mut text = String::with_capacity(template.len() + 32);
    let mut rest = template;
    while let Some(open) = rest.find('{') {
        text.push_str(&rest[..open]);
        let after = &rest[open + 1..];
        let filled = after.find('}').and_then(|close| {
            let hole = &after[..close];
            let (index, spec) = match hole.split_once(':') {
                Some((index, spec)) => (index, Some(spec)),
                None => (hole, None),
            };
            index
                .parse::<usize>()
                .ok()
                .and_then(|index| args.get(index))
                .map(|arg| (close, arg_text(arg, spec)))
        });
        match filled {
            Some((close, value)) => {
                text.push_str(&value);
                rest = &after[close + 1..];
            }
            None => {
                text.push('{');
                rest = after;
            }
        }
    }
    text.push_str(rest);
    text
}

/// A piece of sentence boxed as an argument.
pub fn text(value: String) -> BevelAdvisoryArg {
    BevelAdvisoryArg::String(value)
}
