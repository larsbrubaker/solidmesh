// The parts one straight line has gone through, in WGSL: the record a path's straight line keeps
// of the see-through parts it has passed and of how many surfaces it has crossed, the CPU's
// `StraightLine` (`see_through.rs`). Part of `PATH_WGSL` after `path.wgsl`, whose constants
// (`MAX_CROSSINGS_PER_LINE`, `LINE_STOPS`, `LINE_PASSES`, `LINE_ENDED`), part opacities and random
// stream it reads, and before `catcher.wgsl` (`path_shader.rs` concatenates them). Every path in a
// scene with see-through parts uses it, in `path.wgsl`'s loop and its shadow rays, whether or not
// the sample is composited; the catcher's coverage line (`path_coverage_along`) uses it too.

// One straight line's crossings (`see_through.rs`'s `StraightLine`). A path and the shadow rays
// from its vertices never use this at the same time: a shadow ray runs between the verdict that
// ended the last straight line and the restart for the next.
var<private> line_passed: array<u32, MAX_CROSSINGS_PER_LINE>;
var<private> line_passed_count: u32;
var<private> line_crossings: u32;

fn line_restart() {
    line_passed_count = 0u;
    line_crossings = 0u;
}

fn line_has_passed(part: u32) -> bool {
    for (var index = 0u; index < line_passed_count; index += 1u) {
        if line_passed[index] == part {
            return true;
        }
    }
    return false;
}

// Flips whether `part` is among the `odd` parts the coverage line keeps after its passed parts,
// those it has crossed an odd number of times past its end; a part leaving gives its place to the
// last one. Answers how many are kept then. Each kept part and each passed part took a crossing,
// so they fit in `line_passed` while the line is under the cap.
fn line_flip_past(part: u32, odd: u32) -> u32 {
    for (var index = 0u; index < odd; index += 1u) {
        if line_passed[line_passed_count + index] == part {
            line_passed[line_passed_count + index] = line_passed[line_passed_count + odd - 1u];
            return odd - 1u;
        }
    }
    line_passed[line_passed_count + odd] = part;
    return odd + 1u;
}

// `StraightLine::at_hit`: whether the part is there, deciding once per part along the line.
fn line_at_hit(part: u32) -> u32 {
    let opacity = path_part_opacity(part);
    if opacity >= 1.0f {
        return LINE_STOPS;
    }
    if line_crossings >= MAX_CROSSINGS_PER_LINE {
        return LINE_ENDED;
    }
    line_crossings += 1u;
    if line_has_passed(part) {
        return LINE_PASSES;
    }
    line_passed[line_passed_count] = part;
    line_passed_count += 1u;
    if opacity > 0.0f && path_next_unit() < opacity {
        line_passed_count -= 1u;
        return LINE_STOPS;
    }
    return LINE_PASSES;
}
