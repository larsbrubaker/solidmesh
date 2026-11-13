//! How many places `PATH_WGSL` calls its heavy functions from. A GPU driver copies a function into
//! every place it is called, so each call site of a function holding the tree traversal (`trace`)
//! adds another whole traversal to the compiled program. A phone's driver (Arm Mali, Valhall)
//! refuses the pipeline once the traversal is copied too many times, and a desktop driver takes
//! seconds longer to build it, so the path loop calls each of these from one place.

use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;

/// `source` with every comment removed, so a name in prose is not counted as a call: `//` to the
/// end of the line, and `/* */` blocks, which nest in WGSL.
fn without_comments(source: &str) -> String {
    let mut code = String::with_capacity(source.len());
    let mut chars = source.chars().peekable();
    let mut depth = 0usize;
    while let Some(c) = chars.next() {
        let next = chars.peek().copied();
        if c == '/' && next == Some('*') {
            chars.next();
            depth += 1;
        } else if depth > 0 && c == '*' && next == Some('/') {
            chars.next();
            depth -= 1;
            // A block comment separates the tokens on either side of it.
            if depth == 0 {
                code.push(' ');
            }
        } else if depth > 0 {
            if c == '\n' {
                code.push(c);
            }
        } else if c == '/' && next == Some('/') {
            for rest in chars.by_ref() {
                if rest == '\n' {
                    code.push(rest);
                    break;
                }
            }
        } else {
            code.push(c);
        }
    }
    code
}

fn is_identifier_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

/// How many places in `source` call `name`: each `name` standing alone as an identifier and
/// followed, past any whitespace, by `(`, less its own `fn name(` definition.
fn call_sites_in(source: &str, name: &str) -> usize {
    let code = without_comments(source);
    let mut count = 0;
    for (at, _) in code.match_indices(name) {
        let before = &code[..at];
        let after = &code[at + name.len()..];
        if before.chars().next_back().is_some_and(is_identifier_char)
            || after.chars().next().is_some_and(is_identifier_char)
            || !after.trim_start().starts_with('(')
        {
            continue;
        }
        let ahead = before.trim_end();
        let defined = ahead.ends_with("fn")
            && !ahead[..ahead.len() - 2]
                .chars()
                .next_back()
                .is_some_and(is_identifier_char);
        if !defined {
            count += 1;
        }
    }
    count
}

/// How many places in `PATH_WGSL` call `name`.
fn call_sites(name: &str) -> usize {
    call_sites_in(PATH_WGSL, name)
}

#[test]
fn the_path_loop_calls_each_traversing_function_from_one_place() {
    // Each of these runs the tree traversal, directly or through one it calls.
    for name in [
        "path_radiance",
        "path_coverage_along",
        "path_sun_line",
        "path_sample_sun",
    ] {
        assert_eq!(
            call_sites(name),
            1,
            "{name} is called from {} places",
            call_sites(name)
        );
    }
    // The path loop traverses in three places: the path's own ray and a line's coverage through
    // `trace_closest`, and the shadow ray to the sun through `trace` itself. The rest are the
    // wrappers `trace_closest` and `trace_any` and the two entry points `trace.wgsl` keeps for
    // tracing a buffer of rays, which a pipeline built for the path loop leaves out.
    assert_eq!(
        call_sites("trace"),
        3,
        "trace_closest, trace_any and path_sun_line"
    );
    assert_eq!(
        call_sites("trace_closest"),
        3,
        "path_radiance, path_coverage_along, trace_closest_main"
    );
    assert_eq!(call_sites("trace_any"), 1, "trace_any_main");
}

#[test]
fn the_call_site_count_reads_calls_and_not_definitions_or_prose() {
    let source = concat!(
        "fn walk (ray: Ray) -> u32 {\n",
        "    let a = walk (ray); // walk(ray) in prose\n",
        "    /* outer /* walk(ray) nested */ walk(ray) still a comment */\n",
        "    let b = walk_far(ray) + sidewalk(ray);\n",
        "    return walk\n(ray);\n",
        "}\n",
    );
    // The two real calls, one with a space and one with a line break before the parenthesis; not
    // the definition, the comments, or the longer names that contain it.
    assert_eq!(call_sites_in(source, "walk"), 2);
    assert_eq!(call_sites_in(source, "walk_far"), 1);
    assert_eq!(call_sites_in(source, "no_such_function"), 0);
    // As in `trace.wgsl`, `trace` is not counted inside `trace_closest(` or `trace_any(`.
    let traces = concat!(
        "fn trace_closest(r: Ray) -> Hit { return trace(r, false); }\n",
        "fn f(r: Ray) -> Hit { return trace_closest(r); }\n",
    );
    assert_eq!(call_sites_in(traces, "trace"), 1);
    assert_eq!(call_sites_in(traces, "trace_closest"), 1);
}
