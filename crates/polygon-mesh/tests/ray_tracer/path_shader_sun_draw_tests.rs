//! Where `PATH_WGSL` draws the two random numbers a vertex's sun direction comes from. On an
//! Apple M5's Metal driver, with `path_sample_sun` drawing them itself just after its early return
//! on the view's sun and strategy, the path loop's default compile lost values held across that
//! call a whole SIMD group at a time, differently from run to run: two fresh renders of one view
//! differed in 64 to 256 of 12288 pixels (a filled bed seen from afar, 3136 to 4844), and the
//! card read darker than the CPU. With the two numbers drawn by `path_radiance` and passed in, two
//! fresh renders agreed bit for bit on every view measured; so they did with the draws moved
//! before that return, or with the return taken away. These tests keep the sun's draw in the
//! caller. They read the shader only: what a driver compiles from it is the GPU route's own tests'
//! to check.

use naga::{Block, Function, Module, Statement};
use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;

/// The functions that advance the path's random stream.
const DRAWS: [&str; 3] = ["path_next_bits", "path_next_unit", "path_next_unit3"];

/// How many calls in `block`, at any depth of its `if`s, `switch`es and loops, go to a function
/// [`DRAWS`] names.
fn draws_in_block(module: &Module, block: &Block) -> usize {
    block
        .iter()
        .map(|statement| match statement {
            Statement::Call { function, .. } => {
                let name = module.functions[*function].name.as_deref().unwrap_or("");
                usize::from(DRAWS.contains(&name))
            }
            Statement::Block(inner) => draws_in_block(module, inner),
            Statement::If { accept, reject, .. } => {
                draws_in_block(module, accept) + draws_in_block(module, reject)
            }
            Statement::Switch { cases, .. } => cases
                .iter()
                .map(|case| draws_in_block(module, &case.body))
                .sum(),
            Statement::Loop {
                body, continuing, ..
            } => draws_in_block(module, body) + draws_in_block(module, continuing),
            _ => 0,
        })
        .sum()
}

/// The function `name` of `module`.
fn function<'a>(module: &'a Module, name: &str) -> &'a Function {
    module
        .functions
        .iter()
        .map(|(_, function)| function)
        .find(|function| function.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("the shader has a `{name}`"))
}

/// How many draws `name`'s own body makes in `source`, counted by call site (a callee's draws are
/// its own).
fn draws_in(source: &str, name: &str) -> usize {
    let module = naga::front::wgsl::parse_str(source).expect("the shader parses as WGSL");
    draws_in_block(&module, &function(&module, name).body)
}

#[test]
fn the_sun_sample_draws_nothing_itself() {
    assert_eq!(
        draws_in(PATH_WGSL, "path_sample_sun"),
        0,
        "`path_sample_sun` must take its sun draw as `u` from `path_radiance`: with the draw made \
         inside it, after its early return, an Apple M5 lost values held across the call a whole \
         SIMD group at a time, and two fresh renders differed"
    );
}

#[test]
fn the_path_loop_draws_the_suns_two_numbers() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the shader parses as WGSL");
    let sample = function(&module, "path_sample_sun");
    let u = sample
        .arguments
        .iter()
        .find(|argument| argument.name.as_deref() == Some("u"))
        .expect("`path_sample_sun` takes the sun's draw as `u`");
    assert_eq!(
        module.types[u.ty].inner,
        naga::TypeInner::Vector {
            size: naga::VectorSize::Bi,
            scalar: naga::Scalar::F32,
        },
        "`u` is the sun's two numbers"
    );
    // The event's `path_next_unit3`, roulette's draw and the sun's two.
    assert_eq!(draws_in(PATH_WGSL, "path_radiance"), 4);
}

#[test]
fn the_count_finds_draws_at_any_depth() {
    let module = |body: &str| {
        format!(
            "var<private> state: u32;\n\
             fn path_next_bits() -> u32 {{ state = state * 3u + 1u; return state; }}\n\
             fn path_next_unit() -> f32 {{ return f32(path_next_bits() >> 8u); }}\n\
             fn path_next_unit3() -> vec3<f32> {{ return vec3<f32>(path_next_unit()); }}\n\
             fn other() -> f32 {{ return path_next_unit(); }}\n\
             {body}\n"
        )
    };
    for (body, draws) in [
        ("fn f() -> f32 { return 1.0; }", 0),
        ("fn f() -> f32 { return other(); }", 0),
        ("fn f() -> f32 { let a = path_next_unit(); return a; }", 1),
        (
            "fn f(c: bool) -> f32 { if c { return 0.0; } let a = path_next_unit(); \
             let b = path_next_unit(); return a + b; }",
            2,
        ),
        (
            "fn f(c: u32) -> f32 { var s = 0.0; loop { if c > 2u { s += path_next_unit(); } \
             switch c { case 1u: { s += path_next_unit3().x; } default: {} } \
             { let b = path_next_bits(); s += f32(b); } break; } return s; }",
            3,
        ),
    ] {
        assert_eq!(draws_in(&module(body), "f"), draws, "{body}");
    }
}
