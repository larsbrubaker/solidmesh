//! Where `PATH_WGSL` keeps the tree traversal's stack, and how many bytes of arrays a function may
//! hold. On an Apple M5's Metal driver, with the stack's two 128-entry arrays (512 bytes each)
//! local to `trace`, the path loop's default compile lost values it held across a traversal, a
//! whole SIMD group at a time: a composited sample's plan read as "no path", and glass drew a
//! third darker than on the CPU. The loss followed the size of that function-local stack: with
//! the two arrays at 32 entries (128 bytes each) in that one function there was almost no loss, at
//! 512 entries more than at 128, and with the stack at module scope none. These tests keep the
//! stack there and hold each function's arrays, sized by naga from the parsed module, to that
//! almost-loss-free case. They read the shader only: what a driver compiles from it is the GPU
//! route's own tests' to check.

use naga::valid::{Capabilities, ValidationFlags, Validator};
use naga::{Function, Module, TypeInner};
use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;

use crate::path_shader_inlining_tests::without_comments;

/// The most bytes of arrays one function may hold: what `trace` held, counted as this test counts,
/// in the measured build with almost no loss: its two stack arrays at 32 four-byte entries, its two
/// 4-entry child-order arrays and its `let` copy of a triangle (three `array<f32, 3>`),
/// `2 * 128 + 2 * 16 + 36`. Held per function, not per array: ten 128-byte arrays in one function
/// would be far past anything measured.
const FUNCTION_ARRAY_BUDGET: u64 = 324;

/// The bytes of arrays a value of `inner` holds: a whole array, or the arrays among a struct's
/// members (arrays inside structs count, since a struct held by value holds them), nothing for a
/// scalar, vector or pointer.
fn array_bytes(module: &Module, inner: &TypeInner) -> u64 {
    match inner {
        TypeInner::Array { .. } => u64::from(inner.size(module.to_ctx())),
        TypeInner::Struct { members, .. } => members
            .iter()
            .map(|member| array_bytes(module, &module.types[member.ty].inner))
            .sum(),
        _ => 0,
    }
}

/// What `function` holds by value, as naga reads it: its `var`s wherever declared (a `for`
/// header's included), its arguments and its result, and each `let` binding, whose value is a
/// copy (`let s = some_array;` holds the array again; a `let` of a pointer holds none). naga names
/// each argument's expression too, so those are skipped among the bindings, which are counted
/// once as arguments; it also names a phony assignment's value (`_ = x`), which is counted, erring
/// toward refusing. An unnamed temporary (`make_stack()[3]`) is not counted.
fn held_array_bytes(module: &Module, info: &naga::valid::FunctionInfo, function: &Function) -> u64 {
    let of = |ty: naga::Handle<naga::Type>| array_bytes(module, &module.types[ty].inner);
    let locals: u64 = function
        .local_variables
        .iter()
        .map(|(_, local)| of(local.ty))
        .sum();
    let arguments: u64 = function
        .arguments
        .iter()
        .map(|argument| of(argument.ty))
        .sum();
    let result = function.result.as_ref().map_or(0, |result| of(result.ty));
    let lets: u64 = function
        .named_expressions
        .keys()
        .filter(|&&expression| {
            !matches!(
                function.expressions[expression],
                naga::Expression::FunctionArgument(_)
            )
        })
        .map(|&expression| array_bytes(module, info[expression].ty.inner_with(&module.types)))
        .sum();
    locals + arguments + result + lets
}

/// Each function and entry point of `source` past [`FUNCTION_ARRAY_BUDGET`], with its bytes.
fn functions_over_budget(source: &str) -> Vec<String> {
    let module = naga::front::wgsl::parse_str(source).expect("the shader parses as WGSL");
    let info = Validator::new(ValidationFlags::all(), Capabilities::all())
        .validate(&module)
        .expect("the shader validates");
    let mut held: Vec<(String, u64)> = module
        .functions
        .iter()
        .map(|(handle, function)| {
            let name = function.name.clone().unwrap_or_default();
            (name, held_array_bytes(&module, &info[handle], function))
        })
        .collect();
    for (index, entry) in module.entry_points.iter().enumerate() {
        let bytes = held_array_bytes(&module, info.get_entry_point(index), &entry.function);
        held.push((entry.name.clone(), bytes));
    }
    held.into_iter()
        .filter(|&(_, bytes)| bytes > FUNCTION_ARRAY_BUDGET)
        .map(|(name, bytes)| format!("{name}: {bytes} bytes"))
        .collect()
}

/// How many times `source` declares `declaration` as a whole line at module scope.
fn module_scope_declarations(source: &str, declaration: &str) -> usize {
    without_comments(source)
        .lines()
        .filter(|line| line.trim_end() == declaration)
        .count()
}

#[test]
fn the_traversal_stack_is_one_module_scope_array_pair() {
    for declaration in [
        "var<private> trace_stack_code: array<u32, STACK_CAPACITY>;",
        "var<private> trace_stack_near: array<f32, STACK_CAPACITY>;",
    ] {
        assert_eq!(
            module_scope_declarations(PATH_WGSL, declaration),
            1,
            "`{declaration}` must be declared once, at module scope"
        );
    }
}

#[test]
fn no_function_holds_more_array_bytes_than_the_measured_case() {
    let over = functions_over_budget(PATH_WGSL);
    assert!(
        over.is_empty(),
        "functions holding more than {FUNCTION_ARRAY_BUDGET} bytes of arrays: with the \
         traversal's two 512-byte stack arrays local to `trace`, an Apple M5 lost values held \
         across a traversal a whole SIMD group at a time, and with them at 128 bytes almost none; \
         keep such an array at module scope:\n{}",
        over.join("\n")
    );
}

#[test]
fn the_budget_counts_named_arrays_arguments_and_results() {
    let module = |body: &str| {
        format!(
            "struct Stack {{ entries: array<u32, 128>, size: u32 }}\n\
             struct Table {{ to_linear: array<vec4<f32>, 64> }}\n\
             @group(0) @binding(0) var<uniform> table: Table;\n\
             var<private> kept: array<u32, 128>;\n\
             {body}\n"
        )
    };
    for over in [
        "fn f() { var a: array<u32, 128>; var b: array<f32, 128>; a[0] = 1u; b[0] = 1.0; }",
        "fn f() { var s: Stack; s.size = 1u; }",
        "fn f() -> f32 { let copy = table; return copy.to_linear[0].x; }",
        "fn f() -> u32 { var copy = kept; return copy[0]; }",
        "fn f(a: array<u32, 128>) -> u32 { return a[0]; }",
        "fn f() -> array<u32, 128> { return kept; }",
        "fn f() { for (var a = array<u32, 128>(); a[0] < 1u; a[0] += 1u) {} }",
        "fn f() { var a: array<u32, 64>; var b: array<u32, 64>; var c: array<u32, 64>; \
         a[0] = 1u; b[0] = 1u; c[0] = 1u; }",
    ] {
        assert_eq!(
            functions_over_budget(&module(over)).len(),
            1,
            "{over} must be refused"
        );
    }
    for within in [
        "fn f() { var a: array<f32, 8>; a[0] = 1.0; }",
        "fn f() { var a: array<u32, 32>; var b: array<f32, 32>; var o: array<u32, 4>; \
         var n: array<f32, 4>; a[0] = 1u; b[0] = 1.0; o[0] = 1u; n[0] = 1.0; }",
        "fn f() -> u32 { kept[0] = 1u; return kept[0]; }",
        "fn f(a: array<u32, 64>) -> u32 { return a[0]; }",
        "fn f() -> f32 { return table.to_linear[3].x; }",
        "fn f() -> u32 { let p = &kept; return (*p)[0]; }",
    ] {
        assert!(
            functions_over_budget(&module(within)).is_empty(),
            "{within} is within the budget"
        );
    }
}
