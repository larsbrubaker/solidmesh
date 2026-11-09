//! The perspective camera ray in `PATH_WGSL`: `path_camera_ray` must not reach its direction
//! through the shared `unit_direction`. That function's early return for a vector already nearly
//! unit length is a divergent branch taken only by the pixels within about a tenth of a pixel of
//! the view's axis, and on one driver (NVIDIA 610.60, Vulkan) that branch at this call site loses
//! the device, while the same shader runs on another driver. The comment at the call in
//! `path.wgsl` says what was measured; this test keeps the call site from drifting back.

use naga::{Function, MathFunction, Statement};
use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;

/// Every statement of `body`, nested blocks included.
fn walk<'a>(body: &'a naga::Block, into: &mut Vec<&'a Statement>) {
    for statement in body.iter() {
        into.push(statement);
        match statement {
            Statement::Block(inner) => walk(inner, into),
            Statement::If { accept, reject, .. } => {
                walk(accept, into);
                walk(reject, into);
            }
            Statement::Switch { cases, .. } => {
                for case in cases {
                    walk(&case.body, into);
                }
            }
            Statement::Loop {
                body, continuing, ..
            } => {
                walk(body, into);
                walk(continuing, into);
            }
            _ => {}
        }
    }
}

fn path_camera_ray(module: &naga::Module) -> &Function {
    module
        .functions
        .iter()
        .map(|(_, function)| function)
        .find(|function| function.name.as_deref() == Some("path_camera_ray"))
        .expect("the path loop has a camera ray")
}

#[test]
fn the_perspective_camera_ray_is_normalized_without_a_branch_on_its_length() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses as WGSL");
    let function = path_camera_ray(&module);

    let mut statements = Vec::new();
    walk(&function.body, &mut statements);
    let called: Vec<&str> = statements
        .iter()
        .filter_map(|statement| match statement {
            Statement::Call { function, .. } => module.functions[*function].name.as_deref(),
            _ => None,
        })
        .collect();
    assert!(
        !called.contains(&"unit_direction"),
        "path_camera_ray goes through unit_direction, whose early return for a nearly unit vector \
         loses the device on NVIDIA Vulkan when a pixel takes it (see path.wgsl); it calls {called:?}"
    );

    let normalizes = function.expressions.iter().any(|(_, expression)| {
        matches!(
            expression,
            naga::Expression::Math {
                fun: MathFunction::Normalize,
                ..
            }
        )
    });
    assert!(
        normalizes,
        "path_camera_ray still has to make its direction unit length"
    );
}
