// The demo pages' scenes, built natively with each page's starting controls: every page must
// produce a non-empty mesh, and every page that promises a closed solid must deliver one. These
// are the same builders the browser calls through `solidmesh_wasm`'s bindings, so a kernel change
// that breaks a page fails here before the site is deployed.

use solidmesh_wasm::booleans::{boolean_scene, BooleanParams};
use solidmesh_wasm::implicit::implicit_scene;
use solidmesh_wasm::morph::{hull_scene, minkowski_scene};
use solidmesh_wasm::outline::{contours_from_flat, extrude_scene, revolve_scene};
use solidmesh_wasm::repair::{repair_scene, RepairParams};
use solidmesh_wasm::showcase::showcase_part;
use solidmesh_wasm::Scene;

fn assert_solid(scene: &Scene, what: &str) {
    assert!(!scene.mesh.faces.is_empty(), "{what}: empty mesh");
    assert!(
        scene.mesh.is_closed_consistently_wound(),
        "{what}: not closed ({} open edges); {}",
        scene.mesh.get_non_manifold_edges().len(),
        scene.note
    );
    assert!(scene.mesh.get_volume() > 0.0, "{what}: no volume");
}

fn booleans(op: u32, messy: u32, nonzero: bool, repair: bool) -> Scene {
    boolean_scene(&BooleanParams {
        shape_a: 0,
        shape_b: 1,
        op,
        offset: [8.0, 6.0, 5.0],
        rotate_deg: 20.0,
        messy,
        nonzero,
        repair,
    })
    .expect("boolean scene")
}

#[test]
fn every_boolean_of_clean_operands_is_a_closed_solid() {
    for op in 0..3 {
        assert_solid(&booleans(op, 0, false, false), &format!("op {op}"));
    }
}

#[test]
fn messy_operands_still_give_a_closed_solid() {
    assert_solid(&booleans(0, 3, true, false), "nonzero union");
    assert_solid(&booleans(1, 3, false, true), "repaired difference");
}

#[test]
fn outlines_extrude_bevel_and_revolve_to_solids() {
    // A square with a square hole, in mm.
    let coords = [
        0.0, 0.0, 20.0, 0.0, 20.0, 20.0, 0.0, 20.0, 6.0, 6.0, 6.0, 14.0, 14.0, 14.0, 14.0, 6.0,
    ];
    let contours = contours_from_flat(&coords, &[4, 4]);
    assert_solid(
        &extrude_scene(&contours, 10.0, 0.0, 0.0, 4, true).expect("extrude"),
        "extrude",
    );
    assert_solid(
        &extrude_scene(&contours, 10.0, 2.0, 1.0, 4, true).expect("bevel"),
        "bevelled extrude",
    );
    let offset: Vec<f64> = coords.chunks(2).flat_map(|p| [p[0] + 5.0, p[1]]).collect();
    let profile = contours_from_flat(&offset, &[4, 4]);
    assert_solid(
        &revolve_scene(&profile, 48, 0.0, 360.0).expect("revolve"),
        "revolve",
    );
    assert_solid(
        &revolve_scene(&profile, 24, 0.0, 270.0).expect("partial revolve"),
        "partial revolve",
    );
}

#[test]
fn implicit_scenes_mesh_with_both_meshers() {
    for scene in 0..3 {
        for mesher in 0..2 {
            let result = implicit_scene(scene, 3.0, 48, mesher).expect("implicit scene");
            assert!(
                !result.mesh.faces.is_empty(),
                "scene {scene} mesher {mesher}: empty"
            );
        }
    }
    assert_solid(
        &implicit_scene(0, 3.0, 48, 0).expect("blob"),
        "marching cubes blob",
    );
}

#[test]
fn minkowski_and_hull_give_solids() {
    assert_solid(
        &minkowski_scene(1, 0, 4.0, false, 10).expect("sum"),
        "Minkowski sum",
    );
    assert_solid(
        &minkowski_scene(2, 1, 4.0, true, 10).expect("difference"),
        "Minkowski difference",
    );
    for source in 0..3 {
        assert_solid(
            &hull_scene(source, 40, 7).expect("hull"),
            &format!("hull {source}"),
        );
    }
}

#[test]
fn repair_closes_the_broken_mesh() {
    let params = |mode| RepairParams {
        shape: 0,
        holes: 3,
        flips: 12,
        soup: true,
        seed: 1,
        mode,
        fill_holes: true,
        face_orientation: true,
        weld_edges: true,
    };
    let broken = repair_scene(&params(0)).expect("broken");
    assert!(!broken.mesh.is_closed_consistently_wound(), "not broken");
    assert_solid(&repair_scene(&params(1)).expect("auto"), "auto repair");
    assert_solid(
        &repair_scene(&params(2)).expect("advanced"),
        "advanced repair",
    );
}

#[test]
fn the_showcase_part_is_one_closed_solid() {
    assert_solid(
        &showcase_part(6, 26.0, 3.0, 48).expect("showcase"),
        "showcase",
    );
    assert_solid(
        &showcase_part(0, 20.0, 0.0, 24).expect("plain"),
        "plain showcase",
    );
}
