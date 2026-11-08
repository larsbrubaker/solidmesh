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

// The Fillet & Chamfer page: a pick made the way the page makes it (a ray at the part, through
// `PickSession::pick`) and built the way the page builds it (`fillet_part`, one `bevel_part` call
// per radius).
mod fillet {
    use polygon_mesh::CancelToken;
    use solidmesh_wasm::fillet::{fillet_part, FilletBuild, FilletPick};
    use solidmesh_wasm::fillet_parts::part;
    use solidmesh_wasm::fillet_pick::{Pick, PickMode, PickSession};
    use vector_math::Vector3;

    /// The mm a pixel spans at 1 mm from the eye in the page's 40 degree view, about 700 px tall.
    const PIXEL_SIZE: f64 = 0.001;

    /// What a click at `target`, seen from `target + from`, picks in `mode`.
    fn pick_at(session: &PickSession, target: Vector3, from: Vector3, mode: PickMode) -> Pick {
        let origin = target + from;
        session
            .pick(origin, target - origin, mode, PIXEL_SIZE, true)
            .expect("the pick runs")
            .expect("the click lands on a pick")
    }

    fn build(index: u32, picks: Vec<(Pick, f64)>) -> FilletBuild {
        geometry_processing::bevel::level_line::level_line_routing::turn_on_for_the_running_app();
        let source = part(index).expect("the part builds");
        let picks: Vec<FilletPick> = picks
            .into_iter()
            .map(|(pick, size)| FilletPick {
                group: pick.group,
                size,
            })
            .collect();
        fillet_part(&source, &picks, true, 6, &CancelToken::new()).expect("the build runs")
    }

    fn assert_built(build: &FilletBuild, what: &str) {
        assert!(
            build.scene.mesh.is_closed_consistently_wound(),
            "{what}: not closed; {}",
            build.scene.note
        );
        for row in &build.rows {
            assert!(row.built && row.code.is_none(), "{what}: {row:?}");
        }
    }

    /// The L-bracket's concave edge inside the L, picked as a chain from above the base, rounds.
    #[test]
    fn the_l_brackets_inside_edge_is_picked_and_filleted() {
        let session = PickSession::new(&part(0).expect("the bracket builds"));
        let pick = pick_at(
            &session,
            Vector3::new(-16.9, 0.0, 8.0),
            Vector3::new(20.0, 0.0, 20.0),
            PickMode::Chain,
        );
        assert_eq!(1, pick.segments.len(), "{}", pick.label);
        let source_volume = part(0).expect("the bracket builds").get_volume();

        let build = build(0, vec![(pick, 4.0)]);

        assert_built(&build, "inside edge");
        assert_eq!(1, build.passes);
        // a concave round adds material
        assert!(build.scene.mesh.get_volume() > source_volume);
    }

    /// The plate's pocket rim, rounded corners and all, is one chain loop and rounds whole.
    #[test]
    fn the_plates_pocket_rim_is_picked_as_one_loop_and_filleted() {
        let session = PickSession::new(&part(1).expect("the plate builds"));
        let pick = pick_at(
            &session,
            Vector3::new(18.1, 0.0, 10.0),
            Vector3::new(15.0, 0.0, 15.0),
            PickMode::Chain,
        );
        assert_eq!(36, pick.segments.len(), "{}", pick.label);

        let build = build(1, vec![(pick, 1.5)]);

        assert_built(&build, "pocket rim");
    }

    /// Picks with different radii are built one radius after another, the later pick found
    /// again on the mesh the earlier one rounded.
    #[test]
    fn two_radii_on_the_l_bracket_are_two_calls_that_both_build() {
        let session = PickSession::new(&part(0).expect("the bracket builds"));
        let inside = pick_at(
            &session,
            Vector3::new(-16.9, 0.0, 8.0),
            Vector3::new(20.0, 0.0, 20.0),
            PickMode::Chain,
        );
        let top = pick_at(
            &session,
            Vector3::new(-17.1, 0.0, 40.0),
            Vector3::new(20.0, 0.0, 20.0),
            PickMode::Chain,
        );

        let build = build(0, vec![(inside, 4.0), (top, 2.0)]);

        assert_built(&build, "two radii");
        assert_eq!(2, build.passes);
        assert_eq!(1, build.rows[1].pass);
    }

    /// A radius wider than the face the round lands on is refused with its code and the size
    /// that does fit, and the part comes back unchanged and closed.
    #[test]
    fn an_oversized_radius_is_refused_with_the_size_that_fits() {
        let session = PickSession::new(&part(0).expect("the bracket builds"));
        let pick = pick_at(
            &session,
            Vector3::new(-16.9, 0.0, 8.0),
            Vector3::new(20.0, 0.0, 20.0),
            PickMode::Chain,
        );

        let build = build(0, vec![(pick, 50.0)]);

        let row = &build.rows[0];
        assert!(!row.built, "{row:?}");
        assert_eq!(Some("SizeExceedsFaceWidth"), row.code.as_deref(), "{row:?}");
        let fits = row.fits.expect("a size that fits");
        assert!(fits > 0.0 && fits < 50.0, "{row:?}");
        assert!(row.text.contains("most that fits"), "{}", row.text);
        assert!(build.scene.mesh.is_closed_consistently_wound());
    }
}
