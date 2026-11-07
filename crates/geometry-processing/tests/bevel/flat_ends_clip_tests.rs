//! The end plane with flat ends on (plan stage 2) is a KNOWN CUT where the band decides sides.
//! Where the end face and a side's crease with it lie IN that plane (a concave chain's apex end on
//! a leaning end face), on the plane the chain's own value places a vertex, negative in the band,
//! so the end vertex leaves the kept mesh and no side sliver is kept whose three corners all read 0.
//! Parameter rows are a loop.

use std::rc::Rc;

use geometry_processing::bevel::level_line::BandGapResult;
use geometry_processing::bevel::level_line::ScalarField;
use vector_math::Vector3;

use crate::end_face_builds::{as_saved, at_bend, build_chains, FlatEnds};
use crate::level_line_fixture::LevelLineFixture;

const BEND_DEGREES: f64 = 20.0;

// The end clip itself stays continuous (lifting points ON the plane too): a clip lifting only
// points strictly past it passes these rows but jumps at the plane, and the refinement cannot
// settle a jump (Center Flows Through ran out of work). The plane is read instead as a KNOWN CUT
// where sides are decided (VertexFieldReading.KnownCutValueAt): a vertex on it in the chain's
// reach is inside, not on the line.
#[test]
fn inner_right_sides_apex_ends_leave_the_kept_mesh() {
    for fraction_of_room in [0.5, 0.9, 1.0] {
        let fixture = Rc::new(LevelLineFixture::load("inner-right-sides-and-center-op0"));
        let all: Vec<usize> = (0..fixture.chains.len()).collect();
        let gap = {
            let _flat_ends = FlatEnds::set(true);
            let room = build_chains(&fixture, &all, fixture.size, &as_saved(&fixture, None)).room;
            build_chains(
                &fixture,
                &all,
                fraction_of_room * room,
                &as_saved(&fixture, None),
            )
        };

        assert!(
            gap.built(),
            "at {fraction_of_room}: {:?}, room {}",
            gap.refusal,
            gap.room
        );
        // Chain 0's first end (5) and chain 2's last end (8): both on the end face x - y = 98.5.
        the_apex_leaves_and_no_band_triangle_reads_all_zero(&gap, fixture.positions[5]);
        the_apex_leaves_and_no_band_triangle_reads_all_zero(&gap, fixture.positions[8]);
    }
}

#[test]
fn bottom_center_edge_apex_leaves_the_kept_mesh() {
    let fixture = Rc::new(LevelLineFixture::load("bottom-center-edge-op0"));
    let gap = {
        let _flat_ends = FlatEnds::set(true);
        build_chains(&fixture, &[0], fixture.size, &at_bend(BEND_DEGREES))
    };

    assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);
    the_apex_leaves_and_no_band_triangle_reads_all_zero(&gap, Vector3::new(-10.0, -108.5, 6.6667));
}

fn the_apex_leaves_and_no_band_triangle_reads_all_zero(gap: &BandGapResult, apex: Vector3) {
    let mesh = gap.mesh.as_ref().expect("the mesh").borrow();
    let near: Vec<i32> = (0..mesh.vertex_count())
        .filter(|&v| (mesh.position(v) - apex).length() < 1e-3)
        .collect();
    assert_eq!(near.len(), 1);
    let v = near[0];

    let value = |vertex: i32| {
        gap.chains
            .iter()
            .map(|f| f.value_at(mesh.position(vertex)).expect("the field reads"))
            .fold(f64::INFINITY, f64::min)
    };
    let max_setback = gap
        .chains
        .iter()
        .map(|f| f.max_setback)
        .fold(f64::NEG_INFINITY, f64::max);
    let mut kept_at_apex = 0;
    let mut all_zero = 0;
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        // Only a SIDE's triangles are the band's to cut; the end face (tag 0, no side) is the
        // end's boundary and stays whole on the kept mesh, the apex with it - the hole is filled
        // against it.
        let tri = mesh.triangles()[t as usize];
        if tri.tag == 0 {
            continue;
        }

        let corners = [tri.a, tri.b, tri.c];
        if corners.contains(&v) {
            kept_at_apex += 1;
        }

        // Within the apex's neighbourhood, a kept side triangle whose three corners all read 0.
        if corners
            .iter()
            .all(|&c| (mesh.position(c) - apex).length() < 4.0 * max_setback)
            && corners.iter().all(|&c| value(c).abs() <= 1e-9)
        {
            all_zero += 1;
        }
    }

    let across_the_cut = gap
        .chains
        .iter()
        .map(|f| {
            f.value_across_the_known_cut(mesh.position(v))
                .expect("the field reads")
        })
        .fold(f64::INFINITY, f64::min);
    println!(
        "apex {apex:?}: kept side triangles on it {kept_at_apex}, kept all-zero side triangles near it {all_zero}, its value {}, across the known cut {across_the_cut}",
        value(v)
    );
    // In the band: the clipped field still reads the end vertex exactly 0 (the plane is the line
    // there), but the end plane is a known cut and on its band side the chain's own value places
    // the vertex inside.
    assert!(across_the_cut < 0.0, "{across_the_cut}");
    assert_eq!(kept_at_apex, 0);
    assert_eq!(all_zero, 0);
}
