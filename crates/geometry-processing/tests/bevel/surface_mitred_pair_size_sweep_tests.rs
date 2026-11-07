//! The mitred-pair pick of
//! `surface_run_out_crosses_a_mitred_pair_tests` swept across every size from 2.00 to 5.20 in
//! hundredths of two, as a RATCHET - at every one of the 161 sizes the kernel either builds a
//! closed manifold shell or refuses by a NAMED code. A size that comes back with the generic
//! `SurfaceResultNotClosed`, or with a mesh that has an open edge in it, is a cause the kernel has
//! not read yet, and is written down here by size - so the list can only ever shrink.
//!
//! Both tables are EMPTY: the three causes that stood in the way were all the run-out's own death -
//! its last column not collapsing where it clears the wall, its dying twice against the plane
//! across the wall's end, and its foot standing a fraction of a ruler from the part's own corner
//! rather than ON it. The single-size tests pin each.

use std::rc::Rc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, SurfaceBevelPlan,
    SurfaceBuildFailure, SurfaceBuildOptions,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::FeatureEdge;
use indexmap::IndexMap;
use polygon_mesh::CancelToken;
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_run_out_crosses_a_band_tests::{lobed_regions, the_barrel, the_far_side};
use crate::surface_test_chains::join_and_open;

/// The first size of the sweep, and the step and count that walk it to 5.20.
const FROM: f64 = 2.00;

const STEP: f64 = 0.02;

const SIZES: i32 = 161;

/// The sizes that still come back with the GENERIC closure complaint, with the cause each was
/// measured to have. EMPTY, and it stays empty: nothing may be ADDED to this table.
fn still_generic() -> IndexMap<String, String> {
    IndexMap::new()
}

/// And the sizes the kernel BUILDS and hands back with a hole in the shell. EMPTY, and it stays
/// empty: nothing may be added here either.
fn still_open() -> IndexMap<String, String> {
    IndexMap::new()
}

/// One size of the sweep, read on its own so the cause this rung names has a red test of its own:
/// at 4.30 the run-out's foot fan draws two triangles on one chord of its own column, walked the
/// same way round both times, which the corner used to complain about as a generic closure
/// failure at the end of everything.
#[test]
fn the_run_outs_foot_fan_doubling_back_is_named_for_the_run_out() {
    let (code, at) = build(4.30);

    assert_eq!(
        code,
        format!(
            "{:?}",
            BevelAdvisoryCode::SurfaceRunOutLiesOnAFaceAlreadyDrawn
        ),
        "the fold is the run-out's own, and is read where it is made - instead {code}"
    );

    assert!(
        (at - Vector3::new(3.5760, 0.0713, 1.1746)).length() < 1e-3,
        "on the chord out of [3.576, 0.0713, 1.1746] - instead {at:?}"
    );
}

/// And the cause this rung reads, which cost three sizes of the sweep a hole apiece: at 3.88 the
/// run-out's last column did not COLLAPSE - the row it is drawn from (the cut's ring turned about
/// the axis) and the trim point (the corner's own circle) stand three rulers apart, so the column
/// came out a thousandth long and the sliver of cap between the two went to nobody.
#[test]
fn the_cap_gives_up_the_sliver_out_to_the_run_outs_last_trim_point() {
    let (code, at) = build(3.88);

    assert_eq!(
        code, "OK",
        "the shell closes on one manifold solid - instead {code} at {at:?}"
    );
}

/// The last size that published a shell with a hole in it: at 4.62 the run-out is ended by the
/// plane across the wall's own END rather than by clearing the wall, and its last two stations are
/// each a column of ONE point. The triangle between those two chords and the trim curve went to
/// nobody, and the far corner had the same with the section's own point in it.
#[test]
fn the_death_of_the_run_out_at_the_walls_end_is_topped_by_the_sections_own_rim() {
    let (code, at) = build(4.62);

    assert_eq!(
        code, "OK",
        "the shell closes on one manifold solid - instead {code} at {at:?}"
    );
}

/// And the last of the three families: at 4.20 the run-out's foot and the part's own corner came
/// out two vertices a fifth of a ruler apart, the end face kept the wedge behind them, and the
/// sewing split the face along the foot's chord so it ended up with THREE faces on it. 4.22 is the
/// same reading seven ten-thousandths apart.
#[test]
fn the_foot_and_the_parts_own_corner_are_one_place() {
    for size in [4.20, 4.22] {
        let (code, at) = build(size);

        assert_eq!(
            code, "OK",
            "{size:.2} closes on one manifold solid - instead {code} at {at:?}"
        );
    }
}

/// And the sweep itself: every size either builds a closed manifold shell or refuses by a name,
/// with the two tables above the whole of what is left.
#[test]
fn every_size_of_the_sweep_builds_or_refuses_by_name() {
    let still_generic = still_generic();
    let still_open = still_open();
    let mut generic = Vec::new();
    let mut open = Vec::new();

    for at in 0..SIZES {
        let size = FROM + (at as f64 * STEP);
        let key = format!("{size:.2}");
        let (code, where_at) = build(size);

        if code == format!("{:?}", BevelAdvisoryCode::SurfaceResultNotClosed) {
            assert!(
                still_generic.contains_key(&key),
                "{key} comes back with the generic closure complaint at {where_at:?}, and only the \
                 sizes already written down may - a new one is a cause that has to be read"
            );

            generic.push(key);
        } else if code == "OPEN" {
            assert!(
                still_open.contains_key(&key),
                "{key} builds a shell with a hole in it at {where_at:?}, and only the sizes already \
                 written down may"
            );

            open.push(key);
        } else {
            assert!(
                code == "OK" || code.starts_with("Surface"),
                "{key} came back {code}, which is neither a build nor a named refusal"
            );
        }
    }

    // The ratchet's teeth: a size that has been fixed has to come OFF the tables, or the next rung
    // is measuring against a list that no longer says anything true.
    let written_generic: Vec<String> = still_generic.keys().cloned().collect();
    assert_eq!(
        generic, written_generic,
        "generic now: {generic:?}; written down: {written_generic:?}"
    );

    let written_open: Vec<String> = still_open.keys().cloned().collect();
    assert_eq!(
        open, written_open,
        "open now: {open:?}; written down: {written_open:?}"
    );
}

/// The whole pick built at one size: `OK` when the shell closes and is one manifold solid, `OPEN`
/// when it does not, and the refusal's own code otherwise.
fn build(size: f64) -> (String, Vector3) {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = lobed_regions(&prism);
    let mut plans: Vec<Rc<SurfaceBevelPlan>> = Vec::new();

    let mut seeds: Vec<FeatureEdge> = the_barrel(&prism).boundary_edges().to_vec();

    seeds.extend_from_slice(the_far_side(&prism).boundary_edges());

    for chain in join_and_open(&bevel_chain_linker::link(&seeds)) {
        match SurfaceBevelPlan::try_create(&chain, &regions, size, true) {
            Ok(plan) => plans.push(plan),
            Err(why) => return (format!("{:?}", why.code()), at(&why)),
        }
    }

    let fit =
        surface_room::try_fit(&plans, size, &CancelToken::new(), None).expect("not cancelled");

    if !fit.fits {
        let refusal = fit
            .refusal
            .expect("a fit that does not fit names its refusal");
        return (format!("{:?}", refusal.code()), at(&refusal));
    }

    let built = match surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: Some(&fit.crossings),
            ..SurfaceBuildOptions::default()
        },
    ) {
        Ok(built) => built.mesh,
        Err(SurfaceBuildFailure::Refused(reason)) => {
            return (format!("{:?}", reason.code()), at(&reason))
        }
        // Any other failure is named as it is, being neither a build nor a named refusal.
        Err(other) => return (format!("{other:?}"), Vector3::ZERO),
    };

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);

    if open > 0 {
        ("OPEN".to_string(), open_at)
    } else if built.is_manifold() && shell_count(&built) == 1 {
        ("OK".to_string(), Vector3::ZERO)
    } else {
        ("NOT ONE MANIFOLD SHELL".to_string(), Vector3::ZERO)
    }
}

fn at(advisory: &BevelAdvisory) -> Vector3 {
    match advisory.args().first() {
        Some(BevelAdvisoryArg::Vector3(where_at)) => *where_at,
        _ => Vector3::ZERO,
    }
}
