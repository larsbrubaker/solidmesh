//! The shapes a standing feature's extension is REFUSED for, each by name. Its sibling,
//! `surface_standing_feature_extension_tests.rs`, is the solids the extension builds, and this one
//! is the parts it will not build, which is the other half of the same ruling. A test that covers
//! several shapes loops over them.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::surface_mitre_joint_step::SideSeams;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_end_cap_around_a_feature, surface_mesh_closure,
    surface_standing_feature_faces, SurfaceBevelPlan, SurfaceBuildFailure, SurfaceEdgePlan,
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceStrip,
    SurfaceStripNotchedByAFeature, SurfaceVertexPool,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::FeatureEdge;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::signed_volume;
use crate::nose_in_notch_test_meshes;
use crate::nose_on_a_cube_corner_test_meshes;
use crate::surface_standing_feature_stretch_tests::{plan_of, top_edge_of};
use crate::surface_walkback_checks::regions_of;
use crate::topology_test_meshes;

/// Rung five. The fin fixture's own chamfer is REFUSED by name: its extended plane meets the
/// chamfer along a line parallel to the rails and runs off the part through the other seam, which
/// is a face this end cut does not own.
#[test]
fn the_fin_is_refused_by_name_because_its_trace_leaves_through_the_other_seam() {
    let part = Arc::new(topology_test_meshes::create_fin_on_a_slab());
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);
    let plan = plan_of(
        &regions,
        &features,
        top_edge_of(
            &features,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(0.0, 10.0, 20.0),
        ),
        7.0,
        false,
    );

    let reason = refusal_of(surface_bevel_builder::try_build_one(
        &part,
        &part.get_mesh_edge_graph(),
        &regions,
        plan,
        8,
        &CancelToken::new(),
    ))
    .unwrap_or_else(|| {
        panic!(
            "the fin's plane leaves the strip through the seam rather than through the band's end"
        )
    });
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
}

/// A notch whose end has a FACE BEYOND it rather than a wall of its own is refused by name. The
/// split cap lays its two pieces IN a wall; it cannot clip them out of a face the section only cuts
/// a notch in, so it says so rather than capping into the middle of one.
///
/// The fixture is the same nose moved onto a cube's front face, running off through the band's end
/// at the cube's own corner (20, 0, 20) - `nose_on_a_cube_corner_test_meshes`. The part's right
/// face stands square across that end and holds the whole profile, so the end is not a run-out at
/// all and this is the refusal.
///
/// **What this fixture was built for, and could not reach.** Three convex edges meet at that
/// corner, so it is the one place a notched end could gather into a corner PATCH - which closes all
/// three ends at once and never reads the notch, so the extension would go in and the patch would
/// cap over it. `surface_builder_standing_features::the_notched_end_can_be_closed` refuses that by
/// name. It is asserted below only as "nothing is published", because the shape cannot be built to
/// ask it properly: the feature's creases have to die IN the band's end plane, that plane is the
/// right face's own, and so the crease ends land on one of the other two edges at the corner -
/// which makes that edge SHORTER than the radius the notch needs, and one size serves the whole
/// operation. A corner that is not a right angle might get round that; this one cannot.
#[test]
fn a_notch_at_an_end_with_a_face_beyond_it_is_refused_by_name() {
    let part = Arc::new(nose_on_a_cube_corner_test_meshes::create());

    let (open, open_at) = surface_mesh_closure::open_edge_count(&part);
    assert_eq!(open, 0, "the fixture itself closes - open at {open_at:?}");
    let volume = signed_volume(&part);
    assert!(
        (volume - (8000.0 + (1000.0 / 9.0))).abs() <= 1e-4,
        "the cube plus the nose's own tetrahedron, 111.111 of it - to a float's own ulp, which is \
         what a mesh stores its positions in - instead {volume}"
    );

    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);
    let the_corner = nose_on_a_cube_corner_test_meshes::the_corner();
    let the_tip = nose_on_a_cube_corner_test_meshes::the_tip();
    let mut at_the_corner = Vec::new();
    let mut the_bands_edge: i32 = -1;

    for index in 0..features.count() {
        let edge = &features.edges()[index];

        if (edge.start - the_corner).length() < 1e-6 || (edge.end - the_corner).length() < 1e-6 {
            at_the_corner.push(index);
        }

        if (edge.start - the_tip).length() < 1e-6 && (edge.end - the_corner).length() < 1e-6 {
            the_bands_edge = index as i32;
        }
    }

    assert_eq!(
        at_the_corner.len(),
        3,
        "three convex edges of the cube meet where the extension runs off"
    );
    assert!(
        the_bands_edge >= 0,
        "and the top front edge runs the whole width of the part, uncut"
    );

    let graph = part.get_mesh_edge_graph();
    let reason = refusal_of(
        surface_bevel_builder::try_build(
            &part,
            &graph,
            &regions,
            &[plan_of(
                &regions,
                &features,
                the_bands_edge as usize,
                15.0,
                true,
            )],
            16,
            &CancelToken::new(),
            crate::surface_test_build::build_options(),
        )
        .map(|built| built.mesh),
    )
    .unwrap_or_else(|| {
        panic!(
            "the section this end gives up is cut out of the right face, and the split cap can \
             only be laid in a wall of the groove's own"
        )
    });
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );

    let gathered: Vec<Rc<SurfaceBevelPlan>> = at_the_corner
        .iter()
        .map(|&index| plan_of(&regions, &features, index, 15.0, true))
        .collect();

    assert!(
        surface_bevel_builder::try_build(
            &part,
            &graph,
            &regions,
            &gathered,
            16,
            &CancelToken::new(),
            crate::surface_test_build::build_options(),
        )
        .is_err(),
        "and all three together publish nothing either - which is what a corner patch capping \
         over the extension would have done"
    );
}

/// Above ten millimetres the band reaches the nose's LOWER crease as well, so the notch is
/// two-faced AND its far end runs out on the strip's own seam rail - and a notch with a crease
/// crossing at that end has no rail outside it for a Q to stand on. Refused by name, at the seam
/// rail's own end point.
///
/// The next construction up, and named here so the ratchet's own sweep can be read honestly: the
/// one-faced reading carries a crease crossing at ONE end of the trace and a Q at the other, and
/// what these sizes want is a crease crossing at one end of a TWO-faced trace. 10.01 is the first
/// size that asks, and 13.334 the last - above that the tangent line clears the whole root and both
/// ends are Qs again, which is what fifteen builds.
#[test]
fn a_notch_running_out_on_the_seam_rail_at_both_ends_is_refused_by_name() {
    for size in [10.01, 10.1, 12.0, 13.0] {
        let part = Arc::new(nose_in_notch_test_meshes::create());
        let (regions, features) = regions_of(&part);
        let regions = Arc::new(regions);
        let plan = plan_of(
            &regions,
            &features,
            top_edge_of(
                &features,
                Vector3::new(0.0, 0.0, 20.0),
                Vector3::new(10.0, 10.0, 20.0),
            ),
            size,
            true,
        );

        let reason = refusal_of(
            surface_bevel_builder::try_build(
                &part,
                &part.get_mesh_edge_graph(),
                &regions,
                &[plan],
                16,
                &CancelToken::new(),
                crate::surface_test_build::build_options(),
            )
            .map(|built| built.mesh),
        )
        .unwrap_or_else(|| {
            panic!(
                "{size}: the trace's far end is the LOWER crease's crossing on the seam rail, and \
                 there is no rail outside that one for a Q to stand on"
            )
        });
        assert_eq!(
            reason.code(),
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            "{size}"
        );

        // Reported where the reading really gave up: the seam rail's own point in the band's end
        // plane, which is the tangent line on the slope at a setback below the top.
        let at = named_at(&reason);
        assert!(
            (at - Vector3::new(10.0, 10.0, 20.0 - size)).length() < 1e-6,
            "{size}: which is where the tangent line stands at the band's end - instead {at:?}"
        );
    }
}

/// The one-faced notch's own three refusals, driven on a reading rather than through a part: the
/// crease running out on NEITHER seam, on BOTH, and on a seam rail that is no part of the run the
/// plane cut.
///
/// Asked of a synthetic feature over the fixture's real strip because that is where the statement
/// lives. The crease crossing is not crossed for - it is taken from the walkback's own seam, which
/// is the whole point of it - so what can go wrong is what the SEAM says, and a seam is a thing a
/// test can set down. Every one of these is a reading of one thing by two routes that have come
/// apart, and the pass must say so rather than sew to whichever was asked first.
#[test]
fn the_one_faced_notch_names_every_way_the_crease_can_fail_to_leave_the_strip() {
    let mut read = OneFacedReading::of(7.0, 16, false);

    // The seam ending nowhere near the crease at all.
    let away = read.away;
    let neither = read.try_read(seam(1, away, 13.435)).expect_err(
        "no seam of this strip ends on the crease, so nothing says where the trace starts",
    );
    assert_eq!(
        neither.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );

    // BOTH seams ending on it, which only one of them can be true of.
    let on_the_crease = read.pool.add(Vector3::new(9.5, 9.5, 13.0));

    let both = read
        .try_read(both_seams(
            seam(1, on_the_crease, 13.435),
            seam(0, on_the_crease, 13.435),
        ))
        .expect_err(
            "the crease lies in ONE of the two landing faces, so one seam rail at most can be the \
             rail it runs out on",
        );
    assert_eq!(
        both.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
    let at = named_at(&both);
    assert!(
        (at - Vector3::new(10.0, 10.0, 40.0 / 3.0)).length() < 1e-4,
        "named at the CREASE, which is the reading that came apart - and not further on at the \
         crossing, which is where taking whichever seam was asked first would have ended up. \
         Instead {at:?}"
    );

    // On the far seam from the rails the plane cut, so the two are not one run.
    let apart = read
        .try_read(both_seams(
            seam(1, away, 13.435),
            seam(0, on_the_crease, 13.435),
        ))
        .expect_err(
            "the rails the plane cuts stand at one edge of the profile and the crease runs out at \
             the other, which is two holes with strip between them",
        );
    assert_eq!(
        apart.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
    let at = named_at(&apart);
    assert!(
        (at - Vector3::new(9.5, 9.5, 13.0)).length() < 1e-4,
        "named at the crossing the two readings could not be joined at - instead {at:?}"
    );

    // And the seam's own point disagreeing with the plane's crossing of the same rail.
    let disagrees = read.try_read(seam(1, on_the_crease, 9.0)).expect_err(
        "the plane says this rail is cut back to 13.435 and the seam says 9 - two readings of one \
         thing, and neither may be preferred quietly",
    );
    assert_eq!(
        disagrees.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
    let at = named_at(&disagrees);
    assert!(
        (at - Vector3::new(9.5, 9.5, 13.0)).length() < 1e-4,
        "named at the point the two disagree about - instead {at:?}"
    );

    // And with the seam telling the truth it reads, so the refusals above are about what was
    // changed.
    let notch = read
        .try_read(seam(1, on_the_crease, 13.435))
        .unwrap_or_else(|why| panic!("the honest seam reads a one-faced notch - {why}"));
    assert!(notch.is_one_faced());
    assert_eq!(
        notch.the_crease_rail, 16,
        "the crease runs out on the slope, which is the last rail of this walk"
    );
}

/// The extended wall is refused by name where the crease's CROSSING and the crease's end in the
/// band's end plane are not one side of the region's outline - so there is no one stretch for the
/// trace to stand in place of.
///
/// The walkback splits the crease edge at the crossing when it cuts the landing face, so on a part
/// the two ARE one side of the outline and this does not come up. It is asked of a hand-made
/// outline because the alternative is to publish a wall whose rim runs through points of the face
/// it was never bounded by, which is a fold nothing downstream would name.
#[test]
fn a_wall_whose_crease_and_crossing_are_not_one_side_is_refused_by_name() {
    let mut read = OneFacedReading::of(7.0, 16, false);
    let on_the_crease = read.pool.add(Vector3::new(9.5, 9.5, 13.0));

    let notch = read
        .try_read(seam(1, on_the_crease, 13.435))
        .unwrap_or_else(|why| panic!("the reading itself is ordinary - {why}"));

    // The wall's outline with something standing BETWEEN the crease's end and the crossing, and
    // nothing in Within to say it belongs to the crease run.
    read.outline_of_the_upper_wall(&[
        Vector3::new(10.0, 10.0, 40.0 / 3.0),
        Vector3::new(15.0, 5.0, 10.0),
        Vector3::new(9.5, 9.5, 13.0),
        Vector3::new(5.0, 5.0, 10.0),
    ]);

    let refusal = surface_standing_feature_faces::try_emit(
        &mut read.pool,
        &mut Vec::new(),
        &read.strip,
        &notch,
    )
    .expect_err(
        "the stretch the band took is not one side of the outline, so the trace has nothing to \
         stand in place of",
    );
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
    let at = named_at(&refusal);
    assert!(
        (at - Vector3::new(9.5, 9.5, 13.0)).length() < 1e-4,
        "named at the crossing the trace would have started from - instead {at:?}"
    );
}

/// A one-faced notch whose feature has no readable RIDGE is refused BY NAME rather than handed
/// back as a false with nothing in it.
///
/// The split cap asks the feature which end of the crease stands on the ridge, so that it knows
/// which vertex its piece turns down at. A feature with one wall has no ridge to ask about, and an
/// answer with no advisory would not be a refusal downstream at all: the app head formats the
/// advisory to say what happened and would be handed nothing to format. So this way out of the
/// reading names itself, as every other one does.
#[test]
fn a_cap_with_no_ridge_to_read_refuses_by_name_rather_than_with_nothing() {
    let mut read = OneFacedReading::of(7.0, 16, true);
    let on_the_crease = read.pool.add(Vector3::new(9.5, 9.5, 13.0));

    let notch = read
        .try_read(seam(1, on_the_crease, 13.435))
        .unwrap_or_else(|why| {
            panic!("one wall is still a one-faced notch as far as the strip is concerned - {why}")
        });

    let mut faces = Vec::new();
    let rail_count = read.strip.rail_count();
    let length = read.length();
    let mut ring = Vec::with_capacity(rail_count as usize);
    let mut walk = Vec::with_capacity(rail_count as usize);

    for rail in 0..rail_count {
        ring.push(read.pool.add(read.strip.point_on(rail, length)));
        walk.push(rail);
    }

    let corner = read.corner();
    let runs_out_facing = -read.strip.along();

    // The cap refuses, and the error carries the advisory naming why.
    let refusal = surface_end_cap_around_a_feature::try_cap(
        &mut read.pool,
        &mut faces,
        &walk,
        -1,
        &notch,
        &ring,
        corner,
        runs_out_facing,
    )
    .expect_err(
        "there is no ridge, so the crease's two ends cannot be told apart and the cap does not \
         know which vertex its piece turns down at",
    );
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend
    );
}

/// Two bands of one chain reaching the SAME facet of a standing feature is refused by name. The
/// notch's end here is the mitre between them, which is the end the split cap and the tail are not
/// drawn for - but the part says so earlier and more plainly: one face of it cannot be carried out
/// two ways at once.
///
/// A guard against the walkback reading a ring the second feature's facet was never handed, which
/// fails in the middle of the build instead of refusing with an advisory.
#[test]
fn one_facet_reached_by_two_bands_is_refused_by_name() {
    let part = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);

    // The two top edges of the notch, linked as one chain: they meet at (10, 10, 20), which is the
    // very end the nose's extension would have to run off through.
    let pair: [FeatureEdge; 2] = [
        features.edges()[top_edge_of(
            &features,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(10.0, 10.0, 20.0),
        )],
        features.edges()[top_edge_of(
            &features,
            Vector3::new(10.0, 10.0, 20.0),
            Vector3::new(20.0, 0.0, 20.0),
        )],
    ];

    let chains = bevel_chain_linker::link(&pair);
    assert_eq!(chains.len(), 1);

    let plan =
        SurfaceBevelPlan::try_create(&chains[0], &regions, 15.0, true).unwrap_or_else(|reason| {
            panic!("the mitred pair has to plan before it can be built: {reason}")
        });

    let reason = refusal_of(surface_bevel_builder::try_build_one(
        &part,
        &part.get_mesh_edge_graph(),
        &regions,
        plan,
        16,
        &CancelToken::new(),
    ))
    .unwrap_or_else(|| panic!("the nose stands under both bands of the pair"));
    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts
    );
}

/// The advisory a refused build names, `None` if it built (or was refused with nothing named).
fn refusal_of(built: Result<Mesh, SurfaceBuildFailure>) -> Option<BevelAdvisory> {
    built.err().and_then(|failure| failure.advisory().cloned())
}

/// The point an advisory names, its first argument.
fn named_at(reason: &BevelAdvisory) -> Vector3 {
    match reason.args().first() {
        Some(BevelAdvisoryArg::Vector3(at)) => *at,
        other => panic!("the advisory names a point first - instead {other:?}"),
    }
}

/// A seam of one vertex, standing where the rung says it does.
fn seam(side: usize, vertex: i32, param: f64) -> SideSeams {
    let mut seams: SideSeams = [None, None];
    seams[side] = Some(vec![(vertex, param)]);

    seams
}

/// Two seams at once, for the rungs that need both sides to say something.
fn both_seams(first: SideSeams, second: SideSeams) -> SideSeams {
    let [first0, first1] = first;
    let [second0, second1] = second;

    [first0.or(second0), first1.or(second1)]
}

/// The nose fixture's own strip and a hand-made standing feature over it, so that the ONE thing
/// each rung changes - what the walkback's seam says - is the only thing that differs from a real
/// build.
struct OneFacedReading {
    pool: SurfaceVertexPool,
    strip: SurfaceStrip,
    edge: SurfaceEdgePlan,
    feature: Rc<SurfaceStandingFeatureWithheld>,
    /// A pooled vertex standing nowhere near the nose's creases.
    away: i32,
}

impl OneFacedReading {
    /// The fixture's strip at one size, with the nose's walls as a standing feature over it.
    fn of(size: f64, segments_per_quarter: i32, one_wall_only: bool) -> Self {
        let part = Arc::new(nose_in_notch_test_meshes::create());
        let (regions, features) = regions_of(&part);
        let regions = Arc::new(regions);
        let plan = plan_of(
            &regions,
            &features,
            top_edge_of(
                &features,
                Vector3::new(0.0, 0.0, 20.0),
                Vector3::new(10.0, 10.0, 20.0),
            ),
            size,
            true,
        );

        let strip = SurfaceStrip::try_create(&plan.edges[0], segments_per_quarter)
            .unwrap_or_else(|reason| panic!("the fixture's own strip has to rule: {reason}"));

        let mut pool = SurfaceVertexPool::new(&part.vertices);
        let mut walls = vec![wall(
            &pool,
            Vector3::new(10.0, 10.0, 40.0 / 3.0),
            Vector3::new(5.0, 5.0, 10.0),
            nose_in_notch_test_meshes::upper_nose_normal(),
        )];

        if !one_wall_only {
            walls.push(wall(
                &pool,
                Vector3::new(10.0, 10.0, 20.0 / 3.0),
                Vector3::new(5.0, 5.0, 10.0),
                nose_in_notch_test_meshes::lower_nose_normal(),
            ));
        }

        let feature = Rc::new(SurfaceStandingFeatureWithheld::new(
            (0, false),
            nose_in_notch_test_meshes::left_slope_normal(),
            walls,
        ));
        let away = pool.add(Vector3::new(0.0, 0.0, 0.0));

        OneFacedReading {
            pool,
            strip,
            edge: plan.edges[0].clone(),
            feature,
            away,
        }
    }

    fn length(&self) -> f64 {
        (self.edge.to() - self.edge.from()).length()
    }

    fn corner(&self) -> Vector3 {
        self.edge.to()
    }

    /// Sets the upper wall's own outline, for the rung that is about what an outline says.
    fn outline_of_the_upper_wall(&mut self, corners: &[Vector3]) {
        let near = SurfaceVertexPool::near_enough_for(self.pool.vertices());
        let mut ring = Vec::with_capacity(corners.len());

        for &corner in corners {
            let found = self.pool.try_find_near(corner, near, None);
            ring.push(found.unwrap_or_else(|| self.pool.add(corner)));
        }

        *self.feature.faces[0].ring.borrow_mut() = Some(ring);
    }

    /// The notch the seams read, or the advisory saying why there is none.
    fn try_read(
        &mut self,
        seams: SideSeams,
    ) -> Result<SurfaceStripNotchedByAFeature, BevelAdvisory> {
        let seams = [seams];

        match SurfaceStripNotchedByAFeature::try_read(
            &mut self.pool,
            &self.strip,
            &self.edge,
            0,
            &self.feature,
            0,
            &[None, None],
            Some(&seams),
        )? {
            Some(notch) => Ok(notch),
            None => panic!("the reading came back with no notch and no refusal"),
        }
    }
}

/// One wall of the nose, as the walkback would have held it back.
fn wall(
    pool: &SurfaceVertexPool,
    dies: Vector3,
    on_the_ridge: Vector3,
    facing: Vector3,
) -> Rc<SurfaceStandingFeatureWithheldFace> {
    Rc::new(SurfaceStandingFeatureWithheldFace::new(
        Vec::new(),
        (at(pool, dies), at(pool, on_the_ridge)),
        Vec::new(),
        on_the_ridge,
        facing,
    ))
}

/// The pool's own vertex at one point of the part.
fn at(pool: &SurfaceVertexPool, point: Vector3) -> i32 {
    pool.try_find_near(
        point,
        SurfaceVertexPool::near_enough_for(pool.vertices()),
        None,
    )
    .unwrap_or_else(|| panic!("the part has no vertex at {point:?}"))
}
