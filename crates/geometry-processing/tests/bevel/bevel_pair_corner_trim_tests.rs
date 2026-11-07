//! Who trims whom at a pair corner, and what happens when the kernel says no -
//! `bevel_pair_corner_trim` on its own, without a rebuild.
//!
//! The plan is pure - candidates in, trims out, no mesh is read - so the tubes here are empty
//! meshes used only for their identity (`Rc::ptr_eq`), and every
//! assertion is about WHICH tube came back rather than what is in it. The last tests run
//! `apply` with the boolean itself passed in, which is how the production code takes it. "The
//! same cutter" means the same vertices and faces: the cutter list holds
//! owned meshes here, so a kept cutter is the one that was there and a replaced one carries the
//! trimmed mesh's geometry.

use std::rc::Rc;

use geometry_processing::bevel::bevel_bead_clip::CombineMeshes;
use geometry_processing::bevel::bevel_pair_corner_trim::{
    apply, plan, BevelPairCornerTrimCandidate, BevelPairCornerTrimTrim,
};
use polygon_mesh::csg::{BooleanError, KernelStatus};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;

/// One candidate, with a tube of its own that can be told apart from every other.
fn candidate(
    cutter_index: i32,
    ball: Option<Rc<Mesh>>,
    corners: &[i32],
) -> BevelPairCornerTrimCandidate {
    BevelPairCornerTrimCandidate::new(cutter_index, ball, Some(corners.to_vec()))
}

fn ball() -> Option<Rc<Mesh>> {
    Some(Rc::new(Mesh::new()))
}

/// The same tubes, by identity, in any order.
fn same_balls(actual: &[Rc<Mesh>], expected: &[&Rc<Mesh>]) -> bool {
    actual.len() == expected.len()
        && expected
            .iter()
            .all(|e| actual.iter().any(|a| Rc::ptr_eq(a, e)))
}

fn single(trims: &[BevelPairCornerTrimTrim], cutter_index: i32) -> &BevelPairCornerTrimTrim {
    let matching: Vec<_> = trims
        .iter()
        .filter(|trim| trim.cutter_index == cutter_index)
        .collect();
    assert_eq!(matching.len(), 1);
    matching[0]
}

fn same_mesh(a: &Mesh, b: &Mesh) -> bool {
    a.vertices == b.vertices && a.faces == b.faces
}

#[test]
fn a_run_alone_at_its_corner_is_not_trimmed() {
    // Two corners, and nobody at either of them: subtracting a chain's own tube from its own cutter
    // would be a boolean that changed nothing, and there is no partner to ask for one.
    let alone = vec![candidate(0, ball(), &[7, 9])];

    assert!(
        plan(&alone).0.is_empty(),
        "one run cannot pair with itself, however many corners it reports"
    );
}

#[test]
fn two_runs_at_one_corner_each_take_the_others_ball() {
    let ball_a = Rc::new(Mesh::new());
    let ball_b = Rc::new(Mesh::new());

    let pair = vec![
        candidate(0, Some(ball_a.clone()), &[4]),
        candidate(1, Some(ball_b.clone()), &[4]),
    ];

    let (trims, _) = plan(&pair);

    assert_eq!(trims.len(), 2, "both cutters are corrected, not just one");

    let for_a = single(&trims, 0);
    let for_b = single(&trims, 1);

    assert!(
        same_balls(&for_a.partner_balls, &[&ball_b]),
        "cutter A is trimmed by B's tube and by nothing else - its own would change nothing"
    );
    assert!(
        same_balls(&for_b.partner_balls, &[&ball_a]),
        "and symmetrically for B"
    );
}

#[test]
fn three_runs_at_one_corner_each_take_both_others() {
    let balls = [
        Rc::new(Mesh::new()),
        Rc::new(Mesh::new()),
        Rc::new(Mesh::new()),
    ];

    let three = vec![
        candidate(0, Some(balls[0].clone()), &[4]),
        candidate(1, Some(balls[1].clone()), &[4]),
        candidate(2, Some(balls[2].clone()), &[4]),
    ];

    let (trims, _) = plan(&three);

    assert_eq!(trims.len(), 3);

    for trim in &trims {
        let others: Vec<&Rc<Mesh>> = balls
            .iter()
            .enumerate()
            .filter(|(index, _)| *index as i32 != trim.cutter_index)
            .map(|(_, ball)| ball)
            .collect();
        assert!(
            same_balls(&trim.partner_balls, &others),
            "a run at a corner shared with two others is trimmed by both of their tubes in one \
             subtract - cutter {} got {}",
            trim.cutter_index,
            trim.partner_balls.len()
        );
    }
}

#[test]
fn two_runs_meeting_at_two_corners_hand_their_ball_over_only_once() {
    // The box lid: two runs of two adjacent top edges each, which meet at BOTH of the corners
    // between them. Subtracting the same tube twice is the same solid and a second import to pay
    // for.
    let twice = vec![
        candidate(0, ball(), &[4, 11]),
        candidate(1, ball(), &[4, 11]),
    ];

    let (trims, _) = plan(&twice);

    assert_eq!(trims.len(), 2);

    for trim in &trims {
        assert_eq!(
            trim.partner_balls.len(),
            1,
            "the partner is one run met at two corners, so its tube is one operand - cutter {} \
             was handed {}",
            trim.cutter_index,
            trim.partner_balls.len()
        );
    }
}

#[test]
fn a_run_whose_tube_would_not_sweep_is_still_trimmed_itself() {
    // A tube that folded is not a tube anyone can subtract, and it is not a reason to leave the run
    // that DID get one uncorrected.
    let ball_b = Rc::new(Mesh::new());

    let lopsided = vec![
        candidate(0, None, &[4]),
        candidate(1, Some(ball_b.clone()), &[4]),
    ];

    let (trims, _) = plan(&lopsided);

    assert_eq!(
        trims.len(),
        1,
        "only the cutter with a partner tube to subtract has anything to do"
    );
    assert_eq!(
        trims[0].cutter_index, 0,
        "it is the run whose OWN tube is missing that still gets trimmed - by its partner's"
    );
    assert!(same_balls(&trims[0].partner_balls, &[&ball_b]));
}

#[test]
fn a_partner_with_no_tube_is_reported_so_the_user_can_be_told() {
    // The quiet fallback this report exists to give a voice to. Cutter 1 meets cutter 0 at corner
    // 4, so a rolling ball's corner was there to be had, and 0's tube would not sweep - which means
    // 1 goes into the union untrimmed and the ridge plan 5.3 leaves stays in the finished part.
    // Nothing downstream can notice that afterwards: a cutter left untrimmed is indistinguishable
    // from one that never had a partner at all, so if it is not said here it is not said anywhere.
    let lopsided = vec![candidate(0, None, &[4]), candidate(1, ball(), &[4])];

    let (_, left_untrimmed) = plan(&lopsided);

    assert_eq!(
        left_untrimmed,
        vec![1],
        "it is the cutter whose PARTNER had no tube that keeps its ridge - cutter 0 was trimmed by \
         1's tube and has nothing to report"
    );

    let (_, both_tubed) = plan(&[candidate(0, ball(), &[4]), candidate(1, ball(), &[4])]);

    assert!(
        both_tubed.is_empty(),
        "a corner both runs can roll a ball round is the corner the user asked for, and there is \
         nothing to advise about it"
    );
}

#[test]
fn runs_at_different_corners_are_not_partners() {
    let far = vec![candidate(0, ball(), &[4]), candidate(1, ball(), &[11])];

    assert!(
        plan(&far).0.is_empty(),
        "two runs are partners because they MEET at a vertex, not because both overshoot \
         somewhere - a tube reaches only the corner it was swept through"
    );
}

/// The trim as the builder runs it, over a cutter list of one, with the boolean handed in.
fn apply_to_one_cutter(cutter: Mesh, boolean: &CombineMeshes<'_>) -> Mesh {
    let mut cutters = vec![cutter];

    let (trims, _) = plan(&[candidate(0, None, &[4]), candidate(1, ball(), &[4])]);

    apply(&mut cutters, &trims, boolean).expect("no cancellation");

    cutters.remove(0)
}

#[test]
fn a_kernel_refusal_leaves_the_cutter_exactly_as_it_was() {
    let cutter = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let kept = apply_to_one_cutter(cutter.clone(), &|_, _| {
        Err(BooleanError::Partial {
            message: "the kernel refused an operand".to_string(),
            partial_result: Box::new(Mesh::new()),
            skipped_operands: Vec::new(),
        })
    });

    assert!(
        same_mesh(&kept, &cutter),
        "a refused subtract leaves plan 5.3's corner standing - the shape that shipped before this \
         stage existed - rather than taking the whole rebuild down with it"
    );
}

#[test]
fn a_result_the_kernel_will_not_vouch_for_leaves_the_cutter_as_it_was() {
    let cutter = platonic_solids::create_cube(10.0, 10.0, 10.0);

    // A boolean with no answer hands back an empty mesh, which no operand check vouches for.
    let kept = apply_to_one_cutter(cutter.clone(), &|_, _| Ok(Mesh::new()));

    assert!(
        same_mesh(&kept, &cutter),
        "the trimmed cutter is validated like every other operand before it replaces one"
    );
}

/// The other voice a refusal comes in. A subtract is not a union, so a kernel that will not take
/// an operand does not skip it and carry on - on a non-union mode it rejects the import outright,
/// and the fallback has to be the one the module documents.
#[test]
fn a_refused_operand_leaves_the_cutter_as_it_was_rather_than_failing_the_rebuild() {
    let cutter = platonic_solids::create_cube(10.0, 10.0, 10.0);

    let kept = apply_to_one_cutter(cutter.clone(), &|_, _| {
        Err(BooleanError::ImportRejected {
            message: "the kernel would not import an operand".to_string(),
            status: KernelStatus::NotManifold,
        })
    });

    assert!(
        same_mesh(&kept, &cutter),
        "a corner treatment that cannot be had leaves plan 5.3's corner standing, and a part that \
         built before this stage existed must not stop building because of it"
    );
}

#[test]
fn a_trimmed_cutter_replaces_the_one_it_was_made_from() {
    let cutter = platonic_solids::create_cube(10.0, 10.0, 10.0);
    let trimmed = platonic_solids::create_cube(8.0, 8.0, 8.0);
    let handed_back = trimmed.clone();

    let kept = apply_to_one_cutter(cutter, &move |_, _| Ok(handed_back.clone()));

    assert!(
        same_mesh(&kept, &trimmed),
        "the whole point is that the union downstream unions the TRIMMED cutter"
    );
}

#[test]
fn no_candidates_is_no_work() {
    // No candidates at all is the empty slice.
    assert!(
        plan(&[]).0.is_empty(),
        "a selection with no pair corner in it - which is most of them - costs no boolean"
    );

    assert!(plan(&Vec::new()).0.is_empty());
}
