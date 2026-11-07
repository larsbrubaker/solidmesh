//! The MITRE ([`try_mitre`]): the ring two consecutive strips of
//! one chain share where they meet, cut from the plane their two walk directions are mirror images
//! in. Its rail-by-rail machinery - where each rail crosses that plane and where the two beads'
//! tangent lines cross on the face they share - is `surface_strip_mitre_rails`; the corners that
//! are not one plain ring are [`super::SurfaceMitreOverTheFaceBeyond`] (the tip past the shared
//! face's rim), [`SurfaceMitreSliver`] (the feet past the far rims), `surface_mitre_chorded_apart`
//! (two arcs chorded at different steps) and [`super::surface_mitre_sector`] (the far face giving up
//! a sector). It stands apart from `surface_strip_ends`, the three ways a strip STOPS, because the
//! mitre is the one of the three with a machine behind it.

// The negated comparisons are deliberate: each is false for NaN, which is the point.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use vector_math::{math_helper, Vector3};

use super::surface_mitre_chorded_apart::{self, ChordedApartSide};
use super::surface_mitre_joint_step::{self, SideBandCorners, SideSeams};
use super::surface_mitre_sliver::SurfaceMitreSliverRun;
use super::surface_strip_mitre_rails::{self as rails, SurfacePlane};
use super::{surface_mitre_sector, SurfaceMitreHull, SurfaceMitreOverTheFaceBeyond};
use super::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceMitreSliver, SurfaceStrip, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How near two arcs' turns have to be before they count as the same turn, in degrees. Two beads of
/// one wedge are the same to the float's last bit, and the smallest turn any corner of the corpus
/// really differs by is the raked bore mouth's 1.39 degrees, so nothing lies between this and a
/// difference that means something.
const TURNS_THE_SAME: f64 = 1e-9;

/// What [`try_mitre`] cut: the six readings it hands back.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceStripMitreRing {
    /// The leaving strip's end ring, one vertex per rail.
    pub ring: Vec<i32>,
    /// The arriving strip's start ring, one vertex per rail.
    pub next_ring: Vec<i32>,
    /// The parameter along the leaving strip each of its rails ends at.
    pub leaving_at: Vec<f64>,
    /// The parameter along the arriving strip each of its rails starts at.
    pub arriving_at: Vec<f64>,
    /// What the leaving ring carries on each of its chords.
    pub between_leaving: Vec<Vec<i32>>,
    /// What the arriving ring carries on each of its chords.
    pub between_arriving: Vec<Vec<i32>>,
}

/// The two strips meeting at a mitre and their plans and edge indices.
#[derive(Clone, Copy)]
pub struct SurfaceStripMitrePair<'a> {
    /// The strip the walk carries the chain into the corner along.
    pub strip: &'a SurfaceStrip,
    /// The strip after it.
    pub next: &'a SurfaceStrip,
    /// `strip`'s plan.
    pub leaving: &'a SurfaceEdgePlan,
    /// `next`'s plan.
    pub arriving: &'a SurfaceEdgePlan,
    /// `strip`'s edge index.
    pub edge_index: i32,
    /// `next`'s edge index.
    pub next_index: i32,
}

/// The ring where two consecutive strips meet - each rail cut at the mitre plane,
/// which is the plane the two walk directions are mirror images in. Built from the first strip and
/// CHECKED against the second, so the two share the ring's vertices rather than arriving at two of
/// them a rounding apart - and so that a corner where they genuinely do not meet is a refusal.
///
/// The plane's NORMAL is the sum of the two walk directions - the mirror carrying the chain's
/// arrival onto its departure. Where it stands is the crossing of the two beads' tangent lines on
/// the face they SHARE ([`rails::try_the_seam_point`]), which is the only point both surfaces are
/// guaranteed to hold: each bead's own tangent line is where its strip meets that face, so the point
/// the two lines cross is on both strips at once whatever their setbacks are.
///
/// Wherever the two beads set back by the SAME amount on that face - two 90 degree edges of a cube,
/// which is every mitre in the corpus so far - that crossing stands at the same distance along each
/// tangent line and the plane through it is the plane through the CORNER VERTEX
/// (`surface_corner_tests::an_equal_setback_mitre_is_unchanged` pins that they remain one plane).
/// They part company as soon as the two wedges differ: a 45 degree knife tip sets its line back
/// r/tan(22.5) where a 90 degree edge sets its own back r, and a plane through the corner then
/// misses the other strip entirely.
#[allow(clippy::too_many_arguments)]
pub fn try_mitre(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    pair: SurfaceStripMitrePair<'_>,
    seams: &[SideSeams],
    band_corners: Option<&[SideBandCorners]>,
    corner: Vector3,
    the_chorded_runs_joint: Option<Vector3>,
    hull: Option<SurfaceMitreHull<'_>>,
) -> Result<SurfaceStripMitreRing, BevelAdvisory> {
    let SurfaceStripMitrePair {
        strip,
        next,
        leaving,
        arriving,
        edge_index,
        next_index,
    } = pair;
    let do_not_mitre = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripsDoNotMitre,
            vec![corner.into()],
        )
    };

    // The mitre plane through the corner is the one each strip is the other's mirror image in, and
    // the mirror that carries the chain's arrival onto its departure - d1 to -d2 - is the one whose
    // normal is d1 + d2. Two collinear edges give 2 d1, which is the square cut a straight run
    // wants; two edges that double straight back on each other give nothing, and there is no mitre
    // there.
    let mut normal = strip.along() + next.along();

    // How far the chain turns at this corner, bounded by the same angle the plan bounds a wedge by
    // (`SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES`). A length test here would be no test at
    // all: the sum is 2 cos(turn / 2) long, so a chain doubling back through 179.99 degrees still
    // leaves a vector 1.7e-4 long - far above any epsilon, and a mitre plane so nearly along the
    // rails that the ring it cuts lands metres from the part.
    let turn_degrees =
        math_helper::radians_to_degrees(strip.along().dot(next.along()).clamp(-1.0, 1.0).acos());

    if !(turn_degrees < SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES) {
        return Err(do_not_mitre());
    }

    normal = normal.get_normal();

    let through =
        rails::try_the_seam_point(pool, leaving, arriving, corner, the_chorded_runs_joint)?;

    // Which rail of each strip stands on the face the two SHARE, which is where the pairing counts
    // from. Rail 0 is the left of the walk and the last rail its right, the same convention the plan
    // names its regions by.
    let (Some(mine_on_the_face), Some(theirs_on_the_face)) = (
        rails::try_the_shared_rail(strip, leaving, arriving),
        rails::try_the_shared_rail(next, arriving, leaving),
    ) else {
        return Err(do_not_mitre());
    };

    let mine_step = if mine_on_the_face == 0 { 1 } else { -1 };
    let theirs_step = if theirs_on_the_face == 0 { 1 } else { -1 };
    let mut paired = strip.rail_count().min(next.rail_count());

    // WHICH of the two is the wider bead is the only thing that differs between the two ways round
    // a corner can be walked. The unpaired rails always belong to the wider strip and always end on
    // the NARROWER one's far landing plane, so the pair is named once here and the two constructions
    // below are asked of the wider one, whether the walk leaves by it or arrives along it.
    //
    // Read by how far each ARC TURNS and not by how many rails it was cut into. The two agree
    // wherever both strips took the same step - the ordinary corner, where a rail more is a step
    // more of one turn - and part where the arcs are chorded apart: on the mouth of a raked bore two
    // neighbouring chords sweep 45.176 and 46.562 degrees in four chords each, so the rail counts
    // TIE while the second bead is the wider by a degree and a third. Tied on the count, the wrong
    // one was called wider, its own last rail was left unpaired, and the narrower's crossing inside
    // that unpaired chord was read as a surface outlasting the bead it is mitred against - refused
    // as SurfaceStripsDoNotMitre at a corner where nothing is wrong. A tie in the turn keeps the
    // count as the tiebreak, which is what the equal-wedge corner has always used.
    let mine_sweeps = surface_mitre_chorded_apart::sweep_of(strip);
    let theirs_sweeps = surface_mitre_chorded_apart::sweep_of(next);

    let the_wider_arrives = if (theirs_sweeps - mine_sweeps).abs() > TURNS_THE_SAME {
        theirs_sweeps > mine_sweeps
    } else {
        next.rail_count() > strip.rail_count()
    };

    let pick = |arrives: bool| {
        if arrives {
            (next, arriving, next_index, theirs_on_the_face, theirs_step)
        } else {
            (strip, leaving, edge_index, mine_on_the_face, mine_step)
        }
    };
    let (wider, wider_plan, wider_index, wider_on_the_face, wider_step) = pick(the_wider_arrives);
    let (narrower, narrower_plan, narrower_index, narrower_on_the_face, narrower_step) =
        pick(!the_wider_arrives);

    // Along the wider strip's own edge, AWAY from the corner: the direction the section it hands
    // back runs in. The walk carries the chain INTO the corner along the strip it leaves by and OUT
    // of it along the one it arrives at, so the two ways round the corner take opposite signs of the
    // same edge direction.
    let away_from_the_corner = if the_wider_arrives {
        arriving.edge.direction()
    } else {
        -leaving.edge.direction()
    };

    // The narrower strip's far landing plane: it holds that strip's far tangent line, and it is the
    // region's own plane, so its normal is the line's direction turned into the face. Read once,
    // because the unpaired rails END on it and the sector the face gives up is CLIPPED on it.
    let their_far = narrower_on_the_face + (narrower_step * (narrower.rail_count() - 1));
    let beyond = SurfacePlane {
        point: narrower.point_on(their_far, 0.0),
        normal: narrower
            .along()
            .cross(if their_far == 0 {
                narrower_plan.left_inward
            } else {
                narrower_plan.right_inward
            })
            .get_normal(),
    };

    if !(beyond.normal.length_squared() > 0.0) {
        return Err(do_not_mitre());
    }

    // And the WIDER strip's own far landing plane, read the same way. Nothing asks for it at an
    // ordinary mitre; at a REFLEX corner it is the face the narrower run's band ended against, and
    // it is what bounds the patch that run's arc goes on to cover (SurfaceMitreSliver).
    let my_far = wider_on_the_face + (wider_step * (wider.rail_count() - 1));
    let wider_beyond = SurfacePlane {
        point: wider.point_on(my_far, 0.0),
        normal: wider
            .along()
            .cross(if my_far == 0 {
                wider_plan.left_inward
            } else {
                wider_plan.right_inward
            })
            .get_normal(),
    };

    let away_from_the_corner_narrower = if the_wider_arrives {
        -leaving.edge.direction()
    } else {
        arriving.edge.direction()
    };

    // And whether the mitre's TIP stands past the shared face's own rim, which throws the rails
    // nearest that face off the part altogether: those are cut on the face beyond it instead, and
    // the face beyond gains the lens between the two exit curves (SurfaceMitreOverTheFaceBeyond).
    let over_the_face = SurfaceMitreOverTheFaceBeyond::try_read(
        pool,
        strip,
        next,
        leaving,
        arriving,
        seams,
        edge_index,
        next_index,
        mine_on_the_face,
        mine_step,
        theirs_on_the_face,
        theirs_step,
        paired,
        corner,
        through,
        normal,
        hull,
    )?;

    // And whether the mitre's two FEET stand past their own runs' far rims, which is the reflex
    // corner of two unequal setbacks: the surplus rails are then cut on the mitre plane like every
    // other rail, the far face gives up no sector, and the narrower run's arc covers the patch
    // between the two rims (SurfaceMitreSliver).
    //
    // WHICH END of each edge's band this corner is, which is what says whose corner the sliver's
    // foot may take: the walk carries the chain INTO this corner along the strip it leaves by, so
    // edge_index is read at its LEAVING end and next_index at its ARRIVING one (SurfaceBevelBuilder's
    // band corner array is keyed 0 arriving, 1 leaving).
    let wider_end = if the_wider_arrives { 0 } else { 1 };
    let narrower_end = if the_wider_arrives { 1 } else { 0 };

    let mut sliver = if over_the_face.is_none() {
        SurfaceMitreSliver::try_read(
            pool,
            SurfaceMitreSliverRun {
                strip: narrower,
                plan: narrower_plan,
                on_the_face: narrower_on_the_face,
                step: narrower_step,
                edge: narrower_index,
                end: narrower_end,
                away_from_the_corner: away_from_the_corner_narrower,
            },
            SurfaceMitreSliverRun {
                strip: wider,
                plan: wider_plan,
                on_the_face: wider_on_the_face,
                step: wider_step,
                edge: wider_index,
                end: wider_end,
                away_from_the_corner,
            },
            seams,
            band_corners,
            through,
            normal,
            wider_beyond.point,
            wider_beyond.normal,
            narrower.outward_reference,
        )?
    } else {
        None
    };

    let mut ring = vec![0; strip.rail_count() as usize];
    let mut next_ring = vec![0; next.rail_count() as usize];
    let mut leaving_at = vec![0.0; strip.rail_count() as usize];
    let mut arriving_at = vec![0.0; next.rail_count() as usize];

    // The two arcs' own angular steps. Where they are ONE step, rail k of each strip stands at one
    // angle and the two cross the mitre plane at one point, which is the pairing below. Where they
    // are not, the pairing is by angle instead and each strip carries the other's crossings BETWEEN
    // its own rails (SurfaceMitreChordedApart) - one ring per rail either way.
    let chorded_apart = over_the_face.is_none()
        && surface_mitre_chorded_apart::the_pairing_by_index_misses(
            pool,
            strip,
            next,
            mine_on_the_face,
            mine_step,
            theirs_on_the_face,
            theirs_step,
            paired,
            through,
            normal,
        );

    let mut union_leaving: Option<Vec<Vec<i32>>> = None;
    let mut union_arriving: Option<Vec<Vec<i32>>> = None;

    // Where the WIDER strip leaves the mitre plane for the face beyond, when that is no rail of its
    // own - see SurfaceMitreChordedApart.
    let mut transition_on_mine = -1;
    let mut transition_on_theirs = -1;

    if chorded_apart {
        let chorded = surface_mitre_chorded_apart::try_the_rails(
            pool,
            ChordedApartSide {
                strip,
                edge: edge_index,
                on_the_face: mine_on_the_face,
                step: mine_step,
            },
            ChordedApartSide {
                strip: next,
                edge: next_index,
                on_the_face: theirs_on_the_face,
                step: theirs_step,
            },
            seams,
            corner,
            through,
            normal,
            sliver.as_ref(),
            &mut ring,
            &mut next_ring,
            &mut leaving_at,
            &mut arriving_at,
        )?;

        union_leaving = Some(chorded.between_leaving);
        union_arriving = Some(chorded.between_arriving);
        transition_on_mine = chorded.transition_on_mine;
        transition_on_theirs = chorded.transition_on_theirs;

        // Counted by angle rather than by rail, the wider strip's own tally of mitred rails is what
        // the face beyond has to open for the rest of.
        paired = if the_wider_arrives {
            chorded.paired_on_theirs
        } else {
            chorded.paired_on_mine
        };
    }

    for step in 0..(if chorded_apart { 0 } else { paired }) {
        let mine = mine_on_the_face + (mine_step * step);
        let theirs = theirs_on_the_face + (theirs_step * step);

        if let Some(over) = over_the_face.as_ref().filter(|over| step < over.run_off) {
            over.try_the_rails(
                pool,
                strip,
                next,
                seams,
                edge_index,
                next_index,
                mine,
                theirs,
                step,
                &mut ring,
                &mut next_ring,
                &mut leaving_at,
                &mut arriving_at,
            )?;

            continue;
        }

        if let Some(sliver) = sliver.as_ref().filter(|sliver| {
            sliver.claims_the_foot(edge_index, mine) || sliver.claims_the_foot(next_index, theirs)
        }) {
            // The patch has claimed one of these two, so the pair no longer stands at one point:
            // the claimed foot stops at its rim and its partner is cut on the mitre plane alone.
            rails::try_one_rail_apart(
                pool,
                strip,
                edge_index,
                mine,
                corner,
                through,
                normal,
                sliver,
                &mut ring,
                &mut leaving_at,
            )?;
            rails::try_one_rail_apart(
                pool,
                next,
                next_index,
                theirs,
                corner,
                through,
                normal,
                sliver,
                &mut next_ring,
                &mut arriving_at,
            )?;

            continue;
        }

        let (vertex, here, there) = rails::try_one_pair_of_the_mitre(
            pool, strip, next, seams, edge_index, next_index, mine, theirs, corner, through, normal,
        )?;
        leaving_at[mine as usize] = here;
        arriving_at[theirs as usize] = there;

        ring[mine as usize] = vertex;
        next_ring[theirs as usize] = vertex;
    }

    // And the wider strip's rails with no partner, which end on the narrower one's FAR landing
    // plane - the part's own face past the mitre, which the narrower bead never reached and which
    // therefore still stands there - and then that face opened for them (SurfaceMitreSector).
    let (wider_ring, wider_at) = if the_wider_arrives {
        (&mut next_ring, &mut arriving_at)
    } else {
        (&mut ring, &mut leaving_at)
    };

    let between = if let Some(sliver) = sliver.as_mut() {
        // The far face here is the PARTNER's own wall, banded to the crease already: nothing lands
        // on it and it gives up nothing, so the surplus rails are cut on the mitre plane like the
        // rest and the patch between the two rims is sewn below.
        sliver.try_the_rails_over_the_narrowers_arc(
            pool,
            wider,
            wider_on_the_face,
            wider_step,
            paired,
            corner,
            through,
            normal,
            wider_ring,
            wider_at,
        )?;

        surface_mitre_sector::nothing_along(wider_ring.len() as i32)
    } else {
        rails::try_the_rails_with_no_partner(
            pool,
            wider,
            seams,
            wider_index,
            corner,
            wider_on_the_face,
            wider_step,
            paired,
            beyond,
            wider_ring,
            wider_at,
        )?;
        surface_mitre_sector::try_give_up_the_sector(
            pool,
            faces,
            wider_plan,
            away_from_the_corner,
            wider_ring,
            wider_on_the_face,
            paired,
            corner,
            through,
            normal,
            beyond.point,
            beyond.normal,
            if the_wider_arrives {
                transition_on_theirs
            } else {
                transition_on_mine
            },
        )?
    };

    if (if the_wider_arrives {
        transition_on_mine
    } else {
        transition_on_theirs
    }) >= 0
    {
        // The NARROWER bead has a crossing past the wider one's last paired rail, which would mean
        // the narrower surface outlasting the wider one it is mitred against. There is no face
        // beyond for it to hand over to - the sector is the wider strip's alone - so it is refused
        // rather than laid somewhere it does not belong.
        return Err(do_not_mitre());
    }

    // The sector's leavings belong to the WIDER strip's station - it is the one whose chords the
    // far face's own triangulation ran along - so which of the two rings carries them is which way
    // round the corner was walked.
    let (leaving_sector, arriving_sector) = if the_wider_arrives {
        (
            surface_mitre_sector::nothing_along(ring.len() as i32),
            between,
        )
    } else {
        (
            between,
            surface_mitre_sector::nothing_along(next_ring.len() as i32),
        )
    };
    let mut between_leaving = rails::merge(leaving_sector, union_leaving.as_deref());
    let mut between_arriving = rails::merge(arriving_sector, union_arriving.as_deref());

    if let Some(sliver) = sliver.as_ref() {
        let transition = if the_wider_arrives {
            transition_on_theirs
        } else {
            transition_on_mine
        };

        if transition >= 0 {
            // Chorded apart, the NARROWER foot's crossing fell past the wider strip's last paired
            // rail and the spread handed it back rather than laying it down. With the surplus rails
            // now cut on the mitre plane it is an ordinary point between two of them - the last
            // paired rail and the first surplus one - and it stands beside the latter.
            rails::beside(
                if the_wider_arrives {
                    &mut between_arriving
                } else {
                    &mut between_leaving
                },
                (wider_on_the_face + (wider_step * (paired - 1)))
                    .min(wider_on_the_face + (wider_step * paired)),
                transition,
                wider_step < 0,
            );
        }

        // Each claimed foot stopped at its rim, so its END RING steps out to its crossing along the
        // chord beside it - the same staircase the sector's own leavings travel by. AFTER the
        // transition, and that ORDER is the whole of the last fold at a chorded-apart sliver
        // corner. On the mitre plane the two feet's crossings stand at the two runs' own sweeps, so
        // the narrower's - which is the transition the spread handed back - stands INSIDE the
        // wider's: at the raked bore's corner between chords 1 and 2 at 45.176 degrees against
        // 46.562. Laid the other way round the wider strip's last row read its chord 607, 601, 600,
        // 587 and drew a triangle across the very segment the patch's arc side covers, walking it
        // 601 to 600 exactly as the patch does - two pieces back to back rather than joined, one at
        // every surplus corner. Beside APPENDS where the foot is the chord's higher rail and
        // PREPENDS where it is the lower, so putting the transition down first leaves the foot's
        // own crossing outermost either way round.
        rails::beside(
            &mut between_leaving,
            sliver.the_step_chord_of(edge_index),
            sliver.the_crossing_of(edge_index),
            sliver.the_step_leads(edge_index),
        );
        rails::beside(
            &mut between_arriving,
            sliver.the_step_chord_of(next_index),
            sliver.the_crossing_of(next_index),
            sliver.the_step_leads(next_index),
        );

        sliver.try_sew(pool, faces)?;
    }

    if let Some(over) = over_the_face.as_ref() {
        let turn_vertex = over.try_gain_the_lens(
            pool,
            faces,
            &ring,
            &next_ring,
            mine_on_the_face,
            mine_step,
            theirs_on_the_face,
            theirs_step,
        );

        // The turn point stands part way along the chord where each strip's end leaves the face
        // beyond for the mitre plane, so it belongs to both stations - a strip sewn past it would
        // draw a straight edge across the very corner its own end turns at.
        rails::on_the_chord(
            &mut between_leaving,
            mine_on_the_face + (mine_step * (over.run_off - 1)),
            mine_step,
            turn_vertex,
        );
        rails::on_the_chord(
            &mut between_arriving,
            theirs_on_the_face + (theirs_step * (over.run_off - 1)),
            theirs_step,
            turn_vertex,
        );
    }

    // And the STEP at the joint itself, on whichever side nothing above laid it: the wedge between
    // the two cuts' near rails, which is ground the run, the corner list and the stitch each own a
    // piece of and none of them covers (SurfaceMitreJointStep). Last, because it reads the faces
    // already laid to find the hole - a side the sliver's patch and ear closed leaves none.
    surface_mitre_joint_step::sew(
        faces,
        Some(seams),
        band_corners,
        edge_index as usize,
        next_index as usize,
    );

    Ok(SurfaceStripMitreRing {
        ring,
        next_ring,
        leaving_at,
        arriving_at,
        between_leaving,
        between_arriving,
    })
}
