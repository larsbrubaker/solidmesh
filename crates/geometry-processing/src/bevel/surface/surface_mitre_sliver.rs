//! WHAT THE NARROWER RUN'S ARC GOES ON TO COVER at a mitre whose feet are cut PAST the far landing
//! face's own rim ([`SurfaceMitreSliver`]): at a REFLEX corner the two runs' far faces are the two
//! walls of the inside corner and they meet along a crease, and a mitre of unequal setbacks cuts
//! each strip's foot rail past that crease rather than on it - so the narrower run's arc continues
//! past its own far tangent line to the wider run's sweep, and the little patch it covers there is
//! this. This file reads the patch and hands its feet and crossings to the mitre; sewing it is
//! `surface_mitre_sliver_sew`.
//!
//! **Why the feet stand past the rim at all.** Each bead sets its tangent line back on the shared
//! face by its own wedge, so the mitre plane - the one through the crossing of those two lines -
//! leans toward the narrower run, and at the far end that lean carries it past the crease the two
//! far faces meet along. On the raked L at radius 1 - a square wall and a wall raked 15 degrees
//! under the part - the plane is `x - y = 0.3032`, which cuts the square run's foot rail (`x = 0,
//! z = -1`) at `y = -0.3032` while that wall's rim stands at `y = z tan 15 = -0.2679`: 0.0353 past
//! it. Where the two setbacks are EQUAL - the same reflex corner of two square wedges - the plane
//! passes through the crease exactly, which is why this fires on the two SWEEPS differing rather
//! than on the corner being reflex.
//!
//! **What the part really carries there.** A rolling ball leaves the higher of the two runs'
//! cylinders, and past the narrower run's own tangent line that is still the narrower run's
//! cylinder - its lower branch, standing where the wall it was tangent to has ended and the
//! partner's band has taken the material beside it away. The patch is bounded by four things and
//! closes exactly: the narrower foot's own step from its rim out to its crossing, the mitre
//! polyline over the angles the narrower run has no rails at, the wider foot's step back to ITS
//! rim, and the crease between the two rims. Its far corner is one point three surfaces meet at -
//! the mitre plane, the wider run's far landing plane and the narrower run's cylinder - and that
//! point IS the wider foot's own crossing, which is checked rather than assumed.
//!
//! **And the far landing face gives up nothing.** [`super::surface_mitre_sector`] is the other
//! reading of a mitre of unequal setbacks - the knife tip, where the narrower edge's far landing
//! face is a face of the part nobody touched and the wider strip's surplus rails LAND on it, so
//! that face hands over a sector. Here that face is the partner's own wall, banded to the crease
//! already, and the surplus rails do not land on it at all: they are cut on the mitre plane like
//! every other rail. No sector is taken, and the two readings never both fire - the sector wants
//! the feet inside the far tangent line and this wants them past it.
//!
//! **What it assumes, and refuses by name where it cannot have it.** That the narrower run's rim
//! stands ON the wider run's far landing plane - which is what makes the crease the one rim both
//! bands end at - refused as `SurfaceStripEndNotSupported`; and that the wider foot's crossing
//! stands a radius from the narrower run's own centre line, which is the two ends turning at one
//! point, refused as `SurfaceStripSeamDidNotMeet`. Two runs of ONE sweep are left alone, so a
//! corner where both feet overrun by the same amount is not this reading and keeps whatever
//! complaint it already had.

// The negated comparisons are deliberate: each is false for NaN, which is the point.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use vector_math::Vector3;

use super::surface_mitre_chorded_apart;
use super::surface_mitre_joint_step::{SideBandCorners, SideSeams};
use super::{SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One run of a sliver read: the strip, its plan, where its rails count from, which edge and which
/// end of that edge's band the corner is, and the way along its edge away from the corner.
#[derive(Clone, Copy)]
pub struct SurfaceMitreSliverRun<'a> {
    /// The run's strip.
    pub strip: &'a SurfaceStrip,
    /// The run's edge plan.
    pub plan: &'a SurfaceEdgePlan,
    /// The strip's rail standing on the face the two strips share.
    pub on_the_face: i32,
    /// +1 where the rails count up from that face, -1 where they count down.
    pub step: i32,
    /// The run's edge index.
    pub edge: i32,
    /// Which END of the run's own band this corner is - 0 where the chain arrives at that edge and
    /// 1 where it leaves it - which is the only end whose corners this foot may take.
    pub end: usize,
    /// Along the run's edge, away from the corner.
    pub away_from_the_corner: Vector3,
}

/// The patch the narrower run's arc covers past its far rim.
#[derive(Debug, Clone)]
pub struct SurfaceMitreSliver {
    pub(super) narrower_edge: i32,
    pub(super) narrower_foot: i32,
    pub(super) narrower_rim: i32,
    narrower_rim_param: f64,
    pub(super) narrower_crossing: i32,
    /// The band corner the narrower foot's two-plane identity NAMED, before any nearness budget, or
    /// -1 where none was named or where the one named is the foot's own rim or its own crossing. It
    /// is what the wall's rim really turns at, which the straight rail's crossing is not.
    pub(super) narrower_named: i32,
    /// Where the narrower foot's crossing itself stands, kept so a taken vertex can be given back.
    #[allow(dead_code)]
    narrower_crossing_at: Vector3,
    wider_edge: i32,
    wider_foot: i32,
    pub(super) wider_rim: i32,
    wider_rim_param: f64,
    pub(super) wider_crossing: i32,
    /// The same for the wider foot's.
    pub(super) wider_named: i32,
    /// The same for the wider foot's.
    #[allow(dead_code)]
    wider_crossing_at: Vector3,
    /// Whether either of the two crossings was TAKEN as its rim rather than minted.
    pub(super) took_the_rims: bool,
    pub(super) outward: Vector3,
    /// The wider run's crossings over the angles the narrower run has no rails at, foot last.
    pub(super) over_the_narrowers_arc: Vec<i32>,
}

/// What [`the_foot_overruns`] reads of one foot: its rim and the rim's parameter, the crossing,
/// the pooled corner the crossing IS (or -1) and the corner the identity gates NAMED (or -1).
struct FootOverrun {
    rim: i32,
    rim_param: f64,
    crossing: Vector3,
    corner: i32,
    named: i32,
}

impl SurfaceMitreSliver {
    /// Whether this mitre's two feet stand past their runs' far rims, and where everything the
    /// patch is bounded by stands. `Ok(None)` at every mitre this is not, so the ordinary reading
    /// keeps its own complaint.
    ///
    /// `outward` is which way the patch faces, to within a right angle: the narrower run's own
    /// [`SurfaceStrip::outward_reference`], which is what every triangle of that run is wound by.
    /// Read from the strip rather than from its far landing plane, whose normal is the walk
    /// direction crossed into the face and so turns over with the direction the chain happened to
    /// be walked - the plane is the same plane either way and the winding is not.
    #[allow(clippy::too_many_arguments)]
    pub fn try_read(
        pool: &mut SurfaceVertexPool,
        narrower: SurfaceMitreSliverRun<'_>,
        wider: SurfaceMitreSliverRun<'_>,
        seams: &[SideSeams],
        band_corners: Option<&[SideBandCorners]>,
        through: Vector3,
        normal: Vector3,
        wider_beyond_point: Vector3,
        wider_beyond_normal: Vector3,
        outward: Vector3,
    ) -> Result<Option<SurfaceMitreSliver>, BevelAdvisory> {
        if !(surface_mitre_chorded_apart::sweep_of(wider.strip)
            > surface_mitre_chorded_apart::sweep_of(narrower.strip))
        {
            // Two runs of one sweep have no rails past each other and no patch between them: every
            // mitre of two equal setbacks, and the reflex corner of two square wedges with them.
            return Ok(None);
        }

        let narrower_foot =
            narrower.on_the_face + (narrower.step * (narrower.strip.rail_count() - 1));
        let wider_foot = wider.on_the_face + (wider.step * (wider.strip.rail_count() - 1));

        let read = |run: &SurfaceMitreSliverRun<'_>, foot: i32| {
            the_foot_overruns(
                pool,
                run.strip,
                run.plan,
                seams,
                band_corners,
                run.edge,
                run.end,
                foot,
                through,
                normal,
                run.away_from_the_corner,
            )
        };

        // One of the two feet landing where the walkback left its seam, or short of it, or on no
        // crossing at all, is none of this reading. (The wider foot is read only when the narrower
        // one overran.)
        let Some(n) = read(&narrower, narrower_foot) else {
            return Ok(None);
        };
        let Some(w) = read(&wider, wider_foot) else {
            return Ok(None);
        };

        if !(wider_beyond_normal.length_squared() > 0.0)
            || (pool.position_of(n.rim) - wider_beyond_point)
                .dot(wider_beyond_normal.get_normal())
                .abs()
                > pool.near_enough()
        {
            // The narrower band's rim does not stand on the wider run's far face, so the two bands
            // do not end at ONE rim and there is no crease for the patch's far side to run along.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripEndNotSupported,
                vec![pool.position_of(n.rim).into()],
            ));
        }

        // The narrower run's own radius, read off its foot rail rather than off the plan: every
        // rail of a round stands a radius from the center line, and the plan's own CenterDistance
        // is the distance to the EDGE along the bisector (r / sin(wedge / 2)) rather than the
        // radius.
        let radius = narrower
            .plan
            .center_line
            .distance_to(narrower.strip.point_on(narrower_foot, 0.0));

        // THE CROSSING'S IDENTITY, read before anything is minted. A foot overrun is a step of a
        // few welds, so a crossing can stand a ruler's width from the very rim vertex its own run
        // ended at and still be the SAME PLACE as it: minting it then gives the patch a twin of a
        // vertex the wall's own faces are sewn to, the patch bounds itself by the twin and the
        // stitch by the wall's, and between them they lay one triangle twice. Where the crossing is
        // within the pool's ruler of that rim vertex it IS that vertex - identity, not a fresh
        // mint - and the reach is that vertex alone: taking the NEAREST pooled vertex anywhere
        // closes the same rows but drags in a vertex off the crossing's own rim, which collapses
        // the patch.
        let narrower_is_its_rim = the_crossing_is_its_rim(pool, n.rim, n.crossing);
        let mut wider_is_its_rim = the_crossing_is_its_rim(pool, w.rim, w.crossing);

        if wider_is_its_rim
            && !turns_where_the_narrowers_arc_would(
                pool,
                narrower.plan,
                radius,
                pool.position_of(w.rim),
            )
            && turns_where_the_narrowers_arc_would(pool, narrower.plan, radius, w.crossing)
        {
            // Declined at this corner: the rim vertex is within the ruler of the crossing but the
            // patch's far corner read at it no longer turns where the narrower run's own arc would,
            // and a corner that built on the crossing may not be refused for the sake of an
            // identity.
            wider_is_its_rim = false;
        }

        let wider_corner = if wider_is_its_rim {
            pool.position_of(w.rim)
        } else {
            w.crossing
        };

        if !turns_where_the_narrowers_arc_would(pool, narrower.plan, radius, wider_corner) {
            // The wider foot's crossing does not stand a radius from the narrower run's centre
            // line, so it is not where that run's own arc, carried on to this sweep, would have met
            // the mitre - the patch's two ends turn at two different points and bound nothing.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                vec![wider_corner.into()],
            ));
        }

        let narrower_crossing_vertex = if narrower_is_its_rim {
            n.rim
        } else if n.corner >= 0 {
            n.corner
        } else {
            pool.add(n.crossing)
        };
        let wider_crossing_vertex = if wider_is_its_rim {
            w.rim
        } else if w.corner >= 0 {
            w.corner
        } else {
            pool.add(w.crossing)
        };

        Ok(Some(SurfaceMitreSliver {
            narrower_edge: narrower.edge,
            narrower_foot,
            narrower_rim: n.rim,
            narrower_rim_param: n.rim_param,
            narrower_crossing: narrower_crossing_vertex,
            narrower_crossing_at: n.crossing,
            narrower_named: the_named_corner(n.named, n.rim, narrower_crossing_vertex),
            wider_edge: wider.edge,
            wider_foot,
            wider_rim: w.rim,
            wider_rim_param: w.rim_param,
            wider_crossing: wider_crossing_vertex,
            wider_crossing_at: w.crossing,
            wider_named: the_named_corner(w.named, w.rim, wider_crossing_vertex),
            took_the_rims: narrower_is_its_rim || wider_is_its_rim,
            outward,
            over_the_narrowers_arc: Vec::new(),
        }))
    }

    /// Whether this patch has claimed one strip's foot rail, which stands at its rim.
    pub fn claims_the_foot(&self, edge_index: i32, rail: i32) -> bool {
        (edge_index == self.narrower_edge && rail == self.narrower_foot)
            || (edge_index == self.wider_edge && rail == self.wider_foot)
    }

    /// The claimed foot's ring vertex, which is the one the walkback left at the rim: the stitch
    /// sews a strip to EVERY vertex of its two seams, so a foot that minted its crossing instead
    /// would leave the rim's own vertex standing part way along the strip's own edge.
    pub fn take_the_foot(&self, edge_index: i32, ring: &mut [i32], at: &mut [f64]) {
        if edge_index == self.narrower_edge {
            ring[self.narrower_foot as usize] = self.narrower_rim;
            at[self.narrower_foot as usize] = self.narrower_rim_param;

            return;
        }

        ring[self.wider_foot as usize] = self.wider_rim;
        at[self.wider_foot as usize] = self.wider_rim_param;
    }

    /// The crossing a claimed foot steps out to, which its end ring carries BETWEEN two rails - the
    /// same staircase an end cut's own leavings travel by.
    pub fn the_crossing_of(&self, edge_index: i32) -> i32 {
        if edge_index == self.narrower_edge {
            self.narrower_crossing
        } else {
            self.wider_crossing
        }
    }

    /// Which of a strip's chords that step stands on - the one bounded by the foot rail.
    pub fn the_step_chord_of(&self, edge_index: i32) -> i32 {
        let foot = if edge_index == self.narrower_edge {
            self.narrower_foot
        } else {
            self.wider_foot
        };

        if foot == 0 {
            0
        } else {
            foot - 1
        }
    }

    /// Whether that step's own crossing leads the chord it stands on. It stands beside the FOOT, so
    /// it leads where the foot is the chord's lower rail and follows where it is the higher.
    pub fn the_step_leads(&self, edge_index: i32) -> bool {
        (if edge_index == self.narrower_edge {
            self.narrower_foot
        } else {
            self.wider_foot
        }) == 0
    }

    /// The wider run's rails over the angles the narrower run has none at, cut on the MITRE PLANE
    /// like every other rail rather than landed on the far face: that face is the partner's own
    /// wall and the band beside it ended at the crease, so there is nothing out there to land on.
    #[allow(clippy::too_many_arguments)]
    pub fn try_the_rails_over_the_narrowers_arc(
        &mut self,
        pool: &mut SurfaceVertexPool,
        wider: &SurfaceStrip,
        wider_on_the_face: i32,
        wider_step: i32,
        paired: i32,
        corner: Vector3,
        through: Vector3,
        normal: Vector3,
        ring: &mut [i32],
        at: &mut [f64],
    ) -> Result<(), BevelAdvisory> {
        self.over_the_narrowers_arc.clear();

        for step in paired..wider.rail_count() {
            let rail = wider_on_the_face + (wider_step * step);
            let r = rail as usize;

            if rail == self.wider_foot {
                ring[r] = self.wider_rim;
                at[r] = self.wider_rim_param;

                continue;
            }

            let Some(param) = wider.try_cross(rail, through, normal) else {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripsDoNotMitre,
                    vec![corner.into()],
                ));
            };
            at[r] = param;

            ring[r] = pool.add(wider.point_on(rail, at[r]));

            self.over_the_narrowers_arc.push(ring[r]);
        }

        // The foot's own crossing closes the polyline, whether or not the foot was the last rail
        // counted.
        self.over_the_narrowers_arc.push(self.wider_crossing);

        Ok(())
    }
}

/// A named band corner kept only where it is a THIRD place: a corner that is the foot's own rim, or
/// the very vertex the crossing was taken as, says nothing the patch does not already draw.
fn the_named_corner(named: i32, rim: i32, crossing: i32) -> i32 {
    if named == rim || named == crossing {
        -1
    } else {
        named
    }
}

/// Whether a foot's crossing and the rim vertex its own run ended at are ONE POINT, asked against
/// the pool's near-enough ruler rather than its weld. The overrun itself is read against the WELD -
/// that is what makes it an overrun at all - so this is the coarser question of whether the wall
/// already carries a vertex where the patch wants its corner.
fn the_crossing_is_its_rim(pool: &SurfaceVertexPool, rim: i32, crossing: Vector3) -> bool {
    (pool.position_of(rim) - crossing).length() <= pool.near_enough()
}

/// Whether a point stands a radius from the narrower run's centre line, which is the patch's two
/// ends turning at ONE point: where the narrower run's own arc, carried on to the wider run's
/// sweep, would have met the mitre.
fn turns_where_the_narrowers_arc_would(
    pool: &SurfaceVertexPool,
    narrower_plan: &SurfaceEdgePlan,
    radius: f64,
    at: Vector3,
) -> bool {
    narrower_plan.has_center_line
        && (narrower_plan.center_line.distance_to(at) - radius).abs() <= pool.near_enough()
}

/// Whether one strip's foot rail is cut PAST the rim its walkback ended at, and where both stand -
/// `None` at an ordinary foot, which lands on that rim itself, and at one cut short of it (a foot
/// stopped inside its own band is a different fault and keeps its own name). `band_corners` are the
/// walkback's own band corners per rail AND per end, or `None` where none were published; `end` is
/// which end of this edge's band the mitre stands at - 0 arriving, 1 leaving - the only end of it
/// whose corners this foot may take.
#[allow(clippy::too_many_arguments)]
fn the_foot_overruns(
    pool: &SurfaceVertexPool,
    strip: &SurfaceStrip,
    plan: &SurfaceEdgePlan,
    seams: &[SideSeams],
    band_corners: Option<&[SideBandCorners]>,
    edge_index: i32,
    end: usize,
    foot: i32,
    through: Vector3,
    normal: Vector3,
    away_from_the_corner: Vector3,
) -> Option<FootOverrun> {
    let crossing_param = strip.try_cross(foot, through, normal)?;

    let mut crossing = strip.point_on(foot, crossing_param);

    let side = if foot == 0 { 0 } else { 1 };
    let run = seams
        .get(edge_index as usize)
        .and_then(|seam| seam[side].as_deref())
        .filter(|run| !run.is_empty())?;

    // Which end of that seam's run this mitre is at is which way the strip was walked, so the
    // nearer of the two is taken - the same two-sided reading every other foot takes.
    let mut standing = run[run.len() - 1];

    if (pool.position_of(run[0].0) - crossing).length()
        < (pool.position_of(standing.0) - crossing).length()
    {
        standing = run[0];
    }

    let (rim, rim_param) = standing;

    // Read against the WELD distance and not against surface_strip_ends::meets_the_seam, which is
    // ten of them. What is asked here is whether the crossing and the rim are ONE POINT, which is
    // the weld's own question, and the seam ruler answers a coarser one - whether the build could
    // tell a strip's end from the seam it was sewn to. The two part company on a big part, because
    // both are read off the box round it: the picture hanger's raked mouth carries a foot that
    // overruns its rim by 7.4999e-4, which is 9.76 welds and a real step, and the seam ruler of
    // 7.6843e-4 calls it nothing - so the patch declines, a sector is asked of a face beyond that
    // holds no profile, and the corner refuses SurfaceStripEndNotSupported. The same bore in a
    // plate a third the size builds, which is the whole of the difference.
    if !((pool.position_of(rim) - crossing).length() > pool.weld_distance()
        && (crossing - pool.position_of(rim)).dot(away_from_the_corner) < 0.0)
    {
        return None;
    }

    let mut corner = -1;
    let mut named = -1;

    // THE BAND'S OWN CORNER, where the walkback left one at THIS FOOT'S OWN END. The crossing above
    // is read on the STRAIGHT rail, and where the band was drawn on a borrowed rail across a crease
    // the corner it really cut stands a hair off that line - far enough that the tangent run ends
    // short of it. Taking the corner by identity is what keeps the wall's rim and this patch's one
    // polyline instead of two.
    //
    // Four gates. Three of them are IDENTITY: THE END is the foot's own - a corner of the cut's
    // other end is a different place entirely - and then two planes, the MITRE plane, which says
    // the corner stands at this corner at all, and THIS CUT'S OWN TANGENT PLANE, which says it is a
    // corner of THIS foot's rail and not of a neighbouring rim that shares the mitre. The fourth,
    // the pool's nearness of the straight crossing, is NOT identity and is not here to name the
    // corner.
    //
    // The tangent plane names it, and it names it by orders of magnitude rather than by a hair: at
    // the hanger stand-off exactly one corner of each foot's own end stands within a hundredth of a
    // weld of its own cut's tangent plane and every other corner of that end stands 51 to 20000
    // welds off it. That is what the end alone could never do, the mitre plane being shared by BOTH
    // edges and running the length of the corner, so that on the end and the mitre alone a foot
    // takes whatever corner of its own end stands on that plane however far along it and eight
    // builds refuse SurfaceStripEndNotSupported. With the tangent plane in, the end and the mitre
    // leave at most one candidate per foot and the nearness names nothing.
    //
    // What the nearness is, now that it names nothing, is the BUDGET on how far this take may move
    // the crossing. The corner the stand-off's forked rim wants stands 0.00 welds from its own
    // cut's tangent plane - it IS a tangent-clip vertex of that cut, minted where the band was
    // drawn on a borrowed rail across a crease, 10.94 welds off the tangent LINE and on the tangent
    // PLANE exactly - and 10.95 welds from the straight crossing. Admitting it, and it alone, by
    // widening this ruler a fifth refuses the stand-off's own build as SurfaceStripSeamDidNotMeet:
    // a crossing moved eleven welds is past what the seam either strip was sewn with will still
    // meet. So the corner is correctly NAMED here and cannot be TAKEN here; carrying it is the
    // patch emitter's to do, not this crossing's. An edge index past the table reads as no corners
    // published: a guard, since the builder publishes a table entry for every edge it hands a
    // mitre.
    let of_this_end = band_corners
        .and_then(|corners| corners.get(edge_index as usize))
        .and_then(|corners| corners[side][end].as_deref());

    if let Some(of_this_end) = of_this_end {
        // The straight rail's own crossing, kept apart from the out value so that taking a corner
        // does not move the point the rest of the candidates are measured against.
        let on_the_rail = crossing;
        let mut nearest = f64::MAX;

        // The cut's own tangent plane: the plane the walkback clipped this facet on, through this
        // rail's tangent line with Inward for its normal - the very plane SurfaceFacetCut measures
        // its signed distance against, read off the plan the cut was minted from.
        let rail_plane_point = if foot == 0 {
            plan.left_tangent_line.point
        } else {
            plan.right_tangent_line.point
        };
        let rail_plane_normal = if foot == 0 {
            plan.left_inward
        } else {
            plan.right_inward
        }
        .get_normal();

        for &candidate in of_this_end {
            let at = pool.position_of(candidate);

            if (at - through).dot(normal.get_normal()).abs() > pool.weld_distance()
                || (at - rail_plane_point).dot(rail_plane_normal).abs() > pool.weld_distance()
            {
                continue;
            }

            // More than one corner of one end standing on BOTH planes is the band cut on two facets
            // that both reach the mitre along this one rail; the nearer to the straight crossing is
            // the one this foot's own rail ran at. Nothing in the kernel suite has two - every foot
            // that takes a corner at all has exactly one - so the tiebreak is a guard and not a
            // reading anything here depends on.
            let away = (at - on_the_rail).length();

            if away >= nearest {
                continue;
            }

            nearest = away;
            named = candidate;

            // The nearness is the BUDGET on how far this take may move the crossing, and it gates
            // the TAKE alone: the name above stands either way.
            if away <= pool.near_enough() {
                corner = candidate;
                crossing = at;
            }
        }
    }

    Some(FootOverrun {
        rim,
        rim_param,
        crossing,
        corner,
        named,
    })
}
