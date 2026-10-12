//! Port of `R/Tests/GeometryProcessingTests/BevelCanalOverTheRoundTests.cs`: the second pass's
//! bead over the rounded ridge of the nose-in-notch part (`nose_in_notch_cut`), held to the
//! rolling-ball CANAL a CAD package builds there - a radius-2 pipe whose centre keeps a distance of
//! 2 from both the right slope plane and the true cylinder of the round, so its spine is an
//! ellipse arc, not a polyline.
//!
//! The analytic canal, derived from the part and the radius: the nose's half wedge is
//! atan(2/3) = 33.69 degrees, so a round of radius r on the ridge is the cylinder on the axis
//! (., 5 + r / sin 33.69, 10). A ball of radius r rolling in the crease between that cylinder and
//! the right slope x + y = 20 has its centre in the plane x + y = 20 - r sqrt 2, 2r from the axis:
//! an ellipse, touching the round for spine angles within 90 - 33.69 degrees of the crest and
//! rolling on the flat nose faces (the straight legs) beyond. Each surface point of the bead
//! `bevel_second_pass::rebead` builds, over the arc band, has to lie within the chord tolerance of
//! that canal.

use std::fmt::Write as _;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_end_cap::BevelChainEnds;
use geometry_processing::bevel::bevel_chain_sweep_mesh::SweptSegment;
use geometry_processing::bevel::bevel_chain_sweep_rings::{profile_deviation, try_build_rings};
use geometry_processing::bevel::bevel_chain_sweep_sections::try_build_segments;
use geometry_processing::bevel::bevel_edge_tool_builder::profile_wedge_angle_degrees;
use geometry_processing::bevel::bevel_mitre_fit::SECTIONS_DIFFER_FRACTION;
use geometry_processing::bevel::bevel_round_entry;
use geometry_processing::bevel::bevel_second_pass::{self, face_tolerance_degrees};
use geometry_processing::bevel::{BevelRoundFaces, BevelSweptSection};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelEdgeGroupResolution, ChainEdge, FeatureEdgeChain, FeatureEdgeSet,
    MatchTolerances,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::math_helper::radians_to_degrees;
use vector_math::{Vector2, Vector3};

use crate::nose_in_notch_cut::{anchor, cut_ridge};
use crate::nose_in_notch_test_meshes as meshes;

const RADIUS: f64 = 2.0;
const SEGMENTS: i32 = 16;

/// The same chord allowance the control fixtures judge a rebuilt surface at.
const CHORD_TOLERANCE: f64 = 0.05;

/// The spine keeps the ball a radius off the round's surface, so two radii off its axis.
const SPINE_RADIUS: f64 = 2.0 * RADIUS;

/// How far past the arc the straight legs of the spine are followed - enough to cover the tip
/// region.
const LEG_LENGTH: f64 = 6.0;

/// The nose's half wedge, atan(2/3): the faces rise 10/3 in z over 5 in y from the ridge.
fn half_wedge() -> f64 {
    (meshes::upper_crease_end().z - meshes::RIDGE_Z)
        .atan2(meshes::upper_crease_end().y - meshes::RIDGE_Y)
}

/// The round's axis runs along x through this (y, z).
fn axis() -> Vector2 {
    Vector2::new(
        meshes::RIDGE_Y + (RADIUS / half_wedge().sin()),
        meshes::RIDGE_Z,
    )
}

/// The plane the spine lies in, x + y = this: a radius in from the slope x + y = 20.
fn spine_plane() -> f64 {
    meshes::SIZE - (RADIUS * 2.0_f64.sqrt())
}

/// How far either side of the crest the ball still touches the round rather than a nose face.
fn arc_half_angle() -> f64 {
    (std::f64::consts::PI / 2.0) - half_wedge()
}

#[test]
fn the_bead_over_the_round_lies_on_the_rolling_ball_canal() {
    let ridge_cut = cut_ridge(RADIUS, SEGMENTS);
    let cut = ridge_cut.cut;
    let (upper, lower) = (ridge_cut.upper, ridge_cut.lower);
    let mut report = String::new();

    // THE RIDGE GROUP IS CARRIED TOO, ahead of the two creases, because a rebuild carries it: the
    // second pass reads the first pass's own resolutions to find out where the round it is beading
    // over was centred (BevelRoundFaces), and a fixture that handed it only the concave groups
    // would be measuring a code path the application never takes.
    let groups = vec![
        anchor(
            &FeatureEdgeSet::create(&meshes::create()),
            meshes::left_ridge_end(),
            meshes::right_ridge_end(),
        ),
        upper.clone(),
        lower,
    ];

    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &meshes::create());
    let eligible = [true, true, true];
    let rounds = BevelRoundFaces::create(&cut, &resolutions, &eligible, RADIUS, SEGMENTS, true);
    let _ = writeln!(
        report,
        "Faces of the cut part read as being on a round: {}, as one of a round's tangent planes: {}",
        rounds.count(),
        rounds.tangent_plane_count()
    );
    let (_chain, swept, rings) = sweep_the_merged_crease(&cut, &upper, &rounds, &mut report);
    let bead = rebead(&cut, &groups, &resolutions, &eligible);
    let _ = writeln!(
        report,
        "Rebead tool: {} vertices, {} faces",
        bead.vertices.len(),
        bead.faces.len()
    );
    let stations = measure_stations(&swept, &rings, &bead, &mut report);
    report_the_cut_part(&cut, &mut report);
    println!("{report}");

    let in_the_band: Vec<&Station> = stations
        .iter()
        .filter(|station| station.in_the_band)
        .collect();
    assert!(
        in_the_band.len() > 4,
        "the fixture means nothing unless the crease runs over the round: {report}"
    );
    let worst = in_the_band
        .iter()
        .map(|station| station.worst)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        worst <= CHORD_TOLERANCE,
        "the bead over the round has to be the rolling-ball canal to the chord tolerance, and is \
         out by {worst:.4}. {report}"
    );

    // AND THE RUN HAS TO STAY ONE RUN. Each section over the round stands on its own station of
    // the spine, so consecutive ones are no longer identical, and past a quarter of the size
    // BevelMitreFit calls that a joint the mitre cannot fit and BevelChainLinker cuts the crease
    // there - which would hand the second pass a string of stubs to blend rather than one bead.
    let (deviation, joint) = worst_profile_deviation(&swept);
    assert!(
        deviation < RADIUS * SECTIONS_DIFFER_FRACTION,
        "consecutive sections over the round are {deviation:.4} apart at joint {joint} - between \
         the segments ending at {} and {} - which the mitre has to be able to average into one \
         ring. {report}",
        f(swept[joint.saturating_sub(1)].to),
        f(swept[joint].to)
    );
}

/// The merged crease read off the cut part as the second pass reads it, swept by the sweep's own
/// per-segment sections and mitred rings - the very surface Rebead's bead is sewn from.
fn sweep_the_merged_crease(
    cut: &Mesh,
    group: &BevelEdgeGroup,
    rounds: &BevelRoundFaces,
    report: &mut String,
) -> (Vec<ChainEdge>, Vec<SweptSegment>, Vec<Vec<Vector3>>) {
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        cut,
        &cut.get_mesh_edge_graph(),
        group.flat_angle_degrees,
    ));
    let crease = bevel_edge_group_resolver::resolve_crease(
        group,
        features,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        face_tolerance_degrees(SEGMENTS),
        MatchTolerances {
            ambiguity_ratio: 1.0,
            ..MatchTolerances::default()
        },
    );
    let crease_chain = match (&crease.chain, crease.is_resolved()) {
        (Some(chain), true) => chain,
        _ => panic!("the fixture's crease did not resolve on the cut part"),
    };

    // THE SAME CUT THE SWEEP MAKES FOR ITSELF one call before it builds its sections
    // (BevelChainSweep.TryCreate): the edge that crosses the flat face the round runs out onto is
    // split where the rolling ball enters the round's span, so no segment stands for both sides of
    // it. Taken here rather than inside TryBuildSegments so this fixture reports and measures the
    // very segments the bead below is sewn from, one per chain edge.
    let chain = bevel_round_entry::subdivide(crease_chain.edges(), Some(rounds), RADIUS, true)
        .unwrap_or_else(|| crease_chain.edges().to_vec());
    let mut swept = try_build_segments(
        &chain,
        RADIUS,
        SEGMENTS,
        true,
        BevelSweptSection::Tool,
        0.0,
        Some(rounds),
    )
    .unwrap_or_else(|failure| panic!("the sections did not build: {:?}", failure.code()));

    let point_count = swept[0].profile.len();
    let rings = try_build_rings(
        &mut swept,
        false,
        &BevelChainEnds::overshoot(),
        RADIUS,
        point_count,
        0.0,
        0.0,
        BevelSweptSection::Tool,
        None,
    )
    .unwrap_or_else(|failure| panic!("the rings did not build: {:?}", failure.code()));

    let _ = writeln!(
        report,
        "Merged crease: {} edges from {} to {}",
        chain.len(),
        f(chain[0].from()),
        f(chain[chain.len() - 1].to())
    );
    let _ = writeln!(
        report,
        "Per segment: interior angle, tangent distance, and the section's ball centre at the \
         segment's midpoint measured to the slope plane (want 2), the round's axis (want 4 over \
         the round) and the analytic spine (want 0):"
    );
    let axis = axis();
    for (i, segment) in swept.iter().enumerate() {
        let centre = section_ball_centre(segment);
        let mid = ((segment.from + segment.to) * 0.5) + centre;
        let (to_spine, phi, on_arc) = nearest_on_the_spine(mid);
        let to_axis = Vector2::new(mid.y - axis.x, mid.z - axis.y).length();
        let to_slope = (meshes::SIZE - mid.x - mid.y) / 2.0_f64.sqrt();
        let wedge = profile_wedge_angle_degrees(&chain[i].edge);
        // The ball the canal really has at this section: the spine point IN the section plane
        // (the plane square to the edge through its midpoint), and the round's tangent point
        // under it - which is the face the section should have been drawn against.
        let mid_edge = (segment.from + segment.to) * 0.5;
        let (true_phi, true_centre) = spine_in_the_plane(mid_edge, segment.direction);
        let crease_phi = (mid_edge.z - axis.y).atan2(-(mid_edge.y - axis.x));
        let true_tangency = true_centre
            + ((Vector3::new(true_centre.x, axis.x, axis.y) - true_centre).get_normal() * RADIUS);
        // Where the section's own tangent point on the round's side of the profile lands, and how
        // far that is off the round's true surface: the facet plane runs on past its own facet.
        let round_side = section_tangent_point_on_the_round(segment, mid_edge);
        let round_side_to_axis =
            Vector2::new(round_side.y - axis.x, round_side.z - axis.y).length();
        // And how far the true ball slides along the spine over this ONE segment - the plane
        // square to the chord at its start against the same plane at its end - which is what a
        // constant section per segment has to stand in for.
        let (at_start, start_centre) = spine_in_the_plane(segment.from, segment.direction);
        let (at_end, end_centre) = spine_in_the_plane(segment.to, segment.direction);
        let _ = writeln!(
            report,
            "  seg {i:2} {} -> {} len {:.3} wedge {wedge:.2} tan {:.3} centre->slope {to_slope:.4} \
             ->axis {to_axis:.4} ->spine {to_spine:.4} at phi {:.1}{} | crease phi {:.1}, spine \
             in this plane at phi {:.1}, centre is {:.4} from it; true ball slides {:.3} along \
             the spine (phi {:.1} to {:.1}) over this segment; {}",
            f(segment.from),
            f(segment.to),
            segment.length,
            segment.tangent_distance,
            radians_to_degrees(phi),
            if on_arc { " arc" } else { " leg" },
            radians_to_degrees(crease_phi),
            radians_to_degrees(true_phi),
            (mid - true_centre).length(),
            (end_centre - start_centre).length(),
            radians_to_degrees(at_start),
            radians_to_degrees(at_end),
            if on_arc {
                format!(
                    "tangent point on the round side {} is {:+.4} off the round, the true \
                     tangency is {}",
                    f(round_side),
                    round_side_to_axis - RADIUS,
                    f(true_tangency)
                )
            } else {
                "(a leg: its round-side tangent point is on the flat nose face)".to_string()
            }
        );
    }

    (chain, swept, rings)
}

/// The bead the second pass builds along the merged crease - ONE run, ONE bead.
fn rebead(
    cut: &Arc<Mesh>,
    groups: &[BevelEdgeGroup],
    resolutions: &[BevelEdgeGroupResolution],
    eligible: &[bool],
) -> Mesh {
    let answer = bevel_second_pass::rebead(
        Some(cut),
        groups,
        resolutions,
        eligible,
        true,
        RADIUS,
        SEGMENTS,
        &CancelToken::new(),
    )
    .expect("not cancelled");

    if answer.runs.len() != 1 || answer.runs[0].tools.len() != 1 {
        let reasons: Vec<String> = answer
            .kept_first_pass
            .iter()
            .map(|(_, reason)| format!("{:?}", reason.code()))
            .collect();
        panic!(
            "expected one run with one bead, got {} runs ({})",
            answer.runs.len(),
            reasons.join(", ")
        );
    }

    answer.runs[0].tools[0].mesh.clone()
}

#[derive(Default)]
struct Station {
    vertex: Vector3,
    in_the_band: bool,
    worst: f64,
    signed: f64,
    at: Vector3,
    phi: f64,
}

/// Every surface point of every interior ring measured to the analytic canal, grouped by station.
/// Each point is also checked to be a vertex of the bead Rebead handed back, so what is measured
/// IS that bead's surface and not a sweep of this test's own.
#[allow(clippy::needless_range_loop)] // C#'s index loops, read against the rings and the profile.
fn measure_stations(
    swept: &[SweptSegment],
    rings: &[Vec<Vector3>],
    bead: &Mesh,
    report: &mut String,
) -> Vec<Station> {
    let bead_vertices: Vec<Vector3> = bead.vertices.iter().map(|v| v.as_vector3()).collect();
    let mut stations = Vec::new();
    let mut missing = 0;
    let _ = writeln!(
        report,
        "Per station (ring at each chain vertex): worst |distance to spine - r| over the ring's \
         surface points, signed (+ proud of the canal, - inside it):"
    );
    let axis = axis();
    for ring in 1..rings.len() - 1 {
        let segment = &swept[ring - 1];
        let mut station = Station {
            vertex: segment.to,
            ..Station::default()
        };
        let point_count = segment.profile.len();
        for point in 0..point_count {
            if !is_surface_point(point, point_count, segment.profile_reversed) {
                continue;
            }

            let position = rings[ring][point];
            if !bead_vertices
                .iter()
                .any(|&v| (v - position).length() < 1e-4)
            {
                missing += 1;
            }

            let (distance, phi, on_arc) = nearest_on_the_spine(position);
            let signed = distance - RADIUS;
            if signed.abs() > station.worst {
                station.worst = signed.abs();
                station.signed = signed;
                station.at = position;
                station.phi = phi;
                station.in_the_band = on_arc;
            }
        }

        // The station is in the band when its own crease vertex sits on the round, and the ring's
        // worst point is judged against the arc.
        let to_axis = Vector2::new(station.vertex.y - axis.x, station.vertex.z - axis.y).length();
        let vertex_on_the_round = (to_axis - RADIUS).abs() < 0.02;
        station.in_the_band = station.in_the_band && vertex_on_the_round;
        let _ = writeln!(
            report,
            "  station {ring:2} at {}{}: worst {:+.4} at {} phi {:.1}{}",
            f(station.vertex),
            if vertex_on_the_round {
                " (on the round)"
            } else {
                ""
            },
            station.signed,
            f(station.at),
            radians_to_degrees(station.phi),
            if station.in_the_band { " [band]" } else { "" }
        );
        stations.push(station);
    }

    let _ = writeln!(
        report,
        "Surface ring points not found among the bead's vertices: {missing}"
    );

    // And what the mitre is being asked to average, against the fraction of the size
    // BevelMitreFit would cut the run at. The sections over a round are no longer identical -
    // each stands on its own station of the spine - so this is the reading that says the run
    // still holds together as one.
    let (worst_deviation, worst_at) = worst_profile_deviation(swept);
    let _ = writeln!(
        report,
        "Worst profile deviation between consecutive sections: {worst_deviation:.4} at joint \
         {worst_at}, which is {:.3} of the size against the {SECTIONS_DIFFER_FRACTION:.2} \
         BevelMitreFit cuts a run at",
        worst_deviation / RADIUS
    );

    // And the same reading over the bead's own vertices in the control fixture's tip region, for
    // the numbers the app-side test reports.
    let mut tip_worst = 0.0;
    let mut tip_at = Vector3::ZERO;
    let mut surface = std::collections::BTreeSet::new();
    for ring in 1..rings.len() - 1 {
        let segment = &swept[ring - 1];
        for point in 0..segment.profile.len() {
            if is_surface_point(point, segment.profile.len(), segment.profile_reversed) {
                let position = rings[ring][point];
                for (v, vertex) in bead_vertices.iter().enumerate() {
                    if (*vertex - position).length() < 1e-4 {
                        surface.insert(v);
                    }
                }
            }
        }
    }

    for v in surface {
        let position = bead_vertices[v];
        if position.x > 10.5 && position.y < 8.0 && (position.z - 10.0).abs() < 4.0 {
            let (distance, _, on_arc) = nearest_on_the_spine(position);
            if on_arc && (distance - RADIUS).abs() > tip_worst {
                tip_worst = (distance - RADIUS).abs();
                tip_at = position;
            }
        }
    }

    let _ = writeln!(
        report,
        "Worst bead surface vertex in the tip region against the canal (arc band): {tip_worst:.4} \
         at {}",
        f(tip_at)
    );
    stations
}

/// The largest distance any point of one section is from the same point of the next, over every
/// joint of the run, with the joint it happens at. It is exactly what
/// `BevelMitreFit.JointsTheMitreCannotFit` measures: past a fraction of the size the two sections
/// are no longer one section to mitre and the linker cuts the run there instead.
fn worst_profile_deviation(swept: &[SweptSegment]) -> (f64, usize) {
    let mut worst = 0.0;
    let mut at = 0;

    for joint in 1..swept.len() {
        let deviation = profile_deviation(&swept[joint - 1], &swept[joint]);

        if deviation > worst {
            worst = deviation;
            at = joint;
        }
    }

    (worst, at)
}

/// Which profile points are the SURFACE - the two tangent points and the arc between them. The
/// section is [cutStart, back, apex, back, cutEnd, arc...] (BevelEdgeToolBuilder
/// .TryBuildCrossSection), reversed wholesale where the sweep turned the winding.
fn is_surface_point(point: usize, point_count: usize, reversed: bool) -> bool {
    let original = if reversed {
        point_count - 1 - point
    } else {
        point
    };
    !(1..=3).contains(&original)
}

/// The section's arc centre - the ball's centre for this segment - as an offset from the edge.
fn section_ball_centre(segment: &SweptSegment) -> Vector3 {
    let count = segment.profile.len();
    // three points of the arc, whichever way the profile is wound
    let arc: Vec<Vector2> = (0..count)
        .filter(|&point| {
            (if segment.profile_reversed {
                count - 1 - point
            } else {
                point
            }) >= 5
        })
        .map(|point| segment.profile[point])
        .collect();
    let a = arc[0];
    let b = arc[arc.len() / 2];
    let c = arc[arc.len() - 1];
    let centre = circumcentre(a, b, c);
    (segment.x_axis * centre.x) + (segment.y_axis * centre.y)
}

fn circumcentre(a: Vector2, b: Vector2, c: Vector2) -> Vector2 {
    let d = 2.0 * ((a.x * (b.y - c.y)) + (b.x * (c.y - a.y)) + (c.x * (a.y - b.y)));
    let ux = ((a.length_squared() * (b.y - c.y))
        + (b.length_squared() * (c.y - a.y))
        + (c.length_squared() * (a.y - b.y)))
        / d;
    let uy = ((a.length_squared() * (c.x - b.x))
        + (b.length_squared() * (a.x - c.x))
        + (c.length_squared() * (b.x - a.x)))
        / d;
    Vector2::new(ux, uy)
}

/// A point of the spine's arc, by its angle from the crest about the round's axis.
fn spine(phi: f64) -> Vector3 {
    let axis = axis();
    let y = axis.x - (SPINE_RADIUS * phi.cos());
    let z = axis.y + (SPINE_RADIUS * phi.sin());
    Vector3::new(spine_plane() - y, y, z)
}

/// The distance from a point to the whole spine - the arc and the two straight legs - with the
/// arc angle of the nearest point and whether that nearest point is on the arc.
fn nearest_on_the_spine(point: Vector3) -> (f64, f64, bool) {
    const SAMPLES: usize = 512;
    let arc_half_angle = arc_half_angle();
    let mut best_index = 0;
    let mut best = f64::INFINITY;
    for i in 0..=SAMPLES {
        let phi = -arc_half_angle + (2.0 * arc_half_angle * i as f64 / SAMPLES as f64);
        let d = (spine(phi) - point).length();
        if d < best {
            best = d;
            best_index = i;
        }
    }

    let mut low = -arc_half_angle
        + (2.0 * arc_half_angle * best_index.saturating_sub(1) as f64 / SAMPLES as f64);
    let mut high = -arc_half_angle
        + (2.0 * arc_half_angle * SAMPLES.min(best_index + 1) as f64 / SAMPLES as f64);
    for _ in 0..60 {
        let m1 = low + ((high - low) / 3.0);
        let m2 = high - ((high - low) / 3.0);
        if (spine(m1) - point).length() < (spine(m2) - point).length() {
            high = m2;
        } else {
            low = m1;
        }
    }

    let arc_phi = (low + high) / 2.0;
    let arc_distance = (spine(arc_phi) - point).length();

    // The legs: from each end of the arc, along the straight crease's own direction.
    let upper_leg = (meshes::upper_crease_end() - meshes::right_ridge_end()).get_normal();
    let lower_leg = (meshes::lower_crease_end() - meshes::right_ridge_end()).get_normal();
    let to_upper = to_segment(
        point,
        spine(arc_half_angle),
        spine(arc_half_angle) + (upper_leg * LEG_LENGTH),
    );
    let to_lower = to_segment(
        point,
        spine(-arc_half_angle),
        spine(-arc_half_angle) + (lower_leg * LEG_LENGTH),
    );
    let leg_distance = to_upper.min(to_lower);
    if leg_distance < arc_distance - 1e-9 {
        (
            leg_distance,
            if to_upper < to_lower {
                arc_half_angle
            } else {
                -arc_half_angle
            },
            false,
        )
    } else {
        (arc_distance, arc_phi, true)
    }
}

/// Where the spine crosses the plane through `point` square to `direction`: the ball the canal
/// really carries at that section. On the arc where it can be, else on a leg.
fn spine_in_the_plane(point: Vector3, direction: Vector3) -> (f64, Vector3) {
    // Every crossing of the plane - the arc's, and each leg's - and then the one nearest the
    // point, because a plane square to one leg also crosses the far end of the arc, which is not
    // its ball.
    let mut crossings: Vec<(f64, Vector3)> = Vec::new();
    const SAMPLES: usize = 512;
    let arc_half_angle = arc_half_angle();
    let side = |phi: f64| (spine(phi) - point).dot(direction);
    let sign = |value: f64| {
        // C# Math.Sign: -1, 0 or 1
        if value > 0.0 {
            1
        } else if value < 0.0 {
            -1
        } else {
            0
        }
    };
    let mut previous = side(-arc_half_angle);
    for i in 1..=SAMPLES {
        let phi = -arc_half_angle + (2.0 * arc_half_angle * i as f64 / SAMPLES as f64);
        let here = side(phi);
        if sign(here) != sign(previous) {
            let mut low = phi - (2.0 * arc_half_angle / SAMPLES as f64);
            let mut high = phi;
            for _ in 0..60 {
                let mid = (low + high) / 2.0;
                if sign(side(mid)) == sign(side(low)) {
                    low = mid;
                } else {
                    high = mid;
                }
            }

            let root = (low + high) / 2.0;
            crossings.push((root, spine(root)));
        }

        previous = here;
    }

    for (end, toward, phi) in [
        (
            spine(arc_half_angle),
            (meshes::upper_crease_end() - meshes::right_ridge_end()).get_normal(),
            arc_half_angle,
        ),
        (
            spine(-arc_half_angle),
            (meshes::lower_crease_end() - meshes::right_ridge_end()).get_normal(),
            -arc_half_angle,
        ),
    ] {
        let along = toward.dot(direction);
        if along.abs() < 1e-9 {
            continue;
        }

        let t = -(end - point).dot(direction) / along;
        if (0.0..=LEG_LENGTH).contains(&t) {
            crossings.push((phi, end + (toward * t)));
        }
    }

    // OrderBy is stable: the first of equal distances wins.
    crossings
        .into_iter()
        .fold(None, |best: Option<(f64, Vector3)>, crossing| match best {
            Some(kept) if (kept.1 - point).length() <= (crossing.1 - point).length() => Some(kept),
            _ => Some(crossing),
        })
        .unwrap_or((f64::NAN, Vector3::ZERO))
}

/// The section's tangent point on the round's side of the profile - the one off the slope plane.
fn section_tangent_point_on_the_round(segment: &SweptSegment, at: Vector3) -> Vector3 {
    let count = segment.profile.len();
    let candidates: Vec<Vector3> = [0usize, 4]
        .iter()
        .map(|&original| {
            if segment.profile_reversed {
                count - 1 - original
            } else {
                original
            }
        })
        .map(|point| at + segment.offset(point))
        .collect();
    // OrderBy(...).First(): the first of equal keys wins.
    if candidates[1].x + candidates[1].y < candidates[0].x + candidates[0].y {
        candidates[1]
    } else {
        candidates[0]
    }
}

fn f(v: Vector3) -> String {
    format!("({:.3}, {:.3}, {:.3})", v.x, v.y, v.z)
}

fn f2(v: Vector2) -> String {
    format!("({:.4}, {:.4})", v.x, v.y)
}

fn to_segment(point: Vector3, from: Vector3, to: Vector3) -> f64 {
    let along = to - from;
    let t = ((point - from).dot(along) / along.length_squared()).clamp(0.0, 1.0);
    (point - (from + (along * t))).length()
}

/// Where the cut part's round actually ends on its crest and tangent lines - the flat end cap the
/// fixture remarks describe.
fn report_the_cut_part(cut: &Mesh, report: &mut String) {
    let axis = axis();
    let half_wedge = half_wedge();
    let on_the_round: Vec<Vector3> = cut
        .vertices
        .iter()
        .map(|v| v.as_vector3())
        .filter(|v| (Vector2::new(v.y - axis.x, v.z - axis.y).length() - RADIUS).abs() < 1e-3)
        .collect();
    let crest = on_the_round
        .iter()
        .filter(|v| (v.z - meshes::RIDGE_Z).abs() < 1e-3)
        .fold(None, |best: Option<Vector3>, &v| match best {
            Some(kept) if kept.x >= v.x => Some(kept),
            _ => Some(v),
        })
        .unwrap_or(Vector3::ZERO);
    let tangent_y = meshes::RIDGE_Y + (RADIUS / half_wedge.tan()) * half_wedge.cos();
    let mut tangent_lines: Vec<Vector3> = on_the_round
        .iter()
        .copied()
        .filter(|v| (v.y - tangent_y).abs() < 1e-2)
        .collect();
    // OrderByDescending is stable.
    tangent_lines.sort_by(|a, b| b.x.total_cmp(&a.x));
    tangent_lines.truncate(2);
    let _ = writeln!(
        report,
        "Cut part: {} vertices on the round, crest ends at {}, tangent lines end at {}",
        on_the_round.len(),
        f(crest),
        tangent_lines
            .iter()
            .map(|&v| f(v))
            .collect::<Vec<_>>()
            .join(" / ")
    );
    let _ = writeln!(
        report,
        "Analytic: crest dies on the canal at x = {:.3}, tangent lines at x = {:.3}; axis (y, z) \
         = {}; spine plane x + y = {:.4}",
        spine(0.0).x,
        spine(arc_half_angle()).x,
        f2(axis),
        spine_plane()
    );
}
