//! The second half of `around_center_kink_measure_tests` (its `flow` module, in
//! `around_center_kink_measure_tests.rs`, whose gate, build and log plumbing it shares): a
//! MEASUREMENT of the flow design ("a blend that flows onto the neighbouring face") at the tip wall
//! of "5mm Bevel Around Center".
//!
//! Asserts nothing; gated and logged as the kinked-chain reading (log file name plus `.flow.txt`).

use std::fmt::Write as _;
use std::rc::Rc;

use solidmesh::bevel::level_line::{gap_fill, triangle_edges, ChainField};
use solidmesh_math::Vector3;

use super::{build, p, publish, to_control, RUN_VARIABLE};
use crate::encroaching_side_room_tests::up_to;
use crate::level_line_fixture::LevelLineFixture;

const MITRE_DEGREES: f64 = 10.0;

const CREASE_SAMPLES: usize = 4000;

/// The far face is x = -20; y runs away from the crease (y = -118.5) to -98.5.
fn far(d: f64, z: f64) -> Vector3 {
    Vector3::new(-20.0, -118.5 + d, z)
}

fn line_distance(q: Vector3, o: Vector3, dir: Vector3) -> f64 {
    let v = q - o;
    (v - (dir * v.dot(dir))).length()
}

/// The tangent at a rail end: its triangle's plane cut by the row's plane, pointing back to the
/// chain.
fn tangent(
    end: Vector3,
    face_normal: Vector3,
    row_normal: Vector3,
    toward_chain: Vector3,
) -> Vector3 {
    let t = face_normal.cross(row_normal);
    if t.length() < 1e-9 {
        return Vector3::ZERO;
    }

    let t = t.get_normal();
    if t.dot(toward_chain - end) < 0.0 {
        -t
    } else {
        t
    }
}

/// Where the lines p + s tp and q + t tq come nearest (midpoint); `None` if parallel or behind
/// either end.
fn meet(p: Vector3, tp: Vector3, q: Vector3, tq: Vector3) -> Option<Vector3> {
    let w0 = p - q;
    let b = tp.dot(tq);
    let den = 1.0 - (b * b);
    if tp.length() < 0.5 || tq.length() < 0.5 || den < 1e-9 {
        return None;
    }

    let dd = tp.dot(w0);
    let e = tq.dot(w0);
    let s = ((b * e) - dd) / den;
    let t = (e - (b * dd)) / den;
    if s <= 0.0 || t <= 0.0 {
        return None;
    }

    Some(((p + (tp * s)) + (q + (tq * t))) / 2.0)
}

/// The other rail's vertex at the tip, with its triangle's normal.
struct OtherRail {
    at: Vector3,
    normal: Vector3,
}

/// The wedge's row reader: the control, the fillet's segments and the other rail's end.
struct Rows<'a> {
    control: &'a LevelLineFixture,
    segments: i32,
    rail: OtherRail,
}

impl Rows<'_> {
    fn to_control(&self, q: Vector3) -> f64 {
        to_control(self.control, q).0
    }

    /// The worst arc vertex to the control, and the arc's midpoint's.
    fn row(&self, l: Vector3, c: Vector3) -> (f64, f64) {
        let r = self.rail.at;
        let mut worst = 0.0f64;
        for s in 1..self.segments {
            worst = worst.max(self.to_control(gap_fill::arc_point(
                l,
                c,
                r,
                s as f64 / self.segments as f64,
            )));
        }

        (worst, self.to_control(gap_fill::arc_point(l, c, r, 0.5)))
    }

    /// The design's control point: the row's plane is through both rail ends and a third point -
    /// (b) the foot the wall end reads (its ray foot, or the crease's for a flowed end), (c) the
    /// chain point today's row uses - and the tangents meet in it; fall back to the foot.
    fn tangent_meet(
        &self,
        l: Vector3,
        l_normal: Vector3,
        foot: Vector3,
        plane_point: Vector3,
    ) -> (Vector3, &'static str) {
        let r = self.rail.at;
        let row_normal = (l - plane_point).cross(r - plane_point);
        if row_normal.length() < 1e-9 {
            return (foot, "foot (row degenerate)");
        }

        let row_normal = row_normal.get_normal();
        let Some(m) = meet(
            l,
            tangent(l, l_normal, row_normal, foot),
            r,
            tangent(r, self.rail.normal, row_normal, foot),
        ) else {
            return (foot, "foot (tangents do not meet)");
        };

        // Outside the gap: farther from the foot than the chord is long.
        if (m - foot).length() > (l - r).length() {
            return (foot, "foot (meet outside gap)");
        }

        (m, "tangent meet")
    }
}

/// The mitred field (sharp turn 10 degrees) along the
/// knife crease v0-v2, the value carried one hop onto the `x = -20` face and the width of the strip
/// it reads below zero, the mitred rail's crossing of the crease, and the mitre wedge's rows laid
/// with today's control point and with the tangent-meet one. The rows are laid by hand from the
/// fixture's frame (tip v1, wall `x - y = 98.5`) - the build does not yet take the mitre, so there
/// is no band to read them from.
#[test]
fn the_flow_design_reads_against_the_control() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load("around-center-r5"));
    let control = LevelLineFixture::load("around-center-control");
    let positions = &fixture.positions;
    let mut log = String::new();

    let gap = build(&fixture);
    let _ = writeln!(
        log,
        "band {:?}, segments {}, size {}, mitre {MITRE_DEGREES} deg",
        gap.refusal, fixture.segments, fixture.size
    );
    if !gap.built() {
        println!("{log}");
        return;
    }

    // The build's own field re-made with the sharp-turn angle: same points, same setback at each
    // point.
    let plain = gap.chain().expect("a built band has its chain");
    let pts = plain.points();
    let mut setbacks = Vec::with_capacity(pts.len());
    let mut along = 0.0;
    for i in 0..pts.len() {
        along += if i == 0 {
            0.0
        } else {
            (pts[i] - pts[i - 1]).length()
        };
        setbacks.push(plain.setback_at(along));
    }

    let mitred = ChainField::from_polyline_sharp(pts, plain.is_closed, &setbacks, MITRE_DEGREES)
        .expect("the build's own points make a field");
    let value = |q: Vector3| mitred.evaluate(q).expect("the field reads").value;
    let tip = positions[1];
    let tip_parameter = plain.evaluate(tip).expect("the field reads").parameter;
    let _ = writeln!(
        log,
        "chain points {}, tip v1 {} parameter {}, setback there {}",
        pts.len(),
        p(tip),
        up_to(tip_parameter, 4),
        up_to(plain.setback_at(tip_parameter), 3)
    );

    // 1. The crease v0-v2 (x = -20, y = -118.5, z 0..20) and the far face x = -20.
    let c0 = positions[0];
    let c2 = positions[2];
    let mut crease = Vec::with_capacity(CREASE_SAMPLES + 1);
    let mut crease_value = Vec::with_capacity(CREASE_SAMPLES + 1);
    let mut min_at = 0;
    let mut plain_min = f64::INFINITY;
    for k in 0..=CREASE_SAMPLES {
        crease.push(c0 + ((c2 - c0) * (k as f64 / CREASE_SAMPLES as f64)));
        crease_value.push(value(crease[k]));
        plain_min = plain_min.min(plain.evaluate(crease[k]).expect("the field reads").value);
        if crease_value[k] < crease_value[min_at] {
            min_at = k;
        }
    }

    let _ = writeln!(
        log,
        "1. mitred value along crease v0-v2: min {} at {}; plain field's min there {}",
        up_to(crease_value[min_at], 4),
        p(crease[min_at]),
        up_to(plain_min, 4)
    );
    let carried = |q: Vector3| {
        let mut best = f64::INFINITY;
        for k in 0..=CREASE_SAMPLES {
            best = best.min(crease_value[k] + (q - crease[k]).length());
        }

        best
    };

    let _ = writeln!(
        log,
        "   carried on far-face vertices: v3 {} {}, v7 {} {}",
        p(positions[3]),
        up_to(carried(positions[3]), 3),
        p(positions[7]),
        up_to(carried(positions[7]), 3)
    );
    let strip_width = |z: f64| {
        if carried(far(0.0, z)) >= 0.0 {
            return 0.0;
        }

        let (mut lo, mut hi) = (0.0, 20.0);
        for _ in 0..50 {
            let mid = (lo + hi) / 2.0;
            if carried(far(mid, z)) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        lo
    };

    let mut widest = (0.0, 0.0);
    let mut z = 0.0;
    while z <= 20.0001 {
        let w = strip_width(z);
        if w > widest.1 {
            widest = (z, w);
        }

        z += 0.01;
    }

    let mut grid_negative = 0;
    let mut grid_count = 0;
    let mut d = 0.0;
    while d <= 20.0001 {
        let mut z = 0.0;
        while z <= 20.0001 {
            grid_count += 1;
            if carried(far(d, z)) < 0.0 {
                grid_negative += 1;
            }

            z += 0.1;
        }

        d += 0.1;
    }

    let _ = writeln!(
        log,
        "   strip on the far face where carried < 0: widest {} at z {}; grid 0.1: {grid_negative} of {grid_count} points negative",
        up_to(widest.1, 4),
        up_to(widest.0, 2)
    );
    let mut z = 9.5;
    while z <= 10.5001 {
        let _ = writeln!(log, "     z {z:.1}: width {}", up_to(strip_width(z), 4));
        z += 0.1;
    }

    // 2. The mitred rail's crossings of the crease, against the control's own vertices on the
    // crease.
    let mut crossings = Vec::new();
    for k in 0..CREASE_SAMPLES {
        if (crease_value[k] < 0.0) != (crease_value[k + 1] < 0.0) {
            let t = crease_value[k] / (crease_value[k] - crease_value[k + 1]);
            crossings.push(crease[k].z + ((crease[k + 1].z - crease[k].z) * t));
        }
    }

    let listed = |zs: &[f64]| {
        zs.iter()
            .map(|&z| up_to(z, 4))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let _ = writeln!(
        log,
        "2. mitred rail crosses the crease at z [{}]",
        listed(&crossings)
    );
    let mut control_on_crease: Vec<f64> = control
        .positions
        .iter()
        .filter(|q| (q.x + 20.0).abs() < 1e-3 && (q.y + 118.5).abs() < 1e-3)
        .map(|q| q.z)
        .collect();
    control_on_crease.sort_by(f64::total_cmp);
    let _ = writeln!(
        log,
        "   control vertices on the crease at z [{}]",
        listed(&control_on_crease)
    );

    // 3. The mitre wedge's rows. Wall rail: on the wall (plane x - y = 98.5) walk out from the tip
    // in direction cos(phi) u + sin(phi) w to the mitred field's zero, clipped at the crease (zero
    // width).
    let u = Vector3::new(-1.0, -1.0, 0.0).get_normal();
    let up = Vector3::new(0.0, 0.0, 1.0);
    let wall_normal = Vector3::new(1.0, -1.0, 0.0).get_normal();
    let far_normal = Vector3::new(1.0, 0.0, 0.0);
    let crease_reach = (Vector3::new(-20.0, -118.5, tip.z) - tip).length();

    // The other rail's vertex at the tip's parameter: on the inside faces where both edges' offsets
    // meet, the point with least |value| + |distance to line in - distance to line out|, on a fine
    // grid.
    let d_in = (tip - positions[8]).get_normal();
    let d_out = (positions[5] - tip).get_normal();
    let inside = [(1, 5, 9), (1, 9, 11), (1, 11, 8)];
    let mut rail = OtherRail {
        at: tip,
        normal: Vector3::ZERO,
    };
    let mut r_score = f64::INFINITY;
    for (a, b, c) in inside {
        const G: i32 = 300;
        let (pa, pb, pc) = (positions[a], positions[b], positions[c]);
        for i in 0..=G {
            for j in 0..=(G - i) {
                let q =
                    pa + ((pb - pa) * (i as f64 / G as f64)) + ((pc - pa) * (j as f64 / G as f64));
                let score = value(q).abs()
                    + (line_distance(q, tip, d_in) - line_distance(q, tip, d_out)).abs();
                if score < r_score {
                    r_score = score;
                    rail.at = q;
                    rail.normal = triangle_edges::normal(positions, (a as i32, b as i32, c as i32));
                }
            }
        }
    }

    let r_sample = mitred.evaluate(rail.at).expect("the field reads");
    let rows = Rows {
        control: &control,
        segments: fixture.segments,
        rail,
    };
    let _ = writeln!(
        log,
        "3. other rail at the tip parameter: {} value {} parameter {}, to control {}",
        p(rows.rail.at),
        up_to(r_sample.value, 4),
        up_to(r_sample.parameter, 4),
        up_to(rows.to_control(rows.rail.at), 3)
    );

    let mut worst_a = 0.0f64;
    let mut worst_b = 0.0f64;
    let mut worst_b_unclipped = 0.0f64;
    let mut worst_c = 0.0f64;
    let mut row_count = 0;
    let _ = writeln!(
        log,
        "   wedge rows (phi deg, wall end, ray foot, clipped?, (a) PointAt worst/mid, (b) worst/mid how):"
    );
    let mut phi_deg = -89.0;
    while phi_deg <= 89.0 {
        let phi = phi_deg * std::f64::consts::PI / 180.0;
        let dir = (u * phi.cos()) + (up * phi.sin());
        let (mut lo, mut hi) = (0.0, 20.0);
        for _ in 0..60 {
            let mid = (lo + hi) / 2.0;
            if value(tip + (dir * mid)) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let mut l = tip + (dir * lo);
        let sample = mitred.evaluate(l).expect("the field reads");
        let ray_read = (sample.nearest - tip).length() > 1e-6
            && (sample.parameter - tip_parameter).abs() < 1e-9;
        if !ray_read {
            phi_deg += 1.0;
            continue;
        }

        let mut clipped = false;
        if (l - tip).dot(u) > crease_reach {
            // Past the knife: the zero-width band stops on the crease along this ray.
            l = tip + (dir * (crease_reach / phi.cos()));
            clipped = true;
        }

        let foot = sample.nearest;
        let (a_worst, a_mid) = rows.row(l, mitred.point_at(tip_parameter));
        let (m, how) = rows.tangent_meet(l, wall_normal, foot, foot);
        let (b_worst, b_mid) = rows.row(l, m);
        let (m_c, how_c) = rows.tangent_meet(l, wall_normal, foot, tip);
        let (c_worst, _) = rows.row(l, m_c);
        worst_a = worst_a.max(a_worst);
        worst_b = worst_b.max(b_worst);
        worst_c = worst_c.max(c_worst);
        if !clipped {
            worst_b_unclipped = worst_b_unclipped.max(b_worst);
        }

        row_count += 1;
        let _ = writeln!(
            log,
            "     {} L {} foot {}{}: (a) {}/{}, (b) {}/{} {how}, (c) {} {how_c}",
            up_to(phi_deg, 0),
            p(l),
            p(foot),
            if clipped { " CLIPPED" } else { "" },
            up_to(a_worst, 3),
            up_to(a_mid, 3),
            up_to(b_worst, 3),
            up_to(b_mid, 3),
            up_to(c_worst, 3)
        );
        phi_deg += 1.0;
    }

    let _ = writeln!(
        log,
        "   {row_count} wedge rows: worst arc vertex to control (a) PointAt {}, (b) tangent meet, plane through ray foot {} (unclipped rows only {}), (c) tangent meet, plane through the tip {}",
        up_to(worst_a, 4),
        up_to(worst_b, 4),
        up_to(worst_b_unclipped, 4),
        up_to(worst_c, 4)
    );

    // 4. The flowed sliver: where the carried field reads below zero on the far face, the row's
    // wall end is instead on the far face at the carried zero, with the far face's tangent.
    let mut worst_flow = 0.0f64;
    let mut worst_flow_a = 0.0f64;
    let mut worst_flow_c = 0.0f64;
    let mut flow_rows = 0;
    let mut z = 0.0;
    while z <= 20.0001 {
        let width = strip_width(z);
        if width > 0.0 {
            let l = far(width, z);
            let foot = mitred
                .evaluate(Vector3::new(-20.0, -118.5, z))
                .expect("the field reads")
                .nearest;
            let (m, how) = rows.tangent_meet(l, far_normal, foot, foot);
            let (b_worst, b_mid) = rows.row(l, m);
            let (m_c, how_c) = rows.tangent_meet(l, far_normal, foot, tip);
            let (c_worst, _) = rows.row(l, m_c);
            let (a_worst, _) = rows.row(l, tip);
            worst_flow = worst_flow.max(b_worst);
            worst_flow_a = worst_flow_a.max(a_worst);
            worst_flow_c = worst_flow_c.max(c_worst);
            flow_rows += 1;
            let _ = writeln!(
                log,
                "4.   z {z:.2} L {} foot {}: (a) {}, (b) {}/{} {how}, (c) {} {how_c}",
                p(l),
                p(foot),
                up_to(a_worst, 3),
                up_to(b_worst, 3),
                up_to(b_mid, 3),
                up_to(c_worst, 3)
            );
        }

        z += 0.02;
    }

    let _ = writeln!(
        log,
        "4. {flow_rows} flowed rows: worst arc vertex to control (a) PointAt {}, (b) {}, (c) {}",
        up_to(worst_flow_a, 4),
        up_to(worst_flow, 4),
        up_to(worst_flow_c, 4)
    );

    publish(&log, ".flow.txt");
}
