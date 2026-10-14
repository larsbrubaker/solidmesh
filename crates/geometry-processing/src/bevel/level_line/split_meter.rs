//! A measurement only: every vertex the cut ADDS to a `LevelLineMesh`, and whether it lands on top
//! of one already there: what added a vertex ([`SplitMeterSplitKind`]) and the record kept of it
//! ([`SplitMeterSplitRecord`]), and the meter's state, switch, report and vertex grid
//! ([`SplitMeter`]) with the record it makes of an added vertex, and
//! the record's text ([`SplitMeterSplitRecord::describe`]); the hooks that note a vertex are in
//! `level_line_mesh_edits`.
//!
//! Costs one static bool read per vertex added when it is off: [`SplitMeter::on`] is false until a
//! caller calls [`SplitMeter::begin`] AND the field counters' meter is on. It changes nothing the
//! kernel builds. Not per thread, unlike the field counters: one caller turns it on round one
//! build and reads it back on the same thread, and the seeding is keyed on the mesh INSTANCE, so a
//! second mesh under the meter starts the grid again. The meter's state is one process-wide
//! `Mutex`.

use std::collections::HashMap;
use std::sync::{Mutex, MutexGuard};

use vector_math::number_format::{self, format_i64_number};
use vector_math::Vector3;

use super::refinement_splits;
use super::stopwatch::e3;
use super::{FieldCounters, LevelLineMesh, LevelLineRule, PointPlace};

/// What a vertex was added BY - the three ways a position enters the
/// mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SplitMeterSplitKind {
    /// An edge split: every triangle on the edge was divided with it.
    #[default]
    OnEdge = 0,

    /// A point put in a triangle's interior; its three edges were left alone.
    Interior,

    /// A free-standing vertex the fill laid in, on no triangle yet.
    Free,
}

/// One added vertex, kept so a later coincidence can say what the
/// vertex it landed on was.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SplitMeterSplitRecord {
    /// The added vertex.
    pub vertex: i32,
    /// Where it was added.
    pub point: Vector3,
    /// How it was added.
    pub kind: SplitMeterSplitKind,
    /// Whichever refinement rule's search was running when the split was asked for.
    pub rule: LevelLineRule,
    /// The context triangle.
    pub triangle: i32,
    /// The context triangle's first corner.
    pub corner_a: i32,
    /// The context triangle's second corner.
    pub corner_b: i32,
    /// The context triangle's third corner.
    pub corner_c: i32,
    /// The edge that was split, for [`SplitMeterSplitKind::OnEdge`]: its first end; -1 otherwise.
    pub edge_from: i32,
    /// The split edge's second end; -1 otherwise.
    pub edge_to: i32,
    /// How many triangles stood on the split edge - ONE means the split could not be conforming.
    pub triangles_on_the_edge: i32,
    /// How far the point stood off the context triangle's edge AB.
    pub distance_ab: f64,
    /// How far the point stood off the context triangle's edge BC.
    pub distance_bc: f64,
    /// How far the point stood off the context triangle's edge CA.
    pub distance_ca: f64,
    /// The length `RefinementSplits.WhereThePointStands` compared those three against when the
    /// field rules ask it - a millionth of the context triangle.
    pub edge_tolerance: f64,
    /// That routine's own three-way verdict on the context triangle. It is recorded rather than
    /// derived, because a point at a CORNER and a point in the INTERIOR are two different findings
    /// and the on-an-edge question answers no to both of them.
    pub place: PointPlace,
    /// The corner, or the named edge's first end; -1 in the interior.
    pub judged_from: i32,
    /// The same corner again, or the named edge's second end; -1 in the interior.
    pub judged_to: i32,
}

impl SplitMeterSplitRecord {
    /// The record as one line of text - what added the vertex, the rule,
    /// the context triangle and its corners, the point's distances off the triangle's edges
    /// against the tolerance (`E3`), the placement's verdict and the point (`F6`,
    /// InvariantCulture).
    pub fn describe(&self) -> String {
        let place = match self.kind {
            SplitMeterSplitKind::OnEdge => format!(
                "on-edge split of ({},{}), {} triangle(s) on it",
                self.edge_from, self.edge_to, self.triangles_on_the_edge
            ),
            SplitMeterSplitKind::Interior => "interior point".to_string(),
            SplitMeterSplitKind::Free => "free vertex (fill)".to_string(),
        };
        let judged = match self.place {
            PointPlace::OnAnEdge => format!("stands on ({},{})", self.judged_from, self.judged_to),
            PointPlace::AtACorner => format!("stands at corner {}", self.judged_from),
            PointPlace::Interior => "judged interior".to_string(),
        };
        let f6 = |v: f64| number_format::format_f64_fixed(v, 6);
        format!(
            "{place}; rule {:?}; triangle {} [{},{},{}]; off its edges AB {}, BC {}, CA {} against {} - {judged}; at ({}, {}, {})",
            self.rule,
            self.triangle,
            self.corner_a,
            self.corner_b,
            self.corner_c,
            e3(self.distance_ab),
            e3(self.distance_bc),
            e3(self.distance_ca),
            e3(self.edge_tolerance),
            f6(self.point.x),
            f6(self.point.y),
            f6(self.point.z),
        )
    }

    /// Builds a record.
    #[allow(clippy::too_many_arguments)] // One field per argument.
    pub fn new(
        vertex: i32,
        point: Vector3,
        kind: SplitMeterSplitKind,
        rule: LevelLineRule,
        triangle: i32,
        corner_a: i32,
        corner_b: i32,
        corner_c: i32,
        edge_from: i32,
        edge_to: i32,
        triangles_on_the_edge: i32,
        distance_ab: f64,
        distance_bc: f64,
        distance_ca: f64,
        edge_tolerance: f64,
        place: PointPlace,
        judged_from: i32,
        judged_to: i32,
    ) -> Self {
        Self {
            vertex,
            point,
            kind,
            rule,
            triangle,
            corner_a,
            corner_b,
            corner_c,
            edge_from,
            edge_to,
            triangles_on_the_edge,
            distance_ab,
            distance_bc,
            distance_ca,
            edge_tolerance,
            place,
            judged_from,
            judged_to,
        }
    }
}

/// The first this many coincidences are recorded in full; the rest are only counted.
pub const SPLIT_METER_CAP: usize = 40;

/// The split meter's process-wide state.
#[derive(Debug, Default)]
pub struct SplitMeterState {
    /// Within this of a vertex already there is a coincidence; 0 when the meter is off.
    pub coincident: f64,
    /// The boundary between the two near-miss bands.
    pub middling: f64,
    /// The outer edge of the near misses, and the grid's cell size.
    pub near: f64,
    /// The address of the mesh the grid was seeded from (compared by identity, not contents);
    /// `None` until the first vertex is noted.
    pub mesh_under_the_meter: Option<usize>,
    /// The vertices by grid cell.
    pub grid: HashMap<(i64, i64, i64), Vec<i32>>,
    /// The added vertices' records, by vertex.
    pub records: HashMap<i32, SplitMeterSplitRecord>,
    /// The coincidences, in the order they were made.
    pub duplicates: Vec<String>,
    pub coincident_count: i64,
    pub near_middling: i64,
    pub near_far: i64,
    pub added: i64,
}

static STATE: Mutex<Option<SplitMeterState>> = Mutex::new(None);

/// The split meter (see the module docs).
pub struct SplitMeter;

impl SplitMeter {
    /// Runs `f` on the meter's state. Never call a `SplitMeter` method from inside `f`.
    pub fn with<R>(f: impl FnOnce(&mut SplitMeterState) -> R) -> R {
        let mut guard: MutexGuard<'_, Option<SplitMeterState>> =
            // A panic while metering leaves numbers, not invariants, behind: read on.
            STATE.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        f(guard.get_or_insert_with(SplitMeterState::default))
    }

    /// True when a vertex added to a mesh should be weighed against the ones already there. Off
    /// unless a caller asked for the meter AND the per-rule meter is on.
    pub fn on() -> bool {
        Self::with(|s| s.coincident > 0.0) && FieldCounters::meter_on()
    }

    /// Starts the meter for a pick of this size. Coincidences are recorded at a millionth of the
    /// size, the length the FIELD rules read a corner and an edge by in
    /// `RefinementSplits.WhereThePointStands` - and near misses out to a hundredth of it are
    /// counted in two bands, so a point that merely stands NEAR a long edge can be told from one
    /// that stands on it.
    pub fn begin(size: f64) {
        Self::with(|s| {
            *s = SplitMeterState {
                coincident: 1e-6 * size,
                middling: 1e-4 * size,
                near: 1e-2 * size,
                ..SplitMeterState::default()
            };
        });
    }

    /// Stops the meter; the tallies stay for the report.
    pub fn end() {
        Self::with(|s| {
            s.coincident = 0.0;
            s.grid = HashMap::new();
            s.records = HashMap::new();
            s.mesh_under_the_meter = None;
        });
    }

    /// The coincidences, in the order they were made, and the tallies behind them.
    pub fn report() -> String {
        Self::with(|s| {
            let mut lines = vec![format!(
                "split meter: {} vertices added, {} of them within {} of one already there; \
                 near misses {} in [{}, {}), {} in [{}, {})",
                format_i64_number(s.added),
                format_i64_number(s.coincident_count),
                e3(s.coincident),
                format_i64_number(s.near_middling),
                e3(s.coincident),
                e3(s.middling),
                format_i64_number(s.near_far),
                e3(s.middling),
                e3(s.near)
            )];
            lines.extend(s.duplicates.iter().cloned());
            lines.join("\n")
        })
    }

    /// The record of a vertex added at `point`: the context triangle's corners, how far the point
    /// stood off each of its edges, and the production routine's OWN three-way answer, asked of the
    /// same triangle and point - not a copy of its arithmetic, so the meter cannot disagree with
    /// the code that took the decision. Asked by the field rules' length, since that is the one
    /// the counters are filed by. A `triangle` not in the mesh records no context.
    #[allow(clippy::too_many_arguments)] // One reading per argument.
    pub fn describe(
        mesh: &LevelLineMesh,
        vertex: i32,
        point: Vector3,
        kind: SplitMeterSplitKind,
        triangle: i32,
        from: i32,
        to: i32,
        triangles_on_the_edge: i32,
    ) -> SplitMeterSplitRecord {
        let rule = FieldCounters::with(|t| t.current_rule);
        if triangle < 0 || triangle >= mesh.triangle_count() {
            return SplitMeterSplitRecord::new(
                vertex,
                point,
                kind,
                rule,
                triangle,
                -1,
                -1,
                -1,
                from,
                to,
                triangles_on_the_edge,
                f64::NAN,
                f64::NAN,
                f64::NAN,
                f64::NAN,
                PointPlace::Interior,
                -1,
                -1,
            );
        }

        let t = mesh.triangles()[triangle as usize];
        let size = refinement_splits::longest_edge_length(mesh, t);
        let mut off = [0.0; 3];
        for (corner, distance) in off.iter_mut().enumerate() {
            let corner = corner as i32;
            *distance = Self::off_the_edge(
                mesh.position(t.corner(corner)),
                mesh.position(t.corner((corner + 1) % 3)),
                point,
            );
        }

        let (place, judged_from, judged_to) =
            refinement_splits::where_the_point_stands(mesh, t, point, None);
        SplitMeterSplitRecord::new(
            vertex,
            point,
            kind,
            rule,
            triangle,
            t.a,
            t.b,
            t.c,
            from,
            to,
            triangles_on_the_edge,
            off[0],
            off[1],
            off[2],
            1e-6 * size,
            place,
            judged_from,
            judged_to,
        )
    }

    /// How far `point` stands off the segment `a b`.
    pub fn off_the_edge(a: Vector3, b: Vector3, point: Vector3) -> f64 {
        let along = b - a;
        let length_squared = along.length_squared();
        if length_squared <= 0.0 {
            return (point - a).length();
        }

        let s = (point - a).dot(along) / length_squared;
        // NaN passes through the clamp, so a degenerate reading stays visible as NaN.
        let s = s.clamp(0.0, 1.0);
        (point - (a + (along * s))).length()
    }

    /// The nearest vertex the grid holds to `point` in the 27 cells round it, other than
    /// `ignore`, and how far it stands; (-1, +infinity) for none.
    pub fn nearest(
        state: &SplitMeterState,
        mesh: &LevelLineMesh,
        point: Vector3,
        ignore: i32,
    ) -> (i32, f64) {
        let mut best = -1;
        let mut best_distance = f64::INFINITY;
        let cell = Self::cell(state, point);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(here) = state.grid.get(&(cell.0 + dx, cell.1 + dy, cell.2 + dz))
                    else {
                        continue;
                    };

                    for &v in here {
                        if v == ignore {
                            continue;
                        }

                        let distance = (mesh.position(v) - point).length();
                        if distance < best_distance {
                            best_distance = distance;
                            best = v;
                        }
                    }
                }
            }
        }

        (best, best_distance)
    }

    /// Files `vertex` at `point` in the grid.
    pub fn place(state: &mut SplitMeterState, vertex: i32, point: Vector3) {
        let cell = Self::cell(state, point);
        state
            .grid
            .entry(cell)
            .or_insert_with(|| Vec::with_capacity(2))
            .push(vertex);
    }

    /// The grid cell holding `point`. One cell per NEAR radius, so the 27 cells round a point
    /// cover every near miss as well as every coincidence - one grid answers both questions.
    /// (`as` saturates an out-of-range value, so a point far off still gets a cell.)
    pub fn cell(state: &SplitMeterState, point: Vector3) -> (i64, i64, i64) {
        (
            (point.x / state.near).floor() as i64,
            (point.y / state.near).floor() as i64,
            (point.z / state.near).floor() as i64,
        )
    }
}

impl SplitMeter {
    /// An edge split: the point, the edge it was taken on, and the first triangle that stood on it.
    pub fn note_edge_split(
        mesh: &LevelLineMesh,
        vertex: i32,
        point: Vector3,
        from: i32,
        to: i32,
        standing: &[i32],
    ) {
        let triangle = standing.first().copied().unwrap_or(-1);
        Self::note(
            mesh,
            vertex,
            point,
            SplitMeterSplitKind::OnEdge,
            triangle,
            from,
            to,
            standing.len() as i32,
        );
    }

    /// A point put inside a triangle.
    pub fn note_triangle_split(mesh: &LevelLineMesh, vertex: i32, point: Vector3, triangle: i32) {
        Self::note(
            mesh,
            vertex,
            point,
            SplitMeterSplitKind::Interior,
            triangle,
            -1,
            -1,
            0,
        );
    }

    /// A free-standing vertex the fill laid in.
    pub fn note_free_vertex(mesh: &LevelLineMesh, vertex: i32, point: Vector3) {
        Self::note(
            mesh,
            vertex,
            point,
            SplitMeterSplitKind::Free,
            -1,
            -1,
            -1,
            0,
        );
    }

    /// Weighs an added vertex against the ones already there: a fresh mesh seeds the grid with
    /// everything on it before the first add (an input vertex is as much "one already there" as a
    /// split is), a coincidence is recorded in full up to [`SPLIT_METER_CAP`], and a near miss is
    /// counted in its band.
    #[allow(clippy::too_many_arguments)] // One reading per argument.
    pub fn note(
        mesh: &LevelLineMesh,
        vertex: i32,
        point: Vector3,
        kind: SplitMeterSplitKind,
        triangle: i32,
        from: i32,
        to: i32,
        triangles_on_the_edge: i32,
    ) {
        if !Self::on() {
            return;
        }

        // Read before taking the meter's state: the record reads the field counters' rule.
        let record = Self::describe(
            mesh,
            vertex,
            point,
            kind,
            triangle,
            from,
            to,
            triangles_on_the_edge,
        );
        let identity = mesh as *const LevelLineMesh as usize;
        Self::with(|s| {
            if s.mesh_under_the_meter != Some(identity) {
                s.mesh_under_the_meter = Some(identity);
                s.grid.clear();
                s.records.clear();
                let mut v = 0;
                while v < vertex && v < mesh.vertex_count() {
                    Self::place(s, v, mesh.position(v));
                    v += 1;
                }
            }

            s.added += 1;
            s.records.insert(vertex, record);

            let (nearest, distance) = Self::nearest(s, mesh, point, vertex);
            if nearest >= 0 && distance <= s.coincident {
                s.coincident_count += 1;
                if s.duplicates.len() < SPLIT_METER_CAP {
                    let before = match s.records.get(&nearest) {
                        Some(older) => older.describe(),
                        None => {
                            let at = mesh.position(nearest);
                            format!("INPUT vertex at ({:.6}, {:.6}, {:.6})", at.x, at.y, at.z)
                        }
                    };
                    let index = s.duplicates.len() + 1;
                    s.duplicates.push(format!(
                        "  #{index} vertex {vertex} landed on {nearest}, {} apart\n      new:      {}\n      standing: {before}",
                        e3(distance),
                        record.describe()
                    ));
                }
            } else if nearest >= 0 && distance < s.near {
                if distance < s.middling {
                    s.near_middling += 1;
                } else {
                    s.near_far += 1;
                }
            }

            Self::place(s, vertex, point);
        });
    }
}
