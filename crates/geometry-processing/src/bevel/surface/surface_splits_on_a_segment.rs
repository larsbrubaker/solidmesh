//! Which of a cut's new vertices stand along a stretch of a mesh ([`SurfaceSplitsOnASegment`]) -
//! asked of the stretch's two END VERTICES and of nothing else, so that every face meeting along
//! one stretch is told the same thing about it. Ports the fields of
//! `Bevel/Surface/SurfaceSplitsOnASegment.cs` and its stretch readings (`between`, refined to a
//! standstill); its constructor lands with a later package as a further `impl` block.

use std::collections::{HashMap, HashSet};

use vector_math::Vector3;

use super::{surface_split_sewing, SurfaceVertexPool};

/// C# `SurfaceSplitsOnASegment`: the splits standing along each stretch, refined to a standstill
/// and remembered by the stretch's two end vertices.
///
/// The sewing reads each side of a face for the splits standing on it (`SurfaceSplitSewing`). Read
/// against the straight chord between the side's two corners that misses points: the splits are
/// arrived at by other roads, so each stands somewhere inside the ruler of the chord rather than on
/// it, and the polyline they draw bows off the chord by nearly a whole ruler. So the stretch is
/// refined: the splits found are spliced in, and each piece between two neighbours is read again,
/// to a standstill.
///
/// The thing that makes it safe is the KEY. The answer depends on the two end vertices and nothing
/// else, so face A's stretch from a split to a split and face B's side from that same split to that
/// same corner are one question asked twice, and get one answer. A reading that depended on what
/// the face asking had already taken in would not: measured on the Round Side at radius 8.5, one
/// face saw a row of four stations between two of its own splits and took them in, while the face
/// across the line saw the same row between a split and a CORNER and did not - six open edges at
/// each of two places.
///
/// Lives for one `SewInTheSplits` pass, which only READS the pool while it does, so the pool is
/// borrowed rather than shared. C#'s initializers - `Settled = true`, an empty memo - are the
/// constructor's to set, as there is no default pool to borrow.
#[derive(Debug, Clone)]
pub struct SurfaceSplitsOnASegment<'a> {
    /// The pool the stretches' vertices are read from.
    pub pool: &'a SurfaceVertexPool,
    /// The candidate splits: one stage's own new vertices, which is what the sewing is handed.
    pub candidates: Vec<i32>,
    /// What has been read so far, keyed by the stretch's two end vertices, low first. Only ever
    /// looked up by key, never iterated, so a `HashMap`'s order cannot reach an output.
    pub known: HashMap<(i32, i32), Vec<i32>>,
    /// Whether every stretch read so far settled (C# initializer `true`). A reading that did not
    /// settle has handed back a stretch that is CUT SHORT of the refinement, and what is built on
    /// it is not to be published: see [`SurfaceSplitsOnASegment::the_stretch_that_did_not_settle`]
    /// and the refusal in `SurfaceSplitSewing.SewInTheSplits`.
    pub settled: bool,
    /// The ends of the first stretch whose refinement did not settle, for the advisory to point at.
    pub the_stretch_that_did_not_settle: Option<(i32, i32)>,
}

/// The stretch readings (C# `Between`, `Refine`, `StandingOn`).
///
/// C# keeps each stretch's list in `known` BY REFERENCE while its refinement runs, so a piece read
/// part way through sees what the stretch carries so far. Here the refinement works on its own
/// copy and writes it back into `known` after every change, which hands a nested reading of the
/// same key the same contents at the same moment.
impl<'a> SurfaceSplitsOnASegment<'a> {
    /// C#'s constructor (with its field initializers: nothing known yet, settled).
    pub fn new(pool: &'a SurfaceVertexPool, candidates: Vec<i32>) -> Self {
        SurfaceSplitsOnASegment {
            pool,
            candidates,
            known: HashMap::new(),
            settled: true,
            the_stretch_that_did_not_settle: None,
        }
    }
}

impl SurfaceSplitsOnASegment<'_> {
    /// C# `Between`: the candidates standing along the stretch from `from` to `to`, in order from
    /// one to the other, neither end included.
    pub fn between(&mut self, from: i32, to: i32) -> Vec<i32> {
        let key = (from.min(to), from.max(to));

        let along = match self.known.get(&key) {
            Some(along) => along.clone(),
            None => {
                // Written down BEFORE the refinement runs, and empty: a piece of a stretch cannot be a
                // piece of itself, but two points nearer than the ruler can each read as standing on
                // the other's stretch, and an empty answer breaks that walk rather than following it
                // round.
                self.known.insert(key, Vec::new());

                self.refine(key.0, key.1);

                self.known.get(&key).cloned().unwrap_or_default()
            }
        };

        if from == key.0 {
            return along;
        }

        let mut back = along;

        back.reverse();

        back
    }

    /// C# `Refine`: the stretch read to a standstill - what stands on it directly, then what stands
    /// on each piece between two neighbours, until no piece gains a point.
    ///
    /// It stops BY CONSTRUCTION: every take-in puts on the stretch at least one candidate the
    /// stretch was not already carrying, and there are only so many candidates, so the loop runs at
    /// most that many times. Without that floor it does not stop at all - THREE points in a cluster
    /// finer than the ruler each stand on the line between the other two, so the piece from the
    /// first to the second hands back the third, which the stretch already carries, and the walk
    /// goes round that triangle splicing the same points in again and again
    /// (`SurfaceSplitsOnASegmentTests`).
    ///
    /// A crowd like that has no answer to give: there IS no order of those three along the stretch,
    /// and any of the three orders leaves a point of the cluster off some face's side. So the
    /// refinement stops where it stands and the reading says it did not settle, for the caller to
    /// refuse by name - never a panic, and never a stretch that visits one vertex twice.
    fn refine(&mut self, from: i32, to: i32) {
        let key = (from, to);
        let mut along = Vec::new();

        self.standing_on(
            self.pool.position_of(from),
            self.pool.position_of(to),
            from,
            to,
            &mut along,
        );
        self.known.insert(key, along.clone());

        let mut carried: HashSet<i32> = along.iter().copied().collect();

        let mut taking_in = true;

        while taking_in {
            taking_in = false;

            let mut walk = Vec::with_capacity(along.len() + 2);
            walk.push(from);
            walk.extend(along.iter().copied());
            walk.push(to);

            let mut step = 0;
            while step + 1 < walk.len() {
                // The piece's own answer, by its own key - so a piece that is somebody else's whole
                // stretch is read once and remembered.
                let inside = self.between(walk[step], walk[step + 1]);

                if inside.is_empty() {
                    step += 1;
                    continue;
                }

                for &point in &inside {
                    // Already on the stretch, or an end of it: the piece is handing back what it was
                    // cut out of, which is the crowd closing on itself rather than a finer reading.
                    if point == from || point == to || !carried.insert(point) {
                        self.settled = false;
                        if self.the_stretch_that_did_not_settle.is_none() {
                            self.the_stretch_that_did_not_settle = Some((from, to));
                        }

                        return;
                    }
                }

                // C# `along.IndexOf(walk[step]) + 1`: IndexOf is -1 for `from`, which is not in
                // `along`, so the piece goes in at the front.
                let at = along
                    .iter()
                    .position(|&vertex| vertex == walk[step])
                    .map_or(0, |index| index + 1);
                along.splice(at..at, inside);
                self.known.insert(key, along.clone());
                taking_in = true;
                break;
            }
        }
    }

    /// C# `StandingOn`: the candidates standing within the ruler of the straight stretch between two
    /// points, strictly between its ends, appended in order along it. C#'s `List.Sort` is unstable;
    /// this sort is stable, so candidates at one place keep the candidates' order (two welded
    /// vertices are never two candidates at one place, so the tie does not arise).
    fn standing_on(
        &self,
        from: Vector3,
        to: Vector3,
        from_vertex: i32,
        to_vertex: i32,
        along: &mut Vec<i32>,
    ) {
        let mut found: Vec<(f64, i32)> = Vec::new();

        for &candidate in &self.candidates {
            if candidate == from_vertex || candidate == to_vertex {
                continue;
            }

            let (on, where_) = surface_split_sewing::stands_on_the_side(
                self.pool,
                from,
                to,
                self.pool.position_of(candidate),
            );

            if on {
                found.push((where_, candidate));
            }
        }

        // Every find is strictly inside (0, 1), so no NaN reaches the comparison.
        found.sort_by(|first, second| first.0.total_cmp(&second.0));

        along.extend(found.into_iter().map(|(_, vertex)| vertex));
    }
}
