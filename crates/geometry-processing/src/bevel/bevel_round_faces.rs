//! Which faces of a part are facets of a ROUND, and of which round. Built on
//! [`BevelRoundDescriptor`]'s containment and tangent-plane tests; `BevelSourceRounds` (the rounds
//! a part arrives with) labels through [`BevelRoundFaces::label`] too, and the sweep
//! (`BevelChainSweep.TryStandOnTheRound`) and `BevelRoundEntry` read it.
//!
//! It is the one thing the sweep cannot work out for itself: a facet is a plane like any other,
//! and only something outside the sweep - the first pass's own resolutions, or a fit of the
//! surface the part arrived with - says that this plane is a chord of a cylinder.
//!
//! Keyed by MESH FACE rather than by mesh edge, because that is what the sweep has: a `ChainEdge`
//! carries its two face indices, and either of them may be the round. The crease the second pass
//! re-reads runs between a round and a planar slope, so at most one of the two ever answers.
//!
//! **Two of them, from two places.** [`BevelRoundFaces::create`] describes the rounds the FIRST
//! PASS CUT, off its own resolutions, and is built once per `BevelSecondPass.Rebead` and handed
//! down: on the uncut part those rounds do not exist yet. `BevelSourceRounds.Create` fits the
//! rounds the part ARRIVED with - a modelled bore or boss - and hands the result to the first
//! pass, which is the only pass whose crease can run along one. Both end up here, through
//! [`BevelRoundFaces::label`], and every reader downstream treats them alike but for the one flag
//! that says which they are ([`BevelRoundFaces::read_from_the_source_mesh`]).
//!
//! Building it is every face of the part against every round described, which is why it is built
//! once for the whole pass and why the walk drops a face against the rounds' shared bounding box
//! before it asks any round anything. A part whose selection describes many rounds would want them
//! in a spatial index rather than a list; nothing measured yet has asked for it.

use std::collections::HashMap;

use polygon_mesh::Mesh;
use vector_math::{math_helper, AxisAlignedBoundingBox, Vector3};

use super::BevelRoundDescriptor;
use crate::topology::{BevelEdgeGroupResolution, EdgeConvexity, FeatureEdge};

/// How far off a round's radius a vertex may sit and still be read as on it, as a fraction of the
/// size. It only has to swallow what the boolean's arithmetic moved the cut surface by; the
/// SAGITTA of a facet is allowed for separately ([`BevelRoundDescriptor::contains`]).
const RADIUS_TOLERANCE_FRACTION: f64 = 1e-3;

/// How near a span-end radial a planar face's normal has to point before that face is read as the
/// round's TANGENT PLANE there. Wide enough to swallow what the boolean did to the cut surface and
/// far too narrow to catch the next facet round the arc, which is a whole 180 / segments away.
const TANGENT_PLANE_ANGLE_DEGREES: f64 = 2.0;

/// How far off the tangent LINE a corner of such a face may sit and still be read as on it, as a
/// fraction of the size. Looser than [`RADIUS_TOLERANCE_FRACTION`] because this vertex is one the
/// BOOLEAN built - it is where the cutter's last facet ran out onto the flat - rather than one the
/// first pass placed itself.
const TANGENT_LINE_TOLERANCE_FRACTION: f64 = 1e-2;

/// Which faces of a part lie on a round, and which are a round's tangent planes. Lookups only, so
/// the face maps are `HashMap`s: no iteration order reaches a result.
#[derive(Debug, Clone)]
pub struct BevelRoundFaces {
    by_face: HashMap<i32, BevelRoundDescriptor>,
    tangent_planes: HashMap<i32, BevelRoundDescriptor>,
    read_from_the_source_mesh: bool,
}

impl BevelRoundFaces {
    /// The fewest chords an arc may be drawn with and still be read as a round - see `describe`
    /// for why a coarser one is not a surface the ball can be rolled on.
    pub const MINIMUM_SEGMENTS_TO_ROLL_ON: i32 = 4;

    /// Whether these rounds were read off the SOURCE mesh (`BevelSourceRounds`) rather than
    /// described from the cutters the first pass ran.
    ///
    /// The two are not equally safe to take up. A cut round is one the operation itself put there
    /// and the second pass has always drawn its sections against it; a round the part arrived with
    /// is under EVERY boss fillet in every part ever built, where the facet plane the section was
    /// inscribed in is already the surface the ball rolls on. So a source round is taken up only
    /// where the contact has rolled off the crease's own facet - see
    /// `BevelChainSweep.TryStandOnTheRound`.
    pub fn read_from_the_source_mesh(&self) -> bool {
        self.read_from_the_source_mesh
    }

    /// How many faces of the part were read as being on a round.
    pub fn count(&self) -> usize {
        self.by_face.len()
    }

    /// How many were read as one of a round's two tangent planes.
    pub fn tangent_plane_count(&self) -> usize {
        self.tangent_planes.len()
    }

    /// Reads the rounds the first pass cut into the part, and labels the faces that lie on them.
    ///
    /// `cut_mesh` is the part with every first-pass cutter already taken out of it; `resolutions`
    /// what the first pass made of the groups, on the uncut part; `eligible` which of them the
    /// first pass was willing to build a tool for; `size` the operation's size, which is every
    /// round's radius; `segments` how many chords each round's arc was drawn with.
    ///
    /// `round_profile` is whether the tool's profile is an arc (normally true). A CHAMFER's first
    /// pass left flat cuts rather than rounds, and both readers of this -
    /// `BevelChainSweep.TryStandOnTheRound` and `BevelRoundEntry` - are gated on the same flag, so
    /// nothing would ever be asked of what this walk of the whole mesh would find. It answers with
    /// an empty one instead.
    pub fn create(
        cut_mesh: &Mesh,
        resolutions: &[BevelEdgeGroupResolution],
        eligible: &[bool],
        size: f64,
        segments: i32,
        round_profile: bool,
    ) -> BevelRoundFaces {
        if !round_profile {
            return BevelRoundFaces {
                by_face: HashMap::new(),
                tangent_planes: HashMap::new(),
                read_from_the_source_mesh: false,
            };
        }

        let mut rounds = Vec::new();

        for (i, resolution) in resolutions.iter().enumerate() {
            if !eligible[i] || !resolution.is_resolved() {
                continue;
            }

            for edge in &resolution.edges {
                if edge.convexity != EdgeConvexity::Convex {
                    continue;
                }

                if let Some(described) = describe(edge, size, segments) {
                    Self::add(&mut rounds, described, size * RADIUS_TOLERANCE_FRACTION);
                }
            }
        }

        Self::label(cut_mesh, &rounds, size, false)
    }

    /// Labels the faces of a part against a list of rounds already described: which of them lie
    /// ON one, and which are one's TANGENT PLANE.
    ///
    /// The half of [`Self::create`] that does not care where the rounds came from, so a round read
    /// off the SOURCE mesh (`BevelSourceRounds`) is labelled by exactly the same tests the cut ones
    /// are and every reader downstream - `BevelChainSweep.TryStandOnTheRound`, `BevelRoundEntry` -
    /// works unchanged. `size` is the operation's size, which every tolerance below is a fraction
    /// of; `read_from_the_source_mesh` (normally false) whether these rounds were read off the part
    /// as it arrived - see [`Self::read_from_the_source_mesh`].
    pub fn label(
        part: &Mesh,
        rounds: &[BevelRoundDescriptor],
        size: f64,
        read_from_the_source_mesh: bool,
    ) -> BevelRoundFaces {
        let mut by_face = HashMap::new();
        let mut tangent_planes = HashMap::new();

        if !rounds.is_empty() {
            let tolerance = size * RADIUS_TOLERANCE_FRACTION;
            let tangent_tolerance = size * TANGENT_LINE_TOLERANCE_FRACTION;
            let tangent_dot = math_helper::degrees_to_radians(TANGENT_PLANE_ANGLE_DEGREES).cos();

            // The walk below is every face against every round, and a part can carry hundreds of
            // both. The one cheap thing that can be said before either test is asked is WHERE the
            // rounds are: both tests need a corner within a radius of an axis (Contains all three,
            // IsTangentPlane the one that sits on the tangent line), so a face with no corner in
            // the box every round's neighbourhood fits inside cannot answer either, whatever round
            // is put to it.
            let near = neighbourhood_of_the_rounds(rounds, size);

            for (face, mesh_face) in part.faces.iter().enumerate() {
                let face = face as i32;
                let a = Vector3::from_vector3_float(part.vertices[mesh_face.v0 as usize]);
                let b = Vector3::from_vector3_float(part.vertices[mesh_face.v1 as usize]);
                let c = Vector3::from_vector3_float(part.vertices[mesh_face.v2 as usize]);

                if !near.contains_point(a) && !near.contains_point(b) && !near.contains_point(c) {
                    continue;
                }

                // All three corners, so a face that merely touches the round at one vertex - every
                // face of the flat nose the round is tangent to has one - is not mistaken for it.
                if let Some(round) = rounds.iter().find(|round| {
                    round.contains(a, tolerance, size)
                        && round.contains(b, tolerance, size)
                        && round.contains(c, tolerance, size)
                }) {
                    by_face.insert(face, round.clone());
                    continue;
                }

                // AND THE TWO FLAT FACES EACH ROUND RUNS OUT ONTO. They are not on the round and
                // never carry an arc section of their own, but a ball rolling in a crease that
                // crosses one of them can already be standing on the ROUND while the crease point
                // is still out on the flat - the contact leads the crease by up to a radius - and a
                // section drawn against the flat plane there is that whole lead off the canal. See
                // BevelRoundEntry, which cuts such an edge where the ball leaves the span so each
                // piece gets the section its own midpoint really stands on.
                let normal = (b - a).cross(c - a);

                let area = normal.length();
                if area.is_nan() || area <= 0.0 {
                    continue;
                }

                let normal = normal.get_normal();

                if let Some(round) = rounds.iter().find(|round| {
                    round.is_tangent_plane(normal, a, b, c, tangent_dot, tangent_tolerance, size)
                }) {
                    tangent_planes.insert(face, round.clone());
                }
            }
        }

        BevelRoundFaces {
            by_face,
            tangent_planes,
            read_from_the_source_mesh,
        }
    }

    /// The round a face belongs to and whether the face is one of its tangent planes, or `None`
    /// where it belongs to none.
    ///
    /// The flag is false where the face is a FACET of the round - its own plane a chord of the
    /// cylinder - and true where it is one of the two flat faces the round runs out onto
    /// tangentially. Both answer with the same round, because a ball rolling in a crease across
    /// either of them may be standing on the round's surface. What differs is that a tangent
    /// plane's crease has a stretch where it is NOT, which is why `BevelRoundEntry` cuts such an
    /// edge in two.
    pub fn try_read_face(&self, face: i32) -> Option<(&BevelRoundDescriptor, bool)> {
        if let Some(round) = self.by_face.get(&face) {
            return Some((round, false));
        }

        self.tangent_planes
            .get(&face)
            .map(|tangent| (tangent, true))
    }

    /// Adds a round to a list, merging it into one already there where it is the same cylinder.
    pub fn add(
        rounds: &mut Vec<BevelRoundDescriptor>,
        round: BevelRoundDescriptor,
        tolerance: f64,
    ) {
        for existing in rounds.iter_mut() {
            if existing.is_the_same_round_as(&round, tolerance) {
                *existing = existing.merged(&round);
                return;
            }
        }

        rounds.push(round);
    }
}

/// The cylinder one convex edge's cutter left, or `None` where the two faces fold flat back on
/// each other and there is no inscribed arc to describe.
///
/// Also `None` below [`BevelRoundFaces::MINIMUM_SEGMENTS_TO_ROLL_ON`] chords. What the cutter drew
/// there is a coarse prism, not a surface a ball can be rolled on: the sagitta of a chord is a
/// fifth of the radius at three chords and the whole of it at one, so a section drawn against the
/// ANALYTIC cylinder would stand that far off the facets the part actually has - worse than the
/// facet plane the section is inscribed in without any of this.
fn describe(edge: &FeatureEdge, size: f64, segments: i32) -> Option<BevelRoundDescriptor> {
    if segments < BevelRoundFaces::MINIMUM_SEGMENTS_TO_ROLL_ON {
        return None;
    }

    let along = edge.end - edge.start;
    let length = along.length();

    if length.is_nan() || length <= 0.0 {
        return None;
    }

    let opening = 1.0 + edge.normal0.dot(edge.normal1);

    if opening < 1e-6 {
        return None;
    }

    // The one point a size inside both face planes - see BevelRoundDescriptor's remarks. It is
    // square to the edge because both normals are, so the edge's own span along the axis starts
    // at zero.
    let axis_point = edge.start - ((edge.normal0 + edge.normal1) * (size / opening));

    Some(BevelRoundDescriptor::new(
        axis_point,
        edge.direction,
        size,
        edge.normal0,
        edge.normal1,
        0.0,
        length,
        segments,
    ))
}

/// The box every round's own neighbourhood fits inside: each axis walked from its axial extent
/// `slack` either way, grown by a radius and the loosest tolerance any test below uses. A point
/// outside it is outside every round's [`BevelRoundDescriptor::contains`] and off every tangent
/// line, so the face that owns it can be dropped without asking either question of any round.
fn neighbourhood_of_the_rounds(
    rounds: &[BevelRoundDescriptor],
    slack: f64,
) -> AxisAlignedBoundingBox {
    let mut bounds = AxisAlignedBoundingBox::new(
        Vector3::new(f64::MAX, f64::MAX, f64::MAX),
        Vector3::new(f64::MIN, f64::MIN, f64::MIN),
    );

    for round in rounds {
        let reach = Vector3::new(1.0, 1.0, 1.0)
            * (round.radius + (slack * TANGENT_LINE_TOLERANCE_FRACTION));
        let from = round.axis_point + (round.axis_direction * (round.axial_from - slack));
        let to = round.axis_point + (round.axis_direction * (round.axial_to + slack));

        bounds.expand_to_include_point(from - reach);
        bounds.expand_to_include_point(from + reach);
        bounds.expand_to_include_point(to - reach);
        bounds.expand_to_include_point(to + reach);
    }

    bounds
}
