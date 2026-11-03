//! The room, read off a mesh and a pick without building anything: the least of what foreign
//! features bound, what two chains of one pick leave each other, and what each side holds of
//! itself, less the margin a kept strip needs to exist at all (the pair room is
//! `band_room_pair_room.rs`, and the reading of a pick of several chains is
//! `band_room_read_pick.rs`): the chains' lengths
//! ([`lengths_of`]), the room of one chain ([`read_room_of_one`], over `room_within`), the
//! past-the-end test ([`is_past_an_end_face`]), the convex end faces as a list ([`present`]), which
//! foreign reading names the room ([`the_foreign_source`]), the room read off a mesh cut into sides
//! ([`room_of_cut_mesh`]) and the room's rounding down ([`steady_down`], [`on_grid`]).

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::{ieee_math, Vector3};

use super::band_gap_reading::{self, ReadRoomError};
use super::band_side_tagging_ends::{self, SideReach, SideTaggingError};
use super::end_face_sense::{self, PointPlane};
use super::foreign_edge_room_readings::{room_for, where_the_room_is_set, FieldAt, ForeignEdge};
use super::level_line_mesh_edits::LevelLineMeshError;
use super::pick_end_bounds::PickEnds;
use super::pick_validation::PickArgumentError;
use super::seam_edges::FeatureEdgeTest;
use super::{chain_on_the_mesh, foreign_edge_room, self_room, swallowed_side_runs};
use super::{
    BandProfile, ChainEdgeNotOnTwoTriangles, ChainField, FieldSample, GrowingCreaseRuns,
    LevelLineMesh, LevelLineWork, LevelLineWorkPhase, RoomField, RoomReading, RoomSource,
    WorkBudget, WorkError,
};

/// Significant digits the room is rounded down to, so it reads the same
/// at any size asked.
pub const ROOM_DIGITS: i32 = 12;

/// The ONE size every room is read at. A setback is held per unit of
/// size, so the reading is a ratio and the number that comes out is a size; reading at the size a
/// caller asked for instead would tie the answer to the question, because the searches underneath
/// carry tolerances that scale with it.
pub const REFERENCE_SIZE: f64 = 1.0;

/// Each chain's length along its own edges.
pub fn lengths_of(positions: &[Vector3], chains: &[Vec<i32>], closed: &[bool]) -> Vec<f64> {
    let mut lengths = vec![0.0; chains.len()];
    for (k, chain) in chains.iter().enumerate() {
        let mut total = 0.0;
        for (from, to) in chain_on_the_mesh::chain_edges(chain, closed[k]) {
            total += (positions[to as usize] - positions[from as usize]).length();
        }

        lengths[k] = total;
    }

    lengths
}

/// The room of one chain, spending no more than `budget`. A reading
/// that ran out of work refuses by name (`read` false, room NaN); a chain edge not on two
/// triangles is an answer (room 0, the edge named); a cancel leaves as `Err`: it says nothing
/// about the part. Other argument faults come back as [`ReadRoomError`].
#[allow(clippy::too_many_arguments)]
pub fn read_room_of_one(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    chain_is_closed: bool,
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    budget: &Rc<WorkBudget>,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
) -> Result<RoomReading, ReadRoomError> {
    let _scope = LevelLineWork::begin(Some(budget.clone()));
    LevelLineWork::enter(LevelLineWorkPhase::Room);
    match room_within(
        positions,
        indices,
        chain,
        chain_is_closed,
        is_feature_edge,
        profile,
        snap,
        seam_epsilon,
        sharp_turn_degrees,
    ) {
        Ok(within) => Ok(RoomReading {
            read: true,
            room: within.room,
            room_set_by: within.set_by,
            chain_divides_sides: within.divides_sides,
            chain_has_a_crease_to_bevel: within.has_a_crease_to_bevel,
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }),
        // The caller asked for this, so it leaves as cancellation and never as a refusal: a refusal
        // is a statement about the part, and this one says nothing about the part at all.
        Err(RoomWithinError::Work(WorkError::Cancelled)) => Err(ReadRoomError::Cancelled),
        // The same statement about the part the build makes - see
        // BandGapRefusal::ChainEdgeNotOnTwoTriangles. The reading SUCCEEDED in the sense that it has
        // an answer: no size builds this pick.
        Err(RoomWithinError::ChainEdge(bad)) => Ok(RoomReading {
            read: true,
            room: 0.0,
            chain_edge_stands_on_two_triangles: false,
            offending_edge: (bad.from, bad.to),
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }),
        Err(RoomWithinError::Work(WorkError::BudgetExceeded(_))) => Ok(RoomReading {
            read: false,
            room: f64::NAN,
            phase: budget.exhausted_in.get(),
            work_spent: budget.spent.get(),
            work_limit: budget.limit,
            ..RoomReading::default()
        }),
        Err(RoomWithinError::Read(error)) => Err(error),
    }
}

/// What [`room_within`] reads besides the room.
#[derive(Debug, Clone, Copy)]
pub struct RoomWithin {
    /// The room.
    pub room: f64,
    /// False when the chain does not divide its sides.
    pub divides_sides: bool,
    /// False when there is no crease under the chain to round.
    pub has_a_crease_to_bevel: bool,
    /// Which reading named the room.
    pub set_by: RoomSource,
}

/// How [`room_within`] can leave without a room.
#[derive(Debug, Clone)]
pub enum RoomWithinError {
    /// The budget ran out, or the caller cancelled.
    Work(WorkError),
    /// A chain edge not on two triangles.
    ChainEdge(ChainEdgeNotOnTwoTriangles),
    /// A caller's bug: arguments no room can be read for.
    Read(ReadRoomError),
}

impl From<WorkError> for RoomWithinError {
    fn from(error: WorkError) -> Self {
        RoomWithinError::Work(error)
    }
}

impl From<ChainEdgeNotOnTwoTriangles> for RoomWithinError {
    fn from(error: ChainEdgeNotOnTwoTriangles) -> Self {
        RoomWithinError::ChainEdge(error)
    }
}

impl From<SideTaggingError> for RoomWithinError {
    fn from(error: SideTaggingError) -> Self {
        match error {
            SideTaggingError::Work(work) => RoomWithinError::Work(work),
            SideTaggingError::ChainEdge(edge) => RoomWithinError::ChainEdge(edge),
        }
    }
}

/// The room of one chain read off the input mesh tagged into its sides.
#[allow(clippy::too_many_arguments)]
fn room_within(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    chain_is_closed: bool,
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
) -> Result<RoomWithin, RoomWithinError> {
    let mut within = RoomWithin {
        room: 0.0,
        divides_sides: true,
        has_a_crease_to_bevel: true,
        set_by: RoomSource::nothing(),
    };
    if chain.len() < 2 {
        return Err(RoomWithinError::Read(ReadRoomError::Arguments(
            PickArgumentError::ChainTooShort,
        )));
    }

    let no_feature_edges = |_: i32, _: i32| false;
    let is_feature_edge: &dyn Fn(i32, i32) -> bool = match &is_feature_edge {
        Some(test) => test.as_ref(),
        None => &no_feature_edges,
    };
    let chain_edges = chain_on_the_mesh::chain_edges(chain, chain_is_closed);
    // The ends as the band is cut on them (`pick_end_bounds.rs`), less the square crease clips
    // the room is read without.
    let ends = PickEnds::of(
        positions,
        indices,
        &[chain.to_vec()],
        &[chain_is_closed],
        sharp_turn_degrees,
        is_feature_edge,
    );
    let end_rays = ends.end_rays[0];
    let end_ray_bounds = ends.bounds_without_square_crease_clips(0);
    let (tags, divides) = band_side_tagging_ends::tag_sides(
        indices,
        &chain_edges,
        is_feature_edge,
        true,
        Some(positions),
        end_rays,
        SideReach::default(),
    )?;
    if !divides && chain_is_closed {
        within.divides_sides = false;
        return Ok(within);
    }

    if !divides && !chain_edges.iter().any(|e| is_feature_edge(e.0, e.1)) {
        // One region round the chain AND no crease anywhere under it: there is nothing to round at
        // any size, and the band's own cut says the same by taking no triangle off (BandGap refuses
        // ChainHasNoCreaseToBevel). The room is NOT infinity - an unbounded room handed to a drag
        // would let a user drag a bevel that never appears - and it is not a size statement at all.
        within.has_a_crease_to_bevel = false;
        return Ok(within);
    }

    // An OPEN chain whose sides fall in ONE region is a crease that DIES OUT past its end, and it
    // builds (see BandGap and DyingCreaseFill), so its room is read off that one region exactly as
    // a divided pick's is off its two - the reading and the build must name the same number.

    let mesh = match LevelLineMesh::from_triangles(positions, indices, Some(&tags)) {
        Ok(mesh) => mesh,
        Err(LevelLineMeshError::Work(work)) => return Err(RoomWithinError::Work(work)),
        Err(other) => return Err(RoomWithinError::Read(ReadRoomError::Mesh(other))),
    };
    let unit_setback = chain_on_the_mesh::unit_setbacks(
        positions,
        indices,
        chain,
        chain_is_closed,
        &chain_edges,
        profile,
    )?;
    let chain_points: Vec<Vector3> = chain.iter().map(|&v| positions[v as usize]).collect();
    let field_of = |s: f64, rays: (bool, bool)| {
        let setbacks: Vec<f64> = unit_setback.iter().map(|u| u * s).collect();
        ChainField::from_polyline_bounded(
            &chain_points,
            chain_is_closed,
            &setbacks,
            sharp_turn_degrees,
            rays,
            end_ray_bounds,
        )
    };

    // Scaling the setbacks moves no point, so a polyline that builds a field at one size builds it
    // at every size: a chain of no length is caught once, here.
    if let Err(bad) = field_of(REFERENCE_SIZE, end_rays) {
        return Err(RoomWithinError::Read(ReadRoomError::ChainField(bad)));
    }

    let field_at = |s: f64| {
        field_of(s, end_rays).expect("the chain's polyline built a field at the reference size")
    };

    let convex_ends = end_face_sense::convex_end_faces(positions, indices, chain, end_rays);
    let without_convex_rays = (
        end_rays.0 && convex_ends.0.is_none(),
        end_rays.1 && convex_ends.1.is_none(),
    );
    let field_without_convex_rays = |s: f64| {
        field_of(s, without_convex_rays)
            .expect("the chain's polyline built a field at the reference size")
    };

    let present = present(convex_ends);
    let (room, set_by) = room_of_cut_mesh(
        &Rc::new(RefCell::new(mesh)),
        &chain_edges,
        &field_at,
        &unit_setback,
        chain_is_closed,
        snap,
        seam_epsilon,
        profile,
        0,
        Some(&present),
        Some(&field_without_convex_rays),
    )?;
    within.room = room;
    within.set_by = set_by;
    Ok(within)
}

/// True when both ends of `edge` stand more than `snap` beyond one of the convex end faces (each
/// a point on it and its outward normal).
pub fn is_past_an_end_face(
    mesh: &LevelLineMesh,
    edge: (i32, i32),
    convex_end_faces: &[(Vector3, Vector3)],
    snap: f64,
) -> bool {
    let beyond = |p: Vector3, face: &(Vector3, Vector3)| (p - face.0).dot(face.1) > snap;
    convex_end_faces
        .iter()
        .any(|face| beyond(mesh.position(edge.0), face) && beyond(mesh.position(edge.1), face))
}

/// The convex end faces `EndFaceSense.ConvexEndFaces` found, as a list.
pub fn present(ends: (Option<PointPlane>, Option<PointPlane>)) -> Vec<PointPlane> {
    let mut faces = Vec::new();
    if let Some(first) = ends.0 {
        faces.push(first);
    }

    if let Some(last) = ends.1 {
        faces.push(last);
    }

    faces
}

/// Which foreign reading a one-chain room names: a growing
/// neighbour's far crease only where it alone sets the room - strictly under every edge on the face
/// (a tie stays the face's own reading, the sentence the user has always read) and not undercut by
/// the fillet-pierce room, which is the ridge leaving the round and no crease at all. `crease_at` is
/// read only then.
pub fn the_foreign_source(
    near_room: f64,
    far_room: f64,
    room: f64,
    crease_at: &dyn Fn() -> Vector3,
) -> RoomSource {
    if room == f64::INFINITY {
        return RoomSource::nothing();
    }

    if far_room < near_room && room == far_room {
        RoomSource::growing_crease(0, crease_at())
    } else {
        RoomSource::foreign(0)
    }
}

/// The room read off a mesh tagged into sides, saying also WHICH of
/// the two - what bounds the sides from outside, or a side running out of itself - named it. The
/// chain index it carries is 0, the only chain there is.
///
/// No size enters here, and none may: the room is a statement about the part and the pick, and a
/// reading taken at the size a caller happened to ask for is a DIFFERENT number - the search
/// tolerances below scale with the size they are given, so the same geometry read at two sizes
/// answers two rooms and a build at the room a refusal named could be refused again. Everything is
/// read at [`REFERENCE_SIZE`]; the setback is held per unit of size, so the answer is a ratio and
/// any size would have given the same room but for those tolerances. Every foreign edge and both
/// sides' self rooms are read against the chain's own field, and growing crease runs read their
/// pierce room from `unit_setback`'s angle centre. The optional arguments are passed explicitly
/// (`seam_epsilon` 0, `profile` Fillet, `fillet_segments` 0, no convex end faces).
#[allow(clippy::too_many_arguments)]
pub fn room_of_cut_mesh(
    shared_mesh: &Rc<RefCell<LevelLineMesh>>,
    chain_edges: &[(i32, i32)],
    field_at: FieldAt,
    unit_setback: &[f64],
    chain_is_closed: bool,
    snap: f64,
    seam_epsilon: f64,
    profile: BandProfile,
    fillet_segments: i32,
    convex_end_faces: Option<&[PointPlane]>,
    field_without_convex_rays: Option<FieldAt>,
) -> Result<(f64, RoomSource), WorkError> {
    let field = Rc::new(field_at(REFERENCE_SIZE));
    // The mesh is shared (`Rc<RefCell>`) because the growing runs the foreign reading hands back
    // keep it, live; nothing here edits it.

    let mut growing: Vec<GrowingCreaseRuns> = Vec::new();
    let mut foreign: Vec<ForeignEdge> = foreign_edge_room::foreign_edges(
        shared_mesh,
        chain_edges,
        &field,
        chain_is_closed,
        seam_epsilon,
        Some(&mut growing),
    )?
    .into_iter()
    .map(|e| (e.0, e.1, 0.0))
    .collect();
    let mesh_ref = shared_mesh.borrow();
    let mesh: &LevelLineMesh = &mesh_ref;

    // A convex end's ray keeps its full reach for the build, but the convex band is clipped at the
    // end face, so nothing wholly beyond that face's plane can meet the band through the ray. Such
    // an edge is read against the chain without that ray - its end vertex - not against the ray's
    // line. Read against the ray, a crease lying on the chain's line past an oblique end ("Chain
    // and Face Should Both work.mcx" c20: the top/end-wall crease 4.9 mm on past a 45-degree end
    // face) reads room 0 and the pick is refused at every size.
    let mut past_an_end_face: Vec<ForeignEdge> = Vec::new();
    if let (Some(faces), Some(_)) = (convex_end_faces, field_without_convex_rays) {
        if !faces.is_empty() {
            past_an_end_face = foreign
                .iter()
                .copied()
                .filter(|e| is_past_an_end_face(mesh, (e.0, e.1), faces, snap))
                .collect();
            foreign = except(&foreign, &past_an_end_face);
        }
    }

    // A growing neighbour's far creases are read apart from the rest, only where there are any, so
    // the user is told the room was set by a crease away from the face and not by the face's own
    // width. The room is the least over edges, so reading them in two calls leaves it the same
    // number.
    let far_bounds: Vec<ForeignEdge> = if growing.is_empty() {
        Vec::new()
    } else {
        foreign
            .iter()
            .copied()
            .filter(|e| growing.iter().any(|g| g.is_a_far_bound((e.0, e.1))))
            .collect()
    };
    if !far_bounds.is_empty() {
        foreign = except(&foreign, &far_bounds);
    }

    let near_room = ieee_math::min_f64(
        room_for(mesh, &foreign, field_at, unit_setback, REFERENCE_SIZE)?,
        room_for(
            mesh,
            &past_an_end_face,
            field_without_convex_rays.unwrap_or(field_at),
            unit_setback,
            REFERENCE_SIZE,
        )?,
    );
    let far_room = room_for(mesh, &far_bounds, field_at, unit_setback, REFERENCE_SIZE)?;
    let foreign_room = ieee_math::min_f64(near_room, far_room);
    let mut room = GrowingCreaseRuns::least(
        foreign_room,
        &growing,
        unit_setback,
        profile,
        fillet_segments,
    );
    // The crease is read only where the far bound alone sets the room; reading it can spend work,
    // so it is read here, under that condition, before the source is named.
    let crease_at = if room != f64::INFINITY && far_room < near_room && room == far_room {
        where_the_room_is_set(mesh, &far_bounds, field_at, unit_setback, far_room)?
    } else {
        Vector3::ZERO
    };
    let mut set_by = the_foreign_source(near_room, far_room, room, &|| crease_at);

    // The SELF-room: a side can run out of itself without meeting anything foreign - a face rim's
    // band closes on its own middle and takes the whole face. The greatest field distance standing
    // inside a side, scaled to a size, is that side's largest buildable size, and the smaller of
    // the two rooms is the room.
    let room_field = RoomField::from_chain(field);
    let mut samples: Vec<Option<FieldSample>> = vec![None; mesh.vertex_count() as usize];
    let pick_ceiling = self_room::self_room_ceiling(mesh, chain_edges);
    for side in [1, 2] {
        // An OPEN chain's band is not bounded by the pick: a fade runs a size past each end and
        // round the crease, so a one-facet pick can swallow a side a dozen times its own diameter -
        // which the pick's ceiling reports UNBOUNDED, and the room then over-reads to the next
        // foreign edge. The side's own reach is read instead, which is a ceiling the side really
        // has.
        let ceiling = if chain_is_closed {
            pick_ceiling
        } else {
            self_room::reach_of_side(mesh, &room_field, side, REFERENCE_SIZE, &mut samples)?
        };

        // The room already found is itself a ceiling: a side whose own room stands above it cannot
        // change the answer, so there is nothing to read exactly beyond that point.
        let mut of_side = self_room::self_room_of_side(
            mesh,
            &room_field,
            side,
            REFERENCE_SIZE,
            &mut samples,
            ieee_math::min_f64(ceiling, room),
        )?;

        // A side the round SWALLOWS past its self room (its far corner on the crease two growing
        // walls share) does not stop there: the walls clip the whole strip, until the ties on that
        // crease merge.
        if !chain_is_closed && of_side < room && of_side.is_finite() {
            if let Some(merge) =
                swallowed_side_runs::ties_merge_room(mesh, field_at, side, profile, of_side)?
            {
                of_side = merge;
            }
        }

        if of_side < room {
            room = of_side;
            set_by = RoomSource::self_room(0);
        }
    }

    // The geometric room is the size at which the band exactly reaches what bounds it, and AT
    // that size there is nothing left to mesh: the kept strip has zero width, the side leaves no
    // loop and the fill refuses SideDoesNotLeaveOneLoop. That reads to a user as "no, but yes" - a
    // refusal naming a room that itself does not build. So the room reported IS the usable room:
    // the geometric room less the width the kept strip must stand to exist at all.
    let largest = if unit_setback.is_empty() {
        0.0
    } else {
        unit_setback
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, ieee_math::max_f64)
    };
    Ok((
        steady_down(room - band_gap_reading::room_margin(largest, snap)),
        set_by,
    ))
}

/// The distinct elements of `first` not in `second`, in order.
fn except(first: &[ForeignEdge], second: &[ForeignEdge]) -> Vec<ForeignEdge> {
    let mut kept: Vec<ForeignEdge> = Vec::new();
    for &e in first {
        if !second.contains(&e) && !kept.contains(&e) {
            kept.push(e);
        }
    }

    kept
}

/// The room rounded DOWN onto a grid of [`ROOM_DIGITS`] significant
/// digits, which is what makes the room a number a user can hand straight back.
///
/// A room is a division and a subtraction over floating point, so two readings of the same
/// geometry can differ in their last bits - the strip's face read its half width as
/// 2.0000000000000004 one way and 2.0 the other, and the lower of the two refused the very size the
/// higher had named. Rounding down lands both on one number, and down rather than near, so the
/// rounding can only ever offer LESS than the geometry has.
pub fn steady_down(room: f64) -> f64 {
    if !room.is_finite() {
        // Infinity is a room - nothing bounds the sides - and NaN is a reading that ran out of
        // work, which no size compares above. Neither is a number to clamp.
        return room;
    }

    if room <= 0.0 {
        // A room is a SIZE, so it never reads below zero. Every reading here has had the kept
        // strip's margin taken off it, and where the geometry holds less room than that margin the
        // subtraction goes past zero - a face pick in the app read -1.67e-9 and handed that back as
        // the size a drag was allowed. Zero says exactly what a negative says - nothing builds - in
        // a number a user can be shown.
        return 0.0;
    }

    on_grid(room, ROOM_DIGITS)
}

/// A positive number rounded DOWN onto a grid of the given significant
/// digits.
pub fn on_grid(value: f64, digits: i32) -> f64 {
    if !value.is_finite() || value <= 0.0 {
        return value;
    }

    let scale = 10f64.powf(f64::from(digits - 1) - value.log10().floor());
    (value * scale).floor() / scale
}
