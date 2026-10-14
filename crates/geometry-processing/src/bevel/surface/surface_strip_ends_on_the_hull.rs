//! Where a strip leaves the PART at one of its ends ([`SurfaceStripEndsOnTheHull`]): which of the
//! part's own hull planes ([`SurfaceStripEndsOnTheHullSupportingPlane`]) each rail goes out
//! through, at what parameter along the edge, and the one point the end turns at where the rails
//! change from one plane to the other. Here are its fields, its constructor, the part's supporting
//! planes ([`SurfaceStripEndsOnTheHull::supporting_planes_of`]) and the readings the end is built
//! from (the turning point between two planes, the ties settled, a face on a plane, the apex on a
//! knife edge); the reading ([`SurfaceStripEndsOnTheHull::try_read`]) and the closing
//! ([`SurfaceStripEndsOnTheHull::try_close`]).

use vector_math::Vector3;

use std::collections::HashSet;

use polygon_mesh::Mesh;

use super::super::{BevelAdvisory, BevelAdvisoryCode};
use super::surface_mitre_joint_step::SideSeams;
use super::{
    surface_end_gives_up_the_section, surface_end_notch, surface_mesh_closure, surface_strip_ends,
    SurfaceBandRunsOffOverAStretch, SurfaceEdgePlan, SurfaceLine, SurfaceStrip,
    SurfaceStripStation, SurfaceVertexPool,
};

/// Where a strip leaves the part at one of its ends.
///
/// A strip whose band ran off the face it landed on (`SurfaceBandRunsOffOverAStretch`) has no face
/// left to end on at that end of the edge: the bead carries on past the last of the wall and out of
/// the solid altogether. What ends it there is the part's own OUTSIDE, so the end is read against
/// the planes the part is bounded by rather than against any face the band lands on.
///
/// **The furthest cut back wins.** Walking toward the end, a rail may cross several of those
/// planes; the one that really ends it is the one that cuts it back FURTHEST from the end, because
/// past that crossing the rail is outside the part whatever the others say. Where two planes cut
/// one rail back by the same amount the tie is settled by which plane its NEIGHBOURS leave through,
/// because a run of rails leaving through one plane is what the end cap is cut from.
///
/// **Two planes at most, in two runs.** The cap this end leaves is one piece per plane, so an end
/// over three planes, or one whose runs interleave, is named rather than built
/// (`SurfaceHullEndOnThreePlanes`, `SurfaceHullEndCrossesTheCorner`). Where there ARE two
/// contiguous runs, the end turns at one point on the CHORD between the two rails either side of
/// the change, because the band is chorded and the cap has to be bounded by the surface that is
/// really built.
/// **And the apex is looked up** in the pool rather than minted, so that the cap, the faces beside
/// it and the part's own triangles all name one vertex.
///
/// **A rung, not a bug.** The hull is read off the mesh the WALKBACK left, so a plane that was not
/// supporting on the part the user picked can become supporting once a band has been cut away from
/// in front of it. Neither fixture reaches it, and the honest reading is of the ORIGINAL mesh,
/// handed down from the caller. Named here so the next rung has it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStripEndsOnTheHull {
    /// Which end of the edge was read.
    pub at_the_start: bool,
    /// Where each rail leaves the part, as a parameter along the edge from its start.
    pub exit_at: Vec<f64>,
    /// The plane each rail leaves through.
    pub plane_of: Vec<SurfaceStripEndsOnTheHullSupportingPlane>,
    /// The rail the end changes plane after - so rails up to and including it leave through one
    /// plane and the rest through the other - or -1 where every rail leaves through the same one.
    pub changes_after: i32,
    /// Where the end turns from the one plane to the other, on the chord between the two rails
    /// either side of [`SurfaceStripEndsOnTheHull::changes_after`]. Meaningless where that is -1.
    pub between: Vector3,
    /// The part's own vertex where the two planes meet on the knife edge the band ran off, or -1
    /// where no knife edge was handed over to look for one on.
    pub apex: i32,
}

/// One plane the part is
/// bounded by, with every one of its vertices on the inner side - the six faces of the cube the
/// recess is cut into, and nothing of the recess itself. That is the definition a hull has, read on
/// the mesh rather than built as one, and it is what makes "where does this rail leave the part" a
/// question with a closed-form answer - a rail is a line, so each crossing is one division. The
/// zero value (no normal) is the plane that does not exist.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceStripEndsOnTheHullSupportingPlane {
    /// A point on the plane.
    pub at: Vector3,
    /// Its outward normal, which is the part's own.
    pub normal: Vector3,
}

/// The end's turning point between two hull planes (`try_between`, `try_three_planes`).
impl SurfaceStripEndsOnTheHull {
    /// Where the end turns from one plane to the other - the line the two planes
    /// meet in, crossed with the CHORD between the two rails either side of the change. `None` where
    /// the two planes do not meet in a line at all, or where they meet the band outside that chord -
    /// which is the corner being crossed somewhere the rails do not rule. The rails' exit parameters
    /// play no part in the answer, so they are not taken.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_between(
        strip: &SurfaceStrip,
        plane_of: &[SurfaceStripEndsOnTheHullSupportingPlane],
        changes_after: i32,
        length: f64,
        near: f64,
    ) -> Option<Vector3> {
        // The band between two rails is a ruled quad and so a PLANE - both rails run along the same
        // direction - and the chord the section is really bounded by is where one hull plane cuts that
        // quad. So where the section turns from the one plane to the other is where all THREE meet:
        // one point, by Cramer, with nothing sampled and nothing searched for.
        let foot = strip.point_on(changes_after, 0.0);
        let across = strip.point_on(changes_after + 1, 0.0) - foot;
        let column = strip.along().cross(across);

        if !(column.length_squared() > 0.0) {
            return None;
        }

        let between = Self::try_three_planes(
            plane_of[changes_after as usize],
            plane_of[changes_after as usize + 1],
            foot,
            column,
        )?;

        // And it has to stand ON the band: between the two rails, and inside the edge's own span. A
        // corner the two planes meet at somewhere else is a corner this end does not turn at.
        let sideways = (between - foot).dot(across) / across.length_squared();
        let along = strip.param_of(between) - strip.param_of(foot);

        (sideways >= -near && sideways <= 1.0 + near && along >= -near && along <= length + near)
            .then_some(between)
    }

    /// Where three planes meet, by Cramer's rule - `None` where they do not meet
    /// at a point.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_three_planes(
        a: SurfaceStripEndsOnTheHullSupportingPlane,
        b: SurfaceStripEndsOnTheHullSupportingPlane,
        point_c: Vector3,
        normal_c: Vector3,
    ) -> Option<Vector3> {
        let volume = a.normal.dot(b.normal.cross(normal_c));

        if !(volume.abs() > 1e-9) {
            return None;
        }

        Some(
            ((b.normal.cross(normal_c) * a.at.dot(a.normal))
                + (normal_c.cross(a.normal) * b.at.dot(b.normal))
                + (a.normal.cross(b.normal) * point_c.dot(normal_c)))
                / volume,
        )
    }
}

impl SurfaceStripEndsOnTheHullSupportingPlane {
    /// The plane through `at` facing `normal`.
    pub fn new(at: Vector3, normal: Vector3) -> Self {
        SurfaceStripEndsOnTheHullSupportingPlane { at, normal }
    }

    /// Whether this is a plane at all (the default value is none).
    pub fn exists(&self) -> bool {
        self.normal.length_squared() > 0.0
    }

    /// How far `point` stands outside the plane.
    pub fn outside(&self, point: Vector3) -> f64 {
        (point - self.at).dot(self.normal)
    }
}

impl SurfaceStripEndsOnTheHull {
    /// An end from its readings, as [`Self::try_read`] makes it.
    pub fn new(
        at_the_start: bool,
        exit_at: Vec<f64>,
        plane_of: Vec<SurfaceStripEndsOnTheHullSupportingPlane>,
        changes_after: i32,
        between: Vector3,
        apex: i32,
    ) -> Self {
        SurfaceStripEndsOnTheHull {
            at_the_start,
            exit_at,
            plane_of,
            changes_after,
            between,
            apex,
        }
    }

    /// The part's supporting planes: every face plane that has the whole part on its inner side
    /// (within `near`), each plane once - keyed on the plane itself, rounded to `near`, so a wall
    /// drawn as a hundred triangles is measured once.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)`, which refuses a NaN as well.
    pub fn supporting_planes_of(
        mesh: &Mesh,
        near: f64,
    ) -> Vec<SurfaceStripEndsOnTheHullSupportingPlane> {
        let mut supporting = Vec::new();
        let mut seen: HashSet<(i64, i64, i64, i64)> = HashSet::new();
        let step = vector_math::cs_math::max_f64(near, 1e-9);
        // Banker's rounding to the nearest step; `as` saturates a reading too large for an `i64`,
        // so even that gets a key instead of an undefined one.
        let key = |reading: f64| (reading / step).round_ties_even() as i64;

        for face in mesh.faces.iter() {
            let normal = Vector3::from_vector3_float(face.normal);

            if !(normal.length_squared() > 0.0) {
                continue;
            }

            let normal = normal.get_normal();

            let at = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);

            if !seen.insert((
                key(normal.x),
                key(normal.y),
                key(normal.z),
                key(at.dot(normal)),
            )) {
                continue;
            }

            let inside = mesh
                .vertices
                .iter()
                .all(|&vertex| !((Vector3::from_vector3_float(vertex) - at).dot(normal) > near));

            if inside {
                supporting.push(SurfaceStripEndsOnTheHullSupportingPlane::new(at, normal));
            }
        }

        supporting
    }

    /// Settles the rails standing on two planes at once (`tied`): each takes the plane its
    /// neighbour took, forward then back.
    pub fn settle_the_ties(
        plane_of: &mut [SurfaceStripEndsOnTheHullSupportingPlane],
        tied: &[Vec<SurfaceStripEndsOnTheHullSupportingPlane>],
    ) {
        for rail in 1..plane_of.len() {
            let with = plane_of[rail - 1];
            Self::agree(plane_of, tied, rail, with);
        }

        for rail in (0..plane_of.len().saturating_sub(1)).rev() {
            let with = plane_of[rail + 1];
            Self::agree(plane_of, tied, rail, with);
        }
    }

    /// Rail `rail`, tied between two planes or more, takes `with` where that is one of them.
    pub fn agree(
        plane_of: &mut [SurfaceStripEndsOnTheHullSupportingPlane],
        tied: &[Vec<SurfaceStripEndsOnTheHullSupportingPlane>],
        rail: usize,
        with: SurfaceStripEndsOnTheHullSupportingPlane,
    ) {
        if tied[rail].len() < 2 {
            return;
        }

        if tied[rail].contains(&with) {
            plane_of[rail] = with;
        }
    }

    /// Whether one of the first `up_to` `faces` uses `vertex`, lies in `plane` (to `near`) and
    /// faces the way it does.
    pub fn theres_a_face_at(
        pool: &SurfaceVertexPool,
        faces: &[(i32, i32, i32)],
        up_to: usize,
        vertex: i32,
        plane: SurfaceStripEndsOnTheHullSupportingPlane,
        near: f64,
    ) -> bool {
        faces.iter().take(up_to).any(|&(v0, v1, v2)| {
            if v0 != vertex && v1 != vertex && v2 != vertex {
                return false;
            }

            let a = pool.position_of(v0);
            let b = pool.position_of(v1);
            let c = pool.position_of(v2);

            if plane.outside(a).abs() > near
                || plane.outside(b).abs() > near
                || plane.outside(c).abs() > near
            {
                return false;
            }

            // And facing the way the plane does: a face lying in it the other way about is
            // material on the far side of it.
            let outward = (b - a).cross(c - a);

            outward.length_squared() > 0.0 && outward.get_normal().dot(plane.normal) > 0.9
        })
    }

    /// Whether the rails go out through more than two distinct planes.
    pub fn three_distinct(plane_of: &[SurfaceStripEndsOnTheHullSupportingPlane]) -> bool {
        let mut distinct: Vec<SurfaceStripEndsOnTheHullSupportingPlane> = Vec::new();

        for &plane in plane_of {
            if !distinct.contains(&plane) {
                distinct.push(plane);
            }
        }

        distinct.len() > 2
    }

    /// The part's own vertex at the corner planes `a` and `b` make on the `knife_line`, where the
    /// knife edge passes through that corner; `None` where it runs along both planes or meets one
    /// off the other, or no vertex stands there.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)`, which refuses a NaN as well.
    pub fn try_the_apex(
        pool: &SurfaceVertexPool,
        a: SurfaceStripEndsOnTheHullSupportingPlane,
        b: SurfaceStripEndsOnTheHullSupportingPlane,
        knife_line: SurfaceLine,
        near: f64,
    ) -> Option<i32> {
        let crosses_a = knife_line.direction.dot(a.normal);
        let crosses_b = knife_line.direction.dot(b.normal);
        let through_a = crosses_a.abs() > crosses_b.abs();
        let crosses = if through_a { crosses_a } else { crosses_b };

        if !(crosses.abs() > 1e-9) {
            // A knife edge running ALONG both planes names no one point on either of them.
            return None;
        }

        let crossing = if through_a {
            knife_line.point + (knife_line.direction * (-a.outside(knife_line.point) / crosses_a))
        } else {
            knife_line.point + (knife_line.direction * (-b.outside(knife_line.point) / crosses_b))
        };

        if a.outside(crossing).abs() > near || b.outside(crossing).abs() > near {
            // The knife edge does not pass through the corner the two planes make.
            return None;
        }

        pool.try_find_near(crossing, pool.snap_near(), None)
    }
}

impl SurfaceStripEndsOnTheHull {
    /// Where each rail of `strip` leaves the part at this end of `edge` - the
    /// supporting plane (of `planes`) its walk goes out through first, at what parameter - with ties
    /// settled, at most two runs of planes, the turning point between them and, on a knife edge
    /// (`knife_line`), the part's vertex both planes meet it at. `Err` names why the end is not one
    /// the hull can close.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)`, which refuses a NaN as well.
    pub fn try_read(
        pool: &SurfaceVertexPool,
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        at_the_start: bool,
        planes: &[SurfaceStripEndsOnTheHullSupportingPlane],
        knife_line: Option<SurfaceLine>,
    ) -> Result<SurfaceStripEndsOnTheHull, BevelAdvisory> {
        let length = (edge.to() - edge.from()).dot(strip.along());
        let near = pool.near_enough();
        let walk = if at_the_start {
            -strip.along()
        } else {
            strip.along()
        };
        let ends_at = if at_the_start { edge.from() } else { edge.to() };

        let rails = strip.rail_count() as usize;
        let mut exit_at = vec![0.0; rails];
        let mut plane_of = vec![SurfaceStripEndsOnTheHullSupportingPlane::default(); rails];
        let mut tied: Vec<Vec<SurfaceStripEndsOnTheHullSupportingPlane>> = vec![Vec::new(); rails];

        for rail in 0..rails {
            let mut cut_back_by = f64::NEG_INFINITY;

            for &plane in planes {
                if !(plane.normal.dot(walk) > 1e-9) {
                    continue;
                }
                // A plane the walk does not go out through, or one the rail runs along.
                let Some(param) = strip.try_cross(rail as i32, plane.at, plane.normal) else {
                    continue;
                };

                let back = if at_the_start { param } else { length - param };
                if back > cut_back_by + near {
                    cut_back_by = back;
                    tied[rail].clear();
                } else if !(back > cut_back_by - near) {
                    continue;
                }

                cut_back_by = vector_math::cs_math::max_f64(cut_back_by, back);
                tied[rail].push(plane);
            }

            if tied[rail].is_empty() || cut_back_by < -near || cut_back_by > length + near {
                // A rail that leaves through nothing, or outside the edge's own span.
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceHullEndNotSupported,
                    vec![strip
                        .point_on(rail as i32, if at_the_start { 0.0 } else { length })
                        .into()],
                ));
            }

            exit_at[rail] = if at_the_start {
                cut_back_by
            } else {
                length - cut_back_by
            };
            plane_of[rail] = tied[rail][0];
        }

        Self::settle_the_ties(&mut plane_of, &tied);

        let mut changes_after: i32 = -1;
        for rail in 1..plane_of.len() {
            if plane_of[rail] == plane_of[rail - 1] {
                continue;
            }

            if changes_after >= 0 {
                // A third run: a third plane, or the first come back again.
                let code = if Self::three_distinct(&plane_of) {
                    BevelAdvisoryCode::SurfaceHullEndOnThreePlanes
                } else {
                    BevelAdvisoryCode::SurfaceHullEndCrossesTheCorner
                };
                return Err(BevelAdvisory::new(
                    code,
                    vec![strip.point_on(rail as i32, exit_at[rail]).into()],
                ));
            }

            changes_after = rail as i32 - 1;
        }

        if Self::three_distinct(&plane_of) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceHullEndOnThreePlanes,
                vec![ends_at.into()],
            ));
        }

        let mut between = Vector3::ZERO;
        if changes_after >= 0 {
            match Self::try_between(strip, &plane_of, changes_after, length, near) {
                Some(at) => between = at,
                None => {
                    let c = changes_after as usize;
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceHullEndCrossesTheCorner,
                        vec![strip.point_on(changes_after, exit_at[c]).into()],
                    ));
                }
            }
        }

        let mut apex = -1;
        if changes_after >= 0 {
            if let Some(knife_line) = knife_line {
                let c = changes_after as usize;
                match Self::try_the_apex(pool, plane_of[c], plane_of[c + 1], knife_line, near) {
                    Some(found) => apex = found,
                    // The two planes meet the knife edge where the part has no vertex: named.
                    None => {
                        return Err(BevelAdvisory::new(
                            BevelAdvisoryCode::SurfaceHullEndNotSupported,
                            vec![ends_at.into()],
                        ));
                    }
                }
            }
        }

        Ok(SurfaceStripEndsOnTheHull::new(
            at_the_start,
            exit_at,
            plane_of,
            changes_after,
            between,
            apex,
        ))
    }
}

impl SurfaceStripEndsOnTheHull {
    /// A band that RAN OFF its wall over `stretch` closed where it leaves the part -
    /// read on the hull ([`Self::try_read`], on the stretch's knife line), the ring standing on the
    /// two seams' end vertices, and the end cap in two pieces either side of the turn between the
    /// planes: the run-off side's fanned from the knife edge's apex, the far side's from the chain's
    /// corner. Returns the station the stitch sews to, or why the end is not one the hull closes.
    /// `seams` is the `[edge, side]` table of seam runs indexed by `edge_index`; a missing or empty
    /// run there is refused as an end not supported rather than a panic.
    #[allow(clippy::too_many_arguments)]
    pub fn try_close(
        pool: &mut SurfaceVertexPool,
        faces: &mut Vec<(i32, i32, i32)>,
        faces_before_the_ends: usize,
        strip: &SurfaceStrip,
        edge: &SurfaceEdgePlan,
        seams: &[SideSeams],
        edge_index: i32,
        corner: Vector3,
        at_the_start: bool,
        stretch: &SurfaceBandRunsOffOverAStretch,
        planes: &[SurfaceStripEndsOnTheHullSupportingPlane],
    ) -> Result<SurfaceStripStation, BevelAdvisory> {
        let not_supported = || {
            BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceHullEndNotSupported,
                vec![corner.into()],
            )
        };
        let ends = Self::try_read(
            pool,
            strip,
            edge,
            at_the_start,
            planes,
            Some(SurfaceLine::new(
                stretch.knife_line.start,
                stretch.knife_line.direction,
            )),
        )?;

        if ends.changes_after < 0 || ends.apex < 0 {
            // One plane for every rail: no turn in the end and no corner of the part to die on.
            return Err(not_supported());
        }

        let rails = strip.rail_count();
        let count = rails as usize;
        let mut ring = vec![0; count];
        let mut meets_at = vec![0.0; count];

        for rail in 0..count {
            let at = strip.point_on(rail as i32, ends.exit_at[rail]);
            meets_at[rail] = ends.exit_at[rail];

            if rail == 0 || rail == count - 1 {
                // The walkback has already left a vertex where this rail's tangent line dies, and the
                // strip ends on THAT vertex.
                let side = if rail == 0 { 0 } else { 1 };
                let vertex = seams
                    .get(edge_index as usize)
                    .and_then(|sides| sides[side].as_ref())
                    .and_then(|run| {
                        if at_the_start {
                            run.first()
                        } else {
                            run.last()
                        }
                    })
                    .map(|&(vertex, _)| vertex);
                let Some(vertex) = vertex else {
                    return Err(not_supported());
                };

                if (pool.position_of(vertex) - at).length()
                    > surface_strip_ends::meets_the_seam(pool)
                {
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                        vec![at.into()],
                    ));
                }

                ring[rail] = vertex;
                continue;
            }

            ring[rail] = pool.add(at);
        }

        let mut between = vec![Vec::new(); (rails - 1).max(0) as usize];

        // Where the end turns from the one plane to the other: minted once, shared by both pieces of
        // the cap and the stitch.
        let between_them = pool.add(ends.between);
        between[ends.changes_after as usize] = vec![between_them];

        let apex = pool.position_of(ends.apex);
        let into_the_chain = if at_the_start {
            edge.edge.direction()
        } else {
            -edge.edge.direction()
        };

        // Two pieces, each walked from its own side of the strip INWARD to the turn between the
        // planes: the run-off side's fanned from the knife edge's apex, the far side's from the
        // chain's corner.
        let run_off_rail = if stretch.side == 0 { 0 } else { rails - 1 };
        let far_rail = if stretch.side == 0 { rails - 1 } else { 0 };
        let mut run_off_piece = Vec::new();
        let mut far_piece = Vec::new();

        for step in 0..rails {
            for from in [run_off_rail, far_rail] {
                let rail = if from == 0 { step } else { rails - 1 - step };
                let on_the_run_offs_side = (rail <= ends.changes_after) == (run_off_rail == 0);
                if on_the_run_offs_side == (from == run_off_rail) {
                    let piece = if from == run_off_rail {
                        &mut run_off_piece
                    } else {
                        &mut far_piece
                    };
                    piece.push(ring[rail as usize]);
                }
            }
        }

        run_off_piece.push(between_them);
        far_piece.push(between_them);

        // And the far piece closes on the apex as well, along the rim the two planes meet in.
        far_piece.push(ends.apex);

        if ends.plane_of[far_rail as usize].outside(corner).abs() > pool.near_enough() {
            // The chain's corner does not stand IN the far piece's plane, so it is no corner of it.
            return Err(not_supported());
        }

        // Both pieces are assembled before either is laid, so the corner is pooled here, before the
        // run-off piece's cap: the pool's order is the order of the mesh's vertices.
        let corner_vertex = pool.add(corner);
        let pieces = [
            (
                run_off_piece,
                ends.plane_of[run_off_rail as usize],
                apex,
                ends.apex,
            ),
            (
                far_piece,
                ends.plane_of[far_rail as usize],
                corner,
                corner_vertex,
            ),
        ];

        for (profile, plane, at, vertex) in pieces {
            // Asked at the vertex the piece is fanned FROM, of the faces the part had before any end
            // of this pass was closed.
            if Self::theres_a_face_at(
                pool,
                faces,
                faces_before_the_ends,
                vertex,
                plane,
                pool.near_enough(),
            ) && surface_end_gives_up_the_section::the_face_is_behind_it(
                into_the_chain,
                plane.normal,
            ) {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceHullEndGivesUp,
                    vec![corner.into()],
                ));
            }

            let before = faces.len();
            surface_end_notch::try_cap_with_the_section(pool, faces, &profile, at, plane.normal)?;

            // And the piece really is ONE cover of its own section: named, never picked apart.
            let (folds, folded) = surface_mesh_closure::folded_edge_count(&faces[before..]);
            if folds > 0 {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceHullEndNotSupported,
                    vec![((pool.position_of(folded.0) + pool.position_of(folded.1)) / 2.0).into()],
                ));
            }
        }

        Ok(SurfaceStripStation::new(ring, meets_at, between))
    }
}
