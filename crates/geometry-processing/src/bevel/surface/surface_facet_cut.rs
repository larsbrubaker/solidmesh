//! One tangent line read as the half space a face region is kept in ([`SurfaceFacetCut`]): the
//! fields, the three constructors ([`SurfaceFacetCut::new`], [`SurfaceFacetCut::with_own_ends`] and
//! [`SurfaceFacetCut::with_ends`]) and the clipping and removal readings.

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::{ieee_math, Vector3};

use super::{surface_cut_clip, surface_ring, SurfaceBandEnd, SurfaceLine, SurfaceVertexPool};

/// One tangent line read as the half space a face region
/// is kept in - the plane that stands square to the region through the tangent line, with the
/// material side named.
///
/// A PLANE rather than a 2D line on a projected polygon, and that is the phase's one real decision.
/// The plan says a face is walked back TO a line lying in it; cutting the face's own triangles by
/// the plane through that line gives the same answer where the region is planar - the plane meets
/// the face plane in exactly the tangent line - and needs no frame, no projection and no mapping
/// back, so the vertices it makes are on the tangent line to the last bit rather than to the
/// accuracy of a round trip. Where a "planar" region is slightly warped (the fixture
/// `WarpedWallTestMeshes`) it degrades the way one would want: each facet is cut in its own plane
/// and the cut stays ON the surface, where a mean-plane projection would land the line off the
/// facets. What it does NOT do is bend round a curved region, which is phase 5's business.
///
/// The half space is infinite, which is what makes an open chain's tangent line run "from rim to
/// rim": one picked edge of a cube top walks the WHOLE top face back by the setback, not just the
/// part over the edge. What bounds it is not the plane but the MATERIAL: the walkback applies a cut
/// only to the facets its own edge's band actually reaches across the region
/// ([`SurfaceFacetCut::seed_face`]), so a notch bitten out of the region's side stops the cut dead
/// rather than letting it take a second band off the far side of the notch - a band no strip is
/// going to fill, and so a hole.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceFacetCut {
    /// The planes that bound the band along the edge, none of them on a chain that runs out to the
    /// region's own rim at both ends ([`SurfaceBandEnd`]).
    pub ends: Vec<SurfaceBandEnd>,
    /// Which PLAN this cut was made for, as its place in the plans handed to the walkback, and -1
    /// for a cut with no chain behind it (`SurfaceEndCut`).
    ///
    /// One region holds the cuts of every plan that reaches it, and the standing-feature guard's
    /// chain exemption is a thing one plan may claim only of its OWN chain
    /// (`surface_walkback_bands::try_check_nothing_is_standing_in_the_band`): a boss's foot being
    /// bevelled excuses the boss's own band, not a rim edge's band that happens to run into the
    /// same foot. [`SurfaceFacetCut::edge_index`] cannot answer it, since that numbers the plans'
    /// edges end to end and says nothing about where one plan stops.
    pub plan_index: i32,
    /// Which edge of the plan this cut belongs to, as an index into its edges.
    pub edge_index: i32,
    /// Whether it is the left face of the walk that is being cut.
    pub left: bool,
    /// The line in the face the walkback cuts to.
    pub tangent_line: SurfaceLine,
    /// The unit direction, lying in the face and square to the chain edge, that points away from
    /// the edge into the face - the side of the cut the region is KEPT on.
    pub inward: Vector3,
    /// The region's own facet that the chain edge runs along - where the band this cut takes off
    /// starts. The walkback spreads the cut out from here through the material it can reach and no
    /// further, which is what keeps one edge's band from reappearing across a notch.
    pub seed_face: i32,
    /// The only facets of the region this cut may take material off, or `None` to let the band
    /// spread through whatever material it can reach - which is what every cut between two planes
    /// does. Shared with the edge plan's band it was read from, so held as `Rc`.
    ///
    /// A cut is written as a half space and nothing else, and that is right wherever the region is
    /// flat: the band is then bounded by the MATERIAL it can reach. On a wall rolled along, it is
    /// not - the half space through the foot with the touched facet's inward direction also
    /// swallows the facets coming back round the far side of a wall that turns more than half a
    /// circle, and those are joined to the band by material all the way round. So the roll names
    /// the facets its own sections paced out, and the band's flood is kept inside them
    /// (`surface_walkback_bands::limit_each_cut_to_the_band_its_own_edge_reaches`).
    pub may_touch: Option<Rc<HashSet<i32>>>,
    /// The two vertices the chord this cut is drawn along runs between, where it was drawn along
    /// one the construction had already stood, or `None`. A ring corner that IS one of them stands
    /// on the cut by identity (`surface_cut_clip::clip_by`).
    pub the_cuts_own_ends: Option<Vec<i32>>,
    /// Where the chain edge this cut belongs to begins, in the mesh's own space.
    pub chain_from: Vector3,
    /// Where it ends. The same point as [`SurfaceFacetCut::chain_from`] for a cut that has no chain
    /// edge behind it, which is the end cut's own (`SurfaceEndCut`).
    pub chain_to: Vector3,
}

impl SurfaceFacetCut {
    /// The cut of one side (`left`) of chain edge `edge_index`: the half space on the `inward`
    /// side of `tangent_line`, flooded from `seed_face`, with no band ends and no chain span.
    pub fn new(
        edge_index: i32,
        left: bool,
        tangent_line: SurfaceLine,
        inward: Vector3,
        seed_face: i32,
    ) -> Self {
        Self::with_ends(
            edge_index,
            left,
            tangent_line,
            inward,
            seed_face,
            Vec::new(),
            tangent_line.point,
            tangent_line.point,
            -1,
            None,
        )
    }

    /// A cut belonging to no edge, whose clip snaps onto `the_cuts_own_ends`.
    pub fn with_own_ends(
        tangent_line: SurfaceLine,
        inward: Vector3,
        the_cuts_own_ends: Vec<i32>,
    ) -> Self {
        SurfaceFacetCut {
            the_cuts_own_ends: Some(the_cuts_own_ends),
            ..Self::with_ends(
                -1,
                false,
                tangent_line,
                inward,
                -1,
                Vec::new(),
                tangent_line.point,
                tangent_line.point,
                -1,
                None,
            )
        }
    }

    /// A cut with everything named: the band `ends` the cut stops at, the chain edge's span
    /// (`chain_from`..`chain_to`), the plan it belongs to and the facets it may touch (`None`
    /// for any).
    #[allow(clippy::too_many_arguments)]
    pub fn with_ends(
        edge_index: i32,
        left: bool,
        tangent_line: SurfaceLine,
        inward: Vector3,
        seed_face: i32,
        ends: Vec<SurfaceBandEnd>,
        chain_from: Vector3,
        chain_to: Vector3,
        plan_index: i32,
        may_touch: Option<Rc<HashSet<i32>>>,
    ) -> Self {
        SurfaceFacetCut {
            ends,
            plan_index,
            edge_index,
            left,
            tangent_line,
            inward,
            seed_face,
            may_touch,
            the_cuts_own_ends: None,
            chain_from,
            chain_to,
        }
    }

    /// The same cut - its band ends, chain span, plan and facets - drawn along another `rail`
    /// facing `inward`; the clip's own snap ends are not carried over to the new rail.
    pub fn drawn_along(&self, rail: SurfaceLine, inward: Vector3) -> SurfaceFacetCut {
        Self::with_ends(
            self.edge_index,
            self.left,
            rail,
            inward,
            self.seed_face,
            self.ends.clone(),
            self.chain_from,
            self.chain_to,
            self.plan_index,
            self.may_touch.clone(),
        )
    }

    /// How far `point` stands on the kept side of the tangent line.
    pub fn signed_distance(&self, point: Vector3) -> f64 {
        (point - self.tangent_line.point).dot(self.inward)
    }

    /// `polygon` clipped to the kept side.
    pub fn clip(&self, polygon: &[i32], pool: &mut SurfaceVertexPool) -> Vec<i32> {
        surface_cut_clip::clip_by(
            polygon,
            pool,
            self.tangent_line.point,
            self.inward,
            None,
            self.the_cuts_own_ends.as_deref(),
        )
    }

    /// Whether the cut removes `point`: inside every band end (to `on_the_cut`) and past the
    /// tangent line.
    pub fn removes(&self, point: Vector3, on_the_cut: f64) -> bool {
        for end in &self.ends {
            if end.signed_distance(point) < -on_the_cut {
                return false;
            }
        }

        self.signed_distance(point) < -on_the_cut
    }

    /// The plane through the chain edge's start the band stands behind - its own BACK - or `None`
    /// where the cut has no chain span.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
    pub fn try_the_bands_own_back(&self) -> Option<SurfaceBandEnd> {
        if !((self.chain_to - self.chain_from).length_squared() > 0.0) {
            return None;
        }

        Some(SurfaceBandEnd::new(self.chain_from, self.inward))
    }

    /// Whether the cut removes `point` from its own band: removed, and not behind the band's
    /// back.
    pub fn removes_from_the_band(&self, point: Vector3, on_the_cut: f64) -> bool {
        self.removes(point, on_the_cut)
            && self
                .try_the_bands_own_back()
                .is_none_or(|back| back.signed_distance(point) >= -on_the_cut)
    }

    /// The point of `from`..`to` the cut removes most deeply, within the band's own back as well
    /// where it has one; `None` where it removes none of it.
    pub fn try_find_removed_point_in_the_band(
        &self,
        from: Vector3,
        to: Vector3,
        on_the_cut: f64,
    ) -> Option<Vector3> {
        match self.try_the_bands_own_back() {
            Some(back) => self.try_find_removed_point_also_inside(from, to, on_the_cut, Some(back)),
            None => self.try_find_removed_point(from, to, on_the_cut),
        }
    }

    /// Whether `point` stands within `near` of the chain edge's own span.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
    pub fn stands_on_the_chain_edge(&self, point: Vector3, near: f64) -> bool {
        let run = self.chain_to - self.chain_from;

        if !(run.length_squared() > 0.0) {
            return false;
        }

        let where_ = ieee_math::min_f64(
            ieee_math::max_f64(
                (point - self.chain_from).dot(run) / run.length_squared(),
                0.0,
            ),
            1.0,
        );

        (point - (self.chain_from + (run * where_))).length() <= near
    }

    /// Whether the cut removes any part of `from`..`to`.
    pub fn removes_part_of(&self, from: Vector3, to: Vector3, on_the_cut: f64) -> bool {
        self.try_find_removed_point(from, to, on_the_cut).is_some()
    }

    /// The point of `from`..`to` the cut removes most deeply (within every band end), or `None`
    /// where it removes none of it.
    pub fn try_find_removed_point(
        &self,
        from: Vector3,
        to: Vector3,
        on_the_cut: f64,
    ) -> Option<Vector3> {
        self.try_find_removed_point_also_inside(from, to, on_the_cut, None)
    }

    /// [`SurfaceFacetCut::try_find_removed_point`], kept inside `also_inside` as well.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` refuses a NaN as well.
    pub fn try_find_removed_point_also_inside(
        &self,
        from: Vector3,
        to: Vector3,
        on_the_cut: f64,
        also_inside: Option<SurfaceBandEnd>,
    ) -> Option<Vector3> {
        let mut start = 0.0;
        let mut finish = 1.0;

        for &end in &self.ends {
            if !Self::inside(end, from, to, on_the_cut, &mut start, &mut finish) {
                return None;
            }
        }

        if let Some(also_inside) = also_inside {
            if !Self::inside(also_inside, from, to, on_the_cut, &mut start, &mut finish) {
                return None;
            }
        }

        if !(finish > start) {
            return None;
        }

        let removed_at_from = self.signed_distance(from);
        let removed_at_to = self.signed_distance(to);

        let at_start = removed_at_from + ((removed_at_to - removed_at_from) * start);
        let at_finish = removed_at_from + ((removed_at_to - removed_at_from) * finish);

        if ieee_math::min_f64(at_start, at_finish) >= -on_the_cut {
            return None;
        }

        Some(from + ((to - from) * (if at_start <= at_finish { start } else { finish })))
    }

    /// Narrows `start..finish` along `from`..`to` to the part inside `end` (to `on_the_cut`);
    /// false where none of it is.
    pub fn inside(
        end: SurfaceBandEnd,
        from: Vector3,
        to: Vector3,
        on_the_cut: f64,
        start: &mut f64,
        finish: &mut f64,
    ) -> bool {
        let at_from = end.signed_distance(from) - on_the_cut;
        let at_to = end.signed_distance(to) - on_the_cut;

        if at_from < 0.0 && at_to < 0.0 {
            return false;
        }

        if at_from < 0.0 || at_to < 0.0 {
            let crossing = at_from / (at_from - at_to);

            if at_from < 0.0 {
                *start = ieee_math::max_f64(*start, crossing);
            } else {
                *finish = ieee_math::min_f64(*finish, crossing);
            }
        }

        true
    }

    /// Whether a SIDE of `ring` lies in `end`'s plane (both its corners on it, to the pool's
    /// ruler) without the ring standing clear of the plane on both sides by more than that side is
    /// long - a convex ring with a side in a plane has nothing on the other side of it to cut off.
    pub fn ring_lies_along(end: SurfaceBandEnd, ring: &[i32], pool: &SurfaceVertexPool) -> bool {
        let mut least = f64::MAX;
        let mut most = f64::MIN;

        for &corner in ring {
            let at = end.signed_distance(pool.position_of(corner));

            least = ieee_math::min_f64(least, at);
            most = ieee_math::max_f64(most, at);
        }

        let stands_on =
            |corner: i32| end.signed_distance(pool.position_of(corner)).abs() <= pool.near_enough();

        for i in 0..ring.len() {
            let next = (i + 1) % ring.len();

            if !stands_on(ring[i]) || !stands_on(ring[next]) {
                continue;
            }

            let side = (pool.position_of(ring[i]) - pool.position_of(ring[next])).length();

            if least < -side && most > side {
                // Clear of the plane on both sides by more than this "side" is long: the two
                // corners are one place read twice, and the plane really does cut the ring.
                continue;
            }

            return true;
        }

        false
    }

    /// `polygon` cut into the pieces the cut leaves: those beyond each band end, and what stands
    /// between the ends on the kept side of the tangent line - knitted back together where they
    /// share sides. Where `band_corners` is given, every kept vertex standing on a band end (to
    /// the weld) is recorded as `(end, vertex)`.
    ///
    /// A facet wholly beyond an end (to the pool's NEARNESS, so a facet that only grazes the plane
    /// is kept whole) is kept as it stands; this disagrees with the flood's ruler, which
    /// `SurfaceCutRulerTests` holds as a queue entry.
    pub fn clip_to_pieces(
        &self,
        polygon: &[i32],
        pool: &mut SurfaceVertexPool,
        mut band_corners: Option<&mut Vec<(usize, i32)>>,
    ) -> Vec<Vec<i32>> {
        let mut pieces = Vec::with_capacity(self.ends.len() + 1);
        let mut between = polygon.to_vec();

        for &end in &self.ends {
            let mut least = f64::MAX;
            let mut most = f64::MIN;

            for &vertex in &between {
                let at = end.signed_distance(pool.position_of(vertex));

                least = ieee_math::min_f64(least, at);
                most = ieee_math::max_f64(most, at);
            }

            let on_the_cut = surface_cut_clip::on_the_cut(pool);

            if most <= pool.near_enough() {
                // Wholly beyond this end: nothing of it is in the band, so it is kept as it stands.
                pieces.push(between);
                return surface_ring::knit(pieces, pool);
            }

            if least < -on_the_cut {
                if Self::ring_lies_along(end, &between, pool) {
                    // The ring's own SIDE is in this plane, so the ring is on one side of it whole.
                    continue;
                }

                let beyond =
                    surface_cut_clip::clip_by(&between, pool, end.point, -end.inward, None, None);

                if beyond.len() >= 3 {
                    pieces.push(beyond);
                }

                between =
                    surface_cut_clip::clip_by(&between, pool, end.point, end.inward, None, None);

                if between.len() < 3 {
                    return surface_ring::knit(pieces, pool);
                }
            }
        }

        let kept = self.clip(&between, pool);

        if kept.len() >= 3 {
            if let Some(band_corners) = band_corners.as_mut() {
                // Read on the WELD and not on the pool's nearness: a corner is the two planes' own
                // meeting point and stands on both exactly.
                for &vertex in &kept {
                    let at = pool.position_of(vertex);

                    for (end, band_end) in self.ends.iter().enumerate() {
                        if band_end.signed_distance(at).abs() <= pool.weld_distance() {
                            band_corners.push((end, vertex));
                        }
                    }
                }
            }

            pieces.push(kept);
        }

        surface_ring::knit(pieces, pool)
    }
}
