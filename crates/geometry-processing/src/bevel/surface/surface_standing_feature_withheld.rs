//! One standing feature the walkback held BACK rather than carrying over
//! ([`SurfaceStandingFeatureWithheld`]) and its withheld faces
//! ([`SurfaceStandingFeatureWithheldFace`]): its fields and its ridge, crease and wedge readings.
//!
//! The walkback builds these and then keeps filling them in through shared handles - the merge of
//! two bands that reached one feature appends to its reaches and its faces' creases, and the
//! rewrite gathers each face's rings facet by facet through a by-facet index before taking its
//! outline - while a notch names the faces that cut its rails. So both are held as `Rc`, and every
//! part mutated after construction is interior-mutable.

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::{SurfaceStrip, SurfaceVertexPool};

/// One standing feature the walkback held BACK rather than carrying over - its own FACES (whole
/// coplanar regions, not single triangles), the creases they lost their footing on, the plane each
/// stands in, and which band undercut them.
///
/// Held back because those faces are not finished. The band took the material under their creases,
/// so each of them has to be EXTENDED through the removed corner until it meets the fillet cylinder
/// (`SurfaceStandingFeatureFaces`) - Lars's ruling, and what the Onshape control builds. Carrying
/// them over as they are and capping the gap flat, which is what the earlier island did, puts a
/// floor where the control has a wall.
///
/// The vertex indices here are the pool's while the walkback is building and the MESH's afterwards:
/// `surface_walkback_rewrite::assemble` renumbers them along with the tangent runs, since a region
/// held back contributes no face and so has no vertex of its own in the mesh yet.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStandingFeatureWithheld {
    /// Which chain edge's band, and which side of it (`(edge_index, left)`), undercut this
    /// feature - one entry per band that did, in the order they were merged.
    ///
    /// A LIST because one feature can stand under two bands of two different chains, which is the
    /// nose of the nose-in-notch part with both knife tips beveled: its ridge ends stand 7.071
    /// along each slope from that slope's tip edge, and a bead of 3.4 reaches 8.208. Each band
    /// uncovers its own end of the feature, and each face of the feature therefore loses its
    /// footing on TWO runs of crease, one per reach - so
    /// [`SurfaceStandingFeatureWithheldFace::creases`] and
    /// [`SurfaceStandingFeatureWithheldFace::withins`] are indexed by the same number this list is
    /// (`SurfaceFaceWalkback` merges them, and refuses where the two readings do not name the same
    /// faces).
    ///
    /// A feature under two bands of ONE region is a different shape and is still refused before it
    /// gets here: there the two bands uncover overlapping parts of one root, and closing them one
    /// at a time would lay the same triangle down twice
    /// (`SurfaceStandingFeatureStretch::try_read`).
    pub reaches: RefCell<Vec<(i32, bool)>>,
    /// The landing region's outward normal, which the feature stands out of - the FIRST reach's,
    /// where two were merged, since the two bands land on two different regions. Nothing reads it
    /// today.
    pub landing: Vector3,
    /// The withheld faces, one per coplanar region of the feature the band undercut.
    pub faces: Vec<Rc<SurfaceStandingFeatureWithheldFace>>,
}

/// One withheld FACE of the feature - the whole coplanar region, not one triangle of it: the mesh
/// facets it is cut into, its outline, the crease run it stands on, and the plane it lies in.
///
/// The region and not the facet, because the extension is a property of the FACE. A face fanned
/// from a point inside itself - what a boolean leaves - has only one of its triangles across the
/// crease, and withholding that one alone leaves its two coplanar neighbours standing on a ridge
/// end whose material has gone. The whole region goes out as one ring, which is also the only way
/// the old triangulation's diagonals stop being edges of the result.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStandingFeatureWithheldFace {
    /// The region's facets in the source mesh, which name it while the walkback runs.
    pub mesh_faces: Vec<i32>,
    /// The ring of each of those facets as the walkback read it, carrying any crossing put on its
    /// edges - gathered here only so that [`SurfaceStandingFeatureWithheldFace::ring`] can be taken
    /// from them.
    pub rings: RefCell<Vec<Vec<i32>>>,
    /// The region's own outline, which is the boundary of the union of its facets; `None` until the
    /// walkback has taken it.
    pub ring: RefCell<Option<Vec<i32>>>,
    /// The ENDS of the crease run this face lost its footing on (`(from, to)`) - one on the
    /// feature's ridge, one where the run dies - one entry per reach, indexed as
    /// [`SurfaceStandingFeatureWithheld::reaches`] is.
    pub creases: RefCell<Vec<(i32, i32)>>,
    /// The points of each crease run BETWEEN those two ends, which a face whose root line is cut
    /// into several mesh edges has. They go the way the crease does: interior to the extension.
    pub withins: RefCell<Vec<Vec<i32>>>,
    /// A point on the face's own plane.
    pub plane_point: Vector3,
    /// The face's outward normal, which the extension is fanned to face along.
    pub plane_normal: Vector3,
}

/// How far from parallel two of the feature's planes have to be to meet along a ridge at all, as
/// the sine of the angle between them - about a hundredth of a degree.
const THEY_MEET: f64 = 1e-4;

/// The feature's ridge, crease and wedge readings.
impl SurfaceStandingFeatureWithheld {
    /// The line the feature's two face planes meet along, as a point on it and its unit direction -
    /// the ridge one face runs on past the other.
    ///
    /// Read off the two PLANES and not off the mesh. The ridge is where the two faces meet, and
    /// that is a property of the faces, not of how they happen to be cut into triangles: a face
    /// fanned from a point inside itself - which is what a BOOLEAN leaves, and what "15mm Bevel Top
    /// Edge.mcx" carries - has its facet across the crease sharing nothing with the other face but
    /// a single corner.
    ///
    /// `None` where the feature does not have two distinct planes to meet along - one plane, or
    /// more than two - and where the two are PARALLEL, which is two faces that never meet at all.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn try_the_ridge(&self) -> Option<(Vector3, Vector3)> {
        let mut first: Option<&SurfaceStandingFeatureWithheldFace> = None;
        let mut second: Option<&SurfaceStandingFeatureWithheldFace> = None;

        for face in &self.faces {
            if first.is_none_or(|first| coplanar(first, face)) {
                first.get_or_insert(face);
                continue;
            }

            if second.is_none_or(|second| coplanar(second, face)) {
                second.get_or_insert(face);
                continue;
            }

            // A third plane is a third wall, and three walls meet at a POINT rather than along a
            // line.
            return None;
        }

        let (first, second) = (first?, second?);

        let along = first.plane_normal.cross(second.plane_normal);

        if !(along.length_squared() > THEY_MEET * THEY_MEET) {
            return None;
        }

        let first_d = first.plane_normal.dot(first.plane_point);
        let second_d = second.plane_normal.dot(second.plane_point);

        // The point of the line nearest the origin, which is as good a point on it as any: the line
        // is carried by its direction from here on.
        let point = ((second.plane_normal.cross(along) * first_d)
            + (along.cross(first.plane_normal) * second_d))
            / along.length_squared();

        Some((point, along.get_normal()))
    }

    /// Which end of one face's crease (the one at `reach`) stands ON the ridge, and which stands in
    /// the band's end plane, as `(on_the_ridge, on_the_end)`.
    ///
    /// The crease runs from the feature's own ridge down to where it dies, so one of its two ends
    /// is on the ridge line and the other is not - measured against the line rather than matched
    /// against an edge, for the same reason the line itself is read off the planes. Both ends on
    /// it, or neither, is a crease this reading cannot tell which way round, and the caller names
    /// it.
    pub fn try_the_ends_of_the_crease(
        &self,
        pool: &SurfaceVertexPool,
        face: &SurfaceStandingFeatureWithheldFace,
        reach: i32,
    ) -> Option<(i32, i32)> {
        let (point, along) = self.try_the_ridge()?;

        let (crease_from, crease_to) = face.creases.borrow()[reach as usize];
        let near = SurfaceVertexPool::near_enough_for(pool.vertices());
        let from = off_the_line(pool.position_of(crease_from), point, along) < near;
        let to = off_the_line(pool.position_of(crease_to), point, along) < near;

        if from == to {
            return None;
        }

        Some(if from {
            (crease_from, crease_to)
        } else {
            (crease_to, crease_from)
        })
    }

    /// Where this feature's WEDGE cuts one rail of a strip - every crossing of one of its face
    /// planes that really stands on the feature, in face order, written into `found` (cleared
    /// first).
    ///
    /// A crossing is on the feature when it stands inside every OTHER face's half space, since the
    /// feature is the wedge those half spaces cut out. Which face a rail is cut by is therefore
    /// asked of WHERE the crossing lands and never of the rail's number. A face running ALONG the
    /// rail cuts it nowhere and is passed over.
    ///
    /// **One reading, two answers.** A rail with ONE crossing is cut back by that face, which is
    /// the end notch's reading (`SurfaceStripNotchedByAFeature`); a rail with TWO is a rail the
    /// wedge passes clean through, which is the bite's (`SurfaceStripBittenByAFeature`).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn crossings_on_the_wedge(
        &self,
        strip: &SurfaceStrip,
        rail: i32,
        snap: f64,
        found: &mut Vec<(Rc<SurfaceStandingFeatureWithheldFace>, f64)>,
    ) {
        found.clear();

        for candidate in &self.faces {
            let Some(at) = strip.try_cross(rail, candidate.plane_point, candidate.plane_normal)
            else {
                continue;
            };

            let point = strip.point_on(rail, at);

            let on_the_wedge = self.faces.iter().all(|other| {
                // `!(.. > snap)` reads a NaN as on the wedge.
                Rc::ptr_eq(other, candidate)
                    || !(other.plane_normal.dot(point - other.plane_point) > snap)
            });

            if on_the_wedge {
                found.push((candidate.clone(), at));
            }
        }
    }
}

/// How far one point stands off the line through `point` along a unit direction.
fn off_the_line(at: Vector3, point: Vector3, along: Vector3) -> f64 {
    let across = at - point;

    (across - (along * across.dot(along))).length()
}

/// Whether two of the feature's faces lie in ONE plane, which is what a face cut into several
/// triangles leaves. The bound is the same hundredth of a degree the plan's own flat-angle reading
/// works to.
fn coplanar(
    first: &SurfaceStandingFeatureWithheldFace,
    second: &SurfaceStandingFeatureWithheldFace,
) -> bool {
    first.plane_normal.dot(second.plane_normal) > 1.0 - THEY_MEET
}

impl SurfaceStandingFeatureWithheld {
    /// The feature as first reached - by one side (`reaching`: edge and whether left) - its landing
    /// and its faces.
    pub fn new(
        reaching: (i32, bool),
        landing: Vector3,
        faces: Vec<Rc<SurfaceStandingFeatureWithheldFace>>,
    ) -> Self {
        SurfaceStandingFeatureWithheld {
            reaches: RefCell::new(vec![reaching]),
            landing,
            faces,
        }
    }
}

impl SurfaceStandingFeatureWithheldFace {
    /// One face of the feature, its first crease and what stands within it, its plane; no rings
    /// yet.
    pub fn new(
        mesh_faces: Vec<i32>,
        crease: (i32, i32),
        within: Vec<i32>,
        plane_point: Vector3,
        plane_normal: Vector3,
    ) -> Self {
        let rings = Vec::with_capacity(mesh_faces.len());
        SurfaceStandingFeatureWithheldFace {
            mesh_faces,
            rings: RefCell::new(rings),
            ring: RefCell::new(None),
            creases: RefCell::new(vec![crease]),
            withins: RefCell::new(vec![within]),
            plane_point,
            plane_normal,
        }
    }

    /// The face's first crease.
    pub fn crease(&self) -> (i32, i32) {
        self.creases.borrow()[0]
    }

    /// What stands within the face's first crease.
    pub fn within(&self) -> Vec<i32> {
        self.withins.borrow()[0].clone()
    }
}
