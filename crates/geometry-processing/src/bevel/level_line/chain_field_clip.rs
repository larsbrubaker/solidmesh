//! The chain field's end clips: whether two ends stand on one plane, the greatest end plane's
//! reading at a point, the clipped point sample, the end-plane reader handed to the convex cell
//! bound, and the convex searches the clipped least runs (along a line, over a triangle, of one
//! clipped term). Ports `OnePlane`, `ClipAt`, `Clipped`, `EndPlaneReader`, `EndPlaneAt`,
//! `LeastOfClippedTerm`, `LeastAlongLine` and `LeastOverTriangleByTernary` from
//! `Bevel/LevelLine/ChainField.EndClip.cs`, `ClipToKeep` from `ChainField.Mitre.cs`, and `Offer`
//! and `MeshPiece.Corners` from `ChainField.Queries.cs`. The type lives in `chain_field.rs`.

use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::super::float_grid;
use super::{ChainField, ChainFieldEndClip, ChainFieldExtension, ChainFieldMeshPiece, FieldSample};

/// Whether two end clips are one plane to FLOAT NOISE only: the same facing to 1e-9 and each
/// vertex within four float steps (at the vertices' magnitude) of the other's plane. Held to float
/// noise because the clip then reads the NEARER end's plane rather than the greater.
pub fn one_plane(a: ChainFieldEndClip, b: ChainFieldEndClip) -> bool {
    let noise = 4.0
        * cs_math::max_f64(
            float_grid::step_at_point(a.vertex),
            float_grid::step_at_point(b.vertex),
        );
    1.0 - a.normal.dot(b.normal) < 1e-9
        && (b.vertex - a.vertex).dot(a.normal).abs() <= noise
        && (a.vertex - b.vertex).dot(b.normal).abs() <= noise
}

/// The greatest end plane's reading at a point over `clips`, and which end it is; -infinity (and
/// the default clip) with none. Both ends on ONE plane (rounds first's joined crease, both ends on
/// the notch wall) read the NEARER end, and a point within a few float steps of that plane reads as
/// ON it, not past it, at both ends.
pub fn clip_at(clips: &[ChainFieldEndClip], point: Vector3) -> (f64, ChainFieldEndClip) {
    if clips.len() == 2 && one_plane(clips[0], clips[1]) {
        // The greater reading is only float noise here - it handed every wall point the far end's
        // parameter (RoundsFirstTests.TheJoinedCreaseCutsAtTheSavedSize). The nearer end is the one.
        let which = if (point - clips[0].vertex).length_squared()
            <= (point - clips[1].vertex).length_squared()
        {
            clips[0]
        } else {
            clips[1]
        };

        // A point ON that plane reads the same from either end: within a few float steps of the
        // plane a point is on it and not PAST it, and keeps its chain point on the end ray at both
        // ends.
        let across = (point - which.vertex).dot(which.normal);
        let value = if across.abs() <= 4.0 * float_grid::step_at_point(point) {
            -ChainField::ON_THE_JOINED_PLANE
        } else {
            across
        };
        return (value, which);
    }

    let mut best = f64::NEG_INFINITY;
    let mut which = ChainFieldEndClip::default();
    for &clip in clips {
        let value = (point - clip.vertex).dot(clip.normal);
        if value > best {
            best = value;
            which = clip;
        }
    }

    (best, which)
}

/// The part of a convex polygon on the kept side of a plane (Sutherland-Hodgman); at most one more
/// point than it had.
pub fn clip_to_keep(polygon: &[Vector3], origin: Vector3, keep: Vector3) -> Vec<Vector3> {
    let mut clipped = Vec::with_capacity(polygon.len() + 1);
    for k in 0..polygon.len() {
        let p = polygon[k];
        let q = polygon[(k + 1) % polygon.len()];
        let sp = (p - origin).dot(keep);
        let sq = (q - origin).dot(keep);
        if sp >= 0.0 {
            clipped.push(p);
        }

        if (sp >= 0.0) != (sq >= 0.0) {
            clipped.push(p + ((q - p) * (sp / (sp - sq))));
        }
    }

    clipped
}

/// The least of a convex function along a segment, ends included, by fixed-step ternary search,
/// with the point attaining it.
pub fn least_along_line(h: &dyn Fn(Vector3) -> f64, a: Vector3, b: Vector3) -> (f64, Vector3) {
    let mut lo = 0.0;
    let mut hi = 1.0;
    for _ in 0..ChainField::CLIP_STEPS {
        let third = (hi - lo) / 3.0;
        if h(a + ((b - a) * (lo + third))) < h(a + ((b - a) * (hi - third))) {
            hi -= third;
        } else {
            lo += third;
        }
    }

    let mut at = a;
    let mut best = h(a);
    for t in [0.5 * (lo + hi), 1.0] {
        let p = a + ((b - a) * t);
        let value = h(p);
        if value < best {
            best = value;
            at = p;
        }
    }

    (best, at)
}

/// The least of a convex function over a triangle: `p = a + s (b - a) + r (c - b)`,
/// `0 <= r <= s <= 1` is affine in (s, r) on a convex domain, so the least over each slice s is
/// convex in s.
pub fn least_over_triangle_by_ternary(
    h: &dyn Fn(Vector3) -> f64,
    a: Vector3,
    b: Vector3,
    c: Vector3,
) -> (f64, Vector3) {
    let slice = |s: f64| least_along_line(h, a + ((b - a) * s), a + ((b - a) * s) + ((c - b) * s));

    let mut lo = 0.0;
    let mut hi = 1.0;
    for _ in 0..ChainField::CLIP_STEPS {
        let third = (hi - lo) / 3.0;
        if slice(lo + third).0 < slice(hi - third).0 {
            hi -= third;
        } else {
            lo += third;
        }
    }

    let (mut best, mut at) = slice(0.0);
    for s in [0.5 * (lo + hi), 1.0] {
        let (value, p) = slice(s);
        if value < best {
            best = value;
            at = p;
        }
    }

    (best, at)
}

impl ChainField {
    /// The greatest end plane's reading at a point, and which end it is; -infinity with no clip.
    pub fn clip_at(&self, point: Vector3) -> (f64, ChainFieldEndClip) {
        clip_at(self.end_clips.as_deref().unwrap_or(&[]), point)
    }

    /// A point sample with the clip applied: where an end plane stands above the chain's reading,
    /// or ties it to rounding ([`ChainField::END_PLANE_TIE_RELATIVE`]), the value is the plane's
    /// and the chain point reported is that end's - its vertex and end parameter. Such a tie CLIPS:
    /// the value is the same either way, and the vertex is where the band stops, so its chain point
    /// is the end. The tie is rounding-sized, a tenth of the field's: an exact tie clips wherever
    /// the part sits, while a root-solved vertex beside the plane keeps its own chain point.
    pub fn clipped(&self, point: Vector3, sample: FieldSample) -> FieldSample {
        if self.end_clips.is_none() {
            return sample;
        }

        let (plane, clip) = self.clip_at(point);
        let threshold = sample.value - (Self::END_PLANE_TIE_RELATIVE * (1.0 + sample.value.abs()));
        // C#'s `!(plane > threshold)`: a NaN on either side keeps the sample.
        if plane.is_nan() || threshold.is_nan() || plane <= threshold {
            return sample;
        }

        FieldSample {
            distance: (point - clip.vertex).length(),
            parameter: clip.parameter,
            value: plane,
            nearest: clip.vertex,
            clipped: true,
            ..sample
        }
    }

    /// The greatest end plane's reading at a point, or `None` for a field no end plane clips -
    /// what the convex cell bound needs to bound the plane's part of the field over a cell. Made
    /// once (C# `endPlaneReader ??=`); it carries its own copy of the clips, which never change.
    pub fn end_plane_reader(&self) -> Option<super::ChainFieldEndPlaneReader> {
        let clips = self.end_clips.as_ref()?;
        Some(Rc::clone(self.end_plane_reader.get_or_init(|| {
            let clips = clips.clone();
            Rc::new(move |_chain: i32, point: Vector3| clip_at(&clips, point).0)
        })))
    }

    /// The greatest end plane's reading at a point; -infinity when no end is clipped.
    pub fn end_plane_at(&self, point: Vector3) -> f64 {
        if self.end_clips.is_none() {
            f64::NEG_INFINITY
        } else {
            self.clip_at(point).0
        }
    }

    /// The least over the piece - cut to the ray's side of its bisector when `ray` is given - of
    /// `max(term(p), plane(p))`, a convex function, by fixed-step ternary search, with where it
    /// stands; +infinity where the ray's side cuts the piece.
    pub fn least_of_clipped_term(
        &self,
        piece: &ChainFieldMeshPiece,
        ray: Option<&ChainFieldExtension>,
        term: &dyn Fn(Vector3) -> f64,
    ) -> (f64, Vector3) {
        let h = |p: Vector3| cs_math::max_f64(term(p), self.clip_at(p).0);

        let (corners, count) = piece.corners();
        let mut polygon: Vec<Vector3> = corners[..count].to_vec();
        if let Some(ray) = ray {
            // A segment clips to its two kept ends (or one end and the crossing), a triangle to 3
            // or 4 corners.
            let clipped = clip_to_keep(&polygon, ray.origin, ray.keep);
            if clipped.len() < count {
                return (f64::INFINITY, Vector3::ZERO);
            }

            polygon = clipped[..if count == 2 { 2 } else { clipped.len() }].to_vec();
        }

        if polygon.len() == 2 {
            return least_along_line(&h, polygon[0], polygon[1]);
        }

        let (mut best, mut at) =
            least_over_triangle_by_ternary(&h, polygon[0], polygon[1], polygon[2]);
        if polygon.len() == 4 {
            let (other, other_at) =
                least_over_triangle_by_ternary(&h, polygon[0], polygon[2], polygon[3]);
            if other < best {
                best = other;
                at = other_at;
            }
        }

        (best, at)
    }

    /// Takes one segment's answer if it beats what is held - strictly less, with a few bits of
    /// room, so a tie keeps the LOWER chain segment's answer whichever order the two were visited
    /// in. The very first reading is taken outright, because every comparison against an infinite
    /// best is a NaN and therefore false. The held answer is (value, point on the piece, segment).
    pub fn offer(segment: i32, value: f64, on_piece: Vector3, best: &mut (f64, Vector3, i32)) {
        let margin = Self::TIE_RELATIVE * (1.0 + best.0.abs());
        let take_it = best.0 == f64::INFINITY
            || value < best.0 - margin
            || (value < best.0 + margin && segment < best.2);
        if take_it {
            *best = (value, on_piece, segment);
        }
    }
}

impl ChainFieldMeshPiece {
    /// The piece's corners - two for a segment, three for a triangle - and how many there are.
    pub fn corners(&self) -> ([Vector3; 3], usize) {
        (
            [self.a, self.b, self.c],
            if self.is_triangle { 3 } else { 2 },
        )
    }
}
