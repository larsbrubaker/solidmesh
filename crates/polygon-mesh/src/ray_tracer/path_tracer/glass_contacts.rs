//! Where glass parts lie face on face: [`GlassContacts`], found once when a
//! [`super::RenderScene`] is built, lists for each triangle of a glass part the triangles of other
//! glass parts lying on it, and answers at a hit whether the point is covered by one of them.
//! The same search, run between each glass part and each part that is not glass
//! ([`GlassContacts::find_both`] builds both tables), gives a second table of where glass rests on
//! an opaque surface, a floor most of all (`glass_rest` says what a path does there).
//!
//! **What counts as a contact.** Two triangles of different glass parts are in contact when
//! their normals point opposite ways (cosine below `-ANTI_PARALLEL`), every corner of each lies
//! within the pair's tolerance of the other's plane, measured along the normal, and the two
//! overlap in the plane by more than that tolerance (touching only along an edge is no contact).
//! The tolerance is `CONTACT_ROUNDING_STEPS` times the larger of the two triangles' rounding
//! scales, the distance a continuing ray already starts off a face (`surface_point`'s header):
//! the `f32` rounding of the corners as the triangle is traced, carried through the part's
//! matrix, plus the `f64` rounding of the world position. That is the error the `f32` trace itself
//! makes on the triangle, whose plane it knows only to steps of its corners' size: two faces
//! closer than that can't be told apart by tracing, so nothing tighter would hold. It is fixed by
//! the coordinates, not by any ray, and it follows the larger triangle's size: twice 8 `f32` steps
//! of its largest corner L1 magnitude, about 3.8e-6 times the half side `h` of a floor quad
//! centred under the glass. Over a floor of `h` = 200 that is 7.6e-4, so a face a modelled gap
//! away (0.05 off it) is a real gap of air and stays one. Over a floor of `h` = 40,000 it is about
//! 0.15, and of `h` = 90,000 about 0.34: glass lifted less than that is taken as resting on the
//! floor. A caller who models a gap under a part keeps the triangles there small (a large floor
//! split into tiles near the parts), because the tolerance follows them.
//!
//! **How it is found without comparing every pair.** A scene with fewer than two glass parts
//! searches nothing for glass on glass, and one with no glass or no other part nothing for glass
//! resting; the resting search pairs glass parts only with parts that are not glass. Pairs are
//! first kept only where their world bounds come within a bound on the tolerance of each other
//! (worked out from the bounds and the matrix, no triangle needed), and only the parts in a kept
//! pair have their triangles carried into world space, a glass part once for both searches. Within
//! a pair, only triangles inside the other part's bounds are compared. Those of one part go into a
//! uniform grid over the region the two parts share, by their bounds widened by the tolerance, so a
//! triangle near a cell edge is in every cell it could match from; each triangle of the other part
//! is tested against the grid's triangles in the cells its bounds cover. Both are clamped to the
//! shared region, so a triangle far larger than that region (a floor under a small part) visits
//! only its cells: the work follows the shared region and the triangles in it, never the size of a
//! triangle reaching past it.
//!
//! **Which edges are widened.** At a hit the point is tested against the contact triangles with
//! each edge moved out by the tolerance only where nothing but glass can lie beyond it: an edge
//! the contact triangle shares with another contact triangle of the same face, or one lying along
//! an edge of the hit face itself. An edge across the middle of the hit face, where the covering
//! part ends and air begins, is tested as it is, so a point in the air beyond it is never taken
//! for a seam.
//!
//! **Order.** The list is sorted by (part, face, other part, other face), so a triangle's
//! contacts are one contiguous run whose order follows from the parts alone: the same scene gives
//! the same list, and the first covering contact is always the same one.
//!
//! **Plain data.** Each glass part with any contact has one start per face plus an end into the
//! shared list; each entry is the other part and face, its world corners, which of its edges are
//! widened and the pair's tolerance. A part with no contact has no starts at all, so the only
//! cost a hit on it pays is one empty check.

use std::collections::HashMap;

use solidmesh_math::Vector3;

use super::super::surface_point::{
    l1_magnitude, surface_offset, SURFACE_OFFSET_F32_STEPS, WORLD_OFFSET_F64_STEPS,
};
use super::super::surface_table::unit_or;
use crate::ray_tracer::{PlacedPart, PlacedParts, ScatteringSurface, Traceable};

/// How many rounding scales apart two faces may be and still lie on each other. The rounding
/// scale is already 8 `f32` steps of the corners, which measured as enough for a ray to leave its
/// own face; two faces each carry their own rounding, so twice the larger covers both.
const CONTACT_ROUNDING_STEPS: f64 = 2.0;

/// How close to exactly opposite two contact faces' normals must be: about 0.08 degrees.
const ANTI_PARALLEL: f64 = 0.999_999;

/// The most grid cells along an axis of the region two parts share, so a few large triangles
/// among many small ones can't make the grid explode.
const MOST_CELLS_PER_AXIS: f64 = 64.0;

/// One triangle of another part lying on a triangle of this one: in the glass-on-glass table
/// another glass part's, in the resting table the part across the contact (glass under an opaque
/// face's entry, opaque under a glass face's).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ContactFace {
    /// The other part's index in the scene.
    pub(crate) part: usize,
    /// The other part's face.
    pub(crate) face: usize,
    /// The other face's corners in world space.
    pub(crate) corners: [Vector3; 3],
    /// How far apart, along the normal or in the plane, the two faces may be and still touch.
    pub(crate) tolerance: f64,
    /// For each edge, from corner `i` to corner `i + 1`, whether a hit may lie the tolerance
    /// beyond it and still be covered.
    pub(crate) widened: [bool; 3],
}

/// What finding where glass faces lie on each other, or on opaque faces, took when a scene was
/// built ([`RenderScene::glass_contact_search_work`](super::RenderScene::glass_contact_search_work),
/// [`RenderScene::glass_rest_search_work`](super::RenderScene::glass_rest_search_work)): a
/// read-only statistic, counted in work and not time so it is the same on every machine, for a
/// caller or test to see that the search follows the region parts share and not the scene's size.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ContactSearchWork {
    /// Triangles carried into world space for the search.
    pub triangles_built: usize,
    /// Grid cells a triangle was put into or looked up in.
    pub cells_visited: usize,
    /// Triangle pairs tested for lying on each other.
    pub pairs_tested: usize,
}

/// For each triangle of each part with a contact, the triangles of the other parts lying on it:
/// glass on glass in one table, glass on opaque in the other (each a [`GlassContacts`]).
#[derive(Debug, Clone, Default)]
pub(crate) struct GlassContacts {
    /// Per part, empty when the part has no contact, else one start per face and the end.
    starts: Vec<Vec<u32>>,
    entries: Vec<ContactFace>,
    work: ContactSearchWork,
}

/// A triangle of a glass part, in world space, with what the search needs.
struct WorldTriangle {
    face: usize,
    corners: [Vector3; 3],
    normal: Vector3,
    rounding: f64,
    low: [f64; 3],
    high: [f64; 3],
}

/// One contact as the search finds it: face `face` of part `part`, with `own` its corners.
struct Found {
    part: usize,
    face: usize,
    own: [Vector3; 3],
    contact: ContactFace,
}

/// A part's world bounds and an upper bound on any of its triangles' rounding scales, from the
/// bounds and the matrix alone.
struct PartBounds {
    low: [f64; 3],
    high: [f64; 3],
    rounding: f64,
}

impl PartBounds {
    fn of(part: &PlacedPart) -> PartBounds {
        let world = part.get_axis_aligned_bounding_box();
        let mesh = world.new_transformed(part.world_to_mesh());
        let farthest_l1 = |low: Vector3, high: Vector3| {
            (0..3).fold(0.0, |sum, axis| sum + low[axis].abs().max(high[axis].abs()))
        };
        // The matrix stretches a vector by at most the root of the sum of its squared axes.
        let to_world = part.mesh_to_world();
        let stretch = [Vector3::UNIT_X, Vector3::UNIT_Y, Vector3::UNIT_Z]
            .iter()
            .fold(0.0, |sum, &axis| {
                sum + axis.transform_vector(to_world).length_squared()
            })
            .sqrt();
        let mesh_l1 = farthest_l1(mesh.min_xyz, mesh.max_xyz);
        let world_l1 = farthest_l1(world.min_xyz, world.max_xyz);
        let rounding = mesh_l1 * f64::from(f32::EPSILON) * SURFACE_OFFSET_F32_STEPS * stretch
            + world_l1 * f64::EPSILON * WORLD_OFFSET_F64_STEPS;
        PartBounds {
            low: [world.min_xyz.x, world.min_xyz.y, world.min_xyz.z],
            high: [world.max_xyz.x, world.max_xyz.y, world.max_xyz.z],
            rounding,
        }
    }
}

/// An edge by its two corners' bits, the same whichever way round it runs, and the part it is
/// on: two contact triangles of one part share an edge exactly when they share its corners.
type EdgeKey = (usize, [u64; 3], [u64; 3]);

fn edge_key(part: usize, a: Vector3, b: Vector3) -> EdgeKey {
    let (a, b) = (
        [a.x.to_bits(), a.y.to_bits(), a.z.to_bits()],
        [b.x.to_bits(), b.y.to_bits(), b.z.to_bits()],
    );
    if a <= b {
        (part, a, b)
    } else {
        (part, b, a)
    }
}

/// How many contact triangles of `group` have each edge.
fn edge_counts(group: &[Found]) -> HashMap<EdgeKey, u32> {
    let mut counts = HashMap::new();
    for item in group {
        let corners = item.contact.corners;
        for edge in 0..3 {
            let key = edge_key(item.contact.part, corners[edge], corners[(edge + 1) % 3]);
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts
}

/// Which edges of `item`'s contact triangle are widened: those it shares with another contact
/// triangle of its group (`counts`), and those along an edge of the face they all lie on.
fn widened_edges(item: &Found, counts: &HashMap<EdgeKey, u32>) -> [bool; 3] {
    let contact = &item.contact;
    let own = item.own;
    std::array::from_fn(|edge| {
        let (start, end) = (contact.corners[edge], contact.corners[(edge + 1) % 3]);
        let shared = counts
            .get(&edge_key(contact.part, start, end))
            .is_some_and(|&count| count >= 2);
        shared
            || (0..3).any(|k| {
                let (a, b) = (own[k], own[(k + 1) % 3]);
                off_line(start, a, b) <= contact.tolerance
                    && off_line(end, a, b) <= contact.tolerance
            })
    })
}

/// How far `point` is from the line through `a` and `b`.
fn off_line(point: Vector3, a: Vector3, b: Vector3) -> f64 {
    let along = b - a;
    let length = along.length();
    if length <= 0.0 {
        return (point - a).length();
    }
    (point - a).cross(along).length() / length
}

impl GlassContacts {
    /// Where faces of the parts whose `surfaces` are glass lie on each other, and where they lie on
    /// faces of the parts whose surfaces are not (`glass_rest` says what a path does there), in
    /// that order. In the second table each entry of a glass face is an opaque face and each entry
    /// of an opaque face a glass one. A glass part's world triangles are carried into world space
    /// once for both searches.
    pub(crate) fn find_both(
        parts: &PlacedParts,
        surfaces: &[ScatteringSurface],
    ) -> (GlassContacts, GlassContacts) {
        let count = parts.parts().len();
        let is_glass_at = |at: usize| surfaces.get(at).is_some_and(is_glass);
        let glass: Vec<usize> = (0..count).filter(|&at| is_glass_at(at)).collect();
        let opaque: Vec<usize> = (0..count).filter(|&at| !is_glass_at(at)).collect();
        let mut built: Vec<Option<Vec<WorldTriangle>>> = (0..count).map(|_| None).collect();
        let contacts = if glass.len() < 2 {
            GlassContacts::default()
        } else {
            let pairs = (0..glass.len())
                .flat_map(|first| (first + 1..glass.len()).map(move |second| (first, second)));
            GlassContacts::search(parts, (&glass, &glass), pairs, &mut built)
        };
        let rests = if glass.is_empty() || opaque.is_empty() {
            GlassContacts::default()
        } else {
            let pairs = (0..glass.len())
                .flat_map(|first| (0..opaque.len()).map(move |second| (first, second)));
            GlassContacts::search(parts, (&glass, &opaque), pairs, &mut built)
        };
        (contacts, rests)
    }

    /// The contacts between parts `firsts[i]` and `seconds[j]` for each pair `(i, j)` of `pairs`
    /// whose bounds come close enough to touch. `built` holds each scene part's world triangles
    /// once carried; the work counts only the triangles this search carried itself.
    fn search(
        parts: &PlacedParts,
        (firsts, seconds): (&[usize], &[usize]),
        pairs: impl Iterator<Item = (usize, usize)>,
        built: &mut [Option<Vec<WorldTriangle>>],
    ) -> GlassContacts {
        let bounds_of_parts = |members: &[usize]| -> Vec<PartBounds> {
            members
                .iter()
                .map(|&at| PartBounds::of(&parts.parts()[at]))
                .collect()
        };
        let (first_bounds, second_bounds) = (bounds_of_parts(firsts), bounds_of_parts(seconds));
        let mut kept = Vec::new();
        for (first, second) in pairs {
            let (a, b) = (&first_bounds[first], &second_bounds[second]);
            let margin = CONTACT_ROUNDING_STEPS * a.rounding.max(b.rounding);
            if boxes_meet(&a.low, &a.high, &b.low, &b.high, margin) {
                kept.push((firsts[first], seconds[second]));
            }
        }
        let mut work = ContactSearchWork::default();
        for &(a, b) in &kept {
            for at in [a, b] {
                if built[at].is_none() {
                    let triangles = world_triangles(&parts.parts()[at]);
                    work.triangles_built += triangles.len();
                    built[at] = Some(triangles);
                }
            }
        }
        let mut found: Vec<Found> = Vec::new();
        for &(a, b) in &kept {
            let (Some(triangles_a), Some(triangles_b)) = (&built[a], &built[b]) else {
                continue;
            };
            if triangles_a.is_empty() || triangles_b.is_empty() {
                continue;
            }
            contacts_between((a, triangles_a), (b, triangles_b), &mut found, &mut work);
        }
        GlassContacts::from_found(parts, found, work)
    }

    /// The table of `found`, contacts in both directions, sorted and grouped by face.
    fn from_found(
        parts: &PlacedParts,
        mut found: Vec<Found>,
        work: ContactSearchWork,
    ) -> GlassContacts {
        if found.is_empty() {
            return GlassContacts {
                work,
                ..GlassContacts::default()
            };
        }
        found.sort_by_key(|x| (x.part, x.face, x.contact.part, x.contact.face));
        let mut starts = vec![Vec::new(); parts.parts().len()];
        let mut entries = Vec::with_capacity(found.len());
        let mut at = 0;
        while at < found.len() {
            let part = found[at].part;
            let faces = parts.parts()[part].surface().face_count();
            let mut part_starts = Vec::with_capacity(faces + 1);
            for face in 0..faces {
                // The list is far below u32::MAX entries for any mesh a scene can hold.
                part_starts.push(entries.len() as u32);
                let first = at;
                while at < found.len() && found[at].part == part && found[at].face == face {
                    at += 1;
                }
                let group = &found[first..at];
                let counts = edge_counts(group);
                for item in group {
                    let mut contact = item.contact;
                    contact.widened = widened_edges(item, &counts);
                    entries.push(contact);
                }
            }
            part_starts.push(entries.len() as u32);
            starts[part] = part_starts;
        }
        GlassContacts {
            starts,
            entries,
            work,
        }
    }

    /// What finding the contacts took.
    pub(crate) fn work(&self) -> ContactSearchWork {
        self.work
    }

    /// Whether no face lies on another in this table.
    pub(crate) fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The contacts lying on face `face` of `part`, in list order; empty where it meets air.
    pub(crate) fn face_contacts(&self, part: usize, face: usize) -> &[ContactFace] {
        let Some(starts) = self.starts.get(part) else {
            return &[];
        };
        match (starts.get(face), starts.get(face + 1)) {
            (Some(&start), Some(&end)) => self
                .entries
                .get(start as usize..end as usize)
                .unwrap_or(&[]),
            _ => &[],
        }
    }

    /// The contact face of another part covering `position` on face `face` of `part` (another
    /// glass part's in the glass-on-glass table, the part across the contact in the resting one),
    /// the first in list order when two do, `None` where the face meets air there.
    pub(crate) fn covering(
        &self,
        part: usize,
        face: usize,
        position: Vector3,
    ) -> Option<&ContactFace> {
        let starts = self.starts.get(part)?;
        let (&start, &end) = (starts.get(face)?, starts.get(face + 1)?);
        self.entries
            .get(start as usize..end as usize)?
            .iter()
            .find(|contact| covers(contact, position))
    }
}

/// Whether `surface` is glass: it lets light through and is not all metal.
pub(crate) fn is_glass(surface: &ScatteringSurface) -> bool {
    surface.transmission() > 0.0 && surface.metallic() < 1.0
}

/// The part's triangles in world space; faces with broken corners are left out.
fn world_triangles(part: &PlacedPart) -> Vec<WorldTriangle> {
    let mesh_to_world = part.mesh_to_world();
    let world_to_mesh = part.world_to_mesh();
    let table = part.surface();
    (0..table.face_count())
        .filter_map(|face| {
            let local = table.face_corners(face)?;
            let corners = local.map(|corner| corner.transform_position(mesh_to_world));
            if corners.iter().any(|corner| !corner.is_valid()) {
                return None;
            }
            let (local_normal, _) = table.face_normals(face);
            let normal = unit_or(
                local_normal.transform_normal_inverse(world_to_mesh),
                local_normal,
            );
            let largest_l1 = corners.iter().fold(0.0_f64, |largest, &corner| {
                largest.max(l1_magnitude(corner))
            });
            let rounding = (local_normal * surface_offset(local))
                .transform_vector(mesh_to_world)
                .length()
                + largest_l1 * f64::EPSILON * WORLD_OFFSET_F64_STEPS;
            let mut low = [f64::INFINITY; 3];
            let mut high = [f64::NEG_INFINITY; 3];
            for corner in corners {
                for axis in 0..3 {
                    low[axis] = low[axis].min(corner[axis]);
                    high[axis] = high[axis].max(corner[axis]);
                }
            }
            Some(WorldTriangle {
                face,
                corners,
                normal,
                rounding,
                low,
                high,
            })
        })
        .collect()
}

/// The bounds of `triangles` and their largest rounding scale.
fn bounds_of(triangles: &[WorldTriangle]) -> ([f64; 3], [f64; 3], f64) {
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    let mut rounding = 0.0_f64;
    for triangle in triangles {
        for axis in 0..3 {
            low[axis] = low[axis].min(triangle.low[axis]);
            high[axis] = high[axis].max(triangle.high[axis]);
        }
        rounding = rounding.max(triangle.rounding);
    }
    (low, high, rounding)
}

fn boxes_meet(
    low: &[f64; 3],
    high: &[f64; 3],
    other_low: &[f64; 3],
    other_high: &[f64; 3],
    margin: f64,
) -> bool {
    (0..3).all(|axis| {
        low[axis] <= other_high[axis] + margin && other_low[axis] <= high[axis] + margin
    })
}

/// Adds to `found` every contact between the triangles of parts `a` and `b`, both ways round.
fn contacts_between(
    (part_a, triangles_a): (usize, &[WorldTriangle]),
    (part_b, triangles_b): (usize, &[WorldTriangle]),
    found: &mut Vec<Found>,
    work: &mut ContactSearchWork,
) {
    let (low_a, high_a, rounding_a) = bounds_of(triangles_a);
    let (low_b, high_b, rounding_b) = bounds_of(triangles_b);
    let margin = CONTACT_ROUNDING_STEPS * rounding_a.max(rounding_b);
    if !boxes_meet(&low_a, &high_a, &low_b, &high_b, margin) {
        return;
    }
    let near_b: Vec<&WorldTriangle> = triangles_a
        .iter()
        .filter(|t| boxes_meet(&t.low, &t.high, &low_b, &high_b, margin))
        .collect();
    let near_a: Vec<&WorldTriangle> = triangles_b
        .iter()
        .filter(|t| boxes_meet(&t.low, &t.high, &low_a, &high_a, margin))
        .collect();
    if near_b.is_empty() || near_a.is_empty() {
        return;
    }
    let grid = Grid::of(&near_a, margin, &low_a, &high_a, &low_b, &high_b, work);
    let mut candidates = Vec::new();
    for triangle in &near_b {
        work.cells_visited += grid.candidates(triangle, margin, &mut candidates);
        work.pairs_tested += candidates.len();
        for &at in &candidates {
            let other = near_a[at];
            let tolerance = CONTACT_ROUNDING_STEPS * triangle.rounding.max(other.rounding);
            if lie_on_each_other(triangle, other, tolerance) {
                found.push(Found {
                    part: part_a,
                    face: triangle.face,
                    own: triangle.corners,
                    contact: ContactFace {
                        part: part_b,
                        face: other.face,
                        corners: other.corners,
                        tolerance,
                        widened: [false; 3],
                    },
                });
                found.push(Found {
                    part: part_b,
                    face: other.face,
                    own: other.corners,
                    contact: ContactFace {
                        part: part_a,
                        face: triangle.face,
                        corners: triangle.corners,
                        tolerance,
                        widened: [false; 3],
                    },
                });
            }
        }
    }
}

/// A uniform grid of triangles by their widened bounds, over the region two parts share.
struct Grid {
    cell: f64,
    origin: [f64; 3],
    /// The last cell index along each axis inside the shared region.
    last: [i64; 3],
    cells: HashMap<[i64; 3], Vec<usize>>,
}

impl Grid {
    /// `triangles` in cells about their mean size, no smaller than the region the two parts
    /// share over `MOST_CELLS_PER_AXIS`.
    fn of(
        triangles: &[&WorldTriangle],
        margin: f64,
        low_a: &[f64; 3],
        high_a: &[f64; 3],
        low_b: &[f64; 3],
        high_b: &[f64; 3],
        work: &mut ContactSearchWork,
    ) -> Grid {
        let mut size_sum = 0.0;
        for triangle in triangles {
            let extent = (0..3).fold(0.0_f64, |e, axis| {
                e.max(triangle.high[axis] - triangle.low[axis])
            });
            size_sum += extent;
        }
        let mean = size_sum / triangles.len() as f64;
        let origin = [0, 1, 2].map(|axis| low_a[axis].max(low_b[axis]) - margin);
        let end = [0, 1, 2].map(|axis| high_a[axis].min(high_b[axis]) + margin);
        let region = (0..3).fold(0.0_f64, |e, axis| e.max(end[axis] - origin[axis]));
        let cell = mean
            .max(region / MOST_CELLS_PER_AXIS)
            .max(margin * 4.0)
            .max(f64::MIN_POSITIVE);
        // The region is at most MOST_CELLS_PER_AXIS cells across, so the cast is exact.
        let last =
            [0, 1, 2].map(|axis| ((end[axis] - origin[axis]) / cell).floor().max(0.0) as i64);
        let mut grid = Grid {
            cell,
            origin,
            last,
            cells: HashMap::new(),
        };
        for (at, triangle) in triangles.iter().enumerate() {
            let (from, to) = grid.span(triangle, margin);
            for x in from[0]..=to[0] {
                for y in from[1]..=to[1] {
                    for z in from[2]..=to[2] {
                        grid.cells.entry([x, y, z]).or_default().push(at);
                        work.cells_visited += 1;
                    }
                }
            }
        }
        grid
    }

    /// The cells of the shared region that `triangle`'s bounds widened by `margin` cover; an
    /// empty range along an axis where they miss it.
    fn span(&self, triangle: &WorldTriangle, margin: f64) -> ([i64; 3], [i64; 3]) {
        // `as` saturates for a triangle reaching far past the region, and the clamp then brings
        // the index back inside it, so a huge triangle visits only the region's cells.
        let index =
            |value: f64, axis: usize| ((value - self.origin[axis]) / self.cell).floor() as i64;
        let from = [0, 1, 2].map(|axis| index(triangle.low[axis] - margin, axis).max(0));
        let to =
            [0, 1, 2].map(|axis| index(triangle.high[axis] + margin, axis).min(self.last[axis]));
        (from, to)
    }

    /// The grid's triangles in the cells `triangle` covers, ascending and each once.
    /// Returns how many cells it looked in.
    fn candidates(&self, triangle: &WorldTriangle, margin: f64, out: &mut Vec<usize>) -> usize {
        out.clear();
        let mut visited = 0;
        let (from, to) = self.span(triangle, margin);
        for x in from[0]..=to[0] {
            for y in from[1]..=to[1] {
                for z in from[2]..=to[2] {
                    visited += 1;
                    if let Some(cell) = self.cells.get(&[x, y, z]) {
                        out.extend_from_slice(cell);
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        visited
    }
}

/// Whether `a` and `b` face opposite ways on one plane and overlap there.
fn lie_on_each_other(a: &WorldTriangle, b: &WorldTriangle, tolerance: f64) -> bool {
    if a.normal.dot(b.normal) > -ANTI_PARALLEL {
        return false;
    }
    let off_plane = |plane: &WorldTriangle, corners: &[Vector3; 3]| {
        corners
            .iter()
            .all(|&corner| (corner - plane.corners[0]).dot(plane.normal).abs() <= tolerance)
    };
    if !off_plane(a, &b.corners) || !off_plane(b, &a.corners) {
        return false;
    }
    let drop = dominant_axis(a.normal);
    let flat_a = a.corners.map(|corner| flatten(corner, drop));
    let flat_b = b.corners.map(|corner| flatten(corner, drop));
    overlap_in_plane(&flat_a, &flat_b, tolerance)
}

/// The axis `normal` is most along, the one a projection onto the plane drops.
pub(crate) fn dominant_axis(normal: Vector3) -> usize {
    let (x, y, z) = (normal.x.abs(), normal.y.abs(), normal.z.abs());
    if x >= y && x >= z {
        0
    } else if y >= z {
        1
    } else {
        2
    }
}

/// `point` in the plane that drops axis `drop`, the two kept axes in cyclic order.
pub(crate) fn flatten(point: Vector3, drop: usize) -> [f64; 2] {
    match drop {
        0 => [point.y, point.z],
        1 => [point.z, point.x],
        _ => [point.x, point.y],
    }
}

/// Whether two flat triangles overlap by more than `tolerance` across every edge direction of
/// either: separated along any edge's normal means they don't overlap with area.
fn overlap_in_plane(a: &[[f64; 2]; 3], b: &[[f64; 2]; 3], tolerance: f64) -> bool {
    for triangle in [a, b] {
        for edge in 0..3 {
            let start = triangle[edge];
            let end = triangle[(edge + 1) % 3];
            let axis = [end[1] - start[1], start[0] - end[0]];
            let length = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
            if length <= 0.0 {
                return false;
            }
            let along = |p: &[f64; 2]| (p[0] * axis[0] + p[1] * axis[1]) / length;
            let (low_a, high_a) = interval(a, along);
            let (low_b, high_b) = interval(b, along);
            if high_a.min(high_b) - low_a.max(low_b) <= tolerance {
                return false;
            }
        }
    }
    true
}

fn interval(triangle: &[[f64; 2]; 3], along: impl Fn(&[f64; 2]) -> f64) -> (f64, f64) {
    triangle
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), p| {
            let value = along(p);
            (low.min(value), high.max(value))
        })
}

/// Whether `position` lies on `contact`'s triangle in its plane, its widened edges moved out by
/// the contact's tolerance so a point on an edge two contact triangles share is covered by both.
pub(crate) fn covers(contact: &ContactFace, position: Vector3) -> bool {
    let [c0, c1, c2] = contact.corners;
    let normal = (c1 - c0).cross(c2 - c0);
    let drop = dominant_axis(normal);
    let corners = contact.corners.map(|corner| flatten(corner, drop));
    let point = flatten(position, drop);
    let area = (corners[1][0] - corners[0][0]) * (corners[2][1] - corners[0][1])
        - (corners[1][1] - corners[0][1]) * (corners[2][0] - corners[0][0]);
    if area == 0.0 {
        return false;
    }
    let sign = if area > 0.0 { 1.0 } else { -1.0 };
    (0..3).all(|edge| {
        let start = corners[edge];
        let end = corners[(edge + 1) % 3];
        let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
        let side = (dx * (point[1] - start[1]) - dy * (point[0] - start[0])) * sign;
        if contact.widened[edge] {
            side >= -contact.tolerance * (dx * dx + dy * dy).sqrt()
        } else {
            side >= 0.0
        }
    })
}
