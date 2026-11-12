//! Glass lying on glass, packed for the WGSL path loop: the scene's glass contacts
//! (`path_tracer/glass_contacts.rs`) as plain `u32` words after the part records, so
//! the shader can decide a seam (`path_tracer/glass_seam.rs`) with no surface comparison of its
//! own. `gpu_pack.rs` builds it when it finishes a pack; `gpu_scene.rs` carries it.
//!
//! **The block rides in the part buffer.** A default WebGPU device allows 8 storage buffers per
//! stage and the path loop's are spoken for, so the contacts take no binding of their own: their
//! words follow the part records in the part buffer (`path.wgsl` reads it as words), and the view
//! record says where they start ([`super::ViewRecord::contact_block`], a `u32` of its own in the
//! lane after `forward`). A scene without contacts appends nothing and leaves that word 0 and
//! [`super::SCENE_FLAG_GLASS_CONTACTS`] clear, so every buffer it packs is the same bits as before
//! contacts existed. Otherwise the block is the triangle count `T`; then word `t` for each triangle
//! `t` in `source_index` order, the index of its first entry, with [`CONTACT_FULLY_COVERED`] set
//! when its first entry covers the whole face, and one more holding the entry count, so triangle
//! `t`'s entries run from its start to the next triangle's, nearly always none; then the entries,
//! [`CONTACT_ENTRY_WORDS`] words each, in the CPU's list order, so the first covering entry is the
//! one the CPU finds. `seam.wgsl` reads it.
//!
//! **An entry** is everything one contact triangle decides, worked out in `f64` here:
//! - its three edges as the shader tests them, in the scene-centre frame the packed triangles use
//!   (`gpu_scene.rs`, recentring), projected by dropping the contact triangle's dominant axis
//!   (`glass_contacts::flatten`): an edge's start, its direction already multiplied by the sign of
//!   the triangle's projected area, and how far behind the edge a point may lie and still be
//!   covered (the tolerance times the edge's length on a widened edge, 0 on any other). A point
//!   `p` is covered when, for every edge, `direction.u * (p.v - start.v) - direction.v * (p.u -
//!   start.u) >= -allowance`, the CPU's `covers` with the sign folded in;
//! - the tolerance: the larger of the CPU's contact tolerance and the two faces' own `f32`
//!   continuing-ray offsets, since the shader's hit point carries `f32` rounding the CPU's does
//!   not, and a ray skipping the seam by twice it must start clear of both faces as packed. The
//!   same tolerance widens the edges, so a point on an edge two contact triangles share is
//!   covered whichever way its `f32` rounding moved it;
//! - the covering part (for the per-line presence rule), the covering face's color (for that part's
//!   own glass against air), the flags below and the interface surface for two different glasses
//!   (`glass_seam::seam_surface`: relative index from the `f64` indices, colors multiplied, the
//!   larger roughness and metallic, the smaller transmission). The interface is the same whichever
//!   side the path comes from; only which side is denser depends on that, so both answers are
//!   flags.
//!
//! **Fully covered faces.** A face whose first entry covers all three of its corners (by the CPU's
//! own `f64` test) is covered by that entry everywhere, since a widened triangle is convex, and that
//! entry is first: the shader takes it with no point test, so `f32` and `f64` cannot disagree there.

use solidmesh_math::Vector3;

use super::{FaceRecord, FlatTreeError, SurfaceRecord, PART_STRIDE, SURFACE_RECORD_STRIDE};
use crate::ray_tracer::path_tracer::{covers, dominant_axis, flatten, seam_surface, ContactFace};
use crate::ray_tracer::RenderScene;

/// Words per contact entry: three edges of [`CONTACT_EDGE_WORDS`], then drop axis, tolerance,
/// covering part, flags, covering face color, and the interface surface record.
pub const CONTACT_ENTRY_WORDS: usize = CONTACT_WORD_SURFACE + SURFACE_RECORD_STRIDE / 4;
/// Words per edge of an entry: its start (two), its direction (two) and its allowance.
pub const CONTACT_EDGE_WORDS: usize = 5;
/// An edge's start, in words from the edge's first.
pub const CONTACT_EDGE_START: usize = 0;
/// An edge's direction, in words from the edge's first.
pub const CONTACT_EDGE_DIRECTION: usize = 2;
/// An edge's allowance, in words from the edge's first.
pub const CONTACT_EDGE_ALLOWANCE: usize = 4;
/// An entry's dropped axis, in words from the entry's first.
pub const CONTACT_WORD_DROP: usize = 3 * CONTACT_EDGE_WORDS;
/// An entry's tolerance, in words from the entry's first.
pub const CONTACT_WORD_TOLERANCE: usize = CONTACT_WORD_DROP + 1;
/// An entry's covering part, in words from the entry's first.
pub const CONTACT_WORD_PART: usize = CONTACT_WORD_DROP + 2;
/// An entry's flags, in words from the entry's first.
pub const CONTACT_WORD_FLAGS: usize = CONTACT_WORD_DROP + 3;
/// An entry's covering face color, in words from the entry's first.
pub const CONTACT_WORD_COLOR: usize = CONTACT_WORD_DROP + 4;
/// An entry's interface surface record, its words in [`SurfaceRecord`] order.
pub const CONTACT_WORD_SURFACE: usize = CONTACT_WORD_DROP + 5;
/// How many tolerances off the seam a continuing ray starts, as on the CPU.
pub const CONTACT_SKIP_TOLERANCES: f64 = crate::ray_tracer::path_tracer::SEAM_SKIP_TOLERANCES;
/// A start word's bit: the triangle's first entry covers the whole face.
pub const CONTACT_FULLY_COVERED: u32 = 1 << 31;
/// An entry's flag: the two glasses are the same surface, so the seam is no interface at all.
pub const CONTACT_SAME_GLASS: u32 = 1;
/// An entry's flag: the hit part's refractive index is at least the covering part's, so a path arriving at
/// the hit face's front (leaving the covering glass) goes into the denser or equal one.
pub const CONTACT_HIT_DENSER: u32 = 2;
/// An entry's flag: the covering part's refractive index is at least the hit part's, so a path arriving at
/// the hit face's back (leaving the hit glass) goes into the denser or equal one.
pub const CONTACT_COVER_DENSER: u32 = 4;
/// An entry's flag: one of the two surfaces could not be formed, so, as on the CPU, the covering
/// entry makes the hit an ordinary one and no later entry is tried.
pub const CONTACT_NO_SEAM: u32 = 8;

/// One edge of a contact triangle as the shader tests it; see the module header.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactEdge {
    pub start: [f32; 2],
    pub direction: [f32; 2],
    pub allowance: f32,
}

/// One contact triangle lying on a packed face; see the module header.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlassContactRecord {
    pub edges: [ContactEdge; 3],
    /// The axis the projection drops: 0 keeps `(y, z)`, 1 keeps `(z, x)`, 2 keeps `(x, y)`.
    pub drop_axis: u32,
    pub tolerance: f32,
    pub part: u32,
    pub flags: u32,
    /// The covering face's color, `0xBBGGRR` as in [`FaceRecord::color`].
    pub color: u32,
    pub interface: SurfaceRecord,
}

impl GlassContactRecord {
    /// This entry's words, in buffer order.
    pub fn to_words(&self) -> [u32; CONTACT_ENTRY_WORDS] {
        let mut words = [0u32; CONTACT_ENTRY_WORDS];
        for (at, edge) in self.edges.iter().enumerate() {
            let base = at * CONTACT_EDGE_WORDS;
            words[base + CONTACT_EDGE_START] = edge.start[0].to_bits();
            words[base + CONTACT_EDGE_START + 1] = edge.start[1].to_bits();
            words[base + CONTACT_EDGE_DIRECTION] = edge.direction[0].to_bits();
            words[base + CONTACT_EDGE_DIRECTION + 1] = edge.direction[1].to_bits();
            words[base + CONTACT_EDGE_ALLOWANCE] = edge.allowance.to_bits();
        }
        words[CONTACT_WORD_DROP] = self.drop_axis;
        words[CONTACT_WORD_TOLERANCE] = self.tolerance.to_bits();
        words[CONTACT_WORD_PART] = self.part;
        words[CONTACT_WORD_FLAGS] = self.flags;
        words[CONTACT_WORD_COLOR] = self.color;
        let surface = self.interface.to_le_bytes();
        for (at, chunk) in surface.as_chunks::<4>().0.iter().enumerate() {
            words[CONTACT_WORD_SURFACE + at] = u32::from_le_bytes(*chunk);
        }
        words
    }

    /// Whether `position`, in the scene-centre `f32` frame, lies on this contact triangle: the
    /// shader's point test, in `f32` in the shader's order, for checking the packed equations
    /// against the CPU's `f64` test without a graphics card.
    pub fn covers_f32(&self, position: [f32; 3]) -> bool {
        let point = match self.drop_axis {
            0 => [position[1], position[2]],
            1 => [position[2], position[0]],
            _ => [position[0], position[1]],
        };
        self.edges.iter().all(|edge| {
            let side = edge.direction[0] * (point[1] - edge.start[1])
                - edge.direction[1] * (point[0] - edge.start[0]);
            side >= -edge.allowance
        })
    }
}

/// The packed contacts: per-triangle starts and the entries. Both empty for a scene without any.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GpuGlassContacts {
    /// One word per triangle and the entry count, as in the module header; empty without contacts.
    pub starts: Vec<u32>,
    pub entries: Vec<GlassContactRecord>,
}

impl GpuGlassContacts {
    /// The block's words, appended to the part buffer: none when there are no contacts, else the
    /// triangle count, the starts and the entries.
    pub fn to_words(&self) -> Vec<u32> {
        if self.starts.is_empty() {
            return Vec::new();
        }
        let mut words = Vec::with_capacity(1 + self.starts.len());
        // The starts are one more than the triangles, whose count fits u32 (`FlatTree` limits it).
        words.push((self.starts.len() - 1) as u32);
        words.extend_from_slice(&self.starts);
        words.reserve(self.entries.len() * CONTACT_ENTRY_WORDS);
        for entry in &self.entries {
            words.extend_from_slice(&entry.to_words());
        }
        words
    }

    /// The buffer's bytes, little-endian.
    pub fn to_le_bytes(&self) -> Vec<u8> {
        self.to_words()
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .collect()
    }

    /// The entries of triangle `triangle` (its `source_index`) and whether its first covers it all.
    pub fn of_triangle(&self, triangle: usize) -> (&[GlassContactRecord], bool) {
        let (Some(&start), Some(&end)) = (self.starts.get(triangle), self.starts.get(triangle + 1))
        else {
            return (&[], false);
        };
        let first = (start & !CONTACT_FULLY_COVERED) as usize;
        let last = (end & !CONTACT_FULLY_COVERED) as usize;
        let entries = self.entries.get(first..last).unwrap_or(&[]);
        (entries, start & CONTACT_FULLY_COVERED != 0)
    }
}

/// Packs `scene`'s glass contacts against `faces` (recentred, offsets set) about `origin`, refused
/// when the part buffer with the block after it would pass `max_storage_binding`.
pub(super) fn pack_glass_contacts(
    scene: &RenderScene,
    faces: &[FaceRecord],
    origin: Vector3,
    max_storage_binding: u64,
) -> Result<GpuGlassContacts, FlatTreeError> {
    let contacts = scene.glass_contacts();
    if contacts.is_empty() {
        return Ok(GpuGlassContacts::default());
    }
    let parts = scene.parts().parts();
    let mut first_triangle = Vec::with_capacity(parts.len());
    let mut total = 0usize;
    for part in parts {
        first_triangle.push(total);
        total += part.surface().face_count();
    }
    let mut packed = GpuGlassContacts {
        starts: Vec::with_capacity(total + 1),
        entries: Vec::new(),
    };
    for (part_index, part) in parts.iter().enumerate() {
        let table = part.surface();
        let mesh_to_world = part.mesh_to_world();
        for face in 0..table.face_count() {
            let start = entry_index(packed.entries.len())?;
            let list = contacts.face_contacts(part_index, face);
            let triangle = first_triangle[part_index] + face;
            let fully = match (list.first(), table.face_corners(face)) {
                (Some(first), Some(local)) => local
                    .map(|corner| corner.transform_position(mesh_to_world))
                    .iter()
                    .all(|&corner| covers(first, corner)),
                _ => false,
            };
            packed.starts.push(if fully {
                start | CONTACT_FULLY_COVERED
            } else {
                start
            });
            for contact in list {
                let record = entry(
                    scene,
                    faces,
                    &first_triangle,
                    (part_index, triangle),
                    contact,
                    origin,
                )?;
                packed.entries.push(record);
            }
        }
    }
    packed.starts.push(entry_index(packed.entries.len())?);
    let part_words = (parts.len() * (PART_STRIDE / 4)) as u64;
    let bytes = (part_words
        + 1
        + packed.starts.len() as u64
        + (packed.entries.len() * CONTACT_ENTRY_WORDS) as u64)
        * 4;
    if bytes > max_storage_binding {
        return Err(FlatTreeError::PackedBufferTooLarge {
            buffer: "part (with its glass contacts)",
            needed_bytes: bytes,
            limit_bytes: max_storage_binding,
            what: "contact entries",
            largest_that_fits: (max_storage_binding / 4)
                .saturating_sub(part_words + 1 + packed.starts.len() as u64)
                / CONTACT_ENTRY_WORDS as u64,
        });
    }
    Ok(packed)
}

/// An entry index as a start word, refused where it would reach the fully-covered bit.
fn entry_index(count: usize) -> Result<u32, FlatTreeError> {
    u32::try_from(count)
        .ok()
        .filter(|&index| index & CONTACT_FULLY_COVERED == 0)
        .ok_or(FlatTreeError::NotRepresentable {
            reason: "more glass contact entries than a 31-bit start word can index",
        })
}

/// The record for `contact` lying on triangle `triangle` of part `part`.
fn entry(
    scene: &RenderScene,
    faces: &[FaceRecord],
    first_triangle: &[usize],
    (part, triangle): (usize, usize),
    contact: &ContactFace,
    origin: Vector3,
) -> Result<GlassContactRecord, FlatTreeError> {
    let parts = scene.parts().parts();
    let covering = parts
        .get(contact.part)
        .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
    let hit_face = faces
        .get(triangle)
        .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
    let covering_triangle = first_triangle
        .get(contact.part)
        .map(|&first| first + contact.face)
        .and_then(|at| faces.get(at))
        .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
    let tolerance = contact
        .tolerance
        .max(f64::from(hit_face.ray_offset))
        .max(f64::from(covering_triangle.ray_offset));
    let [c0, c1, c2] = contact.corners;
    let drop = dominant_axis((c1 - c0).cross(c2 - c0));
    // Recentred in f64 before anything narrows, as the packed triangles are.
    let flat = contact.corners.map(|corner| flatten(corner - origin, drop));
    let area = (flat[1][0] - flat[0][0]) * (flat[2][1] - flat[0][1])
        - (flat[1][1] - flat[0][1]) * (flat[2][0] - flat[0][0]);
    let sign = if area > 0.0 { 1.0 } else { -1.0 };
    let edges = std::array::from_fn(|edge| {
        let start = flat[edge];
        let end = flat[(edge + 1) % 3];
        let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
        let allowance = if contact.widened[edge] {
            tolerance * (dx * dx + dy * dy).sqrt()
        } else {
            0.0
        };
        // `as` rounds to nearest: recentred corners are within the f32 range the pack checked.
        ContactEdge {
            start: [start[0] as f32, start[1] as f32],
            direction: [(dx * sign) as f32, (dy * sign) as f32],
            allowance: allowance as f32,
        }
    });
    let covering_color = covering.surface().face_color(contact.face);
    let hit_color = parts
        .get(part)
        .map(|placed| placed.surface().face_color(triangle - first_triangle[part]))
        .ok_or(FlatTreeError::UnexpectedBvhLeaf)?;
    let surfaces = (
        scene.surface_at(part, hit_color),
        scene.surface_at(contact.part, covering_color),
    );
    let (flags, interface) = match surfaces {
        (Some(here), Some(there)) => {
            let mut flags = 0;
            if here == there {
                flags |= CONTACT_SAME_GLASS;
            }
            if here.ior() >= there.ior() {
                flags |= CONTACT_HIT_DENSER;
            }
            if there.ior() >= here.ior() {
                flags |= CONTACT_COVER_DENSER;
            }
            (
                flags,
                SurfaceRecord::from_surface(&seam_surface(&here, &there)),
            )
        }
        _ => (
            CONTACT_NO_SEAM,
            SurfaceRecord {
                base_color: [0.0; 3],
                metallic: 0.0,
                roughness: 0.0,
                transmission: 0.0,
                ior: 0.0,
                ior_minus_one: 0.0,
                flags: 0,
            },
        ),
    };
    Ok(GlassContactRecord {
        edges,
        drop_axis: drop as u32,
        tolerance: tolerance as f32,
        // Part indices fit u32: the part buffer's size was checked against a u64 binding limit.
        part: contact.part as u32,
        flags,
        color: u32::from(covering_color.red)
            | u32::from(covering_color.green) << 8
            | u32::from(covering_color.blue) << 16,
        interface,
    })
}
