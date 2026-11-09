//! The flat tree's GPU side: the WGSL compute traversal [`TRACE_WGSL`] (`trace.wgsl`, a function
//! by function mirror of `traverse`), and the two records the shader adds to the tree's own
//! buffers: a ray and a hit. Callers give the shader these bytes and read hits back through
//! [`FlatRay::to_le_bytes`] and [`hit_from_le_bytes`]; a test pins the bytes to the WGSL structs.
//!
//! Bindings, all in group 0: 0 the node buffer, 1 the triangle buffer, 2 the rays, 3 the hits
//! (`read_write`, one per ray, stride [`HIT_STRIDE`]), 4 a uniform `{ first_ray: u32, ray_count:
//! u32 }`, the slice of the ray buffer one dispatch covers. Entry points are `trace_closest_main`
//! and `trace_any_main`, workgroup size 64. Any-hit stops at the first hit it finds, so its hit
//! record only says whether the ray was blocked and by what, never by what is nearest.
//!
//! The ray is padded out to [`RAY_STRIDE`] because a `vec3<f32>` aligns to 16 bytes in a storage
//! buffer.

use super::traverse::{FlatHit, FlatRay};

/// The WGSL source of the compute traversal.
pub const TRACE_WGSL: &str = include_str!("trace.wgsl");

/// A ray record's size in the shader's `rays` buffer.
pub const RAY_STRIDE: usize = 48;

/// A hit record's size in the shader's `hits` buffer.
pub const HIT_STRIDE: usize = 28;

/// A hit record's `triangle` when the ray found nothing.
pub const NO_HIT: u32 = 0xFFFF_FFFF;

impl FlatRay {
    /// This ray's record for the shader's ray buffer, its bytes in buffer order.
    pub fn to_le_bytes(&self) -> [u8; RAY_STRIDE] {
        let mut bytes = [0u8; RAY_STRIDE];
        let words: [u32; 9] = [
            self.origin[0].to_bits(),
            self.origin[1].to_bits(),
            self.origin[2].to_bits(),
            self.t_min.to_bits(),
            self.direction[0].to_bits(),
            self.direction[1].to_bits(),
            self.direction[2].to_bits(),
            self.t_max.to_bits(),
            self.face_mask,
        ];
        for (index, word) in words.iter().enumerate() {
            bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        bytes
    }
}

/// This hit's record in the shader's hit buffer, a miss for `None`.
pub fn hit_to_le_bytes(hit: Option<&FlatHit>) -> [u8; HIT_STRIDE] {
    let (t, triangle, source_index, front_face, u, v, w) = match hit {
        Some(hit) => (
            hit.t,
            hit.triangle,
            hit.source_index,
            u32::from(hit.front_face),
            hit.barycentric[0],
            hit.barycentric[1],
            hit.barycentric[2],
        ),
        None => (0.0, NO_HIT, 0, 0, 0.0, 0.0, 0.0),
    };
    let words = [
        t.to_bits(),
        triangle,
        source_index,
        front_face,
        u.to_bits(),
        v.to_bits(),
        w.to_bits(),
    ];
    let mut bytes = [0u8; HIT_STRIDE];
    for (index, word) in words.iter().enumerate() {
        bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
    bytes
}

/// The hit a record holds, or `None` for a miss.
pub fn hit_from_le_bytes(bytes: &[u8; HIT_STRIDE]) -> Option<FlatHit> {
    let word = |index: usize| {
        u32::from_le_bytes([
            bytes[index * 4],
            bytes[index * 4 + 1],
            bytes[index * 4 + 2],
            bytes[index * 4 + 3],
        ])
    };
    let triangle = word(1);
    if triangle == NO_HIT {
        return None;
    }
    Some(FlatHit {
        t: f32::from_bits(word(0)),
        triangle,
        source_index: word(2),
        barycentric: [
            f32::from_bits(word(4)),
            f32::from_bits(word(5)),
            f32::from_bits(word(6)),
        ],
        front_face: word(3) != 0,
    })
}
