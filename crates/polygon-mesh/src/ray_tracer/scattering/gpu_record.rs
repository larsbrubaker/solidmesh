//! The scattering model's plain buffers for a WebGPU compute shader: [`SurfaceRecord`] (one
//! surface, the bytes the `Surface` struct of `flat_tree/scatter.wgsl` reads) and
//! [`albedo_table_bytes`] (the precomputed energy table, `AlbedoEntry` records). Written as `f32`
//! and `u32` field by field in little-endian order, in the style of `flat_tree/layout.rs`.
//!
//! The narrowing from `f64` happens only here, once, and only what a shader needs crosses it. The
//! branch points of the model are decided in `f64` and stored as [`SurfaceRecord::flags`]: a
//! roughness or index of refraction within an `f32` rounding of a threshold must take the same
//! branch on the GPU as on the CPU. The table carries `1 - A - B` as well as `A` and `B`
//! because in `f32` the difference of the two is mostly noise at the narrow end of the lobe, where
//! the true value is around 1e-5 (the shader's header says where that matters).
//!
//! `ior_minus_one` is stored as well as `ior`: glass near index 1 is made of that small step, and
//! subtracting two `f32`s would leave only a few of its bits.
//!
//! The table holds [`ALBEDO_TABLE_SIZE`] rows of [`ALBEDO_TABLE_SIZE`] cells, in row order, then
//! one mean entry per row, so a cell's index is `row * 32 + column` and a row mean's is
//! `32 * 32 + row`. Entries are `[A, B, loss]`.

use super::albedo::ALBEDO_TABLE_SIZE;
use super::albedo_table::{ALBEDO, MEAN_ALBEDO};
use super::microfacet::{INDEX_MATCHED, MIN_ALPHA};
use super::surface::ScatteringSurface;

/// A surface record's size in bytes.
pub const SURFACE_RECORD_STRIDE: usize = 36;

/// An albedo table entry's size in bytes.
pub const ALBEDO_ENTRY_STRIDE: usize = 12;

/// How many entries the albedo buffer holds: the grid, then the row means.
pub const ALBEDO_ENTRY_COUNT: usize = ALBEDO_TABLE_SIZE * ALBEDO_TABLE_SIZE + ALBEDO_TABLE_SIZE;

/// [`SurfaceRecord::flags`]: the surface is smooth, a delta lobe.
pub const SURFACE_SMOOTH: u32 = 1;
/// [`SurfaceRecord::flags`]: glass whose index of refraction is too close to 1 for a microfacet.
pub const SURFACE_INDEX_MATCHED: u32 = 2;
/// [`SurfaceRecord::flags`]: the surface has a glass share at all.
pub const SURFACE_HAS_GLASS: u32 = 4;

/// One surface for a shader: the fields of a [`ScatteringSurface`] narrowed to `f32`, and its
/// branch points decided in `f64`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct SurfaceRecord {
    pub base_color: [f32; 3],
    pub metallic: f32,
    pub roughness: f32,
    pub transmission: f32,
    pub ior: f32,
    /// `ior - 1`, kept apart: the index step is what glass near index 1 is made of, and subtracting
    /// two f32s would leave only a few bits of it.
    pub ior_minus_one: f32,
    pub flags: u32,
}

impl SurfaceRecord {
    /// The record for `surface`.
    pub fn from_surface(surface: &ScatteringSurface) -> SurfaceRecord {
        let mut flags = 0;
        if surface.roughness * surface.roughness < MIN_ALPHA {
            flags |= SURFACE_SMOOTH;
        }
        if surface.ior - 1.0 < INDEX_MATCHED {
            flags |= SURFACE_INDEX_MATCHED;
        }
        if (1.0 - surface.metallic) * surface.transmission > 0.0 {
            flags |= SURFACE_HAS_GLASS;
        }
        SurfaceRecord {
            base_color: surface.base_color.map(|channel| channel as f32),
            metallic: surface.metallic as f32,
            roughness: surface.roughness as f32,
            transmission: surface.transmission as f32,
            ior: surface.ior as f32,
            ior_minus_one: (surface.ior - 1.0) as f32,
            flags,
        }
    }

    /// This record's bytes, in buffer order.
    pub fn to_le_bytes(&self) -> [u8; SURFACE_RECORD_STRIDE] {
        let words = [
            self.base_color[0].to_bits(),
            self.base_color[1].to_bits(),
            self.base_color[2].to_bits(),
            self.metallic.to_bits(),
            self.roughness.to_bits(),
            self.transmission.to_bits(),
            self.ior.to_bits(),
            self.ior_minus_one.to_bits(),
            self.flags,
        ];
        let mut bytes = [0u8; SURFACE_RECORD_STRIDE];
        for (index, word) in words.iter().enumerate() {
            bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        bytes
    }
}

/// The albedo table's buffer: [`ALBEDO_ENTRY_COUNT`] entries of [`ALBEDO_ENTRY_STRIDE`] bytes, the
/// grid then the row means. Pure function of the generated tables.
pub fn albedo_table_bytes() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(ALBEDO_ENTRY_COUNT * ALBEDO_ENTRY_STRIDE);
    let mut push_entry = |[a, b]: [f64; 2]| {
        for value in [a as f32, b as f32, (1.0 - a - b) as f32] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
    };
    for row in ALBEDO.iter() {
        for &cell in row.iter() {
            push_entry(cell);
        }
    }
    for &mean in MEAN_ALBEDO.iter() {
        push_entry(mean);
    }
    bytes
}
