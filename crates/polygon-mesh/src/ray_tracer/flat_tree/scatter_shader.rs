//! The scattering model's GPU side: the WGSL library [`SCATTER_WGSL`] (`scatter.wgsl`, a function
//! by function mirror of `scattering`'s `evaluate`, `pdf` and `sample`) and the compute entry
//! point [`SCATTER_CHECK_WGSL`] that runs it over fixed cases, with the two records that entry
//! point reads and writes. The surface and albedo buffers are in `scattering/gpu_record.rs`
//! (re-exported from `flat_tree`). `SCATTER_WGSL` has no entry point, so an integrator includes it;
//! its albedo binding is group 1 binding 0, clear of the traversal's group 0.
//!
//! The check's bindings are listed at the top of `scatter_check.wgsl`, entry point
//! `scatter_check_main`, workgroup size 64. A test pins these bytes to the WGSL structs.

/// The WGSL source of the scattering model: `surface_evaluate`, `surface_pdf`, `surface_sample`.
pub const SCATTER_WGSL: &str = include_str!("scatter.wgsl");

/// The WGSL source of the check: [`SCATTER_WGSL`] with its entry point.
pub const SCATTER_CHECK_WGSL: &str = concat!(
    include_str!("scatter.wgsl"),
    include_str!("scatter_check.wgsl")
);

/// A case record's size in the check's `cases` buffer.
pub const SCATTER_CASE_STRIDE: usize = 40;

/// A result record's size in the check's `results` buffer.
pub const SCATTER_RESULT_STRIDE: usize = 48;

/// A result record's `flags`: `surface_sample` found a direction.
pub const SAMPLE_FOUND: u32 = 1;
/// A result record's `flags`: the sampled event was specular.
pub const SAMPLE_SPECULAR: u32 = 2;

/// One case of the scattering check: a surface (index into the surfaces buffer), `wo`, `wi`, and
/// the three random numbers for `sample`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScatterCheckCase {
    pub surface: u32,
    pub wo: [f32; 3],
    pub wi: [f32; 3],
    pub u: [f32; 3],
}

impl ScatterCheckCase {
    /// This case's record, its bytes in buffer order.
    pub fn to_le_bytes(&self) -> [u8; SCATTER_CASE_STRIDE] {
        let words = [
            self.surface,
            self.wo[0].to_bits(),
            self.wo[1].to_bits(),
            self.wo[2].to_bits(),
            self.wi[0].to_bits(),
            self.wi[1].to_bits(),
            self.wi[2].to_bits(),
            self.u[0].to_bits(),
            self.u[1].to_bits(),
            self.u[2].to_bits(),
        ];
        let mut bytes = [0u8; SCATTER_CASE_STRIDE];
        for (index, word) in words.iter().enumerate() {
            bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        bytes
    }
}

/// One answer of the scattering check.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ScatterCheckResult {
    pub evaluate: [f32; 3],
    pub pdf: f32,
    pub sample_direction: [f32; 3],
    pub sample_pdf: f32,
    pub sample_weight: [f32; 3],
    /// [`SAMPLE_FOUND`] and [`SAMPLE_SPECULAR`].
    pub flags: u32,
}

impl ScatterCheckResult {
    /// This result's record, its bytes in buffer order.
    pub fn to_le_bytes(&self) -> [u8; SCATTER_RESULT_STRIDE] {
        let words = [
            self.evaluate[0].to_bits(),
            self.evaluate[1].to_bits(),
            self.evaluate[2].to_bits(),
            self.pdf.to_bits(),
            self.sample_direction[0].to_bits(),
            self.sample_direction[1].to_bits(),
            self.sample_direction[2].to_bits(),
            self.sample_pdf.to_bits(),
            self.sample_weight[0].to_bits(),
            self.sample_weight[1].to_bits(),
            self.sample_weight[2].to_bits(),
            self.flags,
        ];
        let mut bytes = [0u8; SCATTER_RESULT_STRIDE];
        for (index, word) in words.iter().enumerate() {
            bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        bytes
    }

    /// The result a record holds.
    pub fn from_le_bytes(bytes: &[u8; SCATTER_RESULT_STRIDE]) -> ScatterCheckResult {
        let word = |index: usize| {
            u32::from_le_bytes([
                bytes[index * 4],
                bytes[index * 4 + 1],
                bytes[index * 4 + 2],
                bytes[index * 4 + 3],
            ])
        };
        let float = |index: usize| f32::from_bits(word(index));
        ScatterCheckResult {
            evaluate: [float(0), float(1), float(2)],
            pdf: float(3),
            sample_direction: [float(4), float(5), float(6)],
            sample_pdf: float(7),
            sample_weight: [float(8), float(9), float(10)],
            flags: word(11),
        }
    }
}
