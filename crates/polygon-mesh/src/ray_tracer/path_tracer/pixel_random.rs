//! [`PixelRandom`], the random stream of one sample of one pixel, and [`pixel_jitter`], where in
//! the pixel that sample's camera ray goes. Both are pure functions of a seed, the pixel and the
//! sample index: nothing about tile size, tile order or thread reaches them, which is what makes
//! the render bit-identical however the work is sliced.
//!
//! The stream is SplitMix64, a 64-bit counter passed through a strong mixer. The start of the
//! stream folds the seed, pixel and sample index through that same mixer, one word at a time, so
//! neighbouring pixels and consecutive samples start at unrelated states.

/// The counter step of the stream (the golden ratio in 64 bits, which visits every state).
const COUNTER_STEP: u64 = 0x9e37_79b9_7f4a_7c15;

/// The largest value below 1, so a draw is always in `[0, 1)`.
const ALMOST_ONE: f64 = 1.0 - f64::EPSILON / 2.0;

/// The stream word that gives a pixel its jitter offset: separate from any sample's own stream,
/// so the jitter doesn't reuse the random numbers its path scatters with.
const JITTER_STREAM: u32 = u32::MAX;

/// The base of the two-dimensional low-discrepancy sequence the jitter steps along: the plastic
/// number, the 2D analogue of the golden ratio.
const PLASTIC_NUMBER: f64 = 1.324_717_957_244_746;

/// Mixes the bits of `z` so that every input bit affects every output bit.
fn mix(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// The random numbers of one sample of one pixel.
#[derive(Debug, Clone)]
pub struct PixelRandom {
    state: u64,
}

impl PixelRandom {
    /// The stream of `sample_index` at pixel (`x`, `y`) under `seed`.
    pub fn for_pixel(seed: u64, x: u32, y: u32, sample_index: u32) -> Self {
        let mut state = mix(seed.wrapping_add(COUNTER_STEP));
        for word in [x, y, sample_index] {
            state = mix((state ^ u64::from(word)).wrapping_add(COUNTER_STEP));
        }
        PixelRandom { state }
    }

    /// The next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(COUNTER_STEP);
        mix(self.state)
    }

    /// A number in `[0, 1)`.
    pub fn next_unit(&mut self) -> f64 {
        // 53 bits fit an `f64` exactly, so the conversion cannot round up to 1.
        let bits = (self.next_u64() >> 11) as f64;
        (bits / 9_007_199_254_740_992.0).min(ALMOST_ONE)
    }

    /// Three numbers in `[0, 1)`.
    pub fn next_unit3(&mut self) -> [f64; 3] {
        [self.next_unit(), self.next_unit(), self.next_unit()]
    }
}

/// The fractional part of `value`.
fn fraction(value: f64) -> f64 {
    (value - value.floor()).min(ALMOST_ONE)
}

/// Where sample `sample_index` of pixel (`x`, `y`) falls inside that pixel, each coordinate in
/// `[0, 1)`. A low-discrepancy sequence covers the pixel evenly as samples accumulate, and a
/// per-pixel offset keeps neighbouring pixels from sharing a pattern.
pub fn pixel_jitter(seed: u64, x: u32, y: u32, sample_index: u32) -> (f64, f64) {
    let mut offset = PixelRandom::for_pixel(seed, x, y, JITTER_STREAM);
    let (offset_x, offset_y) = (offset.next_unit(), offset.next_unit());
    let index = f64::from(sample_index);
    (
        fraction(offset_x + index / PLASTIC_NUMBER),
        fraction(offset_y + index / (PLASTIC_NUMBER * PLASTIC_NUMBER)),
    )
}
