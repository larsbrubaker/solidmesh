//! A seeded random generator: Knuth's subtractive generator, derived from the .NET runtime's
//! `CompatPrng` (MIT, see THIRD_PARTY_NOTICES.md). A given seed always draws the same numbers,
//! so a saved design that uses `rand(seed)` in an expression rebuilds to the same shape, and
//! the convex hull's starting simplex and the test fixtures stay fixed. The unseeded generator
//! lives in mattercad-expressions, its only user.

const MBIG: i32 = i32::MAX;
const MSEED: i32 = 161803398;

/// `new Random(seed)`.
pub struct DotNetRandom {
    seed_array: [i32; 56],
    inext: usize,
    inextp: usize,
}

impl DotNetRandom {
    pub fn new(seed: i32) -> DotNetRandom {
        let mut seed_array = [0i32; 56];
        let subtraction = if seed == i32::MIN {
            i32::MAX
        } else {
            seed.abs()
        };
        let mut mj = MSEED - subtraction;
        seed_array[55] = mj;
        let mut mk = 1i32;
        let mut ii = 0usize;
        for _ in 1..55 {
            ii += 21;
            if ii >= 55 {
                ii -= 55;
            }
            seed_array[ii] = mk;
            mk = mj.wrapping_sub(mk);
            if mk < 0 {
                mk = mk.wrapping_add(MBIG);
            }
            mj = seed_array[ii];
        }
        for _ in 1..5 {
            for i in 1..56 {
                let mut n = i + 30;
                if n >= 55 {
                    n -= 55;
                }
                seed_array[i] = seed_array[i].wrapping_sub(seed_array[1 + n]);
                if seed_array[i] < 0 {
                    seed_array[i] = seed_array[i].wrapping_add(MBIG);
                }
            }
        }
        DotNetRandom {
            seed_array,
            inext: 0,
            inextp: 21,
        }
    }

    fn internal_sample(&mut self) -> i32 {
        let mut loc_inext = self.inext + 1;
        if loc_inext >= 56 {
            loc_inext = 1;
        }
        let mut loc_inextp = self.inextp + 1;
        if loc_inextp >= 56 {
            loc_inextp = 1;
        }
        let mut ret = self.seed_array[loc_inext].wrapping_sub(self.seed_array[loc_inextp]);
        if ret == MBIG {
            ret -= 1;
        }
        if ret < 0 {
            ret = ret.wrapping_add(MBIG);
        }
        self.seed_array[loc_inext] = ret;
        self.inext = loc_inext;
        self.inextp = loc_inextp;
        ret
    }

    /// `NextDouble()` (`Sample()`): [0, 1).
    pub fn next_double(&mut self) -> f64 {
        self.internal_sample() as f64 * (1.0 / MBIG as f64)
    }

    /// A number in [0, max_value): `next_double() * max_value`, truncated. `next_double()` is
    /// below 1, so the product never reaches `max_value`.
    pub fn next(&mut self, max_value: i32) -> i32 {
        (self.next_double() * max_value as f64) as i32
    }
}
