//! .NET `System.Random` seeded with a fixed value (`new Random(seed)`, the legacy
//! `Net5CompatSeedImpl`: Knuth's subtractive generator). MIConvexHull picks its starting simplex
//! with `new Random(1)`, so the hull's vertex choice depends on reproducing this sequence exactly.
//!
//! A copy of the seeded half of `mattercad-expressions`' `net_random.rs`, which polygon-mesh
//! cannot depend on (it sits above this crate). Ports .NET Runtime's `CompatPrng` (MIT, see
//! THIRD_PARTY_NOTICES.md).

const MBIG: i32 = i32::MAX;
const MSEED: i32 = 161803398;

/// `new Random(seed)`.
pub(super) struct NetRandom {
    seed_array: [i32; 56],
    inext: usize,
    inextp: usize,
}

impl NetRandom {
    pub(super) fn new(seed: i32) -> NetRandom {
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
        NetRandom {
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

    /// `Next(maxValue)`: `(int)(Sample() * maxValue)`, in [0, max_value).
    pub(super) fn next(&mut self, max_value: i32) -> i32 {
        let sample = self.internal_sample() as f64 * (1.0 / MBIG as f64);
        (sample * max_value as f64) as i32
    }
}

#[cfg(test)]
mod tests {
    use super::NetRandom;

    #[test]
    fn rust_only_seeded_sequence_matches_dotnet() {
        // `new Random(1).Next(100)` x5 and `new Random(1).Next(7)` x5 from .NET 10.
        let mut r = NetRandom::new(1);
        let a: Vec<i32> = (0..5).map(|_| r.next(100)).collect();
        let mut r = NetRandom::new(1);
        let b: Vec<i32> = (0..5).map(|_| r.next(7)).collect();
        assert_eq!(a, NET_100);
        assert_eq!(b, NET_7);
    }

    const NET_100: [i32; 5] = [24, 11, 46, 77, 65];
    const NET_7: [i32; 5] = [1, 0, 3, 5, 4];
}
