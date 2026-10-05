// Rust-only checks for `vector_math::dotnet_random`: the seeded sequences match .NET 10's
// `new Random(seed)` (values printed by a .NET 10 console program, `NextDouble().ToString("R")`).

use vector_math::dotnet_random::DotNetRandom;

#[test]
fn rust_only_seeded_sequence_matches_dotnet() {
    // `new Random(1).Next(100)` x5 and `new Random(1).Next(7)` x5 from .NET 10.
    let mut r = DotNetRandom::new(1);
    let a: Vec<i32> = (0..5).map(|_| r.next(100)).collect();
    let mut r = DotNetRandom::new(1);
    let b: Vec<i32> = (0..5).map(|_| r.next(7)).collect();
    assert_eq!(a, [24, 11, 46, 77, 65]);
    assert_eq!(b, [1, 0, 3, 5, 4]);
}

#[test]
fn rust_only_seeded_next_double_matches_dotnet() {
    let mut r = DotNetRandom::new(42);
    let values: Vec<f64> = (0..5).map(|_| r.next_double()).collect();
    assert_eq!(
        values,
        [
            0.6681064659115423,
            0.14090729837348093,
            0.12551828945312568,
            0.5227642760252413,
            0.16843422416990353,
        ]
    );
}

#[test]
fn rust_only_negative_and_min_seeds_match_dotnet() {
    // A negative seed uses its absolute value; `int.MinValue` (no absolute value) uses
    // `int.MaxValue`.
    let mut r = DotNetRandom::new(-7);
    assert_eq!(r.next_double(), 0.38322046929189024);
    assert_eq!(r.next_double(), 0.8712556827213874);
    assert_eq!(r.next_double(), 0.6609386227377405);
    let mut r = DotNetRandom::new(i32::MIN);
    assert_eq!(r.next_double(), 0.7262432699679598);
    assert_eq!(r.next_double(), 0.8173253595909687);
    assert_eq!(r.next_double(), 0.7680226921886312);
}
