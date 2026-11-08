// Rust-only checks for `fixed_tie_sort`: the tie order .NET 10's `List<T>.Sort(IComparer)` leaves,
// captured by running that sort on the same inputs (keys `(i * 7) % 3` for several lengths, then
// 40 equal keys). The BVH builders depend on this order matching.

use solidmesh_math::fixed_tie_sort::introspective_sort;

fn sorted_indices(keys: impl Fn(usize) -> i32, n: usize) -> Vec<usize> {
    let mut items: Vec<(i32, usize)> = (0..n).map(|i| (keys(i), i)).collect();
    introspective_sort(&mut items, &mut |a, b| a.0.cmp(&b.0));
    items.iter().map(|item| item.1).collect()
}

#[test]
fn rust_only_introsort_leaves_ties_in_their_pinned_places() {
    let expected: [&[usize]; 4] = [
        &[0, 3, 1, 4, 2],
        &[0, 12, 3, 9, 6, 15, 13, 10, 16, 4, 1, 7, 5, 11, 2, 14, 8],
        &[
            0, 36, 33, 30, 27, 24, 21, 18, 15, 12, 39, 9, 3, 6, 25, 37, 1, 34, 31, 28, 4, 10, 19,
            16, 13, 7, 22, 14, 35, 8, 2, 32, 23, 29, 38, 26, 20, 5, 17, 11,
        ],
        &[
            0, 117, 114, 111, 108, 105, 102, 198, 96, 93, 90, 87, 84, 81, 78, 75, 72, 69, 66, 63,
            60, 57, 120, 123, 126, 129, 195, 192, 189, 186, 183, 180, 177, 174, 171, 168, 54, 165,
            159, 156, 153, 150, 147, 144, 141, 138, 135, 132, 162, 51, 99, 15, 27, 33, 24, 21, 36,
            18, 39, 12, 42, 9, 45, 30, 3, 48, 6, 19, 139, 136, 190, 193, 133, 22, 130, 127, 142,
            124, 25, 1, 121, 118, 196, 115, 112, 28, 178, 148, 7, 175, 172, 181, 169, 10, 166, 184,
            145, 163, 160, 13, 157, 154, 109, 151, 187, 16, 4, 49, 199, 106, 88, 37, 85, 82, 79,
            76, 73, 91, 70, 67, 64, 61, 46, 58, 55, 52, 43, 94, 40, 103, 31, 97, 34, 100, 23, 179,
            65, 176, 44, 8, 173, 116, 68, 170, 32, 101, 62, 104, 5, 167, 113, 185, 59, 188, 47, 56,
            191, 2, 29, 194, 53, 110, 197, 182, 71, 164, 128, 95, 131, 35, 92, 134, 89, 137, 20,
            50, 140, 86, 143, 98, 38, 146, 17, 83, 119, 74, 161, 41, 26, 158, 11, 77, 155, 107,
            152, 80, 122, 149, 14, 125,
        ],
    ];
    for (n, expected) in [5, 17, 40, 200].into_iter().zip(expected) {
        assert_eq!(
            sorted_indices(|i| ((i * 7) % 3) as i32, n),
            expected,
            "length {n}"
        );
    }

    assert_eq!(
        sorted_indices(|_| 0, 40),
        vec![
            0, 22, 23, 24, 25, 26, 27, 28, 21, 29, 31, 32, 33, 34, 35, 36, 37, 30, 20, 19, 18, 1,
            2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 38, 39
        ]
    );
}
