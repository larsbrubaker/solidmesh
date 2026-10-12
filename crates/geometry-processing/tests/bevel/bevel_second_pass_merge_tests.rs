//! Port of `R/Tests/GeometryProcessingTests/BevelSecondPassMergeTests.cs`: merging the creases the
//! second pass re-reads into runs ([`merge`]) and which members a run's swept reading answers for
//! ([`longest_reading_and_what_it_covers`]). The C# tests that cut the nose-in-notch ridge and
//! re-bead it (`BevelSecondPass`) land with that type.

use geometry_processing::bevel::bevel_crease_merge::merge;
use geometry_processing::bevel::bevel_second_pass_run_plan::longest_reading_and_what_it_covers;

fn readings(lists: &[Option<&[i32]>]) -> Vec<Option<Vec<i32>>> {
    lists
        .iter()
        .map(|list| list.map(|edges| edges.to_vec()))
        .collect()
}

fn sorted(mut list: Vec<usize>) -> Vec<usize> {
    list.sort();
    list
}

#[test]
fn a_crease_shared_through_a_middle_one_is_all_one_run() {
    let merged = merge(&readings(&[
        Some(&[10, 11]),
        Some(&[30, 31]),
        Some(&[11, 12, 30]),
    ]));

    // The third crease shares an edge with each of the other two, so all three are one crease.
    assert_eq!(merged.len(), 1);
    assert_eq!(merged[0], vec![0, 1, 2]);
}

#[test]
fn creases_that_share_nothing_stay_apart_and_keep_their_order() {
    let merged = merge(&readings(&[Some(&[5]), None, Some(&[6]), Some(&[5, 7])]));

    assert_eq!(merged.len(), 2);
    // The last shares an edge with the first, and the run is named by the earliest in it.
    assert_eq!(merged[0], vec![0, 3]);
    // And the one that shares nothing is its own run - the null taking no part at all.
    assert_eq!(merged[1], vec![2]);
}

#[test]
fn no_member_of_a_merged_run_loses_its_bead_to_a_reading_that_does_not_cover_it() {
    let readings_ = readings(&[Some(&[10, 11]), Some(&[11, 12, 30]), Some(&[30, 31])]);

    let merged = merge(&readings_);

    // A shares an edge with B and B with C, so the three of them are one run.
    assert_eq!(merged.len(), 1);

    let (longest, covered, not_covered) =
        longest_reading_and_what_it_covers(&merged[0], &readings_);

    // B reads the most of the crease, so B's reading is the one swept.
    assert_eq!(longest, 1);
    // Only the member the swept chain really carries may have its first-pass bead withdrawn.
    assert_eq!(covered, vec![1]);
    // A and C each read an edge B never touches, so both keep the bead the first pass built.
    assert_eq!(sorted(not_covered.clone()), vec![0, 2]);
    // Every member is accounted for one way or the other - a member in neither list is a bead
    // deleted in silence.
    let mut all = covered.clone();
    all.extend(not_covered);
    assert_eq!(sorted(all), vec![0, 1, 2]);

    let inside = readings(&[Some(&[11, 12]), Some(&[11, 12, 30]), Some(&[30, 31])]);

    let (_, also_covered, still_kept) =
        longest_reading_and_what_it_covers(&merge(&inside)[0], &inside);

    // The first reading is wholly inside the one being swept, so that bead really is replaced.
    assert_eq!(sorted(also_covered), vec![0, 1]);
    assert_eq!(still_kept, vec![2]);
}
