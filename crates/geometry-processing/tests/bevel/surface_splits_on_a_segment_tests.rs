//! Port of `SurfaceSplitsOnASegmentTests.cs`: the reading of a stretch has to STOP, and a cluster of
//! splits finer than the ruler must come back saying it did not settle.
//!
//! C#'s two nested test classes are the two modules below, with C#'s test names.

/// C# `SurfaceSplitsOnASegmentTests.ACrowdedClusterOfSplits`.
mod a_crowded_cluster_of_splits {
    use std::collections::HashSet;

    use geometry_processing::bevel::surface::{SurfaceSplitsOnASegment, SurfaceVertexPool};
    use vector_math::Vector3Float;

    /// Three splits in a cluster finer than the ruler, on a long side: the reading must come back, and
    /// must say it did not settle rather than handing out a stretch that repeats a vertex.
    #[test]
    fn the_reading_comes_back_and_says_it_did_not_settle() {
        // A part a metre across, so the ruler is a hundredth of a millimetre.
        let vertices = vec![
            Vector3Float::new(0.0, 0.0, 0.0),
            Vector3Float::new(1000.0, 0.0, 0.0),
            Vector3Float::new(500.0, 0.0, 0.0),
            Vector3Float::new(500.002, 0.003, 0.0),
            Vector3Float::new(500.004, -0.001, 0.0),
        ];

        let pool = SurfaceVertexPool::new(&vertices);

        assert!(
            pool.near_enough() > 0.005,
            "the cluster is a few thousandths across and has to read as one crowd"
        );

        let mut standing_on = SurfaceSplitsOnASegment::new(&pool, vec![2, 3, 4]);

        let along = standing_on.between(0, 1);

        assert!(
            !standing_on.settled,
            "the three of them cannot be put in an order that settles"
        );

        let mut seen = HashSet::new();

        for vertex in along {
            assert!(
                seen.insert(vertex),
                "a stretch that visits one vertex twice is a ring nobody can sew"
            );
        }
    }
}

/// C# `SurfaceSplitsOnASegmentTests.SplitsWellApart`.
mod splits_well_apart {
    use geometry_processing::bevel::surface::{SurfaceSplitsOnASegment, SurfaceVertexPool};
    use vector_math::Vector3Float;

    /// The ordinary reading, kept beside the crowded one: splits comfortably apart on a side are handed
    /// back in order along it, and the reading says it settled.
    #[test]
    fn are_handed_back_in_order_and_the_reading_settles() {
        let vertices = vec![
            Vector3Float::new(0.0, 0.0, 0.0),
            Vector3Float::new(1000.0, 0.0, 0.0),
            Vector3Float::new(750.0, 0.0, 0.0),
            Vector3Float::new(250.0, 0.0, 0.0),
        ];

        let pool = SurfaceVertexPool::new(&vertices);

        let mut standing_on = SurfaceSplitsOnASegment::new(&pool, vec![2, 3]);

        assert_eq!(standing_on.between(0, 1), vec![3, 2]);
        assert_eq!(standing_on.between(1, 0), vec![2, 3]);
        assert!(standing_on.settled);
    }
}
