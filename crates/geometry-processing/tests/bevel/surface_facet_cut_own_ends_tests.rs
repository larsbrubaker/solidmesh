//! Port of `R/Tests/GeometryProcessingTests/SurfaceFacetCutOwnEndsTests.cs`: a cut drawn along a
//! chord the ring it is cutting ALREADY CARRIES - the chord's two ends are corners of the ring, so
//! the boundary is drawn and there is nothing on it to find (`surface_cut_clip::clip_by`).
//!
//! Measured on "Every Chamfer Works.mcx": a band's end section is cut along the chord between its
//! rail 2 at (-3.3187, 47.7141, 20) and the crossing's foot at (-3.2759, 47.6912, 20), and the ring
//! being cut carries both of them. Read by DISTANCE the two stand a rounding either side of the
//! line - the cross product over that 0.0485 chord is 6.5e-7 - so the arithmetic answers with a
//! crossing 84 % along it, a third point on a line that already had two. The face across keeps the
//! whole chord, and the T junction between them is what a shell does not close over.
//!
//! So the reading is by IDENTITY: a ring corner that IS one of the chord's own ends stands on the
//! cut because it is that end, not because of what a dot product says, and a side whose two ends
//! both stand there is the cut itself and is never intersected.

use geometry_processing::bevel::surface::surface_cut_clip;
use geometry_processing::bevel::surface::SurfaceVertexPool;
use vector_math::Vector3;

/// The ring, the chord's two ends among its corners, and a cut line that MEASURES them a hair
/// apart - the collinear reading that mints the third point.
fn the_collinear_cut(the_rest_is_kept: bool) -> (SurfaceVertexPool, Vec<i32>, Vector3, Vector3) {
    let mut pool = SurfaceVertexPool::new(&[]);
    let beyond = if the_rest_is_kept { 1.0 } else { -1.0 };

    let from = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let to = pool.add(Vector3::new(1.0, 0.0, 0.0));
    let ring = vec![
        from,
        to,
        pool.add(Vector3::new(1.0, beyond, 0.0)),
        pool.add(Vector3::new(0.0, beyond, 0.0)),
    ];

    // Square to the chord, but tilted by a rounding, so that its two ends measure on opposite
    // sides of the line by four thousandths - which is what a chord read off two different
    // constructions does, and is far outside any ruler.
    (
        pool,
        ring,
        Vector3::new(0.5, 0.0, 0.0),
        Vector3::new(0.008, 1.0, 0.0).get_normal(),
    )
}

/// Whether no corner of a piece stands along the chord itself other than its own two ends - which
/// is the T junction this rule is about, and is asked of the chord rather than of the pool because
/// a clip may perfectly well stand a crossing on some OTHER side of the ring.
fn nothing_stands_on_the_chord(pool: &SurfaceVertexPool, piece: &[i32], ring: &[i32]) -> bool {
    let from = pool.position_of(ring[0]);
    let along = pool.position_of(ring[1]) - from;

    for &corner in piece {
        if corner == ring[0] || corner == ring[1] {
            continue;
        }

        let at = pool.position_of(corner) - from;
        let how_far = at.dot(along) / along.length_squared();

        if how_far > 0.0 && how_far < 1.0 && (at - (along * how_far)).length() < 0.01 {
            return false;
        }
    }

    true
}

/// Clipped by its own chord, the ring keeps its own vertices and no new one is minted along it.
#[test]
fn a_chord_the_ring_already_carries_mints_nothing() {
    // The rest of the ring stands on the side the cut keeps, so the line meets it along that chord
    // and nowhere else: what a section standing against a band's own footprint looks like.
    let (mut pool, ring, point, inward) = the_collinear_cut(true);
    let _stood = pool.vertices().len();

    let own_ends = [ring[0], ring[1]];
    let kept = surface_cut_clip::clip_by(&ring, &mut pool, point, inward, None, Some(&own_ends));

    assert!(
        nothing_stands_on_the_chord(&pool, &kept, &ring),
        "the chord's two ends are the boundary, so the cut stands no third point along it"
    );

    assert!(
        kept.contains(&ring[0]) && kept.contains(&ring[1]),
        "both ends of the cut's own chord stand on it, so both are kept whichever side they measure"
    );

    // And the other side of the same cut, which is the piece the section itself is: the two
    // together are bounded by the same two vertices and nothing between them.
    let beyond = surface_cut_clip::clip_by(&ring, &mut pool, point, -inward, None, Some(&own_ends));

    assert!(
        nothing_stands_on_the_chord(&pool, &beyond, &ring),
        "nor does the other side of it stand one, so the two pieces are bounded by the same two vertices"
    );
}

/// And where only ONE end of the chord is a corner of the ring, the other side still crosses as it
/// always did: the rule is about a boundary already drawn, not about chords in general.
#[test]
fn one_end_alone_still_crosses() {
    let (mut pool, ring, point, inward) = the_collinear_cut(false);
    let stood = pool.vertices().len();

    // The chord's far end is not a corner of this ring, so the side into it is a side like any
    // other.
    let own_ends = [ring[0], pool.add(Vector3::new(5.0, 5.0, 5.0))];
    let kept = surface_cut_clip::clip_by(&ring, &mut pool, point, inward, None, Some(&own_ends));

    assert!(
        pool.vertices().len() > stood + 1,
        "the far end was added to the pool, and the ring's own sides across the line still cross it"
    );

    assert!(kept.len() >= 3, "what is left of the ring is still a piece");

    let minted = kept.iter().filter(|corner| !ring.contains(corner)).count();

    assert!(
        minted >= 1,
        "a corner of the piece is a crossing this cut really made, one end being no boundary"
    );
}
