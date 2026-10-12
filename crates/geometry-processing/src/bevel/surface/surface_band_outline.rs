//! THE BAND'S OUTLINE ON A CHORDED WALL (`Bevel/Surface/SurfaceBandOutline.cs`, a static class; free
//! functions here): which rail a cut draws its band's far side along on a facet that is not its own.
//! It fills in [`SurfaceRegionWalkback::rail_borrowed_on`] once the walkback has every cut and reach
//! of a region (`SurfaceFaceWalkback`), and [`as_drawn_on`] is what the walkback clips each facet by.
//!
//! A cut is a half space through a tangent LINE, and a half space has no idea where the facet it was
//! written for ends; on a curved region read as one surface - a bore's chorded wall, a prism's lobe -
//! the flood carries chord j's cut past the crease its own facet ends at and onto facet j+1, where a
//! leaning plane's trace is not that rail at all but a LIP cut past it.
//!
//! **What the outline is.** The band the walkback takes off a chorded wall is the material above each
//! facet's OWN rail, and the two rails' different depths meet in a step down the crease between them.
//! That is the very polyline the strips' mitre patch reads from the rails, so drawing it here is what
//! makes the wall's rim and the patch's rim one outline rather than two between the same two ends. On
//! the raked bore's mouth the difference is a single vertex per surplus corner, 2.734e-3 off the
//! crease - ten of the pool's rulers - and the mesh says so exactly: the two segments into that lip are used
//! once by the walkback alone and the crease segment the two outlines share is used three times
//! (`SurfaceRakedBoreMouthClaimTests`).
//!
//! **Why it is a BORROWED rail and not a limit.** Stopping the flood at the crease was tried and
//! measured: it leaves the sliver of facet j+1 between the crease and the mitre plane standing, which
//! the mitre's own patch spans, so the shell is left open along the crease - and it refuses a leaning
//! mixed corner that really does carry its blend across a lobe's creases. Borrowing does neither. The
//! cut still reaches as far as it did, still bounded by its own band ends, and only the LINE it is
//! drawn to changes - to the rail that lies in the facet it has reached. So no band stops crossing a
//! crease, and no band crosses one at the wrong depth.
//!
//! **What it is not asked of.** A cut whose own tangent line already lies in the facet's plane - every
//! facet of a planar region, however it is triangulated - has nothing to borrow and is left alone,
//! which is why nothing planar in the corpus can move under this. Nor is a ROLLED cut asked: the roll
//! names the facets its own sections paced out and its tangency is already read facet by facet
//! ([`SurfaceFacetCut::may_touch`]), so re-aiming it would undo the walk it made.

use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use super::{SurfaceFacetCut, SurfaceRegionWalkback, SurfaceVertexPool};

/// C# `SquareEnoughToThePlane`: how far off a facet's plane a tangent line may stand and still be
/// read as lying IN it: the direction to a hair of square, and the point to the pool's own nearness. A
/// neighbouring facet of a forty-chord bore turns nine degrees, so nothing near the boundary of this
/// reading exists on the walls it is asked about.
const SQUARE_ENOUGH_TO_THE_PLANE: f64 = 1e-6;

/// C# `BorrowTheRailOfEachFacetReachedAcrossACrease`: for each cut of each region, the facets it
/// reaches ACROSS a crease and the sibling cut whose own rail lies in each of those facets' planes -
/// which is the line it draws its band's far side along there instead of its own.
///
/// `walkbacks` is C#'s `Dictionary<int, SurfaceRegionWalkback>`, in its insertion order. A cut's
/// reaches are a `HashSet`, so the order the borrowed facets go into a cut's map is not C#'s; the map
/// is only ever looked up by facet ([`as_drawn_on`]), never walked.
pub fn borrow_the_rail_of_each_facet_reached_across_a_crease(
    mesh: &Mesh,
    walkbacks: &mut IndexMap<i32, SurfaceRegionWalkback>,
    pool: &SurfaceVertexPool,
) {
    for walkback in walkbacks.values_mut() {
        for cut in 0..walkback.cuts.len() {
            walkback.rail_borrowed_on.push(None);

            if walkback.cuts[cut].may_touch.is_some() {
                // A rolled cut has already read its tangency facet by facet.
                continue;
            }

            let reached: Vec<i32> = walkback.reaches[cut].iter().copied().collect();

            for face_index in reached {
                if lies_in_the_facet(mesh, face_index, &walkback.cuts[cut], pool) {
                    continue;
                }

                let borrowed =
                    the_sibling_whose_rail_lies_in_it(mesh, face_index, walkback, cut, pool);

                if borrowed < 0 {
                    // Nothing of this region's own rails lies in that facet, so there is no outline
                    // to draw and the half space stands as it was written.
                    continue;
                }

                walkback.rail_borrowed_on[cut]
                    .get_or_insert_with(IndexMap::new)
                    .insert(face_index, borrowed);
            }
        }
    }
}

/// C# `AsDrawnOn` (an extension method): the cut a facet is really clipped by - the one whose rail
/// lies in that facet's own plane where this one's does not, and this one otherwise. The borrowed rail
/// replaces the LINE alone - the band ends, the seed and the chain the cut belongs to are all still its
/// own.
pub fn as_drawn_on(walkback: &SurfaceRegionWalkback, cut: i32, face_index: i32) -> SurfaceFacetCut {
    let cut = cut as usize;

    let borrowed = walkback
        .rail_borrowed_on
        .get(cut)
        .and_then(|borrowed_on| borrowed_on.as_ref())
        .and_then(|borrowed_on| borrowed_on.get(&face_index).copied());

    let Some(borrowed) = borrowed else {
        return walkback.cuts[cut].clone();
    };

    let sibling = &walkback.cuts[borrowed as usize];

    walkback.cuts[cut].drawn_along(sibling.tangent_line, sibling.inward)
}

/// C# `TheSiblingWhoseRailLiesInIt`: which of the region's other cuts has its rail lying in this
/// facet's own plane, or -1. Where more than one does - two chords of a wall whose facets are
/// coplanar - the one whose plane keeps the most material at the facet is taken, so that borrowing can only ever
/// leave a band shallower than the half space it replaces and never gouge deeper than one.
fn the_sibling_whose_rail_lies_in_it(
    mesh: &Mesh,
    face_index: i32,
    walkback: &SurfaceRegionWalkback,
    cut: usize,
    pool: &SurfaceVertexPool,
) -> i32 {
    let middle = the_middle_of(mesh, face_index);
    let mut found = -1;
    let mut kept_most = f64::MIN;

    for sibling in 0..walkback.cuts.len() {
        if sibling == cut
            || walkback.cuts[sibling].may_touch.is_some()
            || walkback.cuts[sibling].left != walkback.cuts[cut].left
            || walkback.cuts[sibling].inward.dot(walkback.cuts[cut].inward) <= 0.0
            || !lies_in_the_facet(mesh, face_index, &walkback.cuts[sibling], pool)
        {
            continue;
        }

        let keeps = walkback.cuts[sibling].signed_distance(middle);

        if keeps > kept_most {
            kept_most = keeps;
            found = sibling as i32;
        }
    }

    found
}

/// C# `LiesInTheFacet`: whether a cut's own tangent line lies in a facet's plane - its direction
/// square to the facet's normal and its point on the facet's plane - which is what makes the cut's
/// trace on that facet the rail itself rather than a lip leaning past it.
fn lies_in_the_facet(
    mesh: &Mesh,
    face_index: i32,
    cut: &SurfaceFacetCut,
    pool: &SurfaceVertexPool,
) -> bool {
    let normal = Vector3::from_vector3_float(mesh.faces[face_index as usize].normal);

    cut.tangent_line.direction.dot(normal).abs() <= SQUARE_ENOUGH_TO_THE_PLANE
        && (cut.tangent_line.point - the_middle_of(mesh, face_index))
            .dot(normal)
            .abs()
            <= pool.near_enough()
}

/// C# `TheMiddleOf`: the centroid of one face.
fn the_middle_of(mesh: &Mesh, face_index: i32) -> Vector3 {
    let face = &mesh.faces[face_index as usize];

    (Vector3::from_vector3_float(mesh.vertices[face.v0 as usize])
        + Vector3::from_vector3_float(mesh.vertices[face.v1 as usize])
        + Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]))
        / 3.0
}
