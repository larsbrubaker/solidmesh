//! The standing features the walkback withholds, reconciled: the ONE record made of the two
//! readings a feature standing out of two regions arrives as, and the check that no facet
//! withheld with a feature is also inside a region a band is being cut on, as free functions.
//!
//! Split out of the face walkback (`SurfaceFaceWalkback`), whose proper work is the two-pass cut
//! of the mesh; this is the bookkeeping that has to be right BEFORE that cut runs, because
//! everything downstream indexes a withheld feature ([`SurfaceStandingFeatureWithheld`]) by its
//! facets and a feature read twice has two of every index.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use super::{
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

/// Makes ONE record of the features two different regions' bands both reached, by appending the
/// second reading's reach and its crease runs to the first's faces.
///
/// **Why two readings arrive at all.** A stretch is read over one region's rim, because a feature
/// standing on a region is that region's own business. A feature standing out of TWO regions -
/// the nose in the chevron notch stands out of both slopes - is therefore read twice, once from
/// each, and each reading names the same facets and the run of crease ITS band uncovered. They are
/// two readings of one thing, and everything downstream indexes by facet, so they have to become
/// one before anything else is asked.
///
/// **What is merged and what is refused.** The first record's faces are kept and the second's
/// reach and crease run appended to them, matched face for face by the facets they are made of.
/// Two records whose face sets DIFFER are not two readings of one feature - they are two features
/// sharing a facet, which is one face of the part asked to be extended two ways - and that is
/// refused by name here (`Err`) rather than merged into a record whose creases do not all belong
/// to the same shape.
pub fn try_merge_what_two_bands_both_reached(
    withheld: &mut Vec<Rc<SurfaceStandingFeatureWithheld>>,
    pool: &SurfaceVertexPool,
) -> Result<(), BevelAdvisory> {
    let mut owner_of_facet: HashMap<i32, usize> = HashMap::new();
    let mut merged: Vec<Rc<SurfaceStandingFeatureWithheld>> = Vec::with_capacity(withheld.len());

    for feature in withheld.iter() {
        let mut into: Option<usize> = None;

        for face in &feature.faces {
            for facet in &face.mesh_faces {
                if let Some(&already) = owner_of_facet.get(facet) {
                    into = Some(already);
                }
            }
        }

        let Some(into) = into else {
            for face in &feature.faces {
                for &facet in &face.mesh_faces {
                    owner_of_facet.insert(facet, merged.len());
                }
            }

            merged.push(feature.clone());
            continue;
        };

        if !try_take_in_the_other_reach(&merged[into], feature) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts,
                vec![BevelAdvisoryArg::Vector3(
                    pool.position_of(feature.faces[0].crease().0),
                )],
            ));
        }
    }

    *withheld = merged;

    Ok(())
}

/// Appends one reading's reach and crease runs to another's faces, or false where the two do not
/// name the same faces and so are not two readings of one feature.
fn try_take_in_the_other_reach(
    into: &SurfaceStandingFeatureWithheld,
    other: &SurfaceStandingFeatureWithheld,
) -> bool {
    if into.faces.len() != other.faces.len() || other.reaches.borrow().len() != 1 {
        return false;
    }

    // Matched by the facets each is made of rather than by order, which is the order two
    // independent floods happened to run in.
    let mut pairs: Vec<(
        &Rc<SurfaceStandingFeatureWithheldFace>,
        &Rc<SurfaceStandingFeatureWithheldFace>,
    )> = Vec::with_capacity(into.faces.len());

    for face in &other.faces {
        let found = into
            .faces
            .iter()
            .find(|candidate| the_same_facets(&candidate.mesh_faces, &face.mesh_faces));

        let Some(found) = found else {
            return false;
        };

        pairs.push((found, face));
    }

    for (first, second) in pairs {
        let crease = second.creases.borrow()[0];
        first.creases.borrow_mut().push(crease);
        let within = second.withins.borrow()[0].clone();
        first.withins.borrow_mut().push(within);
    }

    let reach = other.reaches.borrow()[0];
    into.reaches.borrow_mut().push(reach);

    true
}

/// Whether two faces are made of the very same facets of the source mesh.
fn the_same_facets(mine: &[i32], theirs: &[i32]) -> bool {
    if mine.len() != theirs.len() {
        return false;
    }

    let standing: HashSet<i32> = mine.iter().copied().collect();

    theirs.iter().all(|facet| standing.contains(facet))
}

/// Whether no facet of any withheld face is ALSO inside a region a band is being walked back on
/// (`Err` naming where one is).
///
/// The two readings disagree about the same triangle where they overlap, and the walkback asks
/// the withholding first: a facet flooded into a feature's face because it is flat-connected to
/// it, and at the same time inside another chain's own footprint, would be carried out of the part
/// whole with the cut that should have taken a bite out of it never applied. That is material left
/// standing exactly where a band went through, and nothing downstream has anything to say about
/// it. Named here, where both answers are in hand.
pub fn none_of_the_feature_is_also_being_cut(
    withheld: &[Rc<SurfaceStandingFeatureWithheld>],
    region_of_face: &[i32],
    pool: &SurfaceVertexPool,
) -> Result<(), BevelAdvisory> {
    for feature in withheld {
        for face in &feature.faces {
            for &facet in &face.mesh_faces {
                if region_of_face[facet as usize] >= 0 {
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                        vec![BevelAdvisoryArg::Vector3(pool.position_of(face.crease().0))],
                    ));
                }
            }
        }
    }

    Ok(())
}
