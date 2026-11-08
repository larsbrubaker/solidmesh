//! What a band finds when it passes UNDER a feature standing on the face it lands on: the feature's
//! creases across the region, linked end to end into the one thing they belong to
//! ([`SurfaceStandingFeatureStretch`]), and the feature's faces held BACK from the walkback
//! ([`try_withhold`]).
//!
//! The room reading (`surface_room_standing_feature`) is what lets the band get there at all - such
//! a feature bounds nothing, because the region goes on behind it. What the band then leaves is a
//! feature standing on air. Its faces are EXTENDED through the removed corner until they meet the
//! fillet - Lars's ruling, and what the Onshape control does - so the walkback withholds them
//! ([`SurfaceStandingFeatureWithheld`]) and `surface_standing_feature_faces` emits each merged with
//! its extension as one ring.
//!
//! A stretch is linked over the WHOLE rim rather than over what one band reaches, because a
//! feature is one thing: the nose in the chevron notch has one crease inside the band and one only
//! grazed by it, and reading those as two stretches asks each half to stand on its own.
//!
//! The stretch's facets and the rim's ends are walked here (the first normal found is the landing),
//! so they are `IndexSet`/`IndexMap`: insertion order keeps the landing the same on every run.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::{IndexMap, IndexSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::surface_cut_clip;
use super::surface_room_standing_feature::stands_on;
use super::{
    SurfaceFacetCut, SurfaceRegionWalkback, SurfaceStandingFeatureWithheld,
    SurfaceStandingFeatureWithheldFace, SurfaceVertexPool,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{FaceGroup, FeatureEdge};

/// One feature's creases across the region - the vertices of the stretch in order along it, the
/// region's own facets and normal there, the creases themselves with the facets standing across
/// them, and what the walk found the shape of it to be.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceStandingFeatureStretch {
    /// The stretch's vertices, in order along it.
    pub vertices: Vec<i32>,
    /// The region's own facets along the stretch, which is what says a band reaches it.
    pub faces: IndexSet<i32>,
    /// The creases themselves (`(from, to, beyond)`), each with the feature's own facet standing
    /// across it - the rim edge's OWN far face, carried from where the crease was found.
    pub creases: Vec<(i32, i32, i32)>,
    /// The region's outward normal along the stretch.
    pub landing: Vector3,
    /// Whether every facet along it agrees with that normal.
    pub is_on_one_plane: bool,
    /// Whether the walk came back to where it started.
    pub is_closed: bool,
    /// Whether the rim FORKS anywhere along it, which cannot be read at all.
    pub is_branching: bool,
}

/// The standing features every band passes under, with their own facets held BACK rather than
/// carried over - each has to be extended through the corner its band took.
///
/// One reading of the rim for the whole region: the features standing on it are the region's own
/// business, not each cut's, and a feature reached by two cuts has to be RECOGNISED as one thing
/// before anything can be said about it ([`try_read`]).
pub fn try_withhold(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    walkbacks: &IndexMap<i32, SurfaceRegionWalkback>,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
    pool: &SurfaceVertexPool,
) -> Result<Vec<Rc<SurfaceStandingFeatureWithheld>>, BevelAdvisory> {
    let mut withheld = Vec::new();

    let on_the_cut = surface_cut_clip::on_the_cut(pool);

    for walkback in walkbacks.values() {
        for stretch in standing_stretches(walkback, beveled_mesh_edges, chain_vertices) {
            let mut reaching: Option<usize> = None;
            let mut also_reaching: Option<usize> = None;

            for cut in 0..walkback.cuts.len() {
                // Both readings, not either. The flood says the band's material reach takes in this
                // feature's facets at all; the CREASE reading says the feature's root line really
                // stands in the footprint the band takes. A feature the band only grazes is left
                // exactly as it was, which keeps the room's predicate and the walkback agreeing.
                // (The walkback has filled `reaches` for every cut.)
                let overlaps = walkback
                    .reaches
                    .get(cut)
                    .is_some_and(|reaches| stretch.faces.iter().any(|face| reaches.contains(face)));

                if !overlaps
                    || !the_crease_is_in_the_band(&walkback.cuts[cut], &stretch, pool, on_the_cut)
                {
                    continue;
                }

                if reaching.is_none() {
                    reaching = Some(cut);
                } else {
                    also_reaching = Some(cut);
                }
            }

            let Some(reaching) = reaching else {
                continue;
            };

            try_read(&stretch, also_reaching.is_some(), pool)?;

            let faces = try_the_faces_of_it(mesh, graph, &stretch, pool)?;

            if faces.is_empty() {
                continue;
            }

            let cut = &walkback.cuts[reaching];
            withheld.push(Rc::new(SurfaceStandingFeatureWithheld::new(
                (cut.edge_index, cut.left),
                stretch.landing,
                faces,
            )));
        }
    }

    Ok(withheld)
}

/// The feature's FACES along one stretch - one per coplanar region standing across its creases,
/// each with the run of crease its region stands on.
///
/// The region and not the facet: a face fanned from a point inside itself (what a boolean leaves,
/// and what "15mm Bevel Top Edge.mcx" carries) has only ONE of its triangles across the crease.
/// A face whose root line is cut into several mesh edges brings several creases to one region, and
/// they have to be one unbroken run along the stretch - named rather than worked around.
fn try_the_faces_of_it(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    stretch: &SurfaceStandingFeatureStretch,
    pool: &SurfaceVertexPool,
) -> Result<Vec<Rc<SurfaceStandingFeatureWithheldFace>>, BevelAdvisory> {
    let mut faces = Vec::new();

    let mut region_of_facet: HashMap<i32, usize> = HashMap::new();
    let mut regions: Vec<Vec<i32>> = Vec::new();
    let mut creases_of: Vec<Vec<usize>> = Vec::new();

    for (crease, &(_, _, beyond)) in stretch.creases.iter().enumerate() {
        if let Some(&already) = region_of_facet.get(&beyond) {
            creases_of[already].push(crease);
            continue;
        }

        let region = try_the_region_of(mesh, graph, beyond, pool)?;

        for &facet in &region {
            region_of_facet.insert(facet, regions.len());
        }

        regions.push(region);
        creases_of.push(vec![crease]);
    }

    for (region, along) in regions.into_iter().zip(&creases_of) {
        let first = stretch.creases[along[0]];

        for (step, &crease) in along.iter().enumerate().skip(1) {
            // Read against the stretch's own ORDER, so on a closed stretch a region standing on
            // the last crease and the first - one run, wrapped round the start of the walk - reads
            // as two and is refused. Conservative on purpose: a wrap and a region really standing
            // on two separate creases look exactly the same.
            if crease != along[0] + step {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
                    vec![pool.position_of(first.0).into()],
                ));
            }
        }

        let within: Vec<i32> = along[..along.len() - 1]
            .iter()
            .map(|&crease| stretch.creases[crease].1)
            .collect();

        let beyond = &mesh.faces[first.2 as usize];
        faces.push(Rc::new(SurfaceStandingFeatureWithheldFace::new(
            region,
            (first.0, stretch.creases[along[along.len() - 1]].1),
            within,
            pool.position_of(beyond.v0),
            Vector3::from_vector3_float(beyond.normal),
        )));
    }

    Ok(faces)
}

/// The whole coplanar region one facet belongs to - the FACE of the part it is a triangle of - or a
/// refusal where what the flood reached is not one plane after all.
///
/// The flood is `FaceGroup`'s at the flat angle the rest of the pass reads faces by. What it cannot
/// say on its own is that the face is FLAT: it crosses each edge on its own account, so a finely
/// tessellated cylinder arrives as one region - and a region held back is extended as a PLANE. So
/// every facet is measured against the SEED's own normal, and a drift the flood would walk over is
/// named.
pub fn try_the_region_of(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    seed: i32,
    pool: &SurfaceVertexPool,
) -> Result<Vec<i32>, BevelAdvisory> {
    let does_not_extend = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
            vec![at.into()],
        )
    };

    // The seed is always a facet of `mesh` here (a rim edge's far face); one that is not is refused
    // as a guard, named at the origin for want of a facet to name.
    let region =
        FaceGroup::create_with_graph(mesh, graph, seed, FeatureEdge::default_flat_angle_degrees())
            .map_err(|_| does_not_extend(Vector3::ZERO))?
            .faces()
            .to_vec();

    let seed_normal = Vector3::from_vector3_float(mesh.faces[seed as usize].normal);

    for &facet in &region {
        let face = &mesh.faces[facet as usize];

        if seed_normal.dot(Vector3::from_vector3_float(face.normal))
            < FeatureEdge::COPLANAR_FACE_DOT
        {
            return Err(does_not_extend(pool.position_of(face.v0)));
        }
    }

    Ok(region)
}

/// Whether the feature's ROOT LINE has a point inside the band - past the tangent line and between
/// the band's ends - which is what makes this a feature the band undercut rather than one it passed
/// beside. The same question the room's predicate answers for the region as a whole, so room and
/// walkback never disagree.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x < y)` on purpose: it keeps a NaN out as well.
fn the_crease_is_in_the_band(
    cut: &SurfaceFacetCut,
    stretch: &SurfaceStandingFeatureStretch,
    pool: &SurfaceVertexPool,
    on_the_cut: f64,
) -> bool {
    stretch.vertices.iter().any(|&vertex| {
        let at = pool.position_of(vertex);

        if !(cut.signed_distance(at) < -on_the_cut) {
            return false;
        }

        cut.ends
            .iter()
            .all(|end| !(end.signed_distance(at) < -on_the_cut))
    })
}

/// The three things a stretch has to be before its faces can be extended as flat planes, each named
/// rather than worked around - not BRANCHING (a forked rim has no order to walk it in), on ONE
/// PLANE (the region is cut as a plane), and under ONE band (two bands' extensions would overlap,
/// the same triangle emitted twice).
pub fn try_read(
    stretch: &SurfaceStandingFeatureStretch,
    under_two_cuts: bool,
    pool: &SurfaceVertexPool,
) -> Result<(), BevelAdvisory> {
    if stretch.is_branching || !stretch.is_on_one_plane {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotClose,
            vec![pool.position_of(stretch.vertices[0]).into()],
        ));
    }

    if under_two_cuts {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts,
            vec![pool.position_of(stretch.vertices[0]).into()],
        ));
    }

    Ok(())
}

/// The stretches of standing feature on one region, linked from the rim edges
/// `surface_room_standing_feature::stands_on` names. Open stretches first, each from an END so that
/// one walk covers it; what is left after that is rings, started wherever first met.
pub fn standing_stretches(
    walkback: &SurfaceRegionWalkback,
    beveled_mesh_edges: &HashSet<i32>,
    chain_vertices: &HashSet<i32>,
) -> Vec<SurfaceStandingFeatureStretch> {
    let mut ends: IndexMap<i32, Vec<i32>> = IndexMap::new();
    let mut faces_at: HashMap<i32, Vec<i32>> = HashMap::new();
    let mut normal_of: HashMap<i32, Vector3> = HashMap::new();
    let mut crease_between: HashMap<(i32, i32), i32> = HashMap::new();
    let mut found = Vec::new();

    for rim in walkback.region.boundary_edges() {
        if !stands_on(&walkback.region, rim, beveled_mesh_edges, chain_vertices) {
            continue;
        }

        let face0_is_on_it = walkback.faces.contains(&rim.face0);
        let on_the_region = if face0_is_on_it { rim.face0 } else { rim.face1 };
        let beyond = if face0_is_on_it { rim.face1 } else { rim.face0 };

        normal_of.insert(
            on_the_region,
            if face0_is_on_it {
                rim.normal0
            } else {
                rim.normal1
            },
        );

        // Which facet stands across each crease, kept by the crease it belongs to: the extension
        // is built per facet from the crease it lost its footing on, so the pairing has to survive
        // the linking below.
        crease_between.insert(ordered(rim.start_vertex, rim.end_vertex), beyond);

        note(&mut faces_at, rim.start_vertex, on_the_region);
        note(&mut faces_at, rim.end_vertex, on_the_region);

        join(&mut ends, rim.start_vertex, rim.end_vertex);
        join(&mut ends, rim.end_vertex, rim.start_vertex);
    }

    let mut walked = HashSet::new();

    for pass in 0..2 {
        for (&start, neighbours) in &ends {
            if walked.contains(&start) || (pass == 0 && neighbours.len() != 1) {
                continue;
            }

            let (path, is_branching) = walk(&ends, start, &mut walked);

            // Read off the walk rather than assumed from which pass found it: a ring with a tail
            // is met tail-first, and calling what is left of it closed would shut it on a chord.
            let last = path[path.len() - 1];
            let is_closed = path.len() > 2 && ends[&last].contains(&path[0]);

            let mut creases = Vec::new();

            for pair in path.windows(2) {
                if let Some(&facet) = crease_between.get(&ordered(pair[0], pair[1])) {
                    creases.push((pair[0], pair[1], facet));
                }
            }

            if is_closed {
                if let Some(&closing) = crease_between.get(&ordered(last, path[0])) {
                    creases.push((last, path[0], closing));
                }
            }

            let faces = faces_along(&path, &faces_at);
            let (landing, is_on_one_plane) = normal_along(&faces, &normal_of);

            found.push(SurfaceStandingFeatureStretch {
                vertices: path,
                faces,
                creases,
                landing,
                is_on_one_plane,
                is_closed,
                is_branching,
            });
        }
    }

    found
}

/// One crease named the same way whichever end of it is read first.
fn ordered(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// The region's own facets along one stretch.
fn faces_along(path: &[i32], faces_at: &HashMap<i32, Vec<i32>>) -> IndexSet<i32> {
    let mut faces = IndexSet::new();

    for vertex in path {
        if let Some(along) = faces_at.get(vertex) {
            faces.extend(along.iter().copied());
        }
    }

    faces
}

/// The region's outward normal along one stretch (its facets `faces`), and whether every facet
/// along it agrees with the first to within the angle the pass calls one plane
/// (`FeatureEdge::COPLANAR_FACE_DOT`).
fn normal_along(faces: &IndexSet<i32>, normal_of: &HashMap<i32, Vector3>) -> (Vector3, bool) {
    let mut landing = Vector3::ZERO;
    let mut is_on_one_plane = true;

    for face in faces {
        let Some(&normal) = normal_of.get(face) else {
            continue;
        };

        if landing == Vector3::ZERO {
            landing = normal;
        } else if landing.dot(normal) < FeatureEdge::COPLANAR_FACE_DOT {
            is_on_one_plane = false;
        }
    }

    (landing, is_on_one_plane)
}

/// The stretch from one of its vertices to the far end of it, marking what it covers, and whether
/// the rim FORKED anywhere on the way - at a fork there is no "the far end" and the walk's own
/// order means nothing, so the caller refuses.
fn walk(ends: &IndexMap<i32, Vec<i32>>, start: i32, walked: &mut HashSet<i32>) -> (Vec<i32>, bool) {
    let mut path = vec![start];

    walked.insert(start);
    let mut is_branching = ends[&start].len() > 2;

    let mut at = start;

    loop {
        let Some(&next) = ends[&at].iter().find(|next| !walked.contains(*next)) else {
            return (path, is_branching);
        };

        walked.insert(next);
        path.push(next);
        at = next;
        is_branching |= ends[&next].len() > 2;
    }
}

/// Notes which of the region's facets a vertex of the stretch stands on.
fn note(faces_at: &mut HashMap<i32, Vec<i32>>, vertex: i32, face: i32) {
    let faces = faces_at
        .entry(vertex)
        .or_insert_with(|| Vec::with_capacity(2));

    if !faces.contains(&face) {
        faces.push(face);
    }
}

/// Notes that two vertices of the rim are joined by one standing crease.
fn join(ends: &mut IndexMap<i32, Vec<i32>>, from: i32, to: i32) {
    let neighbours = ends.entry(from).or_insert_with(|| Vec::with_capacity(2));

    if !neighbours.contains(&to) {
        neighbours.push(to);
    }
}
