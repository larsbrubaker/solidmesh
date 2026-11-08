//! The second half of the pierced face's junction tests, split from
//! [`crate::section_pierced_face_junction_tests`] only for length: the junction's place in a whole
//! RUN of the pierced top face's 45 edge chain - the builder keeping both junctions and every other
//! station untouched, the face rebuild and loft closing round them, each wall end placed in (and
//! standing on) its own wall, and the split station beside a junction laid on the carried wall end.
//! The class's helpers are [`crate::section_pierced_face_readings`].

use std::collections::HashMap;
use std::rc::Rc;

use solidmesh::bevel::sections::section_smooth_side::{self, SmoothSidePlace};
use solidmesh::bevel::sections::{
    section_builder_run, section_face_rebuild, section_fan_walk, section_loft,
    section_surface_walk, SectionKind, SectionPool, SectionRun,
};
use solidmesh::bevel::surface::{SurfaceBevelPlan, SurfaceVertexPool};
use solidmesh::topology::FeatureEdgeSet;
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use crate::inner_face_test_mesh;
use crate::section_pierced_face_readings::{
    the_normal_of, the_operations_feature_reading, the_plans, the_triangle_holds, RADIUS,
};

/// The index-weighted sums of every ORDINARY station's profile points, pinned so that a later change
/// to the junction wiring cannot move a point the junction does not own. Measured at
/// `Radius = 0.5` with 8 segments on the pierced top face (measured with the split station beside a
/// junction walking on its own).
const ORDINARY_STATIONS_DIGEST_X: f64 = -576078.6491500206;

const ORDINARY_STATIONS_DIGEST_Y: f64 = -1642863.3466091815;

const ORDINARY_STATIONS_DIGEST_Z: f64 = 884771.0926676003;

/// How nearly a place's face must lie with the wall it is claimed to be in. A
/// wall may be TESSELLATED - the bore at this part's junctions steps 7.5 degrees a facet, and P2's
/// walk lands one facet round from the wall the junction names, reading 0.9877 - while the FOLD to
/// the other wall turns 87 degrees, reading 0.0555. This sits in the wide gap between the two.
const THE_SAME_WALL_DOT: f64 = 0.9;

type Faces = Vec<(i32, i32, i32)>;

/// THE RUN CARRIES THE JUNCTION. The builder - asked for the pierced face's own plan - draws every
/// station of the 45 edge chain, keeps the two junctions in the run's ledger, hands the LEAVING
/// section back as the station's own section and the ARRIVING one back as the end of the strip
/// arriving there. Every other station is untouched.
#[test]
fn the_run_keeps_the_junctions_and_leaves_every_other_station_alone() {
    const SEGMENTS: i32 = 8;

    let part = inner_face_test_mesh::the_part();
    let (run, pool) = the_run(&part, RADIUS, SEGMENTS);

    assert_eq!(run.count(), 45, "the pierced top face's chain is 45 edges");

    let mut keys: Vec<i32> = run.junctions.keys().copied().collect();
    keys.sort();
    assert_eq!(
        keys,
        the_junction_stations_of(&run),
        "the ledger holds exactly the stations the predicate reads as junctions"
    );

    for vertex in [19, 59] {
        let stations: Vec<i32> = (0..run.count())
            .filter(|&i| run.edges[i as usize].from_vertex() == vertex)
            .collect();
        assert_eq!(stations.len(), 1);
        let station = stations[0];
        let arriving_edge = (station - 1 + run.count()) % run.count();
        let junction = &run.junctions[&station];

        assert_eq!(junction.chain_vertex, vertex);

        assert!(
            Rc::ptr_eq(&run.sections[station as usize], &junction.leaving_section),
            "the station's one section is the LEAVING row"
        );

        assert!(Rc::ptr_eq(
            &run.start_of(station),
            &junction.leaving_section
        ));

        assert!(
            Rc::ptr_eq(
                &run.end_of(arriving_edge)
                    .expect("a closed run has every end"),
                &junction.arriving_section
            ),
            "the strip arriving stops at the arriving wall end"
        );

        assert_eq!(junction.leaving_section.kind, SectionKind::Junction);
        assert_eq!(junction.arriving_section.kind, SectionKind::Junction);

        // EITHER side may be the far one - which of the two canals meets its own wall first is a
        // reading of the two wedges, not of the walk's direction - so the claim is that the NEAR row
        // is the ordinary rung and the far one carries the samples past it.
        assert_eq!(
            junction.near_wall_end(),
            SEGMENTS,
            "the near wall end is the ordinary rung's last sample"
        );

        assert!(
            junction.far_wall_end() > SEGMENTS,
            "the far row carries the samples past the near wall end"
        );

        let arriving_segments = junction.arriving_section.segments();
        let leaving_segments = junction.leaving_section.segments();

        assert_eq!(
            arriving_segments.min(leaving_segments),
            junction.near_wall_end()
        );

        assert_eq!(
            arriving_segments.max(leaving_segments),
            junction.far_wall_end()
        );

        // The two rows share their opening stretch BY INDEX, which is the junction's whole closure
        // claim - the seam between the two strips is the same points, not two copies of them.
        let arriving = &junction.arriving_section.profile;
        let leaving = &junction.leaving_section.profile;
        let from_the_flat_end = junction.flat_on_the_left;

        for k in 0..=(junction.near_wall_end() as usize) {
            let one = if from_the_flat_end {
                arriving[k]
            } else {
                arriving[arriving.len() - 1 - k]
            };
            let other = if from_the_flat_end {
                leaving[k]
            } else {
                leaving[leaving.len() - 1 - k]
            };

            assert_eq!(
                one, other,
                "sample {k} of the two rows at {vertex} is the same pool point"
            );
        }
    }

    // AND EVERY OTHER STATION IS UNTOUCHED: the ordinary stations' rungs are all the usual length,
    // and their points stand where this reading first put them, to an index-weighted digest that no
    // single point can move without changing.
    let mut weighted_x = 0.0;
    let mut weighted_y = 0.0;
    let mut weighted_z = 0.0;
    let mut ordinary = 0;

    for station in 0..run.count() {
        if run.junctions.contains_key(&station) {
            continue;
        }

        let section = &run.sections[station as usize];

        assert_ne!(section.kind, SectionKind::Junction);

        assert_eq!(
            section.segments(),
            SEGMENTS,
            "station {station} at vertex {} is an ordinary rung",
            section.chain_vertex
        );

        for k in 0..section.profile.len() {
            let at = pool.position_of(section.profile[k]);
            let weight = ((station + 1) * (k as i32 + 1)) as f64;

            weighted_x += weight * at.x;
            weighted_y += weight * at.y;
            weighted_z += weight * at.z;
        }

        ordinary += 1;
    }

    assert_eq!(ordinary, run.count() - 2);

    assert!(
        (weighted_x - ORDINARY_STATIONS_DIGEST_X).abs() <= 1e-6,
        "{weighted_x}"
    );
    assert!(
        (weighted_y - ORDINARY_STATIONS_DIGEST_Y).abs() <= 1e-6,
        "{weighted_y}"
    );
    assert!(
        (weighted_z - ORDINARY_STATIONS_DIGEST_Z).abs() <= 1e-6,
        "{weighted_z}"
    );
}

/// THE REBUILT PART CLOSES AT THE JUNCTION - duty three. The run is taken through the face rebuild
/// and the loft, the two passes the assembly runs over a closed run, and every directed edge the
/// junction's own ledger names stands in the result EXACTLY TWICE, once each way: the patch is laid,
/// the wall-side fan is split at the fold, and the two rim chords onto F1 are drawn.
///
/// The claim is made on the REBUILD AND LOFT, not on the whole assembly pass: the assembly's FIRST
/// pass is the smooth-side cut, so the rebuild and the loft are called here directly, and what is
/// measured is the junction's own neighbourhood, which is exactly what the ledger is for.
#[test]
fn the_rebuilt_part_closes_around_the_junctions() {
    const SEGMENTS: i32 = 8;

    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let (run, pool) = the_run(&part, RADIUS, SEGMENTS);

    let mut faces =
        section_face_rebuild::try_rebuild(&part, &graph, std::slice::from_ref(&run), &pool, None)
            .unwrap_or_else(|refused| {
                panic!(
                    "the rebuild lays the junction's patch - instead {:?} at {:?}",
                    refused.code(),
                    refused.args()
                )
            });

    if let Err(unlofted) = section_loft::try_emit(&run, &pool, &mut faces, None) {
        panic!(
            "and the strips are lofted over it - instead {:?}",
            unlofted.code()
        );
    }

    let laid = the_directed_edges_of(&faces);

    let mut stations: Vec<i32> = run.junctions.keys().copied().collect();
    stations.sort();

    for station in stations {
        let junction = &run.junctions[&station];

        for triangle in junction.patch_triangles() {
            assert!(
                faces.iter().any(|face| the_same_triangle(*face, &triangle)),
                "the patch at vertex {} is a face of the part",
                junction.chain_vertex
            );
        }

        for (from, to, piece) in junction.directed_edges() {
            assert_eq!(
                laid.get(&(from, to)).copied().unwrap_or_default(),
                1,
                "{piece:?} lays {from} -> {to} once at vertex {}",
                junction.chain_vertex
            );

            assert_eq!(
                laid.get(&(to, from)).copied().unwrap_or_default(),
                1,
                "and something lays {to} -> {from} back at vertex {}",
                junction.chain_vertex
            );
        }
    }

    // AND THE CLAIM IS NO LONGER LOCAL: with both junctions wired the WHOLE rebuilt part closes -
    // every directed edge is laid exactly once and its opposite exactly once, and all of it hangs
    // together as one shell.
    let open: Vec<&(i32, i32)> = laid
        .keys()
        .filter(|edge| !laid.contains_key(&(edge.1, edge.0)))
        .collect();

    assert_eq!(
        open.len(),
        0,
        "nothing is left open - instead {:?}",
        open.iter().take(8).collect::<Vec<_>>()
    );

    let doubled: Vec<(&(i32, i32), &i32)> = laid.iter().filter(|one| *one.1 != 1).collect();

    assert_eq!(
        doubled.len(),
        0,
        "and nothing is laid twice the same way - instead {:?}",
        doubled.iter().take(8).collect::<Vec<_>>()
    );

    assert_eq!(
        the_shell_count_of(&faces),
        1,
        "and the rebuilt part is one shell"
    );
}

/// EACH WALL END IS PLACED IN ITS OWN WALL. The smooth side's cut lays a junction station as three
/// points - P1 on the ARRIVING canal's wall, F1 on the fold, P2 on the LEAVING canal's - and each of
/// the two wall ends is WALKED to its place across the surface. The fold between the two walls runs
/// through the one fan at the chain vertex, so a walk handed the whole fan sets out through
/// whichever face holds its direction and the fold then stops it short of its own wall. So the
/// claim is on the PLACE: the face each end is placed in lies in that end's own wall.
///
/// The cut's own answer is not asserted here - what this pins is the placement, which the boundary
/// lays before any join is tried, so the reading stands whether the cut goes through or not.
#[test]
fn each_junctions_wall_ends_are_placed_in_their_own_walls() {
    const SEGMENTS: i32 = 8;

    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let (run, mut pool) = the_run(&part, RADIUS, SEGMENTS);

    // The smooth side is the side the junction's shared FLAT face is NOT: the walls are what the
    // cut lays the two ends on.
    let smooth_side_is_left = !run
        .junctions
        .values()
        .next()
        .expect("two junctions")
        .flat_on_the_left;
    let is_a_feature_edge = the_operations_feature_reading(&part, &graph);

    let (cut, _) = section_smooth_side::try_cut_the_smooth_side(
        &part,
        &graph,
        &edge_at_pair,
        &mut pool,
        &run,
        smooth_side_is_left,
        Some(&*is_a_feature_edge),
    );

    for junction in run.junctions.values() {
        for (end, wall, other, carried_onto, name) in [
            (
                junction.samples[junction.arriving_wall_end as usize],
                junction.arriving_wall,
                junction.leaving_wall,
                junction.carried_onto[junction.arriving_wall_end as usize],
                "P1",
            ),
            (
                junction.samples[junction.leaving_wall_end as usize],
                junction.leaving_wall,
                junction.arriving_wall,
                junction.carried_onto[junction.leaving_wall_end as usize],
                "P2",
            ),
        ] {
            let place = cut.places.get(&end).unwrap_or_else(|| {
                panic!("{name} at vertex {} was laid at all", junction.chain_vertex)
            });

            // A CARRIED end is placed where the CARRY put it, and that facet need not lie in the
            // end's own wall at all: the far stretch is taken out onto the facet that HOLDS it. The
            // own-wall reading below is of a WALKED end.
            if carried_onto >= 0 {
                assert_eq!(
                    place.face, carried_onto,
                    "{name} at vertex {} is placed where it was carried",
                    junction.chain_vertex
                );

                continue;
            }

            let normal = the_normal_of_the_place(&part, &graph, place, wall);
            let said = format!(
                "{name} at vertex {} is placed in {}, normal {normal:?} against its own wall \
                 {wall:?} and the other wall {other:?}",
                junction.chain_vertex,
                if place.face >= 0 {
                    format!("face {}", place.face)
                } else {
                    format!("edge {}", place.edge)
                }
            );

            assert!(normal.dot(wall) > normal.dot(other), "{said}");

            assert!(normal.dot(wall) > THE_SAME_WALL_DOT, "{said}");
        }
    }
}

/// EVERY JUNCTION WALL END STANDS ON THE PART. A boundary point is joined to its neighbours by a
/// walk ACROSS THE SURFACE, so a point that is not on the surface at all cannot be arrived at. Both
/// wall ends of both junctions are therefore held to the part's own skin, at the mesh's weld
/// distance, and every sample of the far stretch carried out onto a facet stands ON that facet.
///
/// The far - tessellated - wall's end is the tangency on the canal's IDEAL wall; the whole far
/// stretch is carried OUT onto the facet that holds it
/// (`SectionJunction.TryCarryTheFarStretchOntoTheFacets`), each sample keeping its sweep parameter
/// and its direction from the far canal's axis. Read at `RADIUS` and at `2 * RADIUS`, in a
/// loop.
#[test]
fn each_junctions_wall_ends_stand_on_the_parts_surface() {
    for radius in [RADIUS, 2.0 * RADIUS] {
        each_junctions_wall_ends_stand_on_the_parts_surface_at(radius);
    }
}

fn each_junctions_wall_ends_stand_on_the_parts_surface_at(radius: f64) {
    let part = inner_face_test_mesh::the_part();

    // Read at the operation's own size as well as at the fillet's, so the reading is of the pick
    // the parked row made and not only of a smaller fillet that happens to keep clear of the bore.
    let (run, pool) = the_run(&part, radius, inner_face_test_mesh::SEGMENTS);

    let weld = SurfaceVertexPool::weld_distance_for(&part.vertices);

    for (station, junction) in &run.junctions {
        for (end, name) in [
            (junction.arriving_wall_end, "P1"),
            (junction.leaving_wall_end, "P2"),
        ] {
            let at = pool.position_of(junction.samples[end as usize]);
            let off = the_distance_to_the_parts_surface(&part, at);

            assert!(
                off <= weld,
                "station {station}'s {name} at {at:?} stands {off:.6} off the part (radius {radius})"
            );
        }

        for k in 0..junction.samples.len() {
            let face = junction.carried_onto[k];

            if face < 0 {
                // An INTERIOR sample of the fillet stands inside the material and must - that is
                // what a fillet at a convex edge cuts away. Only the FAR stretch, which is the row's
                // contact with the wall, is held to the skin.
                continue;
            }

            let at = pool.position_of(junction.samples[k]);
            let depth = the_depth_inside_the_part(&part, at);

            assert!(
                depth <= weld,
                "station {station}'s carried sample {k} at {at:?} stands {depth:.6} INSIDE the part"
            );

            // A carried sample must stand ON the facet it was carried onto - not merely on its
            // plane, which is what the ideal wall already gave: the whole failure was a point on
            // its own facet's plane but PAST that facet's edge.
            assert!(
                the_distance_to_the_plane_of(&part, face, at) <= weld,
                "station {station}'s carried sample {k} stands off face {face}'s plane"
            );

            assert!(
                the_triangle_holds(&part, face, at, weld),
                "station {station}'s carried sample {k} at {at:?} stands outside face {face}"
            );
        }
    }
}

/// A SPLIT STATION BESIDE A JUNCTION IS LAID ON THE JUNCTION'S CARRIED WALL END. Station 2 and
/// station 40 are the split stations next to the run's two junctions, and each is placed on the
/// JOIN between its two unsplit neighbours (`SectionBuilder.TheJoinsOwnPoint`). The neighbour on
/// the wall side IS the junction, whose own section carries no walk face by construction, so the
/// far end of that join is taken from the junction's LEDGER instead: the wall end on the side the
/// join arrives from, at the facet it was carried onto.
///
/// What this pins is the OUTCOME the cut needs: the station has a walk face on the wall side, and
/// its boundary point stands IN that face to the mesh's own lattice.
#[test]
fn a_split_station_beside_a_junction_is_laid_on_the_carried_wall_end() {
    const SEGMENTS: i32 = 8;

    let part = inner_face_test_mesh::the_part();
    let (run, pool) = the_run(&part, RADIUS, SEGMENTS);

    // The wall side is the side the shared FLAT face is not - the side the two wall ends stand on.
    let smooth_side_is_left = !run
        .junctions
        .values()
        .next()
        .expect("two junctions")
        .flat_on_the_left;
    let lattice = section_surface_walk::grid_of(&part);

    for station in [2usize, 40] {
        let section = &run.sections[station];
        let face = if smooth_side_is_left {
            section.left_walk_face
        } else {
            section.right_walk_face
        };
        let at = pool.position_of(if smooth_side_is_left {
            section.left_boundary
        } else {
            section.right_boundary
        });

        assert!(
            face >= 0,
            "station {station} beside a junction is laid on a face, standing at {at:?}"
        );

        let corner =
            Vector3::from_vector3_float(part.vertices[part.faces[face as usize].v0 as usize]);
        let off = (at - corner).dot(the_normal_of(&part, face));

        assert!(
            off.abs() <= lattice,
            "station {station}'s boundary {at:?} stands in face {face}'s plane"
        );

        assert!(
            the_triangle_holds(&part, face, at, lattice),
            "station {station}'s boundary {at:?} stands within face {face}"
        );
    }
}

/// The run the builder draws for the pierced face's plan through vertex 19, with the operation's
/// own feature reading, asserted to build.
pub(crate) fn the_run(part: &Mesh, radius: f64, segments: i32) -> (SectionRun, SectionPool) {
    let graph = part.get_mesh_edge_graph();
    let (plans, _, _) = the_plans(part, None);
    let features = FeatureEdgeSet::create_with_graph(
        part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );

    let plan = the_plan_through_19(&plans);
    let mut pool = SectionPool::new(&part.vertices);

    let run = section_builder_run::try_build_plan(
        Some(plan),
        part,
        &graph,
        &mut pool,
        radius,
        segments,
        Some(&features),
    )
    .unwrap_or_else(|failure| panic!("the junction is drawn, not refused - instead {failure:?}"));

    (run, pool)
}

/// The one plan with an edge leaving vertex 19; fails on none or several.
pub(crate) fn the_plan_through_19(plans: &[Rc<SurfaceBevelPlan>]) -> Rc<SurfaceBevelPlan> {
    let found: Vec<&Rc<SurfaceBevelPlan>> = plans
        .iter()
        .filter(|one| one.edges.iter().any(|edge| edge.edge.from_vertex() == 19))
        .collect();

    assert_eq!(found.len(), 1, "one plan runs through vertex 19");

    Rc::clone(found[0])
}

/// The unit normal of the face a place names - or, where the place is on a
/// mesh EDGE, of whichever of that edge's two faces stands nearest the wall asked about.
fn the_normal_of_the_place(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    place: &SmoothSidePlace,
    wall: Vector3,
) -> Vector3 {
    if place.face >= 0 {
        return the_normal_of(part, place.face);
    }

    let mut best = Vector3::ZERO;

    for i in 0..graph.get_face_count(place.edge) {
        let normal = the_normal_of(part, graph.get_face(place.edge, i));

        if best == Vector3::ZERO || normal.dot(wall) > best.dot(wall) {
            best = normal;
        }
    }

    best
}

/// How many connected pieces a face list falls into, joined by shared corner
/// index.
fn the_shell_count_of(faces: &Faces) -> usize {
    let mut up_from: HashMap<i32, i32> = HashMap::new();

    fn root_of(up_from: &mut HashMap<i32, i32>, at: i32) -> i32 {
        let Some(&up) = up_from.get(&at) else {
            up_from.insert(at, at);

            return at;
        };

        if up == at {
            return at;
        }

        let root = root_of(up_from, up);
        up_from.insert(at, root);

        root
    }

    let mut join = |one: i32, other: i32| {
        let a = root_of(&mut up_from, one);
        let b = root_of(&mut up_from, other);

        if a != b {
            up_from.insert(a, b);
        }
    };

    for face in faces {
        join(face.0, face.1);
        join(face.1, face.2);
    }

    let keys: Vec<i32> = up_from.keys().copied().collect();
    let mut roots: Vec<i32> = keys
        .into_iter()
        .map(|key| root_of(&mut up_from, key))
        .collect();
    roots.sort();
    roots.dedup();

    roots.len()
}

/// Every directed edge of a face list, with how many faces lay it.
fn the_directed_edges_of(faces: &Faces) -> HashMap<(i32, i32), i32> {
    let mut laid = HashMap::with_capacity(3 * faces.len());

    for face in faces {
        for pair in [(face.0, face.1), (face.1, face.2), (face.2, face.0)] {
            *laid.entry(pair).or_insert(0) += 1;
        }
    }

    laid
}

/// Whether one face is the given triangle of pool indices, up to rotation but
/// not winding.
fn the_same_triangle(face: (i32, i32, i32), triangle: &[i32; 3]) -> bool {
    (face.0 == triangle[0] && face.1 == triangle[1] && face.2 == triangle[2])
        || (face.1 == triangle[0] && face.2 == triangle[1] && face.0 == triangle[2])
        || (face.2 == triangle[0] && face.0 == triangle[1] && face.1 == triangle[2])
}

/// The stations of a run whose vertex the junction predicate reads as a
/// junction.
fn the_junction_stations_of(run: &SectionRun) -> Vec<i32> {
    (0..run.count())
        .filter(|&i| {
            let vertex = run.edges[i as usize].from_vertex();
            vertex == 19 || vertex == 59
        })
        .collect()
}

/// How far a point stands off the part's skin, read as the
/// nearest face whose own triangle holds the point's projection.
fn the_distance_to_the_parts_surface(part: &Mesh, at: Vector3) -> f64 {
    let mut nearest = f64::MAX;

    for drawn in part.faces.iter() {
        let a = Vector3::from_vector3_float(part.vertices[drawn.v0 as usize]);
        let b = Vector3::from_vector3_float(part.vertices[drawn.v1 as usize]);
        let c = Vector3::from_vector3_float(part.vertices[drawn.v2 as usize]);
        let normal = (b - a).cross(c - a).get_normal();
        let whole = (b - a).cross(c - a).dot(normal);

        if whole.abs() < 1e-300 {
            continue;
        }

        let beta = (at - a).cross(c - a).dot(normal) / whole;
        let gamma = (b - a).cross(at - a).dot(normal) / whole;

        if beta < -1e-9 || gamma < -1e-9 || 1.0 - beta - gamma < -1e-9 {
            continue;
        }

        nearest = nearest.min((at - a).dot(normal).abs());
    }

    nearest
}

/// How far INTO the material a point stands, read as the faces whose
/// own triangle holds its projection; a point standing proud of the skin reads 0.
fn the_depth_inside_the_part(part: &Mesh, at: Vector3) -> f64 {
    let mut nearest_signed = f64::MIN;

    for face in 0..part.faces.len() as i32 {
        if !the_triangle_holds(part, face, at, 0.0) {
            continue;
        }

        let drawn = &part.faces[face as usize];
        let a = Vector3::from_vector3_float(part.vertices[drawn.v0 as usize]);

        nearest_signed = nearest_signed.max((at - a).dot(the_normal_of(part, face)));
    }

    if nearest_signed == f64::MIN {
        0.0
    } else {
        (-nearest_signed).max(0.0)
    }
}

/// How far a point stands off one face's own plane.
fn the_distance_to_the_plane_of(part: &Mesh, face: i32, at: Vector3) -> f64 {
    let a = Vector3::from_vector3_float(part.vertices[part.faces[face as usize].v0 as usize]);

    (at - a).dot(the_normal_of(part, face)).abs()
}
