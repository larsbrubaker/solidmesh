//! A strip ENDED on the part's own hull, built - the tapered slab's block foot filleted at 1.5 mm,
//! where the ledge the fillet lands on is only 1 mm wide at the far end, so the last millimetre of
//! the band runs off it and the strip leaves the part instead of stopping on a face
//! (`SurfaceStripEndsOnTheHull`). The volume is checked against a numeric integral of the section
//! the fillet really adds, and the cap areas against the closed-form polygons the end reading's own
//! points bound. The reading itself is `surface_strip_ends_on_the_hull_tests`.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_chain_join, surface_mesh_closure, surface_room_measure,
    surface_strip_ends, SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan, SurfaceLine,
    SurfaceStrip, SurfaceStripEndsOnTheHull, SurfaceVertexPool,
};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::{Vector2, Vector3};
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::tapered_ledge_test_meshes::{
    self, create_wall_of_widths, create_wall_tapering_to_a_knife_edge, length_of, PINCH,
    TAPER_RATE, WIDTH_AT_THE_RIM,
};

/// The size: half a millimetre more setback than the ledge has left at its narrow end.
const RADIUS: f64 = 1.5;

/// How many segments a quarter turn of the fillet is chorded at.
const SEGMENTS: i32 = 8;

/// The slab without any bevel: the plate over its own tapering footprint, and the block on it.
const THE_PLAIN_SLAB: f64 = 820.0;

/// The whole thing, closed: the end the build used to refuse now carries the two cap pieces, and
/// the part is one closed manifold solid with no fold in it.
#[test]
fn the_end_closes_on_the_parts_own_hull() {
    let built = the_slab(RADIUS, false);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built.mesh);
    assert_eq!(
        open, 0,
        "every edge is used twice, once each way - instead open at {open_at}"
    );

    let (folded, folded_at) = surface_mesh_closure::folded_edge_count(&the_faces_of(&built.mesh));
    assert_eq!(
        folded, 0,
        "and nothing is laid over anything - instead folded at {folded_at:?}"
    );

    assert!(built.mesh.is_manifold());

    assert_eq!(
        shell_count(&built.mesh),
        1,
        "one solid, with the fillet joined onto it rather than standing beside it"
    );
}

/// And the solid is the fillet the part really gains: the section between the corner and the
/// chorded profile, clipped at the plate's side wherever the ledge has run out from under it,
/// integrated along the block (Simpson's over 4000 steps a piece, split at x = 9 where the
/// integrand has its one corner).
#[test]
fn the_volume_is_the_fillet_it_adds() {
    let built = the_slab(RADIUS, false);
    let added = signed_volume(&built.mesh) - THE_PLAIN_SLAB;
    let integral = the_fillets_volume(RADIUS);

    assert!(
        (added - integral).abs() <= 1e-3,
        "the fillet adds {integral:.5} by the integral, and the part gained {added:.5}"
    );
}

/// The two pieces of cap are the sections the two planes really take: measured off the published
/// mesh as the triangles standing in each plane where the part had no face before, and checked
/// against the closed-form polygons the reading's own points bound.
#[test]
fn the_caps_are_the_sections_the_two_planes_take() {
    let built = the_slab(RADIUS, false);
    let length = length_of();

    // The far piece stands in the plane the block ENDS in, on the side of the chain the ledge is -
    // where the plate's own end face stops at z = 0, so anything above that is the cap's.
    let at_the_end = area_of_the_faces_where(&built.mesh, |at| {
        (at.x - length).abs() < 1e-6 && at.y <= 1e-6 && at.z >= -1e-6
    });

    // And the run-off piece stands in the plate's tapering side, above that wall's own top rim.
    let side = Vector3::new(TAPER_RATE, -1.0, 0.0).get_normal();
    let on_the_side = area_of_the_faces_where(&built.mesh, |at| {
        (at - Vector3::new(0.0, -WIDTH_AT_THE_RIM, 0.0))
            .dot(side)
            .abs()
            < 1e-6
            && at.z >= -1e-6
    });

    assert!(
        (at_the_end - built.far_cap).abs() <= 1e-6,
        "the section at the block's end is {:.5} by the reading's own points - instead {at_the_end:.5}",
        built.far_cap
    );

    assert!(
        (on_the_side - built.run_off_cap).abs() <= 1e-6,
        "and the one on the plate's side is {:.5} - instead {on_the_side:.5}",
        built.run_off_cap
    );

    assert!(
        built.run_off_cap > 0.0,
        "the run-off really does leave a piece of cap on the plane it ran off onto"
    );
}

/// The knife edge's own split point - where the fillet's tangent line dies on the ledge's rim - is
/// SEWN into the faces beside it rather than standing in the middle of one of their edges.
#[test]
fn the_knife_edges_split_point_is_sewn_in() {
    let built = the_slab(RADIUS, false);
    let mut carried = 0;

    for face in built.mesh.faces.iter() {
        for corner in [face.v0, face.v1, face.v2] {
            if (Vector3::from_vector3_float(built.mesh.vertices[corner as usize])
                - built.where_the_tangent_dies)
                .length()
                < 1e-6
            {
                carried += 1;
            }
        }
    }

    assert!(
        carried >= 4,
        "the point at {} is a corner of the strip, of the cap and of the faces either side of the \
         rim - instead {carried} faces carry it",
        built.where_the_tangent_dies
    );
}

/// And nothing is minted twice: no two vertices of the answer stand at one place.
#[test]
fn nothing_is_minted_twice() {
    let built = the_slab(RADIUS, false);
    let weld = SurfaceVertexPool::weld_distance_for(&built.mesh.vertices);
    let mut twice = 0;
    let mut at = Vector3::ZERO;
    let vertices: Vec<Vector3> = built
        .mesh
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();

    for one in 0..vertices.len() {
        for other in one + 1..vertices.len() {
            if (vertices[one] - vertices[other]).length() <= weld {
                twice += 1;
                at = vertices[one];
            }
        }
    }

    assert_eq!(
        twice, 0,
        "two vertices a weld apart are one point said twice - instead {twice}, one of them at {at}"
    );

    // And nothing EXTRA is minted where the end closes: the plane the block ends in carries the
    // seven rails that leave through it, the one point the end turns at, the part's own corner
    // where the knife edge dies, the chain's own corner, and the block's own top corner - eleven.
    let length = length_of();
    let standing = built
        .mesh
        .vertices
        .iter()
        .filter(|vertex| {
            (vertex.x as f64 - length).abs() < 1e-6
                && vertex.y as f64 <= 1e-6
                && vertex.z as f64 >= -1e-6
        })
        .count();

    assert_eq!(
        standing, 11,
        "seven rails, the turn, the apex, the chain's corner and the block's top - instead {standing} \
         vertices stand in that plane"
    );
}

/// And the walk is not the shape: the same slab with its taper the other way round builds the
/// mirror image of the same solid, to the same volume and the same cap areas.
#[test]
fn the_mirrored_walk_gives_the_same_solid() {
    let built = the_slab(RADIUS, false);
    let mirrored = the_slab(RADIUS, true);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mirrored.mesh);
    assert_eq!(
        open, 0,
        "the mirror closes as well - instead open at {open_at}"
    );

    assert!(
        (signed_volume(&mirrored.mesh) - signed_volume(&built.mesh)).abs() <= 1e-6,
        "and holds the same material, whichever way the chain was walked"
    );

    assert!((mirrored.far_cap - built.far_cap).abs() <= 1e-9);
    assert!((mirrored.run_off_cap - built.run_off_cap).abs() <= 1e-9);
}

/// And a strip that ran off BOTH of its landings at one end is refused by name rather than closed
/// on whichever side was read first. Driven directly, since no fixture makes one.
#[test]
fn a_strip_that_ran_off_both_its_landings_is_refused_by_name() {
    let mut both: Vec<[Option<SurfaceBandRunsOffOverAStretch>; 2]> = vec![[None, None]];

    for (side, slot) in both[0].iter_mut().enumerate() {
        *slot = Some(
            SurfaceBandRunsOffOverAStretch::new(
                true,
                1.0,
                FeatureEdge::default(),
                Vector3::UNIT_Z,
                Vector3::ZERO,
                Arc::new(HashSet::new()),
            )
            .on(0, 0, side as i32),
        );
    }

    let failure = surface_strip_ends::try_the_stretch_at(Some(&both), 0, true, Vector3::ZERO)
        .expect_err("two run-offs at one end of one strip");

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStripRunsOffBothSides,
        "named as the pair it is - instead {:?}",
        failure.code()
    );

    // And one side alone is the ordinary case, whichever side it is.
    for side in 0..2 {
        let mut one: Vec<[Option<SurfaceBandRunsOffOverAStretch>; 2]> = vec![[None, None]];

        one[0][side] = both[0][side].clone();

        let stretch = surface_strip_ends::try_the_stretch_at(Some(&one), 0, true, Vector3::ZERO)
            .unwrap_or_else(|failure| panic!("{failure}"));

        assert_eq!(
            stretch.expect("one side ran off").side,
            side as i32,
            "and it is the side that really ran off"
        );
    }

    let at_the_other_end =
        surface_strip_ends::try_the_stretch_at(Some(&both), 0, false, Vector3::ZERO)
            .expect("while the other end of the same strip has nothing standing at it");

    assert!(at_the_other_end.is_none());
}

/// The slab built at one size, with the end reading's own numbers: the cap areas in closed form
/// and the point the run-off side's tangent line dies at.
struct TheSlab {
    mesh: Mesh,
    run_off_cap: f64,
    far_cap: f64,
    where_the_tangent_dies: Vector3,
}

fn the_slab(size: f64, mirrored: bool) -> TheSlab {
    let length = length_of();

    // The mirror is the same taper read the other way along the block, which is the same solid
    // turned about and so the same chain walked the other way.
    let part = if mirrored {
        create_wall_of_widths(&[(0.0, PINCH), (length, WIDTH_AT_THE_RIM)])
    } else {
        create_wall_tapering_to_a_knife_edge(false)
    };

    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        FeatureEdge::default_flat_angle_degrees(),
    ));
    let features = FeatureEdgeSet::create(&part);

    let feet: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            edge.start.y.abs() < 1e-9
                && edge.end.y.abs() < 1e-9
                && edge.start.z.abs() < 1e-9
                && edge.end.z.abs() < 1e-9
        })
        .copied()
        .collect();
    assert_eq!(feet.len(), 1, "the block has one foot crease");

    let mut chains = surface_chain_join::join(&bevel_chain_linker::link(&feet));
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    let plan: Rc<SurfaceBevelPlan> = SurfaceBevelPlan::try_create(&chain, &regions, size, true)
        .unwrap_or_else(|reason| {
            panic!("the foot crease has to plan at {size}: {:?}", reason.code())
        });

    // The reading the build makes, taken here as well so the test's own numbers come from the
    // construction's points rather than from the mesh it published.
    let mut stretches: Vec<SurfaceBandRunsOffOverAStretch> = Vec::new();

    surface_room_measure::measure(std::slice::from_ref(&plan), None, Some(&mut stretches));

    assert_eq!(stretches.len(), 1);
    let stretch = stretches.remove(0);
    let edge = plan.edges[0].clone();
    let pool = SurfaceVertexPool::new(&part.vertices);

    let strip = SurfaceStrip::try_create(&edge, SEGMENTS)
        .unwrap_or_else(|reason| panic!("and it has to rule: {:?}", reason.code()));

    let ends = SurfaceStripEndsOnTheHull::try_read(
        &pool,
        &strip,
        &edge,
        stretch.at_the_start,
        &SurfaceStripEndsOnTheHull::supporting_planes_of(&part, pool.near_enough()),
        Some(SurfaceLine::new(
            stretch.knife_line.start,
            stretch.knife_line.direction,
        )),
    )
    .unwrap_or_else(|reason| panic!("and the end has to read: {:?}", reason.code()));

    let mesh = surface_bevel_builder::try_build(
        &part,
        &graph,
        &regions,
        std::slice::from_ref(&plan),
        SEGMENTS,
        &CancelToken::new(),
        crate::surface_test_build::build_options(),
    )
    .unwrap_or_else(|reason| panic!("and it has to build: {reason:?}"))
    .mesh;

    let rail_count = strip.rail_count();
    let run_off_rail = if stretch.side == 0 { 0 } else { rail_count - 1 };
    let far_rail = if stretch.side == 0 { rail_count - 1 } else { 0 };
    let apex = pool.position_of(ends.apex);

    let mut run_off = vec![apex];
    let mut far = Vec::new();

    // Each rail put on the piece whose PLANE the reading says it leaves through, walked out from
    // the run-off's own rail: asked of the reading rather than partitioned again here.
    for step in 0..rail_count {
        for (from, piece) in [(run_off_rail, &mut run_off), (far_rail, &mut far)] {
            let rail = if from == 0 {
                step
            } else {
                rail_count - 1 - step
            };

            if ends.plane_of[rail as usize] == ends.plane_of[from as usize] {
                piece.push(strip.point_on(rail, ends.exit_at[rail as usize]));
            }
        }
    }

    run_off.push(ends.between);
    far.push(ends.between);
    far.push(apex);
    far.insert(
        0,
        if stretch.at_the_start {
            edge.from()
        } else {
            edge.to()
        },
    );

    TheSlab {
        mesh,
        run_off_cap: area_of_3d(&run_off),
        far_cap: area_of_3d(&far),
        where_the_tangent_dies: strip.point_on(run_off_rail, ends.exit_at[run_off_rail as usize]),
    }
}

/// The area of a planar polygon, by Newell's own normal.
fn area_of_3d(polygon: &[Vector3]) -> f64 {
    let mut twice = Vector3::ZERO;

    for at in 0..polygon.len() {
        twice += polygon[at].cross(polygon[(at + 1) % polygon.len()]);
    }

    twice.length() / 2.0
}

/// The area of every face of a mesh all three of whose corners answer `where_`.
fn area_of_the_faces_where(mesh: &Mesh, where_: impl Fn(Vector3) -> bool) -> f64 {
    let mut area = 0.0;

    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        if where_(a) && where_(b) && where_(c) {
            area += (b - a).cross(c - a).length() / 2.0;
        }
    }

    area
}

/// The faces of a mesh as the list the closure reading asks for.
fn the_faces_of(mesh: &Mesh) -> Vec<(i32, i32, i32)> {
    mesh.faces
        .iter()
        .map(|face| (face.v0, face.v1, face.v2))
        .collect()
}

/// How much material the fillet really adds, by integrating its cross section along the block.
fn the_fillets_volume(radius: f64) -> f64 {
    let length = length_of();

    // Where the ledge narrows past the setback, which is the only corner the integrand has.
    let runs_out = (WIDTH_AT_THE_RIM - radius) / TAPER_RATE;

    simpson(|x| the_section_at(x, radius), 0.0, runs_out, 4000)
        + simpson(|x| the_section_at(x, radius), runs_out, length, 4000)
}

/// The cross section at one station: the corner, the chorded arc between the two tangent points,
/// and the clip at the plate's side, read in the (y, z) of that station.
fn the_section_at(x: f64, radius: f64) -> f64 {
    let mut section = vec![Vector2::new(0.0, 0.0)];

    for at in 0..=SEGMENTS {
        let turn = at as f64 * (std::f64::consts::PI / 2.0) / SEGMENTS as f64;

        // The ball sits a radius off each face, and the profile runs from its touch on the ledge to
        // its touch on the block - the same chording SurfaceStrip rules the rails by.
        section.push(Vector2::new(
            -radius + (radius * turn.sin()),
            radius - (radius * turn.cos()),
        ));
    }

    // And the plate's own side, past which there is no ledge to hold the fillet up.
    let width = tapered_ledge_test_meshes::width_at(x, WIDTH_AT_THE_RIM, TAPER_RATE, PINCH, false);

    area_of_2d(&clip_to(&section, -width))
}

/// The part of a section standing at or beyond `least_across`, by Sutherland-Hodgman.
fn clip_to(polygon: &[Vector2], least_across: f64) -> Vec<Vector2> {
    let mut kept = Vec::new();

    for at in 0..polygon.len() {
        let here = polygon[at];
        let next = polygon[(at + 1) % polygon.len()];
        let in_here = here.x >= least_across;
        let in_next = next.x >= least_across;

        if in_here {
            kept.push(here);
        }

        if in_here != in_next {
            let along = (least_across - here.x) / (next.x - here.x);

            kept.push(Vector2::new(
                least_across,
                here.y + ((next.y - here.y) * along),
            ));
        }
    }

    kept
}

/// The area of a polygon in the plane, by the shoelace.
fn area_of_2d(polygon: &[Vector2]) -> f64 {
    let mut twice = 0.0;

    for at in 0..polygon.len() {
        let here = polygon[at];
        let next = polygon[(at + 1) % polygon.len()];

        twice += (here.x * next.y) - (next.x * here.y);
    }

    twice.abs() / 2.0
}

/// Simpson's rule over an even number of steps.
fn simpson(of: impl Fn(f64) -> f64, from: f64, to: f64, steps: i32) -> f64 {
    let step = (to - from) / steps as f64;
    let mut total = of(from) + of(to);

    for at in 1..steps {
        total += of(from + (step * at as f64)) * (if at % 2 == 0 { 2.0 } else { 4.0 });
    }

    total * step / 3.0
}
