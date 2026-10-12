//! Port of `Fixtures/BoreTestMeshesEdgeBore.cs`: the block whose bore comes out through its own
//! EDGE - the kernel's cut of "Inner Face Should Bevel.mcx", built exactly and with no boolean
//! under it. A 20 mm cube and a 40 sided bore of radius 5 whose axis runs through the cube's centre
//! along (1, 0, 1)/sqrt 2, so each mouth is a closed loop draped over a convex 90 degree edge and
//! crosses the cube's crease twice, at two of its own vertices. The other bore fixtures are in
//! `bore_test_meshes.rs`.

use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

/// C# `EdgeBoreSides`: how many facets round the edge-bore fixture, which is the part's own count.
pub const EDGE_BORE_SIDES: i32 = 40;

/// C# `EdgeBoreRadius`: that bore's circumradius, measured square to its own axis.
pub const EDGE_BORE_RADIUS: f64 = 5.0;

/// C# `EdgeBoreBlock`: how wide the block is, the part's own 20 mm cube.
pub const EDGE_BORE_BLOCK: f64 = 20.0;

/// C# `EdgeBoreDrillAxis`: 45 degrees in the x z plane, so the drill runs corner to corner of the
/// block's section and leaves through an EDGE rather than a face.
pub fn edge_bore_drill_axis() -> Vector3 {
    Vector3::new(0.5_f64.sqrt(), 0.0, 0.5_f64.sqrt())
}

/// C# `CreateTheEdgeBoreFixture`: the edge-bore fixture at the dimensions every suite reads.
pub fn create_the_edge_bore_fixture() -> Mesh {
    create_block_with_a_bore_through_its_edge(EDGE_BORE_SIDES, EDGE_BORE_RADIUS, EDGE_BORE_BLOCK)
}

/// C# `CreateBlockWithABoreThroughItsEdge`: a block with a round bore drilled through it at 45
/// degrees THROUGH ITS OWN EDGE, so each mouth is a hole draped over a convex 90 degree edge - half
/// of it on the top face and half on the side.
///
/// Built and not subtracted: the part's Subtract leaves two vertices one ulp apart at each
/// crossing, while built the crossing is ONE vertex. The top face and the side face a mouth is
/// draped over are tiled together as one bent surface unfolded into (u, v), and every triangle that
/// straddles the crease is cut on it, so no triangle is bent across the block's own edge.
pub fn create_block_with_a_bore_through_its_edge(sides: i32, radius: f64, block: f64) -> Mesh {
    let half = block / 2.0;
    let axis = edge_bore_drill_axis();
    let across = Vector3::new(axis.z, 0.0, -axis.x);
    let along = Vector3::new(0.0, 1.0, 0.0);
    let count = sides as usize;
    let mut upper = Vec::with_capacity(count);

    for i in 0..sides {
        let turn = 2.0 * std::f64::consts::PI * f64::from(i) / f64::from(sides);
        let on_the_section = (across * (radius * turn.cos())) + (along * (radius * turn.sin()));

        // Slid along the drill's own axis until it reaches whichever of the two faces bounds it
        // first - the top or the side - which is what drapes the mouth over the edge between them.
        let to_the_top = (half - on_the_section.z) / axis.z;
        let to_the_side = (half - on_the_section.x) / axis.x;
        let slid = on_the_section + (axis * to_the_top.min(to_the_side));

        // Set exactly ON the face it leaves through rather than within a rounding of it, so the
        // crossing's two vertices are the same point read either way round.
        upper.push(if to_the_top <= to_the_side {
            Vector3::new(slid.x, slid.y, half)
        } else {
            Vector3::new(half, slid.y, slid.z)
        });
    }

    // The block and the drill are both symmetric through the origin, so the lower mouth is the
    // upper one reflected, half a turn round.
    let lower: Vec<Vector3> = (0..count)
        .map(|i| -upper[(i + (count / 2)) % count])
        .collect();

    let mut mesh = Mesh::new();

    tile_the_bent_faces_around(&mut mesh, &upper, half, false);
    tile_the_bent_faces_around(&mut mesh, &upper, half, true);

    mesh.create_face(&[
        Vector3::new(-half, half, -half),
        Vector3::new(-half, half, half),
        Vector3::new(half, half, half),
        Vector3::new(half, half, -half),
    ]);
    mesh.create_face(&[
        Vector3::new(-half, -half, -half),
        Vector3::new(half, -half, -half),
        Vector3::new(half, -half, half),
        Vector3::new(-half, -half, half),
    ]);

    for i in 0..count {
        let next = (i + 1) % count;

        // The bore's wall, whose outward normal points IN toward the axis because the material of
        // a block is outside its bore.
        mesh.create_face(&[upper[i], upper[next], lower[next], lower[i]]);
    }

    mesh.clean_and_merge();

    mesh
}

type Laid = (Vector2, Vector2, Vector2);

/// C# `TileTheBentFacesAround`: tiles the two block faces one mouth is draped over, as one bent
/// surface unfolded into (u, v) - u runs from the far edge of the first face, across the crease at
/// u = half and on down the second, v is y throughout.
fn tile_the_bent_faces_around(mesh: &mut Mesh, mouth: &[Vector3], half: f64, reflected: bool) {
    let flat: Vec<Vector2> = mouth
        .iter()
        .map(|at| {
            // Which face it stands on is asked as which coordinate has reached the block, not as a
            // comparison against half that a rounding either way could turn over.
            Vector2::new(
                if at.z >= at.x {
                    at.x
                } else {
                    (2.0 * half) - at.z
                },
                at.y,
            )
        })
        .collect();

    let outline = [
        Vector2::new(-half, -half),
        Vector2::new(half, -half),
        Vector2::new(3.0 * half, -half),
        Vector2::new(3.0 * half, half),
        Vector2::new(half, half),
        Vector2::new(-half, half),
    ];

    let into_space = |at: Vector2| {
        let placed = if at.x <= half {
            Vector3::new(at.x, at.y, half)
        } else {
            Vector3::new(half, at.y, (2.0 * half) - at.x)
        };

        if reflected {
            -placed
        } else {
            placed
        }
    };

    for laid in walk_the_ring_between(&flat, &outline, Vector2::new(half, 0.0)) {
        for (a, b, c) in cut_on_the_crease(laid, half) {
            // A point reflection turns a mesh inside out, so the other mouth's faces are wound back.
            if reflected {
                mesh.create_face(&[into_space(a), into_space(c), into_space(b)]);
            } else {
                mesh.create_face(&[into_space(a), into_space(b), into_space(c)]);
            }
        }
    }
}

/// C# `WalkTheRingBetween`, the 2D form of `TileTheRingBetween`: walks an inner loop and an outer
/// one together in ANGLE about a point both contain, and hands back the triangles between the
/// spokes.
fn walk_the_ring_between(mouth: &[Vector2], rim: &[Vector2], about: Vector2) -> Vec<Laid> {
    let mut laid = Vec::new();
    let sides = mouth.len();
    let rim_sides = rim.len();
    let start = (mouth[0].y - about.y).atan2(mouth[0].x - about.x);
    let ahead = |at: Vector2| {
        let turn = (at.y - about.y).atan2(at.x - about.x) - start;

        if turn < 0.0 {
            turn + (2.0 * std::f64::consts::PI)
        } else {
            turn
        }
    };

    let mut inner = vec![0.0; sides + 1];

    for (k, slot) in inner.iter_mut().enumerate().take(sides).skip(1) {
        *slot = ahead(mouth[k]);
    }

    inner[sides] = 2.0 * std::f64::consts::PI;

    // Which of the outline's vertices stands just past the mouth's first, so the walk starts on a
    // spoke that spans less than one step of either loop.
    let mut first = 0;
    let mut least = f64::MAX;

    for (k, &at) in rim.iter().enumerate() {
        let turn = ahead(at);

        if turn > 0.0 && turn < least {
            least = turn;
            first = k;
        }
    }

    let mut outer = vec![0.0; rim_sides + 1];
    outer[0] = least;

    for k in 1..=rim_sides {
        let turn = ahead(rim[(first + k) % rim_sides]);
        outer[k] = if turn <= outer[k - 1] {
            turn + (2.0 * std::f64::consts::PI)
        } else {
            turn
        };
    }

    let mut walked = 0;
    let mut turned = 0;

    while walked < sides || turned < rim_sides {
        if walked < sides && (turned >= rim_sides || inner[walked + 1] <= outer[turned + 1]) {
            laid.push((
                mouth[walked],
                rim[(first + turned) % rim_sides],
                mouth[(walked + 1) % sides],
            ));
            walked += 1;
        } else {
            laid.push((
                mouth[walked % sides],
                rim[(first + turned) % rim_sides],
                rim[(first + turned + 1) % rim_sides],
            ));
            turned += 1;
        }
    }

    laid
}

/// C# `CutOnTheCrease`: cuts one laid triangle on the crease line u = `at`, so nothing the walk
/// lays is bent across the block's own edge. A triangle is convex, so each side of the cut is one
/// convex polygon and a fan off its first vertex tiles it.
fn cut_on_the_crease(laid: Laid, at: f64) -> Vec<Laid> {
    let corners = [laid.0, laid.1, laid.2];
    let below = corners.iter().any(|corner| corner.x < at);
    let above = corners.iter().any(|corner| corner.x > at);

    if !below || !above {
        return vec![laid];
    }

    let mut pieces = Vec::new();

    for keeping in [true, false] {
        let mut kept = Vec::new();

        for k in 0..3 {
            let from = corners[k];
            let to = corners[(k + 1) % 3];
            let from_in = if keeping { from.x <= at } else { from.x >= at };
            let to_in = if keeping { to.x <= at } else { to.x >= at };

            if from_in {
                kept.push(from);
            }

            if from_in != to_in {
                kept.push(Vector2::new(
                    at,
                    from.y + ((to.y - from.y) * ((at - from.x) / (to.x - from.x))),
                ));
            }
        }

        for k in 1..kept.len().saturating_sub(1) {
            pieces.push((kept[0], kept[k], kept[k + 1]));
        }
    }

    pieces
}
