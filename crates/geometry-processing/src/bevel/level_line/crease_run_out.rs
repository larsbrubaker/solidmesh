//! A fillet's RUN-OUT along a crease nobody picked ([`CreaseRunOut`]): the type, its constructor
//! and closed-form readings along the
//! dual spine ([`CreaseRunOut::centre_at`], [`CreaseRunOut::wall_contact_at`],
//! [`CreaseRunOut::tube_contact_at`]), the turn it ends at, the reading at one corner
//! ([`CreaseRunOut::read`]), an edge's sense ([`CreaseRunOut::sense`]) and the finder over a pick
//! ([`CreaseRunOut::find`]).

use indexmap::IndexMap;
use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};

/// The sine below which two faces read as one plane, or an edge reads as
/// neither convex nor concave - dimensionless, so it does not grow with the part.
pub const FLAT: f64 = 1e-6;

/// A fillet's RUN-OUT along a crease nobody picked. At a pick corner where a
/// convex chain meets a concave one, the convex chain's round does not stop on the unpicked concave
/// crease its side face ends on - the concave chain's void ball keeps rolling round the round and
/// dies tangentially on that crease, a way along it (4.5846 radii on the Top Center Face nose).
/// This reads where, in closed form.
///
/// **The corner.** A picked convex edge and a picked concave edge share a corner vertex and a
/// SHARED face (the nose's upper face). The convex edge's other face is its SIDE face (the lower
/// face); the concave edge's other face is its WALL (the notch slope). The run-out exists where an
/// unpicked concave feature crease leaves the corner between the side face and the wall. A picked
/// crease, a crease between other faces, or no opposite-sense chain at the corner is not this case.
///
/// **The law** (measured against the Onshape control in `BevelTopCenterFaceCanalTests`). The void
/// ball of radius r stays r OUTSIDE the wall and 2r from the round's own axis (it touches the
/// round's tube from outside), so its centre runs an ellipse - the dual spine E. Turned from the
/// shared face's normal to the side face's, E is the corner canal (112.62 degrees on the nose);
/// turned further, the ball's contact with the wall leaves the crease and comes back to it, and the
/// first return is where the run-out ends (158.5036 degrees). There the centre stands exactly r
/// from the crease's line: the ball touches the sharp edge at a point, which is why the blend fades
/// out instead of cutting a step. Nothing here depends on the frozen Surface namespace;
/// `SurfaceMixedCornerDualFrame` is the same spine written for it.
///
/// `BandGroupFields` reads it for the run-out field (gated off by default). There is no profile
/// filter - a chamfer takes a run-out too. On the nose both faces of the wedge land in one group,
/// so one corner hole spans both ridge ends and the apex strip.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CreaseRunOut {
    /// Where the round's axis meets the wall's plane: the origin the ball's centre slides from.
    pub rail_centre: Vector3,
    /// The unit direction of the convex round's axis.
    pub axis_direction: Vector3,
    /// The wall's unit normal, out of the wall.
    pub out_of_the_wall: Vector3,
    /// `axis_direction . out_of_the_wall`: how fast sliding along the axis leaves the wall.
    pub along_the_axis_out_of_the_wall: f64,
    /// The unit direction square to the axis toward the shared face - the canal's turn zero.
    pub towards_the_shared_face: Vector3,
    /// The unit direction square to the axis and to
    /// [`CreaseRunOut::towards_the_shared_face`], toward the side face - a quarter turn on.
    pub towards_the_side_square: Vector3,
    /// The pick corner's vertex.
    pub corner: i32,
    /// The unpicked crease's far vertex.
    pub crease_end: i32,
    /// Where the corner stands.
    pub corner_point: Vector3,
    /// The unit direction along the crease, away from the corner.
    pub crease_direction: Vector3,
    /// The crease's length as one mesh edge - past it the run-out does not fit.
    pub crease_length: f64,
    /// The fillet radius the run-out was read at.
    pub size: f64,
    /// The turn the corner canal ends at: from the shared face's normal to the side face's.
    pub canal_turn_radians: f64,
    /// The turn the run-out ends at: the wall contact's first return onto the crease past the
    /// canal.
    pub end_turn_radians: f64,
    /// How far along the crease the run-out ends (P*).
    pub reach: f64,
    /// The wall contact T(theta) sampled from the canal's end to the run-out's.
    pub wall_contact: Vec<Vector3>,
    /// The contact on the round's tube, sampled at the same turns as
    /// [`CreaseRunOut::wall_contact`].
    pub tube_contact: Vec<Vector3>,
}

impl CreaseRunOut {
    /// The run-out at `corner` along the crease to `crease_end`, before
    /// the turn it ends at is read.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        corner: i32,
        crease_end: i32,
        corner_point: Vector3,
        crease_direction: Vector3,
        crease_length: f64,
        size: f64,
        rail_centre: Vector3,
        axis_direction: Vector3,
        out_of_the_wall: Vector3,
        towards_the_shared_face: Vector3,
        towards_the_side_square: Vector3,
        canal_turn_radians: f64,
    ) -> Self {
        CreaseRunOut {
            rail_centre,
            axis_direction,
            out_of_the_wall,
            along_the_axis_out_of_the_wall: axis_direction.dot(out_of_the_wall),
            towards_the_shared_face,
            towards_the_side_square,
            corner,
            crease_end,
            corner_point,
            crease_direction,
            crease_length,
            size,
            canal_turn_radians,
            end_turn_radians: 0.0,
            reach: 0.0,
            wall_contact: Vec::new(),
            tube_contact: Vec::new(),
        }
    }

    /// The run-out's end on the crease, P*.
    pub fn end(&self) -> Vector3 {
        self.corner_point + (self.crease_direction * self.reach)
    }

    /// Where the void ball's centre stands at `turn`: on the dual
    /// spine E.
    pub fn centre_at(&self, turn: f64) -> Vector3 {
        self.rail_centre
            + (self.axis_direction * self.slide_at(turn))
            + (self.out_of_the_axis(turn) * (2.0 * self.size))
    }

    /// Where the ball touches the wall at `turn`: T(theta).
    pub fn wall_contact_at(&self, turn: f64) -> Vector3 {
        self.centre_at(turn) - (self.out_of_the_wall * self.size)
    }

    /// Where the ball touches the round's tube at `turn`.
    pub fn tube_contact_at(&self, turn: f64) -> Vector3 {
        self.rail_centre
            + (self.axis_direction * self.slide_at(turn))
            + (self.out_of_the_axis(turn) * self.size)
    }

    /// The unit direction square to the axis at `turn`, from the
    /// shared face's side toward the side face's.
    pub fn out_of_the_axis(&self, turn: f64) -> Vector3 {
        (self.towards_the_shared_face * turn.cos()) + (self.towards_the_side_square * turn.sin())
    }

    /// How far along the axis the centre slides to stay r outside the
    /// wall at `turn`.
    pub fn slide_at(&self, turn: f64) -> f64 {
        (self.size - (2.0 * self.size * self.out_of_the_wall.dot(self.out_of_the_axis(turn))))
            / self.along_the_axis_out_of_the_wall
    }

    /// The first turn past the canal's end at which the
    /// wall contact comes back onto the crease's line, or `None`.
    ///
    /// The contact leaves the crease early in the turn - it crosses the side face's root and comes
    /// out on the wall beyond - so the crossing wanted is the one on the way BACK, and the scan
    /// starts at the canal's end. Found by sign change of the contact's offset from the crease's
    /// line, across it in the wall, then bisected; the scan stops at twice the canal's turn.
    pub fn try_the_turn_it_ends_at(&self) -> Option<f64> {
        let across = self.crease_direction.cross(self.out_of_the_wall);
        let off = |turn: f64| (self.wall_contact_at(turn) - self.corner_point).dot(across);

        let mut from = self.canal_turn_radians;
        let sign = off(from) > 0.0;
        let step = self.canal_turn_radians / 512.0;
        let mut to = from + step;
        while to <= 2.0 * self.canal_turn_radians {
            if (off(to) > 0.0) == sign {
                from = to;
                to += step;
                continue;
            }

            let mut lo = from;
            let mut hi = to;
            for _ in 0..100 {
                let mid = (lo + hi) / 2.0;
                if (off(mid) > 0.0) == sign {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }

            return Some((lo + hi) / 2.0);
        }

        None
    }

    /// The run-out at `corner` along the crease to `crease_end`, the
    /// convex round running from the corner toward `along_the_round` between the shared and side
    /// faces, against the wall; `None` where the round lies in the wall, the faces are one plane,
    /// or the contact never returns to the crease ahead of the corner.
    #[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
    pub fn read(
        positions: &[Vector3],
        corner: i32,
        crease_end: i32,
        along_the_round: i32,
        shared_normal: Vector3,
        side_normal: Vector3,
        wall_normal: Vector3,
        size: f64,
        samples: i32,
    ) -> Option<CreaseRunOut> {
        let at = positions[corner as usize];
        let axis_direction = (positions[along_the_round as usize] - at).get_normal();
        let along_the_axis_out_of_the_wall = axis_direction.dot(wall_normal);

        // A round lying IN the wall meets the plane r outside it in lines or nothing: no ellipse
        // to roll on.
        if !(along_the_axis_out_of_the_wall.abs() > FLAT) {
            return None;
        }

        // The round's axis: r inside both its faces, beside the corner. Where it crosses the
        // wall's plane (which the corner lies in) is the spine's rail centre.
        let cosine = shared_normal.dot(side_normal);
        let axis_point = at - ((shared_normal + side_normal) * (size / (1.0 + cosine)));
        let rail_centre = axis_point
            + (axis_direction
                * ((at - axis_point).dot(wall_normal) / along_the_axis_out_of_the_wall));

        let square = side_normal - (shared_normal * cosine);
        if !(square.length() > FLAT) {
            return None;
        }

        let crease_vector = positions[crease_end as usize] - at;
        let mut run_out = CreaseRunOut::new(
            corner,
            crease_end,
            at,
            crease_vector.get_normal(),
            crease_vector.length(),
            size,
            rail_centre,
            axis_direction,
            wall_normal,
            shared_normal,
            square.get_normal(),
            cosine.clamp(-1.0, 1.0).acos(),
        );

        let ends_at = run_out.try_the_turn_it_ends_at()?;
        let reach = (run_out.wall_contact_at(ends_at) - at).dot(run_out.crease_direction);
        if !(reach > 0.0) {
            return None;
        }

        run_out.end_turn_radians = ends_at;
        run_out.reach = reach;
        let turns: Vec<f64> = (0..=samples)
            .map(|i| {
                run_out.canal_turn_radians
                    + ((ends_at - run_out.canal_turn_radians) * i as f64 / samples as f64)
            })
            .collect();
        run_out.wall_contact = turns.iter().map(|&t| run_out.wall_contact_at(t)).collect();
        run_out.tube_contact = turns.iter().map(|&t| run_out.tube_contact_at(t)).collect();
        Some(run_out)
    }

    /// +1 for a concave edge, -1 for a convex one, 0 for flat, not
    /// two-sided or not in the mesh.
    pub fn sense(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        standing: &StandingMap,
        key: (i32, i32),
    ) -> i32 {
        let Some(faces) = standing.get(&key) else {
            return 0;
        };
        if faces.len() != 2 {
            return 0;
        }

        let (a, b, c) = indices[faces[1] as usize];
        let opposite = if a != key.0 && a != key.1 {
            a
        } else if b != key.0 && b != key.1 {
            b
        } else {
            c
        };
        let to_opposite = positions[opposite as usize] - positions[key.0 as usize];
        let rise = triangle_edges::normal(positions, indices[faces[0] as usize]).dot(to_opposite)
            / to_opposite.length();
        if rise > FLAT {
            1
        } else if rise < -FLAT {
            -1
        } else {
            0
        }
    }

    /// Appends `key` to `vertex`'s list in `map`, making the list where
    /// there is none (insertion order kept, so the corners are read in the same order every run).
    pub fn add(map: &mut IndexMap<i32, Vec<(i32, i32)>>, vertex: i32, key: (i32, i32)) {
        map.entry(vertex).or_default().push(key);
    }

    /// How far `p` stands off the crease's line.
    pub fn distance_to_crease_line(&self, p: Vector3) -> f64 {
        let from = p - self.corner_point;
        (from - (self.crease_direction * from.dot(self.crease_direction))).length()
    }
}

impl CreaseRunOut {
    /// Every run-out at a corner where a picked convex chain edge and a
    /// picked concave one share a face and an unpicked concave feature crease leaves the corner
    /// between the convex edge's side face and the concave edge's wall. `chains` are the picked
    /// chains as vertex lists (`closed` per chain); `is_feature_edge` says whether a mesh edge is a
    /// crease at all. Corners are visited in vertex order; `samples` is normally 16.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn find(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        is_feature_edge: &dyn Fn(i32, i32) -> bool,
        size: f64,
        samples: i32,
    ) -> Vec<CreaseRunOut> {
        let standing = triangle_edges::standing_map(indices);
        let mut chain_keys = std::collections::HashSet::new();
        let mut at_vertex: IndexMap<i32, Vec<(i32, i32)>> = IndexMap::new();

        for (k, chain) in chains.iter().enumerate() {
            let edges = if closed[k] {
                chain.len()
            } else {
                chain.len() - 1
            };

            for i in 0..edges {
                let key = triangle_edges::key(chain[i], chain[(i + 1) % chain.len()]);
                if chain_keys.insert(key) {
                    CreaseRunOut::add(&mut at_vertex, key.0, key);
                    CreaseRunOut::add(&mut at_vertex, key.1, key);
                }
            }
        }

        let n = |t: i32| triangle_edges::normal(positions, indices[t as usize]);
        let coplanar = |a: i32, b: i32| n(a).dot(n(b)) > 1.0 - FLAT;
        let sense = |key: (i32, i32)| CreaseRunOut::sense(positions, indices, &standing, key);

        at_vertex.sort_keys();

        let mut found = Vec::new();

        for (&corner, edges) in &at_vertex {
            for &convex in edges.iter().filter(|&&e| sense(e) < 0) {
                for &concave in edges.iter().filter(|&&e| sense(e) > 0) {
                    // The shared face stands on both chain edges; each edge's other face is its
                    // side / wall.
                    let convex_faces = &standing[&convex];
                    let concave_faces = &standing[&concave];
                    let shared: Vec<(i32, i32)> = convex_faces
                        .iter()
                        .flat_map(|&a| {
                            concave_faces
                                .iter()
                                .filter(move |&&b| coplanar(a, b))
                                .map(move |&b| (a, b))
                        })
                        .collect();

                    if shared.len() != 1 {
                        continue;
                    }

                    let side = *convex_faces
                        .iter()
                        .find(|&&t| t != shared[0].0)
                        .expect("a convex chain edge stands on two faces");
                    let wall = *concave_faces
                        .iter()
                        .find(|&&t| t != shared[0].1)
                        .expect("a concave chain edge stands on two faces");

                    for (&crease_key, faces) in standing.iter().filter(|(k, _)| {
                        (k.0 == corner || k.1 == corner) && !chain_keys.contains(*k)
                    }) {
                        if faces.len() != 2
                            || !is_feature_edge(crease_key.0, crease_key.1)
                            || !(sense(crease_key) > 0)
                        {
                            continue;
                        }

                        let on_side = faces
                            .iter()
                            .copied()
                            .find(|&t| coplanar(t, side))
                            .unwrap_or(-1);
                        let on_wall = faces
                            .iter()
                            .copied()
                            .find(|&t| t != on_side && coplanar(t, wall))
                            .unwrap_or(-1);

                        if on_side < 0 || on_wall < 0 {
                            continue;
                        }

                        let far = if crease_key.0 == corner {
                            crease_key.1
                        } else {
                            crease_key.0
                        };
                        let along = if convex.0 == corner {
                            convex.1
                        } else {
                            convex.0
                        };

                        if let Some(run_out) = CreaseRunOut::read(
                            positions,
                            corner,
                            far,
                            along,
                            n(shared[0].0),
                            n(side),
                            n(wall),
                            size,
                            samples,
                        ) {
                            found.push(run_out);
                        }
                    }
                }
            }
        }

        found
    }
}
