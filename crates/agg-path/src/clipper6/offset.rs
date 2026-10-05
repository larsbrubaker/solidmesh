//! Clipper 6's `ClipperOffset` (`clipper.cs`): grows or shrinks closed polygons and open lines
//! by a distance with square, round or mitred corners, then unions the raw offset rings with
//! [`Clipper`] to clean up the corners. `Execute(ref PolyTree, delta)` is not ported (nothing
//! in MatterCAD or agg-sharp calls it).

use std::f64::consts::PI;

use super::engine::Clipper;
use super::statics::{get_bounds, orientation};
use super::types::{
    ip, near_zero, round, ClipType, ClipperError, EndType, IntPoint, JoinType, Path, Paths,
    PolyFillType, PolyType,
};

const TWO_PI: f64 = PI * 2.0;
const DEF_ARC_TOLERANCE: f64 = 0.25;

/// C# `DoublePoint`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct DoublePoint {
    x: f64,
    y: f64,
}

/// One added path (C#'s child `PolyNode` of `m_polyNodes`).
#[derive(Clone, Debug)]
struct OffsetNode {
    polygon: Path,
    join_type: JoinType,
    end_type: EndType,
}

/// C# `ClipperOffset`.
#[derive(Clone, Debug)]
pub struct ClipperOffset {
    dest_polys: Paths,
    src_poly: Path,
    dest_poly: Path,
    normals: Vec<DoublePoint>,
    delta: f64,
    sin_a: f64,
    sin: f64,
    cos: f64,
    miter_lim: f64,
    steps_per_rad: f64,
    /// (child, point) of the lowest point of all closed polygons (C#'s `m_lowest`, X = -1
    /// when unset).
    lowest: Option<(usize, usize)>,
    poly_nodes: Vec<OffsetNode>,
    pub arc_tolerance: f64,
    pub miter_limit: f64,
}

impl Default for ClipperOffset {
    fn default() -> Self {
        Self::new(2.0, DEF_ARC_TOLERANCE)
    }
}

impl ClipperOffset {
    /// C# `ClipperOffset(miterLimit = 2.0, arcTolerance = 0.25)`.
    pub fn new(miter_limit: f64, arc_tolerance: f64) -> Self {
        ClipperOffset {
            dest_polys: Paths::new(),
            src_poly: Path::new(),
            dest_poly: Path::new(),
            normals: Vec::new(),
            delta: 0.0,
            sin_a: 0.0,
            sin: 0.0,
            cos: 0.0,
            miter_lim: 0.0,
            steps_per_rad: 0.0,
            lowest: None,
            poly_nodes: Vec::new(),
            arc_tolerance,
            miter_limit,
        }
    }

    /// C# `Clear`.
    pub fn clear(&mut self) {
        self.poly_nodes.clear();
        self.lowest = None;
    }

    /// C# `AddPath`.
    pub fn add_path(&mut self, path: &[IntPoint], join_type: JoinType, end_type: EndType) {
        let mut high_i = path.len() as i32 - 1;
        if high_i < 0 {
            return;
        }

        //strip duplicate points from path and also get index to the lowest point ...
        if end_type == EndType::ClosedLine || end_type == EndType::ClosedPolygon {
            while high_i > 0 && path[0] == path[high_i as usize] {
                high_i -= 1;
            }
        }
        let mut polygon = Vec::with_capacity(high_i as usize + 1);
        polygon.push(path[0]);
        let (mut j, mut k) = (0usize, 0usize);
        for &pt in &path[1..=high_i as usize] {
            if polygon[j] != pt {
                j += 1;
                polygon.push(pt);
                if pt.y > polygon[k].y || (pt.y == polygon[k].y && pt.x < polygon[k].x) {
                    k = j;
                }
            }
        }
        if end_type == EndType::ClosedPolygon && j < 2 {
            return;
        }

        let lowest_pt = polygon[k];
        self.poly_nodes.push(OffsetNode {
            polygon,
            join_type,
            end_type,
        });

        //if this path's lowest pt is lower than all the others then update m_lowest
        if end_type != EndType::ClosedPolygon {
            return;
        }
        let child = self.poly_nodes.len() - 1;
        match self.lowest {
            None => self.lowest = Some((child, k)),
            Some((c, p)) => {
                let ip = self.poly_nodes[c].polygon[p];
                if lowest_pt.y > ip.y || (lowest_pt.y == ip.y && lowest_pt.x < ip.x) {
                    self.lowest = Some((child, k));
                }
            }
        }
    }

    /// C# `AddPaths`.
    pub fn add_paths(&mut self, paths: &[Path], join_type: JoinType, end_type: EndType) {
        for p in paths {
            self.add_path(p, join_type, end_type);
        }
    }

    /// Fixup orientations of all closed paths if the orientation of the closed path with the
    /// lowermost vertex is wrong ...
    fn fix_orientations(&mut self) {
        let lowest_is_wrong = self
            .lowest
            .is_some_and(|(c, _)| !orientation(&self.poly_nodes[c].polygon));
        for node in &mut self.poly_nodes {
            let reverse = if lowest_is_wrong {
                node.end_type == EndType::ClosedPolygon
                    || (node.end_type == EndType::ClosedLine && orientation(&node.polygon))
            } else {
                node.end_type == EndType::ClosedLine && !orientation(&node.polygon)
            };
            if reverse {
                node.polygon.reverse();
            }
        }
    }

    fn do_offset(&mut self, delta: f64) {
        self.dest_polys = Paths::new();
        self.delta = delta;

        //if Zero offset, just copy any CLOSED polygons to m_p and return ...
        if near_zero(delta) {
            for node in &self.poly_nodes {
                if node.end_type == EndType::ClosedPolygon {
                    self.dest_polys.push(node.polygon.clone());
                }
            }
            return;
        }

        //see offset_triginometry3.svg in the documentation folder ...
        if self.miter_limit > 2.0 {
            self.miter_lim = 2.0 / (self.miter_limit * self.miter_limit);
        } else {
            self.miter_lim = 0.5;
        }

        let y = if self.arc_tolerance <= 0.0 {
            DEF_ARC_TOLERANCE
        } else if self.arc_tolerance > delta.abs() * DEF_ARC_TOLERANCE {
            delta.abs() * DEF_ARC_TOLERANCE
        } else {
            self.arc_tolerance
        };
        //see offset_triginometry2.svg in the documentation folder ...
        let steps = PI / (1.0 - y / delta.abs()).acos();
        self.sin = (TWO_PI / steps).sin();
        self.cos = (TWO_PI / steps).cos();
        self.steps_per_rad = steps / TWO_PI;
        if delta < 0.0 {
            self.sin = -self.sin;
        }

        for i in 0..self.poly_nodes.len() {
            let (join_type, end_type) = (self.poly_nodes[i].join_type, self.poly_nodes[i].end_type);
            self.src_poly = self.poly_nodes[i].polygon.clone();

            let len = self.src_poly.len();

            if len == 0 || (delta <= 0.0 && (len < 3 || end_type != EndType::ClosedPolygon)) {
                continue;
            }

            self.dest_poly = Path::new();

            if len == 1 {
                let src = self.src_poly[0];
                if join_type == JoinType::Round {
                    let (mut x, mut y) = (1.0, 0.0);
                    let mut j = 1;
                    while j as f64 <= steps {
                        self.dest_poly.push(ip(
                            round(src.x as f64 + x * delta),
                            round(src.y as f64 + y * delta),
                        ));
                        let x2 = x;
                        x = x * self.cos - self.sin * y;
                        y = x2 * self.sin + y * self.cos;
                        j += 1;
                    }
                } else {
                    let (mut x, mut y) = (-1.0, -1.0);
                    for _ in 0..4 {
                        self.dest_poly.push(ip(
                            round(src.x as f64 + x * delta),
                            round(src.y as f64 + y * delta),
                        ));
                        if x < 0.0 {
                            x = 1.0;
                        } else if y < 0.0 {
                            y = 1.0;
                        } else {
                            x = -1.0;
                        }
                    }
                }
                self.dest_polys.push(std::mem::take(&mut self.dest_poly));
                continue;
            }

            //build m_normals ...
            self.normals.clear();
            for j in 0..len - 1 {
                self.normals
                    .push(get_unit_normal(self.src_poly[j], self.src_poly[j + 1]));
            }
            if end_type == EndType::ClosedLine || end_type == EndType::ClosedPolygon {
                self.normals
                    .push(get_unit_normal(self.src_poly[len - 1], self.src_poly[0]));
            } else {
                self.normals.push(self.normals[len - 2]);
            }

            if end_type == EndType::ClosedPolygon {
                let mut k = len - 1;
                for j in 0..len {
                    self.offset_point(j, &mut k, join_type);
                }
                self.dest_polys.push(std::mem::take(&mut self.dest_poly));
            } else if end_type == EndType::ClosedLine {
                let mut k = len - 1;
                for j in 0..len {
                    self.offset_point(j, &mut k, join_type);
                }
                self.dest_polys.push(std::mem::take(&mut self.dest_poly));
                //re-build m_normals ...
                let n = self.normals[len - 1];
                for j in (1..len).rev() {
                    self.normals[j] = negate(self.normals[j - 1]);
                }
                self.normals[0] = negate(n);
                k = 0;
                for j in (0..len).rev() {
                    self.offset_point(j, &mut k, join_type);
                }
                self.dest_polys.push(std::mem::take(&mut self.dest_poly));
            } else {
                let mut k = 0;
                for j in 1..len - 1 {
                    self.offset_point(j, &mut k, join_type);
                }

                if end_type == EndType::OpenButt {
                    let j = len - 1;
                    self.push_along_normal(j, j, 1.0);
                    self.push_along_normal(j, j, -1.0);
                } else {
                    let j = len - 1;
                    k = len - 2;
                    self.sin_a = 0.0;
                    self.normals[j] = negate(self.normals[j]);
                    if end_type == EndType::OpenSquare {
                        self.do_square(j, k);
                    } else {
                        self.do_round(j, k);
                    }
                }

                //re-build m_normals ...
                for j in (1..len).rev() {
                    self.normals[j] = negate(self.normals[j - 1]);
                }
                self.normals[0] = negate(self.normals[1]);

                k = len - 1;
                for j in (1..k).rev() {
                    self.offset_point(j, &mut k, join_type);
                }

                if end_type == EndType::OpenButt {
                    self.push_along_normal(0, 0, -1.0);
                    self.push_along_normal(0, 0, 1.0);
                } else {
                    self.sin_a = 0.0;
                    if end_type == EndType::OpenSquare {
                        self.do_square(0, 1);
                    } else {
                        self.do_round(0, 1);
                    }
                }
                self.dest_polys.push(std::mem::take(&mut self.dest_poly));
            }
        }
    }

    /// `m_destPoly.Add(new IntPoint(Round(src[j].X + normals[n].X * sign * delta), ...))`;
    /// the C#'s open-butt ends write `X - normal.X * delta` as `sign = -1`, which rounds the
    /// same because negating a double is exact.
    fn push_along_normal(&mut self, j: usize, n: usize, sign: f64) {
        let (src, normal) = (self.src_poly[j], self.normals[n]);
        let pt = if sign > 0.0 {
            ip(
                round(src.x as f64 + normal.x * self.delta),
                round(src.y as f64 + normal.y * self.delta),
            )
        } else {
            ip(
                round(src.x as f64 - normal.x * self.delta),
                round(src.y as f64 - normal.y * self.delta),
            )
        };
        self.dest_poly.push(pt);
    }

    /// C# `Execute(ref Paths solution, delta)`.
    pub fn execute(&mut self, solution: &mut Paths, delta: f64) -> Result<(), ClipperError> {
        solution.clear();
        self.fix_orientations();
        self.do_offset(delta);
        //now clean up 'corners' ...
        let mut clpr = Clipper::new();
        clpr.add_paths(&self.dest_polys, PolyType::Subject, true)?;
        if delta > 0.0 {
            clpr.execute(
                ClipType::Union,
                solution,
                PolyFillType::Positive,
                PolyFillType::Positive,
            )?;
        } else {
            let r = get_bounds(&self.dest_polys);
            let outer = vec![
                ip(r.left - 10, r.bottom + 10),
                ip(r.right + 10, r.bottom + 10),
                ip(r.right + 10, r.top - 10),
                ip(r.left - 10, r.top - 10),
            ];

            clpr.add_path(&outer, PolyType::Subject, true)?;
            clpr.reverse_solution = true;
            clpr.execute(
                ClipType::Union,
                solution,
                PolyFillType::Negative,
                PolyFillType::Negative,
            )?;
            if !solution.is_empty() {
                solution.remove(0);
            }
        }
        Ok(())
    }

    fn offset_point(&mut self, j: usize, k: &mut usize, join_type: JoinType) {
        let (nj, nk) = (self.normals[j], self.normals[*k]);
        //cross product ...
        self.sin_a = nk.x * nj.y - nj.x * nk.y;

        if (self.sin_a * self.delta).abs() < 1.0 {
            //dot product ...
            let cos_a = nk.x * nj.x + nj.y * nk.y;
            if cos_a > 0.0 {
                // angle ==> 0 degrees
                self.push_along_normal(j, *k, 1.0);
                return;
            }
            //else angle ==> 180 degrees
        } else if self.sin_a > 1.0 {
            self.sin_a = 1.0;
        } else if self.sin_a < -1.0 {
            self.sin_a = -1.0;
        }

        if self.sin_a * self.delta < 0.0 {
            self.push_along_normal(j, *k, 1.0);
            self.dest_poly.push(self.src_poly[j]);
            self.push_along_normal(j, j, 1.0);
        } else {
            match join_type {
                JoinType::Miter => {
                    let r = 1.0 + (nj.x * nk.x + nj.y * nk.y);
                    if r >= self.miter_lim {
                        self.do_miter(j, *k, r);
                    } else {
                        self.do_square(j, *k);
                    }
                }
                JoinType::Square => self.do_square(j, *k),
                JoinType::Round => self.do_round(j, *k),
            }
        }
        *k = j;
    }

    fn do_square(&mut self, j: usize, k: usize) {
        let (nj, nk, src) = (self.normals[j], self.normals[k], self.src_poly[j]);
        let dx = (self.sin_a.atan2(nk.x * nj.x + nk.y * nj.y) / 4.0).tan();
        self.dest_poly.push(ip(
            round(src.x as f64 + self.delta * (nk.x - nk.y * dx)),
            round(src.y as f64 + self.delta * (nk.y + nk.x * dx)),
        ));
        self.dest_poly.push(ip(
            round(src.x as f64 + self.delta * (nj.x + nj.y * dx)),
            round(src.y as f64 + self.delta * (nj.y - nj.x * dx)),
        ));
    }

    fn do_miter(&mut self, j: usize, k: usize, r: f64) {
        let (nj, nk, src) = (self.normals[j], self.normals[k], self.src_poly[j]);
        let q = self.delta / r;
        self.dest_poly.push(ip(
            round(src.x as f64 + (nk.x + nj.x) * q),
            round(src.y as f64 + (nk.y + nj.y) * q),
        ));
    }

    fn do_round(&mut self, j: usize, k: usize) {
        let (nj, nk, src) = (self.normals[j], self.normals[k], self.src_poly[j]);
        let a = self.sin_a.atan2(nk.x * nj.x + nk.y * nj.y);
        // C# `(int)Round(...)`: Round gives a long, the cast keeps its low 32 bits.
        let steps = std::cmp::max(round(self.steps_per_rad * a.abs()) as i32, 1);

        let (mut x, mut y) = (nk.x, nk.y);
        for _ in 0..steps {
            self.dest_poly.push(ip(
                round(src.x as f64 + x * self.delta),
                round(src.y as f64 + y * self.delta),
            ));
            let x2 = x;
            x = x * self.cos - self.sin * y;
            y = x2 * self.sin + y * self.cos;
        }
        self.dest_poly.push(ip(
            round(src.x as f64 + nj.x * self.delta),
            round(src.y as f64 + nj.y * self.delta),
        ));
    }
}

/// C# `GetUnitNormal`.
fn get_unit_normal(pt1: IntPoint, pt2: IntPoint) -> DoublePoint {
    let mut dx = (pt2.x - pt1.x) as f64;
    let mut dy = (pt2.y - pt1.y) as f64;
    if dx == 0.0 && dy == 0.0 {
        return DoublePoint::default();
    }

    let f = 1.0 / (dx * dx + dy * dy).sqrt();
    dx *= f;
    dy *= f;

    DoublePoint { x: dy, y: -dx }
}

fn negate(p: DoublePoint) -> DoublePoint {
    DoublePoint { x: -p.x, y: -p.y }
}
