//! geometry3Sharp's implicit functions (Boost Software License 1.0) that agg-sharp's implicit
//! booleans reach: the `BoundedImplicitFunction3d` interface (`implicit/Implicit3d.cs`), the
//! sampled field `DenseGridTrilinearImplicit` (`implicit/GridImplicits3d.cs`) and the boolean
//! operators `ImplicitUnion3d`, `ImplicitIntersection3d`, `ImplicitDifference3d` and their n-ary
//! forms (`Implicit3d.cs`). Negative is inside; the surface is the zero isocontour.

use super::dmesh3::AxisAlignedBox3d;
use super::signed_distance_grid::SignedDistanceGrid;
use crate::sdf::DenseGrid3f;

/// g3 `BoundedImplicitFunction3d`: a field with a box holding its interesting part.
pub(crate) trait BoundedImplicitFunction3d {
    /// C# `Value(ref Vector3d)`.
    fn value(&self, pt: [f64; 3]) -> f64;
    /// C# `Bounds()`.
    fn bounds(&self) -> AxisAlignedBox3d;
}

/// A shared, boxed implicit (C#'s interface references).
pub(crate) type BoxedImplicit = Box<dyn BoundedImplicitFunction3d>;

/// g3 `DenseGridTrilinearImplicit`: the trilinear interpolant of a grid placed at `grid_origin`.
pub(crate) struct DenseGridTrilinearImplicit {
    pub(crate) grid: DenseGrid3f,
    pub(crate) cell_size: f64,
    pub(crate) grid_origin: [f64; 3],
    /// C# `Outside`: the value off the grid (big, but squarable without overflow).
    pub(crate) outside: f64,
}

impl DenseGridTrilinearImplicit {
    /// C# `new DenseGridTrilinearImplicit(sdf.Grid, sdf.GridOrigin, sdf.CellSize)`.
    pub(crate) fn from_sdf(sdf: SignedDistanceGrid) -> Self {
        DenseGridTrilinearImplicit {
            grid: sdf.grid,
            cell_size: sdf.cell_size as f64,
            grid_origin: sdf.grid_origin.map(|v| v as f64),
            outside: f64::MAX.sqrt().sqrt(),
        }
    }

    /// C# `DenseGrid3f.get_x_pair`.
    fn x_pair(&self, i0: i32, j: i32, k: i32) -> (f64, f64) {
        let offset = self.grid.ni * (j as usize + self.grid.nj * k as usize);
        let i0 = i0 as usize;
        (
            self.grid.buffer[offset + i0] as f64,
            self.grid.buffer[offset + i0 + 1] as f64,
        )
    }
}

impl BoundedImplicitFunction3d for DenseGridTrilinearImplicit {
    fn bounds(&self) -> AxisAlignedBox3d {
        let o = self.grid_origin;
        AxisAlignedBox3d {
            min: o,
            max: [
                o[0] + self.cell_size * self.grid.ni as f64,
                o[1] + self.cell_size * self.grid.nj as f64,
                o[2] + self.cell_size * self.grid.nk as f64,
            ],
        }
    }

    fn value(&self, pt: [f64; 3]) -> f64 {
        let gx = (pt[0] - self.grid_origin[0]) / self.cell_size;
        let gy = (pt[1] - self.grid_origin[1]) / self.cell_size;
        let gz = (pt[2] - self.grid_origin[2]) / self.cell_size;

        // C# `(int)` truncates toward zero, so a point less than a cell below the origin reads
        // cell 0 (`as` truncates the same way and saturates far off the grid, which is outside
        // either way).
        let x0 = gx as i32;
        let (y0, z0) = (gy as i32, gz as i32);
        let (y1, z1) = (y0 + 1, z0 + 1);
        if x0 < 0
            || (x0 + 1) as i64 >= self.grid.ni as i64
            || y0 < 0
            || y1 as i64 >= self.grid.nj as i64
            || z0 < 0
            || z1 as i64 >= self.grid.nk as i64
        {
            return self.outside;
        }

        let fax = gx - x0 as f64;
        let fay = gy - y0 as f64;
        let faz = gz - z0 as f64;
        let one_minus_fax = 1.0 - fax;

        let (xa, xb) = self.x_pair(x0, y0, z0);
        let mut yz = (1.0 - fay) * (1.0 - faz);
        let mut sum = (one_minus_fax * xa + fax * xb) * yz;

        let (xa, xb) = self.x_pair(x0, y0, z1);
        yz = (1.0 - fay) * faz;
        sum += (one_minus_fax * xa + fax * xb) * yz;

        let (xa, xb) = self.x_pair(x0, y1, z0);
        yz = fay * (1.0 - faz);
        sum += (one_minus_fax * xa + fax * xb) * yz;

        let (xa, xb) = self.x_pair(x0, y1, z1);
        yz = fay * faz;
        sum += (one_minus_fax * xa + fax * xb) * yz;

        sum
    }
}

/// C# `Math.Min(double, double)`: NaN wins, unlike `f64::min`.
fn cs_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}

/// C# `Math.Max(double, double)`: NaN wins, unlike `f64::max`.
fn cs_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}

/// g3 `ImplicitUnion3d`: A or B.
pub(crate) struct ImplicitUnion3d {
    pub(crate) a: BoxedImplicit,
    pub(crate) b: BoxedImplicit,
}

impl BoundedImplicitFunction3d for ImplicitUnion3d {
    fn value(&self, pt: [f64; 3]) -> f64 {
        cs_min(self.a.value(pt), self.b.value(pt))
    }

    fn bounds(&self) -> AxisAlignedBox3d {
        let mut b = self.a.bounds();
        b.contain(&self.b.bounds());
        b
    }
}

/// g3 `ImplicitIntersection3d`: A and B (its bounds are the union's, a C# TODO).
pub(crate) struct ImplicitIntersection3d {
    pub(crate) a: BoxedImplicit,
    pub(crate) b: BoxedImplicit,
}

impl BoundedImplicitFunction3d for ImplicitIntersection3d {
    fn value(&self, pt: [f64; 3]) -> f64 {
        cs_max(self.a.value(pt), self.b.value(pt))
    }

    fn bounds(&self) -> AxisAlignedBox3d {
        let mut b = self.a.bounds();
        b.contain(&self.b.bounds());
        b
    }
}

/// g3 `ImplicitDifference3d`: A and not B, bounded by A.
pub(crate) struct ImplicitDifference3d {
    pub(crate) a: BoxedImplicit,
    pub(crate) b: BoxedImplicit,
}

impl BoundedImplicitFunction3d for ImplicitDifference3d {
    fn value(&self, pt: [f64; 3]) -> f64 {
        cs_max(self.a.value(pt), -self.b.value(pt))
    }

    fn bounds(&self) -> AxisAlignedBox3d {
        self.a.bounds()
    }
}

/// g3 `ImplicitNaryUnion3d`: any child.
pub(crate) struct ImplicitNaryUnion3d {
    pub(crate) children: Vec<BoxedImplicit>,
}

impl BoundedImplicitFunction3d for ImplicitNaryUnion3d {
    fn value(&self, pt: [f64; 3]) -> f64 {
        let mut f = self.children[0].value(pt);
        for child in &self.children[1..] {
            f = cs_min(f, child.value(pt));
        }
        f
    }

    fn bounds(&self) -> AxisAlignedBox3d {
        let mut b = self.children[0].bounds();
        for child in &self.children[1..] {
            b.contain(&child.bounds());
        }
        b
    }
}

/// g3 `ImplicitNaryIntersection3d`: every child, bounded by the overlap of their boxes.
pub(crate) struct ImplicitNaryIntersection3d {
    pub(crate) children: Vec<BoxedImplicit>,
}

impl BoundedImplicitFunction3d for ImplicitNaryIntersection3d {
    fn value(&self, pt: [f64; 3]) -> f64 {
        let mut f = self.children[0].value(pt);
        for child in &self.children[1..] {
            f = cs_max(f, child.value(pt));
        }
        f
    }

    fn bounds(&self) -> AxisAlignedBox3d {
        let mut b = self.children[0].bounds();
        for child in &self.children[1..] {
            b = b.intersect(&child.bounds());
        }
        b
    }
}

/// g3 `ImplicitNaryDifference3d`: A minus the union of `b_set`, bounded by A.
pub(crate) struct ImplicitNaryDifference3d {
    pub(crate) a: BoxedImplicit,
    pub(crate) b_set: Vec<BoxedImplicit>,
}

impl BoundedImplicitFunction3d for ImplicitNaryDifference3d {
    fn value(&self, pt: [f64; 3]) -> f64 {
        let fa = self.a.value(pt);
        let Some((first, rest)) = self.b_set.split_first() else {
            return fa;
        };
        let mut fb = first.value(pt);
        for b in rest {
            fb = cs_min(fb, b.value(pt));
        }
        cs_max(fa, -fb)
    }

    fn bounds(&self) -> AxisAlignedBox3d {
        self.a.bounds()
    }
}
