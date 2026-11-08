//! [`LocalFrame`]: an orthonormal frame around a surface normal, so a tracer can bring world
//! directions into the local frame the scattering model works in (+z is the normal) and take
//! sampled directions back out. Part of `scattering`.

use solidmesh_math::Vector3;

use super::microfacet::unit_direction;

/// An orthonormal frame with `normal` as local +z.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalFrame {
    pub tangent: Vector3,
    pub bitangent: Vector3,
    pub normal: Vector3,
}

impl LocalFrame {
    /// The frame around `normal`, which need not be unit length; `None` when it has no direction
    /// (zero length or not finite). Well conditioned for every normal: the only division is by
    /// `sign + z`, whose magnitude is at least 1 because `sign` matches `z`.
    pub fn from_normal(normal: Vector3) -> Option<Self> {
        let n = unit_direction(normal)?;
        let sign = 1.0_f64.copysign(n.z);
        let a = -1.0 / (sign + n.z);
        let b = n.x * n.y * a;
        Some(LocalFrame {
            tangent: Vector3::new(1.0 + sign * n.x * n.x * a, sign * b, -sign * n.x),
            bitangent: Vector3::new(b, sign + n.y * n.y * a, -n.y),
            normal: n,
        })
    }

    /// `world` in this frame's coordinates.
    pub fn to_local(&self, world: Vector3) -> Vector3 {
        Vector3::new(
            world.dot(self.tangent),
            world.dot(self.bitangent),
            world.dot(self.normal),
        )
    }

    /// `local`, in this frame's coordinates, back in the coordinates the normal was given in.
    pub fn to_world(&self, local: Vector3) -> Vector3 {
        self.tangent * local.x + self.bitangent * local.y + self.normal * local.z
    }
}
