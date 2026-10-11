//! The ray tracer's settings and its primary-ray trace (agg-sharp
//! `PolygonMesh/RayTracer/RayTracer.cs`, the class MatterCAD's tests also alias
//! `RayTracerEngine`). [`RayTracer::trace_primary_ray`] finds the nearest hit among a
//! [`Scene`]'s shapes; the image renderer (`RayTraceScene`, `FullyTraceRay`, the color, depth
//! and normal buffers, anti-aliasing and the image copies) and ray bundles arrive with the
//! renderer slice, which reads the settings stored here.

use vector_math::{IntersectionType, Ray};

use super::{IntersectInfo, Scene};

/// How many extra rays the renderer fires per pixel that differs from its neighbors (C#
/// `AntiAliasing`; the discriminant is the sample count).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AntiAliasing {
    None = 0,
    Low = 4,
    #[default]
    Medium = 8,
    High = 16,
    VeryHigh = 32,
}

/// A ray tracer's settings (C# `RayTracer`), defaulting as the C# properties do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RayTracer {
    pub anti_aliasing: AntiAliasing,
    pub multi_threaded: bool,
    pub render_diffuse: bool,
    pub render_highlights: bool,
    pub render_reflection: bool,
    pub render_refraction: bool,
    pub render_shadow: bool,
    pub trace_with_ray_bundles: bool,
}

impl Default for RayTracer {
    fn default() -> Self {
        RayTracer {
            anti_aliasing: AntiAliasing::Medium,
            multi_threaded: true,
            render_diffuse: true,
            render_highlights: true,
            render_reflection: true,
            render_refraction: true,
            render_shadow: true,
            trace_with_ray_bundles: false,
        }
    }
}

impl RayTracer {
    /// A ray tracer with the default settings (C# `new RayTracer()`).
    pub fn new() -> Self {
        Self::default()
    }

    /// The nearest hit of `ray` among the scene's shapes at a distance of at least 0 (C#
    /// `TracePrimaryRay`); a miss is an [`IntersectInfo::new`] (hit type `None`, distance
    /// `f64::MAX`). As in C#, the shapes narrow the ray's `max_distance_to_consider` as they
    /// find hits, so a later shape is only tested up to the nearest hit so far.
    pub fn trace_primary_ray<'a>(&self, ray: &mut Ray, scene: &'a Scene) -> IntersectInfo<'a> {
        let mut primary_ray_intersection = IntersectInfo::new();

        for shape_to_test in &scene.shapes {
            let info = shape_to_test.get_closest_intersection(ray);
            if let Some(info) = info {
                if info.hit_type != IntersectionType::None
                    && info.distance_to_hit < primary_ray_intersection.distance_to_hit
                    && info.distance_to_hit >= 0.0
                {
                    primary_ray_intersection = info;
                }
            }
        }

        primary_ray_intersection
    }
}
