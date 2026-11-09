//! How light scatters off a surface: the maths a path tracer shades with. Pure functions on
//! `f64`, all in a local frame whose +z is the shading normal ([`LocalFrame`] gets a tracer into
//! and out of it); no randomness inside, the caller hands the random numbers in.
//!
//! [`ScatteringSurface`] (in `surface`) is the plain description: base color, metallic, roughness,
//! transmission and index of refraction, with named presets. `model` gives it the three
//! operations a tracer needs: [`ScatteringSurface::evaluate`], [`ScatteringSurface::sample`] and
//! [`ScatteringSurface::pdf`]. `microfacet` holds the formulas they share; `albedo` and its
//! generated `albedo_table` hold the precomputed energy of the reflective lobe. `gpu_record` is
//! a surface and that table as plain buffers for the WGSL port in `flat_tree/scatter.wgsl`.
//!
//! # The model
//!
//! A surface is a blend of three materials, in shares that sum to 1: metal (`metallic`), glass
//! (`(1 - metallic) * transmission`) and opaque dielectric, the rest (plastic).
//! - Metal and plastic reflect with a rough specular lobe (GGX microfacets, height-correlated
//!   masking-shadowing, Schlick Fresnel). Metal's reflectance at normal incidence is its base
//!   color and it goes to white at grazing; plastic's comes from the index of refraction.
//! - A single-scatter lobe loses the light that bounces between facets more than once, so a
//!   multiple-scatter lobe puts it back: `(1 - E(wo)) * (1 - E(wi)) / (pi * (1 - E_avg))`, with `E`
//!   the true directional albedo of the white-Fresnel single-scatter lobe (the precomputed table)
//!   and `E_avg` its cosine-weighted mean. It is colored by the mean Fresnel reflectance, so a
//!   white metal returns all its light and a colored metal keeps its saturation. Multiple scatter
//!   is sampled from the cosine hemisphere with the diffuse.
//! - Plastic has Lambert diffuse under that specular, scaled by what the specular leaves behind
//!   on the way in and on the way out, `(1 - E(wo)) * (1 - E(wi)) / (1 - E_avg)` with `E` now
//!   everything the plastic's specular lobe reflects. That shape is reciprocal, and it makes a
//!   white plastic return all its light in a white environment at every angle and roughness,
//!   which the furnace tests measure, to the accuracy of the table's interpolation.
//! - Glass reflects and refracts through one microfacet interface using the exact dielectric
//!   Fresnel equations, with total internal reflection, on either side of the surface. It is
//!   tinted by `sqrt(base_color)` per crossing, so a closed solid entered and left again shows
//!   exactly its base color. Rough glass is single-scatter only and not compensated: at high
//!   roughness it loses the light that bounces between facets, which the furnace tests pin. A
//!   compensating lobe for a transmissive surface needs its own table over the index of
//!   refraction.
//!
//! Schlick's formula gives a dielectric a grazing reflectance of `min(1, 50 * F0)` rather than 1,
//! so an interface with no index step (IOR 1, F0 = 0) truly does not reflect and a pure diffuse
//! surface is exactly `base / pi`. Metals always go to 1.
//!
//! # Conventions
//!
//! `wo` is the direction toward the camera and `wi` toward the light, both pointing away from the
//! surface point: this is **radiance** tracing from the camera. Refraction carries the
//! `eta_wo^2 / eta_wi^2` radiance scaling, so a path entering glass has its weight divided by
//! `ior^2` and regains it on leaving; a tracer applies nothing extra. Because of that factor,
//! transmission is reciprocal only as `f(wo, wi) / eta_wo^2 == f(wi, wo) / eta_wi^2`.
//!
//! `wo` and `wi` are normalized on the way in. A direction that is zero length, not finite, or
//! within `MIN_COS` of the horizon scatters nothing. A purely opaque surface is one sided: with
//! `wo` below it, there is nothing to return (a tracer flips the shading normal toward the ray
//! first). A surface with any glass share is handed its true outside as +z, since that is how the
//! glass tells entering from leaving, so `wo` can come from below; its opaque shares then shade
//! on the side `wo` is on, mirrored in z, which `evaluate`, `sample`, `pdf` and the lobe choice
//! all do alike. So moving `transmission` off 0 never blackens a back face.
//!
//! `roughness` is perceptual, as a slider is: the GGX width is `roughness^2`, clamped to at least
//! `MIN_ALPHA` so nothing divides by zero. A surface whose unclamped width is below that minimum
//! is **smooth**: its reflection and refraction are delta lobes, which contribute nothing to
//! `evaluate` and `pdf` and only `sample` can produce, with [`ScatteringSample::is_specular`] set.
//! Glass with an index of refraction within `INDEX_MATCHED` of 1 is treated as a smooth interface
//! whatever its roughness, with its exact Fresnel and Snell; exactly at IOR 1 it passes light
//! straight through.
//!
//! [`ScatteringSurface::evaluate`], `sample` and `pdf` never return NaN, infinity or a negative
//! for any input: a value that would overflow is reported as zero. [`LocalFrame`] returns no frame
//! for a normal with no direction; its conversions are plain dot products and carry a NaN in the
//! direction through.

mod albedo;
mod albedo_table;
mod frame;
mod gpu_record;
mod microfacet;
mod model;
mod surface;

pub use albedo::{
    albedo_column_cos, albedo_row_alpha, directional_albedo, directional_albedo_by_quadrature,
    mean_albedo_of_row, stored_albedo_cell, stored_mean_albedo, AlbedoTable, ALBEDO_TABLE_SIZE,
    GENERATION_SUBDIVISIONS,
};
pub use frame::LocalFrame;
pub use gpu_record::{
    albedo_table_bytes, SurfaceRecord, ALBEDO_ENTRY_COUNT, ALBEDO_ENTRY_STRIDE, SURFACE_HAS_GLASS,
    SURFACE_INDEX_MATCHED, SURFACE_RECORD_STRIDE, SURFACE_SMOOTH,
};
pub use model::ScatteringSample;
pub use surface::ScatteringSurface;
