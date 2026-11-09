//! Checks the GPU's work on the GPU Ray Tracing page: how many of its hit records match the
//! reference traversal in `gpu_bench` (`Bench::compare`), and the picture drawn from its hits
//! (`Bench::picture`). A disagreement is reported with the ray and both answers, so it can be
//! found, never only counted.
//!
//! The GPU and this traversal run the same arithmetic in `f32`, so a closest hit should find the
//! same triangle. Two hits can still differ honestly on a shared edge or with a coincident
//! triangle: a different triangle at the same distance (within [`TIE_TOLERANCE`], relative to the
//! distance) is a tie, counted apart. An any-hit ray only says blocked or not, so it is compared
//! on that alone.

use solidmesh::mesh::ray_tracer::flat_tree::{hit_from_le_bytes, FlatHit, HIT_STRIDE};

use crate::gpu_bench::{dot, facing_normal, normalized, Bench, RayKind, IMAGE_HEIGHT, NO_RAY};

/// Two different triangles hit within this fraction of the distance are the same answer.
pub const TIE_TOLERANCE: f32 = 1e-4;

/// How many disagreeing rays a result lists.
const MAX_EXAMPLES: usize = 5;

/// The light level of a surface in shadow, and the rest of the light added when lit.
const AMBIENT: f32 = 0.25;
const DIRECT: f32 = 0.75;

/// A colour for the sky.
const SKY: [u8; 3] = [205, 220, 238];

/// How the GPU's answer for one ray compares with the reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HitMatch {
    /// The same triangle, or both found nothing.
    Exact,
    /// A different triangle at the same distance.
    Tie,
    Differ,
}

/// The agreement of the GPU's hits with the reference, over one ray set.
#[derive(Debug, Clone, PartialEq)]
pub struct Agreement {
    pub compared: u64,
    pub exact: u64,
    pub ties: u64,
    pub disagreed: u64,
    /// The first few disagreements, in words.
    pub examples: Vec<String>,
}

/// How a closest-hit answer from the GPU compares with the reference's.
pub fn classify_closest(reference: Option<&FlatHit>, gpu: Option<&FlatHit>) -> HitMatch {
    match (reference, gpu) {
        (None, None) => HitMatch::Exact,
        (Some(a), Some(b)) if a.triangle == b.triangle => HitMatch::Exact,
        (Some(a), Some(b)) => {
            let scale = a.t.abs().max(b.t.abs());
            if (a.t - b.t).abs() <= scale * TIE_TOLERANCE {
                HitMatch::Tie
            } else {
                HitMatch::Differ
            }
        }
        _ => HitMatch::Differ,
    }
}

/// How an any-hit answer compares: only whether the ray was blocked.
pub fn classify_any(reference_blocked: bool, gpu_blocked: bool) -> HitMatch {
    if reference_blocked == gpu_blocked {
        HitMatch::Exact
    } else {
        HitMatch::Differ
    }
}

/// The hit record at `index`, or `None` for a miss.
fn hit_at(bytes: &[u8], index: usize) -> Option<FlatHit> {
    let mut record = [0u8; HIT_STRIDE];
    record.copy_from_slice(&bytes[index * HIT_STRIDE..(index + 1) * HIT_STRIDE]);
    hit_from_le_bytes(&record)
}

fn describe(hit: Option<&FlatHit>) -> String {
    match hit {
        Some(hit) => format!("triangle {} at t {}", hit.triangle, hit.t),
        None => "nothing".to_string(),
    }
}

impl Bench {
    /// `gpu_hits` (one [`HIT_STRIDE`]-byte record per ray of `kind`) against the reference.
    pub fn compare(&self, kind: RayKind, gpu_hits: &[u8]) -> Result<Agreement, String> {
        let rays = self.rays(kind);
        if gpu_hits.len() != rays.len() * HIT_STRIDE {
            return Err(format!(
                "The GPU returned {} bytes of hits for {} rays, and each hit is {HIT_STRIDE} \
                 bytes: run the benchmark again.",
                gpu_hits.len(),
                rays.len()
            ));
        }
        let mut agreement = Agreement {
            compared: rays.len() as u64,
            exact: 0,
            ties: 0,
            disagreed: 0,
            examples: Vec::new(),
        };
        for (index, ray) in rays.iter().enumerate() {
            let gpu = hit_at(gpu_hits, index);
            let (result, reference) = if kind.is_any_hit() {
                let blocked = self.tree.trace_any(ray);
                (classify_any(blocked, gpu.is_some()), None)
            } else {
                let reference = self.tree.trace_closest(ray);
                (
                    classify_closest(reference.as_ref(), gpu.as_ref()),
                    reference,
                )
            };
            match result {
                HitMatch::Exact => agreement.exact += 1,
                HitMatch::Tie => agreement.ties += 1,
                HitMatch::Differ => {
                    agreement.disagreed += 1;
                    if agreement.examples.len() < MAX_EXAMPLES {
                        let wording = if kind.is_any_hit() {
                            format!(
                                "ray {index}: reference {}, GPU {}",
                                describe_any(self.tree.trace_any(ray)),
                                describe_any(gpu.is_some())
                            )
                        } else {
                            format!(
                                "ray {index}: reference {}, GPU {}",
                                describe(reference.as_ref()),
                                describe(gpu.as_ref())
                            )
                        };
                        agreement.examples.push(wording);
                    }
                }
            }
        }
        Ok(agreement)
    }

    /// The picture from the GPU's hits: each first hit shaded flat by its triangle's normal,
    /// darkened where the GPU's shadow ray was blocked or the surface faces away from the light.
    pub fn picture(&self, camera_hits: &[u8], shadow_hits: &[u8]) -> Result<Vec<u8>, String> {
        let camera_rays = self.rays(RayKind::Camera);
        if camera_hits.len() != camera_rays.len() * HIT_STRIDE
            || shadow_hits.len() != self.rays(RayKind::Shadow).len() * HIT_STRIDE
        {
            return Err(
                "The GPU returned the wrong number of hits to draw: run the benchmark again."
                    .to_string(),
            );
        }
        let mut pixels = Vec::with_capacity(camera_rays.len() * 4);
        for (pixel, ray) in camera_rays.iter().enumerate() {
            let Some(hit) = hit_at(camera_hits, pixel) else {
                pixels.extend_from_slice(&[SKY[0], SKY[1], SKY[2], 255]);
                continue;
            };
            let normal = facing_normal(&self.tree, &hit, ray.direction);
            let point = [
                ray.origin[0] + ray.direction[0] * hit.t,
                ray.origin[1] + ray.direction[1] * hit.t,
                ray.origin[2] + ray.direction[2] * hit.t,
            ];
            let to_light = normalized([
                self.light[0] - point[0],
                self.light[1] - point[1],
                self.light[2] - point[2],
            ]);
            let diffuse = dot(normal, to_light).max(0.0);
            let shadow_ray = self.shadow_of_pixel[pixel];
            let lit = shadow_ray != NO_RAY && hit_at(shadow_hits, shadow_ray as usize).is_none();
            let light = AMBIENT + DIRECT * if lit { diffuse } else { 0.0 };
            for channel in normal {
                // `as` saturates; the value is in 0..=255 by construction.
                pixels.push(((channel * 0.5 + 0.5) * light * 255.0).round_ties_even() as u8);
            }
            pixels.push(255);
        }
        debug_assert_eq!(
            pixels.len(),
            camera_rays.len() * 4,
            "one RGBA pixel per camera ray of a {IMAGE_HEIGHT}-row picture"
        );
        Ok(pixels)
    }
}

fn describe_any(blocked: bool) -> &'static str {
    if blocked {
        "blocked"
    } else {
        "unblocked"
    }
}
