//! The GPU route's line toward the sun, `path_sun_line` in `sun.wgsl`, in Rust: [`sun_line_f32`]
//! walks a packed scene with the `f32` reference traversal ([`super::FlatTree::trace_closest`]
//! and [`super::FlatTree::trace_any`]) in the shader's order of operations, so a test can hold the
//! card's line against the CPU's (`sun_through_glass.rs`, through `RenderScene::sun_line_at`)
//! line for line without a graphics card, as `coverage_line.rs` does for the coverage walk. A
//! test pins the shader's text to these steps. The pieces the walk reads are mirrored here too:
//! a hit's point and its facing (`path_surface_point`, `path_facing`), the continuing-ray start,
//! the surface of a part at a face's color, the seam (`seam_at`), the floor's distance and rest
//! rule, and the scattering model's straight transmittance (`surface_straight_transmittance`)
//! with the shader's Fresnel.
//!
//! **The vertex** ([`SunVertexF32`]) is what the path loop knows where it samples the sun: the
//! point, the seam interface there, and whether the point is on a contact with the floor (the
//! floor under a part, the part's face hit kept, or a part's face over the floor). A caller
//! builds it from the hits the path loop would take; the walk decides nothing about it.
//!
//! **Draws.** Where the shader draws `path_next_unit() < opacity` at a part's first crossing, the
//! walk asks `there` with the part's opacity, in the same order, so a caller can feed it the
//! CPU's own draws and compare the two lines decision for decision.

use super::coverage_line::{MAX_CROSSINGS_PER_LINE_F32, PATH_FAR_F32};
use super::floor_rest::{floor_offset_f32, rests_on_floor_f32};
use super::gpu_contacts::{
    CONTACT_COVER_DENSER, CONTACT_HIT_DENSER, CONTACT_NO_SEAM, CONTACT_SAME_GLASS,
};
use super::gpu_scene::GpuRenderScene;
use super::gpu_view::{SCENE_FLAG_FLOOR, SCENE_FLAG_GLASS_CONTACTS, SCENE_FLAG_SEE_THROUGH};
use super::traverse::{FlatHit, FlatRay, FACE_BOTH};
use super::{SurfaceRecord, SURFACE_HAS_GLASS};

/// The most faces of glass a line toward the sun goes through (`PATH_MAX_GLASS_CROSSINGS` in
/// `sun.wgsl`, `MAX_GLASS_CROSSINGS` on the CPU).
pub const MAX_GLASS_CROSSINGS_F32: u32 = 16;

/// The part a point on the floor carries (`PATH_FLOOR_PART` in `catcher.wgsl`).
pub const FLOOR_PART_F32: u32 = 0xFFFF_FFFE;

/// The shader's `CONTACT_SKIP_TOLERANCES`.
const SKIP_TOLERANCES_F32: f32 = 2.0;

/// `UNIT_LENGTH_SLACK` in `scatter.wgsl`.
const UNIT_LENGTH_SLACK_F32: f32 = 9.536_743e-7;

/// `MIN_DIRECTION` in `trace.wgsl`.
const MIN_DIRECTION_F32: f32 = 1e-18;

type V3 = [f32; 3];

fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: V3, s: f32) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn neg(a: V3) -> V3 {
    [-a[0], -a[1], -a[2]]
}

/// `unit_direction` in `scatter.wgsl`: `v` scaled to unit length, the zero vector without one.
pub fn unit_direction_f32(v: V3) -> V3 {
    if (dot(v, v) - 1.0).abs() < UNIT_LENGTH_SLACK_F32 {
        return v;
    }
    let largest = v[0].abs().max(v[1].abs()).max(v[2].abs());
    if largest <= 0.0 {
        return [0.0; 3];
    }
    let scaled = [v[0] / largest, v[1] / largest, v[2] / largest];
    let length = dot(scaled, scaled).sqrt();
    if length > 0.0 {
        return [scaled[0] / length, scaled[1] / length, scaled[2] / length];
    }
    [0.0; 3]
}

/// A point on a packed face or on the floor (`PathPoint` in `path.wgsl`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathPointF32 {
    pub position: V3,
    pub geometric: V3,
    pub shading: V3,
    pub ray_offset: f32,
    pub part: u32,
    pub color: u32,
}

/// `path_surface_point`: the point of `hit`, rebuilt from its triangle's corners.
pub fn surface_point_f32(pack: &GpuRenderScene, hit: &FlatHit) -> PathPointF32 {
    let face = pack.faces()[hit.source_index as usize];
    let triangle = pack.tree().triangles()[hit.triangle as usize];
    let [u, v, w] = hit.barycentric;
    let position = add(
        add(scale(triangle.v0, u), scale(triangle.v1, v)),
        scale(triangle.v2, w),
    );
    let mixed = add(
        add(scale(face.normals[0], u), scale(face.normals[1], v)),
        scale(face.normals[2], w),
    );
    let mut shading = unit_direction_f32(mixed);
    if shading == [0.0; 3] {
        shading = face.geometric;
    }
    PathPointF32 {
        position,
        geometric: face.geometric,
        shading,
        ray_offset: face.ray_offset,
        part: face.part,
        color: face.color,
    }
}

/// `path_floor_point_under`: the floor `floor_plane` right under `position`.
pub fn floor_point_under_f32(position: V3, floor_plane: [f32; 4]) -> PathPointF32 {
    let at = [position[0], position[1], floor_plane[2]];
    PathPointF32 {
        position: at,
        geometric: [0.0, 0.0, 1.0],
        shading: [0.0, 0.0, 1.0],
        ray_offset: floor_offset_f32(at),
        part: FLOOR_PART_F32,
        color: 0x00FF_FFFF,
    }
}

/// `path_facing`: the shading normal on the side the line along `incoming` arrives from, and
/// whether it arrives at the back of the face.
pub fn facing_f32(point: &PathPointF32, incoming: V3) -> (V3, bool) {
    let toward = neg(incoming);
    let back_face = dot(point.geometric, toward) < 0.0;
    let geometric = if back_face {
        neg(point.geometric)
    } else {
        point.geometric
    };
    let mut shading = if back_face {
        neg(point.shading)
    } else {
        point.shading
    };
    if !(dot(shading, geometric) > 0.0 && dot(shading, toward) > 0.0) {
        shading = geometric;
    }
    (shading, back_face)
}

/// `path_next_origin`: where a line leaving `point` along `outgoing` starts.
pub fn next_origin_f32(point: &PathPointF32, outgoing: V3) -> V3 {
    let side = if dot(outgoing, point.geometric) >= 0.0 {
        point.ray_offset
    } else {
        -point.ray_offset
    };
    add(point.position, scale(point.geometric, side))
}

/// `seam_origin`: past both faces of a seam of half width `skip`, on the side `direction` leaves by.
fn seam_origin_f32(point: &PathPointF32, direction: V3, skip: f32) -> V3 {
    if dot(direction, point.geometric) >= 0.0 {
        add(point.position, scale(point.geometric, skip))
    } else {
        add(point.position, scale(point.geometric, -skip))
    }
}

/// `path_surface_at`: the surface of `part` at a face of `color` (`0xBBGGRR`), the floor's for the
/// floor `floor_surface`.
fn surface_at_f32(
    pack: &GpuRenderScene,
    floor_surface: SurfaceRecord,
    part: u32,
    color: u32,
) -> SurfaceRecord {
    if part == FLOOR_PART_F32 {
        return floor_surface;
    }
    let mut surface = pack.parts()[part as usize].surface;
    if color & 0x00FF_FFFF != 0x00FF_FFFF {
        let table = pack.srgb_table_bytes();
        let linear = |byte: u32| -> f32 {
            let at = byte as usize * 4;
            f32::from_le_bytes([table[at], table[at + 1], table[at + 2], table[at + 3]])
        };
        surface.base_color[0] *= linear(color & 255);
        surface.base_color[1] *= linear((color >> 8) & 255);
        surface.base_color[2] *= linear((color >> 16) & 255);
    }
    surface
}

/// The shader's `dielectric_fresnel` from the air side (`GlassSide(false, .., 1, ior, ior - 1)`).
fn fresnel_from_air_f32(cos_incident: f32, s: &SurfaceRecord) -> f32 {
    let a = 1.0f32;
    let b = s.ior;
    let c = cos_incident.abs().min(1.0);
    let step_sum = s.ior_minus_one * (a + b);
    let cos2_t = (step_sum + a * a * c * c) / (b * b);
    if cos2_t <= 0.0 {
        return 1.0;
    }
    let cos_t = cos2_t.sqrt();
    let perpendicular_sum = a * c + b * cos_t;
    let perpendicular = -step_sum / (perpendicular_sum * perpendicular_sum);
    let parallel_sum = b * c + a * cos_t;
    let parallel =
        step_sum * (c * c * (a * a + b * b) - a * a) / (b * b * parallel_sum * parallel_sum);
    0.5 * (parallel * parallel + perpendicular * perpendicular)
}

/// `surface_passes_sun_straight`: the surface has a glass share.
pub fn passes_sun_straight_f32(s: &SurfaceRecord) -> bool {
    s.flags & SURFACE_HAS_GLASS != 0
}

/// `surface_straight_transmittance`: what one crossing at cosine `cos` passes, `glass share x (1 -
/// Fresnel from the air side) x the one-crossing tint`.
pub fn straight_transmittance_f32(s: &SurfaceRecord, cos: f32) -> V3 {
    let fresnel = fresnel_from_air_f32(cos, s);
    let glass = (1.0 - s.metallic) * s.transmission;
    let share = glass * (1.0 - fresnel);
    s.base_color.map(|channel| channel.sqrt() * share)
}

/// `path_sun_bent` (`FirstFace::bent`): the direction inside glass of index `ior` that leaves
/// through a face of outward normal `outward` along `line`, and the cosines outside and inside.
pub fn sun_bent_f32(outward: V3, ior: f32, line: V3) -> Option<(V3, f32, f32)> {
    let cos_out = dot(outward, line);
    if cos_out <= 0.0 || ior < 1.0 {
        return None;
    }
    let tangent = add(line, scale(outward, -cos_out));
    let sin2_in = (1.0 - cos_out * cos_out) / (ior * ior);
    let cos_in = (1.0 - sin2_in).max(0.0).sqrt();
    let inside = unit_direction_f32(add(
        [tangent[0] / ior, tangent[1] / ior, tangent[2] / ior],
        scale(outward, cos_in),
    ));
    Some((inside, cos_out, cos_in))
}

/// The one interface at a seam a vertex is on (`PathSeam` of kind `SEAM_INTERFACE`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamF32 {
    pub surface: SurfaceRecord,
    pub into_denser: bool,
    pub skip: f32,
    /// The part not there on the line that reached the seam, and whether its side is the one the
    /// hit face's normal points to.
    pub absent: Option<(u32, bool)>,
}

/// What `seam_at` found at a hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SeamAtF32 {
    None,
    Straight { skip: f32 },
    Interface(SeamF32),
    Ended,
}

/// Where the vertex is on a contact with the floor: `resting` in `path.wgsl`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FloorContactF32 {
    /// The floor under a part, whose face's hit is `rest_hit` (`floor_first`).
    FloorUnder { rest_hit: FlatHit },
    /// A part's face over the floor.
    OverFloor,
}

/// The vertex a line toward the sun leaves from (`PathVertex`'s `point`, `seam`, `resting`,
/// `floor_first` and `rest_hit`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunVertexF32 {
    pub point: PathPointF32,
    pub seam: Option<SeamF32>,
    pub contact: Option<FloorContactF32>,
}

/// What the line found (`SunLine`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SunLineF32 {
    pub reached: bool,
    pub transmittance: V3,
    pub crossed: u32,
    /// The first face of glass crossed from inside its solid: its outward shading normal and the
    /// glass's index.
    pub first_face: Option<(V3, f32)>,
}

/// The shader's `line_*` record of one straight line.
#[derive(Debug, Default)]
pub struct LineRecordF32 {
    passed: Vec<u32>,
    crossings: u32,
}

/// `line_at_hit`'s verdicts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineVerdictF32 {
    Stops,
    Passes,
    Ended,
}

impl LineRecordF32 {
    fn restart(&mut self) {
        self.passed.clear();
        self.crossings = 0;
    }

    fn starts_past(&mut self, part: u32) {
        if (self.passed.len() as u32) < MAX_CROSSINGS_PER_LINE_F32 {
            self.passed.push(part);
        }
    }

    /// `line_at_hit`, the draw asked of `there` with the part's opacity.
    fn at_hit(
        &mut self,
        pack: &GpuRenderScene,
        part: u32,
        there: &mut dyn FnMut(f32) -> bool,
    ) -> LineVerdictF32 {
        let opacity = pack.parts()[part as usize].opacity;
        if opacity >= 1.0 {
            return LineVerdictF32::Stops;
        }
        if self.crossings >= MAX_CROSSINGS_PER_LINE_F32 {
            return LineVerdictF32::Ended;
        }
        self.crossings += 1;
        if self.passed.contains(&part) {
            return LineVerdictF32::Passes;
        }
        self.passed.push(part);
        if opacity > 0.0 && there(opacity) {
            self.passed.pop();
            return LineVerdictF32::Stops;
        }
        LineVerdictF32::Passes
    }
}

/// What a line walk reads of a packed scene: the pack, the analytic floor when the path may meet
/// one (`path_floor_active`), and whether the pack's flags would carry `PATH_FLAG_SUN_STRAIGHT`.
#[derive(Debug, Clone, Copy)]
pub struct SunLineScene<'a> {
    pack: &'a GpuRenderScene,
    /// The floor's plane (center x, center y, height, half side) and its surface.
    floor: Option<([f32; 4], SurfaceRecord)>,
    sun_straight: bool,
}

impl<'a> SunLineScene<'a> {
    /// The walk over `pack`, its view's floor met when `floor_active` and the view has one, the
    /// line straight through glass when `sun_straight`.
    pub fn new(pack: &'a GpuRenderScene, floor_active: bool, sun_straight: bool) -> Self {
        let view = pack.view();
        let floor = (floor_active && view.flags & SCENE_FLAG_FLOOR != 0).then(|| {
            let (base, rest) = (view.floor_base, view.floor_rest);
            let surface = SurfaceRecord {
                base_color: [base[0], base[1], base[2]],
                metallic: base[3],
                roughness: rest[0],
                transmission: rest[1],
                ior: rest[2],
                ior_minus_one: rest[3],
                flags: view.floor_surface_flags,
            };
            (view.floor_plane, surface)
        });
        SunLineScene {
            pack,
            floor,
            sun_straight,
        }
    }

    /// The floor's plane, when the walk meets the floor.
    pub fn floor_plane(&self) -> Option<[f32; 4]> {
        self.floor.map(|(plane, _)| plane)
    }
    fn flags(&self) -> u32 {
        self.pack.view().flags
    }

    fn floor_surface(&self) -> SurfaceRecord {
        self.floor
            .map_or(self.pack.parts()[0].surface, |(_, surface)| surface)
    }

    /// `path_floor_distance` from `origin` along `direction`, or a negative number.
    fn floor_distance(&self, origin: V3, direction: V3) -> f32 {
        let Some((plane, _)) = self.floor else {
            return -1.0;
        };
        let dz = direction[2];
        if dz.abs() < MIN_DIRECTION_F32 {
            return -1.0;
        }
        let t = (plane[2] - origin[2]) / dz;
        // The shader's `!(t >= t_min && t <= t_max)`: a NaN misses too.
        if !(0.0..=PATH_FAR_F32).contains(&t) {
            return -1.0;
        }
        let at = [origin[0] + direction[0] * t, origin[1] + direction[1] * t];
        if (at[0] - plane[0]).abs() > plane[3] || (at[1] - plane[1]).abs() > plane[3] {
            return -1.0;
        }
        t
    }

    /// `path_rests_on_floor`.
    fn rests_on_floor(&self, point: &PathPointF32) -> bool {
        self.floor.is_some_and(|(plane, _)| {
            rests_on_floor_f32(point.position, point.geometric, point.ray_offset, plane)
        })
    }

    /// `seam_at`: the seam at `point` on the hit's packed face `triangle`, for a line along
    /// `direction` that found the hit part there when `here_present`.
    pub fn seam_at(
        &self,
        point: &PathPointF32,
        triangle: u32,
        direction: V3,
        here_present: bool,
        line: &mut LineRecordF32,
        there: &mut dyn FnMut(f32) -> bool,
    ) -> SeamAtF32 {
        let (entries, fully) = self.pack.glass_contacts().of_triangle(triangle as usize);
        let entry = if fully {
            entries.first()
        } else {
            entries
                .iter()
                .find(|entry| entry.covers_f32(point.position))
        };
        let Some(entry) = entry else {
            return SeamAtF32::None;
        };
        if entry.flags & CONTACT_NO_SEAM != 0 {
            return SeamAtF32::None;
        }
        let mut there_present = true;
        if self.flags() & SCENE_FLAG_SEE_THROUGH != 0 {
            let verdict = line.at_hit(self.pack, entry.part, there);
            if verdict == LineVerdictF32::Ended {
                return SeamAtF32::Ended;
            }
            there_present = verdict == LineVerdictF32::Stops;
        }
        let skip = SKIP_TOLERANCES_F32 * entry.tolerance;
        let leaving_here = facing_f32(point, direction).1;
        if !here_present && !there_present {
            return SeamAtF32::Straight { skip };
        }
        if here_present && there_present && entry.flags & CONTACT_SAME_GLASS != 0 {
            if line.crossings >= MAX_CROSSINGS_PER_LINE_F32 {
                return SeamAtF32::Ended;
            }
            line.crossings += 1;
            return SeamAtF32::Straight { skip };
        }
        let floor = self.floor_surface();
        let (surface, into_denser, absent) = if here_present && !there_present {
            (
                surface_at_f32(self.pack, floor, point.part, point.color),
                !leaving_here,
                Some((entry.part, true)),
            )
        } else if !here_present && there_present {
            (
                surface_at_f32(self.pack, floor, entry.part, entry.color),
                leaving_here,
                Some((point.part, false)),
            )
        } else {
            let denser = if leaving_here {
                entry.flags & CONTACT_COVER_DENSER != 0
            } else {
                entry.flags & CONTACT_HIT_DENSER != 0
            };
            (entry.interface, denser, None)
        };
        SeamAtF32::Interface(SeamF32 {
            surface,
            into_denser,
            skip,
            absent,
        })
    }
}

/// `seam_absent_toward`.
fn absent_toward(seam: &SeamF32, point: &PathPointF32, direction: V3) -> Option<u32> {
    let (part, on_normal_side) = seam.absent?;
    let toward_normal_side = dot(direction, point.geometric) >= 0.0;
    (toward_normal_side == on_normal_side).then_some(part)
}

fn ray_from(origin: V3, direction: V3) -> FlatRay {
    FlatRay {
        origin,
        direction,
        t_min: 0.0,
        t_max: PATH_FAR_F32,
        face_mask: FACE_BOTH,
    }
}

/// `path_sun_line`: the line toward the sun along `direction` from `vertex`, straight through
/// glass when `straight` and the scene's `sun_straight` both hold, `there` deciding a part's
/// presence at its first crossing; `line` is the shader's private line record, which the walk
/// restarts as the shader does.
pub fn sun_line_f32(
    scene: &SunLineScene<'_>,
    vertex: &SunVertexF32,
    direction: V3,
    straight: bool,
    line: &mut LineRecordF32,
    there: &mut dyn FnMut(f32) -> bool,
) -> SunLineF32 {
    let pack = scene.pack;
    let mut found = SunLineF32 {
        reached: false,
        transmittance: [1.0; 3],
        crossed: 0,
        first_face: None,
    };
    let see_through = scene.flags() & SCENE_FLAG_SEE_THROUGH != 0;
    let walk = scene.sun_straight && straight;
    if see_through || scene.sun_straight {
        line.restart();
    }
    let floor_surface = scene.floor_surface();
    let mut origin = next_origin_f32(&vertex.point, direction);
    let contact = vertex
        .contact
        .filter(|_| dot(direction, vertex.point.geometric) > 0.0);
    if let (true, Some(seam)) = (scene.sun_straight, vertex.seam) {
        if let Some(part) = absent_toward(&seam, &vertex.point, direction) {
            line.starts_past(part);
        }
        origin = seam_origin_f32(&vertex.point, direction, seam.skip);
    } else if let (true, Some(contact)) = (scene.sun_straight, contact) {
        let across = match contact {
            FloorContactF32::FloorUnder { rest_hit } => surface_point_f32(pack, &rest_hit),
            FloorContactF32::OverFloor => {
                let plane = scene.floor.map_or([0.0; 4], |(plane, _)| plane);
                floor_point_under_f32(vertex.point.position, plane)
            }
        };
        let mut present = true;
        if see_through && across.part != FLOOR_PART_F32 {
            match line.at_hit(pack, across.part, there) {
                LineVerdictF32::Ended => return found,
                verdict => present = verdict != LineVerdictF32::Passes,
            }
        }
        if present {
            if !walk {
                return found;
            }
            let surface = surface_at_f32(pack, floor_surface, across.part, across.color);
            if !passes_sun_straight_f32(&surface) {
                return found;
            }
            let cos = dot(facing_f32(&across, direction).0, direction).abs();
            found.transmittance = straight_transmittance_f32(&surface, cos);
            found.crossed = 1;
            line.restart();
            origin = next_origin_f32(&across, direction);
        }
    }
    if scene.floor_distance(origin, direction) >= 0.0 {
        return found;
    }
    let stop_at_any = !walk && !see_through;
    let mut ray = ray_from(origin, direction);
    loop {
        if stop_at_any {
            found.reached = !pack.tree().trace_any(&ray);
            return found;
        }
        let Some(hit) = pack.tree().trace_closest(&ray) else {
            found.reached = true;
            return found;
        };
        let point = surface_point_f32(pack, &hit);
        let mut present = true;
        if see_through {
            match line.at_hit(pack, point.part, there) {
                LineVerdictF32::Ended => return found,
                verdict => present = verdict != LineVerdictF32::Passes,
            }
        }
        if !walk {
            if present {
                return found;
            }
            ray = ray_from(next_origin_f32(&point, direction), direction);
            continue;
        }
        let mut seam = None;
        if scene.flags() & SCENE_FLAG_GLASS_CONTACTS != 0 {
            match scene.seam_at(&point, hit.source_index, direction, present, line, there) {
                SeamAtF32::Ended => return found,
                SeamAtF32::Straight { skip } => {
                    ray = ray_from(seam_origin_f32(&point, direction, skip), direction);
                    continue;
                }
                SeamAtF32::Interface(interface) => {
                    seam = Some(interface);
                    present = true;
                }
                SeamAtF32::None => {}
            }
        }
        if !present {
            ray = ray_from(next_origin_f32(&point, direction), direction);
            continue;
        }
        let surface = match &seam {
            Some(interface) => interface.surface,
            None => surface_at_f32(pack, floor_surface, point.part, point.color),
        };
        if !passes_sun_straight_f32(&surface) {
            return found;
        }
        let (shading, back_face) = facing_f32(&point, direction);
        let passed = straight_transmittance_f32(&surface, dot(shading, direction).abs());
        if found.crossed == 0 && seam.is_none() && back_face {
            found.first_face = Some((neg(shading), surface.ior));
        }
        if seam.is_none() && dot(direction, point.geometric) > 0.0 && scene.rests_on_floor(&point) {
            return found;
        }
        if found.crossed >= MAX_GLASS_CROSSINGS_F32 {
            return found;
        }
        found.crossed += 1;
        found.transmittance = std::array::from_fn(|c| found.transmittance[c] * passed[c]);
        let largest = found.transmittance[0]
            .max(found.transmittance[1])
            .max(found.transmittance[2]);
        if largest <= 0.0 {
            return found;
        }
        line.restart();
        ray = match &seam {
            Some(interface) => {
                if let Some(part) = absent_toward(interface, &point, direction) {
                    line.starts_past(part);
                }
                ray_from(
                    seam_origin_f32(&point, direction, interface.skip),
                    direction,
                )
            }
            None => ray_from(next_origin_f32(&point, direction), direction),
        };
    }
}
