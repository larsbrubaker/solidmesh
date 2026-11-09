//! A flat, four-wide bounding-volume tree in plain `f32`/`u32` buffers, for tracing rays in a
//! WebGPU compute shader. WGSL has no pointers or recursion and browsers have no hardware ray
//! queries, so the tree a shader walks has to be an array of nodes it can index and a loop with a
//! fixed-size stack. [`FlatTree::build`] takes world-space triangles, builds the crate's usual
//! BVH ([`super::BvhBuilderLocallyOrderedClustering`], no second builder), collapses it into
//! these buffers (`build`), and [`FlatTree::trace_closest`] and [`FlatTree::trace_any`] are the
//! reference traversal over exactly those buffers (`traverse`), written to port line for line to
//! WGSL. `layout` holds the two records and their bytes, and `shader` the WGSL traversal itself
//! ([`TRACE_WGSL`], from `trace.wgsl`) with the ray and hit records it adds. `scatter_shader` is
//! the WGSL port of the scattering model ([`SCATTER_WGSL`]) and the entry point that checks it
//! against the CPU model, reading [`SurfaceRecord`] and [`albedo_table_bytes`].
//!
//! This module is the one place geometry narrows from `f64` to `f32`, because WGSL has no `f64`
//! and a GPU read of `f32` is the point of the buffers. Everything upstream stays `f64` in a
//! stable operation order; nothing here feeds back into a mesh, a saved design or a boolean.
//!
//! # Byte layout
//!
//! Both buffers are little-endian, tightly packed arrays, ready for a WebGPU storage binding
//! ([`FlatTree::node_bytes`], [`FlatTree::triangle_bytes`]). The offsets are pinned by a test.
//!
//! The **node buffer** is an array of 112-byte records, stride 112, element 0 the root:
//!
//! ```text
//! offset  size  field     WGSL
//!      0    16  min_x     vec4<f32>   minimum x of each of the four children
//!     16    16  max_x     vec4<f32>
//!     32    16  min_y     vec4<f32>
//!     48    16  max_y     vec4<f32>
//!     64    16  min_z     vec4<f32>
//!     80    16  max_z     vec4<f32>
//!     96    16  child     vec4<u32>   one code per child
//! ```
//!
//! which is, field for field, `struct Node { min_x: vec4<f32>, max_x: vec4<f32>, min_y:
//! vec4<f32>, max_y: vec4<f32>, min_z: vec4<f32>, max_z: vec4<f32>, child: vec4<u32> }`, size 112
//! and align 16 under WGSL's storage-buffer rules, so `array<Node>` has stride 112. Storing the
//! four children's boxes in the parent means one node fetch tests all four, and the
//! structure-of-arrays form is `vec4` arithmetic in the shader.
//!
//! A child code is one of (see [`child_code`] for the constants):
//! - [`child_code::EMPTY`] (`0xFFFF_FFFF`): the slot is unused, and its box is all zeros. A shader
//!   tests the code, not the box, whose values stay finite because WGSL may assume no infinities.
//! - top bit clear: the index of another node in this buffer.
//! - top bit set: a leaf, `0x8000_0000 | first_triangle << 2 | (count - 1)`, the run of `count`
//!   (1 to [`MAX_LEAF_TRIANGLES`]) records starting at `first_triangle` in the triangle buffer.
//!
//! The **triangle buffer** is an array of 40-byte records, stride 40:
//!
//! ```text
//! offset  size  field         WGSL
//!      0    12  v0            array<f32, 3>
//!     12    12  v1            array<f32, 3>
//!     24    12  v2            array<f32, 3>
//!     36     4  source_index  u32           this triangle's position in the input list
//! ```
//!
//! which is `struct Triangle { v0: array<f32, 3>, v1: array<f32, 3>, v2: array<f32, 3>,
//! source_index: u32 }`, size 40 and align 4. The corners are arrays and not `vec3<f32>` because a
//! `vec3` is aligned to 16 bytes in a storage buffer: three of them with the index come to a
//! stride of 48, 8 of those bytes wasted, where these arrays need 40.
//!
//! # Width and leaf size
//!
//! Four children: the width a `vec4` tests in one go, and each halving of tree depth halves the
//! node fetches, which are what a GPU pays for. Leaves hold up to [`MAX_LEAF_TRIANGLES`]
//! triangles side by side, since a run of records in one buffer costs one fetch and nodes over
//! single triangles would be most of the node buffer.
//!
//! # Conservative bounds
//!
//! Each corner's bounds are the `f64` coordinate rounded outward to `f32`, then stepped one more
//! `f32` outward even when the cast was exact ([`bounds_min_f32`], [`bounds_max_f32`]). The
//! rounding covers narrowing: the bounds hold both the exact corner and the nearest-`f32` corner
//! the triangle buffer stores. The extra step puts an origin lying exactly on one of a box's
//! planes (an axis-aligned camera over a grid of whole numbers) strictly inside it, and turns a
//! zero-thickness plate's box into a slab with thickness. A ray running parallel to a slab is
//! decided by that containment directly, not by a distance from its reciprocal (`traverse`).
//!
//! # Traversal stack bound
//!
//! A traversal keeps a fixed stack of [`STACK_CAPACITY`] entries. Popping one node and pushing at
//! most four children grows the stack by at most three per level walked, so a tree of depth `D`
//! needs at most `3 * D + 1` entries. [`FlatTree::build`] refuses a tree deeper than
//! [`MAX_TREE_DEPTH`] with [`FlatTreeError::TreeTooDeep`], so the stack cannot overflow on any
//! tree it returns.
//!
//! # Limits
//!
//! A corner coordinate beyond ±[`COORDINATE_LIMIT`] or not finite is refused: narrowed bounds and
//! the slab arithmetic must stay finite, since WGSL implementations may assume no infinities or
//! NaNs. A ray's origin is the caller's to keep within the same limit ([`FlatRay`]), and its
//! direction must be a unit vector: rays come straight from a camera or a surface, so the
//! traversal does not re-check them.

mod build;
mod layout;
mod scatter_shader;
mod shader;
mod traverse;

use std::fmt;

pub use super::scattering::{
    albedo_table_bytes, SurfaceRecord, ALBEDO_ENTRY_COUNT, ALBEDO_ENTRY_STRIDE, SURFACE_HAS_GLASS,
    SURFACE_INDEX_MATCHED, SURFACE_RECORD_STRIDE, SURFACE_SMOOTH,
};
pub use layout::{child_code, FlatNode, FlatTriangle, NODE_STRIDE, TRIANGLE_STRIDE};
pub use scatter_shader::{
    ScatterCheckCase, ScatterCheckResult, SAMPLE_FOUND, SAMPLE_SPECULAR, SCATTER_CASE_STRIDE,
    SCATTER_CHECK_WGSL, SCATTER_RESULT_STRIDE, SCATTER_WGSL,
};
pub use shader::{hit_from_le_bytes, hit_to_le_bytes, HIT_STRIDE, NO_HIT, RAY_STRIDE, TRACE_WGSL};
pub use traverse::{FlatHit, FlatRay, FACE_BACK, FACE_BOTH, FACE_FRONT};

use solidmesh_math::Vector3;

/// How many triangles one leaf may hold.
pub const MAX_LEAF_TRIANGLES: u32 = 4;

/// The fixed traversal stack's size; see the module header for why it cannot overflow.
pub const STACK_CAPACITY: usize = 128;

/// The deepest tree [`FlatTree::build`] returns: `3 * depth + 1` stays within [`STACK_CAPACITY`].
pub const MAX_TREE_DEPTH: u32 = 42;

/// The largest corner coordinate magnitude accepted.
pub const COORDINATE_LIMIT: f64 = 1e18;

/// The most triangles a leaf code can address: `first_triangle` has 29 bits and its top value is
/// reserved so no leaf code can equal [`child_code::EMPTY`].
pub const MAX_TRIANGLES: u64 = (1 << 29) - 1;

/// WebGPU's default `maxStorageBufferBindingSize`, for callers with no adapter limit in hand.
pub const WEBGPU_DEFAULT_MAX_STORAGE_BINDING: u64 = 134_217_728;

/// Which of the two buffers a [`FlatTreeError::BufferTooLarge`] is about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlatTreeBuffer {
    Nodes,
    Triangles,
}

impl fmt::Display for FlatTreeBuffer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            FlatTreeBuffer::Nodes => "node",
            FlatTreeBuffer::Triangles => "triangle",
        })
    }
}

/// Why a [`FlatTree`] was not built.
#[derive(Debug, Clone, PartialEq)]
pub enum FlatTreeError {
    /// There were no triangles, so there is nothing to trace and no buffer a GPU could bind.
    NoTriangles,
    /// More triangles than a leaf code can address.
    TooManyTriangles { count: u64, limit: u64 },
    /// A corner coordinate is not finite or is beyond [`COORDINATE_LIMIT`].
    InvalidCoordinate { triangle_index: u64 },
    /// A buffer would exceed the caller's storage binding limit.
    BufferTooLarge {
        buffer: FlatTreeBuffer,
        needed_bytes: u64,
        limit_bytes: u64,
        triangle_count: u64,
        /// About how many triangles of this kind of scene would have fit in both buffers.
        triangles_that_fit: u64,
    },
    /// The tree would be deeper than the traversal stack allows.
    TreeTooDeep { depth: u32, limit: u32 },
    /// The BVH handed back a leaf that is not one of the triangles built for it.
    UnexpectedBvhLeaf,
}

impl fmt::Display for FlatTreeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlatTreeError::NoTriangles => write!(
                f,
                "there are no triangles to trace: add a part, or skip tracing an empty scene"
            ),
            FlatTreeError::TooManyTriangles { count, limit } => write!(
                f,
                "this scene has {count} triangles and the GPU tree can address at most {limit}: \
                 simplify or split the scene to at most {limit} triangles"
            ),
            FlatTreeError::InvalidCoordinate { triangle_index } => write!(
                f,
                "triangle {triangle_index} has a corner that is not a finite number within \
                 {COORDINATE_LIMIT:e} of the origin: repair the mesh or scale it down before \
                 tracing it on the GPU"
            ),
            FlatTreeError::BufferTooLarge {
                buffer,
                needed_bytes,
                limit_bytes,
                triangle_count,
                triangles_that_fit,
            } => write!(
                f,
                "the {buffer} buffer would need {needed_bytes} bytes but this GPU allows \
                 {limit_bytes} bytes in one storage binding; the scene has {triangle_count} \
                 triangles and about {triangles_that_fit} would have fit: simplify the scene to \
                 that many triangles, or request a larger maxStorageBufferBindingSize"
            ),
            FlatTreeError::TreeTooDeep { depth, limit } => write!(
                f,
                "the GPU tree for this scene would be {depth} levels deep and the GPU traversal \
                 can walk at most {limit}: this mesh's triangles are spread very unevenly; \
                 remesh it more evenly or trace it on the CPU"
            ),
            FlatTreeError::UnexpectedBvhLeaf => write!(
                f,
                "the BVH builder returned a leaf that is not a triangle, which is a bug in the \
                 ray tracer: please report it"
            ),
        }
    }
}

impl std::error::Error for FlatTreeError {}

/// The sizes a [`FlatTree`]'s buffers came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlatTreeSizes {
    pub node_count: u64,
    pub triangle_count: u64,
    pub node_bytes: u64,
    pub triangle_bytes: u64,
    /// The levels of nodes on the longest path from the root, counting the root as 1.
    pub depth: u32,
}

/// A tree in GPU-ready buffers; see the module header for the layout.
#[derive(Debug, Clone, PartialEq)]
pub struct FlatTree {
    nodes: Vec<FlatNode>,
    triangles: Vec<FlatTriangle>,
    depth: u32,
}

impl FlatTree {
    /// The tree over `triangles` (world space; a hit's `source_index` is a position in this
    /// list), refused with a [`FlatTreeError`] when a buffer would exceed
    /// `max_storage_binding_size` bytes ([`WEBGPU_DEFAULT_MAX_STORAGE_BINDING`] is WebGPU's
    /// default). The same input always gives the same bytes.
    pub fn build(
        triangles: &[[Vector3; 3]],
        max_storage_binding_size: u64,
    ) -> Result<FlatTree, FlatTreeError> {
        build::build(triangles, max_storage_binding_size)
    }

    pub fn nodes(&self) -> &[FlatNode] {
        &self.nodes
    }

    pub fn triangles(&self) -> &[FlatTriangle] {
        &self.triangles
    }

    /// The node buffer's bytes, ready to upload.
    pub fn node_bytes(&self) -> Vec<u8> {
        layout::nodes_to_bytes(&self.nodes)
    }

    /// The triangle buffer's bytes, ready to upload.
    pub fn triangle_bytes(&self) -> Vec<u8> {
        layout::triangles_to_bytes(&self.triangles)
    }

    /// The buffer sizes and the tree's depth.
    pub fn sizes(&self) -> FlatTreeSizes {
        FlatTreeSizes {
            node_count: self.nodes.len() as u64,
            triangle_count: self.triangles.len() as u64,
            node_bytes: self.nodes.len() as u64 * NODE_STRIDE as u64,
            triangle_bytes: self.triangles.len() as u64 * TRIANGLE_STRIDE as u64,
            depth: self.depth,
        }
    }

    /// The nearest hit of `ray` within its window and face mask.
    pub fn trace_closest(&self, ray: &FlatRay) -> Option<FlatHit> {
        traverse::trace_closest(&self.nodes, &self.triangles, ray)
    }

    /// True when `ray` hits anything within its window and face mask, stopping at the first hit.
    pub fn trace_any(&self, ray: &FlatRay) -> bool {
        traverse::trace_any(&self.nodes, &self.triangles, ray)
    }
}

/// The stored lower bound for `value`: rounded down to `f32`, then one more step down.
pub fn bounds_min_f32(value: f64) -> f32 {
    let cast = value as f32;
    let rounded_down = if f64::from(cast) > value {
        cast.next_down()
    } else {
        cast
    };
    rounded_down.next_down()
}

/// The stored upper bound for `value`: rounded up to `f32`, then one more step up.
pub fn bounds_max_f32(value: f64) -> f32 {
    let cast = value as f32;
    let rounded_up = if f64::from(cast) < value {
        cast.next_up()
    } else {
        cast
    };
    rounded_up.next_up()
}
