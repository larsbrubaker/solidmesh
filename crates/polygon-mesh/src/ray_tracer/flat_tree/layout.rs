//! The two records of a flat tree and their bytes: [`FlatNode`] and [`FlatTriangle`], laid out as
//! the module header in `flat_tree` specifies. A shader reads these bytes through a WGSL struct,
//! so the serialization is written field by field in little-endian order instead of
//! reinterpreting memory, and decoding is its exact inverse.

/// Node record size in bytes.
pub const NODE_STRIDE: usize = 112;

/// Triangle record size in bytes.
pub const TRIANGLE_STRIDE: usize = 40;

/// The values a node's `child` entry can take.
pub mod child_code {
    /// The slot is unused.
    pub const EMPTY: u32 = 0xFFFF_FFFF;
    /// Set on a leaf code, clear on a node index.
    pub const LEAF_BIT: u32 = 0x8000_0000;
    /// How many low bits hold `count - 1` in a leaf code.
    pub const COUNT_BITS: u32 = 2;
    /// The mask of those bits.
    pub const COUNT_MASK: u32 = (1 << COUNT_BITS) - 1;

    /// The code of a leaf holding `count` records from `first_triangle`.
    pub const fn leaf(first_triangle: u32, count: u32) -> u32 {
        LEAF_BIT | (first_triangle << COUNT_BITS) | (count - 1)
    }

    /// True when `code` is a leaf and not an empty slot.
    pub const fn is_leaf(code: u32) -> bool {
        code & LEAF_BIT != 0 && code != EMPTY
    }

    /// The first triangle record of a leaf code.
    pub const fn first_triangle(code: u32) -> u32 {
        (code & !LEAF_BIT) >> COUNT_BITS
    }

    /// The record count of a leaf code.
    pub const fn count(code: u32) -> u32 {
        (code & COUNT_MASK) + 1
    }
}

/// A node with up to four children: one [`NODE_STRIDE`]-byte record.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FlatNode {
    pub min_x: [f32; 4],
    pub max_x: [f32; 4],
    pub min_y: [f32; 4],
    pub max_y: [f32; 4],
    pub min_z: [f32; 4],
    pub max_z: [f32; 4],
    pub child: [u32; 4],
}

impl FlatNode {
    /// A node of four empty slots.
    pub const EMPTY: FlatNode = FlatNode {
        min_x: [0.0; 4],
        max_x: [0.0; 4],
        min_y: [0.0; 4],
        max_y: [0.0; 4],
        min_z: [0.0; 4],
        max_z: [0.0; 4],
        child: [child_code::EMPTY; 4],
    };

    /// This record's bytes, in buffer order.
    pub fn to_le_bytes(&self) -> [u8; NODE_STRIDE] {
        let mut bytes = [0u8; NODE_STRIDE];
        let floats = [
            self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z,
        ];
        let mut offset = 0;
        for lane in floats {
            for value in lane {
                bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
                offset += 4;
            }
        }
        for value in self.child {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        bytes
    }

    /// The record these bytes encode.
    pub fn from_le_bytes(bytes: &[u8; NODE_STRIDE]) -> FlatNode {
        let mut node = FlatNode::EMPTY;
        let lanes = [
            &mut node.min_x,
            &mut node.max_x,
            &mut node.min_y,
            &mut node.max_y,
            &mut node.min_z,
            &mut node.max_z,
        ];
        let mut offset = 0;
        for lane in lanes {
            for value in lane {
                *value = f32::from_le_bytes(word_at(bytes, offset));
                offset += 4;
            }
        }
        for value in &mut node.child {
            *value = u32::from_le_bytes(word_at(bytes, offset));
            offset += 4;
        }
        node
    }
}

/// One triangle: one [`TRIANGLE_STRIDE`]-byte record.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FlatTriangle {
    pub v0: [f32; 3],
    pub v1: [f32; 3],
    pub v2: [f32; 3],
    /// This triangle's position in the list given to the build.
    pub source_index: u32,
}

impl FlatTriangle {
    /// This record's bytes, in buffer order.
    pub fn to_le_bytes(&self) -> [u8; TRIANGLE_STRIDE] {
        let mut bytes = [0u8; TRIANGLE_STRIDE];
        let mut offset = 0;
        for corner in [self.v0, self.v1, self.v2] {
            for value in corner {
                bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
                offset += 4;
            }
        }
        bytes[offset..offset + 4].copy_from_slice(&self.source_index.to_le_bytes());
        bytes
    }

    /// The record these bytes encode.
    pub fn from_le_bytes(bytes: &[u8; TRIANGLE_STRIDE]) -> FlatTriangle {
        let mut triangle = FlatTriangle {
            v0: [0.0; 3],
            v1: [0.0; 3],
            v2: [0.0; 3],
            source_index: 0,
        };
        let mut offset = 0;
        for corner in [&mut triangle.v0, &mut triangle.v1, &mut triangle.v2] {
            for value in corner {
                *value = f32::from_le_bytes(word_at(bytes, offset));
                offset += 4;
            }
        }
        triangle.source_index = u32::from_le_bytes(word_at(bytes, offset));
        triangle
    }
}

/// The four bytes at `offset`.
fn word_at(bytes: &[u8], offset: usize) -> [u8; 4] {
    [
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ]
}

/// The node buffer's bytes.
pub(super) fn nodes_to_bytes(nodes: &[FlatNode]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(nodes.len() * NODE_STRIDE);
    for node in nodes {
        bytes.extend_from_slice(&node.to_le_bytes());
    }
    bytes
}

/// The triangle buffer's bytes.
pub(super) fn triangles_to_bytes(triangles: &[FlatTriangle]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(triangles.len() * TRIANGLE_STRIDE);
    for triangle in triangles {
        bytes.extend_from_slice(&triangle.to_le_bytes());
    }
    bytes
}
