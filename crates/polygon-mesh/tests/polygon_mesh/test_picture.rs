//! The picture the face-texture tests hand a mesh: a packed four-bytes-a-pixel buffer behind the
//! kernel's [`TexturePixels`] trait, laid out as the app's byte image lays out a new picture
//! (`width * height * 4` bytes, all zero), so a mesh wearing it hashes as one wearing that image.

#![allow(dead_code)]

use std::any::Any;
use std::sync::Arc;

use solidmesh_math::TexturePixels;

/// A `width` by `height` picture of four bytes a pixel.
#[derive(Debug, Clone)]
pub struct TestPicture {
    width: i32,
    height: i32,
    bytes: Vec<u8>,
}

impl TestPicture {
    /// A picture of every byte zero.
    pub fn new(width: i32, height: i32) -> TestPicture {
        // Test sizes are small and non-negative, so the product fits.
        let length = (width.max(0) as usize) * (height.max(0) as usize) * 4;
        TestPicture {
            width,
            height,
            bytes: vec![0; length],
        }
    }

    /// The bytes, to paint a pattern into.
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        &mut self.bytes
    }
}

impl TexturePixels for TestPicture {
    fn width(&self) -> i32 {
        self.width
    }

    fn height(&self) -> i32 {
        self.height
    }

    fn pixel_bytes(&self) -> &[u8] {
        &self.bytes
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn into_any_arc(self: Arc<Self>) -> Arc<dyn Any + Send + Sync> {
        self
    }
}
