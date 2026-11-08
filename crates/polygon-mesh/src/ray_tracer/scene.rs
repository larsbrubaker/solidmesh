//! The scene a [`RayTracer`](super::RayTracer) traces into and its [`Background`].
//!
//! The scene holds no camera or lights: only a shading image renderer would read them, and the
//! tracer here follows primary rays. What it reads is the shapes.

use solidmesh_math::ColorF;

use super::{BoundingVolumeHierarchy, BvhCreationOptions, TraceableRef};

/// The background of a scene.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Background {
    /// The color of the background.
    pub color: ColorF,
    /// The ambient lighting used, in [0, 1].
    pub ambience: f64,
}

impl Background {
    /// Specifies the background of the scene.
    pub fn new(color: ColorF, ambience: f64) -> Self {
        Background { color, ambience }
    }
}

/// A scene: a background and the objects in it, called the shapes.
pub struct Scene {
    pub background: Background,
    pub shapes: Vec<TraceableRef>,
}

impl Scene {
    /// An empty scene on a transparent black background with 0.2 ambience.
    pub fn new() -> Self {
        Scene {
            background: Background::new(ColorF::new(0.0, 0.0, 0.0, 0.0), 0.2),
            shapes: Vec::new(),
        }
    }

    /// Removes the shapes from `shapes`, puts them in a bounding volume hierarchy and adds that as
    /// the single element of `shapes`. Returns the hierarchy, `None` (leaving `shapes` as it was)
    /// when there were no shapes.
    pub fn move_shapes_into_bounding_volume_hierarchy(&mut self) -> Option<TraceableRef> {
        let root_object = BoundingVolumeHierarchy::create_new_hierarchy(
            self.shapes.clone(),
            BvhCreationOptions::default(),
        );
        if let Some(root) = &root_object {
            self.shapes.clear();
            self.shapes.push(root.clone());
        }

        root_object
    }
}

impl Default for Scene {
    fn default() -> Self {
        Self::new()
    }
}
