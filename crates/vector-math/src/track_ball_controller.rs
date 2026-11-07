//! `TrackBallController`: turns mouse drags into rotation, translation and scale of a
//! [`WorldView`].
//!
//! The methods that change the view take it as `&mut WorldView`, so the controller holds only
//! its own drag state and never a reference to the view. A mouse event while the tracking type
//! is `None` has nothing to drive, so the method returns [`TrackBallError::NotTracking`].

use crate::{Matrix4X4, Quaternion, Vector2, Vector3, WorldView};

/// What a drag does.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum TrackBallTransformType {
    #[default]
    None,
    Translation,
    Rotation,
    Scale,
}

/// A mouse event arrived with no drag type to apply it to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackBallError {
    NotTracking,
}

impl std::fmt::Display for TrackBallError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "the track ball has no transform type to apply")
    }
}

impl std::error::Error for TrackBallError {}

/// Drag state for a track ball.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackBallController {
    local_to_screen_transform: Matrix4X4,
    mouse_down_position: Vector2,
    pub screen_center: Vector2,
    last_translation_mouse_position: Vector2,
    last_scale_mouse_position: Vector2,
    current_tracking_type: TrackBallTransformType,
    pub track_ball_radius: f64,
}

impl TrackBallController {
    /// A controller with the given radius (usually 1).
    pub fn new(track_ball_radius: f64) -> Self {
        TrackBallController {
            local_to_screen_transform: Matrix4X4::default(),
            mouse_down_position: Vector2::ZERO,
            screen_center: Vector2::ZERO,
            last_translation_mouse_position: Vector2::ZERO,
            last_scale_mouse_position: Vector2::ZERO,
            current_tracking_type: TrackBallTransformType::None,
            track_ball_radius,
        }
    }

    pub fn current_tracking_type(&self) -> TrackBallTransformType {
        self.current_tracking_type
    }

    /// Where a drag from `start_position` to `end_position` lands on the track ball sphere.
    pub fn map_move_to_sphere(
        screen_center: Vector2,
        track_ball_radius: f64,
        start_position: Vector2,
        end_position: Vector2,
        rotate_on_screen_z: bool,
    ) -> Vector3 {
        if rotate_on_screen_z {
            let angle_to_travel = screen_center.get_delta_angle(start_position, end_position);

            // now rotate that position about z in the direction of the screen vector
            Vector3::new(1.0, 0.0, 0.0)
                .transform(Matrix4X4::create_rotation_z(angle_to_travel / 2.0))
        } else {
            let delta_from_start_pixels = end_position - start_position;
            let delta_on_surface = Vector2::new(
                delta_from_start_pixels.x / track_ball_radius,
                delta_from_start_pixels.y / track_ball_radius,
            );

            let length_on_surface_radi = delta_on_surface.length();

            // get this rotation on the surface of the sphere about y
            let position_about_y = Vector3::new(0.0, 0.0, 1.0)
                .transform(Matrix4X4::create_rotation_y(length_on_surface_radi));

            // get the angle that this distance travels around the sphere
            let angle_to_travel = delta_on_surface.y.atan2(delta_on_surface.x);

            // now rotate that position about z in the direction of the screen vector
            position_about_y.transform(Matrix4X4::create_rotation_z(angle_to_travel))
        }
    }

    /// Starts a drag of the given type (usually `Rotation`).
    pub fn on_mouse_down(
        &mut self,
        mouse_position: Vector2,
        screen_to_local: Matrix4X4,
        track_type: TrackBallTransformType,
    ) -> Result<(), TrackBallError> {
        self.current_tracking_type = track_type;
        match self.current_tracking_type {
            TrackBallTransformType::Rotation => {
                self.mouse_down_position = mouse_position;
            }
            TrackBallTransformType::Translation => {
                self.local_to_screen_transform = screen_to_local.inverted();
                self.last_translation_mouse_position = mouse_position;
            }
            TrackBallTransformType::Scale => {
                self.last_scale_mouse_position = mouse_position;
            }
            TrackBallTransformType::None => return Err(TrackBallError::NotTracking),
        }

        Ok(())
    }

    /// Applies the drag so far to `world`.
    pub fn on_mouse_move(
        &mut self,
        world: &mut WorldView,
        mouse_position: Vector2,
    ) -> Result<(), TrackBallError> {
        match self.current_tracking_type {
            TrackBallTransformType::Rotation => {
                let active_rotation_quaternion = Self::get_rotation_for_move(
                    self.screen_center,
                    self.track_ball_radius,
                    self.mouse_down_position,
                    mouse_position,
                    false,
                );

                if active_rotation_quaternion != Quaternion::IDENTITY {
                    self.mouse_down_position = mouse_position;
                    world.set_rotation_matrix(
                        world.rotation_matrix()
                            * Matrix4X4::create_rotation_quaternion(active_rotation_quaternion),
                    );
                }
            }
            TrackBallTransformType::Translation => {
                let mouse_delta = mouse_position - self.last_translation_mouse_position;
                let scaled_delta = mouse_delta / self.screen_center.x * 4.75;
                let mut offset = Vector3::new(scaled_delta.x, scaled_delta.y, 0.0);
                offset = offset.transform_position(world.rotation_matrix().inverted());
                offset = offset.transform_position(self.local_to_screen_transform);
                world.set_translation_matrix(
                    world.translation_matrix() * Matrix4X4::create_translation_vector(offset),
                );
                self.last_translation_mouse_position = mouse_position;
            }
            TrackBallTransformType::Scale => {
                let mouse_delta = mouse_position - self.last_scale_mouse_position;
                let mut zoom_delta = 1.0;
                if mouse_delta.y < 0.0 {
                    zoom_delta = 1.0 - (-mouse_delta.y / 100.0);
                } else if mouse_delta.y > 0.0 {
                    zoom_delta = 1.0 + (mouse_delta.y / 100.0);
                }

                world.set_scale(world.scale() * zoom_delta);
                self.last_scale_mouse_position = mouse_position;
            }
            TrackBallTransformType::None => return Err(TrackBallError::NotTracking),
        }

        Ok(())
    }

    /// The rotation a drag from `start_position` to `end_position` makes; identity when the
    /// drag is too small to define an axis.
    pub fn get_rotation_for_move(
        screen_center: Vector2,
        track_ball_radius: f64,
        start_position: Vector2,
        end_position: Vector2,
        rotate_on_z: bool,
    ) -> Quaternion {
        let mut active_rotation_quaternion = Quaternion::IDENTITY;

        //Map the point to the sphere
        let move_sphere_position = Self::map_move_to_sphere(
            screen_center,
            track_ball_radius,
            start_position,
            end_position,
            rotate_on_z,
        );

        //Return the quaternion equivalent to the rotation
        //Compute the vector perpendicular to the begin and end vectors
        let mut rotation_start_3d = Vector3::new(0.0, 0.0, 1.0);
        if rotate_on_z {
            rotation_start_3d = Vector3::new(1.0, 0.0, 0.0);
        }

        let perp = rotation_start_3d.cross(move_sphere_position);

        //Compute the length of the perpendicular vector
        let epsilon = 1.0e-5;
        if perp.length() > epsilon {
            //if its non-zero
            //We're ok, so return the perpendicular vector as the transform after all
            active_rotation_quaternion.x = perp.x;
            active_rotation_quaternion.y = perp.y;
            active_rotation_quaternion.z = perp.z;
            //In the quaternion values, w is cosine (theta / 2), where theta is the rotation angle
            active_rotation_quaternion.w = -rotation_start_3d.dot(move_sphere_position);
        }

        active_rotation_quaternion
    }

    /// Ends the drag.
    pub fn on_mouse_up(&mut self) -> Result<(), TrackBallError> {
        if self.current_tracking_type == TrackBallTransformType::None {
            return Err(TrackBallError::NotTracking);
        }

        self.current_tracking_type = TrackBallTransformType::None;
        Ok(())
    }

    /// Zooms `world` in (positive delta) or out (negative delta).
    pub fn on_mouse_wheel(&self, world: &mut WorldView, wheel_delta: i32) {
        let mut zoom_delta = 1.0;
        if wheel_delta > 0 {
            zoom_delta = 1.2;
        } else if wheel_delta < 0 {
            zoom_delta = 0.8;
        }

        world.set_scale(world.scale() * zoom_delta);
    }
}
