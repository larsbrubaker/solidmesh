//! `WorldView`: the 3D view's camera — rotation and translation matrices, the modelview
//! matrix built from them, and a perspective or orthographic projection — with the conversions
//! between worldspace, viewspace, NDC and screenspace.
//!
//! Values only the view itself may change are getter methods; `width`/`height` are public
//! fields. The rotation, translation, scale and eye-position setters are `set_*` methods that
//! rebuild the modelview matrix, so it never goes stale. The perspective setters return an
//! error for a near plane that sanitising cannot make finite, instead of a NaN projection.
//! There is no screen-to-world conversion: picking goes through rays instead. The projection
//! math keeps a fixed term order, so the matrices are identical on every platform.

use crate::ieee_math::{max_f64, min_f64};
use crate::{math_helper, MathError, Matrix4X4, Quaternion, Ray, Vector2, Vector3, Vector4};

/// The camera and projection of a 3D view.
#[derive(Debug, Clone, PartialEq)]
pub struct WorldView {
    pub height: f64,
    pub width: f64,
    v_fov_degrees: f64,
    h_fov_degrees: f64,
    near_plane_height_in_viewspace: f64,
    is_orthographic: bool,
    modelview_matrix: Matrix4X4,
    projection_matrix: Matrix4X4,
    inverse_modelview_matrix: Matrix4X4,
    inverse_projection_matrix: Matrix4X4,
    near_z: f64,
    far_z: f64,
    rotation_matrix: Matrix4X4,
    translation_matrix: Matrix4X4,
}

impl WorldView {
    pub const DEFAULT_PERSPECTIVE_VFOV_DEGREES: f64 = 45.0;
    pub const DEFAULT_NEAR_Z: f64 = 0.1;
    pub const DEFAULT_FAR_Z: f64 = 100.0;
    /// Force this minimum on near Z.
    /// As the dynamic near plane will take this value inside an AABB, it'll be the same as the
    /// default for now.
    pub const PERSPECTIVE_PROJECTION_MINIMUM_NEAR_Z: f64 = 0.001;
    /// far / near >= this
    const PERSPECTIVE_PROJECTION_MINIMUM_FAR_NEAR_RATIO: f64 = 1.001;
    /// Force this minimum distance between the near and far planes for the orthographic
    /// projection.
    const ORTHOGRAPHIC_PROJECTION_MINIMUM_NEAR_FAR_GAP: f64 = 0.001;
    const CAMERA_Z_TRANSLATION_FUDGE: f64 = -7.0;
    /// What [`WorldView::get_world_units_per_screen_pixel_at_position`] answers when the camera
    /// or the position gives no usable number. It is the value that call used to clamp
    /// everything to, so a degenerate camera behaves exactly as it always has.
    pub const DEGENERATE_WORLD_UNITS_PER_PIXEL: f64 = 5.0;

    /// The orthographic minimum height, derived from the perspective minimum near Z.
    pub fn orthographic_projection_minimum_height() -> f64 {
        Self::calc_perspective_height(
            Self::PERSPECTIVE_PROJECTION_MINIMUM_NEAR_Z,
            Self::DEFAULT_PERSPECTIVE_VFOV_DEGREES,
        )
    }

    /// A typical perspective projection and a default camera transform for a viewport.
    pub fn new(width: f64, height: f64) -> Self {
        let mut world = WorldView {
            height: 0.0,
            width: 0.0,
            v_fov_degrees: 0.0,
            h_fov_degrees: 0.0,
            near_plane_height_in_viewspace: 0.0,
            is_orthographic: false,
            modelview_matrix: Matrix4X4::IDENTITY,
            projection_matrix: Matrix4X4::IDENTITY,
            inverse_modelview_matrix: Matrix4X4::IDENTITY,
            inverse_projection_matrix: Matrix4X4::IDENTITY,
            near_z: 0.0,
            far_z: 0.0,
            rotation_matrix: Matrix4X4::IDENTITY,
            translation_matrix: Matrix4X4::IDENTITY,
        };
        world
            .calculate_perspective_matrix(width, height, Self::DEFAULT_NEAR_Z, Self::DEFAULT_FAR_Z)
            .expect("the default near and far planes are a valid perspective projection");
        world.calculate_modelview_matrix();
        world
    }

    pub fn viewport_size(&self) -> Vector2 {
        Vector2::new(self.width, self.height)
    }

    /// The vertical FOV of the latest perspective projection. Untouched by orthographic
    /// projection.
    pub fn v_fov_degrees(&self) -> f64 {
        self.v_fov_degrees
    }

    /// The horizontal FOV of the latest perspective projection. Untouched by orthographic
    /// projection. May not be symmetric due to center offset.
    pub fn h_fov_degrees(&self) -> f64 {
        self.h_fov_degrees
    }

    /// The height of the near plane in viewspace in either projection mode. For orthographic,
    /// this is constant for all Z. This value is used to maintain orthographic scale during
    /// viewport resize.
    pub fn near_plane_height_in_viewspace(&self) -> f64 {
        self.near_plane_height_in_viewspace
    }

    pub fn is_orthographic(&self) -> bool {
        self.is_orthographic
    }

    pub fn modelview_matrix(&self) -> Matrix4X4 {
        self.modelview_matrix
    }

    pub fn projection_matrix(&self) -> Matrix4X4 {
        self.projection_matrix
    }

    pub fn inverse_modelview_matrix(&self) -> Matrix4X4 {
        self.inverse_modelview_matrix
    }

    pub fn inverse_projection_matrix(&self) -> Matrix4X4 {
        self.inverse_projection_matrix
    }

    /// Signed distance of the near plane from the eye along the forward axis. Positive for
    /// perspective projection.
    pub fn near_z(&self) -> f64 {
        self.near_z
    }

    /// Signed distance of the far plane from the eye along the forward axis. Positive for
    /// perspective projection.
    pub fn far_z(&self) -> f64 {
        self.far_z
    }

    pub fn rotation_matrix(&self) -> Matrix4X4 {
        self.rotation_matrix
    }

    pub fn set_rotation_matrix(&mut self, value: Matrix4X4) {
        if self.rotation_matrix != value {
            self.rotation_matrix = value;
            self.on_transform_changed();
        }
    }

    pub fn translation_matrix(&self) -> Matrix4X4 {
        self.translation_matrix
    }

    pub fn set_translation_matrix(&mut self, value: Matrix4X4) {
        if self.translation_matrix != value {
            self.translation_matrix = value;
            self.on_transform_changed();
        }
    }

    /// The uniform scale the camera applies to the world.
    pub fn scale(&self) -> f64 {
        // The unit X as a VECTOR: as a position the translation leaks in and a moved camera reads
        // its distance to the origin times the scale (or 0 when world X = 1 sits at the view
        // pivot).
        let scaled_unit_vector = Vector3::UNIT_X.transform_vector(self.get_transform_4x4());
        scaled_unit_vector.length()
    }

    /// Scales about the view pivot (the world point at the view-space origin), so the eye moves
    /// along its line to that point. Does nothing while the current scale is not positive.
    pub fn set_scale(&mut self, value: f64) {
        if self.scale() > 0.0 {
            let required_change = value / self.scale();

            self.set_translation_matrix(
                self.translation_matrix * Matrix4X4::create_scale_uniform(required_change),
            );
            self.on_transform_changed();
        }
    }

    pub fn calculate_modelview_matrix(&mut self) {
        self.modelview_matrix = self.get_transform_4x4()
            * Matrix4X4::create_translation(0.0, 0.0, Self::CAMERA_Z_TRANSLATION_FUDGE);
        self.inverse_modelview_matrix = self.modelview_matrix.inverted();
    }

    pub fn eye_position(&self) -> Vector3 {
        Vector3::ZERO.transform(self.inverse_modelview_matrix)
    }

    pub fn set_eye_position(&mut self, value: Vector3) {
        self.translate(self.eye_position() - value);
    }

    /// Sets a typical perspective projection with an FOV of 45. `z_near`/`z_far` are positive
    /// positions along the forward axis (usually [`Self::DEFAULT_NEAR_Z`] and
    /// [`Self::DEFAULT_FAR_Z`]).
    pub fn calculate_perspective_matrix(
        &mut self,
        width: f64,
        height: f64,
        z_near: f64,
        z_far: f64,
    ) -> Result<(), MathError> {
        self.calculate_perspective_matrix_off_center(
            width,
            height,
            0.0,
            z_near,
            z_far,
            Self::DEFAULT_PERSPECTIVE_VFOV_DEGREES,
        )
    }

    /// The near and far planes made usable for a perspective projection.
    pub fn sanitise_perspective_near_far(near: f64, far: f64) -> (f64, f64) {
        let mut near = max_f64(near, Self::PERSPECTIVE_PROJECTION_MINIMUM_NEAR_Z);
        if near.is_nan() {
            near = Self::PERSPECTIVE_PROJECTION_MINIMUM_NEAR_Z;
        }

        let mut far = max_f64(
            far,
            near * Self::PERSPECTIVE_PROJECTION_MINIMUM_FAR_NEAR_RATIO,
        );
        if far.is_nan() || far.is_infinite() {
            far = near * Self::PERSPECTIVE_PROJECTION_MINIMUM_FAR_NEAR_RATIO;
        }

        if far <= near {
            // zNear is so large that the addition didn't make a difference.
            // Hope it's not infinity and do something multiplicative instead.
            if near >= 0.0 {
                far = near * 2.0;
            } else {
                far = near / 2.0;
            }
        }

        (near, far)
    }

    /// The visible height within a symmetric vertical FOV at a signed distance from the camera.
    pub fn calc_perspective_height(distance: f64, vfov_degrees: f64) -> f64 {
        distance * 2.0 * libm::tan(math_helper::degrees_to_radians(vfov_degrees) * 0.5)
    }

    /// The distance from the camera at which a symmetric vertical FOV shows `height`.
    pub fn calc_perspective_distance(height: f64, vfov_degrees: f64) -> f64 {
        height * 0.5 / libm::tan(math_helper::degrees_to_radians(vfov_degrees) * 0.5)
    }

    /// The symmetric vertical FOV in degrees that shows `height` at `distance`.
    pub fn calc_perspective_vfov_degrees_from_distance_and_height(
        distance: f64,
        height: f64,
    ) -> f64 {
        math_helper::radians_to_degrees(libm::atan(height * 0.5 / distance) * 2.0)
    }

    /// Sets a perspective projection with a center adjustment (`center_offset_x` moves the
    /// right edge) and FOV (usually near 0.1, far 100, 45 degrees).
    pub fn calculate_perspective_matrix_off_center(
        &mut self,
        width: f64,
        height: f64,
        center_offset_x: f64,
        z_near: f64,
        z_far: f64,
        vfov_degrees: f64,
    ) -> Result<(), MathError> {
        let width = max_f64(1.0, width);
        let height = max_f64(1.0, height);
        let (z_near, z_far) = Self::sanitise_perspective_near_far(z_near, z_far);

        let screen_dist = Self::calc_perspective_distance(height, vfov_degrees);
        let center = width / 2.0;
        let x_angle_l = libm::atan2(-center - center_offset_x / 2.0, screen_dist);
        let x_angle_r = libm::atan2(center - center_offset_x / 2.0, screen_dist);

        // calculate yMin and yMax at the near clip plane
        let y_max = Self::calc_perspective_height(z_near, vfov_degrees) * 0.5;
        let y_min = -y_max;
        let x_max = z_near * libm::tan(x_angle_r);
        let x_min = z_near * libm::tan(x_angle_l);

        let projection_matrix =
            Matrix4X4::create_perspective_off_center(x_min, x_max, y_min, y_max, z_near, z_far)?;

        self.projection_matrix = projection_matrix;
        self.inverse_projection_matrix = projection_matrix.inverted();
        self.width = width;
        self.height = height;
        self.v_fov_degrees = vfov_degrees;
        self.h_fov_degrees = math_helper::radians_to_degrees((x_angle_r - x_angle_l).abs());
        self.near_plane_height_in_viewspace = y_max * 2.0;
        self.near_z = z_near;
        self.far_z = z_far;
        self.is_orthographic = false;
        Ok(())
    }

    /// The near and far planes made usable for an orthographic projection.
    pub fn sanitise_orthographic_near_far(near: f64, far: f64) -> (f64, f64) {
        let mut far = max_f64(
            far,
            near + Self::ORTHOGRAPHIC_PROJECTION_MINIMUM_NEAR_FAR_GAP,
        );
        if far <= near {
            // zNear is so large that the addition didn't make a difference.
            // Hope it's not infinity and do something multiplicative instead.
            if near >= 0.0 {
                far = near * 2.0;
            } else {
                far = near / 2.0;
            }
        }

        (near, far)
    }

    /// Sets an orthographic projection with an explicit height in viewspace; `z_near`/`z_far`
    /// are signed positions along the forward axis (usually 0.1 and 100). The name's spelling
    /// (`orthogrphic`) is kept so its callers stay unchanged.
    pub fn calculate_orthographic_matrix_off_center_with_viewspace_height(
        &mut self,
        width: f64,
        height: f64,
        center_offset_x: f64,
        height_in_viewspace: f64,
        z_near: f64,
        z_far: f64,
    ) {
        let width = max_f64(1.0, width);
        let height = max_f64(1.0, height);
        let height_in_viewspace = max_f64(
            height_in_viewspace,
            Self::orthographic_projection_minimum_height(),
        );
        let (z_near, z_far) = Self::sanitise_orthographic_near_far(z_near, z_far);

        let effective_view_width = max_f64(1.0, width + center_offset_x);
        let screen_center_x = effective_view_width / 2.0;
        let x_max = height_in_viewspace / height * (width - screen_center_x);
        let x_min = height_in_viewspace / height * -screen_center_x;
        let y_max = height_in_viewspace / 2.0;
        let y_min = -y_max;

        let projection_matrix =
            Matrix4X4::create_orthographic_off_center(x_min, x_max, y_min, y_max, z_near, z_far);

        self.width = width;
        self.height = height;
        self.projection_matrix = projection_matrix;
        self.inverse_projection_matrix = projection_matrix.inverted();
        self.near_plane_height_in_viewspace = height_in_viewspace;
        self.near_z = z_near;
        self.far_z = z_far;
        self.is_orthographic = true;
    }

    /// `[0, 0]..[Width, Height]` (+Y is up) for a worldspace position.
    pub fn get_screen_position(&self, world_position: Vector3) -> Vector2 {
        let viewspace = self.worldspace_to_viewspace(world_position);
        let ndc = self.viewspace_to_ndc(viewspace);
        self.ndc_to_bottom_screenspace_2(ndc.xy())
    }

    /// NDC before the perspective divide (clip-space).
    pub fn world_to_screen_space(&self, world_position: Vector3) -> Vector3 {
        let view_position = world_position.transform(self.modelview_matrix);
        view_position.transform(self.projection_matrix)
    }

    /// A ray in worldspace along all points at a bottom-left-origin screen position. In
    /// perspective mode the origin is the eye; in orthographic mode it is on the near plane
    /// with infinite extent in both directions (`min_distance_to_consider` is -infinity).
    pub fn get_ray_for_local_bounds(&self, screen_position: Vector2) -> Ray {
        let near_ndc =
            self.bottom_screenspace_to_ndc_3(Vector3::from_vector2(screen_position, -1.0));
        let near_viewspace_position = self.ndc_to_viewspace(near_ndc);
        if self.is_orthographic {
            Ray::with_limits(
                near_viewspace_position.transform_position(self.inverse_modelview_matrix),
                (-Vector3::UNIT_Z)
                    .transform_vector(self.inverse_modelview_matrix)
                    .get_normal(),
                f64::NEG_INFINITY,
                f64::INFINITY,
                crate::IntersectionType::FrontFace,
            )
        } else {
            Ray::new(
                self.eye_position(),
                near_viewspace_position
                    .transform_vector(self.inverse_modelview_matrix)
                    .get_normal(),
            )
        }
    }

    pub fn get_transform_4x4(&self) -> Matrix4X4 {
        self.translation_matrix * self.rotation_matrix
    }

    pub fn get_viewspace_height_at_position(&self, viewspace_position: Vector3) -> f64 {
        if self.is_orthographic {
            self.near_plane_height_in_viewspace
        } else {
            self.near_plane_height_in_viewspace * viewspace_position.z / -self.near_z
        }
    }

    /// The ratio a caller multiplies a pixel count by to get a worldspace size at
    /// `world_position`. This is how every 3D handle, gizmo and pick tolerance stays the same
    /// size on screen at any zoom. `max_ratio` is an optional ceiling (+infinity for none).
    /// Always finite and positive: the absolute value is taken, it is floored at a minimum
    /// derived from the minimum allowed near plane, and a camera or position that produces no
    /// usable number falls back to [`Self::DEGENERATE_WORLD_UNITS_PER_PIXEL`].
    pub fn get_world_units_per_screen_pixel_at_position(
        &self,
        world_position: Vector3,
        max_ratio: f64,
    ) -> f64 {
        let viewspace = self.worldspace_to_viewspace(world_position);
        let mut viewspace_units_per_pixel =
            self.get_viewspace_height_at_position(viewspace) / self.height;
        let min_magnitude = self.get_viewspace_height_at_position(Vector3::new(
            0.0,
            0.0,
            -Self::PERSPECTIVE_PROJECTION_MINIMUM_NEAR_Z,
        )) / self.height;
        viewspace_units_per_pixel = max_f64(viewspace_units_per_pixel.abs(), min_magnitude);
        let worldspace_x_units_per_pixel = Vector3::new(viewspace_units_per_pixel, 0.0, 0.0)
            .transform_vector(self.inverse_modelview_matrix)
            .length();

        let ratio = min_f64(worldspace_x_units_per_pixel, max_ratio);

        // Callers size geometry with this and several divide by it, so a NaN or an infinity
        // here would spread through the scene rather than show up as one bad handle. A NaN loses
        // every comparison, so test for what is wanted rather than for what is wrong - and test
        // it last, after the caller's own ceiling has been applied, since a nonsense ceiling is
        // no better than a nonsense camera.
        if !ratio.is_finite() || ratio <= 0.0 {
            return Self::DEGENERATE_WORLD_UNITS_PER_PIXEL;
        }

        ratio
    }

    pub fn on_transform_changed(&mut self) {
        self.calculate_modelview_matrix();
    }

    pub fn reset(&mut self) {
        self.set_rotation_matrix(Matrix4X4::IDENTITY);
        self.set_translation_matrix(Matrix4X4::IDENTITY);
    }

    pub fn rotate(&mut self, rotation: Quaternion) {
        self.set_rotation_matrix(
            self.rotation_matrix * Matrix4X4::create_rotation_quaternion(rotation),
        );
    }

    pub fn rotate_around_position(&mut self, world_position: Vector3, rotation: Quaternion) {
        let new_rotation = self.rotation_matrix * Matrix4X4::create_rotation_quaternion(rotation);
        self.set_rotation_hold_position(world_position, new_rotation);
    }

    pub fn translate(&mut self, delta_position: Vector3) {
        self.set_translation_matrix(
            Matrix4X4::create_translation_vector(delta_position) * self.translation_matrix,
        );
    }

    /// `set_rotation_hold_position` with a quaternion.
    pub fn set_rotation_hold_position_quaternion(
        &mut self,
        world_position: Vector3,
        new_rotation: Quaternion,
    ) {
        self.set_rotation_hold_position(
            world_position,
            Matrix4X4::create_rotation_quaternion(new_rotation),
        );
    }

    /// Sets the rotation, then translates so `world_position` stays where it was on screen.
    pub fn set_rotation_hold_position(&mut self, world_position: Vector3, new_rotation: Matrix4X4) {
        // remember where we started on the screen
        let camera_space_start = world_position.transform(self.modelview_matrix);

        // do the rotation
        self.set_rotation_matrix(new_rotation);

        // move back to where we started
        let world_start_post_rotation = camera_space_start.transform(self.inverse_modelview_matrix);
        let delta = world_start_post_rotation - world_position;
        self.translate(delta);
    }

    /// `[0..0]..[Width, Height]` if on-screen (+Y is up), with the NDC depth as z.
    pub fn worldspace_to_bottom_screenspace(&self, worldspace_position: Vector3) -> Vector3 {
        let viewspace = self.worldspace_to_viewspace(worldspace_position);
        let ndc = self.viewspace_to_ndc(viewspace);
        self.ndc_to_bottom_screenspace_3(ndc)
    }

    /// Worldspace to `[l, b, -near]..[r, t, -far]` (if ortho).
    fn worldspace_to_viewspace(&self, worldspace_position: Vector3) -> Vector3 {
        worldspace_position.transform_position(self.modelview_matrix)
    }

    /// `[l, b, -near]..[r, t, -far]` (if ortho) to `[-1, -1, -1]..[1, 1, 1]`.
    fn viewspace_to_ndc(&self, viewspace_position: Vector3) -> Vector3 {
        let v = Vector4::from_vector3_w(viewspace_position, 1.0).transform(self.projection_matrix);
        v.xyz() / v.w
    }

    /// `[-1, -1, -1]..[1, 1, 1]` to `[l, b, -near]..[r, t, -far]` (if ortho).
    pub fn ndc_to_viewspace(&self, ndc_position: Vector3) -> Vector3 {
        let v =
            Vector4::from_vector3_w(ndc_position, 1.0).transform(self.inverse_projection_matrix);
        v.xyz() / v.w
    }

    /// `[-1, -1]..[1, 1]` (+Y is up) to `[0, 0]..[Width, Height]` (+Y is down). Unused; kept
    /// with its siblings so the set of conversions is complete.
    #[allow(dead_code)]
    fn ndc_to_top_screenspace(&self, ndc_position: Vector2) -> Vector2 {
        Vector2::multiply_components(
            Vector2::multiply_components(
                ndc_position - Vector2::new(-1.0, 1.0),
                self.viewport_size(),
            ),
            Vector2::new(0.5, -0.5),
        )
    }

    /// `[-1, -1]..[1, 1]` (+Y is up) to `[0, 0]..[Width, Height]` (+Y is up).
    fn ndc_to_bottom_screenspace_2(&self, ndc_position: Vector2) -> Vector2 {
        Vector2::multiply_components(
            Vector2::multiply_components(
                ndc_position - Vector2::new(-1.0, -1.0),
                self.viewport_size(),
            ),
            Vector2::new(0.5, 0.5),
        )
    }

    /// `[0, 0]..[Width, Height]` (+Y is up) to `[-1, -1]..[1, 1]` (+Y is up).
    fn bottom_screenspace_to_ndc_2(&self, screenspace_position: Vector2) -> Vector2 {
        Vector2::divide_components(
            Vector2::multiply_components(screenspace_position, Vector2::new(2.0, 2.0)),
            self.viewport_size(),
        ) + Vector2::new(-1.0, -1.0)
    }

    fn ndc_to_bottom_screenspace_3(&self, ndc_position: Vector3) -> Vector3 {
        Vector3::from_vector2(
            self.ndc_to_bottom_screenspace_2(ndc_position.xy()),
            ndc_position.z,
        )
    }

    fn bottom_screenspace_to_ndc_3(&self, screenspace_position: Vector3) -> Vector3 {
        Vector3::from_vector2(
            self.bottom_screenspace_to_ndc_2(screenspace_position.xy()),
            screenspace_position.z,
        )
    }
}
