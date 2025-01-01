use super::math::{Ray, Vec3};

// TODO: add field of view to the camera
pub struct Camera {
    forward: Vec3,
    up: Vec3,
    right: Vec3,
    position: Vec3,
    zoom: f64,
}

impl Camera {
    pub fn new(position: Vec3) -> Self {
        Self {
            up: Vec3::y_axis(),
            forward: Vec3::z_axis(),
            right: Vec3::x_axis(),
            position,
            zoom: 1.0,
        }
    }

    /// Creates a new camera positioned at zero looking toward the z axis.
    pub fn default() -> Self {
        Self::new(Vec3::zero())
    }

    /// Creates a camera that looks at a specific point from a specific position.
    /// The point is referred to be the center of the screen.
    pub fn look_at(position: Vec3, point: Vec3) -> Self {
        let world_up = Vec3::y_axis();
        let forward = (point - position).normalize();
        let right = world_up.cross(forward).normalize();
        // to get an orthonormal base, we should calculate the up vector with two perpendicular vectors
        let up = forward.cross(right).normalize();
        Self {
            forward,
            up,
            right,
            position,
            zoom: 1.0,
        }
    }

    /// Create a ray from the camera position to the relative uv coordinate on his screen.
    pub fn shoot_to(&self, u: f64, v: f64) -> Ray {
        let direction = self.forward * self.zoom + self.up * v + self.right * u;
        Ray {
            origin: self.position,
            direction: direction.normalize(),
        }
    }
}
