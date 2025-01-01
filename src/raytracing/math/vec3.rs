use rand::{self, Rng};
use std::ops;

#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<[f64; 3]> for Vec3 {
    #[inline(always)]
    fn from(value: [f64; 3]) -> Self {
        Vec3::new(value[0], value[1], value[2])
    }
}

impl From<[f32; 3]> for Vec3 {
    #[inline(always)]
    fn from(value: [f32; 3]) -> Self {
        Vec3::new(value[0] as f64, value[1] as f64, value[2] as f64)
    }
}

impl ops::Add<Vec3> for Vec3 {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl ops::AddAssign<Vec3> for Vec3 {
    #[inline(always)]
    fn add_assign(&mut self, rhs: Vec3) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::Sub<Vec3> for Vec3 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl ops::Mul<f64> for Vec3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: f64) -> Self::Output {
        Vec3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl ops::Mul<Vec3> for Vec3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

impl ops::Div<f64> for Vec3 {
    type Output = Self;

    #[inline(always)]
    fn div(self, rhs: f64) -> Self::Output {
        Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl ops::Div<Vec3> for Vec3 {
    type Output = Self;

    #[inline(always)]
    fn div(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x / rhs.x,
            y: self.y / rhs.y,
            z: self.z / rhs.z,
        }
    }
}

impl Vec3 {
    #[inline(always)]
    pub fn zero() -> Vec3 {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    #[inline(always)]
    pub fn one() -> Vec3 {
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        }
    }

    #[inline(always)]
    pub fn x_axis() -> Vec3 {
        Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }
    }

    #[inline(always)]
    pub fn y_axis() -> Vec3 {
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        }
    }

    #[inline(always)]
    pub fn z_axis() -> Vec3 {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        }
    }

    #[inline(always)]
    pub fn random() -> Vec3 {
        let dx = rand::thread_rng().gen_range(0.0..1.0);
        let dy = rand::thread_rng().gen_range(0.0..1.0);
        let dz = rand::thread_rng().gen_range(0.0..1.0);
        Vec3::new(dx, dy, dz)
    }

    #[inline(always)]
    pub fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[inline(always)]
    pub fn reflect(self: Self, axis: Vec3) -> Vec3 {
        // reflect the passed vector with respect at this vector used as axis
        self - axis * 2.0 * self.dot(axis)
    }

    #[inline(always)]
    pub fn dot(self: &Self, other: Vec3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline(always)]
    pub fn cross(self, other: Vec3) -> Vec3 {
        Vec3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    #[inline(always)]
    pub fn distance(self, other: Vec3) -> f64 {
        (self - other).len()
    }

    #[inline(always)]
    pub fn squared_len(self) -> f64 {
        self.dot(self)
    }

    #[inline(always)]
    pub fn len(self) -> f64 {
        let squared_len = self.squared_len();
        squared_len.sqrt()
    }

    #[inline(always)]
    pub fn normalize(self: &Self) -> Vec3 {
        *self / self.len()
    }
}
