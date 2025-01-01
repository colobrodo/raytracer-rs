use super::Vec3;

#[derive(Debug)]
pub struct Mat4 {
    value: [f64; 16],
}

impl Mat4 {
    pub fn identity() -> Mat4 {
        Mat4 {
            value: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    pub fn scale(factor: f64) -> Mat4 {
        Mat4 {
            value: [
                factor, 0.0, 0.0, 0.0, 0.0, factor, 0.0, 0.0, 0.0, 0.0, factor, 0.0, 0.0, 0.0, 0.0,
                1.0,
            ],
        }
    }

    pub fn translate(offset: Vec3) -> Mat4 {
        Mat4 {
            value: [
                1.0, 0.0, 0.0, offset.x, 0.0, 1.0, 0.0, offset.y, 0.0, 0.0, 1.0, offset.z, 0.0,
                0.0, 0.0, 1.0,
            ],
        }
    }

    pub fn rotate(axis: Vec3, angle: f64) -> Mat4 {
        // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
        let u = axis.normalize();
        let cos_t = angle.cos();
        let sin_t = angle.sin();
        Mat4 {
            value: [
                cos_t + (u.x * u.x) * (1.0 - cos_t),
                u.x * u.y * (1.0 - cos_t) - u.z * sin_t,
                u.x * u.z * (1.0 - cos_t) - u.y * sin_t,
                0.0,
                u.x * u.y * (1.0 - cos_t) - u.x * sin_t,
                cos_t + (u.y * u.y) * (1.0 - cos_t),
                u.z * u.y * (1.0 - cos_t) - u.x * sin_t,
                0.0,
                u.x * u.z * (1.0 - cos_t) - u.y * sin_t,
                u.z * u.y * (1.0 - cos_t) + u.x * sin_t,
                cos_t + u.z * u.z * (1.0 - cos_t),
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        }
    }

    pub fn then(&self, other: &Mat4) -> Mat4 {
        // other * self
        let _a11 = other.value[0] * self.value[0]
            + other.value[1] * self.value[4]
            + other.value[2] * self.value[8]
            + other.value[3] * self.value[12];
        let _a21 = other.value[4] * self.value[0]
            + other.value[5] * self.value[4]
            + other.value[6] * self.value[8]
            + other.value[7] * self.value[12];
        let _a31 = other.value[8] * self.value[0]
            + other.value[9] * self.value[4]
            + other.value[10] * self.value[8]
            + other.value[11] * self.value[12];
        let _a41 = other.value[12] * self.value[0]
            + other.value[13] * self.value[4]
            + other.value[14] * self.value[8]
            + other.value[15] * self.value[12];

        let _a12 = other.value[0] * self.value[1]
            + other.value[1] * self.value[5]
            + other.value[2] * self.value[9]
            + other.value[3] * self.value[13];
        let _a22 = other.value[4] * self.value[1]
            + other.value[5] * self.value[5]
            + other.value[6] * self.value[9]
            + other.value[7] * self.value[13];
        let _a32 = other.value[8] * self.value[1]
            + other.value[9] * self.value[5]
            + other.value[10] * self.value[9]
            + other.value[11] * self.value[13];
        let _a42 = other.value[12] * self.value[1]
            + other.value[13] * self.value[5]
            + other.value[14] * self.value[9]
            + other.value[15] * self.value[13];

        let _a13 = other.value[0] * self.value[2]
            + other.value[1] * self.value[6]
            + other.value[2] * self.value[10]
            + other.value[3] * self.value[14];
        let _a23 = other.value[4] * self.value[2]
            + other.value[5] * self.value[6]
            + other.value[6] * self.value[10]
            + other.value[7] * self.value[14];
        let _a33 = other.value[8] * self.value[2]
            + other.value[9] * self.value[6]
            + other.value[10] * self.value[10]
            + other.value[11] * self.value[14];
        let _a43 = other.value[12] * self.value[2]
            + other.value[13] * self.value[6]
            + other.value[14] * self.value[10]
            + other.value[15] * self.value[14];

        let _a14 = other.value[0] * self.value[3]
            + other.value[1] * self.value[7]
            + other.value[2] * self.value[11]
            + other.value[3] * self.value[15];
        let _a24 = other.value[4] * self.value[3]
            + other.value[5] * self.value[7]
            + other.value[6] * self.value[11]
            + other.value[7] * self.value[15];
        let _a34 = other.value[8] * self.value[3]
            + other.value[9] * self.value[7]
            + other.value[10] * self.value[11]
            + other.value[11] * self.value[15];
        let _a44 = other.value[12] * self.value[3]
            + other.value[13] * self.value[7]
            + other.value[14] * self.value[11]
            + other.value[15] * self.value[15];

        Mat4 {
            value: [
                _a11, _a12, _a13, _a14, _a21, _a22, _a23, _a24, _a31, _a32, _a33, _a34, _a41, _a42,
                _a43, _a44,
            ],
        }
    }

    pub fn apply(&self, v: Vec3) -> Vec3 {
        let x = self.value[0] * v.x + self.value[1] * v.y + self.value[2] * v.z + self.value[3];
        let y = self.value[4] * v.x + self.value[5] * v.y + self.value[6] * v.z + self.value[7];
        let z = self.value[8] * v.x + self.value[9] * v.y + self.value[10] * v.z + self.value[11];
        let w = self.value[12] * v.x + self.value[13] * v.y + self.value[14] * v.z + self.value[15];
        Vec3::new(x / w, y / w, z / w)
    }
}
