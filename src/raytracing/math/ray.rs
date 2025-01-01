use super::Vec3;

#[derive(Clone)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    pub fn at(self: &Self, t: f64) -> Vec3 {
        self.origin + self.direction * t
    }
}
