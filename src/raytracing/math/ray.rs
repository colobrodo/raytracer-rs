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

#[derive(Clone, Copy)]
pub struct HitResult {
    // TODO: is not better to store directly the intersection point?
    //       we should always recompute it from t at least in the end raytracing procedure
    pub t: f64,
    pub normal: Vec3,
}

pub trait RayIntersectable {
    fn intersect(&self, ray: &Ray) -> Option<f64>;
}
pub trait RayHittable {
    fn hit(&self, ray: &Ray) -> Option<HitResult>;
}
