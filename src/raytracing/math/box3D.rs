use super::{Ray, Vec3};

#[derive(Debug, Clone, Copy)]
pub struct Box3 {
    pub center: Vec3,
    pub half_extension: Vec3,
}

impl Box3 {
    pub fn new(center: Vec3, half_extension: Vec3) -> Box3 {
        Box3 {
            center,
            half_extension,
        }
    }

    pub fn from_single_point(point: Vec3) -> Box3 {
        Box3 {
            center: point,
            half_extension: Vec3::zero(),
        }
    }

    pub fn from_min_max(min: Vec3, max: Vec3) -> Box3 {
        Box3 {
            center: (min + max) * 0.5,
            half_extension: (max - min) * 0.5,
        }
    }

    pub fn include(&mut self, point: Vec3) {
        let dist = point - self.center;
        if dist.x.abs() > self.half_extension.x {
            self.half_extension.x = (dist.x.abs() + self.half_extension.x) / 2.0;
            self.center.x += (dist.x - self.half_extension.x * dist.x.signum()) / 2.0;
        }
        if dist.y.abs() > self.half_extension.y {
            self.half_extension.y = (dist.y.abs() + self.half_extension.y) / 2.0;
            self.center.y += (dist.y - self.half_extension.y * dist.y.signum()) / 2.0;
        }
        if dist.z.abs() > self.half_extension.z {
            self.half_extension.z = (dist.z.abs() + self.half_extension.z) / 2.0;
            self.center.z += (dist.z - self.half_extension.z * dist.z.signum()) / 2.0;
        }
    }

    #[inline(always)]
    pub fn contains(&self, point: Vec3) -> bool {
        let dist = point - self.center;
        return (dist.x >= -self.half_extension.x && dist.x <= self.half_extension.x)
            && (dist.y >= -self.half_extension.y && dist.y <= self.half_extension.y)
            && (dist.z >= -self.half_extension.z && dist.z <= self.half_extension.z);
    }

    pub fn min(self: &Self) -> Vec3 {
        self.center - self.half_extension
    }

    pub fn max(self: &Self) -> Vec3 {
        self.center + self.half_extension
    }

    pub fn intersect_ray(&self, ray: &Ray) -> Option<f64> {
        let dirfrac = Vec3::new(
            1.0 / ray.direction.x,
            1.0 / ray.direction.y,
            1.0 / ray.direction.z,
        );
        let relative_min_box = self.min() - ray.origin;
        let relative_max_box = self.max() - ray.origin;
        let t1 = relative_min_box.x * dirfrac.x;
        let t2 = relative_max_box.x * dirfrac.x;
        let t3 = relative_min_box.y * dirfrac.y;
        let t4 = relative_max_box.y * dirfrac.y;
        let t5 = relative_min_box.z * dirfrac.z;
        let t6 = relative_max_box.z * dirfrac.z;

        let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
        let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

        // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
        if tmax < 0.0 {
            return None;
        }
        // if tmin > tmax, ray doesn't intersect AABB
        if tmin > tmax {
            return None;
        }
        return Some(tmin);
    }
}
