use std::ops;
use std::vec::Vec;

use rand::{self, Rng};

const EPSILON: f64 = 1e-5;

pub struct Scene {
    pub objects: Vec<SceneObject>,
    pub lights: Vec<Light>,
}

pub struct Light {
    pub position: Vec3,
    pub color: Vec3,
    pub radius: f64,
}

#[derive(Clone, Copy, Debug)]
pub enum MaterialType {
    Plastic,
    Metal,   
}

#[derive(Clone, Copy, Debug)]
pub struct Material {
    pub color: Vec3,
    pub type_: MaterialType,
}

#[derive(Debug)]
pub struct SceneObject {
    pub solid: Solid,
    // TODO: this should be refactored into a material class
    pub material: Material,
}

pub struct HitResult {
    pub t: f64,
    pub normal: Vec3,
}

pub struct RaycastResult<'a> {
    pub hitted_object: &'a SceneObject,
    pub hit_point: Vec3,
    pub normal: Vec3,
}

#[derive(Debug)]
pub enum Solid {
    Sphere { center: Vec3, radius: f64 },
    Plane {normal: Vec3, distance: f64},
}

pub fn hit<'a>(scene: &'a Scene, ray: &'a Ray) -> Option<RaycastResult<'a>> {
    let mut closest_t = f64::INFINITY;
    let mut closest_object = None;
    let mut raycast_to_closest = None;
    for object in &scene.objects {
        if let Some(result) = collide(&object.solid, ray) {
            // avoid t too small (shadow acne)
            if result.t <= EPSILON {
                continue;
            }
            
            if result.t < closest_t {
                closest_t = result.t;
                raycast_to_closest = Some(result);
                closest_object = Some(object);
            }
        }
    }

    // if closest_object.is_some() {
    //     println!("ray {:?}  -> {:?}", ray.origin, ray.direction);
    //     println!("closest t {:?}:  {:?}", closest_t, ray.at(closest_t));
    // }

    closest_object.map(|object| RaycastResult {
        hitted_object: object, 
        hit_point: ray.at(closest_t),
        normal: raycast_to_closest.unwrap().normal,
    })
}

pub fn collide(solid: &Solid, ray: &Ray) -> Option<HitResult> {
    match solid {
        Solid::Sphere {center, radius} => {
            let oc = ray.origin - *center;
            let a = ray.direction.dot(&ray.direction);
            let b = 2.0 * ray.direction.dot(&oc);
            let c = oc.dot(&oc) - radius * radius;
            let discriminant = b * b - 4.0 * a * c;

            if discriminant < 0.0 {
                return None
            }

            // println!("ray.direction {:?}", ray.direction);
            // println!("oc {:?}", oc);
            // println!("a {}, b {}, c {}", a, b, c);
            // println!("discriminant {}", discriminant);

            let t = (-b - discriminant.sqrt()) / (2.0 * a);
            let normal = (ray.at(t) - *center).normalize();
            return Some(HitResult {t, normal});
        },
        Solid::Plane {normal, distance} => {
            let dv = normal.dot(&ray.direction);
            let center = *normal * *distance;
            if dv.abs() < EPSILON {
                return None;
            }
            let d2 = (center - ray.origin).dot(normal);
            let t = d2 / dv;
            if t < EPSILON {
                return None;
            }
            Some(HitResult { t, normal: *normal })
        }
    }
}

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

#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}


impl ops::Add<Vec3> for Vec3 {
    type Output = Vec3;
    fn add(self, rhs: Vec3) -> Self::Output {
        Vec3 {x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z}
    }
}

impl ops::AddAssign<Vec3> for Vec3 {
    fn add_assign(&mut self, rhs: Vec3) {
        self.x += rhs.x;   
        self.y += rhs.y;   
        self.z += rhs.z;   
    }
}

impl ops::Sub<Vec3> for Vec3 {
    type Output = Vec3;
    fn sub(self, rhs: Vec3) -> Self::Output {
        Vec3 {x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z}
    }
}

impl ops::Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, rhs: f64) -> Self::Output {
        Vec3 {x: self.x * rhs, y: self.y * rhs, z: self.z * rhs}
    }
}

impl ops::Mul<Vec3> for Vec3 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3 {x: self.x * rhs.x, y: self.y * rhs.y, z: self.z * rhs.z}
    }
}

impl ops::Div<f64> for Vec3 {
    type Output = Vec3;
    fn div(self, rhs: f64) -> Self::Output {
        Vec3 {x: self.x / rhs, y: self.y / rhs, z: self.z / rhs}
    }
}

impl Vec3 {
    pub fn zero() -> Vec3 {
        Vec3 {x: 0.0, y: 0.0, z: 0.0}
    }
    
    pub fn one() -> Vec3 {
        Vec3 {x: 1.0, y: 1.0, z: 1.0}
    }
    
    pub fn random() -> Vec3 {
        let dx = rand::thread_rng().gen_range(0.0..1.0);
        let dy = rand::thread_rng().gen_range(0.0..1.0);
        let dz = rand::thread_rng().gen_range(0.0..1.0);
        Vec3::new(dx, dy, dz)
    }

    pub fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 {x, y, z}
    }

    pub fn reflect(self: Self, axis: Vec3) -> Vec3 {
        // reflect the passed vector with respect at this vector used as axis
        self - axis * 2.0 * self.dot(&axis)
     }

    pub fn dot(self: &Self, other: &Vec3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn distance(self: &Self, other: Vec3) -> f64 {
        (*self - other).len()
    }

    pub fn len(self: &Self) -> f64 {
        let squared_len = self.dot(&self);
        squared_len.sqrt()
    }

    pub fn normalize(self: &Self) -> Vec3 {
        *self / self.len()
    }
}