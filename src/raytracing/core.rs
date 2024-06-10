use std::error::Error;
use std::fs::File;
use std::{io::BufReader, ops};
use std::vec::Vec;

use obj::{Obj, load_obj};

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

#[derive(Clone, Copy)]
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
struct Box3 {
    center: Vec3,
    half_extension: Vec3,
}

impl Box3 {
    pub fn min(self: &Self) -> Vec3 {
        self.center - self.half_extension
    }

    pub fn max(self: &Self) -> Vec3 {
        self.center + self.half_extension
    }

    fn collide(self: &Self, ray: &Ray) -> bool {
        let mut dirfrac = Vec3::zero();
        dirfrac.x = 1.0 / ray.direction.x;
        dirfrac.y = 1.0 / ray.direction.y;
        dirfrac.z = 1.0 / ray.direction.z;
        // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
        // r.org is origin of ray
        let t1 = (self.min().x - ray.origin.x) * dirfrac.x;
        let t2 = (self.max().x - ray.origin.x) * dirfrac.x;
        let t3 = (self.min().y - ray.origin.y) * dirfrac.y;
        let t4 = (self.max().y - ray.origin.y) * dirfrac.y;
        let t5 = (self.min().z - ray.origin.z) * dirfrac.z;
        let t6 = (self.max().z - ray.origin.z) * dirfrac.z;
    
        let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
        let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));
    
        // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
        if tmax < 0.0 {
            return false;
        }
        // if tmin > tmax, ray doesn't intersect AABB
        if tmin > tmax {
            return false;
        }
        return true;
    }
}

#[derive(Debug)]
pub struct Mat4 {
    value: [f64; 16],
}

impl Mat4 {
    pub fn identity() -> Mat4 {
        Mat4 {
            value: [1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 
                    0.0, 0.0, 1.0, 0.0, 
                    0.0, 0.0, 0.0, 1.0]
        }
    }
    
    pub fn scale(factor: f64) -> Mat4 {
        Mat4 {
            value: [factor, 0.0, 0.0, 0.0,
                    0.0, factor, 0.0, 0.0, 
                    0.0, 0.0, factor, 0.0, 
                    0.0, 0.0, 0.0,    1.0]
        }
    }
    
    pub fn translate(offset: Vec3) -> Mat4 {
        Mat4 {
            value: [1.0, 0.0, 0.0, offset.x,
            0.0, 1.0, 0.0, offset.y, 
            0.0, 0.0, 1.0, offset.z, 
            0.0, 0.0, 0.0, 1.0]
        }
    }
            
    pub fn rotate(axis: Vec3, angle: f64) -> Mat4 {
        // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
        let u = axis.normalize();
        let cos_t = angle.cos();
        let sin_t = angle.sin();
        Mat4 {
            value: [cos_t + (u.x * u.x) * (1.0 - cos_t), u.x * u.y * (1.0 - cos_t) - u.z * sin_t, u.x * u.z * (1.0 - cos_t) - u.y * sin_t, 0.0,
                    u.x * u.y * (1.0 - cos_t) - u.x * sin_t, cos_t + (u.y * u.y) * (1.0 - cos_t), u.z * u.y * (1.0 - cos_t) - u.x * sin_t, 0.0, 
                    u.x * u.z * (1.0 - cos_t) - u.y * sin_t, u.z * u.y * (1.0 - cos_t) + u.x * sin_t, cos_t + u.z * u.z * (1.0 - cos_t), 0.0,
                    0.0, 0.0, 0.0, 1.0]
            }
        
        }
        
    pub fn then(&self, other: &Mat4) -> Mat4 {
        // other * self 
        let _a11 = other.value[0] * self.value[0] + other.value[1] * self.value[4] + other.value[2] * self.value[8] + other.value[3] * self.value[12];
        let _a21 = other.value[4] * self.value[0] + other.value[5] * self.value[4] + other.value[6] * self.value[8] + other.value[7] * self.value[12];
        let _a31 = other.value[8] * self.value[0] + other.value[9] * self.value[4] + other.value[10] * self.value[8] + other.value[11] * self.value[12];
        let _a41 = other.value[12] * self.value[0] + other.value[13] * self.value[4] + other.value[14] * self.value[8] + other.value[15] * self.value[12];
        
        let _a12 = other.value[0] * self.value[1] + other.value[1] * self.value[5] + other.value[2] * self.value[9] + other.value[3] * self.value[13];
        let _a22 = other.value[4] * self.value[1] + other.value[5] * self.value[5] + other.value[6] * self.value[9] + other.value[7] * self.value[13];
        let _a32 = other.value[8] * self.value[1] + other.value[9] * self.value[5] + other.value[10] * self.value[9] + other.value[11] * self.value[13];
        let _a42 = other.value[12] * self.value[1] + other.value[13] * self.value[5] + other.value[14] * self.value[9] + other.value[15] * self.value[13];
        
        let _a13 = other.value[0] * self.value[2] + other.value[1] * self.value[6] + other.value[2] * self.value[10] + other.value[3] * self.value[14];
        let _a23 = other.value[4] * self.value[2] + other.value[5] * self.value[6] + other.value[6] * self.value[10] + other.value[7] * self.value[14];
        let _a33 = other.value[8] * self.value[2] + other.value[9] * self.value[6] + other.value[10] * self.value[10] + other.value[11] * self.value[14];
        let _a43 = other.value[12] * self.value[2] + other.value[13] * self.value[6] + other.value[14] * self.value[10] + other.value[15] * self.value[14];
        
        let _a14 = other.value[0] * self.value[3] + other.value[1] * self.value[7] + other.value[2] * self.value[11] + other.value[3] * self.value[15];
        let _a24 = other.value[4] * self.value[3] + other.value[5] * self.value[7] + other.value[6] * self.value[11] + other.value[7] * self.value[15];
        let _a34 = other.value[8] * self.value[3] + other.value[9] * self.value[7] + other.value[10] * self.value[11] + other.value[11] * self.value[15];
        let _a44 = other.value[12] * self.value[3] + other.value[13] * self.value[7] + other.value[14] * self.value[11] + other.value[15] * self.value[15];

        Mat4 {
            value: [
                _a11, _a12, _a13, _a14,
                _a21, _a22, _a23, _a24,
                _a31, _a32, _a33, _a34,
                _a41, _a42, _a43, _a44,
            ]
        }
    }

    fn apply(&self, v: Vec3) -> Vec3 {
        let x = self.value[0] * v.x + self.value[1] * v.y + self.value[2] * v.z + self.value[3];
        let y = self.value[4] * v.x + self.value[5] * v.y + self.value[6] * v.z + self.value[7];
        let z = self.value[8] * v.x + self.value[9] * v.y + self.value[10] * v.z + self.value[11];
        let w = self.value[12] * v.x + self.value[13] * v.y + self.value[14] * v.z + self.value[15];
        Vec3::new(x / w, y / w, z / w)
    }

}

#[derive(Debug)]
struct Model {
    obj: Obj, 
    trasform: Mat4, 
    bounding_box: Box3
}

#[derive(Debug)]
pub enum Solid {
    Sphere { center: Vec3, radius: f64 },
    Plane {normal: Vec3, distance: f64},
    Model { model: Model }
}

impl Model {
    fn iter_triangles(&self) -> impl Iterator<Item = (Vec3, Vec3, Vec3)> + '_ {
        (0..self.obj.indices.len() / 3).map(|i| {
            let v0: Vec3 = self.obj.vertices[self.obj.indices[i * 3 + 0] as usize].position.into();
            let v0 = self.trasform.apply(v0);
            let v1: Vec3 = self.obj.vertices[self.obj.indices[i * 3 + 1] as usize].position.into();
            let v1 = self.trasform.apply(v1);
            let v2: Vec3 = self.obj.vertices[self.obj.indices[i * 3 + 2] as usize].position.into();
            let v2 = self.trasform.apply(v2);
            (v0, v1, v2)
        }).into_iter()
    }
}

fn calculate_bounding_box(obj: &Obj, trasform: &Mat4) -> Box3 {
    let mut max = Vec3::new(-f64::INFINITY, -f64::INFINITY, -f64::INFINITY);
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    for vertex in &obj.vertices {
        let vertex: Vec3 = vertex.position.into();
        let vertex = trasform.apply(vertex);
        if max.x < vertex.x as f64 {
            max.x = vertex.x as f64;
        }
        if max.y < vertex.y as f64 {
            max.y = vertex.y as f64;
        }
        if max.z < vertex.z as f64 {
            max.z = vertex.z as f64;
        }

        if min.x > vertex.x as f64 {
            min.x = vertex.x as f64;
        }
        if min.y > vertex.y as f64 {
            min.y = vertex.y as f64;
        }
        if min.z > vertex.z as f64 {
            min.z = vertex.z as f64;
        }
    }
    Box3 {
        center: (min + max) * 0.5,
        half_extension: (max - min) * 0.5,
    }
}

impl Solid {
    pub fn load_model(filename: &str, trasform: Mat4) -> Result<Solid, Box<dyn Error>> {
        let input = BufReader::new(File::open(filename)?);
        let obj = load_obj(input)?;
        let bounding_box = calculate_bounding_box(&obj, &trasform); 
        // DEBUG: remove
        println!("loaded {}, with bounding box {:?}", filename, bounding_box);
        Ok(Solid::Model {
            model: Model {
                obj,
                trasform,
                bounding_box,
            }
        })
    }
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
        Solid::Model { model } => {
            if !model.bounding_box.collide(ray) {
                return None;
            }

            let mut hit: Option<HitResult> = None;
            for (v0, v1, v2) in model.iter_triangles() {
                let v0v1 = v1 - v0;
                let v0v2 = v2 - v0;
                let pvec = ray.direction.cross(&v0v2);
                let det = v0v1.dot(&pvec);
                // ray and triangle are parallel if det is close to 0
                if det.abs() < EPSILON {
                    continue;
                }
                let inv_det = 1.0 / det;
                let tvec = ray.direction - v0;
                let u = tvec.dot(&pvec) * inv_det;
                if u < 0.0 || u > 1.0 {
                    continue;                    
                }
                let qvec = tvec.cross(&v0v1);
                let v = ray.direction.dot(&qvec) * inv_det;
                if v < 0.0 || u + v > 1.0 {
                    continue;
                }
    
                let t = v0v2.dot(&qvec) * inv_det;
                if t < 0.0 {
                    continue;
                }
    
                // for first iteration
                if hit.is_none() || t < hit.unwrap().t {
                    hit = Some(HitResult {
                        normal: v0v1.cross(&v0v2).normalize(),
                        t: v0v2.dot(&qvec) * inv_det,
                    });
                }
            }
            hit
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

impl From<[f64; 3]> for Vec3  {
    fn from(value: [f64; 3]) -> Self {
        Vec3::new(value[0], value[1], value[2])
    }
}

impl From<[f32; 3]> for Vec3  {
    fn from(value: [f32; 3]) -> Self {
        Vec3::new(value[0] as f64, value[1] as f64, value[2] as f64)
    }
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

    pub fn x_axis() -> Vec3 {
        Vec3 {x: 1.0, y: 0.0, z: 0.0}
    }

    pub fn y_axis() -> Vec3 {
        Vec3 {x: 0.0, y: 1.0, z: 0.0}
    }

    pub fn z_axis() -> Vec3 {
        Vec3 {x: 0.0, y: 0.0, z: 1.0}
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

    pub fn cross(self: &Self, other: &Vec3) -> Vec3 {
        Vec3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
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