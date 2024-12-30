use std::error::Error;
use std::fs::File;
use std::vec::Vec;
use std::{io::BufReader, ops};

use obj::{load_obj, Obj};

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
    // TODO: is not better to store directly the intersection point?
    //       we should always recompute it from t at least in the end raytracing procedure
    pub t: f64,
    pub normal: Vec3,
}

pub struct RaycastResult<'a> {
    pub hitted_object: &'a SceneObject,
    pub hit_point: Vec3,
    pub normal: Vec3,
}

#[derive(Debug, Clone, Copy)]
struct Box3 {
    center: Vec3,
    half_extension: Vec3,
}

impl Box3 {
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

    pub fn contains(&mut self, point: Vec3) -> bool {
        let dist = point - self.center;
        return dist.x.abs() <= self.half_extension.x
            && dist.y.abs() <= self.half_extension.y
            && dist.z.abs() <= self.half_extension.z;
    }

    pub fn min(self: &Self) -> Vec3 {
        self.center - self.half_extension
    }

    pub fn max(self: &Self) -> Vec3 {
        self.center + self.half_extension
    }

    fn collide(self: &Self, ray: &Ray) -> bool {
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
    bounding_box: Box3,
}

#[derive(Debug)]
struct ModelGrid {
    bounding_box: Box3,
    cells_per_side: u32,
    offset_array: Vec<usize>,
    triangle_indices: Vec<usize>,
}

fn triangle_ray_intersection(triangle: (Vec3, Vec3, Vec3), ray: &Ray) -> Option<HitResult> {
    // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    let (v0, v1, v2) = triangle;
    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;
    let ray_cross_e2 = ray.direction.cross(v0v2);
    let determinant = v0v1.dot(ray_cross_e2);
    // ray and triangle are parallel if det is close to 0
    if determinant.abs() < EPSILON {
        return None;
    }
    let inverse_determinant = 1.0 / determinant;
    let tvec = ray.origin - v0;
    let u = tvec.dot(ray_cross_e2) * inverse_determinant;
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let qvec = tvec.cross(v0v1);
    let v = ray.direction.dot(qvec) * inverse_determinant;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    // At this stage we can compute t to find out where the intersection point is on the line
    let t = v0v2.dot(qvec) * inverse_determinant;
    if t < 0.0 {
        return None;
    }

    Some(HitResult {
        normal: v0v1.cross(v0v2).normalize(),
        t,
    })
}

fn ray_intersect(model: &Model, grid: &ModelGrid, ray: &Ray) -> Option<HitResult> {
    let mut closest_hit: Option<HitResult> = None;
    for ((ix, iy, iz), bbox) in grid.enum_boxes() {
        if !bbox.collide(ray) {
            continue;
        }

        for triangle_i in grid.triangles(ix, iy, iz) {
            let triangle = model.get_triangle(*triangle_i);
            if let Some(hit) = triangle_ray_intersection(triangle, ray) {
                if closest_hit.is_none() || hit.t < closest_hit.unwrap().t {
                    closest_hit = Some(hit);
                }
            }
        }
    }
    closest_hit
}

impl ModelGrid {
    fn new(model: &Model, cells_per_side: usize) -> ModelGrid {
        // foreach box create a vec
        let mut cells_triangles: Vec<Vec<usize>> =
            vec![Vec::new(); cells_per_side * cells_per_side * cells_per_side];
        let cell_size =
            (model.bounding_box.max() - model.bounding_box.min()) / cells_per_side as f64;
        for (i, (v0, v1, v2)) in model.iter_triangles().enumerate() {
            // triangle bounding box
            let mut bbox = Box3 {
                center: v0,
                half_extension: Vec3::zero(),
            };
            bbox.include(v1);
            bbox.include(v2);
            // check each corner of the bounding box which cell intersect
            // get the buffer index of the minimim corner of the bounding box
            let min_point = (bbox.min() - model.bounding_box.min()) / cell_size;
            let min_ix = (min_point.x as usize).min(cells_per_side - 1);
            let min_iy = (min_point.y as usize).min(cells_per_side - 1);
            let min_iz = (min_point.z as usize).min(cells_per_side - 1);
            // get the buffer index of the maximim corner of the bounding box
            let max_point = (bbox.max() - model.bounding_box.min()) / cell_size;
            let max_ix = (max_point.x as usize).min(cells_per_side - 1);
            let max_iy = (max_point.y as usize).min(cells_per_side - 1);
            let max_iz = (max_point.z as usize).min(cells_per_side - 1);
            for ix in min_ix..=max_ix {
                for iy in min_iy..=max_iy {
                    for iz in min_iz..=max_iz {
                        let buffer_index =
                            iz * cells_per_side * cells_per_side + iy * cells_per_side + ix;
                        // adds to that vec the triangle index i
                        if let Some(index) = cells_triangles[buffer_index].last() {
                            if *index == i {
                                continue;
                            }
                        }
                        cells_triangles[buffer_index].push(i);
                    }
                }
            }
        }
        // create a single offset array with the dimension of the respective vector
        let mut offsets = vec![0; cells_per_side * cells_per_side * cells_per_side];
        let mut triangle_indices = Vec::new();
        let mut offset = 0;
        for (i, triangles) in cells_triangles.into_iter().enumerate() {
            offset += triangles.len();
            offsets[i] = offset;
            // add all the triangles id in the triangles indices array
            triangle_indices.extend(triangles);
        }

        ModelGrid {
            bounding_box: model.bounding_box,
            cells_per_side: cells_per_side as u32,
            offset_array: offsets,
            triangle_indices,
        }
    }

    fn triangles(&self, ix: u32, iy: u32, iz: u32) -> impl Iterator<Item = &usize> {
        let index = (iz * self.cells_per_side * self.cells_per_side + iy * self.cells_per_side + ix)
            as usize;
        let start = if index > 0 {
            self.offset_array[index - 1]
        } else {
            0
        };
        let end = self.offset_array[index];
        self.triangle_indices[start..end].into_iter()
    }

    fn cell_box(&self, ix: u32, iy: u32, iz: u32) -> Box3 {
        let half_cell_size = self.bounding_box.half_extension / self.cells_per_side as f64;
        Box3 {
            half_extension: half_cell_size,
            center: self.bounding_box.min()
                + half_cell_size
                    * 2.0
                    * Vec3::new(ix as f64 + 0.5, iy as f64 + 0.5, iz as f64 + 0.5),
        }
    }

    fn boxes_indices(&self) -> impl Iterator<Item = (u32, u32, u32)> + '_ {
        (0..self.cells_per_side)
            .flat_map(|i| (0..self.cells_per_side).map(move |j| (i, j)))
            .flat_map(|(i, j)| (0..self.cells_per_side).map(move |k| (i, j, k)))
            .into_iter()
    }

    fn enum_boxes(&self) -> impl Iterator<Item = ((u32, u32, u32), Box3)> + '_ {
        self.boxes_indices()
            .map(|(ix, iy, iz)| ((ix, iy, iz), self.cell_box(ix, iy, iz)))
            .into_iter()
    }
}

#[derive(Debug)]
pub enum Solid {
    Sphere { center: Vec3, radius: f64 },
    Plane { normal: Vec3, distance: f64 },
    Model { model: Model, grid: ModelGrid },
}

impl Model {
    fn get_triangle(&self, i: usize) -> (Vec3, Vec3, Vec3) {
        let v0: Vec3 = self.obj.vertices[self.obj.indices[i * 3 + 0] as usize]
            .position
            .into();
        let v0 = self.trasform.apply(v0);
        let v1: Vec3 = self.obj.vertices[self.obj.indices[i * 3 + 1] as usize]
            .position
            .into();
        let v1 = self.trasform.apply(v1);
        let v2: Vec3 = self.obj.vertices[self.obj.indices[i * 3 + 2] as usize]
            .position
            .into();
        let v2 = self.trasform.apply(v2);
        (v0, v1, v2)
    }

    fn iter_triangles(&self) -> impl Iterator<Item = (Vec3, Vec3, Vec3)> + '_ {
        (0..self.obj.indices.len() / 3)
            .map(|i| self.get_triangle(i))
            .into_iter()
    }
}

fn calculate_bounding_box(obj: &Obj, trasform: &Mat4) -> Box3 {
    let mut max = Vec3::new(-f64::INFINITY, -f64::INFINITY, -f64::INFINITY);
    let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    for vertex in &obj.vertices {
        let vertex: Vec3 = vertex.position.into();
        let vertex = trasform.apply(vertex);
        if max.x < vertex.x {
            max.x = vertex.x;
        }
        if max.y < vertex.y {
            max.y = vertex.y;
        }
        if max.z < vertex.z {
            max.z = vertex.z;
        }

        if min.x > vertex.x {
            min.x = vertex.x;
        }
        if min.y > vertex.y {
            min.y = vertex.y;
        }
        if min.z > vertex.z {
            min.z = vertex.z;
        }
    }
    Box3::from_min_max(min, max)
}

impl Solid {
    pub fn load_model(filename: &str, trasform: Mat4) -> Result<Solid, Box<dyn Error>> {
        let input = BufReader::new(File::open(filename)?);
        let obj = load_obj(input)?;
        let bounding_box = calculate_bounding_box(&obj, &trasform);
        // DEBUG: remove commented println!
        // println!("loaded {}, with bounding box {:?}", filename, bounding_box);
        let model = Model {
            obj,
            trasform,
            bounding_box,
        };
        let grid = ModelGrid::new(&model, 4);
        // println!("Created grid with this offsets: {:?}", grid.offset_array);
        Ok(Solid::Model { grid, model })
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
        Solid::Sphere { center, radius } => {
            let oc = ray.origin - *center;
            let a = ray.direction.dot(ray.direction);
            let b = 2.0 * ray.direction.dot(oc);
            let c = oc.dot(oc) - radius * radius;
            let discriminant = b * b - 4.0 * a * c;

            if discriminant < 0.0 {
                return None;
            }

            let t = (-b - discriminant.sqrt()) / (2.0 * a);
            let normal = (ray.at(t) - *center).normalize();
            return Some(HitResult { t, normal });
        }
        Solid::Plane { normal, distance } => {
            let dv = normal.dot(ray.direction);
            let center = *normal * *distance;
            if dv.abs() < EPSILON {
                return None;
            }
            let d2 = (center - ray.origin).dot(*normal);
            let t = d2 / dv;
            if t < EPSILON {
                return None;
            }
            Some(HitResult { t, normal: *normal })
        }
        Solid::Model { model, grid } => {
            if !model.bounding_box.collide(ray) {
                return None;
            }
            ray_intersect(model, grid, ray)
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

impl From<[f64; 3]> for Vec3 {
    fn from(value: [f64; 3]) -> Self {
        Vec3::new(value[0], value[1], value[2])
    }
}

impl From<[f32; 3]> for Vec3 {
    fn from(value: [f32; 3]) -> Self {
        Vec3::new(value[0] as f64, value[1] as f64, value[2] as f64)
    }
}

impl ops::Add<Vec3> for Vec3 {
    type Output = Vec3;
    fn add(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
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
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl ops::Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, rhs: f64) -> Self::Output {
        Vec3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl ops::Mul<Vec3> for Vec3 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

impl ops::Div<f64> for Vec3 {
    type Output = Vec3;
    fn div(self, rhs: f64) -> Self::Output {
        Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl ops::Div<Vec3> for Vec3 {
    type Output = Vec3;
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
    pub fn len(self) -> f64 {
        let squared_len = self.dot(self);
        squared_len.sqrt()
    }

    #[inline(always)]
    pub fn normalize(self: &Self) -> Vec3 {
        *self / self.len()
    }
}
