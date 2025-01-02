use super::math::{Mat4, Ray, Vec3};
use std::fs::File;
use std::vec::Vec;
use std::{error::Error, io::BufReader};

use obj::{load_obj, Obj};

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

    fn intersect_ray(&self, ray: &Ray) -> Option<f64> {
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
    let bbox_intersection_t = model.bounding_box.intersect_ray(ray)?;
    let intersection_point = ray.at(bbox_intersection_t);
    // we use closest cell index to avoid the case where the intersection point is on the border of the grid
    // we have implicitly checked that the point is inside the grid by simply calculating it from the ray intersection
    let (ix, iy, iz) = grid.closest_cell_index_that_include(intersection_point);
    let (mut ix, mut iy, mut iz) = (ix as i32, iy as i32, iz as i32);
    // direction sign of each step (1 or -1)
    let step_x = if ray.direction.x > 0.0 { 1 } else { -1 };
    let step_y = if ray.direction.y > 0.0 { 1 } else { -1 };
    let step_z = if ray.direction.z > 0.0 { 1 } else { -1 };
    // t values: the value of t at which the ray crosses the closest voxel boundary in each dimension
    // they always refere to the **next** intersection along that dimension
    let next_cell_ix = ix + step_x.max(0);
    let next_cell_iy = iy + step_y.max(0);
    let next_cell_iz = iz + step_z.max(0);
    let next_cell_bbox = grid
        .cell_box(
            next_cell_ix as u32,
            next_cell_iy as u32,
            next_cell_iz as u32,
        )
        .min();
    let mut t_max_x = (next_cell_bbox.x - intersection_point.x) / ray.direction.x;
    let mut t_max_y = (next_cell_bbox.y - intersection_point.y) / ray.direction.y;
    let mut t_max_z = (next_cell_bbox.z - intersection_point.z) / ray.direction.z;

    // t_deltas: the *increment* of t when moving one unit along the ray in each dimension
    let cell_size = grid.cell_size();
    let t_delta_x = step_x as f64 * cell_size.x / ray.direction.x;
    let t_delta_y = step_y as f64 * cell_size.y / ray.direction.y;
    let t_delta_z = step_z as f64 * cell_size.z / ray.direction.z;

    let mut closest_hit: Option<HitResult> = None;
    while ix >= 0
        && ix < grid.cells_per_side as _
        && iy >= 0
        && iy < grid.cells_per_side as _
        && iz >= 0
        && iz < grid.cells_per_side as _
    {
        // checking for collision inside the list of triangles of this cell
        for triangle_i in grid.triangles(ix as _, iy as _, iz as _) {
            let triangle = model.get_triangle(*triangle_i);
            if let Some(hit) = triangle_ray_intersection(triangle, ray) {
                if closest_hit.is_none() || hit.t < closest_hit.unwrap().t {
                    closest_hit = Some(hit);
                }
            }
        }
        // found the closest hit in the list of triangles in the current cell
        // no need to proceed further searching in farther cells
        if closest_hit.is_some() {
            return closest_hit;
        }
        // advancing the ray to the next cell using the DDA algorithm
        if t_max_x < t_max_y {
            if t_max_x < t_max_z {
                ix += step_x;
                t_max_x += t_delta_x;
            } else {
                iz += step_z;
                t_max_z += t_delta_z;
            }
        } else {
            if t_max_y < t_max_z {
                iy += step_y;
                t_max_y += t_delta_y;
            } else {
                iz += step_z;
                t_max_z += t_delta_z;
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

    fn cell_size(&self) -> Vec3 {
        (self.bounding_box.max() - self.bounding_box.min()) / self.cells_per_side as f64
    }

    /// Returns the index of the cell that contains the point, None if the point is outside the grid
    fn cell_index_that_include(&self, point: Vec3) -> Option<(u32, u32, u32)> {
        if !self.bounding_box.contains(point) {
            return None;
        }
        let rel_point = (point - self.bounding_box.min()) / self.cell_size();
        let ix = rel_point.x as u32;
        let iy = rel_point.y as u32;
        let iz = rel_point.z as u32;
        // the point is for shure contained in the box of the grid,
        // but it could be on the border of the grid so have the grid coordinates equals to cells_per_side
        // clamp it to avoid this case
        Some((
            ix.clamp(0, self.cells_per_side - 1),
            iy.clamp(0, self.cells_per_side - 1),
            iz.clamp(0, self.cells_per_side - 1),
        ))
    }

    /// Returns the index of the cell that contains the point, clamped to the boundaries of the grid if the point is outside
    fn closest_cell_index_that_include(&self, point: Vec3) -> (u32, u32, u32) {
        let rel_point = (point - self.bounding_box.min()) / self.cell_size();
        let ix = rel_point.x as u32;
        let iy = rel_point.y as u32;
        let iz = rel_point.z as u32;
        // the point is for shure contained in the box of the grid,
        // but it could be on the border of the grid so have the grid coordinates equals to cells_per_side
        // clamp it to avoid this case
        (
            ix.clamp(0, self.cells_per_side - 1),
            iy.clamp(0, self.cells_per_side - 1),
            iz.clamp(0, self.cells_per_side - 1),
        )
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
        let grid = ModelGrid::new(&model, 32);
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
        Solid::Model { model, grid } => ray_intersect(model, grid, ray),
    }
}
