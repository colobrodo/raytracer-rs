use super::math::{Mat4, Ray, Vec3};
use super::model::{Model, ModelGrid, Triangle};
use super::{HitResult, RayHittable, RayIntersectable};
use std::fs::File;
use std::vec::Vec;
use std::{error::Error, io::BufReader};

use obj::load_obj;

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

pub struct RaycastResult<'a> {
    pub hitted_object: &'a SceneObject,
    pub hit_point: Vec3,
    pub normal: Vec3,
}

#[derive(Debug)]
pub enum Solid {
    Sphere { center: Vec3, radius: f64 },
    Plane { normal: Vec3, distance: f64 },
    Model { model: Model, grid: ModelGrid },
}

impl Solid {
    pub fn load_model(filename: &str, trasform: Mat4) -> Result<Solid, Box<dyn Error>> {
        let input = BufReader::new(File::open(filename)?);
        let obj = load_obj(input)?;
        // DEBUG: remove commented println!
        // println!("loaded {}, with bounding box {:?}", filename, bounding_box);
        let model = Model::new(obj, trasform);
        let grid = model.create_grid(32);
        // println!("Created grid with this offsets: {:?}", grid.offset_array);
        Ok(Solid::Model { grid, model })
    }
}

pub fn hit<'a>(scene: &'a Scene, ray: &'a Ray) -> Option<RaycastResult<'a>> {
    let mut closest_t = f64::INFINITY;
    let mut closest_object = None;
    let mut raycast_to_closest = None;
    for object in &scene.objects {
        if let Some(result) = object.solid.hit(ray) {
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

fn ray_intersect(model: &Model, grid: &ModelGrid, ray: &Ray) -> Option<HitResult> {
    let bbox_intersection_t = model.bounding_box.intersect(ray)?;
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

    // save the closest triangle hittend and the t value for that hit
    let mut closest_hit: Option<(usize, f64)> = None;
    while ix >= 0
        && ix < grid.cells_per_side() as _
        && iy >= 0
        && iy < grid.cells_per_side() as _
        && iz >= 0
        && iz < grid.cells_per_side() as _
    {
        // checking for collision inside the list of triangles of this cell
        for &triangle_i in grid.triangles(ix as _, iy as _, iz as _) {
            let triangle = model.get_triangle(triangle_i);
            if let Some(t) = triangle.intersect(ray) {
                if closest_hit.is_none() || t < closest_hit.unwrap().1 {
                    closest_hit = Some((triangle_i, t));
                }
            }
        }
        // found the closest hit in the list of triangles in the current cell
        // no need to proceed further searching in farther cells
        if closest_hit.is_some() {
            break;
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
    closest_hit.map(|(triangle_i, t)| {
        let triangle = model.get_triangle(triangle_i);
        let hit_point = ray.at(t);
        let normals = model.get_triangle_normals(triangle_i);
        HitResult {
            t,
            normal: triangle.normal_on_point(hit_point, normals),
        }
    })
}

impl RayHittable for Solid {
    fn hit(&self, ray: &Ray) -> Option<HitResult> {
        match self {
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
}
