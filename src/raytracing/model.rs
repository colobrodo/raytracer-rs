use obj::Obj;

use super::{Box3, HitResult, Mat4, Ray, RayHittable, RayIntersectable, Vec3};

pub type Triangle = (Vec3, Vec3, Vec3);

#[derive(Debug)]
pub struct Model {
    obj: Obj,
    trasform: Mat4,
    pub bounding_box: Box3,
}

impl RayIntersectable for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<f64> {
        // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
        let (v0, v1, v2) = *self;
        let v0v1 = v1 - v0;
        let v0v2 = v2 - v0;
        let ray_cross_e2 = ray.direction.cross(v0v2);
        let determinant = v0v1.dot(ray_cross_e2);
        // ray and triangle are parallel if det is close to 0
        if determinant.abs() < f64::EPSILON {
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

        Some(t)
    }
}

impl RayHittable for Triangle {
    fn hit(&self, ray: &Ray) -> Option<HitResult> {
        let t = self.intersect(ray)?;
        let (v0, v1, v2) = *self;
        let v0v1 = v1 - v0;
        let v0v2 = v2 - v0;
        Some(HitResult {
            normal: v0v1.cross(v0v2).normalize(),
            t,
        })
    }
}

impl Model {
    pub fn new(obj: Obj, trasform: Mat4) -> Model {
        let bounding_box = calculate_bounding_box(&obj, &trasform);
        Model {
            obj,
            trasform,
            bounding_box,
        }
    }

    pub fn get_triangle(&self, i: usize) -> Triangle {
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

    fn iter_triangles(&self) -> impl Iterator<Item = Triangle> + '_ {
        (0..self.obj.indices.len() / 3)
            .map(|i| self.get_triangle(i))
            .into_iter()
    }

    pub fn create_grid(&self, cells_per_side: usize) -> ModelGrid {
        // foreach box create a vec
        let mut cells_triangles: Vec<Vec<usize>> =
            vec![Vec::new(); cells_per_side * cells_per_side * cells_per_side];
        let cell_size = (self.bounding_box.max() - self.bounding_box.min()) / cells_per_side as f64;
        for (i, (v0, v1, v2)) in self.iter_triangles().enumerate() {
            // triangle bounding box
            let mut bbox = Box3::from_single_point(v0);
            bbox.include(v1);
            bbox.include(v2);
            // check each corner of the bounding box which cell intersect
            // get the buffer index of the minimim corner of the bounding box
            let min_point = (bbox.min() - self.bounding_box.min()) / cell_size;
            let min_ix = (min_point.x as usize).min(cells_per_side - 1);
            let min_iy = (min_point.y as usize).min(cells_per_side - 1);
            let min_iz = (min_point.z as usize).min(cells_per_side - 1);
            // get the buffer index of the maximim corner of the bounding box
            let max_point = (bbox.max() - self.bounding_box.min()) / cell_size;
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
            bounding_box: self.bounding_box,
            cells_per_side: cells_per_side as u32,
            offset_array: offsets,
            triangle_indices,
        }
    }
}

#[derive(Debug)]
pub struct ModelGrid {
    bounding_box: Box3,
    cells_per_side: u32,
    offset_array: Vec<usize>,
    triangle_indices: Vec<usize>,
}

impl ModelGrid {
    pub fn cells_per_side(&self) -> u32 {
        self.cells_per_side
    }

    /// Returns the buffer's indexes of the triangles that intersect the cell  
    pub fn triangles(&self, ix: u32, iy: u32, iz: u32) -> impl Iterator<Item = &usize> {
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

    /// Returns the bounding box of the cell at the given discrete index
    pub fn cell_box(&self, ix: u32, iy: u32, iz: u32) -> Box3 {
        let half_cell_size = self.bounding_box.half_extension / self.cells_per_side as f64;
        Box3::new(
            self.bounding_box.min()
                + half_cell_size
                    * 2.0
                    * Vec3::new(ix as f64 + 0.5, iy as f64 + 0.5, iz as f64 + 0.5),
            half_cell_size,
        )
    }

    /// Returns the dimension of a single cell: note that all the cells have the same size
    pub fn cell_size(&self) -> Vec3 {
        (self.bounding_box.max() - self.bounding_box.min()) / self.cells_per_side as f64
    }

    /// Returns the index of the cell that contains the point, None if the point is outside the grid
    pub fn cell_index_that_include(&self, point: Vec3) -> Option<(u32, u32, u32)> {
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
    pub fn closest_cell_index_that_include(&self, point: Vec3) -> (u32, u32, u32) {
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

    pub fn boxes_indices(&self) -> impl Iterator<Item = (u32, u32, u32)> + '_ {
        (0..self.cells_per_side)
            .flat_map(|i| (0..self.cells_per_side).map(move |j| (i, j)))
            .flat_map(|(i, j)| (0..self.cells_per_side).map(move |k| (i, j, k)))
            .into_iter()
    }

    pub fn enum_boxes(&self) -> impl Iterator<Item = ((u32, u32, u32), Box3)> + '_ {
        self.boxes_indices()
            .map(|(ix, iy, iz)| ((ix, iy, iz), self.cell_box(ix, iy, iz)))
            .into_iter()
    }
}

pub fn calculate_bounding_box(obj: &Obj, trasform: &Mat4) -> Box3 {
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
