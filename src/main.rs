use std::fs;
use std::time::Instant;
use std::{cmp::min, error::Error};

use image::{ImageBuffer, Rgb};
use rand::{self, Rng};

use rayon::prelude::*;

use clap::Parser;

mod raytracing;
use raytracing::core::{hit, Material, MaterialType, Ray, Scene, Vec3};
use raytracing::parser::SceneParser;

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Args {
    scene: String,

    #[arg(short, long, default_value_t = ("output.bmp".to_string()))]
    output: String,

    #[arg(short, long, default_value_t = 20)]
    sample_rate: u32,
}

impl From<Vec3> for image::Rgb<u8>  {
    fn from(value: Vec3) -> Self {
        let r = min((value.x * 255.0) as u8, 255);
        let g = min((value.y * 255.0) as u8, 255);
        let b = min((value.z * 255.0) as u8, 255);
        image::Rgb([r, g, b])
    }
}

fn get_bounce_direction(ray: &Ray, material: &Material, normal: Vec3) -> Vec3 {
    match material.type_ {
        MaterialType::Metal => ray.direction.reflect(normal),
        MaterialType::Plastic => {
            (normal + Vec3::random()).normalize()
        }
    }
}

fn cast(scene: &Scene, ray: &Ray) -> Vec3 {
    let mut color = Vec3::zero();

    let mut attenuation = 1.0;

    let mut current_ray = ray.clone();
    const K: i32 = 10;
    for _ in 0..K {
        if let Some(result) = hit(scene, &current_ray) {
            let mut diffuse_color = Vec3::zero();
            
            let normal = if current_ray.direction.dot(&result.normal) > 0.001 {
                // reverse the normal when we it a internal surface
                result.normal * -1.0
            } else {
                result.normal
            };
    
            // DEBUG: show the normal of the solid
            // return (normal + Vec3::one()) * 0.5;
            
            for light in &scene.lights {
                // check if some object occlude the light
                let v = (light.position - result.hit_point).normalize();
                let light_ray = Ray {origin: result.hit_point, direction: v};
                let distance_from_light = result.hit_point.distance(light.position);
                if let Some(occlusion_raycast) = hit(scene, &light_ray) {
                    let distance_from_occluder = occlusion_raycast.hit_point.distance(result.hit_point);
                    if distance_from_occluder <= distance_from_light {
                        continue;
                    }
                    // here the light is not really occluded, because the object is behind the light
                }
    
                // add to the diffusion component the diffusion light for this source
                let diffuse_effect = v.dot(&normal);
                if diffuse_effect > 0.001 {
                    let d = (distance_from_light / light.radius).max(1.0);
                    let decay_rate = 1.0 / (d * d);
                    // println!("light {:?} hitpoint {:?}", light.position, result.hit_point);
                    // println!("vector from light{:?} diffuse effect {:?}  normal {:?}", v, diffuse_effect, normal);
                    diffuse_color += light.color * decay_rate * diffuse_effect;
                }
            }
            
            let material = result.hitted_object.material;
            let diffuse_k = match material.type_ {
                MaterialType::Metal => 0.2,
                MaterialType::Plastic => 0.9,
            };
            let specular_k = match material.type_ {
                MaterialType::Metal => 0.8,
                MaterialType::Plastic => 0.1,
            };
            // the bounce changes based on the material type
            let bounce_direction = get_bounce_direction(&current_ray, &material, normal);
            color += material.color * attenuation * (diffuse_color * diffuse_k);
            // update ray and specular component for the next iteration
            current_ray = Ray {origin: result.hit_point, direction: bounce_direction};
            // attenuate the next bounced ray by the current specular component
            attenuation *= specular_k;
        } else {
            color += Vec3::one() * attenuation;
            break;
        }
    }
    return color;
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    
    // TODO: make the camera settable also from the file, with a custom orientation
    let cam_position = Vec3::zero();

    let content = fs::read_to_string(args.scene)?;
    let mut parser = SceneParser::new(&content);
    let parser_result = parser.parse_scene();
    if let Err(parser_error) = parser_result {
        parser_error.print_error_location(&content);
        return Err(Box::from("parser error"));
    }

    let data = parser_result.unwrap();

    let width = data.width;
    let height = data.height;

    let zoom = -1.0;
        
    let total_stripes = 32;
    let mut pixels = vec![Vec3::zero(); (width * height) as usize];
    let stripe_size = (height / total_stripes * width) as usize;
    let stripes: Vec<(usize, &mut [Vec3])> = pixels.chunks_mut(stripe_size)
        .enumerate()
        .collect();
    // measure time
    let start = Instant::now();
    // render the image inside the vec
    stripes.into_par_iter().for_each(|(stripe_index, stripe)| {
        for (i, vpixel) in stripe.into_iter().enumerate() {
            for _ in 0..args.sample_rate {
                let x = i % width as usize;
                let y = (stripe_size * stripe_index + i) / width as usize;
                let x_offset = rand::thread_rng().gen_range(-0.5..0.5);
                let y_offset = rand::thread_rng().gen_range(-0.5..0.5);
                // getting pixel ray coordinate
                let u = (x as f64 + x_offset - (width as f64) * 0.5) / (width as f64);
                let v = (y as f64 + y_offset - (height as f64) * 0.5) / (height as f64);
    
                let ray = Ray {
                    origin: cam_position,
                    direction: Vec3::new(u, v, zoom) - cam_position,
                };
                *vpixel += cast(&data.scene, &ray) / (args.sample_rate as f64);
            }
        }
    });

    let mut buffer = ImageBuffer::new(width, height);

    // write the raytracing result into the ImageBuffer
    for (x, y, pixel) in buffer.enumerate_pixels_mut() {
        let idx = (x + width * y) as usize;
        let p: Rgb<u8> = pixels[idx].into();
        *pixel = p;
    }

    let total_time = start.elapsed();
    println!("Rendered {} in {:?}", args.output, total_time);

    buffer.save(&args.output).unwrap();
    Ok(())
}
