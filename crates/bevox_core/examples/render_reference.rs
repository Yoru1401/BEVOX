//! Renders the reference scene to `reference.png`.
//!
//! Run with: cargo run -p bevox_core --example render_reference --release

use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::{DEFAULT_DENSITY, Material, MaterialId, MaterialTable};
use bevox_core::normal::implicit_normal;
use glam::{Affine3A, UVec3, Vec3};

const WIDTH: u32 = 512;
const HEIGHT: u32 = 512;

fn main() {
    let mut table = MaterialTable::new();
    let stone = table.push(Material { color: [140, 140, 150, 255], density: DEFAULT_DENSITY }).unwrap();
    let brick = table.push(Material { color: [180, 90, 70, 255], density: DEFAULT_DENSITY }).unwrap();

    let tree = build_scene(stone, brick);
    let sun = Vec3::new(0.4, 1.0, 0.25).normalize();
    let eye = Vec3::new(-30.0, 48.0, -30.0);
    let target = Vec3::new(32.0, 12.0, 32.0);

    let forward = (target - eye).normalize();
    let right = forward.cross(Vec3::Y).normalize();
    let up = right.cross(forward);

    let mut stats = MarchStats::default();
    let mut buffer = Vec::with_capacity((WIDTH * HEIGHT * 3) as usize);

    for py in 0..HEIGHT {
        for px in 0..WIDTH {
            let u = (px as f32 + 0.5) / WIDTH as f32 - 0.5;
            let v = 0.5 - (py as f32 + 0.5) / HEIGHT as f32;
            let dir = (forward + right * u + up * v).normalize();

            let color = match march(
                &tree,
                Affine3A::IDENTITY,
                eye,
                dir,
                500.0,
                false,
                &mut stats,
            ) {
                Some(hit) => {
                    let normal = implicit_normal(&tree, hit.voxel, hit.face_normal);
                    shade(
                        &tree,
                        &table,
                        hit.voxel,
                        hit.material,
                        normal,
                        hit.face_normal,
                        sun,
                        &mut stats,
                    )
                }
                None => [90, 120, 180],
            };
            buffer.extend_from_slice(&color);
        }
    }

    println!("march steps: {}, overruns: {}", stats.steps, stats.overruns);
    assert_eq!(stats.overruns, 0, "a ray exceeded the step cap");

    image::save_buffer("reference.png", &buffer, WIDTH, HEIGHT, image::ExtendedColorType::Rgb8)
        .expect("failed to write reference.png");
    println!("wrote reference.png");
}

fn shade(
    tree: &Contree,
    table: &MaterialTable,
    voxel: UVec3,
    material: MaterialId,
    normal: Vec3,
    face_normal: Vec3,
    sun: Vec3,
    stats: &mut MarchStats,
) -> [u8; 3] {
    let base = table.get(material).color;

    // Offset along the normal so the shadow ray does not re-hit its own voxel.
    // Face normal rather than the smoothed one: at a three-way corner the
    // blended normal is normalize(1,1,1), and 0.75 along it clears only 0.433
    // per axis -- inside the voxel's own 0.5 half-extent, so the shadow ray
    // hits the surface it started from.
    let origin = voxel.as_vec3() + Vec3::splat(0.5) + face_normal * 0.75;
    let shadowed = march(tree, Affine3A::IDENTITY, origin, sun, 500.0, true, stats).is_some();

    let ambient = 0.25;
    let diffuse = if shadowed { 0.0 } else { normal.dot(sun).max(0.0) * 0.75 };
    let light = ambient + diffuse;

    [
        (base[0] as f32 * light) as u8,
        (base[1] as f32 * light) as u8,
        (base[2] as f32 * light) as u8,
    ]
}

fn build_scene(stone: MaterialId, brick: MaterialId) -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..8 {
                dense.set(UVec3::new(x, y, z), stone);
            }
        }
    }
    for z in 28..36 {
        for y in 8..24 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), brick);
            }
        }
    }
    let mut tree = dense.into_contree();
    // Carve a hollow so the brush path appears in the deliverable image.
    tree.apply_sphere(Vec3::new(32.0, 20.0, 32.0), 5.0, MaterialId::EMPTY);
    tree
}
