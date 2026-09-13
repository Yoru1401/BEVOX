use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::march::{MarchStats, march};
use bevox_core::material::MaterialId;
use glam::{Affine3A, UVec3, Vec3};

/// A floor slab with a block standing on it.
fn scene() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..8 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    for z in 28..36 {
        for y in 8..24 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    dense.into_contree()
}

#[test]
fn rendering_the_same_scene_twice_produces_identical_output() {
    let tree = scene();
    let a = render(&tree);
    let b = render(&tree);
    assert_eq!(a, b);
}

#[test]
fn the_reference_render_hits_the_floor_and_misses_the_sky() {
    let tree = scene();
    let mut stats = MarchStats::default();

    // Straight down onto the floor.
    let down = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(10.0, 40.0, 10.0),
        Vec3::NEG_Y,
        200.0,
        false,
        &mut stats,
    );
    assert!(down.is_some());
    assert_eq!(down.unwrap().voxel.y, 7);

    // Straight up into nothing.
    let up = march(
        &tree,
        Affine3A::IDENTITY,
        Vec3::new(10.0, 40.0, 10.0),
        Vec3::Y,
        200.0,
        false,
        &mut stats,
    );
    assert!(up.is_none());
    assert_eq!(stats.overruns, 0);
}

/// Renders a tiny image and returns the raw pixels.
fn render(tree: &Contree) -> Vec<u8> {
    let (w, h) = (32u32, 32u32);
    let mut pixels = Vec::with_capacity((w * h * 3) as usize);
    let mut stats = MarchStats::default();
    let eye = Vec3::new(-40.0, 40.0, -40.0);
    for py in 0..h {
        for px in 0..w {
            let u = px as f32 / w as f32 - 0.5;
            let v = 0.5 - py as f32 / h as f32;
            let dir = (Vec3::new(32.0, 16.0, 32.0) - eye).normalize()
                + Vec3::new(u, v, 0.0);
            match march(tree, Affine3A::IDENTITY, eye, dir.normalize(), 500.0, false, &mut stats) {
                Some(hit) => {
                    let shade = (hit.face_normal.dot(Vec3::new(0.3, 0.9, 0.2).normalize())
                        * 0.5
                        + 0.5)
                        * 255.0;
                    pixels.extend_from_slice(&[shade as u8, shade as u8, shade as u8]);
                }
                None => pixels.extend_from_slice(&[20, 30, 50]),
            }
        }
    }
    pixels
}
