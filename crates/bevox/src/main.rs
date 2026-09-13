use bevy::prelude::*;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::material::MaterialId;
use bevox_render::BevoxRenderPlugin;
use bevox_render::camera::FlyCamera;
use bevox_render::upload::VoxelScene;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "BEVOX".into(),
                        resolution: (1280u32, 720u32).into(),
                        ..default()
                    }),
                    ..default()
                })
                // Shaders live in the render crate so the parity test can read
                // them crate-relative. Bevy resolves a relative asset root
                // against the executable's directory, not the workspace, so
                // this is made absolute at compile time instead.
                .set(AssetPlugin {
                    file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/../bevox_render/assets")
                        .to_string(),
                    ..default()
                }),
        )
        .add_plugins(BevoxRenderPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    // 2D camera composites the sprite showing the marched image.
    commands.spawn(Camera2d);

    // 3D camera exists only to supply view and projection matrices to the
    // shader; it renders nothing itself.
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_xyz(-30.0, 40.0, -30.0),
        // Yaw and pitch must agree with the intended direction: the fly camera
        // rewrites the transform's rotation from them every frame.
        FlyCamera::looking_at(Vec3::new(-30.0, 40.0, -30.0), Vec3::new(32.0, 12.0, 32.0)),
    ));

    commands.insert_resource(VoxelScene { tree: demo_scene(), generation: 1 });
}

/// The same floor, column and carved sphere the parity test uses, so what is on
/// screen is what the test proved correct.
fn demo_scene() -> Contree {
    let mut dense = DenseVolume::new(64).unwrap();
    for z in 0..64 {
        for x in 0..64 {
            for y in 0..6 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    for z in 28..36 {
        for y in 6..26 {
            for x in 28..36 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    let mut tree = dense.into_contree();
    tree.apply_sphere(Vec3::new(32.0, 18.0, 32.0), 5.0, MaterialId::EMPTY);
    tree
}
