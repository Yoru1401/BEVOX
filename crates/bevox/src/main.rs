use bevy::prelude::*;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::distance_field::DistanceField;
use bevox_core::material::{Material, MaterialId, MaterialTable};
use bevox_render::BevoxRenderPlugin;
use bevox_render::camera::{FlyCamera, fly_camera_system};
use bevox_render::pick::pick_voxel;
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
        // Frame times are the argument for the optimisations in milestone 8, so
        // they are measured rather than estimated by eye.
        .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
        .add_plugins(bevy::diagnostic::LogDiagnosticsPlugin {
            wait_duration: std::time::Duration::from_secs(3),
            ..default()
        })
        .add_plugins(BevoxRenderPlugin)
        .init_resource::<BrushSettings>()
        .add_systems(Startup, setup)
        .add_systems(Update, brush_input.after(fly_camera_system))
        .run();
}

fn setup(mut commands: Commands) {
    // 2D camera composites the sprite showing the marched image.
    commands.spawn(Camera2d);

    let (tree, materials) = match std::env::args().nth(1) {
        Some(path) => match bevox_core::vox::load_scene(std::path::Path::new(&path)) {
            Ok((tree, materials)) => {
                info!(
                    "loaded {path}: extent {}, {} arena nodes",
                    tree.extent(),
                    tree.arena().nodes().len()
                );
                (tree, materials)
            }
            Err(e) => {
                // A bad path is a typo, not a crash: say so and show the demo.
                error!("could not load {path}: {e}");
                demo_scene()
            }
        },
        None => demo_scene(),
    };

    // Framed from the volume's extent rather than hardcoded: an imported model
    // may be 16 or 1024 voxels across, and a fixed position would put the camera
    // inside the geometry or leave it off screen.
    let extent = tree.extent() as f32;
    let centre = Vec3::splat(extent * 0.5);
    let eye = centre + Vec3::new(-1.0, 1.2, -1.0).normalize() * extent * 1.1;

    // 3D camera exists only to supply view and projection matrices to the
    // shader; it renders nothing itself.
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_translation(eye),
        // Yaw and pitch must agree with the intended direction: the fly camera
        // rewrites the transform's rotation from them every frame.
        FlyCamera::looking_at(eye, centre),
    ));

    let field = DistanceField::build(&tree);
    commands.insert_resource(VoxelScene { tree, materials, generation: 1, field, field_dirty: None });
}

/// What the brush paints and how big it is.
#[derive(Resource)]
struct BrushSettings {
    radius: f32,
    material: MaterialId,
}

impl Default for BrushSettings {
    fn default() -> Self {
        Self { radius: 4.0, material: MaterialId(1) }
    }
}

/// Left click paints at the cursor, right click erases, the wheel resizes.
///
/// Camera look is on middle-drag, which is what frees both other buttons for
/// editing. Two earlier arrangements were bugs: erase on right-click when look
/// was also on right-click destroyed geometry on every camera rotation, and
/// erase on middle-click shared a physical control with the resize wheel. Ctrl
/// is the movement-speed boost and nothing else.
///
/// The pick runs against the same tree the renderer draws, so what is clicked
/// is what was seen. Placing the sphere at the hit point rather than at the
/// voxel centre keeps the brush from stepping in whole voxels as the camera
/// turns.
/// The cursor in normalised device coordinates: -1 to 1 on each axis, y up.
///
/// Screen coordinates run y-down from the top-left, so y is flipped. This
/// matches `primary_ray` in march.wgsl, which is what makes a click land on
/// the voxel that was drawn under the pointer.
fn cursor_ndc(window: &Window) -> Option<Vec2> {
    let p = window.cursor_position()?;
    let w = window.width();
    let h = window.height();
    if w <= 0.0 || h <= 0.0 {
        return None;
    }
    Some(Vec2::new(p.x / w * 2.0 - 1.0, 1.0 - p.y / h * 2.0))
}

fn brush_input(
    buttons: Res<ButtonInput<MouseButton>>,
    mut wheel: MessageReader<bevy::input::mouse::MouseWheel>,
    mut brush: ResMut<BrushSettings>,
    mut scene: ResMut<VoxelScene>,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
    windows: Query<&Window>,
) {
    for event in wheel.read() {
        brush.radius = (brush.radius + event.y).clamp(1.0, 32.0);
    }

    let paint = buttons.just_pressed(MouseButton::Left);
    let erase = buttons.just_pressed(MouseButton::Right);
    if !paint && !erase {
        return;
    }
    let Ok((transform, projection)) = camera.single() else {
        return;
    };

    let eye = transform.translation();
    let world_from_clip =
        (projection.get_clip_from_view() * transform.to_matrix().inverse()).inverse();

    // The cursor, not the crosshair. Look is on middle-drag, so the pointer is
    // free and the user aims with it. A cursor outside the window picks
    // nothing rather than falling back to the centre, which would place a
    // sphere somewhere the user was not pointing.
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(ndc) = cursor_ndc(window) else {
        return;
    };
    let Some(hit) = pick_voxel(&scene.tree, world_from_clip, eye, ndc) else {
        return;
    };

    let material = if erase { MaterialId::EMPTY } else { brush.material };
    // Paint on the near side of the surface so a click adds material in front
    // of what was hit rather than burying it inside.
    let centre = if erase {
        hit.position
    } else {
        hit.position + hit.normal * brush.radius
    };
    scene.tree.apply_sphere(centre, brush.radius, material);
    // Only painting needs the field touched: it adds geometry, which lowers
    // true distances, and a stale field would then over-estimate and let a
    // ray skip the new geometry. Erasing only raises true distances, so a
    // stale field merely under-estimates -- costing speed, never correctness
    // -- and is left alone.
    if !erase {
        scene.field_dirty = Some(scene.field.lower_around(centre, brush.radius));
    }
    // Deliberately not bumped: an edit is uploaded by range, and bumping the
    // generation is what asks for a full rebuild.
}

/// The same floor, column and carved sphere the parity test uses, so what is on
/// screen is what the test proved correct, together with the palette it is
/// drawn from.
fn demo_scene() -> (Contree, MaterialTable) {
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

    let mut materials = MaterialTable::new();
    materials.push(Material { color: [140, 140, 150, 255] }).unwrap(); // 1: stone
    materials.push(Material { color: [180, 90, 70, 255] }).unwrap(); // 2: brick
    (tree, materials)
}
