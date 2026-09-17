use bevy::prelude::*;
use bevox_core::body::Body;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::distance_field::DistanceField;
use bevox_core::material::{Material, MaterialId, MaterialTable};
use bevox_core::physics::GRAVITY;
use bevox_core::physics::solver::step;
use bevox_render::BevoxRenderPlugin;
use bevox_render::camera::{FlyCamera, fly_camera_system, start_camera};
use bevox_render::pick::pick_voxel;
use bevox_render::upload::{VoxelScene, apply_brush, build_gpu_scene};

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
        // Before the rebuild too, not merely before staging. On a frame that
        // rebuilds because the last stroke outgrew the world region, a stroke
        // landing between the rebuild and the staging is drained into an update
        // the render world discards in favour of the snapshot -- which was
        // taken before that stroke. It is then lost until the next rebuild.
        .add_systems(Update, brush_input.after(fly_camera_system).before(build_gpu_scene))
        // Fixed timestep: physics must not depend on the frame rate. Bevy runs
        // `FixedUpdate` before `Update`, so a removal's generation bump lands
        // before `build_gpu_scene`.
        .add_systems(FixedUpdate, physics_system)
        .add_systems(Update, drop_body_input.after(fly_camera_system).before(build_gpu_scene))
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

    let (eye, look_at) = start_camera(&tree);

    // 3D camera exists only to supply view and projection matrices to the
    // shader; it renders nothing itself.
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_translation(eye),
        // Yaw and pitch must agree with the intended direction: the fly camera
        // rewrites the transform's rotation from them every frame.
        FlyCamera::looking_at(eye, look_at),
    ));

    // Fourteen voxels ahead, ten short of the surface the camera faces, so on
    // screen at start whatever scene was loaded. Six to the right of the line
    // of sight, more than the cube's half-diagonal of about five, so it never
    // sits under the middle of the screen: picking does not see bodies yet,
    // and a click on the body would paint or erase the world behind it.
    let forward = (look_at - eye).normalize();
    let right = forward.cross(Vec3::Y).normalize_or_zero();
    let mut body = demo_body(eye + forward * 14.0 + right * 6.0, Quat::IDENTITY);
    // With mass, it falls from there onto whatever is below it.
    body.recompute(&materials);

    let field = DistanceField::build(&tree);
    commands.insert_resource(VoxelScene {
        tree,
        materials,
        generation: 1,
        field,
        field_dirty: None,
        bodies: vec![body],
    });
}

/// A six-voxel cube of material 2 -- brick, in the demo palette -- whose middle
/// is at `centre`, turned by `orientation`. Call `recompute` before simulating
/// it.
///
/// Off the 4-voxel brick grid (5..11 in a 16 volume) so its bricks are partial
/// and it owns real voxel bytes, like the body the parity tests prove, rather
/// than collapsing to uniform nodes.
fn demo_body(centre: Vec3, orientation: Quat) -> Body {
    let mut dense = DenseVolume::new(16).unwrap();
    for z in 5..11 {
        for y in 5..11 {
            for x in 5..11 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    Body::new(dense.into_contree(), centre - orientation * Vec3::splat(8.0), orientation)
}

/// One physics tick for every body.
fn physics_system(time: Res<Time>, mut scene: ResMut<VoxelScene>) {
    let scene = &mut *scene;
    let changed = step(
        &mut scene.bodies,
        &scene.tree,
        &scene.field,
        &scene.materials,
        GRAVITY,
        time.delta_secs(),
    );
    if changed {
        // A body left the world. The body list is packed into the scene
        // buffers, so they must be rebuilt.
        scene.generation += 1;
    }
}

/// `F` drops a tilted cube in front of the camera, to watch landing and
/// settling again and again.
fn drop_body_input(
    keys: Res<ButtonInput<KeyCode>>,
    camera: Query<&GlobalTransform, With<Camera3d>>,
    mut scene: ResMut<VoxelScene>,
) {
    if !keys.just_pressed(KeyCode::KeyF) {
        return;
    }
    let Ok(transform) = camera.single() else {
        return;
    };
    let tilt = Quat::from_euler(EulerRot::XYZ, 0.4, 0.7, 0.2);
    let mut body =
        demo_body(transform.translation() + transform.forward().as_vec3() * 16.0, tilt);
    if body.recompute(&scene.materials) {
        scene.bodies.push(body);
        // A new body adds geometry to the packed buffers.
        scene.generation += 1;
    }
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
    apply_brush(&mut scene, centre, brush.radius, material);
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
    // A slippery strip and a bouncy patch in the floor's top layer, so a
    // dropped body shows friction and restitution without any editing.
    for z in 0..64 {
        for x in 8..24 {
            dense.set(UVec3::new(x, 5, z), MaterialId(3)); // ice
        }
    }
    for z in 40..56 {
        for x in 40..56 {
            dense.set(UVec3::new(x, 5, z), MaterialId(4)); // rubber
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
    materials
        .push(Material { color: [140, 140, 150, 255], density: 2600, friction: 60, restitution: 5 })
        .unwrap(); // 1: stone
    materials
        .push(Material { color: [180, 90, 70, 255], density: 1900, friction: 70, restitution: 5 })
        .unwrap(); // 2: brick
    materials
        .push(Material { color: [170, 210, 235, 255], density: 900, friction: 4, restitution: 10 })
        .unwrap(); // 3: ice
    materials
        .push(Material { color: [40, 40, 45, 255], density: 1100, friction: 80, restitution: 80 })
        .unwrap(); // 4: rubber
    (tree, materials)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The demo floor has an ice strip and a rubber patch, so dropping bodies
    /// shows friction and bounce without editing anything.
    #[test]
    fn the_demo_scene_has_something_slippery_and_something_bouncy() {
        let (tree, materials) = demo_scene();
        let has = |m: MaterialId| {
            (0..64).any(|x| (0..64).any(|z| tree.get(UVec3::new(x, 5, z)) == m))
        };
        assert!(has(MaterialId(3)), "no ice on the floor");
        assert!(has(MaterialId(4)), "no rubber on the floor");
        assert_eq!(materials.get(MaterialId(3)).friction, 4);
        assert_eq!(materials.get(MaterialId(4)).restitution, 80);
    }

    /// The system steps the scene's bodies, and a body leaving the world bumps
    /// the generation, because the packed buffers hold the body list.
    #[test]
    fn the_physics_system_moves_bodies_and_rebuilds_when_one_leaves() {
        let mut world = World::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f64(1.0 / 64.0));
        world.insert_resource(time);
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut falling = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(falling.recompute(&materials));
        let mut gone = demo_body(Vec3::new(20.0, -100.0, 20.0), Quat::IDENTITY);
        assert!(gone.recompute(&materials));
        world.insert_resource(VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![falling, gone],
        });

        let physics = world.register_system(physics_system);
        world.run_system(physics).unwrap();

        let scene = world.resource::<VoxelScene>();
        assert_eq!(scene.bodies.len(), 1, "the body below the world was not removed");
        assert_eq!(scene.generation, 2, "removing a body did not ask for a rebuild");
        assert!(scene.bodies[0].velocity.y < 0.0, "the body did not start to fall");
    }
}
