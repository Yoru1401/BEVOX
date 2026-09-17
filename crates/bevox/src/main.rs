use bevy::prelude::*;
use bevox_core::body::Body;
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::distance_field::DistanceField;
use bevox_core::material::{Material, MaterialId, MaterialTable};
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
        // Before the rebuild, and so before the staging that follows it: a
        // rebuild frame packs this frame's turn into the new buffers, and every
        // other frame's table carries it too, rather than the last one.
        .add_systems(Update, spin_bodies.before(build_gpu_scene))
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
    let body = demo_body(eye + forward * 14.0 + right * 6.0);

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

/// A six-voxel cube of material 2 -- brick, in the demo palette -- centred on
/// `centre`.
///
/// Off the 4-voxel brick grid (5..11 in a 16 volume) so its bricks are partial
/// and it owns real voxel bytes, like the body the parity tests prove, rather
/// than collapsing to uniform nodes.
fn demo_body(centre: Vec3) -> Body {
    let mut dense = DenseVolume::new(16).unwrap();
    for z in 5..11 {
        for y in 5..11 {
            for x in 5..11 {
                dense.set(UVec3::new(x, y, z), MaterialId(2));
            }
        }
    }
    Body::new(dense.into_contree(), centre - Vec3::splat(8.0), Quat::IDENTITY)
}

/// Turns the demo body, so the composition can be seen working before any
/// physics exists. Milestone 2 replaces this with integration.
///
/// About the volume's centre, not its local origin: a body turns about its
/// origin, which is a corner, and would otherwise swing through a sphere
/// rather than spin in place. Recovering the centre from the position every
/// step lets it creep by float error -- about 0.06 voxels over 100,000 frames,
/// half an hour at 60 Hz -- which a stand-in for physics can afford.
fn spin_bodies(time: Res<Time>, mut scene: ResMut<VoxelScene>) {
    let turn = Quat::from_rotation_y(time.delta_secs() * 0.7)
        * Quat::from_rotation_x(time.delta_secs() * 0.3);
    for body in &mut scene.bodies {
        let half = Vec3::splat(body.volume.extent() as f32 * 0.5);
        let centre = body.world_from_local().transform_point3(half);
        // Renormalised every step: a quaternion accumulated by repeated
        // multiplication drifts off unit length, and the drift shows up as a
        // body that slowly shears.
        body.orientation = (turn * body.orientation).normalize();
        body.position = centre - body.orientation * half;
        // Only the transform changed, so `generation` stays put: the body
        // table is re-uploaded every frame. Changing a body's geometry is
        // what needs the bump.
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
    materials.push(Material { color: [140, 140, 150, 255], density: 2600 }).unwrap(); // 1: stone
    materials.push(Material { color: [180, 90, 70, 255], density: 1900 }).unwrap(); // 2: brick
    (tree, materials)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Many frames of spin must leave the orientation a rotation -- unit
    /// length -- and leave the cube where it started.
    #[test]
    fn a_spun_body_stays_unit_length_and_in_place() {
        const STEPS: u32 = 100_000;
        let mut world = World::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_millis(16));
        world.insert_resource(time);
        let tree = Contree::empty(2);
        let field = DistanceField::build(&tree);
        let centre = Vec3::new(40.0, 21.0, 40.0);
        world.insert_resource(VoxelScene {
            tree,
            materials: MaterialTable::new(),
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![demo_body(centre)],
        });

        let spin = world.register_system(spin_bodies);
        world.run_system(spin).unwrap();
        assert_ne!(
            world.resource::<VoxelScene>().bodies[0].orientation,
            Quat::IDENTITY,
            "one step did not turn the body, so the checks below prove nothing"
        );
        for _ in 0..STEPS {
            world.run_system(spin).unwrap();
        }

        let body = &world.resource::<VoxelScene>().bodies[0];
        let length = body.orientation.length();
        assert!(
            (length - 1.0).abs() < 1e-5,
            "after {STEPS} steps the orientation has length {length}; the body would shear"
        );
        let now = body.world_from_local().transform_point3(Vec3::splat(8.0));
        // Float creep is ~0.06 voxels at this step count. Swinging about the
        // corner instead would carry the centre round a 14-voxel radius.
        assert!(
            (now - centre).length() < 0.1,
            "the cube's centre wandered from {centre:?} to {now:?}"
        );
    }
}
