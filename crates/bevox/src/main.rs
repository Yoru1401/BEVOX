use bevy::prelude::*;
use bevox_core::body::{Body, BodyId};
use bevox_core::contree::Contree;
use bevox_core::dense::DenseVolume;
use bevox_core::distance_field::DistanceField;
use bevox_core::material::{Material, MaterialId, MaterialTable};
use bevox_core::physics::GRAVITY;
use bevox_core::physics::detach::detach;
use bevox_core::physics::joint::{Joint, JointKind, follow};
use bevox_core::physics::sculpt::sculpt;
use bevox_core::physics::solver::{Grab, step};
use bevox_render::BevoxRenderPlugin;
use bevox_render::camera::{FlyCamera, fly_camera_system, start_camera};
use bevox_render::pick::{Hit, Target, cursor_ray, pick};
use bevox_render::pipeline::MAX_BODIES;
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
        .init_resource::<GrabState>()
        .init_resource::<JointState>()
        .init_resource::<Joints>()
        .add_systems(Startup, setup)
        // Before the rebuild too, not merely before staging. On a frame that
        // rebuilds because the last stroke outgrew the world region, a stroke
        // landing between the rebuild and the staging is drained into an update
        // the render world discards in favour of the snapshot -- which was
        // taken before that stroke. It is then lost until the next rebuild.
        .add_systems(Update, toggle_grab_mode.before(brush_input).before(grab_input))
        .add_systems(Update, toggle_joint_mode.after(toggle_grab_mode).before(joint_input))
        .add_systems(Update, joint_input.after(fly_camera_system))
        .add_systems(Update, brush_input.after(fly_camera_system).before(build_gpu_scene))
        .add_systems(Update, grab_input.after(fly_camera_system))
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
    // screen at start whatever scene was loaded, and six to the right of the
    // line of sight, so it does not hide what the camera is looking at.
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
fn physics_system(
    time: Res<Time>,
    grab: Res<GrabState>,
    mut joints: ResMut<Joints>,
    mut scene: ResMut<VoxelScene>,
) {
    let scene = &mut *scene;
    let changed = step(
        &mut scene.bodies,
        &scene.tree,
        &scene.field,
        &scene.materials,
        GRAVITY,
        time.delta_secs(),
        grab.held.as_ref(),
        &mut joints.0,
    );
    // Edits since the last tick may have split or removed a jointed body.
    follow(&mut joints.0, &scene.bodies);
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

/// The mouse grab: whether the left mouse grabs instead of painting, what it
/// holds, and how far in front of the camera the held point is kept.
#[derive(Resource, Default)]
struct GrabState {
    enabled: bool,
    held: Option<Grab>,
    distance: f32,
}

/// `G` switches grab mode on and off. Switching it off lets go, and the window
/// title says which mode is on.
fn toggle_grab_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<GrabState>,
    mut joint: ResMut<JointState>,
    mut windows: Query<&mut Window>,
) {
    if !keys.just_pressed(KeyCode::KeyG) {
        return;
    }
    state.enabled = !state.enabled;
    state.held = None;
    if state.enabled {
        joint.enabled = false;
        joint.pending = None;
    }
    if let Ok(mut window) = windows.single_mut() {
        window.title = title(&state, &joint);
    }
}

/// The window title, naming the tool in use.
fn title(grab: &GrabState, joint: &JointState) -> String {
    if grab.enabled {
        "BEVOX \u{2014} grab mode (G)".into()
    } else if joint.enabled {
        let kind = match joint.kind {
            JointKind::Ball => "ball",
            JointKind::Hinge => "hinge",
        };
        format!("BEVOX \u{2014} joint mode: {kind} (J; H switches)")
    } else {
        "BEVOX".into()
    }
}

/// Every joint in the scene.
#[derive(Resource, Default)]
struct Joints(Vec<Joint>);

/// The joint tool: whether the left mouse makes joints, which kind, and the
/// first click of a joint still waiting for its second: the body, the pivot,
/// and the hinge axis.
#[derive(Resource)]
struct JointState {
    enabled: bool,
    kind: JointKind,
    pending: Option<(BodyId, Vec3, Vec3)>,
}

impl Default for JointState {
    fn default() -> Self {
        Self { enabled: false, kind: JointKind::Ball, pending: None }
    }
}

/// `J` switches joint mode on and off, turning grab mode off; `H`, in joint
/// mode, switches between ball and hinge.
fn toggle_joint_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<JointState>,
    mut grab: ResMut<GrabState>,
    mut windows: Query<&mut Window>,
) {
    let mut changed = false;
    if keys.just_pressed(KeyCode::KeyJ) {
        state.enabled = !state.enabled;
        state.pending = None;
        if state.enabled {
            grab.enabled = false;
            grab.held = None;
        }
        changed = true;
    }
    if state.enabled && keys.just_pressed(KeyCode::KeyH) {
        state.kind = match state.kind {
            JointKind::Ball => JointKind::Hinge,
            JointKind::Hinge => JointKind::Ball,
        };
        changed = true;
    }
    if changed && let Ok(mut window) = windows.single_mut() {
        window.title = title(&grab, &state);
    }
}

/// One click of the joint tool.
///
/// The first, on a body, sets the pivot just inside the clicked face, and for a
/// hinge takes the face's normal as the axis. The second fastens that pivot to
/// what was clicked, a body or the world, wherever it is: nothing moves. A
/// second click on the same body cancels.
fn joint_click(state: &mut JointState, joints: &mut Joints, bodies: &[Body], hit: Hit) {
    let Some((first, pivot, axis)) = state.pending else {
        if let Target::Body(i) = hit.target {
            state.pending = Some((bodies[i].id, hit.position - hit.normal * 0.01, hit.normal));
        }
        return;
    };
    state.pending = None;
    let Some(a) = bodies.iter().find(|b| b.id == first) else {
        return;
    };
    let b = match hit.target {
        Target::World => None,
        Target::Body(i) if bodies[i].id == first => return,
        Target::Body(i) => Some(&bodies[i]),
    };
    joints.0.push(Joint::new(state.kind, a, b, pivot, axis));
}

/// In joint mode, a left click picks what is under the cursor and makes the
/// next click of a joint.
fn joint_input(
    buttons: Res<ButtonInput<MouseButton>>,
    mut state: ResMut<JointState>,
    mut joints: ResMut<Joints>,
    scene: Res<VoxelScene>,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
    windows: Query<&Window>,
) {
    if !state.enabled || !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    let (Ok((transform, projection)), Ok(window)) = (camera.single(), windows.single()) else {
        return;
    };
    let Some(ndc) = cursor_ndc(window) else {
        return;
    };
    let eye = transform.translation();
    let world_from_clip =
        (projection.get_clip_from_view() * transform.to_matrix().inverse()).inverse();
    if let Some(hit) = pick(&scene.tree, &scene.bodies, world_from_clip, eye, ndc) {
        joint_click(&mut state, &mut joints, &scene.bodies, hit);
    }
}

/// In grab mode, holding the left mouse on a body picks it up: a spring pulls
/// the clicked point toward a point on the cursor's ray, as far away as it was
/// when clicked. The wheel moves it nearer or farther; letting go drops it,
/// with whatever momentum it has.
fn grab_input(
    buttons: Res<ButtonInput<MouseButton>>,
    mut wheel: MessageReader<bevy::input::mouse::MouseWheel>,
    mut state: ResMut<GrabState>,
    scene: Res<VoxelScene>,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
    windows: Query<&Window>,
) {
    let notches: f32 = wheel.read().map(|e| e.y).sum();
    if !state.enabled {
        return;
    }
    if buttons.just_released(MouseButton::Left) {
        state.held = None;
        return;
    }
    // A held body that was erased away, or removed, is no longer held.
    if let Some(held) = state.held
        && !scene.bodies.iter().any(|b| b.id == held.body)
    {
        state.held = None;
    }

    let Ok((transform, projection)) = camera.single() else {
        return;
    };
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(ndc) = cursor_ndc(window) else {
        return;
    };
    let eye = transform.translation();
    let world_from_clip =
        (projection.get_clip_from_view() * transform.to_matrix().inverse()).inverse();

    if buttons.just_pressed(MouseButton::Left) {
        if let Some(hit) = pick(&scene.tree, &scene.bodies, world_from_clip, eye, ndc)
            && let Target::Body(i) = hit.target
        {
            state.held = Some(Grab::new(&scene.bodies[i], hit.position));
            state.distance = (hit.position - eye).length();
        }
        return;
    }

    if buttons.pressed(MouseButton::Left) && state.held.is_some() {
        state.distance = (state.distance + notches).clamp(2.0, 200.0);
        let target = eye + cursor_ray(world_from_clip, eye, ndc) * state.distance;
        if let Some(held) = state.held.as_mut() {
            held.target = target;
        }
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

#[allow(clippy::too_many_arguments)]
fn brush_input(
    buttons: Res<ButtonInput<MouseButton>>,
    mut wheel: MessageReader<bevy::input::mouse::MouseWheel>,
    mut brush: ResMut<BrushSettings>,
    grab: Res<GrabState>,
    joint: Res<JointState>,
    mut scene: ResMut<VoxelScene>,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
    windows: Query<&Window>,
) {
    // While a body is held the wheel moves it nearer or farther instead.
    for event in wheel.read() {
        if grab.held.is_none() {
            brush.radius = (brush.radius + event.y).clamp(1.0, 32.0);
        }
    }

    // In grab and joint mode the left mouse is the tool's; only the erase is
    // left here.
    let paint = buttons.just_pressed(MouseButton::Left) && !grab.enabled && !joint.enabled;
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
    // Bodies as well as the world: the stroke edits whatever is under the
    // cursor, and only that.
    let Some(hit) = pick(&scene.tree, &scene.bodies, world_from_clip, eye, ndc) else {
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
    stroke(&mut scene, hit.target, centre, brush.radius, material);
}

/// One brush stroke on whatever the cursor was on.
///
/// On the world, an edit is uploaded by range and does not bump the
/// generation, which is what asks for a full rebuild; an erase then detaches
/// whatever it cut free. On a body, the edit changes geometry that is packed
/// whole, so it does.
fn stroke(scene: &mut VoxelScene, target: Target, centre: Vec3, radius: f32, material: MaterialId) {
    match target {
        Target::World if material.is_empty() => erase_and_detach(scene, centre, radius),
        Target::World => apply_brush(scene, centre, radius, material),
        Target::Body(i) => {
            sculpt(&mut scene.bodies, i, centre, radius, material, &scene.materials, MAX_BODIES);
            scene.generation += 1;
        }
    }
}

/// Erases a sphere, then hands whatever it cut free to the physics as bodies.
///
/// The cap is the renderer's: past `MAX_BODIES` a body is uploaded but not
/// marched, so a piece with nowhere to go stays in the world rather than
/// disappearing.
fn erase_and_detach(scene: &mut VoxelScene, centre: Vec3, radius: f32) {
    apply_brush(scene, centre, radius, MaterialId::EMPTY);
    let reach = radius.ceil() as i32 + 1;
    let hit = centre.round().as_ivec3();
    let room = MAX_BODIES.saturating_sub(scene.bodies.len());
    let VoxelScene { tree, materials, bodies, generation, .. } = scene;
    let freed = detach(tree, materials, hit - reach, hit + reach, room);
    if !freed.is_empty() {
        bodies.extend(freed);
        // The body list changed, so the packed buffers must be rebuilt.
        *generation += 1;
    }
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

    /// Erasing the base of the demo scene's column drops it: the erase runs
    /// detachment, and the freed piece arrives as a body.
    #[test]
    fn erasing_a_support_spawns_a_body() {
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut scene = VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![],
        };
        // The demo column stands on the floor at x = 28..36, z = 28..36, from
        // y = 6; a radius-5 sphere at its foot cuts the whole cross-section.
        erase_and_detach(&mut scene, Vec3::new(32.0, 7.0, 32.0), 5.0);
        assert_eq!(scene.bodies.len(), 1, "the column did not come free");
        assert!(scene.bodies[0].mass.mass > 0.0);
        assert_eq!(scene.generation, 2, "the body list changed without asking for a rebuild");
        assert!(
            scene.tree.get(UVec3::new(32, 20, 32)).is_empty()
                && scene.tree.get(UVec3::new(29, 20, 29)).is_empty(),
            "the column is still in the world as well"
        );
    }

    /// A stroke on a body edits the body, not the world behind it, and asks for
    /// a rebuild, because a body's geometry is packed whole.
    #[test]
    fn a_stroke_on_a_body_edits_the_body() {
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut body = demo_body(Vec3::new(10.0, 40.0, 50.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let voxels = body.volume.voxels().len();
        let world = tree.voxels().len();
        let mut scene = VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![body],
        };
        stroke(&mut scene, Target::Body(0), Vec3::new(10.0, 40.0, 50.0), 2.0, MaterialId::EMPTY);
        assert!(scene.bodies[0].volume.voxels().len() < voxels, "the body was not edited");
        assert_eq!(scene.tree.voxels().len(), world, "the world was edited too");
        assert_eq!(scene.generation, 2, "a body edit did not ask for a rebuild");
    }

    /// `G` switches grab mode on and off, and switching it off lets go.
    #[test]
    fn g_toggles_grab_mode() {
        let mut world = World::new();
        world.init_resource::<GrabState>();
        world.init_resource::<JointState>();
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        let toggle = world.register_system(toggle_grab_mode);
        world.run_system(toggle).unwrap();
        assert!(world.resource::<GrabState>().enabled);

        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        world.resource_mut::<GrabState>().held = Some(Grab {
            body: bevox_core::body::BodyId(7),
            anchor: Vec3::ZERO,
            target: Vec3::ZERO,
        });
        world.run_system(toggle).unwrap();
        let state = world.resource::<GrabState>();
        assert!(!state.enabled);
        assert!(state.held.is_none(), "leaving grab mode did not let go");
    }

    /// A held body is pulled by the physics tick.
    #[test]
    fn the_physics_system_pulls_a_held_body() {
        let mut world = World::new();
        let mut time = Time::<()>::default();
        time.advance_by(std::time::Duration::from_secs_f64(1.0 / 64.0));
        world.insert_resource(time);
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let mut body = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let mut grab = Grab::new(&body, body.position);
        grab.target = body.position + Vec3::new(10.0, 0.0, 0.0);
        world.insert_resource(GrabState { enabled: true, held: Some(grab), distance: 10.0 });
        world.init_resource::<Joints>();
        world.insert_resource(VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            bodies: vec![body],
        });
        let physics = world.register_system(physics_system);
        world.run_system(physics).unwrap();
        let v = world.resource::<VoxelScene>().bodies[0].velocity;
        assert!(v.x > 0.5, "the grab did not pull: {v:?}");
    }

    /// Two clicks make a joint: the first on a body sets the pivot, the second
    /// on the world fastens it there.
    #[test]
    fn two_clicks_make_a_joint() {
        let materials = demo_scene().1;
        let mut body = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let bodies = vec![body];
        let mut state = JointState { enabled: true, kind: JointKind::Hinge, pending: None };
        let mut joints = Joints::default();
        let on_body =
            Hit { target: Target::Body(0), position: Vec3::new(20.0, 43.0, 20.0), normal: Vec3::Y };
        joint_click(&mut state, &mut joints, &bodies, on_body);
        assert!(joints.0.is_empty() && state.pending.is_some(), "the first click made a joint");
        let on_world =
            Hit { target: Target::World, position: Vec3::new(20.0, 6.0, 20.0), normal: Vec3::Y };
        joint_click(&mut state, &mut joints, &bodies, on_world);
        assert_eq!(joints.0.len(), 1);
        assert_eq!(joints.0[0].kind, JointKind::Hinge);
        assert!(joints.0[0].b.is_none(), "not fastened to the world");
        assert!(state.pending.is_none());

        // Clicking the same body twice cancels rather than joining it to itself.
        joint_click(&mut state, &mut joints, &bodies, on_body);
        joint_click(&mut state, &mut joints, &bodies, on_body);
        assert_eq!(joints.0.len(), 1, "a body was joined to itself");
        assert!(state.pending.is_none());
    }

    /// `J` and `G` are exclusive: one tool at a time.
    #[test]
    fn joint_and_grab_modes_exclude_each_other() {
        let mut world = World::new();
        world.init_resource::<GrabState>();
        world.init_resource::<JointState>();
        world.resource_mut::<GrabState>().enabled = true;
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyJ);
        world.insert_resource(keys);
        let toggle = world.register_system(toggle_joint_mode);
        world.run_system(toggle).unwrap();
        assert!(world.resource::<JointState>().enabled);
        assert!(!world.resource::<GrabState>().enabled, "grab mode stayed on");

        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        let toggle = world.register_system(toggle_grab_mode);
        world.run_system(toggle).unwrap();
        assert!(world.resource::<GrabState>().enabled);
        assert!(!world.resource::<JointState>().enabled, "joint mode stayed on");
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

        world.init_resource::<GrabState>();
        world.init_resource::<Joints>();
        let physics = world.register_system(physics_system);
        world.run_system(physics).unwrap();

        let scene = world.resource::<VoxelScene>();
        assert_eq!(scene.bodies.len(), 1, "the body below the world was not removed");
        assert_eq!(scene.generation, 2, "removing a body did not ask for a rebuild");
        assert!(scene.bodies[0].velocity.y < 0.0, "the body did not start to fall");
    }
}
