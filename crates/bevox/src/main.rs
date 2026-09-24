mod scenes;

use bevy::prelude::*;
use bevox_core::distance_field::DistanceField;
use bevox_core::material::MaterialId;
use bevox_core::physics::merge::merge;
use bevox_core::physics::sleep::wake_near;
use bevox_core::physics::{GRAVITY, MERGE_AFTER, SLEEP_AFTER};
use bevox_core::physics::detach::detach;
use bevox_core::physics::joint::{Joint, follow};
use bevox_core::physics::sculpt::sculpt;
use bevox_core::physics::solver::step;
use bevox_render::BevoxRenderPlugin;
use bevox_render::camera::{FlyCamera, fly_camera_system};
use bevox_render::cull::{Frustum, world_bound};
use bevox_render::pick::{Target, cursor_ray, pick};
use bevox_render::pipeline::MAX_BODIES;
use bevox_render::upload::{GpuBody, VoxelScene, apply_brush, build_gpu_scene, offset_from_clip};
use scenes::{SceneKind, demo_body};

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
        .init_resource::<Joints>()
        .init_resource::<SceneKind>()
        .add_systems(Startup, setup)
        // Before the rebuild too, not merely before staging. On a frame that
        // rebuilds because the last stroke outgrew the world region, a stroke
        // landing between the rebuild and the staging is drained into an update
        // the render world discards in favour of the snapshot -- which was
        // taken before that stroke. It is then lost until the next rebuild.
        .add_systems(Update, toggle_grab_mode.before(brush_input).before(grab_input))
        .add_systems(
            Update,
            scene_keys.before(build_gpu_scene).before(grab_input).before(brush_input),
        )
        .add_systems(Update, brush_input.after(fly_camera_system).before(build_gpu_scene))
        .add_systems(Update, grab_input.after(fly_camera_system))
        // Fixed timestep: physics must not depend on the frame rate. Bevy runs
        // `FixedUpdate` before `Update`, so a removal's generation bump lands
        // before `build_gpu_scene`.
        .add_systems(FixedUpdate, physics_system)
        .add_systems(Update, drop_body_input.after(fly_camera_system).before(build_gpu_scene))
        // After the physics tick, which runs in `FixedUpdate` ahead of this, and
        // before the rebuild a merge asks for.
        .add_systems(Update, merge_system.after(fly_camera_system).before(build_gpu_scene))
        .run();
}

fn setup(mut commands: Commands) {
    // 2D camera composites the sprite showing the marched image.
    commands.spawn(Camera2d);

    let scene = match std::env::args().nth(1) {
        Some(path) => match bevox_core::vox::load_scene(std::path::Path::new(&path)) {
            Ok((tree, materials)) => {
                info!(
                    "loaded {path}: extent {}, {} arena nodes",
                    tree.extent(),
                    tree.arena().nodes().len()
                );
                scenes::with_falling_body(tree, materials)
            }
            Err(e) => {
                // A bad path is a typo, not a crash: say so and show the demo.
                error!("could not load {path}: {e}");
                scenes::demo()
            }
        },
        None => scenes::demo(),
    };

    // 3D camera exists only to supply view and projection matrices to the
    // shader; it renders nothing itself.
    commands.spawn((
        Camera3d::default(),
        Camera { order: -1, is_active: false, ..default() },
        Transform::from_translation(scene.eye),
        // Yaw and pitch must agree with the intended direction: the fly camera
        // rewrites the transform's rotation from them every frame.
        FlyCamera::looking_at(scene.eye, scene.look_at),
    ));

    let field = DistanceField::build(&scene.tree);
    let fullness = bevox_core::fullness::Fullness::build(&scene.tree);
    commands.insert_resource(VoxelScene {
        tree: scene.tree,
        materials: scene.materials,
        generation: 1,
        field,
        field_dirty: None,
        fullness,
        fullness_dirty: None,
        bodies: scene.bodies,
    });
    commands.insert_resource(Joints(scene.joints));
}

/// `1` loads the demo scene and `2` the joint scene; pressing either again
/// resets it.
///
/// Loading replaces the world, the bodies and the joints, lets go of any grab,
/// asks for a rebuild, and puts the camera where the scene starts it.
fn scene_keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut kind: ResMut<SceneKind>,
    mut scene: ResMut<VoxelScene>,
    mut joints: ResMut<Joints>,
    mut grab: ResMut<GrabState>,
    mut camera: Query<(&mut Transform, &mut FlyCamera), With<Camera3d>>,
    mut windows: Query<&mut Window>,
) {
    let chosen = if keys.just_pressed(KeyCode::Digit1) {
        SceneKind::Demo
    } else if keys.just_pressed(KeyCode::Digit2) {
        SceneKind::Joints
    } else {
        return;
    };
    let built = chosen.build();
    if let Ok((mut transform, mut fly)) = camera.single_mut() {
        transform.translation = built.eye;
        let facing = FlyCamera::looking_at(built.eye, built.look_at);
        fly.yaw = facing.yaw;
        fly.pitch = facing.pitch;
    }
    let field = DistanceField::build(&built.tree);
    let fullness = bevox_core::fullness::Fullness::build(&built.tree);
    let generation = scene.generation + 1;
    *scene = VoxelScene {
        tree: built.tree,
        materials: built.materials,
        generation,
        field,
        field_dirty: None,
        fullness,
        fullness_dirty: None,
        bodies: built.bodies,
    };
    joints.0 = built.joints;
    grab.held = None;
    *kind = chosen;
    if let Ok(mut window) = windows.single_mut() {
        window.title = title(&grab, chosen);
    }
}

/// One physics tick for every body.
fn physics_system(
    time: Res<Time>,
    mut grab: ResMut<GrabState>,
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
        grab.held.as_mut(),
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
    held: Option<Joint>,
    distance: f32,
}

/// `G` switches grab mode on and off. Switching it off lets go, and the window
/// title says which mode is on.
fn toggle_grab_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<GrabState>,
    kind: Res<SceneKind>,
    mut windows: Query<&mut Window>,
) {
    if !keys.just_pressed(KeyCode::KeyG) {
        return;
    }
    state.enabled = !state.enabled;
    state.held = None;
    if let Ok(mut window) = windows.single_mut() {
        window.title = title(&state, *kind);
    }
}

/// The window title: the scene, and the tool in use.
fn title(grab: &GrabState, scene: SceneKind) -> String {
    let mode = if grab.enabled { ", grab mode (G)" } else { "" };
    format!("BEVOX \u{2014} {} scene (1, 2){mode}", scene.name())
}

/// Every joint in the scene.
#[derive(Resource, Default)]
struct Joints(Vec<Joint>);

/// In grab mode, holding the left mouse on a body picks it up: a joint holds
/// the clicked point at a point on the cursor's ray, as far away as it was when
/// clicked, and keeps the body's orientation. The wheel moves it nearer or
/// farther; letting go drops it, with whatever momentum it has.
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
        && !scene.bodies.iter().any(|b| b.id == held.a)
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
            state.held = Some(Joint::grab(&scene.bodies[i], hit.position));
            state.distance = (hit.position - eye).length();
        }
        return;
    }

    if buttons.pressed(MouseButton::Left) && state.held.is_some() {
        state.distance = (state.distance + notches).clamp(2.0, 200.0);
        let target = eye + cursor_ray(world_from_clip, eye, ndc) * state.distance;
        if let Some(held) = state.held.as_mut() {
            held.anchor_b = target;
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

    // In grab mode the left mouse grabs; only the erase is left here.
    let paint = buttons.just_pressed(MouseButton::Left) && !grab.enabled;
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
    // Whatever the edit touched may move now: a sleeper whose support was
    // erased, or one painted on or beside. A voxel past the sphere, for the
    // bodies resting on what changed.
    let reach = Vec3::splat(radius + 1.0);
    wake_near(&mut scene.bodies, centre - reach, centre + reach);
}

/// Merges every body settled out of view back into the world, freeing its
/// slot, and returns how many.
///
/// Only a body that came out of the terrain: merging puts it back where it
/// came from. One spawned outright, like an `F` drop or a scene's own bodies,
/// stays a body however long it sleeps.
///
/// Settled: asleep for `MERGE_AFTER` after the `SLEEP_AFTER` it took to fall
/// asleep. Out of view: its bounding sphere wholly outside `frustum`, so its
/// snap to the world's grid is never seen. Never a body a joint names, or the
/// one the mouse holds. Each merge lowers the distance field over the body's
/// sphere, as painting does: the world gained geometry, and a field left
/// stale would let rays skip it. One rebuild covers them all.
fn merge_settled(
    scene: &mut VoxelScene,
    joints: &[Joint],
    held: Option<&Joint>,
    frustum: &Frustum,
) -> usize {
    let named: Vec<bevox_core::body::BodyId> = joints
        .iter()
        .chain(held)
        .flat_map(|j| std::iter::once(j.a).chain(j.b))
        .collect();
    let mut merged = 0;
    let mut i = 0;
    while i < scene.bodies.len() {
        let body = &scene.bodies[i];
        let settled =
            body.from_terrain && body.asleep && body.still_for >= SLEEP_AFTER + MERGE_AFTER;
        let bound = bevox_core::body::occupied_bounds(&body.volume)
            .map(|local| world_bound(local, &GpuBody::default().placed(body)));
        let out_of_view = bound.is_some_and(|b| !frustum.sees(b));
        if !settled || !out_of_view || named.contains(&body.id) {
            i += 1;
            continue;
        }
        let body = scene.bodies.remove(i);
        let written = merge(&body, &mut scene.tree);
        if let Some(b) = bound {
            scene.lower_field(b.centre, b.radius);
        }
        // The merged voxels are solid world now, wherever the body came to
        // rest, and that is never where the brush last was.
        if let (Some(lo), Some(hi)) = (
            written.iter().copied().reduce(UVec3::min),
            written.iter().copied().reduce(UVec3::max),
        ) {
            scene.world_changed(lo.as_ivec3(), hi.as_ivec3());
        }
        merged += 1;
    }
    if merged > 0 {
        // The body list and the world both changed: rebuild everything.
        scene.generation += 1;
    }
    merged
}

/// Runs `merge_settled` against what the camera sees now.
fn merge_system(
    mut scene: ResMut<VoxelScene>,
    joints: Res<Joints>,
    grab: Res<GrabState>,
    camera: Query<(&GlobalTransform, &Projection), With<Camera3d>>,
) {
    let Ok((transform, projection)) = camera.single() else {
        return;
    };
    let offset = offset_from_clip(transform.rotation(), projection.get_clip_from_view());
    let frustum = Frustum::from_camera(offset, offset.inverse(), transform.translation());
    // Through a plain borrow, so a frame with nothing to merge does not mark
    // the scene changed.
    if scene.bodies.iter().any(|b| b.asleep && b.still_for >= SLEEP_AFTER + MERGE_AFTER) {
        merge_settled(&mut scene, &joints.0, grab.held.as_ref(), &frustum);
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
    let VoxelScene { tree, materials, .. } = scene;
    let freed = detach(tree, materials, hit - reach, hit + reach, room);
    if freed.is_empty() {
        return;
    }
    // A freed piece is gone from the world, and it can reach a long way past
    // the brush -- the bridge that falls when its one support is erased. The
    // brush recounted its own neighbourhood and nothing else, so each piece's
    // own box is recounted here or the air it left keeps occluding.
    for boxed in freed.iter().filter_map(world_box).collect::<Vec<_>>() {
        scene.world_changed(boxed.0, boxed.1);
    }
    scene.bodies.extend(freed);
    // The body list changed, so the packed buffers must be rebuilt.
    scene.generation += 1;
}

/// The world box a body covers, in voxels, wide enough to hold it at any angle.
///
/// The sphere is the one the culling already builds, so a body's reach has a
/// single spelling.
fn world_box(body: &bevox_core::body::Body) -> Option<(IVec3, IVec3)> {
    let local = bevox_core::body::occupied_bounds(&body.volume)?;
    let b = world_bound(local, &GpuBody::default().placed(body));
    Some((
        (b.centre - b.radius).floor().as_ivec3(),
        (b.centre + b.radius).ceil().as_ivec3(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenes::demo_scene;

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
        let fullness = bevox_core::fullness::Fullness::build(&tree);
        let mut scene = VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            fullness,
            fullness_dirty: None,
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
        // The column reaches far above the brush that cut its foot, so this is
        // the case the brush's own recount cannot cover.
        fullness_describes_the_world(&scene, "a detachment");
    }

    /// A stroke on a body edits the body, not the world behind it, and asks for
    /// a rebuild, because a body's geometry is packed whole.
    #[test]
    fn a_stroke_on_a_body_edits_the_body() {
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let fullness = bevox_core::fullness::Fullness::build(&tree);
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
            fullness,
            fullness_dirty: None,
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
        world.init_resource::<SceneKind>();
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        let toggle = world.register_system(toggle_grab_mode);
        world.run_system(toggle).unwrap();
        assert!(world.resource::<GrabState>().enabled);

        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyG);
        world.insert_resource(keys);
        let mut body = demo_body(Vec3::ZERO, Quat::IDENTITY);
        assert!(body.recompute(&demo_scene().1));
        world.resource_mut::<GrabState>().held = Some(Joint::grab(&body, Vec3::ZERO));
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
        let fullness = bevox_core::fullness::Fullness::build(&tree);
        let mut body = demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY);
        assert!(body.recompute(&materials));
        let mut grab = Joint::grab(&body, body.position);
        grab.anchor_b = body.position + Vec3::new(10.0, 0.0, 0.0);
        world.insert_resource(GrabState { enabled: true, held: Some(grab), distance: 10.0 });
        world.init_resource::<Joints>();
        world.insert_resource(VoxelScene {
            tree,
            materials,
            generation: 1,
            field,
            field_dirty: None,
            fullness,
            fullness_dirty: None,
            bodies: vec![body],
        });
        let physics = world.register_system(physics_system);
        world.run_system(physics).unwrap();
        let v = world.resource::<VoxelScene>().bodies[0].velocity;
        assert!(v.x > 0.5, "the grab did not pull: {v:?}");
    }

    /// `2` loads the joint scene, and pressing it again resets it; `1` goes back
    /// to the demo. Each load asks for a rebuild and lets go of any grab.
    #[test]
    fn number_keys_load_and_reset_scenes() {
        let mut world = World::new();
        let demo = scenes::demo();
        let field = DistanceField::build(&demo.tree);
        let fullness = bevox_core::fullness::Fullness::build(&demo.tree);
        let held = Joint::grab(&demo.bodies[0], demo.bodies[0].position);
        world.insert_resource(VoxelScene {
            tree: demo.tree,
            materials: demo.materials,
            generation: 1,
            field,
            field_dirty: None,
            fullness,
            fullness_dirty: None,
            bodies: demo.bodies,
        });
        world.insert_resource(GrabState { enabled: true, held: Some(held), distance: 10.0 });
        world.init_resource::<Joints>();
        world.init_resource::<SceneKind>();
        let keys = world.register_system(scene_keys);
        let press = |world: &mut World, key: KeyCode| {
            let mut input = ButtonInput::<KeyCode>::default();
            input.press(key);
            world.insert_resource(input);
            world.run_system(keys).unwrap();
        };

        press(&mut world, KeyCode::Digit2);
        assert_eq!(world.resource::<VoxelScene>().bodies.len(), 12);
        assert_eq!(world.resource::<VoxelScene>().generation, 2, "loading did not ask for a rebuild");
        assert_eq!(world.resource::<Joints>().0.len(), 13);
        assert!(world.resource::<GrabState>().held.is_none(), "a grab outlived its scene");
        assert_eq!(*world.resource::<SceneKind>(), SceneKind::Joints);

        let fresh = scenes::joints().bodies[0].position;
        world.resource_mut::<VoxelScene>().bodies[0].position += Vec3::splat(5.0);
        press(&mut world, KeyCode::Digit2);
        assert_eq!(world.resource::<VoxelScene>().bodies[0].position, fresh, "2 did not reset");
        assert_eq!(world.resource::<VoxelScene>().generation, 3);

        press(&mut world, KeyCode::Digit1);
        assert_eq!(world.resource::<VoxelScene>().bodies.len(), 1);
        assert!(world.resource::<Joints>().0.is_empty());
        assert_eq!(*world.resource::<SceneKind>(), SceneKind::Demo);
    }

    /// A frustum for a camera at `eye` looking at `target`.
    fn frustum_looking(eye: Vec3, target: Vec3) -> Frustum {
        let view = Mat4::look_at_rh(Vec3::ZERO, target - eye, Vec3::Y);
        let projection = Mat4::perspective_rh(0.9, 16.0 / 9.0, 0.1, 500.0);
        let offset = (projection * view).inverse();
        Frustum::from_camera(offset, offset.inverse(), eye)
    }

    /// Fullness must still describe the world it was built from.
    ///
    /// Ambient occlusion has no safe direction to be stale in, and the two
    /// paths that change the world away from the brush -- a piece detaching, a
    /// body merging -- are the ones that forget. A mismatch here is a dark
    /// crease in open air, or a merged pile that lights as if it were not
    /// there, and it lasts until the scene is rebuilt.
    fn fullness_describes_the_world(scene: &VoxelScene, what: &str) {
        let fresh = bevox_core::fullness::Fullness::build(&scene.tree);
        let wrong = scene
            .fullness
            .cells()
            .iter()
            .zip(fresh.cells())
            .filter(|(have, want)| have != want)
            .count();
        assert_eq!(wrong, 0, "{wrong} fullness cells are stale after {what}");
    }

    /// The demo scene with one brick cube centred at `centre`, asleep long
    /// enough to merge.
    fn settled_scene(centre: Vec3) -> VoxelScene {
        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let fullness = bevox_core::fullness::Fullness::build(&tree);
        let mut body = demo_body(centre, Quat::IDENTITY);
        assert!(body.recompute(&materials));
        // As if detachment had cut it out of the terrain.
        body.from_terrain = true;
        body.asleep = true;
        body.still_for = SLEEP_AFTER + MERGE_AFTER;
        VoxelScene { tree, materials, generation: 1, field, field_dirty: None,
            fullness, fullness_dirty: None, bodies: vec![body] }
    }

    /// Where the settled body sleeps: in open air, where the distance field
    /// reads far from anything, so a field left stale by the merge would
    /// over-estimate. On the floor it already reads zero, and would hide that.
    const ALOFT: Vec3 = Vec3::new(50.0, 40.0, 12.0);

    /// A settled body out of view merges: its slot is free, the world holds its
    /// voxels, a rebuild is asked for, and the distance field is nowhere above
    /// one built fresh from the new world.
    #[test]
    fn a_settled_body_out_of_view_merges_into_the_world() {
        let mut scene = settled_scene(ALOFT);
        let before = scene.tree.voxels().len();
        let cells = scene.bodies[0].volume.voxels().len();
        let away = frustum_looking(Vec3::new(50.0, 30.0, 40.0), Vec3::new(50.0, 30.0, 100.0));
        assert_eq!(merge_settled(&mut scene, &[], None, &away), 1);
        assert!(scene.bodies.is_empty(), "the body kept its slot");
        assert_eq!(scene.generation, 2, "the merge did not ask for a rebuild");
        assert_eq!(scene.tree.voxels().len(), before + cells, "the world did not gain the body's voxels");
        assert_eq!(scene.tree.get(UVec3::new(50, 40, 12)), MaterialId(2), "the body's middle is not brick");
        let fresh = DistanceField::build(&scene.tree);
        let over = scene.field.cells().iter().zip(fresh.cells()).filter(|(have, want)| have > want).count();
        assert_eq!(over, 0, "{over} field cells over-estimate the merged world, so rays would skip it");
        fullness_describes_the_world(&scene, "a merge");
    }

    /// No merge in view, while jointed, while grabbed, before its time, or awake.
    #[test]
    fn a_body_that_should_stay_does_not_merge() {
        let away = frustum_looking(Vec3::new(50.0, 30.0, 40.0), Vec3::new(50.0, 30.0, 100.0));
        let at = frustum_looking(Vec3::new(50.0, 30.0, 40.0), ALOFT);

        let mut scene = settled_scene(ALOFT);
        assert_eq!(merge_settled(&mut scene, &[], None, &at), 0, "a body in view merged");

        let mut scene = settled_scene(ALOFT);
        let pin = scene.bodies[0].position;
        let joint = Joint::new(
            &scene.bodies[0], None, bevox_core::physics::joint::Linear::Point,
            bevox_core::physics::joint::Angular::Free, pin, pin, Vec3::Y,
        );
        assert_eq!(merge_settled(&mut scene, &[joint], None, &away), 0, "a jointed body merged");

        let mut scene = settled_scene(ALOFT);
        let held = Joint::grab(&scene.bodies[0], scene.bodies[0].position);
        assert_eq!(merge_settled(&mut scene, &[], Some(&held), &away), 0, "a grabbed body merged");

        let mut scene = settled_scene(ALOFT);
        scene.bodies[0].still_for = SLEEP_AFTER + MERGE_AFTER - 0.1;
        assert_eq!(merge_settled(&mut scene, &[], None, &away), 0, "a body merged before its time");

        let mut scene = settled_scene(ALOFT);
        scene.bodies[0].asleep = false;
        assert_eq!(merge_settled(&mut scene, &[], None, &away), 0, "an awake body merged");

        // Spawned, not cut out of the terrain: it stays a body forever.
        let mut scene = settled_scene(ALOFT);
        scene.bodies[0].from_terrain = false;
        assert_eq!(merge_settled(&mut scene, &[], None, &away), 0, "a body that was never terrain merged");
        assert_eq!(scene.generation, 1, "no merge, yet a rebuild was asked for");
    }

    /// What the app spawns is not terrain, so it never merges; what an erase
    /// cuts loose is.
    #[test]
    fn only_bodies_cut_from_the_terrain_may_merge() {
        assert!(!demo_body(Vec3::new(20.0, 40.0, 20.0), Quat::IDENTITY).from_terrain, "a spawned body claims to be terrain");

        let (tree, materials) = demo_scene();
        let field = DistanceField::build(&tree);
        let fullness = bevox_core::fullness::Fullness::build(&tree);
        let mut scene =
            VoxelScene { tree, materials, generation: 1, field, field_dirty: None,
                fullness, fullness_dirty: None, bodies: vec![] };
        // The demo column stands at x = 28..36, z = 28..36 from y = 6; cutting
        // its foot frees the whole thing.
        erase_and_detach(&mut scene, Vec3::new(32.0, 7.0, 32.0), 5.0);
        assert_eq!(scene.bodies.len(), 1, "the column did not come free");
        assert!(scene.bodies[0].from_terrain, "a body cut out of the terrain does not know it");
    }

    /// An erase under a sleeping body wakes it.
    #[test]
    fn an_erase_under_a_sleeper_wakes_it() {
        // On the floor, whose top is at 6.
        let mut scene = settled_scene(Vec3::new(50.0, 9.0, 12.0));
        stroke(&mut scene, Target::World, Vec3::new(50.0, 5.0, 12.0), 3.0, MaterialId::EMPTY);
        assert!(!scene.bodies[0].asleep, "the sleeper slept through its support being erased");
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
        let fullness = bevox_core::fullness::Fullness::build(&tree);
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
            fullness,
            fullness_dirty: None,
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
