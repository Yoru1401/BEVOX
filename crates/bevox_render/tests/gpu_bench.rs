//! Headless timing. No window and no surface, so nothing here is vsync-locked.
//!
//! Frame rate in the app cannot measure traversal cost: it is capped at 60, and
//! a scene that renders nothing at all reports exactly the same number as one
//! that renders correctly. That is not hypothetical — it is how a ray budget bug
//! survived a frame-rate check in the previous milestone.
//!
//! Run with `--release`: the scene is built on the CPU, and an unoptimised build
//! spends longer constructing it than the GPU spends rendering it.

use bevox_core::contree::Contree;
use bevox_core::gpu::GpuVolume;
use bevox_core::material::MaterialId;
use bevox_render::upload::march_flags;
use glam::{Mat4, UVec3, Vec3};

mod common;
use common::*;

/// Dispatches discarded before timing, so shader compilation and first touch of
/// the buffers do not land in the measurement.
const WARMUP: u32 = 10;
/// Dispatches per timed batch. Batching amortises submit overhead.
const BATCH: u32 = 30;
/// A/B/A rounds. The two A readings bracket B, so drift is visible rather than
/// being silently attributed to the change under test.
const ROUNDS: u32 = 3;

/// A scene with the shape that hurts: a large floor, columns rising from it, and
/// open space between, at an extent deep enough to need real traversal.
///
/// Generated rather than loaded. `assets/` is gitignored, and a benchmark that
/// needs a file nobody has is a benchmark nobody runs.
pub fn bench_scene() -> (Contree, u32) {
    let extent = 1024u32;
    let mut voxels = Vec::new();

    for z in 0..extent {
        for x in 0..extent {
            for y in 0..2 {
                voxels.push((UVec3::new(x, y, z), MaterialId(1)));
            }
        }
    }
    for cz in 0..6u32 {
        for cx in 0..6u32 {
            let ox = 96 + cx * 160;
            let oz = 96 + cz * 160;
            for y in 2..82 {
                for dz in 0..16 {
                    for dx in 0..16 {
                        voxels.push((UVec3::new(ox + dx, y, oz + dz), MaterialId(2)));
                    }
                }
            }
        }
    }

    (Contree::from_voxels(extent, &voxels), extent)
}

/// Milliseconds per dispatch, averaged over a batch.
pub fn time_dispatches(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    prepared: &Prepared,
    iterations: u32,
) -> f32 {
    for _ in 0..WARMUP {
        prepared.dispatch(device, queue);
    }
    device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");

    let started = std::time::Instant::now();
    for _ in 0..iterations {
        prepared.dispatch(device, queue);
    }
    device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
    started.elapsed().as_secs_f32() * 1000.0 / iterations as f32
}

/// Runs baseline, variant, baseline, interleaved, and returns the medians.
///
/// Both baseline readings are returned so drift is visible. A variant that
/// "wins" by less than the spread between them has not been shown to win.
pub fn compare_aba(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    baseline: &Prepared,
    variant: &Prepared,
) -> (f32, f32, f32) {
    aba(baseline, variant, |p| time_dispatches(device, queue, p, BATCH))
}

/// A/B/A with any clock: `time` reads one configuration once.
fn aba(
    baseline: &Prepared,
    variant: &Prepared,
    mut time: impl FnMut(&Prepared) -> f32,
) -> (f32, f32, f32) {
    let mut a1 = Vec::new();
    let mut b = Vec::new();
    let mut a2 = Vec::new();
    for _ in 0..ROUNDS {
        a1.push(time(baseline));
        b.push(time(variant));
        a2.push(time(baseline));
    }
    (median(&mut a1), median(&mut b), median(&mut a2))
}

fn median(v: &mut [f32]) -> f32 {
    v.sort_by(f32::total_cmp);
    v[v.len() / 2]
}

/// The camera position the app slowed down at: close to geometry, looking along
/// the floor between columns.
pub fn bench_camera(extent: u32) -> (Vec3, Mat4) {
    let half = extent as f32 * 0.5;
    let eye = Vec3::new(half, 40.0, half - 120.0);
    let view = Mat4::look_at_rh(Vec3::ZERO, Vec3::new(half, 40.0, half) - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1280.0 / 720.0, 0.1, 20_000.0);
    (eye, (projection * view).inverse())
}

#[test]
fn record_the_baseline() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let built = std::time::Instant::now();
    let (tree, extent) = bench_scene();
    println!(
        "scene: extent {extent}, {} arena nodes, {} voxel bytes, built in {:.2}s",
        tree.arena().nodes().len(),
        tree.arena().voxels().len(),
        built.elapsed().as_secs_f32()
    );

    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let prepared = Prepared::new(
        &device,
        &shader,
        "march",
        &tree,
        offset_from_clip,
        eye,
        1280,
        720,
        0,
        &[],
    );

    let ms = time_dispatches(&device, &queue, &prepared, BATCH);
    println!("baseline: {ms:.2} ms per frame at 1280x720");

    assert!(ms > 0.0, "the timer returned nothing");
}

/// Every optimisation, and every combination the app might ship, against the
/// scan — interleaved in one process on one device.
///
/// A/B/A, not A-then-B: the two baseline readings bracket each variant, so
/// thermal drift and clock ramping are visible rather than being attributed to
/// the change. A win smaller than the spread between them is not a win. This is
/// also why the numbers here do not match a baseline recorded in an earlier
/// process — only the comparison within one run means anything.
#[test]
fn optimisations_are_measured_against_the_baseline() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };

    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let make = |flags: u32| {
        Prepared::new(
            &device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720, flags,
            &[],
        )
    };
    let baseline = make(march_flags::NONE);

    for (name, flags) in [
        ("dda", march_flags::DDA),
        ("mask", march_flags::MASK_FILTER),
        ("dda+mask", march_flags::DDA | march_flags::MASK_FILTER),
        ("beam", march_flags::BEAM),
        ("dda+mask+beam", march_flags::DDA | march_flags::MASK_FILTER | march_flags::BEAM),
        ("field", march_flags::DISTANCE_FIELD),
        // `DEFAULT`, whatever it currently holds -- today that is all four
        // optimisations plus `BODIES`, `CULL_BODIES` and `BODY_RECT`, which a
        // body-free scene never runs.
        ("default", march_flags::DEFAULT),
    ] {
        let variant = make(flags);
        let (a1, b, a2) = compare_aba(&device, &queue, &baseline, &variant);
        let drift = (a1 - a2).abs();
        let scan = (a1 + a2) * 0.5;
        let gain = scan - b;
        println!(
            "{name:>13}: {b:6.2} ms vs scan {a1:.2}/{a2:.2} (drift {drift:.2})  gain {gain:6.2} ms ({:5.1}%) {}{}",
            gain / scan * 100.0,
            if gain.abs() > drift { "" } else { "<- within drift, not a result" },
            gpu_time_note(&device, &queue, &variant),
        );
        assert!(b > 0.0, "{name}: the timer returned nothing");
    }
}

/// The composed .vox scenes, at the two camera positions milestone 8 was
/// argued from. Ignored by default: it needs the gitignored `assets/`, and it
/// spends minutes on scenes at extent 4096.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench -- --ignored
/// --nocapture`.
///
/// Measured headlessly rather than by reading the app's frame counter. The
/// counter is capped at 60, so a framed-back reading can only say "at least as
/// fast as before" -- and a scene rendering nothing at all reports exactly the
/// same 60, which is how a ray budget bug survived a frame-rate check once
/// already.
#[test]
#[ignore]
fn the_real_scenes_are_measured_with_the_defaults() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let dir = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets"));
    let Ok(entries) = std::fs::read_dir(dir) else {
        eprintln!("no assets directory, skipping");
        return;
    };
    let mut files: Vec<_> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("vox")))
        .collect();
    files.sort();

    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    for path in files {
        let name = path.file_stem().unwrap_or_default().to_string_lossy().to_string();
        let Ok((tree, _materials)) = bevox_core::vox::load_scene(&path) else {
            println!("{name}: failed to load, skipping");
            continue;
        };
        let extent = tree.extent();

        for (label, (eye, offset_from_clip)) in
            [("framed", framed_camera(extent)), ("close", close_camera(&tree, extent))]
        {
            let baseline = Prepared::new(
                &device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720,
                march_flags::NONE, &[],
            );
            let variant = Prepared::new(
                &device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720,
                march_flags::DEFAULT, &[],
            );
            let (a1, b, a2) = compare_aba(&device, &queue, &baseline, &variant);
            let scan = (a1 + a2) * 0.5;
            println!(
                "{name:>22} {label:>6} (extent {extent}): scan {scan:7.2} ms -> {b:7.2} ms  \
                 {:5.1}% (drift {:.2})",
                (scan - b) / scan * 100.0,
                (a1 - a2).abs()
            );
        }
    }
}

/// The whole volume in view, the way the app frames a scene on load.
fn framed_camera(extent: u32) -> (Vec3, Mat4) {
    let e = extent as f32;
    let centre = Vec3::splat(e * 0.5);
    let eye = centre + Vec3::new(0.6, 0.5, 1.0).normalize() * e * 1.1;
    let view = Mat4::look_at_rh(Vec3::ZERO, centre - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1280.0 / 720.0, 0.1, e * 8.0);
    (eye, (projection * view).inverse())
}

/// Thirty voxels in front of the first surface the framed camera sees.
///
/// A fixed distance in voxels, not a fraction of the extent: "close" has to mean
/// the same angular size for a cathedral at extent 4096 and a model at 256, or
/// the large scenes get measured from further away and look cheap.
fn close_camera(tree: &bevox_core::contree::Contree, extent: u32) -> (Vec3, Mat4) {
    let e = extent as f32;
    let centre = Vec3::splat(e * 0.5);
    let (eye, _) = framed_camera(extent);
    let dir = (centre - eye).normalize();
    let mut stats = bevox_core::march::MarchStats::default();
    let surface = match bevox_core::march::march(
        tree,
        glam::Affine3A::IDENTITY,
        eye,
        dir,
        e * 8.0,
        false,
        &mut stats,
    ) {
        Some(hit) => eye + dir * hit.t,
        // An empty line of sight is still a valid measurement; stand at the centre.
        None => centre,
    };
    let close = surface - dir * 30.0;
    let view = Mat4::look_at_rh(Vec3::ZERO, (surface + dir * e * 0.25) - close, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1280.0 / 720.0, 0.1, e * 8.0);
    (close, (projection * view).inverse())
}

/// What an edit actually costs: the CPU rewrite, and the bytes it uploads.
///
/// The claim this milestone makes is "editing without full re-upload", and the
/// number that supports it is the ratio of bytes written to bytes the scene
/// occupies. Wall-clock time for the write is dominated by queue submission at
/// these sizes, so bytes are the honest measure and are reported as such.
#[test]
#[ignore]
fn an_edit_uploads_a_fraction_of_the_scene() {
    let (tree, extent) = bench_scene();
    let field = bevox_core::distance_field::DistanceField::build(&tree);
    let mut scene = bevox_render::upload::VoxelScene {
        tree,
        materials: bevox_core::material::MaterialTable::new(),
        generation: 1,
        field,
        field_dirty: None,
        bodies: Vec::new(),
    };
    scene.tree.arena_mut().clear_dirty();

    let whole_nodes = scene.tree.arena().nodes().len() * 16;
    let whole_voxels = scene.tree.arena().voxels().len();
    // Raw cell count, matching how `whole_voxels` counts pre-packing bytes
    // rather than the four-cells-per-word form the GPU buffer actually holds.
    let whole_field = scene.field.cells().len();
    let whole = whole_nodes + whole_voxels + whole_field;
    println!(
        "scene: extent {extent}, {} node bytes, {} voxel bytes, {} field bytes",
        whole_nodes, whole_voxels, whole_field
    );

    for radius in [2.0f32, 8.0, 32.0] {
        let centre = Vec3::splat(extent as f32 * 0.5);
        let started = std::time::Instant::now();
        // `apply_brush`, not `tree.apply_sphere` directly: only `apply_brush`
        // lowers the field and marks it dirty, and the field is the largest
        // of the three components below. Painting through `apply_sphere`
        // alone would leave `update.field` empty and this benchmark would
        // report a byte total that omits the biggest write entirely.
        bevox_render::upload::apply_brush(
            &mut scene,
            centre,
            radius,
            bevox_core::material::MaterialId(3),
        );
        let edit_ms = started.elapsed().as_secs_f32() * 1000.0;

        let staged = std::time::Instant::now();
        let update = bevox_render::upload::stage_scene_update(&mut scene);
        let stage_ms = staged.elapsed().as_secs_f32() * 1000.0;

        let node_bytes: usize = update.nodes.iter().map(|w| w.nodes.len() * 16).sum();
        let voxel_bytes: usize = update.voxels.iter().map(|w| w.words.len() * 4).sum();
        let field_bytes: usize = update.field.iter().map(|w| w.words.len() * 4).sum();
        let total = node_bytes + voxel_bytes + field_bytes;
        println!(
            "radius {radius:>5}: edit {edit_ms:6.2} ms, stage {stage_ms:5.2} ms, upload \
             nodes {node_bytes:>9} + voxels {voxel_bytes:>9} + field {field_bytes:>9} \
             = {total:>9} bytes ({:.3}% of the scene) in {} ranges",
            total as f64 / whole as f64 * 100.0,
            update.nodes.len() + update.voxels.len() + update.field.len()
        );
        assert!(
            total < whole,
            "radius {radius} uploaded the whole scene; the dirty ranges are not narrowing anything"
        );
    }
}

/// One scene's clone cost, A/B/A interleaved, printed as a row.
fn measure_clone(name: &str, scene: &bevox_render::upload::GpuSceneData) {
    use bevox_render::upload::scene_copy_is_stale;

    let payload = scene.nodes.len() * size_of::<bevox_core::gpu::GpuNode>()
        + scene.voxels.len() * 4
        + scene.palette.len() * 16
        + scene.direction_masks.len() * 8;

    // Enough iterations to be stable, few enough that a 30 MB payload does not
    // turn one row into a minute.
    let iterations = if payload > 8 * 1024 * 1024 { 60 } else { 400 };

    let mut a1 = Vec::new();
    let mut b = Vec::new();
    let mut a2 = Vec::new();
    for _ in 0..3 {
        a1.push(time_calls(iterations, || {
            std::hint::black_box(scene.clone());
        }));
        b.push(time_calls(iterations, || {
            // The new path on a quiet frame: ask whether the copy is stale,
            // find that it is not, and return without touching the payload.
            std::hint::black_box(scene_copy_is_stale(false, Some(1), 1));
        }));
        a2.push(time_calls(iterations, || {
            std::hint::black_box(scene.clone());
        }));
    }

    let a1 = median_of(&mut a1);
    let b = median_of(&mut b);
    let a2 = median_of(&mut a2);
    let drift = (a1 - a2).abs();
    let saved = (a1 + a2) * 0.5 - b;

    println!(
        "{name:>22} (extent {:>4}, {:>8.2} MB): clone {a1:6.3}/{a2:6.3} ms  skip {b:.4} ms  saved {saved:6.3} ms ({:5.1}% of a 16.7 ms frame, drift {drift:.3}) {}",
        scene.extent,
        payload as f64 / (1024.0 * 1024.0),
        saved / 16.667 * 100.0,
        if saved > drift { "" } else { "<- within drift" }
    );
}

/// Milliseconds per call, averaged over a batch.
fn time_calls(iterations: u32, mut f: impl FnMut()) -> f32 {
    let started = std::time::Instant::now();
    for _ in 0..iterations {
        f();
    }
    started.elapsed().as_secs_f32() * 1000.0 / iterations as f32
}

fn median_of(v: &mut [f32]) -> f32 {
    v.sort_by(f32::total_cmp);
    v[v.len() / 2]
}

/// What the per-frame scene clone cost, and what skipping it saves.
///
/// `ExtractResourcePlugin` copied the whole `GpuSceneData` into the render
/// world every frame; `extract_gpu_scene` copies it only when it changed. This
/// measures the two sides of that on a quiet frame -- one where nothing was
/// edited, which is almost every frame.
///
/// It measures the clone in isolation, NOT end-to-end frame time. That is a
/// deliberate limit and the number should not be read as a frame-rate claim:
/// the app is vsync-capped at 60, and this project has already been burned once
/// by a frame counter that read a healthy 60 while the renderer drew nothing.
/// What this does show is the CPU work the extract schedule no longer does.
///
/// A/B/A interleaved in one process, medians reported with the drift between
/// the two baseline readings, per this project's measurement rule.
#[test]
#[ignore]
fn the_scene_clone_is_measured_against_not_cloning() {
    use bevox_core::material::MaterialTable;
    use bevox_render::upload::{GpuSceneData, WorldRegion, gpu_direction_masks};

    let (tree, extent) = bench_scene();
    let volume = GpuVolume::from_contree(&tree);
    let field = bevox_core::distance_field::DistanceField::build(&tree);
    let scene = GpuSceneData {
        // First, while `volume.voxels` is still here to measure.
        world_region: WorldRegion::around(
            volume.buffer_nodes().len() as u32,
            volume.voxels.len() as u32,
        ),
        nodes: volume.buffer_nodes(),
        voxels: volume.voxels,
        bodies: Vec::new(),
        body_local_bounds: Vec::new(),
        palette: MaterialTable::new().to_gpu(),
        direction_masks: gpu_direction_masks(),
        depth: tree.depth(),
        extent,
        field_edge: field.edge(),
        distance_field: bevox_render::upload::pack_field(&field),
        generation: 1,
    };

    measure_clone("bench_scene", &scene);

    // The composed scenes are where this actually bites: an order of magnitude
    // more payload than the generated benchmark. Skipped when assets/ is absent,
    // which is normal -- it is gitignored.
    let dir = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets"));
    if let Ok(entries) = std::fs::read_dir(dir) {
        let mut files: Vec<_> = entries
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("vox")))
            .collect();
        files.sort();
        for path in files {
            let name = path.file_stem().unwrap_or_default().to_string_lossy().to_string();
            let Ok((tree, materials)) = bevox_core::vox::load_scene(&path) else {
                continue;
            };
            let volume = GpuVolume::from_contree(&tree);
            let field = bevox_core::distance_field::DistanceField::build(&tree);
            let real = GpuSceneData {
                world_region: WorldRegion::around(
                    volume.buffer_nodes().len() as u32,
                    volume.voxels.len() as u32,
                ),
                nodes: volume.buffer_nodes(),
                voxels: volume.voxels,
                bodies: Vec::new(),
                body_local_bounds: Vec::new(),
                palette: materials.to_gpu(),
                direction_masks: gpu_direction_masks(),
                depth: tree.depth(),
                extent: tree.extent(),
                field_edge: field.edge(),
                distance_field: bevox_render::upload::pack_field(&field),
                generation: 1,
            };
            measure_clone(&name, &real);
        }
    }

    assert!(scene.nodes.len() > 1, "the benchmark scene is empty");
}

/// What the GPU itself says the dispatches took, as a suffix for a bench row.
///
/// The wall-clock figures above are submit-to-poll over a batch, so they carry
/// queue submission and driver overhead that no shader change can move. These
/// come from timestamps written at each pass boundary, so they are the shader's
/// own time -- narrower, and the honest number when the question is "did the
/// traversal get faster" rather than "did the frame get cheaper".
///
/// Empty when the adapter has no timestamp support, which is not a failure.
fn gpu_time_note(device: &wgpu::Device, queue: &wgpu::Queue, prepared: &Prepared) -> String {
    // One untimed dispatch first: the first run of a pipeline pays for shader
    // compilation and first touch of every buffer, which is not its steady cost.
    prepared.dispatch(device, queue);
    device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");

    let Some(t) = prepared.dispatch_timed(device, queue) else {
        return String::new();
    };
    if t.beam > 0.0 {
        format!("  [gpu {:.2} ms = beam {:.2} + main {:.2}]", t.total(), t.beam, t.main)
    } else {
        format!("  [gpu {:.2} ms]", t.main)
    }
}

/// The GPU's own clock, reported next to the wall-clock numbers it qualifies.
///
/// Wall-clock over a batch includes submit and driver overhead; a timestamp
/// pair around each pass does not. Printing both is the point -- when they
/// disagree, the difference is the overhead, and that is worth seeing rather
/// than averaging away.
#[test]
#[ignore]
fn the_dispatches_are_timed_by_the_gpu() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    if !timestamps_supported(&device) {
        eprintln!("adapter has no TIMESTAMP_QUERY, skipping");
        return;
    }

    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    println!("scene: extent {extent}, 1280x720, close to geometry");
    for (name, flags) in [
        ("scan", march_flags::NONE),
        ("dda", march_flags::DDA),
        ("mask", march_flags::MASK_FILTER),
        ("beam", march_flags::BEAM),
        ("dda+mask+beam", march_flags::DDA | march_flags::MASK_FILTER | march_flags::BEAM),
        ("field", march_flags::DISTANCE_FIELD),
        ("default", march_flags::DEFAULT),
    ] {
        let prepared = Prepared::new(
            &device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720, flags,
            &[],
        );
        // Discard the first dispatch: it pays for pipeline compilation.
        prepared.dispatch(&device, &queue);
        device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");

        // Several readings; the GPU clock is steadier than wall clock but the
        // card still ramps, so report the median rather than one sample.
        let mut totals: Vec<f32> = (0..7)
            .map(|_| prepared.dispatch_timed(&device, &queue).expect("timestamps").total())
            .collect();
        let mut beams: Vec<f32> = (0..7)
            .map(|_| prepared.dispatch_timed(&device, &queue).expect("timestamps").beam)
            .collect();

        let total = median_of(&mut totals);
        let beam = median_of(&mut beams);
        if beam > 0.0 {
            println!("{name:>13}: {total:7.3} ms total  (beam {beam:.3} + main {:.3})", total - beam);
        } else {
            println!("{name:>13}: {total:7.3} ms total");
        }
    }
}

/// What building the distance field costs at load.
///
/// The sweep is 13 neighbours x 2 passes over every cell, and at extent 4096
/// that is 16.7M cells. Scenes already take seconds to compose, so this is
/// worth knowing rather than assuming: it runs once per load and again on any
/// rebuild that outgrows the buffers.
#[test]
#[ignore]
fn building_the_field_is_timed() {
    let dir = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets"));
    let Ok(entries) = std::fs::read_dir(dir) else {
        eprintln!("no assets directory, skipping");
        return;
    };
    let mut files: Vec<_> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("vox")))
        .collect();
    files.sort();

    for path in files {
        let name = path.file_stem().unwrap_or_default().to_string_lossy().to_string();
        let Ok((tree, _)) = bevox_core::vox::load_scene(&path) else {
            continue;
        };
        let started = std::time::Instant::now();
        let field = bevox_core::distance_field::DistanceField::build(&tree);
        let ms = started.elapsed().as_secs_f32() * 1000.0;
        let edge = field.edge();
        println!(
            "{name:>22} (extent {:>4}): field {edge}^3 = {:>9} cells, built in {ms:8.1} ms, \
             {:.1} MB",
            tree.extent(),
            field.cells().len(),
            field.cells().len() as f64 / (1024.0 * 1024.0),
        );
    }
}

/// Sixteen copies of `cube`, a 4x4 grid `ahead` voxels in front of the bench
/// camera, each placed so its local point `middle` lands on the grid. At 120
/// that is the open corridor, clear of the floor and the columns, so each is on
/// screen and unoccluded; at -120 it is behind the camera, where every ray
/// misses every body's box. Turned off-axis, as the demo body spins, rather
/// than meeting every ray square-on.
fn bench_bodies(eye: Vec3, ahead: f32, cube: &Contree, middle: f32) -> Vec<bevox_core::body::Body> {
    let orientation = glam::Quat::from_euler(glam::EulerRot::XYZ, 0.4, 0.7, 0.0);
    let mut bodies = Vec::new();
    for row in 0..4 {
        for column in 0..4 {
            let centre = eye
                + Vec3::new((column as f32 - 1.5) * 40.0, 8.0 + (row as f32 - 1.5) * 24.0, ahead);
            let position = centre - orientation * Vec3::splat(middle);
            bodies.push(bevox_core::body::Body::new(cube.clone(), position, orientation));
        }
    }
    bodies
}

/// A/B/A of `variant` against `baseline` on both clocks: wall, then GPU (median
/// of seven readings after one untimed dispatch), each as (a1, b, a2) medians.
fn aba_both(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    baseline: &Prepared,
    variant: &Prepared,
) -> [(f32, f32, f32); 2] {
    let gpu_ms = |p: &Prepared| {
        p.dispatch(device, queue);
        device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
        let mut t: Vec<f32> =
            (0..7).map(|_| p.dispatch_timed(device, queue).expect("timestamps").total()).collect();
        median(&mut t)
    };
    [compare_aba(device, queue, baseline, variant), aba(baseline, variant, gpu_ms)]
}

/// One bench row, and what B adds over the mean of its bracketing A readings,
/// on the wall clock and the GPU's.
fn row_text(r: [(f32, f32, f32); 2]) -> (String, f32, f32) {
    let added = |(a1, b, a2): (f32, f32, f32)| b - (a1 + a2) * 0.5;
    let [(a1, b, a2), (g1, gb, g2)] = r;
    let text = format!(
        "wall {b:6.2} ms vs {a1:.2}/{a2:.2} (drift {:.2}) = {:+6.2} ms | \
         gpu {gb:6.2} ms vs {g1:.2}/{g2:.2} (drift {:.2}) = {:+6.2} ms",
        (a1 - a2).abs(),
        added(r[0]),
        (g1 - g2).abs(),
        added(r[1]),
    );
    (text, added(r[0]), added(r[1]))
}

/// What composing rigid bodies costs, and so how many the frame can afford.
///
/// Composition is `1 + N` marches per ray, and a body march gets neither the
/// beam seed nor the distance-field skip -- both know only the static world --
/// so each is an unaccelerated march over the whole ray. The static world's
/// accelerated cost says nothing about that, so it is measured.
///
/// Every count is A/B/A against the zero-body scene, on the wall clock and the
/// GPU's own. The zero-body row is a second, separately built zero-body
/// configuration, so what it reads is the noise floor. Per-body cost is the
/// least-squares slope over all four counts, not any single one.
///
/// Before any timing, each count's image is diffed against the zero-body image:
/// a body off screen or hidden would time a march that finds nothing, and the
/// changed-pixel count growing with every count is the proof that it does not.
///
/// One extra row puts all sixteen behind the camera, where no ray enters any
/// body's box. It is not part of the slope; it separates what a body costs a
/// ray that traverses it from what it costs a ray that never comes near it,
/// which is the question a bounding-volume rejection test would answer.
///
/// Another puts sixteen tight bodies where the visible ones are: the same
/// 16-voxel cube, but filling a 16-voxel volume instead of sitting in a 64 one,
/// so no ray enters a body's box and misses its voxels. Also not in the slope.
///
/// Every row runs under each combination of the two body rejections --
/// neither, `CULL_BODIES`, `BODY_RECT`, both -- on top of `DEFAULT` with both
/// removed, and each combination's image is asserted identical to neither's.
/// Each 16-body row then measures every rejection A/B/A directly against
/// neither on the same bodies, so a rejection's gain is read from one
/// comparison rather than from the difference of two.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench bodies --
/// --ignored --nocapture`. `MAX_BODIES` in `bevox_render::pipeline` and
/// `march_flags::DEFAULT` are set from its output.
#[test]
#[ignore]
fn bodies_are_measured_against_none() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    if !timestamps_supported(&device) {
        eprintln!("adapter has no TIMESTAMP_QUERY, skipping");
        return;
    }

    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    // The cube spans 25..41 of its 64 volume, so its middle is local 33.
    let loose = body_cube();
    let mut dense = bevox_core::dense::DenseVolume::new(16).expect("16 is a volume extent");
    for z in 0..16 {
        for y in 0..16 {
            for x in 0..16 {
                dense.set(UVec3::new(x, y, z), MaterialId(1));
            }
        }
    }
    let tight = dense.into_contree();
    let ahead = bench_bodies(eye, 120.0, &loose, 33.0);
    let behind = bench_bodies(eye, -120.0, &loose, 33.0);
    let tight_ahead = bench_bodies(eye, 120.0, &tight, 8.0);

    let base = march_flags::DEFAULT & !(march_flags::CULL_BODIES | march_flags::BODY_RECT);
    let combos = [
        ("neither", base),
        ("cull", base | march_flags::CULL_BODIES),
        ("rect", base | march_flags::BODY_RECT),
        ("both", base | march_flags::CULL_BODIES | march_flags::BODY_RECT),
    ];
    let make = |flags: u32, bodies: &[bevox_core::body::Body]| {
        Prepared::new(
            &device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720, flags, bodies,
        )
    };

    let baseline = make(base, &[]);
    let empty_image = baseline.read_back(&device, &queue);

    println!(
        "scene: extent {extent}, 1280x720, bench camera, flags DEFAULT without CULL_BODIES and \
         BODY_RECT ({base}) plus each combination; loose = 16-voxel cube in a 64 volume, tight = \
         16-voxel cube filling a 16 volume"
    );
    let mut previous_coverage = 0;
    let (mut statics, mut gpu_statics) = (Vec::new(), Vec::new());
    // Per combination: (count, wall added, gpu added) for each visible loose row.
    let mut points = vec![Vec::new(); combos.len()];
    for (label, bodies, visible, in_slope) in [
        ("0 bodies", &ahead[..0], true, true),
        ("1 body", &ahead[..1], true, true),
        ("4 bodies", &ahead[..4], true, true),
        ("16 bodies", &ahead[..], true, true),
        ("16 behind the camera", &behind[..], false, false),
        ("16 tight", &tight_ahead[..], true, false),
    ] {
        let variants: Vec<Prepared> = combos.iter().map(|&(_, f)| make(f, bodies)).collect();
        let image = variants[0].read_back(&device, &queue);
        for ((name, _), v) in combos.iter().zip(&variants).skip(1) {
            assert!(v.read_back(&device, &queue) == image, "{label}: {name} changed the image");
        }
        let coverage =
            empty_image.chunks(4).zip(image.chunks(4)).filter(|(a, b)| a != b).count();
        assert!(
            match (bodies.is_empty() || !visible, in_slope) {
                (true, _) => coverage == 0,
                (false, true) => coverage > previous_coverage,
                (false, false) => coverage > 0,
            },
            "{label} changed {coverage} pixels after {previous_coverage}"
        );
        if in_slope {
            previous_coverage = coverage;
        }

        for (((name, flags), v), points) in combos.iter().zip(&variants).zip(&mut points) {
            // The cull must drop exactly the bodies behind the camera, or this
            // row times something other than what it is labelled.
            let culled = flags & march_flags::CULL_BODIES != 0 && !visible;
            assert_eq!(v.body_count as usize, if culled { 0 } else { bodies.len() }, "{label} {name}");

            let r = aba_both(&device, &queue, &baseline, v);
            let (text, wall, gpu) = row_text(r);
            println!("{label:>20} {name:>7}: {text} | {coverage:>6} px changed, {} marched", v.body_count);
            assert!(r[0].1 > 0.0 && r[1].1 > 0.0, "{label} {name}: the timer returned nothing");
            statics.extend([r[0].0, r[0].2]);
            gpu_statics.extend([r[1].0, r[1].2]);
            if in_slope {
                points.push((bodies.len() as f32, wall, gpu));
            }
        }
        if bodies.len() == 16 {
            for ((name, _), v) in combos.iter().zip(&variants).skip(1) {
                let (text, ..) = row_text(aba_both(&device, &queue, &variants[0], v));
                println!("{label:>20} {name:>7} against neither: {text}");
            }
        }
    }

    // Least-squares slope of the added cost against the body count.
    let slope = |points: &[(f32, f32, f32)], pick: fn(&(f32, f32, f32)) -> f32| {
        let mean_n = points.iter().map(|p| p.0).sum::<f32>() / points.len() as f32;
        let mean_y = points.iter().map(pick).sum::<f32>() / points.len() as f32;
        let covariance: f32 = points.iter().map(|p| (p.0 - mean_n) * (pick(p) - mean_y)).sum();
        let variance: f32 = points.iter().map(|p| (p.0 - mean_n).powi(2)).sum();
        covariance / variance
    };
    let static_wall = median(&mut statics);
    let static_gpu = median(&mut gpu_statics);
    println!("static world: wall {static_wall:.2} ms, gpu {static_gpu:.2} ms");
    for ((name, _), points) in combos.iter().zip(&points) {
        let (per_body, gpu_per_body) = (slope(points, |p| p.1), slope(points, |p| p.2));
        println!(
            "{name:>7}: per visible body (slope over 0/1/4/16): wall {per_body:.3} ms = {:.1}% of \
             the static march, gpu {gpu_per_body:.3} ms = {:.1}%; bodies that fit a 16.7 ms frame \
             beside the static world: wall {:.1}, gpu {:.1}",
            per_body / static_wall * 100.0,
            gpu_per_body / static_gpu * 100.0,
            (16.7 - static_wall) / per_body,
            (16.7 - static_gpu) / gpu_per_body,
        );
    }
}
