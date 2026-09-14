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
    let mut a1 = Vec::new();
    let mut b = Vec::new();
    let mut a2 = Vec::new();
    for _ in 0..ROUNDS {
        a1.push(time_dispatches(device, queue, baseline, BATCH));
        b.push(time_dispatches(device, queue, variant, BATCH));
        a2.push(time_dispatches(device, queue, baseline, BATCH));
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
    let view = Mat4::look_at_rh(eye, Vec3::new(half, 40.0, half), Vec3::Y);
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
    let volume = GpuVolume::from_contree(&tree);
    println!(
        "scene: extent {extent}, {} arena nodes, {} voxel bytes, built in {:.2}s",
        tree.arena().nodes().len(),
        tree.arena().voxels().len(),
        built.elapsed().as_secs_f32()
    );

    let (eye, world_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let prepared = Prepared::new(
        &device,
        &shader,
        "march",
        &tree,
        &volume,
        world_from_clip,
        eye,
        1280,
        720,
        0,
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
    let volume = GpuVolume::from_contree(&tree);
    let (eye, world_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    let make = |flags: u32| {
        Prepared::new(
            &device, &shader, "march", &tree, &volume, world_from_clip, eye, 1280, 720, flags,
        )
    };
    let baseline = make(march_flags::NONE);

    for (name, flags) in [
        ("dda", march_flags::DDA),
        ("mask", march_flags::MASK_FILTER),
        ("dda+mask", march_flags::DDA | march_flags::MASK_FILTER),
        ("beam", march_flags::BEAM),
        ("all", march_flags::DDA | march_flags::MASK_FILTER | march_flags::BEAM),
        ("field", march_flags::DISTANCE_FIELD),
        ("all+field", march_flags::DEFAULT | march_flags::DISTANCE_FIELD),
    ] {
        let variant = make(flags);
        let (a1, b, a2) = compare_aba(&device, &queue, &baseline, &variant);
        let drift = (a1 - a2).abs();
        let scan = (a1 + a2) * 0.5;
        let gain = scan - b;
        println!(
            "{name:>9}: {b:6.2} ms vs scan {a1:.2}/{a2:.2} (drift {drift:.2})  gain {gain:6.2} ms ({:5.1}%) {}{}",
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
        let volume = GpuVolume::from_contree(&tree);
        let extent = tree.extent();

        for (label, (eye, world_from_clip)) in
            [("framed", framed_camera(extent)), ("close", close_camera(&tree, extent))]
        {
            let baseline = Prepared::new(
                &device, &shader, "march", &tree, &volume, world_from_clip, eye, 1280, 720,
                march_flags::NONE,
            );
            let variant = Prepared::new(
                &device, &shader, "march", &tree, &volume, world_from_clip, eye, 1280, 720,
                march_flags::DEFAULT,
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
    let view = Mat4::look_at_rh(eye, centre, Vec3::Y);
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
    let view = Mat4::look_at_rh(close, surface + dir * e * 0.25, Vec3::Y);
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
    };
    scene.tree.arena_mut().clear_dirty();

    let whole_nodes = scene.tree.arena().nodes().len() * 16;
    let whole_voxels = scene.tree.arena().voxels().len();
    println!(
        "scene: extent {extent}, {} node bytes, {} voxel bytes",
        whole_nodes, whole_voxels
    );

    for radius in [2.0f32, 8.0, 32.0] {
        let centre = Vec3::splat(extent as f32 * 0.5);
        let started = std::time::Instant::now();
        scene.tree.apply_sphere(centre, radius, bevox_core::material::MaterialId(3));
        let edit_ms = started.elapsed().as_secs_f32() * 1000.0;

        let staged = std::time::Instant::now();
        let update = bevox_render::upload::stage_scene_update(&mut scene);
        let stage_ms = staged.elapsed().as_secs_f32() * 1000.0;

        let node_bytes: usize = update.nodes.iter().map(|w| w.nodes.len() * 16).sum();
        let voxel_bytes: usize = update.voxels.iter().map(|w| w.words.len() * 4).sum();
        let total = node_bytes + voxel_bytes;
        println!(
            "radius {radius:>5}: edit {edit_ms:6.2} ms, stage {stage_ms:5.2} ms, \
             upload {total:>9} bytes ({:.3}% of the scene) in {} ranges",
            total as f64 / (whole_nodes + whole_voxels) as f64 * 100.0,
            update.nodes.len() + update.voxels.len()
        );
        assert!(
            total < whole_nodes + whole_voxels,
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
    use bevox_render::upload::{GpuSceneData, gpu_direction_masks};

    let (tree, extent) = bench_scene();
    let volume = GpuVolume::from_contree(&tree);
    let field = bevox_core::distance_field::DistanceField::build(&tree);
    let scene = GpuSceneData {
        nodes: volume.buffer_nodes(),
        voxels: volume.voxels,
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
                nodes: volume.buffer_nodes(),
                voxels: volume.voxels,
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
    let volume = GpuVolume::from_contree(&tree);
    let (eye, world_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    println!("scene: extent {extent}, 1280x720, close to geometry");
    for (name, flags) in [
        ("scan", march_flags::NONE),
        ("dda", march_flags::DDA),
        ("mask", march_flags::MASK_FILTER),
        ("beam", march_flags::BEAM),
        ("all", march_flags::DEFAULT),
        ("field", march_flags::DISTANCE_FIELD),
        ("all+field", march_flags::DEFAULT | march_flags::DISTANCE_FIELD),
    ] {
        let prepared = Prepared::new(
            &device, &shader, "march", &tree, &volume, world_from_clip, eye, 1280, 720, flags,
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
            println!("{name:>6}: {total:7.3} ms total  (beam {beam:.3} + main {:.3})", total - beam);
        } else {
            println!("{name:>6}: {total:7.3} ms total");
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
