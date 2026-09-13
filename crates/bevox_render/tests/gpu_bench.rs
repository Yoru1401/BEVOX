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
