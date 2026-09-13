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
    ] {
        let variant = make(flags);
        let (a1, b, a2) = compare_aba(&device, &queue, &baseline, &variant);
        let drift = (a1 - a2).abs();
        let scan = (a1 + a2) * 0.5;
        let gain = scan - b;
        println!(
            "{name:>9}: {b:6.2} ms vs scan {a1:.2}/{a2:.2} (drift {drift:.2})  gain {gain:6.2} ms ({:5.1}%) {}",
            gain / scan * 100.0,
            if gain.abs() > drift { "" } else { "<- within drift, not a result" }
        );
        assert!(b > 0.0, "{name}: the timer returned nothing");
    }
}
