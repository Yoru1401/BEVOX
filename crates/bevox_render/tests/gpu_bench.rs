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

/// Far enough that a voxel covers about one pixel, looking along the floor so
/// the frame is terrain rather than sky.
///
/// **This is the worst case for the sun store and the reason it exists as a
/// camera.** The store's whole win is redundancy -- `bench_camera` has 9.22
/// pixels sharing one voxel face's answer at 1080p -- and a voxel that covers
/// one pixel has none to share. Every insert, every compaction entry, every
/// per-pixel record and every extra read in the composite is then paid for a
/// cache that is never hit twice.
///
/// At 1024 voxels a vertical field of 0.9 rad over 1080 rows gives 8.33e-4 rad
/// a pixel, so a voxel subtends about a pixel at 1200 voxels' distance. The eye
/// is high and near one edge, aimed down-range at the far edge of the floor, so
/// the whole frame is floor at 800 to 1200 voxels rather than half sky. At 720p
/// the same camera is harsher still -- 1.25e-3 rad a pixel, so a voxel covers
/// about two thirds of one -- which is why the factor is reported per
/// resolution rather than asserted as a constant.
fn distant_camera(extent: u32) -> (Vec3, Mat4) {
    let e = extent as f32;
    let eye = Vec3::new(e * 0.5, e * 0.68, e * 0.04);
    let target = Vec3::new(e * 0.5, 2.0, e);
    let view = Mat4::look_at_rh(Vec3::ZERO, target - eye, Vec3::Y);
    let projection = Mat4::perspective_rh(0.9, 1280.0 / 720.0, 0.1, e * 8.0);
    (eye, (projection * view).inverse())
}

/// `march.wgsl` as some earlier revision wrote it, for an A/B/A across two
/// source files rather than within one module.
///
/// Measuring the new path against `march` inside the *same* module flatters it:
/// splitting the pass cost the byte-for-byte unedited `march` up to +0.67 ms at
/// 1080p, measured in Task 2, so the in-module A side carries a penalty the
/// shipped shader before this work did not. See
/// `docs/concepts/gpu-codegen-cliff.md` -- the driver compiles the whole module
/// and what it does with register pressure is not visible from the source.
///
/// Returns None rather than failing when git cannot answer, so a checkout
/// without history still runs the rest of the suite.
fn shader_at(revision: &str) -> Option<String> {
    let root = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."));
    let spec = format!("{revision}:crates/bevox_render/assets/shaders/march.wgsl");
    let out = std::process::Command::new("git")
        .args(["show", &spec])
        .current_dir(root)
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    String::from_utf8(out.stdout).ok()
}

/// **The worst case, and whether a spatially coherent hash closes it.**
///
/// Two cameras. `bench_camera` is close, 9.22 pixels a face, and is where the
/// +5.3 ms was collected. `distant_camera` covers about one pixel a face and is
/// where the four-pass path was measured 6.8-7.1 ms SLOWER than doing nothing.
///
/// Three baselines, because they answer three different questions:
///
/// - `34f2d45`, before any of this work, whose shader has no store at all.
///   **This is the gate**: positive means the whole change is a loss.
/// - `9fb62ba`, the same four passes with the **scrambling** hash, which
///   isolates the hash change from everything else.
/// - `4560d58`, which inserts and reads nothing. It already pays the insert, so
///   an A side taken here flatters the four passes.
///
/// The redundancy factor is reported first: a gate on a camera that turned out
/// to have redundancy left would prove nothing.
///
/// The number this exists for is **ns a ray in the sun pass**. A per-slot ray
/// cost 20.0 ns against 9.1 for the per-pixel one it replaced, and that ratio
/// is the break-even redundancy -- so halving it halves what the store needs to
/// share before it pays. The primary pass is printed beside it because
/// clustering lengthens probes and the probes are paid there.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench worst_case --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn the_worst_case_is_measured_against_the_parent() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    if !timestamps_supported(&device) {
        eprintln!("adapter has no TIMESTAMP_QUERY, skipping");
        return;
    }
    let (tree, extent) = bench_scene();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    /// The revision whose `march.wgsl` is the same four passes with the
    /// scrambling hash, so the hash can be measured on its own.
    const SCRAMBLED: &str = "9fb62ba";

    for (where_from, (eye, offset_from_clip)) in
        [("bench", bench_camera(extent)), ("distant", distant_camera(extent))]
    {
        for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
            let build = |source: &str, entry: &str| {
                Prepared::new(
                    &device, source, entry, &tree, offset_from_clip, eye, w, h,
                    march_flags::DEFAULT, &[],
                )
            };

            let store = {
                let probe = build(&shader, "march");
                probe.clear_sun_pixel_slots(&queue);
                probe.sun_store(&device, &queue)
            };
            let redundancy = store.pixels_with_slot as f64 / store.distinct_keys.max(1) as f64;
            println!(
                "\n{where_from} camera {w}x{h}: {} of {} pixels inserted, {} distinct faces, \
                 {:.4} slots a face\n  REDUNDANCY {redundancy:.2} pixels per distinct face",
                store.pixels_with_slot,
                w * h,
                store.distinct_keys,
                store.occupied as f64 / store.distinct_keys.max(1) as f64,
            );

            // One timing configuration at a time past here: each holds a sun
            // table of about 91 MB at 1080p and this card is shared.
            let one = {
                let single = build(&shader, "march");
                pass_times(&device, &queue, &single)
            };
            let four = {
                let split = build(&shader, "march_composite");
                pass_times(&device, &queue, &split)
            };
            let ns_slot = four[3] * 1.0e6 / store.distinct_keys.max(1) as f32;
            let ns_pixel = (one[4] - four[1]) * 1.0e6 / store.pixels_with_slot.max(1) as f32;
            println!(
                "  per pass, GPU ms, median of 7:\n    \
                 one:  beam {:.2} + march {:.2} = {:.2}\n    \
                 four: beam {:.2} + primary {:.2} + compact {:.2} + sun {:.2} + composite \
                 {:.2} = {:.2}\n  \
                 SUN RAY {ns_slot:.1} ns against {ns_pixel:.1} ns per pixel, so BREAK-EVEN is \
                 {:.2} pixels a face before the store's own overhead",
                one[0], one[4], one.iter().sum::<f32>(),
                four[0], four[1], four[2], four[3], four[4], four.iter().sum::<f32>(),
                ns_slot / ns_pixel,
            );

            if let Some(scrambled) = shader_at(SCRAMBLED) {
                let old = {
                    let p = build(&scrambled, "march_composite");
                    pass_times(&device, &queue, &p)
                };
                println!(
                    "  {SCRAMBLED}'s scrambling hash, same four passes: primary {:.2} (against \
                     {:.2}), sun {:.2} (against {:.2})\n    \
                     so the coherent hash moves the primary {:+.2} ms -- where the probes are \
                     paid -- and the sun pass {:+.2}",
                    old[1], four[1], old[3], four[3], four[1] - old[1], four[3] - old[3],
                );
            }

            let split = build(&shader, "march_composite");
            for revision in [SCRAMBLED, "34f2d45", "4560d58"] {
                let Some(parent) = shader_at(revision) else {
                    eprintln!("  git could not produce {revision}'s shader, skipping it");
                    continue;
                };
                let entry = if revision == SCRAMBLED { "march_composite" } else { "march" };
                let base = build(&parent, entry);
                let (text, wall, gpu) = row_text(aba_both(&device, &queue, &base, &split));
                println!("  against {revision} ({entry}):\n    {text}");
                println!(
                    "    at {redundancy:.2} pixels a face this is {gpu:+.2} ms on the GPU clock \
                     ({wall:+.2} wall); positive is SLOWER",
                );
            }
        }
    }
}

/// **The overflow fallback, gated by actually overflowing the table.**
///
/// A pixel whose insert spends all `SUN_PROBES` probes carries `SUN_NO_SLOT`
/// and marches its own shadow ray in the composite, which is the same fallback
/// a key mismatch takes. Task 2 exercised it incidentally through a key-break;
/// this fills the table instead, which is the condition the spec names.
///
/// The slot count travels in the uniform, so this shrinks it without editing
/// one byte of the shader and without a smaller buffer -- what is compared is
/// one compiled module against itself, with no codegen cliff in the way.
///
/// What it reports: how many pixels came away with a slot at each capacity, so
/// the capacity at which inserts genuinely start failing is read off rather
/// than assumed, and what the frame costs there.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench overflow --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn shrinking_the_table_until_it_overflows_is_measured() {
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

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        let build = |entry: &str, slots: u32| {
            Prepared::with_workgroup_and_slots(
                &device, &shader, entry, &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[], 8, slots,
            )
        };
        println!("\n{w}x{h} as the table shrinks:");
        let mut full = 0usize;
        for power in (14u32..=21).rev() {
            let slots = 1u32 << power;
            let p = build("march", slots);
            p.clear_sun_pixel_slots(&queue);
            let store = p.sun_store(&device, &queue);
            if power == 21 {
                full = store.pixels_with_slot;
            }
            println!(
                "  2^{power} = {slots:>9} slots: {:>7} pixels got one of {full} ({:>5.1}% gave \
                 up and marched their own ray), {:>6} distinct faces, load {:.2}",
                store.pixels_with_slot,
                (full.saturating_sub(store.pixels_with_slot)) as f64 / full.max(1) as f64 * 100.0,
                store.distinct_keys,
                store.distinct_keys as f64 / f64::from(slots),
            );
        }
        for power in [16u32, 14] {
            let slots = 1u32 << power;
            let a = build("march_composite", bevox_render::pipeline::SUN_SLOTS);
            let b = build("march_composite", slots);
            let (text, _, gpu) = row_text(aba_both(&device, &queue, &a, &b));
            println!("  2^{power} slots against 2^21, four passes both sides:\n    {text}");
            println!("    overflowing to 2^{power} costs {gpu:+.2} ms on the GPU clock");
        }
    }
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
    let fullness = bevox_core::fullness::Fullness::build(&tree);
    let mut scene = bevox_render::upload::VoxelScene {
        tree,
        materials: bevox_core::material::MaterialTable::new(),
        generation: 1,
        field,
        field_dirty: None,
            fullness,
            fullness_dirty: None,
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
    let fullness = bevox_core::fullness::Fullness::build(&tree);
    let scene = GpuSceneData {
        fullness_base: bevox_render::upload::field_words(&field),
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
        distance_field: bevox_render::upload::pack_grids(&field, &fullness),
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
            let fullness = bevox_core::fullness::Fullness::build(&tree);
            let real = GpuSceneData {
                fullness_base: bevox_render::upload::field_words(&field),
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
                distance_field: bevox_render::upload::pack_grids(&field, &fullness),
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
/// so inside a body's box nothing accelerates the walk. The static world's
/// accelerated cost says nothing about that, so it is measured.
///
/// This said "an unaccelerated march over the whole ray" until 2026-09-29, and
/// that conclusion was wrong and cost a day: `traverse_at` seeds its first
/// frame at `max(root_slab.t_enter, t_start)`, so a body march starts at the
/// body's bounding box and never walks the empty space in front of it. The
/// premise about the beam and the field was true; the conclusion drawn from it
/// was never checked against the traversal. See
/// `docs/concepts/an-inference-is-not-an-observation.md`.
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

/// What body shadows cost: `BODY_SHADOWS` A/B/A against the same flags without
/// it, on the wall clock and the GPU's.
///
/// - No bodies: the caster loop runs zero times, so this reads what adding it
///   to the shader costs by itself, which is where a driver codegen cliff would
///   show. The image must not change.
/// - Sixteen loose bodies in view, as `bodies_are_measured_against_none` times
///   them: every lit pixel's shadow ray now tests them all.
/// - The same sixteen behind the camera: culled from the primary march, but
///   still shadow casters.
///
/// The total for sixteen in view with shadows is what `DEFAULT` is decided on,
/// against a 16.7 ms frame.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench shadows --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn body_shadows_are_measured() {
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
    let loose = body_cube();
    let ahead = bench_bodies(eye, 120.0, &loose, 33.0);
    let behind = bench_bodies(eye, -120.0, &loose, 33.0);
    let without = march_flags::DEFAULT & !march_flags::BODY_SHADOWS;
    let with = without | march_flags::BODY_SHADOWS;
    let make = |flags: u32, bodies: &[bevox_core::body::Body]| {
        Prepared::new(&device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720, flags, bodies)
    };
    println!("scene: extent {extent}, 1280x720, bench camera; A = flags {without}, B = {with}");
    for (label, bodies) in [("0 bodies", &ahead[..0]), ("16 in view", &ahead[..]), ("16 behind", &behind[..])] {
        let (a, b) = (make(without, bodies), make(with, bodies));
        let changed = a
            .read_back(&device, &queue)
            .chunks(4)
            .zip(b.read_back(&device, &queue).chunks(4))
            .filter(|(x, y)| x != y)
            .count();
        if bodies.is_empty() {
            assert_eq!(changed, 0, "body shadows changed a scene with no bodies");
        }
        let r = aba_both(&device, &queue, &a, &b);
        let (text, ..) = row_text(r);
        println!("{label:>11}: {text} | {changed:>6} px shadowed by bodies | with shadows: wall {:.2} ms, gpu {:.2} ms", r[0].1, r[1].1);
    }
}

/// What ambient occlusion costs: `AO` A/B/A against the same flags without it,
/// on a body-free scene and on sixteen bodies in view.
///
/// The fullness grid is read once per hit pixel, eight cells trilinearly
/// blended, so the cost should track the number of hit pixels and nothing else.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench ambient --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn ambient_occlusion_is_measured() {
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
    let loose = body_cube();
    let ahead = bench_bodies(eye, 120.0, &loose, 33.0);
    let without = march_flags::DEFAULT & !march_flags::AO;
    let with = without | march_flags::AO;
    let make = |flags: u32, bodies: &[bevox_core::body::Body]| {
        Prepared::new(&device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720, flags, bodies)
    };
    println!("scene: extent {extent}, 1280x720, bench camera; A = flags {without}, B = {with}");
    for (label, bodies) in [("0 bodies", &ahead[..0]), ("16 in view", &ahead[..])] {
        let (a, b) = (make(without, bodies), make(with, bodies));
        let changed = a
            .read_back(&device, &queue)
            .chunks(4)
            .zip(b.read_back(&device, &queue).chunks(4))
            .filter(|(x, y)| x != y)
            .count();
        assert!(changed > 1000, "{label}: ambient occlusion darkened only {changed} pixels");
        let r = aba_both(&device, &queue, &a, &b);
        let (text, ..) = row_text(r);
        println!("{label:>11}: {text} | {changed:>6} px darkened | with AO: wall {:.2} ms, gpu {:.2} ms", r[0].1, r[1].1);
    }
}

/// Where a body's cost actually goes: the static march, the primary body
/// marches, and the shadow rays' body tests, separated.
///
/// `MAX_BODIES` was set from a per-body figure measured on 2026-09-17, before
/// body shadows (2026-09-18) and ambient occlusion (2026-09-24) joined
/// `DEFAULT`. Nothing has re-measured it since, so the number the cap rests on
/// describes a shader that no longer exists.
///
/// This decides what to do about it. Composition is `1 + N` marches per ray for
/// the primary ray, and a shadow ray that the static world leaves clear tests
/// every caster as well, so **both halves scale with the body count** — but not
/// necessarily alike, and the fix differs:
///
/// - If the primary marches dominate, the answer is Dwyer's devlog 2: one
///   interleaved traversal that steps whichever volume offers the nearest next
///   voxel, instead of `N` separate marches compared by depth. That removes the
///   per-body term from the primary loop.
/// - If the shadow tests dominate, the answer is his devlogs 7 and 19: sun
///   visibility is a property of a voxel, not a pixel, so it is computed once
///   per visible voxel and cached, not once per pixel.
///
/// See `docs/reference/dwyer-drift.md`, divergences 1 to 3.
///
/// Every configuration is timed in the same invocation, round-robin over three
/// rounds, because cross-run drift on this machine is routinely larger than the
/// effects being separated. Medians of three rounds of seven readings.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench where_a_body
/// -- --ignored --nocapture`.
#[test]
#[ignore]
fn where_a_body_s_cost_goes() {
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
    let cube = body_cube();
    let all = bench_bodies(eye, 120.0, &cube, 33.0);

    let lit = march_flags::DEFAULT;
    let unshadowed = march_flags::DEFAULT & !march_flags::BODY_SHADOWS;
    let make = |flags: u32, bodies: &[bevox_core::body::Body]| {
        Prepared::new(
            &device, &shader, "march", &tree, offset_from_clip, eye, 1280, 720, flags, bodies,
        )
    };

    // Named configurations, each built once and timed in rotation.
    let mut configs: Vec<(String, Prepared)> = Vec::new();
    configs.push(("static, DEFAULT".to_string(), make(lit, &[])));
    configs.push(("static, no body shadows".to_string(), make(unshadowed, &[])));
    for n in [1usize, 4, 16] {
        configs.push((format!("{n:>2} bodies, DEFAULT"), make(lit, &all[..n])));
        configs.push((format!("{n:>2} bodies, no shadows"), make(unshadowed, &all[..n])));
    }

    let gpu_ms = |p: &Prepared| {
        p.dispatch(&device, &queue);
        device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
        let mut t: Vec<f32> =
            (0..7).map(|_| p.dispatch_timed(&device, &queue).expect("timestamps").total()).collect();
        median(&mut t)
    };

    let mut rounds: Vec<Vec<f32>> = vec![Vec::new(); configs.len()];
    for _ in 0..3 {
        for (i, (_, prepared)) in configs.iter().enumerate() {
            rounds[i].push(gpu_ms(prepared));
        }
    }

    println!("\nGPU ms, median of 3 rounds x 7 readings, 1280x720, bench camera:");
    let mut medians = Vec::new();
    for ((name, _), samples) in configs.iter().zip(&mut rounds) {
        let spread = samples.iter().fold(0.0f32, |a, &b| a.max(b))
            - samples.iter().fold(f32::MAX, |a, &b| a.min(b));
        let m = median(samples);
        medians.push(m);
        println!("  {name:>24}: {m:6.2} ms   (spread {spread:.2})");
    }

    // medians: [static lit, static unlit, 1 lit, 1 unlit, 4 lit, 4 unlit, 16 lit, 16 unlit]
    let (static_lit, static_unlit) = (medians[0], medians[1]);
    println!("\nWhat each body adds, against the matching static baseline:");
    let mut primary = Vec::new();
    let mut shadow = Vec::new();
    for (k, n) in [1usize, 4, 16].iter().enumerate() {
        let (lit_n, unlit_n) = (medians[2 + k * 2], medians[3 + k * 2]);
        let per_primary = (unlit_n - static_unlit) / *n as f32;
        let per_shadow = ((lit_n - static_lit) - (unlit_n - static_unlit)) / *n as f32;
        primary.push(per_primary);
        shadow.push(per_shadow);
        println!(
            "  {n:>2} bodies: total {:+6.2} ms | primary {:+6.2} ({per_primary:.3}/body) | \
             shadow {:+6.2} ({per_shadow:.3}/body)",
            lit_n - static_lit,
            unlit_n - static_unlit,
            (lit_n - static_lit) - (unlit_n - static_unlit),
        );
    }

    let at16 = (primary[2], shadow[2]);
    let total16 = at16.0 + at16.1;
    println!(
        "\nAt sixteen bodies: {:.3} ms per body, of which primary {:.0}% and shadow {:.0}%.",
        total16,
        at16.0 / total16 * 100.0,
        at16.1 / total16 * 100.0,
    );
    println!(
        "  Frame: {:.2} ms static + {:.2} ms for sixteen = {:.2} ms against 16.7.",
        static_lit,
        total16 * 16.0,
        medians[6],
    );
    println!(
        "  Interleaved stepping (devlog 2) attacks the primary share; a per-voxel sun cache \
         (devlogs 7, 19) attacks the shadow share."
    );

    // Linear in the body count is the `1 + N` signature, and the reason either
    // rewrite is worth doing at all. If cost were sublinear something already
    // rejects bodies and the premise is wrong.
    let ratio = (primary[2] + shadow[2]) / (primary[0] + shadow[0]);
    println!(
        "\nPer-body cost at 16 against at 1: {ratio:.2}x. Near 1.0 means cost is linear in the \
         body count, which is what `1 + N` predicts."
    );
}

/// What this machine's adapter actually allows, against what the engine asks
/// for.
///
/// `gpu_device` requests `wgpu::Limits::default()`, which is the WebGPU *spec
/// baseline* -- the set chosen so that code runs everywhere, browsers included.
/// BEVOX's core spec excludes WebAssembly permanently and says native desktop
/// may be assumed everywhere, so any gap between the baseline and the hardware
/// is headroom the engine is declining for no reason.
///
/// The eight-storage-buffer ceiling is the one that has shaped a design
/// decision: the ambient-occlusion fullness grid rides packed behind the
/// distance field because the compute stage was "already at the limit". This
/// prints whether that limit was the hardware's or the baseline's.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench what_the_adapter
/// -- --ignored --nocapture`.
#[test]
#[ignore]
fn what_the_adapter_allows() {
    let instance = wgpu::Instance::default();
    let Some(adapter) = pollster::block_on(
        instance.request_adapter(&wgpu::RequestAdapterOptions::default()),
    )
    .ok() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let info = adapter.get_info();
    let have = adapter.limits();
    let asked = wgpu::Limits::default();

    println!("\n{} ({:?}, {:?})", info.name, info.device_type, info.backend);
    println!("\n{:<44} {:>14} {:>14}", "limit", "engine asks", "adapter allows");
    let rows: [(&str, u32, u32); 6] = [
        ("max_storage_buffers_per_shader_stage",
         asked.max_storage_buffers_per_shader_stage, have.max_storage_buffers_per_shader_stage),
        ("max_sampled_textures_per_shader_stage",
         asked.max_sampled_textures_per_shader_stage, have.max_sampled_textures_per_shader_stage),
        ("max_samplers_per_shader_stage",
         asked.max_samplers_per_shader_stage, have.max_samplers_per_shader_stage),
        ("max_storage_textures_per_shader_stage",
         asked.max_storage_textures_per_shader_stage, have.max_storage_textures_per_shader_stage),
        ("max_compute_invocations_per_workgroup",
         asked.max_compute_invocations_per_workgroup, have.max_compute_invocations_per_workgroup),
        ("max_compute_workgroup_storage_size",
         asked.max_compute_workgroup_storage_size, have.max_compute_workgroup_storage_size),
    ];
    for (name, a, h) in rows {
        let note = if h > a { "  <-- headroom" } else { "" };
        println!("{name:<44} {a:>14} {h:>14}{note}");
    }
    println!("\n{:<44} {:>14} {:>14}", "max_buffer_size (MB)",
        asked.max_buffer_size / (1024 * 1024), have.max_buffer_size / (1024 * 1024));
    println!("{:<44} {:>14} {:>14}", "max_storage_buffer_binding_size (MB)",
        asked.max_storage_buffer_binding_size as u64 / (1024 * 1024),
        have.max_storage_buffer_binding_size as u64 / (1024 * 1024));
}

/// What the sun shadow ray costs, which bounds what a per-voxel sun store can win.
///
/// The shading path marches a second ray to the sun on every hit. The origin is
/// snapped to the voxel centre, so **the answer is already per voxel** — a whole
/// face is lit or shadowed together — while the work is per pixel. At 1080p a
/// voxel covers many pixels and the same answer is recomputed for each of them.
///
/// This is a **cost probe, not a bit-identity test**: forcing the sun unshadowed
/// changes pixels by construction. It measures the ceiling on any scheme that
/// computes sun visibility once per voxel, and nothing more.
#[test]
#[ignore]
fn what_the_sun_shadow_ray_costs() {
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

    // The shading call site. Replacing the condition with `false` removes the
    // march and leaves everything else — the hit, the normal, the ambient term —
    // exactly as it was.
    let needle = "if shadowed(origin, sun, max_ray_distance()) {";
    assert_eq!(
        shader.matches(needle).count(),
        1,
        "the shading call site moved; this probe edits it by text and must be updated with it",
    );
    let no_sun_march = shader.replace(needle, "if false {");

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        let built: Vec<(&str, Prepared)> = vec![
            ("with the shadow march", Prepared::new(
                &device, &shader, "march", &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            )),
            ("without it", Prepared::new(
                &device, &no_sun_march, "march", &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            )),
        ];
        let gpu_ms = |p: &Prepared| {
            p.dispatch(&device, &queue);
            device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
            let mut t: Vec<f32> = (0..7)
                .map(|_| p.dispatch_timed(&device, &queue).expect("timestamps").total())
                .collect();
            median(&mut t)
        };
        let mut rounds: Vec<Vec<f32>> = vec![Vec::new(); built.len()];
        for _ in 0..3 {
            for (i, (_, p)) in built.iter().enumerate() {
                rounds[i].push(gpu_ms(p));
            }
        }
        let with = median(&mut rounds[0]);
        let without = median(&mut rounds[1]);
        let spread = |v: &Vec<f32>| {
            v.iter().fold(0.0f32, |a, &b| a.max(b)) - v.iter().fold(f32::MAX, |a, &b| a.min(b))
        };
        println!(
            "\n{w}x{h}, GPU ms, median of 3 rounds x 7, interleaved:\n  \
             {:<22} {with:.2} (spread {:.2})\n  {:<22} {without:.2} (spread {:.2})\n  \
             the sun shadow march is {:.2} ms, {:.1}% of the frame",
            built[0].0, spread(&rounds[0]), built[1].0, spread(&rounds[1]),
            with - without,
            (with - without) / with * 100.0,
        );
    }
}

/// What the static march costs at the resolution the target is written in.
///
/// The frame budget is quoted at 1280x720 because that is where every earlier
/// number was taken. The target is **60 fps at 1920x1080**, which is 2.25x the
/// rays — and whether the cost actually scales with rays is a measurement, not
/// an assumption. The beam prepass runs at 1/8 per axis and the distance-field
/// skip happens before a ray enters the tree, so there is no reason in advance
/// to expect exactly 2.25x in either direction.
///
/// Both resolutions are timed in one invocation, interleaved, so the ratio
/// cannot be an artefact of two runs.
#[test]
#[ignore]
fn the_march_is_measured_at_both_resolutions() {
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

    let sizes = [(1280u32, 720u32), (1920, 1080)];
    let built: Vec<((u32, u32), Prepared)> = sizes
        .iter()
        .map(|&(w, h)| {
            let p = Prepared::new(
                &device, &shader, "march", &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            );
            ((w, h), p)
        })
        .collect();

    let gpu_ms = |p: &Prepared| {
        p.dispatch(&device, &queue);
        device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
        let mut t: Vec<f32> =
            (0..7).map(|_| p.dispatch_timed(&device, &queue).expect("timestamps").total()).collect();
        median(&mut t)
    };
    let mut rounds: Vec<Vec<f32>> = vec![Vec::new(); built.len()];
    for _ in 0..3 {
        for (i, (_, p)) in built.iter().enumerate() {
            rounds[i].push(gpu_ms(p));
        }
    }

    println!("\nstatic march at DEFAULT, GPU ms, median of 3 rounds x 7, interleaved:");
    let mut base_ms = 0.0f32;
    let mut base_px = 0.0f64;
    for (((w, h), _), samples) in built.iter().zip(&mut rounds) {
        let spread = samples.iter().fold(0.0f32, |a, &b| a.max(b))
            - samples.iter().fold(f32::MAX, |a, &b| a.min(b));
        let ms = median(samples);
        let px = f64::from(*w) * f64::from(*h);
        if base_ms == 0.0 {
            base_ms = ms;
            base_px = px;
            println!("  {w}x{h}: {ms:.2} ms (spread {spread:.2})  -- baseline");
        } else {
            println!(
                "  {w}x{h}: {ms:.2} ms (spread {spread:.2})  {:.2}x the time for {:.2}x the rays \
                 -- {:.2} ms per megaray, against {:.2}",
                ms / base_ms,
                px / base_px,
                f64::from(ms) / (px / 1e6),
                f64::from(base_ms) / (base_px / 1e6),
            );
            println!(
                "\n  60 fps at {w}x{h} needs 16.7 ms. This is {:.2} ms, so the speedup wanted \
                 is {:.2}x.",
                ms,
                ms / 16.7,
            );
        }
    }
}

/// What the static march costs, and why — workgroup size, and how many steps a
/// ray actually takes.
///
/// The static world is **13.23 ms of a 16.7 ms frame** before a body exists,
/// against Dwyer's **7 ms for a Teardown castle on a 1660 Ti** with a primary
/// and a shadow ray (his devlog 17). A 1650 is roughly a third down on a
/// 1660 Ti, so the hardware explains part of a 2x gap and not all of it. This
/// asks two questions the flag benchmarks cannot:
///
/// 1. **Is the marcher step-bound or bandwidth-bound?** The mean steps per ray
///    divides the frame into "how many steps" and "what a step costs". If the
///    mean is low and the frame is still slow, more traversal cleverness is the
///    wrong lever.
/// 2. **Does workgroup size matter?** `WORKGROUP` has been 8 since the shader
///    was written, never measured. For a memory-bound kernel it sets occupancy
///    and how neighbouring rays' reads coalesce. 16x16 is 256 invocations,
///    exactly the WebGPU baseline's ceiling, which is one reason it was never
///    tried; `device_limits` now asks for 1024 so 32x32 is reachable too.
///
/// The shader is read from disk, so the workgroup size is rewritten in the
/// source and every variant is compiled and timed in this one invocation —
/// cross-run drift here is routinely larger than the effect.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench static_march
/// -- --ignored --nocapture`.
#[test]
#[ignore]
fn the_static_march_is_measured() {
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
    let (w, h) = (1280u32, 720u32);
    let lit = march_flags::DEFAULT;

    // How many steps a ray spends. `march_steps` writes steps/64 into red,
    // clamped, so the mean red channel times 64 is the mean step count for
    // every ray that did not saturate, and the saturated share is reported
    // beside it so the mean is never read as the whole story.
    let heat = Prepared::new(
        &device, &shader, "march_steps", &tree, offset_from_clip, eye, w, h, lit, &[],
    );
    let image = heat.read_back(&device, &queue);
    let reds: Vec<u32> = image.chunks_exact(4).map(|p| u32::from(p[0])).collect();
    let saturated = reds.iter().filter(|&&r| r == 255).count();
    let mean_steps = reds.iter().sum::<u32>() as f64 / reds.len() as f64 / 255.0 * 64.0;
    println!("\nsteps per ray on the bench scene, from the `march_steps` view:");
    println!("  mean {mean_steps:.1} of a 64-step scale");
    println!(
        "  {} of {} rays saturated the scale ({:.2}%), so the true mean is at least this",
        saturated,
        reds.len(),
        saturated as f64 / reds.len() as f64 * 100.0,
    );

    // Workgroup size, every variant compiled and timed in this run.
    let sizes = [8u32, 16, 32];
    let built: Vec<(u32, Prepared)> = sizes
        .iter()
        .map(|&n| {
            let src = shader.replace(
                "@workgroup_size(8, 8, 1)",
                &format!("@workgroup_size({n}, {n}, 1)"),
            );
            let p = Prepared::with_workgroup(
                &device, &src, "march", &tree, offset_from_clip, eye, w, h, lit, &[], n,
            );
            (n, p)
        })
        .collect();

    // Every variant must draw the same picture: workgroup size is a scheduling
    // choice and may not move a pixel.
    let reference = built[0].1.read_back(&device, &queue);
    for (n, p) in &built[1..] {
        let got = p.read_back(&device, &queue);
        let differing = reference
            .chunks_exact(4)
            .zip(got.chunks_exact(4))
            .filter(|(a, b)| a != b)
            .count();
        assert_eq!(differing, 0, "{n}x{n} changed {differing} pixels; it is a scheduling choice");
    }

    let gpu_ms = |p: &Prepared| {
        p.dispatch(&device, &queue);
        device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
        let mut t: Vec<f32> =
            (0..7).map(|_| p.dispatch_timed(&device, &queue).expect("timestamps").total()).collect();
        median(&mut t)
    };
    let mut rounds: Vec<Vec<f32>> = vec![Vec::new(); built.len()];
    for _ in 0..3 {
        for (i, (_, p)) in built.iter().enumerate() {
            rounds[i].push(gpu_ms(p));
        }
    }

    println!("\nstatic march, GPU ms, median of 3 rounds x 7, interleaved:");
    let mut base = 0.0f32;
    for ((n, _), samples) in built.iter().zip(&mut rounds) {
        let spread = samples.iter().fold(0.0f32, |a, &b| a.max(b))
            - samples.iter().fold(f32::MAX, |a, &b| a.min(b));
        let m = median(samples);
        if *n == 8 {
            base = m;
        }
        let delta = if *n == 8 { String::new() } else { format!("  {:+.2} ms", m - base) };
        println!("  {n:>2}x{n:<2} ({:>4} invocations): {m:6.2} ms  (spread {spread:.2}){delta}", n * n);
    }
    println!(
        "\n  A difference smaller than its spread is not a result. Dwyer's reference point: 7 ms \
         for a Teardown castle on a 1660 Ti, primary and shadow ray."
    );
}

/// The one line the sun store adds to the shading path, which every probe here
/// removes to get its baseline.
///
/// Removing the call leaves both bindings declared, so what the probes compare
/// is the insert and not the declarations. The declarations were measured
/// separately, against the parent revision's shader, when they were added.
const SUN_INSERT_CALL: &str = "        sun_remember(hit, id, size);\n";

/// The one line that builds a claim word out of the stamp and the key's tag.
const SUN_CLAIM_WORD: &str =
    "    let want = (stamp << SUN_STAMP_SHIFT) | (sun_key_tag(lo, hi) & SUN_TAG_MASK);";

/// The source with the tag dropped from the claim word, so every face this frame
/// claims the same word and any slot claimed this frame reads as its own.
fn without_the_key_tag(shader: &str) -> String {
    assert_eq!(
        shader.matches(SUN_CLAIM_WORD).count(),
        1,
        "the claim word moved; the sun-store breaks edit it by text and must be updated with it",
    );
    shader.replace(SUN_CLAIM_WORD, "    let want = stamp << SUN_STAMP_SHIFT;")
}

/// The source with the insert race back: the claim word is the stamp alone, and
/// the insert reads the full key out of `sun_table` to decide whether a slot
/// claimed this frame is its own. That read is what the race was.
fn with_the_insert_race(shader: &str) -> String {
    let mine = "        if held == want {\n            return slot;\n        }\n";
    assert_eq!(
        shader.matches(mine).count(),
        1,
        "the insert's ownership test moved; this break edits it by text and must be updated \
         with it",
    );
    // The "this frame, another face" branch goes too, which the tag-less form
    // never had: with no tag it cannot fire, and leaving dead code in the loop
    // is exactly the kind of thing this project has watched move a frame by
    // tens of percent.
    // Matched from the branch's head to its tail rather than quoted whole: the
    // commentary inside it is a dozen lines explaining the run-width stride,
    // and a break that has to be re-typed every time a comment is reworded is a
    // break that gets deleted.
    let other_head = "        if (held >> SUN_STAMP_SHIFT) == stamp {\n";
    let other_tail = "            continue;\n        }\n";
    let other = {
        assert_eq!(
            shader.matches(other_head).count(),
            1,
            "the insert's probe-on branch moved; this break edits it by text",
        );
        let from = shader.find(other_head).expect("checked above");
        let to = shader[from..].find(other_tail).expect("the probe-on branch has no tail")
            + from
            + other_tail.len();
        shader[from..to].to_string()
    };
    let other = other.as_str();
    without_the_key_tag(shader).replace(other, "").replace(
        mine,
        "        if held == want {\n\
         \x20           if sun_table[slot * 2u] == lo && sun_table[slot * 2u + 1u] == hi {\n\
         \x20               return slot;\n\
         \x20           }\n\
         \x20           slot = (slot + 1u) & mask;\n\
         \x20           continue;\n\
         \x20       }\n",
    )
}

/// The source with the insert removed and everything else, bindings included,
/// left where it is.
fn without_the_sun_store(shader: &str) -> String {
    assert_eq!(
        shader.matches(SUN_INSERT_CALL).count(),
        1,
        "the insert call site moved; every sun-store probe edits it by text and must be \
         updated with it",
    );
    shader.replace(SUN_INSERT_CALL, "")
}

/// Inserting into the sun store moves no pixel, at either resolution, with
/// bodies and without.
///
/// The premise of the whole plan is that sun visibility is *already* a per-voxel
/// property, so a store nothing reads cannot move a pixel. This revision writes
/// the store and shading still marches its own shadow ray, so the image is the
/// image. A differing pixel here is not a trade-off: it means the insert changed
/// something it was not supposed to touch.
///
/// Not ignored. It is the gate the rest of the plan rests on and it costs one
/// scene build shared with the benches in this binary.
#[test]
fn the_store_changes_no_pixel() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let baseline = without_the_sun_store(&shader);
    let cube = body_cube();
    let bodies = bench_bodies(eye, 120.0, &cube, 33.0);

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        for (what, placed) in [("no bodies", &[][..]), ("sixteen bodies", &bodies[..])] {
            let build = |source: &str| {
                Prepared::new(
                    &device, source, "march", &tree, offset_from_clip, eye, w, h,
                    march_flags::DEFAULT, placed,
                )
            };
            let before = build(&baseline).read_back(&device, &queue);
            let after = build(&shader).read_back(&device, &queue);
            let differing = before
                .chunks_exact(4)
                .zip(after.chunks_exact(4))
                .filter(|(a, b)| a != b)
                .count();
            assert_eq!(
                differing, 0,
                "{w}x{h}, {what}: the sun store moved {differing} pixels, and nothing reads it",
            );
        }
    }
}

/// How many times the engine computes each sun answer today, which is the
/// number that predicts the entire win.
///
/// The store is keyed on `(voxel, face)` and nothing reads it, so after one
/// dispatch the occupied slots are the distinct voxel faces on screen. Pixels
/// that inserted, divided by those faces, is the redundancy factor: literally
/// how many pixels share one shadow ray's answer.
///
/// **Near 1 means there is nothing to collect** and the plan should stop here,
/// because a cache with no hits is all cost.
///
/// This reports and does not gate, beyond asserting something was inserted.
/// `nearly_every_hit_pixel_gets_a_slot` is the gate on the figure that caught
/// both of this work's silent defects, and it is not ignored.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench sun_store --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn the_sun_store_occupancy_is_reported() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");

    println!("\nthe sun store after one dispatch, {} slots:", bevox_render::pipeline::SUN_SLOTS);
    // Both cameras. The slots-per-face figure is the one that has to stay at
    // 1.000, and a spatially coherent hash clusters by design, so the distant
    // camera -- four times the distinct faces at 1080p, and the faces arriving
    // in spatial runs rather than scattered -- is where clustering would break
    // the dedup if it were going to.
    for (where_from, (eye, offset_from_clip)) in
        [("bench", bench_camera(extent)), ("distant", distant_camera(extent))]
    {
        for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
            let prepared = Prepared::new(
                &device, &shader, "march", &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            );
            prepared.clear_sun_pixel_slots(&queue);
            let store = prepared.sun_store(&device, &queue);
            let pixels = (w * h) as f64;
            println!(
                "  {where_from} {w}x{h}: {} pixels, {} inserted, {} slots occupied, {} distinct \
                 faces\n    \
                 load factor {:.4}\n    \
                 REDUNDANCY {:.2} pixels per distinct face ({:.2} counting sky pixels too)\n    \
                 {:.4} SLOTS PER FACE, so a pass over the occupied slots would compute each \
                 answer that many times and collect {:.2} pixels per answer instead",
                w * h,
                store.pixels_with_slot,
                store.occupied,
                store.distinct_keys,
                store.occupied as f64 / f64::from(bevox_render::pipeline::SUN_SLOTS),
                store.pixels_with_slot as f64 / store.distinct_keys.max(1) as f64,
                pixels / store.distinct_keys.max(1) as f64,
                store.occupied as f64 / store.distinct_keys.max(1) as f64,
                store.pixels_with_slot as f64 / store.occupied.max(1) as f64,
            );
            assert!(
                store.distinct_keys > 0,
                "nothing was inserted; the store is not being written",
            );
        }
    }
}

/// **What dispatching the sun pass over the whole table costs**, against the
/// pixel-bounded dispatch it replaced.
///
/// The pixel bound was wrong, not merely tight. `sun_compact` queues every slot
/// whose stamp is this frame's, and an 8-bit stamp makes that set the slots
/// claimed this frame *plus* those claimed exactly 255 frames ago -- up to twice
/// a frame's inserts -- so the work list is bounded by the table and not by the
/// screen. The tail a short dispatch drops carries correct keys beside older
/// frames' visibility bits. This bench is what it cost to make the bound
/// structural.
///
/// Both rows are one compiled module, one `Prepared` each, differing only in the
/// dispatch size, interleaved A/B/A in one invocation. The extra invocations are
/// 18,368 workgroups at 720p and 368 at 1080p, so the 720p row is the one with
/// anything to see.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench whole_table --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn what_the_whole_table_sun_dispatch_costs() {
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
    let slots = bevox_render::pipeline::SUN_SLOTS;

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        let build = || {
            Prepared::new(
                &device, &shader, "march_composite", &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            )
        };
        let whole = build();
        let bounded = build();
        bounded.force_sun_dispatch((w * h).min(slots));

        let (text, wall, gpu) = row_text(aba_both(&device, &queue, &bounded, &whole));
        let sun_bounded = pass_times(&device, &queue, &bounded)[3];
        let sun_whole = pass_times(&device, &queue, &whole)[3];
        println!(
            "\n{w}x{h}: the sun pass over {} invocations against {}\n  {text}\n  \
             whole-table dispatch costs {gpu:+.3} ms on the GPU clock ({wall:+.3} wall)\n  \
             the sun pass alone: {sun_whole:.3} ms against {sun_bounded:.3}, so \
             {:+.3} ms over {} extra workgroups",
            whole.sun_workgroups() * bevox_render::pipeline::SUN_PASS_WORKGROUP,
            bounded.sun_workgroups() * bevox_render::pipeline::SUN_PASS_WORKGROUP,
            sun_whole - sun_bounded,
            whole.sun_workgroups() - bounded.sun_workgroups(),
        );
    }
}

/// The hash break: the face's local index taken from two *fixed* axes instead of
/// the two tangent to the face, which is how the first brick index was wrong.
const SUN_LOCAL_INDEX: &str = "    let local = (u << SUN_BRICK_BITS) | v;";

/// The source with the local index wasting the axis a surface is flat in.
///
/// A floor varies in x and z and is constant in y, so `(x, y)` gives every face
/// of one floor patch the same low 3 bits and only 8 of its run's 64 slots are
/// ever reachable. That is the 3D brick index's failure in one line, and it is
/// what `nearly_every_hit_pixel_gets_a_slot` exists to catch.
fn with_the_flat_axis_wasted(shader: &str) -> String {
    assert_eq!(
        shader.matches(SUN_LOCAL_INDEX).count(),
        1,
        "the hash's local index moved; this break edits it by text",
    );
    shader.replace(SUN_LOCAL_INDEX, "    let local = (x << SUN_BRICK_BITS) | y;")
}

/// **Nearly every pixel that hits the static world must come away with a slot**,
/// at the shipped slot count, on the bench camera.
///
/// This is the metric that caught the two silent defects this work has had, and
/// until now it was asserted nowhere -- every use of it was a `println!` and the
/// one test that reported it was `#[ignore]`d behind `distinct_keys > 0`. Both
/// defects left the image perfectly correct, because a pixel with no slot
/// marches its own ray, and both deleted most of the saving:
///
/// - a 3D brick index wasted the axis a surface is flat in, so **51% of hit
///   pixels on the distant camera got no slot**;
/// - linear probing could not escape a clustered run in eight tries, which cost
///   17% on the near camera and 54% on the distant one.
///
/// 0.95 rather than 1.0 because eight probes at a 0.06 load factor is a
/// statistical thing, not a guaranteed one, and
/// `shrinking_the_table_until_it_overflows_is_measured` puts the first failed
/// insert at 2^19 -- 25 pixels of 485,361. The measured figure at 2^21 is 100%.
///
/// The threshold is proved against a break that reproduces the first defect in
/// one line, so the number is known to be able to fail.
///
/// Not ignored.
#[test]
fn nearly_every_hit_pixel_gets_a_slot() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let wasted = with_the_flat_axis_wasted(&shader);

    // Both cameras: the near one is the configuration the win is measured on,
    // the distant one is where both defects were four to five times worse.
    for (where_from, (eye, offset_from_clip)) in
        [("bench", bench_camera(extent)), ("distant", distant_camera(extent))]
    {
        for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
            let share = |source: &str| {
                let p = Prepared::new(
                    &device, source, "march_composite", &tree, offset_from_clip, eye, w, h,
                    march_flags::DEFAULT, &[],
                );
                // A pixel that inserted nothing has to be distinguishable from
                // one that took slot 0, and the record's done flag has to be
                // set before the primary clears it.
                p.clear_sun_pixel_slots(&queue);
                let store = p.sun_store(&device, &queue);
                let share = store.pixels_with_slot as f64 / store.static_hits.max(1) as f64;
                (store, share)
            };

            let (store, got) = share(&shader);
            eprintln!(
                "{where_from} {w}x{h}: {} of {} pixels that hit the static world got a slot \
                 ({:.1}%), filling {} slots with {} distinct faces",
                store.pixels_with_slot,
                store.static_hits,
                got * 100.0,
                store.occupied,
                store.distinct_keys,
            );
            assert!(
                store.static_hits > 0,
                "{where_from} {w}x{h}: no pixel recorded a static-world hit, so this gate is \
                 measuring nothing",
            );
            assert!(
                got >= 0.95,
                "{where_from} {w}x{h}: only {:.1}% of the {} pixels that hit the static world \
                 got a slot. The image is still correct -- they march their own ray -- and the \
                 saving the store exists for is gone for all of them",
                got * 100.0,
                store.static_hits,
            );
        }
    }

    // The break, at the one configuration it was worst on, so the threshold is
    // known to be able to fail rather than argued to be.
    let (eye, offset_from_clip) = distant_camera(extent);
    let (w, h) = (1920u32, 1080u32);
    let broken = {
        let p = Prepared::new(
            &device, &wasted, "march_composite", &tree, offset_from_clip, eye, w, h,
            march_flags::DEFAULT, &[],
        );
        p.clear_sun_pixel_slots(&queue);
        let store = p.sun_store(&device, &queue);
        store.pixels_with_slot as f64 / store.static_hits.max(1) as f64
    };
    eprintln!("with the flat axis wasted, distant {w}x{h}: {:.1}%", broken * 100.0);
    assert!(
        broken < 0.95,
        "the flat-axis break left {:.1}% of hit pixels with a slot, so it does not reproduce \
         the defect and the gate above is not known to test anything",
        broken * 100.0,
    );
}

/// What the store costs when nothing reads it, which is its overhead alone.
///
/// This revision is a deliberate regression: it pays for the atomic claim, the
/// key write and the per-pixel slot write, and shading still marches its own
/// shadow ray. Measuring here is the only chance to see the overhead on its
/// own, because once anything reads the store the two move together.
///
/// Interleaved A/B/A in one invocation at both resolutions, drift reported
/// beside the difference, per `docs/concepts/gpu-codegen-cliff.md`. The ceiling
/// this comes out of is the 7.98-8.36 ms the shadow march costs at 1080p.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench sun_store --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn what_the_sun_store_costs() {
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
    let baseline = without_the_sun_store(&shader);

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        let build = |source: &str| {
            Prepared::new(
                &device, source, "march", &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            )
        };
        let (a, b) = (build(&baseline), build(&shader));
        let (text, wall, gpu) = row_text(aba_both(&device, &queue, &a, &b));
        println!("\n{w}x{h}  insert on against off:\n  {text}");
        println!(
            "  the store costs {gpu:+.2} ms on the GPU clock ({wall:+.2} wall), against the \
             {:.2} ms ceiling the shadow march sets at 1080p",
            8.36,
        );
    }
}

/// The claim word's tag is what tells one face's slot from another's, and
/// reporting every tag equal is the break that shows it.
///
/// A claim word is this frame's stamp over 24 bits of the key's hash. Forcing
/// the tag to the same value for every face -- here by dropping it from the word
/// -- makes an insert accept the first slot claimed this frame that a probe
/// lands on, whoever owns it, so every face that collided is swallowed by the
/// face that got there first.
///
/// The number swallowed is the number of faces that would read another voxel's
/// answer, and it is reported rather than asserted away: a birthday count,
/// `n^2 / 2m` for `n` faces in `m` slots, about 2,100 of 94,000 at 1280x720
/// against 2^21 slots. **Not "most", which is what a table at a 4.5% load factor
/// in faces buys**, and not nothing either -- without the tag those thousands of
/// faces share an answer, and at 1080p there are four times as many.
///
/// The full key in `sun_table` is the second guard and it is checked by whoever
/// reads the store. Nothing reads it yet, so this revision cannot show either
/// guard as a broken image, and saying otherwise would be a lie. What it shows
/// is the sharing itself.
#[test]
fn forcing_every_key_to_match_shares_slots_between_voxels() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let blind = without_the_key_tag(&shader);

    let (w, h) = (1280u32, 720u32);
    let build = |source: &str| {
        Prepared::new(
            &device, source, "march", &tree, offset_from_clip, eye, w, h, march_flags::DEFAULT,
            &[],
        )
    };
    let honest = build(&shader).sun_store(&device, &queue);
    let shared = build(&blind).sun_store(&device, &queue);
    let swallowed = honest.distinct_keys.saturating_sub(shared.distinct_keys);
    let slots = f64::from(bevox_render::pipeline::SUN_SLOTS);
    let expected = honest.distinct_keys.pow(2) as f64 / (2.0 * slots);

    eprintln!(
        "tags kept: {} faces in {} slots. Every tag equal: {} faces, so {swallowed} voxel \
         faces took a slot another voxel owns -- a birthday count predicts {expected:.0}.",
        honest.distinct_keys, honest.occupied, shared.distinct_keys,
    );
    assert!(
        swallowed > 0,
        "with every tag reported equal, no face took another's slot; either nothing collides \
         or the tag is not what decides a slot's owner",
    );
    // Half the prediction, not the prediction: the count is a race between
    // pixels and the exact number moves between runs.
    assert!(
        (swallowed as f64) > expected * 0.5,
        "{swallowed} faces shared a slot against about {expected:.0} predicted; the collision \
         the tag guards is not happening at the rate the capacity implies",
    );
}

/// One slot per distinct face, and the race that cost three.
///
/// The insert's contract is one slot per distinct `(voxel, face)`. It holds
/// because the claim word carries the stamp and the key's tag together, so the
/// compare-exchange that claims a slot settles both questions at once and
/// nothing is written after it for a losing lane to half-read.
///
/// The break puts the earlier form back: the claim word is the stamp alone, and
/// an insert that finds this frame's stamp reads the 39-bit key out of
/// `sun_table` to decide whether the slot is its own. That read is unordered
/// against the winner's write of the same key, so a workgroup's lanes -- which
/// cover about seven faces between sixty-four of them -- see a fresh stamp
/// beside a stale key, call it someone else's slot, and probe on to claim
/// another for the same face. Every duplicate is correct and all but one is
/// waste: a pass dispatched over occupied slots would march every one of them.
///
/// **A measurement break, not an image break, and that is the honest shape for
/// this defect: it never changed a pixel.** It cost about 2.4 ms of the 8.36 ms
/// the shadow march is worth, which is not something an image can show.
#[test]
fn reintroducing_the_insert_race_costs_slots_per_face() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let raced = with_the_insert_race(&shader);

    let (w, h) = (1280u32, 720u32);
    let build = |source: &str| {
        Prepared::new(
            &device, source, "march", &tree, offset_from_clip, eye, w, h, march_flags::DEFAULT,
            &[],
        )
    };
    let fixed = build(&shader).sun_store(&device, &queue);
    let before = build(&raced).sun_store(&device, &queue);
    let per_face = |s: &SunStore| s.occupied as f64 / s.distinct_keys.max(1) as f64;

    eprintln!(
        "claim word carries the tag: {:.3} slots a face ({} slots, {} faces). Key read back \
         after the claim instead: {:.3} slots a face ({} slots, {} faces).",
        per_face(&fixed),
        fixed.occupied,
        fixed.distinct_keys,
        per_face(&before),
        before.occupied,
        before.distinct_keys,
    );
    assert!(
        per_face(&fixed) < 1.02,
        "{:.3} slots a face: the insert is claiming more than one slot per distinct face, \
         which is work a pass over occupied slots would repeat",
        per_face(&fixed),
    );
    assert!(
        per_face(&before) > 1.5,
        "putting the race back cost only {:.3} slots a face, against the 1.676 measured with \
         this hash, so this break does not reproduce the defect it guards and the gate above \
         is not known to test anything. The figure is hash-dependent -- it read 2.676 with the \
         scrambling hash -- so re-read it rather than lowering the threshold",
        per_face(&before),
    );
}

/// The composite pass's key comparison, which is the only thing that tells two
/// faces sharing a slot apart.
const SUN_KEY_CHECK: &str =
    "    if slot != SUN_NO_SLOT && sun_table[slot * 2u] == lo && sun_table[slot * 2u + 1u] == hi {";

/// The source with the read side's key verification dropped: a slot is believed
/// on the strength of the claim word's tag alone.
fn without_the_key_check(shader: &str) -> String {
    assert_eq!(
        shader.matches(SUN_KEY_CHECK).count(),
        1,
        "the composite's key check moved; this break edits it by text and must be updated \
         with it",
    );
    shader.replace(SUN_KEY_CHECK, "    if slot != SUN_NO_SLOT {")
}

/// The guard the compaction skips a free slot with, which is also where a break
/// can switch the sun pass off: an empty work list is a sun pass with nothing
/// to march.
const SUN_PASS_GUARD: &str = "    if slot >= sun_slots() { return; }";

/// The source with the compaction gathering nothing, so the sun pass is
/// dispatched and marches no ray. `sun_stamp` is never 0, so this returns every
/// time -- but it is a runtime value, so the code below it is not statically
/// dead and the driver still compiles it. Removing it from the source instead
/// would be measuring a different shader.
fn with_the_sun_pass_silenced(shader: &str) -> String {
    assert_eq!(
        shader.matches(SUN_PASS_GUARD).count(),
        1,
        "the compaction's guard moved; this break edits it by text",
    );
    shader.replace(
        SUN_PASS_GUARD,
        "    if slot >= sun_slots() || sun_stamp() != 0u { return; }",
    )
}

/// The three lines in the sun pass that actually march.
const SUN_PASS_MARCH: &str = concat!(
    "    var bit = 0u;\n",
    "    if shadowed(sun_key_origin(lo, hi), view.sun_direction.xyz, max_ray_distance()) {\n",
    "        bit = 1u;\n",
    "    }\n",
);

/// The source with the sun pass writing a constant bit instead of marching.
///
/// Both constants are *believed* by the composite, because the pass still
/// stamps the visibility word it writes -- which is what separates this break
/// from `with_the_sun_pass_silenced`, where the word is never written at all.
fn with_a_constant_sun(shader: &str, bit: u32) -> String {
    assert_eq!(
        shader.matches(SUN_PASS_MARCH).count(),
        1,
        "the sun pass's march moved; this break edits it by text",
    );
    shader.replace(SUN_PASS_MARCH, &format!("    var bit = {bit}u;\n"))
}

/// **The gate the whole plan rests on**: the three-pass lit path draws the same
/// image as the single-pass one, pixel for pixel, at both resolutions, with
/// bodies and without.
///
/// Sun visibility is already a per-voxel property -- `shadow_origin` snaps to
/// the voxel centre and offsets along the axis-aligned face normal -- so
/// computing it once per `(voxel, face)` and reading it back per pixel may not
/// move a pixel. A differing pixel is not a trade-off: it means the key does
/// not identify everything the ray depends on.
///
/// Both sides are the same shader module, so this cannot accidentally compare
/// two builds. `march` is left byte for byte as it was for exactly this reason.
///
/// **The split path is dispatched twice, under two different stamps, and both
/// images are compared.** Until this did so, nothing anywhere in the suite ever
/// ran the split path more than once: every image gate was a single dispatch on
/// a freshly zeroed store, so the state the app is in from frame 2 onward --
/// every slot already carrying a stamp, every claim word to be won again, the
/// visibility words holding the last frame's answers -- was a state no test had
/// rendered. That is the whole cross-frame class, and it is where an 8-bit
/// stamp's wrap lives.
///
/// Not ignored.
#[test]
fn the_split_pass_changes_no_pixel() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let cube = body_cube();

    // Both cameras, because they exercise different halves of the key. The
    // bench camera is close, where one face covers nine pixels and the store is
    // read back far more often than it is written. The distant one covers about
    // one pixel a face, which is four times as many distinct keys at 1080p and
    // so four times as much of the table -- and the nearest thing this suite
    // has to the aliasing case, where neighbouring pixels land on faces that
    // are neighbours in space and nothing alike in the hash.
    for (where_from, (eye, offset_from_clip)) in
        [("bench camera", bench_camera(extent)), ("distant camera", distant_camera(extent))]
    {
        let bodies = bench_bodies(eye, 120.0, &cube, 33.0);
        for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
            for (what, placed) in [("no bodies", &[][..]), ("sixteen bodies", &bodies[..])] {
                let build = |entry: &str| {
                    Prepared::new(
                        &device, &shader, entry, &tree, offset_from_clip, eye, w, h,
                        march_flags::DEFAULT, placed,
                    )
                };
                let before = build("march").read_back(&device, &queue);
                // One `Prepared`, two submits. Each advances the stamp, so the
                // second meets a store this same configuration filled under
                // the previous one.
                let split = build("march_composite");
                let frames =
                    [split.read_back(&device, &queue), split.read_back(&device, &queue)];
                for (frame, after) in frames.iter().enumerate() {
                    let differing = before
                        .chunks_exact(4)
                        .zip(after.chunks_exact(4))
                        .filter(|(a, b)| a != b)
                        .count();
                    assert_eq!(
                        differing, 0,
                        "{where_from} {w}x{h}, {what}, frame {}: the split pass moved \
                         {differing} pixels of {}; the key does not identify what the shadow \
                         ray depends on",
                        frame + 1,
                        w * h,
                    );
                }
            }
        }
    }
}

/// **The overflow fallback's image gate: a table far too small to hold the
/// scene draws the same picture.**
///
/// The B side is told the store has 2^14 slots for a view whose distinct voxel
/// faces number six figures, so the overwhelming majority of inserts spend
/// every probe and return `SUN_NO_SLOT`, and the pixels that do get a slot
/// mostly share it with another voxel's key. Both the give-up and the key
/// mismatch land in the composite's fallback, which marches the pixel's own
/// shadow ray, and the image may not move by one pixel.
///
/// This is the spec's overflow path gated by overflowing, rather than by
/// argument. Task 2 reached the same branch through a key break, which proved
/// the fallback is bit-identical but not that the table filling is what
/// reaches it.
///
/// No shader edit: the slot count travels in the uniform, so both sides are one
/// compiled module and the comparison cannot be a codegen difference in
/// disguise -- see `docs/concepts/gpu-codegen-cliff.md`.
///
/// `shrinking_the_table_until_it_overflows_is_measured` is the same condition
/// with the occupancy and the frame time.
///
/// Not ignored.
#[test]
fn overflowing_the_table_changes_no_pixel() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let cube = body_cube();
    let bodies = bench_bodies(eye, 120.0, &cube, 33.0);
    /// Slots the overflowing side is given: four orders of magnitude below the
    /// distinct faces either resolution produces, so inserts fail in bulk
    /// rather than at the margin.
    const SMALL: u32 = 1 << 14;

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        for (what, placed) in [("no bodies", &[][..]), ("sixteen bodies", &bodies[..])] {
            let build = |entry: &str, slots: u32| {
                Prepared::with_workgroup_and_slots(
                    &device, &shader, entry, &tree, offset_from_clip, eye, w, h,
                    march_flags::DEFAULT, placed, 8, slots,
                )
            };
            // One configuration alive at a time. Each holds a sun table of
            // `SUN_SLOTS * 4 + pixels * 7` words -- about 91 MB at 1080p -- and
            // this suite runs its GPU tests in parallel on a 4 GB card, so
            // three live `Prepared` here is an out-of-memory failure in every
            // other test as well as this one.
            //
            // The A side is the per-pixel path, which never reads the store, so
            // it is the same image at any capacity. The same configuration
            // gives the roomy occupancy, because the overflow has to be shown
            // to happen -- a bit-identity gate on a table that was never full
            // proves nothing.
            let (before, roomy) = {
                let full = build("march", bevox_render::pipeline::SUN_SLOTS);
                full.clear_sun_pixel_slots(&queue);
                let roomy = full.sun_store(&device, &queue).pixels_with_slot;
                (full.read_back(&device, &queue), roomy)
            };
            let cramped = {
                let probe = build("march", SMALL);
                probe.clear_sun_pixel_slots(&queue);
                probe.sun_store(&device, &queue).pixels_with_slot
            };
            assert!(
                cramped * 2 < roomy,
                "{w}x{h}, {what}: {cramped} of {roomy} pixels still got a slot at {SMALL} \
                 slots, so the table did not overflow and this gate proves nothing",
            );

            let after = build("march_composite", SMALL).read_back(&device, &queue);
            let differing = before
                .chunks_exact(4)
                .zip(after.chunks_exact(4))
                .filter(|(a, b)| a != b)
                .count();
            assert_eq!(
                differing, 0,
                "{w}x{h}, {what}: with the table overflowing ({cramped} of {roomy} pixels got a \
                 slot) {differing} pixels of {} moved; the fallback is not bit-identical",
                w * h,
            );
        }
    }
}

/// The read side's key check is load-bearing, and this is the image that shows
/// it.
///
/// `sun_claim` returns a slot on a matching claim word **without reading the
/// key**, because reading the key is what the insert race was. So until the
/// composite compares the full 39-bit key, two faces are told apart by the claim
/// word's 24-bit tag alone. Honestly tagged that is one pair in sixteen million
/// and no image could show it, so this break drops the tag from the claim word
/// as well -- which is Task 1's own break -- and thousands of faces then land in
/// a slot another voxel owns.
///
/// Two halves, and both matter:
///
/// - **Tag dropped, key check kept: the image is bit-identical.** Every face
///   that took another's slot sees the mismatch and marches its own ray, which
///   is what the guard is for.
/// - **Tag dropped, key check removed: the image changes.** Those faces read
///   another voxel's sun.
///
/// Nothing else in the suite exercises the full key any more: the insert stopped
/// reading keys when the race was fixed.
///
/// Not ignored.
#[test]
fn the_key_check_is_what_keeps_two_faces_apart() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let colliding = without_the_key_tag(&shader);
    let blind = without_the_key_check(&colliding);

    let (w, h) = (1280u32, 720u32);
    let build = |source: &str| {
        Prepared::new(
            &device, source, "march_composite", &tree, offset_from_clip, eye, w, h,
            march_flags::DEFAULT, &[],
        )
    };
    let honest = build(&shader).read_back(&device, &queue);
    let guarded = build(&colliding).read_back(&device, &queue);
    let unguarded = build(&blind).read_back(&device, &queue);
    let differing = |a: &[u8], b: &[u8]| {
        a.chunks_exact(4).zip(b.chunks_exact(4)).filter(|(x, y)| x != y).count()
    };

    let kept = differing(&honest, &guarded);
    let dropped = differing(&honest, &unguarded);
    eprintln!(
        "every tag reported equal, so thousands of faces share a slot: with the key check \
         {kept} pixels differ of {}, without it {dropped}.",
        w * h,
    );
    assert_eq!(
        kept, 0,
        "{kept} pixels moved with the key check in place; a face that took another's slot is \
         supposed to see the mismatch and march its own ray",
    );
    assert!(
        dropped > 0,
        "dropping the key check moved no pixel, so nothing in this scene reads a shared slot \
         and the guard is not known to be load-bearing",
    );
}

/// A cached value nothing computes is not a cache: the sun pass is what the
/// image depends on, twice over.
///
/// Writing a constant 1 shadows everything the store answers for; writing a
/// constant 0 lights it. Both still stamp the visibility word, so the composite
/// believes both, and the two images differ from today's and from each other --
/// which is what rules out a composite that quietly marched its own ray and
/// never read the store at all.
///
/// **The third case is the stamp guard, and it is the one that must move
/// nothing.** Switching the pass off leaves the visibility words unwritten, so
/// every slot's word carries a stamp that is not this frame's and the composite
/// marches its own ray for all of them. Before the stamp rode in that word, an
/// unwritten word read as *lit* and moved 16,264 pixels -- which is the same
/// failure an under-dispatched sun pass produces for the tail of its work list,
/// except silently and for a handful of pixels rather than visibly for all of
/// them. This asserting 0 is the belt to the key check's braces.
///
/// Not ignored.
#[test]
fn the_sun_pass_is_what_the_image_depends_on() {
    let Some((device, queue)) = gpu_device() else {
        eprintln!("no GPU adapter available, skipping");
        return;
    };
    let (tree, extent) = bench_scene();
    let (eye, offset_from_clip) = bench_camera(extent);
    let shader = std::fs::read_to_string("assets/shaders/march.wgsl").expect("shader missing");
    let shadowing = with_a_constant_sun(&shader, 1);
    let lighting = with_a_constant_sun(&shader, 0);
    let silent = with_the_sun_pass_silenced(&shader);

    let (w, h) = (1280u32, 720u32);
    let build = |source: &str| {
        Prepared::new(
            &device, source, "march_composite", &tree, offset_from_clip, eye, w, h,
            march_flags::DEFAULT, &[],
        )
    };
    let honest = build(&shader).read_back(&device, &queue);
    let all_dark = build(&shadowing).read_back(&device, &queue);
    let all_lit = build(&lighting).read_back(&device, &queue);
    let unanswered = build(&silent).read_back(&device, &queue);
    let differing = |a: &[u8], b: &[u8]| {
        a.chunks_exact(4).zip(b.chunks_exact(4)).filter(|(x, y)| x != y).count()
    };

    let dark = differing(&honest, &all_dark);
    let lit = differing(&honest, &all_lit);
    let between = differing(&all_dark, &all_lit);
    let stale = differing(&honest, &unanswered);
    eprintln!(
        "sun pass writing a constant 1: {dark} pixels differ of {}. Writing a constant 0: \
         {lit}. Between those two: {between}. Sun pass switched off, so no visibility word \
         carries this frame's stamp: {stale}.",
        w * h,
    );
    assert!(dark > 0, "a sun pass writing a constant moved no pixel, so nothing reads its bit");
    assert!(lit > 0, "a sun pass lighting everything moved no pixel, so nothing reads its bit");
    assert_ne!(
        dark, lit,
        "the two breaks moved the same number of pixels, which a shadowed-everything and a \
         lit-everything store should not",
    );
    assert!(
        between > 0,
        "shadowing everything and lighting everything drew the same image, so the composite \
         is not reading the visibility word it claims to",
    );
    assert_eq!(
        stale, 0,
        "with the sun pass switched off, {stale} pixels of {} shaded from a visibility word \
         this frame never wrote. The stamp the pass packs above the bit is what should send \
         every one of them to the fallback march instead",
        w * h,
    );
}

/// Median per-pass GPU time over seven readings, for one configuration:
/// beam, primary, compaction, sun, main.
fn pass_times(device: &wgpu::Device, queue: &wgpu::Queue, p: &Prepared) -> [f32; 5] {
    p.dispatch(device, queue);
    device.poll(wgpu::PollType::wait_indefinitely()).expect("poll");
    let t: Vec<_> =
        (0..7).map(|_| p.dispatch_timed(device, queue).expect("timestamps")).collect();
    let at = |f: fn(&GpuTime) -> f32| {
        let mut v: Vec<f32> = t.iter().map(f).collect();
        median(&mut v)
    };
    [at(|g| g.beam), at(|g| g.primary), at(|g| g.compact), at(|g| g.sun), at(|g| g.main)]
}

/// **What the plan actually collects**: the three-pass lit path against the
/// single-pass one, interleaved A/B/A in one invocation at both resolutions.
///
/// The ceiling is the 7.98-8.36 ms the shadow march costs at 1080p, measured by
/// `what_the_sun_shadow_ray_costs`. Out of it come the store's own overhead
/// (about +0.4 ms at 1080p, measured in Task 1 while nothing read it), the
/// per-pixel record the primary pass writes and the composite reads, and the
/// sun pass's dispatch over the whole table.
///
/// Three rows, so the sparse dispatch can be judged rather than assumed:
///
/// - the single-pass `march`, which is today;
/// - the four passes;
/// - the same four with the sun pass writing a constant bit instead of marching.
///   The gap between that and the real one is the marching, separated from the
///   rest of the restructure.
///
/// **That third row used to empty the work list instead, and it cannot any
/// more.** The composite now requires the visibility word to carry this frame's
/// stamp, so a slot the sun pass never wrote sends its pixels to the fallback
/// march -- which moves the shadow march into the composite rather than removing
/// it, and the row would have read as the restructure's cost while measuring
/// the march twice over. Writing a constant keeps the work list, the dispatch
/// and the stamped write and drops only the ray.
///
/// Run with `cargo test --release -p bevox_render --test gpu_bench collects --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn what_the_split_sun_pass_collects() {
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
    let rayless = with_a_constant_sun(&shader, 0);

    for &(w, h) in &[(1280u32, 720u32), (1920u32, 1080u32)] {
        let build = |source: &str, entry: &str| {
            Prepared::new(
                &device, source, entry, &tree, offset_from_clip, eye, w, h,
                march_flags::DEFAULT, &[],
            )
        };
        let single = build(&shader, "march");
        let split = build(&shader, "march_composite");
        let floor = build(&rayless, "march_composite");

        let (text, wall, gpu) = row_text(aba_both(&device, &queue, &single, &split));
        println!("\n{w}x{h}  four passes against one:\n  {text}");
        println!(
            "  the split collects {:+.2} ms on the GPU clock ({:+.2} wall) of the 8.36 ms the \
             shadow march is worth at 1080p",
            -gpu, -wall,
        );
        let (text, _, floor_gpu) = row_text(aba_both(&device, &queue, &single, &floor));
        println!("  the same four passes with the sun pass marching no ray:\n  {text}");
        println!(
            "  so the marching the sun pass does costs {:+.2} ms, and everything else about \
             the split costs {floor_gpu:+.2}",
            gpu - floor_gpu,
        );

        let one = pass_times(&device, &queue, &single);
        let three = pass_times(&device, &queue, &split);
        let empty = pass_times(&device, &queue, &floor);
        println!(
            "  per pass, GPU ms, median of 7:\n    \
             one:  beam {:.2} + march {:.2} = {:.2}\n    \
             four: beam {:.2} + primary {:.2} + compact {:.2} + sun {:.2} + composite {:.2} \
             = {:.2}\n    \
             with the sun pass marching no ray: compact {:.2} + sun {:.2}, over {} and {} \
             workgroups",
            one[0], one[4], one.iter().sum::<f32>(),
            three[0], three[1], three[2], three[3], three[4], three.iter().sum::<f32>(),
            empty[2], empty[3], split.compact_workgroups(), split.sun_workgroups(),
        );

        let store = {
            let p = build(&shader, "march");
            p.clear_sun_pixel_slots(&queue);
            p.sun_store(&device, &queue)
        };
        println!(
            "  the sun pass dispatches {} invocations for {} occupied slots: {:.2}% of them \
             have a ray to march, and {} pixels share each answer",
            bevox_render::pipeline::SUN_SLOTS,
            store.occupied,
            store.occupied as f64 / f64::from(bevox_render::pipeline::SUN_SLOTS) * 100.0,
            store.pixels_with_slot / store.distinct_keys.max(1),
        );
    }
}


