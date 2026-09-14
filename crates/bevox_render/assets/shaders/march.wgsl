// Voxel ray marcher.
//
// A direct port of the CPU reference in bevox_core::march: recursive descent
// expressed as an explicit stack, ray-box per child, children visited near to
// far. Optimisations belong in a later plan; this revision exists to be
// *correct*, and its correctness is pinned by a parity test against the CPU.

struct MarchUniform {
    world_from_clip: mat4x4<f32>,
    camera_position: vec4<f32>,
    sun_direction: vec4<f32>,
    volume_params: vec4<u32>,  // [depth, extent, flags, 0]
};

// Traversal optimisations, matching bevox_render::upload::march_flags. One
// binary renders both sides of every comparison, so a bit-identity test cannot
// accidentally compare two different builds.
const FLAG_DDA: u32 = 1u;
const FLAG_MASK_FILTER: u32 = 2u;
const FLAG_BEAM: u32 = 4u;

@group(0) @binding(0) var<uniform> view: MarchUniform;
@group(0) @binding(1) var<storage, read> nodes: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> voxels: array<u32>;
@group(0) @binding(3) var<storage, read> palette: array<vec4<f32>>;
@group(0) @binding(4) var output: texture_storage_2d<rgba8unorm, write>;
// Reachability masks as low/high halves, indexed cell * 8 + octant. WGSL has no
// 64-bit integer, so the u64 table is split rather than reshaped.
@group(0) @binding(5) var<storage, read> direction_masks: array<vec2<u32>>;
// Beam prepass results, one distance per coarse pixel, row major. A buffer
// rather than a second storage texture: one binding instead of two, and no
// read-access storage texture to negotiate with the adapter.
@group(0) @binding(6) var<storage, read_write> beam: array<f32>;

const BRICK_EDGE: u32 = 4u;
const CHILDREN: u32 = 64u;
const MAX_STEPS: u32 = 4096u;
const MAX_DEPTH: u32 = 8u;
/// Full-resolution pixels per beam sample, per axis.
const BEAM_SCALE: u32 = 8u;

fn flag_enabled(bit: u32) -> bool {
    return (view.volume_params.z & bit) != 0u;
}

fn node_mask_lo(n: vec4<u32>) -> u32 { return n.x; }
fn node_mask_hi(n: vec4<u32>) -> u32 { return n.y; }
fn node_child_base(n: vec4<u32>) -> u32 { return n.z; }
fn node_material(n: vec4<u32>) -> u32 { return n.w; }

fn is_subdivided(n: vec4<u32>) -> bool {
    return (node_mask_lo(n) | node_mask_hi(n)) != 0u;
}
fn is_empty(n: vec4<u32>) -> bool {
    return !is_subdivided(n) && node_material(n) == 0u;
}
fn is_uniform_solid(n: vec4<u32>) -> bool {
    return !is_subdivided(n) && node_material(n) != 0u;
}

/// Whether child `i` exists, reading the mask as two 32-bit halves.
fn has_child(n: vec4<u32>, i: u32) -> bool {
    if i < 32u {
        return (node_mask_lo(n) & (1u << i)) != 0u;
    }
    return (node_mask_hi(n) & (1u << (i - 32u))) != 0u;
}

/// Popcount of the mask bits below `i` — the same addressing the CPU uses.
fn child_slot(n: vec4<u32>, i: u32) -> u32 {
    var prefix: u32;
    if i < 32u {
        prefix = countOneBits(node_mask_lo(n) & ((1u << i) - 1u));
    } else {
        let low = countOneBits(node_mask_lo(n));
        var high_mask: u32 = 0u;
        if i > 32u {
            high_mask = (1u << (i - 32u)) - 1u;
        }
        prefix = low + countOneBits(node_mask_hi(n) & high_mask);
    }
    return node_child_base(n) + prefix;
}

fn voxel_byte(index: u32) -> u32 {
    return (voxels[index / 4u] >> ((index % 4u) * 8u)) & 0xFFu;
}

fn level_extent(level: u32) -> u32 {
    return 1u << (2u * (level + 1u));
}

struct Slab { hit: bool, t_enter: f32, t_exit: f32 };

fn ray_box(origin: vec3<f32>, inv_dir: vec3<f32>, lo: vec3<f32>, hi: vec3<f32>) -> Slab {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = min(t0, t1);
    let far = max(t0, t1);
    let t_enter = max(max(near.x, near.y), near.z);
    let t_exit = min(min(far.x, far.y), far.z);
    return Slab(t_enter <= t_exit && t_exit >= 0.0, t_enter, t_exit);
}

/// No DDA cell visited yet in this frame.
const NO_CURSOR: u32 = 0xFFFFFFFFu;

struct Frame {
    node: vec4<u32>,
    origin: vec3<u32>,
    level: u32,
    t_enter: f32,
    t_exit: f32,
    // Children of this frame already descended into, as two 32-bit halves.
    // Used by the scan path only; DDA visits each crossed cell once, in order,
    // so it has nothing to remember.
    visited_lo: u32,
    visited_hi: u32,
    // DDA resume point: the cell last descended into and its entry distance.
    // Storing the cursor rather than the whole DDA state keeps the frame small,
    // at the cost of recomputing three boundary distances on re-entry. A frame
    // twelve words wider would cost occupancy, which is the thing being bought.
    cursor: u32,
    cursor_t: f32,
};

/// Distance along the ray to each of this cell's exit planes.
fn cell_exits(
    origin: vec3<f32>, dir: vec3<f32>, inv_dir: vec3<f32>,
    node_lo: vec3<f32>, cell_size: f32, cell: vec3<i32>, stepv: vec3<i32>,
) -> vec3<f32> {
    // The exit plane is the far face in the direction of travel.
    let next = vec3<f32>(cell + max(stepv, vec3<i32>(0)));
    let boundary = node_lo + next * cell_size;
    let t = (boundary - origin) * inv_dir;
    // A zero direction component never crosses that axis. Left alone it would
    // produce a NaN or a negative infinity and win the minimum, stalling the walk.
    return vec3<f32>(
        select(t.x, 1e30, dir.x == 0.0),
        select(t.y, 1e30, dir.y == 0.0),
        select(t.z, 1e30, dir.z == 0.0),
    );
}

/// Distance at which the ray leaves this cell: the first exit plane it reaches.
fn cell_exit(t: vec3<f32>) -> f32 {
    return min(t.x, min(t.y, t.z));
}

/// The next cell along, stepping every axis whose exit plane is reached at the
/// same distance.
///
/// At an exact corner or edge crossing, stepping one axis at a time lands in a
/// cell the ray only touches and reaches the diagonal neighbour a step later.
/// Moving diagonally is both cheaper (no branches) and closer to what the ray
/// does; no test distinguishes the two, so this is robustness, not a fix.
fn step_cell(cell: vec3<i32>, t: vec3<f32>, t_out: f32, stepv: vec3<i32>) -> vec3<i32> {
    return cell + select(vec3<i32>(0), stepv, t == vec3<f32>(t_out));
}

fn index_of_cell(c: vec3<i32>) -> u32 {
    return u32(c.x) + u32(c.y) * BRICK_EDGE + u32(c.z) * BRICK_EDGE * BRICK_EDGE;
}

fn cell_of_index(i: u32) -> vec3<i32> {
    return vec3<i32>(
        i32(i % BRICK_EDGE),
        i32((i / BRICK_EDGE) % BRICK_EDGE),
        i32(i / (BRICK_EDGE * BRICK_EDGE)),
    );
}

fn direction_mask(cell: u32, octant: u32) -> vec2<u32> {
    return direction_masks[cell * 8u + octant];
}

/// Whether this ray, entering `node` at `cell`, can still reach any occupied
/// child of it. Conservative: a true answer does not promise a hit.
fn brick_reachable(node: vec4<u32>, cell: u32, octant: u32) -> bool {
    let m = direction_mask(cell, octant);
    return (node_mask_lo(node) & m.x) != 0u || (node_mask_hi(node) & m.y) != 0u;
}

/// Sign octant of a direction, matching bevox_core::mask_table::octant_index.
/// A zero component counts as positive, which keeps the mask conservative
/// instead of dropping a whole plane of cells.
fn octant_of(dir: vec3<f32>) -> u32 {
    var octant: u32 = 0u;
    if dir.x < 0.0 { octant = octant | 1u; }
    if dir.y < 0.0 { octant = octant | 2u; }
    if dir.z < 0.0 { octant = octant | 4u; }
    return octant;
}

struct Hit {
    hit: bool,
    material: u32,
    t: f32,
    voxel: vec3<u32>,
    face_normal: vec3<f32>,
};

/// Which face of a box the ray entered, from the per-axis entry distances.
/// Mirrors `entry_normal` in bevox_core::march.
fn entry_normal(origin: vec3<f32>, inv_dir: vec3<f32>, lo: vec3<f32>, hi: vec3<f32>) -> vec3<f32> {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = min(t0, t1);
    if near.x >= near.y && near.x >= near.z {
        if inv_dir.x >= 0.0 { return vec3<f32>(-1.0, 0.0, 0.0); }
        return vec3<f32>(1.0, 0.0, 0.0);
    }
    if near.y >= near.z {
        if inv_dir.y >= 0.0 { return vec3<f32>(0.0, -1.0, 0.0); }
        return vec3<f32>(0.0, 1.0, 0.0);
    }
    if inv_dir.z >= 0.0 { return vec3<f32>(0.0, 0.0, -1.0); }
    return vec3<f32>(0.0, 0.0, 1.0);
}

fn traverse(origin: vec3<f32>, dir: vec3<f32>, max_dist: f32) -> Hit {
    // Float division by zero yields infinity in WGSL, which is exactly what the
    // CPU reference stores for a zero direction component. Writing it as a plain
    // divide keeps both sides comparable instead of substituting a large finite.
    let inv_dir = vec3<f32>(1.0) / dir;

    let depth = view.volume_params.x;
    let extent = f32(view.volume_params.y);
    let root_slab = ray_box(origin, inv_dir, vec3<f32>(0.0), vec3<f32>(extent));
    if !root_slab.hit {
        return Hit(false, 0u, 0.0, vec3<u32>(0u), vec3<f32>(0.0));
    }

    // Element 0 of `nodes` is the root written by the uploader, so every arena
    // slot n lives at index n + 1.
    var stack: array<Frame, MAX_DEPTH>;
    var sp: u32 = 0u;
    var running: bool = true;
    stack[0] = Frame(
        nodes[0],
        vec3<u32>(0u),
        depth - 1u,
        max(root_slab.t_enter, 0.0),
        root_slab.t_exit,
        0u,
        0u,
        NO_CURSOR,
        0.0,
    );

    // Constant for the whole ray.
    let octant = octant_of(dir);

    var steps: u32 = 0u;

    loop {
        if !running { break; }
        steps = steps + 1u;
        if steps > MAX_STEPS { break; }

        let frame = stack[sp];

        if is_empty(frame.node) {
            if sp == 0u { running = false; } else { sp = sp - 1u; }
            continue;
        }
        if is_uniform_solid(frame.node) {
            let region = f32(level_extent(frame.level));
            let lo = vec3<f32>(frame.origin);
            let hi = lo + vec3<f32>(region);
            // A collapsed region covers many voxels: report the one actually
            // entered, not the region's origin. bevox_core::march does the same,
            // and the parity test pins it.
            let point = origin + dir * frame.t_enter;
            let entered = clamp(floor(point), lo, lo + vec3<f32>(region - 1.0));
            return Hit(
                true,
                node_material(frame.node),
                frame.t_enter,
                vec3<u32>(entered),
                entry_normal(origin, inv_dir, lo, hi),
            );
        }

        // Nearest child this ray crosses that this frame has not descended into.
        var best_i: u32 = CHILDREN;
        var best_t: f32 = 1e30;
        var best_exit: f32 = 0.0;
        var best_origin = vec3<u32>(0u);

        var step_size: u32 = 1u;
        if frame.level > 0u {
            step_size = level_extent(frame.level - 1u);
        }

        if !flag_enabled(FLAG_DDA) {
        for (var i: u32 = 0u; i < CHILDREN; i = i + 1u) {
            if !has_child(frame.node, i) { continue; }

            // Branch rather than select: select evaluates both arms, and the
            // high arm would shift by (i - 32) and underflow when i < 32.
            var taken: bool;
            if i < 32u {
                taken = (frame.visited_lo & (1u << i)) != 0u;
            } else {
                taken = (frame.visited_hi & (1u << (i - 32u))) != 0u;
            }
            if taken { continue; }

            let cx = i % BRICK_EDGE;
            let cy = (i / BRICK_EDGE) % BRICK_EDGE;
            let cz = i / (BRICK_EDGE * BRICK_EDGE);
            let child_origin = frame.origin + vec3<u32>(cx, cy, cz) * step_size;
            let lo = vec3<f32>(child_origin);
            let hi = lo + vec3<f32>(f32(step_size));
            let slab = ray_box(origin, inv_dir, lo, hi);
            if !slab.hit { continue; }
            if slab.t_exit < frame.t_enter || slab.t_enter > frame.t_exit { continue; }

            let t = max(slab.t_enter, frame.t_enter);
            if t < best_t {
                best_t = t;
                best_i = i;
                best_exit = slab.t_exit;
                best_origin = child_origin;
            }
        }

        } else {
            // DDA: step the ray through this node's children in order instead of
            // rescanning all 64 for the nearest unvisited one. Cells here are
            // equal sized and disjoint, so ray order and nearest-entry order are
            // the same, which is why output can stay bit-identical.
            let cell_size = f32(step_size);
            let node_lo = vec3<f32>(frame.origin);
            let stepv = vec3<i32>(
                select(-1, 1, dir.x > 0.0),
                select(-1, 1, dir.y > 0.0),
                select(-1, 1, dir.z > 0.0),
            );

            var cell: vec3<i32>;
            var t_cur: f32;
            if frame.cursor == NO_CURSOR {
                t_cur = frame.t_enter;
                let p = origin + dir * t_cur;
                cell = clamp(
                    vec3<i32>(floor((p - node_lo) / cell_size)),
                    vec3<i32>(0),
                    vec3<i32>(3),
                );
            } else {
                // Resuming: step past the cell already descended into.
                let resume = cell_of_index(frame.cursor);
                let tm = cell_exits(origin, dir, inv_dir, node_lo, cell_size, resume, stepv);
                t_cur = cell_exit(tm);
                cell = step_cell(resume, tm, t_cur, stepv);
            }

            // A ray crosses at most a handful of cells in a 4x4x4 grid; the
            // bound is generous and exists so a degenerate direction cannot spin.
            var guard: u32 = 0u;
            loop {
                guard = guard + 1u;
                if guard > CHILDREN { break; }
                if cell.x < 0 || cell.y < 0 || cell.z < 0
                    || cell.x > 3 || cell.y > 3 || cell.z > 3 { break; }
                if t_cur > frame.t_exit || t_cur > max_dist { break; }

                let i = index_of_cell(cell);
                let tm = cell_exits(origin, dir, inv_dir, node_lo, cell_size, cell, stepv);
                let t_out = cell_exit(tm);

                if has_child(frame.node, i) {
                    best_i = i;
                    best_t = max(t_cur, frame.t_enter);
                    best_exit = min(t_out, frame.t_exit);
                    best_origin = frame.origin + vec3<u32>(cell) * step_size;
                    stack[sp].cursor = i;
                    stack[sp].cursor_t = t_cur;
                    break;
                }

                t_cur = t_out;
                cell = step_cell(cell, tm, t_out, stepv);
            }
        }

        if best_i == CHILDREN || best_t > max_dist {
            if sp == 0u { running = false; } else { sp = sp - 1u; }
            continue;
        }

        // Mark it visited in the stored frame, not the local copy. DDA needs
        // no such record: it visits each crossed cell once, in order.
        if flag_enabled(FLAG_DDA) {
            // nothing to record
        } else if best_i < 32u {
            stack[sp].visited_lo = stack[sp].visited_lo | (1u << best_i);
        } else {
            stack[sp].visited_hi = stack[sp].visited_hi | (1u << (best_i - 32u));
        }

        let slot = child_slot(frame.node, best_i);

        if frame.level == 0u {
            let lo = vec3<f32>(best_origin);
            let hi = lo + vec3<f32>(1.0);
            return Hit(
                true,
                voxel_byte(slot),
                best_t,
                best_origin,
                entry_normal(origin, inv_dir, lo, hi),
            );
        }

        let child = nodes[slot + 1u];

        // Skip a child the ray cannot reach anything inside of. Only subdivided
        // children carry an occupancy mask; a uniform solid's mask is zero and
        // the filter would erase it from the scene.
        if flag_enabled(FLAG_MASK_FILTER) && is_subdivided(child) {
            let child_cell_size = f32(step_size) * 0.25;
            let point = origin + dir * best_t;
            let cell = clamp(
                vec3<i32>(floor((point - vec3<f32>(best_origin)) / child_cell_size)),
                vec3<i32>(0),
                vec3<i32>(3),
            );
            // The child is already recorded as visited above, so continuing
            // re-enters this frame and moves on to the next child.
            if !brick_reachable(child, index_of_cell(cell), octant) {
                continue;
            }
        }

        sp = sp + 1u;
        stack[sp] = Frame(
            child,
            best_origin,
            frame.level - 1u,
            best_t,
            min(best_exit, frame.t_exit),
            0u,
            0u,
            NO_CURSOR,
            0.0,
        );
    }

    return Hit(false, 0u, 0.0, vec3<u32>(0u), vec3<f32>(0.0));
}

/// Material at a voxel coordinate, or 0 outside the volume.
///
/// This is the tree descent from Contree::get: at each level pick the child
/// containing the coordinate, stopping early at an unsubdivided node. An empty
/// node's material is 0 and a uniform solid's is its own, so the single early
/// return covers both without conflating them.
fn material_at(p: vec3<i32>) -> u32 {
    let extent = i32(view.volume_params.y);
    if p.x < 0 || p.y < 0 || p.z < 0 || p.x >= extent || p.y >= extent || p.z >= extent {
        return 0u;
    }

    var node = nodes[0];
    var level = view.volume_params.x - 1u;
    var local = vec3<u32>(p);

    // Bounded by tree depth; a volume is never deeper than MAX_DEPTH.
    for (var guard: u32 = 0u; guard < MAX_DEPTH; guard = guard + 1u) {
        if !is_subdivided(node) {
            return node_material(node);
        }

        if level == 0u {
            let i = local.x + local.y * BRICK_EDGE + local.z * BRICK_EDGE * BRICK_EDGE;
            if !has_child(node, i) {
                return 0u;
            }
            return voxel_byte(child_slot(node, i));
        }

        let step_size = level_extent(level - 1u);
        let cell = local / step_size;
        let i = cell.x + cell.y * BRICK_EDGE + cell.z * BRICK_EDGE * BRICK_EDGE;
        if !has_child(node, i) {
            return 0u;
        }
        node = nodes[child_slot(node, i) + 1u];
        local = local - cell * step_size;
        level = level - 1u;
    }
    return 0u;
}

/// Sums the directions in which a voxel is exposed, falling back to the entry
/// face when that carries no information. Mirrors bevox_core::normal.
fn implicit_normal(voxel: vec3<u32>, face_normal: vec3<f32>) -> vec3<f32> {
    let base = vec3<i32>(voxel);
    var sum = vec3<f32>(0.0);

    if material_at(base + vec3<i32>(1, 0, 0)) == 0u { sum += vec3<f32>(1.0, 0.0, 0.0); }
    if material_at(base + vec3<i32>(-1, 0, 0)) == 0u { sum += vec3<f32>(-1.0, 0.0, 0.0); }
    if material_at(base + vec3<i32>(0, 1, 0)) == 0u { sum += vec3<f32>(0.0, 1.0, 0.0); }
    if material_at(base + vec3<i32>(0, -1, 0)) == 0u { sum += vec3<f32>(0.0, -1.0, 0.0); }
    if material_at(base + vec3<i32>(0, 0, 1)) == 0u { sum += vec3<f32>(0.0, 0.0, 1.0); }
    if material_at(base + vec3<i32>(0, 0, -1)) == 0u { sum += vec3<f32>(0.0, 0.0, -1.0); }

    if dot(sum, sum) < 1e-6 {
        return face_normal;
    }
    return normalize(sum);
}

/// How far a ray may travel before giving up.
///
/// Derived from the volume rather than fixed: a camera framed at a few extents
/// away from a 4096 scene sits ~4500 units out, so a hardcoded budget of 1000
/// kills every ray before it reaches the geometry and the screen shows only sky.
/// Rays terminate at the volume's exit anyway, so a generous bound costs
/// nothing; the step cap is what bounds the work.
fn max_ray_distance() -> f32 {
    return max(f32(view.volume_params.y) * 8.0, 1000.0);
}

/// Ray direction for a pixel. Shared so both entry points march identical rays.
fn primary_ray(id: vec3<u32>, size: vec2<u32>) -> vec3<f32> {
    let ndc = vec2<f32>(
        (f32(id.x) + 0.5) / f32(size.x) * 2.0 - 1.0,
        1.0 - (f32(id.y) + 0.5) / f32(size.y) * 2.0,
    );
    let far = view.world_from_clip * vec4<f32>(ndc, 1.0, 1.0);
    return normalize(far.xyz / far.w - view.camera_position.xyz);
}

/// Whether anything is hit within `max_dist`. Shadow rays do not care which
/// voxel occludes them, only that one does.
///
/// A thin wrapper rather than a second traversal: duplicating the walk would let
/// the two drift apart, and the shadow path would stop being covered by the
/// parity test that guards the primary one.
fn traverse_any(origin: vec3<f32>, dir: vec3<f32>, max_dist: f32) -> bool {
    return traverse(origin, dir, max_dist).hit;
}

fn beam_dims(size: vec2<u32>) -> vec2<u32> {
    return max((size + vec2<u32>(BEAM_SCALE - 1u)) / BEAM_SCALE, vec2<u32>(1u));
}

/// Distance at which one voxel shrinks to the spacing between beam rays.
///
/// Past this a voxel can sit entirely between two beams and be missed, so the
/// seed is capped here whatever the beam actually found. Two adjacent beam
/// directions give the spacing directly, which avoids passing a field of view
/// through the uniform and going stale when the projection changes.
fn beam_safe_distance(bd: vec2<u32>) -> f32 {
    let a = primary_ray(vec3<u32>(0u, 0u, 0u), bd);
    let b = primary_ray(vec3<u32>(1u, 0u, 0u), bd);
    let spread = length(b - a);
    if spread <= 1e-9 {
        return 1e30;
    }
    return 1.0 / spread;
}

/// Coarse pass: how far each beam travelled before meeting anything.
@compute @workgroup_size(8, 8, 1)
fn beam_prepass(@builtin(global_invocation_id) id: vec3<u32>) {
    let bd = beam_dims(textureDimensions(output));
    if id.x >= bd.x || id.y >= bd.y { return; }

    let hit = traverse(view.camera_position.xyz, primary_ray(id, bd), max_ray_distance());
    let cap = beam_safe_distance(bd);
    var seed = cap;
    if hit.hit {
        seed = min(hit.t, cap);
    }
    beam[id.y * bd.x + id.x] = seed;
}

/// Smallest distance any surrounding beam reported.
///
/// The minimum over a 3x3 neighbourhood is what makes this safe: a pixel sits
/// anywhere inside its beam's cell, so the geometry it is about to hit may have
/// been seen by the beam on either side, not only by its own or the next one.
fn beam_seed(id: vec3<u32>, size: vec2<u32>) -> f32 {
    let bd = beam_dims(size);
    let centre = vec2<i32>(id.xy / BEAM_SCALE);
    var m = 1e30;
    for (var dy = -1; dy <= 1; dy = dy + 1) {
        for (var dx = -1; dx <= 1; dx = dx + 1) {
            let s = clamp(centre + vec2<i32>(dx, dy), vec2<i32>(0), vec2<i32>(bd) - vec2<i32>(1));
            m = min(m, beam[u32(s.y) * bd.x + u32(s.x)]);
        }
    }
    return m;
}

/// The camera ray for this pixel, started wherever the prepass proved empty.
///
/// The seed offsets the ray origin rather than being threaded into `traverse`
/// as a start parameter. `traverse` is the function every parity test pins;
/// a seed inside it would put the optimisation within the thing being verified.
fn primary_hit(id: vec3<u32>, size: vec2<u32>) -> Hit {
    let dir = primary_ray(id, size);
    var t_seed = 0.0;
    if flag_enabled(FLAG_BEAM) {
        t_seed = beam_seed(id, size);
    }
    var hit = traverse(
        view.camera_position.xyz + dir * t_seed,
        dir,
        max_ray_distance() - t_seed,
    );
    // Measured from the offset origin; callers want it absolute.
    hit.t = hit.t + t_seed;
    return hit;
}

/// Shadow flag in red: 255 shadowed, 0 lit. Parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_shadow(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        let n = implicit_normal(hit.voxel, hit.face_normal);
        // Offset along the normal so the ray does not immediately re-hit its own
        // voxel. 0.75 matches bevox_core's reference renderer.
        let origin = vec3<f32>(hit.voxel) + vec3<f32>(0.5) + n * 0.75;
        var shadowed = 0.0;
        if traverse_any(origin, view.sun_direction.xyz, max_ray_distance()) {
            shadowed = 1.0;
        }
        colour = vec4<f32>(shadowed, 0.0, 0.0, 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// Normal encoded into unsigned bytes, hit flag in alpha. Parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_normal(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        let n = implicit_normal(hit.voxel, hit.face_normal);
        colour = vec4<f32>(n * 0.5 + vec3<f32>(0.5), 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// Hit voxel coordinate in RGB, hit flag in alpha. Read by the parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_voxel_id(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        // Volumes in these tests are at most 256 per axis, so a byte each.
        colour = vec4<f32>(
            f32(hit.voxel.x) / 255.0,
            f32(hit.voxel.y) / 255.0,
            f32(hit.voxel.z) / 255.0,
            1.0,
        );
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// Material identity in red, hit flag in green. Read by the parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_identity(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    if hit.hit {
        colour = vec4<f32>(f32(hit.material) / 255.0, 1.0, 0.0, 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// What the window shows: palette colour, diffuse from the implicit normal, and
/// a shadow ray toward the sun.
@compute @workgroup_size(8, 8, 1)
fn march(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec3<f32>(0.35, 0.47, 0.70);  // sky
    if hit.hit {
        let n = implicit_normal(hit.voxel, hit.face_normal);
        let sun = view.sun_direction.xyz;

        let origin = vec3<f32>(hit.voxel) + vec3<f32>(0.5) + n * 0.75;
        var diffuse = max(dot(n, sun), 0.0) * 0.75;
        if traverse_any(origin, sun, max_ray_distance()) {
            diffuse = 0.0;
        }

        let base = palette[hit.material].rgb;
        colour = base * (0.25 + diffuse);
    }
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(colour, 1.0));
}
