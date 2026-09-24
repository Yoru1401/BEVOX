// Voxel ray marcher.
//
// A direct port of the CPU reference in bevox_core::march: recursive descent
// expressed as an explicit stack, ray-box per child, children visited near to
// far. Optimisations belong in a later plan; this revision exists to be
// *correct*, and its correctness is pinned by a parity test against the CPU.

struct MarchUniform {
    // Clip space to the world-space offset from camera_position: no translation.
    offset_from_clip: mat4x4<f32>,
    camera_position: vec4<f32>,
    sun_direction: vec4<f32>,
    volume_params: vec4<u32>,  // [depth, extent, flags, 0]
    // [field_edge, field_cell_size, shadow caster count, where the casters start]
    field_params: vec4<u32>,
    // [the word the fullness grid starts at inside `distance_field`, 0, 0, 0]
    ao_params: vec4<u32>,
};

// Traversal optimisations, matching bevox_render::upload::march_flags. One
// binary renders both sides of every comparison, so a bit-identity test cannot
// accidentally compare two different builds.
const FLAG_DDA: u32 = 1u;
const FLAG_MASK_FILTER: u32 = 2u;
const FLAG_BEAM: u32 = 4u;
const FLAG_DISTANCE_FIELD: u32 = 8u;
const FLAG_BODIES: u32 = 16u;
const FLAG_BODY_RECT: u32 = 64u;
const FLAG_BODY_SHADOWS: u32 = 128u;
const FLAG_AO: u32 = 256u;

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
// Distance field: Chebyshev distance to the nearest solid voxel per coarse
// cell, four cells packed per word.
@group(0) @binding(7) var<storage, read> distance_field: array<u32>;

// A rigid body's placement and where its own geometry lives in the shared
// node/voxel buffers. Walked by compose_bodies, alongside the static world.
struct GpuBody {
    local_from_world: mat4x4<f32>,
    rotation: mat4x4<f32>,
    node_base: u32,
    voxel_base: u32,
    depth: u32,
    extent: u32,
    // World bounding sphere, centre and radius: shadow casters only.
    bound: vec4<f32>,
};
@group(0) @binding(8) var<storage, read> bodies: array<GpuBody>;
// Which marched-table entry produced this invocation's primary hit, when a body
// did. Set by `compose_bodies`, read by `shadow_origin`. Kept out of `Hit`, which
// every traversal builds and returns, so the traversal is untouched by it.
var<private> hit_body: u32;
// Each body's screen footprint in pixels, inclusive, in the same order as
// `bodies`. A separate 16-byte array rather than a field of GpuBody, so testing
// it cannot load the whole 160-byte entry. The eighth storage buffer in the
// compute stage, wgpu's default limit: no room for another without raising it.
struct GpuBodyRect { min: vec2<u32>, max: vec2<u32> };
@group(0) @binding(9) var<storage, read> body_rects: array<GpuBodyRect>;

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

/// `voxel_byte`, but starting from a volume's own word offset in the shared
/// buffer rather than word 0. `voxel_base` is in words: each packed volume
/// begins on a word boundary, so only the word half of the split needs it.
fn voxel_byte_at(voxel_base: u32, index: u32) -> u32 {
    return (voxels[voxel_base + index / 4u] >> ((index % 4u) * 8u)) & 0xFFu;
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
    // DDA resume point, packed by `pack_cursor`: the cell the ray was in at the
    // distance last descended at, the tie state there and the child descended
    // into; and that distance. Storing the cursor rather than the whole DDA
    // state keeps the frame small, at the cost of recomputing three boundary
    // distances on re-entry. A frame twelve words wider would cost occupancy,
    // which is the thing being bought.
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
/// At an exact corner or edge crossing this moves diagonally, into the cell the
/// ray goes on through. The cells it only touches there are not skipped:
/// `touched_child` visits them, because the scan path and the CPU reference
/// count a touch as a crossing.
fn step_cell(cell: vec3<i32>, t: vec3<f32>, t_out: f32, stepv: vec3<i32>) -> vec3<i32> {
    return cell + select(vec3<i32>(0), stepv, t == vec3<f32>(t_out));
}

/// Axes whose exit plane is reached at `t_out`, as bits x = 1, y = 2, z = 4.
fn axis_bits(hit: vec3<bool>) -> u32 {
    return select(0u, 1u, hit.x) | select(0u, 2u, hit.y) | select(0u, 4u, hit.z);
}

fn in_node(c: vec3<i32>) -> bool {
    return all(c >= vec3<i32>(0)) && all(c <= vec3<i32>(3));
}

/// Tie state for DDA: bits 0-2 are the axes whose planes into the current cell
/// the ray crosses at exactly the current distance, and bit 3 marks a frame's
/// entry, where the cell behind across all of them is touched too.
const TIE_ENTRY: u32 = 8u;

/// The occupied child the scan path would take next among the cells the ray
/// touches at the distance it enters `cell`, or CHILDREN for none. Children
/// with an index below `after` were already descended into.
///
/// The scan and the CPU reference take every child whose slab the ray touches,
/// nearest first and, at equal distance, lowest index first. At an exact edge
/// or corner crossing several children share that distance: `cell` and, for
/// each subset of the tied axes, the cell behind it across those planes, which
/// the ray only grazes. Behind across all of them is the cell the ray came from,
/// entered earlier -- except on entering a frame, where it is grazed too.
///
/// Those cells are a box, one or two coordinates per axis, so they are built as
/// a 64-bit child mask and the lowest set bit left after the node's mask and
/// `after` is the answer. No loop and no branch per cell, on purpose: written as
/// a loop over the subsets of the tied axes, this function slowed the GTX 1650
/// driver's code for the whole traversal -- the scan path, which never calls it,
/// went from 38 to 57 ms at 1280x720, and DDA from 13 to 18. Unrolling that loop
/// was slower still. Rare as ties are, the size of this code is what costs.
fn touched_child(node: vec4<u32>, cell: vec3<i32>, stepv: vec3<i32>, ties: u32, after: u32) -> u32 {
    let tied = (vec3<u32>(ties) & vec3<u32>(1u, 2u, 4u)) != vec3<u32>(0u);
    let xs = coord_bit(cell.x) | select(0u, coord_bit(cell.x - stepv.x), tied.x);
    let ys = coord_bit(cell.y) | select(0u, coord_bit(cell.y - stepv.y), tied.y);
    let zs = coord_bit(cell.z) | select(0u, coord_bit(cell.z - stepv.z), tied.z);
    // Index x + 4y + 16z: rows of x replicated at each y, planes at each z.
    let plane = xs * spread4(ys);
    var lo = select(0u, plane, (zs & 1u) != 0u) | select(0u, plane << 16u, (zs & 2u) != 0u);
    var hi = select(0u, plane, (zs & 4u) != 0u) | select(0u, plane << 16u, (zs & 8u) != 0u);
    let came_from = cell - stepv * vec3<i32>(tied);
    if (ties & 7u) != 0u && (ties & TIE_ENTRY) == 0u && in_node(came_from) {
        let i = index_of_cell(came_from);
        if i < 32u { lo = lo & ~(1u << i); } else { hi = hi & ~(1u << (i - 32u)); }
    }
    // Indices from `after` up. Shift amounts are masked and the arms selected,
    // because `select` evaluates both and a shift of 32 or more is invalid.
    lo = lo & node_mask_lo(node) & select(0u, ~0u << (after & 31u), after < 32u);
    hi = hi & node_mask_hi(node)
        & select(select(0u, ~0u << ((after - 32u) & 31u), after < 64u), ~0u, after < 32u);
    if lo != 0u { return firstTrailingBit(lo); }
    if hi != 0u { return 32u + firstTrailingBit(hi); }
    return CHILDREN;
}

/// One bit for which of a node's coordinates 0..3 `c` is, or 0 outside them.
fn coord_bit(c: i32) -> u32 {
    return ((1u << u32(clamp(c + 1, 0, 5))) >> 1u) & 15u;
}

/// A 4-bit coordinate set, bit k moved to bit 4k.
fn spread4(b: u32) -> u32 {
    return (b & 1u) | ((b & 2u) << 3u) | ((b & 4u) << 6u) | ((b & 8u) << 9u);
}

/// Packs a DDA resume point. The cell may lie one outside the node on any axis
/// -- a crossing out of the node can still graze a cell inside it -- so each
/// coordinate is stored offset by one, in three bits.
fn pack_cursor(cell: vec3<i32>, ties: u32, after: u32) -> u32 {
    let c = vec3<u32>(cell + vec3<i32>(1));
    return c.x | (c.y << 3u) | (c.z << 6u) | (ties << 9u) | (after << 13u);
}

fn cursor_cell(cursor: u32) -> vec3<i32> {
    return vec3<i32>(
        i32(cursor & 7u),
        i32((cursor >> 3u) & 7u),
        i32((cursor >> 6u) & 7u),
    ) - vec3<i32>(1);
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
    // Set when a body, not the static world, produced this hit. Its `voxel` is
    // then a coordinate in the body's own volume and means nothing against the
    // static tree or as a world position.
    from_body: bool,
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

/// `traverse`, told where a volume's nodes and voxels begin.
///
/// The static world is this with both bases zero. A body's root sits at
/// `node_base` and its arena slot n at `node_base + 1 + n`, which is the same
/// root-first layout the static world uses, just offset.
///
/// `t_start` is where along the ray the walk begins, from `origin` itself: a
/// seed that proved the space before it empty. Every distance is still measured
/// from `origin`, so each plane is reached at the same float distance with or
/// without the seed. A walk from an origin moved to the seed point rounds those
/// distances differently, and where a ray crosses a voxel edge exactly that
/// picked a different voxel.
fn traverse_at(
    origin: vec3<f32>, dir: vec3<f32>, t_start: f32, max_dist: f32,
    node_base: u32, voxel_base: u32, depth: u32, extent: u32,
) -> Hit {
    // Float division by zero yields infinity in WGSL, which is exactly what the
    // CPU reference stores for a zero direction component. Writing it as a plain
    // divide keeps both sides comparable instead of substituting a large finite.
    let inv_dir = vec3<f32>(1.0) / dir;

    let root_slab = ray_box(origin, inv_dir, vec3<f32>(0.0), vec3<f32>(f32(extent)));
    if !root_slab.hit || t_start > root_slab.t_exit {
        return Hit(false, 0u, 0.0, vec3<u32>(0u), vec3<f32>(0.0), false);
    }

    // Index `node_base` is the root written by the uploader, so every arena
    // slot n lives at index node_base + 1 + n.
    var stack: array<Frame, MAX_DEPTH>;
    var sp: u32 = 0u;
    var running: bool = true;
    stack[0] = Frame(
        nodes[node_base],
        vec3<u32>(0u),
        depth - 1u,
        max(root_slab.t_enter, t_start),
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
                false,
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

            // `cell` is the cell the ray is in just after `t_cur`; `ties` and
            // `after` say which cells it touches at `t_cur` are still to visit.
            var cell: vec3<i32>;
            var t_cur: f32;
            var ties: u32;
            var after: u32;
            if frame.cursor == NO_CURSOR {
                t_cur = frame.t_enter;
                let p = origin + dir * t_cur;
                cell = clamp(
                    vec3<i32>(floor((p - node_lo) / cell_size)),
                    vec3<i32>(0),
                    vec3<i32>(3),
                );
                // Entering exactly on an inner plane grazes the cell behind it.
                // The entering axis itself always lands on the node's own outer
                // face (index 0 or 4), never an inner one, and recomputing its
                // distance here reproduces `t_cur` bit for bit -- not a real tie,
                // just the same float formula read twice. Restricting to inner
                // planes (1..3) keeps the real ties and drops that false one,
                // which otherwise ran the subset loop below on every entry.
                let k = cell + max(-stepv, vec3<i32>(0));
                let inner = k > vec3<i32>(0) & k < vec3<i32>(4);
                let near = node_lo + vec3<f32>(k) * cell_size;
                ties = axis_bits(inner & ((near - origin) * inv_dir == vec3<f32>(t_cur))) | TIE_ENTRY;
                after = 0u;
            } else {
                // Resuming at the distance last descended at, past that child.
                cell = cursor_cell(frame.cursor);
                ties = (frame.cursor >> 9u) & 15u;
                after = (frame.cursor >> 13u) & 127u;
                t_cur = frame.cursor_t;
            }

            // A ray crosses at most a handful of cells in a 4x4x4 grid; the
            // bound is generous and exists so a degenerate direction cannot spin.
            var guard: u32 = 0u;
            loop {
                guard = guard + 1u;
                if guard > CHILDREN { break; }
                if t_cur > frame.t_exit || t_cur > max_dist { break; }

                let i = touched_child(frame.node, cell, stepv, ties, after);
                if i != CHILDREN {
                    let child_cell = cell_of_index(i);
                    best_i = i;
                    best_t = max(t_cur, frame.t_enter);
                    if all(child_cell == cell) {
                        let tm = cell_exits(origin, dir, inv_dir, node_lo, cell_size, cell, stepv);
                        best_exit = min(cell_exit(tm), frame.t_exit);
                    } else {
                        // Only grazed: its slab ends where it begins.
                        best_exit = min(t_cur, frame.t_exit);
                    }
                    best_origin = frame.origin + vec3<u32>(child_cell) * step_size;
                    stack[sp].cursor = pack_cursor(cell, ties, i + 1u);
                    stack[sp].cursor_t = t_cur;
                    break;
                }

                if !in_node(cell) { break; }
                let tm = cell_exits(origin, dir, inv_dir, node_lo, cell_size, cell, stepv);
                let t_out = cell_exit(tm);
                let crossed = axis_bits(tm == vec3<f32>(t_out));
                // One plane crossed grazes nothing new: behind it is this cell.
                ties = select(0u, crossed, countOneBits(crossed) > 1u);
                after = 0u;
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
                voxel_byte_at(voxel_base, slot),
                best_t,
                best_origin,
                entry_normal(origin, inv_dir, lo, hi),
                false,
            );
        }

        let child = nodes[node_base + 1u + slot];

        // Skip a child the ray cannot reach anything inside of. Only subdivided
        // children carry an occupancy mask; a uniform solid's mask is zero and
        // the filter would erase it from the scene.
        if flag_enabled(FLAG_MASK_FILTER) && is_subdivided(child) {
            let child_cell_size = f32(step_size) * 0.25;
            let child_lo = vec3<f32>(best_origin);
            let point = origin + dir * best_t;
            let entered = clamp(
                vec3<i32>(floor((point - child_lo) / child_cell_size)),
                vec3<i32>(0),
                vec3<i32>(3),
            );
            // Entering exactly on an inner plane grazes the cell behind it, as
            // DDA's entry does: ask from that cell, which reaches everything the
            // one ahead does and itself, or the grazed voxel is filtered away.
            let stepv = vec3<i32>(
                select(-1, 1, dir.x > 0.0),
                select(-1, 1, dir.y > 0.0),
                select(-1, 1, dir.z > 0.0),
            );
            let near = child_lo + vec3<f32>(entered + max(-stepv, vec3<i32>(0))) * child_cell_size;
            let grazed = (near - origin) * inv_dir == vec3<f32>(best_t);
            let cell = clamp(
                entered - select(vec3<i32>(0), stepv, grazed),
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

    return Hit(false, 0u, 0.0, vec3<u32>(0u), vec3<f32>(0.0), false);
}

/// The static world, at its fixed place in the shared buffers. Its signature
/// has already grown once, to take the seed distance and the ray's distance
/// budget -- but every existing call site and every parity test is pinned to
/// what it does: walk only the static tree, rooted at node_base 0.
fn traverse(origin: vec3<f32>, dir: vec3<f32>, t_start: f32, max_dist: f32) -> Hit {
    return traverse_at(origin, dir, t_start, max_dist, 0u, 0u, view.volume_params.x, view.volume_params.y);
}

/// The nearest of the static world's hit and every body's.
///
/// `t` is directly comparable because body transforms are rigid: the ray is
/// rotated and translated into the body's frame, never scaled, so a distance
/// means the same thing in both. The normal comes back through the rotation
/// alone -- putting a normal through the full affine would add the translation
/// and point it nowhere.
///
/// `id` is the pixel this primary ray belongs to, for the screen rectangles.
/// Primary rays only: a rectangle is a primary-ray footprint, so a shadow ray
/// must never reach this.
fn compose_bodies(origin: vec3<f32>, dir: vec3<f32>, world_hit: Hit, max_dist: f32, id: vec2<u32>) -> Hit {
    var best = world_hit;
    var limit = max_dist;
    if best.hit { limit = best.t; }

    let count = view.volume_params.w;
    for (var i = 0u; i < count; i = i + 1u) {
        // Cheaper than the read and the two transforms below, and ahead of
        // them. `traverse_at`'s slab test already rejects a ray that misses the
        // body's box, but only after that work has been paid.
        if flag_enabled(FLAG_BODY_RECT) {
            let r = body_rects[i];
            if id.x < r.min.x || id.x > r.max.x || id.y < r.min.y || id.y > r.max.y {
                continue;
            }
        }
        let b = bodies[i];
        let local_origin = (b.local_from_world * vec4<f32>(origin, 1.0)).xyz;
        let local_dir = (b.local_from_world * vec4<f32>(dir, 0.0)).xyz;

        let h = traverse_at(local_origin, local_dir, 0.0, limit, b.node_base, b.voxel_base, b.depth, b.extent);
        if h.hit && h.t < limit {
            best = h;
            best.face_normal = (b.rotation * vec4<f32>(h.face_normal, 0.0)).xyz;
            best.from_body = true;
            hit_body = i;
            limit = h.t;
        }
    }
    return best;
}

/// Material at a voxel coordinate of the static world, or 0 outside it.
fn material_at(p: vec3<i32>) -> u32 {
    return material_in(p, 0u, 0u, view.volume_params.x, view.volume_params.y);
}

/// Material at a voxel coordinate of the volume rooted at `node_base`, with its
/// voxel bytes from `voxel_base`: the static world, or a body. 0 outside it.
///
/// This is the tree descent from Contree::get: at each level pick the child
/// containing the coordinate, stopping early at an unsubdivided node. An empty
/// node's material is 0 and a uniform solid's is its own, so the single early
/// return covers both without conflating them. One descent for the world and
/// every body, laid out as `traverse_at` reads them: arena slot `n` at
/// `node_base + 1 + n`.
fn material_in(p: vec3<i32>, node_base: u32, voxel_base: u32, depth: u32, volume_extent: u32) -> u32 {
    let extent = i32(volume_extent);
    if p.x < 0 || p.y < 0 || p.z < 0 || p.x >= extent || p.y >= extent || p.z >= extent {
        return 0u;
    }

    var node = nodes[node_base];
    var level = depth - 1u;
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
            return voxel_byte_at(voxel_base, child_slot(node, i));
        }

        let step_size = level_extent(level - 1u);
        let cell = local / step_size;
        let i = cell.x + cell.y * BRICK_EDGE + cell.z * BRICK_EDGE * BRICK_EDGE;
        if !has_child(node, i) {
            return 0u;
        }
        node = nodes[node_base + child_slot(node, i) + 1u];
        local = local - cell * step_size;
        level = level - 1u;
    }
    return 0u;
}

/// Sums the directions in which a voxel is exposed, falling back to the entry
/// face when that carries no information. Mirrors bevox_core::normal. The
/// static world's; `implicit_normal_in` takes any volume.
fn implicit_normal(voxel: vec3<u32>, face_normal: vec3<f32>) -> vec3<f32> {
    return implicit_normal_in(voxel, face_normal, 0u, 0u, view.volume_params.x, view.volume_params.y);
}

fn implicit_normal_in(
    voxel: vec3<u32>,
    face_normal: vec3<f32>,
    node_base: u32,
    voxel_base: u32,
    depth: u32,
    extent: u32,
) -> vec3<f32> {
    let base = vec3<i32>(voxel);
    var sum = vec3<f32>(0.0);

    if material_in(base + vec3<i32>(1, 0, 0), node_base, voxel_base, depth, extent) == 0u { sum += vec3<f32>(1.0, 0.0, 0.0); }
    if material_in(base + vec3<i32>(-1, 0, 0), node_base, voxel_base, depth, extent) == 0u { sum += vec3<f32>(-1.0, 0.0, 0.0); }
    if material_in(base + vec3<i32>(0, 1, 0), node_base, voxel_base, depth, extent) == 0u { sum += vec3<f32>(0.0, 1.0, 0.0); }
    if material_in(base + vec3<i32>(0, -1, 0), node_base, voxel_base, depth, extent) == 0u { sum += vec3<f32>(0.0, -1.0, 0.0); }
    if material_in(base + vec3<i32>(0, 0, 1), node_base, voxel_base, depth, extent) == 0u { sum += vec3<f32>(0.0, 0.0, 1.0); }
    if material_in(base + vec3<i32>(0, 0, -1), node_base, voxel_base, depth, extent) == 0u { sum += vec3<f32>(0.0, 0.0, -1.0); }

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
///
/// Camera-relative: the unprojected point is already an offset from the eye, so
/// nothing is subtracted. Unprojecting through an absolute matrix and
/// subtracting the eye loses about |eye| * epsilon / near of direction, which a
/// thousand units out is pixels.
fn primary_ray(id: vec3<u32>, size: vec2<u32>) -> vec3<f32> {
    // (2 * id + 1 - size) / size: the pixel centre in [-1, 1]. An exact integer
    // difference times an explicit reciprocal, so no compiler can reorder it.
    // Written as `(id + 0.5) / size * 2 - 1`, the GPU driver divided by
    // multiplying with the rounded reciprocal and landed 2 ulp from the CPU
    // mirror's true division, which moved a ray across a voxel edge.
    let ndc = vec2<f32>(
        f32(2u * id.x + 1u) - f32(size.x),
        f32(size.y) - f32(2u * id.y + 1u),
    ) * (vec2<f32>(1.0) / vec2<f32>(size));
    let p = view.offset_from_clip * vec4<f32>(ndc, 1.0, 1.0);
    return normalize(p.xyz / p.w);
}

/// Voxels per field cell, per axis, as built by `DistanceField::build`.
///
/// Carried in the uniform rather than a shader-side constant: a constant here
/// would be a second spelling of `bevox_core::distance_field::CELL_VOXELS`,
/// free to drift from it, and a mismatch would make the field's promised
/// cube the wrong size on the GPU while every CPU-side check kept passing.
fn field_cell_size() -> f32 {
    return f32(view.field_params.y);
}

fn field_at(cell: vec3<i32>) -> u32 {
    let edge = i32(view.field_params.x);
    if cell.x < 0 || cell.y < 0 || cell.z < 0
        || cell.x >= edge || cell.y >= edge || cell.z >= edge {
        return 0u;
    }
    let i = u32(cell.x + cell.y * edge + cell.z * edge * edge);
    return (distance_field[i / 4u] >> ((i % 4u) * 8u)) & 0xFFu;
}

/// Advances `t` past empty space the field can prove is empty.
///
/// The field holds a Chebyshev distance, so a value of `d` at the ray's cell
/// promises the whole cube of `d` cells around it is empty. Advancing to that
/// cube's exit plane is therefore safe in any direction, which is the property
/// a Euclidean field would not give.
///
/// Conservative by construction: it never advances past the cube the field
/// promised, so it cannot skip geometry unless the field itself lied.
fn skip_empty_space(origin: vec3<f32>, dir: vec3<f32>, inv_dir: vec3<f32>, start: f32, max_dist: f32) -> f32 {
    var t = start;
    let cell_size = field_cell_size();
    // A ray crosses a bounded number of cubes before it either hits something
    // or leaves; the bound stops a degenerate direction spinning here.
    for (var i = 0u; i < 64u; i = i + 1u) {
        if t > max_dist { return t; }
        let p = origin + dir * t;
        let cell = vec3<i32>(floor(p / cell_size));
        let d = field_at(cell);
        if d == 0u { return t; }

        // Exit plane of the cube of `d` cells around this one.
        let lo = (vec3<f32>(cell) - vec3<f32>(f32(d) - 1.0)) * cell_size;
        let hi = (vec3<f32>(cell) + vec3<f32>(f32(d))) * cell_size;
        let t0 = (lo - origin) * inv_dir;
        let t1 = (hi - origin) * inv_dir;
        let far = max(t0, t1);
        let exit = min(min(far.x, far.y), far.z);
        // Nudge past the boundary, or the next sample lands on the same cell
        // and the loop makes no progress.
        if exit <= t { return t; }
        // Relative, not fixed: at t in the thousands a 1e-3 nudge is below
        // float32's ULP and rounds straight back to `exit`, so the walk stops
        // early on exactly the large scenes this is meant to help. Kept well
        // under one cell so it can never step past the cube just cleared.
        t = exit + max(1e-3, exit * 1e-5);
    }
    return t;
}

/// Whether anything is hit within `max_dist`. Shadow rays do not care which
/// voxel occludes them, only that one does.
///
/// A thin wrapper rather than a second traversal: duplicating the walk would let
/// the two drift apart, and the shadow path would stop being covered by the
/// parity test that guards the primary one.
fn traverse_any(origin: vec3<f32>, dir: vec3<f32>, max_dist: f32) -> bool {
    var t_seed = 0.0;
    if flag_enabled(FLAG_DISTANCE_FIELD) {
        t_seed = skip_empty_space(origin, dir, vec3<f32>(1.0) / dir, 0.0, max_dist);
    }
    return traverse(origin, dir, t_seed, max_dist).hit;
}

/// Whether the ray toward the sun from `origin` is blocked: by the static world,
/// or, with FLAG_BODY_SHADOWS, by any body.
///
/// Every body up to the cap, from the shadow casters after the table's room,
/// never from the culled table: a body just off screen can shadow what is on it.
/// No screen rectangle either, for the same reason. From t = 0, as
/// `compose_bodies` marches: the beam seed and the distance field know nothing
/// of bodies. The first hit ends it, since which body blocks does not matter.
fn shadowed(origin: vec3<f32>, dir: vec3<f32>, max_dist: f32) -> bool {
    if traverse_any(origin, dir, max_dist) {
        return true;
    }
    if !flag_enabled(FLAG_BODY_SHADOWS) {
        return false;
    }
    let count = view.field_params.z;
    let start = view.field_params.w;
    for (var i = 0u; i < count; i = i + 1u) {
        // The body's bounding sphere first, read on its own so a body the ray
        // misses costs one field and no transform. Most shadow rays pass
        // nowhere near most bodies: measured, sixteen bodies behind the camera
        // cost 20 ms a frame of shadow rays without this. `dir` is the unit sun
        // direction. The radius is widened a hair, so a ray grazing the box's
        // corner, which lies on the sphere, is never rounded out.
        let bound = bodies[start + i].bound;
        let to = bound.xyz - origin;
        let along = dot(to, dir);
        let r = bound.w * 1.001 + 1e-3;
        if along < -r || dot(to, to) - along * along > r * r {
            continue;
        }
        let b = bodies[start + i];
        let local_origin = (b.local_from_world * vec4<f32>(origin, 1.0)).xyz;
        let local_dir = (b.local_from_world * vec4<f32>(dir, 0.0)).xyz;
        if traverse_at(local_origin, local_dir, 0.0, max_dist, b.node_base, b.voxel_base, b.depth, b.extent).hit {
            return true;
        }
    }
    return false;
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

    let hit = traverse(view.camera_position.xyz, primary_ray(id, bd), 0.0, max_ray_distance());
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
/// The seed is where the walk starts, not a new origin: `traverse_at` says why.
/// A seed of zero is the unseeded walk exactly, which is what every parity test
/// pins.
fn primary_hit(id: vec3<u32>, size: vec2<u32>) -> Hit {
    let dir = primary_ray(id, size);
    var t_seed = 0.0;
    if flag_enabled(FLAG_BEAM) {
        t_seed = beam_seed(id, size);
    }
    if flag_enabled(FLAG_DISTANCE_FIELD) {
        t_seed = skip_empty_space(view.camera_position.xyz, dir, vec3<f32>(1.0) / dir, t_seed, max_ray_distance());
    }
    var hit = traverse(view.camera_position.xyz, dir, t_seed, max_ray_distance());
    if flag_enabled(FLAG_BODIES) {
        // From the camera over the whole ray, not from the seed distance. The
        // beam seed and the distance-field skip are static-world accelerators:
        // the prepass marched only the static tree and the field was built only
        // from it, so neither carries any information about bodies. Starting a
        // body march from them would skip every body standing in the empty
        // static space they jumped over. The world hit's absolute `t` still
        // bounds the search, because nothing behind it can be seen.
        hit = compose_bodies(view.camera_position.xyz, dir, hit, max_ray_distance(), id.xy);
    }
    return hit;
}

/// The shading normal for a hit: the implicit normal, which blends the
/// directions a voxel is open to, so edges and corners shade apart from the
/// faces around them. For bodies as for the static world.
///
/// A body hit's voxel is a coordinate in the body's own volume, so it is
/// probed there, not in the static tree, from the face normal carried back into
/// the body's frame, and the result is rotated into the world. The face normal
/// comes back rounded: the rotation there and back leaves it a hair off its
/// axis, and it is the fallback the CPU gives exactly.
fn shading_normal(hit: Hit) -> vec3<f32> {
    if hit.from_body {
        let b = bodies[hit_body];
        let local_face = round((transpose(b.rotation) * vec4<f32>(hit.face_normal, 0.0)).xyz);
        let n = implicit_normal_in(hit.voxel, local_face, b.node_base, b.voxel_base, b.depth, b.extent);
        return (b.rotation * vec4<f32>(n, 0.0)).xyz;
    }
    return implicit_normal(hit.voxel, hit.face_normal);
}

/// The centre of the voxel that was hit, in the world.
///
/// For a body the centre is found in its own frame and carried back through its
/// placement, by way of the world hit point: `shadow_origin` says why that
/// route and not straight from the voxel.
fn voxel_centre(hit: Hit, id: vec3<u32>, size: vec2<u32>) -> vec3<f32> {
    if hit.from_body {
        let b = bodies[hit_body];
        let a = mat3x3<f32>(b.local_from_world[0].xyz, b.local_from_world[1].xyz, b.local_from_world[2].xyz);
        let point = view.camera_position.xyz + primary_ray(id, size) * hit.t;
        let local_point = a * point + b.local_from_world[3].xyz;
        return point + transpose(a) * (vec3<f32>(hit.voxel) + vec3<f32>(0.5) - local_point);
    }
    return vec3<f32>(hit.voxel) + vec3<f32>(0.5);
}

/// How full of solid one coarse cell is, 0 to 1. Outside the grid is empty:
/// past the world there is nothing to shut the light out.
fn fullness_cell(cell: vec3<i32>) -> f32 {
    let edge = i32(view.field_params.x);
    if cell.x < 0 || cell.y < 0 || cell.z < 0
        || cell.x >= edge || cell.y >= edge || cell.z >= edge {
        return 0.0;
    }
    let i = u32(cell.x + cell.y * edge + cell.z * edge * edge);
    let word = distance_field[view.ao_params.x + i / 4u];
    return f32((word >> ((i % 4u) * 8u)) & 0xFFu) / 255.0;
}

/// How full the world is around `p`, blended between the eight cell centres
/// around it. Dwyer's devlog #15: a flat surface has half its surroundings
/// full, an inner corner about three quarters.
fn fullness_at(p: vec3<f32>) -> f32 {
    let c = p / field_cell_size() - vec3<f32>(0.5);
    let base = floor(c);
    let f = c - base;
    var total = 0.0;
    for (var i = 0u; i < 8u; i = i + 1u) {
        let step = vec3<f32>(f32(i & 1u), f32((i >> 1u) & 1u), f32((i >> 2u) & 1u));
        let w = mix(vec3<f32>(1.0) - f, f, step);
        total += w.x * w.y * w.z * fullness_cell(vec3<i32>(base + step));
    }
    return total;
}

/// How much of the ambient light is shut out at the hit voxel, 0 to 1.
///
/// Half full is a flat surface and shuts out nothing; what lies past half is
/// the darkening, so an inner corner at three quarters loses half its ambient.
/// One sample at the voxel's centre, so the shading stays per voxel.
fn occlusion(hit: Hit, id: vec3<u32>, size: vec2<u32>) -> f32 {
    if !flag_enabled(FLAG_AO) {
        return 0.0;
    }
    return clamp((fullness_at(voxel_centre(hit, id, size)) - 0.5) * 2.0, 0.0, 1.0);
}

/// Where the shadow ray toward the sun starts for this hit.
///
/// From the centre of the voxel that was hit, 0.75 along its face, for bodies
/// as for the static world. So a whole voxel face is lit or shadowed together:
/// shadows fall in voxels, on bodies as on terrain.
fn shadow_origin(hit: Hit, id: vec3<u32>, size: vec2<u32>) -> vec3<f32> {
    if hit.from_body {
        // From the world hit point to the voxel's centre, a move made in the
        // body's own frame and carried back through its rotation: for a rigid
        // `local = A world + b`, a local offset `d` is `Aᵀ d` in the world.
        // `face_normal` is already in the world, and 0.75 along it is 0.75
        // along the local face, which clears the voxel as on the world grid.
        //
        // Through the hit point rather than straight from the voxel, which
        // gives the same centre: measured A/B/A, dropping `primary_ray` from
        // this branch, which a body-free scene never runs, cost that scene
        // 0.58 ms of 11.8, a driver codegen cliff over the whole march. This
        // form costs +0.04 ms.
        return voxel_centre(hit, id, size) + hit.face_normal * 0.75;
    }
    // Offset along the FACE normal, not the smoothed one. The implicit
    // normal is a blend of neighbouring empty faces, so at a three-way
    // corner it is normalize(1,1,1) and 0.75 along it clears only 0.433
    // per axis -- less than the voxel's 0.5 half-extent, leaving the ray
    // inside its own voxel to immediately self-shadow. The face normal is
    // axis-aligned, so 0.75 always clears.
    return vec3<f32>(hit.voxel) + vec3<f32>(0.5) + hit.face_normal * 0.75;
}

/// Shadow flag in red: 255 shadowed, 0 lit. Parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_shadow(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        let origin = shadow_origin(hit, id, size);
        var flag = 0.0;
        if shadowed(origin, view.sun_direction.xyz, max_ray_distance()) {
            flag = 1.0;
        }
        colour = vec4<f32>(flag, 0.0, 0.0, 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// Ambient occlusion in red, 255 fully shut out. Parity test only.
@compute @workgroup_size(8, 8, 1)
fn march_ao(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y { return; }

    let hit = primary_hit(id, size);

    var colour = vec4<f32>(0.0, 0.0, 0.0, 0.0);
    if hit.hit {
        colour = vec4<f32>(occlusion(hit, id, size), 0.0, 0.0, 1.0);
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
        let n = shading_normal(hit);
        colour = vec4<f32>(n * 0.5 + vec3<f32>(0.5), 1.0);
    }
    textureStore(output, vec2<i32>(id.xy), colour);
}

/// Hit voxel coordinate in RGB, hit flag in alpha. Read by the parity test only.
/// For a body hit this is the body-local voxel: an identity, not a position.
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
        let n = shading_normal(hit);
        let sun = view.sun_direction.xyz;

        let origin = shadow_origin(hit, id, size);
        var diffuse = max(dot(n, sun), 0.0) * 0.75;
        if shadowed(origin, sun, max_ray_distance()) {
            diffuse = 0.0;
        }

        // Ambient occlusion darkens the ambient light only: the sun's own is
        // already answered by the shadow ray.
        let ambient = 0.25 * (1.0 - occlusion(hit, id, size));
        let base = palette[hit.material].rgb;
        colour = base * (ambient + diffuse);
    }
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(colour, 1.0));
}
