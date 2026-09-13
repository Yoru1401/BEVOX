// Ray marcher. This revision writes a flat colour so the plumbing can be
// verified before any traversal logic exists.

struct MarchUniform {
    world_from_clip: mat4x4<f32>,
    camera_position: vec4<f32>,
    // [depth, extent, 0, 0]
    volume_params: vec4<u32>,
};

@group(0) @binding(0) var<uniform> view: MarchUniform;
@group(0) @binding(1) var<storage, read> nodes: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> voxels: array<u32>;
@group(0) @binding(3) var output: texture_storage_2d<rgba8unorm, write>;

@compute @workgroup_size(8, 8, 1)
fn march(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(output);
    if id.x >= size.x || id.y >= size.y {
        return;
    }

    // Flat colour, plus a faint gradient so a stuck frame is obvious.
    let uv = vec2<f32>(f32(id.x) / f32(size.x), f32(id.y) / f32(size.y));
    textureStore(output, vec2<i32>(id.xy), vec4<f32>(0.15, 0.35 + uv.y * 0.2, 0.6, 1.0));
}
