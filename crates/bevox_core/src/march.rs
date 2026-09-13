//! CPU reference ray marcher.
//!
//! Deliberately simple: recursive descent, ray-box per child, children visited
//! in near-to-far order. It is ground truth for the GPU shader, so it shares
//! none of that shader's optimisations — a reference that repeats the clever
//! part would repeat the clever part's bugs.

use crate::contree::{Contree, level_extent};
use crate::material::MaterialId;
use crate::node::{BRICK_EDGE, CHILDREN, Node, child_index};
use glam::{Affine3A, IVec3, UVec3, Vec3};

/// Upper bound on child visits for one ray. Exceeding it reports an overrun and
/// returns a miss rather than looping.
pub const MAX_STEPS: u32 = 65_536;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Hit {
    /// Distance along the world-space ray.
    pub t: f32,
    /// Voxel coordinate within the volume.
    pub voxel: UVec3,
    pub material: MaterialId,
    /// Normal of the face the ray entered through, in volume space.
    pub face_normal: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MarchStats {
    pub steps: u32,
    pub overruns: u32,
}

/// Slab test. Returns the entry and exit distances, which overlap only when the
/// ray actually crosses the box.
fn ray_box(origin: Vec3, inv_dir: Vec3, lo: Vec3, hi: Vec3) -> Option<(f32, f32)> {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = t0.min(t1);
    let far = t0.max(t1);
    let t_enter = near.max_element();
    let t_exit = far.min_element();
    if t_enter <= t_exit && t_exit >= 0.0 {
        Some((t_enter, t_exit))
    } else {
        None
    }
}

/// Which face of a box the ray entered, given the per-axis entry distances.
fn entry_normal(origin: Vec3, inv_dir: Vec3, lo: Vec3, hi: Vec3) -> Vec3 {
    let t0 = (lo - origin) * inv_dir;
    let t1 = (hi - origin) * inv_dir;
    let near = t0.min(t1);
    if near.x >= near.y && near.x >= near.z {
        if inv_dir.x >= 0.0 { Vec3::NEG_X } else { Vec3::X }
    } else if near.y >= near.z {
        if inv_dir.y >= 0.0 { Vec3::NEG_Y } else { Vec3::Y }
    } else if inv_dir.z >= 0.0 {
        Vec3::NEG_Z
    } else {
        Vec3::Z
    }
}

struct Ray {
    origin: Vec3,
    dir: Vec3,
    inv_dir: Vec3,
    max_dist: f32,
    any_hit: bool,
}

/// Marches a ray through a volume placed by `volume_to_world`.
///
/// `dir` need not be normalised; `t` is expressed in units of `dir`'s length,
/// and `max_dist` in the same units.
pub fn march(
    volume: &Contree,
    volume_to_world: Affine3A,
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    any_hit: bool,
    stats: &mut MarchStats,
) -> Option<Hit> {
    let world_to_volume = volume_to_world.inverse();
    let local_origin = world_to_volume.transform_point3(origin);
    let local_dir = world_to_volume.transform_vector3(dir);

    // A zero component would divide by zero; infinity is the correct limit here
    // and the slab test handles it.
    let inv_dir = Vec3::new(
        if local_dir.x == 0.0 { f32::INFINITY } else { 1.0 / local_dir.x },
        if local_dir.y == 0.0 { f32::INFINITY } else { 1.0 / local_dir.y },
        if local_dir.z == 0.0 { f32::INFINITY } else { 1.0 / local_dir.z },
    );

    let ray = Ray { origin: local_origin, dir: local_dir, inv_dir, max_dist, any_hit };
    let extent = volume.extent() as f32;
    let (t_enter, t_exit) = ray_box(local_origin, inv_dir, Vec3::ZERO, Vec3::splat(extent))?;

    visit(
        volume,
        volume.root(),
        volume.depth() - 1,
        UVec3::ZERO,
        &ray,
        t_enter.max(0.0),
        t_exit,
        stats,
    )
}

fn visit(
    volume: &Contree,
    node: Node,
    level: u32,
    origin: UVec3,
    ray: &Ray,
    t_enter: f32,
    t_exit: f32,
    stats: &mut MarchStats,
) -> Option<Hit> {
    if node.is_empty() || t_enter > ray.max_dist || t_enter > t_exit {
        return None;
    }

    if node.is_uniform_solid() {
        let extent = level_extent(level);
        let lo = origin.as_vec3();
        let hi = lo + Vec3::splat(extent as f32);
        // A collapsed region covers many voxels. Report the one the ray actually
        // entered, not the region's origin, or normals get computed in the wrong
        // place and every large uniform surface shades incorrectly.
        let point = ray.origin + ray.dir * t_enter;
        let lo_i = origin.as_ivec3();
        let voxel = point
            .floor()
            .as_ivec3()
            .clamp(lo_i, lo_i + IVec3::splat(extent as i32 - 1))
            .as_uvec3();
        return Some(Hit {
            t: t_enter,
            voxel,
            material: node.material(),
            face_normal: entry_normal(ray.origin, ray.inv_dir, lo, hi),
        });
    }

    let step = if level == 0 { 1 } else { level_extent(level - 1) };

    // Gather the children this ray crosses, with their entry distances.
    let mut candidates: Vec<(f32, u32, UVec3)> = Vec::with_capacity(8);
    for z in 0..BRICK_EDGE {
        for y in 0..BRICK_EDGE {
            for x in 0..BRICK_EDGE {
                let i = child_index(x, y, z);
                if node.child_slot(i).is_none() {
                    continue;
                }
                stats.steps += 1;
                if stats.steps > MAX_STEPS {
                    stats.overruns += 1;
                    return None;
                }
                let child_origin = origin + UVec3::new(x, y, z) * step;
                let lo = child_origin.as_vec3();
                let hi = lo + Vec3::splat(step as f32);
                if let Some((child_enter, child_exit)) = ray_box(ray.origin, ray.inv_dir, lo, hi)
                    && child_exit >= t_enter
                    && child_enter <= t_exit
                {
                    candidates.push((child_enter.max(t_enter), i, child_origin));
                }
            }
        }
    }

    // Closest-hit needs near-to-far order so the first hit found is the closest.
    // Shadow rays do not care which voxel they find, so they skip the sort.
    if !ray.any_hit {
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    }

    for (child_enter, i, child_origin) in candidates {
        if child_enter > ray.max_dist {
            // Sorted candidates are monotonic, so the rest are further still.
            if ray.any_hit {
                continue;
            }
            break;
        }
        let slot = node.child_slot(i).expect("candidate implies an occupied slot");

        if level == 0 {
            let material = MaterialId(volume.arena().voxel(slot));
            let lo = child_origin.as_vec3();
            let hi = lo + Vec3::ONE;
            return Some(Hit {
                t: child_enter,
                voxel: child_origin,
                material,
                face_normal: entry_normal(ray.origin, ray.inv_dir, lo, hi),
            });
        }

        let child = volume.arena().node(slot);
        let lo = child_origin.as_vec3();
        let hi = lo + Vec3::splat(step as f32);
        let (_, child_exit) = ray_box(ray.origin, ray.inv_dir, lo, hi)
            .expect("candidate implies an intersection");

        if let Some(hit) = visit(
            volume,
            child,
            level - 1,
            child_origin,
            ray,
            child_enter,
            child_exit.min(t_exit),
            stats,
        ) {
            return Some(hit);
        }
    }

    None
}
