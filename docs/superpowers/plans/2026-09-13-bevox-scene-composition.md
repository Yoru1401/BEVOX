# BEVOX MagicaVoxel scene composition — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Load a whole MagicaVoxel scene — every model, placed and rotated by its scene graph — instead of one model out of dozens.

**Architecture:** The scene graph is walked from its root, accumulating translation and rotation down through transform and group nodes, producing a flat list of placed models. Their voxels are transformed into a common space, their bounds measured, and a contree built directly from that sparse voxel list. The dense intermediate is bypassed entirely: a scene spanning 1024 voxels per axis would need a gigabyte of RAM as a `DenseVolume` and only a few megabytes as a contree.

**Tech Stack:** Rust, dot_vox 5.2, glam 0.32. No GPU work — every task here is headless and testable.

**Spec:** `docs/superpowers/specs/2026-09-13-bevox-raymarcher-core-design.md`
**Predecessor:** `2026-09-13-bevox-shading-and-vox.md` (milestones 5–6)

## Why this exists

`vox_info` on the models in `assets/` reports what single-model import was missing:

| file | models | scene nodes | voxels | model 0 |
|---|---|---|---|---|
| custom.vox | 50 | 462 | 2.6M | 16x16x2, 192 voxels |
| sponza.vox | 64 | 130 | 5.1M | 190x117x79, 94k voxels |

`import_model(data, 0)` renders one piece. For sponza that is a wall; for
custom.vox it is a 16x16x2 sliver, which is why it loaded as extent 16.

## Global Constraints

- Rust edition 2024. No Bevy, no GPU: everything here lives in `bevox_core`.
- `MaterialId(0)` is empty; dot_vox palette indices shift up by one, as
  `import_model` already does.
- MagicaVoxel is **Z-up right-handed**; we are Y-up. The swap happens once, at
  the boundary, exactly as single-model import does it.
- Existing `import_model` behaviour must not change. Its tests are the
  regression net for the shared palette and axis handling.
- **The sparse builder must produce trees identical to `Contree::from_dense`**
  for the same content. That equivalence is the correctness proof, and it is
  testable without a GPU.
- Determinism: identical input produces byte-identical arenas, as
  `construction_is_deterministic` already requires of `from_dense`.

## Verified dot_vox API

Confirmed against the installed 5.2.0 source before writing this plan.

- `SceneNode::Transform { attributes, frames: Vec<Frame>, child: u32, layer_id: u32 }`
- `SceneNode::Group { attributes, children: Vec<u32> }`
- `SceneNode::Shape { attributes, models: Vec<ShapeModel> }`
- `ShapeModel { model_id: u32, attributes: Dict }`
- `Frame::position() -> Option<Position { x: i32, y: i32, z: i32 }>`
- `Frame::orientation() -> Option<Rotation>`
- `Rotation::to_cols_array_2d() -> [[f32; 3]; 3]` — a signed permutation matrix,
  so dot_vox decodes MagicaVoxel's packed rotation byte for us
- `Rotation::IDENTITY`

## File Structure

| File | Responsibility |
|---|---|
| `crates/bevox_core/src/vox.rs` | Scene walk, placement, `import_scene` |
| `crates/bevox_core/src/contree.rs` | `Contree::from_voxels`, the sparse builder |
| `crates/bevox_core/examples/vox_info.rs` | Reports composed scene bounds |
| `crates/bevox/src/main.rs` | Loads scenes rather than single models |

---

### Task 1: Walk the scene graph into placed models

**Files:**
- Modify: `crates/bevox_core/src/vox.rs`

**Interfaces:**
- Produces: `PlacedModel { model_id: u32, translation: IVec3, rotation: [[f32; 3]; 3] }`; `placed_models(data: &DotVoxData) -> Vec<PlacedModel>`.

A scene with no graph at all (some files carry models and no `nTRN` chunks)
yields one placed model per model, untransformed, so single-model files keep
working through the same path.

- [ ] **Step 1: Write the failing tests**

Add to `crates/bevox_core/src/vox.rs`, inside the existing test module:

```rust
    use dot_vox::{Dict, Frame, SceneNode, ShapeModel};

    /// A transform node carrying a translation, wrapping `child`.
    fn transform_node(child: u32, t: (i32, i32, i32)) -> SceneNode {
        let mut attributes = Dict::new();
        attributes.insert("_t".to_string(), format!("{} {} {}", t.0, t.1, t.2));
        SceneNode::Transform {
            attributes: Dict::new(),
            frames: vec![Frame::new(attributes)],
            child,
            layer_id: 0,
        }
    }

    fn group_node(children: Vec<u32>) -> SceneNode {
        SceneNode::Group { attributes: Dict::new(), children }
    }

    fn shape_node(model_id: u32) -> SceneNode {
        SceneNode::Shape {
            attributes: Dict::new(),
            models: vec![ShapeModel { model_id, attributes: Dict::new() }],
        }
    }

    #[test]
    fn a_file_with_no_scene_graph_places_every_model_at_the_origin() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.models.push(data.models[0].clone());
        data.scenes.clear();

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 2, "both models should be placed");
        assert!(placed.iter().all(|p| p.translation == IVec3::ZERO));
    }

    #[test]
    fn a_transform_above_a_shape_places_that_model() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(1, (10, 20, 30)), shape_node(0)];

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].model_id, 0);
        assert_eq!(placed[0].translation, IVec3::new(10, 20, 30));
    }

    #[test]
    fn nested_transforms_accumulate() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        // root transform -> group -> transform -> shape
        data.scenes = vec![
            transform_node(1, (100, 0, 0)),
            group_node(vec![2]),
            transform_node(3, (5, 7, 0)),
            shape_node(0),
        ];

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].translation, IVec3::new(105, 7, 0));
    }

    #[test]
    fn a_group_places_each_of_its_children() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.models.push(data.models[0].clone());
        data.scenes = vec![
            group_node(vec![1, 3]),
            transform_node(2, (10, 0, 0)),
            shape_node(0),
            transform_node(4, (-10, 0, 0)),
            shape_node(1),
        ];

        let placed = placed_models(&data);
        assert_eq!(placed.len(), 2);
        assert_eq!(placed[0].translation, IVec3::new(10, 0, 0));
        assert_eq!(placed[1].translation, IVec3::new(-10, 0, 0));
    }

    #[test]
    fn a_cycle_in_the_graph_terminates() {
        // Malformed files exist. A child index pointing back at an ancestor must
        // not hang the importer.
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(1, (1, 0, 0)), group_node(vec![0])];

        let placed = placed_models(&data);
        assert!(placed.len() < 100, "walk did not terminate, produced {}", placed.len());
    }

    #[test]
    fn a_child_index_past_the_end_is_ignored() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(99, (1, 0, 0))];
        assert!(placed_models(&data).is_empty());
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core vox`
Expected: FAIL — `placed_models` and `PlacedModel` not found.

> If `Dict`, `Frame::new` or the `_t` attribute spelling differ from the above,
> read `scene.rs` in the installed dot_vox and follow what is there. `_t` as a
> space-separated string is what its own parser writes, but the fixture must
> match the parser, not this plan.

- [ ] **Step 3: Write the implementation**

```rust
/// One model, positioned by the scene graph.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlacedModel {
    pub model_id: u32,
    /// Translation in MagicaVoxel's Z-up space, of the model's centre.
    pub translation: IVec3,
    /// Signed permutation matrix, columns as dot_vox reports them.
    pub rotation: [[f32; 3]; 3],
}

/// Upper bound on nodes visited, so a malformed or cyclic graph terminates.
const MAX_SCENE_NODES: usize = 1 << 20;

/// Flattens the scene graph into placed models.
///
/// A file with no graph yields every model at the origin, so single-model files
/// travel the same path as scenes.
pub fn placed_models(data: &DotVoxData) -> Vec<PlacedModel> {
    if data.scenes.is_empty() {
        return (0..data.models.len() as u32)
            .map(|model_id| PlacedModel {
                model_id,
                translation: IVec3::ZERO,
                rotation: identity_rotation(),
            })
            .collect();
    }

    let mut out = Vec::new();
    // Explicit stack rather than recursion: these graphs are file data and can
    // be deep or malformed.
    let mut stack = vec![(0u32, IVec3::ZERO, identity_rotation())];
    let mut visited = 0usize;

    while let Some((index, translation, rotation)) = stack.pop() {
        visited += 1;
        if visited > MAX_SCENE_NODES {
            break;
        }
        let Some(node) = data.scenes.get(index as usize) else {
            continue;
        };

        match node {
            SceneNode::Transform { frames, child, .. } => {
                // Only the first frame is used: animation is out of scope, and
                // a static import wants the model's resting position.
                let (t, r) = frames
                    .first()
                    .map(frame_transform)
                    .unwrap_or((IVec3::ZERO, identity_rotation()));
                let combined_rotation = multiply(rotation, r);
                let combined_translation = translation + apply(rotation, t);
                stack.push((*child, combined_translation, combined_rotation));
            }
            SceneNode::Group { children, .. } => {
                // Reversed so children are emitted in file order once popped.
                for child in children.iter().rev() {
                    stack.push((*child, translation, rotation));
                }
            }
            SceneNode::Shape { models, .. } => {
                for m in models {
                    out.push(PlacedModel {
                        model_id: m.model_id,
                        translation,
                        rotation,
                    });
                }
            }
        }
    }

    out
}

fn identity_rotation() -> [[f32; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

fn frame_transform(frame: &Frame) -> (IVec3, [[f32; 3]; 3]) {
    let t = frame
        .position()
        .map(|p| IVec3::new(p.x, p.y, p.z))
        .unwrap_or(IVec3::ZERO);
    let r = frame
        .orientation()
        .map(|o| o.to_cols_array_2d())
        .unwrap_or_else(identity_rotation);
    (t, r)
}

/// Column-major matrix product, matching dot_vox's column convention.
fn multiply(a: [[f32; 3]; 3], b: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0.0f32; 3]; 3];
    for c in 0..3 {
        for r in 0..3 {
            out[c][r] = (0..3).map(|k| a[k][r] * b[c][k]).sum();
        }
    }
    out
}

/// Applies a signed permutation matrix to an integer vector.
fn apply(m: [[f32; 3]; 3], v: IVec3) -> IVec3 {
    let f = v.as_vec3();
    IVec3::new(
        (m[0][0] * f.x + m[1][0] * f.y + m[2][0] * f.z).round() as i32,
        (m[0][1] * f.x + m[1][1] * f.y + m[2][1] * f.z).round() as i32,
        (m[0][2] * f.x + m[1][2] * f.y + m[2][2] * f.z).round() as i32,
    )
}
```

Add `use dot_vox::{Frame, SceneNode};` and `use glam::IVec3;` to the imports.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core vox`
Expected: PASS — 6 new tests plus the 8 existing import tests, still green.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): flatten the MagicaVoxel scene graph into placed models" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Place a model's voxels in scene space

**Files:**
- Modify: `crates/bevox_core/src/vox.rs`

**Interfaces:**
- Produces: `place_voxel(local: UVec3, size: UVec3, placed: &PlacedModel) -> IVec3`, returning Z-up scene coordinates.

MagicaVoxel translations position a model's **centre**, not its corner, so the
voxel is offset by half the model's size before rotation and the translation is
applied after. Getting this backwards shifts every model by half its own size,
which looks like a slightly wrong scene rather than an obvious break.

- [ ] **Step 1: Write the failing tests**

```rust
    fn placed_at(t: (i32, i32, i32)) -> PlacedModel {
        PlacedModel {
            model_id: 0,
            translation: IVec3::new(t.0, t.1, t.2),
            rotation: identity_rotation(),
        }
    }

    #[test]
    fn an_untransformed_model_keeps_its_shape_around_the_origin() {
        let size = UVec3::new(4, 4, 4);
        // The centre voxel of a 4-cube sits at local (2,2,2), which maps to 0.
        assert_eq!(place_voxel(UVec3::new(2, 2, 2), size, &placed_at((0, 0, 0))), IVec3::ZERO);
        assert_eq!(
            place_voxel(UVec3::new(0, 0, 0), size, &placed_at((0, 0, 0))),
            IVec3::new(-2, -2, -2)
        );
    }

    #[test]
    fn translation_moves_the_whole_model() {
        let size = UVec3::new(4, 4, 4);
        assert_eq!(
            place_voxel(UVec3::new(2, 2, 2), size, &placed_at((10, 20, 30))),
            IVec3::new(10, 20, 30)
        );
    }

    #[test]
    fn a_quarter_turn_about_z_swaps_x_and_y() {
        // Signed permutation: x <- -y, y <- x.
        let rotation = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let placed = PlacedModel { model_id: 0, translation: IVec3::ZERO, rotation };
        let size = UVec3::new(4, 4, 4);

        // Local (3,2,2) is +1 on x from the centre; after the turn it is +1 on y.
        assert_eq!(place_voxel(UVec3::new(3, 2, 2), size, &placed), IVec3::new(0, 1, 0));
    }

    #[test]
    fn rotation_happens_before_translation() {
        let rotation = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let placed = PlacedModel {
            model_id: 0,
            translation: IVec3::new(100, 0, 0),
            rotation,
        };
        let size = UVec3::new(4, 4, 4);
        // Rotating first then translating puts this at (100, 1, 0); translating
        // first would put it at (0, 101, 0).
        assert_eq!(place_voxel(UVec3::new(3, 2, 2), size, &placed), IVec3::new(100, 1, 0));
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core vox`
Expected: FAIL — `place_voxel` not found.

- [ ] **Step 3: Write the implementation**

```rust
/// Places one of a model's voxels into scene space, still Z-up.
///
/// MagicaVoxel's translations position the model's centre, so the voxel is
/// measured from that centre before being rotated and moved.
pub fn place_voxel(local: UVec3, size: UVec3, placed: &PlacedModel) -> IVec3 {
    let centred = local.as_ivec3() - (size.as_ivec3() / 2);
    placed.translation + apply(placed.rotation, centred)
}
```

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core vox`
Expected: PASS, 4 new tests.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): place model voxels into scene space" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Build a contree directly from a sparse voxel list

The reason this task exists: composing sponza spans hundreds of voxels per axis,
and a `DenseVolume` of extent 1024 is a gigabyte of RAM for five million voxels
that fit in a few megabytes sparse.

**Files:**
- Modify: `crates/bevox_core/src/contree.rs`

**Interfaces:**
- Produces: `Contree::from_voxels(extent: u32, voxels: &[(UVec3, MaterialId)]) -> Contree`.

- [ ] **Step 1: Write the failing tests**

Add to `crates/bevox_core/tests/round_trip.rs`:

```rust
use bevox_core::material::MaterialId;
use bevox_core::node::Node;

/// The sparse builder must produce exactly what the dense one does. This is the
/// correctness proof for the whole task: the dense path is already trusted, so
/// equivalence transfers that trust rather than asking for new faith.
#[test]
fn the_sparse_builder_matches_the_dense_builder() {
    for (extent, seed) in [(4u32, 11u64), (16, 12), (64, 13)] {
        let dense = mixed_volume(extent, seed);

        let mut voxels = Vec::new();
        for z in 0..extent {
            for y in 0..extent {
                for x in 0..extent {
                    let p = UVec3::new(x, y, z);
                    let m = dense.get(p);
                    if !m.is_empty() {
                        voxels.push((p, m));
                    }
                }
            }
        }

        let from_dense = Contree::from_dense(&dense);
        let from_voxels = Contree::from_voxels(extent, &voxels);

        assert_eq!(from_voxels.root(), from_dense.root(), "extent {extent} root");
        assert_eq!(
            from_voxels.arena().nodes(),
            from_dense.arena().nodes(),
            "extent {extent} nodes"
        );
        assert_eq!(
            from_voxels.arena().voxels(),
            from_dense.arena().voxels(),
            "extent {extent} voxels"
        );
        assert_eq!(from_voxels.check_canonical(), Ok(()));
    }
}

#[test]
fn an_empty_voxel_list_builds_an_empty_tree() {
    let tree = Contree::from_voxels(64, &[]);
    assert!(tree.root().is_empty());
    assert_eq!(tree.arena().nodes().len(), 0);
}

#[test]
fn later_voxels_win_when_a_coordinate_repeats() {
    let p = UVec3::new(1, 2, 3);
    let tree = Contree::from_voxels(16, &[(p, MaterialId(4)), (p, MaterialId(9))]);
    assert_eq!(tree.get(p), MaterialId(9));
}

#[test]
fn a_fully_solid_volume_collapses_from_the_sparse_path_too() {
    let extent = 16u32;
    let mut voxels = Vec::new();
    for z in 0..extent {
        for y in 0..extent {
            for x in 0..extent {
                voxels.push((UVec3::new(x, y, z), MaterialId(3)));
            }
        }
    }
    let tree = Contree::from_voxels(extent, &voxels);
    assert!(tree.root().is_uniform_solid(), "a solid volume must collapse");
    assert_eq!(tree.arena().nodes().len(), 0);
    assert_eq!(tree.root().material(), MaterialId(3));
}

#[test]
fn the_sparse_builder_is_deterministic() {
    let dense = mixed_volume(16, 77);
    let mut voxels = Vec::new();
    for z in 0..16 {
        for y in 0..16 {
            for x in 0..16 {
                let p = UVec3::new(x, y, z);
                if !dense.get(p).is_empty() {
                    voxels.push((p, dense.get(p)));
                }
            }
        }
    }
    let a = Contree::from_voxels(16, &voxels);
    let b = Contree::from_voxels(16, &voxels);
    assert_eq!(a.arena().nodes(), b.arena().nodes());
    assert_eq!(a.root(), b.root());
}
```

These comparisons work on slices, so `Node` itself need not be named; add only
`MaterialId` to the imports.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core --test round_trip`
Expected: FAIL — no function `from_voxels` for `Contree`.

- [ ] **Step 3: Write the implementation**

The build is bottom-up: bucket voxels into leaf bricks, build those, then group
each level's nodes into their parents until one root remains. Nothing walks empty
space, so cost tracks the number of filled voxels rather than the volume.

```rust
use std::collections::HashMap;

impl Contree {
    /// Builds a tree from a sparse voxel list, without a dense intermediate.
    ///
    /// Bottom-up: voxels are bucketed into 4x4x4 leaf bricks, those are built,
    /// and each level is then grouped into its parents. Cost tracks the number
    /// of filled voxels rather than the volume, which is what makes a scene
    /// spanning hundreds of voxels per axis affordable.
    ///
    /// Where a coordinate repeats, the later entry wins.
    pub fn from_voxels(extent: u32, voxels: &[(UVec3, MaterialId)]) -> Self {
        debug_assert!(extent.is_power_of_two() && extent.trailing_zeros() % 2 == 0);
        let depth = extent.trailing_zeros() / 2;
        let mut tree = Self::empty(depth);

        // Bucket into leaf bricks, keyed by brick coordinate.
        let mut bricks: HashMap<UVec3, [MaterialId; CHILDREN as usize]> = HashMap::new();
        for (p, m) in voxels {
            if m.is_empty() || p.x >= extent || p.y >= extent || p.z >= extent {
                continue;
            }
            let brick = *p / BRICK_EDGE;
            let local = *p % BRICK_EDGE;
            let slot = child_index(local.x, local.y, local.z) as usize;
            bricks
                .entry(brick)
                .or_insert([MaterialId::EMPTY; CHILDREN as usize])[slot] = *m;
        }

        // Level 0: build every non-empty leaf brick.
        let mut level_nodes: HashMap<UVec3, Node> = HashMap::new();
        let mut keys: Vec<UVec3> = bricks.keys().copied().collect();
        keys.sort_by_key(|k| (k.z, k.y, k.x));
        for key in keys {
            let materials = &bricks[&key];
            let node = tree.build_leaf_from_materials(materials);
            if !node.is_empty() {
                level_nodes.insert(key, node);
            }
        }

        // Each level up: group nodes by parent coordinate.
        for _ in 1..depth {
            let mut parents: HashMap<UVec3, [Node; CHILDREN as usize]> = HashMap::new();
            let mut keys: Vec<UVec3> = level_nodes.keys().copied().collect();
            keys.sort_by_key(|k| (k.z, k.y, k.x));
            for key in keys {
                let parent = key / BRICK_EDGE;
                let local = key % BRICK_EDGE;
                let slot = child_index(local.x, local.y, local.z) as usize;
                parents
                    .entry(parent)
                    .or_insert([Node::EMPTY; CHILDREN as usize])[slot] = level_nodes[&key];
            }

            let mut next: HashMap<UVec3, Node> = HashMap::new();
            let mut parent_keys: Vec<UVec3> = parents.keys().copied().collect();
            parent_keys.sort_by_key(|k| (k.z, k.y, k.x));
            for key in parent_keys {
                let node = tree.build_branch_from_children(&parents[&key]);
                if !node.is_empty() {
                    next.insert(key, node);
                }
            }
            level_nodes = next;
        }

        tree.set_root(level_nodes.get(&UVec3::ZERO).copied().unwrap_or(Node::EMPTY));
        tree
    }

    /// Leaf brick from 64 materials, collapsing an entirely uniform one.
    /// Shares its rules with `build_leaf` so both builders agree.
    fn build_leaf_from_materials(&mut self, materials: &[MaterialId; CHILDREN as usize]) -> Node {
        let mut mask = 0u64;
        for (i, m) in materials.iter().enumerate() {
            if !m.is_empty() {
                mask |= 1u64 << i;
            }
        }
        if mask == 0 {
            return Node::EMPTY;
        }
        if mask == u64::MAX {
            let first = materials[0];
            if materials.iter().all(|m| *m == first) {
                return Node::uniform(first);
            }
        }

        let base = self.arena.alloc_voxels(mask.count_ones());
        let mut slot = base;
        for m in materials.iter() {
            if !m.is_empty() {
                self.arena.set_voxel(slot, m.0);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }

    /// Branch from 64 children, collapsing when all are the same uniform solid.
    fn build_branch_from_children(&mut self, children: &[Node; CHILDREN as usize]) -> Node {
        let mut mask = 0u64;
        for (i, c) in children.iter().enumerate() {
            if !c.is_empty() {
                mask |= 1u64 << i;
            }
        }
        if mask == 0 {
            return Node::EMPTY;
        }
        if let Some(node) = collapse_uniform(children) {
            return node;
        }

        let base = self.arena.alloc_nodes(mask.count_ones());
        let mut slot = base;
        for c in children.iter() {
            if !c.is_empty() {
                self.arena.set_node(slot, *c);
                slot += 1;
            }
        }
        Node::subdivided(mask, base)
    }
}
```

> **Why the sort matters.** `HashMap` iteration order varies between runs, and
> arena slots are handed out in visit order, so an unsorted walk would produce
> arenas that differ run to run. The equivalence test against `from_dense` also
> requires visiting in the same order the dense builder does: z outermost, then
> y, then x.

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS, 5 new tests.

If the equivalence test fails on node ordering rather than content, the visit
order is the cause, not the collapse rules — compare the first differing slot
and check which coordinate the dense builder reaches first.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): build a contree from a sparse voxel list" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Compose a whole scene

**Files:**
- Modify: `crates/bevox_core/src/vox.rs`

**Interfaces:**
- Produces: `SceneBounds { min: IVec3, max: IVec3 }` with `extent(&self) -> u32`; `import_scene(data: &DotVoxData) -> Result<(Contree, MaterialTable), VoxError>`.
- `VoxError::TooLarge` now names the composed extent. The cap rises from 1024 to 4096, which the shader's eight-deep stack already allows; VRAM is the real limit past that.

- [ ] **Step 1: Write the failing tests**

```rust
    #[test]
    fn an_empty_scene_is_an_error() {
        let mut data = model((4, 4, 4), &[]);
        data.models.clear();
        data.scenes.clear();
        assert_eq!(import_scene(&data).unwrap_err(), VoxError::NoModels);
    }

    #[test]
    fn two_models_placed_apart_both_appear() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0), (3, 3, 3, 0)]);
        data.models.push(data.models[0].clone());
        data.scenes = vec![
            group_node(vec![1, 3]),
            transform_node(2, (0, 0, 0)),
            shape_node(0),
            transform_node(4, (40, 0, 0)),
            shape_node(1),
        ];

        let (tree, _) = import_scene(&data).unwrap();
        let dense = tree.to_dense();

        // Both copies survived: count solid voxels across the composed volume.
        let mut solid = 0;
        for z in 0..dense.extent() {
            for y in 0..dense.extent() {
                for x in 0..dense.extent() {
                    if !dense.get(UVec3::new(x, y, z)).is_empty() {
                        solid += 1;
                    }
                }
            }
        }
        assert_eq!(solid, 4, "two models of two voxels each");
    }

    #[test]
    fn the_composed_volume_starts_at_the_origin() {
        // Models placed at negative coordinates must be shifted into range, not
        // clipped away.
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.scenes = vec![transform_node(1, (-500, -500, -500)), shape_node(0)];

        let (tree, _) = import_scene(&data).unwrap();
        let dense = tree.to_dense();
        let mut found = false;
        for z in 0..dense.extent() {
            for y in 0..dense.extent() {
                for x in 0..dense.extent() {
                    if !dense.get(UVec3::new(x, y, z)).is_empty() {
                        found = true;
                    }
                }
            }
        }
        assert!(found, "the voxel was shifted out of existence");
    }

    #[test]
    fn a_scene_spanning_too_far_is_rejected() {
        let mut data = model((4, 4, 4), &[(0, 0, 0, 0)]);
        data.models.push(data.models[0].clone());
        data.scenes = vec![
            group_node(vec![1, 3]),
            transform_node(2, (0, 0, 0)),
            shape_node(0),
            transform_node(4, (9000, 0, 0)),
            shape_node(1),
        ];
        assert!(matches!(
            import_scene(&data).unwrap_err(),
            VoxError::TooLarge { .. }
        ));
    }

    #[test]
    fn composition_keeps_the_z_up_to_y_up_swap() {
        // Two voxels differing only along MagicaVoxel's z. After the swap they
        // must differ along our y and share our z. One voxel would prove
        // nothing: it defines its own bounds and always lands at the origin.
        let data = {
            let mut d = model((4, 4, 4), &[(0, 0, 0, 6), (0, 0, 3, 6)]);
            d.scenes = vec![transform_node(1, (0, 0, 0)), shape_node(0)];
            d
        };
        let (tree, _) = import_scene(&data).unwrap();
        let dense = tree.to_dense();

        let mut found = Vec::new();
        for z in 0..dense.extent() {
            for y in 0..dense.extent() {
                for x in 0..dense.extent() {
                    if !dense.get(UVec3::new(x, y, z)).is_empty() {
                        found.push(UVec3::new(x, y, z));
                    }
                }
            }
        }

        assert_eq!(found.len(), 2, "both voxels should survive: {found:?}");
        assert_ne!(found[0].y, found[1].y, "vox z must become our y: {found:?}");
        assert_eq!(found[0].z, found[1].z, "our z must be unchanged: {found:?}");
        assert_eq!(found[0].x, found[1].x, "our x must be unchanged: {found:?}");
    }
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p bevox_core vox`
Expected: FAIL — `import_scene` not found.

- [ ] **Step 3: Write the implementation**

```rust
/// Bounds of a composed scene, in our Y-up space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SceneBounds {
    pub min: IVec3,
    pub max: IVec3,
}

impl SceneBounds {
    /// Smallest legal volume extent that holds these bounds.
    pub fn extent(&self) -> Option<u32> {
        let span = (self.max - self.min) + IVec3::ONE;
        let longest = span.x.max(span.y).max(span.z).max(1) as u32;
        fitting_extent(longest)
    }
}

/// The largest volume the engine will build. The shader's stack is eight deep,
/// so depth six is within reach; VRAM is the practical limit beyond this.
const MAX_EXTENT: u32 = 4096;

/// Composes every model in the file into one volume.
pub fn import_scene(data: &DotVoxData) -> Result<(Contree, MaterialTable), VoxError> {
    if data.models.is_empty() {
        return Err(VoxError::NoModels);
    }

    // First pass: place every voxel and measure the bounds.
    let placed = placed_models(data);
    let mut points: Vec<(IVec3, MaterialId)> = Vec::new();
    let mut min = IVec3::splat(i32::MAX);
    let mut max = IVec3::splat(i32::MIN);

    for p in &placed {
        let Some(m) = data.models.get(p.model_id as usize) else {
            continue;
        };
        let size = UVec3::new(m.size.x, m.size.y, m.size.z);
        for v in &m.voxels {
            let vox_space = place_voxel(UVec3::new(v.x as u32, v.y as u32, v.z as u32), size, p);
            // Z-up to Y-up, the same swap single-model import performs.
            let ours = IVec3::new(vox_space.x, vox_space.z, vox_space.y);
            min = min.min(ours);
            max = max.max(ours);
            points.push((ours, MaterialId(v.i.saturating_add(1))));
        }
    }

    if points.is_empty() {
        return Err(VoxError::NoModels);
    }

    let bounds = SceneBounds { min, max };
    let extent = bounds.extent().filter(|e| *e <= MAX_EXTENT).ok_or_else(|| {
        let span = (max - min) + IVec3::ONE;
        VoxError::TooLarge { extent: span.x.max(span.y).max(span.z) as u32 }
    })?;

    // Second pass: shift into a volume starting at the origin.
    let shifted: Vec<(UVec3, MaterialId)> = points
        .into_iter()
        .map(|(p, m)| ((p - min).as_uvec3(), m))
        .collect();

    let tree = Contree::from_voxels(extent, &shifted);

    let mut materials = MaterialTable::new();
    for c in data.palette.iter().take(255) {
        materials
            .push(Material { color: [c.r, c.g, c.b, c.a] })
            .expect("at most 255 entries are pushed");
    }

    Ok((tree, materials))
}
```

`fitting_extent`'s own ceiling rises to `MAX_EXTENT`:

```rust
/// Smallest power of four that fits `size`, or `None` past `MAX_EXTENT`.
pub fn fitting_extent(size: u32) -> Option<u32> {
    let mut extent = 4u32;
    while extent < size {
        extent *= 4;
        if extent > MAX_EXTENT {
            return None;
        }
    }
    Some(extent)
}
```

and its test gains the new boundary while keeping a rejection above it:

```rust
        assert_eq!(fitting_extent(1024), Some(1024));
        assert_eq!(fitting_extent(1025), Some(4096));
        assert_eq!(fitting_extent(4096), Some(4096));
        assert_eq!(fitting_extent(4097), None);
```

Note that `an_oversized_model_is_rejected` uses 2000, which now fits at 4096, so
it must move above the new ceiling to keep testing rejection:

```rust
    #[test]
    fn an_oversized_model_is_rejected() {
        let data = model((5000, 4, 4), &[]);
        assert_eq!(import_model(&data, 0).unwrap_err(), VoxError::TooLarge { extent: 5000 });
    }
```

- [ ] **Step 4: Run to verify they pass**

Run: `cargo test -p bevox_core`
Expected: PASS. Every earlier `vox` test stays green: `import_model` is
untouched.

- [ ] **Step 5: Commit**

```bash
git add crates/bevox_core
git commit -m "feat(core): compose whole MagicaVoxel scenes" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Load scenes in the app and measure the real files

**Files:**
- Modify: `crates/bevox/src/main.rs`, `crates/bevox_core/examples/vox_info.rs`

- [ ] **Step 1: Report composed bounds from the inspector**

Add to `vox_info`, after the per-model listing:

```rust
    match bevox_core::vox::import_scene(&data) {
        Ok((tree, _)) => println!(
            "composed  : extent {}, {} arena nodes, {} voxel bytes",
            tree.extent(),
            tree.arena().nodes().len(),
            tree.arena().voxels().len()
        ),
        Err(e) => println!("composed  : rejected, {e}"),
    }
```

- [ ] **Step 2: Measure every file in assets**

Run, for each file:

```bash
cargo run -p bevox_core --release --example vox_info -- assets/sponza.vox
```

Record extent, node count and voxel bytes for each. A file rejected as
`TooLarge` is a finding, not a failure: it says the composed scene exceeds 4096
per axis and needs streaming rather than one volume.

- [ ] **Step 3: Load scenes in the app**

In `crates/bevox/src/main.rs`, replace the `load_vox` call with a scene load:

```rust
        Some(path) => match bevox_core::vox::load_scene(std::path::Path::new(&path)) {
            Ok((tree, materials)) => {
                info!("loaded {path}: extent {}", tree.extent());
                (tree, materials)
            }
            Err(e) => {
                error!("could not load {path}: {e}");
                demo_scene()
            }
        },
```

with `load_scene` alongside `load_vox` in `vox.rs`:

```rust
/// Reads a `.vox` file and composes its whole scene.
pub fn load_scene(path: &std::path::Path) -> Result<(Contree, MaterialTable), VoxError> {
    let data = dot_vox::load(path.to_str().unwrap_or_default()).map_err(|_| VoxError::ReadFailed)?;
    import_scene(&data)
}
```

`load_vox` stays for single-model use and keeps its tests.

- [ ] **Step 4: Confirm the deliverable**

Run: `cargo run -p bevox --release -- assets/sponza.vox`

Expected: the whole scene, not one wall. Fly through it and check that pieces
line up — floors meeting walls, columns standing on the floor rather than
floating or sunk. A scene where everything is present but offset by half a model
points at the centre-versus-corner convention in `place_voxel`; one where pieces
are rotated wrongly points at the matrix order in `multiply`.

- [ ] **Step 5: Note the frame time**

The traversal is still the unoptimised version. Record whether a composed scene
is interactive at 1280x720, since that number is the argument for milestone 8.
If it is not, say so plainly rather than treating slowness as expected.

- [ ] **Step 6: Run the whole suite**

Run: `cargo test`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add crates
git commit -m "feat: load whole MagicaVoxel scenes" -m "Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Measurements

Composed on a GTX 1650, release build, from `vox_info`:

| file | models | voxels | composed extent | arena nodes | voxel bytes | compose |
|---|---|---|---|---|---|---|
| Church_Of_St_Sophia | 571 | 14.3M | 4096 | 977k | 14.3 MB | 2.12s |
| castle | 455 | 22.0M | 4096 | 712k | 18.3 MB | 2.06s |
| custom | 50 | 2.6M | 256 | 60k | 3.5 MB | 0.24s |
| nuke | 1193 | 27.7M | 4096 | 909k | 27.7 MB | 2.62s |
| sponza | 64 | 5.1M | 1024 | 264k | 5.1 MB | 0.54s |

Nothing was rejected, and three of five need the raised 4096 cap. A dense
intermediate at extent 4096 would be 68 GB; the same scene is about 30 MB as a
contree, which is what makes the sparse builder load-bearing rather than an
optimisation.

Frame times at 1280x720, Church at extent 4096:

| camera | frame time | fps |
|---|---|---|
| framed at 1.1 extents | 16.6 ms | 60, vsync-locked |
| close to geometry | 37-101 ms | 10-27 |

This is milestone 8's argument. Cost rises sharply when geometry fills the
screen: more pixels hit, every hit casts a shadow ray, and every ray descends
through near-field nodes where the current shader linearly scans all 64 children
of each node regardless of how few are occupied. DDA within bricks, the bitmask
filter and the beam prepass all target exactly that case.

## What this plan deliberately does not do

No animation: only the first frame of each transform is read. No layer
visibility: hidden layers are composed like any other. No streaming — a scene is
one volume, and one that spans more than 4096 per axis is rejected rather than
partially loaded.

No renderer changes at all. If composed scenes prove too slow, that is
milestone 8's argument, and this plan's job is to produce the measurement rather
than pre-empt it.

## Subsequent plans

- **Milestone 7.** The sphere brush wired to mouse input, with dirty-range upload.
- **Milestone 8.** DDA, bitmask filtering and the beam prepass, each measured A/B/A and each required to leave the parity tests bit-identical.
