//! Reports what a `.vox` file actually contains.
//!
//! Run with: cargo run -p bevox_core --example vox_info -- path/to/model.vox

fn main() {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: vox_info <file.vox>");
        std::process::exit(2);
    };

    let data = match dot_vox::load(&path) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("could not read {path}: {e}");
            std::process::exit(1);
        }
    };

    println!("file      : {path}");
    println!("version   : {}", data.version);
    println!("models    : {}", data.models.len());
    println!("palette   : {} entries", data.palette.len());
    println!("scenes    : {} nodes", data.scenes.len());
    println!("layers    : {}", data.layers.len());

    let mut total_voxels = 0usize;
    let mut largest = (0u32, 0usize);
    for (i, m) in data.models.iter().enumerate() {
        total_voxels += m.voxels.len();
        let longest = m.size.x.max(m.size.y).max(m.size.z);
        if longest > largest.0 {
            largest = (longest, i);
        }
    }

    println!("voxels    : {total_voxels} across all models");
    println!(
        "largest   : model {} at {} per axis",
        largest.1, largest.0
    );

    let started = std::time::Instant::now();
    match bevox_core::vox::import_scene(&data) {
        Ok((tree, _)) => println!(
            "composed  : extent {}, {} arena nodes, {} voxel bytes, {:.2}s",
            tree.extent(),
            tree.arena().nodes().len(),
            tree.arena().voxels().len(),
            started.elapsed().as_secs_f32()
        ),
        Err(e) => println!("composed  : rejected, {e}"),
    }

    // The first few models, which is what import_model(.., 0) would pick up.
    for (i, m) in data.models.iter().take(5).enumerate() {
        println!(
            "  model {i}: {}x{}x{}, {} voxels",
            m.size.x,
            m.size.y,
            m.size.z,
            m.voxels.len()
        );
    }
}
