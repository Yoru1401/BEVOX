//! The `docs/map` pages are checked against the code they describe.
//!
//! A one-page map earns its place by being true at a glance, and a page that
//! has quietly stopped being true is worse than no page — this bundle carried
//! eleven stale claims until someone went looking. So the half of each page
//! that is derivable from the source is derived here and compared, in both
//! directions: nothing the code exposes may be missing from its page, and
//! nothing a page names may be absent from the code.
//!
//! It lives in the binary crate because the maps describe all three libraries
//! and this is the one that depends on all three. Everything here reads source
//! files as text, so it needs none of their APIs.
//!
//! What is deliberately **not** checked is the half worth reading: the tick
//! order, the frame order, what each constant is for. No test can hold those.
//! The listing of one tick in `physics.md` is the most valuable thing on any of
//! these pages and the least defended — if the substeps are reordered, it lies
//! silently, and only review will catch it.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

/// Lines a map page may have.
///
/// The cap is the format, not a style rule: what does not fit becomes another
/// page, or a concept the page links to. Raising it is how a map turns back
/// into a plan. It has already earned its keep once — giving every physics
/// constant an explanation that stands on its own took `physics.md` to 177
/// lines, and the split into `physics-constants.md` was the better half of that
/// trade.
const ONE_PAGE: usize = 150;

/// Every map page, and the crate whose `pub mod` and `pub const` it must
/// account for. `index.md` describes the workspace rather than a crate.
const MAPS: [(&str, Option<&str>); 5] = [
    ("index.md", None),
    ("core.md", Some("bevox_core")),
    ("physics.md", Some("bevox_physics")),
    ("physics-constants.md", None),
    ("render.md", Some("bevox_render")),
];

/// Which page carries each crate's constant table. Physics keeps its own.
fn constants_page(krate: &str) -> (&'static str, &'static str) {
    match krate {
        "bevox_physics" => ("physics-constants.md", "# The physics constants"),
        "bevox_core" => ("core.md", "# Constants"),
        "bevox_render" => ("render.md", "# Constants"),
        other => panic!("no constant table is claimed for {other}"),
    }
}

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn read(path: impl AsRef<Path>) -> String {
    let path = path.as_ref();
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

fn page(name: &str) -> String {
    read(repo().join("docs/map").join(name))
}

/// Every `.rs` file of a crate's source, so nothing hides in a module the scan
/// forgot to look in.
fn sources(krate: &str) -> Vec<String> {
    let dir = repo().join("crates").join(krate).join("src");
    let mut out = Vec::new();
    for entry in std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("{}: {e}", dir.display())) {
        let path = entry.expect("a directory entry").path();
        if path.extension().is_some_and(|e| e == "rs") {
            out.push(read(&path));
        }
    }
    assert!(out.len() > 3, "{krate}: only {} source files; wrong directory", out.len());
    out
}

/// A crate's tunables: `pub const NAME` at the top level of a module.
///
/// Column zero on purpose. An indented one is an associated constant such as
/// `MaterialId::EMPTY` or `BodyId::WORLD` — part of a type's API, naming a
/// value that is not a choice anyone tunes, and it has no business on a page
/// headed "Constants". `pub const fn` is a function, not a value.
///
/// The one indented set that *is* a tuning surface is `march_flags`, and it has
/// a table and a gate of its own.
fn declared_constants(krate: &str) -> BTreeSet<String> {
    sources(krate)
        .iter()
        .flat_map(|src| src.lines().collect::<Vec<_>>())
        .filter_map(|line| line.strip_prefix("pub const "))
        .filter(|rest| !rest.starts_with("fn "))
        .filter_map(|rest| rest.split([':', ' ']).next())
        .map(str::to_owned)
        .collect()
}

/// The flags inside `march_flags`, which sit a level in because they are a
/// module of their own.
fn declared_flags() -> BTreeSet<String> {
    let src = read(repo().join("crates/bevox_render/src/upload.rs"));
    let body = src
        .split_once("pub mod march_flags {")
        .expect("bevox_render::upload has no march_flags module")
        .1;
    let body = body.split_once("\n}").map_or(body, |(before, _)| before);
    body.lines()
        .map(str::trim)
        .filter_map(|line| line.strip_prefix("pub const "))
        .filter_map(|rest| rest.split([':', ' ']).next())
        .map(str::to_owned)
        .collect()
}

fn declared_modules(krate: &str) -> BTreeSet<String> {
    read(repo().join("crates").join(krate).join("src/lib.rs"))
        .lines()
        .filter_map(|line| line.strip_prefix("pub mod "))
        .map(|rest| rest.trim_end_matches(';').to_owned())
        .collect()
}

/// Every backticked name in the first cell of every table row under `heading`,
/// up to the next `#` heading, with any `module::` qualifier dropped.
///
/// All of these pages put the name of the thing a row is about in its first
/// cell, so one reader serves them all — including a section split across
/// several tables, as every grouped one is, and a cell naming two things at
/// once, as the flags table does.
fn named_under(page_name: &str, heading: &str) -> BTreeSet<String> {
    let text = page(page_name);
    let body = text
        .split_once(heading)
        .unwrap_or_else(|| panic!("docs/map/{page_name} has no `{heading}` section"))
        .1;
    let body = body.split_once("\n# ").map_or(body, |(before, _)| before);
    body.lines()
        .filter(|line| line.starts_with("| `"))
        .flat_map(|line| {
            let cell = line.trim_start_matches('|').split('|').next().unwrap_or("");
            cell.split('`')
                .skip(1)
                .step_by(2)
                .map(|name| name.rsplit("::").next().unwrap_or(name).to_owned())
                .collect::<Vec<_>>()
        })
        .collect()
}

fn compare(what: &str, page_name: &str, declared: &BTreeSet<String>, named: &BTreeSet<String>) {
    let missing: Vec<_> = declared.difference(named).collect();
    let invented: Vec<_> = named.difference(declared).collect();
    assert!(
        missing.is_empty(),
        "docs/map/{page_name} does not name these {what}: {missing:?} — the page claims to \
         account for all of them, so it has to"
    );
    assert!(
        invented.is_empty(),
        "docs/map/{page_name} names {what} that do not exist: {invented:?}"
    );
}

/// Every module of every crate is on its page, and every module a page names
/// exists.
///
/// Breaks that must fail it: add a `pub mod` without a row; rename a module and
/// leave the row behind.
#[test]
fn the_maps_name_every_module() {
    for (page_name, krate) in MAPS {
        let Some(krate) = krate else { continue };
        let declared = declared_modules(krate);
        assert!(declared.len() >= 6, "{krate}: only {} modules found", declared.len());
        compare("modules", page_name, &declared, &named_under(page_name, "# Where it lives"));
    }
}

/// Every constant of every crate is on its page, and every constant a page
/// names exists.
///
/// This is what rots first: a constant is added beside the one it belongs with,
/// its page is three directories away, and nothing complains.
///
/// Breaks that must fail it: add a `pub const` without a row; delete one and
/// leave its row; rename one on either side.
#[test]
fn the_maps_name_every_constant() {
    for (_, krate) in MAPS {
        let Some(krate) = krate else { continue };
        let (page_name, heading) = constants_page(krate);
        let declared = declared_constants(krate);
        assert!(declared.len() >= 8, "{krate}: only {} constants found", declared.len());
        compare("constants", page_name, &declared, &named_under(page_name, heading));
    }
}

/// Every march flag is in the flags table, and every flag the table names
/// exists.
///
/// The flags are the switch panel for the whole renderer, and one added without
/// a row is a feature nobody downstream knows they can turn off.
///
/// Breaks that must fail it: add a flag without a row; rename one either side.
#[test]
fn the_render_map_names_every_march_flag() {
    let declared = declared_flags();
    assert!(declared.len() >= 10, "only {} march flags found: {declared:?}", declared.len());
    compare("march flags", "render.md", &declared, &named_under("render.md", "# Flags"));
}

/// Every page fits on a page.
#[test]
fn the_maps_fit_one_page() {
    for (page_name, _) in MAPS {
        let lines = page(page_name).lines().count();
        assert!(
            lines <= ONE_PAGE,
            "docs/map/{page_name} is {lines} lines against a cap of {ONE_PAGE}; what does not \
             fit belongs on a page of its own, or in a concept this one links to"
        );
    }
}
