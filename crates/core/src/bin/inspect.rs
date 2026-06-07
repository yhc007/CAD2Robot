// Small helper binary inside the core crate for quick native testing during development.
// The real user-facing binary lives in crates/cli.

use cad2robot_core::{GeometryKernel, StubKernel};
use std::env;

fn main() {
    let path = env::args().nth(1).expect("usage: cargo run -p cad2robot-core --bin inspect -- <step-file>");
    let bytes = std::fs::read(&path).expect("failed to read STEP");

    let mut kernel = StubKernel::new(); // will become OcctKernel when feature is stable
    match kernel.load_step(&bytes) {
        Ok(h) => {
            println!("Loaded STEP: handle={h}");
            if let Ok(tree) = kernel.get_assembly_tree(h) {
                println!("Assembly tree (stub): {tree:?}");
            }
            if let Ok(v) = kernel.get_volume(h) {
                println!("Volume (kernel units): {v}");
            }
        }
        Err(e) => eprintln!("Error: {e}"),
    }
}
