//! Integration / golden tests for STEP ingest (PR1).
//
//! Per the design document's Testing Strategy:
//! - Golden fixtures live in fixtures/
//! - Tests must assert meter-scale outputs (scale_factor applied).
//! - The real integration test is ignored until a fixture exists.

use cad2robot_core::{GeometryKernel, RobotModel, StubKernel};
use std::path::Path;

#[test]
fn model_scale_factor_default_and_application() {
    let mut model = RobotModel::new("test_robot");
    assert!((model.meta.scale_factor - 0.001).abs() < 1e-9);

    let mm = 1000.0;
    assert!((model.scale_length(mm) - 1.0).abs() < 1e-9);
    assert!((model.scale_volume(1_000_000_000.0) - 1.0).abs() < 1e-6);
}

#[test]
fn stub_kernel_basic_ingest_and_tess() {
    let mut kernel = StubKernel::new();
    // A tiny non-empty "STEP" blob is enough for the stub
    let fake_step = b"ISO-10303-21; ... minimal ...";
    let h = kernel.load_step(fake_step).expect("stub should accept non-empty input");

    let vol = kernel.get_volume(h).unwrap();
    assert!(vol > 0.0);

    let tess = kernel.tessellate(h, 0.5, 0.5).unwrap();
    assert!(!tess.positions.is_empty());
    assert!(!tess.indices.is_empty());
}

/// Real STEP integration test.
/// Ignored by default because we do not commit large STEP files.
/// To run locally:
///   1. Put a small robot STEP at fixtures/example_arm.step
///   2. cargo test -p cad2robot-core -- --ignored
#[test]
#[ignore]
fn real_step_ingest_and_scale() {
    let path = Path::new("fixtures/example_arm.step");
    if !path.exists() {
        eprintln!("No fixture at fixtures/example_arm.step — see fixtures/README.md");
        return;
    }

    let bytes = std::fs::read(path).expect("failed to read fixture");
    let mut kernel = StubKernel::new(); // replace with OcctKernel when ready
    let h = kernel.load_step(&bytes).expect("should parse the provided STEP");

    let vol_kernel_units = kernel.get_volume(h).unwrap();
    println!("Volume in kernel units: {}", vol_kernel_units);

    // In a real test we would create a RobotModel, set scale, compute expected m³, etc.
    // For PR1 we just prove we can ingest a real file and the scale math is wired.
    let mut model = RobotModel::new("from_fixture");
    // simulate what PR3 will do
    let scaled_vol = model.scale_volume(vol_kernel_units);
    assert!(scaled_vol > 0.0, "scaled volume must be positive");
    println!("Scaled volume (should be m³): {}", scaled_vol);
}
