//! Property-based style invariants (will use proptest in later PRs).
//! For PR1 we keep simple unit tests that will evolve into the full Testing Strategy.

use cad2robot_core::RobotModel;

#[test]
fn root_model_has_sane_defaults() {
    let model = RobotModel::new("demo");
    assert_eq!(model.links.len(), 0);
    assert_eq!(model.joints.len(), 0);
    assert!((model.meta.scale_factor - 0.001).abs() < 1e-12);
}
