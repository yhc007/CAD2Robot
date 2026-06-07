# Test Fixtures

This directory holds (or points to) STEP files used for golden testing and demos.

## PR1 Status

We do **not** commit large proprietary or very large STEP files.

For the integration test and `cad2robot inspect` demo you need at least one small robot-like STEP.

### Recommended sources (public / permissive)

1. GrabCAD – search "UR5", "Franka Panda", "simple robot arm", "gripper" and filter for STEP.
2. Official robot manufacturers sometimes publish STEP under permissive terms (check license).
3. Onshape public documents → export STEP (requires free account).

### How to add a fixture for local development

```bash
# Example (user action)
mkdir -p fixtures
cp ~/Downloads/my_robot_arm.step fixtures/example_arm.step
```

Then run:

```bash
cargo test -p cad2robot-core -- --ignored   # the integration test is ignored until a file exists
cargo run -p cad2robot -- inspect fixtures/example_arm.step
```

### Golden artifacts (future PRs)

When we have stable fixtures we will also commit:
- `expected_assembly_tree.json`
- `expected_inertials.json` (at default density + scale 0.001)
- `expected_urdf.urdf` + `meshes/`

A small regeneration script will live in `scripts/regen-golden.sh` (see Testing Strategy in the design doc).

## Scale note

All golden expectations must be in **meters** (URDF convention). The core applies `Metadata.scale_factor` (default 0.001).

## License

Only commit files whose license allows redistribution (or document clearly that the file is user-provided for local testing only).
