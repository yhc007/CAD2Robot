# Contributing to CAD2Robot

Thank you for helping build a modern, web-first, Rust-powered STEP → URDF tool!

## Development Principles (from the approved design)

- Follow the **PR Plan** in `docs/design/CAD2Robot-STEP-to-URDF-Web-RustWASM-Design.md`.
- Every PR must be **independently reviewable and mergeable**.
- Early PRs deliver **vertical slices** of value (you can demo something useful after PR1 and PR2).
- **Tests + "how to demo"** are mandatory in every PR description.

## Getting Started (PR1 style)

```bash
# 1. Clone + build
cargo check -p cad2robot-core
cargo test -p cad2robot-core

# 2. Try the CLI (needs a STEP file)
cargo run -p cad2robot -- inspect fixtures/example_arm.step
```

See `fixtures/README.md` for how to obtain a test STEP.

## Checklist for every PR

- [ ] Added or updated tests (see Testing Strategy in the design doc)
- [ ] `cargo test --all` and relevant wasm/node tests pass
- [ ] Added/updated "How to demo this slice" in the PR description + ideally in docs
- [ ] Updated relevant section of the design doc if architecture changed
- [ ] Considered scale_factor (units) and the hybrid CADRef strategy
- [ ] Ran `cargo clippy --all-targets -- -D warnings`

## Kernel Choice

We are currently using a `StubKernel` + feature-gated `OcctKernel` (occt-wasm).
A short spike to finalize the real kernel integration is tracked as Open Question #1 in the design.

## Questions?

Open an issue or discuss in the design document comments.
