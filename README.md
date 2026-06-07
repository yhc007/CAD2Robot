# CAD2Robot

Web-based UI/UX + Rust-native core (WASM + native/CLI/PyO3) for converting STEP CAD assemblies into simulation-ready URDF robot descriptions, with strong support for NVIDIA Isaac Sim / Omniverse, ROS, Gazebo, and RViz.

**Status**: Design complete and approved. Ready for implementation.

## Quick Links

- **Full Design Document** (the approved plan): [docs/design/CAD2Robot-STEP-to-URDF-Web-RustWASM-Design.md](docs/design/CAD2Robot-STEP-to-URDF-Web-RustWASM-Design.md)
  - Includes: architecture, data model (ER), processing pipeline, detailed web UI/UX (layout + joint definition flows), WASM API surface, Risks & Mitigations table, Testing Strategy, Key Decisions, and a 10-PRs vertical-slice implementation roadmap.

## What It Does (from the design)

1. Drag/drop a real STEP assembly (from Fusion, SolidWorks, Onshape, CATIA, etc.).
2. Interactive web tool helps you map bodies → Links, define joints with intuitive 3D picking/gizmos/snapping + live FK preview (the historically hardest part of CAD→URDF).
3. Compute accurate inertials directly from B-Rep (or mesh fallback), separate visual vs. collision geometry, tune materials/density.
4. Export a clean URDF + binary STL meshes (scale-correct, validated) that loads well in Isaac Sim's URDF importer (and other tools).
5. Same Rust core powers:
   - Fully client-side browser app (proprietary CAD never leaves your machine).
   - Native CLI (`cad2robot inspect/export ...`).
   - PyO3 Python module for direct use inside Isaac Sim scripts, Omniverse extensions, or ROS build pipelines.

## Core Tech Choices (Key Decisions excerpt)

- **Geometry kernel**: occt-wasm primary (full OCCT fidelity + direct volume/CoM/inertia queries + practical ~4.5 MB brotli WASM + same artifact for native wasmtime). Truck (monstertruck) as abstracted alternative for pure-Rust path.
- **Intermediate model**: Rich `RobotModel` (richer than URDF) owned in Rust with `GeometryHandle` + hybrid `CADRef` back-refs for re-derivation after edits/save/load. Serializable JSON.
- **Web**: Vite + TS + React + @react-three/fiber/drei + Tailwind + Zustand. Rust WASM is the source of truth; JS holds lightweight mirrors + transient previews.
- **URDF**: Own lightweight emitter (for Isaac-friendly control) + urdf-rs for parse/roundtrip validation only.
- **Units**: Hard-coded default `scale_factor: 0.001` (mm STEP → m URDF) stored in model metadata + applied everywhere + UI warning/override + golden tests.
- **Delivery**: One core → WASM (client-only web) + native CLI + PyO3. Privacy-first, no server in v1.
- **Phasing**: 10 ordered, independently reviewable+mergeable PRs that deliver early vertical value (real STEP mesh preview in browser by PR2; usable structure by PR3; first good joint UX by PR5; end-to-end export by PR7).

See the full design doc for the complete rationale, Mermaid diagrams (architecture, data model, UI layout, flow), WASM TS interface sketch, Risks table, Testing Strategy (golden fixtures, proptest parity, per-PR mandates), and exact file lists per PR with "how to demo" commands.

## Getting Started (Implementation)

The design document's **PR Plan** is the execution roadmap. Each PR is scoped to be valuable on its own.

Recommended first steps (following PR 1 + parallel prep):
1. Bootstrap the Cargo workspace + occt-wasm dep + basic kernel facade + STEP ingest + CLI `inspect` + 1 golden fixture + CI skeleton (see PR1 "Files" and starter snippets in the design doc).
2. Add the web scaffold (Vite/React/three) in parallel where possible.
3. Use the "How to demo this slice" notes in every PR.

All major risks (WASM payload/perf, STEP fidelity, units, joint UX, browser matrix incl. no Firefox, sync contract, native licensing for embeds, inertial accuracy) are called out with mitigations and PR linkages in the design.

## Success Criteria (from design)

A competent user can take a real multi-part robot STEP, produce a usable kinematic tree + inertials + collisions in <30-60 minutes, and load the resulting URDF + meshes successfully in Isaac Sim (or RViz/Gazebo) with correct scale, no wildly wrong dynamics, and minimal manual massaging.

## License & Notes

Apache-2.0 for our code. The chosen kernel (occt-wasm) embeds OCCT (LGPL-2.1 obligations documented for web override and native/CLI/Py distributions — see Security and Risks sections in the design).

This project was designed via a full write-review-revise loop (design skill) to produce a high-quality, reviewed foundation directly from the user's request for web UI/UX + Rust Native WASM program usable with Isaac Sim & Omniverse.

---

Start with the design document. Let me know which PR (or specific part) you'd like to implement next, or if you want me to scaffold the initial repo structure following the PR1 starters right now.