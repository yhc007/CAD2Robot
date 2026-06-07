//! CAD2Robot Core
//!
//! Authoritative in-memory Robot Model + geometry kernel abstraction.
//! Designed so the exact same types and logic power:
//! - Native CLI
//! - WASM (via thin wasm-bindgen surface)
//! - PyO3 (future)
//!
//! See the design document for the full data model and rationale.

pub mod error;
pub mod kernel;
pub mod model;

// Re-exports for convenience
pub use error::CoreError;
pub use kernel::{GeometryKernel, KernelHandle, StubKernel};
#[cfg(feature = "occt")]
pub use kernel::OcctKernel;
pub use model::{
    generate_minimal_urdf, mesh_to_stl_bytes, write_binary_stl, Inertial, Metadata, Pose,
    RobotModel,
};

/// Convenience result type used throughout the crate.
pub type Result<T> = std::result::Result<T, CoreError>;
