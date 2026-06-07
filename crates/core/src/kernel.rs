//! Geometry kernel abstraction (the heart of CAD ingestion).
//!
//! Design goal: one trait so we can swap occt-wasm <-> truck (or future kernels)
//! without touching the rest of the model or exporters.
//!
//! PR1 scope: trait + basic ingest surface (assembly tree + volume/CoM).
//! Full tessellation, inertia tensor, and collision simplification come in later PRs.

use crate::error::CoreError;
use crate::model::{MeshSnapshot, Pose};
use crate::Result;

/// Stable handle type returned by the kernel (u32 arena index in occt-wasm).
pub type KernelHandle = u32;

/// High-level description of a body/part coming from the STEP assembly.
#[derive(Debug, Clone, serde::Serialize)]
pub struct AssemblyNode {
    pub handle: KernelHandle,
    pub name: String,
    pub children: Vec<AssemblyNode>,
}

/// Trait that all kernels must implement.
pub trait GeometryKernel: Send + Sync {
    /// Load a STEP file (bytes). Returns a root assembly handle.
    fn load_step(&mut self, bytes: &[u8]) -> Result<KernelHandle>;

    /// Return a hierarchical view of the assembly (names + handles).
    fn get_assembly_tree(&mut self, root: KernelHandle) -> Result<Vec<AssemblyNode>>;

    /// Compute volume in the kernel's native units (caller applies scale).
    fn get_volume(&mut self, handle: KernelHandle) -> Result<f64>;

    /// Center of mass in kernel units (caller applies scale + transforms).
    fn get_center_of_mass(&mut self, handle: KernelHandle) -> Result<[f64; 3]>;

    /// Tessellate the shape for rendering or collision.
    /// linear_deflection and angular_deflection control quality (smaller = finer mesh, more triangles).
    /// For visual: small values (e.g. 0.1). For collision: larger values (e.g. 1.0~5.0) or use convex hull later.
    fn tessellate(&mut self, handle: KernelHandle, linear_deflection: f64, angular_deflection: f64) -> Result<MeshSnapshot>;

    /// Compute full inertial tensor (later; returns (mass, com, inertia_xx_xy...)).
    fn compute_inertial(&mut self, handle: KernelHandle, density: f64) -> Result<(f64, Pose, [f64; 6])>;

    /// Release resources associated with a handle (important for WASM memory).
    fn dispose(&mut self, handle: KernelHandle);
}

/// Real implementation using occt-wasm (feature "occt").
///
/// This is the production kernel as recommended in the design document.
/// It provides excellent fidelity for real commercial STEP files (including XCAF
/// assemblies, names, colors) and direct B-Rep queries for volume, center of mass,
/// and (in future) inertia without mesh approximation.
///
/// Note: `OcctKernel::new()` is relatively expensive the first time (decompress +
/// JIT the embedded ~4.7MB brotli WASM via wasmtime). Reuse the kernel instance.
#[cfg(feature = "occt")]
pub struct OcctKernel {
    inner: occt_wasm::OcctKernel,
    /// We keep our own list of ShapeHandles. Our public KernelHandle is a stable
    /// index into this vec (1-based for nicer debugging).
    shapes: Vec<occt_wasm::ShapeHandle>,
}

#[cfg(feature = "occt")]
impl OcctKernel {
    pub fn new() -> Result<Self> {
        let inner = occt_wasm::OcctKernel::new()
            .map_err(|e| CoreError::Kernel(format!("failed to init OcctKernel: {}", e)))?;
        Ok(Self {
            inner,
            shapes: vec![], // handles are indices into this vec
        })
    }

    fn to_shape(&self, h: KernelHandle) -> Result<occt_wasm::ShapeHandle> {
        self.shapes
            .get(h as usize)
            .cloned()
            .ok_or(CoreError::InvalidHandle)
    }
}

#[cfg(feature = "occt")]
impl GeometryKernel for OcctKernel {
    fn load_step(&mut self, bytes: &[u8]) -> Result<KernelHandle> {
        let data = std::str::from_utf8(bytes)
            .map_err(|_| CoreError::StepParse("STEP data is not valid UTF-8".into()))?;

        let shape = self
            .inner
            .import_step(data)
            .map_err(|e| CoreError::StepParse(format!("occt import_step failed: {}", e)))?;

        self.shapes.push(shape);
        Ok((self.shapes.len() - 1) as KernelHandle)
    }

    fn get_assembly_tree(&mut self, root: KernelHandle) -> Result<Vec<AssemblyNode>> {
        // Simple flat view for the spike. Full XCAF tree (names, hierarchy, colors)
        // can be implemented using xcaf_* methods + get_sub_shapes.
        let _ = self.to_shape(root)?; // validate
        let name = format!("shape_{}", root);
        Ok(vec![AssemblyNode {
            handle: root,
            name,
            children: vec![],
        }])
    }

    fn get_volume(&mut self, handle: KernelHandle) -> Result<f64> {
        let shape = self.to_shape(handle)?;
        self.inner
            .get_volume(shape)
            .map_err(|e| CoreError::Kernel(format!("get_volume: {}", e)))
    }

    fn get_center_of_mass(&mut self, handle: KernelHandle) -> Result<[f64; 3]> {
        let shape = self.to_shape(handle)?;
        let v = self
            .inner
            .get_center_of_mass(shape)
            .map_err(|e| CoreError::Kernel(format!("get_center_of_mass: {}", e)))?;
        if v.len() >= 3 {
            Ok([v[0], v[1], v[2]])
        } else {
            Ok([0.0, 0.0, 0.0])
        }
    }

    fn tessellate(&mut self, handle: KernelHandle, linear_deflection: f64, angular_deflection: f64) -> Result<MeshSnapshot> {
        let shape = self.to_shape(handle)?;
        // Use provided deflection/angle for quality control.
        // Visual: small values (fine mesh). Collision: large values (coarse, fewer triangles).
        let mesh = self
            .inner
            .tessellate(shape, linear_deflection, angular_deflection)
            .map_err(|e| CoreError::Kernel(format!("tessellate: {}", e)))?;

        Ok(MeshSnapshot {
            positions: mesh.positions,
            normals: mesh.normals,
            indices: mesh.indices,
        })
    }

    fn compute_inertial(&mut self, handle: KernelHandle, density: f64) -> Result<(f64, Pose, [f64; 6])> {
        let vol = self.get_volume(handle)?;
        let mass = vol * density;
        let com = self.get_center_of_mass(handle)?;
        // Placeholder inertia until we wire a proper tensor query.
        let ixx = mass * 0.1;
        let inertia = [ixx, 0.0, 0.0, ixx, 0.0, ixx];
        Ok((
            mass,
            Pose {
                xyz: com,
                quat: [1.0, 0.0, 0.0, 0.0],
            },
            inertia,
        ))
    }

    fn dispose(&mut self, handle: KernelHandle) {
        if (handle as usize) < self.shapes.len() {
            let shape = self.shapes[handle as usize];
            let _ = self.inner.release(shape);
            // We leave the slot (or could swap_remove + remap, but for simplicity we just release in OCCT).
        }
    }
}

/// Fallback implementation that always works (great for early tests and when occt feature is off).
pub struct StubKernel {
    next_handle: KernelHandle,
}

impl StubKernel {
    pub fn new() -> Self {
        Self { next_handle: 1 }
    }
}

impl GeometryKernel for StubKernel {
    fn load_step(&mut self, bytes: &[u8]) -> Result<KernelHandle> {
        if bytes.is_empty() {
            return Err(CoreError::StepParse("empty STEP".into()));
        }
        let h = self.next_handle;
        self.next_handle += 1;
        Ok(h)
    }

    fn get_assembly_tree(&mut self, root: KernelHandle) -> Result<Vec<AssemblyNode>> {
        Ok(vec![AssemblyNode {
            handle: root,
            name: "stub_body_from_step".to_string(),
            children: vec![],
        }])
    }

    fn get_volume(&mut self, _handle: KernelHandle) -> Result<f64> {
        Ok(1000.0) // 1 liter in mm³
    }

    fn get_center_of_mass(&mut self, _handle: KernelHandle) -> Result<[f64; 3]> {
        Ok([0.0, 0.0, 0.0])
    }

    fn tessellate(&mut self, _handle: KernelHandle, linear_deflection: f64, _angular_deflection: f64) -> Result<MeshSnapshot> {
        // Stub: for coarse collision (large deflection), return a very simple tetrahedron (4 triangles)
        // instead of full cube. This demonstrates "simpler collision mesh".
        // Real OCCT will use the deflection params to produce significantly fewer triangles for collision.
        if linear_deflection > 1.0 {
            // Simple tetrahedron (very coarse collision proxy)
            return Ok(MeshSnapshot {
                positions: vec![
                    0.,0.,0.,  2.,0.,0.,  1.,2.,0.,  1.,1.,2.,
                ],
                normals: vec![0.; 12],
                indices: vec![
                    0,1,2,  0,1,3,  0,2,3,  1,2,3,
                ],
            });
        }

        // Fine: unit cube (12 triangles)
        Ok(MeshSnapshot {
            positions: vec![
                0.,0.,0., 1.,0.,0., 1.,1.,0., 0.,1.,0.,
                0.,0.,1., 1.,0.,1., 1.,1.,1., 0.,1.,1.,
            ],
            normals: vec![0.; 24],
            indices: vec![
                0,1,2, 0,2,3, 4,5,6, 4,6,7,
                0,1,5, 0,5,4, 2,3,7, 2,7,6,
                0,3,7, 0,7,4, 1,2,6, 1,6,5,
            ],
        })
    }

    fn compute_inertial(&mut self, handle: KernelHandle, density: f64) -> Result<(f64, Pose, [f64; 6])> {
        let vol = self.get_volume(handle)?;
        let mass = vol * density * 1e-9; // very rough mm³ → m³ adjustment for demo
        let i = [mass * 0.083, 0., 0., mass * 0.083, 0., mass * 0.083];
        Ok((mass, Pose { xyz: [0.,0.,0.], quat: [1.,0.,0.,0.] }, i))
    }

    fn dispose(&mut self, _handle: KernelHandle) {}
}
