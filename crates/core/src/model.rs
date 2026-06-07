//! Intermediate Robot Model (richer than URDF).
//!
//! This is the live, editable source of truth. It stores back-references
//! (CADRef) so that changing density, tessellation params, or joint axes
//! can re-derive geometry and inertial properties without re-parsing the
//! original STEP.
//!
//! Key decision from design: scale_factor lives in Metadata and is applied
//! uniformly (default 0.001 for mm STEP → m URDF).

use serde::{Deserialize, Serialize};

use crate::kernel::KernelHandle;

/// Project-level metadata. Persisted with every RobotModel snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metadata {
    /// STEP files are typically in millimeters. URDF / Isaac / ROS expect meters.
    /// Default = 0.001 (mm → m). Applied to all coordinates, volumes (³), masses, inertias.
    pub scale_factor: f64,

    /// Human readable name for the robot (will become <robot name="...">).
    pub robot_name: String,

    /// Free-form user notes / provenance.
    #[serde(default)]
    pub notes: String,
}

impl Default for Metadata {
    fn default() -> Self {
        Self {
            scale_factor: 0.001,
            robot_name: "my_robot".to_string(),
            notes: String::new(),
        }
    }
}

/// 6-DOF pose (xyz + quaternion or rpy). We store both for convenience.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub xyz: [f64; 3],
    pub quat: [f64; 4], // w, x, y, z
}

/// Inertial properties for a Link (in the link's own frame, after scale).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Inertial {
    pub mass: f64,
    pub origin: Pose,
    /// Inertia tensor in the inertial frame (xx, xy, xz, yy, yz, zz).
    pub inertia: [f64; 6],
}

/// Reference back to original CAD entity (hybrid strategy per design).
/// `semantic` is the persistent, serializable part (XCAF label, part name, "face:23").
/// `kernel_handle` is only valid for the current kernel session (used for high-fidelity picking).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CADRef {
    pub semantic: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kernel_handle: Option<KernelHandle>,
}

/// Handle to geometry owned by the kernel (visual or collision).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometryHandle {
    pub cad_ref: CADRef,
    /// Optional cached tessellation (positions/normals/indices as flat arrays).
    /// Populated on demand by the kernel.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual_tess: Option<MeshSnapshot>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collision_tess: Option<MeshSnapshot>,
}

/// Lightweight tessellation snapshot (transferable to WASM/JS).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshSnapshot {
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub indices: Vec<u32>,
}

/// A rigid body in the robot (corresponds to one <link> in URDF).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Link {
    pub id: String,
    pub name: String,
    pub visual: Vec<GeometryHandle>,
    pub collision: Vec<GeometryHandle>,
    pub inertial: Inertial,
    /// Density used for the last inertial computation (kg/m³).
    pub density_kg_m3: f64,
}

/// Joint type (URDF compatible).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum JointType {
    Fixed,
    Revolute,
    Prismatic,
    Continuous,
}

/// A joint between two links.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    pub id: String,
    pub name: String,
    pub parent: String, // link id
    pub child: String,  // link id
    pub joint_type: JointType,
    pub origin: Pose,
    pub axis: [f64; 3], // unit vector in child frame
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limits: Option<JointLimits>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

/// The canonical live robot description.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotModel {
    pub meta: Metadata,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    /// Root link (the one with no parent joint).
    pub root_link_id: String,
}

impl RobotModel {
    pub fn new(name: impl Into<String>) -> Self {
        let meta = Metadata {
            robot_name: name.into(),
            ..Default::default()
        };
        Self {
            meta,
            links: vec![],
            joints: vec![],
            root_link_id: String::new(),
        }
    }

    /// Apply the project's scale_factor to a raw length (mm → m etc.).
    pub fn scale_length(&self, v: f64) -> f64 {
        v * self.meta.scale_factor
    }

    /// Apply scale to a volume-derived quantity (mm³ → m³).
    pub fn scale_volume(&self, v: f64) -> f64 {
        v * self.meta.scale_factor.powi(3)
    }
}

/// Write a binary STL (little-endian) from the given mesh snapshot.
/// Vertices are scaled by `scale` (e.g. 0.001 to go from mm in STEP to meters in URDF).
pub fn write_binary_stl(
    mesh: &MeshSnapshot,
    mut w: impl std::io::Write,
    scale: f64,
) -> std::io::Result<()> {
    // 80-byte header
    let mut header = [0u8; 80];
    let title = b"CAD2Robot binary STL";
    header[..title.len()].copy_from_slice(title);
    w.write_all(&header)?;

    let tri_count = (mesh.indices.len() / 3) as u32;
    w.write_all(&tri_count.to_le_bytes())?;

    for tri in mesh.indices.chunks_exact(3) {
        // Normal (zero is acceptable; viewers often recompute)
        let n = [0f32, 0f32, 0f32];
        w.write_all(&n[0].to_le_bytes())?;
        w.write_all(&n[1].to_le_bytes())?;
        w.write_all(&n[2].to_le_bytes())?;

        for &i in tri {
            let base = i as usize * 3;
            for j in 0..3 {
                let v = mesh.positions[base + j] as f64 * scale;
                let vf = v as f32;
                w.write_all(&vf.to_le_bytes())?;
            }
        }

        let attr = 0u16;
        w.write_all(&attr.to_le_bytes())?;
    }

    Ok(())
}

/// Convert a MeshSnapshot to raw binary STL bytes (with scale applied).
pub fn mesh_to_stl_bytes(mesh: &MeshSnapshot, scale: f64) -> Vec<u8> {
    let mut buf = Vec::new();
    // ignore error for in-memory
    let _ = write_binary_stl(mesh, &mut buf, scale);
    buf
}

/// Generate a minimal single-link URDF string suitable for early prototyping / Isaac Sim.
///
/// This mirrors the logic currently used by the CLI convert command.
pub fn generate_minimal_urdf(
    package_name: &str,
    base_link_name: &str,
    scale: f64,
    mass: f64,
    com: [f64; 3],
) -> String {
    format!(
        r#"<?xml version="1.0" ?>
<robot name="{base_link_name}">
  <!-- Generated by cad2robot web UI / convert -->
  <!-- scale_factor applied: {scale} (mm → m) -->
  <!-- TODO(PR3+): full kinematic tree, joints, accurate inertia, multiple links -->

  <link name="{base_link_name}">
    <inertial>
      <origin xyz="{cx:.6} {cy:.6} {cz:.6}" rpy="0 0 0" />
      <mass value="{mass:.6}" />
      <!-- Placeholder inertia - will be replaced by proper tensor -->
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>

    <visual>
      <geometry>
        <mesh filename="package://{package_name}/meshes/{base_link_name}_visual.stl" />
      </geometry>
    </visual>

    <collision>
      <geometry>
        <mesh filename="package://{package_name}/meshes/{base_link_name}_collision.stl" />
      </geometry>
    </collision>
  </link>

</robot>
"#,
        base_link_name = base_link_name,
        package_name = package_name,
        scale = scale,
        cx = com[0],
        cy = com[1],
        cz = com[2],
        mass = mass,
    )
}

