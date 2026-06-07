//! cad2robot - command line interface
//!
//! PR1 delivers the `inspect` subcommand as the first vertical slice.
//! Later PRs will add `convert`, `validate`, etc.

use anyhow::Result;
use clap::{Parser, Subcommand};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[derive(Parser)]
#[command(name = "cad2robot", version, about = "STEP CAD → Robot Description (URDF) toolkit")]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    /// Enable verbose logging
    #[arg(short, long, global = true)]
    verbose: bool,
}

#[derive(Subcommand)]
enum Commands {
    /// Inspect a STEP file and print basic geometric properties (PR1)
    Inspect {
        /// Path to .step or .stp file
        step: String,

        /// Assume STEP units are millimeters (default). Future: --units mm|inch
        #[arg(long, default_value_t = true)]
        mm: bool,
    },

    /// Convert STEP into a minimal URDF skeleton (basic support; full export in later PRs)
    Convert {
        /// Path to the .step / .stp file
        step: String,

        /// Optional output path for the .urdf (defaults to <stem>.urdf next to the input)
        #[arg(short, long)]
        output: Option<String>,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let level = if cli.verbose { "debug" } else { "info" };
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| format!("cad2robot={level},cad2robot_core={level}").into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .init();

    // Create the kernel (prefer real OCCT when the feature is enabled).
    // occt-wasm can fail to initialize on some hosts (missing Emscripten imports in wasmtime).
    // We fall back gracefully so the tool remains usable.
    let mut kernel: Box<dyn cad2robot_core::GeometryKernel> = if cfg!(feature = "occt") {
        match cad2robot_core::OcctKernel::new() {
            Ok(k) => Box::new(k),
            Err(e) => {
                eprintln!("Warning: Could not initialize real OcctKernel: {}", e);
                eprintln!("         Falling back to geometric stub. Real OCCT fidelity is not available in this environment.");
                eprintln!("         (This is a known integration limitation of occt-wasm on some wasmtime hosts.)");
                Box::new(cad2robot_core::StubKernel::new())
            }
        }
    } else {
        Box::new(cad2robot_core::StubKernel::new())
    };

    match cli.command {
        Commands::Inspect { step, mm: _ } => {
            println!("Inspecting STEP: {}", step);
            let bytes = std::fs::read(&step)?;

            let handle = kernel.load_step(&bytes)?;
            println!("Loaded successfully (handle={}).", handle);

            if let Ok(tree) = kernel.get_assembly_tree(handle) {
                println!("\nTop-level nodes:");
                for node in tree {
                    println!("  - {} (handle {})", node.name, node.handle);
                }
            }

            let vol = kernel.get_volume(handle)?;
            let com = kernel.get_center_of_mass(handle)?;
            println!("\nVolume (kernel units): {:.3}", vol);
            println!("Center of mass (kernel units): [{:.3}, {:.3}, {:.3}]", com[0], com[1], com[2]);

            println!("\nTip: Use `convert` (or the web UI) to produce an initial URDF skeleton from this geometry.");
            println!("      Full kinematic tree, visual/collision meshes and accurate inertials come in later PRs.");
        }

        Commands::Convert { step, output } => {
            println!("Converting STEP: {}", step);
            let bytes = std::fs::read(&step)?;

            let handle = kernel.load_step(&bytes)?;
            println!("  Loaded (handle={})", handle);

            let vol_kernel = kernel.get_volume(handle).unwrap_or(0.0);
            let com_kernel = kernel.get_center_of_mass(handle).unwrap_or([0.0, 0.0, 0.0]);

            // --- Scale handling (mm in STEP → meters for URDF/Isaac) ---
            // TODO: make this come from a proper RobotModel / CLI flag in future.
            let scale: f64 = 0.001;

            let scaled_vol = vol_kernel * scale.powi(3);
            const STEEL_DENSITY: f64 = 7800.0; // kg/m³
            let mass = scaled_vol * STEEL_DENSITY;

            let scaled_com = [
                com_kernel[0] * scale,
                com_kernel[1] * scale,
                com_kernel[2] * scale,
            ];

            // Derive names
            let input_path = std::path::Path::new(&step);
            let stem = input_path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("robot")
                .replace([' ', '-', '.'], "_");

            let package_name = format!("{}_description", stem);

            // Determine output location
            let base_dir = if let Some(ref o) = output {
                let p = std::path::Path::new(o);
                if o.ends_with(".urdf") {
                    p.parent().unwrap_or(std::path::Path::new(".")).to_path_buf()
                } else {
                    p.to_path_buf()
                }
            } else {
                std::env::current_dir().unwrap_or_else(|_| std::path::PathBuf::from("."))
            };

            let package_dir = base_dir.join(&package_name);
            let urdf_dir = package_dir.join("urdf");
            let meshes_dir = package_dir.join("meshes");

            std::fs::create_dir_all(&urdf_dir)?;
            std::fs::create_dir_all(&meshes_dir)?;

            // --- Tessellate and export meshes (scaled to meters) ---
            // Visual: fine tessellation for good visuals
            // Collision: coarser tessellation → much simpler mesh (fewer triangles) for faster/ more stable physics
            // This is the "coarser tessellation" approach for simplifying collision geometry.
            // (Convex hull would be an even stronger option for very simple collision and can be added later.)
            let visual_mesh = kernel.tessellate(handle, 0.1, 0.1)?;     // fine
            let collision_mesh = kernel.tessellate(handle, 2.0, 2.0)?;  // coarser

            let visual_stl_path = meshes_dir.join("base_link_visual.stl");
            {
                let mut f = std::fs::File::create(&visual_stl_path)?;
                cad2robot_core::write_binary_stl(&visual_mesh, &mut f, scale)?;
            }

            let collision_stl_path = meshes_dir.join("base_link_collision.stl");
            {
                let mut f = std::fs::File::create(&collision_stl_path)?;
                cad2robot_core::write_binary_stl(&collision_mesh, &mut f, scale)?;
            }

            // --- Generate URDF with mesh references ---
            let urdf_content = format!(
                r#"<?xml version="1.0" ?>
<robot name="{stem}">
  <!-- Generated by cad2robot convert -->
  <!-- scale_factor applied: {scale} (mm → m) -->
  <!-- TODO: full kinematic tree, joints, accurate inertia, multiple links (PR3+) -->

  <link name="base_link">
    <inertial>
      <origin xyz="{cx:.6} {cy:.6} {cz:.6}" rpy="0 0 0" />
      <mass value="{mass:.6}" />
      <!-- Placeholder inertia - will be replaced by proper tensor -->
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>

    <visual>
      <geometry>
        <mesh filename="package://{package_name}/meshes/base_link_visual.stl" />
      </geometry>
    </visual>

    <collision>
      <geometry>
        <mesh filename="package://{package_name}/meshes/base_link_collision.stl" />
      </geometry>
    </collision>
  </link>

</robot>
"#,
                stem = stem,
                scale = scale,
                cx = scaled_com[0],
                cy = scaled_com[1],
                cz = scaled_com[2],
                mass = mass,
                package_name = package_name,
            );

            let urdf_path = urdf_dir.join(format!("{}.urdf", stem));
            std::fs::write(&urdf_path, urdf_content)?;

            println!("\n✅ Conversion complete!");
            println!("   Package: {}", package_dir.display());
            println!("   URDF:    {}", urdf_path.display());
            println!("   Visual:  {}", visual_stl_path.display());
            println!("   Collision: {}", collision_stl_path.display());
            println!();
            println!("   Volume (kernel units): {:.3}", vol_kernel);
            println!("   Scaled volume (m³):    {:.6}", scaled_vol);
            println!("   Approx mass (kg):      {:.3}", mass);
            println!();
            println!("Tip: You can now load the URDF in Isaac Sim / RViz.");
            println!("     For multi-link robots and proper joint definition, use the web UI (PR3+) or improve this file manually.");
        }
    }

    Ok(())
}
