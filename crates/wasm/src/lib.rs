//! cad2robot-wasm
//!
//! WASM bindings for the CAD2Robot core (PR2).
//!
//! Exposes:
//! - load_step(bytes) -> handle (number)
//! - get_assembly_tree(handle) -> simple JSON description
//! - tessellate(handle) -> {positions: Float32Array, normals, indices: Uint32Array}
//!
//! The web side (three.js) consumes the arrays directly for BufferGeometry.
//!
//! For the browser preview we currently get meshes from the StubKernel (or real
//! if occt feature is somehow active). Full high-fidelity browser OCCT will use
//! the companion occt-wasm JS bindings in a future iteration.

use cad2robot_core::{GeometryKernel, StubKernel};
use js_sys::{Float32Array, Uint32Array};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
}

static mut KERNEL: Option<StubKernel> = None;

fn get_kernel() -> &'static mut StubKernel {
    unsafe {
        if KERNEL.is_none() {
            KERNEL = Some(StubKernel::new());
        }
        KERNEL.as_mut().unwrap()
    }
}

/// Load a STEP file. Returns a numeric handle for subsequent calls.
#[wasm_bindgen]
pub fn load_step(bytes: &[u8]) -> Result<u32, JsValue> {
    let kernel = get_kernel();
    kernel
        .load_step(bytes)
        .map_err(|e| JsValue::from_str(&e.to_string()))
}

/// Very simple tree description for the UI (will be richer later).
#[wasm_bindgen]
pub fn get_assembly_tree(handle: u32) -> Result<String, JsValue> {
    let kernel = get_kernel();
    let tree = kernel
        .get_assembly_tree(handle)
        .map_err(|e| JsValue::from_str(&e.to_string()))?;

    // Minimal JSON for the left panel
    let json = serde_json::to_string(&tree).unwrap_or_else(|_| "[]".to_string());
    Ok(json)
}

/// Tessellate and return transferable arrays for three.js (fine quality for visual preview).
///
/// Returns an object with:
///   positions: Float32Array
///   normals: Float32Array  
///   indices: Uint32Array
#[wasm_bindgen]
pub fn tessellate(handle: u32) -> Result<JsValue, JsValue> {
    let kernel = get_kernel();
    // Fine quality for live visual preview
    let mesh = kernel
        .tessellate(handle, 0.1, 0.1)
        .map_err(|e| JsValue::from_str(&e.to_string()))?;

    let positions = Float32Array::from(&mesh.positions[..]);
    let normals = Float32Array::from(&mesh.normals[..]);
    let indices = Uint32Array::from(&mesh.indices[..]);

    let obj = js_sys::Object::new();
    js_sys::Reflect::set(&obj, &"positions".into(), &positions)?;
    js_sys::Reflect::set(&obj, &"normals".into(), &normals)?;
    js_sys::Reflect::set(&obj, &"indices".into(), &indices)?;

    Ok(obj.into())
}

/// Simple version string / health check.
#[wasm_bindgen]
pub fn version() -> String {
    "cad2robot-wasm 0.1 (PR2 spike + occt native)".to_string()
}

use js_sys::Uint8Array;

/// Generate a full "URDF package" result from a loaded STEP handle.
/// This is the web equivalent of the CLI `convert` command.
///
/// Returns a JS object:
/// {
///   urdf: string,
///   visual_stl: Uint8Array,
///   collision_stl: Uint8Array,
///   package_name: string,
///   base_link_name: string,
///   scale: number,
///   mass: number
/// }
#[wasm_bindgen]
pub fn generate_urdf_package(
    handle: u32,
    scale: f64,
    density: f64,
    base_link_name: &str,
) -> Result<JsValue, JsValue> {
    let kernel = get_kernel();

    // Tessellate the main shape twice:
    // - Visual: fine tessellation (more triangles, good appearance)
    // - Collision: coarser tessellation (fewer triangles → simpler/faster collision geometry for physics)
    // This fulfills the request to simplify collision mesh via coarser tessellation.
    // (Convex hull would be an even stronger simplification and can be added later via kernel.convex_hull if needed.)
    let visual_mesh = kernel
        .tessellate(handle, 0.1, 0.1)   // fine
        .map_err(|e| JsValue::from_str(&e.to_string()))?;

    let collision_mesh = kernel
        .tessellate(handle, 2.0, 2.0)   // coarser for collision
        .map_err(|e| JsValue::from_str(&e.to_string()))?;

    // Very rough mass (same logic as CLI)
    // Note: for real high-quality results the kernel should provide volume/CoM.
    // Here we use a placeholder volume (the stub always gives 1000 units).
    let vol_kernel: f64 = 1000.0; // TODO: replace with real kernel.get_volume when available in WASM
    let scaled_vol = vol_kernel * scale.powi(3);
    let mass = scaled_vol * density;

    let scaled_com = [0.0, 0.0, 0.0]; // TODO: use real CoM scaled

    let package_name = format!("{}_description", base_link_name.replace(' ', "_"));

    // Generate STL bytes (scaled) — collision now uses the coarser mesh
    let visual_bytes = cad2robot_core::mesh_to_stl_bytes(&visual_mesh, scale);
    let collision_bytes = cad2robot_core::mesh_to_stl_bytes(&collision_mesh, scale);

    // Generate URDF text
    let urdf = cad2robot_core::generate_minimal_urdf(
        &package_name,
        base_link_name,
        scale,
        mass,
        scaled_com,
    );

    // Build JS return value
    let obj = js_sys::Object::new();
    js_sys::Reflect::set(&obj, &"urdf".into(), &urdf.into())?;
    js_sys::Reflect::set(
        &obj,
        &"visual_stl".into(),
        &Uint8Array::from(&visual_bytes[..]),
    )?;
    js_sys::Reflect::set(
        &obj,
        &"collision_stl".into(),
        &Uint8Array::from(&collision_bytes[..]),
    )?;
    js_sys::Reflect::set(&obj, &"package_name".into(), &package_name.into())?;
    js_sys::Reflect::set(&obj, &"base_link_name".into(), &base_link_name.into())?;
    js_sys::Reflect::set(&obj, &"scale".into(), &scale.into())?;
    js_sys::Reflect::set(&obj, &"mass".into(), &mass.into())?;

    Ok(obj.into())
}