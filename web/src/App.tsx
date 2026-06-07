import { useState, useEffect, useRef } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import JSZip from 'jszip'
import './App.css'

// Dynamic import of the generated wasm-bindgen glue (produced by wasm-pack)
let wasmApi: any = null

async function ensureWasm() {
  if (wasmApi) return wasmApi
  // @ts-ignore - generated at build time by wasm-pack
  const mod = await import('../pkg/cad2robot_wasm.js')
  await mod.default() // init the WASM
  wasmApi = mod
  return wasmApi
}

function LoadedMesh({ meshData }: { meshData: any }) {
  const meshRef = useRef<THREE.Mesh>(null!)

  useEffect(() => {
    if (!meshData || !meshRef.current) return

    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute('position', new THREE.BufferAttribute(meshData.positions, 3))
    if (meshData.normals && meshData.normals.length > 0) {
      geometry.setAttribute('normal', new THREE.BufferAttribute(meshData.normals, 3))
    }
    geometry.setIndex(new THREE.BufferAttribute(meshData.indices, 1))

    if (meshRef.current.geometry) meshRef.current.geometry.dispose()
    meshRef.current.geometry = geometry
  }, [meshData])

  return (
    <mesh ref={meshRef}>
      <meshStandardMaterial color="#22c55e" side={THREE.DoubleSide} />
    </mesh>
  )
}

function Scene({ currentMesh }: { currentMesh: any }) {
  return (
    <>
      <ambientLight intensity={0.6} />
      <directionalLight position={[10, 10, 5]} intensity={1} />
      {currentMesh ? (
        <LoadedMesh meshData={currentMesh} />
      ) : (
        <mesh>
          <boxGeometry args={[1.2, 0.8, 1.0]} />
          <meshStandardMaterial color="#4f46e5" />
        </mesh>
      )}
      <gridHelper args={[10, 10]} />
    </>
  )
}

interface PackageResult {
  urdf: string
  visual_stl: Uint8Array
  collision_stl: Uint8Array
  package_name: string
  base_link_name: string
  scale: number
  mass: number
}

export default function App() {
  const [fileName, setFileName] = useState<string | null>(null)
  const [status, setStatus] = useState('Drop a .step / .stp file — real WASM tessellation active')
  const [currentMesh, setCurrentMesh] = useState<any>(null)
  const [currentHandle, setCurrentHandle] = useState<number | null>(null)
  const [wasmReady, setWasmReady] = useState(false)

  // URDF Package state
  const [pkgScale, setPkgScale] = useState(0.001)
  const [pkgDensity, setPkgDensity] = useState(7800)
  const [baseLinkName, setBaseLinkName] = useState('base_link')
  const [packageResult, setPackageResult] = useState<PackageResult | null>(null)
  const [isGenerating, setIsGenerating] = useState(false)

  useEffect(() => {
    ensureWasm()
      .then(() => setWasmReady(true))
      .catch((e) => setStatus('WASM init failed: ' + e))
  }, [])

  const handleDrop = async (e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault()
    const file = e.dataTransfer.files[0]
    if (!file || !(file.name.endsWith('.step') || file.name.endsWith('.stp'))) {
      setStatus('Please drop a STEP file (.step or .stp)')
      return
    }

    setFileName(file.name)
    setPackageResult(null)
    setStatus(`Loading ${file.name} via WASM...`)

    try {
      const api = await ensureWasm()
      const bytes = new Uint8Array(await file.arrayBuffer())

      const handle: number = api.load_step(bytes)
      setCurrentHandle(handle)
      setStatus(`STEP loaded (handle=${handle}). Tessellating...`)

      const meshData = api.tessellate(handle)
      setCurrentMesh(meshData)
      setStatus(`Preview ready: ${file.name} (${(meshData.positions.length / 3) | 0} verts). Configure & generate URDF package on the right.`)
    } catch (err: any) {
      console.error(err)
      setStatus('Error: ' + (err?.message || err))
    }
  }

  const generatePackage = async () => {
    if (!currentHandle) {
      setStatus('Load a STEP first by dragging it into the 3D viewport.')
      return
    }

    setIsGenerating(true)
    setStatus('Generating URDF package (tessellating main shape → STL → URDF with <visual>/<collision>)...')

    try {
      const api = await ensureWasm()
      const result = await api.generate_urdf_package(currentHandle, pkgScale, pkgDensity, baseLinkName)
      setPackageResult(result as PackageResult)
      setStatus(`Package ready: ${result.package_name}. Preview the URDF below and use the download buttons.`)
    } catch (err: any) {
      console.error(err)
      setStatus('Package generation failed: ' + (err?.message || err))
    } finally {
      setIsGenerating(false)
    }
  }

  const downloadText = (content: string, filename: string) => {
    const blob = new Blob([content], { type: 'text/plain;charset=utf-8' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = filename
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  const downloadBinary = (bytes: Uint8Array, filename: string) => {
    const blob = new Blob([new Uint8Array(bytes)], { type: 'application/octet-stream' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = filename
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  const downloadFullZip = async () => {
    if (!packageResult) return

    const zip = new JSZip()
    const meshesFolder = zip.folder('meshes')!

    zip.file(`${packageResult.base_link_name}.urdf`, packageResult.urdf)
    meshesFolder.file(`${packageResult.base_link_name}_visual.stl`, new Blob([new Uint8Array(packageResult.visual_stl)]))
    meshesFolder.file(`${packageResult.base_link_name}_collision.stl`, new Blob([new Uint8Array(packageResult.collision_stl)]))

    const content = await zip.generateAsync({ type: 'blob' })
    const url = URL.createObjectURL(content)
    const a = document.createElement('a')
    a.href = url
    a.download = `${packageResult.package_name}.zip`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  return (
    <div className="app">
      <header className="topbar">
        <h1>CAD2Robot</h1>
        <span className="tag">STEP → URDF (Isaac Sim ready) — Web UI</span>
        <span style={{ marginLeft: 'auto', fontSize: 12, opacity: 0.6 }}>
          {wasmReady ? 'WASM ready' : 'Loading WASM...'}
        </span>
      </header>

      <div className="layout">
        <aside className="panel left">
          <h3>Parts (from STEP)</h3>
          <div className="placeholder">
            {fileName ? `Loaded: ${fileName}` : 'Drop STEP to populate tree (future)'}
          </div>
          <small>Live tessellation preview below. Generate full package on the right.</small>
        </aside>

        <main
          className="viewport"
          onDragOver={(e) => e.preventDefault()}
          onDrop={handleDrop}
        >
          <Canvas camera={{ position: [3, 3, 4], fov: 50 }} style={{ background: '#0a0a0a' }}>
            <Scene currentMesh={currentMesh} />
            <OrbitControls />
          </Canvas>

          <div className="overlay">
            <div className="status">{status}</div>
            <div className="hint">Drag &amp; drop a STEP file. Then use the panel on the right to generate URDF + STL with &lt;visual&gt; / &lt;collision&gt;.</div>
          </div>
        </main>

        <aside className="panel right">
          <h3>URDF Package (Preview + Download)</h3>

          <div className="placeholder" style={{ marginBottom: 12, fontSize: 12 }}>
            {currentMesh
              ? `${(currentMesh.positions.length / 3) | 0} verts — ready for package generation`
              : 'Drop a STEP in the viewport first'}
          </div>

          <div style={{ borderTop: '1px solid #444', paddingTop: 10 }}>
            <label style={{ fontSize: 12, display: 'block' }}>Scale (mm → m)</label>
            <input type="number" step="0.0001" value={pkgScale} onChange={e => setPkgScale(parseFloat(e.target.value) || 0.001)} style={{ width: '100%', marginBottom: 6 }} />

            <label style={{ fontSize: 12, display: 'block' }}>Density (kg/m³)</label>
            <input type="number" value={pkgDensity} onChange={e => setPkgDensity(parseFloat(e.target.value) || 7800)} style={{ width: '100%', marginBottom: 6 }} />

            <label style={{ fontSize: 12, display: 'block' }}>Base link name</label>
            <input type="text" value={baseLinkName} onChange={e => setBaseLinkName(e.target.value || 'base_link')} style={{ width: '100%', marginBottom: 10 }} />

            <button
              onClick={generatePackage}
              disabled={!currentHandle || isGenerating}
              style={{ width: '100%', padding: '8px 12px', marginBottom: 10, fontWeight: 600 }}
            >
              {isGenerating ? 'Generating...' : 'Generate URDF + STL Package'}
            </button>

            {packageResult && (
              <div style={{ fontSize: 13 }}>
                <div style={{ marginBottom: 8, lineHeight: 1.3 }}>
                  <strong>Package:</strong> {packageResult.package_name}<br />
                  <strong>Approx mass:</strong> {packageResult.mass.toFixed(3)} kg @ scale {packageResult.scale}
                </div>

                <div style={{ display: 'flex', flexDirection: 'column', gap: 4, marginBottom: 8 }}>
                  <button onClick={() => downloadText(packageResult.urdf, `${packageResult.base_link_name}.urdf`)}>Download .urdf</button>
                  <button onClick={() => downloadBinary(packageResult.visual_stl, `${packageResult.base_link_name}_visual.stl`)}>Download visual.stl</button>
                  <button onClick={() => downloadBinary(packageResult.collision_stl, `${packageResult.base_link_name}_collision.stl`)}>Download collision.stl</button>
                  <button onClick={downloadFullZip} style={{ fontWeight: 700, background: '#334155', color: 'white' }}>Download full .zip package</button>
                </div>

                <details>
                  <summary style={{ cursor: 'pointer', fontSize: 12, userSelect: 'none' }}>Preview URDF</summary>
                  <pre style={{ fontSize: 10, background: '#111', padding: 6, maxHeight: 180, overflow: 'auto', whiteSpace: 'pre-wrap', marginTop: 4 }}>
                    {packageResult.urdf}
                  </pre>
                </details>
              </div>
            )}
          </div>
        </aside>
      </div>

      <footer className="footer">
        <div>
          CLI: <code>cargo run -p cad2robot -- convert "your.step"</code>
        </div>
        <div className="compat">
          The web UI now lets you preview the tessellated result and download a complete URDF package (with &lt;visual&gt; + &lt;collision&gt; meshes) directly in the browser.
        </div>
      </footer>
    </div>
  )
}
