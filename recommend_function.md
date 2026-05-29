# Recommended Function Changes & Additions

Based on reading `LiDARManager.swift`, `ContentView.swift`, and `DepthMaskProcessor.swift`.

---

## 1. `LiDARManager.startSession()` — MODIFY

**File:** `Komatsu/LiDARManager.swift` lines 90–109

Changes:
- Remove `config.environmentTexturing = .automatic`
- Add `.sceneDepth` alongside `.smoothedSceneDepth`
- Add calibration reference tracking

New properties to add to the class:
```swift
private var referencePlaneAnchor: ARPlaneAnchor?
private var referenceTransform: simd_float4x4?
@Published var calibrationDrift: Float = 0       // meters
@Published var isCalibrated: Bool = false
private let driftThreshold: Float = 0.02         // 2 cm
```

New methods to add:
- `setCalibrationReference(_ anchor: ARPlaneAnchor)` — store first detected plane as reference pose
- `updateCalibrationDrift(currentCamera: ARCamera)` — compute `simd_distance` vs. reference, called from `session(_:didUpdate:)` (cheap, non-blocking)

---

## 2. `DepthMaskProcessor.swift` — CONFLICT / DECISION NEEDED

**Existing file:** Vision-based `VNGenerateForegroundInstanceMaskRequest` (iOS 17+), used for instance segmentation bounding box.

**Spec asks for:** HSV color-threshold yellow mask via vImage, for per-pixel filtering inside the depth unprojection loop.

**These serve different purposes** — do NOT replace the existing one.

Recommended: create **new file** `Komatsu/YellowMaskProcessor.swift` with:
```swift
static func createYellowMask(frame: ARFrame, depthW: Int, depthH: Int) -> CVPixelBuffer?
// Steps:
//   1. Lock capturedImage CVPixelBuffer (YCbCr)
//   2. Convert each pixel to approximate HSV
//   3. Threshold: H=18–35, S=80–255, V=80–255
//   4. Use vImage to scale 1920×1440 mask → depthW×depthH (256×192)
//   5. Return 8-bit single-channel CVPixelBuffer
```

---

## 3. NEW FILE: `Komatsu/ScanFilter.swift` — ADD

Centralises the 3 inline checks that are currently scattered in `ingestScanFrame` (lines 535–550).

```swift
struct ScanFilter {
    var minDepth: Float = 0.3
    var maxDepth: Float = 3.0          // exposed to UI slider
    var requireYellow: Bool = true
    var minConfidence: ARConfidenceLevel = .medium

    func passes(depth: Float, conf: UInt8, isYellow: Bool) -> Bool {
        guard depth >= minDepth, depth <= maxDepth else { return false }
        if let minConf = ARConfidenceLevel(rawValue: Int(minConfidence.rawValue)) {
            guard conf >= UInt8(minConf.rawValue) else { return false }
        }
        if requireYellow { guard isYellow else { return false } }
        return true
    }
}
```

Add `var scanFilter = ScanFilter()` to `LiDARManager`.

---

## 4. `LiDARManager.ingestScanFrame()` — MODIFY

**File:** `Komatsu/LiDARManager.swift` lines 432–585

### 4a. Coordinate sign fix (lines 552–554)

```swift
// CURRENT (potential Y-flip bug)
let xc = (Float(u) - cx) * d / fx
let yc = (Float(v) - cy) * d / fy
let world4 = camToWorld * SIMD4<Float>(xc, yc, -d, 1)

// CORRECTED (ARKit convention: Y down in camera, negate for world up)
let xc =  (Float(u) - cx) / fx * d
let yc = -(Float(v) - cy) / fy * d
let zc = -d
let world4 = camToWorld * SIMD4<Float>(xc, yc, zc, 1)
```

> Warning: if your existing captures look geometrically correct, verify the Y-flip against a real scan before committing — it will mirror the cloud vertically if already correct.

### 4b. Add bilinear depth sampling — new private helper

```swift
private func sampleDepthBilinear(_ ptr: UnsafePointer<Float32>,
                                  rowStride: Int, w: Int, h: Int,
                                  u: Float, v: Float) -> Float {
    let u0 = max(0, min(w - 2, Int(u))); let u1 = u0 + 1
    let v0 = max(0, min(h - 2, Int(v))); let v1 = v0 + 1
    let fu = u - Float(u0); let fv = v - Float(v0)
    let d00 = ptr[v0 * rowStride + u0]; let d10 = ptr[v0 * rowStride + u1]
    let d01 = ptr[v1 * rowStride + u0]; let d11 = ptr[v1 * rowStride + u1]
    return d00*(1-fu)*(1-fv) + d10*fu*(1-fv) + d01*(1-fu)*fv + d11*fu*fv
}
```

Replace nearest-neighbour `depthPtr[v * depthRow + u]` lookup with this.

### 4c. Wire in ScanFilter + YellowMaskProcessor

Replace the three inline checks (lines 535–550) with `scanFilter.passes(depth:conf:isYellow:)`.

---

## 5. `LiDARManager.exportPLY()` — MODIFY

**File:** `Komatsu/LiDARManager.swift` lines 354–402

| What | Where | Change |
|---|---|---|
| `voxelSize` | line 58 | `0.005` → `0.003` |
| PLY header | line 371 | Add `property float nx\nproperty float ny\nproperty float nz\n` |
| Point packing | lines 376–389 | Write 6 floats per point (x y z nx ny nz); use zeros for normals (Python pipeline recomputes them via `estimate_normals`) |

> Note: `bucket_pipeline.py` already calls `pcd.estimate_normals()`, so zero normals in the file are safe. They signal to downstream tools that normals are present but uncomputed on-device.

---

## 6. `ContentView.swift` — MODIFY

**File:** `Komatsu/ContentView.swift`

| What | Where | Change |
|---|---|---|
| Remove mesh overlay | line 371 | Delete `arView.debugOptions = [.showSceneUnderstanding]` |
| Depth range slider | `controls` var (~line 271) | Add `Slider(value: $mgr.scanFilter.maxDepth, in: 0.3...3.0)` with label showing value in meters |
| Calibration indicator | `header` var (~line 153) | Add green/red dot: `Circle().fill(mgr.calibrationDrift < 0.02 ? .green : .red)` + text "สอบเทียบ OK / ดริฟท์" |

---

## Summary Table

| File | Action | Functions / Properties |
|---|---|---|
| `LiDARManager.swift` | Modify | `startSession`, `session(_:didUpdate:)`, `ingestScanFrame`, `exportPLY`; add `setCalibrationReference`, `updateCalibrationDrift`, `sampleDepthBilinear`, `scanFilter: ScanFilter` |
| `DepthMaskProcessor.swift` | Keep as-is | No changes — Vision instance mask stays for detection bbox |
| `YellowMaskProcessor.swift` | Create new | `createYellowMask(frame:depthW:depthH:)` |
| `ScanFilter.swift` | Create new | `passes(depth:conf:isYellow:)` |
| `ContentView.swift` | Modify | `ARMeasureView.makeUIView` (remove debugOptions), `controls` (add slider), `header` (add calibration dot) |

---

## Open Question

> Do you want to **keep both** mask systems (Vision instance mask in `DepthMaskProcessor` + HSV yellow mask in new `YellowMaskProcessor`), or replace the existing one?
>
> Recommendation: **keep both** — they do different jobs. The Vision mask drives the green bounding box overlay. The HSV mask filters depth pixels during scan ingestion.
