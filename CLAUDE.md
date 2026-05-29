# Skill: Komatsu Bucket Measurement — iOS ARKit + LiDAR

## Goal

Build a SwiftUI iOS app that detects a Komatsu excavator bucket via Vision, measures its surface area from ARKit LiDAR mesh triangles, measures length via raycasting, and computes volume (area × length). Captured point clouds are exported as `.ply` for offline analysis with the Python pipeline.

## Environment Constraints

- **Frameworks:** SwiftUI, ARKit, RealityKit, Vision, simd, Combine. UIKit only via `UIViewRepresentable`.
- **Hardware/OS:** iPhone 12 Pro+ (LiDAR required), iOS 16+ (iOS 17+ for `VNGenerateForegroundInstanceMaskRequest`).
- **Files:**
  - `LiDARManager.swift` — ARSessionDelegate, detection pipeline, length raycasting, volume
  - `BucketDetector.swift` — Vision foreground detection (iOS 17+) + saliency fallback
  - `MeshAnalyzer.swift` — surface area from `ARMeshAnchor` triangles
  - `ContentView.swift` — SwiftUI UI + `ARMeasureView` (UIViewRepresentable)
- **Python pipeline:** `scripts/bucket_pipeline.py` — offline `.ply` processing (open3d, numpy)
- **Units (internal):** meters; convert to cm²/cm/cm³ only at display time

## Hard Rules

1. **Always** enable `sceneReconstruction = .mesh` and `frameSemantics = .smoothedSceneDepth` in `ARWorldTrackingConfiguration`.
2. **Never** run `VNImageRequestHandler` on the main thread or on every AR frame — throttle with a `Timer` (0.5 s interval) or frame counter.
3. **Always** run Vision requests on `DispatchQueue.global(qos: .userInitiated)`; publish results back via `DispatchQueue.main.async`.
4. **Only** sum mesh triangles whose screen-projected centroid falls inside the normalized detection `CGRect`.
5. **Never** block the AR session delegate (`session(_:didUpdate:)`) — keep it minimal; use `isReady` flag only.
6. **Always** lock/unlock `CVPixelBuffer` base addresses and wrap pixel loops in `autoreleasepool`.
7. Volume = `capturedArea × measuredLength` (both in meters); only compute when both are > 0.

## Minimal Patterns

```swift
// BucketDetector — iOS 17+ foreground detection
@available(iOS 17.0, *)
func detectWithInstanceMask(pixelBuffer: CVPixelBuffer, completion: @escaping (DetectionResult?) -> Void) {
    let request = VNGenerateForegroundInstanceMaskRequest()
    let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
    try? handler.perform([request])
    guard let result = request.results?.first else { completion(nil); return }
    let maskBuffer = try? result.generateScaledMaskForImage(forInstances: result.allInstances, from: handler)
    completion(DetectionResult(boundingBox: boundingBoxFromMask(maskBuffer!), mask: maskBuffer, confidence: 0.9))
}

// MeshAnalyzer — area from mesh triangles inside bbox
for faceIdx in 0..<geo.faces.count {
    let (i0, i1, i2) = faceIndices(geo: geo, at: faceIdx)
    let w1 = (anchorTransform * SIMD4(v(i0), 1)).xyz
    let w2 = (anchorTransform * SIMD4(v(i1), 1)).xyz
    let w3 = (anchorTransform * SIMD4(v(i2), 1)).xyz
    let screen = camera.projectPoint((w1+w2+w3)/3, orientation: .portrait, viewportSize: viewportSize)
    let norm = CGPoint(x: screen.x/viewportSize.width, y: screen.y/viewportSize.height)
    if detectionBox.contains(norm) { totalArea += 0.5 * length(cross(w2-w1, w3-w1)) }
}

// LiDARManager — length via raycast (2-tap)
let hits = arView.raycast(from: screenPoint, allowing: .estimatedPlane, alignment: .any)
let wp = SIMD3<Float>(hits[0].worldTransform.columns.3.xyz)
measuredLength = distance(startPoint, wp)
```

```python
# bucket_pipeline.py — offline .ply processing
pcd = o3d.io.read_point_cloud("scan.ply")
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcd = pcd.voxel_down_sample(voxel_size=0.003)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(k=15)
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
mesh.remove_vertices_by_mask(np.asarray(densities) < np.quantile(densities, 0.05))
area_cm2 = mesh.get_surface_area() * 1e4
```

## Common Mistakes

- **Resolution mismatch:** `capturedImage` is ~1920×1440; `smoothedSceneDepth` is ~256×192. Scale the Vision mask to depth resolution before pixel iteration.
- **Coordinate flip:** Vision results use bottom-left origin; ARKit screen uses top-left. Flip Y when mapping mask → depth coordinates.
- **No normals before Poisson:** `create_from_point_cloud_poisson` silently produces garbage if normals are missing or unoriented.
- **Volume on open mesh:** `mesh.get_volume()` is only valid when `mesh.is_watertight()` — always check first.
- **Forgetting `autoreleasepool`:** Iterating millions of mesh vertices without it causes memory spikes and OOM crashes on device.
