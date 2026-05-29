# Komatsu Bucket Pipeline — Implementation Plan (v4)

*Field-robust surface-area measurement for excavator buckets via iPhone
LiDAR + RGB. Designed for real yards (multiple buckets, mixed colors,
outdoor lighting, reflective ground).*

---

## Goal
Measure the surface area of a SPECIFIC, USER-SELECTED excavator
bucket (บุ้งกี๋แมคโคร) using iPhone LiDAR point cloud (.ply) + RGB
image, robust to:
  - Multiple buckets visible in frame (a yard / equipment lot)
  - Mixed-color bucket parts (yellow paint + red wear teeth + dark
    steel mounting bracket)
  - Outdoor lighting (sun, shadow, specular highlights)
  - Reflective ground, gravel, sky, foliage in background

---

## Priority: Must-Do + High (implement these first)

### Must-Do 1 — Coordinate Y-fix in `ingestScanFrame`
**File:** `Komatsu/LiDARManager.swift` lines 552–554

```swift
// Replace current
let xc = (Float(u) - cx) * d / fx
let yc = (Float(v) - cy) * d / fy
let world4 = camToWorld * SIMD4<Float>(xc, yc, -d, 1)

// With corrected ARKit convention
let xc =  (Float(u) - cx) / fx * d
let yc = -(Float(v) - cy) / fy * d
let zc = -d
let world4 = camToWorld * SIMD4<Float>(xc, yc, zc, 1)
```

> Verify against one real scan before committing — if existing scans look geometrically correct, a compensating flip may already exist elsewhere.

---

### Must-Do 2 — `ScanFilter` struct + depth slider
**New file:** `Komatsu/ScanFilter.swift`

```swift
struct ScanFilter {
    var minDepth: Float = 0.3
    var maxDepth: Float = 3.0
    var requireYellow: Bool = true
    var minConfidence: ARConfidenceLevel = .medium

    func passes(depth: Float, conf: UInt8, isYellow: Bool) -> Bool {
        guard depth >= minDepth, depth <= maxDepth else { return false }
        guard conf >= UInt8(minConfidence.rawValue) else { return false }
        if requireYellow { guard isYellow else { return false } }
        return true
    }
}
```

Replace the 3 inline checks in `ingestScanFrame` (lines 535–550) with `scanFilter.passes(...)`.

**ContentView change:** add `Slider(value: $mgr.scanFilter.maxDepth, in: 0.3...3.0)` to the controls section — lets user cut background beyond the bucket at scan time.

---

### High Priority — `YellowMaskProcessor` (HSV color filter)
**New file:** `Komatsu/YellowMaskProcessor.swift`

```swift
static func createYellowMask(frame: ARFrame, depthW: Int, depthH: Int) -> CVPixelBuffer?
// 1. Lock capturedImage YCbCr CVPixelBuffer
// 2. Per-pixel: convert to approximate HSV
// 3. Threshold: H=18–35, S=80–255, V=80–255
// 4. vImage scale mask from 1920×1440 → depthW×depthH (256×192)
// 5. Return 8-bit single-channel CVPixelBuffer
```

Wire result into `ScanFilter.passes(isYellow:)` inside `ingestScanFrame`.

> Note: hardcoded thresholds may need tuning for different lighting / mud conditions.

---

## Key Insight (from v3)

Color-based filtering is the WRONG primary signal in real yards.
A bucket photo usually contains:
  - Other buckets of the same color
  - Non-yellow parts that ARE part of the bucket (red teeth, black bracket)
  - Background that reflects bucket color

The PRIMARY signal must be **user-selected instance segmentation**
from the iOS side. Color/depth/RANSAC become refinement on the
chosen instance, not the gate.

---

## Pipeline (8 stages)

Stage 0 — Multi-view fusion (optional)
  ICP-merge multiple .ply scans for full bucket coverage.
  Single-view supported but tagged confidence ≤ medium.

Stage 1 — Instance Mask Acquisition  (PRIMARY)
  iOS exports for each scan:
    - scan.ply           — point cloud (LiDAR + tap-segmented points)
    - mask.png           — Vision foreground instance mask for the
                           tapped bucket only (not all yellow regions)
    - meta.json          — { intrinsics, tap_location, model_code,
                             stamped_size_in, ocr_text, timestamp }
  Python LOADS these. No HSV unless iOS mask is missing.
  Fallback (no iOS mask): HSV + LAB + GrabCut + largest blob.

Stage 2 — Ground-plane Removal  (NEW)
  In a yard, the largest planar surface is the GROUND. RANSAC will
  fit it first and ruin everything.
  • Detect largest plane whose normal aligns with gravity (±15° of
    world up). Use IMU gravity from ARKit if available, else assume
    [0, 1, 0] in ARKit frame.
  • REMOVE these inlier points before any bucket processing.

Stage 3 — Depth + Mask Refinement
  • Apply depth threshold (auto valley / p30 fallback) to constrain
    to foreground.
  • Project iOS mask onto remaining cloud (Y-flipped to OpenCV).
  • If iOS mask missing: use HSV/LAB mask + keep_largest_blob.

Stage 4 — Specular & Shadow Recovery  (NEW)
  Real bucket paint has bright specular spots and deep shadows that
  fail naive color thresholds.
  • Specular: pixels with V>240 AND S<30 SURROUNDED by yellow → relabel as yellow.
  • Shadow: pixels with H≈yellow AND S>80 AND V<80 → relabel as yellow.
  Implement as a post-process on the HSV mask only (skip if using iOS mask).

Stage 5 — Multi-color Part Inclusion  (NEW)
  A bucket = yellow paint + red wear teeth + dark steel mounting.
  If using HSV path, OR three masks:
    - yellow_mask  (paint)
    - red_mask     (H∈[0,10]∪[170,179], S>100, V>80 — wear teeth)
    - dark_steel_mask (V<60, S<60 — bracket; only if adjacent to yellow blob)
  Union, then keep_largest_blob.
  If using iOS mask, this stage is skipped (Vision already includes all parts).

Stage 6 — Multi-plane RANSAC
  After ground is gone, iteratively segment up to 5 planes on what
  remains; keep planes with inlier_ratio > 5%. Bucket has 3–5
  surfaces (back wall, side walls, floor, cutting edge).

Stage 7 — 3D Instance Separation  (NEW)
  DBSCAN on remaining points. Among clusters:
    - Reject any whose bbox exceeds 1.3 × max known bucket size
      for the declared model (likely merged with neighbor).
    - Reject any whose centroid is far from the tap_location ray
      (back-project tap_location through intrinsics → 3D ray →
      pick cluster closest to that ray).
  Keep ONE cluster — the target bucket.

Stage 8 — Reconstruction + Scale Validation
  • BPA (open-surface) by default. Poisson only when watertight check passes.
  • Validate scale against:
      1. meta.json.stamped_size_in (from OCR — absolute truth)
      2. Komatsu/Caterpillar/Hitachi/Volvo bucket spec table
      3. person_height_m (if person in frame)
  • Confidence:
      - HIGH:   OCR scale + multi-view + watertight Poisson
      - MEDIUM: spec-table scale OR single-view BPA with > 1000 pts
      - LOW:    no scale match OR < 1000 pts OR bbox-rejected cluster

---

## iOS-side changes required

  1. `BucketDetector.swift` — keep `VNGenerateForegroundInstanceMaskRequest`.
     Add `exportMaskPNG(to:URL)` that writes the user-tapped instance mask
     ONLY (not all foreground).
  2. `LiDARManager.swift` — when `exportPLY()` is called, also:
       - write `mask.png` (from BucketDetector)
       - write `meta.json` with intrinsics + tap point + OCR text
  3. `BucketColorClassifier.swift` — already exists; surface its
     output color cluster for visualization only.
  4. New: `BucketTextOCR.swift` — `VNRecognizeTextRequest` on the
     captured frame; parse model code (PC###) and stamped size
     (e.g. `54"`, `60"`). Save into meta.json.

## Komatsu / 3rd-party bucket spec table (expand)

KOMATSU_BUCKET_WIDTH_M = {
    "PC30":  0.55,   "PC55":  0.70,   "PC78":  0.78,
    "PC200": 1.00,   "PC220": 1.37,   "PC360": 1.40,
}
CAT_BUCKET_WIDTH_M = {
    "320": 1.05,  "330": 1.20,  "336": 1.40,  "349": 1.55,
}
HITACHI_BUCKET_WIDTH_M = {
    "ZX200": 1.00,  "ZX350": 1.40,
}
VOLVO_BUCKET_WIDTH_M = {
    "EC220": 1.30,  "EC380": 1.55,
}

## Input  (Python)
- ply_path:        str | list[str]
- image_path:      str
- intrinsics:      dict {fx, fy, cx, cy, distortion?}
- ios_mask_path:   str | None              # PRIMARY signal
- meta_path:       str | None              # JSON from iOS
- person_height_m: float | None
- bucket_model:    str | None              # overrides meta_path
- auto_calibrate:  bool                    # HSV calibration UI
- visualize:       bool

## Output
- area_m2, area_cm2, confidence
- bbox_width_m, bbox_height_m, bbox_depth_m
- recon_method (bpa/poisson), is_watertight
- scale_source (ocr / spec-table / person / none)
- per-stage point counts
- mesh (.ply export)

---

## New Functions to add to bucket_pipeline.py

### load_ios_artifacts(ply_path, mask_path, meta_path) -> dict
  Load .ply + mask.png + meta.json. Validate intrinsics consistency.
  Returns dict with pcd, mask, intrinsics, tap_location, ocr_size_m.

### remove_ground_plane(pcd, gravity, angle_tol_deg=15) -> pcd
  Find largest plane with normal aligned to gravity vector. Remove
  its inliers. Return cloud without ground.

### recover_specular_and_shadow(image, mask) -> mask
  Post-process HSV mask: relabel saturated highlights and deep
  shadows that are surrounded by yellow.

### build_multicolor_bucket_mask(image) -> mask
  Union yellow + red + dark-steel masks. For HSV fallback path only.

### back_project_tap_to_3d(tap_xy, intrinsics, pcd) -> ray
  Convert iOS tap pixel into a 3D ray in cloud frame. Used to pick
  the right cluster among many.

### select_cluster_near_ray(pcd, ray, max_size_m=2.0) -> pcd
  DBSCAN, then pick the cluster whose centroid is closest to the
  back-projected tap ray AND whose bbox ≤ max_size_m on each axis.

### lookup_bucket_spec_width(model_code) -> float | None
  Search KOMATSU / CAT / HITACHI / VOLVO tables. Return width_m
  or None.

### parse_ocr_size(ocr_text) -> float | None
  Parse strings like "54\"", "60 in", "1.37m" → meters.

### validate_scale (updated):
  Priority order:
    1. ocr_size_m (if present)
    2. lookup_bucket_spec_width(model_code)
    3. person_height_m
    4. none → low confidence

### run_pipeline (updated steps):
  Stage 0 — load .ply(s); ICP if multi-view
  Stage 1 — load iOS mask + meta OR fallback HSV
  Stage 2 — remove ground plane
  Stage 3 — depth threshold + mask projection
  Stage 4 — specular/shadow recovery (HSV path only)
  Stage 5 — multi-color part union (HSV path only)
  Stage 6 — multi-plane RANSAC
  Stage 7 — instance separation (tap-ray + bbox)
  Stage 8 — reconstruction + scale validation
  Stage 9 — 7-panel viz: image|mask|ground|depth|final-pcd|mesh|counts

---

## Hard Rules (updated)

1. PREFER iOS instance mask over HSV. Never re-segment if iOS provided one.
2. ALWAYS remove ground plane before bucket RANSAC.
3. ALWAYS use tap-location to disambiguate when multiple clusters survive.
4. ALWAYS apply ARKit Y-flip in projection unless .ply is in OpenCV frame.
5. NEVER use Poisson for area on open single-view scans — BPA only.
6. ALWAYS log per-stage point counts; flag a stage that drops > 90%.
7. If OCR size is present, it OVERRIDES spec-table scale.
8. Reject any cluster whose bbox > 1.3× spec-table max for the model.

## Constraints
- Python 3.11+
- Libraries: open3d, opencv-python, numpy, scipy, matplotlib
- Optional: pytesseract (only if iOS OCR is unavailable)
- Type hints + Thai/English docstrings, no global state

## Example
if __name__ == "__main__":
    result = run_pipeline(
        ply_path="scan.ply",
        image_path="bucket_photo.jpg",
        intrinsics={"fx": 1488, "fy": 1488, "cx": 960, "cy": 720},
        ios_mask_path="mask.png",        # iOS Vision output
        meta_path="meta.json",           # tap_location + OCR
        person_height_m=None,
        bucket_model=None,               # filled from meta.json
        visualize=True,
    )
    print(f"พื้นที่ผิวบุ้งกี๋: {result['area_m2']:.4f} m² "
          f"[{result['confidence']}, scale={result['scale_source']}]")
