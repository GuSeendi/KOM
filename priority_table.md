# Implementation Priority Table

| # | Priority | File | Function / Change | Why | Status |
|---|---|---|---|---|---|
| 1 | 🔴 Must-Do | `LiDARManager.swift` | Coordinate Y-sign fix in `ingestScanFrame` | Silent geometry bug — point cloud mirrors vertically without it | ✅ Done |
| 2 | 🔴 Must-Do | `ScanFilter.swift` (new) | `ScanFilter.passes(depth:conf:isYellow:)` | Centralises 3 inline checks; required by depth slider | ✅ Done |
| 3 | 🔴 Must-Do | `ContentView.swift` | Depth range slider (0.3–3.0 m) | Lets user cut background at scan time — core scan quality control | ✅ Done |
| 4 | 🔴 Must-Do | `ContentView.swift` | Remove `.showSceneUnderstanding` debug overlay | Ships in production UI — should not be visible to end user | ✅ Done |
| 5 | 🟠 High | `YellowMaskProcessor.swift` (new) | `createYellowMask(frame:depthW:depthH:)` | Pre-built HSV mask per frame; biggest noise reduction for Komatsu yellow | ✅ Done |
| 6 | 🟠 High | `LiDARManager.swift` | Wire `YellowMaskProcessor` into `ingestScanFrame` | Replace slower per-pixel classifier call with mask lookup | ✅ Done |
| 7 | 🟠 High | `bucket_pipeline.py` | `refine_with_ransac_multi()` — 3 cm tolerance + DBSCAN fallback | Bucket is curved — exact-inlier-only keep loses valid surface points | ✅ Done |
| 8 | 🟠 High | `bucket_pipeline.py` | `reconstruct_surface()` — `orient_normals_towards_camera_location` | Prevents inward-facing normals that silently corrupt Poisson output | ✅ Done |
| 9 | 🟡 Medium | `bucket_pipeline.py` | `apply_taubin_filter()` (new) + wired into `run_pipeline` | Geometric noise removal before reconstruction; volume-preserving smoothing | ✅ Done |
| 10 | 🟡 Medium | `bucket_pipeline.py` | `reconstruct_surface()` — density threshold 5th → 10th percentile | Removes more low-density mesh boundary artefacts | ✅ Done |
| 11 | 🟡 Medium | `bucket_pipeline.py` | `validate_with_reference()` (new) + `ground_truth_area` param | MAE / MAPE validation against known ground truth; Bondar (2026) method | ✅ Done |
| 12 | 🔵 Low | `LiDARManager.swift` | Calibration mode (`setCalibrationReference`, drift indicator) | Nice safety net for field use — not blocking core measurement | ❌ Not started |
| 13 | 🔵 Low | `ContentView.swift` | Calibration green/red dot in header | UI indicator for task #12 — depends on it | ❌ Not started |
| 14 | 🔵 Low | `LiDARManager.swift` | `sampleDepthBilinear()` — bilinear depth interpolation | Small accuracy gain; risk of blending across depth edges | ❌ Not started |
| 15 | 🔵 Low | `LiDARManager.swift` | `exportPLY()` — add zero normal vectors to PLY header | PLY compatibility with MeshLab/CloudCompare; Python pipeline recomputes anyway | ❌ Not started |
| 16 | 🔵 Low | `LiDARManager.swift` | `voxelSize` 0.005 → 0.003 | More surface detail in PLY; larger file size | ❌ Not started |

---

## Summary

| Priority | Count | Done |
|---|---|---|
| 🔴 Must-Do | 4 | 4 / 4 |
| 🟠 High | 5 | 5 / 5 |
| 🟡 Medium | 3 | 3 / 3 |
| 🔵 Low | 4 | 0 / 4 |
| **Total** | **16** | **12 / 16** |
