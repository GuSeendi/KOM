#!/usr/bin/env python3
"""
Komatsu Bucket Surface Area Pipeline (v3 — yard-robust)
========================================================

Stage 0 — Multi-view ICP fusion (optional)
Stage 1 — Instance mask acquisition (iOS PRIMARY, HSV fallback)
Stage 2 — Ground-plane removal
Stage 3 — Depth threshold + mask projection
Stage 4 — Specular/shadow recovery (HSV fallback path only)
Stage 5 — Multi-color part union (HSV fallback path only)
Stage 6 — Multi-plane RANSAC
Stage 7 — 3D instance separation (tap-ray + bbox constraint)
Stage 8 — Reconstruction (BPA / Poisson) + scale validation

Dependencies: open3d, opencv-python, numpy, scipy, matplotlib
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Optional, Union

import numpy as np

try:
    import cv2
    import open3d as o3d
    from scipy.signal import find_peaks
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"❌ Missing dependency: {e.name}")
    print("   pip install open3d opencv-python numpy scipy matplotlib")
    sys.exit(1)

KOMATSU_BUCKET_WIDTH_M = {
    "PC30":  0.55, "PC55":  0.70, "PC78":  0.78,
    "PC200": 1.00, "PC220": 1.37, "PC360": 1.40,
}
CAT_BUCKET_WIDTH_M = {
    "320": 1.05, "330": 1.20, "336": 1.40, "349": 1.55,
}
HITACHI_BUCKET_WIDTH_M = {
    "ZX200": 1.00, "ZX350": 1.40,
}
VOLVO_BUCKET_WIDTH_M = {
    "EC220": 1.30, "EC380": 1.55,
}

def lookup_bucket_spec_width(model_code: str) -> Optional[float]:
    """ค้นหา width จาก spec table หลายยี่ห้อ"""
    if not model_code:
        return None
    code = model_code.upper().strip()
    for table in (KOMATSU_BUCKET_WIDTH_M, CAT_BUCKET_WIDTH_M,
                  HITACHI_BUCKET_WIDTH_M, VOLVO_BUCKET_WIDTH_M):
        if code in table:
            return table[code]
    return None

def parse_ocr_size(ocr_text: str) -> Optional[float]:
    """
    Parse "54\"", "60 in", "1.37m", "1370mm" → meters
    """
    if not ocr_text:
        return None
    s = ocr_text.lower().replace(",", "")

    m = re.search(r"(\d+(?:\.\d+)?)\s*m(?!\w)", s)
    if m:
        return float(m.group(1))

    m = re.search(r"(\d+(?:\.\d+)?)\s*mm", s)
    if m:
        return float(m.group(1)) / 1000.0

    m = re.search(r"(\d+(?:\.\d+)?)\s*(?:\"|in\b|inch(?:es)?)", s)
    if m:
        return float(m.group(1)) * 0.0254

    m = re.search(r"(\d+(?:\.\d+)?)\s*(?:'|ft\b|feet)", s)
    if m:
        return float(m.group(1)) * 0.3048
    return None

def load_ios_artifacts(
    ply_path: str,
    mask_path: Optional[str],
    meta_path: Optional[str],
) -> dict:
    """
    โหลด .ply + mask.png + meta.json จาก iOS

    meta.json schema (เขียนโดย LiDARManager.exportPLY):
        {
          "intrinsics": {"fx":..., "fy":..., "cx":..., "cy":...},
          "tap_location": [x_px, y_px],
          "image_size": [width, height],
          "model_code": "PC220",
          "stamped_size": "54\"",
          "ocr_text": "PC220 54\"",
          "gravity": [0.0, 1.0, 0.0],
          "timestamp": "2026-05-21T..."
        }
    """
    pcd = o3d.io.read_point_cloud(ply_path)
    out: dict = {"pcd": pcd, "mask": None, "meta": None,
                 "intrinsics": None, "tap_location": None,
                 "ocr_size_m": None, "model_code": None,
                 "gravity": None}

    if mask_path and Path(mask_path).exists():
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if mask is not None:
            _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
            out["mask"] = mask
            print(f"  [iOS] mask loaded: {mask.shape}")

    if meta_path and Path(meta_path).exists():
        meta = json.loads(Path(meta_path).read_text())
        out["meta"] = meta
        out["intrinsics"] = meta.get("intrinsics")
        out["tap_location"] = meta.get("tap_location")
        out["model_code"] = meta.get("model_code")
        out["gravity"] = np.array(meta.get("gravity", [0, 1, 0]), dtype=float)
        ocr = meta.get("ocr_text") or meta.get("stamped_size")
        if ocr:
            out["ocr_size_m"] = parse_ocr_size(ocr)
            if out["ocr_size_m"]:
                print(f"  [iOS] OCR size: '{ocr}' → {out['ocr_size_m']:.3f}m")
        print(f"  [iOS] meta loaded: model={out['model_code']}, "
              f"tap={out['tap_location']}")

    return out

def filter_by_yellow_hsv(image, lower=(18, 80, 80), upper=(35, 255, 255), morph_kernel=7):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    print(f"  [HSV] yellow: {100.0 * np.count_nonzero(mask) / mask.size:.2f}%")
    return mask

def filter_by_yellow_lab(image, a_min=15, b_min=25):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    a = lab[:, :, 1].astype(np.int16) - 128
    b = lab[:, :, 2].astype(np.int16) - 128
    mask = ((a > a_min) & (b > b_min)).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    print(f"  [LAB] yellow: {100.0 * np.count_nonzero(mask) / mask.size:.2f}%")
    return mask

def filter_by_red(image):
    """แดง (cutting teeth + wear plates)"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0, 100, 80]), np.array([10, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([170, 100, 80]), np.array([179, 255, 255]))
    mask = cv2.bitwise_or(m1, m2)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

def filter_dark_steel(image):
    """เหล็กดำ/เทาเข้ม ของ bracket"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array([0, 0, 0]), np.array([179, 60, 60]))

def keep_largest_blob(mask):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return mask
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest = 1 + int(np.argmax(areas))
    out = np.where(labels == largest, 255, 0).astype(np.uint8)
    print(f"  [Blob] kept largest of {n-1} ({areas.max():,}px)")
    return out

def recover_specular_and_shadow(image: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """
    คืน pixel ที่หายไปเพราะ specular (V≈255 S≈0) หรือ shadow (V<80)
    เฉพาะที่ติดอยู่กับบริเวณ mask
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    specular = (V > 240) & (S < 30)
    shadow = (((H >= 15) & (H <= 38)) & (S > 80) & (V < 80))

    near = cv2.dilate(mask, np.ones((15, 15), np.uint8))
    recover = ((specular | shadow) & (near > 0)).astype(np.uint8) * 255

    combined = cv2.bitwise_or(mask, recover)
    delta = np.count_nonzero(combined) - np.count_nonzero(mask)
    print(f"  [Recovery] +{delta:,}px (specular+shadow)")
    return combined

def build_multicolor_bucket_mask(image: np.ndarray) -> np.ndarray:
    """
    yellow paint ∪ red teeth ∪ dark steel (เฉพาะที่ติดกับ yellow)
    """
    yellow = filter_by_yellow_hsv(image)
    yellow = recover_specular_and_shadow(image, yellow)

    red = filter_by_red(image)
    dark = filter_dark_steel(image)

    yellow_near = cv2.dilate(yellow, np.ones((25, 25), np.uint8))
    dark_adj = cv2.bitwise_and(dark, yellow_near)

    combined = cv2.bitwise_or(yellow, red)
    combined = cv2.bitwise_or(combined, dark_adj)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, k)

    print(f"  [Multi-color] combined: {100.0 * np.count_nonzero(combined) / combined.size:.2f}%")
    return combined

def remove_ground_plane(
    pcd: o3d.geometry.PointCloud,
    gravity: Optional[np.ndarray] = None,
    angle_tol_deg: float = 15.0,
    distance_threshold: float = 0.03,
    num_iterations: int = 1000,
) -> o3d.geometry.PointCloud:
    """
    หา plane ที่ normal ขนานกับ gravity แล้วลบจุด inlier ออก
    ป้องกัน RANSAC ในขั้นต่อไปจับพื้นแทนบุ้งกี๋
    """
    pts = np.asarray(pcd.points)
    if len(pts) < 100:
        return pcd

    if gravity is None:
        gravity = np.array([0.0, 1.0, 0.0])
    g = gravity / (np.linalg.norm(gravity) + 1e-9)

    try:
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=num_iterations,
        )
    except RuntimeError:
        print("  [Ground] segment_plane failed — skip")
        return pcd

    a, b, c, _ = plane_model
    n = np.array([a, b, c])
    n = n / (np.linalg.norm(n) + 1e-9)

    cos_th = abs(float(np.dot(n, g)))
    angle = np.degrees(np.arccos(np.clip(cos_th, 0, 1)))

    if angle > angle_tol_deg:
        print(f"  [Ground] largest plane is not horizontal "
              f"(angle={angle:.1f}° > {angle_tol_deg}°) — no ground removed")
        return pcd

    outlier_mask = np.ones(len(pts), dtype=bool)
    outlier_mask[inliers] = False
    without_ground = pcd.select_by_index(np.where(outlier_mask)[0].tolist())
    print(f"  [Ground] removed plane (angle={angle:.1f}°) "
          f"{len(pts):,} → {len(without_ground.points):,}")
    return without_ground

def cut_by_depth(pcd, method="auto"):
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return pcd, 0.0
    z = pts[:, 2]
    if method == "auto":
        hist, edges = np.histogram(z, bins=64)
        peaks, _ = find_peaks(-hist.astype(float), distance=3)
        threshold = float(edges[peaks[0] + 1]) if len(peaks) else float(np.percentile(z, 30))
    else:
        threshold = float(np.percentile(z, 30))
    idx = np.where(z < threshold)[0]
    out = pcd.select_by_index(idx.tolist())
    print(f"  [Depth] Z<{threshold:.3f}m  {len(pts):,} → {len(out.points):,}")
    return out, threshold

def undistort_image(image, intrinsics):
    d = intrinsics.get("distortion")
    if d is None:
        return image
    fx, fy, cx, cy = intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    return cv2.undistort(image, K, np.array(d, dtype=np.float64))

def apply_color_mask_to_pcd(pcd, mask, image_shape, intrinsics, flip_y=True):
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return pcd
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    img_h, img_w = image_shape[:2]
    mh, mw = mask.shape[:2]
    X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]
    if flip_y:
        Y = -Y
    valid = Z > 1e-4
    px = np.zeros_like(X); py = np.zeros_like(Y)
    px[valid] = fx * X[valid] / Z[valid] + cx
    py[valid] = fy * Y[valid] / Z[valid] + cy
    mx = (px * mw / img_w).astype(np.int32)
    my = (py * mh / img_h).astype(np.int32)
    in_b = (mx >= 0) & (mx < mw) & (my >= 0) & (my < mh) & valid
    keep = np.zeros(len(pts), dtype=bool)
    keep[in_b] = mask[my[in_b], mx[in_b]] > 0
    out = pcd.select_by_index(np.where(keep)[0].tolist())
    print(f"  [Mask] {len(pts):,} → {len(out.points):,}")
    return out

def refine_with_ransac_multi(pcd, max_planes=5, distance_threshold=0.02,
                              num_iterations=1000, min_inlier_ratio=0.05,
                              near_plane_dist=0.03, dbscan_fallback_ratio=0.30):
    """
    Multi-plane RANSAC keeping all points within near_plane_dist of each plane.
    Falls back to DBSCAN largest-cluster when initial inlier ratio < dbscan_fallback_ratio
    (bucket surface is curved — not all points lie on a perfect plane).
    """
    pts = np.asarray(pcd.points)
    n0 = len(pts)
    if n0 < 30:
        return pcd

    # Check initial inlier ratio to decide whether RANSAC makes sense
    try:
        _model, _inliers = pcd.segment_plane(
            distance_threshold=distance_threshold, ransac_n=3,
            num_iterations=num_iterations,
        )
        initial_ratio = len(_inliers) / n0
    except RuntimeError:
        initial_ratio = 0.0

    if initial_ratio < dbscan_fallback_ratio:
        print(f"  [RANSAC] initial inlier ratio {initial_ratio*100:.1f}% < "
              f"{dbscan_fallback_ratio*100:.0f}% — switching to DBSCAN (curved surface)")
        labels = np.array(pcd.cluster_dbscan(eps=0.03, min_points=10, print_progress=False))
        if labels.max() < 0:
            print("  [DBSCAN] no clusters found — return as-is")
            return pcd
        counts = np.bincount(labels[labels >= 0])
        largest = int(np.argmax(counts))
        result = pcd.select_by_index(np.where(labels == largest)[0].tolist())
        print(f"  [DBSCAN fallback] {n0:,} → {len(result.points):,} (largest cluster)")
        return result

    remaining = pcd
    parent_idx = np.arange(n0)
    near_mask = np.zeros(n0, dtype=bool)

    for i in range(max_planes):
        n_rem = len(remaining.points)
        if n_rem < 30:
            break
        plane_model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold, ransac_n=3,
            num_iterations=num_iterations,
        )
        ratio = len(inliers) / n_rem
        a, b, c, d = plane_model
        print(f"  [RANSAC #{i+1}] {a:+.2f}x {b:+.2f}y {c:+.2f}z {d:+.2f}=0  "
              f"inliers={len(inliers):,} ({ratio*100:.1f}%)")
        if ratio < min_inlier_ratio:
            break

        # Keep all points within near_plane_dist of this plane (not just exact inliers)
        rem_pts = np.asarray(remaining.points)
        norm_len = np.sqrt(a**2 + b**2 + c**2) + 1e-9
        dists = np.abs(rem_pts @ np.array([a, b, c]) + d) / norm_len
        near_idx = np.where(dists <= near_plane_dist)[0]
        near_mask[parent_idx[near_idx]] = True

        # Remove inliers from remaining for next plane search
        outlier_mask = np.ones(n_rem, dtype=bool)
        outlier_mask[inliers] = False
        remaining = remaining.select_by_index(np.where(outlier_mask)[0].tolist())
        parent_idx = parent_idx[outlier_mask]

    kept = np.where(near_mask)[0]
    if len(kept) == 0:
        return pcd
    result = pcd.select_by_index(kept.tolist())
    print(f"  [RANSAC merged] {n0:,} → {len(result.points):,} "
          f"(within {near_plane_dist*100:.0f}cm of planes)")
    return result

def apply_taubin_filter(
    mesh: "o3d.geometry.TriangleMesh",
    lambda_: float = 0.5,
    mu: float = -0.53,
    iterations: int = 10,
) -> "o3d.geometry.TriangleMesh":
    """
    Taubin smoothing — reduces geometric noise while preserving volume better
    than plain Laplacian. Ref: Günen et al. (2023).
    Insert AFTER RANSAC, BEFORE Poisson reconstruction.
    """
    smoothed = mesh.filter_smooth_taubin(
        number_of_iterations=iterations,
        lambda_filter=lambda_,
        mu=mu,
    )
    smoothed.remove_degenerate_triangles()
    smoothed.remove_duplicated_triangles()
    smoothed.remove_duplicated_vertices()
    smoothed.remove_non_manifold_edges()
    print(f"  [Taubin] λ={lambda_} μ={mu} iter={iterations} "
          f"→ {len(smoothed.vertices):,} verts / {len(smoothed.triangles):,} tris")
    return smoothed


def back_project_tap_to_3d(tap_xy, intrinsics, flip_y=True):
    """แปลง 2D tap → 3D ray (origin, direction) ใน cloud frame"""
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    u, v = tap_xy
    x = (u - cx) / fx
    y = (v - cy) / fy
    if flip_y:
        y = -y
    direction = np.array([x, y, 1.0])
    direction /= np.linalg.norm(direction)
    return np.zeros(3), direction

def _point_to_ray_distance(points, ray_origin, ray_dir):
    diff = points - ray_origin
    proj = diff @ ray_dir
    closest = ray_origin + np.outer(proj, ray_dir)
    return np.linalg.norm(points - closest, axis=1)

def select_cluster_near_ray(
    pcd: o3d.geometry.PointCloud,
    tap_xy: Optional[tuple],
    intrinsics: Optional[dict],
    max_bucket_size_m: float = 2.0,
    eps: float = 0.03,
    min_points: int = 30,
) -> o3d.geometry.PointCloud:
    """
    DBSCAN → ตัดอันที่ใหญ่เกิน max_bucket_size_m → เลือกอันที่ใกล้ tap-ray ที่สุด
    """
    if len(pcd.points) < min_points:
        return pcd
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    if labels.max() < 0:
        print("  [Cluster] no clusters — return as-is")
        return pcd

    pts = np.asarray(pcd.points)
    n_clusters = labels.max() + 1
    candidates: list[tuple[int, np.ndarray]] = []

    for c in range(n_clusters):
        idx = np.where(labels == c)[0]
        if len(idx) < min_points:
            continue
        sub = pts[idx]
        ext = sub.max(axis=0) - sub.min(axis=0)
        if (ext > max_bucket_size_m).any():
            print(f"  [Cluster #{c}] rejected — bbox {ext.round(2)} > {max_bucket_size_m}m")
            continue
        candidates.append((c, idx))

    if not candidates:
        print("  [Cluster] all rejected by bbox — keep largest anyway")
        counts = np.bincount(labels[labels >= 0])
        largest = int(np.argmax(counts))
        return pcd.select_by_index(np.where(labels == largest)[0].tolist())

    if tap_xy and intrinsics:
        origin, direction = back_project_tap_to_3d(tap_xy, intrinsics)
        best_c, best_idx, best_d = -1, None, 1e9
        for c, idx in candidates:
            centroid = pts[idx].mean(axis=0)
            d = float(_point_to_ray_distance(centroid[None, :], origin, direction)[0])
            print(f"  [Cluster #{c}] centroid={centroid.round(2)}  ray-dist={d:.3f}m  pts={len(idx):,}")
            if d < best_d:
                best_d = d
                best_c, best_idx = c, idx
        print(f"  [Cluster] picked #{best_c} (ray-dist={best_d:.3f}m)")
        return pcd.select_by_index(best_idx.tolist())

    candidates.sort(key=lambda ci: -len(ci[1]))
    c, idx = candidates[0]
    print(f"  [Cluster] no tap — picked largest #{c} ({len(idx):,} pts)")
    return pcd.select_by_index(idx.tolist())

def merge_scans_icp(scan_paths, voxel_size=0.005, icp_threshold=0.02):
    clouds = []
    for p in scan_paths:
        pcd = o3d.io.read_point_cloud(str(p))
        if len(pcd.points) == 0:
            continue
        pcd = pcd.voxel_down_sample(voxel_size)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=30))
        clouds.append(pcd)
        print(f"  loaded {p}: {len(pcd.points):,} pts")
    if not clouds:
        raise RuntimeError("no usable scan")
    if len(clouds) == 1:
        return clouds[0]
    merged = clouds[0]
    for i, src in enumerate(clouds[1:], start=2):
        reg = o3d.pipelines.registration.registration_icp(
            src, merged, icp_threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
        )
        print(f"  [ICP {i}/{len(clouds)}] fitness={reg.fitness:.3f} rmse={reg.inlier_rmse:.4f}m")
        src.transform(reg.transformation)
        merged = merged + src
    merged = merged.voxel_down_sample(voxel_size)
    return merged

def reconstruct_surface(pcd, method="auto", voxel_size=0.005):
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*4, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(k=15)

    # Orient normals to face outward from the cloud centroid before Poisson
    centroid = np.asarray(pcd.points).mean(axis=0)
    pcd.orient_normals_towards_camera_location(camera_location=centroid)

    def _bpa():
        radii = [voxel_size*2, voxel_size*4, voxel_size*8]
        m = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii))
        m.remove_degenerate_triangles(); m.remove_duplicated_triangles()
        m.remove_duplicated_vertices(); m.remove_non_manifold_edges()
        return m

    def _poisson():
        m, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
        densities = np.asarray(densities)
        # 10th percentile (was 5th) — removes more low-density mesh edges
        m.remove_vertices_by_mask(densities < np.quantile(densities, 0.10))
        m.remove_degenerate_triangles(); m.remove_duplicated_triangles()
        m.remove_duplicated_vertices(); m.remove_non_manifold_edges()
        return m

    if method == "bpa":
        return _bpa(), "bpa"
    if method == "poisson":
        m = _poisson()
        if not m.is_watertight():
            print("  [Recon] Poisson not watertight → BPA")
            return _bpa(), "bpa"
        return m, "poisson"
    return _bpa(), "bpa"

def compute_surface_area(mesh):
    a = float(mesh.get_surface_area())
    return {"area_m2": a, "area_cm2": a*1e4,
            "vertex_count": len(mesh.vertices),
            "triangle_count": len(mesh.triangles),
            "is_watertight": bool(mesh.is_watertight())}

def validate_with_reference(
    measured_area: float,
    ground_truth_area: float,
) -> dict:
    """
    Compare pipeline output against a known ground-truth area.
    Ref: Bondar et al. (2026) validation methodology.

    Returns:
        mae_m2      — Mean Absolute Error in m²
        mape_pct    — Mean Absolute Percentage Error in %
        within_20pct — True when MAPE ≤ 20%
    """
    mae = abs(measured_area - ground_truth_area)
    mape = mae / (ground_truth_area + 1e-9) * 100.0
    within_20 = mape <= 20.0

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter([ground_truth_area], [measured_area], s=120, zorder=5,
               color="#2196F3", label="Measurement")

    lim = max(ground_truth_area, measured_area) * 1.35
    xs = np.linspace(0, lim, 300)
    ax.plot(xs, xs, "k--", linewidth=1.2, label="Perfect (0% error)")
    ax.fill_between(xs, xs * 0.80, xs * 1.20, alpha=0.12, color="green", label="±20% band")

    ax.annotate(
        f"MAE={mae:.4f} m²\nMAPE={mape:.1f}%",
        xy=(ground_truth_area, measured_area),
        xytext=(ground_truth_area * 0.6, measured_area * 1.15),
        arrowprops=dict(arrowstyle="->", color="gray"),
        fontsize=9,
    )
    ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    ax.set_xlabel("Ground Truth (m²)")
    ax.set_ylabel("Measured (m²)")
    ax.set_title("Measured vs. Ground Truth Surface Area")
    ax.legend()
    plt.tight_layout()
    plt.show()

    return {"mae_m2": mae, "mape_pct": mape, "within_20pct": within_20}


def validate_scale(
    bbox_extent: np.ndarray,
    ocr_size_m: Optional[float] = None,
    bucket_model: Optional[str] = None,
    person_height_m: Optional[float] = None,
) -> dict:
    """
    Priority: OCR (absolute) > spec table > person height > none.
    """
    width_m = float(bbox_extent[0])

    if ocr_size_m and ocr_size_m > 0:
        dev = 100.0 * abs(width_m - ocr_size_m) / ocr_size_m
        return {"passed": dev <= 15.0, "deviation_pct": dev,
                "message": f"OCR={ocr_size_m:.3f}m  measured={width_m:.3f}m  dev={dev:.1f}%",
                "source": "ocr"}

    spec = lookup_bucket_spec_width(bucket_model) if bucket_model else None
    if spec:
        dev = 100.0 * abs(width_m - spec) / spec
        return {"passed": dev <= 15.0, "deviation_pct": dev,
                "message": f"{bucket_model} spec={spec:.2f}m  measured={width_m:.2f}m  dev={dev:.1f}%",
                "source": "spec-table"}

    if person_height_m and bbox_extent[1] > 0:
        ratio = float(bbox_extent[1]) / person_height_m
        return {"passed": True, "deviation_pct": 0.0,
                "message": f"height ratio vs person: {ratio:.2f}",
                "source": "person"}

    return {"passed": True, "deviation_pct": 0.0,
            "message": "no scale reference", "source": "none"}

def run_pipeline(
    ply_path: Union[str, list[str]],
    image_path: str,
    intrinsics: Optional[dict] = None,
    ios_mask_path: Optional[str] = None,
    meta_path: Optional[str] = None,
    person_height_m: Optional[float] = None,
    bucket_model: Optional[str] = None,
    auto_calibrate: bool = False,
    visualize: bool = True,
    ground_truth_area: Optional[float] = None,
) -> dict:
    print("\n╔══════════════════════════════════════════════════╗")
    print("║  🏗️   Komatsu Bucket Pipeline v3                 ║")
    print("╚══════════════════════════════════════════════════╝")

    counts: dict[str, int] = {}

    print("\n[0] Loading point cloud(s)…")
    if isinstance(ply_path, list):
        pcd_raw = merge_scans_icp(ply_path)
        multi_view = True
    else:
        pcd_raw = o3d.io.read_point_cloud(ply_path)
        multi_view = False
    counts["initial"] = len(pcd_raw.points)
    print(f"  initial: {counts['initial']:,} pts (multi_view={multi_view})")

    print("\n[1] iOS artifacts…")
    artifacts = load_ios_artifacts(
        ply_path if isinstance(ply_path, str) else ply_path[0],
        ios_mask_path, meta_path,
    )

    if intrinsics is None:
        intrinsics = artifacts["intrinsics"]
    if intrinsics is None:
        raise ValueError("intrinsics required (pass via arg or meta.json)")
    if bucket_model is None:
        bucket_model = artifacts.get("model_code")
    ocr_size_m = artifacts.get("ocr_size_m")
    tap_xy = artifacts.get("tap_location")
    gravity = artifacts.get("gravity")

    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"อ่านภาพไม่ได้: {image_path}")
    image = undistort_image(image, intrinsics)

    using_ios_mask = artifacts["mask"] is not None
    if using_ios_mask:
        mask = artifacts["mask"]
        print(f"  using iOS instance mask (primary)")
    else:
        print("  no iOS mask — fallback to multi-color HSV")
        if auto_calibrate:

            pass
        mask = build_multicolor_bucket_mask(image)
        mask = keep_largest_blob(mask)

    print("\n[2] Ground-plane removal…")
    pcd_noground = remove_ground_plane(pcd_raw, gravity=gravity)
    counts["after_ground"] = len(pcd_noground.points)

    print("\n[3] Depth threshold + mask projection…")
    pcd_depth, depth_thr = cut_by_depth(pcd_noground, method="auto")
    counts["after_depth"] = len(pcd_depth.points)
    pcd_masked = apply_color_mask_to_pcd(pcd_depth, mask, image.shape, intrinsics, flip_y=True)
    counts["after_mask"] = len(pcd_masked.points)

    print("\n[6] Multi-plane RANSAC…")
    pcd_planes = refine_with_ransac_multi(pcd_masked)
    counts["after_ransac"] = len(pcd_planes.points)

    print("\n[7] Instance separation (tap-ray + bbox)…")

    spec_w = lookup_bucket_spec_width(bucket_model) if bucket_model else None
    max_size = (spec_w * 1.3) if spec_w else 2.0
    pcd_final = select_cluster_near_ray(
        pcd_planes, tap_xy=tap_xy, intrinsics=intrinsics,
        max_bucket_size_m=max_size,
    )
    counts["after_instance"] = len(pcd_final.points)

    print("\n[7.5] Taubin smoothing filter…")
    if counts["after_instance"] >= 100:
        # Build a quick BPA mesh to smooth before final reconstruction
        pcd_final.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.015, max_nn=30))
        _pre_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd_final, o3d.utility.DoubleVector([0.006, 0.012, 0.024]))
        if len(_pre_mesh.vertices) > 0:
            _smoothed = apply_taubin_filter(_pre_mesh)
            # Convert smoothed mesh vertices back to point cloud for reconstruction
            pcd_final = o3d.geometry.PointCloud()
            pcd_final.points = _smoothed.vertices
            if _smoothed.has_vertex_normals():
                pcd_final.normals = _smoothed.vertex_normals
        else:
            print("  [Taubin] pre-mesh empty — skip smoothing")

    print("\n[8] Reconstruction…")
    if counts["after_instance"] < 100:
        print(f"  ⚠️  too few points ({counts['after_instance']})")
        mesh = o3d.geometry.TriangleMesh()
        method_used = "none"
    else:
        method_used = "auto" if not multi_view else "poisson"
        mesh, method_used = reconstruct_surface(pcd_final, method=method_used)

    area_info = compute_surface_area(mesh) if len(mesh.vertices) > 0 else {
        "area_m2": 0.0, "area_cm2": 0.0, "vertex_count": 0,
        "triangle_count": 0, "is_watertight": False}

    if len(pcd_final.points) > 0:
        ext = pcd_final.get_axis_aligned_bounding_box().get_extent()
    else:
        ext = np.zeros(3)
    width_m, height_m, depth_m = float(ext[0]), float(ext[1]), float(ext[2])

    scale = validate_scale(ext, ocr_size_m=ocr_size_m,
                           bucket_model=bucket_model,
                           person_height_m=person_height_m)
    print(f"\n[scale] {scale['source']}: {scale['message']}")

    if not scale["passed"]:
        confidence = "low"
    elif scale["source"] == "ocr":
        confidence = "high" if (multi_view and area_info["is_watertight"]) else "medium"
    elif method_used == "poisson" and area_info["is_watertight"]:
        confidence = "high"
    elif counts["after_instance"] > 1000:
        confidence = "medium"
    else:
        confidence = "low"

    print("\n╔══════════════════════════════════════════════════╗")
    print("║                   📐 สรุปผลลัพธ์                  ║")
    print("╚══════════════════════════════════════════════════╝")
    for k, v in counts.items():
        print(f"  {k:>14}: {v:,}")
    print(f"  พื้นที่ผิว:        {area_info['area_m2']:.4f} m² "
          f"({area_info['area_cm2']:,.0f} cm²) [{method_used}]")
    print(f"  ขนาด bbox:       กว้าง {width_m:.2f} × สูง {height_m:.2f} × ลึก {depth_m:.2f} m")
    print(f"  Scale [{scale['source']}]: {scale['message']}")
    print(f"  Confidence:      {confidence.upper()}")

    result = {
        **area_info,
        "bbox_width_m": width_m, "bbox_height_m": height_m, "bbox_depth_m": depth_m,
        "depth_threshold": depth_thr, "counts": counts,
        "recon_method": method_used, "scale_passed": scale["passed"],
        "scale_message": scale["message"], "scale_source": scale["source"],
        "confidence": confidence, "multi_view": multi_view,
        "used_ios_mask": using_ios_mask,
    }

    if ground_truth_area is not None and area_info["area_m2"] > 0:
        print("\n[Validation] comparing against ground truth…")
        val = validate_with_reference(area_info["area_m2"], ground_truth_area)
        result["validation"] = val
        status = "✅ within ±20%" if val["within_20pct"] else "❌ outside ±20%"
        print(f"  MAE:  {val['mae_m2']:.4f} m²")
        print(f"  MAPE: {val['mape_pct']:.1f}%  {status}")

    if visualize:
        _visualize_panels(image, mask, pcd_noground, pcd_depth, pcd_final, mesh, counts)
    return result

def _visualize_panels(image, mask, pcd_noground, pcd_depth, pcd_final, mesh, counts):
    fig = plt.figure(figsize=(18, 10))

    ax1 = fig.add_subplot(2, 4, 1)
    ax1.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    ax1.set_title("1. Image"); ax1.axis("off")

    ax2 = fig.add_subplot(2, 4, 2)
    ax2.imshow(mask, cmap="gray")
    ax2.set_title("2. Mask (iOS or HSV)"); ax2.axis("off")

    for i, (pcd, title) in enumerate([
        (pcd_noground, "3. After ground removal"),
        (pcd_depth, "4. After depth cut"),
        (pcd_final, "5. After instance pick"),
    ], start=3):
        ax = fig.add_subplot(2, 4, i, projection="3d")
        pts = np.asarray(pcd.points)
        if len(pts) > 0:
            if len(pts) > 5000:
                pts = pts[np.random.choice(len(pts), 5000, replace=False)]
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1, c=pts[:, 2], cmap="viridis")
        ax.set_title(title)

    ax6 = fig.add_subplot(2, 4, 6, projection="3d")
    if len(mesh.vertices) > 0:
        v = np.asarray(mesh.vertices); f = np.asarray(mesh.triangles)
        ax6.plot_trisurf(v[:, 0], v[:, 1], v[:, 2], triangles=f,
                         color="green", alpha=0.7, edgecolor="darkgreen", linewidth=0.1)
    ax6.set_title("6. Reconstructed mesh")

    ax7 = fig.add_subplot(2, 4, 7)
    labels = list(counts.keys())
    values = [counts[k] for k in labels]
    ax7.bar(labels, values, color="#69b")
    ax7.set_title("7. Per-stage counts")
    ax7.tick_params(axis="x", rotation=45)
    for i, v in enumerate(values):
        ax7.text(i, v, f"{v:,}", ha="center", va="bottom", fontsize=7)

    plt.tight_layout()
    plt.show()

def _cli():
    """
    Usage:
        python bucket_pipeline.py <stem>
        python bucket_pipeline.py bucket_2026-05-21.ply

    Resolves companion files automatically:
        <stem>.ply | <stem>.jpg | <stem>.meta.json | <stem>.mask.png
    """
    import argparse
    p = argparse.ArgumentParser(description="Komatsu bucket surface-area pipeline")
    p.add_argument("input", help="Path to .ply file or shared stem")
    p.add_argument("--model", default=None, help="Bucket model code (overrides meta.json)")
    p.add_argument("--person-height", type=float, default=None, help="Person height in meters (scale fallback)")
    p.add_argument("--no-viz", action="store_true", help="Skip matplotlib visualization")
    args = p.parse_args()

    stem = args.input[:-4] if args.input.endswith(".ply") else args.input
    ply = stem + ".ply"
    jpg = stem + ".jpg"
    meta = stem + ".meta.json"
    mask = stem + ".mask.png"

    if not Path(ply).exists():
        print(f"❌ not found: {ply}")
        sys.exit(1)
    if not Path(jpg).exists():
        print(f"❌ not found: {jpg}")
        sys.exit(1)

    result = run_pipeline(
        ply_path=ply,
        image_path=jpg,
        meta_path=meta if Path(meta).exists() else None,
        ios_mask_path=mask if Path(mask).exists() else None,
        bucket_model=args.model,
        person_height_m=args.person_height,
        visualize=not args.no_viz,
    )
    print(f"\nพื้นที่ผิว: {result['area_m2']:.4f} m² "
          f"[{result['confidence']}, scale={result['scale_source']}]")

if __name__ == "__main__":
    _cli()
