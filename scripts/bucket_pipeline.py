#!/usr/bin/env python3
"""
Komatsu Bucket Surface Area Pipeline
=====================================

ขั้นที่ 3-5 ของ pipeline วัดพื้นที่ผิวบุ้งกี๋:
  3. Preprocessing  — noise removal, voxel downsampling, ICP registration
  4. Reconstruction — Poisson Surface Reconstruction → triangle mesh
  5. Measurement    — mesh.get_surface_area() → cm²

Usage:
    # scan ไฟล์เดียว
    python bucket_pipeline.py scan.ply

    # หลาย scan — merge อัตโนมัติ
    python bucket_pipeline.py front.ply side.ply back.ply

    # ตั้งค่าเพิ่มเติม
    python bucket_pipeline.py scan.ply --voxel 0.005 --depth 9 --visualize

Dependencies:
    pip install open3d numpy
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("❌ ต้องติดตั้ง Open3D ก่อน:")
    print("   pip install open3d")
    sys.exit(1)


# ──────────────────────────────────────────────────────────────
#  ขั้นที่ 3: Preprocessing
# ──────────────────────────────────────────────────────────────

def load_and_clean(
    ply_path: str,
    voxel_size: float = 0.003,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0,
) -> o3d.geometry.PointCloud:
    """
    โหลด .ply แล้วกรอง noise + downsample

    Args:
        ply_path:      Path ของไฟล์ .ply
        voxel_size:    ขนาด voxel สำหรับ downsampling (เมตร)
        nb_neighbors:  จำนวน neighbor สำหรับ SOR
        std_ratio:     ค่า std สำหรับตัดสิน outlier

    Returns:
        Point cloud ที่ผ่านการ clean แล้ว
    """
    print(f"  📂 Loading: {ply_path}")
    pcd = o3d.io.read_point_cloud(str(ply_path))

    if len(pcd.points) == 0:
        print(f"  ⚠️  ไฟล์ว่างหรืออ่านไม่ได้: {ply_path}")
        return pcd

    print(f"     Raw points: {len(pcd.points):,}")

    # --- Statistical Outlier Removal ---
    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    pcd = pcd.select_by_index(ind)
    print(f"     After SOR (nb={nb_neighbors}, std={std_ratio}): {len(pcd.points):,}")

    # --- Voxel Downsampling ---
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"     After voxel downsample ({voxel_size*1000:.1f}mm): {len(pcd.points):,}")

    return pcd


def merge_scans(
    scan_paths: list,
    voxel_size: float = 0.003,
    icp_threshold: float = 0.01,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0,
) -> o3d.geometry.PointCloud:
    """
    โหลด scan หลายมุมแล้วรวมด้วย ICP Registration

    Args:
        scan_paths:     List ของ paths ไฟล์ .ply
        voxel_size:     ขนาด voxel สำหรับ downsampling
        icp_threshold:  Max correspondence distance สำหรับ ICP (เมตร)

    Returns:
        Point cloud ที่รวมแล้ว
    """
    clouds = []

    for path in scan_paths:
        print(f"\n{'─' * 40}")
        pcd = load_and_clean(path, voxel_size, nb_neighbors, std_ratio)

        if len(pcd.points) == 0:
            print(f"  ⚠️  ข้าม scan ว่าง: {path}")
            continue

        # Estimate normals (จำเป็นสำหรับ point-to-plane ICP)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 5,
                max_nn=30,
            )
        )
        clouds.append(pcd)

    if len(clouds) == 0:
        print("❌ ไม่มี point cloud ที่ใช้ได้")
        sys.exit(1)

    if len(clouds) == 1:
        print(f"\n✅ Single scan: {len(clouds[0].points):,} points")
        return clouds[0]

    # --- ICP Registration ---
    merged = clouds[0]

    for i, source in enumerate(clouds[1:], start=2):
        print(f"\n🔗 ICP: Registering scan {i}/{len(clouds)} → base...")

        # Initial alignment: identity (ถ้า scan จากแอปเดียวกันจะอยู่ใน world space เดียว)
        init_transform = np.eye(4)

        reg = o3d.pipelines.registration.registration_icp(
            source,
            merged,
            icp_threshold,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
        )

        print(f"     Fitness:  {reg.fitness:.4f}")
        print(f"     RMSE:     {reg.inlier_rmse:.6f} m")

        if reg.fitness < 0.3:
            print(f"  ⚠️  Fitness ต่ำมาก — scan อาจไม่ overlap กัน")

        source.transform(reg.transformation)
        merged = merged + source

    # Final downsample
    merged = merged.voxel_down_sample(voxel_size=voxel_size)
    print(f"\n✅ Merged total: {len(merged.points):,} points")

    return merged


# ──────────────────────────────────────────────────────────────
#  ขั้นที่ 4: Surface Reconstruction
# ──────────────────────────────────────────────────────────────

def reconstruct_mesh(
    pcd: o3d.geometry.PointCloud,
    depth: int = 9,
    density_quantile: float = 0.05,
    normal_radius: float = 0.01,
) -> o3d.geometry.TriangleMesh:
    """
    สร้าง mesh ด้วย Poisson Surface Reconstruction

    Args:
        pcd:               Point cloud (ต้องมี normals)
        depth:             Octree depth สำหรับ Poisson (สูง = ละเอียดมาก)
        density_quantile:  Quantile สำหรับตัด low-density vertices
        normal_radius:     Radius สำหรับ estimate normals

    Returns:
        Triangle mesh
    """
    print(f"\n{'═' * 50}")
    print(f"🏔️  Poisson Surface Reconstruction (depth={depth})")
    print(f"{'═' * 50}")

    # Estimate normals ใหม่ให้ consistent
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius,
            max_nn=30,
        )
    )

    # Orient normals ให้ชี้ออกด้านเดียว
    pcd.orient_normals_consistent_tangent_plane(k=15)
    print(f"  ✅ Normals estimated & oriented")

    # Poisson reconstruction
    t0 = time.time()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd,
        depth=depth,
        width=0,
        scale=1.1,
        linear_fit=False,
    )
    elapsed = time.time() - t0

    print(f"  ✅ Mesh created in {elapsed:.1f}s")
    print(f"     Vertices:  {len(mesh.vertices):,}")
    print(f"     Triangles: {len(mesh.triangles):,}")

    # --- Trim low-density vertices ---
    densities = np.asarray(densities)
    threshold = np.quantile(densities, density_quantile)
    vertices_to_remove = densities < threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)

    print(f"  ✅ After density trim (q={density_quantile}):")
    print(f"     Vertices:  {len(mesh.vertices):,}")
    print(f"     Triangles: {len(mesh.triangles):,}")

    # Clean mesh
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    return mesh


# ──────────────────────────────────────────────────────────────
#  ขั้นที่ 5: Surface Area Calculation
# ──────────────────────────────────────────────────────────────

def compute_surface_area(mesh: o3d.geometry.TriangleMesh) -> dict:
    """
    คำนวณพื้นที่ผิว mesh

    Returns:
        dict with area_m2, area_cm2, area_ft2
    """
    area_m2 = mesh.get_surface_area()
    area_cm2 = area_m2 * 1e4
    area_ft2 = area_m2 * 10.7639

    return {
        "area_m2": area_m2,
        "area_cm2": area_cm2,
        "area_ft2": area_ft2,
        "vertices": len(mesh.vertices),
        "triangles": len(mesh.triangles),
    }


def compute_volume(mesh: o3d.geometry.TriangleMesh) -> float:
    """
    คำนวณปริมาตร mesh (ถ้า watertight)

    Returns:
        Volume in m³ หรือ -1 ถ้า mesh ไม่ watertight
    """
    if mesh.is_watertight():
        return mesh.get_volume()
    else:
        return -1.0


# ──────────────────────────────────────────────────────────────
#  Visualization
# ──────────────────────────────────────────────────────────────

def visualize_result(pcd, mesh):
    """แสดง point cloud และ mesh ใน Open3D viewer"""
    print("\n🔍 เปิด Open3D Viewer...")
    print("   กด Q เพื่อปิด")

    # แสดง mesh + wireframe
    mesh_wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    mesh_wire.paint_uniform_color([0.3, 0.3, 0.3])

    mesh_vis = o3d.geometry.TriangleMesh(mesh)
    mesh_vis.compute_vertex_normals()
    mesh_vis.paint_uniform_color([0.7, 0.85, 0.95])

    o3d.visualization.draw_geometries(
        [mesh_vis],
        window_name="Komatsu Bucket — Surface Mesh",
        width=1200,
        height=800,
    )


# ──────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Komatsu Bucket Surface Area Pipeline (ขั้น 3-5)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
ตัวอย่างการใช้งาน:
  python bucket_pipeline.py scan.ply
  python bucket_pipeline.py front.ply side.ply back.ply
  python bucket_pipeline.py scan.ply --voxel 0.005 --depth 9 --visualize
  python bucket_pipeline.py scan.ply --output result_mesh.ply
        """,
    )

    parser.add_argument(
        "scans",
        nargs="+",
        help="ไฟล์ .ply point cloud (1 ไฟล์ขึ้นไป)",
    )
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.003,
        help="ขนาด voxel สำหรับ downsampling (เมตร, default: 0.003)",
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=9,
        help="Octree depth สำหรับ Poisson reconstruction (default: 9)",
    )
    parser.add_argument(
        "--icp-threshold",
        type=float,
        default=0.01,
        help="Max correspondence distance สำหรับ ICP (เมตร, default: 0.01)",
    )
    parser.add_argument(
        "--density-trim",
        type=float,
        default=0.05,
        help="Density quantile สำหรับตัด low-density vertices (default: 0.05)",
    )
    parser.add_argument(
        "--sor-neighbors",
        type=int,
        default=20,
        help="จำนวน neighbors สำหรับ Statistical Outlier Removal (default: 20)",
    )
    parser.add_argument(
        "--sor-std",
        type=float,
        default=2.0,
        help="Standard deviation ratio สำหรับ SOR (default: 2.0)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="bucket_mesh.ply",
        help="ชื่อไฟล์ mesh output (default: bucket_mesh.ply)",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="เปิด Open3D viewer แสดง mesh หลังประมวลผล",
    )
    parser.add_argument(
        "--save-cleaned",
        type=str,
        default=None,
        help="บันทึก cleaned point cloud ก่อน reconstruction",
    )

    args = parser.parse_args()

    # ตรวจไฟล์ input
    for scan_path in args.scans:
        if not Path(scan_path).exists():
            print(f"❌ ไม่พบไฟล์: {scan_path}")
            sys.exit(1)

    # ─────────────────────────────────
    print()
    print("╔" + "═" * 50 + "╗")
    print("║  🏗️  Komatsu Bucket Surface Area Pipeline        ║")
    print("╚" + "═" * 50 + "╝")

    t_start = time.time()

    # ── ขั้นที่ 3: Preprocess & Merge ──
    print(f"\n📋 Input: {len(args.scans)} scan(s)")
    merged_pcd = merge_scans(
        args.scans,
        voxel_size=args.voxel,
        icp_threshold=args.icp_threshold,
        nb_neighbors=args.sor_neighbors,
        std_ratio=args.sor_std,
    )

    if args.save_cleaned:
        o3d.io.write_point_cloud(args.save_cleaned, merged_pcd)
        print(f"\n💾 Cleaned point cloud saved: {args.save_cleaned}")

    # ── ขั้นที่ 4: Surface Reconstruction ──
    mesh = reconstruct_mesh(
        merged_pcd,
        depth=args.depth,
        density_quantile=args.density_trim,
        normal_radius=args.voxel * 5,
    )

    # ── ขั้นที่ 5: Surface Area ──
    result = compute_surface_area(mesh)
    volume = compute_volume(mesh)

    t_total = time.time() - t_start

    # ── Results ──
    print()
    print("╔" + "═" * 50 + "╗")
    print("║  📐 ผลลัพธ์                                       ║")
    print("╠" + "═" * 50 + "╣")
    print(f"║  พื้นที่ผิว:  {result['area_cm2']:>12,.2f} cm²              ║")
    print(f"║             {result['area_m2']:>12,.4f} m²               ║")
    print(f"║             {result['area_ft2']:>12,.2f} ft²              ║")
    print("╠" + "═" * 50 + "╣")

    if volume > 0:
        volume_liters = volume * 1000
        print(f"║  ปริมาตร:    {volume_liters:>12,.2f} L                ║")
        print(f"║             {volume:>12,.6f} m³               ║")
        print("╠" + "═" * 50 + "╣")

    print(f"║  Vertices:   {result['vertices']:>12,}                  ║")
    print(f"║  Triangles:  {result['triangles']:>12,}                  ║")
    print(f"║  เวลา:       {t_total:>12.1f} s                 ║")
    print("╚" + "═" * 50 + "╝")

    # ── Save mesh ──
    o3d.io.write_triangle_mesh(args.output, mesh)
    print(f"\n💾 Mesh saved: {args.output}")
    print(f"   → เปิดใน MeshLab เพื่อตรวจสอบด้วยตาก่อนยืนยันค่า")

    # ── Visualize ──
    if args.visualize:
        visualize_result(merged_pcd, mesh)

    return result


if __name__ == "__main__":
    main()
