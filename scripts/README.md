# 🏗️ Komatsu Bucket Pipeline — Python Scripts

สคริปต์ Python สำหรับประมวลผล point cloud จาก LiDAR scan (ขั้นที่ 3–5)

## ติดตั้ง

```bash
# สร้าง virtual environment (แนะนำ)
python3 -m venv venv
source venv/bin/activate

# ติดตั้ง dependencies
pip install -r requirements.txt
```

## วิธีใช้

### Scan ไฟล์เดียว

```bash
python bucket_pipeline.py scan.ply
```

### หลาย Scan (merge อัตโนมัติด้วย ICP)

```bash
python bucket_pipeline.py front.ply side.ply back.ply
```

### ตั้งค่าเพิ่มเติม

```bash
python bucket_pipeline.py scan.ply \
    --voxel 0.005 \           # ขนาด voxel 5mm
    --depth 9 \               # ความละเอียด Poisson
    --icp-threshold 0.01 \    # ICP max distance 1cm
    --density-trim 0.05 \     # ตัด low-density 5%
    --output result.ply \     # ชื่อไฟล์ output
    --visualize               # เปิด Open3D viewer
```

### บันทึก Cleaned Point Cloud

```bash
python bucket_pipeline.py scan.ply --save-cleaned cleaned.ply
```

## Pipeline

| ขั้น | กระบวนการ | หมายเหตุ |
|------|-----------|----------|
| 3.1 | Statistical Outlier Removal | กรอง noise จาก LiDAR |
| 3.2 | Voxel Downsampling | ลด density ให้สม่ำเสมอ |
| 3.3 | ICP Registration | รวม scan หลายมุม |
| 4 | Poisson Reconstruction | สร้าง triangle mesh |
| 5 | `get_surface_area()` | คำนวณพื้นที่ผิว (cm²) |

## Output

- `bucket_mesh.ply` — Triangle mesh สำหรับตรวจสอบใน MeshLab
- Console output — พื้นที่ผิว (cm², m², ft²) + ปริมาตร (ถ้า watertight)

## ⚠️ ข้อควรระวัง

- พื้นผิวโลหะมันวาวจะสะท้อน LiDAR → point cloud หายบางส่วน
- แนะนำพ่นสี matte ชั่วคราว หรือ scan ในแสงสม่ำเสมอ
- ตรวจสอบ mesh ใน MeshLab ก่อนยืนยันค่าพื้นที่
