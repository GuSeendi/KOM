# Bucket Pipeline (Python)

Offline surface-area pipeline for excavator buckets scanned with iPhone LiDAR.

## Install

```bash
pip install -r requirements.txt
```

## Inputs (from the iOS app's Documents folder)

Each scan exports three companion files with a shared timestamp stem:

| File | Required | Purpose |
|---|---|---|
| `bucket_<ts>.ply` | yes | LiDAR + tap-segmented point cloud |
| `bucket_<ts>.jpg` | yes | RGB frame at scan time |
| `bucket_<ts>.meta.json` | recommended | Camera intrinsics, tap pixel, gravity, OCR text |
| `bucket_<ts>.mask.png` | optional | iOS Vision foreground instance mask (most accurate) |

If `meta.json` is missing, pass `intrinsics=` to `run_pipeline()` manually.
If `mask.png` is missing, the pipeline falls back to multi-color HSV
(yellow paint + red wear teeth + dark steel bracket).

## Run

```bash
python bucket_pipeline.py
```

Edit the `__main__` block in `bucket_pipeline.py` to point at your scan files.

## Pipeline stages

```
[0] Multi-view ICP fusion         (optional; list of .ply)
[1] Instance mask                 (iOS PRIMARY → HSV+LAB fallback)
[2] Ground-plane removal          (gravity-aligned RANSAC)
[3] Depth threshold + projection  (Y-flip ARKit → OpenCV)
[4] Specular & shadow recovery    (HSV path only)
[5] Multi-color part union        (HSV path only)
[6] Multi-plane RANSAC            (up to 5 planes)
[7] Instance separation           (DBSCAN + tap-ray pick + bbox limit)
[8] Reconstruction + scale check  (BPA default; Poisson if watertight)
```

## Output

- `area_m2`, `area_cm2`, `confidence` (high / medium / low)
- bbox dimensions (width × height × depth)
- `recon_method` (`bpa` or `poisson`)
- `scale_source` (`ocr` / `spec-table` / `person` / `none`)
- Per-stage point counts
- 7-panel matplotlib visualization

## Scale validation priority

1. OCR (`stamped_size` in meta.json) — absolute ground truth
2. Spec table (Komatsu / Cat / Hitachi / Volvo) by `model_code`
3. Person height (`person_height_m` arg)
4. None → confidence drops to `low`

See `../IMPLEMENTATION_PLAN.md` for design notes and hard rules.
