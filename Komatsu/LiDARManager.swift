import ARKit
import Combine
import CoreImage
import RealityKit
import simd

enum MeasureMode: String, CaseIterable {
    case surface = "พื้นที่ผิว"
    case length = "ความยาว"
}

enum DetectionState: String {
    case searching = "กำลังค้นหาบุ้งกี๋..."
    case detected = "พบวัตถุ"
    case captured = "จับพื้นผิวแล้ว"
}

struct MeasurePoint: Identifiable {
    let id = UUID()
    let position: SIMD3<Float>
    let anchor: AnchorEntity
}

class LiDARManager: NSObject, ObservableObject, ARSessionDelegate {

    let session = ARSession()
    var arView: ARView?

    @Published var mode: MeasureMode = .surface
    @Published var detectionState: DetectionState = .searching
    @Published var detectedBBox: CGRect = .zero
    @Published var meshSurfaceArea: Float = 0
    @Published var meshTriangleCount: Int = 0
    @Published var isSurfaceCaptured: Bool = false
    @Published var capturedArea: Float = 0

    @Published var measuredLength: Float = 0
    @Published var lengthPoints: [MeasurePoint] = []
    @Published var isMeasuringLength: Bool = false
    @Published var liveDistance: Float = 0

    @Published var volume: Float = 0

    @Published var statusMessage: String = "ชี้กล้องไปที่บุ้งกี๋"
    @Published var isReady: Bool = false

    @Published var isScanning: Bool = false
    @Published var scannedPointCount: Int = 0
    @Published var lastExportURL: URL?

    private let detector = BucketDetector()
    private let meshAnalyzer = MeshAnalyzer()
    private var updateTimer: Timer?

    private let classifier = BucketColorClassifier()
    private var accumulatedPoints: [SIMD3<Float>] = []
    private var scanFrameCounter: Int = 0
    private let voxelSize: Float = 0.005
    private var voxelSet: Set<SIMD3<Int32>> = []

    private var tapNormPortrait: CGPoint = .zero
    private var referenceDepth: Float = -1

    @Published var scanFilter = ScanFilter()

    private let ingestQueue = DispatchQueue(label: "LiDARManager.ingest", qos: .userInitiated)
    private var ingestInFlight: Bool = false
    private let ingestLock = NSLock()

    private struct FrameSnapshot {
        let jpegData: Data
        let imageWidth: Int
        let imageHeight: Int
        let fx: Float
        let fy: Float
        let cx: Float
        let cy: Float
        let tapImagePx: (Int, Int)
        let gravity: SIMD3<Float>
    }
    private var lastSnapshot: FrameSnapshot?

    private var lengthLineAnchor: AnchorEntity?
    private var liveLineAnchor: AnchorEntity?
    private var liveEndAnchor: AnchorEntity?

    override init() {
        super.init()
        session.delegate = self
    }

    func startSession() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        config.environmentTexturing = .automatic

        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }

        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
        }

        session.run(config, options: [.resetTracking, .removeExistingAnchors])

        updateTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.runDetectionPipeline()
            self?.updateLiveLine()
        }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        if !isReady {
            DispatchQueue.main.async { self.isReady = true }
        }
        guard isScanning else { return }

        ingestLock.lock()
        if ingestInFlight { ingestLock.unlock(); return }
        ingestInFlight = true
        ingestLock.unlock()

        ingestQueue.async { [weak self] in
            self?.ingestScanFrame(frame)
            self?.ingestLock.lock()
            self?.ingestInFlight = false
            self?.ingestLock.unlock()
        }
    }

    private func runDetectionPipeline() {
        guard mode == .surface, !isSurfaceCaptured else { return }

        guard !isScanning else { return }
        guard let frame = session.currentFrame else { return }

        detector.detect(frame: frame) { [weak self] result in
            guard let self = self, let detection = result else {
                DispatchQueue.main.async {
                    self?.detectionState = .searching
                    self?.detectedBBox = .zero
                    self?.meshSurfaceArea = 0
                    self?.statusMessage = "กำลังค้นหาบุ้งกี๋..."
                }
                return
            }

            let viewportSize = DispatchQueue.main.sync { self.arView?.bounds.size ?? CGSize(width: 390, height: 844) }
            let meshAnchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }

            let meshResult = self.meshAnalyzer.calculateArea(
                meshAnchors: meshAnchors,
                camera: frame.camera,
                detectionBox: detection.boundingBox,
                viewportSize: viewportSize
            )

            DispatchQueue.main.async {
                self.detectedBBox = detection.boundingBox
                self.meshSurfaceArea = meshResult.surfaceArea
                self.meshTriangleCount = meshResult.triangleCount

                if detection.boundingBox.width > 0.05 {
                    self.detectionState = .detected
                    let cm2 = meshResult.surfaceArea * 10_000
                    if cm2 > 1 {
                        self.statusMessage = "พบวัตถุ — พื้นที่: \(self.fmtArea(meshResult.surfaceArea)) — กด \"จับพื้นผิว\""
                    } else {
                        self.statusMessage = "พบวัตถุ — กำลังสร้าง mesh..."
                    }
                } else {
                    self.detectionState = .searching
                    self.statusMessage = "ชี้กล้องไปที่บุ้งกี๋"
                }
            }
        }
    }

    func captureSurface() {
        guard detectionState == .detected, meshSurfaceArea > 0 else {
            statusMessage = "ยังตรวจจับไม่ได้ — ลองเข้าใกล้อีก"
            return
        }

        capturedArea = meshSurfaceArea
        isSurfaceCaptured = true
        detectionState = .captured
        statusMessage = "พื้นที่ผิว: \(fmtArea(capturedArea))"
        recalcVolume()
    }

    func resetSurface() {
        isSurfaceCaptured = false
        capturedArea = 0
        meshSurfaceArea = 0
        meshTriangleCount = 0
        detectedBBox = .zero
        detectionState = .searching
        volume = 0
        statusMessage = "ชี้กล้องไปที่บุ้งกี๋"
    }

    func handleTap(at screenPoint: CGPoint) {
        guard mode == .length, let arView = arView else { return }
        let hits = arView.raycast(from: screenPoint, allowing: .estimatedPlane, alignment: .any)
        guard let hit = hits.first else { statusMessage = "ไม่เจอพื้นผิว"; return }

        let wp = SIMD3<Float>(hit.worldTransform.columns.3.x,
                               hit.worldTransform.columns.3.y,
                               hit.worldTransform.columns.3.z)

        if !isMeasuringLength {

            if lengthPoints.count >= 2 { clearLength() }

            let anchor = AnchorEntity(world: wp)
            let sphere = ModelEntity(mesh: .generateSphere(radius: 0.012),
                                      materials: [SimpleMaterial(color: .cyan, isMetallic: false)])
            anchor.addChild(sphere)
            arView.scene.addAnchor(anchor)
            lengthPoints.append(MeasurePoint(position: wp, anchor: anchor))

            isMeasuringLength = true
            statusMessage = "เลื่อนกล้องไปจุดปลาย แล้วแตะ"
        } else {

            let anchor = AnchorEntity(world: wp)
            let sphere = ModelEntity(mesh: .generateSphere(radius: 0.012),
                                      materials: [SimpleMaterial(color: .cyan, isMetallic: false)])
            anchor.addChild(sphere)
            arView.scene.addAnchor(anchor)
            lengthPoints.append(MeasurePoint(position: wp, anchor: anchor))

            isMeasuringLength = false
            measuredLength = distance(lengthPoints[0].position, wp)
            liveDistance = 0
            removeLiveVisuals()
            drawLine(from: lengthPoints[0].position, to: wp, color: .cyan, isLive: false)
            statusMessage = "ความยาว: \(fmtLen(measuredLength))"
            recalcVolume()
        }
    }

    private func updateLiveLine() {
        guard mode == .length, isMeasuringLength,
              let arView = arView, let start = lengthPoints.first
        else { return }

        let center = CGPoint(x: arView.bounds.midX, y: arView.bounds.midY)
        let hits = arView.raycast(from: center, allowing: .estimatedPlane, alignment: .any)
        guard let hit = hits.first else { return }

        let endPos = SIMD3<Float>(hit.worldTransform.columns.3.x,
                                   hit.worldTransform.columns.3.y,
                                   hit.worldTransform.columns.3.z)
        let dist = distance(start.position, endPos)

        DispatchQueue.main.async {
            self.liveDistance = dist
            self.statusMessage = "\(self.fmtLen(dist)) — แตะเพื่อกำหนดจุดสิ้นสุด"
        }

        removeLiveVisuals()
        drawLine(from: start.position, to: endPos, color: .cyan.withAlphaComponent(0.6), isLive: true)

        let endAnchor = AnchorEntity(world: endPos)
        let dot = ModelEntity(mesh: .generateSphere(radius: 0.01),
                               materials: [SimpleMaterial(color: .cyan.withAlphaComponent(0.5), isMetallic: false)])
        endAnchor.addChild(dot)
        arView.scene.addAnchor(endAnchor)
        liveEndAnchor = endAnchor
    }

    private func drawLine(from a: SIMD3<Float>, to b: SIMD3<Float>, color: UIColor, isLive: Bool) {
        guard let arView = arView else { return }
        let mid = (a + b) / 2
        let len = distance(a, b)
        guard len > 0.001 else { return }

        let anchor = AnchorEntity(world: mid)
        let line = ModelEntity(
            mesh: .generateBox(size: SIMD3<Float>(0.005, 0.005, len), cornerRadius: 0.002),
            materials: [SimpleMaterial(color: color, isMetallic: false)]
        )
        line.look(at: b, from: mid, relativeTo: nil)
        anchor.addChild(line)
        arView.scene.addAnchor(anchor)

        if isLive { liveLineAnchor = anchor } else { lengthLineAnchor = anchor }
    }

    private func removeLiveVisuals() {
        if let l = liveLineAnchor { arView?.scene.removeAnchor(l); liveLineAnchor = nil }
        if let e = liveEndAnchor { arView?.scene.removeAnchor(e); liveEndAnchor = nil }
    }

    private func recalcVolume() {
        let area = capturedArea > 0 ? capturedArea : meshSurfaceArea
        if area > 0 && measuredLength > 0 {
            volume = area * measuredLength
        }
    }

    private func clearLength() {
        for p in lengthPoints { arView?.scene.removeAnchor(p.anchor) }
        lengthPoints.removeAll()
        if let l = lengthLineAnchor { arView?.scene.removeAnchor(l) }
        lengthLineAnchor = nil
        removeLiveVisuals()
        isMeasuringLength = false
        liveDistance = 0
        measuredLength = 0
    }

    func resetAll() {
        resetSurface()
        clearLength()
        volume = 0
        isScanning = false
        ingestQueue.sync {
            self.accumulatedPoints.removeAll(keepingCapacity: false)
            self.voxelSet.removeAll(keepingCapacity: false)
            self.referenceDepth = -1
        }
        scannedPointCount = 0
        lastExportURL = nil
    }

    func startSegmentationScan(at tapPoint: CGPoint) {
        guard let view = arView else { return }
        let bounds = view.bounds
        guard bounds.width > 0, bounds.height > 0 else { return }

        let norm = CGPoint(x: max(0, min(1, tapPoint.x / bounds.width)),
                           y: max(0, min(1, tapPoint.y / bounds.height)))

        ingestQueue.sync {
            self.accumulatedPoints.removeAll(keepingCapacity: true)
            self.voxelSet.removeAll(keepingCapacity: true)
            self.scanFrameCounter = 0
            self.tapNormPortrait = norm
            self.referenceDepth = -1
        }
        scannedPointCount = 0
        lastExportURL = nil
        isScanning = true
        statusMessage = "กำลังสแกน… ค่อยๆ เลื่อนกล้องรอบวัตถุ"
    }

    func stopSegmentationScan() {
        isScanning = false
        statusMessage = "หยุดสแกน — \(scannedPointCount) จุด — กด Export"
    }

    @discardableResult
    func exportPLY() -> URL? {

        let points: [SIMD3<Float>] = ingestQueue.sync { self.accumulatedPoints }
        guard !points.isEmpty else {
            statusMessage = "ยังไม่มีจุดให้ export"
            return nil
        }
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let ts = ISO8601DateFormatter().string(from: Date())
            .replacingOccurrences(of: ":", with: "-")
        let url = docs.appendingPathComponent("bucket_\(ts).ply")

        var header = ""
        header += "ply\n"
        header += "format binary_little_endian 1.0\n"
        header += "element vertex \(points.count)\n"
        header += "property float x\nproperty float y\nproperty float z\n"
        header += "end_header\n"

        guard let headerData = header.data(using: .ascii) else { return nil }

        var packed = [Float]()
        packed.reserveCapacity(points.count * 3)
        for p in points {
            packed.append(p.x); packed.append(p.y); packed.append(p.z)
        }
        var data = Data(capacity: headerData.count + packed.count * 4)
        data.append(headerData)
        packed.withUnsafeBufferPointer { buf in
            if let base = buf.baseAddress {
                base.withMemoryRebound(to: UInt8.self, capacity: packed.count * 4) { ptr in
                    data.append(ptr, count: packed.count * 4)
                }
            }
        }

        do {
            try data.write(to: url, options: .atomic)
            lastExportURL = url

            writeSidecarFiles(plyURL: url)
            statusMessage = "บันทึก \(accumulatedPoints.count) จุดที่ \(url.lastPathComponent)"
            return url
        } catch {
            statusMessage = "Export ล้มเหลว: \(error.localizedDescription)"
            return nil
        }
    }

    private func writeSidecarFiles(plyURL: URL) {
        let snapshot: FrameSnapshot? = ingestQueue.sync { self.lastSnapshot }
        guard let snap = snapshot else { return }

        let stem = plyURL.deletingPathExtension()
        let imageURL = stem.appendingPathExtension("jpg")
        let metaURL = stem.appendingPathExtension("meta.json")

        try? snap.jpegData.write(to: imageURL, options: .atomic)

        let meta: [String: Any] = [
            "intrinsics": [
                "fx": snap.fx, "fy": snap.fy, "cx": snap.cx, "cy": snap.cy,
            ],
            "image_size": [snap.imageWidth, snap.imageHeight],
            "tap_location": [snap.tapImagePx.0, snap.tapImagePx.1],
            "gravity": [snap.gravity.x, snap.gravity.y, snap.gravity.z],
            "model_code": NSNull(),
            "stamped_size": NSNull(),
            "ocr_text": NSNull(),
            "timestamp": ISO8601DateFormatter().string(from: Date()),
        ]
        if let data = try? JSONSerialization.data(withJSONObject: meta,
                                                  options: [.prettyPrinted]) {
            try? data.write(to: metaURL, options: .atomic)
        }
    }

    private func ingestScanFrame(_ frame: ARFrame) {
        guard let depthData = frame.smoothedSceneDepth else { return }
        scanFrameCounter &+= 1

        let image = frame.capturedImage
        let depthMap = depthData.depthMap
        let confMap = depthData.confidenceMap

        let imgW = CVPixelBufferGetWidth(image)
        let imgH = CVPixelBufferGetHeight(image)
        let dW = CVPixelBufferGetWidth(depthMap)
        let dH = CVPixelBufferGetHeight(depthMap)
        guard imgW > 0, imgH > 0, dW > 0, dH > 0 else { return }

        let sx = Float(dW) / Float(imgW)
        let sy = Float(dH) / Float(imgH)
        let K = frame.camera.intrinsics
        let fx = K[0][0] * sx
        let fy = K[1][1] * sy
        let cx = K[2][0] * sx
        let cy = K[2][1] * sy
        let camToWorld = frame.camera.transform

        let nx = Float(tapNormPortrait.x)
        let ny = Float(tapNormPortrait.y)
        let tapDU = min(dW - 1, max(0, Int(ny * Float(dW))))
        let tapDV = min(dH - 1, max(0, Int((1.0 - nx) * Float(dH))))

        let tapIX = min(imgW - 1, max(0, Int(ny * Float(imgW))))
        let tapIY = min(imgH - 1, max(0, Int((1.0 - nx) * Float(imgH))))

        if scanFrameCounter % 30 == 0 {
            let ci = CIImage(cvPixelBuffer: image).oriented(.right)
            let ctx = CIContext()
            if let jpeg = ctx.jpegRepresentation(of: ci, colorSpace: CGColorSpaceCreateDeviceRGB(),
                                                 options: [kCGImageDestinationLossyCompressionQuality as CIImageRepresentationOption: 0.85]) {
                let snap = FrameSnapshot(
                    jpegData: jpeg,
                    imageWidth: imgW,
                    imageHeight: imgH,
                    fx: K[0][0], fy: K[1][1], cx: K[2][0], cy: K[2][1],
                    tapImagePx: (tapIX, tapIY),
                    gravity: SIMD3<Float>(0, 1, 0)
                )
                ingestQueue.async { [weak self] in
                    self?.lastSnapshot = snap
                }
            }
        }

        // Build yellow HSV mask at depth resolution before locking image again
        let yellowMask: CVPixelBuffer? = scanFilter.requireYellow
            ? YellowMaskProcessor.createYellowMask(frame: frame, depthW: dW, depthH: dH)
            : nil

        autoreleasepool {
            CVPixelBufferLockBaseAddress(image, .readOnly)
            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            if let c = confMap { CVPixelBufferLockBaseAddress(c, .readOnly) }
            if let ym = yellowMask { CVPixelBufferLockBaseAddress(ym, .readOnly) }
            defer {
                CVPixelBufferUnlockBaseAddress(image, .readOnly)
                CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
                if let c = confMap { CVPixelBufferUnlockBaseAddress(c, .readOnly) }
                if let ym = yellowMask { CVPixelBufferUnlockBaseAddress(ym, .readOnly) }
            }

            var yellowPtr: UnsafePointer<UInt8>? = nil
            var yellowRow = 0
            if let ym = yellowMask, let base = CVPixelBufferGetBaseAddress(ym) {
                yellowPtr = base.assumingMemoryBound(to: UInt8.self)
                yellowRow = CVPixelBufferGetBytesPerRow(ym)
            }

            guard let yBase = CVPixelBufferGetBaseAddressOfPlane(image, 0),
                  let cBase = CVPixelBufferGetBaseAddressOfPlane(image, 1) else { return }
            let yPtr = yBase.assumingMemoryBound(to: UInt8.self)
            let cPtr = cBase.assumingMemoryBound(to: UInt8.self)
            let yRow = CVPixelBufferGetBytesPerRowOfPlane(image, 0)
            let cRow = CVPixelBufferGetBytesPerRowOfPlane(image, 1)

            guard let depthBase = CVPixelBufferGetBaseAddress(depthMap) else { return }
            let depthPtr = depthBase.assumingMemoryBound(to: Float32.self)
            let depthRow = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size

            var confPtr2: UnsafePointer<UInt8>? = nil
            var confRow = 0
            if let confMap = confMap,
               let confBase = CVPixelBufferGetBaseAddress(confMap) {
                confPtr2 = UnsafePointer(confBase.assumingMemoryBound(to: UInt8.self))
                confRow = CVPixelBufferGetBytesPerRow(confMap)
            }

            if referenceDepth <= 0 {
                let d0 = depthPtr[tapDV * depthRow + tapDU]
                if d0 > 0 && d0 < 5 {
                    referenceDepth = d0
                    DispatchQueue.main.async {
                        self.statusMessage = String(format: "ระยะอ้างอิง %.2f m — เลื่อนกล้องรอบบุ้งกี๋", d0)
                    }
                } else {

                    return
                }
            }
            let refD = referenceDepth

            let imgPerDU = Float(imgW) / Float(dW)
            let imgPerDV = Float(imgH) / Float(dH)

            var candidates: [SIMD3<Float>] = []
            candidates.reserveCapacity(2048)
            let step = 2

            for v in stride(from: 0, to: dH, by: step) {
                for u in stride(from: 0, to: dW, by: step) {

                    let confVal: UInt8 = confPtr2.map { $0[v * confRow + u] } ?? UInt8(ARConfidenceLevel.high.rawValue)
                    let d = depthPtr[v * depthRow + u]
                    if d <= 0 { continue }

                    if !classifier.isWithinDepth(d, reference: refD) { continue }

                    let isYellow: Bool
                    if let yp = yellowPtr {
                        isYellow = yp[v * yellowRow + u] > 128
                    } else {
                        let ix = min(imgW - 1, max(0, Int(Float(u) * imgPerDU)))
                        let iy = min(imgH - 1, max(0, Int(Float(v) * imgPerDV)))
                        let yVal = yPtr[iy * yRow + ix]
                        let cIdx = (iy >> 1) * cRow + ((ix >> 1) << 1)
                        isYellow = classifier.isYellow(y: yVal, cb: cPtr[cIdx], cr: cPtr[cIdx + 1])
                    }

                    if !scanFilter.passes(depth: d, conf: confVal, isYellow: isYellow) { continue }

                    let xc =  (Float(u) - cx) / fx * d
                    let yc = -(Float(v) - cy) / fy * d
                    let zc = -d
                    let world4 = camToWorld * SIMD4<Float>(xc, yc, zc, 1)
                    candidates.append(SIMD3<Float>(world4.x, world4.y, world4.z))
                }
            }

            guard !candidates.isEmpty else { return }

            let keep: [SIMD3<Float>]
            if let fit = classifier.ransacPlane(points: candidates) {
                keep = fit.inlierIndices.map { candidates[$0] }
            } else {
                keep = candidates
            }

            let voxInv = 1.0 / voxelSize
            var added = 0
            for p in keep {
                let key = SIMD3<Int32>(Int32((p.x * voxInv).rounded()),
                                       Int32((p.y * voxInv).rounded()),
                                       Int32((p.z * voxInv).rounded()))
                if voxelSet.insert(key).inserted {
                    accumulatedPoints.append(p)
                    added += 1
                }
            }

            if added > 0 {
                let total = accumulatedPoints.count
                DispatchQueue.main.async { self.scannedPointCount = total }
            }
        }
    }

    func fmtArea(_ a: Float) -> String {
        let cm2 = a * 10_000
        return cm2 >= 10_000 ? String(format: "%.2f m²", a) : String(format: "%.1f cm²", cm2)
    }
    func fmtLen(_ l: Float) -> String {
        let cm = l * 100
        return cm >= 100 ? String(format: "%.2f m", l) : String(format: "%.1f cm", cm)
    }
    func fmtVol(_ v: Float) -> String {
        let L = v * 1000; let cm3 = v * 1_000_000
        return L >= 1 ? String(format: "%.2f L", L) : String(format: "%.0f cm³", cm3)
    }
}

