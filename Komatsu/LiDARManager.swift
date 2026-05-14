//
//  LiDARManager.swift
//  Komatsu
//
//  Orchestrate: Detection → Mesh Analysis → Surface Area
//  + Length Measurement (Apple Measure style)
//

import ARKit
import Combine
import RealityKit
import simd

// MARK: - Types

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

// MARK: - LiDAR Manager

class LiDARManager: NSObject, ObservableObject, ARSessionDelegate {
    
    let session = ARSession()
    var arView: ARView?
    
    // --- Detection ---
    @Published var mode: MeasureMode = .surface
    @Published var detectionState: DetectionState = .searching
    @Published var detectedBBox: CGRect = .zero            // normalized (0-1)
    @Published var meshSurfaceArea: Float = 0              // m²
    @Published var meshTriangleCount: Int = 0
    @Published var isSurfaceCaptured: Bool = false
    @Published var capturedArea: Float = 0                 // m² (frozen value)
    
    // --- Length ---
    @Published var measuredLength: Float = 0
    @Published var lengthPoints: [MeasurePoint] = []
    @Published var isMeasuringLength: Bool = false
    @Published var liveDistance: Float = 0
    
    // --- Volume ---
    @Published var volume: Float = 0
    
    // --- Status ---
    @Published var statusMessage: String = "ชี้กล้องไปที่บุ้งกี๋"
    @Published var isReady: Bool = false
    
    // --- Internal ---
    private let detector = BucketDetector()
    private let meshAnalyzer = MeshAnalyzer()
    private var updateTimer: Timer?
    
    // Length visuals
    private var lengthLineAnchor: AnchorEntity?
    private var liveLineAnchor: AnchorEntity?
    private var liveEndAnchor: AnchorEntity?
    
    override init() {
        super.init()
        session.delegate = self
    }
    
    // MARK: - Start Session
    
    func startSession() {
        let config = ARWorldTrackingConfiguration()
        config.planeDetection = [.horizontal, .vertical]
        config.environmentTexturing = .automatic
        
        // เปิด LiDAR mesh reconstruction (สำคัญ!)
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        
        // เปิด depth data สำหรับ detection
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
        }
        
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        
        // Timer: detection + live line update
        updateTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.runDetectionPipeline()
            self?.updateLiveLine()
        }
    }
    
    // MARK: - ARSessionDelegate
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        if !isReady {
            DispatchQueue.main.async { self.isReady = true }
        }
    }
    
    // MARK: - Detection Pipeline
    
    private func runDetectionPipeline() {
        guard mode == .surface, !isSurfaceCaptured else { return }
        guard let frame = session.currentFrame else { return }
        
        // ขั้น 1: ตรวจจับวัตถุด้วย Vision
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
            
            // ขั้น 2: คำนวณพื้นที่จาก mesh
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
    
    // MARK: - Capture Surface
    
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
    
    // MARK: - Length Measurement (Apple Measure style)
    
    func handleTap(at screenPoint: CGPoint) {
        guard mode == .length, let arView = arView else { return }
        let hits = arView.raycast(from: screenPoint, allowing: .estimatedPlane, alignment: .any)
        guard let hit = hits.first else { statusMessage = "ไม่เจอพื้นผิว"; return }
        
        let wp = SIMD3<Float>(hit.worldTransform.columns.3.x,
                               hit.worldTransform.columns.3.y,
                               hit.worldTransform.columns.3.z)
        
        if !isMeasuringLength {
            // กดครั้งแรก: จุดเริ่ม
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
            // กดครั้งที่ 2: จุดสิ้นสุด
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
    
    // MARK: - Live Line
    
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
    
    // MARK: - Volume
    
    private func recalcVolume() {
        let area = capturedArea > 0 ? capturedArea : meshSurfaceArea
        if area > 0 && measuredLength > 0 {
            volume = area * measuredLength
        }
    }
    
    // MARK: - Reset
    
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
    }
    
    // MARK: - Format
    
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

