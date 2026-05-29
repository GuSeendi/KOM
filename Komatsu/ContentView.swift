import SwiftUI
import ARKit
import RealityKit

struct ContentView: View {
    @StateObject private var mgr = LiDARManager()
    @State private var showReset = false

    var body: some View {
        ZStack {

            ARMeasureView(manager: mgr).ignoresSafeArea()

            VStack(spacing: 0) {
                LinearGradient(colors: [.black.opacity(0.55), .clear], startPoint: .top, endPoint: .bottom)
                    .frame(height: 120)
                Spacer()
                LinearGradient(colors: [.clear, .black.opacity(0.75)], startPoint: .top, endPoint: .bottom)
                    .frame(height: 320)
            }
            .ignoresSafeArea()
            .allowsHitTesting(false)

            if mgr.mode == .surface && mgr.detectionState == .detected && !mgr.isSurfaceCaptured {
                GeometryReader { geo in
                    let box = mgr.detectedBBox
                    let rect = CGRect(
                        x: box.origin.x * geo.size.width,
                        y: box.origin.y * geo.size.height,
                        width: box.size.width * geo.size.width,
                        height: box.size.height * geo.size.height
                    )

                    RoundedRectangle(cornerRadius: 8)
                        .stroke(Color.green, style: StrokeStyle(lineWidth: 2, dash: [8, 4]))
                        .frame(width: rect.width, height: rect.height)
                        .position(x: rect.midX, y: rect.midY)

                    cornerMarkers(rect: rect, color: .green)

                    if mgr.meshSurfaceArea > 0 {
                        Text(mgr.fmtArea(mgr.meshSurfaceArea))
                            .font(.system(size: 14, weight: .bold, design: .monospaced))
                            .foregroundColor(.white)
                            .padding(.horizontal, 10)
                            .padding(.vertical, 5)
                            .background(Capsule().fill(.green.opacity(0.7)))
                            .position(x: rect.midX, y: rect.minY - 20)
                    }
                }
                .ignoresSafeArea()
                .allowsHitTesting(false)
            }

            if mgr.mode == .surface && mgr.isSurfaceCaptured {
                GeometryReader { geo in
                    let box = mgr.detectedBBox
                    let rect = CGRect(
                        x: box.origin.x * geo.size.width,
                        y: box.origin.y * geo.size.height,
                        width: box.size.width * geo.size.width,
                        height: box.size.height * geo.size.height
                    )

                    RoundedRectangle(cornerRadius: 8)
                        .stroke(Color.yellow, lineWidth: 2)
                        .frame(width: rect.width, height: rect.height)
                        .position(x: rect.midX, y: rect.midY)

                    cornerMarkers(rect: rect, color: .yellow)

                    Text("✓ \(mgr.fmtArea(mgr.capturedArea))")
                        .font(.system(size: 14, weight: .bold, design: .monospaced))
                        .foregroundColor(.black)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 5)
                        .background(Capsule().fill(.yellow))
                        .position(x: rect.midX, y: rect.minY - 20)
                }
                .ignoresSafeArea()
                .allowsHitTesting(false)
            }

            if mgr.mode == .length && mgr.isMeasuringLength {
                crosshair
            }

            if mgr.mode == .length && mgr.isMeasuringLength && mgr.liveDistance > 0 {
                Text(mgr.fmtLen(mgr.liveDistance))
                    .font(.system(size: 20, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
                    .padding(.horizontal, 14)
                    .padding(.vertical, 8)
                    .background(Capsule().fill(.cyan.opacity(0.7)))
                    .offset(y: -50)
            }

            VStack(spacing: 0) {
                header
                Spacer()
                resultPanel
                controls
            }
        }
        .alert("ล้างข้อมูลทั้งหมด?", isPresented: $showReset) {
            Button("ยกเลิก", role: .cancel) {}
            Button("ล้าง", role: .destructive) { mgr.resetAll() }
        }
    }

    private func cornerMarkers(rect: CGRect, color: Color) -> some View {
        let size: CGFloat = 16
        let width: CGFloat = 3

        return ZStack {

            cornerL(at: CGPoint(x: rect.minX, y: rect.minY), size: size, width: width, color: color, flipX: false, flipY: false)

            cornerL(at: CGPoint(x: rect.maxX, y: rect.minY), size: size, width: width, color: color, flipX: true, flipY: false)

            cornerL(at: CGPoint(x: rect.minX, y: rect.maxY), size: size, width: width, color: color, flipX: false, flipY: true)

            cornerL(at: CGPoint(x: rect.maxX, y: rect.maxY), size: size, width: width, color: color, flipX: true, flipY: true)
        }
    }

    private func cornerL(at pos: CGPoint, size: CGFloat, width: CGFloat, color: Color, flipX: Bool, flipY: Bool) -> some View {
        let xDir: CGFloat = flipX ? -1 : 1
        let yDir: CGFloat = flipY ? -1 : 1

        return ZStack {
            Rectangle()
                .fill(color)
                .frame(width: size, height: width)
                .position(x: pos.x + xDir * size / 2, y: pos.y)
            Rectangle()
                .fill(color)
                .frame(width: width, height: size)
                .position(x: pos.x, y: pos.y + yDir * size / 2)
        }
    }

    private var crosshair: some View {
        ZStack {
            Circle()
                .stroke(Color.cyan, lineWidth: 2)
                .frame(width: 50, height: 50)
            Rectangle().fill(Color.white.opacity(0.6)).frame(width: 1, height: 16)
            Rectangle().fill(Color.white.opacity(0.6)).frame(width: 16, height: 1)
        }
    }

    private var header: some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text("Komatsu Bucket")
                    .font(.system(size: 18, weight: .bold))
                    .foregroundColor(.white)
                Text("วัดพื้นที่ผิว & ปริมาตร")
                    .font(.system(size: 12)).foregroundColor(.white.opacity(0.6))
            }
            Spacer()

            HStack(spacing: 5) {
                Circle()
                    .fill(detectionColor)
                    .frame(width: 8, height: 8)
                Text(mgr.detectionState.rawValue)
                    .font(.system(size: 11, weight: .semibold))
            }
            .foregroundColor(.white)
            .padding(.horizontal, 10).padding(.vertical, 6)
            .background(detectionColor.opacity(0.2))
            .cornerRadius(8)
        }
        .padding(.horizontal, 20).padding(.top, 60)
    }

    private var detectionColor: Color {
        switch mgr.detectionState {
        case .searching: return .orange
        case .detected: return .green
        case .captured: return .yellow
        }
    }

    private var resultPanel: some View {
        VStack(spacing: 10) {

            HStack(spacing: 6) {
                Image(systemName: statusIcon)
                    .foregroundColor(statusColor).font(.system(size: 14))
                Text(mgr.statusMessage)
                    .font(.system(size: 13, weight: .medium)).foregroundColor(.white)
                    .lineLimit(2)
            }
            .padding(.horizontal, 16).padding(.vertical, 8)
            .background(Capsule().fill(.ultraThinMaterial))

            let area = mgr.isSurfaceCaptured ? mgr.capturedArea : mgr.meshSurfaceArea
            if area > 0 || mgr.measuredLength > 0 {
                VStack(spacing: 10) {
                    HStack(spacing: 12) {
                        tile(label: "พื้นที่ผิว (A)",
                             value: area > 0 ? mgr.fmtArea(area) : "—",
                             subtitle: mgr.meshTriangleCount > 0 ? "\(mgr.meshTriangleCount) triangles" : nil,
                             icon: "square.on.square", color: .yellow)
                        tile(label: "ความยาว (L)",
                             value: mgr.measuredLength > 0 ? mgr.fmtLen(mgr.measuredLength) : "—",
                             subtitle: nil,
                             icon: "ruler", color: .cyan)
                    }

                    if mgr.volume > 0 {
                        VStack(spacing: 4) {
                            Text("A × L = ปริมาตร")
                                .font(.system(size: 11, weight: .medium))
                                .foregroundColor(.white.opacity(0.5))
                            Text(mgr.fmtVol(mgr.volume))
                                .font(.system(size: 24, weight: .bold, design: .rounded))
                                .foregroundColor(.white)
                        }
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(RoundedRectangle(cornerRadius: 14)
                            .fill(LinearGradient(colors: [.green.opacity(0.3), .green.opacity(0.15)],
                                                  startPoint: .topLeading, endPoint: .bottomTrailing)))
                        .overlay(RoundedRectangle(cornerRadius: 14).stroke(.green.opacity(0.3)))
                    }
                }
                .padding(16)
                .background(RoundedRectangle(cornerRadius: 20).fill(.ultraThinMaterial))
                .padding(.horizontal, 20)
            }
        }
        .padding(.bottom, 12)
    }

    private func tile(label: String, value: String, subtitle: String?, icon: String, color: Color) -> some View {
        VStack(spacing: 4) {
            Image(systemName: icon).font(.system(size: 16)).foregroundColor(color)
            Text(value)
                .font(.system(size: 18, weight: .bold, design: .monospaced))
                .foregroundColor(.white).minimumScaleFactor(0.6).lineLimit(1)
            Text(label).font(.system(size: 11, weight: .medium)).foregroundColor(.white.opacity(0.5))
            if let sub = subtitle {
                Text(sub).font(.system(size: 9)).foregroundColor(.white.opacity(0.3))
            }
        }
        .frame(maxWidth: .infinity).padding(.vertical, 12)
        .background(RoundedRectangle(cornerRadius: 12).fill(color.opacity(0.1)))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(color.opacity(0.2)))
    }

    private var statusIcon: String {
        switch mgr.detectionState {
        case .searching: return "viewfinder"
        case .detected: return "viewfinder.rectangular"
        case .captured: return "checkmark.circle.fill"
        }
    }

    private var statusColor: Color {
        switch mgr.detectionState {
        case .searching: return .orange
        case .detected: return .green
        case .captured: return .yellow
        }
    }

    private var controls: some View {
        VStack(spacing: 12) {
            Picker("โหมด", selection: $mgr.mode) {
                ForEach(MeasureMode.allCases, id: \.self) { m in
                    Label(m.rawValue, systemImage: m == .surface ? "square.on.square" : "ruler").tag(m)
                }
            }
            .pickerStyle(.segmented).padding(.horizontal, 20)

            HStack(spacing: 12) {

                if mgr.mode == .surface {
                    Button {
                        if mgr.isSurfaceCaptured { mgr.resetSurface() }
                        else { mgr.captureSurface() }
                    } label: {
                        Label(mgr.isSurfaceCaptured ? "Scan ใหม่" : "จับพื้นผิว",
                              systemImage: mgr.isSurfaceCaptured ? "arrow.counterclockwise" : "camera.viewfinder")
                            .font(.system(size: 14, weight: .bold))
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity).padding(.vertical, 14)
                            .background(RoundedRectangle(cornerRadius: 14)
                                .fill(mgr.isSurfaceCaptured
                                      ? LinearGradient(colors: [.orange, .orange.opacity(0.8)], startPoint: .top, endPoint: .bottom)
                                      : LinearGradient(colors: [.green, .green.opacity(0.8)], startPoint: .top, endPoint: .bottom)))
                            .shadow(color: .green.opacity(0.3), radius: 8, y: 4)
                    }
                    .disabled(!mgr.isSurfaceCaptured && mgr.detectionState != .detected)
                }

                Button {
                    showReset = true
                } label: {
                    Label("ล้าง", systemImage: "trash")
                        .font(.system(size: 14, weight: .semibold))
                        .foregroundColor(.white)
                        .frame(maxWidth: mgr.mode == .surface ? nil : .infinity)
                        .padding(.vertical, 14).padding(.horizontal, 20)
                        .background(RoundedRectangle(cornerRadius: 14).fill(.red.opacity(0.3)))
                }
            }
            .padding(.horizontal, 20)

            if mgr.mode == .surface {
                VStack(spacing: 4) {
                    HStack {
                        Text("ระยะสแกน")
                            .font(.system(size: 11, weight: .medium))
                            .foregroundColor(.white.opacity(0.6))
                        Spacer()
                        Text(String(format: "%.1f m", mgr.scanFilter.maxDepth))
                            .font(.system(size: 11, weight: .semibold, design: .monospaced))
                            .foregroundColor(.white)
                    }
                    Slider(value: $mgr.scanFilter.maxDepth, in: 0.3...3.0, step: 0.1)
                        .accentColor(.cyan)
                }
                .padding(.horizontal, 20)

                HStack(spacing: 12) {
                    Button {
                        if mgr.isScanning {
                            mgr.stopSegmentationScan()
                        } else {
                            mgr.statusMessage = "แตะที่บุ้งกี๋ในภาพเพื่อเริ่มสแกน"
                        }
                    } label: {
                        Label(mgr.isScanning ? "หยุดสแกน" : "แตะวัตถุเพื่อสแกน",
                              systemImage: mgr.isScanning ? "stop.circle.fill" : "hand.tap.fill")
                            .font(.system(size: 13, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(RoundedRectangle(cornerRadius: 12)
                                .fill(mgr.isScanning ? Color.red.opacity(0.75)
                                                     : Color.blue.opacity(0.55)))
                    }

                    Button {
                        _ = mgr.exportPLY()
                    } label: {
                        Label("Export PLY", systemImage: "square.and.arrow.down")
                            .font(.system(size: 13, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(RoundedRectangle(cornerRadius: 12)
                                .fill(mgr.scannedPointCount > 0
                                      ? Color.purple.opacity(0.75) : Color.gray.opacity(0.4)))
                    }
                    .disabled(mgr.scannedPointCount == 0)
                }
                .padding(.horizontal, 20)

                if mgr.scannedPointCount > 0 {
                    Text("จุดที่สแกนได้: \(mgr.scannedPointCount)")
                        .font(.system(size: 12, weight: .medium, design: .monospaced))
                        .foregroundColor(.white.opacity(0.75))
                }
            }

        }
        .padding(.bottom, 36)
    }
}

struct ARMeasureView: UIViewRepresentable {
    let manager: LiDARManager

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        arView.session = manager.session
        manager.arView = arView
        manager.startSession()


        let tap = UITapGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.handleTap(_:)))
        arView.addGestureRecognizer(tap)
        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {}
    func makeCoordinator() -> Coordinator { Coordinator(manager: manager) }

    class Coordinator: NSObject {
        let manager: LiDARManager
        init(manager: LiDARManager) { self.manager = manager }

        @objc func handleTap(_ g: UITapGestureRecognizer) {
            guard let v = g.view as? ARView else { return }
            let pt = g.location(in: v)
            if manager.mode == .surface {

                if !manager.isScanning {
                    manager.startSegmentationScan(at: pt)
                }
            } else {
                manager.handleTap(at: pt)
            }
        }
    }
}

