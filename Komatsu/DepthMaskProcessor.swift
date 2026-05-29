import Vision
import ARKit
import CoreVideo

@available(iOS 17.0, *)
final class DepthMaskProcessor {

    private var maskBits: [UInt8] = []
    private var maskW: Int = 0
    private var maskH: Int = 0

    private var selectedInstance: UInt8 = 0

    private var tapNorm: CGPoint = .zero

    private let queue = DispatchQueue(label: "DepthMaskProcessor.lock")
    private var isProcessing = false

    func setTap(_ normalized: CGPoint) {
        queue.sync { tapNorm = normalized }
    }

    var hasMask: Bool { queue.sync { !maskBits.isEmpty } }

    func updateMask(pixelBuffer: CVPixelBuffer,
                    depthW: Int, depthH: Int,
                    completion: @escaping (Bool) -> Void) {
        let alreadyRunning = queue.sync { () -> Bool in
            if isProcessing { return true }
            isProcessing = true
            return false
        }
        if alreadyRunning { completion(false); return }

        let tap = queue.sync { tapNorm }

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { completion(false); return }
            defer { self.queue.sync { self.isProcessing = false } }

            autoreleasepool {
                let request = VNGenerateForegroundInstanceMaskRequest()
                let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer,
                                                    orientation: .right, options: [:])
                do {
                    try handler.perform([request])
                    guard let observation = request.results?.first else {
                        completion(false); return
                    }

                    let chosen = self.pickInstance(in: observation, at: tap) ?? observation.allInstances

                    let scaled = try observation.generateScaledMaskForImage(forInstances: chosen, from: handler)
                    self.ingest(maskBuffer: scaled, depthW: depthW, depthH: depthH)
                    completion(true)
                } catch {
                    completion(false)
                }
            }
        }
    }

    func isInside(depthU: Int, depthV: Int) -> Bool {
        return queue.sync {
            guard !maskBits.isEmpty,
                  depthU >= 0, depthU < maskW,
                  depthV >= 0, depthV < maskH else { return false }
            return maskBits[depthV * maskW + depthU] > 128
        }
    }

    func clear() {
        queue.sync {
            maskBits.removeAll(keepingCapacity: false)
            maskW = 0; maskH = 0; selectedInstance = 0
        }
    }

    private func pickInstance(in obs: VNInstanceMaskObservation, at tap: CGPoint) -> IndexSet? {
        let instanceMask = obs.instanceMask
        CVPixelBufferLockBaseAddress(instanceMask, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(instanceMask, .readOnly) }
        let w = CVPixelBufferGetWidth(instanceMask)
        let h = CVPixelBufferGetHeight(instanceMask)
        let stride = CVPixelBufferGetBytesPerRow(instanceMask)
        guard let base = CVPixelBufferGetBaseAddress(instanceMask) else { return nil }
        let ptr = base.assumingMemoryBound(to: UInt8.self)
        let tx = min(w - 1, max(0, Int(tap.x * CGFloat(w))))
        let ty = min(h - 1, max(0, Int(tap.y * CGFloat(h))))
        let id = ptr[ty * stride + tx]
        guard id > 0 else { return nil }
        var set = IndexSet(); set.insert(Int(id))
        return set
    }

    private func ingest(maskBuffer: CVPixelBuffer, depthW: Int, depthH: Int) {
        guard depthW > 0, depthH > 0 else { return }
        CVPixelBufferLockBaseAddress(maskBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(maskBuffer, .readOnly) }
        let srcW = CVPixelBufferGetWidth(maskBuffer)
        let srcH = CVPixelBufferGetHeight(maskBuffer)
        let srcStride = CVPixelBufferGetBytesPerRow(maskBuffer)
        guard let base = CVPixelBufferGetBaseAddress(maskBuffer) else { return }
        let src = base.assumingMemoryBound(to: UInt8.self)

        var bits = [UInt8](repeating: 0, count: depthW * depthH)

        var sxForDv = [Int](repeating: 0, count: depthH)
        for dv in 0..<depthH {
            let pnx = 1.0 - Double(dv) / Double(depthH)
            sxForDv[dv] = min(srcW - 1, max(0, Int(pnx * Double(srcW))))
        }
        for du in 0..<depthW {
            let pny = Double(du) / Double(depthW)
            let sy = min(srcH - 1, max(0, Int(pny * Double(srcH))))
            let srcRow = sy * srcStride
            for dv in 0..<depthH {
                bits[dv * depthW + du] = src[srcRow + sxForDv[dv]]
            }
        }

        queue.sync {
            self.maskBits = bits
            self.maskW = depthW
            self.maskH = depthH
        }
    }
}

