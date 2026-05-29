import Vision
import ARKit
import CoreImage
import CoreML

class BucketDetector {

    struct DetectionResult {
        let boundingBox: CGRect
        let mask: CVPixelBuffer?
        let confidence: Float
    }

    private var isProcessing = false
    private var vnModel: VNCoreMLModel?

    // Class indices from data.yaml that count as "bucket"
    private let bucketClassLabels: Set<String> = ["bucket", "Mobile-plant-bucket"]

    private let confidenceThreshold: Float = 0.4

    init() {
        loadModel()
    }

    private func loadModel() {
        do {
            let config = MLModelConfiguration()
            let mlModel = try ExcavatorDetector(configuration: config).model
            self.vnModel = try VNCoreMLModel(for: mlModel)
        } catch {
            print("Failed to load ExcavatorDetector model: \(error)")
            self.vnModel = nil
        }
    }

    func detect(frame: ARFrame, completion: @escaping (DetectionResult?) -> Void) {
        guard !isProcessing else { completion(nil); return }
        isProcessing = true

        let pixelBuffer = frame.capturedImage

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            defer { self?.isProcessing = false }

            if self?.vnModel != nil {
                self?.detectWithCoreML(pixelBuffer: pixelBuffer, completion: completion)
            } else {
                self?.detectWithSaliency(pixelBuffer: pixelBuffer, completion: completion)
            }
        }
    }

    private func detectWithCoreML(pixelBuffer: CVPixelBuffer, completion: @escaping (DetectionResult?) -> Void) {
        guard let vnModel = vnModel else { completion(nil); return }

        let request = VNCoreMLRequest(model: vnModel) { [weak self] request, error in
            guard let self = self else { completion(nil); return }

            if let error = error {
                print("CoreML request error: \(error)")
                completion(nil)
                return
            }

            guard let observations = request.results as? [VNRecognizedObjectObservation], !observations.isEmpty else {
                completion(nil)
                return
            }

            let buckets = observations.compactMap { obs -> (CGRect, Float)? in
                guard let top = obs.labels.first else { return nil }
                guard self.bucketClassLabels.contains(top.identifier) else { return nil }
                guard top.confidence >= self.confidenceThreshold else { return nil }
                return (obs.boundingBox, top.confidence)
            }

            guard let best = buckets.max(by: { $0.1 < $1.1 }) else {
                completion(nil)
                return
            }

            // Vision bbox origin = bottom-left; ARKit/UIKit screen = top-left → flip Y
            let v = best.0
            let flipped = CGRect(
                x: v.minX,
                y: 1.0 - v.maxY,
                width: v.width,
                height: v.height
            )

            completion(DetectionResult(
                boundingBox: flipped,
                mask: nil,
                confidence: best.1
            ))
        }

        request.imageCropAndScaleOption = .scaleFill

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        do {
            try handler.perform([request])
        } catch {
            print("VNImageRequestHandler failed: \(error)")
            completion(nil)
        }
    }

    private func detectWithSaliency(pixelBuffer: CVPixelBuffer, completion: @escaping (DetectionResult?) -> Void) {
        let request = VNGenerateObjectnessBasedSaliencyImageRequest()
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])

        do {
            try handler.perform([request])

            guard let result = request.results?.first as? VNSaliencyImageObservation,
                  let salientObject = result.salientObjects?.first
            else {
                completion(nil)
                return
            }

            completion(DetectionResult(
                boundingBox: salientObject.boundingBox,
                mask: nil,
                confidence: salientObject.confidence
            ))
        } catch {
            print("Saliency detection error: \(error)")
            completion(nil)
        }
    }
}
