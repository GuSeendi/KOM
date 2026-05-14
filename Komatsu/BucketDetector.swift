//
//  BucketDetector.swift
//  Komatsu
//
//  ตรวจจับวัตถุหลัก (บุ้งกี๋) ด้วย Vision framework
//  ใช้ VNGenerateForegroundInstanceMaskRequest (iOS 17+)
//

import Vision
import ARKit
import CoreImage
import UIKit

class BucketDetector {
    
    // ผลลัพธ์
    struct DetectionResult {
        let boundingBox: CGRect       // normalized (0-1)
        let mask: CVPixelBuffer?      // foreground mask
        let confidence: Float         // ความมั่นใจ 0-1
    }
    
    private var isProcessing = false
    
    // MARK: - Detect Foreground Object
    
    /// ตรวจจับวัตถุหลักจาก ARFrame
    func detect(frame: ARFrame, completion: @escaping (DetectionResult?) -> Void) {
        guard !isProcessing else { completion(nil); return }
        isProcessing = true
        
        let pixelBuffer = frame.capturedImage
        
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            defer { self?.isProcessing = false }
            
            if #available(iOS 17.0, *) {
                self?.detectWithInstanceMask(pixelBuffer: pixelBuffer, completion: completion)
            } else {
                // Fallback: ใช้ saliency detection (iOS 13+)
                self?.detectWithSaliency(pixelBuffer: pixelBuffer, completion: completion)
            }
        }
    }
    
    // MARK: - iOS 17+ Foreground Instance Mask
    
    @available(iOS 17.0, *)
    private func detectWithInstanceMask(pixelBuffer: CVPixelBuffer, completion: @escaping (DetectionResult?) -> Void) {
        let request = VNGenerateForegroundInstanceMaskRequest()
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .right, options: [:])
        
        do {
            try handler.perform([request])
            
            guard let result = request.results?.first else {
                completion(nil)
                return
            }
            
            // สร้าง mask สำหรับทุก instance ที่ตรวจพบ
            let maskBuffer = try result.generateScaledMaskForImage(
                forInstances: result.allInstances,
                from: handler
            )
            
            // หา bounding box จาก mask
            let bbox = boundingBoxFromMask(maskBuffer)
            
            let detection = DetectionResult(
                boundingBox: bbox,
                mask: maskBuffer,
                confidence: bbox.width > 0.05 ? 0.9 : 0.3
            )
            
            completion(detection)
            
        } catch {
            print("Vision detection error: \(error)")
            completion(nil)
        }
    }
    
    // MARK: - Fallback: Saliency Detection (iOS 13+)
    
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
            
            let detection = DetectionResult(
                boundingBox: salientObject.boundingBox,
                mask: nil,
                confidence: salientObject.confidence
            )
            
            completion(detection)
            
        } catch {
            print("Saliency detection error: \(error)")
            completion(nil)
        }
    }
    
    // MARK: - Helpers
    
    /// หา bounding box ของ non-zero pixels ใน mask
    private func boundingBoxFromMask(_ mask: CVPixelBuffer) -> CGRect {
        CVPixelBufferLockBaseAddress(mask, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(mask, .readOnly) }
        
        let width = CVPixelBufferGetWidth(mask)
        let height = CVPixelBufferGetHeight(mask)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(mask)
        
        guard let baseAddress = CVPixelBufferGetBaseAddress(mask) else {
            return .zero
        }
        
        let buffer = baseAddress.assumingMemoryBound(to: UInt8.self)
        
        var minX = width, minY = height, maxX = 0, maxY = 0
        let step = 4 // Sample ทุก 4 pixel เพื่อ performance
        
        for y in stride(from: 0, to: height, by: step) {
            for x in stride(from: 0, to: width, by: step) {
                let pixel = buffer[y * bytesPerRow + x]
                if pixel > 128 { // threshold
                    minX = min(minX, x)
                    minY = min(minY, y)
                    maxX = max(maxX, x)
                    maxY = max(maxY, y)
                }
            }
        }
        
        guard maxX > minX, maxY > minY else { return .zero }
        
        // Normalize to 0-1
        return CGRect(
            x: CGFloat(minX) / CGFloat(width),
            y: CGFloat(minY) / CGFloat(height),
            width: CGFloat(maxX - minX) / CGFloat(width),
            height: CGFloat(maxY - minY) / CGFloat(height)
        )
    }
}
