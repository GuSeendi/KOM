import ARKit
import Accelerate
import CoreVideo

enum YellowMaskProcessor {

    // H=18–35 (normalised 0–1 from 0–179 range), S=80–255, V=80–255
    private static let hMin: Float = 18.0 / 179.0
    private static let hMax: Float = 35.0 / 179.0
    private static let sMin: Float = 80.0 / 255.0
    private static let vMin: Float = 80.0 / 255.0

    /// Returns an 8-bit single-channel CVPixelBuffer at depth-map resolution
    /// (depthW × depthH) where 255 = yellow pixel, 0 = not yellow.
    static func createYellowMask(frame: ARFrame, depthW: Int, depthH: Int) -> CVPixelBuffer? {
        let image = frame.capturedImage
        let imgW = CVPixelBufferGetWidth(image)
        let imgH = CVPixelBufferGetHeight(image)
        guard imgW > 0, imgH > 0, depthW > 0, depthH > 0 else { return nil }

        CVPixelBufferLockBaseAddress(image, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(image, .readOnly) }

        guard let yBase = CVPixelBufferGetBaseAddressOfPlane(image, 0),
              let cBase = CVPixelBufferGetBaseAddressOfPlane(image, 1) else { return nil }

        let yPtr = yBase.assumingMemoryBound(to: UInt8.self)
        let cPtr = cBase.assumingMemoryBound(to: UInt8.self)
        let yRow = CVPixelBufferGetBytesPerRowOfPlane(image, 0)
        let cRow = CVPixelBufferGetBytesPerRowOfPlane(image, 1)

        var fullMask = [UInt8](repeating: 0, count: imgW * imgH)

        autoreleasepool {
            for row in 0..<imgH {
                for col in 0..<imgW {
                    let y  = Float(yPtr[row * yRow + col])
                    let cb = Float(cPtr[(row >> 1) * cRow + ((col >> 1) << 1)])
                    let cr = Float(cPtr[(row >> 1) * cRow + ((col >> 1) << 1) + 1])

                    let r = max(0, min(255, y + 1.402  * (cr - 128))) / 255.0
                    let g = max(0, min(255, y - 0.344  * (cb - 128) - 0.714 * (cr - 128))) / 255.0
                    let b = max(0, min(255, y + 1.772  * (cb - 128))) / 255.0

                    let (h, s, v) = rgbToHSV(r, g, b)
                    if h >= hMin && h <= hMax && s >= sMin && v >= vMin {
                        fullMask[row * imgW + col] = 255
                    }
                }
            }
        }

        return scaleMask(&fullMask, srcW: imgW, srcH: imgH, dstW: depthW, dstH: depthH)
    }

    private static func rgbToHSV(_ r: Float, _ g: Float, _ b: Float) -> (h: Float, s: Float, v: Float) {
        let cMax = max(r, max(g, b))
        let cMin = min(r, min(g, b))
        let delta = cMax - cMin
        let v = cMax
        let s = cMax > 0 ? delta / cMax : 0
        var h: Float = 0
        if delta > 0 {
            if cMax == r      { h = (g - b) / delta }
            else if cMax == g { h = 2 + (b - r) / delta }
            else              { h = 4 + (r - g) / delta }
            h /= 6
            if h < 0 { h += 1 }
        }
        return (h, s, v)
    }

    private static func scaleMask(_ src: inout [UInt8], srcW: Int, srcH: Int,
                                   dstW: Int, dstH: Int) -> CVPixelBuffer? {
        return src.withUnsafeMutableBytes { srcRaw in
            var srcBuffer = vImage_Buffer(
                data: srcRaw.baseAddress!,
                height: vImagePixelCount(srcH),
                width:  vImagePixelCount(srcW),
                rowBytes: srcW
            )

            var dstData = [UInt8](repeating: 0, count: dstW * dstH)
            let result: CVPixelBuffer? = dstData.withUnsafeMutableBytes { dstRaw in
                var dstBuffer = vImage_Buffer(
                    data: dstRaw.baseAddress!,
                    height: vImagePixelCount(dstH),
                    width:  vImagePixelCount(dstW),
                    rowBytes: dstW
                )
                let err = vImageScale_Planar8(&srcBuffer, &dstBuffer, nil,
                                              vImage_Flags(kvImageHighQualityResampling))
                guard err == kvImageNoError else { return nil }
                return makeCVPixelBuffer(from: dstRaw, w: dstW, h: dstH, rowBytes: dstW)
            }
            return result
        }
    }

    private static func makeCVPixelBuffer(from raw: UnsafeMutableRawBufferPointer,
                                           w: Int, h: Int, rowBytes: Int) -> CVPixelBuffer? {
        var pb: CVPixelBuffer?
        let attrs: [String: Any] = [
            kCVPixelBufferCGImageCompatibilityKey as String: false,
            kCVPixelBufferCGBitmapContextCompatibilityKey as String: false,
        ]
        guard CVPixelBufferCreate(kCFAllocatorDefault, w, h,
                                   kCVPixelFormatType_OneComponent8,
                                   attrs as CFDictionary, &pb) == kCVReturnSuccess,
              let pixelBuffer = pb else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, [])
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, []) }
        guard let base = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }
        let pbRowBytes = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let src = raw.baseAddress!.assumingMemoryBound(to: UInt8.self)
        let dst = base.assumingMemoryBound(to: UInt8.self)
        for row in 0..<h {
            (dst + row * pbRowBytes).update(from: src + row * rowBytes, count: w)
        }
        return pixelBuffer
    }
}
