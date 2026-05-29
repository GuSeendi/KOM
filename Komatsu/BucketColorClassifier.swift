import simd
import CoreVideo
import Foundation

final class BucketColorClassifier {

    var yMin: UInt8   = 90
    var cbMax: UInt8  = 110
    var crMin: UInt8  = 140
    var crMax: UInt8  = 210

    @inline(__always)
    func isYellow(y: UInt8, cb: UInt8, cr: UInt8) -> Bool {
        return y >= yMin && cb <= cbMax && cr >= crMin && cr <= crMax
    }

    var depthBand: Float = 0.4

    @inline(__always)
    func isWithinDepth(_ d: Float, reference: Float) -> Bool {
        return abs(d - reference) <= depthBand
    }

    var ransacIterations: Int = 60
    var ransacInlierDist: Float = 0.015
    var ransacMinInlierRatio: Float = 0.3

    struct PlaneFit {
        let normal: SIMD3<Float>
        let d: Float
        let inlierIndices: [Int]
    }

    func ransacPlane(points: [SIMD3<Float>]) -> PlaneFit? {
        let n = points.count
        guard n >= 30 else { return nil }

        var bestCount = 0
        var bestNormal = SIMD3<Float>(0, 1, 0)
        var bestD: Float = 0

        for _ in 0..<ransacIterations {
            let i0 = Int.random(in: 0..<n)
            let i1 = Int.random(in: 0..<n)
            let i2 = Int.random(in: 0..<n)
            if i0 == i1 || i1 == i2 || i0 == i2 { continue }
            let p0 = points[i0], p1 = points[i1], p2 = points[i2]
            let v1 = p1 - p0
            let v2 = p2 - p0
            let cross = simd_cross(v1, v2)
            let len = simd_length(cross)
            if len < 1e-6 { continue }
            let normal = cross / len
            let d = -simd_dot(normal, p0)

            var count = 0
            for p in points {
                if abs(simd_dot(normal, p) + d) < ransacInlierDist { count += 1 }
            }
            if count > bestCount {
                bestCount = count
                bestNormal = normal
                bestD = d
            }
        }

        guard Float(bestCount) / Float(n) >= ransacMinInlierRatio else { return nil }

        var inliers = [Int]()
        inliers.reserveCapacity(bestCount)
        for i in 0..<n {
            if abs(simd_dot(bestNormal, points[i]) + bestD) < ransacInlierDist {
                inliers.append(i)
            }
        }
        return PlaneFit(normal: bestNormal, d: bestD, inlierIndices: inliers)
    }
}

