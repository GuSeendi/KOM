import ARKit

struct ScanFilter {
    var minDepth: Float = 0.3
    var maxDepth: Float = 3.0
    var requireYellow: Bool = true
    var minConfidence: ARConfidenceLevel = .medium

    func passes(depth: Float, conf: UInt8, isYellow: Bool) -> Bool {
        guard depth >= minDepth, depth <= maxDepth else { return false }
        guard conf >= UInt8(minConfidence.rawValue) else { return false }
        if requireYellow { guard isYellow else { return false } }
        return true
    }
}
