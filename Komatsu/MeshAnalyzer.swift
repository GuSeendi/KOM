//
//  MeshAnalyzer.swift
//  Komatsu
//
//  คำนวณพื้นที่ผิวจาก ARKit mesh anchors
//  กรองเฉพาะ triangles ที่อยู่ใน bounding box ของวัตถุที่ตรวจจับ
//

import ARKit
import simd

class MeshAnalyzer {
    
    struct MeshResult {
        let surfaceArea: Float    // m²
        let triangleCount: Int
        let vertexCount: Int
    }
    
    // MARK: - Calculate Surface Area from Mesh Anchors
    
    /// คำนวณพื้นที่ผิวจาก mesh anchors ที่อยู่ใน bounding box
    func calculateArea(
        meshAnchors: [ARMeshAnchor],
        camera: ARCamera,
        detectionBox: CGRect,       // normalized screen coords (0-1)
        viewportSize: CGSize
    ) -> MeshResult {
        
        var totalArea: Float = 0
        var totalTriangles = 0
        var totalVertices = 0
        
        for anchor in meshAnchors {
            let geo = anchor.geometry
            let anchorTransform = anchor.transform
            
            totalVertices += geo.vertices.count
            
            for faceIdx in 0..<geo.faces.count {
                // อ่าน vertex indices ของ triangle
                let indices = faceIndices(geo: geo, at: faceIdx)
                
                // อ่าน vertex positions (local space)
                let v1Local = vertexPosition(geo: geo, at: indices.0)
                let v2Local = vertexPosition(geo: geo, at: indices.1)
                let v3Local = vertexPosition(geo: geo, at: indices.2)
                
                // แปลง local → world space
                let v1World = anchorTransform * SIMD4<Float>(v1Local.x, v1Local.y, v1Local.z, 1)
                let v2World = anchorTransform * SIMD4<Float>(v2Local.x, v2Local.y, v2Local.z, 1)
                let v3World = anchorTransform * SIMD4<Float>(v3Local.x, v3Local.y, v3Local.z, 1)
                
                let w1 = SIMD3<Float>(v1World.x, v1World.y, v1World.z)
                let w2 = SIMD3<Float>(v2World.x, v2World.y, v2World.z)
                let w3 = SIMD3<Float>(v3World.x, v3World.y, v3World.z)
                
                // Project center of triangle ลงหน้าจอ
                let center = (w1 + w2 + w3) / 3.0
                let screenPoint = camera.projectPoint(
                    center,
                    orientation: .portrait,
                    viewportSize: viewportSize
                )
                
                // Normalize screen point (0-1)
                let normalizedX = screenPoint.x / viewportSize.width
                let normalizedY = screenPoint.y / viewportSize.height
                
                // ตรวจว่า triangle อยู่ใน detection bounding box
                if detectionBox.contains(CGPoint(x: normalizedX, y: normalizedY)) {
                    // คำนวณพื้นที่ triangle = ½ × |AB × AC|
                    let edge1 = w2 - w1
                    let edge2 = w3 - w1
                    let crossProduct = cross(edge1, edge2)
                    let area = 0.5 * length(crossProduct)
                    
                    totalArea += area
                    totalTriangles += 1
                }
            }
        }
        
        return MeshResult(
            surfaceArea: totalArea,
            triangleCount: totalTriangles,
            vertexCount: totalVertices
        )
    }
    
    // MARK: - Mesh Data Helpers
    
    /// อ่าน face indices (3 ค่า) จาก ARMeshGeometry
    private func faceIndices(geo: ARMeshGeometry, at index: Int) -> (UInt32, UInt32, UInt32) {
        let facesBuffer = geo.faces.buffer
        let bytesPerIndex = geo.faces.bytesPerIndex
        let indexCountPerFace = geo.faces.indexCountPerPrimitive
        let offset = index * indexCountPerFace * bytesPerIndex
        
        let ptr = facesBuffer.contents().advanced(by: offset)
        
        if bytesPerIndex == 4 {
            let indices = ptr.assumingMemoryBound(to: UInt32.self)
            return (indices[0], indices[1], indices[2])
        } else {
            let indices = ptr.assumingMemoryBound(to: UInt16.self)
            return (UInt32(indices[0]), UInt32(indices[1]), UInt32(indices[2]))
        }
    }
    
    /// อ่าน vertex position จาก ARMeshGeometry
    private func vertexPosition(geo: ARMeshGeometry, at index: UInt32) -> SIMD3<Float> {
        let vertexBuffer = geo.vertices.buffer
        let stride = geo.vertices.stride
        let vertexOffset = Int(index) * stride
        
        let ptr = vertexBuffer.contents().advanced(by: vertexOffset)
        return ptr.assumingMemoryBound(to: SIMD3<Float>.self).pointee
    }
}
