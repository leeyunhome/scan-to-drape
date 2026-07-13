#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace gsplat {

struct MeshVertex {
    float px, py, pz;
    float nx, ny, nz;
};

struct Mesh {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
};

// Lat/long UV sphere, radius 1 centered at the origin (translate via the model matrix).
inline Mesh makeUvSphere(int latSegments = 24, int lonSegments = 36) {
    Mesh mesh;
    for (int lat = 0; lat <= latSegments; ++lat) {
        float theta = static_cast<float>(lat) / latSegments * 3.14159265f;  // 0..pi
        float y = std::cos(theta);
        float ringRadius = std::sin(theta);
        for (int lon = 0; lon <= lonSegments; ++lon) {
            float phi = static_cast<float>(lon) / lonSegments * 2.0f * 3.14159265f;  // 0..2pi
            float x = ringRadius * std::cos(phi);
            float z = ringRadius * std::sin(phi);
            mesh.vertices.push_back({x, y, z, x, y, z});
        }
    }
    int stride = lonSegments + 1;
    for (int lat = 0; lat < latSegments; ++lat) {
        for (int lon = 0; lon < lonSegments; ++lon) {
            uint32_t a = static_cast<uint32_t>(lat * stride + lon);
            uint32_t b = static_cast<uint32_t>(a + stride);
            mesh.indices.push_back(a);
            mesh.indices.push_back(b);
            mesh.indices.push_back(a + 1);
            mesh.indices.push_back(a + 1);
            mesh.indices.push_back(b);
            mesh.indices.push_back(b + 1);
        }
    }
    return mesh;
}

}  // namespace gsplat
