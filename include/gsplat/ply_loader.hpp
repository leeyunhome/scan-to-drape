#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace gsplat {

// One instance's worth of GPU-ready data: world-space center, the upper
// triangle of the 3x3 world-space covariance matrix (cov = R * diag(s^2) * R^T,
// precomputed once at load time since the splat never deforms), and RGBA.
struct SplatInstance {
    float px, py, pz;
    float cov_xx, cov_xy, cov_xz;
    float cov_yy, cov_yz, cov_zz;
    float r, g, b, a;
};

struct Bounds {
    float minX, minY, minZ, maxX, maxY, maxZ;
    float centerX() const { return 0.5f * (minX + maxX); }
    float centerY() const { return 0.5f * (minY + maxY); }
    float centerZ() const { return 0.5f * (minZ + maxZ); }
    float radius() const {
        float dx = maxX - minX, dy = maxY - minY, dz = maxZ - minZ;
        return 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

struct SplatCloud {
    std::vector<SplatInstance> splats;
    Bounds bounds{};
};

namespace detail {

inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

// Quaternion (w, x, y, z) -> 3x3 rotation matrix, row-major in a flat array.
inline std::array<float, 9> quatToRotation(float w, float x, float y, float z) {
    float n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-12f) n = 1.0f;
    w /= n; x /= n; y /= n; z /= n;

    return {
        1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - w * z),       2.0f * (x * z + w * y),
        2.0f * (x * y + w * z),        1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - w * x),
        2.0f * (x * z - w * y),        2.0f * (y * z + w * x),       1.0f - 2.0f * (x * x + y * y),
    };
}

// R * diag(sx^2, sy^2, sz^2) * R^T, returned as the six unique entries.
inline void worldCovariance(const std::array<float, 9>& R, float sx, float sy, float sz,
                             float& xx, float& xy, float& xz, float& yy, float& yz, float& zz) {
    float sx2 = sx * sx, sy2 = sy * sy, sz2 = sz * sz;
    // M = R * diag(s^2)  (columns of R scaled by s^2)
    float M00 = R[0] * sx2, M01 = R[1] * sy2, M02 = R[2] * sz2;
    float M10 = R[3] * sx2, M11 = R[4] * sy2, M12 = R[5] * sz2;
    float M20 = R[6] * sx2, M21 = R[7] * sy2, M22 = R[8] * sz2;
    // cov = M * R^T
    xx = M00 * R[0] + M01 * R[1] + M02 * R[2];
    xy = M00 * R[3] + M01 * R[4] + M02 * R[5];
    xz = M00 * R[6] + M01 * R[7] + M02 * R[8];
    yy = M10 * R[3] + M11 * R[4] + M12 * R[5];
    yz = M10 * R[6] + M11 * R[7] + M12 * R[8];
    zz = M20 * R[6] + M21 * R[7] + M22 * R[8];
}

}  // namespace detail

// Loads the standard INRIA 3D Gaussian Splatting PLY export: binary little-endian,
// one `float` per property, property names looked up by name so the loader tolerates
// a different SH degree (different f_rest_* count) than the fixed layout used here.
inline SplatCloud loadGaussianPly(const std::string& path, float keepPercentile = 1.0f) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("could not open PLY file: " + path);

    std::string line;
    std::vector<std::string> propertyNames;
    size_t vertexCount = 0;
    bool binaryLittleEndian = false;

    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "end_header") break;

        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == "format") {
            std::string fmt;
            iss >> fmt;
            binaryLittleEndian = (fmt == "binary_little_endian");
        } else if (token == "element") {
            std::string what;
            iss >> what;
            if (what == "vertex") iss >> vertexCount;
        } else if (token == "property") {
            std::string type, name;
            iss >> type >> name;
            propertyNames.push_back(name);
        }
    }

    if (!binaryLittleEndian) throw std::runtime_error("only binary_little_endian PLY is supported: " + path);

    std::unordered_map<std::string, size_t> index;
    for (size_t i = 0; i < propertyNames.size(); ++i) index[propertyNames[i]] = i;

    auto require = [&](const std::string& name) -> size_t {
        auto it = index.find(name);
        if (it == index.end()) throw std::runtime_error("PLY missing property: " + name);
        return it->second;
    };

    const size_t stride = propertyNames.size();
    const size_t ix = require("x"), iy = require("y"), iz = require("z");
    const size_t idc0 = require("f_dc_0"), idc1 = require("f_dc_1"), idc2 = require("f_dc_2");
    const size_t iop = require("opacity");
    const size_t is0 = require("scale_0"), is1 = require("scale_1"), is2 = require("scale_2");
    const size_t ir0 = require("rot_0"), ir1 = require("rot_1"), ir2 = require("rot_2"), ir3 = require("rot_3");

    std::vector<float> row(stride);
    std::vector<SplatInstance> raw;
    std::vector<float> maxScale;
    raw.reserve(vertexCount);
    maxScale.reserve(vertexCount);

    constexpr float SH_C0 = 0.28209479177387814f;  // Y_0^0 basis value

    for (size_t i = 0; i < vertexCount; ++i) {
        file.read(reinterpret_cast<char*>(row.data()), static_cast<std::streamsize>(stride * sizeof(float)));
        if (!file) throw std::runtime_error("unexpected end of PLY data: " + path);

        float px = row[ix], py = row[iy], pz = row[iz];
        float sx = std::exp(row[is0]), sy = std::exp(row[is1]), sz = std::exp(row[is2]);
        float qw = row[ir0], qx = row[ir1], qy = row[ir2], qz = row[ir3];

        auto R = detail::quatToRotation(qw, qx, qy, qz);
        SplatInstance inst{};
        inst.px = px; inst.py = py; inst.pz = pz;
        detail::worldCovariance(R, sx, sy, sz, inst.cov_xx, inst.cov_xy, inst.cov_xz, inst.cov_yy, inst.cov_yz, inst.cov_zz);

        inst.r = std::clamp(0.5f + SH_C0 * row[idc0], 0.0f, 1.0f);
        inst.g = std::clamp(0.5f + SH_C0 * row[idc1], 0.0f, 1.0f);
        inst.b = std::clamp(0.5f + SH_C0 * row[idc2], 0.0f, 1.0f);
        inst.a = detail::sigmoid(row[iop]);

        raw.push_back(inst);
        maxScale.push_back(std::max({sx, sy, sz}));
    }

    // Untrained/under-constrained "floater" splats from sparse-view capture can end
    // up with wildly oversized scale (here: median ~0.04, but a long tail past 1.0,
    // vs. a ~28-unit object radius) and render as huge screen-space streaks. Drop the
    // top 0.5% by scale as a stand-in for interactive cleanup (e.g. Supersplat).
    SplatCloud cloud;
    if (!raw.empty()) {
        std::vector<float> sorted = maxScale;
        size_t cutoffIdx = std::min(static_cast<size_t>(sorted.size() * keepPercentile), sorted.size() - 1);
        std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(cutoffIdx), sorted.end());
        float cutoff = keepPercentile >= 1.0f ? std::numeric_limits<float>::max() : sorted[cutoffIdx];

        cloud.splats.reserve(raw.size());
        for (size_t i = 0; i < raw.size(); ++i) {
            if (maxScale[i] > cutoff) continue;
            cloud.splats.push_back(raw[i]);
        }
        std::cout << "ply_loader: dropped " << (raw.size() - cloud.splats.size())
                  << " oversized-scale splats (top " << (100.0f * (1.0f - keepPercentile))
                  << "%, cutoff=" << cutoff << ")\n";
    }

    Bounds b{1e30f, 1e30f, 1e30f, -1e30f, -1e30f, -1e30f};
    for (const auto& inst : cloud.splats) {
        b.minX = std::min(b.minX, inst.px); b.maxX = std::max(b.maxX, inst.px);
        b.minY = std::min(b.minY, inst.py); b.maxY = std::max(b.maxY, inst.py);
        b.minZ = std::min(b.minZ, inst.pz); b.maxZ = std::max(b.maxZ, inst.pz);
    }

    cloud.bounds = b;
    return cloud;
}

}  // namespace gsplat
