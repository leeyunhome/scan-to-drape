// cloth.hpp — a small, dependency-free cloth simulation core (C++17, header-only)
//
// Vendored from https://github.com/leeyunhome/cloth-simulation (cpp/include/cloth/cloth.hpp),
// same author, unmodified except for one addition: `teleport()`, needed to reposition a cloth
// grid as a rigid whole (the constructor only lays it out flat in the XY plane) without a wrong
// first-step velocity — see the method for why `setPos()` alone isn't enough for that.
//
// Model
//   Mass–spring grid integrated with position Verlet, constraints enforced by
//   iterative relaxation (Jakobsen, "Advanced Character Physics", GDC 2001).
//
// Why Verlet + relaxation instead of explicit-Euler force springs?
//   Cloth needs very stiff structural springs. With explicit Euler the stable
//   timestep shrinks as stiffness grows (k·dt² bound) and the sim explodes.
//   Position Verlet stores no explicit velocity, so projecting positions onto
//   constraints is unconditionally stable: each relaxation pass moves both
//   endpoints toward the rest length, weighted by inverse mass. Stiffness is
//   then controlled by the *iteration count*, not by a spring constant.
//
// This header is the reference implementation for the WebGL demo — the demo
// ports exactly this logic to JavaScript. Unit tests live in tests/.

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace cloth {

// ── Minimal 3D vector ───────────────────────────────────────────────────────
struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3  operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3  operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3  operator*(float s)       const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    float dot(const Vec3& o)   const { return x * o.x + y * o.y + z * o.z; }
    Vec3  cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float length()  const { return std::sqrt(dot(*this)); }
    Vec3  normalized() const {
        const float len = length();
        return len > 1e-8f ? (*this) * (1.f / len) : Vec3{};
    }
};

// ── Constraint between two particles ────────────────────────────────────────
enum class SpringType : std::uint8_t {
    Structural,  // grid neighbors — resists stretch
    Shear,       // diagonals     — resists shearing
    Bend         // skip-one      — resists folding
};

struct Constraint {
    int        a, b;         // particle indices
    float      rest;         // rest length
    SpringType type;
    bool       alive = true; // false once torn
};

struct Params {
    Vec3  gravity     {0.f, -3.2f, 0.f};
    float damping     = 0.985f; // velocity kept per step (1 = no damping)
    int   iterations  = 4;      // relaxation passes per step
    float tearFactor  = 0.f;    // tear when len > rest*factor (0 = never)
};

// ── Cloth grid ──────────────────────────────────────────────────────────────
class Cloth {
public:
    // A cols×rows particle grid spanning width×height, top edge at y = 0,
    // laid out in the XY plane (z = 0), row-major from the top-left.
    Cloth(int cols, int rows, float width, float height)
        : cols_(cols), rows_(rows) {
        const int n = cols * rows;
        pos_.resize(n);
        prev_.resize(n);
        acc_.assign(n, Vec3{});
        invMass_.assign(n, 1.f);

        // Guard the degenerate 1×N / N×1 grids: without this, height/(rows-1)
        // divides by zero and the resulting inf turns every position into NaN
        // (0 * inf). Caught by the free-fall unit test.
        const float dx = cols > 1 ? width  / static_cast<float>(cols - 1) : 0.f;
        const float dy = rows > 1 ? height / static_cast<float>(rows - 1) : 0.f;
        for (int yv = 0; yv < rows; ++yv)
            for (int xv = 0; xv < cols; ++xv) {
                pos_[indexOf(xv, yv)] = {xv * dx - width * 0.5f, -yv * dy, 0.f};
            }
        prev_ = pos_;

        auto link = [&](int ax, int ay, int bx, int by, SpringType t) {
            const int ia = indexOf(ax, ay), ib = indexOf(bx, by);
            cons_.push_back({ia, ib, (pos_[ib] - pos_[ia]).length(), t, true});
        };
        for (int yv = 0; yv < rows; ++yv)
            for (int xv = 0; xv < cols; ++xv) {
                if (xv + 1 < cols) link(xv, yv, xv + 1, yv, SpringType::Structural);
                if (yv + 1 < rows) link(xv, yv, xv, yv + 1, SpringType::Structural);
                if (xv + 1 < cols && yv + 1 < rows) {
                    link(xv, yv, xv + 1, yv + 1, SpringType::Shear);
                    link(xv + 1, yv, xv, yv + 1, SpringType::Shear);
                }
                if (xv + 2 < cols) link(xv, yv, xv + 2, yv, SpringType::Bend);
                if (yv + 2 < rows) link(xv, yv, xv, yv + 2, SpringType::Bend);
            }
    }

    // ── Topology ────────────────────────────────────────────────────────────
    int indexOf(int x, int y) const { return y * cols_ + x; }
    int cols() const { return cols_; }
    int rows() const { return rows_; }
    int particleCount() const { return static_cast<int>(pos_.size()); }

    // Two triangles per grid cell, CCW as seen from +z.
    std::vector<int> triangles() const {
        std::vector<int> tri;
        tri.reserve((cols_ - 1) * (rows_ - 1) * 6);
        for (int yv = 0; yv + 1 < rows_; ++yv)
            for (int xv = 0; xv + 1 < cols_; ++xv) {
                const int i0 = indexOf(xv, yv),     i1 = indexOf(xv + 1, yv);
                const int i2 = indexOf(xv, yv + 1), i3 = indexOf(xv + 1, yv + 1);
                tri.insert(tri.end(), {i0, i2, i1,  i1, i2, i3});
            }
        return tri;
    }

    // ── Pinning ─────────────────────────────────────────────────────────────
    void pin(int idx)   { invMass_[idx] = 0.f; prev_[idx] = pos_[idx]; }
    void unpin(int idx) { invMass_[idx] = 1.f; }
    bool pinned(int idx) const { return invMass_[idx] == 0.f; }

    // ── Forces ──────────────────────────────────────────────────────────────
    void addAcceleration(int idx, const Vec3& a) { acc_[idx] += a; }

    // Aerodynamic wind: for each triangle, the force is along the surface
    // normal, proportional to how squarely the wind hits it (n · w), spread
    // over the three vertices. This is what makes cloth billow instead of
    // translating rigidly.
    void applyWind(const Vec3& wind) {
        const auto tri = triangles();
        for (std::size_t t = 0; t < tri.size(); t += 3) {
            const Vec3& p0 = pos_[tri[t]];
            const Vec3& p1 = pos_[tri[t + 1]];
            const Vec3& p2 = pos_[tri[t + 2]];
            const Vec3 n = (p1 - p0).cross(p2 - p0).normalized();
            const Vec3 f = n * n.dot(wind);
            for (int k = 0; k < 3; ++k) acc_[tri[t + k]] += f;
        }
    }

    // ── Simulation step: Verlet integrate, then relax constraints ──────────
    void step(float dt) {
        const float dt2 = dt * dt;
        for (std::size_t i = 0; i < pos_.size(); ++i) {
            if (invMass_[i] == 0.f) { acc_[i] = Vec3{}; continue; }
            const Vec3 vel  = (pos_[i] - prev_[i]) * params.damping;
            const Vec3 next = pos_[i] + vel + (params.gravity + acc_[i]) * dt2;
            prev_[i] = pos_[i];
            pos_[i]  = next;
            acc_[i]  = Vec3{};
        }
        satisfy();
    }

    // Tearing is decided on the pre-relaxation state, in its own pass.
    // Checking inside the relaxation loop is order-dependent (Gauss–Seidel):
    // a tear-exempt bend constraint that happens to run first can restore the
    // length before the structural spring ever sees the overstretch — a bug
    // the tearing unit test caught.
    void tearOverstretched() {
        if (params.tearFactor <= 0.f) return;
        for (auto& c : cons_) {
            if (!c.alive || c.type == SpringType::Bend) continue;
            const float len = (pos_[c.b] - pos_[c.a]).length();
            if (len > c.rest * params.tearFactor) c.alive = false;
        }
    }

    void satisfy() {
        tearOverstretched();
        for (int it = 0; it < params.iterations; ++it) {
            for (auto& c : cons_) {
                if (!c.alive) continue;
                const Vec3  d   = pos_[c.b] - pos_[c.a];
                const float len = d.length();
                if (len < 1e-8f) continue;
                const float wa = invMass_[c.a], wb = invMass_[c.b];
                const float wsum = wa + wb;
                if (wsum == 0.f) continue;
                const Vec3 corr = d * ((len - c.rest) / (len * wsum));
                pos_[c.a] += corr * wa;
                pos_[c.b] -= corr * wb;
            }
        }
    }

    // Project any penetrating particle back onto the sphere surface. `friction` in
    // [0,1] damps the *tangential* (along-the-surface) component of the Verlet-implicit
    // velocity (pos - prev) at the moment of contact, and also zeroes the normal
    // component (inelastic; no bounce). `friction = 0` (the default) instead reproduces
    // the original projection-only behavior exactly -- both velocity components carry
    // straight through -- which is frictionless: an unpinned cloth slides off a smooth
    // sphere indefinitely instead of clinging to it, confirmed by rendering the
    // scan-to-drape integration (see its README/PORTFOLIO_PLAN for how this was found).
    void collideSphere(const Vec3& center, float radius, float friction = 0.f) {
        const float r = radius + 1e-3f; // small skin to avoid z-fighting/jitter
        for (std::size_t i = 0; i < pos_.size(); ++i) {
            if (invMass_[i] == 0.f) continue;
            const Vec3  d    = pos_[i] - center;
            const float dist = d.length();
            if (dist < r && dist > 1e-8f) {
                const Vec3 n = d * (1.f / dist);
                const Vec3 projected = center + n * r;
                if (friction > 0.f) {
                    const Vec3  vel     = pos_[i] - prev_[i];
                    const Vec3  velTang = vel - n * vel.dot(n);
                    prev_[i] = projected - velTang * (1.f - friction);
                } else {
                    prev_[i] = projected - (pos_[i] - prev_[i]);
                }
                pos_[i] = projected;
            }
        }
    }

    // ── Inspection (used by tests and the headless demo) ───────────────────
    const Vec3& posOf(int idx)  const { return pos_[idx]; }
    const Vec3& prevOf(int idx) const { return prev_[idx]; }
    void setPos(int idx, const Vec3& p) { pos_[idx] = p; }

    // Repositions a particle AND its previous-position sample together, so
    // Verlet's implicit velocity (pos - prev) stays zero across the move.
    // `setPos()` alone would leave `prev_` at the old (e.g. flat-grid-layout)
    // location, and the next `step()` would read that mismatch as a huge,
    // spurious one-frame velocity — added specifically for rigidly relocating
    // a whole freshly-constructed grid (which starts at rest) to a world-space
    // start pose before the first `step()`, without an explosive first step.
    void teleport(int idx, const Vec3& p) { pos_[idx] = p; prev_[idx] = p; }

    const std::vector<Constraint>& constraints() const { return cons_; }
    int aliveConstraintCount() const {
        int n = 0;
        for (const auto& c : cons_) n += c.alive ? 1 : 0;
        return n;
    }

    // Worst stretch among alive structural constraints (1 = perfectly rested).
    float maxStretchRatio() const {
        float worst = 1.f;
        for (const auto& c : cons_) {
            if (!c.alive || c.type != SpringType::Structural) continue;
            const float len = (pos_[c.b] - pos_[c.a]).length();
            worst = std::max(worst, len / c.rest);
        }
        return worst;
    }

    // Mean per-step displacement — a cheap "kinetic energy" proxy the tests
    // use to check that a damped cloth settles instead of oscillating forever.
    float meanSpeed(float dt) const {
        float sum = 0.f;
        for (std::size_t i = 0; i < pos_.size(); ++i)
            sum += (pos_[i] - prev_[i]).length();
        return sum / (static_cast<float>(pos_.size()) * dt);
    }

    Params params;

private:
    int cols_, rows_;
    std::vector<Vec3>       pos_, prev_, acc_;
    std::vector<float>      invMass_;
    std::vector<Constraint> cons_;
};

} // namespace cloth
