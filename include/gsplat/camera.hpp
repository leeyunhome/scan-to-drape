#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace gsplat {

// Orbit camera: yaw/pitch/distance around a fixed target point.
class OrbitCamera {
public:
    glm::vec3 target{0.0f};
    float yaw = 0.0f;      // radians, around +Y
    float pitch = 0.0f;    // radians, clamped away from the poles
    float distance = 3.0f;

    void orbit(float dYaw, float dPitch) {
        yaw += dYaw;
        pitch += dPitch;
        constexpr float limit = glm::radians(89.0f);
        pitch = glm::clamp(pitch, -limit, limit);
    }

    void zoom(float factor) {
        distance = glm::clamp(distance * factor, 0.01f, 1000.0f);
    }

    glm::vec3 eye() const {
        float cp = std::cos(pitch), sp = std::sin(pitch);
        float cy = std::cos(yaw), sy = std::sin(yaw);
        glm::vec3 offset{cp * sy, sp, cp * cy};
        return target + offset * distance;
    }

    glm::mat4 viewMatrix() const {
        return glm::lookAt(eye(), target, glm::vec3(0.0f, 1.0f, 0.0f));
    }
};

}  // namespace gsplat
