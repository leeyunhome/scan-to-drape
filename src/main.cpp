#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define GLFW_INCLUDE_ES3
#else
#include <glad/glad.h>
#endif
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifndef __EMSCRIPTEN__
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "cloth/cloth.hpp"
#include "gsplat/camera.hpp"
#include "gsplat/mesh.hpp"
#include "gsplat/ply_loader.hpp"
#include "gsplat/shader.hpp"

namespace {

struct InputState {
    gsplat::OrbitCamera* camera = nullptr;
    double lastX = 0.0, lastY = 0.0;
    bool dragging = false;
    glm::vec3 resetTarget{0.0f};
    float resetYaw = 0.0f, resetPitch = 0.0f, resetDistance = 1.0f;
};

// "Front" isn't yaw=0 -- COLMAP/3DGS reconstructions land in an arbitrary world orientation,
// unrelated to which way the actual object faces. Found empirically, not guessed: computed the
// centroid of the figure's distinctly saturated red/yellow splats (its chest marking, the one
// visible landmark) versus the whole splat cloud's centroid, and solved for the yaw that puts
// the camera on that same side -- 179.2 degrees, independently confirmed by a 12-angle render
// sweep (the marking sits dead-center in frame right around yaw=180, and visibly off-center at
// every other angle tried). The original default view (yaw=30) was ~150 degrees away from this
// -- effectively a back-left angle -- which is exactly why nothing in this whole session's
// screenshots looked like a clearly-facing robot until this was measured.
constexpr float kFrontYawDeg = 180.0f;
constexpr float kPresetPitchDeg = 15.0f;  // the elevation used everywhere else in this project

// Preset ids: 0 front, 1 left, 2 right, 3 top, 4 bottom, 5 reset (back to the initial
// --cam-yaw/--cam-pitch/--cam-dist-factor pose). All presets re-center on the same target
// and re-zoom to the same distance as the initial view, so switching between them never
// leaves the object off-screen. Top/bottom use 89 degrees, not 90 -- exactly vertical makes
// the look-at direction parallel to the (0,1,0) up vector, which is a degenerate lookAt.
void applyPreset(InputState& input, int preset) {
    if (!input.camera) return;
    input.camera->target = input.resetTarget;
    input.camera->distance = input.resetDistance;
    switch (preset) {
        case 0: input.camera->yaw = glm::radians(kFrontYawDeg); input.camera->pitch = glm::radians(kPresetPitchDeg); break;
        case 1: input.camera->yaw = glm::radians(kFrontYawDeg - 90.0f); input.camera->pitch = glm::radians(kPresetPitchDeg); break;
        case 2: input.camera->yaw = glm::radians(kFrontYawDeg + 90.0f); input.camera->pitch = glm::radians(kPresetPitchDeg); break;
        case 3: input.camera->yaw = glm::radians(kFrontYawDeg); input.camera->pitch = glm::radians(89.0f); break;
        case 4: input.camera->yaw = glm::radians(kFrontYawDeg); input.camera->pitch = glm::radians(-89.0f); break;
        default: input.camera->yaw = input.resetYaw; input.camera->pitch = input.resetPitch; break;  // reset
    }
}

void keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    if (key == GLFW_KEY_R) applyPreset(*input, 5);
    else if (key == GLFW_KEY_1) applyPreset(*input, 0);
    else if (key == GLFW_KEY_2) applyPreset(*input, 1);
    else if (key == GLFW_KEY_3) applyPreset(*input, 2);
    else if (key == GLFW_KEY_4) applyPreset(*input, 3);
    else if (key == GLFW_KEY_5) applyPreset(*input, 4);
}

// Web only: lets the HTML shell's preset buttons drive the same camera the keyboard
// shortcuts do, via Module.ccall. `g_input` is set once in main(), after app.input exists.
InputState* g_input = nullptr;

#ifdef __EMSCRIPTEN__
extern "C" {
EMSCRIPTEN_KEEPALIVE void appSetPreset(int preset) {
    if (g_input) applyPreset(*g_input, preset);
}
}
#endif

void mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/) {
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        input->dragging = (action == GLFW_PRESS);
        glfwGetCursorPos(window, &input->lastX, &input->lastY);
    }
}

void cursorPosCallback(GLFWwindow* window, double x, double y) {
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    if (input->dragging) {
        double dx = x - input->lastX;
        double dy = y - input->lastY;
        input->camera->orbit(static_cast<float>(-dx) * 0.005f, static_cast<float>(-dy) * 0.005f);
    }
    input->lastX = x;
    input->lastY = y;
}

void scrollCallback(GLFWwindow* window, double /*dx*/, double dy) {
    auto* input = static_cast<InputState*>(glfwGetWindowUserPointer(window));
    input->camera->zoom(std::pow(0.9f, static_cast<float>(dy)));
}

// One shared quad (a triangle strip: bottom-left, bottom-right, top-left, top-right).
constexpr float kQuadVerts[8] = {-1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};

// Collision proxy extracted directly from the splat cloud: the centroid of every gaussian
// center, and a radius at the given percentile of per-splat distance from that centroid.
// A single sphere is a coarse stand-in for a real simplified collision mesh (roadmap item),
// but a percentile radius (rather than the full bounding-box extent, which is stretched by
// outlying limbs/parts) gives a "body-hugging" size instead of one dominated by the farthest
// single point.
struct CollisionProxy {
    glm::vec3 center;
    float radius;
};

CollisionProxy computeCollisionProxy(const std::vector<gsplat::SplatInstance>& splats, float percentile) {
    glm::vec3 sum(0.0f);
    for (const auto& s : splats) sum += glm::vec3(s.px, s.py, s.pz);
    glm::vec3 centroid = sum / static_cast<float>(splats.size());

    std::vector<float> dist(splats.size());
    for (size_t i = 0; i < splats.size(); ++i) {
        dist[i] = glm::length(glm::vec3(splats[i].px, splats[i].py, splats[i].pz) - centroid);
    }
    size_t idx = std::min(static_cast<size_t>(dist.size() * percentile), dist.size() - 1);
    std::nth_element(dist.begin(), dist.begin() + static_cast<std::ptrdiff_t>(idx), dist.end());
    return {centroid, dist[idx]};
}

// Per-vertex normals for the cloth mesh: accumulate each triangle's face normal onto its
// three vertices, then normalize. Recomputed every frame since the cloth deforms.
std::vector<glm::vec3> computeSmoothNormals(const cloth::Cloth& sim, const std::vector<int>& tri) {
    std::vector<glm::vec3> normals(static_cast<size_t>(sim.particleCount()), glm::vec3(0.0f));
    for (size_t t = 0; t < tri.size(); t += 3) {
        int ia = tri[t], ib = tri[t + 1], ic = tri[t + 2];
        glm::vec3 pa(sim.posOf(ia).x, sim.posOf(ia).y, sim.posOf(ia).z);
        glm::vec3 pb(sim.posOf(ib).x, sim.posOf(ib).y, sim.posOf(ib).z);
        glm::vec3 pc(sim.posOf(ic).x, sim.posOf(ic).y, sim.posOf(ic).z);
        glm::vec3 n = glm::cross(pb - pa, pc - pa);
        normals[ia] += n;
        normals[ib] += n;
        normals[ic] += n;
    }
    for (auto& n : normals) {
        float len = glm::length(n);
        n = len > 1e-8f ? n / len : glm::vec3(0.0f, 1.0f, 0.0f);
    }
    return normals;
}

// Everything the per-frame body touches, so the same frame function can be driven either by
// a native `while` loop or by `emscripten_set_main_loop_arg` (which needs a plain function
// pointer + a void* payload, not a closure over locals).
struct App {
    GLFWwindow* window = nullptr;

    gsplat::SplatCloud cloud;
    size_t splatCount = 0;
    gsplat::OrbitCamera camera;
    InputState input;

    GLuint program = 0;
    GLint uView = -1, uProj = -1, uViewport = -1, uFocal = -1;
    GLuint vao = 0, quadVbo = 0, instanceVbo = 0;
    std::vector<std::pair<float, uint32_t>> order;
    std::vector<gsplat::SplatInstance> sorted;

    bool showTestMesh = false;
    bool drape = false;
    GLuint meshProgram = 0;
    GLint mUModel = -1, mUView = -1, mUProj = -1, mUCameraPos = -1, mUBaseColor = -1;

    GLuint meshVao = 0, meshVbo = 0, meshEbo = 0;
    gsplat::Mesh testMesh;
    glm::mat4 meshModel{1.0f};

    CollisionProxy proxy{};
    std::unique_ptr<cloth::Cloth> clothSim;
    std::vector<int> clothTriangles;
    GLuint clothVao = 0, clothVbo = 0, clothEbo = 0;
    std::vector<gsplat::MeshVertex> clothVertexBuf;
    float floorY = 0.0f;
    float clothFriction = 0.6f;

    std::string screenshotPath;
    int screenshotFrame = 30;
    int frame = 0;
    std::chrono::steady_clock::time_point lastReport;
    int framesSinceReport = 0;

    // On-screen axis gizmo (bottom-left corner): shows current camera orientation so
    // orbiting reads as "turning around the object" instead of losing track of up/forward.
    GLuint axisProgram = 0;
    GLint aUMvp = -1;
    GLuint axisVao = 0, axisVbo = 0;
};

void frameStep(App& app) {
    glfwPollEvents();

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(app.window, &fbWidth, &fbHeight);
    if (fbWidth == 0 || fbHeight == 0) return;

    glm::mat4 view = app.camera.viewMatrix();
    float aspect = static_cast<float>(fbWidth) / static_cast<float>(fbHeight);
    float fovY = glm::radians(50.0f);
    glm::mat4 proj = glm::perspective(fovY, aspect, 0.01f, std::max(app.camera.distance * 20.0f, 10.0f));
    float focalY = static_cast<float>(fbHeight) / (2.0f * std::tan(fovY * 0.5f));

    // Back-to-front depth sort (painter's algorithm) so alpha-blended
    // splats composite correctly without a per-pixel depth test.
    for (size_t i = 0; i < app.splatCount; ++i) {
        const auto& s = app.cloud.splats[i];
        glm::vec4 v = view * glm::vec4(s.px, s.py, s.pz, 1.0f);
        app.order[i] = {v.z, static_cast<uint32_t>(i)};
    }
    std::sort(app.order.begin(), app.order.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    for (size_t i = 0; i < app.splatCount; ++i) app.sorted[i] = app.cloud.splats[app.order[i].second];

    glBindBuffer(GL_ARRAY_BUFFER, app.instanceVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(app.splatCount * sizeof(gsplat::SplatInstance)),
                 app.sorted.data(), GL_DYNAMIC_DRAW);

    glViewport(0, 0, fbWidth, fbHeight);
    // glClear(GL_DEPTH_BUFFER_BIT) is a no-op wherever glDepthMask is false, and the splat
    // pass below leaves it false at the end of every frame -- force it back on so the clear
    // actually takes effect instead of leaving stale depth from the previous frame.
    glDepthMask(GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (app.drape) {
        // Advance the cloth one fixed step per rendered frame (not wall-clock-locked --
        // simulation "time" is tied to frame count, which is what makes --frames give a
        // deterministic, reproducible pose for screenshots).
        app.clothSim->step(1.0f / 60.0f);
        app.clothSim->collideSphere(cloth::Vec3(app.proxy.center.x, app.proxy.center.y, app.proxy.center.z),
                                     app.proxy.radius, app.clothFriction);
        for (int i = 0; i < app.clothSim->particleCount(); ++i) {
            if (app.clothSim->posOf(i).y < app.floorY) {
                app.clothSim->setPos(i, cloth::Vec3(app.clothSim->posOf(i).x, app.floorY, app.clothSim->posOf(i).z));
            }
        }

        std::vector<glm::vec3> normals = computeSmoothNormals(*app.clothSim, app.clothTriangles);
        for (int i = 0; i < app.clothSim->particleCount(); ++i) {
            const cloth::Vec3& p = app.clothSim->posOf(i);
            app.clothVertexBuf[static_cast<size_t>(i)] = {p.x, p.y, p.z, normals[static_cast<size_t>(i)].x,
                                                           normals[static_cast<size_t>(i)].y, normals[static_cast<size_t>(i)].z};
        }
        glBindBuffer(GL_ARRAY_BUFFER, app.clothVbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                         static_cast<GLsizeiptr>(app.clothVertexBuf.size() * sizeof(gsplat::MeshVertex)),
                         app.clothVertexBuf.data());
    }

    if (app.showTestMesh || app.drape) {
        // Opaque pass: normal z-buffered rendering, writes depth for the splat pass to read.
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);

        glUseProgram(app.meshProgram);
        glUniformMatrix4fv(app.mUView, 1, GL_FALSE, &view[0][0]);
        glUniformMatrix4fv(app.mUProj, 1, GL_FALSE, &proj[0][0]);
        glUniform3fv(app.mUCameraPos, 1, &app.camera.eye()[0]);

        if (app.drape) {
            glm::mat4 identity(1.0f);
            glUniformMatrix4fv(app.mUModel, 1, GL_FALSE, &identity[0][0]);
            glUniform3f(app.mUBaseColor, 0.65f, 0.68f, 0.85f);
            glBindVertexArray(app.clothVao);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(app.clothTriangles.size()), GL_UNSIGNED_INT, nullptr);
        } else {
            glUniformMatrix4fv(app.mUModel, 1, GL_FALSE, &app.meshModel[0][0]);
            glUniform3f(app.mUBaseColor, 0.85f, 0.35f, 0.25f);
            glBindVertexArray(app.meshVao);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(app.testMesh.indices.size()), GL_UNSIGNED_INT, nullptr);
        }

        // Splat pass: depth-tested against the mesh (so mesh occludes splats behind it)
        // but not depth-written, since splat-vs-splat order is handled by the CPU sort.
        glDepthMask(GL_FALSE);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(app.program);
    glUniformMatrix4fv(app.uView, 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(app.uProj, 1, GL_FALSE, &proj[0][0]);
    glUniform2f(app.uViewport, static_cast<float>(fbWidth), static_cast<float>(fbHeight));
    glUniform2f(app.uFocal, focalY, focalY);

    glBindVertexArray(app.vao);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(app.splatCount));

    // Axis gizmo: bottom-left corner, rotation-only (no translation/perspective) so it shows
    // orientation alone, drawn last with its own small viewport and no depth test so it's
    // always on top regardless of what's behind it in the main scene.
    {
        int giz = std::min({fbWidth, fbHeight, 130}) - 20;
        giz = std::max(giz, 60);
        glViewport(16, 16, giz, giz);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glm::mat4 rotOnly = glm::mat4(glm::mat3(view));
        glm::mat4 ortho = glm::ortho(-1.3f, 1.3f, -1.3f, 1.3f, -10.0f, 10.0f);

        glUseProgram(app.axisProgram);
        glBindVertexArray(app.axisVao);

        glm::mat4 identity(1.0f);
        glUniformMatrix4fv(app.aUMvp, 1, GL_FALSE, &identity[0][0]);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);  // dark backdrop, fills the gizmo viewport

        glm::mat4 axisMvp = ortho * rotOnly;
        glUniformMatrix4fv(app.aUMvp, 1, GL_FALSE, &axisMvp[0][0]);
        glDrawArrays(GL_LINES, 4, 6);  // X (red) / Y (green) / Z (blue)
    }

    glfwSwapBuffers(app.window);

    ++app.frame;
    ++app.framesSinceReport;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - app.lastReport).count();
    if (elapsed >= 1.0) {
        std::cout << "frame " << app.frame << " | " << (app.framesSinceReport / elapsed) << " fps\n";
        app.framesSinceReport = 0;
        app.lastReport = now;
    }

#ifndef __EMSCRIPTEN__
    if (!app.screenshotPath.empty() && app.frame == app.screenshotFrame) {
        std::vector<unsigned char> pixels(static_cast<size_t>(fbWidth) * fbHeight * 3);
        glReadPixels(0, 0, fbWidth, fbHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        std::vector<unsigned char> flipped(pixels.size());
        for (int y = 0; y < fbHeight; ++y) {
            std::memcpy(&flipped[static_cast<size_t>(y) * fbWidth * 3],
                        &pixels[static_cast<size_t>(fbHeight - 1 - y) * fbWidth * 3],
                        static_cast<size_t>(fbWidth) * 3);
        }
        stbi_write_png(app.screenshotPath.c_str(), fbWidth, fbHeight, 3, flipped.data(), fbWidth * 3);
        std::cout << "wrote screenshot: " << app.screenshotPath << "\n";
        glfwSetWindowShouldClose(app.window, GLFW_TRUE);
    }
#endif
}

#ifdef __EMSCRIPTEN__
void emscriptenFrameStep(void* arg) { frameStep(*static_cast<App*>(arg)); }
#endif

}  // namespace

int main(int argc, char** argv) {
    App app;

    std::string plyPath;
    float keepPercentile = 1.0f;  // real cleanup now happens offline via tools/clean_scan.py
    float sphereRadiusFactor = 0.4f;
    int clothGrid = 40;
    float proxyPercentile = 0.65f;
    float clothSizeFactor = 2.6f;
    float startHeightFactor = 2.0f;
    // Default to a 3/4 view near the measured front (kFrontYawDeg), not an arbitrary angle --
    // the first thing anyone sees (native window or the web demo) should read as "a figure",
    // not an unrecognizable blob.
    float camYawDeg = kFrontYawDeg - 25.0f, camPitchDeg = kPresetPitchDeg, camDistFactor = 2.5f;
    int width = 1280, height = 800;

#ifdef __EMSCRIPTEN__
    // No command line in a browser: the web build always shows the same fixed demo (the
    // cloth drape, the most complete Phase 3 result) against the bundled, web-trimmed scan.
    plyPath = "mechander_web.ply";
    app.drape = true;
#else
    if (argc < 2) {
        std::cerr << "usage: viewer <path-to-3dgs.ply> [--screenshot out.png] [--frames N] [--width W] [--height H]\n";
        return 1;
    }
    plyPath = argv[1];

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--screenshot" && i + 1 < argc) {
            app.screenshotPath = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            app.screenshotFrame = std::atoi(argv[++i]);
        } else if (arg == "--width" && i + 1 < argc) {
            width = std::atoi(argv[++i]);
        } else if (arg == "--height" && i + 1 < argc) {
            height = std::atoi(argv[++i]);
        } else if (arg == "--keep-percentile" && i + 1 < argc) {
            keepPercentile = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--test-mesh") {
            app.showTestMesh = true;
        } else if (arg == "--sphere-radius-factor" && i + 1 < argc) {
            sphereRadiusFactor = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--drape") {
            app.drape = true;
        } else if (arg == "--cloth-grid" && i + 1 < argc) {
            clothGrid = std::atoi(argv[++i]);
        } else if (arg == "--proxy-percentile" && i + 1 < argc) {
            proxyPercentile = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--cam-yaw" && i + 1 < argc) {
            camYawDeg = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--cam-pitch" && i + 1 < argc) {
            camPitchDeg = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--cam-dist-factor" && i + 1 < argc) {
            camDistFactor = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--friction" && i + 1 < argc) {
            app.clothFriction = static_cast<float>(std::atof(argv[++i]));
        }
    }
#endif

    std::cout << "loading " << plyPath << " ...\n";
    try {
        app.cloud = gsplat::loadGaussianPly(plyPath, keepPercentile);
    } catch (const std::exception& e) {
        std::cerr << "failed to load PLY: " << e.what() << "\n";
        return 1;
    }
    app.splatCount = app.cloud.splats.size();
    std::cout << "loaded " << app.splatCount << " splats, bounds radius "
              << app.cloud.bounds.radius() << "\n";

    if (!glfwInit()) {
        std::cerr << "glfwInit failed\n";
        return 1;
    }
#ifndef __EMSCRIPTEN__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#else
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    app.window = glfwCreateWindow(width, height, "Scan-to-Drape Viewer", nullptr, nullptr);
    if (!app.window) {
        std::cerr << "glfwCreateWindow failed\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(app.window);

#ifndef __EMSCRIPTEN__
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "gladLoadGLLoader failed\n";
        return 1;
    }
#endif

    app.camera.target = glm::vec3(app.cloud.bounds.centerX(), app.cloud.bounds.centerY(), app.cloud.bounds.centerZ());
    app.camera.distance = std::max(app.cloud.bounds.radius() * camDistFactor, 0.1f);
    app.camera.pitch = glm::radians(camPitchDeg);
    app.camera.yaw = glm::radians(camYawDeg);

    app.input.camera = &app.camera;
    app.input.resetTarget = app.camera.target;
    app.input.resetYaw = app.camera.yaw;
    app.input.resetPitch = app.camera.pitch;
    app.input.resetDistance = app.camera.distance;
    g_input = &app.input;
    glfwSetWindowUserPointer(app.window, &app.input);
    glfwSetMouseButtonCallback(app.window, mouseButtonCallback);
    glfwSetCursorPosCallback(app.window, cursorPosCallback);
    glfwSetScrollCallback(app.window, scrollCallback);
    glfwSetKeyCallback(app.window, keyCallback);

    app.program = gsplat::linkProgram(std::string(SHADER_DIR) + "/splat.vert",
                                       std::string(SHADER_DIR) + "/splat.frag");
    app.uView = glGetUniformLocation(app.program, "uView");
    app.uProj = glGetUniformLocation(app.program, "uProj");
    app.uViewport = glGetUniformLocation(app.program, "uViewport");
    app.uFocal = glGetUniformLocation(app.program, "uFocal");

    glGenVertexArrays(1, &app.vao);
    glGenBuffers(1, &app.quadVbo);
    glGenBuffers(1, &app.instanceVbo);

    glBindVertexArray(app.vao);

    glBindBuffer(GL_ARRAY_BUFFER, app.quadVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVerts), kQuadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void*>(0));

    glBindBuffer(GL_ARRAY_BUFFER, app.instanceVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(app.splatCount * sizeof(gsplat::SplatInstance)),
                 app.cloud.splats.data(), GL_DYNAMIC_DRAW);

    constexpr GLsizei stride = sizeof(gsplat::SplatInstance);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offsetof(gsplat::SplatInstance, px)));
    glVertexAttribDivisor(1, 1);

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offsetof(gsplat::SplatInstance, cov_xx)));
    glVertexAttribDivisor(2, 1);

    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offsetof(gsplat::SplatInstance, cov_yy)));
    glVertexAttribDivisor(3, 1);

    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offsetof(gsplat::SplatInstance, r)));
    glVertexAttribDivisor(4, 1);

    glBindVertexArray(0);

    // Opaque mesh pass (Phase 2: hybrid splat+mesh depth compositing), shared by the static
    // test sphere and the Phase 3 cloth drape — both are just triangles under the same shader.
    if (app.showTestMesh || app.drape) {
        app.meshProgram = gsplat::linkProgram(std::string(SHADER_DIR) + "/mesh.vert",
                                               std::string(SHADER_DIR) + "/mesh.frag");
        app.mUModel = glGetUniformLocation(app.meshProgram, "uModel");
        app.mUView = glGetUniformLocation(app.meshProgram, "uView");
        app.mUProj = glGetUniformLocation(app.meshProgram, "uProj");
        app.mUCameraPos = glGetUniformLocation(app.meshProgram, "uCameraPos");
        app.mUBaseColor = glGetUniformLocation(app.meshProgram, "uBaseColor");
    }

    if (app.showTestMesh) {
        app.testMesh = gsplat::makeUvSphere();
        glGenVertexArrays(1, &app.meshVao);
        glGenBuffers(1, &app.meshVbo);
        glGenBuffers(1, &app.meshEbo);
        glBindVertexArray(app.meshVao);
        glBindBuffer(GL_ARRAY_BUFFER, app.meshVbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(app.testMesh.vertices.size() * sizeof(gsplat::MeshVertex)),
                     app.testMesh.vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, app.meshEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(app.testMesh.indices.size() * sizeof(uint32_t)),
                     app.testMesh.indices.data(), GL_STATIC_DRAW);
        constexpr GLsizei meshStride = sizeof(gsplat::MeshVertex);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, meshStride, reinterpret_cast<void*>(offsetof(gsplat::MeshVertex, px)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, meshStride, reinterpret_cast<void*>(offsetof(gsplat::MeshVertex, nx)));
        glBindVertexArray(0);
    }

    glm::vec3 sphereCenter(app.cloud.bounds.centerX(), app.cloud.bounds.centerY(), app.cloud.bounds.centerZ());
    float sphereRadius = app.cloud.bounds.radius() * sphereRadiusFactor;
    app.meshModel = glm::translate(glm::mat4(1.0f), sphereCenter) * glm::scale(glm::mat4(1.0f), glm::vec3(sphereRadius));

    // Phase 3: collision proxy extracted from the splat cloud, a cloth grid draped over it.
    if (app.drape) {
        app.proxy = computeCollisionProxy(app.cloud.splats, proxyPercentile);
        app.floorY = app.proxy.center.y - app.proxy.radius * 1.05f;

        float clothSize = app.proxy.radius * clothSizeFactor;
        app.clothSim = std::make_unique<cloth::Cloth>(clothGrid, clothGrid, clothSize, clothSize);
        app.clothSim->params.iterations = 6;
        float startY = app.proxy.center.y + app.proxy.radius * startHeightFactor;
        for (int y = 0; y < clothGrid; ++y) {
            for (int x = 0; x < clothGrid; ++x) {
                int idx = app.clothSim->indexOf(x, y);
                const cloth::Vec3& local = app.clothSim->posOf(idx);
                // Flatten the constructor's vertical XY layout into a horizontal sheet:
                // local.x stays a horizontal offset, local.y (originally "down the rows")
                // becomes the other horizontal axis, and every particle starts at the same
                // world height above the proxy so the sheet falls flat onto it.
                cloth::Vec3 world(local.x + app.proxy.center.x, startY, local.y + app.proxy.center.z);
                app.clothSim->teleport(idx, world);
            }
        }
        app.clothTriangles = app.clothSim->triangles();

        glGenVertexArrays(1, &app.clothVao);
        glGenBuffers(1, &app.clothVbo);
        glGenBuffers(1, &app.clothEbo);
        glBindVertexArray(app.clothVao);
        app.clothVertexBuf.resize(static_cast<size_t>(app.clothSim->particleCount()));
        glBindBuffer(GL_ARRAY_BUFFER, app.clothVbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(app.clothVertexBuf.size() * sizeof(gsplat::MeshVertex)),
                     nullptr, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, app.clothEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(app.clothTriangles.size() * sizeof(int)),
                     app.clothTriangles.data(), GL_STATIC_DRAW);
        constexpr GLsizei clothStride = sizeof(gsplat::MeshVertex);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, clothStride, reinterpret_cast<void*>(offsetof(gsplat::MeshVertex, px)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, clothStride, reinterpret_cast<void*>(offsetof(gsplat::MeshVertex, nx)));
        glBindVertexArray(0);

        std::cout << "drape: proxy center=(" << app.proxy.center.x << "," << app.proxy.center.y << "," << app.proxy.center.z
                  << ") radius=" << app.proxy.radius << " cloth=" << clothGrid << "x" << clothGrid
                  << " size=" << clothSize << " startY=" << startY << " floorY=" << app.floorY << "\n";
    }

    app.axisProgram = gsplat::linkProgram(std::string(SHADER_DIR) + "/axis.vert",
                                           std::string(SHADER_DIR) + "/axis.frag");
    app.aUMvp = glGetUniformLocation(app.axisProgram, "uMvp");
    {
        // 4 backdrop verts (identity-MVP, fills the gizmo viewport) + 6 axis-line verts
        // (origin -> +X/+Y/+Z, drawn under an orbit-rotation-only MVP). Interleaved
        // position (vec3) + color (vec4), 7 floats/vertex.
        const float v[10 * 7] = {
            // backdrop quad (triangle strip), dark + semi-transparent
            -1.0f, -1.0f, 0.0f,  0.05f, 0.05f, 0.07f, 0.75f,
             1.0f, -1.0f, 0.0f,  0.05f, 0.05f, 0.07f, 0.75f,
            -1.0f,  1.0f, 0.0f,  0.05f, 0.05f, 0.07f, 0.75f,
             1.0f,  1.0f, 0.0f,  0.05f, 0.05f, 0.07f, 0.75f,
            // X axis, red
            0.0f, 0.0f, 0.0f,  0.9f, 0.25f, 0.25f, 1.0f,
            0.9f, 0.0f, 0.0f,  0.9f, 0.25f, 0.25f, 1.0f,
            // Y axis, green
            0.0f, 0.0f, 0.0f,  0.35f, 0.85f, 0.35f, 1.0f,
            0.0f, 0.9f, 0.0f,  0.35f, 0.85f, 0.35f, 1.0f,
            // Z axis, blue
            0.0f, 0.0f, 0.0f,  0.35f, 0.55f, 0.95f, 1.0f,
            0.0f, 0.0f, 0.9f,  0.35f, 0.55f, 0.95f, 1.0f,
        };
        glGenVertexArrays(1, &app.axisVao);
        glGenBuffers(1, &app.axisVbo);
        glBindVertexArray(app.axisVao);
        glBindBuffer(GL_ARRAY_BUFFER, app.axisVbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(v), v, GL_STATIC_DRAW);
        constexpr GLsizei axisStride = 7 * sizeof(float);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, axisStride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, axisStride, reinterpret_cast<void*>(3 * sizeof(float)));
        glBindVertexArray(0);
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.05f, 0.05f, 0.07f, 1.0f);

    app.order.resize(app.splatCount);
    app.sorted.resize(app.splatCount);
    app.lastReport = std::chrono::steady_clock::now();

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(emscriptenFrameStep, &app, 0, 1);
#else
    while (!glfwWindowShouldClose(app.window)) frameStep(app);
    glfwDestroyWindow(app.window);
    glfwTerminate();
#endif
    return 0;
}
