#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#else
#include <glad/glad.h>
#endif

namespace gsplat {

inline std::string readTextFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("could not open file: " + path);
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

// Shader files are written without a `#version` line so the same source works on both
// targets: desktop needs `#version 330 core`, WebGL2/GLES needs `#version 300 es` plus an
// explicit fragment-shader precision qualifier (vertex shaders default to highp already
// per the GLSL ES spec, so only fragment shaders need the extra line).
inline std::string addVersionHeader(const std::string& source, GLenum stage) {
#ifdef __EMSCRIPTEN__
    std::string header = "#version 300 es\n";
    if (stage == GL_FRAGMENT_SHADER) header += "precision highp float;\n";
#else
    std::string header = "#version 330 core\n";
    (void)stage;
#endif
    return header + source;
}

inline GLuint compileShader(GLenum stage, const std::string& source) {
    GLuint shader = glCreateShader(stage);
    std::string full = addVersionHeader(source, stage);
    const char* src = full.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("shader compile error: ") + log);
    }
    return shader;
}

inline GLuint linkProgram(const std::string& vertPath, const std::string& fragPath) {
    GLuint vs = compileShader(GL_VERTEX_SHADER, readTextFile(vertPath));
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, readTextFile(fragPath));

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        glDeleteProgram(program);
        throw std::runtime_error(std::string("program link error: ") + log);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}

}  // namespace gsplat
