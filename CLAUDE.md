# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A native C++/OpenGL 3D Gaussian Splatting viewer, built toward draping a cloth simulation over a
real 3DGS scan. See `README.md` and `../../HonglabSplatting/PORTFOLIO_PLAN.md` for the phase-by-phase
plan and results. This file is about the environment/build gotchas hit while developing it — the
lessons the README doesn't need but the next session working on this machine does.

## Toolchain (this machine)

No compiler, CMake, or vcpkg is on `PATH` by default, and `vswhere.exe` isn't found either (its
absence causes a benign, ignorable warning line from `vcvarsall.bat` — not a real error). Everything
has to be invoked by full path, from a single PowerShell/cmd command so the environment persists:

```powershell
$vcvars = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat"
$cmake  = "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$ninja  = "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
$toolchain = "C:\coding\vcpkg\scripts\buildsystems\vcpkg.cmake"

cmd.exe /c "`"$vcvars`" x64 && `"$cmake`" -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=`"$toolchain`" -DCMAKE_MAKE_PROGRAM=`"$ninja`" -DCMAKE_BUILD_TYPE=RelWithDebInfo"
cmd.exe /c "`"$vcvars`" x64 && `"$cmake`" --build build"
```

vcpkg lives at `C:\coding\vcpkg` (shared across projects on this machine, not vendored in this repo).
Packages needed here: `glfw3 glad glm stb` for `x64-windows` — already installed there; a fresh
machine would need `vcpkg install glfw3 glad glm stb --triplet x64-windows` first.

**Source encoding**: comments in this codebase use em dashes (—) and other non-ASCII punctuation.
Without `/utf-8` (already in `CMakeLists.txt`'s MSVC branch), `cl.exe` reads source files using the
system codepage (949/Korean on this machine) and emits `C4819` warnings — harmless, but if a new
translation unit is added outside CMake's control, make sure it inherits that flag.

## Python side (`tools/clean_scan.py`)

Same gotcha as `HonglabSplatting/CLAUDE.md`: the Bash tool's default `python` (mingw64) has no pip
and lacks `numpy`/`scipy`/`open3d`. Use `C:\Users\<user>\miniconda3\python.exe` explicitly for
anything under `tools/`.

## A real OpenGL bug worth knowing before touching the render loop

`glClear(GL_DEPTH_BUFFER_BIT)` is a no-op wherever `glDepthMask` is currently `false` at the moment
`glClear` is called — not just during the actual depth write later in the frame. The splat pass
leaves depth mask `false` at the end of every frame (intentionally — it must not write depth). The
fix already in `src/main.cpp` is to force `glDepthMask(GL_TRUE)` immediately before the per-frame
`glClear` call, before anything else runs. If that line ever gets removed or reordered, the opaque
mesh/cloth pass will silently stop rendering (zero GL errors, zero visible geometry, zero occlusion)
because it keeps testing new fragments against last frame's un-cleared depth values. This was only
caught by `glReadPixels`-ing a specific pixel right after the draw call and comparing it to the clear
color — code review alone didn't surface it. If mesh/cloth rendering ever silently goes blank again,
check this first.

## Known residual issue (parked, not a regression)

At camera pitches much above ~15° (the default), a handful of oversized splats can dominate the
frame from certain angles — a side effect of Phase 1.6's cleanup being *local*-outlier-based rather
than fully exhaustive (see README's Phase 3 section for how this was isolated from an unrelated
cloth-physics bug). Not yet fixed; stick to shallow camera pitches for demo screenshots until it is.
