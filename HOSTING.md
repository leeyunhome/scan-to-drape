# Hosting plan — GitHub Pages

Same convention as the other projects on this account: one repo per project, served at
`leeyunhome.github.io/<repo>/`.

## Layout

```
scan-to-drape/            → https://leeyunhome.github.io/scan-to-drape/
├── index.html             # placeholder landing page (full case study pending)
├── demo/index.html         → /scan-to-drape/demo/  (the interactive WASM/WebGL2 viewer)
├── demo/viewer.js|wasm|data
├── src/, include/, shaders/, tools/   # the actual C++ source (native + web build from the same code)
├── models/                 # native-build .ply assets (the web build's copy is baked into demo/viewer.data)
├── notes/                  # screenshots referenced from README.md
└── README.md               # the real write-up: what was built, bugs found, how they were fixed
```

`demo/` is built by `cmake --build build-web` (see `README.md` → Building) and then copied in:

```bash
cp build-web/viewer.html demo/index.html
cp build-web/viewer.js build-web/viewer.wasm build-web/viewer.data demo/
```

## GitHub Pages settings

One-time, in the repo on GitHub:

1. **Settings → Pages**
2. **Source:** *Deploy from a branch*
3. **Branch:** `main` · folder `/ (root)` → **Save**
4. Wait 1–2 min → live at `https://leeyunhome.github.io/scan-to-drape/`

No workflow file needed — plain static files, no build step on GitHub's side.

## Redeploying after a change to `src/`

Rebuild the web target and re-copy the four files into `demo/`, then commit + push. Pages
redeploys automatically on push to `main`.
