layout(location = 0) in vec2 aQuadPos;   // shared quad corner: (-1,-1)..(1,1)
layout(location = 1) in vec3 iCenter;    // world-space splat center
layout(location = 2) in vec3 iCovA;      // world cov upper triangle: xx, xy, xz
layout(location = 3) in vec3 iCovB;      // world cov upper triangle: yy, yz, zz
layout(location = 4) in vec4 iColor;     // rgb + opacity

uniform mat4 uView;
uniform mat4 uProj;
uniform vec2 uViewport;
uniform vec2 uFocal;

out vec4 vColor;
out vec2 vQuadPos;

void main() {
    vec4 viewPos = uView * vec4(iCenter, 1.0);
    vec4 clipPos = uProj * viewPos;

    // Cull behind-camera and far-outside-frustum splats before doing the
    // (division-heavy) covariance projection below.
    float guard = 1.3 * clipPos.w;
    if (clipPos.w <= 0.0001 || clipPos.z < -guard || clipPos.x < -guard || clipPos.x > guard ||
        clipPos.y < -guard || clipPos.y > guard) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);  // outside NDC after the w-divide
        vColor = vec4(0.0);
        vQuadPos = vec2(0.0);
        return;
    }

    // 3D covariance -> 2D screen-space covariance, via the affine (Jacobian)
    // approximation of the perspective projection. Kerbl et al. 2023,
    // "3D Gaussian Splatting for Real-Time Radiance Field Rendering".
    mat3 Vrk = mat3(
        iCovA.x, iCovA.y, iCovA.z,
        iCovA.y, iCovB.x, iCovB.y,
        iCovA.z, iCovB.y, iCovB.z
    );

    mat3 J = mat3(
        uFocal.x / viewPos.z, 0.0, -(uFocal.x * viewPos.x) / (viewPos.z * viewPos.z),
        0.0, uFocal.y / viewPos.z, -(uFocal.y * viewPos.y) / (viewPos.z * viewPos.z),
        0.0, 0.0, 0.0
    );

    mat3 W = transpose(mat3(uView));
    mat3 T = W * J;
    mat3 cov2d = transpose(T) * Vrk * T;

    // Low-pass dilation so sub-pixel splats stay visible (same fudge factor
    // as the reference CUDA rasterizer).
    cov2d[0][0] += 0.3;
    cov2d[1][1] += 0.3;

    float a = cov2d[0][0];
    float bTerm = cov2d[0][1];
    float c = cov2d[1][1];

    // Eigen-decomposition of the symmetric 2x2 covariance: closed form.
    float mid = 0.5 * (a + c);
    float diff = 0.5 * (a - c);
    float radius = sqrt(max(diff * diff + bTerm * bTerm, 0.0));
    float lambda1 = mid + radius;
    float lambda2 = max(mid - radius, 0.0);

    vec2 axisDir = (abs(bTerm) > 1e-6)
        ? normalize(vec2(bTerm, lambda1 - a))
        : ((a >= c) ? vec2(1.0, 0.0) : vec2(0.0, 1.0));

    // Quad half-extent = sqrt(8*lambda) along each principal axis. With that
    // scaling, aQuadPos is exactly the Gaussian's local coordinate and the
    // unit circle in aQuadPos-space is the exp(-4) falloff ring (see splat.frag).
    vec2 v1 = axisDir * min(sqrt(8.0 * lambda1), 2048.0);
    vec2 v2 = vec2(-axisDir.y, axisDir.x) * min(sqrt(8.0 * lambda2), 2048.0);

    vec2 screenOffset = aQuadPos.x * v1 + aQuadPos.y * v2;
    vec2 ndcOffset = screenOffset / uViewport * 2.0;
    vec2 ndcCenter = clipPos.xy / clipPos.w;

    gl_Position = vec4(ndcCenter + ndcOffset, clipPos.z / clipPos.w, 1.0);
    vColor = iColor;
    vQuadPos = aQuadPos;
}
