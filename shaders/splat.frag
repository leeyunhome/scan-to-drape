in vec4 vColor;
in vec2 vQuadPos;

out vec4 fragColor;

void main() {
    float r2 = dot(vQuadPos, vQuadPos);
    if (r2 > 1.0) discard;

    float weight = exp(-4.0 * r2) * vColor.a;
    if (weight < (1.0 / 255.0)) discard;

    // Premultiplied alpha; blended with GL_ONE / GL_ONE_MINUS_SRC_ALPHA
    // over splats submitted back-to-front.
    fragColor = vec4(vColor.rgb * weight, weight);
}
