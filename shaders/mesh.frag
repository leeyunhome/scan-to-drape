in vec3 vNormal;
in vec3 vWorldPos;

uniform vec3 uCameraPos;
uniform vec3 uBaseColor;

out vec4 fragColor;

void main() {
    vec3 n = normalize(vNormal);
    vec3 lightDir = normalize(vec3(0.4, 0.8, 0.5));
    float diffuse = max(dot(n, lightDir), 0.0);
    vec3 viewDir = normalize(uCameraPos - vWorldPos);
    vec3 halfDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(n, halfDir), 0.0), 32.0);

    vec3 color = uBaseColor * (0.25 + 0.7 * diffuse) + vec3(1.0) * spec * 0.3;
    fragColor = vec4(color, 1.0);
}
