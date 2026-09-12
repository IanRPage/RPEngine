#version 410 core

in vec3 vWorldNormal;
in vec4 vColor;

uniform vec3 uLightDirection;
uniform float uAmbientStrength;

out vec4 FragColor;

void main() {
  vec3 normal = normalize(vWorldNormal);
  vec3 toLight = normalize(-uLightDirection);
  float diffuse = max(dot(normal, toLight), 0.0);
  float lighting = uAmbientStrength + (1.0 - uAmbientStrength) * diffuse;
  FragColor = vec4(vColor.rgb * lighting, vColor.a);
}
