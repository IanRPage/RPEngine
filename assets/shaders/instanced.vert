#version 410 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

layout(location = 2) in vec3 iPosition;
layout(location = 3) in vec4 iOrientation;  // (x, y, z, w)
layout(location = 4) in vec3 iScale;
layout(location = 5) in vec4 iColor;

uniform mat4 uViewProjection;

out vec3 vWorldNormal;
out vec4 vColor;

mat3 quatToMat3(vec4 q) {
  float x = q.x, y = q.y, z = q.z, w = q.w;
  float x2 = x + x, y2 = y + y, z2 = z + z;
  float xx = x * x2, xy = x * y2, xz = x * z2;
  float yy = y * y2, yz = y * z2, zz = z * z2;
  float wx = w * x2, wy = w * y2, wz = w * z2;

  return mat3(
      1.0 - (yy + zz), xy + wz, xz - wy,
      xy - wz, 1.0 - (xx + zz), yz + wx,
      xz + wy, yz - wx, 1.0 - (xx + yy));
}

void main() {
  mat3 rotation = quatToMat3(iOrientation);
  vec3 worldPos = rotation * (aPosition * iScale) + iPosition;
  vWorldNormal = rotation * (aNormal / iScale);
  vColor = iColor;
  gl_Position = uViewProjection * vec4(worldPos, 1.0);
}
