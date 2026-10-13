#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <core/World.hpp>

namespace scenes {

struct Scene {
  std::vector<BodyHandle> bodies;
  Vec3f min{0.0f};
  Vec3f max{0.0f};

  bool contains(Vec3f p) const {
    return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y &&
           p.z >= min.z && p.z <= max.z;
  }
};

inline void addGround(World& world, float friction) {
  Transform t;
  t.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(50.0f, 0.5f, 50.0f)}}, t,
                         friction, /*restitution=*/0.0f);
}

// unit boxes with a 1 cm gap. a box leaving the half-box-wide column means it
// toppled
inline Scene buildBoxStack(World& world, int height) {
  addGround(world, /*friction=*/0.5f);
  Scene scene;
  scene.min = Vec3f(-0.25f, 0.0f, -0.25f);
  scene.max = Vec3f(0.25f, static_cast<float>(height) * 1.01f, 0.25f);
  for (int i = 0; i < height; i++) {
    Transform t;
    t.position = Vec3f(0.0f, 0.5f + static_cast<float>(i) * 1.01f, 0.0f);
    scene.bodies.push_back(world.createDynamicBody(
        ShapeVariant{BoxShape{Vec3f(0.5f)}}, t, /*mass=*/1.0f,
        /*friction=*/0.5f, /*restitution=*/0.0f));
  }
  return scene;
}

// `n` bodies on a jittered, layered grid inside addWorldBoundaries walls
inline Scene buildPile(World& world, bool mixed, int n) {
  const float cell = mixed ? 1.5f : 1.2f;
  const int layers =
      std::max(1, static_cast<int>(std::lround(std::cbrt(n) / 2.0)));
  const int k = static_cast<int>(std::ceil(std::sqrt(double(n) / layers)));
  const int usedLayers = (n + k * k - 1) / (k * k);
  const float halfWidth = 0.5f * (static_cast<float>(k) * cell + 0.5f);
  const float height = static_cast<float>(usedLayers) * cell + 1.5f;

  Scene scene;
  scene.min = Vec3f(-halfWidth, 0.0f, -halfWidth);
  scene.max = Vec3f(halfWidth, height, halfWidth);
  world.addWorldBoundaries(scene.min, scene.max, /*thickness=*/1.0f,
                           /*friction=*/0.5f, /*restitution=*/0.2f,
                           /*is2D=*/false);

  std::mt19937 rng(0x5EED);
  std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
  std::normal_distribution<float> gauss;
  for (int i = 0; i < n; i++) {
    const int layer = i / (k * k);
    const int ix = (i % (k * k)) % k;
    const int iz = (i % (k * k)) / k;
    Transform t;
    t.position =
        Vec3f(-halfWidth + 0.25f + cell * (static_cast<float>(ix) + 0.5f) +
                  jitter(rng),
              0.25f + cell * (static_cast<float>(layer) + 0.5f),
              -halfWidth + 0.25f + cell * (static_cast<float>(iz) + 0.5f) +
                  jitter(rng));

    ShapeVariant shape{SphereShape{0.5f}};
    if (mixed) {
      t.orientation =
          glm::normalize(Quatf(gauss(rng), gauss(rng), gauss(rng), gauss(rng)));
      const auto pick = rng() % 3;
      if (pick == 0) { shape = ShapeVariant{BoxShape{Vec3f(0.4f)}}; }
      if (pick == 2) { shape = ShapeVariant{CapsuleShape{0.3f, 0.4f}}; }
    }
    scene.bodies.push_back(world.createDynamicBody(
        shape, t, /*mass=*/1.0f, /*friction=*/0.5f, /*restitution=*/0.2f));
  }
  return scene;
}

}  // namespace scenes
