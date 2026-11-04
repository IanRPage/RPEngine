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

inline Scene buildDemoScene(World& world, int n = 1000) {
  CompoundShape openBox({
      CompoundChild{BoxShape{Vec3f(8.5f, 0.5f, 8.5f)},
                    Transform{Vec3f(0.0f, -0.5f, 0.0f)}},
      CompoundChild{BoxShape{Vec3f(0.5f, 3.5f, 8.5f)},
                    Transform{Vec3f(-8.0f, 3.5f, 0.0f)}},
      CompoundChild{BoxShape{Vec3f(0.5f, 3.5f, 8.5f)},
                    Transform{Vec3f(8.0f, 3.5f, 0.0f)}},
      CompoundChild{BoxShape{Vec3f(7.5f, 3.5f, 0.5f)},
                    Transform{Vec3f(0.0f, 3.5f, -8.0f)}},
      CompoundChild{BoxShape{Vec3f(7.5f, 3.5f, 0.5f)},
                    Transform{Vec3f(0.0f, 3.5f, 8.0f)}},
  });
  world.createStaticBody(std::move(openBox), Transform{}, 0.5f, 0.3f);

  CompoundShape hammer =
      CompoundShape({
          CompoundChild{BoxShape{Vec3f(0.15f, 0.6f, 0.15f)}, Transform{}, 1.0f},
          CompoundChild{BoxShape{Vec3f(0.5f, 0.2f, 0.25f)},
                        Transform{Vec3f(0.0f, 0.8f, 0.0f)}, 3.0f},
          CompoundChild{SphereShape{0.25f},
                        Transform{Vec3f(0.0f, -0.75f, 0.0f)}, 1.0f},
      })
          .recentered();

  Scene scene;
  scene.min = Vec3f(-200.0f, -600.0f, -200.0f);
  scene.max =
      Vec3f(200.0f, 7.0f + static_cast<float>(n) * 0.5f + 50.0f, 200.0f);

  std::mt19937 rng(0x5EED);
  for (int i = 0; i < n; ++i) {
    float x = static_cast<float>(static_cast<int>(rng() % 800) - 400) / 100.0f;
    float z = static_cast<float>(static_cast<int>(rng() % 800) - 400) / 100.0f;
    float y = 7.0f + static_cast<float>(i) * 0.5f;
    Transform t{Vec3f(x, y, z), Quatf(1.0f, 0.0f, 0.0f, 0.0f)};

    BodyHandle handle;
    switch (i % 4) {
      case 0:
        handle =
            world.createDynamicBody(SphereShape{0.5f}, t, 1.0f, 0.5f, 0.5f);
        break;
      case 1:
        handle = world.createDynamicBody(BoxShape{Vec3f(0.5f, 0.5f, 0.5f)}, t,
                                         1.0f, 0.5f, 0.3f);
        break;
      case 2:
        handle = world.createDynamicBody(CapsuleShape{0.35f, 0.45f}, t, 1.0f,
                                         0.5f, 0.4f);
        break;
      default:
        handle = world.createDynamicBody(hammer, t, 2.0f, 0.5f, 0.3f);
        break;
    }
    scene.bodies.push_back(handle);
  }
  return scene;
}

}  // namespace scenes
