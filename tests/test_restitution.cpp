#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include <core/World.hpp>

#include "test_helpers.hpp"

using test_helpers::stepWorld;
using test_helpers::totalKineticEnergy;

namespace {
constexpr float kDt = 1.0f / 240.0f;

float measureBounceHeight(const ShapeVariant& shape, float restOffset,
                          float restitution, float h0) {
  World world;
  world.setGravity(Vec3f(0.0f, -9.81f, 0.0f));

  Transform groundT;
  groundT.position = Vec3f(0.0f, -0.5f, 0.0f);
  world.createStaticBody(ShapeVariant{BoxShape{Vec3f(20.0f, 0.5f, 0.0f)}},
                         groundT, /*friction=*/0.0f, /*restitution=*/0.0f,
                         /*constrainTo2D=*/true);

  Transform bodyT;
  bodyT.position = Vec3f(0.0f, restOffset + h0, 0.0f);
  BodyHandle body = world.createDynamicBody(shape, bodyT, /*mass=*/1.0f,
                                            /*friction=*/0.0f, restitution,
                                            /*constrainTo2D=*/true);

  const float restY = restOffset;
  bool wasFalling = false;
  bool bounced = false;
  float apexY = restY;

  for (int i = 0; i < 240 * 6; i++) {
    world.step(kDt);
    float y = world.bodies().position(body).y;
    float vy = world.bodies().linearVelocity(body).y;

    if (!bounced) {
      if (wasFalling && vy > 0.0f) { bounced = true; }
      wasFalling = vy < 0.0f;
    } else {
      if (vy <= 0.0f) { break; }
      apexY = std::max(apexY, y);
    }
  }

  return bounced ? apexY - restY : -1.0f;
}

}  // namespace

TEST(RestitutionTest, DropBounceHeightMatchesRSquaredAcrossShapes) {
  const float restitution = 0.8f;
  const float h0 = 5.0f;
  const float expected = restitution * restitution * h0;

  struct Case {
    std::string name;
    ShapeVariant shape;
    float restOffset;
  };
  std::vector<Case> cases = {
      {"Sphere", ShapeVariant{SphereShape{0.5f}}, 0.5f},
      {"Box", ShapeVariant{BoxShape{Vec3f(0.5f, 0.5f, 0.0f)}}, 0.5f},
      {"Capsule", ShapeVariant{CapsuleShape{0.3f, 0.3f}}, 0.6f},
  };

  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    float bounceHeight =
        measureBounceHeight(c.shape, c.restOffset, restitution, h0);
    ASSERT_GT(bounceHeight, 0.0f) << c.name << " never bounced";
    EXPECT_NEAR(bounceHeight, expected, expected * 0.3f + 0.1f);
  }
}

namespace {

std::pair<float, float> runHeadOnCollision(const ShapeVariant& shapeA,
                                           const ShapeVariant& shapeB,
                                           float restitution) {
  World world;
  world.setGravity(Vec3f(0.0f));

  BodyHandle a =
      world.createDynamicBody(shapeA, Transform{Vec3f(-2.0f, 0.0f, 0.0f)},
                              /*mass=*/1.0f, /*friction=*/0.0f, restitution);
  BodyHandle b =
      world.createDynamicBody(shapeB, Transform{Vec3f(2.0f, 0.0f, 0.0f)},
                              /*mass=*/1.0f, /*friction=*/0.0f, restitution);
  world.bodies().linearVelocity(a) = Vec3f(2.0f, 0.0f, 0.0f);
  world.bodies().linearVelocity(b) = Vec3f(-2.0f, 0.0f, 0.0f);

  std::array<BodyHandle, 2> bodies{a, b};
  float keBefore = totalKineticEnergy(world.bodies(), bodies);
  Vec3f velABefore = world.bodies().linearVelocity(a);

  stepWorld(world, 300, kDt);

  EXPECT_GT(glm::length(world.bodies().linearVelocity(a) - velABefore), 0.1f)
      << "bodies never actually collided -- scenario has no signal";

  float keAfter = totalKineticEnergy(world.bodies(), bodies);
  return {keBefore, keAfter};
}

}  // namespace

TEST(RestitutionTest, EnergyNeverIncreasesForRestitutionBelowOne) {
  struct ShapeCase {
    std::string name;
    ShapeVariant shape;
  };
  std::vector<ShapeCase> shapes = {
      {"Sphere", ShapeVariant{SphereShape{0.5f}}},
      {"Box", ShapeVariant{BoxShape{Vec3f(0.5f)}}},
      {"Capsule", ShapeVariant{CapsuleShape{0.3f, 0.3f}}},
  };
  const std::vector<float> restitutions = {0.0f, 0.3f, 0.7f, 1.0f};

  for (size_t i = 0; i < shapes.size(); i++) {
    for (size_t j = i; j < shapes.size(); j++) {
      std::vector<float> keAfterByRestitution;
      for (float restitution : restitutions) {
        SCOPED_TRACE(shapes[i].name + " vs " + shapes[j].name +
                     " r=" + std::to_string(restitution));
        auto [keBefore, keAfter] =
            runHeadOnCollision(shapes[i].shape, shapes[j].shape, restitution);

        EXPECT_LE(keAfter, keBefore + 1e-4f);
        keAfterByRestitution.push_back(keAfter);
      }
      for (size_t k = 1; k < keAfterByRestitution.size(); k++) {
        EXPECT_GE(keAfterByRestitution[k], keAfterByRestitution[k - 1] - 1e-4f)
            << shapes[i].name << " vs " << shapes[j].name
            << ": KE retained should be non-decreasing in restitution";
      }
    }
  }
}
