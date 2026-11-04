#include <gtest/gtest.h>

#include <render/Camera.hpp>

namespace {

render::Vec2f projectToViewport(const render::Mat4f& vp, render::Vec3f p,
                                render::Vec2f size) {
  render::Vec4f clip = vp * render::Vec4f(p, 1.0f);
  render::Vec2f ndc(clip.x / clip.w, clip.y / clip.w);
  return render::Vec2f((ndc.x + 1.0f) * 0.5f * size.x,
                       (1.0f - ndc.y) * 0.5f * size.y);
}

}  // namespace

TEST(CameraRayTest, PerspectiveRayRoundTripsProjection) {
  render::PerspectiveCamera cam;
  cam.position = render::Vec3f(0.0f, 5.0f, 10.0f);
  cam.target = render::Vec3f(1.0f, 0.0f, 0.0f);
  render::Vec2f size(1000.0f, 500.0f);
  cam.aspect = size.x / size.y;
  render::Mat4f vp = cam.viewProjection();

  render::Ray center =
      cam.screenPointToRay(render::Vec2f(500.0f, 250.0f), size);
  render::Vec3f expectedForward = glm::normalize(cam.target - cam.position);
  EXPECT_NEAR(center.origin.x, cam.position.x, 1e-5f);
  EXPECT_NEAR(center.origin.y, cam.position.y, 1e-5f);
  EXPECT_NEAR(center.origin.z, cam.position.z, 1e-5f);
  EXPECT_NEAR(center.direction.x, expectedForward.x, 1e-5f);
  EXPECT_NEAR(center.direction.y, expectedForward.y, 1e-5f);
  EXPECT_NEAR(center.direction.z, expectedForward.z, 1e-5f);

  for (render::Vec2f px : {render::Vec2f(0.0f, 0.0f),
                           render::Vec2f(500.0f, 250.0f),
                           render::Vec2f(1000.0f, 500.0f),
                           render::Vec2f(123.0f, 456.0f)}) {
    render::Ray ray = cam.screenPointToRay(px, size);
    EXPECT_NEAR(glm::length(ray.direction), 1.0f, 1e-5f);
    render::Vec2f back =
        projectToViewport(vp, ray.origin + ray.direction * 7.0f, size);
    EXPECT_NEAR(back.x, px.x, 1e-2f);
    EXPECT_NEAR(back.y, px.y, 1e-2f);
  }
}

TEST(CameraRayTest, OrthographicMapsCornersToWorldExtents) {
  render::OrthographicCamera cam;
  cam.center = render::Vec2f(3.0f, -2.0f);
  cam.halfHeight = 5.0f;
  cam.aspect = 2.0f;
  render::Vec2f size(800.0f, 400.0f);

  render::Ray topLeft = cam.screenPointToRay(render::Vec2f(0.0f), size);
  EXPECT_NEAR(topLeft.origin.x, 3.0f - 10.0f, 1e-5f);
  EXPECT_NEAR(topLeft.origin.y, -2.0f + 5.0f, 1e-5f);
  EXPECT_NEAR(topLeft.direction.z, -1.0f, 1e-6f);

  render::Ray center = cam.screenPointToRay(render::Vec2f(400.0f, 200.0f), size);
  EXPECT_NEAR(center.origin.x, 3.0f, 1e-5f);
  EXPECT_NEAR(center.origin.y, -2.0f, 1e-5f);

  render::Vec2f px(600.0f, 100.0f);
  render::Ray ray = cam.screenPointToRay(px, size);
  render::Vec2f back = projectToViewport(
      cam.viewProjection(), ray.origin + ray.direction * 1.0f, size);
  EXPECT_NEAR(back.x, px.x, 1e-2f);
  EXPECT_NEAR(back.y, px.y, 1e-2f);
}
