#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <geometric_shapes/shapes.h>
#include "franka_hand_controllers/path_clearance.hpp"

namespace {
planning_scene::PlanningScenePtr scene() {
  const std::string xml = R"(<robot name="clearance_test">
    <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
    <link name="tool"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
    <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>
      <axis xyz="1 0 0"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  </robot>)";
  auto urdf = urdf::parseURDF(xml);
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, "<robot name='clearance_test'/>");
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto result = std::make_shared<planning_scene::PlanningScene>(model);
  result->getAllowedCollisionMatrixNonConst().setEntry("base", "tool", false);
  return result;
}

bool original(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state) {
  return scene.distanceToCollision(state)>=.001 &&
    scene.getCollisionEnv()->distanceSelf(state,scene.getAllowedCollisionMatrix())>=.001;
}

TEST(PathClearance, RetainsWorldAndSelfClearanceAndAllowedPairs) {
  auto s=scene();
  s->getWorldNonConst()->addToObject("obstacle", shapes::ShapeConstPtr(new shapes::Box(.1,.1,.1)),
    Eigen::Translation3d(2.,0.,0.)*Eigen::Isometry3d::Identity());
  auto state=s->getCurrentState();
  // World near/collision and self near/collision cases, both sides of 1 mm.
  for(double q:{1.,1.8985,1.8995,1.95,.1015,.1005,.05}) {
    state.setVariablePosition("slide",q); state.update();
    EXPECT_EQ(franka_hand_controllers::path_clearance(*s,state),original(*s,state)) << q;
  }
  state.setVariablePosition("slide",1.8995); state.update();
  EXPECT_FALSE(franka_hand_controllers::path_clearance(*s,state));
  s->getAllowedCollisionMatrixNonConst().setEntry("tool","obstacle",true);
  EXPECT_TRUE(franka_hand_controllers::path_clearance(*s,state));
  state.setVariablePosition("slide",.1005); state.update();
  EXPECT_FALSE(franka_hand_controllers::path_clearance(*s,state));
  s->getAllowedCollisionMatrixNonConst().setEntry("base","tool",true);
  EXPECT_TRUE(franka_hand_controllers::path_clearance(*s,state));
}
}  // namespace
