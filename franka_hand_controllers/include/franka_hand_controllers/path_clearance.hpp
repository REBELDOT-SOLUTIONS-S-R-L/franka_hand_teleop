#pragma once
#include <moveit/planning_scene/planning_scene.hpp>

namespace franka_hand_controllers {
// We need proof of >=1 mm clearance, not the exact distance to distant meshes.
// Keep all robot links, attached bodies, world objects and the existing ACM.
inline bool path_clearance(const planning_scene::PlanningScene& scene,
                           moveit::core::RobotState& state) {
  state.updateCollisionBodyTransforms();
  collision_detection::DistanceRequest request;
  request.acm = &scene.getAllowedCollisionMatrix();
  request.enableGroup(scene.getRobotModel());
  request.distance_threshold = .001;
  collision_detection::DistanceResult world, self;
  scene.getCollisionEnv()->distanceRobot(request, world, state);
  scene.getCollisionEnv()->distanceSelf(request, self, state);
  return !world.collision && !self.collision &&
    world.minimum_distance.distance >= .001 && self.minimum_distance.distance >= .001;
}
}  // namespace franka_hand_controllers
