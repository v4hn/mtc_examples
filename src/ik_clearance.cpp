#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/cost_terms.h>

using namespace moveit::task_constructor;

/* ComputeIK(FixedState) */
int main(int argc, char **argv) {
  ros::init(argc, argv, "mtc_test");

  Task t("clearance IK");
  t.loadRobotModel();

  auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  auto& robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues();
  robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"),"extended");

  auto initial = std::make_unique<stages::FixedState>();
  initial->setState(scene);

  auto ik = std::make_unique<stages::ComputeIK>();
  ik->insert(std::move(initial));
  ik->setGroup("panda_arm");
  ik->setTargetPose(
           Eigen::Translation3d(.3, 0, .35)
           * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  ik->setMaxIKSolutions(100);

  cost::Clearance cl_cost;
  cl_cost.cumulative = true;
  ik->setCostTerm(cl_cost);

  t.add(std::move(ik));

  try {
	 t.plan(0);
  } catch (const InitStageException &e) {
    std::cout << e << std::endl;
  }

  ros::spin();

  return 0;
}
