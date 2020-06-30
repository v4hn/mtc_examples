#include <moveit/task_constructor/task.h>
#include <ros/ros.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

using namespace moveit::task_constructor;

/** CurrentState -> MoveTo(CartesianPath) */
int main(int argc, char **argv) {
  ros::init(argc, argv, "mtc_test");

  Task t("test");
  t.loadRobotModel();

  Stage *current_state = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  current_state = initial.get();
  t.add(std::move(initial));

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setJumpThreshold(0.0);

  {
    auto move_to = std::make_unique<stages::MoveTo>("move to side", cartesian);
    move_to->setGroup("left_arm");
    move_to->setGoal("left_arm_to_side");
    t.add(std::move(move_to));
  }

  try {
    t.init();
    std::cout << t << std::endl;
    t.plan(1);
  } catch (const InitStageException &e) {
    std::cout << e << std::endl;
  }

  ros::spin();

  return 0;
}
