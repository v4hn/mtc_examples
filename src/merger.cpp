#include <moveit/task_constructor/task.h>
#include <ros/ros.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>

using namespace moveit::task_constructor;

/* CurrentState -> Alternatives(MoveRelative(Cart), MoveRelative(Cart) )*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "mtc_test");

  Task t("merge arms");
  t.loadRobotModel();

  assert(t.getRobotModel()->getName() == "panda");

  Stage *current_state = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  current_state = initial.get();
  t.add(std::move(initial));

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setJumpThreshold(0.0);

  auto merger = std::make_unique<Alternatives>();
  {
    auto stage =
        std::make_unique<stages::MoveRelative>("lower object left", cartesian);
    stage->setMarkerNS("lower_object");
    stage->setIKFrame("l_gripper_tool_frame");
    stage->setGroup("left_arm");
    stage->setMinMaxDistance(.03, .13);
    // Set downward direction
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "base_footprint";
    vec.vector.z = -1.0;
    stage->setDirection(vec);

    // stage->restrictDirection(PropagatingEitherWay::FORWARD);
    merger->insert(std::move(stage));
  }
  {
    auto stage =
        std::make_unique<stages::MoveRelative>("lower object right", cartesian);
    stage->setMarkerNS("lower_object");
    stage->setIKFrame("rh_palm");
    stage->setGroup("right_arm_and_hand");
    stage->setMinMaxDistance(.03, .13);
    // Set downward direction
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "base_footprint";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    // stage->restrictDirection(PropagatingEitherWay::FORWARD);
    // TODO: could be inferred even if only one is present

    merger->insert(std::move(stage));
  }

  t.add(std::move(merger));

  try {
    t.init();
    std::cout << t << std::endl;
  } catch (const InitStageException &e) {
    std::cout << e << std::endl;
  }
  return 0;
}
