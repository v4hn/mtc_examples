#include <moveit/task_constructor/task.h>
#include <ros/ros.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>

// hack to get access to correct outer interface state
#define protected public
#include <moveit/task_constructor/container_p.h>
#undef protected

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

using namespace moveit::task_constructor;

class ModifyingWrapper : public WrapperBase {
public:
	ModifyingWrapper(const std::string& name, Stage::pointer&& child = nullptr) :
		WrapperBase(name, std::move(child)){}

	void onNewSolution(const SolutionBase& s) override {
		// require SubTrajectory
		auto* sub_trajectory { dynamic_cast<const SubTrajectory*>(&s)};
		if(!sub_trajectory){
			liftSolution(s, std::numeric_limits<double>::infinity(), "Received solution is no SubTrajectory. Nothing else is supported.");
		}

		// copy&modify trajectory
		auto trajectory { std::make_shared<robot_trajectory::RobotTrajectory>(*sub_trajectory->trajectory(), true) };
		trajectory_processing::TimeOptimalTrajectoryGeneration totg;
		totg.computeTimeStamps(*trajectory);

		// forward cost, comment and markers from other solution
		SubTrajectory modified_solution { trajectory, s.cost(), s.comment() };
		modified_solution.markers() = s.markers();

		// sendForward requires a *new* end state, so just use a diff of the old end scene
		planning_scene::PlanningScenePtr new_end { s.end()->scene()->diff() };

		// TODO: this is a dirty hack and we need a proper interface to "lift" modified solutions
		auto& start { *pimpl()->internalToExternalMap().find(s.start())->second };

		// This restricts the modifyingWrapper to use in PropagateForwards contexts
		// TODO: we do not yet have a good interface to handle the direction-agnostic case
		sendForward(start, new_end, std::move(modified_solution));
	}
};


/** CurrentState -> MoveTo(SamplingPlanner) */
int main(int argc, char **argv) {
  ros::init(argc, argv, "mtc_test");

  Task t { "test" };
  t.loadRobotModel();

  assert(t.getRobotModel()->getName() == "panda");

  ros::Duration(.5).sleep();

  auto scene { std::make_shared<planning_scene::PlanningScene>(t.getRobotModel()) };
  auto& robot_state { scene->getCurrentStateNonConst() };
  robot_state.setToDefaultValues();
  robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "extended");

  auto initial { std::make_unique<stages::FixedState>("start") };
  initial->setState(scene);
  t.add(std::move(initial));

  auto pipeline { std::make_shared<solvers::PipelinePlanner>() };

  {
	 auto move_to { std::make_unique<stages::MoveTo>("move to side", pipeline) };
	 move_to->setGroup("panda_arm");
	 move_to->setGoal("ready");

	 auto wrapper { std::make_unique<ModifyingWrapper>("modify trajectory", std::move(move_to)) };

	 t.add(std::move(wrapper));
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
