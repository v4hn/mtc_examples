#include <memory>

#include <ros/ros.h>

#include <tf2/utils.h>
#include <tf2/convert.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>

namespace mtc = moveit::task_constructor;
using namespace mtc;
using namespace mtc::stages;

class ComputeIKSeed : public Generator
{
public:
	ComputeIKSeed() : Generator{"ComputeIKSeed"} {}

	planning_scene::PlanningScenePtr scene;

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		scene = [&]() {
			auto s { std::make_shared<planning_scene::PlanningScene>(robot_model) };
			s->getCurrentStateNonConst().setToDefaultValues("panda_arm", "ready");
			s->processCollisionObjectMsg([](){
				moveit_msgs::CollisionObject co;
				co.id= "wall";
				co.operation= co.ADD;
				co.header.frame_id= "panda_link0";
				co.pose.position.z= .6;
				co.pose.orientation.w= 1.0;
				co.primitive_poses.push_back( [](){ geometry_msgs::Pose p; p.orientation.w = 1.0; return p; }() );
				co.primitives.push_back( [](){ shape_msgs::SolidPrimitive p; p.type=p.BOX; p.dimensions = { .3, 0.3, .05 }; return p; }() );
				return co;
			}());
			return s;
		}();
	}

	bool canCompute() const override { return scene != nullptr; }
	void compute() override {
		InterfaceState is{ scene };
		is.properties().set("target_pose", []() {
			geometry_msgs::PoseStamped ps;
			ps.header.frame_id= "panda_link0";
			tf2::convert([](){ tf2::Quaternion q; q.setRPY(M_PI, 0.0, 0.0); return q; }(), ps.pose.orientation);
			//ps.pose.orientation.w= 1.0;
			ps.pose.position.x= .3;
			ps.pose.position.z= .3;
			return ps;
		}());
		spawn(std::move(is), 0.0);

		scene.reset();
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "broken_ik");

	Task t{"broken_ik"};
	t.loadRobotModel();

	auto seed { std::make_unique<ComputeIKSeed>()};

	auto cik { std::make_unique<ComputeIK>("IK", std::move(seed)) };
	cik->setGroup("panda_arm");
	cik->setIKFrame("panda_hand");
	cik->setMaxIKSolutions(32);
	cik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"}); // TODO(v4hn): all alternatives to this line are much more complex
	t.add(std::move(cik));

	try {
		t.init();
	}  catch (const InitStageException& e) {
		ROS_FATAL_STREAM(e);
	}
	t.plan();

	ros::spin();

	return 0;
}
