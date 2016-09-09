#include <ros/ros.h>

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
class RobotHead
{
private:
	PointHeadClient* point_head_client_;
public:
	RobotHead()
	{
		//point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action",true);
		point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
		while (!point_head_client_->waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for Action Server");
		}
	}

	~RobotHead()
	{
		delete point_head_client_;
	}
	void lookat(std::string frame_id , double x, double y, double z)
	{
		pr2_controllers_msgs::PointHeadGoal goal;

		//The point to be looking is expressed in frame_id
		geometry_msgs::PointStamped point;
		point.header.frame_id = frame_id;
		point.point.x = x;
		point.point.y = y;
		point.point.z = z;
		goal.target = point;

		goal.pointing_frame = "/high_def_frame";

		//take at least 0.5 seconds to get there
		goal.min_duration = ros::Duration(0.5);

		//and go no faster than 1 rad/s
		goal.max_velocity = 1.0;

		goal.pointing_axis.x = 1;
		goal.pointing_axis.y = 0;
		goal.pointing_axis.z = 0;

		point_head_client_->sendGoal(goal);

		//point_head_client_->waitForResult(ros::Duration(2.0));

	}
};
int main(int argc, char** argv)
	{
		//init the ROS node
		ros::init(argc, argv, "robot_driver");

		RobotHead head;
		while (ros::ok())
		{
		  std::cout <<"looking at"<<std::endl;
		  //head.lookat("r_gripper_tool_frame", 0, 0, 0);
		  //head.lookat("odom_combined",5,0,0);
		  head.lookat("head_1",0,0,0);
		  usleep(50000);
		}

	}



