#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"

using namespace std;

class drive_robot
{
private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	geometry_msgs::Twist base_command;

public:
	tf::TransformListener listener;
	tf::StampedTransform transform;
	float x_history[3],y_history[3];
	bool stop;

	float x_disp,y_disp;

	drive_robot() : listener(ros::Duration(30.0))
	{
		pub = nh.advertise<geometry_msgs::Twist>("base_controller/command",1);
		x_history[0] = x_history[1] = x_history[2] = 0.0;
		y_history[0] = y_history[1] = y_history[2] = 0.0;
		stop = false;
	}

	~drive_robot(){}

	void init_velocities()
	{
		base_command.linear.x = 0.0;
		base_command.linear.y = 0.0;
		base_command.linear.z = 0.0;
		base_command.angular.x = 0.0;
		base_command.angular.y = 0.0;
		base_command.angular.z = 0.0;
	}
	void transform_update()
	{
		try
		{
			ros::Time now = ros::Time::now();
			//listener.waitForTransform("/base_footprint","/my_frame",now,ros::Duration(1.0));
			//listener.lookupTransform("/base_footprint","/my_frame",now,transform);
			listener.waitForTransform("/base_footprint","/torso_1",now,ros::Duration(1.0));
			string error_msg;
			cout << "Latest common time: " << listener.getLatestCommonTime("/base_footprint","/torso_1",now,&error_msg) << endl;
			cout << "Error: " << error_msg << endl;
			listener.lookupTransform("/base_footprint","/torso_1",now,transform);
			

			//Conditions
			x_disp = transform.getOrigin().getX();
			y_disp = transform.getOrigin().getY();

			//Maintaining Histories
			x_history[0] = x_history[1];
			x_history[1] = x_history[2];
			x_history[2] = x_disp;

			y_history[0] = y_history[1];
			y_history[1] = y_history[2];
			y_history[2] = y_disp;

			if( (x_history[0] == x_history[1]) && (x_history[1] == x_history[2])
			   && ( y_history[0] == y_history[1] ) && ( y_history[1] == y_history[2] ) )
			{
				std::cout << "---Transform is old---" << std::endl;
				//stop = true;
				stop = false;
			}
			else
			{
				stop = false;
			}

			std::cout
					<< now << "---"
			  	    << transform.stamp_ << "  "
					<< transform.getOrigin().getX()
					<< " "
					<< transform.getOrigin().getY()
					<<
			std::endl;

			std::cout
			<< x_history[0] << " " << x_history[1] << " " <<  x_history[2]
			                                                    << std::endl;
		}
		catch(tf::TransformException& ex)
		{
			stop = true;
			//Initializing Velocities
			ROS_ERROR("%s",ex.what());
			cout << "An exception has occurred" << endl;
		}
	}

	void take_decision()
	{
		if(x_disp > 2.6 && !stop )
		{
			std::cout << " in x_disp > 0.5 " << std::endl;
			base_command.linear.x = 0.3;
			base_command.linear.y = 0.0;
			base_command.linear.z = 0.0;
			base_command.angular.x = 0.0;
			base_command.angular.y = 0.0;
			base_command.angular.z = 0.0;
		}
		else if(x_disp < -0.5 && !stop )
		{
			std::cout << " in x_disp < -0.5 " << std::endl;
			base_command.linear.x = -0.3;
			base_command.linear.y = 0.0;
			base_command.linear.z = 0.0;
			base_command.angular.x = 0.0;
			base_command.angular.y = 0.0;
			base_command.angular.z = 0.0;
		}
		else if( y_disp > 0.4 && !stop)
		{
			std::cout << " in y_disp > 0.2 " << std::endl;
			base_command.linear.x = 0.0;
			base_command.linear.y = 0.3;
			base_command.linear.z = 0.0;
			base_command.angular.x = 0.0;
			base_command.angular.y = 0.0;
			base_command.angular.z = 0.0;
		}
		else if( y_disp < -0.2 && !stop)
		{
			std::cout << " in y_disp < -0.2 " << std::endl;
			base_command.linear.x = 0.0;
			base_command.linear.y = -0.3;
			base_command.linear.z = 0.0;
			base_command.angular.x = 0.0;
			base_command.angular.y = 0.0;
			base_command.angular.z = 0.0;
		}
		else
		{
			std::cout << " stopping " << std::endl;
			base_command.linear.x = 0.0;
			base_command.linear.y = 0.0;
			base_command.linear.z = 0.0;
			base_command.angular.x = 0.0;
			base_command.angular.y = 0.0;
			base_command.angular.z = 0.0;

		}

	}
	void publish_basecommand()
	{
		pub.publish(base_command);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"move_the_base");
	drive_robot robot;

	//Initializing Velocities
	robot.init_velocities();

	//ros::Rate loop_rate(1);
	while(ros::ok())
		{
			std::cout << "Looping the while loop" << std::endl;
			robot.transform_update(); //Getting latest transforms
			cout << "But we still keep working at it" << endl;
			robot.take_decision(); //Taking decisions, where to go
			robot.publish_basecommand();//Publishing base Commands
		}
}
