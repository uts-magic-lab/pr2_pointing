#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"

using namespace std;

void getCoordinatesAtZ(float x1,float y1,float z1,float x2,float y2,float z2,float& x, float& y, float z)
{
	//(x1,y1,z1)->from (in base_footprint)
	//(x2,y2,z2)->to  (in base_footprint)
	//x -> abscissa of resultant point (in base_footprint)
	//y -> ordinate of resultant point (in base_footprint)
	//z -> "0" in case of base_footprint
	//A line will be drawn from (x2,y2,z2) to (x,y,z) in base_footprint
	
	float t = (z-z1)/(z2-z1);
	x = (t * (x2-x1) ) + x1;
	y = (t * (y2-y1) ) + y1; 
	
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes_extension");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher marker_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  geometry_msgs::Point my_point[2];
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::TransformListener listener_;
  tf::StampedTransform transform_;
  float x,y,z;
  z=0.0;
  
  //For broadCasting a frame
  ros::Rate r_transform(100);
  tf::TransformBroadcaster broadcaster;
  
  //float x1=atof(argv[1]);
  //float y1=atof(argv[2]);
  //float z1=atof(argv[3]);
  //float x2=atof(argv[4]);
  //float y2=atof(argv[5]);
  //float z2=atof(argv[6]);
  
  //my_point[0].x = x1;
  //my_point[0].y = y1;
  //my_point[0].z = z1;
  
  //my_point[1].x = x2;
  //my_point[1].y = y2;
  //my_point[1].z = z2;
  
  // Set shape type to be a Arrow
  uint32_t shape = visualization_msgs::Marker::ARROW;

  while (ros::ok())
  {
	
	int person = 1;
	bool success = false;
	ros::Time now;
	
	while (!success) {
		try {
			now = ros::Time::now();
			
			listener.waitForTransform("/base_footprint",std::string("/left_elbow_") + char('0' + person),now,ros::Duration(1.0));
			listener.lookupTransform("/base_footprint",std::string("/left_elbow_") + char('0' + person),now,transform);
			
			//ros::Time now_ = ros::Time::now();
			listener_.waitForTransform("/base_footprint",std::string("/left_hand_") + char('0' + person),now,ros::Duration(1.0));
			listener_.lookupTransform("/base_footprint",std::string("/left_hand_") + char('0' + person),now,transform_);
			success = true;
		}
		catch (tf::TransformException& ex)
		{
			person = person % 4 + 1;
			ROS_ERROR("%s",ex.what());
			cout << "person:" << person << std::endl;
		}
	}
	
	
			std::cout
					<< now << "base_footprint->leb1"
			  	    << transform.stamp_ << "  "
					<< transform.getOrigin().getX()
					<< " "
					<< transform.getOrigin().getY()
					<< " "
					<< transform.getOrigin().getZ()
					<<
			std::endl;
			std::cout
					<< now << "base_footprint->leb2"
			  	    << transform_.stamp_ << "  "
					<< transform_.getOrigin().getX()
					<< " "
					<< transform_.getOrigin().getY()
					<< " "
					<< transform_.getOrigin().getZ()
					<<
			std::endl;
			
			float x1_from = transform.getOrigin().getX();
			float y1_from = transform.getOrigin().getY();
			float z1_from = transform.getOrigin().getZ();
			
			float x2_to = transform_.getOrigin().getX();
			float y2_to = transform_.getOrigin().getY();
			float z2_to = transform_.getOrigin().getZ();
			
			getCoordinatesAtZ(x1_from,y1_from,z1_from,x2_to,y2_to,z2_to,x,y,z);
			my_point[0].x = x2_to;
			my_point[0].y = y2_to;
			my_point[0].z = z2_to;
  
			my_point[1].x = x;
			my_point[1].y = y;
			my_point[1].z = z;
			
			broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(x,y,z)),
			ros::Time::now(),"base_footprint","destination_frame"));
			//r_transform.sleep();
			
	
	
    visualization_msgs::Marker marker, connecting_givenframes;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //marker.header.frame_id = "/torso_lift_link";
    marker.header.frame_id = "/base_footprint";
    connecting_givenframes.header.frame_id = "/base_footprint";
    
    marker.header.stamp = ros::Time::now();
	connecting_givenframes.header.stamp = ros::Time::now();
	
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes_ext";
    marker.id = 0;

	connecting_givenframes.ns = "basic_shapes_original";
    connecting_givenframes.id = 0;

    // Set the marker type.  
    marker.type = shape;
    connecting_givenframes.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    connecting_givenframes.action = visualization_msgs::Marker::ADD;

	//Arrow's Starting and ending Points
	marker.points = std::vector<geometry_msgs::Point>(2);
	marker.points[0].x = my_point[0].x;
	marker.points[0].y = my_point[0].y;
	marker.points[0].z = my_point[0].z;
	marker.points[1].x = my_point[1].x;
	marker.points[1].y = my_point[1].y;
	marker.points[1].z = my_point[1].z;
	
	connecting_givenframes.points = std::vector<geometry_msgs::Point>(2);
	connecting_givenframes.points[0].x = transform.getOrigin().getX();
	connecting_givenframes.points[0].y = transform.getOrigin().getY();
	connecting_givenframes.points[0].z = transform.getOrigin().getZ();
	connecting_givenframes.points[1].x = transform_.getOrigin().getX();
	connecting_givenframes.points[1].y = transform_.getOrigin().getY();
	connecting_givenframes.points[1].z = transform_.getOrigin().getZ();
	
	
	
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.03;
    marker.scale.y = 0.05;
    marker.scale.z = 0.0;
    
    connecting_givenframes.scale.x = 0.03;
    connecting_givenframes.scale.y = 0.05;
    connecting_givenframes.scale.z = 0.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    connecting_givenframes.color.r = 1.0f;
    connecting_givenframes.color.g = 0.0f;
    connecting_givenframes.color.b = 1.0f;
    connecting_givenframes.color.a = 1.0;

    marker.lifetime = ros::Duration();
    connecting_givenframes.lifetime = ros::Duration();

    // Publish the markers
    marker_pub.publish(marker);
    marker_pub_.publish(connecting_givenframes);
    

    //r.sleep();
  }
}
