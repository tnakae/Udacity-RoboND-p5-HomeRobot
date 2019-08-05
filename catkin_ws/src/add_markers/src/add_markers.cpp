#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

double pose_x = 0.;
double pose_y = 0.;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  pose_x = msg->pose.pose.position.x;
  pose_y = msg->pose.pose.position.y;
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n1;
  ros::Publisher marker_pub = n1.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::NodeHandle n2;
  ros::Subscriber sub_pose = n2.subscribe("odom", 100, poseCallback);  
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set marker publish phase
  //  Phase 0 : show a marker on pickup zone
  //  Phase 1 : no marker is shown
  //  Phase 2 : show a marker on drop off zone (loop infinitely)
  int marker_phase = 0;
  
  while (ros::ok()) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    if (marker_phase == 1) {
      marker.action = visualization_msgs::Marker::DELETE;
    } else {
      marker.action = visualization_msgs::Marker::ADD;
      if (marker_phase == 0) {
	// pickup zone
	marker.pose.position.x = -0.5;
	marker.pose.position.y = -4.0;
	marker.pose.position.z = 0.0;
      } else {
	// drop off zone
	marker.pose.position.x = -0.5;
	marker.pose.position.y = -1.0;
	marker.pose.position.z = 0.0;
      }
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
    }

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok())
	return 0;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    ros::Rate sleep_rate(100);
    for (int counter = 0; counter < 20; counter++) {
      ros::spinOnce();
      sleep_rate.sleep();
    }
    
    if (marker_phase == 0) {
      double dx = pose_x + 0.5;
      double dy = pose_y + 4.0;
      double dist_square = dx * dx + dy * dy;
      ROS_INFO("dist^2 from pickup zone : %.2lf", dist_square);
      ROS_INFO(" x = %.2lf, y=%.2lf", pose_x, pose_y);
      if (dist_square < 0.75) {
	marker_phase++;
	ROS_INFO("picked up!");
      }
    } else if (marker_phase == 1) {
      double dx = pose_x + 0.5;
      double dy = pose_y + 1.0;
      double dist_square = dx * dx + dy * dy;
      ROS_INFO("dist^2 from drop off zone : %.2lf", dist_square);
      ROS_INFO(" x = %.2lf, y=%.2lf", pose_x, pose_y);
      if (dist_square < 0.75) {
	marker_phase++;
	ROS_INFO("dropped off!");
      }
    }
  }
}
