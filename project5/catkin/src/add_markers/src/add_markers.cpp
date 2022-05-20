#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.
    marker.pose.position.x = 0.0;
    marker.pose.position.y = -1.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker, 0.5m for each side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set red colour to pick up
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
        return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    // Publish pick up marker
    marker_pub.publish(marker);

    // Pause 5 sec
    ros::Duration(5.0).sleep();

    // Hide marker
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Pause 5 sec
    ros::Duration(5.0).sleep();

    // Publish drop marker
    // set blue
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.b = 1.0f;
    marker.pose.position.x = -1.5;
    marker.pose.position.y = 1.5;
    marker_pub.publish(marker);

    ros::spin();
  
}