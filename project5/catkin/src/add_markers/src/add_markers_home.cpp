#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

ros::Publisher marker_pub;
ros::Subscriber marker_sub;
visualization_msgs::Marker marker;

void initMarker(){
    // Set frame id as /map
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;

    // Fixed marker pose
    marker.pose.position.z = 0.05;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker, 0.3m for each side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set full color opacity
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();
}

// Goal status callback event function
void goalStatusCallback(const std_msgs::String& msg)
{
    ROS_INFO("Status received: [%s]", msg.data.c_str());
  
    if(msg.data == "is-picked-up")
    {
        // Hide marker
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
    }
    else if(msg.data == "is-drop")
    {
        marker.color.b = 0.0f;
        marker.color.g = 1.0f;
        marker.pose.position.x = 5.0;
        marker.pose.position.y = 0.5;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
    }
    else
    {
        ROS_WARN("No goal status");
    }
}


int main( int argc, char** argv )
{
    // Initialize the add_markers node
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Subscribe from pick objects
    marker_sub = n.subscribe("goal_status", 1, goalStatusCallback);

    // Init marker
    initMarker();

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

    ROS_INFO("add_markers node is subscribed");

    // Publish pick up marker
    marker.pose.position.x = -1.0;
    marker.pose.position.y = 1.0;
    marker.color.b = 1.0f;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);    

    ros::spin();
}