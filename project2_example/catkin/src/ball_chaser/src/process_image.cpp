#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Drive position
enum DrivePosition{
    Stop,
    Left,
    Forward,
    Right
};

                            // Linear_x, Angular_z
float RobotVelocity[4][2] = {   {0.0,       0.0}, // Stop
                                {0.0,       0.5}, // Left
                                {0.5,       0.0}, // Forward
                                {0.0,      -0.5}};// Right

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // Default stop
    DrivePosition position = Stop;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int j = 0;
    for (int i = 0; i < img.height * img.step; i++)
    {
        // Check white ball
        if (img.data[i] == white_pixel)
        {
            float pos = (float)j / img.step;
            // Check left
            if(pos < 0.333)
                position = Left;
            // Check forward
            else if(pos < 0.666)
                position = Forward;
            // Check right
            else
                position = Right;

            ROS_INFO("Drive robot: Linear_x: %f Angular_z: %f", RobotVelocity[position][0],RobotVelocity[position][1]);
            break;
        }
        j = (j + 1) % img.step;
    }

    drive_robot(RobotVelocity[position][0],
                RobotVelocity[position][1]);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}