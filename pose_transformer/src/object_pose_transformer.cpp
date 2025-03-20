#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher global_pose_pub;
ros::Publisher relative_pose_pub;

// Robot's global position
const double robot_x = 0.0;
const double robot_y = 0.0;
const double robot_z = 0.0;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& object_in_camera_frame) {
    // Define the camera's pose in the global frame (manually or received)
    geometry_msgs::Pose camera_in_global_frame;
    /*
    camera_in_global_frame.position.x = 0.0;
    camera_in_global_frame.position.y = 1.0;
    camera_in_global_frame.position.z = 0.9;
    camera_in_global_frame.orientation.x = 0.5;
    camera_in_global_frame.orientation.y = 0.5;
    camera_in_global_frame.orientation.z = 0.5;
    camera_in_global_frame.orientation.w = 0.5;
    */

    camera_in_global_frame.position.x = 0.85;
    camera_in_global_frame.position.y = 0;
    camera_in_global_frame.position.z = 0.09;
    camera_in_global_frame.orientation.x = 0.4912;
    camera_in_global_frame.orientation.y = 0.4912;
    camera_in_global_frame.orientation.z = 0.50865;
    camera_in_global_frame.orientation.w = 0.50865;

    try {
        // Create a transform from camera pose (global frame)
        geometry_msgs::TransformStamped camera_transform;
        camera_transform.transform.translation.x = camera_in_global_frame.position.x;
        camera_transform.transform.translation.y = camera_in_global_frame.position.y;
        camera_transform.transform.translation.z = camera_in_global_frame.position.z;
        camera_transform.transform.rotation = camera_in_global_frame.orientation;

        // Transform the object's pose from the camera frame to the global frame
        geometry_msgs::PoseStamped object_in_global_frame;
        geometry_msgs::PoseStamped adjusted_object_in_camera_frame = *object_in_camera_frame;

        // Adjust position based on axis conventions (if necessary)
        adjusted_object_in_camera_frame.pose.position.y *= -1;
        adjusted_object_in_camera_frame.pose.position.z *= -1;

        // Transform the adjusted pose
        tf2::doTransform(adjusted_object_in_camera_frame, object_in_global_frame, camera_transform);

        // Publish object's global pose
        global_pose_pub.publish(object_in_global_frame);

        // Compute relative position to robot
        geometry_msgs::PoseStamped object_relative_to_robot;
        object_relative_to_robot.header.stamp = ros::Time::now();
        object_relative_to_robot.header.frame_id = "root";
        object_relative_to_robot.pose.position.x = object_in_global_frame.pose.position.x - robot_x;
        object_relative_to_robot.pose.position.y = object_in_global_frame.pose.position.y - robot_y;
        object_relative_to_robot.pose.position.z = object_in_global_frame.pose.position.z - robot_z;

        // Publish relative position
        relative_pose_pub.publish(object_relative_to_robot);

        // Output results
        ROS_INFO_STREAM("Object's position in global frame: x=" << object_in_global_frame.pose.position.x
                        << ", y=" << object_in_global_frame.pose.position.y
                        << ", z=" << object_in_global_frame.pose.position.z);

        ROS_INFO_STREAM("Object's position relative to robot: x=" << object_relative_to_robot.pose.position.x
                        << ", y=" << object_relative_to_robot.pose.position.y
                        << ", z=" << object_relative_to_robot.pose.position.z);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform object pose: %s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_pose_transformer");
    ros::NodeHandle nh;

    // Create a ROS subscriber to listen for object pose in the camera frame
    ros::Subscriber sub = nh.subscribe("dope/pose_soup", 10, poseCallback);
    global_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/dope/global_soup_pose", 10);
    relative_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/dope/relative_soup_pose", 10);

    ros::spin();
    return 0;
}
