#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher pose_pub;

// Function to perform transformation from camera to global frame
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& object_in_camera_frame) {
    // Step 1: Define the camera's pose in the global frame (manually or received)
    geometry_msgs::Pose camera_in_global_frame;
    camera_in_global_frame.position.x = 0.0;  // Example camera position in global frame
    camera_in_global_frame.position.y = 1.0;
    camera_in_global_frame.position.z = 0.9;

    camera_in_global_frame.orientation.x = 0.5;
    camera_in_global_frame.orientation.y = 0.5;
    camera_in_global_frame.orientation.z = 0.5;
    camera_in_global_frame.orientation.w = 0.5;

    try {
        // Step 2: Create a transform from camera pose (global frame)
        geometry_msgs::TransformStamped camera_transform;
        camera_transform.transform.translation.x = camera_in_global_frame.position.x;
        camera_transform.transform.translation.y = camera_in_global_frame.position.y;
        camera_transform.transform.translation.z = camera_in_global_frame.position.z;
        camera_transform.transform.rotation = camera_in_global_frame.orientation;

        /// Step 3: Transform the object's pose from the camera frame to the global frame
        geometry_msgs::PoseStamped object_in_global_frame;

        // Adjust the object's position based on the camera's axis convention
        geometry_msgs::PoseStamped adjusted_object_in_camera_frame = *object_in_camera_frame;

        // Adjust position based on axis conventions (if Y-axis is inverted)
        adjusted_object_in_camera_frame.pose.position.y *= -1; 
        adjusted_object_in_camera_frame.pose.position.z *= -1; 

        // Transform the adjusted pose
        tf2::doTransform(adjusted_object_in_camera_frame, object_in_global_frame, camera_transform);

        // Output the object's global position
        ROS_INFO_STREAM("Object's position in global frame: "
                        << "x=" << object_in_global_frame.pose.position.x
                        << ", y=" << object_in_global_frame.pose.position.y
                        << ", z=" << object_in_global_frame.pose.position.z);

        pose_pub.publish(object_in_global_frame);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform object pose: %s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_pose_transformer");
    ros::NodeHandle nh;

    // Step 4: Create a ROS subscriber to listen for object pose in the camera frame
    ros::Subscriber sub = nh.subscribe("dope/pose_soup", 10, poseCallback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/dope/global_soup_pose", 10);
    // Keep the node running
    ros::spin();
    return 0;
}