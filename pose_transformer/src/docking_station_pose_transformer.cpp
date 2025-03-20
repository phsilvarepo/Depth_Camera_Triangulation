#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher relative_obj_pos_pub;

// Robot's global position
const double robot_x = 0.84948;
const double robot_y = -2.01203;
const double robot_z = 0.77137;

void positionCallback(const geometry_msgs::Point::ConstPtr& object_in_global_frame) {
        // Compute relative position to robot
        geometry_msgs::Point object_pos_relative_to_robot;
        object_pos_relative_to_robot.x = object_in_global_frame->x - robot_x;
        object_pos_relative_to_robot.y = object_in_global_frame->y - robot_y;
        object_pos_relative_to_robot.z = object_in_global_frame->z - robot_z;

        // Publish relative position
        relative_obj_pos_pub.publish(object_pos_relative_to_robot);

        ROS_INFO_STREAM("Object's position relative to robot: x=" << object_pos_relative_to_robot.x
                        << ", y=" << object_pos_relative_to_robot.y
                        << ", z=" << object_pos_relative_to_robot.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_pose_transformer");
    ros::NodeHandle nh;

    // Create a ROS subscriber to listen for object pose in the camera frame
    ros::Subscriber relative_obj_pos_sub = nh.subscribe("detected_object_pos", 10, positionCallback);
    relative_obj_pos_pub = nh.advertise<geometry_msgs::Point>("/docking_station/detected_object", 10);

    ros::spin();
    return 0;
}
