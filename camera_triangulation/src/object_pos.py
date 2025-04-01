import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import message_filters
from geometry_msgs.msg import Point
from std_msgs.msg import String
import ast
'''
# Initialize CvBridge
bridge = CvBridge()

# Global variables for object detection coordinates
u, v = None, None

# Define publisher for object position
global_position_pub = rospy.Publisher('/detected_object_pos', Point, queue_size=10)

# Define callback for single depth camera
def synchronized_callback(depth_image_msg):
    #Processes depth image using the latest bounding box center (u, v).
    global u, v
    
    if u is None or v is None:
        rospy.logwarn("No valid bounding box coordinates received yet.")
        return

    try:
        # Convert ROS Image message to OpenCV image
        depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
        
        # Example coordinates of the object's center (replace with actual object detection coordinates)
        #u, v = 832, 576  # Object center in the image

        # Extract the depth value at these coordinates
        Z = depth_image[v, u] #* 100  # Ensure units are in meters or centimeters as per your depth sensor configuration
        if Z == 0:  # Check if depth is not valid
            rospy.logwarn("Invalid depth value at coordinates ({}, {}): {}".format(u, v, Z))
            return
        
        print("Depth Position:", Z)

        # Camera parameters (replace with actual calibration data)
        camera_params = {'fx': 1108.512451171875, 'fy': 1108.512451171875, 'cx': 640.0, 'cy': 360.0}

        X = (u - camera_params['cx']) * Z / camera_params['fx']
        Y = (v - camera_params['cy']) * Z / camera_params['fy']
        
        # 3D position in the camera frame
        object_position_3D = np.array([X, Y, Z])
        print("Object 3D Position in Camera Frame:", object_position_3D)

        # If you want the object position in the global frame (robot's world frame), 
        # you would apply the camera's global position and orientation.
        # Example global position of the camera (replace with actual pose)
        #camera_global_pos = np.array([1.5, -2.01203, 1.61432])  # Global position of the camera FACTORY
        camera_global_pos = np.array([0.65052, 0, 0.84295])
        camera_global_quat = (0.0, 0.0, 0.0, -1.0)  # Global orientation of the camera

        # Compute global position of the object
        object_position_global = Point()
        object_position_global.x = camera_global_pos[0] - object_position_3D[0]
        object_position_global.y = camera_global_pos[1] + object_position_3D[1]
        object_position_global.z = camera_global_pos[2] - object_position_3D[2]

        print(object_position_global)

        #For test purposes
        #object_position_global.x = 0.69432
        #object_position_global.y = 0.0
        #object_position_global.z = 0.09496
        
        rospy.loginfo("Object Global 3D Position: ({}, {}, {})".format(
            object_position_global.x, object_position_global.y, object_position_global.z))
        
        # Publish global position
        global_position_pub.publish(object_position_global)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def synchronized_callback(depth_image_msg):
    global u, v
    
    if u is None or v is None:
        rospy.logwarn("No valid bounding box coordinates received yet. Not publishing.")
        return  # Prevent publishing if no object is detected

    try:
        # Convert ROS Image message to OpenCV image
        depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

        # Extract the depth value at these coordinates
        Z = depth_image[v, u]  
        if Z == 0:  
            rospy.logwarn("Invalid depth value at coordinates ({}, {}): {}".format(u, v, Z))
            return  # Prevent publishing if depth value is invalid

        print("Depth Position:", Z)

        # Camera parameters
        camera_params = {'fx': 1108.512451171875, 'fy': 1108.512451171875, 'cx': 640.0, 'cy': 360.0}

        X = (u - camera_params['cx']) * Z / camera_params['fx']
        Y = (v - camera_params['cy']) * Z / camera_params['fy']

        object_position_global = Point()
        object_position_global.x = 0.65052 - X
        object_position_global.y = 0 + Y
        object_position_global.z = 0.84295 - Z

        rospy.loginfo("Object Global 3D Position: ({}, {}, {})".format(
            object_position_global.x, object_position_global.y, object_position_global.z))
        
        # Publish ONLY if a valid object position exists
        global_position_pub.publish(object_position_global)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def bounding_boxes_callback(msg):
    """Extracts the first object's bounding box center (u, v) from the YOLO detection."""
    global u, v
    
    try:
        # Convert string message to list of dictionaries
        bounding_boxes = ast.literal_eval(msg.data)  # Converts string to list

        if bounding_boxes and isinstance(bounding_boxes, list):
            # Extract the first detected object's center
            first_object = bounding_boxes[0]
            u = int(first_object['center_x'])
            v = int(first_object['center_y'])
            
            rospy.loginfo("Received Bounding Box: u={}, v={}".format(u, v))
    
    except Exception as e:
        rospy.logerr("Error parsing bounding box data: {}".format(e))
    
# Function to convert quaternion to rotation matrix
def quaternion_to_rotation_matrix(quat):
    from scipy.spatial.transform import Rotation as R
    rotation = R.from_quat(quat)
    return rotation.as_matrix()

if __name__ == '__main__':
    rospy.init_node('object_3d_position_node', anonymous=True)

    # Define subscriber for the depth
    image_sub = rospy.Subscriber("/dock_camera/depth_img", Image, synchronized_callback)
    bounding_boxes_sub = rospy.Subscriber("/ultralytics/detection/bounding_boxes", String, bounding_boxes_callback)

    rospy.spin()
'''

# Initialize CvBridge
bridge = CvBridge()

# Global variables for object detection coordinates
u, v = None, None

# Define publisher for object position
global_position_pub = rospy.Publisher('/detected_object_pos', Point, queue_size=10)

def synchronized_callback(depth_image_msg):
    """Processes depth image using the latest bounding box center (u, v)."""
    global u, v

    # Do not proceed if no valid detection exists
    if u is None or v is None:
        rospy.logwarn("No valid bounding box coordinates. Stopping publishing.")
        return  

    try:
        # Convert ROS Image message to OpenCV image
        depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

        # Extract the depth value at these coordinates
        Z = depth_image[v, u]  
        if Z == 0:  
            rospy.logwarn("Invalid depth at ({}, {}): {}".format(u, v, Z))
            return  

        print("Depth Position:", Z)

        # Camera parameters
        camera_params = {'fx': 1108.512451171875, 'fy': 1108.512451171875, 'cx': 640.0, 'cy': 360.0}

        X = (u - camera_params['cx']) * Z / camera_params['fx']
        Y = (v - camera_params['cy']) * Z / camera_params['fy']

        object_position_global = Point()
        object_position_global.x = 0.65052 - X
        object_position_global.y = 0 + Y
        object_position_global.z = 0.84295 - Z

        rospy.loginfo("Object Global 3D Position: ({}, {}, {})".format(
            object_position_global.x, object_position_global.y, object_position_global.z))
        
        # Publish global position
        global_position_pub.publish(object_position_global)

        # Reset u, v after publishing to prevent stale detections
        u, v = None, None  

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def bounding_boxes_callback(msg):
    """Extracts the first object's bounding box center (u, v) from YOLO detection."""
    global u, v

    try:
        bounding_boxes = ast.literal_eval(msg.data)  

        if bounding_boxes and isinstance(bounding_boxes, list) and len(bounding_boxes) > 0:
            first_object = bounding_boxes[0]
            u = int(first_object['center_x'])
            v = int(first_object['center_y'])
            rospy.loginfo("Received Bounding Box: u={}, v={}".format(u, v))
        else:
            # Reset u, v if no detections are received
            u, v = None, None  

    except Exception as e:
        rospy.logerr("Error parsing bounding box data: {}".format(e))
        u, v = None, None  # Reset in case of error

if __name__ == '__main__':
    rospy.init_node('object_3d_position_node', anonymous=True)

    # Define subscribers
    image_sub = rospy.Subscriber("/dock_camera/depth_img", Image, synchronized_callback)
    bounding_boxes_sub = rospy.Subscriber("/ultralytics/detection/bounding_boxes", String, bounding_boxes_callback)

    rospy.spin()