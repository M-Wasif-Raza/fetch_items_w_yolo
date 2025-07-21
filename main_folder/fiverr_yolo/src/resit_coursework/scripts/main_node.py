#!/usr/bin/env python3

import rospy
import actionlib
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

# Import navigation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# Import the custom message
from resit_coursework.msg import HotelRequest

# Import YOLO detector
from yolov4 import Detector

# Define room coordinates
ROOM_COORDINATES = {
    'A': {'x': 2.0, 'y': 8.5, 'theta': 0.0},   # Pantry
    'B': {'x': 6.0, 'y': 8.5, 'theta': 0.0},   # Guest room
    'C': {'x': 11.0, 'y': 8.5, 'theta': 0.0},  # Guest room
    'D': {'x': 2.0, 'y': 3.0, 'theta': 0.0},   # Front Desk
    'E': {'x': 6.0, 'y': 3.0, 'theta': 0.0},   # Lobby (start)
    'F': {'x': 11.0, 'y': 3.0, 'theta': 0.0},  # Guest room
}

# SMACH States
class WaitInLobby(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['request_received'],
                             output_keys=['object_requested', 'delivery_room'])
        self.main_node = main_node
        
    def execute(self, userdata):
        rospy.loginfo('Waiting in lobby for hotel request...')
        self.main_node.stop_robot()
        
        while not rospy.is_shutdown() and not self.main_node.has_new_request:
            rospy.sleep(0.1)
        
        if self.main_node.has_new_request:
            userdata.object_requested = self.main_node.current_request.request
            userdata.delivery_room = self.main_node.current_request.room
            self.main_node.has_new_request = False
            
            rospy.loginfo(f'Received request: {self.main_node.current_request.request} to room {self.main_node.current_request.room}')
            return 'request_received'
        
        return 'request_received'

class NavigateToRoom(smach.State):
    def __init__(self, target_room=None):
        smach.State.__init__(self, 
                             outcomes=['arrived', 'failed'],
                             input_keys=['delivery_room'] if target_room is None else [])
        self.target_room = target_room
        
    def execute(self, userdata):
        room = self.target_room if self.target_room else userdata.delivery_room
        rospy.loginfo(f'Navigating to room {room}')
        
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        if not client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn('move_base server not available, simulating navigation')
            time.sleep(2)
            return 'arrived'
        
        # Create navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        coords = ROOM_COORDINATES[room]
        goal.target_pose.pose.position.x = coords['x']
        goal.target_pose.pose.position.y = coords['y']
        goal.target_pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, coords['theta'])
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        client.send_goal(goal)
        result = client.wait_for_result(timeout=rospy.Duration(60.0))
        
        if result:
            rospy.loginfo(f'Successfully reached room {room}')
            time.sleep(1)
            return 'arrived'
        else:
            rospy.logwarn(f'Failed to reach room {room}')
            return 'failed'

class SearchInPantry(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['object_found', 'object_not_found'],
                             input_keys=['object_requested'])
        self.main_node = main_node
        
    def execute(self, userdata):
        object_name = userdata.object_requested
        rospy.loginfo(f'Searching for {object_name} in pantry')
        
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.1)
        
        twist = Twist()
        search_time = 0
        max_search_time = 60
        rate = rospy.Rate(10)
        
        detection_counter = 0
        detection_interval = 20  # Run YOLO every 2 seconds
        
        while search_time < max_search_time and not rospy.is_shutdown():
            # Continuous rotation
            twist.angular.z = 0.2
            vel_pub.publish(twist)
            
            # Periodic YOLO detection
            if detection_counter >= detection_interval:
                if self.main_node.detect_object_with_yolo(object_name):
                    twist.angular.z = 0.0
                    vel_pub.publish(twist)
                    self.announce_object_found(object_name)
                    return 'object_found'
                detection_counter = 0
            else:
                detection_counter += 1
            
            search_time += 0.1
            rate.sleep()
        
        twist.angular.z = 0.0
        vel_pub.publish(twist)
        rospy.logwarn(f'Could not find {object_name} in pantry')
        return 'object_not_found'
    
    def announce_object_found(self, object_name):
        announcement = f"I found the {object_name}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        self.main_node.tts_pub.publish(String(announcement))

class CheckForPerson(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['person_found', 'person_not_found'])
        self.main_node = main_node
        
    def execute(self, userdata):
        rospy.loginfo('Checking for person in room...')
        time.sleep(2)
        
        if self.main_node.detect_person_with_yolo():
            rospy.loginfo('Person detected in room')
            return 'person_found'
        else:
            rospy.loginfo('No person detected in room')
            return 'person_not_found'

class DeliverToGuest(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['delivered'],
                             input_keys=['object_requested'])
        self.main_node = main_node
        
    def execute(self, userdata):
        object_name = userdata.object_requested
        announcement = f"I am delivering your {object_name}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        self.main_node.tts_pub.publish(String(announcement))
        time.sleep(2)
        return 'delivered'

class ReportToFrontDesk(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['reported'],
                             input_keys=['object_requested', 'delivery_room'])
        self.main_node = main_node
        
    def execute(self, userdata):
        object_name = userdata.object_requested
        room = userdata.delivery_room
        announcement = f"The person was not available to deliver the {object_name} in room {room}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        self.main_node.tts_pub.publish(String(announcement))
        time.sleep(2)
        return 'reported'

# Main Hotel Node
class HotelMainNode:
    def __init__(self):
        rospy.init_node('main_node')
        rospy.sleep(2)
        rospy.loginfo("main_node started successfully!")
        
        # Hotel request handling
        self.current_request = HotelRequest()
        self.has_new_request = False
        
        # Camera and YOLO
        self.bridge = CvBridge()
        self.current_image = None
        
        # Initialize YOLO detector
        self.detector = self.init_yolo_detector()
        
        # Publishers and subscribers
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.Subscriber("/hotel_request", HotelRequest, self.hotel_request_callback)
        rospy.Subscriber("/camera/image", Image, self.camera_callback)
        
        rospy.loginfo('Hotel service robot initialized and ready in lobby')
        self.run_hotel_service()
    
    def init_yolo_detector(self):
        try:
            detector = Detector(
                gpu_id=-1,
                config_path='/opt/darknet/cfg/yolov4.cfg',
                weights_path='/opt/darknet/yolov4.weights', 
                lib_darknet_path='/opt/darknet/libdarknet.so',
                meta_path='/opt/darknet/cfg/coco.data'
            )
            rospy.loginfo("YOLO detector initialized successfully")
            return detector
        except Exception as e:
            rospy.logerr(f"Failed to initialize YOLO detector: {e}")
            return None
    
    def camera_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f'Error converting image: {e}')
            self.current_image = None
    
    def detect_object_with_yolo(self, target_object):
        if self.current_image is None or self.detector is None:
            return False
        
        try:
            cv_copy = cv2.resize(self.current_image.copy(), (608, 608))
            detections = self.detector.perform_detect(image_path_or_buf=cv_copy, show_image=False)
            
            for detection in detections:
                if detection.class_name.lower() == target_object.lower() and detection.class_confidence > 0.3:
                    return True
            return False
            
        except Exception as e:
            rospy.logerr(f'YOLO detection error: {e}')
            return False
    
    def detect_person_with_yolo(self):
        if self.current_image is None or self.detector is None:
            return False
        
        try:
            cv_copy = cv2.resize(self.current_image.copy(), (608, 608))
            detections = self.detector.perform_detect(image_path_or_buf=cv_copy, show_image=False)
            
            for detection in detections:
                if detection.class_name.lower() == 'person' and detection.class_confidence > 0.3:
                    return True
            return False
            
        except Exception as e:
            rospy.logerr(f'YOLO person detection error: {e}')
            return False
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def hotel_request_callback(self, msg):
    if not self.has_new_request:
        # Validate request
        if msg.room not in ROOM_COORDINATES:
            rospy.logerr(f'Invalid room: {msg.room}')
            return
        
        rospy.loginfo(f'Received hotel request: {msg.request} to room {msg.room}')
        self.current_request.request = msg.request
        self.current_request.room = msg.room
        self.has_new_request = True
    else:
        rospy.loginfo('Request ignored - already processing a request')
    
    def run_hotel_service(self):
        sm = self.create_hotel_service_state_machine()
        outcome = sm.execute()
        rospy.loginfo(f'Hotel service ended with outcome: {outcome}')
    
    def create_hotel_service_state_machine(self):
        sm = smach.StateMachine(outcomes=['service_completed', 'service_failed'])
        sm.userdata.object_requested = ''
        sm.userdata.delivery_room = ''
        
        with sm:
            smach.StateMachine.add('WAIT_IN_LOBBY', WaitInLobby(self),
                                   transitions={'request_received': 'GO_TO_PANTRY'})
            
            smach.StateMachine.add('GO_TO_PANTRY', NavigateToRoom('A'),
                                   transitions={'arrived': 'SEARCH_IN_PANTRY', 'failed': 'service_failed'})
            
            smach.StateMachine.add('SEARCH_IN_PANTRY', SearchInPantry(self),
                                   transitions={'object_found': 'GO_TO_DELIVERY_ROOM', 'object_not_found': 'WAIT_IN_LOBBY'})
            
            smach.StateMachine.add('GO_TO_DELIVERY_ROOM', NavigateToRoom(),
                                   transitions={'arrived': 'CHECK_FOR_PERSON', 'failed': 'service_failed'})
            
            smach.StateMachine.add('CHECK_FOR_PERSON', CheckForPerson(self),
                                   transitions={'person_found': 'DELIVER_TO_GUEST', 'person_not_found': 'GO_TO_FRONT_DESK'})
            
            smach.StateMachine.add('DELIVER_TO_GUEST', DeliverToGuest(self),
                                   transitions={'delivered': 'RETURN_TO_LOBBY'})
            
            smach.StateMachine.add('GO_TO_FRONT_DESK', NavigateToRoom('D'),
                                   transitions={'arrived': 'REPORT_TO_FRONT_DESK', 'failed': 'service_failed'})
            
            smach.StateMachine.add('REPORT_TO_FRONT_DESK', ReportToFrontDesk(self),
                                   transitions={'reported': 'RETURN_TO_LOBBY'})
            
            smach.StateMachine.add('RETURN_TO_LOBBY', NavigateToRoom('E'),
                                   transitions={'arrived': 'WAIT_IN_LOBBY', 'failed': 'service_failed'})
        
        return sm

if __name__ == '__main__':
    try:
        node = HotelMainNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass