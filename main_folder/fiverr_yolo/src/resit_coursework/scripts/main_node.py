#!/usr/bin/env python3

import rospy
import actionlib
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import time
import os

# Import YOLO messages
from darknet_ros_msgs.msg import BoundingBoxes

# Import navigation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# Import the custom message
from resit_coursework.msg import HotelRequest

# Define room coordinates (based on actual RViz map coordinates)
ROOM_COORDINATES = {
    'A': {'x': 2.0, 'y': 8.5, 'theta': 0.0},   # Pantry
    'B': {'x': 6.0, 'y': 8.5, 'theta': 0.0},   # Guest room
    'C': {'x': 11.0, 'y': 8.5, 'theta': 0.0},  # Guest room
    'D': {'x': 2.0, 'y': 3.0, 'theta': 0.0},   # Front Desk
    'E': {'x': 6.0, 'y': 3.0, 'theta': 0.0},   # Lobby (start here)
    'F': {'x': 11.0, 'y': 3.0, 'theta': 0.0},  # Guest room
}

# Hotel objects that can be retrieved
HOTEL_OBJECTS = ['toothbrush', 'banana', 'sandwich', 'pizza', 'broccoli']

# SMACH States for the hotel robot behavior
class WaitInLobby(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['request_received'],
                             output_keys=['object_requested', 'delivery_room'])
        self.main_node = main_node
        
    def execute(self, userdata):
        rospy.loginfo('Waiting in lobby for hotel request...')
        
        # Wait for hotel request
        while not rospy.is_shutdown() and not self.main_node.has_new_request:
            rospy.sleep(0.1)
        
        if self.main_node.has_new_request:
            # Set userdata first
            userdata.object_requested = self.main_node.current_request.request
            userdata.delivery_room = self.main_node.current_request.room
            self.main_node.has_new_request = False
            
            # Now we can safely log the values
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
        # Use dynamic room if target_room is None
        room = self.target_room if self.target_room else userdata.delivery_room
        rospy.loginfo(f'Navigating to room {room}')
        
        # Create move_base action client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        if not client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn('move_base server not available, simulating navigation')
            time.sleep(2)  # Simulate travel time
            return 'arrived'
        
        # Create navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set target coordinates
        coords = ROOM_COORDINATES[room]
        goal.target_pose.pose.position.x = coords['x']
        goal.target_pose.pose.position.y = coords['y']
        goal.target_pose.pose.position.z = 0.0
        
        # Set orientation
        q = quaternion_from_euler(0, 0, coords['theta'])
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        rospy.loginfo(f'Sending goal: x={coords["x"]}, y={coords["y"]}')
        
        # Send goal and wait for result
        client.send_goal(goal)
        result = client.wait_for_result(timeout=rospy.Duration(60.0))
        
        if result:
            rospy.loginfo(f'Successfully reached room {room}')
            time.sleep(1)  # Brief pause to ensure robot is settled
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
        self.search_time = 0
        self.max_search_time = 60  # 60 seconds max search
        
    def execute(self, userdata):
        object_name = userdata.object_requested
        rospy.loginfo(f'Searching for {object_name} in pantry')
        
        # Move around in pantry while searching
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        twist = Twist()
        
        self.search_time = 0
        rate = rospy.Rate(10)  # 10 Hz
        
        while self.search_time < self.max_search_time and not rospy.is_shutdown():
            # Move around (rotate slowly)
            twist.angular.z = 0.2  # Rotate slowly
            vel_pub.publish(twist)
            
            # Check YOLO detections
            detected_objects = self.main_node.detected_objects
            
            if object_name.lower() in [obj.lower() for obj in detected_objects]:
                # Stop moving
                twist.angular.z = 0.0
                vel_pub.publish(twist)
                
                rospy.loginfo(f'Found {object_name} in pantry!')
                self.announce_object_found(object_name)
                return 'object_found'
            
            self.search_time += 0.1
            rate.sleep()
        
        # Stop moving
        twist.angular.z = 0.0
        vel_pub.publish(twist)
        
        rospy.logwarn(f'Could not find {object_name} in pantry')
        return 'object_not_found'
    
    def announce_object_found(self, object_name):
        announcement = f"I found the {object_name}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        os.system(f"espeak '{announcement}'")

class CheckForPerson(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['person_found', 'person_not_found'])
        self.main_node = main_node
        
    def execute(self, userdata):
        rospy.loginfo('Checking for person in room...')
        
        # Wait a moment for YOLO to detect
        time.sleep(2)
        
        # Check YOLO detections for person
        detected_objects = self.main_node.detected_objects
        
        if 'person' in [obj.lower() for obj in detected_objects]:
            rospy.loginfo('Person detected in room')
            return 'person_found'
        else:
            rospy.loginfo('No person detected in room')
            return 'person_not_found'

class DeliverToGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['delivered'],
                             input_keys=['object_requested'])
        
    def execute(self, userdata):
        object_name = userdata.object_requested
        
        announcement = f"I am delivering your {object_name}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        os.system(f"espeak '{announcement}'")
        
        time.sleep(2)  # Wait for delivery
        return 'delivered'

class ReportToFrontDesk(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reported'],
                             input_keys=['object_requested', 'delivery_room'])
        
    def execute(self, userdata):
        object_name = userdata.object_requested
        room = userdata.delivery_room
        
        announcement = f"The person was not available to deliver the {object_name} in room {room}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        os.system(f"espeak '{announcement}'")
        
        time.sleep(2)  # Wait after report
        return 'reported'

# Main Hotel Node
class HotelMainNode:
    def __init__(self):
        rospy.init_node('main_node')
        
        # Hotel request handling
        self.current_request = HotelRequest()
        self.has_new_request = False
        
        # Initialize detected objects list
        self.detected_objects = []
        
        # Subscribe to hotel requests - FIXED: Added message type
        rospy.Subscriber("/hotel_request", HotelRequest, self.hotel_request_callback)
        
        # Subscribe to YOLO bounding boxes
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_callback)
        
        rospy.loginfo('Hotel service robot initialized and ready in lobby')
        
        # Start the main service loop
        self.run_hotel_service()
    
    def hotel_request_callback(self, msg):
        # Handle the HotelRequest message
        rospy.loginfo(f'Received hotel request: {msg.request} to room {msg.room}')
        self.current_request.request = msg.request
        self.current_request.room = msg.room
        self.has_new_request = True
    
    def yolo_callback(self, msg):
        # Process YOLO detections
        self.detected_objects = []
        for box in msg.bounding_boxes:
            self.detected_objects.append(box.Class.lower())
        
        if self.detected_objects:
            rospy.loginfo(f'YOLO detected objects: {self.detected_objects}')
    
    def run_hotel_service(self):
        # Main service loop
        while not rospy.is_shutdown():
            # Create and run the state machine for hotel service
            sm = self.create_hotel_service_state_machine()
            outcome = sm.execute()
            
            rospy.loginfo(f'Hotel service cycle completed with outcome: {outcome}')
            
            # Brief pause before next cycle
            rospy.sleep(1.0)
    
    def create_hotel_service_state_machine(self):
        # Create a SMACH state machine for hotel service
        sm = smach.StateMachine(outcomes=['service_completed', 'service_failed'])
        
        # Set up userdata keys for the state machine
        sm.userdata.object_requested = ''
        sm.userdata.delivery_room = ''
        
        with sm:
            # Wait for request in lobby
            smach.StateMachine.add(
                'WAIT_IN_LOBBY',
                WaitInLobby(self),
                transitions={'request_received': 'GO_TO_PANTRY'}
            )
            
            # Navigate to pantry
            smach.StateMachine.add(
                'GO_TO_PANTRY',
                NavigateToRoom('A'),
                transitions={
                    'arrived': 'SEARCH_IN_PANTRY',
                    'failed': 'service_failed'
                }
            )
            
            # Search for object in pantry
            smach.StateMachine.add(
                'SEARCH_IN_PANTRY',
                SearchInPantry(self),
                transitions={
                    'object_found': 'GO_TO_DELIVERY_ROOM',
                    'object_not_found': 'RETURN_TO_LOBBY'
                }
            )
            
            # Navigate to delivery room (dynamic based on request)
            smach.StateMachine.add(
                'GO_TO_DELIVERY_ROOM',
                NavigateToRoom(),  # No target_room = uses userdata
                transitions={
                    'arrived': 'CHECK_FOR_PERSON',
                    'failed': 'service_failed'
                }
            )
            
            # Check if person is in room
            smach.StateMachine.add(
                'CHECK_FOR_PERSON',
                CheckForPerson(self),
                transitions={
                    'person_found': 'DELIVER_TO_GUEST',
                    'person_not_found': 'GO_TO_FRONT_DESK'
                }
            )
            
            # Deliver to guest
            smach.StateMachine.add(
                'DELIVER_TO_GUEST',
                DeliverToGuest(),
                transitions={'delivered': 'RETURN_TO_LOBBY'}
            )
            
            # Go to front desk if person not found
            smach.StateMachine.add(
                'GO_TO_FRONT_DESK',
                NavigateToRoom('D'),
                transitions={
                    'arrived': 'REPORT_TO_FRONT_DESK',
                    'failed': 'service_failed'
                }
            )
            
            # Report to front desk
            smach.StateMachine.add(
                'REPORT_TO_FRONT_DESK',
                ReportToFrontDesk(),
                transitions={'reported': 'RETURN_TO_LOBBY'}
            )
            
            # Return to lobby
            smach.StateMachine.add(
                'RETURN_TO_LOBBY',
                NavigateToRoom('E'),
                transitions={
                    'arrived': 'service_completed',
                    'failed': 'service_failed'
                }
            )
        
        return sm

if __name__ == '__main__':
    try:
        node = HotelMainNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass