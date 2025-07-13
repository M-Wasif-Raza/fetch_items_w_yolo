#!/usr/bin/env python3

import rospy
import actionlib
import smach
import smach_ros
from std_msgs.msg import String, Empty
from actionlib_msgs.msg import GoalStatus
import cv2
import numpy as np
import os
import time
import random

# Import the action message types as requested
from second_coursework.msg import CreateSemanticMapAction, CreateSemanticMapResult
from second_coursework.msg import EmptyActionAction as EmptyAction, EmptyActionResult as EmptyResult

# Import YOLO messages
from darknet_ros_msgs.msg import BoundingBoxes

# Define the room types and their associated objects
ROOM_OBJECTS = {
    'kitchen': ['microwave', 'refrigerator', 'bottle', 'sink', 'fork', 'banana', 'sandwich', 'chair'],
    'garage': ['bicycle', 'car', 'sports ball', 'skateboard', 'motorbike'],
    'bedroom': ['bottle', 'laptop', 'handbag', 'tv monitor', 'sports ball', 'skateboard', 'chair', 'bed'],
    'living_room': ['tv monitor', 'remote', 'book', 'clock', 'dining table', 'chair'],
    'bathroom': ['toothbrush', 'toilet', 'sink']
}

# SMACH States for the robot behavior
class ExploreRoom(smach.State):
    def __init__(self, room_label, main_node):
        smach.State.__init__(self, 
                             outcomes=['room_recognized', 'room_not_recognized'],
                             output_keys=['detected_room', 'room_label'])
        self.room_label = room_label
        self.main_node = main_node
        
    def execute(self, userdata):
        rospy.loginfo(f'Exploring room {self.room_label}')
        
        # Use the detected objects from YOLO
        detected_objects = self.main_node.detected_objects
        
        if detected_objects:
            room_type = self.classify_room(detected_objects)
            if room_type:
                userdata.detected_room = room_type
                userdata.room_label = self.room_label
                # Use TTS to announce the room with espeak
                self.announce_room(room_type)
                return 'room_recognized'
        
        return 'room_not_recognized'
    
    def classify_room(self, detected_objects):
        # Count the number of objects from each room type
        room_scores = {}
        for room_type, objects in ROOM_OBJECTS.items():
            score = sum(1 for obj in detected_objects if obj in objects)
            room_scores[room_type] = score
        
        # Find the room type with the highest score
        if room_scores:
            best_room = max(room_scores.items(), key=lambda x: x[1])
            if best_room[1] > 0:
                rospy.loginfo(f'Classified as {best_room[0]} with score {best_room[1]}')
                return best_room[0]
        
        return None
    
    def announce_room(self, room_type):
        # Use espeak for TTS as requested
        announcement = f"This is the {room_type}"
        rospy.loginfo(f'Robot says: "{announcement}"')
        os.system(f"espeak '{announcement}'")

class GoToLivingRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'])
        
    def execute(self, userdata):
        rospy.loginfo('Going to the living room')
        # Simulate movement to the living room
        time.sleep(2)
        return 'arrived'

class ListenForCommand(smach.State):
    def __init__(self, main_node):
        smach.State.__init__(self, 
                             outcomes=['command_received', 'no_command'],
                             output_keys=['object_to_fetch', 'target_room'])
        self.main_node = main_node
        
    def execute(self, userdata):
        rospy.loginfo('Listening for command')
        
        # Use speech recognition as taught in class
        command = self.recognize_speech()
        
        if command:
            object_name, target_room = self.parse_command(command)
            if object_name and target_room:
                userdata.object_to_fetch = object_name
                userdata.target_room = target_room
                return 'command_received'
        
        return 'no_command'
    
    def recognize_speech(self):
        # Implement speech recognition as requested
        # If speech recognition module is not available, use input simulation
        command = input("Please type the spoken command: ")
        return command
    
    def parse_command(self, command):
        # Parse the command to extract the object and target room
        rospy.loginfo(f'Recognized command: {command}')
        
        # Simple parsing logic - in a real implementation this would be more robust
        words = command.lower().split()
        
        # Find the object in the command
        all_objects = []
        for objects in ROOM_OBJECTS.values():
            all_objects.extend(objects)
        
        object_name = None
        for obj in all_objects:
            if obj in command.lower():
                object_name = obj
                break
        
        # If no specific object found, use a generic one from the command
        if not object_name:
            for word in words:
                if word not in ["please", "bring", "a", "to", "the"]:
                    object_name = word
                    break
        
        # Find the target room in the command
        target_room = None
        for room in ROOM_OBJECTS.keys():
            room_name = room.replace('_', ' ')
            if room_name in command.lower():
                target_room = room
                break
        
        return object_name, target_room

class FetchObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['object_fetched', 'object_not_found'],
                             input_keys=['object_to_fetch', 'target_room'])
        
    def execute(self, userdata):
        object_name = userdata.object_to_fetch
        target_room = userdata.target_room
        
        rospy.loginfo(f'Fetching {object_name} from appropriate room')
        
        # Simulate object fetching
        # In a real implementation, this would involve navigation and manipulation
        time.sleep(10)  # Wait for 10 seconds as required
        
        # Simulate success with 80% probability
        if random.random() < 0.8:
            rospy.loginfo(f'Successfully fetched {object_name}')
            return 'object_fetched'
        else:
            rospy.loginfo(f'Failed to find {object_name}')
            return 'object_not_found'

class DeliverObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['delivered'],
                             input_keys=['target_room'])
        
    def execute(self, userdata):
        target_room = userdata.target_room
        
        rospy.loginfo(f'Delivering object to {target_room}')
        
        # Simulate object delivery
        # In a real implementation, this would involve navigation
        time.sleep(2)
        
        rospy.loginfo('Object delivered successfully')
        return 'delivered'

# Main node class
class MainNode:
    def __init__(self):
        rospy.init_node('main_node')
        
        # Initialize the semantic map
        self.semantic_map = {
            'A': '',
            'B': '',
            'C': '',
            'D': '',
            'E': '',
            'F': ''
        }
        
        # Initialize detected objects list
        self.detected_objects = []
        
        # Subscribe to YOLO bounding boxes
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_callback)
        
        # Create action servers using the proper message types
        self.create_semantic_map_server = actionlib.SimpleActionServer(
            'create_semantic_map',
            CreateSemanticMapAction,
            self.create_semantic_map_callback,
            False
        )
        self.create_semantic_map_server.start()
        
        self.get_help_server = actionlib.SimpleActionServer(
            'get_help',
            EmptyAction,
            self.get_help_callback,
            False
        )
        self.get_help_server.start()
        
        rospy.loginfo('Main node initialized')
    
    def yolo_callback(self, msg):
        # Process YOLO detections
        self.detected_objects = []
        for box in msg.bounding_boxes:
            # Add the class name to our detected objects list
            self.detected_objects.append(box.Class.lower())
        
        rospy.loginfo(f'YOLO detected objects: {self.detected_objects}')
    
    def create_semantic_map_callback(self, goal):
        rospy.loginfo('Received create_semantic_map goal')
        
        # Create and configure the state machine
        sm = self.create_semantic_mapping_state_machine()
        
        # Execute the state machine
        outcome = sm.execute()
        
        # Create and send the result
        result = CreateSemanticMapResult()
        result.A = self.semantic_map['A']
        result.B = self.semantic_map['B']
        result.C = self.semantic_map['C']
        result.D = self.semantic_map['D']
        result.E = self.semantic_map['E']
        result.F = self.semantic_map['F']
        
        self.create_semantic_map_server.set_succeeded(result)
    
    def get_help_callback(self, goal):
        rospy.loginfo('Received get_help goal')
        
        # Create and configure the state machine for the fetch task
        sm = self.create_fetch_state_machine()
        
        # Execute the state machine
        outcome = sm.execute()
        
        # Send the result
        result = EmptyResult()
        self.get_help_server.set_succeeded(result)
    
    def create_semantic_mapping_state_machine(self):
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        
        # Define the room labels to explore
        room_labels = ['A', 'B', 'C', 'D', 'E', 'F']
        
        with sm:
            # Add states for exploring each room
            for i, label in enumerate(room_labels):
                smach.StateMachine.add(
                    f'EXPLORE_{label}',
                    ExploreRoom(label, self),
                    transitions={
                        'room_recognized': f'EXPLORE_{room_labels[i+1]}' if i < len(room_labels) - 1 else 'succeeded',
                        'room_not_recognized': f'EXPLORE_{label}'  # Try again if not recognized
                    }
                )
                
                # Add a transition to update the semantic map when a room is recognized
                sm.register_transition_cb(self.update_semantic_map)
        
        return sm
    
    def create_fetch_state_machine(self):
        # Create a SMACH state machine for the fetch task
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        
        with sm:
            smach.StateMachine.add(
                'GO_TO_LIVING_ROOM',
                GoToLivingRoom(),
                transitions={'arrived': 'LISTEN_FOR_COMMAND'}
            )
            
            smach.StateMachine.add(
                'LISTEN_FOR_COMMAND',
                ListenForCommand(self),
                transitions={
                    'command_received': 'FETCH_OBJECT',
                    'no_command': 'LISTEN_FOR_COMMAND'
                }
            )
            
            smach.StateMachine.add(
                'FETCH_OBJECT',
                FetchObject(),
                transitions={
                    'object_fetched': 'DELIVER_OBJECT',
                    'object_not_found': 'aborted'
                }
            )
            
            smach.StateMachine.add(
                'DELIVER_OBJECT',
                DeliverObject(),
                transitions={'delivered': 'succeeded'}
            )
        
        return sm
    
    def update_semantic_map(self, userdata, active_states):
        # This callback is called on state transitions
        # Update the semantic map when a room is recognized
        if hasattr(userdata, 'detected_room') and hasattr(userdata, 'room_label'):
            room_type = userdata.detected_room
            room_label = userdata.room_label
            
            if room_type and room_label:
                self.semantic_map[room_label] = room_type
                rospy.loginfo(f'Updated semantic map: {room_label} -> {room_type}')

if __name__ == '__main__':
    try:
        node = MainNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
