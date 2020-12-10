#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         11-11-2020
Description:  TBD
"""
import sys
import rospy
import triple_s_util.srv
from triple_s_util.bin_picking.util import rosparamOrDefault
from triple_s_util.bin_picking.planner import Planner
from onrobot_rg2.srv import ControlRG2

class BinPickingSequencer():
    def __init__(self):
        self.planner = Planner()
        object_request_service_name = rosparamOrDefault('~object_request_service', '/object_request')
        self.rg2_gripper_control_val = rospy.ServiceProxy('/control_rg2', ControlRG2)
        rospy.wait_for_service(object_request_service_name)
        self.requestObjectPose = rospy.ServiceProxy(object_request_service_name, triple_s_util.srv.ObjectRequest)
        self.sequence()

    def sequence(self):
        objects_to_request = ['TomatoSauce','gelatin','meat','sugar']

        for object_to_request in objects_to_request:

            # Temporary: Move to start position
            rospy.loginfo('Moving to start position')
            self.planner.planAndExecuteNamedTarget('dope_to_rviz')
	
            #Open the gripper:
            rospy.loginfo('Opening Gripper')
	    _ = self.rg2_gripper_control_val(110)
            rospy.loginfo('Opened Gripper')
            
            # Request a pose of an object through the service
            rospy.loginfo('Requesting object position')
            request = self.requestObjectPose(object_to_request)
            rospy.loginfo('Got object position')

            if request.found_object:
                # Set the reference frame of the pose to the camera
                request.object_pose.header.frame_id = rosparamOrDefault('~camera_link', 'camera_sim_link')

                if not self.planner.planAndExecuteInReferenceFrame(request.object_pose):
                    rospy.logwarn('Couldn\'t move into position to grab the object!')
                else:
                    rospy.loginfo('Moved to object')
                    rospy.sleep(2)
                    _ = self.rg2_gripper_control_val(60)
                    rospy.sleep(2)
                    rospy.loginfo('Going to the bin')
                    self.planner.planAndExecuteNamedTarget('above_bin')
                    rospy.loginfo('Drop the object')
                    _ = self.rg2_gripper_control_val(100)
                    rospy.sleep(2)

            else:
                 rospy.logwarn('Couldn\'t find any objects of type \"%s\"' % object_to_request)
        rospy.sleep(2)


        # Sleep for a bit
        #rospy.sleep(10)

        # Repeat
        self.sequence()

    
if __name__ == '__main__':
    rospy.init_node('bin_picking_sequencer', anonymous=True)
    
    rospy.loginfo('Initializing bin_picking_sequencer.py')

    BinPickingSequencer()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')
