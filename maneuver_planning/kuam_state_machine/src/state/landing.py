import rospy
import smach
import state
import copy

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from kuam_msgs.msg import LandingState

class Landing(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints'], 
                                output_keys=['setpoint', 'setpoints'], 
                                outcomes=['emerg', 'manual'])
        state.Base.__init__(self)

        self.transition = 'none'
        
        # Flag
        self.is_detected = False
        self.is_land = False
        
        # Target state
        self.target_id = 0 # Change to param
        self.target_pose = Pose()
        
        # Param
        self.landing_threshold_m = None
        self.landing_standby_alt_m = 5.0

        # tf
        self.tfBuffer = None
        self.listener = None

        '''
        Change target pose and target id to dictionary when the id is more than two
        '''
        
    def execute(self, userdata):
        self.Start(userdata)
        self.Running()
        return self.Terminate(userdata)

    '''
    State functions
    '''
    def Start(self, userdata):
        # Initialize setpoint
        self.setpoints = copy.deepcopy(userdata.setpoints)
        self.setpoint.pose = self.setpoints.poses[-1]
        self.setpoint.pose.position.z = self.landing_standby_alt_m

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'emerg') or (self.transition == 'manual'):
                    break
                else:
                    self.transition = 'none'    
            # Update virtual border

            # Update setpoint
            if self.is_detected == True:
                self.setpoint.pose = self.target_pose

            # Update landing state
            self.UpdateLandingState()

            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition

        self.transition = 'none'
        return trans

    '''
    Util functions
    '''
    def UpdateLandingState(self):
        if self.landing_threshold_m is None:
            pass
        else:
            h = self.ego_pose.position.z

            if h < self.landing_threshold_m:
                self.is_land = True

    def IsValid(self, pose_stamped):
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z

        if (x>1e+100) or (x<-1e+100):
            return False
        if (y>1e+100) or (y<-1e+100):
            return False
        if (z>1e+100) or (z<-1e+100):
            return False
        return True

    '''
    Callback functions
    '''
    def MarkerCB(self, msg):
        self.target_id = msg.id

        if msg.is_detected == True:
            msg_pose_stamped = PoseStamped()
            msg_pose_stamped.header = msg.header
            msg_pose_stamped.pose = msg.pose
            # transform from camera_link to map
            is_transformed = False
            try:
                transform = self.tfBuffer.lookup_transform('map', msg_pose_stamped.header.frame_id, rospy.Time())
                is_transformed = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            if is_transformed == True:
                pose_transformed = tf2_geometry_msgs.do_transform_pose(msg_pose_stamped, transform)
                if self.IsValid(pose_transformed):
                    self.is_detected = True
                    self.target_pose = pose_transformed.pose
                    # target_pose_pub.publish(pose_transformed)
                else:
                    self.is_detected = False

    # # Get virtual border side
    # def GetVBside(self):
    #     '''
    #     ego orientation -> roll, pitch, yaw 
    #     get VBS (virtual border side).
    #     body -> global (trans, yaw)
    #     '''