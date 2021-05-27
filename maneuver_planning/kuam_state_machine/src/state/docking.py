import rospy
import smach

# define state DOCKING
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['wait'])
def DockingCB(user_data):
    rospy.loginfo("Docking callback")

    return 'wait'
