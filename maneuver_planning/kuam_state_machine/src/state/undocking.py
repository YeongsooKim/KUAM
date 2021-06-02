import rospy
import smach

# define state UNDOCKING
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['wait'])
def UndockingCB(user_data):
    rospy.loginfo("Undocking callback")

    return 'wait'
