import rospy
import smach

# define state TRANSITION
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['flight'])
def TransitionCB(user_data):
    rospy.loginfo("Transition callback")

    return 'flight'
