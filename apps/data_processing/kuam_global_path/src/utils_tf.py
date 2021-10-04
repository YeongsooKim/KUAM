from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2
from geometry_msgs.msg import Quaternion

def GetOrientation(src_pos, trg_pos):
    dx = trg_pos.x - src_pos.x;
    dy = trg_pos.y - src_pos.y;
    yaw_rad = atan2(dy, dx);

    q_ = quaternion_from_euler(0.0, 0.0, yaw_rad)

    q = Quaternion()
    q.x = q_[0]
    q.y = q_[1]
    q.z = q_[2]
    q.w = q_[3]

    return q;