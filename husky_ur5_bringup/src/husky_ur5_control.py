#!/usr/bin/env python3

import rospy
import random
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

class husky_ur5:
    def __init__(self):

        self.set_init_pose_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Service("husky_ur5/random", Trigger, self.setting)
        
    def setting(self, req):

        res = TriggerResponse()

        req = ModelState()
        req.model_name = 'robot'
        req.pose.position.x = 9.0 + random.uniform(-0.1, 0.1)
        req.pose.position.y = 13.3 + random.uniform(-0.1, 0.1)
        req.pose.position.z = 0.1323

        angle = random.uniform(-95, -85)
        r = R.from_euler("z", angle, degrees=True)
        quat = r.as_quat()
        req.pose.orientation.x = quat[0]
        req.pose.orientation.y = quat[1]
        req.pose.orientation.z = quat[2]
        req.pose.orientation.w = quat[3]
        self.set_init_pose_srv(req)

        res.success = True

        return res

if __name__ == '__main__':
    rospy.init_node("husky_ur5_control_node", anonymous=False)
    Husky_UR5 = husky_ur5()
    rospy.spin()