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
        self.arm_home_srv = rospy.ServiceProxy("/robot/ur5/go_home", Trigger)

        # For husky ur5 ex2
        rospy.Service("/husky_ur5/random", Trigger, self.go_husky_ur5_init)

        # For ex3 box
        rospy.Service("/husky_ur5/pull_box", Trigger, self.pull_box_init)

        # For ex3 cardboard
        rospy.Service("/husky_ur5/pull_cardboard", Trigger, self.pull_cardboard_init)

        # For husky vx300s ex2
        rospy.Service("/husky_vx300s/init", Trigger, self.go_husky_vx300s_init)

        # For husky ur5 ex1 
        rospy.Service("/husky_ur5/init", Trigger, self.go_ex1_init)

        # For pokingbot ex1
        rospy.Service("/husky/init", Trigger, self.go_pokinbbot_init)
        self.set_door_srv = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

    def go_pokinbbot_init(self, req):

        res = TriggerResponse()

        req = ModelState()
        req.model_name = 'robot'
        req.pose.position.x = random.uniform(7.0, 11.0)
        req.pose.position.y = 17.0
        req.pose.position.z = 0.1323
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = -0.707
        req.pose.orientation.w = 0.707

        # set robot
        self.set_init_pose_srv(req)

        # set door 
        self.set_door_srv("hinge_door_0", "", ["hinge"], [0])

        res.success = True

        return res

    def go_husky_vx300s_init(self, req):

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

        # set robot
        self.set_init_pose_srv(req)

        # set door 
        self.set_door_srv("hinge_door_0", "", ["hinge"], [0])

        res.success = True

        return res

    def go_ex1_init(self, req):

        res = TriggerResponse()

        req = ModelState()
        req.model_name = 'robot'
        req.pose.position.x = random.uniform(7.0, 11.0)
        req.pose.position.y = 17.0
        req.pose.position.z = 0.1323
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = -0.707
        req.pose.orientation.w = 0.707

        # set robot
        self.set_init_pose_srv(req)

        # set door 
        self.set_door_srv("hinge_door_0", "", ["hinge"], [0])

        # arm go home
        self.arm_home_srv()

        res.success = True

        return res
        
    def go_husky_ur5_init(self, req):

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
        
        # set robot
        self.set_init_pose_srv(req)

        # set door 
        self.set_door_srv("hinge_door_0", "", ["hinge"], [0])
        
        # arm go home
        self.arm_home_srv()

        res.success = True

        return res

    def pull_box_init(self, req):

        res = TriggerResponse()

        req = ModelState()
        req.model_name = 'robot'
        req.pose.position.x = 9.0 + random.uniform(-0.1, 0.1)
        req.pose.position.y = 14.2 + random.uniform(-0.1, 0.1)
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

    def pull_cardboard_init(self, req):

        res = TriggerResponse()

        req = ModelState()
        req.model_name = 'robot'
        req.pose.position.x = 9.0 + random.uniform(-0.1, 0.1)
        req.pose.position.y = 13.5 + random.uniform(-0.1, 0.1)
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