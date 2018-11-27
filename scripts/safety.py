#!/usr/bin/python

import yaml
import numpy as np
import rospy
import os
import rospkg

from std_msgs.msg import (
    Empty
)

from baxter_core_msgs.msg import (
    EndpointState
)

from sensor_msgs.msg import (
    JointState
)



_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2',
           'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
_coordinates = ['x', 'y', 'z']
_orientations = ['x', 'y', 'z', 'w']


class SafetyNode(object):
    # monitor the robot and make sure it follows certain constraints,
    # if these constratins are violated, the robot shuts down

    def __init__(self, param_file):
        self._estop_pub = rospy.Publisher(
            '/robot/set_super_stop', Empty, queue_size=2)
        self._spin_rate = 10
        self._spin_rate_control = rospy.Rate(self._spin_rate)
        self._kill_flag = False

        self._left_endpoint_sub = rospy.Subscriber(
            '/robot/limb/left/endpoint_state', EndpointState, self._left_endpoint_cb, queue_size=1)
        self._right_endpoint_sub = rospy.Subscriber(
            '/robot/limb/right/endpoint_state', EndpointState, self._right_endpoint_cb, queue_size=1)
        self._jointstate_sub = rospy.Subscriber(
            '/robot/joint_states', JointState, self._jointstate_cb, queue_size=1)

        self._last_left_endpoint = None
        self._last_right_endpoint = None
        self._last_jointstate = None

        self._params = param_file
        #TODO maybe in the future have some way to check if the file is valid



    def _left_endpoint_cb(self, msg):
        self._last_left_endpoint = msg

    def _right_endpoint_cb(self, msg):
        self._last_right_endpoint = msg

    def _jointstate_cb(self, msg):
        self._last_jointstate = msg

    def _orientation_constraints(self):

        if self._kill_flag or 'endpoint_orientation' not in self._params:
            return

        # check if left endpoint oreintation is valid
        if self._last_left_endpoint != None:
            left_orientation = self._last_left_endpoint.pose.orientation
            for i in _orientations:
                # skip empty yaml file entries
                if self._params["endpoint_orientation"]["left"][i]["max"] == None:
                    continue
                # compare values to max parameters
                if getattr(left_orientation, i) > self._params["endpoint_orientation"]["left"][i]["max"]:
                    rospy.logerr("Left endpoint, coord: %s  orient: %s  max: %s", i, getattr(
                        left_orientation, i), self._params["endpoint_orientation"]["left"][i]["max"])
                    self._kill_flag = True
            for j in _orientations:
                # skip empty yaml file entries
                if self._params["endpoint_orientation"]["left"][j]["min"] == None:
                    continue
                # compare values to min parameters
                if getattr(left_orientation, j) < self._params["endpoint_orientation"]["left"][j]["min"]:
                    rospy.logerr("Left endpoint, coord: %s  orient: %s  min: %s", j, getattr(
                        left_orientation, j), self._params["endpoint_orientation"]["left"][j]["min"])
                    self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING LEFT ENDPOINT ORIENTATION')

        # check if right endpoint orientation is valid
        if self._last_right_endpoint != None:
            right_orientation = self._last_right_endpoint.pose.orientation
            for i in _orientations:
                if self._params["endpoint_orientation"]["right"][i]["max"] == None:
                    continue
                if getattr(right_orientation, i) > self._params["endpoint_orientation"]["right"][i]["max"]:
                    rospy.logerr("Right endpoint, coord: %s  orient: %s  max: %s", i, getattr(
                        right_orientation, i), self._params["endpoint_orientation"]["right"][i]["max"])
                    self._kill_flag = True
            for j in _orientations:
                if self._params["endpoint_orientation"]["right"][j]["min"] == None:
                    continue
                if getattr(right_orientation, j) < self._params["endpoint_orientation"]["right"][j]["min"]:
                    rospy.logerr("Right endpoint, coord: %s  orient: %s  min: %s", j, getattr(
                        right_orientation, j), self._params["endpoint_orientation"]["right"][j]["min"])
                    self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING RIGHT ENDPOINT ORIENTATION')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! ORIENTATION VIOLATED')

    def _position_constraints(self):

        if self._kill_flag or 'endpoint_position' not in self._params:
            return

        # check if left endpoint position is valid
        if self._last_left_endpoint != None:
            jointstate = self._last_jointstate
            left_position = self._last_left_endpoint.pose.position
            for i in _coordinates:
                # skip empty yaml file entries
                if self._params["endpoint_position"]["left"][i]["max"] == None:
                    continue
                # compare values to max parameters
                if getattr(left_position, i) > self._params["endpoint_position"]["left"][i]["max"]:
                    rospy.logerr("Left endpoint, coord: %s  pos: %s  max: %s", i, getattr(
                        left_position, i), self._params["endpoint_position"]["left"][i]["max"])
                    self._kill_flag = True
            for j in _coordinates:
                # skip empty yaml file entries
                if self._params["endpoint_position"]["left"][j]["min"] == None:
                    continue
                # compare values to min parameters
                if getattr(left_position, j) < self._params["endpoint_position"]["left"][j]["min"]:
                    rospy.logerr("Left endpoint, coord: %s  pos: %s  min: %s", j, getattr(
                        left_position, j), self._params["endpoint_position"]["left"][j]["min"])
                    self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING LEFT ENDPOINT POSITION')

        # check if right endpoint position is valid
        if self._last_right_endpoint != None:
            jointstate = self._last_jointstate
            right_position = self._last_right_endpoint.pose.position
            for i in _coordinates:
                if self._params["endpoint_position"]["right"][i]["max"] == None:
                    continue
                if getattr(right_position, i) > self._params["endpoint_position"]["right"][i]["max"]:
                    rospy.logerr("Right endpoint, coord: %s  pos: %s  max: %s", i, getattr(
                        right_position, i), self._params["endpoint_position"]["right"][i]["max"])
                    self._kill_flag = True
            for j in _coordinates:
                if self._params["endpoint_position"]["right"][j]["min"] == None:
                    continue
                if getattr(right_position, j) < self._params["endpoint_position"]["right"][j]["min"]:
                    rospy.logerr("Right endpoint, coord: %s  pos %s  min: %s", j, getattr(
                        right_position, j), self._params["endpoint_position"]["right"][j]["min"])
                    self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING RIGHT ENDPOINT POSITION')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! POSITION VIOLATED')


    def _joint_constraints(self):

        if self._kill_flag or 'joint_position' not in self._params:
            return

        # check if joint positions are valid
        if self._last_jointstate != None:
            jointstate = self._last_jointstate
            for i, pos in enumerate(jointstate.position):
                if jointstate.name[i] in _joints:
                    if self._params["joint_position"][jointstate.name[i]]["max"] == None:
                        continue
                    if pos > self._params["joint_position"][jointstate.name[i]]["max"]:
                        rospy.logerr("Joint, name: %s  pos: %s  max: %s",
                                     jointstate.name[i], pos, self._params["joint_position"][jointstate.name[i]]["max"])
                        self._kill_flag = True
            for j, pos in enumerate(jointstate.position):
                if jointstate.name[j] in _joints:
                    if self._params["joint_position"][jointstate.name[j]]["min"] == None:
                        continue
                    if pos < self._params["joint_position"][jointstate.name[j]]["min"]:
                        rospy.logerr("Joint, name: %s  pos: %s  min: %s",
                                     jointstate.name[j], pos, self._params["joint_position"][jointstate.name[j]]["min"])
                        self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING JOINT POSITIONS')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! JOINT POSITION VIOLATED')

    def _velocity_constraints(self):

        if self._kill_flag or 'endpoint_velocity' not in self._params:
            return

        # check if left endpoint velocity is valid
        if self._last_left_endpoint != None:
            left_angular = self._last_left_endpoint.twist.angular
            for i in _coordinates:
                # skip empty yaml file entries
                if self._params["endpoint_velocity"]["left"]["angular"][i] == None:
                    continue
                # compare values to max angular parameters
                if np.abs(getattr(left_angular, i)) > self._params["endpoint_velocity"]["left"]["angular"][i]:
                    rospy.logerr("Left angular, coord: %s  vel: %s  max: %s", i, np.abs(
                        getattr(left_angular, i)), self._params["endpoint_velocity"]["left"]["angular"][i])
                    self._kill_flag = True
            left_linear = self._last_left_endpoint.twist.linear
            for j in _coordinates:
                # skip empty yaml file entries
                if self._params["endpoint_velocity"]["left"]["linear"][j] == None:
                    continue
                # compare values to max linear parameters
                if np.abs(getattr(left_linear, j)) > self._params["endpoint_velocity"]["left"]["linear"][j]:
                    rospy.logerr("Left linear, coord: %s  vel: %s  max: %s", j, np.abs(
                        getattr(left_linear, j)), self._params["endpoint_velocity"]["left"]["linear"][j])
                    self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING LEFT ENDPOINT VELOCITY')

        # check if right endpoint velocity is valid
        if self._last_right_endpoint != None:
            right_angular = self._last_right_endpoint.twist.angular
            for i in _coordinates:
                if self._params["endpoint_velocity"]["right"]["angular"][i] == None:
                    continue
                if np.abs(getattr(right_angular, i)) > self._params["endpoint_velocity"]["right"]["angular"][i]:
                    rospy.logerr("Right angular, coord: %s  vel: %s  max: %s", i, np.abs(
                        getattr(right_angular, i)), self._params["endpoint_velocity"]["right"]["angular"][i])
                    self._kill_flag = True
            right_linear = self._last_right_endpoint.twist.linear
            for j in _coordinates:
                if self._params["endpoint_velocity"]["right"]["linear"][j] == None:
                    continue
                if np.abs(getattr(right_linear, j)) > self._params["endpoint_velocity"]["right"]["linear"][j]:
                    rospy.logerr("Right linear, coord: %s  vel: %s  max: %s", j, np.abs(
                        getattr(right_linear, j)), self._params["endpoint_velocity"]["right"]["linear"][j])
                    self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING RIGHT ENDPOINT VELOCITY')

        # check if joint velocities are valid
        if self._last_jointstate != None:
            jointstate = self._last_jointstate
            for i, vel in enumerate(jointstate.velocity):
                if jointstate.name[i] in _joints:
                    if self._params["joint_velocity"][jointstate.name[i]] == None:
                        continue
                    if np.abs(vel) > self._params["joint_velocity"][jointstate.name[i]]:
                        rospy.logerr("Joint, name: %s  vel: %s  max: %s", jointstate.name[i], np.abs(
                            vel), self._params["joint_velocity"][jointstate.name[i]])
                        self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING JOINT VELOCITIES')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! VELOCITY VIOLATED')

    def _check_constraints(self):
        self._orientation_constraints()
        self._position_constraints()
        self._velocity_constraints()
        self._joint_constraints()

    def spin(self):
        # our spin loop
        while not rospy.is_shutdown():
            # Step 1: check constraints
            self._check_constraints()
            # Step 2: check if robot should kill
            if self._kill_flag:
                self.kill()
                # return
            self._spin_rate_control.sleep()

    def kill(self):
        # send the kill commands
        self._estop_pub.publish()


if __name__ == '__main__':
    rospy.init_node('safety_node')

    #try to find where the yaml file that define baxter's property is
    rospack = rospkg.RosPack()
    rospy.loginfo('{}'.format(rospy.get_namespace()))
    yaml_file_path = rospy.get_param('safety_param_file_path', os.path.join(rospack.get_path("success_baxter_safety"), "default_params.yaml"))
    rospy.loginfo('safety parameter file path:{}'.format(yaml_file_path))

    #try loading the file
    with open(yaml_file_path, 'r') as stream:
        try:
            yaml_file = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    _estop_pub = rospy.Publisher('/robot/set_super_stop', Empty, queue_size=2)
    try:
        sn = SafetyNode(yaml_file)
        rospy.loginfo("Safety node start running")
        sn.spin()
    except Exception as ex:
        # seriously, this shouldn't be happening. estop just in case!!!
        r = rospy.Rate(10)
        rospy.logerr("UNKNOWN ERROR!!!! SAFETY MODULE HAVING EXCEPTIONS!!!!")
        rospy.logerr("{}".format(ex))
        # while True:
        #     _estop_pub.publish()
        #     r.sleep()
