#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
import robotpy_apriltag
import cv2
from xarm import version
from xarm.wrapper import XArmAPI


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._ignore_exit_state = False
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    # Robot Main Run
    def run(self,coors,detector,cap):
        try:
            # Install xArm Gripper
            code = self._arm.set_counter_reset()
            if not self._check_code(code, 'set_counter_reset'):
                return
            self._tcp_speed = 200
            self._tcp_acc = 5000
            # Initial position
            code = self._arm.set_servo_angle(angle=[101.0, -11.1, -56.4, 0.0, 67.5, 24.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            code = self._arm.set_gripper_position(800, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            
            # See if tag remaining
            while len(coors)!=0:
                if not self.is_alive:
                    break
                t1 = time.monotonic()

                code = self._arm.set_counter_increase()
                if not self._check_code(code, 'set_counter_increase'):
                    return
                
                # Set TCP payload as the xArm Gripper
                code = self._arm.set_tcp_load(0.82, [0, 0, 48])
                if not self._check_code(code, 'set_tcp_load'):
                    return
                
                # Detect goal Tag
                tag_name=(coors.keys())[0]
                currently_at=get_pose(tag_name,detector,cap)
                final_location=coors[tag_name][0]

                # Remove visited location
                if len(coors[tag_name])==1:
                    del coors[tag_name]
                else:
                    coors[tag_name]=coors[tag_name][1:]

                # Get trajectory
                code,pos_to_move_to=self._arm.get_inverse_kinematics(
                    currently_at,False,False)

                code = self._arm.set_position(*pos_to_move_to, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                
                # Close gripper
                code = self._arm.set_gripper_position(0, wait=True, speed=5000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # set new weight
                code = self._arm.set_tcp_load(0, [0, 0, 0])
                if not self._check_code(code, 'set_tcp_load'):
                    return
                
                # move to final
                code = self._arm.set_position(*final_location, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                
                # Drop object
                code = self._arm.set_gripper_position(800, wait=True, speed=5000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # Return to start
                code = self._arm.set_position(*[-79.8, 412.1, 400.0, 180.0, 0.0, 76.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                
                interval = time.monotonic() - t1
                if interval < 0.01:
                    time.sleep(0.01 - interval)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)
            if hasattr(self._arm, 'release_count_changed_callback'):
                self._arm.release_count_changed_callback(self._count_changed_callback)

def get_pose(name,detector,cap):
    detector.addFamily(name)
    ret, frame = cap.read()
    gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags=detector.detect(gray)
    # Inititalize estimator  (Read documentation for Config parameters which differ based on camera) 
    pose_estimator=robotpy_apriltag.AprilTagPoseEstimator(robotpy_apriltag.AprilTagPoseEstimator.Config(0.0772,20,20,20,20))
    if len(tags)>=1:
        tag=tags[0]
        
        # Estimate tag pose
        pose=pose_estimator.estimate(tag)
        coor=(pose.translation().X(),pose.translation().Y(),pose.translation().Z())
        rotate=(pose.rotation().X(),pose.rotation().Y(),pose.rotation().Z())
        output=list(coor+rotate)
        return output
    else:
        raise ModuleNotFoundError

if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('172.17.0.2', baud_checkset=False)
    coors={"tag36h11":[(3,1)]}
    cap = cv2.VideoCapture(0)
    detector=robotpy_apriltag.AprilTagDetector()
    robot_main = RobotMain(arm)
    robot_main.run(coors,detector,cap)
