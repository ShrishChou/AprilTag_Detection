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
from xarm.wrapper import XArmAPI
import cv2
import numpy as np
from xarm.wrapper import XArmAPI
import os
import sys
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
def hangle_err_warn_changed(item):
    print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))
print("1")
arm = XArmAPI(ip, is_radian=False)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
print("fail")
# arm.reset(wait=True)

arm.clean_error()
print("2")
# arm = XArmAPI('COM5')
# arm = XArmAPI('192.168.1.241')
# arm = XArmAPI('192.168.1.241', do_not_open=False)
# arm = XArmAPI('192.168.1.241', is_radian=False)
    # Robot Main Run
def start():
    try:
        # Install xArm Gripper
        code = arm.set_counter_reset()
        t1 = time.monotonic()

        code=arm.set_servo_angle(angle=[0,0,0,0,0,-90,0],is_radian=False,speed=100)
        interval = time.monotonic() - t1
        if interval < 0.01:
            time.sleep(0.01 - interval)
    except Exception as e:
        print('MainException: {}'.format(e))
    # finally:
    #     self.alive = False
    #     self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
    #     self._arm.release_state_changed_callback(self._state_changed_callback)
    #     if hasattr(self._arm, 'release_count_changed_callback'):
    #         self._arm.release_count_changed_callback(self._count_changed_callback)

def pickup(arm,coor):
    highcoor=coor[:2]+[400]+coor[3:]
    # code = arm.set_gripper_position(800, wait=True, speed=5000, auto_enable=True)
    # if not self._check_code(code, 'set_gripper_position'):
    #     return
    x=coor[0]
    y=coor[1]
    new_angle=math.atan2(y,x)/math.pi*180
    
    print("***************************",new_angle,coor)
    code=arm.set_servo_angle(servo_id=1,angle=new_angle,wait=True,is_radian=False,speed=70)
    # if not self._check_code(code, 'set_servo_angle'):
    #     return
    print("here*******",new_angle,coor)

    code = arm.set_position_aa(highcoor, speed=200, mvacc=100, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return
    print("**************here",new_angle,coor)

    code = arm.set_position_aa(coor,speed=100, wait=True)
    # arm.set_pause_time(0.2)

    # if not self._check_code(code, 'set_position'):
    #     return
    # code = arm.set_gripper_position(40, wait=True, speed=5000, auto_enable=True)
    # if not self._check_code(code, 'set_gripper_position'):
    #     return
    print("here1")

    code = arm.set_position_aa(highcoor, speed=100,mvacc=100, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return
    print("here")
    code=arm.set_servo_angle(angle=[new_angle,0,0,0,0,-90,0],is_radian=False)
    # if not self._check_code(code, 'set_servo_angle'):
    #     return
    print("h")
    return

def drop(arm,coor):
    highcoor=coor[:2]+[400]+coor[3:]
    x=coor[0]
    y=coor[1]
    new_angle=math.atan2(y,x)/math.pi*180
    code=arm.set_servo_angle(servo_id=1,wait=True,angle=new_angle,is_radian=False,speed=70)
    # if not self._check_code(code, 'set_servo_angle'):
    #     return
    code = arm.set_position_aa(highcoor,is_radian=False, speed=arm.tcp_speed_limit,  mvacc=arm.tcp_acc_limit, radius=0.0, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return

    code = arm.set_position_aa(coor,is_radian=False, speed=arm.tcp_speed_limit,  mvacc=arm.tcp_acc_limit, radius=0.0, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return
    # code = arm.set_gripper_position(800, wait=True, speed=5000, auto_enable=True)
    # if not self._check_code(code, 'set_gripper_position'):
    #     return
    code = arm.set_position_aa(highcoor,is_radian=False, speed=arm.tcp_speed_limit,  mvacc=arm.tcp_acc_limit, radius=0.0, wait=True)
    # if not self._check_code(code, 'set_position'):
    #     return
    code=arm.set_servo_angle(angle=[new_angle,0,0,0,0,-90,0],is_radian=False)
    # if not self._check_code(code, 'set_servo_angle'):
    #     return
    return
def validate(self,coor):
    x=coor[0]
    y=coor[1]
    code,val=self._arm.get_position_aa(is_radian=False)
    x_curr=val[0]
    y_curr=val[1]
    quad=0
    quadcurr=0
    if x>=0 and y>=0:
        quad=1
    elif x<=0 and y>=0:
        quad=2
    elif x<=0 and y<=0:
        quad=3
    else:
        quad=4

    if x_curr>=0 and y_curr>=0:
        quad_curr=1
    elif x_curr<=0 and y_curr>=0:
        quad_curr=2
    elif x_curr<=0 and y_curr<=0:
        quad_curr=3
    else:
        quad_curr=4

    code,angle=self._arm.get_servo_angle(servo_id=1,is_radian=False)
    print("angle",angle)
    new_angle=angle+(quad-quadcurr)*90
    new_angle=new_angle%360
    print("new_angle",new_angle)
    code=self._arm.set_servo_angle(servo_id=1,wait=True,angle=new_angle,is_radian=False)

    code = self._arm.set_position_aa(coor,is_radian=False, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
    if not self._check_code(code, 'set_position'):
        return
    return
def average_pose(rvec1, tvec1, rvec2, tvec2):
        # Average the translation vectors
        tvec_avg = (tvec1 + tvec2) / 2

        # Convert rotation vectors to matrices
        R1, _ = cv2.Rodrigues(rvec1)
        R2, _ = cv2.Rodrigues(rvec2)

        # Average the rotation matrices
        R_avg = (R1 + R2) / 2

        # Ensure R_avg is a valid rotation matrix by orthogonalizing it using SVD
        U, _, Vt = np.linalg.svd(R_avg)
        R_avg_orthogonal = U @ Vt

        # Convert the averaged rotation matrix back to a rotation vector
        rvec_avg, _ = cv2.Rodrigues(R_avg_orthogonal)

        return rvec_avg, tvec_avg


if __name__ == '__main__':
    # RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    # Load calibration data
    calibration_data = np.load('stereo_calibration.npz')
    mtx1 = calibration_data['mtx1']
    dist1 = calibration_data['dist1']
    mtx2 = calibration_data['mtx2']
    dist2 = calibration_data['dist2']
    R = calibration_data['R']
    T = calibration_data['T']

    # Compute projection matrices
    proj1 = mtx1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
    proj2 = mtx2 @ np.hstack((R, T))

    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # Define the ArUco marker size (in meters)
    aruco_size = 0.065

    # Define the 3D points of the ArUco marker corners (assuming square markers)
    aruco_corners_3d = np.array([
        [-aruco_size / 2, -aruco_size / 2, 0],
        [aruco_size / 2, -aruco_size / 2, 0],
        [aruco_size / 2, aruco_size / 2, 0],
        [-aruco_size / 2, aruco_size / 2, 0]
    ], dtype=np.float32)

    # Scaling factor
    scale_factor = 1.22 / 0.74

    # Main loop for real-time detection
    cap1 = cv2.VideoCapture(1, cv2.CAP_MSMF)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap2 = cv2.VideoCapture(2, cv2.CAP_MSMF)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

    target_id = 4
    start()
    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            print("Error: Could not read frame from one or both cameras.")
            break

        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners1, ids1, _ = detector.detectMarkers(gray1)
        corners2, ids2, _ = detector.detectMarkers(gray2)
        pose1=None
        pose2=None
        if ids1 is not None and target_id in ids1.flatten():
            idx1 = np.where(ids1.flatten() == target_id)[0][0]
            corners1_target = corners1[idx1].reshape(-1, 2)

            # Estimate pose using solvePnP
            success1, rvec1, tvec1 = cv2.solvePnP(aruco_corners_3d, corners1_target, mtx1, dist1)

            if success1:
                # Scale and adjust the position
                tvec1 = tvec1 * scale_factor
                

                # Draw the detected marker and axes
                cv2.aruco.drawDetectedMarkers(frame1, corners1)
                cv2.drawFrameAxes(frame1, mtx1, dist1, rvec1, tvec1, 0.1)
                tvec1[2] = 1.3-tvec1[2]
                # Convert rotation vector to a rotation matrix
                R1, _ = cv2.Rodrigues(rvec1)
                pose_matrix1 = np.hstack((R1, tvec1))

                # Print position and orientation
                print(f"Position (Camera 1): {tvec1.flatten()}")
                pose1=tvec1.flatten()
                print(f"Rotation Matrix (Camera 1):\n{R1}")

        if ids2 is not None and target_id in ids2.flatten():
            idx2 = np.where(ids2.flatten() == target_id)[0][0]
            corners2_target = corners2[idx2].reshape(-1, 2)

            # Estimate pose using solvePnP
            success2, rvec2, tvec2 = cv2.solvePnP(aruco_corners_3d, corners2_target, mtx2, dist2)

            if success2:
                # Scale and adjust the position
                tvec2 = tvec2 * scale_factor

                # Draw the detected marker and axes
                cv2.aruco.drawDetectedMarkers(frame2, corners2)
                cv2.drawFrameAxes(frame2, mtx2, dist2, rvec2, tvec2, 0.1)
                tvec2[2] = 1.3-tvec2[2]

                # Convert rotation vector to a rotation matrix
                R2, _ = cv2.Rodrigues(rvec2)
                pose_matrix2 = np.hstack((R2, tvec2))

                # Print position and orientation
                print(f"Position (Camera 2): {tvec2.flatten()}")
                pose2=tvec2.flatten()
                print(f"Rotation Matrix (Camera 2):\n{R2}")
        if pose1 is not None and pose2 is not None:

            rvec_avg, tvec_avg = average_pose(rvec1, tvec1, rvec2, tvec2)
            print(f"Averaged Position:{tvec_avg.flatten()}")
            print(f"Averaged Rotation Vector: {rvec_avg.flatten()}")
            pickup(arm,tvec_avg.flatten()+[0,0,0])
        # Display frames
        cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera 1', 3840, 2160)
        cv2.imshow('Camera 1', frame1)
        cv2.namedWindow('Camera 2', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera 2', 3840, 2160)
        cv2.imshow('Camera 2', frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
