#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import time
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion

def load_model(model_path):
    with open(model_path, 'r') as model_file:
        model_xml = model_file.read()
    return model_xml

def spawn_model(model_name, model_xml, pose, reference_frame="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(model_name, model_xml, "", pose, reference_frame)
        rospy.loginfo("Model {} spawned successfully".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_prox(model_name)
        rospy.loginfo("Model {} deleted successfully".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def set_model_state(model_name, pose):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state_prox = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = pose
        set_model_state_prox(model_state)
        rospy.loginfo("Model {} moved to new position".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    rospy.init_node('move_model_node')

    model_name = "cafe_table"
    model_path = "/home/asic/.gazebo/models/cafe_table/model.sdf"

    model_xml = load_model(model_path)

    # 初始位置和姿态
    start_pose = Pose()
    start_pose.position = Point(x=1, y=2, z=0)
    start_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

    end_pose = Pose()
    end_pose.position = Point(x=3, y=2, z=0)
    end_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

    # 生成模型
    spawn_model(model_name, model_xml, start_pose)

    # 在两点之间来回移动
    rate = rospy.Rate(1)  # 1 Hz
    direction = 1
    current_pose = start_pose

    while not rospy.is_shutdown():
        if direction == 1:
            current_pose.position.x += 0.1
            if current_pose.position.x >= end_pose.position.x:
                direction = -1
        else:
            current_pose.position.x -= 0.1
            if current_pose.position.x <= start_pose.position.x:
                direction = 1

        set_model_state(model_name, current_pose)
        rate.sleep()

    # 删除模型
    delete_model(model_name)