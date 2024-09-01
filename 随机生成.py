#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import rospy
import random #随机
from gazebo_msgs.srv import SpawnModel, DeleteModel
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

if __name__ == "__main__":
    rospy.init_node('spawn_random_models_node')

    model_name_prefix = "cafe_table_"
    model_path = "/home/asic/.gazebo/models/cafe_table/model.sdf"
    model_xml = load_model(model_path)
    num_models = 10

    # 地图边界定义（
    x_min, x_max = -10, 10
    y_min, y_max = -10, 10

    # 生成随机位置并生成模型
    for i in range(num_models):
        model_name = model_name_prefix + str(i)
        pose = Pose()
        pose.position = Point(
            x=random.uniform(x_min, x_max),
            y=random.uniform(y_min, y_max),
            z=0
        )
        pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        spawn_model(model_name, model_xml, pose)

    for i in range(num_models):
        model_name = model_name_prefix + str(i)
        delete_model(model_name)