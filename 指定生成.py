 
#!/usr/bin/env python3
 
-*- coding: utf-8 -*-
 
#必须添加以上两行
 
 
 
import sys
 
import os
 
sys.path.append('/usr/lib/python3/dist-packages')#手动添加路径
 
 
 
print("PYTHONPATH:", os.environ.get('PYTHONPATH'))
 
print("sys.path:", sys.path)
 
 
 
import rospy
 
from gazebo_msgs.srv import SpawnModel
 
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
 
 
 
if name == "__main__":
 
rospy.init_node('spawn_model_node')
 
 
 
model_name = "cafe_table"
 
model_path = "/home/asic/.gazebo/models/cafe_table/model.sdf"
 
 
 
model_xml = load_model(model_path)
 
pose = Pose()
 
pose.position = Point(x=1, y=2, z=0) # 修改为你需要的位置
 
pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
 
 
 
spawn_model(model_name, model_xml, pose)

