# -*- coding: utf-8 -*-
#!/usr/bin/env python
 
import os
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi,sin,cos
 

def rpy2quaternion(roll, pitch, yaw):
  x=sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
  y=sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
  z=cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
  w=cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
  return Quaternion(x,y,z,w)


# 定义生成模型的函数
def spawn_aruco_cubo_hover(name='2'):
    
    model_name = "box" + name
    model_path = "/home/leoluo0115/ros_ws/src/universal_robot/ur_gazebo/sdf/box/model.sdf"
    # 
    initial_pose = Pose(position=Point(x=-0.0, y=1, z=0.6), orientation = rpy2quaternion(0,0,1.57))
 
    # 从文件加载模型
    with open(model_path, "r") as f:
        model_xml = f.read()
 
    # 调用Gazebo的SpawnModel服务
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    resp_sdf = spawn_model(model_name, model_xml, "", initial_pose, "world")
 
    if resp_sdf.success:
        rospy.loginfo("模型 '{}' 生成成功。".format(model_name))
    else:
        rospy.logerr("模型 '{}' 生成失败。".format(model_name))
 
# 调用生成模型的函数
if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('spawn_box', anonymous=True)
        spawn_aruco_cubo_hover()
    except rospy.ROSInterruptException:
        pass