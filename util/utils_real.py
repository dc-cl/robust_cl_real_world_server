import sys
sys.path.append('../')
from pathlib import Path
import numpy as np
from math import pi
import rospy
import matplotlib.pyplot as plt
import scienceplots
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import parameters as para
from nlink_parser.msg import LinktrackNodeframe2


NUM_ROBOTS = para.NUM_ROBOTS
start_time = None

# 获取真值
class TopicSubscriber:
    def __init__(self, topic_name):
        # 初始化 ROS 节点
        rospy.init_node('topic_subscriber', anonymous=True)
        # 订阅话题
        self.sub = rospy.Subscriber(topic_name, LinktrackNodeframe2, self.callback)
        # 存储数据
        self.data_list = []

    def callback(self, msg_data,msg_nodes):
        self.id = msg_data.id
        self.data_list.append(msg_data.pos_3d)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
data = []
a = TopicSubscriber('LinktrackNodeframe2_0')
b = TopicSubscriber('LinktrackNodeframe2_1')
c = TopicSubscriber('LinktrackNodeframe2_2')
a.run()
b.run()
c.run()
for subscriber in (a, b, c):
    if subscriber.id == 0:
        data[0] = subscriber
    if subscriber.id == 2:
        data[1] = subscriber


def init(init_X):

    global start_time
    # Initialize
    true_robots = init_X.copy()

    str_start_time = '/start_time'
    while not rospy.has_param(str_start_time):
        rospy.sleep(0.1)
    start_time = rospy.get_param(str_start_time)




# 画图
marker = Marker()
# marker.header.frame_id = "base_link"  # Set the frame ID
# marker.header.stamp = rospy.Time.now()
marker.type = Marker.POINTS
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.1  # Point size
marker.scale.y = 0.1
marker.color.r = 1.0  # Red color
marker.color.a = 1.0  # Full opacity，这是颜色的 Alpha 分量，控制透明度

def draw_true(data):
    # TODO 接收真值 TBC
    point = Point()
    point.x = data.x
    point.y = data.y
    point.z = data.z

if __name__ == '__main__':

    rospy.init_node('server', anonymous=False) # 

    rospy.Subscriber('true', Float64MultiArray, draw_true)
    # 真值
    # for r in range(NUM_ROBOTS):
    #     rospy.Subscriber('client' + str(r), Float64MultiArray, draw_tra)
    
    rospy.spin()
