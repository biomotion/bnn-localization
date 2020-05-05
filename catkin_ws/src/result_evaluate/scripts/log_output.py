#!/usr/bin/env python

import rospy
import csv
import tf
from geometry_msgs.msg import PoseStamped


output = []

def pose_cb(msg):
    rospy.loginfo("recieve pose\n%s" % str(msg.pose))
    # msg = PoseStamped()
    data = [msg.header.stamp.to_sec(), 
            msg.pose.position.x, 
            msg.pose.position.y, 
            msg.pose.position.z]
    q = (msg.pose.orientation.x,
         msg.pose.orientation.y,
         msg.pose.orientation.z,
         msg.pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)
    data.append(rpy[2])
    data.append(rpy[1])
    data.append(rpy[0])

    output.append(data)

def on_shutdown():
    with open("test.csv",'w') as csv_file:
        csv_writer = csv.writer(csv_file)
        for row in output:
            csv_writer.writerow(row)
        csv_file.close()

if __name__ == "__main__":
    rospy.init_node("log_csv_node")

    # sub_pose = rospy.Subscriber("/car_combine", PoseStamped, pose_cb)
    sub_pose = rospy.Subscriber("/car_pose", PoseStamped, pose_cb)
    rospy.on_shutdown(on_shutdown)
    rospy.spin()