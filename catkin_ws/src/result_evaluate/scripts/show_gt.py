#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

import csv

pose_array = []
pub_pose = None

def pose_cb(msg):
    sec = msg.header.stamp.to_sec()
    rospy.loginfo("got msg with stamp: %f" % sec)

    for pose in pose_array:
        if float(pose[0]) == sec:
            q = tf.transformations.quaternion_from_euler(float(pose[6]), float(pose[5]), float(pose[4]))
            gt = PoseStamped()
            gt.header = msg.header
            gt.pose.position.x = float(pose[1])
            gt.pose.position.y = float(pose[2])
            gt.pose.position.z = float(pose[3])
            gt.pose.orientation.x = q[0]
            gt.pose.orientation.y = q[1]
            gt.pose.orientation.z = q[2]
            gt.pose.orientation.w = q[3]
            
            pub_pose.publish(gt)

            
            # rospy.loginfo("found 1 match")


if __name__ == "__main__":
    rospy.init_node("show_gt_node")

    sub_pose = rospy.Subscriber("/car_pose", PoseStamped, pose_cb, queue_size=1)
    file_name = rospy.get_param("file_name", "/home/biomotion/bnn-localization/bags/itri/ITRI_Public_Ground_truth.csv")
    pub_pose = rospy.Publisher("/gt_pose", PoseStamped, queue_size=1)
    with open(file_name, "r") as csv_file:
        rows = csv.reader(csv_file)
        for row in rows:
            pose_array.append(row)

        csv_file.close()

    rospy.loginfo("initialize done")

    rospy.spin()
    