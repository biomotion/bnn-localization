#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf
import csv

if __name__=="__main__":
    rospy.init_node("visualize_csv_node")
    rospy.loginfo("[%s] initializing" % rospy.get_name())
    pub_path = rospy.Publisher("path_csv", Marker, queue_size=10)

    csv_file = rospy.get_param("csv_file", "/bags/unknown.csv")
    marker_color = rospy.get_param("marker_color", "blue")
    color = ColorRGBA()
    color.a = 1
    
    if marker_color == "blue":
        color.b = 1.0
    elif marker_color == "red":
        color.r = 1.0
    elif marker_color == "green":
        color.g = 1.0
    else:
        color.r = color.g = color.b = 1.0

    with open(csv_file, "r") as f:
        rows = csv.reader(f)
        arrow_marker = Marker()
        trajectory_marker = Marker()

        arrow_marker.type = Marker.ARROW
        arrow_marker.id = 0
        arrow_marker.header.frame_id = "map"
        arrow_marker.ns = "car_pose"
        arrow_marker.scale.x = 100
        arrow_marker.scale.y = 10
        arrow_marker.scale.z = 10
        arrow_marker.color = color

        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.id = 0
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.scale.x = 1
        trajectory_marker.color = color

        for row in rows:
            print row
            new_point = Point()
            new_point.x = float(row[1])
            new_point.y = float(row[2])
            new_point.z = float(row[3])
            y, p, r = float(row[4]), float(row[5]), float(row[6])
            quaternion = tf.transformations.quaternion_from_euler(y, p, r)
            
            arrow_marker.id += 1
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = new_point.x
            arrow_marker.pose.position.y = new_point.y
            arrow_marker.pose.position.z = new_point.z
            arrow_marker.pose.orientation = quaternion
            arrow_marker.header.stamp = rospy.Time.from_sec(float(row[0]))

            trajectory_marker.id += 1
            trajectory_marker.action = Marker.ADD
            trajectory_marker.points.append(new_point)
            trajectory_marker.header.stamp = rospy.Time.from_sec(float(row[0]))
            pub_path.publish(arrow_marker)
            pub_path.publish(trajectory_marker)

            # rospy.sleep(0.01)

            
