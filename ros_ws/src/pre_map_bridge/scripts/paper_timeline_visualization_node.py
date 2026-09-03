#!/usr/bin/env python3
"""Interactive RViz timeline for paper exploration figures."""
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, MarkerArray


class TimelineView:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.width = float(rospy.get_param("~trajectory_width", 0.10))
        self.lift = float(rospy.get_param("~trajectory_lift", 0.18))
        self.data = np.load(Path(rospy.get_param("~timeline_path")), allow_pickle=False)
        self.fractions = self.data["fractions"]
        self.times = self.data["snapshot_times"]
        self.trajectory = self.data["trajectory"]
        self.trajectory_times = self.data["trajectory_times"]
        self.final_cloud = self.data[f"cloud_{len(self.fractions)-1:03d}"]
        self.explored_pub = rospy.Publisher("/pre_map_vln/timeline_explored", PointCloud2, queue_size=1, latch=True)
        self.context_pub = rospy.Publisher("/pre_map_vln/timeline_final_context", PointCloud2, queue_size=1, latch=True)
        self.trajectory_pub = rospy.Publisher("/pre_map_vln/timeline_trajectory", MarkerArray, queue_size=1, latch=True)
        self.server = InteractiveMarkerServer("paper_timeline")
        self.minimum_x = float(np.percentile(self.final_cloud[:, 0], 1))
        self.maximum_x = float(np.percentile(self.final_cloud[:, 0], 99))
        self.slider_y = float(np.percentile(self.final_cloud[:, 1], 1) - 1.2)
        self.slider_z = float(np.percentile(self.final_cloud[:, 2], 99) + 0.8)
        self.index = int(round(float(rospy.get_param("~initial_percent", 35.0)) * (len(self.fractions)-1) / 100.0))
        self.make_slider()
        self.publish_context()
        self.publish_index(self.index)

    def cloud(self, points):
        return point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(), frame_id=self.frame_id), points.tolist())

    def make_slider(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = "exploration_percent"
        marker.description = "Drag: exploration time"
        marker.scale = 0.8
        marker.pose.position.x = self.minimum_x + self.fractions[self.index] * (self.maximum_x-self.minimum_x)
        marker.pose.position.y = self.slider_y
        marker.pose.position.z = self.slider_z
        visible = InteractiveMarkerControl()
        visible.orientation.w = 1.0
        visible.always_visible = True
        knob = Marker(type=Marker.SPHERE)
        knob.scale.x = knob.scale.y = knob.scale.z = 0.42
        knob.color.r, knob.color.g, knob.color.b, knob.color.a = 0.88, 0.08, 0.04, 1.0
        visible.markers.append(knob)
        marker.controls.append(visible)
        move = InteractiveMarkerControl()
        move.name = "move_time"
        move.orientation.w = 0.70710678
        move.orientation.x = 0.70710678
        move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move)
        self.server.insert(marker, self.feedback)
        self.server.applyChanges()
        self.publish_slider_bar()

    def publish_slider_bar(self):
        pub = rospy.Publisher("/pre_map_vln/timeline_slider_bar", MarkerArray, queue_size=1, latch=True)
        result = MarkerArray()
        line = Marker()
        line.header.frame_id = self.frame_id
        line.ns, line.id, line.type, line.action = "timeline", 0, Marker.LINE_LIST, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.07
        line.color.r = line.color.g = line.color.b = 0.25
        line.color.a = 0.9
        line.points = [Point(self.minimum_x, self.slider_y, self.slider_z), Point(self.maximum_x, self.slider_y, self.slider_z)]
        result.markers.append(line)
        pub.publish(result)
        self.slider_bar_pub = pub

    def feedback(self, feedback):
        ratio = np.clip((feedback.pose.position.x-self.minimum_x) / (self.maximum_x-self.minimum_x), 0.0, 1.0)
        index = int(round(ratio * (len(self.fractions)-1)))
        if index != self.index:
            self.index = index
            self.publish_index(index)
        feedback.pose.position.x = self.minimum_x + self.fractions[index] * (self.maximum_x-self.minimum_x)
        feedback.pose.position.y = self.slider_y
        feedback.pose.position.z = self.slider_z
        self.server.setPose("exploration_percent", feedback.pose)
        self.server.applyChanges()

    def publish_context(self):
        self.context_pub.publish(self.cloud(self.final_cloud))

    def publish_index(self, index):
        self.explored_pub.publish(self.cloud(self.data[f"cloud_{index:03d}"]))
        cutoff = self.times[index]
        count = int(np.searchsorted(self.trajectory_times, cutoff, side="right"))
        shown = self.trajectory[:count].copy()
        if len(shown):
            shown[:, 2] += self.lift
        result = MarkerArray()
        path = Marker()
        path.header.frame_id = self.frame_id
        path.ns, path.id, path.type, path.action = "executed", 0, Marker.LINE_STRIP, Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = self.width
        path.color.r, path.color.g, path.color.b, path.color.a = 1.0, 0.30, 0.02, 1.0
        path.points = [Point(*point) for point in shown]
        result.markers.append(path)
        if len(shown):
            current = Marker()
            current.header.frame_id = self.frame_id
            current.ns, current.id, current.type, current.action = "current", 1, Marker.SPHERE, Marker.ADD
            current.pose.position = Point(*shown[-1])
            current.pose.orientation.w = 1.0
            current.scale.x = current.scale.y = current.scale.z = 0.32
            current.color.r, current.color.g, current.color.b, current.color.a = 0.85, 0.02, 0.04, 1.0
            result.markers.append(current)
        self.trajectory_pub.publish(result)
        rospy.loginfo("Timeline set to %.0f%% (%.1f s), %d occupied points", 100*self.fractions[index], cutoff-self.times[0], len(self.data[f"cloud_{index:03d}"]))


if __name__ == "__main__":
    rospy.init_node("paper_timeline_visualization")
    TimelineView()
    rospy.spin()
