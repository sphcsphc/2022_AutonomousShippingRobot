#!/home/scout/.pyenv/versions/spkm_py2718/bin/python
# -- coding: utf-8 --

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from open3d_ros_helper import open3d_ros_helper as orh


def main():
    rospy.init_node("lidar_test")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    rospy.Subscriber('velodyne_points', PointCloud2, driver.lds_callback)
    rospy.spin()


class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher

        self.data_array = np.array([])

        self.ahead_b = np.array([])
        self.ahead_u = np.array([])
        self.ahead_l = np.array([])
        self.ahead_r = np.array([])
        self.ahead_p = np.array([])
        self.ahead_n = np.array([])

        self.side_l = np.array([])
        self.side_r = np.array([])

    def lds_callback(self, vel_po):
        turtle_vel = Twist()
        data = orh.rospc_to_o3dpc(vel_po)
        self.preprocessing(data)

        self.predict_ahead()

        self.moving()

        turtle_vel.linear.x = self.speed_x
        turtle_vel.angular.z = self.speed_z
        self.publisher.publish(turtle_vel)

    def preprocessing(self, data):
        self.data_array = np.asarray(data.points)
        self.data_array = self.data_array[
            np.logical_and(np.logical_or(self.data_array[:, 0] > -0.45, self.data_array[:, 0] < -0.47),
                           np.abs(self.data_array[:, 1]) > 0.17)]
        self.data_array = self.data_array[np.logical_and(self.data_array[:, 2] > -0.55, self.data_array[:, 2] < 0.5)]

    def visualizing(self, data):
        data.points = o3d.utility.Vector3dVector(self.data_array)
        o3d.visualization.draw_geometries([data])

    def predict_ahead(self):
        self.ahead_b = np.logical_and(
            np.logical_and(
                np.logical_and(self.data_array[:, 0] > 0., self.data_array[:, 0] < 0.65),
                np.logical_and(self.data_array[:, 1] > -0.4, self.data_array[:, 1] < 0.4)),
            np.logical_and(self.data_array[:, 2] >= -0.55, self.data_array[:, 2] < -0.2)
        )
        self.ahead_u = np.logical_and(
            np.logical_and(
                np.logical_and(self.data_array[:, 0] > 0., self.data_array[:, 0] < 0.65),
                np.logical_and(self.data_array[:, 1] > -0.4, self.data_array[:, 1] < 0.4)),
            np.logical_and(self.data_array[:, 2] >= -0.2, self.data_array[:, 2] < 0.)
        )
        self.ahead_l = np.logical_and(np.logical_and(self.data_array[:, 0] > 0., self.data_array[:, 0] < 0.6),
                         np.logical_and(self.data_array[:, 1] > 0., self.data_array[:, 1] < 0.4))
        self.ahead_r = np.logical_and(np.logical_and(self.data_array[:, 0] > 0., self.data_array[:, 0] < 0.6),
                                 np.logical_and(self.data_array[:, 1] > -0.4, self.data_array[:, 1] < 0.))
        self.ahead_p = np.logical_and(np.logical_and(self.data_array[:, 0] >= 0.6, self.data_array[:, 0] < 1.2),
                                      np.logical_and(self.data_array[:, 1] > -0.4, self.data_array[:, 1] < 0.4))
        self.ahead_n = np.logical_and(np.logical_and(self.data_array[:, 0] >= 1.2, self.data_array[:, 0] < 1.8),
                                 np.logical_and(self.data_array[:, 1] > -0.4, self.data_array[:, 1] < 0.4))

        self.side_l = np.logical_and(
            np.logical_and(
                np.logical_and(self.data_array[:, 0] > -0.45, self.data_array[:, 0] < 0.),
                np.logical_and(self.data_array[:, 1] >= 0.4, self.data_array[:, 1] < 0.68)),
            np.logical_and(self.data_array[:, 2] >= -0.55, self.data_array[:, 2] < -0.2)
        )
        self.side_r = np.logical_and(
            np.logical_and(
                np.logical_and(self.data_array[:, 0] > -0.45, self.data_array[:, 0] < 0.),
                np.logical_and(self.data_array[:, 1] > -0.68, self.data_array[:, 1] <= -0.4)),
            np.logical_and(self.data_array[:, 2] >= -0.55, self.data_array[:, 2] < -0.2)
        )

    def moving(self):
        if self.speed_x < 0:
            self.speed_x = 0.2
        if True in self.ahead_b and True not in self.ahead_u:
            rospy.loginfo("glass clear")
            self.speed_x = 0.1
            self.speed_z = 0.3
        elif True in self.ahead_r and True in self.ahead_l:  # 장애물이 양 옆에 있어서 목표물과의 거리로 가까운 곳으로 회전하게 한다
            self.speed_x = -0.2
            self.speed_z = 0.3
        elif True in self.ahead_l:  # 우회전
            self.speed_x = 0.1
            self.speed_z = -0.3
        elif True in self.ahead_r:  # 좌회전
            self.speed_x = 0.1
            self.speed_z = 0.3
        elif True in self.ahead_p:
            self.speed_x *= 0.6
            if self.speed_x < 0.20:
                self.speed_x = 0.20
            self.speed_z = 0.
        elif True in self.ahead_n:
            self.speed_x *= 0.9
            if self.speed_x < 0.30:
                self.speed_x = 0.30
            self.speed_z = 0.
        else:
            self.speed_x *= 1.2
            if self.speed_x > 1.0:
                self.speed_x = 1.0
            self.speed_z = 0.

        if True in self.side_l and True in self.side_r:
            self.speed_x = -0.2
            self.speed_z = 0.1
        elif True in self.side_l:
            self.speed_z = -0.1
        elif True in self.side_r:
            self.speed_z = 0.1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
