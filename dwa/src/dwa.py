#!/home/scout/.pyenv/versions/rospy368/bin/python
# -- coding: utf-8 --

import numpy as np
import rospy
import tf
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Pose2D
from open3d_ros_helper import open3d_ros_helper as orh


def main():
    rospy.init_node('dwa_test')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = DWA(publisher)
    rospy.Subscriber('velodyne_points', PointCloud2, driver.lds_callback)
    rospy.spin()


class DWA:
    def __init__(self, publisher):
        rospy.Subscriber('dwa_motion_start', String, self.control_callback)
        rospy.Subscriber('camera1_aruco_xyz', Pose, self.camera_callback)
        rospy.Subscriber('current_pose', Pose, self.current_pose)
        rospy.Subscriber('which_print', String, self.print_callback)
        self.output = rospy.Publisher('dwa_motion_fin', String, queue_size=1)
        self.publisher = publisher
        self.scout_pose = Pose2D()
        self.goal_pos = [[0.00, 0.00], [0.00, 0.00], [0.00, 0.00]]
        self.home_pos = [0.00, 0.00]
        self.goal = list()
        self.dwa_mode = "none"
        self.prn = "dwa"
        self.check_point = 0

    def control_callback(self, data):
        self.dwa_mode = data.data
        if self.prn == "control":
            rospy.loginfo("{}".format(self.dwa_mode))

    def camera_callback(self, data):
        self.goal_pos.append([data.position.x, data.position.y])
        self.check_point = len(self.goal_pos) - 1
        if self.prn == "aruco":
            rospy.loginfo("{} {}".format(self.goal_pos[0], self.goal_pos[1]))

    def slam_callback(self, data):
        self.scout_pose.x = data.position.x
        self.scout_pose.y = data.position.y
        _, _, self.scout_pose.theta = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        if self.prn == "slam":
            rospy.loginfo("{} {} {}".format(self.scout_pose.x, self.scout_pose.x, self.scout_pose.theta))

    def print_callback(self, data):
        self.prn = data.data
    
    def lds_callback(self, velo):
        turtle_vel = Twist()
        mps = [0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24, 0.26, 0.28]
        radps = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
                 -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9]
        data = orh.rospc_to_o3dpc(velo)
        data_array = self.preprocessing(data)
        local_pos = self.make_combination(mps, radps)
        pos_candidates = self.create_pos_candidates(local_pos)
        if self.dwa_mode == "go_to_aruco":
            self.goal = self.goal_pos[self.check_point]
        elif self.dwa_mode == "go_to_home":
            self.goal = self.home_pos
        else:
            self.output.publish("none")
            return
        if np.hypot(self.goal[0] - self.scout_pose.x, self.goal[1] - self.scout_pose.y) <= 0.8 \
                and self.check_point < len(self.goal_pos) - 1:
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
            self.check_point += 1
        elif np.hypot(self.goal[0] - self.scout_pose.x, self.goal[1] - self.scout_pose.y) <= 0.8:
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
            self.check_point = 0
            self.dwa_mode = "none"
            self.output.publish("dwa_fin")
        else:
            best_score, back_check = self.evaluate_scores(pos_candidates, self.goal, data_array, local_pos)
            turtle_vel.linear.x, turtle_vel.angular.z = mps[best_score[0]], radps[best_score[1]]
            if back_check:
                turtle_vel.linear.x = mps[best_score[0]] * -1
        if self.prn == "dwa":
            rospy.loginfo("{} {}".format(turtle_vel.linear.x, turtle_vel.angular.z))
        self.publisher.publish(turtle_vel)

    @staticmethod
    def preprocessing(data):
        data_array = np.asarray(data.points)
        data_array = data_array[(data_array[:, 0] > -0.45 or data_array[:, 0] < -0.47)
                                and np.abs(data_array[:, 1]) > 0.16]
        data_array = data_array[-0.55 < data_array[:, 2] < 0.5]
        data_array = data_array[-0.1 < data_array[:, 0] < 2.0 and -2.0 < data_array[:, 1] < 2.0]
        return data_array

    @staticmethod
    def make_combination(mps, radps):
        line_motions = np.array(mps).reshape(len(mps), 1)
        radps_array = np.delete(np.array(radps), 0)
        rotational_motions = 2 * (line_motions / radps_array) * np.sin(0.5 * radps_array) + 0.1
        local_x_pos = np.concatenate((line_motions, rotational_motions * np.cos(0.5 * radps_array)), axis=1)
        local_y_pos = np.concatenate(
            (np.zeros((len(mps), 1)), rotational_motions * np.sin(0.5 * radps_array)), axis=1)
        local_pos = np.concatenate((np.reshape(local_x_pos, (-1, 1)), np.reshape(local_y_pos, (-1, 1))), axis=1)
        return local_pos

    def create_pos_candidates(self, local_pos):
        rotation_matrix = np.array(
            [[np.cos(self.scout_pose.theta), -np.sin(self.scout_pose.theta)],
             [np.sin(self.scout_pose.theta), np.cos(self.scout_pose.theta)]])
        rotation_trans = np.round_((np.dot(local_pos, rotation_matrix.T)), 4)
        global_x_pos = self.scout_pose.x + np.delete(rotation_trans, 1, axis=1)
        global_y_pos = self.scout_pose.y + np.delete(rotation_trans, 0, axis=1)
        global_pos = np.concatenate((global_x_pos, global_y_pos), axis=1)
        return global_pos

    def evaluate_scores(self, pos_candidates, goal_pos, data_array, local_pos):
        remaining_scores = self.find_remaining_scores(pos_candidates, goal_pos)
        obstacle_distance, obstacle_scores, clearance_scores = self.find_other_scores(data_array, local_pos)
        scores = 1.6 * remaining_scores + 1.2 * obstacle_scores + clearance_scores
        best_score = np.unravel_index(np.argmax(scores), scores.shape)
        return best_score, obstacle_distance[best_score] <= 0.45

    @staticmethod
    def find_remaining_scores(pos_candidates, goal_pos):
        x = goal_pos[0] - np.delete(pos_candidates, 1, axis=1)
        y = goal_pos[1] - np.delete(pos_candidates, 0, axis=1)
        scores = 1 / np.reshape(np.hypot(x, y), (10, 20))
        norm = np.linalg.norm(scores)
        scores = scores / norm
        return scores

    @staticmethod
    def find_other_scores(data_array, local_pos):
        x = data_array[:, 0] - np.delete(local_pos, 1, axis=1)
        y = data_array[:, 1] - np.delete(local_pos, 0, axis=1)
        obstacle_distance = np.reshape(np.amin(np.hypot(x, y), axis=1), (10, 20))
        obstacle_scores = np.where(obstacle_distance <= 0.45, 0, obstacle_distance)
        clearance_scores = np.reshape(np.mean(np.hypot(x, y), axis=1), (10, 20))
        if np.any(obstacle_scores) != 0:
            norm = np.linalg.norm(obstacle_scores)
            obstacle_scores = obstacle_scores / norm
        norm = np.linalg.norm(clearance_scores)
        clearance_scores = clearance_scores / norm
        return obstacle_distance, obstacle_scores, clearance_scores


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
