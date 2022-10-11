import cv2
import glob
import pyrealsense2
from cv2 import aruco
from realsense_depth import *
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

point = (400, 300)

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)

def cal():
    allCorners = []
    allIds = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    images = glob.glob('/home/x/Downloads/study/tb3/aruco_pycharm/src/camera_cal_img/cal*.png')
    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
                allCorners.append(res2[1])
                allIds.append(res2[2])

    imsize = gray.shape
    return allCorners, allIds, imsize

def calibrate_charuco(allCorners, allIds, imsize):
    cameraMatrixInit = np.array([[249.5, 0., imsize[1] / 2.],
                                 [0., 249.5, imsize[0] / 2.],
                                 [0., 0., 1.]])
    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

# Create mouse event
cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

def detect_marker(mtx, dist):
    param = cv2.aruco.DetectorParameters_create()
    aruco_pub = rospy.Publisher('aruco_xyzw', Pose, queue_size=1)
    check_pub = rospy.Publisher('check_aruco', Bool, queue_size=1)
    rate = rospy.Rate(10)
    aruco = Pose()
    check = Bool()
    while not rospy.is_shutdown():
        ret, depth_frame, color_frame = dc.get_frame()
        gray_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        coners, ids, p_ = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=param)
        if np.all(ids != None):
            check.data = True
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, 0.02, mtx, dist)
            color_frame = cv2.aruco.drawAxis(color_frame, mtx, dist, rvecs[0], tvecs[0], 0.02)
            rvecs_msg = rvecs.tolist()
            tvecs_msg = tvecs.tolist()
            aruco.orientation.x = rvecs_msg[0][0][0]
            aruco.orientation.y = rvecs_msg[0][0][1]
            aruco.orientation.z = rvecs_msg[0][0][2]
            aruco.position.x = tvecs_msg[0][0][0]
            aruco.position.y = tvecs_msg[0][0][1]
            aruco.position.z = tvecs_msg[0][0][2]
            aruco_pub.publish(aruco)
        else:
            check.data = False
        check_pub.publish(check)
        rate.sleep()
        color_frame = cv2.aruco.drawDetectedMarkers(color_frame, coners, ids)
        # Show distance for a specific point
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1], point[0]]

        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)

def main():
    rospy.init_node("aruco_pub")
    allCorners, allIds, imsize = cal()
    print("Calibration is Completed. Starting tracking marker.")
    ret, mtx, dist, rvec, tvec = calibrate_charuco(allCorners, allIds, imsize)
    detect_marker(mtx, dist)


if __name__ == "__main__":
    main()