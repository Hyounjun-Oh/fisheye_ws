#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# MODE 0 : Calibration Mode
# MODE 1 : Normal Mode
MODE = 0
# v4l2-ctl --list-devices

# Define the intrinsic parameters
intrinsics1 = [-0.011292810603096847, 0.6214442445507559, 526.4064917357634, 
              523.2566774987718, 1024.8874802657313, 525.7188741337991]
intrinsics2 = [-0.32996105,   0.56835387, 175.84624974, 174.74124717, 316.52955466, 241.78316385]

def main():
    rospy.init_node('fisheye_camera_publisher', anonymous=True)
    
    image_pub_1 = rospy.Publisher('/camera/image_raw_front', Image, queue_size=10)
    image_pub_2 = rospy.Publisher('/camera/image_raw_left', Image, queue_size=10)
    image_pub_3 = rospy.Publisher('/camera/image_raw_rear', Image, queue_size=10)
    image_pub_4 = rospy.Publisher('/camera/image_raw_right', Image, queue_size=10)
    bridge = CvBridge()
    
    cap = cv.VideoCapture(2)
    cap3 = cv.VideoCapture(10)
    cap2 = cv.VideoCapture(6)
    cap4 = cv.VideoCapture(14)
    
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))  # MJPG 포맷 사용
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    cap2.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))  # MJPG 포맷 사용
    cap2.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    cap3.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))  # MJPG 포맷 사용
    cap3.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap3.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    cap4.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))  # MJPG 포맷 사용
    cap4.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap4.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # Check if the cameras are opened
    if not cap.isOpened():
        rospy.logerr("카메라 1을 열 수 없습니다.")
        return
    rospy.loginfo("카메라 1이 성공적으로 열렸습니다.")

    if not cap2.isOpened():
        rospy.logerr("카메라 2를 열 수 없습니다.")
        return
    rospy.loginfo("카메라 2가 성공적으로 열렸습니다.")

    if not cap3.isOpened():
        rospy.logerr("카메라 3를 열 수 없습니다.")
        return
    rospy.loginfo("카메라 3가 성공적으로 열렸습니다.")

    if not cap3.isOpened():
        rospy.logerr("카메라 4를 열 수 없습니다.")
        return
    rospy.loginfo("카메라 4가 성공적으로 열렸습니다.")

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        ret2, frame2 = cap2.read()
        ret3, frame3 = cap3.read()
        ret4, frame4 = cap4.read()
        # Check if the frames are successfully captured
        if not ret or frame is None:
            rospy.logerr("카메라 1에서 프레임을 읽을 수 없습니다.")
            break
        if not ret2 or frame2 is None:
            rospy.logerr("카메라 2에서 프레임을 읽을 수 없습니다.")
            break
        if not ret3 or frame3 is None:
            rospy.logerr("카메라 3에서 프레임을 읽을 수 없습니다.")
            break
        if not ret4 or frame4 is None:
            rospy.logerr("카메라 4에서 프레임을 읽을 수 없습니다.")
            break
        
        rospy.loginfo("프레임을 성공적으로 읽었습니다.")

        # Fisheye 보정
        h, w = frame.shape[:2]
        h2, w2 = frame2.shape[:2]
        h3, w3 = frame2.shape[:2]

        # Convert the intrinsics to the camera matrix format (K)
        K = np.array([[intrinsics1[2], 0, intrinsics1[4]],
                      [0, intrinsics1[3], intrinsics1[5]],
                      [0, 0, 1]], dtype=np.float32)
        K2 = np.array([[intrinsics2[2], 0, intrinsics2[4]],
                      [0, intrinsics2[3], intrinsics2[5]],
                      [0, 0, 1]], dtype=np.float32)

        # Fisheye does not use distortion coefficients in this case
        D = np.zeros((4, 1), dtype=np.float32)  # Empty distortion coefficients
        
        # Get optimal new camera matrix
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        new_camera_matrix2, roi2 = cv.getOptimalNewCameraMatrix(K2, D, (w2, h2), 1, (w2, h2))

        # Undistort the image
        undistorted_image = cv.fisheye.undistortImage(frame, K, D, None, new_camera_matrix)
        undistorted_image2 = cv.fisheye.undistortImage(frame2, K2, D, None, new_camera_matrix2)

        # Publish the images
        

        cv.imshow("raw image1", frame)
        ros_image1 = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_image1.header.stamp = rospy.Time.now()
        cv.imshow("raw image2", frame2)
        ros_image2 = bridge.cv2_to_imgmsg(frame2, encoding="bgr8")
        ros_image2.header.stamp = rospy.Time.now()
        cv.imshow("raw image3", frame3)
        ros_image3 = bridge.cv2_to_imgmsg(frame3, encoding="bgr8")
        ros_image3.header.stamp = rospy.Time.now()
        cv.imshow("raw image4", frame4)
        ros_image4 = bridge.cv2_to_imgmsg(frame4, encoding="bgr8")
        ros_image4.header.stamp = rospy.Time.now()
        # cv.imshow("undistorted image1", undistorted_image)
        # cv.imshow("undistorted image2", undistorted_image2)
        cv.waitKey(1)

        image_pub_1.publish(ros_image1)
        image_pub_2.publish(ros_image2)
        image_pub_3.publish(ros_image3)
        image_pub_4.publish(ros_image4)
        rospy.loginfo("이미지 퍼블리시 완료")

        rate.sleep()

    cap.release()
    cap2.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
