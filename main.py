import time
import enum
import imutils
import math
import depthai as dai
import numpy as np
import cv2
import socket
import lgpio
from cv2 import aruco
import depth_video_v2
from TCPLink import TCP_init, send, receive
from OAKdLink import oakd_init, set_oakd_props, aruco_init
from Controller import aruco_control, frame_counter, max_x
from depth_video_v2 import *


class Mode(enum.Enum):
    follow = 1
    push = 2
    remote = 3


def main():
    print("[INFO] Program Starting!")
    steer = 0
    # positive steer = clockwise
    speed = 0
    # positive Speed = move ahead (push the cart)
    # Units:
    #   Speed : 110 units = 1m / s
    #   steer: 40 units = 30 deg / s

    print("[INFO] Setting up oak-d cam ...")
    pipeline, camRGB, xoutVideo = oakd_init()
    set_oakd_props(camRGB, xoutVideo)

    print("[INFO] Setting up Aruco dictionary and camera coefficients ...")
    camera_matrix, distortion_coeffs, arucoDict, arucoParams = aruco_init()

    # Set default mode
    mode = Mode.push
    
    # Set GPIO
    h = lgpio.gpiochip_open(0)
    pushbtn = 23                # pushing mode button
    followbtn = 24              # follow mode button
    pushled = 22                # pushing mode led
    followled = 27              # follow mode led
    buzzer = 17                 # buzzer pin
    lgpio.gpio_claim_output(h,pushled)
    lgpio.gpio_claim_output(h,followled)
    lgpio.gpio_claim_output(h,buzzer)
    lgpio.gpio_claim_input(h,pushbtn)
    lgpio.gpio_claim_input(h,followbtn)

    # write all the outputs to 1 so the user is notified when the program starts
    lgpio.gpio_write(h, pushled, 1)
    lgpio.gpio_write(h, followled, 1)
    lgpio.gpio_write(h, buzzer, 1)        
    
    time.sleep(2.0)  # Necessary !!!

    # set the outputs back to zero except the default mode
    lgpio.gpio_write(h, pushled, 1)
    lgpio.gpio_write(h, followled, 0)
    lgpio.gpio_write(h, buzzer, 0)        

    print("[INFO] Initializing TCP connection ...")
    s = TCP_init()  # Socket object

    fc = 0  # Frame counter

    monoRight, monoLeft, stereo = depth_video_v2.depth_init(pipeline)
    xoutDisp, xoutRectifiedRight, xoutRectifiedLeft = depth_video_v2.setter(pipeline, stereo)
    # Pipeline is defined, now we can connect to the device

    # Main loop
    with dai.Device(pipeline) as device:  # used with OAK-D camera
        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)  # establish queue

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        disparityQueue, rectifiedRightQueue, rectifiedLeftQueue = depth_video_v2.get_queues(device)
        # Calculate a multiplier for colormapping disparity map
        disparityMultiplier = 255 / stereo.getMaxDisparity()

        # start = timeit.default_timer()  # Enable for debugging
        while True:
            if mode is Mode.remote:  # TODO: Stop TCP Connection
                continue

            if mode is Mode.push:
                lgpio.gpio_write(h,buzzer,0)  # reset the buzzer to 0

            # region of interest of depth output:
            ROI, section = depth_video_v2.get_map(disparityQueue, disparityMultiplier)

            videoIn = video.get()  # OAK-D cam
            frame = videoIn.getCvFrame()  # OAK-D cam
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray,
                                                               arucoDict,
                                                               parameters=arucoParams,
                                                               cameraMatrix=camera_matrix,
                                                               distCoeff=distortion_coeffs)
            tx, ty, tz, norm_x = np.Inf, np.Inf, np.Inf, np.Inf  # initializing position values
            rx, ry, rz = np.Inf, np.Inf, np.Inf  # initializing rotation values

            # Cases in which you do nothing
            if (len(corners) != 1  # If more than one Aruco detected (or non)
                    or (mode is Mode.push and ids != 201)  # If robot in pushing mode, but it reads follow Aruco
                    or (mode is Mode.follow and ids != 200)):  # If robot in follow mode, but it reads push Aruco
                fc = frame_counter(fc, inc=False, dec=True)
                if fc == 0:
                    speed, steer = 0, 0
                    if mode is Mode.follow:
                        lgpio.gpio_write(h,buzzer,1)  # No user, set the buzzer high to notify him

            elif len(corners) == 1:  # Exactly one Aruco detected
                fc = frame_counter(fc, inc=True, dec=False)

                # Following mode: 14cm.   Pushing mode: 5cm
                aruco_size = 0.14 if np.squeeze(ids) == 200 else 0.05
                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    aruco_size,
                    camera_matrix,
                    distortion_coeffs)

                # Draw the detected marker and its axis
                aruco.drawDetectedMarkers(frame, corners, ids)
                aruco.drawAxis(frame, camera_matrix, distortion_coeffs, rvecs, tvecs, 0.01)

                # Extract information of the position of the detected marker for interpretation
                tvecs = np.squeeze(tvecs)
                tx, ty, tz = tvecs[0] * 100, tvecs[1] * 100, tvecs[2] * 100
                maximum_x = max_x(81, tz)
                norm_x = tx / maximum_x * 10

                rvecs = np.squeeze(rvecs)
                rx, ry, rz = obtain_angles(rvecs)
                # Mode values: follow = 1, Push=2 , remote = 3
                if mode is Mode.follow and ROI > 25:  # if there is an obstacle:
                    speed = 0
                    steer = 0
                    print("Detected an obstacle!")
                    lgpio.gpio_write(h,buzzer,1)
                    
                else:                             # no obstacles:
                    lgpio.gpio_write(h,buzzer,0)  # reset the buzzer
                    speed, steer = aruco_control(mode.value, tz, max_threshold=400, forward_threshold=100,
                                                 back_threshold=50, x=norm_x, rot=rz, prev_speed=speed)

            # if the `q` key was pressed, break from the loop (enable for debugging)
            # key = show_frame(frame, tx, ty, tz, norm_x, rx, ry, rz, aruco_id=ids if len(corners) == 1 else math.inf)
            # cv2.imshow("section", section)

            # if key == ord("q"):
            #     break
            # if key == ord("p"):
            #     mode = Mode.push
            # if key == ord("f"):
            #     mode = Mode.follow

            # change the mode based on the pressed buttons
            # TODO: allow changing to remote mode using buttons (currently, needs to close the RPi to connect phone app)
            # Remote mode
            # if lgpio.gpio_read(h,pushbtn)==1 and lgpio.gpio_read(h,followbtn)==1:
            #     mode = Mode.remote
            #     lgpio.gpio_write(h, pushled, 1)
            #     lgpio.gpio_write(h, followled, 1)

            # push mode
            if lgpio.gpio_read(h,pushbtn)==1:
                mode = Mode.push
                lgpio.gpio_write(h, pushled, 1)
                lgpio.gpio_write(h, followled, 0)

            # follow mode
            if lgpio.gpio_read(h,followbtn)==1:
                mode = Mode.follow
                lgpio.gpio_write(h, pushled, 0)
                lgpio.gpio_write(h, followled, 1)

            # send and receive through TCP connection
            try:
                send(s, speed, steer)
                receive(s, debug=True)
            except socket.error as e:
                s = TCP_init()


def show_frame(frame, tx=math.inf, ty=math.inf, tz=math.inf, norm_x=math.inf, rx=math.inf, ry=math.inf, rz=math.inf,
               aruco_id=math.inf):
    # Resize the frame (This is safe, because we already did the processing)
    frame = imutils.resize(frame, 800)

    # Enable for debugging
    frame = cv2.flip(frame, 1)

    # Put text on the frame to display it
    cv2.putText(frame, "Position: x:%.2f, y:%.2f, z:%.2f, x_percentage:%3f" % (tx, ty, tz, norm_x),
                (0, 100), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
    if rz < 0: rz += 360
    cv2.putText(frame, "Rotation: x:%.2f, y:%.2f, z:%.2f" % (rx, ry, rz),
                (0, 200), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
    cv2.putText(frame, "ID: %.2f" % aruco_id,
                (0, 300), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
    # cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    return key


def obtain_angles(rvec):
    # Source: https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/aruco_pose_estimation.py
    # -- Obtain the rotation matrix tag->camera
    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
    R_tc = R_ct.T
    R_flip = np.zeros((3, 3), dtype=np.float32)
    R_flip[0, 0] = 1.0
    R_flip[1, 1] = -1.0
    R_flip[2, 2] = -1.0

    # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)

    # -- Print the marker's attitude respect to camera frame
    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (math.degrees(roll_marker), math.degrees(pitch_marker),
                                                                  math.degrees(yaw_marker))
    return math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    # refer to source https://learnopencv.com/rotation-matrix-to-euler-angles/
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles100big
# The result is the same as MATLAB except the order
# of the euler angles100big ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    # refer to source https://learnopencv.com/rotation-matrix-to-euler-angles/
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


if __name__ == '__main__':
    main()
