#!/bin/bash -x
#Use this script instead of launch/usb_stereo_calib2fay11.launch
#if it returns an error: Waiting for service .../set_camera_info - Service not found
ids=""
rosrun ay_vision bin/cv_usb_node config/usbcam2fay12_calib1.yaml __name:=cv_usb_node1 &
ids="$ids $!"
rosrun ay_vision bin/cv_usb_node config/usbcam2fay12_calib2.yaml __name:=cv_usb_node2 &
ids="$ids $!"
sleep 1
echo "Press enter after cameras are opened"
read s
#NOTE: USE SMALL CALIB BOARD
rosrun camera_calibration cameracalibrator.py  \
    --size 6x4 --square 0.0191 --approximate=0.1 -c ''  \
    left:=/cv_usb_node1/usbcam2fay12_l/image_raw         \
    right:=/cv_usb_node2/usbcam2fay12_r/image_raw        \
    left_camera:=/cv_usb_node1/usbcam2fay12_l            \
    right_camera:=/cv_usb_node2/usbcam2fay12_r

kill -SIGINT $ids
sleep 2
