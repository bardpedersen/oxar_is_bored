#Nuc
rosbag record tf_static usb_cam_0/camera_info usb_cam_0/image_raw/compressed /clock_sync -o test_nuc.bag
#Left 
rosbag record tf_static camera_l1/color/camera_info camera_l2/color/camera_info camera_l1/color/image_raw/compressed camera_l2/color/image_raw/compressed camera_l1/depth/color/points camera_l2/depth/color/pints /clock_sync -o test_left.bag
#Right (2080)
rosbag record tf_static camera_r1/color/camera_info camera_r2/color/camera_info camera_r1/color/image_raw camera_r2/color/image_raw camera_r1/depth/color/points camera_r2/depth/color/pints /clock_sync -o test_right.bag